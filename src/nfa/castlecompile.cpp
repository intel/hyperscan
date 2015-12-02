/*
 * Copyright (c) 2015, Intel Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of Intel Corporation nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \file
 * \brief Castle: multi-tenant repeat engine, compiler code.
 */
#include "castlecompile.h"

#include "castle_internal.h"
#include "nfa_internal.h"
#include "repeatcompile.h"
#include "shufticompile.h"
#include "trufflecompile.h"
#include "nfagraph/ng_dump.h"
#include "nfagraph/ng_equivalence.h"
#include "nfagraph/ng_repeat.h"
#include "nfagraph/ng_redundancy.h"
#include "nfagraph/ng_util.h"
#include "util/alloc.h"
#include "util/compile_context.h"
#include "util/container.h"
#include "util/dump_charclass.h"
#include "util/graph.h"
#include "util/make_unique.h"
#include "util/multibit_internal.h"
#include "util/ue2_containers.h"
#include "util/verify_types.h"
#include "grey.h"

#include <stack>
#include <cassert>

#include <boost/range/adaptor/map.hpp>

using namespace std;
using boost::adaptors::map_keys;
using boost::adaptors::map_values;

namespace ue2 {

#define CASTLE_MAX_TOPS 32

static
u32 depth_to_u32(const depth &d) {
    assert(d.is_reachable());
    if (d.is_infinite()) {
        return REPEAT_INF;
    }

    u32 d_val = d;
    assert(d_val < REPEAT_INF);
    return d_val;
}

static
void writeCastleScanEngine(const CharReach &cr, Castle *c) {
    if (cr.all()) {
        c->type = CASTLE_DOT;
        return;
    }

    if (cr.count() == 1) {
        c->type = CASTLE_NVERM;
        c->u.verm.c = cr.find_first();
        return;
    }

    const CharReach negated(~cr);
    if (negated.count() == 1) {
        c->type = CASTLE_VERM;
        c->u.verm.c = negated.find_first();
        return;
    }

    if (shuftiBuildMasks(negated, &c->u.shuf.mask_lo, &c->u.shuf.mask_hi) != -1) {
        c->type = CASTLE_SHUFTI;
        return;
    }

    c->type = CASTLE_TRUFFLE;
    truffleBuildMasks(negated, &c->u.truffle.mask1, &c->u.truffle.mask2);
}

static
size_t literalOverlap(const vector<CharReach> &a, const vector<CharReach> &b) {
    for (size_t i = 0; i < b.size(); i++) {
        size_t overlap_len = b.size() - i;
        if (overlap_len <= a.size()) {
            if (matches(a.end() - overlap_len, a.end(), b.begin(),
                        b.end() - i)) {
                return i;
            }
        } else {
            assert(overlap_len > a.size());
            if (matches(a.begin(), a.end(), b.end() - i - a.size(),
                        b.end() - i)) {
                return i;
            }
        }
    }

    return b.size();
}

//  UE-2666 case 1: The problem of find largest exclusive subcastles group
//  can be reformulated as finding the largest clique (subgraph where every
//  vertex is connected to every other vertex) in the graph. We use an
//  approximate algorithm here to find the maximum clique.
//  References
//  ----------
//      [1] Boppana, R., & Halldórsson, M. M. (1992).
//      Approximating maximum independent sets by excluding subgraphs.
//      BIT Numerical Mathematics, 32(2), 180–196. Springer.
//      doi:10.1007/BF01994876
//  ----------

struct CliqueVertexProps {
    CliqueVertexProps() {}
    explicit CliqueVertexProps(u32 state_in) : stateId(state_in) {}

    u32 stateId = ~0U;
    u32 parentId = ~0U;
    bool leftChild = false; /* tells us if it is the left child of its parent */

    vector<u32> clique1; /* clique for the left branch */
    vector<u32> indepSet1; /* independent set for the left branch */
    vector<u32> clique2; /* clique for the right branch */
    vector<u32> indepSet2; /* independent set for the right branch */
};

typedef boost::adjacency_list<boost::listS, boost::listS, boost::undirectedS,
                              CliqueVertexProps> CliqueGraph;
typedef CliqueGraph::vertex_descriptor CliqueVertex;

static
unique_ptr<CliqueGraph> makeCG(const vector<vector<u32>> &exclusiveSet) {
    u32 size = exclusiveSet.size();

    vector<CliqueVertex> vertices;
    unique_ptr<CliqueGraph> cg = make_unique<CliqueGraph>();
    for (u32 i = 0; i < size; ++i) {
        CliqueVertex v = add_vertex(CliqueVertexProps(i), *cg);
        vertices.push_back(v);
    }

    // construct the complement graph, then its maximum independent sets
    // are equal to the maximum clique of the original graph
    for (u32 i = 0; i < size; ++i) {
        CliqueVertex s = vertices[i];
        vector<u32> complement(size, 0);
        for (u32 j = 0; j < exclusiveSet[i].size(); ++j) {
            u32 val = exclusiveSet[i][j];
            complement[val] = 1;
        }

        for (u32 k = i + 1; k < size; ++k) {
             if (!complement[k]) {
                CliqueVertex d = vertices[k];
                add_edge(s, d, *cg);
             }
        }
    }
    return cg;
}

static
void updateCliqueInfo(CliqueGraph &cg, const CliqueVertex &n,
                      vector<u32> &clique, vector<u32> &indepSet) {
    u32 id = cg[n].stateId;
    if (cg[n].clique1.size() + 1 > cg[n].clique2.size()) {
        cg[n].clique1.push_back(id);
        clique.swap(cg[n].clique1);
    } else {
        clique.swap(cg[n].clique2);
    }

    if (cg[n].indepSet2.size() + 1 > cg[n].indepSet1.size()) {
        cg[n].indepSet2.push_back(id);
        indepSet.swap(cg[n].indepSet2);
    } else {
        indepSet.swap(cg[n].indepSet1);
    }
}

static
void getNeighborInfo(const CliqueGraph &g, vector<u32> &neighbor,
                     vector<u32> &nonneighbor, const CliqueVertex &cv,
                     const set<u32> &group) {
    u32 id = g[cv].stateId;
    ue2::unordered_set<u32> neighborId;

    // find neighbors for cv
    for (const auto &v : adjacent_vertices_range(cv, g)) {
        if (g[v].stateId != id && contains(group, g[v].stateId)) {
            neighbor.push_back(g[v].stateId);
            neighborId.insert(g[v].stateId);
        }
    }

    neighborId.insert(id);
    // find non-neighbors for cv
    for (const auto &v : vertices_range(g)) {
        if (!contains(neighborId, g[v].stateId) &&
            contains(group, g[v].stateId)) {
            nonneighbor.push_back(g[v].stateId);
        }
    }
}

static
void findCliqueGroup(CliqueGraph &cg, vector<u32> &clique,
                     vector<u32> &indepSet) {
    stack<vector<u32>> gStack;

    // create mapping between vertex and id
    map<u32, CliqueVertex> vertexMap;
    vector<u32> init;
    for (auto &v : vertices_range(cg)) {
        vertexMap[cg[v].stateId] = v;
        init.push_back(cg[v].stateId);
    }
    gStack.push(init);

    // get the vertex to start from
    set<u32> foundVertexId;
    ue2::unordered_set<u32> visitedId;
    CliqueGraph::vertex_iterator vi, ve;
    tie(vi, ve) = vertices(cg);
    CliqueVertex start = *vi;
    u32 startId = cg[start].stateId;
    DEBUG_PRINTF("startId:%u\n", startId);
    bool leftChild = false;
    u32 prevId = startId;
    while (!gStack.empty()) {
        const auto &g = gStack.top();

        // choose a vertex from the graph
        assert(!g.empty());
        u32 id = g[0];
        CliqueVertex &n = vertexMap.at(id);

        vector<u32> neighbor;
        vector<u32> nonneighbor;
        set<u32> subgraphId(g.begin(), g.end());
        getNeighborInfo(cg, neighbor, nonneighbor, n, subgraphId);
        if (contains(foundVertexId, id)) {
            prevId = id;
            // get non-neighbors for right branch
            if (visitedId.insert(id).second) {
                DEBUG_PRINTF("right branch\n");
                if (!nonneighbor.empty()) {
                    gStack.push(nonneighbor);
                    leftChild = false;
                }
            } else {
                if (id != startId) {
                    // both the left and right branches are visited,
                    // update its parent's clique and independent sets
                    u32 parentId = cg[n].parentId;
                    CliqueVertex &parent = vertexMap.at(parentId);
                    if (cg[n].leftChild) {
                        updateCliqueInfo(cg, n, cg[parent].clique1,
                                         cg[parent].indepSet1);
                    } else {
                        updateCliqueInfo(cg, n, cg[parent].clique2,
                                         cg[parent].indepSet2);
                    }
                }
                gStack.pop();
            }
        } else {
            foundVertexId.insert(id);
            cg[n].leftChild = leftChild;
            cg[n].parentId = prevId;
            cg[n].clique1.clear();
            cg[n].clique2.clear();
            cg[n].indepSet1.clear();
            cg[n].indepSet2.clear();
            // get neighbors for left branch
            if (!neighbor.empty()) {
                gStack.push(neighbor);
                leftChild = true;
            }
            prevId = id;
        }
    }
    updateCliqueInfo(cg, start, clique, indepSet);
}

template<typename Graph>
bool graph_empty(const Graph &g) {
    typename Graph::vertex_iterator vi, ve;
    tie(vi, ve) = vertices(g);
    return vi == ve;
}

static
vector<u32> removeClique(CliqueGraph &cg) {
    vector<vector<u32>> cliquesVec(1);
    vector<vector<u32>> indepSetsVec(1);
    DEBUG_PRINTF("graph size:%lu\n", num_vertices(cg));
    findCliqueGroup(cg, cliquesVec[0], indepSetsVec[0]);
    while (!graph_empty(cg)) {
        const vector<u32> &c = cliquesVec.back();
        vector<CliqueVertex> dead;
        for (const auto &v : vertices_range(cg)) {
            if (find(c.begin(), c.end(), cg[v].stateId) != c.end()) {
                dead.push_back(v);
            }
        }
        for (const auto &v : dead) {
            clear_vertex(v, cg);
            remove_vertex(v, cg);
        }
        if (graph_empty(cg)) {
            break;
        }
        vector<u32> clique;
        vector<u32> indepSet;
        findCliqueGroup(cg, clique, indepSet);
        cliquesVec.push_back(clique);
        indepSetsVec.push_back(indepSet);
    }

    // get the independent set with max size
    size_t max = 0;
    size_t id = 0;
    for (size_t j = 0; j < indepSetsVec.size(); ++j) {
        if (indepSetsVec[j].size() > max) {
            max = indepSetsVec[j].size();
            id = j;
        }
    }

    DEBUG_PRINTF("clique size:%lu\n", indepSetsVec[id].size());
    return indepSetsVec[id];
}

static
vector<u32> findMaxClique(const vector<vector<u32>> &exclusiveSet) {
    auto cg = makeCG(exclusiveSet);
    return removeClique(*cg);
}

// if the location of any reset character in one literal are after
// the end locations where it overlaps with other literals,
// then the literals are mutual exclusive
static
bool findExclusivePair(const u32 id1, const u32 id2,
                       const vector<vector<size_t>> &min_reset_dist,
                       const vector<vector<vector<CharReach>>> &triggers) {
    const auto &triggers1 = triggers[id1];
    const auto &triggers2 = triggers[id2];
    for (u32 i = 0; i < triggers1.size(); ++i) {
        for (u32 j = 0; j < triggers2.size(); ++j) {
            size_t max_overlap1 = literalOverlap(triggers1[i], triggers2[j]);
            size_t max_overlap2 = literalOverlap(triggers2[j], triggers1[i]);
            if (max_overlap1 <= min_reset_dist[id2][j] ||
                max_overlap2 <= min_reset_dist[id1][i]) {
                return false;
            }
        }
    }
    return true;
}

static
vector<u32> checkExclusion(const CharReach &cr,
                           const vector<vector<vector<CharReach>>> &triggers) {
    vector<u32> group;
    if (!triggers.size() || triggers.size() == 1) {
        return group;
    }

    vector<vector<size_t> > min_reset_dist;
    // get min reset distance for each repeat
    for (auto it = triggers.begin(); it != triggers.end(); it++) {
        const vector<size_t> &tmp_dist = minResetDistToEnd(*it, cr);
        min_reset_dist.push_back(tmp_dist);
    }

    vector<vector<u32>> exclusiveSet;
    // find exclusive pair for each repeat
    for (u32 i = 0; i < triggers.size(); ++i) {
        vector<u32> repeatIds;
        for (u32 j = i + 1; j < triggers.size(); ++j) {
            if (findExclusivePair(i, j, min_reset_dist, triggers)) {
                repeatIds.push_back(j);
            }
        }
        exclusiveSet.push_back(repeatIds);
        DEBUG_PRINTF("Exclusive pair size:%lu\n", repeatIds.size());
    }

    // find the largest exclusive group
    return findMaxClique(exclusiveSet);
}

static
void buildSubcastles(const CastleProto &proto, vector<SubCastle> &subs,
                     vector<RepeatInfo> &infos, vector<u64a> &patchSize,
                     const vector<pair<depth, bool>> &repeatInfoPair,
                     u32 &scratchStateSize, u32 &streamStateSize,
                     u32 &tableSize, vector<u64a> &tables, u32 &sparseRepeats,
                     const set<u32> &exclusiveGroup) {
    u32 i = 0;
    u32 maxStreamSize = 0;
    bool exclusive = exclusiveGroup.size() > 1;
    for (auto it = proto.repeats.begin(), ite = proto.repeats.end();
         it != ite; ++it, ++i) {
        const PureRepeat &pr = it->second;
        depth min_period = repeatInfoPair[i].first;
        bool is_reset = repeatInfoPair[i].second;

        enum RepeatType rtype = chooseRepeatType(pr.bounds.min, pr.bounds.max,
                                                 min_period, is_reset);
        RepeatStateInfo rsi(rtype, pr.bounds.min, pr.bounds.max, min_period);

        DEBUG_PRINTF("sub %u: selected %s model for %s repeat\n", i,
                     repeatTypeName(rtype), pr.bounds.str().c_str());

        u32 subScratchStateSize;
        u32 subStreamStateSize;

        SubCastle &sub = subs[i];
        RepeatInfo &info = infos[i];

        // handle exclusive case differently
        if (exclusive && exclusiveGroup.find(i) != exclusiveGroup.end()) {
            maxStreamSize = MAX(maxStreamSize, rsi.packedCtrlSize);
        } else {
            subScratchStateSize = verify_u32(sizeof(RepeatControl));
            subStreamStateSize = verify_u32(rsi.packedCtrlSize + rsi.stateSize);

            info.packedCtrlSize = rsi.packedCtrlSize;
            sub.fullStateOffset = scratchStateSize;
            sub.streamStateOffset = streamStateSize;

            scratchStateSize += subScratchStateSize;
            streamStateSize += subStreamStateSize;
        }

        info.type = verify_u8(rtype);
        info.repeatMin = depth_to_u32(pr.bounds.min);
        info.repeatMax = depth_to_u32(pr.bounds.max);
        info.stateSize = rsi.stateSize;
        info.horizon = rsi.horizon;
        info.minPeriod = min_period.is_finite() ? (u32)min_period : ~0U;
        assert(rsi.packedFieldSizes.size()
               <= ARRAY_LENGTH(info.packedFieldSizes));
        copy(rsi.packedFieldSizes.begin(), rsi.packedFieldSizes.end(),
             info.packedFieldSizes);
        info.patchCount = rsi.patchCount;
        info.patchSize = rsi.patchSize;
        info.encodingSize = rsi.encodingSize;
        info.patchesOffset = rsi.patchesOffset;

        sub.report = *pr.reports.begin();

        if (rtype == REPEAT_SPARSE_OPTIMAL_P) {
           for (u32 j = 0; j < rsi.patchSize; j++) {
               tables.push_back(rsi.table[j]);
           }
           sparseRepeats++;
           patchSize[i] = rsi.patchSize;
           tableSize += rsi.patchSize;
        }
    }

    if (exclusive) {
        for (auto k : exclusiveGroup) {
            SubCastle &sub = subs[k];
            RepeatInfo &info = infos[k];
            info.packedCtrlSize = maxStreamSize;
            sub.fullStateOffset = scratchStateSize;
            sub.streamStateOffset = streamStateSize;
        }
        scratchStateSize += verify_u32(sizeof(RepeatControl));
        streamStateSize += maxStreamSize;
    }
}

aligned_unique_ptr<NFA>
buildCastle(const CastleProto &proto,
            const map<u32, vector<vector<CharReach>>> &triggers,
            const CompileContext &cc) {
    assert(cc.grey.allowCastle);

    const size_t numRepeats = proto.repeats.size();
    assert(numRepeats > 0 && numRepeats <= proto.max_occupancy);

    const CharReach &cr = proto.reach();

    DEBUG_PRINTF("reach %s, %zu repeats\n", describeClass(cr).c_str(),
                 numRepeats);

    vector<SubCastle> subs(numRepeats);
    memset(&subs[0], 0, sizeof(SubCastle) * numRepeats);

    vector<RepeatInfo> infos(numRepeats);
    memset(&infos[0], 0, sizeof(RepeatInfo) * numRepeats);

    vector<u64a> patchSize(numRepeats);
    memset(&patchSize[0], 0, sizeof(u64a) * numRepeats);

    vector<u64a> tables;

    // We start with enough stream state to store the active bitfield.
    u32 streamStateSize = mmbit_size(numRepeats);

    // We have a copy of the stream state in scratch for castleMatchLoop.
    u32 scratchStateSize = ROUNDUP_N(streamStateSize, alignof(RepeatControl));

    depth minWidth(depth::infinity());
    depth maxWidth(0);

    u32 i = 0;
    vector<u32> candidateRepeats;
    vector<vector<vector<CharReach>>> candidateTriggers;
    vector<pair<depth, bool>> repeatInfoPair;
    for (auto it = proto.repeats.begin(), ite = proto.repeats.end();
         it != ite; ++it, ++i) {
        const u32 top = it->first;
        const PureRepeat &pr = it->second;
        assert(pr.reach == cr);
        assert(pr.reports.size() == 1);

        if (top != i) {
            // Tops have not been remapped?
            assert(0);
            throw std::logic_error("Tops not remapped");
        }

        minWidth = min(minWidth, pr.bounds.min);
        maxWidth = max(maxWidth, pr.bounds.max);

        bool is_reset = false;
        depth min_period = depth::infinity();

        // If we've got a top in the castle without any trigger information, it
        // possibly means that we've got a repeat that we can't trigger. We do
        // need to cope with it though.
        if (contains(triggers, top)) {
            min_period = minPeriod(triggers.at(top), cr, &is_reset);
        }

        if (min_period > pr.bounds.max) {
            DEBUG_PRINTF("trigger is longer than repeat; only need one offset\n");
            is_reset = true;
        }

        repeatInfoPair.push_back(make_pair(min_period, is_reset));

        if (is_reset) {
            candidateTriggers.push_back(triggers.at(top));
            candidateRepeats.push_back(i);
        }
    }

    // Case 1: exclusive repeats
    bool exclusive = false;
    bool pureExclusive = false;
    u8 activeIdxSize = 0;
    set<u32> exclusiveGroup;
    if (cc.grey.castleExclusive) {
        vector<u32> tmpGroup = checkExclusion(cr, candidateTriggers);
        const u32 exclusiveSize = tmpGroup.size();
        if (exclusiveSize > 1) {
            // Case 1: mutual exclusive repeats group found, initialize state
            // sizes
            exclusive = true;
            activeIdxSize = calcPackedBytes(exclusiveSize);
            if (exclusiveSize == numRepeats) {
                pureExclusive = true;
                streamStateSize = 0;
                scratchStateSize = 0;
            }
            streamStateSize += activeIdxSize;

            // replace with top values
            for (const auto &val : tmpGroup) {
                exclusiveGroup.insert(candidateRepeats[val]);
            }
        }
    }

    DEBUG_PRINTF("reach %s exclusive %u\n", describeClass(cr).c_str(),
                 exclusive);

    u32 tableSize = 0;
    u32 sparseRepeats = 0;
    buildSubcastles(proto, subs, infos, patchSize, repeatInfoPair,
                    scratchStateSize, streamStateSize, tableSize,
                    tables, sparseRepeats, exclusiveGroup);

    const size_t total_size =
        sizeof(NFA) +                      // initial NFA structure
        sizeof(Castle) +                   // Castle structure
        sizeof(SubCastle) * subs.size() +  // SubCastles themselves
        sizeof(RepeatInfo) * subs.size() + // RepeatInfo structure
        sizeof(u64a) * tableSize +         // table size for
                                           // REPEAT_SPARSE_OPTIMAL_P
        sizeof(u64a) * sparseRepeats;      // paddings for
                                           // REPEAT_SPARSE_OPTIMAL_P tables

    aligned_unique_ptr<NFA> nfa = aligned_zmalloc_unique<NFA>(total_size);
    nfa->type = verify_u8(CASTLE_NFA_0);
    nfa->length = verify_u32(total_size);
    nfa->nPositions = verify_u32(subs.size());
    nfa->streamStateSize = streamStateSize;
    nfa->scratchStateSize = scratchStateSize;
    nfa->minWidth = verify_u32(minWidth);
    nfa->maxWidth = maxWidth.is_finite() ? verify_u32(maxWidth) : 0;

    char *ptr = (char *)nfa.get() + sizeof(NFA);
    Castle *c = (Castle *)ptr;
    c->numRepeats = verify_u32(subs.size());
    c->exclusive = exclusive;
    c->pureExclusive = pureExclusive;
    c->activeIdxSize = activeIdxSize;

    writeCastleScanEngine(cr, c);

    ptr += sizeof(Castle);
    SubCastle *subCastles = ((SubCastle *)(ROUNDUP_PTR(ptr, alignof(u32))));
    copy(subs.begin(), subs.end(), subCastles);

    u32 length = 0;
    u32 tableIdx = 0;
    for (i = 0; i < numRepeats; i++) {
        u32 offset = sizeof(SubCastle) * (numRepeats - i) + length;
        SubCastle *sub = &subCastles[i];
        sub->repeatInfoOffset = offset;

        ptr = (char *)sub + offset;
        memcpy(ptr, &infos[i], sizeof(RepeatInfo));

        if (patchSize[i]) {
            RepeatInfo *info = (RepeatInfo *)ptr;
            u64a *table = ((u64a *)(ROUNDUP_PTR(((char *)(info) +
                                    sizeof(*info)), alignof(u64a))));
            copy(tables.begin() + tableIdx,
                 tables.begin() + tableIdx + patchSize[i], table);
            u32 diff = (char *)table - (char *)info +
                       sizeof(u64a) * patchSize[i];
            info->length = diff;
            length += diff;
            tableIdx += patchSize[i];
        } else {
            length += sizeof(RepeatInfo);
        }

        // set exclusive group info
        if (exclusiveGroup.find(i) != exclusiveGroup.end()) {
            sub->exclusive = 1;
        } else {
            sub->exclusive = 0;
        }
    }
    return nfa;
}

set<ReportID> all_reports(const CastleProto &proto) {
    set<ReportID> reports;
    for (const ReportID &report : proto.report_map | map_keys) {
        reports.insert(report);
    }
    return reports;
}

depth findMinWidth(const CastleProto &proto) {
    depth min_width(depth::infinity());
    for (const PureRepeat &pr : proto.repeats | map_values) {
        min_width = min(min_width, pr.bounds.min);
    }
    return min_width;
}

depth findMaxWidth(const CastleProto &proto) {
    depth max_width(0);
    for (const PureRepeat &pr : proto.repeats | map_values) {
        max_width = max(max_width, pr.bounds.max);
    }
    return max_width;
}

depth findMinWidth(const CastleProto &proto, u32 top) {
    if (!contains(proto.repeats, top)) {
        assert(0); // should not happen
        return depth::infinity();
    }
    return proto.repeats.at(top).bounds.min;
}

depth findMaxWidth(const CastleProto &proto, u32 top) {
    if (!contains(proto.repeats, top)) {
        assert(0); // should not happen
        return depth(0);
    }
    return proto.repeats.at(top).bounds.max;
}

CastleProto::CastleProto(const PureRepeat &pr) {
    assert(pr.reach.any());
    assert(pr.reports.size() == 1);
    u32 top = 0;
    repeats.emplace(top, pr);
    for (const auto &report : pr.reports) {
        report_map[report].insert(top);
    }
}

const CharReach &CastleProto::reach() const {
    assert(!repeats.empty());
    return repeats.begin()->second.reach;
}

static
u32 find_next_top(const map<u32, PureRepeat> &repeats) {
    u32 top = verify_u32(repeats.size());
    assert(!contains(repeats, top));
    return top;
}

u32 CastleProto::add(const PureRepeat &pr) {
    assert(repeats.size() < max_occupancy);
    assert(pr.reach == reach());
    assert(pr.reports.size() == 1);
    u32 top = find_next_top(repeats);
    DEBUG_PRINTF("selected unused top %u\n", top);
    repeats.emplace(top, pr);
    for (const auto &report : pr.reports) {
        report_map[report].insert(top);
    }
    return top;
}

u32 CastleProto::merge(const PureRepeat &pr) {
    assert(repeats.size() <= max_occupancy);
    assert(pr.reach == reach());
    assert(pr.reports.size() == 1);

    // First, see if this repeat is already in this castle.
    for (const auto &m : repeats) {
        if (m.second == pr) {
            DEBUG_PRINTF("repeat already present, with top %u\n", m.first);
            return m.first;
        }
    }

    if (repeats.size() == max_occupancy) {
        DEBUG_PRINTF("this castle is full\n");
        return max_occupancy;
    }

    return add(pr);
}

bool mergeCastle(CastleProto &c1, const CastleProto &c2,
                 map<u32, u32> &top_map) {
    assert(&c1 != &c2);

    DEBUG_PRINTF("c1 has %zu repeats, c2 has %zu repeats\n", c1.repeats.size(),
                 c2.repeats.size());

    if (c1.reach() != c2.reach()) {
        DEBUG_PRINTF("different reach!\n");
        return false;
    }

    if (c1.repeats.size() + c2.repeats.size() > c1.max_occupancy) {
        DEBUG_PRINTF("too many repeats to merge\n");
        return false;
    }

    top_map.clear();

    for (const auto &m : c2.repeats) {
        const u32 top = m.first;
        const PureRepeat &pr = m.second;
        DEBUG_PRINTF("top %u\n", top);
        u32 new_top = c1.add(pr);
        top_map[top] = new_top;
        DEBUG_PRINTF("adding repeat: map %u->%u\n", top, new_top);
    }

    assert(c1.repeats.size() <= c1.max_occupancy);
    return true;
}

void remapCastleTops(CastleProto &proto, map<u32, u32> &top_map) {
    map<u32, PureRepeat> out;
    top_map.clear();

    for (const auto &m : proto.repeats) {
        const u32 top = m.first;
        const PureRepeat &pr = m.second;
        u32 new_top = find_next_top(out);
        out.emplace(new_top, pr);
        top_map[top] = new_top;
    }

    proto.repeats.swap(out);

    // Remap report map.
    proto.report_map.clear();
    for (const auto &m : proto.repeats) {
        const u32 top = m.first;
        const PureRepeat &pr = m.second;
        for (const auto &report : pr.reports) {
            proto.report_map[report].insert(top);
        }
    }

    assert(proto.repeats.size() <= proto.max_occupancy);
}

namespace {
struct HasReport {
    explicit HasReport(ReportID r) : report(r) {}

    bool operator()(const pair<u32, PureRepeat> &a) const {
        return contains(a.second.reports, report);
    }

private:
    ReportID report;
};
}

bool is_equal(const CastleProto &c1, ReportID report1, const CastleProto &c2,
              ReportID report2) {
    assert(!c1.repeats.empty());
    assert(!c2.repeats.empty());

    if (c1.reach() != c2.reach()) {
        DEBUG_PRINTF("different reach\n");
        return false;
    }

    map<u32, PureRepeat>::const_iterator it = c1.repeats.begin(),
                                         ite = c1.repeats.end(),
                                         jt = c2.repeats.begin(),
                                         jte = c2.repeats.end();

    for (;; ++it, ++jt) {
        it = find_if(it, ite, HasReport(report1));
        jt = find_if(jt, jte, HasReport(report2));

        if (it == ite && jt == jte) {
            DEBUG_PRINTF("success, cases are equivalent!\n");
            return true;
        }

        if (it == ite || jt == jte) {
            DEBUG_PRINTF("no match for one repeat\n");
            break;
        }

        if (it->first != jt->first) {
            DEBUG_PRINTF("different tops\n");
            break;
        }

        const PureRepeat &r1 = it->second;
        const PureRepeat &r2 = jt->second;
        assert(r1.reach == c1.reach());
        assert(r2.reach == c1.reach());
        if (r1.bounds != r2.bounds) {
            DEBUG_PRINTF("different bounds\n");
            break;
        }
    }

    return false;
}

bool is_equal(const CastleProto &c1, const CastleProto &c2) {
    assert(!c1.repeats.empty());
    assert(!c2.repeats.empty());

    if (c1.reach() != c2.reach()) {
        DEBUG_PRINTF("different reach\n");
        return false;
    }

    return c1.repeats == c2.repeats;
}

bool requiresDedupe(const CastleProto &proto,
                    const ue2::flat_set<ReportID> &reports) {
    for (const auto &report : reports) {
        auto it = proto.report_map.find(report);
        if (it == end(proto.report_map)) {
            continue;
        }
        if (it->second.size() > 1) {
            DEBUG_PRINTF("castle proto %p has dupe report %u\n", &proto,
                         report);
            return true;
        }
    }
    return false;
}

static
void addToHolder(NGHolder &g, u32 top, const PureRepeat &pr) {
    DEBUG_PRINTF("top %u -> repeat %s\n", top, pr.bounds.str().c_str());
    NFAVertex u = g.start;

    // Mandatory repeats to min bound.
    u32 min_bound = pr.bounds.min; // always finite
    if (min_bound == 0) { // Vacuous case, we can only do this once.
        assert(!edge(g.start, g.accept, g).second);
        NFAEdge e = add_edge(g.start, g.accept, g).first;
        g[e].top = top;
        g[u].reports.insert(pr.reports.begin(), pr.reports.end());
        min_bound = 1;
    }

    for (u32 i = 0; i < min_bound; i++) {
        NFAVertex v = add_vertex(g);
        g[v].char_reach = pr.reach;
        NFAEdge e = add_edge(u, v, g).first;
        if (u == g.start) {
            g[e].top = top;
        }
        u = v;
    }

    NFAVertex head = u;

    // Optional repeats to max bound.
    if (pr.bounds.max.is_finite()) {
        assert(pr.bounds.max > depth(0));
        const u32 max_bound = pr.bounds.max;
        for (u32 i = 0; i < max_bound - min_bound; i++) {
            NFAVertex v = add_vertex(g);
            g[v].char_reach = pr.reach;
            if (head != u) {
                add_edge(head, v, g);
            }
            NFAEdge e = add_edge(u, v, g).first;
            if (u == g.start) {
                g[e].top = top;
            }
            u = v;
        }
    } else {
        assert(pr.bounds.max.is_infinite());
        add_edge(u, u, g);
    }

    // Connect to accept.
    add_edge(u, g.accept, g);
    g[u].reports.insert(pr.reports.begin(), pr.reports.end());
    if (u != head) {
        add_edge(head, g.accept, g);
        g[head].reports.insert(pr.reports.begin(), pr.reports.end());
    }
}

static
bool hasZeroMinBound(const CastleProto &proto) {
    const depth zero(0);
    for (const PureRepeat &pr : proto.repeats | map_values) {
        if (pr.bounds.min == zero) {
            return true;
        }
    }
    return false;
}

unique_ptr<NGHolder> makeHolder(const CastleProto &proto, nfa_kind kind,
                                const CompileContext &cc) {
    assert(!proto.repeats.empty());

    // Vacuous edges are only doable in the NGHolder if we are a single-top
    // Castle.
    if (hasZeroMinBound(proto)) {
        if (proto.repeats.size() != 1 || proto.repeats.begin()->first != 0) {
            DEBUG_PRINTF("can't build multi-top vacuous holder\n");
            return nullptr;
        }
    }

    unique_ptr<NGHolder> g = ue2::make_unique<NGHolder>(kind);

    for (const auto &m : proto.repeats) {
        if (m.first >= CASTLE_MAX_TOPS) {
            DEBUG_PRINTF("top %u too big for an NFA\n", m.first);
            return nullptr;
        }

        addToHolder(*g, m.first, m.second);
    }

    //dumpGraph("castle_holder.dot", g->g);

    // Sanity checks.
    assert(allMatchStatesHaveReports(*g));
    assert(!has_parallel_edge(*g));

    reduceGraphEquivalences(*g, cc);

    removeRedundancy(*g, SOM_NONE);

    return g;
}

} // namespace ue2
