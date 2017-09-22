/*
 * Copyright (c) 2015-2017, Intel Corporation
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

/**
 * \file
 * \brief Castle: multi-tenant repeat engine, compiler code.
 */

#include "castlecompile.h"

#include "castle_internal.h"
#include "limex_limits.h"
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
#include "util/flat_containers.h"
#include "util/graph.h"
#include "util/make_unique.h"
#include "util/multibit_build.h"
#include "util/report_manager.h"
#include "util/verify_types.h"
#include "grey.h"

#include <stack>
#include <cassert>

#include <boost/graph/adjacency_list.hpp>
#include <boost/range/adaptor/map.hpp>

using namespace std;
using boost::adaptors::map_keys;
using boost::adaptors::map_values;

namespace ue2 {

#define CLIQUE_GRAPH_MAX_SIZE 1000

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

    if (shuftiBuildMasks(negated, (u8 *)&c->u.shuf.mask_lo,
                         (u8 *)&c->u.shuf.mask_hi) != -1) {
        c->type = CASTLE_SHUFTI;
        return;
    }

    c->type = CASTLE_TRUFFLE;
    truffleBuildMasks(negated, (u8 *)(u8 *)&c->u.truffle.mask1,
                      (u8 *)&c->u.truffle.mask2);
}

static
bool literalOverlap(const vector<CharReach> &a, const vector<CharReach> &b,
                    const size_t dist) {
    for (size_t i = 0; i < b.size(); i++) {
        if (i > dist) {
            return true;
        }
        size_t overlap_len = b.size() - i;
        if (overlap_len <= a.size()) {
            if (matches(a.end() - overlap_len, a.end(), b.begin(),
                        b.end() - i)) {
                return false;
            }
        } else {
            assert(overlap_len > a.size());
            if (matches(a.begin(), a.end(), b.end() - i - a.size(),
                        b.end() - i)) {
                return false;
            }
        }
    }

    return b.size() > dist;
}

struct CliqueVertexProps {
    CliqueVertexProps() {}
    explicit CliqueVertexProps(u32 state_in) : stateId(state_in) {}

    u32 stateId = ~0U;
};

typedef boost::adjacency_list<boost::listS, boost::listS, boost::undirectedS,
                              CliqueVertexProps> CliqueGraph;
typedef CliqueGraph::vertex_descriptor CliqueVertex;

static
void getNeighborInfo(const CliqueGraph &g, vector<u32> &neighbor,
                     const CliqueVertex &cv, const set<u32> &group) {
    u32 id = g[cv].stateId;

    // find neighbors for cv
    for (const auto &v : adjacent_vertices_range(cv, g)) {
        if (g[v].stateId != id && contains(group, g[v].stateId)) {
            neighbor.push_back(g[v].stateId);
            DEBUG_PRINTF("Neighbor:%u\n", g[v].stateId);
        }
    }
}

static
void findCliqueGroup(CliqueGraph &cg, vector<u32> &clique) {
    stack<vector<u32>> gStack;

    // Create mapping between vertex and id
    map<u32, CliqueVertex> vertexMap;
    vector<u32> init;
    for (const auto &v : vertices_range(cg)) {
        vertexMap[cg[v].stateId] = v;
        init.push_back(cg[v].stateId);
    }
    gStack.push(init);

    // Get the vertex to start from
    CliqueGraph::vertex_iterator vi, ve;
    tie(vi, ve) = vertices(cg);
    while (!gStack.empty()) {
        vector<u32> g = gStack.top();
        gStack.pop();

        // Choose a vertex from the graph
        u32 id = g[0];
        const CliqueVertex &n = vertexMap.at(id);
        clique.push_back(id);
        // Corresponding vertex in the original graph
        vector<u32> neighbor;
        set<u32> subgraphId(g.begin(), g.end());
        getNeighborInfo(cg, neighbor, n, subgraphId);
        // Get graph consisting of neighbors for left branch
        if (!neighbor.empty()) {
            gStack.push(neighbor);
        }
    }
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
    DEBUG_PRINTF("graph size:%zu\n", num_vertices(cg));
    findCliqueGroup(cg, cliquesVec[0]);
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
        findCliqueGroup(cg, clique);
        cliquesVec.push_back(clique);
    }

    // get the independent set with max size
    size_t max = 0;
    size_t id = 0;
    for (size_t j = 0; j < cliquesVec.size(); ++j) {
        if (cliquesVec[j].size() > max) {
            max = cliquesVec[j].size();
            id = j;
        }
    }

    DEBUG_PRINTF("clique size:%zu\n", cliquesVec[id].size());
    return cliquesVec[id];
}

// if the location of any reset character in one literal are after
// the end locations where it overlaps with other literals,
// then the literals are mutual exclusive
static
bool findExclusivePair(const size_t id1, const size_t id2,
                       const size_t lower,
                       const vector<vector<size_t>> &min_reset_dist,
                       const vector<vector<vector<CharReach>>> &triggers) {
    const auto &triggers1 = triggers[id1];
    const auto &triggers2 = triggers[id2];
    for (size_t i = 0; i < triggers1.size(); ++i) {
        for (size_t j = 0; j < triggers2.size(); ++j) {
            if (!literalOverlap(triggers1[i], triggers2[j],
                                min_reset_dist[id2 - lower][j]) ||
                !literalOverlap(triggers2[j], triggers1[i],
                                min_reset_dist[id1 - lower][i])) {
                return false;
            }
        }
    }
    return true;
}

static
vector<vector<u32>> checkExclusion(u32 &streamStateSize,
                       const CharReach &cr,
                       const vector<vector<vector<CharReach>>> &triggers,
                       enum ExclusiveType &exclusive,
                       const size_t numRepeats) {
    vector<vector<u32>> groups;
    size_t trigSize = triggers.size();
    DEBUG_PRINTF("trigSize %zu\n", trigSize);

    size_t lower = 0;
    size_t total = 0;
    while (lower < trigSize) {
        vector<CliqueVertex> vertices;
        unique_ptr<CliqueGraph> cg = make_unique<CliqueGraph>();

        vector<vector<size_t>> min_reset_dist;
        size_t upper = min(lower + CLIQUE_GRAPH_MAX_SIZE, trigSize);
        // get min reset distance for each repeat
        for (size_t i = lower; i < upper; i++) {
            CliqueVertex v = add_vertex(CliqueVertexProps(i), *cg);
            vertices.push_back(v);

            const vector<size_t> &tmp_dist =
                minResetDistToEnd(triggers[i], cr);
            min_reset_dist.push_back(tmp_dist);
        }

        // find exclusive pair for each repeat
        for (size_t i = lower; i < upper; i++) {
            CliqueVertex s = vertices[i - lower];
            for (size_t j = i + 1; j < upper; j++) {
                if (findExclusivePair(i, j, lower, min_reset_dist,
                                      triggers)) {
                    CliqueVertex d = vertices[j - lower];
                    add_edge(s, d, *cg);
                }
            }
        }

        // find the largest exclusive group
        auto clique = removeClique(*cg);
        size_t cliqueSize = clique.size();
        if (cliqueSize > 1) {
            groups.push_back(clique);
            exclusive = EXCLUSIVE;
            total += cliqueSize;
        }

        lower += CLIQUE_GRAPH_MAX_SIZE;
    }
    DEBUG_PRINTF("clique size %zu, num of repeats %zu\n",
                 total, numRepeats);
    if (total == numRepeats) {
        exclusive = PURE_EXCLUSIVE;
        streamStateSize = 0;
    };

    return groups;
}

namespace {
struct ExclusiveInfo {

    /** Mapping between top and exclusive group id */
    map<u32, u32> groupId;

    /** Number of exclusive groups */
    u32 numGroups = 0;
};
}

static
void buildSubcastles(const CastleProto &proto, vector<SubCastle> &subs,
                     vector<RepeatInfo> &infos, vector<u64a> &patchSize,
                     const vector<pair<depth, bool>> &repeatInfoPair,
                     u32 &scratchStateSize, u32 &streamStateSize,
                     u32 &tableSize, vector<u64a> &tables, u32 &sparseRepeats,
                     const ExclusiveInfo &exclusiveInfo,
                     vector<u32> &may_stale, const ReportManager &rm) {
    const bool remap_reports = has_managed_reports(proto.kind);

    u32 i = 0;
    const auto &groupId = exclusiveInfo.groupId;
    const auto &numGroups = exclusiveInfo.numGroups;
    vector<u32> maxStreamSize(numGroups, 0);

    for (auto it = proto.repeats.begin(), ite = proto.repeats.end();
         it != ite; ++it, ++i) {
        const PureRepeat &pr = it->second;
        depth min_period = repeatInfoPair[i].first;
        bool is_reset = repeatInfoPair[i].second;

        enum RepeatType rtype = chooseRepeatType(pr.bounds.min, pr.bounds.max,
                                                 min_period, is_reset, true);
        RepeatStateInfo rsi(rtype, pr.bounds.min, pr.bounds.max, min_period);

        DEBUG_PRINTF("sub %u: selected %s model for %s repeat\n", i,
                     repeatTypeName(rtype), pr.bounds.str().c_str());

        SubCastle &sub = subs[i];
        RepeatInfo &info = infos[i];

        info.packedCtrlSize = rsi.packedCtrlSize;
        u32 subStreamStateSize = verify_u32(rsi.packedCtrlSize + rsi.stateSize);

        // Handle stream/scratch space alloc for exclusive case differently.
        if (contains(groupId, i)) {
            u32 id = groupId.at(i);
            maxStreamSize[id] = max(maxStreamSize[id], subStreamStateSize);
            // SubCastle full/stream state offsets are written in for the group
            // below.
        } else {
            sub.fullStateOffset = scratchStateSize;
            sub.streamStateOffset = streamStateSize;
            scratchStateSize += verify_u32(sizeof(RepeatControl));
            streamStateSize += subStreamStateSize;
        }

        if (pr.bounds.max.is_finite()) {
            may_stale.push_back(i);
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

        assert(pr.reports.size() == 1);
        ReportID id = *pr.reports.begin();
        sub.report = remap_reports ? rm.getProgramOffset(id) : id;

        if (rtype == REPEAT_SPARSE_OPTIMAL_P) {
            for (u32 j = 0; j < rsi.patchSize; j++) {
                tables.push_back(rsi.table[j]);
            }
            sparseRepeats++;
            patchSize[i] = rsi.patchSize;
            tableSize += rsi.patchSize;
        }
    }

    vector<u32> scratchOffset(numGroups, 0);
    vector<u32> streamOffset(numGroups, 0);
    for (const auto &j : groupId) {
        u32 top = j.first;
        u32 id = j.second;
        SubCastle &sub = subs[top];
        if (!scratchOffset[id]) {
            sub.fullStateOffset = scratchStateSize;
            sub.streamStateOffset = streamStateSize;
            scratchOffset[id] = scratchStateSize;
            streamOffset[id] = streamStateSize;
            scratchStateSize += verify_u32(sizeof(RepeatControl));
            streamStateSize += maxStreamSize[id];
        } else {
            sub.fullStateOffset = scratchOffset[id];
            sub.streamStateOffset = streamOffset[id];
        }
    }
}

bytecode_ptr<NFA>
buildCastle(const CastleProto &proto,
            const map<u32, vector<vector<CharReach>>> &triggers,
            const CompileContext &cc, const ReportManager &rm) {
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
    ExclusiveInfo exclusiveInfo;
    vector<vector<vector<CharReach>>> candidateTriggers;
    vector<u32> candidateRepeats;
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
            min_period = depth(minPeriod(triggers.at(top), cr, &is_reset));
        }

        if (min_period > pr.bounds.max) {
            DEBUG_PRINTF("trigger is longer than repeat; only need one offset\n");
            is_reset = true;
        }

        repeatInfoPair.push_back(make_pair(min_period, is_reset));

        candidateTriggers.push_back(triggers.at(top));
        candidateRepeats.push_back(i);
    }

    // Case 1: exclusive repeats
    enum ExclusiveType exclusive = NOT_EXCLUSIVE;
    u32 activeIdxSize = 0;
    u32 groupIterOffset = 0;
    if (cc.grey.castleExclusive) {
        auto cliqueGroups =
            checkExclusion(streamStateSize, cr, candidateTriggers,
                           exclusive, numRepeats);
        for (const auto &group : cliqueGroups) {
            // mutual exclusive repeats group found,
            // update state sizes
            activeIdxSize = calcPackedBytes(numRepeats + 1);
            streamStateSize += activeIdxSize;

            // replace with top values
            for (const auto &val : group) {
                const u32 top = candidateRepeats[val];
                exclusiveInfo.groupId[top] = exclusiveInfo.numGroups;
            }
            exclusiveInfo.numGroups++;
        }

        if (exclusive) {
            groupIterOffset = streamStateSize;
            streamStateSize += mmbit_size(exclusiveInfo.numGroups);
        }

        DEBUG_PRINTF("num of groups:%u\n", exclusiveInfo.numGroups);
    }
    candidateRepeats.clear();

    DEBUG_PRINTF("reach %s exclusive %u\n", describeClass(cr).c_str(),
                 exclusive);

    u32 tableSize = 0;
    u32 sparseRepeats = 0;
    vector<u32> may_stale; /* sub castles that may go stale */

    buildSubcastles(proto, subs, infos, patchSize, repeatInfoPair,
                    scratchStateSize, streamStateSize, tableSize,
                    tables, sparseRepeats, exclusiveInfo, may_stale, rm);

    DEBUG_PRINTF("%zu subcastles may go stale\n", may_stale.size());
    vector<mmbit_sparse_iter> stale_iter;
    if (!may_stale.empty()) {
        stale_iter = mmbBuildSparseIterator(may_stale, numRepeats);
    }


    size_t total_size =
        sizeof(NFA) +                      // initial NFA structure
        sizeof(Castle) +                   // Castle structure
        sizeof(SubCastle) * subs.size() +  // SubCastles themselves
        sizeof(RepeatInfo) * subs.size() + // RepeatInfo structure
        sizeof(u64a) * tableSize +         // table size for
                                           // REPEAT_SPARSE_OPTIMAL_P
        sizeof(u64a) * sparseRepeats;      // paddings for
                                           // REPEAT_SPARSE_OPTIMAL_P tables

    total_size = ROUNDUP_N(total_size, alignof(mmbit_sparse_iter));
    total_size += byte_length(stale_iter); // stale sparse iter

    auto nfa = make_zeroed_bytecode_ptr<NFA>(total_size);
    nfa->type = verify_u8(CASTLE_NFA);
    nfa->length = verify_u32(total_size);
    nfa->nPositions = verify_u32(subs.size());
    nfa->streamStateSize = streamStateSize;
    nfa->scratchStateSize = scratchStateSize;
    nfa->minWidth = verify_u32(minWidth);
    nfa->maxWidth = maxWidth.is_finite() ? verify_u32(maxWidth) : 0;

    char * const base_ptr = (char *)nfa.get() + sizeof(NFA);
    char *ptr = base_ptr;
    Castle *c = (Castle *)ptr;
    c->numRepeats = verify_u32(subs.size());
    c->numGroups = exclusiveInfo.numGroups;
    c->exclusive = verify_s8(exclusive);
    c->activeIdxSize = verify_u8(activeIdxSize);
    c->activeOffset = verify_u32(c->numGroups * activeIdxSize);
    c->groupIterOffset = groupIterOffset;

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
        if (contains(exclusiveInfo.groupId, i)) {
            sub->exclusiveId = exclusiveInfo.groupId[i];
        } else {
            sub->exclusiveId = numRepeats;
        }
    }

    ptr = base_ptr + total_size - sizeof(NFA) - byte_length(stale_iter);

    assert(ptr + byte_length(stale_iter) == base_ptr + total_size - sizeof(NFA));
    if (!stale_iter.empty()) {
        c->staleIterOffset = verify_u32(ptr - base_ptr);
        copy_bytes(ptr, stale_iter);
        ptr += byte_length(stale_iter);
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

CastleProto::CastleProto(nfa_kind k, const PureRepeat &pr) : kind(k) {
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

u32 CastleProto::add(const PureRepeat &pr) {
    assert(repeats.size() < max_occupancy);
    assert(pr.reach == reach());
    assert(pr.reports.size() == 1);
    u32 top = next_top++;
    DEBUG_PRINTF("selected unused top %u\n", top);
    assert(!contains(repeats, top));
    repeats.emplace(top, pr);
    for (const auto &report : pr.reports) {
        report_map[report].insert(top);
    }
    return top;
}

void CastleProto::erase(u32 top) {
    DEBUG_PRINTF("erase top %u\n", top);
    assert(contains(repeats, top));
    repeats.erase(top);
    for (auto &m : report_map) {
        m.second.erase(top);
    }
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
    assert(c1.kind == c2.kind);

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
        u32 new_top = c1.merge(pr);
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
        u32 new_top = out.size();
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
    assert(c1.kind == c2.kind);

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
    assert(c1.kind == c2.kind);

    if (c1.reach() != c2.reach()) {
        DEBUG_PRINTF("different reach\n");
        return false;
    }

    return c1.repeats == c2.repeats;
}

bool requiresDedupe(const CastleProto &proto,
                    const flat_set<ReportID> &reports) {
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
        NFAEdge e = add_edge(g.start, g.accept, g);
        g[e].tops.insert(top);
        g[u].reports.insert(pr.reports.begin(), pr.reports.end());
        min_bound = 1;
    }

    for (u32 i = 0; i < min_bound; i++) {
        NFAVertex v = add_vertex(g);
        g[v].char_reach = pr.reach;
        NFAEdge e = add_edge(u, v, g);
        if (u == g.start) {
            g[e].tops.insert(top);
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
            NFAEdge e = add_edge(u, v, g);
            if (u == g.start) {
                g[e].tops.insert(top);
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

unique_ptr<NGHolder> makeHolder(const CastleProto &proto,
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

    auto g = ue2::make_unique<NGHolder>(proto.kind);

    for (const auto &m : proto.repeats) {
        addToHolder(*g, m.first, m.second);
    }

    //dumpGraph("castle_holder.dot", *g);

    // Sanity checks.
    assert(allMatchStatesHaveReports(*g));
    assert(!has_parallel_edge(*g));

    reduceGraphEquivalences(*g, cc);

    removeRedundancy(*g, SOM_NONE);

    return g;
}

} // namespace ue2
