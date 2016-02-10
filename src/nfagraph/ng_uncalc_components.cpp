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
 * \brief NFA graph merging ("uncalc")
 *
 * The file contains our collection of NFA graph merging strategies.
 *
 * NFAGraph merging is generally guided by the length of the common prefix
 * between NFAGraph pairs.
 */
#include "grey.h"
#include "ng_holder.h"
#include "ng_limex.h"
#include "ng_redundancy.h"
#include "ng_region.h"
#include "ng_restructuring.h"
#include "ng_uncalc_components.h"
#include "ng_util.h"
#include "ue2common.h"
#include "util/compile_context.h"
#include "util/container.h"
#include "util/graph_range.h"
#include "util/ue2string.h"

#include <algorithm>
#include <deque>
#include <map>
#include <queue>
#include <set>
#include <vector>

using namespace std;

namespace ue2 {

static const u32 FAST_STATE_LIMIT = 256; /**< largest possible desirable NFA */

/** Sentinel value meaning no component has yet been selected. */
static const u32 NO_COMPONENT = 0xffffffffu;

static
vector<NFAVertex> getSortedVA(const NGHolder &g,
            const ue2::unordered_map<NFAVertex, u32> &state_ids) {
    vector<NFAVertex> out;
    out.reserve(num_vertices(g));

    for (auto v : vertices_range(g)) {
        assert(contains(state_ids, v));
        if (state_ids.at(v) == NO_STATE) {
            continue;
        }
        out.push_back(v);
    }

    // Order vertices by their state indices.
    sort(begin(out), end(out), [&state_ids](NFAVertex a, NFAVertex b) {
        return state_ids.at(a) < state_ids.at(b);
    });

#ifndef NDEBUG
    // State indices should match vector indices.
    for (u32 i = 0; i < out.size(); i++) {
        assert(state_ids.at(out.at(i)) == i);
    }
#endif

    return out;
}

static never_inline
bool cplVerticesMatch(const NGHolder &ga, NFAVertex va,
                      const NGHolder &gb, NFAVertex vb) {
    // Must have the same reachability.
    if (ga[va].char_reach != gb[vb].char_reach) {
        return false;
    }

    // If they're start vertices, they must be the same one.
    if (is_any_start(va, ga) || is_any_start(vb, gb)) {
        if (ga[va].index != gb[vb].index) {
            return false;
        }
    }

    bool va_accept = edge(va, ga.accept, ga).second;
    bool vb_accept = edge(vb, gb.accept, gb).second;
    bool va_acceptEod = edge(va, ga.acceptEod, ga).second;
    bool vb_acceptEod = edge(vb, gb.acceptEod, gb).second;

    // Must have the same accept/acceptEod edges.
    if (va_accept != vb_accept || va_acceptEod != vb_acceptEod) {
        return false;
    }

    return true;
}

static never_inline
u32 cplCommonReachAndSimple(const NGHolder &ga, const vector<NFAVertex> &a,
                            const NGHolder &gb, const vector<NFAVertex> &b) {
    u32 ml = min(a.size(), b.size());
    if (ml > 65535) {
        ml = 65535;
    }

    // Count the number of common vertices which share reachability, report and
    // "startedness" properties.
    u32 max = 0;
    for (; max < ml; max++) {
        if (!cplVerticesMatch(ga, a[max], gb, b[max])) {
            break;
        }
    }

    return max;
}

u32 commonPrefixLength(const NGHolder &ga,
                       const ue2::unordered_map<NFAVertex, u32> &a_state_ids,
                       const NGHolder &gb,
                       const ue2::unordered_map<NFAVertex, u32> &b_state_ids) {
    vector<NFAVertex> a = getSortedVA(ga, a_state_ids);
    vector<NFAVertex> b = getSortedVA(gb, b_state_ids);

    /* upper bound on the common region based on local properties */
    u32 max = cplCommonReachAndSimple(ga, a, gb, b);
    DEBUG_PRINTF("cpl upper bound %u\n", max);

    while (max > 0) {
        bool ok = true;

        /* shrink max region based on in-edges from outside the region */
        for (size_t j = max; j > 0; j--) {
            for (auto u : inv_adjacent_vertices_range(a[j - 1], ga)) {
                u32 state_id = a_state_ids.at(u);
                if (state_id != NO_STATE && state_id >= max) {
                    max = j - 1;
                    DEBUG_PRINTF("lowering max to %u\n", max);
                    goto next_vertex;
                }
            }

            for (auto u : inv_adjacent_vertices_range(b[j - 1], gb)) {
                u32 state_id = b_state_ids.at(u);
                if (state_id != NO_STATE && state_id >= max) {
                    max = j - 1;
                    DEBUG_PRINTF("lowering max to %u\n", max);
                    goto next_vertex;
                }
            }

        next_vertex:;
        }

        /* Ensure that every pair of vertices has same out-edges to vertices in
           the region. */
        for (size_t i = 0; ok && i < max; i++) {
            size_t a_count = 0;
            size_t b_count = 0;

            NFAGraph::out_edge_iterator ei, ee;
            for (tie(ei, ee) = out_edges(a[i], ga); ok && ei != ee; ++ei) {
                u32 sid = a_state_ids.at(target(*ei, ga));
                if (sid == NO_STATE || sid >= max) {
                    continue;
                }

                a_count++;

                NFAEdge b_edge;
                bool has_b_edge;
                tie(b_edge, has_b_edge) = edge(b[i], b[sid], gb);

                if (!has_b_edge) {
                    max = i;
                    ok = false;
                    DEBUG_PRINTF("lowering max to %u due to edge %zu->%u\n",
                                 max, i, sid);
                    break;
                }

                if (ga[*ei].top != gb[b_edge].top) {
                    max = i;
                    ok = false;
                    DEBUG_PRINTF("tops don't match on edge %zu->%u\n",
                                 i, sid);
                }
            }

            NFAGraph::adjacency_iterator ai, ae;
            for (tie(ai, ae) = adjacent_vertices(b[i], gb); ok && ai != ae;
                 ++ai) {
                u32 sid = b_state_ids.at(*ai);
                if (sid == NO_STATE || sid >= max) {
                    continue;
                }

                b_count++;
            }

            if (a_count != b_count) {
                max = i;
                DEBUG_PRINTF("lowering max to %u due to a,b count "
                             "(a_count=%zu, b_count=%zu)\n", max, a_count,
                             b_count);
                ok = false;
            }
        }

        if (ok) {
            DEBUG_PRINTF("survived checks, returning cpl %u\n", max);
            return max;
        }
    }

    DEBUG_PRINTF("failed to find any common region\n");
    return 0;
}

static never_inline
void mergeNfa(NGHolder &dest, vector<NFAVertex> &destStateMap,
              ue2::unordered_map<NFAVertex, u32> &dest_state_ids,
              NGHolder &vic, vector<NFAVertex> &vicStateMap,
              size_t common_len) {
    map<NFAVertex, NFAVertex> vmap; // vic -> dest

    vmap[vic.start]     = dest.start;
    vmap[vic.startDs]   = dest.startDs;
    vmap[vic.accept]    = dest.accept;
    vmap[vic.acceptEod] = dest.acceptEod;
    vmap[nullptr] = nullptr;

    u32 stateNum = countStates(dest, dest_state_ids);

    // For vertices in the common len, add to vmap and merge in the reports, if
    // any.
    for (u32 i = 0; i < common_len; i++) {
        NFAVertex v_old = vicStateMap[i], v = destStateMap[i];
        vmap[v_old] = v;

        const auto &reports = vic[v_old].reports;
        dest[v].reports.insert(reports.begin(), reports.end());
    }

    // Add in vertices beyond the common len, giving them state numbers
    // starting at stateNum.
    for (u32 i = common_len; i < vicStateMap.size(); i++) {
        NFAVertex v_old = vicStateMap[i];

        if (is_special(v_old, vic)) {
            // Dest already has start vertices, just merge the reports.
            u32 idx = vic[v_old].index;
            NFAVertex v = dest.getSpecialVertex(idx);
            const auto &reports = vic[v_old].reports;
            dest[v].reports.insert(reports.begin(), reports.end());
            continue;
        }

        NFAVertex v = add_vertex(vic[v_old], dest);
        dest_state_ids[v] = stateNum++;
        vmap[v_old] = v;
    }

    /* add edges */
    DEBUG_PRINTF("common_len=%zu\n", common_len);
    for (const auto &e : edges_range(vic)) {
        NFAVertex u_old = source(e, vic), v_old = target(e, vic);
        NFAVertex u = vmap[u_old], v = vmap[v_old];
        bool uspecial = is_special(u, dest);
        bool vspecial = is_special(v, dest);

        // Skip stylised edges that are already present.
        if (uspecial && vspecial && edge(u, v, dest).second) {
            continue;
        }

        // We're in the common region if v's state ID is low enough, unless v
        // is a special (an accept), in which case we use u's state ID.
        assert(contains(dest_state_ids, v));
        bool in_common_region = dest_state_ids.at(v) < common_len;
        if (vspecial && dest_state_ids.at(u) < common_len) {
            in_common_region = true;
        }

        DEBUG_PRINTF("adding idx=%u (state %u) -> idx=%u (state %u)%s\n",
                     dest[u].index, dest_state_ids.at(u),
                     dest[v].index, dest_state_ids.at(v),
                     in_common_region ? " [common]" : "");

        if (in_common_region) {
            if (!is_special(v, dest)) {
                DEBUG_PRINTF("skipping common edge\n");
                assert(edge(u, v, dest).second);
                // Should never merge edges with different top values.
                assert(vic[e].top == dest[edge(u, v, dest).first].top);
                continue;
            } else {
                assert(is_any_accept(v, dest));
                // If the edge exists in both graphs, skip it.
                if (edge(u, v, dest).second) {
                    DEBUG_PRINTF("skipping common edge to accept\n");
                    continue;
                }
            }
        }

        assert(!edge(u, v, dest).second);
        add_edge(u, v, vic[e], dest);
    }

    dest.renumberEdges();
    dest.renumberVertices();
}

static never_inline
void mergeNfaComponent(NGHolder &pholder, NGHolder &vholder, size_t cpl) {
    assert(&pholder != &vholder);

    auto v_state_ids = numberStates(vholder);
    auto p_state_ids = numberStates(pholder);
    auto vhvmap = getSortedVA(vholder, v_state_ids);
    auto phvmap = getSortedVA(pholder, p_state_ids);

    mergeNfa(pholder, phvmap, p_state_ids, vholder, vhvmap, cpl);
}

namespace {
struct NfaMergeCandidateH {
    NfaMergeCandidateH(size_t cpl_in, NGHolder *first_in, NGHolder *second_in,
                       u32 tb_in)
        : cpl(cpl_in), first(first_in), second(second_in), tie_breaker(tb_in) {}

    size_t cpl;       //!< common prefix length
    NGHolder *first;  //!< first component to merge
    NGHolder *second; //!< second component to merge
    u32 tie_breaker;  //!< for determinism

    bool operator<(const NfaMergeCandidateH &other) const {
        if (cpl != other.cpl) {
            return cpl < other.cpl;
        } else {
            return tie_breaker < other.tie_breaker;
        }
    }
};

} // end namespace

/** Returns true if graphs \p h1 and \p h2 can (and should) be merged. */
static
bool shouldMerge(NGHolder &ha,
                 const ue2::unordered_map<NFAVertex, u32> &a_state_ids,
                 NGHolder &hb,
                 const ue2::unordered_map<NFAVertex, u32> &b_state_ids,
                 size_t cpl, const ReportManager *rm,
                 const CompileContext &cc) {
    size_t combinedStateCount =
        countStates(ha, a_state_ids) + countStates(hb, b_state_ids) - cpl;

    if (combinedStateCount > FAST_STATE_LIMIT) {
        // More complex implementability check.
        NGHolder h_temp;
        cloneHolder(h_temp, ha);
        assert(h_temp.kind == hb.kind);
        mergeNfaComponent(h_temp, hb, cpl);
        reduceImplementableGraph(h_temp, SOM_NONE, rm, cc);
        u32 numStates = isImplementableNFA(h_temp, rm, cc);
        DEBUG_PRINTF("isImplementableNFA returned %u states\n", numStates);
        if (!numStates) {
            DEBUG_PRINTF("not implementable\n");
            return false;
        } else if (numStates > FAST_STATE_LIMIT) {
            DEBUG_PRINTF("too many states to merge\n");
            return false;
        }
    }

    return true;
}

/** Returns true if the graph has start vertices that are compatible for
 * merging. Rose may generate all sorts of wacky vacuous cases, and the merge
 * code isn't currently up to handling them. */
static
bool compatibleStarts(const NGHolder &ga, const NGHolder &gb) {
    // Start and startDs must have the same self-loops.
    return (edge(ga.startDs, ga.startDs, ga).second ==
            edge(gb.startDs, gb.startDs, gb).second) &&
           (edge(ga.start, ga.start, ga).second ==
            edge(gb.start, gb.start, gb).second);
}

static never_inline
void buildNfaMergeQueue(const vector<NGHolder *> &cluster,
                        priority_queue<NfaMergeCandidateH> *pq) {
    const size_t cs = cluster.size();
    assert(cs < NO_COMPONENT);

    // First, make sure all holders have numbered states and collect their
    // counts.
    vector<ue2::unordered_map<NFAVertex, u32>> states_map(cs);
    for (size_t i = 0; i < cs; i++) {
        assert(cluster[i]);
        NGHolder &g = *(cluster[i]);
        states_map[i] = numberStates(g);
    }

    vector<u16> seen_cpl(cs * cs, 0);
    vector<u32> best_comp(cs, NO_COMPONENT);

    /* TODO: understand, explain */
    for (u32 ci = 0; ci < cs; ci++) {
        for (u32 cj = ci + 1; cj < cs; cj++) {
            u16 cpl = 0;
            bool calc = false;

            if (best_comp[ci] != NO_COMPONENT) {
                u32 bc = best_comp[ci];
                if (seen_cpl[bc + cs * cj] < seen_cpl[bc + cs * ci]) {
                    cpl = seen_cpl[bc + cs * cj];
                    DEBUG_PRINTF("using cached cpl from %u %u\n", bc, cpl);
                    calc = true;
                }
            }

            if (!calc && best_comp[cj] != NO_COMPONENT) {
                u32 bc = best_comp[cj];
                if (seen_cpl[bc + cs * ci] < seen_cpl[bc + cs * cj]) {
                    cpl = seen_cpl[bc + cs * ci];
                    DEBUG_PRINTF("using cached cpl from %u %u\n", bc, cpl);
                    calc = true;
                }
            }

            NGHolder &g_i = *(cluster[ci]);
            NGHolder &g_j = *(cluster[cj]);

            if (!compatibleStarts(g_i, g_j)) {
                continue;
            }

            if (!calc) {
                cpl = commonPrefixLength(g_i, states_map[ci],
                                         g_j, states_map[cj]);
            }

            seen_cpl[ci + cs * cj] = cpl;
            seen_cpl[cj + cs * ci] = cpl;

            if (best_comp[cj] == NO_COMPONENT
                || seen_cpl[best_comp[cj] + cs * cj] < cpl) {
                best_comp[cj] = ci;
            }

            DEBUG_PRINTF("cpl %u %u = %u\n", ci, cj, cpl);

            pq->push(NfaMergeCandidateH(cpl, cluster[ci], cluster[cj],
                                        ci * cs + cj));
        }
    }
}

/**
 * True if the graphs have mergeable starts.
 *
 * Nowadays, this means that any vacuous edges must have the same tops. In
 * addition, mixed-accept cases need to have matching reports.
 */
static
bool mergeableStarts(const NGHolder &h1, const NGHolder &h2) {
    if (!isVacuous(h1) || !isVacuous(h2)) {
        return true;
    }

    // Vacuous edges from startDs should not occur: we have better ways to
    // implement true dot-star relationships. Just in case they do, ban them
    // from being merged unless they have identical reports.
    if (is_match_vertex(h1.startDs, h1) || is_match_vertex(h2.startDs, h2)) {
        assert(0);
        return false;
    }

    // If both graphs have edge (start, accept), the tops must match.
    auto e1_accept = edge(h1.start, h1.accept, h1);
    auto e2_accept = edge(h2.start, h2.accept, h2);
    if (e1_accept.second && e2_accept.second &&
        h1[e1_accept.first].top != h2[e2_accept.first].top) {
        return false;
    }

    // If both graphs have edge (start, acceptEod), the tops must match.
    auto e1_eod = edge(h1.start, h1.acceptEod, h1);
    auto e2_eod = edge(h2.start, h2.acceptEod, h2);
    if (e1_eod.second && e2_eod.second &&
        h1[e1_eod.first].top != h2[e2_eod.first].top) {
        return false;
    }

    // If one graph has an edge to accept and the other has an edge to
    // acceptEod, the reports must match for the merge to be safe.
    if ((e1_accept.second && e2_eod.second) ||
        (e2_accept.second && e1_eod.second)) {
        if (h1[h1.start].reports != h2[h2.start].reports) {
            return false;
        }
    }

    return true;
}

/** Merge graph \p ga into graph \p gb. Returns false on failure. */
bool mergeNfaPair(NGHolder &ga, NGHolder &gb, const ReportManager *rm,
                  const CompileContext &cc) {
    assert(ga.kind == gb.kind);
    auto a_state_ids = numberStates(ga);
    auto b_state_ids = numberStates(gb);

    // Vacuous NFAs require special checks on their starts to ensure that tops
    // match, and that reports match for mixed-accept cases.
    if (!mergeableStarts(ga, gb)) {
        DEBUG_PRINTF("starts aren't mergeable\n");
        return false;
    }

    u32 cpl = commonPrefixLength(ga, a_state_ids, gb, b_state_ids);
    if (!shouldMerge(gb, b_state_ids, ga, a_state_ids, cpl, rm, cc)) {
        return false;
    }

    mergeNfaComponent(gb, ga, cpl);
    reduceImplementableGraph(gb, SOM_NONE, rm, cc);
    b_state_ids = numberStates(gb);
    return true;
}

/** Merge the group of graphs in \p cluster where possible. The (from, to)
 * mapping of merged graphs is returned in \p merged. */
void mergeNfaCluster(const vector<NGHolder *> &cluster,
                     const ReportManager *rm,
                     map<NGHolder *, NGHolder *> &merged,
                     const CompileContext &cc) {
    if (cluster.size() < 2) {
        return;
    }

    DEBUG_PRINTF("new cluster, size %zu\n", cluster.size());
    merged.clear();

    priority_queue<NfaMergeCandidateH> pq;
    buildNfaMergeQueue(cluster, &pq);

    while (!pq.empty()) {
        NGHolder &pholder = *pq.top().first;
        NGHolder &vholder = *pq.top().second;
        pq.pop();

        if (contains(merged, &pholder) || contains(merged, &vholder)) {
            DEBUG_PRINTF("dead\n");
            continue;
        }

        if (!mergeNfaPair(vholder, pholder, rm, cc)) {
            DEBUG_PRINTF("merge failed\n");
            continue;
        }

        merged.emplace(&vholder, &pholder);

        // Seek closure.
        for (auto &m : merged) {
            if (m.second == &vholder) {
                m.second = &pholder;
            }
        }
    }
}

} // namespace ue2
