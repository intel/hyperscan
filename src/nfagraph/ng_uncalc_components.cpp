/*
 * Copyright (c) 2015-2016, Intel Corporation
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

#include <boost/range/adaptor/map.hpp>

using namespace std;
using boost::adaptors::map_values;

namespace ue2 {

static const u32 FAST_STATE_LIMIT = 256; /**< largest possible desirable NFA */

/** Sentinel value meaning no component has yet been selected. */
static const u32 NO_COMPONENT = ~0U;

static const u32 UNUSED_STATE = ~0U;

namespace {
struct ranking_info {
    explicit ranking_info(const NGHolder &h) : to_vertex(getTopoOrdering(h)) {
        u32 rank = 0;

        reverse(to_vertex.begin(), to_vertex.end());

        for (NFAVertex v : to_vertex) {
            to_rank[v] = rank++;
        }

        for (NFAVertex v : vertices_range(h)) {
            if (!contains(to_rank, v)) {
                to_rank[v] = UNUSED_STATE;
            }
        }
    }

    NFAVertex at(u32 ranking) const { return to_vertex.at(ranking); }
    u32 get(NFAVertex v) const { return to_rank.at(v); }
    u32 size() const { return (u32)to_vertex.size(); }
    u32 add_to_tail(NFAVertex v) {
        u32 rank = size();
        to_rank[v] = rank;
        to_vertex.push_back(v);
        return rank;
    }

private:
    vector<NFAVertex> to_vertex;
    unordered_map<NFAVertex, u32> to_rank;
};
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
u32 cplCommonReachAndSimple(const NGHolder &ga, const ranking_info &a_ranking,
                            const NGHolder &gb, const ranking_info &b_ranking) {
    u32 ml = min(a_ranking.size(), b_ranking.size());
    if (ml > 65535) {
        ml = 65535;
    }

    // Count the number of common vertices which share reachability, report and
    // "startedness" properties.
    u32 max = 0;
    for (; max < ml; max++) {
        if (!cplVerticesMatch(ga, a_ranking.at(max), gb, b_ranking.at(max))) {
            break;
        }
    }

    return max;
}

static
u32 commonPrefixLength(const NGHolder &ga, const ranking_info &a_ranking,
                       const NGHolder &gb, const ranking_info &b_ranking) {
    /* upper bound on the common region based on local properties */
    u32 max = cplCommonReachAndSimple(ga, a_ranking, gb, b_ranking);
    DEBUG_PRINTF("cpl upper bound %u\n", max);

    while (max > 0) {
        /* shrink max region based on in-edges from outside the region */
        for (size_t j = max; j > 0; j--) {
            NFAVertex a_v = a_ranking.at(j - 1);
            NFAVertex b_v = b_ranking.at(j - 1);
            for (auto u : inv_adjacent_vertices_range(a_v, ga)) {
                u32 state_id = a_ranking.get(u);
                if (state_id != UNUSED_STATE && state_id >= max) {
                    max = j - 1;
                    DEBUG_PRINTF("lowering max to %u\n", max);
                    goto next_vertex;
                }
            }

            for (auto u : inv_adjacent_vertices_range(b_v, gb)) {
                u32 state_id = b_ranking.get(u);
                if (state_id != UNUSED_STATE && state_id >= max) {
                    max = j - 1;
                    DEBUG_PRINTF("lowering max to %u\n", max);
                    goto next_vertex;
                }
            }

        next_vertex:;
        }

        /* Ensure that every pair of vertices has same out-edges to vertices in
           the region. */
        for (size_t i = 0; i < max; i++) {
            size_t a_count = 0;
            size_t b_count = 0;

            for (NFAEdge a_edge : out_edges_range(a_ranking.at(i), ga)) {
                u32 sid = a_ranking.get(target(a_edge, ga));
                if (sid == UNUSED_STATE || sid >= max) {
                    continue;
                }

                a_count++;

                NFAEdge b_edge = edge(b_ranking.at(i), b_ranking.at(sid), gb);

                if (!b_edge) {
                    max = i;
                    DEBUG_PRINTF("lowering max to %u due to edge %zu->%u\n",
                                 max, i, sid);
                    goto try_smaller;
                }

                if (ga[a_edge].tops != gb[b_edge].tops) {
                    max = i;
                    DEBUG_PRINTF("tops don't match on edge %zu->%u\n", i, sid);
                    goto try_smaller;
                }
            }

            for (NFAVertex b_v : adjacent_vertices_range(b_ranking.at(i), gb)) {
                u32 sid = b_ranking.get(b_v);
                if (sid == UNUSED_STATE || sid >= max) {
                    continue;
                }

                b_count++;
            }

            if (a_count != b_count) {
                max = i;
                DEBUG_PRINTF("lowering max to %u due to a,b count (a_count=%zu,"
                             " b_count=%zu)\n", max, a_count, b_count);
                goto try_smaller;
            }
        }

        DEBUG_PRINTF("survived checks, returning cpl %u\n", max);
        return max;
    try_smaller:;
    }

    DEBUG_PRINTF("failed to find any common region\n");
    return 0;
}

u32 commonPrefixLength(const NGHolder &ga, const NGHolder &gb) {
    return commonPrefixLength(ga, ranking_info(ga), gb, ranking_info(gb));
}

static never_inline
void mergeNfaComponent(NGHolder &dest, const NGHolder &vic, size_t common_len) {
    assert(&dest != &vic);

    auto dest_info = ranking_info(dest);
    auto vic_info = ranking_info(vic);

    map<NFAVertex, NFAVertex> vmap; // vic -> dest

    vmap[vic.start]     = dest.start;
    vmap[vic.startDs]   = dest.startDs;
    vmap[vic.accept]    = dest.accept;
    vmap[vic.acceptEod] = dest.acceptEod;
    vmap[NGHolder::null_vertex()] = NGHolder::null_vertex();

    // For vertices in the common len, add to vmap and merge in the reports, if
    // any.
    for (u32 i = 0; i < common_len; i++) {
        NFAVertex v_old = vic_info.at(i);
        NFAVertex v = dest_info.at(i);
        vmap[v_old] = v;

        const auto &reports = vic[v_old].reports;
        dest[v].reports.insert(reports.begin(), reports.end());
    }

    // Add in vertices beyond the common len
    for (u32 i = common_len; i < vic_info.size(); i++) {
        NFAVertex v_old = vic_info.at(i);

        if (is_special(v_old, vic)) {
            // Dest already has start vertices, just merge the reports.
            u32 idx = vic[v_old].index;
            NFAVertex v = dest.getSpecialVertex(idx);
            const auto &reports = vic[v_old].reports;
            dest[v].reports.insert(reports.begin(), reports.end());
            continue;
        }

        NFAVertex v = add_vertex(vic[v_old], dest);
        dest_info.add_to_tail(v);
        vmap[v_old] = v;
    }

    /* add edges */
    DEBUG_PRINTF("common_len=%zu\n", common_len);
    for (const auto &e : edges_range(vic)) {
        NFAVertex u_old = source(e, vic);
        NFAVertex v_old = target(e, vic);
        NFAVertex u = vmap[u_old];
        NFAVertex v = vmap[v_old];
        bool uspecial = is_special(u, dest);
        bool vspecial = is_special(v, dest);

        // Skip stylised edges that are already present.
        if (uspecial && vspecial && edge(u, v, dest).second) {
            continue;
        }

        // We're in the common region if v's state ID is low enough, unless v
        // is a special (an accept), in which case we use u's state ID.
        bool in_common_region = dest_info.get(v) < common_len;
        if (vspecial && dest_info.get(u) < common_len) {
            in_common_region = true;
        }

        DEBUG_PRINTF("adding idx=%zu (state %u) -> idx=%zu (state %u)%s\n",
                     dest[u].index, dest_info.get(u),
                     dest[v].index, dest_info.get(v),
                     in_common_region ? " [common]" : "");

        if (in_common_region) {
            if (!is_special(v, dest)) {
                DEBUG_PRINTF("skipping common edge\n");
                assert(edge(u, v, dest).second);
                // Should never merge edges with different top values.
                assert(vic[e].tops == dest[edge(u, v, dest)].tops);
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

    renumber_edges(dest);
    renumber_vertices(dest);
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
bool shouldMerge(const NGHolder &ha, const NGHolder &hb, size_t cpl,
                 const ReportManager *rm, const CompileContext &cc) {
    size_t combinedStateCount = num_vertices(ha) + num_vertices(hb) - cpl;

    combinedStateCount -= 2 * 2; /* discount accepts from both */

    if (is_triggered(ha)) {
        /* allow for a state for each top, ignore existing starts */
        combinedStateCount -= 2; /* for start, startDs */
        auto tops = getTops(ha);
        insert(&tops, getTops(hb));
        combinedStateCount += tops.size();
    }

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
    vector<ranking_info> states_map;
    states_map.reserve(cs);
    for (size_t i = 0; i < cs; i++) {
        assert(cluster[i]);
        assert(states_map.size() == i);
        const NGHolder &g = *(cluster[i]);
        states_map.emplace_back(g);
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

    /* TODO: relax top checks if reports match */

    // If both graphs have edge (start, accept), the tops must match.
    NFAEdge e1_accept = edge(h1.start, h1.accept, h1);
    NFAEdge e2_accept = edge(h2.start, h2.accept, h2);
    if (e1_accept && e2_accept && h1[e1_accept].tops != h2[e2_accept].tops) {
        return false;
    }

    // If both graphs have edge (start, acceptEod), the tops must match.
    NFAEdge e1_eod = edge(h1.start, h1.acceptEod, h1);
    NFAEdge e2_eod = edge(h2.start, h2.acceptEod, h2);
    if (e1_eod && e2_eod && h1[e1_eod].tops != h2[e2_eod].tops) {
        return false;
    }

    // If one graph has an edge to accept and the other has an edge to
    // acceptEod, the reports must match for the merge to be safe.
    if ((e1_accept && e2_eod) || (e2_accept && e1_eod)) {
        if (h1[h1.start].reports != h2[h2.start].reports) {
            return false;
        }
    }

    return true;
}

/** Merge graph \p ga into graph \p gb. Returns false on failure. */
bool mergeNfaPair(const NGHolder &ga, NGHolder &gb, const ReportManager *rm,
                  const CompileContext &cc) {
    assert(ga.kind == gb.kind);

    // Vacuous NFAs require special checks on their starts to ensure that tops
    // match, and that reports match for mixed-accept cases.
    if (!mergeableStarts(ga, gb)) {
        DEBUG_PRINTF("starts aren't mergeable\n");
        return false;
    }

    u32 cpl = commonPrefixLength(ga, gb);
    if (!shouldMerge(gb, ga, cpl, rm, cc)) {
        return false;
    }

    mergeNfaComponent(gb, ga, cpl);
    reduceImplementableGraph(gb, SOM_NONE, rm, cc);
    return true;
}

map<NGHolder *, NGHolder *> mergeNfaCluster(const vector<NGHolder *> &cluster,
                                            const ReportManager *rm,
                                            const CompileContext &cc) {
    map<NGHolder *, NGHolder *> merged;

    if (cluster.size() < 2) {
        return merged;
    }

    DEBUG_PRINTF("new cluster, size %zu\n", cluster.size());

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

    return merged;
}

} // namespace ue2
