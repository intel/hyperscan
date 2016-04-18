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
 * \brief Limex NFA construction code.
 */
#include "ng_limex.h"

#include "grey.h"
#include "ng_equivalence.h"
#include "ng_holder.h"
#include "ng_misc_opt.h"
#include "ng_prune.h"
#include "ng_redundancy.h"
#include "ng_repeat.h"
#include "ng_reports.h"
#include "ng_restructuring.h"
#include "ng_squash.h"
#include "ng_util.h"
#include "ng_width.h"
#include "ue2common.h"
#include "nfa/limex_compile.h"
#include "nfa/limex_limits.h"
#include "nfa/nfa_internal.h"
#include "util/compile_context.h"
#include "util/container.h"
#include "util/graph_range.h"
#include "util/report_manager.h"
#include "util/ue2_containers.h"
#include "util/verify_types.h"

#include <map>
#include <vector>

using namespace std;

namespace ue2 {

#ifndef NDEBUG
// Some sanity checking for the graph; returns false if something is wrong.
// Only used in assertions.
static
bool sanityCheckGraph(const NGHolder &g,
                      const ue2::unordered_map<NFAVertex, u32> &state_ids) {
    ue2::unordered_set<u32> seen_states;

    for (auto v : vertices_range(g)) {
        // Non-specials should have non-empty reachability.
        if (!is_special(v, g)) {
            if (g[v].char_reach.none()) {
                DEBUG_PRINTF("vertex %u has empty reach\n",
                             g[v].index);
                return false;
            }
        }

        // Vertices with edges to accept or acceptEod must have reports.
        if (is_match_vertex(v, g) && v != g.accept) {
            if (g[v].reports.empty()) {
                DEBUG_PRINTF("vertex %u has no reports\n",
                             g[v].index);
                return false;
            }
        }

        // Participant vertices should have distinct state indices.
        if (!contains(state_ids, v)) {
            DEBUG_PRINTF("vertex %u has no state index!\n",
                         g[v].index);
            return false;
        }
        u32 s = state_ids.at(v);
        if (s != NO_STATE && !seen_states.insert(s).second) {
            DEBUG_PRINTF("vertex %u has dupe state %u\n",
                         g[v].index, s);
            return false;
        }
    }

    return true;
}
#endif

static
void findSquashStates(const NGHolder &g,
                      const vector<BoundedRepeatData> &repeats,
                      map<NFAVertex, NFAStateSet> &squashMap) {
    squashMap = findSquashers(g);
    filterSquashers(g, squashMap);

    /* We also filter out the cyclic states representing bounded repeats, as
     * they are not really cyclic. */
    for (const auto &br : repeats) {
        squashMap.erase(br.cyclic);
    }
}

/**
 * \brief Drop edges from start to vertices that also have an edge from
 * startDs.
 *
 * Note that this also includes the (start, startDs) edge, which is not
 * necessary for actual NFA implementation (and is actually something we don't
 * want to affect state numbering, etc).
 */
static
void dropRedundantStartEdges(NGHolder &g) {
    remove_out_edge_if(g.start, [&](const NFAEdge &e) {
        return edge(g.startDs, target(e, g), g).second;
    }, g);

    // Ensure that we always remove (start, startDs), even if startDs has had
    // its self-loop removed as an optimization.
    remove_edge(g.start, g.startDs, g);
}

static
void makeTopStates(NGHolder &g, map<u32, NFAVertex> &tops,
                   const map<u32, CharReach> &top_reach) {
    map<u32, vector<NFAVertex>> top_succs;
    for (const auto &e : out_edges_range(g.start, g)) {
        NFAVertex v = target(e, g);
        if (v == g.startDs) {
            continue;
        }
        u32 t = g[e].top;
        top_succs[t].push_back(v);
    }

    for (const auto &top : top_succs) {
        u32 t = top.first;

        CharReach top_cr;
        if (contains(top_reach, t)) {
            top_cr = top_reach.at(t);
        } else {
            top_cr = CharReach::dot();
        }

        assert(!contains(tops, t));

        NFAVertex s = NFAGraph::null_vertex();
        flat_set<NFAVertex> succs;
        insert(&succs, top.second);

        for (auto v : top.second) {
            if (!top_cr.isSubsetOf(g[v].char_reach)) {
                continue;
            }

            flat_set<NFAVertex> vsuccs;
            insert(&vsuccs, adjacent_vertices(v, g));

            if (succs != vsuccs) {
                continue;
            }

            if (g[v].reports != g[g.start].reports) {
                continue;
            }
            s = v;
            break;
        }

        if (!s) {
            s = add_vertex(g[g.start], g);
            g[s].char_reach = top_cr;
            for (auto v : top.second) {
                add_edge(s, v, g);
            }
        }
        tops[t] = s;
    }

    // We are completely replacing the start vertex, so clear its reports.
    clear_out_edges(g.start, g);
    add_edge(g.start, g.startDs, g);
    g[g.start].reports.clear();

    // Only retain reports (which we copied on add_vertex above) for new top
    // vertices connected to accepts.
    for (const auto &m : tops) {
        NFAVertex v = m.second;
        if (!edge(v, g.accept, g).second && !edge(v, g.acceptEod, g).second) {
            g[v].reports.clear();
        }
    }
}

static
set<NFAVertex> findZombies(const NGHolder &h,
            const map<NFAVertex, BoundedRepeatSummary> &br_cyclic,
            const ue2::unordered_map<NFAVertex, u32> &state_ids,
            const CompileContext &cc) {
    set<NFAVertex> zombies;
    if (!cc.grey.allowZombies) {
        return zombies;
    }

    // We only use zombie masks in streaming mode.
    if (!cc.streaming) {
        return zombies;
    }

    if (in_degree(h.acceptEod, h) != 1 || all_reports(h).size() != 1) {
        DEBUG_PRINTF("can be made undead - bad reports\n");
        return zombies;
    }

    for (auto u : inv_adjacent_vertices_range(h.accept, h)) {
        assert(h[u].reports.size() == 1);
        for (auto v : adjacent_vertices_range(u, h)) {
            if (edge(v, h.accept, h).second
                && h[v].char_reach.all()) {
                if (!contains(br_cyclic, v)) {
                    goto ok;
                }

                const BoundedRepeatSummary &sum = br_cyclic.at(v);

                if (u == v && sum.repeatMax.is_infinite()) {
                    goto ok;
                }

            }
        }
        DEBUG_PRINTF("does not go to dot accept\n");
        return zombies;
    ok:;
    }

    for (const auto &v : inv_adjacent_vertices_range(h.accept, h)) {
        if (state_ids.at(v) != NO_STATE) {
            zombies.insert(v);
        }
    }
    return zombies;
}

static
void reverseStateOrdering(ue2::unordered_map<NFAVertex, u32> &state_ids) {
    vector<NFAVertex> ordering;
    for (auto &e : state_ids) {
        if (e.second == NO_STATE) {
            continue;
        }
        ordering.push_back(e.first);
    }

    // Sort in reverse order by state ID.
    sort(ordering.begin(), ordering.end(),
         [&state_ids](NFAVertex a, NFAVertex b) {
             return state_ids.at(a) > state_ids.at(b);
         });

    u32 stateNum = 0;

    for (const auto &v : ordering) {
        DEBUG_PRINTF("renumber, %u -> %u\n", state_ids.at(v), stateNum);
        state_ids[v] = stateNum++;
    }
}

static
map<u32, CharReach>
findTopReach(const map<u32, vector<vector<CharReach>>> &triggers) {
    map<u32, CharReach> top_reach;

    for (const auto &m : triggers) {
        const auto top = m.first;
        CharReach cr;
        for (const auto &trigger : m.second) {
            if (trigger.empty()) {
                // We don't know anything about this trigger. Assume it can
                // have any reach.
                cr.setall();
                break;
            }
            cr |= *trigger.rbegin();
        }

        top_reach.emplace(top, cr);
    }

    return top_reach;
}

static
unique_ptr<NGHolder>
prepareGraph(const NGHolder &h_in, const ReportManager *rm,
             const map<u32, u32> &fixed_depth_tops,
             const map<u32, vector<vector<CharReach>>> &triggers,
             bool impl_test_only, const CompileContext &cc,
             ue2::unordered_map<NFAVertex, u32> &state_ids,
             vector<BoundedRepeatData> &repeats, map<u32, NFAVertex> &tops) {
    assert(is_triggered(h_in) || fixed_depth_tops.empty());

    unique_ptr<NGHolder> h = cloneHolder(h_in);

    // Bounded repeat handling.
    analyseRepeats(*h, rm, fixed_depth_tops, triggers, &repeats, cc.streaming,
                   impl_test_only, cc.grey);

    // If we're building a rose/suffix, do the top dance.
    if (is_triggered(*h)) {
        makeTopStates(*h, tops, findTopReach(triggers));
    }

    dropRedundantStartEdges(*h);

    // Do state numbering
    state_ids = numberStates(*h, tops);
    dropUnusedStarts(*h, state_ids);

    // In debugging, we sometimes like to reverse the state numbering to stress
    // the NFA construction code.
    if (cc.grey.numberNFAStatesWrong) {
        reverseStateOrdering(state_ids);
    }

    assert(sanityCheckGraph(*h, state_ids));
    return h;
}

static
void remapReportsToPrograms(NGHolder &h, const ReportManager &rm) {
    for (const auto &v : vertices_range(h)) {
        auto &reports = h[v].reports;
        if (reports.empty()) {
            continue;
        }
        auto old_reports = reports;
        reports.clear();
        for (const ReportID &id : old_reports) {
            u32 program = rm.getProgramOffset(id);
            reports.insert(program);
        }
        DEBUG_PRINTF("vertex %u: remapped reports {%s} to programs {%s}\n",
                     h[v].index, as_string_list(old_reports).c_str(),
                     as_string_list(reports).c_str());
    }
}

static
aligned_unique_ptr<NFA>
constructNFA(const NGHolder &h_in, const ReportManager *rm,
             const map<u32, u32> &fixed_depth_tops,
             const map<u32, vector<vector<CharReach>>> &triggers,
             bool compress_state, bool do_accel, bool impl_test_only, u32 hint,
             const CompileContext &cc) {
    if (!generates_callbacks(h_in)) {
        rm = nullptr;
    } else {
        assert(rm);
    }

    ue2::unordered_map<NFAVertex, u32> state_ids;
    vector<BoundedRepeatData> repeats;
    map<u32, NFAVertex> tops;
    unique_ptr<NGHolder> h
        = prepareGraph(h_in, rm, fixed_depth_tops, triggers, impl_test_only, cc,
                       state_ids, repeats, tops);

    // Quick exit: if we've got an embarrassment of riches, i.e. more states
    // than we can implement in our largest NFA model, bail here.
    u32 numStates = countStates(*h, state_ids, false);
    if (numStates > NFA_MAX_STATES) {
        DEBUG_PRINTF("Can't build an NFA with %u states\n", numStates);
        return nullptr;
    }

    map<NFAVertex, BoundedRepeatSummary> br_cyclic;
    for (const auto &br : repeats) {
        br_cyclic[br.cyclic] = BoundedRepeatSummary(br.repeatMin, br.repeatMax);
    }

    map<NFAVertex, NFAStateSet> reportSquashMap;
    map<NFAVertex, NFAStateSet> squashMap;

    // build map of squashed and squashers
    if (cc.grey.squashNFA) {
        findSquashStates(*h, repeats, squashMap);

        if (rm && cc.grey.highlanderSquash) {
            reportSquashMap = findHighlanderSquashers(*h, *rm);
        }
    }

    set<NFAVertex> zombies = findZombies(*h, br_cyclic, state_ids, cc);

    if (generates_callbacks(*h)) {
        assert(rm);
        remapReportsToPrograms(*h, *rm);
    }

    if (!cc.streaming || !cc.grey.compressNFAState) {
        compress_state = false;
    }

    return generate(*h, state_ids, repeats, reportSquashMap, squashMap, tops,
                    zombies, do_accel, compress_state, hint, cc);
}

aligned_unique_ptr<NFA>
constructNFA(const NGHolder &h_in, const ReportManager *rm,
             const map<u32, u32> &fixed_depth_tops,
             const map<u32, vector<vector<CharReach>>> &triggers,
             bool compress_state, const CompileContext &cc) {
    const u32 hint = INVALID_NFA;
    const bool do_accel = cc.grey.accelerateNFA;
    const bool impl_test_only = false;
    return constructNFA(h_in, rm, fixed_depth_tops, triggers, compress_state,
                        do_accel, impl_test_only, hint, cc);
}

#ifndef RELEASE_BUILD
// Variant that allows a hint to be specified.
aligned_unique_ptr<NFA>
constructNFA(const NGHolder &h_in, const ReportManager *rm,
             const map<u32, u32> &fixed_depth_tops,
             const map<u32, vector<vector<CharReach>>> &triggers,
             bool compress_state, u32 hint, const CompileContext &cc) {
    const bool do_accel = cc.grey.accelerateNFA;
    const bool impl_test_only = false;
    return constructNFA(h_in, rm, fixed_depth_tops, triggers,
                        compress_state, do_accel, impl_test_only, hint, cc);
}
#endif // RELEASE_BUILD

static
aligned_unique_ptr<NFA> constructReversedNFA_i(const NGHolder &h_in, u32 hint,
                                               const CompileContext &cc) {
    // Make a mutable copy of the graph that we can renumber etc.
    NGHolder h;
    cloneHolder(h, h_in);
    assert(h.kind == NFA_REV_PREFIX); /* triggered, raises internal callbacks */

    // Do state numbering.
    auto state_ids = numberStates(h);

    dropUnusedStarts(h, state_ids);

    // Quick exit: if we've got an embarrassment of riches, i.e. more states
    // than we can implement in our largest NFA model, bail here.
    u32 numStates = countStates(h, state_ids, false);
    if (numStates > NFA_MAX_STATES) {
        DEBUG_PRINTF("Can't build an NFA with %u states\n", numStates);
        return nullptr;
    }

    assert(sanityCheckGraph(h, state_ids));

    map<u32, NFAVertex> tops; /* only the standards tops for nfas */
    set<NFAVertex> zombies;
    vector<BoundedRepeatData> repeats;
    map<NFAVertex, NFAStateSet> reportSquashMap;
    map<NFAVertex, NFAStateSet> squashMap;

    return generate(h, state_ids, repeats, reportSquashMap, squashMap, tops,
                    zombies, false, false, hint, cc);
}

aligned_unique_ptr<NFA> constructReversedNFA(const NGHolder &h_in,
                                             const CompileContext &cc) {
    u32 hint = INVALID_NFA; // no hint
    return constructReversedNFA_i(h_in, hint, cc);
}

#ifndef RELEASE_BUILD
// Variant that allows a hint to be specified.
aligned_unique_ptr<NFA> constructReversedNFA(const NGHolder &h_in, u32 hint,
                                             const CompileContext &cc) {
    return constructReversedNFA_i(h_in, hint, cc);
}
#endif // RELEASE_BUILD

u32 isImplementableNFA(const NGHolder &g, const ReportManager *rm,
                       const CompileContext &cc) {
    // Quick check: we can always implement an NFA with less than NFA_MAX_STATES
    // states. Note that top masks can generate extra states, so we account for
    // those here too.
    if (num_vertices(g) + NFA_MAX_TOP_MASKS < NFA_MAX_STATES) {
        return true;
    }

    if (!generates_callbacks(g)) {
        rm = nullptr;
    } else {
        assert(rm);
    }

    // The BEST way to tell if an NFA is implementable is to implement it!
    const bool impl_test_only = true;
    const map<u32, u32> fixed_depth_tops; // empty
    const map<u32, vector<vector<CharReach>>> triggers; // empty

    /* Perform the first part of the construction process and see if the
     * resultant NGHolder has <= NFA_MAX_STATES. If it does, we know we can
     * implement it as an NFA. */

    ue2::unordered_map<NFAVertex, u32> state_ids;
    vector<BoundedRepeatData> repeats;
    map<u32, NFAVertex> tops;
    unique_ptr<NGHolder> h
        = prepareGraph(g, rm, fixed_depth_tops, triggers, impl_test_only, cc,
                       state_ids, repeats, tops);
    assert(h);
    u32 numStates = countStates(*h, state_ids, false);
    if (numStates <= NFA_MAX_STATES) {
        return numStates;
    }

    return 0;
}

void reduceImplementableGraph(NGHolder &g, som_type som, const ReportManager *rm,
                              const CompileContext &cc) {
    NGHolder g_pristine;
    cloneHolder(g_pristine, g);

    reduceGraphEquivalences(g, cc);

    removeRedundancy(g, som);

    if (rm && generates_callbacks(g)) {
        pruneHighlanderDominated(g, *rm);
    }

    if (!isImplementableNFA(g, rm, cc)) {
        DEBUG_PRINTF("reductions made graph unimplementable, roll back\n");
        clear_graph(g);
        cloneHolder(g, g_pristine);
    }
}

u32 countAccelStates(const NGHolder &g, const ReportManager *rm,
                     const CompileContext &cc) {
    if (!generates_callbacks(g)) {
        rm = nullptr;
    } else {
        assert(rm);
    }

    const bool impl_test_only = true;
    const map<u32, u32> fixed_depth_tops; // empty
    const map<u32, vector<vector<CharReach>>> triggers; // empty

    ue2::unordered_map<NFAVertex, u32> state_ids;
    vector<BoundedRepeatData> repeats;
    map<u32, NFAVertex> tops;
    unique_ptr<NGHolder> h
        = prepareGraph(g, rm, fixed_depth_tops, triggers, impl_test_only, cc,
                       state_ids, repeats, tops);

    if (!h || countStates(*h, state_ids, false) > NFA_MAX_STATES) {
        DEBUG_PRINTF("not constructible\n");
        return NFA_MAX_ACCEL_STATES + 1;
    }

    assert(h->kind == g.kind);

    // Should have no bearing on accel calculation, so we leave these empty.
    const set<NFAVertex> zombies;
    const map<NFAVertex, NFAStateSet> reportSquashMap;
    const map<NFAVertex, NFAStateSet> squashMap;

    return countAccelStates(*h, state_ids, repeats, reportSquashMap, squashMap,
                            tops, zombies, cc);
}

} // namespace ue2
