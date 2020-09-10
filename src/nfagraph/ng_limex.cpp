/*
 * Copyright (c) 2015-2020, Intel Corporation
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
#include "util/flat_containers.h"
#include "util/verify_types.h"

#include <algorithm>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <boost/range/adaptor/map.hpp>

using namespace std;
using boost::adaptors::map_values;
using boost::adaptors::map_keys;

namespace ue2 {

#ifndef NDEBUG
// Some sanity checking for the graph; returns false if something is wrong.
// Only used in assertions.
static
bool sanityCheckGraph(const NGHolder &g,
                      const unordered_map<NFAVertex, u32> &state_ids) {
    unordered_set<u32> seen_states;

    for (auto v : vertices_range(g)) {
        // Non-specials should have non-empty reachability.
        if (!is_special(v, g)) {
            if (g[v].char_reach.none()) {
                DEBUG_PRINTF("vertex %zu has empty reach\n", g[v].index);
                return false;
            }
        }

        // Vertices with edges to accept or acceptEod must have reports and
        // other vertices must not have them.
        if (is_match_vertex(v, g) && v != g.accept) {
            if (g[v].reports.empty()) {
                DEBUG_PRINTF("vertex %zu has no reports\n", g[v].index);
                return false;
            }
        } else if (!g[v].reports.empty()) {
            DEBUG_PRINTF("vertex %zu has reports but no accept edge\n",
                         g[v].index);
            return false;
        }

        // Participant vertices should have distinct state indices.
        if (!contains(state_ids, v)) {
            DEBUG_PRINTF("vertex %zu has no state index!\n", g[v].index);
            return false;
        }
        u32 s = state_ids.at(v);
        if (s != NO_STATE && !seen_states.insert(s).second) {
            DEBUG_PRINTF("vertex %zu has dupe state %u\n", g[v].index, s);
            return false;
        }
    }

    return true;
}
#endif

static
unordered_map<NFAVertex, NFAStateSet> findSquashStates(const NGHolder &g,
                                    const vector<BoundedRepeatData> &repeats) {
    auto squashMap = findSquashers(g);
    filterSquashers(g, squashMap);

    /* We also filter out the cyclic states representing bounded repeats, as
     * they are not really cyclic -- they may turn off unexpectedly. */
    for (const auto &br : repeats) {
        if (br.repeatMax.is_finite()) {
            squashMap.erase(br.cyclic);
        }
    }

    return squashMap;
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
CharReach calcTopVertexReach(const flat_set<u32> &tops,
                             const map<u32, CharReach> &top_reach) {
    CharReach top_cr;
    for (u32 t : tops) {
        if (contains(top_reach, t)) {
            top_cr |= top_reach.at(t);
        } else {
            top_cr = CharReach::dot();
            break;
        }
    }
    return top_cr;
}

static
NFAVertex makeTopStartVertex(NGHolder &g, const flat_set<u32> &tops,
                             const flat_set<NFAVertex> &succs,
                             const map<u32, CharReach> &top_reach) {
    assert(!succs.empty());
    assert(!tops.empty());

    bool reporter = false;

    NFAVertex u = add_vertex(g[g.start], g);
    CharReach top_cr = calcTopVertexReach(tops, top_reach);
    g[u].char_reach = top_cr;

    for (auto v : succs) {
        if (v == g.accept || v == g.acceptEod) {
            reporter = true;
        }
        add_edge(u, v, g);
    }

    // Only retain reports (which we copied on add_vertex above) for new top
    // vertices connected to accepts.
    if (!reporter) {
        g[u].reports.clear();
    }

    return u;
}

static
void pickNextTopStateToHandle(const map<u32, flat_set<NFAVertex>> &top_succs,
                              const map<NFAVertex, flat_set<u32>> &succ_tops,
                              flat_set<u32> *picked_tops,
                              flat_set<NFAVertex> *picked_succs) {
    /* pick top or vertex we want to handle */
    if (top_succs.size() < succ_tops.size()) {
        auto best = top_succs.end();
        for (auto it = top_succs.begin(); it != top_succs.end(); ++it) {
            if (best == top_succs.end()
                || it->second.size() < best->second.size()) {
                best = it;
            }
        }
        assert(best != top_succs.end());
        assert(!best->second.empty()); /* should already been pruned */

        *picked_tops = { best->first };
        *picked_succs = best->second;
    } else {
        auto best = succ_tops.end();
        for (auto it = succ_tops.begin(); it != succ_tops.end(); ++it) {
            /* have to worry about determinism for this one */
            if (best == succ_tops.end()
                || it->second.size() < best->second.size()
                || (it->second.size() == best->second.size()
                    && it->second < best->second)) {
                best = it;
            }
        }
        assert(best != succ_tops.end());
        assert(!best->second.empty()); /* should already been pruned */

        *picked_succs = { best->first };
        *picked_tops = best->second;
    }
}

static
void expandCbsByTops(const map<u32, flat_set<NFAVertex>> &unhandled_top_succs,
                     const map<u32, flat_set<NFAVertex>> &top_succs,
                     const map<NFAVertex, flat_set<u32>> &succ_tops,
                     flat_set<u32> &picked_tops,
                     flat_set<NFAVertex> &picked_succs) {
    NFAVertex v = *picked_succs.begin(); /* arbitrary successor - all equiv */
    const auto &cand_tops = succ_tops.at(v);

    for (u32 t : cand_tops) {
        if (!contains(unhandled_top_succs, t)) {
            continue;
        }
        if (!has_intersection(unhandled_top_succs.at(t), picked_succs)) {
            continue; /* not adding any useful work that hasn't already been
                       * done */
        }
        if (!is_subset_of(picked_succs, top_succs.at(t))) {
            continue; /* will not form a cbs */
        }
        picked_tops.insert(t);
    }
}

static
void expandCbsBySuccs(const map<NFAVertex, flat_set<u32>> &unhandled_succ_tops,
                      const map<u32, flat_set<NFAVertex>> &top_succs,
                      const map<NFAVertex, flat_set<u32>> &succ_tops,
                      flat_set<u32> &picked_tops,
                      flat_set<NFAVertex> &picked_succs) {
    u32 t = *picked_tops.begin(); /* arbitrary top - all equiv */
    const auto &cand_succs = top_succs.at(t);

    for (NFAVertex v : cand_succs) {
        if (!contains(unhandled_succ_tops, v)) {
            continue;
        }
        if (!has_intersection(unhandled_succ_tops.at(v), picked_tops)) {
            continue; /* not adding any useful work that hasn't already been
                       * done */
        }
        if (!is_subset_of(picked_tops, succ_tops.at(v))) {
            continue; /* will not form a cbs */
        }
        picked_succs.insert(v);
    }
}

/* See if we can expand the complete bipartite subgraph (cbs) specified by the
 * picked tops/succs by adding more to either of the tops or succs.
 */
static
void expandTopSuccCbs(const map<u32, flat_set<NFAVertex>> &top_succs,
                      const map<NFAVertex, flat_set<u32>> &succ_tops,
                      const map<u32, flat_set<NFAVertex>> &unhandled_top_succs,
                      const map<NFAVertex, flat_set<u32>> &unhandled_succ_tops,
                      flat_set<u32> &picked_tops,
                      flat_set<NFAVertex> &picked_succs) {
    /* Note: all picked (tops|succs) are equivalent */

    /* Try to expand first (as we are more likely to succeed) on the side
     * with fewest remaining things to be handled */

    if (unhandled_top_succs.size() < unhandled_succ_tops.size()) {
        expandCbsByTops(unhandled_top_succs, top_succs, succ_tops,
                        picked_tops, picked_succs);
        expandCbsBySuccs(unhandled_succ_tops, top_succs, succ_tops,
                        picked_tops, picked_succs);
    } else {
        expandCbsBySuccs(unhandled_succ_tops, top_succs, succ_tops,
                        picked_tops, picked_succs);
        expandCbsByTops(unhandled_top_succs, top_succs, succ_tops,
                        picked_tops, picked_succs);
    }
}

static
void markTopSuccAsHandled(NFAVertex start_v,
                          const flat_set<u32> &handled_tops,
                          const flat_set<NFAVertex> &handled_succs,
                          map<u32, set<NFAVertex>> &tops_out,
                          map<u32, flat_set<NFAVertex>> &unhandled_top_succs,
                          map<NFAVertex, flat_set<u32>> &unhandled_succ_tops) {
    for (u32 t : handled_tops) {
        tops_out[t].insert(start_v);
        assert(contains(unhandled_top_succs, t));
        erase_all(&unhandled_top_succs[t], handled_succs);
        if (unhandled_top_succs[t].empty()) {
            unhandled_top_succs.erase(t);
        }
    }

    for (NFAVertex v : handled_succs) {
        assert(contains(unhandled_succ_tops, v));
        erase_all(&unhandled_succ_tops[v], handled_tops);
        if (unhandled_succ_tops[v].empty()) {
            unhandled_succ_tops.erase(v);
        }
    }
}

static
void attemptToUseAsStart(const NGHolder &g,  NFAVertex u,
                         const map<u32, CharReach> &top_reach,
                         map<u32, flat_set<NFAVertex>> &unhandled_top_succs,
                         map<NFAVertex, flat_set<u32>> &unhandled_succ_tops,
                         map<u32, set<NFAVertex>> &tops_out) {
    flat_set<u32> top_inter = unhandled_succ_tops.at(u);
    flat_set<NFAVertex> succs;
    for (NFAVertex v : adjacent_vertices_range(u, g)) {
        if (!contains(unhandled_succ_tops, v)) {
            return;
        }
        /* if it has vacuous reports we need to make sure that the report sets
         * are the same */
        if ((v == g.accept || v == g.acceptEod)
            && g[g.start].reports != g[u].reports) {
            DEBUG_PRINTF("different report behaviour\n");
            return;
        }
        const flat_set<u32> &v_tops = unhandled_succ_tops.at(v);
        flat_set<u32> new_inter;
        auto ni_inserter = inserter(new_inter, new_inter.end());
        set_intersection(top_inter.begin(), top_inter.end(),
                         v_tops.begin(), v_tops.end(), ni_inserter);
        top_inter = std::move(new_inter);
        succs.insert(v);
    }

    if (top_inter.empty()) {
        return;
    }

    auto top_cr = calcTopVertexReach(top_inter, top_reach);
    if (!top_cr.isSubsetOf(g[u].char_reach)) {
        return;
    }

    DEBUG_PRINTF("reusing %zu is a start vertex\n", g[u].index);
    markTopSuccAsHandled(u, top_inter, succs, tops_out, unhandled_top_succs,
                         unhandled_succ_tops);
}

/* We may have cases where a top triggers something that starts with a .* (or
 * similar state). In these cases we can make use of that state as a start
 * state.
 */
static
void reusePredsAsStarts(const NGHolder &g, const map<u32, CharReach> &top_reach,
                        map<u32, flat_set<NFAVertex>> &unhandled_top_succs,
                        map<NFAVertex, flat_set<u32>> &unhandled_succ_tops,
                        map<u32, set<NFAVertex>> &tops_out) {
    /* create list of candidates first, to avoid issues of iter invalidation */
    DEBUG_PRINTF("attempting to reuse vertices for top starts\n");
    vector<NFAVertex> cand_starts;
    for (NFAVertex u : unhandled_succ_tops | map_keys) {
        if (hasSelfLoop(u, g)) {
            cand_starts.push_back(u);
        }
    }

    for (NFAVertex u : cand_starts) {
        if (!contains(unhandled_succ_tops, u)) {
            continue;
        }
        attemptToUseAsStart(g, u, top_reach, unhandled_top_succs,
                            unhandled_succ_tops, tops_out);
     }
}

static
void makeTopStates(NGHolder &g, map<u32, set<NFAVertex>> &tops_out,
                   const map<u32, CharReach> &top_reach) {
    /* Ideally, we want to add the smallest number of states to the graph for
     * tops to turn on so that they can accurately trigger their successors.
     *
     * The relationships between tops and their successors forms a bipartite
     * graph. Finding the optimal number of start states to add is equivalent to
     * finding a minimal biclique coverings. Unfortunately, this is known to be
     * NP-complete.
     *
     * Given this, we will just do something simple to avoid creating something
     * truly wasteful:
     * 1) Try to find any cyclic states which can act as their own start states
     * 2) Pick a top or a succ to create a start state for and then try to find
     *    the largest complete bipartite subgraph that it is part of.
     */

    map<u32, flat_set<NFAVertex>> top_succs;
    map<NFAVertex, flat_set<u32>> succ_tops;
    for (const auto &e : out_edges_range(g.start, g)) {
        NFAVertex v = target(e, g);
        for (u32 t : g[e].tops) {
            top_succs[t].insert(v);
            succ_tops[v].insert(t);
        }
    }

    auto unhandled_top_succs = top_succs;
    auto unhandled_succ_tops = succ_tops;

    reusePredsAsStarts(g, top_reach, unhandled_top_succs, unhandled_succ_tops,
                       tops_out);

    /* Note: there may be successors which are equivalent (in terms of
       top-triggering), it may be more efficient to discover this and treat them
       as a unit. TODO */

    while (!unhandled_succ_tops.empty()) {
        assert(!unhandled_top_succs.empty());
        DEBUG_PRINTF("creating top start vertex\n");
        flat_set<u32> u_tops;
        flat_set<NFAVertex> u_succs;
        pickNextTopStateToHandle(unhandled_top_succs, unhandled_succ_tops,
                                 &u_tops, &u_succs);

        expandTopSuccCbs(top_succs, succ_tops, unhandled_top_succs,
                         unhandled_succ_tops, u_tops, u_succs);

        /* create start vertex to handle this top/succ combination */
        NFAVertex u = makeTopStartVertex(g, u_tops, u_succs, top_reach);

        /* update maps */
        markTopSuccAsHandled(u, u_tops, u_succs, tops_out, unhandled_top_succs,
                             unhandled_succ_tops);
    }
    assert(unhandled_top_succs.empty());

    // We are completely replacing the start vertex, so clear its reports.
    clear_out_edges(g.start, g);
    add_edge(g.start, g.startDs, g);
    g[g.start].reports.clear();
}

static
set<NFAVertex> findZombies(const NGHolder &h,
            const map<NFAVertex, BoundedRepeatSummary> &br_cyclic,
            const unordered_map<NFAVertex, u32> &state_ids,
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
        DEBUG_PRINTF("cannot be made undead - bad reports\n");
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
void reverseStateOrdering(unordered_map<NFAVertex, u32> &state_ids) {
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
             unordered_map<NFAVertex, u32> &state_ids,
             vector<BoundedRepeatData> &repeats,
             map<u32, set<NFAVertex>> &tops) {
    assert(is_triggered(h_in) || fixed_depth_tops.empty());

    unique_ptr<NGHolder> h = cloneHolder(h_in);

    // Bounded repeat handling.
    analyseRepeats(*h, rm, fixed_depth_tops, triggers, &repeats, cc.streaming,
                   impl_test_only, cc.grey);

    // If we're building a rose/suffix, do the top dance.
    flat_set<NFAVertex> topVerts;
    if (is_triggered(*h)) {
        makeTopStates(*h, tops, findTopReach(triggers));

        for (const auto &vv : tops | map_values) {
            insert(&topVerts, vv);
        }
    }

    dropRedundantStartEdges(*h);

    // Do state numbering
    state_ids = numberStates(*h, topVerts);

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
        DEBUG_PRINTF("vertex %zu: remapped reports {%s} to programs {%s}\n",
                     h[v].index, as_string_list(old_reports).c_str(),
                     as_string_list(reports).c_str());
    }
}

static
bytecode_ptr<NFA>
constructNFA(const NGHolder &h_in, const ReportManager *rm,
             const map<u32, u32> &fixed_depth_tops,
             const map<u32, vector<vector<CharReach>>> &triggers,
             bool compress_state, bool do_accel, bool impl_test_only,
             bool &fast, u32 hint, const CompileContext &cc) {
    if (!has_managed_reports(h_in)) {
        rm = nullptr;
    } else {
        assert(rm);
    }

    unordered_map<NFAVertex, u32> state_ids;
    vector<BoundedRepeatData> repeats;
    map<u32, set<NFAVertex>> tops;
    unique_ptr<NGHolder> h
        = prepareGraph(h_in, rm, fixed_depth_tops, triggers, impl_test_only, cc,
                       state_ids, repeats, tops);

    // Quick exit: if we've got an embarrassment of riches, i.e. more states
    // than we can implement in our largest NFA model, bail here.
    u32 numStates = countStates(state_ids);
    if (numStates > NFA_MAX_STATES) {
        DEBUG_PRINTF("Can't build an NFA with %u states\n", numStates);
        return nullptr;
    }

    map<NFAVertex, BoundedRepeatSummary> br_cyclic;
    for (const auto &br : repeats) {
        br_cyclic[br.cyclic] = BoundedRepeatSummary(br.repeatMin, br.repeatMax);
    }

    unordered_map<NFAVertex, NFAStateSet> reportSquashMap;
    unordered_map<NFAVertex, NFAStateSet> squashMap;

    // build map of squashed and squashers
    if (cc.grey.squashNFA) {
        squashMap = findSquashStates(*h, repeats);

        if (rm && cc.grey.highlanderSquash) {
            reportSquashMap = findHighlanderSquashers(*h, *rm);
        }
    }

    set<NFAVertex> zombies = findZombies(*h, br_cyclic, state_ids, cc);

    if (has_managed_reports(*h)) {
        assert(rm);
        remapReportsToPrograms(*h, *rm);
    }

    if (!cc.streaming || !cc.grey.compressNFAState) {
        compress_state = false;
    }

    return generate(*h, state_ids, repeats, reportSquashMap, squashMap, tops,
                    zombies, do_accel, compress_state, fast, hint, cc);
}

bytecode_ptr<NFA>
constructNFA(const NGHolder &h_in, const ReportManager *rm,
             const map<u32, u32> &fixed_depth_tops,
             const map<u32, vector<vector<CharReach>>> &triggers,
             bool compress_state, bool &fast, const CompileContext &cc) {
    const u32 hint = INVALID_NFA;
    const bool do_accel = cc.grey.accelerateNFA;
    const bool impl_test_only = false;
    return constructNFA(h_in, rm, fixed_depth_tops, triggers, compress_state,
                        do_accel, impl_test_only, fast, hint, cc);
}

#ifndef RELEASE_BUILD
// Variant that allows a hint to be specified.
bytecode_ptr<NFA>
constructNFA(const NGHolder &h_in, const ReportManager *rm,
             const map<u32, u32> &fixed_depth_tops,
             const map<u32, vector<vector<CharReach>>> &triggers,
             bool compress_state, bool &fast, u32 hint, const CompileContext &cc) {
    const bool do_accel = cc.grey.accelerateNFA;
    const bool impl_test_only = false;
    return constructNFA(h_in, rm, fixed_depth_tops, triggers, compress_state,
                        do_accel, impl_test_only, fast, hint, cc);
}
#endif // RELEASE_BUILD

static
bytecode_ptr<NFA> constructReversedNFA_i(const NGHolder &h_in, u32 hint,
                                         const CompileContext &cc) {
    // Make a mutable copy of the graph that we can renumber etc.
    NGHolder h;
    cloneHolder(h, h_in);
    assert(h.kind == NFA_REV_PREFIX); /* triggered, raises internal callbacks */

    // Do state numbering.
    auto state_ids = numberStates(h, {});

    // Quick exit: if we've got an embarrassment of riches, i.e. more states
    // than we can implement in our largest NFA model, bail here.
    u32 numStates = countStates(state_ids);
    if (numStates > NFA_MAX_STATES) {
        DEBUG_PRINTF("Can't build an NFA with %u states\n", numStates);
        return nullptr;
    }

    assert(sanityCheckGraph(h, state_ids));

    map<u32, set<NFAVertex>> tops; /* only the standards tops for nfas */
    set<NFAVertex> zombies;
    vector<BoundedRepeatData> repeats;
    unordered_map<NFAVertex, NFAStateSet> reportSquashMap;
    unordered_map<NFAVertex, NFAStateSet> squashMap;
    UNUSED bool fast = false;

    return generate(h, state_ids, repeats, reportSquashMap, squashMap, tops,
                    zombies, false, false, fast, hint, cc);
}

bytecode_ptr<NFA> constructReversedNFA(const NGHolder &h_in,
                                       const CompileContext &cc) {
    u32 hint = INVALID_NFA; // no hint
    return constructReversedNFA_i(h_in, hint, cc);
}

#ifndef RELEASE_BUILD
// Variant that allows a hint to be specified.
bytecode_ptr<NFA> constructReversedNFA(const NGHolder &h_in, u32 hint,
                                       const CompileContext &cc) {
    return constructReversedNFA_i(h_in, hint, cc);
}
#endif // RELEASE_BUILD

u32 isImplementableNFA(const NGHolder &g, const ReportManager *rm,
                       const CompileContext &cc) {
    if (!cc.grey.allowLimExNFA) {
        return false;
    }

    assert(!can_never_match(g));

    // Quick check: we can always implement an NFA with less than NFA_MAX_STATES
    // states. Note that top masks can generate extra states, so we account for
    // those here too.
    if (num_vertices(g) + getTops(g).size() < NFA_MAX_STATES) {
        return true;
    }

    if (!has_managed_reports(g)) {
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

    unordered_map<NFAVertex, u32> state_ids;
    vector<BoundedRepeatData> repeats;
    map<u32, set<NFAVertex>> tops;
    unique_ptr<NGHolder> h
        = prepareGraph(g, rm, fixed_depth_tops, triggers, impl_test_only, cc,
                       state_ids, repeats, tops);
    assert(h);
    u32 numStates = countStates(state_ids);
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

    if (rm && has_managed_reports(g)) {
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
    if (!has_managed_reports(g)) {
        rm = nullptr;
    } else {
        assert(rm);
    }

    const bool impl_test_only = true;
    const map<u32, u32> fixed_depth_tops; // empty
    const map<u32, vector<vector<CharReach>>> triggers; // empty

    unordered_map<NFAVertex, u32> state_ids;
    vector<BoundedRepeatData> repeats;
    map<u32, set<NFAVertex>> tops;
    unique_ptr<NGHolder> h
        = prepareGraph(g, rm, fixed_depth_tops, triggers, impl_test_only, cc,
                       state_ids, repeats, tops);

    if (!h || countStates(state_ids) > NFA_MAX_STATES) {
        DEBUG_PRINTF("not constructible\n");
        return NFA_MAX_ACCEL_STATES + 1;
    }

    assert(h->kind == g.kind);

    // Should have no bearing on accel calculation, so we leave these empty.
    const set<NFAVertex> zombies;
    unordered_map<NFAVertex, NFAStateSet> reportSquashMap;
    unordered_map<NFAVertex, NFAStateSet> squashMap;

    return countAccelStates(*h, state_ids, repeats, reportSquashMap, squashMap,
                            tops, zombies, cc);
}

} // namespace ue2
