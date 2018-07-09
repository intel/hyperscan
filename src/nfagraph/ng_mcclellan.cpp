/*
 * Copyright (c) 2015-2018, Intel Corporation
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
 * \brief Build code for McClellan DFA.
 */
#include "ng_mcclellan.h"

#include "grey.h"
#include "nfa/dfa_min.h"
#include "nfa/rdfa.h"
#include "ng_holder.h"
#include "ng_mcclellan_internal.h"
#include "ng_squash.h"
#include "ng_util.h"
#include "ue2common.h"
#include "util/bitfield.h"
#include "util/determinise.h"
#include "util/flat_containers.h"
#include "util/graph_range.h"
#include "util/hash.h"
#include "util/hash_dynamic_bitset.h"
#include "util/make_unique.h"
#include "util/report_manager.h"

#include <algorithm>
#include <functional>
#include <map>
#include <set>
#include <unordered_map>
#include <vector>

#include <boost/dynamic_bitset.hpp>

using namespace std;
using boost::dynamic_bitset;

namespace ue2 {

#define FINAL_DFA_STATE_LIMIT 16383
#define DFA_STATE_LIMIT 1024
#define NFA_STATE_LIMIT 256

u16 buildAlphabetFromEquivSets(const std::vector<CharReach> &esets,
                               array<u16, ALPHABET_SIZE> &alpha,
                               array<u16, ALPHABET_SIZE> &unalpha) {
    u16 i = 0;
    for (; i < esets.size(); i++) {
        const CharReach &cr = esets[i];

#ifdef DEBUG
        DEBUG_PRINTF("eq set: ");
        for (size_t s = cr.find_first(); s != CharReach::npos;
             s = cr.find_next(s)) {
            printf("%02hhx ", (u8)s);
        }
        printf("-> %u\n", i);
#endif
        u16 leader = cr.find_first();
        for (size_t s = cr.find_first(); s != CharReach::npos;
             s = cr.find_next(s)) {
            alpha[s] = i;
        }
        unalpha[i] = leader;
    }

    for (u16 j = N_CHARS; j < ALPHABET_SIZE; j++, i++) {
        alpha[j] = i;
        unalpha[i] = j;
    }

    return i; // alphabet size
}

void calculateAlphabet(const NGHolder &g, array<u16, ALPHABET_SIZE> &alpha,
                       array<u16, ALPHABET_SIZE> &unalpha, u16 *alphasize) {
    vector<CharReach> esets(1, CharReach::dot());

    for (auto v : vertices_range(g)) {
        if (is_special(v, g)) {
            continue;
        }

        const CharReach &cr = g[v].char_reach;

        for (size_t i = 0; i < esets.size(); i++) {
            if (esets[i].count() == 1) {
                continue;
            }

            CharReach t = cr & esets[i];
            if (t.any() && t != esets[i]) {
                esets[i] &= ~t;
                esets.push_back(t);
            }
        }
    }
    // for deterministic compiles
    sort(esets.begin(), esets.end());

    assert(alphasize);
    *alphasize = buildAlphabetFromEquivSets(esets, alpha, unalpha);
}

static
bool allExternalReports(const ReportManager &rm,
                        const flat_set<ReportID> &reports) {
    for (auto report_id : reports) {
        if (!isExternalReport(rm.getReport(report_id))) {
            return false;
        }
    }

    return true;
}

static
dstate_id_t successor(const vector<dstate> &dstates, dstate_id_t c,
                      const array<u16, ALPHABET_SIZE> &alpha, symbol_t s) {
    return dstates[c].next[alpha[s]];
}

void getFullTransitionFromState(const raw_dfa &n, dstate_id_t state,
                                dstate_id_t *out_table) {
    for (u32 i = 0; i < ALPHABET_SIZE; i++) {
        out_table[i] = successor(n.states, state, n.alpha_remap, i);
    }
}

template<typename stateset>
static
void populateInit(const NGHolder &g, const flat_set<NFAVertex> &unused,
                  stateset *init, stateset *init_deep,
                  vector<NFAVertex> *v_by_index) {
    for (auto v : vertices_range(g)) {
        if (contains(unused, v)) {
            continue;
        }

        u32 vert_id = g[v].index;
        assert(vert_id < init->size());

        if (is_any_start(v, g)) {
            init->set(vert_id);
            if (hasSelfLoop(v, g) || is_triggered(g)) {
                DEBUG_PRINTF("setting %u\n", vert_id);
                init_deep->set(vert_id);
            }
        }
    }

    v_by_index->clear();
    v_by_index->resize(num_vertices(g), NGHolder::null_vertex());

    for (auto v : vertices_range(g)) {
        u32 vert_id = g[v].index;
        assert((*v_by_index)[vert_id] == NGHolder::null_vertex());
        (*v_by_index)[vert_id] = v;
    }

    if (is_triggered(g)) {
        *init_deep = *init;
    }
}

template<typename StateSet>
void populateAccepts(const NGHolder &g, const flat_set<NFAVertex> &unused,
                     StateSet *accept, StateSet *acceptEod) {
    for (auto v : inv_adjacent_vertices_range(g.accept, g)) {
        if (contains(unused, v)) {
            continue;
        }
        accept->set(g[v].index);
    }
    for (auto v : inv_adjacent_vertices_range(g.acceptEod, g)) {
        if (v == g.accept) {
            continue;
        }
        if (contains(unused, v)) {
            continue;
        }
        acceptEod->set(g[v].index);
    }
}

static
bool canPruneEdgesFromAccept(const ReportManager &rm, const NGHolder &g) {
    bool seen = false;
    u32 ekey = 0;

    for (auto v : inv_adjacent_vertices_range(g.accept, g)) {
        if (is_special(v, g)) {
            continue;
        }

        for (auto report_id : g[v].reports) {
            const Report &ir = rm.getReport(report_id);

            if (!isSimpleExhaustible(ir)) {
                return false;
            }

            if (!seen) {
                seen = true;
                ekey = ir.ekey;
            } else if (ekey != ir.ekey) {
                return false;
            }
        }
    }

    /* need to check accept eod does not have any unseen reports as well */
    for (auto v : inv_adjacent_vertices_range(g.acceptEod, g)) {
        if (is_special(v, g)) {
            continue;
        }

        for (auto report_id : g[v].reports) {
            const Report &ir = rm.getReport(report_id);

            if (!isSimpleExhaustible(ir)) {
                return false;
            }

            if (!seen) {
                seen = true;
                ekey = ir.ekey;
            } else if (ekey != ir.ekey) {
                return false;
            }
        }
    }

    return true;
}

static
bool overhangMatchesTrigger(const vector<vector<CharReach> > &all_triggers,
                            vector<CharReach>::const_reverse_iterator itb,
                            vector<CharReach>::const_reverse_iterator ite) {
    for (const auto &trigger : all_triggers) {
        vector<CharReach>::const_reverse_iterator it = itb;
        vector<CharReach>::const_reverse_iterator kt = trigger.rbegin();
        for (; it != ite && kt != trigger.rend(); ++it, ++kt) {
            if ((*it & *kt).none()) {
                /* this trigger does not match the overhang, try next */
                goto try_next_trigger;
            }
        }

        return true;
    try_next_trigger:;
    }

    return false; /* no trigger matches the over hang */
}

static
bool triggerAllowed(const NGHolder &g, const NFAVertex v,
                    const vector<vector<CharReach> > &all_triggers,
                    const vector<CharReach> &trigger) {
    flat_set<NFAVertex> curr({v});
    flat_set<NFAVertex> next;

    for (auto it = trigger.rbegin(); it != trigger.rend(); ++it) {
        next.clear();

        for (auto u : curr) {
            assert(u != g.startDs); /* triggered graphs should not use sds */
            if (u == g.start) {
                if (overhangMatchesTrigger(all_triggers, it, trigger.rend())) {
                    return true;
                }
                continue;
            }

            if ((g[u].char_reach & *it).none()) {
                continue;
            }
            insert(&next, inv_adjacent_vertices(u, g));
        }

        if (next.empty()) {
            return false;
        }

        next.swap(curr);
    }

    return true;
}

void markToppableStarts(const NGHolder &g, const flat_set<NFAVertex> &unused,
                        bool single_trigger,
                        const vector<vector<CharReach>> &triggers,
                        dynamic_bitset<> *out) {
    if (single_trigger) {
        return; /* no live states can lead to new states */
    }

    for (auto v : vertices_range(g)) {
        if (contains(unused, v)) {
            continue;
        }
        for (const auto &trigger : triggers) {
            if (triggerAllowed(g, v, triggers, trigger)) {
                DEBUG_PRINTF("idx %zu is valid location for top\n", g[v].index);
                out->set(g[v].index);
                break;
            }
        }
    }

    assert(out->test(g[g.start].index));
}

namespace {

template<typename Automaton_Traits>
class Automaton_Base {
public:
    using StateSet = typename Automaton_Traits::StateSet;
    using StateMap = typename Automaton_Traits::StateMap;

    Automaton_Base(const ReportManager *rm_in, const NGHolder &graph_in,
                   bool single_trigger,
                   const vector<vector<CharReach>> &triggers, bool prunable_in)
        : rm(rm_in), graph(graph_in), numStates(num_vertices(graph)),
          unused(getRedundantStarts(graph_in)),
          init(Automaton_Traits::init_states(numStates)),
          initDS(Automaton_Traits::init_states(numStates)),
          squash(Automaton_Traits::init_states(numStates)),
          accept(Automaton_Traits::init_states(numStates)),
          acceptEod(Automaton_Traits::init_states(numStates)),
          toppable(Automaton_Traits::init_states(numStates)),
          dead(Automaton_Traits::init_states(numStates)),
          prunable(prunable_in) {
        populateInit(graph, unused, &init, &initDS, &v_by_index);
        populateAccepts(graph, unused, &accept, &acceptEod);

        start_anchored = DEAD_STATE + 1;
        if (initDS == init) {
            start_floating = start_anchored;
        } else if (initDS.any()) {
            start_floating = start_anchored + 1;
        } else {
            start_floating = DEAD_STATE;
        }

        calculateAlphabet(graph, alpha, unalpha, &alphasize);

        for (const auto &sq : findSquashers(graph)) {
            NFAVertex v = sq.first;
            u32 vert_id = graph[v].index;
            squash.set(vert_id);
            squash_mask[vert_id]
                = Automaton_Traits::copy_states(std::move(sq.second),
                                                numStates);
        }

        cr_by_index = populateCR(graph, v_by_index, alpha);
        if (is_triggered(graph)) {
            dynamic_bitset<> temp(numStates);
            markToppableStarts(graph, unused, single_trigger, triggers,
                               &temp);
            toppable = Automaton_Traits::copy_states(std::move(temp),
                                                     numStates);
        }
    }

public:
    void transition(const StateSet &in, StateSet *next) {
        transition_graph(*this, v_by_index, in, next);
    }

    const vector<StateSet> initial() {
        vector<StateSet> rv = {init};
        if (start_floating != DEAD_STATE && start_floating != start_anchored) {
            rv.push_back(initDS);
        }
        return rv;
    }

private:
    void reports_i(const StateSet &in, bool eod, flat_set<ReportID> &rv) {
        StateSet acc = in & (eod ? acceptEod : accept);
        for (size_t i = acc.find_first(); i != StateSet::npos;
             i = acc.find_next(i)) {
            NFAVertex v = v_by_index[i];
            DEBUG_PRINTF("marking report\n");
            const auto &my_reports = graph[v].reports;
            rv.insert(my_reports.begin(), my_reports.end());
        }
    }

public:
    void reports(const StateSet &in, flat_set<ReportID> &rv) {
        reports_i(in, false, rv);
    }
    void reportsEod(const StateSet &in, flat_set<ReportID> &rv) {
        reports_i(in, true, rv);
    }

    bool canPrune(const flat_set<ReportID> &test_reports) const {
        if (!rm || !prunable || !canPruneEdgesFromAccept(*rm, graph)) {
            return false;
        }
        return allExternalReports(*rm, test_reports);
    }

private:
    const ReportManager *rm;
public:
    const NGHolder &graph;
    u32 numStates;
    const flat_set<NFAVertex> unused;
    vector<NFAVertex> v_by_index;
    vector<CharReach> cr_by_index; /* pre alpha'ed */
    StateSet init;
    StateSet initDS;
    StateSet squash; /* states which allow us to mask out other states */
    StateSet accept;
    StateSet acceptEod;
    StateSet toppable; /* states which are allowed to be on when a top arrives,
                        * triggered dfas only */
    StateSet dead;
    map<u32, StateSet> squash_mask;
    bool prunable;
    array<u16, ALPHABET_SIZE> alpha;
    array<u16, ALPHABET_SIZE> unalpha;
    u16 alphasize;

    u16 start_anchored;
    u16 start_floating;
};

struct Big_Traits {
    using StateSet = dynamic_bitset<>;
    using StateMap = unordered_map<StateSet, dstate_id_t, hash_dynamic_bitset>;

    static StateSet init_states(u32 num) {
        return StateSet(num);
    }

    static StateSet copy_states(dynamic_bitset<> in, UNUSED u32 num) {
        assert(in.size() == num);
        return in;
    }
};

class Automaton_Big : public Automaton_Base<Big_Traits> {
public:
    Automaton_Big(const ReportManager *rm_in, const NGHolder &graph_in,
                  bool single_trigger,
                  const vector<vector<CharReach>> &triggers, bool prunable_in)
        : Automaton_Base(rm_in, graph_in, single_trigger, triggers,
                         prunable_in) {}
};

struct Graph_Traits {
    using StateSet = bitfield<NFA_STATE_LIMIT>;
    using StateMap = unordered_map<StateSet, dstate_id_t>;

    static StateSet init_states(UNUSED u32 num) {
        assert(num <= NFA_STATE_LIMIT);
        return StateSet();
    }

    static StateSet copy_states(const dynamic_bitset<> &in, u32 num) {
        StateSet out = init_states(num);
        for (size_t i = in.find_first(); i != in.npos && i < out.size();
             i = in.find_next(i)) {
            out.set(i);
        }
        return out;
    }
};

class Automaton_Graph : public Automaton_Base<Graph_Traits> {
public:
    Automaton_Graph(const ReportManager *rm_in, const NGHolder &graph_in,
                    bool single_trigger,
                    const vector<vector<CharReach>> &triggers, bool prunable_in)
        : Automaton_Base(rm_in, graph_in, single_trigger, triggers,
                         prunable_in) {}
};

} // namespace

static
bool startIsRedundant(const NGHolder &g) {
    set<NFAVertex> start;
    set<NFAVertex> startDs;

    insert(&start, adjacent_vertices(g.start, g));
    insert(&startDs, adjacent_vertices(g.startDs, g));

    return start == startDs;
}

flat_set<NFAVertex> getRedundantStarts(const NGHolder &g) {
    flat_set<NFAVertex> dead;
    if (startIsRedundant(g)) {
        dead.insert(g.start);
    }
    if (proper_out_degree(g.startDs, g) == 0) {
        dead.insert(g.startDs);
    }
    return dead;
}

unique_ptr<raw_dfa> buildMcClellan(const NGHolder &graph,
                                   const ReportManager *rm, bool single_trigger,
                                   const vector<vector<CharReach>> &triggers,
                                   const Grey &grey, bool finalChance) {
    if (!grey.allowMcClellan) {
        return nullptr;
    }

    DEBUG_PRINTF("attempting to build %s mcclellan\n",
                 to_string(graph.kind).c_str());
    assert(allMatchStatesHaveReports(graph));

    bool prunable = grey.highlanderPruneDFA && has_managed_reports(graph);
    assert(rm || !has_managed_reports(graph));
    if (!has_managed_reports(graph)) {
        rm = nullptr;
    }

    assert(triggers.empty() == !is_triggered(graph));

    /* We must be getting desperate if it is an outfix, so use the final chance
     * state limit logic */
    u32 state_limit
        = (graph.kind == NFA_OUTFIX || finalChance) ? FINAL_DFA_STATE_LIMIT
                                                    : DFA_STATE_LIMIT;

    const u32 numStates = num_vertices(graph);
    DEBUG_PRINTF("determinising nfa with %u vertices\n", numStates);

    if (numStates > FINAL_DFA_STATE_LIMIT) {
        DEBUG_PRINTF("rejecting nfa as too many vertices\n");
        return nullptr;
    }

    auto rdfa = ue2::make_unique<raw_dfa>(graph.kind);

    if (numStates <= NFA_STATE_LIMIT) {
        /* Fast path. Automaton_Graph uses a bitfield internally to represent
         * states and is quicker than Automaton_Big. */
        Automaton_Graph n(rm, graph, single_trigger, triggers, prunable);
        if (!determinise(n, rdfa->states, state_limit)) {
            DEBUG_PRINTF("state limit exceeded\n");
            return nullptr; /* over state limit */
        }

        rdfa->start_anchored = n.start_anchored;
        rdfa->start_floating = n.start_floating;
        rdfa->alpha_size = n.alphasize;
        rdfa->alpha_remap = n.alpha;
    } else {
        /* Slow path. Too many states to use Automaton_Graph. */
        Automaton_Big n(rm, graph, single_trigger, triggers, prunable);
        if (!determinise(n, rdfa->states, state_limit)) {
            DEBUG_PRINTF("state limit exceeded\n");
            return nullptr; /* over state limit */
        }

        rdfa->start_anchored = n.start_anchored;
        rdfa->start_floating = n.start_floating;
        rdfa->alpha_size = n.alphasize;
        rdfa->alpha_remap = n.alpha;
    }

    minimize_hopcroft(*rdfa, grey);

    DEBUG_PRINTF("after determinised into %zu states, building impl dfa "
                 "(a,f) = (%hu,%hu)\n", rdfa->states.size(),
                 rdfa->start_anchored, rdfa->start_floating);

    return rdfa;
}

unique_ptr<raw_dfa> buildMcClellan(const NGHolder &g, const ReportManager *rm,
                                   const Grey &grey) {
    assert(!is_triggered(g));
    vector<vector<CharReach>> triggers;
    return buildMcClellan(g, rm, false, triggers, grey);
}

} // namespace ue2
