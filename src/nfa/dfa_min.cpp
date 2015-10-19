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
* \brief Build code for DFA minimization
*/

/**
 * /Summary of the Hopcrofts algorithm/
 * partition := {F, Q \ F};
 * work_queue := {F};
 * while (work_queue is not empty) do
 *    choose and remove a set A from work_queue
 *    for each c in . do
 *         let X be the set of states for which a transition on c
 *                                                  leads to a state in A
 *         for each set Y in partition for which X . Y is nonempty and
 *                                                  Y \ X is nonempty do
 *              replace Y in partition by the two sets X . Y and Y \ X
 *              if Y is in work_queue
 *                   replace Y in work_queue by the same two sets
 *              else
 *                   if |X . Y| <= |Y \ X|
 *                        add X . Y to work_queue
 *                   else
 *                        add Y \ X to work_queue
 *         end;
 *    end;
 * end;
 */

#include "dfa_min.h"

#include "grey.h"
#include "nfa/rdfa.h"
#include "nfagraph/ng_mcclellan.h"
#include "ue2common.h"
#include "util/partitioned_set.h"
#include "util/container.h"
#include "util/ue2_containers.h"

#include <algorithm>
#include <functional>
#include <map>
#include <set>
#include <vector>
#include <iterator>

#include <boost/core/noncopyable.hpp>
#include <boost/dynamic_bitset.hpp>

using namespace std;

namespace ue2 {

namespace {

struct hopcroft_state_info {
    vector<vector<dstate_id_t> > prev;
};

struct DFA_components : boost::noncopyable {
    dstate_id_t nstates;
    size_t inp_size;
    set<size_t> work_queue;
    /*Partition contains reduced states*/
    partitioned_set<dstate_id_t> partition;
    vector<hopcroft_state_info> states;

    explicit DFA_components(const raw_dfa &rdfa);
};

} //namespace

/**
 * create_map:
 *   Creates an initial partitioning and work_queue.
 *   Initial partition contains {accepting states..., Non-accepting states}
 *   Initial work_queue contains accepting state subsets
 *
 *   The initial partitioning needs to distinguish between the different
 *   reporting behaviours (unlike standard hopcroft) --> more than one subset
 *   possible for the accepting states.
 *
 *   Look for accepting states in both reports and reports_eod.
 *   Creates a map with a key(reports, reports_eod) and an id.
 *   Reports of each state are searched against the map and
 *   added to the corresponding id -> partition[id] and work_queue[id].
 *   Non Accept states are added to partition[id+1].
 */
static
vector<size_t> create_map(const raw_dfa &rdfa, set<size_t> &work_queue) {
    using ReportKey = pair<flat_set<ReportID>, flat_set<ReportID>>;
    map<ReportKey, size_t> subset_map;
    vector<size_t> state_to_subset(rdfa.states.size(), INVALID_SUBSET);

    for (size_t i = 0; i < rdfa.states.size(); i++) {
        if (!rdfa.states[i].reports.empty() ||
            !rdfa.states[i].reports_eod.empty()) {
            ReportKey key(rdfa.states[i].reports, rdfa.states[i].reports_eod);
            if (contains(subset_map, key)) {
                state_to_subset[i] = subset_map[key];
            } else {
                size_t sub = subset_map.size();
                subset_map[key] = sub;
                state_to_subset[i] = sub;
                work_queue.insert(sub);
            }
        }
    }

    /* handle non accepts */
    size_t non_accept_sub = subset_map.size();
    for (size_t i = 0; i < state_to_subset.size(); i++) {
        if (state_to_subset[i] == INVALID_SUBSET) {
            state_to_subset[i] = non_accept_sub;
        }
    }

    return state_to_subset;
}

DFA_components::DFA_components(const raw_dfa &rdfa)
                             : nstates(rdfa.states.size()),
                               inp_size(rdfa.states[nstates - 1].next.size()),
                               partition(create_map(rdfa, work_queue)) {
    /* initializing states */
    for (size_t i = 0; i < nstates; i++) {
        states.push_back(hopcroft_state_info());
        states.back().prev.resize(inp_size);
    }

    for (size_t i = 0; i < nstates; i++) {  // i is the previous state
        for (size_t  j = 0; j < inp_size; j++) {
            /* Creating X_table */
            dstate_id_t present_state = rdfa.states[i].next[j];
            states[present_state].prev[j].push_back(i);

            DEBUG_PRINTF("rdfa.states[%zu].next[%zu] %hu \n", i, j,
                         rdfa.states[i].next[j]);
        }
    }
}

/**
 * choose and remove a set A from work_queue.
 */
static
void get_work_item(DFA_components &mdfa, ue2::flat_set<dstate_id_t> &A) {
    A.clear();
    assert(!mdfa.work_queue.empty());
    set<size_t>::iterator pt = mdfa.work_queue.begin();
    insert(&A, mdfa.partition[*pt]);
    mdfa.work_queue.erase(pt);
}

/**
 * X is the set of states for which a transition on the input leads to a state
 * in A.
 */
static
void create_X(const DFA_components &mdfa, const ue2::flat_set<dstate_id_t> &A,
              size_t inp, ue2::flat_set<dstate_id_t> &X) {
    X.clear();

    for (dstate_id_t id : A) {
        insert(&X, mdfa.states[id].prev[inp]);
    }
}

/**
 * For a split set X, each subset S (given by part_index) in the partition, two
 * sets are created: v_inter (X intersection S) and v_sub (S - X).
 *
 * For each subset S in the partition that could be split (v_inter is nonempty
 * and v_sub is nonempty):
 *  - replace S in partition by the two sets v_inter and v_sub.
 *  - if S is in work_queue:
 *      - replace S in work_queue by the two subsets.
 *  - else:
 *      - replace S in work_queue by the smaller of the two sets.
 */
static
void split_and_replace_set(const size_t part_index, DFA_components &mdfa,
                           const ue2::flat_set<dstate_id_t> &splitter) {
    /* singleton sets cannot be split */
    if (mdfa.partition[part_index].size() == 1) {
        return;
    }

    size_t small_index = mdfa.partition.split(part_index, splitter);

    if (small_index == INVALID_SUBSET) {
        /* the set could not be split */
        return;
    }

    /* larger subset remains at the input subset index, if the input subset was
     * already in the work queue then the larger subset will remain there. */

    mdfa.work_queue.insert(small_index);
}

/**
 * The complete Hopcrofts algorithm is implemented in this function.
 * Choose and remove a set tray from work_queue
 * For each input- X is created.
 * For each subset in the partition, split_and_replace_sets are called with the
 * split set.
 */
static
void dfa_min(DFA_components &mdfa) {
    ue2::flat_set<dstate_id_t> A, X;
    vector<size_t> cand_subsets;

    while (!mdfa.work_queue.empty()) {
        get_work_item(mdfa, A);

        for (size_t inp = 0; inp < mdfa.inp_size; inp++) {
            create_X(mdfa, A, inp, X);
            if (X.empty()) {
                continue;
            }

            /* we only need to consider subsets with at least one member in X for
             * splitting */
            cand_subsets.clear();
            mdfa.partition.find_overlapping(X, &cand_subsets);

            for (size_t sub : cand_subsets) {
                split_and_replace_set(sub, mdfa, X);
            }
        }
    }
}

/**
 * Creating new dfa table
 * Map ordering contains key being an equivalence classes first state
 * and the value being the equivalence class index.
 * Eq_state[i] tells us new state id the equivalence class located at
 * partition[i].
 */
static
void mapping_new_states(const DFA_components &mdfa,
                        vector<dstate_id_t> &old_to_new,
                        raw_dfa &rdfa) {
    const size_t num_partitions = mdfa.partition.size();

    // Mapping from equiv class's first state to equiv class index.
    map<dstate_id_t, size_t> ordering;

    // New state id for each equiv class.
    vector<dstate_id_t> eq_state(num_partitions);

    for (size_t i = 0; i < num_partitions; i++) {
        ordering[*mdfa.partition[i].begin()] = i;
    }

    dstate_id_t new_id = 0;
    for (const auto &m : ordering) {
        eq_state[m.second] = new_id++;
    }

    for (size_t t = 0; t < mdfa.partition.size(); t++) {
        for (dstate_id_t id : mdfa.partition[t]) {
            old_to_new[id] = eq_state[t];
        }
    }

    vector<dstate> new_states;
    new_states.reserve(num_partitions);
    for (size_t i = 0; i < mdfa.nstates; i++) {
        if (contains(ordering, i)) {
            new_states.push_back(rdfa.states[i]);
        }
    }
    rdfa.states.swap(new_states);
}

static
void renumber_new_states(const DFA_components &mdfa,
                         const vector<dstate_id_t> &old_to_new,
                         raw_dfa &rdfa) {
    for (size_t i = 0; i < mdfa.partition.size(); i++) {
        for (size_t j = 0; j < mdfa.inp_size; j++) {
            dstate_id_t output = rdfa.states[i].next[j];
            rdfa.states[i].next[j] = old_to_new[output];
        }
        dstate_id_t dad = rdfa.states[i].daddy;
        rdfa.states[i].daddy = old_to_new[dad];
    }

    rdfa.start_floating = old_to_new[rdfa.start_floating];
    rdfa.start_anchored = old_to_new[rdfa.start_anchored];
}

static
void new_dfa(raw_dfa &rdfa, const DFA_components &mdfa) {
    if (mdfa.partition.size() != mdfa.nstates) {
        vector<dstate_id_t> old_to_new(mdfa.nstates);
        mapping_new_states(mdfa, old_to_new, rdfa);
        renumber_new_states(mdfa, old_to_new, rdfa);
    }
}

/**
 * MAIN FUNCTION
 */
void minimize_hopcroft(raw_dfa &rdfa, const Grey &grey) {
    if (!grey.minimizeDFA) {
        return;
    }

    UNUSED const size_t states_before = rdfa.states.size();

    DFA_components mdfa(rdfa);

    dfa_min(mdfa);
    new_dfa(rdfa, mdfa);

    DEBUG_PRINTF("reduced from %zu to %zu states\n", states_before,
                 rdfa.states.size());
}

} // namespace ue2
