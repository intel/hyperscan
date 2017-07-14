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
 * \brief Build code for DFA minimization.
 */

/**
 * /Summary of the Hopcroft minimisation algorithm/
 *
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
#include "mcclellancompile_util.h"
#include "rdfa.h"
#include "ue2common.h"
#include "util/container.h"
#include "util/flat_containers.h"
#include "util/noncopyable.h"
#include "util/partitioned_set.h"

#include <algorithm>
#include <functional>
#include <iterator>
#include <map>
#include <queue>
#include <set>
#include <vector>

using namespace std;

namespace ue2 {

namespace {

struct hopcroft_state_info {
    explicit hopcroft_state_info(size_t alpha_size) : prev(alpha_size) {}

    /** \brief Mapping from symbol to a list of predecessors that transition to
     * this state on that symbol. */
    vector<vector<dstate_id_t>> prev;
};

struct HopcroftInfo : noncopyable {
    size_t alpha_size; //!< Size of DFA alphabet.
    queue<size_t> work_queue; //!< Hopcroft work queue of partition indices.
    partitioned_set<dstate_id_t> partition; //!< Partition set of DFA states.
    vector<hopcroft_state_info> states; //!< Pre-calculated state info (preds)

    explicit HopcroftInfo(const raw_dfa &rdfa);
};

} // namespace

/**
 * \brief Create an initial partitioning and work_queue.
 *
 * Initial partition contains {accepting states..., Non-accepting states}
 * Initial work_queue contains accepting state subsets
 *
 * The initial partitioning needs to distinguish between the different
 * reporting behaviours (unlike standard Hopcroft) --> more than one subset
 * possible for the accepting states.
 *
 * Look for accepting states in both reports and reports_eod.
 * Creates a map with a key(reports, reports_eod) and an id.
 * Reports of each state are searched against the map and
 * added to the corresponding id -> partition[id] and work_queue[id].
 * Non Accept states are added to partition[id+1].
 */
static
vector<size_t> create_map(const raw_dfa &rdfa, queue<size_t> &work_queue) {
    using ReportKey = pair<flat_set<ReportID>, flat_set<ReportID>>;
    map<ReportKey, size_t> subset_map;
    vector<size_t> state_to_subset(rdfa.states.size(), INVALID_SUBSET);

    for (size_t i = 0; i < rdfa.states.size(); i++) {
        const auto &ds = rdfa.states[i];
        if (!ds.reports.empty() || !ds.reports_eod.empty()) {
            ReportKey key(ds.reports, ds.reports_eod);
            if (contains(subset_map, key)) {
                state_to_subset[i] = subset_map[key];
            } else {
                size_t sub = subset_map.size();
                subset_map.emplace(std::move(key), sub);
                state_to_subset[i] = sub;
                work_queue.push(sub);
            }
        }
    }

    /* Give non-accept states their own subset. */
    size_t non_accept_sub = subset_map.size();
    replace(state_to_subset.begin(), state_to_subset.end(), INVALID_SUBSET,
            non_accept_sub);

    return state_to_subset;
}

HopcroftInfo::HopcroftInfo(const raw_dfa &rdfa)
    : alpha_size(rdfa.alpha_size), partition(create_map(rdfa, work_queue)),
      states(rdfa.states.size(), hopcroft_state_info(alpha_size)) {
    /* Construct predecessor lists for each state, indexed by symbol. */
    for (size_t i = 0; i < states.size(); i++) { // i is the previous state
        for (size_t sym = 0; sym < alpha_size; sym++) {
            dstate_id_t present_state = rdfa.states[i].next[sym];
            states[present_state].prev[sym].push_back(i);
        }
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
void split_and_replace_set(const size_t part_index, HopcroftInfo &info,
                           const flat_set<dstate_id_t> &splitter) {
    /* singleton sets cannot be split */
    if (info.partition[part_index].size() == 1) {
        return;
    }

    size_t small_index = info.partition.split(part_index, splitter);

    if (small_index == INVALID_SUBSET) {
        /* the set could not be split */
        return;
    }

    /* larger subset remains at the input subset index, if the input subset was
     * already in the work queue then the larger subset will remain there. */

    info.work_queue.push(small_index);
}

/**
 * \brief Core of the Hopcroft minimisation algorithm.
 */
static
void dfa_min(HopcroftInfo &info) {
    flat_set<dstate_id_t> curr, sym_preds;
    vector<size_t> cand_subsets;

    while (!info.work_queue.empty()) {
        /* Choose and remove a set of states (curr, or A in the description
         * above) from the work queue. Note that we copy the set because the
         * partition may be split by the loop below. */
        curr.clear();
        insert(&curr, info.partition[info.work_queue.front()]);
        info.work_queue.pop();

        for (size_t sym = 0; sym < info.alpha_size; sym++) {
            /* Find the set of states sym_preds for which a transition on the
             * given symbol leads to a state in curr. */
            sym_preds.clear();
            for (dstate_id_t s : curr) {
                insert(&sym_preds, info.states[s].prev[sym]);
            }

            if (sym_preds.empty()) {
                continue;
            }

            /* we only need to consider subsets with at least one member in
             * sym_preds for splitting */
            cand_subsets.clear();
            info.partition.find_overlapping(sym_preds, &cand_subsets);

            for (size_t sub : cand_subsets) {
                split_and_replace_set(sub, info, sym_preds);
            }
        }
    }
}

/**
 * \brief Build the new DFA state table.
 */
static
void mapping_new_states(const HopcroftInfo &info,
                        vector<dstate_id_t> &old_to_new, raw_dfa &rdfa) {
    const size_t num_partitions = info.partition.size();

    // Mapping from equiv class's first state to equiv class index.
    map<dstate_id_t, size_t> ordering;

    // New state id for each equiv class.
    vector<dstate_id_t> eq_state(num_partitions);

    for (size_t i = 0; i < num_partitions; i++) {
        ordering[*info.partition[i].begin()] = i;
    }

    dstate_id_t new_id = 0;
    for (const auto &m : ordering) {
        eq_state[m.second] = new_id++;
    }

    for (size_t t = 0; t < info.partition.size(); t++) {
        for (dstate_id_t id : info.partition[t]) {
            old_to_new[id] = eq_state[t];
        }
    }

    vector<dstate> new_states;
    new_states.reserve(num_partitions);

    for (const auto &m : ordering) {
        new_states.push_back(rdfa.states[m.first]);
    }
    rdfa.states = std::move(new_states);
}

static
void renumber_new_states(const HopcroftInfo &info,
                         const vector<dstate_id_t> &old_to_new, raw_dfa &rdfa) {
    for (size_t i = 0; i < info.partition.size(); i++) {
        for (size_t sym = 0; sym < info.alpha_size; sym++) {
            dstate_id_t output = rdfa.states[i].next[sym];
            rdfa.states[i].next[sym] = old_to_new[output];
        }
        dstate_id_t dad = rdfa.states[i].daddy;
        rdfa.states[i].daddy = old_to_new[dad];
    }

    rdfa.start_floating = old_to_new[rdfa.start_floating];
    rdfa.start_anchored = old_to_new[rdfa.start_anchored];
}

static
void new_dfa(raw_dfa &rdfa, const HopcroftInfo &info) {
    if (info.partition.size() == info.states.size()) {
        return;
    }

    vector<dstate_id_t> old_to_new(info.states.size());
    mapping_new_states(info, old_to_new, rdfa);
    renumber_new_states(info, old_to_new, rdfa);
}

void minimize_hopcroft(raw_dfa &rdfa, const Grey &grey) {
    if (!grey.minimizeDFA) {
        return;
    }

    if (is_dead(rdfa)) {
        DEBUG_PRINTF("dfa is empty\n");
    }

    UNUSED const size_t states_before = rdfa.states.size();

    HopcroftInfo info(rdfa);

    dfa_min(info);
    new_dfa(rdfa, info);

    DEBUG_PRINTF("reduced from %zu to %zu states\n", states_before,
                 rdfa.states.size());
}

} // namespace ue2
