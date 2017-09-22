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

/** \file
 * \brief State numbering and late graph restructuring code.
 */
#include "ng_restructuring.h"

#include "grey.h"
#include "ng_holder.h"
#include "ng_util.h"
#include "ue2common.h"
#include "util/graph_range.h"

#include <algorithm>
#include <cassert>

#include <boost/graph/transpose_graph.hpp>

using namespace std;

namespace ue2 {

/** Connect the start vertex to each of the vertices in \p tops. This is useful
 * temporarily for when we need to run a graph algorithm that expects a single
 * source vertex. */
static
void wireStartToTops(NGHolder &g, const flat_set<NFAVertex> &tops,
                     vector<NFAEdge> &tempEdges) {
    for (NFAVertex v : tops) {
        assert(!isLeafNode(v, g));

        const NFAEdge &e = add_edge(g.start, v, g);
        tempEdges.push_back(e);
    }
}

/**
 * Returns true if start's successors (aside from startDs) are subset of
 * startDs's proper successors or if start has no successors other than startDs.
 */
static
bool startIsRedundant(const NGHolder &g) {
    /* We ignore startDs as the self-loop may have been stripped as an
     * optimisation for repeats (improveLeadingRepeats()). */
    set<NFAVertex> start;
    insert(&start,  adjacent_vertices_range(g.start, g));
    start.erase(g.startDs);

    // Trivial case: start has no successors other than startDs.
    if (start.empty()) {
        DEBUG_PRINTF("start has no out-edges other than to startDs\n");
        return true;
    }

    set<NFAVertex> startDs;
    insert(&startDs,  adjacent_vertices_range(g.startDs, g));
    startDs.erase(g.startDs);

    if (!is_subset_of(start, startDs)) {
        DEBUG_PRINTF("out-edges of start and startDs aren't equivalent\n");
        return false;
    }

    return true;
}

static
void getStateOrdering(NGHolder &g, const flat_set<NFAVertex> &tops,
                      vector<NFAVertex> &ordering) {
    // First, wire up our "tops" to start so that we have a single source,
    // which will give a nicer topo order.
    vector<NFAEdge> tempEdges;
    wireStartToTops(g, tops, tempEdges);

    renumber_vertices(g);

    vector<NFAVertex> temp = getTopoOrdering(g);

    remove_edges(tempEdges, g);

    // Move {start, startDs} to the end, so they'll be first when we reverse
    // the ordering (if they are required).
    temp.erase(remove(temp.begin(), temp.end(), g.startDs));
    temp.erase(remove(temp.begin(), temp.end(), g.start));
    if (proper_out_degree(g.startDs, g)) {
        temp.push_back(g.startDs);
    }
    if (!startIsRedundant(g)) {
        temp.push_back(g.start);
    }

    // Walk ordering, remove vertices that shouldn't be participating in state
    // numbering, such as accepts.
    for (auto v : temp) {
        if (is_any_accept(v, g)) {
            continue; // accepts don't need states
        }

        ordering.push_back(v);
    }

    // Output of topo order was in reverse.
    reverse(ordering.begin(), ordering.end());
}

// Returns the number of states.
static
unordered_map<NFAVertex, u32>
getStateIndices(const NGHolder &h, const vector<NFAVertex> &ordering) {
    unordered_map<NFAVertex, u32> states;
    for (const auto &v : vertices_range(h)) {
        states[v] = NO_STATE;
    }

    u32 stateNum = 0;
    for (auto v : ordering) {
        DEBUG_PRINTF("assigning state num %u to vertex %zu\n", stateNum,
                     h[v].index);
        states[v] = stateNum++;
    }
    return states;
}

/** UE-1648: A state with a single successor that happens to be a predecessor
 * can be given any ol' state ID by the topological ordering, so we sink it
 * next to its pred. This enables better merging. */
static
void optimiseTightLoops(const NGHolder &g, vector<NFAVertex> &ordering) {
    deque<pair<NFAVertex, NFAVertex>> candidates;

    auto start = ordering.begin();
    for (auto it = ordering.begin(), ite = ordering.end(); it != ite; ++it) {
        NFAVertex v = *it;
        if (is_special(v, g)) {
            continue;
        }

        if (out_degree(v, g) == 1) {
            NFAVertex t = *(adjacent_vertices(v, g).first);
            if (v == t) {
                continue;
            }
            if (edge(t, v, g).second && find(start, it, t) != ite) {
                candidates.push_back(make_pair(v, t));
            }
        }
    }

    for (const auto &cand : candidates) {
        NFAVertex v = cand.first, u = cand.second;
        auto u_it = find(ordering.begin(), ordering.end(), u);
        auto v_it = find(ordering.begin(), ordering.end(), v);

        // Only move candidates backwards in the ordering, and only move them
        // when necessary.
        if (u_it >= v_it || distance(u_it, v_it) == 1) {
            continue;
        }

        DEBUG_PRINTF("moving vertex %zu next to %zu\n", g[v].index, g[u].index);

        ordering.erase(v_it);
        ordering.insert(++u_it, v);
    }
}

unordered_map<NFAVertex, u32>
numberStates(NGHolder &h, const flat_set<NFAVertex> &tops) {
    DEBUG_PRINTF("numbering states for holder %p\n", &h);

    vector<NFAVertex> ordering;
    getStateOrdering(h, tops, ordering);

    optimiseTightLoops(h, ordering);

    return getStateIndices(h, ordering);
}

u32 countStates(const unordered_map<NFAVertex, u32> &state_ids) {
    if (state_ids.empty()) {
        return 0;
    }

    u32 max_state = 0;
    for (const auto &m : state_ids) {
        if (m.second != NO_STATE) {
            max_state = max(m.second, max_state);
        }
    }
    u32 num_states = max_state + 1;

    return num_states;
}

} // namespace ue2
