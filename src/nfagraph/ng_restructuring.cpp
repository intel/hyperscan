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
void wireStartToTops(NGHolder &g, const map<u32, NFAVertex> &tops,
                     vector<NFAEdge> &topEdges) {
    for (const auto &top : tops) {
        NFAVertex v = top.second;
        assert(!isLeafNode(v, g));

        const NFAEdge &e = add_edge(g.start, v, g).first;
        topEdges.push_back(e);
    }
}

static
void getStateOrdering(NGHolder &g, const map<u32, NFAVertex> &tops,
                      vector<NFAVertex> &ordering) {
    // First, wire up our "tops" to start so that we have a single source,
    // which will give a nicer topo order.
    vector<NFAEdge> topEdges;
    wireStartToTops(g, tops, topEdges);

    renumberGraphVertices(g);

    vector<NFAVertex> temp = getTopoOrdering(g);

    remove_edges(topEdges, g);

    // Move {start, startDs} to the end, so they'll be first when we reverse
    // the ordering.
    temp.erase(remove(temp.begin(), temp.end(), g.startDs));
    temp.erase(remove(temp.begin(), temp.end(), g.start));
    temp.push_back(g.startDs);
    temp.push_back(g.start);

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
ue2::unordered_map<NFAVertex, u32>
getStateIndices(const NGHolder &h, const vector<NFAVertex> &ordering) {
    ue2::unordered_map<NFAVertex, u32> states;
    for (const auto &v : vertices_range(h)) {
        states[v] = NO_STATE;
    }

    u32 stateNum = 0;
    for (auto v : ordering) {
        DEBUG_PRINTF("assigning state num %u to vertex %u\n", stateNum,
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

        DEBUG_PRINTF("moving vertex %u next to %u\n",
                     g[v].index, g[u].index);

        ordering.erase(v_it);
        ordering.insert(++u_it, v);
    }
}

ue2::unordered_map<NFAVertex, u32>
numberStates(NGHolder &h, const map<u32, NFAVertex> &tops) {
    DEBUG_PRINTF("numbering states for holder %p\n", &h);

    vector<NFAVertex> ordering;
    getStateOrdering(h, tops, ordering);

    optimiseTightLoops(h, ordering);

    ue2::unordered_map<NFAVertex, u32> states = getStateIndices(h, ordering);

    return states;
}

u32 countStates(const NGHolder &g,
                const ue2::unordered_map<NFAVertex, u32> &state_ids,
                bool addTops) {
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

    assert(contains(state_ids, g.start));
    if (addTops && state_ids.at(g.start) != NO_STATE) {
        num_states--;
        set<u32> tops;
        for (auto e : out_edges_range(g.start, g)) {
            tops.insert(g[e].top);
        }
        num_states += tops.size();
    }

    return num_states;
}

/**
 * Returns true if start leads to all of startDs's proper successors or if
 * start has no successors other than startDs.
 */
static
bool startIsRedundant(const NGHolder &g) {
    set<NFAVertex> start, startDs;

    for (const auto &e : out_edges_range(g.start, g)) {
        NFAVertex v = target(e, g);
        if (v == g.startDs) {
            continue;
        }
        start.insert(v);
    }

    for (const auto &e : out_edges_range(g.startDs, g)) {
        NFAVertex v = target(e, g);
        if (v == g.startDs) {
            continue;
        }
        startDs.insert(v);
    }

    // Trivial case: start has no successors other than startDs.
    if (start.empty()) {
        DEBUG_PRINTF("start has no out-edges other than to startDs\n");
        return true;
    }

    if (start != startDs) {
        DEBUG_PRINTF("out-edges of start and startDs aren't equivalent\n");
        return false;
    }

    return true;
}

/** One final, FINAL optimisation. Drop either start or startDs if it's unused
 * in this graph. We leave this until this late because having both vertices in
 * the graph, with fixed state indices, is useful for merging and other
 * analyses. */
void dropUnusedStarts(NGHolder &g, ue2::unordered_map<NFAVertex, u32> &states) {
    u32 adj = 0;

    if (startIsRedundant(g)) {
        DEBUG_PRINTF("dropping unused start\n");
        states[g.start] = NO_STATE;
        adj++;
    }

    if (proper_out_degree(g.startDs, g) == 0) {
        DEBUG_PRINTF("dropping unused startDs\n");
        states[g.startDs] = NO_STATE;
        adj++;
    }

    if (!adj) {
        DEBUG_PRINTF("both start and startDs must remain\n");
        return;
    }

    // We have removed one or both of the starts. Walk the non-special vertices
    // in the graph with state indices assigned to them and subtract
    // adj from all of them.
    for (auto v : vertices_range(g)) {
        u32 &state = states[v]; // note ref
        if (state == NO_STATE) {
            continue;
        }
        if (is_any_start(v, g)) {
            assert(state <= 1);
            state = 0; // one start remains
        } else {
            assert(!is_special(v, g));
            assert(state >= adj);
            state -= adj;
        }
    }
}

flat_set<NFAVertex> findUnusedStates(const NGHolder &g) {
    flat_set<NFAVertex> dead;
    if (startIsRedundant(g)) {
        dead.insert(g.start);
    }
    if (proper_out_degree(g.startDs, g) == 0) {
        dead.insert(g.startDs);
    }
    return dead;
}

/** Construct a reversed copy of an arbitrary NGHolder, mapping starts to
 * accepts. */
void reverseHolder(const NGHolder &g_in, NGHolder &g) {
    // Make the BGL do the grunt work.
    ue2::unordered_map<NFAVertex, NFAVertex> vertexMap;
    boost::transpose_graph(g_in.g, g.g,
                orig_to_copy(boost::make_assoc_property_map(vertexMap)).
                vertex_index_map(get(&NFAGraphVertexProps::index, g_in.g)));

    // The transpose_graph operation will have created extra copies of our
    // specials. We have to rewire their neighbours to the 'real' specials and
    // delete them.
    NFAVertex start = vertexMap[g_in.acceptEod];
    NFAVertex startDs = vertexMap[g_in.accept];
    NFAVertex accept = vertexMap[g_in.startDs];
    NFAVertex acceptEod = vertexMap[g_in.start];

    // Successors of starts.
    for (const auto &e : out_edges_range(start, g)) {
        NFAVertex v = target(e, g);
        add_edge(g.start, v, g[e], g);
    }
    for (const auto &e : out_edges_range(startDs, g)) {
        NFAVertex v = target(e, g);
        add_edge(g.startDs, v, g[e], g);
    }

    // Predecessors of accepts.
    for (const auto &e : in_edges_range(accept, g)) {
        NFAVertex u = source(e, g);
        add_edge(u, g.accept, g[e], g);
    }
    for (const auto &e : in_edges_range(acceptEod, g)) {
        NFAVertex u = source(e, g);
        add_edge(u, g.acceptEod, g[e], g);
    }

    // Remove our impostors.
    clear_vertex(start, g);
    remove_vertex(start, g);
    clear_vertex(startDs, g);
    remove_vertex(startDs, g);
    clear_vertex(accept, g);
    remove_vertex(accept, g);
    clear_vertex(acceptEod, g);
    remove_vertex(acceptEod, g);

    // Renumber so that g's properties (number of vertices, edges) are
    // accurate.
    g.renumberVertices();
    g.renumberEdges();

    assert(num_vertices(g) == num_vertices(g_in));
    assert(num_edges(g) == num_edges(g_in));
}

} // namespace ue2
