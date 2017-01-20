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
 * \brief Add redundancy to graph to assist in SOM analysis.
 *
 * Currently patterns of the form:
 *
 *     /(GET|POST).*foo/
 *
 * baffle our SOM analysis as the T's get merged into one by our graph
 * reductions and they lose the fixed depth property. One way to solve this is
 * to tell the T vertex to go fork itself before we do the main SOM pass.
 *
 * Overall plan:
 *
 * 1. build a topo ordering
 * 2. walk vertices in topo order
 * 3. fix up vertices where possible
 * 4. go home
 *
 * Vertex fix up plan:
 *
 * 1. consider depth of vertex
 *   - if vertex is at fixed depth continue to next vertex
 *   - if vertex can be at an unbounded depth continue to next vertex
 *   - if vertex has a pred which is not a fixed depth continue to next vertex
 * 2. group preds by their depth
 * 3. for each group:
 *   - create a clone of the vertex (vertex props and out edges)
 *   - create edges from each vertex in the group to the clone
 *   - work out the depth for the clone
 * 4. blow away original vertex
 *
 * Originally in UE-1862.
 */
#include "ng_som_add_redundancy.h"

#include "ng_dump.h"
#include "ng_holder.h"
#include "ng_util.h"
#include "ue2common.h"
#include "util/container.h"
#include "util/depth.h"
#include "util/graph.h"
#include "util/graph_range.h"

using namespace std;

namespace ue2 {

/** \brief Hard limit on the maximum number of new vertices to create. */
static const size_t MAX_NEW_VERTICES = 32;

static
const DepthMinMax &getDepth(NFAVertex v, const NGHolder &g,
                            const vector<DepthMinMax> &depths) {
    return depths.at(g[v].index);
}

static
bool hasFloatingPred(NFAVertex v, const NGHolder &g,
                     const vector<DepthMinMax> &depths) {
    for (auto u : inv_adjacent_vertices_range(v, g)) {
        const DepthMinMax &d = getDepth(u, g, depths);
        if (d.min != d.max) {
            return true;
        }
    }
    return false;
}

static
bool forkVertex(NFAVertex v, NGHolder &g, vector<DepthMinMax> &depths,
                set<NFAVertex> &dead, size_t *numNewVertices) {
    map<depth, vector<NFAEdge>> predGroups;
    for (const auto &e : in_edges_range(v, g)) {
        const DepthMinMax &d = getDepth(source(e, g), g, depths);
        assert(d.min == d.max);
        predGroups[d.min].push_back(e);
    }

    DEBUG_PRINTF("forking vertex with %zu pred groups\n", predGroups.size());

    if (*numNewVertices + predGroups.size() > MAX_NEW_VERTICES) {
        return false;
    }
    *numNewVertices += predGroups.size();

    for (auto &group : predGroups) {
        const depth &predDepth = group.first;
        const vector<NFAEdge> &preds = group.second;

        // Clone v for this depth with all its associated out-edges.
        u32 clone_idx = depths.size(); // next index to be used
        NFAVertex clone = add_vertex(g[v], g);
        depth clone_depth = predDepth + 1;
        g[clone].index = clone_idx;
        depths.push_back(DepthMinMax(clone_depth, clone_depth));
        DEBUG_PRINTF("cloned vertex %u with depth %s\n", clone_idx,
                     clone_depth.str().c_str());

        // Add copies of the out-edges from v.
        for (const auto &e : out_edges_range(v, g)) {
            add_edge(clone, target(e, g), g[e], g);
        }

        // Add in-edges from preds in this group.
        for (const auto &e : preds) {
            add_edge(source(e, g), clone, g[e], g);
        }
    }

    clear_vertex(v, g);
    dead.insert(v);
    return true;
}

bool addSomRedundancy(NGHolder &g, vector<DepthMinMax> &depths) {
    DEBUG_PRINTF("entry\n");

    const vector<NFAVertex> ordering = getTopoOrdering(g);

    set<NFAVertex> dead;
    size_t numNewVertices = 0;

    for (auto it = ordering.rbegin(), ite = ordering.rend(); it != ite; ++it) {
        NFAVertex v = *it;

        if (is_special(v, g)) {
            continue;
        }
        if (!in_degree(v, g)) {
            continue; // unreachable, probably killed
        }

        const DepthMinMax &d = getDepth(v, g, depths);

        DEBUG_PRINTF("vertex %zu has depths %s\n", g[v].index,
                     d.str().c_str());

        if (d.min == d.max) {
            DEBUG_PRINTF("fixed depth\n");
            continue;
        }

        if (d.max.is_unreachable()) {
            DEBUG_PRINTF("unbounded depth\n");
            continue;
        }

        if (hasFloatingPred(v, g, depths)) {
            DEBUG_PRINTF("has floating pred\n");
            continue;
        }

        if (!forkVertex(v, g, depths, dead, &numNewVertices)) {
            DEBUG_PRINTF("new vertex limit reached\n");
            break;
        }
    }

    assert(numNewVertices <= MAX_NEW_VERTICES);

    if (dead.empty()) {
        return false; // no changes made to the graph
    }

    remove_vertices(dead, g);
    return true;
}

} // namespace ue2
