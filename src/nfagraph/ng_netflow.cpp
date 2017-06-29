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
 * \brief Network flow (min flow, max cut) algorithms.
 */
#include "ng_netflow.h"

#include "ng_holder.h"
#include "ng_literal_analysis.h"
#include "ng_util.h"
#include "ue2common.h"
#include "util/container.h"
#include "util/graph_range.h"
#include "util/graph_small_color_map.h"

#include <algorithm>
#include <boost/graph/boykov_kolmogorov_max_flow.hpp>

using namespace std;
using boost::default_color_type;

namespace ue2 {

static
void addReverseEdge(const NGHolder &g, vector<NFAEdge> &reverseEdge,
                    NFAEdge fwd, NFAEdge rev) {
    u32 fwdIndex = g[fwd].index;
    u32 revIndex = g[rev].index;

    // Make sure our vector is big enough.
    size_t sz = max(fwdIndex, revIndex) + 1;
    if (reverseEdge.size() < sz) {
        reverseEdge.resize(sz);
    }

    // Add entries to list.
    reverseEdge[fwdIndex] = rev;
    reverseEdge[revIndex] = fwd;
}

/** Add temporary reverse edges to the graph \p g, as they are required by the
 * BGL's boykov_kolmogorov_max_flow algorithm. */
static
void addReverseEdges(NGHolder &g, vector<NFAEdge> &reverseEdge,
                     vector<u64a> &capacityMap) {
    // We're probably going to need space for 2x edge count.
    const size_t numEdges = num_edges(g);
    reverseEdge.reserve(numEdges * 2);
    capacityMap.reserve(numEdges * 2);

    // To avoid walking the graph for _ages_, we build a temporary map of all
    // edges indexed by vertex pair for existence checks.
    map<pair<size_t, size_t>, NFAEdge> allEdges;
    for (const auto &e : edges_range(g)) {
        NFAVertex u = source(e, g), v = target(e, g);
        size_t uidx = g[u].index, vidx = g[v].index;
        allEdges[make_pair(uidx, vidx)] = e;
    }

    // Now we walk over all edges and add their reverse edges to the reverseEdge
    // vector, also adding them to the graph when they don't already exist.
    for (const auto &m : allEdges) {
        const NFAEdge &fwd = m.second;
        const size_t uidx = m.first.first, vidx = m.first.second;

        auto it = allEdges.find(make_pair(vidx, uidx));
        if (it == allEdges.end()) {
            // No reverse edge, add one.
            NFAVertex u = source(fwd, g), v = target(fwd, g);
            NFAEdge rev = add_edge(v, u, g);
            it = allEdges.insert(make_pair(make_pair(vidx, uidx), rev)).first;
            // Add to capacity map.
            u32 revIndex = g[rev].index;
            if (capacityMap.size() < revIndex + 1) {
                capacityMap.resize(revIndex + 1);
            }
            capacityMap[revIndex] = 0;
        }

        addReverseEdge(g, reverseEdge, fwd, it->second);
    }
}

/** Remove all edges with indices >= \p idx. */
static
void removeEdgesFromIndex(NGHolder &g, vector<u64a> &capacityMap, u32 idx) {
    remove_edge_if([&](const NFAEdge &e) { return g[e].index >= idx; }, g);
    capacityMap.resize(idx);
    renumber_edges(g);
}

/** A wrapper around boykov_kolmogorov_max_flow, returns the max flow and
 * colour map (from which we can find the min cut). */
static
u64a getMaxFlow(NGHolder &h, const vector<u64a> &capacityMap_in,
                decltype(make_small_color_map(NGHolder())) &colorMap) {
    vector<u64a> capacityMap = capacityMap_in;
    NFAVertex src = h.start;
    NFAVertex sink = h.acceptEod;

    // netflow relies on these stylised edges, as all starts should be covered
    // by our source and all accepts by our sink.
    assert(edge(h.start, h.startDs, h).second);
    assert(edge(h.accept, h.acceptEod, h).second);

    // The boykov_kolmogorov_max_flow algorithm requires us to have reverse
    // edges for all edges in the graph, so we create them here (and remove
    // them after the call).
    const unsigned int numRealEdges = num_edges(h);
    vector<NFAEdge> reverseEdges;
    addReverseEdges(h, reverseEdges, capacityMap);

    const unsigned int numTotalEdges = num_edges(h);
    const unsigned int numVertices = num_vertices(h);

    vector<u64a> edgeResiduals(numTotalEdges);
    vector<NFAEdge> predecessors(numVertices);
    vector<s32> distances(numVertices);

    auto v_index_map = get(vertex_index, h);
    auto e_index_map = get(edge_index, h);

    u64a flow = boykov_kolmogorov_max_flow(h,
         make_iterator_property_map(capacityMap.begin(), e_index_map),
         make_iterator_property_map(edgeResiduals.begin(), e_index_map),
         make_iterator_property_map(reverseEdges.begin(), e_index_map),
         make_iterator_property_map(predecessors.begin(), v_index_map),
         colorMap,
         make_iterator_property_map(distances.begin(), v_index_map),
         v_index_map,
         src, sink);

    // Remove reverse edges from graph.
    removeEdgesFromIndex(h, capacityMap, numRealEdges);
    assert(num_edges(h) == numRealEdges);

    DEBUG_PRINTF("flow = %llu\n", flow);
    return flow;
}

/** Returns a min cut (in \p cutset) for the graph in \p h. */
vector<NFAEdge> findMinCut(NGHolder &h, const vector<u64a> &scores) {
    assert(hasCorrectlyNumberedEdges(h));
    assert(hasCorrectlyNumberedVertices(h));

    auto colors = make_small_color_map(h);
    u64a flow = getMaxFlow(h, scores, colors);

    vector<NFAEdge> picked_white;
    vector<NFAEdge> picked_black;
    u64a observed_black_flow = 0;
    u64a observed_white_flow = 0;

    for (const auto &e : edges_range(h)) {
        NFAVertex from = source(e, h);
        NFAVertex to = target(e, h);
        u64a ec = scores[h[e].index];
        if (ec == 0) {
            continue; // skips, among other things, reverse edges
        }

        auto fromColor = get(colors, from);
        auto toColor = get(colors, to);

        if (fromColor != small_color::white && toColor == small_color::white) {
            assert(ec <= INVALID_EDGE_CAP);
            DEBUG_PRINTF("found white cut edge %zu->%zu cap %llu\n",
                     h[from].index, h[to].index, ec);
            observed_white_flow += ec;
            picked_white.push_back(e);
        }
        if (fromColor == small_color::black && toColor != small_color::black) {
            assert(ec <= INVALID_EDGE_CAP);
            DEBUG_PRINTF("found black cut edge %zu->%zu cap %llu\n",
                     h[from].index, h[to].index, ec);
            observed_black_flow += ec;
            picked_black.push_back(e);
        }
    }

    DEBUG_PRINTF("min flow = %llu b flow = %llu w flow %llu\n", flow,
                 observed_black_flow, observed_white_flow);
    if (min(observed_white_flow, observed_black_flow) != flow) {
        DEBUG_PRINTF("bad cut\n");
    }

    if (observed_white_flow < observed_black_flow) {
        return picked_white;
    } else {
        return picked_black;
    }
}

} // namespace ue2
