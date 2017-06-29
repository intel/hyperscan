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
 * \brief NFA graph vertex depth calculations.
 */
#include "ng_depth.h"
#include "ng_util.h"
#include "ue2common.h"
#include "util/graph_range.h"
#include "util/graph_small_color_map.h"

#include <deque>
#include <vector>

#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/dag_shortest_paths.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/property_maps/constant_property_map.hpp>
#include <boost/graph/reverse_graph.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/range/adaptor/reversed.hpp>

using namespace std;
using boost::filtered_graph;
using boost::make_filtered_graph;
using boost::make_constant_property;
using boost::reverse_graph;
using boost::adaptors::reverse;

namespace ue2 {

namespace {

/** Distance value used to indicate that the vertex can't be reached. */
static constexpr int DIST_UNREACHABLE = INT_MAX;

/**
 * Distance value used to indicate that the distance to a vertex is infinite
 * (for example, it's the max distance and there's a cycle in the path) or so
 * large that we should consider it effectively infinite.
 */
static constexpr int DIST_INFINITY = INT_MAX - 1;

//
// Filters
//

template <class GraphT>
struct NodeFilter {
    typedef typename GraphT::edge_descriptor EdgeT;
    NodeFilter() {} // BGL filters must be default-constructible.
    NodeFilter(const vector<bool> *bad_in, const GraphT *g_in)
        : bad(bad_in), g(g_in) { }
    bool operator()(const EdgeT &e) const {
        assert(g && bad);

        u32 src_idx = (*g)[source(e, *g)].index;
        u32 tar_idx = (*g)[target(e, *g)].index;

        if (tar_idx == NODE_START_DOTSTAR) {
            return false;
        }

        return !(*bad)[src_idx] && !(*bad)[tar_idx];
    }

private:
    const vector<bool> *bad = nullptr;
    const GraphT *g = nullptr;
};

template <class GraphT>
struct StartFilter {
    typedef typename GraphT::edge_descriptor EdgeT;
    StartFilter() {} // BGL filters must be default-constructible.
    explicit StartFilter(const GraphT *g_in) : g(g_in) { }
    bool operator()(const EdgeT &e) const {
        assert(g);

        u32 src_idx = (*g)[source(e, *g)].index;
        u32 tar_idx = (*g)[target(e, *g)].index;

        // Remove our stylised edges from anchored start to startDs.
        if (src_idx == NODE_START && tar_idx == NODE_START_DOTSTAR) {
            return false;
        }
        // Also remove the equivalent in the reversed direction.
        if (src_idx == NODE_ACCEPT_EOD && tar_idx == NODE_ACCEPT) {
            return false;
        }
        return true;
    }

private:
    const GraphT *g = nullptr;
};

} // namespace

template<class Graph>
static
vector<bool> findLoopReachable(const Graph &g,
                               const typename Graph::vertex_descriptor src) {
    vector<bool> deadNodes(num_vertices(g));

    using Edge = typename Graph::edge_descriptor;
    using Vertex = typename Graph::vertex_descriptor;
    using EdgeSet = set<Edge>;

    EdgeSet deadEdges;
    BackEdges<EdgeSet> be(deadEdges);

    auto colors = make_small_color_map(g);

    depth_first_search(g, be, colors, src);
    auto af = make_bad_edge_filter(&deadEdges);
    auto acyclic_g = make_filtered_graph(g, af);

    vector<Vertex> topoOrder; /* actually reverse topological order */
    topoOrder.reserve(deadNodes.size());
    topological_sort(acyclic_g, back_inserter(topoOrder), color_map(colors));

    for (const auto &e : deadEdges) {
        size_t srcIdx = g[source(e, g)].index;
        if (srcIdx != NODE_START_DOTSTAR) {
            deadNodes[srcIdx] = true;
        }
    }

    for (auto v : reverse(topoOrder)) {
        for (const auto &e : in_edges_range(v, g)) {
            if (deadNodes[g[source(e, g)].index]) {
                deadNodes[g[v].index] = true;
                break;
            }
        }
    }

    return deadNodes;
}

template <class GraphT>
static
void calcDepthFromSource(const GraphT &g,
                         typename GraphT::vertex_descriptor srcVertex,
                         const vector<bool> &deadNodes, vector<int> &dMin,
                         vector<int> &dMax) {
    typedef typename GraphT::edge_descriptor EdgeT;

    const size_t numVerts = num_vertices(g);

    NodeFilter<GraphT> nf(&deadNodes, &g);
    StartFilter<GraphT> sf(&g);

    /* minimum distance needs to run on a graph with  .*start unreachable
     * from start */
    typedef filtered_graph<GraphT, StartFilter<GraphT> > StartFilteredGraph;
    const StartFilteredGraph mindist_g(g, sf);

    /* maximum distance needs to run on a graph without cycles & nodes
     * reachable from cycles */
    typedef filtered_graph<GraphT, NodeFilter<GraphT> > NodeFilteredGraph;
    const NodeFilteredGraph maxdist_g(g, nf);

    // Record distance of each vertex from source using one of the following
    // algorithms.

    /* note: filtered graphs have same num_{vertices,edges} as base */

    dMin.assign(numVerts, DIST_UNREACHABLE);
    dMax.assign(numVerts, DIST_UNREACHABLE);
    dMin[mindist_g[srcVertex].index] = 0;

    using boost::make_iterator_property_map;

    auto min_index_map = get(vertex_index, mindist_g);

    breadth_first_search(mindist_g, srcVertex,
                         visitor(make_bfs_visitor(record_distances(
                             make_iterator_property_map(dMin.begin(),
                                                        min_index_map),
                             boost::on_tree_edge())))
                         .color_map(make_small_color_map(mindist_g)));

    auto max_index_map = get(vertex_index, maxdist_g);

    dag_shortest_paths(maxdist_g, srcVertex,
                       distance_map(make_iterator_property_map(dMax.begin(),
                                                               max_index_map))
                       .weight_map(make_constant_property<EdgeT>(-1))
                       .color_map(make_small_color_map(maxdist_g)));

    for (size_t i = 0; i < numVerts; i++) {
        if (dMin[i] > DIST_UNREACHABLE) {
            dMin[i] = DIST_UNREACHABLE;
        }
        DEBUG_PRINTF("%zu: dm %d %d\n", i, dMin[i], dMax[i]);
        if (dMax[i] >= DIST_UNREACHABLE && dMin[i] < DIST_UNREACHABLE) {
            dMax[i] = -DIST_INFINITY; /* max depths currently negative */
            DEBUG_PRINTF("bumping max to %d\n", dMax[i]);
        } else if (dMax[i] >= DIST_UNREACHABLE
                   || dMax[i] < -DIST_UNREACHABLE) {
            dMax[i] = -DIST_UNREACHABLE;
            DEBUG_PRINTF("bumping max to %d\n", dMax[i]);
        }
    }
}

/**
 * \brief Convert the integer distance we use in our shortest path calculations
 * to a \ref depth value.
 */
static
depth depthFromDistance(int val) {
    assert(val >= 0);
    if (val >= DIST_UNREACHABLE) {
        return depth::unreachable();
    } else if (val == DIST_INFINITY) {
        return depth::infinity();
    }
    return depth((u32)val);
}

static
DepthMinMax getDepths(u32 idx, const vector<int> &dMin,
                      const vector<int> &dMax) {
    DepthMinMax d(depthFromDistance(dMin[idx]),
                  depthFromDistance(-1 * dMax[idx]));
    DEBUG_PRINTF("idx=%u, depths=%s\n", idx, d.str().c_str());
    assert(d.min <= d.max);
    return d;
}

template<class Graph, class Output>
static
void calcAndStoreDepth(const Graph &g,
                       const typename Graph::vertex_descriptor src,
                       const vector<bool> &deadNodes,
                       vector<int> &dMin /* util */,
                       vector<int> &dMax /* util */,
                       vector<Output> &depths,
                       DepthMinMax Output::*store) {
    calcDepthFromSource(g, src, deadNodes, dMin, dMax);

    for (auto v : vertices_range(g)) {
        u32 idx = g[v].index;
        assert(idx < depths.size());
        Output &d = depths.at(idx);
        d.*store = getDepths(idx, dMin, dMax);
    }
}

vector<NFAVertexDepth> calcDepths(const NGHolder &g) {
    assert(hasCorrectlyNumberedVertices(g));
    const size_t numVertices = num_vertices(g);

    vector<NFAVertexDepth> depths(numVertices);
    vector<int> dMin;
    vector<int> dMax;

    /*
     * create a filtered graph for max depth calculations: all nodes/edges
     * reachable from a loop need to be removed
     */
    auto deadNodes = findLoopReachable(g, g.start);

    DEBUG_PRINTF("doing start\n");
    calcAndStoreDepth(g, g.start, deadNodes, dMin, dMax, depths,
                      &NFAVertexDepth::fromStart);
    DEBUG_PRINTF("doing startds\n");
    calcAndStoreDepth(g, g.startDs, deadNodes, dMin, dMax, depths,
                      &NFAVertexDepth::fromStartDotStar);

    return depths;
}

vector<NFAVertexRevDepth> calcRevDepths(const NGHolder &g) {
    assert(hasCorrectlyNumberedVertices(g));
    const size_t numVertices = num_vertices(g);

    vector<NFAVertexRevDepth> depths(numVertices);
    vector<int> dMin;
    vector<int> dMax;

    /* reverse the graph before walking it */
    typedef reverse_graph<NGHolder, const NGHolder &> RevNFAGraph;
    const RevNFAGraph rg(g);

    assert(num_vertices(g) == num_vertices(rg));

    /*
     * create a filtered graph for max depth calculations: all nodes/edges
     * reachable from a loop need to be removed
     */
    auto deadNodes = findLoopReachable(rg, g.acceptEod);

    DEBUG_PRINTF("doing accept\n");
    calcAndStoreDepth<RevNFAGraph, NFAVertexRevDepth>(
        rg, g.accept, deadNodes, dMin, dMax, depths,
        &NFAVertexRevDepth::toAccept);
    DEBUG_PRINTF("doing accepteod\n");
    deadNodes[NODE_ACCEPT] = true; // Hide accept->acceptEod edge.
    calcAndStoreDepth<RevNFAGraph, NFAVertexRevDepth>(
        rg, g.acceptEod, deadNodes, dMin, dMax, depths,
        &NFAVertexRevDepth::toAcceptEod);

    return depths;
}

vector<NFAVertexBidiDepth> calcBidiDepths(const NGHolder &g) {
    assert(hasCorrectlyNumberedVertices(g));
    const size_t numVertices = num_vertices(g);

    vector<NFAVertexBidiDepth> depths(numVertices);
    vector<int> dMin;
    vector<int> dMax;

    /*
     * create a filtered graph for max depth calculations: all nodes/edges
     * reachable from a loop need to be removed
     */
    auto deadNodes = findLoopReachable(g, g.start);

    DEBUG_PRINTF("doing start\n");
    calcAndStoreDepth<NGHolder, NFAVertexBidiDepth>(
        g, g.start, deadNodes, dMin, dMax, depths,
        &NFAVertexBidiDepth::fromStart);
    DEBUG_PRINTF("doing startds\n");
    calcAndStoreDepth<NGHolder, NFAVertexBidiDepth>(
        g, g.startDs, deadNodes, dMin, dMax, depths,
        &NFAVertexBidiDepth::fromStartDotStar);

    /* Now go backwards */
    typedef reverse_graph<NGHolder, const NGHolder &> RevNFAGraph;
    const RevNFAGraph rg(g);
    deadNodes = findLoopReachable(rg, g.acceptEod);

    DEBUG_PRINTF("doing accept\n");
    calcAndStoreDepth<RevNFAGraph, NFAVertexBidiDepth>(
        rg, g.accept, deadNodes, dMin, dMax, depths,
        &NFAVertexBidiDepth::toAccept);
    DEBUG_PRINTF("doing accepteod\n");
    deadNodes[NODE_ACCEPT] = true; // Hide accept->acceptEod edge.
    calcAndStoreDepth<RevNFAGraph, NFAVertexBidiDepth>(
        rg, g.acceptEod, deadNodes, dMin, dMax, depths,
        &NFAVertexBidiDepth::toAcceptEod);

    return depths;
}

vector<DepthMinMax> calcDepthsFrom(const NGHolder &g, const NFAVertex src) {
    assert(hasCorrectlyNumberedVertices(g));
    const size_t numVertices = num_vertices(g);

    auto deadNodes = findLoopReachable(g, g.start);

    vector<int> dMin, dMax;
    calcDepthFromSource(g, src, deadNodes, dMin, dMax);

    vector<DepthMinMax> depths(numVertices);

    for (auto v : vertices_range(g)) {
        auto idx = g[v].index;
        depths.at(idx) = getDepths(idx, dMin, dMax);
    }

    return depths;
}

} // namespace ue2
