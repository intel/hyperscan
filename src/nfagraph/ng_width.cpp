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
 * \brief Functions for finding the min/max width of the input required to
 * match a pattern.
 */
#include "ng_width.h"

#include "ng_holder.h"
#include "ng_util.h"
#include "ue2common.h"
#include "util/depth.h"
#include "util/graph.h"
#include "util/graph_small_color_map.h"

#include <deque>
#include <vector>

#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/dag_shortest_paths.hpp>
#include <boost/graph/filtered_graph.hpp>

using namespace std;

namespace ue2 {

namespace {

/**
 * Filter out special edges, or in the top-specific variant, start edges that
 * don't have the right top set.
 */
struct SpecialEdgeFilter {
    SpecialEdgeFilter() {}
    explicit SpecialEdgeFilter(const NGHolder &h_in) : h(&h_in) {}
    SpecialEdgeFilter(const NGHolder &h_in, u32 top_in)
        : h(&h_in), single_top(true), top(top_in) {}

    bool operator()(const NFAEdge &e) const {
        NFAVertex u = source(e, *h);
        NFAVertex v = target(e, *h);
        if ((is_any_start(u, *h) && is_any_start(v, *h)) ||
            (is_any_accept(u, *h) && is_any_accept(v, *h))) {
            return false;
        }
        if (single_top) {
            if (u == h->start && !contains((*h)[e].tops, top)) {
                return false;
            }
            if (u == h->startDs) {
                return false;
            }
        }
        return true;

    }
private:
    const NGHolder *h = nullptr;
    bool single_top = false;
    u32 top = 0;
};

} // namespace

static
depth findMinWidth(const NGHolder &h, const SpecialEdgeFilter &filter,
                   NFAVertex src) {
    if (isLeafNode(src, h)) {
        return depth::unreachable();
    }

    boost::filtered_graph<NGHolder, SpecialEdgeFilter> g(h, filter);

    assert(hasCorrectlyNumberedVertices(h));
    const size_t num = num_vertices(h);
    vector<depth> distance(num, depth::unreachable());
    distance.at(g[src].index) = depth(0);

    auto index_map = get(&NFAGraphVertexProps::index, g);

    // Since we are interested in the single-source shortest paths on a graph
    // with the same weight on every edge, using BFS will be faster than
    // Dijkstra here.
    breadth_first_search(g, src,
        visitor(make_bfs_visitor(record_distances(
                    make_iterator_property_map(distance.begin(), index_map),
                    boost::on_tree_edge()))));

    DEBUG_PRINTF("d[accept]=%s, d[acceptEod]=%s\n",
                 distance.at(NODE_ACCEPT).str().c_str(),
                 distance.at(NODE_ACCEPT_EOD).str().c_str());

    depth d = min(distance.at(NODE_ACCEPT), distance.at(NODE_ACCEPT_EOD));

    if (d.is_unreachable()) {
        return d;
    }

    assert(d.is_finite());
    assert(d > depth(0));
    return d - depth(1);
}

static
depth findMaxWidth(const NGHolder &h, const SpecialEdgeFilter &filter,
                   NFAVertex src) {
    if (isLeafNode(src, h)) {
        return depth::unreachable();
    }

    if (hasReachableCycle(h, src)) {
        // There's a cycle reachable from this src, so we have inf width.
        return depth::infinity();
    }

    boost::filtered_graph<NGHolder, SpecialEdgeFilter> g(h, filter);

    assert(hasCorrectlyNumberedVertices(h));
    const size_t num = num_vertices(h);
    vector<int> distance(num);
    auto colors = make_small_color_map(h);

    auto index_map = get(&NFAGraphVertexProps::index, g);

    // DAG shortest paths with negative edge weights.
    dag_shortest_paths(g, src,
        distance_map(make_iterator_property_map(distance.begin(), index_map))
            .weight_map(boost::make_constant_property<NFAEdge>(-1))
            .color_map(colors));

    depth acceptDepth, acceptEodDepth;
    if (get(colors, h.accept) == small_color::white) {
        acceptDepth = depth::unreachable();
    } else {
        acceptDepth = depth(-1 * distance.at(NODE_ACCEPT));
    }
    if (get(colors, h.acceptEod) == small_color::white) {
        acceptEodDepth = depth::unreachable();
    } else {
        acceptEodDepth = depth(-1 * distance.at(NODE_ACCEPT_EOD));
    }

    depth d;
    if (acceptDepth.is_unreachable()) {
        d = acceptEodDepth;
    } else if (acceptEodDepth.is_unreachable()) {
        d = acceptDepth;
    } else {
        d = max(acceptDepth, acceptEodDepth);
    }

    if (d.is_unreachable()) {
        assert(findMinWidth(h, filter, src).is_unreachable());
        return d;
    }

    // Invert sign and subtract one for start transition.
    assert(d.is_finite() && d > depth(0));
    return d - depth(1);
}

static
depth findMinWidth(const NGHolder &h, const SpecialEdgeFilter &filter) {
    depth startDepth = findMinWidth(h, filter, h.start);
    depth dotstarDepth = findMinWidth(h, filter, h.startDs);
    DEBUG_PRINTF("startDepth=%s, dotstarDepth=%s\n", startDepth.str().c_str(),
                 dotstarDepth.str().c_str());
    if (startDepth.is_unreachable()) {
        assert(dotstarDepth.is_finite());
        return dotstarDepth;
    } else if (dotstarDepth.is_unreachable()) {
        assert(startDepth.is_finite());
        return startDepth;
    } else {
        assert(min(startDepth, dotstarDepth).is_finite());
        return min(startDepth, dotstarDepth);
    }
}

depth findMinWidth(const NGHolder &h) {
    return findMinWidth(h, SpecialEdgeFilter(h));
}

depth findMinWidth(const NGHolder &h, u32 top) {
    return findMinWidth(h, SpecialEdgeFilter(h, top));
}

static
depth findMaxWidth(const NGHolder &h, const SpecialEdgeFilter &filter) {
    depth startDepth = findMaxWidth(h, filter, h.start);
    depth dotstarDepth = findMaxWidth(h, filter, h.startDs);
    DEBUG_PRINTF("startDepth=%s, dotstarDepth=%s\n", startDepth.str().c_str(),
                 dotstarDepth.str().c_str());
    if (startDepth.is_unreachable()) {
        return dotstarDepth;
    } else if (dotstarDepth.is_unreachable()) {
        return startDepth;
    } else {
        return max(startDepth, dotstarDepth);
    }
}

depth findMaxWidth(const NGHolder &h) {
    return findMaxWidth(h, SpecialEdgeFilter(h));
}

depth findMaxWidth(const NGHolder &h, u32 top) {
    return findMaxWidth(h, SpecialEdgeFilter(h, top));
}

} // namespace ue2
