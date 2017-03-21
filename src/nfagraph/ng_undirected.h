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
 * \brief Create an undirected graph from an NFAGraph.
 */

#ifndef NG_UNDIRECTED_H
#define NG_UNDIRECTED_H

#include "ng_holder.h"
#include "ng_util.h"
#include "ue2common.h"
#include "util/graph_range.h"
#include "util/ue2_containers.h"

#include <vector>

#include <boost/graph/adjacency_list.hpp>

namespace ue2 {

/**
 * \brief BGL graph type for the undirected NFA graph.
 *
 * Note that we use a set for the out-edge lists: this avoids the construction
 * of parallel edges. The only vertex property constructed is \a
 * vertex_index_t.
 */
using NFAUndirectedGraph = boost::adjacency_list<
    boost::listS,                                    // out edges
    boost::listS,                                    // vertices
    boost::undirectedS,                              // graph is undirected
    boost::property<boost::vertex_index_t, size_t>>; // vertex properties

using NFAUndirectedVertex = NFAUndirectedGraph::vertex_descriptor;

/**
 * Make a copy of an NFAGraph with undirected edges, optionally without start
 * vertices. Mappings from the original graph to the new one are provided.
 *
 * Note that new vertex indices are assigned contiguously in \a vertices(g)
 * order.
 */
template <typename Graph>
NFAUndirectedGraph createUnGraph(const Graph &g,
           bool excludeStarts,
           bool excludeAccepts,
           unordered_map<typename Graph::vertex_descriptor,
                         NFAUndirectedVertex> &old2new) {
    NFAUndirectedGraph ug;
    size_t idx = 0;

    assert(old2new.empty());
    old2new.reserve(num_vertices(g));

    for (auto v : ue2::vertices_range(g)) {
        // skip all accept nodes
        if (excludeAccepts && is_any_accept(v, g)) {
            continue;
        }

        // skip starts if required
        if (excludeStarts && is_any_start(v, g)) {
            continue;
        }

        auto nuv = boost::add_vertex(ug);
        old2new.emplace(v, nuv);
        boost::put(boost::vertex_index, ug, nuv, idx++);
    }

    // Track seen edges so that we don't insert parallel edges.
    using Vertex = typename Graph::vertex_descriptor;
    unordered_set<std::pair<Vertex, Vertex>> seen;
    seen.reserve(num_edges(g));
    auto make_ordered_edge = [](Vertex a, Vertex b) {
        return std::make_pair(std::min(a, b), std::max(a, b));
    };

    for (const auto &e : ue2::edges_range(g)) {
        auto u = source(e, g);
        auto v = target(e, g);

        if ((excludeAccepts && is_any_accept(u, g))
            || (excludeStarts && is_any_start(u, g))) {
            continue;
        }

        if ((excludeAccepts && is_any_accept(v, g))
            || (excludeStarts && is_any_start(v, g))) {
            continue;
        }

        if (!seen.emplace(make_ordered_edge(u, v)).second) {
            continue; // skip parallel edge.
        }

        NFAUndirectedVertex new_u = old2new.at(u);
        NFAUndirectedVertex new_v = old2new.at(v);

        boost::add_edge(new_u, new_v, ug);
    }

    assert(!has_parallel_edge(ug));
    return ug;
}

} // namespace ue2

#endif /* NG_UNDIRECTED_H */
