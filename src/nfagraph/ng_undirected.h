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
 * \brief Create an undirected graph from an NFAGraph.
 */

#ifndef NG_UNDIRECTED_H_CB42C71CF38E3D
#define NG_UNDIRECTED_H_CB42C71CF38E3D

#include "ng_holder.h"
#include "ng_util.h"
#include "ue2common.h"
#include "util/graph_range.h"
#include "util/ue2_containers.h"

namespace ue2 {

/**
 * \brief BGL graph type for the undirected NFA graph.
 *
 * Note that we use a set for the out-edge lists: this avoids the construction
 * of parallel edges. The only vertex property constructed is \a
 * vertex_index_t.
 */
typedef boost::adjacency_list<boost::setS,        // out edges
                              boost::listS,       // vertices
                              boost::undirectedS, // graph is undirected
                              boost::property<boost::vertex_index_t, u32> >
NFAUndirectedGraph;

typedef NFAUndirectedGraph::vertex_descriptor NFAUndirectedVertex;

/**
 * Make a copy of an NFAGraph with undirected edges, optionally without start
 * vertices. Mappings from the original graph to the new one are provided.
 *
 * Note that new vertex indices are assigned contiguously in \a vertices(g) order.
 */
template <typename GraphT>
void createUnGraph(const GraphT &g,
                   bool excludeStarts,
                   bool excludeAccepts,
                   NFAUndirectedGraph &ug,
                   ue2::unordered_map<NFAVertex, NFAUndirectedVertex> &old2new,
                   ue2::unordered_map<u32, NFAVertex> &newIdx2old) {
    u32 idx = 0;

    for (auto v : ue2::vertices_range(g)) {
        // skip all accept nodes
        if (excludeAccepts && is_any_accept(v, g)) {
            continue;
        }

        // skip starts if required
        if (excludeStarts && is_any_start(v, g)) {
            continue;
        }

        NFAUndirectedVertex nuv = boost::add_vertex(ug);
        old2new[v] = nuv;
        newIdx2old[idx] = v;
        boost::put(boost::vertex_index, ug, nuv, idx++);
    }

    for (const auto &e : ue2::edges_range(g)) {
        NFAVertex src = source(e, g);
        NFAVertex targ = target(e, g);

        if ((excludeAccepts && is_any_accept(src, g))
            || (excludeStarts && is_any_start(src, g))) {
            continue;
        }

        if ((excludeAccepts && is_any_accept(targ, g))
            || (excludeStarts && is_any_start(targ, g))) {
            continue;
        }

        NFAUndirectedVertex new_src = old2new[src];
        NFAUndirectedVertex new_targ = old2new[targ];

        boost::add_edge(new_src, new_targ, ug);
    }
}

} // namespace ue2

#endif /* NG_UNDIRECTED_H_CB42C71CF38E3D */
