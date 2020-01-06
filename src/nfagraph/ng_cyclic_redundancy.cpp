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
 * \brief Cyclic Path Redundancy pass. Removes redundant vertices on paths
 * leading to a cyclic repeat.
 *
 * This is a graph reduction pass intended to remove vertices that are
 * redundant because they lead solely to a cyclic vertex with a superset of
 * their character reachability. For example, in this pattern:
 *
 *     /(abc|def|abcghi).*0123/s
 *
 * The vertices for 'ghi' can be removed due to the presence of the dot-star
 * repeat.
 *
 * Algorithm:
 *
 *     for each cyclic vertex V:
 *       for each proper predecessor U of V:
 *         let S be the set of successors of U that are successors of V
 *                           (including V itself)
 *         for each successor W of U not in S:
 *           perform a DFS forward from W, stopping exploration when a vertex
 *                           in S is encountered;
 *           if a vertex with reach not in reach(V) or an accept is encountered:
 *             fail and continue to the next W.
 *           else:
 *             remove (U, W)
 *
 * NOTE: the following code is templated not just for fun, but so that we can
 * run this analysis both forward and in reverse over the graph.
 */
#include "ng_cyclic_redundancy.h"

#include "ng_holder.h"
#include "ng_prune.h"
#include "ng_util.h"
#include "util/container.h"
#include "util/flat_containers.h"
#include "util/graph_range.h"
#include "util/graph_small_color_map.h"

#include <algorithm>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/reverse_graph.hpp>

using namespace std;
using boost::reverse_graph;

namespace ue2 {

namespace {

// Terminator function for depth first traversal, tells us not to explore
// beyond vertices in set S.
template<class Vertex, class Graph>
class VertexInSet {
    public:
        explicit VertexInSet(const flat_set<Vertex> &s) : verts(s) {}
        bool operator()(const Vertex &v, const Graph&) const {
            return contains(verts, v);
        }

    private:
        const flat_set<Vertex> &verts;
};

struct SearchFailed {};

// Visitor for depth first traversal, throws an error if we encounter a vertex
// with bad reach or a report.
class SearchVisitor : public boost::default_dfs_visitor {
    public:
        explicit SearchVisitor(const CharReach &r) : cr(r) {}

        template<class Vertex, class Graph>
        void discover_vertex(const Vertex &v, const Graph &g) const {
            DEBUG_PRINTF("vertex %zu\n", g[v].index);
            if (is_special(v, g)) {
                DEBUG_PRINTF("start or accept\n");
                throw SearchFailed();
            }

            if (g[v].assert_flags) {
                DEBUG_PRINTF("assert flags\n");
                throw SearchFailed();
            }

            const CharReach &vcr = g[v].char_reach;
            if (vcr != (vcr & cr)) {
                DEBUG_PRINTF("bad reach\n");
                throw SearchFailed();
            }
        }

    private:
        const CharReach &cr;
};

} // namespace

template<class Graph, class ColorMap>
static
bool searchForward(const Graph &g, const CharReach &reach,
                   ColorMap &colours,
                   const flat_set<typename Graph::vertex_descriptor> &s,
                   typename Graph::vertex_descriptor w) {
    colours.fill(small_color::white);
    try {
        depth_first_visit(g, w, SearchVisitor(reach), colours,
            VertexInSet<typename Graph::vertex_descriptor, Graph>(s));
    } catch (SearchFailed &) {
        return false;
    }

    return true;
}

static
NFAEdge to_raw(const NFAEdge &e, const NGHolder &) {
    return e;
}

static
NFAEdge to_raw(const reverse_graph<NGHolder, NGHolder &>::edge_descriptor &e,
               const reverse_graph<NGHolder, NGHolder &> &g) {
    return get(boost::edge_underlying, g, e);
}

/* returns true if we did stuff */
template<class Graph>
static
bool removeCyclicPathRedundancy(Graph &g, typename Graph::vertex_descriptor v,
                                NGHolder &raw) {
    bool did_stuff = false;

    const CharReach &reach = g[v].char_reach;

    typedef typename Graph::vertex_descriptor vertex_descriptor;

    // Colour map used for depth_first_visit().
    auto colours = make_small_color_map(g);

    // precalc successors of v.
    flat_set<vertex_descriptor> succ_v;
    insert(&succ_v, adjacent_vertices(v, g));

    flat_set<vertex_descriptor> s;

    for (const auto &e : in_edges_range(v, g)) {
        vertex_descriptor u = source(e, g);
        if (u == v) {
            continue;
        }
        if (is_any_accept(u, g)) {
            continue;
        }

        DEBUG_PRINTF("- checking u %zu\n", g[u].index);

        // let s be intersection(succ(u), succ(v))
        s.clear();
        for (auto b : adjacent_vertices_range(u, g)) {
            if (contains(succ_v, b)) {
                s.insert(b);
            }
        }

        for (const auto &e_u : make_vector_from(out_edges(u, g))) {
            vertex_descriptor w = target(e_u, g);
            if (is_special(w, g) || contains(s, w)) {
                continue;
            }

            const CharReach &w_reach = g[w].char_reach;
            if (!w_reach.isSubsetOf(reach)) {
                continue;
            }

            DEBUG_PRINTF("  - checking w %zu\n", g[w].index);

            if (!searchForward(g, reach, colours, succ_v, w)) {
                continue;
            }

            DEBUG_PRINTF("removing edge (%zu,%zu)\n", g[u].index, g[w].index);
            /* we are currently iterating over the in-edges of v, so it
               would be unwise to remove edges to v. However, */
            assert(w != v); /* as v is in s */
            remove_edge(to_raw(e_u, g), raw);
            did_stuff = true;
        }
    }

    return did_stuff;
}

template<class Graph>
static
bool cyclicPathRedundancyPass(Graph &g, NGHolder &raw) {
    bool did_stuff = false;

    for (auto v : vertices_range(g)) {
        if (is_special(v, g) || !edge(v, v, g).second) {
            continue;
        }

        DEBUG_PRINTF("examining cyclic vertex %zu\n", g[v].index);
        did_stuff |= removeCyclicPathRedundancy(g, v, raw);
    }

    return did_stuff;
}

bool removeCyclicPathRedundancy(NGHolder &g) {
    assert(hasCorrectlyNumberedVertices(g));

    // Forward pass.
    bool f_changed = cyclicPathRedundancyPass(g, g);
    if (f_changed) {
        DEBUG_PRINTF("edges removed by forward pass\n");
        pruneUseless(g);
    }

    // Reverse pass.
    DEBUG_PRINTF("REVERSE PASS\n");
    typedef reverse_graph<NGHolder, NGHolder &> RevGraph;
    RevGraph revg(g);
    bool r_changed = cyclicPathRedundancyPass(revg, g);
    if (r_changed) {
        DEBUG_PRINTF("edges removed by reverse pass\n");
        pruneUseless(g);
    }

    return f_changed || r_changed;
}

} // namespace ue2
