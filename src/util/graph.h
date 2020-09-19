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
 * \brief Functions for graph manipulation that aren't in the base BGL toolkit.
 */

#ifndef UTIL_GRAPH_H
#define UTIL_GRAPH_H

#include "container.h"
#include "ue2common.h"
#include "util/flat_containers.h"
#include "util/graph_range.h"
#include "util/unordered.h"

#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/strong_components.hpp>
#include <boost/range/adaptor/map.hpp>

#include <algorithm>
#include <map>
#include <set>
#include <utility>
#include <vector>

namespace ue2 {

/** \brief True if the given vertex has no out-edges. */
template<class Graph>
bool isLeafNode(const typename Graph::vertex_descriptor& v, const Graph& g) {
    return out_degree(v, g) == 0;
}

/** \brief True if vertex \a v has an edge to itself. */
template<class Graph>
bool hasSelfLoop(const typename Graph::vertex_descriptor &v, const Graph &g) {
    return edge(v, v, g).second;
}

/** \brief True if any vertex in [it, end) has an edge to itself. */
template<class Graph, class Iterator>
bool anySelfLoop(const Graph &g, Iterator it, const Iterator &end) {
    for (; it != end; ++it) {
        if (hasSelfLoop(*it, g)) {
            return true;
        }
    }

    return false;
}

/** \brief Returns the out-degree of vertex \a v, ignoring self-loops. */
template<class Graph>
size_t proper_out_degree(const typename Graph::vertex_descriptor &v,
                         const Graph &g) {
    return out_degree(v, g) - (edge(v, v, g).second ? 1 : 0);
}

/** \brief Returns the in-degree of vertex \a v, ignoring self-loops. */
template<class Graph>
size_t proper_in_degree(const typename Graph::vertex_descriptor &v,
                        const Graph &g) {
    return in_degree(v, g) - (edge(v, v, g).second ? 1 : 0);
}

/** \brief True if vertex \a v has at least one successor. */
template<class Graph>
bool has_successor(const typename Graph::vertex_descriptor &v, const Graph &g) {
    return out_degree(v, g) > 0;
}

/** \brief True if vertex \a v has at least one successor other than itself. */
template<class Graph>
bool has_proper_successor(const typename Graph::vertex_descriptor &v,
                          const Graph &g) {
    typename Graph::adjacency_iterator ai, ae;
    std::tie(ai, ae) = adjacent_vertices(v, g);
    if (ai == ae) {
        return false;
    }
    if (*ai == v) {
        ++ai; // skip self-loop
    }

    return ai != ae;
}

/** \brief Find the set of vertices that are reachable from the vertices in \a
 * sources. */
template<class Graph, class SourceCont, class OutCont>
void find_reachable(const Graph &g, const SourceCont &sources, OutCont *out) {
    using vertex_descriptor = typename Graph::vertex_descriptor;
    std::unordered_map<vertex_descriptor, boost::default_color_type> colours;

    for (auto v : sources) {
        boost::depth_first_visit(g, v,
                                 boost::make_dfs_visitor(boost::null_visitor()),
                                 boost::make_assoc_property_map(colours));
    }

    for (const auto &e : colours) {
        out->insert(e.first);
    }
}

/** \brief Find the set of vertices that are NOT reachable from the vertices in
 * \a sources. */
template<class Graph, class SourceCont, class OutCont>
void find_unreachable(const Graph &g, const SourceCont &sources, OutCont *out) {
    using vertex_descriptor = typename Graph::vertex_descriptor;
    std::unordered_set<vertex_descriptor> reachable;

    find_reachable(g, sources, &reachable);

    for (const auto &v : vertices_range(g)) {
        if (!contains(reachable, v)) {
            out->insert(v);
        }
    }
}

template <class Graph>
flat_set<typename Graph::vertex_descriptor>
find_vertices_in_cycles(const Graph &g) {
    using vertex_descriptor = typename Graph::vertex_descriptor;

    std::map<vertex_descriptor, size_t> comp_map;

    boost::strong_components(g, boost::make_assoc_property_map(comp_map));

    std::map<size_t, std::vector<vertex_descriptor>> comps;

    for (const auto &e : comp_map) {
        comps[e.second].push_back(e.first);
    }

    flat_set<vertex_descriptor> rv;

    for (const auto &comp : comps | boost::adaptors::map_values) {
        /* every vertex in a strongly connected component is reachable from
         * every other vertex in the component. A vertex is involved in a cycle
         * therefore if it is in a strongly connected component with more than
         * one vertex or if it is the only vertex and it has a self loop. */
        assert(!comp.empty());
        if (comp.size() > 1) {
            insert(&rv, comp);
            continue;
        }
        vertex_descriptor v = *comp.begin();
        if (hasSelfLoop(v, g)) {
            rv.insert(v);
        }
    }

    return rv;
}

template <class Graph>
bool has_parallel_edge(const Graph &g) {
    using vertex_descriptor = typename Graph::vertex_descriptor;
    ue2_unordered_set<std::pair<vertex_descriptor, vertex_descriptor>> seen;

    for (const auto &e : edges_range(g)) {
        auto u = source(e, g);
        auto v = target(e, g);
        if (!seen.emplace(u, v).second) {
            return true;
        }
    }
    return false;
}

struct found_back_edge {};
struct detect_back_edges : public boost::default_dfs_visitor {
    explicit detect_back_edges(bool ignore_self_in)
        : ignore_self(ignore_self_in) {}
    template <class Graph>
    void back_edge(const typename Graph::edge_descriptor &e,
                   const Graph &g) const {
        if (ignore_self && source(e, g) == target(e, g)) {
            return;
        }
        throw found_back_edge();
    }
    bool ignore_self;
};

template <class Graph>
bool is_dag(const Graph &g, bool ignore_self_loops = false) {
    try {
        depth_first_search(g, visitor(detect_back_edges(ignore_self_loops)));
    } catch (const found_back_edge &) {
        return false;
    }

    return true;
}

template<typename Cont>
class vertex_recorder : public boost::default_dfs_visitor {
public:
    explicit vertex_recorder(Cont &o) : out(o) {}
    template<class G>
    void discover_vertex(typename Cont::value_type v, const G &) {
        out.insert(v);
    }
    Cont &out;
};

template<typename Cont>
vertex_recorder<Cont> make_vertex_recorder(Cont &o) {
    return vertex_recorder<Cont>(o);
}

/**
 * \brief A vertex recorder visitor that sets the bits in the given bitset
 * type (e.g. boost::dynamic_bitset) corresponding to the indices of the
 * vertices encountered.
 */
template<typename Bitset>
class vertex_index_bitset_recorder : public boost::default_dfs_visitor {
public:
    explicit vertex_index_bitset_recorder(Bitset &o) : out(o) {}
    template<class Graph>
    void discover_vertex(typename Graph::vertex_descriptor v, const Graph &g) {
        assert(g[v].index < out.size());
        out.set(g[v].index);
    }
    Bitset &out;
};

template<typename Bitset>
vertex_index_bitset_recorder<Bitset>
make_vertex_index_bitset_recorder(Bitset &o) {
    return vertex_index_bitset_recorder<Bitset>(o);
}

template <class Graph>
std::pair<typename Graph::edge_descriptor, bool>
add_edge_if_not_present(typename Graph::vertex_descriptor u,
                        typename Graph::vertex_descriptor v, Graph &g) {
    std::pair<typename Graph::edge_descriptor, bool> e = edge(u, v, g);
    if (!e.second) {
        e = add_edge(u, v, g);
    }
    return e;
}

template <class Graph>
std::pair<typename Graph::edge_descriptor, bool> add_edge_if_not_present(
    typename Graph::vertex_descriptor u, typename Graph::vertex_descriptor v,
    const typename Graph::edge_property_type &prop, Graph &g) {
    std::pair<typename Graph::edge_descriptor, bool> e = edge(u, v, g);
    if (!e.second) {
        e = add_edge(u, v, prop, g);
    }
    return e;
}

#ifndef NDEBUG

template <class Graph>
bool hasCorrectlyNumberedVertices(const Graph &g) {
    auto count = num_vertices(g);
    std::vector<bool> ids(count, false);
    for (auto v : vertices_range(g)) {
        auto id = g[v].index;
        if (id >= count || ids[id]) {
            return false; // duplicate
        }
        ids[id] = true;
    }
    return std::find(ids.begin(), ids.end(), false) == ids.end()
        && count == vertex_index_upper_bound(g);
}

template <class Graph>
bool hasCorrectlyNumberedEdges(const Graph &g) {
    auto count = num_edges(g);
    std::vector<bool> ids(count, false);
    for (const auto &e : edges_range(g)) {
        auto id = g[e].index;
        if (id >= count || ids[id]) {
            return false; // duplicate
        }
        ids[id] = true;
    }
    return std::find(ids.begin(), ids.end(), false) == ids.end()
        && count == edge_index_upper_bound(g);
}

#endif

} // namespace ue2

#endif // UTIL_GRAPH_H
