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
 * \brief Functions for graph manipulation that aren't in the base BGL toolkit.
 */

#ifndef UTIL_GRAPH_H
#define UTIL_GRAPH_H

#include "container.h"
#include "ue2common.h"
#include "util/graph_range.h"
#include "util/ue2_containers.h"

#include <boost/graph/adjacency_iterator.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/graph_traits.hpp>

namespace ue2 {

/** \brief True if the given vertex has no out-edges. */
template<class Graph>
bool isLeafNode(const typename Graph::vertex_descriptor& v, const Graph& g) {
    typename Graph::adjacency_iterator ai, ae;
    std::tie(ai, ae) = adjacent_vertices(v, g);
    return ai == ae; // no out edges
}

/** \brief True if the out-degree of vertex \a v is greater than the given
 * limit. */
template<class Graph>
bool hasGreaterOutDegree(size_t limit,
                         const typename Graph::vertex_descriptor& v,
                         const Graph& g) {
    typename Graph::out_edge_iterator ei, ee;
    for (std::tie(ei, ee) = out_edges(v, g); ei != ee; ++ei) {
        if (limit-- == 0) {
            return true;
        }
    }
    return false;
}

/** \brief Returns true if the in-degree of vertex \a v is greater than the
 * given limit. */
template<class Graph>
bool hasGreaterInDegree(size_t limit,
                        const typename Graph::vertex_descriptor& v,
                        const Graph& g) {
    typename Graph::in_edge_iterator ei, ee;
    for (std::tie(ei, ee) = in_edges(v, g); ei != ee; ++ei) {
        if (limit-- == 0) {
            return true;
        }
    }
    return false;
}

/**
 * \brief True if the degree of vertex \a v is greater than the given limit.
 */
template <class Graph>
bool has_greater_degree(size_t limit,
                        const typename Graph::vertex_descriptor &v,
                        const Graph &g) {
    typename Graph::in_edge_iterator ei, ee;
    for (std::tie(ei, ee) = in_edges(v, g); ei != ee; ++ei) {
        if (limit-- == 0) {
            return true;
        }
    }
    typename Graph::out_edge_iterator oi, oe;
    for (std::tie(oi, oe) = out_edges(v, g); oi != oe; ++oi) {
        if (limit-- == 0) {
            return true;
        }
    }
    return false;
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

/** \brief Returns true iff the in-degree of vertex \a v is \a expected */
template<class Graph>
bool in_degree_equal_to(const typename Graph::vertex_descriptor &v,
                        const Graph &g, size_t expected) {
    size_t seen = 0;
    typename Graph::in_edge_iterator ei, ee;
    for (std::tie(ei, ee) = in_edges(v, g);; ++ei, seen++) {
        if (seen == expected) {
            return ei == ee;
        }
        if (ei == ee) {
            return false;
        }
    }
}

/** \brief same as edge(s, t, g) by finds edge by inspecting in-edges of target.
 * Should be used when it is known that t has a small in-degree and when s
 * may have a large out-degree.
 */
template<class Graph>
std::pair<typename Graph::edge_descriptor, bool>
edge_by_target(const typename Graph::vertex_descriptor &s,
               const typename Graph::vertex_descriptor &t, const Graph &g) {
    typename Graph::in_edge_iterator ei, ee;
    for (std::tie(ei, ee) = in_edges(t, g); ei != ee; ++ei) {
        if (source(*ei, g) == s) {
            return std::make_pair(*ei, true);
        }
    }

    return std::make_pair(typename Graph::edge_descriptor(), false);
}


/** \brief True if vertex \a v has at least one successor. */
template<class Graph>
bool has_successor(const typename Graph::vertex_descriptor &v, const Graph &g) {
    typename Graph::adjacency_iterator ai, ae;
    std::tie(ai, ae) = adjacent_vertices(v, g);

    return ai != ae;
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

/** \brief A version of clear_vertex that explicitly removes in- and out-edges
 * for vertex \a v. For many graphs, this is faster than the BGL clear_vertex
 * function, which walks the graph's full edge list. */
template <class Graph>
void clear_vertex_faster(typename Graph::vertex_descriptor v, Graph &g) {
    typename Graph::in_edge_iterator ei, ee;
    tie(ei, ee) = in_edges(v, g);
    while (ei != ee) {
        remove_edge(*ei++, g);
    }

    typename Graph::out_edge_iterator oi, oe;
    tie(oi, oe) = out_edges(v, g);
    while (oi != oe) {
        // NOTE: version that takes out_edge_iterator is faster according to
        // the BGL docs.
        remove_edge(oi++, g);
    }
}

/** \brief Find the set of vertices that are reachable from the vertices in \a
 * sources. */
template<class Graph, class SourceCont, class OutCont>
void find_reachable(const Graph &g, const SourceCont &sources, OutCont *out) {
    using vertex_descriptor = typename Graph::vertex_descriptor;
    ue2::unordered_map<vertex_descriptor, boost::default_color_type> colours;

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
    ue2::unordered_set<vertex_descriptor> reachable;

    find_reachable(g, sources, &reachable);

    for (const auto &v : vertices_range(g)) {
        if (!contains(reachable, v)) {
            out->insert(v);
        }
    }
}

template <class Graph>
bool has_parallel_edge(const Graph &g) {
    using vertex_descriptor = typename Graph::vertex_descriptor;
    ue2::unordered_set<std::pair<vertex_descriptor, vertex_descriptor>> seen;
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

} // namespace ue2

#endif // UTIL_GRAPH_H
