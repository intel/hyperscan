/*
 * Copyright (c) 2018, Intel Corporation
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
 * \brief Adaptor that presents an undirected view of a bidirectional BGL graph.
 *
 * Analogous to the reverse_graph adapter. You can construct one of these for
 * bidirectional graph g with:
 *
 *          auto ug = make_undirected_graph(g);
 *
 * The vertex descriptor type is the same as that of the underlying graph, but
 * the edge descriptor is different.
 */

#ifndef GRAPH_UNDIRECTED_H
#define GRAPH_UNDIRECTED_H

#include "util/operators.h"

#include <boost/graph/adjacency_iterator.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/iterator/iterator_facade.hpp>

#include <type_traits>
#include <utility>

namespace ue2 {

struct undirected_graph_tag {};

template <class BidirectionalGraph, class GraphRef>
class undirected_graph;

namespace undirected_detail {

template <typename BidirectionalGraph>
class undirected_graph_edge_descriptor
    : totally_ordered<undirected_graph_edge_descriptor<BidirectionalGraph>> {
    using base_graph_type = BidirectionalGraph;
    using base_graph_traits = typename boost::graph_traits<base_graph_type>;
    using base_edge_type = typename base_graph_traits::edge_descriptor;
    using base_vertex_type = typename base_graph_traits::vertex_descriptor;

    base_edge_type underlying_edge;
    const base_graph_type *g;
    bool reverse; // if true, reverse vertices in source() and target()

    inline std::pair<base_vertex_type, base_vertex_type>
    canonical_edge() const {
        auto u = std::min(source(underlying_edge, *g),
                          target(underlying_edge, *g));
        auto v = std::max(source(underlying_edge, *g),
                          target(underlying_edge, *g));
        return std::make_pair(u, v);
    }

    template <class BidiGraph, class GraphRef>
    friend class ::ue2::undirected_graph;

public:
    undirected_graph_edge_descriptor() = default;

    undirected_graph_edge_descriptor(base_edge_type edge,
                                     const base_graph_type &g_in,
                                     bool reverse_in)
        : underlying_edge(std::move(edge)), g(&g_in), reverse(reverse_in) {}

    bool operator==(const undirected_graph_edge_descriptor &other) const {
        return canonical_edge() == other.canonical_edge();
    }

    bool operator<(const undirected_graph_edge_descriptor &other) const {
        return canonical_edge() < other.canonical_edge();
    }

    base_vertex_type get_source() const {
        return reverse ? target(underlying_edge, *g)
                       : source(underlying_edge, *g);
    }

    base_vertex_type get_target() const {
        return reverse ? source(underlying_edge, *g)
                       : target(underlying_edge, *g);
    }
};

} // namespace undirected_detail

template <class BidirectionalGraph, class GraphRef = const BidirectionalGraph &>
class undirected_graph {
private:
    using Self = undirected_graph<BidirectionalGraph, GraphRef>;
    using Traits = boost::graph_traits<BidirectionalGraph>;

public:
    using base_type = BidirectionalGraph;
    using base_ref_type = GraphRef;

    explicit undirected_graph(GraphRef g_in) : g(g_in) {}

    // Graph requirements
    using vertex_descriptor = typename Traits::vertex_descriptor;
    using edge_descriptor =
        undirected_detail::undirected_graph_edge_descriptor<base_type>;
    using directed_category = boost::undirected_tag;
    using edge_parallel_category = boost::disallow_parallel_edge_tag;
    using traversal_category = typename Traits::traversal_category;

    // IncidenceGraph requirements

    /**
     * \brief Templated iterator used for out_edge_iterator and
     * in_edge_iterator, depending on the value of Reverse.
     */
    template <bool Reverse>
    class adj_edge_iterator
        : public boost::iterator_facade<
              adj_edge_iterator<Reverse>, edge_descriptor,
              boost::forward_traversal_tag, edge_descriptor> {
        vertex_descriptor u;
        const base_type *g;
        typename Traits::in_edge_iterator in_it;
        typename Traits::out_edge_iterator out_it;
        bool done_in = false;
    public:
        adj_edge_iterator() = default;

        adj_edge_iterator(vertex_descriptor u_in, const base_type &g_in,
                          bool end_iter)
            : u(std::move(u_in)), g(&g_in) {
            auto pi = in_edges(u, *g);
            auto po = out_edges(u, *g);
            if (end_iter) {
                in_it = pi.second;
                out_it = po.second;
                done_in = true;
            } else {
                in_it = pi.first;
                out_it = po.first;
                if (in_it == pi.second) {
                    done_in = true;
                    find_first_valid_out();
                }
            }
        }

    private:
        friend class boost::iterator_core_access;

        void find_first_valid_out() {
            auto out_end = out_edges(u, *g).second;
            for (; out_it != out_end; ++out_it) {
                auto v = target(*out_it, *g);
                if (!edge(v, u, *g).second) {
                    break;
                }
            }
        }

        void increment() {
            if (!done_in) {
                auto in_end = in_edges(u, *g).second;
                assert(in_it != in_end);
                ++in_it;
                if (in_it == in_end) {
                    done_in = true;
                    find_first_valid_out();
                }
            } else {
                ++out_it;
                find_first_valid_out();
            }
        }
        bool equal(const adj_edge_iterator &other) const {
            return in_it == other.in_it && out_it == other.out_it;
        }
        edge_descriptor dereference() const {
            if (done_in) {
                return edge_descriptor(*out_it, *g, Reverse);
            } else {
                return edge_descriptor(*in_it, *g, !Reverse);
            }
        }
    };

    using out_edge_iterator = adj_edge_iterator<false>;
    using in_edge_iterator = adj_edge_iterator<true>;

    using degree_size_type = typename Traits::degree_size_type;

    // AdjacencyGraph requirements
    using adjacency_iterator =
        typename boost::adjacency_iterator_generator<Self, vertex_descriptor,
                                                     out_edge_iterator>::type;
    using inv_adjacency_iterator =
        typename boost::inv_adjacency_iterator_generator<
            Self, vertex_descriptor, in_edge_iterator>::type;

    // VertexListGraph requirements
    using vertex_iterator = typename Traits::vertex_iterator;

    // EdgeListGraph requirements
    enum {
        is_edge_list = std::is_convertible<traversal_category,
                                      boost::edge_list_graph_tag>::value
    };

    /** \brief Iterator used for edges(). */
    class edge_iterator
        : public boost::iterator_facade<edge_iterator, edge_descriptor,
                                        boost::forward_traversal_tag,
                                        edge_descriptor> {
        const base_type *g;
        typename Traits::edge_iterator it;
    public:
        edge_iterator() = default;

        edge_iterator(typename Traits::edge_iterator it_in,
                      const base_type &g_in)
            : g(&g_in), it(std::move(it_in)) {
            find_first_valid_edge();
        }

    private:
        friend class boost::iterator_core_access;

        void find_first_valid_edge() {
            const auto end = edges(*g).second;
            for (; it != end; ++it) {
                const auto &u = source(*it, *g);
                const auto &v = target(*it, *g);
                if (!edge(v, u, *g).second) {
                    break; // No reverse edge, we must visit this one
                }
                if (u <= v) {
                    // We have a reverse edge, but we'll return this one (and
                    // skip the other). Note that (u, u) shouldn't be skipped.
                    break;
                }
            }
        }

        void increment() {
            assert(it != edges(*g).second);
            ++it;
            find_first_valid_edge();
        }
        bool equal(const edge_iterator &other) const {
            return it == other.it;
        }
        edge_descriptor dereference() const {
            return edge_descriptor(*it, *g, false);
        }
    };

    using vertices_size_type = typename Traits::vertices_size_type;
    using edges_size_type = typename Traits::edges_size_type;

    using graph_tag = undirected_graph_tag;

    using vertex_bundle_type =
        typename boost::vertex_bundle_type<base_type>::type;
    using edge_bundle_type = typename boost::edge_bundle_type<base_type>::type;

    vertex_bundle_type &operator[](const vertex_descriptor &d) {
        return const_cast<base_type &>(g)[d];
    }
    const vertex_bundle_type &operator[](const vertex_descriptor &d) const {
        return g[d];
    }

    edge_bundle_type &operator[](const edge_descriptor &d) {
        return const_cast<base_type &>(g)[d.underlying_edge];
    }
    const edge_bundle_type &operator[](const edge_descriptor &d) const {
        return g[d.underlying_edge];
    }

    static vertex_descriptor null_vertex() { return Traits::null_vertex(); }

    // Accessor free functions follow

    friend std::pair<vertex_iterator, vertex_iterator>
    vertices(const undirected_graph &ug) {
        return vertices(ug.g);
    }

    friend std::pair<edge_iterator, edge_iterator>
    edges(const undirected_graph &ug) {
        auto e = edges(ug.g);
        return std::make_pair(edge_iterator(e.first, ug.g),
                              edge_iterator(e.second, ug.g));
    }

    friend std::pair<out_edge_iterator, out_edge_iterator>
    out_edges(const vertex_descriptor &u, const undirected_graph &ug) {
        return std::make_pair(out_edge_iterator(u, ug.g, false),
                              out_edge_iterator(u, ug.g, true));
    }

    friend vertices_size_type num_vertices(const undirected_graph &ug) {
        return num_vertices(ug.g);
    }

    friend edges_size_type num_edges(const undirected_graph &ug) {
        auto p = edges(ug);
        return std::distance(p.first, p.second);
    }

    friend degree_size_type out_degree(const vertex_descriptor &u,
                                       const undirected_graph &ug) {
        return degree(u, ug);
    }

    friend vertex_descriptor vertex(vertices_size_type n,
                                    const undirected_graph &ug) {
        return vertex(n, ug.g);
    }

    friend std::pair<edge_descriptor, bool> edge(const vertex_descriptor &u,
                                                 const vertex_descriptor &v,
                                                 const undirected_graph &ug) {
        auto e = edge(u, v, ug.g);
        if (e.second) {
            return std::make_pair(edge_descriptor(e.first, ug.g, false), true);
        }
        auto e_rev = edge(v, u, ug.g);
        if (e_rev.second) {
            return std::make_pair(edge_descriptor(e_rev.first, ug.g, true),
                                  true);
        }
        return std::make_pair(edge_descriptor(), false);
    }

    friend std::pair<in_edge_iterator, in_edge_iterator>
    in_edges(const vertex_descriptor &v, const undirected_graph &ug) {
        return std::make_pair(in_edge_iterator(v, ug.g, false),
                              in_edge_iterator(v, ug.g, true));
    }

    friend std::pair<adjacency_iterator, adjacency_iterator>
    adjacent_vertices(const vertex_descriptor &u, const undirected_graph &ug) {
        out_edge_iterator oi, oe;
        std::tie(oi, oe) = out_edges(u, ug);
        return std::make_pair(adjacency_iterator(oi, &ug),
                              adjacency_iterator(oe, &ug));
    }

    friend std::pair<inv_adjacency_iterator, inv_adjacency_iterator>
    inv_adjacent_vertices(const vertex_descriptor &v,
                          const undirected_graph &ug) {
        in_edge_iterator ei, ee;
        std::tie(ei, ee) = in_edges(v, ug);
        return std::make_pair(inv_adjacency_iterator(ei, &ug),
                              inv_adjacency_iterator(ee, &ug));
    }

    friend degree_size_type in_degree(const vertex_descriptor &v,
                                      const undirected_graph &ug) {
        return degree(v, ug);
    }

    friend vertex_descriptor source(const edge_descriptor &e,
                                    const undirected_graph &) {
        return e.get_source();
    }

    friend vertex_descriptor target(const edge_descriptor &e,
                                    const undirected_graph &) {
        return e.get_target();
    }

    friend degree_size_type degree(const vertex_descriptor &u,
                                   const undirected_graph &ug) {
        auto p = out_edges(u, ug);
        return std::distance(p.first, p.second);
    }

    // Property accessors.

    template <typename Property>
    using prop_map = typename boost::property_map<undirected_graph, Property>;

    template <typename Property>
    friend typename prop_map<Property>::type
    get(Property p, undirected_graph &ug) {
        return get(p, ug.g);
    }

    template <typename Property>
    friend typename prop_map<Property>::const_type
    get(Property p, const undirected_graph &ug) {
        return get(p, ug.g);
    }

    template <typename Property, typename Key>
    friend typename boost::property_traits<
        typename prop_map<Property>::const_type>::value_type
    get(Property p, const undirected_graph &ug, const Key &k) {
        return get(p, ug.g, get_underlying_descriptor(k));
    }

    template <typename Property, typename Value, typename Key>
    friend void put(Property p, const undirected_graph &ug,
                    const Key &k, const Value &val) {
        put(p, const_cast<BidirectionalGraph &>(ug.g),
            get_underlying_descriptor(k), val);
    }

private:
    // Accessors are here because our free friend functions (above) cannot see
    // edge_descriptor's private members.
    static typename base_type::vertex_descriptor
    get_underlying_descriptor(const vertex_descriptor &v) {
        return v;
    }
    static typename base_type::edge_descriptor
    get_underlying_descriptor(const edge_descriptor &e) {
        return e.underlying_edge;
    }

    // Reference to underlying bidirectional graph
    GraphRef g;
};

template <class BidirectionalGraph>
undirected_graph<BidirectionalGraph>
make_undirected_graph(const BidirectionalGraph &g) {
    return undirected_graph<BidirectionalGraph>(g);
}

} // namespace ue2

namespace boost {

/* Derive all the property map specializations from the underlying
 * bidirectional graph. */

template <typename BidirectionalGraph, typename GraphRef, typename Property>
struct property_map<ue2::undirected_graph<BidirectionalGraph, GraphRef>,
                    Property> {
    using base_map_type = property_map<BidirectionalGraph, Property>;
    using type = typename base_map_type::type;
    using const_type = typename base_map_type::const_type;
};

template <class BidirectionalGraph, class GraphRef>
struct vertex_property_type<ue2::undirected_graph<BidirectionalGraph, GraphRef>>
    : vertex_property_type<BidirectionalGraph> {};

template <class BidirectionalGraph, class GraphRef>
struct edge_property_type<ue2::undirected_graph<BidirectionalGraph, GraphRef>>
    : edge_property_type<BidirectionalGraph> {};

template <class BidirectionalGraph, class GraphRef>
struct graph_property_type<ue2::undirected_graph<BidirectionalGraph, GraphRef>>
    : graph_property_type<BidirectionalGraph> {};

template <typename BidirectionalGraph, typename GraphRef>
struct vertex_bundle_type<ue2::undirected_graph<BidirectionalGraph, GraphRef>>
    : vertex_bundle_type<BidirectionalGraph> {};

template <typename BidirectionalGraph, typename GraphRef>
struct edge_bundle_type<ue2::undirected_graph<BidirectionalGraph, GraphRef>>
    : edge_bundle_type<BidirectionalGraph> {};

template <typename BidirectionalGraph, typename GraphRef>
struct graph_bundle_type<ue2::undirected_graph<BidirectionalGraph, GraphRef>>
    : graph_bundle_type<BidirectionalGraph> {};

} // namespace boost

#endif // GRAPH_UNDIRECTED_H
