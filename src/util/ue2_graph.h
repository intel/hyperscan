/*
 * Copyright (c) 2016, Intel Corporation
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

#ifndef UE2_GRAPH_H
#define UE2_GRAPH_H

#include "ue2common.h"
#include "util/graph_range.h"

#include <boost/operators.hpp>
#include <boost/functional/hash.hpp>
#include <boost/graph/properties.hpp> /* vertex_index_t, ... */
#include <boost/pending/property.hpp> /* no_property */
#include <boost/property_map/property_map.hpp>
#include <boost/intrusive/list.hpp>
#include <boost/iterator/iterator_adaptor.hpp>
#include <boost/iterator/iterator_facade.hpp>

#include <tuple> /* tie */
#include <utility> /* pair, declval */

/*
 * Basic design of ue2_graph:
 *
 * Fairly standard adjacency list type graph structure. The main internal
 * structures are vertex_node and edge_node.
 *
 * Each vertex_node maintains lists of incoming and outgoing edge_nodes, a
 * serial number and the vertex properties.
 *
 * Each edge_node contains pointers to the source and target vertex as well as
 * the serial number and edge properties.
 *
 * Every time an edge_node or vertex_node is created in the graph, it is given a
 * unique serial number by increasing a private counter in the graph.
 *
 * The main thing to note is that the in and out edge lists are intrusive lists
 * with the edge_node containing the necessary hooks. This means that we can
 * easily convert the edge_node to iterators of the in_edge_list and
 * out_edge_list and remove them from the lists.
 *
 * vertex_descriptor and edge_descriptor structures both just wrap pointers to
 * the relevant node structure along with the serial number. operator<() for the
 * descriptors is overridden to look at the serial member of the node.
 * We do not use:
 *    - the address of the node structure as this would lead to an unstable
 *      ordering of vertices between runs.
 *    - the index field as this would mean that the generation of new index
 *      values (during say renumbering of vertex nodes after removing some
 *      vertices) would potentially reorder vertices and corrupt containers
 *      such as std::set<>.
 * The serial number is copied into the descriptors so that we can still have
 * descriptors in a container (such as set or unordered_set) after removing the
 * underlying node.
 *
 * Hashing of descriptors is based on the serial field for similar reasons.
 *
 *
 *
 * Main differences from boost::adjacency_list<> with listS:
 *
 * (1) Deterministic ordering for vertices and edges
 *     boost::adjacency_list<> uses pointer ordering for vertex_descriptors. As
 *     a result, ordering of vertices and edges between runs is
 *     non-deterministic  unless containers, etc use custom comparators.
 *
 * (2) Proper types for descriptors, etc.
 *     No more void * for vertex_descriptors and trying to use it for the wrong
 *     graph type.
 *
 * (3) Constant time num_edges(), num_vertices(), degree(), in_degree() and
 *     out_degree()
 *     std::list is meant to have constant time in C++11 ::size(), but this is
 *     not always implemented as people want to keep ABI compatibility with
 *     existing C++98 standard libraries (gcc 4.8). As ue2_graph_h uses
 *     intrusive lists rather than std::list this is not an issue for us.
 *
 * (4) Constant time remove_edge(e, g)
 *     ue2_graph uses boost::intrusive_lists internally so we can easily unlink
 *     an edge from the in and out edgelist of its source and target.
 *
 * (5) More efficient edge(u, v, g) and remove_edge(u, v, g)
 *     ue2_graph will check which of u and v has the smallest relevant degree
 *     and use that to search for the edge(s).
 *
 * (6) Automatically populate the index field of vertex and edge bundles.
 *     Saves us from doing it manually. Naturally there is nothing to prevent
 *     the user from stuffing up the index properties later.
 *
 * (7) Different edge iteration order
 *     ue2_graph does not maintain an explicit global edge list, so the
 *     edge_iterator is constructed out of vertex_iterator and
 *     out_edge_iterators by iterating the out_edges of each vertices. This
 *     means that edge iteration order is not insertion order like for
 *     adjacency_list.
 *
 * (8) null_edge()
 *     Because why not?
 *
 * (9) vertex and edge properties must have an index field.
 *     We generally need them so the effort has not been put into specialising
 *     for when they are not present.
 *
 *
 *
 * Possible Future Work:
 *
 * (1) Improve edge(u, v, g) performance
 *     This function sees a fair amount of use and is O(n) in the smallest of
 *     the source out_degree or target in_degree. This could be improved by
 *     changes on of the edge containers to be something similar to a multiset.
 *
 * (2) 'Lie' about the number of edges / vertices
 *
 *     One of the main uses of num_edges() and num_vertices() is to allocate a
 *     vector, etc so that it can be indexed by edge or vertex index. If
 *     num_edges() and num_vertices() returned the appropriate size for such a
 *     vector (at least one more than the largest index), we would be able to
 *     avoid some renumbering operations. Functions would have to be provided to
 *     get the real number of vertices and edges. Having num_vertices() and
 *     num_edges() return an over-estimate is not without precedence in the BGL
 *     - the filtered_graph adaptor does the same thing and is compatible with
 *     various (all?) BGL algorithms. It is not clear that this was done
 *     deliberately for the same reason or because it is difficult for
 *     filtered_graph to get the true counts.
 *
 * (3) Investigate slab/pooled allocation schemes for nodes.
 */

namespace ue2 {

namespace graph_detail {

class graph_base : boost::noncopyable {
};

struct default_edge_property {
    size_t index;
};

struct default_vertex_property {
    size_t index;
};

}

template<typename Graph,
         typename VertexPropertyType = graph_detail::default_vertex_property,
         typename EdgePropertyType = graph_detail::default_edge_property>
class ue2_graph : graph_detail::graph_base {
private:
    struct in_edge_tag { };
    struct out_edge_tag { };

    struct vertex_node;

    using out_edge_hook
       = boost::intrusive::list_base_hook<boost::intrusive::tag<out_edge_tag> >;

    /* in_edge_hook does not use safe mode as during graph destruction we do not
     * maintain the in edge lists */
    using in_edge_hook
       = boost::intrusive::list_base_hook<boost::intrusive::tag<in_edge_tag>,
                   boost::intrusive::link_mode<boost::intrusive::normal_link> >;

    struct edge_node : public out_edge_hook, public in_edge_hook {
        explicit edge_node(u64a serial_in) : serial(serial_in) { }

        vertex_node *source = nullptr;
        vertex_node *target = nullptr;
        const u64a serial; /*< used to order edges. We do not use props.index so
                            * that there is no danger of invalidating sets or
                            * other containers by changing the index due to
                            * renumbering */
        EdgePropertyType props;
    };

    template<typename hook_type> using vertex_edge_list
        = boost::intrusive::list<edge_node,
                                 boost::intrusive::base_hook<hook_type> >;

    struct vertex_node : public boost::intrusive::list_base_hook<> {
        explicit vertex_node(u64a serial_in) : serial(serial_in) { }

        VertexPropertyType props;
        const u64a serial; /*< used to order vertices. We do not use props.index
                            * so that there is no danger of invalidating sets or
                            * other containers by changing the index due to
                            * renumbering */

        /* The incoming edges are not considered owned by the vertex */
        vertex_edge_list<in_edge_hook> in_edge_list;

        /* The out going edges are considered owned by the vertex and
         * need to be freed when the graph is begin destroyed */
        vertex_edge_list<out_edge_hook> out_edge_list;

        /* The destructor only frees memory owned by the vertex and will leave
         * the neighbour's edges in a bad state. If a vertex is being removed
         * (rather than the graph being destroyed), then the more gentle clean
         * up of clear_vertex() is required to be called first */
        ~vertex_node() {
            out_edge_list.clear_and_dispose(delete_disposer());
        }
    };

    struct delete_disposer {
        template<typename T> void operator()(const T *d) const { delete d; }
    };

    struct in_edge_disposer {
        void operator()(edge_node *e) const {
            /* remove from source's out edge list before deleting */
            vertex_node *u = e->source;
            u->out_edge_list.erase(u->out_edge_list.iterator_to(*e));
            delete e;
        }
    };

    struct out_edge_disposer {
        void operator()(edge_node *e) const {
            /* remove from target's in edge list before deleting */
            vertex_node *v = e->target;
            v->in_edge_list.erase(v->in_edge_list.iterator_to(*e));
            delete e;
        }
    };

    using vertices_list_type
        = boost::intrusive::list<vertex_node,
             boost::intrusive::base_hook<boost::intrusive::list_base_hook<> > >;

    vertices_list_type vertices_list;

protected: /* to allow renumbering */
    static const size_t N_SPECIAL_VERTICES = 0; /* override in derived class */
    size_t next_vertex_index = 0;
    size_t next_edge_index = 0;

private:
    size_t graph_edge_count = 0; /* maintained explicitly as we have no global
                                    edge list */

    u64a next_serial = 0;
    u64a new_serial() {
        u64a serial = next_serial++;
        if (!next_serial) {
            /* if we have created enough graph edges/vertices to overflow a u64a
             * we must have spent close to an eternity adding to this graph so
             * something must have gone very wrong and we will not be producing
             * a final bytecode in a reasonable amount of time. Or, more likely,
             * the next_serial value has become corrupt. */
            throw std::overflow_error("too many graph edges/vertices created");
        }
        return serial;
    }
public:
    using vertices_size_type = typename vertices_list_type::size_type;
    using degree_size_type
        = typename vertex_edge_list<out_edge_hook>::size_type;
    using edges_size_type = size_t;

    using vertex_property_type = VertexPropertyType;
    using edge_property_type = EdgePropertyType;

    using graph_bundled = boost::no_property;
    using vertex_bundled = VertexPropertyType;
    using edge_bundled = EdgePropertyType;

    class vertex_descriptor : boost::totally_ordered<vertex_descriptor> {
    public:
        vertex_descriptor() : p(nullptr), serial(0) { }
        explicit vertex_descriptor(vertex_node *pp)
            : p(pp), serial(pp->serial) { }

        operator bool() const { return p; }
        bool operator<(const vertex_descriptor b) const {
            if (p && b.p) {
                 /* no vertices in the same graph can have the same serial */
                assert(p == b.p || serial != b.serial);
                return serial < b.serial;
            } else {
                return p < b.p;
            }
        }
        bool operator==(const vertex_descriptor b) const {
            return p == b.p;
        }

        friend size_t hash_value(vertex_descriptor v) {
            using boost::hash_value;
            return hash_value(v.serial);
        }

    private:
        vertex_node *p;
        u64a serial;
        friend ue2_graph;
    };

    class edge_descriptor : boost::totally_ordered<edge_descriptor> {
    public:
        edge_descriptor() : p(nullptr), serial(0) { }
        explicit edge_descriptor(edge_node *pp) : p(pp), serial(pp->serial) { }

        operator bool() const { return p; }
        bool operator<(const edge_descriptor b) const {
            if (p && b.p) {
                 /* no edges in the same graph can have the same serial */
                assert(p == b.p || serial != b.serial);
                return serial < b.serial;
            } else {
                return p < b.p;
            }
        }
        bool operator==(const edge_descriptor b) const {
            return p == b.p;
        }

        friend size_t hash_value(edge_descriptor e) {
            using boost::hash_value;
            return hash_value(e.serial);
        }

    private:
        edge_node *p;
        u64a serial;
        friend ue2_graph;
    };

private:
    static
    vertex_node *raw(vertex_descriptor v) { return v.p; }

    static
    edge_node *raw(edge_descriptor e) { return e.p; }

    /* Note: apparently, nested class templates cannot be fully specialised but
     * they can be partially specialised. Sigh, ... */
    template<typename BundleType, typename dummy = void>
    struct bundle_key_type {
    };

    template<typename dummy>
    struct bundle_key_type<VertexPropertyType, dummy> {
        using type = vertex_descriptor;
    };

    template<typename dummy>
    struct bundle_key_type<EdgePropertyType, dummy> {
        using type = edge_descriptor;
    };

public:
    class out_edge_iterator : public boost::iterator_adaptor<
        out_edge_iterator,
        typename vertex_edge_list<out_edge_hook>::const_iterator,
        edge_descriptor,
        boost::bidirectional_traversal_tag,
        edge_descriptor> {
        using super = typename out_edge_iterator::iterator_adaptor_;
    public:
        out_edge_iterator() : super() { }
        explicit out_edge_iterator(
            typename vertex_edge_list<out_edge_hook>::const_iterator it)
            : super(it) { }
        edge_descriptor dereference() const {
            /* :( const_cast makes me sad but constness is defined by the graph
             * parameter of bgl api calls */
            return edge_descriptor(const_cast<edge_node *>(&*super::base()));
        }
    };

    class in_edge_iterator : public boost::iterator_adaptor<
        in_edge_iterator,
        typename vertex_edge_list<in_edge_hook>::const_iterator,
        edge_descriptor,
        boost::bidirectional_traversal_tag,
        edge_descriptor> {
        using super = typename in_edge_iterator::iterator_adaptor_;
    public:
        in_edge_iterator() : super() { }
        explicit in_edge_iterator(
            typename vertex_edge_list<in_edge_hook>::const_iterator it)
            : super(it) { }
        edge_descriptor dereference() const {
            /* :( const_cast makes me sad but constness is defined by the graph
             * parameter of bgl api calls */
            return edge_descriptor(const_cast<edge_node *>(&*super::base()));
        }
    };

    class adjacency_iterator : public boost::iterator_adaptor<
        adjacency_iterator,
        out_edge_iterator,
        vertex_descriptor,
        boost::bidirectional_traversal_tag,
        vertex_descriptor> {
        using super = typename adjacency_iterator::iterator_adaptor_;
    public:
        adjacency_iterator(out_edge_iterator a) : super(std::move(a)) { }
        adjacency_iterator() { }

        vertex_descriptor dereference() const {
            return vertex_descriptor(super::base()->p->target);
        }
    };

    class inv_adjacency_iterator : public boost::iterator_adaptor<
        inv_adjacency_iterator,
        in_edge_iterator,
        vertex_descriptor,
        boost::bidirectional_traversal_tag,
        vertex_descriptor> {
        using super = typename inv_adjacency_iterator::iterator_adaptor_;
    public:
        inv_adjacency_iterator(in_edge_iterator a) : super(std::move(a)) { }
        inv_adjacency_iterator() { }

        vertex_descriptor dereference() const {
            return vertex_descriptor(super::base()->p->source);
        }
    };

    class vertex_iterator : public boost::iterator_adaptor<
        vertex_iterator,
        typename vertices_list_type::const_iterator,
        vertex_descriptor,
        boost::bidirectional_traversal_tag,
        vertex_descriptor> {
        using super = typename vertex_iterator::iterator_adaptor_;
    public:
        vertex_iterator() : super() { }
        explicit vertex_iterator(typename vertices_list_type::const_iterator it)
            : super(it) { }
        vertex_descriptor dereference() const {
            /* :( const_cast makes me sad but constness is defined by the graph
             * parameter of bgl api calls */
            return vertex_descriptor(
                       const_cast<vertex_node *>(&*super::base()));
        }
    };

    class edge_iterator : public boost::iterator_facade<
        edge_iterator,
        edge_descriptor,
        boost::forward_traversal_tag, /* TODO: make bidi */
        edge_descriptor> {
    public:
        using main_base_iter_type = vertex_iterator;
        using aux_base_iter_type = out_edge_iterator;

        edge_iterator(main_base_iter_type b, main_base_iter_type e)
            : main(std::move(b)), main_end(std::move(e)) {
            if (main == main_end) {
                return;
            }
            std::tie(aux, aux_end) = out_edges_i(*main);
            while (aux == aux_end) {
                ++main;
                if (main == main_end) {
                    break;
                }
                std::tie(aux, aux_end) = out_edges_i(*main);
            }
        }
        edge_iterator() { }

        friend class boost::iterator_core_access;
        void increment() {
            ++aux;
            while (aux == aux_end) {
                ++main;
                if (main == main_end) {
                    break;
                }
                std::tie(aux, aux_end) = out_edges_i(*main);
            }
        }
        bool equal(const edge_iterator &other) const {
            return main == other.main && (main == main_end || aux == other.aux);
        }
        edge_descriptor dereference() const {
            return *aux;
        }

        main_base_iter_type main;
        main_base_iter_type main_end;
        aux_base_iter_type aux;
        aux_base_iter_type aux_end;
    };

private:
    static
    std::pair<out_edge_iterator, out_edge_iterator>
    out_edges_i(vertex_descriptor v) {
        return {out_edge_iterator(raw(v)->out_edge_list.begin()),
                out_edge_iterator(raw(v)->out_edge_list.end())};
    }

public:
    static
    vertex_descriptor null_vertex() { return vertex_descriptor(); }

    friend
    vertex_descriptor add_vertex(Graph &g) {
        vertex_node *v = new vertex_node(g.new_serial());
        v->props.index = g.next_vertex_index++;
        g.vertices_list.push_back(*v);
        return vertex_descriptor(v);
    }

    friend
    void remove_vertex(vertex_descriptor v, Graph &g) {
        vertex_node *vv = Graph::raw(v);
        assert(vv->in_edge_list.empty());
        assert(vv->out_edge_list.empty());
        g.vertices_list.erase_and_dispose(g.vertices_list.iterator_to(*vv),
                                          delete_disposer());
    }

    friend
    void clear_in_edges(vertex_descriptor v, Graph &g) {
        g.graph_edge_count -= Graph::raw(v)->in_edge_list.size();
        Graph::raw(v)->in_edge_list.clear_and_dispose(in_edge_disposer());
    }

    friend
    void clear_out_edges(vertex_descriptor v, Graph &g) {
        g.graph_edge_count -= Graph::raw(v)->out_edge_list.size();
        Graph::raw(v)->out_edge_list.clear_and_dispose(out_edge_disposer());
    }

    friend
    void clear_vertex(vertex_descriptor v, Graph &g) {
        clear_in_edges(v, g);
        clear_out_edges(v, g);
    }

    /* IncidenceGraph concept functions */

    friend
    vertex_descriptor source(edge_descriptor e, const Graph &) {
        return vertex_descriptor(Graph::raw(e)->source);
    }

    friend
    vertex_descriptor target(edge_descriptor e, const Graph &) {
        return vertex_descriptor(Graph::raw(e)->target);
    }

    friend
    degree_size_type out_degree(vertex_descriptor v, const Graph &) {
        return Graph::raw(v)->out_edge_list.size();
    }

    friend
    std::pair<out_edge_iterator, out_edge_iterator>
    out_edges(vertex_descriptor v, const Graph &) {
        return Graph::out_edges_i(v);
    }

    /* BidirectionalGraph concept functions */

    friend
    degree_size_type in_degree(vertex_descriptor v, const Graph &) {
        return Graph::raw(v)->in_edge_list.size();
    }

    friend
    std::pair<in_edge_iterator, in_edge_iterator>
    in_edges(vertex_descriptor v, const Graph &) {
        return {in_edge_iterator(Graph::raw(v)->in_edge_list.begin()),
                in_edge_iterator(Graph::raw(v)->in_edge_list.end())};
    }

    /* Note: this is defined so that self loops are counted twice - which may or
     * may not be what you want. Actually, you probably don't want this at
     * all. */
    friend
    degree_size_type degree(vertex_descriptor v, const Graph &g) {
        return in_degree(v, g) + out_degree(v, g);
    }

    /* AdjacencyList concept functions */

    friend
    std::pair<adjacency_iterator, adjacency_iterator>
    adjacent_vertices(vertex_descriptor v, const Graph &g) {
        auto out_edge_its = out_edges(v, g);
        return {adjacency_iterator(out_edge_its.first),
                adjacency_iterator(out_edge_its.second)};
    }

    /* AdjacencyMatrix concept functions
     * (Note: complexity guarantee is not met) */

    friend
    std::pair<edge_descriptor, bool> edge(vertex_descriptor u,
                                          vertex_descriptor v, const Graph &g) {
        if (in_degree(v, g) < out_degree(u, g)) {
            for (const edge_descriptor &e : in_edges_range(v, g)) {
                if (source(e, g) == u) {
                    return {e, true};
                }
            }
        } else {
            for (const edge_descriptor &e : out_edges_range(u, g)) {
                if (target(e, g) == v) {
                    return {e, true};
                }
            }
        }

        return {edge_descriptor(), false};
    }

    /* Misc functions that don't actually seem to belong to a formal BGL
       concept. */
    static
    edge_descriptor null_edge() { return edge_descriptor(); }

    friend
    std::pair<inv_adjacency_iterator, inv_adjacency_iterator>
    inv_adjacent_vertices(vertex_descriptor v, const Graph &g) {
        auto in_edge_its = in_edges(v, g);
        return {inv_adjacency_iterator(in_edge_its.first),
                inv_adjacency_iterator(in_edge_its.second)};
    }

    /* MutableGraph concept functions */

    friend
    std::pair<edge_descriptor, bool>
    add_edge(vertex_descriptor u, vertex_descriptor v, Graph &g) {
        bool added = true; /* we always allow parallel edges */
        edge_node *e = new edge_node(g.new_serial());
        e->source = Graph::raw(u);
        e->target = Graph::raw(v);
        e->props.index = g.next_edge_index++;

        Graph::raw(u)->out_edge_list.push_back(*e);
        Graph::raw(v)->in_edge_list.push_back(*e);

        g.graph_edge_count++;
        return {edge_descriptor(e), added};
    }

    friend
    void remove_edge(edge_descriptor e, Graph &g) {
        g.graph_edge_count--;

        vertex_node *u = Graph::raw(source(e, g));
        vertex_node *v = Graph::raw(target(e, g));

        v->in_edge_list.erase(v->in_edge_list.iterator_to(*Graph::raw(e)));
        u->out_edge_list.erase(u->out_edge_list.iterator_to(*Graph::raw(e)));

        delete Graph::raw(e);
    }

    template<class Iter>
    friend
    void remove_edge(Iter it, Graph &g) {
        remove_edge(*it, g);
    }

    template<class Predicate>
    friend
    void remove_out_edge_if(vertex_descriptor v, Predicate pred, Graph &g) {
        out_edge_iterator it, ite;
        std::tie(it, ite) = out_edges(v, g);
        while (it != ite) {
            auto jt = it;
            ++it;
            if (pred(*jt)) {
                remove_edge(*jt, g);
            }
        }
    }

    template<class Predicate>
    friend
    void remove_in_edge_if(vertex_descriptor v, Predicate pred, Graph &g) {
        in_edge_iterator it, ite;
        std::tie(it, ite) = in_edges(v, g);
        while (it != ite) {
            auto jt = it;
            ++it;
            if (pred(*jt)) {
                remove_edge(*jt, g);
            }
        }
    }

    template<class Predicate>
    friend
    void remove_edge_if(Predicate pred, Graph &g) {
        edge_iterator it, ite;
        std::tie(it, ite) = edges(g);
        while (it != ite) {
            auto jt = it;
            ++it;
            if (pred(*jt)) {
                remove_edge(*jt, g);
            }
        }
    }

private:
    /* GCC 4.8 has bugs with lambdas in templated friend functions, so: */
    struct source_match {
        source_match(const vertex_descriptor &uu, const Graph &gg)
            : u(uu), g(gg) { }
        bool operator()(edge_descriptor e) const { return source(e, g) == u; }
        const vertex_descriptor &u;
        const Graph &g;
    };

    struct target_match {
        target_match(const vertex_descriptor &vv, const Graph &gg)
            : v(vv), g(gg) { }
        bool operator()(edge_descriptor e) const { return target(e, g) == v; }
        const vertex_descriptor &v;
        const Graph &g;
    };
public:

    /* Note: (u,v) variant needs to remove all (parallel) edges between (u,v).
     *
     * The edge_descriptor version should be strongly preferred if the
     * edge_descriptor is available.
     */
    friend
    void remove_edge(const vertex_descriptor &u,
                     const vertex_descriptor &v,
                     Graph &g) {
        if (in_degree(v, g) < out_degree(u, g)) {
            remove_in_edge_if(v, source_match(u, g), g);
        } else {
            remove_out_edge_if(u, target_match(v, g), g);
        }
    }

    /* VertexListGraph concept functions */

    friend
    vertices_size_type num_vertices(const Graph &g) {
        return g.vertices_list.size();
    }

    friend
    std::pair<vertex_iterator, vertex_iterator> vertices(const Graph &g) {
        return {vertex_iterator(g.vertices_list.begin()),
                vertex_iterator(g.vertices_list.end())};
    }

    /* EdgeListGraph concept functions (aside from those in IncidenceGraph) */

    friend
    edges_size_type num_edges(const Graph &g) {
        return g.graph_edge_count;
    }

    friend
    std::pair<edge_iterator, edge_iterator> edges(const Graph &g) {
        vertex_iterator vi, ve;
        std::tie(vi, ve) = vertices(g);

        return {edge_iterator(vi, ve), edge_iterator(ve, ve)};
    }

    /* bundled properties functions */

    vertex_property_type &operator[](vertex_descriptor v) {
        return raw(v)->props;
    }

    const vertex_property_type &operator[](vertex_descriptor v) const {
        return raw(v)->props;
    }

    edge_property_type &operator[](edge_descriptor e) {
        return raw(e)->props;
    }

    const edge_property_type &operator[](edge_descriptor e) const {
        return raw(e)->props;
    }

    /* PropertyGraph concept functions & helpers */

    template<typename R, typename P_of>
    struct prop_map : public boost::put_get_helper<R, prop_map<R, P_of> > {
        using value_type = typename std::decay<R>::type;
        using reference = R;
        using key_type = typename bundle_key_type<P_of>::type;

        typedef typename boost::lvalue_property_map_tag category;

        prop_map(value_type P_of::*m_in) : member(m_in) { }

        reference operator[](key_type k) const {
            return Graph::raw(k)->props.*member;
        }
        reference operator()(key_type k) const { return (*this)[k]; }

    private:
        value_type P_of::*member;
    };

    template<typename R>
    struct prop_map_all : public boost::put_get_helper<R, prop_map_all<R> > {
        using value_type = typename std::decay<R>::type;
        using reference = R;
        using key_type = typename bundle_key_type<value_type>::type;

        typedef typename boost::lvalue_property_map_tag category;

        reference operator[](key_type k) const {
            return Graph::raw(k)->props;
        }
        reference operator()(key_type k) const { return (*this)[k]; }
    };

    template<typename P_type, typename P_of>
    friend
    prop_map<P_type &, P_of> get(P_type P_of::*t, Graph &) {
        return prop_map<P_type &, P_of>(t);
    }

    template<typename P_type, typename P_of>
    friend
    prop_map<const P_type &, P_of> get(P_type P_of::*t, const Graph &) {
        return prop_map<const P_type &, P_of>(t);
    }

    /* We can't seem to use auto/decltype returns here as it seems that the
     * templated member functions are not yet visible when the compile is
     * evaluating the decltype for the return value. We could probably work
     * around it by making this a dummy templated function. */
    friend
    prop_map<size_t &, VertexPropertyType>
    get(boost::vertex_index_t, Graph &g) {
        return get(&VertexPropertyType::index, g);
    }

    friend
    prop_map<const size_t &, VertexPropertyType>
    get(boost::vertex_index_t, const Graph &g) {
        return get(&VertexPropertyType::index, g);
    }

    friend
    prop_map<size_t &, EdgePropertyType>
    get(boost::edge_index_t, Graph &g) {
        return get(&EdgePropertyType::index, g);
    }

    friend
    prop_map<const size_t &, EdgePropertyType>
    get(boost::edge_index_t, const Graph &g) {
        return get(&EdgePropertyType::index, g);
    }

    friend
    prop_map_all<VertexPropertyType &> get(boost::vertex_all_t, Graph &) {
        return {};
    }

    friend
    prop_map_all<const VertexPropertyType &> get(boost::vertex_all_t,
                                                 const Graph &) {
        return {};
    }

    friend
    prop_map_all<EdgePropertyType &> get(boost::edge_all_t, Graph &) {
        return {};
    }

    friend
    prop_map_all<const EdgePropertyType &> get(boost::edge_all_t,
                                               const Graph &) {
        return {};
    }

    friend
    prop_map_all<VertexPropertyType &> get(boost::vertex_bundle_t, Graph &) {
        return {};
    }

    friend
    prop_map_all<const VertexPropertyType &> get(boost::vertex_bundle_t,
                                                 const Graph &) {
        return {};
    }

    friend
    prop_map_all<EdgePropertyType &> get(boost::edge_bundle_t, Graph &) {
        return {};
    }

    friend
    prop_map_all<const EdgePropertyType &> get(boost::edge_bundle_t,
                                               const Graph &) {
        return {};
    }

    template<typename Prop, typename K>
    friend
    auto get(Prop p, Graph &g, K key) -> decltype(get(p, g)[key]) {
        return get(p, g)[key];
    }

    template<typename Prop, typename K>
    friend
    auto get(Prop p, const Graph &g, K key) -> decltype(get(p, g)[key]) {
        return get(p, g)[key];
    }

    template<typename Prop, typename K, typename V>
    friend
    void put(Prop p, Graph &g, K key, const V &value) {
        get(p, g)[key] = value;
    }

    /* MutablePropertyGraph concept functions */

    /* Note: add_vertex(g, vp) allocates a next index value for the vertex
     * rather than using the index in vp. i.e., except for in rare coincidences:
     *     g[add_vertex(g, vp)].index != vp.index
     */
    friend
    vertex_descriptor add_vertex(const VertexPropertyType &vp, Graph &g) {
        vertex_descriptor v = add_vertex(g);
        auto i = g[v].index;
        g[v] = vp;
        g[v].index = i;

        return v;
    }

    /* Note: add_edge(u, v, g, vp) allocates a next index value for the edge
     * rather than using the index in ep. i.e., except for in rare coincidences:
     *     g[add_edge(u, v, g, ep)].index != ep.index
     */
    friend
    std::pair<edge_descriptor, bool>
    add_edge(vertex_descriptor u, vertex_descriptor v,
             const EdgePropertyType &ep, Graph &g) {
        auto e = add_edge(u, v, g);
        auto i = g[e.first].index;
        g[e.first] = ep;
        g[e.first].index = i;

        return e;
    }

    /* End MutablePropertyGraph */

    /** Pack the edge index into a contiguous range [ 0, num_edges(g) ). */
    friend
    void renumber_edges(Graph &g) {
        g.next_edge_index = 0;
        for (const auto &e : edges_range(g)) {
            g[e].index = g.next_edge_index++;
        }
    }

    /** Pack the vertex index into a contiguous range [ 0, num_vertices(g) ).
     *  Vertices with indices less than N_SPECIAL_VERTICES are not renumbered.
     */
    friend
    void renumber_vertices(Graph &g) {
        DEBUG_PRINTF("renumbering above %zu\n", Graph::N_SPECIAL_VERTICES);
        g.next_vertex_index = Graph::N_SPECIAL_VERTICES;
        for (const auto &v : vertices_range(g)) {
            if (g[v].index < Graph::N_SPECIAL_VERTICES) {
                continue;
            }

            g[v].index = g.next_vertex_index++;
        }
    }

    /** Returns what the next allocated vertex index will be. This is an upper
     *  on the values of index for vertices (vertex removal means that there may
     *  be gaps). */
    friend
    vertices_size_type vertex_index_upper_bound(const Graph &g) {
        return g.next_vertex_index;
    }

    /** Returns what the next allocated edge index will be. This is an upper on
     *  the values of index for edges (edge removal means that there may be
     *  gaps). */
    friend
    vertices_size_type edge_index_upper_bound(const Graph &g) {
        return g.next_edge_index;
    }

    using directed_category = boost::directed_tag;
    using edge_parallel_category = boost::allow_parallel_edge_tag;
    struct traversal_category :
        public virtual boost::bidirectional_graph_tag,
        public virtual boost::adjacency_graph_tag,
        public virtual boost::vertex_list_graph_tag,
        public virtual boost::edge_list_graph_tag { };

    ue2_graph() = default;

    ue2_graph(ue2_graph &&old)
    : next_vertex_index(old.next_vertex_index),
      next_edge_index(old.next_edge_index),
      graph_edge_count(old.graph_edge_count),
      next_serial(old.next_serial) {
        using std::swap;
        swap(vertices_list, old.vertices_list);
    }

    ue2_graph &operator=(ue2_graph &&old) {
        next_vertex_index = old.next_vertex_index;
        next_edge_index = old.next_edge_index;
        graph_edge_count = old.graph_edge_count;
        next_serial = old.next_serial;
        using std::swap;
        swap(vertices_list, old.vertices_list);
        return *this;
    }

    ~ue2_graph() {
        vertices_list.clear_and_dispose(delete_disposer());
    }
};

using boost::vertex_index;
using boost::edge_index;

}

namespace boost {

/* Install partial specialisation of property_map - this is required for
 * adaptors (like filtered_graph) to know the type of the property maps */
template<typename Graph, typename Prop>
struct property_map<Graph, Prop,
                typename std::enable_if<
                    std::is_base_of<ue2::graph_detail::graph_base, Graph>::value
                 >::type > {
    typedef decltype(get(std::declval<Prop>(),
                         std::declval<Graph &>())) type;
    typedef decltype(get(std::declval<Prop>(),
                         std::declval<const Graph &>())) const_type;
};

}
#endif
