/*
 * Copyright (c) 2016-2018, Intel Corporation
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
#include "util/noncopyable.h"
#include "util/operators.h"

#include <boost/graph/properties.hpp> /* vertex_index_t, ... */
#include <boost/pending/property.hpp> /* no_property */
#include <boost/property_map/property_map.hpp>
#include <boost/intrusive/list.hpp>
#include <boost/iterator/iterator_adaptor.hpp>
#include <boost/iterator/iterator_facade.hpp>

#include <functional> /* hash */
#include <tuple> /* tie */
#include <type_traits> /* is_same, etc */
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
 *     non-deterministic unless containers, etc use custom comparators.
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

class graph_base : noncopyable {
};

struct default_edge_property {
    size_t index;
};

struct default_vertex_property {
    size_t index;
};

template<typename Graph>
class vertex_descriptor : totally_ordered<vertex_descriptor<Graph>> {
    using vertex_node = typename Graph::vertex_node;
public:
    vertex_descriptor() : p(nullptr), serial(0) {}
    explicit vertex_descriptor(vertex_node *pp) : p(pp), serial(pp->serial) {}

    explicit operator bool() const { return p; }
    bool operator<(const vertex_descriptor b) const {
        if (p && b.p) {
            /* no vertices in the same graph can have the same serial */
            assert(p == b.p || serial != b.serial);
            return serial < b.serial;
        } else {
            return p < b.p;
        }
    }
    bool operator==(const vertex_descriptor b) const { return p == b.p; }

    size_t hash() const {
        return std::hash<u64a>()(serial);
    }

private:
    vertex_node *raw(void) { return p; }
    vertex_node *p;
    u64a serial;
    friend Graph;
};

template<typename Graph>
class edge_descriptor : totally_ordered<edge_descriptor<Graph>> {
    using edge_node = typename Graph::edge_node;
public:
    edge_descriptor() : p(nullptr), serial(0) {}
    explicit edge_descriptor(edge_node *pp) : p(pp), serial(pp->serial) {}

    /* Convenience ctor to allow us to directly get an edge_descriptor from
     * edge() and add_edge(). As we have null_edges and we always allow
     * parallel edges, the bool component of the return from these functions is
     * not required. */
    edge_descriptor(const std::pair<edge_descriptor, bool> &tup)
        : p(tup.first.p), serial(tup.first.serial) {
        assert(tup.second == (bool)tup.first);
    }

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
    bool operator==(const edge_descriptor b) const { return p == b.p; }

    size_t hash() const {
        return std::hash<u64a>()(serial);
    }

private:
    edge_node *raw(void) { return p; }
    edge_node *p;
    u64a serial;
    friend Graph;
};

} // namespace graph_detail

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
         * need to be freed when the graph is being destroyed */
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
    using vertex_descriptor = graph_detail::vertex_descriptor<ue2_graph>;
    using edge_descriptor = graph_detail::edge_descriptor<ue2_graph>;
    friend vertex_descriptor;
    friend edge_descriptor;

    using vertices_size_type = typename vertices_list_type::size_type;
    using degree_size_type
        = typename vertex_edge_list<out_edge_hook>::size_type;
    using edges_size_type = size_t;

    using vertex_property_type = VertexPropertyType;
    using edge_property_type = EdgePropertyType;

    using graph_bundled = boost::no_property;
    using vertex_bundled = VertexPropertyType;
    using edge_bundled = EdgePropertyType;

private:
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
            std::tie(aux, aux_end) = out_edges_impl(*main);
            while (aux == aux_end) {
                ++main;
                if (main == main_end) {
                    break;
                }
                std::tie(aux, aux_end) = out_edges_impl(*main);
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
                std::tie(aux, aux_end) = out_edges_impl(*main);
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

public:
    static
    vertex_descriptor null_vertex() { return vertex_descriptor(); }

    vertex_descriptor add_vertex_impl() {
        vertex_node *v = new vertex_node(new_serial());
        v->props.index = next_vertex_index++;
        vertices_list.push_back(*v);
        return vertex_descriptor(v);
    }

    void remove_vertex_impl(vertex_descriptor v) {
        vertex_node *vv = v.raw();
        assert(vv->in_edge_list.empty());
        assert(vv->out_edge_list.empty());
        vertices_list.erase_and_dispose(vertices_list.iterator_to(*vv),
                                        delete_disposer());
    }

    void clear_in_edges_impl(vertex_descriptor v) {
        graph_edge_count -= v.raw()->in_edge_list.size();
        v.raw()->in_edge_list.clear_and_dispose(in_edge_disposer());
    }

    void clear_out_edges_impl(vertex_descriptor v) {
        graph_edge_count -= v.raw()->out_edge_list.size();
        v.raw()->out_edge_list.clear_and_dispose(out_edge_disposer());
    }

    /* IncidenceGraph concept functions */

    static
    vertex_descriptor source_impl(edge_descriptor e) {
        return vertex_descriptor(e.raw()->source);
    }

    static
    vertex_descriptor target_impl(edge_descriptor e) {
        return vertex_descriptor(e.raw()->target);
    }

    static
    degree_size_type out_degree_impl(vertex_descriptor v) {
        return v.raw()->out_edge_list.size();
    }

    static
    std::pair<out_edge_iterator, out_edge_iterator>
    out_edges_impl(vertex_descriptor v) {
        return {out_edge_iterator(v.raw()->out_edge_list.begin()),
                out_edge_iterator(v.raw()->out_edge_list.end())};
    }

    /* BidirectionalGraph concept functions */

    static
    degree_size_type in_degree_impl(vertex_descriptor v) {
        return v.raw()->in_edge_list.size();
    }

    static
    std::pair<in_edge_iterator, in_edge_iterator>
    in_edges_impl(vertex_descriptor v) {
        return {in_edge_iterator(v.raw()->in_edge_list.begin()),
                in_edge_iterator(v.raw()->in_edge_list.end())};
    }

    /* Note: this is defined so that self loops are counted twice - which may or
     * may not be what you want. Actually, you probably don't want this at
     * all. */
    static
    degree_size_type degree_impl(vertex_descriptor v) {
        return in_degree_impl(v) + out_degree_impl(v);
    }

    /* AdjacencyList concept functions */

    static
    std::pair<adjacency_iterator, adjacency_iterator>
    adjacent_vertices_impl(vertex_descriptor v) {
        auto out_edge_its = out_edges_impl(v);
        return {adjacency_iterator(out_edge_its.first),
                adjacency_iterator(out_edge_its.second)};
    }

    /* AdjacencyMatrix concept functions
     * (Note: complexity guarantee is not met) */

    std::pair<edge_descriptor, bool> edge_impl(vertex_descriptor u,
                                               vertex_descriptor v) const {
        if (in_degree_impl(v) < out_degree_impl(u)) {
            for (const edge_descriptor &e : in_edges_range(v, *this)) {
                if (source_impl(e) == u) {
                    return {e, true};
                }
            }
        } else {
            for (const edge_descriptor &e : out_edges_range(u, *this)) {
                if (target_impl(e) == v) {
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

    static
    std::pair<inv_adjacency_iterator, inv_adjacency_iterator>
    inv_adjacent_vertices_impl(vertex_descriptor v) {
        auto in_edge_its = in_edges_impl(v);
        return {inv_adjacency_iterator(in_edge_its.first),
                inv_adjacency_iterator(in_edge_its.second)};
    }

    /* MutableGraph concept functions */

    std::pair<edge_descriptor, bool>
    add_edge_impl(vertex_descriptor u, vertex_descriptor v) {
        bool added = true; /* we always allow parallel edges */
        edge_node *e = new edge_node(new_serial());
        e->source = u.raw();
        e->target = v.raw();
        e->props.index = next_edge_index++;

        u.raw()->out_edge_list.push_back(*e);
        v.raw()->in_edge_list.push_back(*e);

        graph_edge_count++;
        return {edge_descriptor(e), added};
    }

    void remove_edge_impl(edge_descriptor e) {
        graph_edge_count--;

        vertex_node *u = e.raw()->source;
        vertex_node *v = e.raw()->target;

        v->in_edge_list.erase(v->in_edge_list.iterator_to(*e.raw()));
        u->out_edge_list.erase(u->out_edge_list.iterator_to(*e.raw()));

        delete e.raw();
    }

    template<class Predicate>
    void remove_out_edge_if_impl(vertex_descriptor v, Predicate pred) {
        out_edge_iterator it, ite;
        std::tie(it, ite) = out_edges_impl(v);
        while (it != ite) {
            auto jt = it;
            ++it;
            if (pred(*jt)) {
                this->remove_edge_impl(*jt);
            }
        }
    }

    template<class Predicate>
    void remove_in_edge_if_impl(vertex_descriptor v, Predicate pred) {
        in_edge_iterator it, ite;
        std::tie(it, ite) = in_edges_impl(v);
        while (it != ite) {
            auto jt = it;
            ++it;
            if (pred(*jt)) {
                remove_edge_impl(*jt);
            }
        }
    }

    template<class Predicate>
    void remove_edge_if_impl(Predicate pred) {
        edge_iterator it, ite;
        std::tie(it, ite) = edges_impl();
        while (it != ite) {
            auto jt = it;
            ++it;
            if (pred(*jt)) {
                remove_edge_impl(*jt);
            }
        }
    }

private:
    /* GCC 4.8 has bugs with lambdas in templated friend functions, so: */
    struct source_match {
        explicit source_match(const vertex_descriptor &uu) : u(uu) { }
        bool operator()(edge_descriptor e) const { return source_impl(e) == u; }
        const vertex_descriptor &u;
    };

    struct target_match {
        explicit target_match(const vertex_descriptor &vv) : v(vv) { }
        bool operator()(edge_descriptor e) const { return target_impl(e) == v; }
        const vertex_descriptor &v;
    };
public:
    /* Note: (u,v) variant needs to remove all (parallel) edges between (u,v).
     *
     * The edge_descriptor version should be strongly preferred if the
     * edge_descriptor is available.
     */
    void remove_edge_impl(const vertex_descriptor &u,
                          const vertex_descriptor &v) {
        if (in_degree_impl(v) < out_degree_impl(u)) {
            remove_in_edge_if_impl(v, source_match(u));
        } else {
            remove_out_edge_if_impl(u, target_match(v));
        }
    }

    /* VertexListGraph concept functions */
    vertices_size_type num_vertices_impl() const {
        return vertices_list.size();
    }

    std::pair<vertex_iterator, vertex_iterator> vertices_impl() const {
        return {vertex_iterator(vertices_list.begin()),
                vertex_iterator(vertices_list.end())};
    }

    /* EdgeListGraph concept functions (aside from those in IncidenceGraph) */

    edges_size_type num_edges_impl() const {
        return graph_edge_count;
    }

    std::pair<edge_iterator, edge_iterator> edges_impl() const {
        vertex_iterator vi, ve;
        std::tie(vi, ve) = vertices_impl();

        return {edge_iterator(vi, ve), edge_iterator(ve, ve)};
    }

    /* bundled properties functions */

    vertex_property_type &operator[](vertex_descriptor v) {
        return v.raw()->props;
    }

    const vertex_property_type &operator[](vertex_descriptor v) const {
        return v.raw()->props;
    }

    edge_property_type &operator[](edge_descriptor e) {
        return e.raw()->props;
    }

    const edge_property_type &operator[](edge_descriptor e) const {
        return e.raw()->props;
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
            return k.raw()->props.*member;
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
            return k.raw()->props;
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
    vertex_descriptor add_vertex_impl(const VertexPropertyType &vp) {
        vertex_descriptor v = add_vertex_impl();
        auto i = (*this)[v].index;
        (*this)[v] = vp;
        (*this)[v].index = i;

        return v;
    }

    /* Note: add_edge(u, v, g, vp) allocates a next index value for the edge
     * rather than using the index in ep. i.e., except for in rare coincidences:
     *     g[add_edge(u, v, g, ep)].index != ep.index
     */
    std::pair<edge_descriptor, bool>
    add_edge_impl(vertex_descriptor u, vertex_descriptor v,
                  const EdgePropertyType &ep) {
        auto e = add_edge_impl(u, v);
        auto i = (*this)[e.first].index;
        (*this)[e.first] = ep;
        (*this)[e.first].index = i;

        return e;
    }

    /* End MutablePropertyGraph */

    /** Pack the edge index into a contiguous range [ 0, num_edges(g) ). */
    void renumber_edges_impl() {
        next_edge_index = 0;
        edge_iterator it;
        edge_iterator ite;
        for (std::tie(it, ite) = edges_impl(); it != ite; ++it) {
            (*this)[*it].index = next_edge_index++;
        }
    }

    /** Pack the vertex index into a contiguous range [ 0, num_vertices(g) ).
     *  Vertices with indices less than N_SPECIAL_VERTICES are not renumbered.
     */
    void renumber_vertices_impl() {
        DEBUG_PRINTF("renumbering above %zu\n", Graph::N_SPECIAL_VERTICES);
        next_vertex_index = Graph::N_SPECIAL_VERTICES;
        vertex_iterator it;
        vertex_iterator ite;
        for (std::tie(it, ite) = vertices_impl(); it != ite; ++it) {
            if ((*this)[*it].index < Graph::N_SPECIAL_VERTICES) {
                continue;
            }

            (*this)[*it].index = next_vertex_index++;
        }
    }

    /** Returns what the next allocated vertex index will be. This is an upper
     *  on the values of index for vertices (vertex removal means that there may
     *  be gaps). */
    vertices_size_type vertex_index_upper_bound_impl() const {
        return next_vertex_index;
    }

    /** Returns what the next allocated edge index will be. This is an upper on
     *  the values of index for edges (edge removal means that there may be
     *  gaps). */
    vertices_size_type edge_index_upper_bound_impl() const {
        return next_edge_index;
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

/** \brief Type trait to enable on whether the Graph is an ue2_graph. */
template<typename Graph>
struct is_ue2_graph
    : public ::std::integral_constant<
          bool, std::is_base_of<graph_detail::graph_base, Graph>::value> {};

template<typename Graph>
typename std::enable_if<is_ue2_graph<Graph>::value,
                        typename Graph::vertex_descriptor>::type
add_vertex(Graph &g) {
    return g.add_vertex_impl();
}

template<typename Graph>
typename std::enable_if<is_ue2_graph<Graph>::value>::type
remove_vertex(typename Graph::vertex_descriptor v, Graph &g) {
    g.remove_vertex_impl(v);
}

template<typename Graph>
typename std::enable_if<is_ue2_graph<Graph>::value>::type
clear_in_edges(typename Graph::vertex_descriptor v, Graph &g) {
    g.clear_in_edges_impl(v);
}

template<typename Graph>
typename std::enable_if<is_ue2_graph<Graph>::value>::type
clear_out_edges(typename Graph::vertex_descriptor v, Graph &g) {
    g.clear_out_edges_impl(v);
}

template<typename Graph>
typename std::enable_if<is_ue2_graph<Graph>::value>::type
clear_vertex(typename Graph::vertex_descriptor v, Graph &g) {
    g.clear_in_edges_impl(v);
    g.clear_out_edges_impl(v);
}

template<typename Graph>
typename std::enable_if<is_ue2_graph<Graph>::value,
                        typename Graph::vertex_descriptor>::type
source(typename Graph::edge_descriptor e, const Graph &) {
    return Graph::source_impl(e);
}

template<typename Graph>
typename std::enable_if<is_ue2_graph<Graph>::value,
                        typename Graph::vertex_descriptor>::type
target(typename Graph::edge_descriptor e, const Graph &) {
    return Graph::target_impl(e);
}

template<typename Graph>
typename std::enable_if<is_ue2_graph<Graph>::value,
                        typename Graph::degree_size_type>::type
out_degree(typename Graph::vertex_descriptor v, const Graph &) {
    return Graph::out_degree_impl(v);
}

template<typename Graph>
typename std::enable_if<is_ue2_graph<Graph>::value,
                        std::pair<typename Graph::out_edge_iterator,
                                  typename Graph::out_edge_iterator>>::type
out_edges(typename Graph::vertex_descriptor v, const Graph &) {
    return Graph::out_edges_impl(v);
}

template<typename Graph>
typename std::enable_if<is_ue2_graph<Graph>::value,
                        typename Graph::degree_size_type>::type
in_degree(typename Graph::vertex_descriptor v, const Graph &) {
    return Graph::in_degree_impl(v);
}

template<typename Graph>
typename std::enable_if<is_ue2_graph<Graph>::value,
                        std::pair<typename Graph::in_edge_iterator,
                                  typename Graph::in_edge_iterator>>::type
in_edges(typename Graph::vertex_descriptor v, const Graph &) {
    return Graph::in_edges_impl(v);
}

template<typename Graph>
typename std::enable_if<is_ue2_graph<Graph>::value,
                        typename Graph::degree_size_type>::type
degree(typename Graph::vertex_descriptor v, const Graph &) {
    return Graph::degree_impl(v);
}

template<typename Graph>
typename std::enable_if<is_ue2_graph<Graph>::value,
                        std::pair<typename Graph::adjacency_iterator,
                                  typename Graph::adjacency_iterator>>::type
adjacent_vertices(typename Graph::vertex_descriptor v, const Graph &) {
    return Graph::adjacent_vertices_impl(v);
}

template<typename Graph>
typename std::enable_if<is_ue2_graph<Graph>::value,
                        std::pair<typename Graph::edge_descriptor, bool>>::type
edge(typename Graph::vertex_descriptor u, typename Graph::vertex_descriptor v,
     const Graph &g) {
    return g.edge_impl(u, v);
}

template<typename Graph>
typename std::enable_if<is_ue2_graph<Graph>::value,
                        std::pair<typename Graph::inv_adjacency_iterator,
                                  typename Graph::inv_adjacency_iterator>>::type
inv_adjacent_vertices(typename Graph::vertex_descriptor v, const Graph &) {
    return Graph::inv_adjacent_vertices_impl(v);
}

template<typename Graph>
typename std::enable_if<is_ue2_graph<Graph>::value,
                        std::pair<typename Graph::edge_descriptor, bool>>::type
add_edge(typename Graph::vertex_descriptor u,
         typename Graph::vertex_descriptor v, Graph &g) {
    return g.add_edge_impl(u, v);
}

template<typename Graph>
typename std::enable_if<is_ue2_graph<Graph>::value>::type
remove_edge(typename Graph::edge_descriptor e, Graph &g) {
    g.remove_edge_impl(e);
}

template<typename Graph, typename Iter>
typename std::enable_if<
    !std::is_convertible<Iter, typename Graph::edge_descriptor>::value &&
    is_ue2_graph<Graph>::value>::type
remove_edge(Iter it, Graph &g) {
    g.remove_edge_impl(*it);
}

template<typename Graph, typename Predicate>
typename std::enable_if<is_ue2_graph<Graph>::value>::type
remove_out_edge_if(typename Graph::vertex_descriptor v, Predicate pred,
                   Graph &g) {
    g.remove_out_edge_if_impl(v, pred);
}

template<typename Graph, typename Predicate>
typename std::enable_if<is_ue2_graph<Graph>::value>::type
remove_in_edge_if(typename Graph::vertex_descriptor v, Predicate pred,
                  Graph &g) {
    g.remove_in_edge_if_impl(v, pred);
}

template<typename Graph, typename Predicate>
typename std::enable_if<is_ue2_graph<Graph>::value>::type
remove_edge_if(Predicate pred, Graph &g) {
    g.remove_edge_if_impl(pred);
}

template<typename Graph>
typename std::enable_if<is_ue2_graph<Graph>::value>::type
remove_edge(const typename Graph::vertex_descriptor &u,
            const typename Graph::vertex_descriptor &v, Graph &g) {
    g.remove_edge_impl(u, v);
}

template<typename Graph>
typename std::enable_if<is_ue2_graph<Graph>::value,
                        typename Graph::vertices_size_type>::type
num_vertices(const Graph &g) {
    return g.num_vertices_impl();
}

template<typename Graph>
typename std::enable_if<is_ue2_graph<Graph>::value,
                        std::pair<typename Graph::vertex_iterator,
                                  typename Graph::vertex_iterator>>::type
vertices(const Graph &g) {
    return g.vertices_impl();
}

template<typename Graph>
typename std::enable_if<is_ue2_graph<Graph>::value,
                        typename Graph::edges_size_type>::type
num_edges(const Graph &g) {
    return g.num_edges_impl();
}

template<typename Graph>
typename std::enable_if<is_ue2_graph<Graph>::value,
                        std::pair<typename Graph::edge_iterator,
                                  typename Graph::edge_iterator>>::type
edges(const Graph &g) {
    return g.edges_impl();
}

template<typename Graph>
typename std::enable_if<is_ue2_graph<Graph>::value,
                        typename Graph::vertex_descriptor>::type
add_vertex(const typename Graph::vertex_property_type &vp, Graph &g) {
    return g.add_vertex_impl(vp);
}

template<typename Graph>
typename std::enable_if<is_ue2_graph<Graph>::value,
                        std::pair<typename Graph::edge_descriptor, bool>>::type
add_edge(typename Graph::vertex_descriptor u,
         typename Graph::vertex_descriptor v,
         const typename Graph::edge_property_type &ep, Graph &g) {
    return g.add_edge_impl(u, v, ep);
}

template<typename Graph>
typename std::enable_if<is_ue2_graph<Graph>::value>::type
renumber_edges(Graph &g) {
    g.renumber_edges_impl();
}

template<typename Graph>
typename std::enable_if<is_ue2_graph<Graph>::value>::type
renumber_vertices(Graph &g) {
    g.renumber_vertices_impl();
}

template<typename Graph>
typename std::enable_if<is_ue2_graph<Graph>::value,
                        typename Graph::vertices_size_type>::type
vertex_index_upper_bound(const Graph &g) {
    return g.vertex_index_upper_bound_impl();
}

template<typename Graph>
typename std::enable_if<is_ue2_graph<Graph>::value,
                        typename Graph::edges_size_type>::type
edge_index_upper_bound(const Graph &g) {
    return g.edge_index_upper_bound_impl();
}

template<typename T> struct pointer_to_member_traits {};

template<typename Return, typename Class>
struct pointer_to_member_traits<Return(Class::*)> {
    using member_type = Return;
    using class_type = Class;
};

template<typename Graph, typename Property, typename Enable = void>
struct is_ue2_vertex_or_edge_property {
    static constexpr bool value = false;
};

template<typename Graph, typename Property>
struct is_ue2_vertex_or_edge_property<
    Graph, Property, typename std::enable_if<is_ue2_graph<Graph>::value &&
                                             std::is_member_object_pointer<
                                                 Property>::value>::type> {
private:
    using class_type = typename pointer_to_member_traits<Property>::class_type;
    using vertex_type = typename Graph::vertex_property_type;
    using edge_type = typename Graph::edge_property_type;
public:
    static constexpr bool value =
        std::is_same<class_type, vertex_type>::value ||
        std::is_same<class_type, edge_type>::value;
};

using boost::vertex_index;
using boost::edge_index;

} // namespace ue2

namespace boost {

/* Install partial specialisation of property_map - this is required for
 * adaptors (like filtered_graph) to know the type of the property maps */
template<typename Graph, typename Prop>
struct property_map<Graph, Prop,
                    typename std::enable_if<ue2::is_ue2_graph<Graph>::value &&
                                            ue2::is_ue2_vertex_or_edge_property<
                                                Graph, Prop>::value>::type> {
private:
    using prop_traits = ue2::pointer_to_member_traits<Prop>;
    using member_type = typename prop_traits::member_type;
    using class_type = typename prop_traits::class_type;
public:
    using type = typename Graph::template prop_map<member_type &, class_type>;
    using const_type = typename Graph::template prop_map<const member_type &,
                                                         class_type>;
};

template<typename Graph>
struct property_map<Graph, vertex_index_t,
    typename std::enable_if<ue2::is_ue2_graph<Graph>::value>::type> {
    using v_prop_type = typename Graph::vertex_property_type;
    using type = typename Graph::template prop_map<size_t &, v_prop_type>;
    using const_type =
        typename Graph::template prop_map<const size_t &, v_prop_type>;
};

template<typename Graph>
struct property_map<Graph, edge_index_t,
    typename std::enable_if<ue2::is_ue2_graph<Graph>::value>::type> {
    using e_prop_type = typename Graph::edge_property_type;
    using type = typename Graph::template prop_map<size_t &, e_prop_type>;
    using const_type =
        typename Graph::template prop_map<const size_t &, e_prop_type>;
};

template<typename Graph>
struct property_map<Graph, vertex_all_t,
    typename std::enable_if<ue2::is_ue2_graph<Graph>::value>::type> {
    using v_prop_type = typename Graph::vertex_property_type;
    using type = typename Graph::template prop_map_all<v_prop_type &>;
    using const_type =
        typename Graph::template prop_map_all<const v_prop_type &>;
};

template<typename Graph>
struct property_map<Graph, edge_all_t,
    typename std::enable_if<ue2::is_ue2_graph<Graph>::value>::type> {
    using e_prop_type = typename Graph::edge_property_type;
    using type = typename Graph::template prop_map_all<e_prop_type &>;
    using const_type =
        typename Graph::template prop_map_all<const e_prop_type &>;
};

} // namespace boost

namespace std {

/* Specialization of std::hash so that vertex_descriptor can be used in
 * unordered containers. */
template<typename Graph>
struct hash<ue2::graph_detail::vertex_descriptor<Graph>> {
    using vertex_descriptor = ue2::graph_detail::vertex_descriptor<Graph>;
    std::size_t operator()(const vertex_descriptor &v) const {
        return v.hash();
    }
};

/* Specialization of std::hash so that edge_descriptor can be used in
 * unordered containers. */
template<typename Graph>
struct hash<ue2::graph_detail::edge_descriptor<Graph>> {
    using edge_descriptor = ue2::graph_detail::edge_descriptor<Graph>;
    std::size_t operator()(const edge_descriptor &e) const {
        return e.hash();
    }
};

} // namespace std
#endif
