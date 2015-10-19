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

#ifndef NG_HOLDER_H
#define NG_HOLDER_H

#include "ng_graph.h"
#include "ue2common.h"
#include "nfa/nfa_kind.h"

#include <boost/graph/adjacency_iterator.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

namespace ue2 {

/** \brief Encapsulates an NFAGraph, stores special vertices and other
 * metadata.
 *
 * When constructed, the graph will have the following stylised "special"
 * edges:
 *
 * - (start, startDs)
 * - (startDs, startDs) (self-loop)
 * - (accept, acceptEod)
 */
class NGHolder : boost::noncopyable {
public:
    NGHolder(void);
    explicit NGHolder(nfa_kind kind);
    virtual ~NGHolder(void);

    // Pack edge and vertex indices.
    // Note: maintaining edge index order can be expensive due to the frequency
    // of edge removal/addition, so only renumberEdges() when required by
    // operations on edge lists.
    void renumberEdges();
    void renumberVertices();

    NFAVertex getSpecialVertex(u32 id) const;

    nfa_kind kind = NFA_OUTFIX; /* Role that this plays in Rose */

    /** \brief Underlying graph object */
    NFAGraph g;

    const NFAVertex start;     //!< Anchored start vertex.
    const NFAVertex startDs;   //!< Unanchored start-dotstar vertex.
    const NFAVertex accept;    //!< Accept vertex.
    const NFAVertex acceptEod; //!< Accept at EOD vertex.

    using directed_category = NFAGraph::directed_category;
    using edge_parallel_category = NFAGraph::edge_parallel_category;
    using traversal_category = NFAGraph::traversal_category;

    using vertex_descriptor = NFAGraph::vertex_descriptor;
    using edge_descriptor = NFAGraph::edge_descriptor;
    using adjacency_iterator = NFAGraph::adjacency_iterator;
    using edge_iterator = NFAGraph::edge_iterator;
    using in_edge_iterator = NFAGraph::in_edge_iterator;
    using inv_adjacency_iterator = NFAGraph::inv_adjacency_iterator;
    using out_edge_iterator = NFAGraph::out_edge_iterator;
    using vertex_iterator = NFAGraph::vertex_iterator;
    using edge_property_type = NFAGraph::edge_property_type;
    using vertex_property_type = NFAGraph::vertex_property_type;

    // These free functions, which follow the BGL model, are the interface to
    // the graph held by this class.
    friend size_t num_vertices(NGHolder &h);
    friend size_t num_vertices(const NGHolder &h);
    friend size_t num_edges(NGHolder &h);
    friend size_t num_edges(const NGHolder &h);
    friend void remove_vertex(NFAVertex v, NGHolder &h);
    friend void clear_vertex(NFAVertex v, NGHolder &h);
    friend void clear_in_edges(NFAVertex v, NGHolder &h);
    friend void clear_out_edges(NFAVertex v, NGHolder &h);
    friend void remove_edge(const NFAEdge &e, NGHolder &h);
    friend void remove_edge(NFAVertex u, NFAVertex v, NGHolder &h);

    template<class Predicate>
    friend void remove_out_edge_if(NFAVertex v, Predicate pred, NGHolder &h) {
        boost::remove_out_edge_if(v, pred, h.g);
        h.isValidNumEdges = false;
    }

    template<class Predicate>
    friend void remove_in_edge_if(NFAVertex v, Predicate pred, NGHolder &h) {
        boost::remove_in_edge_if(v, pred, h.g);
        h.isValidNumEdges = false;
    }

    template<class Predicate>
    friend void remove_edge_if(Predicate pred, NGHolder &h) {
        boost::remove_edge_if(pred, h.g);
        h.isValidNumEdges = false;
    }

    friend std::pair<NFAEdge, bool> add_edge(NFAVertex u, NFAVertex v,
                                             NGHolder &h);
    friend std::pair<NFAEdge, bool> add_edge(NFAVertex u, NFAVertex v,
                                             const edge_property_type &ep,
                                             NGHolder &h);
    friend NFAVertex add_vertex(NGHolder &h);
    friend NFAVertex add_vertex(const vertex_property_type &vp, NGHolder &h);

    static NFAVertex null_vertex(void) { return NFAGraph::null_vertex(); }

    // Subscript operators for BGL bundled properties.
    using graph_bundled = NFAGraph::graph_bundled;
    using vertex_bundled = NFAGraph::vertex_bundled;
    using edge_bundled = NFAGraph::edge_bundled;

    vertex_bundled &operator[](NFAVertex v) {
        return get(boost::vertex_bundle, g)[v];
    }
    const vertex_bundled &operator[](NFAVertex v) const {
        return get(boost::vertex_bundle, g)[v];
    }
    edge_bundled &operator[](const NFAEdge &e) {
        return get(boost::edge_bundle, g)[e];
    }
    const edge_bundled &operator[](const NFAEdge &e) const {
        return get(boost::edge_bundle, g)[e];
    }

protected:

    /* Since the NFAGraph vertex/edge list selectors are std::lists, computing
     * num_vertices and num_edges is O(N). We use these members to store a
     * cached copy of the size.
     *
     * In the future, with C++11's constant-time std::list::size, these may
     * become obsolete. */

    u32 numVertices;
    u32 numEdges;
    bool isValidNumEdges;
    bool isValidNumVertices;
};

/** \brief True if the vertex \p v is one of our special vertices. */
template <typename GraphT>
static really_inline
bool is_special(const NFAVertex v, const GraphT &g) {
    return g[v].index < N_SPECIALS;
}

static really_inline
std::pair<NFAGraph::adjacency_iterator, NFAGraph::adjacency_iterator>
adjacent_vertices(NFAVertex v, const NGHolder &h) {
    return adjacent_vertices(v, h.g);
}

static really_inline
std::pair<NFAEdge, bool> edge(NFAVertex u, NFAVertex v, const NGHolder &h) {
    return boost::edge(u, v, h.g);
}

static really_inline
std::pair<NFAGraph::edge_iterator, NFAGraph::edge_iterator>
edges(const NGHolder &h) {
    return edges(h.g);
}

static really_inline
size_t in_degree(NFAVertex v, const NGHolder &h) {
    return in_degree(v, h.g);
}

static really_inline
std::pair<NFAGraph::in_edge_iterator, NFAGraph::in_edge_iterator>
in_edges(NFAVertex v, const NGHolder &h) {
    return in_edges(v, h.g);
}

static really_inline
std::pair<NFAGraph::inv_adjacency_iterator, NFAGraph::inv_adjacency_iterator>
inv_adjacent_vertices(NFAVertex v, const NGHolder &h) {
    return inv_adjacent_vertices(v, h.g);
}

static really_inline
size_t out_degree(NFAVertex v, const NGHolder &h) {
    return out_degree(v, h.g);
}

static really_inline
std::pair<NFAGraph::out_edge_iterator, NFAGraph::out_edge_iterator>
out_edges(NFAVertex v, const NGHolder &h) {
    return out_edges(v, h.g);
}

static really_inline
NFAVertex source(const NFAEdge &e, const NGHolder &h) {
    return source(e, h.g);
}

static really_inline
NFAVertex target(const NFAEdge &e, const NGHolder &h) {
    return target(e, h.g);
}

static really_inline
std::pair<NFAGraph::vertex_iterator, NFAGraph::vertex_iterator>
vertices(const NGHolder &h) {
    return vertices(h.g);
}

/**
 * \brief Clears all non-special vertices and edges from the graph.
 *
 * Note: not the same as the BGL's clear() function, which removes all vertices
 * and edges.
 */
void clear_graph(NGHolder &h);

/*
 * \brief Clear and remove all of the vertices pointed to by the given iterator
 * range.
 *
 * If renumber is false, no renumbering of vertex indices is done.
 *
 * Note: should not be called with iterators that will be invalidated by vertex
 * removal (such as NFAGraph::vertex_iterator).
 */
template <class Iter>
void remove_vertices(Iter begin, Iter end, NGHolder &h, bool renumber = true) {
    if (begin == end) {
        return;
    }

    for (Iter it = begin; it != end; ++it) {
        NFAVertex v = *it;
        if (!is_special(v, h)) {
            clear_vertex(v, h);
            remove_vertex(v, h);
        } else {
            assert(0);
        }
    }

    if (renumber) {
        h.renumberEdges();
        h.renumberVertices();
    }
}

/** \brief Clear and remove all of the vertices pointed to by the vertex
 * descriptors in the given container.
 *
 * This is a convenience wrapper around the iterator variant above.
 */
template <class Container>
void remove_vertices(const Container &c, NGHolder &h, bool renumber = true) {
    remove_vertices(c.begin(), c.end(), h, renumber);
}

/*
 * \brief Clear and remove all of the edges pointed to by the given iterator
 * range.
 *
 * If renumber is false, no renumbering of vertex indices is done.
 *
 * Note: should not be called with iterators that will be invalidated by vertex
 * removal (such as NFAGraph::edge_iterator).
 */
template <class Iter>
void remove_edges(Iter begin, Iter end, NGHolder &h, bool renumber = true) {
    if (begin == end) {
        return;
    }

    for (Iter it = begin; it != end; ++it) {
        const NFAEdge &e = *it;
        remove_edge(e, h);
    }

    if (renumber) {
        h.renumberEdges();
    }
}

/** \brief Clear and remove all of the edges pointed to by the edge descriptors
 * in the given container.
 *
 * This is a convenience wrapper around the iterator variant above.
 */
template <class Container>
void remove_edges(const Container &c, NGHolder &h, bool renumber = true) {
    remove_edges(c.begin(), c.end(), h, renumber);
}

static UNUSED
bool is_triggered(const NGHolder &g) {
    return is_triggered(g.kind);
}

static UNUSED
bool generates_callbacks(const NGHolder &g) {
    return generates_callbacks(g.kind);
}
} // namespace ue2

#endif
