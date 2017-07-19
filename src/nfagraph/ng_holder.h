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
 * \brief Definition of the NGHolder type used for to represent general nfa
 * graphs as well as all associated types (vertex and edge properties, etc).
 *
 * The NGHolder also contains the special vertices used to represents starts and
 * accepts.
 */

#ifndef NG_HOLDER_H
#define NG_HOLDER_H

#include "ue2common.h"
#include "nfa/nfa_kind.h"
#include "util/charreach.h"
#include "util/flat_containers.h"
#include "util/ue2_graph.h"

namespace ue2 {

/** \brief Properties associated with each vertex in an NFAGraph. */
struct NFAGraphVertexProps {
    /** \brief Set of characters on which this vertex is reachable. */
    CharReach char_reach;

    /** \brief Set of reports raised by this vertex. */
    flat_set<ReportID> reports;

    /** \brief Unique index for this vertex, used for BGL algorithms. */
    size_t index = 0;

    /** \brief Flags associated with assertions. */
    u32 assert_flags = 0;
};

/** \brief Properties associated with each edge in an NFAGraph. */
struct NFAGraphEdgeProps {
    /** \brief Unique index for this edge, used for BGL algorithms. */
    size_t index = 0;

    /** \brief For graphs that will be implemented as multi-top engines, this
     * specifies the top events. Only used on edges from the start vertex. */
    flat_set<u32> tops;

    /** \brief Flags associated with assertions. */
    u32 assert_flags = 0;
};

/** \brief vertex_index values for special nodes in the NFAGraph. */
enum SpecialNodes {
    /** \brief Anchored start vertex. WARNING: this may be triggered at various
     * locations (not just zero) for triggered graphs. */
    NODE_START,

    /** \brief Unanchored start-dotstar vertex. WARNING: this may not have a
     * proper self-loop. */
    NODE_START_DOTSTAR,

    /** \brief Accept vertex. All vertices that can match at arbitrary offsets
     * must have an edge to this vertex. */
    NODE_ACCEPT,

    /** \brief Accept-EOD vertex. Vertices that must raise a match at EOD only
     * must have an edge to this vertex. */
    NODE_ACCEPT_EOD,

    /** \brief Sentinel, number of special vertices. */
    N_SPECIALS
};

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
class NGHolder : public ue2_graph<NGHolder, NFAGraphVertexProps,
                                  NFAGraphEdgeProps> {
public:
    explicit NGHolder(nfa_kind kind);
    NGHolder(void) : NGHolder(NFA_OUTFIX) {};
    virtual ~NGHolder(void);

    nfa_kind kind; /* Role that this plays in Rose */

    static const size_t N_SPECIAL_VERTICES = N_SPECIALS;
public:
    const vertex_descriptor start;     //!< Anchored start vertex.
    const vertex_descriptor startDs;   //!< Unanchored start-dotstar vertex.
    const vertex_descriptor accept;    //!< Accept vertex.
    const vertex_descriptor acceptEod; //!< Accept at EOD vertex.

    vertex_descriptor getSpecialVertex(u32 id) const;
};

typedef NGHolder::vertex_descriptor NFAVertex;
typedef NGHolder::edge_descriptor NFAEdge;

/** \brief True if the vertex \p v is one of our special vertices. */
template <typename GraphT>
bool is_special(const typename GraphT::vertex_descriptor v, const GraphT &g) {
    return g[v].index < N_SPECIALS;
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
        renumber_edges(h);
        renumber_vertices(h);
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
        renumber_edges(h);
    }
}

#define DEFAULT_TOP 0U

/** \brief Clear and remove all of the edges pointed to by the edge descriptors
 * in the given container.
 *
 * This is a convenience wrapper around the iterator variant above.
 */
template <class Container>
void remove_edges(const Container &c, NGHolder &h, bool renumber = true) {
    remove_edges(c.begin(), c.end(), h, renumber);
}

inline
bool is_triggered(const NGHolder &g) {
    return is_triggered(g.kind);
}

inline
bool generates_callbacks(const NGHolder &g) {
    return generates_callbacks(g.kind);
}

inline
bool has_managed_reports(const NGHolder &g) {
    return has_managed_reports(g.kind);
}

inline
bool inspects_states_for_accepts(const NGHolder &g) {
    return inspects_states_for_accepts(g.kind);
}

} // namespace ue2

#endif
