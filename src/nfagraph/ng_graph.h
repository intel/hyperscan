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
 * \brief Definition of the NFAGraph type used for all NFA graph
 * representations.
 *
 * Note that most of the time we don't work on a bare NFAGraph: instead
 * we use an NGHolder, which wraps the graph and defines our special vertices,
 * etc.
 */

#ifndef NG_GRAPH_H
#define NG_GRAPH_H

#include "util/charreach.h"
#include "util/ue2_containers.h"
#include "ue2common.h"

#include <boost/graph/adjacency_iterator.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

namespace ue2 {

/** \brief Properties associated with each vertex in an NFAGraph. */
struct NFAGraphVertexProps {
    /** \brief Set of characters on which this vertex is reachable. */
    CharReach char_reach;

    /** \brief Set of reports raised by this vertex. */
    ue2::flat_set<ReportID> reports;

    /** \brief Unique index for this vertex, used for BGL algorithms. */
    u32 index = 0;

    /** \brief Flags associated with assertions. */
    u32 assert_flags = 0;
};

/** \brief Properties associated with each edge in an NFAGraph. */
struct NFAGraphEdgeProps {
    /** \brief Unique index for this edge, used for BGL algorithms. */
    u32 index = 0;

    /** \brief For graphs that will be implemented as multi-top engines, this
     * specifies the top event. Only used on edges from the start vertex. */
    u32 top = 0;

    /** \brief Flags associated with assertions. */
    u32 assert_flags = 0;
};

// For flexibility: boost::listS, boost::listS for out-edge and vertex lists.
// boost::bidirectionalS for directed graph so that we can get at in-edges.
typedef boost::adjacency_list<boost::listS,
                              boost::listS,
                              boost::bidirectionalS,
                              NFAGraphVertexProps,
                              NFAGraphEdgeProps> NFAGraph;

typedef NFAGraph::vertex_descriptor NFAVertex;
typedef NFAGraph::edge_descriptor NFAEdge;

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

} // namespace ue2

#endif
