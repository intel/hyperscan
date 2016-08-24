/*
 * Copyright (c) 2015-2016, Intel Corporation
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
 * \brief Convenience functions allowing range-based-for over BGL graphs.
 *
 * Normally with the BGL in C++98 you need to do this to iterate over graph
 * elements:
 *
 *        Graph:out_edge_iterator ei, ee;
 *        for (tie(ei, ee) = out_edges(v, g); ei != ee; ++ei) {
 *            do_thing_with_edge(*ei, g);
 *        }
 *
 * But now, with C++11 range-based-for and these functions, you can do this
 * instead:
 *
 *        for (auto e : out_edges_range(v, g)) {
 *            do_thing_with_edge(e, g);
 *        }
 *
 * This syntax is much more compact and keeps the iterator vars from cluttering
 * the outer scope.
 */

#ifndef UTIL_GRAPH_RANGE_H
#define UTIL_GRAPH_RANGE_H

#include <boost/range/iterator_range.hpp>

namespace ue2 {

/** Adapts a pair of iterators into a range. */
template <class Iter>
inline boost::iterator_range<Iter> pair_range(const std::pair<Iter, Iter> &p) {
    return boost::make_iterator_range(p.first, p.second);
}

/** vertices(g) */
template <class Graph>
inline auto vertices_range(const Graph &g)
    -> decltype(pair_range(vertices(g))) {
    return pair_range(vertices(g));
}

/** edges(g) */
template <class Graph>
inline auto edges_range(const Graph &g) -> decltype(pair_range(edges(g))) {
    return pair_range(edges(g));
}

/** out_edges(v, g) */
template <class Graph>
inline auto out_edges_range(const typename Graph::vertex_descriptor &v,
                            const Graph &g)
    -> decltype(pair_range(out_edges(v, g))) {
    return pair_range(out_edges(v, g));
}

/** in_edges(v, g) */
template <class Graph>
inline auto in_edges_range(const typename Graph::vertex_descriptor &v,
                           const Graph &g)
    -> decltype(pair_range(in_edges(v, g))) {
    return pair_range(in_edges(v, g));
}

/** adjacent_vertices(v, g) */
template <class Graph>
inline auto adjacent_vertices_range(const typename Graph::vertex_descriptor &v,
                                    const Graph &g)
    -> decltype(pair_range(adjacent_vertices(v, g))) {
    return pair_range(adjacent_vertices(v, g));
}

/** inv_adjacent_vertices(v, g) */
template <class Graph>
inline auto inv_adjacent_vertices_range(
    const typename Graph::vertex_descriptor &v, const Graph &g)
    -> decltype(pair_range(inv_adjacent_vertices(v, g))) {
    return pair_range(inv_adjacent_vertices(v, g));
}

} // namespace ue2

#endif // UTIL_GRAPH_RANGE_H
