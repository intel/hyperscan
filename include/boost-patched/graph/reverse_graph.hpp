#ifndef REVERSE_GRAPH_PATCHED_H_
#define REVERSE_GRAPH_PATCHED_H_

#include <boost/version.hpp>

#include <boost/graph/reverse_graph.hpp>

#if defined(BOOST_REVGRAPH_PATCH)

// Boost 1.62.0 does not implement degree() in reverse_graph which is required
// by BidirectionalGraph, so add it.

namespace boost {

template <class BidirectionalGraph, class GRef>
inline typename graph_traits<BidirectionalGraph>::degree_size_type
degree(const typename graph_traits<BidirectionalGraph>::vertex_descriptor u,
       const reverse_graph<BidirectionalGraph,GRef>& g)
{
    return degree(u, g.m_g);
}

} // namespace boost

#endif // Boost 1.62.0

#endif
