# Boost 1.62 has a bug that we've patched around, check if it is required
if (Boost_VERSION EQUAL 106200)
    set (CMAKE_REQUIRED_INCLUDES ${BOOST_INCLUDEDIR} "${PROJECT_SOURCE_DIR}/include")
    set (BOOST_REV_TEST "
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/reverse_graph.hpp>
#include <boost/concept/assert.hpp>

int main(int,char*[])
{
  using namespace boost;
  // Check const reverse_graph
  {
    typedef adjacency_list< vecS, vecS, bidirectionalS,
      property<vertex_color_t, int>,
      property<edge_weight_t, int>,
      property<graph_name_t, std::string>
    > AdjList;
    typedef reverse_graph<AdjList> Graph;
    BOOST_CONCEPT_ASSERT(( BidirectionalGraphConcept<Graph> ));
  }
  return 0;
}
")

    CHECK_CXX_SOURCE_COMPILES("${BOOST_REV_TEST}" BOOST_REVGRAPH_OK)

    if (NOT BOOST_REVGRAPH_OK)
        message(STATUS "trying patched")
        CHECK_CXX_SOURCE_COMPILES("
#include <boost-patched/graph/reverse_graph.hpp>
${BOOST_REV_TEST}" BOOST_REVGRAPH_PATCH)
    endif()

    if (NOT BOOST_REVGRAPH_OK AND NOT BOOST_REVGRAPH_PATCH)
        message(FATAL_ERROR "Something is wrong with this copy of boost::reverse_graph")
    endif()

    unset (CMAKE_REQUIRED_INCLUDES)
endif () # Boost 1.62.0
