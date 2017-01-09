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

#include "config.h"
#include "gtest/gtest.h"
#include "util/graph.h"
#include "util/ue2_graph.h"

#include <boost/graph/adjacency_iterator.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/reverse_graph.hpp>

#include <type_traits>

using namespace boost;
using namespace std;
using namespace ue2;

typedef adjacency_list<vecS, vecS, bidirectionalS, no_property, no_property,
                       no_property> unit_graph;
typedef unit_graph::vertex_descriptor unit_vertex;
typedef unit_graph::edge_descriptor unit_edge;

TEST(graph_util, parallel) {
    unit_graph g;

    ASSERT_FALSE(has_parallel_edge(g));

    unit_vertex v1 = add_vertex(g);
    unit_vertex v2 = add_vertex(g);

    ASSERT_FALSE(has_parallel_edge(g));

    add_edge(v1, v2, g);

    ASSERT_FALSE(has_parallel_edge(g));

    add_edge(v2, v1, g);

    ASSERT_FALSE(has_parallel_edge(g));

    add_edge(v2, v1, g);

    ASSERT_TRUE(has_parallel_edge(g));
}

TEST(graph_util, dag) {
    unit_graph g;
    ASSERT_TRUE(is_dag(g));

    unit_vertex v1 = add_vertex(g);
    unit_vertex v2 = add_vertex(g);

    ASSERT_TRUE(is_dag(g));

    add_edge(v1, v2, g);

    unit_vertex v3 = add_vertex(g);

    add_edge(v1, v3, g);
    add_edge(v2, v3, g);

    ASSERT_TRUE(is_dag(g));

    add_edge(v2, v2, g);

    ASSERT_FALSE(is_dag(g));
    ASSERT_TRUE(is_dag(g, true)); /* only cycle is self loop */

    unit_vertex v4 = add_vertex(g);

    add_edge(v3, v4, g);
    add_edge(v4, v1, g);

    ASSERT_FALSE(is_dag(g, true)); /* now have a large cycle */
}

TEST(graph_util, degrees) {
    unit_graph g;

    unit_vertex a = add_vertex(g);
    unit_vertex b = add_vertex(g);
    unit_vertex c = add_vertex(g);
    unit_vertex d = add_vertex(g);
    unit_vertex e = add_vertex(g);
    unit_vertex f = add_vertex(g);

    add_edge(a, b, g);
    add_edge(b, c, g);
    add_edge(d, e, g);

    add_edge(f, a, g);
    add_edge(f, b, g);
    add_edge(f, c, g);

    unit_graph::vertex_iterator vi, ve;

    tie(vi, ve) = vertices(g);
    ASSERT_FALSE(anySelfLoop(g, vi, ve));

    add_edge(b, b, g);

    tie(vi, ve) = vertices(g);
    ASSERT_TRUE(anySelfLoop(g, vi, ve));

    add_edge(e, e, g);

    ASSERT_FALSE(isLeafNode(a, g));
    ASSERT_FALSE(isLeafNode(b, g));
    ASSERT_TRUE( isLeafNode(c, g));
    ASSERT_FALSE(isLeafNode(d, g));
    ASSERT_FALSE( isLeafNode(e, g));
    ASSERT_FALSE(isLeafNode(f, g));

    ASSERT_FALSE(hasSelfLoop(a, g));
    ASSERT_TRUE( hasSelfLoop(b, g));
    ASSERT_FALSE(hasSelfLoop(c, g));
    ASSERT_FALSE(hasSelfLoop(d, g));
    ASSERT_TRUE( hasSelfLoop(e, g));
    ASSERT_FALSE(hasSelfLoop(f, g));

    ASSERT_EQ((size_t)1, proper_out_degree(a, g));
    ASSERT_EQ((size_t)1, proper_out_degree(b, g));
    ASSERT_EQ((size_t)0, proper_out_degree(c, g));
    ASSERT_EQ((size_t)1, proper_out_degree(d, g));
    ASSERT_EQ((size_t)0, proper_out_degree(e, g));
    ASSERT_EQ((size_t)3, proper_out_degree(f, g));

    ASSERT_EQ((size_t)1, proper_in_degree(a, g));
    ASSERT_EQ((size_t)2, proper_in_degree(b, g));
    ASSERT_EQ((size_t)2, proper_in_degree(c, g));
    ASSERT_EQ((size_t)0, proper_in_degree(d, g));
    ASSERT_EQ((size_t)1, proper_in_degree(e, g));

    ASSERT_TRUE( has_successor(a, g));
    ASSERT_TRUE( has_successor(b, g));
    ASSERT_FALSE(has_successor(c, g));
    ASSERT_TRUE( has_successor(d, g));
    ASSERT_TRUE( has_successor(e, g));
    ASSERT_TRUE( has_successor(f, g));

    ASSERT_TRUE( has_proper_successor(a, g));
    ASSERT_TRUE( has_proper_successor(b, g));
    ASSERT_FALSE(has_proper_successor(c, g));
    ASSERT_TRUE( has_proper_successor(d, g));
    ASSERT_FALSE(has_proper_successor(e, g));
    ASSERT_TRUE( has_proper_successor(f, g));
}

struct SimpleV {
    size_t index;
    string test_v = "SimpleV";
};

struct SimpleE {
    size_t index;
    string test_e = "SimpleE";
};

struct SimpleG : public ue2_graph<SimpleG, SimpleV, SimpleE> {
};

TEST(ue2_graph, graph_concept) {
    static_assert(std::is_same<SimpleG::vertex_descriptor,
                               graph_traits<SimpleG>::vertex_descriptor>::value,
                  "vertex_descriptor");
    static_assert(std::is_same<SimpleG::edge_descriptor,
                               graph_traits<SimpleG>::edge_descriptor>::value,
                  "edge_descriptor");
    static_assert(std::is_same<SimpleG::directed_category,
                               graph_traits<SimpleG>::directed_category>::value,
                  "directed_category");
    static_assert(std::is_same<SimpleG::edge_parallel_category,
                           graph_traits<SimpleG>::edge_parallel_category>::value,
                  "edge_parallel_category");
    static_assert(std::is_same<SimpleG::traversal_category,
                               graph_traits<SimpleG>::traversal_category>::value,
                  "traversal_category");

    UNUSED SimpleG::vertex_descriptor n = SimpleG::null_vertex();

    BOOST_CONCEPT_ASSERT((GraphConcept<SimpleG>));
}

TEST(ue2_graph, vertex_list_concept) {
    BOOST_CONCEPT_ASSERT((VertexListGraphConcept<SimpleG>));
}

TEST(ue2_graph, edge_list_concept) {
    BOOST_CONCEPT_ASSERT((EdgeListGraphConcept<SimpleG>));
}

TEST(ue2_graph, incidence_concept) {
    BOOST_CONCEPT_ASSERT((IncidenceGraphConcept<SimpleG>));
}

TEST(ue2_graph, bidi_concept) {
    BOOST_CONCEPT_ASSERT((BidirectionalGraphConcept<SimpleG>));
}

TEST(ue2_graph, mutable_concept) {
    BOOST_CONCEPT_ASSERT((MutableGraphConcept<SimpleG>));
}

TEST(ue2_graph, property_concept) {
    static_assert(std::is_same<SimpleG::vertex_property_type, SimpleV>::value,
                  "vertex_property_type");
    static_assert(std::is_same<SimpleG::edge_property_type, SimpleE>::value,
                  "edge_property_type");

    /* Although documented as part of the MutablePropertyGraph concept,
     * (vertex|edge)_property_type don't appear to exist in the traits for any
     * existing graph types and the typedefs are not installed by default */

    // static_assert(std::is_same<
    //                   typename graph_traits<SimpleG>::vertex_property_type,
    //                   SimpleV>::value,
    //               "vertex_property_type");
    // static_assert(std::is_same<
    //                   typename graph_traits<SimpleG>::edge_property_type,
    //                   SimpleE>::value,
    //               "edge_property_type");

    /* However, there does seem to be an undocumented templated structure
     * paralleling the main graph_traits */
    static_assert(std::is_same<
                      typename vertex_property_type<SimpleG>::type,
                      SimpleV>::value,
                  "vertex_property_type");
    static_assert(std::is_same<
                      typename edge_property_type<SimpleG>::type,
                      SimpleE>::value,
                  "edge_property_type");

    BOOST_CONCEPT_ASSERT((VertexMutablePropertyGraphConcept<SimpleG>));
    BOOST_CONCEPT_ASSERT((EdgeMutablePropertyGraphConcept<SimpleG>));
}

TEST(ue2_graph, add_vertex) {
    SimpleG g;
    SimpleG::vertex_descriptor a = add_vertex(g);
    ASSERT_NE(SimpleG::null_vertex(), a);
}

TEST(ue2_graph, add_and_remove_vertex) {
    SimpleG g;
    ASSERT_EQ(0U, num_vertices(g));

    SimpleG::vertex_descriptor a = add_vertex(g);
    ASSERT_EQ(1U, num_vertices(g));
    ASSERT_NE(SimpleG::null_vertex(), a);
    auto p = vertices(g);
    ASSERT_NE(p.first, p.second);
    ASSERT_EQ(a, *p.first);
    ++p.first;
    ASSERT_EQ(p.first, p.second);

    remove_vertex(a, g);
    ASSERT_EQ(0U, num_vertices(g));
    auto q = vertices(g);
    ASSERT_EQ(q.first, q.second);
}

TEST(ue2_graph, add_edge) {
    SimpleG g;
    SimpleG::vertex_descriptor a = add_vertex(g);
    ASSERT_NE(SimpleG::null_vertex(), a);
    SimpleG::vertex_descriptor b = add_vertex(g);
    ASSERT_NE(SimpleG::null_vertex(), b);
    ASSERT_NE(a, b);
    auto p = add_edge(a, b, g);
    ASSERT_TRUE(p.second);
    ASSERT_EQ(1U, num_edges(g));

    ASSERT_EQ(a, source(p.first, g));
    ASSERT_EQ(b, target(p.first, g));

    auto q = edge(a, b, g);
    ASSERT_TRUE(q.second);
    ASSERT_EQ(p.second, q.first);
}

TEST(ue2_graph, add_remove_edge1) {
    SimpleG g;
    SimpleG::vertex_descriptor a = add_vertex(g);
    ASSERT_NE(SimpleG::null_vertex(), a);
    SimpleG::vertex_descriptor b = add_vertex(g);
    ASSERT_NE(SimpleG::null_vertex(), b);
    ASSERT_NE(a, b);
    auto p = add_edge(a, b, g);
    ASSERT_TRUE(p.second);
    ASSERT_EQ(1U, num_edges(g));

    ASSERT_EQ(a, source(p.first, g));
    ASSERT_EQ(b, target(p.first, g));

    remove_edge(p.first, g);
    auto q = edge(a, b, g);
    ASSERT_FALSE(q.second);
    ASSERT_EQ(q.first, SimpleG::null_edge());
    ASSERT_EQ(0U, num_edges(g));
}

TEST(ue2_graph, add_remove_edge2) {
    SimpleG g;
    SimpleG::vertex_descriptor a = add_vertex(g);
    ASSERT_NE(SimpleG::null_vertex(), a);
    SimpleG::vertex_descriptor b = add_vertex(g);
    ASSERT_NE(SimpleG::null_vertex(), b);
    ASSERT_NE(a, b);
    auto p = add_edge(a, b, g);
    ASSERT_TRUE(p.second);
    ASSERT_EQ(1U, num_edges(g));

    ASSERT_EQ(a, source(p.first, g));
    ASSERT_EQ(b, target(p.first, g));

    remove_edge(a, b, g);
    auto q = edge(a, b, g);
    ASSERT_FALSE(q.second);
    ASSERT_EQ(q.first, SimpleG::null_edge());
    ASSERT_EQ(0U, num_edges(g));
}

TEST(ue2_graph, add_edge_clear1) {
    SimpleG g;
    SimpleG::vertex_descriptor a = add_vertex(g);
    ASSERT_NE(SimpleG::null_vertex(), a);
    SimpleG::vertex_descriptor b = add_vertex(g);
    ASSERT_NE(SimpleG::null_vertex(), b);
    ASSERT_NE(a, b);
    auto p = add_edge(a, b, g);
    ASSERT_TRUE(p.second);
    ASSERT_EQ(1U, num_edges(g));

    ASSERT_EQ(a, source(p.first, g));
    ASSERT_EQ(b, target(p.first, g));

    clear_vertex(a, g);
    auto q = edge(a, b, g);
    ASSERT_FALSE(q.second);
    ASSERT_EQ(q.first, SimpleG::null_edge());
    ASSERT_EQ(0U, num_edges(g));
}

TEST(ue2_graph, add_edge_clear2) {
    SimpleG g;
    SimpleG::vertex_descriptor a = add_vertex(g);
    ASSERT_NE(SimpleG::null_vertex(), a);
    SimpleG::vertex_descriptor b = add_vertex(g);
    ASSERT_NE(SimpleG::null_vertex(), b);
    ASSERT_NE(a, b);
    auto p = add_edge(a, b, g);
    ASSERT_TRUE(p.second);
    ASSERT_EQ(1U, num_edges(g));

    ASSERT_EQ(a, source(p.first, g));
    ASSERT_EQ(b, target(p.first, g));

    clear_vertex(b, g);
    auto q = edge(a, b, g);
    ASSERT_FALSE(q.second);
    ASSERT_EQ(q.first, SimpleG::null_edge());
    ASSERT_EQ(0U, num_edges(g));
}

TEST(ue2_graph, add_edge_clear_out) {
    SimpleG g;
    SimpleG::vertex_descriptor a = add_vertex(g);
    ASSERT_NE(SimpleG::null_vertex(), a);
    SimpleG::vertex_descriptor b = add_vertex(g);
    ASSERT_NE(SimpleG::null_vertex(), b);
    ASSERT_NE(a, b);
    auto p = add_edge(a, b, g);
    ASSERT_TRUE(p.second);
    ASSERT_EQ(1U, num_edges(g));

    ASSERT_EQ(a, source(p.first, g));
    ASSERT_EQ(b, target(p.first, g));

    clear_out_edges(a, g);
    auto q = edge(a, b, g);
    ASSERT_FALSE(q.second);
    ASSERT_EQ(q.first, SimpleG::null_edge());
    ASSERT_EQ(0U, num_edges(g));
}

TEST(ue2_graph, add_edge_clear_in) {
    SimpleG g;
    SimpleG::vertex_descriptor a = add_vertex(g);
    ASSERT_NE(SimpleG::null_vertex(), a);
    SimpleG::vertex_descriptor b = add_vertex(g);
    ASSERT_NE(SimpleG::null_vertex(), b);
    ASSERT_NE(a, b);
    auto p = add_edge(a, b, g);
    ASSERT_TRUE(p.second);
    ASSERT_EQ(1U, num_edges(g));

    ASSERT_EQ(a, source(p.first, g));
    ASSERT_EQ(b, target(p.first, g));

    clear_in_edges(b, g);
    auto q = edge(a, b, g);
    ASSERT_FALSE(q.second);
    ASSERT_EQ(q.first, SimpleG::null_edge());
    ASSERT_EQ(0U, num_edges(g));
}

TEST(ue2_graph, add_remove_edge_iter) {
    SimpleG g;
    SimpleG::vertex_descriptor a = add_vertex(g);
    ASSERT_NE(SimpleG::null_vertex(), a);
    SimpleG::vertex_descriptor b = add_vertex(g);
    ASSERT_NE(SimpleG::null_vertex(), b);
    ASSERT_NE(a, b);
    auto p = add_edge(a, b, g);
    ASSERT_TRUE(p.second);
    ASSERT_EQ(1U, num_edges(g));

    ASSERT_EQ(a, source(p.first, g));
    ASSERT_EQ(b, target(p.first, g));

    remove_edge(edges(g).first, g);
    auto q = edge(a, b, g);
    ASSERT_FALSE(q.second);
    ASSERT_EQ(q.first, SimpleG::null_edge());
    ASSERT_EQ(0U, num_edges(g));
}

TEST(ue2_graph, vertices_0) {
    SimpleG g;
    auto p = vertices(g);
    ASSERT_EQ(p.first, p.second);
}

TEST(ue2_graph, vertices_1) {
    SimpleG g;
    SimpleG::vertex_iterator vi;
    SimpleG::vertex_iterator ve;
    auto a = add_vertex(g);

    ASSERT_EQ(1U, num_vertices(g));
    tie(vi, ve) = vertices(g);
    ASSERT_EQ(a, *vi++);
    ASSERT_EQ(vi, ve);

    auto b = add_vertex(g);
    auto c = add_vertex(g);
    auto d = add_vertex(g);

    ASSERT_EQ(4U, num_vertices(g));
    tie(vi, ve) = vertices(g);
    ASSERT_EQ(a, *vi++);
    ASSERT_EQ(b, *vi++);
    ASSERT_EQ(c, *vi++);
    ASSERT_EQ(d, *vi++);
    ASSERT_EQ(vi, ve);

    remove_vertex(c, g);

    ASSERT_EQ(3U, num_vertices(g));
    tie(vi, ve) = vertices(g);
    ASSERT_EQ(a, *vi++);
    ASSERT_EQ(b, *vi++);
    ASSERT_EQ(d, *vi++);
    ASSERT_EQ(vi, ve);

    remove_vertex(a, g);

    ASSERT_EQ(2U, num_vertices(g));
    tie(vi, ve) = vertices(g);
    ASSERT_EQ(b, *vi++);
    ASSERT_EQ(d, *vi++);
    ASSERT_EQ(vi, ve);

    auto e = add_vertex(g);

    ASSERT_EQ(3U, num_vertices(g));
    tie(vi, ve) = vertices(g);
    ASSERT_EQ(b, *vi++);
    ASSERT_EQ(d, *vi++);
    ASSERT_EQ(e, *vi++);
    ASSERT_EQ(vi, ve);

    remove_vertex(e, g);

    ASSERT_EQ(2U, num_vertices(g));
    tie(vi, ve) = vertices(g);
    ASSERT_EQ(b, *vi++);
    ASSERT_EQ(d, *vi++);
    ASSERT_EQ(vi, ve);

    remove_vertex(b, g);
    remove_vertex(d, g);

    ASSERT_EQ(0U, num_vertices(g));
    tie(vi, ve) = vertices(g);
    ASSERT_EQ(vi, ve);
}

TEST(ue2_graph, out_edges_1) {
    SimpleG g;
    auto a = add_vertex(g);

    ASSERT_EQ(1U, num_vertices(g));
    ASSERT_EQ(0U, out_degree(a, g));

    SimpleG::out_edge_iterator ei;
    SimpleG::out_edge_iterator ee;

    tie(ei, ee) = out_edges(a, g);
    ASSERT_TRUE(ei == ee);

    auto p = add_edge(a, a, g);
    ASSERT_TRUE(p.second);
    ASSERT_EQ(1U, num_edges(g));
    SimpleG::edge_descriptor e1 = p.first;

    ASSERT_EQ(1U, out_degree(a, g));
    tie(ei, ee) = out_edges(a, g);
    ASSERT_EQ(e1, *ei++);
    ASSERT_EQ(ei, ee);

    p = add_edge(a, a, g);
    ASSERT_TRUE(p.second);
    ASSERT_EQ(2U, num_edges(g));
    SimpleG::edge_descriptor e2 = p.first;

    ASSERT_EQ(2U, out_degree(a, g));
    tie(ei, ee) = out_edges(a, g);
    ASSERT_EQ(e1, *ei++);
    ASSERT_EQ(e2, *ei++);
    ASSERT_EQ(ei, ee);
}

TEST(ue2_graph, out_edges_2) {
    SimpleG g;
    auto a = add_vertex(g);
    auto b = add_vertex(g);
    auto c = add_vertex(g);

    ASSERT_EQ(3U, num_vertices(g));
    ASSERT_EQ(0U, out_degree(a, g));

    SimpleG::out_edge_iterator ei;
    SimpleG::out_edge_iterator ee;

    tie(ei, ee) = out_edges(a, g);
    ASSERT_TRUE(ei == ee);

    auto p = add_edge(a, b, g);
    ASSERT_TRUE(p.second);
    ASSERT_EQ(1U, num_edges(g));
    SimpleG::edge_descriptor e1 = p.first;

    ASSERT_EQ(1U, out_degree(a, g));
    tie(ei, ee) = out_edges(a, g);
    ASSERT_EQ(e1, *ei++);
    ASSERT_EQ(ei, ee);

    p = add_edge(a, c, g);
    ASSERT_TRUE(p.second);
    ASSERT_EQ(2U, num_edges(g));
    SimpleG::edge_descriptor e2 = p.first;

    ASSERT_EQ(2U, out_degree(a, g));
    tie(ei, ee) = out_edges(a, g);
    ASSERT_EQ(e1, *ei++);
    ASSERT_EQ(e2, *ei++);
    ASSERT_EQ(ei, ee);

    p = add_edge(c, b, g);
    ASSERT_TRUE(p.second);
    ASSERT_EQ(3U, num_edges(g));

    ASSERT_EQ(2U, out_degree(a, g));
    tie(ei, ee) = out_edges(a, g);
    ASSERT_EQ(e1, *ei++);
    ASSERT_EQ(e2, *ei++);
    ASSERT_EQ(ei, ee);

    p = add_edge(b, a, g);
    ASSERT_TRUE(p.second);
    ASSERT_EQ(4U, num_edges(g));

    ASSERT_EQ(2U, out_degree(a, g));
    tie(ei, ee) = out_edges(a, g);
    ASSERT_EQ(e1, *ei++);
    ASSERT_EQ(e2, *ei++);
    ASSERT_EQ(ei, ee);

    remove_edge(a, c, g);
    ASSERT_EQ(3U, num_edges(g));

    ASSERT_EQ(1U, out_degree(a, g));
    tie(ei, ee) = out_edges(a, g);
    ASSERT_EQ(e1, *ei++);
    ASSERT_EQ(ei, ee);

    p = add_edge(a, a, g);
    ASSERT_EQ(4U, num_edges(g));
    ASSERT_TRUE(p.second);
    SimpleG::edge_descriptor e3 = p.first;

    ASSERT_EQ(2U, out_degree(a, g));
    tie(ei, ee) = out_edges(a, g);
    ASSERT_EQ(e1, *ei++);
    ASSERT_EQ(e3, *ei++);
    ASSERT_EQ(ei, ee);

    clear_out_edges(a, g);
    ASSERT_EQ(2U, num_edges(g));

    ASSERT_EQ(0U, out_degree(a, g));
    tie(ei, ee) = out_edges(a, g);
    ASSERT_EQ(ei, ee);
}

TEST(ue2_graph, in_edges_1) {
    SimpleG g;
    auto a = add_vertex(g);

    ASSERT_EQ(1U, num_vertices(g));
    ASSERT_EQ(0U, in_degree(a, g));

    SimpleG::in_edge_iterator ei;
    SimpleG::in_edge_iterator ee;

    tie(ei, ee) = in_edges(a, g);
    ASSERT_TRUE(ei == ee);

    auto p = add_edge(a, a, g);
    ASSERT_TRUE(p.second);
    ASSERT_EQ(1U, num_edges(g));
    SimpleG::edge_descriptor e1 = p.first;

    ASSERT_EQ(1U, in_degree(a, g));
    tie(ei, ee) = in_edges(a, g);
    ASSERT_EQ(e1, *ei++);
    ASSERT_EQ(ei, ee);

    p = add_edge(a, a, g);
    ASSERT_TRUE(p.second);
    ASSERT_EQ(2U, num_edges(g));
    SimpleG::edge_descriptor e2 = p.first;

    ASSERT_EQ(2U, in_degree(a, g));
    tie(ei, ee) = in_edges(a, g);
    ASSERT_EQ(e1, *ei++);
    ASSERT_EQ(e2, *ei++);
    ASSERT_EQ(ei, ee);
}

TEST(ue2_graph, in_edges_2) {
    SimpleG g;
    auto a = add_vertex(g);
    auto b = add_vertex(g);
    auto c = add_vertex(g);

    ASSERT_EQ(3U, num_vertices(g));
    ASSERT_EQ(0U, in_degree(a, g));

    SimpleG::in_edge_iterator ei;
    SimpleG::in_edge_iterator ee;

    tie(ei, ee) = in_edges(a, g);
    ASSERT_TRUE(ei == ee);

    auto p = add_edge(b, a, g);
    ASSERT_TRUE(p.second);
    ASSERT_EQ(1U, num_edges(g));
    SimpleG::edge_descriptor e1 = p.first;

    ASSERT_EQ(1U, in_degree(a, g));
    tie(ei, ee) = in_edges(a, g);
    ASSERT_EQ(e1, *ei++);
    ASSERT_EQ(ei, ee);

    p = add_edge(c, a, g);
    ASSERT_TRUE(p.second);
    ASSERT_EQ(2U, num_edges(g));
    SimpleG::edge_descriptor e2 = p.first;

    ASSERT_EQ(2U, in_degree(a, g));
    tie(ei, ee) = in_edges(a, g);
    ASSERT_EQ(e1, *ei++);
    ASSERT_EQ(e2, *ei++);
    ASSERT_EQ(ei, ee);

    p = add_edge(c, b, g);
    ASSERT_TRUE(p.second);
    ASSERT_EQ(3U, num_edges(g));

    ASSERT_EQ(2U, in_degree(a, g));
    tie(ei, ee) = in_edges(a, g);
    ASSERT_EQ(e1, *ei++);
    ASSERT_EQ(e2, *ei++);
    ASSERT_EQ(ei, ee);

    p = add_edge(a, b, g);
    ASSERT_TRUE(p.second);
    ASSERT_EQ(4U, num_edges(g));

    ASSERT_EQ(2U, in_degree(a, g));
    tie(ei, ee) = in_edges(a, g);
    ASSERT_EQ(e1, *ei++);
    ASSERT_EQ(e2, *ei++);
    ASSERT_EQ(ei, ee);

    remove_edge(c, a, g);
    ASSERT_EQ(3U, num_edges(g));

    ASSERT_EQ(1U, in_degree(a, g));
    tie(ei, ee) = in_edges(a, g);
    ASSERT_EQ(e1, *ei++);
    ASSERT_EQ(ei, ee);

    p = add_edge(a, a, g);
    ASSERT_EQ(4U, num_edges(g));
    ASSERT_TRUE(p.second);
    SimpleG::edge_descriptor e3 = p.first;

    ASSERT_EQ(2U, in_degree(a, g));
    tie(ei, ee) = in_edges(a, g);
    ASSERT_EQ(e1, *ei++);
    ASSERT_EQ(e3, *ei++);
    ASSERT_EQ(ei, ee);

    clear_in_edges(a, g);
    ASSERT_EQ(2U, num_edges(g));

    ASSERT_EQ(0U, in_degree(a, g));
    tie(ei, ee) = in_edges(a, g);
    ASSERT_EQ(ei, ee);
}

TEST(ue2_graph, parallel_1) {
    SimpleG g;
    SimpleG::vertex_iterator vi;
    SimpleG::vertex_iterator ve;
    auto a = add_vertex(g);

    ASSERT_EQ(1U, num_vertices(g));
    ASSERT_EQ(0U, out_degree(a, g));

    SimpleG::out_edge_iterator ei;
    SimpleG::out_edge_iterator ee;

    tie(ei, ee) = out_edges(a, g);
    ASSERT_TRUE(ei == ee);

    auto p = add_edge(a, a, g);
    ASSERT_TRUE(p.second);
    ASSERT_EQ(1U, num_edges(g));
    SimpleG::edge_descriptor e1 = p.first;

    ASSERT_EQ(1U, out_degree(a, g));
    tie(ei, ee) = out_edges(a, g);
    ASSERT_EQ(e1, *ei++);
    ASSERT_EQ(ei, ee);

    p = add_edge(a, a, g);
    ASSERT_TRUE(p.second);
    ASSERT_EQ(2U, num_edges(g));
    SimpleG::edge_descriptor e2 = p.first;

    ASSERT_EQ(2U, out_degree(a, g));
    tie(ei, ee) = out_edges(a, g);
    ASSERT_EQ(e1, *ei++);
    ASSERT_EQ(e2, *ei++);
    ASSERT_EQ(ei, ee);

    remove_edge(e1, g);

    ASSERT_EQ(1U, out_degree(a, g));
    tie(ei, ee) = out_edges(a, g);
    ASSERT_EQ(e2, *ei++);
    ASSERT_EQ(ei, ee);

    p = add_edge(a, a, g);
    ASSERT_TRUE(p.second);
    ASSERT_EQ(2U, num_edges(g));
    SimpleG::edge_descriptor e3 = p.first;

    ASSERT_EQ(2U, out_degree(a, g));
    tie(ei, ee) = out_edges(a, g);
    ASSERT_EQ(e2, *ei++);
    ASSERT_EQ(e3, *ei++);
    ASSERT_EQ(ei, ee);

    remove_edge(a, a, g);
    ASSERT_EQ(0U, out_degree(a, g));
    tie(ei, ee) = out_edges(a, g);
    ASSERT_EQ(ei, ee);
}

TEST(ue2_graph, edges_0a) {
    SimpleG g;
    auto p = edges(g);
    ASSERT_EQ(p.first, p.second);
}

TEST(ue2_graph, edges_0b) {
    SimpleG g;
    add_vertex(g);
    ASSERT_EQ(1U, num_vertices(g));
    auto p = edges(g);
    ASSERT_EQ(p.first, p.second);
}

TEST(ue2_graph, edges_0c) {
    SimpleG g;
    add_vertex(g);
    add_vertex(g);
    ASSERT_EQ(2U, num_vertices(g));
    auto p = edges(g);
    ASSERT_EQ(p.first, p.second);
}

TEST(ue2_graph, edges_1a) {
    SimpleG g;
    ASSERT_EQ(0U, num_edges(g));

    auto v = add_vertex(g);

    ASSERT_EQ(0U, num_edges(g));
    auto e1 = add_edge(v, v, g).first;

    SimpleG::edge_iterator ei, ee;

    ASSERT_EQ(1U, num_edges(g));
    tie(ei, ee) = edges(g);
    ASSERT_EQ(e1, *ei++);
    ASSERT_EQ(ee, ei);

    remove_edge(e1, g);

    ASSERT_EQ(0U, num_edges(g));
    tie(ei, ee) = edges(g);
    ASSERT_EQ(ee, ei);
}

TEST(ue2_graph, edges_1b) {
    SimpleG g;
    ASSERT_EQ(0U, num_edges(g));

    auto u = add_vertex(g);
    auto v = add_vertex(g);

    ASSERT_EQ(0U, num_edges(g));
    auto e1 = add_edge(u, v, g).first;

    SimpleG::edge_iterator ei, ee;

    ASSERT_EQ(1U, num_edges(g));
    tie(ei, ee) = edges(g);
    ASSERT_EQ(e1, *ei++);
    ASSERT_EQ(ee, ei);

    remove_edge(e1, g);

    ASSERT_EQ(0U, num_edges(g));
    tie(ei, ee) = edges(g);
    ASSERT_EQ(ee, ei);
}

TEST(ue2_graph, edges_1c) {
    SimpleG g;
    ASSERT_EQ(0U, num_edges(g));

    auto u = add_vertex(g);
    auto v = add_vertex(g);

    ASSERT_EQ(0U, num_edges(g));
    auto e1 = add_edge(v, u, g).first;

    SimpleG::edge_iterator ei, ee;

    ASSERT_EQ(1U, num_edges(g));
    tie(ei, ee) = edges(g);
    ASSERT_EQ(e1, *ei++);
    ASSERT_EQ(ee, ei);

    remove_edge(e1, g);

    ASSERT_EQ(0U, num_edges(g));
    tie(ei, ee) = edges(g);
    ASSERT_EQ(ee, ei);
}

TEST(ue2_graph, edges_1d) {
    SimpleG g;
    ASSERT_EQ(0U, num_edges(g));

    UNUSED auto u = add_vertex(g);
    UNUSED auto v = add_vertex(g);
    auto w = add_vertex(g);
    auto x = add_vertex(g);
    UNUSED auto y = add_vertex(g);
    UNUSED auto z = add_vertex(g);

    ASSERT_EQ(0U, num_edges(g));
    auto e1 = add_edge(w, x, g).first;

    SimpleG::edge_iterator ei, ee;

    ASSERT_EQ(1U, num_edges(g));
    tie(ei, ee) = edges(g);
    ASSERT_EQ(e1, *ei++);
    ASSERT_EQ(ee, ei);

    remove_edge(e1, g);

    ASSERT_EQ(0U, num_edges(g));
    tie(ei, ee) = edges(g);
    ASSERT_EQ(ee, ei);
}

TEST(ue2_graph, edges_2a) {
    SimpleG g;
    ASSERT_EQ(0U, num_edges(g));

    auto v = add_vertex(g);

    ASSERT_EQ(0U, num_edges(g));
    auto e1 = add_edge(v, v, g).first;
    auto e2 = add_edge(v, v, g).first;

    SimpleG::edge_iterator ei, ee;

    ASSERT_EQ(2U, num_edges(g));
    tie(ei, ee) = edges(g);
    ASSERT_EQ(e1, *ei++);
    ASSERT_EQ(e2, *ei++);
    ASSERT_EQ(ee, ei);

    remove_edge(e1, g);

    ASSERT_EQ(1U, num_edges(g));
    tie(ei, ee) = edges(g);
    ASSERT_EQ(e2, *ei++);
    ASSERT_EQ(ee, ei);

    remove_edge(e2, g);

    ASSERT_EQ(0U, num_edges(g));
    tie(ei, ee) = edges(g);
    ASSERT_EQ(ee, ei);
}

TEST(ue2_graph, edges_2b) {
    SimpleG g;
    ASSERT_EQ(0U, num_edges(g));

    auto u = add_vertex(g);
    auto v = add_vertex(g);

    ASSERT_EQ(0U, num_edges(g));
    auto e1 = add_edge(u, v, g).first;
    auto e2 = add_edge(v, u, g).first;

    SimpleG::edge_iterator ei, ee;

    ASSERT_EQ(2U, num_edges(g));
    tie(ei, ee) = edges(g);
    ASSERT_EQ(e1, *ei++);
    ASSERT_EQ(e2, *ei++);
    ASSERT_EQ(ee, ei);

    remove_edge(e1, g);

    ASSERT_EQ(1U, num_edges(g));
    tie(ei, ee) = edges(g);
    ASSERT_EQ(e2, *ei++);
    ASSERT_EQ(ee, ei);

    remove_edge(e2, g);

    ASSERT_EQ(0U, num_edges(g));
    tie(ei, ee) = edges(g);
    ASSERT_EQ(ee, ei);
}

TEST(ue2_graph, edges_2c) {
    SimpleG g;
    ASSERT_EQ(0U, num_edges(g));

    UNUSED auto s = add_vertex(g);
    UNUSED auto t = add_vertex(g);
    auto u = add_vertex(g);
    UNUSED auto v = add_vertex(g);
    auto w = add_vertex(g);
    auto x = add_vertex(g);
    UNUSED auto y = add_vertex(g);
    UNUSED auto z = add_vertex(g);

    ASSERT_EQ(0U, num_edges(g));
    auto e1 = add_edge(w, x, g).first;
    auto e2 = add_edge(u, x, g).first;

    SimpleG::edge_iterator ei, ee;

    ASSERT_EQ(2U, num_edges(g));
    tie(ei, ee) = edges(g);
    ASSERT_EQ(e2, *ei++);
    ASSERT_EQ(e1, *ei++);
    ASSERT_EQ(ee, ei);

    clear_in_edges(x, g);

    ASSERT_EQ(0U, num_edges(g));
    tie(ei, ee) = edges(g);
    ASSERT_EQ(ee, ei);
}

TEST(ue2_graph, edges_3a) {
    SimpleG g;
    ASSERT_EQ(0U, num_edges(g));

    UNUSED auto s = add_vertex(g);
    UNUSED auto t = add_vertex(g);
    auto u = add_vertex(g);
    auto v = add_vertex(g);
    auto w = add_vertex(g);
    auto x = add_vertex(g);
    UNUSED auto y = add_vertex(g);
    auto z = add_vertex(g);

    ASSERT_EQ(0U, num_edges(g));
    auto e1 = add_edge(w, x, g).first;
    auto e2 = add_edge(u, v, g).first;
    auto e3 = add_edge(u, z, g).first;

    SimpleG::edge_iterator ei, ee;

    ASSERT_EQ(3U, num_edges(g));
    tie(ei, ee) = edges(g);
    ASSERT_EQ(e2, *ei++);
    ASSERT_EQ(e3, *ei++);
    ASSERT_EQ(e1, *ei++);
    ASSERT_EQ(ee, ei);

    remove_edge(e1, g);

    ASSERT_EQ(2U, num_edges(g));
    clear_out_edges(u, g);

    ASSERT_EQ(0U, num_edges(g));

    tie(ei, ee) = edges(g);
    ASSERT_EQ(ee, ei);
}

TEST(ue2_graph, degree) {
    SimpleG g;
    auto a = add_vertex(g);
    auto b = add_vertex(g);
    auto c = add_vertex(g);
    auto d = add_vertex(g);

    add_edge(a, b, g);
    add_edge(a, c, g);
    add_edge(a, d, g);

    ASSERT_EQ(3U, degree(a, g));
    ASSERT_EQ(1U, degree(b, g));
    ASSERT_EQ(1U, degree(c, g));
    ASSERT_EQ(1U, degree(d, g));

    add_edge(b, c, g);

    ASSERT_EQ(3U, degree(a, g));
    ASSERT_EQ(2U, degree(b, g));
    ASSERT_EQ(2U, degree(c, g));
    ASSERT_EQ(1U, degree(d, g));

    add_edge(d, d, g);
    ASSERT_EQ(3U, degree(a, g));
    ASSERT_EQ(2U, degree(b, g));
    ASSERT_EQ(2U, degree(c, g));
    ASSERT_EQ(3U, degree(d, g));

    add_edge(b, a, g);
    ASSERT_EQ(4U, degree(a, g));
    ASSERT_EQ(3U, degree(b, g));
    ASSERT_EQ(2U, degree(c, g));
    ASSERT_EQ(3U, degree(d, g));

    add_edge(b, a, g);
    ASSERT_EQ(5U, degree(a, g));
    ASSERT_EQ(4U, degree(b, g));
    ASSERT_EQ(2U, degree(c, g));
    ASSERT_EQ(3U, degree(d, g));

    add_edge(d, d, g);
    ASSERT_EQ(5U, degree(a, g));
    ASSERT_EQ(4U, degree(b, g));
    ASSERT_EQ(2U, degree(c, g));
    ASSERT_EQ(5U, degree(d, g));
}

TEST(ue2_graph, adj) {
    SimpleG g;
    auto a = add_vertex(g);
    auto b = add_vertex(g);
    auto c = add_vertex(g);
    auto d = add_vertex(g);

    add_edge(a, b, g);
    add_edge(a, c, g);
    add_edge(a, d, g);
    add_edge(b, a, g);
    add_edge(b, b, g);

    SimpleG::adjacency_iterator ai, ae;
    tie(ai, ae) = adjacent_vertices(a, g);
    ASSERT_EQ(b, *ai++);
    ASSERT_EQ(c, *ai++);
    ASSERT_EQ(d, *ai++);
    ASSERT_EQ(ai, ae);

    tie(ai, ae) = adjacent_vertices(b, g);
    ASSERT_EQ(a, *ai++);
    ASSERT_EQ(b, *ai++);
    ASSERT_EQ(ai, ae);

    tie(ai, ae) = adjacent_vertices(c, g);
    ASSERT_EQ(ai, ae);

    tie(ai, ae) = adjacent_vertices(d, g);
    ASSERT_EQ(ai, ae);
}

TEST(ue2_graph, inv_adj) {
    SimpleG g;
    auto a = add_vertex(g);
    auto b = add_vertex(g);
    auto c = add_vertex(g);
    auto d = add_vertex(g);

    add_edge(a, b, g);
    add_edge(a, c, g);
    add_edge(a, d, g);
    add_edge(b, a, g);
    add_edge(b, b, g);

    SimpleG::inv_adjacency_iterator ai, ae;
    tie(ai, ae) = inv_adjacent_vertices(a, g);
    ASSERT_EQ(b, *ai++);
    ASSERT_EQ(ai, ae);

    tie(ai, ae) = inv_adjacent_vertices(b, g);
    ASSERT_EQ(a, *ai++);
    ASSERT_EQ(b, *ai++);
    ASSERT_EQ(ai, ae);

    tie(ai, ae) = inv_adjacent_vertices(c, g);
    ASSERT_EQ(a, *ai++);
    ASSERT_EQ(ai, ae);

    tie(ai, ae) = inv_adjacent_vertices(d, g);
    ASSERT_EQ(a, *ai++);
    ASSERT_EQ(ai, ae);
}

TEST(ue2_graph, square_brackets_v) {
    SimpleG g;
    auto a = add_vertex(g);
    auto b = add_vertex(g);
    auto c = add_vertex(g);
    auto d = add_vertex(g);

    ASSERT_EQ(0U, g[a].index);
    ASSERT_EQ(1U, g[b].index);
    ASSERT_EQ(2U, g[c].index);
    ASSERT_EQ(3U, g[d].index);

    ASSERT_EQ("SimpleV", g[a].test_v);
    ASSERT_EQ("SimpleV", g[b].test_v);
    ASSERT_EQ("SimpleV", g[c].test_v);
    ASSERT_EQ("SimpleV", g[d].test_v);

    g[a].test_v = "a";
    g[b].test_v = "b";
    g[c].test_v = "c";
    g[d].test_v = "d";

    ASSERT_EQ("a", g[a].test_v);
    ASSERT_EQ("b", g[b].test_v);
    ASSERT_EQ("c", g[c].test_v);
    ASSERT_EQ("d", g[d].test_v);
}

TEST(ue2_graph, square_brackets_e) {
    SimpleG g;
    auto u = add_vertex(g);
    auto v = add_vertex(g);
    auto a = add_edge(u, v, g).first;
    auto b = add_edge(u, v, g).first;
    auto c = add_edge(u, u, g).first;
    auto d = add_edge(v, u, g).first;

    ASSERT_EQ(0U, g[a].index);
    ASSERT_EQ(1U, g[b].index);
    ASSERT_EQ(2U, g[c].index);
    ASSERT_EQ(3U, g[d].index);

    ASSERT_EQ("SimpleE", g[a].test_e);
    ASSERT_EQ("SimpleE", g[b].test_e);
    ASSERT_EQ("SimpleE", g[c].test_e);
    ASSERT_EQ("SimpleE", g[d].test_e);

    g[a].test_e = "a";
    g[b].test_e = "b";
    g[c].test_e = "c";
    g[d].test_e = "d";

    ASSERT_EQ("a", g[a].test_e);
    ASSERT_EQ("b", g[b].test_e);
    ASSERT_EQ("c", g[c].test_e);
    ASSERT_EQ("d", g[d].test_e);
}

TEST(ue2_graph, vertex_ordering_1) {
    SimpleG g;
    auto a = add_vertex(g);
    auto b = add_vertex(g);
    auto c = add_vertex(g);
    auto d = add_vertex(g);

    ASSERT_LE(a, b);
    ASSERT_LE(a, c);
    ASSERT_LE(a, d);
    ASSERT_LE(b, c);
    ASSERT_LE(b, d);
    ASSERT_LE(c, d);

    g[a].index = 5;
    g[b].index = 0;
    g[c].index = 3;
    g[d].index = 1;

    ASSERT_LE(a, b);
    ASSERT_LE(a, c);
    ASSERT_LE(a, d);
    ASSERT_LE(b, c);
    ASSERT_LE(b, d);
    ASSERT_LE(c, d);
}

TEST(ue2_graph, vertex_ordering_2) {
    SimpleG g;
    auto a = add_vertex(g);
    auto b = add_vertex(g);
    auto c = add_vertex(g);
    auto d = add_vertex(g);

    set<SimpleG::vertex_descriptor> s;
    s.insert(a);
    s.insert(b);
    s.insert(c);
    s.insert(d);

    auto it = s.begin();
    ASSERT_EQ(a, *it++);
    ASSERT_EQ(b, *it++);
    ASSERT_EQ(c, *it++);
    ASSERT_EQ(d, *it++);
    ASSERT_EQ(it, s.end());

    g[a].index = 5;
    g[b].index = 0;
    g[c].index = 3;
    g[d].index = 1;

    it = s.begin();
    ASSERT_EQ(a, *it++);
    ASSERT_EQ(b, *it++);
    ASSERT_EQ(c, *it++);
    ASSERT_EQ(d, *it++);
    ASSERT_EQ(it, s.end());
}

TEST(ue2_graph, get_v_2_arg) {
    SimpleG g;
    auto a = add_vertex(g);
    auto b = add_vertex(g);

    auto pm = get(&SimpleV::test_v, g);

    ASSERT_EQ("SimpleV", pm[a]);
    ASSERT_EQ("SimpleV", pm[b]);

    pm[a] = "a";
    pm[b] = "b";

    ASSERT_EQ("a", pm[a]);
    ASSERT_EQ("b", pm[b]);

    ASSERT_EQ("a", g[a].test_v);
    ASSERT_EQ("b", g[b].test_v);

    g[a].test_v = "X";
    g[b].test_v = "Y";

    ASSERT_EQ("X", pm[a]);
    ASSERT_EQ("Y", pm[b]);

    ASSERT_EQ("X", get(pm, a));
    ASSERT_EQ("Y", get(pm, b));

    put(pm, a, "A");
    put(pm, b, "B");

    ASSERT_EQ("A", g[a].test_v);
    ASSERT_EQ("B", g[b].test_v);
}

TEST(ue2_graph, get_v_2_arg_const) {
    SimpleG g;
    const SimpleG &gg = g;
    auto a = add_vertex(g);
    auto b = add_vertex(g);

    auto pm = get(&SimpleV::test_v, gg);

    ASSERT_EQ("SimpleV", pm[a]);
    ASSERT_EQ("SimpleV", pm[b]);

    g[a].test_v = "a";
    g[b].test_v = "b";

    ASSERT_EQ("a", pm[a]);
    ASSERT_EQ("b", pm[b]);

    ASSERT_EQ("a", get(pm, a));
    ASSERT_EQ("b", get(pm, b));
}

TEST(ue2_graph, get_e_2_arg) {
    SimpleG g;
    auto u = add_vertex(g);
    auto v = add_vertex(g);
    auto a = add_edge(u, v, g).first;
    auto b = add_edge(v, u, g).first;

    auto pm = get(&SimpleE::test_e, g);

    ASSERT_EQ("SimpleE", pm[a]);
    ASSERT_EQ("SimpleE", pm[b]);

    pm[a] = "a";
    pm[b] = "b";

    ASSERT_EQ("a", pm[a]);
    ASSERT_EQ("b", pm[b]);

    ASSERT_EQ("a", g[a].test_e);
    ASSERT_EQ("b", g[b].test_e);

    g[a].test_e = "X";
    g[b].test_e = "Y";

    ASSERT_EQ("X", pm[a]);
    ASSERT_EQ("Y", pm[b]);

    ASSERT_EQ("X", get(pm, a));
    ASSERT_EQ("Y", get(pm, b));

    put(pm, a, "A");
    put(pm, b, "B");

    ASSERT_EQ("A", g[a].test_e);
    ASSERT_EQ("B", g[b].test_e);
}

TEST(ue2_graph, get_e_2_arg_const) {
    SimpleG g;
    const SimpleG &gg = g;
    auto u = add_vertex(g);
    auto v = add_vertex(g);
    auto a = add_edge(u, v, g).first;
    auto b = add_edge(v, u, g).first;

    auto pm = get(&SimpleE::test_e, gg);

    ASSERT_EQ("SimpleE", pm[a]);
    ASSERT_EQ("SimpleE", pm[b]);

    g[a].test_e = "a";
    g[b].test_e = "b";

    ASSERT_EQ("a", pm[a]);
    ASSERT_EQ("b", pm[b]);

    ASSERT_EQ("a", get(pm, a));
    ASSERT_EQ("b", get(pm, b));
}

TEST(ue2_graph, get_v_3_arg) {
    SimpleG g;
    auto a = add_vertex(g);
    auto b = add_vertex(g);

    ASSERT_EQ("SimpleV", get(&SimpleV::test_v, g, a));
    ASSERT_EQ("SimpleV", get(&SimpleV::test_v, g, a));

    get(&SimpleV::test_v, g, a) = "a";
    get(&SimpleV::test_v, g, b) = "b";

    ASSERT_EQ("a", get(&SimpleV::test_v, g, a));
    ASSERT_EQ("b", get(&SimpleV::test_v, g, b));

    ASSERT_EQ("a", g[a].test_v);
    ASSERT_EQ("b", g[b].test_v);

    g[a].test_v = "X";
    g[b].test_v = "Y";

    ASSERT_EQ("X", get(&SimpleV::test_v, g, a));
    ASSERT_EQ("Y", get(&SimpleV::test_v, g, b));

    //std::decay<decltype(get(&SimpleV::test_v, g)[a])>::type x = "A";

    put(&SimpleV::test_v, g, a, "A");
    put(&SimpleV::test_v, g, b, "B");

    ASSERT_EQ("A", g[a].test_v);
    ASSERT_EQ("B", g[b].test_v);
}

TEST(ue2_graph, get_v_3_arg_const) {
    SimpleG g;
    const SimpleG &gg = g;
    auto a = add_vertex(g);
    auto b = add_vertex(g);

    ASSERT_EQ("SimpleV", get(&SimpleV::test_v, gg, a));
    ASSERT_EQ("SimpleV", get(&SimpleV::test_v, gg, b));

    g[a].test_v = "a";
    g[b].test_v = "b";

    ASSERT_EQ("a", get(&SimpleV::test_v, gg, a));
    ASSERT_EQ("b", get(&SimpleV::test_v, gg, b));
}

TEST(ue2_graph, get_e_3_arg) {
    SimpleG g;
    auto u = add_vertex(g);
    auto v = add_vertex(g);
    auto a = add_edge(u, v, g).first;
    auto b = add_edge(v, u, g).first;

    ASSERT_EQ("SimpleE", get(&SimpleE::test_e, g, a));
    ASSERT_EQ("SimpleE", get(&SimpleE::test_e, g, b));

    get(&SimpleE::test_e, g, a) = "a";
    get(&SimpleE::test_e, g, b) = "b";

    ASSERT_EQ("a", get(&SimpleE::test_e, g, a));
    ASSERT_EQ("b", get(&SimpleE::test_e, g, b));

    ASSERT_EQ("a", g[a].test_e);
    ASSERT_EQ("b", g[b].test_e);

    g[a].test_e = "X";
    g[b].test_e = "Y";

    ASSERT_EQ("X", get(&SimpleE::test_e, g, a));
    ASSERT_EQ("Y", get(&SimpleE::test_e, g, b));
}

TEST(ue2_graph, get_e_3_arg_const) {
    SimpleG g;
    const SimpleG &gg = g;
    auto u = add_vertex(g);
    auto v = add_vertex(g);
    auto a = add_edge(u, v, g).first;
    auto b = add_edge(v, u, g).first;

    ASSERT_EQ("SimpleE", get(&SimpleE::test_e, gg, a));
    ASSERT_EQ("SimpleE", get(&SimpleE::test_e, gg, b));

    g[a].test_e = "a";
    g[b].test_e = "b";

    ASSERT_EQ("a", get(&SimpleE::test_e, gg, a));
    ASSERT_EQ("b", get(&SimpleE::test_e, gg, b));
}

TEST(ue2_graph, get_vertex_index) {
    SimpleG g;
    auto a = add_vertex(g);
    auto pm = get(vertex_index, g);
    ASSERT_EQ(0U, pm(a));
    pm(a) = 1;
    ASSERT_EQ(1U, pm[a]);
    ASSERT_EQ(1U, g[a].index);
    ASSERT_EQ(1U, get(vertex_index, g, a));
}

TEST(ue2_graph, get_vertex_index_const) {
    SimpleG g;
    const SimpleG &gg = g;
    auto a = add_vertex(g);
    auto pm = get(vertex_index, gg);
    ASSERT_EQ(0U, pm(a));
    g[a].index = 1;
    ASSERT_EQ(1U, pm[a]);
    ASSERT_EQ(1U, get(vertex_index, gg, a));
}

TEST(ue2_graph, get_edge_index) {
    SimpleG g;
    auto u = add_vertex(g);
    auto v = add_vertex(g);
    auto a = add_edge(u, v, g).first;
    auto pm = get(edge_index, g);
    ASSERT_EQ(0U, pm(a));
    pm(a) = 1;
    ASSERT_EQ(1U, pm[a]);
    ASSERT_EQ(1U, g[a].index);
    ASSERT_EQ(1U, get(edge_index, g, a));
}

TEST(ue2_graph, get_edge_index_const) {
    SimpleG g;
    const SimpleG &gg = g;
    auto u = add_vertex(g);
    auto v = add_vertex(g);
    auto a = add_edge(u, v, g).first;
    auto pm = get(edge_index, gg);
    ASSERT_EQ(0U, pm(a));
    g[a].index = 1;
    ASSERT_EQ(1U, pm[a]);
    ASSERT_EQ(1U, get(edge_index, gg, a));
}

TEST(ue2_graph, get_vertex_all) {
    SimpleG g;
    auto a = add_vertex(g);
    auto pm = get(vertex_all, g);
    ASSERT_EQ(0U, pm(a).index);
    pm(a).index = 1;
    ASSERT_EQ(1U, pm[a].index);
    ASSERT_EQ(1U, g[a].index);
    ASSERT_EQ(1U, get(vertex_all, g, a).index);
    auto &a_all = get(vertex_all, g, a);
    ASSERT_EQ(1U, a_all.index);
    g[a].index = 2;
    ASSERT_EQ(2U, a_all.index);
}

TEST(ue2_graph, get_vertex_all_const) {
    SimpleG g;
    const SimpleG &gg = g;
    auto a = add_vertex(g);
    auto pm = get(vertex_all, gg);
    ASSERT_EQ(0U, pm(a).index);
    g[a].index = 1;
    ASSERT_EQ(1U, pm[a].index);
    ASSERT_EQ(1U, get(vertex_all, gg, a).index);
    auto &a_all = get(vertex_all, gg, a);
    ASSERT_EQ(1U, a_all.index);
    g[a].index = 2;
    ASSERT_EQ(2U, a_all.index);
}

TEST(ue2_graph, get_vertex_bundle) {
    SimpleG g;
    auto a = add_vertex(g);
    auto pm = get(vertex_bundle, g);
    ASSERT_EQ(0U, pm(a).index);
    pm(a).index = 1;
    ASSERT_EQ(1U, pm[a].index);
    ASSERT_EQ(1U, g[a].index);
    ASSERT_EQ(1U, get(vertex_bundle, g, a).index);
    auto &a_bundle = get(vertex_bundle, g, a);
    ASSERT_EQ(1U, a_bundle.index);
    g[a].index = 2;
    ASSERT_EQ(2U, a_bundle.index);
}

TEST(ue2_graph, get_vertex_bundle_const) {
    SimpleG g;
    const SimpleG &gg = g;
    auto a = add_vertex(g);
    auto pm = get(vertex_bundle, gg);
    ASSERT_EQ(0U, pm(a).index);
    g[a].index = 1;
    ASSERT_EQ(1U, pm[a].index);
    ASSERT_EQ(1U, get(vertex_bundle, gg, a).index);
    auto &a_bundle = get(vertex_bundle, gg, a);
    ASSERT_EQ(1U, a_bundle.index);
    g[a].index = 2;
    ASSERT_EQ(2U, a_bundle.index);
}

TEST(ue2_graph, get_edge_all) {
    SimpleG g;
    auto u = add_vertex(g);
    auto v = add_vertex(g);
    auto a = add_edge(u, v, g).first;
    auto pm = get(edge_all, g);
    ASSERT_EQ(0U, pm(a).index);
    pm(a).index = 1;
    ASSERT_EQ(1U, pm[a].index);
    ASSERT_EQ(1U, g[a].index);
    ASSERT_EQ(1U, get(edge_all, g, a).index);
    auto &a_all = get(edge_all, g, a);
    ASSERT_EQ(1U, a_all.index);
    g[a].index = 2;
    ASSERT_EQ(2U, a_all.index);
}

TEST(ue2_graph, get_edge_all_const) {
    SimpleG g;
    const SimpleG &gg = g;
    auto u = add_vertex(g);
    auto v = add_vertex(g);
    auto a = add_edge(u, v, g).first;
    auto pm = get(edge_all, gg);
    ASSERT_EQ(0U, pm(a).index);
    g[a].index = 1;
    ASSERT_EQ(1U, pm[a].index);
    ASSERT_EQ(1U, get(edge_all, gg, a).index);
    auto &a_all = get(edge_all, gg, a);
    ASSERT_EQ(1U, a_all.index);
    g[a].index = 2;
    ASSERT_EQ(2U, a_all.index);
}

TEST(ue2_graph, get_edge_bundle) {
    SimpleG g;
    auto u = add_vertex(g);
    auto v = add_vertex(g);
    auto a = add_edge(u, v, g).first;
    auto pm = get(edge_bundle, g);
    ASSERT_EQ(0U, pm(a).index);
    pm(a).index = 1;
    ASSERT_EQ(1U, pm[a].index);
    ASSERT_EQ(1U, g[a].index);
    ASSERT_EQ(1U, get(edge_bundle, g, a).index);
    auto &a_bundle = get(edge_bundle, g, a);
    ASSERT_EQ(1U, a_bundle.index);
    g[a].index = 2;
    ASSERT_EQ(2U, a_bundle.index);
}

TEST(ue2_graph, get_edge_bundle_const) {
    SimpleG g;
    const SimpleG &gg = g;
    auto u = add_vertex(g);
    auto v = add_vertex(g);
    auto a = add_edge(u, v, g).first;
    auto pm = get(edge_bundle, gg);
    ASSERT_EQ(0U, pm(a).index);
    g[a].index = 1;
    ASSERT_EQ(1U, pm[a].index);
    ASSERT_EQ(1U, get(edge_bundle, gg, a).index);
    auto &a_bundle = get(edge_bundle, gg, a);
    ASSERT_EQ(1U, a_bundle.index);
    g[a].index = 2;
    ASSERT_EQ(2U, a_bundle.index);
}

TEST(ue2_graph, add_vertex_prop) {
    SimpleG g;
    SimpleV vp;
    vp.index = 42;
    vp.test_v = "prop";
    auto u = add_vertex(vp, g);
    auto v = add_vertex(vp, g);

    ASSERT_EQ(0U, g[u].index);
    ASSERT_EQ(1U, g[v].index);

    ASSERT_EQ("prop", g[u].test_v);
    ASSERT_EQ("prop", g[v].test_v);
}

TEST(ue2_graph, add_edge_prop) {
    SimpleG g;
    SimpleE ep;
    ep.index = 42;
    ep.test_e = "prop";
    auto u = add_vertex(g);
    auto v = add_vertex(g);

    auto e = add_edge(u, v, ep, g).first;
    auto f = add_edge(u, v, ep, g).first;

    ASSERT_EQ(0U, g[e].index);
    ASSERT_EQ(1U, g[f].index);

    ASSERT_EQ("prop", g[e].test_e);
    ASSERT_EQ("prop", g[f].test_e);
}

TEST(ue2_graph, reverse_graph) {
    SimpleG g;
    auto a = add_vertex(g);
    auto b = add_vertex(g);
    auto e = add_edge(a, b, g).first;
    reverse_graph<SimpleG, SimpleG &> rg(g);
    auto index_map = get(vertex_index, rg);

    ASSERT_EQ(0U, rg[a].index);
    ASSERT_EQ(1U, rg[b].index);
    ASSERT_EQ(0U, rg[e].index);

    ASSERT_EQ(0U, get(vertex_index, rg, a));
    ASSERT_EQ(1U, get(vertex_index, rg, b));
    ASSERT_EQ(0U, get(edge_index, rg, edge(b, a, rg).first));

    ASSERT_EQ(0U, index_map(a));
    ASSERT_EQ(1U, index_map(b));

    ASSERT_TRUE(edge(b, a, rg).second);
    ASSERT_FALSE(edge(a, b, rg).second);
}

TEST(ue2_graph, reverse_graph_const) {
    SimpleG g;
    auto a = add_vertex(g);
    auto b = add_vertex(g);
    auto e = add_edge(a, b, g).first;
    reverse_graph<const SimpleG, const SimpleG &> rg(g);
    auto index_map = get(&SimpleV::index, rg);

    // Note: reverse_graph fails to make bundles const so things break.
    // ASSERT_EQ(0U, rg[a].index);
    // ASSERT_EQ(1U, rg[b].index);
    // ASSERT_EQ(0U, rg[e].index);

    ASSERT_EQ(0U, get(vertex_index, g, a));
    ASSERT_EQ(1U, get(vertex_index, g, b));
    ASSERT_EQ(0U, get(edge_index, g, e));

    ASSERT_EQ(0U, index_map(a));
    ASSERT_EQ(1U, index_map(b));

    ASSERT_TRUE(edge(b, a, rg).second);
    ASSERT_FALSE(edge(a, b, rg).second);
}

TEST(ue2_graph, default_param) {
    struct TestGraph : ue2_graph<TestGraph> { };
    TestGraph g;

    auto v = add_vertex(g);
    auto e = add_edge(v, v, g).first;

    ASSERT_EQ(0U, get(vertex_index, g, v));
    ASSERT_EQ(0U, get(edge_index, g, e));
#if !defined(_MSC_VER)
    /* This makes MSVC up to VS2015 sad in ways that shouldn't happen. */
    ASSERT_EQ(0U, get(&ue2::graph_detail::default_edge_property::index, g, e));
#endif
}
