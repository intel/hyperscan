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

#include "config.h"
#include "gtest/gtest.h"
#include "util/graph.h"

#include <boost/graph/adjacency_iterator.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

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

    ASSERT_TRUE( hasGreaterInDegree(0, a, g));
    ASSERT_FALSE(hasGreaterInDegree(1, a, g));
    ASSERT_TRUE( hasGreaterInDegree(2, b, g));
    ASSERT_FALSE(hasGreaterInDegree(3, b, g));
    ASSERT_TRUE( hasGreaterInDegree(1, c, g));
    ASSERT_FALSE(hasGreaterInDegree(2, c, g));
    ASSERT_FALSE(hasGreaterInDegree(0, d, g));
    ASSERT_TRUE( hasGreaterInDegree(1, e, g));
    ASSERT_FALSE(hasGreaterInDegree(2, e, g));
    ASSERT_FALSE(hasGreaterInDegree(0, f, g));

    ASSERT_TRUE( hasGreaterOutDegree(0, a, g));
    ASSERT_FALSE(hasGreaterOutDegree(1, a, g));
    ASSERT_TRUE( hasGreaterOutDegree(1, b, g));
    ASSERT_FALSE(hasGreaterOutDegree(2, b, g));
    ASSERT_FALSE(hasGreaterOutDegree(0, c, g));
    ASSERT_TRUE( hasGreaterOutDegree(0, d, g));
    ASSERT_FALSE(hasGreaterOutDegree(1, d, g));
    ASSERT_TRUE( hasGreaterOutDegree(0, e, g));
    ASSERT_FALSE(hasGreaterOutDegree(1, e, g));
    ASSERT_TRUE( hasGreaterOutDegree(2, f, g));
    ASSERT_FALSE(hasGreaterOutDegree(3, f, g));
}

TEST(graph_util, in_degree_equal_to_1) {
    unit_graph g;

    unit_vertex a = add_vertex(g);
    unit_vertex b = add_vertex(g);
    unit_vertex c = add_vertex(g);
    unit_vertex d = add_vertex(g);

    ASSERT_TRUE(in_degree_equal_to(a, g, 0));
    ASSERT_FALSE(in_degree_equal_to(a, g, 1));
    ASSERT_FALSE(in_degree_equal_to(a, g, 2));

    add_edge(b, a, g);

    ASSERT_FALSE(in_degree_equal_to(a, g, 0));
    ASSERT_TRUE(in_degree_equal_to(a, g, 1));
    ASSERT_FALSE(in_degree_equal_to(a, g, 2));

    add_edge(c, a, g);

    ASSERT_FALSE(in_degree_equal_to(a, g, 0));
    ASSERT_FALSE(in_degree_equal_to(a, g, 1));
    ASSERT_TRUE(in_degree_equal_to(a, g, 2));

    add_edge(d, a, g);

    ASSERT_FALSE(in_degree_equal_to(a, g, 0));
    ASSERT_FALSE(in_degree_equal_to(a, g, 1));
    ASSERT_FALSE(in_degree_equal_to(a, g, 2));
}

TEST(graph_util, edge_by_target_1) {
    unit_graph g;

    unit_vertex a = add_vertex(g);
    unit_vertex b = add_vertex(g);
    unit_vertex c = add_vertex(g);

    ASSERT_FALSE(edge_by_target(a, a, g).second);
    ASSERT_FALSE(edge_by_target(a, b, g).second);
    ASSERT_FALSE(edge_by_target(a, c, g).second);
    ASSERT_FALSE(edge_by_target(b, a, g).second);
    ASSERT_FALSE(edge_by_target(c, b, g).second);

    unit_edge ab = add_edge(a, b, g).first;

    ASSERT_FALSE(edge_by_target(a, a, g).second);
    ASSERT_TRUE(edge_by_target(a, b, g).second);
    ASSERT_TRUE(ab == edge_by_target(a, b, g).first);
    ASSERT_FALSE(edge_by_target(a, c, g).second);
    ASSERT_FALSE(edge_by_target(b, a, g).second);
    ASSERT_FALSE(edge_by_target(b, b, g).second);
    ASSERT_FALSE(edge_by_target(c, b, g).second);

    unit_edge cb = add_edge(c, b, g).first;

    ASSERT_FALSE(edge_by_target(a, a, g).second);
    ASSERT_TRUE(edge_by_target(a, b, g).second);
    ASSERT_TRUE(ab == edge_by_target(a, b, g).first);
    ASSERT_FALSE(edge_by_target(a, c, g).second);
    ASSERT_FALSE(edge_by_target(b, a, g).second);
    ASSERT_FALSE(edge_by_target(b, b, g).second);
    ASSERT_TRUE(edge_by_target(c, b, g).second);
    ASSERT_TRUE(cb == edge_by_target(c, b, g).first);

    unit_edge aa = add_edge(a, a, g).first;
    unit_edge bb = add_edge(b, b, g).first;

    ASSERT_TRUE(edge_by_target(a, a, g).second);
    ASSERT_TRUE(aa == edge_by_target(a, a, g).first);
    ASSERT_TRUE(edge_by_target(a, b, g).second);
    ASSERT_TRUE(ab == edge_by_target(a, b, g).first);
    ASSERT_FALSE(edge_by_target(a, c, g).second);
    ASSERT_FALSE(edge_by_target(b, a, g).second);
    ASSERT_TRUE(edge_by_target(b, b, g).second);
    ASSERT_TRUE(bb == edge_by_target(b, b, g).first);
    ASSERT_TRUE(edge_by_target(c, b, g).second);
    ASSERT_TRUE(cb == edge_by_target(c, b, g).first);
}
