/*
 * Copyright (c) 2015-2018, Intel Corporation
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
#include "util/container.h"
#include "util/graph.h"
#include "util/graph_range.h"
#include "util/graph_undirected.h"
#include "util/ue2_graph.h"

#include <boost/graph/adjacency_list.hpp>

using namespace std;
using namespace ue2;

struct SimpleV {
    size_t index;
    string test_v = "SimpleV";
};

struct SimpleE {
    size_t index;
    string test_e = "SimpleE";
};

struct SimpleG : public ue2_graph<SimpleG, SimpleV, SimpleE> {};

using SimpleVertex = SimpleG::vertex_descriptor;

template<typename Graph, typename Range>
vector<size_t> to_indices(const Range &range, const Graph &g) {
    vector<size_t> indices;
    for (const auto &elem : range) {
        indices.push_back(g[elem].index);
    }
    sort(indices.begin(), indices.end());
    return indices;
}

template<typename Graph, typename T>
vector<size_t> to_indices(const std::initializer_list<T> &range,
                          const Graph &g) {
    vector<size_t> indices;
    for (const auto &elem : range) {
        indices.push_back(g[elem].index);
    }
    sort(indices.begin(), indices.end());
    return indices;
}

TEST(graph_undirected, simple_ue2_graph) {
    SimpleG g;
    auto a = add_vertex(g);
    ASSERT_NE(SimpleG::null_vertex(), a);
    auto b = add_vertex(g);
    ASSERT_NE(SimpleG::null_vertex(), b);
    auto c = add_vertex(g);
    ASSERT_NE(SimpleG::null_vertex(), c);

    add_edge(a, b, g);
    add_edge(b, a, g);
    add_edge(a, c, g);
    add_edge(c, b, g);
    add_edge(c, c, g);

    auto ug = make_undirected_graph(g);

    ASSERT_EQ(3, num_vertices(ug));
    ASSERT_EQ(4, num_edges(ug));

    // Check adjacencies

    ASSERT_EQ(2, out_degree(a, ug));
    ASSERT_EQ(to_indices({b, c}, ug),
              to_indices(adjacent_vertices_range(a, ug), ug));

    ASSERT_EQ(2, out_degree(b, ug));
    ASSERT_EQ(to_indices({a, c}, ug),
              to_indices(adjacent_vertices_range(b, ug), ug));

    ASSERT_EQ(3, out_degree(c, ug));
    ASSERT_EQ(to_indices({a, b, c}, ug),
              to_indices(adjacent_vertices_range(c, ug), ug));

    ASSERT_EQ(2, in_degree(b, ug));
    ASSERT_EQ(to_indices({a, c}, ug),
              to_indices(inv_adjacent_vertices_range(b, ug), ug));

    // Test reverse edge existence

    ASSERT_TRUE(edge(a, b, ug).second);
    ASSERT_TRUE(edge(b, a, ug).second);
    ASSERT_TRUE(edge(a, c, ug).second);
    ASSERT_TRUE(edge(c, a, ug).second); // (a,c) actually exists
    ASSERT_TRUE(edge(b, c, ug).second); // (c,b) actually exists
    ASSERT_FALSE(edge(a, a, ug).second);

    // Vertex properties

    g[c].test_v = "vertex c";
    ASSERT_EQ("vertex c", ug[c].test_v);
    ASSERT_EQ("vertex c", get(&SimpleV::test_v, ug, c));

    ug[c].test_v = "vertex c again";
    ASSERT_EQ("vertex c again", g[c].test_v);
    ASSERT_EQ("vertex c again", get(&SimpleV::test_v, g, c));

    put(&SimpleV::test_v, ug, c, "vertex c once more");
    ASSERT_EQ("vertex c once more", g[c].test_v);

    const auto &vprops1 = ug[b];
    ASSERT_EQ(1, vprops1.index);

    const auto &vprops2 = get(boost::vertex_all, ug, b);
    ASSERT_EQ(1, vprops2.index);

    // Edge Properties

    auto edge_undirected = edge(a, b, ug).first;
    ug[edge_undirected].test_e = "edge (a,b)";
    ASSERT_EQ("edge (a,b)", ug[edge_undirected].test_e);
    ASSERT_EQ("edge (a,b)", get(&SimpleE::test_e, ug, edge_undirected));

    ug[edge_undirected].test_e = "edge (a,b) again";
    put(&SimpleE::test_e, ug, edge_undirected, "edge (a,b) once more");
}

TEST(graph_undirected, simple_adjacency_list) {
    using AdjListG =
        boost::adjacency_list<boost::listS, boost::listS, boost::bidirectionalS,
                              SimpleV, SimpleE>;

    AdjListG g;
    auto a = add_vertex(g);
    ASSERT_NE(AdjListG::null_vertex(), a);
    g[a].index = 0;
    auto b = add_vertex(g);
    ASSERT_NE(AdjListG::null_vertex(), b);
    g[b].index = 1;
    auto c = add_vertex(g);
    ASSERT_NE(AdjListG::null_vertex(), c);
    g[c].index = 2;

    add_edge(a, b, g);
    add_edge(b, a, g);
    add_edge(a, c, g);
    add_edge(c, b, g);
    add_edge(c, c, g);

    auto ug = make_undirected_graph(g);

    ASSERT_EQ(3, num_vertices(ug));
    ASSERT_EQ(4, num_edges(ug));

    // Check adjacencies

    ASSERT_EQ(2, out_degree(a, ug));
    ASSERT_EQ(to_indices({b, c}, ug),
              to_indices(adjacent_vertices_range(a, ug), ug));

    ASSERT_EQ(2, out_degree(b, ug));
    ASSERT_EQ(to_indices({a, c}, ug),
              to_indices(adjacent_vertices_range(b, ug), ug));

    ASSERT_EQ(3, out_degree(c, ug));
    ASSERT_EQ(to_indices({a, b, c}, ug),
              to_indices(adjacent_vertices_range(c, ug), ug));

    ASSERT_EQ(2, in_degree(b, ug));
    ASSERT_EQ(to_indices({a, c}, ug),
              to_indices(inv_adjacent_vertices_range(b, ug), ug));

    // Test reverse edge existence

    ASSERT_TRUE(edge(a, b, ug).second);
    ASSERT_TRUE(edge(b, a, ug).second);
    ASSERT_TRUE(edge(a, c, ug).second);
    ASSERT_TRUE(edge(c, a, ug).second); // (a,c) actually exists
    ASSERT_TRUE(edge(b, c, ug).second); // (c,b) actually exists
    ASSERT_FALSE(edge(a, a, ug).second);

    // Vertex properties

    g[c].test_v = "vertex c";
    ASSERT_EQ("vertex c", ug[c].test_v);
    ASSERT_EQ("vertex c", get(&SimpleV::test_v, ug, c));

    ug[c].test_v = "vertex c again";
    ASSERT_EQ("vertex c again", g[c].test_v);
    ASSERT_EQ("vertex c again", get(&SimpleV::test_v, g, c));

    put(&SimpleV::test_v, ug, c, "vertex c once more");
    ASSERT_EQ("vertex c once more", g[c].test_v);

    const auto &vprops1 = ug[b];
    ASSERT_EQ(1, vprops1.index);

    const auto &vprops2 = get(boost::vertex_all, ug, b);
    ASSERT_EQ(1, vprops2.index);

    // Edge Properties

    auto edge_undirected = edge(a, b, ug).first;
    ug[edge_undirected].test_e = "edge (a,b)";
    ASSERT_EQ("edge (a,b)", ug[edge_undirected].test_e);
    ASSERT_EQ("edge (a,b)", get(&SimpleE::test_e, ug, edge_undirected));

    ug[edge_undirected].test_e = "edge (a,b) again";
    put(&SimpleE::test_e, ug, edge_undirected, "edge (a,b) once more");
}
