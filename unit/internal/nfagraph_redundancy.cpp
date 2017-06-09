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

/**
 * Unit tests for checking the removeRedundancy code in
 * nfagraph/ng_redundancy.cpp.
 */

#include "config.h"
#include "gtest/gtest.h"
#include "nfagraph_common.h"
#include "grey.h"
#include "hs.h"
#include "parser/Component.h"
#include "parser/Parser.h"
#include "compiler/compiler.h"
#include "nfagraph/ng.h"
#include "nfagraph/ng_builder.h"
#include "nfagraph/ng_redundancy.h"
#include "nfagraph/ng_edge_redundancy.h"
#include "util/target_info.h"

using namespace std;
using namespace ue2;

TEST(NFAGraph, RemoveRedundancy1) {
    // Build a small graph with a redundant vertex: (a|b)c
    // The character reachability should be merged into: [ab]c
    CompileContext cc(false, false, get_current_target(), Grey());

    auto graph(constructGraphWithCC("(a|b)c", cc, 0));
    ASSERT_TRUE(graph.get() != nullptr);
    NGHolder &g = *graph;

    // Run removeRedundancy
    removeRedundancy(g, SOM_NONE);

    // Our graph should only have two non-special nodes
    ASSERT_EQ((size_t)N_SPECIALS + 2, num_vertices(g));

    // Dot-star start state should be connected to itself and a single other
    // vertex
    ASSERT_EQ(2U, out_degree(graph->startDs, g));

    // That single vertex should have reachability [ab]
    NFAVertex v = NGHolder::null_vertex();
    for (NFAVertex t : adjacent_vertices_range(graph->startDs, g)) {
        v = t;
        if (v != graph->startDs) {
            break;
        }
    }
    const CharReach &cr = g[v].char_reach;
    ASSERT_EQ(2U, cr.count());
    ASSERT_TRUE(cr.test('a'));
    ASSERT_TRUE(cr.test('b'));

    // There should be a single edge from v to a node with char reachability 'c'
    ASSERT_EQ(1U, out_degree(v, g));
    NFAVertex v2 = *(adjacent_vertices(v, g).first);
    const CharReach &cr2 = g[v2].char_reach;
    ASSERT_EQ(1U, cr2.count());
    ASSERT_TRUE(cr2.test('c'));

    // 'c' should have an edge to accept
    ASSERT_TRUE(edge(v2, graph->accept, g).second);
}

TEST(NFAGraph, RemoveRedundancy2) {
    // Build a small graph with a redundant vertex: a.*b?c
    // The dot-star should swallow the 'b?', leaving a.*c
    CompileContext cc(false, false, get_current_target(), Grey());
    auto graph(constructGraphWithCC("a.*b?c", cc, HS_FLAG_DOTALL));
    ASSERT_TRUE(graph.get() != nullptr);
    NGHolder &g = *graph;

    // Run removeRedundancy
    removeRedundancy(g, SOM_NONE);

    // Our graph should now have only 3 non-special vertices
    ASSERT_EQ((size_t)N_SPECIALS + 3, num_vertices(g));

    // Dot-star start state should be connected to itself and a single other
    // vertex
    ASSERT_EQ(2U, out_degree(graph->startDs, g));

    // That single vertex should have reachability [a]
    NFAVertex v = NGHolder::null_vertex();
    for (NFAVertex t : adjacent_vertices_range(graph->startDs, g)) {
        v = t;
        if (v != graph->startDs) {
            break;
        }
    }
    const CharReach &cr = g[v].char_reach;
    ASSERT_EQ(1U, cr.count());
    ASSERT_TRUE(cr.test('a'));

    // 'a' should have two out edges: one to a dot with a cycle (.*) and one to
    // 'c'
    ASSERT_EQ(2U, out_degree(v, g));
    NFAVertex dotstar = NGHolder::null_vertex();
    NFAVertex vc = NGHolder::null_vertex();
    for (NFAVertex t : adjacent_vertices_range(v, g)) {
        const CharReach &cr2 = g[t].char_reach;
        if (cr2.count() == 1 && cr2.test('c')) {
            vc = t;
        } else if (cr2.all()) {
            dotstar = t;
        } else {
            FAIL();
        }
    }
    ASSERT_TRUE(vc != NGHolder::null_vertex());
    ASSERT_TRUE(dotstar != NGHolder::null_vertex());

    // Dot-star node should have a self-loop and an edge to vertex 'c'
    ASSERT_EQ(2U, out_degree(dotstar, g));
    ASSERT_EQ(2U, in_degree(vc, g));
    ASSERT_TRUE(edge(dotstar, dotstar, g).second);
    ASSERT_TRUE(edge(dotstar, vc, g).second);

    // 'c' should have an edge to accept
    ASSERT_TRUE(edge(vc, graph->accept, g).second);
}

TEST(NFAGraph, RemoveRedundancy3) {
    CompileContext cc(false, false, get_current_target(), Grey());
    auto graph(constructGraphWithCC("foobar.*(a|b)?teakettle", cc, 0));
    ASSERT_TRUE(graph.get() != nullptr);

    unsigned countBefore = num_vertices(*graph);
    removeRedundancy(*graph, SOM_NONE);

    // The '(a|b)?' construction (two states) should have disappeared, leaving
    // this expr as 'foobar.*teakettle'
    ASSERT_EQ(countBefore - 2, num_vertices(*graph));
}

TEST(NFAGraph, RemoveRedundancy4) {
    CompileContext cc(false, false, get_current_target(), Grey());
    auto graph(constructGraphWithCC("foo([A-Z]|a|b|q)", cc, 0));
    ASSERT_TRUE(graph.get() != nullptr);

    unsigned countBefore = num_vertices(*graph);
    removeRedundancy(*graph, SOM_NONE);

    // We should end up with the alternation collapsing into one state
    ASSERT_EQ(countBefore - 3, num_vertices(*graph));
}

TEST(NFAGraph, RemoveRedundancy5) {
    CompileContext cc(false, false, get_current_target(), Grey());
    auto graph(constructGraphWithCC("[0-9]?badgerbrush", cc, 0));
    ASSERT_TRUE(graph.get() != nullptr);

    unsigned countBefore = num_vertices(*graph);
    removeRedundancy(*graph, SOM_NONE);

    // Since we don't return a start offset, the first state ('[0-9]?') is
    // redundant.
    ASSERT_EQ(countBefore - 1, num_vertices(*graph));
}

TEST(NFAGraph, RemoveEdgeRedundancy1) {
    CompileContext cc(false, false, get_current_target(), Grey());

    auto graph = constructGraphWithCC("A+hatstand", cc, HS_FLAG_DOTALL);
    ASSERT_TRUE(graph.get() != nullptr);

    unsigned countBefore = num_edges(*graph);

    removeEdgeRedundancy(*graph, SOM_NONE, cc);

    // One edge (the self-loop on the leading A+) should have been removed.
    ASSERT_EQ(countBefore - 1, num_edges(*graph));
}

TEST(NFAGraph, RemoveEdgeRedundancy2) {
    CompileContext cc(false, false, get_current_target(), Grey());

    auto graph = constructGraphWithCC("foo.*A*bar", cc, HS_FLAG_DOTALL);
    ASSERT_TRUE(graph.get() != nullptr);

    size_t numEdgesBefore = num_edges(*graph);
    size_t numVertsBefore = num_vertices(*graph);

    removeEdgeRedundancy(*graph, SOM_NONE, cc);

    // The .* should swallow up the A* and its self-loop.
    ASSERT_EQ(numEdgesBefore - 4, num_edges(*graph));
    ASSERT_EQ(numVertsBefore - 1, num_vertices(*graph));
}
