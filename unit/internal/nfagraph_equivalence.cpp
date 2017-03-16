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
 * Unit tests for checking the removeGraphEquivalences code in
 * nfagraph/ng_equivalence.cpp.
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
#include "nfagraph/ng_equivalence.h"
#include "nfagraph/ng_misc_opt.h"
#include "util/target_info.h"

using namespace std;
using namespace ue2;

// left equivalence
TEST(NFAGraph, RemoveEquivalence1) {
    // Build a small graph with a redundant vertex: (ab|ac)
    // The graph should be merged into: a(b|c)
    CompileContext cc(false, false, get_current_target(), Grey());

    auto graph(constructGraphWithCC("(ab|ac)", cc, 0));
    ASSERT_TRUE(graph != nullptr);
    NGHolder &g = *graph;
    g.kind = NFA_SUFFIX;

    // Run reduceGraphEquivalences
    ASSERT_TRUE(reduceGraphEquivalences(g, cc));

    // Our graph should only have three non-special nodes
    ASSERT_EQ((size_t)N_SPECIALS + 3, num_vertices(g));

    // Start dot start state should have edges to itself and one vertex
    ASSERT_EQ(2U, out_degree(g.startDs, g));

    // Accept state should have two incoming edges
    ASSERT_EQ(2U, in_degree(g.accept, g));

    // Find a vertex that goes right after startDs
    NFAVertex a = NGHolder::null_vertex();
    for (NFAVertex v : adjacent_vertices_range(g.startDs, g)) {
        a = v;
        if (a == g.startDs) {
            continue;
        }
        // check if it has the right char reach
        const CharReach &tmpcr = g[a].char_reach;
        ASSERT_EQ(1U, tmpcr.count());
        ASSERT_TRUE(tmpcr.test('a'));
    }
    // check if we found our vertex
    ASSERT_TRUE(a != NGHolder::null_vertex());

    // There should be two edges from v to nodes with reachability 'b' and 'c'
    NFAVertex b = NGHolder::null_vertex();
    NFAVertex c = NGHolder::null_vertex();
    for (NFAVertex tmp : adjacent_vertices_range(a, g)) {
        const CharReach &tmpcr = g[tmp].char_reach;
        ASSERT_EQ(1U, tmpcr.count());
        if (tmpcr.test('b')) {
            b = tmp;
        } else if (tmpcr.test('c')) {
            c = tmp;
        } else {
            FAIL();
        }
    }
    // check if we found our vertices
    ASSERT_TRUE(b != NGHolder::null_vertex());
    ASSERT_TRUE(c != NGHolder::null_vertex());

    // both vertices should have an edge to accept
    ASSERT_TRUE(edge(b, g.accept, g).second);
    ASSERT_TRUE(edge(c, g.accept, g).second);
}

// right equivalence
TEST(NFAGraph, RemoveEquivalence2) {
    // Build a small graph with a redundant vertex: (ba|ca)
    // The graph should be merged into: (b|c)a
    CompileContext cc(false, false, get_current_target(), Grey());

    auto graph(constructGraphWithCC("(ba|ca)", cc, 0));
    ASSERT_TRUE(graph != nullptr);
    NGHolder &g = *graph;
    g.kind = NFA_SUFFIX;

    // Run reduceGraphEquivalences
    ASSERT_TRUE(reduceGraphEquivalences(g, cc));

    // Our graph should only have two non-special nodes
    ASSERT_EQ((size_t)N_SPECIALS + 3, num_vertices(g));

    // Start dot start state should have edges to itself and two more vertices
    ASSERT_EQ(3U, out_degree(g.startDs, g));

    // Accept start state should have edges from one vertex only
    ASSERT_EQ(1U, in_degree(g.accept, g));

    // Find a vertex leading to accept
    NFAVertex a = NGHolder::null_vertex();
    for (NFAVertex v : inv_adjacent_vertices_range(g.accept, g)) {
        a = v;
        if (a == g.accept) {
            continue;
        }
        // check if it has the right char reach
        const CharReach &tmpcr = g[a].char_reach;
        ASSERT_EQ(1U, tmpcr.count());
        ASSERT_TRUE(tmpcr.test('a'));
    }
    // check if we found our vertex
    ASSERT_TRUE(a != NGHolder::null_vertex());

    // There should be two edges from v to nodes with reachability 'b' and 'c'
    NFAVertex b = NGHolder::null_vertex();
    NFAVertex c = NGHolder::null_vertex();
    for (NFAVertex tmp : inv_adjacent_vertices_range(a, g)) {
        const CharReach &tmpcr = g[tmp].char_reach;
        ASSERT_EQ(1U, tmpcr.count());
        if (tmpcr.test('b')) {
            b = tmp;
        } else if (tmpcr.test('c')) {
            c = tmp;
        } else {
            FAIL();
        }
    }
    // check if we found our vertices
    ASSERT_TRUE(b != NGHolder::null_vertex());
    ASSERT_TRUE(c != NGHolder::null_vertex());

    // both new vertices should have edges from startDs
    ASSERT_TRUE(edge(g.startDs, b, g).second);
    ASSERT_TRUE(edge(g.startDs, c, g).second);
}

// more complex left equivalence
TEST(NFAGraph, RemoveEquivalence3) {
    // Build a small graph with a redundant vertex: a(..)+X|a(..)+Y
    // The graph should be merged into: a(..)+(X|Y)
    CompileContext cc(false, false, get_current_target(), Grey());

    auto graph(constructGraphWithCC("a(..)+X|a(..)+Y", cc, HS_FLAG_DOTALL));
    ASSERT_TRUE(graph != nullptr);
    NGHolder &g = *graph;
    g.kind = NFA_SUFFIX;

    // Run reduceGraphEquivalences
    ASSERT_TRUE(reduceGraphEquivalences(g, cc));

    // Our graph should only have five non-special nodes
    ASSERT_EQ((size_t)N_SPECIALS + 5, num_vertices(g));

    // Start dot start state should have edges to itself and one vertex
    ASSERT_EQ(2U, out_degree(g.startDs, g));

    // Accept state should have two incoming edges
    ASSERT_EQ(2U, in_degree(g.accept, g));

    // Find a vertex 'a' that goes right after startDs
    NFAVertex a = NGHolder::null_vertex();
    for (NFAVertex v : adjacent_vertices_range(g.startDs, g)) {
        a = v;
        if (a == g.startDs) {
            continue;
        }
        // check if it has the right char reach
        const CharReach &tmpcr = g[a].char_reach;
        ASSERT_EQ(1U, tmpcr.count());
        ASSERT_TRUE(tmpcr.test('a'));
    }
    // check if we found our 'a'
    ASSERT_TRUE(a != NGHolder::null_vertex());

    // There should be an edge from 'a' to '.'
    ASSERT_EQ(1U, out_degree(a, g));
    NFAVertex dot1 = *(adjacent_vertices(a, g).first);

    // check if this is a dot
    const CharReach dot1cr = g[dot1].char_reach;
    ASSERT_TRUE(dot1cr.all());

    // After dot1, there should be another '.'
    ASSERT_EQ(1U, out_degree(dot1, g));
    NFAVertex dot2 = *(adjacent_vertices(dot1, g).first);

    // check its char reach as well
    const CharReach dot2cr = g[dot2].char_reach;
    ASSERT_TRUE(dot2cr.all());

    // the second dot should have three edges - to dot1, and to X and Y
    // first, check an edge to dot1
    ASSERT_EQ(3U, out_degree(dot2, g));
    ASSERT_TRUE(edge(dot2, dot1, g).second);

    // now, let's find X and Y nodes
    NFAVertex X = NGHolder::null_vertex();
    NFAVertex Y = NGHolder::null_vertex();
    for (NFAVertex tmp : adjacent_vertices_range(dot2, g)) {
        // we already know about dot1, so skip it
        if (tmp == dot1) {
            continue;
        }
        const CharReach tmpcr = g[tmp].char_reach;

        ASSERT_EQ(1U, tmpcr.count());
        if (tmpcr.test('X')) {
            X = tmp;
        } else if (tmpcr.test('Y')) {
            Y = tmp;
        } else {
            FAIL();
        }
    }
    // check if we found both vertices
    ASSERT_TRUE(X != NGHolder::null_vertex());
    ASSERT_TRUE(Y != NGHolder::null_vertex());

    // finally, check if these two vertices only have edges to accept
    ASSERT_EQ(1U, out_degree(X, g));
    ASSERT_EQ(1U, out_degree(Y, g));
    ASSERT_TRUE(edge(X, g.accept, g).second);
    ASSERT_TRUE(edge(Y, g.accept, g).second);
}

// more complex right equivalence
TEST(NFAGraph, RemoveEquivalence4) {
    // Build a small graph with a redundant vertex: X(..)+a|Y(..)+a
    // The graph should be merged into: (X|Y)(..)+a
    CompileContext cc(false, false, get_current_target(), Grey());

    auto graph(constructGraphWithCC("X(..)+a|Y(..)+a", cc, HS_FLAG_DOTALL));
    ASSERT_TRUE(graph != nullptr);
    NGHolder &g = *graph;
    g.kind = NFA_SUFFIX;

    // Run reduceGraphEquivalences
    ASSERT_TRUE(reduceGraphEquivalences(g, cc));

    // Our graph should only have five non-special nodes
    ASSERT_EQ((size_t)N_SPECIALS + 5, num_vertices(g));

    // Start dot start state should have edges to itself and two vertices
    ASSERT_EQ(3U, out_degree(g.startDs, g));

    // Accept state should have one incoming edge
    ASSERT_EQ(1U, in_degree(g.accept, g));

    // Find X and Y nodes that are connected to startDs
    NFAVertex X = NGHolder::null_vertex();
    NFAVertex Y = NGHolder::null_vertex();
    for (NFAVertex tmp : adjacent_vertices_range(g.startDs, g)) {
        // skip startDs
        if (tmp == g.startDs) {
            continue;
        }
        // get char reach
        const CharReach tmpcr = g[tmp].char_reach;

        ASSERT_EQ(1U, tmpcr.count());
        if (tmpcr.test('X')) {
            X = tmp;
        } else if (tmpcr.test('Y')) {
            Y = tmp;
        } else {
            FAIL();
        }
    }
    // check if we found both vertices
    ASSERT_TRUE(X != NGHolder::null_vertex());
    ASSERT_TRUE(Y != NGHolder::null_vertex());

    // now, find first dot from X
    ASSERT_EQ(1U, out_degree(X, g));
    NFAVertex dot1 = *(adjacent_vertices(X, g).first);

    // make sure Y also has an edge to that dot
    ASSERT_EQ(1U, out_degree(Y, g));
    ASSERT_TRUE(edge(Y, dot1, g).second);

    // now, verify that it's actually a dot
    const CharReach dot1cr = g[dot1].char_reach;
    ASSERT_TRUE(dot1cr.all());

    // the first dot has one edge to another dot
    ASSERT_EQ(1U, out_degree(dot1, g));
    NFAVertex dot2 = *(adjacent_vertices(dot1, g).first);

    // verify it's a dot
    const CharReach dot2cr = g[dot2].char_reach;
    ASSERT_TRUE(dot2cr.all());

    // second dot should have two edges - to dot1 and to 'a'
    ASSERT_EQ(2U, out_degree(dot2, g));
    ASSERT_TRUE(edge(dot2, dot1, g).second);

    // now find 'a'
    NFAVertex a = NGHolder::null_vertex();
    for (NFAVertex tmp : adjacent_vertices_range(dot2, g)) {
        // skip dot1
        if (tmp == dot1) {
            continue;
        }
        // get char reach
        const CharReach tmpcr = g[tmp].char_reach;

        ASSERT_EQ(1U, tmpcr.count());
        if (tmpcr.test('a')) {
            a = tmp;
        } else {
            FAIL();
        }
    }
    // make sure we found our 'a'
    ASSERT_TRUE(a != NGHolder::null_vertex());

    // now, check if 'a' has an edge to accept
    ASSERT_EQ(1U, out_degree(a, g));
    ASSERT_TRUE(edge(a, g.accept, g).second);
}

// catching UE-2693
TEST(NFAGraph, RemoveEquivalence5) {
    // Build a small graph with a redundant vertex: [^\x00][^x00]*[\\x00]
    // The graph should be merged into: [^\x00]*[\x00]
    CompileContext cc(false, false, get_current_target(), Grey());

    auto graph(constructGraphWithCC("[^\\x00][^\\x00]*[\\x00]", cc, 0));
    ASSERT_TRUE(graph != nullptr);
    NGHolder &g = *graph;
    g.kind = NFA_PREFIX;

    // Run reduceGraphEquivalences
    ASSERT_TRUE(reduceGraphEquivalences(g, cc));

    // Our graph should only have two non-special nodes
    ASSERT_EQ((size_t)N_SPECIALS + 2, num_vertices(g));

    // Start dot start state should have edges to itself and one vertex
    ASSERT_EQ(2U, out_degree(g.startDs, g));

    // Accept state should have one incoming edge
    ASSERT_EQ(1U, in_degree(g.accept, g));

    // find first vertex and ensure it has a self loop
    NFAVertex v = NGHolder::null_vertex();
    for (NFAVertex t : adjacent_vertices_range(g.startDs, g)) {
        v = t;
        if (v == g.startDs) {
            continue;
        }
        // check if it has the right char reach
        const CharReach &tmpcr = g[v].char_reach;
        ASSERT_EQ(255U, tmpcr.count());
        ASSERT_TRUE(!tmpcr.test(0));
        ASSERT_TRUE(edge(v, v, g).second);
    }
    // check if we found our vertex
    ASSERT_TRUE(v != NGHolder::null_vertex());

    // now, find the vertex leading to accept
    NFAVertex v2 = NGHolder::null_vertex();
    for (NFAVertex tmp : adjacent_vertices_range(v, g)) {
        // skip self-loop
        if (tmp == v) {
            continue;
        }
        v2 = tmp;
        // get char reach
        const CharReach tmpcr = g[tmp].char_reach;

        ASSERT_EQ(1U, tmpcr.count());
        ASSERT_TRUE(tmpcr.test(0));
        ASSERT_TRUE(edge(tmp, g.accept, g).second);
    }
    // check if we found our vertex
    ASSERT_TRUE(v2 != NGHolder::null_vertex());
}

// catching UE-2692
TEST(NFAGraph, RemoveEquivalence6) {
    // Build a small graph with two redundant vertices: ^(.*|.*)a
    // The graph should be merged into: a
    auto graph(constructGraph("^(.*|.*)a", HS_FLAG_DOTALL));
    ASSERT_TRUE(graph != nullptr);
    NGHolder &g = *graph;

    // Run mergeCyclicDotStars
    ASSERT_TRUE(mergeCyclicDotStars(g));

    // Our graph should only have one non-special node
    ASSERT_EQ((size_t)N_SPECIALS + 1, num_vertices(g));

    // Start dot start state should have edges to itself and one vertex
    ASSERT_EQ(2U, out_degree(g.startDs, g));

    // Accept state should have one incoming edge
    ASSERT_EQ(1U, in_degree(g.accept, g));

    // find that vertex and ensure it has no self loops and an edge to accept
    NFAVertex v = NGHolder::null_vertex();
    for (NFAVertex t : adjacent_vertices_range(g.startDs, g)) {
        v = t;
        if (v == g.startDs) {
            continue;
        }
        // check if it has the right char reach
        const CharReach &tmpcr = g[v].char_reach;
        ASSERT_EQ(1U, tmpcr.count());
        ASSERT_TRUE(tmpcr.test('a'));
        ASSERT_TRUE(!edge(v, v, g).second);
        ASSERT_TRUE(edge(v, g.accept, g).second);
    }
    // check if we found our vertex
    ASSERT_TRUE(v != NGHolder::null_vertex());
}

// catching UE-2692
TEST(NFAGraph, RemoveEquivalence7) {
    // Build a small graph with no redundant vertices: ^.+a
    // Make sure we don't merge anything
    auto graph(constructGraph("^.+a", HS_FLAG_DOTALL));
    ASSERT_TRUE(graph != nullptr);
    NGHolder &g = *graph;

    // Run mergeCyclicDotStars
    ASSERT_FALSE(mergeCyclicDotStars(g));

    // Our graph should have two non-special nodes
    ASSERT_EQ((size_t)N_SPECIALS + 2, num_vertices(g));

    // Start dot start state should have an edge to itself only
    ASSERT_EQ(1U, out_degree(g.startDs, g));

    // Start should have edges to startDs and one other vertex
    ASSERT_EQ(2U, out_degree(g.start, g));

    // Accept state should have one incoming edge
    ASSERT_EQ(1U, in_degree(g.accept, g));

    // find that vertex and ensure it's a dot self loop and has one outgoing edge
    NFAVertex v = NGHolder::null_vertex();
    for (NFAVertex t : adjacent_vertices_range(g.start, g)) {
        if (t == g.startDs) {
            continue;
        }
        v = t;
        // check if it has the right char reach
        const CharReach &tmpcr = g[v].char_reach;
        ASSERT_TRUE(tmpcr.all());
        ASSERT_TRUE(edge(v, v, g).second);
        ASSERT_EQ(1U, proper_out_degree(v, g));
    }
    // check if we found our vertex
    ASSERT_TRUE(v != NGHolder::null_vertex());

    // find the next vertex and ensure it has an edge to accept
    NFAVertex v2 = NGHolder::null_vertex();
    for (NFAVertex t : adjacent_vertices_range(v, g)) {
        // skip self loop
        if (t == v) {
            continue;
        }
        v2 = t;
        // check if it has the right char reach
        const CharReach &tmpcr = g[v2].char_reach;
        ASSERT_EQ(1U, tmpcr.count());
        ASSERT_TRUE(tmpcr.test('a'));
        ASSERT_TRUE(!edge(v2, v2, g).second);
        ASSERT_EQ(1U, proper_out_degree(v2, g));
        ASSERT_TRUE(edge(v2, g.accept, g).second);
    }
    // check if we found our vertex
    ASSERT_TRUE(v2 != NGHolder::null_vertex());
}

TEST(NFAGraph, RemoveEquivalence_Reports1) {
    CompileContext cc(false, false, get_current_target(), Grey());

    NGHolder g(NFA_SUFFIX);
    NFAVertex a = add_vertex(g);
    NFAVertex b = add_vertex(g);
    NFAVertex c = add_vertex(g);

    add_edge(g.start, a, g);
    add_edge(a, b, g);
    add_edge(a, c, g);

    add_edge(b, g.accept, g);
    add_edge(c, g.accept, g);

    g[b].reports.insert(0);
    g[c].reports.insert(1);

    ASSERT_TRUE(reduceGraphEquivalences(g, cc));

    ASSERT_EQ(num_vertices(g), N_SPECIALS + 2); /* b, c should be merged */
    ASSERT_EQ(in_degree(g.accept, g), 1);
}

TEST(NFAGraph, RemoveEquivalence_Reports2) {
    CompileContext cc(false, false, get_current_target(), Grey());

    NGHolder g(NFA_SUFFIX);
    NFAVertex a = add_vertex(g);
    NFAVertex b = add_vertex(g);
    NFAVertex c = add_vertex(g);

    add_edge(g.start, a, g);
    add_edge(a, b, g);
    add_edge(a, c, g);

    add_edge(b, g.accept, g);
    add_edge(c, g.acceptEod, g);

    g[b].reports.insert(0);
    g[c].reports.insert(1);

    ASSERT_TRUE(reduceGraphEquivalences(g, cc));

    ASSERT_EQ(num_vertices(g), N_SPECIALS + 3); /* b, c should not be merged */
    ASSERT_EQ(in_degree(g.accept, g), 1);
    ASSERT_EQ(in_degree(g.acceptEod, g), 2);
}

TEST(NFAGraph, RemoveEquivalence_Reports3) {
    CompileContext cc(false, false, get_current_target(), Grey());

    NGHolder g(NFA_SUFFIX);
    NFAVertex a = add_vertex(g);
    NFAVertex b = add_vertex(g);
    NFAVertex c = add_vertex(g);

    add_edge(g.start, a, g);
    add_edge(a, b, g);
    add_edge(a, c, g);

    add_edge(b, g.accept, g);
    add_edge(c, g.acceptEod, g);

    g[b].reports.insert(0);
    g[c].reports.insert(0);

    ASSERT_TRUE(reduceGraphEquivalences(g, cc));

    ASSERT_EQ(num_vertices(g), N_SPECIALS + 2); /* b, c should be merged */
    ASSERT_EQ(in_degree(g.accept, g), 1);
    ASSERT_EQ(in_degree(g.acceptEod, g), 2);
}

TEST(NFAGraph, RemoveEquivalence_Reports4) {
    CompileContext cc(false, false, get_current_target(), Grey());

    NGHolder g(NFA_SUFFIX);
    NFAVertex a = add_vertex(g);
    NFAVertex b = add_vertex(g);
    NFAVertex c = add_vertex(g);
    NFAVertex d = add_vertex(g);

    add_edge(g.start, a, g);
    add_edge(a, b, g);
    add_edge(a, c, g);
    add_edge(c, d, g);
    add_edge(d, d, g);

    add_edge(b, g.accept, g);

    g[b].reports.insert(0);

    ASSERT_TRUE(reduceGraphEquivalences(g, cc));

    ASSERT_EQ(num_vertices(g), N_SPECIALS + 3); /* b, c should be merged */
    ASSERT_EQ(in_degree(g.accept, g), 1);
    ASSERT_EQ(in_degree(g.acceptEod, g), 1);
}

TEST(NFAGraph, RemoveEquivalence_Reports5) {
    CompileContext cc(false, false, get_current_target(), Grey());

    NGHolder g(NFA_SUFFIX);
    NFAVertex a = add_vertex(g);
    NFAVertex b = add_vertex(g);
    NFAVertex c = add_vertex(g);
    NFAVertex d = add_vertex(g);

    add_edge(g.start, a, g);
    add_edge(a, b, g);
    add_edge(a, c, g);
    add_edge(c, d, g);
    add_edge(d, d, g);

    add_edge(b, g.acceptEod, g);

    g[b].reports.insert(0);

    ASSERT_TRUE(reduceGraphEquivalences(g, cc));

    ASSERT_EQ(num_vertices(g), N_SPECIALS + 3); /* b, c should be merged */
    ASSERT_EQ(in_degree(g.accept, g), 0);
    ASSERT_EQ(in_degree(g.acceptEod, g), 2);
}
