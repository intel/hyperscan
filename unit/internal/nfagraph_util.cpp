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
#include "nfagraph/ng.h"
#include "nfagraph/ng_split.h"
#include "nfagraph/ng_util.h"

using namespace std;
using namespace boost;
using namespace ue2;

TEST(NFAGraph, split1) {
    NGHolder src, lhs, rhs;

    NFAVertex a = add_vertex(src);
    NFAVertex b = add_vertex(src);
    NFAVertex c = add_vertex(src);
    NFAVertex d = add_vertex(src);
    NFAVertex e = add_vertex(src);
    NFAVertex f = add_vertex(src);
    NFAVertex g = add_vertex(src);
    NFAVertex h = add_vertex(src);
    NFAVertex i = add_vertex(src);

    src[a].char_reach = CharReach('a');
    src[b].char_reach = CharReach('b');
    src[c].char_reach = CharReach('c');
    src[d].char_reach = CharReach('d');
    src[e].char_reach = CharReach('e');
    src[f].char_reach = CharReach('f');
    src[g].char_reach = CharReach('g');
    src[h].char_reach = CharReach('h');
    src[i].char_reach = CharReach('i');

    add_edge(src.start, a, src);
    add_edge(f, src.acceptEod, src);

    add_edge(a, b, src);
    add_edge(b, c, src);
    add_edge(c, d, src);
    add_edge(d, e, src);
    add_edge(e, f, src);

    add_edge(c, g, src);
    add_edge(g, b, src);

    add_edge(d, h, src);
    add_edge(h, c, src);

    add_edge(c, i, src);
    add_edge(i, c, src);

    NFAVertex pivot = c;

    unordered_map<NFAVertex, NFAVertex> lhs_map;
    unordered_map<NFAVertex, NFAVertex> rhs_map;

    splitGraph(src, pivot, &lhs, &lhs_map, &rhs, &rhs_map);

    ASSERT_EQ(3U + N_SPECIALS, num_vertices(lhs));
    for (NFAVertex v : vertices_range(lhs)) {
        if (is_special(v, lhs)) {
            continue;
        }

        u32 cr = lhs[v].char_reach.find_first();
        SCOPED_TRACE(cr);
        ASSERT_TRUE((cr >= 'a' && cr <= 'c'));
    }

    ASSERT_EQ(8U + N_SPECIALS, num_vertices(rhs) );
    for (NFAVertex v : vertices_range(rhs)) {
        if (is_special(v, rhs)) {
            continue;
        }

        u32 cr = rhs[v].char_reach.find_first();
        SCOPED_TRACE(cr);
        ASSERT_TRUE(cr >= 'b' && cr <= 'i');
    }
}

TEST(NFAGraph, split2) {
    NGHolder src, lhs, rhs;

    NFAVertex a = add_vertex(src);
    NFAVertex b = add_vertex(src);
    NFAVertex c = add_vertex(src);
    NFAVertex d = add_vertex(src);

    src[a].char_reach = CharReach('a');
    src[b].char_reach = CharReach('b');
    src[c].char_reach = CharReach('c');
    src[d].char_reach = CharReach('d');

    add_edge(src.start, a, src);
    add_edge(d, src.acceptEod, src);

    add_edge(a, b, src);
    add_edge(b, c, src);
    add_edge(c, d, src);
    add_edge(d, b, src);

    NFAVertex pivot = c;

    unordered_map<NFAVertex, NFAVertex> lhs_map;
    unordered_map<NFAVertex, NFAVertex> rhs_map;

    splitGraph(src, pivot, &lhs, &lhs_map, &rhs, &rhs_map);

    ASSERT_EQ(3U + N_SPECIALS, num_vertices(lhs));
    for (NFAVertex v : vertices_range(lhs)) {
        if (is_special(v, lhs)) {
            continue;
        }

        u32 cr = lhs[v].char_reach.find_first();
        SCOPED_TRACE(cr);
        ASSERT_TRUE(cr >= 'a' && cr <= 'c');
    }

    ASSERT_EQ(3U + N_SPECIALS, num_vertices(rhs) );
    for (NFAVertex v : vertices_range(rhs)) {
        if (is_special(v, rhs)) {
            continue;
        }

        u32 cr = rhs[v].char_reach.find_first();
        SCOPED_TRACE(cr);
        ASSERT_TRUE(cr >= 'b' && cr <= 'd');
    }
}

TEST(NFAGraph, split3) {
    NGHolder src, lhs, rhs;

    NFAVertex a = add_vertex(src);
    NFAVertex b = add_vertex(src);
    NFAVertex c = add_vertex(src);
    NFAVertex d = add_vertex(src);
    NFAVertex e = add_vertex(src);
    NFAVertex f = add_vertex(src);
    NFAVertex g = add_vertex(src);
    NFAVertex h = add_vertex(src);
    NFAVertex i = add_vertex(src);

    src[a].char_reach = CharReach('a');
    src[b].char_reach = CharReach('b');
    src[c].char_reach = CharReach('c');
    src[d].char_reach = CharReach('d');
    src[e].char_reach = CharReach('e');
    src[f].char_reach = CharReach('f');
    src[g].char_reach = CharReach('g');
    src[h].char_reach = CharReach('h');
    src[i].char_reach = CharReach('i');

    add_edge(src.start, a, src);
    add_edge(src.start, e, src);
    add_edge(i, src.acceptEod, src);

    add_edge(a, b, src);
    add_edge(b, c, src);
    add_edge(c, d, src);

    add_edge(e, f, src);
    add_edge(f, g, src);

    add_edge(h, i, src);

    add_edge(c, h, src);
    add_edge(d, h, src);
    add_edge(g, h, src);

    vector<NFAVertex> pivots;
    pivots.push_back(c);
    pivots.push_back(d);
    pivots.push_back(g);

    unordered_map<NFAVertex, NFAVertex> lhs_map;
    unordered_map<NFAVertex, NFAVertex> rhs_map;

    splitGraph(src, pivots, &lhs, &lhs_map, &rhs, &rhs_map);

    ASSERT_EQ(7U + N_SPECIALS, num_vertices(lhs));
    for (NFAVertex v : vertices_range(lhs)) {
        if (is_special(v, lhs)) {
            continue;
        }

        u32 cr = lhs[v].char_reach.find_first();
        SCOPED_TRACE(cr);
        ASSERT_TRUE((cr >= 'a' && cr <= 'g'));
    }

    ASSERT_EQ(2U + N_SPECIALS, num_vertices(rhs) );
    for (NFAVertex v : vertices_range(rhs)) {
        if (is_special(v, rhs)) {
            continue;
        }

        u32 cr = rhs[v].char_reach.find_first();
        SCOPED_TRACE(cr);
        ASSERT_TRUE(cr >= 'h' && cr <= 'i');
    }
}

TEST(NFAGraph, split4) {
    NGHolder src, lhs, rhs;

    NFAVertex a = add_vertex(src);
    NFAVertex b = add_vertex(src);
    NFAVertex c = add_vertex(src);
    NFAVertex d = add_vertex(src);
    NFAVertex e = add_vertex(src);
    NFAVertex f = add_vertex(src);
    NFAVertex g = add_vertex(src);
    NFAVertex h = add_vertex(src);
    NFAVertex i = add_vertex(src);

    src[a].char_reach = CharReach('a');
    src[b].char_reach = CharReach('b');
    src[c].char_reach = CharReach('c');
    src[d].char_reach = CharReach('d');
    src[e].char_reach = CharReach('e');
    src[f].char_reach = CharReach('f');
    src[g].char_reach = CharReach('g');
    src[h].char_reach = CharReach('h');
    src[i].char_reach = CharReach('i');

    add_edge(src.start, a, src);
    add_edge(src.start, e, src);
    add_edge(i, src.acceptEod, src);

    add_edge(a, b, src);
    add_edge(b, c, src);
    add_edge(c, d, src);

    add_edge(e, f, src);
    add_edge(f, g, src);

    add_edge(h, i, src);

    add_edge(c, h, src);
    add_edge(d, h, src);
    add_edge(g, h, src);

    /* loops */
    add_edge(g, e, src);
    add_edge(d, d, src);

    vector<NFAVertex> pivots;
    pivots.push_back(c);
    pivots.push_back(d);
    pivots.push_back(g);

    unordered_map<NFAVertex, NFAVertex> lhs_map;
    unordered_map<NFAVertex, NFAVertex> rhs_map;

    splitGraph(src, pivots, &lhs, &lhs_map, &rhs, &rhs_map);

    ASSERT_EQ(7U + N_SPECIALS, num_vertices(lhs));
    for (NFAVertex v : vertices_range(lhs)) {
        if (is_special(v, lhs)) {
            continue;
        }

        u32 cr = lhs[v].char_reach.find_first();
        SCOPED_TRACE(cr);
        ASSERT_TRUE((cr >= 'a' && cr <= 'g'));
    }

    ASSERT_TRUE(edge(lhs_map[g], lhs_map[e], lhs).second);
    ASSERT_TRUE(edge(lhs_map[d], lhs_map[d], lhs).second);

    ASSERT_EQ(2U + N_SPECIALS, num_vertices(rhs) );
    for (NFAVertex v : vertices_range(rhs)) {
        if (is_special(v, rhs)) {
            continue;
        }

        u32 cr = rhs[v].char_reach.find_first();
        SCOPED_TRACE(cr);
        ASSERT_TRUE(cr >= 'h' && cr <= 'i');
    }
}

TEST(NFAGraph, cyclicVerts1) {
    NGHolder g;

    NFAVertex a = add_vertex(g);
    NFAVertex b = add_vertex(g);

    add_edge(a, b, g);
    add_edge(b, a, g);

    auto cyclics = find_vertices_in_cycles(g);

    ASSERT_EQ(flat_set<NFAVertex>({g.startDs, a, b}), cyclics);
}

TEST(NFAGraph, cyclicVerts2) {
    NGHolder g;

    NFAVertex a = add_vertex(g);
    NFAVertex b = add_vertex(g);
    NFAVertex c = add_vertex(g);
    NFAVertex d = add_vertex(g);
    NFAVertex e = add_vertex(g);

    add_edge(g.start, a, g);
    add_edge(a, b, g);
    add_edge(b, c, g);
    add_edge(c, a, g);
    add_edge(c, d, g);
    add_edge(a, e, g);

    auto cyclics = find_vertices_in_cycles(g);

    ASSERT_EQ(flat_set<NFAVertex>({g.startDs, a, b, c}), cyclics);
}

TEST(NFAGraph, cyclicVerts3) {
    NGHolder g;

    NFAVertex a = add_vertex(g);
    NFAVertex b = add_vertex(g);
    NFAVertex c = add_vertex(g);
    NFAVertex d = add_vertex(g);
    NFAVertex e = add_vertex(g);
    NFAVertex f = add_vertex(g);
    NFAVertex h = add_vertex(g);

    add_edge(g.start, a, g);
    add_edge(g.start, d, g);
    add_edge(a, b, g);
    add_edge(b, c, g);
    add_edge(b, f, g);
    add_edge(c, a, g);
    add_edge(b, d, g);
    add_edge(d, e, g);
    add_edge(e, b, g);
    add_edge(f, h, g);
    add_edge(h, h, g);

    auto cyclics = find_vertices_in_cycles(g);

    ASSERT_EQ(flat_set<NFAVertex>({g.startDs, a, b, c, d, e, h}), cyclics);
}

TEST(NFAGraph, cyclicVerts4) {
    NGHolder g;

    NFAVertex a = add_vertex(g);
    NFAVertex b = add_vertex(g);
    NFAVertex c = add_vertex(g);
    NFAVertex d = add_vertex(g);
    NFAVertex e = add_vertex(g);
    NFAVertex f = add_vertex(g);
    NFAVertex h = add_vertex(g);

    add_edge(g.start, a, g);
    add_edge(g.start, d, g);
    add_edge(a, b, g);
    add_edge(b, c, g);
    add_edge(b, d, g);
    add_edge(c, e, g);
    add_edge(d, e, g);
    add_edge(e, a, g);
    add_edge(e, f, g);
    add_edge(f, h, g);

    auto cyclics = find_vertices_in_cycles(g);

    ASSERT_EQ(flat_set<NFAVertex>({g.startDs, a, b, c, d, e}), cyclics);
}

TEST(NFAGraph, cyclicVerts5) {
    NGHolder g;

    NFAVertex a = add_vertex(g);
    NFAVertex b = add_vertex(g);
    NFAVertex c = add_vertex(g);
    NFAVertex d = add_vertex(g);
    NFAVertex e = add_vertex(g);

    add_edge(g.start, a, g);
    add_edge(g.start, e, g);
    add_edge(a, b, g);
    add_edge(b, c, g);
    add_edge(c, b, g);
    add_edge(c, d, g);
    add_edge(e, c, g);

    auto cyclics = find_vertices_in_cycles(g);

    ASSERT_EQ(flat_set<NFAVertex>({g.startDs, b, c}), cyclics);
}
