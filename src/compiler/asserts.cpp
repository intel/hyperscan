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

/** \file
 * \brief Convert temporary assert vertices (from construction method) to
 * edge-based flags.
 *
 * This pass converts the temporary assert vertices created by the Glushkov
 * construction process above (vertices with special assertions flags) into
 * edges between those vertices' neighbours in the graph.
 *
 * These edges have the appropriate flags applied to them -- a path (u,t,v)
 * through an assert vertex t will be replaced with the edge (u,v) with the
 * assertion flags from t.
 *
 * Edges with mutually incompatible flags (such as the conjunction of
 * word-to-word and word-to-nonword) are dropped.
 */
#include "asserts.h"

#include "compiler/compiler.h"
#include "nfagraph/ng.h"
#include "nfagraph/ng_prune.h"
#include "nfagraph/ng_redundancy.h"
#include "nfagraph/ng_util.h"
#include "parser/position.h" // for POS flags
#include "util/compile_error.h"
#include "util/graph_range.h"

#include <queue>
#include <set>

using namespace std;

namespace ue2 {

/** Hard limit on the maximum number of edges we'll clone before we throw up
 * our hands and report 'Pattern too large.' */
static const size_t MAX_ASSERT_EDGES = 300000;

/** Flags representing the word-boundary assertions, \\b or \\B. */
static const int WORDBOUNDARY_FLAGS = POS_FLAG_ASSERT_WORD_TO_WORD
                                    | POS_FLAG_ASSERT_WORD_TO_NONWORD
                                    | POS_FLAG_ASSERT_NONWORD_TO_WORD
                                    | POS_FLAG_ASSERT_NONWORD_TO_NONWORD
                                    | POS_FLAG_ASSERT_WORD_TO_WORD_UCP
                                    | POS_FLAG_ASSERT_WORD_TO_NONWORD_UCP
                                    | POS_FLAG_ASSERT_NONWORD_TO_WORD_UCP
                                    | POS_FLAG_ASSERT_NONWORD_TO_NONWORD_UCP;

#define OPEN_EDGE 0U
#define DEAD_EDGE (~0U)

static
u32 disjunct(u32 flags1, u32 flags2) {
    /* from two asserts in parallel */
    DEBUG_PRINTF("disjunct %x %x\n", flags1, flags2);
    u32 rv;
    if (flags1 == DEAD_EDGE) {
        rv = flags2;
    } else if (flags2 == DEAD_EDGE) {
        rv = flags1;
    } else if (flags1 == OPEN_EDGE || flags2 == OPEN_EDGE) {
        rv = OPEN_EDGE;
    } else {
        rv = flags1 | flags2;
    }
    DEBUG_PRINTF("--> %x\n", rv);
    return rv;
}

static
u32 conjunct(u32 flags1, u32 flags2) {
    /* from two asserts in series */
    DEBUG_PRINTF("conjunct %x %x\n", flags1, flags2);
    u32 rv;
    if (flags1 == OPEN_EDGE) {
        rv = flags2;
    } else if (flags2 == OPEN_EDGE) {
        rv = flags1;
    } else if (flags1 & flags2) {
        rv = flags1 & flags2;
    } else {
        rv = DEAD_EDGE; /* the conjunction of two different word boundary
                         * assertion is impassable */
    }

    DEBUG_PRINTF("--> %x\n", rv);
    return rv;
}

typedef map<pair<NFAVertex, NFAVertex>, NFAEdge> edge_cache_t;

static
void replaceAssertVertex(NGHolder &g, NFAVertex t, const ExpressionInfo &expr,
                         edge_cache_t &edge_cache, u32 &assert_edge_count) {
    DEBUG_PRINTF("replacing assert vertex %zu\n", g[t].index);

    const u32 flags = g[t].assert_flags;
    DEBUG_PRINTF("consider assert vertex %zu with flags %u\n", g[t].index,
                 flags);

    // Wire up all the predecessors to all the successors.

    for (const auto &inEdge : in_edges_range(t, g)) {
        NFAVertex u = source(inEdge, g);
        if (u == t) {
            continue; // ignore self-loops
        }

        const u32 flags_inc_in = conjunct(g[inEdge].assert_flags,
                                          flags);
        if (flags_inc_in == DEAD_EDGE) {
            DEBUG_PRINTF("fail, in-edge has bad flags %d\n",
                         g[inEdge].assert_flags);
            continue;
        }

        for (const auto &outEdge : out_edges_range(t, g)) {
            NFAVertex v = target(outEdge, g);

            DEBUG_PRINTF("consider path [%zu,%zu,%zu]\n", g[u].index,
                         g[t].index, g[v].index);

            if (v == t) {
                continue; // ignore self-loops
            }

            const u32 flags_final = conjunct(g[outEdge].assert_flags,
                                             flags_inc_in);

            if (flags_final == DEAD_EDGE) {
                DEBUG_PRINTF("fail, out-edge has bad flags %d\n",
                             g[outEdge].assert_flags);
                continue;
            }

            if ((g[u].assert_flags & POS_FLAG_MULTILINE_START)
                && v == g.acceptEod) {
                DEBUG_PRINTF("fail, (?m)^ does not match \\n at eod\n");
                continue;
            }

            /* Replace path (u,t,v) with direct edge (u,v), unless the edge
             * already exists, in which case we just need to edit its
             * properties.
             *
             * Use edge_cache to prevent us going O(N).
             */
            auto cache_key = make_pair(u, v);
            auto ecit = edge_cache.find(cache_key);
            if (ecit == edge_cache.end()) {
                DEBUG_PRINTF("adding edge %zu %zu\n", g[u].index, g[v].index);
                NFAEdge e = add_edge(u, v, g);
                edge_cache.emplace(cache_key, e);
                g[e].assert_flags = flags;
                if (++assert_edge_count > MAX_ASSERT_EDGES) {
                    throw CompileError(expr.index, "Pattern is too large.");
                }
            } else {
                NFAEdge e = ecit->second;
                DEBUG_PRINTF("updating edge %zu %zu [a %zu]\n", g[u].index,
                             g[v].index, g[t].index);
                // Edge already exists.
                u32 &e_flags = g[e].assert_flags;
                e_flags = disjunct(e_flags, flags_final);
                assert(e_flags != DEAD_EDGE);
            }
        }
    }

    // Clear vertex t to remove all the old edges.
    /* no need to clear the cache, as we will never look up its edge as it is
     * unreachable */
    clear_vertex(t, g);
}

static
void setReportId(ReportManager &rm, NGHolder &g, const ExpressionInfo &expr,
                 NFAVertex v, s32 adj) {
    // Don't try and set the report ID of a special vertex.
    assert(!is_special(v, g));

    // There should be no reports set already.
    assert(g[v].reports.empty());

    Report r = rm.getBasicInternalReport(expr, adj);

    g[v].reports.insert(rm.getInternalId(r));
    DEBUG_PRINTF("set report id for vertex %zu, adj %d\n", g[v].index, adj);
}

static
void checkForMultilineStart(ReportManager &rm, NGHolder &g,
                            const ExpressionInfo &expr) {
    vector<NFAEdge> dead;
    for (auto v : adjacent_vertices_range(g.start, g)) {
        if (!(g[v].assert_flags & POS_FLAG_MULTILINE_START)) {
            continue;
        }
        DEBUG_PRINTF("mls %zu %08x\n", g[v].index, g[v].assert_flags);

        /* we have found a multi-line start (maybe more than one) */

        /* we need to interpose a dummy dot vertex between v and accept if
         * required so that ^ doesn't match trailing \n */
         for (const auto &e : out_edges_range(v, g)) {
            if (target(e, g) == g.accept) {
                dead.push_back(e);
            }
        }
        /* assert has been resolved; clear flag */
        g[v].assert_flags &= ~POS_FLAG_MULTILINE_START;
    }

    for (const auto &e : dead) {
        NFAVertex dummy = add_vertex(g);
        g[dummy].char_reach.setall();
        setReportId(rm, g, expr, dummy, -1);
        add_edge(source(e, g), dummy, g[e], g);
        add_edge(dummy, g.accept, g);
    }

    remove_edges(dead, g);
}

static
bool hasAssertVertices(const NGHolder &g) {
    for (auto v : vertices_range(g)) {
        int flags = g[v].assert_flags;
        if (flags & WORDBOUNDARY_FLAGS) {
            return true;
        }
    }
    return false;
}

/** \brief Convert temporary assert vertices (from construction method) to
 * edge-based flags.
 *
 * Remove the horrors that are the temporary assert vertices which arise from
 * our construction method. Allows the rest of our code base to live in
 * blissful ignorance of their existence. */
void removeAssertVertices(ReportManager &rm, NGHolder &g,
                          const ExpressionInfo &expr) {
    size_t num = 0;

    DEBUG_PRINTF("before: graph has %zu vertices\n", num_vertices(g));

    // Sweep over the graph and ascertain that we do actually have vertices
    // with assertion flags set. Otherwise, we're done.
    if (!hasAssertVertices(g)) {
        DEBUG_PRINTF("no assert vertices, done\n");
        return;
    }

    u32 assert_edge_count = 0;

    // Build a cache of (u, v) vertex pairs to edge descriptors.
    edge_cache_t edge_cache;
    for (const auto &e : edges_range(g)) {
        edge_cache[make_pair(source(e, g), target(e, g))] = e;
    }

    for (auto v : vertices_range(g)) {
        if (g[v].assert_flags & WORDBOUNDARY_FLAGS) {
            replaceAssertVertex(g, v, expr, edge_cache, assert_edge_count);
            num++;
        }
    }

    checkForMultilineStart(rm, g, expr);

    if (num) {
        DEBUG_PRINTF("resolved %zu assert vertices\n", num);
        pruneUseless(g);
        pruneEmptyVertices(g);
        renumber_vertices(g);
        renumber_edges(g);
    }

    DEBUG_PRINTF("after: graph has %zu vertices\n", num_vertices(g));
    assert(!hasAssertVertices(g));
}

} // namespace ue2
