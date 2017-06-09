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
 * \brief Resolve special assert vertices.
 *
 * The assert resolution algorithm proceeds by iterating over those edges with
 * assertion flags, considering source and target vertices of each edge. If a
 * vertex has a superset of the reachability demanded by the assertion on the
 * edge, it is split into alternatives providing the word and non-word paths
 * through that vertex.
 *
 * A great deal of the complexity in the resolveAsserts pass is devoted to
 * handling these assertions when the UCP flag is specified (meaning \\w and \\W
 * are implemented with Unicode properties, rather than their ASCII
 * interpretation) and the prefiltering flag is also used. Complete,
 * non-prefiltering UCP support is not available yet.
 */
#include "ng_asserts.h"

#include "ng.h"
#include "ng_prune.h"
#include "ng_redundancy.h"
#include "ng_util.h"
#include "compiler/compiler.h"
#include "parser/position.h" // for POS flags
#include "util/bitutils.h" // for findAndClearLSB_32
#include "util/boundary_reports.h"
#include "util/container.h"
#include "util/compile_context.h"
#include "util/compile_error.h"
#include "util/graph_range.h"
#include "util/report_manager.h"
#include "util/unicode_def.h"

#include <queue>

using namespace std;

namespace ue2 {

/** \brief Hard limit on the maximum number of vertices we'll clone before we
 * throw up our hands and report 'Pattern too large.' */
static const size_t MAX_CLONED_VERTICES = 2048;

/** \brief The definition of \\w, since we use it everywhere in here. */
static const CharReach CHARREACH_WORD(CharReach('a', 'z') |
        CharReach('A', 'Z') | CharReach('0', '9') | CharReach('_'));

/** \brief \\W is the inverse of \\w */
static const CharReach CHARREACH_NONWORD(~CHARREACH_WORD);

/** \brief Prefiltering definition of \\w for UCP mode.
 *
 * Includes all high bytes as to capture all non-ASCII, however depending on
 * direction only continuers or starters are strictly required - as the input
 * is well-formed, this laxness will not cost us. */
static const CharReach CHARREACH_WORD_UCP_PRE(CHARREACH_WORD
                                              | CharReach(128, 255));

/** \brief Prefiltering definition of \\W for UCP Mode.
 *
 * (non-word already includes high bytes) */
static const CharReach CHARREACH_NONWORD_UCP_PRE(CHARREACH_NONWORD);

/** \brief Find all the edges with assertion flags. */
static
vector<NFAEdge> getAsserts(const NGHolder &g) {
    vector<NFAEdge> out;
    for (const auto &e : edges_range(g)) {
        if (g[e].assert_flags) {
            out.push_back(e);
        }
    }
    return out;
}

static
void addToSplit(const NGHolder &g, NFAVertex v, map<u32, NFAVertex> *to_split) {
    DEBUG_PRINTF("%zu needs splitting\n", g[v].index);
    to_split->emplace(g[v].index, v);
}

/** \brief Find vertices that need to be split due to an assertion edge.
 *
 * A vertex needs to be split if has an edge to/from it with an assert with a
 * restriction on the relevant end. */
static
void findSplitters(const NGHolder &g, const vector<NFAEdge> &asserts,
                   map<u32, NFAVertex> *to_split,
                   map<u32, NFAVertex> *to_split_ucp) {
    for (const auto &e : asserts) {
        NFAVertex u = source(e, g);
        NFAVertex v = target(e, g);
        u32 flags = g[e].assert_flags;
        assert(flags);

        const CharReach &u_cr = g[u].char_reach;
        const CharReach &v_cr = g[v].char_reach;

        bool ucp_assert = flags & UCP_ASSERT_FLAGS;
        bool normal_assert = flags & NON_UCP_ASSERT_FLAGS;
        /* In reality, an expression can only be entirely ucp or not ucp */
        assert(ucp_assert != normal_assert);

        if (normal_assert) {
            /* assume any flag results in us have to split if the vertex is not
             * a subset of word or completely disjoint from it. We could be more
             * nuanced if flags is a disjunction of multiple assertions. */
            if (!u_cr.isSubsetOf(CHARREACH_WORD)
                && !u_cr.isSubsetOf(CHARREACH_NONWORD)
                && u != g.start) { /* start is always considered a nonword */
                addToSplit(g, u, to_split);
            }

            if (!v_cr.isSubsetOf(CHARREACH_WORD)
                && !v_cr.isSubsetOf(CHARREACH_NONWORD)
                && v != g.accept /* accept require special handling, done on a
                                  * per edge basis in resolve asserts
                                  */
                && v != g.acceptEod) { /* eod is always considered a nonword */
                addToSplit(g, v, to_split);
            }
        }

        if (ucp_assert) {
            /* note: the ucp prefilter crs overlap - requires a bit more care */
            if (u == g.start) { /* start never needs to be split,
                                 * treat nonword */
            } else if (flags & POS_FLAG_ASSERT_WORD_TO_ANY_UCP) {
                if (!u_cr.isSubsetOf(CHARREACH_WORD_UCP_PRE)
                    && !u_cr.isSubsetOf(~CHARREACH_WORD_UCP_PRE)) {
                    addToSplit(g, u, to_split_ucp);
                }
            } else {
                assert(flags & POS_FLAG_ASSERT_NONWORD_TO_ANY_UCP);
                if (!u_cr.isSubsetOf(CHARREACH_NONWORD_UCP_PRE)
                    && !u_cr.isSubsetOf(~CHARREACH_NONWORD_UCP_PRE)) {
                    addToSplit(g, u, to_split_ucp);
                }
            }

            if (v == g.acceptEod /* eod is always considered a nonword */
                || v == g.accept) {  /* accept require special handling, done on
                                      * a per edge basis in resolve asserts */
            } else if (flags & POS_FLAG_ASSERT_ANY_TO_WORD_UCP) {
                if (!v_cr.isSubsetOf(CHARREACH_WORD_UCP_PRE)
                    && !v_cr.isSubsetOf(~CHARREACH_WORD_UCP_PRE)) {
                    addToSplit(g, v, to_split_ucp);
                }
            } else {
                assert(flags & POS_FLAG_ASSERT_ANY_TO_NONWORD_UCP);
                if (!v_cr.isSubsetOf(CHARREACH_NONWORD_UCP_PRE)
                    && !v_cr.isSubsetOf(~CHARREACH_NONWORD_UCP_PRE)) {
                    addToSplit(g, v, to_split_ucp);
                }
            }
        }
    }
}

static
void setReportId(ReportManager &rm, NGHolder &g, const ExpressionInfo &expr,
                 NFAVertex v, s32 adj) {
    // Don't try and set the report ID of a special vertex.
    assert(!is_special(v, g));

    // If there's a report set already, we're replacing it.
    g[v].reports.clear();

    Report ir = rm.getBasicInternalReport(expr, adj);

    g[v].reports.insert(rm.getInternalId(ir));
    DEBUG_PRINTF("set report id for vertex %zu, adj %d\n", g[v].index, adj);
}

static
NFAVertex makeClone(ReportManager &rm, NGHolder &g, const ExpressionInfo &expr,
                    NFAVertex v, const CharReach &cr_mask) {
    NFAVertex clone = clone_vertex(g, v);
    g[clone].char_reach &= cr_mask;
    clone_out_edges(g, v, clone);
    clone_in_edges(g, v, clone);

    if (v == g.startDs) {
        if (expr.utf8) {
            g[clone].char_reach &= ~UTF_START_CR;
        }

        DEBUG_PRINTF("marked as virt\n");
        g[clone].assert_flags = POS_FLAG_VIRTUAL_START;

        setReportId(rm, g, expr, clone, 0);
    }

    return clone;
}

static
void splitVertex(ReportManager &rm, NGHolder &g, const ExpressionInfo &expr,
                 NFAVertex v, bool ucp) {
    assert(v != g.start);
    assert(v != g.accept);
    assert(v != g.acceptEod);
    DEBUG_PRINTF("partitioning vertex %zu ucp:%d\n", g[v].index, (int)ucp);

    CharReach cr_word = ucp ? CHARREACH_WORD_UCP_PRE : CHARREACH_WORD;
    CharReach cr_nonword = ucp ? CHARREACH_NONWORD_UCP_PRE : CHARREACH_NONWORD;

    auto has_no_assert = [&g](const NFAEdge &e) { return !g[e].assert_flags; };

    // Split v into word/nonword vertices with only asserting out-edges.
    NFAVertex w_out = makeClone(rm, g, expr, v, cr_word);
    NFAVertex nw_out = makeClone(rm, g, expr, v, cr_nonword);
    remove_out_edge_if(w_out, has_no_assert, g);
    remove_out_edge_if(nw_out, has_no_assert, g);

    // Split v into word/nonword vertices with only asserting in-edges.
    NFAVertex w_in = makeClone(rm, g, expr, v, cr_word);
    NFAVertex nw_in = makeClone(rm, g, expr, v, cr_nonword);
    remove_in_edge_if(w_in, has_no_assert, g);
    remove_in_edge_if(nw_in, has_no_assert, g);

    // Prune edges with asserts from original v.
    auto has_assert = [&g](const NFAEdge &e) { return g[e].assert_flags; };
    remove_in_edge_if(v, has_assert, g);
    remove_out_edge_if(v, has_assert, g);
}

static
void resolveEdges(ReportManager &rm, NGHolder &g, const ExpressionInfo &expr,
                  set<NFAEdge> *dead) {
    for (const auto &e : edges_range(g)) {
        u32 flags = g[e].assert_flags;
        if (!flags) {
            continue;
        }

        NFAVertex u = source(e, g);
        NFAVertex v = target(e, g);

        assert(u != g.startDs);

        const CharReach &u_cr = g[u].char_reach;
        const CharReach &v_cr = g[v].char_reach;

        bool impassable = true;
        bool ucp = flags & UCP_ASSERT_FLAGS;
        DEBUG_PRINTF("resolving edge %zu->%zu (flags=0x%x, ucp=%d)\n",
                     g[u].index, g[v].index, flags, (int)ucp);
        while (flags && impassable) {
            u32 flag = 1U << findAndClearLSB_32(&flags);
            switch (flag) {
            case POS_FLAG_ASSERT_NONWORD_TO_NONWORD:
            case POS_FLAG_ASSERT_NONWORD_TO_WORD:
                if ((u_cr & CHARREACH_NONWORD).none() && u != g.start) {
                    continue;
                }
                break;
            case POS_FLAG_ASSERT_WORD_TO_NONWORD:
            case POS_FLAG_ASSERT_WORD_TO_WORD:
                if ((u_cr & CHARREACH_WORD).none() || u == g.start) {
                    continue;
                }
                break;
            case POS_FLAG_ASSERT_NONWORD_TO_NONWORD_UCP:
            case POS_FLAG_ASSERT_NONWORD_TO_WORD_UCP:
                if ((u_cr & ~CHARREACH_NONWORD_UCP_PRE).any() && u != g.start) {
                    continue;
                }
                break;
            case POS_FLAG_ASSERT_WORD_TO_NONWORD_UCP:
            case POS_FLAG_ASSERT_WORD_TO_WORD_UCP:
                if ((u_cr & ~CHARREACH_WORD_UCP_PRE).any() || u == g.start) {
                    continue;
                }
                break;
            default:
                assert(0);
            }

            if (v == g.accept) {
                /* accept special will need to be treated specially later */
                impassable = false;
                continue;
            }

            switch (flag) {
            case POS_FLAG_ASSERT_NONWORD_TO_NONWORD:
            case POS_FLAG_ASSERT_WORD_TO_NONWORD:
                if ((v_cr & CHARREACH_NONWORD).none() && v != g.acceptEod) {
                    continue;
                }
                break;
            case POS_FLAG_ASSERT_WORD_TO_WORD:
            case POS_FLAG_ASSERT_NONWORD_TO_WORD:
                if ((v_cr & CHARREACH_WORD).none() || v == g.acceptEod) {
                    continue;
                }
                break;
            case POS_FLAG_ASSERT_NONWORD_TO_NONWORD_UCP:
            case POS_FLAG_ASSERT_WORD_TO_NONWORD_UCP:
                if ((v_cr & ~CHARREACH_NONWORD_UCP_PRE).any()
                    && v != g.acceptEod) {
                    continue;
                }
                break;
            case POS_FLAG_ASSERT_WORD_TO_WORD_UCP:
            case POS_FLAG_ASSERT_NONWORD_TO_WORD_UCP:
                if ((v_cr & ~CHARREACH_WORD_UCP_PRE).any()
                    || v == g.acceptEod) {
                    continue;
                }
                break;
            default:
                assert(0);
            }
            impassable = false;
        }

        if (impassable) {
            dead->insert(e);
        } else if (v == g.accept && !ucp) {
            bool u_w = (u_cr & CHARREACH_NONWORD).none() && u != g.start;
            UNUSED bool u_nw = (u_cr & CHARREACH_WORD).none() || u == g.start;
            assert(u_w != u_nw);
            bool v_w = false;
            bool v_nw = false;

            flags = g[e].assert_flags;
            if (u_w) {
                v_w = flags & POS_FLAG_ASSERT_WORD_TO_WORD;
                v_nw = flags & POS_FLAG_ASSERT_WORD_TO_NONWORD;
            } else {
                v_w = flags & POS_FLAG_ASSERT_NONWORD_TO_WORD;
                v_nw = flags & POS_FLAG_ASSERT_NONWORD_TO_NONWORD;
            }
            assert(v_w || v_nw);
            if (v_w && v_nw) {
                /* edge is effectively unconditional */
                g[e].assert_flags = 0;
            } else if (v_w) {
                /* need to add a word byte */
                NFAVertex vv = add_vertex(g);
                setReportId(rm, g, expr, vv, -1);
                g[vv].char_reach = CHARREACH_WORD;
                add_edge(vv, g.accept, g);
                g[e].assert_flags = 0;
                add_edge(u, vv, g[e], g);
                dead->insert(e);
            } else {
                /* need to add a non word byte or see eod */
                NFAVertex vv = add_vertex(g);
                setReportId(rm, g, expr, vv, -1);
                g[vv].char_reach = CHARREACH_NONWORD;
                add_edge(vv, g.accept, g);
                g[e].assert_flags = 0;
                add_edge(u, vv, g[e], g);
                /* there may already be a different edge from start to eod if so
                 * we need to make it unconditional and alive
                 */
                if (NFAEdge start_eod = edge(u, g.acceptEod, g)) {
                    g[start_eod].assert_flags = 0;
                    dead->erase(start_eod);
                } else {
                    add_edge(u, g.acceptEod, g[e], g);
                }
                dead->insert(e);
            }
        } else if (v == g.accept && ucp) {
            DEBUG_PRINTF("resolving ucp assert to accept\n");
            assert(u_cr.any());
            bool u_w = (u_cr & CHARREACH_WORD_UCP_PRE).any()
                       && u != g.start;
            bool u_nw = (u_cr & CHARREACH_NONWORD_UCP_PRE).any()
                       || u == g.start;
            assert(u_w || u_nw);

            bool v_w = false;
            bool v_nw = false;

            flags = g[e].assert_flags;
            if (u_w) {
                v_w |= flags & POS_FLAG_ASSERT_WORD_TO_WORD_UCP;
                v_nw |= flags & POS_FLAG_ASSERT_WORD_TO_NONWORD_UCP;
            }
            if (u_nw) {
                v_w |= flags & POS_FLAG_ASSERT_NONWORD_TO_WORD_UCP;
                v_nw |= flags & POS_FLAG_ASSERT_NONWORD_TO_NONWORD_UCP;
            }
            assert(v_w || v_nw);
            if (v_w && v_nw) {
                /* edge is effectively unconditional */
                g[e].assert_flags = 0;
            } else if (v_w) {
                /* need to add a word byte */
                NFAVertex vv = add_vertex(g);
                setReportId(rm, g, expr, vv, -1);
                g[vv].char_reach = CHARREACH_WORD_UCP_PRE;
                add_edge(vv, g.accept, g);
                g[e].assert_flags = 0;
                add_edge(u, vv, g[e], g);
                dead->insert(e);
            } else {
                /* need to add a non word byte or see eod */
                NFAVertex vv = add_vertex(g);
                setReportId(rm, g, expr, vv, -1);
                g[vv].char_reach = CHARREACH_NONWORD_UCP_PRE;
                add_edge(vv, g.accept, g);
                g[e].assert_flags = 0;
                add_edge(u, vv, g[e], g);
                /* there may already be a different edge from start to eod if so
                 * we need to make it unconditional and alive
                 */
                if (NFAEdge start_eod = edge(u, g.acceptEod, g)) {
                    g[start_eod].assert_flags = 0;
                    dead->erase(start_eod);
                } else {
                    add_edge(u, g.acceptEod, g[e], g);
                }
                dead->insert(e);
            }
        } else {
            /* we can remove the asserts as we have partitioned the vertices
             * into w/nw around the assert edges
             */
            g[e].assert_flags = 0;
        }
    }
}

void resolveAsserts(ReportManager &rm, NGHolder &g,
                    const ExpressionInfo &expr) {
    vector<NFAEdge> asserts = getAsserts(g);
    if (asserts.empty()) {
        return;
    }

    map<u32, NFAVertex> to_split; /* by index, for determinism */
    map<u32, NFAVertex> to_split_ucp; /* by index, for determinism */
    findSplitters(g, asserts, &to_split, &to_split_ucp);
    if (to_split.size() + to_split_ucp.size() > MAX_CLONED_VERTICES) {
        throw CompileError(expr.index, "Pattern is too large.");
    }

    for (const auto &m : to_split) {
        assert(!contains(to_split_ucp, m.first));
        splitVertex(rm, g, expr, m.second, false);
    }

    for (const auto &m : to_split_ucp) {
        splitVertex(rm, g, expr, m.second, true);
    }

    set<NFAEdge> dead;
    resolveEdges(rm, g, expr, &dead);

    remove_edges(dead, g);
    renumber_vertices(g);
    pruneUseless(g);
    pruneEmptyVertices(g);

    renumber_vertices(g);
    renumber_edges(g);
    clearReports(g);
}

void ensureCodePointStart(ReportManager &rm, NGHolder &g,
                          const ExpressionInfo &expr) {
    /* In utf8 mode there is an implicit assertion that we start at codepoint
     * boundaries. Assert resolution handles the badness coming from asserts.
     * The only other source of trouble is startDs->accept connections.
     */
    NFAEdge orig = edge(g.startDs, g.accept, g);
    if (expr.utf8 && orig) {
        DEBUG_PRINTF("rectifying %u\n", expr.report);
        Report ir = rm.getBasicInternalReport(expr);
        ReportID rep = rm.getInternalId(ir);

        NFAVertex v_a = add_vertex(g);
        g[v_a].assert_flags = POS_FLAG_VIRTUAL_START;
        g[v_a].char_reach = UTF_ASCII_CR;
        add_edge(v_a, g.accept, g[orig], g);

        NFAVertex v_2 = add_vertex(g);
        g[v_2].assert_flags = POS_FLAG_VIRTUAL_START;
        g[v_2].char_reach = CharReach(UTF_TWO_BYTE_MIN, UTF_TWO_BYTE_MAX);

        NFAVertex v_3 = add_vertex(g);
        g[v_3].assert_flags = POS_FLAG_VIRTUAL_START;
        g[v_3].char_reach = CharReach(UTF_THREE_BYTE_MIN, UTF_THREE_BYTE_MAX);

        NFAVertex v_4 = add_vertex(g);
        g[v_4].assert_flags = POS_FLAG_VIRTUAL_START;
        g[v_4].char_reach = CharReach(UTF_FOUR_BYTE_MIN, UTF_FOUR_BYTE_MAX);

        NFAVertex v_c = add_vertex(g);
        g[v_c].assert_flags = POS_FLAG_VIRTUAL_START;
        g[v_c].char_reach = UTF_CONT_CR;
        add_edge(v_c, g.accept, g[orig], g);

        add_edge(v_2, v_c, g);

        NFAVertex v_3c = add_vertex(g);
        g[v_3c].assert_flags = POS_FLAG_VIRTUAL_START;
        g[v_3c].char_reach = UTF_CONT_CR;
        add_edge(v_3c, v_c, g);
        add_edge(v_3, v_3c, g);

        NFAVertex v_4c = add_vertex(g);
        g[v_4c].assert_flags = POS_FLAG_VIRTUAL_START;
        g[v_4c].char_reach = UTF_CONT_CR;
        add_edge(v_4c, v_3c, g);
        add_edge(v_4, v_4c, g);

        g[v_a].reports.insert(rep);
        g[v_c].reports.insert(rep);

        add_edge(g.start, v_a, g);
        add_edge(g.startDs, v_a, g);
        add_edge(g.start, v_2, g);
        add_edge(g.startDs, v_2, g);
        add_edge(g.start, v_3, g);
        add_edge(g.startDs, v_3, g);
        add_edge(g.start, v_4, g);
        add_edge(g.startDs, v_4, g);
        remove_edge(orig, g);
        renumber_edges(g);
        clearReports(g);
    }
}

} // namespace ue2
