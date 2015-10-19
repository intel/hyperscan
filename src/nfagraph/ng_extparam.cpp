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

/** \file
 * \brief Propagate extended parameters to vertex reports and reduce graph if
 * possible.
 *
 * This code handles the propagation of the extension parameters specified by
 * the user with the hs_expr_ext structure into the reports on the graph's
 * vertices.
 *
 * There are also some analyses that prune edges that cannot contribute to a
 * match given these constraints, or transform the graph in order to make a
 * constraint implicit.
 */
#include "ng.h"
#include "ng_depth.h"
#include "ng_dump.h"
#include "ng_extparam.h"
#include "ng_prune.h"
#include "ng_reports.h"
#include "ng_som_util.h"
#include "ng_width.h"
#include "ng_util.h"
#include "ue2common.h"
#include "parser/position.h"
#include "util/compile_context.h"
#include "util/compile_error.h"
#include "util/container.h"
#include "util/graph.h"
#include "util/graph_range.h"

#include <sstream>
#include <string>

using namespace std;

namespace ue2 {

static const u32 MAX_MAXOFFSET_TO_ANCHOR = 2000;
static const u32 MAX_MINLENGTH_TO_CONVERT = 2000;

/** \brief Find the (min, max) offset adjustment for the reports on a given
 * vertex. */
static
pair<s32,s32> getMinMaxOffsetAdjust(const ReportManager &rm,
                                    const NGHolder &g, NFAVertex v) {
    s32 minAdj = 0, maxAdj = 0;
    const auto &reports = g[v].reports;
    for (auto ri = reports.begin(), re = reports.end(); ri != re; ++ri) {
        const Report &ir = rm.getReport(*ri);
        if (ri == reports.begin()) {
            minAdj = ir.offsetAdjust;
            maxAdj = ir.offsetAdjust;
        } else {
            minAdj = min(minAdj, ir.offsetAdjust);
            maxAdj = max(maxAdj, ir.offsetAdjust);
        }
    }

    return make_pair(minAdj, maxAdj);
}

/** \brief Find the (min, max) length of any match for the given holder. */
static
DepthMinMax findMatchLengths(const ReportManager &rm, const NGHolder &g) {
    DepthMinMax match_depths;

    vector<DepthMinMax> depths = getDistancesFromSOM(g);

    pair<s32, s32> adj;

    for (auto v : inv_adjacent_vertices_range(g.accept, g)) {
        u32 idx = g[v].index;
        DepthMinMax d = depths[idx]; // copy
        adj = getMinMaxOffsetAdjust(rm, g, v);
        DEBUG_PRINTF("vertex %u: depths=%s, adj=[%d,%d]\n", idx,
                     d.str().c_str(), adj.first, adj.second);
        d.min += adj.first;
        d.max += adj.second;
        match_depths = unionDepthMinMax(match_depths, d);
    }

    for (auto v : inv_adjacent_vertices_range(g.acceptEod, g)) {
        if (v == g.accept) {
            continue;
        }
        u32 idx = g[v].index;
        DepthMinMax d = depths[idx]; // copy
        adj = getMinMaxOffsetAdjust(rm, g, v);
        DEBUG_PRINTF("vertex %u: depths=%s, adj=[%d,%d]\n", idx,
                     d.str().c_str(), adj.first, adj.second);
        d.min += adj.first;
        d.max += adj.second;
        match_depths = unionDepthMinMax(match_depths, d);
    }

    DEBUG_PRINTF("match_depths=%s\n", match_depths.str().c_str());

    assert(match_depths.min.is_reachable());
    assert(match_depths.max.is_reachable());
    return match_depths;
}

/** \brief Replace the graph's reports with new reports that specify bounds. */
static
void updateReportBounds(ReportManager &rm, NGWrapper &g, NFAVertex accept,
                        set<NFAVertex> &done) {
    for (auto v : inv_adjacent_vertices_range(accept, g)) {
        // Don't operate on g.accept itself.
        if (v == g.accept) {
            assert(accept == g.acceptEod);
            continue;
        }

        // Don't operate on a vertex we've already done.
        if (contains(done, v)) {
            continue;
        }
        done.insert(v);

        flat_set<ReportID> new_reports;
        auto &reports = g[v].reports;

        for (auto id : reports) {
            Report ir = rm.getReport(id); // make a copy
            assert(!ir.hasBounds());

            // Note that we need to cope with offset adjustment here.

            ir.minOffset = g.min_offset - ir.offsetAdjust;
            if (g.max_offset == MAX_OFFSET) {
                ir.maxOffset = MAX_OFFSET;
            } else {
                ir.maxOffset = g.max_offset - ir.offsetAdjust;
            }
            assert(ir.maxOffset >= ir.minOffset);

            ir.minLength = g.min_length;
            if (g.min_length && !g.som) {
                ir.quashSom = true;
            }

            DEBUG_PRINTF("id %u -> min_offset=%llu, max_offset=%llu, "
                         "min_length=%llu\n",
                         id, ir.minOffset, ir.maxOffset, ir.minLength);
            new_reports.insert(rm.getInternalId(ir));
        }

        DEBUG_PRINTF("swapping reports on vertex %u\n",
                     g[v].index);
        reports.swap(new_reports);
    }
}

static
bool hasVirtualStarts(const NGHolder &g) {
    for (auto v : adjacent_vertices_range(g.start, g)) {
        if (g[v].assert_flags & POS_FLAG_VIRTUAL_START) {
            return true;
        }
    }
    return false;
}

/** If the pattern is unanchored, has a max_offset and has not asked for SOM,
 * we can use that knowledge to anchor it which will limit its lifespan. Note
 * that we can't use this transformation if there's a min_length, as it's
 * currently handled using "sly SOM".
 *
 * Note that it is possible to handle graphs that have a combination of
 * anchored and unanchored paths, but it's too tricky for the moment.
 */
static
bool anchorPatternWithBoundedRepeat(NGWrapper &g, const depth &minWidth,
                                    const depth &maxWidth) {
    assert(!g.som);
    assert(g.max_offset != MAX_OFFSET);
    assert(minWidth <= maxWidth);
    assert(maxWidth.is_reachable());

    DEBUG_PRINTF("widths=[%s,%s], min/max offsets=[%llu,%llu]\n",
                 minWidth.str().c_str(), maxWidth.str().c_str(), g.min_offset,
                 g.max_offset);

    if (g.max_offset > MAX_MAXOFFSET_TO_ANCHOR) {
        return false;
    }

    if (g.max_offset < minWidth) {
        assert(0);
        return false;
    }

    // If the pattern has virtual starts, we probably don't want to touch it.
    if (hasVirtualStarts(g)) {
        DEBUG_PRINTF("virtual starts, bailing\n");
        return false;
    }

    // Similarly, bail if the pattern is vacuous. TODO: this could be done, we
    // would just need to be a little careful with reports.
    if (isVacuous(g)) {
        DEBUG_PRINTF("vacuous, bailing\n");
        return false;
    }

    u32 min_bound, max_bound;
    if (maxWidth.is_infinite()) {
        min_bound = 0;
        max_bound = g.max_offset - minWidth;
    } else {
        min_bound = g.min_offset > maxWidth ? g.min_offset - maxWidth : 0;
        max_bound = g.max_offset - minWidth;
    }

    DEBUG_PRINTF("prepending ^.{%u,%u}\n", min_bound, max_bound);

    vector<NFAVertex> initials;
    for (auto v : adjacent_vertices_range(g.startDs, g)) {
        if (v == g.startDs) {
            continue;
        }
        initials.push_back(v);
    }
    if (initials.empty()) {
        DEBUG_PRINTF("no initial vertices\n");
        return false;
    }

    // Wire up 'min_offset' mandatory dots from anchored start.
    NFAVertex u = g.start;
    for (u32 i = 0; i < min_bound; i++) {
        NFAVertex v = add_vertex(g);
        g[v].char_reach.setall();
        add_edge(u, v, g);
        u = v;
    }

    NFAVertex head = u;

    // Wire up optional dots for (max_offset - min_offset).
    for (u32 i = 0; i < max_bound - min_bound; i++) {
        NFAVertex v = add_vertex(g);
        g[v].char_reach.setall();
        if (head != u) {
            add_edge(head, v, g);
        }
        add_edge(u, v, g);
        u = v;
    }

    // Remove edges from starts and wire both head and u to our initials.
    for (auto v : initials) {
        remove_edge(g.startDs, v, g);
        remove_edge(g.start, v, g);

        if (head != u) {
            add_edge(head, v, g);
        }
        add_edge(u, v, g);
    }

    g.renumberVertices();
    g.renumberEdges();

    return true;
}

static
NFAVertex findSingleCyclic(const NGHolder &g) {
    NFAVertex v = NFAGraph::null_vertex();
    for (const auto &e : edges_range(g)) {
        if (source(e, g) == target(e, g)) {
            if (source(e, g) == g.startDs) {
                continue;
            }
            if (v != NFAGraph::null_vertex()) {
                // More than one cyclic vertex.
                return NFAGraph::null_vertex();
            }
            v = source(e, g);
        }
    }

    if (v != NFAGraph::null_vertex()) {
        DEBUG_PRINTF("cyclic is %u\n", g[v].index);
        assert(!is_special(v, g));
    }
    return v;
}

static
bool hasOffsetAdjust(const ReportManager &rm, NGWrapper &g,
                     int *adjust) {
    const auto &reports = all_reports(g);
    if (reports.empty()) {
        assert(0);
        return false;
    }

    int offsetAdjust = rm.getReport(*reports.begin()).offsetAdjust;
    for (auto report : reports) {
        const Report &ir = rm.getReport(report);
        if (ir.offsetAdjust != offsetAdjust) {
            DEBUG_PRINTF("different adjusts!\n");
            return false;
        }
    }

    *adjust = offsetAdjust;
    return true;
}

/** If the pattern has a min_length and is of "ratchet" form with one unbounded
 * repeat, that repeat can become a bounded repeat.
 *
 *     /foo.*bar/{min_length=100} --> /foo.{94,}bar/
 */
static
bool transformMinLengthToRepeat(const ReportManager &rm, NGWrapper &g) {
    assert(g.min_length);

    if (g.min_length > MAX_MINLENGTH_TO_CONVERT) {
        return false;
    }

    // If the pattern has virtual starts, we probably don't want to touch it.
    if (hasVirtualStarts(g)) {
        DEBUG_PRINTF("virtual starts, bailing\n");
        return false;
    }

    // The graph must contain a single cyclic vertex (other than startDs), and
    // that vertex can have one pred and one successor.
    NFAVertex cyclic = findSingleCyclic(g);
    if (cyclic == NFAGraph::null_vertex()) {
        return false;
    }

    NFAGraph::adjacency_iterator ai, ae;
    tie(ai, ae) = adjacent_vertices(g.start, g);
    if (*ai == g.startDs) {
        ++ai;
    }
    NFAVertex v = *ai;
    if (++ai != ae) {
        DEBUG_PRINTF("more than one initial vertex\n");
        return false;
    }

    u32 width = 0;


    // Walk from the start vertex to the cyclic state and ensure we have a
    // chain of vertices.
    while (v != cyclic) {
        DEBUG_PRINTF("vertex %u\n", g[v].index);
        width++;
        tie(ai, ae) = adjacent_vertices(v, g);
        set<NFAVertex> succ(ai, ae);
        if (contains(succ, cyclic)) {
            if (succ.size() == 1) {
                v = cyclic;
            } else if (succ.size() == 2) {
                // Cyclic and jump edge.
                succ.erase(cyclic);
                NFAVertex v2 = *succ.begin();
                if (!edge(cyclic, v2, g).second) {
                    DEBUG_PRINTF("bad form\n");
                    return false;
                }
                v = cyclic;
            } else {
                DEBUG_PRINTF("bad form\n");
                return false;
            }
        } else {
            if (succ.size() != 1) {
                DEBUG_PRINTF("bad form\n");
                return false;
            }
            v = *succ.begin();
        }
    }

    // Check the cyclic state is A-OK.
    v = getSoleDestVertex(g, cyclic);
    if (v == NFAGraph::null_vertex()) {
        DEBUG_PRINTF("cyclic has more than one successor\n");
        return false;
    }

    // Walk from the cyclic state to an accept and ensure we have a chain of
    // vertices.
    while (!is_any_accept(v, g)) {
        DEBUG_PRINTF("vertex %u\n", g[v].index);
        width++;
        tie(ai, ae) = adjacent_vertices(v, g);
        set<NFAVertex> succ(ai, ae);
        if (succ.size() != 1) {
            DEBUG_PRINTF("bad form\n");
            return false;
        }
        v = *succ.begin();
    }

    int offsetAdjust = 0;
    if (!hasOffsetAdjust(rm, g, &offsetAdjust)) {
        return false;
    }
    DEBUG_PRINTF("adjusting width by %d\n", offsetAdjust);
    width += offsetAdjust;

    DEBUG_PRINTF("width=%u, vertex %u is cyclic\n", width,
                  g[cyclic].index);

    if (width >= g.min_length) {
        DEBUG_PRINTF("min_length=%llu is guaranteed, as width=%u\n",
                      g.min_length, width);
        g.min_length = 0;
        return true;
    }

    vector<NFAVertex> preds;
    vector<NFAEdge> dead;
    for (auto u : inv_adjacent_vertices_range(cyclic, g)) {
        DEBUG_PRINTF("pred %u\n", g[u].index);
        if (u == cyclic) {
            continue;
        }
        preds.push_back(u);

        // We want to delete the out-edges of each predecessor, but need to
        // make sure we don't delete the startDs self loop.
        for (const auto &e : out_edges_range(u, g)) {
            if (target(e, g) != g.startDs) {
                dead.push_back(e);
            }
        }
    }

    remove_edges(dead, g);

    assert(!preds.empty());

    const CharReach &cr = g[cyclic].char_reach;

    for (u32 i = 0; i < g.min_length - width - 1; ++i) {
        v = add_vertex(g);
        g[v].char_reach = cr;

        for (auto u : preds) {
            add_edge(u, v, g);
        }
        preds.clear();
        preds.push_back(v);
    }
    assert(!preds.empty());
    for (auto u : preds) {
        add_edge(u, cyclic, g);
    }

    g.renumberVertices();
    g.renumberEdges();
    clearReports(g);

    g.min_length = 0;
    return true;
}

static
bool hasExtParams(const NGWrapper &g) {
    if (g.min_length != 0) {
        return true;
    }
    if (g.min_offset != 0) {
        return true;
    }
    if (g.max_offset != MAX_OFFSET) {
        return true;
    }
    return false;
}

static
depth maxDistFromStart(const NFAVertexBidiDepth &d) {
    if (!d.fromStartDotStar.max.is_unreachable()) {
        // A path from startDs, any path, implies we can match at any offset.
        return depth::infinity();
    }
    return d.fromStart.max;
}

static
const depth& maxDistToAccept(const NFAVertexBidiDepth &d) {
    if (d.toAccept.max.is_unreachable()) {
        return d.toAcceptEod.max;
    } else if (d.toAcceptEod.max.is_unreachable()) {
        return d.toAccept.max;
    }
    return max(d.toAccept.max, d.toAcceptEod.max);
}

static
const depth& minDistFromStart(const NFAVertexBidiDepth &d) {
    return min(d.fromStartDotStar.min, d.fromStart.min);
}

static
const depth& minDistToAccept(const NFAVertexBidiDepth &d) {
    return min(d.toAccept.min, d.toAcceptEod.min);
}

static
bool isEdgePrunable(const NGWrapper &g,
                    const vector<NFAVertexBidiDepth> &depths,
                    const NFAEdge &e) {
    const NFAVertex u = source(e, g);
    const NFAVertex v = target(e, g);

    DEBUG_PRINTF("edge (%u,%u)\n", g[u].index,
                 g[v].index);

    // Leave our special-to-special edges alone.
    if (is_special(u, g) && is_special(v, g)) {
        DEBUG_PRINTF("ignoring special-to-special\n");
        return false;
    }

    // We must be careful around start: we don't want to remove (start, v) if
    // (startDs, v) exists as well, since later code will assume the presence
    // of both edges, but other cases are OK.
    if (u == g.start && edge(g.startDs, v, g).second) {
        DEBUG_PRINTF("ignoring unanchored start edge\n");
        return false;
    }

    u32 u_idx = g[u].index;
    u32 v_idx = g[v].index;
    assert(u_idx < depths.size() && v_idx < depths.size());

    const NFAVertexBidiDepth &du = depths.at(u_idx);
    const NFAVertexBidiDepth &dv = depths.at(v_idx);

    if (g.min_offset) {
        depth max_offset = maxDistFromStart(du) + maxDistToAccept(dv);
        if (max_offset.is_finite() && max_offset < g.min_offset) {
            DEBUG_PRINTF("max_offset=%s too small\n", max_offset.str().c_str());
            return true;
        }
    }

    if (g.max_offset != MAX_OFFSET) {
        depth min_offset = minDistFromStart(du) + minDistToAccept(dv);
        assert(min_offset.is_finite());

        if (min_offset > g.max_offset) {
            DEBUG_PRINTF("min_offset=%s too large\n", min_offset.str().c_str());
            return true;
        }
    }

    if (g.min_length && is_any_accept(v, g)) {
        // Simple take on min_length. If we're an edge to accept and our max
        // dist from start is too small, we can be pruned.
        const depth &width = du.fromStart.max;
        if (width.is_finite() && width < g.min_length) {
            DEBUG_PRINTF("max width %s from start too small for min_length\n",
                         width.str().c_str());
            return true;
        }
    }

    return false;
}

static
void pruneExtUnreachable(NGWrapper &g) {
    vector<NFAVertexBidiDepth> depths;
    calcDepths(g, depths);

    vector<NFAEdge> dead;

    for (const auto &e : edges_range(g)) {
        if (isEdgePrunable(g, depths, e)) {
            DEBUG_PRINTF("pruning\n");
            dead.push_back(e);
        }
    }

    if (dead.empty()) {
        return;
    }

    remove_edges(dead, g);
    pruneUseless(g);
}

/** Remove vacuous edges in graphs where the min_offset or min_length
 * constraints dictate that they can never produce a match. */
static
void pruneVacuousEdges(NGWrapper &g) {
    if (!g.min_length && !g.min_offset) {
        return;
    }

    vector<NFAEdge> dead;

    for (const auto &e : edges_range(g)) {
        const NFAVertex u = source(e, g);
        const NFAVertex v = target(e, g);

        // Special case: Crudely remove vacuous edges from start in graphs with a
        // min_offset.
        if (g.min_offset && u == g.start && is_any_accept(v, g)) {
            DEBUG_PRINTF("vacuous edge in graph with min_offset!\n");
            dead.push_back(e);
            continue;
        }

        // If a min_length is set, vacuous edges can be removed.
        if (g.min_length && is_any_start(u, g) && is_any_accept(v, g)) {
            DEBUG_PRINTF("vacuous edge in graph with min_length!\n");
            dead.push_back(e);
            continue;
        }
    }

    if (dead.empty()) {
        return;
    }

    remove_edges(dead, g);
    pruneUseless(g);
}

static
void pruneUnmatchable(NGWrapper &g, const vector<DepthMinMax> &depths,
                      const ReportManager &rm, NFAVertex accept) {
    vector<NFAEdge> dead;

    for (const auto &e : in_edges_range(accept, g)) {
        NFAVertex v = source(e, g);
        if (v == g.accept) {
            assert(accept == g.acceptEod); // stylised edge
            continue;
        }

        u32 idx = g[v].index;
        DepthMinMax d = depths[idx]; // copy
        pair<s32, s32> adj = getMinMaxOffsetAdjust(rm, g, v);
        DEBUG_PRINTF("vertex %u: depths=%s, adj=[%d,%d]\n", idx,
                     d.str().c_str(), adj.first, adj.second);
        d.min += adj.first;
        d.max += adj.second;

        if (d.max.is_finite() && d.max < g.min_length) {
            DEBUG_PRINTF("prune, max match length %s < min_length=%llu\n",
                         d.max.str().c_str(), g.min_length);
            dead.push_back(e);
            continue;
        }

        if (g.max_offset != MAX_OFFSET && d.min > g.max_offset) {
            DEBUG_PRINTF("prune, min match length %s > max_offset=%llu\n",
                         d.min.str().c_str(), g.max_offset);
            dead.push_back(e);
            continue;
        }
    }

    remove_edges(dead, g);
}

/** Remove edges to accepts that can never produce a match long enough to
 * satisfy our min_length and max_offset constraints. */
static
void pruneUnmatchable(NGWrapper &g, const ReportManager &rm) {
    if (!g.min_length) {
        return;
    }

    vector<DepthMinMax> depths = getDistancesFromSOM(g);

    pruneUnmatchable(g, depths, rm, g.accept);
    pruneUnmatchable(g, depths, rm, g.acceptEod);

    pruneUseless(g);
}

static
bool isUnanchored(const NGHolder &g) {
    for (auto v : adjacent_vertices_range(g.start, g)) {
        if (!edge(g.startDs, v, g).second) {
            DEBUG_PRINTF("fail, %u is anchored vertex\n",
                         g[v].index);
            return false;
        }
    }
    return true;
}

static
bool hasOffsetAdjustments(const ReportManager &rm, const NGHolder &g) {
    for (auto report : all_reports(g)) {
        const Report &ir = rm.getReport(report);
        if (ir.offsetAdjust) {
            return true;
        }
    }
    return false;
}

void handleExtendedParams(ReportManager &rm, NGWrapper &g,
                          UNUSED const CompileContext &cc) {
    if (!hasExtParams(g)) {
        return;
    }

    depth minWidth = findMinWidth(g);
    depth maxWidth = findMaxWidth(g);
    bool is_anchored = !has_proper_successor(g.startDs, g)
                     && out_degree(g.start, g);
    bool has_offset_adj = hasOffsetAdjustments(rm, g);

    DEBUG_PRINTF("minWidth=%s, maxWidth=%s, anchored=%d, offset_adj=%d\n",
                 minWidth.str().c_str(), maxWidth.str().c_str(), is_anchored,
                 has_offset_adj);

    DepthMinMax match_depths = findMatchLengths(rm, g);
    DEBUG_PRINTF("match depths %s\n", match_depths.str().c_str());

    if (is_anchored && maxWidth.is_finite() && g.min_offset > maxWidth) {
        ostringstream oss;
        oss << "Expression is anchored and cannot satisfy min_offset="
            << g.min_offset << " as it can only produce matches of length "
            << maxWidth << " bytes at most.";
        throw CompileError(g.expressionIndex, oss.str());
    }

    if (minWidth > g.max_offset) {
        ostringstream oss;
        oss << "Expression has max_offset=" << g.max_offset << " but requires "
             << minWidth << " bytes to match.";
        throw CompileError(g.expressionIndex, oss.str());
    }

    if (maxWidth.is_finite() && match_depths.max < g.min_length) {
        ostringstream oss;
        oss << "Expression has min_length=" << g.min_length << " but can "
            "only produce matches of length " << match_depths.max <<
            " bytes at most.";
        throw CompileError(g.expressionIndex, oss.str());
    }

    if (g.min_length && g.min_length <= match_depths.min) {
        DEBUG_PRINTF("min_length=%llu constraint is unnecessary\n",
                     g.min_length);
        g.min_length = 0;
    }

    if (!hasExtParams(g)) {
        return;
    }

    pruneVacuousEdges(g);
    pruneUnmatchable(g, rm);

    if (!has_offset_adj) {
        pruneExtUnreachable(g);
    }

    // We may have removed all the edges to accept, in which case this
    // expression cannot match.
    if (in_degree(g.accept, g) == 0 && in_degree(g.acceptEod, g) == 1) {
        throw CompileError(g.expressionIndex, "Extended parameter "
                "constraints can not be satisfied for any match from "
                "this expression.");
    }

    // Remove reports on vertices without an edge to accept (which have been
    // pruned above).
    clearReports(g);

    // Recalc.
    minWidth = findMinWidth(g);
    maxWidth = findMaxWidth(g);
    is_anchored = proper_out_degree(g.startDs, g) == 0 &&
                  out_degree(g.start, g);
    has_offset_adj = hasOffsetAdjustments(rm, g);

    // If the pattern is completely anchored and has a min_length set, this can
    // be converted to a min_offset.
    if (g.min_length && (g.min_offset <= g.min_length) && is_anchored) {
        DEBUG_PRINTF("converting min_length to min_offset=%llu for "
                     "anchored case\n", g.min_length);
        g.min_offset = g.min_length;
        g.min_length = 0;
    }

    if (g.min_offset && g.min_offset <= minWidth && !has_offset_adj) {
        DEBUG_PRINTF("min_offset=%llu constraint is unnecessary\n",
                     g.min_offset);
        g.min_offset = 0;
    }

    if (!hasExtParams(g)) {
        return;
    }

    // If the pattern has a min_length and is of "ratchet" form with one
    // unbounded repeat, that repeat can become a bounded repeat.
    // e.g. /foo.*bar/{min_length=100} --> /foo.{94,}bar/
    if (g.min_length && transformMinLengthToRepeat(rm, g)) {
        DEBUG_PRINTF("converted min_length to bounded repeat\n");
        // recalc
        minWidth = findMinWidth(g);
    }

    // If the pattern is unanchored, has a max_offset and has not asked for
    // SOM, we can use that knowledge to anchor it which will limit its
    // lifespan. Note that we can't use this transformation if there's a
    // min_length, as it's currently handled using "sly SOM".

    // Note that it is possible to handle graphs that have a combination of
    // anchored and unanchored paths, but it's too tricky for the moment.

    if (g.max_offset != MAX_OFFSET && !g.som && !g.min_length &&
                !has_offset_adj && isUnanchored(g)) {
        if (anchorPatternWithBoundedRepeat(g, minWidth, maxWidth)) {
            DEBUG_PRINTF("minWidth=%s, maxWidth=%s\n", minWidth.str().c_str(),
                         maxWidth.str().c_str());
            if (minWidth == maxWidth) {
                // For a fixed width pattern, we can retire the offsets as they
                // are implicit in the graph now.
                g.min_offset = 0;
                g.max_offset = MAX_OFFSET;
            }
        }
    }
    //dumpGraph("final.dot", g.g);

    if (!hasExtParams(g)) {
        return;
    }

    set<NFAVertex> done;
    updateReportBounds(rm, g, g.accept, done);
    updateReportBounds(rm, g, g.acceptEod, done);
}

} // namespace ue2
