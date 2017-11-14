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
 * \file
 * \brief Propagate extended parameters to vertex reports and reduce graph if
 * possible.
 *
 * This code handles the propagation of the extension parameters specified by
 * the user with the \ref hs_expr_ext structure into the reports on the graph's
 * vertices.
 *
 * There are also some analyses that prune edges that cannot contribute to a
 * match given these constraints, or transform the graph in order to make a
 * constraint implicit.
 */

#include "ng_extparam.h"

#include "ng.h"
#include "ng_depth.h"
#include "ng_dump.h"
#include "ng_prune.h"
#include "ng_reports.h"
#include "ng_som_util.h"
#include "ng_width.h"
#include "ng_util.h"
#include "ue2common.h"
#include "compiler/compiler.h"
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

/** True if all the given reports have the same extparam bounds. */
template<typename Container>
bool hasSameBounds(const Container &reports, const ReportManager &rm) {
    assert(!reports.empty());

    const auto &first = rm.getReport(*reports.begin());
    for (auto id : reports) {
        const auto &report = rm.getReport(id);
        if (report.minOffset != first.minOffset ||
            report.maxOffset != first.maxOffset ||
            report.minLength != first.minLength) {
            return false;
        }
    }

    return true;
}

/**
 * \brief Find the (min, max) offset adjustment for the reports on a given
 * vertex.
 */
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

template<typename Function>
void replaceReports(NGHolder &g, NFAVertex accept, flat_set<NFAVertex> &seen,
                    Function func) {
    for (auto v : inv_adjacent_vertices_range(accept, g)) {
        if (v == g.accept) {
            // Don't operate on accept: the accept->acceptEod edge is stylised.
            assert(accept == g.acceptEod);
            assert(g[v].reports.empty());
            continue;
        }

        if (!seen.insert(v).second) {
            continue; // We have already processed v.
        }

        auto &reports = g[v].reports;
        if (reports.empty()) {
            continue;
        }
        decltype(g[v].reports) new_reports;
        for (auto id : g[v].reports) {
            new_reports.insert(func(v, id));
        }
        reports = std::move(new_reports);
    }
}

/**
 * Generic function for replacing all the reports in the graph.
 *
 * Pass this a function that takes a vertex and a ReportID returns another
 * ReportID (or the same one) to replace it with.
 */
template<typename Function>
void replaceReports(NGHolder &g, Function func) {
    flat_set<NFAVertex> seen;
    replaceReports(g, g.accept, seen, func);
    replaceReports(g, g.acceptEod, seen, func);
}

/** \brief Replace the graph's reports with new reports that specify bounds. */
static
void updateReportBounds(ReportManager &rm, NGHolder &g,
                        const ExpressionInfo &expr) {
    DEBUG_PRINTF("updating report bounds\n");
    replaceReports(g, [&](NFAVertex, ReportID id) {
        Report report = rm.getReport(id); // make a copy
        assert(!report.hasBounds());

        // Note that we need to cope with offset adjustment here.

        report.minOffset = expr.min_offset - report.offsetAdjust;
        if (expr.max_offset == MAX_OFFSET) {
            report.maxOffset = MAX_OFFSET;
        } else {
            report.maxOffset = expr.max_offset - report.offsetAdjust;
        }
        assert(report.maxOffset >= report.minOffset);

        report.minLength = expr.min_length;
        if (expr.min_length && !expr.som) {
            report.quashSom = true;
        }

        DEBUG_PRINTF("id %u -> min_offset=%llu, max_offset=%llu, "
                     "min_length=%llu\n", id, report.minOffset,
                     report.maxOffset, report.minLength);

        return rm.getInternalId(report);
    });
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

/** Set the min_length param for all reports to zero.  */
static
void clearMinLengthParam(NGHolder &g, ReportManager &rm) {
    DEBUG_PRINTF("clearing min length\n");
    replaceReports(g, [&rm](NFAVertex, ReportID id) {
        const auto &report = rm.getReport(id);
        if (report.minLength) {
            Report new_report = report;
            new_report.minLength = 0;
            return rm.getInternalId(new_report);
        }
        return id;
    });
}

/**
 * Set the min_offset param to zero and the max_offset param to MAX_OFFSET for
 * all reports.
 */
static
void clearOffsetParams(NGHolder &g, ReportManager &rm) {
    DEBUG_PRINTF("clearing min and max offset\n");
    replaceReports(g, [&rm](NFAVertex, ReportID id) {
        const auto &report = rm.getReport(id);
        if (report.minLength) {
            Report new_report = report;
            new_report.minOffset = 0;
            new_report.maxOffset = MAX_OFFSET;
            return rm.getInternalId(new_report);
        }
        return id;
    });
}

/**
 * If the pattern is unanchored, has a max_offset and has not asked for SOM, we
 * can use that knowledge to anchor it which will limit its lifespan. Note that
 * we can't use this transformation if there's a min_length, as it's currently
 * handled using "sly SOM".
 *
 * Note that it is possible to handle graphs that have a combination of
 * anchored and unanchored paths, but it's too tricky for the moment.
 */
static
bool anchorPatternWithBoundedRepeat(NGHolder &g, ReportManager &rm) {
    if (!isFloating(g)) {
        return false;
    }

    const auto &reports = all_reports(g);
    if (reports.empty()) {
        return false;
    }

    if (any_of_in(reports, [&](ReportID id) {
            const auto &report = rm.getReport(id);
            return report.maxOffset == MAX_OFFSET || report.minLength ||
                   report.offsetAdjust;
        })) {
        return false;
    }

    if (!hasSameBounds(reports, rm)) {
        DEBUG_PRINTF("mixed report bounds\n");
        return false;
    }

    const depth minWidth = findMinWidth(g);
    const depth maxWidth = findMaxWidth(g);

    assert(minWidth <= maxWidth);
    assert(maxWidth.is_reachable());

    const auto &first_report = rm.getReport(*reports.begin());
    const auto min_offset = first_report.minOffset;
    const auto max_offset = first_report.maxOffset;
    assert(max_offset < MAX_OFFSET);

    DEBUG_PRINTF("widths=[%s,%s], min/max offsets=[%llu,%llu]\n",
                 minWidth.str().c_str(), maxWidth.str().c_str(),
                 min_offset, max_offset);

    if (max_offset > MAX_MAXOFFSET_TO_ANCHOR) {
        return false;
    }

    if (max_offset < minWidth) {
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
        max_bound = max_offset - minWidth;
    } else {
        min_bound = min_offset > maxWidth ? min_offset - maxWidth : 0;
        max_bound = max_offset - minWidth;
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

    renumber_vertices(g);
    renumber_edges(g);

    if (minWidth == maxWidth) {
        // For a fixed width pattern, we can retire the offsets as
        // they are implicit in the graph now.
        clearOffsetParams(g, rm);
    }

    clearReports(g);
    return true;
}

static
NFAVertex findSingleCyclic(const NGHolder &g) {
    NFAVertex v = NGHolder::null_vertex();
    for (const auto &e : edges_range(g)) {
        if (source(e, g) == target(e, g)) {
            if (source(e, g) == g.startDs) {
                continue;
            }
            if (v != NGHolder::null_vertex()) {
                // More than one cyclic vertex.
                return NGHolder::null_vertex();
            }
            v = source(e, g);
        }
    }

    if (v != NGHolder::null_vertex()) {
        DEBUG_PRINTF("cyclic is %zu\n", g[v].index);
        assert(!is_special(v, g));
    }
    return v;
}

static
bool hasOffsetAdjust(const ReportManager &rm, NGHolder &g,
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

/**
 * If the pattern has a min_length and is of "ratchet" form with one unbounded
 * repeat, that repeat can become a bounded repeat.
 *
 *     /foo.*bar/{min_length=100} --> /foo.{94,}bar/
 */
static
bool transformMinLengthToRepeat(NGHolder &g, ReportManager &rm) {
    const auto &reports = all_reports(g);

    if (reports.empty()) {
        return false;
    }

    if (!hasSameBounds(reports, rm)) {
        DEBUG_PRINTF("mixed report bounds\n");
        return false;
    }

    const auto &min_length = rm.getReport(*reports.begin()).minLength;
    if (!min_length || min_length > MAX_MINLENGTH_TO_CONVERT) {
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
    if (cyclic == NGHolder::null_vertex()) {
        return false;
    }

    NGHolder::adjacency_iterator ai, ae;
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
        DEBUG_PRINTF("vertex %zu\n", g[v].index);
        width++;
        auto succ = succs(v, g);
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
    if (v == NGHolder::null_vertex()) {
        DEBUG_PRINTF("cyclic has more than one successor\n");
        return false;
    }

    // Walk from the cyclic state to an accept and ensure we have a chain of
    // vertices.
    while (!is_any_accept(v, g)) {
        DEBUG_PRINTF("vertex %zu\n", g[v].index);
        width++;
        auto succ = succs(v, g);
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

    DEBUG_PRINTF("width=%u, vertex %zu is cyclic\n", width,
                  g[cyclic].index);

    if (width >= min_length) {
        DEBUG_PRINTF("min_length=%llu is guaranteed, as width=%u\n",
                      min_length, width);
        clearMinLengthParam(g, rm);
        return true;
    }

    vector<NFAVertex> preds;
    vector<NFAEdge> dead;
    for (auto u : inv_adjacent_vertices_range(cyclic, g)) {
        DEBUG_PRINTF("pred %zu\n", g[u].index);
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

    for (u32 i = 0; i < min_length - width - 1; ++i) {
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

    renumber_vertices(g);
    renumber_edges(g);
    clearMinLengthParam(g, rm);
    clearReports(g);
    return true;
}

static
bool hasExtParams(const ExpressionInfo &expr) {
    if (expr.min_length != 0) {
        return true;
    }
    if (expr.min_offset != 0) {
        return true;
    }
    if (expr.max_offset != MAX_OFFSET) {
        return true;
    }
    return false;
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
bool isEdgePrunable(const NGHolder &g, const Report &report,
                    const vector<NFAVertexBidiDepth> &depths,
                    const NFAEdge &e) {
    const NFAVertex u = source(e, g);
    const NFAVertex v = target(e, g);

    DEBUG_PRINTF("edge (%zu,%zu)\n", g[u].index, g[v].index);

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

    if (report.minOffset) {
        depth max_offset = maxDistFromStartOfData(du) + maxDistToAccept(dv);
        if (max_offset.is_finite() && max_offset < report.minOffset) {
            DEBUG_PRINTF("max_offset=%s too small\n", max_offset.str().c_str());
            return true;
        }
    }

    if (report.maxOffset != MAX_OFFSET) {
        depth min_offset = minDistFromStart(du) + minDistToAccept(dv);
        assert(min_offset.is_finite());

        if (min_offset > report.maxOffset) {
            DEBUG_PRINTF("min_offset=%s too large\n", min_offset.str().c_str());
            return true;
        }
    }

    if (report.minLength && is_any_accept(v, g)) {
        // Simple take on min_length. If we're an edge to accept and our max
        // dist from start is too small, we can be pruned.
        const depth &width = maxDistFromInit(du);
        if (width.is_finite() && width < report.minLength) {
            DEBUG_PRINTF("max width %s from start too small for min_length\n",
                         width.str().c_str());
            return true;
        }
    }

    return false;
}

static
void pruneExtUnreachable(NGHolder &g, const ReportManager &rm) {
    const auto &reports = all_reports(g);
    if (reports.empty()) {
        return;
    }

    if (!hasSameBounds(reports, rm)) {
        DEBUG_PRINTF("report bounds vary\n");
        return;
    }

    const auto &report = rm.getReport(*reports.begin());

    auto depths = calcBidiDepths(g);

    vector<NFAEdge> dead;

    for (const auto &e : edges_range(g)) {
        if (isEdgePrunable(g, report, depths, e)) {
            DEBUG_PRINTF("pruning\n");
            dead.push_back(e);
        }
    }

    if (dead.empty()) {
        return;
    }

    remove_edges(dead, g);
    pruneUseless(g);
    clearReports(g);
}

/**
 * Remove vacuous edges in graphs where the min_offset or min_length
 * constraints dictate that they can never produce a match.
 */
static
void pruneVacuousEdges(NGHolder &g, const ReportManager &rm) {
    vector<NFAEdge> dead;

    auto has_min_offset = [&](NFAVertex v) {
        assert(!g[v].reports.empty()); // must be reporter
        return all_of_in(g[v].reports, [&](ReportID id) {
            return rm.getReport(id).minOffset > 0;
        });
    };

    auto has_min_length = [&](NFAVertex v) {
        assert(!g[v].reports.empty()); // must be reporter
        return all_of_in(g[v].reports, [&](ReportID id) {
            return rm.getReport(id).minLength > 0;
        });
    };

    for (const auto &e : edges_range(g)) {
        const NFAVertex u = source(e, g);
        const NFAVertex v = target(e, g);

        // Special case: Crudely remove vacuous edges from start in graphs with
        // a min_offset.
        if (u == g.start && is_any_accept(v, g) && has_min_offset(u)) {
            DEBUG_PRINTF("vacuous edge in graph with min_offset!\n");
            dead.push_back(e);
            continue;
        }

        // If a min_length is set, vacuous edges can be removed.
        if (is_any_start(u, g) && is_any_accept(v, g) && has_min_length(u)) {
            DEBUG_PRINTF("vacuous edge in graph with min_length!\n");
            dead.push_back(e);
            continue;
        }
    }

    if (dead.empty()) {
        return;
    }

    DEBUG_PRINTF("removing %zu vacuous edges\n", dead.size());
    remove_edges(dead, g);
    pruneUseless(g);
    clearReports(g);
}

static
void pruneUnmatchable(NGHolder &g, const vector<DepthMinMax> &depths,
                      const ReportManager &rm, NFAVertex accept) {
    vector<NFAEdge> dead;

    for (const auto &e : in_edges_range(accept, g)) {
        NFAVertex v = source(e, g);
        if (v == g.accept) {
            assert(accept == g.acceptEod); // stylised edge
            continue;
        }

        if (!hasSameBounds(g[v].reports, rm)) {
            continue;
        }
        const auto &report = rm.getReport(*g[v].reports.begin());

        u32 idx = g[v].index;
        DepthMinMax d = depths[idx]; // copy
        pair<s32, s32> adj = getMinMaxOffsetAdjust(rm, g, v);
        DEBUG_PRINTF("vertex %u: depths=%s, adj=[%d,%d]\n", idx,
                     d.str().c_str(), adj.first, adj.second);
        d.min += adj.first;
        d.max += adj.second;

        if (d.max.is_finite() && d.max < report.minLength) {
            DEBUG_PRINTF("prune, max match length %s < min_length=%llu\n",
                         d.max.str().c_str(), report.minLength);
            dead.push_back(e);
            continue;
        }

        if (report.maxOffset != MAX_OFFSET && d.min > report.maxOffset) {
            DEBUG_PRINTF("prune, min match length %s > max_offset=%llu\n",
                         d.min.str().c_str(), report.maxOffset);
            dead.push_back(e);
            continue;
        }
    }

    remove_edges(dead, g);
}

/**
 * Remove edges to accepts that can never produce a match long enough to
 * satisfy our min_length and max_offset constraints.
 */
static
void pruneUnmatchable(NGHolder &g, const ReportManager &rm) {
    if (!any_of_in(all_reports(g), [&](ReportID id) {
            return rm.getReport(id).minLength > 0;
        })) {
        return;
    }

    vector<DepthMinMax> depths = getDistancesFromSOM(g);

    pruneUnmatchable(g, depths, rm, g.accept);
    pruneUnmatchable(g, depths, rm, g.acceptEod);

    pruneUseless(g);
    clearReports(g);
}

static
bool hasOffsetAdjustments(const ReportManager &rm, const NGHolder &g) {
    return any_of_in(all_reports(g), [&rm](ReportID id) {
        return rm.getReport(id).offsetAdjust != 0;
    });
}

void propagateExtendedParams(NGHolder &g, ExpressionInfo &expr,
                             ReportManager &rm) {
    if (!hasExtParams(expr)) {
        return;
    }

    depth minWidth = findMinWidth(g);
    depth maxWidth = findMaxWidth(g);
    bool is_anchored = !has_proper_successor(g.startDs, g)
                     && out_degree(g.start, g);

    DepthMinMax match_depths = findMatchLengths(rm, g);
    DEBUG_PRINTF("match depths %s\n", match_depths.str().c_str());

    if (is_anchored && maxWidth.is_finite() && expr.min_offset > maxWidth) {
        ostringstream oss;
        oss << "Expression is anchored and cannot satisfy min_offset="
            << expr.min_offset << " as it can only produce matches of length "
            << maxWidth << " bytes at most.";
        throw CompileError(expr.index, oss.str());
    }

    if (minWidth > expr.max_offset) {
        ostringstream oss;
        oss << "Expression has max_offset=" << expr.max_offset
            << " but requires " << minWidth << " bytes to match.";
        throw CompileError(expr.index, oss.str());
    }

    if (maxWidth.is_finite() && match_depths.max < expr.min_length) {
        ostringstream oss;
        oss << "Expression has min_length=" << expr.min_length << " but can "
            "only produce matches of length " << match_depths.max <<
            " bytes at most.";
        throw CompileError(expr.index, oss.str());
    }

    if (expr.min_length && expr.min_length <= match_depths.min) {
        DEBUG_PRINTF("min_length=%llu constraint is unnecessary\n",
                     expr.min_length);
        expr.min_length = 0;
    }

    if (!hasExtParams(expr)) {
        return;
    }

    updateReportBounds(rm, g, expr);
}

/**
 * If the pattern is completely anchored and has a min_length set, this can
 * be converted to a min_offset.
 */
static
void replaceMinLengthWithOffset(NGHolder &g, ReportManager &rm) {
    if (has_proper_successor(g.startDs, g)) {
        return; // not wholly anchored
    }

    replaceReports(g, [&rm](NFAVertex, ReportID id) {
        const auto &report = rm.getReport(id);
        if (report.minLength) {
            Report new_report = report;
            u64a min_len_offset = report.minLength - report.offsetAdjust;
            new_report.minOffset = max(report.minOffset, min_len_offset);
            new_report.minLength = 0;
            return rm.getInternalId(new_report);
        }
        return id;
    });
}

/**
 * Clear offset bounds on reports that are not needed because they're satisfied
 * by vertex depth.
 */
static
void removeUnneededOffsetBounds(NGHolder &g, ReportManager &rm) {
    auto depths = calcDepths(g);

    replaceReports(g, [&](NFAVertex v, ReportID id) {
        const auto &d = depths.at(g[v].index);
        const depth &min_depth = min(d.fromStartDotStar.min, d.fromStart.min);
        const depth &max_depth = maxDistFromStartOfData(d);

        DEBUG_PRINTF("vertex %zu has min_depth=%s, max_depth=%s\n", g[v].index,
                     min_depth.str().c_str(), max_depth.str().c_str());

        Report report = rm.getReport(id); // copy
        bool modified = false;
        if (report.minOffset && !report.offsetAdjust &&
            report.minOffset <= min_depth) {
            report.minOffset = 0;
            modified = true;
        }
        if (report.maxOffset != MAX_OFFSET && max_depth.is_finite() &&
            report.maxOffset >= max_depth) {
            report.maxOffset = MAX_OFFSET;
            modified = true;
        }
        if (modified) {
            DEBUG_PRINTF("vertex %zu, changed bounds to [%llu,%llu]\n",
                         g[v].index, report.minOffset, report.maxOffset);
            return rm.getInternalId(report);
        }

        return id;
    });
}

void reduceExtendedParams(NGHolder &g, ReportManager &rm, som_type som) {
    if (!any_of_in(all_reports(g),
                   [&](ReportID id) { return rm.getReport(id).hasBounds(); })) {
        DEBUG_PRINTF("no extparam bounds\n");
        return;
    }

    DEBUG_PRINTF("graph has extparam bounds\n");

    pruneVacuousEdges(g, rm);
    if (can_never_match(g)) {
        return;
    }

    pruneUnmatchable(g, rm);
    if (can_never_match(g)) {
        return;
    }

    if (!hasOffsetAdjustments(rm, g)) {
        pruneExtUnreachable(g, rm);
        if (can_never_match(g)) {
            return;
        }
    }

    replaceMinLengthWithOffset(g, rm);
    if (can_never_match(g)) {
        return;
    }

    // If the pattern has a min_length and is of "ratchet" form with one
    // unbounded repeat, that repeat can become a bounded repeat.
    // e.g. /foo.*bar/{min_length=100} --> /foo.{94,}bar/
    transformMinLengthToRepeat(g, rm);
    if (can_never_match(g)) {
        return;
    }

    // If the pattern is unanchored, has a max_offset and has not asked for
    // SOM, we can use that knowledge to anchor it which will limit its
    // lifespan. Note that we can't use this transformation if there's a
    // min_length, as it's currently handled using "sly SOM".
    if (som == SOM_NONE) {
        anchorPatternWithBoundedRepeat(g, rm);
        if (can_never_match(g)) {
            return;
        }
    }

    removeUnneededOffsetBounds(g, rm);
}

} // namespace ue2
