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
 * \brief Code for discovering properties of an NFA graph used by
 * hs_expression_info().
 */
#include "ng_expr_info.h"

#include "hs_internal.h"
#include "ng.h"
#include "ng_asserts.h"
#include "ng_depth.h"
#include "ng_edge_redundancy.h"
#include "ng_extparam.h"
#include "ng_fuzzy.h"
#include "ng_holder.h"
#include "ng_prune.h"
#include "ng_reports.h"
#include "ng_util.h"
#include "ue2common.h"
#include "compiler/expression_info.h"
#include "parser/position.h" // for POS flags
#include "util/boundary_reports.h"
#include "util/compile_context.h"
#include "util/depth.h"
#include "util/graph.h"
#include "util/graph_range.h"
#include "util/report_manager.h"

#include <limits.h>
#include <set>

using namespace std;

namespace ue2 {

/* get rid of leading \b and multiline ^ vertices */
static
void removeLeadingVirtualVerticesFromRoot(NGHolder &g, NFAVertex root) {
    vector<NFAVertex> victims;

    for (auto v : adjacent_vertices_range(root, g)) {
        if (g[v].assert_flags & POS_FLAG_VIRTUAL_START) {
            DEBUG_PRINTF("(?m)^ vertex or leading \\[bB] vertex\n");
            victims.push_back(v);
        }
    }

    for (auto u : victims) {
        for (auto v : adjacent_vertices_range(u, g)) {
            add_edge_if_not_present(root, v, g);
        }
    }

    remove_vertices(victims, g);
}

static
void checkVertex(const ReportManager &rm, const NGHolder &g, NFAVertex v,
                 const vector<DepthMinMax> &depths, DepthMinMax &info) {
    if (is_any_accept(v, g)) {
        return;
    }
    if (is_any_start(v, g)) {
        info.min = depth(0);
        info.max = max(info.max, depth(0));
        return;
    }

    u32 idx = g[v].index;
    assert(idx < depths.size());
    const DepthMinMax &d = depths.at(idx);

    for (ReportID report_id : g[v].reports) {
        const Report &report = rm.getReport(report_id);
        assert(report.type == EXTERNAL_CALLBACK);

        DepthMinMax rd = d;

        // Compute graph width to this report, taking any offset adjustment
        // into account.
        rd.min += report.offsetAdjust;
        rd.max += report.offsetAdjust;

        // A min_length param is a lower bound for match width.
        if (report.minLength && report.minLength <= depth::max_value()) {
            depth min_len((u32)report.minLength);
            rd.min = max(rd.min, min_len);
            rd.max = max(rd.max, min_len);
        }

        // A max_offset param is an upper bound for match width.
        if (report.maxOffset && report.maxOffset <= depth::max_value()) {
            depth max_offset((u32)report.maxOffset);
            rd.min = min(rd.min, max_offset);
            rd.max = min(rd.max, max_offset);
        }

        DEBUG_PRINTF("vertex %zu report %u: %s\n", g[v].index, report_id,
                      rd.str().c_str());

        info = unionDepthMinMax(info, rd);
    }
}

static
bool hasOffsetAdjust(const ReportManager &rm, const NGHolder &g) {
    for (const auto &report_id : all_reports(g)) {
        if (rm.getReport(report_id).offsetAdjust) {
            return true;
        }
    }
    return false;
}

void fillExpressionInfo(ReportManager &rm, const CompileContext &cc,
                        NGHolder &g, ExpressionInfo &expr,
                        hs_expr_info *info) {
    assert(info);

    // remove reports that aren't on vertices connected to accept.
    clearReports(g);

    assert(allMatchStatesHaveReports(g));

    /*
     * Note: the following set of analysis passes / transformations should
     * match those in NG::addGraph().
     */

    /* ensure utf8 starts at cp boundary */
    ensureCodePointStart(rm, g, expr);

    if (can_never_match(g)) {
        throw CompileError(expr.index, "Pattern can never match.");
    }

    bool hamming = expr.hamm_distance > 0;
    u32 e_dist = hamming ? expr.hamm_distance : expr.edit_distance;

    // validate graph's suitability for fuzzing
    validate_fuzzy_compile(g, e_dist, hamming, expr.utf8, cc.grey);

    resolveAsserts(rm, g, expr);
    assert(allMatchStatesHaveReports(g));

    // fuzz graph - this must happen before any transformations are made
    make_fuzzy(g, e_dist, hamming, cc.grey);

    pruneUseless(g);
    pruneEmptyVertices(g);

    if (can_never_match(g)) {
        throw CompileError(expr.index, "Pattern can never match.");
    }

    optimiseVirtualStarts(g);

    propagateExtendedParams(g, expr, rm);

    removeLeadingVirtualVerticesFromRoot(g, g.start);
    removeLeadingVirtualVerticesFromRoot(g, g.startDs);

    auto depths = calcDepthsFrom(g, g.start);

    DepthMinMax d;

    for (auto u : inv_adjacent_vertices_range(g.accept, g)) {
        checkVertex(rm, g, u, depths, d);
    }

    for (auto u : inv_adjacent_vertices_range(g.acceptEod, g)) {
        checkVertex(rm, g, u, depths, d);
    }

    if (d.max.is_finite()) {
        info->max_width = d.max;
    } else {
        info->max_width = UINT_MAX;
    }
    if (d.min.is_finite()) {
        info->min_width = d.min;
    } else {
        info->min_width = UINT_MAX;
    }

    info->unordered_matches = hasOffsetAdjust(rm, g);
    info->matches_at_eod = can_match_at_eod(g);
    info->matches_only_at_eod = can_only_match_at_eod(g);
}

} // namespace ue2
