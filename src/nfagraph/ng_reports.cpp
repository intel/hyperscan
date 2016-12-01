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
 * \brief Utility functions for working with Report ID sets.
 */
#include "ng_reports.h"

#include "ng_holder.h"
#include "util/container.h"
#include "util/compile_context.h"
#include "util/graph_range.h"
#include "util/report_manager.h"

using namespace std;

namespace ue2 {

/** Returns the set of all reports in the graph. */
set<ReportID> all_reports(const NGHolder &g) {
    set<ReportID> rv;
    for (auto v : inv_adjacent_vertices_range(g.accept, g)) {
        insert(&rv, g[v].reports);
    }
    for (auto v : inv_adjacent_vertices_range(g.acceptEod, g)) {
        insert(&rv, g[v].reports);
    }

    return rv;
}

/** True if *all* reports in the graph are exhaustible. */
bool can_exhaust(const NGHolder &g, const ReportManager &rm) {
    for (ReportID report_id : all_reports(g)) {
        if (rm.getReport(report_id).ekey == INVALID_EKEY) {
            return false;
        }
    }

    return true;
}

void set_report(NGHolder &g, ReportID internal_report) {
    // First, wipe the report IDs on all vertices.
    for (auto v : vertices_range(g)) {
        g[v].reports.clear();
    }

    // Any predecessors of accept get our id.
    for (auto v : inv_adjacent_vertices_range(g.accept, g)) {
        g[v].reports.insert(internal_report);
    }

    // Same for preds of acceptEod, except accept itself.
    for (auto v : inv_adjacent_vertices_range(g.acceptEod, g)) {
        if (v == g.accept) {
            continue;
        }
        g[v].reports.insert(internal_report);
    }
}

/** Derive a maximum offset for the graph from the max_offset values of its
 * reports. Returns MAX_OFFSET for inf. */
u64a findMaxOffset(const NGHolder &g, const ReportManager &rm) {
    u64a maxOffset = 0;
    set<ReportID> reports = all_reports(g);
    assert(!reports.empty());

    for (ReportID report_id : all_reports(g)) {
        const Report &ir = rm.getReport(report_id);
        if (ir.hasBounds()) {
            maxOffset = max(maxOffset, ir.maxOffset);
        } else {
            return MAX_OFFSET;
        }
    }
    return maxOffset;
}

} // namespace ue2
