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
 * \brief Build code for vacuous graphs.
 */
#include "ng_vacuous.h"

#include "grey.h"
#include "ng.h"
#include "ng_util.h"

using namespace std;

namespace ue2 {

static
ReportID getInternalId(ReportManager &rm, const NGWrapper &graph) {
    Report ir = rm.getBasicInternalReport(graph);

    // Apply any extended params.
    if (graph.min_offset || graph.max_offset != MAX_OFFSET) {
        ir.minOffset = graph.min_offset;
        ir.maxOffset = graph.max_offset;
    }

    assert(!graph.min_length); // should be handled elsewhere.

    return rm.getInternalId(ir);
}

static
void makeFirehose(BoundaryReports &boundary, ReportManager &rm, NGWrapper &g) {
    const ReportID r = getInternalId(rm, g);

    boundary.report_at_0_eod.insert(r);
    boundary.report_at_0.insert(r);

    // Replace the graph with a '.+'.

    clear_graph(g);
    clearReports(g);
    remove_edge(g.start, g.accept, g);
    remove_edge(g.start, g.acceptEod, g);
    remove_edge(g.startDs, g.accept, g);
    remove_edge(g.startDs, g.acceptEod, g);

    NFAVertex v = add_vertex(g);
    g[v].char_reach.setall();
    g[v].reports.insert(r);
    add_edge(v, v, g);
    add_edge(g.start, v, g);
    add_edge(g.startDs, v, g);
    add_edge(v, g.accept, g);
}

static
void makeAnchoredAcceptor(BoundaryReports &boundary, ReportManager &rm,
                          NGWrapper &g) {
    boundary.report_at_0.insert(getInternalId(rm, g));
    remove_edge(g.start, g.accept, g);
    remove_edge(g.start, g.acceptEod, g);
    g[g.start].reports.clear();
}

static
void makeEndAnchoredAcceptor(BoundaryReports &boundary, ReportManager &rm,
                             NGWrapper &g) {
    boundary.report_at_eod.insert(getInternalId(rm, g));
    remove_edge(g.startDs, g.acceptEod, g);
    remove_edge(g.start, g.acceptEod, g);
    g[g.start].reports.clear();
    g[g.startDs].reports.clear();
}

static
void makeNothingAcceptor(BoundaryReports &boundary, ReportManager &rm,
                         NGWrapper &g) {
    boundary.report_at_0_eod.insert(getInternalId(rm, g));
    remove_edge(g.start, g.acceptEod, g);
    g[g.start].reports.clear();
}

bool splitOffVacuous(BoundaryReports &boundary, ReportManager &rm,
                     NGWrapper &g) {
    if (edge(g.startDs, g.accept, g).second) {
        // e.g. '.*'; match "between" every byte
        DEBUG_PRINTF("graph is firehose\n");
        makeFirehose(boundary, rm, g);
        return true;
    }

    bool work_done = false;

    if (edge(g.start, g.accept, g).second) {
        DEBUG_PRINTF("creating anchored acceptor\n");
        makeAnchoredAcceptor(boundary, rm, g);
        work_done = true;
    }

    if (edge(g.startDs, g.acceptEod, g).second) {
        DEBUG_PRINTF("creating end-anchored acceptor\n");
        makeEndAnchoredAcceptor(boundary, rm, g);
        work_done = true;
    }

    if (edge(g.start, g.acceptEod, g).second) {
        DEBUG_PRINTF("creating nothing acceptor\n");
        makeNothingAcceptor(boundary, rm, g);
        work_done = true;
    }

    return work_done;
}

} // namespace ue2
