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

#include "nfagraph/ng_holder.h"
#include "rose/rose_build.h"
#include "rose/rose_build_impl.h"
#include "rose/rose_build_merge.h"
#include "rose/rose_build_role_aliasing.h"
#include "util/report_manager.h"
#include "util/boundary_reports.h"
#include "util/compile_context.h"
#include "util/graph_range.h"
#include "util/make_unique.h"
#include "smallwrite/smallwrite_build.h"
#include "som/slot_manager.h"

#include <memory>
#include <unordered_set>
#include <vector>

using namespace std;
using namespace ue2;

static
std::unique_ptr<NGHolder> makeSuffixGraph(ReportID report) {
    auto h = ue2::make_unique<NGHolder>(NFA_SUFFIX);
    NGHolder &g = *h;

    NFAVertex v = add_vertex(g);
    g[v].char_reach = CharReach('A', 'Z');
    g[v].reports.insert(report);
    add_edge(g.start, v, g);
    add_edge(v, g.accept, g);

    return h;
}

static
RoseVertex addVertex(RoseBuildImpl &build, RoseVertex parent, u32 lit_id) {
    RoseGraph &g = build.g;

    RoseVertex v = add_vertex(g);
    g[v].min_offset = 0;
    g[v].max_offset = ROSE_BOUND_INF;
    g[v].literals.insert(lit_id);

    RoseEdge e = add_edge(parent, v, g).first;
    g[e].minBound = 0;
    g[e].maxBound = ROSE_BOUND_INF;
    g[e].history = ROSE_ROLE_HISTORY_NONE;

    return v;
}

static
size_t numUniqueSuffixGraphs(const RoseGraph &g) {
    unordered_set<const NGHolder *> seen;

    for (const auto &v : vertices_range(g)) {
        if (g[v].suffix) {
            assert(g[v].suffix.graph);
            seen.insert(g[v].suffix.graph.get());
        }
    }
    return seen.size();
}

TEST(RoseMerge, uncalcLeaves_nonleaf) {
    Grey grey;
    CompileContext cc(true, false, get_current_target(), grey);
    ReportManager rm(cc.grey);
    SomSlotManager ssm(8); // som precision
    auto smwr = makeSmallWriteBuilder(1, rm, cc);
    BoundaryReports boundary;
    auto build_base = makeRoseBuilder(rm, ssm, *smwr, cc, boundary);
    ASSERT_NE(nullptr, build_base);

    RoseBuildImpl &build = static_cast<RoseBuildImpl &>(*build_base);
    RoseGraph &g = build.g;

    std::shared_ptr<NGHolder> shared_suffix = makeSuffixGraph(1000);

    // foo with shared_suffix
    RoseVertex v1 = addVertex(build, build.root,
        build.getLiteralId(ue2_literal("foo", false), 0, ROSE_FLOATING));
    g[v1].suffix.graph = shared_suffix;

    // foo with shared_suffix -> bar
    RoseVertex v2 = addVertex(build, build.root,
        build.getLiteralId(ue2_literal("foo", false), 0, ROSE_FLOATING));
    g[v2].suffix.graph = shared_suffix; // same as v1
    RoseVertex v3 = addVertex(build, v2,
        build.getLiteralId(ue2_literal("bar", false), 0, ROSE_FLOATING));
    g[v3].reports.insert(1001);

    // foo with a different suffix
    RoseVertex v4 = addVertex(build, build.root,
        build.getLiteralId(ue2_literal("foo", false), 0, ROSE_FLOATING));
    g[v4].suffix.graph = makeSuffixGraph(2000);

    ASSERT_EQ(6, num_vertices(g));
    ASSERT_EQ(2, numUniqueSuffixGraphs(g));

    // uncalcLeaves should leave this case alone. The suffixes on leaf nodes v1
    // and v4 are not mergeable, as v1 shares its suffix with v2.
    uncalcLeaves(build);

    ASSERT_EQ(6, num_vertices(g));
    ASSERT_EQ(2, numUniqueSuffixGraphs(g));
}
