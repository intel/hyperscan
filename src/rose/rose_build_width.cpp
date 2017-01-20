/*
 * Copyright (c) 2015-2016, Intel Corporation
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

#include "rose_build_width.h"

#include "nfagraph/ng_holder.h"
#include "nfagraph/ng_dump.h"
#include "nfagraph/ng_width.h"
#include "rose_build_impl.h"
#include "ue2common.h"
#include "util/graph.h"
#include "util/graph_range.h"

#include <algorithm>

using namespace std;

namespace ue2 {

static
bool is_end_anchored(const RoseGraph &g, RoseVertex v) {
    for (auto w : adjacent_vertices_range(v, g)) {
        if (g[w].eod_accept) {
            return true;
        }
    }

    return false;
}

u32 findMinWidth(const RoseBuildImpl &tbi, enum rose_literal_table table) {
    if (table != ROSE_FLOATING && table != ROSE_ANCHORED &&
        table != ROSE_EOD_ANCHORED) {
        /* handle other tables if ever required */
        assert(0);
        return 0;
    }

    const RoseGraph &g = tbi.g;

    vector<RoseVertex> table_verts;

    for (auto v : vertices_range(g)) {
        if (tbi.hasLiteralInTable(v, table)) {
            table_verts.push_back(v);
        }
    }

    set<RoseVertex> reachable;
    find_reachable(g, table_verts, &reachable);

    u32 minWidth = ROSE_BOUND_INF;
    for (auto v : reachable) {
        if (g[v].eod_accept) {
            DEBUG_PRINTF("skipping %zu - not a real vertex\n", g[v].index);
            continue;
        }

        const u32 w = g[v].min_offset;

        if (!g[v].reports.empty()) {
            DEBUG_PRINTF("%zu can fire report at offset %u\n", g[v].index, w);
            minWidth = min(minWidth, w);
        }

        if (is_end_anchored(g, v)) {
            DEBUG_PRINTF("%zu can fire eod report at offset %u\n", g[v].index,
                         w);
            minWidth = min(minWidth, w);
        }

        if (g[v].suffix) {
            depth suffix_width = findMinWidth(g[v].suffix, g[v].suffix.top);
            assert(suffix_width.is_reachable());
            DEBUG_PRINTF("%zu has suffix with top %u (width %s), can fire "
                         "report at %u\n",
                         g[v].index, g[v].suffix.top, suffix_width.str().c_str(),
                         w + suffix_width);
            minWidth = min(minWidth, w + suffix_width);
        }
    }

    /* TODO: take into account the chain relationship between the mpv and other
     * engines */
    DEBUG_PRINTF("min width %u\n", minWidth);
    return minWidth;
}

u32 findMaxBAWidth(const RoseBuildImpl &tbi) {
    const RoseGraph &g = tbi.g;
    if (!isLeafNode(tbi.root, g)) {
        DEBUG_PRINTF("floating literal -> no max width\n");
        return ROSE_BOUND_INF;
    }

    u64a maxWidth = 0;

    for (const auto &outfix : tbi.outfixes) {
        maxWidth = max(maxWidth, (u64a)outfix.maxBAWidth);
        if (maxWidth >= ROSE_BOUND_INF) {
            DEBUG_PRINTF("outfix with no max ba width\n");
            return ROSE_BOUND_INF;
        }
    }

    // Everyone's anchored, so the max width can be taken from the max
    // max_offset on our vertices (so long as all accepts are EOD).
    for (auto v : vertices_range(g)) {
        if (!g[v].reports.empty() && !g[v].eod_accept) {
            DEBUG_PRINTF("accept not at eod\n");
            return ROSE_BOUND_INF;
        }

        if (g[v].reports.empty() && !g[v].suffix) {
            continue;
        }

        assert(g[v].eod_accept || g[v].suffix);

        u64a w = g[v].max_offset;

        if (g[v].suffix) {
            if (has_non_eod_accepts(g[v].suffix)) {
                return ROSE_BOUND_INF;
            }
            depth suffix_width = findMaxWidth(g[v].suffix, g[v].suffix.top);
            DEBUG_PRINTF("suffix max width for top %u is %s\n", g[v].suffix.top,
                         suffix_width.str().c_str());
            assert(suffix_width.is_reachable());
            if (!suffix_width.is_finite()) {
                DEBUG_PRINTF("suffix too wide\n");
                return ROSE_BOUND_INF;
            }

            w += suffix_width;
        }

        maxWidth = max(maxWidth, w);
        if (maxWidth >= ROSE_BOUND_INF) {
            DEBUG_PRINTF("too wide\n");
            return ROSE_BOUND_INF;
        }
    }

    DEBUG_PRINTF("max ba width %llu\n", maxWidth);
    assert(maxWidth < ROSE_BOUND_INF);
    return maxWidth;
}

u32 findMaxBAWidth(const RoseBuildImpl &tbi, enum rose_literal_table table) {
    const RoseGraph &g = tbi.g;
    if (!isLeafNode(tbi.root, g) && table == ROSE_FLOATING) {
        DEBUG_PRINTF("floating literal -> no max width\n");
        return ROSE_BOUND_INF;
    }

    if (table != ROSE_FLOATING && table != ROSE_ANCHORED) {
        /* handle other tables if ever required */
        assert(0);
        return ROSE_BOUND_INF;
    }

    DEBUG_PRINTF("looking for a max ba width for %s\n",
                  table == ROSE_FLOATING ? "floating" : "anchored");

    vector<RoseVertex> table_verts;

    for (auto v : vertices_range(g)) {
        if ((table == ROSE_FLOATING && tbi.isFloating(v))
            || (table == ROSE_ANCHORED && tbi.isAnchored(v))) {
            table_verts.push_back(v);
        }
    }

    set<RoseVertex> reachable;
    find_reachable(g, table_verts, &reachable);

    u64a maxWidth = 0;
    // Everyone's anchored, so the max width can be taken from the max
    // max_offset on our vertices (so long as all accepts are ACCEPT_EOD).
    for (auto v : reachable) {
        DEBUG_PRINTF("inspecting vert %zu\n", g[v].index);

        if (g[v].eod_accept) {
            DEBUG_PRINTF("skipping %zu - not a real vertex\n", g[v].index);
            continue;
        }

        if (!g[v].reports.empty()) {
            DEBUG_PRINTF("accept not at eod\n");
            return ROSE_BOUND_INF;
        }

        u64a w = g[v].max_offset;

        u64a follow_max = tbi.calcSuccMaxBound(v); /* may have a long bound to
                                                      accept_eod node */

        if (g[v].suffix) {
            if (has_non_eod_accepts(g[v].suffix)) {
                DEBUG_PRINTF("has accept\n");
                return ROSE_BOUND_INF;
            }
            depth suffix_width = findMaxWidth(g[v].suffix);
            DEBUG_PRINTF("suffix max width %s\n", suffix_width.str().c_str());
            assert(suffix_width.is_reachable());
            if (!suffix_width.is_finite()) {
                DEBUG_PRINTF("suffix too wide\n");
                return ROSE_BOUND_INF;
            }
            follow_max = max(follow_max, (u64a)suffix_width);
        }

        w += follow_max;

        DEBUG_PRINTF("w %llu\n", w);

        maxWidth = max(maxWidth, w);
        if (maxWidth >= ROSE_BOUND_INF) {
            DEBUG_PRINTF("too wide\n");
            return ROSE_BOUND_INF;
        }
    }

    DEBUG_PRINTF("max ba width %llu\n", maxWidth);
    assert(maxWidth < ROSE_BOUND_INF);
    return maxWidth;
}

} // namespace ue2
