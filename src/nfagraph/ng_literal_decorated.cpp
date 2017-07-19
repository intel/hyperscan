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
 * \brief Analysis for literals decorated by leading/trailing assertions or
 * character classes.
 */
#include "ng_literal_decorated.h"

#include "nfagraph/ng_holder.h"
#include "nfagraph/ng_util.h"
#include "rose/rose_build.h"
#include "rose/rose_in_graph.h"
#include "rose/rose_in_util.h"
#include "util/compile_context.h"
#include "util/dump_charclass.h"
#include "util/make_unique.h"

#include <algorithm>
#include <memory>
#include <sstream>

using namespace std;

namespace ue2 {

namespace {

/** \brief Max fixed-width paths to generate from a graph. */
static constexpr size_t MAX_PATHS = 10;

/** \brief Max degree for any non-special vertex in the graph. */
static constexpr size_t MAX_VERTEX_DEGREE = 6;

using Path = vector<NFAVertex>;

} // namespace

static
bool findPaths(const NGHolder &g, vector<Path> &paths) {
    vector<NFAVertex> order = getTopoOrdering(g);

    vector<size_t> read_count(num_vertices(g));
    vector<vector<Path>> built(num_vertices(g));

    for (auto it = order.rbegin(); it != order.rend(); ++it) {
        NFAVertex v = *it;
        auto &out = built[g[v].index];
        assert(out.empty());

        read_count[g[v].index] = out_degree(v, g);

        DEBUG_PRINTF("setting read_count to %zu for %zu\n",
                      read_count[g[v].index], g[v].index);

        if (v == g.start || v == g.startDs) {
            out.push_back({v});
            continue;
        }

        // The paths to v are the paths to v's predecessors, with v added to
        // the end of each.
        for (auto u : inv_adjacent_vertices_range(v, g)) {
            // We have a stylized connection from start -> startDs, but we
            // don't need anchored and unanchored versions of the same path.
            if (u == g.start && edge(g.startDs, v, g).second) {
                continue;
            }

            // Similarly, avoid the accept->acceptEod edge.
            if (u == g.accept) {
                assert(v == g.acceptEod);
                continue;
            }

            assert(!built[g[u].index].empty());
            assert(read_count[g[u].index]);

            for (const auto &p : built[g[u].index]) {
                out.push_back(p);
                out.back().push_back(v);

                if (out.size() > MAX_PATHS) {
                    // All these paths should eventually end up at a sink, so
                    // we've blown past our limit.
                    DEBUG_PRINTF("path limit exceeded\n");
                    return false;
                }
            }

            read_count[g[u].index]--;
            if (!read_count[g[u].index]) {
                DEBUG_PRINTF("clearing %zu as finished reading\n", g[u].index);
                built[g[u].index].clear();
                built[g[u].index].shrink_to_fit();
            }
        }
    }

    insert(&paths, paths.end(), built[NODE_ACCEPT]);
    insert(&paths, paths.end(), built[NODE_ACCEPT_EOD]);

    DEBUG_PRINTF("%zu paths generated\n", paths.size());

    return paths.size() <= MAX_PATHS;
}

static
bool hasLargeDegreeVertex(const NGHolder &g) {
    for (const auto &v : vertices_range(g)) {
        if (is_special(v, g)) { // specials can have large degree
            continue;
        }
        if (degree(v, g) > MAX_VERTEX_DEGREE) {
            DEBUG_PRINTF("vertex %zu has degree %zu\n", g[v].index,
                         degree(v, g));
            return true;
        }
    }
    return false;
}

#if defined(DEBUG) || defined(DUMP_SUPPORT)
static UNUSED
string dumpPath(const NGHolder &g, const Path &path) {
    ostringstream oss;
    for (const auto &v : path) {
        switch (g[v].index) {
        case NODE_START:
            oss << "<start>";
            break;
        case NODE_START_DOTSTAR:
            oss << "<startDs>";
            break;
        case NODE_ACCEPT:
            oss << "<accept>";
            break;
        case NODE_ACCEPT_EOD:
            oss << "<acceptEod>";
            break;
        default:
            oss << describeClass(g[v].char_reach);
            break;
        }
    }
    return oss.str();
}
#endif

struct PathMask {
    PathMask(const NGHolder &g, const Path &path)
        : is_anchored(path.front() == g.start),
          is_eod(path.back() == g.acceptEod) {
        assert(path.size() >= 2);
        mask.reserve(path.size() - 2);
        for (const auto &v : path) {
            if (is_special(v, g)) {
                continue;
            }
            mask.push_back(g[v].char_reach);
        }

        // Reports are attached to the second-to-last vertex.
        NFAVertex u = *std::next(path.rbegin());
        reports = g[u].reports;
        assert(!reports.empty());
    }

    vector<CharReach> mask;
    flat_set<ReportID> reports;
    bool is_anchored;
    bool is_eod;
};

bool handleDecoratedLiterals(RoseBuild &rose, const NGHolder &g,
                             const CompileContext &cc) {
    if (!cc.grey.allowDecoratedLiteral) {
        return false;
    }

    if (!isAcyclic(g)) {
        DEBUG_PRINTF("not acyclic\n");
        return false;
    }

    if (!hasNarrowReachVertex(g)) {
        DEBUG_PRINTF("no narrow reach vertices\n");
        return false;
    }

    if (hasLargeDegreeVertex(g)) {
        DEBUG_PRINTF("large degree\n");
        return false;
    }

    vector<Path> paths;
    if (!findPaths(g, paths)) {
        DEBUG_PRINTF("couldn't split into a small number of paths\n");
        return false;
    }

    assert(!paths.empty());
    assert(paths.size() <= MAX_PATHS);

    vector<PathMask> masks;
    masks.reserve(paths.size());

    for (const auto &path : paths) {
        DEBUG_PRINTF("path: %s\n", dumpPath(g, path).c_str());
        PathMask pm(g, path);
        if (!rose.validateMask(pm.mask, pm.reports, pm.is_anchored,
                               pm.is_eod)) {
            DEBUG_PRINTF("failed validation\n");
            return false;
        }
        masks.push_back(move(pm));
    }

    for (const auto &pm : masks) {
        rose.addMask(pm.mask, pm.reports, pm.is_anchored, pm.is_eod);
    }

    DEBUG_PRINTF("all ok, %zu masks added\n", masks.size());
    return true;
}

} // namespace ue2
