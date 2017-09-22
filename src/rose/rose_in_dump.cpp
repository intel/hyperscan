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

#include "rose_in_dump.h"

#include "grey.h"
#include "ue2common.h"
#include "nfagraph/ng_dump.h"
#include "nfagraph/ng_util.h"
#include "util/container.h"
#include "util/dump_util.h"
#include "util/graph_range.h"

#include <cstdio>
#include <map>
#include <sstream>

#ifndef DUMP_SUPPORT
#error No dump support!
#endif

using namespace std;

namespace ue2 {

void dumpPreRoseGraph(const RoseInGraph &ig, const Grey &grey,
                      const char *filename) {
    if (!(grey.dumpFlags & Grey::DUMP_INT_GRAPH)) {
        return;
    }

    if (!filename) {
        filename = "pre_rose.dot";
    }
    DEBUG_PRINTF("dumping rose graphs\n");
    StdioFile f(grey.dumpPath + filename, "w");
    fprintf(f, "digraph NFA {\n");
    fprintf(f, "rankdir=LR;\n");
    fprintf(f, "size=\"11.5,8\"\n");
    fprintf(f, "node [ shape = circle ];\n");

    u32 next_id = 0;
    map<RoseInVertex, u32> i_map;
    for (auto v : vertices_range(ig)) {
        u32 id = next_id++;
        i_map[v] = id;
        const RoseInVertexProps &vp = ig[v];
        fprintf(f, "%u [ width = 1, fontsize = 12, label = \"%u:", id, id);
        switch(vp.type) {
        case RIV_LITERAL:
            fprintf(f, "%s", dotEscapeString(dumpString(vp.s)).c_str());
            break;
        case RIV_START:
            fprintf(f, "[START]");
            break;
        case RIV_ANCHORED_START:
            fprintf(f, "[ANCHOR]");
            break;
        case RIV_ACCEPT:
            if (!vp.reports.empty()) {
                fprintf(f, "[ACCEPT %s]", as_string_list(vp.reports).c_str());
            } else {
                fprintf(f, "[ACCEPT]");
            }
            break;
        case RIV_ACCEPT_EOD:
            fprintf(f, "[EOD %s]", as_string_list(vp.reports).c_str());
            break;
        }
        fprintf(f, "\" ]; \n");
    }

    map<NGHolder *, size_t> graph_ids;

    for (const auto &e : edges_range(ig)) {
        u32 u = i_map[source(e, ig)];
        u32 v = i_map[target(e, ig)];
        fprintf(f, "%u -> %u [label=\"", u, v);
        if (ig[e].graph) {
            if (!contains(graph_ids, &*ig[e].graph)) {
                size_t id = graph_ids.size();
                graph_ids[&*ig[e].graph] = id;
            }
            fprintf(f, "graph %zu\n%s", graph_ids[&*ig[e].graph],
                    to_string(ig[e].graph->kind).c_str());
        }
        if (ig[e].haig) {
            fprintf(f, "haig ");
        }
        fprintf(f, "\"]\n");
    }

    for (const auto &e : graph_ids) {
        NGHolder *h = e.first;
        size_t id = e.second;

        ostringstream name;
        name << grey.dumpPath << "pre_rose_" << id << ".dot";
        dumpGraph(name.str().c_str(), *h);
        assert(allMatchStatesHaveReports(*h));
    }

    fprintf(f, "}\n");
}

}
