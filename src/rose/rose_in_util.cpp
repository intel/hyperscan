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

#include "rose_in_util.h"

#include "rose_build_util.h"
#include "nfa/goughcompile.h"
#include "nfagraph/ng_depth.h"
#include "nfagraph/ng_util.h"
#include "nfagraph/ng_width.h"
#include "util/container.h"
#include "util/graph_range.h"
#include "util/make_unique.h"

#include <vector>

#include <boost/graph/copy.hpp>
#include <boost/graph/reverse_graph.hpp>
#include <boost/graph/topological_sort.hpp>

using namespace std;

namespace ue2 {

/* Returns a topological ordering of the vertices in g. That is the starts are
 * at the front and all the predecessors of a vertex occur earlier in the list
 * than the vertex. */
vector<RoseInVertex> topo_order(const RoseInGraph &g) {
    assert(hasCorrectlyNumberedVertices(g));
    vector<RoseInVertex> v_order;
    v_order.reserve(num_vertices(g));

    boost::topological_sort(g, back_inserter(v_order));

    reverse(v_order.begin(), v_order.end()); /* put starts at the front */

    return v_order;
}

namespace {
struct RoseEdgeCopier {
    typedef unordered_map<const NGHolder *, shared_ptr<NGHolder>> GraphMap;
    typedef unordered_map<const raw_som_dfa *, shared_ptr<raw_som_dfa>> HaigMap;

    RoseEdgeCopier(const RoseInGraph &g1, RoseInGraph &g2,
                   const GraphMap &graph_map_in, const HaigMap &haig_map_in)
        : ig(g1), out(g2), graph_map(graph_map_in), haig_map(haig_map_in) {}

    void operator()(const RoseInEdge &e1, RoseInEdge &e2) {
        // Clone all properties.
        put(boost::edge_all, out, e2, get(boost::edge_all, ig, e1));
        // Substitute in cloned graphs.
        if (ig[e1].graph) {
            out[e2].graph = graph_map.at(ig[e1].graph.get());
        }
        if (ig[e1].haig) {
            out[e2].haig = haig_map.at(ig[e1].haig.get());
        }
    }

private:
    const RoseInGraph &ig;
    RoseInGraph &out;
    const GraphMap &graph_map;
    const HaigMap &haig_map;
};
}

unique_ptr<RoseInGraph> cloneRoseGraph(const RoseInGraph &ig) {
    assert(hasCorrectlyNumberedVertices(ig));
    unique_ptr<RoseInGraph> out = make_unique<RoseInGraph>();

    unordered_map<const NGHolder *, shared_ptr<NGHolder>> graph_map;
    unordered_map<const raw_som_dfa *, shared_ptr<raw_som_dfa>> haig_map;

    for (const auto &e : edges_range(ig)) {
        const RoseInEdgeProps &ep = ig[e];
        if (ep.graph && !contains(graph_map, ep.graph.get())) {
            graph_map[ep.graph.get()] = cloneHolder(*ep.graph);
        }
        if (ep.haig && !contains(haig_map, ep.haig.get())) {
            haig_map[ep.haig.get()] = make_shared<raw_som_dfa>(*ep.haig);
        }
    }

    copy_graph(ig, *out,
               boost::edge_copy(RoseEdgeCopier(ig, *out, graph_map, haig_map)));
    return out;
}

void calcVertexOffsets(RoseInGraph &g) {
    vector<RoseInVertex> v_order = topo_order(g);

    for (RoseInVertex v : v_order) {
        if (g[v].type == RIV_START) {
            g[v].min_offset = 0;
            g[v].max_offset = ROSE_BOUND_INF;
            continue;
        } else if (g[v].type == RIV_ANCHORED_START) {
            g[v].min_offset = 0;
            g[v].max_offset = 0;
            continue;
        }

        DEBUG_PRINTF("vertex '%s'\n", dumpString(g[v].s).c_str());

        // Min and max predecessor depths.
        u32 min_d = ROSE_BOUND_INF;
        u32 max_d = 0;

        for (const auto &e : in_edges_range(v, g)) {
            RoseInVertex u = source(e, g);
            u32 e_min = g[u].min_offset;
            u32 e_max = g[u].max_offset;

            DEBUG_PRINTF("in-edge from u with offsets [%u,%u]\n", e_min, e_max);

            if (g[e].graph) {
                const NGHolder &h = *g[e].graph;
                depth g_min_width = findMinWidth(h);
                depth g_max_width =
                    isAnchored(h) ? findMaxWidth(h) : depth::infinity();
                u32 graph_lag = g[e].graph_lag;

                DEBUG_PRINTF("edge has graph, depths [%s,%s] and lag %u\n",
                             g_min_width.str().c_str(),
                             g_max_width.str().c_str(), graph_lag);
                g_min_width += graph_lag;
                g_max_width += graph_lag;
                e_min = add_rose_depth(e_min, g_min_width);
                if (g_max_width.is_finite()) {
                    e_max = add_rose_depth(e_max, g_max_width);
                } else {
                    e_max = ROSE_BOUND_INF;
                }
            } else {
                DEBUG_PRINTF("edge has bounds [%u,%u]\n", g[e].minBound,
                             g[e].maxBound);
                e_min = add_rose_depth(e_min, g[e].minBound);
                e_max = add_rose_depth(e_max, g[e].maxBound);
                if (g[v].type == RIV_LITERAL) {
                    u32 len = g[v].s.length();
                    DEBUG_PRINTF("lit len %u\n", len);
                    e_min = add_rose_depth(e_min, len);
                    e_max = add_rose_depth(e_max, len);
                }
            }

            min_d = min(min_d, e_min);
            max_d = max(max_d, e_max);
        }

        DEBUG_PRINTF("vertex depths [%u,%u]\n", min_d, max_d);

        assert(max_d >= min_d);
        g[v].min_offset = min_d;
        g[v].max_offset = max_d;
    }

    // It's possible that we may have literal delays assigned to vertices here
    // as well. If so, these need to be added to the min/max offsets.
    for (RoseInVertex v : v_order) {
        const u32 delay = g[v].delay;
        g[v].min_offset = add_rose_depth(g[v].min_offset, delay);
        g[v].max_offset = add_rose_depth(g[v].max_offset, delay);
    }
}

nfa_kind whatRoseIsThis(const RoseInGraph &in, const RoseInEdge &e) {
    RoseInVertex u = source(e, in);
    RoseInVertex v = target(e, in);

    bool start = in[u].type == RIV_START || in[u].type == RIV_ANCHORED_START;
    bool end = in[v].type == RIV_ACCEPT || in[v].type == RIV_ACCEPT_EOD;

    if (start && !end) {
        return NFA_PREFIX;
    } else if (!start && end) {
        return NFA_SUFFIX;
    } else if (!start && !end) {
        return NFA_INFIX;
    } else {
        assert(in[v].type == RIV_ACCEPT_EOD);
        return NFA_OUTFIX;
    }
}

void pruneUseless(RoseInGraph &g) {
    DEBUG_PRINTF("pruning useless vertices\n");

    set<RoseInVertex> dead;
    RoseInVertex dummy_start
        = add_vertex(RoseInVertexProps::makeStart(true), g);
    RoseInVertex dummy_end
        = add_vertex(RoseInVertexProps::makeAccept(set<ReportID>()), g);
    dead.insert(dummy_start);
    dead.insert(dummy_end);
    for (auto v : vertices_range(g)) {
        if (v == dummy_start || v == dummy_end) {
            continue;
        }
        switch (g[v].type) {
        case RIV_ANCHORED_START:
        case RIV_START:
            add_edge(dummy_start, v, g);
            break;
        case RIV_ACCEPT:
        case RIV_ACCEPT_EOD:
            add_edge(v, dummy_end, g);
            break;
        default:
            break;
        }
    }

    find_unreachable(g, vector<RoseInVertex>(1, dummy_start), &dead);
    find_unreachable(boost::reverse_graph<RoseInGraph, RoseInGraph &>(g),
                     vector<RoseInVertex>(1, dummy_end), &dead);

    for (auto v : dead) {
        clear_vertex(v, g);
        remove_vertex(v, g);
    }
}

}
