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

#include "goughcompile_internal.h"
#include "gough_internal.h"
#include "grey.h"
#include "mcclellancompile.h"
#include "util/container.h"
#include "util/graph.h"
#include "util/graph_range.h"

#include "ue2common.h"

#include <map>
#include <vector>

using namespace std;

namespace ue2 {

template<typename Graph>
void add_edge_if_not_selfloop(const typename Graph::vertex_descriptor &u,
                              const typename Graph::vertex_descriptor &v,
                              Graph &g) {
    if (u != v) {
        add_edge(u, v, g);
    }
}

static
bool can_accel_over_selfloop(const GoughVertexProps &vp, const GoughEdge &e,
                             const GoughEdgeProps &ep, u32 *margin) {
    if (vp.vars.empty() && ep.vars.empty()) {
        /* if we update no som information, then it is trivial to accelerate */
        *margin = 0;
        return true;
    }

    /* if the effect of running a self loop stabilises after a small number of
     * iterations, it is possible to accelerate over the state and only then run
     * the block N times. To model this we create a graph which shows how the
     * value for a variable at the end of a self loop block is related to values
     * at the start */

    typedef boost::adjacency_list<boost::vecS, boost::vecS,
                                  boost::bidirectionalS> basic_graph;
    typedef basic_graph::vertex_descriptor basic_vertex;
    basic_graph bg;

    map<const GoughSSAVar *, basic_vertex> verts;

    /* create verts */
    for (const auto &var : ep.vars) {
        verts[var.get()] = add_vertex(bg);
    }

    for (const auto &var : vp.vars) {
        verts[var.get()] = add_vertex(bg);
    }

    /* wire edges */
    set<basic_vertex> done;
    for (const auto &var : ep.vars) {
        assert(contains(verts, var.get()));
        basic_vertex v = verts[var.get()];
        for (GoughSSAVar *pred : var->get_inputs()) {
            if (!contains(verts, pred)) {
                continue;
            }
            basic_vertex u = verts[pred];
            if (contains(done, u)) { /* u has already taken on new values this
                                      * iteration */
                for (auto p : inv_adjacent_vertices_range(u, bg)) {
                    add_edge_if_not_selfloop(p, v, bg);
                }
            } else {
                add_edge_if_not_selfloop(u, v, bg);
            }
        }
        done.insert(v);
    }

    for (const auto &var : vp.vars) {
        GoughSSAVar *pred = var->get_input(e);
        assert(contains(verts, var.get()));
        basic_vertex v = verts[var.get()];
        if (!contains(verts, pred)) {
            continue;
        }

        basic_vertex u = verts[pred];
        if (contains(done, u)) { /* u has already taken on new values this
                                  * iteration */
            for (auto p : inv_adjacent_vertices_range(u, bg)) {
                add_edge_if_not_selfloop(p, v, bg);
            }
        } else {
            add_edge_if_not_selfloop(u, v, bg);
        }
        /* do not add v to done as all joins happen in parallel */
    }

    /* check for loops - non self loops may prevent settling */

    if (!is_dag(bg)) {
        DEBUG_PRINTF("can not %u accel as large loops\n", vp.state_id);
        return false;
    }

    *margin = num_vertices(bg); /* TODO: be less conservative */

    if (*margin > 50) {
        return false;
    }

    return true;
}

static
bool verify_neighbour(const GoughGraph &g, GoughVertex u,
                      const map<gough_edge_id, vector<gough_ins> > &blocks,
                      const set<GoughVertex> &succs,
                      const vector<gough_ins> &block_sl) {
    for (const auto &e : out_edges_range(u, g)) {
        if (!g[e].reach.any()) { /* ignore top edges */
            continue;
        }

        GoughVertex t = target(e, g);
        if (!contains(succs, t)) { /* must be an escape string */
            continue;
        }

        if (!contains(blocks, gough_edge_id(g, e))) {
            return false;
        }

        if (blocks.at(gough_edge_id(g, e)) != block_sl) {
             return false;
        }
    }

    return true;
}

static
bool verify_neighbour_no_block(const GoughGraph &g, GoughVertex u,
                        const map<gough_edge_id, vector<gough_ins> > &blocks,
                        const set<GoughVertex> &succs) {
    for (const auto &e : out_edges_range(u, g)) {
        if (!g[e].reach.any()) { /* ignore top edges */
            continue;
        }

        GoughVertex t = target(e, g);
        if (!contains(succs, t)) { /* must be an escape string */
            continue;
        }

        if (contains(blocks, gough_edge_id(g, e))) {
            return false;
        }
    }

    return true;
}

/* Checks the som aspects of allowing two byte accel - it is expected that the
 * mcclellan logic will identify escape strings.
 *
 * For 2 byte acceleration to be correct we require that any non-escape sequence
 * characters xy from the accel state has the same effect as just the character
 * of y.
 *
 * The current way of ensuring this is to require:
 * (a) all edges out of the cyclic state behave identically to the cyclic self
 *     loop edge
 * (b) edges out of the neighbouring state which do not correspond to escape
 *     string behave identical to the cyclic state edges.
 *
 * TODO: these restrictions could be relaxed by looking at the effect on
 * relevant (live?) vars only, allowing additions to the escape string set, and
 * considering one byte escapes.
 */
static
bool allow_two_byte_accel(const GoughGraph &g,
                          const map<gough_edge_id, vector<gough_ins> > &blocks,
                          GoughVertex v, const GoughEdge &self_loop) {
    if (contains(blocks, gough_edge_id(g, self_loop))) {
        DEBUG_PRINTF("edge plan on self loop\n");
        const auto &block_sl = blocks.at(gough_edge_id(g, self_loop));

        set<GoughVertex> succs;
        for (const auto &e : out_edges_range(v, g)) {
            if (g[e].reach.none()) { /* ignore top edges */
                continue;
            }

            gough_edge_id ged(g, e);
            if (!contains(blocks, ged) || blocks.at(ged) != block_sl) {
                DEBUG_PRINTF("different out-edge behaviour\n");
                return false;
            }
            succs.insert(target(e, g));
        }

        for (auto w : adjacent_vertices_range(v, g)) {
            if (w != v && !verify_neighbour(g, w, blocks, succs, block_sl)) {
                return false;
            }
        }
    } else {
        DEBUG_PRINTF("no edge plan on self loop\n");
        set<GoughVertex> succs;
        for (const auto &e : out_edges_range(v, g)) {
            if (g[e].reach.none()) { /* ignore top edges */
                continue;
            }

            gough_edge_id ged(g, e);
            if (contains(blocks, ged)) {
                DEBUG_PRINTF("different out-edge behaviour\n");
                return false;
            }
            succs.insert(target(e, g));

            for (auto w : adjacent_vertices_range(v, g)) {
                if (w != v && !verify_neighbour_no_block(g, w, blocks, succs)) {
                    return false;
                }
            }
        }
    }

    DEBUG_PRINTF("allowing two byte accel for %u\n", g[v].state_id);
    return true;
}

void find_allowed_accel_states(const GoughGraph &g,
              const map<gough_edge_id, vector<gough_ins> > &blocks,
              map<dstate_id_t, gough_accel_state_info> *out) {
    for (auto v : vertices_range(g)) {
        GoughEdge e;
        if (!find_normal_self_loop(v, g, &e)) {
            continue; /* not accelerable */
        }
        u32 margin = 0;
        if (!can_accel_over_selfloop(g[v], e, g[e], &margin)) {
            continue; /* not accelerable */
        }
        bool tba = allow_two_byte_accel(g, blocks, v, e);
        out->emplace(g[v].state_id, gough_accel_state_info(margin, tba));
    }
}

} // namespace ue2
