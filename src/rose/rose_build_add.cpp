/*
 * Copyright (c) 2015-2018, Intel Corporation
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

#include "rose_build_add_internal.h"
#include "rose_build_impl.h"

#include "ue2common.h"
#include "grey.h"
#include "rose_build_anchored.h"
#include "rose_in_util.h"
#include "hwlm/hwlm_literal.h"
#include "nfa/goughcompile.h"
#include "nfa/nfa_api_queue.h"
#include "nfagraph/ng_depth.h"
#include "nfagraph/ng_dump.h"
#include "nfagraph/ng_holder.h"
#include "nfagraph/ng_limex.h"
#include "nfagraph/ng_mcclellan.h"
#include "nfagraph/ng_prefilter.h"
#include "nfagraph/ng_prune.h"
#include "nfagraph/ng_region.h"
#include "nfagraph/ng_repeat.h"
#include "nfagraph/ng_reports.h"
#include "nfagraph/ng_util.h"
#include "nfagraph/ng_width.h"
#include "util/charreach.h"
#include "util/charreach_util.h"
#include "util/compare.h"
#include "util/compile_context.h"
#include "util/container.h"
#include "util/dump_charclass.h"
#include "util/graph_range.h"
#include "util/insertion_ordered.h"
#include "util/make_unique.h"
#include "util/noncopyable.h"
#include "util/order_check.h"
#include "util/report_manager.h"
#include "util/ue2string.h"
#include "util/verify_types.h"

#include <algorithm>
#include <map>
#include <set>
#include <string>
#include <vector>
#include <utility>

using namespace std;

namespace ue2 {

/**
 * \brief Data used by most of the construction code in this file.
 */
struct RoseBuildData : noncopyable {
    RoseBuildData(const RoseInGraph &ig_in, bool som_in)
        : ig(ig_in), som(som_in) {}

    /** Input rose graph. */
    const RoseInGraph &ig;

    /** Edges we've transformed (in \ref transformAnchoredLiteralOverlap) which
     * require ANCH history to prevent overlap. */
    unordered_set<RoseInEdge> anch_history_edges;

    /** True if we're tracking Start of Match. */
    bool som;
};

static
ReportID findReportId(const NGHolder &g) {
    /* prefix/infix always have an edge to accept and only 1 reportid initially
     */
    assert(in_degree(g.accept, g));
    const auto &rep = g[*inv_adjacent_vertices(g.accept, g).first].reports;
    assert(!rep.empty());
    return *rep.begin();
}

static
RoseVertex createVertex(RoseBuildImpl *build, u32 literalId, u32 min_offset,
                        u32 max_offset) {
    RoseGraph &g = build->g;
    // add to tree
    RoseVertex v = add_vertex(g);
    g[v].min_offset = min_offset;
    g[v].max_offset = max_offset;

    DEBUG_PRINTF("insert vertex %zu into literal %u's vertex set\n", g[v].index,
                 literalId);
    g[v].literals.insert(literalId);
    build->literal_info[literalId].vertices.insert(v);

    return v;
}

RoseVertex createVertex(RoseBuildImpl *build, const RoseVertex parent,
                        u32 minBound, u32 maxBound, u32 literalId,
                        size_t literalLength,
                        const flat_set<ReportID> &reports) {
    assert(parent != RoseGraph::null_vertex());

    RoseGraph &g = build->g;
    // add to tree (offsets set latter)
    RoseVertex v = createVertex(build, literalId, 0U, 0U);

    /* fill in report information */
    g[v].reports.insert(reports.begin(), reports.end());

    RoseEdge e = add_edge(parent, v, g);
    DEBUG_PRINTF("adding edge (%u, %u) to parent\n", minBound, maxBound);

    g[e].minBound = minBound;
    g[e].maxBound = maxBound;
    g[e].rose_top = 0;

    u32 min_offset = add_rose_depth(g[parent].min_offset, minBound);
    u32 max_offset = add_rose_depth(g[parent].max_offset, maxBound);

    /* take literal length into account for offsets */
    const u32 lit_len = verify_u32(literalLength);
    min_offset = add_rose_depth(min_offset, lit_len);
    max_offset = add_rose_depth(max_offset, lit_len);

    g[v].min_offset = min_offset;
    g[v].max_offset = max_offset;

    return v;
}

static
RoseVertex createAnchoredVertex(RoseBuildImpl *build, u32 literalId,
                                u32 min_offset, u32 max_offset) {
    RoseGraph &g = build->g;
    RoseVertex v = createVertex(build, literalId, min_offset, max_offset);

    DEBUG_PRINTF("created anchored vertex %zu with lit id %u\n", g[v].index,
                 literalId);

    RoseEdge e = add_edge(build->anchored_root, v, g);
    g[e].minBound = min_offset;
    g[e].maxBound = max_offset;

    return v;
}

static
RoseVertex duplicate(RoseBuildImpl *build, RoseVertex v) {
    RoseGraph &g = build->g;
    RoseVertex w = add_vertex(g[v], g);
    DEBUG_PRINTF("added vertex %zu\n", g[w].index);

    for (auto lit_id : g[w].literals) {
        build->literal_info[lit_id].vertices.insert(w);
    }

    for (const auto &e : in_edges_range(v, g)) {
        RoseVertex s = source(e, g);
        add_edge(s, w, g[e], g);
        DEBUG_PRINTF("added edge (%zu,%zu)\n", g[s].index, g[w].index);
    }

    return w;
}

namespace {
struct created_key {
    explicit created_key(const RoseInEdgeProps &trep)
        : prefix(trep.graph.get()), lag(trep.graph_lag) {
    }
    bool operator<(const created_key &b) const {
        const created_key &a = *this;
        ORDER_CHECK(prefix);
        ORDER_CHECK(lag);
        return false;
    }
    NGHolder *prefix;
    u32 lag;
};
}

static
bool isPureAnchored(const NGHolder &h) {
    return !proper_out_degree(h.startDs, h);
}

static
RoseRoleHistory selectHistory(const RoseBuildImpl &tbi, const RoseBuildData &bd,
                              const RoseInEdge &rose_edge, const RoseEdge &e) {
    const RoseGraph &g = tbi.g;
    const RoseVertex u = source(e, g), v = target(e, g);
    const bool fixed_offset_src = g[u].fixedOffset();
    const bool has_bounds = g[e].minBound || (g[e].maxBound != ROSE_BOUND_INF);

    DEBUG_PRINTF("edge %zu->%zu, bounds=[%u,%u], fixed_u=%d, prefix=%d\n",
                 g[u].index, g[v].index, g[e].minBound, g[e].maxBound,
                 (int)g[u].fixedOffset(), (int)g[v].left);

    if (g[v].left) {
        // Roles with prefix engines have their history handled by that prefix.
        assert(!contains(bd.anch_history_edges, rose_edge));
        return ROSE_ROLE_HISTORY_NONE;
    }

    if (contains(bd.anch_history_edges, rose_edge)) {
        DEBUG_PRINTF("needs anch history\n");
        return ROSE_ROLE_HISTORY_ANCH;
    }

    if (fixed_offset_src && has_bounds) {
        DEBUG_PRINTF("needs anch history\n");
        return ROSE_ROLE_HISTORY_ANCH;
    }

    return ROSE_ROLE_HISTORY_NONE;
}

static
bool hasSuccessorLiterals(RoseInVertex iv, const RoseInGraph &ig) {
    for (auto v : adjacent_vertices_range(iv, ig)) {
        if (ig[v].type != RIV_ACCEPT) {
            return true;
        }
    }
    return false;
}

static
void createVertices(RoseBuildImpl *tbi,
                    map<RoseInVertex, vector<RoseVertex> > &vertex_map,
                    const vector<pair<RoseVertex, RoseInEdge> > &parents,
                    RoseInVertex iv, u32 min_offset, u32 max_offset,
                    u32 literalId, u32 delay, const RoseBuildData &bd) {
    RoseGraph &g = tbi->g;

    DEBUG_PRINTF("vertex has %zu parents\n", parents.size());

    map<created_key, RoseVertex> created;

    for (const auto &pv : parents) {
        RoseVertex w;
        const RoseInEdgeProps &edge_props = bd.ig[pv.second];
        shared_ptr<NGHolder> prefix_graph = edge_props.graph;
        u32 prefix_lag = edge_props.graph_lag;

        created_key key(edge_props);

        if (!contains(created, key)) {
            assert(prefix_graph || !edge_props.haig);
            w = createVertex(tbi, literalId, min_offset, max_offset);
            created[key] = w;

            if (prefix_graph) {
                g[w].left.graph = prefix_graph;
                if (edge_props.dfa) {
                    g[w].left.dfa = edge_props.dfa;
                }
                g[w].left.haig = edge_props.haig;
                g[w].left.lag = prefix_lag;

                // The graph already has its report id allocated - find it.
                g[w].left.leftfix_report = findReportId(*prefix_graph);

                if (g[w].left.dfa || g[w].left.haig) {
                    assert(prefix_graph);
                    g[w].left.dfa_min_width = findMinWidth(*prefix_graph);
                    g[w].left.dfa_max_width = findMaxWidth(*prefix_graph);
                }
            }

            if (bd.som && !g[w].left.haig) {
                /* no prefix - som based on literal start */
                assert(!prefix_graph);
                g[w].som_adjust = tbi->literals.at(literalId).elength();
                DEBUG_PRINTF("set som_adjust to %u\n", g[w].som_adjust);
            }

            DEBUG_PRINTF("  adding new vertex index=%zu\n", tbi->g[w].index);
            vertex_map[iv].push_back(w);
        } else {
            w = created[key];
        }

        RoseVertex p = pv.first;

        RoseEdge e = add_edge(p, w, g);
        DEBUG_PRINTF("adding edge (%u,%u) to parent\n", edge_props.minBound,
                     edge_props.maxBound);
        g[e].minBound = edge_props.minBound;
        if (p != tbi->root && g[w].left.graph
            && (!tbi->isAnyStart(p) || isPureAnchored(*g[w].left.graph))) {
            depth mw = findMaxWidth(*g[w].left.graph);
            if (mw.is_infinite()) {
                g[e].maxBound = ROSE_BOUND_INF;
            } else {
                DEBUG_PRINTF("setting max to %s + %u\n", mw.str().c_str(),
                             prefix_lag);
                g[e].maxBound = prefix_lag + mw;
            }
        } else {
            g[e].maxBound = edge_props.maxBound;
        }
        g[e].rose_top = 0;
        g[e].history = selectHistory(*tbi, bd, pv.second, e);
    }

    if (delay && hasSuccessorLiterals(iv, bd.ig)) {
        // Add an undelayed "ghost" vertex for this literal.
        u32 ghostId = tbi->literal_info[literalId].undelayed_id;
        DEBUG_PRINTF("creating delay ghost vertex, id=%u\n", ghostId);
        assert(ghostId != literalId);
        assert(tbi->literals.at(ghostId).delay == 0);

        // Adjust offsets, removing delay.
        u32 ghost_min = min_offset, ghost_max = max_offset;
        assert(ghost_min < ROSE_BOUND_INF && ghost_min >= delay);
        ghost_min -= delay;
        ghost_max -= ghost_max == ROSE_BOUND_INF ? 0 : delay;

        RoseVertex g_v = createVertex(tbi, ghostId, ghost_min, ghost_max);

        for (const auto &pv : parents) {
            const RoseInEdgeProps &edge_props = bd.ig[pv.second];
            RoseEdge e = add_edge(pv.first, g_v, tbi->g);
            g[e].minBound = edge_props.minBound;
            g[e].maxBound = edge_props.maxBound;
            g[e].history = selectHistory(*tbi, bd, pv.second, e);
            DEBUG_PRINTF("parent edge has bounds [%u,%u]\n",
                         edge_props.minBound, edge_props.maxBound);
        }

        for (auto &m : created) {
            tbi->ghost[m.second] = g_v;
        }
    }
}

/* ensure the holder does not accept any paths which do not end with lit */
static
void removeFalsePaths(NGHolder &g, const ue2_literal &lit) {
    DEBUG_PRINTF("strip '%s'\n", dumpString(lit).c_str());
    set<NFAVertex> curr, next;
    curr.insert(g.accept);
    curr.insert(g.acceptEod);

    for (auto it = lit.rbegin(), ite = lit.rend(); it != ite; ++it) {
        next.clear();
        for (auto curr_v : curr) {
            DEBUG_PRINTF("handling %zu\n", g[curr_v].index);
            vector<NFAVertex> next_cand;
            insert(&next_cand, next_cand.end(),
                   inv_adjacent_vertices(curr_v, g));
            clear_in_edges(curr_v, g);
            if (curr_v == g.acceptEod) {
                add_edge(g.accept, g.acceptEod, g);
            }

            for (auto v : next_cand) {
                assert(v != g.startDs);
                if (v == g.start || v == g.startDs || v == g.accept) {
                    continue;
                }

                const CharReach &cr = g[v].char_reach;

                if (!overlaps(*it, cr)) {
                    DEBUG_PRINTF("false edge %zu\n", g[v].index);
                    continue;
                }

                NFAVertex v2 = clone_vertex(g, v);
                clone_in_edges(g, v, v2);
                add_edge(v2, curr_v, g);
                g[v2].char_reach &= *it;
                DEBUG_PRINTF("next <- %zu\n", g[v2].index);
                next.insert(v2);
            }
        }

        curr.swap(next);
    }

    pruneUseless(g);
    clearReports(g);
    assert(in_degree(g.accept, g) || in_degree(g.acceptEod, g) > 1);
    assert(allMatchStatesHaveReports(g));

    DEBUG_PRINTF("graph has %zu vertices left\n", num_vertices(g));
}

static
RoseVertex tryForAnchoredVertex(RoseBuildImpl *tbi,
                                const RoseInVertexProps &iv_info,
                                const RoseInEdgeProps &ep) {
    if (ep.graph_lag && ep.graph_lag != iv_info.s.length()) {
        DEBUG_PRINTF("bad lag %u != %zu\n", ep.graph_lag, iv_info.s.length());
        return RoseGraph::null_vertex(); /* TODO: better */
    }

    const depth anchored_max_depth(tbi->cc.grey.maxAnchoredRegion);
    depth min_width(0), max_width(0);

    if (ep.graph.get()) {
        const depth graph_lag(ep.graph_lag);
        max_width = findMaxWidth(*ep.graph) + graph_lag;
        min_width = findMinWidth(*ep.graph) + graph_lag;
        if (proper_out_degree(ep.graph->startDs, *ep.graph)) {
            max_width = depth::infinity();
        }
    }

    DEBUG_PRINTF("mw = %s; lag = %u\n", max_width.str().c_str(), ep.graph_lag);

    NGHolder h;

    if (ep.graph.get() && max_width <= anchored_max_depth) {
        cloneHolder(h, *ep.graph);

        /* add literal/dots */
        if (ep.graph_lag) {
            assert(ep.graph_lag == iv_info.s.length());
            appendLiteral(h, iv_info.s);
        } else {
            removeFalsePaths(h, iv_info.s);
        }
    } else if (!ep.graph.get() && ep.maxBound < ROSE_BOUND_INF
               && iv_info.s.length() + ep.maxBound
               <= tbi->cc.grey.maxAnchoredRegion) {
        if (ep.maxBound || ep.minBound) {
            /* TODO: handle, however these cases are not generated currently by
               ng_violet */
            return RoseGraph::null_vertex();
        }
        max_width = depth(ep.maxBound + iv_info.s.length());
        min_width = depth(ep.minBound + iv_info.s.length());
        add_edge(h.start, h.accept, h);
        appendLiteral(h, iv_info.s);
    } else {
        return RoseGraph::null_vertex();
    }

    u32 anchored_exit_id = tbi->getNewLiteralId();
    u32 remap_id = 0;
    DEBUG_PRINTF("  trying to add dfa stuff\n");
    int rv = addToAnchoredMatcher(*tbi, h, anchored_exit_id, &remap_id);

    if (rv == ANCHORED_FAIL) {
        return RoseGraph::null_vertex();
    } else if (rv == ANCHORED_REMAP) {
        anchored_exit_id = remap_id;
    } else {
        assert(rv == ANCHORED_SUCCESS);
    }

    // Store the literal itself in a side structure so that we can use it for
    // overlap calculations later. This may be obsolete when the old Rose
    // construction path (and its history selection code) goes away.
    rose_literal_id lit(iv_info.s, ROSE_ANCHORED, 0);
    tbi->anchoredLitSuffix.insert(make_pair(anchored_exit_id, lit));

    assert(min_width <= anchored_max_depth);
    assert(max_width <= anchored_max_depth);
    assert(min_width <= max_width);

    /* Note: bounds are end-to-end as anchored lits are considered
     * to have 0 length. */
    RoseVertex v = createAnchoredVertex(tbi, anchored_exit_id, min_width,
                                        max_width);
    return v;
}

static
u32 findRoseAnchorFloatingOverlap(const RoseInEdgeProps &ep,
                                  const RoseInVertexProps &succ_vp) {
    /* we need to ensure there is enough history to find the successor literal
     * when we enable its group.
     */

    if (!ep.graph.get()) {
        return 0; /* non overlapping */
    }
    depth graph_min_width = findMinWidth(*ep.graph);
    u32 min_width = ep.graph_lag + graph_min_width;
    u32 s_len = succ_vp.s.length();

    if (s_len <= min_width) {
        return 0; /* no overlap */
    }

    u32 overlap = s_len - min_width;
    DEBUG_PRINTF("found overlap of %u\n", overlap);
    return overlap;
}

static
void findRoseLiteralMask(const NGHolder &h, const u32 lag, vector<u8> &msk,
                         vector<u8> &cmp) {
    if (lag >= HWLM_MASKLEN) {
        msk.clear(); cmp.clear();
        return;
    }

    assert(in_degree(h.acceptEod, h) == 1); // no eod reports

    // Start with the set of reporter vertices for this rose.
    set<NFAVertex> curr, next;
    insert(&curr, inv_adjacent_vertices(h.accept, h));
    assert(!curr.empty());

    msk.assign(HWLM_MASKLEN, 0);
    cmp.assign(HWLM_MASKLEN, 0);
    size_t i = HWLM_MASKLEN - lag - 1;
    do {
        if (curr.empty() || contains(curr, h.start) ||
            contains(curr, h.startDs)) {
            DEBUG_PRINTF("end of the road\n");
            break;
        }

        next.clear();
        CharReach cr;
        for (auto v : curr) {
            DEBUG_PRINTF("vertex %zu, reach %s\n", h[v].index,
                         describeClass(h[v].char_reach).c_str());
            cr |= h[v].char_reach;
            insert(&next, inv_adjacent_vertices(v, h));
        }
        make_and_cmp_mask(cr, &msk[i], &cmp[i]);
        DEBUG_PRINTF("%zu: reach=%s, msk=%u, cmp=%u\n", i,
                     describeClass(cr).c_str(), msk.at(i), cmp.at(i));
        curr.swap(next);
    } while (i-- > 0);
}

static
void doRoseLiteralVertex(RoseBuildImpl *tbi, bool use_eod_table,
                         map<RoseInVertex, vector<RoseVertex> > &vertex_map,
                         const vector<pair<RoseVertex, RoseInEdge> > &parents,
                         RoseInVertex iv, const RoseBuildData &bd) {
    const RoseInGraph &ig = bd.ig;
    const RoseInVertexProps &iv_info = ig[iv];
    assert(iv_info.type == RIV_LITERAL);
    assert(!parents.empty()); /* start vertices should not be here */

    // ng_violet should have ensured that mixed-sensitivity literals are no
    // longer than the benefits max width.
    assert(iv_info.s.length() <= MAX_MASK2_WIDTH ||
           !mixed_sensitivity(iv_info.s));

    // Rose graph construction process should have given us a min_offset.
    assert(iv_info.min_offset > 0);

    if (use_eod_table) {
        goto floating;
    }

    DEBUG_PRINTF("rose find vertex\n");
    if (parents.size() == 1) {
        const RoseVertex u = parents.front().first;
        const RoseInEdgeProps &ep = ig[parents.front().second];

        if (!tbi->isAnyStart(u)) {
            goto floating;
        }

        if (!ep.graph && ep.maxBound == ROSE_BOUND_INF) {
            goto floating;
        }
        if (ep.graph && !isAnchored(*ep.graph)) {
            goto floating;
        }

        DEBUG_PRINTF("cand for anchored maxBound %u, %p (%d)\n", ep.maxBound,
                     ep.graph.get(), ep.graph ? (int)isAnchored(*ep.graph) : 3);

        /* need to check if putting iv into the anchored table would create
         * any bad_overlap relationships with its successor literals */
        for (const auto &e : out_edges_range(iv, ig)) {
            RoseInVertex t = target(e, ig);
            u32 overlap = findRoseAnchorFloatingOverlap(ig[e], ig[t]);
            DEBUG_PRINTF("found overlap of %u\n", overlap);
            if (overlap > tbi->cc.grey.maxHistoryAvailable + 1) {
                goto floating;
            }
        }

        RoseVertex v = tryForAnchoredVertex(tbi, iv_info, ep);
        if (v != RoseGraph::null_vertex()) {
            DEBUG_PRINTF("add anchored literal vertex\n");
            vertex_map[iv].push_back(v);
            return;
        }
    }

floating:
    vector<u8> msk, cmp;
    if (tbi->cc.grey.roseHamsterMasks && in_degree(iv, ig) == 1) {
        RoseInEdge e = *in_edges(iv, ig).first;
        if (ig[e].graph) {
            findRoseLiteralMask(*ig[e].graph, ig[e].graph_lag, msk, cmp);
        }
    }

    u32 delay = iv_info.delay;
    rose_literal_table table = use_eod_table ? ROSE_EOD_ANCHORED : ROSE_FLOATING;

    u32 literalId = tbi->getLiteralId(iv_info.s, msk, cmp, delay, table);

    DEBUG_PRINTF("literal=%u (len=%zu, delay=%u, offsets=[%u,%u] '%s')\n",
                 literalId, iv_info.s.length(), delay, iv_info.min_offset,
                 iv_info.max_offset, dumpString(iv_info.s).c_str());

    createVertices(tbi, vertex_map, parents, iv, iv_info.min_offset,
                   iv_info.max_offset, literalId, delay, bd);
}

static
unique_ptr<NGHolder> makeRoseEodPrefix(const NGHolder &h, RoseBuildImpl &build,
                                   map<flat_set<ReportID>, ReportID> &remap) {
    assert(generates_callbacks(h));
    assert(!in_degree(h.accept, h));
    auto gg = cloneHolder(h);
    NGHolder &g = *gg;
    g.kind = is_triggered(h) ? NFA_INFIX : NFA_PREFIX;

    // Move acceptEod edges over to accept.
    vector<NFAEdge> dead;
    for (const auto &e : in_edges_range(g.acceptEod, g)) {
        NFAVertex u = source(e, g);
        if (u == g.accept) {
            continue;
        }
        add_edge_if_not_present(u, g.accept, g);
        dead.push_back(e);

        if (!contains(remap, g[u].reports)) {
            remap[g[u].reports] = build.getNewNfaReport();
        }

        g[u].reports = { remap[g[u].reports] };
    }

    remove_edges(dead, g);
    return gg;
}

static
u32 getEodEventID(RoseBuildImpl &build) {
    // Allocate the EOD event if it hasn't been already.
    if (build.eod_event_literal_id == MO_INVALID_IDX) {
        build.eod_event_literal_id = build.getLiteralId({}, 0, ROSE_EVENT);
    }

    return build.eod_event_literal_id;
}

static
void makeEodEventLeftfix(RoseBuildImpl &build, RoseVertex u,
                         const NGHolder &h) {
    assert(!build.isInETable(u));

    RoseGraph &g = build.g;
    map<flat_set<ReportID>, ReportID> report_remap;
    shared_ptr<NGHolder> eod_leftfix
        = makeRoseEodPrefix(h, build, report_remap);

    u32 eod_event = getEodEventID(build);

    for (const auto &report_mapping : report_remap) {
        RoseVertex v = add_vertex(g);
        g[v].literals.insert(eod_event);
        build.literal_info[eod_event].vertices.insert(v);

        g[v].left.graph = eod_leftfix;
        g[v].left.leftfix_report = report_mapping.second;
        g[v].left.lag = 0;
        RoseEdge e1 = add_edge(u, v, g);
        g[e1].minBound = 0;
        g[e1].maxBound = ROSE_BOUND_INF;
        g[v].min_offset = add_rose_depth(g[u].min_offset,
                                         findMinWidth(*g[v].left.graph));
        g[v].max_offset = ROSE_BOUND_INF;

        depth max_width = findMaxWidth(*g[v].left.graph);
        if (u != build.root && max_width.is_finite()
            && (!build.isAnyStart(u) || isPureAnchored(*g[v].left.graph))) {
            g[e1].maxBound = max_width;
            g[v].max_offset = add_rose_depth(g[u].max_offset, max_width);
        }

        g[e1].history = ROSE_ROLE_HISTORY_NONE; // handled by prefix
        RoseVertex w = add_vertex(g);
        g[w].eod_accept = true;
        g[w].reports = report_mapping.first;
        g[w].min_offset = g[v].min_offset;
        g[w].max_offset = g[v].max_offset;
        RoseEdge e = add_edge(v, w, g);
        g[e].minBound = 0;
        g[e].maxBound = 0;
        /* No need to set history as the event is only delivered at the last
         * byte anyway - no need to invalidate stale entries. */
        g[e].history = ROSE_ROLE_HISTORY_NONE;
        DEBUG_PRINTF("accept eod vertex (index=%zu)\n", g[w].index);
    }
}

static
void doRoseAcceptVertex(RoseBuildImpl *tbi,
                        const vector<pair<RoseVertex, RoseInEdge> > &parents,
                        RoseInVertex iv, const RoseBuildData &bd) {
    const RoseInGraph &ig = bd.ig;
    assert(ig[iv].type == RIV_ACCEPT || ig[iv].type == RIV_ACCEPT_EOD);

    RoseGraph &g = tbi->g;

    for (const auto &pv : parents) {
        RoseVertex u = pv.first;
        const RoseInEdgeProps &edge_props = bd.ig[pv.second];

        /* We need to duplicate the parent vertices if:
         *
         * 1) It already has a suffix, etc as we are going to add the specified
         * suffix, etc to the parents and we do not want to overwrite the
         * existing information.
         *
         * 2) We are making the an EOD accept and the vertex already has other
         * out-edges - The LAST_BYTE history used for EOD accepts is
         * incompatible with normal successors. As accepts are processed last we
         * do not need to worry about other normal successors being added later.
         */
        if (g[u].suffix || !g[u].reports.empty()
            || (ig[iv].type == RIV_ACCEPT_EOD && out_degree(u, g)
                && !edge_props.graph)
            || (!isLeafNode(u, g) && !tbi->isAnyStart(u))) {
            DEBUG_PRINTF("duplicating for parent %zu\n", g[u].index);
            assert(!tbi->isAnyStart(u));
            u = duplicate(tbi, u);
            g[u].suffix.reset();
            g[u].eod_accept = false;
        }

        assert(!g[u].suffix);
        if (ig[iv].type == RIV_ACCEPT) {
            assert(!tbi->isAnyStart(u));
            if (edge_props.dfa) {
                DEBUG_PRINTF("adding early dfa suffix to i%zu\n", g[u].index);
                g[u].suffix.rdfa = edge_props.dfa;
                g[u].suffix.dfa_min_width = findMinWidth(*edge_props.graph);
                g[u].suffix.dfa_max_width = findMaxWidth(*edge_props.graph);
            } else if (edge_props.graph) {
                DEBUG_PRINTF("adding suffix to i%zu\n", g[u].index);
                g[u].suffix.graph = edge_props.graph;
                assert(g[u].suffix.graph->kind == NFA_SUFFIX);
                /* TODO: set dfa_(min|max)_width */
            } else if (edge_props.haig) {
                DEBUG_PRINTF("adding suffaig to i%zu\n", g[u].index);
                g[u].suffix.haig = edge_props.haig;
            } else {
                DEBUG_PRINTF("adding boring accept to i%zu\n", g[u].index);
                assert(!g[u].eod_accept);
                g[u].reports = ig[iv].reports;
            }
        } else {
            assert(ig[iv].type == RIV_ACCEPT_EOD);
            assert(!edge_props.haig);

            if (!edge_props.graph) {
                RoseVertex w = add_vertex(g);
                g[w].eod_accept = true;
                g[w].reports = ig[iv].reports;
                g[w].min_offset = g[u].min_offset;
                g[w].max_offset = g[u].max_offset;
                RoseEdge e = add_edge(u, w, g);
                g[e].minBound = 0;
                g[e].maxBound = 0;
                g[e].history = ROSE_ROLE_HISTORY_LAST_BYTE;
                DEBUG_PRINTF("accept eod vertex (index=%zu)\n", g[w].index);
                continue;
            }

            const NGHolder &h = *edge_props.graph;
            assert(!in_degree(h.accept, h));
            assert(generates_callbacks(h));

            if (tbi->isInETable(u)) {
                assert(h.kind == NFA_SUFFIX);
                assert(!tbi->isAnyStart(u));
                /* etable can't/shouldn't use eod event */
                DEBUG_PRINTF("adding suffix to i%zu\n", g[u].index);
                g[u].suffix.graph = edge_props.graph;
                continue;
            }

            makeEodEventLeftfix(*tbi, u, h);
        }
    }
}

static
bool suitableForEod(const RoseInGraph &ig, vector<RoseInVertex> topo,
                    u32 *max_len, const CompileContext &cc) {
    map<RoseInVertex, u32> max_depth_from_eod;
    *max_len = 0;

    reverse(topo.begin(), topo.end()); /* we want to start at accept end */

    for (auto v : topo) {
        u32 v_depth = 0;

        if (ig[v].type == RIV_ACCEPT) {
            DEBUG_PRINTF("[ACCEPT]\n");
            for (const auto &e : in_edges_range(v, ig)) {
                if (!ig[e].graph || !can_only_match_at_eod(*ig[e].graph)) {
                    DEBUG_PRINTF("floating accept\n");
                    return false;
                }
            }
        }

        switch (ig[v].type) {
        case RIV_LITERAL:
            DEBUG_PRINTF("[LITERAL]\n");
            break;
        case RIV_START:
            DEBUG_PRINTF("[START]\n");
            break;
        case RIV_ANCHORED_START:
            DEBUG_PRINTF("[ANCHOR]\n");
            break;
        case RIV_ACCEPT:
            break;
        case RIV_ACCEPT_EOD:
            DEBUG_PRINTF("[EOD]\n");
            break;
        default:
            assert(0);
            DEBUG_PRINTF("????\n");
            return false;
        }

        for (const auto &e : out_edges_range(v, ig)) {
            RoseInVertex t = target(e, ig);

            assert(contains(max_depth_from_eod, t));
            u64a max_width;

            if (ig[v].type == RIV_START || ig[v].type == RIV_ANCHORED_START) {
                /* start itself doesn't need to be in history buffer
                 * just need to make sure all succ literals are ok */
                if (ig[t].type == RIV_LITERAL) {
                    max_width = ig[t].s.length();
                } else {
                    max_width = 0;
                }
                if (ig[e].graph) {
                    depth graph_max_width = findMaxWidth(*ig[e].graph);
                    DEBUG_PRINTF("graph max width %s, lag %u\n",
                                 graph_max_width.str().c_str(),
                                 ig[e].graph_lag);
                    if (!graph_max_width.is_finite()) {
                        DEBUG_PRINTF("fail due to graph with inf max width\n");
                        return false;
                    }
                    max_width += graph_max_width;
                }
            } else if (ig[e].haig) {
                DEBUG_PRINTF("fail due to haig\n");
                return false;
            } else if (ig[e].graph) {
                depth graph_max_width = findMaxWidth(*ig[e].graph);
                DEBUG_PRINTF("graph max width %s, lag %u\n",
                             graph_max_width.str().c_str(), ig[e].graph_lag);
                if (!graph_max_width.is_finite()) {
                    DEBUG_PRINTF("fail due to graph with inf max width\n");
                    return false;
                }
                max_width = ig[e].graph_lag + graph_max_width;
            } else {
                max_width = ig[e].maxBound;
                if (ig[t].type == RIV_LITERAL) {
                    max_width += ig[t].s.length();
                }
            }

            max_width += max_depth_from_eod[t];
            if (max_width > ROSE_BOUND_INF) {
                max_width = ROSE_BOUND_INF;
            }

            DEBUG_PRINTF("max_width=%llu\n", max_width);

            ENSURE_AT_LEAST(&v_depth, (u32)max_width);
        }

        if (v_depth == ROSE_BOUND_INF
            || v_depth > cc.grey.maxHistoryAvailable) {
            DEBUG_PRINTF("not suitable for eod table %u\n", v_depth);
            return false;
        }

        max_depth_from_eod[v] = v_depth;
        ENSURE_AT_LEAST(max_len, v_depth);
    }

    DEBUG_PRINTF("to the eod table and beyond\n");
    return true;
}

static
void shift_accepts_to_end(const RoseInGraph &ig,
                          vector<RoseInVertex> &topo_order) {
    stable_partition(begin(topo_order), end(topo_order),
                     [&](RoseInVertex v){ return !is_any_accept(v, ig); });
}

static
void populateRoseGraph(RoseBuildImpl *tbi, RoseBuildData &bd) {
    const RoseInGraph &ig = bd.ig;

    /* add the pattern in to the main rose graph */
    DEBUG_PRINTF("%srose pop\n", bd.som ? "som " : "");

    /* Note: an input vertex may need to create several rose vertices. This is
     * primarily because a RoseVertex can only have 1 one leftfix */
    map<RoseInVertex, vector<RoseVertex> > vertex_map;

    vector<RoseInVertex> v_order = topo_order(ig);
    shift_accepts_to_end(ig, v_order);

    u32 eod_space_required;
    bool use_eod_table = suitableForEod(ig, v_order, &eod_space_required,
                                        tbi->cc);
    if (use_eod_table) {
        ENSURE_AT_LEAST(&tbi->ematcher_region_size, eod_space_required);
    }

    assert(ig[v_order.front()].type == RIV_START
           || ig[v_order.front()].type == RIV_ANCHORED_START);

    for (RoseInVertex iv : v_order) {
        DEBUG_PRINTF("vertex %zu\n", ig[iv].index);

        if (ig[iv].type == RIV_START) {
            DEBUG_PRINTF("is root\n");
            vertex_map[iv].push_back(tbi->root);
            continue;
        } else if (ig[iv].type == RIV_ANCHORED_START) {
            DEBUG_PRINTF("is anchored root\n");
            vertex_map[iv].push_back(tbi->anchored_root);
            continue;
        }

        vector<pair<RoseVertex, RoseInEdge> > parents;
        for (const auto &e : in_edges_range(iv, ig)) {
            RoseInVertex u = source(e, ig);
            assert(contains(vertex_map, u));
            const vector<RoseVertex> &images = vertex_map[u];

            // We should have no dupes.
            assert(set<RoseVertex>(images.begin(), images.end()).size()
                   == images.size());

            for (auto v_image : images) {
                // v_image should NOT already be in our parents list.
                assert(find_if(parents.begin(), parents.end(),
                            [&v_image](const pair<RoseVertex, RoseInEdge> &p) {
                                return p.first == v_image;
                            }) == parents.end());

                parents.emplace_back(v_image, e);

                if (tbi->isAnchored(v_image)) {
                    assert(!use_eod_table);
                    u32 overlap = findRoseAnchorFloatingOverlap(ig[e], ig[iv]);
                    assert(overlap <= tbi->cc.grey.maxHistoryAvailable + 1);
                    ENSURE_AT_LEAST(&tbi->max_rose_anchored_floating_overlap,
                                    overlap);
                }
            }
        }

        if (ig[iv].type == RIV_LITERAL) {
            DEBUG_PRINTF("LITERAL '%s'\n", dumpString(ig[iv].s).c_str());
            assert(!isLeafNode(iv, ig));
            doRoseLiteralVertex(tbi, use_eod_table, vertex_map, parents, iv,
                                bd);
        } else {
            if (ig[iv].type == RIV_ACCEPT) {
                DEBUG_PRINTF("ACCEPT\n");
            } else {
                assert(ig[iv].type == RIV_ACCEPT_EOD);
                DEBUG_PRINTF("ACCEPT_EOD\n");
            }
            assert(isLeafNode(iv, ig)); /* accepts are final */
            doRoseAcceptVertex(tbi, parents, iv, bd);
        }
    }
    DEBUG_PRINTF("done\n");
}

template<typename GraphT>
static
bool empty(const GraphT &g) {
    typename GraphT::vertex_iterator vi, ve;
    tie(vi, ve) = vertices(g);
    return vi == ve;
}

static
bool canImplementGraph(NGHolder &h, bool prefilter, const ReportManager &rm,
                       const CompileContext &cc) {
    if (isImplementableNFA(h, &rm, cc)) {
        return true;
    }

    if (prefilter && cc.grey.prefilterReductions) {
        // If we're prefiltering, we can have another go with a reduced graph.
        UNUSED size_t numBefore = num_vertices(h);
        prefilterReductions(h, cc);
        UNUSED size_t numAfter = num_vertices(h);
        DEBUG_PRINTF("reduced from %zu to %zu vertices\n", numBefore, numAfter);

        if (isImplementableNFA(h, &rm, cc)) {
            return true;
        }
    }

    DEBUG_PRINTF("unable to build engine\n");
    return false;
}

static
bool predsAreDelaySensitive(const RoseInGraph &ig, RoseInVertex v) {
    assert(in_degree(v, ig));

    for (const auto &e : in_edges_range(v, ig)) {
        if (ig[e].graph || ig[e].haig) {
            DEBUG_PRINTF("edge graph\n");
            return true;
        }
        if (ig[e].minBound || ig[e].maxBound != ROSE_BOUND_INF) {
            DEBUG_PRINTF("edge bounds\n");
            return true;
        }

        RoseInVertex u = source(e, ig);
        if (ig[u].type == RIV_START) {
            continue;
        }
        if (ig[u].type != RIV_LITERAL) {
            DEBUG_PRINTF("unsafe pred vertex\n");
            return true;
        }
        if (ig[u].delay) {
            DEBUG_PRINTF("pred has delay\n");
            return true;
        }
    }

    return false;
}

static
u32 maxAvailableDelay(const ue2_literal &pred_key, const ue2_literal &lit_key) {
    /* overly conservative if only part of the string is nocase */
    string pred = pred_key.get_string();
    string lit = lit_key.get_string();

    if (pred_key.any_nocase() || lit_key.any_nocase()) {
        upperString(pred);
        upperString(lit);
    }

    string::size_type last = pred.rfind(lit);
    if (last == string::npos) {
        return MAX_DELAY;
    }

    u32 raw = pred.size() - last - 1;
    return MIN(raw, MAX_DELAY);
}

static
u32 findMaxSafeDelay(const RoseInGraph &ig, RoseInVertex u, RoseInVertex v) {
    // First, check the overlap constraints on (u,v).
    size_t max_delay;
    if (ig[v].type == RIV_LITERAL) {
        DEBUG_PRINTF("lit->lit edge: '%s' -> '%s'\n",
                     escapeString(ig[u].s).c_str(),
                     escapeString(ig[v].s).c_str());
        max_delay = maxAvailableDelay(ig[u].s, ig[v].s);
    } else if (ig[v].type == RIV_ACCEPT) {
        DEBUG_PRINTF("lit->accept edge: '%s' -> ACCEPT\n",
                     escapeString(ig[u].s).c_str());
        max_delay = MAX_DELAY;
    } else {
        assert(0);
        return 0;
    }

    DEBUG_PRINTF("max safe delay for this edge: %zu\n", max_delay);

    // Now consider the predecessors of u.
    for (const auto &e : in_edges_range(u, ig)) {
        RoseInVertex w = source(e, ig);
        if (ig[w].type == RIV_START) {
            continue;
        }
        assert(ig[w].type == RIV_LITERAL);
        assert(ig[w].delay == 0);

        DEBUG_PRINTF("pred lit->lit edge: '%s' -> '%s'\n",
                     escapeString(ig[w].s).c_str(),
                     escapeString(ig[u].s).c_str());

        // We cannot delay the literal on u so much that a predecessor literal
        // could occur in the delayed region. For example, consider
        // 'barman.*foobar': if we allow 'foobar' to be delayed by 3, then
        // 'barman' could occur in the input string and race with 'foobar', as
        // in 'foobarman'.

        const size_t pred_len = ig[w].s.length();
        size_t overlap = maxOverlap(ig[u].s, ig[w].s, 0);
        DEBUG_PRINTF("pred_len=%zu, overlap=%zu\n", pred_len, overlap);
        assert(overlap <= pred_len);
        size_t max_lit_delay = pred_len - min(overlap + 1, pred_len);
        DEBUG_PRINTF("overlap=%zu -> max_lit_delay=%zu\n", overlap,
                     max_lit_delay);
        max_delay = min(max_delay, max_lit_delay);
    }

    DEBUG_PRINTF("max_delay=%zu\n", max_delay);
    assert(max_delay <= MAX_DELAY);
    return max_delay;
}

static
bool transformInfixToDelay(const RoseInGraph &ig, const RoseInEdge &e,
                           const CompileContext &cc, u32 *delay_out) {
    const u32 max_history =
        cc.streaming ? cc.grey.maxHistoryAvailable : ROSE_BOUND_INF;

    const RoseInVertex u = source(e, ig), v = target(e, ig);
    const u32 graph_lag = ig[e].graph_lag;

    // Clone a copy of the graph, as we need to be able to roll back this
    // operation.
    NGHolder h;
    cloneHolder(h, *ig[e].graph);

    DEBUG_PRINTF("target literal: %s\n", dumpString(ig[v].s).c_str());
    DEBUG_PRINTF("graph with %zu vertices and graph_lag %u\n", num_vertices(h),
                 graph_lag);

    assert(graph_lag <= ig[v].s.length());
    if (graph_lag < ig[v].s.length()) {
        size_t len = ig[v].s.length() - graph_lag;
        ue2_literal lit(ig[v].s.substr(0, len));
        DEBUG_PRINTF("lit2=%s\n", dumpString(lit).c_str());
        u32 delay2 = removeTrailingLiteralStates(h, lit, max_history);
        if (delay2 == MO_INVALID_IDX) {
            DEBUG_PRINTF("couldn't remove trailing literal\n");
            return false;
        }
        if (delay2 != len) {
            DEBUG_PRINTF("couldn't remove entire trailing literal\n");
            return false;
        }
    }

    PureRepeat repeat;
    if (!isPureRepeat(h, repeat)) {
        DEBUG_PRINTF("graph is not repeat\n");
        return false;
    }
    DEBUG_PRINTF("graph is %s repeat\n", repeat.bounds.str().c_str());
    if (!repeat.bounds.max.is_infinite()) {
        DEBUG_PRINTF("not inf\n");
        return false;
    }

    if (!repeat.reach.all()) {
        DEBUG_PRINTF("non-dot reach\n");
        return false;
    }

    u32 delay = ig[v].s.length() + repeat.bounds.min;
    if (delay > MAX_DELAY) {
        DEBUG_PRINTF("delay %u > MAX_DELAY\n", delay);
        return false;
    }

    if (delay + ig[u].s.length() - 1 > max_history) {
        DEBUG_PRINTF("delay too large for history\n");
        return false;
    }

    *delay_out = delay;
    return true;
}

static
void transformLiteralDelay(RoseInGraph &ig, const CompileContext &cc) {
    if (!cc.grey.roseTransformDelay) {
        return;
    }

    for (auto u : vertices_range(ig)) {
        if (ig[u].type != RIV_LITERAL) {
            continue;
        }
        if (out_degree(u, ig) != 1) {
            continue;
        }

        RoseInEdge e = *out_edges(u, ig).first;
        RoseInVertex v = target(e, ig);
        if (ig[v].type != RIV_LITERAL) {
            continue;
        }
        if (ig[e].haig) {
            continue;
        }
        if (!ig[e].graph) {
            continue;
        }

        if (predsAreDelaySensitive(ig, u)) {
            DEBUG_PRINTF("preds are delay sensitive\n");
            continue;
        }

        u32 max_delay = findMaxSafeDelay(ig, u, v);

        DEBUG_PRINTF("lit->lit edge with graph: '%s' -> '%s'\n",
                     escapeString(ig[u].s).c_str(),
                     escapeString(ig[v].s).c_str());

        u32 delay = 0;
        if (!transformInfixToDelay(ig, e, cc, &delay)) {
            continue;
        }

        if (delay > max_delay) {
            DEBUG_PRINTF("delay=%u > max_delay=%u\n", delay, max_delay);
            continue;
        }

        DEBUG_PRINTF("setting lit delay to %u and deleting graph\n", delay);
        ig[u].delay = delay;
        ig[u].min_offset = add_rose_depth(ig[u].min_offset, delay);
        ig[u].max_offset = add_rose_depth(ig[u].max_offset, delay);
        ig[e].graph_lag = 0;
        ig[e].graph.reset();
        ig[e].minBound = 0;
        ig[e].maxBound = ROSE_BOUND_INF;
    }
}

static
bool transformInfixToAnchBounds(const RoseInGraph &ig, const RoseInEdge &e,
                                const CompileContext &cc, DepthMinMax *bounds) {
    const u32 max_history = cc.streaming ? cc.grey.maxHistoryAvailable
                                         : ROSE_BOUND_INF;

    const RoseInVertex v = target(e, ig);
    const u32 graph_lag = ig[e].graph_lag;

    // Clone a copy of the graph, as we need to be able to roll back this
    // operation.
    NGHolder h;
    cloneHolder(h, *ig[e].graph);

    DEBUG_PRINTF("graph with %zu vertices and graph_lag %u\n", num_vertices(h),
                 graph_lag);

    assert(graph_lag <= ig[v].s.length());
    if (graph_lag < ig[v].s.length()) {
        size_t len = ig[v].s.length() - graph_lag;
        ue2_literal lit(ig[v].s.substr(0, len));
        DEBUG_PRINTF("lit2=%s\n", dumpString(lit).c_str());
        u32 delay2 = removeTrailingLiteralStates(h, lit, max_history);
        if (delay2 == MO_INVALID_IDX) {
            DEBUG_PRINTF("couldn't remove trailing literal\n");
            return false;
        }
        if (delay2 != len) {
            DEBUG_PRINTF("couldn't remove entire trailing literal\n");
            return false;
        }
    }

    PureRepeat repeat;
    if (!isPureRepeat(h, repeat)) {
        DEBUG_PRINTF("graph is not repeat\n");
        return false;
    }
    DEBUG_PRINTF("graph is %s repeat\n", repeat.bounds.str().c_str());
    if (!repeat.bounds.max.is_infinite()) {
        DEBUG_PRINTF("not inf\n");
        return false;
    }

    if (!repeat.reach.all()) {
        DEBUG_PRINTF("non-dot reach\n");
        return false;
    }

    *bounds = repeat.bounds;
    return true;
}

static
void transformAnchoredLiteralOverlap(RoseInGraph &ig, RoseBuildData &bd,
                                     const CompileContext &cc) {
    if (!cc.grey.roseTransformDelay) {
        return;
    }

    for (const auto &e : edges_range(ig)) {
        const RoseInVertex u = source(e, ig);
        const RoseInVertex v = target(e, ig);

        if (ig[u].type != RIV_LITERAL || ig[v].type != RIV_LITERAL) {
            continue;
        }
        if (ig[e].haig || !ig[e].graph) {
            continue;
        }

        if (ig[u].min_offset != ig[u].max_offset) {
            DEBUG_PRINTF("u not fixed depth\n");
            continue;
        }

        DEBUG_PRINTF("anch_lit->lit edge with graph: '%s' -> '%s'\n",
                     escapeString(ig[u].s).c_str(),
                     escapeString(ig[v].s).c_str());

        DepthMinMax bounds;
        if (!transformInfixToAnchBounds(ig, e, cc, &bounds)) {
            continue;
        }

        DEBUG_PRINTF("setting bounds to %s and deleting graph\n",
                     bounds.str().c_str());
        ig[e].graph_lag = 0;
        ig[e].graph.reset();
        ig[e].minBound = bounds.min;
        ig[e].maxBound = bounds.max.is_finite() ? (u32)bounds.max
                                                : ROSE_BOUND_INF;
        bd.anch_history_edges.insert(e);
    }
}

/**
 * \brief Transform small trailing dot repeat suffixes into delay on the last
 * literal.
 *
 * For example, the case /hatstand.*teakettle./s can just delay 'teakettle' +1
 * rather than having a suffix to handle the dot.
 *
 * This transformation looks for literal->accept edges and transforms them if
 * appropriate. It doesn't handle complex cases where the literal has more than
 * one successor.
 */
static
void transformSuffixDelay(RoseInGraph &ig, const CompileContext &cc) {
    if (!cc.grey.roseTransformDelay) {
        return;
    }

    const u32 max_history = cc.streaming ? cc.grey.maxHistoryAvailable
                                         : ROSE_BOUND_INF;

    set<RoseInVertex> modified_accepts; // may be dead after transform

    for (auto u : vertices_range(ig)) {
        if (ig[u].type != RIV_LITERAL) {
            continue;
        }
        if (out_degree(u, ig) != 1) {
            continue;
        }

        RoseInEdge e = *out_edges(u, ig).first;
        RoseInVertex v = target(e, ig);
        if (ig[v].type != RIV_ACCEPT) {
            continue;
        }
        if (ig[e].haig) {
            continue;
        }
        if (!ig[e].graph) {
            continue;
        }

        if (predsAreDelaySensitive(ig, u)) {
            DEBUG_PRINTF("preds are delay sensitive\n");
            continue;
        }

        DEBUG_PRINTF("lit->accept edge with graph: lit='%s'\n",
                      escapeString(ig[u].s).c_str());

        const NGHolder &h = *ig[e].graph;
        const set<ReportID> reports = all_reports(h);
        if (reports.size() != 1) {
            DEBUG_PRINTF("too many reports\n");
            continue;
        }

        PureRepeat repeat;
        if (!isPureRepeat(h, repeat)) {
            DEBUG_PRINTF("suffix graph is not repeat\n");
            continue;
        }
        DEBUG_PRINTF("suffix graph is %s repeat\n",
                     repeat.bounds.str().c_str());

        if (!repeat.reach.all()) {
            DEBUG_PRINTF("non-dot reach\n");
            continue;
        }

        if (repeat.bounds.min != repeat.bounds.max ||
            repeat.bounds.min > depth(MAX_DELAY)) {
            DEBUG_PRINTF("repeat is variable or too large\n");
            continue;
        }

        u32 max_delay = findMaxSafeDelay(ig, u, v);

        u32 delay = repeat.bounds.min;
        if (delay > max_delay) {
            DEBUG_PRINTF("delay=%u > max_delay=%u\n", delay, max_delay);
            continue;
        }

        if (delay + ig[u].s.length() - 1 > max_history) {
            DEBUG_PRINTF("delay too large for history\n");
            continue;
        }

        DEBUG_PRINTF("setting lit delay to %u and removing suffix\n", delay);
        ig[u].delay = delay;
        ig[u].min_offset = add_rose_depth(ig[u].min_offset, delay);
        ig[u].max_offset = add_rose_depth(ig[u].max_offset, delay);

        // Construct a new accept vertex for this report and remove edge e.
        // (This allows us to cope if v has more than one in-edge).
        RoseInVertex v2 =
            add_vertex(RoseInVertexProps::makeAccept(reports), ig);
        add_edge(u, v2, ig);
        remove_edge(e, ig);
        modified_accepts.insert(v);
    }

    DEBUG_PRINTF("%zu modified accepts\n", modified_accepts.size());

    for (auto v : modified_accepts) {
        if (in_degree(v, ig) == 0) {
            DEBUG_PRINTF("removing accept vertex with no preds\n");
            remove_vertex(v, ig);
        }
    }
}

#ifndef NDEBUG
static
bool validateKinds(const RoseInGraph &g) {
    for (const auto &e : edges_range(g)) {
        if (g[e].graph && g[e].graph->kind != whatRoseIsThis(g, e)) {
            return false;
        }
    }

    return true;
}
#endif

bool RoseBuildImpl::addRose(const RoseInGraph &ig, bool prefilter) {
    DEBUG_PRINTF("trying to rose\n");
    assert(validateKinds(ig));
    assert(hasCorrectlyNumberedVertices(ig));

    if (::ue2::empty(ig)) {
        assert(0);
        return false;
    }

    const unique_ptr<RoseInGraph> in_ptr = cloneRoseGraph(ig);
    RoseInGraph &in = *in_ptr;

    RoseBuildData bd(in, false);

    transformLiteralDelay(in, cc);
    transformAnchoredLiteralOverlap(in, bd, cc);
    transformSuffixDelay(in, cc);

    renumber_vertices(in);
    assert(validateKinds(in));

    insertion_ordered_map<NGHolder *, vector<RoseInEdge>> graphs;

    for (const auto &e : edges_range(in)) {
        if (!in[e].graph) {
            assert(!in[e].dfa);
            assert(!in[e].haig);
            continue; // no graph
        }

        if (in[e].haig || in[e].dfa) {
            /* Early DFAs/Haigs are always implementable (we've already built
             * the raw DFA). */
            continue;
        }

        NGHolder *h = in[e].graph.get();

        assert(isCorrectlyTopped(*h));
        graphs[h].push_back(e);
    }

    vector<RoseInEdge> graph_edges;

    for (const auto &m : graphs) {
        NGHolder *h = m.first;
        if (!canImplementGraph(*h, prefilter, rm, cc)) {
            return false;
        }
        insert(&graph_edges, graph_edges.end(), m.second);
    }

    /* we are now past the point of no return. We can start making irreversible
       changes to the rose graph, etc */

    for (const auto &e : graph_edges) {
        assert(in[e].graph);
        assert(!in[e].haig);
        NGHolder &h = *in[e].graph;
        DEBUG_PRINTF("handling %p\n", &h);
        assert(allMatchStatesHaveReports(h));

        if (!generates_callbacks(whatRoseIsThis(in, e))
            && in[target(e, in)].type != RIV_ACCEPT_EOD) {
            set_report(h, getNewNfaReport());
        }
    }

    populateRoseGraph(this, bd);

    return true;
}

bool RoseBuildImpl::addSombeRose(const RoseInGraph &ig) {
    DEBUG_PRINTF("rose is trying to consume a sombe\n");
    assert(validateKinds(ig));

    if (::ue2::empty(ig)) {
        assert(0);
        return false;
    }

    RoseBuildData bd(ig, true);

    for (const auto &e : edges_range(ig)) {
        if (!ig[e].graph) {
            continue; // no graph
        }
        DEBUG_PRINTF("handling %p\n", ig[e].graph.get());
        assert(allMatchStatesHaveReports(*ig[e].graph));
        assert(ig[e].haig);
    }

    populateRoseGraph(this, bd);

    return true;
}

bool roseCheckRose(const RoseInGraph &ig, bool prefilter,
                   const ReportManager &rm, const CompileContext &cc) {
    assert(validateKinds(ig));

    if (::ue2::empty(ig)) {
        assert(0);
        return false;
    }

    vector<NGHolder *> graphs;

    for (const auto &e : edges_range(ig)) {
        if (!ig[e].graph) {
            continue; // no graph
        }

        if (ig[e].haig) {
            // Haigs are always implementable (we've already built the raw DFA).
            continue;
        }

        graphs.push_back(ig[e].graph.get());
    }

    for (const auto &g : graphs) {
        if (!canImplementGraph(*g, prefilter, rm, cc)) {
            return false;
        }
    }

    return true;
}

void RoseBuildImpl::add(bool anchored, bool eod, const ue2_literal &lit,
                        const flat_set<ReportID> &reports) {
    assert(!reports.empty());

    if (cc.grey.floodAsPuffette && !anchored && !eod && is_flood(lit) &&
        lit.length() > 3) {
        DEBUG_PRINTF("adding as puffette\n");
        const CharReach &cr = *lit.begin();
        for (const auto &report : reports) {
            addOutfix(raw_puff(lit.length(), true, report, cr, true));
        }

        return;
    }

    RoseInGraph ig;
    RoseInVertex start = add_vertex(RoseInVertexProps::makeStart(anchored), ig);
    RoseInVertex accept = add_vertex(
                    eod ? RoseInVertexProps::makeAcceptEod(set<ReportID>())
                        : RoseInVertexProps::makeAccept(set<ReportID>()), ig);
    RoseInVertex v = add_vertex(RoseInVertexProps::makeLiteral(lit), ig);

    add_edge(start, v, RoseInEdgeProps(0U, anchored ? 0U : ROSE_BOUND_INF), ig);
    add_edge(v, accept, RoseInEdgeProps(0U, 0U), ig);

    calcVertexOffsets(ig);

    ig[accept].reports.insert(reports.begin(), reports.end());

    addRose(ig, false);
}

static
u32 findMaxBAWidth(const NGHolder &h) {
    // Must be bi-anchored: no out-edges from startDs (other than its
    // self-loop), no in-edges to accept.
    if (out_degree(h.startDs, h) > 1 || in_degree(h.accept, h)) {
        return ROSE_BOUND_INF;
    }
    depth d = findMaxWidth(h);
    assert(d.is_reachable());

    if (!d.is_finite()) {
        return ROSE_BOUND_INF;
    }
    return d;
}

static
void populateOutfixInfo(OutfixInfo &outfix, const NGHolder &h,
                        const RoseBuildImpl &tbi) {
    outfix.maxBAWidth = findMaxBAWidth(h);
    outfix.minWidth = findMinWidth(h);
    outfix.maxWidth = findMaxWidth(h);
    outfix.maxOffset = findMaxOffset(h, tbi.rm);
    populateReverseAccelerationInfo(outfix.rev_info, h);
}

static
bool addEodOutfix(RoseBuildImpl &build, const NGHolder &h) {
    map<flat_set<ReportID>, ReportID> report_remap;
    shared_ptr<NGHolder> eod_leftfix
        = makeRoseEodPrefix(h, build, report_remap);

    bool nfa_ok = isImplementableNFA(h, &build.rm, build.cc);

    /* TODO: check if early dfa is possible */

    if (!nfa_ok) {
        DEBUG_PRINTF("could not build as  NFA\n");
        return false;
    }

    u32 eod_event = getEodEventID(build);

    auto &g = build.g;
    for (const auto &report_mapping : report_remap) {
        RoseVertex v = add_vertex(g);
        g[v].literals.insert(eod_event);
        build.literal_info[eod_event].vertices.insert(v);

        g[v].left.graph = eod_leftfix;
        g[v].left.leftfix_report = report_mapping.second;
        g[v].left.lag = 0;
        RoseEdge e1 = add_edge(build.anchored_root, v, g);
        g[e1].minBound = 0;
        g[e1].maxBound = ROSE_BOUND_INF;
        g[v].min_offset = findMinWidth(*eod_leftfix);
        g[v].max_offset = ROSE_BOUND_INF;

        depth max_width = findMaxWidth(*g[v].left.graph);
        if (max_width.is_finite() && isPureAnchored(*eod_leftfix)) {
            g[e1].maxBound = max_width;
            g[v].max_offset = max_width;
        }

        g[e1].history = ROSE_ROLE_HISTORY_NONE; // handled by prefix
        RoseVertex w = add_vertex(g);
        g[w].eod_accept = true;
        g[w].reports = report_mapping.first;
        g[w].min_offset = g[v].min_offset;
        g[w].max_offset = g[v].max_offset;
        RoseEdge e = add_edge(v, w, g);
        g[e].minBound = 0;
        g[e].maxBound = 0;
        g[e].history = ROSE_ROLE_HISTORY_NONE;
        DEBUG_PRINTF("accept eod vertex (index=%zu)\n", g[w].index);
    }

    return true;
}

bool RoseBuildImpl::addOutfix(const NGHolder &h) {
    DEBUG_PRINTF("%zu vertices, %zu edges\n", num_vertices(h), num_edges(h));

    /* TODO: handle more than one report */
    if (!in_degree(h.accept, h)
        && all_reports(h).size() == 1
        && addEodOutfix(*this, h)) {
        return true;
    }

    const u32 nfa_states = isImplementableNFA(h, &rm, cc);
    if (nfa_states) {
        DEBUG_PRINTF("implementable as an NFA in %u states\n", nfa_states);
    } else {
        DEBUG_PRINTF("not implementable as an NFA\n");
    }

    bool dfa_cand = !nfa_states || nfa_states > 128 /* slow model */
                    || can_exhaust(h, rm); /* can be pruned */

    unique_ptr<raw_dfa> rdfa;

    if (!nfa_states || cc.grey.roseMcClellanOutfix == 2 ||
        (cc.grey.roseMcClellanOutfix == 1 && dfa_cand)) {
        rdfa = buildMcClellan(h, &rm, cc.grey);
    }

    if (!nfa_states && !rdfa) {
        DEBUG_PRINTF("could not build as either an NFA or a DFA\n");
        return false;
    }

    if (rdfa) {
        outfixes.push_back(OutfixInfo(move(rdfa)));
    } else {
        outfixes.push_back(OutfixInfo(cloneHolder(h)));
    }

    populateOutfixInfo(outfixes.back(), h, *this);

    return true;
}

bool RoseBuildImpl::addOutfix(const NGHolder &h, const raw_som_dfa &haig) {
    DEBUG_PRINTF("haig with %zu states\n", haig.states.size());

    outfixes.push_back(OutfixInfo(ue2::make_unique<raw_som_dfa>(haig)));
    populateOutfixInfo(outfixes.back(), h, *this);

    return true; /* failure is not yet an option */
}

bool RoseBuildImpl::addOutfix(const raw_puff &rp) {
    if (!mpv_outfix) {
        mpv_outfix = make_unique<OutfixInfo>(MpvProto());
    }

    auto *mpv = mpv_outfix->mpv();
    assert(mpv);
    mpv->puffettes.push_back(rp);

    mpv_outfix->maxBAWidth = ROSE_BOUND_INF; /* not ba */
    mpv_outfix->minWidth = min(mpv_outfix->minWidth, depth(rp.repeats));
    mpv_outfix->maxWidth = rp.unbounded
                              ? depth::infinity()
                              : max(mpv_outfix->maxWidth, depth(rp.repeats));

    if (mpv_outfix->maxOffset == ROSE_BOUND_INF || rp.unbounded) {
        mpv_outfix->maxOffset = ROSE_BOUND_INF;
    } else {
        mpv_outfix->maxOffset = MAX(mpv_outfix->maxOffset, rp.repeats);
    }

    return true; /* failure is not yet an option */
}

bool RoseBuildImpl::addChainTail(const raw_puff &rp, u32 *queue_out,
                                 u32 *event_out) {
    if (!mpv_outfix) {
        mpv_outfix = make_unique<OutfixInfo>(MpvProto());
    }

    auto *mpv = mpv_outfix->mpv();
    assert(mpv);
    mpv->triggered_puffettes.push_back(rp);

    mpv_outfix->maxBAWidth = ROSE_BOUND_INF; /* not ba */
    mpv_outfix->minWidth = min(mpv_outfix->minWidth, depth(rp.repeats));
    mpv_outfix->maxWidth = rp.unbounded
                              ? depth::infinity()
                              : max(mpv_outfix->maxWidth, depth(rp.repeats));

    mpv_outfix->maxOffset = ROSE_BOUND_INF; /* TODO: we could get information from
                                            * the caller */

    *queue_out = mpv_outfix->get_queue(qif);
    *event_out = MQE_TOP_FIRST + mpv->triggered_puffettes.size() - 1;

    return true; /* failure is not yet an option */
}

static
bool prepAcceptForAddAnchoredNFA(RoseBuildImpl &tbi, const NGHolder &w,
                                 NFAVertex u,
                                 const vector<DepthMinMax> &vertexDepths,
                                 map<u32, DepthMinMax> &depthMap,
                                 map<NFAVertex, set<u32>> &reportMap,
                                 map<ReportID, u32> &allocated_reports,
                                 flat_set<u32> &added_lit_ids) {
    const depth max_anchored_depth(tbi.cc.grey.maxAnchoredRegion);
    const size_t index = w[u].index;
    assert(index < vertexDepths.size());
    const DepthMinMax &d = vertexDepths.at(index);

    for (const auto &int_report : w[u].reports) {
        assert(int_report != MO_INVALID_IDX);

        u32 lit_id;
        if (!contains(allocated_reports, int_report)) {
            lit_id = tbi.getNewLiteralId();
            added_lit_ids.insert(lit_id);
            allocated_reports[int_report] = lit_id;
        } else {
            lit_id = allocated_reports[int_report];
        }

        reportMap[u].insert(lit_id);

        if (!contains(depthMap, lit_id)) {
            depthMap[lit_id] = d;
        } else {
            depthMap[lit_id] = unionDepthMinMax(depthMap[lit_id], d);
        }

        if (depthMap[lit_id].max > max_anchored_depth) {
            DEBUG_PRINTF("depth=%s exceeds maxAnchoredRegion=%u\n",
                         depthMap[lit_id].max.str().c_str(),
                         tbi.cc.grey.maxAnchoredRegion);
            return false;
        }
    }

    return true;
}

// Failure path for addAnchoredAcyclic: removes the literal IDs that have been
// added to support anchored NFAs. Assumes that they are a contiguous range at
// the end of the RoseBuildImpl::literal_info vector.
static
void removeAddedLiterals(RoseBuildImpl &tbi, const flat_set<u32> &lit_ids) {
    if (lit_ids.empty()) {
        return;
    }

    DEBUG_PRINTF("remove last %zu literals\n", lit_ids.size());

    // lit_ids should be a contiguous range.
    assert(lit_ids.size() == *lit_ids.rbegin() - *lit_ids.begin() + 1);
    assert(*lit_ids.rbegin() == tbi.literals.size() - 1);

    assert(all_of_in(lit_ids, [&](u32 lit_id) {
        return lit_id < tbi.literal_info.size() &&
               tbi.literals.at(lit_id).table == ROSE_ANCHORED &&
               tbi.literal_info[lit_id].vertices.empty();
    }));

    tbi.literals.erase_back(lit_ids.size());
    assert(tbi.literals.size() == *lit_ids.begin());

    // lit_ids should be at the end of tbi.literal_info.
    assert(tbi.literal_info.size() == *lit_ids.rbegin() + 1);
    tbi.literal_info.resize(*lit_ids.begin()); // remove all ids in lit_ids
}

bool RoseBuildImpl::addAnchoredAcyclic(const NGHolder &h) {
    auto vertexDepths = calcDepthsFrom(h, h.start);

    map<NFAVertex, set<u32> > reportMap;  /* NFAVertex -> literal ids */
    map<u32, DepthMinMax> depthMap;       /* literal id -> min/max depth */
    map<ReportID, u32> allocated_reports; /* report -> literal id */
    flat_set<u32> added_lit_ids;          /* literal ids added for this NFA */

    for (auto v : inv_adjacent_vertices_range(h.accept, h)) {
        if (!prepAcceptForAddAnchoredNFA(*this, h, v, vertexDepths, depthMap,
                                         reportMap, allocated_reports,
                                         added_lit_ids)) {
            removeAddedLiterals(*this, added_lit_ids);
            return false;
        }
    }

    map<ReportID, u32> allocated_reports_eod; /* report -> literal id */

    for (auto v : inv_adjacent_vertices_range(h.acceptEod, h)) {
        if (v == h.accept) {
            continue;
        }
        if (!prepAcceptForAddAnchoredNFA(*this, h, v, vertexDepths, depthMap,
                                         reportMap, allocated_reports_eod,
                                         added_lit_ids)) {
            removeAddedLiterals(*this, added_lit_ids);
            return false;
        }
    }

    assert(!reportMap.empty());

    int rv = addAnchoredNFA(*this, h, reportMap);
    if (rv != ANCHORED_FAIL) {
        assert(rv != ANCHORED_REMAP);
        DEBUG_PRINTF("added anchored nfa\n");
        /* add edges to the rose graph to bubble the match up */
        for (const auto &m : allocated_reports) {
            const ReportID &report = m.first;
            const u32 &lit_id = m.second;
            assert(depthMap[lit_id].max.is_finite());
            u32 minBound = depthMap[lit_id].min;
            u32 maxBound = depthMap[lit_id].max;
            RoseVertex v
                = createAnchoredVertex(this, lit_id, minBound, maxBound);
            g[v].reports.insert(report);
        }

        for (const auto &m : allocated_reports_eod) {
            const ReportID &report = m.first;
            const u32 &lit_id = m.second;
            assert(depthMap[lit_id].max.is_finite());
            u32 minBound = depthMap[lit_id].min;
            u32 maxBound = depthMap[lit_id].max;
            RoseVertex v
                = createAnchoredVertex(this, lit_id, minBound, maxBound);
            RoseVertex eod = add_vertex(g);
            g[eod].eod_accept = true;
            g[eod].reports.insert(report);
            g[eod].min_offset = g[v].min_offset;
            g[eod].max_offset = g[v].max_offset;
            add_edge(v, eod, g);
        }

        return true;
    } else {
        DEBUG_PRINTF("failed to add anchored nfa\n");
        removeAddedLiterals(*this, added_lit_ids);
        return false;
    }
}

} // namespace ue2
