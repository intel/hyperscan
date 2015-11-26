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
 * \brief Rose construction from NGHolder.
 */

// #define DEBUG
// #define DEBUG_ROSE
#include "ng_rose.h"

#include "grey.h"
#include "ng_depth.h"
#include "ng_dominators.h"
#include "ng_equivalence.h"
#include "ng_holder.h"
#include "ng_is_equal.h"
#include "ng_literal_analysis.h"
#include "ng_netflow.h"
#include "ng_prune.h"
#include "ng_redundancy.h"
#include "ng_region.h"
#include "ng_reports.h"
#include "ng_split.h"
#include "ng_util.h"
#include "ng_width.h"
#include "rose/rose_build.h"
#include "rose/rose_build_util.h"
#include "rose/rose_in_dump.h"
#include "rose/rose_in_graph.h"
#include "rose/rose_in_util.h"
#include "util/compare.h"
#include "util/compile_context.h"
#include "util/container.h"
#include "util/graph.h"
#include "util/graph_range.h"
#include "util/make_unique.h"
#include "util/order_check.h"
#include "util/ue2string.h"
#include "util/ue2_containers.h"

#include <set>
#include <utility>
#include <vector>
#include <boost/core/noncopyable.hpp>
#include <boost/graph/reverse_graph.hpp>
#include <boost/graph/topological_sort.hpp>

#define NDEBUG_PRINTF(x, ...) \
    do { if (0) { DEBUG_PRINTF(x,  ## __VA_ARGS__); } } while (0)

using namespace std;

namespace ue2 {

/**
 * Maps vertices in the original graph to vertices on edge graphs. Each edge
 * graph should contain at most one copy of the vertex. Multiple images for a
 * vertex arise after we split on multiple literals - in this cases all edges
 * should share a common graph.
 *
 * If, when an edge is split, a vertex ends up in both the LHS and RHS then only
 * the LHS is tracked. This is because in general we want to simplify the LHS
 * and allow complexity to be pushed further back.
 */
typedef ue2::unordered_map<NFAVertex, vector<pair<RoseInEdge, NFAVertex> > >
    vdest_map_t;

typedef ue2::unordered_map<RoseInEdge, vector<NFAVertex> > vsrc_map_t;

/**
 * \brief Maximum width of the character class usable as an escape class.
 */
static const u32 MAX_ESCAPE_CHARS = 20;

static
u32 maxDelay(const CompileContext &cc) {
    if (!cc.streaming) {
        return MO_INVALID_IDX;
    }
    return cc.grey.maxHistoryAvailable;
}

static
bool createsAnchoredLHS(const NGHolder &g, const vector<NFAVertex> &vv,
                        const vector<NFAVertexDepth> &depths,
                        const Grey &grey, depth max_depth = depth::infinity()) {
    max_depth = min(max_depth, depth(grey.maxAnchoredRegion));

    for (auto v : vv) {
        /* avoid issues of self loops blowing out depths:
         *     look at preds, add 1 */
        for (auto u : inv_adjacent_vertices_range(v, g)) {
            if (u == v) {
                continue;
            }

            u32 idx = g[u].index;
            assert(idx < depths.size());
            if (maxDistFromStartOfData(depths.at(idx)) >= max_depth) {
                return false;
            }
        }
    }
    return true;
}

static
bool createsTransientLHS(const NGHolder &g, const vector<NFAVertex> &vv,
                         const vector<NFAVertexDepth> &depths,
                         const Grey &grey) {
    const depth max_depth(grey.maxHistoryAvailable);

    for (auto v : vv) {
        /* avoid issues of self loops blowing out depths:
         *     look at preds, add 1 */
        for (auto u : inv_adjacent_vertices_range(v, g)) {
            if (u == v) {
                continue;
            }

            u32 idx = g[u].index;
            assert(idx < depths.size());
            if (maxDistFromInit(depths.at(idx)) >= max_depth) {
                return false;
            }
        }
    }
    return true;
}

static
bool isLHSUsablyAnchored(const NGHolder &g,
                         const vector<NFAVertexDepth> &depths,
                         const Grey &grey) {
    assert(in_degree(g.acceptEod, g) == 1);

    vector<NFAVertex> accepts;
    insert(&accepts, accepts.end(), inv_adjacent_vertices(g.accept, g));

    bool rv = createsAnchoredLHS(g, accepts, depths, grey);
    DEBUG_PRINTF("lhs is %susably anchored\n", rv ? "" : "not ");
    return rv;
}

static
bool isLHSTransient(const NGHolder &g,
                    const vector<NFAVertexDepth> &depths,
                    const Grey &grey) {
    assert(in_degree(g.acceptEod, g) == 1);

    vector<NFAVertex> accepts;
    insert(&accepts, accepts.end(), inv_adjacent_vertices(g.accept, g));

    bool rv = createsTransientLHS(g, accepts, depths, grey);
    DEBUG_PRINTF("lhs is %stransient\n", rv ? "" : "not ");
    return rv;
}

namespace {

/**
 * Information on a cut: vertices and literals.
 */
struct VertLitInfo {
    VertLitInfo(NFAVertex v, const set<ue2_literal> &litlit)
        : vv(vector<NFAVertex>(1, v)), lit(litlit) {}
    VertLitInfo(const vector<NFAVertex> &vvvv, const set<ue2_literal> &litlit)
        : vv(vvvv), lit(litlit) {}
    vector<NFAVertex> vv;
    set<ue2_literal> lit;
};

/**
 * A factory for candidate simple cuts (literals/vertices).
 */
class LitCollection : boost::noncopyable {
    vector<unique_ptr<VertLitInfo>> lits; /**< sorted list of potential cuts */
    const NGHolder &g; /**< graph on which cuts are found */
    const vector<NFAVertexDepth> &depths; /**< depth information for g */
    const ue2::unordered_map<NFAVertex, u32> &region_map; /**< region map for g */

    /** Set of vertices to avoid selecting as end vertices for cuts as previous
     * cuts overlap them. This is solely to prevent us picking literal sets
     * which do not add significant value. */
    ue2::unordered_set<NFAVertex> poisoned;

    /** Back-edges in g. */
    ue2::unordered_map<NFAVertex, vector<NFAVertex> > back_edges;

    const Grey &grey;
    bool seeking_transient;
    bool seeking_anchored;

    void poisonLHS(const VertLitInfo &picked);
    void poisonLitVerts(const VertLitInfo &picked);
    void poisonCandidates(const VertLitInfo &picked);

    friend class LitComparator;

public:
    LitCollection(const NGHolder &g_in, const vector<NFAVertexDepth> &depths_in,
                  const ue2::unordered_map<NFAVertex, u32> &region_map_in,
                  const set<NFAVertex> &ap, const set<NFAVertex> &ap_raw,
                  u32 min_len, bool desperation, const CompileContext &cc,
                  bool override_literal_quality_check = false);

    /**< Returns the next candidate cut. Cut still needs to be inspected for
     * complete envelopment. */
    unique_ptr<VertLitInfo> pickNext(void);
};

/**
 * \brief Comparator class for sorting LitCollection::lits.
 *
 * This is separated out from LitCollection itself as passing LitCollection to
 * std::sort() would incur a (potentially expensive) copy.
 */
class LitComparator {
public:
    explicit LitComparator(const LitCollection &lc_in) : lc(lc_in) {}
    bool operator()(const unique_ptr<VertLitInfo> &a,
                    const unique_ptr<VertLitInfo> &b) const {
        assert(a && b);

        if (lc.seeking_anchored) {
            bool a_anchored =
                createsAnchoredLHS(lc.g, a->vv, lc.depths, lc.grey);
            bool b_anchored =
                createsAnchoredLHS(lc.g, b->vv, lc.depths, lc.grey);

            if (a_anchored != b_anchored) {
                return a_anchored < b_anchored;
            }
        }

        if (lc.seeking_transient) {
            bool a_transient =
                createsTransientLHS(lc.g, a->vv, lc.depths, lc.grey);
            bool b_transient =
                createsTransientLHS(lc.g, b->vv, lc.depths, lc.grey);

            if (a_transient != b_transient) {
                return a_transient < b_transient;
            }
        }

        u64a score_a = scoreSet(a->lit);
        u64a score_b = scoreSet(b->lit);

        if (score_a != score_b) {
            return score_a > score_b;
        }

        /* vertices should only be in one candidate cut */
        assert(a->vv == b->vv || a->vv.front() != b->vv.front());
        return lc.g[a->vv.front()].index >
               lc.g[b->vv.front()].index;
    }

private:
    const LitCollection &lc;
};

static
size_t shorter_than(const set<ue2_literal> &s, size_t limit) {
    size_t count = 0;

    for (const auto &lit : s) {
        if (lit.length() < limit) {
            count++;
        }
    }

    return count;
}

static
u32 min_len(const set<ue2_literal> &s) {
    u32 rv = ~0U;

    for (const auto &lit : s) {
        rv = min(rv, (u32)lit.length());
    }

    return rv;
}

static
u32 max_len(const set<ue2_literal> &s) {
    u32 rv = 0;

    for (const auto &lit : s) {
        rv = max(rv, (u32)lit.length());
    }

    return rv;
}

static
u32 min_period(const set<ue2_literal> &s) {
    u32 rv = ~0U;

    for (const auto &lit : s) {
        rv = min(rv, (u32)minStringPeriod(lit));
    }
    DEBUG_PRINTF("min period %u\n", rv);
    return rv;
}

static
bool validateRoseLiteralSetQuality(const set<ue2_literal> &s, u64a score,
                                   u32 min_allowed_len, bool desperation,
                                   bool override_literal_quality_check) {
    if (!override_literal_quality_check && score >= NO_LITERAL_AT_EDGE_SCORE) {
        DEBUG_PRINTF("candidate is too bad %llu/%zu\n", score, s.size());
        return false;
    }

    assert(!s.empty());
    if (s.empty()) {
        DEBUG_PRINTF("candidate is too bad/something went wrong\n");
        return false;
    }

    u32 s_min_len = min_len(s);
    u32 s_min_period = min_period(s);
    size_t short_count = shorter_than(s, 5);

    DEBUG_PRINTF("cand '%s': score %llu count=%zu min_len=%u min_period=%u"
                 " short_count=%zu desp=%d\n",
                 dumpString(*s.begin()).c_str(), score, s.size(), s_min_len,
                 s_min_period, short_count, (int)desperation);

    bool ok = true;

    if (s.size() > 10 /* magic number is magic */
        || s_min_len < min_allowed_len
        || (s_min_period <= 1 && !override_literal_quality_check
            && min_allowed_len != 1)) {
        ok = false;
    }

    if (!ok && desperation
        && s.size() <= 20 /* more magic numbers are magical */
        && (s_min_len > 5 || (s_min_len > 2 && short_count <= 10))
        && s_min_period > 1) {
        DEBUG_PRINTF("candidate is ok\n");
        ok = true;
    }

    if (!ok && desperation
        && s.size() <= 50 /* more magic numbers are magical */
        && s_min_len > 10
        && s_min_period > 1) {
        DEBUG_PRINTF("candidate is ok\n");
        ok = true;
    }

    if (!ok) {
        DEBUG_PRINTF("candidate is too bad\n");
        return false;
    }

    return true;
}

static UNUSED
void dumpRoseLiteralSet(const set<ue2_literal> &s) {
    for (UNUSED const auto &lit : s) {
        DEBUG_PRINTF("    lit: %s\n", dumpString(lit).c_str());
    }
}

static
void getSimpleRoseLiterals(const NGHolder &g, const set<NFAVertex> &a_dom,
                           vector<unique_ptr<VertLitInfo>> *lits,
                           u32 min_allowed_len, bool desperation,
                           bool override_literal_quality_check) {
    map<NFAVertex, u64a> scores;
    map<NFAVertex, unique_ptr<VertLitInfo>> lit_info;
    set<ue2_literal> s;

    for (auto v : a_dom) {
        s = getLiteralSet(g, v, true); /* RHS will take responsibility for any
                                          revisits to the target vertex */

        if (s.empty()) {
            DEBUG_PRINTF("candidate is too bad\n");
            continue;
        }

        DEBUG_PRINTF("|candidate raw literal set| = %zu\n", s.size());
        dumpRoseLiteralSet(s);
        u64a score = compressAndScore(s);

        if (!validateRoseLiteralSetQuality(s, score, min_allowed_len,
                                           desperation,
                                           override_literal_quality_check)) {
            continue;
        }

        DEBUG_PRINTF("candidate is a candidate\n");
        scores[v] = score;
        lit_info.insert(make_pair(v, ue2::make_unique<VertLitInfo>(v, s)));
    }

    /* try to filter out cases where appending some characters produces worse
     * literals. Only bother to look back one byte, TODO make better */
    for (auto u : a_dom) {
        if (out_degree(u, g) != 1 || !scores[u]) {
            continue;
        }
        NFAVertex v = *adjacent_vertices(u, g).first;
        if (contains(scores, v) && scores[v] >= scores[u]) {
            DEBUG_PRINTF("killing off v as score %llu >= %llu\n",
                         scores[v], scores[u]);
            lit_info.erase(v);
        }
    }

    lits->reserve(lit_info.size());
    for (auto &m : lit_info) {
        lits->push_back(move(m.second));
    }
    DEBUG_PRINTF("%zu candidate literal sets\n", lits->size());
}

static
void getRegionRoseLiterals(const NGHolder &g,
                           const ue2::unordered_map<NFAVertex, u32> &region_map,
                           const set<NFAVertex> &a_dom_raw,
                           vector<unique_ptr<VertLitInfo>> *lits,
                           u32 min_allowed_len, bool desperation,
                           bool override_literal_quality_check) {
    /* This allows us to get more places to chop the graph as we are not limited
       to points where there is a single vertex to split. */

    /* TODO: operate over 'proto-regions' which ignore back edges */

    set<u32> mand, optional;
    map<u32, vector<NFAVertex> > exits;

    for (auto v : vertices_range(g)) {
        assert(contains(region_map, v));
        const u32 region = region_map.at(v);

        if (is_any_start(v, g) || region == 0) {
            continue;
        }

        if (is_any_accept(v, g)) {
            continue;
        }

        if (isRegionExit(g, v, region_map)) {
            exits[region].push_back(v);
        }

        if (isRegionEntry(g, v, region_map)) {
            // Determine whether this region is mandatory or optional. We only
            // need to do this check for the first entry vertex we encounter
            // for this region.
            if (!contains(mand, region) && !contains(optional, region)) {
                if (isOptionalRegion(g, v, region_map)) {
                    optional.insert(region);
                } else {
                    mand.insert(region);
                }
            }
        }
    }

    for (const auto &m : exits) {
        if (0) {
        next_cand:
            continue;
        }

        const u32 region = m.first;
        const vector<NFAVertex> &vv = m.second;
        assert(!vv.empty());

        if (!contains(mand, region)) {
            continue;
        }

        for (auto v : vv) {
             /* if an exit is in a_dom_raw, the region is already handled well
              * by getSimpleRoseLiterals */
            if (contains(a_dom_raw, v)) {
                goto next_cand;
            }
        }

        /* the final region may not have a neat exit. validate that all exits
         * have an edge to each accept or none do */
        bool edge_to_a = edge(vv[0], g.accept, g).second;
        bool edge_to_aeod = edge(vv[0], g.acceptEod, g).second;
        const auto &reports = g[vv[0]].reports;
        for (auto v : vv) {
            if (edge_to_a != edge(v, g.accept, g).second) {
                goto next_cand;
            }

            if (edge_to_aeod != edge(v, g.acceptEod, g).second) {
                goto next_cand;
            }

            if (g[v].reports != reports) {
                goto next_cand;
            }
        }

        DEBUG_PRINTF("inspecting region %u\n", region);
        set<ue2_literal> s;
        for (auto v : vv) {
            DEBUG_PRINTF("   exit vertex: %u\n", g[v].index);
            /* Note: RHS can not be depended on to take all subsequent revisits
             * to this vertex */
            set<ue2_literal> ss = getLiteralSet(g, v, false);
            if (ss.empty()) {
                DEBUG_PRINTF("candidate is too bad\n");
                goto next_cand;
            }
            insert(&s, ss);
        }

        assert(!s.empty());

        DEBUG_PRINTF("|candidate raw literal set| = %zu\n", s.size());
        dumpRoseLiteralSet(s);
        u64a score = compressAndScore(s);
        DEBUG_PRINTF("|candidate literal set| = %zu\n", s.size());
        dumpRoseLiteralSet(s);

        if (!validateRoseLiteralSetQuality(s, score, min_allowed_len,
                                           desperation,
                                           override_literal_quality_check)) {
            continue;
        }

        DEBUG_PRINTF("candidate is a candidate\n");
        lits->push_back(ue2::make_unique<VertLitInfo>(vv, s));
    }
}

static
void gatherBackEdges(const NGHolder &g,
                     ue2::unordered_map<NFAVertex, vector<NFAVertex>> *out) {
    set<NFAEdge> backEdges;
    BackEdges<set<NFAEdge>> be(backEdges);
    depth_first_search(g.g, visitor(be).root_vertex(g.start).vertex_index_map(
                                get(&NFAGraphVertexProps::index, g.g)));

    for (const auto &e : backEdges) {
        (*out)[source(e, g)].push_back(target(e, g));
    }
}

LitCollection::LitCollection(const NGHolder &g_in,
                        const vector<NFAVertexDepth> &depths_in,
                        const ue2::unordered_map<NFAVertex, u32> &region_map_in,
                        const set<NFAVertex> &a_dom,
                        const set<NFAVertex> &a_dom_raw, u32 min_len,
                        bool desperation, const CompileContext &cc,
                        bool override_literal_quality_check)
    : g(g_in), depths(depths_in), region_map(region_map_in), grey(cc.grey),
      seeking_transient(cc.streaming), seeking_anchored(true) {
    getSimpleRoseLiterals(g, a_dom, &lits, min_len, desperation,
                          override_literal_quality_check);
    getRegionRoseLiterals(g, region_map, a_dom_raw, &lits, min_len, desperation,
                          override_literal_quality_check);
    DEBUG_PRINTF("lit coll is looking for a%d t%d\n", (int)seeking_anchored,
                 (int)seeking_transient);
    DEBUG_PRINTF("we have %zu candidate literal splits\n", lits.size());
    sort(lits.begin(), lits.end(), LitComparator(*this));
    gatherBackEdges(g, &back_edges);
}

void LitCollection::poisonLHS(const VertLitInfo &picked) {
        DEBUG_PRINTF("found anchored %d transient %d\n",
                     (int)createsAnchoredLHS(g, picked.vv, depths, grey),
                     (int)createsTransientLHS(g, picked.vv, depths, grey));
    set<NFAVertex> curr;
    set<NFAVertex> next;

    insert(&curr, picked.vv);

    while (!curr.empty()) {
        insert(&poisoned, curr);
        next.clear();
        for (auto v : curr) {
            for (auto u : inv_adjacent_vertices_range(v, g)) {
                if (!is_special(u, g) && !contains(poisoned, u)) {
                    next.insert(u);
                }
            }
        }

        curr.swap(next);
    }

    seeking_transient = false;
    seeking_anchored = false;

    /* reprioritise cuts now that the LHS is taken care off */
    sort(lits.begin(), lits.end(), LitComparator(*this));
}

static
void flood_back(const NGHolder &g, u32 len, const set<NFAVertex> &initial,
                set<NFAVertex> *visited) {
    vector<NFAVertex> curr;
    vector<NFAVertex> next;

    insert(&curr, curr.end(), initial);

    insert(visited, initial);

    /* bfs: flood back len vertices */
    for (u32 i = 1; i < len; i++) {
        next.clear();
        DEBUG_PRINTF("poison %u/%u: curr %zu\n", i, len, curr.size());

        for (auto v : curr) {
            for (auto u : inv_adjacent_vertices_range(v, g)) {
                if (!contains(*visited, u)) {
                    next.push_back(u);
                    visited->insert(u);
                }
            }
        }

        next.swap(curr);
    }
}

/**
 * Add vertices near a picked literal to the poison set unless it looks
 * like they may still add value (ie they are on they other side of cycle).
 */
void LitCollection::poisonLitVerts(const VertLitInfo &picked) {
    DEBUG_PRINTF("poisoning vertices associated with picked literals\n");

    u32 len = max_len(picked.lit);

    /* poison vertices behind */

    set<NFAVertex> starters;
    insert(&starters, picked.vv);

    set<NFAVertex> visited;

    flood_back(g, len, starters, &visited);

    DEBUG_PRINTF("flood %zu vertices\n", visited.size());

    /* inspect any back edges which are in the flooded subgraph; look for any
     * destination vertices which are not starters */
    set<NFAVertex> anti;
    for (auto u : visited) {
        if (!contains(back_edges, u) || contains(starters, u)) {
            continue;
        }

        for (auto v : back_edges[u]) {
            if (contains(visited, v) && !contains(starters, v)) {
                anti.insert(v);
            }
        }
    }
    DEBUG_PRINTF("%zu cycle ends\n", visited.size());

    /* remove any vertices which lie on the other side of a cycle from the
     * visited set */
    set<NFAVertex> anti_pred;
    flood_back(g, len - 1, anti, &anti_pred);

    DEBUG_PRINTF("flood visited %zu vertices; anti %zu\n", visited.size(),
                 anti_pred.size());

    erase_all(&visited, anti_pred);

    DEBUG_PRINTF("filtered flood visited %zu vertices\n", visited.size());

    insert(&poisoned, visited);

    insert(&poisoned, starters); /* complicated back loops can result in start
                                    vertices being removed from the visited
                                    set */

    for (UNUSED auto v : picked.vv) {
        assert(contains(poisoned, v));
    }

    /* TODO: poison vertices in front of us? */
}

void LitCollection::poisonCandidates(const VertLitInfo &picked) {
    assert(!picked.lit.empty());
    if (picked.lit.empty()) {
        return;
    }

    if ((seeking_anchored && createsAnchoredLHS(g, picked.vv, depths, grey))
    || (seeking_transient && createsTransientLHS(g, picked.vv, depths, grey))) {
        /* We don't want to pick anything to the LHS of picked.v any more as we
         * have something good. We also don't want to provide any bonus for
         * remaining literals based on anchoredness/transientness of the lhs.
         */
        poisonLHS(picked);
    } else {
        poisonLitVerts(picked);
    }
}

unique_ptr<VertLitInfo> LitCollection::pickNext() {
    while (!lits.empty()) {
        if (0) {
        next_lit:
            continue;
        }

        for (auto v : lits.back()->vv) {
            if (contains(poisoned, v)) {
                DEBUG_PRINTF("skipping '%s' as overlapped\n",
                     ((const string &)*lits.back()->lit.begin()).c_str());
                lits.pop_back();
                goto next_lit;
            }
        }

        unique_ptr<VertLitInfo> rv = move(lits.back());
        lits.pop_back();
        poisonCandidates(*rv);
        DEBUG_PRINTF("best is '%s' %u a%d t%d\n",
                     ((const string &)*rv->lit.begin()).c_str(),
                     g[rv->vv.front()].index,
                     (int)createsAnchoredLHS(g, rv->vv, depths, grey),
                     (int)createsTransientLHS(g, rv->vv, depths, grey));

        return rv;
    }

    return nullptr;
}

}

/** \brief Returns true if the given literal is the only thing in the graph,
 * from start to accept. */
static
bool literalIsWholeGraph(const NGHolder &g, const ue2_literal &lit) {
    NFAVertex v = g.accept;

    for (auto it = lit.rbegin(), ite = lit.rend(); it != ite; ++it) {
        NFAGraph::inv_adjacency_iterator ai, ae;
        tie(ai, ae) = inv_adjacent_vertices(v, g);
        if (ai == ae) {
            assert(0); // no predecessors?
            return false;
        }
        v = *ai++;
        if (ai != ae) {
            DEBUG_PRINTF("branch, fail\n");
            return false;
        }

        if (is_special(v, g)) {
            DEBUG_PRINTF("special found, fail\n");
            return false;
        }

        const CharReach &cr = g[v].char_reach;
        if (cr != *it) {
            DEBUG_PRINTF("reach fail\n");
            return false;
        }
    }

    // Our last value for v should have only start states for predecessors.
    for (auto u : inv_adjacent_vertices_range(v, g)) {
        if (!is_any_start(u, g)) {
            DEBUG_PRINTF("pred is not start\n");
            return false;
        }
    }

    assert(num_vertices(g) == lit.length() + N_SPECIALS);

    DEBUG_PRINTF("ok\n");
    return true;
}

static
bool can_match(const NGHolder &g, const ue2_literal &lit, bool overhang_ok) {
    set<NFAVertex> curr, next;
    curr.insert(g.accept);

    for (auto it = lit.rbegin(); it != lit.rend(); ++it) {
        next.clear();

        for (auto v : curr) {
            for (auto u : inv_adjacent_vertices_range(v, g)) {
                if (u == g.start) {
                    if (overhang_ok) {
                        DEBUG_PRINTF("bail\n");
                        return true;
                    } else {
                        continue; /* it is not possible for a lhs literal to
                                   * overhang the start */
                    }
                }

                const CharReach &cr = g[u].char_reach;
                if (!overlaps(*it, cr)) {
                    DEBUG_PRINTF("skip\n");
                    continue;
                }

                next.insert(u);
            }
        }

        curr.swap(next);
    }

    return !curr.empty();
}

u32 removeTrailingLiteralStates(NGHolder &g, const ue2_literal &lit,
                                u32 max_delay, bool overhang_ok) {
    if (max_delay == MO_INVALID_IDX) {
        max_delay--;
    }

    DEBUG_PRINTF("killing off '%s'\n", ((const string &)lit).c_str());
    set<NFAVertex> curr, next;
    curr.insert(g.accept);

    auto it = lit.rbegin();
    for (u32 delay = max_delay; delay > 0 && it != lit.rend(); delay--, ++it) {
        next.clear();
        for (auto v : curr) {
            for (auto u : inv_adjacent_vertices_range(v, g)) {
                if (u == g.start) {
                    if (overhang_ok) {
                        DEBUG_PRINTF("bail\n");
                        goto bail; /* things got complicated */
                    } else {
                        continue; /* it is not possible for a lhs literal to
                                   * overhang the start */
                    }
                }

                const CharReach &cr = g[u].char_reach;
                if (!overlaps(*it, cr)) {
                    DEBUG_PRINTF("skip\n");
                    continue;
                }
                if (isSubsetOf(*it, cr)) {
                    next.insert(u);
                } else {
                    DEBUG_PRINTF("bail\n");
                    goto bail; /* things got complicated */
                }
            }
        }

        curr.swap(next);
    }
 bail:
    if (curr.empty()) {
        /* This can happen when we have an edge representing a cross from two
         * sides of an alternation. This whole edge needs to be marked as
         * dead */
        assert(0); /* should have been picked up by can match */
        return MO_INVALID_IDX;
    }

    u32 delay = distance(lit.rbegin(), it);
    assert(delay <= max_delay);
    assert(delay <= lit.length());
    DEBUG_PRINTF("managed delay %u (of max %u)\n", delay, max_delay);

    // For determinism, we make sure that we create these edges from vertices
    // in index-sorted order.
    set<NFAVertex> pred;
    for (auto v : curr) {
        insert(&pred, inv_adjacent_vertices_range(v, g));
    }

    clear_in_edges(g.accept, g);

    vector<NFAVertex> verts(pred.begin(), pred.end());
    sort(verts.begin(), verts.end(), VertexIndexOrdering<NGHolder>(g));

    for (auto v : verts) {
        add_edge(v, g.accept, g);
        g[v].reports.insert(0);
    }

    pruneUseless(g);
    assert(allMatchStatesHaveReports(g));

    DEBUG_PRINTF("graph has %zu vertices left\n", num_vertices(g));
    return delay;
}

static
void restoreTrailingLiteralStates(NGHolder &g, const ue2_literal &lit,
                                  u32 delay) {
    assert(delay <= lit.length());
    DEBUG_PRINTF("adding on '%s' %u\n", ((const string &)lit).c_str(), delay);

    vector<NFAVertex> preds;
    insert(&preds, preds.end(), inv_adjacent_vertices(g.accept, g));
    clear_in_edges(g.accept, g);

    for (auto v : preds) {
        g[v].reports.clear(); /* clear report from old accepts */
    }

    NFAVertex prev = g.accept;
    auto it = lit.rbegin();
    while (delay--) {
        NFAVertex curr = add_vertex(g);
        assert(it != lit.rend());
        g[curr].char_reach = *it;
        add_edge(curr, prev, g);
        ++it;
        prev = curr;
    }

    for (auto v : preds) {
        add_edge(v, prev, g);
    }

    // Every predecessor of accept must have a report.
    for (auto u : inv_adjacent_vertices_range(g.accept, g)) {
        g[u].reports.insert(0);
    }

    g.renumberVertices();
    g.renumberEdges();
    assert(allMatchStatesHaveReports(g));
}

/* return false if we should get rid of the edge altogether */
static
bool removeLiteralFromLHS(RoseInGraph &ig, const RoseInEdge &lhs,
                          const CompileContext &cc) {
    unique_ptr<NGHolder> h = cloneHolder(*ig[lhs].graph);
    NGHolder &g = *h;
    assert(ig[target(lhs, ig)].type == RIV_LITERAL);
    const ue2_literal &lit = ig[target(lhs, ig)].s;

    /* lhs should be connected to a start */
    assert(ig[source(lhs, ig)].type == RIV_START
           || ig[source(lhs, ig)].type == RIV_ANCHORED_START);

    if (in_degree(g.acceptEod, g) != 1 /* edge from accept */) {
        assert(0);
        return true;
    }
    if (lit.empty()) {
        assert(0);
        return true;
    }

    const u32 max_delay = maxDelay(cc);

    // In streaming mode, we must limit the depth to the available history
    // UNLESS the given literal follows start or startDs and has nothing
    // before it that we will need to account for. In that case, we can
    // lean on FDR's support for long literals.
    if (literalIsWholeGraph(g, lit)) {
        assert(!ig[lhs].haig);
        assert(ig[lhs].minBound == 0);
        assert(ig[lhs].maxBound == ROSE_BOUND_INF);
        DEBUG_PRINTF("literal is the whole graph\n");

        u32 delay = removeTrailingLiteralStates(g, lit, MO_INVALID_IDX, false);
        assert(delay == lit.length());
        ig[lhs].graph = move(h);
        ig[lhs].graph_lag = delay;
        return true;
    }

    if (!can_match(g, lit, false)) {
        /* This is can happen if the literal arises from a large cyclic
           to/beyond the pivot. As the LHS graph only cares about the first
           reach of the pivot, this literal is junk */
        DEBUG_PRINTF("bogus edge\n");
        return false;
    }

    u32 delay = removeTrailingLiteralStates(g, lit, max_delay,
                                            false /* can't overhang start */);

    if (delay == MO_INVALID_IDX) {
        /* This is can happen if the literal arises from a large cyclic
           to/beyond the pivot. As the LHS graph only cares about the first
           reach of the pivot, this literal is junk */
        DEBUG_PRINTF("bogus edge\n");
        return false;
    }

    if (!delay) {
        return true;
    }

    DEBUG_PRINTF("setting delay %u on lhs %p\n", delay, h.get());

    ig[lhs].graph = move(h);
    ig[lhs].graph_lag = delay;
    return true;
}

static
void handleLhsCliche(RoseInGraph &ig, const RoseInEdge &lhs) {
    const NGHolder &h = *ig[lhs].graph;

    size_t s_od = out_degree(h.start, h);
    size_t sds_od = out_degree(h.startDs, h);

    assert(in_degree(h.acceptEod, h) == 1 /* edge from accept */);
    /* need to check if simple floating start */
    if (edge(h.startDs, h.accept, h).second && sds_od == 2
        && ((s_od == 2 && edge(h.start, h.accept, h).second) || s_od == 1)) {
        /* no need for graph */
        ig[lhs].graph.reset();
        ig[lhs].graph_lag = 0;
        DEBUG_PRINTF("lhs is floating start\n");
        return;
    }

    /* need to check if a simple anchor */
    /* start would have edges to sds and accept in this case */
    if (edge(h.start, h.accept, h).second && s_od == 2 && sds_od == 1) {
        if (ig[source(lhs, ig)].type == RIV_ANCHORED_START) {
            // assert(ig[lhs].graph_lag == ig[target(lhs, ig)].s.length());
            if (ig[lhs].graph_lag != ig[target(lhs, ig)].s.length()) {
                DEBUG_PRINTF("oddness\n");
                return;
            }
            ig[lhs].graph.reset();
            ig[lhs].graph_lag = 0;
            ig[lhs].maxBound = 0;
            DEBUG_PRINTF("lhs is anchored start\n");
        } else {
            DEBUG_PRINTF("lhs rewiring start\n");
            assert(ig[source(lhs, ig)].type == RIV_START);
            RoseInVertex t = target(lhs, ig);
            remove_edge(lhs, ig);
            RoseInVertex s2
                = add_vertex(RoseInVertexProps::makeStart(true), ig);
            add_edge(s2, t, RoseInEdgeProps(0U, 0U), ig);
        }
        return;
    }
}

static
void filterCandPivots(const NGHolder &g, const set<NFAVertex> &cand_raw,
                      set<NFAVertex> *out) {
    for (auto u : cand_raw) {
        const CharReach &u_cr = g[u].char_reach;
        if (u_cr.count() > 40) {
            continue; /* too wide to be plausible */
        }

        if (u_cr.count() > 2) {
            /* include u as a candidate as successor may have backed away from
             * expanding through it */
            out->insert(u);
            continue;
        }

        NFAVertex v = getSoleDestVertex(g, u);
        if (v && in_degree(v, g) == 1 && out_degree(u, g) == 1) {
            const CharReach &v_cr = g[v].char_reach;
            if (v_cr.count() == 1 || v_cr.isCaselessChar()) {
                continue; /* v will always generate better literals */
            }
        }

        out->insert(u);
    }
}

/* cand_raw is the candidate set before filtering points which are clearly
 * a bad idea. */
static
void getCandidatePivots(const NGHolder &g, set<NFAVertex> *cand,
                        set<NFAVertex> *cand_raw) {
    ue2::unordered_map<NFAVertex, NFAVertex> dominators =
        findDominators(g);

    set<NFAVertex> accepts;

    for (auto v : inv_adjacent_vertices_range(g.accept, g)) {
        if (is_special(v, g)) {
            continue;
        }
        accepts.insert(v);
    }
    for (auto v : inv_adjacent_vertices_range(g.acceptEod, g)) {
        if (is_special(v, g)) {
            continue;
        }
        accepts.insert(v);
    }

    assert(!accepts.empty());

    vector<NFAVertex> dom_trace;
    auto ait = accepts.begin();
    assert(ait != accepts.end());
    NFAVertex curr = *ait;
    while (curr && !is_special(curr, g)) {
        dom_trace.push_back(curr);
        curr = dominators[curr];
    }
    reverse(dom_trace.begin(), dom_trace.end());
    for (++ait; ait != accepts.end(); ++ait) {
        curr = *ait;
        vector<NFAVertex> dom_trace2;
        while (curr && !is_special(curr, g)) {
            dom_trace2.push_back(curr);
            curr = dominators[curr];
        }
        reverse(dom_trace2.begin(), dom_trace2.end());
        auto dti = dom_trace.begin(), dtie = dom_trace.end();
        auto dtj = dom_trace2.begin(), dtje = dom_trace2.end();
        while (dti != dtie && dtj != dtje && *dti == *dtj) {
            ++dti;
            ++dtj;
        }
        dom_trace.erase(dti, dtie);
    }

    cand_raw->insert(dom_trace.begin(), dom_trace.end());

    filterCandPivots(g, *cand_raw, cand);
}

static
void deanchorIfNeeded(NGHolder &g, bool *orig_anch) {
    DEBUG_PRINTF("hi\n");
    if (proper_out_degree(g.startDs, g)) {
        return;
    }

    /* look for a non-special dot with a loop following start */
    set<NFAVertex> succ_g;
    insert(&succ_g, adjacent_vertices(g.start, g));
    succ_g.erase(g.startDs);

    for (auto v : adjacent_vertices_range(g.start, g)) {
        DEBUG_PRINTF("inspecting cand %u || =%zu\n", g[v].index,
                     g[v].char_reach.size());

        if (v == g.startDs || !g[v].char_reach.all()) {
            continue;
        }

        set<NFAVertex> succ_v;
        insert(&succ_v, adjacent_vertices(v, g));

        if (succ_v == succ_g) {
            DEBUG_PRINTF("found ^.*\n");
            *orig_anch = true;
            for (auto succ : succ_g) {
                add_edge(g.startDs, succ, g);
            }
            clear_vertex(v, g);
            remove_vertex(v, g);
            g.renumberVertices();
            return;
        }

        if (succ_g.size() == 1 && hasSelfLoop(v, g)) {
            DEBUG_PRINTF("found ^.+\n");
            *orig_anch = true;
            add_edge(g.startDs, v, g);
            remove_edge(v, v, g);
            return;
        }
    }
}

static
unique_ptr<RoseInGraph> makeTrivialGraph(const NGHolder &h,
                                         vdest_map_t &v_dest_map,
                                         vsrc_map_t &v_src_map) {
    shared_ptr<NGHolder> root_g = cloneHolder(h);
    bool orig_anch = isAnchored(*root_g);
    deanchorIfNeeded(*root_g, &orig_anch);

    DEBUG_PRINTF("orig_anch %d\n", (int)orig_anch);

    unique_ptr<RoseInGraph> igp = ue2::make_unique<RoseInGraph>();
    RoseInVertex start =
        add_vertex(RoseInVertexProps::makeStart(orig_anch), *igp);
    RoseInVertex accept =
        add_vertex(RoseInVertexProps::makeAccept(set<ReportID>()), *igp);

    RoseInEdge e =
        add_edge(start, accept, RoseInEdgeProps(root_g, 0), *igp).first;

    for (auto v : vertices_range(*root_g)) {
        v_dest_map[v].emplace_back(e, v);
        v_src_map[e].push_back(v);
    }

    return igp;
}

static never_inline
void updateVDestMap(const vector<pair<RoseInEdge, NFAVertex> > &images,
                    const ue2::unordered_map<NFAVertex, NFAVertex> &lhs_map,
                    const vector<RoseInEdge> &l_e,
                    const ue2::unordered_map<NFAVertex, NFAVertex> &rhs_map,
                    const vector<RoseInEdge> &r_e,
                    vdest_map_t &v_dest_map, vsrc_map_t &v_src_map) {
    RoseInEdge e = images.front().first;
    set<RoseInEdge> edge_set;
    for (const auto &image : images) {
        edge_set.insert(image.first);
    }
    const vector<NFAVertex> &domain = v_src_map[e];
    vector<pair<RoseInEdge, NFAVertex> > temp;

    for (auto v : domain) {
        vdest_map_t::iterator it = v_dest_map.find(v);
        assert(it != v_dest_map.end());

        temp.clear();

        for (const auto &dest : it->second) {
            const RoseInEdge &old_e = dest.first;
            const NFAVertex old_dest = dest.second;
            if (old_e != e) {
                if (!contains(edge_set, old_e)) {
                    temp.emplace_back(old_e, old_dest);
                }
            } else if (contains(lhs_map, old_dest)) {
                for (const auto &e2 : l_e) {
                    temp.emplace_back(e2, lhs_map.at(old_dest));
                }
            /* only allow v to be tracked on one side of the split */
            } else if (contains(rhs_map, old_dest)) {
                for (const auto &e2 : r_e) {
                    temp.emplace_back(e2, rhs_map.at(old_dest));
                }
            }
        }
        NDEBUG_PRINTF("%zu images for vertex; prev %zu\n", temp.size(),
                     it->second.size());
        it->second.swap(temp);
    }
}

/** Returns the collection of vertices from the original graph which end up
 * having an image in the [lr]hs side of the graph split. */
static never_inline
void fillDomain(const vdest_map_t &v_dest_map, const vsrc_map_t &v_src_map,
                RoseInEdge e,
                const ue2::unordered_map<NFAVertex, NFAVertex> &split_map,
                vector<NFAVertex> *out) {
    const vector<NFAVertex> &presplit_domain = v_src_map.at(e);
    for (auto v : presplit_domain) {
        /* v is in the original graph, need to find its image on e's graph */
        typedef vector<pair<RoseInEdge, NFAVertex> > dests_t;
        const dests_t &dests = v_dest_map.at(v);
        for (const auto &dest : dests) {
            if (dest.first == e) {
                NFAVertex vv = dest.second;
                /* vv is v image on e's graph */
                if (contains(split_map, vv)) {
                    out->push_back(v);
                }
            }
        }
    }
}

static
void getSourceVerts(RoseInGraph &ig,
                    const vector<pair<RoseInEdge, NFAVertex> > &images,
                    vector<RoseInVertex> *out) {
    set<RoseInVertex> seen;
    for (const auto &image : images) {
        RoseInVertex s = source(image.first, ig);
        if (contains(seen, s)) {
            continue;
        }
        seen.insert(s);
        out->push_back(s);
    }
}

static
void getDestVerts(RoseInGraph &ig,
                  const vector<pair<RoseInEdge, NFAVertex> > &images,
                  vector<RoseInVertex> *out) {
    set<RoseInVertex> seen;
    for (const auto &image : images) {
        RoseInVertex t = target(image.first, ig);
        if (contains(seen, t)) {
            continue;
        }
        seen.insert(t);
        out->push_back(t);
    }
}

static
void getSourceVerts(RoseInGraph &ig, const vector<RoseInEdge> &edges,
                    vector<RoseInVertex> *out) {
    set<RoseInVertex> seen;
    for (const auto &e : edges) {
        RoseInVertex s = source(e, ig);
        if (contains(seen, s)) {
            continue;
        }
        seen.insert(s);
        out->push_back(s);
    }
}

static
void getDestVerts(RoseInGraph &ig, const vector<RoseInEdge> &edges,
                  vector<RoseInVertex> *out) {
    set<RoseInVertex> seen;
    for (const auto &e : edges) {
        RoseInVertex t = target(e, ig);
        if (contains(seen, t)) {
            continue;
        }
        seen.insert(t);
        out->push_back(t);
    }
}

static
bool splitRoseEdge(RoseInGraph &ig, const VertLitInfo &split,
                   vdest_map_t &v_dest_map, vsrc_map_t &v_src_map) {
    const vector<NFAVertex> &root_splitters = split.vv; /* vertices in the
                                                           'root' graph */
    assert(!root_splitters.empty());

    /* need copy as split rose edge will update orig map */
    vector<pair<RoseInEdge, NFAVertex> > images
        = v_dest_map[root_splitters[0]];
    DEBUG_PRINTF("splitting %zu rose edge with %zu literals\n",
                 images.size(), split.lit.size());

    /* note: as we haven't removed literals yet the graphs on all edges that we
     * are going to split should be identical */
    const auto &base_graph = ig[images.front().first].graph;

    vector<NFAVertex> splitters; /* vertices in the graph being split */
    for (auto v : root_splitters) {
        if (!contains(v_dest_map, v)) {
            DEBUG_PRINTF("vertex to split on is no longer in the graph\n");
            return false;
        }

        /* sanity check: verify all edges have the same underlying graph */
        for (UNUSED const auto &m : v_dest_map[v]) {
            assert(base_graph == ig[m.first].graph);
        }
        assert(v_dest_map[v].size() == images.size());

        splitters.push_back(v_dest_map[v].front().second);
    }

    /* note: the set of split edges should form a complete bipartite graph */
    vector<RoseInVertex> src_verts;
    vector<RoseInVertex> dest_verts;
    getSourceVerts(ig, images, &src_verts);
    getDestVerts(ig, images, &dest_verts);
    assert(images.size() == src_verts.size() * dest_verts.size());

    shared_ptr<NGHolder> lhs = make_shared<NGHolder>();
    shared_ptr<NGHolder> rhs = make_shared<NGHolder>();

    ue2::unordered_map<NFAVertex, NFAVertex> lhs_map;
    ue2::unordered_map<NFAVertex, NFAVertex> rhs_map;

    assert(base_graph);
    splitGraph(*base_graph, splitters, lhs.get(), &lhs_map,
                                       rhs.get(), &rhs_map);

    RoseInEdge first_e = images.front().first;

    /* all will be suffix or none */
    bool suffix = ig[target(first_e, ig)].type == RIV_ACCEPT;

    set<ReportID> splitter_reports;
    for (auto v : splitters) {
        insert(&splitter_reports, (*base_graph)[v].reports);
    }

    bool do_accept = false;
    bool do_accept_eod = false;
    assert(rhs);
    if (isVacuous(*rhs) && suffix) {
        if (edge(rhs->start, rhs->accept, *rhs).second) {
                DEBUG_PRINTF("rhs has a cliche\n");
                do_accept = true;
                remove_edge(rhs->start, rhs->accept, *rhs);
        }

        if (edge(rhs->start, rhs->acceptEod, *rhs).second) {
            DEBUG_PRINTF("rhs has an eod cliche\n");
            do_accept_eod = true;
            remove_edge(rhs->start, rhs->acceptEod, *rhs);
        }
    }

    bool do_norm = out_degree(rhs->start, *rhs) != 1; /* check if we still have
                                                         a graph left over */
    vector<NFAVertex> lhs_domain;
    vector<NFAVertex> rhs_domain;
    fillDomain(v_dest_map, v_src_map, first_e, lhs_map, &lhs_domain);
    fillDomain(v_dest_map, v_src_map, first_e, rhs_map, &rhs_domain);

    vector<RoseInEdge> l_e;
    vector<RoseInEdge> r_e;
    for (const auto &lit : split.lit) {
        DEBUG_PRINTF("best is '%s'\n", escapeString(lit).c_str());
        RoseInVertex v
            = add_vertex(RoseInVertexProps::makeLiteral(lit), ig);

        /* work out delay later */
        if (do_accept) {
            DEBUG_PRINTF("rhs has a cliche\n");
            RoseInVertex tt = add_vertex(RoseInVertexProps::makeAccept(
                                        splitter_reports), ig);
            add_edge(v, tt, RoseInEdgeProps(0U, 0U), ig);
        }

        if (do_accept_eod) {
            DEBUG_PRINTF("rhs has an eod cliche\n");
            RoseInVertex tt = add_vertex(RoseInVertexProps::makeAcceptEod(
                                        splitter_reports), ig);
            add_edge(v, tt, RoseInEdgeProps(0U, 0U), ig);
        }

        for (auto src_v : src_verts) {
            l_e.push_back(add_edge(src_v, v,
                                   RoseInEdgeProps(lhs, 0U), ig).first);
            v_src_map[l_e.back()] = lhs_domain;
        }

        if (do_norm) {
            for (auto dst_v : dest_verts) {
                /* work out delay later */
                assert(out_degree(rhs->start, *rhs) > 1);
                r_e.push_back(
                    add_edge(v, dst_v, RoseInEdgeProps(rhs, 0U), ig).first);
                v_src_map[r_e.back()] = rhs_domain;
            }
        }
    }

    updateVDestMap(images, lhs_map, l_e, rhs_map, r_e, v_dest_map, v_src_map);

    for (const auto &image : images) {
        /* remove old edge */
        remove_edge(image.first, ig);
        v_src_map.erase(image.first);
    }

    return true;
}

static
bool isStarCliche(const NGHolder &g) {
    DEBUG_PRINTF("checking graph with %zu vertices\n", num_vertices(g));

    bool nonspecials_seen = false;

    for (auto v : vertices_range(g)) {
        if (is_special(v, g)) {
            continue;
        }

        if (nonspecials_seen) {
            return false;
        }
        nonspecials_seen = true;

        if (!g[v].char_reach.all()) {
            return false;
        }

        if (!hasSelfLoop(v, g)) {
            return false;
        }
        if (!edge(v, g.accept, g).second) {
            return false;
        }
    }

    if (!nonspecials_seen) {
        return false;
    }

    if (!edge(g.start, g.accept, g).second) {
        return false;
    }

    return true;
}

static
void processInfixes(RoseInGraph &ig, const CompileContext &cc) {
    /* we want to ensure that every prefix/infix graph is unique at this stage
     * as we have not done any analysis to check if they are safe to share */

    vector<RoseInEdge> dead;

    for (const auto &e : edges_range(ig)) {
        if (!ig[e].graph) {
            continue;
        }

        RoseInVertex u = source(e, ig), v = target(e, ig);

        // Infixes are edges between two literals.
        if (ig[u].type != RIV_LITERAL || ig[v].type != RIV_LITERAL) {
            continue;
        }

        if (ig[e].graph_lag) {
            continue; /* already looked at */
        }

        DEBUG_PRINTF("looking at infix %p\n", ig[e].graph.get());

        const ue2_literal &lit1 = ig[u].s;
        const ue2_literal &lit2 = ig[v].s;
        size_t overlap = maxOverlap(lit1, lit2, 0);

        const NGHolder &h = *ig[e].graph;

        DEBUG_PRINTF("infix rose between literals '%s' and '%s', overlap %zu,"
                     "size %zu\n",
                     dumpString(lit1).c_str(), dumpString(lit2).c_str(),
                     overlap, num_vertices(h));

        if (!can_match(h, lit2, true)) {
            DEBUG_PRINTF("found bogus edge\n");
            dead.push_back(e);
            continue;
        }

        unique_ptr<NGHolder> h_new = cloneHolder(h);

        u32 delay = removeTrailingLiteralStates(*h_new, lit2, MO_INVALID_IDX);
        if (delay == MO_INVALID_IDX) {
            DEBUG_PRINTF("found bogus edge\n");
            dead.push_back(e);
            continue;
        }

        // Delay can be set to at most lit2.length() - overlap, but we must
        // truncate to history available in streaming mode.
        u32 max_allowed_delay = lit2.length() - overlap;
        LIMIT_TO_AT_MOST(&max_allowed_delay, delay);

        if (cc.streaming) {
            LIMIT_TO_AT_MOST(&max_allowed_delay, cc.grey.maxHistoryAvailable);
        }

        if (delay != max_allowed_delay) {
            restoreTrailingLiteralStates(*h_new, lit2, delay);
            delay = removeTrailingLiteralStates(*h_new, lit2, max_allowed_delay);
        }

        if (isStarCliche(*h_new)) {
            DEBUG_PRINTF("is a X star!\n");
            ig[e].graph.reset();
            ig[e].graph_lag = 0;
        } else {
            ig[e].graph = move(h_new);
            ig[e].graph_lag = delay;
            DEBUG_PRINTF("delay increased to %u\n", delay);
        }
    }

    for (const auto &e : dead) {
        remove_edge(e, ig);
    }
}

static
void poisonNetflowScores(RoseInGraph &ig, RoseInEdge lhs,
                         vector<u64a> *scores) {
    assert(ig[lhs].graph);
    NGHolder &h = *ig[lhs].graph;

    if (ig[target(lhs, ig)].type != RIV_LITERAL) {
        /* nothing to poison in outfixes */
        assert(ig[target(lhs, ig)].type == RIV_ACCEPT);
        return;
    }

    set<NFAVertex> curr, next;
    insert(&curr, inv_adjacent_vertices(h.accept, h));
    set<NFAEdge> poisoned;
    u32 len = ig[target(lhs, ig)].s.length();
    assert(len);
    while (len) {
        next.clear();
        for (auto v : curr) {
            insert(&poisoned, in_edges(v, h));
            insert(&next, inv_adjacent_vertices(v, h));
        }

        curr.swap(next);
        len--;
    }

    for (const auto &e : poisoned) {
        (*scores)[h[e].index] = NO_LITERAL_AT_EDGE_SCORE;
    }
}

#define MAX_NETFLOW_CUT_WIDTH 40 /* magic number is magic */
#define MAX_LEN_2_LITERALS_PER_CUT 3

static
bool checkValidNetflowLits(NGHolder &h, const vector<u64a> &scores,
                           const map<NFAEdge, set<ue2_literal>> &cut_lits,
                           const Grey &grey) {
    DEBUG_PRINTF("cut width %zu\n", cut_lits.size());
    if (cut_lits.size() > MAX_NETFLOW_CUT_WIDTH) {
        return false;
    }

    u32 len_2_count = 0;

    for (const auto &cut : cut_lits) {
        if (scores[h[cut.first].index] >= NO_LITERAL_AT_EDGE_SCORE) {
            DEBUG_PRINTF("cut uses a forbidden edge\n");
            return false;
        }

        if (min_len(cut.second) < grey.minRoseNetflowLiteralLength) {
            DEBUG_PRINTF("cut uses a bad literal\n");
            return false;
        }

        for (const auto &lit : cut.second) {
            if (lit.length() == 2) {
                len_2_count++;
            }
        }
    }

    if (len_2_count > MAX_LEN_2_LITERALS_PER_CUT) {
        return false;
    }

    return true;
}

static
void splitEdgesByCut(RoseInGraph &ig, const vector<RoseInEdge> &to_cut,
                     const vector<NFAEdge> &cut,
                     const map<NFAEdge, set<ue2_literal> > &cut_lits) {
    assert(!to_cut.empty());
    assert(ig[to_cut.front()].graph);
    NGHolder &h = *ig[to_cut.front()].graph;

    /* note: the set of split edges should form a complete bipartite graph */
    vector<RoseInVertex> src_verts;
    vector<RoseInVertex> dest_verts;
    getSourceVerts(ig, to_cut, &src_verts);
    getDestVerts(ig, to_cut, &dest_verts);
    assert(to_cut.size() == src_verts.size() * dest_verts.size());

    map<vector<NFAVertex>, shared_ptr<NGHolder> > done_rhs;

    /* iterate over cut for determinism */
    for (const auto &e : cut) {
        NFAVertex prev_v = source(e, h);
        NFAVertex pivot = target(e, h);

        vector<NFAVertex> adj;
        insert(&adj, adj.end(), adjacent_vertices(pivot, h));
        /* we can ignore presence of accept, accepteod in adj as it is best
           effort */

        if (!contains(done_rhs, adj)) {
            ue2::unordered_map<NFAVertex, NFAVertex> temp_map;
            shared_ptr<NGHolder> new_rhs = make_shared<NGHolder>();
            splitRHS(h, adj, new_rhs.get(), &temp_map);
            remove_edge(new_rhs->start, new_rhs->accept, *new_rhs);
            remove_edge(new_rhs->start, new_rhs->acceptEod, *new_rhs);
            done_rhs.insert(make_pair(adj, new_rhs));
            /* TODO need to update v_mapping (if we were doing more cuts) */
        }

        DEBUG_PRINTF("splitting on pivot %u\n", h[pivot].index);
        ue2::unordered_map<NFAVertex, NFAVertex> temp_map;
        shared_ptr<NGHolder> new_lhs = make_shared<NGHolder>();
        splitLHS(h, pivot, new_lhs.get(), &temp_map);

        /* want to cut of paths to pivot from things other than the pivot -
         * makes a more svelte graphy */
        clear_in_edges(temp_map[pivot], *new_lhs);
        add_edge(temp_map[prev_v], temp_map[pivot], *new_lhs);

        pruneUseless(*new_lhs);

        const set<ue2_literal> &lits = cut_lits.at(e);
        for (const auto &lit : lits) {
            RoseInVertex v
                = add_vertex(RoseInVertexProps::makeLiteral(lit), ig);

            if (edge(pivot, h.accept, h).second) {
                /* literal has a direct connection to accept */
                assert(ig[dest_verts.front()].type == RIV_ACCEPT);
                const auto &reports = h[pivot].reports;
                RoseInVertex tt =
                    add_vertex(RoseInVertexProps::makeAccept(reports), ig);
                add_edge(v, tt, RoseInEdgeProps(0U, 0U), ig);
            }

            if (edge(pivot, h.acceptEod, h).second) {
                /* literal has a direct connection to accept */
                assert(ig[dest_verts.front()].type == RIV_ACCEPT);
                const auto &reports = h[pivot].reports;
                RoseInVertex tt = add_vertex(
                    RoseInVertexProps::makeAcceptEod(reports), ig);
                add_edge(v, tt, RoseInEdgeProps(0U, 0U), ig);
            }

            assert(done_rhs[adj].get());
            shared_ptr<NGHolder> new_rhs = done_rhs[adj];
            if (out_degree(new_rhs->start, *new_rhs) != 1) {
                for (auto dst_v : dest_verts) {
                    add_edge(v, dst_v, RoseInEdgeProps(done_rhs[adj], 0), ig);
                }
            }

            for (auto src_v : src_verts) {
                add_edge(src_v, v, RoseInEdgeProps(new_lhs, 0), ig);
            }
        }
    }

    /* TODO need to update v_mapping (if we were doing more cuts) */

    for (const auto &e : to_cut) {
        assert(ig[e].graph.get() == &h);
        remove_edge(e, ig);
    }
}

static
bool doNetflowCut(RoseInGraph &ig, const vector<RoseInEdge> &to_cut,
                  const Grey &grey) {
    DEBUG_PRINTF("doing netflow cut\n");
    /* TODO: we should really get literals/scores from the full graph as this
     * allows us to overlap the graph. Doesn't matter at the moment as we
     * are working on the LHS. */

    NGHolder &h = *ig[to_cut.front()].graph;
    if (num_edges(h) > grey.maxRoseNetflowEdges) {
        /* We have a limit on this because scoring edges and running netflow
         * gets very slow for big graphs. */
        DEBUG_PRINTF("too many edges, skipping netflow cut\n");
        return false;
    }

    h.renumberVertices();
    h.renumberEdges();
    /* Step 1: Get scores for all edges */
    vector<u64a> scores = scoreEdges(h); /* scores by edge_index */
    /* Step 2: poison scores for edges covered by successor literal */
    for (const auto &e : to_cut) {
        assert(&h == ig[e].graph.get());
        poisonNetflowScores(ig, e, &scores);
    }
    /* Step 3: Find cutset based on scores */
    vector<NFAEdge> cut = findMinCut(h, scores);

    /* Step 4: Get literals corresponding to cut edges */
    map<NFAEdge, set<ue2_literal>> cut_lits;
    for (const auto &e : cut) {
        set<ue2_literal> lits = getLiteralSet(h, e);
        compressAndScore(lits);
        cut_lits[e] = lits;

        DEBUG_PRINTF("cut lit '%s'\n",
                     ((const string &)*cut_lits[e].begin()).c_str());
    }

    /* if literals are underlength bail or if it involves a forbidden edge*/
    if (!checkValidNetflowLits(h, scores, cut_lits, grey)) {
        return false;
    }
    DEBUG_PRINTF("splitting\n");

    /* Step 5: Split graph based on cuts */
    splitEdgesByCut(ig, to_cut, cut, cut_lits);
    return true;
}

/** \brief Returns the number of intermediate vertices in the shortest path
 * between (from, to). */
static
u32 min_dist_between(NFAVertex from, NFAVertex to, const NGHolder &g) {
    // Check for the trivial case: that way we don't have to set up the
    // containers below.
    if (edge(from, to, g).second) {
        return 0;
    }

    ue2::unordered_set<NFAVertex> visited;
    visited.insert(from);

    flat_set<NFAVertex> curr, next;
    curr.insert(from);

    assert(from != to);

    u32 d = 0;

    while (!curr.empty()) {
        next.clear();
        for (auto v : curr) {
            for (auto w : adjacent_vertices_range(v, g)) {
                if (w == to) {
                    return d;
                }
                if (visited.insert(w).second) { // first visit to *ai
                    next.insert(w);
                }
            }
        }

        d++;
        curr.swap(next);
    }
    assert(0);
    return ROSE_BOUND_INF;
}

/** Literals which are completely enveloped by a successor are trouble because
 * hamsterwheel acceleration can skip past the start of the literal. */
static
bool enveloped(const vector<NFAVertex> &cand_split_v,
               const set<ue2_literal> &cand_lit, const NGHolder &g,
               const RoseInVertexProps &succ) {
    if (succ.type != RIV_LITERAL) {
        return false;
    }

    /* TODO: handle multiple v more precisely: not all candidate v can start all
     * candidate literals */

    for (auto v : cand_split_v) {
        u32 rhs_min_len = min_dist_between(v, g.accept, g);
        if (rhs_min_len + min_len(cand_lit) >= succ.s.length()) {
            return false;
        }
    }

    return true; /* we are in trouble */
}

static
bool enveloped(const VertLitInfo &cand_split, const RoseInGraph &ig,
               const vdest_map_t &v_dest_map) {
    for (auto v : cand_split.vv) {
        const auto &images = v_dest_map.at(v);
        for (const auto &image : images) {
            /* check that we aren't enveloped by the successor */
            if (enveloped(vector<NFAVertex>(1, image.second), cand_split.lit,
                          *ig[image.first].graph,
                          ig[target(image.first, ig)])) {
                return true;
            }

            const RoseInVertexProps &pred = ig[source(image.first, ig)];
            if (pred.type != RIV_LITERAL) {
                continue;
            }

            /* check we don't envelop the pred */
            const NGHolder &g = *ig[image.first].graph;
            u32 lhs_min_len = min_dist_between(g.start, image.second, g);
            if (lhs_min_len + pred.s.length() < max_len(cand_split.lit)) {
                return true;
            }
        }
    }

    return false;
}

static
bool attemptSplit(RoseInGraph &ig, vdest_map_t &v_dest_map,
                  vsrc_map_t &v_src_map, const vector<RoseInEdge> &v_e,
                  LitCollection &lits) {
    NGHolder &h = *ig[v_e.front()].graph;
    unique_ptr<VertLitInfo> split = lits.pickNext();

    while (split) {
        for (const auto &e : v_e) {
            RoseInVertex t = target(e, ig);
            if (enveloped(split->vv, split->lit, h, ig[t])) {
                DEBUG_PRINTF("enveloped\n");
                split = lits.pickNext();
                goto next_split;
            }
        }
        break;
    next_split:;
    }

    if (!split) {
        return false;
    }

    for (auto v : split->vv) {
        if (edge(v, h.accept, h).second) {
            return false;
        }
    }

    DEBUG_PRINTF("saved by a bad literal\n");
    splitRoseEdge(ig, *split, v_dest_map, v_src_map);
    return true;
}

static
void appendLiteral(const ue2_literal &s, const CharReach &cr,
                   vector<ue2_literal> *out) {
    for (size_t c = cr.find_first(); c != CharReach::npos;
         c = cr.find_next(c)) {
        bool nocase = ourisalpha(c) && cr.test(mytoupper(c))
            && cr.test(mytolower(c));

        if (nocase && (char)c == mytolower(c)) {
            continue; /* uppercase already handled us */
        }

        out->push_back(s);
        out->back().push_back(c, nocase);
    }
}

static
bool findAnchoredLiterals(const NGHolder &g, vector<ue2_literal> *out,
                          vector<NFAVertex> *pivots_out) {

    DEBUG_PRINTF("trying for anchored\n");
#define MAX_ANCHORED_LITERALS 30
#define MAX_ANCHORED_LITERAL_LEN 30

    /* TODO: this could be beefed up by going region-by-region but currently
     * that brings back bad memories of ng_rose. OR any AA region we can build
     * a dfa out of */
    assert(!proper_out_degree(g.startDs, g));

    vector<ue2_literal> lits;
    lits.push_back(ue2_literal());

    set<NFAVertex> curr;
    insert(&curr, adjacent_vertices(g.start, g));
    curr.erase(g.startDs);

    set<NFAVertex> old;

    if (contains(curr, g.accept) || curr.empty()) {
        DEBUG_PRINTF("surprise accept/voidness\n");
        return false;
    }

    while (!curr.empty()) {
        set<NFAVertex> next_verts;
        insert(&next_verts, adjacent_vertices(*curr.begin(), g));
        bool can_extend
            = !next_verts.empty() && !contains(next_verts, g.accept);
        CharReach cr;

        for (auto v : curr) {
            assert(!is_special(v, g));

            if (can_extend) {
                /* next verts must agree */
                set<NFAVertex> next_verts_local;
                insert(&next_verts_local, adjacent_vertices(v, g));
                can_extend = next_verts_local == next_verts;
            }

            cr |= g[v].char_reach;
        }

        if (!can_extend) {
            goto bail;
        }

        /* extend literals */
        assert(cr.any());
        vector<ue2_literal> next_lits;
        for (const auto &lit : lits) {
            appendLiteral(lit, cr, &next_lits);
            if (next_lits.size() > MAX_ANCHORED_LITERALS) {
                goto bail;
            }
        }

        assert(!next_lits.empty());
        old.swap(curr);

        if (next_lits[0].length() <= MAX_ANCHORED_LITERAL_LEN) {
            curr.swap(next_verts);
        } else {
            curr.clear();
        }

        lits.swap(next_lits);
    }
 bail:
    assert(!lits.empty());
    for (UNUSED const auto &lit : lits) {
        DEBUG_PRINTF("found anchored string: %s\n", dumpString(lit).c_str());
    }

    insert(pivots_out, pivots_out->end(), old);
    out->swap(lits);
    return !out->empty() && !out->begin()->empty();
}

static
bool tryForAnchoredImprovement(RoseInGraph &ig, RoseInEdge e) {
    vector<ue2_literal> lits;
    vector<NFAVertex> pivots;

    if (!findAnchoredLiterals(*ig[e].graph, &lits, &pivots)) {
        DEBUG_PRINTF("unable to find literals\n");
        return false;
    }
    DEBUG_PRINTF("found %zu literals to act as anchors\n", lits.size());

    RoseInVertex s = source(e, ig);
    RoseInVertex t = target(e, ig);

    assert(!ig[e].graph_lag);

    shared_ptr<NGHolder> lhs = make_shared<NGHolder>();
    shared_ptr<NGHolder> rhs = make_shared<NGHolder>();
    ue2::unordered_map<NFAVertex, NFAVertex> temp1;
    ue2::unordered_map<NFAVertex, NFAVertex> temp2;

    splitGraph(*ig[e].graph, pivots, lhs.get(), &temp1, rhs.get(), &temp2);

    for (const auto &lit : lits) {
        RoseInVertex v = add_vertex(RoseInVertexProps::makeLiteral(lit),
                                      ig);
        add_edge(s, v, RoseInEdgeProps(lhs, 0U), ig);
        add_edge(v, t, RoseInEdgeProps(rhs, 0U), ig);
    }
    remove_edge(e, ig);

    return true;
}

#define MAX_SINGLE_BYTE_ANCHORED_DIST 30

/* returns true if we should make another pass */
static
bool lastChanceImproveLHS(RoseInGraph &ig, RoseInEdge lhs,
                          const CompileContext &cc) {
    DEBUG_PRINTF("argh lhs is nasty\n");
    assert(ig[lhs].graph);

    /* customise the lhs for this literal */
    /* TODO better, don't recalc */
    if (ig[target(lhs, ig)].type == RIV_LITERAL) {
        const NGHolder &h = *ig[lhs].graph;

        /* sanitise literal on lhs */
        const ue2_literal &s = ig[target(lhs, ig)].s;

        if (!can_match(h, s, false)) {
            DEBUG_PRINTF("found bogus edge\n");
            return false;
        }

        /* see if we can build some anchored literals out of this */
        if (isAnchored(h) && tryForAnchoredImprovement(ig, lhs)) {
            return true;
        }

        unique_ptr<NGHolder> cust = cloneHolder(h);
        u32 d = removeTrailingLiteralStates(*cust, s, MO_INVALID_IDX);
        if (d == MO_INVALID_IDX) {
            DEBUG_PRINTF("found bogus edge\n");
            return false;
        }
        restoreTrailingLiteralStates(*cust, s, d);
        ig[lhs].graph = move(cust);
    }

    NGHolder &lhs_graph = *ig[lhs].graph;
    set<NFAVertex> cand;
    set<NFAVertex> cand_raw;
    getCandidatePivots(lhs_graph, &cand, &cand_raw);
    vdest_map_t v_dest_map;
    vsrc_map_t v_src_map;
    for (auto v : vertices_range(lhs_graph)) {
        v_dest_map[v].emplace_back(lhs, v);
        v_src_map[lhs].push_back(v);
    }

    vector<NFAVertexDepth> depths;
    calcDepths(lhs_graph, depths);

    /* need to ensure regions are valid before we do lit discovery */
    auto region_map = assignRegions(lhs_graph);

    vector<RoseInEdge> to_cut(1, lhs);
    DEBUG_PRINTF("see if we can get a better lhs by another cut\n");
    LitCollection lit1(lhs_graph, depths, region_map, cand, cand_raw,
                       cc.grey.minRoseLiteralLength, true, cc);
    if (attemptSplit(ig, v_dest_map, v_src_map, to_cut, lit1)) {
        return true;
    }

    if (doNetflowCut(ig, to_cut, cc.grey)) {
        return true;
    }

    DEBUG_PRINTF("eek last chance try len 1 if it creates an anchored lhs\n");
    {
        LitCollection lits(lhs_graph, depths, region_map, cand, cand_raw, 1,
                           true, cc, true);
        unique_ptr<VertLitInfo> split = lits.pickNext();

        /* TODO fix edge to accept check */
        while (split
               && (enveloped(split->vv, split->lit, lhs_graph,
                             ig[target(lhs, ig)])
                   || edge(split->vv.front(), lhs_graph.accept, lhs_graph).second
                   || !createsAnchoredLHS(lhs_graph, split->vv, depths, cc.grey,
                                          MAX_SINGLE_BYTE_ANCHORED_DIST))) {
            split = lits.pickNext();
        }

        if (split) {
            DEBUG_PRINTF("saved by a really bad literal\n");
            splitRoseEdge(ig, *split, v_dest_map, v_src_map);
            return true;
        }
    }

    return false;
}

/* returns false if nothing happened */
static
bool lastChanceImproveLHS(RoseInGraph &ig, const vector<RoseInEdge> &to_cut,
                          const CompileContext &cc) {
    DEBUG_PRINTF("argh lhses are nasty\n");

    NGHolder &lhs_graph = *ig[to_cut.front()].graph;
    set<NFAVertex> cand;
    set<NFAVertex> cand_raw;
    getCandidatePivots(lhs_graph, &cand, &cand_raw);
    vdest_map_t v_dest_map;
    vsrc_map_t v_src_map;
    for (auto v : vertices_range(lhs_graph)) {
        for (const auto &e : to_cut) {
            v_dest_map[v].emplace_back(e, v);
            v_src_map[e].push_back(v);
        }
    }

    vector<NFAVertexDepth> depths;
    calcDepths(lhs_graph, depths);

    auto region_map = assignRegions(lhs_graph);

    DEBUG_PRINTF("see if we can get a better lhs by allowing another cut\n");
    LitCollection lit1(lhs_graph, depths, region_map, cand, cand_raw,
                       cc.grey.minRoseLiteralLength, true, cc);
    if (attemptSplit(ig, v_dest_map, v_src_map, to_cut, lit1)) {
        return true;
    }

    return doNetflowCut(ig, to_cut, cc.grey);
}

static
bool improveLHS(RoseInGraph &ig, const vector<RoseInEdge> &edges,
                const CompileContext &cc) {
    bool rv = false;

    vector<RoseInVertex> src_verts;
    getSourceVerts(ig, edges, &src_verts);

    map<RoseInVertex, vector<RoseInEdge>> by_src;
    for (const auto &e : edges) {
        by_src[source(e, ig)].push_back(e);
    }

    for (auto v : src_verts) {
        const vector<RoseInEdge> &local = by_src[v];

        vector<NGHolder *> graphs;
        map<RoseInVertex, vector<RoseInEdge> > by_graph;
        for (const auto &e : local) {
            NGHolder *gp = ig[e].graph.get();
            if (!contains(by_graph, gp)) {
                graphs.push_back(gp);
            }
            by_graph[gp].push_back(e);
        }

        for (auto h : graphs) {
            const vector<RoseInEdge> &local2 = by_graph[h];
            if (local2.size() == 1) {
                rv |= lastChanceImproveLHS(ig, local2.front(), cc);
                continue;
            }

            bool lrv = lastChanceImproveLHS(ig, local2, cc);
            if (lrv) {
                rv = true;
            } else {
                for (const auto &e2 : local2) {
                    rv |= lastChanceImproveLHS(ig, e2, cc);
                }
            }
        }
    }

    return rv;
}

static
void processLHS(RoseInGraph &ig, const CompileContext &cc) {
    bool redo;
    do {
        redo = false;
        vector<RoseInEdge> to_improve;
        for (const auto &lhs : edges_range(ig)) {
            if (ig[source(lhs, ig)].type != RIV_START
                && ig[source(lhs, ig)].type != RIV_ANCHORED_START) {
                continue;
            }

            if (ig[target(lhs, ig)].type == RIV_LITERAL) {
                DEBUG_PRINTF("checking lhs->'%s'\n",
                             ig[target(lhs, ig)].s.c_str());
            } else {
                DEBUG_PRINTF("checking lhs->?\n");
            }


            /* if check if lhs is nasty */
            if (ig[target(lhs, ig)].type == RIV_ACCEPT) {
                to_improve.push_back(lhs);
                continue;
            }

            assert(ig[lhs].graph);
            const NGHolder *h = ig[lhs].graph.get();

            vector<NFAVertexDepth> depths;
            calcDepths(*h, depths);

            if (!isLHSTransient(*h, depths, cc.grey)
                && !literalIsWholeGraph(*h, ig[target(lhs, ig)].s)
                && !isLHSUsablyAnchored(*h, depths, cc.grey)) {
                to_improve.push_back(lhs);
            }
        }

        DEBUG_PRINTF("inspecting %zu lhs\n", to_improve.size());
        if (to_improve.size() > 50) {
            DEBUG_PRINTF("too big\n");
            break;
        }

        redo = improveLHS(ig, to_improve, cc);
        DEBUG_PRINTF("redo = %d\n", (int)redo);
    } while (redo);

    vector<RoseInEdge> to_inspect; /* to prevent surprises caused by us
                                    * altering the graph while iterating */
    for (const auto &e : edges_range(ig)) {
        if (ig[source(e, ig)].type == RIV_START
            || ig[source(e, ig)].type == RIV_ANCHORED_START) {
            to_inspect.push_back(e);
        }
    }

    for (const auto &lhs : to_inspect) {
        if (ig[target(lhs, ig)].type == RIV_LITERAL) {
            if (removeLiteralFromLHS(ig, lhs, cc)) {
                handleLhsCliche(ig, lhs);
            } else {
                /* telling us to delete the edge */
                remove_edge(lhs, ig);
            }
        }
    }
}

static
void tryNetflowCutForRHS(RoseInGraph &ig, const Grey &grey) {
    vector<RoseInEdge> to_improve;
    for (const auto &rhs : edges_range(ig)) {
        if (ig[target(rhs, ig)].type != RIV_ACCEPT) {
            continue;
        }

        if (ig[source(rhs, ig)].type == RIV_LITERAL) {
            DEBUG_PRINTF("checking '%s'->rhs\n", ig[source(rhs, ig)].s.c_str());
        } else {
            DEBUG_PRINTF("checking ?->rhs\n");
        }

        if (!ig[rhs].graph) {
            continue;
        }

        DEBUG_PRINTF("%zu vertices\n", num_vertices(*ig[rhs].graph));
        if (num_vertices(*ig[rhs].graph) < 512) {
            DEBUG_PRINTF("small\n");
            continue;
        }

        /* if check if rhs is nasty */
        to_improve.push_back(rhs);
    }

    DEBUG_PRINTF("inspecting %zu lhs\n", to_improve.size());
    if (to_improve.size() > 50) {
        DEBUG_PRINTF("too big\n");
        return;
    }

    for (const auto &e : to_improve) {
        vector<RoseInEdge> to_cut(1, e);
        doNetflowCut(ig, to_cut, grey);
    }
}

/* just make the string nocase and get the graph to handle case mask, TODO.
 * This could be more nuanced but the effort would probably be better spent
 * just making rose less bad. */
static
void makeNocaseWithPrefixMask(RoseInGraph &g, RoseInVertex v) {
    for (const auto &e : in_edges_range(v, g)) {
        const RoseInVertex u = source(e, g);

        if (!g[e].graph) {
            g[e].graph = make_shared<NGHolder>(whatRoseIsThis(g, e));
            g[e].graph_lag = g[v].s.length();
            NGHolder &h = *g[e].graph;

            assert(!g[e].maxBound || g[e].maxBound == ROSE_BOUND_INF);

            if (g[u].type == RIV_START) {
                add_edge(h.startDs, h.accept, h);
                h[h.startDs].reports.insert(0);
            } else if (g[e].maxBound == ROSE_BOUND_INF) {
                add_edge(h.start, h.accept, h);
                NFAVertex ds = add_vertex(h);

                h[ds].char_reach = CharReach::dot();

                add_edge(h.start, ds, h);
                add_edge(ds, ds, h);
                add_edge(ds, h.accept, h);
                h[h.start].reports.insert(0);
                h[ds].reports.insert(0);
            } else {
                add_edge(h.start, h.accept, h);
                h[h.start].reports.insert(0);
            }
        }

        if (!g[e].graph_lag) {
            continue;
        }
        unique_ptr<NGHolder> newg = cloneHolder(*g[e].graph);
        restoreTrailingLiteralStates(*newg, g[v].s, g[e].graph_lag);
        g[e].graph_lag = 0;
        g[e].graph = move(newg);
    }

    make_nocase(&g[v].s);
}

static
unique_ptr<NGHolder> makeGraphCopy(const NGHolder *g) {
    if (g) {
        return cloneHolder(*g);
    } else {
        return nullptr;
    }
}

static
void explodeLiteral(RoseInGraph &g, RoseInVertex v,
                    vector<ue2_literal> &exploded) {
    for (const auto &lit : exploded) {
        RoseInVertex v_new = add_vertex(g[v], g);
        g[v_new].s = lit;

        for (const auto &e : in_edges_range(v, g)) {
            RoseInEdge e2 = add_edge(source(e, g), v_new, g[e], g).first;
            // FIXME: are we safe to share graphs here? For now, make our very
            // own copy.
            g[e2].graph = makeGraphCopy(g[e].graph.get());
        }

        for (const auto &e : out_edges_range(v, g)) {
            RoseInEdge e2 = add_edge(v_new, target(e, g), g[e], g).first;
            // FIXME: are we safe to share graphs here? For now, make our very
            // own copy.
            g[e2].graph = makeGraphCopy(g[e].graph.get());
        }
    }

    clear_vertex(v, g);
    remove_vertex(v, g);
}

/* Sadly rose is hacky in terms of mixed case literals. TODO: remove when rose
 * becomes less bad */
static
void handleLongMixedSensitivityLiterals(RoseInGraph &g) {
    const size_t maxExploded = 8; // only case-explode this far

    vector<RoseInVertex> verts;

    for (auto v : vertices_range(g)) {
        if (g[v].type != RIV_LITERAL) {
            continue;
        }

        ue2_literal &s = g[v].s;

        if (!mixed_sensitivity(s)) {
            continue;
        }

        if (s.length() < MAX_MASK2_WIDTH) {
            DEBUG_PRINTF("mixed lit will be handled by benefits mask\n");
            continue;
        }

        DEBUG_PRINTF("found mixed lit of len %zu\n", s.length());
        verts.push_back(v);
    }

    for (auto v : verts) {
        vector<ue2_literal> exploded;
        case_iter cit = caseIterateBegin(g[v].s), cite = caseIterateEnd();
        for (; cit != cite; ++cit) {
            exploded.emplace_back(*cit, false);
            if (exploded.size() > maxExploded) {
                goto dont_explode;
            }
        }

        DEBUG_PRINTF("exploding literal into %zu pieces\n", exploded.size());
        explodeLiteral(g, v, exploded);
        continue;

    dont_explode:
        DEBUG_PRINTF("converting to nocase with prefix mask\n");
        makeNocaseWithPrefixMask(g, v);
    }

    DEBUG_PRINTF("done!\n");
}

static
void dedupe(RoseInGraph &g) {
    /* We know that every prefix/infix is unique after the rose construction.
     *
     * If a vertex has out-going graphs with the same rewind and they are equal
     * we can dedupe the graph.
     *
     * After this, we may share graphs on out-edges of a vertex. */
    map<pair<u32, u64a>, vector<shared_ptr<NGHolder>>> buckets;

    for (auto v : vertices_range(g)) {
        buckets.clear();

        for (const auto &e : out_edges_range(v, g)) {
            if (!g[e].graph || g[target(e, g)].type != RIV_LITERAL) {
                continue;
            }
            auto k = make_pair(g[e].graph_lag, hash_holder(*g[e].graph));
            auto &bucket = buckets[k];
            for (const auto &h : bucket) {
                if (is_equal(*g[e].graph, 0U, *h, 0U)) {
                    g[e].graph = h;
                    goto next_edge;
                }
            }

            bucket.push_back(g[e].graph);
        next_edge:;
        }
    }
}

static
bool pureReport(NFAVertex v, const NGHolder &g) {
    for (auto w : adjacent_vertices_range(v, g)) {
        if (w != g.accept && w != g.acceptEod) {
            return false;
        }
    }
    return true;
}

static
bool pureReport(const vector<NFAVertex> &vv, const NGHolder &g) {
    for (auto v : vv) {
        if (!pureReport(v, g)) {
            return false;
        }
    }

    return true;
}

/* ensures that a vertex is followed by a start construct AND the cyclic states
 * has a reasonably wide reach */
static
bool followedByStar(NFAVertex v, const NGHolder &g) {
    set<NFAVertex> succ;
    insert(&succ, adjacent_vertices(v, g));

    set<NFAVertex> asucc;

    for (auto w : adjacent_vertices_range(v, g)) {
        if (g[w].char_reach.count() < N_CHARS - MAX_ESCAPE_CHARS) {
            continue; /* state is too narrow to be considered as a sane star
                         cyclic */
        }

        asucc.clear();
        insert(&asucc, adjacent_vertices(w, g));

        if (asucc == succ) {
            return true;
        }
    }
    return false;
}

static
bool followedByStar(const vector<NFAVertex> &vv, const NGHolder &g) {
    for (auto v : vv) {
        if (!followedByStar(v, g)) {
            return false;
        }
    }

    return true;
}

static
bool isEodPrefixCandidate(const NGHolder &g) {
    if (hasGreaterInDegree(0, g.accept, g)) {
        DEBUG_PRINTF("graph isn't eod anchored\n");
        return false;
    }

    // TODO: handle more than one report.
    if (all_reports(g).size() != 1) {
        return false;
    }

    return true;
}


static
bool isEodWithPrefix(const RoseInGraph &g) {
    if (num_vertices(g) != 2) {
        return false;
    }

    for (const auto &e : edges_range(g)) {
        RoseInVertex u = source(e, g), v = target(e, g);
        DEBUG_PRINTF("edge from %d -> %d\n", g[u].type, g[v].type);

        if (g[u].type != RIV_START && g[u].type != RIV_ANCHORED_START) {
            DEBUG_PRINTF("source not start, type=%d\n", g[u].type);
            return false;
        }

        if (g[v].type != RIV_ACCEPT && g[v].type != RIV_ACCEPT_EOD) {
            DEBUG_PRINTF("target not accept, type=%d\n", g[v].type);
            return false;
        }

        // Haigs not handled.
        if (g[e].haig) {
            DEBUG_PRINTF("edge has haig\n");
            return false;
        }

        if (!g[e].graph) {
            DEBUG_PRINTF("no graph on edge\n");
            return false;
        }

        if (!isEodPrefixCandidate(*g[e].graph)) {
            DEBUG_PRINTF("graph is not eod prefix candidate\n");
            return false;
        }
    }

    return true;
}

static
void processEodPrefixes(RoseInGraph &g) {
    // Find edges to accept with EOD-anchored graphs that we can move over to
    // acceptEod.
    vector<RoseInEdge> acc_edges;
    for (const auto &e : edges_range(g)) {
        if (g[target(e, g)].type != RIV_ACCEPT) {
            continue;
        }
        if (g[e].haig || !g[e].graph) {
            continue;
        }
        if (!isEodPrefixCandidate(*g[e].graph)) {
            continue;
        }

        // TODO: handle cases with multiple out-edges.
        if (hasGreaterOutDegree(1, source(e, g), g)) {
            continue;
        }

        acc_edges.push_back(e);
    }

    set<RoseInVertex> accepts;

    for (const RoseInEdge &e : acc_edges) {
        RoseInVertex u = source(e, g), v = target(e, g);
        assert(g[e].graph);
        assert(g[v].type == RIV_ACCEPT);
        assert(all_reports(*g[e].graph).size() == 1);

        // Move this edge from accept to acceptEod and give it the right reports
        // from the graph on the edge.
        const set<ReportID> reports = all_reports(*g[e].graph);
        RoseInVertex w = add_vertex(
                RoseInVertexProps::makeAcceptEod(reports), g);
        add_edge(u, w, g[e], g);

        remove_edge(e, g);
        accepts.insert(v);
    }

    for (auto v : accepts) {
        if (!hasGreaterInDegree(0, v, g)) {
            remove_vertex(v, g);
        }
    }
}

/** Run some reduction passes on the graphs on our edges. */
static
void reduceGraphs(RoseInGraph &g, const CompileContext &cc) {
    for (const auto &e : edges_range(g)) {
        if (!g[e].graph) {
            continue;
        }
        NGHolder &h = *g[e].graph;
        assert(h.kind == whatRoseIsThis(g, e));
        DEBUG_PRINTF("before, graph %p has %zu vertices, %zu edges\n", &h,
                     num_vertices(h), num_edges(h));

        pruneUseless(h);

        reduceGraphEquivalences(h, cc);

        removeRedundancy(h, SOM_NONE); /* rose doesn't track som */

        DEBUG_PRINTF("after, graph %p has %zu vertices, %zu edges\n", &h,
                     num_vertices(h), num_edges(h));

        // It's possible that one of our graphs may have reduced to a dot-star
        // cliche, i.e. it contains a startDs->accept edge. If so, we can
        // remove it from the edge and just use edge bounds to represent it.
        if (edge(h.startDs, h.accept, h).second) {
            DEBUG_PRINTF("graph reduces to dot-star, deleting\n");
            g[e].graph.reset();
            g[e].graph_lag = 0;
            g[e].minBound = 0;
            g[e].maxBound = ROSE_BOUND_INF;
        }
    }
}

static
unique_ptr<RoseInGraph> buildRose(const NGHolder &h, bool desperation,
                                  const CompileContext &cc) {
    /* Need to pick a pivot point which splits the graph in two with starts on
     * one side and accepts on the other. Thus the pivot needs to dominate all
     * the accept vertices */

    /* maps a vertex in h to one of its images in the rose graph */
    vdest_map_t v_dest_map;
    vsrc_map_t v_src_map;

    /* create trivial rose graph */
    unique_ptr<RoseInGraph> igp = makeTrivialGraph(h, v_dest_map, v_src_map);
    RoseInGraph &ig = *igp;

    /* root graph is the graph on the only edge in our new RoseInGraph */
    assert(num_edges(ig) == 1);
    shared_ptr<NGHolder> root_g = ig[*edges(ig).first].graph;
    assert(root_g);

    /* find the literals */
    set<NFAVertex> cand;
    set<NFAVertex> cand_raw;
    getCandidatePivots(*root_g, &cand, &cand_raw);

    DEBUG_PRINTF("|cand| = %zu\n", cand.size());

    vector<NFAVertexDepth> depths;
    calcDepths(*root_g, depths);

    auto region_map = assignRegions(*root_g);

    LitCollection lits(*root_g, depths, region_map, cand, cand_raw,
                       cc.grey.minRoseLiteralLength, desperation, cc);

    for (u32 i = 0; i < cc.grey.roseDesiredSplit; ++i) {
        DEBUG_PRINTF("attempting split %u (desired %u)\n", i,
                     cc.grey.roseDesiredSplit);
        unique_ptr<VertLitInfo> split = lits.pickNext();

        /* need to check we aren't creating any enveloping literals */
        while (split && enveloped(*split, ig, v_dest_map)) {
            DEBUG_PRINTF("bad cand; getting next split\n");
            split = lits.pickNext();
        }

        if (!split) {
            DEBUG_PRINTF("no more lits :(\n");
            break;
        }
        splitRoseEdge(ig, *split, v_dest_map, v_src_map);
    }

    /* try for more split literals if they are followed by .* or accept */
    for (;;) {
        DEBUG_PRINTF("attempting bonus split\n");
        unique_ptr<VertLitInfo> split = lits.pickNext();

        /* need to check we aren't creating any enveloping literals */
        while (split
               && (enveloped(*split, ig, v_dest_map)
                   || (!pureReport(split->vv, *root_g)
                       && !followedByStar(split->vv, *root_g)))) {
            DEBUG_PRINTF("bad cand; getting next split\n");
            split = lits.pickNext();
        }

        if (!split) {
            DEBUG_PRINTF("no more lits :(\n");
            break;
        }
        DEBUG_PRINTF("got bonus split\n");
        splitRoseEdge(ig, *split, v_dest_map, v_src_map);
    }

    processLHS(ig, cc);

    if (num_vertices(ig) <= 2) {
        // At present, we don't accept all outfixes.
        // However, we do handle the specific case of a rose that precedes an
        // acceptEod, which we will support as a prefix to a special EOD event
        // "literal".
        if (!isEodWithPrefix(ig)) {
            igp.reset();
            return igp;
        }
    }

    processEodPrefixes(ig);

    processInfixes(ig, cc);

    handleLongMixedSensitivityLiterals(ig);

    dedupe(ig);

    pruneUseless(ig);

    reduceGraphs(ig, cc);

    dumpPreRoseGraph(ig, cc.grey);

    calcVertexOffsets(ig);
    return igp;
}

static
void desperationImprove(RoseInGraph &ig, const CompileContext &cc) {
    DEBUG_PRINTF("rose said no; can we do better?\n");

    /* infixes are tricky as we have to worry about delays, enveloping
     * literals, etc */
    tryNetflowCutForRHS(ig, cc.grey);
    processInfixes(ig, cc);

    handleLongMixedSensitivityLiterals(ig);
    dedupe(ig);
    pruneUseless(ig);
    calcVertexOffsets(ig);
}

bool splitOffRose(RoseBuild &rose, const NGHolder &h, bool prefilter,
                  const CompileContext &cc) {
    if (!cc.grey.allowRose) {
        return false;
    }

    // We should have at least one edge into accept or acceptEod!
    assert(hasGreaterInDegree(0, h.accept, h) ||
           hasGreaterInDegree(1, h.acceptEod, h));

    unique_ptr<RoseInGraph> igp = buildRose(h, false, cc);
    if (igp && rose.addRose(*igp, prefilter)) {
        goto ok;
    }

    igp = buildRose(h, true, cc);

    if (igp) {
        if (rose.addRose(*igp, prefilter)) {
            goto ok;
        }

        desperationImprove(*igp, cc);

        if (rose.addRose(*igp, prefilter)) {
            goto ok;
        }
    }

    DEBUG_PRINTF("rose build failed\n");
    return false;

ok:
    DEBUG_PRINTF("rose build ok\n");
    return true;
}

bool finalChanceRose(RoseBuild &rose, const NGHolder &h, bool prefilter,
                     const CompileContext &cc) {
    DEBUG_PRINTF("final chance rose\n");
    if (!cc.grey.allowRose) {
        return false;
    }
    assert(h.kind == NFA_OUTFIX);

    ue2_literal lit;
    bool anch = false;
    shared_ptr<NGHolder> rhs = make_shared<NGHolder>();
    if (!splitOffLeadingLiteral(h, &lit, &*rhs)) {
        DEBUG_PRINTF("no floating literal\n");
        anch = true;
        if (!splitOffAnchoredLeadingLiteral(h, &lit, &*rhs)) {
            DEBUG_PRINTF("no anchored literal\n");
            return false;
        }
    }

    if (lit.length() < cc.grey.minRoseLiteralLength
        || minStringPeriod(lit) < 2 ) {
        DEBUG_PRINTF("lit too weak\n");
        return false;
    }

    assert(lit.length() <= MAX_MASK2_WIDTH || !mixed_sensitivity(lit));

    RoseInGraph ig;
    RoseInVertex s
        = add_vertex(RoseInVertexProps::makeStart(anch), ig);
    RoseInVertex v = add_vertex(RoseInVertexProps::makeLiteral(lit), ig);
    add_edge(s, v, RoseInEdgeProps(0, anch ? 0 : ROSE_BOUND_INF), ig);

    ue2_literal lit2;
    if (getTrailingLiteral(h, &lit2)
        && lit2.length() >= cc.grey.minRoseLiteralLength
        && minStringPeriod(lit2) >= 2) {

        /* TODO: handle delay */
        size_t overlap = maxOverlap(lit, lit2, 0);
        u32 delay2 = lit2.length() - overlap;
        delay2 = min(delay2, maxDelay(cc));
        delay2 = removeTrailingLiteralStates(*rhs, lit2, delay2);
        rhs->kind = NFA_INFIX;
        assert(delay2 <= lit2.length());

        RoseInVertex w
            = add_vertex(RoseInVertexProps::makeLiteral(lit2), ig);
        add_edge(v, w, RoseInEdgeProps(rhs, delay2), ig);

        NFAVertex reporter = getSoleSourceVertex(h, h.accept);
        assert(reporter);
        const auto &reports = h[reporter].reports;
        RoseInVertex a =
            add_vertex(RoseInVertexProps::makeAccept(reports), ig);
        add_edge(w, a, RoseInEdgeProps(0U, 0U), ig);
    } else {
        RoseInVertex a =
            add_vertex(RoseInVertexProps::makeAccept(set<ReportID>()), ig);
        add_edge(v, a, RoseInEdgeProps(rhs, 0U), ig);
    }

    calcVertexOffsets(ig);

    return rose.addRose(ig, prefilter, true /* final chance */);
}

bool checkRose(const ReportManager &rm, const NGHolder &h, bool prefilter,
               const CompileContext &cc) {
    if (!cc.grey.allowRose) {
        return false;
    }

    // We should have at least one edge into accept or acceptEod!
    assert(hasGreaterInDegree(0, h.accept, h) ||
           hasGreaterInDegree(1, h.acceptEod, h));

    unique_ptr<RoseInGraph> igp;

    // First pass.

    igp = buildRose(h, false, cc);
    if (igp && roseCheckRose(*igp, prefilter, rm, cc)) {
        return true;
    }

    // Second ("desperation") pass.

    igp = buildRose(h, true, cc);
    if (igp) {
        if (roseCheckRose(*igp, prefilter, rm, cc)) {
            return true;
        }

        desperationImprove(*igp, cc);

        if (roseCheckRose(*igp, prefilter, rm, cc)) {
            return true;
        }
    }

    return false;
}

} // namespace ue2
