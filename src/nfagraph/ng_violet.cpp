/*
 * Copyright (c) 2016-2018, Intel Corporation
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

#include "ng_violet.h"

#include "grey.h"
#include "ng_depth.h"
#include "ng_dominators.h"
#include "ng_dump.h"
#include "ng_equivalence.h"
#include "ng_holder.h"
#include "ng_is_equal.h"
#include "ng_literal_analysis.h"
#include "ng_limex.h"
#include "ng_mcclellan.h"
#include "ng_netflow.h"
#include "ng_prune.h"
#include "ng_redundancy.h"
#include "ng_region.h"
#include "ng_reports.h"
#include "ng_split.h"
#include "ng_util.h"
#include "ng_width.h"
#include "nfa/rdfa.h"
#include "rose/rose_build.h"
#include "rose/rose_build_util.h"
#include "rose/rose_in_dump.h"
#include "rose/rose_in_graph.h"
#include "rose/rose_in_util.h"
#include "util/compare.h"
#include "util/compile_context.h"
#include "util/container.h"
#include "util/flat_containers.h"
#include "util/graph.h"
#include "util/graph_range.h"
#include "util/graph_small_color_map.h"
#include "util/insertion_ordered.h"
#include "util/make_unique.h"
#include "util/order_check.h"
#include "util/target_info.h"
#include "util/ue2string.h"

#include <set>
#include <utility>
#include <vector>
#include <boost/dynamic_bitset.hpp>
#include <boost/range/adaptor/map.hpp>

#define STAGE_DEBUG_PRINTF DEBUG_PRINTF

using namespace std;
using boost::adaptors::map_values;

namespace ue2 {

/* createsAnchoredLHS() is conservative as the depths take into account
 * back edges that come from beyond the split point and would be missing after
 * the graph is split. */
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

/* createsTransientLHS() is conservative as the depths take into account
 * back edges that come from beyond the split point and would be missing after
 * the graph is split. */
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

/**
 * Counts the number of vertices that are reachable from the set of sources
 * given.
 */
static
size_t count_reachable(const NGHolder &g, const vector<NFAVertex> &sources,
                small_color_map<decltype(get(vertex_index, g))> &color_map) {
    auto null_visitor = boost::make_dfs_visitor(boost::null_visitor());
    color_map.fill(small_color::white);

    for (auto v : sources) {
        boost::depth_first_visit(g, v, null_visitor, color_map);
    }

    return color_map.count(small_color::black);
}

static
size_t shorter_than(const set<ue2_literal> &s, size_t limit) {
    return count_if(s.begin(), s.end(),
                    [&](const ue2_literal &a) { return a.length() < limit; });
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
u32 min_period(const set<ue2_literal> &s) {
    u32 rv = ~0U;

    for (const auto &lit : s) {
        rv = min(rv, (u32)minStringPeriod(lit));
    }
    DEBUG_PRINTF("min period %u\n", rv);
    return rv;
}

namespace {
/**
 * Information on a cut: vertices and literals.
 */
struct VertLitInfo {
    VertLitInfo() {}
    VertLitInfo(NFAVertex v, const set<ue2_literal> &litlit, bool c_anch,
                bool c_tran = false)
        : vv(vector<NFAVertex>(1, v)), lit(litlit), creates_anchored(c_anch),
          creates_transient(c_tran) {}
    VertLitInfo(const vector<NFAVertex> &vv_in, const set<ue2_literal> &lit_in,
                bool c_anch)
        : vv(vv_in), lit(lit_in), creates_anchored(c_anch) {}
    vector<NFAVertex> vv;
    set<ue2_literal> lit;

    bool creates_anchored = false;
    bool creates_transient = false;
    double split_ratio = 0;
};

#define LAST_CHANCE_STRONG_LEN 1

/**
 * \brief Comparator class for comparing different literal cuts.
 */
class LitComparator {
public:
    LitComparator(const NGHolder &g_in, bool sa, bool st, bool lc)
        : g(g_in), seeking_anchored(sa), seeking_transient(st),
          last_chance(lc) {}
    bool operator()(const unique_ptr<VertLitInfo> &a,
                    const unique_ptr<VertLitInfo> &b) const {
        assert(a && b);

        if (seeking_anchored) {
            if (a->creates_anchored != b->creates_anchored) {
                return a->creates_anchored < b->creates_anchored;
            }
        }

        if (seeking_transient) {
            if (a->creates_transient != b->creates_transient) {
                return a->creates_transient < b->creates_transient;
            }
        }

        if (last_chance
            && min_len(a->lit) > LAST_CHANCE_STRONG_LEN
            && min_len(b->lit) > LAST_CHANCE_STRONG_LEN) {
            DEBUG_PRINTF("using split ratio %g , %g\n", a->split_ratio,
                          b->split_ratio);
            return a->split_ratio < b->split_ratio;
        }

        u64a score_a = scoreSet(a->lit);
        u64a score_b = scoreSet(b->lit);

        if (score_a != score_b) {
            return score_a > score_b;
        }

        /* vertices should only be in one candidate cut */
        assert(a->vv == b->vv || a->vv.front() != b->vv.front());
        return g[a->vv.front()].index > g[b->vv.front()].index;
    }

private:
    const NGHolder &g; /**< graph on which cuts are found */

    bool seeking_anchored;
    bool seeking_transient;
    bool last_chance;
};
}

#define MIN_ANCHORED_LEN 2
#define MIN_ANCHORED_DESPERATE_LEN 1

/* anchored here means that the cut creates a 'usefully' anchored LHS */
static
bool validateRoseLiteralSetQuality(const set<ue2_literal> &s, u64a score,
                                   bool anchored, u32 min_allowed_floating_len,
                                   bool desperation, bool last_chance) {
    u32 min_allowed_len = anchored ? MIN_ANCHORED_LEN
                                   : min_allowed_floating_len;
    if (anchored && last_chance) {
        min_allowed_len = MIN_ANCHORED_DESPERATE_LEN;
    }
    if (last_chance) {
        desperation = true;
    }

    DEBUG_PRINTF("validating%s set, min allowed len %u\n",
                 anchored ? " anchored" : "", min_allowed_len);

    assert(none_of(begin(s), end(s), bad_mixed_sensitivity));

    if (score >= NO_LITERAL_AT_EDGE_SCORE) {
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
        || (s_min_period <= 1 && min_allowed_len != 1)) {
        DEBUG_PRINTF("candidate may be bad\n");
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
        DEBUG_PRINTF("candidate is too shitty\n");
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
void getSimpleRoseLiterals(const NGHolder &g, bool seeking_anchored,
                           const vector<NFAVertexDepth> *depths,
                           const set<NFAVertex> &a_dom,
                           vector<unique_ptr<VertLitInfo>> *lits,
                           u32 min_allowed_len, bool desperation,
                           bool last_chance, const CompileContext &cc) {
    assert(depths || !seeking_anchored);

    map<NFAVertex, u64a> scores;
    map<NFAVertex, unique_ptr<VertLitInfo>> lit_info;
    set<ue2_literal> s;

    for (auto v : a_dom) {
        s = getLiteralSet(g, v, true); /* RHS will take responsibility for any
                                          revisits to the target vertex */

        if (s.empty()) {
            DEBUG_PRINTF("candidate is too shitty\n");
            continue;
        }

        DEBUG_PRINTF("|candidate raw literal set| = %zu\n", s.size());
        dumpRoseLiteralSet(s);
        u64a score = sanitizeAndCompressAndScore(s);

        bool anchored = false;
        if (seeking_anchored) {
            anchored = createsAnchoredLHS(g, {v}, *depths, cc.grey);
        }

        if (!validateRoseLiteralSetQuality(s, score, anchored, min_allowed_len,
                                           desperation, last_chance)) {
            continue;
        }

        DEBUG_PRINTF("candidate is a candidate\n");
        scores[v] = score;
        lit_info[v] = make_unique<VertLitInfo>(v, s, anchored);
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
void getRegionRoseLiterals(const NGHolder &g, bool seeking_anchored,
                           const vector<NFAVertexDepth> *depths,
                           const set<NFAVertex> &bad,
                           const set<NFAVertex> *allowed,
                           vector<unique_ptr<VertLitInfo>> *lits,
                           u32 min_allowed_len, bool desperation,
                           bool last_chance, const CompileContext &cc) {
    /* This allows us to get more places to split the graph as we are not
       limited to points where there is a single vertex to split at. */

    assert(depths || !seeking_anchored);

    /* TODO: operate over 'proto-regions' which ignore back edges */
    auto regions = assignRegions(g);

    set<u32> mand, optional;
    map<u32, vector<NFAVertex> > exits;

    for (auto v : vertices_range(g)) {
        u32 region = regions[v];
        if (is_any_start(v, g) || region == 0) {
            continue;
        }

        if (is_any_accept(v, g)) {
            continue;
        }

        if (!generates_callbacks(g) && is_match_vertex(v, g)) {
            /* we cannot leave a completely vacuous infix */
            continue;
        }

        if (isRegionExit(g, v, regions)) {
            exits[region].push_back(v);
        }

        if (isRegionEntry(g, v, regions)) {
            // Determine whether this region is mandatory or optional. We only
            // need to do this check for the first entry vertex we encounter
            // for this region.
            if (!contains(mand, region) && !contains(optional, region)) {
                if (isOptionalRegion(g, v, regions)) {
                    optional.insert(region);
                } else {
                    mand.insert(region);
                }
            }
        }
    }

    for (const auto &m : exits) {
        if (false) {
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
             /* if an exit is in bad, the region is already handled well
              * by getSimpleRoseLiterals or is otherwise bad */
            if (contains(bad, v)) {
                goto next_cand;
            }
            /* if we are only allowed to consider some vertices, v must be in
               the list; */
            if (allowed && !contains(*allowed, v)) {
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
            DEBUG_PRINTF("   exit vertex: %zu\n", g[v].index);
            /* Note: RHS can not be depended on to take all subsequent revisits
             * to this vertex */
            set<ue2_literal> ss = getLiteralSet(g, v, false);
            if (ss.empty()) {
                DEBUG_PRINTF("candidate is too shitty\n");
                goto next_cand;
            }
            insert(&s, ss);
        }

        assert(!s.empty());

        DEBUG_PRINTF("|candidate raw literal set| = %zu\n", s.size());
        dumpRoseLiteralSet(s);
        u64a score = sanitizeAndCompressAndScore(s);

        DEBUG_PRINTF("|candidate literal set| = %zu\n", s.size());
        dumpRoseLiteralSet(s);

        bool anchored = false;
        if (seeking_anchored) {
            anchored = createsAnchoredLHS(g, vv, *depths, cc.grey);
        }

        if (!validateRoseLiteralSetQuality(s, score, anchored, min_allowed_len,
                                           desperation, last_chance)) {
            goto next_cand;
        }

        DEBUG_PRINTF("candidate is a candidate\n");
        lits->push_back(make_unique<VertLitInfo>(vv, s, anchored));
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
    auto dominators = findDominators(g);

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
unique_ptr<VertLitInfo> findBestSplit(const NGHolder &g,
                                      const vector<NFAVertexDepth> *depths,
                                      bool for_prefix, u32 min_len,
                                      const set<NFAVertex> *allowed_cand,
                                      const set<NFAVertex> *disallowed_cand,
                                      bool last_chance,
                                      const CompileContext &cc) {
    assert(!for_prefix || depths);

    /* look for a single simple split point */
    set<NFAVertex> cand;
    set<NFAVertex> cand_raw;

    getCandidatePivots(g, &cand, &cand_raw);

    if (allowed_cand) {
        set<NFAVertex> cand2;
        set<NFAVertex> cand2_raw;
        set_intersection(allowed_cand->begin(), allowed_cand->end(),
                         cand.begin(), cand.end(),
                         inserter(cand2, cand2.begin()));

        set_intersection(allowed_cand->begin(), allowed_cand->end(),
                         cand_raw.begin(), cand_raw.end(),
                         inserter(cand2_raw, cand2_raw.begin()));

        cand = std::move(cand2);
        cand_raw = std::move(cand2_raw);
    }
    if (disallowed_cand) {
        DEBUG_PRINTF("%zu disallowed candidates\n", disallowed_cand->size());
        DEBUG_PRINTF("|old cand| = %zu\n", cand.size());
        erase_all(&cand, *disallowed_cand);
        insert(&cand_raw, *disallowed_cand);
    }

    if (!generates_callbacks(g)) {
        /* not output exposed so must leave some RHS */
        for (NFAVertex v : inv_adjacent_vertices_range(g.accept, g)) {
            cand.erase(v);
            cand_raw.erase(v);
        }

        for (NFAVertex v : inv_adjacent_vertices_range(g.acceptEod, g)) {
            cand.erase(v);
            cand_raw.erase(v);
        }
    }

    DEBUG_PRINTF("|cand| = %zu\n", cand.size());

    bool seeking_anchored = for_prefix;
    bool seeking_transient = for_prefix;

    bool desperation = for_prefix && cc.streaming;

    vector<unique_ptr<VertLitInfo>> lits; /**< sorted list of potential cuts */

    getSimpleRoseLiterals(g, seeking_anchored, depths, cand, &lits, min_len,
                          desperation, last_chance, cc);
    getRegionRoseLiterals(g, seeking_anchored, depths, cand_raw, allowed_cand,
                          &lits, min_len, desperation, last_chance, cc);

    if (lits.empty()) {
        DEBUG_PRINTF("no literals found\n");
        return nullptr;
    }

    if (seeking_transient) {
        for (auto &a : lits) {
            a->creates_transient
                = createsTransientLHS(g, a->vv, *depths, cc.grey);
        }
    }

    if (last_chance) {
        const size_t num_verts = num_vertices(g);
        auto color_map = make_small_color_map(g);
        for (auto &a : lits) {
            size_t num_reachable = count_reachable(g, a->vv, color_map);
            double ratio = (double)num_reachable / (double)num_verts;
            a->split_ratio = ratio > 0.5 ? 1 - ratio : ratio;
        }
    }

    auto cmp = LitComparator(g, seeking_anchored, seeking_transient,
                             last_chance);

    unique_ptr<VertLitInfo> best = move(lits.back());
    lits.pop_back();
    while (!lits.empty()) {
        if (cmp(best, lits.back())) {
            best = move(lits.back());
        }
        lits.pop_back();
    }

    DEBUG_PRINTF("best is '%s' %zu a%d t%d\n",
        dumpString(*best->lit.begin()).c_str(),
        g[best->vv.front()].index,
        depths ? (int)createsAnchoredLHS(g, best->vv, *depths, cc.grey) : 0,
        depths ? (int)createsTransientLHS(g, best->vv, *depths, cc.grey) : 0);

    return best;
}

static
void poisonFromSuccessor(const NGHolder &h, const ue2_literal &succ,
                         bool overhang_ok, flat_set<NFAEdge> &bad) {
    DEBUG_PRINTF("poisoning holder of size %zu, succ len %zu\n",
                 num_vertices(h), succ.length());

    using EdgeSet = boost::dynamic_bitset<>;

    const size_t edge_count = num_edges(h);
    EdgeSet bad_edges(edge_count);

    unordered_map<NFAVertex, EdgeSet> curr;
    for (const auto &e : in_edges_range(h.accept, h)) {
        auto &path_set = curr[source(e, h)];
        if (path_set.empty()) {
            path_set.resize(edge_count);
        }
        path_set.set(h[e].index);
    }

    unordered_map<NFAVertex, EdgeSet> next;
    for (auto it = succ.rbegin(); it != succ.rend(); ++it) {
        for (const auto &path : curr) {
            NFAVertex u = path.first;
            const auto &path_set = path.second;
            if (u == h.start && overhang_ok) {
                DEBUG_PRINTF("poisoning early %zu [overhang]\n",
                             path_set.count());
                bad_edges |= path_set;
                continue;
            }
            if (overlaps(h[u].char_reach, *it)) {
                for (const auto &e : in_edges_range(u, h)) {
                    auto &new_path_set = next[source(e, h)];
                    if (new_path_set.empty()) {
                        new_path_set.resize(edge_count);
                    }
                    new_path_set |= path_set;
                    new_path_set.set(h[e].index);
                }
            }
        }
        DEBUG_PRINTF("succ char matches at %zu paths\n", next.size());
        assert(overhang_ok || !curr.empty());
        swap(curr, next);
        next.clear();
    }

    assert(overhang_ok || !curr.empty());
    for (const auto &path : curr) {
        bad_edges |= path.second;
        DEBUG_PRINTF("poisoning %zu vertices\n", path.second.count());
    }

    for (const auto &e : edges_range(h)) {
        if (bad_edges.test(h[e].index)) {
            bad.insert(e);
        }
    }
}

static
void poisonForGoodPrefix(const NGHolder &h,
                         const vector<NFAVertexDepth> &depths,
                         flat_set<NFAEdge> &bad, const Grey &grey) {
    for (const auto &v : vertices_range(h)) {
        if (!createsAnchoredLHS(h, {v}, depths, grey)
            && !createsTransientLHS(h, {v}, depths, grey)) {
            insert(&bad, in_edges_range(v, h));
        }
    }
}

static UNUSED
bool is_any_accept_type(RoseInVertexType t) {
    return t == RIV_ACCEPT || t == RIV_ACCEPT_EOD;
}

static
flat_set<NFAEdge> poisonEdges(const NGHolder &h,
                         const vector<NFAVertexDepth> *depths,
                         const RoseInGraph &vg, const vector<RoseInEdge> &ee,
                         bool for_prefix, const Grey &grey) {
    DEBUG_PRINTF("poisoning edges %zu successor edges\n", ee.size());

    /* poison edges covered by successor literal */

    set<pair<ue2_literal, bool> > succs;
    for (const RoseInEdge &ve : ee) {
        if (vg[target(ve, vg)].type != RIV_LITERAL) {
            /* nothing to poison in suffixes/outfixes */
            assert(generates_callbacks(h));
            assert(is_any_accept_type(vg[target(ve, vg)].type));
            continue;
        }
        succs.insert({vg[target(ve, vg)].s,
                    vg[source(ve, vg)].type == RIV_LITERAL});

    }

    DEBUG_PRINTF("poisoning edges %zu successor literals\n", succs.size());

    flat_set<NFAEdge> bad;
    for (const auto &p : succs) {
        poisonFromSuccessor(h, p.first, p.second, bad);
    }

    /* poison edges which don't significantly improve a prefix */

    if (for_prefix) {
        poisonForGoodPrefix(h, *depths, bad, grey);
    }

    return bad;
}

static
set<NFAVertex> poisonVertices(const NGHolder &h, const RoseInGraph &vg,
                              const vector<RoseInEdge> &ee, const Grey &grey) {
    flat_set<NFAEdge> bad_edges = poisonEdges(h, nullptr, vg, ee, false, grey);
    set<NFAVertex> bad_vertices;
    for (const NFAEdge &e : bad_edges) {
        bad_vertices.insert(target(e, h));
        DEBUG_PRINTF("bad: %zu->%zu\n", h[source(e, h)].index,
                     h[target(e, h)].index);
    }

    return bad_vertices;
}

static
unique_ptr<VertLitInfo> findBestNormalSplit(const NGHolder &g,
                                            const RoseInGraph &vg,
                                            const vector<RoseInEdge> &ee,
                                            const CompileContext &cc) {
    assert(g.kind == NFA_OUTFIX || g.kind == NFA_INFIX || g.kind == NFA_SUFFIX);
    set<NFAVertex> bad_vertices = poisonVertices(g, vg, ee, cc.grey);

    return findBestSplit(g, nullptr, false, cc.grey.minRoseLiteralLength,
                         nullptr, &bad_vertices, false, cc);
}

static
unique_ptr<VertLitInfo> findBestLastChanceSplit(const NGHolder &g,
                                                const RoseInGraph &vg,
                                                const vector<RoseInEdge> &ee,
                                                const CompileContext &cc) {
    assert(g.kind == NFA_OUTFIX || g.kind == NFA_INFIX || g.kind == NFA_SUFFIX);
    set<NFAVertex> bad_vertices = poisonVertices(g, vg, ee, cc.grey);

    return findBestSplit(g, nullptr, false, cc.grey.minRoseLiteralLength,
                         nullptr, &bad_vertices, true, cc);
}

static
unique_ptr<VertLitInfo> findSimplePrefixSplit(const NGHolder &g,
                                              const CompileContext &cc) {
    DEBUG_PRINTF("looking for simple prefix split\n");
    bool anchored = !proper_out_degree(g.startDs, g);
    NFAVertex u = anchored ? g.start : g.startDs;

    if (out_degree(u, g) != 2) { /* startDs + succ */
        return nullptr;
    }

    NFAVertex v = NGHolder::null_vertex();
    for (NFAVertex t : adjacent_vertices_range(u, g)) {
        if (t != g.startDs) {
            assert(!v);
            v = t;
        }
    }
    assert(v);

    if (!anchored) {
        if (out_degree(g.start, g) > 2) {
            return nullptr;
        }
        if (out_degree(g.start, g) == 2 && !edge(g.start, v, g).second) {
            return nullptr;
        }
    }

    NFAVertex best_v = NGHolder::null_vertex();
    ue2_literal best_lit;

    u32 limit = cc.grey.maxHistoryAvailable;
    if (anchored) {
        LIMIT_TO_AT_MOST(&limit, cc.grey.maxAnchoredRegion);
    }

    ue2_literal curr_lit;
    for (u32 i = 0; i < limit; i++) {
        const auto &v_cr = g[v].char_reach;
        if (v_cr.count() == 1 || v_cr.isCaselessChar()) {
            curr_lit.push_back(v_cr.find_first(), v_cr.isCaselessChar());
        } else {
            curr_lit.clear();
        }

        if (curr_lit.length() > best_lit.length()) {
            best_lit = curr_lit;
            best_v = v;
        }

        if (out_degree(v, g) != 1) {
            break;
        }
        v = *adjacent_vertices(v, g).first;
    }

    if (best_lit.length() < cc.grey.minRoseLiteralLength) {
        return nullptr;
    }

    set<ue2_literal> best_lit_set({best_lit});
    if (bad_mixed_sensitivity(best_lit)) {
        sanitizeAndCompressAndScore(best_lit_set);
    }

    return ue2::make_unique<VertLitInfo>(best_v, best_lit_set, anchored, true);
}

static
unique_ptr<VertLitInfo> findBestPrefixSplit(const NGHolder &g,
                                        const vector<NFAVertexDepth> &depths,
                                        const RoseInGraph &vg,
                                        const vector<RoseInEdge> &ee,
                                        bool last_chance,
                                        const CompileContext &cc) {
    assert(g.kind == NFA_PREFIX || g.kind == NFA_OUTFIX);
    set<NFAVertex> bad_vertices = poisonVertices(g, vg, ee, cc.grey);
    auto rv = findBestSplit(g, &depths, true, cc.grey.minRoseLiteralLength,
                            nullptr, &bad_vertices, last_chance, cc);

    /* large back edges may prevent us identifying anchored or transient cases
     * properly - use a simple walk instead */
    if (!rv || !(rv->creates_transient || rv->creates_anchored)) {
        auto rv2 = findSimplePrefixSplit(g, cc);
        if (rv2) {
            return rv2;
        }
    }

    return rv;
}

static
unique_ptr<VertLitInfo> findBestCleanSplit(const NGHolder &g,
                                           const CompileContext &cc) {
    assert(g.kind != NFA_PREFIX);
    set<NFAVertex> cleanSplits;
    for (NFAVertex v : vertices_range(g)) {
        if (!g[v].char_reach.all() || !edge(v, v, g).second) {
            continue;
        }
        insert(&cleanSplits, inv_adjacent_vertices(v, g));
        cleanSplits.erase(v);
    }
    cleanSplits.erase(g.start);
    if (cleanSplits.empty()) {
        return nullptr;
    }
    return findBestSplit(g, nullptr, false, cc.grey.violetEarlyCleanLiteralLen,
                         &cleanSplits, nullptr, false, cc);
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
                     continue;
                }

                next.insert(u);
            }
        }

        curr.swap(next);
    }

    return !curr.empty();
}

static
bool splitRoseEdge(const NGHolder &base_graph, RoseInGraph &vg,
                   const vector<RoseInEdge> &ee, const VertLitInfo &split) {
    const vector<NFAVertex> &splitters = split.vv;
    assert(!splitters.empty());

    shared_ptr<NGHolder> lhs = make_shared<NGHolder>();
    shared_ptr<NGHolder> rhs = make_shared<NGHolder>();

    unordered_map<NFAVertex, NFAVertex> lhs_map;
    unordered_map<NFAVertex, NFAVertex> rhs_map;

    splitGraph(base_graph, splitters, lhs.get(), &lhs_map, rhs.get(), &rhs_map);
    DEBUG_PRINTF("split %s:%zu into %s:%zu + %s:%zu\n",
                 to_string(base_graph.kind).c_str(), num_vertices(base_graph),
                 to_string(lhs->kind).c_str(), num_vertices(*lhs),
                 to_string(rhs->kind).c_str(), num_vertices(*rhs));

    bool suffix = generates_callbacks(base_graph);

    if (is_triggered(base_graph)) {
        /* if we are already guarded, check if the split reduces the size of
         * the problem before continuing with the split */
        if (num_vertices(*lhs) >= num_vertices(base_graph)
            && !(suffix && isVacuous(*rhs))) {
            DEBUG_PRINTF("split's lhs is no smaller\n");
            return false;
        }

        if (num_vertices(*rhs) >= num_vertices(base_graph)) {
            DEBUG_PRINTF("split's rhs is no smaller\n");
            return false;
        }
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

        renumber_edges(*rhs);
    }

    /* check if we still have a useful graph left over */
    bool do_norm = out_degree(rhs->start, *rhs) != 1;

    set<ReportID> splitter_reports;
    for (auto v : splitters) {
        insert(&splitter_reports, base_graph[v].reports);
    }

    /* find the targets of each source vertex; insertion_ordered_map used to
     * preserve deterministic ordering */
    insertion_ordered_map<RoseInVertex, vector<RoseInVertex>> images;
    for (const RoseInEdge &e : ee) {
        RoseInVertex src = source(e, vg);
        RoseInVertex dest = target(e, vg);
        images[src].push_back(dest);
        remove_edge(e, vg);
    }

    map<vector<RoseInVertex>, vector<RoseInVertex>> verts_by_image;

    for (const auto &m : images) {
        const auto &u = m.first;
        const auto &image = m.second;

        if (contains(verts_by_image, image)) {
            for (RoseInVertex v : verts_by_image[image]) {
                add_edge(u, v, RoseInEdgeProps(lhs, 0U), vg);
            }
            continue;
        }

        for (const auto &lit : split.lit) {
            assert(!bad_mixed_sensitivity(lit));

            /* don't allow overhang in can_match() as literals should
             * correspond to the edge graph being split; overhanging the graph
             * would indicate a false path.*/
            if (!can_match(*lhs, lit, false)) {
                DEBUG_PRINTF("'%s' did not match lhs\n",
                             escapeString(lit).c_str());
                continue;
            }

            DEBUG_PRINTF("best is '%s'\n", escapeString(lit).c_str());
            auto v = add_vertex(RoseInVertexProps::makeLiteral(lit), vg);
            add_edge(u, v, RoseInEdgeProps(lhs, 0U), vg);

            /* work out delay later */
            if (do_accept) {
                DEBUG_PRINTF("rhs has a cliche\n");
                auto tt = add_vertex(RoseInVertexProps::makeAccept(
                                                         splitter_reports), vg);
                add_edge(v, tt, RoseInEdgeProps(0U, 0U), vg);
            }

            if (do_accept_eod) {
                DEBUG_PRINTF("rhs has an eod cliche\n");
                auto tt = add_vertex(RoseInVertexProps::makeAcceptEod(
                                                         splitter_reports), vg);
                add_edge(v, tt, RoseInEdgeProps(0U, 0U), vg);
            }

            if (do_norm) {
                assert(out_degree(rhs->start, *rhs) > 1);
                for (RoseInVertex dest : image) {
                    add_edge(v, dest, RoseInEdgeProps(rhs, 0U), vg);
                }
            }
            verts_by_image[image].push_back(v);
        }
    }

    assert(hasCorrectlyNumberedVertices(*rhs));
    assert(hasCorrectlyNumberedEdges(*rhs));
    assert(isCorrectlyTopped(*rhs));
    assert(hasCorrectlyNumberedVertices(*lhs));
    assert(hasCorrectlyNumberedEdges(*lhs));
    assert(isCorrectlyTopped(*lhs));

    return true;
}

#define MAX_NETFLOW_CUT_WIDTH 40 /* magic number is magic */
#define MAX_LEN_2_LITERALS_PER_CUT 3

static
bool checkValidNetflowLits(NGHolder &h, const vector<u64a> &scores,
                           const map<NFAEdge, set<ue2_literal>> &cut_lits,
                           u32 min_allowed_length) {
    DEBUG_PRINTF("cut width %zu; min allowed %u\n", cut_lits.size(),
                 min_allowed_length);
    if (cut_lits.size() > MAX_NETFLOW_CUT_WIDTH) {
        return false;
    }

    u32 len_2_count = 0;

    for (const auto &cut : cut_lits) {
        if (scores[h[cut.first].index] >= NO_LITERAL_AT_EDGE_SCORE) {
            DEBUG_PRINTF("cut uses a forbidden edge\n");
            return false;
        }

        if (min_len(cut.second) < min_allowed_length) {
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
void splitEdgesByCut(NGHolder &h, RoseInGraph &vg,
                     const vector<RoseInEdge> &to_cut,
                     const vector<NFAEdge> &cut,
                     const map<NFAEdge, set<ue2_literal>> &cut_lits) {
    DEBUG_PRINTF("splitting %s (%zu vertices)\n", to_string(h.kind).c_str(),
                 num_vertices(h));

    /* create literal vertices and connect preds */
    unordered_set<RoseInVertex> done_sources;
    map<RoseInVertex, vector<pair<RoseInVertex, NFAVertex>>> verts_by_source;
    for (const RoseInEdge &ve : to_cut) {
        assert(&h == &*vg[ve].graph);
        RoseInVertex src = source(ve, vg);
        if (!done_sources.insert(src).second) {
            continue; /* already processed */
        }

        /* iterate over cut for determinism */
        for (const auto &e : cut) {
            NFAVertex prev_v = source(e, h);
            NFAVertex pivot = target(e, h);

            DEBUG_PRINTF("splitting on pivot %zu\n", h[pivot].index);
            unordered_map<NFAVertex, NFAVertex> temp_map;
            shared_ptr<NGHolder> new_lhs = make_shared<NGHolder>();
            splitLHS(h, pivot, new_lhs.get(), &temp_map);

            /* want to cut off paths to pivot from things other than the pivot -
             * makes a more svelte graphy */
            clear_in_edges(temp_map[pivot], *new_lhs);
            NFAEdge pivot_edge = add_edge(temp_map[prev_v], temp_map[pivot],
                                          *new_lhs);
            if (is_triggered(h) && prev_v == h.start) {
                (*new_lhs)[pivot_edge].tops.insert(DEFAULT_TOP);
            }

            pruneUseless(*new_lhs, false);
            renumber_vertices(*new_lhs);
            renumber_edges(*new_lhs);

            DEBUG_PRINTF("    into lhs %s (%zu vertices)\n",
                         to_string(new_lhs->kind).c_str(),
                         num_vertices(*new_lhs));

            assert(hasCorrectlyNumberedVertices(*new_lhs));
            assert(hasCorrectlyNumberedEdges(*new_lhs));
            assert(isCorrectlyTopped(*new_lhs));

            const set<ue2_literal> &lits = cut_lits.at(e);
            for (const auto &lit : lits) {
                if (!can_match(*new_lhs, lit, is_triggered(h))) {
                    continue;
                }

                RoseInVertex v
                    = add_vertex(RoseInVertexProps::makeLiteral(lit), vg);

                /* if this is a prefix/infix an edge directly to accept should
                 * represent a false path as we have poisoned vertices covered
                 * by the literals. */
                if (generates_callbacks(h)) {
                    if (edge(pivot, h.accept, h).second) {
                        DEBUG_PRINTF("adding acceptEod\n");
                        /* literal has a direct connection to accept */
                        const flat_set<ReportID> &reports = h[pivot].reports;
                        auto tt = add_vertex(
                                    RoseInVertexProps::makeAccept(reports), vg);
                        add_edge(v, tt, RoseInEdgeProps(0U, 0U), vg);
                    }

                    if (edge(pivot, h.acceptEod, h).second) {
                        assert(generates_callbacks(h));
                        DEBUG_PRINTF("adding acceptEod\n");
                        /* literal has a direct connection to accept */
                        const flat_set<ReportID> &reports = h[pivot].reports;
                        auto tt = add_vertex(
                                 RoseInVertexProps::makeAcceptEod(reports), vg);
                        add_edge(v, tt, RoseInEdgeProps(0U, 0U), vg);
                    }
                }

                add_edge(src, v, RoseInEdgeProps(new_lhs, 0), vg);
                verts_by_source[src].push_back({v, pivot});
            }
        }
    }

    /* wire the literal vertices up to successors */
    map<vector<NFAVertex>, shared_ptr<NGHolder> > done_rhs;
    for (const RoseInEdge &ve : to_cut) {
        RoseInVertex src = source(ve, vg);
        RoseInVertex dest = target(ve, vg);

        /* iterate over cut for determinism */
        for (const auto &elem : verts_by_source[src]) {
            NFAVertex pivot = elem.second;
            RoseInVertex v = elem.first;

            vector<NFAVertex> adj;
            insert(&adj, adj.end(), adjacent_vertices(pivot, h));
            /* we can ignore presence of accept, accepteod in adj as it is best
               effort */

            if (!contains(done_rhs, adj)) {
                unordered_map<NFAVertex, NFAVertex> temp_map;
                shared_ptr<NGHolder> new_rhs = make_shared<NGHolder>();
                splitRHS(h, adj, new_rhs.get(), &temp_map);
                remove_edge(new_rhs->start, new_rhs->accept, *new_rhs);
                remove_edge(new_rhs->start, new_rhs->acceptEod, *new_rhs);
                renumber_edges(*new_rhs);
                DEBUG_PRINTF("    into rhs %s (%zu vertices)\n",
                             to_string(new_rhs->kind).c_str(),
                             num_vertices(*new_rhs));
                done_rhs.emplace(adj, new_rhs);
                assert(isCorrectlyTopped(*new_rhs));
            }

            assert(done_rhs[adj].get());
            shared_ptr<NGHolder> new_rhs = done_rhs[adj];

            assert(hasCorrectlyNumberedVertices(*new_rhs));
            assert(hasCorrectlyNumberedEdges(*new_rhs));
            assert(isCorrectlyTopped(*new_rhs));

            if (vg[dest].type == RIV_LITERAL
                && !can_match(*new_rhs, vg[dest].s, true)) {
                continue;
            }

            if (out_degree(new_rhs->start, *new_rhs) != 1) {
                add_edge(v, dest, RoseInEdgeProps(new_rhs, 0), vg);
            }
        }

        remove_edge(ve, vg);
    }
}

static
bool doNetflowCut(NGHolder &h,
                  const vector<NFAVertexDepth> *depths,
                  RoseInGraph &vg,
                  const vector<RoseInEdge> &ee, bool for_prefix,
                  const Grey &grey, u32 min_allowed_length = 0U) {
    ENSURE_AT_LEAST(&min_allowed_length, grey.minRoseNetflowLiteralLength);

    DEBUG_PRINTF("doing netflow cut\n");
    /* TODO: we should really get literals/scores from the full graph as this
     * allows us to overlap with previous cuts. */
    assert(!ee.empty());
    assert(&h == &*vg[ee.front()].graph);
    assert(!for_prefix || depths);

    if (num_edges(h) > grey.maxRoseNetflowEdges) {
        /* We have a limit on this because scoring edges and running netflow
         * gets very slow for big graphs. */
        DEBUG_PRINTF("too many edges, skipping netflow cut\n");
        return false;
    }

    assert(hasCorrectlyNumberedVertices(h));
    assert(hasCorrectlyNumberedEdges(h));

    auto known_bad = poisonEdges(h, depths, vg, ee, for_prefix, grey);

    /* Step 1: Get scores for all edges */
    vector<u64a> scores = scoreEdges(h, known_bad); /* scores by edge_index */

    /* Step 2: Find cutset based on scores */
    vector<NFAEdge> cut = findMinCut(h, scores);

    /* Step 3: Get literals corresponding to cut edges */
    map<NFAEdge, set<ue2_literal>> cut_lits;
    for (const auto &e : cut) {
        set<ue2_literal> lits = getLiteralSet(h, e);
        sanitizeAndCompressAndScore(lits);

        cut_lits[e] = lits;
    }

    /* if literals are underlength bail or if it involves a forbidden edge*/
    if (!checkValidNetflowLits(h, scores, cut_lits, min_allowed_length)) {
        return false;
    }
    DEBUG_PRINTF("splitting\n");

    /* Step 4: Split graph based on cuts */
    splitEdgesByCut(h, vg, ee, cut, cut_lits);

    return true;
}

static
bool deanchorIfNeeded(NGHolder &g) {
    DEBUG_PRINTF("hi\n");
    if (proper_out_degree(g.startDs, g)) {
        return false;
    }

    /* look for a non-special dot with a loop following start */
    set<NFAVertex> succ_g;
    insert(&succ_g, adjacent_vertices(g.start, g));
    succ_g.erase(g.startDs);

    for (auto v : adjacent_vertices_range(g.start, g)) {
        DEBUG_PRINTF("inspecting cand %zu || = %zu\n", g[v].index,
                     g[v].char_reach.count());

        if (v == g.startDs || !g[v].char_reach.all()) {
            continue;
        }

        set<NFAVertex> succ_v;
        insert(&succ_v, adjacent_vertices(v, g));

        if (succ_v == succ_g) {
            DEBUG_PRINTF("found ^.*\n");
            for (auto succ : adjacent_vertices_range(g.start, g)) {
                if (succ == g.startDs) {
                    continue;
                }
                add_edge(g.startDs, succ, g);
            }
            clear_vertex(v, g);
            remove_vertex(v, g);
            renumber_vertices(g);
            return true;
        }

        if (succ_g.size() == 1 && hasSelfLoop(v, g)) {
            DEBUG_PRINTF("found ^.+\n");
            add_edge(g.startDs, v, g);
            remove_edge(v, v, g);
            return true;
        }
    }

    return false;
}

static
RoseInGraph populateTrivialGraph(const NGHolder &h) {
    RoseInGraph g;
    shared_ptr<NGHolder> root_g = cloneHolder(h);
    bool orig_anch = isAnchored(*root_g);
    orig_anch |= deanchorIfNeeded(*root_g);

    DEBUG_PRINTF("orig_anch %d\n", (int)orig_anch);

    auto start = add_vertex(RoseInVertexProps::makeStart(orig_anch), g);
    auto accept = add_vertex(RoseInVertexProps::makeAccept(set<ReportID>()), g);

    add_edge(start, accept, RoseInEdgeProps(root_g, 0), g);

    return g;
}

static
void avoidOutfixes(RoseInGraph &vg, bool last_chance,
                   const CompileContext &cc) {
    STAGE_DEBUG_PRINTF("AVOIDING OUTFIX\n");
    assert(num_vertices(vg) == 2);
    assert(num_edges(vg) == 1);

    RoseInEdge e = *edges(vg).first;

    NGHolder &h = *vg[e].graph;
    assert(isCorrectlyTopped(h));

    renumber_vertices(h);
    renumber_edges(h);

    unique_ptr<VertLitInfo>  split = findBestNormalSplit(h, vg, {e}, cc);

    if (split && splitRoseEdge(h, vg, {e}, *split)) {
        DEBUG_PRINTF("split on simple literal\n");
        return;
    }

    if (last_chance) {
        /* look for a prefix split as it allows us to accept very weak anchored
         * literals. */
        auto depths = calcDepths(h);

        split = findBestPrefixSplit(h, depths, vg, {e}, last_chance, cc);

        if (split && splitRoseEdge(h, vg, {e}, *split)) {
            DEBUG_PRINTF("split on simple literal\n");
            return;
        }
    }

    doNetflowCut(h, nullptr, vg, {e}, false, cc.grey);
}

static
void removeRedundantPrefixes(RoseInGraph &g) {
    STAGE_DEBUG_PRINTF("REMOVING REDUNDANT PREFIXES\n");

    for (const RoseInEdge &e : edges_range(g)) {
        RoseInVertex s = source(e, g);
        RoseInVertex t = target(e, g);

        if (g[s].type != RIV_START || g[t].type != RIV_LITERAL) {
            continue;
        }

        if (!g[e].graph) {
            continue;
        }

        assert(!g[t].delay);
        const ue2_literal &lit = g[t].s;

        if (!literalIsWholeGraph(*g[e].graph, lit)) {
            DEBUG_PRINTF("not whole graph\n");
            continue;
        }

        if (!isFloating(*g[e].graph)) {
            DEBUG_PRINTF("not floating\n");
            continue;
        }
        g[e].graph.reset();
    }
}

static
u32 maxDelay(const CompileContext &cc) {
    if (!cc.streaming) {
        return MO_INVALID_IDX;
    }
    return cc.grey.maxHistoryAvailable;
}

static
void removeRedundantLiteralsFromPrefixes(RoseInGraph &g,
                                         const CompileContext &cc) {
    STAGE_DEBUG_PRINTF("REMOVING LITERALS FROM PREFIXES\n");

    vector<RoseInEdge> to_anchor;
    for (const RoseInEdge &e : edges_range(g)) {
        RoseInVertex s = source(e, g);
        RoseInVertex t = target(e, g);

        if (g[s].type != RIV_START && g[s].type != RIV_ANCHORED_START) {
            continue;
        }

        if (g[t].type != RIV_LITERAL) {
            continue;
        }

        if (!g[e].graph) {
            continue;
        }

        if (g[e].graph_lag) {
            /* already removed redundant parts of literals */
            continue;
        }

        if (g[e].dfa) {
            /* if we removed any more states, we would need to rebuild the
             * the dfa which can be time consuming. */
            continue;
        }

        assert(!g[t].delay);
        const ue2_literal &lit = g[t].s;

        DEBUG_PRINTF("removing states for literal: %s\n",
                     dumpString(lit).c_str());

        unique_ptr<NGHolder> h = cloneHolder(*g[e].graph);
        const u32 max_delay = maxDelay(cc);

        u32 delay = removeTrailingLiteralStates(*h, lit, max_delay,
                                              false /* can't overhang start */);

        DEBUG_PRINTF("got delay %u (max allowed %u)\n", delay, max_delay);

        if (edge(h->startDs, h->accept, *h).second) {
            /* we should have delay == lit.length(), but in really complex
             * cases we may fail to identify that we can remove the whole
             * graph. Regardless, the fact that sds is wired to accept means the
             * graph serves no purpose. */
            DEBUG_PRINTF("whole graph\n");
            g[e].graph.reset();
            continue;
        }

        if (delay == lit.length() && edge(h->start, h->accept, *h).second
            && num_vertices(*h) == N_SPECIALS) {
            to_anchor.push_back(e);
            continue;
        }

        /* if we got here we should still have an interesting graph */
        assert(delay == max_delay || num_vertices(*h) > N_SPECIALS);

        if (delay && delay != MO_INVALID_IDX) {
            DEBUG_PRINTF("setting delay %u on lhs %p\n", delay, h.get());

            g[e].graph = move(h);
            g[e].graph_lag = delay;
        }
    }

    if (!to_anchor.empty()) {
        RoseInVertex anch = add_vertex(RoseInVertexProps::makeStart(true), g);

        for (RoseInEdge e : to_anchor) {
            DEBUG_PRINTF("rehoming to anchor\n");
            RoseInVertex v = target(e, g);
            add_edge(anch, v, g);
            remove_edge(e, g);
        }
    }
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
void removeRedundantLiteralsFromInfix(const NGHolder &h, RoseInGraph &ig,
                                      const vector<RoseInEdge> &ee,
                                      const CompileContext &cc) {
    /* TODO: This could be better by not creating a separate graph for each
     * successor literal. This would require using distinct report ids and also
     * taking into account overlap of successor literals. */

    set<ue2_literal> preds;
    set<ue2_literal> succs;
    for (const RoseInEdge &e : ee) {
        RoseInVertex u = source(e, ig);
        assert(ig[u].type == RIV_LITERAL);
        assert(!ig[u].delay);
        preds.insert(ig[u].s);

        RoseInVertex v = target(e, ig);
        assert(ig[v].type == RIV_LITERAL);
        assert(!ig[v].delay);
        succs.insert(ig[v].s);

        if (ig[e].graph_lag) {
            /* already removed redundant parts of literals */
            return;
        }

        assert(!ig[e].dfa);
    }

    map<ue2_literal, pair<shared_ptr<NGHolder>, u32> > graphs; /* + delay */

    for (const ue2_literal &right : succs) {
        size_t max_overlap = 0;
        for (const ue2_literal &left : preds) {
            size_t overlap = maxOverlap(left, right, 0);
            ENSURE_AT_LEAST(&max_overlap, overlap);
        }

        u32 max_allowed_delay = right.length() - max_overlap;

        if (cc.streaming) {
            LIMIT_TO_AT_MOST(&max_allowed_delay, cc.grey.maxHistoryAvailable);
        }

        if (!max_allowed_delay) {
            continue;
        }

        shared_ptr<NGHolder> h_new = cloneHolder(h);

        u32 delay = removeTrailingLiteralStates(*h_new, right,
                                                max_allowed_delay);

        if (delay == MO_INVALID_IDX) {
            /* successor literal could not match infix -> ignore false path */
            assert(0);
            continue;
        }

        if (!delay) {
            /* unable to trim graph --> no point swapping to new holder */
            continue;
        }

        assert(isCorrectlyTopped(*h_new));
        graphs[right] = make_pair(h_new, delay);
    }

    for (const RoseInEdge &e : ee) {
        RoseInVertex v = target(e, ig);
        const ue2_literal &succ = ig[v].s;
        if (!contains(graphs, succ)) {
            continue;
        }

        ig[e].graph = graphs[succ].first;
        ig[e].graph_lag = graphs[succ].second;

        if (isStarCliche(*ig[e].graph)) {
            DEBUG_PRINTF("is a X star!\n");
            ig[e].graph.reset();
            ig[e].graph_lag = 0;
        }
    }
}

static
void removeRedundantLiteralsFromInfixes(RoseInGraph &g,
                                        const CompileContext &cc) {
    insertion_ordered_map<NGHolder *, vector<RoseInEdge>> infixes;

    for (const RoseInEdge &e : edges_range(g)) {
        RoseInVertex s = source(e, g);
        RoseInVertex t = target(e, g);

        if (g[s].type != RIV_LITERAL || g[t].type != RIV_LITERAL) {
            continue;
        }

        if (!g[e].graph) {
            continue;
        }

        assert(!g[t].delay);
        if (g[e].dfa) {
            /* if we removed any more states, we would need to rebuild the
             * the dfa which can be time consuming. */
            continue;
        }

        NGHolder *h = g[e].graph.get();
        infixes[h].push_back(e);
    }

    for (const auto &m : infixes) {
        NGHolder *h = m.first;
        const auto &edges = m.second;
        removeRedundantLiteralsFromInfix(*h, g, edges, cc);
    }
}

static
void removeRedundantLiterals(RoseInGraph &g, const CompileContext &cc) {
    removeRedundantLiteralsFromPrefixes(g, cc);
    removeRedundantLiteralsFromInfixes(g, cc);
}

static
RoseInVertex getStart(RoseInGraph &vg) {
    for (RoseInVertex v : vertices_range(vg)) {
        if (vg[v].type == RIV_START || vg[v].type == RIV_ANCHORED_START) {
            return v;
        }
    }
    assert(0);
    return RoseInGraph::null_vertex();
}

/**
 * Finds the initial accept vertex created to which suffix/outfixes are
 * attached.
 */
static
RoseInVertex getPrimaryAccept(RoseInGraph &vg) {
    for (RoseInVertex v : vertices_range(vg)) {
        if (vg[v].type == RIV_ACCEPT && vg[v].reports.empty()) {
            return v;
        }
    }
    assert(0);
    return RoseInGraph::null_vertex();
}

static
bool willBeTransient(const depth &max_depth, const CompileContext &cc) {
    if (!cc.streaming) {
        return max_depth <= depth(ROSE_BLOCK_TRANSIENT_MAX_WIDTH);
    } else {
        return max_depth <= depth(cc.grey.maxHistoryAvailable + 1);
    }
}

static
bool willBeAnchoredTable(const depth &max_depth, const Grey &grey) {
    return max_depth <= depth(grey.maxAnchoredRegion);
}

static
unique_ptr<NGHolder> make_chain(u32 count) {
    assert(count);

    auto rv = make_unique<NGHolder>(NFA_INFIX);

    NGHolder &h = *rv;

    NFAVertex u = h.start;
    for (u32 i = 0; i < count; i++) {
        NFAVertex v = add_vertex(h);
        h[v].char_reach = CharReach::dot();
        add_edge(u, v, h);
        u = v;
    }
    h[u].reports.insert(0);
    add_edge(u, h.accept, h);

    setTops(h);

    return rv;
}

#define SHORT_TRIGGER_LEN 16

static
bool makeTransientFromLongLiteral(NGHolder &h, RoseInGraph &vg,
                                  const vector<RoseInEdge> &ee,
                                  const CompileContext &cc) {
    /* check max width and literal lengths to see if possible */
    size_t min_lit = (size_t)~0ULL;
    for (const RoseInEdge &e : ee) {
        RoseInVertex v = target(e, vg);
        LIMIT_TO_AT_MOST(&min_lit, vg[v].s.length());
    }

    if (min_lit <= SHORT_TRIGGER_LEN || min_lit >= UINT_MAX) {
        return false;
    }

    depth max_width = findMaxWidth(h);

    u32 delta = min_lit - SHORT_TRIGGER_LEN;

    if (!willBeTransient(max_width - depth(delta), cc)
        && !willBeAnchoredTable(max_width - depth(delta), cc.grey)) {
        return false;
    }

    DEBUG_PRINTF("candidate for splitting long literal (len %zu)\n", min_lit);
    DEBUG_PRINTF("delta = %u\n", delta);

    /* try split */
    map<RoseInVertex, shared_ptr<NGHolder> > graphs;
    for (const RoseInEdge &e : ee) {
        RoseInVertex v = target(e, vg);

        shared_ptr<NGHolder> h_new = cloneHolder(h);

        u32 delay = removeTrailingLiteralStates(*h_new, vg[v].s, delta);

        DEBUG_PRINTF("delay %u\n", delay);

        if (delay != delta) {
            DEBUG_PRINTF("unable to trim literal\n");
            return false;
        }

        if (in_degree(v, vg) != 1) {
            DEBUG_PRINTF("complicated\n");
            return false;
        }

        DEBUG_PRINTF("new mw = %u\n", (u32)findMaxWidth(*h_new));
        assert(willBeTransient(findMaxWidth(*h_new), cc)
               || willBeAnchoredTable(findMaxWidth(*h_new), cc.grey));

        assert(isCorrectlyTopped(*h_new));
        graphs[v] = h_new;
    }

    /* add .{repeats} from prefixes to long literals */
    for (const RoseInEdge &e : ee) {
        RoseInVertex s = source(e, vg);
        RoseInVertex t = target(e, vg);

        remove_edge(e, vg);
        const ue2_literal &orig_lit = vg[t].s;

        ue2_literal lit(orig_lit.begin(), orig_lit.end() - delta);

        ue2_literal lit2(orig_lit.end() - delta, orig_lit.end());

        assert(lit.length() + delta == orig_lit.length());

        vg[t].s = lit2;

        RoseInVertex v = add_vertex(RoseInVertexProps::makeLiteral(lit), vg);
        add_edge(s, v, RoseInEdgeProps(graphs[t], 0), vg);
        add_edge(v, t, RoseInEdgeProps(make_chain(delta), 0), vg);
    }

    DEBUG_PRINTF("success\n");
    /* TODO: alter split point to avoid pathological splits */
    return true;
}

static
void restoreTrailingLiteralStates(NGHolder &g, const ue2_literal &lit,
                                  u32 delay, const vector<NFAVertex> &preds) {
    assert(delay <= lit.length());
    assert(isCorrectlyTopped(g));
    DEBUG_PRINTF("adding on '%s' %u\n", dumpString(lit).c_str(), delay);

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
        NFAEdge e = add_edge_if_not_present(v, prev, g);
        if (v == g.start && is_triggered(g)) {
            g[e].tops.insert(DEFAULT_TOP);
        }
    }

    // Every predecessor of accept must have a report.
    set_report(g, 0);

    renumber_vertices(g);
    renumber_edges(g);
    assert(allMatchStatesHaveReports(g));
    assert(isCorrectlyTopped(g));
}

static
void restoreTrailingLiteralStates(NGHolder &g,
                                  const vector<pair<ue2_literal, u32>> &lits) {
    vector<NFAVertex> preds;
    insert(&preds, preds.end(), inv_adjacent_vertices(g.accept, g));
    clear_in_edges(g.accept, g);

    for (auto v : preds) {
        g[v].reports.clear(); /* clear report from old accepts */
    }

    for (const auto &p : lits) {
        const ue2_literal &lit = p.first;
        u32 delay = p.second;

        restoreTrailingLiteralStates(g, lit, delay, preds);
    }
}

static
bool improvePrefix(NGHolder &h, RoseInGraph &vg, const vector<RoseInEdge> &ee,
                   const CompileContext &cc) {
    DEBUG_PRINTF("trying to improve prefix %p, %zu verts\n", &h,
                  num_vertices(h));
    assert(isCorrectlyTopped(h));

    renumber_vertices(h);
    renumber_edges(h);

    auto depths = calcDepths(h);

    /* If the reason the prefix is not transient is due to a very long literal
     * following, we can make it transient by restricting ourselves to using
     * just the head of the literal. */
    if (makeTransientFromLongLiteral(h, vg, ee, cc)) {
        return true;
    }

    auto split = findBestPrefixSplit(h, depths, vg, ee, false, cc);

    if (split && (split->creates_transient || split->creates_anchored)
        && splitRoseEdge(h, vg, ee, *split)) {
        DEBUG_PRINTF("split on simple literal\n");
        return true;
    }

    /* large back edges may prevent us identifing anchored or transient cases
     * properly - use a simple walk instead */

    if (doNetflowCut(h, &depths, vg, ee, true, cc.grey)) {
        return true;
    }

    if (split && splitRoseEdge(h, vg, ee, *split)) {
        /* use the simple split even though it doesn't create a transient
         * prefix */
        DEBUG_PRINTF("split on simple literal\n");
        return true;
    }

    /* look for netflow cuts which don't produce good prefixes */
    if (doNetflowCut(h, &depths, vg, ee, false, cc.grey)) {
        return true;
    }

    if (ee.size() > 1) {
        DEBUG_PRINTF("split the prefix apart based on succ literals\n");
        unordered_map<shared_ptr<NGHolder>, vector<pair<RoseInEdge, u32> >,
                      NGHolderHasher, NGHolderEqual> trimmed;

        for (const auto &e : ee) {
            shared_ptr<NGHolder> hh = cloneHolder(h);
            auto succ_lit = vg[target(e, vg)].s;
            assert(isCorrectlyTopped(*hh));
            u32 delay = removeTrailingLiteralStates(*hh, succ_lit,
                                                    succ_lit.length(),
                                              false /* can't overhang start */);
            if (!delay) {
                DEBUG_PRINTF("could not remove any literal, skip over\n");
                continue;
            }

            assert(isCorrectlyTopped(*hh));
            trimmed[hh].emplace_back(e, delay);
        }

        if (trimmed.size() == 1) {
            return false;
        }

        /* shift the contents to a vector so we can modify the graphs without
         * violating the map's invariants. */
        vector<pair<shared_ptr<NGHolder>, vector<pair<RoseInEdge, u32> > > >
            trimmed_vec(trimmed.begin(), trimmed.end());
        trimmed.clear();
        for (auto &elem : trimmed_vec) {
            shared_ptr<NGHolder> &hp = elem.first;
            vector<pair<ue2_literal, u32>> succ_lits;

            for (const auto &edge_delay : elem.second) {
                const RoseInEdge &e = edge_delay.first;
                u32 delay = edge_delay.second;
                auto lit = vg[target(e, vg)].s;

                vg[e].graph = hp;
                assert(delay <= lit.length());
                succ_lits.emplace_back(lit, delay);
            }
            restoreTrailingLiteralStates(*hp, succ_lits);
        }
        return true;
    }

    return false;
}

#define MAX_FIND_BETTER_PREFIX_GEN   4
#define MAX_FIND_BETTER_PREFIX_COUNT 100

static
void findBetterPrefixes(RoseInGraph &vg, const CompileContext &cc) {
    STAGE_DEBUG_PRINTF("FIND BETTER PREFIXES\n");
    RoseInVertex start = getStart(vg);

    insertion_ordered_map<NGHolder *, vector<RoseInEdge>> prefixes;
    bool changed;
    u32 gen = 0;
    do {
        DEBUG_PRINTF("gen %u\n", gen);
        changed = false;
        prefixes.clear();

        /* find prefixes */
        for (const RoseInEdge &e : out_edges_range(start, vg)) {
            /* outfixes shouldn't have made it this far */
            assert(vg[target(e, vg)].type == RIV_LITERAL);
            if (vg[e].graph) {
                NGHolder *h = vg[e].graph.get();
                prefixes[h].push_back(e);
            }
        }

        if (prefixes.size() > MAX_FIND_BETTER_PREFIX_COUNT) {
            break;
        }

        /* look for bad prefixes and try to split */
        for (const auto &m : prefixes) {
            NGHolder *h = m.first;
            const auto &edges = m.second;
            depth max_width = findMaxWidth(*h);
            if (willBeTransient(max_width, cc)
                || willBeAnchoredTable(max_width, cc.grey)) {
                continue;
            }

            changed = improvePrefix(*h, vg, edges, cc);
        }
    } while (changed && gen++ < MAX_FIND_BETTER_PREFIX_GEN);
}

#define STRONG_LITERAL_LENGTH 20
#define MAX_EXTRACT_STRONG_LITERAL_GRAPHS 10

static
bool extractStrongLiteral(NGHolder &h, RoseInGraph &vg,
                          const vector<RoseInEdge> &ee,
                          const CompileContext &cc) {
    DEBUG_PRINTF("looking for string literal\n");
    unique_ptr<VertLitInfo> split = findBestNormalSplit(h, vg, ee, cc);

    if (split && min_len(split->lit) >= STRONG_LITERAL_LENGTH) {
        DEBUG_PRINTF("splitting simple literal\n");
        return splitRoseEdge(h, vg, ee, *split);
    }

    return false;
}

static
void extractStrongLiterals(RoseInGraph &vg, const CompileContext &cc) {
    if (!cc.grey.violetExtractStrongLiterals) {
        return;
    }

    STAGE_DEBUG_PRINTF("EXTRACT STRONG LITERALS\n");

    unordered_set<NGHolder *> stuck;
    insertion_ordered_map<NGHolder *, vector<RoseInEdge>> edges_by_graph;
    bool changed;

    do {
        changed = false;

        edges_by_graph.clear();
        for (const RoseInEdge &ve : edges_range(vg)) {
            if (vg[source(ve, vg)].type != RIV_LITERAL) {
                continue;
            }

            if (vg[ve].graph) {
                NGHolder *h = vg[ve].graph.get();
                edges_by_graph[h].push_back(ve);
            }
        }

        if (edges_by_graph.size() > MAX_EXTRACT_STRONG_LITERAL_GRAPHS) {
            DEBUG_PRINTF("too many graphs, stopping\n");
            return;
        }

        for (const auto &m : edges_by_graph) {
            NGHolder *g = m.first;
            const auto &edges = m.second;
            if (contains(stuck, g)) {
                DEBUG_PRINTF("already known to be bad\n");
                continue;
            }
            bool rv = extractStrongLiteral(*g, vg, edges, cc);
            if (rv) {
                changed = true;
            } else {
                stuck.insert(g);
            }
        }
    } while (changed);
}

#define INFIX_STRONG_GUARD_LEN 8
#define INFIX_MIN_SPLIT_LITERAL_LEN 12

static
bool improveInfix(NGHolder &h, RoseInGraph &vg, const vector<RoseInEdge> &ee,
                  const CompileContext &cc) {
    unique_ptr<VertLitInfo> split = findBestNormalSplit(h, vg, ee, cc);

    if (split && min_len(split->lit) >= INFIX_MIN_SPLIT_LITERAL_LEN
        && splitRoseEdge(h, vg, ee, *split)) {
        DEBUG_PRINTF("splitting simple literal\n");
        return true;
    }

    DEBUG_PRINTF("trying for a netflow cut\n");
    /* look for netflow cuts which don't produce good prefixes */
    bool rv = doNetflowCut(h, nullptr, vg, ee, false, cc.grey, 8);

    DEBUG_PRINTF("did netfow cut? = %d\n", (int)rv);

    return rv;
}

/**
 * Infixes which are weakly guarded can, in effect, act like prefixes as they
 * will often be live. We should try to split these infixes further if they
 * contain strong literals so that we are at least running smaller weak infixes
 * which can hopeful be accelerated/miracled.
 */
static
void improveWeakInfixes(RoseInGraph &vg, const CompileContext &cc) {
    if (!cc.grey.violetAvoidWeakInfixes) {
        return;
    }
    STAGE_DEBUG_PRINTF("IMPROVE WEAK INFIXES\n");

    RoseInVertex start = getStart(vg);

    unordered_set<NGHolder *> weak;

    for (RoseInVertex vv : adjacent_vertices_range(start, vg)) {
        /* outfixes shouldn't have made it this far */
        assert(vg[vv].type == RIV_LITERAL);
        if (vg[vv].s.length() >= INFIX_STRONG_GUARD_LEN) {
            continue;
        }

        for (const RoseInEdge &e : out_edges_range(vv, vg)) {
            if (vg[target(e, vg)].type != RIV_LITERAL || !vg[e].graph) {
                continue;
            }

            NGHolder *h = vg[e].graph.get();
            DEBUG_PRINTF("'%s' guards %p\n", dumpString(vg[vv].s).c_str(), h);
            weak.insert(h);
        }
    }

    insertion_ordered_map<NGHolder *, vector<RoseInEdge>> weak_edges;
    for (const RoseInEdge &ve : edges_range(vg)) {
        NGHolder *h = vg[ve].graph.get();
        if (contains(weak, h)) {
            weak_edges[h].push_back(ve);
        }
    }

    for (const auto &m : weak_edges) {
        NGHolder *h = m.first;
        const auto &edges = m.second;
        improveInfix(*h, vg, edges, cc);
    }
}

static
void splitEdgesForSuffix(const NGHolder &base_graph, RoseInGraph &vg,
                         const vector<RoseInEdge> &ee, const VertLitInfo &split,
                         bool eod, const flat_set<ReportID> &reports) {
    const vector<NFAVertex> &splitters = split.vv;
    assert(!splitters.empty());

    shared_ptr<NGHolder> lhs = make_shared<NGHolder>();
    unordered_map<NFAVertex, NFAVertex> v_map;
    cloneHolder(*lhs, base_graph, &v_map);
    lhs->kind = NFA_INFIX;
    clear_in_edges(lhs->accept, *lhs);
    clear_in_edges(lhs->acceptEod, *lhs);
    add_edge(lhs->accept, lhs->acceptEod, *lhs);
    clearReports(*lhs);
    for (NFAVertex v : splitters) {
        NFAEdge e = add_edge(v_map[v], lhs->accept, *lhs);
        if (v == base_graph.start) {
            (*lhs)[e].tops.insert(DEFAULT_TOP);
        }
        (*lhs)[v_map[v]].reports.insert(0);

    }
    pruneUseless(*lhs);
    assert(isCorrectlyTopped(*lhs));

    /* create literal vertices and connect preds */
    for (const auto &lit : split.lit) {
        if (!can_match(*lhs, lit, is_triggered(*lhs))) {
            continue;
        }

        DEBUG_PRINTF("best is '%s'\n", escapeString(lit).c_str());
        RoseInVertex v = add_vertex(RoseInVertexProps::makeLiteral(lit), vg);

        RoseInVertex tt;
        if (eod) {
            DEBUG_PRINTF("doing eod\n");
            tt = add_vertex(RoseInVertexProps::makeAcceptEod(reports), vg);
        } else {
            DEBUG_PRINTF("doing non-eod\n");
            tt = add_vertex(RoseInVertexProps::makeAccept(reports), vg);
        }
        add_edge(v, tt, RoseInEdgeProps(0U, 0U), vg);

        for (const RoseInEdge &e : ee) {
            RoseInVertex u = source(e, vg);
            assert(!edge(u, v, vg).second);
            add_edge(u, v, RoseInEdgeProps(lhs, 0U), vg);
        }
    }
}

#define MIN_SUFFIX_LEN 6

static
bool replaceSuffixWithInfix(const NGHolder &h, RoseInGraph &vg,
                            const vector<RoseInEdge> &suffix_edges,
                            const CompileContext &cc) {
    DEBUG_PRINTF("inspecting suffix : %p on %zu edges\n", &h,
                 suffix_edges.size());
    /*
     * We would, in general, rather not have output exposed engines because
     * once they are triggered, they must be run while infixes only have to run
     * if the successor literal is seen. Matches from output exposed engines
     * also have to be placed in a priority queue and interleaved with matches
     * from other sources.
     *
     * Note:
     * - if the LHS is extremely unlikely we may be better off leaving
     *   a suffix unguarded.
     *
     * - limited width suffixes may be less bad as they won't be continuously
     *   active, we may want to have (a) stronger controls on if we want to pick
     *   a trailing literal in these cases and/or (b) look also for literals
     *   near accept as well as right on accept
     *
     * TODO: improve heuristics, splitting logic.
     */

    /* we may do multiple splits corresponding to different report behaviour */
    set<NFAVertex> seen;
    map<pair<bool, flat_set<ReportID> >, VertLitInfo> by_reports; /* eod, rep */

    for (NFAVertex v : inv_adjacent_vertices_range(h.accept, h)) {
        set<ue2_literal> ss = getLiteralSet(h, v, false);
        if (ss.empty()) {
            DEBUG_PRINTF("candidate is too shitty\n");
            return false;
        }

        VertLitInfo &vli = by_reports[make_pair(false, h[v].reports)];
        insert(&vli.lit, ss);
        vli.vv.push_back(v);
        seen.insert(v);
    }

    seen.insert(h.accept);
    for (NFAVertex v : inv_adjacent_vertices_range(h.acceptEod, h)) {
        if (contains(seen, v)) {
            continue;
        }

        set<ue2_literal> ss = getLiteralSet(h, v, false);
        if (ss.empty()) {
            DEBUG_PRINTF("candidate is too shitty\n");
            return false;
        }

        VertLitInfo &vli = by_reports[make_pair(true, h[v].reports)];
        insert(&vli.lit, ss);
        vli.vv.push_back(v);
    }

    assert(!by_reports.empty());

    /* TODO: how strong a min len do we want here ? */
    u32 min_len = cc.grey.minRoseLiteralLength;
    ENSURE_AT_LEAST(&min_len, MIN_SUFFIX_LEN);

    for (auto &vli : by_reports | map_values) {
        u64a score = sanitizeAndCompressAndScore(vli.lit);

        if (vli.lit.empty()
            || !validateRoseLiteralSetQuality(vli.lit, score, false, min_len,
                                              false, false)) {
            return false;
        }
    }

    for (const auto &info : by_reports) {
        DEBUG_PRINTF("splitting on simple literals\n");
        splitEdgesForSuffix(h, vg, suffix_edges, info.second,
                            info.first.first /* eod */,
                            info.first.second /* reports */);
    }

    for (const RoseInEdge &e : suffix_edges) {
        remove_edge(e, vg);
    }
    return true;
}

static
void avoidSuffixes(RoseInGraph &vg, const CompileContext &cc) {
    if (!cc.grey.violetAvoidSuffixes) {
        return;
    }

    STAGE_DEBUG_PRINTF("AVOID SUFFIXES\n");

    RoseInVertex accept = getPrimaryAccept(vg);

    insertion_ordered_map<const NGHolder *, vector<RoseInEdge>> suffixes;

    /* find suffixes */
    for (const RoseInEdge &e : in_edges_range(accept, vg)) {
        /* outfixes shouldn't have made it this far */
        assert(vg[source(e, vg)].type == RIV_LITERAL);
        assert(vg[e].graph); /* non suffix paths should be wired to other
                                accepts */
        const NGHolder *h = vg[e].graph.get();
        suffixes[h].push_back(e);
    }

    /* look at suffixes and try to split */
    for (const auto &m : suffixes) {
        const NGHolder *h = m.first;
        const auto &edges = m.second;
        replaceSuffixWithInfix(*h, vg, edges, cc);
    }
}

static
bool leadingDotStartLiteral(const NGHolder &h, VertLitInfo *out) {
    if (out_degree(h.start, h) != 3) {
        return false;
    }

    NFAVertex v = NGHolder::null_vertex();
    NFAVertex ds = NGHolder::null_vertex();

    for (NFAVertex a : adjacent_vertices_range(h.start, h)) {
        if (a == h.startDs) {
            continue;
        }
        if (h[a].char_reach.all()) {
            ds = a;
            if (out_degree(ds, h) != 2 || !edge(ds, ds, h).second) {
                return false;
            }
        } else {
            v = a;
        }
    }

    if (!v || !ds || !edge(ds, v, h).second) {
        return false;
    }

    if (h[v].char_reach.count() != 1 && !h[v].char_reach.isCaselessChar()) {
        return false;
    }

    ue2_literal lit;
    lit.push_back(h[v].char_reach.find_first(),
                  h[v].char_reach.isCaselessChar());
    while (out_degree(v, h) == 1) {
        NFAVertex vv = *adjacent_vertices(v, h).first;
        if (h[vv].char_reach.count() != 1
            && !h[vv].char_reach.isCaselessChar()) {
            break;
        }

        v = vv;

        lit.push_back(h[v].char_reach.find_first(),
                      h[v].char_reach.isCaselessChar());
    }

    if (is_match_vertex(v, h) && h.kind != NFA_SUFFIX) {
        /* we have rediscovered the post-infix literal */
        return false;
    }

    if (bad_mixed_sensitivity(lit)) {
        make_nocase(&lit);
    }

    DEBUG_PRINTF("%zu found %s\n", h[v].index, dumpString(lit).c_str());
    out->vv = {v};
    out->lit = {lit};
    return true;
}

static
bool lookForDoubleCut(const NGHolder &h, const vector<RoseInEdge> &ee,
                      RoseInGraph &vg, const Grey &grey) {
    VertLitInfo info;
    if (!leadingDotStartLiteral(h, &info)
        || min_len(info.lit) < grey.violetDoubleCutLiteralLen) {
        return false;
    }
    DEBUG_PRINTF("performing split\n");
    return splitRoseEdge(h, vg, ee, {info});
}

static
void lookForDoubleCut(RoseInGraph &vg, const CompileContext &cc) {
    if (!cc.grey.violetDoubleCut) {
        return;
    }

    insertion_ordered_map<const NGHolder *, vector<RoseInEdge>> right_edges;
    for (const RoseInEdge &ve : edges_range(vg)) {
        if (vg[ve].graph && vg[source(ve, vg)].type == RIV_LITERAL) {
            const NGHolder *h = vg[ve].graph.get();
            right_edges[h].push_back(ve);
        }
    }

    for (const auto &m : right_edges) {
        const NGHolder *h = m.first;
        const auto &edges = m.second;
        lookForDoubleCut(*h, edges, vg, cc.grey);
    }
}

static
pair<NFAVertex, ue2_literal> findLiteralBefore(const NGHolder &h, NFAVertex v) {
    ue2_literal lit;
    if (h[v].char_reach.count() != 1 && !h[v].char_reach.isCaselessChar()) {
        return {v, std::move(lit) };
    }
    lit.push_back(h[v].char_reach.find_first(),
                  h[v].char_reach.isCaselessChar());

    while (in_degree(v, h) == 1) {
        NFAVertex vv = *inv_adjacent_vertices(v, h).first;
        if (h[vv].char_reach.count() != 1
            && !h[vv].char_reach.isCaselessChar()) {
            break;
        }

        lit.push_back(h[vv].char_reach.find_first(),
                      h[vv].char_reach.isCaselessChar());
        v = vv;
    }

    return {v, std::move(lit) };
}

static
bool lookForDotStarPred(NFAVertex v, const NGHolder &h,
                        NFAVertex *u, NFAVertex *ds) {
    *u = NGHolder::null_vertex();
    *ds = NGHolder::null_vertex();
    for (NFAVertex a : inv_adjacent_vertices_range(v, h)) {
        if (h[a].char_reach.all()) {
            if (!edge(a, a, h).second) {
                return false;
            }

            if (*ds) {
                return false;
            }

            *ds = a;
        } else {
            if (*u) {
                return false;
            }
            *u = a;
        }
    }

    if (!*u || !*ds) {
        return false;
    }

    return true;
}

static
bool trailingDotStarLiteral(const NGHolder &h, VertLitInfo *out) {
    /* Note: there is no delay yet - so the final literal is the already
     * discovered successor literal - we are in fact interested in the literal
     * before it. */

    if (in_degree(h.accept, h) != 1) {
        return false;
    }

    if (in_degree(h.acceptEod, h) != 1) {
        assert(0);
        return false;
    }

    NFAVertex v
        = findLiteralBefore(h, *inv_adjacent_vertices(h.accept, h).first).first;

    NFAVertex u;
    NFAVertex ds;

    if (!lookForDotStarPred(v, h, &u, &ds)) {
        return false;
    }

    v = u;
    auto rv = findLiteralBefore(h, v);

    if (!lookForDotStarPred(v, h, &u, &ds)) {
        return false;
    }

    ue2_literal lit = reverse_literal(rv.second);
    DEBUG_PRINTF("%zu found %s\n", h[v].index, dumpString(lit).c_str());

    if (bad_mixed_sensitivity(lit)) {
        make_nocase(&lit);
    }

    out->vv = {v};
    out->lit = {lit};
    return true;
}

static
bool lookForTrailingLiteralDotStar(const NGHolder &h,
                                   const vector<RoseInEdge> &ee,
                                   RoseInGraph &vg, const Grey &grey) {
    VertLitInfo info;
    if (!trailingDotStarLiteral(h, &info)
        || min_len(info.lit) < grey.violetDoubleCutLiteralLen) {
        return false;
    }
    DEBUG_PRINTF("performing split\n");
    return splitRoseEdge(h, vg, ee, info);
}

/* In streaming mode, active engines have to be caught up at stream boundaries
 * and have to be stored in stream state, so we prefer to decompose patterns
 * in to literals with no state between them if possible. */
static
void decomposeLiteralChains(RoseInGraph &vg, const CompileContext &cc) {
    if (!cc.grey.violetLiteralChains) {
        return;
    }

    insertion_ordered_map<const NGHolder *, vector<RoseInEdge>> right_edges;
    bool changed;
    do {
        changed = false;

        right_edges.clear();
        for (const RoseInEdge &ve : edges_range(vg)) {
            if (vg[ve].graph && vg[source(ve, vg)].type == RIV_LITERAL) {
                const NGHolder *h = vg[ve].graph.get();
                right_edges[h].push_back(ve);
            }
        }

        for (const auto &m : right_edges) {
            const NGHolder *h = m.first;
            const vector<RoseInEdge> &ee = m.second;
            bool rv = lookForDoubleCut(*h, ee, vg, cc.grey);
            if (!rv && h->kind != NFA_SUFFIX) {
                rv = lookForTrailingLiteralDotStar(*h, ee, vg, cc.grey);
            }
            changed |= rv;
        }
    } while (changed);
}

static
bool lookForCleanSplit(const NGHolder &h, const vector<RoseInEdge> &ee,
                       RoseInGraph &vg, const CompileContext &cc) {
    unique_ptr<VertLitInfo> split = findBestCleanSplit(h, cc);

    if (split) {
        return splitRoseEdge(h, vg, {ee}, *split);
    }

    return false;
}

#define MAX_DESIRED_CLEAN_SPLIT_DEPTH 4

static
void lookForCleanEarlySplits(RoseInGraph &vg, const CompileContext &cc) {
    u32 gen = 0;

    insertion_ordered_set<RoseInVertex> prev({getStart(vg)});
    insertion_ordered_set<RoseInVertex> curr;

    while (gen < MAX_DESIRED_CLEAN_SPLIT_DEPTH) {
        curr.clear();
        for (RoseInVertex u : prev) {
            for (auto v : adjacent_vertices_range(u, vg)) {
                curr.insert(v);
            }
        }

        insertion_ordered_map<const NGHolder *, vector<RoseInEdge>> rightfixes;
        for (RoseInVertex v : curr) {
            for (const RoseInEdge &e : out_edges_range(v, vg)) {
                if (vg[e].graph) {
                    NGHolder *h = vg[e].graph.get();
                    rightfixes[h].push_back(e);
                }
            }
        }

        for (const auto &m : rightfixes) {
            const NGHolder *h = m.first;
            const auto &edges = m.second;
            lookForCleanSplit(*h, edges, vg, cc);
        }

        prev = std::move(curr);
        gen++;
    }
}

static
void rehomeEodSuffixes(RoseInGraph &vg) {
    // Find edges to accept with EOD-anchored graphs that we can move over to
    // acceptEod.
    vector<RoseInEdge> acc_edges;
    for (const auto &e : edges_range(vg)) {
        if (vg[target(e, vg)].type != RIV_ACCEPT) {
            continue;
        }
        if (vg[e].haig || !vg[e].graph) {
            continue;
        }

        const NGHolder &h = *vg[e].graph;

        if (in_degree(h.accept, h)) {
            DEBUG_PRINTF("graph isn't eod anchored\n");
            continue;
        }

        acc_edges.push_back(e);
    }

    for (const RoseInEdge &e : acc_edges) {
        // Move this edge from accept to acceptEod
        RoseInVertex w = add_vertex(RoseInVertexProps::makeAcceptEod(), vg);
        add_edge(source(e, vg), w, vg[e], vg);
        remove_edge(e, vg);
    }

    /* old accept vertices will be tidied up by final pruneUseless() call */
}

static
bool tryForEarlyDfa(const NGHolder &h, const CompileContext &cc) {
    switch (h.kind) {
    case NFA_OUTFIX: /* 'prefix' of eod */
    case NFA_PREFIX:
        return cc.grey.earlyMcClellanPrefix;
    case NFA_INFIX:
        return cc.grey.earlyMcClellanInfix;
    case NFA_SUFFIX:
        return cc.grey.earlyMcClellanSuffix;
    default:
        DEBUG_PRINTF("kind %u\n", (u32)h.kind);
        assert(0);
        return false;
    }
}

static
vector<vector<CharReach>> getDfaTriggers(RoseInGraph &vg,
                                         const vector<RoseInEdge> &edges,
                                         bool *single_trigger) {
    vector<vector<CharReach>> triggers;
    u32 min_offset = ~0U;
    u32 max_offset = 0;
    for (const auto &e : edges) {
        RoseInVertex s = source(e, vg);
        if (vg[s].type == RIV_LITERAL) {
            triggers.push_back(as_cr_seq(vg[s].s));
        }
        ENSURE_AT_LEAST(&max_offset, vg[s].max_offset);
        LIMIT_TO_AT_MOST(&min_offset, vg[s].min_offset);
    }

    *single_trigger = min_offset == max_offset;
    DEBUG_PRINTF("trigger offset (%u, %u)\n", min_offset, max_offset);

    return triggers;
}

static
bool doEarlyDfa(RoseBuild &rose, RoseInGraph &vg, NGHolder &h,
                const vector<RoseInEdge> &edges, bool final_chance,
                const ReportManager &rm, const CompileContext &cc) {
    DEBUG_PRINTF("trying for dfa\n");

    bool single_trigger;
    for (const auto &e : edges) {
        if (vg[target(e, vg)].type == RIV_ACCEPT_EOD) {
            /* TODO: support eod prefixes */
            return false;
        }
    }

    auto triggers = getDfaTriggers(vg, edges, &single_trigger);

    /* TODO: literal delay things */
    if (!generates_callbacks(h)) {
        set_report(h, rose.getNewNfaReport());
    }

    shared_ptr<raw_dfa> dfa = buildMcClellan(h, &rm, single_trigger, triggers,
                                             cc.grey, final_chance);

    if (!dfa) {
        return false;
    }

    DEBUG_PRINTF("dfa ok\n");
    for (const auto &e : edges) {
        vg[e].dfa = dfa;
    }

    return true;
}

#define MAX_EDGES_FOR_IMPLEMENTABILITY 50

static
bool splitForImplementability(RoseInGraph &vg, NGHolder &h,
                              const vector<RoseInEdge> &edges,
                              const CompileContext &cc) {
    vector<pair<ue2_literal, u32>> succ_lits;
    DEBUG_PRINTF("trying to split %s with %zu vertices on %zu edges\n",
                  to_string(h.kind).c_str(), num_vertices(h), edges.size());

    if (edges.size() > MAX_EDGES_FOR_IMPLEMENTABILITY) {
        return false;
    }

    if (!generates_callbacks(h)) {
        for (const auto &e : edges) {
            const auto &lit = vg[target(e, vg)].s;
            u32 delay = vg[e].graph_lag;
            vg[e].graph_lag = 0;

            assert(delay <= lit.length());
            succ_lits.emplace_back(lit, delay);
        }
        restoreTrailingLiteralStates(h, succ_lits);
    }

    unique_ptr<VertLitInfo> split;
    bool last_chance = true;
    if (h.kind == NFA_PREFIX) {
        auto depths = calcDepths(h);

        split = findBestPrefixSplit(h, depths, vg, edges, last_chance, cc);
    } else {
        split = findBestLastChanceSplit(h, vg, edges, cc);
    }

    if (split && splitRoseEdge(h, vg, edges, *split)) {
        DEBUG_PRINTF("split on simple literal\n");
        return true;
    }

    DEBUG_PRINTF("trying to netflow\n");
    bool rv = doNetflowCut(h, nullptr, vg, edges, false, cc.grey);
    DEBUG_PRINTF("done\n");

    return rv;
}

#define MAX_IMPLEMENTABLE_SPLITS 50

bool ensureImplementable(RoseBuild &rose, RoseInGraph &vg, bool allow_changes,
                         bool final_chance, const ReportManager &rm,
                         const CompileContext &cc) {
    DEBUG_PRINTF("checking for impl %d\n", final_chance);
    bool changed = false;
    bool need_to_recalc = false;
    u32 added_count = 0;
    unordered_set<shared_ptr<NGHolder>> good; /* known to be implementable */
    do {
        changed = false;
        DEBUG_PRINTF("added %u\n", added_count);
        insertion_ordered_map<shared_ptr<NGHolder>,
                              vector<RoseInEdge>> edges_by_graph;
        for (const RoseInEdge &ve : edges_range(vg)) {
            if (vg[ve].graph && !vg[ve].dfa) {
                auto &h = vg[ve].graph;
                edges_by_graph[h].push_back(ve);
            }
        }
        for (auto &m : edges_by_graph) {
            auto &h = m.first;
            if (contains(good, h)) {
                continue;
            }
            reduceGraphEquivalences(*h, cc);
            if (isImplementableNFA(*h, &rm, cc)) {
                good.insert(h);
                continue;
            }

            const auto &edges = m.second;

            if (tryForEarlyDfa(*h, cc) &&
                doEarlyDfa(rose, vg, *h, edges, final_chance, rm, cc)) {
                continue;
            }

            DEBUG_PRINTF("eek\n");
            if (!allow_changes) {
                return false;
            }

            if (splitForImplementability(vg, *h, edges, cc)) {
                added_count++;
                if (added_count > MAX_IMPLEMENTABLE_SPLITS) {
                    DEBUG_PRINTF("added_count hit limit\n");
                    return false;
                }
                changed = true;
                continue;
            }

            return false;
        }

        assert(added_count <= MAX_IMPLEMENTABLE_SPLITS);

        if (changed) {
            removeRedundantLiterals(vg, cc);
            pruneUseless(vg);
            need_to_recalc = true;
        }
    } while (changed);

    if (need_to_recalc) {
        renumber_vertices(vg);
        calcVertexOffsets(vg);
    }

    DEBUG_PRINTF("ok!\n");
    return true;
}

static
RoseInGraph doInitialVioletTransform(const NGHolder &h, bool last_chance,
                                     const CompileContext &cc) {
    assert(!can_never_match(h));

    RoseInGraph vg = populateTrivialGraph(h);

    if (!cc.grey.allowViolet) {
        return vg;
    }

    /* Avoid running the Violet analysis at all on graphs with no vertices with
     * small reach, since we will not be able to extract any literals. */
    if (!hasNarrowReachVertex(h)) {
        DEBUG_PRINTF("fail, no vertices with small reach\n");
        return vg;
    }

    DEBUG_PRINTF("hello world\n");

    /* Step 1: avoid outfixes as we always have to run them. */
    avoidOutfixes(vg, last_chance, cc);

    if (num_vertices(vg) <= 2) {
        return vg; /* unable to transform pattern */
    }

    removeRedundantPrefixes(vg);
    dumpPreRoseGraph(vg, cc.grey, "pre_prefix_rose.dot");

    /* Step 2: avoid non-transient prefixes (esp in streaming mode) */
    findBetterPrefixes(vg, cc);

    dumpPreRoseGraph(vg, cc.grey, "post_prefix_rose.dot");

    extractStrongLiterals(vg, cc);
    dumpPreRoseGraph(vg, cc.grey, "post_extract_rose.dot");
    improveWeakInfixes(vg, cc);
    dumpPreRoseGraph(vg, cc.grey, "post_infix_rose.dot");

    /* Step 3: avoid output exposed engines if there is a strong trailing
       literal) */
    avoidSuffixes(vg, cc);

    /* Step 4: look for infixes/suffixes with leading .*literals
     * This can reduce the amount of work a heavily picked literal has to do and
     * reduce the amount of state used as .* is handled internally to rose. */
    lookForDoubleCut(vg, cc);

    if (cc.streaming) {
        lookForCleanEarlySplits(vg, cc);
        decomposeLiteralChains(vg, cc);
    }

    rehomeEodSuffixes(vg);
    removeRedundantLiterals(vg, cc);

    pruneUseless(vg);
    dumpPreRoseGraph(vg, cc.grey);
    renumber_vertices(vg);
    calcVertexOffsets(vg);

    return vg;
}

bool doViolet(RoseBuild &rose, const NGHolder &h, bool prefilter,
              bool last_chance, const ReportManager &rm,
              const CompileContext &cc) {
    auto vg = doInitialVioletTransform(h, last_chance, cc);
    if (num_vertices(vg) <= 2) {
        return false;
    }

    /* Step 5: avoid unimplementable, or overly large engines if possible */
    if (!ensureImplementable(rose, vg, last_chance, last_chance, rm, cc)) {
        return false;
    }
    dumpPreRoseGraph(vg, cc.grey, "post_ensure_rose.dot");

    /* Step 6: send to rose */
    bool rv = rose.addRose(vg, prefilter);
    DEBUG_PRINTF("violet: %s\n", rv ? "success" : "fail");
    return rv;
}

bool checkViolet(const ReportManager &rm, const NGHolder &h, bool prefilter,
                 const CompileContext &cc) {
    auto vg = doInitialVioletTransform(h, true, cc);
    if (num_vertices(vg) <= 2) {
        return false;
    }

    bool rv = roseCheckRose(vg, prefilter, rm, cc);
    DEBUG_PRINTF("violet: %s\n", rv ? "success" : "fail");
    return rv;
}

}
