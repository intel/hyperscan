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

#include "rose_build_convert.h"

#include "grey.h"
#include "rose_build.h"
#include "rose_build_impl.h"
#include "rose_build_util.h"
#include "ue2common.h"
#include "hwlm/hwlm_build.h"
#include "nfa/castlecompile.h"
#include "nfa/limex_limits.h"
#include "nfagraph/ng_dump.h"
#include "nfagraph/ng_limex.h"
#include "nfagraph/ng_repeat.h"
#include "nfagraph/ng_reports.h"
#include "nfagraph/ng_split.h"
#include "nfagraph/ng_util.h"
#include "nfagraph/ng_width.h"
#include "util/bitutils.h"
#include "util/charreach.h"
#include "util/charreach_util.h"
#include "util/compile_context.h"
#include "util/depth.h"
#include "util/graph_range.h"
#include "util/make_unique.h"
#include "util/order_check.h"
#include "util/ue2string.h"

#include <algorithm>
#include <map>
#include <queue>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <boost/range/adaptor/map.hpp>

using namespace std;
using boost::adaptors::map_values;

namespace ue2 {

static
NFAVertex addHolderVertex(const CharReach &cr, NGHolder &out) {
    assert(cr.any());
    NFAVertex v = add_vertex(out);
    out[v].char_reach = cr;
    return v;
}

static
size_t suffixFloodLen(const ue2_literal &s) {
    if (s.empty()) {
        return 0;
    }

    const ue2_literal::elem &c = s.back();
    auto it = find_if(s.rbegin(), s.rend(),
                      [&c](const ue2_literal::elem &e) { return e != c; });
    return distance(s.rbegin(), it);
}

static
unique_ptr<NGHolder> makeFloodProneSuffix(const ue2_literal &s, size_t len,
                                          const flat_set<ReportID> &reports) {
    assert(len < s.length());
    assert(!reports.empty());

    unique_ptr<NGHolder> h = ue2::make_unique<NGHolder>(NFA_SUFFIX);

    NFAVertex u = h->start;
    for (auto it = s.begin() + s.length() - len; it != s.end(); ++it) {
        NFAVertex v = addHolderVertex(*it, *h);
        NFAEdge e = add_edge(u, v, *h);
        if (u == h->start) {
            (*h)[e].tops.insert(DEFAULT_TOP);
        }
        u = v;
    }

    (*h)[u].reports.insert(reports.begin(), reports.end());
    add_edge(u, h->accept, *h);
    return h;
}

static
unique_ptr<NGHolder> makeRosePrefix(const ue2_literal &s) {
    unique_ptr<NGHolder> h = ue2::make_unique<NGHolder>(NFA_PREFIX);

    NFAVertex u = h->startDs;
    for (const auto &c : s) {
        NFAVertex v = addHolderVertex(c, *h);
        add_edge(u, v, *h);
        u = v;
    }
    add_edge(u, h->accept, *h);
    return h;
}

static
void replaceWithLitPrefix(RoseBuildImpl &tbi, RoseVertex v, u32 lit_id,
                          const rose_literal_id &lit, size_t suffixlen,
                          size_t delay) {
    assert(suffixlen < lit.s.length());

    DEBUG_PRINTF("replacing '%s' with prefix, length=%zu, delay=%zu\n",
                 dumpString(lit.s).c_str(), lit.s.length() - suffixlen, delay);

    RoseGraph &g = tbi.g;
    ue2_literal new_lit = lit.s.substr(0, lit.s.length() - suffixlen);
    u32 new_id = tbi.getLiteralId(new_lit, delay, ROSE_FLOATING);
    rose_literal_info &old_info = tbi.literal_info.at(lit_id);
    old_info.vertices.erase(v);
    tbi.literal_info.at(new_id).vertices.insert(v);
    g[v].literals.clear();
    g[v].literals.insert(new_id);
}

static
bool delayLiteralWithPrefix(RoseBuildImpl &tbi, RoseVertex v, u32 lit_id,
                            const rose_literal_id &lit, size_t suffixlen) {
    if (suffixlen > MAX_DELAY) {
        DEBUG_PRINTF("delay too large\n");
        return false;
    }

    if (!tbi.isDirectReport(lit_id)) {
        DEBUG_PRINTF("literal is not direct report\n");
        return false;
    }

    if (tbi.cc.streaming &&
        lit.s.length() > tbi.cc.grey.maxHistoryAvailable + 1) {
        DEBUG_PRINTF("insufficient history to delay literal of len %zu\n",
                      lit.s.length());
        return false;
    }

    shared_ptr<NGHolder> h = makeRosePrefix(lit.s);
    ReportID prefix_report = 0;
    set_report(*h, prefix_report);

    if (!isImplementableNFA(*h, &tbi.rm, tbi.cc)) {
        DEBUG_PRINTF("prefix not implementable\n");
        return false;
    }

    RoseGraph &g = tbi.g;
    assert(!g[v].left);
    g[v].left.graph = h;
    g[v].left.lag = 0;
    g[v].left.leftfix_report = prefix_report;

    // Swap v's literal for a shorter one, delayed by suffix len.
    replaceWithLitPrefix(tbi, v, lit_id, lit, suffixlen, suffixlen);

    return true;
}

static
void convertFloodProneSuffix(RoseBuildImpl &tbi, RoseVertex v, u32 lit_id,
                             const rose_literal_id &lit, size_t suffixlen) {
    DEBUG_PRINTF("flood-prone leaf '%s'\n", dumpString(lit.s).c_str());
    DEBUG_PRINTF("turning last %zu chars into a suffix NFA\n", suffixlen);
    RoseGraph &g = tbi.g;
    assert(!g[v].eod_accept);

    // If we're a direct report literal, we may be able to convert this case
    // into a delayed literal with a (very boring) transient prefix that
    // handles our flood-prone suffix.
    if (delayLiteralWithPrefix(tbi, v, lit_id, lit, suffixlen)) {
        DEBUG_PRINTF("implemented as delayed literal with a rose prefix\n");
        return;
    }

    // General case: create a suffix that implements the flood-prone portion.

    // Create the NFA.
    auto h = makeFloodProneSuffix(lit.s, suffixlen, g[v].reports);
    if (!isImplementableNFA(*h, &tbi.rm, tbi.cc)) {
        DEBUG_PRINTF("not implementable\n");
        return;
    }

    // Apply the NFA.
    assert(!g[v].suffix);
    g[v].suffix.graph = move(h);
    g[v].reports.clear();

    // Swap v's literal for a shorter one.
    replaceWithLitPrefix(tbi, v, lit_id, lit, suffixlen, 0);

    // It's possible that min_offset might be an underestimate, so we
    // subtract min(min_offset, suffixlen) for safety.
    g[v].min_offset -= min((size_t)g[v].min_offset, suffixlen);

    if (g[v].max_offset < ROSE_BOUND_INF) {
        assert(g[v].max_offset >= suffixlen);
        g[v].max_offset -= suffixlen;
    }
}

/**
 * Collect an estimate of the number of literals in the floating table, and use
 * this to estimate the flood prone suffix length.
 */
static
size_t findFloodProneSuffixLen(const RoseBuildImpl &tbi) {
    size_t numLiterals = 0;
    for (const rose_literal_id &lit : tbi.literals) {
        if (lit.delay) {
            continue; // delay ids are virtual-ish
        }
        if (lit.table != ROSE_FLOATING) {
            continue;
        }

        numLiterals++;
    }

    return hwlmFloodProneSuffixLen(numLiterals, tbi.cc);
}

/**
 * \brief Convert flood-prone literal suffixes into suffix NFAs.
 *
 * For any trailing string in Rose (string cannot lead to more Rose roles or
 * NFAs, etc) ending with a continuous run of a single character with more than
 * 3 copies of that single character,
 *
 * If the result of removing all but 2 copies of that character yields a string
 * that is greater than FLOOD_PRONE_LIT_MIN_LENGTH characters, remove those
 * final characters from the literal and move them into a suffix NFA.
 */
void convertFloodProneSuffixes(RoseBuildImpl &tbi) {
    static const size_t FLOOD_PRONE_LIT_MIN_LENGTH = 5;

    if (!tbi.cc.grey.roseConvertFloodProneSuffixes) {
        return;
    }

    const size_t floodProneLen = findFloodProneSuffixLen(tbi);
    DEBUG_PRINTF("flood prone suffix len = %zu\n", floodProneLen);

    RoseGraph &g = tbi.g;

    for (auto v : vertices_range(g)) {
        if (!isLeafNode(v, g)) {
            continue;
        }

        if (g[v].reports.empty()) {
            continue;
        }

        // TODO: currently only boring vertices.
        if (!g[v].isBoring()) {
            continue;
        }

        // Currently only handles vertices with a single literal (should always
        // be the case this early in Rose construction).
        if (g[v].literals.size() != 1) {
            continue;
        }

        u32 lit_id = *g[v].literals.begin();
        const rose_literal_id &lit = tbi.literals.at(lit_id);

        // anchored or delayed literals need thought.
        if (lit.table != ROSE_FLOATING || lit.delay) {
            continue;
        }

        // don't do this to literals with msk/cmp.
        if (!lit.msk.empty()) {
            continue;
        }

        // Can't safely do this operation to vertices with delayed
        // predecessors.
        if (tbi.hasDelayPred(v)) {
            DEBUG_PRINTF("delayed pred\n");
            continue;
        }

        if (lit.s.length() <= FLOOD_PRONE_LIT_MIN_LENGTH) {
            DEBUG_PRINTF("literal is short enough already\n");
            continue;
        }

        size_t floodLen = suffixFloodLen(lit.s);
        if (floodLen < floodProneLen) {
            DEBUG_PRINTF("literal not flood-prone\n");
            continue;
        }

        if (floodLen == lit.s.length()) {
            DEBUG_PRINTF("whole literal is a flood\n");
            // Removing the part of the flood from the end of the literal would
            // leave us with a shorter, but still flood-prone, prefix. Better
            // to leave it alone.
            continue;
        }

        size_t suffixLen = floodLen - (floodProneLen - 1);
        if (lit.s.length() - suffixLen < FLOOD_PRONE_LIT_MIN_LENGTH) {
            DEBUG_PRINTF("removing flood would leave literal too short\n");
            continue;
        }

        convertFloodProneSuffix(tbi, v, lit_id, lit, suffixLen);
    }
}

static
CharReach getReachOfNormalVertex(const NGHolder &g) {
    for (auto v : vertices_range(g)) {
        if (is_special(v, g)) {
            continue;
        }
        return g[v].char_reach;
    }
    assert(0);
    return CharReach();
}

/**
 * \brief Set the edge bounds and appropriate history on the given edge in the
 * Rose graph.
 */
static
void setEdgeBounds(RoseGraph &g, const RoseEdge &e, u32 min_bound,
                   u32 max_bound) {
    assert(min_bound <= max_bound);
    assert(max_bound <= ROSE_BOUND_INF);

    g[e].minBound = min_bound;
    g[e].maxBound = max_bound;

    if (min_bound || max_bound < ROSE_BOUND_INF) {
        g[e].history = ROSE_ROLE_HISTORY_ANCH;
    } else {
        g[e].history = ROSE_ROLE_HISTORY_NONE;
    }
}

static
bool handleStartPrefixCliche(const NGHolder &h, RoseGraph &g, RoseVertex v,
                             const RoseEdge &e_old, RoseVertex ar,
                             vector<RoseEdge> *to_delete) {
    DEBUG_PRINTF("hi\n");

    /* check for prefix cliches connected to start (^.{N,M}) */
    if (!getReachOfNormalVertex(h).all()) {
        DEBUG_PRINTF(":(\n");
        return false;
    }

    PureRepeat repeat;
    if (!isPureRepeat(h, repeat)) {
        DEBUG_PRINTF(":(\n");
        return false;
    }

    assert(repeat.bounds.min.is_finite());
    assert(repeat.bounds.max.is_reachable());
    assert(repeat.bounds.min <= repeat.bounds.max);

    DEBUG_PRINTF("prefix is ^.{%s,%s}\n", repeat.bounds.min.str().c_str(),
                 repeat.bounds.max.str().c_str());

    /* update bounds on edge */

    // Convert to Rose graph bounds, which are not (yet?) depth classes.
    u32 bound_min = repeat.bounds.min;
    u32 bound_max =
        repeat.bounds.max.is_finite() ? (u32)repeat.bounds.max : ROSE_BOUND_INF;

    if (source(e_old, g) == ar) {
        assert(g[e_old].minBound <= bound_min);
        assert(g[e_old].maxBound >= bound_max);
        setEdgeBounds(g, e_old, bound_min, bound_max);
    } else {
        RoseEdge e_new = add_edge(ar, v, g);
        setEdgeBounds(g, e_new, bound_min, bound_max);
        to_delete->push_back(e_old);
    }

    g[v].left.reset(); /* clear the prefix info */
    return true;
}

static
bool handleStartDsPrefixCliche(const NGHolder &h, RoseGraph &g, RoseVertex v,
                               const RoseEdge &e) {
    DEBUG_PRINTF("hi\n");
    /* check for prefix cliches connected to start-ds (.{N}, ^.{N,}) */
    u32 repeatCount = 0;
    NFAVertex hu = h.startDs;

    auto start_succ = succs<set<NFAVertex>>(h.start, h);
    auto startds_succ = succs<set<NFAVertex>>(h.startDs, h);

    if (!is_subset_of(start_succ, startds_succ)) {
        DEBUG_PRINTF("not a simple chain\n");
        return false;
    }

    set<NFAVertex> seen;
    do {
        if (!h[hu].char_reach.all()) {
            return false;
        }
        NFAVertex hv = getSoleDestVertex(h, hu);
        if (!hv) {
            return false;
        }
        if (contains(seen, hv)) {
            assert(0);
            return false;
        }
        hu = hv;
        repeatCount++;
        if (hu == h.accept) {
            break;
        }
    } while(1);

    assert(hu == h.accept);

    repeatCount--; /* do not count accept as part of the chain */

    DEBUG_PRINTF("prefix is ^.{%u,}\n", repeatCount);

    /* update bounds on edge */
    assert(g[e].minBound <= repeatCount);
    setEdgeBounds(g, e, repeatCount, ROSE_BOUND_INF);

    g[v].left.reset(); /* clear the prefix info */

    return true;
}

static
bool handleMixedPrefixCliche(const NGHolder &h, RoseGraph &g, RoseVertex v,
                             const RoseEdge &e_old, RoseVertex ar,
                             vector<RoseEdge> *to_delete,
                             const CompileContext &cc) {
    assert(in_degree(h.acceptEod, h) == 1);

    bool anchored = !proper_out_degree(h.startDs, h);
    NFAVertex key = NGHolder::null_vertex();
    NFAVertex base = anchored ? h.start : h.startDs;

    if (!anchored) {
        auto start_succ = succs<set<NFAVertex>>(h.start, h);
        auto startds_succ = succs<set<NFAVertex>>(h.startDs, h);

        if (!is_subset_of(start_succ, startds_succ)) {
            DEBUG_PRINTF("not a simple chain\n");
            return false;
        }
    }

    for (auto w : adjacent_vertices_range(base, h)) {
        DEBUG_PRINTF("checking %zu\n", h[w].index);
        if (!h[w].char_reach.all()) {
            continue;
        }

        if (!is_special(w, h)) {
            key = w;
            break;
        }
    }

    if (!key) {
        return false;
    }

    vector<GraphRepeatInfo> repeats;
    findRepeats(h, 2, &repeats);

    vector<GraphRepeatInfo>::const_iterator it;
    for (it = repeats.begin(); it != repeats.end(); ++it) {
        DEBUG_PRINTF("checking.. %zu verts\n", it->vertices.size());
        if (find(it->vertices.begin(), it->vertices.end(), key)
            != it->vertices.end()) {
            break;
        }
    }
    if (it == repeats.end()) {
        DEBUG_PRINTF("no repeat found\n");
        return false;
    }

    GraphRepeatInfo ri = *it;

    set<NFAVertex> exits_and_repeat_verts;
    for (auto repeat_v : ri.vertices) {
        DEBUG_PRINTF("repeat vertex %zu\n", h[repeat_v].index);
        succ(h, repeat_v, &exits_and_repeat_verts);
        exits_and_repeat_verts.insert(repeat_v);
    }

    DEBUG_PRINTF("repeat {%s,%s}\n", ri.repeatMin.str().c_str(),
                 ri.repeatMax.str().c_str());

    set<NFAVertex> rep_verts;
    insert(&rep_verts, ri.vertices);

    set<NFAVertex> exits;
    exits = exits_and_repeat_verts;
    erase_all(&exits, rep_verts);

    auto base_succ = succs<set<NFAVertex>>(base, h);
    base_succ.erase(h.startDs);

    if (is_subset_of(base_succ, rep_verts)) {
        /* all good: repeat dominates the rest of the pattern */
    } else if (ri.repeatMin == depth(1)
               && is_subset_of(exits, base_succ)
               && is_subset_of(base_succ, exits_and_repeat_verts)) {
        /* we have a jump edge */
        ri.repeatMin = depth(0);
    } else {
        return false;
    }

    DEBUG_PRINTF("repeat {%s,%s}\n", ri.repeatMin.str().c_str(),
                 ri.repeatMax.str().c_str());
    DEBUG_PRINTF("woot?\n");

    shared_ptr<NGHolder> h_new = make_shared<NGHolder>();
    unordered_map<NFAVertex, NFAVertex> rhs_map;
    vector<NFAVertex> exits_vec;
    insert(&exits_vec, exits_vec.end(), exits);
    splitRHS(h, exits_vec, h_new.get(), &rhs_map);
    h_new->kind = NFA_PREFIX;

    if (num_vertices(*h_new) <= N_SPECIALS) {
        DEBUG_PRINTF("not a hybrid??\n");
        /* TODO: pick up these cases, unify code */
        return false;
    }

    for (auto w : adjacent_vertices_range(h_new->start, *h_new)) {
        if (w != h_new->startDs) {
            add_edge(h_new->startDs, w, *h_new);
        }
    }
    clear_out_edges(h_new->start, *h_new);
    add_edge(h_new->start, h_new->startDs, *h_new);

    depth width = findMinWidth(*h_new);
    if (width != findMaxWidth(*h_new)) {
        return false;
    }

    if (g[v].left.dfa) {
        /* we were unable to implement initial graph as an nfa;
         * we need to to check if we still need a dfa and, if so, rebuild. */
        if (!isImplementableNFA(*h_new, nullptr, cc)) {
            return false; /* TODO: handle rebuilding dfa */
        }
    }

    if (anchored) {
        if (ri.repeatMax.is_infinite()) {
            return false; /* TODO */
        }

        if (source(e_old, g) == ar) {
            setEdgeBounds(g, e_old, ri.repeatMin + width, ri.repeatMax + width);
        } else {
            RoseEdge e_new = add_edge(ar, v, g);
            setEdgeBounds(g, e_new, ri.repeatMin + width, ri.repeatMax + width);
            to_delete->push_back(e_old);
        }

    } else {
        assert(g[e_old].minBound <= ri.repeatMin + width);
        setEdgeBounds(g, e_old, ri.repeatMin + width, ROSE_BOUND_INF);
    }

    g[v].left.dfa.reset();
    g[v].left.graph = h_new;

    return true;
}

/* turns simple prefixes like /^.{30,} into bounds on the root roles */
void convertPrefixToBounds(RoseBuildImpl &tbi) {
    RoseGraph &g = tbi.g;

    vector<RoseEdge> to_delete;
    RoseVertex ar = tbi.anchored_root;

    /* graphs with prefixes produced by rose are wired to tbi.root */

    for (const auto &e : out_edges_range(tbi.root, g)) {
        RoseVertex v = target(e, g);

        if (in_degree(v, g) != 1) {
            continue;
        }

        if (!g[v].left.graph) {
            continue;
        }

        if (g[v].left.tracksSom()) {
            continue;
        }

        const NGHolder &h = *g[v].left.graph;

        if (g[v].left.lag != tbi.minLiteralLen(v)
            || g[v].left.lag != tbi.maxLiteralLen(v)) {
            continue;
        }

        if (all_reports(h).size() != 1) {
            assert(0);
            continue;
        }

        DEBUG_PRINTF("inspecting prefix of %zu\n", g[v].index);

        if (!proper_out_degree(h.startDs, h)) {
            if (handleStartPrefixCliche(h, g, v, e, ar, &to_delete)) {
                continue;
            }
        } else {
            if (handleStartDsPrefixCliche(h, g, v, e)) {
                continue;
            }
        }

        /* prefix is not just a simple dot repeat. However, it is still
         * possible that it consists of dot repeat and fixed width mask that we
         * can handle. */
        handleMixedPrefixCliche(h, g, v, e, ar, &to_delete, tbi.cc);
    }

    for (const auto &e : out_edges_range(ar, g)) {
        RoseVertex v = target(e, g);

        /* note: vertices that we have rehomed will currently have an in-degree
         * of 2 */
        if (in_degree(v, g) != 1) {
            continue;
        }

        if (!g[v].left.graph) {
            continue;
        }

        if (g[v].left.tracksSom()) {
            continue;
        }

        if (g[v].left.lag != tbi.minLiteralLen(v)
            || g[v].left.lag != tbi.maxLiteralLen(v)) {
            continue;
        }

        const NGHolder &h = *g[v].left.graph;
        if (all_reports(h).size() != 1) {
            assert(0);
            continue;
        }

        DEBUG_PRINTF("inspecting prefix of %zu\n", g[v].index);

        if (!proper_out_degree(h.startDs, h)) {
            if (handleStartPrefixCliche(h, g, v, e, ar, &to_delete)) {
                continue;
            }
        } else {
            if (handleStartDsPrefixCliche(h, g, v, e)) {
                continue;
            }
        }

        /* prefix is not just a simple dot repeat. However, it is still
         * possible that it consists of dot repeat and fixed width mask that we
         * can handle. */
        handleMixedPrefixCliche(h, g, v, e, ar, &to_delete, tbi.cc);
    }

    for (const auto &e : to_delete) {
        remove_edge(e, g);
    }
}

/**
 * Identify dot-repeat infixes after fixed-depth literals and convert them to
 * edges with ROSE_ROLE_HISTORY_ANCH history and equivalent bounds.
 */
void convertAnchPrefixToBounds(RoseBuildImpl &tbi) {
    RoseGraph &g = tbi.g;

    for (const auto v : vertices_range(g)) {
        if (!g[v].left) {
            continue;
        }

        DEBUG_PRINTF("vertex %zu\n", g[v].index);

        // This pass runs after makeCastles, so we use the fact that bounded
        // repeat detection has already been done for us.

        if (!g[v].left.castle) {
            DEBUG_PRINTF("not a castle\n");
            continue;
        }

        const CastleProto &castle = *g[v].left.castle;

        if (castle.repeats.size() != 1) {
            DEBUG_PRINTF("too many repeats\n");
            assert(0); // Castles should not have been merged yet.
            continue;
        }

        if (!castle.reach().all()) {
            DEBUG_PRINTF("not dot\n");
            continue;
        }

        if (in_degree(v, g) != 1) {
            DEBUG_PRINTF("too many in-edges\n");
            continue;
        }

        RoseEdge e = *in_edges(v, g).first;
        RoseVertex u = source(e, g);

        if (g[e].history != ROSE_ROLE_HISTORY_NONE) {
            DEBUG_PRINTF("history already set to something other than NONE?\n");
            assert(0);
            continue;
        }

        if (g[u].min_offset != g[u].max_offset) {
            DEBUG_PRINTF("pred not fixed offset\n");
            continue;
        }
        DEBUG_PRINTF("pred is fixed offset, at %u\n", g[u].min_offset);
        assert(g[u].min_offset < ROSE_BOUND_INF);

        size_t lit_length = tbi.minLiteralLen(v);
        if (lit_length != tbi.maxLiteralLen(v)) {
            assert(0);
            DEBUG_PRINTF("variable literal lengths\n");
            continue;
        }

        u32 lag = g[v].left.lag;
        DEBUG_PRINTF("lit_length=%zu, lag=%u\n", lit_length, lag);
        assert(lag <= lit_length);
        depth delay_adj(lit_length - lag);

        const PureRepeat &pr = castle.repeats.begin()->second;
        DEBUG_PRINTF("castle has repeat %s\n", pr.bounds.str().c_str());
        DEBUG_PRINTF("delay adj %u\n", (u32)delay_adj);

        if (delay_adj >= pr.bounds.max) {
            DEBUG_PRINTF("delay adj too large\n");
            continue;
        }

        DepthMinMax bounds(pr.bounds); // copy
        if (delay_adj > bounds.min) {
            bounds.min = depth(0);
        } else {
            bounds.min -= delay_adj;
        }
        bounds.max -= delay_adj;
        setEdgeBounds(g, e, bounds.min, bounds.max.is_finite()
                                            ? (u32)bounds.max
                                            : ROSE_BOUND_INF);
        g[v].left.reset();
    }
}

} // namespace ue2
