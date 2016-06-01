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
#include <vector>
#include <utility>

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

// Returns the first and last vertices.
static
pair<NFAVertex, NFAVertex> addLiteralVertices(const RoseGraph &g,
                                              const RoseLiteralMap &literals,
                                              const RoseVertex &t_v,
                                              NGHolder &out) {
    // We have limited cases that we support: one literal of arbitrary length,
    // or a bunch of literals of length one that just become a vertex with
    // their reach unioned together.

    // TODO: generalise this and handle more cases.

    const auto &litids = g[t_v].literals;
    if (litids.size() > 1) {
        // Multiple literals of len 1.
        CharReach v_cr;
        for (const auto &lit_id : litids) {
            const rose_literal_id &litv = literals.right.at(lit_id);
            assert(litv.s.length() == 1);
            v_cr |= *litv.s.begin();
        }

        NFAVertex v = addHolderVertex(v_cr, out);
        return make_pair(v, v);
    }

    // Otherwise, we have a single literal, could be of arbitrary length.
    assert(litids.size() == 1);
    u32 lit_id = *(litids.begin());
    const rose_literal_id &litv = literals.right.at(lit_id);
    assert(!litv.s.empty());

    ue2_literal::const_iterator it = litv.s.begin(), ite = litv.s.end();
    NFAVertex first = addHolderVertex(*it, out), last = first;
    for (++it; it != ite; ++it) {
        NFAVertex v = addHolderVertex(*it, out);
        add_edge(last, v, out);
        last = v;
    }

    return make_pair(first, last);
}

static
unique_ptr<NGHolder> convertLeafToHolder(const RoseGraph &g,
                                         const RoseEdge &t_e,
                                         const RoseLiteralMap &literals) {
    RoseVertex t_v = target(t_e, g); // leaf vertex for demolition.
    u32 minBound = g[t_e].minBound;
    u32 maxBound = g[t_e].maxBound;

    const CharReach dot = CharReach::dot();

    assert(!g[t_v].left);

    auto out = ue2::make_unique<NGHolder>(NFA_SUFFIX);

    // Repeats wired to the start of the graph.
    DEBUG_PRINTF("bounds [%u, %u]\n", minBound, maxBound);
    u32 i = 1;
    NFAVertex last = out->start;
    for (; i <= minBound; i++) {
        NFAVertex v = addHolderVertex(dot, *out);
        add_edge(last, v, *out);
        last = v;
    }
    NFAVertex last_mand = last;
    if (maxBound != ROSE_BOUND_INF) {
        for (; i <= maxBound; i++) {
            NFAVertex v = addHolderVertex(dot, *out);
            add_edge(last_mand, v, *out);
            if (last != last_mand) {
                add_edge(last, v, *out);
            }
            last = v;
        }
    } else {
        if (minBound) {
            add_edge(last_mand, last_mand, *out);
        } else {
            NFAVertex v = addHolderVertex(dot, *out);
            add_edge(last_mand, v, *out);
            add_edge(v, v, *out);
            last = v;
        }
    }

    // Literal vertices wired to accept.
    NFAVertex litfirst, litlast;
    tie(litfirst, litlast) = addLiteralVertices(g, literals, t_v, *out);
    add_edge(last, litfirst, *out);
    if (last != last_mand) {
        add_edge(last_mand, litfirst, *out);
    }
    add_edge(litlast, out->accept, *out);
    insert(&(*out)[litlast].reports, g[t_v].reports);
    return out;
}

static
bool areLiteralsConvertible(const RoseLiteralMap &literals,
                            const flat_set<u32> &ids) {
    // Every literal in v must have the same length.

    // TODO: at the moment, we only handle two cases in construction: (a) one
    // literal of arbitrary length, and (b) many literals, but all with length
    // 1.

    if (ids.empty()) {
        return false;
    }

    auto it = ids.begin(), ite = ids.end();
    const size_t len = literals.right.at(*it).elength();

    // Note: len may be 0 for cases with special literals, like EOD prefixes.

    if (len != 1 && ids.size() != 1) {
        DEBUG_PRINTF("more than one literal of len > 1\n");
        return false;
    }

    // Check the others all have the same length.
    while (++it != ite) {
        if (literals.right.at(*it).elength() != len) {
            DEBUG_PRINTF("literals have different lengths\n");
            return false;
        }
    }

    return true;
}

// Returns true if the given vertex doesn't qualify as a bad leaf to be eaten
// by an NFA.
static
bool isUnconvertibleLeaf(const RoseBuildImpl &tbi, const RoseVertex v) {
    const RoseGraph &g = tbi.g;

    if (in_degree(v, g) != 1) {
        DEBUG_PRINTF("more than one in-edge\n");
        return true;
    }

    const RoseEdge &e = *(in_edges(v, g).first);
    RoseVertex u = source(e, g);

    if (!g[u].reports.empty()) {
        DEBUG_PRINTF("pred has accept\n");
        return true;
    }

    if (g[u].suffix) {
        // TODO: this could be handled by adding new vertices to the existing
        // suffix.
        DEBUG_PRINTF("pred already has suffix\n");
        return true;
    }

    if (tbi.isAnyStart(u)) {
        DEBUG_PRINTF("fail start\n");
        return true;
    }

    if (tbi.isAnchored(u)) {
        /* TODO need to check for possible anchored queue overflow? maybe? */
        DEBUG_PRINTF("fail anchored\n");
        return true;
    }

    if (g[v].reports.empty() || g[v].eod_accept) {
        DEBUG_PRINTF("bad accept\n");
        return true;
    }

    if (g[v].suffix) {
        DEBUG_PRINTF("suffix\n");
        return true;
    }

    if (g[v].left) {
        /* TODO: we really should handle this case as we would be checking
         * an nfa each time. However it requires completely different graph
         * fiddling logic */
        DEBUG_PRINTF("rose prefix action\n");
        return true;
    }

    if (!areLiteralsConvertible(tbi.literals, g[v].literals)) {
        DEBUG_PRINTF("fail length\n");
        return true;
    }

    u32 max_lit_len = tbi.maxLiteralLen(v);

    u32 maxbound = max_lit_len == 1 ? 124 : 32; // arbitrary magic numbers
    if (g[e].maxBound > maxbound && g[e].maxBound != ROSE_BOUND_INF) {
        DEBUG_PRINTF("fail maxbound (%u)\n", maxbound);
        return true;
    }

    if (g[e].maxBound == ROSE_BOUND_INF) {
        /* slightly risky as nfa won't die */
        DEBUG_PRINTF("fail: .*\n");
        return true;
    }

    return false;
}

// Find all of the leaves with literals whose length is <= len.
static
void findBadLeaves(RoseBuildImpl &tbi, RoseVertexSet &bad) {
    RoseGraph &g = tbi.g;
    u32 len = tbi.cc.grey.roseMaxBadLeafLength;

    for (const auto &m : tbi.literals.right) {
        if (m.second.s.length() > len) {
            continue;
        }
        u32 lid = m.first;
        DEBUG_PRINTF("%u is a short lit (length %zu)\n", lid,
                     m.second.s.length());

        if (tbi.isDelayed(lid)) {
            DEBUG_PRINTF("delayed, skipping!\n");
            continue;
        }

        const rose_literal_info &info = tbi.literal_info[lid];

        // Because we do the "clone pred and re-home" trick below, we need to
        // iterate over our vertices in a defined ordering, otherwise we'll get
        // non-determinism in our bytecode. So, copy and sort this literal's
        // vertices.

        vector<RoseVertex> verts(info.vertices.begin(), info.vertices.end());
        sort(verts.begin(), verts.end(), VertexIndexComp(g));

        for (auto v : verts) {
            if (!isLeafNode(v, g)) {
                continue;
            }
            if (isUnconvertibleLeaf(tbi, v)) {
                continue; // we don't want to touch it
            }

            // This leaf may have a predecessor with more than one successor,
            // in which case we want to clone the pred just to support this
            // leaf.
            const RoseEdge &e = *in_edges(v, g).first;
            RoseVertex u = source(e, g);
            if (out_degree(u, g) != 1) {
                DEBUG_PRINTF("re-homing %zu to cloned pred\n", g[v].idx);
                RoseVertex u2 = tbi.cloneVertex(u);
                for (const auto &e_in : in_edges_range(u, g)) {
                    add_edge(source(e_in, g), u2, g[e_in], g);
                }
                add_edge(u2, v, g[e], g);
                remove_edge(e, g);
            }

            DEBUG_PRINTF("%zu is a bad leaf vertex\n", g[v].idx);
            bad.insert(v);
        }
    }
}

void convertBadLeaves(RoseBuildImpl &tbi) {
    RoseGraph &g = tbi.g;
    RoseVertexSet bad(g);
    findBadLeaves(tbi, bad);
    DEBUG_PRINTF("found %zu bad leaves\n", bad.size());

    if (bad.empty()) {
        return;
    }

    vector<RoseVertex> dead;
    for (auto v : bad) {
        assert(in_degree(v, g));

        const RoseEdge &e = *(in_edges(v, g).first);

        shared_ptr<NGHolder> h = convertLeafToHolder(g, e, tbi.literals);
        if (num_vertices(*h) >= NFA_MAX_STATES) {
            assert(0); // too big!
            continue;
        }

        RoseVertex u = source(e, g);
        assert(!g[u].suffix);
        g[u].suffix.graph = h;
        DEBUG_PRINTF("%zu's nfa holder %p\n", g[u].idx, h.get());

        dead.push_back(v);
    }

    tbi.removeVertices(dead);
}

static
size_t suffixFloodLen(const ue2_literal &s) {
    if (s.empty()) {
        return 0;
    }

    const ue2_literal::elem &c = s.back();
    auto it = find_if(s.rbegin(), s.rend(),
                      bind2nd(not_equal_to<ue2_literal::elem>(), c));
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
        add_edge(u, v, *h);
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
    setReportId(*h, prefix_report);

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
    for (const rose_literal_id &lit : tbi.literals.right | map_values) {
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
        const rose_literal_id &lit = tbi.literals.right.at(lit_id);

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
        g[e_old].minBound = bound_min;
        g[e_old].maxBound = bound_max;
        g[e_old].history = ROSE_ROLE_HISTORY_ANCH;
    } else {
        RoseEdge e_new;
        UNUSED bool added;
        tie(e_new, added) = add_edge(ar, v, g);
        assert(added);
        g[e_new].minBound = bound_min;
        g[e_new].maxBound = bound_max;
        g[e_new].history = ROSE_ROLE_HISTORY_ANCH;

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

    set<NFAVertex> start_succ;
    set<NFAVertex> startds_succ;
    succ(h, h.start, &start_succ);
    succ(h, h.startDs, &startds_succ);

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
    g[e].minBound = repeatCount;
    g[e].maxBound = ROSE_BOUND_INF;
    g[e].history = ROSE_ROLE_HISTORY_ANCH;

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
    NFAVertex key = nullptr;
    NFAVertex base = anchored ? h.start : h.startDs;

    if (!anchored) {
        set<NFAVertex> start_succ;
        set<NFAVertex> startds_succ;
        succ(h, h.start, &start_succ);
        succ(h, h.startDs, &startds_succ);

        if (!is_subset_of(start_succ, startds_succ)) {
            DEBUG_PRINTF("not a simple chain\n");
            return false;
        }
    }

    for (auto w : adjacent_vertices_range(base, h)) {
        DEBUG_PRINTF("checking %u\n", h[w].index);
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
        DEBUG_PRINTF("repeat vertex %u\n", h[repeat_v].index);
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

    set<NFAVertex> base_succ;
    succ(h, base, &base_succ);
    base_succ.erase(h.startDs);

    if (is_subset_of(base_succ, rep_verts)) {
        /* all good: repeat dominates the rest of the pattern */
    } else if (ri.repeatMin == depth(1)
               && is_subset_of(exits, base_succ)
               && is_subset_of(base_succ, exits_and_repeat_verts)) {
        /* we have a jump edge */
        ri.repeatMin = 0;
    } else {
        return false;
    }

    DEBUG_PRINTF("repeat {%s,%s}\n", ri.repeatMin.str().c_str(),
                 ri.repeatMax.str().c_str());
    DEBUG_PRINTF("woot?\n");

    shared_ptr<NGHolder> h_new = make_shared<NGHolder>();
    ue2::unordered_map<NFAVertex, NFAVertex> rhs_map;
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
            g[e_old].minBound = ri.repeatMin + width;
            g[e_old].maxBound = ri.repeatMax + width;
            g[e_old].history = ROSE_ROLE_HISTORY_ANCH;
        } else {
            RoseEdge e_new;
            UNUSED bool added;
            tie(e_new, added) = add_edge(ar, v, g);
            assert(added);
            g[e_new].minBound = ri.repeatMin + width;
            g[e_new].maxBound = ri.repeatMax + width;
            g[e_new].history = ROSE_ROLE_HISTORY_ANCH;

            to_delete->push_back(e_old);
        }

    } else {
        assert(g[e_old].minBound <= ri.repeatMin + width);
        g[e_old].minBound = ri.repeatMin + width;
        g[e_old].maxBound = ROSE_BOUND_INF;
        g[e_old].history = ROSE_ROLE_HISTORY_ANCH;
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

        DEBUG_PRINTF("inspecting prefix of %zu\n", g[v].idx);

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

        DEBUG_PRINTF("inspecting prefix of %zu\n", g[v].idx);

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

        DEBUG_PRINTF("vertex %zu\n", g[v].idx);

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
            bounds.min = 0;
        } else {
            bounds.min -= delay_adj;
        }
        bounds.max -= delay_adj;

        g[e].minBound = bounds.min;
        g[e].maxBound =
            bounds.max.is_finite() ? (u32)bounds.max : ROSE_BOUND_INF;

        // It's possible that a (0,inf) case might sneak through here, in which
        // case we don't need ANCH history at all.
        if (g[e].minBound == 0 && g[e].maxBound == ROSE_BOUND_INF) {
            g[e].history = ROSE_ROLE_HISTORY_NONE;
        } else {
            g[e].history = ROSE_ROLE_HISTORY_ANCH;
        }

        g[v].left.reset();
    }
}

} // namespace ue2
