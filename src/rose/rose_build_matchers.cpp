/*
 * Copyright (c) 2016-2017, Intel Corporation
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

/**
 * \file
 * \brief Rose build: code for constructing literal tables.
 */

#include "rose_build_matchers.h"

#include "rose_build_dump.h"
#include "rose_build_impl.h"
#include "rose_build_lit_accel.h"
#include "rose_build_width.h"
#include "hwlm/hwlm_build.h"
#include "hwlm/hwlm_internal.h"
#include "hwlm/hwlm_literal.h"
#include "nfa/castlecompile.h"
#include "nfa/nfa_api_queue.h"
#include "util/charreach_util.h"
#include "util/compile_context.h"
#include "util/compile_error.h"
#include "util/dump_charclass.h"
#include "util/report.h"
#include "util/report_manager.h"
#include "util/verify_types.h"
#include "ue2common.h"

#include <iomanip>
#include <sstream>

#include <boost/range/adaptor/map.hpp>

using namespace std;
using boost::adaptors::map_values;

namespace ue2 {

static const size_t MAX_ACCEL_STRING_LEN = 16;

#ifdef DEBUG
static UNUSED
string dumpMask(const vector<u8> &v) {
    ostringstream oss;
    for (u8 e : v) {
        oss << setfill('0') << setw(2) << hex << (unsigned int)e;
    }
    return oss.str();
}
#endif

static
bool maskFromLeftGraph(const LeftEngInfo &left, vector<u8> &msk,
                       vector<u8> &cmp) {
    const u32 lag = left.lag;
    const ReportID report = left.leftfix_report;

    DEBUG_PRINTF("leftfix with lag %u, report %u\n", lag, report);

    assert(left.graph);
    const NGHolder &h = *left.graph;
    assert(in_degree(h.acceptEod, h) == 1); // no eod reports

    // Start with the set of reporter vertices for this leftfix.
    set<NFAVertex> curr;
    for (auto u : inv_adjacent_vertices_range(h.accept, h)) {
        if (contains(h[u].reports, report)) {
            curr.insert(u);
        }
    }
    assert(!curr.empty());

    size_t i = HWLM_MASKLEN - lag - 1;
    do {
        if (curr.empty() || contains(curr, h.start)
            || contains(curr, h.startDs)) {
            DEBUG_PRINTF("end of the road\n");
            break;
        }

        set<NFAVertex> next;
        CharReach cr;
        for (NFAVertex v : curr) {
            const auto &v_cr = h[v].char_reach;
            DEBUG_PRINTF("vertex %zu, reach %s\n", h[v].index,
                         describeClass(v_cr).c_str());
            cr |= v_cr;
            insert(&next, inv_adjacent_vertices(v, h));
        }
        make_and_cmp_mask(cr, &msk.at(i), &cmp.at(i));
        DEBUG_PRINTF("%zu: reach=%s, msk=%u, cmp=%u\n", i,
                     describeClass(cr).c_str(), msk[i], cmp[i]);
        curr.swap(next);
    } while (i-- > 0);

    return true;
}

static
bool maskFromLeftCastle(const LeftEngInfo &left, vector<u8> &msk,
                        vector<u8> &cmp) {
    const u32 lag = left.lag;
    const ReportID report = left.leftfix_report;

    DEBUG_PRINTF("leftfix with lag %u, report %u\n", lag, report);

    assert(left.castle);
    const CastleProto &c = *left.castle;

    depth min_width(depth::infinity());
    for (const PureRepeat &repeat : c.repeats | map_values) {
        if (contains(repeat.reports, report)) {
            min_width = min(min_width, repeat.bounds.min);
        }
    }

    DEBUG_PRINTF("castle min width for this report is %s\n",
                 min_width.str().c_str());

    if (!min_width.is_finite() || min_width == depth(0)) {
        DEBUG_PRINTF("bad min width\n");
        return false;
    }

    u32 len = min_width;
    u32 end = HWLM_MASKLEN - lag;
    for (u32 i = end; i > end - min(end, len); i--) {
        make_and_cmp_mask(c.reach(), &msk.at(i - 1), &cmp.at(i - 1));
    }

    return true;
}

static
bool maskFromLeft(const LeftEngInfo &left, vector<u8> &msk, vector<u8> &cmp) {
    if (left.lag >= HWLM_MASKLEN) {
        DEBUG_PRINTF("too much lag\n");
        return false;
    }

    if (left.graph) {
        return maskFromLeftGraph(left, msk, cmp);
    } else if (left.castle) {
        return maskFromLeftCastle(left, msk, cmp);
    }

    return false;
}

static
bool maskFromPreds(const RoseBuildImpl &build, const rose_literal_id &id,
                   const RoseVertex v, vector<u8> &msk, vector<u8> &cmp) {
    const RoseGraph &g = build.g;

    // For right now, wuss out and only handle cases with one pred.
    if (in_degree(v, g) != 1) {
        return false;
    }

    // Root successors have no literal before them.
    if (build.isRootSuccessor(v)) {
        return false;
    }

    // If we have a single predecessor with a short bound, we may be able to
    // fill out a mask with the trailing bytes of the previous literal. This
    // allows us to improve literals like the 'bar' in 'fo.bar'.

    RoseEdge e = *(in_edges(v, g).first);
    u32 bound = g[e].maxBound;
    if (bound != g[e].minBound || bound >= HWLM_MASKLEN) {
        return false;
    }

    bound += id.s.length();
    if (bound >= HWLM_MASKLEN) {
        return false;
    }

    DEBUG_PRINTF("bound %u\n", bound);

    RoseVertex u = source(e, g);
    if (g[u].literals.size() != 1) {
        DEBUG_PRINTF("u has %zu literals\n", g[u].literals.size());
        return false;
    }

    u32 u_lit_id = *(g[u].literals.begin());
    const rose_literal_id &u_id = build.literals.at(u_lit_id);
    DEBUG_PRINTF("u has lit: %s\n", escapeString(u_id.s).c_str());

    // Number of characters to take from the back of u's literal.
    size_t u_len = u_id.s.length();
    size_t u_sublen = min(u_len, (size_t)HWLM_MASKLEN - bound);

    size_t i = HWLM_MASKLEN - (bound + u_sublen);

    ue2_literal::const_iterator it, ite;
    for (it = u_id.s.begin() + (u_len - u_sublen), ite = u_id.s.end();
            it != ite; ++it) {
        make_and_cmp_mask(*it, &msk.at(i), &cmp.at(i));
        ++i;
    }

    return true;
}

static
bool findHamsterMask(const RoseBuildImpl &build, const rose_literal_id &id,
                     const rose_literal_info &info, const RoseVertex v,
                     vector<u8> &msk, vector<u8> &cmp) {
    // Start with zero masks.
    msk.assign(HWLM_MASKLEN, 0);
    cmp.assign(HWLM_MASKLEN, 0);

    // Masks can come from literal benefits (for mixed-case literals).
    if (info.requires_benefits) {
        assert(mixed_sensitivity(id.s));

        size_t j = 0;
        for (ue2_literal::const_reverse_iterator it = id.s.rbegin(),
                                                 ite = id.s.rend();
             it != ite && j < HWLM_MASKLEN; ++it, ++j) {
            size_t offset = HWLM_MASKLEN - j - 1;
            const CharReach &cr = *it;
            make_and_cmp_mask(cr, &msk[offset], &cmp[offset]);
        }
        return true;
    }

    const LeftEngInfo &left = build.g[v].left;
    if (left && left.lag < HWLM_MASKLEN) {
        if (maskFromLeft(left, msk, cmp)) {
            DEBUG_PRINTF("mask from a leftfix!\n");
            return true;
        }
    }

    if (id.s.length() < HWLM_MASKLEN) {
        if (maskFromPreds(build, id, v, msk, cmp)) {
            DEBUG_PRINTF("mask from preds!\n");
            return true;
        }
    }

    return false;
}

static
bool hamsterMaskCombine(vector<u8> &msk, vector<u8> &cmp,
                        const vector<u8> &v_msk, const vector<u8> &v_cmp) {
    assert(msk.size() == HWLM_MASKLEN && cmp.size() == HWLM_MASKLEN);
    assert(v_msk.size() == HWLM_MASKLEN && v_cmp.size() == HWLM_MASKLEN);

    u8 all_masks = 0;

    for (size_t i = 0; i < HWLM_MASKLEN; i++) {
        u8 filter = ~(cmp[i] ^ v_cmp[i]);
        msk[i] &= v_msk[i];
        msk[i] &= filter;
        cmp[i] &= filter;

        all_masks |= msk[i];
    }

    // Return false if we have no bits on in any mask elements.
    return all_masks != 0;
}

static
bool findHamsterMask(const RoseBuildImpl &build, const rose_literal_id &id,
                     const rose_literal_info &info,
                     vector<u8> &msk, vector<u8> &cmp) {
    if (!build.cc.grey.roseHamsterMasks) {
        return false;
    }

    if (!info.delayed_ids.empty()) {
        // Not safe to add masks to delayed literals at this late stage.
        return false;
    }

    size_t num = 0;
    vector<u8> v_msk, v_cmp;

    for (RoseVertex v : info.vertices) {
        if (!findHamsterMask(build, id, info, v, v_msk, v_cmp)) {
            DEBUG_PRINTF("no mask\n");
            return false;
        }

        if (!num++) {
            // First (or only) vertex, this becomes the mask/cmp pair.
            msk = v_msk;
            cmp = v_cmp;
        } else {
            // Multiple vertices with potentially different masks. We combine
            // them into an 'advisory' mask.
            if (!hamsterMaskCombine(msk, cmp, v_msk, v_cmp)) {
                DEBUG_PRINTF("mask went to zero\n");
                return false;
            }
        }
    }

    normaliseLiteralMask(id.s, msk, cmp);

    if (msk.empty()) {
        DEBUG_PRINTF("no mask\n");
        return false;
    }

    DEBUG_PRINTF("msk=%s, cmp=%s\n", dumpMask(msk).c_str(),
                 dumpMask(cmp).c_str());
    return true;
}

void findMoreLiteralMasks(RoseBuildImpl &build) {
    if (!build.cc.grey.roseHamsterMasks) {
        return;
    }

    vector<u32> candidates;
    for (u32 id = 0; id < build.literals.size(); id++) {
        const auto &lit = build.literals.at(id);

        if (lit.delay || build.isDelayed(id)) {
            continue;
        }

        // Literal masks are only allowed for literals that will end up in an
        // HWLM table.
        switch (lit.table) {
        case ROSE_FLOATING:
        case ROSE_EOD_ANCHORED:
        case ROSE_ANCHORED_SMALL_BLOCK:
            break;
        default:
            continue;
        }

        if (!lit.msk.empty()) {
            continue;
        }

        const auto &lit_info = build.literal_info.at(id);
        if (lit_info.requires_benefits) {
            continue;
        }
        candidates.push_back(id);
    }

    for (const u32 &id : candidates) {
        const auto &lit = build.literals.at(id);
        auto &lit_info = build.literal_info.at(id);

        vector<u8> msk, cmp;
        if (!findHamsterMask(build, lit, lit_info, msk, cmp)) {
            continue;
        }
        assert(!msk.empty());
        DEBUG_PRINTF("found advisory mask for lit_id=%u (%s)\n", id,
                     dumpString(lit.s).c_str());
        u32 new_id = build.getLiteralId(lit.s, msk, cmp, lit.delay, lit.table);
        assert(new_id != id);
        DEBUG_PRINTF("replacing with new lit_id=%u\n", new_id);

        // Note that our new literal may already exist and have vertices, etc.
        // We assume that this transform is happening prior to group assignment.
        assert(lit_info.group_mask == 0);
        auto &new_info = build.literal_info.at(new_id);

        // Move the vertices across.
        new_info.vertices.insert(begin(lit_info.vertices),
                                 end(lit_info.vertices));
        for (auto v : lit_info.vertices) {
            build.g[v].literals.erase(id);
            build.g[v].literals.insert(new_id);
        }
        lit_info.vertices.clear();

        // Preserve other properties.
        new_info.requires_benefits = lit_info.requires_benefits;
    }
}

static
bool isDirectHighlander(const RoseBuildImpl &build, const u32 id,
                        const rose_literal_info &info) {
    if (!build.isDirectReport(id)) {
        return false;
    }

    auto is_simple_exhaustible = [&build](ReportID rid) {
        const Report &report = build.rm.getReport(rid);
        return isSimpleExhaustible(report);
    };

    assert(!info.vertices.empty());
    for (const auto &v : info.vertices) {
        const auto &reports = build.g[v].reports;
        assert(!reports.empty());
        if (!all_of(begin(reports), end(reports),
                    is_simple_exhaustible)) {
            return false;
        }
    }
    return true;
}

// Called by isNoRunsLiteral below.
static
bool isNoRunsVertex(const RoseBuildImpl &build, RoseVertex u) {
    const RoseGraph &g = build.g;
    if (!g[u].isBoring()) {
        DEBUG_PRINTF("u=%zu is not boring\n", g[u].index);
        return false;
    }

    if (!g[u].reports.empty()) {
        DEBUG_PRINTF("u=%zu has accept\n", g[u].index);
        return false;
    }

    /* TODO: handle non-root roles as well. It can't be that difficult... */

    if (in_degree(u, g) != 1) {
        DEBUG_PRINTF("u=%zu is not a root role\n", g[u].index);
        return false;
    }

    RoseEdge e = edge(build.root, u, g);

    if (!e) {
        DEBUG_PRINTF("u=%zu is not a root role\n", g[u].index);
        return false;
    }

    if (g[e].minBound != 0 || g[e].maxBound != ROSE_BOUND_INF) {
        DEBUG_PRINTF("u=%zu has bounds from root\n", g[u].index);
        return false;
    }

    for (const auto &oe : out_edges_range(u, g)) {
        RoseVertex v = target(oe, g);
        if (g[oe].maxBound != ROSE_BOUND_INF) {
            DEBUG_PRINTF("edge (%zu,%zu) has max bound\n", g[u].index,
                         g[v].index);
            return false;
        }
        if (g[v].left) {
            DEBUG_PRINTF("v=%zu has rose prefix\n", g[v].index);
            return false;
        }
    }
    return true;
}

static
bool isNoRunsLiteral(const RoseBuildImpl &build, const u32 id,
                     const rose_literal_info &info, const size_t max_len) {
    DEBUG_PRINTF("lit id %u\n", id);

    if (info.requires_benefits) {
        DEBUG_PRINTF("requires benefits\n"); // which would need confirm
        return false;
    }

    size_t len = build.literals.at(id).s.length();
    if (len > max_len) {
        DEBUG_PRINTF("long literal, requires confirm\n");
        return false;
    }

    if (len > ROSE_SHORT_LITERAL_LEN_MAX) {
        DEBUG_PRINTF("medium-length literal, requires confirm\n");
        return false;
    }

    if (isDirectHighlander(build, id, info)) {
        DEBUG_PRINTF("highlander direct report\n");
        return true;
    }

    // Undelayed vertices.
    for (RoseVertex v : info.vertices) {
        if (!isNoRunsVertex(build, v)) {
            return false;
        }
    }

    // Delayed vertices.
    for (u32 d : info.delayed_ids) {
        assert(d < build.literal_info.size());
        const rose_literal_info &delayed_info = build.literal_info.at(d);
        assert(delayed_info.undelayed_id == id);
        for (RoseVertex v : delayed_info.vertices) {
            if (!isNoRunsVertex(build, v)) {
                return false;
            }
        }
    }

    DEBUG_PRINTF("is no-runs literal\n");
    return true;
}

static
const raw_puff &getChainedPuff(const RoseBuildImpl &build,
                               const Report &report) {
    DEBUG_PRINTF("chained report, event %u\n", report.onmatch);

    // MPV has already been moved to the outfixes vector.
    assert(!build.mpv_outfix);

    auto mpv_outfix_it = find_if(
        begin(build.outfixes), end(build.outfixes),
        [](const OutfixInfo &outfix) { return outfix.is_nonempty_mpv(); });
    assert(mpv_outfix_it != end(build.outfixes));
    const auto *mpv = mpv_outfix_it->mpv();

    u32 puff_index = report.onmatch - MQE_TOP_FIRST;
    assert(puff_index < mpv->triggered_puffettes.size());
    return mpv->triggered_puffettes.at(puff_index);
}

/**
 * \brief Returns a conservative estimate of the minimum offset at which the
 * given literal can lead to a report.
 *
 * TODO: This could be made more precise by calculating a "distance to accept"
 * for every vertex in the graph; right now we're only accurate for leaf nodes.
 */
static
u64a literalMinReportOffset(const RoseBuildImpl &build,
                           const rose_literal_id &lit,
                           const rose_literal_info &info) {
    const auto &g = build.g;

    const u32 lit_len = verify_u32(lit.elength());

    u64a lit_min_offset = UINT64_MAX;

    for (const auto &v : info.vertices) {
        DEBUG_PRINTF("vertex %zu min_offset=%u\n", g[v].index, g[v].min_offset);

        u64a vert_offset = g[v].min_offset;

        if (vert_offset >= lit_min_offset) {
            continue;
        }

        u64a min_offset = UINT64_MAX;

        for (const auto &id : g[v].reports) {
            const Report &report = build.rm.getReport(id);
            DEBUG_PRINTF("report id %u, min offset=%llu\n", id,
                         report.minOffset);
            if (report.type == INTERNAL_ROSE_CHAIN) {
                // This vertex triggers an MPV, which will fire reports after
                // repeating for a while.
                assert(report.minOffset == 0); // Should not have bounds.
                const auto &puff = getChainedPuff(build, report);
                DEBUG_PRINTF("chained puff repeats=%u\n", puff.repeats);
                const Report &puff_report = build.rm.getReport(puff.report);
                DEBUG_PRINTF("puff report %u, min offset=%llu\n", puff.report,
                              puff_report.minOffset);
                min_offset = min(min_offset, max(vert_offset + puff.repeats,
                                                 puff_report.minOffset));
            } else {
                DEBUG_PRINTF("report min offset=%llu\n", report.minOffset);
                min_offset = min(min_offset, max(vert_offset,
                                                 report.minOffset));
            }
        }

        if (g[v].suffix) {
            depth suffix_width = findMinWidth(g[v].suffix, g[v].suffix.top);
            assert(suffix_width.is_reachable());
            DEBUG_PRINTF("suffix with width %s\n", suffix_width.str().c_str());
            min_offset = min(min_offset, vert_offset + suffix_width);
        }

        if (!isLeafNode(v, g) || min_offset == UINT64_MAX) {
            min_offset = vert_offset;
        }

        lit_min_offset = min(lit_min_offset, min_offset);
    }

    // If this literal in the undelayed literal corresponding to some delayed
    // literals, we must take their minimum offsets into account.
    for (const u32 &delayed_id : info.delayed_ids) {
        const auto &delayed_lit = build.literals.at(delayed_id);
        const auto &delayed_info = build.literal_info.at(delayed_id);
        u64a delayed_min_offset = literalMinReportOffset(build, delayed_lit,
                                                         delayed_info);
        DEBUG_PRINTF("delayed_id=%u, min_offset = %llu\n", delayed_id,
                     delayed_min_offset);
        lit_min_offset = min(lit_min_offset, delayed_min_offset);
    }

    // If we share a vertex with a shorter literal, our min offset might dip
    // below the length of this one.
    lit_min_offset = max(lit_min_offset, u64a{lit_len});

    return lit_min_offset;
}

template<class Container>
void trim_to_suffix(Container &c, size_t len) {
    if (c.size() <= len) {
        return;
    }

    size_t suffix_len = c.size() - len;
    c.erase(c.begin(), c.begin() + suffix_len);
}

namespace {

/** \brief Prototype for literal matcher construction. */
struct MatcherProto {
    /** \brief Literal fragments used to construct the literal matcher. */
    vector<hwlmLiteral> lits;

    /** \brief Longer literals used for acceleration analysis. */
    vector<AccelString> accel_lits;

    /** \brief The history required by the literal matcher. */
    size_t history_required = 0;

    /** \brief Insert the contents of another MatcherProto. */
    void insert(const MatcherProto &a);
};
}

/**
 * \brief Build up a vector of literals (and associated other data) for the
 * given table.
 *
 * If max_offset is specified (and not ROSE_BOUND_INF), then literals that can
 * only lead to a pattern match after max_offset may be excluded.
 */
static
MatcherProto makeMatcherProto(const RoseBuildImpl &build,
                              const vector<LitFragment> &fragments,
                              rose_literal_table table, bool delay_rebuild,
                              size_t max_len, u32 max_offset = ROSE_BOUND_INF) {
    MatcherProto mp;

    if (delay_rebuild) {
        assert(table == ROSE_FLOATING);
        assert(build.cc.streaming);
    }

    for (const auto &f : fragments) {
        for (u32 id : f.lit_ids) {
            const rose_literal_id &lit = build.literals.at(id);

            if (lit.table != table) {
                continue; /* wrong table */
            }

            if (lit.delay) {
                continue;  /* delay id's are virtual-ish */
            }

            assert(id < build.literal_info.size());
            const auto &info = build.literal_info.at(id);

            /* Note: requires_benefits are handled in the literal entries */
            const ue2_literal &s = lit.s;

            DEBUG_PRINTF("lit='%s' (len %zu)\n", escapeString(s).c_str(),
                         s.length());

            // When building the delay rebuild table, we only want to include
            // literals that have delayed variants.
            if (delay_rebuild && info.delayed_ids.empty()) {
                DEBUG_PRINTF("not needed for delay rebuild\n");
                continue;
            }

            if (max_offset != ROSE_BOUND_INF) {
                u64a min_report = literalMinReportOffset(build, lit, info);
                if (min_report > max_offset) {
                    DEBUG_PRINTF("min report offset=%llu exceeds "
                                 "max_offset=%u\n", min_report, max_offset);
                    continue;
                }
            }

            const vector<u8> &msk = lit.msk;
            const vector<u8> &cmp = lit.cmp;
            bool noruns = isNoRunsLiteral(build, id, info, max_len);

            size_t lit_hist_len = 0;
            if (build.cc.streaming) {
                lit_hist_len = max(msk.size(), min(s.length(), max_len));
                lit_hist_len = lit_hist_len ? lit_hist_len - 1 : 0;
            }
            DEBUG_PRINTF("lit requires %zu bytes of history\n", lit_hist_len);
            assert(lit_hist_len <= build.cc.grey.maxHistoryAvailable);

            auto lit_final = s; // copy

            if (lit_final.length() > ROSE_SHORT_LITERAL_LEN_MAX) {
                DEBUG_PRINTF("truncating to tail of length %zu\n",
                             size_t{ROSE_SHORT_LITERAL_LEN_MAX});
                lit_final.erase(0, lit_final.length()
                                - ROSE_SHORT_LITERAL_LEN_MAX);
                // We shouldn't have set a threshold below 8 chars.
                assert(msk.size() <= ROSE_SHORT_LITERAL_LEN_MAX);
                assert(!noruns);
            }

            const auto &s_final = lit_final.get_string();
            bool nocase = lit_final.any_nocase();

            DEBUG_PRINTF("id=%u, s='%s', nocase=%d, noruns=%d, msk=%s, "
                         "cmp=%s\n", f.fragment_id,
                         escapeString(s_final).c_str(), (int)nocase, noruns,
                         dumpMask(msk).c_str(), dumpMask(cmp).c_str());

            if (!maskIsConsistent(s_final, nocase, msk, cmp)) {
                DEBUG_PRINTF("msk/cmp for literal can't match, skipping\n");
                continue;
            }

            mp.accel_lits.emplace_back(s.get_string(), s.any_nocase(), msk, cmp,
                                       info.group_mask);
            mp.history_required = max(mp.history_required, lit_hist_len);

            u32 prog_offset = delay_rebuild ? f.delay_program_offset
                                            : f.lit_program_offset;
            const auto &groups = f.groups;

            mp.lits.emplace_back(move(s_final), nocase, noruns, prog_offset,
                                 groups, msk, cmp);
        }
    }

    sort_and_unique(mp.lits);

    // Literals used for acceleration must be limited to max_len, as that's all
    // we can see in history.
    for_each(begin(mp.accel_lits), end(mp.accel_lits),
             [&max_len](AccelString &a) {
                 trim_to_suffix(a.s, max_len);
                 trim_to_suffix(a.msk, max_len);
                 trim_to_suffix(a.cmp, max_len);
             });

    sort_and_unique(mp.accel_lits);

    return mp;
}

void MatcherProto::insert(const MatcherProto &a) {
    ::ue2::insert(&lits, lits.end(), a.lits);
    ::ue2::insert(&accel_lits, accel_lits.end(), a.accel_lits);
    sort_and_unique(lits);
    sort_and_unique(accel_lits);
    history_required = max(history_required, a.history_required);
}

static
void buildAccel(const RoseBuildImpl &build, const MatcherProto &mp,
                HWLM &hwlm) {
    if (!build.cc.grey.hamsterAccelForward) {
        return;
    }

    if (hwlm.type == HWLM_ENGINE_NOOD) {
        return;
    }

    buildForwardAccel(&hwlm, mp.accel_lits, build.getInitialGroups());
}

bytecode_ptr<HWLM> buildFloatingMatcher(const RoseBuildImpl &build,
                                        const vector<LitFragment> &fragments,
                                        size_t longLitLengthThreshold,
                                        rose_group *fgroups,
                                        size_t *historyRequired) {
    *fgroups = 0;

    auto mp = makeMatcherProto(build, fragments, ROSE_FLOATING, false,
                               longLitLengthThreshold);
    if (mp.lits.empty()) {
        DEBUG_PRINTF("empty floating matcher\n");
        return nullptr;
    }
    dumpMatcherLiterals(mp.lits, "floating", build.cc.grey);

    for (const hwlmLiteral &lit : mp.lits) {
        *fgroups |= lit.groups;
    }

    auto hwlm = hwlmBuild(mp.lits, false, build.cc, build.getInitialGroups());
    if (!hwlm) {
        throw CompileError("Unable to generate bytecode.");
    }

    buildAccel(build, mp, *hwlm);

    if (build.cc.streaming) {
        DEBUG_PRINTF("history_required=%zu\n", mp.history_required);
        assert(mp.history_required <= build.cc.grey.maxHistoryAvailable);
        *historyRequired = max(*historyRequired, mp.history_required);
    }

    DEBUG_PRINTF("built floating literal table size %zu bytes\n", hwlm.size());
    return hwlm;
}

bytecode_ptr<HWLM>
buildDelayRebuildMatcher(const RoseBuildImpl &build,
                         const vector<LitFragment> &fragments,
                         size_t longLitLengthThreshold) {
    if (!build.cc.streaming) {
        DEBUG_PRINTF("not streaming\n");
        return nullptr;
    }

    auto mp = makeMatcherProto(build, fragments, ROSE_FLOATING, true,
                               longLitLengthThreshold);
    if (mp.lits.empty()) {
        DEBUG_PRINTF("empty delay rebuild matcher\n");
        return nullptr;
    }
    dumpMatcherLiterals(mp.lits, "delay_rebuild", build.cc.grey);

    auto hwlm = hwlmBuild(mp.lits, false, build.cc, build.getInitialGroups());
    if (!hwlm) {
        throw CompileError("Unable to generate bytecode.");
    }

    buildAccel(build, mp, *hwlm);

    DEBUG_PRINTF("built delay rebuild table size %zu bytes\n", hwlm.size());
    return hwlm;
}

bytecode_ptr<HWLM>
buildSmallBlockMatcher(const RoseBuildImpl &build,
                       const vector<LitFragment> &fragments) {
    if (build.cc.streaming) {
        DEBUG_PRINTF("streaming mode\n");
        return nullptr;
    }

    u32 float_min = findMinWidth(build, ROSE_FLOATING);
    if (float_min > ROSE_SMALL_BLOCK_LEN) {
        DEBUG_PRINTF("floating table has large min width %u, fail\n",
                     float_min);
        return nullptr;
    }

    auto mp = makeMatcherProto(build, fragments, ROSE_FLOATING, false,
                               ROSE_SMALL_BLOCK_LEN, ROSE_SMALL_BLOCK_LEN);
    if (mp.lits.empty()) {
        DEBUG_PRINTF("no floating table\n");
        return nullptr;
    } else if (mp.lits.size() == 1) {
        DEBUG_PRINTF("single floating literal, noodle will be fast enough\n");
        return nullptr;
    }

    auto mp_anchored = makeMatcherProto(build, fragments,
                                        ROSE_ANCHORED_SMALL_BLOCK, false,
                                        ROSE_SMALL_BLOCK_LEN,
                                        ROSE_SMALL_BLOCK_LEN);
    if (mp_anchored.lits.empty()) {
        DEBUG_PRINTF("no small-block anchored literals\n");
        return nullptr;
    }

    mp.insert(mp_anchored);
    dumpMatcherLiterals(mp.lits, "smallblock", build.cc.grey);

    // None of our literals should be longer than the small block limit.
    assert(all_of(begin(mp.lits), end(mp.lits), [](const hwlmLiteral &lit) {
        return lit.s.length() <= ROSE_SMALL_BLOCK_LEN;
    }));

    if (mp.lits.empty()) {
        DEBUG_PRINTF("no literals shorter than small block len\n");
        return nullptr;
    }

    auto hwlm = hwlmBuild(mp.lits, true, build.cc, build.getInitialGroups());
    if (!hwlm) {
        throw CompileError("Unable to generate bytecode.");
    }

    buildAccel(build, mp, *hwlm);

    DEBUG_PRINTF("built small block literal table size %zu bytes\n",
                 hwlm.size());
    return hwlm;
}

bytecode_ptr<HWLM>
buildEodAnchoredMatcher(const RoseBuildImpl &build,
                        const vector<LitFragment> &fragments) {
    auto mp = makeMatcherProto(build, fragments, ROSE_EOD_ANCHORED, false,
                               build.ematcher_region_size);

    if (mp.lits.empty()) {
        DEBUG_PRINTF("no eod anchored literals\n");
        assert(!build.ematcher_region_size);
        return nullptr;
    }
    dumpMatcherLiterals(mp.lits, "eod", build.cc.grey);

    assert(build.ematcher_region_size);

    auto hwlm = hwlmBuild(mp.lits, true, build.cc, build.getInitialGroups());
    if (!hwlm) {
        throw CompileError("Unable to generate bytecode.");
    }

    buildAccel(build, mp, *hwlm);

    DEBUG_PRINTF("built eod-anchored literal table size %zu bytes\n",
                 hwlm.size());
    return hwlm;
}

} // namespace ue2
