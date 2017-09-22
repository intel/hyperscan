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

#include "rose_build_impl.h"

#include "ue2common.h"
#include "grey.h"
#include "rose_build_add_internal.h"
#include "rose_build_anchored.h"
#include "rose_in_util.h"
#include "hwlm/hwlm_literal.h"
#include "nfagraph/ng_depth.h"
#include "nfagraph/ng_dump.h"
#include "nfagraph/ng_holder.h"
#include "nfagraph/ng_limex.h"
#include "nfagraph/ng_reports.h"
#include "nfagraph/ng_util.h"
#include "nfagraph/ng_width.h"
#include "util/charreach.h"
#include "util/charreach_util.h"
#include "util/compare.h"
#include "util/compile_context.h"
#include "util/container.h"
#include "util/dump_charclass.h"
#include "util/graph.h"
#include "util/make_unique.h"
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

#define MIN_MASK_LIT_LEN     2
#define MAX_MASK_SIZE      255
#define MAX_MASK_LITS       30

static
void findMaskLiteral(const vector<CharReach> &mask, bool streaming,
                     ue2_literal *lit, u32 *offset, const Grey &grey) {
    bool case_fixed = false;
    bool nocase = false;

    size_t best_begin = 0;
    size_t best_end = 0;
    size_t best_len = 0;

    size_t begin = 0;
    size_t end = 0;

    for (size_t i = 0; i < mask.size(); i++) {
        bool fail = false;
        if (mask[i].count() != 1 && !mask[i].isCaselessChar()) {
            DEBUG_PRINTF("hit non-literal char, resetting at %zu\n", i);
            fail = true;
        }

        if (!fail && streaming && (end >= grey.maxHistoryAvailable + 1)) {
            DEBUG_PRINTF("hit literal limit, resetting at %zu\n", i);
            fail = true;
        }

        if (!fail && case_fixed && mask[i].isAlpha()) {
            if (nocase && mask[i].count() != 2) {
                fail = true;
            }

            if (!nocase && mask[i].count() != 1) {
                fail = true;
            }
        }

        if (fail) {
            case_fixed = false;
            nocase = false;
            size_t len = end - begin;
            bool better = len > best_len;
            if (better) {
                best_begin = begin;
                best_end = end;
                best_len = len;
            }
            begin = i + 1;
            end = i + 1;
        } else {
            assert(end == i);
            end = i + 1;

            if (mask[i].isAlpha()) {
                case_fixed = true;
                nocase = mask[i].count() == 2;
            }
        }
    }

    size_t len = end - begin;
    /* Everybody would rather be trigger towards the end */
    bool better = len >= best_len && mask.size() - end <= MAX_DELAY;

    if (better) {
        best_begin = begin;
        best_end = end;
        best_len = len;
    }

    for (size_t i = best_begin; i < best_end; i++) {
        assert(mask[i].count() == 1 || mask[i].count() == 2);
        lit->push_back(mask[i].find_first(), mask[i].count() > 1);
    }

    *offset = verify_u32(best_begin);
}

static
bool initFmlCandidates(const CharReach &cr, vector<ue2_literal> &cand) {
    for (size_t i = cr.find_first(); i != cr.npos; i = cr.find_next(i)) {
        char c = (char)i;
        bool nocase = myisupper(c) && cr.test(mytolower(c));
        if (myislower(c) && cr.test(mytoupper(c))) {
            continue;
        }

        if (cand.size() >= MAX_MASK_LITS) {
            DEBUG_PRINTF("hit lit limit of %u\n", MAX_MASK_LITS);
            return false;
        }

        cand.emplace_back(c, nocase);
    }

    assert(cand.size() <= MAX_MASK_LITS);
    return !cand.empty();
}

static
bool expandFmlCandidates(const CharReach &cr, vector<ue2_literal> &curr,
                         vector<ue2_literal> &cand) {
    DEBUG_PRINTF("expanding string with cr of %zu\n", cr.count());
    DEBUG_PRINTF("  current cand list size %zu\n", cand.size());

    curr.clear();

    for (size_t i = cr.find_first(); i != cr.npos; i = cr.find_next(i)) {
        char c = (char)i;
        bool nocase = myisupper(c) && cr.test(mytolower(c));
        if (myislower(c) && cr.test(mytoupper(c))) {
            continue;
        }

        for (const auto &lit : cand) {
            if (curr.size() >= MAX_MASK_LITS) {
                DEBUG_PRINTF("hit lit limit of %u\n", MAX_MASK_LITS);
                return false;
            }

            curr.push_back(lit);
            curr.back().push_back(c, nocase);
        }
    }

    if (curr.back().length() > MAX_MASK2_WIDTH &&
        any_of(begin(curr), end(curr), mixed_sensitivity)) {
        DEBUG_PRINTF("mixed-sensitivity lit is too long, stopping\n");
        return false;
    }

    assert(curr.size() <= MAX_MASK_LITS);
    cand.swap(curr);
    return true;
}

static
u32 scoreFmlCandidates(const vector<ue2_literal> &cand) {
    if (cand.empty()) {
        DEBUG_PRINTF("no candidates\n");
        return 0;
    }

    const u32 len = cand.back().length();

    DEBUG_PRINTF("length = %u count %zu\n", len, cand.size());
    u32 min_period = len;

    for (const auto &lit : cand) {
        DEBUG_PRINTF("candidate: %s\n", dumpString(lit).c_str());
        u32 period = lit.length() - maxStringSelfOverlap(lit);
        min_period = min(min_period, period);
    }
    DEBUG_PRINTF("min_period %u\n", min_period);
    u32 length_score =
        (5 * min_period + len) * (cand.back().any_nocase() ? 90 : 100);
    u32 count_penalty;
    if (len > 4) {
        count_penalty = 9 * len * cand.size();
    } else {
        count_penalty = 5 * cand.size();
    }
    if (length_score <= count_penalty) {
        return 1;
    }
    return length_score - count_penalty;
}

/* favours later literals */
static
bool findMaskLiterals(const vector<CharReach> &mask, vector<ue2_literal> *lit,
                      u32 *minBound, u32 *length) {
    *minBound = 0;
    *length = 0;

    vector<ue2_literal> candidates, best_candidates, curr_candidates;
    u32 best_score = 0;
    u32 best_minOffset = 0;

    for (auto it = mask.begin(); it != mask.end(); ++it) {
        candidates.clear();
        if (!initFmlCandidates(*it, candidates)) {
            DEBUG_PRINTF("failed to init\n");
            continue;
        }
        DEBUG_PRINTF("++\n");
        auto jt = it;
        while (jt != mask.begin()) {
            --jt;
            DEBUG_PRINTF("--\n");
            if (!expandFmlCandidates(*jt, curr_candidates, candidates)) {
                DEBUG_PRINTF("expansion stopped\n");
                break;
            }
        }

        // Candidates have been expanded in reverse order.
        for (auto &cand : candidates) {
            cand = reverse_literal(cand);
        }

        u32 score = scoreFmlCandidates(candidates);
        DEBUG_PRINTF("scored %u for literal set of size %zu\n", score,
                     candidates.size());
        if (!candidates.empty() && score >= best_score) {
            best_minOffset = it - mask.begin() - candidates.back().length() + 1;
            best_candidates.swap(candidates);
            best_score = score;
       }
    }

    if (!best_score) {
        DEBUG_PRINTF("no lits\n");
        return false;
    }

    *minBound = best_minOffset;
    *length = best_candidates.back().length();

    DEBUG_PRINTF("best minbound %u length %u\n", *minBound, *length);

    assert(all_of_in(best_candidates, [&](const ue2_literal &s) {
        return s.length() == *length;
    }));

    *lit = std::move(best_candidates);
    return true;
}

static
unique_ptr<NGHolder> buildMaskLhs(bool anchored, u32 prefix_len,
                                  const vector<CharReach> &mask) {
    DEBUG_PRINTF("build %slhs len %u/%zu\n", anchored ? "anc " : "", prefix_len,
                 mask.size());

    unique_ptr<NGHolder> lhs = ue2::make_unique<NGHolder>(NFA_PREFIX);

    assert(prefix_len);
    assert(mask.size() >= prefix_len);
    NFAVertex pred = anchored ? lhs->start : lhs->startDs;

    u32 m_idx = 0;
    while (prefix_len--) {
        NFAVertex v = add_vertex(*lhs);
        (*lhs)[v].char_reach = mask[m_idx++];
        add_edge(pred, v, *lhs);
        pred = v;
    }
    add_edge(pred, lhs->accept, *lhs);
    (*lhs)[pred].reports.insert(0);

    return lhs;
}

static
void buildLiteralMask(const vector<CharReach> &mask, vector<u8> &msk,
                      vector<u8> &cmp, u32 delay) {
    msk.clear();
    cmp.clear();
    if (mask.size() <= delay) {
        return;
    }

    // Construct an and/cmp mask from our mask ending at delay positions before
    // the end of the literal, with max length HWLM_MASKLEN.

    auto ite = mask.end() - delay;
    auto it = ite - min(size_t{HWLM_MASKLEN}, mask.size() - delay);

    for (; it != ite; ++it) {
        msk.push_back(0);
        cmp.push_back(0);
        make_and_cmp_mask(*it, &msk.back(), &cmp.back());
    }

    assert(msk.size() == cmp.size());
    assert(msk.size() <= HWLM_MASKLEN);
}

static
bool validateTransientMask(const vector<CharReach> &mask, bool anchored,
                           bool eod, const Grey &grey) {
    assert(!mask.empty());

    // An EOD anchored mask requires that everything fit into history, while an
    // ordinary floating case can handle one byte more (i.e., max history size
    // and one byte in the buffer).
    const size_t max_width = grey.maxHistoryAvailable + (eod ? 0 : 1);
    if (mask.size() > max_width) {
        DEBUG_PRINTF("mask too long for max available history\n");
        return false;
    }

    /* although anchored masks cannot be transient, short masks may be placed
     * into the atable. */
    if (anchored && mask.size() > grey.maxAnchoredRegion) {
        return false;
    }

    vector<ue2_literal> lits;
    u32 lit_minBound; /* minBound of each literal in lit */
    u32 lit_length;   /* length of each literal in lit */
    if (!findMaskLiterals(mask, &lits, &lit_minBound, &lit_length)) {
        DEBUG_PRINTF("failed to find any lits\n");
        return false;
    }

    if (lits.empty()) {
        return false;
    }

    const u32 delay = mask.size() - lit_length - lit_minBound;
    if (delay > MAX_DELAY) {
        DEBUG_PRINTF("delay %u is too much\n", delay);
        return false;
    }

    if (lit_length == 1 && lits.size() > 3) {
        DEBUG_PRINTF("no decent trigger\n");
        return false;
    }

    // Mixed-sensitivity literals require benefits masks to implement, and thus
    // have a maximum length. This has been taken into account in
    // findMaskLiterals.
    assert(lit_length <= MAX_MASK2_WIDTH ||
           none_of(begin(lits), end(lits), mixed_sensitivity));

    // Build the HWLM literal mask.
    vector<u8> msk, cmp;
    if (grey.roseHamsterMasks) {
        buildLiteralMask(mask, msk, cmp, delay);
    }

    // We consider the HWLM mask length to run from the first non-zero byte to
    // the end, and let max(mask length, literal length) be the effective
    // literal length.
    //
    // A one-byte literal with no mask is too short, but a one-byte literal
    // with a few bytes of mask information is OK.

    u32 msk_length = distance(find_if(begin(msk), end(msk),
                              [](u8 v) { return v != 0; }), end(msk));
    u32 eff_lit_length = max(lit_length, msk_length);
    DEBUG_PRINTF("msk_length=%u, eff_lit_length = %u\n", msk_length,
                 eff_lit_length);

    if (eff_lit_length < MIN_MASK_LIT_LEN) {
        DEBUG_PRINTF("literals too short\n");
        return false;
    }

    DEBUG_PRINTF("mask is ok\n");
    return true;
}

static
bool maskIsNeeded(const ue2_literal &lit, const NGHolder &g) {
    flat_set<NFAVertex> curr = {g.accept};
    flat_set<NFAVertex> next;

    for (auto it = lit.rbegin(), ite = lit.rend(); it != ite; ++it) {
        const CharReach &cr = *it;
        DEBUG_PRINTF("check %s\n", describeClass(*it).c_str());
        next.clear();
        for (auto v : curr) {
            for (auto u : inv_adjacent_vertices_range(v, g)) {
                if (isSubsetOf(cr, g[u].char_reach)) {
                    next.insert(u);
                }
            }
        }
        if (next.empty()) {
            DEBUG_PRINTF("no path to start\n");
            return true;
        }
        curr.swap(next);
    }

    for (auto v : curr) {
        for (auto u : inv_adjacent_vertices_range(v, g)) {
            if (u == g.start || u == g.startDs) {
                DEBUG_PRINTF("literal spans graph from start to accept\n");
                return false;

            }
        }
    }

    DEBUG_PRINTF("literal doesn't reach start\n");
    return true;
}

static
void addTransientMask(RoseBuildImpl &build, const vector<CharReach> &mask,
                      const flat_set<ReportID> &reports, bool anchored,
                      bool eod) {
    vector<ue2_literal> lits;
    u32 lit_minBound; /* minBound of each literal in lit */
    u32 lit_length;   /* length of each literal in lit */
    if (!findMaskLiterals(mask, &lits, &lit_minBound, &lit_length)) {
        DEBUG_PRINTF("failed to find any lits\n");
        assert(0);
        return;
    }

    DEBUG_PRINTF("%zu literals, minBound=%u, length=%u\n", lits.size(),
                 lit_minBound, lit_length);

    if (lits.empty()) {
        assert(0);
        return;
    }

    u32 delay = mask.size() - lit_length - lit_minBound;
    assert(delay <= MAX_DELAY);
    DEBUG_PRINTF("delay=%u\n", delay);

    shared_ptr<NGHolder> mask_graph = buildMaskLhs(anchored, mask.size(), mask);

    u32 mask_lag = 0; /* TODO */

    // Everyone gets the same report ID.
    ReportID mask_report = build.getNewNfaReport();
    set_report(*mask_graph, mask_report);

    // Build the HWLM literal mask.
    vector<u8> msk, cmp;
    if (build.cc.grey.roseHamsterMasks) {
        buildLiteralMask(mask, msk, cmp, delay);
    }

    /* adjust bounds to be relative to trigger rather than mask */
    const u32 v_min_offset = add_rose_depth(0, mask.size());
    const u32 v_max_offset =
        add_rose_depth(anchored ? 0 : ROSE_BOUND_INF, mask.size());

    RoseGraph &g = build.g;

    // By default, masked literals go into the floating table (except for eod
    // cases).
    enum rose_literal_table table = ROSE_FLOATING;

    RoseVertex eod_v = RoseGraph::null_vertex();
    if (eod) {
        eod_v = add_vertex(g);
        g[eod_v].eod_accept = true;
        insert(&g[eod_v].reports, reports);
        g[eod_v].min_offset = v_min_offset;
        g[eod_v].max_offset = v_max_offset;

        // Note: because this is a transient mask, we know that we can match it
        // completely inside the history buffer. So, using the EOD literal
        // table is always safe.
        table = ROSE_EOD_ANCHORED;

        // Widen the EOD table window to cover the mask.
        ENSURE_AT_LEAST(&build.ematcher_region_size, mask.size());
    }

    const flat_set<ReportID> no_reports;

    for (const auto &lit : lits) {
        u32 lit_id = build.getLiteralId(lit, msk, cmp, delay, table);
        const RoseVertex parent = anchored ? build.anchored_root : build.root;
        bool use_mask = delay || maskIsNeeded(lit, *mask_graph);

        auto v = createVertex(&build, parent, 0, ROSE_BOUND_INF, lit_id,
                              lit.length(), eod ? no_reports : reports);

        if (use_mask) {
            g[v].left.graph = mask_graph;
            g[v].left.lag = mask_lag;
            g[v].left.leftfix_report = mask_report;
        } else {
            // Make sure our edge bounds are correct.
            RoseEdge e = edge(parent, v, g);
            g[e].minBound = 0;
            g[e].maxBound = anchored ? 0 : ROSE_BOUND_INF;
            g[e].history = anchored ? ROSE_ROLE_HISTORY_ANCH
                                    : ROSE_ROLE_HISTORY_NONE;
        }

        // Set offsets correctly.
        g[v].min_offset = v_min_offset;
        g[v].max_offset = v_max_offset;

        if (eod) {
            RoseEdge e = add_edge(v, eod_v, g);
            g[e].minBound = 0;
            g[e].maxBound = 0;
            g[e].history = ROSE_ROLE_HISTORY_LAST_BYTE;
        }
    }
}

static
unique_ptr<NGHolder> buildMaskRhs(const flat_set<ReportID> &reports,
                                  const vector<CharReach> &mask,
                                  u32 suffix_len) {
    assert(suffix_len);
    assert(mask.size() > suffix_len);

    unique_ptr<NGHolder> rhs = ue2::make_unique<NGHolder>(NFA_SUFFIX);
    NGHolder &h = *rhs;

    NFAVertex succ = h.accept;
    u32 m_idx = mask.size() - 1;
    while (suffix_len--) {
        NFAVertex u = add_vertex(h);
        if (succ == h.accept) {
            h[u].reports.insert(reports.begin(), reports.end());
        }
        h[u].char_reach = mask[m_idx--];
        add_edge(u, succ, h);
        succ = u;
    }

    NFAEdge e = add_edge(h.start, succ, h);
    h[e].tops.insert(DEFAULT_TOP);

    return rhs;
}

static
void doAddMask(RoseBuildImpl &tbi, bool anchored, const vector<CharReach> &mask,
               const ue2_literal &lit, u32 prefix_len, u32 suffix_len,
               const flat_set<ReportID> &reports) {
    /* Note: bounds are relative to literal start */
    RoseInGraph ig;
    RoseInVertex s = add_vertex(RoseInVertexProps::makeStart(anchored), ig);
    RoseInVertex v = add_vertex(RoseInVertexProps::makeLiteral(lit), ig);

    DEBUG_PRINTF("pref + lit = %u\n", prefix_len);
    assert(prefix_len >= lit.length());

    // prefix len is relative to end of literal.
    u32 minBound = prefix_len - lit.length();

    if (minBound) {
        if (anchored && prefix_len > tbi.cc.grey.maxAnchoredRegion) {
            DEBUG_PRINTF("too deep\n");
            /* see if there is an anchored literal we can also hang off */

            ue2_literal lit2;
            u32 lit2_offset;
            vector<CharReach> mask2 = mask;
            assert(mask2.size() > tbi.cc.grey.maxAnchoredRegion);
            mask2.resize(MIN(tbi.cc.grey.maxAnchoredRegion, minBound));

            findMaskLiteral(mask2, tbi.cc.streaming, &lit2, &lit2_offset,
                            tbi.cc.grey);

            if (lit2.length() >= MIN_MASK_LIT_LEN) {
                u32 prefix2_len = lit2_offset + lit2.length();
                assert(prefix2_len < minBound);
                RoseInVertex u
                   = add_vertex(RoseInVertexProps::makeLiteral(lit2), ig);
                if (lit2_offset){
                    DEBUG_PRINTF("building lhs (off %u)\n", lit2_offset);
                    shared_ptr<NGHolder> lhs2
                        = buildMaskLhs(true, lit2_offset, mask);
                    add_edge(s, u, RoseInEdgeProps(lhs2, lit2.length()), ig);
                } else {
                    add_edge(s, u, RoseInEdgeProps(0, 0), ig);
                }

                /* midfix */
                DEBUG_PRINTF("building mhs\n");
                vector<CharReach> mask3(mask.begin() + prefix2_len, mask.end());
                u32 overlap = maxOverlap(lit2, lit, 0);
                u32 delay = lit.length() - overlap;
                shared_ptr<NGHolder> mhs
                    = buildMaskLhs(true, minBound - prefix2_len + overlap,
                                   mask3);
                mhs->kind = NFA_INFIX;
                setTops(*mhs);
                add_edge(u, v, RoseInEdgeProps(mhs, delay), ig);

                DEBUG_PRINTF("add anch literal too!\n");
                goto do_rhs;
            }
        }

        shared_ptr<NGHolder> lhs = buildMaskLhs(anchored, minBound, mask);
        add_edge(s, v, RoseInEdgeProps(lhs, lit.length()), ig);
    } else {
        u32 maxBound = anchored ? minBound : ROSE_BOUND_INF;
        add_edge(s, v, RoseInEdgeProps(minBound, maxBound), ig);
    }

 do_rhs:
    if (suffix_len) {
        shared_ptr<NGHolder> rhs = buildMaskRhs(reports, mask, suffix_len);
        RoseInVertex a =
            add_vertex(RoseInVertexProps::makeAccept(set<ReportID>()), ig);
        add_edge(v, a, RoseInEdgeProps(rhs, 0), ig);
    } else {
        /* Note: masks have no eod connections */
        RoseInVertex a
            = add_vertex(RoseInVertexProps::makeAccept(reports), ig);
        add_edge(v, a, RoseInEdgeProps(0U, 0U), ig);
    }

    calcVertexOffsets(ig);

    bool rv = tbi.addRose(ig, false);

    assert(rv); /* checkAllowMask should have prevented this */
    if (!rv) {
        throw std::exception();
    }
}

static
bool checkAllowMask(const vector<CharReach> &mask, ue2_literal *lit,
                    u32 *prefix_len, u32 *suffix_len,
                    const CompileContext &cc) {
    assert(!mask.empty());
    u32 lit_offset;
    findMaskLiteral(mask, cc.streaming, lit, &lit_offset, cc.grey);

    if (lit->length() < MIN_MASK_LIT_LEN && lit->length() != mask.size()) {
        DEBUG_PRINTF("need more literal - bad mask\n");
        return false;
    }

    DEBUG_PRINTF("mask lit '%s', len=%zu at offset=%u\n",
                 dumpString(*lit).c_str(), lit->length(), lit_offset);

    assert(!cc.streaming || lit->length() <= cc.grey.maxHistoryAvailable + 1);

    /* literal is included in the prefix nfa so that matches from the prefix
     * can't occur in the history buffer - probably should tweak the NFA API
     * to allow such matches not to be suppressed */
    *prefix_len = lit_offset + lit->length();
    *suffix_len = mask.size() - *prefix_len;
    DEBUG_PRINTF("prefix_len=%u, suffix_len=%u\n", *prefix_len, *suffix_len);

    /* check if we can backtrack sufficiently */
    if (cc.streaming && *prefix_len > cc.grey.maxHistoryAvailable + 1) {
        DEBUG_PRINTF("too much lag\n");
        return false;
    }

    if (*suffix_len > MAX_MASK_SIZE || *prefix_len > MAX_MASK_SIZE) {
        DEBUG_PRINTF("too big\n");
        return false;
    }

    return true;
}

bool RoseBuildImpl::add(bool anchored, const vector<CharReach> &mask,
                        const flat_set<ReportID> &reports) {
    if (validateTransientMask(mask, anchored, false, cc.grey)) {
        bool eod = false;
        addTransientMask(*this, mask, reports, anchored, eod);
        return true;
    }

    ue2_literal lit;
    u32 prefix_len = 0;
    u32 suffix_len = 0;

    if (!checkAllowMask(mask, &lit, &prefix_len, &suffix_len, cc)) {
        return false;
    }

    /* we know that the mask can be handled now, start playing with the rose
     * graph */
    doAddMask(*this, anchored, mask, lit, prefix_len,  suffix_len, reports);

    return true;
}

bool RoseBuildImpl::validateMask(const vector<CharReach> &mask,
                                 UNUSED const flat_set<ReportID> &reports,
                                 bool anchored, bool eod) const {
    return validateTransientMask(mask, anchored, eod, cc.grey);
}

static
unique_ptr<NGHolder> makeAnchoredGraph(const vector<CharReach> &mask,
                                       const flat_set<ReportID> &reports,
                                       bool eod) {
    auto gp = ue2::make_unique<NGHolder>();
    NGHolder &g = *gp;

    NFAVertex u = g.start;
    for (const auto &cr : mask) {
        NFAVertex v = add_vertex(g);
        g[v].char_reach = cr;
        add_edge(u, v, g);
        u = v;
    }


    g[u].reports = reports;
    add_edge(u, eod ? g.acceptEod : g.accept, g);

    return gp;
}

static
bool addAnchoredMask(RoseBuildImpl &build, const vector<CharReach> &mask,
                     const flat_set<ReportID> &reports, bool eod) {
    if (!build.cc.grey.allowAnchoredAcyclic) {
        return false;
    }

    auto g = makeAnchoredGraph(mask, reports, eod);
    assert(g);

    return build.addAnchoredAcyclic(*g);
}

void RoseBuildImpl::addMask(const vector<CharReach> &mask,
                            const flat_set<ReportID> &reports, bool anchored,
                            bool eod) {
    if (anchored && addAnchoredMask(*this, mask, reports, eod)) {
        DEBUG_PRINTF("added mask as anchored acyclic graph\n");
        return;
    }

    addTransientMask(*this, mask, reports, anchored, eod);
}

} // namespace ue2
