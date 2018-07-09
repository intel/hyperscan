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

#include "rose_build_misc.h"
#include "rose_build_impl.h"

#include "rose_build_resources.h"
#include "hwlm/hwlm_literal.h"
#include "nfa/castlecompile.h"
#include "nfa/goughcompile.h"
#include "nfa/mcclellancompile_util.h"
#include "nfa/nfa_api.h"
#include "nfa/rdfa.h"
#include "nfa/tamaramacompile.h"
#include "nfagraph/ng_holder.h"
#include "nfagraph/ng_limex.h"
#include "nfagraph/ng_reports.h"
#include "nfagraph/ng_repeat.h"
#include "nfagraph/ng_util.h"
#include "nfagraph/ng_width.h"
#include "smallwrite/smallwrite_build.h"
#include "util/alloc.h"
#include "util/boundary_reports.h"
#include "util/compile_context.h"
#include "util/container.h"
#include "util/graph.h"
#include "util/graph_range.h"
#include "util/make_unique.h"
#include "util/order_check.h"
#include "util/report_manager.h"
#include "util/ue2string.h"
#include "util/verify_types.h"
#include "ue2common.h"
#include "grey.h"

#include <boost/graph/breadth_first_search.hpp>

using namespace std;

namespace ue2 {

// just to get it out of the header
RoseBuild::~RoseBuild() { }

RoseBuildImpl::RoseBuildImpl(ReportManager &rm_in,
                             SomSlotManager &ssm_in,
                             SmallWriteBuild &smwr_in,
                             const CompileContext &cc_in,
                             const BoundaryReports &boundary_in)
    : cc(cc_in),
      root(add_vertex(g)),
      anchored_root(add_vertex(g)),
      hasSom(false),
      group_end(0),
      ematcher_region_size(0),
      eod_event_literal_id(MO_INVALID_IDX),
      max_rose_anchored_floating_overlap(0),
      rm(rm_in),
      ssm(ssm_in),
      smwr(smwr_in),
      boundary(boundary_in),
      next_nfa_report(0) {
    // add root vertices to graph
    g[root].min_offset = 0;
    g[root].max_offset = 0;

    g[anchored_root].min_offset = 0;
    g[anchored_root].max_offset = 0;
}

RoseBuildImpl::~RoseBuildImpl() {
    // empty
}

bool RoseVertexProps::isBoring(void) const {
    return !suffix && !left;
}

bool RoseVertexProps::fixedOffset(void) const {
    assert(min_offset <= max_offset); /* ensure offsets calculated */
    return max_offset == min_offset && max_offset != ROSE_BOUND_INF;
}

bool RoseBuildImpl::isRootSuccessor(const RoseVertex &v) const {
    for (auto u : inv_adjacent_vertices_range(v, g)) {
        if (isAnyStart(u)) {
            return true;
        }
    }
    return false;
}

bool RoseBuildImpl::isNonRootSuccessor(const RoseVertex &v) const {
    for (auto u : inv_adjacent_vertices_range(v, g)) {
        if (!isAnyStart(u)) {
            return true;
        }
    }
    return false;
}

bool hasAnchHistorySucc(const RoseGraph &g, RoseVertex v) {
    for (const auto &e : out_edges_range(v, g)) {
        if (g[e].history == ROSE_ROLE_HISTORY_ANCH) {
            return true;
        }
    }

    return false;
}

bool hasLastByteHistorySucc(const RoseGraph &g, RoseVertex v) {
    for (const auto &e : out_edges_range(v, g)) {
        if (g[e].history == ROSE_ROLE_HISTORY_LAST_BYTE) {
            return true;
        }
    }

    return false;
}

static
bool isInTable(const RoseBuildImpl &tbi, RoseVertex v,
               rose_literal_table table) {
    const auto &lit_ids = tbi.g[v].literals;
    if (lit_ids.empty()) {
        return false; // special role with no literals
    }

    // All literals for a given vertex will be in the same table, so we need
    // only inspect the first one.
    const auto lit_table = tbi.literals.at(*lit_ids.begin()).table;

    // Verify that all literals for this vertex are in the same table.
    assert(all_of_in(lit_ids, [&](u32 lit_id) {
        return tbi.literals.at(lit_id).table == lit_table;
    }));

    return lit_table == table;
}

bool RoseBuildImpl::isAnchored(RoseVertex v) const {
    return isInTable(*this, v, ROSE_ANCHORED);
}

bool RoseBuildImpl::isFloating(RoseVertex v) const {
    return isInTable(*this, v, ROSE_FLOATING);
}

bool RoseBuildImpl::isInETable(RoseVertex v) const {
    return isInTable(*this, v, ROSE_EOD_ANCHORED);
}

bool RoseBuildImpl::hasLiteralInTable(RoseVertex v,
                                      enum rose_literal_table t) const {
    return isInTable(*this, v, t);
}

/* Indicates that the floating table (if it exists) will be only run
   conditionally based on matches from the anchored table. */
bool RoseBuildImpl::hasNoFloatingRoots() const {
    for (auto v : adjacent_vertices_range(root, g)) {
        if (isFloating(v)) {
            DEBUG_PRINTF("direct floating root %zu\n", g[v].index);
            return false;
        }
    }

    /* need to check if the anchored_root has any literals which are too deep */
    for (auto v : adjacent_vertices_range(anchored_root, g)) {
        if (isFloating(v)) {
            DEBUG_PRINTF("indirect floating root %zu\n", g[v].index);
            return false;
        }
    }

    return true;
}

size_t RoseBuildImpl::maxLiteralLen(RoseVertex v) const {
    const auto &lit_ids = g[v].literals;
    assert(!lit_ids.empty());

    size_t maxlen = 0;

    for (const auto &lit_id : lit_ids) {
        maxlen = max(maxlen, literals.at(lit_id).elength());
    }

    return maxlen;
}

size_t RoseBuildImpl::minLiteralLen(RoseVertex v) const {
    const auto &lit_ids = g[v].literals;
    assert(!lit_ids.empty());

    size_t minlen = ROSE_BOUND_INF;

    for (const auto &lit_id : lit_ids) {
        minlen = min(minlen, literals.at(lit_id).elength());
    }

    return minlen;
}

// RoseBuild factory
unique_ptr<RoseBuild> makeRoseBuilder(ReportManager &rm,
                                      SomSlotManager &ssm,
                                      SmallWriteBuild &smwr,
                                      const CompileContext &cc,
                                      const BoundaryReports &boundary) {
    return ue2::make_unique<RoseBuildImpl>(rm, ssm, smwr, cc, boundary);
}

bool roseIsPureLiteral(const RoseEngine *t) {
    return t->runtimeImpl == ROSE_RUNTIME_PURE_LITERAL;
}

// Returns non-zero max overlap len if a suffix of the literal 'a' overlaps
// with a prefix of the literal 'b' or 'a' can be contained in 'b'.
size_t maxOverlap(const ue2_literal &a, const ue2_literal &b, u32 b_delay) {
    /* overly conservative if only part of the string is nocase */
    bool nocase = a.any_nocase() || b.any_nocase();
    DEBUG_PRINTF("max overlap %s %s+%u %d\n", dumpString(a).c_str(),
                 dumpString(b).c_str(), b_delay, (int)nocase);
    size_t a_len = a.length();
    size_t b_len = b.length();
    const char *a_end = a.c_str() + a_len;
    const char *b_end = b.c_str() + b_len;
    if (b_delay >= a_len) {
        return b_len + b_delay;
    } else if (b_delay) {
        /* a can be a substring of b which overlaps some of the end dots
         * OR b can be a substring near the end of a */
        /* ignore overlap due to the final trailing dot as delayed literals
         * are delivered before undelayed */
        for (u32 j = b_delay - 1; j > 0; j--) {
            if (b_len + j >= a_len) {
                if (!cmp(a.c_str(), b_end + j - a_len, a_len - j, nocase)) {
                    return b_len + j;
                }
            } else {
                if (!cmp(a_end - j - b_len, b.c_str(), b_len, nocase)) {
                    return b_len + j;
                }
            }
        }
    }

    return maxStringOverlap(a.get_string(), b.get_string(), nocase);
}

// Returns non-zero max overlap len if a suffix of the literal ID 'a' overlaps
// with a prefix of the literal ID 'b' or 'a' can be contained in 'b'.
size_t maxOverlap(const rose_literal_id &a, const rose_literal_id &b) {
    assert(!a.delay);
    return maxOverlap(a.s, b.s, b.delay);
}

static
const rose_literal_id &getOverlapLiteral(const RoseBuildImpl &tbi,
                                         u32 literal_id) {
    auto it = tbi.anchoredLitSuffix.find(literal_id);
    if (it != tbi.anchoredLitSuffix.end()) {
        return it->second;
    }
    return tbi.literals.at(literal_id);
}

ue2_literal findNonOverlappingTail(const set<ue2_literal> &lits,
                                   const ue2_literal &s) {
    size_t max_overlap = 0;

    for (const auto &lit : lits) {
        size_t overlap = lit != s ? maxStringOverlap(lit, s)
                                  : maxStringSelfOverlap(s);
        max_overlap = max(max_overlap, overlap);
    }

    /* find the tail that doesn't overlap */
    ue2_literal tail = s.substr(max_overlap);
    DEBUG_PRINTF("%zu overlap, tail: '%s'\n", max_overlap,
                 dumpString(tail).c_str());
    return tail;
}

size_t RoseBuildImpl::maxLiteralOverlap(RoseVertex u, RoseVertex v) const {
    size_t overlap = 0;
    for (auto u_lit_id : g[u].literals) {
        const rose_literal_id &ul = getOverlapLiteral(*this, u_lit_id);
        for (auto v_lit_id : g[v].literals) {
            const rose_literal_id &vl = getOverlapLiteral(*this, v_lit_id);
            overlap = max(overlap, maxOverlap(ul, vl));
        }
    }
    return overlap;
}

void RoseBuildImpl::removeVertices(const vector<RoseVertex> &dead) {
    for (auto v : dead) {
        assert(!isAnyStart(v));
        DEBUG_PRINTF("removing vertex %zu\n", g[v].index);
        for (auto lit_id : g[v].literals) {
            literal_info[lit_id].vertices.erase(v);
        }
        clear_vertex(v, g);
        remove_vertex(v, g);
    }
    renumber_vertices(g);
}

// Find the maximum bound on the edges to this vertex's successors ignoring
// those via infixes.
u32 RoseBuildImpl::calcSuccMaxBound(RoseVertex u) const {
    u32 maxBound = 0;
    for (const auto &e : out_edges_range(u, g)) {
        RoseVertex v = target(e, g);

        if (g[v].left) {
            continue;
        }

        u32 thisBound = g[e].maxBound;

        if (thisBound == ROSE_BOUND_INF) {
            return ROSE_BOUND_INF;
        }

        if (!g[v].eod_accept) {
            // Add the length of the longest of our literals.
            thisBound += maxLiteralLen(v);
        }

        maxBound = max(maxBound, thisBound);
    }

    assert(maxBound <= ROSE_BOUND_INF);
    return maxBound;
}

u32 RoseBuildImpl::getLiteralId(const ue2_literal &s, u32 delay,
                                rose_literal_table table) {
    DEBUG_PRINTF("getting id for %s in table %d\n", dumpString(s).c_str(),
                 table);
    assert(table != ROSE_ANCHORED);
    rose_literal_id key(s, table, delay);

    auto m = literals.insert(key);
    u32 id = m.first;
    bool inserted = m.second;

    if (inserted) {
        literal_info.push_back(rose_literal_info());
        assert(literal_info.size() == id + 1);

        if (delay) {
            u32 undelayed_id = getLiteralId(s, 0, table);
            literal_info[id].undelayed_id = undelayed_id;
            literal_info[undelayed_id].delayed_ids.insert(id);
        } else {
            literal_info[id].undelayed_id = id;
        }
    }
    return id;
}

// Function that operates on a msk/cmp pair and a literal, as used in
// hwlmLiteral, and zeroes msk elements that don't add any power to the
// literal.
void normaliseLiteralMask(const ue2_literal &s_in, vector<u8> &msk,
                          vector<u8> &cmp) {
    assert(msk.size() == cmp.size());
    assert(msk.size() <= HWLM_MASKLEN);

    if (msk.empty()) {
        return;
    }

    // Work over a caseless copy if the string contains nocase chars. This will
    // ensure that we treat masks designed to handle mixed-sensitivity literals
    // correctly: these will be matched by the literal matcher in caseless
    // mode, with the mask used to narrow the matches.
    ue2_literal s(s_in);
    if (s.any_nocase()) {
        make_nocase(&s);
    }

    ue2_literal::const_reverse_iterator it = s.rbegin(), ite = s.rend();
    size_t i = msk.size();
    while (i-- != 0 && it != ite) {
        const CharReach &cr = *it;
        for (size_t c = cr.find_first(); c != CharReach::npos;
             c = cr.find_next(c)) {
            if (((u8)c & msk[i]) != cmp[i]) {
                goto skip;
            }
        }

        // If we didn't jump out of the loop to skip, then this mask position
        // doesn't further narrow the set of acceptable literals from those
        // accepted by s. So we can zero this element.
        msk[i] = 0;
        cmp[i] = 0;
    skip:
        ++it;
    }

    // Wipe out prefix zeroes.
    while (!msk.empty() && msk[0] == 0) {
        msk.erase(msk.begin());
        cmp.erase(cmp.begin());
    }
}

rose_literal_id::rose_literal_id(const ue2_literal &s_in,
        const vector<u8> &msk_in, const vector<u8> &cmp_in,
        rose_literal_table table_in, u32 delay_in)
            : s(s_in), msk(msk_in), cmp(cmp_in), table(table_in),
              delay(delay_in), distinctiveness(0) {
    assert(msk.size() == cmp.size());
    assert(msk.size() <= HWLM_MASKLEN);
    assert(delay <= MAX_DELAY);

    normaliseLiteralMask(s, msk, cmp);
}

u32 RoseBuildImpl::getLiteralId(const ue2_literal &s, const vector<u8> &msk,
                                const vector<u8> &cmp, u32 delay,
                                rose_literal_table table) {
    DEBUG_PRINTF("getting id for %s in table %d\n", dumpString(s).c_str(),
                 table);
    assert(table != ROSE_ANCHORED);
    rose_literal_id key(s, msk, cmp, table, delay);

    /* ue2_literals are always uppercased if nocase and must have an
     * alpha char */

    auto m = literals.insert(key);
    u32 id = m.first;
    bool inserted = m.second;

    if (inserted) {
        literal_info.push_back(rose_literal_info());
        assert(literal_info.size() == id + 1);

        if (delay) {
            u32 undelayed_id = getLiteralId(s, msk, cmp, 0, table);
            literal_info[id].undelayed_id = undelayed_id;
            literal_info[undelayed_id].delayed_ids.insert(id);
        } else {
            literal_info[id].undelayed_id = id;
        }
    }
    return id;
}

u32 RoseBuildImpl::getNewLiteralId() {
    rose_literal_id key(ue2_literal(), ROSE_ANCHORED, 0);
    u32 numLiterals = verify_u32(literals.size());
    key.distinctiveness = numLiterals;

    auto m = literals.insert(key);
    assert(m.second);
    u32 id = m.first;

    literal_info.push_back(rose_literal_info());
    assert(literal_info.size() == id + 1);

    literal_info[id].undelayed_id = id;

    return id;
}

bool operator<(const RoseEdgeProps &a, const RoseEdgeProps &b) {
    ORDER_CHECK(minBound);
    ORDER_CHECK(maxBound);
    ORDER_CHECK(history);
    return false;
}

#ifndef NDEBUG
bool roseHasTops(const RoseBuildImpl &build, RoseVertex v) {
    const RoseGraph &g = build.g;
    assert(g[v].left);

    set<u32> graph_tops;
    if (!build.isRootSuccessor(v)) {
        for (const auto &e : in_edges_range(v, g)) {
            graph_tops.insert(g[e].rose_top);
        }
    }

    return is_subset_of(graph_tops, all_tops(g[v].left));
}
#endif

u32 OutfixInfo::get_queue(QueueIndexFactory &qif) {
    if (queue == ~0U) {
        queue = qif.get_queue();
    }

    return queue;
}

namespace {
class OutfixAllReports : public boost::static_visitor<set<ReportID>> {
public:
    set<ReportID> operator()(const boost::blank &) const {
        return set<ReportID>();
    }

    template<class T>
    set<ReportID> operator()(const unique_ptr<T> &x) const {
        return all_reports(*x);
    }

    set<ReportID> operator()(const MpvProto &mpv) const {
        set<ReportID> reports;
        for (const auto &puff : mpv.puffettes) {
            reports.insert(puff.report);
        }
        for (const auto &puff : mpv.triggered_puffettes) {
            reports.insert(puff.report);
        }
        return reports;
    }
};
}

set<ReportID> all_reports(const OutfixInfo &outfix) {
    auto reports = boost::apply_visitor(OutfixAllReports(), outfix.proto);
    assert(!reports.empty());
    return reports;
}

bool RoseSuffixInfo::operator==(const RoseSuffixInfo &b) const {
    return top == b.top && graph == b.graph && castle == b.castle &&
           rdfa == b.rdfa && haig == b.haig && tamarama == b.tamarama;
}

bool RoseSuffixInfo::operator<(const RoseSuffixInfo &b) const {
    const RoseSuffixInfo &a = *this;
    ORDER_CHECK(top);
    ORDER_CHECK(graph);
    ORDER_CHECK(castle);
    ORDER_CHECK(haig);
    ORDER_CHECK(rdfa);
    ORDER_CHECK(tamarama);
    assert(a.dfa_min_width == b.dfa_min_width);
    assert(a.dfa_max_width == b.dfa_max_width);
    return false;
}

size_t RoseSuffixInfo::hash() const {
    return hash_all(top, graph, castle, rdfa, haig, tamarama);
}

void RoseSuffixInfo::reset(void) {
    top = 0;
    graph.reset();
    castle.reset();
    rdfa.reset();
    haig.reset();
    tamarama.reset();
    dfa_min_width = depth(0);
    dfa_max_width = depth::infinity();
}

std::set<ReportID> all_reports(const suffix_id &s) {
    assert(s.graph() || s.castle() || s.haig() || s.dfa());
    if (s.tamarama()) {
        return all_reports(*s.tamarama());
    } else if (s.graph()) {
        return all_reports(*s.graph());
    } else if (s.castle()) {
        return all_reports(*s.castle());
    } else if (s.dfa()) {
        return all_reports(*s.dfa());
    } else {
        return all_reports(*s.haig());
    }
}

depth findMinWidth(const suffix_id &s) {
    assert(s.graph() || s.castle() || s.haig() || s.dfa());
    if (s.graph()) {
        return findMinWidth(*s.graph());
    } else if (s.castle()) {
        return findMinWidth(*s.castle());
    } else {
        return s.dfa_min_width;
    }
}

depth findMinWidth(const suffix_id &s, u32 top) {
    assert(s.graph() || s.castle() || s.haig() || s.dfa());
    if (s.graph()) {
        return findMinWidth(*s.graph(), top);
    } else if (s.castle()) {
        return findMinWidth(*s.castle(), top);
    } else {
        return s.dfa_min_width;
    }
}

depth findMaxWidth(const suffix_id &s) {
    assert(s.graph() || s.castle() || s.haig() || s.dfa());
    if (s.graph()) {
        return findMaxWidth(*s.graph());
    } else if (s.castle()) {
        return findMaxWidth(*s.castle());
    } else {
        return s.dfa_max_width;
    }
}

depth findMaxWidth(const suffix_id &s, u32 top) {
    assert(s.graph() || s.castle() || s.haig() || s.dfa());
    if (s.graph()) {
        return findMaxWidth(*s.graph(), top);
    } else if (s.castle()) {
        return findMaxWidth(*s.castle(), top);
    } else {
        return s.dfa_max_width;
    }
}

bool has_eod_accepts(const suffix_id &s) {
    assert(s.graph() || s.castle() || s.haig() || s.dfa());
    if (s.graph()) {
        /* ignore accept -> eod edge */
        return in_degree(s.graph()->acceptEod, *s.graph()) > 1;
    } else if (s.castle()) {
        return false;
    } else if (s.dfa()) {
        return has_eod_accepts(*s.dfa());
    } else {
        return has_eod_accepts(*s.haig());
    }
}

bool has_non_eod_accepts(const suffix_id &s) {
    assert(s.graph() || s.castle() || s.haig() || s.dfa());
    if (s.graph()) {
        return in_degree(s.graph()->accept, *s.graph());
    } else if (s.castle()) {
        return true;
    } else if (s.dfa()) {
        return has_non_eod_accepts(*s.dfa());
    } else {
        return has_non_eod_accepts(*s.haig());
    }
}

set<u32> all_tops(const suffix_id &s) {
    assert(s.graph() || s.castle() || s.haig() || s.dfa());
    if (s.graph()) {
        flat_set<u32> tops = getTops(*s.graph());
        assert(!tops.empty());
        return {tops.begin(), tops.end()};
    }

    if (s.castle()) {
        return assoc_keys(s.castle()->repeats);
    }

    // Other types of suffix are not multi-top.
    return {0};
}

size_t suffix_id::hash() const {
    return hash_all(g, c, d, h, t);
}

bool isAnchored(const left_id &r) {
    assert(r.graph() || r.castle() || r.haig() || r.dfa());
    if (r.graph()) {
        return isAnchored(*r.graph());
    }
    if (r.dfa()) {
        return r.dfa()->start_anchored == DEAD_STATE;
    }
    if (r.haig()) {
        return r.haig()->start_anchored == DEAD_STATE;
    }

    // All other types are explicitly anchored.
    return true;
}

depth findMinWidth(const left_id &r) {
    assert(r.graph() || r.castle() || r.haig() || r.dfa());
    if (r.graph()) {
        return findMinWidth(*r.graph());
    } else if (r.castle()) {
        return findMinWidth(*r.castle());
    } else {
        return r.dfa_min_width;
    }
}

depth findMaxWidth(const left_id &r) {
    assert(r.graph() || r.castle() || r.haig() || r.dfa());
    if (r.graph()) {
        return findMaxWidth(*r.graph());
    } else if (r.castle()) {
        return findMaxWidth(*r.castle());
    } else {
        return r.dfa_max_width;
    }
}

set<u32> all_tops(const left_id &r) {
    assert(r.graph() || r.castle() || r.haig() || r.dfa());
    if (r.graph()) {
        flat_set<u32> tops = getTops(*r.graph());
        return {tops.begin(), tops.end()};
    }

    if (r.castle()) {
        return assoc_keys(r.castle()->repeats);
    }

    // Other types of rose are not multi-top.
    return {0};
}

set<u32> all_reports(const left_id &left) {
    assert(left.graph() || left.castle() || left.haig() || left.dfa());
    if (left.graph()) {
        return all_reports(*left.graph());
    } else if (left.castle()) {
        return all_reports(*left.castle());
    } else if (left.dfa()) {
        return all_reports(*left.dfa());
    } else {
        return all_reports(*left.haig());
    }
}

u32 num_tops(const left_id &r) {
    return all_tops(r).size();
}

size_t left_id::hash() const {
    return hash_all(g, c, d, h);
}

u64a findMaxOffset(const set<ReportID> &reports, const ReportManager &rm) {
    assert(!reports.empty());
    u64a maxOffset = 0;
    for (const auto &report_id : reports) {
        const Report &ir = rm.getReport(report_id);
        if (ir.hasBounds()) {
            maxOffset = max(maxOffset, ir.maxOffset);
        } else {
            return MAX_OFFSET;
        }
    }
    return maxOffset;
}

size_t LeftEngInfo::hash() const {
    return hash_all(graph, castle, dfa, haig, tamarama, lag, leftfix_report);
}

void LeftEngInfo::reset(void) {
    graph.reset();
    castle.reset();
    dfa.reset();
    haig.reset();
    tamarama.reset();
    lag = 0;
    leftfix_report = MO_INVALID_IDX;
    dfa_min_width = depth(0);
    dfa_max_width = depth::infinity();
}

LeftEngInfo::operator bool() const {
    assert((int)!!castle + (int)!!dfa + (int)!!haig <= 1);
    assert(!castle || !graph);
    assert(!dfa || graph); /* dfas always have the graph as well */
    assert(!haig || graph);
    return graph || castle || dfa || haig;
}

u32 roseQuality(const RoseResources &res, const RoseEngine *t) {
    /* Rose is low quality if the atable is a Mcclellan 16 or has multiple DFAs
     */
    if (res.has_anchored) {
        if (res.has_anchored_multiple) {
            DEBUG_PRINTF("multiple atable engines\n");
            return 0;
        }

        if (res.has_anchored_large) {
            DEBUG_PRINTF("m16 atable engine\n");
            return 0;
        }
    }

    /* if we always run multiple engines then we are slow */
    u32 always_run = 0;

    if (res.has_anchored) {
        always_run++;
    }

    if (t->eagerIterOffset) {
        /* eager prefixes are always run */
        always_run++;
    }

    if (res.has_floating) {
        /* TODO: ignore conditional ftables, or ftables beyond smwr region */
        always_run++;
    }

    if (t->ematcherOffset) {
        always_run++;
    }

    /* ignore mpv outfixes as they are v good, mpv outfixes are before begin */
    if (t->outfixBeginQueue != t->outfixEndQueue) {
        /* TODO: ignore outfixes > smwr region */
        always_run++;
    }

    bool eod_prefix = false;

    const LeftNfaInfo *left = getLeftTable(t);
    for (u32 i = 0; i < t->activeLeftCount; i++) {
        if (left->eod_check) {
            eod_prefix = true;
            break;
        }
    }

    if (eod_prefix) {
        always_run++;
        DEBUG_PRINTF("eod prefixes are slow");
        return 0;
    }

    if (always_run > 1) {
        DEBUG_PRINTF("we always run %u engines\n", always_run);
        return 0;
    }

    return 1;
}

u32 findMinOffset(const RoseBuildImpl &build, u32 lit_id) {
    const auto &lit_vertices = build.literal_info.at(lit_id).vertices;
    assert(!lit_vertices.empty());

    u32 min_offset = UINT32_MAX;
    for (const auto &v : lit_vertices) {
        min_offset = min(min_offset, build.g[v].min_offset);
    }

    return min_offset;
}

u32 findMaxOffset(const RoseBuildImpl &build, u32 lit_id) {
    const auto &lit_vertices = build.literal_info.at(lit_id).vertices;
    assert(!lit_vertices.empty());

    u32 max_offset = 0;
    for (const auto &v : lit_vertices) {
        max_offset = max(max_offset, build.g[v].max_offset);
    }

    return max_offset;
}

bool canEagerlyReportAtEod(const RoseBuildImpl &build, const RoseEdge &e) {
    const auto &g = build.g;
    const auto v = target(e, g);

    if (!build.g[v].eod_accept) {
        return false;
    }

    // If there's a graph between us and EOD, we shouldn't be eager.
    if (build.g[v].left) {
        return false;
    }

    // Must be exactly at EOD.
    if (g[e].minBound != 0 || g[e].maxBound != 0) {
        return false;
    }

    // In streaming mode, we can only eagerly report EOD for literals in the
    // EOD-anchored table, as that's the only time we actually know where EOD
    // is. In block mode, we always have this information.
    const auto u = source(e, g);
    if (build.cc.streaming && !build.isInETable(u)) {
        return false;
    }

    return true;
}

#ifndef NDEBUG
/** \brief Returns true if all the graphs (NFA, DFA, Haig, etc) in this Rose
 * graph are implementable. */
bool canImplementGraphs(const RoseBuildImpl &tbi) {
    const RoseGraph &g = tbi.g;

    // First, check the Rose leftfixes.

    for (auto v : vertices_range(g)) {
        DEBUG_PRINTF("leftfix: check vertex %zu\n", g[v].index);

        if (g[v].left.castle) {
            DEBUG_PRINTF("castle ok\n");
            continue;
        }
        if (g[v].left.dfa) {
            DEBUG_PRINTF("dfa ok\n");
            continue;
        }
        if (g[v].left.haig) {
            DEBUG_PRINTF("haig ok\n");
            continue;
        }
        if (g[v].left.graph) {
            assert(g[v].left.graph->kind
                   == (tbi.isRootSuccessor(v) ? NFA_PREFIX : NFA_INFIX));
            if (!isImplementableNFA(*g[v].left.graph, nullptr, tbi.cc)) {
                DEBUG_PRINTF("nfa prefix %zu failed (%zu vertices)\n",
                             g[v].index, num_vertices(*g[v].left.graph));
                return false;
            }
        }
    }

    // Suffix graphs.

    for (auto v : vertices_range(g)) {
        DEBUG_PRINTF("suffix: check vertex %zu\n", g[v].index);

        const RoseSuffixInfo &suffix = g[v].suffix;
        if (suffix.castle) {
            DEBUG_PRINTF("castle suffix ok\n");
            continue;
        }
        if (suffix.rdfa) {
            DEBUG_PRINTF("dfa suffix ok\n");
            continue;
        }
        if (suffix.haig) {
            DEBUG_PRINTF("haig suffix ok\n");
            continue;
        }
        if (suffix.graph) {
            assert(suffix.graph->kind == NFA_SUFFIX);
            if (!isImplementableNFA(*suffix.graph, &tbi.rm, tbi.cc)) {
                DEBUG_PRINTF("nfa suffix %zu failed (%zu vertices)\n",
                             g[v].index, num_vertices(*suffix.graph));
                return false;
            }
        }
    }

    return true;
}

/**
 * \brief True if there is an engine with a top that is not triggered by a
 * vertex in the Rose graph. This is a consistency check used in assertions.
 */
bool hasOrphanedTops(const RoseBuildImpl &build) {
    const RoseGraph &g = build.g;

    unordered_map<left_id, set<u32>> leftfixes;
    unordered_map<suffix_id, set<u32>> suffixes;

    for (auto v : vertices_range(g)) {
        if (g[v].left) {
            set<u32> &tops = leftfixes[g[v].left];
            if (!build.isRootSuccessor(v)) {
                // Tops for infixes come from the in-edges.
                for (const auto &e : in_edges_range(v, g)) {
                    tops.insert(g[e].rose_top);
                }
            }
        }
        if (g[v].suffix) {
            suffixes[g[v].suffix].insert(g[v].suffix.top);
        }
    }

    for (const auto &e : leftfixes) {
        if (all_tops(e.first) != e.second) {
            DEBUG_PRINTF("rose tops (%s) don't match rose graph (%s)\n",
                         as_string_list(all_tops(e.first)).c_str(),
                         as_string_list(e.second).c_str());
            return true;
        }
    }

    for (const auto &e : suffixes) {
        if (all_tops(e.first) != e.second) {
            DEBUG_PRINTF("suffix tops (%s) don't match rose graph (%s)\n",
                         as_string_list(all_tops(e.first)).c_str(),
                         as_string_list(e.second).c_str());
            return true;
        }
    }

    return false;
}

#endif // NDEBUG

} // namespace ue2
