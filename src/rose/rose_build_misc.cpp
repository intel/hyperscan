/*
 * Copyright (c) 2015-2016, Intel Corporation
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

#include "hwlm/hwlm_build.h"
#include "nfa/castlecompile.h"
#include "nfa/goughcompile.h"
#include "nfa/mcclellancompile_util.h"
#include "nfa/nfa_api.h"
#include "nfa/rdfa.h"
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

#include <boost/functional/hash/hash_fwd.hpp>
#include <boost/graph/breadth_first_search.hpp>

using namespace std;
using boost::hash_combine;

namespace ue2 {

// just to get it out of the header
RoseBuild::~RoseBuild() { }

RoseBuildImpl::RoseBuildImpl(ReportManager &rm_in, SomSlotManager &ssm_in,
                             const CompileContext &cc_in,
                             const BoundaryReports &boundary_in)
    : cc(cc_in),
      root(add_vertex(g)),
      anchored_root(add_vertex(g)),
      vertexIndex(0),
      delay_base_id(MO_INVALID_IDX),
      hasSom(false),
      group_weak_end(0),
      group_end(0),
      anchored_base_id(MO_INVALID_IDX),
      ematcher_region_size(0),
      eod_event_literal_id(MO_INVALID_IDX),
      max_rose_anchored_floating_overlap(0),
      rm(rm_in),
      ssm(ssm_in),
      boundary(boundary_in),
      next_nfa_report(0) {
    // add root vertices to graph
    g[root].idx = vertexIndex++;
    g[root].min_offset = 0;
    g[root].max_offset = 0;

    g[anchored_root].idx = vertexIndex++;
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
    const auto lit_table = tbi.literals.right.at(*lit_ids.begin()).table;

#ifndef NDEBUG
    // Verify that all literals for this vertex are in the same table.
    for (auto lit_id : lit_ids) {
        assert(tbi.literals.right.at(lit_id).table == lit_table);
    }
#endif

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
            DEBUG_PRINTF("direct floating root %zu\n", g[v].idx);
            return false;
        }
    }

    /* need to check if the anchored_root has any literals which are too deep */
    for (auto v : adjacent_vertices_range(anchored_root, g)) {
        if (isFloating(v)) {
            DEBUG_PRINTF("indirect floating root %zu\n", g[v].idx);
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
        maxlen = max(maxlen, literals.right.at(lit_id).elength());
    }

    return maxlen;
}

size_t RoseBuildImpl::minLiteralLen(RoseVertex v) const {
    const auto &lit_ids = g[v].literals;
    assert(!lit_ids.empty());

    size_t minlen = ROSE_BOUND_INF;

    for (const auto &lit_id : lit_ids) {
        minlen = min(minlen, literals.right.at(lit_id).elength());
    }

    return minlen;
}

// RoseBuild factory
unique_ptr<RoseBuild> makeRoseBuilder(ReportManager &rm, SomSlotManager &ssm,
                                      const CompileContext &cc,
                                      const BoundaryReports &boundary) {
    return ue2::make_unique<RoseBuildImpl>(rm, ssm, cc, boundary);
}

size_t roseSize(const RoseEngine *t) {
    assert(t);
    return t->size;
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
    map<u32, rose_literal_id>::const_iterator it =
        tbi.anchoredLitSuffix.find(literal_id);
    if (it != tbi.anchoredLitSuffix.end()) {
        return it->second;
    }
    return tbi.literals.right.at(literal_id);
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
        DEBUG_PRINTF("removing vertex %zu\n", g[v].idx);
        for (auto lit_id : g[v].literals) {
            literal_info[lit_id].vertices.erase(v);
        }
        clear_vertex_faster(v, g);
        remove_vertex(v, g);
    }
    renumberVertices();
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
    DEBUG_PRINTF("getting id for %s\n", dumpString(s).c_str());
    assert(table != ROSE_ANCHORED);
    rose_literal_id key(s, table, delay);
    u32 numLiterals = verify_u32(literals.left.size());

    RoseLiteralMap::iterator it;
    bool inserted;
    tie(it, inserted)
        = literals.insert(RoseLiteralMap::value_type(key, numLiterals));
    u32 id = it->right;

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
    DEBUG_PRINTF("getting id for %s\n", dumpString(s).c_str());
    assert(table != ROSE_ANCHORED);
    rose_literal_id key(s, msk, cmp, table, delay);
    u32 numLiterals = verify_u32(literals.left.size());

    /* ue2_literals are always uppercased if nocase and must have an
     * alpha char */

    RoseLiteralMap::iterator it;
    bool inserted;
    tie(it, inserted) = literals.insert(
            RoseLiteralMap::value_type(key, numLiterals));
    u32 id = it->right;

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

bool RoseBuildImpl::hasLiteral(const ue2_literal &s,
                               rose_literal_table table) const {
    DEBUG_PRINTF("looking if %s exists\n", dumpString(s).c_str());
    assert(table != ROSE_ANCHORED);

    for (RoseLiteralMap::left_map::const_iterator it
        = literals.left.lower_bound(rose_literal_id(s, table, 0));
         it != literals.left.end(); ++it) {
        if (it->first.table != table || it->first.s != s) {
            break;
        }
        const rose_literal_info &info = literal_info[it->second];
        if (!info.vertices.empty()) {
            return true;
        }
    }

    DEBUG_PRINTF("(used) literal not found\n");

    return false;
}

u32 RoseBuildImpl::getNewLiteralId() {
    rose_literal_id key(ue2_literal(), ROSE_ANCHORED, 0);
    u32 numLiterals = verify_u32(literals.left.size());
    key.distinctiveness = numLiterals;

    RoseLiteralMap::iterator it;
    bool inserted;
    tie(it, inserted)
        = literals.insert(RoseLiteralMap::value_type(key, numLiterals));
    u32 id = it->right;

    assert(inserted);

    literal_info.push_back(rose_literal_info());
    assert(literal_info.size() == id + 1);

    literal_info[id].undelayed_id = id;

    return id;
}

static
bool requiresDedupe(const NGHolder &h, const ue2::flat_set<ReportID> &reports,
                    const Grey &grey) {
    /* TODO: tighten */
    NFAVertex seen_vert = NFAGraph::null_vertex();

    for (auto v : inv_adjacent_vertices_range(h.accept, h)) {
        if (has_intersection(h[v].reports, reports)) {
            if (seen_vert != NFAGraph::null_vertex()) {
                return true;
            }
            seen_vert = v;
        }
    }

    for (auto v : inv_adjacent_vertices_range(h.acceptEod, h)) {
        if (has_intersection(h[v].reports, reports)) {
            if (seen_vert != NFAGraph::null_vertex()) {
                return true;
            }
            seen_vert = v;
        }
    }

    if (seen_vert) {
        /* if the reporting vertex is part of of a terminal repeat, the
         * construction process may reform the graph splitting it into two
         * vertices (pos, cyclic) and hence require dedupe */
        vector<GraphRepeatInfo> repeats;
        findRepeats(h, grey.minExtBoundedRepeatSize, &repeats);
        for (const auto &repeat : repeats) {
            if (find(repeat.vertices.begin(), repeat.vertices.end(),
                     seen_vert) != repeat.vertices.end()) {
                return true;
            }
        }
    }

    return false;
}

class RoseDedupeAuxImpl : public RoseDedupeAux {
public:
    explicit RoseDedupeAuxImpl(const RoseBuildImpl &tbi_in);
    bool requiresDedupeSupport(
        const ue2::flat_set<ReportID> &reports) const override;

    const RoseBuildImpl &tbi;
    map<ReportID, set<RoseVertex>> vert_map;
    map<ReportID, set<suffix_id>> suffix_map;
    map<ReportID, set<const OutfixInfo *>> outfix_map;
    map<ReportID, set<const raw_puff *>> puff_map;
};

unique_ptr<RoseDedupeAux> RoseBuildImpl::generateDedupeAux() const {
    return ue2::make_unique<RoseDedupeAuxImpl>(*this);
}

RoseDedupeAux::~RoseDedupeAux() {
}

RoseDedupeAuxImpl::RoseDedupeAuxImpl(const RoseBuildImpl &tbi_in)
    : tbi(tbi_in) {
    const RoseGraph &g = tbi.g;

    set<suffix_id> suffixes;

    for (auto v : vertices_range(g)) {
        // Literals in the small block table don't count as dupes: although
        // they have copies in the anchored table, the two are never run in the
        // same runtime invocation. All other literals count, though.
        if (!tbi.hasLiteralInTable(v, ROSE_ANCHORED_SMALL_BLOCK)) {
            for (const auto &report_id : g[v].reports) {
                vert_map[report_id].insert(v);
            }
        }

        // Several vertices may share a suffix, so we collect the set of
        // suffixes first to avoid repeating work.
        if (g[v].suffix) {
            suffixes.insert(g[v].suffix);
        }
    }

    for (const auto &suffix : suffixes) {
        for (const auto &report_id : all_reports(suffix)) {
            suffix_map[report_id].insert(suffix);
        }
    }

    for (const auto &outfix : tbi.outfixes) {
        for (const auto &report_id : all_reports(outfix)) {
            outfix_map[report_id].insert(&outfix);
        }
    }

    if (tbi.mpv_outfix) {
        auto *mpv = tbi.mpv_outfix->mpv();
        for (const auto &puff : mpv->puffettes) {
            puff_map[puff.report].insert(&puff);
        }
        for (const auto &puff : mpv->triggered_puffettes) {
            puff_map[puff.report].insert(&puff);
        }
    }
}

static
vector<CharReach> makePath(const rose_literal_id &lit) {
    vector<CharReach> path(begin(lit.s), end(lit.s));
    for (u32 i = 0; i < lit.delay; i++) {
        path.push_back(CharReach::dot());
    }
    return path;
}

/**
 * \brief True if one of the given literals overlaps with the suffix of
 * another, meaning that they could arrive at the same offset.
 */
static
bool literalsCouldRace(const rose_literal_id &lit1,
                       const rose_literal_id &lit2) {
    DEBUG_PRINTF("compare %s (delay %u) and %s (delay %u)\n",
                 dumpString(lit1.s).c_str(), lit1.delay,
                 dumpString(lit2.s).c_str(), lit2.delay);

    // Add dots on the end of each literal for delay.
    const auto v1 = makePath(lit1);
    const auto v2 = makePath(lit2);

    // See if the smaller path is a suffix of the larger path.
    const auto *smaller = v1.size() < v2.size() ? &v1 : &v2;
    const auto *bigger = v1.size() < v2.size() ? &v2 : &v1;
    auto r = mismatch(smaller->rbegin(), smaller->rend(), bigger->rbegin(),
                      overlaps);
    return r.first == smaller->rend();
}

bool RoseDedupeAuxImpl::requiresDedupeSupport(
    const ue2::flat_set<ReportID> &reports) const {
    /* TODO: this could be expanded to check for offset or character
       constraints */

    const RoseGraph &g = tbi.g;

    bool has_suffix = false;
    bool has_outfix = false;

    if (reports.size() > 1) {
        /* may have offset adjust */
        /* TODO: work out if the offset adjust will actually cause problems */
        return true;
    }

    set<RoseVertex> roles;
    set<suffix_id> suffixes;
    set<const OutfixInfo *> outfixes;
    set<const raw_puff *> puffettes;
    for (ReportID r : reports) {
        if (contains(vert_map, r)) {
            insert(&roles, vert_map.at(r));
        }

        if (contains(suffix_map, r)) {
            insert(&suffixes, suffix_map.at(r));
        }

        if (contains(outfix_map, r)) {
            insert(&outfixes, outfix_map.at(r));
        }

        if (contains(puff_map, r)) {
            insert(&puffettes, puff_map.at(r));
        }
    }

    /* roles */

    map<u32, u32> lits; // Literal ID -> count of occurrences.

    const bool has_role = !roles.empty();
    for (auto v : roles) {
        for (const auto &lit : g[v].literals) {
            lits[lit]++;
        }
        if (g[v].eod_accept) {
            // Literals plugged into this EOD accept must be taken into account
            // as well.
            for (auto u : inv_adjacent_vertices_range(v, g)) {
                for (const auto &lit : g[u].literals) {
                    lits[lit]++;
                }
            }
        }
    }

    /* literals */

    for (const auto &m : lits) {
        if (m.second > 1) {
            DEBUG_PRINTF("lit %u used by >1 reporting roles\n", m.first);
            return true;
        }
    }

    for (auto it = begin(lits); it != end(lits); ++it) {
        const auto &lit1 = tbi.literals.right.at(it->first);
        for (auto jt = next(it); jt != end(lits); ++jt) {
            const auto &lit2 = tbi.literals.right.at(jt->first);
            if (literalsCouldRace(lit1, lit2)) {
                DEBUG_PRINTF("literals could race\n");
                return true;
            }
        }
    }

    /* suffixes */

    for (const auto &suffix : suffixes) {
        if (has_suffix || has_role) {
            return true; /* scope for badness */
        }

        has_suffix = true;

        /* some lesser suffix engines (nfas, haig, castle) can raise multiple
         * matches for a report id at the same offset if there are multiple
         * report states live. */
        if (suffix.haig()) {
            return true;
        }
        if (suffix.graph() &&
            requiresDedupe(*suffix.graph(), reports, tbi.cc.grey)) {
            return true;
        }
        if (suffix.castle() && requiresDedupe(*suffix.castle(), reports)) {
            return true;
        }
    }

    /* outfixes */

    for (const auto &outfix_ptr : outfixes) {
        assert(outfix_ptr);
        const OutfixInfo &out = *outfix_ptr;

        if (has_outfix || has_role || has_suffix) {
            return true;
        }
        has_outfix = true;

        if (out.haig()) {
            return true; /* haig may report matches with different SOM at the
                            same offset */
        }

        if (out.holder() &&
            requiresDedupe(*out.holder(), reports, tbi.cc.grey)) {
            return true;
        }
    }

    /* mpv */
    for (UNUSED const auto &puff : puffettes) {
        if (has_outfix || has_role || has_suffix) {
            return true;
        }
        has_outfix = true;
    }

    /* boundary */
    if (has_intersection(tbi.boundary.report_at_eod, reports)) {
        if (has_outfix || has_role || has_suffix) {
            return true;
        }
    }

    return false;
}

// Sets the report ID for all vertices connected to an accept to `id`.
void setReportId(NGHolder &g, ReportID id) {
    // First, wipe the report IDs on all vertices.
    for (auto v : vertices_range(g)) {
        g[v].reports.clear();
    }

    // Any predecessors of accept get our id.
    for (auto v : inv_adjacent_vertices_range(g.accept, g)) {
        g[v].reports.insert(id);
    }

    // Same for preds of acceptEod, except accept itself.
    for (auto v : inv_adjacent_vertices_range(g.acceptEod, g)) {
        if (v == g.accept) {
            continue;
        }
        g[v].reports.insert(id);
    }
}

bool operator<(const RoseEdgeProps &a, const RoseEdgeProps &b) {
    ORDER_CHECK(minBound);
    ORDER_CHECK(maxBound);
    ORDER_CHECK(history);
    return false;
}

// Note: only clones the vertex, you'll have to wire up your own edges.
RoseVertex RoseBuildImpl::cloneVertex(RoseVertex v) {
    RoseVertex v2 = add_vertex(g[v], g);
    g[v2].idx = vertexIndex++;

    for (const auto &lit_id : g[v2].literals) {
        literal_info[lit_id].vertices.insert(v2);
    }

    return v2;
}

#ifndef NDEBUG
bool roseHasTops(const RoseGraph &g, RoseVertex v) {
    assert(g[v].left);

    set<u32> graph_tops;
    for (const auto &e : in_edges_range(v, g)) {
        graph_tops.insert(g[e].rose_top);
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
        return {};
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
           rdfa == b.rdfa && haig == b.haig;
}

bool RoseSuffixInfo::operator<(const RoseSuffixInfo &b) const {
    const RoseSuffixInfo &a = *this;
    ORDER_CHECK(top);
    ORDER_CHECK(graph);
    ORDER_CHECK(castle);
    ORDER_CHECK(haig);
    ORDER_CHECK(rdfa);
    assert(a.dfa_min_width == b.dfa_min_width);
    assert(a.dfa_max_width == b.dfa_max_width);
    return false;
}


void RoseSuffixInfo::reset(void) {
    top = 0;
    graph.reset();
    castle.reset();
    rdfa.reset();
    haig.reset();
    dfa_min_width = 0;
    dfa_max_width = depth::infinity();
}

std::set<ReportID> all_reports(const suffix_id &s) {
    assert(s.graph() || s.castle() || s.haig() || s.dfa());
    if (s.graph()) {
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
        set<u32> tops;
        const NGHolder &h = *s.graph();
        for (const auto &e : out_edges_range(h.start, h)) {
            if (target(e, h) == h.startDs) {
                continue;
            }
            tops.insert(h[e].top);
        }
        if (tops.empty()) {
            tops.insert(0); // Vacuous graph, triggered on zero top.
        }
        return tops;
    }

    if (s.castle()) {
        return assoc_keys(s.castle()->repeats);
    }

    // Other types of suffix are not multi-top.
    return {0};
}

size_t suffix_id::hash() const {
    size_t val = 0;
    hash_combine(val, g);
    hash_combine(val, c);
    hash_combine(val, d);
    hash_combine(val, h);
    return val;
}

size_t hash_value(const suffix_id &s) {
    return s.hash();
}

bool isAnchored(const left_id &r) {
    assert(r.graph() || r.castle() || r.haig() || r.dfa());
    if (r.graph()) {
        return isAnchored(*r.graph());
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
        set<u32> tops;
        const NGHolder &h = *r.graph();
        for (const auto &e : out_edges_range(h.start, h)) {
            if (target(e, h) == h.startDs) {
                continue;
            }
            tops.insert(h[e].top);
        }
        if (tops.empty()) {
            tops.insert(0); // Vacuous graph, triggered on zero top.
        }
        return tops;
    }

    if (r.castle()) {
        return assoc_keys(r.castle()->repeats);
    }

    // Other types of rose are not multi-top.
    return {0};
}

u32 num_tops(const left_id &r) {
    return all_tops(r).size();
}

size_t left_id::hash() const {
    size_t val = 0;
    hash_combine(val, g);
    hash_combine(val, c);
    hash_combine(val, d);
    hash_combine(val, h);
    return val;
}

size_t hash_value(const left_id &r) {
    return r.hash();
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

void LeftEngInfo::reset(void) {
    graph.reset();
    castle.reset();
    dfa.reset();
    haig.reset();
    lag = 0;
    leftfix_report = MO_INVALID_IDX;
    dfa_min_width = 0;
    dfa_max_width = depth::infinity();
}

LeftEngInfo::operator bool() const {
    assert((int)!!castle + (int)!!dfa + (int)!!haig <= 1);
    assert(!castle || !graph);
    assert(!dfa || graph); /* dfas always have the graph as well */
    assert(!haig || graph);
    return graph || castle || dfa || haig;
}

u32 roseQuality(const RoseEngine *t) {
    /* Rose is low quality if the atable is a Mcclellan 16 or has multiple DFAs
     */
    const anchored_matcher_info *atable = getALiteralMatcher(t);
    if (atable) {
        if (atable->next_offset) {
            DEBUG_PRINTF("multiple atable engines\n");
            return 0;
        }
        const NFA *nfa = (const NFA *)((const char *)atable + sizeof(*atable));

        if (nfa->type != MCCLELLAN_NFA_8) {
            DEBUG_PRINTF("m16 atable engine\n");
            return 0;
        }
    }

    /* if we always run multiple engines then we are slow */
    u32 always_run = 0;

    if (atable) {
        always_run++;
    }

    const HWLM *ftable = getFLiteralMatcher(t);
    if (ftable) {
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

/** \brief Add a SMWR engine to the given RoseEngine. */
aligned_unique_ptr<RoseEngine> roseAddSmallWrite(const RoseEngine *t,
                                                 const SmallWriteEngine *smwr) {
    assert(t);
    assert(smwr);

    const u32 mainSize = roseSize(t);
    const u32 smallWriteSize = smwrSize(smwr);

    u32 smwrOffset = ROUNDUP_CL(mainSize);
    u32 newSize = smwrOffset + smallWriteSize;

    aligned_unique_ptr<RoseEngine> t2 =
        aligned_zmalloc_unique<RoseEngine>(newSize);
    char *ptr = (char *)t2.get();
    memcpy(ptr, t, mainSize);
    memcpy(ptr + smwrOffset, smwr, smallWriteSize);

    t2->smallWriteOffset = smwrOffset;
    t2->size = newSize;

    return t2;
}

#ifndef NDEBUG
/** \brief Returns true if all the graphs (NFA, DFA, Haig, etc) in this Rose
 * graph are implementable. */
bool canImplementGraphs(const RoseBuildImpl &tbi) {
    const RoseGraph &g = tbi.g;

    // First, check the Rose leftfixes.

    for (auto v : vertices_range(g)) {
        DEBUG_PRINTF("leftfix: check vertex %zu\n", g[v].idx);

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
                   == tbi.isRootSuccessor(v) ? NFA_PREFIX : NFA_INFIX);
            if (!isImplementableNFA(*g[v].left.graph, nullptr, tbi.cc)) {
                DEBUG_PRINTF("nfa prefix %zu failed (%zu vertices)\n", g[v].idx,
                             num_vertices(*g[v].left.graph));
                return false;
            }
        }
    }

    // Suffix graphs.

    for (auto v : vertices_range(g)) {
        DEBUG_PRINTF("suffix: check vertex %zu\n", g[v].idx);

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
                DEBUG_PRINTF("nfa suffix %zu failed (%zu vertices)\n", g[v].idx,
                             num_vertices(*suffix.graph));
                return false;
            }
        }
    }

    return true;
}
#endif // NDEBUG

} // namespace ue2
