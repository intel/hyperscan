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

#include "grey.h"
#include "hs_internal.h"
#include "rose_build_anchored.h"
#include "rose_build_castle.h"
#include "rose_build_convert.h"
#include "rose_build_dump.h"
#include "rose_build_merge.h"
#include "rose_build_role_aliasing.h"
#include "rose_build_util.h"
#include "ue2common.h"
#include "nfa/nfa_internal.h"
#include "nfa/rdfa.h"
#include "nfagraph/ng_holder.h"
#include "nfagraph/ng_dump.h"
#include "nfagraph/ng_execute.h"
#include "nfagraph/ng_is_equal.h"
#include "nfagraph/ng_limex.h"
#include "nfagraph/ng_mcclellan.h"
#include "nfagraph/ng_repeat.h"
#include "nfagraph/ng_reports.h"
#include "nfagraph/ng_stop.h"
#include "nfagraph/ng_util.h"
#include "nfagraph/ng_width.h"
#include "util/bitutils.h"
#include "util/charreach.h"
#include "util/charreach_util.h"
#include "util/compare.h"
#include "util/compile_context.h"
#include "util/container.h"
#include "util/dump_charclass.h"
#include "util/graph_range.h"
#include "util/order_check.h"
#include "util/report_manager.h"
#include "util/ue2_containers.h"
#include "util/ue2string.h"
#include "util/verify_types.h"

#include <algorithm>
#include <functional>
#include <map>
#include <queue>
#include <set>
#include <string>
#include <vector>
#include <utility>

#include <boost/range/adaptor/map.hpp>

using namespace std;
using boost::adaptors::map_keys;
using boost::adaptors::map_values;

namespace ue2 {

#define ROSE_LONG_LITERAL_LEN 8

#define ANCHORED_REHOME_MIN_FLOATING 800
#define ANCHORED_REHOME_MIN_FLOATING_SHORT 50
#define ANCHORED_REHOME_ALLOW_SHORT 20
#define ANCHORED_REHOME_DEEP 25
#define ANCHORED_REHOME_SHORT_LEN 3

static
bool superStrong(const rose_literal_id &lit) {
    if (lit.s.length() < ROSE_LONG_LITERAL_LEN) {
        return false;
    }

    const u32 EXPECTED_FDR_BUCKET_LENGTH = 8;

    assert(lit.s.length() >= EXPECTED_FDR_BUCKET_LENGTH);
    size_t len = lit.s.length();
    const string &s = lit.s.get_string();

    for (size_t i = 1; i < EXPECTED_FDR_BUCKET_LENGTH; i++) {
        if (s[len - 1 - i] != s[len - 1]) {
            return true; /* we have at least some variation in the tail */
        }
    }
    DEBUG_PRINTF("lit '%s' is not superstrong due to tail\n",
                 escapeString(s).c_str());
    return false;
}

rose_group RoseBuildImpl::getGroups(RoseVertex v) const {
    rose_group groups = 0;

    for (u32 id : g[v].literals) {
        u32 lit_id = literal_info.at(id).undelayed_id;

        rose_group mygroups = literal_info[lit_id].group_mask;
        groups |= mygroups;
    }

    return groups;
}

/** \brief Get the groups of the successor literals of a given vertex. */
rose_group RoseBuildImpl::getSuccGroups(RoseVertex start) const {
    rose_group initialGroups = 0;

    for (auto v : adjacent_vertices_range(start, g)) {
        initialGroups |= getGroups(v);
    }

    return initialGroups;
}

#ifdef DEBUG
static UNUSED
void printLitInfo(const rose_literal_info &li, u32 id) {
    DEBUG_PRINTF("lit_info %u\n", id);
    DEBUG_PRINTF("  parent %u%s", li.undelayed_id,
                 li.delayed_ids.empty() ? "":", children:");
    for (u32 d_id : li.delayed_ids) {
        printf(" %u", d_id);
    }
    printf("\n");
    DEBUG_PRINTF("  group %llu %s\n", li.group_mask, li.squash_group ? "s":"");
}
#endif

static
void allocateFinalIdToSet(const RoseGraph &g, const set<u32> &lits,
                          deque<rose_literal_info> *literal_info,
                          map<u32, set<u32> > *final_id_to_literal,
                          u32 *next_final_id) {
    /* We can allocate the same final id to multiple literals of the same type
     * if they share the same vertex set and trigger the same delayed literal
     * ids and squash the same roles and have the same group squashing
     * behaviour. Benefits literals cannot be merged. */

    for (u32 int_id : lits) {
        rose_literal_info &curr_info = (*literal_info)[int_id];
        const auto &verts = curr_info.vertices;

        if (!verts.empty() && !curr_info.requires_benefits
            && curr_info.delayed_ids.empty()) {
            vector<u32> cand;
            insert(&cand, cand.end(), g[*verts.begin()].literals);
            for (auto v : verts) {
                vector<u32> temp;
                set_intersection(cand.begin(), cand.end(),
                                 g[v].literals.begin(),
                                 g[v].literals.end(),
                                 inserter(temp, temp.end()));
                cand.swap(temp);
            }

            for (u32 cand_id : cand) {
                if (cand_id >= int_id) {
                    break;
                }

                const rose_literal_info &cand_info = (*literal_info)[cand_id];

                if (cand_info.requires_benefits) {
                    continue;
                }

                if (!cand_info.delayed_ids.empty()) {
                    /* TODO: allow cases where delayed ids are equivalent.
                     * This is awkward currently as the have not had their
                     * final ids allocated yet */
                    continue;
                }

                if (lits.find(cand_id) == lits.end()
                    || cand_info.vertices.size() != verts.size()
                    || cand_info.squash_group != curr_info.squash_group) {
                    continue;
                }

                /* if we are squashing groups we need to check if they are the
                 * same group */
                if (cand_info.squash_group
                    && cand_info.group_mask != curr_info.group_mask) {
                    continue;
                }

                u32 final_id = cand_info.final_id;
                assert(final_id != MO_INVALID_IDX);
                assert(curr_info.final_id == MO_INVALID_IDX);
                curr_info.final_id = final_id;
                (*final_id_to_literal)[final_id].insert(int_id);
                goto next_lit;
            }
        }

        /* oh well, have to give it a fresh one, hang the expense */
        DEBUG_PRINTF("allocating final id %u to %u\n", *next_final_id, int_id);
                assert(curr_info.final_id == MO_INVALID_IDX);
        curr_info.final_id = *next_final_id;
        (*final_id_to_literal)[*next_final_id].insert(int_id);
        (*next_final_id)++;
    next_lit:;
    }
}

static
bool isUsedLiteral(const RoseBuildImpl &build, u32 lit_id) {
    assert(lit_id < build.literal_info.size());
    const auto &info = build.literal_info[lit_id];
    if (!info.vertices.empty()) {
        return true;
    }

    for (const u32 &delayed_id : info.delayed_ids) {
        assert(delayed_id < build.literal_info.size());
        const rose_literal_info &delayed_info = build.literal_info[delayed_id];
        if (!delayed_info.vertices.empty()) {
            return true;
        }
    }

    DEBUG_PRINTF("literal %u has no refs\n", lit_id);
    return false;
}

/** \brief Allocate final literal IDs for all literals.
 *
 * These are the literal ids used in the bytecode.
 */
static
void allocateFinalLiteralId(RoseBuildImpl &tbi) {
    RoseGraph &g = tbi.g;

    set<u32> anch;
    set<u32> norm;
    set<u32> delay;

    /* undelayed ids come first */
    assert(tbi.final_id_to_literal.empty());
    u32 next_final_id = 0;
    for (u32 i = 0; i < tbi.literal_info.size(); i++) {
        assert(!tbi.hasFinalId(i));

        if (!isUsedLiteral(tbi, i)) {
            /* what is this literal good for? absolutely nothing */
            continue;
        }

        // The special EOD event literal has its own program and does not need
        // a real literal ID.
        if (i == tbi.eod_event_literal_id) {
            assert(tbi.eod_event_literal_id != MO_INVALID_IDX);
            continue;
        }

        if (tbi.isDelayed(i)) {
            assert(!tbi.literal_info[i].requires_benefits);
            delay.insert(i);
        } else if (tbi.literals.right.at(i).table == ROSE_ANCHORED) {
            anch.insert(i);
        } else {
            norm.insert(i);
        }
    }

    /* normal lits */
    allocateFinalIdToSet(g, norm, &tbi.literal_info, &tbi.final_id_to_literal,
                         &next_final_id);

    /* next anchored stuff */
    tbi.anchored_base_id = next_final_id;
    allocateFinalIdToSet(g, anch, &tbi.literal_info, &tbi.final_id_to_literal,
                         &next_final_id);

    /* delayed ids come last */
    tbi.delay_base_id = next_final_id;
    allocateFinalIdToSet(g, delay, &tbi.literal_info, &tbi.final_id_to_literal,
                         &next_final_id);
}

#define MAX_EXPLOSION_NC 3
static
bool limited_explosion(const ue2_literal &s) {
    u32 nc_count = 0;

    for (const auto &e : s) {
        if (e.nocase) {
            nc_count++;
        }
    }

    return nc_count <= MAX_EXPLOSION_NC;
}

void RoseBuildImpl::handleMixedSensitivity(void) {
    for (const auto &e : literals.right) {
        u32 id = e.first;
        const rose_literal_id &lit = e.second;

        if (lit.delay) {
            continue; /* delay id's are virtual-ish */
        }

        if (lit.table == ROSE_ANCHORED || lit.table == ROSE_EVENT) {
            continue; /* wrong table */
        }

        if (!mixed_sensitivity(lit.s)) {
            continue;
        }

        if (limited_explosion(lit.s)) {
            DEBUG_PRINTF("need to explode existing string '%s'\n",
                         dumpString(lit.s).c_str());
            literal_info[id].requires_explode = true;
        } else {
            literal_info[id].requires_benefits = true;
        }
    }
}

// Returns the length of the longest prefix of s that is (a) also a suffix of s
// and (b) not s itself.
static
size_t maxPeriod(const ue2_literal &s) {
    /* overly conservative if only part of the string is nocase */
    if (s.empty()) {
        return 0;
    }

    const size_t len = s.length();
    const char *begin = s.c_str(), *end = begin + len;
    size_t i;
    for (i = len - 1; i != 0; i--) {
        if (!cmp(begin, end - i, i, s.any_nocase())) {
            break;
        }
    }

    return i;
}

bool RoseBuildImpl::isPseudoStar(const RoseEdge &e) const {
    return !g[e].minBound && isPseudoStarOrFirstOnly(e);
}

bool RoseBuildImpl::isPseudoStarOrFirstOnly(const RoseEdge &e) const {
    RoseVertex u = source(e, g);
    RoseVertex v = target(e, g);

    if (g[e].maxBound != ROSE_BOUND_INF) {
        return false;
    }

    if (isAnyStart(u)) {
        return true;
    }

    if (isAnchored(u)) {
        /* anchored table runs out of order */
        return false;
    }

    if (hasDelayedLiteral(u)) {
        return false;
    }

    if (g[v].left) {
        return false;
    }

    if (g[v].eod_accept) {
        return true;
    }

    assert(!g[v].literals.empty());
    if (maxLiteralOverlap(u, v)) {
        return false;
    }

    return true;
}

bool RoseBuildImpl::hasOnlyPseudoStarInEdges(RoseVertex v) const {
    for (const auto &e : in_edges_range(v, g)) {
        if (!isPseudoStar(e)) {
            return false;
        }
    }
    return true;
}

void RoseBuildImpl::renumberVertices() {
    vertexIndex = 0;
    DEBUG_PRINTF("renumbering vertices\n");
    for (auto v : vertices_range(g)) {
        g[v].idx = vertexIndex++;
    }
}

static
size_t trailerDueToSelf(const rose_literal_id &lit) {
    size_t trailer = lit.s.length() - maxPeriod(lit.s);
    if (trailer > 255) {
        return 255;
    }
    if (!trailer) {
        return 1;
    }
    return trailer;
}

static
RoseRoleHistory findHistoryScheme(const RoseBuildImpl &tbi, const RoseEdge &e) {
    const RoseGraph &g = tbi.g;
    const RoseVertex u = source(e, g); /* pred role */
    const RoseVertex v = target(e, g); /* current role */

    DEBUG_PRINTF("find history for [%zu,%zu]\n", g[u].idx, g[v].idx);
    DEBUG_PRINTF("u has min_offset=%u, max_offset=%u\n", g[u].min_offset,
                 g[u].max_offset);

    if (g[v].left) {
        if (!tbi.isAnyStart(u)) {
            /* infix nfa will track history, treat as pseudo .*. Note: rose lits
             * may overlap so rose history track would be wrong anyway */
            DEBUG_PRINTF("skipping history as prefix\n");
            return ROSE_ROLE_HISTORY_NONE;
        }
        if (g[e].minBound || g[e].maxBound != ROSE_BOUND_INF) {
            DEBUG_PRINTF("rose prefix with external bounds\n");
            return ROSE_ROLE_HISTORY_ANCH;
        } else {
            return ROSE_ROLE_HISTORY_NONE;
        }
    }

    // Handle EOD cases.
    if (g[v].eod_accept) {
        const u32 minBound = g[e].minBound, maxBound = g[e].maxBound;
        DEBUG_PRINTF("EOD edge with bounds [%u,%u]\n", minBound, maxBound);

        // Trivial case: we don't need history for {0,inf} bounds
        if (minBound == 0 && maxBound == ROSE_BOUND_INF) {
            return ROSE_ROLE_HISTORY_NONE;
        }

        // Event literals store no history.
        if (tbi.hasLiteralInTable(u, ROSE_EVENT)) {
            return ROSE_ROLE_HISTORY_NONE;
        }

        // Trivial case: fixed offset from anchor
        if (g[u].fixedOffset()) {
            return ROSE_ROLE_HISTORY_ANCH;
        }

        // If the bounds are {0,0}, this role can only match precisely at EOD.
        if (minBound == 0 && maxBound == 0) {
            return ROSE_ROLE_HISTORY_LAST_BYTE;
        }

        // XXX: No other history schemes should be possible any longer.
        assert(0);
    }

    // Non-EOD cases.

    DEBUG_PRINTF("examining edge [%zu,%zu] with bounds {%u,%u}\n",
                 g[u].idx, g[v].idx, g[e].minBound, g[e].maxBound);

    if (tbi.isAnchored(v)) {
        // Matches for literals in the anchored table will always arrive at the
        // right offsets, so there's no need for history-based confirmation.
        DEBUG_PRINTF("v in anchored table, no need for history\n");
        assert(u == tbi.anchored_root);
        return ROSE_ROLE_HISTORY_NONE;
    }

    if (g[u].fixedOffset()) {
        DEBUG_PRINTF("fixed offset -> anch\n");
        return ROSE_ROLE_HISTORY_ANCH;
    }

    return ROSE_ROLE_HISTORY_NONE;
}

static
void assignHistories(RoseBuildImpl &tbi) {
    for (const auto &e : edges_range(tbi.g)) {
        if (tbi.g[e].history == ROSE_ROLE_HISTORY_INVALID) {
            tbi.g[e].history = findHistoryScheme(tbi, e);
        }
    }
}

bool RoseBuildImpl::isDirectReport(u32 id) const {
    assert(id < literal_info.size());

    // Literal info properties.
    const rose_literal_info &info = literal_info[id];
    if (info.vertices.empty()) {
        return false;
    }

    if (!info.delayed_ids.empty() /* dr's don't set groups */
        || info.requires_benefits) { /* dr's don't require confirm */
        return false;
    }

    if (isDelayed(id)) { /* can't handle delayed dr atm as we require delay
                          * ids to be dense */
        return false;
    }

    // Role properties.

    // Note that a literal can have multiple roles and still be a direct
    // report; it'll become a multi-direct report ("MDR") that fires each
    // role's reports from a list.

    for (auto v : info.vertices) {
        assert(contains(g[v].literals, id));

        if (g[v].reports.empty() ||
            g[v].eod_accept || // no accept EOD
            !g[v].isBoring() ||
            !isLeafNode(v, g) || // Must have no out-edges
            in_degree(v, g) != 1) { // Role must have exactly one in-edge
            return false;
        }

        // Use the program to handle cases that aren't external reports.
        for (const ReportID &id : g[v].reports) {
            if (!isExternalReport(rm.getReport(id))) {
                return false;
            }
        }

        if (literals.right.at(id).table == ROSE_ANCHORED) {
            /* in-edges are irrelevant for anchored region. */
            continue;
        }

        /* The in-edge must be an (0, inf) edge from root. */
        assert(in_degree(v, g) != 0);
        RoseEdge e = *(in_edges(v, g).first);
        if (source(e, g) != root || g[e].minBound != 0 ||
            g[e].maxBound != ROSE_BOUND_INF) {
            return false;
        }

        // Note: we allow ekeys; they will result in unused roles being built as
        // direct reporting will be used when actually matching in Rose.
        /* TODO: prevent roles being created */
    }

    DEBUG_PRINTF("literal %u ('%s') is a %s report\n", id,
                 dumpString(literals.right.at(id).s).c_str(),
                 info.vertices.size() > 1 ? "multi-direct" : "direct");
    return true;
}

static
bool checkEodStealFloating(const RoseBuildImpl &tbi,
                           const vector<u32> &eodLiteralsForFloating,
                           u32 numFloatingLiterals,
                           size_t shortestFloatingLen) {
    if (eodLiteralsForFloating.empty()) {
        DEBUG_PRINTF("no eod literals\n");
        return true;
    }

    if (!numFloatingLiterals) {
        DEBUG_PRINTF("no floating table\n");
        return false;
    }

    if (tbi.hasNoFloatingRoots()) {
        DEBUG_PRINTF("skipping as floating table is conditional\n");
        /* TODO: investigate putting stuff in atable */
        return false;
    }

    DEBUG_PRINTF("%zu are eod literals, %u floating; floating len=%zu\n",
                 eodLiteralsForFloating.size(), numFloatingLiterals,
                 shortestFloatingLen);
    u32 new_floating_lits = 0;

    for (u32 eod_id : eodLiteralsForFloating) {
        const rose_literal_id &lit = tbi.literals.right.at(eod_id);
        DEBUG_PRINTF("checking '%s'\n", dumpString(lit.s).c_str());

        if (tbi.hasLiteral(lit.s, ROSE_FLOATING)) {
            DEBUG_PRINTF("skip; there is already a floating version\n");
            continue;
        }

        // Don't want to make the shortest floating literal shorter/worse.
        if (trailerDueToSelf(lit) < 4 || lit.s.length() < shortestFloatingLen) {
            DEBUG_PRINTF("len=%zu, selfOverlap=%zu\n", lit.s.length(),
                         trailerDueToSelf(lit));
            DEBUG_PRINTF("would shorten, bailing\n");
            return false;
        }

        new_floating_lits++;
    }
    DEBUG_PRINTF("..would require %u new floating literals\n",
                  new_floating_lits);

    // Magic number thresholds: we only want to get rid of our EOD table if it
    // would make no real difference to the FDR.
    if (numFloatingLiterals / 8 < new_floating_lits
        && (new_floating_lits > 3 || numFloatingLiterals <= 2)) {
        DEBUG_PRINTF("leaving eod table alone.\n");
        return false;
    }

    return true;
}

static
void promoteEodToFloating(RoseBuildImpl &tbi, const vector<u32> &eodLiterals) {
    DEBUG_PRINTF("promoting eod literals to floating table\n");

    for (u32 eod_id : eodLiterals) {
        const rose_literal_id &lit = tbi.literals.right.at(eod_id);
        u32 floating_id = tbi.getLiteralId(lit.s, lit.msk, lit.cmp, lit.delay,
                                           ROSE_FLOATING);
        auto &float_verts = tbi.literal_info[floating_id].vertices;
        auto &eod_verts = tbi.literal_info[eod_id].vertices;

        insert(&float_verts, eod_verts);
        eod_verts.clear();

        DEBUG_PRINTF("eod_lit=%u -> float_lit=%u\n", eod_id, floating_id);

        for (auto v : float_verts) {
            tbi.g[v].literals.erase(eod_id);
            tbi.g[v].literals.insert(floating_id);
        }

        tbi.literal_info[floating_id].requires_explode
            = tbi.literal_info[eod_id].requires_explode;
        tbi.literal_info[floating_id].requires_benefits
            = tbi.literal_info[eod_id].requires_benefits;
    }
}

static
bool promoteEodToAnchored(RoseBuildImpl &tbi, const vector<u32> &eodLiterals) {
    DEBUG_PRINTF("promoting eod literals to anchored table\n");
    bool rv = true;

    for (u32 eod_id : eodLiterals) {
        const rose_literal_id &lit = tbi.literals.right.at(eod_id);

        NGHolder h;
        add_edge(h.start, h.accept, h);
        appendLiteral(h, lit.s); /* we only accept cases which are anchored
                                  * hard up against start */

        u32 a_id = tbi.getNewLiteralId();
        u32 remap_id = 0;
        DEBUG_PRINTF("  trying to add dfa stuff\n");
        int anch_ok = addToAnchoredMatcher(tbi, h, a_id, &remap_id);

        if (anch_ok == ANCHORED_FAIL) {
            DEBUG_PRINTF("failed to promote to anchored need to keep etable\n");
            rv = false;
            continue;
        } else if (anch_ok == ANCHORED_REMAP) {
            DEBUG_PRINTF("remapped\n");
            a_id = remap_id;
        } else {
            assert(anch_ok == ANCHORED_SUCCESS);
        }

        // Store the literal itself in a side structure so that we can use it
        // for overlap calculations later. This may be obsolete when the old
        // Rose construction path (and its history selection code) goes away.
        tbi.anchoredLitSuffix.insert(make_pair(a_id, lit));

        auto &a_verts = tbi.literal_info[a_id].vertices;
        auto &eod_verts = tbi.literal_info[eod_id].vertices;

        for (auto v : eod_verts) {
            for (const auto &e : in_edges_range(v, tbi.g)) {
                assert(tbi.g[e].maxBound != ROSE_BOUND_INF);
                tbi.g[e].minBound += lit.s.length();
                tbi.g[e].maxBound += lit.s.length();
            }
        }

        insert(&a_verts, eod_verts);
        eod_verts.clear();

        for (auto v : a_verts) {
            tbi.g[v].literals.erase(eod_id);
            tbi.g[v].literals.insert(a_id);
        }
    }

    return rv;
}

static
bool suitableForAnchored(const RoseBuildImpl &tbi, const rose_literal_id &l_id,
                         const rose_literal_info &lit) {
    const RoseGraph &g = tbi.g;

    bool seen = false;
    u32 min_offset = 0;
    u32 max_offset = 0;

    if (!lit.delayed_ids.empty() || l_id.delay) {
        DEBUG_PRINTF("delay\n");
        return false;
    }

    if (!l_id.msk.empty()) {
        DEBUG_PRINTF("msk\n");
        return false;
    }

    for (auto v : lit.vertices) {
        if (!seen) {
            min_offset = g[v].min_offset;
            max_offset = g[v].max_offset;
            seen = true;

            if (max_offset > tbi.cc.grey.maxAnchoredRegion) {
                DEBUG_PRINTF("too deep %u\n", max_offset);
                return false;
            }
        }

        if (max_offset != g[v].max_offset || min_offset != g[v].min_offset) {
            DEBUG_PRINTF(":(\n");
            return false;
        }

        if (!g[v].isBoring()) {
            DEBUG_PRINTF(":(\n");
            return false;
        }

        if (g[v].literals.size() != 1) {
            DEBUG_PRINTF("shared\n");
            return false;
        }

        if (tbi.isNonRootSuccessor(v)) {
            DEBUG_PRINTF("non root\n");
            return false;
        }

        if (max_offset != l_id.s.length() || min_offset != l_id.s.length()) {
            DEBUG_PRINTF("|%zu| (%u,%u):(\n", l_id.s.length(), min_offset,
                          max_offset);
            /* TODO: handle cases with small bounds */
            return false;
        }

        for (auto w : adjacent_vertices_range(v, g)) {
            if (!g[w].eod_accept) {
                DEBUG_PRINTF("non eod accept literal\n");
                return false;
            }
        }
    }
    return true;
}

// If we've got a small number of long, innocuous EOD literals and a large
// floating table, we consider promoting those EOD literals to the floating
// table to avoid having to run both. See UE-2069, consider deleting this and
// replacing with an elegant reverse DFA.
/* We do not want to do this if we would otherwise avoid running the floating
 * table altogether. */
static
void stealEodVertices(RoseBuildImpl &tbi) {
    u32 numFloatingLiterals = 0;
    u32 numAnchoredLiterals = 0;
    size_t shortestFloatingLen = SIZE_MAX;
    vector<u32> eodLiteralsForFloating;
    vector<u32> eodLiteralsForAnchored;
    DEBUG_PRINTF("hi\n");

    for (u32 i = 0; i < tbi.literal_info.size(); i++) {
        const auto &info = tbi.literal_info[i];
        if (info.vertices.empty()) {
            continue; // skip unused literals
        }

        const rose_literal_id &lit = tbi.literals.right.at(i);

        if (lit.table == ROSE_EOD_ANCHORED) {
            if (suitableForAnchored(tbi, lit, info)) {
                eodLiteralsForAnchored.push_back(i);
            } else {
                eodLiteralsForFloating.push_back(i);
            }
        } else if (lit.table == ROSE_FLOATING) {
            numFloatingLiterals++;
            shortestFloatingLen = min(shortestFloatingLen, lit.s.length());
        } else if (lit.table == ROSE_ANCHORED) {
            numAnchoredLiterals++;
        }
    }

    /* given a choice of having either an eod table or an anchored table, we
     * always favour having an anchored table */

    if (!checkEodStealFloating(tbi, eodLiteralsForFloating, numFloatingLiterals,
                               shortestFloatingLen)) {
        DEBUG_PRINTF("removing etable weakens ftable\n");
        return;
    }

    promoteEodToFloating(tbi, eodLiteralsForFloating);

    if (!promoteEodToAnchored(tbi, eodLiteralsForAnchored)) {
        DEBUG_PRINTF("still need ematcher\n");
        return;
    }

    // We're no longer using the EOD matcher.
    tbi.ematcher_region_size = 0;
}

bool RoseBuildImpl::isDelayed(u32 id) const {
    return literal_info.at(id).undelayed_id != id;
}

bool RoseBuildImpl::hasFinalId(u32 id) const {
    return literal_info.at(id).final_id != MO_INVALID_IDX;
}

static
bool eligibleForAlwaysOnGroup(const RoseBuildImpl &tbi, u32 id) {
    /* returns true if it or any of its delay versions have root role */
    for (auto v : tbi.literal_info[id].vertices) {
        if (tbi.isRootSuccessor(v)) {
            NGHolder *h = tbi.g[v].left.graph.get();
            if (!h || proper_out_degree(h->startDs, *h)) {
                return true;
            }
        }
    }

    for (u32 delayed_id : tbi.literal_info[id].delayed_ids) {
        for (auto v : tbi.literal_info[delayed_id].vertices) {
            if (tbi.isRootSuccessor(v)) {
                NGHolder *h = tbi.g[v].left.graph.get();
                if (!h || proper_out_degree(h->startDs, *h)) {
                    return true;
                }
            }
        }
    }

    return false;
}

static
bool requires_group_assignment(const rose_literal_id &lit,
                               const rose_literal_info &info) {
    if (lit.delay) { /* we will check the shadow's master */
        return false;
    }

    if (lit.table == ROSE_ANCHORED || lit.table == ROSE_EVENT) {
        return false;
    }

    // If we already have a group applied, skip.
    if (info.group_mask) {
        return false;
    }

    if (info.vertices.empty() && info.delayed_ids.empty()) {
        DEBUG_PRINTF("literal is good for nothing\n");
        return false;
    }

    return true;
}

static
rose_group calcLocalGroup(const RoseVertex v, const RoseGraph &g,
                          const deque<rose_literal_info> &literal_info,
                          const bool small_literal_count) {
    rose_group local_group = 0;

    for (auto u : inv_adjacent_vertices_range(v, g)) {
        /* In small cases, ensure that siblings have the same rose parentage to
         * allow rose squashing. In larger cases, don't do this as groups are
         * probably too scarce. */
        for (auto w : adjacent_vertices_range(u, g)) {
            if (!small_literal_count || g[v].left == g[w].left) {
                for (u32 lit_id : g[w].literals) {
                    local_group |= literal_info[lit_id].group_mask;
                }
            } else {
                DEBUG_PRINTF("not sibling different mother %zu %zu\n",
                             g[v].idx, g[w].idx);
            }
        }
    }

    return local_group;
}

/* group constants */
#define MAX_LIGHT_LITERAL_CASE 200 /* allow rose to affect group decisions below
                                    * this */

static
flat_set<RoseVertex> getAssociatedVertices(const RoseBuildImpl &build, u32 id) {
    flat_set<RoseVertex> out;
    const auto &info = build.literal_info[id];
    insert(&out, info.vertices);
    for (const auto &delayed : info.delayed_ids) {
        insert(&out, build.literal_info[delayed].vertices);
    }
    return out;
}

static
u32 next_available_group(u32 counter, u32 min_start_group) {
    counter++;
    if (counter == ROSE_GROUPS_MAX) {
        DEBUG_PRINTF("resetting groups\n");
        counter = min_start_group;
    }

    return counter;
}

// Assigns groups to literals in the general case, when we have more literals
// than available groups.
void RoseBuildImpl::assignGroupsToLiterals() {
    bool small_literal_count = literal_info.size() <= MAX_LIGHT_LITERAL_CASE;

    map<u8, u32> groupCount; /* group index to number of members */

    u32 counter = 0;
    u32 group_always_on = 0;

    // First pass: handle always on literals.
    for (const auto &e : literals.right) {
        u32 id = e.first;
        const rose_literal_id &lit = e.second;
        rose_literal_info &info = literal_info[id];

        if (!requires_group_assignment(lit, info)) {
            continue;
        }

        // If this literal has a root role, we always have to search for it
        // anyway, so it goes in the always-on group.
        /* We could end up squashing it if it is followed by a .* */
        if (eligibleForAlwaysOnGroup(*this, id)) {
            info.group_mask = 1ULL << group_always_on;
            groupCount[group_always_on]++;
            continue;
        }
    }

    u32 group_long_lit;
    if (groupCount[group_always_on]) {
        DEBUG_PRINTF("%u always on literals\n", groupCount[group_always_on]);
        group_long_lit = group_always_on;
        counter++;
    } else {
        group_long_lit = counter;
        counter++;
    }

    u32 min_start_group = counter;
    priority_queue<pair<pair<s32, s32>, u32> > pq;

    // Second pass: the other literals.
    for (const auto &e : literals.right) {
        u32 id = e.first;
        const rose_literal_id &lit = e.second;
        rose_literal_info &info = literal_info[id];

        if (!requires_group_assignment(lit, info)) {
            continue;
        }

        assert(!eligibleForAlwaysOnGroup(*this, id));
        pq.push(make_pair(make_pair(-(s32)literal_info[id].vertices.size(),
                                    -(s32)lit.s.length()), id));
    }

    vector<u32> long_lits;
    while (!pq.empty()) {
        u32 id = pq.top().second;
        pq.pop();
        UNUSED const rose_literal_id &lit = literals.right.at(id);
        DEBUG_PRINTF("assigning groups to lit %u (v %zu l %zu)\n", id,
                     literal_info[id].vertices.size(), lit.s.length());

        u8 group_id = 0;
        rose_group group = ~0ULL;
        for (auto v : getAssociatedVertices(*this, id)) {
            rose_group local_group = calcLocalGroup(v, g, literal_info,
                                                    small_literal_count);
            group &= local_group;
            if (!group) {
                break;
            }
        }

        if (group == ~0ULL) {
            goto boring;
        }

        group &= ~((1ULL << min_start_group) - 1); /* ensure the purity of the
                                                    * always_on groups */
        if (!group) {
            goto boring;
        }

        group_id = ctz64(group);

        /* TODO: fairness */
        DEBUG_PRINTF("picking sibling group %hhd\n", group_id);
        literal_info[id].group_mask = 1ULL << group_id;
        groupCount[group_id]++;

        continue;

    boring:
        /* long literals will either be stuck in a mega group or spread around
         * depending on availability */
        if (superStrong(lit)) {
            long_lits.push_back(id);
            continue;
        }

        // Other literals are assigned to our remaining groups round-robin.
        group_id = counter;

        DEBUG_PRINTF("picking boring group %hhd\n", group_id);
        literal_info[id].group_mask = 1ULL << group_id;
        groupCount[group_id]++;
        counter = next_available_group(counter, min_start_group);
    }

    /* spread long literals out amongst unused groups if any, otherwise stick
     * them in the always on the group */

    if (groupCount[counter]) {
        DEBUG_PRINTF("sticking long literals in the image of the always on\n");
        for (u32 lit_id : long_lits) {
            literal_info[lit_id].group_mask = 1ULL << group_long_lit;
            groupCount[group_long_lit]++;
        }
    } else {
        u32 min_long_counter = counter;
        DEBUG_PRINTF("base long lit group = %u\n", min_long_counter);
        for (u32 lit_id : long_lits) {
            u8 group_id = counter;
            literal_info[lit_id].group_mask = 1ULL << group_id;
            groupCount[group_id]++;
            counter = next_available_group(counter, min_long_counter);
        }
    }

    /* assign delayed literals to the same group as their parent */
    for (const auto &e : literals.right) {
        u32 id = e.first;
        const rose_literal_id &lit = e.second;

        if (!lit.delay) {
            continue;
        }

        u32 parent = literal_info[id].undelayed_id;
        DEBUG_PRINTF("%u is shadow picking up groups from %u\n", id, parent);
        assert(literal_info[parent].undelayed_id == parent);
        assert(literal_info[parent].group_mask);
        literal_info[id].group_mask = literal_info[parent].group_mask;
        /* don't increment the group count - these don't really exist */
    }

    DEBUG_PRINTF("populate group to literal mapping\n");
    for (const u32 id : literals.right | map_keys) {
        rose_group groups = literal_info[id].group_mask;
        while (groups) {
            u32 group_id = findAndClearLSB_64(&groups);
            group_to_literal[group_id].insert(id);
        }
    }

    /* find how many groups we allocated */
    for (u32 i = 0; i < ROSE_GROUPS_MAX; i++) {
        if (groupCount[i]) {
            group_end = MAX(group_end, i + 1);
        }
    }
}

bool RoseBuildImpl::hasDelayedLiteral(RoseVertex v) const {
    for (u32 lit_id : g[v].literals) {
        if (literals.right.at(lit_id).delay) {
            return true;
        }
    }

    return false;
}

bool RoseBuildImpl::hasDelayPred(RoseVertex v) const {
    for (auto u : inv_adjacent_vertices_range(v, g)) {
        if (hasDelayedLiteral(u)) {
            return true;
        }
    }

    return false;
}

bool RoseBuildImpl::hasAnchoredTablePred(RoseVertex v) const {
    for (auto u : inv_adjacent_vertices_range(v, g)) {
        if (isAnchored(u)) {
            return true;
        }
    }

    return false;
}

/* returns true if every vertex associated with a groups also belongs to
   lit_info */
static
bool coversGroup(const RoseBuildImpl &tbi, const rose_literal_info &lit_info) {
    if (lit_info.vertices.empty()) {
        DEBUG_PRINTF("no vertices - does not cover\n");
        return false;
    }

    if (!lit_info.group_mask) {
        DEBUG_PRINTF("no group - does not cover\n");
        return false; /* no group (not a floating lit?) */
    }

    assert(popcount64(lit_info.group_mask) == 1);

    /* for each lit in group, ensure that vertices are a subset of lit_info's */
    rose_group groups = lit_info.group_mask;
    while (groups) {
        u32 group_id = findAndClearLSB_64(&groups);
        for (u32 id : tbi.group_to_literal.at(group_id)) {
            DEBUG_PRINTF(" checking against friend %u\n", id);
            if (!is_subset_of(tbi.literal_info[id].vertices,
                              lit_info.vertices)) {
                DEBUG_PRINTF("fail\n");
                return false;
            }
        }
    }

    DEBUG_PRINTF("ok\n");
    return true;
}

static
bool isGroupSquasher(const RoseBuildImpl &tbi, const u32 id /* literal id */,
                     rose_group forbidden_squash_group) {
    const RoseGraph &g = tbi.g;

    const rose_literal_info &lit_info = tbi.literal_info.at(id);

    DEBUG_PRINTF("checking if %u '%s' is a group squasher %016llx\n", id,
                  dumpString(tbi.literals.right.at(id).s).c_str(),
                  lit_info.group_mask);

    if (tbi.literals.right.at(id).table == ROSE_EVENT) {
        DEBUG_PRINTF("event literal, has no groups to squash\n");
        return false;
    }

    if (!coversGroup(tbi, lit_info)) {
        DEBUG_PRINTF("does not cover group\n");
        return false;
    }

    if (lit_info.group_mask & forbidden_squash_group) {
        /* probably a delayed lit */
        DEBUG_PRINTF("skipping as involves a forbidden group\n");
        return false;
    }

    // Single-vertex, less constrained case than the multiple-vertex one below.
    if (lit_info.vertices.size() == 1) {
        const RoseVertex &v = *lit_info.vertices.begin();

        if (tbi.hasDelayPred(v)) { /* due to rebuild issues */
            return false;
        }

        /* there are two ways to be a group squasher:
         * 1) only care about the first accepted match
         * 2) can only match once after a pred match
         *
         * (2) requires analysis of the infix before v and is not implemented,
         * TODO
         */

        /* Case 1 */

        // Can't squash cases with accepts
        if (!g[v].reports.empty()) {
            return false;
        }

        /* Can't squash cases with a suffix without analysis of the suffix.
         * TODO: look at suffixes */
        if (g[v].suffix) {
            return false;
        }

        // Out-edges must have inf max bound, + no other shenanigans */
        for (const auto &e : out_edges_range(v, g)) {
            if (g[e].maxBound != ROSE_BOUND_INF) {
                return false;
            }

            if (g[target(e, g)].left) {
                return false; /* is an infix rose trigger, TODO: analysis */
            }
        }

        DEBUG_PRINTF("%u is a path 1 group squasher\n", id);
        return true;

        /* note: we could also squash the groups of its preds (if nobody else is
         * using them. TODO. */
    }

    // Multiple-vertex case
    for (auto v : lit_info.vertices) {
        assert(!tbi.isAnyStart(v));

        // Can't squash cases with accepts
        if (!g[v].reports.empty()) {
            return false;
        }

        // Suffixes and leftfixes are out too as first literal may not match
        // for everyone.
        if (!g[v].isBoring()) {
            return false;
        }

        /* TODO: checks are solid but we should explain */
        if (tbi.hasDelayPred(v) || tbi.hasAnchoredTablePred(v)) {
            return false;
        }

        // Out-edges must have inf max bound and not directly lead to another
        // vertex with this group, e.g. 'foobar.*foobar'.
        for (const auto &e : out_edges_range(v, g)) {
            if (g[e].maxBound != ROSE_BOUND_INF) {
                return false;
            }
            RoseVertex t = target(e, g);

            if (g[t].left) {
                return false; /* is an infix rose trigger */
            }

            for (u32 lit_id : g[t].literals) {
                if (tbi.literal_info[lit_id].group_mask & lit_info.group_mask) {
                    return false;
                }
            }
        }

        // In-edges must all be dot-stars with no overlap at all, as overlap
        // also causes history to be used.
        /* Different tables are already forbidden by previous checks */
        for (const auto &e : in_edges_range(v, g)) {
            if (!(g[e].minBound == 0 && g[e].maxBound == ROSE_BOUND_INF)) {
                return false;
            }

            // Check overlap, if source was a literal.
            RoseVertex u = source(e, g);
            if (tbi.maxLiteralOverlap(u, v)) {
                return false;
            }
        }
    }

    DEBUG_PRINTF("literal %u is a multi-vertex group squasher\n", id);
    return true;
}

static
void findGroupSquashers(RoseBuildImpl &tbi) {
    rose_group forbidden_squash_group = 0;
    for (const auto &e : tbi.literals.right) {
        if (e.second.delay) {
            forbidden_squash_group |= tbi.literal_info[e.first].group_mask;
        }
    }

    for (u32 id = 0; id < tbi.literal_info.size(); id++) {
        if (isGroupSquasher(tbi, id, forbidden_squash_group)) {
            tbi.literal_info[id].squash_group = true;
        }
    }
}

/**
 * The groups that a role sets are determined by the union of its successor
 * literals. Requires the literals already have had groups assigned.
 */
void RoseBuildImpl::assignGroupsToRoles() {
    /* Note: if there is a succ literal in the sidematcher, its successors
     * literals must be added instead */
    for (auto v : vertices_range(g)) {
        if (isAnyStart(v)) {
            continue;
        }

        const rose_group succ_groups = getSuccGroups(v);
        g[v].groups |= succ_groups;

        if (ghost.find(v) != ghost.end()) {
            /* delayed roles need to supply their groups to the ghost role */
            g[ghost[v]].groups |= succ_groups;
        }

        DEBUG_PRINTF("vertex %zu: groups=%llx\n", g[v].idx, g[v].groups);
    }
}

void RoseBuildImpl::findTransientLeftfixes(void) {
    for (auto v : vertices_range(g)) {
        if (!g[v].left) {
            continue;
        }

        /* infixes can never (or at least not yet) be transient */
        if (isNonRootSuccessor(v)) {
            continue;
        }

        const left_id &left(g[v].left);

        if (::ue2::isAnchored(left) && !isInETable(v)) {
            /* etable prefixes currently MUST be transient as we do not know
             * where we can safely catch them up to (yet). */
            DEBUG_PRINTF("anchored roses in rocky soil are not fleeting\n");
            continue;
        }

        const depth max_width = findMaxWidth(left);
        if (!max_width.is_finite()) {
            DEBUG_PRINTF("inf max width\n");
            continue;
        }

        u32 his = g[v].left.lag + max_width;

        // If this vertex has an event literal, we need to add one to cope
        // with it.
        if (hasLiteralInTable(v, ROSE_EVENT)) {
            his++;
        }

        /* +1 as trigger must appear in main buffer and no byte is needed to
         * decompress the state */
        if (his <= cc.grey.maxHistoryAvailable + 1) {
            transient.insert(left);
            DEBUG_PRINTF("a transient leftfix has been spotted his=%u\n", his);
        }
    }
}

/** Find all the different roses and their associated literals. */
static
map<left_id, vector<RoseVertex>> findLeftSucc(RoseBuildImpl &tbi) {
    map<left_id, vector<RoseVertex>> leftfixes;
    for (auto v : vertices_range(tbi.g)) {
        if (tbi.g[v].left) {
            const LeftEngInfo &lei = tbi.g[v].left;
            leftfixes[lei].push_back(v);
        }
    }
    return leftfixes;
}

static
bool triggerKillsRoseGraph(const RoseBuildImpl &tbi, const left_id &left,
                           const set<ue2_literal> &all_lits,
                           const RoseEdge &e) {
    assert(left.graph());
    const NGHolder &h = *left.graph();

    ue2::flat_set<NFAVertex> all_states;
    insert(&all_states, vertices(h));
    assert(out_degree(h.startDs, h) == 1); /* triggered don't use sds */
    DEBUG_PRINTF("removing sds\n");
    all_states.erase(h.startDs);

    ue2::flat_set<NFAVertex> states;

    /* check each pred literal to see if they all kill previous graph
     * state */
    for (u32 lit_id : tbi.g[source(e, tbi.g)].literals) {
        const rose_literal_id &pred_lit = tbi.literals.right.at(lit_id);
        const ue2_literal s = findNonOverlappingTail(all_lits, pred_lit.s);

        DEBUG_PRINTF("running graph %zu\n", states.size());
        states = execute_graph(h, s, all_states, true);
        DEBUG_PRINTF("ran, %zu states on\n", states.size());

        if (!states.empty()) {
            return false;
        }
    }

    return true;
}

static
bool triggerKillsRose(const RoseBuildImpl &tbi, const left_id &left,
                      const set<ue2_literal> &all_lits, const RoseEdge &e) {
    if (left.haig()) {
        /* TODO: To allow this for som-based engines we would also need to
         * ensure as well that no other triggers can occur at the same location
         * with a different som. */
        return false;
    }

    if (left.graph()) {
        return triggerKillsRoseGraph(tbi, left, all_lits, e);
    }

    if (left.castle()) {
        return triggerKillsRoseCastle(tbi, left, all_lits, e);
    }

    return false;
}

static
void inspectRoseTops(RoseBuildImpl &tbi) {
    /* Sometimes the arrival of a top for a rose infix can ensure that the nfa
     * would be dead at that time. In the case of multiple trigger literals we
     * can only base our decision on that portion of literal after any
     * overlapping literals */

    map<left_id, vector<RoseVertex>> roses =
        findLeftSucc(tbi); /* rose -> succ verts */

    for (const auto &r : roses) {
        const left_id &left = r.first;
        const vector<RoseVertex> &succs = r.second;

        assert(!succs.empty());
        if (tbi.isRootSuccessor(*succs.begin())) {
            /* a prefix is never an infix */
            continue;
        }

        set<u32> tops_seen;
        set<RoseEdge> rose_edges;
        set<u32> pred_lit_ids;

        for (auto v : succs) {
            for (const auto &e : in_edges_range(v, tbi.g)) {
                RoseVertex u = source(e, tbi.g);
                tops_seen.insert(tbi.g[e].rose_top);
                insert(&pred_lit_ids, tbi.g[u].literals);
                rose_edges.insert(e);
            }
        }

        set<ue2_literal> all_lits;

        if (tops_seen.size() > 1) {
            goto next_rose; /* slightly tricky to deal with overlap case */
        }

        for (u32 lit_id : pred_lit_ids) {
            const rose_literal_id &p_lit = tbi.literals.right.at(lit_id);
            if (p_lit.delay || p_lit.table == ROSE_ANCHORED) {
                goto next_rose;
            }
            all_lits.insert(p_lit.s);
            DEBUG_PRINTF("trigger: '%s'\n", dumpString(p_lit.s).c_str());
        }

        DEBUG_PRINTF("rose has %zu trigger literals, %zu edges\n",
                     all_lits.size(), rose_edges.size());

        for (const auto &e : rose_edges) {
            if (triggerKillsRose(tbi, left, all_lits, e)) {
                DEBUG_PRINTF("top will override previous rose state\n");
                tbi.g[e].rose_cancel_prev_top = true;
            }
        }
    next_rose:;
    }
}

static
void buildRoseSquashMasks(RoseBuildImpl &tbi) {
    /* Rose nfa squash masks are applied to the groups when the nfa can no
     * longer match */

    map<left_id, vector<RoseVertex>> roses =
        findLeftSucc(tbi); /* rose -> succ verts */

    /* a rose nfa can squash a group if all literals in that group are a
     * successor of the nfa and all the literals */
    for (const auto &e : roses) {
        const left_id &left = e.first;
        const vector<RoseVertex> &succs = e.second;

        set<u32> lit_ids;
        bool anchored_pred = false;
        for (auto v : succs) {
            lit_ids.insert(tbi.g[v].literals.begin(), tbi.g[v].literals.end());
            for (auto u : inv_adjacent_vertices_range(v, tbi.g)) {
                anchored_pred |= tbi.isAnchored(u);
            }
        }

        /* Due to the anchored table not being able to set groups again,
         * we cannot use a rose nfa for group squashing if it is being triggered
         * from the anchored table and can match more than once. */

        if (anchored_pred) { /* infix with pred in anchored table */
            u32 min_off = ~0U;
            u32 max_off = 0U;
            for (auto v : succs) {
                for (auto u : inv_adjacent_vertices_range(v, tbi.g)) {
                    min_off = min(min_off, tbi.g[u].min_offset);
                    max_off = max(max_off, tbi.g[u].max_offset);
                }
            }
            if (min_off != max_off) {
                 /* leave all groups alone */
                tbi.rose_squash_masks[left] = ~0ULL;
                continue;
            }
        }

        rose_group unsquashable = 0;

        for (u32 lit_id : lit_ids) {
            const rose_literal_info &info = tbi.literal_info[lit_id];
            if (info.vertices.size() > 1 || !info.delayed_ids.empty()) {
                unsquashable |= info.group_mask;
            }
        }

        rose_group squash_mask = ~0ULL; /* leave all groups alone */

        for (u32 i = 0; i < ROSE_GROUPS_MAX; i++) {
            if (is_subset_of(tbi.group_to_literal[i], lit_ids)) {
                squash_mask &= ~(1ULL << i);
            }
        }
        squash_mask |= unsquashable;
        tbi.rose_squash_masks[left] = squash_mask;
    }
}

static
void countFloatingLiterals(const RoseBuildImpl &tbi, u32 *total_count,
                           u32 *short_count) {
    *total_count = 0;
    *short_count = 0;
    for (const rose_literal_id &lit : tbi.literals.right | map_values) {
        if (lit.delay) {
            continue; /* delay id's are virtual-ish */
        }

        if (lit.table != ROSE_FLOATING) {
            continue; /* wrong table */
        }

        ++*total_count;
        if (lit.s.length() <= ANCHORED_REHOME_SHORT_LEN) {
            ++*short_count;
        }
    }
}

static
void rehomeAnchoredLiteral(RoseBuildImpl &tbi, const simple_anchored_info &sai,
                           const set<u32> &lit_ids) {
    /* TODO: verify that vertices only have a single literal at the moment */

    DEBUG_PRINTF("rehoming ^.{%u,%u}%s\n", sai.min_bound, sai.max_bound,
                  dumpString(sai.literal).c_str());

    /* Get a floating literal corresponding to the anchored literal */
    u32 new_literal_id = tbi.getLiteralId(sai.literal, 0, ROSE_FLOATING);
    rose_literal_info &new_lit_info = tbi.literal_info[new_literal_id];
    DEBUG_PRINTF("floating literal id -> %u\n", new_literal_id);

    for (u32 lit_id : lit_ids) {
        rose_literal_info &old_lit_info = tbi.literal_info[lit_id];
        assert(old_lit_info.delayed_ids.empty());

        for (auto v : old_lit_info.vertices) {
            /* Transfer vertex over to new literal id */
            assert(tbi.g[v].literals.size() == 1);
            tbi.g[v].literals.clear();
            tbi.g[v].literals.insert(new_literal_id);
            new_lit_info.vertices.insert(v);

            /* ensure bounds on the vertex's in-edge are correct */
            assert(in_degree(v, tbi.g) == 1);
            const RoseEdge &e = *in_edges(v, tbi.g).first;
            assert(tbi.g[e].minBound == sai.min_bound + sai.literal.length());
            assert(tbi.g[e].maxBound == sai.max_bound + sai.literal.length());
            tbi.g[e].minBound = sai.min_bound;
            tbi.g[e].maxBound = sai.max_bound;
        }

        /* mark the old literal as empty */
        old_lit_info.vertices.clear();
    }
}

static
void rehomeAnchoredLiterals(RoseBuildImpl &tbi) {
    /* if we have many literals in the floating table, we want to push
     * literals which are anchored but deep into the floating table as they
     * are unlikely to reduce the performance of the floating table. */
    u32 total_count;
    u32 short_count;
    countFloatingLiterals(tbi, &total_count, &short_count);

    DEBUG_PRINTF("considering rehoming options\n");

    if (total_count < ANCHORED_REHOME_MIN_FLOATING
        && short_count < ANCHORED_REHOME_MIN_FLOATING_SHORT) {
        DEBUG_PRINTF("not a heavy case %u %u\n", total_count, short_count);
        return;
    }

    u32 min_rehome_len = ANCHORED_REHOME_SHORT_LEN + 1;
    if (short_count >= ANCHORED_REHOME_ALLOW_SHORT) {
        min_rehome_len--;
    }

    for (map<simple_anchored_info, set<u32> >::iterator it
             = tbi.anchored_simple.begin();
         it != tbi.anchored_simple.end();) {
        if (it->first.max_bound < ANCHORED_REHOME_DEEP
            || it->first.literal.length() < min_rehome_len) {
            ++it;
            continue;
        }

        rehomeAnchoredLiteral(tbi, it->first, it->second);
        tbi.anchored_simple.erase(it++);
    }
}

/** \brief Maximum number of single-byte literals to add to the small block
 * table. */
static const size_t MAX_1BYTE_SMALL_BLOCK_LITERALS = 20;

static
void addSmallBlockLiteral(RoseBuildImpl &tbi, const simple_anchored_info &sai,
                          const set<u32> &lit_ids) {
    DEBUG_PRINTF("anchored ^.{%u,%u}%s\n", sai.min_bound, sai.max_bound,
                 dumpString(sai.literal).c_str());

    u32 lit_id = tbi.getLiteralId(sai.literal, 0, ROSE_ANCHORED_SMALL_BLOCK);
    rose_literal_info &lit_info = tbi.literal_info[lit_id];
    DEBUG_PRINTF("anchored small block literal id -> %u\n", lit_id);

    RoseGraph &g = tbi.g;
    const RoseVertex anchored_root = tbi.anchored_root;

    for (u32 old_id : lit_ids) {
        assert(old_id < tbi.literal_info.size());
        const rose_literal_info &li = tbi.literal_info[old_id];

        // For compile determinism, operate over literal vertices in index order.
        vector<RoseVertex> lit_verts(begin(li.vertices), end(li.vertices));
        sort(begin(lit_verts), end(lit_verts), VertexIndexComp(g));

        for (auto lit_v : lit_verts) {
            // Clone vertex with the new literal ID.
            RoseVertex v = add_vertex(g[lit_v], g);
            g[v].idx = tbi.vertexIndex++;
            g[v].literals.clear();
            g[v].literals.insert(lit_id);
            g[v].min_offset = sai.min_bound + sai.literal.length();
            g[v].max_offset = sai.max_bound + sai.literal.length();
            lit_info.vertices.insert(v);

            assert(!g[v].reports.empty());

            bool doDirectReports = true;
            for (ReportID report_id : g[v].reports) {
                const Report &old_rep = tbi.rm.getReport(report_id);
                if (!isExternalReport(old_rep) || old_rep.hasBounds()) {
                    doDirectReports = false;
                    break;
                }
            }

            if (doDirectReports) {
                flat_set<ReportID> dr_reports;
                for (ReportID report_id : g[v].reports) {
                    // These new literal roles can be made direct reports, with
                    // their bounds handled by the bounds on their Report
                    // structures.
                    Report rep(tbi.rm.getReport(report_id)); // copy
                    assert(!rep.hasBounds());
                    rep.minOffset = sai.literal.length() + sai.min_bound;
                    rep.maxOffset = sai.literal.length() + sai.max_bound;
                    dr_reports.insert(tbi.rm.getInternalId(rep));
                }
                g[v].reports = dr_reports;
                RoseEdge e = add_edge(tbi.root, v, g).first;
                g[e].minBound = 0;             // handled by internal_report
                g[e].maxBound = ROSE_BOUND_INF; // handled by internal_report
            } else {
                // If we have a complex internal report, these must become
                // anchored literals with their own roles.
                RoseEdge e = add_edge(anchored_root, v, g).first;
                g[e].minBound = sai.min_bound;
                g[e].maxBound = sai.max_bound;
            }
        }
    }
}

static
void addSmallBlockLiteral(RoseBuildImpl &tbi, const ue2_literal &lit,
                          const flat_set<ReportID> &reports) {
    DEBUG_PRINTF("lit %s, reports: %s\n", dumpString(lit).c_str(),
                 as_string_list(reports).c_str());
    assert(!reports.empty());

    u32 lit_id = tbi.getLiteralId(lit, 0, ROSE_ANCHORED_SMALL_BLOCK);
    assert(lit_id < tbi.literal_info.size());
    rose_literal_info &lit_info = tbi.literal_info[lit_id];

    RoseGraph &g = tbi.g;

    RoseVertex v = add_vertex(g);
    g[v].idx = tbi.vertexIndex++;
    g[v].literals.insert(lit_id);
    g[v].reports = reports;

    RoseEdge e = add_edge(tbi.root, v, g).first;
    g[e].minBound = 0;
    g[e].maxBound = ROSE_BOUND_INF;
    g[v].min_offset = 1;
    g[v].max_offset = ROSE_BOUND_INF;
    lit_info.vertices.insert(v);
}

static
bool stateIsSEPLiteral(const dstate_id_t &s, const symbol_t &sym,
                       const raw_dfa &rdfa) {
    const dstate &ds = rdfa.states[s];
    if (!ds.reports_eod.empty() || ds.reports.empty()) {
        DEBUG_PRINTF("badly formed reports\n");
        return false;
    }

    DEBUG_PRINTF("examine state %u reached by sym %u\n", s, sym);

    for (symbol_t i = 0; i < rdfa.getImplAlphaSize(); i++) {
        const auto &s_next = ds.next[i];
        DEBUG_PRINTF("state %u -> %u on sym %u\n", s, s_next, i);
        if (s_next == DEAD_STATE) {
            continue; // dead, probably pruned
        } else if (s_next == s && i == sym) {
            continue; // self loop on same symbol
        } else if (s_next == rdfa.start_floating) {
            continue; // return to floating start
        }

        // We don't handle any other transitions.
        DEBUG_PRINTF("not single-byte\n");
        return false;
    }

    return true;
}

static
bool extractSEPLiterals(const raw_dfa &rdfa,
                        map<ue2_literal, flat_set<ReportID>> &lits_out) {
    if (rdfa.start_floating == DEAD_STATE) {
        DEBUG_PRINTF("not floating?\n");
        return false;
    }
    if (rdfa.start_anchored != rdfa.start_floating) {
        DEBUG_PRINTF("not all floating?\n");
        return false;
    }

    map<flat_set<ReportID>, vector<u32>> lits; // reports -> symbols

    const dstate &start = rdfa.states[rdfa.start_floating];

    const symbol_t alpha_size = rdfa.getImplAlphaSize();
    for (symbol_t i = 0; i < alpha_size; i++) {
        auto next = start.next[i];
        if (next == DEAD_STATE || next == rdfa.start_floating) {
            continue;
        }

        if (!stateIsSEPLiteral(next, i, rdfa)) {
            return false;
        }
        lits[rdfa.states[next].reports].push_back(i);
    }

    // Map from symbols back to character reachability.
    vector<CharReach> reach(alpha_size);
    for (u32 i = 0; i < N_CHARS; i++) {
        assert(rdfa.alpha_remap[i] < alpha_size);
        reach[rdfa.alpha_remap[i]].set(i);
    }

    for (const auto &m : lits) {
        const auto &reports = m.first;
        const auto &symbols = m.second;

        CharReach cr;
        for (const auto &sym : symbols) {
            cr |= reach[sym];
        }

        for (size_t i = cr.find_first(); i != cr.npos; i = cr.find_next(i)) {
            if (myisupper(i) && cr.test(mytolower(i))) {
                // ignore upper half of a nocase pair
                continue;
            }

            bool nocase = myislower(i) && cr.test(mytoupper(i));
            insert(&lits_out[ue2_literal((char)i, nocase)], reports);
        }
    }

    return true;
}

static
bool extractSEPLiterals(const OutfixInfo &outfix, const ReportManager &rm,
                        map<ue2_literal, flat_set<ReportID>> &lits_out) {
    if (outfix.minWidth != depth(1) || outfix.maxWidth != depth(1)) {
        DEBUG_PRINTF("outfix must be fixed width of one\n");
        return false;
    }

    for (const auto &report_id : all_reports(outfix)) {
        const auto &report = rm.getReport(report_id);
        if (!isSimpleExhaustible(report)) {
            DEBUG_PRINTF("report id %u not simple exhaustible\n", report_id);
            return false;
        }
    }

    // SEP cases should always become DFAs, so that's the only extract code we
    // have implemented here.

    if (outfix.rdfa()) {
        return extractSEPLiterals(*outfix.rdfa(), lits_out);
    }

    DEBUG_PRINTF("cannot extract literals from outfix type\n");
    return false;
}

static
void addAnchoredSmallBlockLiterals(RoseBuildImpl &tbi) {
    if (tbi.cc.streaming) {
        DEBUG_PRINTF("not block mode\n");
        return;
    }
    if (!tbi.anchored_nfas.empty()) {
        DEBUG_PRINTF("anchored table is not purely literal\n");
        return;
    }

    // At the moment, we only use the small-block matcher if all our anchored
    // literals are direct reports (i.e. leaf nodes in the Rose graph).
    for (const set<u32> &lits : tbi.anchored_simple | map_values) {
        for (u32 lit_id : lits) {
            if (!tbi.isDirectReport(lit_id)) {
                DEBUG_PRINTF("not all anchored lits are direct reports\n");
                return;
            }
        }
    }

    vector<pair<simple_anchored_info, set<u32> > > anchored_lits;
    vector<OutfixInfo *> sep_outfixes;
    size_t oneByteLiterals = 0;

    for (const auto &e : tbi.anchored_simple) {
        const simple_anchored_info &sai = e.first;
        const set<u32> &lit_ids = e.second;

        if (sai.literal.length() + sai.min_bound > ROSE_SMALL_BLOCK_LEN) {
            DEBUG_PRINTF("skipping literal '%s' with min bound %u that cannot "
                         "match inside small block width\n",
                         dumpString(sai.literal).c_str(), sai.min_bound);
        }

        anchored_lits.push_back(make_pair(sai, lit_ids));
        if (sai.literal.length() == 1) {
            oneByteLiterals++;
        }
    }

    // Capture SEP outfixes as well, adding them as literals to the small block
    // table.
    map<ue2_literal, flat_set<ReportID>> sep_literals;
    for (OutfixInfo &oi : tbi.outfixes) {
        if (extractSEPLiterals(oi, tbi.rm, sep_literals)) {
            sep_outfixes.push_back(&oi);
        }
    }

    oneByteLiterals += sep_literals.size();
    DEBUG_PRINTF("%zu one-byte literals\n", oneByteLiterals);
    if (oneByteLiterals > MAX_1BYTE_SMALL_BLOCK_LITERALS) {
        DEBUG_PRINTF("too many one-byte literals, not building small block "
                     "table!\n");
        return;
    }

    for (const auto &e : tbi.anchored_simple) {
        const simple_anchored_info &sai = e.first;
        const set<u32> &lit_ids = e.second;

        addSmallBlockLiteral(tbi, sai, lit_ids);
    }

    for (const auto &m : sep_literals) {
        addSmallBlockLiteral(tbi, m.first, m.second);
    }

    for (OutfixInfo *oi : sep_outfixes) {
        assert(oi);
        oi->in_sbmatcher = true;
    }
}

#ifndef NDEBUG
static
bool historiesAreValid(const RoseGraph &g) {
    for (const auto &e : edges_range(g)) {
        if (g[e].history == ROSE_ROLE_HISTORY_INVALID) {
            DEBUG_PRINTF("edge [%zu,%zu] has invalid history\n",
                         g[source(e, g)].idx, g[target(e, g)].idx);
            return false;
        }
    }

    return true;
}

/**
 * Assertion: Returns true if we have a reference hanging around to a vertex
 * that no longer exists in the graph.
 */
static
bool danglingVertexRef(RoseBuildImpl &tbi) {
    RoseGraph::vertex_iterator vi, ve;
    tie(vi, ve) = vertices(tbi.g);
    const ue2::unordered_set<RoseVertex> valid_vertices(vi, ve);

    if (!contains(valid_vertices, tbi.anchored_root)) {
        DEBUG_PRINTF("anchored root vertex %p not in graph\n",
                     tbi.anchored_root);
        return true;
    }

    for (const auto &e : tbi.ghost) {
        if (!contains(valid_vertices, e.first)) {
            DEBUG_PRINTF("ghost key vertex %p not in graph\n", e.first);
            return true;
        }
        if (!contains(valid_vertices, e.second)) {
            DEBUG_PRINTF("ghost value vertex %p not in graph\n", e.second);
            return true;
        }
    }

    return false;
}

static
bool roleOffsetsAreValid(const RoseGraph &g) {
    for (auto v : vertices_range(g)) {
        if (g[v].min_offset >= ROSE_BOUND_INF) {
            DEBUG_PRINTF("invalid min_offset for role %zu\n", g[v].idx);
            return false;
        }
        if (g[v].min_offset > g[v].max_offset) {
            DEBUG_PRINTF("min_offset > max_offset for %zu\n", g[v].idx);
            return false;
        }
    }
    return true;
}

static UNUSED
bool hasOrphanedTops(const RoseBuildImpl &tbi) {
    const RoseGraph &g = tbi.g;

    ue2::unordered_map<left_id, set<u32> > roses;
    ue2::unordered_map<suffix_id, set<u32> > suffixes;

    for (auto v : vertices_range(g)) {
        if (g[v].left) {
            set<u32> &tops = roses[g[v].left];
            if (tbi.isRootSuccessor(v)) {
                // Prefix, has only one top.
                tops.insert(0);
            } else {
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

    for (const auto &e : roses) {
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

aligned_unique_ptr<RoseEngine> RoseBuildImpl::buildRose(u32 minWidth) {
    dumpRoseGraph(*this, nullptr, "rose_early.dot");

    // Early check for Rose implementability.
    assert(canImplementGraphs(*this));

    // Sanity check vertex role offsets.
    assert(roleOffsetsAreValid(g));

    convertPrefixToBounds(*this);

    // Turn flood-prone suffixes into suffix NFAs.
    convertFloodProneSuffixes(*this);

    // Turn repeats into Castle prototypes.
    makeCastles(*this);

    rehomeAnchoredLiterals(*this);

    // If we've got a very small number of EOD-anchored literals, consider
    // moving them into the floating table so that we only have one literal
    // matcher to run. Note that this needs to happen before
    // addAnchoredSmallBlockLiterals as it may create anchored literals.
    assert(roleOffsetsAreValid(g));
    stealEodVertices(*this);

    addAnchoredSmallBlockLiterals(*this);

    // Merge duplicate leaf nodes
    dedupeSuffixes(*this);
    if (cc.grey.roseGraphReduction) {
        mergeDupeLeaves(*this);
        uncalcLeaves(*this);
    }

    assert(roleOffsetsAreValid(g));
    handleMixedSensitivity();

    assignHistories(*this);

    convertAnchPrefixToBounds(*this);

    // Do some final graph reduction.
    dedupeLeftfixes(*this);
    aliasRoles(*this, false); // Don't merge leftfixes.
    dedupeLeftfixes(*this);

    convertBadLeaves(*this);
    uncalcLeaves(*this);

    /* note the leftfixes which do not need to keep state across stream
       boundaries */
    findTransientLeftfixes();

    dedupeLeftfixesVariableLag(*this);
    mergeLeftfixesVariableLag(*this);
    mergeSmallLeftfixes(*this);
    mergeCastleLeftfixes(*this);

    // Do a rose-merging aliasing pass.
    aliasRoles(*this, true);

    // Merging of suffixes _below_ role aliasing, as otherwise we'd have to
    // teach role aliasing about suffix tops.
    mergeCastleSuffixes(*this);
    mergePuffixes(*this);
    mergeAcyclicSuffixes(*this);
    mergeSmallSuffixes(*this);

    // Convert Castles that would be better off as NFAs back to NGHolder
    // infixes/suffixes.
    if (unmakeCastles(*this)) {
        // We may be able to save some stream state by merging the newly
        // "unmade" Castles.
        mergeSmallSuffixes(*this);
        mergeSmallLeftfixes(*this);
    }

    // Do a rose-merging aliasing pass.
    aliasRoles(*this, true);

    // Run a merge pass over the outfixes as well.
    mergeOutfixes(*this);

    assert(!danglingVertexRef(*this));

    assignGroupsToLiterals();
    assignGroupsToRoles();
    findGroupSquashers(*this);

    /* final prep work */
    remapCastleTops(*this);
    allocateFinalLiteralId(*this);
    inspectRoseTops(*this);
    buildRoseSquashMasks(*this);

    rm.assignDkeys(this);

    /* transfer mpv outfix to main queue */
    if (mpv_outfix) {
        outfixes.push_back(move(*mpv_outfix));
        mpv_outfix = nullptr;
    }

    assert(canImplementGraphs(*this));
    assert(!hasOrphanedTops(*this));
    assert(roleOffsetsAreValid(g));
    assert(historiesAreValid(g));

    dumpRoseGraph(*this, nullptr, "rose_pre_norm.dot");

    return buildFinalEngine(minWidth);
}

} // namespace ue2
