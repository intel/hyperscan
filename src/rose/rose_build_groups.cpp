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
 * \brief Rose build: code for analysing literal groups.
 */

#include "rose_build_groups.h"

#include "util/boundary_reports.h"
#include "util/compile_context.h"
#include "util/report_manager.h"

#include <queue>
#include <vector>

#include <boost/graph/topological_sort.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/range/adaptor/reversed.hpp>

using namespace std;
using boost::adaptors::map_keys;

namespace ue2 {

#define ROSE_LONG_LITERAL_LEN 8

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

static
bool eligibleForAlwaysOnGroup(const RoseBuildImpl &build, u32 id) {
    auto eligble = [&](RoseVertex v) {
        return build.isRootSuccessor(v)
        && (!build.g[v].left || !isAnchored(build.g[v].left));
    };

    if (any_of_in(build.literal_info[id].vertices, eligble)) {
        return true;
    }

    for (u32 delayed_id : build.literal_info[id].delayed_ids) {
        if (any_of_in(build.literal_info[delayed_id].vertices, eligble)) {
            return true;
        }
    }

    return false;
}

static
bool requires_group_assignment(const rose_literal_id &lit,
                               const rose_literal_info &info) {
    if (lit.delay) { /* we will check the shadow's leader */
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
                             g[v].index, g[w].index);
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

static
void allocateGroupForBoundary(RoseBuildImpl &build, u32 group_always_on,
                              map<u8, u32> &groupCount) {
    /* Boundary reports at zero will always fired and forgotten, no need to
     * worry about preventing the stream being marked as exhausted */
    if (build.boundary.report_at_eod.empty()) {
        return;
    }

    /* Group based stream exhaustion is only done at stream boundaries */
    if (!build.cc.streaming) {
        return;
    }

    DEBUG_PRINTF("allocating %u as boundary group id\n", group_always_on);

    build.boundary_group_mask = 1ULL << group_always_on;
    groupCount[group_always_on]++;
}

static
void allocateGroupForEvent(RoseBuildImpl &build, u32 group_always_on,
                           map<u8, u32> &groupCount, u32 *counter) {
    if (build.eod_event_literal_id == MO_INVALID_IDX) {
        return;
    }

    /* Group based stream exhaustion is only done at stream boundaries */
    if (!build.cc.streaming) {
        return;
    }

    rose_literal_info &info = build.literal_info[build.eod_event_literal_id];

    if (info.vertices.empty()) {
        return;
    }

    bool new_group = !groupCount[group_always_on];
    for (RoseVertex v : info.vertices) {
        if (build.g[v].left && !isAnchored(build.g[v].left)) {
            new_group = false;
        }
    }

    u32 group;
    if (!new_group) {
        group = group_always_on;
    } else {
        group = *counter;
        *counter += 1;
    }

    DEBUG_PRINTF("allocating %u as eod event group id\n", *counter);
    info.group_mask = 1ULL << group;
    groupCount[group]++;
}

void assignGroupsToLiterals(RoseBuildImpl &build) {
    auto &literals = build.literals;
    auto &literal_info = build.literal_info;

    bool small_literal_count = literal_info.size() <= MAX_LIGHT_LITERAL_CASE;

    map<u8, u32> groupCount; /* group index to number of members */

    u32 counter = 0;
    u32 group_always_on = 0;

    // First pass: handle always on literals.
    for (u32 id = 0; id < literals.size(); id++) {
        const rose_literal_id &lit = literals.at(id);
        rose_literal_info &info = literal_info[id];

        if (!requires_group_assignment(lit, info)) {
            continue;
        }

        // If this literal has a root role, we always have to search for it
        // anyway, so it goes in the always-on group.
        /* We could end up squashing it if it is followed by a .* */
        if (eligibleForAlwaysOnGroup(build, id)) {
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

    allocateGroupForBoundary(build, group_always_on, groupCount);
    allocateGroupForEvent(build, group_always_on, groupCount, &counter);

    u32 min_start_group = counter;
    priority_queue<tuple<s32, s32, u32>> pq;

    // Second pass: the other literals.
    for (u32 id = 0; id < literals.size(); id++) {
        const rose_literal_id &lit = literals.at(id);
        rose_literal_info &info = literal_info[id];

        if (!requires_group_assignment(lit, info)) {
            continue;
        }

        assert(!eligibleForAlwaysOnGroup(build, id));
        pq.emplace(-(s32)info.vertices.size(), -(s32)lit.s.length(), id);
    }
    vector<u32> long_lits;
    while (!pq.empty()) {
        u32 id = get<2>(pq.top());
        pq.pop();
        UNUSED const rose_literal_id &lit = literals.at(id);
        DEBUG_PRINTF("assigning groups to lit %u (v %zu l %zu)\n", id,
                     literal_info[id].vertices.size(), lit.s.length());

        u8 group_id = 0;
        rose_group group = ~0ULL;
        for (auto v : getAssociatedVertices(build, id)) {
            rose_group local_group = calcLocalGroup(v, build.g, literal_info,
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
    for (u32 id = 0; id < literals.size(); id++) {
        const rose_literal_id &lit = literals.at(id);

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
    for (u32 id = 0; id < literals.size(); id++) {
        rose_group groups = literal_info[id].group_mask;
        while (groups) {
            u32 group_id = findAndClearLSB_64(&groups);
            build.group_to_literal[group_id].insert(id);
        }
    }

    /* find how many groups we allocated */
    for (u32 i = 0; i < ROSE_GROUPS_MAX; i++) {
        if (groupCount[i]) {
            build.group_end = max(build.group_end, i + 1);
        }
    }
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

/**
 * The groups that a role sets are determined by the union of its successor
 * literals. Requires the literals already have had groups assigned.
 */
void assignGroupsToRoles(RoseBuildImpl &build) {
    auto &g = build.g;

    /* Note: if there is a succ literal in the sidematcher, its successors
     * literals must be added instead */
    for (auto v : vertices_range(g)) {
        if (build.isAnyStart(v)) {
            continue;
        }

        const rose_group succ_groups = build.getSuccGroups(v);
        g[v].groups |= succ_groups;

        auto ghost_it = build.ghost.find(v);
        if (ghost_it != end(build.ghost)) {
            /* delayed roles need to supply their groups to the ghost role */
            g[ghost_it->second].groups |= succ_groups;
        }

        DEBUG_PRINTF("vertex %zu: groups=%llx\n", g[v].index, g[v].groups);
    }
}

/**
 * \brief Returns a mapping from each graph vertex v to the intersection of the
 * groups switched on by all of the paths leading up to (and including) v from
 * the start vertexes.
 */
unordered_map<RoseVertex, rose_group>
getVertexGroupMap(const RoseBuildImpl &build) {
    const RoseGraph &g = build.g;
    vector<RoseVertex> v_order;
    v_order.reserve(num_vertices(g));

    boost::topological_sort(g, back_inserter(v_order));

    unordered_map<RoseVertex, rose_group> vertex_group_map;
    vertex_group_map.reserve(num_vertices(g));

    const rose_group initial_groups = build.getInitialGroups();

    for (const auto &v : boost::adaptors::reverse(v_order)) {
        DEBUG_PRINTF("vertex %zu\n", g[v].index);

        if (build.isAnyStart(v)) {
            DEBUG_PRINTF("start vertex, groups=0x%llx\n", initial_groups);
            vertex_group_map.emplace(v, initial_groups);
            continue;
        }

        // To get to this vertex, we must have come through a predecessor, and
        // everyone who isn't a start vertex has one.
        assert(in_degree(v, g) > 0);
        rose_group pred_groups = ~rose_group{0};
        for (auto u : inv_adjacent_vertices_range(v, g)) {
            DEBUG_PRINTF("pred %zu\n", g[u].index);
            assert(contains(vertex_group_map, u));
            pred_groups &= vertex_group_map.at(u);
        }

        DEBUG_PRINTF("pred_groups=0x%llx\n", pred_groups);
        DEBUG_PRINTF("g[v].groups=0x%llx\n", g[v].groups);

        rose_group v_groups = pred_groups | g[v].groups;
        DEBUG_PRINTF("v_groups=0x%llx\n", v_groups);

        vertex_group_map.emplace(v, v_groups);
    }

    return vertex_group_map;
}

/**
 * \brief Find the set of groups that can be squashed anywhere in the graph,
 * either by a literal or by a leftfix.
 */
rose_group getSquashableGroups(const RoseBuildImpl &build) {
    rose_group squashable_groups = 0;
    for (const auto &info : build.literal_info) {
        if (info.squash_group) {
            DEBUG_PRINTF("lit squash mask 0x%llx\n", info.group_mask);
            squashable_groups |= info.group_mask;
        }
    }
    for (const auto &m : build.rose_squash_masks) {
        DEBUG_PRINTF("left squash mask 0x%llx\n", ~m.second);
        squashable_groups |= ~m.second;
    }

    DEBUG_PRINTF("squashable groups=0x%llx\n", squashable_groups);
    assert(!(squashable_groups & build.boundary_group_mask));
    return squashable_groups;
}

/**
 * \brief True if every vertex associated with a group also belongs to
 * lit_info.
 */
static
bool coversGroup(const RoseBuildImpl &build,
                 const rose_literal_info &lit_info) {
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
        for (u32 id : build.group_to_literal.at(group_id)) {
            DEBUG_PRINTF(" checking against friend %u\n", id);
            if (!is_subset_of(build.literal_info[id].vertices,
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
bool isGroupSquasher(const RoseBuildImpl &build, const u32 id /* literal id */,
                     rose_group forbidden_squash_group) {
    const RoseGraph &g = build.g;

    const rose_literal_info &lit_info = build.literal_info.at(id);

    DEBUG_PRINTF("checking if %u '%s' is a group squasher %016llx\n", id,
                 dumpString(build.literals.at(id).s).c_str(),
                 lit_info.group_mask);

    if (build.literals.at(id).table == ROSE_EVENT) {
        DEBUG_PRINTF("event literal\n");
        return false;
    }

    if (!coversGroup(build, lit_info)) {
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

        if (build.hasDelayPred(v)) { /* due to rebuild issues */
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

        // Can't squash cases with accepts unless they are all
        // simple-exhaustible.
        if (any_of_in(g[v].reports, [&](ReportID report) {
                return !isSimpleExhaustible(build.rm.getReport(report));
            })) {
            DEBUG_PRINTF("can't squash reporter\n");
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
        assert(!build.isAnyStart(v));

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
        if (build.hasDelayPred(v) || build.hasAnchoredTablePred(v)) {
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
                if (build.literal_info[lit_id].group_mask &
                    lit_info.group_mask) {
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
            if (build.maxLiteralOverlap(u, v)) {
                return false;
            }
        }
    }

    DEBUG_PRINTF("literal %u is a multi-vertex group squasher\n", id);
    return true;
}

void findGroupSquashers(RoseBuildImpl &build) {
    rose_group forbidden_squash_group = build.boundary_group_mask;
    for (u32 id = 0; id < build.literals.size(); id++) {
        const auto &lit = build.literals.at(id);
        if (lit.delay) {
            forbidden_squash_group |= build.literal_info[id].group_mask;
        }
    }

    for (u32 id = 0; id < build.literal_info.size(); id++) {
        if (isGroupSquasher(build, id, forbidden_squash_group)) {
            build.literal_info[id].squash_group = true;
        }
    }
}

} // namespace ue2
