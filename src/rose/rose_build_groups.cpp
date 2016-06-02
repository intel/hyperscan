/*
 * Copyright (c) 2016, Intel Corporation
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
    /* returns true if it or any of its delay versions have root role */
    for (auto v : build.literal_info[id].vertices) {
        if (build.isRootSuccessor(v)) {
            NGHolder *h = build.g[v].left.graph.get();
            if (!h || proper_out_degree(h->startDs, *h)) {
                return true;
            }
        }
    }

    for (u32 delayed_id : build.literal_info[id].delayed_ids) {
        for (auto v : build.literal_info[delayed_id].vertices) {
            if (build.isRootSuccessor(v)) {
                NGHolder *h = build.g[v].left.graph.get();
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

    boost::topological_sort(g, back_inserter(v_order),
                            vertex_index_map(get(&RoseVertexProps::idx, g)));

    unordered_map<RoseVertex, rose_group> vertex_group_map;
    vertex_group_map.reserve(num_vertices(g));

    const rose_group initial_groups = build.getInitialGroups();

    for (const auto &v : boost::adaptors::reverse(v_order)) {
        DEBUG_PRINTF("vertex %zu\n", g[v].idx);

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
            DEBUG_PRINTF("pred %zu\n", g[u].idx);
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
    return squashable_groups;
}

} // namespace ue2
