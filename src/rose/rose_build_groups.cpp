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

#include <vector>

#include <boost/graph/topological_sort.hpp>
#include <boost/range/adaptor/reversed.hpp>

using namespace std;

namespace ue2 {

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
