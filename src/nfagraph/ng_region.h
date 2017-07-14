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

/** \file
 * \brief Region analysis and utility functions.
 */

#ifndef NG_REGION_H
#define NG_REGION_H

#include "ng_holder.h"
#include "util/container.h"
#include "util/graph_range.h"

#include <unordered_map>
#include <vector>

namespace ue2 {

/** \brief Assign a region ID to every vertex in the graph. */
std::unordered_map<NFAVertex, u32> assignRegions(const NGHolder &g);

/** \brief True if vertices \p a and \p b are in the same region. */
template <class Graph>
bool inSameRegion(const Graph &g, NFAVertex a, NFAVertex b,
                  const std::unordered_map<NFAVertex, u32> &region_map) {
    assert(contains(region_map, a) && contains(region_map, b));

    return region_map.at(a) == region_map.at(b) &&
           is_special(a, g) == is_special(b, g);
}

/** \brief True if vertex \p b is in a later region than vertex \p a. */
template <class Graph>
bool inLaterRegion(const Graph &g, NFAVertex a, NFAVertex b,
                   const std::unordered_map<NFAVertex, u32> &region_map) {
    assert(contains(region_map, a) && contains(region_map, b));

    u32 aa = g[a].index;
    u32 bb = g[b].index;

    if (bb == NODE_START || bb == NODE_START_DOTSTAR) {
        return false;
    }

    if (aa == NODE_START || aa == NODE_START_DOTSTAR) {
        return true;
    }

    if (bb == NODE_ACCEPT || bb == NODE_ACCEPT_EOD) {
        return true;
    }
    if (aa == NODE_ACCEPT || aa == NODE_ACCEPT_EOD) {
        return false;
    }

    return region_map.at(a) < region_map.at(b);
}

/** \brief True if vertex \p b is in an earlier region than vertex \p a. */
template <class Graph>
bool inEarlierRegion(const Graph &g, NFAVertex a, NFAVertex b,
                     const std::unordered_map<NFAVertex, u32> &region_map) {
    assert(contains(region_map, a) && contains(region_map, b));

    u32 aa = g[a].index;
    u32 bb = g[b].index;

    if (bb == NODE_START || bb == NODE_START_DOTSTAR) {
        return true;
    }

    if (aa == NODE_START || aa == NODE_START_DOTSTAR) {
        return false;
    }

    if (bb == NODE_ACCEPT || bb == NODE_ACCEPT_EOD) {
        return false;
    }
    if (aa == NODE_ACCEPT || aa == NODE_ACCEPT_EOD) {
        return true;
    }

    return region_map.at(b) < region_map.at(a);
}

/** \brief True if vertex \p v is an entry vertex for its region. */
template <class Graph>
bool isRegionEntry(const Graph &g, NFAVertex v,
                   const std::unordered_map<NFAVertex, u32> &region_map) {
    // Note that some graph types do not have inv_adjacent_vertices, so we must
    // use in_edges here.
    for (const auto &e : in_edges_range(v, g)) {
        if (!inSameRegion(g, v, source(e, g), region_map)) {
            return true;
        }
    }

    return false;
}

/** \brief True if vertex \p v is an exit vertex for its region. */
template <class Graph>
bool isRegionExit(const Graph &g, NFAVertex v,
                  const std::unordered_map<NFAVertex, u32> &region_map) {
    for (auto w : adjacent_vertices_range(v, g)) {
        if (!inSameRegion(g, v, w, region_map)) {
            return true;
        }
    }

    return false;
}

/** \brief True if vertex \p v is in a region all on its own. */
template <class Graph>
bool isSingletonRegion(const Graph &g, NFAVertex v,
                       const std::unordered_map<NFAVertex, u32> &region_map) {
    for (const auto &e : in_edges_range(v, g)) {
        auto u = source(e, g);
        if (u != v && inSameRegion(g, v, u, region_map)) {
            return false;
        }

        for (auto w : ue2::adjacent_vertices_range(u, g)) {
            if (w != v && inSameRegion(g, v, w, region_map)) {
                return false;
            }
        }
    }

    for (auto w : adjacent_vertices_range(v, g)) {
        if (w != v && inSameRegion(g, v, w, region_map)) {
            return false;
        }

        for (const auto &e : in_edges_range(w, g)) {
            auto u = source(e, g);
            if (u != v && inSameRegion(g, v, u, region_map)) {
                return false;
            }
        }

        return true;
    }

    return true;
}

/**
 * \brief True if the region containing vertex \p v is optional. The vertex \p v
 * should be a region leader.
 */
template <class Graph>
bool isOptionalRegion(const Graph &g, NFAVertex v,
                      const std::unordered_map<NFAVertex, u32> &region_map) {
    assert(isRegionEntry(g, v, region_map));

    DEBUG_PRINTF("check if r%u is optional (inspecting v%zu)\n",
                  region_map.at(v), g[v].index);

    // Region zero is never optional.
    assert(contains(region_map, v));
    if (region_map.at(v) == 0) {
        return false;
    }

    // Optional if v has a predecessor in an earlier region that has a
    // successor in a later one.

    for (const auto &e : in_edges_range(v, g)) {
        auto u = source(e, g);
        if (inSameRegion(g, v, u, region_map)) {
            continue;
        }
        DEBUG_PRINTF("  searching from u=%zu\n", g[u].index);

        assert(inEarlierRegion(g, v, u, region_map));

        for (auto w : adjacent_vertices_range(u, g)) {
            DEBUG_PRINTF("    searching to w=%zu\n", g[w].index);
            if (inLaterRegion(g, v, w, region_map)) {
                return true;
            }
        }
        return false;
    }

    return false;
}

} // namespace ue2

#endif
