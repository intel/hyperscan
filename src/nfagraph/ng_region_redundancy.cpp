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
 * \brief Region Redundancy optimisation pass.
 *
 * Identifies and removes entire regions that are adjacent to a cyclic state
 * with a superset of their character reachability.
 */
#include "ng_region_redundancy.h"

#include "ng_holder.h"
#include "ng_region.h"
#include "ng_util.h"
#include "ue2common.h"
#include "util/container.h"
#include "util/graph_range.h"

#include <set>

using namespace std;

namespace ue2 {

namespace {

/** Precalculated information about a region. */
struct RegionInfo {
    NFAVertex entry; //!< arbitrary entry vertex
    CharReach cr;    //!< union of the reach of all vertices in region
};

} // namespace

static
bool regionHasUnexpectedAccept(const NGHolder &g, const u32 region,
                       const flat_set<ReportID> &expected_reports,
                       const unordered_map<NFAVertex, u32> &region_map) {
    /* TODO: only check vertices connected to accept/acceptEOD */
    for (auto v : vertices_range(g)) {
        if (region != region_map.at(v)) {
            continue;
        }

        if (is_any_accept(v, g)) {
            return true; /* encountering an actual special in the region is
                          * possible but definitely unexpected */
        }

        for (auto w : adjacent_vertices_range(v, g)) {
            if (is_any_accept(w, g) && g[v].reports != expected_reports) {
                return true;
            }
        }
    }
    return false;
}

static
void processCyclicStateForward(NGHolder &h, NFAVertex cyc,
                         const map<u32, RegionInfo> &info,
                         const unordered_map<NFAVertex, u32> &region_map,
                         set<u32> &deadRegions) {
    u32 region = region_map.at(cyc);
    CharReach cr = h[cyc].char_reach;
    auto reports = h[cyc].reports;

    DEBUG_PRINTF("going forward from %zu/%u\n", h[cyc].index,
                 region);

    map<u32, RegionInfo>::const_iterator it;
    while ((it = info.find(++region)) != info.end()) {
        NFAVertex v = it->second.entry;
        const CharReach &region_cr = it->second.cr;
        assert(isRegionEntry(h, v, region_map) && !is_special(v, h));
        DEBUG_PRINTF("checking %zu\n", h[v].index);

        if (!region_cr.isSubsetOf(cr)) {
            DEBUG_PRINTF("doesn't cover the reach of region %u\n", region);
            break;
        }

        if (isOptionalRegion(h, v, region_map)
            && !regionHasUnexpectedAccept(h, region, reports, region_map)) {
            DEBUG_PRINTF("cyclic state %zu leads to optional region leader"
                         " %zu\n", h[cyc].index, h[v].index);
            deadRegions.insert(region);
        } else if (isSingletonRegion(h, v, region_map)) {
            /* we can use this region as straw and suck in optional regions on
             * the other side. This allows us to transform /a{n,m}/ to /a{n}/ */
            cr = h[v].char_reach;
            reports = h[v].reports;
            DEBUG_PRINTF("%u is straw\n", region);
            assert(cr.isSubsetOf(h[cyc].char_reach));
            if (hasSelfLoop(v, h)) {
                DEBUG_PRINTF("%u is straw has a self-loop - kill\n", region);
                remove_edge(v, v, h);
            }
        } else {
            break;
        }
    }
}

static
void processCyclicStateReverse(NGHolder &h, NFAVertex cyc,
                         const map<u32, RegionInfo> &info,
                         const unordered_map<NFAVertex, u32> &region_map,
                         set<u32> &deadRegions) {
    u32 region = region_map.at(cyc);
    CharReach cr = h[cyc].char_reach;
    auto reports = h[cyc].reports;

    DEBUG_PRINTF("going back from %zu/%u\n", h[cyc].index, region);

    map<u32, RegionInfo>::const_iterator it;
    while ((it = info.find(--region)) != info.end()) {
        NFAVertex v = it->second.entry;
        const CharReach &region_cr = it->second.cr;
        assert(isRegionEntry(h, v, region_map) && !is_special(v, h));
        DEBUG_PRINTF("checking %zu\n", h[v].index);

        if (!region_cr.isSubsetOf(cr)) {
            DEBUG_PRINTF("doesn't cover the reach of region %u\n", region);
            break;
        }

        if (isOptionalRegion(h, v, region_map)
            && !regionHasUnexpectedAccept(h, region, reports, region_map)) {
            DEBUG_PRINTF("cyclic state %zu trails optional region leader %zu\n",
                         h[cyc].index, h[v].index);
            deadRegions.insert(region);
        } else if (isSingletonRegion(h, v, region_map)) {
            /* we can use this region as a reverse straw and suck in optional
             * regions on the other side. This allows us to transform
             * /^a?a{n}.*b/ to /^a{n}.*b/ */
            cr = h[v].char_reach;
            reports = h[v].reports;
            DEBUG_PRINTF("%u is straw\n", region);
            assert(cr.isSubsetOf(h[cyc].char_reach));
            if (hasSelfLoop(v, h)) {
                DEBUG_PRINTF("%u is straw has a self-loop - kill\n", region);
                remove_edge(v, v, h);
            }
        } else {
            break;
        }

        if (!region) { // No wrapping
            break;
        }
    }
}

static
map<u32, RegionInfo> buildRegionInfoMap(const NGHolder &g,
                   const unordered_map<NFAVertex, u32> &region_map) {
    map<u32, RegionInfo> info;

    for (auto v : vertices_range(g)) {
        u32 region = region_map.at(v);
        if (is_special(v, g) || region == 0) {
            continue;
        }

        RegionInfo &ri = info[region];
        ri.cr |= g[v].char_reach;
        if (isRegionEntry(g, v, region_map)) {
            ri.entry = v;
        }
    }

    return info;
}

static
bool hasNoStartAnchoring(const NGHolder &h) {
    for (auto v : adjacent_vertices_range(h.start, h)) {
        if (!edge(h.startDs, v, h).second) {
            return false;
        }
    }
    return true;
}

void removeRegionRedundancy(NGHolder &g, som_type som) {
    auto region_map = assignRegions(g);

    map<u32, RegionInfo> info = buildRegionInfoMap(g, region_map);

    set<u32> deadRegions;

    /* if we are not tracking som, we can treat sds as a cyclic region if there
     * is no anchoring */
    if (!som && hasNoStartAnchoring(g)) {
        processCyclicStateForward(g, g.startDs, info, region_map, deadRegions);
    }

    // Walk the region mapping, looking for regions that consist of a single
    // cyclic node.

    for (const auto &m : info) {
        // Must not have already been removed
        if (contains(deadRegions, m.first)) {
            continue;
        }

        NFAVertex v = m.second.entry;
        /* require a singleton cyclic region */
        if (!hasSelfLoop(v, g) || !isSingletonRegion(g, v, region_map)) {
            continue;
        }

        if (som && is_virtual_start(v, g)) {
            continue;
        }

        processCyclicStateForward(g, v, info, region_map, deadRegions);
        processCyclicStateReverse(g, v, info, region_map, deadRegions);
    }

    if (deadRegions.empty()) {
        return;
    }

    vector<NFAVertex> dead;

    for (auto v : vertices_range(g)) {
        if (is_special(v, g)) {
            continue;
        }
        u32 region = region_map.at(v);
        if (contains(deadRegions, region)) {
            dead.push_back(v);
        }
    }

    if (!dead.empty()) {
        DEBUG_PRINTF("removing %zu vertices from %zu dead regions\n",
                     dead.size(), deadRegions.size());
        remove_vertices(dead, g);
    }
}

} // namespace ue2
