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

/**
 * \file
 * \brief Prefilter Reductions.
 *
 * This file contains routines for reducing the size of an NFA graph that we
 * know will be used as a prefilter.
 *
 * The approach used is to consider the graph as a chain of region subgraphs,
 * and to reduce the size of the graph by replacing regions with constructs
 * that can be implemented in fewer states.
 *
 * Right now, the approach used is to replace a region with a bounded repeat of
 * vertices (with bounds derived from the min/max width of the region
 * subgraph). These vertices are given the union of the region's character
 * reachability.
 *
 * For regions with bounded max width, this strategy is quite dependent on the
 * LimEx NFA's bounded repeat functionality.
 */
#include "ng_prefilter.h"

#include "ng_holder.h"
#include "ng_region.h"
#include "ng_util.h"
#include "ng_width.h"
#include "ue2common.h"
#include "util/compile_context.h"
#include "util/container.h"
#include "util/dump_charclass.h"
#include "util/graph_range.h"

#include <queue>
#include <unordered_map>
#include <unordered_set>

#include <boost/range/adaptor/map.hpp>

using namespace std;
using boost::adaptors::map_values;

namespace ue2 {

/** Keep attempting to reduce the size of the graph until the number of
 * vertices falls below this value. */
static const size_t MAX_COMPONENT_VERTICES = 128;

/** Only replace a region with at least this many vertices. */
static const size_t MIN_REPLACE_VERTICES = 2;

/** Estimate of how many vertices are required to represent a bounded repeat in
 * the implementation NFA. */
static const size_t BOUNDED_REPEAT_COUNT = 4;

/** Scoring penalty for boundary regions. */
static const size_t PENALTY_BOUNDARY = 32;

/** Regions with max bounds greater than this value will have their max bound
 * replaced with inf. */
static const size_t MAX_REPLACE_BOUND = 10000;

namespace {

/** Information describing a region. */
struct RegionInfo {
    explicit RegionInfo(u32 id_in) : id(id_in) {}
    u32 id;                             //!< region id
    deque<NFAVertex> vertices;          //!< vertices in the region
    CharReach reach;                    //!< union of region reach
    depth minWidth{0};                  //!< min width of region subgraph
    depth maxWidth{depth::infinity()};  //!< max width of region subgraph
    bool atBoundary = false;            //!< region is next to an accept

    // Bigger score is better.
    size_t score() const {
        // TODO: charreach should be a signal?
        size_t numVertices = vertices.size();
        if (atBoundary) {
            return numVertices - min(PENALTY_BOUNDARY, numVertices);
        } else {
            return numVertices;
        }
    }
};

/** Comparator used to order regions for consideration in a priority queue. */
struct RegionInfoQueueComp {
    bool operator()(const RegionInfo &r1, const RegionInfo &r2) const {
        size_t score1 = r1.score(), score2 = r2.score();
        if (score1 != score2) {
            return score1 < score2;
        }
        if (r1.reach.count() != r2.reach.count()) {
            return r1.reach.count() < r2.reach.count();
        }
        return r1.id < r2.id;
    }
};

} // namespace

static
void findWidths(const NGHolder &g,
                const unordered_map<NFAVertex, u32> &region_map,
                RegionInfo &ri) {
    NGHolder rg;
    unordered_map<NFAVertex, NFAVertex> mapping;
    fillHolder(&rg, g, ri.vertices, &mapping);

    // Wire our entries to start and our exits to accept.
    for (auto v : ri.vertices) {
        NFAVertex v_new = mapping[v];
        assert(v_new != NGHolder::null_vertex());

        if (isRegionEntry(g, v, region_map) &&
            !edge(rg.start, v_new, rg).second) {
            add_edge(rg.start, v_new, rg);
        }
        if (isRegionExit(g, v, region_map) &&
            !edge(v_new, rg.accept, rg).second) {
            add_edge(v_new, rg.accept, rg);
        }
    }

    ri.minWidth = findMinWidth(rg);
    ri.maxWidth = findMaxWidth(rg);
}

// acc can be either h.accept or h.acceptEod.
static
void markBoundaryRegions(const NGHolder &h,
                         const unordered_map<NFAVertex, u32> &region_map,
                         map<u32, RegionInfo> &regions, NFAVertex acc) {
    for (auto v : inv_adjacent_vertices_range(acc, h)) {
        if (is_special(v, h)) {
            continue;
        }
        u32 id = region_map.at(v);

        auto ri = regions.find(id);
        if (ri == regions.end()) {
            continue; // Not tracking this region as it's too small.
        }

        ri->second.atBoundary = true;
    }
}

static
map<u32, RegionInfo> findRegionInfo(const NGHolder &h,
               const unordered_map<NFAVertex, u32> &region_map) {
    map<u32, RegionInfo> regions;
    for (auto v : vertices_range(h)) {
        if (is_special(v, h)) {
            continue;
        }
        u32 id = region_map.at(v);
        RegionInfo &ri = regions.emplace(id, RegionInfo(id)).first->second;
        ri.vertices.push_back(v);
        ri.reach |= h[v].char_reach;
    }

    // There's no point tracking more information about regions that we won't
    // consider replacing, so we remove them from the region map.
    for (auto it = regions.begin(); it != regions.end();) {
        if (it->second.vertices.size() < MIN_REPLACE_VERTICES) {
            regions.erase(it++);
        } else {
            ++it;
        }
    }

    DEBUG_PRINTF("%zu regions\n", regions.size());

    markBoundaryRegions(h, region_map,  regions, h.accept);
    markBoundaryRegions(h, region_map, regions, h.acceptEod);

    // Determine min/max widths.
    for (RegionInfo &ri : regions | map_values) {
        findWidths(h, region_map, ri);
        DEBUG_PRINTF("region %u %shas widths [%s,%s]\n", ri.id,
                     ri.atBoundary ? "(boundary) " : "",
                     ri.minWidth.str().c_str(), ri.maxWidth.str().c_str());
    }

    return regions;
}

static
void copyInEdges(NGHolder &g, NFAVertex from, NFAVertex to) {
    for (const auto &e : in_edges_range(from, g)) {
        NFAVertex u = source(e, g);
        add_edge_if_not_present(u, to, g[e], g);
    }
}

static
void copyOutEdges(NGHolder &g, NFAVertex from, NFAVertex to) {
    for (const auto &e : out_edges_range(from, g)) {
        NFAVertex t = target(e, g);
        add_edge_if_not_present(to, t, g[e], g);

        if (is_any_accept(t, g)) {
            const auto &reports = g[from].reports;
            g[to].reports.insert(reports.begin(), reports.end());
        }
    }
}

static
void removeInteriorEdges(NGHolder &g, const RegionInfo &ri) {
    // Set of vertices in region, for quick lookups.
    const unordered_set<NFAVertex> rverts(ri.vertices.begin(),
                                          ri.vertices.end());

    auto is_interior_in_edge = [&](const NFAEdge &e) {
        return contains(rverts, source(e, g));
    };

    for (auto v : ri.vertices) {
        remove_in_edge_if(v, is_interior_in_edge, g);
    }
}

static
void replaceRegion(NGHolder &g, const RegionInfo &ri,
                   size_t *verticesAdded, size_t *verticesRemoved) {
    // TODO: more complex replacements.
    assert(ri.vertices.size() >= MIN_REPLACE_VERTICES);
    assert(ri.minWidth.is_finite());

    depth minWidth = ri.minWidth;
    depth maxWidth = ri.maxWidth;

    if (maxWidth > depth(MAX_REPLACE_BOUND)) {
        DEBUG_PRINTF("using inf instead of large bound %s\n",
                     maxWidth.str().c_str());
        maxWidth = depth::infinity();
    }

    size_t replacementSize;
    if (minWidth == maxWidth || maxWidth.is_infinite()) {
        replacementSize = minWidth; // {N} or {N,}
    } else {
        replacementSize = maxWidth; // {N,M} case
    }

    DEBUG_PRINTF("orig size %zu, replace size %zu\n", ri.vertices.size(),
                 replacementSize);

    vector<NFAVertex> verts;
    verts.reserve(replacementSize);
    for (size_t i = 0; i < replacementSize; i++) {
        NFAVertex v = add_vertex(g);
        g[v].char_reach = ri.reach;
        if (i > 0) {
            add_edge(verts.back(), v, g);
        }
        verts.push_back(v);
    }

    if (maxWidth.is_infinite()) {
        add_edge(verts.back(), verts.back(), g);
    }

    removeInteriorEdges(g, ri);

    for (size_t i = 0; i < replacementSize; i++) {
        NFAVertex v_new = verts[i];

        for (auto v_old : ri.vertices) {
            if (i == 0) {
                copyInEdges(g, v_old, v_new);
            }
            if (i + 1 >= ri.minWidth) {
                copyOutEdges(g, v_old, v_new);
            }
        }
    }

    remove_vertices(ri.vertices, g, false);

    *verticesAdded = verts.size();
    *verticesRemoved = ri.vertices.size();
}

namespace {
struct SourceHasEdgeToAccept {
    explicit SourceHasEdgeToAccept(const NGHolder &g_in) : g(g_in) {}
    bool operator()(const NFAEdge &e) const {
        return edge(source(e, g), g.accept, g).second;
    }
    const NGHolder &g;
};
}

static
void reduceRegions(NGHolder &h) {
    map<u32, RegionInfo> regions = findRegionInfo(h, assignRegions(h));

    RegionInfoQueueComp cmp;
    priority_queue<RegionInfo, deque<RegionInfo>, RegionInfoQueueComp> pq(cmp);

    size_t numVertices = 0;
    for (const RegionInfo &ri : regions | map_values) {
        numVertices += ri.vertices.size();
        pq.push(ri);
    }

    while (numVertices > MAX_COMPONENT_VERTICES && !pq.empty()) {
        const RegionInfo &ri = pq.top();
        DEBUG_PRINTF("region %u: vertices=%zu reach=%s score=%zu, "
                     "widths=[%s,%s]\n",
                     ri.id, ri.vertices.size(), describeClass(ri.reach).c_str(),
                     ri.score(), ri.minWidth.str().c_str(),
                     ri.maxWidth.str().c_str());

        size_t verticesAdded = 0;
        size_t verticesRemoved = 0;
        replaceRegion(h, ri, &verticesAdded, &verticesRemoved);
        DEBUG_PRINTF("%zu vertices removed, %zu vertices added\n",
                     verticesRemoved, verticesAdded);

        // We are trusting that implementation NFAs will be able to use the
        // LimEx bounded repeat code here.
        numVertices -= verticesRemoved;
        numVertices += BOUNDED_REPEAT_COUNT;

        DEBUG_PRINTF("numVertices is now %zu\n", numVertices);
        pq.pop();
    }

    // We may have vertices that have edges to both accept and acceptEod: in
    // this case, we can optimize for performance by removing the acceptEod
    // edges.
    remove_in_edge_if(h.acceptEod, SourceHasEdgeToAccept(h), h);
}

void prefilterReductions(NGHolder &h, const CompileContext &cc) {
    if (!cc.grey.prefilterReductions) {
        return;
    }

    if (num_vertices(h) <= MAX_COMPONENT_VERTICES) {
        DEBUG_PRINTF("graph is already small enough (%zu vertices)\n",
                     num_vertices(h));
        return;
    }

    DEBUG_PRINTF("before: graph with %zu vertices, %zu edges\n",
                 num_vertices(h), num_edges(h));

    renumber_vertices(h);
    renumber_edges(h);

    reduceRegions(h);

    renumber_vertices(h);
    renumber_edges(h);

    DEBUG_PRINTF("after: graph with %zu vertices, %zu edges\n",
                 num_vertices(h), num_edges(h));

}

} // namespace ue2
