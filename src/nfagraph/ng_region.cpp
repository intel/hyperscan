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
 * \brief Region analysis.
 *
 * Definition: a \a region is a subset of vertices in a graph such that:
 * - the edges entering the region are a cutset of the graph
 * - for every in-edge (u, v) to the region there exist edges (u, w) for all
 *       w in {w : w in region and w has an in-edge}
 * - the regions in a graph partition the graph
 *
 * Note:
 * - we partition a graph into the maximal number of regions
 * - similar properties for exit edges should hold as a consequence
 * - graph == sequence of regions
 * - a region is considered to have an epsilon vertex to allow jumps
 * - vertices which only lead to back edges need to be floated up in the topo
 *   order
 *
 * Algorithm overview:
 * -# topo-order over the DAG skeleton;
 * -# incrementally add vertices to the current region until the boundary edges
 *    form a valid cut-set;
 * -# for each back-edge, if the source and target are in different regions,
 *    merge the regions (and all intervening regions) into a common region.
 */
#include "ng_region.h"

#include "ng_holder.h"
#include "ng_util.h"
#include "ue2common.h"
#include "util/container.h"
#include "util/flat_containers.h"
#include "util/graph_range.h"
#include "util/graph_small_color_map.h"

#include <set>
#include <utility>
#include <vector>

#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/topological_sort.hpp>

using namespace std;

namespace ue2 {

using BackEdgeSet = unordered_set<NFAEdge>;
using AcyclicGraph =
    boost::filtered_graph<NGHolder, bad_edge_filter<BackEdgeSet>>;

namespace {
struct exit_info {
    explicit exit_info(NFAVertex v) : exit(v) {}

    NFAVertex exit;
    flat_set<NFAVertex> open;
};
}

static
void checkAndAddExitCandidate(const AcyclicGraph &g,
                              const unordered_set<NFAVertex> &r, NFAVertex v,
                              vector<exit_info> &exits) {
    exit_info v_exit(v);
    auto &open = v_exit.open;

    /* find the set of vertices reachable from v which are not in r */
    for (auto w : adjacent_vertices_range(v, g)) {
        if (!contains(r, w)) {
            open.insert(w);
        }
    }

    if (!open.empty()) {
        DEBUG_PRINTF("exit %zu\n", g[v].index);
        exits.push_back(move(v_exit));
    }
}

static
void findExits(const AcyclicGraph &g, const unordered_set<NFAVertex> &r,
               vector<exit_info> &exits) {
    exits.clear();
    for (auto v : r) {
        checkAndAddExitCandidate(g, r, v, exits);
    }
}

static
void refineExits(const AcyclicGraph &g, const unordered_set<NFAVertex> &r,
                 NFAVertex new_v, vector<exit_info> &exits) {
    /* new_v is no long an open edge */
    for (auto &exit : exits) {
        exit.open.erase(new_v);
    }

    /* no open edges: no longer an exit */
    exits.erase(remove_if(exits.begin(), exits.end(),
                  [&](const exit_info &exit) { return exit.open.empty(); }),
                exits.end());

    checkAndAddExitCandidate(g, r, new_v, exits);
}

/** the set of exits from a candidate region are valid if: FIXME: document
 */
static
bool exitValid(UNUSED const AcyclicGraph &g, const vector<exit_info> &exits,
               const flat_set<NFAVertex> &open_jumps) {
    if (exits.empty() || (exits.size() < 2 && open_jumps.empty())) {
        return true;
    }
    if (exits.size() == 1 && open_jumps.size() == 1) {
        DEBUG_PRINTF("oj %zu, e %zu\n", g[*open_jumps.begin()].index,
                     g[exits[0].exit].index);
        if (*open_jumps.begin() == exits[0].exit) {
            return true;
        }
    }

    assert(!exits.empty());
    const auto &enters = exits.front().open;

    if (!open_jumps.empty() && enters != open_jumps) {
        return false;
    }

    for (auto it = begin(exits) + 1; it != end(exits); ++it) {
        if (it->open != enters) {
            return false;
        }
    }

    return true;
}

static
void setRegion(const unordered_set<NFAVertex> &r, u32 rid,
               unordered_map<NFAVertex, u32> &regions) {
    for (auto v : r) {
        regions[v] = rid;
    }
}

static
void buildInitialCandidate(const AcyclicGraph &g,
                           vector<NFAVertex>::const_reverse_iterator &it,
                           const vector<NFAVertex>::const_reverse_iterator &ite,
                           unordered_set<NFAVertex> &candidate,
                           /* in exits of prev region;
                            * out exits from candidate */
                           vector<exit_info> &exits,
                           flat_set<NFAVertex> &open_jumps) {
    if (it == ite) {
        candidate.clear();
        exits.clear();
        return;
    }

    if (exits.empty()) {
        DEBUG_PRINTF("odd\n");
        candidate.clear();
        DEBUG_PRINTF("adding %zu to initial\n", g[*it].index);
        candidate.insert(*it);
        open_jumps.erase(*it);
        checkAndAddExitCandidate(g, candidate, *it, exits);
        ++it;
        return;
    }

    // Note: findExits() will clear exits, so it's safe to mutate/move its
    // elements here.
    auto &enters = exits.front().open;
    candidate.clear();

    for (; it != ite; ++it) {
        DEBUG_PRINTF("adding %zu to initial\n", g[*it].index);
        candidate.insert(*it);
        if (contains(enters, *it)) {
            break;
        }
    }

    if (it != ite) {
        enters.erase(*it);
        open_jumps = move(enters);
        DEBUG_PRINTF("oj size = %zu\n", open_jumps.size());
        ++it;
    } else {
        open_jumps.clear();
    }

    findExits(g, candidate, exits);
}

static
void findDagLeaders(const NGHolder &h, const AcyclicGraph &g,
                    const vector<NFAVertex> &topo,
                    unordered_map<NFAVertex, u32> &regions) {
    assert(!topo.empty());
    u32 curr_id = 0;
    auto t_it = topo.rbegin();
    unordered_set<NFAVertex> candidate;
    flat_set<NFAVertex> open_jumps;
    DEBUG_PRINTF("adding %zu to current\n", g[*t_it].index);
    assert(t_it != topo.rend());
    candidate.insert(*t_it++);
    DEBUG_PRINTF("adding %zu to current\n", g[*t_it].index);
    assert(t_it != topo.rend());
    candidate.insert(*t_it++);

    vector<exit_info> exits;
    findExits(g, candidate, exits);

    while (t_it != topo.rend()) {
        assert(!candidate.empty());

        if (exitValid(g, exits, open_jumps)) {
            if (contains(candidate, h.accept) && !open_jumps.empty()) {
                /* we have tried to make an optional region containing accept as
                 * we have an open jump to eod. This candidate region needs to
                 * be put in with the previous region. */
                curr_id--;
                DEBUG_PRINTF("merging in with region %u\n", curr_id);
            } else {
                DEBUG_PRINTF("setting region %u\n", curr_id);
            }
            setRegion(candidate, curr_id++, regions);
            buildInitialCandidate(g, t_it, topo.rend(), candidate, exits,
                                  open_jumps);
        } else {
            NFAVertex curr = *t_it;
            DEBUG_PRINTF("adding %zu to current\n", g[curr].index);
            candidate.insert(curr);
            open_jumps.erase(curr);
            refineExits(g, candidate, *t_it, exits);
            DEBUG_PRINTF("    open jumps %zu exits %zu\n", open_jumps.size(),
                         exits.size());
            ++t_it;
        }
    }
    /* assert exits valid */
    setRegion(candidate, curr_id, regions);
}

static
void mergeUnderBackEdges(const NGHolder &g, const vector<NFAVertex> &topo,
                         const BackEdgeSet &backEdges,
                         unordered_map<NFAVertex, u32> &regions) {
    for (const auto &e : backEdges) {
        NFAVertex u = source(e, g);
        NFAVertex v = target(e, g);

        u32 ru = regions[u];
        u32 rv = regions[v];
        if (ru == rv) {
            continue;
        }

        DEBUG_PRINTF("merging v = %zu(%u), u = %zu(%u)\n", g[v].index, rv,
                     g[u].index, ru);
        assert(rv < ru);

        for (auto t : topo) {
            u32 r = regions[t];
            if (r <= ru && r > rv) {
                regions[t] = rv;
            } else if (r > ru) {
                regions[t] = rv + r - ru;
            }
        }
    }
}

static
void reorderSpecials(const NGHolder &w, const AcyclicGraph &acyclic_g,
                     vector<NFAVertex> &topoOrder) {
    // Start is last element of reverse topo ordering.
    auto it = find(topoOrder.begin(), topoOrder.end(), w.start);
    if (it != topoOrder.end() - 1) {
        DEBUG_PRINTF("repositioning start\n");
        assert(it != topoOrder.end());
        topoOrder.erase(it);
        topoOrder.insert(topoOrder.end(), w.start);
    }

    // StartDs is second-to-last element of reverse topo ordering.
    it = find(topoOrder.begin(), topoOrder.end(), w.startDs);
    if (it != topoOrder.end() - 2) {
        DEBUG_PRINTF("repositioning start ds\n");
        assert(it != topoOrder.end());
        topoOrder.erase(it);
        topoOrder.insert(topoOrder.end() - 1, w.startDs);
    }

    // AcceptEOD is first element of reverse topo ordering.
    it = find(topoOrder.begin(), topoOrder.end(), w.acceptEod);
    if (it != topoOrder.begin()) {
        DEBUG_PRINTF("repositioning accept\n");
        assert(it != topoOrder.end());
        topoOrder.erase(it);
        topoOrder.insert(topoOrder.begin(), w.acceptEod);
    }

    // Accept is second element of reverse topo ordering, if it's connected.
    it = find(topoOrder.begin(), topoOrder.end(), w.accept);
    if (it != topoOrder.begin() + 1) {
        DEBUG_PRINTF("repositioning accept\n");
        assert(it != topoOrder.end());
        topoOrder.erase(it);
        if (in_degree(w.accept, acyclic_g) != 0) {
            topoOrder.insert(topoOrder.begin() + 1, w.accept);
        }
    }
}

static
void liftSinks(const AcyclicGraph &acyclic_g, vector<NFAVertex> &topoOrder) {
    unordered_set<NFAVertex> sinks;
    for (auto v : vertices_range(acyclic_g)) {
        if (is_special(v, acyclic_g)) {
            continue;
        }

        if (isLeafNode(v, acyclic_g)) {
            DEBUG_PRINTF("sink found %zu\n", acyclic_g[v].index);
            sinks.insert(NFAVertex(v));
        }
    }

    if (sinks.empty()) {
        DEBUG_PRINTF("no sinks found\n");
        return;
    }

    bool changed;
    do {
        DEBUG_PRINTF("look\n");
        changed = false;
        for (auto v : vertices_range(acyclic_g)) {
            if (is_special(v, acyclic_g) || contains(sinks, NFAVertex(v))) {
                continue;
            }

            for (auto w : adjacent_vertices_range(v, acyclic_g)) {
                if (!contains(sinks, NFAVertex(w))) {
                    goto next;
                }
            }

            DEBUG_PRINTF("sink found %zu\n", acyclic_g[v].index);
            sinks.insert(NFAVertex(v));
            changed = true;
        next:;
        }
    } while (changed);

    for (auto ri = topoOrder.rbegin() + 1; ri != topoOrder.rend(); ++ri) {
        if (!contains(sinks, *ri)) {
            continue;
        }
        NFAVertex s = *ri;
        DEBUG_PRINTF("handling sink %zu\n", acyclic_g[s].index);
        unordered_set<NFAVertex> parents;
        for (const auto &e : in_edges_range(s, acyclic_g)) {
            parents.insert(NFAVertex(source(e, acyclic_g)));
        }

        /* vertex has no children not reachable on a back edge, bubble the
         * vertex up the topo order to be near its parents */
        vector<NFAVertex>::reverse_iterator rj = ri;
        --rj;
        while (rj != topoOrder.rbegin() && !contains(parents, *rj)) {
            /* sink is in rj + 1 */
            assert(*(rj + 1) == s);
            DEBUG_PRINTF("lifting\n");
            using std::swap;
            swap(*rj, *(rj + 1));
            --rj;
        }
    }
}

using ColorMap = decltype(make_small_color_map(NGHolder()));

/** Build a reverse topo ordering (with only the specials that are in use). We
 * also want to ensure vertices which only lead to back edges are placed near
 * their parents. */
static
vector<NFAVertex> buildTopoOrder(const NGHolder &w,
                                 const AcyclicGraph &acyclic_g,
                                 ColorMap &colours) {
    vector<NFAVertex> topoOrder;
    topoOrder.reserve(num_vertices(w));

    topological_sort(acyclic_g, back_inserter(topoOrder),
                     color_map(colours));

    reorderSpecials(w, acyclic_g, topoOrder);

    if (topoOrder.empty()) {
        return topoOrder;
    }

    liftSinks(acyclic_g, topoOrder);

    DEBUG_PRINTF("TOPO ORDER\n");
    for (auto ri = topoOrder.rbegin(); ri != topoOrder.rend(); ++ri) {
        DEBUG_PRINTF("[%zu]\n", acyclic_g[*ri].index);
    }
    DEBUG_PRINTF("----------\n");

    return topoOrder;
}

unordered_map<NFAVertex, u32> assignRegions(const NGHolder &g) {
    assert(hasCorrectlyNumberedVertices(g));
    const u32 numVertices = num_vertices(g);
    DEBUG_PRINTF("assigning regions for %u vertices in holder\n", numVertices);

    auto colours = make_small_color_map(g);

    // Build an acyclic graph for this NGHolder.
    BackEdgeSet deadEdges;
    depth_first_search(g,
                       visitor(BackEdges<BackEdgeSet>(deadEdges))
                       .root_vertex(g.start)
                       .color_map(colours));

    auto af = make_bad_edge_filter(&deadEdges);
    AcyclicGraph acyclic_g(g, af);

    // Build a (reverse) topological ordering.
    vector<NFAVertex> topoOrder = buildTopoOrder(g, acyclic_g, colours);

    // Everybody starts in region 0.
    unordered_map<NFAVertex, u32> regions;
    regions.reserve(numVertices);
    for (auto v : vertices_range(g)) {
        regions.emplace(v, 0);
    }

    findDagLeaders(g, acyclic_g, topoOrder, regions);
    mergeUnderBackEdges(g, topoOrder, deadEdges, regions);

    return regions;
}

} // namespace ue2
