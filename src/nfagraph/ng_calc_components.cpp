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

/** \file
 * \brief Splits an NFA graph into its connected components.
 *
 * This pass takes a NGHolder and splits its graph into a set of connected
 * components, returning them as individual NGHolder graphs. For example, the
 * graph for the regex /foo.*bar|[a-z]{7,13}|hatstand|teakettle$/ will be split
 * into four NGHolders, representing these four components:
 *
 * - /foo.*bar/
 * - /[a-z]{7,13}/
 * - /hatstand/
 * - /teakettle$/
 *
 * The pass operates by creating an undirected graph from the input graph, and
 * then using the BGL's connected_components algorithm to do the work, cloning
 * the identified components into their own graphs. A "shell" of vertices
 * is identified and removed first from the head and tail of the graph, in
 * order to handle cases where there is a common head/tail region.
 *
 * Trivial cases, such as an alternation of single vertices like /a|b|c|d|e|f/,
 * are not split, as later optimisations will handle these cases efficiently.
 */
#include "ng_calc_components.h"

#include "ng_depth.h"
#include "ng_holder.h"
#include "ng_prune.h"
#include "ng_util.h"
#include "grey.h"
#include "ue2common.h"
#include "util/graph_range.h"
#include "util/graph_undirected.h"
#include "util/make_unique.h"

#include <map>
#include <vector>

#include <boost/graph/connected_components.hpp>
#include <boost/graph/filtered_graph.hpp>

using namespace std;

namespace ue2 {

static constexpr u32 MAX_HEAD_SHELL_DEPTH = 3;
static constexpr u32 MAX_TAIL_SHELL_DEPTH = 3;

/**
 * \brief Returns true if the whole graph is just an alternation of character
 * classes.
 */
bool isAlternationOfClasses(const NGHolder &g) {
    for (auto v : vertices_range(g)) {
        if (is_special(v, g)) {
            continue;
        }
        // Vertex must have in edges from starts only.
        for (auto u : inv_adjacent_vertices_range(v, g)) {
            if (!is_any_start(u, g)) {
                return false;
            }
        }
        // Vertex must have out edges to accepts only.
        for (auto w : adjacent_vertices_range(v, g)) {
            if (!is_any_accept(w, g)) {
                return false;
            }
        }
    }

    DEBUG_PRINTF("alternation of single states, treating as one comp\n");
    return true;
}

/**
 * \brief Compute initial max distance to v from start (i.e. ignoring its own
 * self-loop).
 */
static
depth max_dist_from_start(const NGHolder &g,
                          const vector<NFAVertexBidiDepth> &depths,
                          NFAVertex v) {
    depth max_depth(0);
    for (const auto u : inv_adjacent_vertices_range(v, g)) {
        if (u == v) {
            continue;
        }
        const auto &d = depths.at(g[u].index);
        if (d.fromStart.max.is_reachable()) {
            max_depth = max(max_depth, d.fromStart.max);
        }
        if (d.fromStartDotStar.max.is_reachable()) {
            max_depth = max(max_depth, d.fromStartDotStar.max);
        }
    }
    return max_depth + 1;
}

/**
 * \brief Compute initial max depth from v from accept (i.e. ignoring its own
 * self-loop).
 */
static
depth max_dist_to_accept(const NGHolder &g,
                         const vector<NFAVertexBidiDepth> &depths,
                         NFAVertex v) {
    depth max_depth(0);
    for (const auto w : adjacent_vertices_range(v, g)) {
        if (w == v) {
            continue;
        }
        const auto &d = depths.at(g[w].index);
        if (d.toAccept.max.is_reachable()) {
            max_depth = max(max_depth, d.toAccept.max);
        }
        if (d.toAcceptEod.max.is_reachable()) {
            max_depth = max(max_depth, d.toAcceptEod.max);
        }
    }
    return max_depth + 1;
}

static
flat_set<NFAVertex> findHeadShell(const NGHolder &g,
                                  const vector<NFAVertexBidiDepth> &depths,
                                  const depth &max_dist) {
    flat_set<NFAVertex> shell;

    for (auto v : vertices_range(g)) {
        if (is_special(v, g)) {
            continue;
        }
        if (max_dist_from_start(g, depths, v) <= max_dist) {
            shell.insert(v);
        }
    }

    for (UNUSED auto v : shell) {
        DEBUG_PRINTF("shell: %zu\n", g[v].index);
    }

    return shell;
}

static
flat_set<NFAVertex> findTailShell(const NGHolder &g,
                                  const vector<NFAVertexBidiDepth> &depths,
                                  const depth &max_dist) {
    flat_set<NFAVertex> shell;

    for (auto v : vertices_range(g)) {
        if (is_special(v, g)) {
            continue;
        }
        if (max_dist_to_accept(g, depths, v) <= max_dist) {
            shell.insert(v);
        }
    }

    for (UNUSED auto v : shell) {
        DEBUG_PRINTF("shell: %zu\n", g[v].index);
    }

    return shell;
}

static
vector<NFAEdge> findShellEdges(const NGHolder &g,
                               const flat_set<NFAVertex> &head_shell,
                               const flat_set<NFAVertex> &tail_shell) {
    vector<NFAEdge> shell_edges;

    for (const auto &e : edges_range(g)) {
        auto u = source(e, g);
        auto v = target(e, g);

        if (v == g.startDs && is_any_start(u, g)) {
            continue;
        }
        if (u == g.accept && v == g.acceptEod) {
            continue;
        }

        if ((is_special(u, g) || contains(head_shell, u)) &&
            (is_special(v, g) || contains(tail_shell, v))) {
            DEBUG_PRINTF("edge (%zu,%zu) is a shell edge\n", g[u].index,
                         g[v].index);
            shell_edges.push_back(e);
        }
    }

    return shell_edges;
}

template<typename GetAdjRange>
bool shellHasOnePath(const NGHolder &g, const flat_set<NFAVertex> &shell,
                     GetAdjRange adj_range_func) {
    if (shell.empty()) {
        DEBUG_PRINTF("no shell\n");
        return false;
    }

    NFAVertex exit_vertex = NGHolder::null_vertex();
    for (auto u : shell) {
        for (auto v : adj_range_func(u, g)) {
            if (contains(shell, v)) {
                continue;
            }
            if (!exit_vertex) {
                exit_vertex = v;
                continue;
            }
            if (exit_vertex == v) {
                continue;
            }
            return false;
        }
    }

    return true;
}

/**
 * True if all edges out of vertices in the head shell lead to at most a single
 * outside vertex, or the inverse for the tail shell.
 */
static
bool shellHasOnePath(const NGHolder &g, const flat_set<NFAVertex> &head_shell,
                     const flat_set<NFAVertex> &tail_shell) {
    if (shellHasOnePath(g, head_shell, adjacent_vertices_range<NGHolder>)) {
        DEBUG_PRINTF("head shell has only one path through it\n");
        return true;
    }
    if (shellHasOnePath(g, tail_shell, inv_adjacent_vertices_range<NGHolder>)) {
        DEBUG_PRINTF("tail shell has only one path into it\n");
        return true;
    }
    return false;
}

/**
 * Common code called by calc- and recalc- below. Splits the given holder into
 * one or more connected components, adding them to the comps deque.
 */
static
void splitIntoComponents(unique_ptr<NGHolder> g,
                         deque<unique_ptr<NGHolder>> &comps,
                         const depth &max_head_depth,
                         const depth &max_tail_depth, bool *shell_comp) {
    DEBUG_PRINTF("graph has %zu vertices\n", num_vertices(*g));

    assert(shell_comp);
    *shell_comp = false;

    // Compute "shell" head and tail subgraphs.
    auto depths = calcBidiDepths(*g);
    auto head_shell = findHeadShell(*g, depths, max_head_depth);
    auto tail_shell = findTailShell(*g, depths, max_tail_depth);
    for (auto v : head_shell) {
        tail_shell.erase(v);
    }

    if (head_shell.size() + tail_shell.size() + N_SPECIALS >=
        num_vertices(*g)) {
        DEBUG_PRINTF("all in shell component\n");
        comps.push_back(std::move(g));
        *shell_comp = true;
        return;
    }

    // Find edges connecting the head and tail shells directly.
    vector<NFAEdge> shell_edges = findShellEdges(*g, head_shell, tail_shell);

    DEBUG_PRINTF("%zu vertices in head, %zu in tail, %zu shell edges\n",
                 head_shell.size(), tail_shell.size(), shell_edges.size());

    // If there are no shell edges and only one path out of the head shell or
    // into the tail shell, we aren't going to find more than one component.
    if (shell_edges.empty() && shellHasOnePath(*g, head_shell, tail_shell)) {
        DEBUG_PRINTF("single component\n");
        comps.push_back(std::move(g));
        return;
    }

    auto ug = make_undirected_graph(*g);

    // Filter specials and shell vertices from undirected graph.
    unordered_set<NFAVertex> bad_vertices(
        {g->start, g->startDs, g->accept, g->acceptEod});
    bad_vertices.insert(head_shell.begin(), head_shell.end());
    bad_vertices.insert(tail_shell.begin(), tail_shell.end());

    auto filtered_ug = boost::make_filtered_graph(
        ug, boost::keep_all(), make_bad_vertex_filter(&bad_vertices));

    // Actually run the connected components algorithm.
    map<NFAVertex, u32> split_components;
    const u32 num = connected_components(
        filtered_ug, boost::make_assoc_property_map(split_components));

    assert(num > 0);
    if (num == 1 && shell_edges.empty()) {
        DEBUG_PRINTF("single component\n");
        comps.push_back(std::move(g));
        return;
    }

    DEBUG_PRINTF("broke graph into %u components\n", num);

    vector<deque<NFAVertex>> verts(num);

    // Collect vertex lists per component.
    for (const auto &m : split_components) {
        NFAVertex v = m.first;
        u32 c = m.second;
        verts[c].push_back(v);
        DEBUG_PRINTF("vertex %zu is in comp %u\n", (*g)[v].index, c);
    }

    unordered_map<NFAVertex, NFAVertex> v_map; // temp map for fillHolder
    for (auto &vv : verts) {
        // Shells are in every component.
        vv.insert(vv.end(), begin(head_shell), end(head_shell));
        vv.insert(vv.end(), begin(tail_shell), end(tail_shell));

        /* Sort for determinism. Still required as NFAUndirectedVertex have
         * no deterministic ordering (split_components map). */
        sort(begin(vv), end(vv));

        auto gc = ue2::make_unique<NGHolder>();
        v_map.clear();
        fillHolder(gc.get(), *g, vv, &v_map);

        // Remove shell edges, which will get their own component.
        for (const auto &e : shell_edges) {
            auto cu = v_map.at(source(e, *g));
            auto cv = v_map.at(target(e, *g));
            assert(edge(cu, cv, *gc).second);
            remove_edge(cu, cv, *gc);
        }

        pruneUseless(*gc);
        DEBUG_PRINTF("component %zu has %zu vertices\n", comps.size(),
                     num_vertices(*gc));
        comps.push_back(move(gc));
    }

    // Another component to handle the direct shell-to-shell edges.
    if (!shell_edges.empty()) {
        deque<NFAVertex> vv;
        vv.insert(vv.end(), begin(head_shell), end(head_shell));
        vv.insert(vv.end(), begin(tail_shell), end(tail_shell));

        auto gc = ue2::make_unique<NGHolder>();
        v_map.clear();
        fillHolder(gc.get(), *g, vv, &v_map);

        pruneUseless(*gc);
        DEBUG_PRINTF("shell edge component %zu has %zu vertices\n",
                     comps.size(), num_vertices(*gc));
        comps.push_back(move(gc));
        *shell_comp = true;
    }

    // Ensure that only vertices with accept edges have reports.
    for (auto &gc : comps) {
        assert(gc);
        clearReports(*gc);
    }

    // We should never produce empty component graphs.
    assert(all_of(begin(comps), end(comps),
                  [](const unique_ptr<NGHolder> &g_comp) {
                      return num_vertices(*g_comp) > N_SPECIALS;
                  }));
}

deque<unique_ptr<NGHolder>> calcComponents(unique_ptr<NGHolder> g,
                                           const Grey &grey) {
    deque<unique_ptr<NGHolder>> comps;

    // For trivial cases, we needn't bother running the full
    // connected_components algorithm.
    if (!grey.calcComponents || isAlternationOfClasses(*g)) {
        comps.push_back(std::move(g));
        return comps;
    }

    bool shell_comp = false;
    splitIntoComponents(std::move(g), comps, depth(MAX_HEAD_SHELL_DEPTH),
                        depth(MAX_TAIL_SHELL_DEPTH), &shell_comp);

    if (shell_comp) {
        DEBUG_PRINTF("re-running on shell comp\n");
        assert(!comps.empty());
        auto sc = std::move(comps.back());
        comps.pop_back();
        splitIntoComponents(std::move(sc), comps, depth(0), depth(0),
                            &shell_comp);
    }

    DEBUG_PRINTF("finished; split into %zu components\n", comps.size());
    return comps;
}

void recalcComponents(deque<unique_ptr<NGHolder>> &comps, const Grey &grey) {
    if (!grey.calcComponents) {
        return;
    }

    deque<unique_ptr<NGHolder>> out;

    for (auto &gc : comps) {
        if (!gc) {
            continue; // graph has been consumed already.
        }

        if (isAlternationOfClasses(*gc)) {
            out.push_back(std::move(gc));
            continue;
        }

        auto gc_comps = calcComponents(std::move(gc), grey);
        out.insert(end(out), std::make_move_iterator(begin(gc_comps)),
                   std::make_move_iterator(end(gc_comps)));
    }

    // Replace comps with our recalculated list.
    comps.swap(out);
}

} // namespace ue2
