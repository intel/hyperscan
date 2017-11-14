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
 * \brief Miscellaneous NFA graph utilities.
 */
#include "ng_util.h"

#include "grey.h"
#include "ng_dump.h"
#include "ng_prune.h"
#include "ue2common.h"
#include "nfa/limex_limits.h" // for NFA_MAX_TOP_MASKS.
#include "parser/position.h"
#include "util/graph_range.h"
#include "util/graph_small_color_map.h"
#include "util/make_unique.h"
#include "util/order_check.h"
#include "util/ue2string.h"
#include "util/report_manager.h"

#include <limits>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>

#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/range/adaptor/map.hpp>

using namespace std;
using boost::make_filtered_graph;
using boost::make_assoc_property_map;

namespace ue2 {

NFAVertex getSoleDestVertex(const NGHolder &g, NFAVertex a) {
    assert(a != NGHolder::null_vertex());

    NGHolder::out_edge_iterator ii, iie;
    tie(ii, iie) = out_edges(a, g);
    if (ii == iie) {
        return NGHolder::null_vertex();
    }
    NFAVertex b = target(*ii, g);
    if (a == b) {
        ++ii;
        if (ii == iie) {
            return NGHolder::null_vertex();
        }

        b = target(*ii, g);
        if (++ii != iie) {
            return NGHolder::null_vertex();
        }
    } else if (++ii != iie && (target(*ii, g) != a || ++ii != iie)) {
        return NGHolder::null_vertex();
    }

    assert(a != b);
    return b;
}

NFAVertex getSoleSourceVertex(const NGHolder &g, NFAVertex a) {
    assert(a != NGHolder::null_vertex());

    u32 idegree = in_degree(a, g);
    if (idegree != 1 && !(idegree == 2 && hasSelfLoop(a, g))) {
        return NGHolder::null_vertex();
    }

    NGHolder::in_edge_iterator ii, iie;
    tie(ii, iie) = in_edges(a, g);
    if (ii == iie) {
        return NGHolder::null_vertex();
    }
    NFAVertex b = source(*ii, g);
    if (a == b) {
        ++ii;
        if (ii == iie) {
            return NGHolder::null_vertex();
        }

        b = source(*ii, g);
    }

    assert(a != b);
    return b;
}

NFAVertex clone_vertex(NGHolder &g, NFAVertex v) {
    NFAVertex clone = add_vertex(g);
    u32 idx = g[clone].index;
    g[clone] = g[v];
    g[clone].index = idx;

    return clone;
}

void clone_out_edges(NGHolder &g, NFAVertex source, NFAVertex dest) {
    for (const auto &e : out_edges_range(source, g)) {
        NFAVertex t = target(e, g);
        if (edge(dest, t, g).second) {
            continue;
        }
        NFAEdge clone = add_edge(dest, t, g);
        u32 idx = g[clone].index;
        g[clone] = g[e];
        g[clone].index = idx;
    }
}

void clone_in_edges(NGHolder &g, NFAVertex s, NFAVertex dest) {
    for (const auto &e : in_edges_range(s, g)) {
        NFAVertex ss = source(e, g);
        assert(!edge(ss, dest, g).second);
        NFAEdge clone = add_edge(ss, dest, g);
        u32 idx = g[clone].index;
        g[clone] = g[e];
        g[clone].index = idx;
    }
}

bool onlyOneTop(const NGHolder &g) {
    return getTops(g).size() == 1;
}

namespace {
struct CycleFound {};
struct DetectCycles : public boost::default_dfs_visitor {
    explicit DetectCycles(const NGHolder &g) : startDs(g.startDs) {}
    void back_edge(const NFAEdge &e, const NGHolder &g) const {
        NFAVertex u = source(e, g), v = target(e, g);
        // We ignore the startDs self-loop.
        if (u == startDs && v == startDs) {
            return;
        }
        // Any other back-edge indicates a cycle.
        DEBUG_PRINTF("back edge %zu->%zu found\n", g[u].index, g[v].index);
        throw CycleFound();
    }
private:
    const NFAVertex startDs;
};
} // namespace

bool isVacuous(const NGHolder &h) {
    return edge(h.start, h.accept, h).second
        || edge(h.start, h.acceptEod, h).second
        || edge(h.startDs, h.accept, h).second
        || edge(h.startDs, h.acceptEod, h).second;
}

bool isAnchored(const NGHolder &g) {
    for (auto v : adjacent_vertices_range(g.startDs, g)) {
        if (v != g.startDs) {
            return false;
        }
    }
    return true;
}

bool isFloating(const NGHolder &g) {
    for (auto v : adjacent_vertices_range(g.start, g)) {
        if (v != g.startDs && !edge(g.startDs, v, g).second) {
            return false;
        }
    }
    return true;
}

bool isAcyclic(const NGHolder &g) {
    try {
        boost::depth_first_search(g, DetectCycles(g), make_small_color_map(g),
                                  g.start);
    } catch (const CycleFound &) {
        return false;
    }

    return true;
}

/** True if the graph has a cycle reachable from the given source vertex. */
bool hasReachableCycle(const NGHolder &g, NFAVertex src) {
    assert(hasCorrectlyNumberedVertices(g));

    try {
        // Use depth_first_visit, rather than depth_first_search, so that we
        // only search from src.
        boost::depth_first_visit(g, src, DetectCycles(g),
                                 make_small_color_map(g));
    } catch (const CycleFound &) {
        return true;
    }

    return false;
}

bool hasBigCycles(const NGHolder &g) {
    assert(hasCorrectlyNumberedVertices(g));
    set<NFAEdge> dead;
    BackEdges<set<NFAEdge>> backEdgeVisitor(dead);
    boost::depth_first_search(g, backEdgeVisitor, make_small_color_map(g),
                              g.start);

    for (const auto &e : dead) {
        if (source(e, g) != target(e, g)) {
            return true;
        }
    }

    return false;
}

bool hasNarrowReachVertex(const NGHolder &g, size_t max_reach_count) {
    return any_of_in(vertices_range(g), [&](NFAVertex v) {
        return !is_special(v, g) && g[v].char_reach.count() < max_reach_count;
    });
}

bool can_never_match(const NGHolder &g) {
    assert(edge(g.accept, g.acceptEod, g).second);
    if (in_degree(g.accept, g) == 0 && in_degree(g.acceptEod, g) == 1) {
        DEBUG_PRINTF("no paths into accept\n");
        return true;
    }

    return false;
}

bool can_match_at_eod(const NGHolder &h) {
    if (in_degree(h.acceptEod, h) > 1) {
        DEBUG_PRINTF("more than one edge to acceptEod\n");
        return true;
    }

    for (auto e : in_edges_range(h.accept, h)) {
        if (h[e].assert_flags) {
            DEBUG_PRINTF("edge to accept has assert flags %d\n",
                         h[e].assert_flags);
            return true;
        }
    }

    return false;
}

bool can_only_match_at_eod(const NGHolder &g) {
    NGHolder::in_edge_iterator ie, ee;
    tie(ie, ee) = in_edges(g.accept, g);

    return ie == ee;
}

bool matches_everywhere(const NGHolder &h) {
    NFAEdge e = edge(h.startDs, h.accept, h);

    return e && !h[e].assert_flags;
}

bool is_virtual_start(NFAVertex v, const NGHolder &g) {
    return g[v].assert_flags & POS_FLAG_VIRTUAL_START;
}

static
void reorderSpecials(const NGHolder &g, vector<NFAVertex> &topoOrder) {
    // Start is last element of reverse topo ordering.
    auto it = find(topoOrder.begin(), topoOrder.end(), g.start);
    if (it != topoOrder.end() - 1) {
        DEBUG_PRINTF("repositioning start\n");
        assert(it != topoOrder.end());
        topoOrder.erase(it);
        topoOrder.insert(topoOrder.end(), g.start);
    }

    // StartDs is second-to-last element of reverse topo ordering.
    it = find(topoOrder.begin(), topoOrder.end(), g.startDs);
    if (it != topoOrder.end() - 2) {
        DEBUG_PRINTF("repositioning start ds\n");
        assert(it != topoOrder.end());
        topoOrder.erase(it);
        topoOrder.insert(topoOrder.end() - 1, g.startDs);
    }

    // AcceptEOD is first element of reverse topo ordering.
    it = find(topoOrder.begin(), topoOrder.end(), g.acceptEod);
    if (it != topoOrder.begin()) {
        DEBUG_PRINTF("repositioning accept\n");
        assert(it != topoOrder.end());
        topoOrder.erase(it);
        topoOrder.insert(topoOrder.begin(), g.acceptEod);
    }

    // Accept is second element of reverse topo ordering, if it's connected.
    it = find(topoOrder.begin(), topoOrder.end(), g.accept);
    if (it != topoOrder.begin() + 1) {
        DEBUG_PRINTF("repositioning accept\n");
        assert(it != topoOrder.end());
        topoOrder.erase(it);
        if (in_degree(g.accept, g) != 0) {
            topoOrder.insert(topoOrder.begin() + 1, g.accept);
        }
    }
}

vector<NFAVertex> getTopoOrdering(const NGHolder &g) {
    assert(hasCorrectlyNumberedVertices(g));

    // Use the same colour map for both DFS and topological_sort below: avoids
    // having to reallocate it, etc.
    auto colors = make_small_color_map(g);

    using EdgeSet = unordered_set<NFAEdge>;
    EdgeSet backEdges;
    BackEdges<EdgeSet> be(backEdges);

    depth_first_search(g, visitor(be).root_vertex(g.start).color_map(colors));

    auto acyclic_g = make_filtered_graph(g, make_bad_edge_filter(&backEdges));

    vector<NFAVertex> ordering;
    ordering.reserve(num_vertices(g));
    topological_sort(acyclic_g, back_inserter(ordering), color_map(colors));

    reorderSpecials(g, ordering);

    return ordering;
}

static
void mustBeSetBefore_int(NFAVertex u, const NGHolder &g,
                         decltype(make_small_color_map(NGHolder())) &colors) {
    set<NFAVertex> s;
    insert(&s, adjacent_vertices(u, g));

    set<NFAEdge> dead; // Edges leading to u or u's successors.

    for (auto v : inv_adjacent_vertices_range(u, g)) {
        for (const auto &e : out_edges_range(v, g)) {
            NFAVertex t = target(e, g);
            if (t == u || contains(s, t)) {
                dead.insert(e);
            }
        }
    }

    auto prefix = make_filtered_graph(g, make_bad_edge_filter(&dead));

    depth_first_visit(prefix, g.start, make_dfs_visitor(boost::null_visitor()),
                      colors);
}

bool mustBeSetBefore(NFAVertex u, NFAVertex v, const NGHolder &g,
                     mbsb_cache &cache) {
    assert(&cache.g == &g);
    auto key = make_pair(g[u].index, g[v].index);
    DEBUG_PRINTF("cache checking (%zu)\n", cache.cache.size());
    if (contains(cache.cache, key)) {
        DEBUG_PRINTF("cache hit\n");
        return cache.cache[key];
    }

    auto colors = make_small_color_map(g);
    mustBeSetBefore_int(u, g, colors);

    for (auto vi : vertices_range(g)) {
        auto key2 = make_pair(g[u].index, g[vi].index);
        DEBUG_PRINTF("adding %zu %zu\n", key2.first, key2.second);
        assert(!contains(cache.cache, key2));
        bool value = get(colors, vi) == small_color::white;
        cache.cache[key2] = value;
        assert(contains(cache.cache, key2));
    }
    DEBUG_PRINTF("cache miss %zu %zu (%zu)\n", key.first, key.second,
                 cache.cache.size());
    return cache.cache[key];
}

void appendLiteral(NGHolder &h, const ue2_literal &s) {
    DEBUG_PRINTF("adding '%s' to graph\n", dumpString(s).c_str());
    vector<NFAVertex> tail;
    assert(in_degree(h.acceptEod, h) == 1);
    for (auto v : inv_adjacent_vertices_range(h.accept, h)) {
        tail.push_back(v);
    }
    assert(!tail.empty());

    for (auto v : tail) {
        remove_edge(v, h.accept, h);
    }

    for (const auto &c : s) {
        NFAVertex v = add_vertex(h);
        h[v].char_reach = c;
        for (auto u : tail) {
            add_edge(u, v, h);
        }
        tail.clear();
        tail.push_back(v);
    }

    for (auto v : tail) {
        add_edge(v, h.accept, h);
    }
}

flat_set<u32> getTops(const NGHolder &h) {
    flat_set<u32> tops;
    for (const auto &e : out_edges_range(h.start, h)) {
        insert(&tops, h[e].tops);
    }
    return tops;
}

void setTops(NGHolder &h, u32 top) {
    for (const auto &e : out_edges_range(h.start, h)) {
        assert(h[e].tops.empty());
        if (target(e, h) == h.startDs) {
            continue;
        }
        h[e].tops.insert(top);
    }
}

void clearReports(NGHolder &g) {
    DEBUG_PRINTF("clearing reports without an accept edge\n");
    unordered_set<NFAVertex> allow;
    insert(&allow, inv_adjacent_vertices(g.accept, g));
    insert(&allow, inv_adjacent_vertices(g.acceptEod, g));
    allow.erase(g.accept); // due to stylised edge.

    for (auto v : vertices_range(g)) {
        if (contains(allow, v)) {
            continue;
        }
        g[v].reports.clear();
    }
}

void duplicateReport(NGHolder &g, ReportID r_old, ReportID r_new) {
    for (auto v : vertices_range(g)) {
        auto &reports = g[v].reports;
        if (contains(reports, r_old)) {
            reports.insert(r_new);
        }
    }
}

static
void fillHolderOutEdges(NGHolder &out, const NGHolder &in,
                        const unordered_map<NFAVertex, NFAVertex> &v_map,
                        NFAVertex u) {
    NFAVertex u_new = v_map.at(u);

    for (auto e : out_edges_range(u, in)) {
        NFAVertex v = target(e, in);

        if (is_special(u, in) && is_special(v, in)) {
            continue;
        }

        auto it = v_map.find(v);
        if (it == v_map.end()) {
            continue;
        }
        NFAVertex v_new = it->second;
        assert(!edge(u_new, v_new, out).second);
        add_edge(u_new, v_new, in[e], out);
    }
}

void fillHolder(NGHolder *outp, const NGHolder &in, const deque<NFAVertex> &vv,
                unordered_map<NFAVertex, NFAVertex> *v_map_out) {
    NGHolder &out = *outp;
    unordered_map<NFAVertex, NFAVertex> &v_map = *v_map_out;

    out.kind = in.kind;

    for (auto v : vv) {
        if (is_special(v, in)) {
            continue;
        }
        v_map[v] = add_vertex(in[v], out);
    }

    for (u32 i = 0; i < N_SPECIALS; i++) {
        v_map[in.getSpecialVertex(i)] = out.getSpecialVertex(i);
    }

    DEBUG_PRINTF("copied %zu vertices to NG graph\n", v_map.size());

    fillHolderOutEdges(out, in, v_map, in.start);
    fillHolderOutEdges(out, in, v_map, in.startDs);

    for (auto u : vv) {
        if (is_special(u, in)) {
            continue;
        }
        fillHolderOutEdges(out, in, v_map, u);
    }

    renumber_edges(out);
    renumber_vertices(out);
}

void cloneHolder(NGHolder &out, const NGHolder &in) {
    assert(hasCorrectlyNumberedVertices(in));
    assert(hasCorrectlyNumberedVertices(out));
    out.kind = in.kind;

    // Note: depending on the state of the input graph, some stylized edges
    // (e.g. start->startDs) may not exist. This must be propagated to the
    // output graph as well.

    /* remove the existing special edges */
    clear_vertex(out.startDs, out);
    clear_vertex(out.accept, out);
    renumber_edges(out);

    vector<NFAVertex> out_mapping(num_vertices(in));
    out_mapping[NODE_START] = out.start;
    out_mapping[NODE_START_DOTSTAR] = out.startDs;
    out_mapping[NODE_ACCEPT] = out.accept;
    out_mapping[NODE_ACCEPT_EOD] = out.acceptEod;

    for (auto v : vertices_range(in)) {
        u32 i = in[v].index;

        /* special vertices are already in the out graph */
        if (i >= N_SPECIALS) {
            assert(!out_mapping[i]);
            out_mapping[i] = add_vertex(in[v], out);
        }

        out[out_mapping[i]] = in[v];
    }

    for (auto e : edges_range(in)) {
        u32 si = in[source(e, in)].index;
        u32 ti = in[target(e, in)].index;

        DEBUG_PRINTF("adding edge %u->%u\n", si, ti);

        NFAVertex s = out_mapping[si];
        NFAVertex t = out_mapping[ti];
        NFAEdge e2 = add_edge(s, t, out);
        out[e2] = in[e];
    }

    // Safety checks.
    assert(num_vertices(in) == num_vertices(out));
    assert(num_edges(in) == num_edges(out));
    assert(hasCorrectlyNumberedVertices(out));
}

void cloneHolder(NGHolder &out, const NGHolder &in,
                 unordered_map<NFAVertex, NFAVertex> *mapping) {
    cloneHolder(out, in);
    vector<NFAVertex> out_verts(num_vertices(in));
    for (auto v : vertices_range(out)) {
        out_verts[out[v].index] = v;
    }

    mapping->clear();

    for (auto v : vertices_range(in)) {
        (*mapping)[v] = out_verts[in[v].index];
        assert((*mapping)[v]);
    }
}

unique_ptr<NGHolder> cloneHolder(const NGHolder &in) {
    unique_ptr<NGHolder> h = ue2::make_unique<NGHolder>();
    cloneHolder(*h, in);
    return h;
}

void reverseHolder(const NGHolder &g_in, NGHolder &g) {
    // Make the BGL do the grunt work.
    unordered_map<NFAVertex, NFAVertex> vertexMap;
    boost::transpose_graph(g_in, g,
                orig_to_copy(boost::make_assoc_property_map(vertexMap)));

    // The transpose_graph operation will have created extra copies of our
    // specials. We have to rewire their neighbours to the 'real' specials and
    // delete them.
    NFAVertex start = vertexMap[g_in.acceptEod];
    NFAVertex startDs = vertexMap[g_in.accept];
    NFAVertex accept = vertexMap[g_in.startDs];
    NFAVertex acceptEod = vertexMap[g_in.start];

    // Successors of starts.
    for (const auto &e : out_edges_range(start, g)) {
        NFAVertex v = target(e, g);
        add_edge(g.start, v, g[e], g);
    }
    for (const auto &e : out_edges_range(startDs, g)) {
        NFAVertex v = target(e, g);
        add_edge(g.startDs, v, g[e], g);
    }

    // Predecessors of accepts.
    for (const auto &e : in_edges_range(accept, g)) {
        NFAVertex u = source(e, g);
        add_edge(u, g.accept, g[e], g);
    }
    for (const auto &e : in_edges_range(acceptEod, g)) {
        NFAVertex u = source(e, g);
        add_edge(u, g.acceptEod, g[e], g);
    }

    // Remove our impostors.
    clear_vertex(start, g);
    remove_vertex(start, g);
    clear_vertex(startDs, g);
    remove_vertex(startDs, g);
    clear_vertex(accept, g);
    remove_vertex(accept, g);
    clear_vertex(acceptEod, g);
    remove_vertex(acceptEod, g);

    // Renumber so that g's properties (number of vertices, edges) are
    // accurate.
    renumber_vertices(g);
    renumber_edges(g);

    assert(num_vertices(g) == num_vertices(g_in));
    assert(num_edges(g) == num_edges(g_in));
}

u32 removeTrailingLiteralStates(NGHolder &g, const ue2_literal &lit,
                                u32 max_delay, bool overhang_ok) {
    assert(isCorrectlyTopped(g));
    if (max_delay == numeric_limits<u32>::max()) {
        max_delay--;
    }

    DEBUG_PRINTF("killing off '%s'\n", dumpString(lit).c_str());
    set<NFAVertex> curr, next;
    curr.insert(g.accept);

    auto it = lit.rbegin();
    for (u32 delay = max_delay; delay > 0 && it != lit.rend(); delay--, ++it) {
        next.clear();
        for (auto v : curr) {
            for (auto u : inv_adjacent_vertices_range(v, g)) {
                if (u == g.start) {
                    if (overhang_ok) {
                        DEBUG_PRINTF("bail\n");
                        goto bail; /* things got complicated */
                    } else {
                        continue; /* it is not possible for a lhs literal to
                                   * overhang the start */
                    }
                }

                const CharReach &cr = g[u].char_reach;
                if (!overlaps(*it, cr)) {
                    DEBUG_PRINTF("skip\n");
                    continue;
                }
                if (isSubsetOf(*it, cr)) {
                    next.insert(u);
                } else {
                    DEBUG_PRINTF("bail\n");
                    goto bail; /* things got complicated */
                }
            }
        }

        curr.swap(next);
    }
 bail:
    if (curr.empty()) {
        /* This can happen when we have an edge representing a cross from two
         * sides of an alternation. This whole edge needs to be marked as
         * dead */
        assert(0); /* should have been picked up by can match */
        return numeric_limits<u32>::max();
    }

    u32 delay = distance(lit.rbegin(), it);
    assert(delay <= max_delay);
    assert(delay <= lit.length());
    DEBUG_PRINTF("managed delay %u (of max %u)\n", delay, max_delay);

    set<NFAVertex> pred;
    for (auto v : curr) {
        insert(&pred, inv_adjacent_vertices_range(v, g));
    }

    clear_in_edges(g.accept, g);
    clearReports(g);

    for (auto v : pred) {
        NFAEdge e = add_edge(v, g.accept, g);
        g[v].reports.insert(0);
        if (is_triggered(g) && v == g.start) {
            g[e].tops.insert(DEFAULT_TOP);
        }
    }

    pruneUseless(g);
    assert(allMatchStatesHaveReports(g));
    assert(isCorrectlyTopped(g));

    DEBUG_PRINTF("graph has %zu vertices left\n", num_vertices(g));
    return delay;
}

#ifndef NDEBUG

bool allMatchStatesHaveReports(const NGHolder &g) {
    unordered_set<NFAVertex> reporters;
    for (auto v : inv_adjacent_vertices_range(g.accept, g)) {
        if (g[v].reports.empty()) {
            DEBUG_PRINTF("vertex %zu has no reports!\n", g[v].index);
            return false;
        }
        reporters.insert(v);
    }

    for (auto v : inv_adjacent_vertices_range(g.acceptEod, g)) {
        if (v == g.accept) {
            continue; // stylised edge
        }
        if (g[v].reports.empty()) {
            DEBUG_PRINTF("vertex %zu has no reports!\n", g[v].index);
            return false;
        }
        reporters.insert(v);
    }

    for (auto v : vertices_range(g)) {
        if (!contains(reporters, v) && !g[v].reports.empty()) {
            DEBUG_PRINTF("vertex %zu is not a match state, but has reports!\n",
                         g[v].index);
            return false;
        }
    }

    return true;
}

bool isCorrectlyTopped(const NGHolder &g) {
    if (is_triggered(g)) {
        for (const auto &e : out_edges_range(g.start, g)) {
            if (g[e].tops.empty() != (target(e, g) == g.startDs)) {
                return false;
            }
        }
    } else {
        for (const auto &e : out_edges_range(g.start, g)) {
            if (!g[e].tops.empty()) {
                return false;
            }
        }
    }

    return true;
}

#endif // NDEBUG

} // namespace ue2
