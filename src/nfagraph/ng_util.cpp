/*
 * Copyright (c) 2015, Intel Corporation
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
#include "ng_depth.h" // for NFAVertexDepth
#include "ng_dump.h"
#include "ue2common.h"
#include "nfa/limex_limits.h" // for NFA_MAX_TOP_MASKS.
#include "parser/position.h"
#include "util/graph_range.h"
#include "util/make_unique.h"
#include "util/order_check.h"
#include "util/ue2string.h"
#include "util/report_manager.h"

#include <map>
#include <set>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/strong_components.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/range/adaptor/map.hpp>

using namespace std;
using boost::default_color_type;
using boost::filtered_graph;
using boost::make_assoc_property_map;
using boost::adaptors::map_values;

namespace ue2 {

depth maxDistFromInit(const NFAVertexDepth &vd) {
    if (vd.fromStart.max.is_unreachable()) {
        return vd.fromStartDotStar.max;
    } else if (vd.fromStartDotStar.max.is_unreachable()) {
        return vd.fromStart.max;
    } else {
        return max(vd.fromStartDotStar.max, vd.fromStart.max);
    }
}

depth maxDistFromStartOfData(const NFAVertexDepth &vd) {
    if (vd.fromStartDotStar.max.is_reachable()) {
        /* the irrepressible nature of floating literals cannot be contained */
        return depth::infinity();
    } else {
        return vd.fromStart.max;
    }
}

NFAVertex getSoleDestVertex(const NGHolder &g, NFAVertex a) {
    assert(a != NFAGraph::null_vertex());

    NFAGraph::out_edge_iterator ii, iie;
    tie(ii, iie) = out_edges(a, g);
    if (ii == iie) {
        return NFAGraph::null_vertex();
    }
    NFAVertex b = target(*ii, g);
    if (a == b) {
        ++ii;
        if (ii == iie) {
            return NFAGraph::null_vertex();
        }

        b = target(*ii, g);
        if (++ii != iie) {
            return NFAGraph::null_vertex();
        }
    } else if (++ii != iie && (target(*ii, g) != a || ++ii != iie)) {
        return NFAGraph::null_vertex();
    }

    assert(a != b);
    return b;
}

NFAVertex getSoleSourceVertex(const NGHolder &g, NFAVertex a) {
    assert(a != NFAGraph::null_vertex());

    u32 idegree = in_degree(a, g);
    if (idegree != 1 && !(idegree == 2 && hasSelfLoop(a, g))) {
        return NFAGraph::null_vertex();
    }

    NFAGraph::in_edge_iterator ii, iie;
    tie(ii, iie) = in_edges(a, g);
    if (ii == iie) {
        return NFAGraph::null_vertex();
    }
    NFAVertex b = source(*ii, g);
    if (a == b) {
        ++ii;
        if (ii == iie) {
            return NFAGraph::null_vertex();
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
        NFAEdge clone = add_edge(dest, t, g).first;
        u32 idx = g[clone].index;
        g[clone] = g[e];
        g[clone].index = idx;
    }
}

void clone_in_edges(NGHolder &g, NFAVertex s, NFAVertex dest) {
    for (const auto &e : in_edges_range(s, g)) {
        NFAVertex ss = source(e, g);
        assert(!edge(ss, dest, g).second);
        NFAEdge clone = add_edge(ss, dest, g).first;
        u32 idx = g[clone].index;
        g[clone] = g[e];
        g[clone].index = idx;
    }
}

bool onlyOneTop(const NGHolder &g) {
    set<u32> tops;
    for (const auto &e : out_edges_range(g.start, g)) {
        tops.insert(g[e].top);
    }
    assert(!tops.empty());
    return tops.size() == 1;
}

namespace {
struct CycleFound {};
struct DetectCycles : public boost::default_dfs_visitor {
    explicit DetectCycles(const NGHolder &g) : startDs(g.startDs) {}
    void back_edge(const NFAEdge &e, const NFAGraph &g) const {
        NFAVertex u = source(e, g), v = target(e, g);
        // We ignore the startDs self-loop.
        if (u == startDs && v == startDs) {
            return;
        }
        // Any other back-edge indicates a cycle.
        DEBUG_PRINTF("back edge %u->%u found\n", g[u].index,
                     g[v].index);
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

bool isAcyclic(const NGHolder &g) {
    try {
        depth_first_search(
            g.g, visitor(DetectCycles(g))
                     .root_vertex(g.start)
                     .vertex_index_map(get(&NFAGraphVertexProps::index, g.g)));
    } catch (const CycleFound &) {
        return false;
    }

    return true;
}

/** True if the graph has a cycle reachable from the given source vertex. */
bool hasReachableCycle(const NGHolder &g, NFAVertex src) {
    assert(hasCorrectlyNumberedVertices(g));
    vector<default_color_type> colors(num_vertices(g));

    try {
        // Use depth_first_visit, rather than depth_first_search, so that we
        // only search from src.
        auto index_map = get(&NFAGraphVertexProps::index, g.g);
        depth_first_visit(
            g.g, src, DetectCycles(g),
            make_iterator_property_map(colors.begin(), index_map));
    } catch (const CycleFound&) {
        return true;
    }

    return false;
}

bool hasBigCycles(const NGHolder &g) {
    assert(hasCorrectlyNumberedVertices(g));
    set<NFAEdge> dead;
    BackEdges<set<NFAEdge>> backEdgeVisitor(dead);
    depth_first_search(
        g.g, visitor(backEdgeVisitor)
                 .root_vertex(g.start)
                 .vertex_index_map(get(&NFAGraphVertexProps::index, g.g)));

    for (const auto &e : dead) {
        if (source(e, g) != target(e, g)) {
            return true;
        }
    }

    return false;
}

set<NFAVertex> findVerticesInCycles(const NGHolder &g) {
    map<NFAVertex, size_t> comp_map;

    strong_components(g.g, make_assoc_property_map(comp_map),
                      vertex_index_map(get(&NFAGraphVertexProps::index, g.g)));

    map<size_t, set<NFAVertex> > comps;

    for (const auto &e : comp_map) {
        comps[e.second].insert(e.first);
    }


    set<NFAVertex> rv;

    for (const auto &comp : comps | map_values) {
        /* every vertex in a strongly connected component is reachable from
         * every other vertex in the component. A vertex is involved in a cycle
         * therefore if it is in a strongly connected component with more than
         * one vertex or if it is the only vertex and it has a self loop. */
        assert(!comp.empty());
        if (comp.size() > 1) {
            insert(&rv, comp);
        }
        NFAVertex v = *comp.begin();
        if (hasSelfLoop(v, g)) {
            rv.insert(v);
        }
    }

    return rv;
}

bool can_never_match(const NGHolder &g) {
    assert(edge(g.accept, g.acceptEod, g).second);
    if (!hasGreaterInDegree(0, g.accept, g)
        && !hasGreaterInDegree(1, g.acceptEod, g)) {
        DEBUG_PRINTF("no paths into accept\n");
        return true;
    }

    return false;
}

bool can_match_at_eod(const NGHolder &h) {
    if (hasGreaterInDegree(1, h.acceptEod, h)) {
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
    NFAGraph::in_edge_iterator ie, ee;
    tie(ie, ee) = in_edges(g.accept, g);

    return ie == ee;
}

bool matches_everywhere(const NGHolder &h) {
    NFAEdge e;
    bool exists;
    tie(e, exists) = edge(h.startDs, h.accept, h);

    return exists && !h[e].assert_flags;
}

bool is_virtual_start(NFAVertex v, const NGHolder &g) {
    return g[v].assert_flags & POS_FLAG_VIRTUAL_START;
}

vector<NFAVertex> getTopoOrdering(const NGHolder &g) {
    assert(hasCorrectlyNumberedVertices(g));

    // Use the same colour map for both DFS and topological_sort below: avoids
    // having to reallocate it, etc.
    const size_t num_verts = num_vertices(g);
    vector<default_color_type> colour(num_verts);

    using EdgeSet = ue2::unordered_set<NFAEdge>;
    EdgeSet backEdges;
    BackEdges<EdgeSet> be(backEdges);

    auto index_map = get(&NFAGraphVertexProps::index, g.g);
    depth_first_search(g.g, visitor(be)
                                .root_vertex(g.start)
                                .color_map(make_iterator_property_map(
                                    colour.begin(), index_map))
                                .vertex_index_map(index_map));

    AcyclicFilter<EdgeSet> af(&be.backEdges);
    filtered_graph<NFAGraph, AcyclicFilter<EdgeSet>> acyclic_g(g.g, af);

    vector<NFAVertex> ordering;
    ordering.reserve(num_verts);
    topological_sort(
        acyclic_g, back_inserter(ordering),
        color_map(make_iterator_property_map(colour.begin(), index_map))
            .vertex_index_map(index_map));

    return ordering;
}

static
void mustBeSetBefore_int(NFAVertex u, const NGHolder &g,
                         vector<default_color_type> &vertexColor) {
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

    // The AcyclicFilter is badly named, it's really just an edge-set filter.
    filtered_graph<NFAGraph, AcyclicFilter<set<NFAEdge>>> prefix(g.g,
                                    AcyclicFilter<set<NFAEdge>>(&dead));

    depth_first_visit(
        prefix, g.start, make_dfs_visitor(boost::null_visitor()),
        make_iterator_property_map(vertexColor.begin(),
                                   get(&NFAGraphVertexProps::index, g.g)));
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

    vector<default_color_type> vertexColor(num_vertices(g));
    mustBeSetBefore_int(u, g, vertexColor);

    for (auto vi : vertices_range(g)) {
        auto key2 = make_pair(g[u].index,
                              g[vi].index);
        DEBUG_PRINTF("adding %u %u\n", key2.first, key2.second);
        assert(!contains(cache.cache, key2));
        bool value = vertexColor[g[vi].index] == boost::white_color;
        cache.cache[key2] = value;
        assert(contains(cache.cache, key2));
    }
    DEBUG_PRINTF("cache miss %u %u (%zu)\n", key.first, key.second,
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

ue2::flat_set<u32> getTops(const NGHolder &h) {
    ue2::flat_set<u32> tops;
    for (const auto &e : out_edges_range(h.start, h)) {
        NFAVertex v = target(e, h);
        if (v == h.startDs) {
            continue;
        }
        u32 top = h[e].top;
        assert(top < NFA_MAX_TOP_MASKS);
        tops.insert(top);
    }
    return tops;
}

void clearReports(NGHolder &g) {
    DEBUG_PRINTF("clearing reports without an accept edge\n");
    ue2::unordered_set<NFAVertex> allow;
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
                        const ue2::unordered_map<NFAVertex, NFAVertex> &v_map,
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
                ue2::unordered_map<NFAVertex, NFAVertex> *v_map_out) {
    NGHolder &out = *outp;
    ue2::unordered_map<NFAVertex, NFAVertex> &v_map = *v_map_out;

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

    out.renumberEdges();
    out.renumberVertices();
}

void cloneHolder(NGHolder &out, const NGHolder &in) {
    assert(hasCorrectlyNumberedVertices(in));
    out.kind = in.kind;

    // Note: depending on the state of the input graph, some stylized edges
    // (e.g. start->startDs) may not exist. This must be propagated to the
    // output graph as well.

    /* remove the existing special edges */
    clear_vertex(out.startDs, out);
    clear_vertex(out.accept, out);

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
        UNUSED bool added;
        NFAEdge e2;
        tie(e2, added) = add_edge(s, t, out);
        assert(added);
        out[e2] = in[e];
    }

    // Safety checks.
    assert(num_vertices(in.g) == num_vertices(out.g));
    assert(num_edges(in.g) == num_edges(out.g));
    assert(hasCorrectlyNumberedVertices(out));
}

void cloneHolder(NGHolder &out, const NGHolder &in,
                 ue2::unordered_map<NFAVertex, NFAVertex> *mapping) {
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

#ifndef NDEBUG
/** \brief Used in sanity-checking assertions: returns true if all vertices
 * leading to accept or acceptEod have at least one report ID. */
bool allMatchStatesHaveReports(const NGHolder &g) {
    for (auto v : inv_adjacent_vertices_range(g.accept, g)) {
        if (g[v].reports.empty()) {
            DEBUG_PRINTF("vertex %u has no reports!\n",
                         g[v].index);
            return false;
        }
    }
    for (auto v : inv_adjacent_vertices_range(g.acceptEod, g)) {
        if (v == g.accept) {
            continue; // stylised edge
        }
        if (g[v].reports.empty()) {
            DEBUG_PRINTF("vertex %u has no reports!\n",
                         g[v].index);
            return false;
        }
    }
    return true;
}

/** Assertion: returns true if the vertices in this graph are contiguously (and
 * uniquely) numbered from zero. */
bool hasCorrectlyNumberedVertices(const NGHolder &g) {
    size_t count = num_vertices(g);
    vector<bool> ids(count, false);
    for (auto v : vertices_range(g)) {
        u32 id = g[v].index;
        if (id >= count || ids[id]) {
            return false; // duplicate
        }
        ids[id] = true;
    }
    return find(ids.begin(), ids.end(), false) == ids.end();
}

/** Assertion: returns true if the edges in this graph are contiguously (and
 * uniquely) numbered from zero. */
bool hasCorrectlyNumberedEdges(const NGHolder &g) {
    size_t count = num_edges(g);
    vector<bool> ids(count, false);
    for (const auto &e : edges_range(g)) {
        u32 id = g[e].index;
        if (id >= count || ids[id]) {
            return false; // duplicate
        }
        ids[id] = true;
    }
    return find(ids.begin(), ids.end(), false) == ids.end();
}
#endif // NDEBUG

} // namespace ue2
