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
 * \brief Bounded repeat analysis.
 */
#include "ng_repeat.h"

#include "grey.h"
#include "ng_depth.h"
#include "ng_holder.h"
#include "ng_limex_accel.h"
#include "ng_prune.h"
#include "ng_reports.h"
#include "ng_som_util.h"
#include "ng_util.h"
#include "nfa/accel.h"
#include "nfa/limex_limits.h"
#include "nfa/repeat_internal.h"
#include "nfa/repeatcompile.h"
#include "util/container.h"
#include "util/dump_charclass.h"
#include "util/graph_range.h"
#include "util/graph_small_color_map.h"
#include "util/graph_undirected.h"
#include "util/report_manager.h"
#include "util/unordered.h"

#include <algorithm>
#include <map>
#include <queue>
#include <unordered_map>
#include <unordered_set>

#include <boost/graph/connected_components.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/reverse_graph.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/icl/interval_set.hpp>

using namespace std;
using boost::depth_first_search;
using boost::depth_first_visit;
using boost::make_assoc_property_map;

namespace ue2 {

namespace {

/**
 * \brief Filter that retains only edges between vertices with the same
 * reachability. Special vertices are dropped.
 */
template<class Graph>
struct ReachFilter {
    ReachFilter() = default;
    explicit ReachFilter(const Graph *g_in) : g(g_in) {}

    // Convenience typedefs.
    using Traits = typename boost::graph_traits<Graph>;
    using VertexDescriptor = typename Traits::vertex_descriptor;
    using EdgeDescriptor = typename Traits::edge_descriptor;

    bool operator()(const VertexDescriptor &v) const {
        assert(g);
        // Disallow special vertices, as otherwise we will try to remove them
        // later.
        return !is_special(v, *g);
    }

    bool operator()(const EdgeDescriptor &e) const {
        assert(g);
        // Vertices must have the same reach.
        auto u = source(e, *g), v = target(e, *g);
        const CharReach &cr_u = (*g)[u].char_reach;
        const CharReach &cr_v = (*g)[v].char_reach;
        return cr_u == cr_v;
    }

    const Graph *g = nullptr;
};

using RepeatGraph = boost::filtered_graph<NGHolder, ReachFilter<NGHolder>,
                                          ReachFilter<NGHolder>>;

struct ReachSubgraph {
    vector<NFAVertex> vertices;
    depth repeatMin{0};
    depth repeatMax{0};
    u32 minPeriod = 1;
    bool is_reset = false;
    enum RepeatType historyType = REPEAT_RING;
    bool bad = false; // if true, ignore this case
};

} // namespace

static
void findInitDepths(const NGHolder &g,
                    unordered_map<NFAVertex, NFAVertexDepth> &depths) {
    auto d = calcDepths(g);

    for (auto v : vertices_range(g)) {
        size_t idx = g[v].index;
        assert(idx < d.size());
        depths.emplace(v, d[idx]);
    }
}

static
vector<NFAVertex> buildTopoOrder(const RepeatGraph &g) {
    /* Note: RepeatGraph is a filtered version of NGHolder and still has
     * NFAVertex as its vertex descriptor */

    typedef unordered_set<NFAEdge> EdgeSet;
    EdgeSet deadEdges;

    // We don't have indices spanning [0,N] on our filtered graph, so we
    // provide a colour map.
    unordered_map<NFAVertex, boost::default_color_type> colours;

    depth_first_search(g, visitor(BackEdges<EdgeSet>(deadEdges)).
                          color_map(make_assoc_property_map(colours)));
    auto acyclic_g = make_filtered_graph(g, make_bad_edge_filter(&deadEdges));

    vector<NFAVertex> topoOrder;
    topological_sort(acyclic_g, back_inserter(topoOrder),
                     color_map(make_assoc_property_map(colours)));

    reverse(topoOrder.begin(), topoOrder.end());

    return topoOrder;
}

static
void proper_pred(const NGHolder &g, NFAVertex v,
                 unordered_set<NFAVertex> &p) {
    pred(g, v, &p);
    p.erase(v); // self-loops
}

static
void proper_succ(const NGHolder &g, NFAVertex v,
                 unordered_set<NFAVertex> &s) {
    succ(g, v, &s);
    s.erase(v); // self-loops
}

static
bool roguePredecessor(const NGHolder &g, NFAVertex v,
                      const unordered_set<NFAVertex> &involved,
                      const unordered_set<NFAVertex> &pred) {
    u32 seen = 0;

    for (auto u : inv_adjacent_vertices_range(v, g)) {
        if (contains(involved, u)) {
            continue;
        }
        if (!contains(pred, u)) {
            DEBUG_PRINTF("%zu is a rogue pred\n", g[u].index);
            return true;
        }

        seen++;
    }

    // We must have edges from either (a) none of our external predecessors, or
    // (b) all of our external predecessors.
    if (!seen) {
        return false;
    }
    return pred.size() != seen;
}

static
bool rogueSuccessor(const NGHolder &g, NFAVertex v,
                    const unordered_set<NFAVertex> &involved,
                    const unordered_set<NFAVertex> &succ) {
    u32 seen = 0;
    for (auto w : adjacent_vertices_range(v, g)) {
        if (contains(involved, w)) {
            continue;
        }

        if (!contains(succ, w)) {
            DEBUG_PRINTF("%zu is a rogue succ\n", g[w].index);
            return true;
        }

        seen++;
    }

    // We must have edges to either (a) none of our external successors, or
    // (b) all of our external successors.
    if (!seen) {
        return false;
    }
    return succ.size() != seen;
}

static
bool hasDifferentTops(const NGHolder &g, const vector<NFAVertex> &verts) {
    /* TODO: check that we need this now that we allow multiple tops */
    const flat_set<u32> *tops = nullptr;

    for (auto v : verts) {
        for (const auto &e : in_edges_range(v, g)) {
            NFAVertex u = source(e, g);
            if (u != g.start && u != g.startDs) {
                continue; // Only edges from starts have valid top properties.
            }
            DEBUG_PRINTF("edge (%zu,%zu) with %zu tops\n", g[u].index,
                         g[v].index, g[e].tops.size());
            if (!tops) {
                tops = &g[e].tops;
            } else if (g[e].tops != *tops) {
                return true; // More than one set of tops.
            }
        }
    }

    return false;
}

static
bool vertexIsBad(const NGHolder &g, NFAVertex v,
                 const unordered_set<NFAVertex> &involved,
                 const unordered_set<NFAVertex> &tail,
                 const unordered_set<NFAVertex> &pred,
                 const unordered_set<NFAVertex> &succ,
                 const flat_set<ReportID> &reports) {
    DEBUG_PRINTF("check vertex %zu\n", g[v].index);

    // We must drop any vertex that is the target of a back-edge within
    // our subgraph. The tail set contains all vertices that are after v in a
    // topo ordering.
    for (auto u : inv_adjacent_vertices_range(v, g)) {
        if (contains(tail, u)) {
            DEBUG_PRINTF("back-edge (%zu,%zu) in subgraph found\n",
                         g[u].index, g[v].index);
            return true;
        }
    }

    // If this vertex has an entry from outside our subgraph, it must have
    // edges from *all* the vertices in pred and no other external entries.
    // Similarly for exits.
    if (roguePredecessor(g, v, involved, pred)) {
        DEBUG_PRINTF("preds for %zu not well-formed\n", g[v].index);
        return true;
    }

    if (rogueSuccessor(g, v, involved, succ)) {
        DEBUG_PRINTF("succs for %zu not well-formed\n", g[v].index);
        return true;
    }

    // All reporting vertices should have the same reports.
    if (is_match_vertex(v, g) && reports != g[v].reports) {
        DEBUG_PRINTF("report mismatch to %zu\n", g[v].index);
        return true;
    }

    return false;
}

static
void splitSubgraph(const NGHolder &g, const deque<NFAVertex> &verts,
                   const u32 minNumVertices, queue<ReachSubgraph> &q) {
    DEBUG_PRINTF("entry\n");

    // We construct a copy of the graph using just the vertices we want, rather
    // than using a filtered_graph -- this way is faster.
    NGHolder verts_g;
    unordered_map<NFAVertex, NFAVertex> verts_map; // in g -> in verts_g
    fillHolder(&verts_g, g, verts, &verts_map);

    const auto ug = make_undirected_graph(verts_g);

    unordered_map<NFAVertex, u32> repeatMap;

    size_t num = connected_components(ug, make_assoc_property_map(repeatMap));
    DEBUG_PRINTF("found %zu connected repeat components\n", num);
    assert(num > 0);

    vector<ReachSubgraph> rs(num);

    for (auto v : verts) {
        assert(!is_special(v, g));
        auto vu = verts_map.at(v);
        auto rit = repeatMap.find(vu);
        if (rit == repeatMap.end()) {
            continue; /* not part of a repeat */
        }
        u32 comp_id = rit->second;
        assert(comp_id < num);
        rs[comp_id].vertices.push_back(v);
    }

    for (const auto &rsi : rs) {
        if (rsi.vertices.empty()) {
            // Empty elements can happen when connected_components finds a
            // subgraph consisting entirely of specials (which aren't added to
            // ReachSubgraph in the loop above). There's nothing we can do with
            // these, so we skip them.
            continue;
        }
        DEBUG_PRINTF("repeat with %zu vertices\n", rsi.vertices.size());
        if (rsi.vertices.size() >= minNumVertices) {
            DEBUG_PRINTF("enqueuing\n");
            q.push(rsi);
        }
    }
}

static
void findFirstReports(const NGHolder &g, const ReachSubgraph &rsi,
                      flat_set<ReportID> &reports) {
    for (auto v : rsi.vertices) {
        if (is_match_vertex(v, g)) {
            reports = g[v].reports;
            return;
        }
    }
}

static
void checkReachSubgraphs(const NGHolder &g, vector<ReachSubgraph> &rs,
                         const u32 minNumVertices) {
    if (rs.empty()) {
        return;
    }

    DEBUG_PRINTF("%zu subgraphs\n", rs.size());

    vector<ReachSubgraph> rs_out;

    queue<ReachSubgraph> q;
    for (const auto &rsi : rs) {
        if (rsi.vertices.size() < minNumVertices) {
            continue;
        }
        q.push(rsi);
    }

    while (!q.empty()) {
        const ReachSubgraph &rsi = q.front();

        if (rsi.vertices.size() < minNumVertices) {
            q.pop(); // Too small for consideration as a repeat.
            continue;
        }

        DEBUG_PRINTF("subgraph with %zu vertices\n", rsi.vertices.size());

        // Check that all the edges from outside have the same tops. TODO: we
        // don't have to throw the whole subgraph out, we could do this check
        // on a per vertex basis.
        if (hasDifferentTops(g, rsi.vertices)) {
            DEBUG_PRINTF("different tops!\n");
            q.pop();
            continue;
        }

        unordered_set<NFAVertex> involved(rsi.vertices.begin(),
                                          rsi.vertices.end());
        unordered_set<NFAVertex> tail(involved); // to look for back-edges.
        unordered_set<NFAVertex> pred, succ;
        proper_pred(g, rsi.vertices.front(), pred);
        proper_succ(g, rsi.vertices.back(), succ);

        flat_set<ReportID> reports;
        findFirstReports(g, rsi, reports);

        bool recalc = false;
        deque<NFAVertex> verts;

        for (auto v : rsi.vertices) {
            tail.erase(v); // now contains all vertices _after_ this one.

            if (vertexIsBad(g, v, involved, tail, pred, succ, reports)) {
                recalc = true;
                continue;
            }

            verts.push_back(v);
        }

        if (recalc) {
            if (verts.size() < minNumVertices) {
                DEBUG_PRINTF("subgraph got too small\n");
                q.pop();
                continue;
            }
            splitSubgraph(g, verts, minNumVertices, q);
        } else {
            DEBUG_PRINTF("subgraph is ok\n");
            rs_out.push_back(rsi);
        }
        q.pop();
    }

    rs.swap(rs_out);
}

namespace {
class DistanceSet {
private:
    // We use boost::icl to do the heavy lifting.
    typedef boost::icl::closed_interval<u32> ClosedInterval;
    typedef boost::icl::interval_set<u32, std::less, ClosedInterval>
        IntervalSet;
    IntervalSet distances;
public:
    // Add a distance.
    void insert(u32 d) {
        distances.insert(d);
    }

    void add(const DistanceSet &a) {
        distances += a.distances; // union operation
    }

    // Increment all the distances by one and add.
    void add_incremented(const DistanceSet &a) {
        for (const auto &d : a.distances) {
            u32 lo = lower(d) + 1;
            u32 hi = upper(d) + 1;
            distances.insert(boost::icl::construct<ClosedInterval>(lo, hi));
        }
    }

#ifdef DEBUG
    void dump() const {
        if (distances.empty()) {
            printf("<empty>");
            return;
        }

        for (const auto &d : distances) {
            printf("[%u,%u] ", lower(d), upper(d));
        }
    }
#endif

    // True if this distance set is a single contiguous interval.
    bool is_contiguous() const {
        IntervalSet::const_iterator it = distances.begin();
        if (it == distances.end()) {
            return false;
        }
        ++it;
        return (it == distances.end());
    }

    pair<u32, u32> get_range() const {
        assert(is_contiguous());
        return make_pair(lower(distances), upper(distances));
    }
};
}

/**
 * Returns false if the given bounds are too large to be implemented with our
 * runtime engines that handle bounded repeats.
 */
static
bool tooLargeToImplement(const depth &repeatMin, const depth &repeatMax) {
    if (!repeatMin.is_finite()) {
        DEBUG_PRINTF("non-finite min bound %s\n", repeatMin.str().c_str());
        assert(0); // this is a surprise!
        return true;
    }

    if ((u32)repeatMin >= REPEAT_INF) {
        DEBUG_PRINTF("min bound %s too large\n", repeatMin.str().c_str());
        return true;
    }

    if (repeatMax.is_finite() && (u32)repeatMax >= REPEAT_INF) {
        DEBUG_PRINTF("finite max bound %s too large\n", repeatMax.str().c_str());
        return true;
    }

    return false;
}

/** Returns false if the graph is not a supported bounded repeat. */
static
bool processSubgraph(const NGHolder &g, ReachSubgraph &rsi,
                     u32 minNumVertices) {
    DEBUG_PRINTF("reach subgraph has %zu vertices\n", rsi.vertices.size());

    if (rsi.vertices.size() < minNumVertices) {
        DEBUG_PRINTF("too small, min is %u\n", minNumVertices);
        return false;
    }

    NFAVertex first = rsi.vertices.front();
    NFAVertex last = rsi.vertices.back();

    typedef unordered_map<NFAVertex, DistanceSet> DistanceMap;
    DistanceMap dist;

    // Initial distance sets.
    for (auto u : inv_adjacent_vertices_range(first, g)) {
        if (u == first) {
            continue; // no self-loops
        }
        DEBUG_PRINTF("pred vertex %zu\n", g[u].index);
        dist[u].insert(0);
    }

    for (auto v : rsi.vertices) {
        for (auto u : inv_adjacent_vertices_range(v, g)) {
            if (u == v) {
                continue; // no self-loops
            }

            auto di = dist.find(u);
            if (di == dist.end()) {
                assert(0);
                return false;
            }

            dist[v].add_incremented(di->second);
        }
    }

    // Remove pred distances from our map.
    for (auto u : inv_adjacent_vertices_range(first, g)) {
        if (u == first) {
            continue; // no self-loops
        }
        dist.erase(u);
    }

    // Calculate final union of distances.
    DistanceSet final_d;
    for (auto v : adjacent_vertices_range(last, g)) {
        if (v == last) {
            continue; // no self-loops
        }
        for (auto u : inv_adjacent_vertices_range(v, g)) {
            if (u == v) {
                continue; // no self-loops
            }
            auto di = dist.find(u);
            if (di == dist.end()) {
                continue;
            }
            final_d.add(di->second);
        }
    }

#ifdef DEBUG
    DEBUG_PRINTF("final_d dists: ");
    final_d.dump();
    printf("\n");
#endif

    if (!final_d.is_contiguous()) {
        // not handled right now
        DEBUG_PRINTF("not contiguous!\n");
        return false;
    }

    pair<u32, u32> range = final_d.get_range();
    if (range.first > depth::max_value() || range.second > depth::max_value()) {
        DEBUG_PRINTF("repeat (%u,%u) not representable with depths\n",
                     range.first, range.second);
        return false;
    }
    rsi.repeatMin = depth(range.first);
    rsi.repeatMax = depth(range.second);

    // If we've got a self-loop anywhere, we've got inf max.
    if (anySelfLoop(g, rsi.vertices.begin(), rsi.vertices.end())) {
        DEBUG_PRINTF("repeat contains self-loop, setting max to INF\n");
        rsi.repeatMax = depth::infinity();
    }

    // If our pattern contains a bounded repeat that we wouldn't be able to
    // implement as runtime, then we have no strategy that leads to
    // implementation -- it's not like falling back to a DFA or other
    // non-repeat engine is going to succeed.
    if (tooLargeToImplement(rsi.repeatMin, rsi.repeatMax)) {
        throw CompileError("Pattern too large.");
    }

    return true;
}

static
bool allPredsInSubgraph(NFAVertex v, const NGHolder &g,
                        const unordered_set<NFAVertex> &involved) {
    for (auto u : inv_adjacent_vertices_range(v, g)) {
        if (!contains(involved, u)) {
            return false;
        }
    }
    return true;
}

static
void buildTugTrigger(NGHolder &g, NFAVertex cyclic, NFAVertex v,
                     const unordered_set<NFAVertex> &involved,
                     unordered_map<NFAVertex, NFAVertexDepth> &depths,
                     vector<NFAVertex> &tugs) {
    if (allPredsInSubgraph(v, g, involved)) {
        // We can transform this vertex into a tug trigger in-place.
        DEBUG_PRINTF("all preds in subgraph, vertex %zu becomes tug\n",
                     g[v].index);
        add_edge(cyclic, v, g);
        tugs.push_back(v);
        return;
    }

    // Some predecessors of v are not in the subgraph, so we need to clone v
    // and split up its in-edges.
    NFAVertex t = clone_vertex(g, v);
    depths[t] = depths[v];

    DEBUG_PRINTF("there are other paths, cloned tug %zu from vertex %zu\n",
                  g[t].index, g[v].index);

    tugs.push_back(t);
    add_edge(cyclic, t, g);

    // New vertex gets all of v's successors, including v itself if it's
    // cyclic.
    clone_out_edges(g, v, t);
}

static
NFAVertex createCyclic(NGHolder &g, ReachSubgraph &rsi) {
    NFAVertex last = rsi.vertices.back();
    NFAVertex cyclic = clone_vertex(g, last);
    add_edge(cyclic, cyclic, g);

    DEBUG_PRINTF("created cyclic vertex %zu\n", g[cyclic].index);
    return cyclic;
}

static
NFAVertex createPos(NGHolder &g, ReachSubgraph &rsi) {
    NFAVertex pos = add_vertex(g);
    NFAVertex first = rsi.vertices.front();

    g[pos].char_reach = g[first].char_reach;

    DEBUG_PRINTF("created pos vertex %zu\n", g[pos].index);
    return pos;
}

// 2 if v is directly connected to an accept, or 1 if one hop away,
// or 0 otherwise.
static
u32 isCloseToAccept(const NGHolder &g, NFAVertex v) {
    if (is_any_accept(v, g)) {
        return 2;
    }

    for (auto w : adjacent_vertices_range(v, g)) {
        if (is_any_accept(w, g)) {
            return 1;
        }
    }

    return 0;
}

static
u32 unpeelAmount(const NGHolder &g, const ReachSubgraph &rsi) {
    const NFAVertex last = rsi.vertices.back();
    u32 rv = 0;

    for (auto v : adjacent_vertices_range(last, g)) {
        rv = max(rv, isCloseToAccept(g, v));
    }

    return rv;
}

static
void unpeelNearEnd(NGHolder &g, ReachSubgraph &rsi,
                   unordered_map<NFAVertex, NFAVertexDepth> &depths,
                   vector<NFAVertex> *succs) {
    u32 unpeel = unpeelAmount(g, rsi);
    DEBUG_PRINTF("unpeeling %u vertices\n", unpeel);

    while (unpeel) {
        NFAVertex last = rsi.vertices.back();
        NFAVertex first = rsi.vertices.front();

        NFAVertex d = clone_vertex(g, last);
        depths[d] = depths[last];
        DEBUG_PRINTF("created vertex %zu\n", g[d].index);

        for (auto v : *succs) {
            add_edge(d, v, g);
        }

        if (rsi.repeatMin > depth(1)) {
            rsi.repeatMin -= 1;
        } else {
            /* Skip edge for the cyclic state; note that we must clone their
             * edge properties as they may include tops. */
            for (const auto &e : in_edges_range(first, g)) {
                add_edge(source(e, g), d, g[e], g);
            }
        }

        succs->clear();
        succs->push_back(d);

        rsi.repeatMax -= 1;

        assert(rsi.repeatMin > depth(0));
        assert(rsi.repeatMax > depth(0));

        unpeel--;
    }
}

/** Fetch the set of successor vertices of this subgraph. */
static
void getSuccessors(const NGHolder &g, const ReachSubgraph &rsi,
                   vector<NFAVertex> *succs) {
    assert(!rsi.vertices.empty());
    // Successors come from successors of last vertex.
    NFAVertex last = rsi.vertices.back();

    for (auto v : adjacent_vertices_range(last, g)) {
        if (v == last) { /* ignore self loop */
            continue;
        }
        succs->push_back(v);
    }
}

/** Disconnect the given subgraph from its predecessors and successors in the
 * NFA graph and replace it with a cyclic state. */
static
void replaceSubgraphWithSpecial(NGHolder &g, ReachSubgraph &rsi,
                               vector<BoundedRepeatData> *repeats,
                               unordered_map<NFAVertex, NFAVertexDepth> &depths,
                               unordered_set<NFAVertex> &created) {
    assert(!rsi.bad);
    /* As we may need to unpeel 2 vertices, we need the width to be more than 2.
     * This should only happen if the graph did not have redundancy pass
     * performed on as vertex count checks would be prevent us reaching here.
     */
    if (rsi.repeatMax <= depth(2)) {
        return;
    }
    assert(rsi.repeatMin > depth(0));
    assert(rsi.repeatMax >= rsi.repeatMin);
    assert(rsi.repeatMax > depth(2));

    DEBUG_PRINTF("entry\n");

    const unordered_set<NFAVertex> involved(rsi.vertices.begin(),
                                                 rsi.vertices.end());
    vector<NFAVertex> succs;
    getSuccessors(g, rsi, &succs);

    unpeelNearEnd(g, rsi, depths, &succs);

    // Create our replacement cyclic state with the same reachability and
    // report info as the last vertex in our topo-ordered list.
    NFAVertex cyclic = createCyclic(g, rsi);
    created.insert(cyclic);

    // One more special vertex is necessary: the positive trigger (same
    // reach as cyclic).
    NFAVertex pos_trigger = createPos(g, rsi);
    created.insert(pos_trigger);
    add_edge(pos_trigger, cyclic, g);

    // Update depths for our new vertices.
    NFAVertex first = rsi.vertices.front(), last = rsi.vertices.back();
    depths[pos_trigger] = depths[first];
    depths[cyclic].fromStart =
        unionDepthMinMax(depths[first].fromStart, depths[last].fromStart);
    depths[cyclic].fromStartDotStar = unionDepthMinMax(
        depths[first].fromStartDotStar, depths[last].fromStartDotStar);

    // Wire predecessors to positive trigger.
    for (const auto &e : in_edges_range(first, g)) {
        add_edge(source(e, g), pos_trigger, g[e], g);
    }

    // Wire cyclic state to tug trigger states built from successors.
    vector<NFAVertex> tugs;
    for (auto v : succs) {
        buildTugTrigger(g, cyclic, v, involved, depths, tugs);
    }
    created.insert(tugs.begin(), tugs.end());
    assert(!tugs.empty());

    // Wire pos trigger to tugs if min repeat is one -- this deals with cases
    // where we can get a pos and tug trigger on the same byte.
    if (rsi.repeatMin == depth(1)) {
        for (auto v : tugs) {
            add_edge(pos_trigger, v, g);
        }
    }

    // Remove the vertices/edges in the subgraph.
    remove_vertices(rsi.vertices, g, false);
    erase_all(&depths, rsi.vertices);

    repeats->push_back(BoundedRepeatData(rsi.historyType, rsi.repeatMin,
                                         rsi.repeatMax, rsi.minPeriod, cyclic,
                                         pos_trigger, tugs));
}

/** Variant for Rose-specific graphs that terminate in a sole accept, so we can
 * use a "lazy tug". See UE-1636. */
static
void replaceSubgraphWithLazySpecial(NGHolder &g, ReachSubgraph &rsi,
                          vector<BoundedRepeatData> *repeats,
                          unordered_map<NFAVertex, NFAVertexDepth> &depths,
                          unordered_set<NFAVertex> &created) {
    assert(!rsi.bad);
    assert(rsi.repeatMin);
    assert(rsi.repeatMax >= rsi.repeatMin);

    DEBUG_PRINTF("entry\n");

    const unordered_set<NFAVertex> involved(rsi.vertices.begin(),
                                            rsi.vertices.end());
    vector<NFAVertex> succs;
    getSuccessors(g, rsi, &succs);

    // Create our replacement cyclic state with the same reachability and
    // report info as the last vertex in our topo-ordered list.
    NFAVertex cyclic = createCyclic(g, rsi);
    created.insert(cyclic);

    // One more special vertex is necessary: the positive trigger (same
    // reach as cyclic).
    NFAVertex pos_trigger = createPos(g, rsi);
    created.insert(pos_trigger);
    add_edge(pos_trigger, cyclic, g);

    // Update depths for our new vertices.
    NFAVertex first = rsi.vertices.front(), last = rsi.vertices.back();
    depths[pos_trigger] = depths[first];
    depths[cyclic].fromStart =
        unionDepthMinMax(depths[first].fromStart, depths[last].fromStart);
    depths[cyclic].fromStartDotStar = unionDepthMinMax(
        depths[first].fromStartDotStar, depths[last].fromStartDotStar);

    // Wire predecessors to positive trigger.
    for (const auto &e : in_edges_range(first, g)) {
        add_edge(source(e, g), pos_trigger, g[e], g);
    }

    // In the rose case, our tug is our cyclic, and it's wired to our
    // successors (which should be just the accept).
    vector<NFAVertex> tugs;
    assert(succs.size() == 1);
    for (auto v : succs) {
        add_edge(cyclic, v, g);
    }

    // Wire pos trigger to accept if min repeat is one -- this deals with cases
    // where we can get a pos and tug trigger on the same byte.
    if (rsi.repeatMin == depth(1)) {
        for (auto v : succs) {
            add_edge(pos_trigger, v, g);
            g[pos_trigger].reports = g[cyclic].reports;
        }
    }

    // Remove the vertices/edges in the subgraph.
    remove_vertices(rsi.vertices, g, false);
    erase_all(&depths, rsi.vertices);

    repeats->push_back(BoundedRepeatData(rsi.historyType, rsi.repeatMin,
                                         rsi.repeatMax, rsi.minPeriod, cyclic,
                                         pos_trigger, tugs));
}

static
bool isCompBigEnough(const RepeatGraph &rg, const u32 minRepeat) {
    // filtered_graph doesn't filter the num_vertices call.
    size_t n = 0;
    RepeatGraph::vertex_iterator vi, ve;
    for (tie(vi, ve) = vertices(rg); vi != ve; ++vi) {
        if (++n >= minRepeat) {
            return true;
        }
    }
    return false;
}

// Marks the subgraph as bad if it can't be handled.
static
void reprocessSubgraph(const NGHolder &h, const Grey &grey,
                       ReachSubgraph &rsi) {
    vector<ReachSubgraph> rs(1, rsi);
    checkReachSubgraphs(h, rs, grey.minExtBoundedRepeatSize);
    if (rs.size() != 1) {
        DEBUG_PRINTF("subgraph split into %zu\n", rs.size());
        rsi.bad = true;
        return;
    }

    rsi = rs.back(); // Potentially modified.

    if (processSubgraph(h, rsi, grey.minExtBoundedRepeatSize)) {
        DEBUG_PRINTF("reprocessed subgraph is {%s,%s} repeat\n",
                     rsi.repeatMin.str().c_str(), rsi.repeatMax.str().c_str());
    } else {
        DEBUG_PRINTF("reprocessed subgraph is bad\n");
        rsi.bad = true;
    }
}

/** Remove vertices from the beginning and end of the vertex set that are
 * involved in other repeats as a result of earlier repeat transformations. */
static
bool peelSubgraph(const NGHolder &g, const Grey &grey, ReachSubgraph &rsi,
                  const unordered_set<NFAVertex> &created) {
    assert(!rsi.bad);

    if (created.empty()) {
        return true;
    }

    if (rsi.vertices.empty()) {
        return false;
    }

    // Peel involved vertices from the front.
    vector<NFAVertex>::iterator zap = rsi.vertices.end();
    for (auto it = rsi.vertices.begin(), ite = rsi.vertices.end(); it != ite;
         ++it) {
        if (!contains(created, *it)) {
            zap = it;
            break;
        } else {
            DEBUG_PRINTF("%zu is involved in another repeat\n", g[*it].index);
        }
    }
    DEBUG_PRINTF("peeling %zu vertices from front\n",
                 distance(rsi.vertices.begin(), zap));
    rsi.vertices.erase(rsi.vertices.begin(), zap);

    // Peel involved vertices and vertices with edges to involved vertices from
    // the back; otherwise we may try to transform a POS into a TUG.
    zap = rsi.vertices.begin();
    for (auto it = rsi.vertices.rbegin(), ite = rsi.vertices.rend(); it != ite;
         ++it) {
        if (!contains(created, *it) &&
            !contains_any_of(created, adjacent_vertices(*it, g))) {
            zap = it.base(); // Note: erases everything after it.
            break;
        } else {
            DEBUG_PRINTF("%zu is involved in another repeat\n", g[*it].index);
        }
    }
    DEBUG_PRINTF("peeling %zu vertices from back\n",
                 distance(zap, rsi.vertices.end()));
    rsi.vertices.erase(zap, rsi.vertices.end());

    // If vertices in the middle are involved in other repeats, it's a definite
    // no-no.
    for (auto v : rsi.vertices) {
        if (contains(created, v)) {
            DEBUG_PRINTF("vertex %zu is in another repeat\n", g[v].index);
            return false;
        }
    }

    reprocessSubgraph(g, grey, rsi);
    return !rsi.bad;
}

/** For performance reasons, it's nice not to have an exceptional state right
 * next to a startDs state: that way we can do double-byte accel, whereas
 * otherwise the NEG trigger would limit us to single. This might be a good
 * idea to extend to cyclic states, too. */
static
void peelStartDotStar(const NGHolder &g,
                      const unordered_map<NFAVertex, NFAVertexDepth> &depths,
                      const Grey &grey, ReachSubgraph &rsi) {
    if (rsi.vertices.size() < 1) {
        return;
    }

    NFAVertex first = rsi.vertices.front();
    if (depths.at(first).fromStartDotStar.min == depth(1)) {
        DEBUG_PRINTF("peeling start front vertex %zu\n", g[first].index);
        rsi.vertices.erase(rsi.vertices.begin());
        reprocessSubgraph(g, grey, rsi);
    }
}

static
void buildReachSubgraphs(const NGHolder &g, vector<ReachSubgraph> &rs,
                         const u32 minNumVertices) {
    const ReachFilter<NGHolder> fil(&g);
    const RepeatGraph rg(g, fil, fil);

    if (!isCompBigEnough(rg, minNumVertices)) {
        DEBUG_PRINTF("component not big enough, bailing\n");
        return;
    }

    const auto ug = make_undirected_graph(rg);

    unordered_map<NFAVertex, u32> repeatMap;

    unsigned int num;
    num = connected_components(ug, make_assoc_property_map(repeatMap));
    DEBUG_PRINTF("found %u connected repeat components\n", num);

    // Now, we build a set of topo-ordered ReachSubgraphs.
    vector<NFAVertex> topoOrder = buildTopoOrder(rg);

    rs.resize(num);

    for (auto v : topoOrder) {
        auto rit = repeatMap.find(v);
        if (rit == repeatMap.end()) {
            continue; /* not part of a repeat */
        }
        u32 comp_id = rit->second;
        assert(comp_id < num);
        rs[comp_id].vertices.push_back(v);
    }

#ifdef DEBUG
    for (size_t i = 0; i < rs.size(); i++) {
        DEBUG_PRINTF("rs %zu has %zu vertices.\n", i, rs[i].vertices.size());
    }
#endif
}

static
bool hasSkipEdges(const NGHolder &g, const ReachSubgraph &rsi) {
    assert(!rsi.vertices.empty());

    const NFAVertex first = rsi.vertices.front();
    const NFAVertex last = rsi.vertices.back();

    // All of the preds of first must have edges to all the successors of last.
    for (auto u : inv_adjacent_vertices_range(first, g)) {
        for (auto v : adjacent_vertices_range(last, g)) {
            if (!edge(u, v, g).second) {
                return false;
            }
        }
    }

    return true;
}

/* depth info is valid as calculated at entry */
static
bool entered_at_fixed_offset(NFAVertex v, const NGHolder &g,
            const unordered_map<NFAVertex, NFAVertexDepth> &depths,
            const unordered_set<NFAVertex> &reached_by_fixed_tops) {
    DEBUG_PRINTF("|reached_by_fixed_tops| %zu\n",
                  reached_by_fixed_tops.size());
    if (is_triggered(g) && !contains(reached_by_fixed_tops, v)) {
        /* can't do this for infix/suffixes unless we know trigger literals
         * can only occur at one offset */
        DEBUG_PRINTF("bad top(s) for %zu\n", g[v].index);
        return false;
    }

    if (depths.at(v).fromStartDotStar.min.is_reachable()) {
        DEBUG_PRINTF("reachable from startDs\n");
        return false;
    }

    /* look at preds as v may be cyclic */
    const depth &first = depths.at(v).fromStart.min;
    assert(first.is_reachable());
    if (!first.is_finite()) {
        DEBUG_PRINTF("first not finite\n");
        return false;
    }
    DEBUG_PRINTF("first is at least %s from start\n", first.str().c_str());

    for (auto u : inv_adjacent_vertices_range(v, g)) {
        const depth &u_max_depth = depths.at(u).fromStart.max;
        DEBUG_PRINTF("pred %zu max depth %s from start\n", g[u].index,
                     u_max_depth.str().c_str());
        if (u_max_depth != first - depth(1)) {
            return false;
        }
    }

    return true;
}

static
NFAVertex buildTriggerStates(NGHolder &g, const vector<CharReach> &trigger,
                             u32 top) {
    NFAVertex u = g.start;
    for (const auto &cr : trigger) {
        NFAVertex v = add_vertex(g);
        g[v].char_reach = cr;
        add_edge(u, v, g);
        if (u == g.start) {
            g[edge(u, v, g)].tops.insert(top);
        }
        u = v;
    }

    DEBUG_PRINTF("trigger len=%zu has sink %zu\n", trigger.size(), g[u].index);
    return u;
}

/**
 * For triggered graphs, replace the "top" edges from start with the triggers
 * they represent, for the purposes of determining sole entry.
 */
static
void addTriggers(NGHolder &g,
                 const map<u32, vector<vector<CharReach>>> &triggers) {
    if (!is_triggered(g)) {
        assert(triggers.empty());
        return;
    }

    vector<NFAEdge> dead;
    map<u32, vector<NFAVertex>> starts_by_top;

    for (const auto &e : out_edges_range(g.start, g)) {
        const NFAVertex &v = target(e, g);
        if (v == g.startDs) {
            continue;
        }

        const auto &tops = g[e].tops;

        // The caller may not have given us complete trigger information. If we
        // don't have any triggers for a particular top, we should just leave
        // it alone.
        for (u32 top : tops) {
            if (!contains(triggers, top)) {
                DEBUG_PRINTF("no triggers for top %u\n", top);
                goto next_edge;
            }

            starts_by_top[top].push_back(v);
        }
        dead.push_back(e);
    next_edge:;
    }

    remove_edges(dead, g);

    for (const auto &m : starts_by_top) {
        const auto &top = m.first;
        const auto &starts = m.second;

        assert(contains(triggers, top));
        const auto &top_triggers = triggers.at(top);

        for (const auto &trigger : top_triggers) {
            NFAVertex u = buildTriggerStates(g, trigger, top);
            for (const auto &v : starts) {
                add_edge_if_not_present(u, v, g);
            }
        }
    }
}

static
CharReach predReach(const NGHolder &g, NFAVertex v) {
    CharReach cr;
    for (auto u : inv_adjacent_vertices_range(v, g)) {
        cr |= g[u].char_reach;
    }
    return cr;
}

/**
 * Filter the given vertex map (which maps from vertices in another graph to
 * vertices in subg) so that it only contains vertices that actually exist in
 * subg.
 */
static
void filterMap(const NGHolder &subg,
               unordered_map<NFAVertex, NFAVertex> &vmap) {
    NGHolder::vertex_iterator vi, ve;
    tie(vi, ve) = vertices(subg);
    const unordered_set<NFAVertex> remaining_verts(vi, ve);

    unordered_map<NFAVertex, NFAVertex> fmap; // filtered map

    for (const auto &m : vmap) {
        if (contains(remaining_verts, m.second)) {
            fmap.insert(m);
        }
    }

    vmap.swap(fmap);
}

/** Construct a graph for sole entry analysis that only considers paths through
 * the bounded repeat. */
static
void buildRepeatGraph(NGHolder &rg,
                      unordered_map<NFAVertex, NFAVertex> &rg_map,
                      const NGHolder &g, const ReachSubgraph &rsi,
                      const map<u32, vector<vector<CharReach>>> &triggers) {
    cloneHolder(rg, g, &rg_map);
    assert(rg.kind == g.kind);

    clear_in_edges(rg.accept, rg);
    clear_in_edges(rg.acceptEod, rg);
    add_edge(rg.accept, rg.acceptEod, rg);

    // Find the set of vertices in rg involved in the repeat.
    unordered_set<NFAVertex> rg_involved;
    for (const auto &v : rsi.vertices) {
        assert(contains(rg_map, v));
        rg_involved.insert(rg_map.at(v));
    }

    // Remove all out-edges from repeat vertices that aren't to other repeat
    // vertices, then connect terminal repeat vertices to accept.
    for (const auto &v : rsi.vertices) {
        NFAVertex rv = rg_map.at(v);
        remove_out_edge_if(rv, [&](const NFAEdge &e) {
            return !contains(rg_involved, target(e, rg));
        }, rg);
        if (!has_successor(rv, rg)) { // no interior out-edges
            add_edge(rv, rg.accept, rg);
        }
    }

    pruneUseless(rg);

    if (is_triggered(rg)) {
        // Add vertices for all our triggers
        addTriggers(rg, triggers);
        renumber_vertices(rg);

        // We don't know anything about how often this graph is triggered, so we
        // make the start vertex cyclic for the purposes of this analysis ONLY.
        add_edge(rg.start, rg.start, rg);
    }

    filterMap(rg, rg_map);

    // All of our repeat vertices should have vertices in rg.
    assert(all_of(begin(rsi.vertices), end(rsi.vertices),
                  [&](const NFAVertex &v) { return contains(rg_map, v); }));
}

/**
 * Construct an input DAG which accepts on all entries to the repeat.
 */
static
void buildInputGraph(NGHolder &lhs,
                     unordered_map<NFAVertex, NFAVertex> &lhs_map,
                     const NGHolder &g, const NFAVertex first,
                     const map<u32, vector<vector<CharReach>>> &triggers) {
    DEBUG_PRINTF("building lhs with first=%zu\n", g[first].index);
    cloneHolder(lhs, g, &lhs_map);
    assert(g.kind == lhs.kind);
    addTriggers(lhs, triggers);
    renumber_vertices(lhs);

    // Replace each back-edge (u,v) with an edge (startDs,v), which will
    // generate entries at at least the rate of the loop created by that
    // back-edge.
    set<NFAEdge> dead;
    BackEdges<set<NFAEdge> > backEdgeVisitor(dead);
    depth_first_search(lhs, visitor(backEdgeVisitor).root_vertex(lhs.start));
    for (const auto &e : dead) {
        const NFAVertex u = source(e, lhs), v = target(e, lhs);
        if (u == v) {
            continue; // Self-loops are OK.
        }

        DEBUG_PRINTF("replacing back-edge (%zu,%zu) with edge (startDs,%zu)\n",
                     lhs[u].index, lhs[v].index, lhs[v].index);

        add_edge_if_not_present(lhs.startDs, v, lhs);
        remove_edge(e, lhs);
    }

    clear_in_edges(lhs.accept, lhs);
    clear_in_edges(lhs.acceptEod, lhs);
    add_edge(lhs.accept, lhs.acceptEod, lhs);

    // Wire the predecessors of the first repeat vertex to accept, then prune.
    NFAVertex lhs_first = lhs_map.at(first);
    for (auto u : inv_adjacent_vertices_range(lhs_first, lhs)) {
        add_edge_if_not_present(u, lhs.accept, lhs);
    }

    pruneUseless(lhs);
    filterMap(lhs, lhs_map);
}

/**
 * Maximum number of vertices in the input DAG to actually allow sole entry
 * calculation (as very large cases make sentClearsTail take a long, long time
 * to complete.)
 */
static const size_t MAX_SOLE_ENTRY_VERTICES = 10000;

/** True if (1) fixed offset or (2) reentries to this subgraph must involve a
 * character which escapes the repeat, meaning that we only need to store a
 * single offset at runtime. See UE-1361. */
static
bool hasSoleEntry(const NGHolder &g, const ReachSubgraph &rsi,
                  const unordered_map<NFAVertex, NFAVertexDepth> &depths,
                  const unordered_set<NFAVertex> &reached_by_fixed_tops,
                  const map<u32, vector<vector<CharReach>>> &triggers) {
    DEBUG_PRINTF("checking repeat {%s,%s}\n", rsi.repeatMin.str().c_str(),
                 rsi.repeatMax.str().c_str());
    NFAVertex first = rsi.vertices.front();
    const CharReach &repeatReach = g[first].char_reach;

    /* trivial case first is at a fixed depth */
    if (entered_at_fixed_offset(first, g, depths, reached_by_fixed_tops)) {
        DEBUG_PRINTF("fixed depth\n");
        return true;
    }

    DEBUG_PRINTF("repeat reach is %s\n", describeClass(repeatReach).c_str());

    // Nothing can escape a dot repeat.
    if (repeatReach.all()) {
        DEBUG_PRINTF("dot repeat cannot be escaped\n");
        return false;
    }

    // Another easy case: if the union of the reach of all entries to the
    // repeat will always escape the repeat, we have sole entry.
    if (predReach(g, first).isSubsetOf(~repeatReach)) {
        DEBUG_PRINTF("pred reach %s, which is subset of repeat escape\n",
                     describeClass(predReach(g, first)).c_str());
        return true;
    }

    NGHolder rg;
    unordered_map<NFAVertex, NFAVertex> rg_map;
    buildRepeatGraph(rg, rg_map, g, rsi, triggers);
    assert(rg.kind == g.kind);

    NGHolder lhs;
    unordered_map<NFAVertex, NFAVertex> lhs_map;
    buildInputGraph(lhs, lhs_map, g, first, triggers);
    assert(lhs.kind == g.kind);

    if (num_vertices(lhs) > MAX_SOLE_ENTRY_VERTICES) {
        DEBUG_PRINTF("too many vertices (%zu) for sole entry test.\n",
                     num_vertices(lhs));
        return false;
    }

    // Split the repeat graph into two regions: vertices in the LHS input DAG
    // are in one region, vertices in the bounded repeat are in another.
    const u32 lhs_region = 1;
    const u32 repeat_region = 2;
    unordered_map<NFAVertex, u32> region_map;

    for (const auto &v : rsi.vertices) {
        assert(!is_special(v, g)); // no specials in repeats
        assert(contains(rg_map, v));
        DEBUG_PRINTF("rg vertex %zu in repeat\n", rg[rg_map.at(v)].index);
        region_map.emplace(rg_map.at(v), repeat_region);
    }

    for (const auto &v : vertices_range(rg)) {
        if (!contains(region_map, v)) {
            DEBUG_PRINTF("rg vertex %zu in lhs (trigger)\n", rg[v].index);
            region_map.emplace(v, lhs_region);
        }
    }

    u32 bad_region = 0;
    if (sentClearsTail(rg, region_map, lhs, lhs_region, &bad_region)) {
        DEBUG_PRINTF("input dag clears repeat: sole entry\n");
        return true;
    }

    DEBUG_PRINTF("not sole entry\n");
    return false;
}

namespace {

template<class Graph>
struct StrawWalker {
    StrawWalker(const NGHolder &h_in, const Graph &g_in,
                const vector<BoundedRepeatData> &all_repeats)
        : h(h_in), g(g_in), repeats(all_repeats) {}

    /** True if v is a cyclic that belongs to a bounded repeat (one without an
     * inf max bound). */
    bool isBoundedRepeatCyclic(NFAVertex v) const {
        for (const auto &r : repeats) {
            if (r.repeatMax.is_finite() && r.cyclic == v) {
                return true;
            }
        }
        return false;
    }

    NFAVertex step(NFAVertex v) const {
        typename Graph::adjacency_iterator ai, ae;
        tie(ai, ae) = adjacent_vertices(v, g);
        assert(ai != ae);
        NFAVertex next = *ai;
        if (next == v) { // Ignore self loop.
            ++ai;
            if (ai == ae) {
                return NGHolder::null_vertex();
            }
            next = *ai;
        }
        ++ai;
        if (ai != ae && *ai == v) { // Ignore self loop
            ++ai;
        }
        if (ai != ae) {
            DEBUG_PRINTF("more than one succ\n");
            set<NFAVertex> succs;
            insert(&succs, adjacent_vertices(v, g));
            succs.erase(v);
            for (tie(ai, ae) = adjacent_vertices(v, g); ai != ae; ++ai) {
                next = *ai;
                DEBUG_PRINTF("checking %zu\n", g[next].index);
                if (next == v) {
                    continue;
                }
                set<NFAVertex> lsuccs;
                insert(&lsuccs, adjacent_vertices(next, g));

                if (lsuccs != succs) {
                    continue;
                }

                // Ensure that if v is in connected to accept, the reports
                // on `next` much match.
                if (is_match_vertex(v, h) && g[v].reports != g[next].reports) {
                    DEBUG_PRINTF("report mismatch\n");
                    continue;
                }

                return next;
            }
            DEBUG_PRINTF("bailing\n");
            return NGHolder::null_vertex();
        }
        return next;
    }

    NFAVertex walk(NFAVertex v, vector<NFAVertex> &straw) const {
        DEBUG_PRINTF("walk from %zu\n", g[v].index);
        unordered_set<NFAVertex> visited;
        straw.clear();

        while (!is_special(v, g)) {
            DEBUG_PRINTF("checking %zu\n", g[v].index);
            NFAVertex next = step(v);
            if (next == NGHolder::null_vertex()) {
                break;
            }
            if (!visited.insert(next).second) {
                DEBUG_PRINTF("already visited %zu, bailing\n", g[next].index);
                break; /* don't want to get stuck in any complicated loops */
            }

            const CharReach &reach_v = g[v].char_reach;
            const CharReach &reach_next = g[next].char_reach;
            if (!reach_v.isSubsetOf(reach_next)) {
                DEBUG_PRINTF("%zu's reach is not a superset of %zu's\n",
                             g[next].index, g[v].index);
                break;
            }

            // If this is cyclic with the right reach, we're done. Note that
            // startDs fulfils this requirement.
            if (hasSelfLoop(next, g) && !isBoundedRepeatCyclic(next)) {
                DEBUG_PRINTF("found cyclic %zu\n", g[next].index);
                return next;
            }

            v = next;
            straw.push_back(v);
        }

        straw.clear();
        return NGHolder::null_vertex();
    }

private:
    const NGHolder &h; // underlying graph
    const Graph &g;
    const vector<BoundedRepeatData> &repeats;
};

} // namespace

static
NFAVertex walkStrawToCyclicRev(const NGHolder &g, NFAVertex v,
                               const vector<BoundedRepeatData> &all_repeats,
                               vector<NFAVertex> &straw) {
    typedef boost::reverse_graph<NGHolder, const NGHolder &> RevGraph;
    const RevGraph revg(g);

    auto cyclic = StrawWalker<RevGraph>(g, revg, all_repeats).walk(v, straw);
    reverse(begin(straw), end(straw)); // path comes from cyclic
    return cyclic;
}

static
NFAVertex walkStrawToCyclicFwd(const NGHolder &g, NFAVertex v,
                               const vector<BoundedRepeatData> &all_repeats,
                               vector<NFAVertex> &straw) {
    return StrawWalker<NGHolder>(g, g, all_repeats).walk(v, straw);
}

/** True if entries to this subgraph must pass through a cyclic state with
 * reachability that is a superset of the reach of the repeat, and
 * reachabilities along this path "nest" into the reaches of their
 * predecessors.
 *
 * This is what is called a 'straw' in the region code. */
static
bool hasCyclicSupersetEntryPath(const NGHolder &g, const ReachSubgraph &rsi,
                                const vector<BoundedRepeatData> &all_repeats) {
    // Cope with peeling by following a chain of single vertices backwards
    // until we encounter our cyclic, all of which must have superset reach.
    vector<NFAVertex> straw;
    return walkStrawToCyclicRev(g, rsi.vertices.front(), all_repeats, straw) !=
           NGHolder::null_vertex();
}

static
bool hasCyclicSupersetExitPath(const NGHolder &g, const ReachSubgraph &rsi,
                               const vector<BoundedRepeatData> &all_repeats) {
    vector<NFAVertex> straw;
    return walkStrawToCyclicFwd(g, rsi.vertices.back(), all_repeats, straw) !=
           NGHolder::null_vertex();
}

static
bool leadsOnlyToAccept(const NGHolder &g, const ReachSubgraph &rsi) {
    const NFAVertex u = rsi.vertices.back();
    for (auto v : adjacent_vertices_range(u, g)) {
        if (v != g.accept) {
            return false;
        }
    }
    assert(out_degree(u, g));
    return true;
}

static
bool allSimpleHighlander(const ReportManager &rm,
                         const flat_set<ReportID> &reports) {
    assert(!reports.empty());
    for (auto report : reports) {
        if (!isSimpleExhaustible(rm.getReport(report))) {
            return false;
        }
    }

    return true;
}

// Finds a single, fairly unrefined trigger for the repeat by walking backwards
// and collecting the unioned reach at each step.
static
vector<CharReach> getUnionedTrigger(const NGHolder &g, const NFAVertex v) {
    const size_t MAX_TRIGGER_STEPS = 32;

    vector<CharReach> trigger;

    flat_set<NFAVertex> curr, next;
    insert(&curr, inv_adjacent_vertices(v, g));

    if (contains(curr, g.start)) {
        DEBUG_PRINTF("start in repeat's immediate preds\n");
        trigger.push_back(CharReach::dot()); // Trigger could be anything!
        return trigger;
    }

    for (size_t num_steps = 0; num_steps < MAX_TRIGGER_STEPS; num_steps++) {
        next.clear();
        trigger.push_back(CharReach());
        CharReach &cr = trigger.back();

        for (auto v_c : curr) {
            cr |= g[v_c].char_reach;
            insert(&next, inv_adjacent_vertices(v_c, g));
        }

        DEBUG_PRINTF("cr[%zu]=%s\n", num_steps, describeClass(cr).c_str());

        if (next.empty() || contains(next, g.start)) {
            break;
        }

        curr.swap(next);
    }

    reverse(trigger.begin(), trigger.end());
    return trigger;
}

static
vector<vector<CharReach>> getRepeatTriggers(const NGHolder &g,
                                            const NFAVertex sink) {
    const size_t MAX_TRIGGER_STEPS = 32;
    const size_t UNIONED_FALLBACK_THRESHOLD = 100;

    using Path = deque<NFAVertex>;

    vector<vector<CharReach>> triggers;

    deque<Path> q; // work queue
    deque<Path> done; // finished paths

    size_t max_len = MAX_TRIGGER_STEPS;

    // Find a set of paths leading to vertex v by depth first search.

    for (auto u : inv_adjacent_vertices_range(sink, g)) {
        if (is_any_start(u, g)) {
            triggers.push_back({}); // empty
            return triggers;
        }
        q.push_back(Path(1, u));
    }

    while (!q.empty()) {
        Path &path = q.front();
        NFAVertex v = path.back();

        if (path.size() >= max_len) {
            max_len = min(max_len, path.size());
            done.push_back(path);
            goto next_path;
        }

        for (auto u : inv_adjacent_vertices_range(v, g)) {
            if (is_any_start(u, g)) {
                // Found an accept. There's no point expanding this path any
                // further, we're done.
                max_len = min(max_len, path.size());
                done.push_back(path);
                goto next_path;
            }

            if (path.size() + 1 >= max_len) {
                done.push_back(path);
                done.back().push_back(u);
            } else {
                q.push_back(path); // copy
                q.back().push_back(u);
            }
        }

    next_path:
        q.pop_front();

        // If our queue or our finished trigger list gets too large, fall back
        // to generating a single trigger with union reach.
        if (q.size() + done.size() > UNIONED_FALLBACK_THRESHOLD) {
            DEBUG_PRINTF("search too large, fall back to union trigger\n");
            triggers.clear();
            triggers.push_back(getUnionedTrigger(g, sink));
            return triggers;
        }
    }

    assert(!done.empty());

    // Convert our path list into a set of unique triggers.
    ue2_unordered_set<vector<CharReach>> unique_triggers;
    for (const auto &path : done) {
        vector<CharReach> reach_path;
        for (auto jt = path.rbegin(), jte = path.rend(); jt != jte; ++jt) {
            reach_path.push_back(g[*jt].char_reach);
        }
        unique_triggers.insert(reach_path);
    }

    insert(&triggers, triggers.end(), unique_triggers);
    sort(triggers.begin(), triggers.end());
    DEBUG_PRINTF("built %zu unique triggers, max_len=%zu\n", triggers.size(),
                 max_len);
    return triggers;
}

static
void findMinPeriod(const NGHolder &g,
                   const map<u32, vector<vector<CharReach>>> &triggers,
                   ReachSubgraph &rsi) {
    const auto v = rsi.vertices.front();
    const CharReach &cr = g[v].char_reach;

    vector<vector<CharReach>> repeat_triggers;

    if (is_triggered(g)) {
        // Construct a temporary copy of the graph that also contains its
        // triggers, potentially lengthening the repeat's triggers.
        NGHolder tg;
        unordered_map<NFAVertex, NFAVertex> tg_map;
        cloneHolder(tg, g, &tg_map);
        addTriggers(tg, triggers);
        assert(contains(tg_map, v));
        repeat_triggers = getRepeatTriggers(tg, tg_map.at(v));
    } else {
        // Not triggered, no need to mutate the graph.
        repeat_triggers = getRepeatTriggers(g, v);
    }

    rsi.minPeriod = minPeriod(repeat_triggers, cr, &rsi.is_reset);
    DEBUG_PRINTF("%zu triggers, minPeriod=%u, is_reset=%d\n",
                 repeat_triggers.size(), rsi.minPeriod, (int)rsi.is_reset);
}

static
void
selectHistoryScheme(const NGHolder &g, const ReportManager *rm,
                    ReachSubgraph &rsi,
                    const unordered_map<NFAVertex, NFAVertexDepth> &depths,
                    const unordered_set<NFAVertex> &reached_by_fixed_tops,
                    const map<u32, vector<vector<CharReach>>> &triggers,
                    const vector<BoundedRepeatData> &all_repeats,
                    const bool simple_model_selection) {
    // {N,} cases use the FIRST history mechanism.
    if (rsi.repeatMax.is_infinite()) {
        DEBUG_PRINTF("selected FIRST history\n");
        rsi.historyType = REPEAT_FIRST;
        return;
    }

    /* If we have a repeat which only raises a highlander, only the first match
     * matters */
    if (rm && leadsOnlyToAccept(g, rsi)
        && allSimpleHighlander(*rm, g[rsi.vertices.back()].reports)) {
        DEBUG_PRINTF("selected FIRST history (as highlander)\n");
        rsi.historyType = REPEAT_FIRST;
        rsi.repeatMax = depth::infinity(); /* for consistency */
        return;
    }

    // {N,M} cases can use the FIRST mechanism if they follow a cyclic which
    // includes their reachability via a "straw" path. (see UE-1589)
    if (hasCyclicSupersetEntryPath(g, rsi, all_repeats)) {
        DEBUG_PRINTF("selected FIRST history due to cyclic pred with "
                     "superset of reach\n");
        rsi.historyType = REPEAT_FIRST;
        rsi.repeatMax = depth::infinity(); /* will continue to pump out matches */
        return;
    }

    // Similarly, {N,M} cases can use the FIRST mechanism if they precede a
    // cyclic which includes their reachability via a "straw" path.
    if (hasCyclicSupersetExitPath(g, rsi, all_repeats)) {
        DEBUG_PRINTF("selected FIRST history due to cyclic succ with "
                     "superset of reach\n");
        rsi.historyType = REPEAT_FIRST;
        rsi.repeatMax = depth::infinity(); /* will continue to pump out matches */
        return;
    }

    // Could have skip edges and therefore be a {0,N} repeat.
    if (rsi.repeatMin == depth(1) && hasSkipEdges(g, rsi)) {
        DEBUG_PRINTF("selected LAST history\n");
        rsi.historyType = REPEAT_LAST;
        return;
    }

    // Fill minPeriod, is_reset flags
    findMinPeriod(g, triggers, rsi);

    // If we can't re-enter this cyclic state, we have a reset case.
    // This check can be very expensive, so we don't do it if we've been asked
    // for simple model selection.
    if (!simple_model_selection && !rsi.is_reset &&
        hasSoleEntry(g, rsi, depths, reached_by_fixed_tops, triggers)) {
        DEBUG_PRINTF("repeat is sole entry -> reset\n");
        rsi.is_reset = true;
    }

    // We can lean on the common selection code for the remainder of our repeat
    // models.
    rsi.historyType = chooseRepeatType(rsi.repeatMin, rsi.repeatMax,
                                       rsi.minPeriod, rsi.is_reset);
}

static
void buildFeeder(NGHolder &g, const BoundedRepeatData &rd,
                 unordered_set<NFAVertex> &created,
                 const vector<NFAVertex> &straw) {
    if (!g[rd.cyclic].char_reach.all()) {
        // Create another cyclic feeder state with flipped reach.  It has an
        // edge from the repeat's cyclic state and pos_trigger, an edge to the
        // straw, and edges from every vertex along the straw.
        NFAVertex feeder = clone_vertex(g, rd.cyclic);
        created.insert(feeder);
        g[feeder].char_reach.flip();
        add_edge(feeder, feeder, g);
        add_edge(rd.pos_trigger, feeder, g);
        add_edge(rd.cyclic, feeder, g);
        add_edge(feeder, straw.front(), g);

        // An edge from every vertex in the straw.
        for (auto v : straw) {
            add_edge(v, feeder, g);
        }

        // An edge to the feeder from the first vertex in the straw and all of
        // its predecessors (other than the feeder itself, we've already
        // created that edge!)
        for (auto u : inv_adjacent_vertices_range(straw.front(), g)) {
            if (u == feeder) {
                continue;
            }
            add_edge(u, feeder, g);
        }

        DEBUG_PRINTF("added feeder %zu\n", g[feeder].index);
    } else {
        // No neg trigger means feeder is empty, and unnecessary.
        assert(g[rd.pos_trigger].char_reach.all());
    }
}

/**
 * If we have a leading first repeat, we can split startDs so that it is not
 * cyclic so that the repeat is only triggered once, rather than every byte. If we
 * perform this transform we must create another cyclic state to retrigger the
 * repeat after we see an escape for the repeat.
 *
 * We do not use the anchored start state to allow us to restart the NFA at a deep
 * offset.
 */
static
bool improveLeadingRepeat(NGHolder &g, BoundedRepeatData &rd,
                          unordered_set<NFAVertex> &created,
                          const vector<BoundedRepeatData> &all_repeats) {
    assert(edge(g.startDs, g.startDs, g).second);

    // UE-1617: can rewire FIRST history cases that are preceded by
    // startDs.
    if (rd.type != REPEAT_FIRST) {
        return false;
    }

    const CharReach &cyc_cr = g[rd.cyclic].char_reach;

    // This transformation is only worth doing if this would allow us to
    // accelerate the cyclic state (UE-2055).
    if ((~cyc_cr).count() > ACCEL_MAX_STOP_CHAR) {
        DEBUG_PRINTF("we wouldn't be able to accel this case\n");
        return false;
    }

    vector<NFAVertex> straw;
    NFAVertex pred =
        walkStrawToCyclicRev(g, rd.pos_trigger, all_repeats, straw);
    if (pred != g.startDs) {
        DEBUG_PRINTF("straw walk doesn't lead to startDs\n");
        return false;
    }

    // This transformation is only safe if the straw path from startDs that
    // we've discovered can *only* lead to this repeat, since we're going to
    // remove the self-loop on startDs.
    if (proper_out_degree(g.startDs, g) > 1) {
        DEBUG_PRINTF("startDs has other successors\n");
        return false;
    }
    for (const auto &v : straw) {
        if (proper_out_degree(v, g) != 1) {
            DEBUG_PRINTF("branch between startDs and repeat, from vertex %zu\n",
                         g[v].index);
            return false;
        }
    }

    if (g[rd.pos_trigger].char_reach.count() < ACCEL_MAX_STOP_CHAR) {
        DEBUG_PRINTF("entry is narrow, could be accelerable\n");
        return false;
    }

    assert(!straw.empty());

    /* If there is overlap between the feeder and the first vertex in the straw
     * fun things happen. TODO: handle fun things happening (requires more
     * edges and more vertices). */
    if (!g[straw.front()].char_reach.isSubsetOf(cyc_cr)) {
        DEBUG_PRINTF("straw has `interesting' reach\n");
        return false;
    }

    DEBUG_PRINTF("repeat can be improved by removing startDs loop!\n");

    // Remove the self-loop on startDs! What a blast!
    remove_edge(g.startDs, g.startDs, g);

    // Wire up feeder state to straw.
    buildFeeder(g, rd, created, straw);

    return true;
}

static
vector<NFAVertex> makeOwnStraw(NGHolder &g, BoundedRepeatData &rd,
                               const vector<NFAVertex> &straw) {
    // Straw runs from startDs to our pos trigger.
    assert(!straw.empty());
    assert(edge(g.startDs, straw.front(), g).second);
    assert(edge(straw.back(), rd.pos_trigger, g).second);

    vector<NFAVertex> own_straw;
    for (const auto &v : straw) {
        NFAVertex v2 = clone_vertex(g, v);
        if (hasSelfLoop(v, g)) {
            add_edge(v2, v2, g);
        }
        if (!own_straw.empty()) {
            add_edge(own_straw.back(), v2, g);
        }
        own_straw.push_back(v2);
    }

    // Wire our straw to start, not startDs.
    add_edge(g.start, own_straw.front(), g);

    // Swap over to using our own straw to get to the POS trigger.
    remove_edge(straw.back(), rd.pos_trigger, g);
    add_edge(own_straw.back(), rd.pos_trigger, g);

    return own_straw;
}

/**
 * Specialized version of improveLeadingRepeat for outfixes, in which we can
 * rewire the straw to start instead of removing the startDs self-loop.
 */
static
bool improveLeadingRepeatOutfix(NGHolder &g, BoundedRepeatData &rd,
                                unordered_set<NFAVertex> &created,
                                const vector<BoundedRepeatData> &all_repeats) {
    assert(g.kind == NFA_OUTFIX);

    // UE-1617: can rewire FIRST history cases that are preceded by
    // startDs.
    if (rd.type != REPEAT_FIRST) {
        return false;
    }

    const CharReach &cyc_cr = g[rd.cyclic].char_reach;

    // This transformation is only worth doing if this would allow us to
    // accelerate the cyclic state (UE-2055).
    if ((~cyc_cr).count() > ACCEL_MAX_STOP_CHAR) {
        DEBUG_PRINTF("we wouldn't be able to accel this case\n");
        return false;
    }

    vector<NFAVertex> straw;
    NFAVertex pred =
        walkStrawToCyclicRev(g, rd.pos_trigger, all_repeats, straw);
    if (pred != g.startDs) {
        DEBUG_PRINTF("straw walk doesn't lead to startDs\n");
        return false;
    }

    if (g[rd.pos_trigger].char_reach.count() < ACCEL_MAX_STOP_CHAR) {
        DEBUG_PRINTF("entry is narrow, could be accelerable\n");
        return false;
    }

    assert(!straw.empty());

    /* If there is overlap between the feeder and the first vertex in the straw
     * fun things happen. TODO: handle fun things happening (requires more
     * edges and more vertices). */
    if (!g[straw.front()].char_reach.isSubsetOf(cyc_cr)) {
        DEBUG_PRINTF("straw has `interesting' reach\n");
        return false;
    }

    DEBUG_PRINTF("repeat can be improved by rebuilding its entry\n");

    const auto own_straw = makeOwnStraw(g, rd, straw);
    insert(&created, own_straw);

    // Wire up feeder state to our new straw.
    buildFeeder(g, rd, created, own_straw);

    // We may no longer need the original straw.
    pruneUseless(g);

    return true;
}

/** Returns true if doing the bounded repeat transformation on this case
 * results in a smaller NFA model. */
static
bool givesBetterModel(const NGHolder &g, const vector<ReachSubgraph> &rs) {
    static const u32 MAX_FAST_STATES = 128; // bigger NFAs are fat and slow.

    // We use vertex count as an upper bound for the number of states.
    u32 curr_states = num_vertices(g) - 2; // accepts don't have states

    if (curr_states <= MAX_FAST_STATES) {
        return false;
    }
    if (curr_states > NFA_MAX_STATES) {
        return true;
    }

    u32 expected_states = curr_states;
    for (const auto &rsi : rs) {
        /* may be off as unpeeling not done yet */
        expected_states += 2; /* cyclic and pos */
        expected_states -= rsi.vertices.size();
    }

    return ROUNDUP_N(curr_states, 128) != ROUNDUP_N(expected_states, 128);
}

/** True if this repeat terminates with a vertex that leads only to accept. */
static
bool endsInAccept(const NGHolder &g, const ReachSubgraph &rsi) {
    NFAVertex last = rsi.vertices.back();
    return getSoleDestVertex(g, last) == g.accept;
}

static
bool endsInAcceptEod(const NGHolder &g, const ReachSubgraph &rsi) {
    NFAVertex last = rsi.vertices.back();
    return getSoleDestVertex(g, last) == g.acceptEod;
}

namespace {
class pfti_visitor : public boost::default_dfs_visitor {
public:
    pfti_visitor(unordered_map<NFAVertex, depth> &top_depths_in,
                 const depth &our_depth_in)
        : top_depths(top_depths_in), our_depth(our_depth_in) {}

    void discover_vertex(NFAVertex v, UNUSED const NGHolder &g) {
        DEBUG_PRINTF("discovered %zu (depth %s)\n", g[v].index,
                     our_depth.str().c_str());

        auto it = top_depths.find(v);
        if (it != top_depths.end() && it->second != our_depth) {
            // already seen at a different depth, remove from consideration.
            it->second = depth::infinity();
        } else {
            top_depths[v] = our_depth;
        }
    }
    unordered_map<NFAVertex, depth> &top_depths;
    const depth &our_depth;
};
} // namespace

static
void populateFixedTopInfo(const map<u32, u32> &fixed_depth_tops,
                          const NGHolder &g,
                          unordered_set<NFAVertex> *reached_by_fixed_tops) {
    if (fixed_depth_tops.empty()) {
        return; /* we will never find anything */
    }

    assert(!proper_out_degree(g.startDs, g));
    unordered_map<NFAVertex, depth> top_depths;
    auto colours = make_small_color_map(g);

    for (const auto &e : out_edges_range(g.start, g)) {
        NFAVertex v = target(e, g);
        if (v == g.startDs) {
            continue;
        }

        depth td = depth::infinity();
        for (u32 top : g[e].tops) {
            if (!contains(fixed_depth_tops, top)) {
                td = depth::infinity();
                break;
            }
            depth td_t(fixed_depth_tops.at(top));
            if (td == td_t) {
                continue;
            } else if (td == depth::infinity()) {
                td = td_t;
            } else {
                td = depth::infinity();
                break;
            }
        }

        DEBUG_PRINTF("scanning from %zu depth=%s\n", g[v].index,
                     td.str().c_str());
        /* for each vertex reachable from v update its map to reflect that it is
         * reachable from a top of depth td. */

        depth_first_visit(g, v, pfti_visitor(top_depths, td), colours);
    }

    for (const auto &v_depth : top_depths) {
        const NFAVertex v = v_depth.first;
        const depth &d = v_depth.second;
        if (d.is_finite()) {
            DEBUG_PRINTF("%zu reached by fixed tops at depth %s\n",
                         g[v].index, d.str().c_str());
            reached_by_fixed_tops->insert(v);
        }
    }
}

#ifndef NDEBUG
/** Assertion use only. Returns true if the given bounded repeats share any
 * vertices, which we don't allow. */
static
bool hasOverlappingRepeats(UNUSED const NGHolder &g,
                           const vector<BoundedRepeatData> &repeats) {
    unordered_set<NFAVertex> involved;

    for (const auto &br : repeats) {
        if (contains(involved, br.cyclic)) {
            DEBUG_PRINTF("already seen cyclic %zu\n", g[br.cyclic].index);
            return true;
        }
        if (contains(involved, br.pos_trigger)) {
            DEBUG_PRINTF("already seen pos %zu\n", g[br.pos_trigger].index);
            return true;
        }
        for (auto v : br.tug_triggers) {
            if (contains(involved, v)) {
                DEBUG_PRINTF("already seen tug %zu\n", g[v].index);
                return true;
            }
        }

        involved.insert(br.cyclic);
        involved.insert(br.pos_trigger);
        involved.insert(br.tug_triggers.begin(), br.tug_triggers.end());
    }

    return false;
}

#endif // NDEBUG

/**
 * Identifies so-called "nasty" repeats, in which the reachability of both the
 * repeat itself and its tugs are wide, which means that executing the NFA will
 * likely be bogged down in exception processing.
 */
static
bool repeatIsNasty(const NGHolder &g, const ReachSubgraph &rsi,
                   const unordered_map<NFAVertex, NFAVertexDepth> &depths) {
    if (num_vertices(g) > NFA_MAX_STATES) {
        // We may have no choice but to implement this repeat to get the graph
        // down to a tractable number of vertices.
        return false;
    }

    if (!generates_callbacks(g) && endsInAccept(g, rsi)) {
        DEBUG_PRINTF("would generate a lazy tug, repeat is OK\n");
        return false;
    }

    const NFAVertex first = rsi.vertices.front();
    DEBUG_PRINTF("min depth from startds = %s\n",
                 depths.at(first).fromStartDotStar.min.str().c_str());
    if (depths.at(first).fromStartDotStar.min > depth(2)) {
        return false;
    }

    NFAVertex last = rsi.vertices.back();
    const CharReach &cyclicreach = g[last].char_reach;
    CharReach tugreach;
    for (auto v : adjacent_vertices_range(last, g)) {
        if (v == last || is_special(v, g)) {
            continue;
        }
        tugreach |= g[v].char_reach;
    }
    // Deal with unpeeled cases.
    if (tugreach.none()) {
        tugreach = cyclicreach;
    }
    DEBUG_PRINTF("tugreach.count=%zu, cyclicreach.count=%zu\n",
                 tugreach.count(), cyclicreach.count());
    return (tugreach.count() > 200) && (cyclicreach.count() > 200);
}

void analyseRepeats(NGHolder &g, const ReportManager *rm,
                    const map<u32, u32> &fixed_depth_tops,
                    const map<u32, vector<vector<CharReach>>> &triggers,
                    vector<BoundedRepeatData> *repeats, bool streaming,
                    bool simple_model_selection, const Grey &grey,
                    bool *reformed_start_ds) {
    if (!grey.allowExtendedNFA || !grey.allowLimExNFA) {
        return;
    }

    // Quick sanity test.
    assert(allMatchStatesHaveReports(g));

#ifndef NDEBUG
    // So we can assert that the number of tops hasn't changed at the end of
    // this analysis.
    const flat_set<u32> allTops = getTops(g);
#endif

    // Later on, we're (a little bit) dependent on depth information for
    // unpeeling and so forth. Note that these depths MUST be maintained when
    // new vertices are added.
    unordered_map<NFAVertex, NFAVertexDepth> depths;
    findInitDepths(g, depths);

    // Construct our list of subgraphs with the same reach using BGL magic.
    vector<ReachSubgraph> rs;
    buildReachSubgraphs(g, rs, grey.minExtBoundedRepeatSize);

    // Validate and split subgraphs.
    checkReachSubgraphs(g, rs, grey.minExtBoundedRepeatSize);

    // Identify which subgraphs represent bounded repeats in forms ("cliches")
    // that we accept, and mark the others as bad.
    for (auto &rsi: rs) {
        if (!processSubgraph(g, rsi, grey.minExtBoundedRepeatSize)) {
            rsi.bad = true;
            continue;
        }

        DEBUG_PRINTF("rsi min %s=max=%s\n", rsi.repeatMin.str().c_str(),
                     rsi.repeatMax.str().c_str());

        // Identify repeats with wide cyclic and tug reach which will produce
        // low-performance implementations and avoid doing them.
        if (repeatIsNasty(g, rsi, depths)) {
            DEBUG_PRINTF("marking nasty repeat as bad\n");
            rsi.bad = true;
        }
    }

    // Remove bad cases, then sort remaining subgraphs in descending size
    // order.
    rs.erase(remove_if(rs.begin(), rs.end(),
                       [](const ReachSubgraph &r) { return r.bad; }),
                       rs.end());
    stable_sort(rs.begin(), rs.end(),
                [](const ReachSubgraph &a, const ReachSubgraph &b) {
                    return a.vertices.size() > b.vertices.size();
                });

    if (!streaming && !givesBetterModel(g, rs)) {
        /* in block mode, there is no state space so we are only looking for
         * performance wins */
        DEBUG_PRINTF("repeat would not reduce NFA model size, skipping\n");
        return;
    }

    if (rs.empty()) {
        /* no good repeats */
        return;
    }

    // Store a copy of the original, unmodified graph in case we need to revert
    // back: in particular, due to tug cloning it is possible to build a graph
    // that was bigger than the original. See UE-2370.  FIXME: smarter analysis
    // could make this unnecessary?
    const unique_ptr<const NGHolder> orig_g(cloneHolder(g));

    unordered_set<NFAVertex> reached_by_fixed_tops;
    if (is_triggered(g)) {
        populateFixedTopInfo(fixed_depth_tops, g, &reached_by_fixed_tops);
    }

    // Go to town on the remaining acceptable subgraphs.
    unordered_set<NFAVertex> created;
    for (auto &rsi : rs) {
        DEBUG_PRINTF("subgraph (beginning vertex %zu) is a {%s,%s} repeat\n",
                     g[rsi.vertices.front()].index,
                     rsi.repeatMin.str().c_str(), rsi.repeatMax.str().c_str());

        if (!peelSubgraph(g, grey, rsi, created)) {
            DEBUG_PRINTF("peel failed, skipping\n");
            continue;
        }

        // Attempt to peel a vertex if we're up against startDs, for
        // performance reasons.
        peelStartDotStar(g, depths, grey, rsi);

        // Our peeling passes may have killed off this repeat.
        if (rsi.bad) {
            continue;
        }

        selectHistoryScheme(g, rm, rsi, depths, reached_by_fixed_tops, triggers,
                            *repeats, simple_model_selection);

        if (!generates_callbacks(g) && endsInAccept(g, rsi)) {
            DEBUG_PRINTF("accepty-rosy graph\n");
            replaceSubgraphWithLazySpecial(g, rsi, repeats, depths, created);
        } else if (endsInAcceptEod(g, rsi)) {
            DEBUG_PRINTF("accepty-rosy graph\n");
            replaceSubgraphWithLazySpecial(g, rsi, repeats, depths, created);
        } else {
            replaceSubgraphWithSpecial(g, rsi, repeats, depths, created);
        }

        // Some of our analyses require correctly numbered vertices, so we
        // renumber after changes.
        renumber_vertices(g);
    }

    bool modified_start_ds = false;

    // We may be able to make improvements to the graph for performance
    // reasons. Note that this may do 'orrible things like remove the startDs
    // cycle, this should only happen quite late in the graph lifecycle.
    if (repeats->size() == 1) {
        if (g.kind == NFA_OUTFIX) {
            improveLeadingRepeatOutfix(g, repeats->back(), created, *repeats);
            // (Does not modify startDs, so we don't need to set
            // reformed_start_ds for this case.)
        } else {
            modified_start_ds =
                improveLeadingRepeat(g, repeats->back(), created, *repeats);
        }
    }

    if (reformed_start_ds) {
        *reformed_start_ds = modified_start_ds;
    }

    if (!repeats->empty()) {
        if (num_vertices(g) > NFA_MAX_STATES) {
            // We've managed to build an unimplementable NFA. Swap back to the
            // original.
            DEBUG_PRINTF("NFA has %zu vertices; swapping back to the "
                         "original graph\n", num_vertices(g));
            clear_graph(g);
            assert(orig_g);
            cloneHolder(g, *orig_g);
            repeats->clear();
        }

        // Sanity test: we don't want any repeats that share special vertices
        // as our construction code later can't cope with it.
        assert(!hasOverlappingRepeats(g, *repeats));

        // We have modified the graph, so we need to ensure that our edges
        // and vertices are correctly numbered.
        renumber_vertices(g);
        renumber_edges(g);
        // Remove stray report IDs.
        clearReports(g);
    }

    // Quick sanity tests.
    assert(allMatchStatesHaveReports(g));
    assert(!is_triggered(g) || getTops(g) == allTops);
}

/**
 * \brief True if the non-special vertices in the given graph all have the same
 * character reachability.
 */
static
bool allOneReach(const NGHolder &g) {
    const CharReach *cr = nullptr;
    for (const auto &v : vertices_range(g)) {
        if (is_special(v, g)) {
            continue;
        }
        if (!cr) {
            cr = &g[v].char_reach;
        } else {
            if (*cr != g[v].char_reach) {
                return false;
            }
        }
    }
    return true;
}

bool isPureRepeat(const NGHolder &g, PureRepeat &repeat) {
    assert(allMatchStatesHaveReports(g));

    DEBUG_PRINTF("entry\n");

    // Must be start anchored.
    assert(edge(g.startDs, g.startDs, g).second);
    if (out_degree(g.startDs, g) > 1) {
        DEBUG_PRINTF("Unanchored\n");
        return false;
    }

    // Must not be EOD-anchored.
    assert(edge(g.accept, g.acceptEod, g).second);
    if (in_degree(g.acceptEod, g) > 1) {
        DEBUG_PRINTF("EOD anchored\n");
        return false;
    }

    // Must have precisely one top.
    if (is_triggered(g) && !onlyOneTop(g)) {
        DEBUG_PRINTF("Too many tops\n");
        return false;
    }

    if (!allOneReach(g)) {
        DEBUG_PRINTF("vertices with different reach\n");
        return false;
    }

    // We allow this code to report true for any repeat, even for '.*' or '.+'
    // cases.
    const u32 minNumVertices = 1;

    vector<ReachSubgraph> rs;
    buildReachSubgraphs(g, rs, minNumVertices);
    checkReachSubgraphs(g, rs, minNumVertices);
    if (rs.size() != 1) {
        DEBUG_PRINTF("too many subgraphs\n");
        return false;
    }

    ReachSubgraph &rsi = *rs.begin();
    if (!processSubgraph(g, rsi, minNumVertices)) {
        DEBUG_PRINTF("not a supported repeat\n");
        return false;
    }

    if (rsi.vertices.size() + N_SPECIALS != num_vertices(g)) {
        DEBUG_PRINTF("repeat doesn't span graph\n");
        return false;
    }

    assert(!rsi.bad);
    assert(rsi.vertices.size() >= minNumVertices);

    const NFAVertex v = rsi.vertices.back();

    repeat.reach = g[v].char_reach;
    repeat.bounds.min = rsi.repeatMin;
    repeat.bounds.max = rsi.repeatMax;
    insert(&repeat.reports, g[v].reports);

    if (isVacuous(g)) {
        // This graph might be a {0,N} or {0,} repeat. For this to be true, we
        // must have found a {1,N} or {1,} repeat and the start vertex must
        // have the same report set as the vertices in the repeat.
        if (repeat.bounds.min == depth(1) &&
            g[g.start].reports == g[v].reports) {
            repeat.bounds.min = depth(0);
            DEBUG_PRINTF("graph is %s repeat\n", repeat.bounds.str().c_str());
        } else {
            DEBUG_PRINTF("not a supported repeat\n");
            return false;
        }
    }

    assert(all_reports(g) == set<ReportID>(begin(g[v].reports),
                                           end(g[v].reports)));
    return true;
}

void findRepeats(const NGHolder &h, u32 minRepeatVertices,
                 vector<GraphRepeatInfo> *repeats_out) {
    // Construct our list of subgraphs with the same reach using BGL magic.
    vector<ReachSubgraph> rs;
    buildReachSubgraphs(h, rs, minRepeatVertices);
    checkReachSubgraphs(h, rs, minRepeatVertices);

    for (auto &rsi : rs) {
        if (!processSubgraph(h, rsi, minRepeatVertices)) {
            continue;
        }

        DEBUG_PRINTF("rsi min=%s max=%s\n", rsi.repeatMin.str().c_str(),
                     rsi.repeatMax.str().c_str());

        depth repeatMax = rsi.repeatMax;

        vector<BoundedRepeatData> all_repeats; /* we don't mutate the graph in
                                                * this path */
        if (hasCyclicSupersetEntryPath(h, rsi, all_repeats)) {
            DEBUG_PRINTF("selected FIRST history due to cyclic pred with "
                         "superset of reach\n");
            repeatMax = depth::infinity(); /* will continue to pump out matches */
        }
        if (hasCyclicSupersetExitPath(h, rsi, all_repeats)) {
            DEBUG_PRINTF("selected FIRST history due to cyclic succ with "
                         "superset of reach\n");
            repeatMax = depth::infinity(); /* will continue to pump out matches */
        }

        repeats_out->push_back(GraphRepeatInfo());
        GraphRepeatInfo &ri = repeats_out->back();
        ri.vertices.swap(rsi.vertices);
        ri.repeatMin = rsi.repeatMin;
        ri.repeatMax = repeatMax;
    }
}

} // namespace ue2
