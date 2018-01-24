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
 * \brief Graph fuzzer for approximate matching
 */

#include "ng_fuzzy.h"

#include "ng.h"
#include "ng_depth.h"
#include "ng_util.h"

#include <map>
#include <vector>
using namespace std;

namespace ue2 {

// returns all successors up to a given depth in a vector of sets, indexed by
// zero-based depth from source vertex
static
vector<flat_set<NFAVertex>> gatherSuccessorsByDepth(const NGHolder &g,
                                                    NFAVertex src, u32 depth) {
    vector<flat_set<NFAVertex>> result(depth);
    flat_set<NFAVertex> cur, next;

    assert(depth > 0);

    // populate current set of successors
    for (auto v : adjacent_vertices_range(src, g)) {
        // ignore self-loops
        if (src == v) {
            continue;
        }
        DEBUG_PRINTF("Node %zu depth 1\n", g[v].index);
        cur.insert(v);
    }
    result[0] = cur;

    for (unsigned d = 1; d < depth; d++) {
        // collect all successors for all current level vertices
        for (auto v : cur) {
            // don't go past special nodes
            if (is_special(v, g)) {
                continue;
            }

            for (auto succ : adjacent_vertices_range(v, g)) {
                // ignore self-loops
                if (v == succ) {
                    continue;
                }
                DEBUG_PRINTF("Node %zu depth %u\n", g[succ].index, d + 1);
                next.insert(succ);
            }
        }
        result[d] = next;
        next.swap(cur);
        next.clear();
    }

    return result;
}

// returns all predecessors up to a given depth in a vector of sets, indexed by
// zero-based depth from source vertex
static
vector<flat_set<NFAVertex>> gatherPredecessorsByDepth(const NGHolder &g,
                                                      NFAVertex src,
                                                      u32 depth) {
    vector<flat_set<NFAVertex>> result(depth);
    flat_set<NFAVertex> cur, next;

    assert(depth > 0);

    // populate current set of successors
    for (auto v : inv_adjacent_vertices_range(src, g)) {
        // ignore self-loops
        if (src == v) {
            continue;
        }
        DEBUG_PRINTF("Node %zu depth 1\n", g[v].index);
        cur.insert(v);
    }
    result[0] = cur;

    for (unsigned d = 1; d < depth; d++) {
        // collect all successors for all current level vertices
        for (auto v : cur) {
            for (auto pred : inv_adjacent_vertices_range(v, g)) {
                // ignore self-loops
                if (v == pred) {
                    continue;
                }
                DEBUG_PRINTF("Node %zu depth %u\n", g[pred].index, d + 1);
                next.insert(pred);
            }
        }
        result[d] = next;
        next.swap(cur);
        next.clear();
    }

    return result;
}

/*
 * This struct produces a fuzzed graph; that is, a graph that is able to match
 * the original pattern, as well as input data within a certain edit distance.
 * Construct the struct, then call fuzz_graph() to transform the graph.
 *
 * Terminology used:
 * - Shadow vertices: vertices mirroring the original graph at various edit
 * distances
 * - Shadow graph level: edit distance of a particular shadow graph
 * - Helpers: dot vertices assigned to shadow vertices, used for insert/replace
 */
struct ShadowGraph {
    NGHolder &g;
    u32 edit_distance;
    bool hamming;
    map<pair<NFAVertex, u32>, NFAVertex> shadow_map;
    map<pair<NFAVertex, u32>, NFAVertex> helper_map;
    map<NFAVertex, NFAVertex> clones;
    // edge creation is deferred
    vector<pair<NFAVertex, NFAVertex>> edges_to_be_added;
    flat_set<NFAVertex> orig;

    ShadowGraph(NGHolder &g_in, u32 ed_in, bool hamm_in)
        : g(g_in), edit_distance(ed_in), hamming(hamm_in) {}

    void fuzz_graph() {
        if (edit_distance == 0) {
            return;
        }

        DEBUG_PRINTF("edit distance = %u hamming = %s\n", edit_distance,
                     hamming ? "true" : "false");

        // step 1: prepare the vertices, helpers and shadows according to
        // the original graph
        prepare_graph();

        // step 2: add shadow and helper nodes
        build_shadow_graph();

        // step 3: set up reports for newly created vertices (and make clones
        // if necessary)
        if (!hamming) {
            create_reports();
        }

        // step 4: wire up shadow graph and helpers for insert/replace/remove
        connect_shadow_graph();

        // step 5: commit all the edge wirings
        DEBUG_PRINTF("Committing edge wirings\n");
        for (const auto &p : edges_to_be_added) {
            add_edge_if_not_present(p.first, p.second, g);
        }

        DEBUG_PRINTF("Done!\n");
    }

private:
    const NFAVertex& get_clone(const NFAVertex &v) {
        return contains(clones, v) ?
                    clones[v] : v;
    }

    void connect_to_clones(const NFAVertex &u, const NFAVertex &v) {
        const NFAVertex &clone_u = get_clone(u);
        const NFAVertex &clone_v = get_clone(v);

        edges_to_be_added.emplace_back(u, v);
        DEBUG_PRINTF("Adding edge: %zu -> %zu\n", g[u].index, g[v].index);

        // do not connect clones to accepts, we do it during cloning
        if (is_any_accept(clone_v, g)) {
            return;
        }
        edges_to_be_added.emplace_back(clone_u, clone_v);
        DEBUG_PRINTF("Adding edge: %zu -> %zu\n", g[clone_u].index,
                     g[clone_v].index);
    }

    void prepare_graph() {
        DEBUG_PRINTF("Building shadow graphs\n");

        for (auto v : vertices_range(g)) {
            // all level 0 vertices are their own helpers and their own shadows
            helper_map[make_pair(v, 0)] = v;
            shadow_map[make_pair(v, 0)] = v;

            // find special nodes
            if (is_any_accept(v, g)) {
                DEBUG_PRINTF("Node %zu is a special node\n", g[v].index);
                for (unsigned edit = 1; edit <= edit_distance; edit++) {
                    // all accepts are their own shadows and helpers at all
                    // levels
                    shadow_map[make_pair(v, edit)] = v;
                    helper_map[make_pair(v, edit)] = v;
                }
                continue;
            }
            DEBUG_PRINTF("Node %zu is to be shadowed\n", g[v].index);
            orig.insert(v);
        }
    }

    void build_shadow_graph() {
        for (auto v : orig) {
            DEBUG_PRINTF("Adding shadow/helper nodes for node %zu\n",
                         g[v].index);
            for (unsigned dist = 1; dist <= edit_distance; dist++) {
                auto shadow_v = v;

                // start and startDs cannot have shadows but do have helpers
                if (!is_any_start(v, g)) {
                    shadow_v = clone_vertex(g, v);
                    DEBUG_PRINTF("New shadow node ID: %zu (level %u)\n",
                                 g[shadow_v].index, dist);
                }
                shadow_map[make_pair(v, dist)] = shadow_v;

                // if there's nowhere to go from this vertex, no helper needed
                if (proper_out_degree(v, g) < 1) {
                    DEBUG_PRINTF("No helper for node ID: %zu (level %u)\n",
                                 g[shadow_v].index, dist);
                    helper_map[make_pair(v, dist)] = shadow_v;
                    continue;
                }

                // start and startDs only have helpers for insert, so not Hamming
                if (hamming && is_any_start(v, g)) {
                    DEBUG_PRINTF("No helper for node ID: %zu (level %u)\n",
                                 g[shadow_v].index, dist);
                    helper_map[make_pair(v, dist)] = shadow_v;
                    continue;
                }

                auto helper_v = clone_vertex(g, v);
                DEBUG_PRINTF("New helper node ID: %zu (level %u)\n",
                             g[helper_v].index, dist);

                // this is a helper, so make it a dot
                g[helper_v].char_reach = CharReach::dot();
                // do not copy virtual start's assert flags
                if (is_virtual_start(v, g)) {
                    DEBUG_PRINTF("Helper node ID is virtual start: %zu (level %u)\n",
                             g[helper_v].index, dist);
                    g[helper_v].assert_flags = 0;
                }
                helper_map[make_pair(v, dist)] = helper_v;
            }
        }
    }

    // wire up successors according to the original graph, wire helpers
    // to shadow successors (insert/replace)
    void connect_succs(NFAVertex v, u32 dist) {
        DEBUG_PRINTF("Wiring up successors for node %zu shadow level %u\n",
                     g[v].index, dist);
        const auto &cur_shadow_v = shadow_map[make_pair(v, dist)];
        const auto &cur_shadow_helper = helper_map[make_pair(v, dist)];

        // multiple insert
        if (!hamming && dist > 1) {
            const auto &prev_level_helper = helper_map[make_pair(v, dist - 1)];
            connect_to_clones(prev_level_helper, cur_shadow_helper);
        }

        for (auto orig_dst : adjacent_vertices_range(v, g)) {
            const auto &shadow_dst = shadow_map[make_pair(orig_dst, dist)];

            connect_to_clones(cur_shadow_v, shadow_dst);

            // ignore startDs for insert/replace
            if (orig_dst == g.startDs) {
                continue;
            }

            connect_to_clones(cur_shadow_helper, shadow_dst);
        }
    }

    // wire up predecessors according to the original graph, wire
    // predecessors to helpers (replace), wire predecessor helpers to
    // helpers (multiple replace)
    void connect_preds(NFAVertex v, u32 dist) {
        DEBUG_PRINTF("Wiring up predecessors for node %zu shadow level %u\n",
                     g[v].index, dist);
        const auto &cur_shadow_v = shadow_map[make_pair(v, dist)];
        const auto &cur_shadow_helper = helper_map[make_pair(v, dist)];

        auto orig_src_vertices = inv_adjacent_vertices_range(v, g);
        for (auto orig_src : orig_src_vertices) {
            // ignore edges from start to startDs
            if (v == g.startDs && orig_src == g.start) {
                continue;
            }
            // ignore self-loops for replace
            if (orig_src != v) {
                // do not wire a replace node for start vertices if we
                // have a virtual start
                if (is_virtual_start(v, g) && is_any_start(orig_src, g)) {
                    continue;
                }

                if (dist) {
                    const auto &prev_level_src =
                        shadow_map[make_pair(orig_src, dist - 1)];
                    const auto &prev_level_helper =
                        helper_map[make_pair(orig_src, dist - 1)];

                    connect_to_clones(prev_level_src, cur_shadow_helper);
                    connect_to_clones(prev_level_helper, cur_shadow_helper);
                }
            }
            // wire predecessor according to original graph
            const auto &shadow_src = shadow_map[make_pair(orig_src, dist)];

            connect_to_clones(shadow_src, cur_shadow_v);
        }
    }

    // wire up previous level helper to current shadow (insert)
    void connect_helpers(NFAVertex v, u32 dist) {
        DEBUG_PRINTF("Wiring up helpers for node %zu shadow level %u\n",
                     g[v].index, dist);
        const auto &cur_shadow_helper = helper_map[make_pair(v, dist)];
        auto prev_level_v = shadow_map[make_pair(v, dist - 1)];

        connect_to_clones(prev_level_v, cur_shadow_helper);
    }

    /*
     * wiring edges for removal is a special case.
     *
     * when wiring edges for removal, as well as wiring up immediate
     * predecessors to immediate successors, we also need to wire up more
     * distant successors to their respective shadow graph levels.
     *
     * for example, consider graph start->a->b->c->d->accept.
     *
     * at edit distance 1, we need remove edges start->b, a->c, b->d, and
     * c->accept, all going from original graph (level 0) to shadow graph
     * level 1.
     *
     * at edit distance 2, we also need edges start->c, a->d and b->accept,
     * all going from level 0 to shadow graph level 2.
     *
     * this is propagated to all shadow levels; that is, given edit
     * distance 3, we will have edges from shadow levels 0->1, 0->2,
     * 0->3, 1->2, 1->3, and 2->3.
     *
     * therefore, we wire them in steps: first wire with step 1 (0->1, 1->2,
     * 2->3) at depth 1, then wire with step 2 (0->2, 1->3) at depth 2, etc.
     *
     * we also have to wire helpers to their removal successors, to
     * accommodate for a replace followed by a remove, on all shadow levels.
     *
     * and finally, we also have to wire source shadows into removal
     * successor helpers on a level above, to accommodate for a remove
     * followed by a replace.
     */
    void connect_removals(NFAVertex v) {
        DEBUG_PRINTF("Wiring up remove edges for node %zu\n", g[v].index);

        // vertices returned by this function don't include self-loops
        auto dst_vertices_by_depth =
            gatherSuccessorsByDepth(g, v, edit_distance);
        auto orig_src_vertices = inv_adjacent_vertices_range(v, g);
        for (auto orig_src : orig_src_vertices) {
            // ignore self-loops
            if (orig_src == v) {
                continue;
            }
            for (unsigned step = 1; step <= edit_distance; step++) {
                for (unsigned dist = step; dist <= edit_distance; dist++) {
                    auto &dst_vertices = dst_vertices_by_depth[step - 1];
                    for (auto &orig_dst : dst_vertices) {
                        const auto &shadow_src =
                            shadow_map[make_pair(orig_src, dist - step)];
                        const auto &shadow_helper =
                            helper_map[make_pair(orig_src, dist - step)];
                        const auto &shadow_dst =
                                shadow_map[make_pair(orig_dst, dist)];

                        // removal
                        connect_to_clones(shadow_src, shadow_dst);

                        // removal from helper vertex
                        connect_to_clones(shadow_helper, shadow_dst);

                        // removal into helper, requires additional edit
                        if ((dist + 1) <= edit_distance) {
                            const auto &next_level_helper =
                                    helper_map[make_pair(orig_dst, dist + 1)];

                            connect_to_clones(shadow_src, next_level_helper);
                        }
                    }
                }
            }
        }
    }

    void connect_shadow_graph() {
        DEBUG_PRINTF("Wiring up the graph\n");

        for (auto v : orig) {

            DEBUG_PRINTF("Wiring up edges for node %zu\n", g[v].index);

            for (unsigned dist = 0; dist <= edit_distance; dist++) {

                // handle insert/replace
                connect_succs(v, dist);

                // handle replace/multiple insert
                connect_preds(v, dist);

                // handle helpers
                if (!hamming && dist > 0) {
                    connect_helpers(v, dist);
                }
            }

            // handle removals
            if (!hamming) {
                connect_removals(v);
            }
        }
    }

    void connect_to_targets(NFAVertex src, const flat_set<NFAVertex> &targets) {
        for (auto dst : targets) {
            DEBUG_PRINTF("Adding edge: %zu -> %zu\n", g[src].index,
                         g[dst].index);
            edges_to_be_added.emplace_back(src, dst);
        }
    }

    // create a clone of the vertex, but overwrite its report set
    void create_clone(NFAVertex v, const flat_set<ReportID> &reports,
                      unsigned max_edit_distance,
                      const flat_set<NFAVertex> &targets) {
        // some vertices may have the same reports, but different successors;
        // therefore, we may need to connect them multiple times, but still only
        // clone once
        bool needs_cloning = !contains(clones, v);

        DEBUG_PRINTF("Cloning node %zu\n", g[v].index);
        // go through all shadows and helpers, including
        // original vertex
        for (unsigned d = 0; d < max_edit_distance; d++) {
            auto shadow_v = shadow_map[make_pair(v, d)];
            auto helper_v = helper_map[make_pair(v, d)];

            NFAVertex new_shadow_v, new_helper_v;

            // make sure we don't clone the same vertex twice
            if (needs_cloning) {
                new_shadow_v = clone_vertex(g, shadow_v);
                DEBUG_PRINTF("New shadow node ID: %zu (level %u)\n",
                             g[new_shadow_v].index, d);
                clones[shadow_v] = new_shadow_v;
            } else {
                new_shadow_v = clones[shadow_v];
            }
            g[new_shadow_v].reports = reports;

            connect_to_targets(new_shadow_v, targets);

            if (shadow_v == helper_v) {
                continue;
            }
            if (needs_cloning) {
                new_helper_v = clone_vertex(g, helper_v);
                DEBUG_PRINTF("New helper node ID: %zu (level %u)\n",
                             g[new_helper_v].index, d);
                clones[helper_v] = new_helper_v;
            } else {
                new_helper_v = clones[helper_v];
            }
            g[new_helper_v].reports = reports;

            connect_to_targets(new_helper_v, targets);
        }
    }

    void write_reports(NFAVertex v, const flat_set<ReportID> &reports,
                       unsigned max_edit_distance,
                       const flat_set<NFAVertex> &targets) {
        // we're overwriting reports, but we're not losing any
        // information as we already cached all the different report
        // sets, so vertices having different reports will be cloned and set up
        // with the correct report set

        // go through all shadows and helpers, including original
        // vertex
        for (unsigned d = 0; d < max_edit_distance; d++) {
            auto shadow_v = shadow_map[make_pair(v, d)];
            auto helper_v = helper_map[make_pair(v, d)];
            DEBUG_PRINTF("Setting up reports for shadow node: %zu "
                         "(level %u)\n",
                         g[shadow_v].index, d);
            DEBUG_PRINTF("Setting up reports for helper node: %zu "
                         "(level %u)\n",
                         g[helper_v].index, d);
            g[shadow_v].reports = reports;
            g[helper_v].reports = reports;

            connect_to_targets(shadow_v, targets);
            connect_to_targets(helper_v, targets);
        }
    }

    /*
     * we may have multiple report sets per graph. that means, whenever we
     * construct additional paths through the graph (alternations, removals), we
     * have to account for the fact that some vertices are predecessors to
     * vertices with different report sets.
     *
     * whenever that happens, we have to clone the paths for both report sets,
     * and set up these new vertices with their respective report sets as well.
     *
     * in order to do that, we first have to get all the predecessors for accept
     * and acceptEod vertices. then, go through them one by one, and take note
     * of the report lists. the first report set we find, wins, the rest we
     * clone.
     *
     * we also have to do this in two passes, because there may be vertices that
     * are predecessors to vertices with different report sets, so to avoid
     * overwriting reports we will be caching reports info instead.
     */
    void create_reports() {
        map<flat_set<ReportID>, flat_set<NFAVertex>> reports_to_vertices;
        flat_set<NFAVertex> accepts{g.accept, g.acceptEod};

        // gather reports info from all vertices connected to accept
        for (auto accept : accepts) {
            for (auto src : inv_adjacent_vertices_range(accept, g)) {
                // skip special vertices
                if (is_special(src, g)) {
                    continue;
                }
                reports_to_vertices[g[src].reports].insert(src);
            }
        }

        // we expect to see at most two report sets
        assert(reports_to_vertices.size() > 0 &&
               reports_to_vertices.size() <= 2);

        // set up all reports
        bool clone = false;
        for (auto &pair : reports_to_vertices) {
            const auto &reports = pair.first;
            const auto &vertices = pair.second;

            for (auto src : vertices) {
                // get all predecessors up to edit distance
                auto src_vertices_by_depth =
                        gatherPredecessorsByDepth(g, src, edit_distance);

                // find which accepts source vertex connects to
                flat_set<NFAVertex> targets;
                for (const auto &accept : accepts) {
                    NFAEdge e = edge(src, accept, g);
                    if (e) {
                        targets.insert(accept);
                    }
                }
                assert(targets.size());

                for (unsigned d = 0; d < src_vertices_by_depth.size(); d++) {
                    const auto &preds = src_vertices_by_depth[d];
                    for (auto v : preds) {
                        // only clone a node if it already contains reports
                        if (clone && !g[v].reports.empty()) {
                            create_clone(v, reports, edit_distance - d,
                                         targets);
                        } else {
                            write_reports(v, reports, edit_distance - d,
                                          targets);
                        }
                    }
                }
            }
            // clone vertices only if it's not our first report set
            clone = true;
        }
    }
};

// check if we will edit our way into a vacuous pattern
static
bool will_turn_vacuous(const NGHolder &g, u32 edit_distance) {
    auto depths = calcRevDepths(g);

    depth min_depth = depth::infinity();
    auto idx = g[g.start].index;

    // check distance from start to accept/acceptEod
    if (depths[idx].toAccept.min.is_finite()) {
        min_depth = min(depths[idx].toAccept.min, min_depth);
    }
    if (depths[idx].toAcceptEod.min.is_finite()) {
        min_depth = min(depths[idx].toAcceptEod.min, min_depth);
    }

    idx = g[g.startDs].index;

    // check distance from startDs to accept/acceptEod
    if (depths[idx].toAccept.min.is_finite()) {
        min_depth = min(depths[idx].toAccept.min, min_depth);
    }
    if (depths[idx].toAcceptEod.min.is_finite()) {
        min_depth = min(depths[idx].toAcceptEod.min, min_depth);
    }

    assert(min_depth.is_finite());

    // now, check if we can edit our way into a vacuous pattern
    if (min_depth <= (u64a) edit_distance + 1) {
        DEBUG_PRINTF("Pattern will turn vacuous if approximately matched\n");
        return true;
    }
    return false;
}

void validate_fuzzy_compile(const NGHolder &g, u32 edit_distance, bool hamming,
                            bool utf8, const Grey &grey) {
    if (edit_distance == 0) {
        return;
    }
    if (!grey.allowApproximateMatching) {
        throw CompileError("Approximate matching is disabled.");
    }
    if (edit_distance > grey.maxEditDistance) {
        throw CompileError("Edit distance is too big.");
    }
    if (utf8) {
        throw CompileError("UTF-8 is disallowed for approximate matching.");
    }
    // graph isn't fuzzable if there are edge assertions anywhere in the graph
    for (auto e : edges_range(g)) {
        if (g[e].assert_flags) {
            throw CompileError("Zero-width assertions are disallowed for "
                               "approximate matching.");
        }
    }
    if (!hamming && will_turn_vacuous(g, edit_distance)) {
        throw CompileError("Approximate matching patterns that reduce to "
                           "vacuous patterns are disallowed.");
    }
}

void make_fuzzy(NGHolder &g, u32 edit_distance, bool hamming,
                const Grey &grey) {
    if (edit_distance == 0) {
        return;
    }

    assert(grey.allowApproximateMatching);
    assert(grey.maxEditDistance >= edit_distance);

    ShadowGraph sg(g, edit_distance, hamming);
    sg.fuzz_graph();

    // For safety, enforce limit on actual vertex count.
    if (num_vertices(g) > grey.limitApproxMatchingVertices) {
        DEBUG_PRINTF("built %zu vertices > limit of %u\n", num_vertices(g),
                     grey.limitApproxMatchingVertices);
        throw ResourceLimitError();
    }
}

} // namespace ue2
