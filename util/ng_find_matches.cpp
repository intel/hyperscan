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
 * \brief Pattern lifetime analysis.
 */

#include "config.h"

#include "ng_find_matches.h"

#include "nfagraph/ng_holder.h"
#include "nfagraph/ng_util.h"
#include "parser/position.h"
#include "util/container.h"
#include "util/compare.h"
#include "util/report.h"
#include "util/report_manager.h"
#include "util/unordered.h"

#include <algorithm>

using namespace std;
using namespace ue2;

using MatchSet = set<pair<size_t, size_t>>;
using StateBitSet = boost::dynamic_bitset<>;

namespace {

/** \brief Max number of states (taking edit distance into account). */
static constexpr size_t STATE_COUNT_MAX = 15000;

// returns all successors up to a given depth in a vector of sets, indexed by
// zero-based depth from source vertex
static
vector<flat_set<NFAVertex>>
gatherSuccessorsByDepth(const NGHolder &g, const NFAVertex &src, u32 depth) {
    assert(depth > 0);

    vector<flat_set<NFAVertex>> result(depth);

    // populate current set of successors
    for (auto v : adjacent_vertices_range(src, g)) {
        // ignore self-loops
        if (src == v) {
            continue;
        }
        DEBUG_PRINTF("Node %zu depth 1\n", g[v].index);
        result[0].insert(v);
    }

    for (u32 d = 1; d < depth; d++) {
        // collect all successors for all current level vertices
        const auto &cur = result[d - 1];
        auto &next = result[d];
        for (auto u : cur) {
            // don't go past special nodes
            if (is_special(u, g)) {
                continue;
            }

            for (auto v : adjacent_vertices_range(u, g)) {
                // ignore self-loops
                if (u == v) {
                    continue;
                }
                DEBUG_PRINTF("Node %zu depth %u\n", g[v].index, d + 1);
                next.insert(v);
            }
        }
    }

    return result;
}

// returns all predecessors up to a given depth in a vector of sets, indexed by
// zero-based depth from source vertex
static
vector<flat_set<NFAVertex>>
gatherPredecessorsByDepth(const NGHolder &g, NFAVertex src, u32 depth) {
    assert(depth > 0);

    vector<flat_set<NFAVertex>> result(depth);

    // populate current set of successors
    for (auto v : inv_adjacent_vertices_range(src, g)) {
        // ignore self-loops
        if (src == v) {
            continue;
        }
        DEBUG_PRINTF("Node %zu depth 1\n", g[v].index);
        result[0].insert(v);
    }

    for (u32 d = 1; d < depth; d++) {
        // collect all successors for all current level vertices
        const auto &cur = result[d - 1];
        auto &next = result[d];
        for (auto v : cur) {
            for (auto u : inv_adjacent_vertices_range(v, g)) {
                // ignore self-loops
                if (v == u) {
                    continue;
                }
                DEBUG_PRINTF("Node %zu depth %u\n", g[u].index, d + 1);
                next.insert(u);
            }
        }
    }

    return result;
}

// this is a per-vertex, per-shadow level state transition table
struct GraphCache {
    GraphCache(u32 dist_in, u32 hamm_in, const NGHolder &g)
        : hamming(hamm_in > 0), size(num_vertices(g)),
          edit_distance(hamming ? hamm_in : dist_in)
    {
        auto dist_max = edit_distance + 1;

        allocateStateTransitionTable(dist_max);
        populateTransitionCache(g, dist_max);
        populateAcceptCache(g, dist_max);
    }

    void allocateStateTransitionTable(u32 dist_max) {
        // resize level 1 - per vertex
        shadow_transitions.resize(size);
        helper_transitions.resize(size);

        // resize level 2 - per shadow level
        for (u32 i = 0; i < size; i++) {
            shadow_transitions[i].resize(dist_max);
            helper_transitions[i].resize(dist_max);

            // resize level 3 - per vertex
            for (u32 d = 0; d < dist_max; d++) {
                shadow_transitions[i][d].resize(size);
                helper_transitions[i][d].resize(size);
            }
        }

        // accept states are indexed by edit distance
        accept_states.resize(dist_max);
        accept_eod_states.resize(dist_max);

        // vertex report maps are indexed by edit distance
        vertex_reports_by_level.resize(dist_max);
        vertex_eod_reports_by_level.resize(dist_max);
    }

    /*
     * certain transitions to helpers are disallowed:
     *  1. transitions from accept/acceptEod
     *  2. transitions to accept/acceptEod
     *  3. from start to startDs
     *  4. to a virtual/multiline start
     *
     * everything else is allowed.
     */
    bool canTransitionToHelper(NFAVertex u, NFAVertex v, const NGHolder &g) const {
        if (is_any_accept(u, g)) {
            return false;
        }
        if (is_any_accept(v, g)) {
            return false;
        }
        if (u == g.start && v == g.startDs) {
            return false;
        }
        if (is_virtual_start(v, g)) {
            return false;
        }
        return true;
    }

    void populateTransitionCache(const NGHolder &g, u32 dist_max) {
        // populate mapping of vertex index to vertex
        vector<NFAVertex> idx_to_v(size);
        for (auto v : vertices_range(g)) {
            idx_to_v[g[v].index] = v;
        }

        for (u32 i = 0; i < size; i++) {
            auto cur_v = idx_to_v[i];

            // set up transition tables
            auto succs = gatherSuccessorsByDepth(g, cur_v, dist_max);

            assert(succs.size() == dist_max);

            for (u32 d = 0; d < dist_max; d++) {
                auto &v_shadows = shadow_transitions[i][d];
                auto cur_v_bit = i;

                // enable transition to next level helper (this handles insertion)
                if (!hamming && d < edit_distance && !is_any_accept(cur_v, g)) {
                    auto &next_v_helpers = helper_transitions[i][d + 1];

                    next_v_helpers.set(cur_v_bit);
                }

                // if vertex has a self-loop, we can also transition to it,
                // but only if we're at shadow level 0
                if (edge(cur_v, cur_v, g).second && d == 0) {
                    v_shadows.set(cur_v_bit);
                }

                if (hamming && d > 0) {
                    continue;
                }

                // populate state transition tables
                for (auto v : succs[d]) {
                    auto v_bit = g[v].index;

                    // we cannot transition to startDs on any level other than
                    // level 0
                    if (v != g.startDs || d == 0) {
                        // this handles direct transitions as well as removals
                        v_shadows.set(v_bit);
                    }

                    // we can also transition to next-level helper (handles
                    // replace), provided we meet the criteria
                    if (d < edit_distance && canTransitionToHelper(cur_v, v, g)) {
                        auto &next_v_helpers = helper_transitions[i][d + 1];

                        next_v_helpers.set(v_bit);
                    }
                }
            }
        }
    }

    void populateAcceptCache(const NGHolder &g, u32 dist_max) {
        // set up accept states masks
        StateBitSet accept(size);
        accept.set(g[g.accept].index);
        StateBitSet accept_eod(size);
        accept_eod.set(g[g.acceptEod].index);

        // gather accept and acceptEod states
        for (u32 base_dist = 0; base_dist < dist_max; base_dist++) {
            auto &states = accept_states[base_dist];
            auto &eod_states = accept_eod_states[base_dist];

            states.resize(size);
            eod_states.resize(size);

            // inspect each vertex
            for (u32 i = 0; i < size; i++) {
                // inspect all shadow levels from base_dist to dist_max
                for (u32 d = 0; d < dist_max - base_dist; d++) {
                    auto &shadows = shadow_transitions[i][d];

                    // if this state transitions to accept, set its bit
                    if ((shadows & accept).any()) {
                        states.set(i);
                    }
                    if ((shadows & accept_eod).any()) {
                        eod_states.set(i);
                    }
                }
            }
        }

        // populate accepts cache
        for (auto  v : inv_adjacent_vertices_range(g.accept, g)) {
            const auto &rs = g[v].reports;

            for (u32 d = 0; d <= edit_distance; d++) {
                // add self to report list at all levels
                vertex_reports_by_level[d][v].insert(rs.begin(), rs.end());
            }

            if (edit_distance == 0 || hamming) {
                // if edit distance is 0, no predecessors will have reports
                continue;
            }

            auto preds_by_depth = gatherPredecessorsByDepth(g, v, edit_distance);
            for (u32 pd = 0; pd < preds_by_depth.size(); pd++) {
                const auto &preds = preds_by_depth[pd];
                // for each predecessor, add reports up to maximum edit distance
                // for current depth from source vertex
                for (auto pred : preds) {
                    for (u32 d = 0; d < edit_distance - pd; d++) {
                        vertex_reports_by_level[d][pred].insert(rs.begin(), rs.end());
                    }
                }
            }
        }
        for (auto v : inv_adjacent_vertices_range(g.acceptEod, g)) {
            const auto &rs = g[v].reports;

            if (v == g.accept) {
                continue;
            }

            for (u32 d = 0; d <= edit_distance; d++) {
                // add self to report list at all levels
                vertex_eod_reports_by_level[d][v].insert(rs.begin(), rs.end());
            }
            if (edit_distance == 0 || hamming) {
                // if edit distance is 0, no predecessors will have reports
                continue;
            }

            auto preds_by_depth = gatherPredecessorsByDepth(g, v, edit_distance);
            for (u32 pd = 0; pd < preds_by_depth.size(); pd++) {
                const auto &preds = preds_by_depth[pd];
                // for each predecessor, add reports up to maximum edit distance
                // for current depth from source vertex
                for (auto pred : preds) {
                    for (u32 d = 0; d < edit_distance - pd; d++) {
                        vertex_eod_reports_by_level[d][pred].insert(rs.begin(), rs.end());
                    }
                }
            }
        }
    }

#ifdef DEBUG
    void dumpStateTransitionTable(const NGHolder &g) {
        StateBitSet accept(size);
        accept.set(g[g.accept].index);
        StateBitSet accept_eod(size);
        accept_eod.set(g[g.acceptEod].index);

        DEBUG_PRINTF("Dumping state transition tables\n");
        DEBUG_PRINTF("Shadows:\n");
        for (u32 i = 0; i < num_vertices(g); i++) {
            DEBUG_PRINTF("%-7s %3u:", "Vertex", i);
            for (u32 j = 0; j < num_vertices(g); j++) {
                printf("%3i", j);
            }
            printf("\n");
            for (u32 d = 0; d <= edit_distance; d++) {
                DEBUG_PRINTF("%-7s %3u:", "Level", d);
                const auto &s = getShadowTransitions(i, d);
                for (u32 j = 0; j < num_vertices(g); j++) {
                    printf("%3i", s.test(j));
                }
                printf("\n");
            }
            DEBUG_PRINTF("\n");
        }

        DEBUG_PRINTF("Helpers:\n");
        for (u32 i = 0; i < num_vertices(g); i++) {
            DEBUG_PRINTF("%-7s %3u:", "Vertex", i);
            for (u32 j = 0; j < num_vertices(g); j++) {
                printf("%3i", j);
            }
            printf("\n");
            for (u32 d = 0; d <= edit_distance; d++) {
                DEBUG_PRINTF("%-7s %3u:", "Level", d);
                const auto &s = getHelperTransitions(i, d);
                for (u32 j = 0; j < num_vertices(g); j++) {
                    printf("%3i", s.test(j));
                }
                printf("\n");
            }
            DEBUG_PRINTF("\n");
        }

        DEBUG_PRINTF("Accept transitions:\n");
        DEBUG_PRINTF("%-12s", "Vertex idx:");
        for (u32 j = 0; j < num_vertices(g); j++) {
            printf("%3i", j);
        }
        printf("\n");
        for (u32 d = 0; d <= edit_distance; d++) {
            DEBUG_PRINTF("%-7s %3u:", "Level", d);
            const auto &s = getAcceptTransitions(d);
            for (u32 j = 0; j < num_vertices(g); j++) {
                printf("%3i", s.test(j));
            }
            printf("\n");
        }
        DEBUG_PRINTF("\n");

        DEBUG_PRINTF("Accept EOD transitions:\n");
        DEBUG_PRINTF("%-12s", "Vertex idx:");
        for (u32 j = 0; j < num_vertices(g); j++) {
            printf("%3i", j);
        }
        printf("\n");
        for (u32 d = 0; d <= edit_distance; d++) {
            DEBUG_PRINTF("%-7s %3u:", "Level", d);
            const auto &s = getAcceptEodTransitions(d);
            for (u32 j = 0; j < num_vertices(g); j++) {
                printf("%3i", s.test(j));
            }
            printf("\n");
        }
        DEBUG_PRINTF("\n");

        DEBUG_PRINTF("%-12s ", "Accepts:");
        for (u32 i = 0; i < num_vertices(g); i++) {
            printf("%3i", accept.test(i));
        }
        printf("\n");

        DEBUG_PRINTF("%-12s ", "EOD Accepts:");
        for (u32 i = 0; i < num_vertices(g); i++) {
            printf("%3i", accept_eod.test(i));
        }
        printf("\n");

        DEBUG_PRINTF("Reports\n");
        for (auto v : vertices_range(g)) {
            for (u32 d = 0; d <= edit_distance; d++) {
                const auto &r = vertex_reports_by_level[d][v];
                const auto &e = vertex_eod_reports_by_level[d][v];
                DEBUG_PRINTF("%-7s %3zu %-8s %3zu %-8s %3zu\n",
                             "Vertex", g[v].index, "rs:", r.size(), "eod:", e.size());
            }
        }
        printf("\n");
    }
#endif

    const StateBitSet& getShadowTransitions(u32 idx, u32 level) const {
        assert(idx < size);
        assert(level <= edit_distance);
        return shadow_transitions[idx][level];
    }
    const StateBitSet& getHelperTransitions(u32 idx, u32 level) const {
        assert(idx < size);
        assert(level <= edit_distance);
        return helper_transitions[idx][level];
    }
    const StateBitSet& getAcceptTransitions(u32 level) const {
        assert(level <= edit_distance);
        return accept_states[level];
    }
    const StateBitSet& getAcceptEodTransitions(u32 level) const {
        assert(level <= edit_distance);
        return accept_eod_states[level];
    }

    /*
     * the bitsets are indexed by vertex and shadow level. the bitset's length is
     * equal to the total number of vertices in the graph.
     *
     * for convenience, helper functions are provided.
     */
    vector<vector<StateBitSet>> shadow_transitions;
    vector<vector<StateBitSet>> helper_transitions;

    // accept states masks, indexed by shadow level
    vector<StateBitSet> accept_states;
    vector<StateBitSet> accept_eod_states;

    // map of all reports associated with any vertex, indexed by shadow level
    vector<map<NFAVertex, flat_set<ReportID>>> vertex_reports_by_level;
    vector<map<NFAVertex, flat_set<ReportID>>> vertex_eod_reports_by_level;

    bool hamming;
    u32 size;
    u32 edit_distance;
};


/*
 * SOM workflow is expected to be the following:
 * - Caller calls getActiveStates, which reports SOM for each active states
 * - Caller calls getSuccessorStates on each of the active states, which *doesn't*
 *   report SOM
 * - Caller decides if the successor state should be activated, and calls
 *   activateState with SOM set to that of previous active state (not successor!)
 * - activateState then resolves any conflicts between SOMs that may arise from
 *   multiple active states progressing to the same successor
 */
struct StateSet {
    struct State {
        enum node_type {
            NODE_SHADOW = 0,
            NODE_HELPER
        };
        State(size_t idx_in, u32 level_in, size_t som_in, node_type type_in) :
            idx(idx_in), level(level_in), som(som_in), type(type_in) {}
        size_t idx;
        u32 level;
        size_t som;
        node_type type;
    };

    // Temporary working data used for step() which we want to keep around
    // (rather than reallocating vectors all the time).
    struct WorkingData {
        vector<State> active;
        vector<State> succ_list;
    };

    StateSet(size_t sz, u32 dist_in) :
            shadows(dist_in + 1), helpers(dist_in + 1),
            shadows_som(dist_in + 1), helpers_som(dist_in + 1),
            edit_distance(dist_in) {
        for (u32 dist = 0; dist <= dist_in; dist++) {
            shadows[dist].resize(sz, false);
            helpers[dist].resize(sz, false);
            shadows_som[dist].resize(sz, 0);
            helpers_som[dist].resize(sz, 0);
        }
    }

    void reset() {
        for (u32 dist = 0; dist <= edit_distance; dist++) {
            shadows[dist].reset();
            helpers[dist].reset();
            fill(shadows_som[dist].begin(), shadows_som[dist].end(), 0);
            fill(helpers_som[dist].begin(), helpers_som[dist].end(), 0);
        }
    }

    bool empty() const {
        for (u32 dist = 0; dist <= edit_distance; dist++) {
            if (shadows[dist].any()) {
                return false;
            }
            if (helpers[dist].any()) {
                return false;
            }
        }
        return true;
    }

    size_t count() const {
        size_t result = 0;

        for (u32 dist = 0; dist <= edit_distance; dist++) {
            result += shadows[dist].count();
            result += helpers[dist].count();
        }

        return result;
    }

    bool setActive(const State &s) {
        switch (s.type) {
        case State::NODE_HELPER:
            return helpers[s.level].test_set(s.idx);
        case State::NODE_SHADOW:
            return shadows[s.level].test_set(s.idx);
        }
        assert(0);
        return false;
    }

    size_t getCachedSom(const State &s) const {
        switch (s.type) {
        case State::NODE_HELPER:
            return helpers_som[s.level][s.idx];
        case State::NODE_SHADOW:
            return shadows_som[s.level][s.idx];
        }
        assert(0);
        return 0;
    }

    void setCachedSom(const State &s, const size_t som_val) {
        switch (s.type) {
        case State::NODE_HELPER:
            helpers_som[s.level][s.idx] = som_val;
            break;
        case State::NODE_SHADOW:
            shadows_som[s.level][s.idx] = som_val;
            break;
        default:
            assert(0);
        }
    }

#ifdef DEBUG
    void dumpActiveStates() const {
        vector<State> states;
        getActiveStates(states);

        DEBUG_PRINTF("Dumping active states\n");

        for (const auto &state : states) {
            DEBUG_PRINTF("type: %s idx: %zu level: %u som: %zu\n",
                         state.type == State::NODE_HELPER ? "HELPER" : "SHADOW",
                         state.idx, state.level, state.som);
        }
    }
#endif

    void getActiveStates(vector<State> &result) const {
        result.clear();

        for (u32 dist = 0; dist <= edit_distance; dist++) {
            // get all shadow vertices (including original graph)
            const auto &cur_shadow_vertices = shadows[dist];
            for (size_t id = cur_shadow_vertices.find_first();
                 id != cur_shadow_vertices.npos;
                 id = cur_shadow_vertices.find_next(id)) {
                result.emplace_back(id, dist, shadows_som[dist][id],
                                    State::NODE_SHADOW);
            }

            // the rest is only valid for edited graphs
            if (dist == 0) {
                continue;
            }

            // get all helper vertices
            const auto &cur_helper_vertices = helpers[dist];
            for (size_t id = cur_helper_vertices.find_first();
                 id != cur_helper_vertices.npos;
                 id = cur_helper_vertices.find_next(id)) {
                result.emplace_back(id, dist, helpers_som[dist][id],
                                    State::NODE_HELPER);
            }
        }

        sort_and_unique(result);
    }

    // does not return SOM
    void getSuccessors(const State &state, const GraphCache &gc,
                       vector<State> &result) const {
        result.clear();

        // maximum shadow depth that we can go from current level
        u32 max_depth = edit_distance - state.level + 1;

        for (u32 d = 0; d < max_depth; d++) {
            const auto &shadow_succ = gc.getShadowTransitions(state.idx, d);
            for (size_t id = shadow_succ.find_first();
                 id != shadow_succ.npos;
                 id = shadow_succ.find_next(id)) {
                auto new_level = state.level + d;
                result.emplace_back(id, new_level, 0, State::NODE_SHADOW);
            }

            const auto &helper_succ = gc.getHelperTransitions(state.idx, d);
            for (size_t id = helper_succ.find_first();
                 id != helper_succ.npos;
                 id = helper_succ.find_next(id)) {
                auto new_level = state.level + d;
                result.emplace_back(id, new_level, 0, State::NODE_HELPER);
            }
        }

        sort_and_unique(result);
    }

    void getAcceptStates(const GraphCache &gc, vector<State> &result) const {
        result.clear();

        for (u32 dist = 0; dist <= edit_distance; dist++) {
            // get all shadow vertices (including original graph)
            auto cur_shadow_vertices = shadows[dist];
            cur_shadow_vertices &= gc.getAcceptTransitions(dist);
            for (size_t id = cur_shadow_vertices.find_first();
                 id != cur_shadow_vertices.npos;
                 id = cur_shadow_vertices.find_next(id)) {
                result.emplace_back(id, dist, shadows_som[dist][id],
                                    State::NODE_SHADOW);
            }

            auto cur_helper_vertices = helpers[dist];
            cur_helper_vertices &= gc.getAcceptTransitions(dist);
            for (size_t id = cur_helper_vertices.find_first();
                 id != cur_helper_vertices.npos;
                 id = cur_helper_vertices.find_next(id)) {
                result.emplace_back(id, dist, helpers_som[dist][id],
                                    State::NODE_HELPER);
            }
        }

        sort_and_unique(result);
    }

    void getAcceptEodStates(const GraphCache &gc, vector<State> &result) const {
        result.clear();

        for (u32 dist = 0; dist <= edit_distance; dist++) {
            // get all shadow vertices (including original graph)
            auto cur_shadow_vertices = shadows[dist];
            cur_shadow_vertices &= gc.getAcceptEodTransitions(dist);
            for (size_t id = cur_shadow_vertices.find_first();
                 id != cur_shadow_vertices.npos;
                 id = cur_shadow_vertices.find_next(id)) {
                result.emplace_back(id, dist, shadows_som[dist][id],
                                    State::NODE_SHADOW);
            }

            auto cur_helper_vertices = helpers[dist];
            cur_helper_vertices &= gc.getAcceptEodTransitions(dist);
            for (size_t id = cur_helper_vertices.find_first();
                 id != cur_helper_vertices.npos;
                 id = cur_helper_vertices.find_next(id)) {
                result.emplace_back(id, dist, helpers_som[dist][id],
                                    State::NODE_HELPER);
            }
        }

        sort_and_unique(result);
    }

    // the caller must specify SOM at current offset, and must not attempt to
    // resolve SOM inheritance conflicts
    void activateState(const State &state) {
        size_t cur_som = state.som;
        if (setActive(state)) {
            size_t cached_som = getCachedSom(state);
            cur_som = min(cur_som, cached_som);
        }
        setCachedSom(state, cur_som);
    }

    vector<StateBitSet> shadows;
    vector<StateBitSet> helpers;
    vector<vector<size_t>> shadows_som;
    vector<vector<size_t>> helpers_som;
    u32 edit_distance;
};

// for flat_set
bool operator<(const StateSet::State &a, const StateSet::State &b) {
    ORDER_CHECK(idx);
    ORDER_CHECK(level);
    ORDER_CHECK(type);
    ORDER_CHECK(som);
    return false;
}

bool operator==(const StateSet::State &a, const StateSet::State &b) {
    return a.idx == b.idx && a.level == b.level && a.type == b.type &&
           a.som == b.som;
}

/** \brief Cache to speed up edge lookups, rather than hitting the graph. */
struct EdgeCache {
    explicit EdgeCache(const NGHolder &g) {
        cache.reserve(num_vertices(g));
        for (auto e : edges_range(g)) {
            cache.emplace(make_pair(source(e, g), target(e, g)), e);
        }
    }

    NFAEdge get(NFAVertex u, NFAVertex v) const {
        auto it = cache.find(make_pair(u, v));
        if (it != cache.end()) {
            return it->second;
        }
        return NFAEdge();
    }

private:
    ue2_unordered_map<pair<NFAVertex, NFAVertex>, NFAEdge> cache;
};

struct fmstate {
    const size_t num_states; // number of vertices in graph
    StateSet states; // currently active states
    StateSet next; // states on after this iteration
    GraphCache &gc;
    vector<NFAVertex> vertices; // mapping from index to vertex
    EdgeCache edge_cache;
    size_t offset = 0;
    unsigned char cur = 0;
    unsigned char prev = 0;
    const bool utf8;
    const bool allowStartDs;
    const ReportManager &rm;

    fmstate(const NGHolder &g, GraphCache &gc_in, bool utf8_in, bool aSD_in,
            const u32 edit_distance, const ReportManager &rm_in)
        : num_states(num_vertices(g)),
          states(num_states, edit_distance),
          next(num_states, edit_distance),
          gc(gc_in), vertices(num_vertices(g), NGHolder::null_vertex()),
          edge_cache(g), utf8(utf8_in), allowStartDs(aSD_in), rm(rm_in) {
        // init states
        states.activateState(
                    StateSet::State {g[g.start].index, 0, 0,
                                     StateSet::State::NODE_SHADOW});
        if (allowStartDs) {
            states.activateState(
                        StateSet::State {g[g.startDs].index, 0, 0,
                                         StateSet::State::NODE_SHADOW});
        }
        // fill vertex mapping
        for (auto v : vertices_range(g)) {
            vertices[g[v].index] = v;
        }
    }
};

} // namespace

static
bool isWordChar(const unsigned char c) {
    // check if it's an alpha character
    if (ourisalpha(c)) {
        return true;
    }
    // check if it's a digit
    if (c >= '0' && c <= '9') {
        return true;
    }
    // check if it's an underscore
    if (c == '_') {
        return true;
    }
    return false;
}

static
bool isUtf8CodePoint(const char c) {
    // check if this is a start of 4-byte character
    if ((c & 0xF8) == 0xF0) {
        return true;
    }
    // check if this is a start of 3-byte character
    if ((c & 0xF0) == 0xE0) {
        return true;
    }
    // check if this is a start of 2-byte character
    if ((c & 0xE0) == 0xC0) {
        return true;
    }
    // check if this is a single-byte character
    if ((c & 0x80) == 0) {
        return true;
    }
    return false;
}

static
bool canReach(const NGHolder &g, const NFAEdge &e, struct fmstate &state) {
    auto flags = g[e].assert_flags;
    if (!flags) {
        return true;
    }

    if (flags & POS_FLAG_ASSERT_WORD_TO_NONWORD) {
        if (isWordChar(state.prev) && !isWordChar(state.cur)) {
            return true;
        }
    }

    if (flags & POS_FLAG_ASSERT_NONWORD_TO_WORD) {
        if (!isWordChar(state.prev) && isWordChar(state.cur)) {
            return true;
        }
    }

    if (flags & POS_FLAG_ASSERT_WORD_TO_WORD) {
        if (isWordChar(state.prev) && isWordChar(state.cur)) {
            return true;
        }
    }

    if (flags & POS_FLAG_ASSERT_NONWORD_TO_NONWORD) {
        if (!isWordChar(state.prev) && !isWordChar(state.cur)) {
            return true;
        }
    }

    return false;
}

static
void getAcceptMatches(const NGHolder &g, MatchSet &matches,
                      struct fmstate &state, NFAVertex accept_vertex,
                      vector<StateSet::State> &active_states) {
    assert(accept_vertex == g.accept || accept_vertex == g.acceptEod);

    const bool eod = accept_vertex == g.acceptEod;
    if (eod) {
        state.states.getAcceptEodStates(state.gc, active_states);
    } else {
        state.states.getAcceptStates(state.gc, active_states);
    }

    DEBUG_PRINTF("Number of active states: %zu\n", active_states.size());

    for (const auto &cur : active_states) {
        auto u = state.vertices[cur.idx];

        // we can't accept anything from startDs in between UTF-8 codepoints
        if (state.utf8 && u == g.startDs && !isUtf8CodePoint(state.cur)) {
            continue;
        }

        const auto &reports =
            eod ? state.gc.vertex_eod_reports_by_level[cur.level][u]
                : state.gc.vertex_reports_by_level[cur.level][u];

        NFAEdge e = state.edge_cache.get(u, accept_vertex);

        // we assume edge assertions only exist at level 0
        if (e && !canReach(g, e, state)) {
            continue;
        }

        DEBUG_PRINTF("%smatch found at %zu\n", eod ? "eod " : "", state.offset);

        assert(!reports.empty());
        for (const auto &report_id : reports) {
            const Report &ri = state.rm.getReport(report_id);

            DEBUG_PRINTF("report %u has offset adjustment %d\n", report_id,
                         ri.offsetAdjust);
            DEBUG_PRINTF("match from (i:%zu,l:%u,t:%u): (%zu,%zu)\n", cur.idx,
                         cur.level, cur.type, cur.som,
                         state.offset + ri.offsetAdjust);
            matches.emplace(cur.som, state.offset + ri.offsetAdjust);
        }
    }
}

static
void getMatches(const NGHolder &g, MatchSet &matches, struct fmstate &state,
                StateSet::WorkingData &wd, bool allowEodMatches) {
    getAcceptMatches(g, matches, state, g.accept, wd.active);
    if (allowEodMatches) {
        getAcceptMatches(g, matches, state, g.acceptEod, wd.active);
    }
}

static
void step(const NGHolder &g, fmstate &state, StateSet::WorkingData &wd) {
    state.next.reset();

    state.states.getActiveStates(wd.active);

    for (const auto &cur : wd.active) {
        auto u = state.vertices[cur.idx];
        state.states.getSuccessors(cur, state.gc, wd.succ_list);

        for (auto succ : wd.succ_list) {
            auto v = state.vertices[succ.idx];

            if (is_any_accept(v, g)) {
                continue;
            }

            if (!state.allowStartDs && v == g.startDs) {
                continue;
            }

            // GraphCache doesn't differentiate between successors for shadows
            // and helpers, and StateSet does not know anything about the graph,
            // so the only place we can do it is here. we can't self-loop on a
            // startDs if we're startDs's helper, so disallow it.
            if (u == g.startDs && v == g.startDs &&
                succ.level != 0 && succ.level == cur.level) {
                continue;
            }

            // for the reasons outlined above, also putting this here.
            // disallow transitions from start to startDs on levels other than zero
            if (u == g.start && v == g.startDs &&
                cur.level != 0 && succ.level != 0) {
                continue;
            }

            bool can_reach = false;

            if (succ.type == StateSet::State::NODE_HELPER) {
                can_reach = true;
            } else {
                // we assume edge assertions only exist on level 0
                const CharReach &cr = g[v].char_reach;
                NFAEdge e = state.edge_cache.get(u, v);

                if (cr.test(state.cur) &&
                    (!e || canReach(g, e, state))) {
                    can_reach = true;
                }
            }

            // check edge assertions if we are allowed to reach accept
            DEBUG_PRINTF("reaching %zu->%zu ('%c'->'%c'): %s\n",
                         g[u].index, g[v].index,
                         ourisprint(state.prev) ? state.prev : '?',
                         ourisprint(state.cur) ? state.cur : '?',
                         can_reach ? "yes" : "no");

            if (can_reach) {
                // we should use current offset as SOM if:
                //  - we're at level 0 and we're a start vertex
                //  - we're a fake start shadow
                size_t next_som;
                bool reset = is_any_start(u, g) && cur.level == 0;
                reset |= is_virtual_start(u, g) &&
                         cur.type == StateSet::State::NODE_SHADOW;

                if (reset) {
                    next_som = state.offset;
                } else {
                    // else, inherit SOM from predecessor
                    next_som = cur.som;
                }
                succ.som = next_som;

                DEBUG_PRINTF("src: idx %zu level: %u som: %zu type: %s\n",
                             cur.idx, cur.level, cur.som,
                             cur.type == StateSet::State::NODE_HELPER ? "H" : "S");
                DEBUG_PRINTF("dst: idx %zu level: %u som: %zu type: %s\n",
                             succ.idx, succ.level, succ.som,
                             succ.type == StateSet::State::NODE_HELPER ? "H" : "S");

                // activate successor (SOM will be handled by activateState)
                state.next.activateState(succ);
            }
        }
    }
}

// filter extraneous matches
static
void filterMatches(MatchSet &matches) {
    set<size_t> eom;

    // first, collect all end-offset matches
    for (const auto &match : matches) {
        eom.insert(match.second);
    }

    // now, go through all the end-offsets and filter extra matches
    for (const auto &elem : eom) {
        // find minimum SOM for this EOM
        size_t min_som = -1U;
        for (const auto &match : matches) {
            // skip entries with wrong EOM
            if (match.second != elem) {
                continue;
            }

            min_som = min(min_som, match.first);
        }

        auto msit = matches.begin();
        while (msit != matches.end()) {
            // skip everything that doesn't match
            if (msit->second != elem || msit->first <= min_som) {
                ++msit;
                continue;
            }
            DEBUG_PRINTF("erasing match %zu, %zu\n", msit->first, msit->second);
            matches.erase(msit++);
        }
    }
}

/** \brief Find all matches for a given graph when executed against \a input.
 *
 *  Fills \a matches with offsets into the data stream where a match is found.
 */
bool findMatches(const NGHolder &g, const ReportManager &rm,
                 const string &input, MatchSet &matches,
                 const u32 edit_distance, const u32 hamm_distance,
                 const bool notEod, const bool utf8) {
    assert(hasCorrectlyNumberedVertices(g));
    // cannot match fuzzy utf8 patterns, this should've been filtered out at
    // compile time, so make it an assert
    assert(!edit_distance || !utf8);
    // cannot be both edit and Hamming distance at once
    assert(!edit_distance || !hamm_distance);

    bool hamming = hamm_distance > 0;
    auto dist = hamming ? hamm_distance : edit_distance;

    const size_t total_states = num_vertices(g) * (3 * dist + 1);
    DEBUG_PRINTF("Finding matches (%zu total states)\n", total_states);
    if (total_states > STATE_COUNT_MAX) {
        DEBUG_PRINTF("too big\n");
        return false;
    }

    GraphCache gc(edit_distance, hamm_distance, g);
#ifdef DEBUG
    gc.dumpStateTransitionTable(g);
#endif

    const bool allowStartDs = (proper_out_degree(g.startDs, g) > 0);

    struct fmstate state(g, gc, utf8, allowStartDs, dist, rm);

    StateSet::WorkingData wd;

    for (auto it = input.begin(), ite = input.end(); it != ite; ++it) {
#ifdef DEBUG
        state.states.dumpActiveStates();
#endif
        state.offset = std::distance(input.begin(), it);
        state.cur = *it;

        step(g, state, wd);

        getMatches(g, matches, state, wd, false);

        DEBUG_PRINTF("offset %zu, %zu states on\n", state.offset,
                     state.next.count());
        if (state.next.empty()) {
            filterMatches(matches);
            return true;
        }
        state.states = state.next;
        state.prev = state.cur;
    }
#ifdef DEBUG
    state.states.dumpActiveStates();
#endif
    state.offset = input.size();
    state.cur = 0;

    // do additional step to get matches after stream end, this time count eod
    // matches also (or not, if we're in notEod mode)

    DEBUG_PRINTF("Looking for EOD matches\n");
    getMatches(g, matches, state, wd, !notEod);

    filterMatches(matches);
    return true;
}
