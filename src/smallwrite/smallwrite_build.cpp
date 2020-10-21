/*
 * Copyright (c) 2015-2020, Intel Corporation
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
 * \brief Small-write engine build code.
 */

#include "smallwrite/smallwrite_build.h"

#include "grey.h"
#include "ue2common.h"
#include "compiler/compiler.h"
#include "nfa/dfa_min.h"
#include "nfa/mcclellancompile.h"
#include "nfa/mcclellancompile_util.h"
#include "nfa/nfa_internal.h"
#include "nfa/rdfa_merge.h"
#include "nfa/shengcompile.h"
#include "nfagraph/ng.h"
#include "nfagraph/ng_depth.h"
#include "nfagraph/ng_holder.h"
#include "nfagraph/ng_mcclellan.h"
#include "nfagraph/ng_reports.h"
#include "nfagraph/ng_prune.h"
#include "nfagraph/ng_util.h"
#include "smallwrite/smallwrite_internal.h"
#include "util/alloc.h"
#include "util/bytecode_ptr.h"
#include "util/charreach.h"
#include "util/compare.h"
#include "util/compile_context.h"
#include "util/container.h"
#include "util/make_unique.h"
#include "util/ue2_graph.h"
#include "util/ue2string.h"
#include "util/verify_types.h"

#include <map>
#include <set>
#include <vector>
#include <utility>

#include <boost/graph/breadth_first_search.hpp>

using namespace std;

namespace ue2 {

#define DFA_MERGE_MAX_STATES 8000
#define MAX_TRIE_VERTICES 8000

struct LitTrieVertexProps {
    LitTrieVertexProps() = default;
    explicit LitTrieVertexProps(u8 c_in) : c(c_in) {}
    size_t index; // managed by ue2_graph
    u8 c = 0; //!< character reached on this vertex
    flat_set<ReportID> reports; //!< managed reports fired on this vertex
};

struct LitTrieEdgeProps {
    size_t index; // managed by ue2_graph
};

/**
 * \brief BGL graph used to store a trie of literals (for later AC construction
 * into a DFA).
 */
struct LitTrie
    : public ue2_graph<LitTrie, LitTrieVertexProps, LitTrieEdgeProps> {

    LitTrie() : root(add_vertex(*this)) {}

    const vertex_descriptor root; //!< Root vertex for the trie.
};

static
bool is_empty(const LitTrie &trie) {
    return num_vertices(trie) <= 1;
}

static
std::set<ReportID> all_reports(const LitTrie &trie) {
    std::set<ReportID> reports;
    for (auto v : vertices_range(trie)) {
        insert(&reports, trie[v].reports);
    }
    return reports;
}

using LitTrieVertex = LitTrie::vertex_descriptor;
using LitTrieEdge = LitTrie::edge_descriptor;

namespace { // unnamed

// Concrete impl class
class SmallWriteBuildImpl : public SmallWriteBuild {
public:
    SmallWriteBuildImpl(size_t num_patterns, const ReportManager &rm,
                        const CompileContext &cc);

    // Construct a runtime implementation.
    bytecode_ptr<SmallWriteEngine> build(u32 roseQuality) override;

    void add(const NGHolder &g, const ExpressionInfo &expr) override;
    void add(const ue2_literal &literal, ReportID r) override;

    set<ReportID> all_reports() const override;

    const ReportManager &rm;
    const CompileContext &cc;

    vector<unique_ptr<raw_dfa>> dfas;
    LitTrie lit_trie;
    LitTrie lit_trie_nocase;
    size_t num_literals = 0;
    bool poisoned;
};

} // namespace

SmallWriteBuild::~SmallWriteBuild() = default;

SmallWriteBuildImpl::SmallWriteBuildImpl(size_t num_patterns,
                                         const ReportManager &rm_in,
                                         const CompileContext &cc_in)
    : rm(rm_in), cc(cc_in),
      /* small write is block mode only */
      poisoned(!cc.grey.allowSmallWrite
               || cc.streaming
               || num_patterns > cc.grey.smallWriteMaxPatterns) {
}

/**
 * \brief Remove any reports from the given vertex that cannot match within
 * max_depth due to their constraints.
 */
static
bool pruneOverlongReports(NFAVertex v, NGHolder &g, const depth &max_depth,
                          const ReportManager &rm) {
    assert(!g[v].reports.empty());

    vector<ReportID> bad_reports;

    for (ReportID id : g[v].reports) {
        const auto &report = rm.getReport(id);
        if (report.minOffset > max_depth) {
            bad_reports.push_back(id);
        }
    }

    for (ReportID id : bad_reports) {
        g[v].reports.erase(id);
    }

    if (g[v].reports.empty()) {
        DEBUG_PRINTF("none of vertex %zu's reports can match, cut accepts\n",
                     g[v].index);
        remove_edge(v, g.accept, g);
        remove_edge(v, g.acceptEod, g);
    }

    return !bad_reports.empty();
}

/**
 * \brief Prune vertices and reports from the graph that cannot match within
 * max_depth.
 */
static
bool pruneOverlong(NGHolder &g, const depth &max_depth,
                   const ReportManager &rm) {
    bool modified = false;
    auto depths = calcBidiDepths(g);

    for (auto v : vertices_range(g)) {
        if (is_special(v, g)) {
            continue;
        }
        const auto &d = depths.at(g[v].index);
        depth min_match_offset = min(d.fromStart.min, d.fromStartDotStar.min)
                               + min(d.toAccept.min, d.toAcceptEod.min);
        if (min_match_offset > max_depth) {
            clear_vertex(v, g);
            modified = true;
            continue;
        }

        if (is_match_vertex(v, g)) {
            modified |= pruneOverlongReports(v, g, max_depth, rm);
        }
    }

    if (modified) {
        pruneUseless(g);
        DEBUG_PRINTF("pruned graph down to %zu vertices\n", num_vertices(g));
    }

    return modified;
}

/**
 * \brief Attempt to merge the set of DFAs given down into a single raw_dfa.
 * Returns false on failure.
 */
static
bool mergeDfas(vector<unique_ptr<raw_dfa>> &dfas, const ReportManager &rm,
               const CompileContext &cc) {
    assert(!dfas.empty());

    if (dfas.size() == 1) {
        return true;
    }

    DEBUG_PRINTF("attempting to merge %zu DFAs\n", dfas.size());

    vector<const raw_dfa *> dfa_ptrs;
    dfa_ptrs.reserve(dfas.size());
    for (auto &d : dfas) {
        dfa_ptrs.push_back(d.get());
    }

    auto merged = mergeAllDfas(dfa_ptrs, DFA_MERGE_MAX_STATES, &rm, cc.grey);
    if (!merged) {
        DEBUG_PRINTF("merge failed\n");
        return false;
    }

    DEBUG_PRINTF("merge succeeded, result has %zu states\n",
                  merged->states.size());
    dfas.clear();
    dfas.push_back(std::move(merged));
    return true;
}

void SmallWriteBuildImpl::add(const NGHolder &g, const ExpressionInfo &expr) {
    // If the graph is poisoned (i.e. we can't build a SmallWrite version),
    // we don't even try.
    if (poisoned) {
        return;
    }

    if (expr.som) {
        DEBUG_PRINTF("no SOM support in small-write engine\n");
        poisoned = true;
        return;
    }

    if (isVacuous(g)) {
        DEBUG_PRINTF("no vacuous graph support in small-write engine\n");
        poisoned = true;
        return;
    }

    if (any_of_in(::ue2::all_reports(g), [&](ReportID id) {
            return rm.getReport(id).minLength > 0;
        })) {
        DEBUG_PRINTF("no min_length extparam support in small-write engine\n");
        poisoned = true;
        return;
    }

    DEBUG_PRINTF("g=%p\n", &g);

    // make a copy of the graph so that we can modify it for our purposes
    unique_ptr<NGHolder> h = cloneHolder(g);

    pruneOverlong(*h, depth(cc.grey.smallWriteLargestBuffer), rm);

    reduceGraph(*h, SOM_NONE, expr.utf8, cc);

    if (can_never_match(*h)) {
        DEBUG_PRINTF("graph can never match in small block\n");
        return;
    }

    // Now we can actually build the McClellan DFA
    assert(h->kind == NFA_OUTFIX);
    auto r = buildMcClellan(*h, &rm, cc.grey);

    // If we couldn't build a McClellan DFA for this portion, we won't be able
    // build a smwr which represents the pattern set
    if (!r) {
        DEBUG_PRINTF("failed to determinise\n");
        poisoned = true;
        return;
    }

    if (clear_deeper_reports(*r, cc.grey.smallWriteLargestBuffer)) {
        minimize_hopcroft(*r, cc.grey);
    }

    dfas.push_back(std::move(r));

    if (dfas.size() >= cc.grey.smallWriteMergeBatchSize) {
        if (!mergeDfas(dfas, rm, cc)) {
            dfas.clear();
            poisoned = true;
            return;
        }
    }
}

static
bool add_to_trie(const ue2_literal &literal, ReportID report, LitTrie &trie) {
    auto u = trie.root;
    for (const auto &c : literal) {
        auto next = LitTrie::null_vertex();
        for (auto v : adjacent_vertices_range(u, trie)) {
            if (trie[v].c == (u8)c.c) {
                next = v;
                break;
            }
        }
        if (!next) {
            next = add_vertex(LitTrieVertexProps((u8)c.c), trie);
            add_edge(u, next, trie);
        }
        u = next;
    }

    trie[u].reports.insert(report);

    DEBUG_PRINTF("added '%s' (report %u) to trie, now %zu vertices\n",
                  escapeString(literal).c_str(), report, num_vertices(trie));
    return num_vertices(trie) <= MAX_TRIE_VERTICES;
}

void SmallWriteBuildImpl::add(const ue2_literal &literal, ReportID r) {
    // If the graph is poisoned (i.e. we can't build a SmallWrite version),
    // we don't even try.
    if (poisoned) {
        DEBUG_PRINTF("poisoned\n");
        return;
    }

    if (literal.length() > cc.grey.smallWriteLargestBuffer) {
        DEBUG_PRINTF("exceeded length limit\n");
        return; /* too long */
    }

    if (++num_literals > cc.grey.smallWriteMaxLiterals) {
        DEBUG_PRINTF("exceeded literal limit\n");
        poisoned = true;
        return;
    }

    auto &trie = literal.any_nocase() ? lit_trie_nocase : lit_trie;
    if (!add_to_trie(literal, r, trie)) {
        DEBUG_PRINTF("trie add failed\n");
        poisoned = true;
    }
}

namespace {

/**
 * \brief BFS visitor for Aho-Corasick automaton construction.
 *
 * This is doing two things:
 *
 *   - Computing the failure edges (also called fall or supply edges) for each
 *     vertex, giving the longest suffix of the path to that point that is also
 *     a prefix in the trie reached on the same character. The BFS traversal
 *     makes it possible to build these from earlier failure paths.
 *
 *   - Computing the output function for each vertex, which is done by
 *     propagating the reports from failure paths as well. This ensures that
 *     substrings of the current path also report correctly.
 */
struct ACVisitor : public boost::default_bfs_visitor {
    ACVisitor(LitTrie &trie_in,
              unordered_map<LitTrieVertex, LitTrieVertex> &failure_map_in,
              vector<LitTrieVertex> &ordering_in)
        : mutable_trie(trie_in), failure_map(failure_map_in),
          ordering(ordering_in) {}

    LitTrieVertex find_failure_target(LitTrieVertex u, LitTrieVertex v,
                                      const LitTrie &trie) {
        assert(u == trie.root || contains(failure_map, u));
        assert(!contains(failure_map, v));

        const auto &c = trie[v].c;

        while (u != trie.root) {
            auto f = failure_map.at(u);
            for (auto w : adjacent_vertices_range(f, trie)) {
                if (trie[w].c == c) {
                    return w;
                }
            }
            u = f;
        }

        DEBUG_PRINTF("no failure edge\n");
        return LitTrie::null_vertex();
    }

    void tree_edge(LitTrieEdge e, const LitTrie &trie) {
        auto u = source(e, trie);
        auto v = target(e, trie);
        DEBUG_PRINTF("bfs (%zu, %zu) on '%c'\n", trie[u].index, trie[v].index,
                     trie[v].c);
        ordering.push_back(v);

        auto f = find_failure_target(u, v, trie);

        if (f) {
            DEBUG_PRINTF("final failure vertex %zu\n", trie[f].index);
            failure_map.emplace(v, f);

            // Propagate reports from failure path to ensure we correctly
            // report substrings.
            insert(&mutable_trie[v].reports, mutable_trie[f].reports);
        } else {
            DEBUG_PRINTF("final failure vertex root\n");
            failure_map.emplace(v, trie.root);
        }
    }

private:
    LitTrie &mutable_trie; //!< For setting reports property.
    unordered_map<LitTrieVertex, LitTrieVertex> &failure_map;
    vector<LitTrieVertex> &ordering; //!< BFS ordering for vertices.
};
}

static UNUSED
bool isSaneTrie(const LitTrie &trie) {
    CharReach seen;
    for (auto u : vertices_range(trie)) {
        seen.clear();
        for (auto v : adjacent_vertices_range(u, trie)) {
            if (seen.test(trie[v].c)) {
                return false;
            }
            seen.set(trie[v].c);
        }
    }
    return true;
}

/**
 * \brief Turn the given literal trie into an AC automaton by adding additional
 * edges and reports.
 */
static
void buildAutomaton(LitTrie &trie,
                    unordered_map<LitTrieVertex, LitTrieVertex> &failure_map,
                    vector<LitTrieVertex> &ordering) {
    assert(isSaneTrie(trie));

    // Find our failure transitions and reports.
    failure_map.reserve(num_vertices(trie));
    ordering.reserve(num_vertices(trie));
    ACVisitor ac_vis(trie, failure_map, ordering);
    boost::breadth_first_search(trie, trie.root, visitor(ac_vis));

    // Compute missing edges from failure map.
    for (auto v : ordering) {
        DEBUG_PRINTF("vertex %zu\n", trie[v].index);
        CharReach seen;
        for (auto w : adjacent_vertices_range(v, trie)) {
            DEBUG_PRINTF("edge to %zu with reach 0x%02x\n", trie[w].index,
                         trie[w].c);
            assert(!seen.test(trie[w].c));
            seen.set(trie[w].c);
        }
        auto parent = failure_map.at(v);
        for (auto w : adjacent_vertices_range(parent, trie)) {
            if (!seen.test(trie[w].c)) {
                add_edge(v, w, trie);
            }
        }
    }
}

static
vector<u32> findDistFromRoot(const LitTrie &trie) {
    vector<u32> dist(num_vertices(trie), UINT32_MAX);
    dist[trie[trie.root].index] = 0;

    // BFS to find dist from root.
    breadth_first_search(
        trie, trie.root,
        visitor(make_bfs_visitor(record_distances(
            make_iterator_property_map(dist.begin(),
                                       get(&LitTrieVertexProps::index, trie)),
            boost::on_tree_edge()))));

    return dist;
}

static
vector<u32> findDistToAccept(const LitTrie &trie) {
    vector<u32> dist(num_vertices(trie), UINT32_MAX);

    // Start with all reporting vertices.
    deque<LitTrieVertex> q;
    for (auto v : vertices_range(trie)) {
        if (!trie[v].reports.empty()) {
            q.push_back(v);
            dist[trie[v].index] = 0;
        }
    }

    // Custom BFS, since we have a pile of sources.
    while (!q.empty()) {
        auto v = q.front();
        q.pop_front();
        u32 d = dist[trie[v].index];

        for (auto u : inv_adjacent_vertices_range(v, trie)) {
            auto &u_dist = dist[trie[u].index];
            if (u_dist == UINT32_MAX) {
                q.push_back(u);
                u_dist = d + 1;
            }
        }
    }

    return dist;
}

/**
 * \brief Prune all vertices from the trie that do not lie on a path from root
 * to accept of length <= max_depth.
 */
static
void pruneTrie(LitTrie &trie, u32 max_depth) {
    DEBUG_PRINTF("pruning trie to %u\n", max_depth);

    auto dist_from_root = findDistFromRoot(trie);
    auto dist_to_accept = findDistToAccept(trie);

    vector<LitTrieVertex> dead;
    for (auto v : vertices_range(trie)) {
        if (v == trie.root) {
            continue;
        }
        auto v_index = trie[v].index;
        DEBUG_PRINTF("vertex %zu: from_start=%u, to_accept=%u\n", trie[v].index,
                     dist_from_root[v_index], dist_to_accept[v_index]);
        assert(dist_from_root[v_index] != UINT32_MAX);
        assert(dist_to_accept[v_index] != UINT32_MAX);
        u32 min_path_len = dist_from_root[v_index] + dist_to_accept[v_index];
        if (min_path_len > max_depth) {
            DEBUG_PRINTF("pruning vertex %zu (min path len %u)\n",
                         trie[v].index, min_path_len);
            clear_vertex(v, trie);
            dead.push_back(v);
        }
    }

    if (dead.empty()) {
        return;
    }

    for (auto v : dead) {
        remove_vertex(v, trie);
    }

    DEBUG_PRINTF("%zu vertices remain\n", num_vertices(trie));

    renumber_edges(trie);
    renumber_vertices(trie);
}

static
vector<CharReach> getAlphabet(const LitTrie &trie, bool nocase) {
    vector<CharReach> esets = {CharReach::dot()};
    for (auto v : vertices_range(trie)) {
        if (v == trie.root) {
            continue;
        }

        CharReach cr;
        if (nocase) {
            cr.set(mytoupper(trie[v].c));
            cr.set(mytolower(trie[v].c));
        } else {
            cr.set(trie[v].c);
        }

        for (size_t i = 0; i < esets.size(); i++) {
            if (esets[i].count() == 1) {
                continue;
            }

            CharReach t = cr & esets[i];
            if (t.any() && t != esets[i]) {
                esets[i] &= ~t;
                esets.push_back(t);
            }
        }
    }

    // For deterministic compiles.
    sort(esets.begin(), esets.end());
    return esets;
}

static
u16 buildAlphabet(const LitTrie &trie, bool nocase,
                  array<u16, ALPHABET_SIZE> &alpha,
                  array<u16, ALPHABET_SIZE> &unalpha) {
    const auto &esets = getAlphabet(trie, nocase);

    u16 i = 0;
    for (const auto &cr : esets) {
        u16 leader = cr.find_first();
        for (size_t s = cr.find_first(); s != cr.npos; s = cr.find_next(s)) {
            alpha[s] = i;
        }
        unalpha[i] = leader;
        i++;
    }

    for (u16 j = N_CHARS; j < ALPHABET_SIZE; j++, i++) {
        alpha[j] = i;
        unalpha[i] = j;
    }

    DEBUG_PRINTF("alphabet size %u\n", i);
    return i;
}

/**
 * \brief Calculate state mapping, from vertex in trie to state index in BFS
 * ordering.
 */
static
unordered_map<LitTrieVertex, u32>
makeStateMap(const LitTrie &trie, const vector<LitTrieVertex> &ordering) {
    unordered_map<LitTrieVertex, u32> state_ids;
    state_ids.reserve(num_vertices(trie));
    u32 idx = DEAD_STATE + 1;
    state_ids.emplace(trie.root, idx++);
    for (auto v : ordering) {
        state_ids.emplace(v, idx++);
    }
    assert(state_ids.size() == num_vertices(trie));
    return state_ids;
}

/** \brief Construct a raw_dfa from a literal trie. */
static
unique_ptr<raw_dfa> buildDfa(LitTrie &trie, bool nocase) {
    DEBUG_PRINTF("trie has %zu states\n", num_vertices(trie));

    vector<LitTrieVertex> ordering;
    unordered_map<LitTrieVertex, LitTrieVertex> failure_map;
    buildAutomaton(trie, failure_map, ordering);

    // Construct DFA states in BFS order.
    const auto state_ids = makeStateMap(trie, ordering);

    auto rdfa = make_unique<raw_dfa>(NFA_OUTFIX);

    // Calculate alphabet.
    array<u16, ALPHABET_SIZE> unalpha;
    auto &alpha = rdfa->alpha_remap;
    rdfa->alpha_size = buildAlphabet(trie, nocase, alpha, unalpha);

    // Construct states and transitions.
    const u16 root_state = state_ids.at(trie.root);
    assert(root_state == DEAD_STATE + 1);
    rdfa->start_anchored = root_state;
    rdfa->start_floating = root_state;
    rdfa->states.resize(num_vertices(trie) + 1, dstate(rdfa->alpha_size));

    // Dead state.
    fill(rdfa->states[DEAD_STATE].next.begin(),
         rdfa->states[DEAD_STATE].next.end(), DEAD_STATE);

    for (auto u : vertices_range(trie)) {
        auto u_state = state_ids.at(u);
        DEBUG_PRINTF("state %u\n", u_state);
        assert(u_state < rdfa->states.size());
        auto &ds = rdfa->states[u_state];
        ds.reports = trie[u].reports;
        if (!ds.reports.empty()) {
            DEBUG_PRINTF("reports: %s\n", as_string_list(ds.reports).c_str());
        }

        // Set daddy state from failure map.
        if (u == trie.root) {
            ds.daddy = DEAD_STATE;
        } else {
            assert(contains(failure_map, u));
            ds.daddy = state_ids.at(failure_map.at(u));
        }

        // By default, transition back to the root.
        fill(ds.next.begin(), ds.next.end(), root_state);
        // TOP should be a self-loop.
        ds.next[alpha[TOP]] = u_state;

        // Add in the real transitions.
        for (auto v : adjacent_vertices_range(u, trie)) {
            if (v == trie.root) {
                continue;
            }
            auto v_state = state_ids.at(v);
            u16 sym = alpha[trie[v].c];
            DEBUG_PRINTF("edge to %u on 0x%02x (sym %u)\n", v_state,
                         trie[v].c, sym);
            assert(sym < ds.next.size());
            assert(ds.next[sym] == root_state);
            ds.next[sym] = v_state;
        }
    }

    return rdfa;
}

#define MAX_GOOD_ACCEL_DEPTH 4

static
bool is_slow(const raw_dfa &rdfa, const set<dstate_id_t> &accel,
             u32 roseQuality) {
    /* we consider a dfa as slow if there is no way to quickly get into an accel
     * state/dead state. In these cases, it is more likely that we will be
     * running at our unaccelerated dfa speeds so the small write engine is only
     * competitive over a small region where start up costs are dominant. */

    if (roseQuality) {
        return true;
    }

    set<dstate_id_t> visited;
    set<dstate_id_t> next;
    set<dstate_id_t> curr;
    curr.insert(rdfa.start_anchored);

    u32 ialpha_size = rdfa.getImplAlphaSize();

    for (u32 i = 0; i < MAX_GOOD_ACCEL_DEPTH; i++) {
        next.clear();
        for (dstate_id_t s : curr) {
            if (contains(visited, s)) {
                continue;
            }
            visited.insert(s);
            if (s == DEAD_STATE || contains(accel, s)) {
                return false;
            }

            for (size_t j = 0; j < ialpha_size; j++) {
                next.insert(rdfa.states[s].next[j]);
            }
        }
        curr.swap(next);
    }

    return true;
}

static
bytecode_ptr<NFA> getDfa(raw_dfa &rdfa, const CompileContext &cc,
                         const ReportManager &rm, bool has_non_literals,
                         set<dstate_id_t> &accel_states) {
    // If we determinised only literals, then we only need to consider the init
    // states for acceleration.
    bool only_accel_init = !has_non_literals;
    bool trust_daddy_states = !has_non_literals;

    bytecode_ptr<NFA> dfa = nullptr;
    if (cc.grey.allowSmallWriteSheng) {
        dfa = shengCompile(rdfa, cc, rm, only_accel_init, &accel_states);
        if (!dfa) {
            dfa = sheng32Compile(rdfa, cc, rm, only_accel_init, &accel_states);
        }
        if (!dfa) {
            dfa = sheng64Compile(rdfa, cc, rm, only_accel_init, &accel_states);
        }
    }
    if (!dfa) {
        dfa = mcclellanCompile(rdfa, cc, rm, only_accel_init,
                               trust_daddy_states, &accel_states);
    }
    return dfa;
}

static
bytecode_ptr<NFA> prepEngine(raw_dfa &rdfa, u32 roseQuality,
                             const CompileContext &cc, const ReportManager &rm,
                             bool has_non_literals, u32 *start_offset,
                             u32 *small_region) {
    *start_offset = remove_leading_dots(rdfa);

    // Unleash the McClellan!
    set<dstate_id_t> accel_states;

    auto nfa = getDfa(rdfa, cc, rm, has_non_literals, accel_states);
    if (!nfa) {
        DEBUG_PRINTF("DFA compile failed for smallwrite NFA\n");
        return nullptr;
    }

    if (is_slow(rdfa, accel_states, roseQuality)) {
        DEBUG_PRINTF("is slow\n");
        *small_region = cc.grey.smallWriteLargestBufferBad;
        if (*small_region <= *start_offset) {
            return nullptr;
        }
        if (clear_deeper_reports(rdfa, *small_region - *start_offset)) {
            minimize_hopcroft(rdfa, cc.grey);
            if (rdfa.start_anchored == DEAD_STATE) {
                DEBUG_PRINTF("all patterns pruned out\n");
                return nullptr;
            }

            nfa = getDfa(rdfa, cc, rm, has_non_literals, accel_states);
            if (!nfa) {
                DEBUG_PRINTF("DFA compile failed for smallwrite NFA\n");
                assert(0); /* able to build orig dfa but not the trimmed? */
                return nullptr;
            }
        }
    } else {
        *small_region = cc.grey.smallWriteLargestBuffer;
    }

    assert(isDfaType(nfa->type));
    if (nfa->length > cc.grey.limitSmallWriteOutfixSize
        || nfa->length > cc.grey.limitDFASize) {
        DEBUG_PRINTF("smallwrite outfix size too large\n");
        return nullptr; /* this is just a soft failure - don't build smwr */
    }

    nfa->queueIndex = 0; /* dummy, small write API does not use queue */
    return nfa;
}

// SmallWriteBuild factory
unique_ptr<SmallWriteBuild> makeSmallWriteBuilder(size_t num_patterns,
                                                  const ReportManager &rm,
                                                  const CompileContext &cc) {
    return ue2::make_unique<SmallWriteBuildImpl>(num_patterns, rm, cc);
}

bytecode_ptr<SmallWriteEngine> SmallWriteBuildImpl::build(u32 roseQuality) {
    const bool has_literals = !is_empty(lit_trie) || !is_empty(lit_trie_nocase);
    const bool has_non_literals = !dfas.empty();
    if (dfas.empty() && !has_literals) {
        DEBUG_PRINTF("no smallwrite engine\n");
        poisoned = true;
        return nullptr;
    }

    if (poisoned) {
        DEBUG_PRINTF("some pattern could not be made into a smallwrite dfa\n");
        return nullptr;
    }

    // We happen to know that if the rose is high quality, we're going to limit
    // depth further.
    if (roseQuality) {
        u32 max_depth = cc.grey.smallWriteLargestBufferBad;
        if (!is_empty(lit_trie)) {
            pruneTrie(lit_trie, max_depth);
        }
        if (!is_empty(lit_trie_nocase)) {
            pruneTrie(lit_trie_nocase, max_depth);
        }
    }

    if (!is_empty(lit_trie)) {
        dfas.push_back(buildDfa(lit_trie, false));
        DEBUG_PRINTF("caseful literal dfa with %zu states\n",
                     dfas.back()->states.size());
    }
    if (!is_empty(lit_trie_nocase)) {
        dfas.push_back(buildDfa(lit_trie_nocase, true));
        DEBUG_PRINTF("nocase literal dfa with %zu states\n",
                     dfas.back()->states.size());
    }

    if (dfas.empty()) {
        DEBUG_PRINTF("no dfa, pruned everything away\n");
        return nullptr;
    }

    if (!mergeDfas(dfas, rm, cc)) {
        dfas.clear();
        return nullptr;
    }

    assert(dfas.size() == 1);
    auto rdfa = std::move(dfas.front());
    dfas.clear();

    DEBUG_PRINTF("building rdfa %p\n", rdfa.get());

    u32 start_offset;
    u32 small_region;
    auto nfa = prepEngine(*rdfa, roseQuality, cc, rm, has_non_literals,
                          &start_offset, &small_region);
    if (!nfa) {
        DEBUG_PRINTF("some smallwrite outfix could not be prepped\n");
        /* just skip the smallwrite optimization */
        poisoned = true;
        return nullptr;
    }

    u32 size = sizeof(SmallWriteEngine) + nfa->length;
    auto smwr = make_zeroed_bytecode_ptr<SmallWriteEngine>(size);

    smwr->size = size;
    smwr->start_offset = start_offset;
    smwr->largestBuffer = small_region;

    /* copy in nfa after the smwr */
    assert(ISALIGNED_CL(smwr.get() + 1));
    memcpy(smwr.get() + 1, nfa.get(), nfa->length);

    DEBUG_PRINTF("smallwrite done %p\n", smwr.get());
    return smwr;
}

set<ReportID> SmallWriteBuildImpl::all_reports() const {
    set<ReportID> reports;
    if (poisoned) {
        return reports;
    }

    for (const auto &rdfa : dfas) {
        insert(&reports, ::ue2::all_reports(*rdfa));
    }

    insert(&reports, ::ue2::all_reports(lit_trie));
    insert(&reports, ::ue2::all_reports(lit_trie_nocase));

    return reports;
}

} // namespace ue2
