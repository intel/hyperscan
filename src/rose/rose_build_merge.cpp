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
 * \brief Rose Build: functions for reducing the size of the Rose graph
 * through merging.
 */
#include "rose_build_merge.h"

#include "grey.h"
#include "rose_build.h"
#include "rose_build_impl.h"
#include "rose_build_util.h"
#include "ue2common.h"
#include "nfa/castlecompile.h"
#include "nfa/goughcompile.h"
#include "nfa/limex_limits.h"
#include "nfa/mcclellancompile.h"
#include "nfa/nfa_build_util.h"
#include "nfa/rdfa_merge.h"
#include "nfagraph/ng_holder.h"
#include "nfagraph/ng_haig.h"
#include "nfagraph/ng_is_equal.h"
#include "nfagraph/ng_lbr.h"
#include "nfagraph/ng_limex.h"
#include "nfagraph/ng_mcclellan.h"
#include "nfagraph/ng_puff.h"
#include "nfagraph/ng_redundancy.h"
#include "nfagraph/ng_repeat.h"
#include "nfagraph/ng_reports.h"
#include "nfagraph/ng_stop.h"
#include "nfagraph/ng_uncalc_components.h"
#include "nfagraph/ng_util.h"
#include "nfagraph/ng_width.h"
#include "util/bitutils.h"
#include "util/charreach.h"
#include "util/compile_context.h"
#include "util/container.h"
#include "util/dump_charclass.h"
#include "util/graph_range.h"
#include "util/order_check.h"
#include "util/report_manager.h"
#include "util/ue2string.h"

#include <algorithm>
#include <functional>
#include <list>
#include <map>
#include <queue>
#include <set>
#include <string>
#include <vector>
#include <utility>

#include <boost/functional/hash/hash_fwd.hpp>
#include <boost/range/adaptor/map.hpp>

using namespace std;
using boost::adaptors::map_values;
using boost::hash_combine;

namespace ue2 {

static const size_t NARROW_START_MAX = 10;
static const size_t SMALL_MERGE_MAX_VERTICES_STREAM = 128;
static const size_t SMALL_MERGE_MAX_VERTICES_BLOCK = 64;
static const size_t SMALL_ROSE_THRESHOLD_STREAM = 32;
static const size_t SMALL_ROSE_THRESHOLD_BLOCK = 10;
static const size_t MERGE_GROUP_SIZE_MAX = 200;
static const size_t MERGE_CASTLE_GROUP_SIZE_MAX = 1000;

/** \brief Max number of DFAs (McClellan, Haig) to pairwise merge together. */
static const size_t DFA_CHUNK_SIZE_MAX = 200;

/** \brief Max DFA states in a merged DFA. */
static const size_t DFA_MERGE_MAX_STATES = 8000;

/** \brief An LBR must have at least this many vertices to be protected from
 * merging with other graphs. */
static const size_t LARGE_LBR_MIN_VERTICES = 32;

/** \brief In block mode, merge two prefixes even if they don't have identical
 * literal sets if they have fewer than this many states and the merged graph
 * is also small. */
static constexpr size_t MAX_BLOCK_PREFIX_MERGE_VERTICES = 32;

static
size_t small_merge_max_vertices(const CompileContext &cc) {
    return cc.streaming ? SMALL_MERGE_MAX_VERTICES_STREAM
                        : SMALL_MERGE_MAX_VERTICES_BLOCK;
}

static
size_t small_rose_threshold(const CompileContext &cc) {
    return cc.streaming ? SMALL_ROSE_THRESHOLD_STREAM
                        : SMALL_ROSE_THRESHOLD_BLOCK;
}

static
bool isLargeLBR(const NGHolder &g, const Grey &grey) {
    if (num_vertices(g) < LARGE_LBR_MIN_VERTICES) {
        return false;
    }
    return isLBR(g, grey);
}

namespace {
struct DupeLeafKey {
    explicit DupeLeafKey(const RoseVertexProps &litv)
        : literals(litv.literals), reports(litv.reports),
          eod_accept(litv.eod_accept), suffix(litv.suffix), left(litv.left),
          som_adjust(litv.som_adjust) {
        DEBUG_PRINTF("eod_accept %d\n", (int)eod_accept);
        DEBUG_PRINTF("report %u\n", left.leftfix_report);
        DEBUG_PRINTF("lag %u\n", left.lag);
    }

    bool operator<(const DupeLeafKey &b) const {
        const DupeLeafKey &a = *this;
        ORDER_CHECK(literals);
        ORDER_CHECK(eod_accept);
        ORDER_CHECK(suffix);
        ORDER_CHECK(reports);
        ORDER_CHECK(som_adjust);
        ORDER_CHECK(left.leftfix_report);
        ORDER_CHECK(left.lag);
        return false;
    }

    flat_set<u32> literals;
    flat_set<ReportID> reports;
    bool eod_accept;
    suffix_id suffix;
    LeftEngInfo left;
    u32 som_adjust;
};

struct UncalcLeafKey {
    UncalcLeafKey(const RoseGraph &g, RoseVertex v)
        : literals(g[v].literals), rose(g[v].left) {
        for (const auto &e : in_edges_range(v, g)) {
            RoseVertex u = source(e, g);
            preds.insert(make_pair(u, g[e]));
        }
    }

    bool operator<(const UncalcLeafKey &b) const {
        const UncalcLeafKey &a = *this;
        ORDER_CHECK(literals);
        ORDER_CHECK(preds);
        ORDER_CHECK(rose);
        return false;
    }

    flat_set<u32> literals;
    flat_set<pair<RoseVertex, RoseEdgeProps>> preds;
    LeftEngInfo rose;
};
} // namespace

/**
 * This function merges leaf vertices with the same literals and report
 * id/suffix. The leaf vertices of the graph are inspected and a mapping of
 * leaf vertex properties to vertices is built. If the same set of leaf
 * properties has already been seen when we inspect a vertex, we attempt to
 * merge the vertex in with the previously seen vertex. This process can fail
 * if the vertices share a common predecessor vertex but have a differing,
 * incompatible relationship (different bounds or infix) with the predecessor.
 *
 * This takes place after \ref dedupeSuffixes to increase effectiveness as the
 * same suffix is required for a merge to occur.
 */
void mergeDupeLeaves(RoseBuildImpl &tbi) {
    map<DupeLeafKey, RoseVertex> leaves;
    vector<RoseVertex> changed;

    RoseGraph &g = tbi.g;
    for (auto v : vertices_range(g)) {
        if (in_degree(v, g) == 0) {
            assert(tbi.isAnyStart(v));
            continue;
        }

        DEBUG_PRINTF("inspecting vertex index=%zu in_degree %zu "
                     "out_degree %zu\n", g[v].index, in_degree(v, g),
                     out_degree(v, g));

        // Vertex must be a reporting leaf node
        if (g[v].reports.empty() || !isLeafNode(v, g)) {
            continue;
        }

        // At the moment, we ignore all successors of root or anchored_root,
        // since many parts of our runtime assume that these have in-degree 1.
        if (tbi.isRootSuccessor(v)) {
            continue;
        }

        DupeLeafKey dupe(g[v]);
        if (leaves.find(dupe) == leaves.end()) {
            leaves.insert(make_pair(dupe, v));
            continue;
        }

        RoseVertex t = leaves.find(dupe)->second;
        DEBUG_PRINTF("found two leaf dupe roles, index=%zu,%zu\n", g[v].index,
                     g[t].index);

        vector<RoseEdge> deadEdges;
        for (const auto &e : in_edges_range(v, g)) {
            RoseVertex u = source(e, g);
            DEBUG_PRINTF("u index=%zu\n", g[u].index);
            if (RoseEdge et = edge(u, t, g)) {
                if (g[et].minBound <= g[e].minBound
                    && g[et].maxBound >= g[e].maxBound) {
                    DEBUG_PRINTF("remove more constrained edge\n");
                    deadEdges.push_back(e);
                }
            } else {
                DEBUG_PRINTF("rehome edge: add %zu->%zu\n", g[u].index,
                             g[t].index);
                add_edge(u, t, g[e], g);
                deadEdges.push_back(e);
            }
        }

        if (!deadEdges.empty()) {
            for (auto &e : deadEdges) {
                remove_edge(e, g);
            }
            changed.push_back(v);
            g[t].min_offset = min(g[t].min_offset, g[v].min_offset);
            g[t].max_offset = max(g[t].max_offset, g[v].max_offset);
        }
    }
    DEBUG_PRINTF("find loop done\n");

    // Remove any vertices that now have no in-edges.
    size_t countRemovals = 0;
    for (size_t i = 0; i < changed.size(); i++) {
        RoseVertex v = changed[i];
        if (in_degree(v, g) == 0) {
            DEBUG_PRINTF("remove vertex\n");
            if (!tbi.isVirtualVertex(v)) {
                for (u32 lit_id : g[v].literals) {
                    tbi.literal_info[lit_id].vertices.erase(v);
                }
            }
            remove_vertex(v, g);
            countRemovals++;
        }
    }

    // if we've removed anything, we need to renumber vertices
    if (countRemovals) {
        renumber_vertices(g);
        DEBUG_PRINTF("removed %zu vertices.\n", countRemovals);
    }
}

/** Merges the suffixes on the (identical) vertices in \a vcluster, used by
 * \ref uncalcLeaves. */
static
void mergeCluster(RoseGraph &g, const ReportManager &rm,
                  const vector<RoseVertex> &vcluster,
                  vector<RoseVertex> &dead, const CompileContext &cc) {
    if (vcluster.size() <= 1) {
        return; // No merge to perform.
    }

    // Note that we batch merges up fairly crudely for performance reasons.
    vector<RoseVertex>::const_iterator it = vcluster.begin(), it2;
    while (it != vcluster.end()) {
        vector<NGHolder *> cluster;
        map<NGHolder *, RoseVertex> rev;

        for (it2 = it;
             it2 != vcluster.end() && cluster.size() < MERGE_GROUP_SIZE_MAX;
             ++it2) {
            RoseVertex v = *it2;
            NGHolder *h = g[v].suffix.graph.get();
            assert(!g[v].suffix.haig); /* should not be here if haig */
            rev[h] = v;
            cluster.push_back(h);
        }
        it = it2;

        DEBUG_PRINTF("merging cluster %zu\n", cluster.size());
        auto merged = mergeNfaCluster(cluster, &rm, cc);
        DEBUG_PRINTF("done\n");

        for (const auto &m : merged) {
            NGHolder *h_victim = m.first; // mergee
            NGHolder *h_winner = m.second;
            RoseVertex victim = rev[h_victim];
            RoseVertex winner = rev[h_winner];

            LIMIT_TO_AT_MOST(&g[winner].min_offset, g[victim].min_offset);
            ENSURE_AT_LEAST(&g[winner].max_offset, g[victim].max_offset);
            insert(&g[winner].reports, g[victim].reports);

            dead.push_back(victim);
        }
    }
}

static
void findUncalcLeavesCandidates(RoseBuildImpl &tbi,
                           map<UncalcLeafKey, vector<RoseVertex> > &clusters,
                           deque<UncalcLeafKey> &ordered) {
    const RoseGraph &g = tbi.g;

    vector<RoseVertex> suffix_vertices; // vertices with suffix graphs
    ue2::unordered_map<const NGHolder *, u32> fcount; // ref count per graph

    for (auto v : vertices_range(g)) {
        if (g[v].suffix) {
            if (!g[v].suffix.graph) {
                continue; /* cannot uncalc (haig/mcclellan); TODO */
            }

            assert(g[v].suffix.graph->kind == NFA_SUFFIX);

            // Ref count all suffixes, as we don't want to merge a suffix
            // that happens to be shared with a non-leaf vertex somewhere.
            DEBUG_PRINTF("vertex %zu has suffix %p\n", g[v].index,
                         g[v].suffix.graph.get());
            fcount[g[v].suffix.graph.get()]++;

            // Vertex must be a reporting pseudo accept
            if (!isLeafNode(v, g)) {
                continue;
            }

            suffix_vertices.push_back(v);
        }
    }

    for (auto v : suffix_vertices) {
        if (in_degree(v, g) == 0) {
            assert(tbi.isAnyStart(v));
            continue;
        }

        const NGHolder *h = g[v].suffix.graph.get();
        assert(h);
        DEBUG_PRINTF("suffix %p\n", h);

        // We can't easily merge suffixes shared with other vertices, and
        // creating a unique copy to do so may just mean we end up tracking
        // more NFAs. Better to leave shared suffixes alone.
        if (fcount[h] != 1) {
            DEBUG_PRINTF("skipping shared suffix\n");
            continue;
        }

        UncalcLeafKey key(g, v);
        vector<RoseVertex> &vec = clusters[key];
        if (vec.empty()) {

            ordered.push_back(key);
        }
        vec.push_back(v);
    }

    DEBUG_PRINTF("find loop done\n");
}

/**
 * This function attempts to combine identical roles (same literals, same
 * predecessors, etc) with different suffixes into a single role which
 * activates a larger suffix. The leaf vertices of the graph with a suffix are
 * grouped into clusters which have members triggered by identical roles. The
 * \ref mergeNfaCluster function (from ng_uncalc_components) is then utilised
 * to build a set of larger (and still implementable) suffixes. The graph is
 * then updated to point to the new suffixes and any unneeded roles are
 * removed.
 *
 * Note: suffixes which are shared amongst multiple roles are not considered
 * for this pass as the individual suffixes would have to continue to exist for
 * the other roles to trigger resulting in the transformation not producing any
 * savings.
 *
 * Note: as \ref mergeNfaCluster is slow when the cluster sizes are large,
 * clusters of more than \ref MERGE_GROUP_SIZE_MAX roles are split into smaller
 * chunks for processing.
 */
void uncalcLeaves(RoseBuildImpl &tbi) {
    DEBUG_PRINTF("uncalcing\n");

    map<UncalcLeafKey, vector<RoseVertex> > clusters;
    deque<UncalcLeafKey> ordered;
    findUncalcLeavesCandidates(tbi, clusters, ordered);

    vector<RoseVertex> dead;

    for (const auto &key : ordered) {
        DEBUG_PRINTF("cluster of size %zu\n", clusters[key].size());
        mergeCluster(tbi.g, tbi.rm, clusters[key], dead, tbi.cc);
    }
    tbi.removeVertices(dead);
}

/**
 * Returns a loose hash of a leftfix for use in dedupeLeftfixes. Note that
 * reports should not contribute to the hash.
 */
static
size_t hashLeftfix(const LeftEngInfo &left) {
    size_t val = 0;

    if (left.castle) {
        hash_combine(val, left.castle->reach());
        for (const auto &pr : left.castle->repeats) {
            hash_combine(val, pr.first); // top
            hash_combine(val, pr.second.bounds);
        }
    } else if (left.graph) {
        hash_combine(val, hash_holder(*left.graph));
    }

    return val;
}

namespace {

/** Key used to group sets of leftfixes by the dedupeLeftfixes path. */
struct RoseGroup {
    RoseGroup(const RoseBuildImpl &build, RoseVertex v)
        : left_hash(hashLeftfix(build.g[v].left)),
          lag(build.g[v].left.lag), eod_table(build.isInETable(v)) {
        const RoseGraph &g = build.g;
        assert(in_degree(v, g) == 1);
        RoseVertex u = *inv_adjacent_vertices(v, g).first;
        parent = g[u].index;
    }

    bool operator<(const RoseGroup &b) const {
        const RoseGroup &a = *this;
        ORDER_CHECK(parent);
        ORDER_CHECK(left_hash);
        ORDER_CHECK(lag);
        ORDER_CHECK(eod_table);
        return false;
    }

private:
    /** Parent vertex index. We must use the index, rather than the descriptor,
     * for compile determinism. */
    size_t parent;

    /** Quick hash of the leftfix itself. Must be identical for a given pair of
     * graphs if is_equal would return true. */
    size_t left_hash;

    /** Leftfix lag value. */
    u32 lag;

    /** True if associated vertex (successor) is in the EOD table. We don't
     * allow sharing of leftfix engines between "normal" and EOD operation. */
    bool eod_table;
};

/**
 * Trivial Rose comparator intended to find graphs that are identical except
 * for their report IDs. Relies on vertex and edge indices to pick up graphs
 * that have been messily put together in different orderings...
 */
struct RoseComparator {
    explicit RoseComparator(const RoseGraph &g_in) : g(g_in) {}

    bool operator()(const RoseVertex u, const RoseVertex v) const {
        const LeftEngInfo &u_left = g[u].left;
        const LeftEngInfo &v_left = g[v].left;

        if (u_left.castle && v_left.castle) {
            return is_equal(*u_left.castle, u_left.leftfix_report,
                            *v_left.castle, v_left.leftfix_report);
        }

        if (!u_left.graph || !v_left.graph) {
            return false;
        }

        return is_equal(*u_left.graph, u_left.leftfix_report, *v_left.graph,
                        v_left.leftfix_report);
    }

private:
    const RoseGraph &g;
};

} // namespace

/**
 * This pass performs work similar to \ref dedupeSuffixes - it removes
 * duplicate prefix/infixes (that is, leftfixes) which are identical graphs and
 * share the same trigger vertex and lag. Leftfixes are first grouped by
 * parent role and lag to reduce the number of candidates to be inspected
 * for each leftfix. The graphs in each cluster are then compared with each
 * other and the graph is updated to only refer to a canonical version of each
 * graph.
 *
 * Note: only roles with a single predecessor vertex are considered for this
 * transform - it should probably be generalised to work for roles which share
 * the same set of predecessor roles as for \ref dedupeLeftfixesVariableLag or it
 * should be retired entirely.
 */
bool dedupeLeftfixes(RoseBuildImpl &tbi) {
    DEBUG_PRINTF("deduping leftfixes\n");
    map<RoseGroup, deque<RoseVertex>> roses;
    bool work_done = false;

    /* Note: a leftfix's transientness will not be altered by deduping */

    // Collect leftfixes into groups.
    RoseGraph &g = tbi.g;
    for (auto v : vertices_range(g)) {
        if (!g[v].left) {
            continue;
        }
        const left_id left(g[v].left);

        if (left.haig()) {
            /* TODO: allow merging of identical haigs */
            continue;
        }

        if (in_degree(v, g) != 1) {
            continue;
        }

        roses[RoseGroup(tbi, v)].push_back(v);
    }

    DEBUG_PRINTF("collected %zu rose groups\n", roses.size());

    const RoseComparator rosecmp(g);

    // Walk groups and dedupe the roses therein.
    for (deque<RoseVertex> &verts : roses | map_values) {
        DEBUG_PRINTF("group has %zu vertices\n", verts.size());

        ue2::unordered_set<left_id> seen;

        for (auto jt = verts.begin(), jte = verts.end(); jt != jte; ++jt) {
            RoseVertex v = *jt;
            left_id left(g[v].left);

            // Skip cases we've already handled, and mark as seen otherwise.
            if (!seen.insert(left).second) {
                continue;
            }

            // Scan the rest of the list for dupes.
            for (auto kt = std::next(jt); kt != jte; ++kt) {
                if (g[v].left == g[*kt].left || !rosecmp(v, *kt)) {
                    continue;
                }

                // Dupe found.
                DEBUG_PRINTF("rose at vertex %zu is a dupe of %zu\n",
                             g[*kt].index, g[v].index);
                assert(g[v].left.lag == g[*kt].left.lag);
                g[*kt].left = g[v].left;
                work_done = true;
            }
        }
    }

    return work_done;
}

/**
 * \brief Returns a numeric key that can be used to group this suffix with
 * others that may be its duplicate.
 */
static
size_t suffix_size_key(const suffix_id &s) {
    if (s.graph()) {
        return num_vertices(*s.graph());
    }
    if (s.castle()) {
        return s.castle()->repeats.size();
    }
    return 0;
}

static
bool is_equal(const suffix_id &s1, const suffix_id &s2) {
    if (s1.graph() && s2.graph()) {
        return is_equal(*s1.graph(), *s2.graph());
    } else if (s1.castle() && s2.castle()) {
        return is_equal(*s1.castle(), *s2.castle());
    }
    return false;
}

/**
 * This function simply looks for suffix NGHolder graphs which are identical
 * and updates the roles in the RoseGraph to refer to only a single copy. This
 * obviously has benefits in terms of both performance (as we don't run
 * multiple engines doing the same work) and stream state. This function first
 * groups all suffixes by number of vertices and report set to restrict the set
 * of possible candidates. Each group is then walked to find duplicates using
 * the \ref is_equal comparator for NGHolders and updating the RoseGraph as it
 * goes.
 *
 * Note: does not dedupe suffixes of vertices in the EOD table.
 */
void dedupeSuffixes(RoseBuildImpl &tbi) {
    DEBUG_PRINTF("deduping suffixes\n");

    ue2::unordered_map<suffix_id, set<RoseVertex>> suffix_map;
    map<pair<size_t, set<ReportID>>, vector<suffix_id>> part;

    // Collect suffixes into groups.
    RoseGraph &g = tbi.g;
    for (auto v : vertices_range(g)) {
        if (!g[v].suffix || tbi.isInETable(v)) {
            continue;
        }

        const suffix_id s(g[v].suffix);

        if (!(s.graph() || s.castle())) {
            continue; // e.g. Haig
        }

        set<RoseVertex> &verts = suffix_map[s];
        if (verts.empty()) {
            part[make_pair(suffix_size_key(s), all_reports(s))].push_back(s);
        }
        verts.insert(v);
    }

    DEBUG_PRINTF("collected %zu groups\n", part.size());

    for (const auto &cand : part | map_values) {
        if (cand.size() <= 1) {
            continue;
        }
        DEBUG_PRINTF("deduping cand set of size %zu\n", cand.size());

        for (auto jt = cand.begin(); jt != cand.end(); ++jt) {
            if (suffix_map[*jt].empty()) {
                continue;
            }
            for (auto kt = next(jt); kt != cand.end(); ++kt) {
                if (suffix_map[*kt].empty() || !is_equal(*jt, *kt)) {
                    continue;
                }
                DEBUG_PRINTF("found dupe\n");
                for (auto v : suffix_map[*kt]) {
                    RoseVertex dupe = *suffix_map[*jt].begin();
                    assert(dupe != v);
                    g[v].suffix.graph = g[dupe].suffix.graph;
                    g[v].suffix.castle = g[dupe].suffix.castle;
                    assert(suffix_id(g[v].suffix) ==
                           suffix_id(g[dupe].suffix));
                    suffix_map[*jt].insert(v);
                }
                suffix_map[*kt].clear();
            }
        }
    }
}

namespace {

/**
 * This class stores a mapping from an engine reference (left_id, suffix_id,
 * etc) to a list of vertices, and also allows us to iterate over the set of
 * engine references in insertion order -- we add to the mapping in vertex
 * iteration order, so this allows us to provide a consistent ordering.
 */
template<class EngineRef>
class Bouquet {
private:
    list<EngineRef> ordering; // Unique list in insert order.
    typedef ue2::unordered_map<EngineRef, deque<RoseVertex> > BouquetMap;
    BouquetMap bouquet;
public:
    void insert(const EngineRef &h, RoseVertex v) {
        typename BouquetMap::iterator f = bouquet.find(h);
        if (f == bouquet.end()) {
            ordering.push_back(h);
            bouquet[h].push_back(v);
        } else {
            f->second.push_back(v);
        }
    }

    void insert(const EngineRef &h, const deque<RoseVertex> &verts) {
        typename BouquetMap::iterator f = bouquet.find(h);
        if (f == bouquet.end()) {
            ordering.push_back(h);
            bouquet.insert(make_pair(h, verts));
        } else {
            f->second.insert(f->second.end(), verts.begin(), verts.end());
        }
    }

    const deque<RoseVertex> &vertices(const EngineRef &h) const {
        typename BouquetMap::const_iterator it = bouquet.find(h);
        assert(it != bouquet.end()); // must be present
        return it->second;
    }

    void erase(const EngineRef &h) {
        assert(bouquet.find(h) != bouquet.end());
        bouquet.erase(h);
        ordering.remove(h);
    }

    /** Remove all the elements in the given iterator range. */
    template <class Iter>
    void erase_all(Iter erase_begin, Iter erase_end) {
        for (Iter it = erase_begin; it != erase_end; ++it) {
            bouquet.erase(*it);
        }

        // Use a quick-lookup container so that we only have to traverse the
        // 'ordering' list once.
        const set<EngineRef> dead(erase_begin, erase_end);
        for (iterator it = begin(); it != end(); /* incremented inside */) {
            if (contains(dead, *it)) {
                ordering.erase(it++);
            } else {
                ++it;
            }
        }
    }

    void clear() {
        ordering.clear();
        bouquet.clear();
    }

    size_t size() const { return bouquet.size(); }

    // iterate over holders in insert order
    typedef typename list<EngineRef>::iterator iterator;
    iterator begin() { return ordering.begin(); }
    iterator end() { return ordering.end(); }

    // const iterate over holders in insert order
    typedef typename list<EngineRef>::const_iterator const_iterator;
    const_iterator begin() const { return ordering.begin(); }
    const_iterator end() const { return ordering.end(); }
};

typedef Bouquet<left_id> RoseBouquet;
typedef Bouquet<suffix_id> SuffixBouquet;

} // namespace

/**
 * Split a \ref Bouquet of some type into several smaller ones.
 */
template <class EngineRef>
static void chunkBouquets(const Bouquet<EngineRef> &in,
                          deque<Bouquet<EngineRef>> &out,
                          const size_t chunk_size) {
    if (in.size() <= chunk_size) {
        out.push_back(in);
        return;
    }

    out.push_back(Bouquet<EngineRef>());
    for (const auto &engine : in) {
        if (out.back().size() >= chunk_size) {
            out.push_back(Bouquet<EngineRef>());
        }
        out.back().insert(engine, in.vertices(engine));
    }
}

static
bool stringsCanFinishAtSameSpot(const ue2_literal &u,
                                ue2_literal::const_iterator v_b,
                                ue2_literal::const_iterator v_e) {
    ue2_literal::const_iterator u_e = u.end();
    ue2_literal::const_iterator u_b = u.begin();

    while (u_e != u_b && v_e != v_b) {
        --u_e;
        --v_e;

        if (!overlaps(*u_e, *v_e)) {
            return false;
        }
    }

    return true;
}

/**
 * Check that if after u has been seen, that it is impossible for the arrival of
 * v to require the inspection of an engine earlier than u did.
 *
 * Let delta be the earliest that v can be seen after u (may be zero)
 *
 * ie, we require u_loc - ulag <= v_loc - vlag (v_loc = u_loc + delta)
 * ==> - ulag <= delta - vlag
 * ==> vlag - ulag <= delta
 */
static
bool checkPrefix(const rose_literal_id &ul, const u32 ulag,
                 const rose_literal_id &vl, const u32 vlag) {
    DEBUG_PRINTF("'%s'-%u '%s'-%u\n", escapeString(ul.s).c_str(), ulag,
                 escapeString(vl.s).c_str(), vlag);

    if (vl.delay || ul.delay) {
        /* engine related literals should not be delayed anyway */
        return false;
    }

    if (ulag >= vlag) {
        assert(maxOverlap(ul, vl) <= vl.elength() - vlag + ulag);
        return true;
    }

    size_t min_allowed_delta = vlag - ulag;
    DEBUG_PRINTF("min allow distace %zu\n", min_allowed_delta);

    for (size_t i = 0; i < min_allowed_delta; i++) {
        if (stringsCanFinishAtSameSpot(ul.s, vl.s.begin(), vl.s.end() - i)) {
            DEBUG_PRINTF("v can follow u at a (too close) distance of %zu\n", i);
            return false;
        }
    }

    DEBUG_PRINTF("OK\n");
    return true;
}

static
bool hasSameEngineType(const RoseVertexProps &u_prop,
                       const RoseVertexProps &v_prop) {
    const left_id u_left(u_prop.left), v_left(v_prop.left);

    if (u_left.haig() || v_left.haig()) {
        if (u_left.graph() != v_left.graph()) {
            return false;
        }
    }

    if (u_left.dfa() || v_left.dfa()) {
        if (u_left.graph() != v_left.graph()) {
            return false;
        }
    }

    if (u_left.castle() || v_left.castle()) {
        if (!u_left.castle() || !v_left.castle()) {
            return false; // Must both be castles.
        }
    }

    return true;
}

static
bool compatibleLiteralsForMerge(
                     const vector<pair<const rose_literal_id *, u32>> &ulits,
                     const vector<pair<const rose_literal_id *, u32>> &vlits) {
    assert(!ulits.empty());
    assert(!vlits.empty());

    // We cannot merge engines that prefix literals in different tables.
    if (ulits[0].first->table != vlits[0].first->table) {
        DEBUG_PRINTF("literals in different tables\n");
        return false;
    }

    /* An engine requires that all accesses to it are ordered by offsets. (ie,
       we can not check an engine's state at offset Y, if we have already
       checked its status at offset X and X > Y). If we can not establish that
       the literals used for triggering will statisfy this property, then it is
       not safe to merge the engine. */
    for (const auto &ue : ulits) {
        const rose_literal_id &ul = *ue.first;
        u32 ulag = ue.second;

        if (ul.delay) {
            return false; // We don't handle delayed cases yet.
        }

        for (const auto &ve : vlits) {
            const rose_literal_id &vl = *ve.first;
            u32 vlag = ve.second;

            if (vl.delay) {
                return false; // We don't handle delayed cases yet.
            }

            if (!checkPrefix(ul, ulag, vl, vlag)
                || !checkPrefix(vl, vlag, ul, ulag)) {
                DEBUG_PRINTF("prefix check failed\n");
                return false;
            }
        }
    }

    return true;
}

/**
 * True if this graph has few enough accel states to be implemented as an NFA
 * with all of those states actually becoming accel schemes.
 */
static
bool isAccelerableLeftfix(const RoseBuildImpl &build, const NGHolder &g) {
    u32 num = countAccelStates(g, &build.rm, build.cc);
    DEBUG_PRINTF("graph with %zu vertices has %u accel states\n",
                  num_vertices(g), num);
    return num <= NFA_MAX_ACCEL_STATES;
}

/**
 * In block mode, we want to be a little more selective, We will only merge
 * prefix engines when the literal sets are the same, or if the merged graph
 * has only grown by a small amount.
 */
static
bool safeBlockModeMerge(const RoseBuildImpl &build, RoseVertex u,
                        RoseVertex v) {
    assert(!build.cc.streaming);
    assert(build.isRootSuccessor(u) == build.isRootSuccessor(v));

    // Always merge infixes if we can (subject to the other criteria in
    // mergeableRoseVertices).
    if (!build.isRootSuccessor(u)) {
        return true;
    }

    const RoseGraph &g = build.g;

    // Merge prefixes with identical literal sets (as we'd have to run them
    // both when we see those literals anyway).
    if (g[u].literals == g[v].literals) {
        return true;
    }

    // The rest of this function only deals with the case when both vertices
    // have graph leftfixes.
    if (!g[u].left.graph || !g[v].left.graph) {
        return false;
    }

    const size_t u_count = num_vertices(*g[u].left.graph);
    const size_t v_count = num_vertices(*g[v].left.graph);
    DEBUG_PRINTF("u prefix has %zu vertices, v prefix has %zu vertices\n",
                 u_count, v_count);
    if (u_count > MAX_BLOCK_PREFIX_MERGE_VERTICES ||
        v_count > MAX_BLOCK_PREFIX_MERGE_VERTICES) {
        DEBUG_PRINTF("prefixes too big already\n");
        return false;
    }

    DEBUG_PRINTF("trying merge\n");
    NGHolder h;
    cloneHolder(h, *g[v].left.graph);
    if (!mergeNfaPair(*g[u].left.graph, h, nullptr, build.cc)) {
        DEBUG_PRINTF("couldn't merge\n");
        return false;
    }

    const size_t merged_count = num_vertices(h);
    DEBUG_PRINTF("merged result has %zu vertices\n", merged_count);
    if (merged_count > MAX_BLOCK_PREFIX_MERGE_VERTICES) {
        DEBUG_PRINTF("exceeded limit\n");
        return false;
    }

    // We want to only perform merges that take advantage of some
    // commonality in the two input graphs, so we check that the number of
    // vertices has only grown a small amount: somewhere between the sum
    // (no commonality) and the max (no growth at all) of the vertex counts
    // of the input graphs.
    const size_t max_size = u_count + v_count;
    const size_t min_size = max(u_count, v_count);
    const size_t max_growth = ((max_size - min_size) * 25) / 100;
    if (merged_count > min_size + max_growth) {
        DEBUG_PRINTF("grew too much\n");
        return false;
    }

    // We don't want to squander any chances at accelerating.
    if (!isAccelerableLeftfix(build, h) &&
        (isAccelerableLeftfix(build, *g[u].left.graph) ||
         isAccelerableLeftfix(build, *g[v].left.graph))) {
        DEBUG_PRINTF("would lose accel property\n");
        return false;
    }

    DEBUG_PRINTF("safe to merge\n");
    return true;
}

bool mergeableRoseVertices(const RoseBuildImpl &tbi, RoseVertex u,
                           RoseVertex v) {
    assert(u != v);

    if (!hasSameEngineType(tbi.g[u], tbi.g[v])) {
        return false;
    }

    if (!tbi.cc.streaming && !safeBlockModeMerge(tbi, u, v)) {
        return false;
    }

    /* We cannot merge prefixes/vertices if they are successors of different
     * root vertices */
    if (tbi.isRootSuccessor(u)) {
        assert(tbi.isRootSuccessor(v));
        set<RoseVertex> u_preds;
        set<RoseVertex> v_preds;
        insert(&u_preds, inv_adjacent_vertices(u, tbi.g));
        insert(&v_preds, inv_adjacent_vertices(v, tbi.g));

        if (u_preds != v_preds) {
            return false;
        }
    }

    u32 ulag = tbi.g[u].left.lag;
    vector<pair<const rose_literal_id *, u32>> ulits;
    ulits.reserve(tbi.g[u].literals.size());
    for (u32 id : tbi.g[u].literals) {
        ulits.emplace_back(&tbi.literals.at(id), ulag);
    }

    u32 vlag = tbi.g[v].left.lag;
    vector<pair<const rose_literal_id *, u32>> vlits;
    vlits.reserve(tbi.g[v].literals.size());
    for (u32 id : tbi.g[v].literals) {
        vlits.emplace_back(&tbi.literals.at(id), vlag);
    }

    if (!compatibleLiteralsForMerge(ulits, vlits)) {
        return false;
    }

    DEBUG_PRINTF("roses on %zu and %zu are mergeable\n", tbi.g[u].index,
                 tbi.g[v].index);
    return true;
}

/* We cannot merge an engine, if a trigger literal and a post literal overlap
 * in such a way that engine status needs to be check at a point before the
 * engine's current location.
 *
 * i.e., for a trigger literal u and a pos literal v,
 * where delta is the earliest v can appear after t,
 * we require that v_loc - v_lag >= u_loc
 * ==> u_loc + delta - v_lag >= u_loc
 * ==> delta >= v_lag
 *
 */
static
bool checkPredDelay(const rose_literal_id &ul, const rose_literal_id &vl,
                    u32 vlag) {
    DEBUG_PRINTF("%s %s (lag %u)\n", escapeString(ul.s).c_str(),
                 escapeString(vl.s).c_str(), vlag);

    for (size_t i = 0; i < vlag; i++) {
        if (stringsCanFinishAtSameSpot(ul.s, vl.s.begin(), vl.s.end() - i)) {
            DEBUG_PRINTF("v can follow u at a (too close) distance of %zu\n", i);
            return false;
        }
    }

    DEBUG_PRINTF("OK\n");
    return true;
}

static never_inline
bool checkPredDelays(const RoseBuildImpl &tbi, const deque<RoseVertex> &v1,
                     const deque<RoseVertex> &v2) {
    flat_set<RoseVertex> preds;
    for (auto v : v1) {
        insert(&preds, inv_adjacent_vertices(v, tbi.g));
    }

    flat_set<u32> pred_lits;

    /* No need to examine delays of a common pred - as it must already have
     * survived the delay checks.
     *
     * This is important when the pred is in the anchored table as
     * the literal is no longer available. */
    flat_set<RoseVertex> known_good_preds;
    for (auto v : v2) {
        insert(&known_good_preds, inv_adjacent_vertices(v, tbi.g));
    }

    for (auto u : preds) {
        if (!contains(known_good_preds, u)) {
            insert(&pred_lits, tbi.g[u].literals);
        }
    }

    vector<const rose_literal_id *> pred_rose_lits;
    pred_rose_lits.reserve(pred_lits.size());
    for (const auto &p : pred_lits) {
        pred_rose_lits.push_back(&tbi.literals.at(p));
    }

    for (auto v : v2) {
        u32 vlag = tbi.g[v].left.lag;
        if (!vlag) {
            continue;
        }

        for (const u32 vlit : tbi.g[v].literals) {
            const rose_literal_id &vl = tbi.literals.at(vlit);
            assert(!vl.delay); // this should never have got this far?
            for (const auto &ul : pred_rose_lits) {
                assert(!ul->delay); // this should never have got this far?

                if (!checkPredDelay(*ul, vl, vlag)) {
                    return false;
                }
            }
        }
    }

    return true;
}

static
bool mergeableRoseVertices(const RoseBuildImpl &tbi,
                           const deque<RoseVertex> &verts1,
                           const deque<RoseVertex> &verts2) {
    assert(!verts1.empty());
    assert(!verts2.empty());

    RoseVertex u_front = verts1.front();
    RoseVertex v_front = verts2.front();

    /* all vertices must have the same engine type: assume all verts in each
     * group are already of the same type */
    if (!hasSameEngineType(tbi.g[u_front], tbi.g[v_front])) {
        return false;
    }

    bool is_prefix = tbi.isRootSuccessor(u_front);

    /* We cannot merge prefixes/vertices if they are successors of different
     * root vertices: similarly, assume the grouped vertices are compatible */
    if (is_prefix) {
        assert(tbi.isRootSuccessor(v_front));
        set<RoseVertex> u_preds;
        set<RoseVertex> v_preds;
        insert(&u_preds, inv_adjacent_vertices(u_front, tbi.g));
        insert(&v_preds, inv_adjacent_vertices(v_front, tbi.g));

        if (u_preds != v_preds) {
            return false;
        }
    }

    vector<pair<const rose_literal_id *, u32>> ulits; /* lit + lag pairs */
    for (auto a : verts1) {
        if (!tbi.cc.streaming && !safeBlockModeMerge(tbi, u_front, a)) {
            return false;
        }

        u32 ulag = tbi.g[a].left.lag;
        for (u32 id : tbi.g[a].literals) {
            ulits.emplace_back(&tbi.literals.at(id), ulag);
        }
    }

    vector<pair<const rose_literal_id *, u32>> vlits;
    for (auto a : verts2) {
        if (!tbi.cc.streaming && !safeBlockModeMerge(tbi, u_front, a)) {
            return false;
        }

        u32 vlag = tbi.g[a].left.lag;
        for (u32 id : tbi.g[a].literals) {
            vlits.emplace_back(&tbi.literals.at(id), vlag);
        }
    }

    if (!compatibleLiteralsForMerge(ulits, vlits)) {
        return false;
    }

    // Check preds are compatible as well.
    if (!checkPredDelays(tbi, verts1, verts2)
        || !checkPredDelays(tbi, verts2, verts1)) {
        return false;
    }

    DEBUG_PRINTF("vertex sets are mergeable\n");
    return true;
}

bool mergeableRoseVertices(const RoseBuildImpl &tbi, const set<RoseVertex> &v1,
                           const set<RoseVertex> &v2) {
    const deque<RoseVertex> vv1(v1.begin(), v1.end());
    const deque<RoseVertex> vv2(v2.begin(), v2.end());
    return mergeableRoseVertices(tbi, vv1, vv2);
}

/** \brief Priority queue element for Rose merges. */
namespace {
struct RoseMergeCandidate {
    RoseMergeCandidate(const left_id &r1_in, const left_id &r2_in, u32 cpl_in,
                       u32 tb)
        : r1(r1_in), r2(r2_in), stopxor(0), cpl(cpl_in), states(0),
          tie_breaker(tb) {
        if (r1.graph() && r2.graph()) {
            const NGHolder &h1 = *r1.graph(), &h2 = *r2.graph();
            /* som_none as haigs don't merge and just a guiding heuristic */
            CharReach stop1 = findStopAlphabet(h1, SOM_NONE);
            CharReach stop2 = findStopAlphabet(h2, SOM_NONE);
            stopxor = (stop1 ^ stop2).count();

            // We use the number of vertices as an approximation of the state
            // count here, as this is just feeding a comparison.
            u32 vertex_count = num_vertices(h1) + num_vertices(h2);
            states = vertex_count - min(vertex_count, cpl);
        } else if (r1.castle() && r2.castle()) {
            // FIXME
        }
    }

    bool operator<(const RoseMergeCandidate &a) const {
        if (stopxor != a.stopxor) {
            return stopxor > a.stopxor;
        }
        if (cpl != a.cpl) {
            return cpl < a.cpl;
        }
        if (states != a.states) {
            return states > a.states;
        }
        return tie_breaker < a.tie_breaker;
    }

    left_id r1;
    left_id r2;
    u32 stopxor;
    u32 cpl; //!< common prefix length
    u32 states;
    u32 tie_breaker; //!< determinism
};
}

static
bool mergeRosePair(RoseBuildImpl &tbi, left_id &r1, left_id &r2,
                   const deque<RoseVertex> &verts1,
                   const deque<RoseVertex> &verts2) {
    assert(!verts1.empty() && !verts2.empty());

    RoseGraph &g = tbi.g;

    if (r1.graph()) {
        assert(r2.graph());
        assert(r1.graph()->kind == r2.graph()->kind);
        if (!mergeNfaPair(*r1.graph(), *r2.graph(), nullptr, tbi.cc)) {
            DEBUG_PRINTF("nfa merge failed\n");
            return false;
        }

        // The graph in r1 has been merged into the graph in r2. Update r1's
        // vertices with the new graph ptr. Since the parent vertices are the
        // same, we know that tops will already have been distinct.
        shared_ptr<NGHolder> &h = g[verts2.front()].left.graph;
        for (RoseVertex v : verts1) {
            g[v].left.graph = h;
        }

        return true;
    } else if (r1.castle()) {
        assert(r2.castle());
        assert(tbi.cc.grey.allowCastle);

        map<u32, u32> top_map;
        if (!mergeCastle(*r2.castle(), *r1.castle(), top_map)) {
            DEBUG_PRINTF("castle merge failed\n");
            return false;
        }

        // The castle in r1 has been merged into the castle in r2, with tops
        // remapped as per top_map.
        const shared_ptr<CastleProto> &c = g[verts2.front()].left.castle;
        for (RoseVertex v : verts1) {
            g[v].left.castle = c;
            for (const auto &e : in_edges_range(v, g)) {
                g[e].rose_top = top_map.at(g[e].rose_top);
            }
        }
        return true;
    }

    assert(0);
    return false;
}

static
void processMergeQueue(RoseBuildImpl &tbi, RoseBouquet &roses,
                       priority_queue<RoseMergeCandidate> &pq) {
    ue2::unordered_set<left_id> dead;

    DEBUG_PRINTF("merge queue has %zu entries\n", pq.size());

    while (!pq.empty()) {
        DEBUG_PRINTF("pq pop h1=%p, h2=%p, cpl=%u, states=%u\n",
                     pq.top().r1.graph(), pq.top().r2.graph(), pq.top().cpl,
                     pq.top().states);

        left_id r1 = pq.top().r1, r2 = pq.top().r2;
        pq.pop();

        if (contains(dead, r1) || contains(dead, r2)) {
            continue;
        }

        if (r1.graph() && r2.graph()) {
            NGHolder *h1 = r1.graph(), *h2 = r2.graph();
            CharReach stop1 = findStopAlphabet(*h1, SOM_NONE);
            CharReach stop2 = findStopAlphabet(*h2, SOM_NONE);
            CharReach stopboth(stop1 & stop2);
            DEBUG_PRINTF("stop1=%zu, stop2=%zu, stopboth=%zu\n", stop1.count(),
                         stop2.count(), stopboth.count());
            if (stopboth.count() < 10 &&
                (stop1.count() > 10 || stop2.count() > 10)) {
                DEBUG_PRINTF("skip merge, would kill stop alphabet\n");
                continue;
            }
            size_t maxstop = max(stop1.count(), stop2.count());
            if (maxstop > 200 && stopboth.count() < 200) {
                DEBUG_PRINTF("skip merge, would reduce stop alphabet\n");
                continue;
            }
        }

        const deque<RoseVertex> &verts1 = roses.vertices(r1);
        const deque<RoseVertex> &verts2 = roses.vertices(r2);

        if (!mergeableRoseVertices(tbi, verts1, verts2)) {
            continue;
        }

        if (!mergeRosePair(tbi, r1, r2, verts1, verts2)) {
            continue;
        }

        roses.insert(r2, verts1);
        roses.erase(r1);
        dead.insert(r1);
    }
}

static
bool nfaHasNarrowStart(const NGHolder &g) {
    if (out_degree(g.startDs, g) > 1) {
        return false; // unanchored
    }

    CharReach cr;

    for (auto v : adjacent_vertices_range(g.start, g)) {
        if (v == g.startDs) {
            continue;
        }
        cr |= g[v].char_reach;
    }
    return cr.count() <= NARROW_START_MAX;
}

static
bool nfaHasFiniteMaxWidth(const NGHolder &g) {
    return findMaxWidth(g).is_finite();
}

namespace {
struct RoseMergeKey {
    RoseMergeKey(const set<RoseVertex> &parents_in,
                 bool narrowStart_in, bool hasMaxWidth_in) :
                        narrowStart(narrowStart_in),
                        hasMaxWidth(hasMaxWidth_in),
                        parents(parents_in) {}
    bool operator<(const RoseMergeKey &b) const {
        const RoseMergeKey &a = *this;
        ORDER_CHECK(narrowStart);
        ORDER_CHECK(hasMaxWidth);
        ORDER_CHECK(parents);
        return false;
    }

    // NOTE: these two bool discriminators are only used for prefixes, not
    // infixes.
    bool narrowStart;
    bool hasMaxWidth;

    set<RoseVertex> parents;
};
}

static
bool hasReformedStartDotStar(const NGHolder &h, const Grey &grey) {
    if (!proper_out_degree(h.startDs, h)) {
        return false;
    }

    assert(!is_triggered(h));

    NGHolder h_temp;
    cloneHolder(h_temp, h);

    vector<BoundedRepeatData> repeats;
    bool suitable_for_sds_reforming = false;
    const map<u32, u32> fixed_depth_tops; /* not relevant for cfa check */
    const map<u32, vector<vector<CharReach>>> triggers; /* not for cfa check */
    const bool simple_model_selection = true; // FIRST is considered simple
    analyseRepeats(h_temp, nullptr, fixed_depth_tops, triggers, &repeats, true,
                   simple_model_selection, grey, &suitable_for_sds_reforming);

    return suitable_for_sds_reforming;
}

static
u32 commonPrefixLength(left_id &r1, left_id &r2) {
    if (r1.graph() && r2.graph()) {
        return commonPrefixLength(*r1.graph(), *r2.graph());
    } else if (r1.castle() && r2.castle()) {
        return min(findMinWidth(*r1.castle()), findMinWidth(*r2.castle()));
    }
    return 0;
}

/**
 * This pass attempts to merge prefix/infix engines which share a common set of
 * parent vertices.
 *
 * Engines are greedily merged pairwise by this process based on a priority
 * queue keyed off the common prefix length.
 *
 * Engines are not merged if the lags are not compatible or if it would damage
 * the stop alphabet.
 *
 * Infixes:
 * - LBR candidates are not considered.
 *
 * Prefixes:
 * - transient prefixes are not considered.
 * - with a max width or a narrow start are kept segregated by
 *   this phase and can only be merged with similar infixes.
 * - in block mode, merges are only performed if literal sets are the same.
 * - merges are not considered in cases where dot star start state will be
 *   reformed to optimise a leading repeat.
 */
void mergeLeftfixesVariableLag(RoseBuildImpl &tbi) {
    if (!tbi.cc.grey.mergeRose) {
        return;
    }

    map<RoseMergeKey, RoseBouquet> rosesByParent;
    RoseGraph &g = tbi.g;
    set<RoseVertex> parents;

    DEBUG_PRINTF("-----\n");
    DEBUG_PRINTF("entry\n");
    DEBUG_PRINTF("-----\n");

    for (auto v : vertices_range(g)) {
        if (!g[v].left) {
            continue;
        }

        const bool is_prefix = tbi.isRootSuccessor(v);

        // Only non-transient for the moment.
        if (contains(tbi.transient, g[v].left)) {
            continue;
        }

        // No forced McClellan or Haig infix merges.
        if (g[v].left.dfa || (!is_prefix && g[v].left.haig)) {
            continue;
        }

        if (g[v].left.graph) {
            NGHolder &h = *g[v].left.graph;

            /* Ensure that kind on the graph is correct */
            assert(h.kind == (is_prefix ? NFA_PREFIX : NFA_INFIX));

            if (hasReformedStartDotStar(h, tbi.cc.grey)) {
                continue; // preserve the optimisation of the leading repeat
            }

            if (!is_prefix && isLargeLBR(h, tbi.cc.grey)) {
                continue;
            }
        }

        if (g[v].left.castle && !tbi.cc.grey.allowCastle) {
            DEBUG_PRINTF("castle merging disallowed by greybox\n");
            continue;
        }

        // We collapse the anchored root into the root vertex when calculating
        // parents, so that we can merge differently-anchored prefix roses
        // together. (Prompted by UE-2100)
        parents.clear();
        for (auto u : inv_adjacent_vertices_range(v, g)) {
            if (tbi.isAnyStart(u)) {
                parents.insert(tbi.root);
            } else {
                parents.insert(u);
            }
        }

        if (parents.empty()) {
            assert(0);
            continue;
        }

        // We want to distinguish prefixes (but not infixes) on whether they
        // have a narrow start or max width.
        bool narrowStart = false, hasMaxWidth = false;
        if (is_prefix && g[v].left.graph) {
            const NGHolder &h = *g[v].left.graph;
            narrowStart = nfaHasNarrowStart(h);
            hasMaxWidth = nfaHasFiniteMaxWidth(h);
        }

        RoseMergeKey key(parents, narrowStart, hasMaxWidth);
        rosesByParent[key].insert(g[v].left, v);
    }

    for (auto &m : rosesByParent) {
        if (m.second.size() < 2) {
            continue;
        }

        deque<RoseBouquet> rose_groups;
        chunkBouquets(m.second, rose_groups, MERGE_GROUP_SIZE_MAX);
        m.second.clear();
        DEBUG_PRINTF("chunked roses into %zu groups\n", rose_groups.size());

        for (auto &roses : rose_groups) {
            // All pairs on the prio queue.
            u32 tie_breaker = 0;
            priority_queue<RoseMergeCandidate> pq;
            for (auto it = roses.begin(), ite = roses.end(); it != ite; ++it) {
                left_id r1 = *it;
                const deque<RoseVertex> &verts1 = roses.vertices(r1);

                for (auto jt = next(it); jt != ite; ++jt) {
                    left_id r2 = *jt;

                    // Roses must be of the same engine type to be mergeable.
                    if ((!r1.graph() != !r2.graph()) ||
                        (!r1.castle() != !r2.castle())) {
                        continue;
                    }

                    // Castles must have the same reach to be mergeable.
                    if (r1.castle()) {
                        if (r1.castle()->reach() != r2.castle()->reach()) {
                            continue;
                        }
                    }

                    const deque<RoseVertex> &verts2 = roses.vertices(r2);
                    if (!mergeableRoseVertices(tbi, verts1, verts2)) {
                        continue; // No point queueing unmergeable cases.
                    }

                    u32 cpl = commonPrefixLength(r1, r2);
                    pq.push(RoseMergeCandidate(r1, r2, cpl, tie_breaker++));
                }
            }
            processMergeQueue(tbi, roses, pq);
        }
    }

    DEBUG_PRINTF("-----\n");
    DEBUG_PRINTF("exit\n");
    DEBUG_PRINTF("-----\n");
}

namespace {

/**
 * Key used to group sets of leftfixes for the dedupeLeftfixesVariableLag path.
 */
struct DedupeLeftKey {
    DedupeLeftKey(const RoseBuildImpl &build, RoseVertex v)
        : left_hash(hashLeftfix(build.g[v].left)) {
        const auto &g = build.g;
        for (const auto &e : in_edges_range(v, g)) {
            preds.emplace(g[source(e, g)].index, g[e].rose_top);
        }
    }

    bool operator<(const DedupeLeftKey &b) const {
        return tie(left_hash, preds) < tie(b.left_hash, b.preds);
    }

private:
    /** Quick hash of the leftfix itself. Must be identical for a given pair of
     * graphs if is_equal would return true. */
    size_t left_hash;

    /** For each in-edge, the pair of (parent index, edge top). */
    set<pair<size_t, u32>> preds;
};

} // namespace

/**
 * This is a generalisation of \ref dedupeLeftfixes which relaxes two
 * restrictions: multiple predecessor roles are allowed and the delay used by
 * each vertex may not be the same for each vertex. Like \ref dedupeLeftfixes,
 * the leftfixes' successor vertices are first grouped to reduce the number of
 * potential candidates - the grouping in this case is by the set of
 * predecessor roles with their associated top events. For the dedupe to be
 * possible, it is required that:
 *
 * 1. the nfa graphs with respect to the relevant reports are identical
 * 2. the nfa graphs are triggered by the same roles with same events (ensured
 *    by the initial grouping pass)
 * 3. all the successor roles of either graph can inspect the combined leftfix
 *    without advancing the state of the leftfix past the point that another
 *    successor may want to inspect it; the overlap relationships between the
 *    involved literals are examined to ensure that this property holds.
 *
 * Note: in block mode we restrict the dedupe of prefixes further as some of
 * logic checks are shared with the mergeLeftfix functions.
 */
void dedupeLeftfixesVariableLag(RoseBuildImpl &tbi) {
    map<DedupeLeftKey, RoseBouquet> roseGrouping;

    DEBUG_PRINTF("entry\n");

    RoseGraph &g = tbi.g;
    for (auto v : vertices_range(g)) {
        if (!g[v].left) {
            continue;
        }

        const left_id leftfix(g[v].left);

        // Only non-transient for the moment.
        if (contains(tbi.transient, leftfix)) {
            continue;
        }

        if (leftfix.haig()) {
            /* TODO: allow merging of identical haigs */
            continue;
        }

        roseGrouping[DedupeLeftKey(tbi, v)].insert(leftfix, v);
    }

    for (RoseBouquet &roses : roseGrouping | map_values) {
        DEBUG_PRINTF("group of %zu roses\n", roses.size());

        if (roses.size() < 2) {
            continue;
        }

        const RoseComparator rosecmp(g);

        for (auto it = roses.begin(); it != roses.end(); ++it) {
            left_id r1 = *it;
            const deque<RoseVertex> &verts1 = roses.vertices(r1);

            for (auto jt = next(it); jt != roses.end(); ++jt) {
                left_id r2 = *jt;
                const deque<RoseVertex> &verts2 = roses.vertices(r2);

                if (!rosecmp(verts1.front(), verts2.front())) {
                    continue;
                }

                if (!mergeableRoseVertices(tbi, verts1, verts2)) {
                    continue;
                }

                DEBUG_PRINTF("%p and %p are dupes\n", r1.graph(), r2.graph());

                // Replace h1 with h2.

                const LeftEngInfo &v2_left = g[verts2.front()].left;
                assert(v2_left.graph.get() == r2.graph());

                for (auto v : verts1) {
                    DEBUG_PRINTF("replacing report %u with %u on %zu\n",
                                 g[v].left.leftfix_report,
                                 v2_left.leftfix_report, g[v].index);
                    u32 orig_lag = g[v].left.lag;
                    g[v].left = v2_left;
                    g[v].left.lag = orig_lag;
                }
                roses.insert(r2, verts1);
                // no need to erase h1 from roses, that would invalidate `it'.
                break;
            }
        }
    }
}

static
u32 findUnusedTop(const ue2::flat_set<u32> &tops) {
    u32 i = 0;
    while (contains(tops, i)) {
        i++;
    }
    return i;
}

// Replace top 't' on edges with new top 'u'.
static
void replaceTops(NGHolder &h, const map<u32, u32> &top_mapping) {
    for (const auto &e : out_edges_range(h.start, h)) {
        NFAVertex v = target(e, h);
        if (v == h.startDs) {
            continue;
        }
        flat_set<u32> new_tops;
        for (u32 t : h[e].tops) {
            DEBUG_PRINTF("vertex %zu has top %u\n", h[v].index, t);
            new_tops.insert(top_mapping.at(t));
        }
        h[e].tops = std::move(new_tops);
    }
}

static
bool setDistinctTops(NGHolder &h1, const NGHolder &h2,
                     map<u32, u32> &top_mapping) {
    ue2::flat_set<u32> tops1 = getTops(h1), tops2 = getTops(h2);

    DEBUG_PRINTF("before: h1 has %zu tops, h2 has %zu tops\n", tops1.size(),
                 tops2.size());

    // If our tops don't intersect, we're OK to merge with no changes.
    if (!has_intersection(tops1, tops2)) {
        DEBUG_PRINTF("tops don't intersect\n");
        return true;
    }

    // Otherwise, we have to renumber the tops in h1 so that they don't overlap
    // with the tops in h2.
    top_mapping.clear();
    for (u32 t : tops1) {
        u32 u = findUnusedTop(tops2);
        DEBUG_PRINTF("replacing top %u with %u in h1\n", t, u);
        top_mapping.insert(make_pair(t, u));
        assert(!contains(tops2, u));
        tops2.insert(u);
    }

    replaceTops(h1, top_mapping);
    return true;
}

bool setDistinctRoseTops(RoseGraph &g, NGHolder &h1, const NGHolder &h2,
                         const deque<RoseVertex> &verts1) {
    map<u32, u32> top_mapping;
    if (!setDistinctTops(h1, h2, top_mapping)) {
        return false;
    }

    if (top_mapping.empty()) {
        return true; // No remapping necessary.
    }

    for (auto v : verts1) {
        DEBUG_PRINTF("vertex %zu\n", g[v].index);
        assert(!g[v].left.haig);
        assert(!g[v].left.dfa);
        for (const auto &e : in_edges_range(v, g)) {
            u32 t = g[e].rose_top;
            DEBUG_PRINTF("t=%u\n", t);
            assert(contains(top_mapping, t));
            g[e].rose_top = top_mapping[t];
            DEBUG_PRINTF("edge (%zu,%zu) went from top %u to %u\n",
                         g[source(e, g)].index, g[target(e, g)].index, t,
                         top_mapping[t]);
        }
    }

    return true;
}

static
bool setDistinctSuffixTops(RoseGraph &g, NGHolder &h1, const NGHolder &h2,
                           const deque<RoseVertex> &verts1) {
    map<u32, u32> top_mapping;
    if (!setDistinctTops(h1, h2, top_mapping)) {
        return false;
    }

    if (top_mapping.empty()) {
        return true; // No remapping necessary.
    }

    for (auto v : verts1) {
        DEBUG_PRINTF("vertex %zu\n", g[v].index);
        u32 t = g[v].suffix.top;
        assert(contains(top_mapping, t));
        g[v].suffix.top = top_mapping[t];
    }

    return true;
}

/** \brief Estimate the number of accel states in the given graph when built as
 * an NFA.
 *
 * (The easiest way to estimate something like this is to actually build it:
 * the criteria for NFA acceleration are quite complicated and buried in
 * limex_compile.)
 */
static
u32 estimatedAccelStates(const RoseBuildImpl &tbi, const NGHolder &h) {
    return countAccelStates(h, &tbi.rm, tbi.cc);
}

static
void mergeNfaLeftfixes(RoseBuildImpl &tbi, RoseBouquet &roses) {
    RoseGraph &g = tbi.g;
    DEBUG_PRINTF("%zu nfa rose merge candidates\n", roses.size());

    // We track the number of accelerable states for each graph in a map and
    // only recompute them when the graph is modified.
    ue2::unordered_map<left_id, u32> accel_count;
    for (const auto &rose : roses) {
        assert(rose.graph()->kind == NFA_INFIX);
        accel_count[rose] = estimatedAccelStates(tbi, *rose.graph());
    }

    for (auto it = roses.begin(); it != roses.end(); ++it) {
        left_id r1 = *it;
        const deque<RoseVertex> &verts1 = roses.vertices(r1);

        deque<left_id> merged;
        for (auto jt = next(it); jt != roses.end(); ++jt) {
            left_id r2 = *jt;
            const deque<RoseVertex> &verts2 = roses.vertices(r2);

            DEBUG_PRINTF("consider merging rose %p (%zu verts) "
                         "with %p (%zu verts)\n",
                         r1.graph(), verts1.size(), r2.graph(), verts2.size());

            u32 accel1 = accel_count[r1];
            if (accel1 >= NFA_MAX_ACCEL_STATES) {
                DEBUG_PRINTF("h1 has hit max accel\n");
                break; // next h1
            }

            u32 accel2 = accel_count[r2];
            if (accel1 + accel2 > NFA_MAX_ACCEL_STATES) {
                DEBUG_PRINTF("not merging, might make unaccel (accel1=%u, "
                             "accel2=%u)\n",
                             accel1, accel2);
                continue; // next h2
            }

            if (!mergeableRoseVertices(tbi, verts1, verts2)) {
                DEBUG_PRINTF("not mergeable\n");
                continue; // next h2
            }

            // Attempt to merge h2 into h1.

            NGHolder victim;
            cloneHolder(victim, *r2.graph());

            // Store a copy of the in-edge properties in case we have to roll
            // back.
            map<RoseEdge, RoseEdgeProps> edge_props;
            for (auto v : verts2) {
                for (const auto &e : in_edges_range(v, g)) {
                    edge_props[e] = g[e];
                }
            }

            if (!setDistinctRoseTops(g, victim, *r1.graph(), verts2)) {
                DEBUG_PRINTF("can't set distinct tops\n");
                continue; // next h2
            }

            assert(victim.kind == r1.graph()->kind);
            assert(!generates_callbacks(*r1.graph()));
            if (!mergeNfaPair(victim, *r1.graph(), nullptr, tbi.cc)) {
                DEBUG_PRINTF("merge failed\n");
                // Roll back in-edge properties.
                for (const auto &m : edge_props) {
                    g[m.first] = m.second;
                }
                continue; // next h2
            }

            // Update h2's roses to point to h1 now
            shared_ptr<NGHolder> winner = g[verts1.front()].left.graph;
            for (auto v : verts2) {
                g[v].left.graph = winner;
            }
            roses.insert(r1, verts2);

            merged.push_back(r2);

            if (num_vertices(*winner) >= small_merge_max_vertices(tbi.cc)) {
                DEBUG_PRINTF("h1 now has %zu vertices, proceeding to next\n",
                             num_vertices(*winner));
                break; // next h1
            }

            // Update h1's accel count estimate.
            accel_count[r1] = estimatedAccelStates(tbi, *winner);
        }

        DEBUG_PRINTF("%zu roses merged\n", merged.size());
        roses.erase_all(merged.begin(), merged.end());
    }
}

static
void mergeCastleChunk(RoseBuildImpl &tbi, RoseBouquet &cands) {
    /* caller must have already ensured that candidates have the same reach */
    RoseGraph &g = tbi.g;
    DEBUG_PRINTF("%zu castle rose merge candidates\n", cands.size());

    deque<left_id> merged;

    for (auto it = cands.begin(); it != cands.end(); ++it) {
        left_id r1 = *it;
        CastleProto &castle1 = *r1.castle();
        const deque<RoseVertex> &verts1 = cands.vertices(r1);

        merged.clear();

        for (auto jt = next(it); jt != cands.end(); ++jt) {
            left_id r2 = *jt;
            CastleProto &castle2 = *r2.castle();
            const deque<RoseVertex> &verts2 = cands.vertices(r2);

            if (castle1.repeats.size() == castle1.max_occupancy) {
                DEBUG_PRINTF("castle1 has hit max occupancy\n");
                break; // next castle1
            }

            assert(castle1.reach() == castle2.reach());

            if (!mergeableRoseVertices(tbi, verts1, verts2)) {
                DEBUG_PRINTF("not mergeable\n");
                continue; // next castle2
            }

            DEBUG_PRINTF("castle1=%p (size %zu), castle2=%p (size %zu)\n",
                         &castle1, castle1.repeats.size(), &castle2,
                         castle2.repeats.size());

            map<u32, u32> top_map;
            if (!mergeCastle(castle1, castle2, top_map)) {
                DEBUG_PRINTF("couldn't merge\n");
                continue; // next castle2
            }

            // Update castle2's roses to point to castle1 now.
            shared_ptr<CastleProto> winner = g[verts1.front()].left.castle;
            for (auto v : verts2) {
                g[v].left.castle = winner;
                for (const auto &e : in_edges_range(v, g)) {
                    g[e].rose_top = top_map.at(g[e].rose_top);
                }
            }

            cands.insert(r1, verts2);
            merged.push_back(r2);
        }

        DEBUG_PRINTF("%zu roses merged\n", merged.size());
        cands.erase_all(merged.begin(), merged.end());
    }
}

/**
 * This pass attempts to merge prefix/infix engines with a small number of
 * vertices together into larger engines. The engines must not be have a
 * reformed start dot star (due to a leading repeat) nor an infix LBR. Engines
 * that have compatible lag are greedily grouped such that they remain
 * accelerable and only have a small number of states. Note: if a role has an
 * infix with multiple trigger vertices, the role will be left unchanged by this
 * pass and will remain using an unmerged graph.
 */
void mergeSmallLeftfixes(RoseBuildImpl &tbi) {
    DEBUG_PRINTF("entry\n");

    if (!tbi.cc.grey.mergeRose || !tbi.cc.grey.roseMultiTopRoses) {
        return;
    }

    RoseGraph &g = tbi.g;

    RoseBouquet nfa_roses;

    for (auto v : vertices_range(g)) {
        if (!g[v].left) {
            continue;
        }

        // Handle single-parent infixes only.
        if (tbi.isRootSuccessor(v)) {
            continue;
        }

        left_id left(g[v].left);

        // Only non-transient for the moment.
        if (contains(tbi.transient, left)) {
            continue;
        }

        // No DFAs or Haigs right now.
        if (left.dfa() || left.haig()) {
            continue;
        }

        // Castles are handled by a different pass.
        if (left.castle()) {
            continue;
        }

        assert(left.graph());
        NGHolder &h = *left.graph();

        /* Ensure that kind on the graph is correct */
        assert(h.kind == (tbi.isRootSuccessor(v) ? NFA_PREFIX : NFA_INFIX));

        if (hasReformedStartDotStar(h, tbi.cc.grey)) {
            /* We would lose optimisations of the leading repeat by merging. */
            continue;
        }

        // Don't merge cases that will become LBRs or haigs.
        if (isLargeLBR(h, tbi.cc.grey)) {
            continue;
        }

        // Small roses only.
        if (num_vertices(h) > small_rose_threshold(tbi.cc)) {
            continue;
        }

        nfa_roses.insert(left, v);
    }

    deque<RoseBouquet> rose_groups;
    chunkBouquets(nfa_roses, rose_groups, MERGE_GROUP_SIZE_MAX);
    nfa_roses.clear();
    DEBUG_PRINTF("chunked nfa roses into %zu groups\n", rose_groups.size());

    for (auto &group : rose_groups) {
        mergeNfaLeftfixes(tbi, group);
    }
}

void mergeCastleLeftfixes(RoseBuildImpl &tbi) {
    DEBUG_PRINTF("entry\n");

    if (!tbi.cc.grey.mergeRose || !tbi.cc.grey.roseMultiTopRoses ||
        !tbi.cc.grey.allowCastle) {
        return;
    }

    RoseGraph &g = tbi.g;

    map<CharReach, RoseBouquet> by_reach;

    for (auto v : vertices_range(g)) {
        if (!g[v].left) {
            continue;
        }

        // Handle single-parent infixes only.
        if (tbi.isRootSuccessor(v)) {
            continue;
        }

        const left_id left(g[v].left);

        // Only non-transient for the moment.
        if (contains(tbi.transient, left)) {
            continue;
        }

        if (!left.castle()) {
            continue;
        }

        const CastleProto &castle = *left.castle();
        const CharReach &cr = castle.reach();
        by_reach[cr].insert(left, v);
    }

    for (auto &m : by_reach) {
        DEBUG_PRINTF("%zu castles for reach: %s\n", m.second.size(),
                     describeClass(m.first).c_str());
        RoseBouquet &candidates = m.second;
        deque<RoseBouquet> cand_groups;
        chunkBouquets(candidates, cand_groups, MERGE_CASTLE_GROUP_SIZE_MAX);
        candidates.clear();

        for (auto &group : cand_groups) {
            mergeCastleChunk(tbi, group);
        }
    }
}

static
void mergeSuffixes(RoseBuildImpl &tbi, SuffixBouquet &suffixes,
                   const bool acyclic) {
    RoseGraph &g = tbi.g;

    DEBUG_PRINTF("group has %zu suffixes\n", suffixes.size());

    // If this isn't an acyclic case, we track the number of accelerable states
    // for each graph in a map and only recompute them when the graph is
    // modified.
    ue2::unordered_map<suffix_id, u32> accel_count;
    if (!acyclic) {
        for (const auto &suffix : suffixes) {
            assert(suffix.graph() && suffix.graph()->kind == NFA_SUFFIX);
            accel_count[suffix] = estimatedAccelStates(tbi, *suffix.graph());
        }
    }

    for (auto it = suffixes.begin(); it != suffixes.end(); ++it) {
        suffix_id s1 = *it;
        const deque<RoseVertex> &verts1 = suffixes.vertices(s1);
        assert(s1.graph() && s1.graph()->kind == NFA_SUFFIX);

        // Caller should ensure that we don't propose merges of graphs that are
        // already too big.
        assert(num_vertices(*s1.graph()) < small_merge_max_vertices(tbi.cc));

        deque<suffix_id> merged;
        for (auto jt = next(it); jt != suffixes.end(); ++jt) {
            suffix_id s2 = *jt;
            const deque<RoseVertex> &verts2 = suffixes.vertices(s2);
            assert(s2.graph() && s2.graph()->kind == NFA_SUFFIX);

            if (!acyclic) {
                u32 accel1 = accel_count[s1];
                if (accel1 >= NFA_MAX_ACCEL_STATES) {
                    DEBUG_PRINTF("h1 has hit max accel\n");
                    break; // next h1
                }

                u32 accel2 = accel_count[s2];
                if (accel1 + accel2 > NFA_MAX_ACCEL_STATES) {
                    DEBUG_PRINTF("not merging, might make unaccel (accel1=%u, "
                                 "accel2=%u)\n",
                                 accel1, accel2);
                    continue; // next h2
                }
            }

            // Attempt to merge h2 into h1.

            NGHolder victim;
            cloneHolder(victim, *s2.graph());

            // Store a copy of the suffix tops in case we have to roll back.
            map<RoseVertex, u32> old_tops;
            for (auto v : verts2) {
                old_tops[v] = g[v].suffix.top;
            }

            if (!setDistinctSuffixTops(g, victim, *s1.graph(), verts2)) {
                DEBUG_PRINTF("can't set distinct tops\n");
                continue; // next h2
            }

            if (!mergeNfaPair(victim, *s1.graph(), &tbi.rm, tbi.cc)) {
                DEBUG_PRINTF("merge failed\n");
                // Roll back in-edge properties.
                for (const auto &m : old_tops) {
                    g[m.first].suffix.top = m.second;
                }
                continue; // next h2
            }

            // Update h2's roses to point to h1 now
            shared_ptr<NGHolder> winner = g[verts1.front()].suffix.graph;
            for (auto v : verts2) {
                g[v].suffix.graph = winner;
            }
            suffixes.insert(s1, verts2);
            merged.push_back(s2);

            if (num_vertices(*s1.graph()) >= small_merge_max_vertices(tbi.cc)) {
                DEBUG_PRINTF("h1 now has %zu vertices, proceeding to next\n",
                             num_vertices(*s1.graph()));
                break; // next h1
            }

            if (!acyclic) {
                // Update h1's accel count estimate.
                accel_count[s1] = estimatedAccelStates(tbi, *s1.graph());
            }
        }

        DEBUG_PRINTF("%zu suffixes merged\n", merged.size());
        suffixes.erase_all(merged.begin(), merged.end());
    }
}

/**
 * This merge pass combines suffixes from unrelated roles into a single
 * suffix with multiple top events in order to distinguish the triggers
 * from differing roles. mergeAcyclicSuffixes only considers acyclic suffixes
 * while mergeSmallSuffixes only considers small suffixes. The merges will
 * group roles with suffixes in the graph into clusters of at most
 * \ref MERGE_GROUP_SIZE_MAX. Each cluster is processed by iterating over the
 * suffixes and attempting to pairwise merge it with another member. Merges
 * will fail if the result is not implementable, requires too many distinct top
 * events, or if it losses the ability to be accelerated. The merge will modify
 * the existing suffix graph of the one member (g1), the other member updates
 * it graph to refer to g1 instead of its previous graph (g2) and use the new
 * tops created. Other roles may have been sharing g1 - these are unaffected by
 * the change as the existing top events are left untouched. Other roles using
 * g2 are also unaffected as g2 will continue to exist until while it has any
 * roles triggering it.
 *
 * Note: suffixes destined for the LBR are not considered for these merges as
 * the LBR can only handle a single repeat and this type of repeat is ideally
 * handled outside of an NFA or DFA.
 */
void mergeAcyclicSuffixes(RoseBuildImpl &tbi) {
    DEBUG_PRINTF("entry\n");

    if (!tbi.cc.grey.mergeSuffixes) {
        return;
    }

    SuffixBouquet suffixes;

    RoseGraph &g = tbi.g;

    for (auto v : vertices_range(g)) {
        shared_ptr<NGHolder> h = g[v].suffix.graph;
        if (!h || tbi.isInETable(v)) {
            continue;
        }

        assert(!g[v].suffix.haig);

        if (num_vertices(*h) >= small_merge_max_vertices(tbi.cc)) {
            continue;
        }

        if (!isAcyclic(*h)) {
            continue;
        }

        if (isLargeLBR(*h, tbi.cc.grey)) {
            DEBUG_PRINTF("not considering LBR suffix for merge\n");
            continue;
        }

        suffixes.insert(g[v].suffix, v);
    }

    deque<SuffixBouquet> suff_groups;
    chunkBouquets(suffixes, suff_groups, MERGE_GROUP_SIZE_MAX);
    DEBUG_PRINTF("chunked %zu suffixes into %zu groups\n", suffixes.size(),
                 suff_groups.size());
    suffixes.clear();

    for (auto &group : suff_groups) {
        mergeSuffixes(tbi, group, true);
    }
}

/**
 * This merge pass combines suffixes from unrelated roles into a single
 * suffix with multiple top events in order to distinguish the triggers
 * from differing roles. mergeAcyclicSuffixes only considers acyclic suffixes
 * while mergeSmallSuffixes only considers small suffixes. The merges will
 * group roles with suffixes in the graph into clusters of at most
 * \ref MERGE_GROUP_SIZE_MAX. Each cluster is processed by iterating over the
 * suffixes and attempting to pairwise merge it with another member. Merges
 * will fail if the result is not implementable, requires too many distinct top
 * events, or if it losses the ability to be accelerated. The merge will modify
 * the existing suffix graph of the one member (g1), the other member updates
 * it graph to refer to g1 instead of its previous graph (g2) and use the new
 * tops created. Other roles may have been sharing g1 - these are unaffected by
 * the change as the existing top events are left untouched. Other roles using
 * g2 are also unaffected as g2 will continue to exist until while it has any
 * roles triggering it.
 *
 * Note: suffixes destined for the LBR are not considered for these merges as
 * the LBR can only handle a single repeat and this type of repeat is ideally
 * handled outside of an NFA or DFA.
 */
void mergeSmallSuffixes(RoseBuildImpl &tbi) {
    DEBUG_PRINTF("entry\n");

    if (!tbi.cc.grey.mergeSuffixes) {
        return;
    }

    RoseGraph &g = tbi.g;
    SuffixBouquet suffixes;

    for (auto v : vertices_range(g)) {
        shared_ptr<NGHolder> h = g[v].suffix.graph;
        if (!h || tbi.isInETable(v)) {
            continue;
        }
        assert(!g[v].suffix.haig);

        // Leave acyclics out for the moment.
        if (isAcyclic(*h)) {
            continue;
        }

        // Small-ish suffixes only.
        if (num_vertices(*h) > 32) {
            continue;
        }

        if (isLargeLBR(*h, tbi.cc.grey)) {
            DEBUG_PRINTF("not considering LBR suffix for merge\n");
            continue;
        }

        suffixes.insert(g[v].suffix, v);
    }

    deque<SuffixBouquet> suff_groups;
    chunkBouquets(suffixes, suff_groups, MERGE_GROUP_SIZE_MAX);
    DEBUG_PRINTF("chunked %zu suffixes into %zu groups\n", suffixes.size(),
                 suff_groups.size());
    suffixes.clear();

    for (auto &group : suff_groups) {
        mergeSuffixes(tbi, group, false);
    }
}

static
void removeDeadOutfixes(vector<OutfixInfo> &outfixes) {
    auto is_dead = [](const OutfixInfo &outfix) { return outfix.is_dead(); };
    outfixes.erase(remove_if(begin(outfixes), end(outfixes), is_dead),
                   end(outfixes));
}

static
void mergeOutfixInfo(OutfixInfo &winner, const OutfixInfo &victim) {
    assert(!winner.is_dead());

    winner.maxBAWidth = max(winner.maxBAWidth, victim.maxBAWidth);
    winner.minWidth = min(winner.minWidth, victim.minWidth);
    winner.maxWidth = max(winner.maxWidth, victim.maxWidth);
    winner.maxOffset = max(winner.maxOffset, victim.maxOffset);
    mergeReverseAccelerationInfo(winner.rev_info, victim.rev_info);

    // This outfix can be ignored in small block mode if both were. The dedupe
    // layer at runtime will protect us from extra matches if only one was in
    // the small block matcher.
    winner.in_sbmatcher &= victim.in_sbmatcher;
}

static
map<NGHolder *, NGHolder *> chunkedNfaMerge(RoseBuildImpl &build,
                                            const vector<NGHolder *> &nfas) {
    map<NGHolder *, NGHolder *> merged;

    vector<NGHolder *> batch;
    for (auto it = begin(nfas), ite = end(nfas); it != ite; ++it) {
        batch.push_back(*it);
        assert((*it)->kind == NFA_OUTFIX);
        if (batch.size() == MERGE_GROUP_SIZE_MAX || next(it) == ite) {
            auto batch_merged = mergeNfaCluster(batch, &build.rm, build.cc);
            insert(&merged, batch_merged);
            batch.clear();
        }
    }

    return merged;
}

static
void mergeOutfixNfas(RoseBuildImpl &tbi, vector<NGHolder *> &nfas) {
    DEBUG_PRINTF("merging %zu nfas\n", nfas.size());
    if (nfas.size() < 2) {
        return;
    }

    vector<OutfixInfo> &outfixes = tbi.outfixes;

    map<NGHolder *, size_t> nfa_mapping;
    for (size_t i = 0; i < outfixes.size(); i++) {
        auto *holder = outfixes[i].holder();
        if (holder) {
            nfa_mapping[holder] = i;
        }
    }

    map<NGHolder *, NGHolder *> merged = chunkedNfaMerge(tbi, nfas);
    if (merged.empty()) {
        return;
    }

    DEBUG_PRINTF("%zu nfas merged\n", merged.size());

    // Update the outfix info for merged holders.
    for (const auto &m : merged) {
        OutfixInfo &victim = outfixes.at(nfa_mapping[m.first]);
        OutfixInfo &winner = outfixes.at(nfa_mapping[m.second]);
        mergeOutfixInfo(winner, victim);
        victim.clear();
    }

    removeDeadOutfixes(outfixes);
}

namespace {
struct MergeMcClellan {
    MergeMcClellan(const ReportManager &rm_in, const Grey &grey_in)
        : rm(rm_in), grey(grey_in) {}

    unique_ptr<raw_dfa> operator()(const raw_dfa *d1, const raw_dfa *d2) const {
        assert(d1 && d2);
        return mergeTwoDfas(d1, d2, DFA_MERGE_MAX_STATES, &rm, grey);
    }

private:
    const ReportManager &rm;
    const Grey &grey;
};

struct MergeHaig {
    explicit MergeHaig(u32 limit_in) : limit(limit_in) {}

    unique_ptr<raw_som_dfa> operator()(const raw_som_dfa *d1,
                                       const raw_som_dfa *d2) const {
        assert(d1 && d2);
        return attemptToMergeHaig({d1, d2}, limit);
    }

private:
    const u32 limit; //!< state limit for merged result.
};
}

/**
 * Generic pairwise merge algorithm that can be used for either McClellan
 * (RawDfa=raw_dfa) or Haig (RawDfa=raw_som_dfa). Delegates the actual merge
 * operation to a merge functor, which allows the caller to set some policy
 * (state limits, etc).
 *
 * This is currently astonishingly simple and just considers every pair of
 * DFAs, slow and steady. We may wish to actually apply a merge ordering
 * strategy in the future.
 */
template<class RawDfa, class MergeFunctor>
static
void pairwiseDfaMerge(vector<RawDfa *> &dfas,
                      ue2::unordered_map<RawDfa *, size_t> &dfa_mapping,
                      vector<OutfixInfo> &outfixes,
                      MergeFunctor merge_func) {
    DEBUG_PRINTF("merging group of size %zu\n", dfas.size());

    for (auto it = dfas.begin(), ite = dfas.end(); it != ite; ++it) {
        if (!*it) {
            continue;
        }
        for (auto jt = next(it); jt != ite; ++jt) {
            if (!*jt) {
                continue;
            }

            DEBUG_PRINTF("try merge %p and %p\n", *it, *jt);
            unique_ptr<RawDfa> rdfa = merge_func(*it, *jt);
            if (!rdfa) {
                continue; // Merge failed.
            }

            DEBUG_PRINTF("merge succeeded, built %p\n", rdfa.get());
            OutfixInfo &winner = outfixes.at(dfa_mapping[*it]);
            OutfixInfo &victim = outfixes.at(dfa_mapping[*jt]);
            assert(!winner.is_dead() && !victim.is_dead());

            RawDfa *dfa_ptr = rdfa.get();
            dfa_mapping[dfa_ptr] = dfa_mapping[*it];
            dfa_mapping.erase(*it);
            winner.proto = move(rdfa);

            mergeOutfixInfo(winner, victim);

            victim.clear();
            *jt = nullptr; // to be deleted.
            *it = dfa_ptr;
        }
    }
}

template<class RawDfa, class MergeFunctor>
static
void chunkedDfaMerge(vector<RawDfa *> &dfas,
                     ue2::unordered_map<RawDfa *, size_t> &dfa_mapping,
                     vector<OutfixInfo> &outfixes,
                     MergeFunctor merge_func) {
    DEBUG_PRINTF("begin merge of %zu dfas\n", dfas.size());

    vector<RawDfa *> out_dfas;
    vector<RawDfa *> chunk;
    for (auto it = begin(dfas), ite = end(dfas); it != ite; ++it) {
        chunk.push_back(*it);
        if (chunk.size() >= DFA_CHUNK_SIZE_MAX || next(it) == ite) {
            pairwiseDfaMerge(chunk, dfa_mapping, outfixes, merge_func);
            out_dfas.insert(end(out_dfas), begin(chunk), end(chunk));
            chunk.clear();
        }
    }

    // Remove null (merged) DFAs and update vector for subsequent use.
    out_dfas.erase(remove(out_dfas.begin(), out_dfas.end(), nullptr),
                   out_dfas.end());
    dfas.swap(out_dfas);
    DEBUG_PRINTF("after merge there are %zu dfas\n", dfas.size());
}

static
void mergeOutfixDfas(RoseBuildImpl &tbi, vector<raw_dfa *> &dfas) {
    DEBUG_PRINTF("merging %zu nfas\n", dfas.size());
    if (dfas.size() < 2) {
        return;
    }

    vector<OutfixInfo> &outfixes = tbi.outfixes;

    /* key is index into outfix array as iterators, etc may be invalidated by
     * element addition. */
    ue2::unordered_map<raw_dfa *, size_t> dfa_mapping;
    for (size_t i = 0; i < outfixes.size(); i++) {
        auto *rdfa = outfixes[i].rdfa();
        if (rdfa) {
            dfa_mapping[rdfa] = i;
        }
    }

    chunkedDfaMerge(dfas, dfa_mapping, outfixes,
                    MergeMcClellan(tbi.rm, tbi.cc.grey));
    removeDeadOutfixes(outfixes);
}

static
void mergeOutfixCombo(RoseBuildImpl &tbi, const ReportManager &rm,
                      const Grey &grey) {
    if (!grey.roseMcClellanOutfix) {
        return;
    }

    DEBUG_PRINTF("merge combo\n");

    bool seen_dfa = false;
    u32 nfa_count = 0;
    for (const auto &outfix : tbi.outfixes) {
        if (outfix.holder()) {
            DEBUG_PRINTF("nfa\n");
            nfa_count++;
        } else if (outfix.rdfa()) {
            DEBUG_PRINTF("dfa\n");
            seen_dfa = true;
        }
    }

    DEBUG_PRINTF("nfa %u dfas present %d\n", nfa_count,
                  (int)seen_dfa);
    if (!nfa_count || (nfa_count == 1 && !seen_dfa)) {
        DEBUG_PRINTF("no combo merges possible\n");
        return;
    }

    /* key is index into outfix array as iterators, etc may be invalidated by
     * element addition. */
    size_t new_dfas = 0;
    ue2::unordered_map<raw_dfa *, size_t> dfa_mapping;
    vector<raw_dfa *> dfas;

    for (auto it = tbi.outfixes.begin(); it != tbi.outfixes.end(); ++it) {
        auto &outfix = *it;
        assert(!outfix.is_dead());

        if (outfix.rdfa()) {
            auto *rdfa = outfix.rdfa();
            dfas.push_back(rdfa);
            dfa_mapping[rdfa] = it - tbi.outfixes.begin();
            continue;
        }

        if (!outfix.holder()) {
            continue;
        }

        NGHolder *h = outfix.holder();
        assert(h->kind == NFA_OUTFIX);
        auto rdfa = buildMcClellan(*h, &rm, grey);
        if (rdfa) {
            // Transform this outfix into a DFA and add it to the merge set.
            dfa_mapping[rdfa.get()] = it - tbi.outfixes.begin();
            dfas.push_back(rdfa.get());
            outfix.proto = move(rdfa);
            new_dfas++;
        }
    }

    DEBUG_PRINTF("constructed %zu new dfas\n", new_dfas);

    if (!new_dfas) {
        /* assumes normal dfas have already been fully merged */
        return;
    }

    chunkedDfaMerge(dfas, dfa_mapping, tbi.outfixes,
                    MergeMcClellan(tbi.rm, tbi.cc.grey));
    removeDeadOutfixes(tbi.outfixes);
}

static
void mergeOutfixHaigs(RoseBuildImpl &tbi, vector<raw_som_dfa *> &dfas,
                      u32 limit) {
    if (dfas.size() < 2) {
        return;
    }

    vector<OutfixInfo> &outfixes = tbi.outfixes;

    ue2::unordered_map<raw_som_dfa *, size_t> dfa_mapping;
    for (size_t i = 0; i < outfixes.size(); i++) {
        auto *haig = outfixes[i].haig();
        if (haig) {
            dfa_mapping[haig] = i;
        }
    }

    chunkedDfaMerge(dfas, dfa_mapping, outfixes, MergeHaig(limit));
    removeDeadOutfixes(outfixes);
}

/**
 * This pass attempts to merge outfix engines together. At this point in time,
 * the engine type (NFA, DFA, Haig) has already been decided for each outfix
 * and outfixes can only merged with others of their same type. NFAs are merged
 * in a priority order based on common prefix length. The other types are
 * merged blindly. Engines are merged to the extent that they can still be
 * implemented efficiently.
 */
void mergeOutfixes(RoseBuildImpl &tbi) {
    if (!tbi.cc.grey.mergeOutfixes) {
        return;
    }

    vector<NGHolder *> nfas;
    vector<raw_dfa *> dfas;
    vector<raw_som_dfa *> som_dfas;

    for (auto &outfix : tbi.outfixes) {
        if (outfix.rdfa()) {
            dfas.push_back(outfix.rdfa());
        } else if (outfix.holder()) {
            nfas.push_back(outfix.holder());
        } else if (outfix.haig()) {
            som_dfas.push_back(outfix.haig());
        }
    }

    DEBUG_PRINTF("merging %zu dfas, %zu nfas\n",
                 dfas.size(), nfas.size());

    mergeOutfixNfas(tbi, nfas);
    mergeOutfixDfas(tbi, dfas);
    mergeOutfixHaigs(tbi, som_dfas, 255);
    mergeOutfixHaigs(tbi, som_dfas, 8192);
    mergeOutfixCombo(tbi, tbi.rm, tbi.cc.grey);
}

static
u32 allowedSquashDistance(const CharReach &cr, u32 min_width,
                          const RoseBuildImpl &tbi,
                          RoseVertex tv) {
    CharReach accept_cr;
    DEBUG_PRINTF("hello |cr|=%zu\n", cr.count());

    const RoseGraph &g = tbi.g;

    /* TODO: inspect further back in the pattern */
    for (u32 lit_id : g[tv].literals) {
        const rose_literal_id &lit = tbi.literals.at(lit_id);
        if (lit.delay) {
            return 0; /* TODO: better */
        }
        if (lit.table != ROSE_FLOATING && lit.table != ROSE_EOD_ANCHORED) {
            return 0;
        }
        assert(!lit.s.empty());
        accept_cr |= *lit.s.rbegin();
    }

    DEBUG_PRINTF("|accept_cr|=%zu\n", accept_cr.count());

    if ((accept_cr & cr).any()) {
        DEBUG_PRINTF("no squash\n");
        return 0; /* the accept byte doesn't always kill the puffette. TODO:
                   * maybe if we look further back we could find something that
                   * would kill the puffette... */
    }

    DEBUG_PRINTF("allowed to squash %u\n", min_width);
    return min_width;
}

void mergePuffixes(RoseBuildImpl &tbi) {
    DEBUG_PRINTF("entry\n");

    if (!tbi.cc.grey.mergeSuffixes) {
        return;
    }

    RoseGraph &g = tbi.g;

    for (auto v : vertices_range(g)) {
        shared_ptr<NGHolder> h = g[v].suffix.graph;
        if (!h) {
            continue;
        }
        assert(!g[v].suffix.haig);
        assert(!g[v].eod_accept);

        assert(onlyOneTop(*h)); /* we should not have merged yet */
        bool fixed_depth = g[v].min_offset == g[v].max_offset;

        if (!isPuffable(*h, fixed_depth, tbi.rm, tbi.cc.grey)) {
            continue;
        }

        PureRepeat repeat;
        if (!isPureRepeat(*h, repeat)) {
            assert(0);
            continue;
        }

        if (repeat.bounds.min == depth(0)) {
            assert(0); // No vacuous puffs allowed.
            continue;
        }

        assert(repeat.bounds.min.is_finite() &&
               repeat.bounds.max.is_reachable());
        assert(repeat.bounds.max == repeat.bounds.min ||
               repeat.bounds.max.is_infinite());

        const bool unbounded = repeat.bounds.max.is_infinite();
        const set<ReportID> reports = all_reports(*h);
        assert(reports.size() == 1);
        ReportID report = *reports.begin();

        DEBUG_PRINTF("got puffette candidate %u:%s\n", report,
                     repeat.bounds.str().c_str());

        raw_puff rp(repeat.bounds.min, unbounded, report, repeat.reach);

        u32 queue;
        u32 event;
        tbi.addChainTail(rp, &queue, &event);
        u32 squashDistance =
            allowedSquashDistance(repeat.reach, repeat.bounds.min, tbi, v);

        Report ir = makeMpvTrigger(event, squashDistance);
        ReportID id = tbi.rm.getInternalId(ir);

        DEBUG_PRINTF("puffette event q%u t%u\n", queue, event);
        g[v].suffix.reset();
        g[v].reports.insert(id);
    }
}

static
void updateCastleSuffix(RoseGraph &g, const shared_ptr<CastleProto> &m,
                        u32 top, const vector<RoseVertex> &verts) {
    DEBUG_PRINTF("merged in as top %u, updating %zu vertices\n", top,
                  verts.size());

    for (auto v : verts) {
        assert(g[v].suffix.castle);
        g[v].suffix.castle = m;
        g[v].suffix.top = top;
    }
}

static
void mergeCastleSuffixes(RoseBuildImpl &tbi,
            vector<shared_ptr<CastleProto> > &castles,
            map<shared_ptr<CastleProto>, vector<RoseVertex> > &castle_map) {
    if (castles.size() <= 1) {
        return;
    }

    RoseGraph &g = tbi.g;
    const size_t max_size = CastleProto::max_occupancy;

    shared_ptr<CastleProto> m = castles.front();
    assert(m->repeats.size() == 1); // Not yet merged.

    // Cache repeats we've already merged, mapped to (prototype, top). That
    // way, we can ensure that we don't construct more than one completely
    // identical repeat.
    typedef map<PureRepeat, pair<shared_ptr<CastleProto>, u32> > RepeatCache;
    RepeatCache cache;
    {
        // Initial entry in cache.
        const u32 top = m->repeats.begin()->first;
        const PureRepeat &pr = m->repeats.begin()->second;
        cache[pr] = make_pair(m, top);
    }

    for (size_t i = 1; i < castles.size(); i++) {
        shared_ptr<CastleProto> c = castles[i];
        assert(c->repeats.size() == 1); // Not yet merged.
        const PureRepeat &pr = c->repeats.begin()->second;
        RepeatCache::const_iterator it = cache.find(pr);
        if (it != cache.end()) {
            DEBUG_PRINTF("reusing cached merge, top=%u, proto=%p\n",
                         it->second.second, it->second.first.get());
            updateCastleSuffix(g, it->second.first, it->second.second,
                               castle_map[c]);
            continue;
        }

        if (m->repeats.size() == max_size) {
            // No room left to merge into 'm'. This one becomes the new 'm'.
            DEBUG_PRINTF("next mergee\n");
            m = c;
            u32 top = m->repeats.begin()->first;
            cache[pr] = make_pair(m, top);
        } else {
            u32 top = m->add(pr);
            updateCastleSuffix(g, m, top, castle_map[c]);
            DEBUG_PRINTF("added to %p, top %u\n", m.get(), top);
            cache[pr] = make_pair(m, top);
        }
    }
}

void mergeCastleSuffixes(RoseBuildImpl &tbi) {
    DEBUG_PRINTF("entry\n");

    if (!(tbi.cc.grey.allowCastle && tbi.cc.grey.mergeSuffixes)) {
        return;
    }

    map<shared_ptr<CastleProto>, vector<RoseVertex>> castles;
    map<CharReach, vector<shared_ptr<CastleProto>>> by_reach;

    RoseGraph &g = tbi.g;

    for (auto v : vertices_range(g)) {
        if (!g[v].suffix.castle) {
            continue;
        }

        shared_ptr<CastleProto> c = g[v].suffix.castle;

        if (c->repeats.size() != 1) {
            // This code assumes it's the only place merging is being done.
            assert(0);
            continue;
        }

        if (!contains(castles, c)) {
            by_reach[c->reach()].push_back(c);
        }
        castles[c].push_back(v);
    }

    for (auto &m : by_reach) {
        DEBUG_PRINTF("reach %s, %zu elements\n", describeClass(m.first).c_str(),
                     m.second.size());
        mergeCastleSuffixes(tbi, m.second, castles);
    }
}

} // namespace ue2
