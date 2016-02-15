/*
 * Copyright (c) 2015-2016, Intel Corporation
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

#include "rose_build_role_aliasing.h"

#include "ue2common.h"
#include "rose_build_impl.h"
#include "rose_build_merge.h"
#include "rose_build_util.h"
#include "grey.h"
#include "nfa/castlecompile.h"
#include "nfa/goughcompile.h"
#include "nfa/mcclellancompile_util.h"
#include "nfagraph/ng_holder.h"
#include "nfagraph/ng_is_equal.h"
#include "nfagraph/ng_limex.h"
#include "nfagraph/ng_prune.h"
#include "nfagraph/ng_restructuring.h"
#include "nfagraph/ng_uncalc_components.h"
#include "nfagraph/ng_util.h"
#include "util/bitutils.h"
#include "util/compile_context.h"
#include "util/container.h"
#include "util/graph.h"
#include "util/graph_range.h"
#include "util/order_check.h"
#include "util/ue2_containers.h"

#include <algorithm>
#include <numeric>
#include <vector>
#include <boost/functional/hash/hash.hpp>
#include <boost/graph/adjacency_iterator.hpp>
#include <boost/range/adaptor/map.hpp>

using namespace std;
using boost::adaptors::map_values;

namespace ue2 {

namespace {
// Used for checking edge sets (both in- and out-) against each other.
struct EdgeAndVertex {
    EdgeAndVertex(const RoseEdge &e, const RoseVertex v_,
                  const RoseGraph &g) : v(v_), eprops(g[e]) {}
    virtual ~EdgeAndVertex() {}

    virtual bool operator<(const EdgeAndVertex &a) const {
        if (v != a.v) {
            return v < a.v;
        }
        if (eprops.minBound != a.eprops.minBound) {
            return eprops.minBound < a.eprops.minBound;
        }
        if (eprops.maxBound != a.eprops.maxBound) {
            return eprops.maxBound < a.eprops.maxBound;
        }
        if (eprops.rose_top != a.eprops.rose_top) {
            return eprops.rose_top < a.eprops.rose_top;

        }
        return eprops.history < a.eprops.history;
    }

    virtual bool operator==(const EdgeAndVertex &a) const {
        return v == a.v &&
               eprops.minBound == a.eprops.minBound &&
               eprops.maxBound == a.eprops.maxBound &&
               eprops.rose_top == a.eprops.rose_top &&
               eprops.history == a.eprops.history;
    }

private:
    RoseVertex v;
    const RoseEdgeProps &eprops;
};

struct AliasOutEdge : EdgeAndVertex {
    AliasOutEdge(const RoseEdge &e, const RoseGraph &g) :
        EdgeAndVertex(e, target(e, g), g) {}
};

struct AliasInEdge : EdgeAndVertex {
    AliasInEdge(const RoseEdge &e, const RoseGraph &g) :
        EdgeAndVertex(e, source(e, g), g) {}
};

class CandidateSet {
public:
    typedef RoseVertexSet::iterator iterator;
    typedef RoseVertex key_type;

    explicit CandidateSet(const VertexIndexComp &comp) : main_cont(comp) {}

    iterator begin() { return main_cont.begin(); }
    iterator end() { return main_cont.end(); }

    bool contains(RoseVertex a) const {
        return hash_cont.find(a) != hash_cont.end();
    }

    void insert(RoseVertex a) {
        main_cont.insert(a);
        hash_cont.insert(a);
    }

    void erase(iterator aa) {
        RoseVertex a = *aa;
        main_cont.erase(aa);
        hash_cont.erase(a);
    }

    void erase(RoseVertex a) {
        main_cont.erase(a);
        hash_cont.erase(a);
    }

    size_t size() const {
        assert(hash_cont.size() == main_cont.size());
        return main_cont.size();
    }

    bool empty() const {
        assert(hash_cont.size() == main_cont.size());
        return main_cont.empty();
    }

private:
    /* if a vertex is worth storing, it is worth storing twice */
    RoseVertexSet main_cont; /* deterministic iterator */
    ue2::unordered_set<RoseVertex> hash_cont; /* member checks */
};

/**
 * \brief Mapping from a particular rose engine to a set of associated
 * vertices.
 */
typedef ue2::unordered_map<left_id, set<RoseVertex> > revRoseMap;

} // namespace

static
void populateRevRoseMap(const RoseGraph &g, revRoseMap *out) {
    for (auto v : vertices_range(g)) {
        if (g[v].left) {
            (*out)[g[v].left].insert(v);
        }
    }
}

// Check successor set: must lead to the same vertices via edges with the
// same properties.
static
bool sameSuccessors(RoseVertex a, RoseVertex b, const RoseGraph &g) {
    if (out_degree(a, g) != out_degree(b, g)) {
        return false;
    }

    set<AliasOutEdge> succs_a, succs_b;

    for (const auto &e : out_edges_range(a, g)) {
        succs_a.insert(AliasOutEdge(e, g));
    }

    for (const auto &e : out_edges_range(b, g)) {
        succs_b.insert(AliasOutEdge(e, g));
    }

    return (succs_a == succs_b);
}

/* unlike LeftEngInfo::==, this does a deep check to see if the leftfixes are
 * equivalent rather than checking for pointer equality. */
static
bool hasEqualLeftfixes(RoseVertex a, RoseVertex b, const RoseGraph &g) {
    assert(g[a].left || g[b].left);
    if (!g[a].left || !g[b].left) {
        return false;
    }
    const LeftEngInfo &a_left = g[a].left;
    const LeftEngInfo &b_left = g[b].left;

    if (a_left.castle && b_left.castle) {
        return is_equal(*a_left.castle, a_left.leftfix_report,
                        *b_left.castle, b_left.leftfix_report);
    }

    if (a_left.graph && b_left.graph) {
        /* non-castle engines have graphs */
        return is_equal(*a_left.graph, a_left.leftfix_report, *b_left.graph,
                        b_left.leftfix_report);
    }

    /* graph <-> castle cases are not equal */
    return false;
}

// Check predecessor set: must come from the same vertices via edges with
// the same properties.
static
bool samePredecessors(RoseVertex a, RoseVertex b, const RoseGraph &g) {
    if (in_degree(a, g) != in_degree(b, g)) {
        return false;
    }

    set<AliasInEdge> preds_a, preds_b;

    for (const auto &e : in_edges_range(a, g)) {
        preds_a.insert(AliasInEdge(e, g));
    }

    for (const auto &e : in_edges_range(b, g)) {
        preds_b.insert(AliasInEdge(e, g));
    }

    if (preds_a != preds_b) {
        return false;
    }

    if (g[a].left || g[b].left) {
        if (!hasEqualLeftfixes(a, b, g)) {
            return false;
        }

        for (const auto &e_a : in_edges_range(a, g)) {
            bool exists;
            RoseEdge e;
            tie(e, exists) = edge_by_target(source(e_a, g), b, g);
            if (!exists || g[e].rose_top != g[e_a].rose_top) {
                DEBUG_PRINTF("bad tops\n");
                return false;
            }
        }
    }

    return true;
}

static
bool hasCommonSuccWithBadBounds(RoseVertex a, RoseVertex b, const RoseGraph &g) {
    for (const auto &e_a : out_edges_range(a, g)) {
        bool exists;
        RoseEdge e;
        tie(e, exists) = edge(b, target(e_a, g), g);
        if (exists) {
            if (g[e_a].maxBound < g[e].minBound
                || g[e].maxBound < g[e_a].minBound) {
                return true;
            }
            if (g[e_a].rose_top != g[e].rose_top) {
                // Can't trigger two tops on the same leftfix, we can't merge
                // this.
                return true;
            }
        }
    }
    return false;
}

static
bool hasCommonPredWithBadBounds(RoseVertex a, RoseVertex b, const RoseGraph &g) {
    for (const auto &e_a : in_edges_range(a, g)) {
        bool exists;
        RoseEdge e;
        tie(e, exists) = edge_by_target(source(e_a, g), b, g);
        if (exists) {
            if (g[e_a].maxBound < g[e].minBound
                || g[e].maxBound < g[e_a].minBound) {
                return true;
            }

            // XXX: if we're merging two vertices with different roses, we
            // cannot allow them to share a pred, as we would be unable to
            // merge the (necessarily different) tops on the in-edges. This
            // could be relaxed if we made the tops mergeable (by making
            // edge_top a bitfield, for example).
            if (g[a].left != g[b].left) {
                return true;
            }

        }
    }
    return false;
}

static
bool canMergeLiterals(RoseVertex a, RoseVertex b, const RoseBuildImpl &tbi) {
    const auto &lits_a = tbi.g[a].literals;
    const auto &lits_b = tbi.g[b].literals;
    assert(!lits_a.empty() && !lits_b.empty());

    // If both vertices have only pseudo-dotstar in-edges, we can merge
    // literals of different lengths and can avoid the check below.
    if (tbi.hasOnlyPseudoStarInEdges(a) && tbi.hasOnlyPseudoStarInEdges(b)) {
        DEBUG_PRINTF("both have pseudo-dotstar in-edges\n");
        return true;
    }

    // Otherwise, all the literals involved must have the same length.
    for (u32 a_id : lits_a) {
        const rose_literal_id &la = tbi.literals.right.at(a_id);
        for (u32 b_id : lits_b) {
            const rose_literal_id &lb = tbi.literals.right.at(b_id);

            if (la.elength() != lb.elength()) {
                DEBUG_PRINTF("bad merge %zu!=%zu '%s', '%s'\n", la.elength(),
                             lb.elength(), la.s.c_str(), lb.s.c_str());
                return false;
            }
        }
    }

    return true;
}

static
bool isAliasingCandidate(RoseVertex v, const RoseBuildImpl &tbi) {
    const RoseVertexProps &props = tbi.g[v];

    // Must have literals.
    if (props.literals.empty()) {
        return false;
    }

    assert(*props.literals.begin() != MO_INVALID_IDX);

    // Any vertex involved in a "ghost" relationship has already been disallowed

    return true;
}

static
bool sameRoleProperties(const RoseBuildImpl &build, RoseVertex a, RoseVertex b) {
    const RoseGraph &g = build.g;
    const RoseVertexProps &aprops = g[a], &bprops = g[b];

    if (aprops.eod_accept != bprops.eod_accept) {
        return false;
    }

    // We don't want to merge a role with LAST_BYTE history with one without,
    // as a role that can only be triggered at EOD cannot safely precede
    // "ordinary" roles.
    if (hasLastByteHistorySucc(g, a) != hasLastByteHistorySucc(g, b)) {
        return false;
    }

    // We certainly don't want to merge root roles with non-root roles.
    /* TODO: explain */
    if (build.isRootSuccessor(a) != build.isRootSuccessor(b)) {
        return false;
    }

    if (aprops.som_adjust != bprops.som_adjust) {
        return false;
    }

    /* "roses are mergeable" check are handled elsewhere  */

    return true;
}

/* Checks compatibility of role properties if we require that two roles are right
 * equiv. */
static
bool sameRightRoleProperties(const RoseBuildImpl &build, RoseVertex a,
                             RoseVertex b) {
    const RoseGraph &g = build.g;
    const RoseVertexProps &aprops = g[a], &bprops = g[b];

    if (aprops.reports != bprops.reports) {
        return false;
    }

    if (hasAnchHistorySucc(g, a) != hasAnchHistorySucc(g, b)) {
        return false;
    }

    // If the history type is ANCH, then we need to be careful that we only
    // merge literals that occur at the same offsets.
    if (hasAnchHistorySucc(g, a) || hasAnchHistorySucc(g, b)) {
        if (aprops.min_offset != bprops.min_offset
            || aprops.max_offset != bprops.max_offset) {
            return false;
        }
    }

    if (aprops.suffix != bprops.suffix) {
        return false;
    }

    return true;
}

/**
 * Hash on some deterministic props checked in sameRoleProperties + properties
 * required for right equivalence.
 */
static
size_t hashRightRoleProperties(RoseVertex v, const RoseGraph &g) {
    using boost::hash_combine;
    using boost::hash_range;

    const RoseVertexProps &props = g[v];

    size_t val = 0;
    hash_combine(val, hash_range(begin(props.reports), end(props.reports)));

    if (props.suffix) {
        const auto &suffix = props.suffix;
        if (suffix.castle) {
            hash_combine(val, suffix.castle->reach());
            hash_combine(val, suffix.castle->repeats.size());
        }
        if (suffix.graph) {
            hash_combine(val, num_vertices(*suffix.graph));
        }
        if (suffix.haig) {
            hash_combine(val, hash_dfa(*suffix.haig));
        }
    }

    return val;
}

static
void removeVertexFromMaps(RoseVertex v, RoseBuildImpl &build, revRoseMap &rrm) {
    if (build.g[v].left) {
        const left_id left(build.g[v].left);
        assert(contains(rrm[left], v));
        rrm[left].erase(v);
    }
}

static
void mergeEdgeAdd(RoseVertex u, RoseVertex v, const RoseEdge &from_edge,
                  const RoseEdge *to_edge, RoseGraph &g) {
    const RoseEdgeProps &from_props = g[from_edge];

    if (!to_edge) {
        DEBUG_PRINTF("adding edge [%zu,%zu]\n", g[u].idx, g[v].idx);
        add_edge(u, v, from_props, g);
    } else {
        // union of the two edges.
        DEBUG_PRINTF("updating edge [%zu,%zu]\n", g[u].idx, g[v].idx);
        RoseEdgeProps &to_props = g[*to_edge];
        to_props.minBound = min(to_props.minBound, from_props.minBound);
        to_props.maxBound = max(to_props.maxBound, from_props.maxBound);
        assert(to_props.rose_top == from_props.rose_top);
    }
}

/* clone a's edges onto b */
static
void mergeEdges(RoseVertex a, RoseVertex b, RoseGraph &g) {
    // All the edges to or from b for quick lookup.
    typedef map<RoseVertex, RoseEdge> EdgeCache;
    EdgeCache b_edges;

    // Cache b's in-edges so we can look them up by source quickly.
    for (const auto &e : in_edges_range(b, g)) {
        RoseVertex u = source(e, g);
        b_edges.insert(make_pair(u, e));
    }

    // Add a's in-edges to b, merging them in where b already has the new edge.
    // Once handled, the in-edges to a are removed.
    RoseGraph::in_edge_iterator ei, ee;
    tie(ei, ee) = in_edges(a, g);
    while (ei != ee) {
        RoseVertex u = source(*ei, g);
        EdgeCache::const_iterator it = b_edges.find(u);
        const RoseEdge *to_edge = (it == b_edges.end() ? nullptr : &it->second);
        mergeEdgeAdd(u, b, *ei, to_edge, g);
        remove_edge(*ei++, g);
    }

    // Cache b's out-edges so we can look them up by target quickly.
    b_edges.clear();
    for (const auto &e : out_edges_range(b, g)) {
        RoseVertex v = target(e, g);
        b_edges.insert(make_pair(v, e));
    }

    // Add a's out-edges to b, merging them in where b already has the new edge.
    // Once handled, the out-edges to a are removed.
    RoseGraph::out_edge_iterator oi, oe;
    tie(oi, oe) = out_edges(a, g);
    while (oi != oe) {
        RoseVertex v = target(*oi, g);
        EdgeCache::const_iterator it = b_edges.find(v);
        const RoseEdge *to_edge = (it == b_edges.end() ? nullptr : &it->second);
        mergeEdgeAdd(b, v, *oi, to_edge, g);
        remove_edge(*oi++, g);
    }

    // Vertex a should no longer have any in- or out-edges.
    assert(degree(a, g) == 0);
}

static
void mergeLiteralSets(RoseVertex a, RoseVertex b, RoseBuildImpl &tbi) {
    RoseGraph &g = tbi.g;
    const auto &a_literals = g[a].literals;
    for (u32 lit_id : a_literals) {
        auto &lit_vertices = tbi.literal_info[lit_id].vertices;
        lit_vertices.erase(a);
        lit_vertices.insert(b);
    }

    insert(&g[b].literals, a_literals);
}

// Merge role 'a' into 'b'.
static
void mergeVertices(RoseVertex a, RoseVertex b, RoseBuildImpl &tbi,
                   revRoseMap &rrm) {
    RoseGraph &g = tbi.g;
    DEBUG_PRINTF("merging vertex %zu into %zu\n", g[a].idx, g[b].idx);

    // Merge role properties.
    assert(g[a].eod_accept == g[b].eod_accept);
    assert(g[a].left == g[b].left);

    insert(&g[b].reports, g[a].reports);

    // In some situations (ghost roles etc), we can have different groups.
    assert(!g[a].groups && !g[b].groups); /* current structure means groups
                                           * haven't been assigned yet */
    g[b].groups |= g[a].groups;

    g[b].min_offset = min(g[a].min_offset, g[b].min_offset);
    g[b].max_offset = max(g[a].max_offset, g[b].max_offset);

    mergeLiteralSets(a, b, tbi);

    if (!g[b].suffix) {
        g[b].suffix = g[a].suffix;
    } else {
        assert(!g[a].suffix || g[b].suffix == g[a].suffix);
    }

    mergeEdges(a, b, g);
    removeVertexFromMaps(a, tbi, rrm);
}

/**
 * Faster version of \ref mergeVertices for diamond merges, for which we know
 * that the in- and out-edge sets, reports and suffixes are identical.
 */
static
void mergeVerticesDiamond(RoseVertex a, RoseVertex b, RoseBuildImpl &tbi,
                          revRoseMap &rrm) {
    RoseGraph &g = tbi.g;
    DEBUG_PRINTF("merging vertex %zu into %zu\n", g[a].idx, g[b].idx);

    // Merge role properties. For a diamond merge, most properties are already
    // the same (with the notable exception of the literal set).
    assert(g[a].eod_accept == g[b].eod_accept);
    assert(g[a].left == g[b].left);
    assert(g[a].reports == g[b].reports);
    assert(g[a].suffix == g[b].suffix);

    // In some situations (ghost roles etc), we can have different groups.
    assert(!g[a].groups && !g[b].groups); /* current structure means groups
                                           * haven't been assigned yet */
    g[b].groups |= g[a].groups;

    g[b].min_offset = min(g[a].min_offset, g[b].min_offset);
    g[b].max_offset = max(g[a].max_offset, g[b].max_offset);

    mergeLiteralSets(a, b, tbi);
    removeVertexFromMaps(a, tbi, rrm);
}

static never_inline
void findCandidates(const RoseBuildImpl &tbi, CandidateSet *candidates) {
    ue2::unordered_set<RoseVertex> disallowed;

    // We currently deny candidature to any vertex involved in a "ghost"
    // relationship.
    for (const auto &m : tbi.ghost) {
        disallowed.insert(m.first);
        disallowed.insert(m.second);
    }

    for (auto v : vertices_range(tbi.g)) {
        // Ignore ghost relationships.
        if (contains(disallowed, v)) {
            continue;
        }

        if (isAliasingCandidate(v, tbi)) {
            DEBUG_PRINTF("candidate %zu\n", tbi.g[v].idx);
            DEBUG_PRINTF("lits: %u\n", *tbi.g[v].literals.begin());
            candidates->insert(v);
        }
    }

    assert(candidates->size() <= num_vertices(tbi.g));
    DEBUG_PRINTF("found %zu/%zu candidates\n", candidates->size(),
                 num_vertices(tbi.g));
}

static
RoseVertex pickSucc(const RoseVertex v, const RoseGraph &g) {
    RoseGraph::adjacency_iterator ai, ae;
    tie(ai, ae) = adjacent_vertices(v, g);
    if (ai == ae) {
        return RoseGraph::null_vertex();
    }
    return *ai;
}

static
RoseVertex pickPred(const RoseVertex v, const RoseGraph &g,
                    const RoseBuildImpl &tbi) {
    RoseGraph::in_edge_iterator ei, ee;
    tie(ei, ee) = in_edges(v, g);
    if (ei == ee) {
        assert(0); // every candidate should have in-degree!
        return RoseGraph::null_vertex();
    }

    // Avoid roots if we have other options, since it doesn't matter to the
    // merge pass which predecessor we pick.
    RoseVertex u = source(*ei, g);
    while (tbi.isAnyStart(u) && ++ei != ee) {
        u = source(*ei, g);
    }
    return u;
}

template<>
bool contains<>(const CandidateSet &container, const RoseVertex &key) {
    return container.contains(key);
}

// Simplified version of hasCommonPredWithBadBounds for diamond merges.
static
bool hasCommonPredWithDiffRoses(RoseVertex a, RoseVertex b,
                                const RoseGraph &g) {
    if (!g[a].left || !g[b].left) {
        DEBUG_PRINTF("one of (a, b) doesn't have a prefix\n");
        return true;
    }

    // XXX: if we're merging two vertices with different leftfixes, we
    // cannot allow them to share a pred, as we would be unable to
    // merge the (necessarily different) tops on the in-edges. This
    // could be relaxed if we made the tops mergeable (by making
    // edge_top a bitfield, for example).

    const bool equal_roses = hasEqualLeftfixes(a, b, g);

    for (const auto &e_a : in_edges_range(a, g)) {
        bool exists;
        RoseEdge e;
        tie(e, exists) = edge_by_target(source(e_a, g), b, g);
        if (exists) {
            DEBUG_PRINTF("common pred, e_r=%d r_t %u,%u\n",
                         (int)equal_roses, g[e].rose_top, g[e_a].rose_top);
            if (!equal_roses) {
                DEBUG_PRINTF("different roses\n");
                return true;
            }
            if (g[e].rose_top != g[e_a].rose_top) {
                DEBUG_PRINTF("bad tops\n");
                return true;
            }
        }
    }
    DEBUG_PRINTF("ok\n");
    return false;
}

static
void pruneReportIfUnused(const RoseBuildImpl &tbi, shared_ptr<NGHolder> h,
                         const set<RoseVertex> &verts, ReportID report) {
    DEBUG_PRINTF("trying to prune %u from %p (v %zu)\n", report, h.get(),
                 verts.size());
    for (RoseVertex v : verts) {
        if (tbi.g[v].left.graph == h && tbi.g[v].left.leftfix_report == report) {
            DEBUG_PRINTF("report %u still in use\n", report);
            return;
        }
    }

    if (!verts.empty()) {
        // Report no longer in use, but graph h is still alive: we should prune
        // the report if we can do so without rendering the graph
        // unimplementable.

        DEBUG_PRINTF("report %u has been merged away, pruning\n", report);
        assert(h->kind == tbi.isRootSuccessor(*verts.begin()) ? NFA_PREFIX
                                                              : NFA_INFIX);
        unique_ptr<NGHolder> h_new = cloneHolder(*h);
        pruneReport(*h_new, report);

        if (isImplementableNFA(*h_new, nullptr, tbi.cc)) {
            clear_graph(*h);
            cloneHolder(*h, *h_new);
        } else {
            DEBUG_PRINTF("prune produced unimplementable graph, "
                          "leaving as-is\n");
        }
    }
}

/** \brief Remove any tops that don't lead to the given report from this
 * Castle. */
static
void pruneCastle(CastleProto &castle, ReportID report) {
    unordered_set<u32> dead; // tops to remove.
    for (const auto &m : castle.repeats) {
        if (!contains(m.second.reports, report)) {
            dead.insert(m.first);
        }
    }

    for (const auto &top : dead) {
        castle.erase(top);
    }

    assert(!castle.repeats.empty());
}

/** \brief Set all reports to the given one. */
static
void setReports(CastleProto &castle, ReportID report) {
    for (auto &repeat : castle.repeats | map_values) {
        repeat.reports.clear();
        repeat.reports.insert(report);
    }
}

static
void updateEdgeTops(RoseGraph &g, RoseVertex v, const map<u32, u32> &top_map) {
    for (const auto &e : in_edges_range(v, g)) {
        g[e].rose_top = top_map.at(g[e].rose_top);
    }
}

static
void pruneUnusedTops(CastleProto &castle, const RoseGraph &g,
                     const set<RoseVertex> &verts) {
    ue2::unordered_set<u32> used_tops;
    for (auto v : verts) {
        assert(g[v].left.castle.get() == &castle);

        for (const auto &e : in_edges_range(v, g)) {
            u32 top = g[e].rose_top;
            assert(contains(castle.repeats, top));
            used_tops.insert(top);
        }
    }

    DEBUG_PRINTF("castle has %zu tops, graph has %zu tops\n",
                 castle.repeats.size(), used_tops.size());

    for (u32 top : assoc_keys(castle.repeats)) {
        if (!contains(used_tops, top)) {
            DEBUG_PRINTF("removing unused top %u\n", top);
            castle.erase(top);
        }
    }
}

static
void pruneUnusedTops(NGHolder &h, const RoseGraph &g,
                     const set<RoseVertex> &verts) {
    ue2::unordered_set<u32> used_tops;
    for (auto v : verts) {
        assert(g[v].left.graph.get() == &h);

        for (const auto &e : in_edges_range(v, g)) {
            u32 top = g[e].rose_top;
            used_tops.insert(top);
        }
    }

    vector<NFAEdge> dead;
    for (const auto &e : out_edges_range(h.start, h)) {
        NFAVertex v = target(e, h);
        if (v == h.startDs) {
            continue; // stylised edge, leave it alone.
        }
        u32 top = h[e].top;
        if (!contains(used_tops, top)) {
            DEBUG_PRINTF("edge (start,%u) has unused top %u\n",
                          h[v].index, top);
            dead.push_back(e);
        }
    }

    if (dead.empty()) {
        return;
    }

    remove_edges(dead, h);
    pruneUseless(h);
    clearReports(h); // As we may have removed vacuous edges.
}

static
bool mergeSameCastle(RoseBuildImpl &tbi, RoseVertex a, RoseVertex b,
                     revRoseMap &rrm) {
    RoseGraph &g = tbi.g;
    LeftEngInfo &a_left = g[a].left;
    LeftEngInfo &b_left = g[b].left;
    CastleProto &castle = *a_left.castle;

    DEBUG_PRINTF("a report=%u, b report=%u\n", a_left.leftfix_report,
                  b_left.leftfix_report);

    u32 merge_count = 0;
    for (const auto &c : castle.repeats) {
        DEBUG_PRINTF("top %u -> %s report %u\n", c.first,
                     c.second.bounds.str().c_str(), *c.second.reports.begin());
        if (contains(c.second.reports, a_left.leftfix_report) ||
            contains(c.second.reports, b_left.leftfix_report)) {
            merge_count++;
        }
    }

    if (castle.repeats.size() + merge_count > castle.max_occupancy) {
        DEBUG_PRINTF("too big to merge\n");
        return false;
    }

    const ReportID new_report = tbi.getNewNfaReport();
    map<u32, u32> a_top_map, b_top_map;

    for (const auto &c : castle.repeats) {
        u32 old_top = c.first;
        if (contains(c.second.reports, a_left.leftfix_report)) {
            PureRepeat pr = c.second;
            pr.reports.clear();
            pr.reports.insert(new_report);
            u32 new_top = castle.merge(pr);
            assert(new_top < castle.max_occupancy);
            a_top_map[old_top] = new_top;
        } else if (contains(c.second.reports, b_left.leftfix_report)) {
            PureRepeat pr = c.second;
            pr.reports.clear();
            pr.reports.insert(new_report);
            u32 new_top = castle.merge(pr);
            assert(new_top < castle.max_occupancy);
            b_top_map[old_top] = new_top;
        }
    }

    assert(contains(rrm[b_left], b));
    rrm[b_left].erase(b);
    rrm[a_left].insert(b);

    a_left.leftfix_report = new_report;
    b_left.leftfix_report = new_report;
    assert(a_left == b_left);

    updateEdgeTops(g, a, a_top_map);
    updateEdgeTops(g, b, b_top_map);

    pruneUnusedTops(castle, g, rrm[a_left]);
    return true;
}

static
bool attemptRoseCastleMerge(RoseBuildImpl &tbi, bool preds_same, RoseVertex a,
                            RoseVertex b, bool trivialCasesOnly,
                            revRoseMap &rrm) {
    RoseGraph &g = tbi.g;
    LeftEngInfo &a_left = g[a].left;
    LeftEngInfo &b_left = g[b].left;
    left_id a_left_id(a_left);
    left_id b_left_id(b_left);
    CastleProto &a_castle = *a_left_id.castle();
    CastleProto &b_castle = *b_left_id.castle();

    if (a_castle.reach() != b_castle.reach()) {
        DEBUG_PRINTF("different reach\n");
        return false;
    }

    DEBUG_PRINTF("a castle=%p, report=%u\n", &a_castle, a_left.leftfix_report);
    DEBUG_PRINTF("b castle=%p, report=%u\n", &b_castle, b_left.leftfix_report);

    if (&a_castle == &b_castle) {
        DEBUG_PRINTF("castles are the same\n");
        return mergeSameCastle(tbi, a, b, rrm);
    }

    if (is_equal(a_castle, a_left.leftfix_report, b_castle,
                 b_left.leftfix_report)) {
        DEBUG_PRINTF("castles are equiv with respect to reports\n");
        if (rrm[a_left_id].size() == 1) {
            /* nobody else is using a_castle */
            rrm[b_left_id].erase(b);
            rrm[a_left_id].insert(b);
            pruneUnusedTops(b_castle, g, rrm[b_left_id]);
            b_left.castle = a_left.castle;
            b_left.leftfix_report = a_left.leftfix_report;
            DEBUG_PRINTF("OK -> only user of a_castle\n");
            return true;
        }

        if (rrm[b_left_id].size() == 1) {
            /* nobody else is using b_castle */
            rrm[a_left_id].erase(a);
            rrm[b_left_id].insert(a);
            pruneUnusedTops(a_castle, g, rrm[a_left_id]);
            a_left.castle = b_left.castle;
            a_left.leftfix_report = b_left.leftfix_report;
            DEBUG_PRINTF("OK -> only user of b_castle\n");
            return true;
        }

        if (preds_same) {
            /* preds are the same anyway in diamond/left merges just need to
             * check that all the literals in rrm[b_h] can handle a_h */
            for (auto v : rrm[b_left_id]) {
                if (!mergeableRoseVertices(tbi, a, v)) {
                    goto literal_mismatch_1;
                }
            }

            rrm[a_left_id].erase(a);
            rrm[b_left_id].insert(a);
            pruneUnusedTops(a_castle, g, rrm[a_left_id]);
            a_left.castle = b_left.castle;
            a_left.leftfix_report = b_left.leftfix_report;
            DEBUG_PRINTF("OK -> same preds ???\n");
            return true;
        literal_mismatch_1:
            /* preds are the same anyway in diamond/left merges just need to
             * check that all the literals in rrm[a_h] can handle b_h */
            for (auto v : rrm[a_left_id]) {
                if (!mergeableRoseVertices(tbi, v, b)) {
                    goto literal_mismatch_2;
                }
            }

            rrm[b_left_id].erase(b);
            rrm[a_left_id].insert(b);
            pruneUnusedTops(b_castle, g, rrm[b_left_id]);
            b_left.castle = a_left.castle;
            b_left.leftfix_report = a_left.leftfix_report;
            DEBUG_PRINTF("OK -> same preds ???\n");
            return true;
        literal_mismatch_2:;
        }
        DEBUG_PRINTF("OK -> create new\n");
        /* we need to create a new graph as there may be other people
         * using b_left and it would be bad if a's preds started triggering it
         */
        ReportID new_report = tbi.getNewNfaReport();
        shared_ptr<CastleProto> new_castle = make_shared<CastleProto>(a_castle);
        pruneCastle(*new_castle, a_left.leftfix_report);
        setReports(*new_castle, new_report);

        rrm[a_left_id].erase(a);
        rrm[b_left_id].erase(b);
        pruneUnusedTops(*a_left.castle, g, rrm[a_left_id]);
        pruneUnusedTops(*b_left.castle, g, rrm[b_left_id]);

        a_left.leftfix_report = new_report;
        b_left.leftfix_report = new_report;
        a_left.castle = new_castle;
        b_left.castle = new_castle;

        assert(a_left == b_left);
        rrm[a_left].insert(a);
        rrm[a_left].insert(b);
        pruneUnusedTops(*new_castle, g, rrm[a_left]);
        return true;
    }

    // Everything after this point requires more work, so we guard it with the
    // trivial cases argument..
    if (trivialCasesOnly) {
        return false;
    }

    // Only infixes. Prefixes require special care when doing non-trivial
    // merges.
    if (!tbi.isNonRootSuccessor(a) || !tbi.isNonRootSuccessor(b)) {
        return false;
    }

    set<RoseVertex> &b_verts = rrm[b_left_id];
    set<RoseVertex> aa;
    aa.insert(a);

    if (!mergeableRoseVertices(tbi, aa, b_verts)) {
        DEBUG_PRINTF("vertices not mergeable\n");
        return false;
    }

    if (!tbi.cc.grey.roseMultiTopRoses || !tbi.cc.grey.allowCastle) {
        return false;
    }

    DEBUG_PRINTF("merging into new castle\n");

    // Clone new castle with a's repeats in it, set to a new report.
    ReportID new_report = tbi.getNewNfaReport();
    shared_ptr<CastleProto> m_castle = make_shared<CastleProto>(a_castle);
    pruneCastle(*m_castle, a_left.leftfix_report);
    setReports(*m_castle, new_report);

    // Merge in the relevant repeats from b with the new report. Note that
    // we'll have to remap tops appropriately.
    map<u32, u32> b_top_map;
    for (const auto &e : in_edges_range(b, g)) {
        u32 top = g[e].rose_top;
        assert(contains(b_castle.repeats, top));

        PureRepeat pr = b_castle.repeats[top]; // mutable copy
        pr.reports.clear();
        pr.reports.insert(new_report);

        // We should be protected from merging common preds with tops leading
        // to completely different repeats by earlier checks, but just in
        // case...
        if (edge(source(e, g), a, g).second) {
            RoseEdge a_edge = edge(source(e, g), a, g).first;
            u32 a_top = g[a_edge].rose_top;
            const PureRepeat &a_pr = m_castle->repeats[a_top]; // new report
            if (pr != a_pr) {
                DEBUG_PRINTF("merge failed, common pred with diff repeat\n");
                return false;
            }
        }

        u32 new_top = m_castle->merge(pr);
        if (new_top == CastleProto::max_occupancy) {
            DEBUG_PRINTF("merge failed\n");
            return false;
        }
        b_top_map[top] = new_top;
    }

    updateEdgeTops(g, b, b_top_map);

    DEBUG_PRINTF("merged into castle containing %zu repeats\n",
                  m_castle->repeats.size());

    rrm[a_left_id].erase(a);
    rrm[b_left_id].erase(b);
    pruneUnusedTops(*a_left.castle, g, rrm[a_left_id]);
    pruneUnusedTops(*b_left.castle, g, rrm[b_left_id]);

    a_left.castle = m_castle;
    a_left.leftfix_report = new_report;
    b_left.castle = m_castle;
    b_left.leftfix_report = new_report;

    assert(a_left == b_left);
    rrm[a_left].insert(a);
    rrm[a_left].insert(b);
    pruneUnusedTops(*m_castle, g, rrm[a_left]);
    return true;
}

static
bool attemptRoseGraphMerge(RoseBuildImpl &tbi, bool preds_same, RoseVertex a,
                           RoseVertex b, bool trivialCasesOnly,
                           revRoseMap &rrm) {
    RoseGraph &g = tbi.g;
    LeftEngInfo &a_left = g[a].left;
    LeftEngInfo &b_left = g[b].left;
    left_id a_left_id(a_left);
    left_id b_left_id(b_left);
    shared_ptr<NGHolder> a_h = a_left.graph;
    shared_ptr<NGHolder> b_h = b_left.graph;
    assert(a_h && b_h);

    // If we only differ in reports, this is a very easy merge. Just use b's
    // report for both.
    /* Actually not so easy, there may be other poor suckers using a and/or b's
     * reports who will be surprised by this change */
    if (a_h == b_h) {
        DEBUG_PRINTF("OK -> same actual holder\n");
        ReportID a_oldreport = a_left.leftfix_report;
        ReportID b_oldreport = b_left.leftfix_report;
        ReportID new_report = tbi.getNewNfaReport();
        duplicateReport(*a_h, a_left.leftfix_report, new_report);
        duplicateReport(*b_h, b_left.leftfix_report, new_report);
        a_left.leftfix_report = new_report;
        b_left.leftfix_report = new_report;
        pruneReportIfUnused(tbi, b_h, rrm[b_left_id], a_oldreport);
        pruneReportIfUnused(tbi, b_h, rrm[b_left_id], b_oldreport);
        pruneUnusedTops(*b_h, g, rrm[b_left_id]);
        assert(a_left == b_left);
        return true;
    }

    /* if it is the same graph, it is also fairly easy */
    if (is_equal(*a_h, a_left.leftfix_report, *b_h, b_left.leftfix_report)) {
        if (rrm[a_left_id].size() == 1) {
            /* nobody else is using a_h */
            rrm[b_left_id].erase(b);
            rrm[a_left_id].insert(b);
            b_left.graph = a_h;
            b_left.leftfix_report = a_left.leftfix_report;
            pruneUnusedTops(*b_h, g, rrm[b_left_id]);
            DEBUG_PRINTF("OK -> only user of a_h\n");
            return true;
        }

        if (rrm[b_left_id].size() == 1) {
            /* nobody else is using b_h */
            rrm[a_left_id].erase(a);
            rrm[b_left_id].insert(a);
            a_left.graph = b_h;
            a_left.leftfix_report = b_left.leftfix_report;
            pruneUnusedTops(*a_h, g, rrm[a_left_id]);
            DEBUG_PRINTF("OK -> only user of b_h\n");
            return true;
        }

        if (preds_same) {
            /* preds are the same anyway in diamond/left merges just need to
             * check that all the literals in rrm[b_h] can handle a_h */
            for (auto v : rrm[b_left_id]) {
                if (!mergeableRoseVertices(tbi, a, v)) {
                    goto literal_mismatch_1;
                }
            }

            rrm[a_left_id].erase(a);
            rrm[b_left_id].insert(a);
            a_left.graph = b_h;
            a_left.leftfix_report = b_left.leftfix_report;
            pruneUnusedTops(*a_h, g, rrm[a_left_id]);
            DEBUG_PRINTF("OK -> same preds ???\n");
            return true;
        literal_mismatch_1:
            /* preds are the same anyway in diamond/left merges just need to
             * check that all the literals in rrm[a_h] can handle b_h */
            for (auto v : rrm[a_left_id]) {
                if (!mergeableRoseVertices(tbi, v, b)) {
                    goto literal_mismatch_2;
                }
            }

            rrm[b_left_id].erase(b);
            rrm[a_left_id].insert(b);
            b_left.graph = a_h;
            b_left.leftfix_report = a_left.leftfix_report;
            pruneUnusedTops(*b_h, g, rrm[b_left_id]);
            DEBUG_PRINTF("OK -> same preds ???\n");
            return true;
        literal_mismatch_2:;
        }
        DEBUG_PRINTF("OK -> create new\n");
        /* we need to create a new graph as there may be other people
         * using b_left and it would be bad if a's preds started triggering it
         */
        ReportID new_report = tbi.getNewNfaReport();
        shared_ptr<NGHolder> new_graph = cloneHolder(*b_h);
        duplicateReport(*new_graph, b_left.leftfix_report, new_report);
        pruneReportIfUnused(tbi, new_graph, set<NFAVertex>(),
                            b_left.leftfix_report);

        rrm[a_left_id].erase(a);
        rrm[b_left_id].erase(b);
        pruneUnusedTops(*a_h, g, rrm[a_left_id]);
        pruneUnusedTops(*b_h, g, rrm[b_left_id]);

        a_left.leftfix_report = new_report;
        b_left.leftfix_report = new_report;
        a_left.graph = new_graph;
        b_left.graph = new_graph;

        rrm[a_left].insert(a);
        rrm[a_left].insert(b);
        pruneUnusedTops(*new_graph, g, rrm[a_left]);
        return true;
    }

    // Everything after this point requires merging via the uncalc code, so we
    // guard it with the trivial cases arg.
    if (trivialCasesOnly) {
        return false;
    }

    // Only infixes. Prefixes require special care when doing non-trivial
    // merges.
    if (!tbi.isNonRootSuccessor(a) || !tbi.isNonRootSuccessor(b)) {
        return false;
    }

    DEBUG_PRINTF("attempting merge of roses on vertices %zu and %zu\n",
                 g[a].idx, g[b].idx);

    set<RoseVertex> &b_verts = rrm[b_left];
    set<RoseVertex> aa;
    aa.insert(a);

    if (!mergeableRoseVertices(tbi, aa, b_verts)) {
        DEBUG_PRINTF("vertices not mergeable\n");
        return false;
    }

    if (!tbi.cc.grey.roseMultiTopRoses) {
        return false;
    }

    // Clone a copy of a's NFA to operate on, and store a copy of its in-edge
    // properties.

    /* We need to allocate a new report id because */
    ReportID a_oldreport = a_left.leftfix_report;
    ReportID b_oldreport = b_left.leftfix_report;
    ReportID new_report = tbi.getNewNfaReport();
    duplicateReport(*b_h, b_left.leftfix_report, new_report);
    b_left.leftfix_report = new_report;
    pruneReportIfUnused(tbi, b_h, rrm[b_left_id], b_oldreport);

    NGHolder victim;
    cloneHolder(victim, *a_h);
    duplicateReport(victim, a_left.leftfix_report, new_report);
    pruneAllOtherReports(victim, new_report);

    map<RoseVertex, RoseEdgeProps> a_props;
    for (const auto &e : in_edges_range(a, g)) {
        a_props[source(e, g)] = g[e];
    }

    DEBUG_PRINTF("victim %zu states\n", num_vertices(*a_h));
    DEBUG_PRINTF("winner %zu states\n", num_vertices(*b_h));

    if (!setDistinctRoseTops(g, victim, *b_h, deque<RoseVertex>(1, a))) {
        assert(roseHasTops(g, a));
        assert(roseHasTops(g, b));
        return false;
    }

    assert(victim.kind == b_h->kind);
    assert(!generates_callbacks(*b_h));

    if (!mergeNfaPair(victim, *b_h, nullptr, tbi.cc)) {
        DEBUG_PRINTF("merge failed\n");
        // Restore in-edge properties.
        for (const auto &e : in_edges_range(a, g)) {
            g[e] = a_props[source(e, g)];
        }
        assert(roseHasTops(g, a));
        assert(roseHasTops(g, b));
        return false;
    }

    DEBUG_PRINTF("merge succeeded -> %zu vertices\n", num_vertices(*b_h));

    // update A's rose data to point to the merged graph.
    a_left.graph = b_h;
    a_left.leftfix_report = new_report;

    assert(contains(rrm[a_left_id], a));
    assert(contains(rrm[b_left_id], b));
    rrm[a_left_id].erase(a);
    rrm[b_left_id].insert(a);

    pruneUnusedTops(*a_h, g, rrm[a_left_id]);
    pruneUnusedTops(*b_h, g, rrm[b_left_id]);

    // Prune A's report from its old prefix if it was only used by A.
    pruneReportIfUnused(tbi, a_h, rrm[a_left_id], a_oldreport);

    reduceImplementableGraph(*b_h, SOM_NONE, nullptr, tbi.cc);

    assert(roseHasTops(g, a));
    assert(roseHasTops(g, b));
    assert(isImplementableNFA(*b_h, nullptr, tbi.cc));
    return true;
}

// Called by the role aliasing pass: Attempt to merge rose a into b, updating
// the two LeftEngInfo structures to be the same. Returns false if the merge
// is not possible.
static
bool attemptRoseMerge(RoseBuildImpl &tbi, bool preds_same, RoseVertex a,
                      RoseVertex b, bool trivialCasesOnly, revRoseMap &rrm) {
    DEBUG_PRINTF("attempting rose merge, vertices a=%zu, b=%zu\n",
                  tbi.g[a].idx, tbi.g[b].idx);
    assert(a != b);

    RoseGraph &g = tbi.g;
    LeftEngInfo &a_left = g[a].left;
    LeftEngInfo &b_left = g[b].left;

    // Trivial case.
    if (a_left == b_left) {
        DEBUG_PRINTF("roses are identical, no leftfix or already merged\n");
        return true;
    }

    const left_id a_left_id(a_left);
    const left_id b_left_id(b_left);

    /* Haig merges not supported at the moment */
    if (a_left.haig || b_left.haig) {
        return false;
    }

    /* dfa merges not supported at the moment (no multitop) */
    if (a_left.dfa || b_left.dfa) {
        return false;
    }

    // Only non-transients for the moment.
    if (contains(tbi.transient, a_left_id) ||
        contains(tbi.transient, b_left_id)) {
        return false;
    }

    /* It is not possible to merge roles with different lags as we can only
     * test the leftfix at one location relative to the literal match */
    if (a_left.lag != b_left.lag) {
        return false;
    }

    assert(roseHasTops(g, a));
    assert(roseHasTops(g, b));

    if (a_left_id.graph() && b_left_id.graph()) {
        return attemptRoseGraphMerge(tbi, preds_same, a, b, trivialCasesOnly,
                                     rrm);
    }

    if (a_left_id.castle() && b_left_id.castle()) {
        return attemptRoseCastleMerge(tbi, preds_same, a, b, trivialCasesOnly,
                                      rrm);
    }

    return false;
}

static
void splitByReportSuffixBehaviour(const RoseGraph &g,
                                  vector<vector<RoseVertex>> &buckets,
                                  ue2::unordered_map<RoseVertex, size_t> &inv) {
    /* vertices with different report/suffixes can never be considered for right
     * merge. */
    vector<vector<RoseVertex>> out;
    for (const vector<RoseVertex> &b : buckets) {
        assert(!b.empty());
        map<pair<flat_set<ReportID>, RoseSuffixInfo>, size_t> dest_map;
        for (RoseVertex v : b) {
            auto key = decltype(dest_map)::key_type(g[v].reports, g[v].suffix);
            size_t out_bucket;
            if (contains(dest_map, key)) {
                out_bucket = dest_map[key];
            } else {
                out_bucket = out.size();
                out.push_back(vector<RoseVertex>());
                dest_map[key] = out_bucket;
            }
            out[out_bucket].push_back(v);
            inv[v] = out_bucket;
        }

    }

    buckets.swap(out);
}

static
void splitByLiteralTable(const RoseBuildImpl &build,
                         vector<vector<RoseVertex>> &buckets,
                         ue2::unordered_map<RoseVertex, size_t> &inv) {
    const RoseGraph &g = build.g;

    vector<vector<RoseVertex>> out;

    for (const auto &bucket : buckets) {
        assert(!bucket.empty());
        map<rose_literal_table, size_t> dest_map;
        for (RoseVertex v : bucket) {
            auto table = build.literals.right.at(*g[v].literals.begin()).table;
            size_t out_bucket;
            if (contains(dest_map, table)) {
                out_bucket = dest_map[table];
            } else {
                out_bucket = out.size();
                out.push_back(vector<RoseVertex>());
                dest_map[table] = out_bucket;
            }
            out[out_bucket].push_back(v);
            inv[v] = out_bucket;
        }
    }

    buckets.swap(out);
}

static
void splitByNeighbour(const RoseGraph &g, vector<vector<RoseVertex>> &buckets,
                      ue2::unordered_map<RoseVertex, size_t> &inv, bool succ) {
    vector<vector<RoseVertex>> extras;
    map<size_t, vector<RoseVertex>> neighbours_by_bucket;
    set<RoseVertex> picked;
    vector<RoseVertex> leftovers;

    for (RoseVertex u : vertices_range(g)) {
        /* once split by v, stays split. also keeps iterator in buckets valid */
        extras.clear();
        neighbours_by_bucket.clear();
        if (succ) {
            /* forward pass */
            for (RoseVertex v : adjacent_vertices_range(u, g)) {
                auto it = inv.find(v);
                if (it != end(inv)) {
                    neighbours_by_bucket[it->second].push_back(v);
                }
            }
        } else {
            /* backward pass */
            for (RoseVertex v : inv_adjacent_vertices_range(u, g)) {
                auto it = inv.find(v);
                if (it != end(inv)) {
                    neighbours_by_bucket[it->second].push_back(v);
                }
            }
        }
        for (const auto &e : neighbours_by_bucket) {
            size_t old_key = e.first;
            if (buckets[old_key].size() == e.second.size()) {
                /* did not split */
                continue;
            }
            assert(!e.second.empty());

            picked.clear();
            picked.insert(begin(e.second), end(e.second));

            size_t new_key = buckets.size() + extras.size();
            leftovers.clear();
            for (RoseVertex v : buckets[old_key]) {
                if (contains(picked, v)) {
                    inv[v] = new_key;
                } else {
                    leftovers.push_back(v);
                }
            }

            assert(!leftovers.empty());
            assert(e.second.size() + leftovers.size()
                   == buckets[old_key].size());
            extras.push_back(e.second);
            buckets[old_key].swap(leftovers);
        }
        insert(&buckets, buckets.end(), extras);
    }
}

static
vector<vector<RoseVertex>> splitDiamondMergeBuckets(CandidateSet &candidates,
                                                    const RoseBuildImpl &build) {
    const RoseGraph &g = build.g;

    vector<vector<RoseVertex>> buckets(1);
    ue2::unordered_map<RoseVertex, size_t> inv;
    for (RoseVertex v : candidates) {
        buckets[0].push_back(v);
        inv[v] = 0;
    }

    splitByReportSuffixBehaviour(g, buckets, inv);
    splitByLiteralTable(build, buckets, inv);
    splitByNeighbour(g, buckets, inv, true);
    splitByNeighbour(g, buckets, inv, false);

    return buckets;
}
static never_inline
void diamondMergePass(CandidateSet &candidates, RoseBuildImpl &tbi,
                      vector<RoseVertex> *dead, bool mergeRoses,
                      revRoseMap &rrm) {
    DEBUG_PRINTF("begin\n");
    RoseGraph &g = tbi.g;

    if (candidates.empty()) {
        return;
    }

    /* Vertices may only be diamond merged with others in the same bucket */
    auto cand_buckets = splitDiamondMergeBuckets(candidates, tbi);

    for (const vector<RoseVertex> &siblings : cand_buckets) {
        for (auto it = siblings.begin(); it != siblings.end();) {
            RoseVertex a = *it;
            ++it;

            assert(contains(candidates, a));

            DEBUG_PRINTF("trying to merge %zu into somebody\n", g[a].idx);
            for (auto jt = it; jt != siblings.end(); ++jt) {
                RoseVertex b = *jt;
                assert(contains(candidates, b));

                if (!sameRoleProperties(tbi, a, b)) {
                    DEBUG_PRINTF("diff role prop\n");
                    continue;
                }

                // Check "diamond" requirements: must have same right side
                // (successors, reports) and left side (predecessors).
                /* Note: bucketing does not check edge properties (bounds, tops)
                 * so we still have to checks successors and predecessors. */

                if (!sameSuccessors(a, b, g)
                    || !sameRightRoleProperties(tbi, a, b)
                    || !samePredecessors(a, b, g)) {
                    DEBUG_PRINTF("not diamond\n");
                    continue;
                }

                if (!canMergeLiterals(a, b, tbi)) {
                    DEBUG_PRINTF("incompatible lits\n");
                    continue;
                }

                if (!attemptRoseMerge(tbi, true, a, b, !mergeRoses, rrm)) {
                    DEBUG_PRINTF("rose fail\n");
                    continue;
                }

                mergeVerticesDiamond(a, b, tbi, rrm);
                dead->push_back(a);
                candidates.erase(a);
                break; // next a
            }
        }
    }

    DEBUG_PRINTF("%zu candidates remaining\n", candidates.size());
}

static
vector<RoseVertex>::iterator findLeftMergeSibling(
                          vector<RoseVertex>::iterator it,
                          const vector<RoseVertex>::iterator &end,
                          const RoseVertex a, const RoseBuildImpl &build,
                          const CandidateSet &candidates) {
    const RoseGraph &g = build.g;

    for (; it != end; ++it) {
        RoseVertex b = *it;
        if (a == b) {
            continue;
        }

        if (!contains(candidates, b)) {
            continue;
        }

        if (!sameRoleProperties(build, a, b)) {
            continue;
        }

        // Check left-equivalence: must have same predecessors and same
        // literals.

        if (g[a].literals != g[b].literals) {
            continue;
        }

        if (!samePredecessors(a, b, g)) {
            continue;
        }

        if (hasCommonSuccWithBadBounds(a, b, g)) {
            continue;
        }

        if (g[a].suffix && g[b].suffix && g[a].suffix != g[b].suffix) {
            continue; /* we can only trigger one suffix */
        }

        return it;
    }

    return end;
}

static never_inline
void leftMergePass(CandidateSet &candidates, RoseBuildImpl &tbi,
                   vector<RoseVertex> *dead, revRoseMap &rrm) {
    DEBUG_PRINTF("begin (%zu)\n", candidates.size());
    RoseGraph &g = tbi.g;
    vector<RoseVertex> siblings;

    CandidateSet::iterator it = candidates.begin();
    while (it != candidates.end()) {
        RoseVertex a = *it;
        CandidateSet::iterator ait = it;
        ++it;

        // We have to find a sibling to merge `a' with, and we select between
        // two approaches to minimize the number of vertices we have to
        // examine; which we use depends on the shape of the graph.

        assert(!g[a].literals.empty());
        u32 lit_id = *g[a].literals.begin();
        const auto &verts = tbi.literal_info.at(lit_id).vertices;
        RoseVertex pred = pickPred(a, g, tbi);

        siblings.clear();
        if (pred == RoseGraph::null_vertex() || tbi.isAnyStart(pred) ||
                    hasGreaterOutDegree(verts.size(), pred, g)) {
            // Select sibling from amongst the vertices that share a literal.
            siblings.insert(siblings.end(), verts.begin(), verts.end());
        } else {
            // Select sibling from amongst the vertices that share a
            // predecessor.
            insert(&siblings, siblings.end(), adjacent_vertices(pred, g));
        }

        sort(siblings.begin(), siblings.end(), VertexIndexComp(g));

        auto jt = findLeftMergeSibling(siblings.begin(), siblings.end(), a, tbi,
                                       candidates);
        if (jt == siblings.end()) {
            continue;
        }

        RoseVertex b = *jt;

        if (!attemptRoseMerge(tbi, true, a, b, 0, rrm)) {
            DEBUG_PRINTF("rose fail\n");
            continue;
        }

        mergeVertices(a, b, tbi, rrm);
        dead->push_back(a);
        candidates.erase(ait);
    }

    DEBUG_PRINTF("%zu candidates remaining\n", candidates.size());
}

// Can't merge vertices with different root predecessors.
static
bool safeRootPreds(RoseVertex a, RoseVertex b, const RoseGraph &g) {
    set<RoseVertex> a_roots, b_roots;

    for (auto u : inv_adjacent_vertices_range(a, g)) {
        if (!hasGreaterInDegree(0, u, g)) {
            a_roots.insert(u);
        }
    }
    for (auto u : inv_adjacent_vertices_range(b, g)) {
        if (!hasGreaterInDegree(0, u, g)) {
            b_roots.insert(u);
        }
    }

    assert(a_roots.size() <= 1);
    assert(b_roots.size() <= 1);

    return a_roots == b_roots;
}

static never_inline
vector<RoseVertex>::const_iterator findRightMergeSibling(
                           vector<RoseVertex>::const_iterator it,
                           const vector<RoseVertex>::const_iterator &end,
                           const RoseVertex a, const RoseBuildImpl &build,
                           const CandidateSet &candidates) {
    const RoseGraph &g = build.g;

    for (; it != end; ++it) {
        RoseVertex b = *it;
        if (a == b) {
            continue;
        }

        if (!contains(candidates, b)) {
            continue;
        }

        if (!sameRoleProperties(build, a, b)) {
            continue;
        }

        // Check right-equivalence: must have same successors, reports and same
        // literals.

        if (g[a].literals != g[b].literals) {
            continue;
        }

        if (!sameSuccessors(a, b, g)
            || !sameRightRoleProperties(build, a, b)) {
            continue;
        }

        // An extra wrinkle: we cannot merge two vertices that are root
        // successors if their preds are different. (e.g. one is anchored and
        // one is not)
        if (!safeRootPreds(a, b, g)) {
            continue;
        }

        if (hasCommonPredWithBadBounds(a, b, g)) {
            continue;
        }

        if (hasCommonPredWithDiffRoses(a, b, g)) {
            continue;
        }

        return it;
    }

    return end;
}

template<class Iter>
static
void split(map<RoseVertex, size_t> &keys, size_t *next_key, Iter it,
           const Iter end) {
    map<size_t, size_t> new_keys;

    for (; it != end; ++it) {
        RoseVertex v = *it;
        size_t ok = keys[v];
        size_t nk;
        if (contains(new_keys, ok)) {
            nk = new_keys[ok];
        } else {
            nk = (*next_key)++;
            new_keys[ok] = nk;
        }
        keys[v] = nk;
    }
}

static never_inline
void buildCandidateRightSiblings(CandidateSet &candidates, RoseBuildImpl &tbi,
                                 map<size_t, vector<RoseVertex> > &sibling_cache,
                                 map<RoseVertex, size_t> &keys_ext) {
    RoseGraph &g = tbi.g;

    size_t next_key = 1;
    map<RoseVertex, size_t> keys;

    for (const auto &c : candidates) {
        keys[c] = 0;
    }

    set<RoseVertex> done_succ;
    set<u32> done_lit;

    for (auto a : candidates) {
        assert(!g[a].literals.empty());
        u32 lit_id = *g[a].literals.begin();
        RoseVertex succ = pickSucc(a, g);
        const auto &verts = tbi.literal_info.at(lit_id).vertices;
        if (succ != RoseGraph::null_vertex() &&
                !hasGreaterInDegree(verts.size(), succ, g)) {
            if (!done_succ.insert(succ).second) {
                continue; // succ already in done_succ.
            }
            RoseGraph::inv_adjacency_iterator ai, ae;
            tie (ai, ae) = inv_adjacent_vertices(succ, g);
            split(keys, &next_key, ai, ae);
        } else {
            if (!done_lit.insert(lit_id).second) {
                continue; // lit_id already in done_lit.
            }
            split(keys, &next_key, verts.begin(), verts.end());
        }
    }

    map<size_t, map<size_t, size_t>> int_to_ext;

    for (const auto &key : keys) {
        RoseVertex v = key.first;
        u32 ext;
        size_t rph = hashRightRoleProperties(v, g);
        if (contains(int_to_ext[key.second], rph)) {
            ext = int_to_ext[key.second][rph];
        } else {
            ext = keys_ext.size();
            int_to_ext[key.second][rph] = ext;
        }

        keys_ext[v] = ext;
        sibling_cache[ext].push_back(v);
    }

    for (auto &siblings : sibling_cache | map_values) {
        sort(siblings.begin(), siblings.end(), VertexIndexComp(tbi.g));
    }
}

static
const vector<RoseVertex> &getCandidateRightSiblings(
                         const map<size_t, vector<RoseVertex> > &sibling_cache,
                         map<RoseVertex, size_t> &keys, RoseVertex a) {
    size_t key = keys.at(a);
    return sibling_cache.at(key);
}

static never_inline
void rightMergePass(CandidateSet &candidates, RoseBuildImpl &tbi,
                    vector<RoseVertex> *dead, bool mergeRoses,
                    revRoseMap &rrm) {
    DEBUG_PRINTF("begin\n");

    map<size_t, vector<RoseVertex> > sibling_cache;
    map<RoseVertex, size_t> keys;

    buildCandidateRightSiblings(candidates, tbi, sibling_cache, keys);

    CandidateSet::iterator it = candidates.begin();
    while (it != candidates.end()) {
        RoseVertex a = *it;
        CandidateSet::iterator ait = it;
        ++it;

        // We have to find a sibling to merge `a' with, and we select between
        // two approaches to minimize the number of vertices we have to
        // examine; which we use depends on the shape of the graph.

        const vector<RoseVertex> &siblings
            = getCandidateRightSiblings(sibling_cache, keys, a);

        auto jt = siblings.begin();
        while (jt != siblings.end()) {
            jt = findRightMergeSibling(jt, siblings.end(), a, tbi, candidates);
            if (jt == siblings.end()) {
                break;
            }
            if (attemptRoseMerge(tbi, false, a, *jt, !mergeRoses, rrm)) {
                break;
            }
            ++jt;
        }

        if (jt == siblings.end()) {
            continue;
        }

        RoseVertex b = *jt;
        mergeVertices(a, b, tbi, rrm);
        dead->push_back(a);
        candidates.erase(ait);
    }

    DEBUG_PRINTF("%zu candidates remaining\n", candidates.size());
}

/**
 * \brief True if the given vertex has no siblings for the purposes of a
 * diamond merge.
 *
 * This is the case if it has no successors with more than one predecessor
 * (itself), or no predecessors with more than one successor (itself).
 */
static
bool hasNoDiamondSiblings(const RoseGraph &g, RoseVertex v) {
    if (has_successor(v, g)) {
        bool only_succ = true;
        for (const auto &w : adjacent_vertices_range(v, g)) {
            if (hasGreaterInDegree(1, w, g)) {
                only_succ = false;
                break;
            }
        }
        if (only_succ) {
            return true;
        }
    }

    // Any candidate vertex will have a predecessor; the only vertices without
    // preds are the root vertices.
    assert(in_edges(v, g).first != in_edges(v, g).second);

    bool only_pred = true;
    for (const auto &u : inv_adjacent_vertices_range(v, g)) {
        if (hasGreaterOutDegree(1, u, g)) {
            only_pred = false;
            break;
        }
    }

    return only_pred;
}

/**
 * \brief Filter out some merge candidates that are not mergeable by a diamond
 * merge.
 */
static
void filterDiamondCandidates(RoseGraph &g, CandidateSet &candidates) {
    DEBUG_PRINTF("%zu candidates enter\n", candidates.size());

    vector<RoseVertex> dead;
    for (const auto &v : candidates) {
        if (hasNoDiamondSiblings(g, v)) {
            dead.push_back(v);
        }
    }

    for (const auto &v : dead) {
        candidates.erase(v);
    }

    DEBUG_PRINTF("pruned %zu candidates, leaving %zu\n", dead.size(),
                 candidates.size());
}

void aliasRoles(RoseBuildImpl &build, bool mergeRoses) {
    const CompileContext &cc = build.cc;
    RoseGraph &g = build.g;

    if (!cc.grey.roseRoleAliasing || !cc.grey.roseGraphReduction) {
        return;
    }

    revRoseMap rrm;

    DEBUG_PRINTF("doing role aliasing mr=%d\n", (int)mergeRoses);
    populateRevRoseMap(g, &rrm);

    mergeRoses &= cc.grey.mergeRose & cc.grey.roseMergeRosesDuringAliasing;

    CandidateSet candidates(g);
    findCandidates(build, &candidates);

    DEBUG_PRINTF("candidates %zu\n", candidates.size());

    vector<RoseVertex> dead;
    size_t old_dead_size = 0;
    do {
        old_dead_size = dead.size();
        leftMergePass(candidates, build, &dead, rrm);
        rightMergePass(candidates, build, &dead, mergeRoses, rrm);
    } while (old_dead_size != dead.size());

    /* Diamond merge passes cannot create extra merges as they require the same
     * succ and preds before merging --> that if a succ/pred was ineligible due
     * to a merge to different pred/succ before a diamond merge, it will still
     * be afterwards. */
    filterDiamondCandidates(g, candidates);
    diamondMergePass(candidates, build, &dead, mergeRoses, rrm);

    DEBUG_PRINTF("killed %zu vertices\n", dead.size());
    build.removeVertices(dead);
}

} // namespace ue2
