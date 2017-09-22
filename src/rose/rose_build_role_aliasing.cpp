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
#include "nfagraph/ng_uncalc_components.h"
#include "nfagraph/ng_util.h"
#include "util/bitutils.h"
#include "util/compile_context.h"
#include "util/container.h"
#include "util/flat_containers.h"
#include "util/graph.h"
#include "util/graph_range.h"
#include "util/hash.h"
#include "util/order_check.h"

#include <algorithm>
#include <numeric>
#include <vector>
#include <boost/graph/adjacency_iterator.hpp>
#include <boost/range/adaptor/map.hpp>

using namespace std;
using boost::adaptors::map_values;

namespace ue2 {

static constexpr size_t MERGE_GROUP_SIZE_MAX = 200;

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
    using key_type = RoseVertex;
    using iterator = set<RoseVertex>::iterator;
    using const_iterator = set<RoseVertex>::const_iterator;

    iterator begin() { return main_cont.begin(); }
    iterator end() { return main_cont.end(); }
    const_iterator begin() const { return main_cont.begin(); }
    const_iterator end() const { return main_cont.end(); }

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
    set<RoseVertex> main_cont; /* deterministic iterator */
    unordered_set<RoseVertex> hash_cont; /* member checks */
};

struct RoseAliasingInfo {
    RoseAliasingInfo(const RoseBuildImpl &build) {
        const auto &g = build.g;

        // Populate reverse leftfix map.
        for (auto v : vertices_range(g)) {
            if (g[v].left) {
                rev_leftfix[g[v].left].insert(v);
            }
        }

        // Populate reverse ghost vertex map.
        for (const auto &m : build.ghost) {
            rev_ghost[m.second].insert(m.first);
        }
    }

    /** \brief Mapping from leftfix to vertices. */
    unordered_map<left_id, set<RoseVertex>> rev_leftfix;

    /** \brief Mapping from undelayed ghost to delayed vertices. */
    unordered_map<RoseVertex, set<RoseVertex>> rev_ghost;
};

} // namespace

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
            RoseEdge e = edge(source(e_a, g), b, g);
            if (!e || g[e].rose_top != g[e_a].rose_top) {
                DEBUG_PRINTF("bad tops\n");
                return false;
            }
        }
    }

    return true;
}

static
bool hasCommonSuccWithBadBounds(RoseVertex a, RoseVertex b,
                                const RoseGraph &g) {
    for (const auto &e_a : out_edges_range(a, g)) {
        if (RoseEdge e = edge(b, target(e_a, g), g)) {
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
bool hasCommonPredWithBadBounds(RoseVertex a, RoseVertex b,
                                const RoseGraph &g) {
    for (const auto &e_a : in_edges_range(a, g)) {
        if (RoseEdge e = edge(source(e_a, g), b, g)) {
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
bool canMergeLiterals(RoseVertex a, RoseVertex b, const RoseBuildImpl &build) {
    const auto &lits_a = build.g[a].literals;
    const auto &lits_b = build.g[b].literals;
    assert(!lits_a.empty() && !lits_b.empty());

    // If both vertices have only pseudo-dotstar in-edges, we can merge
    // literals of different lengths and can avoid the check below.
    if (build.hasOnlyPseudoStarInEdges(a) &&
        build.hasOnlyPseudoStarInEdges(b)) {
        DEBUG_PRINTF("both have pseudo-dotstar in-edges\n");
        return true;
    }

    // Otherwise, all the literals involved must have the same length.
    for (u32 a_id : lits_a) {
        const rose_literal_id &la = build.literals.at(a_id);
        for (u32 b_id : lits_b) {
            const rose_literal_id &lb = build.literals.at(b_id);

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
bool isAliasingCandidate(RoseVertex v, const RoseBuildImpl &build) {
    const RoseVertexProps &props = build.g[v];

    // Must have literals.
    if (props.literals.empty()) {
        return false;
    }

    assert(*props.literals.begin() != MO_INVALID_IDX);
    return true;
}

static
bool sameGhostProperties(const RoseBuildImpl &build,
                         const RoseAliasingInfo &rai, RoseVertex a,
                         RoseVertex b) {
    // If these are ghost mapping keys, then they must map to the same vertex.
    if (contains(build.ghost, a) || contains(build.ghost, b)) {
        DEBUG_PRINTF("checking ghost key compat\n");
        if (!contains(build.ghost, a) || !contains(build.ghost, b)) {
            DEBUG_PRINTF("missing ghost mapping\n");
            return false;
        }
        if (build.ghost.at(a) != build.ghost.at(b)) {
            DEBUG_PRINTF("diff ghost mapping\n");
            return false;
        }
        DEBUG_PRINTF("ghost mappings ok\n");
        return true;
    }

    // If they are ghost vertices, then they must have the same literals.
    if (contains(rai.rev_ghost, a) || contains(rai.rev_ghost, b)) {
        if (!contains(rai.rev_ghost, a) || !contains(rai.rev_ghost, b)) {
            DEBUG_PRINTF("missing ghost reverse mapping\n");
            return false;
        }
        return build.g[a].literals == build.g[b].literals;
    }

    return true;
}

static
bool sameRoleProperties(const RoseBuildImpl &build, const RoseAliasingInfo &rai,
                        RoseVertex a, RoseVertex b) {
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

    if (!sameGhostProperties(build, rai, a, b)) {
        return false;
    }

    /* "roses are mergeable" check are handled elsewhere  */

    return true;
}

/* Checks compatibility of role properties if we require that two roles are
 * right equiv. */
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

static
void mergeEdgeAdd(RoseVertex u, RoseVertex v, const RoseEdge &from_edge,
                  const RoseEdge *to_edge, RoseGraph &g) {
    const RoseEdgeProps &from_props = g[from_edge];

    if (!to_edge) {
        DEBUG_PRINTF("adding edge [%zu,%zu]\n", g[u].index, g[v].index);
        add_edge(u, v, from_props, g);
    } else {
        // union of the two edges.
        DEBUG_PRINTF("updating edge [%zu,%zu]\n", g[u].index, g[v].index);
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
        b_edges.emplace(u, e);
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
        b_edges.emplace(v, e);
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
void mergeLiteralSets(RoseVertex a, RoseVertex b, RoseBuildImpl &build) {
    RoseGraph &g = build.g;
    const auto &a_literals = g[a].literals;
    for (u32 lit_id : a_literals) {
        auto &lit_vertices = build.literal_info[lit_id].vertices;
        lit_vertices.erase(a);
        lit_vertices.insert(b);
    }

    insert(&g[b].literals, a_literals);
}

static
void updateAliasingInfo(RoseBuildImpl &build, RoseAliasingInfo &rai,
                        RoseVertex a, RoseVertex b) {
    if (build.g[a].left) {
        const left_id left(build.g[a].left);
        assert(contains(rai.rev_leftfix[left], a));
        rai.rev_leftfix[left].erase(a);
    }
    if (contains(build.ghost, a)) {
        auto ghost = build.ghost.at(a);
        assert(contains(build.ghost, b) && ghost == build.ghost.at(b));
        build.ghost.erase(a);
        rai.rev_ghost[ghost].erase(a);
    }

    if (contains(rai.rev_ghost, a)) {
        for (const auto &v : rai.rev_ghost[a]) {
            build.ghost[v] = b;
            rai.rev_ghost[b].insert(v);
        }
        rai.rev_ghost.erase(a);
    }
}

/** \brief Common role merge code used by variants below. */
static
void mergeCommon(RoseBuildImpl &build, RoseAliasingInfo &rai, RoseVertex a,
                 RoseVertex b) {
    RoseGraph &g = build.g;

    assert(g[a].eod_accept == g[b].eod_accept);
    assert(g[a].left == g[b].left);
    assert(!g[a].suffix || g[a].suffix == g[b].suffix);

    // In some situations (ghost roles etc), we can have different groups.
    assert(!g[a].groups && !g[b].groups); /* current structure means groups
                                           * haven't been assigned yet */
    g[b].groups |= g[a].groups;

    mergeLiteralSets(a, b, build);
    updateAliasingInfo(build, rai, a, b);

    // Our min and max_offsets should be sane.
    assert(g[b].min_offset <= g[b].max_offset);

    // Safety check: we should not have created through a merge a vertex that
    // has an out-edge with ANCH history but is not fixed-offset.
    assert(!hasAnchHistorySucc(g, b) || g[b].fixedOffset());
}

/** \brief Merge role 'a' into 'b', left merge path. */
static
void mergeVerticesLeft(RoseVertex a, RoseVertex b, RoseBuildImpl &build,
                       RoseAliasingInfo &rai) {
    RoseGraph &g = build.g;
    DEBUG_PRINTF("merging vertex %zu into %zu\n", g[a].index, g[b].index);

    insert(&g[b].reports, g[a].reports);

    // Since it is a left merge (identical LHS) we should pick the tighter
    // bound.
    g[b].min_offset = max(g[a].min_offset, g[b].min_offset);
    g[b].max_offset = min(g[a].max_offset, g[b].max_offset);

    if (!g[b].suffix) {
        g[b].suffix = g[a].suffix;
    }

    mergeEdges(a, b, g);
    mergeCommon(build, rai, a, b);
}

/** \brief Merge role 'a' into 'b', right merge path. */
static
void mergeVerticesRight(RoseVertex a, RoseVertex b, RoseBuildImpl &build,
                        RoseAliasingInfo &rai) {
    RoseGraph &g = build.g;
    DEBUG_PRINTF("merging vertex %zu into %zu\n", g[a].index, g[b].index);

    insert(&g[b].reports, g[a].reports);
    g[b].min_offset = min(g[a].min_offset, g[b].min_offset);
    g[b].max_offset = max(g[a].max_offset, g[b].max_offset);

    mergeEdges(a, b, g);
    mergeCommon(build, rai, a, b);
}

/**
 * Faster version of \ref mergeVertices for diamond merges, for which we know
 * that the in- and out-edge sets, reports and suffixes are identical.
 */
static
void mergeVerticesDiamond(RoseVertex a, RoseVertex b, RoseBuildImpl &build,
                          RoseAliasingInfo &rai) {
    RoseGraph &g = build.g;
    DEBUG_PRINTF("merging vertex %zu into %zu\n", g[a].index, g[b].index);

    // For a diamond merge, most properties are already the same (with the
    // notable exception of the literal set).
    assert(g[a].reports == g[b].reports);
    assert(g[a].suffix == g[b].suffix);

    g[b].min_offset = min(g[a].min_offset, g[b].min_offset);
    g[b].max_offset = max(g[a].max_offset, g[b].max_offset);

    mergeCommon(build, rai, a, b);
}

static never_inline
void findCandidates(const RoseBuildImpl &build, CandidateSet *candidates) {
    for (auto v : vertices_range(build.g)) {
        if (isAliasingCandidate(v, build)) {
            DEBUG_PRINTF("candidate %zu\n", build.g[v].index);
            DEBUG_PRINTF("lits: %u\n", *build.g[v].literals.begin());
            candidates->insert(v);
        }
    }

    assert(candidates->size() <= num_vertices(build.g));
    DEBUG_PRINTF("found %zu/%zu candidates\n", candidates->size(),
                 num_vertices(build.g));
}

static
RoseVertex pickPred(const RoseVertex v, const RoseGraph &g,
                    const RoseBuildImpl &build) {
    RoseGraph::in_edge_iterator ei, ee;
    tie(ei, ee) = in_edges(v, g);
    if (ei == ee) {
        assert(0); // every candidate should have in-degree!
        return RoseGraph::null_vertex();
    }

    // Avoid roots if we have other options, since it doesn't matter to the
    // merge pass which predecessor we pick.
    RoseVertex u = source(*ei, g);
    while (build.isAnyStart(u) && ++ei != ee) {
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
        if (RoseEdge e = edge(source(e_a, g), b, g)) {
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
void pruneReportIfUnused(const RoseBuildImpl &build, shared_ptr<NGHolder> h,
                         const set<RoseVertex> &verts, ReportID report) {
    DEBUG_PRINTF("trying to prune %u from %p (v %zu)\n", report, h.get(),
                 verts.size());
    for (RoseVertex v : verts) {
        if (build.g[v].left.graph == h &&
            build.g[v].left.leftfix_report == report) {
            DEBUG_PRINTF("report %u still in use\n", report);
            return;
        }
    }

    if (!verts.empty()) {
        // Report no longer in use, but graph h is still alive: we should prune
        // the report if we can do so without rendering the graph
        // unimplementable.

        DEBUG_PRINTF("report %u has been merged away, pruning\n", report);
        assert(h->kind == (build.isRootSuccessor(*verts.begin()) ? NFA_PREFIX
                                                                 : NFA_INFIX));
        unique_ptr<NGHolder> h_new = cloneHolder(*h);
        pruneReport(*h_new, report);

        if (isImplementableNFA(*h_new, nullptr, build.cc)) {
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
    castle.report_map.clear();
    for (auto &e : castle.repeats) {
        u32 top = e.first;
        auto &repeat = e.second;
        repeat.reports.clear();
        repeat.reports.insert(report);
        castle.report_map[report].insert(top);
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
    unordered_set<u32> used_tops;
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
    if (!is_triggered(h)) {
        DEBUG_PRINTF("not triggered, no tops\n");
        return;
    }
    assert(isCorrectlyTopped(h));
    DEBUG_PRINTF("pruning unused tops\n");
    flat_set<u32> used_tops;
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
        flat_set<u32> pruned_tops;
        auto pt_inserter = inserter(pruned_tops, pruned_tops.end());
        set_intersection(h[e].tops.begin(), h[e].tops.end(),
                         used_tops.begin(), used_tops.end(), pt_inserter);
        h[e].tops = std::move(pruned_tops);
        if (h[e].tops.empty()) {
            DEBUG_PRINTF("edge (start,%zu) has only unused tops\n", h[v].index);
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
bool mergeSameCastle(RoseBuildImpl &build, RoseVertex a, RoseVertex b,
                     RoseAliasingInfo &rai) {
    RoseGraph &g = build.g;
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

    const ReportID new_report = build.getNewNfaReport();
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

    assert(contains(rai.rev_leftfix[b_left], b));
    rai.rev_leftfix[b_left].erase(b);
    rai.rev_leftfix[a_left].insert(b);

    a_left.leftfix_report = new_report;
    b_left.leftfix_report = new_report;
    assert(a_left == b_left);

    updateEdgeTops(g, a, a_top_map);
    updateEdgeTops(g, b, b_top_map);

    pruneUnusedTops(castle, g, rai.rev_leftfix[a_left]);
    return true;
}

static
bool attemptRoseCastleMerge(RoseBuildImpl &build, bool preds_same, RoseVertex a,
                            RoseVertex b, bool trivialCasesOnly,
                            RoseAliasingInfo &rai) {
    RoseGraph &g = build.g;
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
        return mergeSameCastle(build, a, b, rai);
    }

    if (is_equal(a_castle, a_left.leftfix_report, b_castle,
                 b_left.leftfix_report)) {
        DEBUG_PRINTF("castles are equiv with respect to reports\n");
        if (rai.rev_leftfix[a_left_id].size() == 1) {
            /* nobody else is using a_castle */
            rai.rev_leftfix[b_left_id].erase(b);
            rai.rev_leftfix[a_left_id].insert(b);
            pruneUnusedTops(b_castle, g, rai.rev_leftfix[b_left_id]);
            b_left.castle = a_left.castle;
            b_left.leftfix_report = a_left.leftfix_report;
            DEBUG_PRINTF("OK -> only user of a_castle\n");
            return true;
        }

        if (rai.rev_leftfix[b_left_id].size() == 1) {
            /* nobody else is using b_castle */
            rai.rev_leftfix[a_left_id].erase(a);
            rai.rev_leftfix[b_left_id].insert(a);
            pruneUnusedTops(a_castle, g, rai.rev_leftfix[a_left_id]);
            a_left.castle = b_left.castle;
            a_left.leftfix_report = b_left.leftfix_report;
            DEBUG_PRINTF("OK -> only user of b_castle\n");
            return true;
        }

        if (preds_same) {
            /* preds are the same anyway in diamond/left merges just need to
             * check that all the literals in rev_leftfix[b_h] can handle a_h */
            for (auto v : rai.rev_leftfix[b_left_id]) {
                if (!mergeableRoseVertices(build, a, v)) {
                    goto literal_mismatch_1;
                }
            }

            rai.rev_leftfix[a_left_id].erase(a);
            rai.rev_leftfix[b_left_id].insert(a);
            pruneUnusedTops(a_castle, g, rai.rev_leftfix[a_left_id]);
            a_left.castle = b_left.castle;
            a_left.leftfix_report = b_left.leftfix_report;
            DEBUG_PRINTF("OK -> same preds ???\n");
            return true;
        literal_mismatch_1:
            /* preds are the same anyway in diamond/left merges just need to
             * check that all the literals in rev_leftfix[a_h] can handle b_h */
            for (auto v : rai.rev_leftfix[a_left_id]) {
                if (!mergeableRoseVertices(build, v, b)) {
                    goto literal_mismatch_2;
                }
            }

            rai.rev_leftfix[b_left_id].erase(b);
            rai.rev_leftfix[a_left_id].insert(b);
            pruneUnusedTops(b_castle, g, rai.rev_leftfix[b_left_id]);
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
        ReportID new_report = build.getNewNfaReport();
        shared_ptr<CastleProto> new_castle = make_shared<CastleProto>(a_castle);
        pruneCastle(*new_castle, a_left.leftfix_report);
        setReports(*new_castle, new_report);

        rai.rev_leftfix[a_left_id].erase(a);
        rai.rev_leftfix[b_left_id].erase(b);
        pruneUnusedTops(*a_left.castle, g, rai.rev_leftfix[a_left_id]);
        pruneUnusedTops(*b_left.castle, g, rai.rev_leftfix[b_left_id]);

        a_left.leftfix_report = new_report;
        b_left.leftfix_report = new_report;
        a_left.castle = new_castle;
        b_left.castle = new_castle;

        assert(a_left == b_left);
        rai.rev_leftfix[a_left].insert(a);
        rai.rev_leftfix[a_left].insert(b);
        pruneUnusedTops(*new_castle, g, rai.rev_leftfix[a_left]);
        return true;
    }

    // Everything after this point requires more work, so we guard it with the
    // trivial cases argument..
    if (trivialCasesOnly) {
        return false;
    }

    // Only infixes. Prefixes require special care when doing non-trivial
    // merges.
    if (!build.isNonRootSuccessor(a) || !build.isNonRootSuccessor(b)) {
        return false;
    }

    set<RoseVertex> &b_verts = rai.rev_leftfix[b_left_id];
    set<RoseVertex> aa;
    aa.insert(a);

    if (!mergeableRoseVertices(build, aa, b_verts)) {
        DEBUG_PRINTF("vertices not mergeable\n");
        return false;
    }

    if (!build.cc.grey.roseMultiTopRoses || !build.cc.grey.allowCastle) {
        return false;
    }

    DEBUG_PRINTF("merging into new castle\n");

    // Clone new castle with a's repeats in it, set to a new report.
    ReportID new_report = build.getNewNfaReport();
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
        if (RoseEdge a_edge = edge(source(e, g), a, g)) {
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

    rai.rev_leftfix[a_left_id].erase(a);
    rai.rev_leftfix[b_left_id].erase(b);
    pruneUnusedTops(*a_left.castle, g, rai.rev_leftfix[a_left_id]);
    pruneUnusedTops(*b_left.castle, g, rai.rev_leftfix[b_left_id]);

    a_left.castle = m_castle;
    a_left.leftfix_report = new_report;
    b_left.castle = m_castle;
    b_left.leftfix_report = new_report;

    assert(a_left == b_left);
    rai.rev_leftfix[a_left].insert(a);
    rai.rev_leftfix[a_left].insert(b);
    pruneUnusedTops(*m_castle, g, rai.rev_leftfix[a_left]);
    return true;
}

static
bool attemptRoseGraphMerge(RoseBuildImpl &build, bool preds_same, RoseVertex a,
                           RoseVertex b, bool trivialCasesOnly,
                           RoseAliasingInfo &rai) {
    RoseGraph &g = build.g;
    LeftEngInfo &a_left = g[a].left;
    LeftEngInfo &b_left = g[b].left;
    left_id a_left_id(a_left);
    left_id b_left_id(b_left);
    shared_ptr<NGHolder> a_h = a_left.graph;
    shared_ptr<NGHolder> b_h = b_left.graph;
    assert(a_h && b_h);
    assert(isImplementableNFA(*a_h, nullptr, build.cc));
    assert(isImplementableNFA(*b_h, nullptr, build.cc));

    // If we only differ in reports, this is a very easy merge. Just use b's
    // report for both.
    /* Actually not so easy, there may be other poor suckers using a and/or b's
     * reports who will be surprised by this change */
    if (a_h == b_h) {
        DEBUG_PRINTF("OK -> same actual holder\n");
        ReportID a_oldreport = a_left.leftfix_report;
        ReportID b_oldreport = b_left.leftfix_report;
        ReportID new_report = build.getNewNfaReport();
        duplicateReport(*a_h, a_left.leftfix_report, new_report);
        duplicateReport(*b_h, b_left.leftfix_report, new_report);
        a_left.leftfix_report = new_report;
        b_left.leftfix_report = new_report;
        pruneReportIfUnused(build, b_h, rai.rev_leftfix[b_left_id],
                            a_oldreport);
        pruneReportIfUnused(build, b_h, rai.rev_leftfix[b_left_id],
                            b_oldreport);
        pruneUnusedTops(*b_h, g, rai.rev_leftfix[b_left_id]);
        assert(a_left == b_left);
        return true;
    }

    /* if it is the same graph, it is also fairly easy */
    if (is_equal(*a_h, a_left.leftfix_report, *b_h, b_left.leftfix_report)) {
        if (rai.rev_leftfix[a_left_id].size() == 1) {
            /* nobody else is using a_h */
            rai.rev_leftfix[b_left_id].erase(b);
            rai.rev_leftfix[a_left_id].insert(b);
            b_left.graph = a_h;
            b_left.leftfix_report = a_left.leftfix_report;
            pruneUnusedTops(*b_h, g, rai.rev_leftfix[b_left_id]);
            DEBUG_PRINTF("OK -> only user of a_h\n");
            return true;
        }

        if (rai.rev_leftfix[b_left_id].size() == 1) {
            /* nobody else is using b_h */
            rai.rev_leftfix[a_left_id].erase(a);
            rai.rev_leftfix[b_left_id].insert(a);
            a_left.graph = b_h;
            a_left.leftfix_report = b_left.leftfix_report;
            pruneUnusedTops(*a_h, g, rai.rev_leftfix[a_left_id]);
            DEBUG_PRINTF("OK -> only user of b_h\n");
            return true;
        }

        if (preds_same) {
            /* preds are the same anyway in diamond/left merges just need to
             * check that all the literals in rev_leftfix[b_h] can handle a_h */
            for (auto v : rai.rev_leftfix[b_left_id]) {
                if (!mergeableRoseVertices(build, a, v)) {
                    goto literal_mismatch_1;
                }
            }

            rai.rev_leftfix[a_left_id].erase(a);
            rai.rev_leftfix[b_left_id].insert(a);
            a_left.graph = b_h;
            a_left.leftfix_report = b_left.leftfix_report;
            pruneUnusedTops(*a_h, g, rai.rev_leftfix[a_left_id]);
            DEBUG_PRINTF("OK -> same preds ???\n");
            return true;
        literal_mismatch_1:
            /* preds are the same anyway in diamond/left merges just need to
             * check that all the literals in rev_leftfix[a_h] can handle b_h */
            for (auto v : rai.rev_leftfix[a_left_id]) {
                if (!mergeableRoseVertices(build, v, b)) {
                    goto literal_mismatch_2;
                }
            }

            rai.rev_leftfix[b_left_id].erase(b);
            rai.rev_leftfix[a_left_id].insert(b);
            b_left.graph = a_h;
            b_left.leftfix_report = a_left.leftfix_report;
            pruneUnusedTops(*b_h, g, rai.rev_leftfix[b_left_id]);
            DEBUG_PRINTF("OK -> same preds ???\n");
            return true;
        literal_mismatch_2:;
        }
        DEBUG_PRINTF("OK -> create new\n");
        /* we need to create a new graph as there may be other people
         * using b_left and it would be bad if a's preds started triggering it
         */
        ReportID new_report = build.getNewNfaReport();
        shared_ptr<NGHolder> new_graph = cloneHolder(*b_h);
        duplicateReport(*new_graph, b_left.leftfix_report, new_report);
        pruneAllOtherReports(*new_graph, new_report);

        if (!isImplementableNFA(*new_graph, nullptr, build.cc)) {
            DEBUG_PRINTF("new graph not implementable\n");
            return false;
        }

        rai.rev_leftfix[a_left_id].erase(a);
        rai.rev_leftfix[b_left_id].erase(b);
        pruneUnusedTops(*a_h, g, rai.rev_leftfix[a_left_id]);
        pruneUnusedTops(*b_h, g, rai.rev_leftfix[b_left_id]);

        a_left.leftfix_report = new_report;
        b_left.leftfix_report = new_report;
        a_left.graph = new_graph;
        b_left.graph = new_graph;

        rai.rev_leftfix[a_left].insert(a);
        rai.rev_leftfix[a_left].insert(b);
        pruneUnusedTops(*new_graph, g, rai.rev_leftfix[a_left]);
        return true;
    }

    // Everything after this point requires merging via the uncalc code, so we
    // guard it with the trivial cases arg.
    if (trivialCasesOnly) {
        return false;
    }

    // Only infixes. Prefixes require special care when doing non-trivial
    // merges.
    if (!build.isNonRootSuccessor(a) || !build.isNonRootSuccessor(b)) {
        return false;
    }

    DEBUG_PRINTF("attempting merge of roses on vertices %zu and %zu\n",
                 g[a].index, g[b].index);

    set<RoseVertex> &b_verts = rai.rev_leftfix[b_left];
    set<RoseVertex> aa;
    aa.insert(a);

    if (!mergeableRoseVertices(build, aa, b_verts)) {
        DEBUG_PRINTF("vertices not mergeable\n");
        return false;
    }

    if (!build.cc.grey.roseMultiTopRoses) {
        return false;
    }

    // Clone a copy of a's NFA to operate on, and store a copy of its in-edge
    // properties.

    /* We need to allocate a new report id because */
    ReportID a_oldreport = a_left.leftfix_report;
    ReportID b_oldreport = b_left.leftfix_report;
    ReportID new_report = build.getNewNfaReport();
    duplicateReport(*b_h, b_left.leftfix_report, new_report);
    b_left.leftfix_report = new_report;
    pruneReportIfUnused(build, b_h, rai.rev_leftfix[b_left_id], b_oldreport);

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
        assert(roseHasTops(build, a));
        assert(roseHasTops(build, b));
        return false;
    }

    assert(victim.kind == b_h->kind);
    assert(!generates_callbacks(*b_h));

    if (!mergeNfaPair(victim, *b_h, nullptr, build.cc)) {
        DEBUG_PRINTF("merge failed\n");
        // Restore in-edge properties.
        for (const auto &e : in_edges_range(a, g)) {
            g[e] = a_props[source(e, g)];
        }
        assert(roseHasTops(build, a));
        assert(roseHasTops(build, b));
        return false;
    }

    DEBUG_PRINTF("merge succeeded -> %zu vertices\n", num_vertices(*b_h));

    // update A's rose data to point to the merged graph.
    a_left.graph = b_h;
    a_left.leftfix_report = new_report;

    assert(contains(rai.rev_leftfix[a_left_id], a));
    assert(contains(rai.rev_leftfix[b_left_id], b));
    rai.rev_leftfix[a_left_id].erase(a);
    rai.rev_leftfix[b_left_id].insert(a);

    pruneUnusedTops(*a_h, g, rai.rev_leftfix[a_left_id]);
    pruneUnusedTops(*b_h, g, rai.rev_leftfix[b_left_id]);

    // Prune A's report from its old prefix if it was only used by A.
    pruneReportIfUnused(build, a_h, rai.rev_leftfix[a_left_id], a_oldreport);

    reduceImplementableGraph(*b_h, SOM_NONE, nullptr, build.cc);

    assert(roseHasTops(build, a));
    assert(roseHasTops(build, b));
    assert(isImplementableNFA(*b_h, nullptr, build.cc));
    return true;
}

// Called by the role aliasing pass: Attempt to merge rose a into b, updating
// the two LeftEngInfo structures to be the same. Returns false if the merge
// is not possible.
static
bool attemptRoseMerge(RoseBuildImpl &build, bool preds_same, RoseVertex a,
                      RoseVertex b, bool trivialCasesOnly,
                      RoseAliasingInfo &rai) {
    DEBUG_PRINTF("attempting rose merge, vertices a=%zu, b=%zu\n",
                  build.g[a].index, build.g[b].index);
    assert(a != b);

    RoseGraph &g = build.g;
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
    if (contains(build.transient, a_left_id) ||
        contains(build.transient, b_left_id)) {
        return false;
    }

    /* It is not possible to merge roles with different lags as we can only
     * test the leftfix at one location relative to the literal match */
    if (a_left.lag != b_left.lag) {
        return false;
    }

    assert(roseHasTops(build, a));
    assert(roseHasTops(build, b));

    if (a_left_id.graph() && b_left_id.graph()) {
        return attemptRoseGraphMerge(build, preds_same, a, b, trivialCasesOnly,
                                     rai);
    }

    if (a_left_id.castle() && b_left_id.castle()) {
        return attemptRoseCastleMerge(build, preds_same, a, b, trivialCasesOnly,
                                      rai);
    }

    return false;
}

/**
 * \brief Buckets that only contain one vertex are never going to lead to a
 * merge.
 */
static
void removeSingletonBuckets(vector<vector<RoseVertex>> &buckets) {
    auto it = remove_if(
        begin(buckets), end(buckets),
        [](const vector<RoseVertex> &bucket) { return bucket.size() < 2; });
    if (it != end(buckets)) {
        DEBUG_PRINTF("deleting %zu singleton buckets\n",
                     distance(it, end(buckets)));
        buckets.erase(it, end(buckets));
    }
}

static
void buildInvBucketMap(const vector<vector<RoseVertex>> &buckets,
                       unordered_map<RoseVertex, size_t> &inv) {
    inv.clear();
    for (size_t i = 0; i < buckets.size(); i++) {
        for (auto v : buckets[i]) {
            assert(!contains(inv, v));
            inv.emplace(v, i);
        }
    }
}

/**
 * \brief Generic splitter that will use the given split function to partition
 * the vector of buckets, then remove buckets with <= 1 entry.
 */
template <class SplitFunction>
void splitAndFilterBuckets(vector<vector<RoseVertex>> &buckets,
                           const SplitFunction &make_split_key) {
    if (buckets.empty()) {
        return;
    }

    vector<vector<RoseVertex>> out;

    // Mapping from split key value to new bucket index.
    using key_type = decltype(make_split_key(RoseGraph::null_vertex()));
    unordered_map<key_type, size_t> dest_map;
    dest_map.reserve(buckets.front().size());

    for (const auto &bucket : buckets) {
        assert(!bucket.empty());
        dest_map.clear();
        for (RoseVertex v : bucket) {
            auto p = dest_map.emplace(make_split_key(v), out.size());
            if (p.second) { // New key, add a bucket.
                out.emplace_back();
            }
            auto out_bucket = p.first->second;
            out[out_bucket].push_back(v);
        }
    }

    if (out.size() == buckets.size()) {
        return; // No new buckets created.
    }

    buckets = std::move(out);
    removeSingletonBuckets(buckets);
}

static
void splitByReportSuffixBehaviour(const RoseGraph &g,
                                  vector<vector<RoseVertex>> &buckets) {
    // Split by report set and suffix info.
    auto make_split_key = [&g](RoseVertex v) {
        return hash_all(g[v].reports, g[v].suffix);
    };
    splitAndFilterBuckets(buckets, make_split_key);
}

static
void splitByLiteralTable(const RoseBuildImpl &build,
                         vector<vector<RoseVertex>> &buckets) {
    const RoseGraph &g = build.g;

    // Split by literal table.
    auto make_split_key = [&](RoseVertex v) {
        const auto &lits = g[v].literals;
        assert(!lits.empty());
        auto table = build.literals.at(*lits.begin()).table;
        return std::underlying_type<decltype(table)>::type(table);
    };
    splitAndFilterBuckets(buckets, make_split_key);
}

static
void splitByNeighbour(const RoseGraph &g, vector<vector<RoseVertex>> &buckets,
                      unordered_map<RoseVertex, size_t> &inv, bool succ) {
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

    removeSingletonBuckets(buckets);
    buildInvBucketMap(buckets, inv);
}

static
vector<vector<RoseVertex>>
splitDiamondMergeBuckets(CandidateSet &candidates, const RoseBuildImpl &build) {
    const RoseGraph &g = build.g;

    vector<vector<RoseVertex>> buckets(1);
    buckets[0].reserve(candidates.size());
    insert(&buckets[0], buckets[0].end(), candidates);

    DEBUG_PRINTF("at start, %zu candidates in 1 bucket\n", candidates.size());

    splitByReportSuffixBehaviour(g, buckets);
    DEBUG_PRINTF("split by report/suffix, %zu buckets\n", buckets.size());
    if (buckets.empty()) {
        return buckets;
    }

    splitByLiteralTable(build, buckets);
    DEBUG_PRINTF("split by lit table, %zu buckets\n", buckets.size());
    if (buckets.empty()) {
        return buckets;
    }

    // Neighbour splits require inverse map.
    unordered_map<RoseVertex, size_t> inv;
    buildInvBucketMap(buckets, inv);

    splitByNeighbour(g, buckets, inv, true);
    DEBUG_PRINTF("split by successor, %zu buckets\n", buckets.size());
    if (buckets.empty()) {
        return buckets;
    }

    splitByNeighbour(g, buckets, inv, false);
    DEBUG_PRINTF("split by predecessor, %zu buckets\n", buckets.size());

    return buckets;
}

static never_inline
void diamondMergePass(CandidateSet &candidates, RoseBuildImpl &build,
                      vector<RoseVertex> *dead, bool mergeRoses,
                      RoseAliasingInfo &rai) {
    DEBUG_PRINTF("begin\n");
    RoseGraph &g = build.g;

    if (candidates.empty()) {
        return;
    }

    /* Vertices may only be diamond merged with others in the same bucket */
    auto cand_buckets = splitDiamondMergeBuckets(candidates, build);

    for (const vector<RoseVertex> &siblings : cand_buckets) {
        for (auto it = siblings.begin(); it != siblings.end();) {
            RoseVertex a = *it;
            ++it;

            assert(contains(candidates, a));

            DEBUG_PRINTF("trying to merge %zu into somebody\n", g[a].index);
            for (auto jt = it; jt != siblings.end(); ++jt) {
                RoseVertex b = *jt;
                assert(contains(candidates, b));

                if (!sameRoleProperties(build, rai, a, b)) {
                    DEBUG_PRINTF("diff role prop\n");
                    continue;
                }

                // Check "diamond" requirements: must have same right side
                // (successors, reports) and left side (predecessors).
                /* Note: bucketing does not check edge properties (bounds, tops)
                 * so we still have to checks successors and predecessors. */

                if (!sameSuccessors(a, b, g)
                    || !sameRightRoleProperties(build, a, b)
                    || !samePredecessors(a, b, g)) {
                    DEBUG_PRINTF("not diamond\n");
                    continue;
                }

                if (!canMergeLiterals(a, b, build)) {
                    DEBUG_PRINTF("incompatible lits\n");
                    continue;
                }

                if (!attemptRoseMerge(build, true, a, b, !mergeRoses, rai)) {
                    DEBUG_PRINTF("rose fail\n");
                    continue;
                }

                mergeVerticesDiamond(a, b, build, rai);
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
                          const RoseAliasingInfo &rai,
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

        if (!sameRoleProperties(build, rai, a, b)) {
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

static
void getLeftMergeSiblings(const RoseBuildImpl &build, RoseVertex a,
                          vector<RoseVertex> &siblings) {
    // We have to find a sibling to merge `a' with, and we select between
    // two approaches to minimize the number of vertices we have to
    // examine; which we use depends on the shape of the graph.

    const RoseGraph &g = build.g;
    assert(!g[a].literals.empty());
    u32 lit_id = *g[a].literals.begin();
    const auto &verts = build.literal_info.at(lit_id).vertices;
    RoseVertex pred = pickPred(a, g, build);

    siblings.clear();

    if (pred == RoseGraph::null_vertex() || build.isAnyStart(pred) ||
        out_degree(pred, g) > verts.size()) {
        // Select sibling from amongst the vertices that share a literal.
        insert(&siblings, siblings.end(), verts);
    } else {
        // Select sibling from amongst the vertices that share a
        // predecessor.
        insert(&siblings, siblings.end(), adjacent_vertices(pred, g));
    }
}

static never_inline
void leftMergePass(CandidateSet &candidates, RoseBuildImpl &build,
                   vector<RoseVertex> *dead, RoseAliasingInfo &rai) {
    DEBUG_PRINTF("begin (%zu)\n", candidates.size());
    vector<RoseVertex> siblings;

    auto it = candidates.begin();
    while (it != candidates.end()) {
        RoseVertex a = *it;
        CandidateSet::iterator ait = it;
        ++it;

        getLeftMergeSiblings(build, a, siblings);

        auto jt = siblings.begin();
        while (jt != siblings.end()) {
            jt = findLeftMergeSibling(jt, siblings.end(), a, build, rai,
                                      candidates);
            if (jt == siblings.end()) {
                break;
            }
            RoseVertex b = *jt;
            if (attemptRoseMerge(build, true, a, b, false, rai)) {
                mergeVerticesLeft(a, b, build, rai);
                dead->push_back(a);
                candidates.erase(ait);
                break; // consider next a
            }
            ++jt;
        }
    }

    DEBUG_PRINTF("%zu candidates remaining\n", candidates.size());
    assert(!hasOrphanedTops(build));
}

// Can't merge vertices with different root predecessors.
static
bool safeRootPreds(RoseVertex a, RoseVertex b, const RoseGraph &g) {
    set<RoseVertex> a_roots, b_roots;

    for (auto u : inv_adjacent_vertices_range(a, g)) {
        if (!in_degree(u, g)) {
            a_roots.insert(u);
        }
    }
    for (auto u : inv_adjacent_vertices_range(b, g)) {
        if (!in_degree(u, g)) {
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
                           const RoseAliasingInfo &rai,
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

        if (!sameRoleProperties(build, rai, a, b)) {
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

static
void splitByRightProps(const RoseGraph &g,
                      vector<vector<RoseVertex>> &buckets) {
    // Successor vector used in make_split_key. We declare it here so we can
    // reuse storage.
    vector<RoseVertex> succ;

    // Split by {successors, literals, reports}.
    auto make_split_key = [&](RoseVertex v) {
        succ.clear();
        insert(&succ, succ.end(), adjacent_vertices(v, g));
        sort(succ.begin(), succ.end());
        return hash_all(g[v].literals, g[v].reports, succ);
    };
    splitAndFilterBuckets(buckets, make_split_key);
}

static never_inline
vector<vector<RoseVertex>>
splitRightMergeBuckets(const CandidateSet &candidates,
                       const RoseBuildImpl &build) {
    const RoseGraph &g = build.g;

    vector<vector<RoseVertex>> buckets(1);
    buckets[0].reserve(candidates.size());
    insert(&buckets[0], buckets[0].end(), candidates);

    DEBUG_PRINTF("at start, %zu candidates in 1 bucket\n", candidates.size());

    splitByReportSuffixBehaviour(g, buckets);
    DEBUG_PRINTF("split by report/suffix, %zu buckets\n", buckets.size());
    if (buckets.empty()) {
        return buckets;
    }

    splitByRightProps(g, buckets);
    DEBUG_PRINTF("split by right-merge properties, %zu buckets\n",
                 buckets.size());
    if (buckets.empty()) {
        return buckets;
    }

    return buckets;
}

static never_inline
void rightMergePass(CandidateSet &candidates, RoseBuildImpl &build,
                    vector<RoseVertex> *dead, bool mergeRoses,
                    RoseAliasingInfo &rai) {
    DEBUG_PRINTF("begin\n");

    if (candidates.empty()) {
        return;
    }

    auto buckets = splitRightMergeBuckets(candidates, build);

    for (const auto &bucket : buckets) {
        assert(!bucket.empty());
        for (auto it = bucket.begin(); it != bucket.end(); it++) {
            RoseVertex a = *it;
            for (auto jt = bucket.begin(); jt != bucket.end(); jt++) {
                jt = findRightMergeSibling(jt, bucket.end(), a, build, rai,
                                           candidates);
                if (jt == bucket.end()) {
                    break;
                }
                RoseVertex b = *jt;
                if (attemptRoseMerge(build, false, a, b, !mergeRoses, rai)) {
                    mergeVerticesRight(a, b, build, rai);
                    dead->push_back(a);
                    candidates.erase(a);
                    break; // consider next a
                }
            }
        }
    }

    DEBUG_PRINTF("%zu candidates remaining\n", candidates.size());
    assert(!hasOrphanedTops(build));
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
            if (in_degree(w, g) > 1) {
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
        if (out_degree(u, g) > 1) {
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
    assert(!hasOrphanedTops(build));
    assert(canImplementGraphs(build));

    if (!cc.grey.roseRoleAliasing || !cc.grey.roseGraphReduction) {
        return;
    }

    DEBUG_PRINTF("doing role aliasing mr=%d\n", (int)mergeRoses);

    RoseAliasingInfo rai(build);

    mergeRoses &= cc.grey.mergeRose & cc.grey.roseMergeRosesDuringAliasing;

    CandidateSet candidates;
    findCandidates(build, &candidates);

    DEBUG_PRINTF("candidates %zu\n", candidates.size());

    vector<RoseVertex> dead;
    size_t old_dead_size = 0;
    do {
        old_dead_size = dead.size();
        leftMergePass(candidates, build, &dead, rai);
        rightMergePass(candidates, build, &dead, mergeRoses, rai);
    } while (old_dead_size != dead.size());

    /* Diamond merge passes cannot create extra merges as they require the same
     * succ and preds before merging --> that if a succ/pred was ineligible due
     * to a merge to different pred/succ before a diamond merge, it will still
     * be afterwards. */
    filterDiamondCandidates(g, candidates);
    diamondMergePass(candidates, build, &dead, mergeRoses, rai);

    DEBUG_PRINTF("killed %zu vertices\n", dead.size());
    build.removeVertices(dead);
    assert(!hasOrphanedTops(build));
    assert(canImplementGraphs(build));
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
 *
 * TODO: work if this is a subset of role aliasing (and if it can be eliminated)
 * or clearly document cases that would not be covered by role aliasing.
 */
void mergeDupeLeaves(RoseBuildImpl &build) {
    map<DupeLeafKey, RoseVertex> leaves;
    vector<RoseVertex> changed;

    RoseGraph &g = build.g;
    for (auto v : vertices_range(g)) {
        if (in_degree(v, g) == 0) {
            assert(build.isAnyStart(v));
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
        if (build.isRootSuccessor(v)) {
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
            if (!build.isVirtualVertex(v)) {
                for (u32 lit_id : g[v].literals) {
                    build.literal_info[lit_id].vertices.erase(v);
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
void findUncalcLeavesCandidates(RoseBuildImpl &build,
                           map<UncalcLeafKey, vector<RoseVertex> > &clusters,
                           deque<UncalcLeafKey> &ordered) {
    const RoseGraph &g = build.g;

    vector<RoseVertex> suffix_vertices; // vertices with suffix graphs
    unordered_map<const NGHolder *, u32> fcount; // ref count per graph

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
            assert(build.isAnyStart(v));
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
void uncalcLeaves(RoseBuildImpl &build) {
    DEBUG_PRINTF("uncalcing\n");

    map<UncalcLeafKey, vector<RoseVertex> > clusters;
    deque<UncalcLeafKey> ordered;
    findUncalcLeavesCandidates(build, clusters, ordered);

    vector<RoseVertex> dead;

    for (const auto &key : ordered) {
        DEBUG_PRINTF("cluster of size %zu\n", clusters[key].size());
        mergeCluster(build.g, build.rm, clusters[key], dead, build.cc);
    }
    build.removeVertices(dead);
}

} // namespace ue2
