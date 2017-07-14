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
 * \brief Edge redundancy graph reductions.
 */
#include "ng_edge_redundancy.h"

#include "ng_holder.h"
#include "ng_prune.h"
#include "ng_util.h"
#include "ue2common.h"
#include "parser/position.h"
#include "util/compile_context.h"
#include "util/container.h"
#include "util/flat_containers.h"
#include "util/graph_range.h"

#include <set>
#include <vector>

using namespace std;

namespace ue2 {

/* reverse edge redundancy removal is possible but is not implemented as it
 * regressed rose pattern support in the regression suite: 19026 - 19027
 * (foo.{1,5}b?ar)
 *
 * If rose becomes smarter we can reimplement.
 */

static never_inline
bool checkVerticesFwd(const NGHolder &g, const set<NFAVertex> &sad,
                      const set<NFAVertex> &happy) {
    /* need to check if for each vertex in sad if it has an edge to a happy
     * vertex */
    for (auto u : sad) {
        bool ok = false;
        for (auto v : adjacent_vertices_range(u, g)) {
            if (contains(happy, v)) {
                ok = true;
                break;
            }
        }

        if (!ok) {
            return false;
        }
    }

    return true;
}

static never_inline
bool checkVerticesRev(const NGHolder &g, const set<NFAVertex> &sad,
                      const set<NFAVertex> &happy) {
    /* need to check if for each vertex in sad if it has an edge to a happy
     * vertex */
    for (auto v : sad) {
        bool ok = false;
        for (auto u : inv_adjacent_vertices_range(v, g)) {
            if (contains(happy, u)) {
                ok = true;
                break;
            }
        }

        if (!ok) {
            return false;
        }
    }

    return true;
}

/** \brief Redundant self-loop removal.
 *
 * A self loop on a vertex v can be removed if:
 *
 *     For every vertex u in pred(v) either:
 *         1: u has a self loop and cr(v) subset of cr(u)
 *         OR
 *         2: u has an edge to vertex satisfying criterion 1
 *
 * Note: we remove all dead loops at the end of the pass and do not check the
 * live status of the loops we are depending on during the analysis.
 *
 * We don't end up in situations where we remove a group of loops which depend
 * on each other as:
 *
 * - there must be at least one vertex not in the group which is a pred of some
 *   member of the group (as we don't remove loops on specials)
 *
 * For each pred vertex of the group:
 * - the vertex must be 'sad' as it is not part of the group
 * - therefore it must have edges to each member of the group (to happy, trans)
 * - therefore the group is enabled simultaneously
 * - due to internal group edges, all members will still be active after the
 *   next character.
 *
 * Actually, the vertex redundancy code will merge the entire group into one
 * cyclic state.
 */
static
bool removeEdgeRedundancyNearCyclesFwd(NGHolder &g, bool ignore_starts) {
    unsigned dead_count = 0;

    set<NFAVertex> happy;
    set<NFAVertex> sad;

    for (auto v : vertices_range(g)) {
        if (is_special(v, g) || !hasSelfLoop(v, g)) {
            continue;
        }

        const CharReach &cr_v = g[v].char_reach;

        happy.clear();
        sad.clear();

        for (auto u : inv_adjacent_vertices_range(v, g)) {
            if (u == v) {
                continue;
            }

            if (!hasSelfLoop(u, g)) {
                sad.insert(u);
                continue;
            }

            if (ignore_starts) {
                if (u == g.startDs || is_virtual_start(u, g)) {
                    sad.insert(u);
                    continue;
                }
            }

            const CharReach &cr_u = g[u].char_reach;

            if ((cr_u & cr_v) != cr_v) {
                sad.insert(u);
                continue;
            }

            happy.insert(u);
        }

        if (!happy.empty() && checkVerticesFwd(g, sad, happy)) {
            dead_count++;
            remove_edge(v, v, g);
        }
    }

    DEBUG_PRINTF("found %u removable edges.\n", dead_count);
    return dead_count;
}

static
bool checkReportsRev(const NGHolder &g, NFAVertex v,
                     const set<NFAVertex> &happy) {
    if (g[v].reports.empty()) {
        return true;
    }

    assert(edge(v, g.accept, g).second || edge(v, g.acceptEod, g).second);

    /* an edge to accept takes priority over eod only accept */
    NFAVertex accept = edge(v, g.accept, g).second ? g.accept : g.acceptEod;

    flat_set<ReportID> happy_reports;
    for (NFAVertex u : happy) {
        if (edge(u, accept, g).second) {
            insert(&happy_reports, g[u].reports);
        }
    }

    return is_subset_of(g[v].reports, happy_reports);
}

/** \brief Redundant self-loop removal (reverse version).
 *
 * A self loop on a vertex v can be removed if:
 *
 *     For every vertex u in succ(v) either:
 *         1: u has a self loop and cr(v) is a subset of cr(u).
 *         OR
 *         2: u is not an accept and u has an edge from a vertex satisfying
 *            criterion 1.
 *         OR
 *         3: u is in an accept and u has an edge from a vertex v' satisfying
 *            criterion 1 and report(v) == report(v').
 */
static
bool removeEdgeRedundancyNearCyclesRev(NGHolder &g) {
    unsigned dead_count = 0;

    set<NFAVertex> happy;
    set<NFAVertex> sad;

    for (auto v : vertices_range(g)) {
        if (is_special(v, g) || !hasSelfLoop(v, g)) {
            continue;
        }

        const CharReach &cr_v = g[v].char_reach;

        happy.clear();
        sad.clear();

        for (auto u : adjacent_vertices_range(v, g)) {
            if (u == v) {
                continue;
            }

            if (!hasSelfLoop(u, g)) {
                sad.insert(u);
                continue;
            }

            assert(!is_special(u, g));

            const CharReach &cr_u = g[u].char_reach;

            if (!cr_v.isSubsetOf(cr_u)) {
                sad.insert(u);
                continue;
            }

            happy.insert(u);
        }

        if (!happy.empty() && checkVerticesRev(g, sad, happy)
            && checkReportsRev(g, v, happy)) {
            dead_count++;
            remove_edge(v, v, g);
        }
    }

    DEBUG_PRINTF("found %u removable edges.\n", dead_count);
    return dead_count;
}

static
bool parentsSubsetOf(const NGHolder &g, NFAVertex v,
                     const flat_set<NFAVertex> &other_parents, NFAVertex other,
                     map<NFAVertex, bool> &done) {
    map<NFAVertex, bool>::const_iterator dit = done.find(v);
    if (dit != done.end()) {
        return dit->second;
    }

    for (auto u : inv_adjacent_vertices_range(v, g)) {
        if (u == v && contains(other_parents, other)) {
            continue;
        }

        if (!contains(other_parents, u)) {
            done[v] = false;
            return false;
        }
    }

    done[v] = true;
    return true;
}

static
bool checkFwdCandidate(const NGHolder &g, NFAVertex fixed_src,
                       const flat_set<NFAVertex> &fixed_parents,
                       const NFAEdge &candidate,
                       map<NFAVertex, bool> &done) {
    NFAVertex w = source(candidate, g);
    NFAVertex v = target(candidate, g);
    const CharReach &cr_w = g[w].char_reach;
    const CharReach &cr_u = g[fixed_src].char_reach;

    /* There is no reason why self loops cannot be considered by this
     * transformation but the removal is already handled by many other
     * transformations. */
    if (w == v) {
        return false;
    }

    if (is_special(w, g)) {
        return false;
    }

    if (!cr_w.isSubsetOf(cr_u)) {
        return false;
    }

    /* check that each parent of w is also a parent of u */
    if (!parentsSubsetOf(g, w, fixed_parents, fixed_src, done)) {
        return false;
    }

    DEBUG_PRINTF("edge (%zu, %zu) killed by edge (%zu, %zu)\n",
                 g[w].index, g[v].index, g[fixed_src].index, g[v].index);
    return true;
}

static never_inline
void checkLargeOutU(const NGHolder &g, NFAVertex u,
                    const flat_set<NFAVertex> &parents_u,
                    flat_set<NFAVertex> &possible_w,
                    map<NFAVertex, bool> &done,
                    set<NFAEdge> *dead) {
    /* only vertices with at least one parent in common with u need to be
     * considered, and we also only consider potential siblings with subset
     * reach. */
    possible_w.clear();
    const CharReach &cr_u = g[u].char_reach;
    for (auto p : parents_u) {
        for (auto v : adjacent_vertices_range(p, g)) {
            const CharReach &cr_w = g[v].char_reach;
            if (cr_w.isSubsetOf(cr_u)) {
                possible_w.insert(v);
            }
        }
    }

    // If there's only one, it's us, and we have no work to do.
    if (possible_w.size() <= 1) {
        assert(possible_w.empty() || *possible_w.begin() == u);
        return;
    }

    for (const auto &e : out_edges_range(u, g)) {
        const NFAVertex v = target(e, g);

        if (is_special(v, g)) {
            continue;
        }

        if (contains(*dead, e)) {
            continue;
        }

        /* Now need check to find any edges which can be removed due to the
         * existence of edge e */
        for (const auto &e2 : in_edges_range(v, g)) {
            if (e == e2 || contains(*dead, e2)) {
                continue;
            }

            const NFAVertex w = source(e2, g);
            if (!contains(possible_w, w)) {
                continue;
            }

            if (checkFwdCandidate(g, u, parents_u, e2, done)) {
                dead->insert(e2);
            }
        }
    }
}

static never_inline
void checkSmallOutU(const NGHolder &g, NFAVertex u,
                    const flat_set<NFAVertex> &parents_u,
                    map<NFAVertex, bool> &done,
                    set<NFAEdge> *dead) {
    for (const auto &e : out_edges_range(u, g)) {
        const NFAVertex v = target(e, g);

        if (is_special(v, g)) {
            continue;
        }

        if (contains(*dead, e)) {
            continue;
        }

        /* Now need check to find any edges which can be removed due to the
         * existence of edge e */
        for (const auto &e2 : in_edges_range(v, g)) {
            if (e == e2 || contains(*dead, e2)) {
                continue;
            }

            if (checkFwdCandidate(g, u, parents_u, e2, done)) {
                dead->insert(e2);
            }
        }
    }
}

/** \brief Forward edge redundancy pass.
 *
 *     An edge e from w to v is redundant if there exists an edge e' such that:
 *         e' is from u to v
 *         and:  reach(w) is a subset of reach(u)
 *         and:  proper_pred(w) is a subset of pred(u)
 *         and:  self_loop(w) implies self_loop(u) or edge from (w to u)
 *
 * Note: edges to accepts also require report ID checks.
 */
static
bool removeEdgeRedundancyFwd(NGHolder &g, bool ignore_starts) {
    set<NFAEdge> dead;
    map<NFAVertex, bool> done;
    flat_set<NFAVertex> parents_u;
    flat_set<NFAVertex> possible_w;

    for (auto u : vertices_range(g)) {
        if (ignore_starts && (u == g.startDs || is_virtual_start(u, g))) {
            continue;
        }

        parents_u.clear();
        pred(g, u, &parents_u);

        done.clear();
        if (out_degree(u, g) > 1) {
            checkLargeOutU(g, u, parents_u, possible_w, done, &dead);
        } else {
            checkSmallOutU(g, u, parents_u, done, &dead);
        }
    }

    if (dead.empty()) {
        return false;
    }

    DEBUG_PRINTF("found %zu removable non-selfloops.\n", dead.size());
    remove_edges(dead, g);
    pruneUseless(g);
    return true;
}

/** Entry point: Runs all the edge redundancy passes. If SoM is tracked,
 * don't consider startDs or virtual starts as cyclic vertices. */
bool removeEdgeRedundancy(NGHolder &g, som_type som, const CompileContext &cc) {
    if (!cc.grey.removeEdgeRedundancy) {
        return false;
    }

    bool changed = false;
    changed |= removeEdgeRedundancyNearCyclesFwd(g, som);
    changed |= removeEdgeRedundancyNearCyclesRev(g);
    changed |= removeEdgeRedundancyFwd(g, som);
    return changed;
}

/** \brief Removes optional stuff from the front of floating patterns, since it's
 * redundant with startDs.
 *
 * For each successor of startDs, remove any in-edges that aren't from either
 * start or startDs. This allows us to prune redundant vertices at the start of
 * a pattern:
 *
 *     /(hat)?stand --> /stand/
 *
 */
bool removeSiblingsOfStartDotStar(NGHolder &g) {
    vector<NFAEdge> dead;

    for (auto v : adjacent_vertices_range(g.startDs, g)) {
        DEBUG_PRINTF("checking %zu\n", g[v].index);
        if (is_special(v, g)) {
            continue;
        }

        for (const auto &e : in_edges_range(v, g)) {
            NFAVertex u = source(e, g);
            if (is_special(u, g)) {
                continue;
            }
            DEBUG_PRINTF("removing %zu->%zu\n", g[u].index, g[v].index);
            dead.push_back(e);
        }
    }

    if (dead.empty()) {
        return false;
    }

    DEBUG_PRINTF("found %zu removable edges.\n", dead.size());
    remove_edges(dead, g);
    pruneUseless(g);
    return true;
}

/** Removes all edges into virtual starts other than those from start/startDs,
 * providing there is an edge from startDs. This operation is an optimisation
 * for SOM mode. (see UE-1544) */
bool optimiseVirtualStarts(NGHolder &g) {
    vector<NFAEdge> dead;
    for (auto v : adjacent_vertices_range(g.startDs, g)) {
        u32 flags = g[v].assert_flags;
        if (!(flags & POS_FLAG_VIRTUAL_START)) {
            continue;
        }

        for (const auto &e : in_edges_range(v, g)) {
            if (!is_any_start(source(e, g), g)) {
                dead.push_back(e);
            }
        }
    }

    if (dead.empty()) {
        return false;
    }

    DEBUG_PRINTF("removing %zu edges into virtual starts\n", dead.size());
    remove_edges(dead, g);
    pruneUseless(g);
    return true;
}

} // namespace ue2
