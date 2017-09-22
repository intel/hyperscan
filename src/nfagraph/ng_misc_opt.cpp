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
 * \brief Miscellaneous optimisations.
 *
 * We sometimes see patterns of the form:
 *
 *     /^.*<[^<]*foobaz/s
 *
 * This is bad for Rose as the escapes from the cyclic state are the same as
 * the trigger. However, we can transform this into:
 *
 *     /^.*<.*foobaz/s
 *
 * ... as the first dot star can eat all but the last '<'.
 *
 * Slightly more formally:
 *
 * Given a cyclic state v with character reachability v_cr and proper preds
 * {p1 .. pn} with character reachability {p1_cr .. pn_cr}.
 *
 * let v_cr' = union(intersection(p1_cr .. pn_cr), v_cr)
 *
 * v_cr can be replaced with v_cr' without changing the behaviour of the system
 * if:
 *
 * for any given proper pred pi: if pi is set in the nfa then after consuming
 * any symbol in v_cr', pi will still be set in the nfa and every successor of
 * v is a successor of pi.
 *
 * The easiest way for this condition to be satisfied is for each proper pred
 * pi to have all its preds all have an edge to a pred of pi with a character
 * reachability containing v_cr'. There are, however, other ways to establish
 * the condition holds.
 *
 * Note: a similar transformation can be applied in reverse, details left as an
 * exercise for the interested reader. */
#include "ng_misc_opt.h"

#include "ng_holder.h"
#include "ng_prune.h"
#include "ng_util.h"
#include "util/charreach.h"
#include "util/container.h"
#include "util/graph_range.h"
#include "util/graph_small_color_map.h"
#include "util/flat_containers.h"
#include "ue2common.h"

#include <boost/dynamic_bitset.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/filtered_graph.hpp>

#include <map>
#include <set>
#include <vector>

using namespace std;
using boost::make_filtered_graph;

namespace ue2 {

static
void findCandidates(NGHolder &g, const vector<NFAVertex> &ordering,
                    vector<NFAVertex> *cand) {
    for (auto it = ordering.rbegin(), ite = ordering.rend(); it != ite; ++it) {
        NFAVertex v = *it;

        if (is_special(v, g)
            || !hasSelfLoop(v, g)
            || g[v].char_reach.all()) {
            continue;
        }

        // For `v' to be a candidate, its predecessors must all have the same
        // successor set as `v'.

        auto succ_v = succs(v, g);
        flat_set<NFAVertex> succ_u;

        for (auto u : inv_adjacent_vertices_range(v, g)) {
            succ_u.clear();
            succ(g, u, &succ_u);
            if (succ_v != succ_u) {
                goto next_cand;
            }
        }
        DEBUG_PRINTF("vertex %zu is a candidate\n", g[v].index);
        cand->push_back(v);
    next_cand:;
    }
}

static
void findCandidates_rev(NGHolder &g, const vector<NFAVertex> &ordering,
                        vector<NFAVertex> *cand) {
    for (auto it = ordering.begin(), ite = ordering.end(); it != ite; ++it) {
        NFAVertex v = *it;

        if (is_special(v, g)
            || !hasSelfLoop(v, g)
            || g[v].char_reach.all()) {
            continue;
        }

        // For `v' to be a candidate, its predecessors must all have the same
        // successor set as `v'.

        auto pred_v = preds(v, g);
        flat_set<NFAVertex> pred_u;

        for (auto u : adjacent_vertices_range(v, g)) {
            pred_u.clear();
            pred(g, u, &pred_u);
            if (pred_v != pred_u) {
                goto next_cand;
            }
        }
        DEBUG_PRINTF("vertex %zu is a candidate\n", g[v].index);
        cand->push_back(v);
    next_cand:;
    }
}

/** Find the intersection of the reachability of the predecessors of \p v. */
static
void predCRIntersection(const NGHolder &g, NFAVertex v, CharReach &add) {
    add.setall();
    for (auto u : inv_adjacent_vertices_range(v, g)) {
        if (u != v) {
            add &= g[u].char_reach;
        }
    }
}

/** Find the intersection of the reachability of the successors of \p v. */
static
void succCRIntersection(const NGHolder &g, NFAVertex v, CharReach &add) {
    add.setall();
    for (auto u : adjacent_vertices_range(v, g)) {
        if (u != v) {
            add &= g[u].char_reach;
        }
    }
}

/** The sustain set is used to show that once vertex p is on it stays on given
 * the alphabet new_cr. Every vertex pp in the sustain set has the following
 * properties:
 * -# an edge to p
 * -# enough edges to vertices in the sustain set to ensure that a vertex in
 *    the sustain set will be on after consuming a character. */
static
set<NFAVertex> findSustainSet(const NGHolder &g, NFAVertex p,
                              bool ignore_starts, const CharReach &new_cr) {
    auto cand = preds<set<NFAVertex>>(p, g);
    if (ignore_starts) {
        cand.erase(g.startDs);
    }
    /* remove elements from cand until the sustain set property holds */
    bool changed;
    do {
        DEBUG_PRINTF("|cand| %zu\n", cand.size());
        changed = false;
        set<NFAVertex>::const_iterator it = cand.begin();
        while (it != cand.end()) {
            NFAVertex u = *it;
            ++it;
            CharReach sus_cr;
            for (auto v : adjacent_vertices_range(u, g)) {
                if (contains(cand, v)) {
                    sus_cr |= g[v].char_reach;
                }
            }

            if (!new_cr.isSubsetOf(sus_cr)) {
                cand.erase(u);
                changed = true;
            }
        }
    } while (changed);

    /* Note: it may be possible to find a (larger) sustain set for a smaller
     * new_cr */
    return cand;
}

/** Finds the reverse version of the sustain set.. whatever that means. */
static
set<NFAVertex> findSustainSet_rev(const NGHolder &g, NFAVertex p,
                                  const CharReach &new_cr) {
    auto cand = succs<set<NFAVertex>>(p, g);
    /* remove elements from cand until the sustain set property holds */
    bool changed;
    do {
        changed = false;
        set<NFAVertex>::const_iterator it = cand.begin();
        while (it != cand.end()) {
            NFAVertex u = *it;
            ++it;
            CharReach sus_cr;
            for (auto v : inv_adjacent_vertices_range(u, g)) {
                if (contains(cand, v)) {
                    sus_cr |= g[v].char_reach;
                }
            }

            if (!new_cr.isSubsetOf(sus_cr)) {
                cand.erase(u);
                changed = true;
            }
        }
    } while (changed);

    /* Note: it may be possible to find a (larger) sustain set for a smaller
     * new_cr */
    return cand;
}

static
bool enlargeCyclicVertex(NGHolder &g, som_type som, NFAVertex v) {
    DEBUG_PRINTF("considering vertex %zu\n", g[v].index);
    const CharReach &v_cr = g[v].char_reach;

    CharReach add;
    predCRIntersection(g, v, add);

    add |= v_cr;

    if (add == v_cr) {
        DEBUG_PRINTF("no benefit\n");
        return false;
    }

    DEBUG_PRINTF("cr of width %zu up for grabs\n", add.count() - v_cr.count());

    for (auto p : inv_adjacent_vertices_range(v, g)) {
        if (p == v) {
            continue;
        }
        DEBUG_PRINTF("looking at pred %zu\n", g[p].index);

        bool ignore_sds = som; /* if we are tracking som, entries into a state
                                  from sds are significant. */

        set<NFAVertex> sustain = findSustainSet(g, p, ignore_sds, add);
        DEBUG_PRINTF("sustain set is %zu\n", sustain.size());
        if (sustain.empty()) {
            DEBUG_PRINTF("yawn\n");
        }

        for (auto pp : inv_adjacent_vertices_range(p, g)) {
            /* we need to ensure that whenever pp sets p, that a member of the
               sustain set is set. Note: p's cr may be not be a subset of
               new_cr */
            CharReach sustain_cr;
            for (auto pv : adjacent_vertices_range(pp, g)) {
                if (contains(sustain, pv)) {
                    sustain_cr |= g[pv].char_reach;
                }
            }
            if (!g[p].char_reach.isSubsetOf(sustain_cr)) {
                DEBUG_PRINTF("unable to establish that preds are forced on\n");
                return false;
            }
        }
    }

    /* the cr can be increased */
    g[v].char_reach = add;
    DEBUG_PRINTF("vertex %zu was widened\n", g[v].index);
    return true;
}

static
bool enlargeCyclicVertex_rev(NGHolder &g, NFAVertex v) {
    DEBUG_PRINTF("considering vertex %zu\n", g[v].index);
    const CharReach &v_cr = g[v].char_reach;

    CharReach add;
    succCRIntersection(g, v, add);

    add |= v_cr;

    if (add == v_cr) {
        DEBUG_PRINTF("no benefit\n");
        return false;
    }

    DEBUG_PRINTF("cr of width %zu up for grabs\n", add.count() - v_cr.count());

    for (auto p : adjacent_vertices_range(v, g)) {
        if (p == v) {
            continue;
        }
        DEBUG_PRINTF("looking at succ %zu\n", g[p].index);

        set<NFAVertex> sustain = findSustainSet_rev(g, p, add);
        DEBUG_PRINTF("sustain set is %zu\n", sustain.size());
        if (sustain.empty()) {
            DEBUG_PRINTF("yawn\n");
        }

        for (auto pp : adjacent_vertices_range(p, g)) {
            /* we need to ensure something - see fwd ver */
            CharReach sustain_cr;
            for (auto pv : inv_adjacent_vertices_range(pp, g)) {
                if (contains(sustain, pv)) {
                    sustain_cr |= g[pv].char_reach;
                }
            }
            if (!g[p].char_reach.isSubsetOf(sustain_cr)) {
                DEBUG_PRINTF("unable to establish that succs are thingy\n");
                return false;
            }
        }
    }

    /* the cr can be increased */
    g[v].char_reach = add;
    DEBUG_PRINTF("vertex %zu was widened\n", g[v].index);
    return true;
}

static
bool enlargeCyclicCR(NGHolder &g, som_type som,
                     const vector<NFAVertex> &ordering) {
    DEBUG_PRINTF("hello\n");

    vector<NFAVertex> candidates;
    findCandidates(g, ordering, &candidates);

    bool rv = false;
    for (auto v : candidates) {
        rv |= enlargeCyclicVertex(g, som, v);
    }

    return rv;
}

static
bool enlargeCyclicCR_rev(NGHolder &g, const vector<NFAVertex> &ordering) {
    DEBUG_PRINTF("olleh\n");

    vector<NFAVertex> candidates;
    findCandidates_rev(g, ordering, &candidates);

    bool rv = false;
    for (auto v : candidates) {
        rv |= enlargeCyclicVertex_rev(g, v);
    }

    return rv;
}

bool improveGraph(NGHolder &g, som_type som) {
    /* use a topo ordering so that we can get chains of cyclic states
     * done in one sweep */

    const vector<NFAVertex> ordering = getTopoOrdering(g);

    return enlargeCyclicCR(g, som, ordering)
        | enlargeCyclicCR_rev(g, ordering);
}

/** finds a smaller reachability for a state by the reverse transformation of
 * enlargeCyclicCR. */
CharReach reduced_cr(NFAVertex v, const NGHolder &g,
                     const map<NFAVertex, BoundedRepeatSummary> &br_cyclic) {
    DEBUG_PRINTF("find minimal cr for %zu\n", g[v].index);
    CharReach v_cr = g[v].char_reach;
    if (proper_in_degree(v, g) != 1) {
        return v_cr;
    }

    NFAVertex pred = getSoleSourceVertex(g, v);
    assert(pred);

    /* require pred to be fed by one vertex OR (start + startDS) */
    NFAVertex predpred;
    size_t idp = in_degree(pred, g);
    if (hasSelfLoop(pred, g)) {
        return v_cr; /* not cliche */
    } else if (idp == 1) {
        predpred = getSoleSourceVertex(g, pred);
    } else if (idp == 2
               && edge(g.start, pred, g).second
               && edge(g.startDs, pred, g).second) {
        predpred = g.startDs;
    } else {
        return v_cr; /* not cliche */
    }

    assert(predpred);

    /* require predpred to be cyclic and its cr to be a superset of
       pred and v */
    if (!hasSelfLoop(predpred, g)) {
        return v_cr; /* not cliche */
    }

    if (contains(br_cyclic, predpred)
        && !br_cyclic.at(predpred).unbounded()) {
        return v_cr; /* fake cyclic */
    }

    const CharReach &p_cr = g[pred].char_reach;
    const CharReach &pp_cr = g[predpred].char_reach;
    if (!v_cr.isSubsetOf(pp_cr) || !p_cr.isSubsetOf(pp_cr)) {
        return v_cr; /* not cliche */
    }

    DEBUG_PRINTF("confirming [x]* prop\n");
    /* we require all of v succs to be succ of p */
    set<NFAVertex> v_succ;
    insert(&v_succ, adjacent_vertices(v, g));
    set<NFAVertex> p_succ;
    insert(&p_succ, adjacent_vertices(pred, g));

    if (!is_subset_of(v_succ, p_succ)) {
        DEBUG_PRINTF("fail\n");
        return v_cr; /* not cliche */
    }

    if (contains(v_succ, g.accept) || contains(v_succ, g.acceptEod)) {
        /* need to check that reports of v are a subset of p's */
        if (!is_subset_of(g[v].reports,
                        g[pred].reports)) {
            DEBUG_PRINTF("fail - reports not subset\n");
            return v_cr; /* not cliche */
        }
    }

    DEBUG_PRINTF("woot success\n");
    v_cr &= ~p_cr;
    return v_cr;
}

vector<CharReach> reduced_cr(const NGHolder &g,
                const map<NFAVertex, BoundedRepeatSummary> &br_cyclic) {
    assert(hasCorrectlyNumberedVertices(g));
    vector<CharReach> refined_cr(num_vertices(g), CharReach());

    for (auto v : vertices_range(g)) {
        u32 v_idx = g[v].index;
        refined_cr[v_idx] = reduced_cr(v, g, br_cyclic);
    }

    return refined_cr;
}

static
bool anyOutSpecial(NFAVertex v, const NGHolder &g) {
    for (auto w : adjacent_vertices_range(v, g)) {
        if (is_special(w, g) && w != v) {
            return true;
        }
    }
    return false;
}

bool mergeCyclicDotStars(NGHolder &g) {
    set<NFAVertex> verticesToRemove;
    set<NFAEdge> edgesToRemove;

    // avoid graphs where startDs is not a free spirit
    if (out_degree(g.startDs, g) > 1) {
        return false;
    }

    // check if any of the connected vertices are dots
    for (auto v : adjacent_vertices_range(g.start, g)) {
        if (is_special(v, g)) {
            continue;
        }
        const CharReach &cr = g[v].char_reach;

        // if this is a cyclic dot
        if (cr.all() && edge(v, v, g).second) {
            // prevent insane graphs
            if (anyOutSpecial(v, g)) {
                continue;
            }
            // we don't know if we're going to remove this vertex yet
            vector<NFAEdge> deadEdges;

            // check if all adjacent vertices have edges from start
            for (const auto &e : out_edges_range(v, g)) {
                NFAVertex t = target(e, g);
                // skip self
                if (t == v) {
                    continue;
                }
                // skip vertices that don't have edges from start
                if (!edge(g.start, t, g).second) {
                    continue;
                }
                // add an edge from startDs to this vertex
                add_edge_if_not_present(g.startDs, t, g);

                // mark this edge for removal
                deadEdges.push_back(e);
            }
            // if the number of edges to be removed equals out degree, vertex
            // needs to be removed; else, only remove the edges
            if (deadEdges.size() == proper_out_degree(v, g)) {
                verticesToRemove.insert(v);
            } else {
                edgesToRemove.insert(deadEdges.begin(), deadEdges.end());
            }
        }
    }

    if (verticesToRemove.empty() && edgesToRemove.empty()) {
        return false;
    }

    DEBUG_PRINTF("removing %zu edges and %zu vertices\n", edgesToRemove.size(),
                 verticesToRemove.size());
    remove_edges(edgesToRemove, g);
    remove_vertices(verticesToRemove, g);
    /* some predecessors to the cyclic vertices may no longer be useful (no out
     * edges), so we can remove them */
    pruneUseless(g);
    return true;
}

struct PrunePathsInfo {
    explicit PrunePathsInfo(const NGHolder &g)
        : color_map(make_small_color_map(g)), bad(num_vertices(g)) {}

    void clear() {
        no_explore.clear();
        color_map.fill(small_color::white);
        bad.reset();
    }

    flat_set<NFAEdge> no_explore;
    using color_map_type = decltype(make_small_color_map(NGHolder()));
    color_map_type color_map;
    boost::dynamic_bitset<> bad;
};

/**
 * Finds the set of vertices that cannot be on if v is not on, setting their
 * indices in bitset PrunePathsInfo::bad.
 */
static
void findDependentVertices(const NGHolder &g, PrunePathsInfo &info,
                           NFAVertex v) {
    /* We need to exclude any vertex that may be reached on a path which is
     * incompatible with the vertex v being on. */

    /* A vertex u is bad if:
     * 1) its reach may be incompatible with v (not a subset)
     * 2) it if there is an edge from a bad vertex b and there is either not an
     *     edge v->u or not an edge b->v.
     * Note: 2) means v is never bad as it has a selfloop
     *
     * Can do this with a DFS from all the initial bad states with a conditional
     * check down edges. Alternately can just filter these edges out of the
     * graph first.
     */
    for (NFAVertex t : adjacent_vertices_range(v, g)) {
        for (NFAEdge e : in_edges_range(t, g)) {
            NFAVertex s = source(e, g);
            if (edge(s, v, g).second) {
                info.no_explore.insert(e);
            }
        }
    }

    auto filtered_g =
        make_filtered_graph(g, make_bad_edge_filter(&info.no_explore));

    // We use a bitset to track bad vertices, rather than filling a (potentially
    // very large) set structure.
    auto recorder = make_vertex_index_bitset_recorder(info.bad);

    for (NFAVertex b : vertices_range(g)) {
        if (b != g.start && g[b].char_reach.isSubsetOf(g[v].char_reach)) {
            continue;
        }
        boost::depth_first_visit(filtered_g, b, recorder, info.color_map);
    }
}

static
bool willBeEnabledConcurrently(NFAVertex main_cyclic, NFAVertex v,
                               const NGHolder &g) {
    return is_subset_of(preds(main_cyclic, g), preds(v, g));
}

static
bool sometimesEnabledConcurrently(NFAVertex main_cyclic, NFAVertex v,
                                  const NGHolder &g) {
    return has_intersection(preds(main_cyclic, g), preds(v, g));
}

static
bool pruneUsingSuccessors(NGHolder &g, PrunePathsInfo &info, NFAVertex u,
                          som_type som) {
    if (som && (is_virtual_start(u, g) || u == g.startDs)) {
        return false;
    }

    bool changed = false;
    DEBUG_PRINTF("using cyclic %zu as base\n", g[u].index);
    info.clear();
    findDependentVertices(g, info, u);
    vector<NFAVertex> u_succs;
    for (NFAVertex v : adjacent_vertices_range(u, g)) {
        if (som && is_virtual_start(v, g)) {
            /* as v is virtual start, its som has been reset so can not override
             * existing in progress matches. */
            continue;
        }
        u_succs.push_back(v);
    }

    stable_sort(u_succs.begin(), u_succs.end(),
         [&](NFAVertex a, NFAVertex b) {
             return g[a].char_reach.count() > g[b].char_reach.count();
         });

    flat_set<NFAEdge> dead;

    for (NFAVertex v : u_succs) {
        DEBUG_PRINTF("    using %zu as killer\n", g[v].index);
        /* Need to distinguish between vertices that are switched on after the
         * cyclic vs vertices that are switched on concurrently with the cyclic
         * if (subject to a suitable reach) */
        bool v_peer_of_cyclic = willBeEnabledConcurrently(u, v, g);
        for (NFAVertex s : adjacent_vertices_range(v, g)) {
            DEBUG_PRINTF("        looking at preds of %zu\n", g[s].index);
            for (NFAEdge e : in_edges_range(s, g)) {
                NFAVertex p = source(e, g);
                if (info.bad.test(g[p].index) || p == v || p == u
                    || p == g.accept) {
                    DEBUG_PRINTF("%zu not a cand\n", g[p].index);
                    continue;
                }
                if (is_any_accept(s, g) && g[p].reports != g[v].reports) {
                    DEBUG_PRINTF("%zu bad reports\n", g[p].index);
                    continue;
                }
                /* the out-edges of a vertex that may be enabled on the same
                 * byte as the cyclic can only be killed by the out-edges of a
                 * peer vertex which will be enabled with the cyclic (a non-peer
                 * may not be switched on until another byte is processed). */
                if (!v_peer_of_cyclic
                    && sometimesEnabledConcurrently(u, p, g)) {
                    DEBUG_PRINTF("%zu can only be squashed by a proper peer\n",
                                 g[p].index);
                   continue;
                }

                if (g[p].char_reach.isSubsetOf(g[v].char_reach)) {
                    dead.insert(e);
                    changed = true;
                    DEBUG_PRINTF("removing edge %zu->%zu\n", g[p].index,
                                  g[s].index);
                } else if (is_subset_of(succs(p, g), succs(u, g))) {
                    if (is_match_vertex(p, g)
                        && !is_subset_of(g[p].reports, g[v].reports)) {
                        continue;
                    }
                    DEBUG_PRINTF("updating reach on %zu\n", g[p].index);
                    changed |= (g[p].char_reach & g[v].char_reach).any();
                    g[p].char_reach &= ~g[v].char_reach;
                }

            }
        }
        remove_edges(dead, g);
        dead.clear();
    }

    DEBUG_PRINTF("changed %d\n", (int)changed);
    return changed;
}

bool prunePathsRedundantWithSuccessorOfCyclics(NGHolder &g, som_type som) {
    /* TODO: the reverse form of this is also possible */
    bool changed = false;
    PrunePathsInfo info(g);

    for (NFAVertex v : vertices_range(g)) {
        if (hasSelfLoop(v, g) && g[v].char_reach.all()) {
            changed |= pruneUsingSuccessors(g, info, v, som);
        }
    }

    if (changed) {
        pruneUseless(g);
        clearReports(g);
    }

    return changed;
}

} // namespace ue2
