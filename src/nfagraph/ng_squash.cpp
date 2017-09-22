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
 * \brief NFA graph state squashing analysis.
 *
 * The basic idea behind the state squashing is that when we are in a cyclic
 * state v there are certain other states which are completely irrelevant. This
 * is used primarily by the determinisation process to produce smaller DFAs by
 * not tracking irrelevant states. It's also used by the LimEx NFA model.
 *
 * Working out which states we can ignore mainly uses the post-dominator
 * analysis.
 *
 * ### Dot Squash Masks:
 *
 * The following vertices are added to the squash mask:
 * - (1) Any vertex post-dominated by the cyclic dot state
 * - (2) Any other vertex post-dominated by the cyclic dot state's successors
 * - (3) Any vertex post-dominated by a predecessor of the cyclic dot state -
 *   provided the predecessor's successors are a subset of the cyclic state's
 *   successors [For (3), the term successor also includes report information]
 *
 * (2) and (3) allow us to get squash masks from .* as well as .+
 *
 * The squash masks are not optimal especially in the case where there
 * alternations on both sides - for example in:
 *
 *     /foo(bar|baz).*(abc|xyz)/s
 *
 * 'foo' is irrelevant once the dot star is hit, but it has no post-dominators
 * so isn't picked up ('bar' and 'baz' are picked up by (2)). We may be able to
 * do a more complete analysis based on cutting the graph and seeing which
 * vertices are unreachable but the current approach is quick and probably
 * adequate.
 *
 *
 * ### Non-Dot Squash Masks:
 *
 * As for dot states. However, if anything in a pdom tree falls outside the
 * character range of the cyclic state the whole pdom tree is ignored. Also when
 * considering the predecessor's pdom tree it is necessary to verify that the
 * predecessor's character reachability falls within that of the cyclic state.
 *
 * We could do better in this case by not throwing away the whole pdom tree -
 * however the bits which we can keep are not clear from the pdom tree of the
 * cyclic state - it probably can be based on the dom or pdom tree of the bad
 * vertex.
 *
 * An example of us doing badly is:
 *
 *     /HTTP.*Referer[^\n]*google/s
 *
 * as '[\\n]*' doesn't get a squash mask at all due to .* but we should be able
 * to squash 'Referer'.
 *
 * ### Extension:
 *
 * If a state leads solely to a squashable state (or its immediate successors)
 * with the same reachability we can make this state a squash state of any of
 * the original states squashees which we postdominate. Could probably tighten
 * this up but it would require thought. May not need to keep the original
 * squasher around but that would also require thought.
 *
 * ### SOM Notes:
 *
 * If (left) start of match is required, it is illegal to squash any state which
 * may result in an early start of match reaching the squashing state.
 */

#include "config.h"

#include "ng_squash.h"

#include "ng_dominators.h"
#include "ng_dump.h"
#include "ng_holder.h"
#include "ng_prune.h"
#include "ng_region.h"
#include "ng_som_util.h"
#include "ng_util.h"
#include "util/container.h"
#include "util/graph_range.h"
#include "util/report_manager.h"
#include "ue2common.h"

#include <deque>
#include <map>
#include <unordered_map>
#include <unordered_set>

#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/reverse_graph.hpp>

using namespace std;

namespace ue2 {

using PostDomTree = unordered_map<NFAVertex, unordered_set<NFAVertex>>;

static
PostDomTree buildPDomTree(const NGHolder &g) {
    PostDomTree tree;
    tree.reserve(num_vertices(g));

    auto postdominators = findPostDominators(g);

    for (auto v : vertices_range(g)) {
        if (is_special(v, g)) {
            continue;
        }
        NFAVertex pdom = postdominators[v];
        if (pdom) {
            DEBUG_PRINTF("vertex %zu -> %zu\n", g[pdom].index, g[v].index);
            tree[pdom].insert(v);
        }
    }
    return tree;
}

/**
 * Builds a squash mask based on the pdom tree of v and the given char reach.
 * The built squash mask is a bit conservative for non-dot cases and could
 * be improved with a bit of thought.
 */
static
void buildSquashMask(NFAStateSet &mask, const NGHolder &g, NFAVertex v,
                     const CharReach &cr, const NFAStateSet &init,
                     const vector<NFAVertex> &vByIndex, const PostDomTree &tree,
                     som_type som, const vector<DepthMinMax> &som_depths,
                     const unordered_map<NFAVertex, u32> &region_map,
                     smgb_cache &cache) {
    DEBUG_PRINTF("build base squash mask for vertex %zu)\n", g[v].index);

    vector<NFAVertex> q;

    auto it = tree.find(v);
    if (it != tree.end()) {
        q.insert(q.end(), it->second.begin(), it->second.end());
    }

    const u32 v_index = g[v].index;

    while (!q.empty()) {
        NFAVertex u = q.back();
        q.pop_back();
        const CharReach &cru = g[u].char_reach;

        if ((cru & ~cr).any()) {
            /* bail: bad cr on vertex u */
            /* TODO: this could be better
             *
             * we still need to ensure that we record any paths leading to u.
             * Hence all vertices R which can reach u must be excluded from the
             * squash mask. Note: R != pdom(u) and there may exist an x in (R -
             * pdom(u)) which is in pdom(y) where y is in q. Clear ?
             */
            mask.set();
            return;
        }

        const u32 u_index = g[u].index;

        if (som) {
            /* We cannot add a state u to the squash mask of v if it may have an
             * earlier start of match offset. ie for us to add a state u to v
             * maxSomDist(u) <= minSomDist(v)
             */
            const depth &max_som_dist_u = som_depths[u_index].max;
            const depth &min_som_dist_v = som_depths[v_index].min;

            if (max_som_dist_u.is_infinite()) {
                /* it is hard to tell due to the INF if u can actually store an
                 * earlier SOM than w (state we are building the squash mask
                 * for) - need to think more deeply
                 */

                if (mustBeSetBefore(u, v, g, cache)
                    && !somMayGoBackwards(u, g, region_map, cache)) {
                    DEBUG_PRINTF("u %u v %u\n", u_index, v_index);
                    goto squash_ok;
                }
            }

           if (max_som_dist_u > min_som_dist_v) {
                /* u can't be squashed as it may be storing an earlier SOM */
                goto add_children_to_queue;
            }

        }

    squash_ok:
        mask.set(u_index);
        DEBUG_PRINTF("pdom'ed %u\n", u_index);
    add_children_to_queue:
        it = tree.find(u);
        if (it != tree.end()) {
            q.insert(q.end(), it->second.begin(), it->second.end());
        }
    }

    if (cr.all()) {
        /* the init states aren't in the pdom tree. If all their succ states
         * are set (or v), we can consider them post dominated */

        /* Note: init states will always result in a later som */
        for (size_t i = init.find_first(); i != init.npos;
             i = init.find_next(i)) {
            /* Yes vacuous patterns do exist */
            NFAVertex iv = vByIndex[i];
            for (auto w : adjacent_vertices_range(iv, g)) {
                if (w == g.accept || w == g.acceptEod) {
                    DEBUG_PRINTF("skipping %zu due to vacuous accept\n", i);
                    goto next_init_state;
                }

                u32 vert_id = g[w].index;
                if (w != iv && w != v && !mask.test(vert_id)) {
                    DEBUG_PRINTF("skipping %zu due to %u\n", i, vert_id);
                    goto next_init_state;
                }
            }
            DEBUG_PRINTF("pdom'ed %zu\n", i);
            mask.set(i);
        next_init_state:;
        }
    }

    mask.flip();
}

static
void buildSucc(NFAStateSet &succ, const NGHolder &g, NFAVertex v) {
    for (auto w : adjacent_vertices_range(v, g)) {
        if (!is_special(w, g)) {
            succ.set(g[w].index);
        }
    }
}

static
void buildPred(NFAStateSet &pred, const NGHolder &g, NFAVertex v) {
    for (auto u : inv_adjacent_vertices_range(v, g)) {
        if (!is_special(u, g)) {
            pred.set(g[u].index);
        }
    }
}

static
void findDerivedSquashers(const NGHolder &g, const vector<NFAVertex> &vByIndex,
                          const PostDomTree &pdom_tree, const NFAStateSet &init,
                          unordered_map<NFAVertex, NFAStateSet> *squash,
                          som_type som, const vector<DepthMinMax> &som_depths,
                          const unordered_map<NFAVertex, u32> &region_map,
                          smgb_cache &cache) {
    deque<NFAVertex> remaining;
    for (const auto &m : *squash) {
        remaining.push_back(m.first);
    }

    while (!remaining.empty()) {
        NFAVertex v = remaining.back();
        remaining.pop_back();

        for (auto u : inv_adjacent_vertices_range(v, g)) {
            if (is_special(u, g)) {
                continue;
            }

            if (g[v].char_reach != g[u].char_reach) {
                continue;
            }

            if (out_degree(u, g) != 1) {
                continue;
            }

            NFAStateSet u_squash(init.size());
            size_t u_index = g[u].index;

            buildSquashMask(u_squash, g, u, g[u].char_reach, init, vByIndex,
                            pdom_tree, som, som_depths, region_map, cache);

            u_squash.set(u_index); /* never clear ourselves */

            if ((~u_squash).any()) { // i.e. some bits unset in mask
                DEBUG_PRINTF("%zu is an upstream squasher of %zu\n", u_index,
                             g[v].index);
                (*squash)[u] = u_squash;
                remaining.push_back(u);
            }
        }
    }
}

/* If there are redundant states in the graph, it may be possible for two
 * sibling .* states to try to squash each other -- which should be prevented.
 *
 * Note: this situation should only happen if ng_equivalence has not been run.
 */
static
void clearMutualSquashers(const NGHolder &g, const vector<NFAVertex> &vByIndex,
                          unordered_map<NFAVertex, NFAStateSet> &squash) {
    for (auto it = squash.begin(); it != squash.end();) {
        NFAVertex a = it->first;
        u32 a_index = g[a].index;

        NFAStateSet a_squash = ~it->second;  /* default is mask of survivors */
        for (auto b_index = a_squash.find_first(); b_index != a_squash.npos;
             b_index = a_squash.find_next(b_index)) {
            assert(b_index != a_index);
            NFAVertex b = vByIndex[b_index];

            auto b_it = squash.find(b);
            if (b_it == squash.end()) {
                continue;
            }
            auto &b_squash = b_it->second;
            if (!b_squash.test(a_index)) {
                /* b and a squash each other, prevent this */
                DEBUG_PRINTF("removing mutual squash %u %zu\n",
                             a_index, b_index);
                b_squash.set(a_index);
                it->second.set(b_index);
            }
        }

        if (it->second.all()) {
            DEBUG_PRINTF("%u is no longer an effective squash state\n",
                         a_index);
            it = squash.erase(it);
        } else {
            ++it;
        }
    }
}

unordered_map<NFAVertex, NFAStateSet> findSquashers(const NGHolder &g,
                                                    som_type som) {
    unordered_map<NFAVertex, NFAStateSet> squash;

    // Number of bits to use for all our masks. If we're a triggered graph,
    // tops have already been assigned, so we don't have to account for them.
    const u32 numStates = num_vertices(g);

    // Build post-dominator tree.
    auto pdom_tree = buildPDomTree(g);

    // Build list of vertices by state ID and a set of init states.
    vector<NFAVertex> vByIndex(numStates, NGHolder::null_vertex());
    NFAStateSet initStates(numStates);
    smgb_cache cache(g);

    // Mappings used for SOM mode calculations, otherwise left empty.
    unordered_map<NFAVertex, u32> region_map;
    vector<DepthMinMax> som_depths;
    if (som) {
        region_map = assignRegions(g);
        som_depths = getDistancesFromSOM(g);
    }

    for (auto v : vertices_range(g)) {
        const u32 vert_id = g[v].index;
        DEBUG_PRINTF("vertex %u/%u\n", vert_id, numStates);
        assert(vert_id < numStates);
        vByIndex[vert_id] = v;

        if (is_any_start(v, g) || !in_degree(v, g)) {
            initStates.set(vert_id);
        }
    }

    for (u32 i = 0; i < numStates; i++) {
        NFAVertex v = vByIndex[i];
        assert(v != NGHolder::null_vertex());
        const CharReach &cr = g[v].char_reach;

        /* only non-init cyclics can be squashers */
        if (!hasSelfLoop(v, g) || initStates.test(i)) {
            continue;
        }

        DEBUG_PRINTF("state %u is cyclic\n", i);

        NFAStateSet mask(numStates), succ(numStates), pred(numStates);
        buildSquashMask(mask, g, v, cr, initStates, vByIndex, pdom_tree, som,
                        som_depths, region_map, cache);
        buildSucc(succ, g, v);
        buildPred(pred, g, v);
        const auto &reports = g[v].reports;

        for (size_t j = succ.find_first(); j != succ.npos;
             j = succ.find_next(j)) {
            NFAVertex vj = vByIndex[j];
            NFAStateSet pred2(numStates);
            buildPred(pred2, g, vj);
            if (pred2 == pred) {
                DEBUG_PRINTF("adding the sm from %zu to %u's sm\n", j, i);
                NFAStateSet tmp(numStates);
                buildSquashMask(tmp, g, vj, cr, initStates, vByIndex, pdom_tree,
                                som, som_depths, region_map, cache);
                mask &= tmp;
            }
        }

        for (size_t j = pred.find_first(); j != pred.npos;
             j = pred.find_next(j)) {
            NFAVertex vj = vByIndex[j];
            NFAStateSet succ2(numStates);
            buildSucc(succ2, g, vj);
            /* we can use j as a basis for squashing if its succs are a subset
             * of ours */
            if ((succ2 & ~succ).any()) {
                continue;
            }

            if (som) {
                /* We cannot use j to add to the squash mask of v if it may
                 * have an earlier start of match offset. ie for us j as a
                 * basis for the squash mask of v we require:
                 * maxSomDist(j) <= minSomDist(v)
                 */

                /* ** TODO ** */

                const depth &max_som_dist_j =
                    som_depths[g[vj].index].max;
                const depth &min_som_dist_v =
                    som_depths[g[v].index].min;
                if (max_som_dist_j > min_som_dist_v ||
                    max_som_dist_j.is_infinite()) {
                    /* j can't be used as it may be storing an earlier SOM */
                    continue;
                }
            }

            const CharReach &crv = g[vj].char_reach;

            /* we also require that j's report information be a subset of ours
             */
            bool seen_special = false;
            for (auto w : adjacent_vertices_range(vj, g)) {
                if (is_special(w, g)) {
                    if (!edge(v, w, g).second) {
                        goto next_j;
                    }
                    seen_special = true;
                }
            }

            // FIXME: should be subset check?
            if (seen_special && g[vj].reports != reports) {
                continue;
            }

            /* ok we can use j */
            if ((crv & ~cr).none()) {
                NFAStateSet tmp(numStates);
                buildSquashMask(tmp, g, vj, cr, initStates, vByIndex, pdom_tree,
                                som, som_depths, region_map, cache);
                mask &= tmp;
                mask.reset(j);
            }

        next_j:;
        }

        mask.set(i); /* never clear ourselves */

        if ((~mask).any()) { // i.e. some bits unset in mask
            DEBUG_PRINTF("%u squashes %zu other states\n", i, (~mask).count());
            squash.emplace(v, mask);
        }
    }

    findDerivedSquashers(g, vByIndex, pdom_tree, initStates, &squash, som,
                         som_depths, region_map, cache);

    clearMutualSquashers(g, vByIndex, squash);

    return squash;
}

#define MIN_PURE_ACYCLIC_SQUASH 10 /** magic number */

/** Some squash states are clearly not advantageous in the NFA, as they do
 * incur the cost of an exception:
 * -# acyclic states
 * -# squash only a few acyclic states
 */
void filterSquashers(const NGHolder &g,
                     unordered_map<NFAVertex, NFAStateSet> &squash) {
    assert(hasCorrectlyNumberedVertices(g));

    DEBUG_PRINTF("filtering\n");
    vector<NFAVertex> rev(num_vertices(g)); /* vertex_index -> vertex */
    for (auto v : vertices_range(g)) {
        rev[g[v].index] = v;
    }

    for (auto v : vertices_range(g)) {
        if (!contains(squash, v)) {
            continue;
        }
        DEBUG_PRINTF("looking at squash set for vertex %zu\n", g[v].index);

        if (!hasSelfLoop(v, g)) {
            DEBUG_PRINTF("acyclic\n");
            squash.erase(v);
            continue;
        }

        NFAStateSet squashed = squash[v];
        squashed.flip(); /* default sense for mask of survivors */
        for (auto sq = squashed.find_first(); sq != squashed.npos;
             sq = squashed.find_next(sq)) {
            NFAVertex u = rev[sq];
            if (hasSelfLoop(u, g)) {
                DEBUG_PRINTF("squashing a cyclic (%zu) is always good\n", sq);
                goto next_vertex;
            }
        }

        if (squashed.count() < MIN_PURE_ACYCLIC_SQUASH) {
            DEBUG_PRINTF("squash set too small\n");
            squash.erase(v);
            continue;
        }

    next_vertex:;
        DEBUG_PRINTF("squash set ok\n");
    }
}

static
void getHighlanderReporters(const NGHolder &g, const NFAVertex accept,
                            const ReportManager &rm,
                            set<NFAVertex> &verts) {
    for (auto v : inv_adjacent_vertices_range(accept, g)) {
        if (v == g.accept) {
            continue;
        }

        const auto &reports = g[v].reports;
        if (reports.empty()) {
            assert(0);
            continue;
        }

        // Must be _all_ highlander callback reports.
        for (auto report : reports) {
            const Report &ir = rm.getReport(report);
            if (ir.ekey == INVALID_EKEY || ir.type != EXTERNAL_CALLBACK) {
                goto next_vertex;
            }

            // If there's any bounds, these are handled outside the NFA and
            // probably shouldn't be pre-empted.
            if (ir.hasBounds()) {
                goto next_vertex;
            }
        }

        verts.insert(v);
    next_vertex:
        continue;
    }
}

static
void removeEdgesToAccept(NGHolder &g, NFAVertex v) {
    const auto &reports = g[v].reports;
    assert(!reports.empty());

    // We remove any accept edge with a non-empty subset of the reports of v.

    set<NFAEdge> dead;

    for (const auto &e : in_edges_range(g.accept, g)) {
        NFAVertex u = source(e, g);
        const auto &r = g[u].reports;
        if (!r.empty() && is_subset_of(r, reports)) {
            DEBUG_PRINTF("vertex %zu\n", g[u].index);
            dead.insert(e);
        }
    }

    for (const auto &e : in_edges_range(g.acceptEod, g)) {
        NFAVertex u = source(e, g);
        const auto &r = g[u].reports;
        if (!r.empty() && is_subset_of(r, reports)) {
            DEBUG_PRINTF("vertex %zu\n", g[u].index);
            dead.insert(e);
        }
    }

    assert(!dead.empty());
    remove_edges(dead, g);
}

static
vector<NFAVertex> findUnreachable(const NGHolder &g) {
    const boost::reverse_graph<NGHolder, const NGHolder &> revg(g);

    unordered_map<NFAVertex, boost::default_color_type> colours;
    colours.reserve(num_vertices(g));

    depth_first_visit(revg, g.acceptEod,
                      make_dfs_visitor(boost::null_visitor()),
                      make_assoc_property_map(colours));

    // Unreachable vertices are not in the colour map.
    vector<NFAVertex> unreach;
    for (auto v : vertices_range(revg)) {
        if (!contains(colours, v)) {
            unreach.push_back(NFAVertex(v));
        }
    }
    return unreach;
}

/** Populates squash masks for states that can be switched off by highlander
 * (single match) reporters. */
unordered_map<NFAVertex, NFAStateSet>
findHighlanderSquashers(const NGHolder &g, const ReportManager &rm) {
    unordered_map<NFAVertex, NFAStateSet> squash;

    set<NFAVertex> verts;
    getHighlanderReporters(g, g.accept, rm, verts);
    getHighlanderReporters(g, g.acceptEod, rm, verts);
    if (verts.empty()) {
        DEBUG_PRINTF("no highlander reports\n");
        return squash;
    }

    const u32 numStates = num_vertices(g);

    for (auto v : verts) {
        DEBUG_PRINTF("vertex %zu with %zu reports\n", g[v].index,
                     g[v].reports.size());

        // Find the set of vertices that lead to v or any other reporter with a
        // subset of v's reports. We do this by creating a copy of the graph,
        // cutting the appropriate out-edges to accept and seeing which
        // vertices become unreachable.

        unordered_map<NFAVertex, NFAVertex> orig_to_copy;
        NGHolder h;
        cloneHolder(h, g, &orig_to_copy);
        removeEdgesToAccept(h, orig_to_copy[v]);

        vector<NFAVertex> unreach = findUnreachable(h);
        DEBUG_PRINTF("can squash %zu vertices\n", unreach.size());
        if (unreach.empty()) {
            continue;
        }

        if (!contains(squash, v)) {
            squash[v] = NFAStateSet(numStates);
            squash[v].set();
        }

        NFAStateSet &mask = squash[v];

        for (auto uv : unreach) {
            DEBUG_PRINTF("squashes index %zu\n", h[uv].index);
            mask.reset(h[uv].index);
        }
    }

    return squash;
}

} // namespace ue2
