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
 * \brief Utility functions related to SOM ("Start of Match").
 */
#include "ng_som_util.h"

#include "ng_depth.h"
#include "ng_execute.h"
#include "ng_holder.h"
#include "ng_prune.h"
#include "ng_util.h"
#include "util/container.h"
#include "util/graph_range.h"

using namespace std;

namespace ue2 {

static
void wireSuccessorsToStart(NGHolder &g, NFAVertex u) {
    for (auto v : adjacent_vertices_range(u, g)) {
        add_edge_if_not_present(g.start, v, g);
    }
}

vector<DepthMinMax> getDistancesFromSOM(const NGHolder &g_orig) {
    // We operate on a temporary copy of the original graph here, so we don't
    // have to mutate the original.
    NGHolder g;
    unordered_map<NFAVertex, NFAVertex> vmap; // vertex in g_orig to vertex in g
    cloneHolder(g, g_orig, &vmap);

    vector<NFAVertex> vstarts;
    for (auto v : vertices_range(g)) {
        if (is_virtual_start(v, g)) {
            vstarts.push_back(v);
        }
    }
    vstarts.push_back(g.startDs);

    // wire the successors of every virtual start or startDs to g.start.
    for (auto v : vstarts) {
        wireSuccessorsToStart(g, v);
    }

    // drop the in-edges of every virtual start so that they don't participate
    // in the depth calculation.
    for (auto v : vstarts) {
        clear_in_edges(v, g);
    }

    //dumpGraph("som_depth.dot", g);

    // Find depths, indexed by vertex index in g
    auto temp_depths = calcDepthsFrom(g, g.start);

    // Transfer depths, indexed by vertex index in g_orig.
    vector<DepthMinMax> depths(num_vertices(g_orig));

    for (auto v_orig : vertices_range(g_orig)) {
        assert(contains(vmap, v_orig));
        NFAVertex v_new = vmap[v_orig];

        u32 orig_idx = g_orig[v_orig].index;

        DepthMinMax &d = depths.at(orig_idx);

        if (v_orig == g_orig.startDs || is_virtual_start(v_orig, g_orig)) {
            // StartDs and virtual starts always have zero depth.
            d = DepthMinMax(depth(0), depth(0));
        } else {
            u32 new_idx = g[v_new].index;
            d = temp_depths.at(new_idx);
        }
    }

    return depths;
}

bool firstMatchIsFirst(const NGHolder &p) {
    /* If the first match (by end offset) is not the first match (by start
     * offset) then we can't create a lock after it.
     *
     * Consider: 4009:/(foobar|ob).*bugger/s
     *
     * We don't care about races on the last byte as they can be resolved easily
     * at runtime /(foobar|obar).*hi/
     *
     * It should be obvious we don't care about one match being a prefix
     * of another as they share the same start offset.
     *
     * Therefore, the case were we cannot establish that the som does not
     * regress is when there exists s1 and s2 in the language of p and s2 is a
     * proper infix of s1.
     *
     * It is tempting to add the further restriction that there does not exist a
     * prefix of s1 that is in the language of p (as in which case we would
     * presume, the lock has already been set). However, we have no way of
     * knowing if the lock can be cleared by some characters, and if so, if it
     * is still set. TODO: if we knew the lock's escapes where we could verify
     * that the rest of s1 does not clear the lock. (1)
     */

    DEBUG_PRINTF("entry\n");

    /* If there are any big cycles throw up our hands in despair */
    if (hasBigCycles(p)) {
        DEBUG_PRINTF("fail, big cycles\n");
        return false;
    }

    flat_set<NFAVertex> states;
    /* turn on all states (except starts - avoid suffix matches) */
    /* If we were doing (1) we would also except states leading to accepts -
       avoid prefix matches */
    for (auto v : vertices_range(p)) {
        assert(!is_virtual_start(v, p));
        if (!is_special(v, p)) {
            DEBUG_PRINTF("turning on %zu\n", p[v].index);
            states.insert(v);
        }
    }

    /* run the prefix the main graph */
    states = execute_graph(p, p, states);

    for (auto v : states) {
        /* need to check if this vertex may represent an infix match - ie
         * it does not have an edge to accept. */
        DEBUG_PRINTF("check %zu\n", p[v].index);
        if (!edge(v, p.accept, p).second) {
            DEBUG_PRINTF("fail %zu\n", p[v].index);
            return false;
        }
    }

    DEBUG_PRINTF("done first is first check\n");
    return true;
}

bool somMayGoBackwards(NFAVertex u, const NGHolder &g,
                       const unordered_map<NFAVertex, u32> &region_map,
                       smgb_cache &cache) {
    /* Need to ensure all matches of the graph g up to u contain no infixes
     * which are also matches of the graph to u.
     *
     * This is basically the same as firstMatchIsFirst except we g is not
     * always a dag. As we haven't gotten around to writing an execute_graph
     * that operates on general graphs, we take some (hopefully) conservative
     * short cuts.
     *
     * Note: if the u can be jumped we will take jump edges
     * into account as a possibility of som going backwards
     *
     * TODO: write a generalised ng_execute_graph/make this less hacky
     */
    assert(&g == &cache.g);
    if (contains(cache.smgb, u)) {
        return cache.smgb[u];
    }

    DEBUG_PRINTF("checking if som can go backwards on %zu\n", g[u].index);

    set<NFAEdge> be;
    BackEdges<set<NFAEdge>> backEdgeVisitor(be);
    boost::depth_first_search(g, visitor(backEdgeVisitor).root_vertex(g.start));

    bool rv;
    if (0) {
    exit:
        DEBUG_PRINTF("using cached result\n");
        cache.smgb[u] = rv;
        return rv;
    }

    assert(contains(region_map, u));
    const u32 u_region = region_map.at(u);

    for (const auto &e : be) {
        NFAVertex s = source(e, g);
        NFAVertex t = target(e, g);
        /* only need to worry about big cycles including/before u */
        DEBUG_PRINTF("back edge %zu %zu\n", g[s].index, g[t].index);
        if (s != t && region_map.at(s) <= u_region) {
            DEBUG_PRINTF("eek big cycle\n");
            rv = true; /* big cycle -> eek */
            goto exit;
        }
    }

    unordered_map<NFAVertex, NFAVertex> orig_to_copy;
    NGHolder c_g;
    cloneHolder(c_g, g, &orig_to_copy);

    /* treat virtual starts as unconditional - wire to startDs instead */
    for (NFAVertex v : vertices_range(g)) {
        if (!is_virtual_start(v, g)) {
            continue;
        }
        NFAVertex c_v = orig_to_copy[v];
        orig_to_copy[v] = c_g.startDs;
        for (NFAVertex c_w : adjacent_vertices_range(c_v, c_g)) {
            add_edge_if_not_present(c_g.startDs, c_w, c_g);
        }
        clear_vertex(c_v, c_g);
    }

    /* treat u as the only accept state */
    NFAVertex c_u = orig_to_copy[u];
    clear_in_edges(c_g.acceptEod, c_g);
    add_edge(c_g.accept, c_g.acceptEod, c_g);
    clear_in_edges(c_g.accept, c_g);
    clear_out_edges(c_u, c_g);
    if (hasSelfLoop(u, g)) {
        add_edge(c_u, c_u, c_g);
    }
    add_edge(c_u, c_g.accept, c_g);

    set<NFAVertex> u_succ;
    insert(&u_succ, adjacent_vertices(u, g));
    u_succ.erase(u);

    for (auto t : inv_adjacent_vertices_range(u, g)) {
        if (t == u) {
            continue;
        }
        for (auto v : adjacent_vertices_range(t, g)) {
            if (contains(u_succ, v)) {
                /* due to virtual starts being aliased with normal starts in the
                 * copy of the graph, we may have already added the edges. */
                add_edge_if_not_present(orig_to_copy[t], c_g.accept, c_g);
                break;
            }
        }
    }

    pruneUseless(c_g);

    be.clear();
    boost::depth_first_search(c_g, visitor(backEdgeVisitor)
                                   .root_vertex(c_g.start));

    for (const auto &e : be) {
        NFAVertex s = source(e, c_g);
        NFAVertex t = target(e, c_g);
        DEBUG_PRINTF("back edge %zu %zu\n", c_g[s].index, c_g[t].index);
        if (s != t) {
            assert(0);
            DEBUG_PRINTF("eek big cycle\n");
            rv = true; /* big cycle -> eek */
            goto exit;
        }
    }

    DEBUG_PRINTF("checking acyclic+selfloop graph\n");

    rv = !firstMatchIsFirst(c_g);
    DEBUG_PRINTF("som may regress? %d\n", (int)rv);
    goto exit;
}

bool sentClearsTail(const NGHolder &g,
                    const unordered_map<NFAVertex, u32> &region_map,
                    const NGHolder &sent, u32 last_head_region,
                    u32 *bad_region) {
    /* if a subsequent match from the prefix clears the rest of the pattern
     * we can just keep track of the last match of the prefix.
     * To see if this property holds, we could:
     *
     * 1A: turn on all states in the tail and run all strings that may
     *    match the prefix past the tail, if we are still in any states then
     *    this property does not hold.
     *
     * 1B: we turn on the initial states of the tail and run any strings which
     *   may finish any partial matches in the prefix and see if we end up with
     *   anything which would also imply that this property does not hold.
     *
     * OR
     *
     * 2: we just turn everything and run the prefix inputs past it and see what
     * we are left with. I think that is equivalent to scheme 1 and is easier to
     * implement. TODO: ponder
     *
     * Anyway, we are going with scheme 2 until further notice.
     */

    u32 first_bad_region = ~0U;
    flat_set<NFAVertex> states;
    /* turn on all states */
    DEBUG_PRINTF("region %u is cutover\n", last_head_region);
    for (auto v : vertices_range(g)) {
        if (v != g.accept && v != g.acceptEod) {
            states.insert(v);
        }
    }

    for (UNUSED auto v : states) {
        DEBUG_PRINTF("start state: %zu\n", g[v].index);
    }

    /* run the prefix the main graph */
    states = execute_graph(g, sent, states);

    /* .. and check if we are left with anything in the tail region */
    for (auto v : states) {
        if (v == g.start || v == g.startDs) {
            continue; /* not in tail */
        }

        DEBUG_PRINTF("v %zu is still on\n", g[v].index);
        assert(v != g.accept && v != g.acceptEod); /* no cr */

        assert(contains(region_map, v));
        const u32 v_region = region_map.at(v);
        if (v_region > last_head_region) {
            DEBUG_PRINTF("bailing, %u > %u\n", v_region, last_head_region);
            first_bad_region = min(first_bad_region, v_region);
        }
    }

    if (first_bad_region != ~0U) {
        DEBUG_PRINTF("first bad region is %u\n", first_bad_region);
        *bad_region = first_bad_region;
        return false;
    }

    return true;
}

} // namespace ue2
