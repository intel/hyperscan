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
 * \brief Functions for splitting NFAGraphs into LHS and RHS.
 */
#include "ng_split.h"

#include "ng_holder.h"
#include "ng_prune.h"
#include "ng_util.h"
#include "util/container.h"
#include "util/graph.h"
#include "util/graph_range.h"

#include <map>
#include <set>
#include <vector>

using namespace std;

namespace ue2 {

static
void clearAccepts(NGHolder &g) {
    for (auto v : inv_adjacent_vertices_range(g.accept, g)) {
        g[v].reports.clear();
    }

    for (auto v : inv_adjacent_vertices_range(g.acceptEod, g)) {
        g[v].reports.clear();
    }

    clear_in_edges(g.accept, g);
    clear_in_edges(g.acceptEod, g);
    add_edge(g.accept, g.acceptEod, g);
}

static
void filterSplitMap(const NGHolder &g,
                    unordered_map<NFAVertex, NFAVertex> *out_map) {
    unordered_set<NFAVertex> verts;
    insert(&verts, vertices(g));
    auto it = out_map->begin();
    while (it != out_map->end()) {
        auto jt = it;
        ++it;
        if (!contains(verts, jt->second)) {
            out_map->erase(jt);
        }
    }
}

static
void splitLHS(const NGHolder &base, const vector<NFAVertex> &pivots,
              const vector<NFAVertex> &rhs_pivots, NGHolder *lhs,
              unordered_map<NFAVertex, NFAVertex> *lhs_map) {
    assert(lhs && lhs_map);

    cloneHolder(*lhs, base, lhs_map);

    clearAccepts(*lhs);

    for (auto pivot : pivots) {
        DEBUG_PRINTF("pivot is %zu lv %zu lm %zu\n", base[pivot].index,
                     num_vertices(*lhs), lhs_map->size());
        assert(contains(*lhs_map, pivot));

        for (auto v : rhs_pivots) {
            assert(contains(*lhs_map, v));
            remove_edge((*lhs_map)[pivot], (*lhs_map)[v], *lhs);
        }

        (*lhs)[(*lhs_map)[pivot]].reports.insert(0);
        add_edge((*lhs_map)[pivot], lhs->accept, *lhs);
    }

    /* should do the renumbering unconditionally as we know edges are already
     * misnumbered */
    pruneUseless(*lhs, false);
    renumber_edges(*lhs);
    renumber_vertices(*lhs);

    filterSplitMap(*lhs, lhs_map);

    switch (base.kind) {
    case NFA_PREFIX:
    case NFA_OUTFIX:
        lhs->kind = NFA_PREFIX;
        break;
    case NFA_INFIX:
    case NFA_SUFFIX:
        lhs->kind = NFA_INFIX;
        break;
    case NFA_EAGER_PREFIX:
        /* Current code should not be assigning eager until well after all the
         * splitting is done. */
        assert(0);
        lhs->kind = NFA_EAGER_PREFIX;
        break;
    case NFA_REV_PREFIX:
    case NFA_OUTFIX_RAW:
        assert(0);
        break;
    }
}

void splitLHS(const NGHolder &base, NFAVertex pivot,
              NGHolder *lhs, unordered_map<NFAVertex, NFAVertex> *lhs_map) {
    vector<NFAVertex> pivots(1, pivot);
    vector<NFAVertex> rhs_pivots;
    insert(&rhs_pivots, rhs_pivots.end(), adjacent_vertices(pivot, base));
    splitLHS(base, pivots, rhs_pivots, lhs, lhs_map);
}

void splitRHS(const NGHolder &base, const vector<NFAVertex> &pivots,
              NGHolder *rhs, unordered_map<NFAVertex, NFAVertex> *rhs_map) {
    assert(rhs && rhs_map);

    cloneHolder(*rhs, base, rhs_map);

    clear_out_edges(rhs->start, *rhs);
    clear_out_edges(rhs->startDs, *rhs);
    add_edge(rhs->start, rhs->startDs, *rhs);
    add_edge(rhs->startDs, rhs->startDs, *rhs);

    for (auto pivot : pivots) {
        assert(contains(*rhs_map, pivot));
        NFAEdge e = add_edge(rhs->start, (*rhs_map)[pivot], *rhs);
        (*rhs)[e].tops.insert(DEFAULT_TOP);
    }

     /* should do the renumbering unconditionally as we know edges are already
      * misnumbered */
    pruneUseless(*rhs, false);
    renumber_edges(*rhs);
    renumber_vertices(*rhs);
    filterSplitMap(*rhs, rhs_map);

    switch (base.kind) {
    case NFA_PREFIX:
    case NFA_INFIX:
        rhs->kind = NFA_INFIX;
        break;
    case NFA_SUFFIX:
    case NFA_OUTFIX:
        rhs->kind = NFA_SUFFIX;
        break;
    case NFA_EAGER_PREFIX:
        /* Current code should not be assigning eager until well after all the
         * splitting is done. */
        assert(0);
        rhs->kind = NFA_INFIX;
        break;
    case NFA_REV_PREFIX:
    case NFA_OUTFIX_RAW:
        assert(0);
        break;
    }
}

/** \brief Fills \a succ with the common successors of the vertices in \a
 * pivots. */
static
void findCommonSuccessors(const NGHolder &g, const vector<NFAVertex> &pivots,
                          vector<NFAVertex> &succ) {
    assert(!pivots.empty());

    set<NFAVertex> adj;
    set<NFAVertex> adj_temp;

    insert(&adj, adjacent_vertices(pivots.at(0), g));

    for (auto it = pivots.begin() + 1, ite = pivots.end(); it != ite; ++it) {
        NFAVertex pivot = *it;
        adj_temp.clear();
        for (auto v : adjacent_vertices_range(pivot, g)) {
            if (contains(adj, v)) {
                adj_temp.insert(v);
            }
        }
        adj.swap(adj_temp);
    }

    succ.insert(succ.end(), adj.begin(), adj.end());
}

void splitGraph(const NGHolder &base, const vector<NFAVertex> &pivots,
                NGHolder *lhs, unordered_map<NFAVertex, NFAVertex> *lhs_map,
                NGHolder *rhs, unordered_map<NFAVertex, NFAVertex> *rhs_map) {
    DEBUG_PRINTF("splitting graph at %zu vertices\n", pivots.size());

    assert(!has_parallel_edge(base));
    assert(isCorrectlyTopped(base));

    /* RHS pivots are built from the common set of successors of pivots. */
    vector<NFAVertex> rhs_pivots;
    findCommonSuccessors(base, pivots, rhs_pivots);

    /* generate lhs */
    splitLHS(base, pivots, rhs_pivots, lhs, lhs_map);

    /* generate the rhs */
    splitRHS(base, rhs_pivots, rhs, rhs_map);

    assert(!has_parallel_edge(*lhs));
    assert(!has_parallel_edge(*rhs));
    assert(isCorrectlyTopped(*lhs));
    assert(isCorrectlyTopped(*rhs));
}

void splitGraph(const NGHolder &base, NFAVertex pivot,
                NGHolder *lhs, unordered_map<NFAVertex, NFAVertex> *lhs_map,
                NGHolder *rhs, unordered_map<NFAVertex, NFAVertex> *rhs_map) {
    vector<NFAVertex> pivots(1, pivot);
    splitGraph(base, pivots, lhs, lhs_map, rhs, rhs_map);
}

} // namespace ue2
