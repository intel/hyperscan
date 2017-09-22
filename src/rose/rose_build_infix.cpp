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

#include "rose/rose_build_infix.h"

#include "ue2common.h"
#include "nfa/castlecompile.h"
#include "nfagraph/ng_dump.h"
#include "nfagraph/ng_width.h"
#include "nfagraph/ng_util.h"
#include "rose/rose_build_impl.h"
#include "util/container.h"
#include "util/dump_charclass.h"
#include "util/flat_containers.h"
#include "util/graph_range.h"
#include "util/graph.h"
#include "util/hash.h"
#include "util/ue2string.h"
#include "util/unordered.h"

#include <algorithm>
#include <set>

using namespace std;

namespace ue2 {

static
bool couldEndLiteral(const ue2_literal &s, NFAVertex initial,
                     const NGHolder &h) {
    flat_set<NFAVertex> curr, next;
    curr.insert(initial);

    for (auto it = s.rbegin(), ite = s.rend(); it != ite; ++it) {
        const CharReach &cr_s = *it;
        bool matched = false;
        next.clear();

        for (auto v : curr) {
            if (v == h.start) {
                // We can't see what we had before the start, so we must assume
                // the literal could overlap with it.
                return true;
            }
            const CharReach &cr_v = h[v].char_reach;
            if (overlaps(cr_v, cr_s)) {
                insert(&next, inv_adjacent_vertices(v, h));
                matched = true;
            }
        }

        if (!matched) {
            return false;
        }

        curr.swap(next);
    }

    return true;
}

using EdgeCache = ue2_unordered_set<pair<NFAVertex, NFAVertex>>;

static
void contractVertex(NGHolder &g, NFAVertex v, EdgeCache &all_edges) {
    for (auto u : inv_adjacent_vertices_range(v, g)) {
        if (u == v) {
            continue; // self-edge
        }
        for (auto w : adjacent_vertices_range(v, g)) {
            if (w == v) {
                continue; // self-edge
            }

            // Construct edge (u, v) only if it doesn't already exist. We use
            // the all_edges container here, as checking existence inside the
            // graph is expensive when u or v have large degree.
            if (all_edges.emplace(u, w).second) {
                add_edge(u, w, g);
            }
        }
    }

    // Note that edges to/from v will remain in all_edges.
    clear_vertex(v, g);
}

static
u32 findMaxLiteralMatches(const NGHolder &h, const set<ue2_literal> &lits) {
    DEBUG_PRINTF("h=%p, %zu literals\n", &h, lits.size());
    //dumpGraph("infix.dot", h);

    // Indices of vertices that could terminate any of the literals in 'lits'.
    set<u32> terms;

    for (const auto &s : lits) {
        DEBUG_PRINTF("lit s='%s'\n", escapeString(s).c_str());
        if (s.empty()) {
            // Likely an anchored case, be conservative here.
            return NO_MATCH_LIMIT;
        }

        for (auto v : vertices_range(h)) {
            if (is_special(v, h)) {
                continue;
            }

            if (couldEndLiteral(s, v, h)) {
                u32 idx = h[v].index;
                DEBUG_PRINTF("vertex %u could terminate lit\n", idx);
                terms.insert(idx);
            }
        }
    }

    if (terms.empty()) {
        DEBUG_PRINTF("literals cannot match inside infix\n");
        return 0;
    }

    NGHolder g;
    cloneHolder(g, h);
    vector<NFAVertex> dead;

    // The set of all edges in the graph is used for existence checks in
    // contractVertex.
    EdgeCache all_edges;
    for (const auto &e : edges_range(g)) {
        all_edges.emplace(source(e, g), target(e, g));
    }

    for (auto v : vertices_range(g)) {
        if (is_special(v, g)) {
            continue;
        }
        if (contains(terms, g[v].index)) {
            continue;
        }

        contractVertex(g, v, all_edges);
        dead.push_back(v);
    }

    remove_vertices(dead, g);
    //dumpGraph("relaxed.dot", g);

    depth maxWidth = findMaxWidth(g);
    DEBUG_PRINTF("maxWidth=%s\n", maxWidth.str().c_str());
    assert(maxWidth.is_reachable());

    if (maxWidth.is_infinite()) {
        // Cycle detected, so we can likely squeeze an unlimited number of
        // matches into this graph.
        return NO_MATCH_LIMIT;
    }

    assert(terms.size() >= maxWidth);
    return maxWidth;
}

namespace {
struct ReachMismatch {
    explicit ReachMismatch(const CharReach &cr_in) : cr(cr_in) {}
    bool operator()(const CharReach &a) const { return !overlaps(cr, a); }

private:
    CharReach cr;
};
}

static
u32 findMaxInfixMatches(const CastleProto &castle,
                        const set<ue2_literal> &lits) {
    DEBUG_PRINTF("castle=%p, %zu literals\n", &castle, lits.size());

    if (castle.repeats.size() > 1) {
        DEBUG_PRINTF("more than one top!\n");
        return NO_MATCH_LIMIT;
    }

    assert(!castle.repeats.empty());
    const PureRepeat &pr = castle.repeats.begin()->second;
    DEBUG_PRINTF("repeat=%s reach=%s\n", pr.bounds.str().c_str(),
                 describeClass(pr.reach).c_str());

    size_t max_count = 0;

    for (const auto &s : lits) {
        DEBUG_PRINTF("lit s='%s'\n", escapeString(s).c_str());
        if (s.empty()) {
            // Likely an anchored case, be conservative here.
            return NO_MATCH_LIMIT;
        }

        size_t count = 0;

        auto f = find_if(s.rbegin(), s.rend(), ReachMismatch(pr.reach));

        if (f == s.rbegin()) {
            DEBUG_PRINTF("lit can't terminate inside infix\n");
            count = 0;
        } else if (f != s.rend()) {
            size_t suffix_len = distance(s.rbegin(), f);
            DEBUG_PRINTF("suffix of len %zu matches at start\n", suffix_len);
            if (pr.bounds.max.is_finite()) {
                count = min(suffix_len, (size_t)pr.bounds.max);
            } else {
                count = suffix_len;
            }
        } else {
            DEBUG_PRINTF("whole lit can match inside infix (repeatedly)\n");
            if (pr.bounds.max.is_finite()) {
                count = pr.bounds.max;
            } else {
                DEBUG_PRINTF("inf bound\n");
                return NO_MATCH_LIMIT;
            }
        }

        DEBUG_PRINTF("count=%zu\n", count);
        max_count = max(max_count, count);
    }

    DEBUG_PRINTF("max_count %zu\n", max_count);

    if (max_count > NO_MATCH_LIMIT) {
        assert(0); // This would be a surprise.
        return NO_MATCH_LIMIT;
    }

    return (u32)max_count;
}

u32 findMaxInfixMatches(const left_id &left, const set<ue2_literal> &lits) {
    if (left.castle()) {
        return findMaxInfixMatches(*left.castle(), lits);
    }
    if (left.graph()) {
        if (!onlyOneTop(*left.graph())) {
            DEBUG_PRINTF("more than one top!n");
            return NO_MATCH_LIMIT;
        }
        return findMaxLiteralMatches(*left.graph(), lits);
    }

    return NO_MATCH_LIMIT;
}

void findCountingMiracleInfo(const left_id &left, const vector<u8> &stopTable,
                             u8 *cm_count, CharReach *cm_cr) {
    DEBUG_PRINTF("hello\n");
    *cm_count = 0;
    cm_cr->clear();
    if (!left.graph()) {
        return;
    }

    const NGHolder &g = *left.graph();

    auto cyclics = find_vertices_in_cycles(g);

    if (!proper_out_degree(g.startDs, g)) {
        cyclics.erase(g.startDs);
    }

    CharReach cyclic_cr;
    for (NFAVertex v : cyclics) {
        DEBUG_PRINTF("considering %zu ||=%zu\n", g[v].index,
                      g[v].char_reach.count());
        cyclic_cr |= g[v].char_reach;
    }

    if (cyclic_cr.none() || cyclic_cr.all()) {
        DEBUG_PRINTF("cyclic cr width %zu\n", cyclic_cr.count());
        return; /* useless */
    }

    *cm_cr = ~cyclic_cr;

    /* stop character will be part of normal miracles, no need to look for them
     * here too */
    assert(stopTable.size() == N_CHARS);
    for (u32 i = 0; i < N_CHARS; i++) {
        if (stopTable[i]) {
            cm_cr->clear(i);
        }
    }

    set<ue2_literal> lits;
    for (size_t c = cm_cr->find_first(); c != CharReach::npos;
         c = cm_cr->find_next(c)) {
        DEBUG_PRINTF("considering %hhx as stop character\n", (u8)c);
        lits.insert(ue2_literal(c, false));
    }

    u32 count = findMaxLiteralMatches(*left.graph(), lits);
    DEBUG_PRINTF("counting miracle %u\n", count + 1);
    if (count && count < 50) {
        *cm_count = count + 1;
    }
}

} // namespace ue2
