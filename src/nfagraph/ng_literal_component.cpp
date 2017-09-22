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
 * \brief Literal Component Splitting. Identifies literals that span the
 * graph and moves them into Rose.
 */

#include "ng_literal_component.h"

#include "grey.h"
#include "ng.h"
#include "ng_prune.h"
#include "ng_util.h"
#include "ue2common.h"
#include "compiler/compiler.h"
#include "rose/rose_build.h"
#include "util/container.h"
#include "util/graph.h"
#include "util/graph_range.h"
#include "util/ue2string.h"

#include <unordered_set>

using namespace std;

namespace ue2 {

static
bool isLiteralChar(const NGHolder &g, NFAVertex v, bool &nocase,
                   bool &casefixed) {
    const CharReach &cr = g[v].char_reach;
    const size_t num = cr.count();
    if (num > 2) {
        return false; // char class
    }

    if (!casefixed) {
        if (num == 2 && cr.isCaselessChar()) {
            nocase = true;
            casefixed = true;
            return true;
        } else if (num == 1) {
            if (cr.isAlpha()) {
                nocase = false;
                casefixed = true;
            }
            // otherwise, still acceptable but we can't fix caselessness yet
            return true;
        }
    } else {
        // nocase property is fixed
        if (nocase) {
            if ((num == 2 && cr.isCaselessChar()) ||
                (num == 1 && !cr.isAlpha())) {
                return true;
            }
        } else {
            return (num == 1);
        }
    }

    return false;
}

static
void addToString(string &s, const NGHolder &g, NFAVertex v) {
    const CharReach &cr = g[v].char_reach;
    assert(cr.count() == 1 || cr.isCaselessChar());

    char c = (char)cr.find_first();
    s.push_back(c);
}

static
bool splitOffLiteral(NG &ng, NGHolder &g, NFAVertex v, const bool anchored,
                     set<NFAVertex> &dead) {
    DEBUG_PRINTF("examine vertex %zu\n", g[v].index);
    bool nocase = false, casefixed = false;

    assert(!is_special(v, g));

    size_t reqInDegree;
    if (anchored) {
        reqInDegree = 1;
        assert(edge(g.start, v, g).second);
    } else {
        reqInDegree = 2;
        assert(edge(g.start, v, g).second);
        assert(edge(g.startDs, v, g).second);
    }
    if (in_degree(v, g) > reqInDegree) {
        DEBUG_PRINTF("extra in-edges\n");
        return false;
    }

    if (!isLiteralChar(g, v, nocase, casefixed)) {
        DEBUG_PRINTF("not literal\n");
        return false;
    }

    string literal;
    addToString(literal, g, v);

    // Remaining vertices must come in a chain, each with one in-edge and one
    // out-edge only.
    NFAVertex u;
    while (1) {
        if (out_degree(v, g) != 1) {
            DEBUG_PRINTF("branches, not literal\n");
            return false;
        }

        u = v; // previous vertex
        v = *(adjacent_vertices(v, g).first);

        DEBUG_PRINTF("loop, v=%zu\n", g[v].index);

        if (is_special(v, g)) {
            if (v == g.accept || v == g.acceptEod) {
                break; // OK
            } else {
                assert(0); // start?
                return false;
            }
        } else {
            // Ordinary, must be literal
            if (!isLiteralChar(g, v, nocase, casefixed)) {
                DEBUG_PRINTF("not literal\n");
                return false;
            }
            if (in_degree(v, g) != 1) {
                DEBUG_PRINTF("branches, not literal\n");
                return false;
            }
        }

        addToString(literal, g, v);
    }

    // Successfully found a literal; there might be multiple report IDs, in
    // which case we add all the reports.
    assert(!is_special(u, g));
    bool eod = v == g.acceptEod;
    assert(eod || v == g.accept);

    DEBUG_PRINTF("success: found %s literal '%s'\n",
                 anchored ? "anchored" : "unanchored",
                 escapeString(literal).c_str());

    // Literals of length 1 are better served going through later optimisation
    // passes, where they might be combined together into a character class.
    if (literal.length() == 1) {
        DEBUG_PRINTF("skipping literal of length 1\n");
        return false;
    }

    ng.rose->add(anchored, eod, ue2_literal(literal, nocase), g[u].reports);

    // Remove the terminal vertex. Later, we rely on pruneUseless to remove the
    // other vertices in this chain, since they'll no longer lead to an accept.
    dead.insert(u);

    return true;
}

/** \brief Split off literals. True if any changes were made to the graph. */
bool splitOffLiterals(NG &ng, NGHolder &g) {
    if (!ng.cc.grey.allowLiteral) {
        return false;
    }

    bool changed = false;
    set<NFAVertex> dead;

    unordered_set<NFAVertex> unanchored; // for faster lookup.
    insert(&unanchored, adjacent_vertices(g.startDs, g));

    // Anchored literals.
    for (auto v : adjacent_vertices_range(g.start, g)) {
        if (!is_special(v, g) && !contains(unanchored, v)) {
            changed |= splitOffLiteral(ng, g, v, true, dead);
        }
    }

    // Unanchored literals.
    for (auto v : adjacent_vertices_range(g.startDs, g)) {
        if (!is_special(v, g)) {
            changed |= splitOffLiteral(ng, g, v, false, dead);
        }
    }

    if (changed) {
        remove_vertices(dead, g);
        pruneUseless(g);
        return true;
    }

    return false;
}

} // namespace ue2
