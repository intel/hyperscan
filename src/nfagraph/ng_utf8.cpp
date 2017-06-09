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
 * \brief UTF-8 transforms and operations.
 */
#include "ng_utf8.h"

#include "ng.h"
#include "ng_prune.h"
#include "ng_util.h"
#include "compiler/compiler.h"
#include "util/graph_range.h"
#include "util/unicode_def.h"

#include <set>
#include <vector>

using namespace std;

namespace ue2 {

static
void allowIllegal(NGHolder &g, NFAVertex v, u8 pred_char) {
    if (in_degree(v, g) != 1) {
        DEBUG_PRINTF("unexpected pred\n");
        assert(0); /* should be true due to the early stage of this analysis */
        return;
    }

    CharReach &cr = g[v].char_reach;
    if (pred_char == 0xe0) {
        assert(cr.isSubsetOf(CharReach(0xa0, 0xbf)));
        if (cr == CharReach(0xa0, 0xbf)) {
            cr |= CharReach(0x80, 0x9f);
        }
    } else if (pred_char == 0xf0) {
        assert(cr.isSubsetOf(CharReach(0x90, 0xbf)));
        if (cr == CharReach(0x90, 0xbf)) {
            cr |= CharReach(0x80, 0x8f);
        }
    } else if (pred_char == 0xf4) {
        assert(cr.isSubsetOf(CharReach(0x80, 0x8f)));
        if (cr == CharReach(0x80, 0x8f)) {
            cr |= CharReach(0x90, 0xbf);
        }
    } else {
        assert(0); /* unexpected pred */
    }
}

/** \brief Relax forbidden UTF-8 sequences.
 *
 * Some byte sequences can not appear in valid UTF-8 as they encode code points
 * above \\x{10ffff} or they represent overlong encodings. As we require valid
 * UTF-8 input, we have no defined behaviour in these cases, as a result we can
 * accept them if it simplifies the graph. */
void relaxForbiddenUtf8(NGHolder &g, const ExpressionInfo &expr) {
    if (!expr.utf8) {
        return;
    }

    const CharReach e0(0xe0);
    const CharReach f0(0xf0);
    const CharReach f4(0xf4);

    for (auto v : vertices_range(g)) {
        const CharReach &cr = g[v].char_reach;
        if (cr == e0 || cr == f0 || cr == f4) {
            u8 pred_char = cr.find_first();
            for (auto t : adjacent_vertices_range(v, g)) {
                allowIllegal(g, t, pred_char);
            }
        }
    }
}

static
bool hasPredInSet(const NGHolder &g, NFAVertex v, const set<NFAVertex> &s) {
    for (auto u : inv_adjacent_vertices_range(v, g)) {
        if (contains(s, u)) {
            return true;
        }
    }
    return false;
}

static
bool hasSuccInSet(const NGHolder &g, NFAVertex v, const set<NFAVertex> &s) {
    for (auto w : adjacent_vertices_range(v, g)) {
        if (contains(s, w)) {
            return true;
        }
    }
    return false;
}

static
void findSeeds(const NGHolder &h, const bool som, vector<NFAVertex> *seeds) {
    set<NFAVertex> bad; /* from zero-width asserts near accepts, etc */
    for (auto v : inv_adjacent_vertices_range(h.accept, h)) {
        const CharReach &cr = h[v].char_reach;
        if (!isutf8ascii(cr) && !isutf8start(cr)) {
            bad.insert(v);
        }
    }

    for (auto v : inv_adjacent_vertices_range(h.acceptEod, h)) {
        const CharReach &cr = h[v].char_reach;
        if (!isutf8ascii(cr) && !isutf8start(cr)) {
            bad.insert(v);
        }
    }

    // we want to be careful with asserts connected to starts
    // as well as they may not finish a code point
    for (auto v : vertices_range(h)) {
        if (is_virtual_start(v, h)) {
            bad.insert(v);
            insert(&bad, adjacent_vertices(v, h));
        }
    }

    /* we cannot handle vertices connected to accept as would report matches in
     * the middle of codepoints. acceptEod is not a problem as the input must
     * end at a codepoint boundary */
    bad.insert(h.accept);

    // If we're in SOM mode, we don't want to mess with vertices that have a
    // direct edge from startDs.
    if (som) {
        insert(&bad, adjacent_vertices(h.startDs, h));
    }

    set<NFAVertex> already_seeds; /* already marked as seeds */
    for (auto v : vertices_range(h)) {
        const CharReach &cr = h[v].char_reach;

        if (!isutf8ascii(cr) || !hasSelfLoop(v, h)) {
            continue;
        }

        if (hasSuccInSet(h, v, bad)) {
            continue;
        }

        // Skip vertices that are directly connected to other vertices already
        // in the seeds list: we can't collapse two of these directly next to
        // each other.
        if (hasPredInSet(h, v, already_seeds) ||
            hasSuccInSet(h, v, already_seeds)) {
            continue;
        }

        DEBUG_PRINTF("%zu is a seed\n", h[v].index);
        seeds->push_back(v);
        already_seeds.insert(v);
    }
}

static
bool expandCyclic(NGHolder &h, NFAVertex v) {
    DEBUG_PRINTF("inspecting %zu\n", h[v].index);
    bool changes = false;

    auto v_preds = preds(v, h);
    auto v_succs = succs(v, h);

    set<NFAVertex> start_siblings;
    set<NFAVertex> end_siblings;

    CharReach &v_cr = h[v].char_reach;

    /* We need to find start vertices which have all of our preds.
     * As we have a self loop, it must be one of our succs. */
    for (auto a : adjacent_vertices_range(v, h)) {
        auto a_preds = preds(a, h);

        if (a_preds == v_preds && isutf8start(h[a].char_reach)) {
            DEBUG_PRINTF("%zu is a start v\n", h[a].index);
            start_siblings.insert(a);
        }
    }

    /* We also need to find full cont vertices which have all our own succs;
     * As we have a self loop, it must be one of our preds. */
    for (auto a : inv_adjacent_vertices_range(v, h)) {
        auto a_succs = succs(a, h);

        if (a_succs == v_succs && h[a].char_reach == UTF_CONT_CR) {
            DEBUG_PRINTF("%zu is a full tail cont\n", h[a].index);
            end_siblings.insert(a);
        }
    }

    for (auto s : start_siblings) {
        if (out_degree(s, h) != 1) {
            continue;
        }

        const CharReach &cr = h[s].char_reach;
        if (cr.isSubsetOf(UTF_TWO_START_CR)) {
            if (end_siblings.find(*adjacent_vertices(s, h).first)
                == end_siblings.end()) {
                DEBUG_PRINTF("%zu is odd\n", h[s].index);
                continue;
            }
        } else if (cr.isSubsetOf(UTF_THREE_START_CR)) {
            NFAVertex m = *adjacent_vertices(s, h).first;

            if (h[m].char_reach != UTF_CONT_CR
                || out_degree(m, h) != 1) {
                continue;
            }
            if (end_siblings.find(*adjacent_vertices(m, h).first)
                == end_siblings.end()) {
                DEBUG_PRINTF("%zu is odd\n", h[s].index);
                continue;
            }
        } else if (cr.isSubsetOf(UTF_FOUR_START_CR)) {
            NFAVertex m1 = *adjacent_vertices(s, h).first;

            if (h[m1].char_reach != UTF_CONT_CR
                || out_degree(m1, h) != 1) {
                continue;
            }

            NFAVertex m2 = *adjacent_vertices(m1, h).first;

            if (h[m2].char_reach != UTF_CONT_CR
                || out_degree(m2, h) != 1) {
                continue;
            }

            if (end_siblings.find(*adjacent_vertices(m2, h).first)
                == end_siblings.end()) {
                DEBUG_PRINTF("%zu is odd\n", h[s].index);
                continue;
            }
        } else {
            DEBUG_PRINTF("%zu is bad\n", h[s].index);
          continue;
        }

        v_cr |= cr;
        clear_vertex(s, h);
        changes = true;
    }

    if (changes) {
        v_cr |= UTF_CONT_CR; /* we need to add in cont reach */
        v_cr.set(0xc0); /* we can also add in the forbidden bytes as we require
                         * valid unicode data */
        v_cr.set(0xc1);
        v_cr |= CharReach(0xf5, 0xff);
    }

    return changes;
}

/** \brief Contract cycles of UTF-8 code points down to a single cyclic vertex
 * where possible, based on the assumption that we will always be matching
 * against well-formed input. */
void utf8DotRestoration(NGHolder &h, bool som) {
    vector<NFAVertex> seeds; /* cyclic ascii vertices */
    findSeeds(h, som, &seeds);

    bool changes = false;
    for (auto v : seeds) {
        changes |= expandCyclic(h, v);
    }

    if (changes) {
        pruneUseless(h);
    }
}

} // namespace ue2
