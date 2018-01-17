/*
 * Copyright (c) 2015-2018, Intel Corporation
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
 * \brief Stop Alphabet calculation.
 */
#include "ng_stop.h"

#include "ng_depth.h"
#include "ng_holder.h"
#include "ng_misc_opt.h"
#include "ng_util.h"
#include "ue2common.h"
#include "nfa/castlecompile.h"
#include "som/som.h"
#include "util/charreach.h"
#include "util/container.h"
#include "util/dump_charclass.h"
#include "util/graph.h"
#include "util/graph_range.h"
#include "util/verify_types.h"

#include <map>
#include <set>
#include <vector>

using namespace std;

namespace ue2 {

/** Stop alphabet depth threshold. */
static const u32 MAX_STOP_DEPTH = 8;

namespace {

/** Depths from start, startDs for this graph. */
struct InitDepths {
    explicit InitDepths(const NGHolder &g)
        : start(calcDepthsFrom(g, g.start)),
          startDs(calcDepthsFrom(g, g.startDs)) {}

    depth maxDist(const NGHolder &g, NFAVertex v) const {
        u32 idx = g[v].index;
        assert(idx < start.size() && idx < startDs.size());
        const depth &d_start = start.at(idx).max;
        const depth &d_startDs = startDs.at(idx).max;
        if (d_start.is_unreachable()) {
            return d_startDs;
        } else if (d_startDs.is_unreachable()) {
            return d_start;
        }
        return max(d_start, d_startDs);
    }

private:
    vector<DepthMinMax> start;
    vector<DepthMinMax> startDs;
};

} // namespace

/** Find the set of characters that are not present in the reachability of
 * graph \p g after a certain depth (currently 8). If a character in this set
 * is encountered, it means that the NFA is either dead or has not progressed
 * more than 8 characters from its start states.
 *
 * This is only used to guide merging heuristics, use
 * findLeftOffsetStopAlphabet for real uses.
 */
CharReach findStopAlphabet(const NGHolder &g, som_type som) {
    const depth max_depth(MAX_STOP_DEPTH);
    const InitDepths depths(g);
    const map<NFAVertex, BoundedRepeatSummary> no_vertices;

    CharReach stopcr;

    for (auto v : vertices_range(g)) {
        if (is_special(v, g)) {
            continue;
        }

        if (depths.maxDist(g, v) >= max_depth) {
            if (som == SOM_NONE) {
                stopcr |= reduced_cr(v, g, no_vertices);
            } else {
                stopcr |= g[v].char_reach;
            }
        }
    }

    // Turn alphabet into stops.
    stopcr.flip();

    return stopcr;
}

/** Calculate the stop alphabet for each depth from 0 to MAX_STOP_DEPTH. Then
 * build an eight-bit mask per character C, with each bit representing the
 * depth before the location of character C (if encountered) that the NFA would
 * be in a predictable start state. */
vector<u8> findLeftOffsetStopAlphabet(const NGHolder &g, som_type som) {
    const depth max_depth(MAX_STOP_DEPTH);
    const InitDepths depths(g);
    const map<NFAVertex, BoundedRepeatSummary> no_vertices;

    vector<CharReach> reach(MAX_STOP_DEPTH);

    for (auto v : vertices_range(g)) {
        if (is_special(v, g)) {
            continue;
        }
        CharReach v_cr;
        if (som == SOM_NONE) {
            v_cr = reduced_cr(v, g, no_vertices);
        } else {
            v_cr = g[v].char_reach;
        }

        u32 d = min(max_depth, depths.maxDist(g, v));
        for (u32 i = 0; i < d; i++) {
            reach[i] |= v_cr;
        }
    }

#ifdef DEBUG
    for (u32 i = 0; i < MAX_STOP_DEPTH; i++) {
        DEBUG_PRINTF("depth %u, stop chars: ", i);
        describeClass(stdout, ~reach[i], 20, CC_OUT_TEXT);
        printf("\n");
    }
#endif

    vector<u8> stop(N_CHARS, 0);

    for (u32 i = 0; i < MAX_STOP_DEPTH; i++) {
        CharReach cr = ~reach[i]; // invert reach for stop chars.
        const u8 mask = 1U << i;
        for (size_t c = cr.find_first(); c != cr.npos; c = cr.find_next(c)) {
            stop[c] |= mask;
        }
    }

    return stop;
}

vector<u8> findLeftOffsetStopAlphabet(const CastleProto &castle,
                                      UNUSED som_type som) {
    const depth max_width = findMaxWidth(castle);
    DEBUG_PRINTF("castle has reach %s and max width %s\n",
                  describeClass(castle.reach()).c_str(),
                  max_width.str().c_str());

    const CharReach escape = ~castle.reach(); // invert reach for stop chars.

    u32 d = min(max_width, depth(MAX_STOP_DEPTH));
    const u8 mask = verify_u8((1U << d) - 1);

    vector<u8> stop(N_CHARS, 0);

    for (size_t c = escape.find_first(); c != escape.npos;
         c = escape.find_next(c)) {
        stop[c] |= mask;
    }

    return stop;
}

} // namespace ue2
