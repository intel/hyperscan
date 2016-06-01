/*
 * Copyright (c) 2015-2016, Intel Corporation
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
 * \brief Pattern lifetime analysis.
 */

#include "config.h"

#include "ng_find_matches.h"

#include "nfagraph/ng_graph.h"
#include "nfagraph/ng_util.h"
#include "parser/position.h"
#include "util/container.h"
#include "util/compare.h"
#include "util/report.h"
#include "util/report_manager.h"

#include <algorithm>

using namespace std;
using namespace ue2;

namespace {

struct StateSet {
    explicit StateSet(size_t sz) : s(sz), som(sz, 0) {}
    boost::dynamic_bitset<> s; // bitset of states that are on
    vector<size_t> som; // som value for each state
};

using MatchSet = set<pair<size_t, size_t>>;

struct fmstate {
    const size_t num_states; // number of vertices in graph
    StateSet states; // currently active states
    StateSet next; // states on after this iteration
    vector<NFAVertex> vertices; // mapping from index to vertex
    size_t offset = 0;
    unsigned char cur = 0;
    unsigned char prev = 0;
    const bool som;
    const bool utf8;
    const bool allowStartDs;
    const ReportManager &rm;

    boost::dynamic_bitset<> accept; // states leading to accept
    boost::dynamic_bitset<> accept_with_eod; // states leading to accept or eod

    fmstate(const NGHolder &g, bool som_in, bool utf8_in, bool aSD_in,
            const ReportManager &rm_in)
        : num_states(num_vertices(g)), states(num_states), next(num_states),
          vertices(num_vertices(g), NFAGraph::null_vertex()), som(som_in),
          utf8(utf8_in), allowStartDs(aSD_in), rm(rm_in), accept(num_states),
          accept_with_eod(num_states) {
        // init states
        states.s.set(g[g.start].index);
        if (allowStartDs) {
            states.s.set(g[g.startDs].index);
        }
        // fill vertex mapping
        for (const auto &v : vertices_range(g)) {
            vertices[g[v].index] = v;
        }
        // init accept states
        for (const auto &u : inv_adjacent_vertices_range(g.accept, g)) {
            accept.set(g[u].index);
        }
        accept_with_eod = accept;
        for (const auto &u : inv_adjacent_vertices_range(g.acceptEod, g)) {
            accept_with_eod.set(g[u].index);
        }
    }
};

} // namespace

static
bool isWordChar(const unsigned char c) {
    // check if it's an alpha character
    if (ourisalpha(c)) {
        return true;
    }
    // check if it's a digit
    if (c >= '0' && c <= '9') {
        return true;
    }
    // check if it's an underscore
    if (c == '_') {
        return true;
    }
    return false;
}

static
bool isUtf8CodePoint(const char c) {
    // check if this is a start of 4-byte character
    if ((c & 0xF8) == 0xF0) {
        return true;
    }
    // check if this is a start of 3-byte character
    if ((c & 0xF0) == 0xE0) {
        return true;
    }
    // check if this is a start of 2-byte character
    if ((c & 0xE0) == 0xC0) {
        return true;
    }
    // check if this is a single-byte character
    if ((c & 0x80) == 0) {
        return true;
    }
    return false;
}

static
bool canReach(const NGHolder &g, const NFAEdge &e,
              struct fmstate &state) {
    auto flags = g[e].assert_flags;
    if (!flags) {
        return true;
    }

    if (flags & POS_FLAG_ASSERT_WORD_TO_NONWORD) {
        if (isWordChar(state.prev) && !isWordChar(state.cur)) {
            return true;
        }
    }

    if (flags & POS_FLAG_ASSERT_NONWORD_TO_WORD) {
        if (!isWordChar(state.prev) && isWordChar(state.cur)) {
            return true;
        }
    }

    if (flags & POS_FLAG_ASSERT_WORD_TO_WORD) {
        if (isWordChar(state.prev) && isWordChar(state.cur)) {
            return true;
        }
    }

    if (flags & POS_FLAG_ASSERT_NONWORD_TO_NONWORD) {
        if (!isWordChar(state.prev) && !isWordChar(state.cur)) {
            return true;
        }
    }

    return false;
}

static
void getMatches(const NGHolder &g, MatchSet &matches, struct fmstate &state,
                bool allowEodMatches) {
    auto acc_states = state.states.s;
    acc_states &= allowEodMatches ? state.accept_with_eod : state.accept;

    for (size_t i = acc_states.find_first(); i != acc_states.npos;
         i = acc_states.find_next(i)) {
        const NFAVertex u = state.vertices[i];
        const size_t &som_offset = state.states.som[i];

        // we can't accept anything from startDs in between UTF-8 codepoints
        if (state.utf8 && u == g.startDs && !isUtf8CodePoint(state.cur)) {
            continue;
        }

        for (const auto &e : out_edges_range(u, g)) {
            NFAVertex v = target(e, g);
            if (v == g.accept || (v == g.acceptEod && allowEodMatches)) {
                // check edge assertions if we are allowed to reach accept
                if (!canReach(g, e, state)) {
                    continue;
                }
                DEBUG_PRINTF("match found at %zu\n", state.offset);

                assert(!g[u].reports.empty());
                for (const auto &report_id : g[u].reports) {
                    const Report &ri = state.rm.getReport(report_id);

                    DEBUG_PRINTF("report %u has offset adjustment %d\n",
                                 report_id, ri.offsetAdjust);
                    matches.emplace(som_offset, state.offset + ri.offsetAdjust);
                }
            }
        }
    }
}

static
void step(const NGHolder &g, struct fmstate &state) {
    state.next.s.reset();

    for (size_t i = state.states.s.find_first(); i != state.states.s.npos;
         i = state.states.s.find_next(i)) {
        const NFAVertex &u = state.vertices[i];
        const size_t &u_som_offset = state.states.som[i];

        for (const auto &e : out_edges_range(u, g)) {
            NFAVertex v = target(e, g);
            if (v == g.acceptEod) {
                // can't know the future: we don't know if we're at EOD.
                continue;
            }
            if (v == g.accept) {
                continue;
            }

            if (!state.allowStartDs && v == g.startDs) {
                continue;
            }

            const CharReach &cr = g[v].char_reach;
            const size_t v_idx = g[v].index;

            // check reachability and edge assertions
            if (cr.test(state.cur) && canReach(g, e, state)) {
                // if we aren't in SOM mode, just set every SOM to 0
                if (!state.som) {
                    state.next.s.set(v_idx);
                    state.next.som[v_idx] = 0;
                    continue;
                }

                // if this is first vertex since start, use current offset as SOM
                size_t next_som;
                if (u == g.start || u == g.startDs || is_virtual_start(u, g)) {
                    next_som = state.offset;
                } else {
                    // else, inherit SOM from predecessor
                    next_som = u_som_offset;
                }

                // check if the vertex is already active
                // if this vertex is not yet active, use current SOM
                if (!state.next.s.test(v_idx)) {
                    state.next.s.set(v_idx);
                    state.next.som[v_idx] = next_som;
                } else {
                    // else, work out leftmost SOM
                    state.next.som[v_idx] =
                        min(next_som, state.next.som[v_idx]);
                }
            }
        }
    }
}

// filter extraneous matches
static
void filterMatches(MatchSet &matches) {
    set<size_t> eom;

    // first, collect all end-offset matches
    for (const auto &match : matches) {
        eom.insert(match.second);
    }

    // now, go through all the end-offsets and filter extra matches
    for (const auto &elem : eom) {
        // find minimum SOM for this EOM
        size_t min_som = -1U;
        for (const auto &match : matches) {
            // skip entries with wrong EOM
            if (match.second != elem) {
                continue;
            }

            min_som = min(min_som, match.first);
        }

        auto msit = matches.begin();
        while (msit != matches.end()) {
            // skip everything that doesn't match
            if (msit->second != elem || msit->first <= min_som) {
                ++msit;
                continue;
            }
            DEBUG_PRINTF("erasing match %zu, %zu\n", msit->first, msit->second);
            matches.erase(msit++);
        }
    }
}

/** \brief Find all matches for a given graph when executed against \a input.
 *
 *  Fills \a matches with offsets into the data stream where a match is found.
 */
void findMatches(const NGHolder &g, const ReportManager &rm,
                 const string &input, MatchSet &matches, const bool notEod,
                 const bool som, const bool utf8) {
    assert(hasCorrectlyNumberedVertices(g));

    const bool allowStartDs = (proper_out_degree(g.startDs, g) > 0);

    struct fmstate state(g, som, utf8, allowStartDs, rm);

    for (auto it = input.begin(), ite = input.end(); it != ite; ++it) {
        state.offset = distance(input.begin(), it);
        state.cur = *it;

        step(g, state);

        getMatches(g, matches, state, false);

        DEBUG_PRINTF("index %zu, %zu states on\n", state.offset,
                     state.next.s.count());
        if (state.next.s.empty()) {
            if (state.som) {
                filterMatches(matches);
            }
            return;
        }
        state.states = state.next;
        state.prev = state.cur;
    }
    state.offset = input.size();
    state.cur = 0;

    // do additional step to get matches after stream end, this time count eod
    // matches also (or not, if we're in notEod mode)

    getMatches(g, matches, state, !notEod);

    if (state.som) {
        filterMatches(matches);
    }
}
