/*
 * Copyright (c) 2015, Intel Corporation
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

// convenience typedefs
typedef map<NFAVertex,size_t> SOMMap;
typedef set<pair<size_t, size_t> > MatchSet;

struct fmstate {
    SOMMap states;
    SOMMap next;
    size_t offset;
    unsigned char cur;
    unsigned char prev;
    const bool som;
    const bool utf8;
    const bool allowStartDs;
    const ReportManager &rm;

    fmstate(const bool som_in, const bool utf8_in, const bool aSD_in,
            const ReportManager &rm_in)
        : offset(0), cur(0), prev(0), som(som_in), utf8(utf8_in),
          allowStartDs(aSD_in), rm(rm_in) {}
};

static
void initStates(const NGHolder &g, struct fmstate &state) {
    state.states.insert(make_pair(g.start, 0));
    if (state.allowStartDs) {
        state.states.insert(make_pair(g.startDs, 0));
    }
}

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
bool canReach(const NGHolder &g, const NFAVertex &src, const NFAVertex &dst,
              struct fmstate &state) {
    // find relevant edge and see whether it has asserts
    NFAEdge e;
    bool exists;
    u32 flags;

    tie(e, exists) = edge(src, dst, g);
    assert(exists);

    flags = g[e].assert_flags;
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
    SOMMap::const_iterator it, ite;

    for (it = state.states.begin(), ite = state.states.end(); it != ite; ++it) {
        NFAGraph::adjacency_iterator ai, ae;

        // we can't accept anything from startDs inbetween UTF-8 codepoints
        if (state.utf8 && it->first == g.startDs && !isUtf8CodePoint(state.cur)) {
            continue;
        }

        for (tie(ai, ae) = adjacent_vertices(it->first, g); ai != ae; ++ai) {
            if (*ai == g.accept || (*ai == g.acceptEod && allowEodMatches)) {
                // check edge assertions if we are allowed to reach accept
                if (!canReach(g, it->first, *ai, state)) {
                    continue;
                }
                DEBUG_PRINTF("match found at %zu\n", state.offset);

                assert(!g[it->first].reports.empty());
                for (const auto &report_id :
                     g[it->first].reports) {
                    const Report &ri = state.rm.getReport(report_id);

                    DEBUG_PRINTF("report %u has offset adjustment %d\n",
                                 report_id, ri.offsetAdjust);
                    matches.insert(
                        make_pair(it->second, state.offset + ri.offsetAdjust));
                }
            }
        }
    }
}

static
void step(const NGHolder &g, struct fmstate &state) {
    state.next.clear();
    SOMMap::iterator it, ite;

    for (it = state.states.begin(), ite = state.states.end(); it != ite; ++it) {
        NFAGraph::adjacency_iterator ai, ae;

        for (tie(ai, ae) = adjacent_vertices(it->first, g); ai != ae; ++ai) {
            if (*ai == g.acceptEod) {
                // can't know the future: we don't know if we're at EOD.
                continue;
            }
            if (*ai == g.accept) {
                continue;
            }

            if (!state.allowStartDs && *ai == g.startDs) {
                continue;
            }

            const CharReach &cr = g[*ai].char_reach;
            // check reachability and edge assertions
            if (cr.test(state.cur) && canReach(g, it->first, *ai, state)) {
                SOMMap::const_iterator ni;
                size_t next_som;

                // if we aren't in SOM mode, just set every SOM to 0
                if (!state.som) {
                    state.next[*ai] = 0;
                    continue;
                }

                // if this is first vertex since start, use current offset as SOM
                if (it->first == g.start || it->first == g.startDs ||
                        is_virtual_start(it->first, g)) {
                    next_som = state.offset;
                } else {
                    // else, inherit SOM from predecessor
                    next_som = it->second;
                }

                // check if the vertex is already active
                ni = state.next.find(*ai);

                // if this vertex is not yet active, use current SOM
                if (ni == state.next.end()) {
                    state.next[*ai] = next_som;
                } else {
                    // else, work out leftmost SOM
                    state.next[*ai] = min(next_som, ni->second);
                }
            }
        }
    }
}

// filter extraneous matches
static void filterMatches(MatchSet &matches) {
    set<size_t> eom;
    MatchSet::iterator msit;

    // first, collect all end-offset matches
    for (msit = matches.begin(); msit != matches.end(); ++msit) {
        eom.insert(msit->second);
    }

    // now, go through all the end-offsets and filter extra matches
    set<size_t>::const_iterator eomit;
    for (eomit = eom.begin(); eomit != eom.end(); ++eomit) {

        // find minimum SOM for this EOM
        size_t min_som = -1U;
        for (msit = matches.begin(); msit != matches.end(); ++msit) {
            // skip entries with wrong EOM
            if (msit->second != *eomit) {
                continue;
            }

            min_som = min(min_som, msit->first);
        }

        msit = matches.begin();
        while (msit != matches.end()) {
            // skip everything that doesn't match
            if (msit->second != *eomit || msit->first <= min_som) {
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
    const bool allowStartDs = (proper_out_degree(g.startDs, g) > 0);

    struct fmstate state(som, utf8, allowStartDs, rm);

    initStates(g, state);

    string::const_iterator it, ite;
    for (it = input.begin(), ite = input.end(); it != ite; ++it) {
        state.offset = distance(input.begin(), it);
        state.cur = *it;

        step(g, state);

        getMatches(g, matches, state, false);

        DEBUG_PRINTF("index %zu, %zu states on\n", state.offset, state.next.size());
        if (state.next.empty()) {
            if (state.som) {
                filterMatches(matches);
            }
            return;
        }
        state.states.swap(state.next);
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
