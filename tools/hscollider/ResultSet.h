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

#ifndef RESULTSET_H
#define RESULTSET_H

#include <iostream>
#include <map>
#include <set>
#include <utility>
#include <vector>

// Type for capturing groups: a vector of (from, to) offsets, with both set to
// -1 for inactive groups (like pcre's ovector). Used by hybrid modes.
typedef std::vector<std::pair<int, int> > CaptureVec;

// Class representing a single match, encapsulating to/from offsets.
class MatchResult {
public:
    MatchResult(unsigned long long start, unsigned long long end)
        : from(start), to(end) {}
    MatchResult(unsigned long long start, unsigned long long end,
                const CaptureVec &cap)
            : from(start), to(end), captured(cap) {}

    bool operator<(const MatchResult &a) const {
        if (from != a.from) {
            return from < a.from;
        }
        if (to != a.to) {
            return to < a.to;
        }
        return captured < a.captured;
    }

    bool operator==(const MatchResult &a) const {
        return from == a.from && to == a.to && captured == a.captured;
    }

    unsigned long long from;
    unsigned long long to;
    CaptureVec captured;
};

enum ResultSource {
    RESULT_FROM_UE2,
    RESULT_FROM_PCRE,
    RESULT_FROM_GRAPH,
};

inline
std::ostream &operator<<(std::ostream &out, ResultSource src) {
    switch (src) {
    case RESULT_FROM_UE2:
        out << "UE2";
        break;
    case RESULT_FROM_GRAPH:
        out << "Graph";
        break;
    case RESULT_FROM_PCRE:
        out << "PCRE";
        break;
    }
    return out;
}

class ResultSet {
public:
    // Constructor.
    explicit ResultSet(ResultSource s) : src(s) {}

    // Can be constructed with a set of end-offsets.
    ResultSet(const std::set<unsigned int> &m, ResultSource s) : src(s) {
        for (const auto &offset : m) {
            matches.emplace(0, offset);
        }
    }

    // Equality.
    bool operator==(const ResultSet &other) const {
        return uoom == other.uoom &&
               match_after_halt == other.match_after_halt &&
               invalid_id == other.invalid_id &&
               matches == other.matches;
    }

    // Inequality.
    bool operator!=(const ResultSet &other) const { return !(*this == other); }

    // Add a match.
    void addMatch(unsigned long long from, unsigned long long to,
                  int block = 0) {
        MatchResult m(from, to);
        matches.insert(m);

        if (matches_by_block[block].find(m) != matches_by_block[block].end()) {
            dupe_matches.insert(m);
        } else {
            matches_by_block[block].insert(m);
        }
    }

    // Add a match (with capturing vector)
    void addMatch(unsigned long long from, unsigned long long to,
                  const CaptureVec &cap, int block = 0) {
        MatchResult m(from, to, cap);
        matches.insert(m);

        if (matches_by_block[block].find(m) != matches_by_block[block].end()) {
            dupe_matches.insert(m);
        } else {
            matches_by_block[block].insert(m);
        }
    }

    // Clear all matches.
    void clear() {
        matches.clear();
        dupe_matches.clear();
        matches_by_block.clear();
    }

    // Unexpected out of order match seen.
    bool uoom = false;

    // A match was received after termination was requested.
    bool match_after_halt = false;

    // A match from an invalid ID was seen.
    bool invalid_id = false;

    // Ordered set of matches.
    std::set<MatchResult> matches;

    // Matches grouped by stream write/block that we see them in.
    std::map<int, std::set<MatchResult>> matches_by_block;

    // Dupe matches that we have seen.
    std::set<MatchResult> dupe_matches;

    /* Where these results came from (does not take part in comparisions) */
    ResultSource src;
};

#endif
