/*
 * Copyright (C) 2026 Intel Corporation
 *
 * This software and the related documents are Intel copyrighted materials,
 * and your use of them is governed by the express license under which they were
 * provided to you ("License"). Unless the License provides otherwise,
 * you may not use, modify, copy, publish, distribute, disclose or transmit this
 * software or the related documents without Intel's prior written permission.
 *
 * This software and the related documents are provided as is, with no express or
 * implied warranties, other than those that are expressly stated in the License.
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
            dupe_matches.insert(std::move(m));
        } else {
            matches_by_block[block].insert(std::move(m));
        }
    }

    // Add a match (with capturing vector)
    void addMatch(unsigned long long from, unsigned long long to,
                  const CaptureVec &cap, int block = 0) {
        MatchResult m(from, to, cap);
        matches.insert(m);

        if (matches_by_block[block].find(m) != matches_by_block[block].end()) {
            dupe_matches.insert(std::move(m));
        } else {
            matches_by_block[block].insert(std::move(m));
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
