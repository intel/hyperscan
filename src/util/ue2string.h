/*
 * Copyright (c) 2015-2019, Intel Corporation
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
 * \brief Tools for string manipulation, ue2_literal definition.
 */

#ifndef UE2STRING_H
#define UE2STRING_H

#include "ue2common.h"
#include "util/charreach.h"
#include "util/compare.h"
#include "util/hash.h"
#include "util/operators.h"

#include <iterator>
#include <string>
#include <vector>

#include <boost/dynamic_bitset.hpp>
#include <boost/iterator/iterator_facade.hpp>

namespace ue2 {

/// Force the given string to upper-case.
void upperString(std::string &s);

size_t maxStringOverlap(const std::string &a, const std::string &b,
                        bool nocase);

size_t maxStringSelfOverlap(const std::string &a, bool nocase);

/// Compares two strings, returns non-zero if they're different.
u32 cmp(const char *a, const char *b, size_t len, bool nocase);

/**
 * \brief String type that also records whether the whole string is caseful or
 * caseless.
 *
 * You should use \ref ue2_literal if you need to represent a mixed-case
 * literal.
 */
struct ue2_case_string {
    ue2_case_string(std::string s_in, bool nocase_in)
        : s(std::move(s_in)), nocase(nocase_in) {
        if (nocase) {
            upperString(s);
        }
    }

    bool operator==(const ue2_case_string &other) const {
        return s == other.s && nocase == other.nocase;
    }

    std::string s;
    bool nocase;
};

struct ue2_literal : totally_ordered<ue2_literal> {
public:
    /// Single element proxy, pointed to by our const_iterator.
    struct elem {
        elem() : c(0), nocase(false) {}
        elem(char c_in, bool nc_in) : c(c_in), nocase(nc_in) {}
        bool operator==(const elem &o) const {
            return c == o.c && nocase == o.nocase;
        }
        bool operator!=(const elem &o) const {
            return c != o.c || nocase != o.nocase;
        }
        operator CharReach() const;
        char c;
        bool nocase;
    };

    /// Boost iterator_facade lets us synthesize an iterator simply.
    class const_iterator : public boost::iterator_facade<
            const_iterator,
            elem const,
            boost::random_access_traversal_tag,
            elem const> {
    public:
        const_iterator() {}
    private:
        friend class boost::iterator_core_access;
        void increment() {
            ++idx;
        }
        void decrement() {
            --idx;
        }
        void advance(size_t n) {
            idx += n;
        }
        difference_type distance_to(const const_iterator &other) const {
            return other.idx - idx;
        }
        bool equal(const const_iterator &other) const {
            return idx == other.idx && lit == other.lit;
        }
        const elem dereference() const {
            return elem(lit->s[idx], lit->nocase[idx]);
        }

        friend struct ue2_literal;
        const_iterator(const ue2_literal &lit_in, size_t idx_in)
            : lit(&lit_in), idx(idx_in) {}

        const ue2_literal *lit = nullptr;
        size_t idx;
    };

    using const_reverse_iterator = std::reverse_iterator<const_iterator>;
    using size_type = std::string::size_type;

    static const size_type npos;

    ue2_literal() = default;
    ue2_literal(const std::string &s_in, bool nc_in);
    ue2_literal(char c, bool nc_in);
    ue2_literal(const ue2_literal &) = default;
    ue2_literal(ue2_literal &&) = default;
    ue2_literal &operator=(const ue2_literal &) = default;
    ue2_literal &operator=(ue2_literal &&) = default;

    template<typename InputIt>
    ue2_literal(InputIt b, InputIt e) {
        for (; b != e; ++b) {
            push_back(*b);
        }
    }

    size_type length() const { return s.length(); }
    bool empty() const { return s.empty(); }
    ue2_literal substr(size_type pos, size_type n = npos) const;
    const char *c_str() const { return s.c_str(); }
    bool any_nocase() const;

    const_iterator begin() const {
        return const_iterator(*this, 0);
    }

    const_iterator end() const {
        return const_iterator(*this, s.size());
    }

    const_reverse_iterator rbegin() const {
        return const_reverse_iterator(end());
    }

    const_reverse_iterator rend() const {
        return const_reverse_iterator(begin());
    }

    ue2_literal &erase(size_type pos = 0, size_type n = npos);
    void push_back(const elem &e) {
        push_back(e.c, e.nocase);
    }

    void push_back(char c, bool nc);
    const elem back() const { return *rbegin(); }

    friend ue2_literal operator+(ue2_literal a, const ue2_literal &b) {
        a += b;
        return a;
    }

    /// Reverse this literal in-place.
    void reverse();

    void operator+=(const ue2_literal &b);
    bool operator==(const ue2_literal &b) const {
        return s == b.s && nocase == b.nocase;
    }
    bool operator<(const ue2_literal &b) const;

    void clear(void) { s.clear(); nocase.clear(); }

    const std::string &get_string() const { return s; }

    void swap(ue2_literal &other) {
        s.swap(other.s);
        nocase.swap(other.nocase);
    }

    size_t hash() const;

    void set_pure() { pure = true; }
    void unset_pure() { pure = false; }
    bool get_pure() const { return pure; }

    /* TODO: consider existing member functions possibly related with pure. */

private:
    friend const_iterator;
    std::string s;
    boost::dynamic_bitset<> nocase;
    bool pure = false; /**< born from cutting or not (pure literal). */
};

/// Return a reversed copy of this literal.
ue2_literal reverse_literal(const ue2_literal &in);

// Escape any meta characters in a string
std::string escapeStringMeta(const std::string &s);

/** Note: may be overly conservative if only partially nocase */
size_t maxStringSelfOverlap(const ue2_literal &a);
size_t minStringPeriod(const ue2_literal &a);
size_t maxStringOverlap(const ue2_literal &a, const ue2_literal &b);

/**
 * \brief True iff the range of a literal given cannot be considered entirely
 * case-sensitive nor entirely case-insensitive.
 */
template<class Iter>
bool mixed_sensitivity_in(Iter begin, Iter end) {
    bool cs = false;
    bool nc = false;
    for (auto it = begin; it != end; ++it) {
        if (!ourisalpha(it->c)) {
            continue;
        }
        if (it->nocase) {
            nc = true;
        } else {
            cs = true;
        }
    }

    return cs && nc;
}

/**
 * \brief True iff the literal cannot be considered entirely case-sensitive
 * nor entirely case-insensitive.
 */
inline
bool mixed_sensitivity(const ue2_literal &s) {
    return mixed_sensitivity_in(s.begin(), s.end());
}

void make_nocase(ue2_literal *lit);

struct case_iter {
    explicit case_iter(const ue2_literal &ss);
    const std::string &operator*() const { return s; } /* limited lifetime */
    case_iter &operator++ ();
    bool operator!=(const case_iter &b) const { return s != b.s; }
private:
    std::string s;
    std::string s_orig;
    std::vector<bool> nocase;
};

case_iter caseIterateBegin(const ue2_literal &lit);
case_iter caseIterateEnd();

/** \brief True if there is any overlap between the characters in \a s and the
 * set characters in \a cr.
 *
 * Note: this means that if \a s is nocase, then \a cr only needs to have
 * either the lower-case or upper-case version of a letter set.  */
bool contains(const ue2_literal &s, const CharReach &cr);

/// Returns true if \a a is a suffix of (or equal to) \a b.
bool isSuffix(const ue2_literal &a, const ue2_literal &b);

static inline
std::vector<CharReach> as_cr_seq(const ue2_literal &s) {
    std::vector<CharReach> rv;
    rv.reserve(s.length());
    rv.insert(rv.end(), s.begin(), s.end());
    return rv;
}

/** \brief True if the given literal consists entirely of a flood of the same
 * character. */
bool is_flood(const ue2_literal &s);

#if defined(DUMP_SUPPORT) || defined(DEBUG)
/* Utility functions for debugging/dumping */

/// Escape a string so it's dot-printable.
std::string dotEscapeString(const std::string &s);

std::string dumpString(const ue2_literal &lit);

/// Escape a string so that it's screen-printable.
std::string escapeString(const std::string &s);

/// Escape a ue2_literal so that it's screen-printable.
std::string escapeString(const ue2_literal &lit);

#endif

} // namespace ue2

namespace std {

template<>
struct hash<ue2::ue2_literal::elem> {
    size_t operator()(const ue2::ue2_literal::elem &elem) const {
        return ue2::hash_all(elem.c, elem.nocase);
    }
};

template<>
struct hash<ue2::ue2_literal> {
    size_t operator()(const ue2::ue2_literal &lit) const {
        return lit.hash();
    }
};

} // namespace std

#endif
