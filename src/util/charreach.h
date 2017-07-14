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
 * \brief Class for representing character reachability.
 *
 * This is a simple (but hopefully fast) class for representing 8-bit character
 * reachability, along with a bunch of useful operations.
 */

#ifndef NG_CHARREACH_H
#define NG_CHARREACH_H

#include "ue2common.h"
#include "util/bitfield.h"

#include <string>

namespace ue2 {

class CharReach {
private:
    /// Underlying storage.
    ue2::bitfield<256> bits;

public:
    static constexpr size_t npos = decltype(bits)::npos; //!< One past the max value.

    /// Empty constructor.
    CharReach() {}

    /// Constructor for a character class containing a single char.
    explicit CharReach(unsigned char c) { set(c); }

    /// Constructor for a character class representing a contiguous range of
    /// chars, inclusive.
    CharReach(unsigned char from, unsigned char to) { setRange(from, to); }

    /// Constructor for a character class based on the set of chars in a
    /// string.
    explicit CharReach(const std::string &str) { set(str); }

    /// Returns total capacity.
    static constexpr size_t size() { return npos; }

    /// Returns a CharReach with complete reachability (a "dot").
    static CharReach dot() { return CharReach(0, 255); }

    /// Complete bitset equality.
    bool operator==(const CharReach &a) const { return bits == a.bits; }

    /// Inequality.
    bool operator!=(const CharReach &a) const { return bits != a.bits; }

    /// Ordering.
    bool operator<(const CharReach &a) const { return bits < a.bits; }

    /// Set all bits.
    void setall() { bits.setall(); }

    /// Clear all bits.
    void clear() { bits.clear(); }

    /// Clear bit N.
    void clear(unsigned char n) { bits.clear(n); }

    /// Set bit N.
    void set(unsigned char n) { bits.set(n); }

    /// Test bit N.
    bool test(unsigned char n) const { return bits.test(n); }

    /// Flip bit N.
    void flip(unsigned char n) { bits.flip(n); }

    /// Flip all bits.
    void flip() { bits.flip(); }

    // Switch on the bit in the range (from, to), inclusive.
    void setRange(unsigned char from, unsigned char to) {
        bits.set_range(from, to);
    }

    // Switch on the bits corresponding to the characters in \a s.
    void set(const std::string &s);

    /// Returns number of bits set on.
    size_t count() const { return bits.count(); }

    /// Are no bits set?
    bool none() const { return bits.none(); }

    /// Is any bit set?
    bool any() const { return bits.any(); }

    /// Are all bits set?
    bool all() const { return bits.all(); }

    /// Returns first bit set, or CharReach::npos if none set.
    size_t find_first() const { return bits.find_first(); }

    /// Returns last bit set, or CharReach::npos if none set.
    size_t find_last() const { return bits.find_last(); }

    /// Returns next bit set, or CharReach::npos if none set after n.
    size_t find_next(size_t last) const { return bits.find_next(last); }

    /// Returns (zero-based) N'th bit set, or CharReach::npos if fewer than
    /// N + 1 bits are on.
    size_t find_nth(size_t n) const { return bits.find_nth(n); }

    /// Bitwise OR.
    CharReach operator|(const CharReach &a) const {
        CharReach cr(*this);
        cr.bits |= a.bits;
        return cr;
    }

    /// Bitwise OR-equals.
    void operator|=(const CharReach &a) { bits |= a.bits; }

    /// Bitwise AND.
    CharReach operator&(const CharReach &a) const {
        CharReach cr(*this);
        cr.bits &= a.bits;
        return cr;
    }

    /// Bitwise AND-equals.
    void operator&=(const CharReach &a) { bits &= a.bits; }

    /// Bitwise XOR.
    CharReach operator^(const CharReach &a) const {
        CharReach cr(*this);
        cr.bits ^= a.bits;
        return cr;
    }

    /// Bitwise complement.
    CharReach operator~(void) const {
        CharReach cr(*this);
        cr.flip();
        return cr;
    }

    /// Do we only contain bits representing alpha characters?
    bool isAlpha() const;

    /// Do we represent an uppercase/lowercase pair?
    bool isCaselessChar() const;

    /// Do we represent a cheapskate caseless set?
    bool isBit5Insensitive() const;

    /// Return a string containing the characters that are switched on.
    std::string to_string() const;

    /// Hash of enabled bits.
    size_t hash() const { return bits.hash(); }

    /// True if this character class is a subset of \a other.
    bool isSubsetOf(const CharReach &other) const;
};

/** \brief True iff there is a non-empty intersection between \a and \a b */
bool overlaps(const CharReach &a, const CharReach &b);

/** \brief True iff \a small is a subset of \a big. */
bool isSubsetOf(const CharReach &small, const CharReach &big);

bool isutf8ascii(const CharReach &cr);
bool isutf8start(const CharReach &cr);

} // namespace ue2

namespace std {

template<>
struct hash<ue2::CharReach> {
    size_t operator()(const ue2::CharReach &cr) const {
        return cr.hash();
    }
};

} // namespace std

#endif // NG_CHARREACH_H
