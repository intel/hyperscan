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
 * \brief Class for representing character reachability.
 *
 * This is a simple (but hopefully fast) class for representing 8-bit character
 * reachability, along with a bunch of useful operations.
 */
#include "ue2common.h"
#include "charreach.h"
#include "charreach_util.h"
#include "compare.h"
#include "unicode_def.h"

#include <cassert>
#include <string>

namespace ue2 {

/// Switch on the bits corresponding to the characters in \a s.
void CharReach::set(const std::string &s) {
    for (const auto &c : s) {
        set(c);
    }
}

/// Do we only contain bits representing alpha characters?
bool CharReach::isAlpha() const {
    if (none()) {
        return false;
    }
    for (size_t i = find_first(); i != npos; i = find_next(i)) {
        if (!ourisalpha((char)i)) {
            return false;
        }
    }
    return true;
}

/// Do we represent an uppercase/lowercase pair?
bool CharReach::isCaselessChar() const {
    if (count() != 2) {
        return false;
    }
    size_t first = find_first();
    size_t second = find_next(first);
    assert(first != npos && second != npos);
    return (char)first == mytoupper((char)second);
}

/// Do we represent a cheapskate caseless set?
bool CharReach::isBit5Insensitive() const {
    for (size_t i = find_first(); i != npos; i = find_next(i)) {
        if (!test((char)i ^ 0x20)) {
            return false;
        }
    }
    return true;
}

/// Return a string containing the characters that are switched on.
std::string CharReach::to_string() const {
    std::string s;
    for (size_t i = find_first(); i != npos; i = find_next(i)) {
        s += (char)i;
    }
    return s;
}

/** \brief True iff there is a non-empty intersection between \a and \a b */
bool overlaps(const CharReach &a, const CharReach &b) {
    return (a & b).any();
}

/** \brief True iff \a small is a subset of \a big. */
bool isSubsetOf(const CharReach &small, const CharReach &big) {
    return small.isSubsetOf(big);
}

/// True if this character class is a subset of \a other.
bool CharReach::isSubsetOf(const CharReach &other) const {
    return (bits & other.bits) == bits;
}

void make_caseless(CharReach *cr) {
    for (char c = 'A'; c <= 'Z'; c++) {
        if (cr->test(c) || cr->test(mytolower(c))) {
            cr->set(c);
            cr->set(mytolower(c));
        }
    }
}

bool isutf8ascii(const CharReach &cr) {
    return (cr & ~CharReach(0x0, 0x7f)).none();
}

bool isutf8start(const CharReach &cr) {
    return (cr & CharReach(0x0, UTF_CONT_MAX)).none();
}

void fill_bitvector(const CharReach &cr, u8 *bits) {
    assert(bits);
    std::fill_n(bits, 32, 0);
    for (size_t i = cr.find_first(); i != cr.npos; i = cr.find_next(i)) {
        bits[i / 8U] |= (u8)1U << (i % 8U);
    }
}

void make_and_cmp_mask(const CharReach &cr, u8 *and_mask, u8 *cmp_mask) {
    u8 lo = 0xff;
    u8 hi = 0;

    for (size_t c = cr.find_first(); c != cr.npos; c = cr.find_next(c)) {
        hi |= (u8)c;
        lo &= (u8)c;
    }

    *and_mask = ~(lo ^ hi);
    *cmp_mask = lo;
}

} // namespace ue2
