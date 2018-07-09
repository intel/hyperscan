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
 * \brief Fast bitset class with find_first and find_next operations.
 */

#ifndef BITFIELD_H
#define BITFIELD_H

#include "ue2common.h"
#include "popcount.h"
#include "util/bitutils.h"
#include "util/hash.h"

#include <array>
#include <cassert>

#include <boost/dynamic_bitset.hpp>

namespace ue2 {

/**
 * \brief Templated bitset class with find_first and find_next operations.
 *
 * This is a simple (but hopefully fast) class to replace our use of
 * std::bitset<>.
 *
 * Note: underlying storage is allocated as an array of 64-bit blocks. All
 * mutating operations MUST ensure that the trailer (the bits between
 * requested_size and the end of the array) is filled with zeroes; there's a
 * clear_trailer member function for this.
 */
template<size_t requested_size>
class bitfield {
public:
    /// Empty constructor, zero initializes all bits.
    bitfield() : bits{{0}} {
        assert(none());
    }

    bitfield(const boost::dynamic_bitset<> &a) : bits{{0}} {
        assert(a.size() == requested_size);
        assert(none());
        for (auto i = a.find_first(); i != a.npos; i = a.find_next(i)) {
            set(i);
        }
    }

    /// Complete bitset equality.
    bool operator==(const bitfield &a) const {
        return bits == a.bits;
    }

    /// Inequality.
    bool operator!=(const bitfield &a) const {
        return bits != a.bits;
    }

    /// Ordering.
    bool operator<(const bitfield &a) const {
        return bits < a.bits;
    }

    /// Set all bits.
    void setall() {
        for (auto &e : bits) {
            e = all_ones;
        }
        clear_trailer();
    }

    /// Set all bits (alias for bitset::setall, to match dynamic_bitset).
    void set() {
        setall();
    }

    /// Clear all bits.
    void clear() {
        for (auto &e : bits) {
            e = 0;
        }
    }

    /// Clear all bits (alias for bitset::clear).
    void reset() {
        clear();
    }

    /// Clear bit N.
    void clear(size_t n) {
        assert(n < size());
        bits[getword(n)] &= ~maskbit(n);
    }

    /// Set bit N.
    void set(size_t n) {
        assert(n < size());
        bits[getword(n)] |= maskbit(n);
    }

    /// Test bit N.
    bool test(size_t n) const {
        assert(n < size());
        return bits[getword(n)] & maskbit(n);
    }

    /// Flip bit N.
    void flip(size_t n) {
        assert(n < size());
        bits[getword(n)] ^= maskbit(n);
    }

    /// Flip all bits.
    void flip() {
        for (auto &e : bits) {
            e = ~e;
        }
        clear_trailer();
    }

    /// Switch on the bit in the range [from, to], inclusive.
    void set_range(size_t from, size_t to) {
        assert(from <= to);
        assert(to < requested_size);

        if (from / block_size == to / block_size) {
            // Small case, our indices are in the same block.
            block_type block = all_ones << (from % block_size);
            if (to % block_size != block_size - 1) {
                block &= maskbit(to + 1) - 1;
            }
            bits[from / block_size] |= block;
            return;
        }

        // Large case, work in block units. Write a partial mask, then a
        // run of all-ones blocks, then a partial mask at the end.
        size_t i = from;
        if (i % block_size) {
            block_type block = all_ones << (i % block_size);
            bits[i / block_size] |= block;
            i = ROUNDUP_N(i, block_size);
        }

        for (; i + block_size <= to + 1; i += block_size) {
            bits[i / block_size] = all_ones;
        }

        if (i <= to) {
            assert(to - i + 1 < block_size);
            bits[i / block_size] |= (maskbit(to + 1) - 1);
        }
    }

    /// Returns total number of bits.
    static constexpr size_t size() {
        return requested_size;
    }

    /// Returns number of bits set on.
    size_t count() const {
        static_assert(block_size == 64, "adjust popcount for block_type");
        size_t sum = 0;
        size_t i = 0;
        for (; i + 4 <= num_blocks; i += 4) {
            sum += popcount64(bits[i]);
            sum += popcount64(bits[i + 1]);
            sum += popcount64(bits[i + 2]);
            sum += popcount64(bits[i + 3]);
        }
        for (; i < num_blocks; i++) {
            sum += popcount64(bits[i]);
        }
        assert(sum <= size());
        return sum;
    }

    /// Are no bits set?
    bool none() const {
        for (const auto &e : bits) {
            if (e != 0) {
                return false;
            }
        }
        return true;
    }

    /// Is any bit set?
    bool any() const {
        return !none();
    }

    /// Are all bits set?
    bool all() const {
        for (size_t i = 0; i < bits.size() - 1; i++) {
            if (bits[i] != all_ones) {
                return false;
            }
        }
        size_t rem = requested_size % block_size;
        block_type exp = rem ? ((block_type{1} << rem) - 1) : all_ones;
        return *bits.rbegin() == exp;
    }

    /// Returns first bit set, or bitfield::npos if none set.
    size_t find_first() const {
        for (size_t i = 0; i < bits.size(); i++) {
            if (bits[i] != 0) {
                return (i * block_size) + word_ctz(i);
            }
        }
        return npos;
    }

    // Returns last bit set, or bitfield::npos if none set.
    size_t find_last() const {
        for (int i = bits.size() - 1; i >= 0; i--) {
            if (bits[i]) {
                static_assert(block_size == 64, "adjust clz for block_type");
                return (i * block_size) + block_size - 1 - clz64(bits[i]);
            }
        }
        return npos;
    }

    /// Returns next bit set, or bitfield::npos if none set after 'last'.
    size_t find_next(size_t last) const {
        if (last >= size()) {
            return npos;
        }

        // check current word.
        size_t i = getword(last);
        block_type lastword = bits[i];

        if ((last % block_size) != (block_size - 1)) {
            lastword &= (all_ones << ((last % block_size) + 1));

            if (lastword) {
                static_assert(block_size == 64, "adjust ctz for block_type");
                return (i * block_size) + ctz64(lastword);
            }
        }

        // check the rest.
        for (i++; i < bits.size(); i++) {
            if (bits[i]) {
                return (i * block_size) + word_ctz(i);
            }
        }

        return npos;
    }

    size_t find_nth(size_t n) const {
        assert(n < npos);

        static_assert(block_size == 64, "adjust for block_type");

        size_t sum = 0;
        for (size_t i = 0; i < bits.size(); i++) {
            block_type block = bits[i];
            size_t aftersum = sum + popcount64(block);
            if (aftersum > n) { // Block contains the nth bit.
                for (; sum < n; sum++) {
                    assert(block);
                    block &= (block - 1);
                }
                assert(block);
                size_t bit = (i * block_size) + ctz64(block);
                assert(test(bit));
                return bit;
            }
            sum = aftersum;
        }

        assert(count() < n + 1);
        return npos;
    }

    /// Bitwise OR.
    bitfield operator|(const bitfield &a) const {
        bitfield b = a;
        b |= *this;
        return b;
    }

    /// Bitwise OR-equals.
    void operator|=(const bitfield &a) {
        size_t i = 0;
        for (; i + 4 <= num_blocks; i += 4) {
            bits[i]     |= a.bits[i];
            bits[i + 1] |= a.bits[i + 1];
            bits[i + 2] |= a.bits[i + 2];
            bits[i + 3] |= a.bits[i + 3];
        }
        for (; i < num_blocks; i++) {
            bits[i] |= a.bits[i];
        }
    }

    /// Bitwise AND.
    bitfield operator&(const bitfield &a) const {
        bitfield b = a;
        b &= *this;
        return b;
    }

    /// Bitwise AND-equals.
    void operator&=(const bitfield &a) {
        size_t i = 0;
        for (; i + 4 <= num_blocks; i += 4) {
            bits[i]     &= a.bits[i];
            bits[i + 1] &= a.bits[i + 1];
            bits[i + 2] &= a.bits[i + 2];
            bits[i + 3] &= a.bits[i + 3];
        }
        for (; i < num_blocks; i++) {
            bits[i] &= a.bits[i];
        }
    }

    /// Bitwise XOR.
    bitfield operator^(bitfield a) const {
        a ^= *this;
        return a;
    }

    /// Bitwise XOR-equals.
    void operator^=(bitfield a) {
        size_t i = 0;
        for (; i + 4 <= num_blocks; i += 4) {
            bits[i]     ^= a.bits[i];
            bits[i + 1] ^= a.bits[i + 1];
            bits[i + 2] ^= a.bits[i + 2];
            bits[i + 3] ^= a.bits[i + 3];
        }
        for (; i < num_blocks; i++) {
            bits[i] ^= a.bits[i];
        }
    }

    /// Bitwise complement.
    bitfield operator~(void) const {
        bitfield cr(*this);
        cr.flip();
        return cr;
    }

    /// Simple hash.
    size_t hash() const {
        return ue2_hasher()(bits);
    }

    /// Sentinel value meaning "no more bits", used by find_first and
    /// find_next.
    static constexpr size_t npos = requested_size;

private:
    /// Underlying block type.
    using block_type = u64a;

    /// A block filled with on bits.
    static constexpr block_type all_ones = ~block_type{0};

    /// Size of a block.
    static constexpr size_t block_size = sizeof(block_type) * 8;

    static size_t getword(size_t n) {
        return n / block_size;
    }

    static block_type maskbit(size_t n) {
        return (block_type{1} << (n % block_size));
    }

    size_t word_ctz(size_t n) const {
        static_assert(block_size == 64, "adjust ctz call for block type");
        return ctz64(bits[n]);
    }

    /// Ensures that bits between our requested size and the end of storage are
    /// zero.
    void clear_trailer() {
        size_t final_bits = requested_size % block_size;
        if (final_bits) {
            bits.back() &= ((block_type{1} << final_bits) - 1);
        }
    }

    /// Size of storage array of blocks.
    static constexpr size_t num_blocks =
        (requested_size + block_size - 1) / block_size;

    /// Underlying storage.
    std::array<block_type, num_blocks> bits;
};

} // namespace ue2

namespace std {

template<size_t requested_size>
struct hash<ue2::bitfield<requested_size>> {
    size_t operator()(const ue2::bitfield<requested_size> &b) const {
        return b.hash();
    }
};

} // namespace std

#endif // BITFIELD_H
