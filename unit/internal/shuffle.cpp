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

#include "config.h"

#include "gtest/gtest.h"

#include "util/simd_utils.h"
#include "util/shuffle.h"
#include "util/shuffle_ssse3.h"

namespace {

// Switch one bit on in a bitmask.
template<class Mask>
Mask setbit(unsigned int bit) {
    union {
        Mask simd;
        char bytes[sizeof(Mask)];
    } cf;

    memset(cf.bytes, 0, sizeof(Mask));
    cf.bytes[bit / 8] = 1U << (bit % 8);

    return cf.simd;
}

TEST(Shuffle, ShuffleDynamic32_1) {
    // Try all possible one-bit masks
    for (unsigned int i = 0; i < 32; i++) {
        // shuffle a single 1 bit to the front
        u32 mask = 1U << i;
        EXPECT_EQ(1U, shuffleDynamic32(mask, mask));
        EXPECT_EQ(1U, shuffleDynamic32(~0U, mask));
        // we should get zero out of these cases
        EXPECT_EQ(0U, shuffleDynamic32(0, mask));
        EXPECT_EQ(0U, shuffleDynamic32(~mask, mask));
        // we should get zero out of all the other bit positions
        for (unsigned int j = 0; (j != i && j < 32); j++) {
            EXPECT_EQ(0U, shuffleDynamic32((1U << j), mask));
        }
    }
}

TEST(Shuffle, ShuffleDynamic32_2) {
    // All 32 bits in mask are on
    u32 mask = ~0U;
    EXPECT_EQ(0U, shuffleDynamic32(0, mask));
    EXPECT_EQ(mask, shuffleDynamic32(mask, mask));
    for (unsigned int i = 0; i < 32; i++) {
        EXPECT_EQ(1U << i, shuffleDynamic32(1U << i, mask));
    }
}

TEST(Shuffle, ShuffleDynamic32_3) {
    // Try setting every second bit
    u32 mask = 0;
    for (unsigned int i = 0; i < 32; i += 2) {
        mask |= 1U << i;
    }

    // Test both cases (all even bits, all odd bits)
    EXPECT_EQ((1U << 16) - 1, shuffleDynamic32(mask, mask));
    EXPECT_EQ((1U << 16) - 1, shuffleDynamic32(~mask, ~mask));
    EXPECT_EQ(0U, shuffleDynamic32(~mask, mask));
    EXPECT_EQ(0U, shuffleDynamic32(mask, ~mask));

    for (unsigned int i = 0; i < 32; i += 2) {
        EXPECT_EQ(1U << (i/2), shuffleDynamic32(1U << i, mask));
        EXPECT_EQ(0U, shuffleDynamic32(1U << i, ~mask));
        EXPECT_EQ(1U << (i/2), shuffleDynamic32(1U << (i+1), ~mask));
        EXPECT_EQ(0U, shuffleDynamic32(1U << (i+1), mask));
    }
}

TEST(Shuffle, ShuffleDynamic64_1) {
    // Try all possible one-bit masks
    for (unsigned int i = 0; i < 64; i++) {
        // shuffle a single 1 bit to the front
        u64a mask = 1ULL << i;
        EXPECT_EQ(1U, shuffleDynamic64(mask, mask));
        EXPECT_EQ(1U, shuffleDynamic64(~0ULL, mask));
        // we should get zero out of these cases
        EXPECT_EQ(0U, shuffleDynamic64(0, mask));
        EXPECT_EQ(0U, shuffleDynamic64(~mask, mask));
        // we should get zero out of all the other bit positions
        for (unsigned int j = 0; (j != i && j < 64); j++) {
            EXPECT_EQ(0U, shuffleDynamic64((1ULL << j), mask));
        }
    }
}

TEST(Shuffle, ShuffleDynamic64_2) {
    // Fill first half of mask
    u64a mask = 0x00000000ffffffffULL;
    EXPECT_EQ(0U, shuffleDynamic64(0, mask));
    EXPECT_EQ(0xffffffffU, shuffleDynamic64(mask, mask));
    for (unsigned int i = 0; i < 32; i++) {
        EXPECT_EQ(1U << i, shuffleDynamic64(1ULL << i, mask));
    }

    // Fill second half of mask
    mask = 0xffffffff00000000ULL;
    EXPECT_EQ(0U, shuffleDynamic64(0, mask));
    EXPECT_EQ(0xffffffffU, shuffleDynamic64(mask, mask));
    for (unsigned int i = 32; i < 64; i++) {
        EXPECT_EQ(1U << (i - 32), shuffleDynamic64(1ULL << i, mask));
    }

    // Try one in the middle
    mask = 0x0000ffffffff0000ULL;
    EXPECT_EQ(0U, shuffleDynamic64(0, mask));
    EXPECT_EQ(0xffffffffU, shuffleDynamic64(mask, mask));
    for (unsigned int i = 16; i < 48; i++) {
        EXPECT_EQ(1U << (i - 16), shuffleDynamic64(1ULL << i, mask));
    }
}

TEST(Shuffle, ShuffleDynamic64_3) {
    // Try setting every second bit (note: 32 bits, the max we can shuffle)
    u64a mask = 0;
    for (unsigned int i = 0; i < 64; i += 2) {
        mask |= 1ULL << i;
    }

    // Test both cases (all even bits, all odd bits)
    EXPECT_EQ(0xffffffffU, shuffleDynamic64(mask, mask));
    EXPECT_EQ(0xffffffffU, shuffleDynamic64(~mask, ~mask));
    EXPECT_EQ(0U, shuffleDynamic64(~mask, mask));
    EXPECT_EQ(0U, shuffleDynamic64(mask, ~mask));

    for (unsigned int i = 0; i < 64; i += 2) {
        EXPECT_EQ(1U << (i/2), shuffleDynamic64(1ULL << i, mask));
        EXPECT_EQ(0U, shuffleDynamic64(1ULL << i, ~mask));
        EXPECT_EQ(1U << (i/2), shuffleDynamic64(1ULL << (i+1), ~mask));
        EXPECT_EQ(0U, shuffleDynamic64(1ULL << (i+1), mask));
    }
}

static
void build_pshufb_masks_onebit(unsigned int bit, m128 *permute, m128 *compare) {
    // permute mask has 0x80 in all bytes except the one we care about
    memset(permute, 0x80, sizeof(*permute));
    memset(compare, 0, sizeof(*compare));
    char *pmsk = (char *)permute;
    char *cmsk = (char *)compare;
    pmsk[0] = bit/8;
    cmsk[0] = ~(1 << (bit % 8));
}

TEST(Shuffle, ShufflePshufb128_1) {
    // Try all possible one-bit masks
    for (unsigned int i = 0; i < 128; i++) {
        // shuffle a single 1 bit to the front
        m128 permute, compare;
        build_pshufb_masks_onebit(i, &permute, &compare);
        EXPECT_EQ(1U, shufflePshufb128(setbit<m128>(i), permute, compare));
        EXPECT_EQ(1U, shufflePshufb128(ones128(), permute, compare));
        // we should get zero out of these cases
        EXPECT_EQ(0U, shufflePshufb128(zeroes128(), permute, compare));
        EXPECT_EQ(0U, shufflePshufb128(not128(setbit<m128>(i)), permute, compare));
        // we should get zero out of all the other bit positions
        for (unsigned int j = 0; (j != i && j < 128); j++) {
            EXPECT_EQ(0U, shufflePshufb128(setbit<m128>(j), permute, compare));
        }
    }
}

} // namespace
