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

#include "config.h"

#include "gtest/gtest.h"
#include "util/arch.h"
#include "util/bitutils.h"
#include "util/popcount.h"

// open coded implementations to test against
static
u32 our_clz(u32 x) {
    u32 n;

    if (x == 0) return(32);
    n = 0;
    if (x <= 0x0000FFFF) { n = n + 16; x = x << 16; }
    if (x <= 0x00FFFFFF) { n = n + 8; x = x << 8; }
    if (x <= 0x0FFFFFFF) { n = n + 4; x = x << 4; }
    if (x <= 0x3FFFFFFF) { n = n + 2; x = x << 2; }
    if (x <= 0x7FFFFFFF) { n = n + 1; }
    return n;
}

static
u32 our_clzll(u64a x) {
    // Synthesise from 32-bit variant.
    u32 high = x >> 32;
    if (high) {
        return our_clz(high);
    }
    return 32 + our_clz(x);
}


TEST(BitUtils, findAndClearLSB32_1) {
    // test that it can find every single-bit case
    for (unsigned int i = 0; i < 32; i++) {
        u32 input = 1 << i;
        u32 idx = findAndClearLSB_32(&input);
        EXPECT_EQ(i, idx);
        EXPECT_EQ(0U, input);
    }
}

TEST(BitUtils, findAndClearLSB32_2) {
    // test that we get all 32 indices from an all-ones input
    u32 input = ~0;
    unsigned int count = 0;
    while (input != 0) {
        ASSERT_GT(32U, count);
        u32 expected_output = input & (input - 1);
        u32 idx = findAndClearLSB_32(&input);
        EXPECT_EQ(count, idx);
        EXPECT_EQ(expected_output, input);
        count++;
    }
    EXPECT_EQ(32U, count);
}

TEST(BitUtils, findAndClearLSB64_1) {
    // test that it can find every single-bit case
    for (unsigned int i = 0; i < 64; i++) {
        u64a input = 1ULL << i;
        u32 idx = findAndClearLSB_64(&input);
        EXPECT_EQ(i, idx);
        EXPECT_EQ(0U, input);
    }
}

TEST(BitUtils, findAndClearLSB64_2) {
    // test that we get all 32 indices from an all-ones input
    u64a input = ~0;
    unsigned int count = 0;
    while (input != 0) {
        ASSERT_GT(64U, count);
        u64a expected_output = input & (input - 1);
        u32 idx = findAndClearLSB_64(&input);
        EXPECT_EQ(count, idx);
        EXPECT_EQ(expected_output, input);
        count++;
    }
    EXPECT_EQ(64U, count);
}

TEST(BitUtils, findAndClearMSB32_1) {
    // test that it can find every single-bit case
    for (unsigned int i = 0; i < 32; i++) {
        u32 input = 1 << i;
        u32 idx = findAndClearMSB_32(&input);
        EXPECT_EQ(i, idx);
        EXPECT_EQ(0U, input);
    }
}

TEST(BitUtils, findAndClearMSB32_2) {
    // test that we get all 32 indices from an all-ones input
    u32 input = ~0;
    unsigned int count = 0;
    while (input != 0) {
        ASSERT_GT(32U, count);
        u32 offset = our_clz(input);
        u32 expected_output = input ^ (1ULL << (31 - our_clz(input)));
        u32 idx = findAndClearMSB_32(&input);
        EXPECT_EQ(offset, count);
        EXPECT_EQ(offset, 31 - idx);
        EXPECT_EQ(expected_output, input);
        count++;
    }
    EXPECT_EQ(32U, count);
}

TEST(BitUtils, findAndClearMSB64_1) {
    // test that it can find every single-bit case
    for (unsigned int i = 0; i < 64; i++) {
        u64a input = 1ULL << i;
        u32 idx = findAndClearMSB_64(&input);
        EXPECT_EQ(i, idx);
        EXPECT_EQ(0U, input);
    }
}

TEST(BitUtils, findAndClearMSB64_2) {
    // test that we get all 64 indices from an all-ones input
    u64a input = ~0;
    unsigned int count = 0;
    while (input != 0) {
        u32 offset = our_clzll(input);
        u64a expected_output = input ^ (1ULL << (63 - our_clzll(input)));
        u32 idx = findAndClearMSB_64(&input);
        EXPECT_EQ(offset, 63 - idx);
        EXPECT_EQ(offset, count);
        EXPECT_EQ(expected_output, input);
        count++;
    }
    EXPECT_EQ(64U, count);
}

TEST(BitUtils, popcount32) {
    // some simple tests
    EXPECT_EQ(0U,  popcount32(0));
    EXPECT_EQ(16U, popcount32(0x0000ffffU));
    EXPECT_EQ(16U, popcount32(0xffff0000U));
    EXPECT_EQ(16U, popcount32(0xf0f0f0f0U));
    EXPECT_EQ(32U, popcount32(0xffffffffU));
    EXPECT_EQ(32U, popcount32(0xffffffffU));

    // single bits
    for (u32 i = 0; i < 32; i++) {
        EXPECT_EQ(1U, popcount32(1U << i));
    }

    // random
    EXPECT_EQ(13U, popcount32(668732496U));
    EXPECT_EQ(15U, popcount32(872800073U));
    EXPECT_EQ(15U, popcount32(2831496658U));
    EXPECT_EQ(12U, popcount32(2895271296U));
}

TEST(BitUtils, popcount64) {
    // some simple tests
    EXPECT_EQ(0U,  popcount64(0));
    EXPECT_EQ(16U, popcount64(0x000000000000ffffULL));
    EXPECT_EQ(16U, popcount64(0xffff000000000000ULL));
    EXPECT_EQ(32U, popcount64(0xf0f0f0f0f0f0f0f0ULL));
    EXPECT_EQ(32U, popcount64(0xffffffff00000000ULL));
    EXPECT_EQ(32U, popcount64(0x00000000ffffffffULL));
    EXPECT_EQ(64U, popcount64(0xffffffffffffffffULL));

    // single bits
    for (u32 i = 0; i < 64; i++) {
        EXPECT_EQ(1U, popcount64(1ULL << i));
    }

    // random
    EXPECT_EQ(13U, popcount64(668732496ULL));
    EXPECT_EQ(15U, popcount64(872800073ULL));
    EXPECT_EQ(15U, popcount64(2831496658ULL));
    EXPECT_EQ(12U, popcount64(2895271296ULL));
    EXPECT_EQ(25U, popcount64(1869119247322218496ULL));
    EXPECT_EQ(31U, popcount64(16235371884097357824ULL));
    EXPECT_EQ(17U, popcount64(9354015527977637888ULL));
}

TEST(BitUtils, clz32) {
    for (u32 i = 0; i < 32; i++) {
        EXPECT_EQ(31 - i, clz32(1 << i));
    }
}

TEST(BitUtils, clz64) {
    for (u64a i = 0; i < 64; i++) {
        EXPECT_EQ(63 - i, clz64(1ULL << i));
    }
}

TEST(BitUtils, ctz32) {
    EXPECT_EQ(0U, ctz32(0x1U));
    EXPECT_EQ(15U, ctz32(0x8000));
}

TEST(BitUtils, ctz64) {
    EXPECT_EQ(0U, ctz64(0x1ULL));
    EXPECT_EQ(15U, ctz64(0x8000ULL));
}

TEST(BitUtils, compress32) {
    const u32 all_ones = 0xffffffffu;
    const u32 odd_bits = 0x55555555u;
    const u32 even_bits = 0xaaaaaaaau;

    EXPECT_EQ(0, compress32(0, 0));
    EXPECT_EQ(0, compress32(0, 1u));
    EXPECT_EQ(0, compress32(0, all_ones));
    EXPECT_EQ(all_ones, compress32(all_ones, all_ones));
    EXPECT_EQ(0xffffu, compress32(odd_bits, odd_bits));
    EXPECT_EQ(0xffffu, compress32(even_bits, even_bits));
    EXPECT_EQ(0, compress32(odd_bits, even_bits));
    EXPECT_EQ(0, compress32(even_bits, odd_bits));

    // Some single-bit tests.
    for (u32 i = 0; i < 32; i++) {
        const u32 one_bit = 1u << i;

        EXPECT_EQ(0, compress32(0, one_bit));
        EXPECT_EQ(1u, compress32(one_bit, one_bit));
        EXPECT_EQ(one_bit, compress32(one_bit, all_ones));

        if (i % 2) {
            EXPECT_EQ(1u << (i / 2), compress32(one_bit, even_bits));
            EXPECT_EQ(0, compress32(one_bit, odd_bits));
        } else {
            EXPECT_EQ(1u << (i / 2), compress32(one_bit, odd_bits));
            EXPECT_EQ(0, compress32(one_bit, even_bits));
        }
    }
}

TEST(BitUtils, compress64) {
    const u64a all_ones = 0xffffffffffffffffull;
    const u64a odd_bits = 0x5555555555555555ull;
    const u64a even_bits = 0xaaaaaaaaaaaaaaaaull;

    EXPECT_EQ(0, compress64(0, 0));
    EXPECT_EQ(0, compress64(0, 1u));
    EXPECT_EQ(0, compress64(0, all_ones));
    EXPECT_EQ(all_ones, compress64(all_ones, all_ones));
    EXPECT_EQ(0xffffffffull, compress64(odd_bits, odd_bits));
    EXPECT_EQ(0xffffffffull, compress64(even_bits, even_bits));
    EXPECT_EQ(0, compress64(odd_bits, even_bits));
    EXPECT_EQ(0, compress64(even_bits, odd_bits));

    // Some single-bit tests.
    for (u32 i = 0; i < 64; i++) {
        const u64a one_bit = 1ull << i;

        EXPECT_EQ(0, compress64(0, one_bit));
        EXPECT_EQ(1ull, compress64(one_bit, one_bit));
        EXPECT_EQ(one_bit, compress64(one_bit, all_ones));

        if (i % 2) {
            EXPECT_EQ(1ull << (i / 2), compress64(one_bit, even_bits));
            EXPECT_EQ(0, compress64(one_bit, odd_bits));
        } else {
            EXPECT_EQ(1ull << (i / 2), compress64(one_bit, odd_bits));
            EXPECT_EQ(0, compress64(one_bit, even_bits));
        }
    }
}

TEST(BitUtils, expand32) {
    const u32 all_ones = 0xffffffffu;
    const u32 odd_bits = 0x55555555u;
    const u32 even_bits = 0xaaaaaaaau;

    EXPECT_EQ(0, expand32(0, 0));
    EXPECT_EQ(0, expand32(0, 1u));
    EXPECT_EQ(0, expand32(0, all_ones));
    EXPECT_EQ(all_ones, expand32(all_ones, all_ones));
    EXPECT_EQ(odd_bits, expand32(0xffffu, odd_bits));
    EXPECT_EQ(even_bits, expand32(0xffffu, even_bits));
    EXPECT_EQ(0, expand32(0xffff0000u, even_bits));
    EXPECT_EQ(0, expand32(0xffff0000u, odd_bits));
    EXPECT_EQ(1u, expand32(1u, odd_bits));
    EXPECT_EQ(2u, expand32(1u, even_bits));

    // Some single-bit tests.
    for (u32 i = 0; i < 32; i++) {
        const u32 one_bit = 1u << i;

        EXPECT_EQ(0, expand32(0, one_bit));
        EXPECT_EQ(one_bit, expand32(1u, one_bit));
        EXPECT_EQ(one_bit, expand32(one_bit, all_ones));

        EXPECT_EQ(one_bit,
                  expand32(1u << (i / 2), i % 2 ? even_bits : odd_bits));
    }
}

TEST(BitUtils, expand64) {
    const u64a all_ones = 0xffffffffffffffffull;
    const u64a odd_bits = 0x5555555555555555ull;
    const u64a even_bits = 0xaaaaaaaaaaaaaaaaull;

    EXPECT_EQ(0, expand64(0, 0));
    EXPECT_EQ(0, expand64(0, 1ull));
    EXPECT_EQ(0, expand64(0, all_ones));
    EXPECT_EQ(all_ones, expand64(all_ones, all_ones));
    EXPECT_EQ(odd_bits, expand64(0xffffffffull, odd_bits));
    EXPECT_EQ(even_bits, expand64(0xffffffffull, even_bits));
    EXPECT_EQ(0, expand64(0xffffffff00000000ull, even_bits));
    EXPECT_EQ(0, expand64(0xffffffff00000000ull, odd_bits));
    EXPECT_EQ(1u, expand64(1u, odd_bits));
    EXPECT_EQ(2u, expand64(1u, even_bits));

    // Some single-bit tests.
    for (u32 i = 0; i < 64; i++) {
        const u64a one_bit = 1ull << i;

        EXPECT_EQ(0, expand64(0, one_bit));
        EXPECT_EQ(one_bit, expand64(1ull, one_bit));
        EXPECT_EQ(one_bit, expand64(one_bit, all_ones));

        EXPECT_EQ(one_bit,
                  expand64(1ull << (i / 2), i % 2 ? even_bits : odd_bits));
    }
}

TEST(BitUtils, bf_op_1) {
    u64a a = 0;
    for (u32 i = 0; i < 64; i++) {
        char rv = bf64_set(&a, i);
        ASSERT_EQ(0, rv);
        if (i == 63) {
            ASSERT_EQ(~0ULL, a);
        } else {
            ASSERT_EQ((1ULL << (i + 1)) - 1, a);
        }

        rv = bf64_set(&a, i);
        ASSERT_EQ(1, rv);
        if (i == 63) {
            ASSERT_EQ(~0ULL, a);
        } else {
            ASSERT_EQ((1ULL << (i + 1)) - 1, a);
        }

        u64a b = 0;
        rv = bf64_set(&b, i);
        ASSERT_EQ(0, rv);
        ASSERT_EQ(1ULL << i, b);

        rv = bf64_set(&b, i);
        ASSERT_EQ(1, rv);
        ASSERT_EQ(1ULL << i, b);

        bf64_unset(&b, i);
        ASSERT_EQ(0, b);
    }
    ASSERT_EQ(~0ULL, a);
}

TEST(BitUtils, bf_it_1) {
    ASSERT_EQ(~0U, bf64_iterate(0ULL, ~0U));
    ASSERT_EQ(~0U, bf64_iterate(0ULL, 0));
    ASSERT_EQ(~0U, bf64_iterate(0ULL, 1));
    ASSERT_EQ(~0U, bf64_iterate(0ULL, 63));

    ASSERT_EQ(0, bf64_iterate(1ULL, ~0U));
    ASSERT_EQ(~0U, bf64_iterate(1ULL, 0));
    ASSERT_EQ(~0U, bf64_iterate(1ULL, 1));
    ASSERT_EQ(~0U, bf64_iterate(1ULL, 63));

    ASSERT_EQ(1, bf64_iterate(2ULL, ~0U));
    ASSERT_EQ(1, bf64_iterate(2ULL, 0));
    ASSERT_EQ(~0U, bf64_iterate(2ULL, 1));
    ASSERT_EQ(~0U, bf64_iterate(2ULL, 63));

    ASSERT_EQ(0, bf64_iterate(3ULL, ~0U));
    ASSERT_EQ(1, bf64_iterate(3ULL, 0));
    ASSERT_EQ(~0U, bf64_iterate(3ULL, 1));
    ASSERT_EQ(~0U, bf64_iterate(3ULL, 63));

    ASSERT_EQ(63, bf64_iterate(1ULL << 63, ~0U));
    ASSERT_EQ(63, bf64_iterate(1ULL << 63, 0));
    ASSERT_EQ(63, bf64_iterate(1ULL << 63, 1));
    ASSERT_EQ(~0U, bf64_iterate(1ULL << 63, 63));
}

TEST(BitUtils, rank_in_mask32) {
    for (u32 i = 0; i < 32; i++) {
        ASSERT_EQ(i, rank_in_mask32(0xffffffff, i));
        ASSERT_EQ(0, rank_in_mask32(1U << i, i));
    }
    ASSERT_EQ(0, rank_in_mask32(0xf0f0f0f0, 4));
    ASSERT_EQ(1, rank_in_mask32(0xf0f0f0f0, 5));
    ASSERT_EQ(3, rank_in_mask32(0xf0f0f0f0, 7));
    ASSERT_EQ(7, rank_in_mask32(0xf0f0f0f0, 15));
    ASSERT_EQ(15, rank_in_mask32(0xf0f0f0f0, 31));
}

TEST(BitUtils, rank_in_mask64) {
    for (u32 i = 0; i < 64; i++) {
        ASSERT_EQ(i, rank_in_mask64(0xffffffffffffffffULL, i));
        ASSERT_EQ(0, rank_in_mask64(1ULL << i, i));
    }
    ASSERT_EQ(0, rank_in_mask64(0xf0f0f0f0f0f0f0f0ULL, 4));
    ASSERT_EQ(1, rank_in_mask64(0xf0f0f0f0f0f0f0f0ULL, 5));
    ASSERT_EQ(3, rank_in_mask64(0xf0f0f0f0f0f0f0f0ULL, 7));
    ASSERT_EQ(7, rank_in_mask64(0xf0f0f0f0f0f0f0f0ULL, 15));
    ASSERT_EQ(15, rank_in_mask64(0xf0f0f0f0f0f0f0f0ULL, 31));
    ASSERT_EQ(31, rank_in_mask64(0xf0f0f0f0f0f0f0f0ULL, 63));
}

#if defined(HAVE_BMI2) && defined(ARCH_64_BIT)
TEST(BitUtils, pdep64) {
    u64a data = 0xF123456789ABCDEF;
    ASSERT_EQ(0xfULL, pdep64(data, 0xf));
    ASSERT_EQ(0xefULL, pdep64(data, 0xff));
    ASSERT_EQ(0xf0ULL, pdep64(data, 0xf0));
    ASSERT_EQ(0xfULL, pdep64(data, 0xf));
    ASSERT_EQ(0xef0ULL, pdep64(data, 0xff0));
    ASSERT_EQ(0xef00ULL, pdep64(data, 0xff00));
    ASSERT_EQ(0xd0e0f00ULL, pdep64(data, 0xf0f0f00));
}
#endif
