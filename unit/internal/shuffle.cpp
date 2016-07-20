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
#include "util/simd_utils.h"
#include "nfa/limex_shuffle.h"

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

TEST(Shuffle, PackedExtract32_1) {
    // Try all possible one-bit masks
    for (unsigned int i = 0; i < 32; i++) {
        // shuffle a single 1 bit to the front
        u32 mask = 1U << i;
        EXPECT_EQ(1U, pext32(mask, mask));
        EXPECT_EQ(1U, pext32(~0U, mask));
        // we should get zero out of these cases
        EXPECT_EQ(0U, pext32(0, mask));
        EXPECT_EQ(0U, pext32(~mask, mask));
        // we should get zero out of all the other bit positions
        for (unsigned int j = 0; (j != i && j < 32); j++) {
            EXPECT_EQ(0U, pext32((1U << j), mask));
        }
    }
}

TEST(Shuffle, PackedExtract32_2) {
    // All 32 bits in mask are on
    u32 mask = ~0U;
    EXPECT_EQ(0U, pext32(0, mask));
    EXPECT_EQ(mask, pext32(mask, mask));
    for (unsigned int i = 0; i < 32; i++) {
        EXPECT_EQ(1U << i, pext32(1U << i, mask));
    }
}

TEST(Shuffle, PackedExtract32_3) {
    // Try setting every second bit
    u32 mask = 0;
    for (unsigned int i = 0; i < 32; i += 2) {
        mask |= 1U << i;
    }

    // Test both cases (all even bits, all odd bits)
    EXPECT_EQ((1U << 16) - 1, pext32(mask, mask));
    EXPECT_EQ((1U << 16) - 1, pext32(~mask, ~mask));
    EXPECT_EQ(0U, pext32(~mask, mask));
    EXPECT_EQ(0U, pext32(mask, ~mask));

    for (unsigned int i = 0; i < 32; i += 2) {
        EXPECT_EQ(1U << (i/2), pext32(1U << i, mask));
        EXPECT_EQ(0U, pext32(1U << i, ~mask));
        EXPECT_EQ(1U << (i/2), pext32(1U << (i+1), ~mask));
        EXPECT_EQ(0U, pext32(1U << (i+1), mask));
    }
}

TEST(Shuffle, PackedExtract64_1) {
    // Try all possible one-bit masks
    for (unsigned int i = 0; i < 64; i++) {
        // shuffle a single 1 bit to the front
        u64a mask = 1ULL << i;
        EXPECT_EQ(1U, pext64(mask, mask));
        EXPECT_EQ(1U, pext64(~0ULL, mask));
        // we should get zero out of these cases
        EXPECT_EQ(0U, pext64(0, mask));
        EXPECT_EQ(0U, pext64(~mask, mask));
        // we should get zero out of all the other bit positions
        for (unsigned int j = 0; (j != i && j < 64); j++) {
            EXPECT_EQ(0U, pext64((1ULL << j), mask));
        }
    }
}

TEST(Shuffle, PackedExtract64_2) {
    // Fill first half of mask
    u64a mask = 0x00000000ffffffffULL;
    EXPECT_EQ(0U, pext64(0, mask));
    EXPECT_EQ(0xffffffffU, pext64(mask, mask));
    for (unsigned int i = 0; i < 32; i++) {
        EXPECT_EQ(1U << i, pext64(1ULL << i, mask));
    }

    // Fill second half of mask
    mask = 0xffffffff00000000ULL;
    EXPECT_EQ(0U, pext64(0, mask));
    EXPECT_EQ(0xffffffffU, pext64(mask, mask));
    for (unsigned int i = 32; i < 64; i++) {
        EXPECT_EQ(1U << (i - 32), pext64(1ULL << i, mask));
    }

    // Try one in the middle
    mask = 0x0000ffffffff0000ULL;
    EXPECT_EQ(0U, pext64(0, mask));
    EXPECT_EQ(0xffffffffU, pext64(mask, mask));
    for (unsigned int i = 16; i < 48; i++) {
        EXPECT_EQ(1U << (i - 16), pext64(1ULL << i, mask));
    }
}

TEST(Shuffle, PackedExtract64_3) {
    // Try setting every second bit (note: 32 bits, the max we can shuffle)
    u64a mask = 0;
    for (unsigned int i = 0; i < 64; i += 2) {
        mask |= 1ULL << i;
    }

    // Test both cases (all even bits, all odd bits)
    EXPECT_EQ(0xffffffffU, pext64(mask, mask));
    EXPECT_EQ(0xffffffffU, pext64(~mask, ~mask));
    EXPECT_EQ(0U, pext64(~mask, mask));
    EXPECT_EQ(0U, pext64(mask, ~mask));

    for (unsigned int i = 0; i < 64; i += 2) {
        EXPECT_EQ(1U << (i/2), pext64(1ULL << i, mask));
        EXPECT_EQ(0U, pext64(1ULL << i, ~mask));
        EXPECT_EQ(1U << (i/2), pext64(1ULL << (i+1), ~mask));
        EXPECT_EQ(0U, pext64(1ULL << (i+1), mask));
    }
}

template<typename T>
static
void build_pshufb_masks_onebit(unsigned int bit, T *permute, T *compare) {
    static_assert(sizeof(T) == sizeof(m128) || sizeof(T) == sizeof(m256) ||
                      sizeof(T) == sizeof(m512),
                  "should be valid type");
    // permute mask has 0x80 in all bytes except the one we care about
    memset(permute, 0x80, sizeof(*permute));
    memset(compare, 0, sizeof(*compare));
    char *pmsk = (char *)permute;
    char *cmsk = (char *)compare;
    u8 off = (bit >= 128) ? (bit >= 256) ? (bit >= 384) ? 0x30 : 0x20 : 0x10 : 0;
    pmsk[off] = bit/8;
    cmsk[off] = ~(1 << (bit % 8));
}

TEST(Shuffle, PackedExtract128_1) {
    // Try all possible one-bit masks
    for (unsigned int i = 0; i < 128; i++) {
        // shuffle a single 1 bit to the front
        m128 permute, compare;
        build_pshufb_masks_onebit(i, &permute, &compare);
        EXPECT_EQ(1U, packedExtract128(setbit<m128>(i), permute, compare));
        EXPECT_EQ(1U, packedExtract128(ones128(), permute, compare));
        // we should get zero out of these cases
        EXPECT_EQ(0U, packedExtract128(zeroes128(), permute, compare));
        EXPECT_EQ(0U, packedExtract128(not128(setbit<m128>(i)), permute, compare));
        // we should get zero out of all the other bit positions
        for (unsigned int j = 0; (j != i && j < 128); j++) {
            EXPECT_EQ(0U, packedExtract128(setbit<m128>(j), permute, compare));
        }
    }
}

#if defined(HAVE_AVX2)
TEST(Shuffle, PackedExtract256_1) {
    // Try all possible one-bit masks
    for (unsigned int i = 0; i < 256; i++) {
        // shuffle a single 1 bit to the front
        m256 permute, compare;
        build_pshufb_masks_onebit(i, &permute, &compare);
        EXPECT_EQ(1U, packedExtract256(setbit<m256>(i), permute, compare));
        EXPECT_EQ(1U, packedExtract256(ones256(), permute, compare));
        // we should get zero out of these cases
        EXPECT_EQ(0U, packedExtract256(zeroes256(), permute, compare));
        EXPECT_EQ(0U, packedExtract256(not256(setbit<m256>(i)), permute, compare));
        // we should get zero out of all the other bit positions
        for (unsigned int j = 0; (j != i && j < 256); j++) {
            EXPECT_EQ(0U, packedExtract256(setbit<m256>(j), permute, compare));
        }
    }
}
#endif

#if defined(HAVE_AVX512)
TEST(Shuffle, PackedExtract512_1) {
    // Try all possible one-bit masks
    for (unsigned int i = 0; i < 512; i++) {
        // shuffle a single 1 bit to the front
        m512 permute, compare;
        build_pshufb_masks_onebit(i, &permute, &compare);
        EXPECT_EQ(1U, packedExtract512(setbit<m512>(i), permute, compare));
        EXPECT_EQ(1U, packedExtract512(ones512(), permute, compare));
        // we should get zero out of these cases
        EXPECT_EQ(0U, packedExtract512(zeroes512(), permute, compare));
        EXPECT_EQ(0U, packedExtract512(not512(setbit<m512>(i)), permute, compare));
        // we should get zero out of all the other bit positions
        for (unsigned int j = 0; (j != i && j < 512); j++) {
            EXPECT_EQ(0U, packedExtract512(setbit<m512>(j), permute, compare));
        }
    }
}
#endif
} // namespace
