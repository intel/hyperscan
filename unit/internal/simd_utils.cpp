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
#include "util/bytecode_ptr.h"
#include "util/make_unique.h"
#include "util/simd_utils.h"

using namespace std;
using namespace ue2;

namespace {

// Switch one bit on in a bitmask.
template<class Mask>
Mask setbit(unsigned int bit) {
    union {
        Mask simd;
        char bytes[sizeof(Mask)];
    } cf;

    memset(cf.bytes, 0, sizeof(Mask));

    unsigned int byte_idx = bit / 8;
    cf.bytes[byte_idx] = 1U << (bit % 8);

    return cf.simd;
}

// Parameterized tests follow!
//
// Irritatingly we have to define a whole bunch of overrides here... because
// templates. One Admiration Unit for anyone able to build a better way of
// doing this.

struct simd_zeroes {
    operator m128() { return zeroes128(); }
    operator m256() { return zeroes256(); }
    operator m384() { return zeroes384(); }
    operator m512() { return zeroes512(); }
};

struct simd_ones {
    operator m128() { return ones128(); }
    operator m256() { return ones256(); }
    operator m384() { return ones384(); }
    operator m512() { return ones512(); }
};

bool simd_diff(const m128 &a, const m128 &b) { return !!diff128(a, b); }
bool simd_diff(const m256 &a, const m256 &b) { return !!diff256(a, b); }
bool simd_diff(const m384 &a, const m384 &b) { return !!diff384(a, b); }
bool simd_diff(const m512 &a, const m512 &b) { return !!diff512(a, b); }
bool simd_isnonzero(const m128 &a) { return !!isnonzero128(a); }
bool simd_isnonzero(const m256 &a) { return !!isnonzero256(a); }
bool simd_isnonzero(const m384 &a) { return !!isnonzero384(a); }
bool simd_isnonzero(const m512 &a) { return !!isnonzero512(a); }
m128 simd_and(const m128 &a, const m128 &b) { return and128(a, b); }
m256 simd_and(const m256 &a, const m256 &b) { return and256(a, b); }
m384 simd_and(const m384 &a, const m384 &b) { return and384(a, b); }
m512 simd_and(const m512 &a, const m512 &b) { return and512(a, b); }
m128 simd_or(const m128 &a, const m128 &b) { return or128(a, b); }
m256 simd_or(const m256 &a, const m256 &b) { return or256(a, b); }
m384 simd_or(const m384 &a, const m384 &b) { return or384(a, b); }
m512 simd_or(const m512 &a, const m512 &b) { return or512(a, b); }
m128 simd_xor(const m128 &a, const m128 &b) { return xor128(a, b); }
m256 simd_xor(const m256 &a, const m256 &b) { return xor256(a, b); }
m384 simd_xor(const m384 &a, const m384 &b) { return xor384(a, b); }
m512 simd_xor(const m512 &a, const m512 &b) { return xor512(a, b); }
m128 simd_andnot(const m128 &a, const m128 &b) { return andnot128(a, b); }
m256 simd_andnot(const m256 &a, const m256 &b) { return andnot256(a, b); }
m384 simd_andnot(const m384 &a, const m384 &b) { return andnot384(a, b); }
m512 simd_andnot(const m512 &a, const m512 &b) { return andnot512(a, b); }
m128 simd_not(const m128 &a) { return not128(a); }
m256 simd_not(const m256 &a) { return not256(a); }
m384 simd_not(const m384 &a) { return not384(a); }
m512 simd_not(const m512 &a) { return not512(a); }
void simd_clearbit(m128 *a, unsigned int i) { return clearbit128(a, i); }
void simd_clearbit(m256 *a, unsigned int i) { return clearbit256(a, i); }
void simd_clearbit(m384 *a, unsigned int i) { return clearbit384(a, i); }
void simd_clearbit(m512 *a, unsigned int i) { return clearbit512(a, i); }
void simd_setbit(m128 *a, unsigned int i) { return setbit128(a, i); }
void simd_setbit(m256 *a, unsigned int i) { return setbit256(a, i); }
void simd_setbit(m384 *a, unsigned int i) { return setbit384(a, i); }
void simd_setbit(m512 *a, unsigned int i) { return setbit512(a, i); }
bool simd_testbit(const m128 &a, unsigned int i) { return testbit128(a, i); }
bool simd_testbit(const m256 &a, unsigned int i) { return testbit256(a, i); }
bool simd_testbit(const m384 &a, unsigned int i) { return testbit384(a, i); }
bool simd_testbit(const m512 &a, unsigned int i) { return testbit512(a, i); }
u32 simd_diffrich(const m128 &a, const m128 &b) { return diffrich128(a, b); }
u32 simd_diffrich(const m256 &a, const m256 &b) { return diffrich256(a, b); }
u32 simd_diffrich(const m384 &a, const m384 &b) { return diffrich384(a, b); }
u32 simd_diffrich(const m512 &a, const m512 &b) { return diffrich512(a, b); }
u32 simd_diffrich64(const m128 &a, const m128 &b) { return diffrich64_128(a, b); }
u32 simd_diffrich64(const m256 &a, const m256 &b) { return diffrich64_256(a, b); }
u32 simd_diffrich64(const m384 &a, const m384 &b) { return diffrich64_384(a, b); }
u32 simd_diffrich64(const m512 &a, const m512 &b) { return diffrich64_512(a, b); }
void simd_store(void *ptr, const m128 &a) { store128(ptr, a); }
void simd_store(void *ptr, const m256 &a) { store256(ptr, a); }
void simd_store(void *ptr, const m384 &a) { store384(ptr, a); }
void simd_store(void *ptr, const m512 &a) { store512(ptr, a); }
void simd_load(m128 *a, const void *ptr) { *a = load128(ptr); }
void simd_load(m256 *a, const void *ptr) { *a = load256(ptr); }
void simd_load(m384 *a, const void *ptr) { *a = load384(ptr); }
void simd_load(m512 *a, const void *ptr) { *a = load512(ptr); }
void simd_loadu(m128 *a, const void *ptr) { *a = loadu128(ptr); }
void simd_loadu(m256 *a, const void *ptr) { *a = loadu256(ptr); }
void simd_loadu(m384 *a, const void *ptr) { *a = loadu384(ptr); }
void simd_loadu(m512 *a, const void *ptr) { *a = loadu512(ptr); }
void simd_storebytes(void *ptr, const m128 &a, unsigned i) { storebytes128(ptr, a, i); }
void simd_storebytes(void *ptr, const m256 &a, unsigned i) { storebytes256(ptr, a, i); }
void simd_storebytes(void *ptr, const m384 &a, unsigned i) { storebytes384(ptr, a, i); }
void simd_storebytes(void *ptr, const m512 &a, unsigned i) { storebytes512(ptr, a, i); }
void simd_loadbytes(m128 *a, const void *ptr, unsigned i) { *a = loadbytes128(ptr, i); }
void simd_loadbytes(m256 *a, const void *ptr, unsigned i) { *a = loadbytes256(ptr, i); }
void simd_loadbytes(m384 *a, const void *ptr, unsigned i) { *a = loadbytes384(ptr, i); }
void simd_loadbytes(m512 *a, const void *ptr, unsigned i) { *a = loadbytes512(ptr, i); }
m128 simd_lshift64(const m128 &a, unsigned i) { return lshift64_m128(a, i); }
m256 simd_lshift64(const m256 &a, unsigned i) { return lshift64_m256(a, i); }
m384 simd_lshift64(const m384 &a, unsigned i) { return lshift64_m384(a, i); }
m512 simd_lshift64(const m512 &a, unsigned i) { return lshift64_m512(a, i); }

template<typename T>
class SimdUtilsTest : public testing::Test {
    // empty
};

typedef ::testing::Types<m128, m256, m384, m512> SimdTypes;
TYPED_TEST_CASE(SimdUtilsTest, SimdTypes);

//
// The tests themselves.
//

TYPED_TEST(SimdUtilsTest, zero) {
    const TypeParam zeroes = simd_zeroes();

    // Should have no bits on.
    char cmp[sizeof(zeroes)];
    memset(cmp, 0, sizeof(zeroes));
    ASSERT_EQ(0, memcmp(cmp, &zeroes, sizeof(zeroes)));
}

TYPED_TEST(SimdUtilsTest, ones) {
    const TypeParam ones = simd_ones();

    // Should have all bits on.
    char cmp[sizeof(ones)];
    memset(cmp, 0xff, sizeof(ones));
    ASSERT_EQ(0, memcmp(cmp, &ones, sizeof(ones)));
}

TYPED_TEST(SimdUtilsTest, and1) {
    const TypeParam zeroes = simd_zeroes();
    const TypeParam ones = simd_ones();

    TypeParam result;

    result = simd_and(zeroes, ones);
    EXPECT_FALSE(simd_diff(result, zeroes));

    result = simd_and(ones, zeroes);
    EXPECT_FALSE(simd_diff(result, zeroes));

    result = simd_and(zeroes, zeroes);
    EXPECT_FALSE(simd_diff(result, zeroes));

    result = simd_and(ones, ones);
    EXPECT_FALSE(simd_diff(result, ones));
}

TYPED_TEST(SimdUtilsTest, and2) {
    TypeParam a, b;
    memset(&a, 0x33, sizeof(a));
    memset(&b, 0x55, sizeof(b));

    union {
        TypeParam simd;
        char bytes[sizeof(TypeParam)];
    } c;
    c.simd = simd_and(a, b);

    const char expected = 0x33 & 0x55;
    for (size_t i = 0; i < sizeof(c); i++) {
        EXPECT_EQ(expected, c.bytes[i]);
    }
}

TEST(SimdUtils, diff256) {
    const unsigned total_bits = 256;

    // Test identical cases
    ASSERT_EQ(0U, diff256(zeroes256(), zeroes256()));
    ASSERT_EQ(0U, diff256(ones256(), ones256()));
    for (unsigned i = 0; i < total_bits; i++) {
        m256 a = setbit<m256>(i);
        m256 b = setbit<m256>(i);
        ASSERT_EQ(0U, diff256(a, b));
    }

    // Cases that differ in one 32-bit word
    for (unsigned i = 0; i < total_bits; i++) {
        m256 a = setbit<m256>(i);
        u32 rv = diff256(zeroes256(), a);
        ASSERT_EQ(1U, rv);
    }
}

TYPED_TEST(SimdUtilsTest, or1) {
    const TypeParam zeroes = simd_zeroes();
    const TypeParam ones = simd_ones();

    TypeParam result;

    result = simd_or(zeroes, ones);
    EXPECT_FALSE(simd_diff(result, ones));

    result = simd_or(ones, zeroes);
    EXPECT_FALSE(simd_diff(result, ones));

    result = simd_or(zeroes, zeroes);
    EXPECT_FALSE(simd_diff(result, zeroes));

    result = simd_or(ones, ones);
    EXPECT_FALSE(simd_diff(result, ones));
}

TYPED_TEST(SimdUtilsTest, or2) {
    TypeParam a, b;
    memset(&a, 0x33, sizeof(a));
    memset(&b, 0x55, sizeof(b));

    for (unsigned j = 0; j < 8; j++) {
        for (unsigned i = 0; i < 32; i++) {
            m256 x = setbit<m256>(j*32+i);
            m256 y = zeroes256();
            ASSERT_EQ(1U << j, diffrich256(x, y)) << "bit " << j*32+i << " not happy";
        }
    }

    union {
        TypeParam simd;
        char bytes[sizeof(TypeParam)];
    } c;
    c.simd = simd_or(a, b);

    const char expected = 0x33 | 0x55;
    for (size_t i = 0; i < sizeof(c); i++) {
        EXPECT_EQ(expected, c.bytes[i]);
    }
}

TYPED_TEST(SimdUtilsTest, xor1) {
    const TypeParam zeroes = simd_zeroes();
    const TypeParam ones = simd_ones();

    TypeParam result;

    result = simd_xor(zeroes, ones);
    EXPECT_FALSE(simd_diff(result, ones));

    result = simd_xor(ones, zeroes);
    EXPECT_FALSE(simd_diff(result, ones));

    result = simd_xor(zeroes, zeroes);
    EXPECT_FALSE(simd_diff(result, zeroes));

    result = simd_xor(ones, ones);
    EXPECT_FALSE(simd_diff(result, zeroes));
}

TYPED_TEST(SimdUtilsTest, xor2) {
    TypeParam a, b;
    memset(&a, 0x33, sizeof(a));
    memset(&b, 0x55, sizeof(b));

    union {
        TypeParam simd;
        char bytes[sizeof(TypeParam)];
    } c;
    c.simd = simd_xor(a, b);

    const char expected = 0x33 ^ 0x55;
    for (size_t i = 0; i < sizeof(c); i++) {
        EXPECT_EQ(expected, c.bytes[i]);
    }
}

TYPED_TEST(SimdUtilsTest, andnot1) {
    const TypeParam zeroes = simd_zeroes();
    const TypeParam ones = simd_ones();

    TypeParam result;

    result = simd_andnot(zeroes, ones);
    EXPECT_FALSE(simd_diff(result, ones));

    result = simd_andnot(ones, zeroes);
    EXPECT_FALSE(simd_diff(result, zeroes));

    result = simd_andnot(zeroes, zeroes);
    EXPECT_FALSE(simd_diff(result, zeroes));

    result = simd_andnot(ones, ones);
    EXPECT_FALSE(simd_diff(result, zeroes));
}

TYPED_TEST(SimdUtilsTest, andnot2) {
    TypeParam a, b;
    memset(&a, 0x33, sizeof(a));
    memset(&b, 0x55, sizeof(b));

    union {
        TypeParam simd;
        char bytes[sizeof(TypeParam)];
    } c;
    c.simd = simd_andnot(a, b);

    const char expected = ~0x33 & 0x55;
    for (size_t i = 0; i < sizeof(c); i++) {
        EXPECT_EQ(expected, c.bytes[i]);
    }
}

TYPED_TEST(SimdUtilsTest, not1) {
    const TypeParam zeroes = simd_zeroes();
    const TypeParam ones = simd_ones();

    TypeParam result;

    result = simd_not(zeroes);
    EXPECT_FALSE(simd_diff(result, ones));

    result = simd_not(ones);
    EXPECT_FALSE(simd_diff(result, zeroes));
}

TYPED_TEST(SimdUtilsTest, not2) {
    TypeParam a;
    memset(&a, 0x33, sizeof(a));

    union {
        TypeParam simd;
        char bytes[sizeof(TypeParam)];
    } c;
    c.simd = simd_not(a);

    const char expected = ~0x33;
    for (size_t i = 0; i < sizeof(c); i++) {
        EXPECT_EQ(expected, c.bytes[i]);
    }
}

TYPED_TEST(SimdUtilsTest, isnonzero) {
    TypeParam a = simd_zeroes();
    EXPECT_FALSE(simd_isnonzero(a));

    a = simd_ones();
    EXPECT_TRUE(simd_isnonzero(a));

    union {
        TypeParam simd;
        char bytes[sizeof(TypeParam)];
    } c;

    // Try every 1-bit case.
    for (size_t i = 0; i < sizeof(a); i++) {
        for (size_t j = 0; j < 8; j++) {
            memset(&c.simd, 0, sizeof(c.simd));
            c.bytes[i] = 1 << j;
            EXPECT_TRUE(simd_isnonzero(c.simd));
        }
    }
}

TYPED_TEST(SimdUtilsTest, clearbit) {
    const unsigned int total_bits = sizeof(TypeParam) * 8;

    const TypeParam ones = simd_ones();

    for (unsigned int i = 0; i < total_bits; i++) {
        TypeParam a = simd_ones();
        simd_clearbit(&a, i);
        ASSERT_NE(0, simd_diff(a, ones)) << "bit " << i << " wasn't cleared";

        TypeParam mask = setbit<TypeParam>(i);
        ASSERT_EQ(0, simd_diff(ones, simd_or(a, mask)))
            << "clearing bit " << i << " caused collateral damage";
    }
}

TYPED_TEST(SimdUtilsTest, testbit) {
    const unsigned int total_bits = sizeof(TypeParam) * 8;

    const TypeParam ones = simd_ones();

    // First, all bits are on in 'ones'.
    for (unsigned int i = 0; i < total_bits; i++) {
        ASSERT_EQ(1, simd_testbit(ones, i)) << "bit " << i << " is on";
    }

    // Try individual bits; only 'i' should be on.
    for (unsigned int i = 0; i < total_bits; i++) {
        TypeParam a = setbit<TypeParam>(i);
        for (unsigned int j = 0; j < total_bits; j++) {
            ASSERT_EQ(i == j ? 1 : 0, simd_testbit(a, j)) << "bit " << i
                                                          << " is wrong";
        }
    }
}

TYPED_TEST(SimdUtilsTest, setbit) {
    const unsigned int total_bits = sizeof(TypeParam) * 8;

    // Try individual bits; only 'i' should be on.
    for (unsigned int i = 0; i < total_bits; i++) {
        TypeParam a = setbit<TypeParam>(i);
        TypeParam x = simd_zeroes();
        simd_setbit(&x, i);
        ASSERT_FALSE(simd_diff(a, x));
    }

    TypeParam a = simd_zeroes();

    // turn on all bits
    for (unsigned int i = 0; i < total_bits; i++) {
        simd_setbit(&a, i);
    }
    ASSERT_FALSE(simd_diff(simd_ones(), a));

}

TYPED_TEST(SimdUtilsTest, diffrich) {
    const unsigned total_bits = sizeof(TypeParam) * 8;

    const TypeParam zeroes = simd_zeroes();
    const TypeParam ones = simd_ones();

    // Test identical cases
    EXPECT_EQ(0U, simd_diffrich(zeroes, zeroes));
    EXPECT_EQ(0U, simd_diffrich(ones, ones));
    for (unsigned i = 0; i < total_bits; i++) {
        TypeParam a = setbit<TypeParam>(i);
        TypeParam b = setbit<TypeParam>(i);
        EXPECT_EQ(0U, simd_diffrich(a, b));
    }

    // and nothing is on in zeroes
    for (unsigned int i = 0; i < total_bits; i++) {
        ASSERT_EQ(0, simd_testbit(zeroes, i)) << "bit " << i << " is off";
    }

    // All-zeroes and all-ones differ in all words
    EXPECT_EQ((1U << (total_bits / 32)) - 1, simd_diffrich(zeroes, ones));

    // Cases that differ in one 32-bit word
    for (unsigned i = 0; i < total_bits; i++) {
        TypeParam a = setbit<TypeParam>(i);
        u32 rv = simd_diffrich(zeroes, a);
        EXPECT_EQ(1U << i / 32, rv);
    }
}

TYPED_TEST(SimdUtilsTest, diffrich64) {
    const unsigned total_bits = sizeof(TypeParam) * 8;

    const TypeParam zeroes = simd_zeroes();
    const TypeParam ones = simd_ones();

    // Test identical cases
    EXPECT_EQ(0U, simd_diffrich64(zeroes, zeroes));
    EXPECT_EQ(0U, simd_diffrich64(ones, ones));
    for (unsigned i = 0; i < total_bits; i++) {
        TypeParam a = setbit<TypeParam>(i);
        TypeParam b = setbit<TypeParam>(i);
        EXPECT_EQ(0U, simd_diffrich64(a, b));
    }

    // All-zeroes and all-ones differ in all words, which will result in every
    // second bit being on.
    EXPECT_EQ(((1U << (total_bits / 32)) - 1) & 0x55555555u,
              simd_diffrich64(zeroes, ones));

    // Cases that differ in one 64-bit word
    for (unsigned i = 0; i < total_bits; i++) {
        TypeParam a = setbit<TypeParam>(i);
        u32 rv = simd_diffrich64(zeroes, a);
        EXPECT_EQ(1U << ((i / 64) * 2), rv);
    }
}

// Unaligned load
TYPED_TEST(SimdUtilsTest, loadu) {
    const TypeParam ones = simd_ones();

    const size_t mem_len = sizeof(ones) * 2;
    unique_ptr<char[]> mem_array = ue2::make_unique<char[]>(mem_len);
    char *mem = mem_array.get();

    for (size_t offset = 1; offset < sizeof(ones); offset++) {
        memset(mem, 0, mem_len);
        memset(mem + offset, 0xff, sizeof(ones));
        TypeParam a;
        simd_loadu(&a, mem + offset);
        ASSERT_EQ(0, simd_diff(a, ones));
    }
}

// Aligned load and store
TYPED_TEST(SimdUtilsTest, load_store) {
    union {
        TypeParam simd;
        char bytes[sizeof(TypeParam)];
    } a;
    for (size_t i = 0; i < sizeof(a); i++) {
        a.bytes[i] = (char)(i % 256);
    }

    auto mem_ptr = make_bytecode_ptr<char>(sizeof(a), alignof(TypeParam));
    char *mem = mem_ptr.get();

    ASSERT_EQ(0, (size_t)mem % 16U);

    memset(mem, 0, sizeof(a));

    simd_store(mem, a.simd);
    ASSERT_EQ(0, memcmp(mem, a.bytes, sizeof(a)));

    TypeParam b;
    simd_load(&b, mem);
    ASSERT_FALSE(simd_diff(a.simd, b));
}

// Packed load and store
TYPED_TEST(SimdUtilsTest, loadbytes_storebytes) {
    union {
        TypeParam simd;
        char bytes[sizeof(TypeParam)];
    } a;
    for (size_t i = 0; i < sizeof(a); i++) {
        a.bytes[i] = (char)(i % 256);
    }

    char mem[sizeof(TypeParam)];
    for (size_t i = 1; i < sizeof(TypeParam); i++) {
        memset(mem, 0xff, sizeof(TypeParam));

        simd_storebytes(mem, a.simd, i);

        union {
            TypeParam simd;
            char bytes[sizeof(TypeParam)];
        } b;
        simd_loadbytes(&b.simd, mem, i);

        // First i bytes should match a, remaining bytes are zero. (Note that
        // this takes endianness into account)
        for (size_t j = 0; j < sizeof(TypeParam); j++) {
            size_t idx = j;
            ASSERT_EQ(j < i ? a.bytes[idx] : 0, b.bytes[idx]);
        }
    }
}

TYPED_TEST(SimdUtilsTest, lshift64) {
    TypeParam a;
    memset(&a, 0x5a, sizeof(a));

    static constexpr u64a exp_val = 0x5a5a5a5a5a5a5a5aULL;

    union {
        TypeParam simd;
        u64a qword[sizeof(TypeParam) / 8];
    } c;

    for (unsigned s = 0; s < 64; s++) {
        c.simd = simd_lshift64(a, s);

        const u64a expected = exp_val << s;
        for (size_t i = 0; i < sizeof(c) / 8; i++) {
            EXPECT_EQ(expected, c.qword[i]);
        }
    }

    /* Clang 3.4 on FreeBSD 10 crashes on the following - disable for now */
#if !(defined(__FreeBSD__) && defined(__clang__) && __clang_major__ == 3)

    // test immediates
    u64a expected;

    c.simd = simd_lshift64(a, 1);
    expected = exp_val << 1;
    for (size_t i = 0; i < sizeof(c) / 8; i++) {
        EXPECT_EQ(expected, c.qword[i]);
    }

    c.simd = simd_lshift64(a, 2);
    expected = exp_val << 2;
    for (size_t i = 0; i < sizeof(c) / 8; i++) {
        EXPECT_EQ(expected, c.qword[i]);
    }

    c.simd = simd_lshift64(a, 7);
    expected = exp_val << 7;
    for (size_t i = 0; i < sizeof(c) / 8; i++) {
        EXPECT_EQ(expected, c.qword[i]);
    }

    c.simd = simd_lshift64(a, 31);
    expected = exp_val << 31;
    for (size_t i = 0; i < sizeof(c) / 8; i++) {
        EXPECT_EQ(expected, c.qword[i]);
    }
#endif
}

TEST(SimdUtilsTest, alignment) {
    ASSERT_EQ(16, alignof(m128));
    ASSERT_EQ(32, alignof(m256));
    ASSERT_EQ(16, alignof(m384));
    ASSERT_EQ(64, alignof(m512));
}

TEST(SimdUtilsTest, movq) {
    m128 simd;

    simd = ones128();
    u64a r = movq(simd);
    ASSERT_EQ((u64a)(~0), r);

    char cmp[sizeof(m128)];
    memset(cmp, 0x80, sizeof(m128));
    simd = set16x8(0x80);
    r = movq(simd);
    ASSERT_EQ(0, memcmp(cmp, &simd, sizeof(simd)));
    ASSERT_EQ(0, memcmp(cmp, &r, sizeof(r)));

    simd = _mm_set_epi64x(~0LL, 0x123456789abcdef);
    r = movq(simd);
    ASSERT_EQ(r, 0x123456789abcdef);
}


TEST(SimdUtilsTest, set16x8) {
    char cmp[sizeof(m128)];

    for (unsigned i = 0; i < 256; i++) {
        m128 simd = set16x8(i);
        memset(cmp, i, sizeof(simd));
        ASSERT_EQ(0, memcmp(cmp, &simd, sizeof(simd)));
    }
}

TEST(SimdUtilsTest, set4x32) {
    u32 cmp[4] = { 0x12345678, 0x12345678, 0x12345678, 0x12345678 };
    m128 simd = set4x32(cmp[0]);
    ASSERT_EQ(0, memcmp(cmp, &simd, sizeof(simd)));
}

#if defined(HAVE_AVX2)
TEST(SimdUtilsTest, set32x8) {
    char cmp[sizeof(m256)];

    for (unsigned i = 0; i < 256; i++) {
        m256 simd = set32x8(i);
        memset(cmp, i, sizeof(simd));
        ASSERT_EQ(0, memcmp(cmp, &simd, sizeof(simd)));
    }
}

TEST(SimdUtilsTest, set2x128) {
    char cmp[sizeof(m256)];

    for (unsigned i = 0; i < 256; i++) {
        m128 x = set16x8(i);
        m256 y = set32x8(i);
        m256 z = set2x128(x);
        memset(cmp, i, sizeof(z));
        ASSERT_EQ(0, memcmp(cmp, &z, sizeof(z)));
        ASSERT_EQ(0, memcmp(&y, &z, sizeof(z)));
    }
}
#endif

TEST(SimdUtilsTest, variableByteShift128) {
    char base[] = "0123456789ABCDEF";
    m128 in = loadu128(base);

    EXPECT_TRUE(!diff128(rshiftbyte_m128(in, 0),
                         variable_byte_shift_m128(in, 0)));
    EXPECT_TRUE(!diff128(rshiftbyte_m128(in, 1),
                         variable_byte_shift_m128(in, -1)));
    EXPECT_TRUE(!diff128(rshiftbyte_m128(in, 2),
                         variable_byte_shift_m128(in, -2)));
    EXPECT_TRUE(!diff128(rshiftbyte_m128(in, 3),
                         variable_byte_shift_m128(in, -3)));
    EXPECT_TRUE(!diff128(rshiftbyte_m128(in, 4),
                         variable_byte_shift_m128(in, -4)));
    EXPECT_TRUE(!diff128(rshiftbyte_m128(in, 5),
                         variable_byte_shift_m128(in, -5)));
    EXPECT_TRUE(!diff128(rshiftbyte_m128(in, 6),
                         variable_byte_shift_m128(in, -6)));
    EXPECT_TRUE(!diff128(rshiftbyte_m128(in, 7),
                         variable_byte_shift_m128(in, -7)));
    EXPECT_TRUE(!diff128(rshiftbyte_m128(in, 8),
                         variable_byte_shift_m128(in, -8)));
    EXPECT_TRUE(!diff128(rshiftbyte_m128(in, 9),
                         variable_byte_shift_m128(in, -9)));
    EXPECT_TRUE(!diff128(rshiftbyte_m128(in, 10),
                         variable_byte_shift_m128(in, -10)));

    EXPECT_TRUE(!diff128(lshiftbyte_m128(in, 0),
                         variable_byte_shift_m128(in, 0)));
    EXPECT_TRUE(!diff128(lshiftbyte_m128(in, 1),
                         variable_byte_shift_m128(in, 1)));
    EXPECT_TRUE(!diff128(lshiftbyte_m128(in, 2),
                         variable_byte_shift_m128(in, 2)));
    EXPECT_TRUE(!diff128(lshiftbyte_m128(in, 3),
                         variable_byte_shift_m128(in, 3)));
    EXPECT_TRUE(!diff128(lshiftbyte_m128(in, 4),
                         variable_byte_shift_m128(in, 4)));
    EXPECT_TRUE(!diff128(lshiftbyte_m128(in, 5),
                         variable_byte_shift_m128(in, 5)));
    EXPECT_TRUE(!diff128(lshiftbyte_m128(in, 6),
                         variable_byte_shift_m128(in, 6)));
    EXPECT_TRUE(!diff128(lshiftbyte_m128(in, 7),
                         variable_byte_shift_m128(in, 7)));
    EXPECT_TRUE(!diff128(lshiftbyte_m128(in, 8),
                         variable_byte_shift_m128(in, 8)));
    EXPECT_TRUE(!diff128(lshiftbyte_m128(in, 9),
                         variable_byte_shift_m128(in, 9)));
    EXPECT_TRUE(!diff128(lshiftbyte_m128(in, 10),
                         variable_byte_shift_m128(in, 10)));

    EXPECT_TRUE(!diff128(zeroes128(), variable_byte_shift_m128(in, 16)));
    EXPECT_TRUE(!diff128(zeroes128(), variable_byte_shift_m128(in, -16)));
}

TEST(SimdUtilsTest, max_u8_m128) {
    char base1[] = "0123456789ABCDE\xfe";
    char base2[] = "!!23455889aBCd\xff\xff";
    char expec[] = "0123456889aBCd\xff\xff";
    m128 in1 = loadu128(base1);
    m128 in2 = loadu128(base2);
    m128 result = max_u8_m128(in1, in2);
    EXPECT_TRUE(!diff128(result, loadu128(expec)));
}

TEST(SimdUtilsTest, min_u8_m128) {
    char base1[] = "0123456789ABCDE\xfe";
    char base2[] = "!!23455889aBCd\xff\xff";
    char expec[] = "!!23455789ABCDE\xfe";
    m128 in1 = loadu128(base1);
    m128 in2 = loadu128(base2);
    m128 result = min_u8_m128(in1, in2);
    EXPECT_TRUE(!diff128(result, loadu128(expec)));
}

TEST(SimdUtilsTest, sadd_u8_m128) {
    unsigned char base1[] = {0, 0x80, 0xff, 'A', '1', '2', '3', '4',
                             '1', '2', '3', '4', '1', '2', '3', '4'};
    unsigned char base2[] = {'a', 0x80, 'b', 'A', 0x10, 0x10, 0x10, 0x10,
                             0x30, 0x30, 0x30, 0x30, 0, 0, 0, 0};
    unsigned char expec[] = {'a', 0xff, 0xff, 0x82, 'A', 'B', 'C', 'D',
                             'a', 'b', 'c', 'd', '1', '2', '3', '4'};
    m128 in1 = loadu128(base1);
    m128 in2 = loadu128(base2);
    m128 result = sadd_u8_m128(in1, in2);
    EXPECT_TRUE(!diff128(result, loadu128(expec)));
}

TEST(SimdUtilsTest, sub_u8_m128) {
    unsigned char base1[] = {'a', 0xff, 0xff, 0x82, 'A', 'B', 'C', 'D',
                             'a', 'b', 'c', 'd', '1', '2', '3', '4'};
    unsigned char base2[] = {0, 0x80, 0xff, 'A', '1', '2', '3', '4',
                             '1', '2', '3', '4', '1', '2', '3', '4'};
    unsigned char expec[] = {'a', 0x7f, 0, 'A', 0x10, 0x10, 0x10, 0x10,
                             0x30, 0x30, 0x30, 0x30, 0, 0, 0, 0};
    m128 in1 = loadu128(base1);
    m128 in2 = loadu128(base2);
    m128 result = sub_u8_m128(in1, in2);
    EXPECT_TRUE(!diff128(result, loadu128(expec)));
}

} // namespace
