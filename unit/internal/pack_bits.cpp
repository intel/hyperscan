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
#include "util/pack_bits.h"
#include "util/make_unique.h"
#include "ue2common.h"

#include <algorithm>
#include <memory>
#include <vector>

using std::unique_ptr;
using std::vector;

template <typename T>
class PackBits : public testing::Test {};

typedef ::testing::Types<u32, u64a> PackBitsTypes;
TYPED_TEST_CASE(PackBits, PackBitsTypes);

// Templated wrappers around the pack_bits_{32,64} functions.
template <typename T>
void pack_bits(char *out, const T *v, const u32 *bits, unsigned elements);
template <typename T>
void unpack_bits(T *v, const char *in, const u32 *bits, unsigned elements);

template <>
void pack_bits<u32>(char *out, const u32 *v, const u32 *bits,
                    unsigned elements) {
    return pack_bits_32(out, v, bits, elements);
}

template <>
void pack_bits<u64a>(char *out, const u64a *v, const u32 *bits,
                     unsigned elements) {
    return pack_bits_64(out, v, bits, elements);
}

template <>
void unpack_bits<u32>(u32 *v, const char *in, const u32 *bits,
                      unsigned elements) {
    return unpack_bits_32(v, (const u8 *)in, bits, elements);
}

template <>
void unpack_bits<u64a>(u64a *v, const char *in, const u32 *bits,
                       unsigned elements) {
    return unpack_bits_64(v, (const u8 *)in, bits, elements);
}

template <typename T>
size_t packed_size(const vector<T> &bits) {
    size_t num_bits = 0;
    for (size_t i = 0; i < bits.size(); i++) {
        num_bits += bits[i];
    }
    return ROUNDUP_N(num_bits, 8U) / 8U;
}

template <typename T>
void test_pack_and_unpack(const vector<T> &v, const vector<u32> &bits) {
    const u32 max_bits = sizeof(T) * 8U;
    const u32 elements = bits.size();

    // Temporary char array to pack into.
    const size_t mem_size = packed_size(bits);
    unique_ptr<char[]> mem = ue2::make_unique<char[]>(mem_size);

    pack_bits<T>(&mem[0], &v[0], &bits[0], elements);

    vector<T> v2(elements, 0xcafecafeu); // Output vector.

    unpack_bits<T>(&v2[0], &mem[0], &bits[0], elements);

    for (u32 i = 0; i < elements; i++) {
        // Check that v2[i] contains the bits[i] low bits of v[i].
        T mask = bits[i] == max_bits ? ~(T)0 : ((T)1U << bits[i]) - 1;
        ASSERT_EQ(v[i] & mask, v2[i]) << "element " << i << " mismatch";
    }
}

TYPED_TEST(PackBits, AllZeroes) {
    for (u32 i = 1; i < 32; i++) {
        SCOPED_TRACE(i);
        vector<TypeParam> v(i, 0);
        vector<u32> bits(i, sizeof(TypeParam) * 8U); // all bits
        test_pack_and_unpack(v, bits);
    }
}

TYPED_TEST(PackBits, AllOnes) {
    for (u32 i = 1; i < 32; i++) {
        SCOPED_TRACE(i);
        vector<TypeParam> v(i, ~((TypeParam)0));

        for (u32 num_bits = 0; num_bits < sizeof(TypeParam) * 8U; num_bits++) {
            vector<u32> bits(i, num_bits);
            test_pack_and_unpack(v, bits);
        }
    }
}

TYPED_TEST(PackBits, SomeOnes) {
    for (u32 i = 1; i < 32; i++) {
        SCOPED_TRACE(i);
        vector<TypeParam> v(i, ~((TypeParam)0));

        // One bit from v[0], two bits from v[1] ...
        vector<u32> bits(i);
        for (u32 j = 0; j < i; j++) {
        }

        test_pack_and_unpack(v, bits);

        // And in reverse
        std::reverse(bits.begin(), bits.end());
        test_pack_and_unpack(v, bits);
    }
}

TYPED_TEST(PackBits, Holes) {
    for (u32 i = 1; i < 32; i++) {
        SCOPED_TRACE(i);
        vector<TypeParam> v(i, ~((TypeParam)0));

        // No bits from odd-numbered words.
        vector<u32> bits(i);
        for (u32 j = 0; j < i; j++) {
            bits[j] = j % 2 ? 0 : sizeof(TypeParam) * 8U;
        }

        test_pack_and_unpack(v, bits);
    }
}
