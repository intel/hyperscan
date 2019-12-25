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

#include "gtest/gtest.h"

#include "config.h"
#include "ue2common.h"
#include "util/uniform_ops.h"
#include "util/alloc.h"

using namespace ue2;

TEST(Uniform, Sizes) {
    // We likely depend on the size of the structures in various different
    // places in UE2, and it's conceivable that they could vary by
    // platform/compiler.
    EXPECT_EQ(1U, sizeof(u8));
    EXPECT_EQ(2U, sizeof(u16));
    EXPECT_EQ(4U, sizeof(u32));
    EXPECT_EQ(8U, sizeof(u64a));
    EXPECT_EQ(16U, sizeof(m128));
    EXPECT_EQ(32U, sizeof(m256));
    EXPECT_EQ(64U, sizeof(m512));
}

TEST(Uniform, loadstore_u8) {
    for (int i = 0; i < 8; i++) {
        u8 in = 1 << i;
        const char *cin = (const char *)(&in);
        u8 out = load_u8(cin);
        EXPECT_EQ(in, out);
        char ALIGN_DIRECTIVE stored[1];
        store_u8(stored, in);
        EXPECT_EQ(0, memcmp(stored, &in, sizeof(in)));
    }
}

TEST(Uniform, loadstore_u16) {
    for (int i = 0; i < 16; i++) {
        u16 in = 1 << i;
        const char *cin = (const char *)(&in);
        u16 out = load_u16(cin);
        EXPECT_EQ(in, out);
        void *stored = aligned_zmalloc(2);
        store_u16(stored, in);
        EXPECT_EQ(0, memcmp(stored, &in, sizeof(in)));
        aligned_free(stored);
    }
}

TEST(Uniform, loadstore_u32) {
    for (int i = 0; i < 32; i++) {
        u32 in = 1U << i;
        const char *cin = (const char *)(&in);
        u32 out = load_u32(cin);
        EXPECT_EQ(in, out);
        void *stored = aligned_zmalloc(32/8);
        store_u32(stored, in);
        EXPECT_EQ(0, memcmp(stored, &in, sizeof(in)));
        aligned_free(stored);
    }
}

TEST(Uniform, loadstore_u64a) {
    for (int i = 0; i < 64; i++) {
        u64a in = 1ULL << i;
        const char *cin = (const char *)(&in);
        u64a out = load_u64a(cin);
        EXPECT_EQ(in, out);
        void *stored = aligned_zmalloc(64/8);
        store_u64a(stored, in);
        EXPECT_EQ(0, memcmp(stored, &in, sizeof(in)));
        aligned_free(stored);
    }
}

TEST(Uniform, loadstore_m128) {
    union {
        m128 simd;
        u32 words[128/32];
    } in;
    for (int i = 0; i < 128; i++) {
        memset(&in, 0, sizeof(in));
        in.words[i/32] = 1U << (i % 32);
        const char *cin = (const char *)(&in);
        m128 out = load_m128(cin);
        EXPECT_EQ(0, memcmp(&out, &in, sizeof(out)));
        void *stored = aligned_zmalloc(128/8);
        store_m128(stored, in.simd);
        EXPECT_EQ(0, memcmp(stored, &in, sizeof(in)));
        aligned_free(stored);
    }
}

TEST(Uniform, loadstore_m256) {
    union {
        m256 simd;
        u32 words[256/32];
    } in;
    for (int i = 0; i < 256; i++) {
        memset(&in, 0, sizeof(in));
        in.words[i/32] = 1U << (i % 32);
        const char *cin = (const char *)(&in);
        m256 out = load_m256(cin);
        EXPECT_EQ(0, memcmp(&out, &in, sizeof(out)));
        void *stored = aligned_zmalloc(256/8);
        store_m256(stored, in.simd);
        EXPECT_EQ(0, memcmp(stored, &in, sizeof(in)));
        aligned_free(stored);
    }
}

TEST(Uniform, loadstore_m512) {
    union {
        m512 simd;
        u32 words[512/32];
    } in;
    for (int i = 0; i < 512; i++) {
        memset(&in, 0, sizeof(in));
        in.words[i/32] = 1U << (i % 32);
        const char *cin = (const char *)(&in);
        m512 out = load_m512(cin);
        EXPECT_EQ(0, memcmp(&out, &in, sizeof(out)));
        void *stored = aligned_zmalloc(512/8);
        store_m512(stored, in.simd);
        EXPECT_EQ(0, memcmp(stored, &in, sizeof(in)));
        aligned_free(stored);
    }
}

TEST(Uniform, testbit_u32) {
    for (u32 i = 0; i < 32; i++) {
        u32 v = 0;
        EXPECT_EQ((char)0, testbit_u32(v, i));
        v |= 1ULL << i;
        EXPECT_EQ((char)1, testbit_u32(v, i));
        v = ~v;
        EXPECT_EQ((char)0, testbit_u32(v, i));
        v |= 1ULL << i;
        EXPECT_EQ((char)1, testbit_u32(v, i));
    }
}

TEST(Uniform, testbit_u64a) {
    for (u32 i = 0; i < 64; i++) {
        u64a v = 0;
        EXPECT_EQ((char)0, testbit_u64a(v, i));
        v |= 1ULL << i;
        EXPECT_EQ((char)1, testbit_u64a(v, i));
        v = ~v;
        EXPECT_EQ((char)0, testbit_u64a(v, i));
        v |= 1ULL << i;
        EXPECT_EQ((char)1, testbit_u64a(v, i));
    }
}

TEST(Uniform, clearbit_u32) {
    for (u32 i = 0; i < 32; i++) {
        u32 v = ~0U;
        clearbit_u32(&v, i);
        EXPECT_EQ((char)0, testbit_u32(v, i));
        v = ~v;
        clearbit_u32(&v, i);
        EXPECT_EQ(0U, v);
    }
}

TEST(Uniform, clearbit_u64a) {
    for (u32 i = 0; i < 64; i++) {
        u64a v = ~0ULL;
        clearbit_u64a(&v, i);
        EXPECT_EQ((char)0, testbit_u64a(v, i));
        v = ~v;
        clearbit_u64a(&v, i);
        EXPECT_EQ(0ULL, v);
    }
}
