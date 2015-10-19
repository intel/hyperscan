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

#include "util/partial_store.h"
#include "gtest/gtest.h"
#include "ue2common.h"

#include <cstring>

using namespace std;

TEST(partial, u32_1) {
    u8 a[16];
    memset(a, 0x1f, sizeof(a));

    u8 b[16];
    memcpy(b, a, sizeof(a));

    partial_store_u32(a + 8, 0xaa, 1);
    b[8] = 0xaa;

    ASSERT_EQ(0xaa, partial_load_u32(a + 8, 1));
    ASSERT_EQ(0, memcmp(a, b, sizeof(a)));
}

TEST(partial, u32_2) {
    u8 a[16];
    memset(a, 0x1f, sizeof(a));

    u8 b[16];
    memcpy(b, a, sizeof(a));

    partial_store_u32(a + 8, 0xaabb, 2);
    b[8] = 0xbb;
    b[9] = 0xaa;

    ASSERT_EQ(0xaabb, partial_load_u32(a + 8, 2));
    ASSERT_EQ(0, memcmp(a, b, sizeof(a)));
}

TEST(partial, u32_3) {
    u8 a[16];
    memset(a, 0x1f, sizeof(a));

    u8 b[16];
    memcpy(b, a, sizeof(a));

    partial_store_u32(a + 8, 0xaabbcc, 3);
    b[8]  = 0xcc;
    b[9]  = 0xbb;
    b[10] = 0xaa;

    ASSERT_EQ(0xaabbcc, partial_load_u32(a + 8, 3));
    ASSERT_EQ(0, memcmp(a, b, sizeof(a)));
}

TEST(partial, u32_4) {
    u8 a[16];
    memset(a, 0x1f, sizeof(a));

    u8 b[16];
    memcpy(b, a, sizeof(a));

    partial_store_u32(a + 8, 0xaabbccdd, 4);
    b[8]  = 0xdd;
    b[9]  = 0xcc;
    b[10] = 0xbb;
    b[11] = 0xaa;

    ASSERT_EQ(0xaabbccdd, partial_load_u32(a + 8, 4));
    ASSERT_EQ(0, memcmp(a, b, sizeof(a)));
}

TEST(partial, u64a_1) {
    u64a val = 0xaa;
    u8 a[16];
    memset(a, 0x1f, sizeof(a));

    u8 b[16];
    memcpy(b, a, sizeof(a));

    partial_store_u64a(a + 8, val, 1);
    b[8] = 0xaa;

    ASSERT_EQ(val, partial_load_u64a(a + 8, 1));
    ASSERT_EQ(0, memcmp(a, b, sizeof(a)));
}

TEST(partial, u64a_2) {
    u64a val = 0xaabb;
    u8 a[16];
    memset(a, 0x1f, sizeof(a));

    u8 b[16];
    memcpy(b, a, sizeof(a));

    partial_store_u64a(a + 8, val, 2);
    b[8] = 0xbb;
    b[9] = 0xaa;

    ASSERT_EQ(val, partial_load_u64a(a + 8, 2));
    ASSERT_EQ(0, memcmp(a, b, sizeof(a)));
}

TEST(partial, u64a_3) {
    u64a val = 0xaabbcc;
    u8 a[16];
    memset(a, 0x1f, sizeof(a));

    u8 b[16];
    memcpy(b, a, sizeof(a));

    partial_store_u64a(a + 8, val, 3);
    b[8]  = 0xcc;
    b[9]  = 0xbb;
    b[10] = 0xaa;

    ASSERT_EQ(val, partial_load_u64a(a + 8, 3));
    ASSERT_EQ(0, memcmp(a, b, sizeof(a)));
}

TEST(partial, u64a_4) {
    u64a val = 0xaabbccdd;
    u8 a[16];
    memset(a, 0x1f, sizeof(a));

    u8 b[16];
    memcpy(b, a, sizeof(a));

    partial_store_u64a(a + 8, val, 4);
    b[8]  = 0xdd;
    b[9]  = 0xcc;
    b[10] = 0xbb;
    b[11] = 0xaa;

    ASSERT_EQ(val, partial_load_u64a(a + 8, 4));
    ASSERT_EQ(0, memcmp(a, b, sizeof(a)));
}

TEST(partial, u64a_5) {
    u64a val = 0xaabbccdd55ULL;
    u8 a[16];
    memset(a, 0x1f, sizeof(a));

    u8 b[16];
    memcpy(b, a, sizeof(a));

    partial_store_u64a(a + 8, val, 5);
    b[8]  = 0x55;
    b[9]  = 0xdd;
    b[10] = 0xcc;
    b[11] = 0xbb;
    b[12] = 0xaa;

    ASSERT_EQ(val, partial_load_u64a(a + 8, 5));
    ASSERT_EQ(0, memcmp(a, b, sizeof(a)));
}

TEST(partial, u64a_6) {
    u64a val = 0xaabbccdd5566ULL;
    u8 a[16];
    memset(a, 0x1f, sizeof(a));

    u8 b[16];
    memcpy(b, a, sizeof(a));

    partial_store_u64a(a + 8, val, 6);
    b[8]  = 0x66;
    b[9]  = 0x55;
    b[10] = 0xdd;
    b[11] = 0xcc;
    b[12] = 0xbb;
    b[13] = 0xaa;

    ASSERT_EQ(val, partial_load_u64a(a + 8, 6));
    ASSERT_EQ(0, memcmp(a, b, sizeof(a)));
}

TEST(partial, u64a_7) {
    u64a val = 0xaabbccdd556677ULL;
    u8 a[16];
    memset(a, 0x1f, sizeof(a));

    u8 b[16];
    memcpy(b, a, sizeof(a));

    partial_store_u64a(a + 8, val, 7);
    b[8]  = 0x77;
    b[9]  = 0x66;
    b[10] = 0x55;
    b[11] = 0xdd;
    b[12] = 0xcc;
    b[13] = 0xbb;
    b[14] = 0xaa;

    ASSERT_EQ(val, partial_load_u64a(a + 8, 7));
    ASSERT_EQ(0, memcmp(a, b, sizeof(a)));
}

TEST(partial, u64a_8) {
    u64a val = 0xaabbccdd55667788ULL;
    u8 a[16];
    memset(a, 0x1f, sizeof(a));

    u8 b[16];
    memcpy(b, a, sizeof(a));

    partial_store_u64a(a + 8, val, 8);
    b[8]  = 0x88;
    b[9]  = 0x77;
    b[10] = 0x66;
    b[11] = 0x55;
    b[12] = 0xdd;
    b[13] = 0xcc;
    b[14] = 0xbb;
    b[15] = 0xaa;

    ASSERT_EQ(val, partial_load_u64a(a + 8, 8));
    ASSERT_EQ(0, memcmp(a, b, sizeof(a)));
}

