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
#include "util/state_compress.h"

#include <vector>
#include <tuple>

using namespace std;

TEST(state_compress, u32) {
    char buf[sizeof(u32)] = { 0 };
    vector<tuple<u32, u32, int> > tests = {
        make_tuple(0U, 0x1U, 1),
        make_tuple(1U, 0x1U, 1),
        make_tuple(0x80008000, 0x80808080, 1),
        make_tuple(0x9f008000, 0x80808080, 1),
        make_tuple(0x9fab8dfe, 0xff0fff71, 3),
        make_tuple(0x9fab8dfe, 0xffefff71, 4),
        make_tuple(0xf0f0f0f0, 0x0f0f0f0f, 4),
    };

    for (const auto &e : tests) {
        u32 val;
        u32 mask;
        int len;
        tie(val, mask, len) = e;

        storecompressed32(&buf, &val, &mask, len);

        u32 val_out;
        loadcompressed32(&val_out, &buf, &mask, len);

        EXPECT_EQ(val & mask, val_out);
    }
}

TEST(state_compress, u64a) {
    char buf[sizeof(u64a)] = { 0 };
    vector<tuple<u64a, u64a, int> > tests = {
        make_tuple(0ULL, 0x1ULL, 1),
        make_tuple(1ULL, 0x1ULL, 1),
        make_tuple(0x80008000ULL, 0x80808080ULL, 1),
        make_tuple(0x9f008000ULL, 0x80808080ULL, 1),
        make_tuple(0x9fab8dfeULL, 0xff0fff71ULL, 3),
        make_tuple(0x9fab8dfeULL, 0xffefff71ULL, 4),
        make_tuple(0xf0f0f0f0ULL, 0x0f0f0f0fULL, 4),
        make_tuple(0x0123456789abcdefULL, 0x0123456789abcdefULL, 5),
        make_tuple(0x0123456789abcdefULL, 0xfedcba9876543210ULL, 6),
        make_tuple(0x0123456789abcdefULL, 0x0123456789abcdefULL, 7),
        make_tuple(0x0123456789abcdefULL, 0xffffffffffffffffULL, 8),
    };

    for (const auto &e : tests) {
        u64a val;
        u64a mask;
        int len;
        tie(val, mask, len) = e;

        storecompressed64(&buf, &val, &mask, len);

        u64a val_out;
        loadcompressed64(&val_out, &buf, &mask, len);

        EXPECT_EQ(val & mask, val_out);
    }
}

TEST(state_compress, m128_1) {
    char buf[sizeof(m128)] = { 0 };

    for (u32 i = 0; i < 16; i++) {
        char mask_raw[16] = { 0 };
        char val_raw[16] = { 0 };

        memset(val_raw, (i << 4) + 3, 16);

        mask_raw[i] = 0xff;
        val_raw[i] = i;

        mask_raw[15 - i] = 0xff;
        val_raw[15 - i] = i;

        m128 val;
        m128 mask;

        memcpy(&val, val_raw, sizeof(val));
        memcpy(&mask, mask_raw, sizeof(mask));

        storecompressed128(&buf, &val, &mask, 0);

        m128 val_out;
        loadcompressed128(&val_out, &buf, &mask, 0);

        EXPECT_TRUE(!diff128(and128(val, mask), val_out));

        mask_raw[i] = 0x0f;
        mask_raw[15 - i] = 0x0f;
        memcpy(&mask, mask_raw, sizeof(mask));
        val_raw[i] = 9;

        storecompressed128(&buf, &val, &mask, 0);
        loadcompressed128(&val_out, &buf, &mask, 0);

        EXPECT_TRUE(!diff128(and128(val, mask), val_out));
    }
}

TEST(state_compress, m128_2) {
    char buf[sizeof(m128)] = { 0 };

    char val_raw[16] = { '0', '1', '2', '3', '4', '5', '6', '7',
                         '8', '9', 'a', 'b', 'c', 'd', 'e', 'f' };
    m128 val;
    memcpy(&val, val_raw, sizeof(val));

    for (u32 i = 0; i < 16; i++) {
        char mask_raw[16];
        memset(mask_raw, 0x7f, sizeof(mask_raw));
        mask_raw[i] = 0;

        m128 mask;
        memcpy(&mask, mask_raw, sizeof(mask));

        storecompressed128(&buf, &val, &mask, 0);

        m128 val_out;
        loadcompressed128(&val_out, &buf, &mask, 0);

        EXPECT_TRUE(!diff128(and128(val, mask), val_out));

        for (u32 j = i + 1; j < 16; j++) {
            mask_raw[j] = 0;
            memcpy(&mask, mask_raw, sizeof(mask));

            storecompressed128(&buf, &val, &mask, 0);
            loadcompressed128(&val_out, &buf, &mask, 0);
            EXPECT_TRUE(!diff128(and128(val, mask), val_out));

            mask_raw[j] = 0x7f;
        }
    }
}

TEST(state_compress, m256_1) {
    char buf[sizeof(m256)] = { 0 };

    for (u32 i = 0; i < 32; i++) {
        char mask_raw[32] = { 0 };
        char val_raw[32] = { 0 };

        memset(val_raw, (i << 3) + 3, 32);

        mask_raw[i] = 0xff;
        val_raw[i] = i;

        mask_raw[31 - i] = 0xff;
        val_raw[31 - i] = i;

        m256 val;
        m256 mask;

        memcpy(&val, val_raw, sizeof(val));
        memcpy(&mask, mask_raw, sizeof(mask));

        storecompressed256(&buf, &val, &mask, 0);

        m256 val_out;
        loadcompressed256(&val_out, &buf, &mask, 0);

        EXPECT_TRUE(!diff256(and256(val, mask), val_out));

        mask_raw[i] = 0x7;
        mask_raw[31 - i] = 0x1f;
        memcpy(&mask, mask_raw, sizeof(mask));
        val_raw[i] = 5;

        storecompressed256(&buf, &val, &mask, 0);
        loadcompressed256(&val_out, &buf, &mask, 0);

        EXPECT_TRUE(!diff256(and256(val, mask), val_out));
    }
}

TEST(state_compress, m256_2) {
    char buf[sizeof(m256)] = { 0 };

    char val_raw[32] = { '0', '1', '2', '3', '4', '5', '6', '7',
                         '8', '9', 'a', 'b', 'c', 'd', 'e', 'f',
                         'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H',
                         'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P' };
    m256 val;
    memcpy(&val, val_raw, sizeof(val));

    for (u32 i = 0; i < 32; i++) {
        char mask_raw[32];
        memset(mask_raw, 0x7f, sizeof(mask_raw));
        mask_raw[i] = 0;

        m256 mask;
        memcpy(&mask, mask_raw, sizeof(mask));

        storecompressed256(&buf, &val, &mask, 0);

        m256 val_out;
        loadcompressed256(&val_out, &buf, &mask, 0);

        EXPECT_TRUE(!diff256(and256(val, mask), val_out));

        for (u32 j = i + 1; j < 32; j++) {
            mask_raw[j] = 0;
            memcpy(&mask, mask_raw, sizeof(mask));

            storecompressed256(&buf, &val, &mask, 0);
            loadcompressed256(&val_out, &buf, &mask, 0);
            EXPECT_TRUE(!diff256(and256(val, mask), val_out));

            mask_raw[j] = 0x7f;
        }
    }
}

TEST(state_compress, m384_1) {
    char buf[sizeof(m384)] = { 0 };

    for (u32 i = 0; i < 48; i++) {
        char mask_raw[48] = { 0 };
        char val_raw[48] = { 0 };

        memset(val_raw, (i << 2) + 3, 48);

        mask_raw[i] = 0xff;
        val_raw[i] = i;

        mask_raw[47 - i] = 0xff;
        val_raw[47 - i] = i;

        m384 val;
        m384 mask;

        memcpy(&val, val_raw, sizeof(val));
        memcpy(&mask, mask_raw, sizeof(mask));

        storecompressed384(&buf, &val, &mask, 0);

        m384 val_out;
        loadcompressed384(&val_out, &buf, &mask, 0);

        EXPECT_TRUE(!diff384(and384(val, mask), val_out));

        mask_raw[i] = 0x3;
        mask_raw[47 - i] = 0x2f;
        memcpy(&mask, mask_raw, sizeof(mask));
        val_raw[i] = 3;

        storecompressed384(&buf, &val, &mask, 0);
        loadcompressed384(&val_out, &buf, &mask, 0);

        EXPECT_TRUE(!diff384(and384(val, mask), val_out));
    }
}

TEST(state_compress, m384_2) {
    char buf[sizeof(m384)] = { 0 };

    char val_raw[48] = { '0', '1', '2', '3', '4', '5', '6', '7',
                         '8', '9', 'a', 'b', 'c', 'd', 'e', 'f',
                         'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H',
                         'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',
                         'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z',
                         'Q', 'R', ' ', '-', ',', '"', ';', ':' };
    m384 val;
    memcpy(&val, val_raw, sizeof(val));

    for (u32 i = 0; i < 48; i++) {
        char mask_raw[48];
        memset(mask_raw, 0x7f, sizeof(mask_raw));
        mask_raw[i] = 0;

        m384 mask;
        memcpy(&mask, mask_raw, sizeof(mask));

        storecompressed384(&buf, &val, &mask, 0);

        m384 val_out;
        loadcompressed384(&val_out, &buf, &mask, 0);

        EXPECT_TRUE(!diff384(and384(val, mask), val_out));

        for (u32 j = i + 1; j < 48; j++) {
            mask_raw[j] = 0;
            memcpy(&mask, mask_raw, sizeof(mask));

            storecompressed384(&buf, &val, &mask, 0);
            loadcompressed384(&val_out, &buf, &mask, 0);
            EXPECT_TRUE(!diff384(and384(val, mask), val_out));

            mask_raw[j] = 0x7f;
        }
    }
}

TEST(state_compress, m512_1) {
    char buf[sizeof(m512)] = { 0 };

    for (u32 i = 0; i < 64; i++) {
        char mask_raw[64] = { 0 };
        char val_raw[64] = { 0 };

        memset(val_raw, (i << 2) + 3, 64);

        mask_raw[i] = 0xff;
        val_raw[i] = i;

        mask_raw[63 - i] = 0xff;
        val_raw[63 - i] = i;

        m512 val;
        m512 mask;

        memcpy(&val, val_raw, sizeof(val));
        memcpy(&mask, mask_raw, sizeof(mask));

        storecompressed512(&buf, &val, &mask, 0);

        m512 val_out;
        loadcompressed512(&val_out, &buf, &mask, 0);

        EXPECT_TRUE(!diff512(and512(val, mask), val_out));

        mask_raw[i] = 0x3;
        mask_raw[63 - i] = 0x2f;
        memcpy(&mask, mask_raw, sizeof(mask));
        val_raw[i] = 3;

        storecompressed512(&buf, &val, &mask, 0);
        loadcompressed512(&val_out, &buf, &mask, 0);

        EXPECT_TRUE(!diff512(and512(val, mask), val_out));
    }
}

TEST(state_compress, m512_2) {
    char buf[sizeof(m512)] = { 0 };

    char val_raw[64] = { '0', '1', '2', '3', '4', '5', '6', '7',
                         '8', '9', 'a', 'b', 'c', 'd', 'e', 'f',
                         'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H',
                         'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',
                         'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z',
                         'Q', 'R', ' ', '-', ',', '"', ';', ':',
                         'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n' };
    m512 val;
    memcpy(&val, val_raw, sizeof(val));

    for (u32 i = 0; i < 64; i++) {
        char mask_raw[64];
        memset(mask_raw, 0x7f, sizeof(mask_raw));
        mask_raw[i] = 0;

        m512 mask;
        memcpy(&mask, mask_raw, sizeof(mask));

        storecompressed512(&buf, &val, &mask, 0);

        m512 val_out;
        loadcompressed512(&val_out, &buf, &mask, 0);

        EXPECT_TRUE(!diff512(and512(val, mask), val_out));

        for (u32 j = i + 1; j < 64; j++) {
            mask_raw[j] = 0;
            memcpy(&mask, mask_raw, sizeof(mask));

            storecompressed512(&buf, &val, &mask, 0);
            loadcompressed512(&val_out, &buf, &mask, 0);
            EXPECT_TRUE(!diff512(and512(val, mask), val_out));

            mask_raw[j] = 0x7f;
        }
    }
}
