/*
 * Copyright (c) 2016, Intel Corporation
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

#include "rose/validate_mask.h"
#include "gtest/gtest.h"

#define ONES32 0xffffffffu

union RoseLookaroundMask32 {
    m256 a256;
    u8 a8[32];
};

struct ValidateMask32TestInfo {
    RoseLookaroundMask32 data;
    u32 valid_mask;
    RoseLookaroundMask32 and_mask;
    RoseLookaroundMask32 cmp_mask;
    u32 neg_mask;
};

struct ValidateMask32InitInfo {
    int idx;
    u8 data;
    u8 and_mask;
    u8 cmp_mask;
    u8 neg_mask;
};


static const ValidateMask32InitInfo testBasicIdx[][33] = {
    {
        {1, 0x34, 0xf8, 0x30, 0},
        {2, 0x34, 0xf8, 0x30, 0},
        {8, 0x23, 0xff, 0x23, 0},
        {9, 0x34, 0xf8, 0x30, 0},
        {10, 0x41, 0xdf, 0x41, 0},
        {11, 0x63, 0xdd, 0x41, 0},
        {12, 0x61, 0xdd, 0x41, 0},
        {13, 0x41, 0xdf, 0x41, 0},
        {14, 0x61, 0xdf, 0x41, 0},
        {15, 0x41, 0xdf, 0x41, 0},
        {16, 0x43, 0xdd, 0x41, 0},
        {17, 0x61, 0xdd, 0x41, 0},
        {23, 0x63, 0xdd, 0x41, 0},
        {24, 0x4f, 0xfc, 0x4c, 0},
        {25, 0x4d, 0xfc, 0x4c, 0},
        {26, 0x4d, 0xfc, 0x4c, 0},
        {-1, 0, 0, 0, 0},
    },
    {
        {11, 0, 0xff, 0x55, 1},
        {12, 0, 0xff, 0x36, 1},
        {13, 0, 0xfe, 0x34, 1},
        {14, 0x4d, 0xfe, 0x4c, 0},
        {15, 0x41, 0xbf, 0x01, 0},
        {16, 0x53, 0xdf, 0x73, 1},
        {17, 0x4b, 0, 0, 0},
        {18, 0, 0x2c, 0x2c, 1},
        {-1, 0, 0, 0, 0},
    },
    {
        {15, 0x46, 0xdf, 0x46, 0},
        {16, 0x4f, 0xdf, 0x46, 1},
        {17, 0x6f, 0xff, 0x6f, 0},
        {18, 0x31, 0xfe, 0x30, 0},
        {19, 0x34, 0xf8, 0x30, 0},
        {20, 0x66, 0xc0, 0x40, 0},
        {21, 0x6f, 0xf0, 0x60, 0},
        {22, 0x6f, 0, 0, 0},
        {23, 0x46, 0xdf, 0x44, 1},
        {24, 0x4f, 0xdf, 0x46, 1},
        {25, 0x6f, 0xff, 0x4f, 1},
        {26, 0x31, 0xfe, 0x30, 0},
        {27, 0x34, 0xf8, 0x34, 1},
        {28, 0x66, 0xc0, 0x60, 1},
        {29, 0x6f, 0xf0, 0x6f, 1},
        {30, 0x6f, 0, 0x60, 1},
        {-1, 0, 0, 0, 0},
    },
    {
        {31, 0x4a, 0x80, 0, 0},
        {-1, 0, 0, 0, 1},
    },
    {
        {12, 0x2b, 0x3d, 0x2d, 1},
        {13, 0x2b, 0x3d, 0x4c, 1},
        {23, 0x4a, 0x88, 0x0a, 1},
        {-1, 0, 0, 0, 0},
    },
};

static void initTestInfo(ValidateMask32TestInfo &t) {
    t.data.a256 = zeroes256();
    t.valid_mask = 0xffffffff;
    t.and_mask.a256 = zeroes256();
    t.cmp_mask.a256 = zeroes256();
    t.neg_mask = 0;
};


static
int testBasicInit(ValidateMask32TestInfo *testB) {
    int len = 0;
    ValidateMask32TestInfo t;
    for (size_t i = 0; i < ARRAY_LENGTH(testBasicIdx); i++) {
        initTestInfo(t);
        for (const auto &line: testBasicIdx[i]) {
            if (line.idx < 0) {
                break;
            }
            int index = line.idx;
            t.data.a8[index] = line.data;
            t.and_mask.a8[index] = line.and_mask;
            t.cmp_mask.a8[index] = line.cmp_mask;
            t.neg_mask |= line.neg_mask << index;
        }
        testB[i] = t;
        len++;
    }
    return len;
}

TEST(ValidateMask32, testMask32_1) {
    ValidateMask32TestInfo testBasic[20];
    int test_len = testBasicInit(testBasic);
    for (int i = 0; i < test_len; i++) {
        const auto t = testBasic[i];
        EXPECT_EQ(1, validateMask32(t.data.a256, t.valid_mask,
                                    t.and_mask.a256, t.cmp_mask.a256,
                                    t.neg_mask));
    }
}

TEST(ValidateMask32, testMask32_2) {
    ValidateMask32TestInfo testBasic[20];
    int test_len = testBasicInit(testBasic);
    for (int left = 0; left <= 32; left++) {
        for (int right = 0; right + left < 32; right++) {
            u32 valid_mask = ONES32 << (left + right) >> left;
            for (int i = 0; i < test_len; i++) {
                const auto &t = testBasic[i];
                int bool_result;
                bool_result = !(valid_mask & t.neg_mask);
                EXPECT_EQ(bool_result, validateMask32(t.data.a256,
                                                      valid_mask,
                                                      t.and_mask.a256,
                                                      t.cmp_mask.a256,
                                                      0));
                bool_result = (valid_mask & t.neg_mask) == valid_mask;
                EXPECT_EQ(bool_result, validateMask32(t.data.a256,
                                                      valid_mask,
                                                      t.and_mask.a256,
                                                      t.cmp_mask.a256,
                                                      ONES32));
            }
        }
    }
}

TEST(ValidateMask32, testMask32_3) {
    ValidateMask32TestInfo testBasic[20];
    testing::internal::Random neg_mask_rand(451);
    int test_len = testBasicInit(testBasic);
    for (int left = 0; left <= 32; left++) {
        for (int right = 0; right + left < 32; right++) {
            u32 valid_mask = ONES32 << (left + right) >> left;
            for (int i = 0; i < test_len; i++) {
                const auto &t = testBasic[i];
                int bool_result;
                for (int j = 0; j < 5000; j++) {
                    u32 neg_mask = neg_mask_rand.Generate(1u << 31);
                    bool_result = (neg_mask & valid_mask) ==
                                  (t.neg_mask & valid_mask);
                    EXPECT_EQ(bool_result, validateMask32(t.data.a256,
                                                          valid_mask,
                                                          t.and_mask.a256,
                                                          t.cmp_mask.a256,
                                                          neg_mask));
                }
            }
        }
    }
}
