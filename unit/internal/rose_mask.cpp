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

#define ONES64 0xffffffffffffffffULL

/* valid_data_mask is flexible, don't need to be fixed in Info */
struct ValidateMaskTestInfo {
    u64a data;
    u64a and_mask;
    u64a cmp_mask;
    u64a neg_mask;
};

static const ValidateMaskTestInfo testBasic[] = {
    /* data is randomly picked */
    {0x1234abcd4321dcbaULL, 0xff09bbdd7f7ffeffULL,
     0x1200abcd4561dcbbULL, 0xffff00ffULL},
    /* data = "VaLiDaTe" */
    {0x56614c6944615465ULL, 0xe0feffffdf7b5480ULL,
     0x40614c6946615400ULL, 0xff0000ff000000ULL},
    /* data = "\0\0\0MASK\0" */
    {0x4d41534b00ULL, 0xfffffefebfdf002cULL,
     0x5536344c0173002cULL, 0xffffff0000ff00ffULL},
    /* data = "FOo14foo" */
    {0x464f6f3134666f6fULL, 0xdfdffffef8c0f000ULL,
     0x46466f3030406000ULL, 0xff000000000000ULL},
    /* data = "FOo14foo" with different cmp_mask and neg_mask*/
    {0x464f6f3134666f6fULL, 0xdfdffffef8c0f000ULL,
     0x44464f3034606f60ULL, 0xffffff00ffffffffULL},
};

/*
 * generate 37 different valid_data_mask
 * 8 from 0xff to 0xff00000000000000
 * 7 from 0xffff to 0xffff000000000000
 * ...
 * 0xffffffffffffffff and 0
 */
static int initLegalValidMasks(u64a validMasks[]) {
    u64a data = ONES64;
    int num = 0;
    for (int i = 0; i < 64; i += 8) {
        for (int j = 0; j <= i; j += 8) {
            validMasks[num] = data << j;
            num++;
        }
        data >>= 8;
    }
    validMasks[num] = 0;
    num++;
    return num;
}

/*
 * generate all 256 neg_masks
 * including 0, 0xff, 0xff00,..., 0xffffffffffffffff
 */
static int initLegalNegMasks(u64a negMasks[]) {
    u64a data = 0;
    u64a offset;
    int num = 0;
    while (data != ONES64) {
        negMasks[num] = data;
        num++;
        offset = (data | (data +1)) ^ data;
        data += 0xfeULL * offset + 1;
    }
    negMasks[num] = data;
    num++;
    return num;
}


/*
 * check all legal valid_mask(37 different) for validateMask[]
 */
TEST(ValidateMask, ValidMaskTests) {
    u64a validMasks[256];
    int num = initLegalValidMasks(validMasks);

    for (const auto &t : testBasic) {
        for (int i = 0; i < num; i++) {
            EXPECT_EQ(1, validateMask(t.data,
                                      validMasks[i],
                                      t.and_mask,
                                      t.cmp_mask,
                                      t.neg_mask));
        }
    }
}

/*
 * fix neg_mask to 0 and ONES64,
 * check output of ValidateMask on different valid_mask,
 * for neg_mask = 0,
 */
TEST(ValidateMask, AdvancedValidMaskTests) {
    u64a validMasks[256];
    int num = initLegalValidMasks(validMasks);
    int bool_result;
    for (const auto &t: testBasic) {
        for (int i = 0; i < num; i++) {
            bool_result = !(validMasks[i] & t.neg_mask);
            EXPECT_EQ(bool_result, validateMask(t.data,
                                                validMasks[i],
                                                t.and_mask,
                                                t.cmp_mask,
                                                0));
            bool_result = (validMasks[i] | t.neg_mask) == t.neg_mask;
            EXPECT_EQ(bool_result, validateMask(t.data,
                                                validMasks[i],
                                                t.and_mask,
                                                t.cmp_mask,
                                                ONES64));
        }
    }
}

/*
 * test every pair of valid_data_mask and neg_mask
 * and compute the expect output by a formula
 */
TEST(ValidateMask, FullTests) {
    u64a validMasks[256];
    u64a negMasks[256];
    int vm_num = initLegalValidMasks(validMasks);
    int nm_num = initLegalNegMasks(negMasks);
    int bool_result;
    for (const auto &t: testBasic) {
        for (int i = 0; i < vm_num; i++) {
            for (int j = 0; j < nm_num; j++) {
                /*
                 * treat t.neg_mask as a truthtable (a negative truthtable)
                 * we expect validateMask output 1 if and only if
                 * the truthtable(tt) and neg_mask(nm) looks same
                 * under "&" operation with valid_data_mask(vdm)
                 * that is
                 * output = (tt & vdm) == (nm & vdm) ? 1 : 0;
                 */
                bool_result = (t.neg_mask & validMasks[i]) ==
                              (negMasks[j] & validMasks[i]);
                EXPECT_EQ(bool_result, validateMask(t.data,
                                                    validMasks[i],
                                                    t.and_mask,
                                                    t.cmp_mask,
                                                    negMasks[j]));
            }
        }
    }
}

/*
 * drop the original validateMask[].neg_mask
 * and test more neg_mask and valid_mask manually
 */
TEST(ValidateMask, ManualTest_0) {
    const auto &t = testBasic[0];
    EXPECT_EQ(1, validateMask(t.data, ONES64 << 8,
                              t.and_mask, t.cmp_mask, 0xffff0000ULL));
    EXPECT_EQ(1, validateMask(t.data, (ONES64 << 16) >> 8,
                              t.and_mask, t.cmp_mask, 0xffff0000ULL));
    EXPECT_EQ(1, validateMask(t.data, ONES64 << 16,
                              t.and_mask, t.cmp_mask, 0xffffff00ULL));
    EXPECT_EQ(1, validateMask(t.data, ONES64 << 24,
                              t.and_mask, t.cmp_mask, 0xff00ffffULL));
    EXPECT_EQ(1, validateMask(t.data, ONES64 >> 32,
                              t.and_mask, t.cmp_mask, 0xffffffff00ffULL));
    EXPECT_EQ(1, validateMask(t.data, ONES64 >> 40,
                              t.and_mask, t.cmp_mask, 0xff00ffULL));
    EXPECT_EQ(1, validateMask(t.data, 0,
                              t.and_mask, t.cmp_mask, ONES64));
    EXPECT_EQ(1, validateMask(t.data, 0,
                              t.and_mask, t.cmp_mask, ~t.neg_mask));
    EXPECT_EQ(0, validateMask(t.data, ONES64 << 16,
                              t.and_mask, t.cmp_mask, 0xff0000ffULL));
    EXPECT_EQ(0, validateMask(t.data, ONES64,
                              t.and_mask, t.cmp_mask, 0xffff0000ULL));
    EXPECT_EQ(0, validateMask(t.data, ONES64 >> 32,
                              t.and_mask, t.cmp_mask, 0xff00ffULL));
    EXPECT_EQ(0, validateMask(t.data, ONES64 << 8,
                              t.and_mask, t.cmp_mask, 0xffffffffULL));
    EXPECT_EQ(0, validateMask(t.data, ONES64 << 16,
                              t.and_mask, t.cmp_mask, 0xff0000ffULL));
}

TEST(ValidateMask, ManualTest_1) {
    const auto &t = testBasic[1];
    EXPECT_EQ(1, validateMask(t.data, ONES64 << 16,
                              t.and_mask, t.cmp_mask, 0xff0000ff00ffffULL));
    EXPECT_EQ(1, validateMask(t.data, ONES64 << 32,
                              t.and_mask, t.cmp_mask, 0xff000000000000ULL));
    EXPECT_EQ(1, validateMask(t.data, ONES64 << 32,
                              t.and_mask, t.cmp_mask, 0xff0000ffff00ffULL));
    EXPECT_EQ(1, validateMask(t.data, ONES64 << 56,
                              t.and_mask, t.cmp_mask, 0));
    EXPECT_EQ(1, validateMask(t.data, ONES64 >> 8,
                              t.and_mask, t.cmp_mask, 0xffff0000ff000000ULL));
    EXPECT_EQ(1, validateMask(t.data, ONES64 >> 16,
                              t.and_mask, t.cmp_mask, 0xff000000ULL));
    EXPECT_EQ(1, validateMask(t.data, (ONES64 << 32) >> 16,
                              t.and_mask, t.cmp_mask, 0xff00ff00));
    EXPECT_EQ(1, validateMask(t.data, ONES64 >> 40,
                              t.and_mask, t.cmp_mask, 0xff00000000ULL));
    EXPECT_EQ(0, validateMask(t.data, ONES64,
                              t.and_mask, t.cmp_mask, 0));
    EXPECT_EQ(0, validateMask(t.data, ONES64 << 48,
                              t.and_mask, t.cmp_mask, 0));
    EXPECT_EQ(0, validateMask(t.data, ONES64 << 56,
                              t.and_mask, t.cmp_mask, 0xff00000000000000ULL));
    EXPECT_EQ(0, validateMask(t.data, ONES64 << 16,
                              t.and_mask, t.cmp_mask, 0xff0000ffff0000ULL));
    EXPECT_EQ(0, validateMask(t.data, ONES64 >> 8,
                              t.and_mask, t.cmp_mask, 0xff000000ULL));
    EXPECT_EQ(0, validateMask(t.data, ONES64 >> 16,
                              t.and_mask, t.cmp_mask, 0xffff000000ULL));
    EXPECT_EQ(0, validateMask(t.data, (ONES64 << 40) >> 16,
                              t.and_mask, t.cmp_mask, 0xff000000000000ULL));
    EXPECT_EQ(0, validateMask(t.data, ONES64 << 8,
                              t.and_mask, t.cmp_mask, ONES64));
}

TEST(ValidateMask, ManualTest_2) {
    const auto &t = testBasic[2];
    EXPECT_EQ(1, validateMask(t.data, ONES64 << 24,
                              t.and_mask, t.cmp_mask, 0xffffff0000000000ULL));
    EXPECT_EQ(1, validateMask(t.data, ONES64 << 56,
                              t.and_mask, t.cmp_mask, 0xff00000000000000ULL));
    EXPECT_EQ(1, validateMask(t.data, ONES64 << 56,
                              t.and_mask, t.cmp_mask, 0xff00ffffff00ffffULL));
    EXPECT_EQ(1, validateMask(t.data, 0,
                              t.and_mask, t.cmp_mask, ONES64));
    EXPECT_EQ(1, validateMask(t.data, ONES64 >> 24,
                              t.and_mask, t.cmp_mask, 0xff00ffULL));
    EXPECT_EQ(1, validateMask(t.data, ONES64 >> 32,
                              t.and_mask, t.cmp_mask, 0xffff00ff00ffULL));
    EXPECT_EQ(1, validateMask(t.data, (ONES64 << 32) >> 24,
                              t.and_mask, t.cmp_mask, 0xff0000ULL));
    EXPECT_EQ(1, validateMask(t.data, (ONES64 << 32) >> 24,
                              t.and_mask, t.cmp_mask, 0xff00ffULL));
    EXPECT_EQ(1, validateMask(t.data, (ONES64 << 56) >> 40,
                              t.and_mask, t.cmp_mask, 0xff0000ULL));
    EXPECT_EQ(1, validateMask(t.data, (ONES64 << 56) >> 32,
                              t.and_mask, t.cmp_mask, 0));
    EXPECT_EQ(1, validateMask(t.data, ONES64 >> 40,
                              t.and_mask, t.cmp_mask, 0xffffffff00ffULL));
    EXPECT_EQ(0, validateMask(t.data, ONES64,
                              t.and_mask, t.cmp_mask, 0));
    EXPECT_EQ(0, validateMask(t.data, ONES64,
                              t.and_mask, t.cmp_mask, ONES64));
    EXPECT_EQ(0, validateMask(t.data, ONES64 << 56,
                              t.and_mask, t.cmp_mask, 0));
    EXPECT_EQ(0, validateMask(t.data, ONES64 << 48,
                              t.and_mask, t.cmp_mask, 0xff00000000000000ULL));
    EXPECT_EQ(0, validateMask(t.data, ONES64 << 8,
                              t.and_mask, t.cmp_mask, 0xffffff00000000ffULL));
    EXPECT_EQ(0, validateMask(t.data, ONES64 >> 32,
                              t.and_mask, t.cmp_mask, 0xffff00ULL));
    EXPECT_EQ(0, validateMask(t.data, ONES64 >> 32,
                              t.and_mask, t.cmp_mask, 0xffffffULL));
    EXPECT_EQ(0, validateMask(t.data, ONES64 >> 16,
                              t.and_mask, t.cmp_mask, 0xff00ffULL));
    EXPECT_EQ(0, validateMask(t.data, (ONES64 << 32) >> 24,
                              t.and_mask, t.cmp_mask, 0));
    EXPECT_EQ(0, validateMask(t.data, (ONES64 << 32) >> 24,
                              t.and_mask, t.cmp_mask, 0xffffff00000000ffULL));
    EXPECT_EQ(0, validateMask(t.data, (ONES64 << 32) >> 24,
                              t.and_mask, t.cmp_mask, 0xffffff000000ff00ULL));
    EXPECT_EQ(0, validateMask(t.data, (ONES64 << 56) >> 40,
                              t.and_mask, t.cmp_mask, 0));
    EXPECT_EQ(0, validateMask(t.data, (ONES64 << 56) >> 48,
                              t.and_mask, t.cmp_mask, 0xff00ULL));
}
