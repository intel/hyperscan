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
#include "util/compare.h"
#include <ctype.h>

using namespace std;
using namespace testing;

class compare : public TestWithParam<int> {
};

TEST_P(compare, my_our_is) {
    int c = GetParam();

    EXPECT_EQ(!!islower(c), !!myislower(c));
    EXPECT_EQ(!!isupper(c), !!myisupper(c));
    EXPECT_EQ(!!isalpha(c), !!ourisalpha(c));
    EXPECT_EQ(!!isprint(c), !!ourisprint(c));
}

TEST_P(compare, my_to) {
    int c = GetParam();

    EXPECT_EQ((char)tolower(c), mytolower(c));
    EXPECT_EQ((char)toupper(c), mytoupper(c));
}

TEST_P(compare, their32) {
    u32 c = (u32)GetParam();
    u32 cu = (u8)toupper(c);
    u32 cccc = c | (c << 8) | (c << 16) | (c << 24);
    u32 uuuu = cu | (cu << 8) | (cu << 16) | (cu << 24);

    EXPECT_EQ(uuuu, theirtoupper32(cccc));

    EXPECT_EQ(uuuu & 0xffU, theirtoupper32(cccc & 0xffU));
    EXPECT_EQ(uuuu & 0xff00U, theirtoupper32(cccc & 0xff00U));
    EXPECT_EQ(uuuu & 0xff0000U, theirtoupper32(cccc & 0xff0000U));
    EXPECT_EQ(uuuu & 0xff000000U, theirtoupper32(cccc & 0xff000000U));

    EXPECT_EQ(uuuu | 0xffffff00U, theirtoupper32(cccc | 0xffffff00U));
    EXPECT_EQ(uuuu | 0xffff00ffU, theirtoupper32(cccc | 0xffff00ffU));
    EXPECT_EQ(uuuu | 0xff00ffffU, theirtoupper32(cccc | 0xff00ffffU));
    EXPECT_EQ(uuuu | 0x00ffffffU, theirtoupper32(cccc | 0x00ffffffU));
}

TEST_P(compare, their64) {
    u64a c = (u64a)GetParam();
    u64a cu = (u8)toupper(c);
    u64a cccc = c | (c << 8) | (c << 16) | (c << 24);
    cccc |= cccc << 32;
    u64a uuuu = cu | (cu << 8) | (cu << 16) | (cu << 24);
    uuuu |= uuuu << 32;

    EXPECT_EQ(uuuu, theirtoupper64(cccc));

    EXPECT_EQ(uuuu & 0xffULL, theirtoupper64(cccc & 0xffULL));
    EXPECT_EQ(uuuu & (0xffULL <<  8), theirtoupper64(cccc &  (0xffULL << 8)));
    EXPECT_EQ(uuuu & (0xffULL << 16), theirtoupper64(cccc & (0xffULL << 16)));
    EXPECT_EQ(uuuu & (0xffULL << 24), theirtoupper64(cccc & (0xffULL << 24)));
    EXPECT_EQ(uuuu & (0xffULL << 32), theirtoupper64(cccc & (0xffULL << 32)));
    EXPECT_EQ(uuuu & (0xffULL << 40), theirtoupper64(cccc & (0xffULL << 40)));
    EXPECT_EQ(uuuu & (0xffULL << 48), theirtoupper64(cccc & (0xffULL << 48)));
    EXPECT_EQ(uuuu & (0xffULL << 56), theirtoupper64(cccc & (0xffULL << 56)));

    EXPECT_EQ(uuuu & ~0xffULL,         theirtoupper64(cccc & ~0xffULL));
    EXPECT_EQ(uuuu & ~(0xffULL <<  8), theirtoupper64(cccc &  ~(0xffULL << 8)));
    EXPECT_EQ(uuuu & ~(0xffULL << 16), theirtoupper64(cccc & ~(0xffULL << 16)));
    EXPECT_EQ(uuuu & ~(0xffULL << 24), theirtoupper64(cccc & ~(0xffULL << 24)));
    EXPECT_EQ(uuuu & ~(0xffULL << 32), theirtoupper64(cccc & ~(0xffULL << 32)));
    EXPECT_EQ(uuuu & ~(0xffULL << 40), theirtoupper64(cccc & ~(0xffULL << 40)));
    EXPECT_EQ(uuuu & ~(0xffULL << 48), theirtoupper64(cccc & ~(0xffULL << 48)));
    EXPECT_EQ(uuuu & ~(0xffULL << 56), theirtoupper64(cccc & ~(0xffULL << 56)));

}


INSTANTIATE_TEST_CASE_P(compare, compare, Range((int)0, (int)256));

