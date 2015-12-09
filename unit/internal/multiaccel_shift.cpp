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
#include "src/ue2common.h"

#include "gtest/gtest.h"
#include "nfa/multiaccel_common.h"

/*
 * Unit tests for the shifters.
 *
 * This is a bit messy, as shifters are macros, so we're using macros to test
 * other macros.
 */

#define TEST_SHIFT(n) \
    do { \
        u64a val = ((u64a) 1 << n) - 1; \
        JOIN(SHIFT, n)(val); \
        ASSERT_EQ(val, 1); \
    } while (0)

TEST(MultiaccelShift, StaticShift) {
    TEST_SHIFT(1);
    TEST_SHIFT(2);
    TEST_SHIFT(3);
    TEST_SHIFT(4);
    TEST_SHIFT(5);
    TEST_SHIFT(6);
    TEST_SHIFT(7);
    TEST_SHIFT(8);
    TEST_SHIFT(10);
    TEST_SHIFT(11);
    TEST_SHIFT(12);
    TEST_SHIFT(13);
    TEST_SHIFT(14);
    TEST_SHIFT(15);
    TEST_SHIFT(16);
    TEST_SHIFT(17);
    TEST_SHIFT(18);
    TEST_SHIFT(19);
    TEST_SHIFT(20);
    TEST_SHIFT(21);
    TEST_SHIFT(22);
    TEST_SHIFT(23);
    TEST_SHIFT(24);
    TEST_SHIFT(25);
    TEST_SHIFT(26);
    TEST_SHIFT(27);
    TEST_SHIFT(28);
    TEST_SHIFT(29);
    TEST_SHIFT(30);
    TEST_SHIFT(31);
    TEST_SHIFT(32);
}
