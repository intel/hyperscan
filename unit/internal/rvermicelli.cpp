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
#include "nfa/vermicelli.h"

#define BOUND (~(VERM_BOUNDARY - 1))

TEST(RVermicelli, ExecNoMatch1) {
    char t1[] = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";

    for (size_t i = 0; i < 16; i++) {
        for (size_t j = 0; j < 16; j++) {
            const u8 *begin = (const u8 *)t1 + i;
            const u8 *end = (const u8 *)t1 + strlen(t1) - j;

            const u8 *rv = rvermicelliExec('a', 0, begin, end);
            ASSERT_EQ(begin - 1, rv);

            rv = rvermicelliExec('B', 0, begin, end);
            ASSERT_EQ(begin - 1, rv);

            rv = rvermicelliExec('A', 1, begin, end);
            ASSERT_EQ(begin - 1, rv);
        }
    }
}

TEST(RVermicelli, Exec1) {
    char t1[] = "bbbbbbbbbbbbbbbbbabbbbbbbbbbbbbbbbbbbbbbbbbbbbbbabbbbbbbbbbbbbbbbbbbbb";

    for (size_t i = 0; i < 16; i++) {
        const u8 *rv = rvermicelliExec('a', 0, (u8 *)t1,
                                      (u8 *)t1 + strlen(t1) - i);

        ASSERT_EQ((size_t)t1 + 48, (size_t)rv);

        rv = rvermicelliExec('A', 1, (u8 *)t1 + i, (u8 *)t1 + strlen(t1));

        ASSERT_EQ((size_t)t1 + 48, (size_t)rv);
    }
}

TEST(RVermicelli, Exec2) {
    char t1[] = "bbbbbbbbbbbbbbbbbabbbbbbbbaaaaaaaaaaaaaaaaaaaaaaabbbbbbbbbbbbbbbbbbbbbb";

    for (size_t i = 0; i < 16; i++) {
        const u8 *rv = rvermicelliExec('a', 0, (u8 *)t1,
                                       (u8 *)t1 + strlen(t1) - i);

        ASSERT_EQ((size_t)t1 + 48, (size_t)rv);

        rv = rvermicelliExec('A', 1, (u8 *)t1, (u8 *)t1 + strlen(t1) - i);

        ASSERT_EQ((size_t)t1 + 48, (size_t)rv);
    }
}

TEST(RVermicelli, Exec3) {
    char t1[] = "bbbbbbbbbbbbbbbbbabbbbbbbbaaaaaaaaaaaaaaaaaaaaaaAbbbbbbbbbbbbbbbbbbbbbb";

    for (size_t i = 0; i < 16; i++) {
        const u8 *rv = rvermicelliExec('a', 0, (u8 *)t1,
                                      (u8 *)t1 + strlen(t1) - i);

        ASSERT_EQ((size_t)t1 + 47, (size_t)rv);

        rv = rvermicelliExec('A', 1, (u8 *)t1, (u8 *)t1 + strlen(t1) - i);

        ASSERT_EQ((size_t)t1 + 48, (size_t)rv);
    }
}

TEST(RVermicelli, Exec4) {
    char t1[] = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";

    for (size_t i = 0; i < 31; i++) {
        t1[16 + i] = 'a';
        const u8 *rv = rvermicelliExec('a', 0, (u8 *)t1, (u8 *)t1 + strlen(t1));

        ASSERT_EQ((size_t)&t1[16 + i], (size_t)rv);

        rv = rvermicelliExec('A', 1, (u8 *)t1, (u8 *)t1 + strlen(t1));

        ASSERT_EQ((size_t)&t1[16 + i], (size_t)rv);
    }
}

TEST(RDoubleVermicelli, Exec1) {
    char t1[] = "bbbbbbbbbbbbbbbbbbabbbbbbbbbbbbbbbbbbbbbbbbbbbbbbabbbbbbbbbbbbbbbbbbbbb";

    for (size_t i = 0; i < 16; i++) {
        const u8 *rv = rvermicelliDoubleExec('a', 'b', 0, (u8 *)t1,
                                      (u8 *)t1 + strlen(t1) - i);

        ASSERT_EQ((size_t)t1 + 50, (size_t)rv);

        rv = rvermicelliDoubleExec('A', 'B', 1, (u8 *)t1 + i,
                            (u8 *)t1 + strlen(t1));

        ASSERT_EQ((size_t)t1 + 50, (size_t)rv);

        rv = rvermicelliDoubleExec('b', 'a', 0, (u8 *)t1 + i,
                            (u8 *)t1 + strlen(t1));

        ASSERT_EQ((size_t)t1 + 49, (size_t)rv);

        rv = rvermicelliDoubleExec('B', 'A', 1, (u8 *)t1 + i,
                            (u8 *)t1 + strlen(t1));

        ASSERT_EQ((size_t)t1 + 49, (size_t)rv);
    }
}

TEST(RDoubleVermicelli, Exec2) {
    char t1[] = "bbbbbbbbbbbbbbbbbaaaaaaaaaaaaaaaaaaaaaaaabbbbbbbaaaaabbbbbbbbbbbbbbbbbb";

    for (size_t i = 0; i < 16; i++) {
        const u8 *rv = rvermicelliDoubleExec('a', 'a', 0, (u8 *)t1,
                                      (u8 *)t1 + strlen(t1) - i);

        ASSERT_EQ((size_t)t1 + 52, (size_t)rv);

        rv = rvermicelliDoubleExec('A', 'A', 1, (u8 *)t1,
                            (u8 *)t1 + strlen(t1) - i);

        ASSERT_EQ((size_t)t1 + 52, (size_t)rv);
    }
}

TEST(RDoubleVermicelli, Exec3) {
    /*           012345678901234567890123 */
    char t1[] = "bbbbbbbbbbbbbbbbbaAaaAAaaaaaaaaaaaaaaaaaabbbbbbbaaaaabbbbbbbbbbbbbbbbbb";

    for (size_t i = 0; i < 16; i++) {
        const u8 *rv = rvermicelliDoubleExec('A', 'a', 0, (u8 *)t1,
                                      (u8 *)t1 + strlen(t1) - i );

        ASSERT_EQ((size_t)t1 + 23, (size_t)rv);

        rv = rvermicelliDoubleExec('A', 'A', 1, (u8 *)t1,
                                  (u8 *)t1 + strlen(t1) - i);

        ASSERT_EQ((size_t)t1 + 52, (size_t)rv);

        rv = rvermicelliDoubleExec('A', 'A', 0, (u8 *)t1,
                                  (u8 *)t1 + strlen(t1) - i);

        ASSERT_EQ((size_t)t1 + 22, (size_t)rv);

        rv = rvermicelliDoubleExec('a', 'A', 0, (u8 *)t1,
                                  (u8 *)t1 + strlen(t1) - i);

        ASSERT_EQ((size_t)t1 + 21, (size_t)rv);
    }
}

TEST(RDoubleVermicelli, Exec4) {
    char t1[] = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";

    for (size_t i = 0; i < 31; i++) {
        t1[32 + i] = 'a';
        t1[32 + i - 1] = 'a';
        const u8 *rv = rvermicelliDoubleExec('a', 'a', 0, (u8 *)t1,
                                            (u8 *)t1 + strlen(t1));
        ASSERT_EQ((size_t)&t1[32 + i], (size_t)rv);

        rv = rvermicelliDoubleExec('A', 'A', 1, (u8 *)t1, (u8 *)t1 + strlen(t1));

        ASSERT_EQ((size_t)&t1[32 + i], (size_t)rv);
    }
}

TEST(RDoubleVermicelli, Exec5) {
    char t1[] = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";

    for (size_t i = 0; i < 16; i++) {
        for (size_t j = 1; j <= 16; j++) {
            t1[strlen(t1) - i - j] = 'a';
            const u8 *rv = rvermicelliDoubleExec('b', 'a', 0, (u8 *)t1,
                                                 (u8 *)t1 + strlen(t1) - i);

            ASSERT_EQ((size_t)&t1[strlen(t1) - i - j], (size_t)rv);

            rv = rvermicelliDoubleExec('B', 'A', 1, (u8 *)t1,
                                       (u8 *)t1 + strlen(t1) -i );

            ASSERT_EQ((size_t)&t1[strlen(t1) - i - j], (size_t)rv);

            t1[strlen(t1) - i - j] = 'b';
        }
    }
}
