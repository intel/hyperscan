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

#include "config.h"

#include "gtest/gtest.h"
#include "nfa/vermicelli.h"

TEST(Vermicelli, ExecNoMatch1) {
    char t1[] = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";

    for (size_t i = 0; i < 16; i++) {
        for (size_t j = 0; j < 16; j++) {
            const u8 *rv = vermicelliExec('a', 0, (u8 *)t1 + i,
                                          (u8 *)t1 + strlen(t1) - j);

            ASSERT_EQ(((size_t)t1 + strlen(t1) - j), (size_t)rv);

            rv = vermicelliExec('B', 0, (u8 *)t1 + i,
                                (u8 *)t1 + strlen(t1) - j);

            ASSERT_EQ(((size_t)t1 + strlen(t1) - j), (size_t)rv);

            rv = vermicelliExec('A', 1, (u8 *)t1 + i,
                                (u8 *)t1 + strlen(t1) - j);

            ASSERT_EQ(((size_t)t1 + strlen(t1) - j), (size_t)rv);
        }
    }
}

TEST(Vermicelli, Exec1) {
    char t1[] = "bbbbbbbbbbbbbbbbbabbbbbbbbbbbbbbbbbbbbbbbbbbbbbbabbbbbbbbbbbb";

    for (size_t i = 0; i < 16; i++) {
        const u8 *rv = vermicelliExec('a', 0, (u8 *)t1 + i,
                                      (u8 *)t1 + strlen(t1));

        ASSERT_EQ((size_t)t1 + 17, (size_t)rv);

        rv = vermicelliExec('A', 1, (u8 *)t1 + i,
                            (u8 *)t1 + strlen(t1));

        ASSERT_EQ((size_t)t1 + 17, (size_t)rv);
    }
}

TEST(Vermicelli,  Exec2) {
    char t1[] = "bbbbbbbbbbbbbbbbbaaaaaaaaaaaaaaaaaaaaaaabbbbbbbbabbbbbbbbbbbb";

    for (size_t i = 0; i < 16; i++) {
        const u8 *rv = vermicelliExec('a', 0, (u8 *)t1 + i,
                                      (u8 *)t1 + strlen(t1));

        ASSERT_EQ((size_t)t1 + 17, (size_t)rv);

        rv = vermicelliExec('A', 1, (u8 *)t1 + i,
                            (u8 *)t1 + strlen(t1));

        ASSERT_EQ((size_t)t1 + 17, (size_t)rv);
    }
}

TEST(Vermicelli,  Exec3) {
    char t1[] = "bbbbbbbbbbbbbbbbbAaaaaaaaaaaaaaaaaaaaaaabbbbbbbbabbbbbbbbbbbb";

    for (size_t i = 0; i < 16; i++) {
        const u8 *rv = vermicelliExec('a', 0, (u8 *)t1 + i,
                                      (u8 *)t1 + strlen(t1));

        ASSERT_EQ((size_t)t1 + 18, (size_t)rv);

        rv = vermicelliExec('A', 1, (u8 *)t1 + i,
                            (u8 *)t1 + strlen(t1));

        ASSERT_EQ((size_t)t1 + 17, (size_t)rv);
    }
}


TEST(Vermicelli, Exec4) {
    char t1[] = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";

    for (size_t i = 0; i < 31; i++) {
        t1[48 - i] = 'a';
        const u8 *rv = vermicelliExec('a', 0, (u8 *)t1, (u8 *)t1 + strlen(t1));

        ASSERT_EQ((size_t)&t1[48 - i], (size_t)rv);

        rv = vermicelliExec('A', 1, (u8 *)t1, (u8 *)t1 + strlen(t1));

        ASSERT_EQ((size_t)&t1[48 - i], (size_t)rv);
    }
}

TEST(DoubleVermicelli, ExecNoMatch1) {
    char t1[] = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";

    for (size_t i = 0; i < 16; i++) {
        for (size_t j = 0; j < 16; j++) {
            const u8 *rv = vermicelliDoubleExec('a', 'b', 0, (u8 *)t1 + i,
                                                (u8 *)t1 + strlen(t1) - j);

            ASSERT_EQ(((size_t)t1 + strlen(t1) - j), (size_t)rv);

            rv = vermicelliDoubleExec('B', 'b', 0, (u8 *)t1 + i,
                                (u8 *)t1 + strlen(t1) - j);

            ASSERT_EQ(((size_t)t1 + strlen(t1) - j), (size_t)rv);

            rv = vermicelliDoubleExec('A', 'B', 1, (u8 *)t1 + i,
                                (u8 *)t1 + strlen(t1) - j);

            ASSERT_EQ(((size_t)t1 + strlen(t1) - j), (size_t)rv);

            /* partial match */
            rv = vermicelliDoubleExec('b', 'B', 0, (u8 *)t1 + i,
                                (u8 *)t1 + strlen(t1) - j);

            ASSERT_EQ(((size_t)t1 + strlen(t1) - j - 1), (size_t)rv);

            /* partial match */
            rv = vermicelliDoubleExec('B', 'A', 1, (u8 *)t1 + i,
                                (u8 *)t1 + strlen(t1) - j);

            ASSERT_EQ(((size_t)t1 + strlen(t1) - j - 1), (size_t)rv);
        }
    }
}

TEST(DoubleVermicelli, Exec1) {
    char t1[] = "bbbbbbbbbbbbbbbbbbabbbbbbbbbbbbbbbbbbbbbbbbbbbbbbabbbbbbbbbbb";

    for (size_t i = 0; i < 16; i++) {
        const u8 *rv = vermicelliDoubleExec('a', 'b', 0, (u8 *)t1 + i,
                                      (u8 *)t1 + strlen(t1));

        ASSERT_EQ((size_t)t1 + 18, (size_t)rv);

        rv = vermicelliDoubleExec('A', 'B', 1, (u8 *)t1 + i,
                            (u8 *)t1 + strlen(t1));

        ASSERT_EQ((size_t)t1 + 18, (size_t)rv);

        rv = vermicelliDoubleExec('b', 'a', 0, (u8 *)t1 + i,
                            (u8 *)t1 + strlen(t1));

        ASSERT_EQ((size_t)t1 + 17, (size_t)rv);

        rv = vermicelliDoubleExec('B', 'A', 1, (u8 *)t1 + i,
                            (u8 *)t1 + strlen(t1));

        ASSERT_EQ((size_t)t1 + 17, (size_t)rv);
    }
}

TEST(DoubleVermicelli,  Exec2) {
    char t1[] = "bbbbbbbbbbbbbbbbbaaaaaaaaaaaaaaaaaaaaaaaabbbbbbbaaaaabbbbbbbb";

    for (size_t i = 0; i < 16; i++) {
        const u8 *rv = vermicelliDoubleExec('a', 'a', 0, (u8 *)t1 + i,
                                      (u8 *)t1 + strlen(t1));

        ASSERT_EQ((size_t)t1 + 17, (size_t)rv);

        rv = vermicelliDoubleExec('A', 'A', 1, (u8 *)t1 + i,
                            (u8 *)t1 + strlen(t1));

        ASSERT_EQ((size_t)t1 + 17, (size_t)rv);
    }
}

TEST(DoubleVermicelli,  Exec3) {
    /*           012345678901234567890123 */
    char t1[] = "bbbbbbbbbbbbbbbbbaAaaAAaaaaaaaaaaaaaaaaaabbbbbbbaaaaabbbbbbbb";

    for (size_t i = 0; i < 16; i++) {
        const u8 *rv = vermicelliDoubleExec('A', 'a', 0, (u8 *)t1 + i,
                                      (u8 *)t1 + strlen(t1));

        ASSERT_EQ((size_t)t1 + 18, (size_t)rv);

        rv = vermicelliDoubleExec('A', 'A', 1, (u8 *)t1 + i,
                                  (u8 *)t1 + strlen(t1));

        ASSERT_EQ((size_t)t1 + 17, (size_t)rv);

        rv = vermicelliDoubleExec('A', 'A', 0, (u8 *)t1 + i,
                                  (u8 *)t1 + strlen(t1));

        ASSERT_EQ((size_t)t1 + 21, (size_t)rv);

        rv = vermicelliDoubleExec('a', 'A', 0, (u8 *)t1 + i,
                                  (u8 *)t1 + strlen(t1));

        ASSERT_EQ((size_t)t1 + 17, (size_t)rv);
    }
}


TEST(DoubleVermicelli, Exec4) {
    char t1[] = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";

    for (size_t i = 0; i < 31; i++) {
        t1[48 - i] = 'a';
        t1[48 - i + 1] = 'a';
        const u8 *rv = vermicelliDoubleExec('a', 'a', 0, (u8 *)t1,
                                            (u8 *)t1 + strlen(t1));

        ASSERT_EQ((size_t)&t1[48 - i], (size_t)rv);

        rv = vermicelliDoubleExec('A', 'A', 1, (u8 *)t1, (u8 *)t1 + strlen(t1));

        ASSERT_EQ((size_t)&t1[48 - i], (size_t)rv);
    }
}

TEST(Vermicelli, noodEarlyExit) {

    // searches that should fail
    static const u8 *lowerAlpha = (const u8*) "abcdefghijklmnopqrstuvwxyz";
    const u8 *la_end = lowerAlpha + 26;
    EXPECT_EQ(la_end, vermicelliExec('0', 0, lowerAlpha, la_end));
    EXPECT_EQ(la_end, vermicelliExec('A', 0, lowerAlpha, la_end));

    // single byte match
    for (unsigned i = 0; i < 26; i++) {
        u8 byte = lowerAlpha[i];
        SCOPED_TRACE(byte);
        EXPECT_EQ(lowerAlpha + i, vermicelliExec(byte, 0, lowerAlpha, la_end));
        byte = toupper(lowerAlpha[i]);
        EXPECT_EQ(lowerAlpha + i, vermicelliExec(byte, 1, lowerAlpha, la_end));
    }
}

TEST(NVermicelli, ExecNoMatch1) {
    char t1[] = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";

    for (size_t i = 0; i < 16; i++) {
        SCOPED_TRACE(i);
        for (size_t j = 0; j < 16; j++) {
            SCOPED_TRACE(j);
            const u8 *rv = nvermicelliExec('b', 0, (u8 *)t1 + i,
                                          (u8 *)t1 + strlen(t1) - j);

            ASSERT_EQ(((size_t)t1 + strlen(t1) - j), (size_t)rv);

            rv = nvermicelliExec('B', 1, (u8 *)t1 + i,
                                (u8 *)t1 + strlen(t1) - j);

            ASSERT_EQ(((size_t)t1 + strlen(t1) - j), (size_t)rv);
        }
    }
}

TEST(NVermicelli, Exec1) {
    char t1[] = "bbbbbbbbbbbbbbbbbabbbbbbbbbbbbbbbbbbbbbbbbbbbbbbabbbbbbbbbbbb";

    for (size_t i = 0; i < 16; i++) {
        SCOPED_TRACE(i);
        const u8 *rv = nvermicelliExec('b', 0, (u8 *)t1 + i,
                                      (u8 *)t1 + strlen(t1));

        ASSERT_EQ((size_t)t1 + 17, (size_t)rv);

        rv = nvermicelliExec('B', 1, (u8 *)t1 + i,
                            (u8 *)t1 + strlen(t1));

        ASSERT_EQ((size_t)t1 + 17, (size_t)rv);
    }
}

TEST(NVermicelli,  Exec2) {
    char t1[] = "bbbbbbbbbbbbbbbbbaaaaaaaaaaaaaaaaaaaaaaabbbbbbbbabbbbbbbbbbbb";

    for (size_t i = 0; i < 16; i++) {
        SCOPED_TRACE(i);
        const u8 *rv = nvermicelliExec('b', 0, (u8 *)t1 + i,
                                      (u8 *)t1 + strlen(t1));

        ASSERT_EQ((size_t)t1 + 17, (size_t)rv);

        rv = nvermicelliExec('B', 1, (u8 *)t1 + i,
                            (u8 *)t1 + strlen(t1));

        ASSERT_EQ((size_t)t1 + 17, (size_t)rv);
    }
}

TEST(NVermicelli,  Exec3) {
    char t1[] = "bbbbbbbbbbbbbbbbbBaaaaaaaaaaaaaaaaaaaaaabbbbbbbbabbbbbbbbbbbb";

    for (size_t i = 0; i < 16; i++) {
        SCOPED_TRACE(i);
        const u8 *rv = nvermicelliExec('b', 0, (u8 *)t1 + i,
                                      (u8 *)t1 + strlen(t1));

        ASSERT_EQ((size_t)t1 + 17, (size_t)rv);

        rv = nvermicelliExec('B', 1, (u8 *)t1 + i,
                            (u8 *)t1 + strlen(t1));

        ASSERT_EQ((size_t)t1 + 18, (size_t)rv);
    }
}

TEST(NVermicelli, Exec4) {
    char t1[] = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";

    for (size_t i = 0; i < 31; i++) {
        SCOPED_TRACE(i);
        t1[48 - i] = 'a';
        const u8 *rv = nvermicelliExec('b', 0, (u8 *)t1, (u8 *)t1 + strlen(t1));

        ASSERT_EQ((size_t)&t1[48 - i], (size_t)rv);

        rv = nvermicelliExec('B', 1, (u8 *)t1, (u8 *)t1 + strlen(t1));

        ASSERT_EQ((size_t)&t1[48 - i], (size_t)rv);
    }
}

TEST(DoubleVermicelliMasked, ExecNoMatch1) {
    std::string t1("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb");
    const u8 *t1_raw = (const u8 *)t1.c_str();

    for (size_t i = 0; i < 16; i++) {
        for (size_t j = 0; j < 16; j++) {
            const u8 *rv = vermicelliDoubleMaskedExec('a', 'b', 0xff, 0xff,
                                                  t1_raw + i,
                                                  t1_raw + t1.length() - i - j);

            ASSERT_EQ(((size_t)t1_raw + t1.length() - i - j), (size_t)rv);

            rv = vermicelliDoubleMaskedExec('B', 'B', 0xff, CASE_CLEAR,
                                            t1_raw + i,
                                            t1_raw + t1.length() - i - j);

            ASSERT_EQ(((size_t)t1_raw + t1.length() - i - j), (size_t)rv);

            rv = vermicelliDoubleMaskedExec('A', 'B', CASE_CLEAR, CASE_CLEAR,
                                            t1_raw + i,
                                            t1_raw + t1.length() -i - j);

            ASSERT_EQ(((size_t)t1_raw + t1.length() - i - j), (size_t)rv);

            /* partial match */
            rv = vermicelliDoubleMaskedExec('B', 'B', CASE_CLEAR, 0xff,
                                            t1_raw + i,
                                            t1_raw + t1.length() - i - j);

            ASSERT_EQ(((size_t)t1_raw + t1.length() - i - j - 1), (size_t)rv);

            rv = vermicelliDoubleMaskedExec('B', 'A', 0xff, 0xff,
                                            t1_raw + i,
                                            t1_raw + t1.length() - i - j);

            ASSERT_EQ(((size_t)t1_raw + t1.length() - i - j), (size_t)rv);
        }
    }
}

TEST(DoubleVermicelliMasked, Exec1) {
    std::string t1("bbbbbbbbbbbbbbbbbbabbbbbbbbbbbbbbbbbbbbbbbbbbbbbbabbbbbbbbbbb");
    const u8 *t1_raw = (const u8 *)t1.c_str();

    for (size_t i = 0; i < 16; i++) {
        const u8 *rv = vermicelliDoubleMaskedExec('a', 'b', 0xff, 0xff,
                                                  t1_raw + i,
                                                  t1_raw + t1.length() - i);

        ASSERT_EQ((size_t)t1_raw + 18, (size_t)rv);

        rv = vermicelliDoubleMaskedExec('A', 'B', CASE_CLEAR, CASE_CLEAR,
                                        t1_raw + i,
                                        t1_raw + t1.length() - i);

        ASSERT_EQ((size_t)t1_raw + 18, (size_t)rv);

        rv = vermicelliDoubleMaskedExec('a', 'B', 0xff, CASE_CLEAR,
                                        t1_raw + i,
                                        t1_raw + t1.length() - i);

        ASSERT_EQ((size_t)t1_raw + 18, (size_t)rv);

        rv = vermicelliDoubleMaskedExec('A', 'b', CASE_CLEAR, 0xff,
                                        t1_raw + i,
                                        t1_raw + t1.length() - i);

        ASSERT_EQ((size_t)t1_raw + 18, (size_t)rv);

        rv = vermicelliDoubleMaskedExec('b', 'a', 0xff, 0xff,
                                        t1_raw + i,
                                        t1_raw + t1.length() - i);

        ASSERT_EQ((size_t)t1_raw + 17, (size_t)rv);

        rv = vermicelliDoubleMaskedExec('B', 'A', CASE_CLEAR, CASE_CLEAR,
                                        t1_raw + i,
                                        t1_raw + t1.length() - i);

        ASSERT_EQ((size_t)t1_raw + 17, (size_t)rv);
    }
}

TEST(DoubleVermicelliMasked,  Exec2) {
    std::string t1("bbbbbbbbbbbbbbbbbaaaaaaaaaaaaaaaaaaaaaaaabbbbbbbaaaaabbbbbbbb");
    const u8 *t1_raw = (const u8 *)t1.c_str();

    for (size_t i = 0; i < 16; i++) {
        const u8 *rv = vermicelliDoubleMaskedExec('a', 'a', 0xff, 0xff,
                                                  t1_raw + i,
                                                  t1_raw + t1.length() - i);

        ASSERT_EQ((size_t)t1_raw + 17, (size_t)rv);

        rv = vermicelliDoubleMaskedExec('A', 'A', CASE_CLEAR, CASE_CLEAR,
                                        t1_raw + i,
                                        t1_raw + t1.length() - i);

        ASSERT_EQ((size_t)t1_raw + 17, (size_t)rv);

        rv = vermicelliDoubleMaskedExec('a', 'A', 0xff, CASE_CLEAR,
                                        t1_raw + i,
                                        t1_raw + t1.length() - i);

        ASSERT_EQ((size_t)t1_raw + 17, (size_t)rv);

        rv = vermicelliDoubleMaskedExec('A', 'a', CASE_CLEAR, 0xff,
                                        t1_raw + i,
                                        t1_raw + t1.length() - i);

        ASSERT_EQ((size_t)t1_raw + 17, (size_t)rv);
}
}

TEST(DoubleVermicelliMasked,  Exec3) {
    /*              012345678901234567890123 */
    std::string t1("bbbbbbbbbbbbbbbbbaAaaAAaaaaaaaaaaaaaaaaaabbbbbbbaaaaabbbbbbbb");
    const u8 *t1_raw = (const u8 *)t1.c_str();

    for (size_t i = 0; i < 16; i++) {
        const u8 *rv = vermicelliDoubleMaskedExec('A', 'a', 0xff, 0xff,
                                                  t1_raw + i,
                                                  t1_raw + t1.length() - i);

        ASSERT_EQ((size_t)t1_raw + 18, (size_t)rv);

        rv = vermicelliDoubleMaskedExec('A', 'A', CASE_CLEAR, CASE_CLEAR,
                                        t1_raw + i,
                                        t1_raw + t1.length() - i);

        ASSERT_EQ((size_t)t1_raw + 17, (size_t)rv);

        rv = vermicelliDoubleMaskedExec('A', 'A', 0xff, 0xff,
                                        t1_raw + i,
                                        t1_raw + t1.length() - i);

        ASSERT_EQ((size_t)t1_raw + 21, (size_t)rv);

        rv = vermicelliDoubleMaskedExec('a', 'A', 0xff, 0xff,
                                        t1_raw + i,
                                        t1_raw + t1.length() - i);

        ASSERT_EQ((size_t)t1_raw + 17, (size_t)rv);

        rv = vermicelliDoubleMaskedExec('a', 'A', 0xff, CASE_CLEAR,
                                        t1_raw + i,
                                        t1_raw + t1.length() - i);

        ASSERT_EQ((size_t)t1_raw + 17, (size_t)rv);

        rv = vermicelliDoubleMaskedExec('A', 'a', CASE_CLEAR, 0xff,
                                        t1_raw + i,
                                        t1_raw + t1.length() - i);

        ASSERT_EQ((size_t)t1_raw + 18, (size_t)rv);
}
}

TEST(DoubleVermicelliMasked, Exec4) {
    std::string t1("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb");
    const u8 *t1_raw = (const u8 *)t1.c_str();

    for (size_t i = 0; i < 31; i++) {
        t1[48 - i] = 'a';
        t1[48 - i + 1] = 'a';
        const u8 *rv = vermicelliDoubleMaskedExec('a', 'a', 0xff, 0xff, t1_raw,
                                                  t1_raw + t1.length());

        ASSERT_EQ((size_t)&t1_raw[48 - i], (size_t)rv);

        rv = vermicelliDoubleMaskedExec('A', 'A', CASE_CLEAR, CASE_CLEAR, t1_raw,
                                        t1_raw + t1.length());

        ASSERT_EQ((size_t)&t1_raw[48 - i], (size_t)rv);
    }
}

