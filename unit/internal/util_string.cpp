/*
 * Copyright (c) 2015-2017, Intel Corporation
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
#include "util/ue2string.h"

using namespace ue2;

#if defined(DUMP_SUPPORT)

namespace ue2 {

static void PrintTo(const ue2_literal &lit, ::std::ostream *os) {
    *os << dumpString(lit);
}

} // namespace ue2

#endif // DUMP_SUPPORT

TEST(string, case_iter1) {
    const char * const expected[] = {
        "3FOO-BAR",
        "3FOO-BAr",
        "3FOO-BaR",
        "3FOO-Bar",
        "3FOO-bAR",
        "3FOO-bAr",
        "3FOO-baR",
        "3FOO-bar",
        "3FOo-BAR",
        "3FOo-BAr",
        "3FOo-BaR",
        "3FOo-Bar",
        "3FOo-bAR",
        "3FOo-bAr",
        "3FOo-baR",
        "3FOo-bar",
        "3FoO-BAR",
        "3FoO-BAr",
        "3FoO-BaR",
        "3FoO-Bar",
        "3FoO-bAR",
        "3FoO-bAr",
        "3FoO-baR",
        "3FoO-bar",
        "3Foo-BAR",
        "3Foo-BAr",
        "3Foo-BaR",
        "3Foo-Bar",
        "3Foo-bAR",
        "3Foo-bAr",
        "3Foo-baR",
        "3Foo-bar",
        "3fOO-BAR",
        "3fOO-BAr",
        "3fOO-BaR",
        "3fOO-Bar",
        "3fOO-bAR",
        "3fOO-bAr",
        "3fOO-baR",
        "3fOO-bar",
        "3fOo-BAR",
        "3fOo-BAr",
        "3fOo-BaR",
        "3fOo-Bar",
        "3fOo-bAR",
        "3fOo-bAr",
        "3fOo-baR",
        "3fOo-bar",
        "3foO-BAR",
        "3foO-BAr",
        "3foO-BaR",
        "3foO-Bar",
        "3foO-bAR",
        "3foO-bAr",
        "3foO-baR",
        "3foO-bar",
        "3foo-BAR",
        "3foo-BAr",
        "3foo-BaR",
        "3foo-Bar",
        "3foo-bAR",
        "3foo-bAr",
        "3foo-baR",
        "3foo-bar",
    };

    case_iter it = caseIterateBegin(ue2_literal(expected[17], true));
    case_iter ite = caseIterateEnd();

    u32 i = 0;
    for(; it != ite; ++it, ++i) {
        EXPECT_EQ(expected[i], *it);
    }
    EXPECT_EQ(ARRAY_LENGTH(expected), i);
}

TEST(string, case_iter2) {
    case_iter it = caseIterateBegin(ue2_literal("4A", true));
    case_iter ite = caseIterateEnd();

    EXPECT_TRUE(ite != it);
    EXPECT_EQ("4A", *it);
    ++it;
    EXPECT_TRUE(ite != it);
    EXPECT_EQ("4a", *it);
    ++it;
    EXPECT_TRUE(!(ite != it));
}

TEST(string, case_iter3a) {
    ue2_literal src;
    src.push_back('a', false);
    src.push_back('B', true);
    src.push_back('c', false);

    case_iter it = caseIterateBegin(src);
    case_iter ite = caseIterateEnd();

    EXPECT_TRUE(ite != it);
    EXPECT_EQ("aBc", *it);
    ++it;
    EXPECT_TRUE(ite != it);
    EXPECT_EQ("abc", *it);
    ++it;
    EXPECT_TRUE(!(ite != it));
}

TEST(string, case_iter3b) {
    ue2_literal src;
    src.push_back('A', false);
    src.push_back('B', true);
    src.push_back('C', false);

    case_iter it = caseIterateBegin(src);
    case_iter ite = caseIterateEnd();

    EXPECT_TRUE(ite != it);
    EXPECT_EQ("ABC", *it);
    ++it;
    EXPECT_TRUE(ite != it);
    EXPECT_EQ("AbC", *it);
    ++it;
    EXPECT_TRUE(!(ite != it));
}

TEST(string, case_iter3c) {
    ue2_literal src;
    src.push_back('A', false);
    src.push_back('B', true);
    src.push_back('C', true);

    case_iter it = caseIterateBegin(src);
    case_iter ite = caseIterateEnd();

    EXPECT_TRUE(ite != it);
    EXPECT_EQ("ABC", *it);
    ++it;
    EXPECT_TRUE(ite != it);
    EXPECT_EQ("ABc", *it);
    ++it;
    EXPECT_TRUE(ite != it);
    EXPECT_EQ("AbC", *it);
    ++it;
    EXPECT_TRUE(ite != it);
    EXPECT_EQ("Abc", *it);
    ++it;
    EXPECT_TRUE(!(ite != it));
}

TEST(string, selfOverlap) {
    EXPECT_EQ(0U, maxStringSelfOverlap("abc", 0));
    EXPECT_EQ(0U, maxStringSelfOverlap("abc", 1));
    EXPECT_EQ(0U, maxStringSelfOverlap("abcA", 0));
    EXPECT_EQ(1U, maxStringSelfOverlap("abcA", 1));
    EXPECT_EQ(3U, maxStringSelfOverlap("aaaa", 0));
    EXPECT_EQ(1U, maxStringSelfOverlap("aaAa", 0));
    EXPECT_EQ(3U, maxStringSelfOverlap("aaAa", 1));
    EXPECT_EQ(1U, maxStringSelfOverlap("aaba", 0));
    EXPECT_EQ(2U, maxStringSelfOverlap("aabaa", 0));
    EXPECT_EQ(3U, maxStringSelfOverlap("aabaab", 0));
}

TEST(string, reverse0) {
    ue2_literal empty;
    EXPECT_TRUE(empty == reverse_literal(empty));
}

TEST(string, reverse1) {
    ue2_literal fwd("0123456789", false);
    ue2_literal bwd("9876543210", false);

    EXPECT_TRUE(bwd == reverse_literal(fwd));
}

TEST(string, reverse2) {
    ue2_literal fwd;
    fwd.push_back('0', false);
    fwd.push_back('a', false);
    fwd.push_back('b', true);
    fwd.push_back('c', true);

    ue2_literal bwd;
    bwd.push_back('c', true);
    bwd.push_back('b', true);
    bwd.push_back('a', false);
    bwd.push_back('0', false);

    EXPECT_TRUE(bwd == reverse_literal(fwd));
}

TEST(string, iterator_op1) {
    ue2_literal abcdef("abcdef", false);
    ue2_literal::const_iterator b = abcdef.begin();
    ue2_literal::const_iterator e = abcdef.end();

    EXPECT_TRUE(b <  e);
    EXPECT_FALSE(b >  e);
    EXPECT_TRUE(e >  b);
    EXPECT_FALSE(e <  b);
    EXPECT_TRUE(e == e);
    EXPECT_TRUE(b == b);
    EXPECT_FALSE(e == b);
    EXPECT_TRUE(b != e);
    EXPECT_FALSE(b != b);
    EXPECT_FALSE(e != e);
}

TEST(string, iterator_op2) {
    ue2_literal empty;
    ue2_literal::const_iterator b = empty.begin();
    ue2_literal::const_iterator e = empty.end();

    EXPECT_FALSE(b >  e);
    EXPECT_FALSE(b <  e);
    EXPECT_TRUE(e == e);
    EXPECT_TRUE(b == b);
    EXPECT_FALSE(e != b);
}

TEST(string, eq) {
    ue2_literal empty1;
    ue2_literal empty2;
    ue2_literal abc_cs("abc", false);
    ue2_literal abc_nc("abc", true);
    ue2_literal abc_ucs("ABC", false);
    ue2_literal abc_unc("ABC", true);

    EXPECT_TRUE(empty1 == empty2);
    EXPECT_FALSE(empty1 != empty2);

    EXPECT_TRUE(abc_cs != abc_nc);
    EXPECT_TRUE(abc_cs != empty1);
    EXPECT_TRUE(abc_cs != abc_nc);
    EXPECT_TRUE(abc_cs != abc_ucs);
    EXPECT_TRUE(abc_cs != abc_unc);
    EXPECT_FALSE(abc_cs == abc_nc);
    EXPECT_FALSE(abc_cs == empty1);
    EXPECT_FALSE(abc_cs == abc_nc);
    EXPECT_FALSE(abc_cs == abc_ucs);
    EXPECT_FALSE(abc_cs == abc_unc);

    EXPECT_TRUE(abc_nc != abc_ucs);
    EXPECT_TRUE(abc_nc == abc_unc);
    EXPECT_FALSE(abc_nc == abc_ucs);
    EXPECT_FALSE(abc_nc != abc_unc);

    EXPECT_TRUE(abc_ucs != abc_unc);
    EXPECT_FALSE(abc_ucs == abc_unc);
}

TEST(string, contains_cs) {
    ue2_literal abc123("abc123", false);
    EXPECT_TRUE(contains(abc123, CharReach('a')));
    EXPECT_TRUE(contains(abc123, CharReach('b')));
    EXPECT_TRUE(contains(abc123, CharReach('c')));
    EXPECT_TRUE(contains(abc123, CharReach('1')));
    EXPECT_TRUE(contains(abc123, CharReach('2')));
    EXPECT_TRUE(contains(abc123, CharReach('3')));
    EXPECT_TRUE(!contains(abc123, CharReach('A')));
    EXPECT_TRUE(!contains(abc123, CharReach('B')));
    EXPECT_TRUE(!contains(abc123, CharReach('C')));
    EXPECT_TRUE(!contains(abc123, CharReach(0x11)));
    EXPECT_TRUE(!contains(abc123, CharReach(0x12)));
    EXPECT_TRUE(!contains(abc123, CharReach(0x13)));
    EXPECT_TRUE(!contains(abc123, CharReach(0)));
    EXPECT_TRUE(!contains(abc123, CharReach('X')));
    EXPECT_TRUE(!contains(abc123, CharReach('\n')));
    EXPECT_TRUE(contains(abc123, CharReach::dot()));
    EXPECT_TRUE(contains(abc123, CharReach("Aa")));
    EXPECT_TRUE(contains(abc123, CharReach("Bb")));
    EXPECT_TRUE(contains(abc123, CharReach("XYZ1")));
    EXPECT_TRUE(contains(abc123, CharReach("3~")));
    EXPECT_TRUE(!contains(abc123, CharReach("45X")));
    EXPECT_TRUE(!contains(abc123, CharReach("Bx")));
}

TEST(string, contains_nc) {
    ue2_literal abc123("abc123", true);
    EXPECT_TRUE(contains(abc123, CharReach('a')));
    EXPECT_TRUE(contains(abc123, CharReach('b')));
    EXPECT_TRUE(contains(abc123, CharReach('c')));
    EXPECT_TRUE(contains(abc123, CharReach('1')));
    EXPECT_TRUE(contains(abc123, CharReach('2')));
    EXPECT_TRUE(contains(abc123, CharReach('3')));
    EXPECT_TRUE(contains(abc123, CharReach('A')));
    EXPECT_TRUE(contains(abc123, CharReach('B')));
    EXPECT_TRUE(contains(abc123, CharReach('C')));
    EXPECT_TRUE(!contains(abc123, CharReach(0x11)));
    EXPECT_TRUE(!contains(abc123, CharReach(0x12)));
    EXPECT_TRUE(!contains(abc123, CharReach(0x13)));
    EXPECT_TRUE(!contains(abc123, CharReach(0)));
    EXPECT_TRUE(!contains(abc123, CharReach('X')));
    EXPECT_TRUE(!contains(abc123, CharReach('\n')));
    EXPECT_TRUE(contains(abc123, CharReach::dot()));
    EXPECT_TRUE(contains(abc123, CharReach("Aa")));
    EXPECT_TRUE(contains(abc123, CharReach("Bb")));
    EXPECT_TRUE(contains(abc123, CharReach("XYZ1")));
    EXPECT_TRUE(contains(abc123, CharReach("3~")));
    EXPECT_TRUE(!contains(abc123, CharReach("45X")));
    EXPECT_TRUE(contains(abc123, CharReach("Bx")));
}

TEST(string, contains_empty) {
    ue2_literal empty;
    EXPECT_TRUE(!contains(empty, CharReach('a')));
    EXPECT_TRUE(!contains(empty, CharReach::dot()));
}

TEST(string, cat) {
    ue2_literal abc("abc", false);
    ue2_literal empty;
    ue2_literal def("def", true);

    ue2_literal abcdef;
    abcdef.push_back('a', false);
    abcdef.push_back('b', false);
    abcdef.push_back('c', false);
    abcdef.push_back('D', true);
    abcdef.push_back('e', true);
    abcdef.push_back('f', true);

    EXPECT_EQ(empty, empty + empty);
    EXPECT_EQ(abc, abc + empty);
    EXPECT_EQ(abc, empty + abc);
    EXPECT_EQ(abcdef, abc + def);
}

TEST(string, erase) {
    ue2_literal abcdefghi("abcdefghi", false);

    ue2_literal abc = abcdefghi;
    abc.erase(3, 10);
    EXPECT_EQ(ue2_literal("abc", false), abc);

    ue2_literal def = abcdefghi;
    def.erase(0, 3);
    def.erase(3);
    EXPECT_EQ(ue2_literal("def", false), def);

    ue2_literal bcdefghi = abcdefghi;
    bcdefghi.erase(0, 1);
    EXPECT_EQ(ue2_literal("bcdefghi", false), bcdefghi);
}

TEST(string, erase_nc) {
    ue2_literal abcdef;
    abcdef.push_back('a', false);
    abcdef.push_back('b', false);
    abcdef.push_back('c', false);
    abcdef.push_back('D', true);
    abcdef.push_back('e', true);
    abcdef.push_back('f', true);


    ue2_literal abc = abcdef;
    abc.erase(3, 6);
    EXPECT_EQ(ue2_literal("abc", false), abc);

    ue2_literal def = abcdef;
    def.erase(0, 3);
    EXPECT_EQ(ue2_literal("def", true), def);
}
