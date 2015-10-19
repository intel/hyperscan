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
#include "util/charreach.h"

using namespace ue2;

TEST(ng_charreach, init) {
    CharReach cr;

    ASSERT_EQ(0U, cr.count());
    ASSERT_TRUE(cr.none());
    ASSERT_FALSE(cr.all());
    ASSERT_EQ(256U, cr.size());
}

TEST(ng_charreach, set) {
    CharReach cr;

    ASSERT_EQ(0U, cr.count());
    ASSERT_TRUE(cr.none());
    ASSERT_FALSE(cr.all());
    cr.set('q');
    ASSERT_EQ(1U, cr.count());
    cr.setall();
    ASSERT_EQ(cr.size(), cr.count());
    ASSERT_TRUE(cr.all());
}

TEST(ng_charreach, clear) {
    CharReach cr;

    ASSERT_EQ(0U, cr.count());
    ASSERT_TRUE(cr.none());
    ASSERT_FALSE(cr.all());
    cr.set('q');
    cr.set('u');
    cr.set('a');
    cr.set('r');
    cr.set('k');
    ASSERT_EQ(5U, cr.count());
    cr.clear('r');
    ASSERT_EQ(4U, cr.count());
    ASSERT_FALSE(cr.test('r'));
    cr.setall();
    ASSERT_EQ(cr.size(), cr.count());
    ASSERT_TRUE(cr.all());
    cr.clear(0xff);
    ASSERT_FALSE(cr.all());
}

TEST(ng_charreach, copy) {
    CharReach cr;
    cr.set('a');
    cr.set('z');

    CharReach cr2(cr);

    ASSERT_EQ(cr.count(), cr2.count());
    ASSERT_TRUE(cr == cr2);
}

TEST(ng_charreach, assignment) {
    CharReach cr;
    cr.set('f');
    cr.set('l');
    cr.set('y');

    CharReach cr2;
    cr2 = cr;

    ASSERT_EQ(cr.count(), cr2.count());
    ASSERT_TRUE(cr == cr2);
}

TEST(ng_charreach, flip) {
    CharReach cr;

    ASSERT_EQ(0U, cr.count());
    ASSERT_TRUE(cr.none());
    cr.flip();
    ASSERT_EQ(cr.size(), cr.count());
    ASSERT_TRUE(cr.all());
    cr.flip();
    ASSERT_EQ(0U, cr.count());
    ASSERT_TRUE(cr.none());
    cr.flip(25);
    ASSERT_FALSE(cr.none());
    ASSERT_FALSE(cr.all());
    ASSERT_EQ(1U, cr.count());
    cr.flip();
    ASSERT_EQ(cr.size() - 1, cr.count());
}

TEST(ng_charreach, count) {
    CharReach cr;

    cr.set(1);
    cr.set(2);
    cr.set('a');
    cr.set('Z');
    cr.set('m');
    cr.set('~');
    cr.set(210);

    size_t n = cr.find_first();
    ASSERT_FALSE(n == CharReach::npos);

    unsigned int i = 0;
    while (n != CharReach::npos) {
        i++;
        n = cr.find_next(n);
    }

    ASSERT_EQ(i, cr.count());
}

TEST(ng_charreach, string) {
    CharReach cr;

    cr.set(1);
    cr.set(2);
    cr.set('a');
    cr.set('Z');
    cr.set('m');
    cr.set('~');
    cr.set(210);
    ASSERT_FALSE(cr.isAlpha());
    cr.flip(1);
    cr.flip(2);
    cr.flip('~');
    cr.flip(210);
    ASSERT_TRUE(cr.isAlpha());

    ASSERT_EQ("Zam", cr.to_string());
}

TEST(ng_charreach, alpha) {
    CharReach cr;

    ASSERT_EQ(0U, cr.count());
    ASSERT_FALSE(cr.isAlpha());
    cr.set('a');
    ASSERT_FALSE(0 == cr.count());
    ASSERT_TRUE(cr.isAlpha());
    cr.set('A');
    cr.set('b');
    cr.set('z');
    ASSERT_TRUE(cr.isAlpha());
    cr.set(1);
    ASSERT_FALSE(cr.isAlpha());
}

TEST(ng_charreach, alpha2) {
    // Test all single chars.
    for (size_t i = 0; i < 256; i++) {
        CharReach cr((unsigned char)i);
        ASSERT_EQ((i >= 'A' && i <= 'Z') || (i >= 'a' && i <= 'z'),
                  cr.isAlpha()) << "Failed for char i=" << i;
    }

    // Test some more complex cases.
    ASSERT_FALSE(CharReach::dot().isAlpha());
    ASSERT_FALSE(CharReach("0123456789").isAlpha());
    ASSERT_FALSE(CharReach("a0").isAlpha());
    ASSERT_FALSE(CharReach("abcdef0").isAlpha());
    ASSERT_FALSE(CharReach('A', 'z').isAlpha());
    ASSERT_FALSE(CharReach('A', 'Z' + 1).isAlpha());
    ASSERT_FALSE(CharReach('A' - 1, 'Z').isAlpha());
    ASSERT_FALSE(CharReach('a', 'z' + 1).isAlpha());
    ASSERT_FALSE(CharReach('a' - 1, 'z').isAlpha());

    ASSERT_TRUE(CharReach('A', 'B').isAlpha());
    ASSERT_TRUE(CharReach('A', 'F').isAlpha());
    ASSERT_TRUE(CharReach('A', 'Z').isAlpha());
    ASSERT_TRUE(CharReach('X', 'Z').isAlpha());
    ASSERT_TRUE(CharReach('a', 'b').isAlpha());
    ASSERT_TRUE(CharReach('a', 'z').isAlpha());
    ASSERT_TRUE(CharReach("ABCDEFabcdef").isAlpha());
}

TEST(ng_charreach, caseless) {
    CharReach cr;

    cr.set('a');
    ASSERT_FALSE(cr.isCaselessChar());
    cr.set('A');
    ASSERT_TRUE(cr.isCaselessChar());
    cr.set('b');
    ASSERT_FALSE(cr.isCaselessChar());
    cr.set('B');
    ASSERT_FALSE(cr.isCaselessChar());
}

TEST(ng_charreach, caseless2) {
    // Test every pair of characters.
    for (size_t i = 0; i < 256; i++) {
        ASSERT_FALSE(CharReach((unsigned char)i).isCaselessChar());
        for (size_t j = 0; j < 256; j++) {
            CharReach cr;
            cr.set(i);
            cr.set(j);

            bool upper_lower = (i >= 'A' && i <= 'Z') && j == i + 0x20;
            bool lower_upper = (i >= 'a' && i <= 'z') && i == j + 0x20;
            bool caseless_pair = upper_lower | lower_upper;

            ASSERT_EQ(caseless_pair, cr.isCaselessChar())
                << "Failed for i=" << i << ", j=" << j;
        }
    }
}

TEST(ng_charreach, bitwise) {
    CharReach cr;
    CharReach cr2;
    CharReach cr3;
    CharReach cr4;

    cr.set('a');

    cr2.set('z');


    cr3.set('a');
    cr3.set('z');

    ASSERT_TRUE(cr < cr3);

    cr4 |= cr;
    cr4 |= cr2;

    ASSERT_TRUE(cr3 == cr4);

    ASSERT_TRUE(cr3 == (cr | cr2));
    ASSERT_TRUE(cr4 == (cr | cr2));

    ASSERT_TRUE(cr == (cr & cr3));
    ASSERT_TRUE(cr2 == (cr2 & cr3));

    cr3 &= cr;

    ASSERT_FALSE(cr3.test('z'));
}


TEST(ng_charreach, bit5) {
    CharReach cr;

    ASSERT_TRUE(cr.isBit5Insensitive());
    cr.set('a');
    ASSERT_FALSE(cr.isBit5Insensitive());
    cr.set('A');
    ASSERT_TRUE(cr.isBit5Insensitive());
    cr.set('!');
    ASSERT_FALSE(cr.isBit5Insensitive());
    cr.set(1);
    ASSERT_TRUE(cr.isBit5Insensitive());
    cr.clear();
    cr.set('!');
    cr.set('A');
    ASSERT_FALSE(cr.isBit5Insensitive());
    cr.clear();
    cr.set('A');
    cr.set('b');
    ASSERT_FALSE(cr.isBit5Insensitive());
    cr.set('a');
    cr.set('B');
    ASSERT_TRUE(cr.isBit5Insensitive());
}

TEST(ng_charreach, find_last) {
    CharReach cr;
    cr.set('a');
    ASSERT_EQ(cr.find_last(), (size_t)'a');
    cr.set('b');
    ASSERT_EQ(cr.find_last(), (size_t)'b');
    cr.set(192);
    ASSERT_EQ(cr.find_last(), (size_t)192);
    cr.set(207);
    ASSERT_EQ(cr.find_last(), (size_t)207);
    cr.set(223);
    ASSERT_EQ(cr.find_last(), (size_t)223);
    cr.set(255);
    ASSERT_EQ(cr.find_last(), (size_t)255);

    cr.clear();
    ASSERT_EQ(cr.find_last(), cr.size());
    cr.set(0);
    ASSERT_EQ(cr.find_last(), (size_t)0);
    cr.set(1);
    ASSERT_EQ(cr.find_last(), (size_t)1);
}

TEST(ng_charreach, setRange) {
    // Exhaustive test: every possible contiguous range.
    for (unsigned range = 0; range < 256; range++) {
        for (unsigned from = 0; from < 256 - range; from++) {
            unsigned to = from + range;
            CharReach cr;
            cr.setRange(from, to);
            ASSERT_EQ(from, cr.find_first());
            ASSERT_EQ(to, cr.find_last());
            ASSERT_EQ(range + 1, cr.count());
        }
    }
}

TEST(ng_charreach, find_nth) {
    const size_t npos = CharReach::npos;

    // One bit cases.
    for (size_t i = 0; i < 256; i++) {
        CharReach cr((unsigned char)i);
        ASSERT_EQ(i, cr.find_nth(0));
        ASSERT_EQ(npos, cr.find_nth(1));
    }

    // All bits set.
    CharReach dot = CharReach::dot();
    for (size_t i = 0; i < 256; i++) {
        ASSERT_EQ(i, dot.find_nth(i));
    }

    // Trivial two bit cases.
    for (size_t i = 0; i < 128; i++) {
        CharReach cr;
        cr.set(i);
        cr.set(256 - i);
        ASSERT_EQ(i, cr.find_nth(0));
        ASSERT_EQ(256 - i, cr.find_nth(1));
        ASSERT_EQ(npos, cr.find_nth(3));
    }

    // More complex case.
    const std::string str("\x01\x02\x03\x05\x06\x20!#$%&./0123568:;ABCDEFMNOPUYZbcdefwxyz");
    CharReach cr(str);
    for (size_t i = 0; i < str.length(); i++) {
        ASSERT_EQ(str[i], cr.find_nth(i));
    }
    ASSERT_EQ(npos, cr.find_nth(str.length()));
}

TEST(ng_charreach, find_empty) {
    const size_t npos = CharReach::npos;
    ASSERT_EQ(npos, CharReach().find_first());
    ASSERT_EQ(npos, CharReach().find_next(npos));
}

TEST(ng_charreach, dot) {
    CharReach dot = CharReach::dot();
    ASSERT_EQ(256, dot.count());
    ASSERT_TRUE(dot.all());
    for (size_t i = 0; i < 256; i++) {
        ASSERT_TRUE(dot.test(i));
    }
}
