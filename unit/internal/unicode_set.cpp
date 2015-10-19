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

#include "gtest/gtest.h"

#include "config.h"

#include "ue2common.h"
#include "util/unicode_set.h"

using namespace ue2;

TEST(unicode_set, basic_1) {
    CodePointSet cps;
    EXPECT_TRUE(cps.none());
    EXPECT_EQ(INVALID_UNICODE, cps.at(0));

    cps.set('a');

    EXPECT_FALSE(cps.none());
    EXPECT_EQ(1U, cps.count());
    EXPECT_EQ('a', cps.at(0));
    EXPECT_EQ(INVALID_UNICODE, cps.at(1));

    cps.unset('a');
    EXPECT_TRUE(cps.none());
    EXPECT_EQ(0U, cps.count());
    EXPECT_EQ(INVALID_UNICODE, cps.at(0));
}

TEST(unicode_set, range_1) {
    CodePointSet cps;
    cps.setRange('b', 'y');

    EXPECT_EQ(24U, cps.count());
    EXPECT_EQ('b', cps.at(0));
    EXPECT_EQ('y', cps.at(23));
    EXPECT_EQ(INVALID_UNICODE, cps.at(24));

    cps.setRange('B', 'Y');

    EXPECT_EQ(48U, cps.count());
    EXPECT_EQ('B', cps.at(0));
    EXPECT_EQ('Y', cps.at(23));
    EXPECT_EQ('b', cps.at(24));
    EXPECT_EQ('y', cps.at(47));
    EXPECT_EQ(INVALID_UNICODE, cps.at(48));

    CodePointSet cps2 = cps;
    cps2.unsetRange('B', 'y');
    EXPECT_TRUE(cps2.none());

    cps2 = cps;
    EXPECT_FALSE(cps2.none());
    cps2.unsetRange(0, 'y');
    EXPECT_TRUE(cps2.none());

    cps2 = cps;
    EXPECT_FALSE(cps2.none());
    cps2.unsetRange('B', MAX_UNICODE);
    EXPECT_TRUE(cps2.none());
}

TEST(unicode_set, range_2) {
    CodePointSet cps;
    cps.setRange('A', 'Z');

    EXPECT_EQ(26U, cps.count());
    EXPECT_EQ('A', cps.at(0));
    EXPECT_EQ('Z', cps.at(25));
    EXPECT_EQ(INVALID_UNICODE, cps.at(26));

    CodePointSet cps2 = cps;
    cps2.setRange('E', 'K');
    EXPECT_TRUE(cps == cps2);
    EXPECT_FALSE(cps != cps2);

    cps2.setRange('a', 'z');

    EXPECT_EQ(52U, cps2.count());
    EXPECT_EQ('A', cps2.at(0));
    EXPECT_EQ('Z', cps2.at(25));
    EXPECT_EQ('a', cps2.at(26));
    EXPECT_EQ('z', cps2.at(51));
    EXPECT_EQ(INVALID_UNICODE, cps2.at(52));

    CodePointSet cps3 = cps2;

    cps3.setRange('a', 'f');
    EXPECT_TRUE(cps3 == cps2);

    cps3.set('g');
    EXPECT_TRUE(cps3 == cps2);

    cps3.setRange('g', 'z');
    EXPECT_TRUE(cps3 == cps2);

    EXPECT_TRUE(cps3.isSubset(cps2));
    EXPECT_TRUE(cps2.isSubset(cps3));
    EXPECT_TRUE(cps3.isSubset(cps));
    EXPECT_FALSE(cps.isSubset(cps3));
}

TEST(unicode_set, range_3) {
    CodePointSet cps;
    cps.setRange(1000, 10000);
    EXPECT_EQ(9001, cps.count());
    EXPECT_EQ(1000, cps.at(0));
    EXPECT_EQ(10000, cps.at(9000));

    cps.unsetRange(2000, 3000);
    EXPECT_EQ(8000, cps.count());
    EXPECT_EQ(1000, cps.at(0));
    EXPECT_EQ(1999, cps.at(999));
    EXPECT_EQ(3001, cps.at(1000));
    EXPECT_EQ(10000, cps.at(7999));

    cps.setRange(1500, 3500);
    EXPECT_EQ(9001, cps.count());
    EXPECT_EQ(1000, cps.at(0));
    EXPECT_EQ(10000, cps.at(9000));
}

TEST(unicode_set, subset) {
    CodePointSet a;
    a.setRange(0, 99);

    CodePointSet b;
    b.setRange(100, 199);

    CodePointSet c;
    c.setRange(200, 299);

    CodePointSet ab;
    ab.setRange(0, 199);

    CodePointSet ac;
    ac.setRange(0, 99);
    ac.setRange(200, 299);

    CodePointSet bc;
    bc.setRange(100, 299);

    CodePointSet abc;
    abc.setRange(0, 299);

    CodePointSet e;

    EXPECT_TRUE(abc.isSubset(a));
    EXPECT_TRUE(abc.isSubset(b));
    EXPECT_TRUE(abc.isSubset(c));
    EXPECT_TRUE(abc.isSubset(e));
    EXPECT_TRUE(abc.isSubset(ab));
    EXPECT_TRUE(abc.isSubset(ac));
    EXPECT_TRUE(abc.isSubset(bc));
    EXPECT_TRUE(abc.isSubset(abc));

    EXPECT_TRUE(ab.isSubset(a));
    EXPECT_TRUE(ab.isSubset(b));
    EXPECT_FALSE(ab.isSubset(c));
    EXPECT_TRUE(ab.isSubset(e));
    EXPECT_TRUE(ab.isSubset(ab));
    EXPECT_FALSE(ab.isSubset(ac));
    EXPECT_FALSE(ab.isSubset(bc));
    EXPECT_FALSE(ab.isSubset(abc));

    EXPECT_TRUE(ac.isSubset(a));
    EXPECT_FALSE(ac.isSubset(b));
    EXPECT_TRUE(ac.isSubset(c));
    EXPECT_TRUE(ac.isSubset(e));
    EXPECT_FALSE(ac.isSubset(ab));
    EXPECT_TRUE(ac.isSubset(ac));
    EXPECT_FALSE(ac.isSubset(bc));
    EXPECT_FALSE(ac.isSubset(abc));

    EXPECT_FALSE(bc.isSubset(a));
    EXPECT_TRUE(bc.isSubset(b));
    EXPECT_TRUE(bc.isSubset(c));
    EXPECT_TRUE(bc.isSubset(e));
    EXPECT_FALSE(bc.isSubset(ab));
    EXPECT_FALSE(bc.isSubset(ac));
    EXPECT_TRUE(bc.isSubset(bc));
    EXPECT_FALSE(bc.isSubset(abc));

    EXPECT_TRUE(a.isSubset(a));
    EXPECT_FALSE(a.isSubset(b));
    EXPECT_FALSE(a.isSubset(c));
    EXPECT_TRUE(a.isSubset(e));
    EXPECT_FALSE(a.isSubset(ab));
    EXPECT_FALSE(a.isSubset(ac));
    EXPECT_FALSE(a.isSubset(bc));
    EXPECT_FALSE(a.isSubset(abc));

    EXPECT_FALSE(b.isSubset(a));
    EXPECT_TRUE(b.isSubset(b));
    EXPECT_FALSE(b.isSubset(c));
    EXPECT_TRUE(b.isSubset(e));
    EXPECT_FALSE(b.isSubset(ab));
    EXPECT_FALSE(b.isSubset(ac));
    EXPECT_FALSE(b.isSubset(bc));
    EXPECT_FALSE(b.isSubset(abc));

    EXPECT_FALSE(c.isSubset(a));
    EXPECT_FALSE(c.isSubset(b));
    EXPECT_TRUE(c.isSubset(c));
    EXPECT_TRUE(c.isSubset(e));
    EXPECT_FALSE(c.isSubset(ab));
    EXPECT_FALSE(c.isSubset(ac));
    EXPECT_FALSE(c.isSubset(bc));
    EXPECT_FALSE(c.isSubset(abc));

    EXPECT_FALSE(e.isSubset(a));
    EXPECT_FALSE(e.isSubset(b));
    EXPECT_FALSE(e.isSubset(c));
    EXPECT_TRUE(e.isSubset(e));
    EXPECT_FALSE(e.isSubset(ab));
    EXPECT_FALSE(e.isSubset(ac));
    EXPECT_FALSE(e.isSubset(bc));
    EXPECT_FALSE(e.isSubset(abc));
}

TEST(unicode_set, set_op) {
    CodePointSet a;
    a.setRange(0, 99);

    CodePointSet b;
    b.setRange(100, 199);

    CodePointSet c;
    c.setRange(200, 299);

    CodePointSet ab;
    ab.setRange(0, 199);

    CodePointSet ac;
    ac.setRange(0, 99);
    ac.setRange(200, 299);

    CodePointSet bc;
    bc.setRange(100, 299);

    CodePointSet abc;
    abc.setRange(0, 299);

    CodePointSet e;

    CodePointSet t = a;
    t |= a;
    EXPECT_TRUE(t == a);
    t |= e;
    EXPECT_TRUE(t == a);
    t |= b;
    EXPECT_TRUE(t == ab);
    t |= b;
    EXPECT_TRUE(t == ab);
    t |= a;
    EXPECT_TRUE(t == ab);
    t |= ac;
    EXPECT_TRUE(t == abc);
    t -= e;
    EXPECT_TRUE(t == abc);
    t -= b;
    EXPECT_TRUE(t == ac);
    t -= b;
    EXPECT_TRUE(t == ac);
    t -= bc;
    EXPECT_TRUE(t == a);
    t -= abc;
    EXPECT_TRUE(t == e);
}
