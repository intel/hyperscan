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

#include "config.h"

#include "gtest/gtest.h"
#include "util/bitfield.h"

#include <algorithm>
#include <unordered_set>

using namespace std;
using namespace ue2;

template<size_t N>
std::ostream& operator<<(std::ostream &os, const bitfield<N> &bf) {
    const size_t npos = bitfield<N>::npos;

    os << "{";

    size_t i = bf.find_first();
    while (i != npos) {
        os << i;
        i = bf.find_next(i);
        if (i != npos) {
            os << ", ";
        }
    }
    os << "}";

    return os;
}

template <typename T> class BitfieldTest : public testing::Test {};

typedef ::testing::Types<
    bitfield<1>,
    bitfield<8>,
    bitfield<32>,
    bitfield<64>,
    bitfield<100>,
    bitfield<127>,
    bitfield<129>,
    bitfield<300>,
    bitfield<512>,
    bitfield<2048>,
    bitfield<10000>
                    > BitfieldTypes;

TYPED_TEST_CASE(BitfieldTest, BitfieldTypes);

TYPED_TEST(BitfieldTest, empty) {
    TypeParam a;
    EXPECT_TRUE(a.none());
    EXPECT_EQ(0, a.count());
}

TYPED_TEST(BitfieldTest, set1) {
    TypeParam a;
    const size_t size = a.size();
    const size_t npos = TypeParam::npos;
    for (size_t i = 0; i < size; i++) {
        a.clear();
        EXPECT_TRUE(a.none());

        a.set(i);
        EXPECT_FALSE(a.none());
        EXPECT_TRUE(a.test(i));
        EXPECT_EQ(i, a.find_first());
        EXPECT_EQ(npos, a.find_next(i));
        EXPECT_EQ(1, a.count());

        a.clear(i);
        EXPECT_TRUE(a.none());
        EXPECT_FALSE(a.test(i));
        EXPECT_EQ(npos, a.find_first());
        EXPECT_EQ(0, a.count());
    }
}

// Tests set(), find_first(), find_next().
TYPED_TEST(BitfieldTest, setn) {
    const size_t size = TypeParam().size();

    const size_t strides[] = { 600, 256, 80, 17, 7, 3 };
    for (size_t s = 0; s < ARRAY_LENGTH(strides); s++) {
        const size_t step = strides[s];
        TypeParam a;
        size_t num = 0;
        for (size_t i = 0; i < size; i += step, num++) {
            a.set(i);
        }
        EXPECT_EQ(num, a.count());
        ASSERT_EQ(0, a.find_first());

        size_t count = 1;
        for (size_t i = a.find_next(0); i != TypeParam::npos;
             i = a.find_next(i), count++) {
            ASSERT_EQ(count * step, i);
        }
        ASSERT_EQ(num, count);
    }
}

TYPED_TEST(BitfieldTest, allbits) {
    TypeParam a;
    a.setall();
    EXPECT_FALSE(a.none());
    EXPECT_TRUE(a.all());
    EXPECT_EQ(a.size(), a.count());

    a.flip();
    EXPECT_TRUE(a.none());
    EXPECT_FALSE(a.all());
    EXPECT_EQ(0, a.count());
}

TYPED_TEST(BitfieldTest, allbits2) {
    TypeParam a;
    a.set(); // alias for setall
    EXPECT_FALSE(a.none());
    EXPECT_TRUE(a.all());
    EXPECT_EQ(a.size(), a.count());

    a.flip();
    EXPECT_TRUE(a.none());
    EXPECT_FALSE(a.all());
    EXPECT_EQ(0, a.count());
}

TYPED_TEST(BitfieldTest, flipn) {
    TypeParam a;
    a.setall();
    EXPECT_FALSE(a.none());
    EXPECT_TRUE(a.all());
    EXPECT_EQ(a.size(), a.count());

    const size_t size = a.size();
    for (size_t i = 0; i < size; i++) {
        a.flip(i);
        EXPECT_FALSE(a.test(i));
        EXPECT_EQ(size - i - 1, a.count());
    }

    EXPECT_TRUE(a.none());
}

TYPED_TEST(BitfieldTest, flip_on) {
    TypeParam a;
    EXPECT_TRUE(a.none());

    const size_t size = a.size();
    for (size_t i = 0; i < size; i++) {
        a.flip(i);
        EXPECT_TRUE(a.test(i));
        EXPECT_EQ(i + 1, a.count());
    }

    EXPECT_TRUE(a.all());
}

TYPED_TEST(BitfieldTest, equality_copy) {
    TypeParam a, b;
    const size_t size = a.size();

    size_t step = (size + 6) / 7;
    ASSERT_NE(0, step);
    for (size_t i = 0; i < size; i += step) {
        a.set(i);
        b.set(i);
    }

    EXPECT_EQ(a, b);
    EXPECT_EQ(a.hash(), b.hash());

    TypeParam c(a); // copy
    EXPECT_EQ(a, c);
    EXPECT_EQ(b, c);
    EXPECT_EQ(c.hash(), a.hash());
    EXPECT_EQ(c.hash(), b.hash());

    c.clear(c.find_first());
    EXPECT_EQ(a.count() - 1, c.count());
    EXPECT_NE(a, c);
}

TYPED_TEST(BitfieldTest, trivial_operations) {
    const TypeParam a = ~TypeParam(); // all ones
    const TypeParam b; // all zeroes
    ASSERT_TRUE(a.all());
    ASSERT_TRUE(b.none());

    EXPECT_EQ(a, ~b);
    EXPECT_EQ(b, ~a);

    EXPECT_EQ(a, a | b);
    EXPECT_EQ(b, a & b);

    TypeParam c;
    c = a; // assignment
    EXPECT_EQ(c, a);
    c &= b;
    EXPECT_EQ(b, c); // all zeroes
    c |= a;
    EXPECT_EQ(a, c); // all ones
    c = a ^ b;
    EXPECT_EQ(a, c);
}

TYPED_TEST(BitfieldTest, even_odd) {
    TypeParam even, odd;
    const size_t size = even.size();

    for (size_t i = 0; i < size; i++) {
        if (i % 2) {
            odd.set(i);
        } else {
            even.set(i);
        }
    }

    for (size_t i = 0; i < size; i++) {
        if (i % 2) {
            EXPECT_TRUE(odd.test(i)) << "odd should contain " << i;
            EXPECT_FALSE(even.test(i)) << "even should not contain " << i;
        } else {
            EXPECT_TRUE(even.test(i)) << "even should contain " << i;
            EXPECT_FALSE(odd.test(i)) << "odd should not contain " << i;
        }
    }

    EXPECT_TRUE(even != odd);
    EXPECT_FALSE(even == odd);

    EXPECT_TRUE((even | odd).all());
    EXPECT_TRUE((even ^ odd).all());
    EXPECT_TRUE((even & odd).none());
}

TYPED_TEST(BitfieldTest, operator_lt) {
    const TypeParam a = ~TypeParam(); // all ones
    const TypeParam b; // all zeroes
    ASSERT_TRUE(a.all());
    ASSERT_TRUE(b.none());

    // Just some trivial tests; we have no particular ordering requirements for
    // bitfield, so long as an ordering exists.

    EXPECT_FALSE(a < a);
    EXPECT_FALSE(b < b);
    EXPECT_TRUE(b < a);
}

TYPED_TEST(BitfieldTest, find_first) {
    TypeParam a;
    a.setall();
    ASSERT_TRUE(a.all());

    const size_t size = a.size();
    for (size_t i = 0; i < size; ++i) {
        ASSERT_EQ(i, a.find_first());
        a.clear(i);
    }

    ASSERT_TRUE(a.none());
    const size_t npos = TypeParam::npos;
    ASSERT_EQ(npos, a.find_first());
}

TYPED_TEST(BitfieldTest, find_last) {
    TypeParam a;
    a.setall();
    ASSERT_TRUE(a.all());

    const size_t size = a.size();
    for (size_t i = size; i > 0; i--) {
        ASSERT_EQ(i - 1, a.find_last());
        a.clear(i - 1);
    }

    ASSERT_TRUE(a.none());
    const size_t npos = TypeParam::npos;
    ASSERT_EQ(npos, a.find_last());
}

TYPED_TEST(BitfieldTest, find_next_all) {
    TypeParam a;
    a.setall();
    ASSERT_TRUE(a.all());
    ASSERT_EQ(0, a.find_first());

    const size_t size = a.size();
    for (size_t i = 1; i < size; ++i) {
        ASSERT_EQ(i, a.find_next(i - 1));
    }

    const size_t npos = TypeParam::npos;
    ASSERT_EQ(npos, a.find_next(size - 1));
}

TYPED_TEST(BitfieldTest, find_next_npos) {
    TypeParam a;
    const size_t size = a.size();
    const size_t npos = TypeParam::npos;
    for (size_t i = 0; i < size; ++i) {
        a.clear();
        a.set(i);
        ASSERT_EQ(i, a.find_first());
        ASSERT_EQ(npos, a.find_next(i));
    }
}

TYPED_TEST(BitfieldTest, find_next_last) {
    TypeParam a;
    const size_t size = a.size();
    const size_t last = size - 1;
    for (size_t i = 0; i < last; ++i) {
        a.clear();
        a.set(i);
        a.set(last);
        ASSERT_EQ(i, a.find_first());
        ASSERT_EQ(last, a.find_next(i));
    }
}

TYPED_TEST(BitfieldTest, find_nth_one) {
    TypeParam a;
    const size_t size = a.size();
    const size_t npos = TypeParam::npos;
    for (size_t i = 0; i < size; ++i) {
        a.clear();
        a.set(i);
        ASSERT_EQ(i, a.find_nth(0));
        if (1 < size) {
            ASSERT_EQ(npos, a.find_nth(1));
        }
    }
}

TYPED_TEST(BitfieldTest, find_nth_all) {
    TypeParam a;
    const size_t size = a.size();

    a.setall();

    for (size_t i = 0; i < size; ++i) {
        ASSERT_EQ(i, a.find_nth(i));
    }
}

TYPED_TEST(BitfieldTest, find_nth_sparse) {
    TypeParam a;
    const size_t size = a.size();
    const size_t stride = std::max(size / 31, size_t{3});

    std::vector<size_t> bits;
    for (size_t i = 0; i < size; i += stride) {
        a.set(i);
        bits.push_back(i);
    }

    ASSERT_EQ(bits.size(), a.count());

    for (size_t n = 0; n < bits.size(); n++) {
        ASSERT_EQ(bits[n], a.find_nth(n));
    }
}

TYPED_TEST(BitfieldTest, unordered_set) {
    const size_t size = TypeParam::size();

    // Exercise the hash specialisation by adding bitfields to an
    // unordered_set.
    unordered_set<TypeParam> s;
    s.reserve(size);

    for (size_t i = 0; i < size; ++i) {
        TypeParam a;
        a.set(i);
        s.insert(a);
    }

    ASSERT_EQ(size, s.size());

    for (size_t i = 0; i < size; ++i) {
        TypeParam a;
        a.set(i);
        ASSERT_TRUE(s.find(a) != s.end());
    }
}

TYPED_TEST(BitfieldTest, set_range_one) {
    TypeParam a;
    const size_t size = a.size();
    for (size_t i = 0; i < size; ++i) {
        a.clear();
        a.set_range(i, i);

        TypeParam b;
        b.set(i);

        ASSERT_EQ(b, a);
    }
}

TYPED_TEST(BitfieldTest, set_range_all) {
    TypeParam a;
    const size_t size = a.size();
    a.set_range(0, size - 1);

    TypeParam b;
    b.setall();

    ASSERT_EQ(b, a);
}

TYPED_TEST(BitfieldTest, set_range_part) {
    TypeParam a;
    const size_t size = a.size();
    const size_t part = size / 3;
    if (part < 1) {
        return;
    }

    for (size_t i = 0; i < size - part; ++i) {
        SCOPED_TRACE(i);
        a.clear();
        a.set_range(i, i + part);

        for (size_t j = i; j <= i + part; j++) {
            ASSERT_TRUE(a.test(j)) << "bit " << j << " should be on!";
        }

        // only the set bits should be on.
        ASSERT_EQ(part + 1, a.count());
    }
}
