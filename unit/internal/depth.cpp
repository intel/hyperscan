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

#include "util/depth.h"
#include "gtest/gtest.h"

#include <unordered_set>

using namespace std;
using namespace ue2;

static UNUSED
std::ostream& operator<<(std::ostream &os, const depth &d) {
    // We reimplement depth::str, as it's only available if dump support is
    // compiled in.
    if (d.is_unreachable()) {
        os << "unr";
        return os;
    } else if (d.is_infinite()) {
        os << "inf";
        return os;
    }

    u32 val = d; // finite val
    os << val;
    return os;
}

static constexpr u32 finite_values[] = {
    0, 1, 2, 3, 4, 5, 6, 32, 1000, 1u << 20, depth::max_value()};

TEST(depth, ctor) {
    ASSERT_EQ(depth::unreachable(), depth()); // default ctor

    // also tests u32 conversion
    for (const auto &val : finite_values) {
        ASSERT_EQ(val, depth(val));
    }

    ASSERT_THROW(depth(depth::max_value() + 1), DepthOverflowError);
}

TEST(depth, convert_throw) {
    ASSERT_THROW(u32 v = depth::infinity(), DepthOverflowError);
    ASSERT_THROW(u32 v = depth::unreachable(), DepthOverflowError);
}

TEST(depth, is_finite) {
    for (const auto &val : finite_values) {
        ASSERT_TRUE(depth(val).is_finite());
    }
    ASSERT_FALSE(depth::infinity().is_finite());
    ASSERT_FALSE(depth::unreachable().is_finite());
}

TEST(depth, is_infinite) {
    for (const auto &val : finite_values) {
        ASSERT_FALSE(depth(val).is_infinite());
    }
    ASSERT_TRUE(depth::infinity().is_infinite());
    ASSERT_FALSE(depth::unreachable().is_infinite());
}

TEST(depth, is_unreachable) {
    for (const auto &val : finite_values) {
        ASSERT_FALSE(depth(val).is_unreachable());
    }
    ASSERT_FALSE(depth::infinity().is_unreachable());
    ASSERT_TRUE(depth::unreachable().is_unreachable());
}

TEST(depth, is_reachable) {
    for (const auto &val : finite_values) {
        ASSERT_TRUE(depth(val).is_reachable());
    }
    ASSERT_TRUE(depth::infinity().is_reachable());
    ASSERT_FALSE(depth::unreachable().is_reachable());
}

TEST(depth, add_finite) {
    ASSERT_EQ(depth(1), depth(0) + depth(1));
    ASSERT_EQ(depth(1), depth(1) + depth(0));
    ASSERT_EQ(depth(1), depth(0) + 1);
    ASSERT_EQ(depth(1), depth(1) + 0);
    ASSERT_EQ(depth(2000), depth(1000) + depth(1000));
    ASSERT_EQ(depth(2000), depth(1000) + 1000);
    ASSERT_EQ(depth(900), depth(1000) + s32{-100});

    // overflow must throw
    depth max_depth(depth::max_value());
    depth d;
    ASSERT_THROW(d = max_depth + depth(1), DepthOverflowError);
    ASSERT_THROW(d = max_depth + 1, DepthOverflowError);

    // underflow must throw
    ASSERT_THROW(d = depth(0) + s32{-1}, DepthOverflowError);
}

TEST(depth, add_inf) {
    // adding anything finite to inf should produce inf
    for (const auto &val : finite_values) {
        ASSERT_EQ(depth::infinity(), depth::infinity() + depth(val));
        ASSERT_EQ(depth::infinity(), depth(val) + depth::infinity());
    }

    ASSERT_EQ(depth::infinity(), depth::infinity() + depth::infinity());
}

TEST(depth, add_unr) {
    // adding anything to unr should produce unr
    for (const auto &val : finite_values) {
        ASSERT_EQ(depth::unreachable(), depth::unreachable() + depth(val));
        ASSERT_EQ(depth::unreachable(), depth(val) + depth::unreachable());
    }

    ASSERT_EQ(depth::unreachable(), depth::unreachable() + depth::infinity());
    ASSERT_EQ(depth::unreachable(), depth::unreachable() + depth::unreachable());
}

TEST(depth, sub_finite) {
    ASSERT_EQ(depth(1), depth(1) - depth(0));
    ASSERT_EQ(depth(0), depth(1) - depth(1));
    ASSERT_EQ(depth(0), depth(1000) - depth(1000));

    // underflow should throw
    ASSERT_THROW(depth(0) - depth(1), DepthOverflowError);
}

TEST(depth, sub_inf) {
    // subtracting anything from inf should produce inf
    for (const auto &val : finite_values) {
        ASSERT_EQ(depth::infinity(), depth::infinity() - depth(val));
    }

    // subtracting non-finite values must throw
    ASSERT_THROW(depth::infinity() - depth::infinity(), DepthOverflowError);
    ASSERT_THROW(depth::infinity() - depth::unreachable(), DepthOverflowError);
}

TEST(depth, sub_unr) {
    // subtracting anything from unr should produce unr
    for (const auto &val : finite_values) {
        ASSERT_EQ(depth::unreachable(), depth::unreachable() - depth(val));
    }

    // subtracting non-finite values must throw
    ASSERT_THROW(depth::unreachable() - depth::infinity(), DepthOverflowError);
    ASSERT_THROW(depth::unreachable() - depth::unreachable(), DepthOverflowError);
}

TEST(depth, operators) {
    for (const auto &val : finite_values) {
        depth d(val);

        ASSERT_TRUE(d == d);
        ASSERT_FALSE(d != d);
        ASSERT_FALSE(d < d);
        ASSERT_FALSE(d > d);
        ASSERT_TRUE(d <= d);
        ASSERT_TRUE(d >= d);

        if (val < depth::max_value()) {
            depth d1(val + 1);
            ASSERT_FALSE(d == d1);
            ASSERT_TRUE(d != d1);
            ASSERT_TRUE(d < d1);
            ASSERT_FALSE(d > d1);
            ASSERT_TRUE(d <= d1);
            ASSERT_FALSE(d >= d1);
        }

        ASSERT_TRUE(d < depth::infinity());
        ASSERT_TRUE(d < depth::unreachable());
        ASSERT_TRUE(d <= depth::infinity());
        ASSERT_TRUE(d <= depth::unreachable());
        ASSERT_TRUE(d <= depth(depth::max_value()));

        ASSERT_TRUE(depth::infinity() > d);
        ASSERT_TRUE(depth::unreachable() > d);
        ASSERT_TRUE(depth::infinity() >= d);
        ASSERT_TRUE(depth::unreachable() >= d);
        ASSERT_TRUE(depth(depth::max_value()) >= d);

        ASSERT_TRUE(d != depth::infinity());
        ASSERT_TRUE(d != depth::unreachable());
    }

    ASSERT_TRUE(depth::infinity() <= depth::infinity());
    ASSERT_TRUE(depth::infinity() >= depth::infinity());
    ASSERT_FALSE(depth::infinity() < depth::infinity());
    ASSERT_FALSE(depth::infinity() > depth::infinity());
    ASSERT_TRUE(depth::infinity() == depth::infinity());
    ASSERT_FALSE(depth::infinity() != depth::infinity());

    ASSERT_TRUE(depth::unreachable() <= depth::unreachable());
    ASSERT_TRUE(depth::unreachable() >= depth::unreachable());
    ASSERT_FALSE(depth::unreachable() < depth::unreachable());
    ASSERT_FALSE(depth::unreachable() > depth::unreachable());
    ASSERT_TRUE(depth::unreachable() == depth::unreachable());
    ASSERT_FALSE(depth::unreachable() != depth::unreachable());
}

TEST(depth, u64a_operators) {
    for (const auto &fval : finite_values) {
        u64a val = fval;
        depth d(fval);

        // trivial tests against own value
        ASSERT_TRUE(d == val);
        ASSERT_FALSE(d != val);
        ASSERT_FALSE(d < val);
        ASSERT_FALSE(d > val);
        ASSERT_TRUE(d <= val);
        ASSERT_TRUE(d >= val);

        // tests against val + 1
        ASSERT_FALSE(d == val + 1);
        ASSERT_TRUE(d != val + 1);
        ASSERT_TRUE(d < val + 1);
        ASSERT_FALSE(d > val + 1);
        ASSERT_TRUE(d <= val + 1);
        ASSERT_FALSE(d >= val + 1);

        // test inf against val
        ASSERT_FALSE(depth::infinity() == val);
        ASSERT_TRUE(depth::infinity() != val);
        ASSERT_FALSE(depth::infinity() < val);
        ASSERT_TRUE(depth::infinity() > val);
        ASSERT_FALSE(depth::infinity() <= val);
        ASSERT_TRUE(depth::infinity() >= val);

        // test unreachable against val
        ASSERT_FALSE(depth::unreachable() == val);
        ASSERT_TRUE(depth::unreachable() != val);
        ASSERT_FALSE(depth::unreachable() < val);
        ASSERT_TRUE(depth::unreachable() > val);
        ASSERT_FALSE(depth::unreachable() <= val);
        ASSERT_TRUE(depth::unreachable() >= val);
    }
}

TEST(depth, unordered_set) {
    unordered_set<depth> depths;

    for (const auto &val : finite_values) {
        depths.emplace(val);
    }

    for (const auto &val : finite_values) {
        ASSERT_TRUE(depths.find(depth(val)) != depths.end());
    }

    ASSERT_TRUE(depths.find(depth::infinity()) == depths.end());
    ASSERT_TRUE(depths.find(depth::unreachable()) == depths.end());
    ASSERT_TRUE(depths.find(depth(1337)) == depths.end());
}
