/*
 * Copyright (c) 2017, Intel Corporation
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

#include "ue2common.h"
#include "util/insertion_ordered.h"

#include "gtest/gtest.h"

using namespace std;
using namespace ue2;

template <class K, class V>
std::ostream &operator<<(std::ostream &os,
                         const insertion_ordered_map<K, V> &m) {
    os << "{";
    for (auto it = begin(m); it != end(m); ++it) {
        os << "{" << it->first << ", " << it->second << "}";
        if (it != end(m)) {
            os << ", ";
        }
    }
    os << "}";
    return os;
}

TEST(insertion_ordered_map, empty) {
    insertion_ordered_map<u32, u32> m;
    EXPECT_TRUE(m.empty());
    EXPECT_TRUE(m.begin() == m.end());
    EXPECT_EQ(0, m.size());

    m.insert({10, 10});
    EXPECT_FALSE(m.empty());
    EXPECT_EQ(1, m.size());

    m.clear();
    EXPECT_TRUE(m.empty());
    EXPECT_TRUE(m.begin() == m.end());
    EXPECT_EQ(0, m.size());
}

TEST(insertion_ordered_map, insert) {
    const vector<pair<u32, u32>> v = {{7, 1},  {1, 2},  {3, 4},
                                      {10, 5}, {99, 6}, {12, 7}};
    insertion_ordered_map<u32, u32> m;
    for (const auto &e : v) {
        m.insert(e);
    }

    EXPECT_FALSE(m.empty());
    EXPECT_EQ(v.size(), m.size());
    vector<pair<u32, u32>> v2(m.begin(), m.end());
    EXPECT_EQ(v, v2);
}

TEST(insertion_ordered_map, insert_iter) {
    const vector<pair<u32, u32>> v = {{7, 1},  {1, 2},  {3, 4},
                                      {10, 5}, {99, 6}, {12, 7}};
    insertion_ordered_map<u32, u32> m;
    m.insert(v.begin(), v.end());

    EXPECT_FALSE(m.empty());
    EXPECT_EQ(v.size(), m.size());
    vector<pair<u32, u32>> v2(m.begin(), m.end());
    EXPECT_EQ(v, v2);
}

TEST(insertion_ordered_map, find_const) {
    const vector<pair<u32, u32>> v = {{7, 1},  {1, 2},  {3, 4},
                                      {10, 5}, {99, 6}, {12, 7}};
    const insertion_ordered_map<u32, u32> m(v.begin(), v.end());

    for (const auto &e : v) {
        auto it = m.find(e.first);
        ASSERT_NE(m.end(), it);
        EXPECT_EQ(e.first, it->first);
        EXPECT_EQ(e.second, it->second);
    }
}

TEST(insertion_ordered_map, find_mutable) {
    const vector<pair<u32, u32>> v = {{7, 1},  {1, 2},  {3, 4},
                                      {10, 5}, {99, 6}, {12, 7}};
    insertion_ordered_map<u32, u32> m(v.begin(), v.end());

    for (const auto &e : v) {
        auto it = m.find(e.first);
        ASSERT_NE(m.end(), it);
        EXPECT_EQ(e.first, it->first);
        EXPECT_EQ(e.second, it->second);
        auto &mut = it->second;
        ++mut;
        EXPECT_EQ(e.second + 1, m.at(e.first));
    }
}

TEST(insertion_ordered_map, operator_brackets) {
    insertion_ordered_map<u32, u32> m;

    u32 val = 1000;
    for (u32 i = 10; i > 0; i--) {
        m[i] = val++;
    }

    EXPECT_EQ(10, m.size());

    val = 1000;
    auto it = m.begin();
    for (u32 i = 10; i > 0; i--) {
        ASSERT_NE(m.end(), it);
        EXPECT_EQ(i, it->first);
        EXPECT_EQ(val, it->second);
        ++val;
        ++it;
    }

    ASSERT_EQ(m.end(), it);
}

template <class K>
std::ostream &operator<<(std::ostream &os, const insertion_ordered_set<K> &s) {
    os << "{";
    for (auto it = begin(s); it != end(s); ++it) {
        os << *it;
        if (it != end(s)) {
            os << ", ";
        }
    }
    os << "}";
    return os;
}

TEST(insertion_ordered_set, empty) {
    insertion_ordered_set<u32> m;
    EXPECT_TRUE(m.empty());
    EXPECT_TRUE(m.begin() == m.end());
    EXPECT_EQ(0, m.size());

    m.insert(10);
    EXPECT_FALSE(m.empty());
    EXPECT_EQ(1, m.size());

    m.clear();
    EXPECT_TRUE(m.empty());
    EXPECT_TRUE(m.begin() == m.end());
    EXPECT_EQ(0, m.size());
}

TEST(insertion_ordered_set, insert) {
    const vector<u32> v = {7, 1, 3, 10, 99, 12};
    insertion_ordered_set<u32> s;
    for (const auto &e : v) {
        s.insert(e);
    }

    EXPECT_FALSE(s.empty());
    EXPECT_EQ(v.size(), s.size());
    vector<u32> v2(s.begin(), s.end());
    EXPECT_EQ(v, v2);
}

TEST(insertion_ordered_set, insert_iter) {
    const vector<u32> v = {7, 1, 3, 10, 99, 12};
    insertion_ordered_set<u32> s;
    s.insert(v.begin(), v.end());

    EXPECT_FALSE(s.empty());
    EXPECT_EQ(v.size(), s.size());
    vector<u32> v2(s.begin(), s.end());
    EXPECT_EQ(v, v2);
}

TEST(insertion_ordered_set, find_const) {
    const vector<u32> v = {7, 1, 3, 10, 99, 12};
    const insertion_ordered_set<u32> s(v.begin(), v.end());

    for (const auto &e : v) {
        auto it = s.find(e);
        ASSERT_NE(s.end(), it);
        EXPECT_EQ(e, *it);
    }
}
