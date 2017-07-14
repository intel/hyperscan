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

#include "util/flat_containers.h"
#include "ue2common.h"

#include "gtest/gtest.h"

#include <algorithm>
#include <iterator>
#include <string>
#include <vector>

using std::string;
using std::tie;
using std::vector;
using namespace ue2;

template <class T>
std::ostream &operator<<(std::ostream &os, const flat_set<T> &f) {
    os << "{";
    for (auto it = begin(f); it != end(f); ++it) {
        os << *it;
        if (it != end(f)) {
            os << ", ";
        }
    }
    os << "}";
    return os;
}

TEST(flat_set, empty) {
    flat_set<u32> f;
    EXPECT_TRUE(f.begin() == f.end());
    EXPECT_TRUE(f.cbegin() == f.cend());
    EXPECT_TRUE(f.rbegin() == f.rend());
    EXPECT_TRUE(f.crbegin() == f.crend());
    EXPECT_TRUE(f.empty());
    EXPECT_EQ(0, f.size());
    EXPECT_EQ(0, f.count(10));
}

TEST(flat_set, clear) {
    flat_set<u32> f = {1000, 2000, 3000};
    EXPECT_EQ(3, f.size());
    EXPECT_FALSE(f.empty());

    f.clear();
    EXPECT_EQ(0, f.size());
    EXPECT_TRUE(f.empty());
}

TEST(flat_set, one_element) {
    flat_set<u32> f;
    EXPECT_TRUE(f.empty());
    f.insert(10);
    EXPECT_FALSE(f.empty());
    EXPECT_EQ(1, f.size());
    EXPECT_EQ(1, f.count(10));
    EXPECT_EQ(0, f.count(11));

    ASSERT_FALSE(f.begin() == f.end());
    ASSERT_FALSE(f.cbegin() == f.cend());
    ASSERT_FALSE(f.rbegin() == f.rend());
    ASSERT_FALSE(f.crbegin() == f.crend());

    EXPECT_EQ(10, *f.begin());
    EXPECT_EQ(10, *f.cbegin());
    EXPECT_EQ(10, *f.rbegin());
    EXPECT_EQ(10, *f.crbegin());

    EXPECT_TRUE(std::next(f.begin()) == f.end());
    EXPECT_TRUE(std::next(f.cbegin()) == f.cend());
    EXPECT_TRUE(std::next(f.rbegin()) == f.rend());
    EXPECT_TRUE(std::next(f.crbegin()) == f.crend());
}

TEST(flat_set, some_elements) {
    vector<u32> input = { 10, 5, 2000, 1, 300 };

    flat_set<u32> f;
    for (const auto &v : input) {
        f.insert(v);
    }

    ASSERT_FALSE(f.empty());
    ASSERT_EQ(5, f.size());

    ASSERT_EQ(1, *f.begin());
    ASSERT_EQ(2000, *f.rbegin());
    ASSERT_TRUE(std::is_sorted(f.begin(), f.end()));

    for (const auto &v : input) {
        ASSERT_TRUE(f.find(v) != f.end());
        ASSERT_EQ(v, *f.find(v));
    }

    ASSERT_TRUE(f.find(2) == f.end());
    ASSERT_TRUE(f.find(300000) == f.end());
}

TEST(flat_set, dupe_elements) {
    flat_set<u32> f;
    f.insert(10);
    f.insert(50);
    f.insert(10);
    f.insert(50);

    ASSERT_FALSE(f.empty());
    ASSERT_EQ(2, f.size());

    ASSERT_EQ(10, *f.begin());
    ASSERT_EQ(50, *f.rbegin());
}

TEST(flat_set, init_ctor) {
    flat_set<u32> f = {1000, 900, 800, 800, 700, 100, 500}; // unsorted + dupe
    ASSERT_EQ(6, f.size());
    ASSERT_TRUE(std::is_sorted(f.begin(), f.end()));
}

TEST(flat_set, insert_ilist) {
    flat_set<u32> f;
    ASSERT_EQ(0, f.size());
    f.insert({10, 30, 20});
    ASSERT_EQ(3, f.size());
    f.insert({10, 30, 20}); // dupes
    ASSERT_EQ(3, f.size());
    f.insert({100, 50});
    ASSERT_EQ(5, f.size());
    ASSERT_TRUE(std::is_sorted(f.begin(), f.end()));
}

TEST(flat_set, custom_compare) {
    flat_set<u32, std::greater<u32>> f;
    ASSERT_EQ(0, f.size());
    f.insert({1, 2, 3, 4, 5, 6, 7, 8, 9, 10});
    ASSERT_EQ(10, f.size());
    ASSERT_EQ(10, *f.begin());
    ASSERT_EQ(1, *f.rbegin());

    ASSERT_TRUE(std::is_sorted(f.begin(), f.end(), f.key_comp()));
    ASSERT_TRUE(std::is_sorted(f.begin(), f.end(), f.value_comp()));
    ASSERT_TRUE(std::is_sorted(f.begin(), f.end(), std::greater<u32>()));
}

TEST(flat_set, erase_values) {
    vector<u32> input = { 10, 5, 2000, 1, 300 };
    flat_set<u32> f(input.begin(), input.end());
    ASSERT_EQ(input.size(), f.size());

    for (const auto &v : input) {
        ASSERT_TRUE(f.find(v) != f.end());
        f.erase(v);
        ASSERT_TRUE(f.find(v) == f.end());
    }

    ASSERT_TRUE(f.empty());
}

TEST(flat_set, erase_iter) {
    vector<u32> input = { 10, 5, 2000, 1, 300 };
    flat_set<u32> f(input.begin(), input.end());
    ASSERT_EQ(input.size(), f.size());

    for (const auto &v : input) {
        auto it = f.find(v);
        ASSERT_TRUE(it != f.end());
        f.erase(it);
        ASSERT_TRUE(f.find(v) == f.end());
    }

    ASSERT_TRUE(f.empty());
}

TEST(flat_set, erase_iters) {
    flat_set<u32> f = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    ASSERT_EQ(10, f.size());

    auto first = f.find(3);
    ASSERT_NE(end(f), first);
    auto last = f.find(8);
    ASSERT_NE(end(f), last);

    f.erase(first, last);

    ASSERT_EQ(5, f.size());
    ASSERT_EQ(flat_set<u32>({1, 2, 8, 9, 10}), f);
}

TEST(flat_set, erase_empty_range) {
    flat_set<u32> f = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    const flat_set<u32> f2 = f; // copy

    ASSERT_EQ(f, f2);

    // Erasing (it, it) should do nothing.
    for (const auto &val : f2) {
        auto it = f.find(val);
        f.erase(it, it);
        ASSERT_EQ(f2, f);
    }
}

namespace {
class MovableOnly {
public:
    MovableOnly(size_t val_in, const string &name_in)
        : val(val_in), name(name_in) {}

    bool operator<(const MovableOnly &other) const {
        return tie(val, name) < tie(other.val, other.name);
    }

    // Can't copy-construct or copy-assign.
    MovableOnly(const MovableOnly &) = delete;
    MovableOnly &operator=(const MovableOnly &) = delete;

    // Moves are OK though.
    MovableOnly(MovableOnly &&) = default;
    MovableOnly &operator=(MovableOnly &&) = default;

    size_t val;
    string name;
};
} // namespace

TEST(flat_set, emplace) {
    flat_set<MovableOnly> f;
    ASSERT_TRUE(f.empty());

    auto rv = f.emplace(10, string("hatstand"));
    ASSERT_NE(end(f), rv.first);
    ASSERT_EQ(10, rv.first->val);
    ASSERT_TRUE(rv.second);

    rv = f.emplace(30, string("badgerbrush"));
    ASSERT_NE(end(f), rv.first);
    ASSERT_EQ(30, rv.first->val);
    ASSERT_TRUE(rv.second);

    rv = f.emplace(20, string("teakettle"));
    ASSERT_NE(end(f), rv.first);
    ASSERT_EQ(20, rv.first->val);
    ASSERT_TRUE(rv.second);

    ASSERT_EQ(3, f.size());
    ASSERT_TRUE(std::is_sorted(begin(f), end(f)));

    rv = f.emplace(10, string("hatstand")); // dupe
    ASSERT_FALSE(rv.second);

    ASSERT_EQ(3, f.size());
    ASSERT_TRUE(std::is_sorted(begin(f), end(f)));
}

TEST(flat_set, swap) {
    flat_set<u32> a = {1, 2, 4, 8, 16, 32, 64};
    flat_set<u32> b = {1, 2, 3, 4, 5, 6, 7};

    swap(a, b);

    EXPECT_EQ(7, a.size());
    EXPECT_EQ(7, b.size());
    EXPECT_EQ(7, *a.rbegin());
    EXPECT_EQ(64, *b.rbegin());
}

TEST(flat_set, iter) {
    const vector<u32> vec = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10}; // sorted
    flat_set<u32> f(begin(vec), end(vec));
    ASSERT_EQ(vec.size(), f.size());

    ASSERT_TRUE(std::equal(f.begin(), f.end(), vec.begin()));
    ASSERT_TRUE(std::equal(f.rbegin(), f.rend(), vec.rbegin()));
}

TEST(flat_set, iter_interop) {
    const vector<u32> vec = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10}; // sorted
    flat_set<u32> f(begin(vec), end(vec));
    ASSERT_EQ(vec.size(), f.size());

    const auto &cf = f; // const reference

    // Forward

    flat_set<u32>::iterator mutable_begin = f.begin();
    ASSERT_EQ(f.cbegin(), mutable_begin);
    ASSERT_EQ(cf.begin(), mutable_begin);

    flat_set<u32>::iterator mutable_end = f.end();
    ASSERT_EQ(f.cend(), mutable_end);
    ASSERT_EQ(cf.end(), mutable_end);

    ASSERT_EQ(f.size(), std::distance(mutable_begin, mutable_end));
    ASSERT_EQ(f.size(), mutable_end - mutable_begin);
    ASSERT_EQ(f.size(), mutable_end - f.cbegin());
    ASSERT_EQ(f.size(), f.cend() - mutable_begin);

    // Reverse

    flat_set<u32>::reverse_iterator mutable_rbegin = f.rbegin();
    ASSERT_EQ(f.crbegin(), mutable_rbegin);
    ASSERT_EQ(cf.rbegin(), mutable_rbegin);

    flat_set<u32>::reverse_iterator mutable_rend = f.rend();
    ASSERT_EQ(f.crend(), mutable_rend);
    ASSERT_EQ(cf.rend(), mutable_rend);

    ASSERT_EQ(f.size(), std::distance(mutable_rbegin, mutable_rend));
    ASSERT_EQ(f.size(), mutable_rend - mutable_rbegin);
    ASSERT_EQ(f.size(), mutable_rend - f.crbegin());
    ASSERT_EQ(f.size(), f.crend() - mutable_rbegin);
}

TEST(flat_set, compare_ops) {
    flat_set<u32> f1 = {1, 2, 3, 4, 5};
    flat_set<u32> f1_copy = f1;
    flat_set<u32> f2 = {2, 4, 6, 8, 10};

    EXPECT_TRUE(f1 == f1);
    EXPECT_TRUE(f1 == f1_copy);
    EXPECT_FALSE(f1 == f2);

    EXPECT_FALSE(f1 != f1);
    EXPECT_FALSE(f1 != f1_copy);
    EXPECT_TRUE(f1 != f2);

    EXPECT_FALSE(f1 < f1);
    EXPECT_FALSE(f1 < f1_copy);
    EXPECT_TRUE(f1 < f2);

    EXPECT_TRUE(f1 <= f1);
    EXPECT_TRUE(f1 <= f1_copy);
    EXPECT_TRUE(f1 <= f2);

    EXPECT_FALSE(f1 > f1);
    EXPECT_FALSE(f1 > f1_copy);
    EXPECT_FALSE(f1 > f2);

    EXPECT_TRUE(f1 >= f1);
    EXPECT_TRUE(f1 >= f1_copy);
    EXPECT_FALSE(f1 >= f2);
}

TEST(flat_set, get_allocator) {
    // Not a very interesting test, but it should pass valgrind leak tests,
    // etc. Just testing the default allocator for now.
    flat_set<u32> f;

    const u32 num = 10;
    u32 *data = f.get_allocator().allocate(num);
    for (u32 i = 0; i < num; i++) {
        data[i] = i;
    }

    for (u32 i = 0; i < num; i++) {
        ASSERT_EQ(i, data[i]);
    }

    f.get_allocator().deallocate(data, num);
}

TEST(flat_set, max_size) {
    flat_set<string> f;
    ASSERT_LE(1ULL << 24, f.max_size());
}

template<typename FlatSet>
size_t hash_value(const FlatSet &f) {
    return std::hash<FlatSet>()(f);
}

TEST(flat_set, hash_value) {
    const vector<u32> input = {0,        15, 3,   1,   20,  32768,
                               24000000, 17, 100, 101, 104, 99999};
    for (size_t len = 0; len < input.size(); len++) {
        flat_set<u32> f1(input.begin(), input.begin() + len);
        flat_set<u32> f2(input.rbegin() + input.size() - len, input.rend());
        EXPECT_EQ(hash_value(f1), hash_value(f2));

        // Try removing an element.
        auto f3 = f1;
        EXPECT_EQ(hash_value(f1), hash_value(f3));
        EXPECT_EQ(hash_value(f2), hash_value(f3));
        if (!f3.empty()) {
            f3.erase(f3.begin());
            EXPECT_NE(hash_value(f1), hash_value(f3));
            EXPECT_NE(hash_value(f2), hash_value(f3));
        }

        // Try adding an element.
        f3 = f1;
        EXPECT_EQ(hash_value(f1), hash_value(f3));
        EXPECT_EQ(hash_value(f2), hash_value(f3));
        f3.insert(32767);
        EXPECT_NE(hash_value(f1), hash_value(f3));
        EXPECT_NE(hash_value(f2), hash_value(f3));
    }
}
