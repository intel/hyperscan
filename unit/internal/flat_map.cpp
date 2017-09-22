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
#include <string>
#include <utility>
#include <vector>

using namespace std;
using namespace ue2;

template <class K, class V>
std::ostream &operator<<(std::ostream &os, const flat_map<K, V> &f) {
    os << "{";
    for (auto it = begin(f); it != end(f); ++it) {
        os << "{" << it->first << ", " << it->second << "}";
        if (it != end(f)) {
            os << ", ";
        }
    }
    os << "}";
    return os;
}

template<class FlatMap, class Compare>
bool flat_map_is_sorted_cmp(const FlatMap &f, const Compare &cmp) {
    using value_type = typename FlatMap::value_type;
    return is_sorted(f.begin(), f.end(),
                     [&cmp](const value_type &a, const value_type &b) {
                         return cmp(a.first, b.first);
                     });
}

template<class FlatMap>
bool flat_map_is_sorted(const FlatMap &f) {
    return flat_map_is_sorted_cmp(f, f.key_comp());
}

TEST(flat_map, empty) {
    flat_map<u32, u32> f;
    EXPECT_TRUE(f.begin() == f.end());
    EXPECT_TRUE(f.cbegin() == f.cend());
    EXPECT_TRUE(f.rbegin() == f.rend());
    EXPECT_TRUE(f.crbegin() == f.crend());
    EXPECT_TRUE(f.empty());
    EXPECT_EQ(0, f.size());
    EXPECT_EQ(0, f.count(10));
}

TEST(flat_map, insert) {
    flat_map<u32, u32> f;

    f.insert(make_pair(1, 10));
    f.insert(make_pair(2, 20));
    EXPECT_EQ(2, f.size());
    EXPECT_TRUE(flat_map_is_sorted(f));

    EXPECT_EQ(1, f.begin()->first);
    EXPECT_EQ(10, f.begin()->second);
    EXPECT_EQ(2, f.rbegin()->first);
    EXPECT_EQ(20, f.rbegin()->second);

    EXPECT_EQ(1, f.count(1));
    EXPECT_EQ(1, f.count(2));
    EXPECT_EQ(0, f.count(3));
}

TEST(flat_map, clear) {
    flat_map<u32, u32> f = {{1, 10}, {3, 30}, {2, 20}};
    EXPECT_EQ(3, f.size());
    EXPECT_FALSE(f.empty());

    f.clear();
    EXPECT_EQ(0, f.size());
    EXPECT_TRUE(f.empty());
}

TEST(flat_map, sorted) {
    vector<pair<u32, u32>> vec = {{7, 700}, {1, 100}, {3, 300}, {4, 400}, {2, 200}};
    flat_map<u32, u32> f(vec.begin(), vec.end());
    EXPECT_EQ(vec.size(), f.size());
    EXPECT_EQ(1, f.begin()->first);
    EXPECT_EQ(7, f.rbegin()->first);
    EXPECT_TRUE(flat_map_is_sorted(f));

    // all our elements are there
    for (const auto &p : vec) {
        auto it = f.find(p.first);
        EXPECT_TRUE(it != f.end());
        EXPECT_EQ(p.first, it->first);
        EXPECT_EQ(p.second, it->second);
    }

    // these aren't there
    EXPECT_TRUE(f.find(0) == f.end());
    EXPECT_TRUE(f.find(5) == f.end());
    EXPECT_TRUE(f.find(6) == f.end());
    EXPECT_TRUE(f.find(8) == f.end());

    // at() checks
    for (const auto &p : vec) {
        EXPECT_EQ(p.second, f.at(p.first));
    }

    ASSERT_THROW(f.at(0), std::out_of_range);
    ASSERT_THROW(f.at(10), std::out_of_range);

    // operator[]
    for (const auto &p : vec) {
        EXPECT_EQ(p.second, f[p.first]);
    }
}

TEST(flat_map, dupe_keys) {
    vector<pair<u32, u32>> vec = {{7, 700},
                                  {1, 100},
                                  {3, 300},
                                  {4, 400},
                                  {2, 200},
                                  // dupes
                                  {7, 700},
                                  {2, 200},
                                  {2, 200}};
    flat_map<u32, u32> f(vec.begin(), vec.end());
    EXPECT_EQ(5, f.size());
    ASSERT_TRUE(flat_map_is_sorted(f));
}

TEST(flat_map, subscript) {
    flat_map<u32, u32> f;
    f[1] = 10;
    f[3] = 30;
    f[2] = 20;
    EXPECT_EQ(3, f.size());

    f[1] = 100;
    f[2] = 200;
    EXPECT_EQ(3, f.size());

    ASSERT_TRUE(flat_map_is_sorted(f));
}

TEST(flat_map, init_list) {
    flat_map<u32, std::string> f = {{1, "sydney"}, {2, "melbourne"}};
    EXPECT_EQ(2, f.size());
    ASSERT_TRUE(flat_map_is_sorted(f));

    f.insert({{17, "adelaide"}, {14, "perth"}, {4, "brisbane" }});
    EXPECT_EQ(5, f.size());
    ASSERT_TRUE(flat_map_is_sorted(f));

    ASSERT_EQ("sydney", f[1]);
    ASSERT_EQ("melbourne", f[2]);
    ASSERT_EQ("adelaide", f[17]);
    ASSERT_EQ("perth", f[14]);
    ASSERT_EQ("brisbane", f[4]);

    f[1] = "hobart"; // replace
    EXPECT_EQ(5, f.size());
    ASSERT_EQ("hobart", f[1]);

    f.erase(100); // not present
    EXPECT_EQ(5, f.size());

    f.erase(17);
    EXPECT_EQ(4, f.size());
    EXPECT_TRUE(f.find(17) == f.end());
}

TEST(flat_map, custom_compare) {
    flat_map<u32, u32, std::greater<u32>> f;
    ASSERT_EQ(0, f.size());
    f.insert({{1, 10}, {2, 20}, {3, 30}, {4, 40}, {5, 50}, {6, 60}, {7, 70},
              {8, 80}, {9, 90}, {10, 100}});
    ASSERT_EQ(10, f.size());
    ASSERT_EQ(10, f.begin()->first);
    ASSERT_EQ(100, f.begin()->second);
    ASSERT_EQ(1, f.rbegin()->first);
    ASSERT_EQ(10, f.rbegin()->second);

    ASSERT_TRUE(flat_map_is_sorted(f));
    ASSERT_TRUE(std::is_sorted(f.begin(), f.end(), f.value_comp()));
    ASSERT_TRUE(flat_map_is_sorted_cmp(f, std::greater<u32>()));
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

TEST(flat_map, emplace) {
    flat_map<u32, MovableOnly> f;
    ASSERT_TRUE(f.empty());

    auto rv = f.emplace(10, MovableOnly(1, string("hatstand")));
    ASSERT_NE(end(f), rv.first);
    ASSERT_EQ(1, rv.first->second.val);
    ASSERT_TRUE(rv.second);

    rv = f.emplace(30, MovableOnly(3, string("badgerbrush")));
    ASSERT_NE(end(f), rv.first);
    ASSERT_EQ(3, rv.first->second.val);
    ASSERT_TRUE(rv.second);

    rv = f.emplace(20, MovableOnly(2, string("teakettle")));
    ASSERT_NE(end(f), rv.first);
    ASSERT_EQ(2, rv.first->second.val);
    ASSERT_TRUE(rv.second);

    ASSERT_EQ(3, f.size());
    ASSERT_TRUE(std::is_sorted(begin(f), end(f)));

    rv = f.emplace(10, MovableOnly(1, string("hatstand"))); // dupe
    ASSERT_FALSE(rv.second);

    ASSERT_EQ(3, f.size());
    ASSERT_TRUE(std::is_sorted(begin(f), end(f)));
}

TEST(flat_map, swap) {
    flat_map<u32, u32> a = {{0, 1}, {1, 2}, {2, 4}, {3, 8}, {4, 16}, {5, 32}};
    flat_map<u32, u32> b = {{0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 5}, {5, 6}};

    swap(a, b);

    EXPECT_EQ(6, a.size());
    EXPECT_EQ(6, b.size());
    EXPECT_EQ(6, a.rbegin()->second);
    EXPECT_EQ(32, b.rbegin()->second);
}

TEST(flat_map, iter) {
    vector<pair<u32, u32>> vec = {{0, 1}, {1, 2}, {2, 4}, {3, 8}, {4, 16}, {5, 32}};
    flat_map<u32, u32> f(begin(vec), end(vec));
    ASSERT_EQ(vec.size(), f.size());

    ASSERT_TRUE(std::equal(f.begin(), f.end(), vec.begin()));
    ASSERT_TRUE(std::equal(f.rbegin(), f.rend(), vec.rbegin()));
}

TEST(flat_map, erase_values) {
    vector<pair<u32, u32>> vec = {{0, 1}, {1, 2}, {2, 4}, {3, 8}, {4, 16}, {5, 32}};
    flat_map<u32, u32> f(begin(vec), end(vec));
    ASSERT_EQ(vec.size(), f.size());

    for (const auto &v : vec) {
        const auto &key = v.first;
        ASSERT_TRUE(f.find(key) != f.end());
        f.erase(key);
        ASSERT_TRUE(f.find(key) == f.end());
    }

    ASSERT_TRUE(f.empty());
}

TEST(flat_map, erase_iter) {
    vector<pair<u32, u32>> vec = {{0, 1}, {1, 2}, {2, 4}, {3, 8}, {4, 16}, {5, 32}};
    flat_map<u32, u32> f(begin(vec), end(vec));
    ASSERT_EQ(vec.size(), f.size());

    for (const auto &v : vec) {
        const auto &key = v.first;
        auto it = f.find(key);
        ASSERT_TRUE(it != f.end());
        f.erase(it);
        ASSERT_TRUE(f.find(key) == f.end());
    }

    ASSERT_TRUE(f.empty());
}

TEST(flat_map, erase_iters) {
    flat_map<u32, u32> f = {{0, 1}, {1, 2}, {2, 4}, {3, 8}, {4, 16}, {5, 32},
                            {6, 64}, {7, 128}, {8, 256}, {9, 512}};
    ASSERT_EQ(10, f.size());

    auto first = f.find(3);
    ASSERT_NE(end(f), first);
    auto last = f.find(8);
    ASSERT_NE(end(f), last);

    f.erase(first, last);

    flat_map<u32, u32> exp = {{0, 1}, {1, 2}, {2, 4}, {8, 256}, {9, 512}};
    ASSERT_EQ(exp, f);
}

TEST(flat_map, erase_empty_range) {
    flat_map<u32, u32> f = {{0, 1}, {1, 2}, {2, 4}, {3, 8}, {4, 16}, {5, 32},
                            {6, 64}, {7, 128}, {8, 256}, {9, 512}, {10, 1024}};
    const auto f2 = f; // copy

    ASSERT_EQ(f, f2);

    // Erasing (it, it) should do nothing.
    for (const auto &val : f2) {
        auto it = f.find(val.first);
        f.erase(it, it);
        ASSERT_EQ(f2, f);
    }
}

TEST(flat_map, get_allocator) {
    // Not a very interesting test, but it should pass valgrind leak tests,
    // etc. Just testing the default allocator for now.
    flat_map<u32, u32> f;

    const u32 num = 10;
    pair<u32, u32> *data = f.get_allocator().allocate(num);
    for (u32 i = 0; i < num; i++) {
        data[i] = make_pair(i, num - i);
    }

    for (u32 i = 0; i < num; i++) {
        ASSERT_EQ(make_pair(i, num - i), data[i]);
    }

    f.get_allocator().deallocate(data, num);
}

TEST(flat_map, compare_ops) {
    flat_map<u32, u32> f1 = {{0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 5}, {5, 6}};
    flat_map<u32, u32> f1_copy = f1;
    flat_map<u32, u32> f2 = {{2, 1}, {4, 2}, {6, 3}, {8, 4}, {10, 5}, {12, 6}};

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

TEST(flat_map, max_size) {
    flat_map<string, string> f;
    ASSERT_LE(1ULL << 24, f.max_size());
}

template<typename FlatMap>
size_t hash_value(const FlatMap &f) {
    return std::hash<FlatMap>()(f);
}

TEST(flat_map, hash_value) {
    const vector<pair<u32, u32>> input = {
        {0, 0}, {3, 1}, {76, 2}, {132, 3}, {77, 4}, {99999, 5}, {100, 6}};
    for (size_t len = 0; len < input.size(); len++) {
        flat_map<u32, u32> f1(input.begin(), input.begin() + len);
        flat_map<u32, u32> f2(input.rbegin() + input.size() - len,
                              input.rend());
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
        f3.emplace(32767, 7);
        EXPECT_NE(hash_value(f1), hash_value(f3));
        EXPECT_NE(hash_value(f2), hash_value(f3));

        // Change a value, but not a key.
        f3 = f1;
        EXPECT_EQ(hash_value(f1), hash_value(f3));
        EXPECT_EQ(hash_value(f2), hash_value(f3));
        f3.erase(77);
        f3.emplace(77, 10);
        EXPECT_NE(hash_value(f1), hash_value(f3));
        EXPECT_NE(hash_value(f2), hash_value(f3));
    }
}
