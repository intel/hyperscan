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

#ifndef UTIL_INSERTION_ORDERED_H
#define UTIL_INSERTION_ORDERED_H

/**
 * \file
 * \brief Insertion-ordered associative containers (set, map).
 */

#include "util/operators.h"
#include "util/unordered.h"

#include <cassert>
#include <iterator>
#include <type_traits>
#include <utility>
#include <vector>

#include <boost/iterator/iterator_facade.hpp>

namespace ue2 {

namespace insertion_ordered_detail {

// Iterator facade that wraps an underlying iterator, so that we get our
// own iterator types.
template<class WrappedIter, class Value>
class iter_wrapper
    : public boost::iterator_facade<iter_wrapper<WrappedIter, Value>, Value,
                                    boost::random_access_traversal_tag> {
public:
    iter_wrapper() = default;
    explicit iter_wrapper(WrappedIter it_in) : it(std::move(it_in)) {}

    // Templated copy-constructor to allow for interoperable iterator and
    // const_iterator.
    template<class, class> friend class iter_wrapper;

    template<class OtherIter, class OtherValue>
    iter_wrapper(iter_wrapper<OtherIter, OtherValue> other,
                 typename std::enable_if<std::is_convertible<
                     OtherIter, WrappedIter>::value>::type * = nullptr)
        : it(std::move(other.it)) {}

    WrappedIter get() const { return it; }

private:
    friend class boost::iterator_core_access;

    WrappedIter it;

    void increment() { ++it; }
    void decrement() { --it; }
    void advance(size_t n) { it += n; }
    typename std::iterator_traits<WrappedIter>::difference_type
    distance_to(const iter_wrapper &other) const {
        return other.it - it;
    }
    bool equal(const iter_wrapper &other) const { return it == other.it; }
    Value &dereference() const { return *it; }
};

template<class Key, class Element>
class element_store {
    std::vector<Element> data;
    ue2_unordered_map<Key, size_t> map;

public:
    bool empty() const {
        return data.empty();
    }

    size_t size() const {
        assert(data.size() == map.size());
        return data.size();
    }

    void clear() {
        data.clear();
        map.clear();
    }

    void reserve(size_t n) {
        data.reserve(n);
        map.reserve(n);
    }

    // Iteration.

    using const_iterator =
        iter_wrapper<typename std::vector<Element>::const_iterator,
                     const Element>;
    using iterator =
        iter_wrapper<typename std::vector<Element>::iterator, Element>;

    const_iterator begin() const {
        return const_iterator(data.begin());
    }

    const_iterator end() const {
        return const_iterator(data.end());
    }

    iterator begin() {
        return iterator(data.begin());
    }

    iterator end() {
        return iterator(data.end());
    }

    // Search.

    const_iterator find(const Key &key) const {
        auto map_it = map.find(key);
        if (map_it == map.end()) {
            return end();
        }
        auto idx = map_it->second;
        assert(idx < data.size());
        return begin() + idx;
    }

    iterator find(const Key &key) {
        auto map_it = map.find(key);
        if (map_it == map.end()) {
            return end();
        }
        auto idx = map_it->second;
        assert(idx < data.size());
        return begin() + idx;
    }

    // Insert.

    std::pair<iterator, bool> insert(const Key &key, const Element &element) {
        const auto idx = data.size();
        if (map.emplace(key, idx).second) {
            data.push_back(element);
            return {begin() + idx, true};
        }
        return {end(), false};
    }

    bool operator==(const element_store &a) const {
        return data == a.data;
    }

    bool operator<(const element_store &a) const {
        return data < a.data;
    }

    void swap(element_store &a) {
        using std::swap;
        swap(data, a.data);
        swap(map, a.map);
    }
};

} // namespace insertion_ordered_detail

template<class Key, class Value>
class insertion_ordered_map
    : public totally_ordered<insertion_ordered_map<Key, Value>> {
public:
    using key_type = Key;
    using mapped_type = Value;
    using value_type = std::pair<const Key, Value>;

private:
    using store_type = insertion_ordered_detail::element_store<Key, value_type>;
    store_type store;

public:
    using const_iterator = typename store_type::const_iterator;
    using iterator = typename store_type::iterator;

    insertion_ordered_map() = default;

    template<class Iter>
    insertion_ordered_map(Iter it, Iter it_end) {
        insert(it, it_end);
    }

    explicit insertion_ordered_map(std::initializer_list<value_type> init) {
        insert(init.begin(), init.end());
    }

    const_iterator begin() const { return store.begin(); }
    const_iterator end() const { return store.end(); }
    iterator begin() { return store.begin(); }
    iterator end() { return store.end(); }

    const_iterator find(const Key &key) const {
        return store.find(key);
    }

    iterator find(const Key &key) {
        return store.find(key);
    }

    std::pair<iterator, bool> insert(const std::pair<const Key, Value> &p) {
        return store.insert(p.first, p);
    }

    template<class Iter>
    void insert(Iter it, Iter it_end) {
        for (; it != it_end; ++it) {
            insert(*it);
        }
    }

    Value &operator[](const Key &key) {
        auto it = find(key);
        if (it == end()) {
            it = insert({key, Value{}}).first;
        }
        return it->second;
    }

    const Value &at(const Key &key) const {
        return find(key)->second;
    }

    Value &at(const Key &key) {
        return find(key)->second;
    }

    bool empty() const {
        return store.empty();
    }

    size_t size() const {
        return store.size();
    }

    void clear() {
        store.clear();
    }

    void reserve(size_t n) {
        store.reserve(n);
    }

    bool operator==(const insertion_ordered_map &a) const {
        return store == a.store;
    }

    bool operator<(const insertion_ordered_map &a) const {
        return store < a.store;
    }

    void swap(insertion_ordered_map &a) {
        store.swap(a.store);
    }

    friend void swap(insertion_ordered_map &a, insertion_ordered_map &b) {
        a.swap(b);
    }
};

template<class Key>
class insertion_ordered_set
    : public totally_ordered<insertion_ordered_set<Key>> {
public:
    using key_type = Key;
    using value_type = Key;

private:
    using store_type = insertion_ordered_detail::element_store<Key, value_type>;
    store_type store;

public:
    using const_iterator = typename store_type::const_iterator;
    using iterator = typename store_type::iterator;

    insertion_ordered_set() = default;

    template<class Iter>
        insertion_ordered_set(Iter it, Iter it_end) {
        insert(it, it_end);
    }

    explicit insertion_ordered_set(std::initializer_list<value_type> init) {
        insert(init.begin(), init.end());
    }

    const_iterator begin() const { return store.begin(); }
    const_iterator end() const { return store.end(); }

    const_iterator find(const Key &key) const {
        return store.find(key);
    }

    std::pair<iterator, bool> insert(const Key &key) {
        return store.insert(key, key);
    }

    template<class Iter>
    void insert(Iter it, Iter it_end) {
        for (; it != it_end; ++it) {
            insert(*it);
        }
    }

    bool empty() const {
        return store.empty();
    }

    size_t size() const {
        return store.size();
    }

    void clear() {
        store.clear();
    }

    void reserve(size_t n) {
        store.reserve(n);
    }

    bool operator==(const insertion_ordered_set &a) const {
        return store == a.store;
    }

    bool operator<(const insertion_ordered_set &a) const {
        return store < a.store;
    }

    void swap(insertion_ordered_set &a) {
        store.swap(a.store);
    }

    friend void swap(insertion_ordered_set &a, insertion_ordered_set &b) {
        a.swap(b);
    }
};

} // namespace ue2

#endif // UTIL_INSERTION_ORDERED_H
