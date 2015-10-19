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

#ifndef UTIL_UE2_CONTAINERS_H_
#define UTIL_UE2_CONTAINERS_H_

#include "ue2common.h"

#include <algorithm>
#include <iterator>
#include <type_traits>
#include <utility>

#include <boost/iterator/iterator_facade.hpp>
#include <boost/unordered/unordered_map.hpp>
#include <boost/unordered/unordered_set.hpp>

namespace ue2 {

/** \brief Unordered set container implemented internally as a hash table. */
using boost::unordered_set;

/** \brief Unordered map container implemented internally as a hash table. */
using boost::unordered_map;

namespace flat_detail {

// Iterator facade that wraps an underlying iterator, so that we get our
// own iterator types.
template <class WrappedIter, class Value>
class iter_wrapper
    : public boost::iterator_facade<iter_wrapper<WrappedIter, Value>, Value,
                                    boost::random_access_traversal_tag> {
public:
    iter_wrapper() {}
    explicit iter_wrapper(const WrappedIter &it_in) : it(it_in) {}

    // Templated copy-constructor to allow for interoperable iterator and
    // const_iterator.
private:
    template <class, class> friend class iter_wrapper;

public:
    template <class OtherIter, class OtherValue>
    iter_wrapper(const iter_wrapper<OtherIter, OtherValue> &other,
                 typename std::enable_if<std::is_convertible<
                     OtherIter, WrappedIter>::value>::type * = nullptr)
        : it(other.it) {}

    WrappedIter get() const { return it; }

private:
    friend class boost::iterator_core_access;

    WrappedIter it;

    void increment() { ++it; }
    void decrement() { --it; }
    void advance(size_t n) { it += n; }
    typename WrappedIter::difference_type
    distance_to(const iter_wrapper &other) const {
        return other.it - it;
    }
    bool equal(const iter_wrapper &other) const { return it == other.it; }
    Value &dereference() const { return *it; }
};

} // namespace flat_detail

/**
 * \brief Set container implemented internally as a sorted vector. Use this
 * rather than std::set for small sets as it's faster, uses less memory and
 * incurs less malloc time.
 *
 * Note: we used to use boost::flat_set, but have run into problems with all
 * the extra machinery it instantiates.
 */
template <class T, class Compare = std::less<T>,
          class Allocator = std::allocator<T>>
class flat_set {
    // Underlying storage is a sorted std::vector.
    using StorageT = std::vector<T, Allocator>;

    Compare comp;
    StorageT data;

public:
    // Member types.
    using key_type = T;
    using value_type = T;
    using size_type = typename StorageT::size_type;
    using difference_type = typename StorageT::difference_type;
    using key_compare = Compare;
    using value_compare = Compare;
    using allocator_type = Allocator;
    using reference = value_type &;
    using const_reference = const value_type &;
    using pointer = typename std::allocator_traits<Allocator>::pointer;
    using const_pointer = typename std::allocator_traits<Allocator>::const_pointer;

    // Iterator types.

    using iterator = flat_detail::iter_wrapper<typename StorageT::iterator,
                                               const value_type>;
    using const_iterator =
        flat_detail::iter_wrapper<typename StorageT::const_iterator,
                                  const value_type>;

    using reverse_iterator = std::reverse_iterator<iterator>;
    using const_reverse_iterator = std::reverse_iterator<const_iterator>;

    // Constructors.

    flat_set(const Compare &compare = Compare(),
             const Allocator &alloc = Allocator())
        : comp(compare), data(alloc) {}

    template <class InputIt>
    flat_set(InputIt first, InputIt last, const Compare &compare = Compare(),
             const Allocator &alloc = Allocator())
        : comp(compare), data(alloc) {
        insert(first, last);
    }

    flat_set(std::initializer_list<value_type> init,
             const Compare &compare = Compare(),
             const Allocator &alloc = Allocator())
        : comp(compare), data(alloc) {
        insert(init.begin(), init.end());
    }

    flat_set(const flat_set &) = default;
    flat_set(flat_set &&) = default;
    flat_set &operator=(const flat_set &) = default;
    flat_set &operator=(flat_set &&) = default;

    // Other members.

    allocator_type get_allocator() const {
        return data.get_allocator();
    }

    // Iterators.

    iterator begin() { return iterator(data.begin()); }
    const_iterator cbegin() const { return const_iterator(data.cbegin()); }
    const_iterator begin() const { return cbegin(); }

    iterator end() { return iterator(data.end()); }
    const_iterator cend() const { return const_iterator(data.cend()); }
    const_iterator end() const { return cend(); }

    reverse_iterator rbegin() { return reverse_iterator(end()); }
    const_reverse_iterator crbegin() const {
        return const_reverse_iterator(cend());
    }
    const_reverse_iterator rbegin() const { return crbegin(); }

    reverse_iterator rend() { return reverse_iterator(begin()); }
    const_reverse_iterator crend() const {
        return const_reverse_iterator(cbegin());
    }
    const_reverse_iterator rend() const { return crend(); }

    // Capacity.

    bool empty() const { return data.empty(); }
    size_t size() const { return data.size(); }
    size_t max_size() const { return data.max_size(); }

    // Modifiers.

    void clear() {
        data.clear();
    }

    std::pair<iterator, bool> insert(const value_type &value) {
        auto it = std::lower_bound(data.begin(), data.end(), value, comp);
        if (it == data.end() || comp(value, *it)) {
            return std::make_pair(iterator(data.insert(it, value)), true);
        }
        return std::make_pair(iterator(it), false);
    }

    std::pair<iterator, bool> insert(value_type &&value) {
        auto it = std::lower_bound(data.begin(), data.end(), value, comp);
        if (it == data.end() || comp(value, *it)) {
            return std::make_pair(iterator(data.insert(it, std::move(value))),
                                  true);
        }
        return std::make_pair(iterator(it), false);
    }

    template <class InputIt>
    void insert(InputIt first, InputIt second) {
        for (; first != second; ++first) {
            insert(*first);
        }
    }

    void insert(std::initializer_list<value_type> ilist) {
        insert(ilist.begin(), ilist.end());
    }

    template<class...Args>
    std::pair<iterator, bool> emplace(Args&&... args) {
        return insert(value_type(std::forward<Args>(args)...));
    }

    void erase(iterator pos) {
        data.erase(pos.get());
    }

    void erase(iterator first, iterator last) {
        data.erase(first.get(), last.get());
    }

    void erase(const key_type &key) {
        auto it = find(key);
        if (it != end()) {
            erase(it);
        }
    }

    void swap(flat_set &a) {
        using std::swap;
        swap(comp, a.comp);
        swap(data, a.data);
    }

    // Lookup.

    size_type count(const value_type &value) const {
        return find(value) != end() ? 1 : 0;
    }

    iterator find(const value_type &value) {
        auto it = std::lower_bound(data.begin(), data.end(), value, comp);
        if (it != data.end() && comp(value, *it)) {
            it = data.end();
        }
        return iterator(it);
    }

    const_iterator find(const value_type &value) const {
        auto it = std::lower_bound(data.begin(), data.end(), value, comp);
        if (it != data.end() && comp(value, *it)) {
            it = data.end();
        }
        return const_iterator(it);
    }

    // Observers.

    key_compare key_comp() const {
        return comp;
    }

    value_compare value_comp() const {
        return comp;
    }

    // Operators.

    bool operator==(const flat_set &a) const {
        return data == a.data;
    }
    bool operator!=(const flat_set &a) const {
        return data != a.data;
    }
    bool operator<(const flat_set &a) const {
        return data < a.data;
    }
    bool operator<=(const flat_set &a) const {
        return data <= a.data;
    }
    bool operator>(const flat_set &a) const {
        return data > a.data;
    }
    bool operator>=(const flat_set &a) const {
        return data >= a.data;
    }

    // Free swap function for ADL.
    friend void swap(flat_set &a, flat_set &b) {
        a.swap(b);
    }
};

/**
 * \brief Map container implemented internally as a sorted vector. Use this
 * rather than std::map for small sets as it's faster, uses less memory and
 * incurs less malloc time.
 *
 * Note: we used to use boost::flat_map, but have run into problems with all
 * the extra machinery it instantiates.
 *
 * Note: ue2::flat_map does NOT provide mutable iterators, as (given the way
 * the data is stored) it is difficult to provide a real mutable iterator that
 * wraps std::pair<const Key, T>. Instead, all iterators are const, and you
 * should use flat_map::at() or flat_map::operator[] to mutate the contents of
 * the container.
 */
template <class Key, class T, class Compare = std::less<Key>,
          class Allocator = std::allocator<std::pair<Key, T>>>
class flat_map {
public:
    // Member types.
    using key_type = Key;
    using mapped_type = T;
    using value_type = std::pair<const Key, T>;

private:
    // Underlying storage is a sorted std::vector.
    using storage_type = std::pair<key_type, mapped_type>;
    using StorageT = std::vector<storage_type, Allocator>;

    Compare comp;
    StorageT data;

public:
    // More Member types.
    using size_type = typename StorageT::size_type;
    using difference_type = typename StorageT::difference_type;
    using key_compare = Compare;
    using allocator_type = Allocator;
    using reference = value_type &;
    using const_reference = const value_type &;
    using pointer = typename std::allocator_traits<Allocator>::pointer;
    using const_pointer = typename std::allocator_traits<Allocator>::const_pointer;

public:
    using const_iterator =
        flat_detail::iter_wrapper<typename StorageT::const_iterator,
                                  const storage_type>;

    using const_reverse_iterator = std::reverse_iterator<const_iterator>;

    // All iterators are const for flat_map.
    using iterator = const_iterator;
    using reverse_iterator = const_reverse_iterator;

    // Constructors.

    flat_map(const Compare &compare = Compare(),
             const Allocator &alloc = Allocator())
        : comp(compare), data(alloc) {}

    template <class InputIt>
    flat_map(InputIt first, InputIt last, const Compare &compare = Compare(),
             const Allocator &alloc = Allocator())
        : comp(compare), data(alloc) {
        insert(first, last);
    }

    flat_map(std::initializer_list<value_type> init,
             const Compare &compare = Compare(),
             const Allocator &alloc = Allocator())
        : comp(compare), data(alloc) {
        insert(init.begin(), init.end());
    }

    flat_map(const flat_map &) = default;
    flat_map(flat_map &&) = default;
    flat_map &operator=(const flat_map &) = default;
    flat_map &operator=(flat_map &&) = default;

    // Other members.

    allocator_type get_allocator() const {
        return data.get_allocator();
    }

    // Iterators.

    const_iterator cbegin() const { return const_iterator(data.cbegin()); }
    const_iterator begin() const { return cbegin(); }

    const_iterator cend() const { return const_iterator(data.cend()); }
    const_iterator end() const { return cend(); }

    const_reverse_iterator crbegin() const {
        return const_reverse_iterator(cend());
    }
    const_reverse_iterator rbegin() const { return crbegin(); }

    const_reverse_iterator crend() const {
        return const_reverse_iterator(cbegin());
    }
    const_reverse_iterator rend() const { return crend(); }

    // Capacity.

    bool empty() const { return data.empty(); }
    size_t size() const { return data.size(); }
    size_t max_size() const { return data.max_size(); }

private:
    using storage_iterator = typename StorageT::iterator;
    using storage_const_iterator = typename StorageT::const_iterator;

    storage_iterator data_lower_bound(const key_type &key) {
        return std::lower_bound(
            data.begin(), data.end(), key,
            [&](const storage_type &elem, const key_type &k) {
                return comp(elem.first, k);
            });
    }

    storage_const_iterator
    data_lower_bound(const key_type &key) const {
        return std::lower_bound(
            data.begin(), data.end(), key,
            [&](const storage_type &elem, const key_type &k) {
                return comp(elem.first, k);
            });
    }

    std::pair<storage_iterator, bool> data_insert(const value_type &value) {
        auto it = data_lower_bound(value.first);
        if (it == data.end() || comp(value.first, it->first)) {
            return std::make_pair(data.insert(it, value), true);
        }
        return std::make_pair(it, false);
    }

    std::pair<storage_iterator, bool> data_insert(value_type &&value) {
        auto it = data_lower_bound(value.first);
        if (it == data.end() || comp(value.first, it->first)) {
            return std::make_pair(data.insert(it, std::move(value)), true);
        }
        return std::make_pair(it, false);
    }

    storage_iterator data_find(const key_type &key) {
        auto it = data_lower_bound(key);
        if (it != data.end() && comp(key, it->first)) {
            it = data.end();
        }
        return it;
    }

    storage_const_iterator data_find(const key_type &key) const {
        auto it = data_lower_bound(key);
        if (it != data.end() && comp(key, it->first)) {
            it = data.end();
        }
        return it;
    }

public:
    // Modifiers.

    void clear() {
        data.clear();
    }

    std::pair<iterator, bool> insert(const value_type &value) {
        auto rv = data_insert(value);
        return std::make_pair(iterator(rv.first), rv.second);
    }

    std::pair<iterator, bool> insert(value_type &&value) {
        auto rv = data_insert(std::move(value));
        return std::make_pair(iterator(rv.first), rv.second);
    }

    template <class InputIt>
    void insert(InputIt first, InputIt second) {
        for (; first != second; ++first) {
            insert(*first);
        }
    }

    void insert(std::initializer_list<value_type> ilist) {
        insert(ilist.begin(), ilist.end());
    }

    template<class...Args>
    std::pair<iterator, bool> emplace(Args&&... args) {
        return insert(value_type(std::forward<Args>(args)...));
    }

    void erase(iterator pos) {
        // Convert to a non-const storage iterator via pointer arithmetic.
        storage_iterator it = data.begin() + distance(begin(), pos);
        data.erase(it);
    }

    void erase(iterator first, iterator last) {
        // Convert to a non-const storage iterator via pointer arithmetic.
        storage_iterator data_first = data.begin() + distance(begin(), first);
        storage_iterator data_last = data.begin() + distance(begin(), last);
        data.erase(data_first, data_last);
    }

    void erase(const key_type &key) {
        auto it = find(key);
        if (it != end()) {
            erase(it);
        }
    }

    void swap(flat_map &a) {
        using std::swap;
        swap(comp, a.comp);
        swap(data, a.data);
    }

    // Lookup.

    size_type count(const key_type &key) const {
        return find(key) != end() ? 1 : 0;
    }

    const_iterator find(const key_type &key) const {
        return const_iterator(data_find(key));
    }

    // Element access.

    mapped_type &at(const key_type &key) {
        auto it = data_find(key);
        if (it == data.end()) {
            throw std::out_of_range("element not found");
        }
        return it->second;
    }

    const mapped_type &at(const key_type &key) const {
        auto it = data_find(key);
        if (it == data.end()) {
            throw std::out_of_range("element not found");
        }
        return it->second;
    }

    mapped_type &operator[](const key_type &key) {
        auto p = data_insert(value_type(key, mapped_type()));
        return p.first->second;
    }

    // Observers.

    key_compare key_comp() const {
        return comp;
    }

    // Operators.

    bool operator==(const flat_map &a) const {
        return data == a.data;
    }
    bool operator!=(const flat_map &a) const {
        return data != a.data;
    }
    bool operator<(const flat_map &a) const {
        return data < a.data;
    }
    bool operator<=(const flat_map &a) const {
        return data <= a.data;
    }
    bool operator>(const flat_map &a) const {
        return data > a.data;
    }
    bool operator>=(const flat_map &a) const {
        return data >= a.data;
    }

    // Free swap function for ADL.
    friend void swap(flat_map &a, flat_map &b) {
        a.swap(b);
    }
};

} // namespace

#endif // UTIL_UE2_CONTAINERS_H_
