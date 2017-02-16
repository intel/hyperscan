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

/** \file
 * \brief Convenience template functions for containers.
 */

#ifndef UTIL_CONTAINER_H
#define UTIL_CONTAINER_H

#include "ue2common.h"

#include <algorithm>
#include <cassert>
#include <cstring>
#include <set>
#include <type_traits>
#include <utility>
#include <vector>

namespace ue2 {

// Existence check for associative containers.
template<typename C>
bool contains(const C &container, const typename C::key_type &key) {
    return container.find(key) != container.end();
}

template<typename C, typename It>
bool contains_any_of(const C &container, const std::pair<It, It> &range) {
    return std::find_first_of(range.first, range.second, container.begin(),
                              container.end()) != range.second;
}

template<typename C, typename It>
void insert(C *container, const std::pair<It, It> &range) {
    container->insert(range.first, range.second);
}

template<typename C, typename It>
void insert(C *container, typename C::iterator pos,
            const std::pair<It, It> &range) {
    container->insert(pos, range.first, range.second);
}

template<typename C, typename D>
void insert(C *container, const D &donor) {
    container->insert(donor.begin(), donor.end());
}

template<typename C, typename D>
void insert(C *container, typename C::iterator pos, const D &donor) {
    container->insert(pos, donor.begin(), donor.end());
}

/**
 * \brief Constructs a vector from a range bounded by the given pair of
 * iterators.
 */
template <typename It>
auto make_vector_from(const std::pair<It, It> &range)
    -> std::vector<decltype(*range.first)> {
    using T = decltype(*range.first);
    return std::vector<T>(range.first, range.second);
}

/** \brief Sort a sequence container and remove duplicates. */
template <typename C, typename Compare = std::less<typename C::value_type>>
void sort_and_unique(C &container, Compare comp = Compare()) {
    std::sort(std::begin(container), std::end(container), comp);
    container.erase(std::unique(std::begin(container), std::end(container)),
                    std::end(container));
}

/** \brief Returns a set containing the keys in the given associative
 * container. */
template <typename C>
std::set<typename C::key_type> assoc_keys(const C &container) {
    std::set<typename C::key_type> keys;
    for (const auto &elem : container) {
        keys.insert(elem.first);
    }
    return keys;
}

/**
 * \brief Return the length in bytes of the given vector of (POD) objects.
 */
template <typename T, typename Alloc>
typename std::vector<T, Alloc>::size_type
byte_length(const std::vector<T, Alloc> &vec) {
    static_assert(std::is_pod<T>::value, "should be pod");
    return vec.size() * sizeof(T);
}

/**
 * \brief Copy the given vector of POD objects to the given location in memory.
 * It is safe to give this function an empty vector.
 */
template<typename T, typename Alloc>
void *copy_bytes(void *dest, const std::vector<T, Alloc> &vec) {
    static_assert(std::is_pod<T>::value, "should be pod");
    assert(dest);

    // Since we're generally using this function to write into the bytecode,
    // dest should be appropriately aligned for T.
    assert(ISALIGNED_N(dest, alignof(T)));

    if (vec.empty()) {
        return dest; // Protect memcpy against null pointers.
    }
    assert(vec.data() != nullptr);
    return std::memcpy(dest, vec.data(), byte_length(vec));
}

template<typename OrderedContainer1, typename OrderedContainer2>
bool is_subset_of(const OrderedContainer1 &small, const OrderedContainer2 &big) {
    static_assert(std::is_same<typename OrderedContainer1::value_type,
                               typename OrderedContainer2::value_type>::value,
                  "Both containers should have the same value_type");
    auto sit = small.begin();
    auto bit = big.begin();
    if (small.size() > big.size()) {
        return false;
    }

    while (sit != small.end()) {
        if (bit == big.end()) {
            return false;
        }

        if (*sit == *bit) {
            ++sit;
            ++bit;
            continue;
        }
        if (*bit < *sit) {
            ++bit;
            continue;
        }

        return false;
    }
    return true;
}

template<typename OrderedContainer1, typename OrderedContainer2>
bool has_intersection(const OrderedContainer1 &a, const OrderedContainer2 &b) {
    static_assert(std::is_same<typename OrderedContainer1::value_type,
                               typename OrderedContainer2::value_type>::value,
                  "Both containers should have the same value_type");
    auto ait = a.begin();
    auto bit = b.begin();
    while (ait != a.end() && bit != b.end()) {
        if (*ait == *bit) {
            return true;
        }

        if (*ait < *bit) {
            ++ait;
        } else {
            ++bit;
        }
    }

    return false;
}

/**
 * \brief Erase the elements (by value) in the donor container from the given
 * container.
 */
template<typename C, typename D>
void erase_all(C *container, const D &donor) {
    for (const auto &elem : donor) {
        container->erase(elem);
    }
}


template<typename C, typename Pred>
bool any_of_in(const C &c, Pred p) {
    return std::any_of(c.begin(), c.end(), std::move(p));
}

template<typename C, typename Pred>
bool all_of_in(const C &c, Pred p) {
    return std::all_of(c.begin(), c.end(), std::move(p));
}

} // namespace ue2

#ifdef DUMP_SUPPORT

#include <sstream>
#include <string>

namespace ue2 {

/**
 * \brief Dump a container of stream-printable objects into a comma-separated
 * list in a string.
 */
template<class C>
std::string as_string_list(const C &c) {
    std::ostringstream oss;
    for (auto it = c.begin(); it != c.end(); ++it) {
        if (it != c.begin()) {
            oss << ", ";
        }
        oss << *it;
    }
    return oss.str();
}

} // namespace ue2

#endif // DUMP_SUPPORT

#endif // UTIL_CONTAINER_H
