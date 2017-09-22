/*
 * Copyright (c) 2016-2017, Intel Corporation
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

/**
 * \file
 * \brief Hashing utility functions.
 */

#ifndef UTIL_HASH_H
#define UTIL_HASH_H

#include <functional>
#include <string>
#include <type_traits>
#include <utility>

namespace ue2 {

namespace hash_detail {

inline
void hash_combine_impl(size_t &seed, size_t value) {
    // Note: constants explicitly truncated on 32-bit platforms.
    const size_t a = (size_t)0x0b4e0ef37bc32127ULL;
    const size_t b = (size_t)0x318f07b0c8eb9be9ULL;
    seed ^= value * a;
    seed += b;
}

/** \brief Helper that determines whether std::begin() exists for T. */
template<typename T>
struct is_container_check {
private:
    template<typename C>
    static auto has_begin_function(const C &obj) -> decltype(std::begin(obj)) {
        return std::begin(obj);
    }
    static void has_begin_function(...) {
        return;
    }
    using has_begin_type = decltype(has_begin_function(std::declval<T>()));

public:
    static const bool value = !std::is_void<has_begin_type>::value;
};

/** \brief Type trait to enable on whether T is a container. */
template<typename T>
struct is_container
    : public ::std::integral_constant<bool, is_container_check<T>::value> {};

/** \brief Helper that determines whether T::hash() exists. */
template<typename T>
struct has_hash_member_check {
private:
    template<typename C>
    static auto has_hash_member_function(const C &obj) -> decltype(obj.hash()) {
        return obj.hash();
    }
    static void has_hash_member_function(...) {
        return;
    }
    using has_hash = decltype(has_hash_member_function(std::declval<T>()));

public:
    static const bool value = !std::is_void<has_hash>::value;
};

/** \brief Type trait to enable on whether T::hash() exists. */
template<typename T>
struct has_hash_member
    : public ::std::integral_constant<bool, has_hash_member_check<T>::value> {};

/** \brief Default hash: falls back to std::hash. */
template<typename T, typename Enable = void>
struct ue2_hash {
    using decayed_type = typename std::decay<T>::type;
    size_t operator()(const T &obj) const {
        return std::hash<decayed_type>()(obj);
    }
};

/** \brief Hash for std::pair. */
template<typename A, typename B>
struct ue2_hash<std::pair<A, B>, void> {
    size_t operator()(const std::pair<A, B> &p) const {
        size_t v = 0;
        hash_combine_impl(v, ue2_hash<A>()(p.first));
        hash_combine_impl(v, ue2_hash<B>()(p.second));
        return v;
    }
};

/** \brief Hash for any type that has a hash() member function. */
template<typename T>
struct ue2_hash<T, typename std::enable_if<has_hash_member<T>::value>::type> {
    size_t operator()(const T &obj) const {
        return obj.hash();
    }
};

/**
 * \brief Hash for any container type that supports std::begin().
 *
 * We exempt std::string as std::hash<std:string> is provided and quicker.
 */
template<typename T>
struct ue2_hash<T, typename std::enable_if<
           is_container<T>::value &&
           !std::is_same<typename std::decay<T>::type, std::string>::value &&
           !has_hash_member<T>::value>::type> {
    size_t operator()(const T &obj) const {
        size_t v = 0;
        for (const auto &elem : obj) {
            using element_type = typename std::decay<decltype(elem)>::type;
            hash_combine_impl(v, ue2_hash<element_type>()(elem));
        }
        return v;
    }
};

/** \brief Hash for enum types. */
template<typename T>
struct ue2_hash<T, typename std::enable_if<std::is_enum<T>::value>::type> {
    size_t operator()(const T &obj) const {
        using utype = typename std::underlying_type<T>::type;
        return ue2_hash<utype>()(static_cast<utype>(obj));
    }
};

template<typename T>
void hash_combine(size_t &seed, const T &obj) {
    hash_combine_impl(seed, ue2_hash<T>()(obj));
}

template<typename T>
void hash_build(size_t &v, const T &obj) {
    hash_combine(v, obj);
}

template<typename T, typename... Args>
void hash_build(size_t &v, const T &obj, Args&&... args) {
    hash_build(v, obj);
    hash_build(v, args...); // recursive
}

} // namespace hash_detail

using hash_detail::hash_combine;

/**
 * \brief Hasher for general use.
 *
 * Provides operators for most standard containers and falls back to
 * std::hash<T>.
 */
struct ue2_hasher {
    template<typename T>
    size_t operator()(const T &obj) const {
        return hash_detail::ue2_hash<T>()(obj);
    }
};

/**
 * \brief Computes the combined hash of all its arguments.
 *
 * Simply use:
 *
 *     size_t hash = hash_all(a, b, c, d);
 *
 * Where a, b, c and d are hashable.
 */
template<typename... Args>
size_t hash_all(Args&&... args) {
    size_t v = 0;
    hash_detail::hash_build(v, args...);
    return v;
}

} // namespace ue2

#endif // UTIL_HASH_H
