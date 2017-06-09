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

#include <iterator>
#include <boost/functional/hash/hash_fwd.hpp>

namespace ue2 {

namespace hash_detail {

template<typename T>
void hash_build(size_t &v, const T &obj) {
    boost::hash_combine(v, obj);
}

template<typename T, typename... Args>
void hash_build(size_t &v, const T &obj, Args&&... args) {
    hash_build(v, obj);
    hash_build(v, args...); // recursive
}

} // namespace hash_detail

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

/**
 * \brief Compute the hash of all the elements of any range on which we can
 * call std::begin() and std::end().
 */
template<typename Range>
size_t hash_range(const Range &r) {
    return boost::hash_range(std::begin(r), std::end(r));
}

} // namespace ue2

#endif // UTIL_HASH_H
