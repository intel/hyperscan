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

#ifndef UTIL_SMALL_VECTOR_H
#define UTIL_SMALL_VECTOR_H

#include <vector>

#include <boost/version.hpp>

/*
 * We use the small_vector constructors introduced in Boost 1.61 (trac bug
 * #11866, github commit b436c91). If the Boost version is too old, we fall
 * back to using std::vector.
 */
#if BOOST_VERSION >= 106100
#  define HAVE_BOOST_CONTAINER_SMALL_VECTOR
#endif

#if defined(HAVE_BOOST_CONTAINER_SMALL_VECTOR)
#  include <boost/container/small_vector.hpp>
#endif

namespace ue2 {

#if defined(HAVE_BOOST_CONTAINER_SMALL_VECTOR)

template <class T, std::size_t N,
          typename Allocator = boost::container::new_allocator<T>>
using small_vector = boost::container::small_vector<T, N, Allocator>;

#else

// Boost version isn't new enough, fall back to just using std::vector.
template <class T, std::size_t N, typename Allocator = std::allocator<T>>
using small_vector = std::vector<T, Allocator>;

// Support workarounds for flat_set/flat_map and GCC 4.8.
#define SMALL_VECTOR_IS_STL_VECTOR 1

#endif // HAVE_BOOST_CONTAINER_SMALL_VECTOR

} // namespace ue2

#endif // UTIL_SMALL_VECTOR_H
