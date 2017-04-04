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

/**
 * \file
 * \brief Aligned memory alloc/free.
 */

#ifndef UTIL_ALLOC_H
#define UTIL_ALLOC_H

#include <cassert>
#include <cstddef> // size_t
#include <limits>
#include <memory>
#include <new>     // std::bad_alloc

namespace ue2 {

/** \brief 64-byte aligned, zeroed malloc.
 *
 * Pointers should be freed with \ref aligned_free. If we are unable to
 * allocate the requested number of bytes, this function will throw
 * std::bad_alloc. */
void *aligned_zmalloc(size_t size);

/** \brief Free a pointer allocated with \ref aligned_zmalloc. */
void aligned_free(void *ptr);

/** \brief Internal use only, used by AlignedAllocator. */
void *aligned_malloc_internal(size_t size, size_t align);

/** \brief Internal use only, used by AlignedAllocator. */
void aligned_free_internal(void *ptr);

/** \brief Aligned allocator class for use with STL containers. Ensures that
 * your objects are aligned to N bytes. */
template <class T, std::size_t N>
class AlignedAllocator {
public:
    using value_type = T;

    AlignedAllocator() noexcept {}

    template <class U, std::size_t N2>
    AlignedAllocator(const AlignedAllocator<U, N2> &) noexcept {}

    template <class U> struct rebind {
        using other = AlignedAllocator<U, N>;
    };

    T *allocate(std::size_t size) const {
        size_t alloc_size = size * sizeof(T);
        return static_cast<T *>(aligned_malloc_internal(alloc_size, N));
    }

    void deallocate(T *x, std::size_t) const noexcept {
        aligned_free_internal(x);
    }
};

template <class T, class U, std::size_t N, std::size_t N2>
bool operator==(const AlignedAllocator<T, N> &,
                const AlignedAllocator<U, N2> &) {
    return true;
}

template <class T, class U, std::size_t N, std::size_t N2>
bool operator!=(const AlignedAllocator<T, N> &a,
                const AlignedAllocator<U, N2> &b) {
    return !(a == b);
}

} // namespace ue2

#endif
