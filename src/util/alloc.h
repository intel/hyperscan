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

/** \file
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

template <typename T> struct AlignedDeleter {
    void operator()(T *ptr) const { aligned_free(ptr); }
};
template <typename T>
using aligned_unique_ptr = std::unique_ptr<T, AlignedDeleter<T>>;

/** \brief 64-byte aligned, zeroed malloc that returns an appropriately-typed
 * aligned_unique_ptr.
 *
 * If the requested size cannot be allocated, throws std::bad_alloc.
 */
template <typename T>
inline
aligned_unique_ptr<T> aligned_zmalloc_unique(size_t size) {
    T* ptr = static_cast<T *>(aligned_zmalloc(size));
    assert(ptr); // Guaranteed by aligned_zmalloc.
    return aligned_unique_ptr<T>(ptr);
}

/** \brief Internal use only, used by AlignedAllocator. */
void *aligned_malloc_internal(size_t size, size_t align);

/** \brief Internal use only, used by AlignedAllocator. */
void aligned_free_internal(void *ptr);

/** \brief Aligned allocator class for use with STL containers. Ensures that
 * your objects are aligned to N bytes. */
template <typename T, std::size_t N> class AlignedAllocator {
public:
    typedef T value_type;
    typedef std::size_t size_type;
    typedef std::ptrdiff_t difference_type;
    typedef T *pointer;
    typedef const T *const_pointer;
    typedef T &reference;
    typedef const T &const_reference;

    template <typename U> struct rebind {
        typedef AlignedAllocator<U, N> other;
    };

    pointer address(reference x) const { return &x; }
    const_pointer address(const_reference x) const { return &x; }

    size_type max_size() const {
        return std::numeric_limits<size_type>::max() / sizeof(value_type);
    }

    pointer allocate(size_type size) const {
        return static_cast<pointer>(
            aligned_malloc_internal(size * sizeof(value_type), N));
    }

    void deallocate(pointer x, size_type) const { aligned_free_internal(x); }

    void construct(pointer x, const value_type &val) const {
        new (x) value_type(val);
    }

    void destroy(pointer p) const { p->~value_type(); }

    bool operator==(const AlignedAllocator<T, N> &) const {
        // All instances of AlignedAllocator can dealloc each others' memory.
        return true;
    }

    bool operator!=(const AlignedAllocator<T, N> &) const {
        // All instances of AlignedAllocator can dealloc each others' memory.
        return false;
    }
};

} // namespace ue2

#endif
