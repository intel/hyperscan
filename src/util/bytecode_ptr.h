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

/**
 * \file
 * \brief bytecode_ptr: Smart pointer that knows its length and alignment.
 */

#ifndef UTIL_BYTECODE_PTR_H
#define UTIL_BYTECODE_PTR_H

#include "util/alloc.h"

#include <cstring>
#include <memory>

namespace ue2 {

/**
 * \brief Smart pointer that knows its length and alignment.
 *
 * This is intended to be used for flat aligned memory regions that will
 * eventually end up copied into the Hyperscan bytecode.
 */
template<typename T> class bytecode_ptr {
public:
    bytecode_ptr() = default;
    explicit bytecode_ptr(size_t size, size_t align = alignof(T))
        : bytes(size), alignment(align) {
        // posix_memalign doesn't like us asking for smaller alignment.
        size_t mem_align = std::max(align, sizeof(void *));
        ptr.reset(static_cast<T *>(aligned_malloc_internal(size, mem_align)));
        if (!ptr) {
            throw std::bad_alloc();
        }
        std::memset(ptr.get(), 0, bytes);
    }

    bytecode_ptr(std::nullptr_t) {}

    T *get() { return ptr.get(); };
    const T *get() const { return ptr.get(); };

    T &operator*() { return *ptr; }
    const T &operator*() const { return *ptr; }

    T *operator->() { return ptr.get(); }
    const T *operator->() const { return ptr.get(); }

    explicit operator bool() const { return ptr != nullptr; }

    /** \brief Move converter for shared_ptr. */
    template <typename ST, class = typename std::enable_if<
                               std::is_convertible<T *, ST *>::value>::type>
    operator std::shared_ptr<ST>() && {
        auto d = ptr.get_deleter();
        return std::shared_ptr<ST>(ptr.release(), d);
    }

    void reset(T* p = nullptr) { ptr.reset(p); }

    void swap(bytecode_ptr &other) {
        using std::swap;
        swap(ptr, other.ptr);
        swap(bytes, other.bytes);
        swap(alignment, other.alignment);
    }

    /** \brief Returns size of the memory region in bytes. */
    size_t size() const { return bytes; }

    /** \brief Returns alignment of the memory region in bytes. */
    size_t align() const { return alignment; }

    bool operator==(const bytecode_ptr &a) const { return ptr == a.ptr; }
    bool operator!=(const bytecode_ptr &a) const { return ptr != a.ptr; }
    bool operator<(const bytecode_ptr &a) const { return ptr < a.ptr; }
    bool operator<=(const bytecode_ptr &a) const { return ptr <= a.ptr; }
    bool operator>(const bytecode_ptr &a) const { return ptr > a.ptr; }
    bool operator>=(const bytecode_ptr &a) const { return ptr >= a.ptr; }

private:
    /** \brief Deleter function for std::unique_ptr. */
    template <typename DT> struct deleter {
        void operator()(DT *p) const { aligned_free_internal(p); }
    };

    std::unique_ptr<T, deleter<T>> ptr = nullptr; //!< Underlying pointer.
    size_t bytes = 0; //!< Size of memory region in bytes.
    size_t alignment = 0; //!< Alignment of memory region in bytes.
};

template<typename T>
inline bytecode_ptr<T> make_bytecode_ptr(size_t size,
                                         size_t align = alignof(T)) {
    return bytecode_ptr<T>(size, align);
}

} // namespace ue2

#endif // UTIL_BYTECODE_PTR_H
