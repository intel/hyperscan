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
#include "ue2common.h"
#include "alloc.h"

#include <cstdlib>
#include <cstring>

namespace ue2 {

// This is one of the simplest ways to catch failure where we aren't using an
// aligned_(zmalloc|_free) pair - it will force death if the wrong free is used.
// We use this whenever assertions are switched on.
#if !defined(NDEBUG)
#define HACK_OFFSET 64
#else
#define HACK_OFFSET 0
#endif

/* get us a posix_memalign from somewhere */
#if !defined(HAVE_POSIX_MEMALIGN)
# if defined(HAVE_MEMALIGN)
    #define posix_memalign(A, B, C) ((*A = (void *)memalign(B, C)) == nullptr)
# elif defined(HAVE__ALIGNED_MALLOC)
    /* on Windows */
    #include <malloc.h>
    #define posix_memalign(A, B, C) ((*A = (void *)_aligned_malloc(C, B)) == nullptr)
# else
    #error no posix_memalign or memalign aligned malloc
# endif
#endif

void *aligned_malloc_internal(size_t size, size_t align) {
    void *mem;
#if !defined(_WIN32)
    int rv = posix_memalign(&mem, align, size);
    if (rv != 0) {
        DEBUG_PRINTF("posix_memalign returned %d when asked for %zu bytes\n",
                     rv, size);
        return nullptr;
    }
#else
    if (nullptr == (mem = _aligned_malloc(size, align))) {
        DEBUG_PRINTF("_aligned_malloc failed when asked for %zu bytes\n",
                     size);
        return nullptr;
    }
#endif

    assert(mem);
    return mem;
}

void aligned_free_internal(void *ptr) {
    if (!ptr) {
        return;
    }

#if defined(_WIN32)
    _aligned_free(ptr);
#else
    free(ptr);
#endif
}

/** \brief 64-byte aligned, zeroed malloc.
 *
 * Pointers should be freed with \ref aligned_free. If we are unable to
 * allocate the requested number of bytes, this function will throw
 * std::bad_alloc. */
void *aligned_zmalloc(size_t size) {
    // Really huge allocations are probably an indication that we've
    // done something wrong.
    assert(size < 1024 * 1024 * 1024); // 1GB

    const size_t alloc_size = size + HACK_OFFSET;

    void *mem = aligned_malloc_internal(alloc_size, 64);
    if (!mem) {
        DEBUG_PRINTF("unable to allocate %zu bytes\n", alloc_size);
        throw std::bad_alloc();
    }

    DEBUG_PRINTF("alloced %p reporting %p\n", mem, (char *)mem + HACK_OFFSET);
    assert(ISALIGNED_N(mem, 64));

    memset(mem, 0, alloc_size);
    return (void *)((char *)mem + HACK_OFFSET);
}

/** \brief Free a pointer allocated with \ref aligned_zmalloc. */
void aligned_free(void *ptr) {
    if (!ptr) {
        return;
    }

    void *addr = (void *)((char *)ptr - HACK_OFFSET);
    DEBUG_PRINTF("asked to free %p freeing %p\n", ptr, addr);

    assert(ISALIGNED_N(addr, 64));
    aligned_free_internal(addr);
}

} // namespace ue2
