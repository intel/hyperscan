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
 * \brief Runtime functions for setting custom allocators.
 */

#include <stdlib.h>
#include <string.h>

#include "allocator.h"

#define default_malloc malloc
#define default_free free

hs_alloc_t hs_database_alloc = default_malloc;
hs_alloc_t hs_misc_alloc = default_malloc;
hs_alloc_t hs_scratch_alloc = default_malloc;
hs_alloc_t hs_stream_alloc = default_malloc;

hs_free_t hs_database_free = default_free;
hs_free_t hs_misc_free = default_free;
hs_free_t hs_scratch_free = default_free;
hs_free_t hs_stream_free = default_free;

static
hs_alloc_t normalise_alloc(hs_alloc_t a) {
    if (!a) {
        return default_malloc;
    } else {
        return a;
    }
}

static
hs_free_t normalise_free(hs_free_t f) {
    if (!f) {
        return default_free;
    } else {
        return f;
    }
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_set_allocator(hs_alloc_t allocfunc, hs_free_t freefunc) {
    hs_set_database_allocator(allocfunc, freefunc);
    hs_set_misc_allocator(allocfunc, freefunc);
    hs_set_stream_allocator(allocfunc, freefunc);
    hs_set_scratch_allocator(allocfunc, freefunc);

    return HS_SUCCESS;
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_set_database_allocator(hs_alloc_t allocfunc,
                                              hs_free_t freefunc) {
    hs_database_alloc = normalise_alloc(allocfunc);
    hs_database_free = normalise_free(freefunc);

    return HS_SUCCESS;
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_set_misc_allocator(hs_alloc_t allocfunc,
                                          hs_free_t freefunc) {
    hs_misc_alloc = normalise_alloc(allocfunc);
    hs_misc_free = normalise_free(freefunc);

    return HS_SUCCESS;
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_set_scratch_allocator(hs_alloc_t allocfunc,
                                             hs_free_t freefunc) {
    hs_scratch_alloc = normalise_alloc(allocfunc);
    hs_scratch_free = normalise_free(freefunc);

    return HS_SUCCESS;
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_set_stream_allocator(hs_alloc_t allocfunc,
                                            hs_free_t freefunc) {
    hs_stream_alloc = normalise_alloc(allocfunc);
    hs_stream_free = normalise_free(freefunc);

    return HS_SUCCESS;
}
