/*
 * Copyright (c) 2018, Intel Corporation
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

#include "ch.h"
#include "ch_common.h"
#include "ch_internal.h"
#include "hs.h"
#include "ue2common.h"

#define default_malloc malloc
#define default_free free

ch_alloc_t ch_database_alloc = default_malloc;
ch_alloc_t ch_misc_alloc = default_malloc;
ch_alloc_t ch_scratch_alloc = default_malloc;

ch_free_t ch_database_free = default_free;
ch_free_t ch_misc_free = default_free;
ch_free_t ch_scratch_free = default_free;

static
ch_alloc_t normalise_alloc(ch_alloc_t a) {
    if (!a) {
        return default_malloc;
    } else {
        return a;
    }
}

static
ch_free_t normalise_free(ch_free_t f) {
    if (!f) {
        return default_free;
    } else {
        return f;
    }
}

HS_PUBLIC_API
ch_error_t HS_CDECL ch_set_allocator(ch_alloc_t allocfunc,
                                     ch_free_t freefunc) {
    ch_set_database_allocator(allocfunc, freefunc);
    ch_set_misc_allocator(allocfunc, freefunc);
    ch_set_scratch_allocator(allocfunc, freefunc);

    // Set core Hyperscan alloc/free.
    hs_error_t ret = hs_set_allocator(allocfunc, freefunc);

    return ret;
}

HS_PUBLIC_API
ch_error_t HS_CDECL ch_set_database_allocator(ch_alloc_t allocfunc,
                                              ch_free_t freefunc) {
    ch_database_alloc = normalise_alloc(allocfunc);
    ch_database_free = normalise_free(freefunc);

    // Set Hyperscan database alloc/free.
    return hs_set_database_allocator(allocfunc, freefunc);
}

HS_PUBLIC_API
ch_error_t HS_CDECL ch_set_misc_allocator(ch_alloc_t allocfunc,
                                          ch_free_t freefunc) {
    ch_misc_alloc = normalise_alloc(allocfunc);
    ch_misc_free = normalise_free(freefunc);

    // Set Hyperscan misc alloc/free.
    return hs_set_misc_allocator(allocfunc, freefunc);
}

HS_PUBLIC_API
ch_error_t HS_CDECL ch_set_scratch_allocator(ch_alloc_t allocfunc,
                                             ch_free_t freefunc) {
    ch_scratch_alloc = normalise_alloc(allocfunc);
    ch_scratch_free = normalise_free(freefunc);

    // Set Hyperscan scratch alloc/free.
    return hs_set_scratch_allocator(allocfunc, freefunc);
}
