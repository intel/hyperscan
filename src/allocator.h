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

#ifndef ALLOCATOR_H
#define ALLOCATOR_H

#include "hs_common.h"
#include "ue2common.h"

#ifdef __cplusplus
extern "C"
{
#endif
extern hs_alloc_t hs_database_alloc;
extern hs_alloc_t hs_misc_alloc;
extern hs_alloc_t hs_scratch_alloc;
extern hs_alloc_t hs_stream_alloc;

extern hs_free_t hs_database_free;
extern hs_free_t hs_misc_free;
extern hs_free_t hs_scratch_free;
extern hs_free_t hs_stream_free;
#ifdef __cplusplus
} /* extern C */
#endif
/** \brief Check the results of an alloc done with hs_alloc for alignment.
 *
 * If we have incorrect alignment, return an error. Caller should free the
 * offending block. */
static really_inline
hs_error_t hs_check_alloc(const void *mem) {
    hs_error_t ret = HS_SUCCESS;
    if (!mem) {
        ret = HS_NOMEM;
    } else if (!ISALIGNED_N(mem, alignof(unsigned long long))) {
        ret = HS_BAD_ALLOC;
    }
    return ret;
}

#endif
