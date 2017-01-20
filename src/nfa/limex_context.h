/*
 * Copyright (c) 2015-2016, Intel Corporation
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
 * \brief Runtime context structures (NFAContext128 and friends) for the NFA.
 */

#ifndef LIMEX_CONTEXT_H
#define LIMEX_CONTEXT_H

#include "ue2common.h"
#include "callback.h"
#include "util/simd_utils.h" // for m128 etc

// Runtime context structures.

/* Note: The size of the context structures may vary from platform to platform
 * (notably, for the Limex64 structure). As a result, information based on the
 * size and other detail of these structures should not be written into the
 * bytecode -- really, the details of the structure should not be accessed by
 * the ue2 compile side at all.
 */
#ifdef __cplusplus
#error ue2 runtime only file
#endif

/* cached_estate/esucc etc...
 *
 * If the exception state matches the cached_estate we will apply
 * the or in the cached_esucc to the successor states rather than processing
 * the exceptions.
 *
 * If the current exception state is a superset of the cached_estate, the
 * cache is NOT used at all.
 *
 * The cache is updated when we see a different cacheable estate.
 */

#define GEN_CONTEXT_STRUCT(nsize, ntype)                                    \
struct ALIGN_CL_DIRECTIVE NFAContext##nsize {                               \
    ntype s; /**< state bitvector (on entry/exit) */                        \
    ntype local_succ; /**< used by exception handling for large models */   \
    ntype cached_estate; /* inited to 0 */                                  \
    ntype cached_esucc;                                                     \
    char cached_br; /**< cached_estate contains a br state */               \
    const ReportID *cached_reports;                                         \
    union RepeatControl *repeat_ctrl;                                       \
    char *repeat_state;                                                     \
    NfaCallback callback;                                                   \
    void *context;                                                          \
};

GEN_CONTEXT_STRUCT(32,  u32)
#ifdef ARCH_64_BIT
GEN_CONTEXT_STRUCT(64,  u64a)
#else
GEN_CONTEXT_STRUCT(64,  m128)
#endif
GEN_CONTEXT_STRUCT(128, m128)
GEN_CONTEXT_STRUCT(256, m256)
GEN_CONTEXT_STRUCT(384, m384)
GEN_CONTEXT_STRUCT(512, m512)

#undef GEN_CONTEXT_STRUCT

#endif
