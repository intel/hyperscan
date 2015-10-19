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
 * \brief Mask-based state compression, used by the NFA.
 */

#ifndef STATE_COMPRESS_H
#define STATE_COMPRESS_H

#include "simd_utils.h"
#include "ue2common.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* Note: bytes is not used by implementations >= 128 */

void storecompressed32(void *ptr, const u32 *x, const u32 *m, u32 bytes);
void loadcompressed32(u32 *x, const void *ptr, const u32 *m, u32 bytes);

void storecompressed64(void *ptr, const u64a *x, const u64a *m, u32 bytes);
void loadcompressed64(u64a *x, const void *ptr, const u64a *m, u32 bytes);

void storecompressed128(void *ptr, const m128 *x, const m128 *m, u32 bytes);
void loadcompressed128(m128 *x, const void *ptr, const m128 *m, u32 bytes);

void storecompressed256(void *ptr, const m256 *x, const m256 *m, u32 bytes);
void loadcompressed256(m256 *x, const void *ptr, const m256 *m, u32 bytes);

void storecompressed384(void *ptr, const m384 *x, const m384 *m, u32 bytes);
void loadcompressed384(m384 *x, const void *ptr, const m384 *m, u32 bytes);

void storecompressed512(void *ptr, const m512 *x, const m512 *m, u32 bytes);
void loadcompressed512(m512 *x, const void *ptr, const m512 *m, u32 bytes);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
