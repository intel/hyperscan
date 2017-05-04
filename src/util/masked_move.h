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

#ifndef MASKED_MOVE_H
#define MASKED_MOVE_H

#include "arch.h"

#if defined(HAVE_AVX2)

#include "unaligned.h"
#include "simd_utils.h"

#ifdef __cplusplus
extern "C" {
#endif
extern const u32 mm_mask_mask[16];
extern const u32 mm_shuffle_end[32][8];
#ifdef __cplusplus
}
#endif

/* load mask for len bytes from start of buffer */
static really_inline m256
_get_mm_mask_end(u32 len) {
    assert(len <= 32);
    const u8 *masky = (const u8 *)mm_mask_mask;
    m256 mask = load256(masky + 32);
    mask = _mm256_sll_epi32(mask, _mm_cvtsi32_si128(8 - (len >> 2)));
    return mask;
}

/*
 * masked_move256_len: Will load len bytes from *buf into m256
 * _______________________________
 * |0<----len---->|            32|
 * -------------------------------
 */
static really_inline m256
masked_move256_len(const u8 *buf, const u32 len) {
    assert(len >= 4);

    m256 lmask = _get_mm_mask_end(len);

    u32 end = unaligned_load_u32(buf + len - 4);
    m256 preshufend = _mm256_broadcastq_epi64(_mm_cvtsi32_si128(end));
    m256 v = _mm256_maskload_epi32((const int *)buf, lmask);
    m256 shufend = pshufb_m256(preshufend,
                               loadu256(&mm_shuffle_end[len - 4]));
    m256 target = or256(v, shufend);

    return target;
}

#endif /* AVX2 */
#endif /* MASKED_MOVE_H */

