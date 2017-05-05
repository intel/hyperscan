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
 * \brief Naive dynamic shuffles.
 *
 * These are written with the assumption that the provided masks are sparsely
 * populated and never contain more than 32 on bits. Other implementations will
 * be faster and actually correct if these assumptions don't hold true.
 */

#ifndef LIMEX_SHUFFLE_H
#define LIMEX_SHUFFLE_H

#include "ue2common.h"
#include "util/arch.h"
#include "util/bitutils.h"
#include "util/simd_utils.h"

static really_inline
u32 packedExtract128(m128 s, const m128 permute, const m128 compare) {
    m128 shuffled = pshufb_m128(s, permute);
    m128 compared = and128(shuffled, compare);
    u16 rv = ~movemask128(eq128(compared, shuffled));
    return (u32)rv;
}

#if defined(HAVE_AVX2)
static really_inline
u32 packedExtract256(m256 s, const m256 permute, const m256 compare) {
    // vpshufb doesn't cross lanes, so this is a bit of a cheat
    m256 shuffled = pshufb_m256(s, permute);
    m256 compared = and256(shuffled, compare);
    u32 rv = ~movemask256(eq256(compared, shuffled));
    // stitch the lane-wise results back together
    return (u32)((rv >> 16) | (rv & 0xffffU));
}
#endif // AVX2

#if defined(HAVE_AVX512)
static really_inline
u32 packedExtract512(m512 s, const m512 permute, const m512 compare) {
    // vpshufb doesn't cross lanes, so this is a bit of a cheat
    m512 shuffled = pshufb_m512(s, permute);
    m512 compared = and512(shuffled, compare);
    u64a rv = ~eq512mask(compared, shuffled);
    // stitch the lane-wise results back together
    rv = rv >> 32 | rv;
    return (u32)(((rv >> 16) | rv) & 0xffffU);
}
#endif // AVX512

#endif // LIMEX_SHUFFLE_H
