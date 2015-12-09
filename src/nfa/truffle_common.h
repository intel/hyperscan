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

#ifndef TRUFFLE_COMMON_H_
#define TRUFFLE_COMMON_H_

#include "util/bitutils.h"
#include "util/simd_utils.h"
#include "util/simd_utils_ssse3.h"

/*
 * Common stuff for all versions of truffle (single, multi and multidouble)
 */
#if !defined(__AVX2__)

static really_inline
const u8 *firstMatch(const u8 *buf, u32 z) {
    if (unlikely(z != 0xffff)) {
        u32 pos = ctz32(~z & 0xffff);
        assert(pos < 16);
        return buf + pos;
    }

    return NULL; // no match
}

#define shift128r(a, b) _mm_srli_epi64((a), (b))
static really_inline
u32 block(m128 shuf_mask_lo_highclear, m128 shuf_mask_lo_highset, m128 v) {

    m128 highconst = _mm_set1_epi8(0x80);
    m128 shuf_mask_hi = _mm_set1_epi64x(0x8040201008040201);

    // and now do the real work
    m128 shuf1 = pshufb(shuf_mask_lo_highclear, v);
    m128 t1 = xor128(v, highconst);
    m128 shuf2 = pshufb(shuf_mask_lo_highset, t1);
    m128 t2 = andnot128(highconst, shift128r(v, 4));
    m128 shuf3 = pshufb(shuf_mask_hi, t2);
    m128 tmp = and128(or128(shuf1, shuf2), shuf3);
    m128 tmp2 = eq128(tmp, zeroes128());
    u32 z = movemask128(tmp2);

    return z;
}

static
const u8 *truffleMini(m128 shuf_mask_lo_highclear, m128 shuf_mask_lo_highset,
                       const u8 *buf, const u8 *buf_end) {
    uintptr_t len = buf_end - buf;
    assert(len < 16);

    m128 chars = zeroes128();
    memcpy(&chars, buf, len);

    u32 z = block(shuf_mask_lo_highclear, shuf_mask_lo_highset, chars);
    // can't be these bytes in z
    u32 mask = (0xFFFF >> (16 - len)) ^ 0xFFFF;
    const u8 *rv = firstMatch(buf, z| mask);

    if (rv) {
        return rv;
    } else {
        return buf_end;
    }
}

#else

static really_inline
const u8 *firstMatch(const u8 *buf, u32 z) {
    if (unlikely(z != 0xffffffff)) {
        u32 pos = ctz32(~z);
        assert(pos < 32);
        return buf + pos;
    }

    return NULL; // no match
}

#define shift256r(a, b) _mm256_srli_epi64((a), (b))
static really_inline
u32 block(m256 shuf_mask_lo_highclear, m256 shuf_mask_lo_highset, m256 v) {

    m256 highconst = _mm256_set1_epi8(0x80);
    m256 shuf_mask_hi = _mm256_set1_epi64x(0x8040201008040201);

    // and now do the real work
    m256 shuf1 = vpshufb(shuf_mask_lo_highclear, v);
    m256 t1 = xor256(v, highconst);
    m256 shuf2 = vpshufb(shuf_mask_lo_highset, t1);
    m256 t2 = andnot256(highconst, shift256r(v, 4));
    m256 shuf3 = vpshufb(shuf_mask_hi, t2);
    m256 tmp = and256(or256(shuf1, shuf2), shuf3);
    m256 tmp2 = eq256(tmp, zeroes256());
    u32 z = movemask256(tmp2);

    return z;
}

static
const u8 *truffleMini(m256 shuf_mask_lo_highclear, m256 shuf_mask_lo_highset,
                       const u8 *buf, const u8 *buf_end) {
    uintptr_t len = buf_end - buf;
    assert(len < 32);

    m256 chars = zeroes256();
    memcpy(&chars, buf, len);

    u32 z = block(shuf_mask_lo_highclear, shuf_mask_lo_highset, chars);
    // can't be these bytes in z
    u32 mask = (0xFFFFFFFF >> (32 - len)) ^ 0xFFFFFFFF;
    const u8 *rv = firstMatch(buf, z | mask);

    if (rv) {
        return rv;
    } else {
        return buf_end;
    }
}

#endif

#endif /* TRUFFLE_COMMON_H_ */
