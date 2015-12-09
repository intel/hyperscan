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

#ifndef SHUFTI_COMMON_H_
#define SHUFTI_COMMON_H_

#include "ue2common.h"

#include "util/bitutils.h"
#include "util/simd_utils.h"
#include "util/unaligned.h"
#include "util/simd_utils_ssse3.h"

/*
 * Common stuff for all versions of shufti (single, multi and multidouble)
 */

/** \brief Naive byte-by-byte implementation. */
static really_inline
const u8 *shuftiFwdSlow(const u8 *lo, const u8 *hi, const u8 *buf,
                        const u8 *buf_end) {
    assert(buf < buf_end);

    for (; buf < buf_end; ++buf) {
        u8 c = *buf;
        if (lo[c & 0xf] & hi[c >> 4]) {
            break;
        }
    }
    return buf;
}

#ifdef DEBUG
#include <ctype.h>

#define DUMP_MSK(_t)                                \
static UNUSED                                       \
void dumpMsk##_t(m##_t msk) {                       \
    u8 * mskAsU8 = (u8 *)&msk;                      \
    for (unsigned i = 0; i < sizeof(msk); i++) {    \
        u8 c = mskAsU8[i];                          \
        for (int j = 0; j < 8; j++) {               \
            if ((c >> (7-j)) & 0x1)                 \
                printf("1");                        \
            else                                    \
                printf("0");                        \
        }                                           \
        printf(" ");                                \
    }                                               \
}                                                   \
static UNUSED                                       \
void dumpMsk##_t##AsChars(m##_t msk) {              \
    u8 * mskAsU8 = (u8 *)&msk;                      \
    for (unsigned i = 0; i < sizeof(msk); i++) {    \
        u8 c = mskAsU8[i];                          \
        if (isprint(c))                             \
            printf("%c",c);                         \
        else                                        \
            printf(".");                            \
    }                                               \
}

#endif

#if !defined(__AVX2__)

#ifdef DEBUG
DUMP_MSK(128)
#endif

#define GET_LO_4(chars) and128(chars, low4bits)
#define GET_HI_4(chars) rshift2x64(andnot128(low4bits, chars), 4)

static really_inline
u32 block(m128 mask_lo, m128 mask_hi, m128 chars, const m128 low4bits,
          const m128 compare) {
    m128 c_lo  = pshufb(mask_lo, GET_LO_4(chars));
    m128 c_hi  = pshufb(mask_hi, GET_HI_4(chars));
    m128 t     = and128(c_lo, c_hi);

#ifdef DEBUG
    DEBUG_PRINTF(" chars: "); dumpMsk128AsChars(chars); printf("\n");
    DEBUG_PRINTF("  char: "); dumpMsk128(chars);        printf("\n");
    DEBUG_PRINTF("  c_lo: "); dumpMsk128(c_lo);         printf("\n");
    DEBUG_PRINTF("  c_hi: "); dumpMsk128(c_hi);         printf("\n");
    DEBUG_PRINTF("     t: "); dumpMsk128(t);            printf("\n");
#endif
    return movemask128(eq128(t, compare));
}

#else

#ifdef DEBUG
DUMP_MSK(256)
#endif

#define GET_LO_4(chars) and256(chars, low4bits)
#define GET_HI_4(chars) rshift4x64(andnot256(low4bits, chars), 4)

static really_inline
u32 block(m256 mask_lo, m256 mask_hi, m256 chars, const m256 low4bits,
          const m256 compare) {
    m256 c_lo  = vpshufb(mask_lo, GET_LO_4(chars));
    m256 c_hi  = vpshufb(mask_hi, GET_HI_4(chars));
    m256 t = and256(c_lo, c_hi);

#ifdef DEBUG
    DEBUG_PRINTF(" chars: "); dumpMsk256AsChars(chars); printf("\n");
    DEBUG_PRINTF("  char: "); dumpMsk256(chars); printf("\n");
    DEBUG_PRINTF("  c_lo: "); dumpMsk256(c_lo); printf("\n");
    DEBUG_PRINTF("  c_hi: "); dumpMsk256(c_hi); printf("\n");
    DEBUG_PRINTF("     t: "); dumpMsk256(t); printf("\n");
#endif

    return movemask256(eq256(t, compare));
}

#endif


#endif /* SHUFTI_COMMON_H_ */
