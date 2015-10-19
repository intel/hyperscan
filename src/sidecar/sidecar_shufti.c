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

#include "sidecar_shufti.h"
#include "sidecar_internal.h"
#include "ue2common.h"
#include "util/simd_utils.h"
#include "util/simd_utils_ssse3.h"

#define GET_LO_4(chars) and128(chars, low4bits)
#define GET_HI_4(chars) rshift2x64(andnot128(low4bits, chars), 4)

#ifdef DEBUG
#include <ctype.h>
UNUSED static void dumpMsk(m128 msk) {
    u8 *maskAsU8 = (u8 *)&msk;
    for (int i = 0; i < 16; i++) {
        printf("%02hhx ", maskAsU8[i]);
    }
}

UNUSED static void dumpMskAsChars(m128 msk) {
    u8 *maskAsU8 = (u8 *)&msk;
    for (int i = 0; i < 16; i++) {
        u8 c = maskAsU8[i];
        if (isprint(c))
            printf("%c",c);
        else
            printf(".");
    }
}
#endif

static really_inline
u8 squash(m128 t) {
    m128 u = byteShiftRight128(t, 8);
    t = and128(t, u);
    m128 v = byteShiftRight128(t, 4);
    t = and128(t, v);
    u32 gpr = movd(t);
    gpr &= gpr >> 16;
    gpr &= gpr >> 8;
    DEBUG_PRINTF("   gpr: %02x\n", (u8)gpr);
    return (u8)gpr;
}


static really_inline
m128 mainLoop(m128 mask_lo, m128 mask_hi, m128 chars, const m128 low4bits) {
    m128 c_lo  = pshufb(mask_lo, GET_LO_4(chars));
    m128 c_hi  = pshufb(mask_hi, GET_HI_4(chars));
    m128 t     = or128(c_lo, c_hi);

#ifdef DEBUG
    DEBUG_PRINTF(" chars: "); dumpMskAsChars(chars); printf("\n");
    DEBUG_PRINTF("  char: "); dumpMsk(chars);        printf("\n");
    DEBUG_PRINTF("  c_lo: "); dumpMsk(c_lo);         printf("\n");
    DEBUG_PRINTF("  c_hi: "); dumpMsk(c_hi);         printf("\n");
    DEBUG_PRINTF("     t: "); dumpMsk(t);            printf("\n");
#endif

    return t;
}

u8 sidecarExec_S_int(const struct sidecar_S *n, const u8 *b,
                     size_t len, u8 state) {
    const m128 low4bits = _mm_set1_epi8(0xf);
    const u8 *b_end = b + len;
    m128 mask_lo = n->lo;
    m128 mask_hi = n->hi;

    // Preconditioning: most of the time our buffer won't be aligned
    DEBUG_PRINTF("warmup %02hhx\n", state);
    m128 chars = loadu128(b);
    m128 t =  _mm_set1_epi8(state);
    t = and128(t, mainLoop(mask_lo, mask_hi, chars, low4bits));
    b = ROUNDUP_PTR(b + 1, 16);

    // Unrolling was here, but it wasn't doing anything but taking up space.
    // Reroll FTW.

    DEBUG_PRINTF("main %02hhx\n", state);
    const u8 *last_block = b_end - 16;
    while (b < last_block) {
        m128 lchars = load128(b);
        m128 rv = mainLoop(mask_lo, mask_hi, lchars, low4bits);
        t = and128(t, rv);
        b += 16;
        if (!squash(t)) {
            return 0;
        }
    }

    DEBUG_PRINTF("cool down %02hhx\n", state);
    assert(b <= b_end && b >= b_end - 16);
    // do an unaligned load the end to accurate picture to the end
    chars = loadu128(b_end - 16);
    m128 rv = mainLoop(mask_lo, mask_hi, chars, low4bits);
    t = and128(t, rv);

    return squash(t);
}
