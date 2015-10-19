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

#ifndef SHUFFLE_SSSE3_H
#define SHUFFLE_SSSE3_H

#include "simd_utils_ssse3.h"

#ifdef DEBUG
#include "compare.h"
static really_inline void shufDumpMsk(m128 msk) {
    u8 * mskAsU8 = (u8 *)&msk;
    for (int i = 0; i < 16; i++) {
        u8 c = mskAsU8[i];
        for (int j = 0; j < 8; j++) {
            if ((c >> (7-j)) & 0x1)
                printf("1");
            else
                printf("0");
        }
        printf(" ");
    }
}

static really_inline void shufDumpMskAsChars(m128 msk) {
    u8 * mskAsU8 = (u8 *)&msk;
    for (int i = 0; i < 16; i++) {
        u8 c = mskAsU8[i];
        if (ourisprint(c))
            printf("%c",c);
        else
            printf(".");
    }
}
#endif

#if !defined(NO_SSSE3)
static really_inline
u32 shufflePshufb128(m128 s, const m128 permute, const m128 compare) {
    m128 shuffled = pshufb(s, permute);
    m128 compared = and128(shuffled, compare);
#ifdef DEBUG
    printf("State:   ");  shufDumpMsk(s);       printf("\n");
    printf("Permute: ");  shufDumpMsk(permute); printf("\n");
    printf("Compare: ");  shufDumpMsk(compare); printf("\n");
    printf("Shuffled: "); shufDumpMsk(shuffled); printf("\n");
    printf("Compared: "); shufDumpMsk(compared); printf("\n");
#endif
    u16 rv = ~cmpmsk8(compared, shuffled);
    return (u32)rv;
}
#endif // NO_SSSE3

#endif // SHUFFLE_SSSE3_H
