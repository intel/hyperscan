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

#ifndef MULTIACCEL_DOUBLESHIFT_H_
#define MULTIACCEL_DOUBLESHIFT_H_

#include "multiaccel_common.h"

#define DOUBLESHIFT_MATCH(len, match_t, match_sz) \
    static really_inline \
    const u8 * JOIN4(doubleshiftMatch_, match_sz, _, len)(const u8 *buf, match_t z, u32 len2) {\
        if (unlikely(z)) { \
            match_t tmp = z; \
            z |= ((match_t) (1 << (len)) - 1) << (match_sz / 2); \
            tmp |= ((match_t) (1 << (len + len2)) - 1) << (match_sz / 2); \
            VARISHIFT(z, z, len); \
            VARISHIFT(tmp, tmp, len2); \
            VARISHIFT(tmp, z, len); \
            return JOIN(match, match_sz)(buf, z); \
        } \
        return NULL; \
    }

#define DOUBLESHIFT_MATCH_32_DEF(n) \
        DOUBLESHIFT_MATCH(n, u32, 32)
#define DOUBLESHIFT_MATCH_64_DEF(n) \
        DOUBLESHIFT_MATCH(n, u64a, 64)
#define DOUBLESHIFT_MATCH_DEF(n) \
    DOUBLESHIFT_MATCH_32_DEF(n) \
    DOUBLESHIFT_MATCH_64_DEF(n)

DOUBLESHIFT_MATCH_DEF(1)
DOUBLESHIFT_MATCH_DEF(2)
DOUBLESHIFT_MATCH_DEF(3)
DOUBLESHIFT_MATCH_DEF(4)
DOUBLESHIFT_MATCH_DEF(5)
DOUBLESHIFT_MATCH_DEF(6)
DOUBLESHIFT_MATCH_DEF(7)
DOUBLESHIFT_MATCH_DEF(8)
DOUBLESHIFT_MATCH_DEF(9)
DOUBLESHIFT_MATCH_DEF(10)
DOUBLESHIFT_MATCH_DEF(11)
DOUBLESHIFT_MATCH_DEF(12)
DOUBLESHIFT_MATCH_DEF(13)
DOUBLESHIFT_MATCH_DEF(14)
DOUBLESHIFT_MATCH_DEF(15)
DOUBLESHIFT_MATCH_64_DEF(16)
DOUBLESHIFT_MATCH_64_DEF(17)
DOUBLESHIFT_MATCH_64_DEF(18)
DOUBLESHIFT_MATCH_64_DEF(19)
DOUBLESHIFT_MATCH_64_DEF(20)
DOUBLESHIFT_MATCH_64_DEF(21)
DOUBLESHIFT_MATCH_64_DEF(22)
DOUBLESHIFT_MATCH_64_DEF(23)
DOUBLESHIFT_MATCH_64_DEF(24)
DOUBLESHIFT_MATCH_64_DEF(25)
DOUBLESHIFT_MATCH_64_DEF(26)
DOUBLESHIFT_MATCH_64_DEF(27)
DOUBLESHIFT_MATCH_64_DEF(28)
DOUBLESHIFT_MATCH_64_DEF(29)
DOUBLESHIFT_MATCH_64_DEF(30)
DOUBLESHIFT_MATCH_64_DEF(31)

static
const UNUSED u8 * (*doubleshift_match_funcs_32[])(const u8 *buf, u32 z, u32 len2) =
{
// skip the first
    0,
    &doubleshiftMatch_32_1,
    &doubleshiftMatch_32_2,
    &doubleshiftMatch_32_3,
    &doubleshiftMatch_32_4,
    &doubleshiftMatch_32_5,
    &doubleshiftMatch_32_6,
    &doubleshiftMatch_32_7,
    &doubleshiftMatch_32_8,
    &doubleshiftMatch_32_9,
    &doubleshiftMatch_32_10,
    &doubleshiftMatch_32_11,
    &doubleshiftMatch_32_12,
    &doubleshiftMatch_32_13,
    &doubleshiftMatch_32_14,
    &doubleshiftMatch_32_15,
};

static
const UNUSED u8 * (*doubleshift_match_funcs_64[])(const u8 *buf, u64a z, u32 len2) =
{
// skip the first
    0,
    &doubleshiftMatch_64_1,
    &doubleshiftMatch_64_2,
    &doubleshiftMatch_64_3,
    &doubleshiftMatch_64_4,
    &doubleshiftMatch_64_5,
    &doubleshiftMatch_64_6,
    &doubleshiftMatch_64_7,
    &doubleshiftMatch_64_8,
    &doubleshiftMatch_64_9,
    &doubleshiftMatch_64_10,
    &doubleshiftMatch_64_11,
    &doubleshiftMatch_64_12,
    &doubleshiftMatch_64_13,
    &doubleshiftMatch_64_14,
    &doubleshiftMatch_64_15,
    &doubleshiftMatch_64_16,
    &doubleshiftMatch_64_17,
    &doubleshiftMatch_64_18,
    &doubleshiftMatch_64_19,
    &doubleshiftMatch_64_20,
    &doubleshiftMatch_64_21,
    &doubleshiftMatch_64_22,
    &doubleshiftMatch_64_23,
    &doubleshiftMatch_64_24,
    &doubleshiftMatch_64_25,
    &doubleshiftMatch_64_26,
    &doubleshiftMatch_64_27,
    &doubleshiftMatch_64_28,
    &doubleshiftMatch_64_29,
    &doubleshiftMatch_64_30,
    &doubleshiftMatch_64_31,
};

#endif /* MULTIACCEL_DOUBLESHIFT_H_ */
