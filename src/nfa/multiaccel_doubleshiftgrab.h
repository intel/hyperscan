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

#ifndef MULTIACCEL_DOUBLESHIFTGRAB_H_
#define MULTIACCEL_DOUBLESHIFTGRAB_H_

#include "multiaccel_common.h"

#define DOUBLESHIFTGRAB_MATCH(len, match_t, match_sz) \
    static really_inline \
    const u8 * JOIN4(doubleshiftgrabMatch_, match_sz, _, len)(const u8 *buf, match_t z, u32 len2) {\
        if (unlikely(z)) { \
            match_t neg = ~z; \
            match_t tmp = z; \
            z |= ((match_t) (1 << (len)) - 1) << (match_sz / 2); \
            tmp |= ((match_t) (1 << (len + len2)) - 1) << (match_sz / 2); \
            neg |= ((match_t) (1 << len) - 1) << (match_sz / 2); \
            VARISHIFT(z, z, len); \
            VARISHIFT(tmp, tmp, len2); \
            VARISHIFT(neg, z, 1); \
            VARISHIFT(tmp, z, len); \
            return JOIN(match, match_sz)(buf, z); \
        } \
        return NULL; \
    }

#define DOUBLESHIFTGRAB_MATCH_32_DEF(n) \
        DOUBLESHIFTGRAB_MATCH(n, u32, 32)
#define DOUBLESHIFTGRAB_MATCH_64_DEF(n) \
        DOUBLESHIFTGRAB_MATCH(n, u64a, 64)
#define DOUBLESHIFTGRAB_MATCH_DEF(n) \
    DOUBLESHIFTGRAB_MATCH_32_DEF(n) \
    DOUBLESHIFTGRAB_MATCH_64_DEF(n)

DOUBLESHIFTGRAB_MATCH_DEF(1)
DOUBLESHIFTGRAB_MATCH_DEF(2)
DOUBLESHIFTGRAB_MATCH_DEF(3)
DOUBLESHIFTGRAB_MATCH_DEF(4)
DOUBLESHIFTGRAB_MATCH_DEF(5)
DOUBLESHIFTGRAB_MATCH_DEF(6)
DOUBLESHIFTGRAB_MATCH_DEF(7)
DOUBLESHIFTGRAB_MATCH_DEF(8)
DOUBLESHIFTGRAB_MATCH_DEF(9)
DOUBLESHIFTGRAB_MATCH_DEF(10)
DOUBLESHIFTGRAB_MATCH_DEF(11)
DOUBLESHIFTGRAB_MATCH_DEF(12)
DOUBLESHIFTGRAB_MATCH_DEF(13)
DOUBLESHIFTGRAB_MATCH_DEF(14)
DOUBLESHIFTGRAB_MATCH_DEF(15)
DOUBLESHIFTGRAB_MATCH_64_DEF(16)
DOUBLESHIFTGRAB_MATCH_64_DEF(17)
DOUBLESHIFTGRAB_MATCH_64_DEF(18)
DOUBLESHIFTGRAB_MATCH_64_DEF(19)
DOUBLESHIFTGRAB_MATCH_64_DEF(20)
DOUBLESHIFTGRAB_MATCH_64_DEF(21)
DOUBLESHIFTGRAB_MATCH_64_DEF(22)
DOUBLESHIFTGRAB_MATCH_64_DEF(23)
DOUBLESHIFTGRAB_MATCH_64_DEF(24)
DOUBLESHIFTGRAB_MATCH_64_DEF(25)
DOUBLESHIFTGRAB_MATCH_64_DEF(26)
DOUBLESHIFTGRAB_MATCH_64_DEF(27)
DOUBLESHIFTGRAB_MATCH_64_DEF(28)
DOUBLESHIFTGRAB_MATCH_64_DEF(29)
DOUBLESHIFTGRAB_MATCH_64_DEF(30)
DOUBLESHIFTGRAB_MATCH_64_DEF(31)

static
const UNUSED u8 * (*doubleshiftgrab_match_funcs_32[])(const u8 *buf, u32 z, u32 len2) =
{
// skip the first
    0,
    &doubleshiftgrabMatch_32_1,
    &doubleshiftgrabMatch_32_2,
    &doubleshiftgrabMatch_32_3,
    &doubleshiftgrabMatch_32_4,
    &doubleshiftgrabMatch_32_5,
    &doubleshiftgrabMatch_32_6,
    &doubleshiftgrabMatch_32_7,
    &doubleshiftgrabMatch_32_8,
    &doubleshiftgrabMatch_32_9,
    &doubleshiftgrabMatch_32_10,
    &doubleshiftgrabMatch_32_11,
    &doubleshiftgrabMatch_32_12,
    &doubleshiftgrabMatch_32_13,
    &doubleshiftgrabMatch_32_14,
    &doubleshiftgrabMatch_32_15,
};

static
const UNUSED u8 * (*doubleshiftgrab_match_funcs_64[])(const u8 *buf, u64a z, u32 len2) =
{
// skip the first
    0,
    &doubleshiftgrabMatch_64_1,
    &doubleshiftgrabMatch_64_2,
    &doubleshiftgrabMatch_64_3,
    &doubleshiftgrabMatch_64_4,
    &doubleshiftgrabMatch_64_5,
    &doubleshiftgrabMatch_64_6,
    &doubleshiftgrabMatch_64_7,
    &doubleshiftgrabMatch_64_8,
    &doubleshiftgrabMatch_64_9,
    &doubleshiftgrabMatch_64_10,
    &doubleshiftgrabMatch_64_11,
    &doubleshiftgrabMatch_64_12,
    &doubleshiftgrabMatch_64_13,
    &doubleshiftgrabMatch_64_14,
    &doubleshiftgrabMatch_64_15,
    &doubleshiftgrabMatch_64_16,
    &doubleshiftgrabMatch_64_17,
    &doubleshiftgrabMatch_64_18,
    &doubleshiftgrabMatch_64_19,
    &doubleshiftgrabMatch_64_20,
    &doubleshiftgrabMatch_64_21,
    &doubleshiftgrabMatch_64_22,
    &doubleshiftgrabMatch_64_23,
    &doubleshiftgrabMatch_64_24,
    &doubleshiftgrabMatch_64_25,
    &doubleshiftgrabMatch_64_26,
    &doubleshiftgrabMatch_64_27,
    &doubleshiftgrabMatch_64_28,
    &doubleshiftgrabMatch_64_29,
    &doubleshiftgrabMatch_64_30,
    &doubleshiftgrabMatch_64_31,
};

#endif /* MULTIACCEL_DOUBLESHIFTGRAB_H_ */
