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

#ifndef MULTIACCEL_SHIFTGRAB_H_
#define MULTIACCEL_SHIFTGRAB_H_

#include "multiaccel_common.h"

#define SHIFTGRAB_MATCH(len, match_t, match_sz) \
    static really_inline \
    const u8 * JOIN4(shiftgrabMatch_, match_sz, _, len)(const u8 *buf, match_t z) {\
        if (unlikely(z)) { \
            match_t tmp = ~z; \
            z |= ((match_t) (1 << (len)) - 1) << (match_sz / 2); \
            tmp |= ((match_t) (1 << len) - 1) << (match_sz / 2); \
            VARISHIFT(z, z, len); \
            VARISHIFT(tmp, z, 1); \
            return JOIN(match, match_sz)(buf, z); \
        } \
        return NULL; \
    }

#define SHIFTGRAB_MATCH_32_DEF(n) \
        SHIFTGRAB_MATCH(n, u32, 32)
#define SHIFTGRAB_MATCH_64_DEF(n) \
        SHIFTGRAB_MATCH(n, u64a, 64)
#define SHIFTGRAB_MATCH_DEF(n) \
    SHIFTGRAB_MATCH_32_DEF(n) \
    SHIFTGRAB_MATCH_64_DEF(n)

SHIFTGRAB_MATCH_DEF(1)
SHIFTGRAB_MATCH_DEF(2)
SHIFTGRAB_MATCH_DEF(3)
SHIFTGRAB_MATCH_DEF(4)
SHIFTGRAB_MATCH_DEF(5)
SHIFTGRAB_MATCH_DEF(6)
SHIFTGRAB_MATCH_DEF(7)
SHIFTGRAB_MATCH_DEF(8)
SHIFTGRAB_MATCH_DEF(9)
SHIFTGRAB_MATCH_DEF(10)
SHIFTGRAB_MATCH_DEF(11)
SHIFTGRAB_MATCH_DEF(12)
SHIFTGRAB_MATCH_DEF(13)
SHIFTGRAB_MATCH_DEF(14)
SHIFTGRAB_MATCH_DEF(15)
SHIFTGRAB_MATCH_64_DEF(16)
SHIFTGRAB_MATCH_64_DEF(17)
SHIFTGRAB_MATCH_64_DEF(18)
SHIFTGRAB_MATCH_64_DEF(19)
SHIFTGRAB_MATCH_64_DEF(20)
SHIFTGRAB_MATCH_64_DEF(21)
SHIFTGRAB_MATCH_64_DEF(22)
SHIFTGRAB_MATCH_64_DEF(23)
SHIFTGRAB_MATCH_64_DEF(24)
SHIFTGRAB_MATCH_64_DEF(25)
SHIFTGRAB_MATCH_64_DEF(26)
SHIFTGRAB_MATCH_64_DEF(27)
SHIFTGRAB_MATCH_64_DEF(28)
SHIFTGRAB_MATCH_64_DEF(29)
SHIFTGRAB_MATCH_64_DEF(30)
SHIFTGRAB_MATCH_64_DEF(31)

static
const UNUSED u8 * (*shiftgrab_match_funcs_32[])(const u8 *buf, u32 z) =
{
// skip the first
    0,
    &shiftgrabMatch_32_1,
    &shiftgrabMatch_32_2,
    &shiftgrabMatch_32_3,
    &shiftgrabMatch_32_4,
    &shiftgrabMatch_32_5,
    &shiftgrabMatch_32_6,
    &shiftgrabMatch_32_7,
    &shiftgrabMatch_32_8,
    &shiftgrabMatch_32_9,
    &shiftgrabMatch_32_10,
    &shiftgrabMatch_32_11,
    &shiftgrabMatch_32_12,
    &shiftgrabMatch_32_13,
    &shiftgrabMatch_32_14,
    &shiftgrabMatch_32_15,
};

static
const UNUSED u8 * (*shiftgrab_match_funcs_64[])(const u8 *buf, u64a z) =
                                                       {
// skip the first
    0,
    &shiftgrabMatch_64_1,
    &shiftgrabMatch_64_2,
    &shiftgrabMatch_64_3,
    &shiftgrabMatch_64_4,
    &shiftgrabMatch_64_5,
    &shiftgrabMatch_64_6,
    &shiftgrabMatch_64_7,
    &shiftgrabMatch_64_8,
    &shiftgrabMatch_64_9,
    &shiftgrabMatch_64_10,
    &shiftgrabMatch_64_11,
    &shiftgrabMatch_64_12,
    &shiftgrabMatch_64_13,
    &shiftgrabMatch_64_14,
    &shiftgrabMatch_64_15,
    &shiftgrabMatch_64_16,
    &shiftgrabMatch_64_17,
    &shiftgrabMatch_64_18,
    &shiftgrabMatch_64_19,
    &shiftgrabMatch_64_20,
    &shiftgrabMatch_64_21,
    &shiftgrabMatch_64_22,
    &shiftgrabMatch_64_23,
    &shiftgrabMatch_64_24,
    &shiftgrabMatch_64_25,
    &shiftgrabMatch_64_26,
    &shiftgrabMatch_64_27,
    &shiftgrabMatch_64_28,
    &shiftgrabMatch_64_29,
    &shiftgrabMatch_64_30,
    &shiftgrabMatch_64_31,
};

#endif /* MULTIACCEL_SHIFTGRAB_H_ */
