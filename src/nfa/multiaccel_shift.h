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

#ifndef MULTIACCEL_SHIFT_H_
#define MULTIACCEL_SHIFT_H_

#include "multiaccel_common.h"

#define SHIFT_MATCH(len, match_t, match_sz) \
    static really_inline \
    const u8 * JOIN4(shiftMatch_, match_sz, _, len)(const u8 *buf, match_t z) {\
        if (unlikely(z)) { \
            z |= ((match_t) (1 << (len)) - 1) << (match_sz / 2); \
            VARISHIFT(z, z, len); \
            return JOIN(match, match_sz)(buf, z); \
        } \
        return NULL; \
    }

#define SHIFT_MATCH_32_DEF(n) \
        SHIFT_MATCH(n, u32, 32)
#define SHIFT_MATCH_64_DEF(n) \
        SHIFT_MATCH(n, u64a, 64)
#define SHIFT_MATCH_DEF(n) \
    SHIFT_MATCH_32_DEF(n) \
    SHIFT_MATCH_64_DEF(n)

SHIFT_MATCH_DEF(1)
SHIFT_MATCH_DEF(2)
SHIFT_MATCH_DEF(3)
SHIFT_MATCH_DEF(4)
SHIFT_MATCH_DEF(5)
SHIFT_MATCH_DEF(6)
SHIFT_MATCH_DEF(7)
SHIFT_MATCH_DEF(8)
SHIFT_MATCH_DEF(9)
SHIFT_MATCH_DEF(10)
SHIFT_MATCH_DEF(11)
SHIFT_MATCH_DEF(12)
SHIFT_MATCH_DEF(13)
SHIFT_MATCH_DEF(14)
SHIFT_MATCH_DEF(15)
SHIFT_MATCH_64_DEF(16)
SHIFT_MATCH_64_DEF(17)
SHIFT_MATCH_64_DEF(18)
SHIFT_MATCH_64_DEF(19)
SHIFT_MATCH_64_DEF(20)
SHIFT_MATCH_64_DEF(21)
SHIFT_MATCH_64_DEF(22)
SHIFT_MATCH_64_DEF(23)
SHIFT_MATCH_64_DEF(24)
SHIFT_MATCH_64_DEF(25)
SHIFT_MATCH_64_DEF(26)
SHIFT_MATCH_64_DEF(27)
SHIFT_MATCH_64_DEF(28)
SHIFT_MATCH_64_DEF(29)
SHIFT_MATCH_64_DEF(30)
SHIFT_MATCH_64_DEF(31)

static
const UNUSED u8 * (*shift_match_funcs_32[])(const u8 *buf, u32 z) =
{
// skip the first
   0,
   &shiftMatch_32_1,
   &shiftMatch_32_2,
   &shiftMatch_32_3,
   &shiftMatch_32_4,
   &shiftMatch_32_5,
   &shiftMatch_32_6,
   &shiftMatch_32_7,
   &shiftMatch_32_8,
   &shiftMatch_32_9,
   &shiftMatch_32_10,
   &shiftMatch_32_11,
   &shiftMatch_32_12,
   &shiftMatch_32_13,
   &shiftMatch_32_14,
   &shiftMatch_32_15,
};

static
const UNUSED u8 * (*shift_match_funcs_64[])(const u8 *buf, u64a z) =
{
// skip the first
    0,
    &shiftMatch_64_1,
    &shiftMatch_64_2,
    &shiftMatch_64_3,
    &shiftMatch_64_4,
    &shiftMatch_64_5,
    &shiftMatch_64_6,
    &shiftMatch_64_7,
    &shiftMatch_64_8,
    &shiftMatch_64_9,
    &shiftMatch_64_10,
    &shiftMatch_64_11,
    &shiftMatch_64_12,
    &shiftMatch_64_13,
    &shiftMatch_64_14,
    &shiftMatch_64_15,
    &shiftMatch_64_16,
    &shiftMatch_64_17,
    &shiftMatch_64_18,
    &shiftMatch_64_19,
    &shiftMatch_64_20,
    &shiftMatch_64_21,
    &shiftMatch_64_22,
    &shiftMatch_64_23,
    &shiftMatch_64_24,
    &shiftMatch_64_25,
    &shiftMatch_64_26,
    &shiftMatch_64_27,
    &shiftMatch_64_28,
    &shiftMatch_64_29,
    &shiftMatch_64_30,
    &shiftMatch_64_31,
};

#endif /* MULTIACCEL_SHIFT_H_ */
