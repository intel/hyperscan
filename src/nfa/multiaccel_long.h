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

#ifndef MULTIACCEL_LONG_H_
#define MULTIACCEL_LONG_H_

#include "multiaccel_common.h"

#define LONG_MATCH(len, match_t, match_sz) \
    static really_inline \
    const u8 * JOIN4(longMatch_, match_sz, _, len)(const u8 *buf, match_t z) { \
        if (unlikely(z)) { \
            z |= ((match_t) (1 << (len - 1)) - 1) << (match_sz / 2); \
            JOIN(SHIFT, len)(z); \
            return JOIN(match, match_sz)(buf, z); \
        } \
        return NULL; \
    }

#define LONG_MATCH_32_DEF(n) \
        LONG_MATCH(n, u32, 32)
#define LONG_MATCH_64_DEF(n) \
        LONG_MATCH(n, u64a, 64)
#define LONG_MATCH_DEF(n) \
    LONG_MATCH_32_DEF(n) \
    LONG_MATCH_64_DEF(n)

LONG_MATCH_DEF(1)
LONG_MATCH_DEF(2)
LONG_MATCH_DEF(3)
LONG_MATCH_DEF(4)
LONG_MATCH_DEF(5)
LONG_MATCH_DEF(6)
LONG_MATCH_DEF(7)
LONG_MATCH_DEF(8)
LONG_MATCH_DEF(9)
LONG_MATCH_DEF(10)
LONG_MATCH_DEF(11)
LONG_MATCH_DEF(12)
LONG_MATCH_DEF(13)
LONG_MATCH_DEF(14)
LONG_MATCH_DEF(15)
LONG_MATCH_64_DEF(16)
LONG_MATCH_64_DEF(17)
LONG_MATCH_64_DEF(18)
LONG_MATCH_64_DEF(19)
LONG_MATCH_64_DEF(20)
LONG_MATCH_64_DEF(21)
LONG_MATCH_64_DEF(22)
LONG_MATCH_64_DEF(23)
LONG_MATCH_64_DEF(24)
LONG_MATCH_64_DEF(25)
LONG_MATCH_64_DEF(26)
LONG_MATCH_64_DEF(27)
LONG_MATCH_64_DEF(28)
LONG_MATCH_64_DEF(29)
LONG_MATCH_64_DEF(30)
LONG_MATCH_64_DEF(31)

static
const UNUSED u8 *(*long_match_funcs_32[])(const u8 *buf, u32 z) =
{
    // skip the first three
     0,
     &longMatch_32_1,
     &longMatch_32_2,
     &longMatch_32_3,
     &longMatch_32_4,
     &longMatch_32_5,
     &longMatch_32_6,
     &longMatch_32_7,
     &longMatch_32_8,
     &longMatch_32_9,
     &longMatch_32_10,
     &longMatch_32_11,
     &longMatch_32_12,
     &longMatch_32_13,
     &longMatch_32_14,
     &longMatch_32_15,
 };

static
const UNUSED u8 *(*long_match_funcs_64[])(const u8 *buf, u64a z) =
{
// skip the first three
    0,
    &longMatch_64_1,
    &longMatch_64_2,
    &longMatch_64_3,
    &longMatch_64_4,
    &longMatch_64_5,
    &longMatch_64_6,
    &longMatch_64_7,
    &longMatch_64_8,
    &longMatch_64_9,
    &longMatch_64_10,
    &longMatch_64_11,
    &longMatch_64_12,
    &longMatch_64_13,
    &longMatch_64_14,
    &longMatch_64_15,
    &longMatch_64_16,
    &longMatch_64_17,
    &longMatch_64_18,
    &longMatch_64_19,
    &longMatch_64_20,
    &longMatch_64_21,
    &longMatch_64_22,
    &longMatch_64_23,
    &longMatch_64_24,
    &longMatch_64_25,
    &longMatch_64_26,
    &longMatch_64_27,
    &longMatch_64_28,
    &longMatch_64_29,
    &longMatch_64_30,
    &longMatch_64_31,
};

#endif /* MULTIACCEL_LONG_H_ */
