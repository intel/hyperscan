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

#ifndef MULTIACCEL_LONGGRAB_H_
#define MULTIACCEL_LONGGRAB_H_

#include "multiaccel_common.h"

#define LONGGRAB_MATCH(len, match_t, match_sz) \
    static really_inline \
    const u8 * JOIN4(longgrabMatch_, match_sz, _, len)(const u8 *buf, match_t z) { \
        if (unlikely(z)) { \
            match_t tmp = ~z; \
            tmp |= ((match_t) (1 << len) - 1) << (match_sz / 2); \
            z |= ((match_t) (1 << (len - 1)) - 1) << (match_sz / 2); \
            JOIN(SHIFT, len)(z); \
            VARISHIFT(tmp, z, len); \
            return JOIN(match, match_sz)(buf, z); \
        } \
        return NULL; \
    }

#define LONGGRAB_MATCH_32_DEF(n) \
        LONGGRAB_MATCH(n, u32, 32)
#define LONGGRAB_MATCH_64_DEF(n) \
        LONGGRAB_MATCH(n, u64a, 64)
#define LONGGRAB_MATCH_DEF(n) \
    LONGGRAB_MATCH_32_DEF(n) \
    LONGGRAB_MATCH_64_DEF(n)

LONGGRAB_MATCH_DEF(1)
LONGGRAB_MATCH_DEF(2)
LONGGRAB_MATCH_DEF(3)
LONGGRAB_MATCH_DEF(4)
LONGGRAB_MATCH_DEF(5)
LONGGRAB_MATCH_DEF(6)
LONGGRAB_MATCH_DEF(7)
LONGGRAB_MATCH_DEF(8)
LONGGRAB_MATCH_DEF(9)
LONGGRAB_MATCH_DEF(10)
LONGGRAB_MATCH_DEF(11)
LONGGRAB_MATCH_DEF(12)
LONGGRAB_MATCH_DEF(13)
LONGGRAB_MATCH_DEF(14)
LONGGRAB_MATCH_DEF(15)
LONGGRAB_MATCH_64_DEF(16)
LONGGRAB_MATCH_64_DEF(17)
LONGGRAB_MATCH_64_DEF(18)
LONGGRAB_MATCH_64_DEF(19)
LONGGRAB_MATCH_64_DEF(20)
LONGGRAB_MATCH_64_DEF(21)
LONGGRAB_MATCH_64_DEF(22)
LONGGRAB_MATCH_64_DEF(23)
LONGGRAB_MATCH_64_DEF(24)
LONGGRAB_MATCH_64_DEF(25)
LONGGRAB_MATCH_64_DEF(26)
LONGGRAB_MATCH_64_DEF(27)
LONGGRAB_MATCH_64_DEF(28)
LONGGRAB_MATCH_64_DEF(29)
LONGGRAB_MATCH_64_DEF(30)
LONGGRAB_MATCH_64_DEF(31)

static
const UNUSED u8 *(*longgrab_match_funcs_32[])(const u8 *buf, u32 z) =
{
// skip the first three
     0,
     &longgrabMatch_32_1,
     &longgrabMatch_32_2,
     &longgrabMatch_32_3,
     &longgrabMatch_32_4,
     &longgrabMatch_32_5,
     &longgrabMatch_32_6,
     &longgrabMatch_32_7,
     &longgrabMatch_32_8,
     &longgrabMatch_32_9,
     &longgrabMatch_32_10,
     &longgrabMatch_32_11,
     &longgrabMatch_32_12,
     &longgrabMatch_32_13,
     &longgrabMatch_32_14,
     &longgrabMatch_32_15,
 };

static
const UNUSED u8 *(*longgrab_match_funcs_64[])(const u8 *buf, u64a z) =
{
// skip the first three
    0,
    &longgrabMatch_64_1,
    &longgrabMatch_64_2,
    &longgrabMatch_64_3,
    &longgrabMatch_64_4,
    &longgrabMatch_64_5,
    &longgrabMatch_64_6,
    &longgrabMatch_64_7,
    &longgrabMatch_64_8,
    &longgrabMatch_64_9,
    &longgrabMatch_64_10,
    &longgrabMatch_64_11,
    &longgrabMatch_64_12,
    &longgrabMatch_64_13,
    &longgrabMatch_64_14,
    &longgrabMatch_64_15,
    &longgrabMatch_64_16,
    &longgrabMatch_64_17,
    &longgrabMatch_64_18,
    &longgrabMatch_64_19,
    &longgrabMatch_64_20,
    &longgrabMatch_64_21,
    &longgrabMatch_64_22,
    &longgrabMatch_64_23,
    &longgrabMatch_64_24,
    &longgrabMatch_64_25,
    &longgrabMatch_64_26,
    &longgrabMatch_64_27,
    &longgrabMatch_64_28,
    &longgrabMatch_64_29,
    &longgrabMatch_64_30,
    &longgrabMatch_64_31,
};

#endif /* MULTIACCEL_LONGGRAB_H_ */
