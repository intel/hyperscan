/*
 * Copyright (c) 2016-2020, Intel Corporation
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

#ifndef VALIDATE_MASK_H
#define VALIDATE_MASK_H

#include "ue2common.h"
#include "util/simd_utils.h"

#if defined(DEBUG)
static
void validateMask32Print(const u8 *mask) {
    int i;
    for (i = 0; i < 32; i++) {
        printf("%02x", mask[i]);
    }
    printf("\n");
}

#ifdef HAVE_AVX512
static
void validateMask64Print(const u8 *mask) {
    int i;
    for (i = 0; i < 64; i++) {
        printf("%02x ", mask[i]);
    }
    printf("\n");
}
#endif
#endif

// check positive bytes in cmp_result.
// return one if the check passed, zero otherwise.
static really_inline
int posValidateMask(const u64a cmp_result, const u64a pos_mask) {
    return !(cmp_result & pos_mask);
}

/*
 * check negative bytes in cmp_result.
 * return one if any byte in cmp_result is not 0, zero otherwise.
 * check lowest 7 bits and highest bit of every byte respectively.
 */
static really_inline
int negValidateMask(const u64a cmp_result, const u64a neg_mask) {
    const u64a count_mask = 0x7f7f7f7f7f7f7f7f;
    // check lowest 7 bits of every byte.
    // the highest bit should be 1 if check passed.
    u64a check_low = (cmp_result & count_mask) + count_mask;
    // check the highest bit of every byte.
    // combine the highest bit and 0x7f to 0xff if check passes.
    // flip all 0xff to 0x00 and 0x7f to 0x80.
    u64a check_all = ~(check_low | cmp_result | count_mask);
    return !(check_all & neg_mask);
}

static really_inline
int validateMask(u64a data, u64a valid_data_mask, u64a and_mask,
                 u64a cmp_mask, u64a neg_mask) {
    // skip some byte where valid_data_mask is 0x00 there.
    and_mask &= valid_data_mask;
    cmp_mask &= valid_data_mask;
    neg_mask &= valid_data_mask;
    u64a cmp_result = (data & and_mask) ^ cmp_mask;
    /* do the positive check first since it's cheaper */
    if (posValidateMask(cmp_result, ~neg_mask)
        && negValidateMask(cmp_result, neg_mask)) {
        return 1;
    } else {
        DEBUG_PRINTF("data %llx valid_data_mask(vdm) %llx\n",
                     data, valid_data_mask);
        DEBUG_PRINTF("and_mask & vdm %llx cmp_mask & vdm %llx\n", and_mask,
                     cmp_mask);
        DEBUG_PRINTF("cmp_result %llx neg_mask & vdm %llx\n",
                     cmp_result, neg_mask);
        return 0;
    }
}

static really_inline
int validateMask32(const m256 data, const u32 valid_data_mask,
                   const m256 and_mask, const m256 cmp_mask,
                   const u32 neg_mask) {
    m256 cmp_result_256 = eq256(and256(data, and_mask), cmp_mask);
    u32 cmp_result = ~movemask256(cmp_result_256);
#ifdef DEBUG
    DEBUG_PRINTF("data\n");
    validateMask32Print((const u8 *)&data);
    DEBUG_PRINTF("cmp_result\n");
    validateMask32Print((const u8 *)&cmp_result_256);
#endif
    DEBUG_PRINTF("cmp_result %08x neg_mask %08x\n", cmp_result, neg_mask);
    DEBUG_PRINTF("valid_data_mask %08x\n", valid_data_mask);

    if ((cmp_result & valid_data_mask) == (neg_mask & valid_data_mask)) {
        DEBUG_PRINTF("checkCompareResult32 passed\n");
        return 1;
    } else {
        DEBUG_PRINTF("checkCompareResult32 failed\n");
        return 0;
    }
}

#ifdef HAVE_AVX512
static really_inline
int validateMask64(const m512 data, const u64a valid_data_mask,
                   const m512 and_mask, const m512 cmp_mask,
                   const u64a neg_mask) {
    u64a cmp_result = ~eq512mask(and512(data, and_mask), cmp_mask);
#ifdef DEBUG
    DEBUG_PRINTF("data\n");
    validateMask64Print((const u8 *)&data);
    DEBUG_PRINTF("cmp_result\n");
    validateMask64Print((const u8 *)&cmp_result);
#endif
    DEBUG_PRINTF("cmp_result %016llx neg_mask %016llx\n", cmp_result, neg_mask);
    DEBUG_PRINTF("valid_data_mask %016llx\n", valid_data_mask);

    if ((cmp_result & valid_data_mask) == (neg_mask & valid_data_mask)) {
        DEBUG_PRINTF("checkCompareResult64 passed\n");
        return 1;
    } else {
        DEBUG_PRINTF("checkCompareResult64 failed\n");
        return 0;
    }
}
#endif

#endif
