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

#ifndef VALIDATE_SHUFTI_H
#define VALIDATE_SHUFTI_H

#include "ue2common.h"
#include "util/simd_utils.h"

#if defined(DEBUG)
static
void dumpMask(const void *mask, int len) {
    const u8 *c = (const u8 *)mask;
    for (int i = 0; i < len; i++) {
        printf("%02x", c[i]);
    }
    printf("\n");
}
#endif

static really_inline
int validateShuftiMask16x16(const m256 data, const m256 hi_mask,
                            const m256 lo_mask, const m256 and_mask,
                            const u32 neg_mask, const u32 valid_data_mask) {
    m256 low4bits = set32x8(0xf);
    m256 c_lo = pshufb_m256(lo_mask, and256(data, low4bits));
    m256 c_hi = pshufb_m256(hi_mask,
                            rshift64_m256(andnot256(low4bits, data), 4));
    m256 t = and256(c_lo, c_hi);
    u32 nresult = movemask256(eq256(and256(t, and_mask), zeroes256()));
#ifdef DEBUG
    DEBUG_PRINTF("data\n");
    dumpMask(&data, 32);
    DEBUG_PRINTF("hi_mask\n");
    dumpMask(&hi_mask, 32);
    DEBUG_PRINTF("lo_mask\n");
    dumpMask(&lo_mask, 32);
    DEBUG_PRINTF("c_lo\n");
    dumpMask(&c_lo, 32);
    DEBUG_PRINTF("c_hi\n");
    dumpMask(&c_hi, 32);
    DEBUG_PRINTF("and_mask\n");
    dumpMask(&and_mask, 32);
    DEBUG_PRINTF("nresult %x\n", nresult);
    DEBUG_PRINTF("valid_data_mask %x\n", valid_data_mask);
#endif
    u32 cmp_result = (((nresult >> 16) & nresult) ^ neg_mask) & valid_data_mask;
    return !cmp_result;
}

static really_inline
int validateShuftiMask16x8(const m128 data, const m256 nib_mask,
                           const m128 and_mask, const u32 neg_mask,
                           const u32 valid_data_mask) {
    m256 data_m256 = combine2x128(rshift64_m128(data, 4), data);
    m256 low4bits = set32x8(0xf);
    m256 c_nib = pshufb_m256(nib_mask, and256(data_m256, low4bits));
    m128 t = and128(movdq_hi(c_nib), movdq_lo(c_nib));
    m128 nresult = eq128(and128(t, and_mask), zeroes128());
#ifdef DEBUG
    DEBUG_PRINTF("data\n");
    dumpMask(&data_m256, 32);
    DEBUG_PRINTF("nib_mask\n");
    dumpMask(&nib_mask, 32);
    DEBUG_PRINTF("c_nib\n");
    dumpMask(&c_nib, 32);
    DEBUG_PRINTF("nresult\n");
    dumpMask(&nresult, 16);
    DEBUG_PRINTF("valid_data_mask %x\n", valid_data_mask);
#endif
    u32 cmp_result = (movemask128(nresult) ^ neg_mask) & valid_data_mask;
    return !cmp_result;
}

static really_inline
int validateShuftiMask32x8(const m256 data, const m256 hi_mask,
                           const m256 lo_mask, const m256 and_mask,
                           const u32 neg_mask, const u32 valid_data_mask) {
    m256 low4bits = set32x8(0xf);
    m256 c_lo = pshufb_m256(lo_mask, and256(data, low4bits));
    m256 c_hi = pshufb_m256(hi_mask,
                            rshift64_m256(andnot256(low4bits, data), 4));
    m256 t = and256(c_lo, c_hi);
    m256 nresult = eq256(and256(t, and_mask), zeroes256());
#ifdef DEBUG
    DEBUG_PRINTF("data\n");
    dumpMask(&data, 32);
    DEBUG_PRINTF("hi_mask\n");
    dumpMask(&hi_mask, 32);
    DEBUG_PRINTF("lo_mask\n");
    dumpMask(&lo_mask, 32);
    DEBUG_PRINTF("c_lo\n");
    dumpMask(&c_lo, 32);
    DEBUG_PRINTF("c_hi\n");
    dumpMask(&c_hi, 32);
    DEBUG_PRINTF("nresult\n");
    dumpMask(&nresult, 32);
    DEBUG_PRINTF("valid_data_mask %x\n", valid_data_mask);
#endif
    u32 cmp_result = (movemask256(nresult) ^ neg_mask) & valid_data_mask;
    return !cmp_result;
}

static really_inline
int validateShuftiMask32x16(const m256 data,
                            const m256 hi_mask_1, const m256 hi_mask_2,
                            const m256 lo_mask_1, const m256 lo_mask_2,
                            const m256 bucket_mask_hi,
                            const m256 bucket_mask_lo, const u32 neg_mask,
                            const u32 valid_data_mask) {
    m256 low4bits = set32x8(0xf);
    m256 data_lo = and256(data, low4bits);
    m256 data_hi = and256(rshift64_m256(data, 4), low4bits);
    m256 c_lo_1 = pshufb_m256(lo_mask_1, data_lo);
    m256 c_lo_2 = pshufb_m256(lo_mask_2, data_lo);
    m256 c_hi_1 = pshufb_m256(hi_mask_1, data_hi);
    m256 c_hi_2 = pshufb_m256(hi_mask_2, data_hi);
    m256 t1 = and256(c_lo_1, c_hi_1);
    m256 t2 = and256(c_lo_2, c_hi_2);
    m256 result = or256(and256(t1, bucket_mask_lo), and256(t2, bucket_mask_hi));
    u32 nresult = movemask256(eq256(result, zeroes256()));
#ifdef DEBUG
    DEBUG_PRINTF("data\n");
    dumpMask(&data, 32);
    DEBUG_PRINTF("data_lo\n");
    dumpMask(&data_lo, 32);
    DEBUG_PRINTF("data_hi\n");
    dumpMask(&data_hi, 32);
    DEBUG_PRINTF("hi_mask_1\n");
    dumpMask(&hi_mask_1, 16);
    DEBUG_PRINTF("hi_mask_2\n");
    dumpMask(&hi_mask_2, 16);
    DEBUG_PRINTF("lo_mask_1\n");
    dumpMask(&lo_mask_1, 16);
    DEBUG_PRINTF("lo_mask_2\n");
    dumpMask(&lo_mask_2, 16);
    DEBUG_PRINTF("c_lo_1\n");
    dumpMask(&c_lo_1, 32);
    DEBUG_PRINTF("c_lo_2\n");
    dumpMask(&c_lo_2, 32);
    DEBUG_PRINTF("c_hi_1\n");
    dumpMask(&c_hi_1, 32);
    DEBUG_PRINTF("c_hi_2\n");
    dumpMask(&c_hi_2, 32);
    DEBUG_PRINTF("result\n");
    dumpMask(&result, 32);
    DEBUG_PRINTF("valid_data_mask %x\n", valid_data_mask);
#endif
    u32 cmp_result = (nresult ^ neg_mask) & valid_data_mask;
    return !cmp_result;
}

#ifdef HAVE_AVX512
static really_inline
int validateShuftiMask64x8(const m512 data, const m512 hi_mask,
                           const m512 lo_mask, const m512 and_mask,
                           const u64a neg_mask, const u64a valid_data_mask) {
    m512 low4bits = set64x8(0xf);
    m512 c_lo = pshufb_m512(lo_mask, and512(data, low4bits));
    m512 c_hi = pshufb_m512(hi_mask,
                            rshift64_m512(andnot512(low4bits, data), 4));
    m512 t = and512(c_lo, c_hi);
    u64a nresult = eq512mask(and512(t, and_mask), zeroes512());
#ifdef DEBUG
    DEBUG_PRINTF("data\n");
    dumpMask(&data, 64);
    DEBUG_PRINTF("hi_mask\n");
    dumpMask(&hi_mask, 64);
    DEBUG_PRINTF("lo_mask\n");
    dumpMask(&lo_mask, 64);
    DEBUG_PRINTF("c_lo\n");
    dumpMask(&c_lo, 64);
    DEBUG_PRINTF("c_hi\n");
    dumpMask(&c_hi, 64);
    DEBUG_PRINTF("nresult %llx\n", nresult);
    DEBUG_PRINTF("valid_data_mask %llx\n", valid_data_mask);
#endif
    u64a cmp_result = (nresult ^ neg_mask) & valid_data_mask;
    return !cmp_result;
}

static really_inline
int validateShuftiMask64x16(const m512 data,
                            const m512 hi_mask_1, const m512 hi_mask_2,
                            const m512 lo_mask_1, const m512 lo_mask_2,
                            const m512 and_mask_hi, const m512 and_mask_lo,
                            const u64a neg_mask, const u64a valid_data_mask) {
    m512 low4bits = set64x8(0xf);
    m512 data_lo = and512(data, low4bits);
    m512 data_hi = and512(rshift64_m512(data, 4), low4bits);
    m512 c_lo_1 = pshufb_m512(lo_mask_1, data_lo);
    m512 c_lo_2 = pshufb_m512(lo_mask_2, data_lo);
    m512 c_hi_1 = pshufb_m512(hi_mask_1, data_hi);
    m512 c_hi_2 = pshufb_m512(hi_mask_2, data_hi);
    m512 t1 = and512(c_lo_1, c_hi_1);
    m512 t2 = and512(c_lo_2, c_hi_2);
    m512 result = or512(and512(t1, and_mask_lo), and512(t2, and_mask_hi));
    u64a nresult = eq512mask(result, zeroes512());
#ifdef DEBUG
    DEBUG_PRINTF("data\n");
    dumpMask(&data, 64);
    DEBUG_PRINTF("data_lo\n");
    dumpMask(&data_lo, 64);
    DEBUG_PRINTF("data_hi\n");
    dumpMask(&data_hi, 64);
    DEBUG_PRINTF("hi_mask_1\n");
    dumpMask(&hi_mask_1, 64);
    DEBUG_PRINTF("hi_mask_2\n");
    dumpMask(&hi_mask_2, 64);
    DEBUG_PRINTF("lo_mask_1\n");
    dumpMask(&lo_mask_1, 64);
    DEBUG_PRINTF("lo_mask_2\n");
    dumpMask(&lo_mask_2, 64);
    DEBUG_PRINTF("c_lo_1\n");
    dumpMask(&c_lo_1, 64);
    DEBUG_PRINTF("c_lo_2\n");
    dumpMask(&c_lo_2, 64);
    DEBUG_PRINTF("c_hi_1\n");
    dumpMask(&c_hi_1, 64);
    DEBUG_PRINTF("c_hi_2\n");
    dumpMask(&c_hi_2, 64);
    DEBUG_PRINTF("result\n");
    dumpMask(&result, 64);
    DEBUG_PRINTF("valid_data_mask %llx\n", valid_data_mask);
#endif
    u64a cmp_result = (nresult ^ neg_mask) & valid_data_mask;
    return !cmp_result;
}
#endif

static really_inline
int checkMultipath32(u32 data, u32 hi_bits, u32 lo_bits) {
    u32 t = ~(data | hi_bits);
    t += lo_bits;
    t &= (~data) & hi_bits;
    DEBUG_PRINTF("t %x\n", t);
    return !!t;
}

static really_inline
int checkMultipath64(u64a data, u64a hi_bits, u64a lo_bits) {
    u64a t = ~(data | hi_bits);
    t += lo_bits;
    t &= (~data) & hi_bits;
    DEBUG_PRINTF("t %llx\n", t);
    return !!t;
}

static really_inline
int validateMultipathShuftiMask16x8(const m128 data,
                                    const m256 nib_mask,
                                    const m128 bucket_select_mask,
                                    const u32 hi_bits, const u32 lo_bits,
                                    const u32 neg_mask,
                                    const u32 valid_path_mask) {
    m256 data_256 = combine2x128(rshift64_m128(data, 4), data);
    m256 low4bits = set32x8(0xf);
    m256 c_nib = pshufb_m256(nib_mask, and256(data_256, low4bits));
    m128 t = and128(movdq_hi(c_nib), movdq_lo(c_nib));
    m128 result = and128(t, bucket_select_mask);
    u32 nresult = movemask128(eq128(result, zeroes128()));
    u32 cmp_result = (nresult ^ neg_mask) | valid_path_mask;

    DEBUG_PRINTF("cmp_result %x\n", cmp_result);

    return checkMultipath32(cmp_result, hi_bits, lo_bits);
}

static really_inline
int validateMultipathShuftiMask32x8(const m256 data,
                                    const m256 hi_mask, const m256 lo_mask,
                                    const m256 bucket_select_mask,
                                    const u32 hi_bits, const u32 lo_bits,
                                    const u32 neg_mask,
                                    const u32 valid_path_mask) {
    m256 low4bits = set32x8(0xf);
    m256 data_lo = and256(data, low4bits);
    m256 data_hi = and256(rshift64_m256(data, 4), low4bits);
    m256 c_lo = pshufb_m256(lo_mask, data_lo);
    m256 c_hi = pshufb_m256(hi_mask, data_hi);
    m256 c = and256(c_lo, c_hi);
    m256 result = and256(c, bucket_select_mask);
    u32 nresult = movemask256(eq256(result, zeroes256()));
    u32 cmp_result = (nresult ^ neg_mask) | valid_path_mask;

    DEBUG_PRINTF("cmp_result %x\n", cmp_result);

    return checkMultipath32(cmp_result, hi_bits, lo_bits);
}

static really_inline
int validateMultipathShuftiMask32x16(const m256 data,
                                     const m256 hi_mask_1, const m256 hi_mask_2,
                                     const m256 lo_mask_1, const m256 lo_mask_2,
                                     const m256 bucket_select_mask_hi,
                                     const m256 bucket_select_mask_lo,
                                     const u32 hi_bits, const u32 lo_bits,
                                     const u32 neg_mask,
                                     const u32 valid_path_mask) {
    m256 low4bits = set32x8(0xf);
    m256 data_lo = and256(data, low4bits);
    m256 data_hi = and256(rshift64_m256(data, 4), low4bits);
    m256 c_lo_1 = pshufb_m256(lo_mask_1, data_lo);
    m256 c_lo_2 = pshufb_m256(lo_mask_2, data_lo);
    m256 c_hi_1 = pshufb_m256(hi_mask_1, data_hi);
    m256 c_hi_2 = pshufb_m256(hi_mask_2, data_hi);
    m256 t1 = and256(c_lo_1, c_hi_1);
    m256 t2 = and256(c_lo_2, c_hi_2);
    m256 result = or256(and256(t1, bucket_select_mask_lo),
                        and256(t2, bucket_select_mask_hi));
    u32 nresult = movemask256(eq256(result, zeroes256()));
    u32 cmp_result = (nresult ^ neg_mask) | valid_path_mask;

    DEBUG_PRINTF("cmp_result %x\n", cmp_result);

    return checkMultipath32(cmp_result, hi_bits, lo_bits);
}

static really_inline
int validateMultipathShuftiMask64(const m256 data_1, const m256 data_2,
                                  const m256 hi_mask, const m256 lo_mask,
                                  const m256 bucket_select_mask_1,
                                  const m256 bucket_select_mask_2,
                                  const u64a hi_bits, const u64a lo_bits,
                                  const u64a neg_mask,
                                  const u64a valid_path_mask) {
    m256 low4bits = set32x8(0xf);
    m256 c_lo_1 = pshufb_m256(lo_mask, and256(data_1, low4bits));
    m256 c_lo_2 = pshufb_m256(lo_mask, and256(data_2, low4bits));
    m256 c_hi_1 = pshufb_m256(hi_mask,
                              rshift64_m256(andnot256(low4bits, data_1), 4));
    m256 c_hi_2 = pshufb_m256(hi_mask,
                              rshift64_m256(andnot256(low4bits, data_2), 4));
    m256 t1 = and256(c_lo_1, c_hi_1);
    m256 t2 = and256(c_lo_2, c_hi_2);
    m256 nresult_1 = eq256(and256(t1, bucket_select_mask_1), zeroes256());
    m256 nresult_2 = eq256(and256(t2, bucket_select_mask_2), zeroes256());
    u64a nresult = (u64a)movemask256(nresult_1) |
                   (u64a)movemask256(nresult_2) << 32;
    u64a cmp_result = (nresult ^ neg_mask) | valid_path_mask;

    DEBUG_PRINTF("cmp_result %llx\n", cmp_result);

    return checkMultipath64(cmp_result, hi_bits, lo_bits);
}

#endif
