/*
 * Copyright (c) 2015-2017, Intel Corporation
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

/** \file
 * \brief Teddy literal matcher: SSSE3 engine runtime.
 */

#include "fdr_internal.h"
#include "flood_runtime.h"
#include "teddy.h"
#include "teddy_internal.h"
#include "teddy_runtime_common.h"
#include "util/simd_utils.h"

const u8 ALIGN_DIRECTIVE p_mask_arr[17][32] = {
    {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
    {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
     0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
    {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
     0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
    {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
     0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
    {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
     0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
    {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
     0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
    {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
    {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
    {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
    {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
    {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
    {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff},
    {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff},
    {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff},
    {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff},
    {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff},
    {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
};

#if defined(__AVX2__) // reinforced teddy

#ifdef ARCH_64_BIT
#define CONFIRM_TEDDY(var, bucket, offset, reason, conf_fn)                 \
do {                                                                        \
    if (unlikely(diff256(var, ones256()))) {                                \
        m128 lo = movdq_lo(var);                                            \
        m128 hi = movdq_hi(var);                                            \
        u64a part1 = movq(lo);                                              \
        u64a part2 = movq(rshiftbyte_m128(lo, 8));                          \
        u64a part3 = movq(hi);                                              \
        u64a part4 = movq(rshiftbyte_m128(hi, 8));                          \
        if (unlikely(part1 != ones_u64a)) {                                 \
            part1 = ~part1;                                                 \
            conf_fn(&part1, bucket, offset, confBase, reason, a, ptr,       \
                    &control, &last_match);                                 \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
        if (unlikely(part2 != ones_u64a)) {                                 \
            part2 = ~part2;                                                 \
            conf_fn(&part2, bucket, offset + 8, confBase, reason, a, ptr,   \
                    &control, &last_match);                                 \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
        if (unlikely(part3 != ones_u64a)) {                                 \
            part3 = ~part3;                                                 \
            conf_fn(&part3, bucket, offset + 16, confBase, reason, a, ptr,  \
                    &control, &last_match);                                 \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
        if (unlikely(part4 != ones_u64a)) {                                 \
            part4 = ~part4;                                                 \
            conf_fn(&part4, bucket, offset + 24, confBase, reason, a, ptr,  \
                    &control, &last_match);                                 \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
    }                                                                       \
} while(0)
#else
#define CONFIRM_TEDDY(var, bucket, offset, reason, conf_fn)                 \
do {                                                                        \
    if (unlikely(diff256(var, ones256()))) {                                \
        m128 lo = movdq_lo(var);                                            \
        m128 hi = movdq_hi(var);                                            \
        u32 part1 = movd(lo);                                               \
        u32 part2 = movd(rshiftbyte_m128(lo, 4));                           \
        u32 part3 = movd(rshiftbyte_m128(lo, 8));                           \
        u32 part4 = movd(rshiftbyte_m128(lo, 12));                          \
        u32 part5 = movd(hi);                                               \
        u32 part6 = movd(rshiftbyte_m128(hi, 4));                           \
        u32 part7 = movd(rshiftbyte_m128(hi, 8));                           \
        u32 part8 = movd(rshiftbyte_m128(hi, 12));                          \
        if (unlikely(part1 != ones_u32)) {                                  \
            part1 = ~part1;                                                 \
            conf_fn(&part1, bucket, offset, confBase, reason, a, ptr,       \
                    &control, &last_match);                                 \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
        if (unlikely(part2 != ones_u32)) {                                  \
            part2 = ~part2;                                                 \
            conf_fn(&part2, bucket, offset + 4, confBase, reason, a, ptr,   \
                    &control, &last_match);                                 \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
        if (unlikely(part3 != ones_u32)) {                                  \
            part3 = ~part3;                                                 \
            conf_fn(&part3, bucket, offset + 8, confBase, reason, a, ptr,   \
                    &control, &last_match);                                 \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
        if (unlikely(part4 != ones_u32)) {                                  \
            part4 = ~part4;                                                 \
            conf_fn(&part4, bucket, offset + 12, confBase, reason, a, ptr,  \
                    &control, &last_match);                                 \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
        if (unlikely(part5 != ones_u32)) {                                  \
            part5 = ~part5;                                                 \
            conf_fn(&part5, bucket, offset + 16, confBase, reason, a, ptr,  \
                    &control, &last_match);                                 \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
        if (unlikely(part6 != ones_u32)) {                                  \
            part6 = ~part6;                                                 \
            conf_fn(&part6, bucket, offset + 20, confBase, reason, a, ptr,  \
                    &control, &last_match);                                 \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
        if (unlikely(part7 != ones_u32)) {                                  \
            part7 = ~part7;                                                 \
            conf_fn(&part7, bucket, offset + 24, confBase, reason, a, ptr,  \
                    &control, &last_match);                                 \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
        if (unlikely(part8 != ones_u32)) {                                  \
            part8 = ~part8;                                                 \
            conf_fn(&part8, bucket, offset + 28, confBase, reason, a, ptr,  \
                    &control, &last_match);                                 \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
    }                                                                       \
} while(0)
#endif

#define PREP_SHUF_MASK_NO_REINFORCEMENT(val)                                \
    m256 lo = and256(val, *lo_mask);                                        \
    m256 hi = and256(rshift64_m256(val, 4), *lo_mask)

#define PREP_SHUF_MASK                                                      \
    PREP_SHUF_MASK_NO_REINFORCEMENT(load256(ptr));                          \
    *c_128 = *(ptr + 15);                                                   \
    m256 r_msk = set64x4(0ULL, r_msk_base[*c_128], 0ULL, r_msk_base[*c_0]); \
    *c_0 = *(ptr + 31)

#define SHIFT_OR_M1                                                         \
    or256(pshufb_m256(dup_mask[0], lo), pshufb_m256(dup_mask[1], hi))

#define SHIFT_OR_M2                                                         \
    or256(lshift128_m256(or256(pshufb_m256(dup_mask[2], lo),                \
                               pshufb_m256(dup_mask[3], hi)),               \
                         1), SHIFT_OR_M1)

#define SHIFT_OR_M3                                                         \
    or256(lshift128_m256(or256(pshufb_m256(dup_mask[4], lo),                \
                               pshufb_m256(dup_mask[5], hi)),               \
                         2), SHIFT_OR_M2)

#define SHIFT_OR_M4                                                         \
    or256(lshift128_m256(or256(pshufb_m256(dup_mask[6], lo),                \
                               pshufb_m256(dup_mask[7], hi)),               \
                         3), SHIFT_OR_M3)

static really_inline
m256 prep_conf_teddy_no_reinforcement_m1(const m256 *lo_mask,
                                         const m256 *dup_mask,
                                         const m256 val) {
    PREP_SHUF_MASK_NO_REINFORCEMENT(val);
    return SHIFT_OR_M1;
}

static really_inline
m256 prep_conf_teddy_no_reinforcement_m2(const m256 *lo_mask,
                                         const m256 *dup_mask,
                                         const m256 val) {
    PREP_SHUF_MASK_NO_REINFORCEMENT(val);
    return SHIFT_OR_M2;
}

static really_inline
m256 prep_conf_teddy_no_reinforcement_m3(const m256 *lo_mask,
                                         const m256 *dup_mask,
                                         const m256 val) {
    PREP_SHUF_MASK_NO_REINFORCEMENT(val);
    return SHIFT_OR_M3;
}

static really_inline
m256 prep_conf_teddy_no_reinforcement_m4(const m256 *lo_mask,
                                         const m256 *dup_mask,
                                         const m256 val) {
    PREP_SHUF_MASK_NO_REINFORCEMENT(val);
    return SHIFT_OR_M4;
}

static really_inline
m256 prep_conf_teddy_m1(const m256 *lo_mask, const m256 *dup_mask,
                        const u8 *ptr, const u64a *r_msk_base,
                        u32 *c_0, u32 *c_128) {
    PREP_SHUF_MASK;
    return or256(SHIFT_OR_M1, r_msk);
}

static really_inline
m256 prep_conf_teddy_m2(const m256 *lo_mask, const m256 *dup_mask,
                        const u8 *ptr, const u64a *r_msk_base,
                        u32 *c_0, u32 *c_128) {
    PREP_SHUF_MASK;
    return or256(SHIFT_OR_M2, r_msk);
}

static really_inline
m256 prep_conf_teddy_m3(const m256 *lo_mask, const m256 *dup_mask,
                        const u8 *ptr, const u64a *r_msk_base,
                        u32 *c_0, u32 *c_128) {
    PREP_SHUF_MASK;
    return or256(SHIFT_OR_M3, r_msk);
}

static really_inline
m256 prep_conf_teddy_m4(const m256 *lo_mask, const m256 *dup_mask,
                        const u8 *ptr, const u64a *r_msk_base,
                        u32 *c_0, u32 *c_128) {
    PREP_SHUF_MASK;
    return or256(SHIFT_OR_M4, r_msk);
}

#else // not defined __AVX2__

#ifdef ARCH_64_BIT
#define CONFIRM_TEDDY(var, bucket, offset, reason, conf_fn)                 \
do {                                                                        \
    if (unlikely(diff128(var, ones128()))) {                                \
        u64a lo = movq(var);                                                \
        u64a hi = movq(rshiftbyte_m128(var, 8));                            \
        if (unlikely(lo != ones_u64a)) {                                    \
            lo = ~lo;                                                       \
            conf_fn(&lo, bucket, offset, confBase, reason, a, ptr,          \
                    &control, &last_match);                                 \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
        if (unlikely(hi != ones_u64a)) {                                    \
            hi = ~hi;                                                       \
            conf_fn(&hi, bucket, offset + 8, confBase, reason, a, ptr,      \
                    &control, &last_match);                                 \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
    }                                                                       \
} while(0)
#else
#define CONFIRM_TEDDY(var, bucket, offset, reason, conf_fn)                 \
do {                                                                        \
    if (unlikely(diff128(var, ones128()))) {                                \
        u32 part1 = movd(var);                                              \
        u32 part2 = movd(rshiftbyte_m128(var, 4));                          \
        u32 part3 = movd(rshiftbyte_m128(var, 8));                          \
        u32 part4 = movd(rshiftbyte_m128(var, 12));                         \
        if (unlikely(part1 != ones_u32)) {                                  \
            part1 = ~part1;                                                 \
            conf_fn(&part1, bucket, offset, confBase, reason, a, ptr,       \
                    &control, &last_match);                                 \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
        if (unlikely(part2 != ones_u32)) {                                  \
            part2 = ~part2;                                                 \
            conf_fn(&part2, bucket, offset + 4, confBase, reason, a, ptr,   \
                    &control, &last_match);                                 \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
        if (unlikely(part3 != ones_u32)) {                                  \
            part3 = ~part3;                                                 \
            conf_fn(&part3, bucket, offset + 8, confBase, reason, a, ptr,   \
                    &control, &last_match);                                 \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
        if (unlikely(part4 != ones_u32)) {                                  \
            part4 = ~part4;                                                 \
            conf_fn(&part4, bucket, offset + 12, confBase, reason, a, ptr,  \
                    &control, &last_match);                                 \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
    }                                                                       \
} while(0)
#endif

static really_inline
m128 prep_conf_teddy_m1(const m128 *maskBase, m128 val) {
    m128 mask = set16x8(0xf);
    m128 lo = and128(val, mask);
    m128 hi = and128(rshift64_m128(val, 4), mask);
    return or128(pshufb_m128(maskBase[0 * 2], lo),
                 pshufb_m128(maskBase[0 * 2 + 1], hi));
}

static really_inline
m128 prep_conf_teddy_m2(const m128 *maskBase, m128 *old_1, m128 val) {
    m128 mask = set16x8(0xf);
    m128 lo = and128(val, mask);
    m128 hi = and128(rshift64_m128(val, 4), mask);
    m128 r = prep_conf_teddy_m1(maskBase, val);

    m128 res_1 = or128(pshufb_m128(maskBase[1 * 2], lo),
                       pshufb_m128(maskBase[1 * 2 + 1], hi));
    m128 res_shifted_1 = palignr(res_1, *old_1, 16 - 1);
    *old_1 = res_1;
    return or128(r, res_shifted_1);
}

static really_inline
m128 prep_conf_teddy_m3(const m128 *maskBase, m128 *old_1, m128 *old_2,
                        m128 val) {
    m128 mask = set16x8(0xf);
    m128 lo = and128(val, mask);
    m128 hi = and128(rshift64_m128(val, 4), mask);
    m128 r = prep_conf_teddy_m2(maskBase, old_1, val);

    m128 res_2 = or128(pshufb_m128(maskBase[2 * 2], lo),
                       pshufb_m128(maskBase[2 * 2 + 1], hi));
    m128 res_shifted_2 = palignr(res_2, *old_2, 16 - 2);
    *old_2 = res_2;
    return or128(r, res_shifted_2);
}

static really_inline
m128 prep_conf_teddy_m4(const m128 *maskBase, m128 *old_1, m128 *old_2,
                        m128 *old_3, m128 val) {
    m128 mask = set16x8(0xf);
    m128 lo = and128(val, mask);
    m128 hi = and128(rshift64_m128(val, 4), mask);
    m128 r = prep_conf_teddy_m3(maskBase, old_1, old_2, val);

    m128 res_3 = or128(pshufb_m128(maskBase[3 * 2], lo),
                       pshufb_m128(maskBase[3 * 2 + 1], hi));
    m128 res_shifted_3 = palignr(res_3, *old_3, 16 - 3);
    *old_3 = res_3;
    return or128(r, res_shifted_3);
}

#endif // __AVX2__

#if defined(__AVX2__) // reinforced teddy

#define PREP_CONF_FN_NO_REINFORCEMENT(val, n)                                 \
    prep_conf_teddy_no_reinforcement_m##n(&lo_mask, dup_mask, val)

#define PREP_CONF_FN(ptr, n)                                                  \
    prep_conf_teddy_m##n(&lo_mask, dup_mask, ptr, r_msk_base, &c_0, &c_128)

#define PREPARE_MASKS_1                                                       \
    dup_mask[0] = set2x128(maskBase[0]);                                      \
    dup_mask[1] = set2x128(maskBase[1]);

#define PREPARE_MASKS_2                                                       \
    PREPARE_MASKS_1                                                           \
    dup_mask[2] = set2x128(maskBase[2]);                                      \
    dup_mask[3] = set2x128(maskBase[3]);

#define PREPARE_MASKS_3                                                       \
    PREPARE_MASKS_2                                                           \
    dup_mask[4] = set2x128(maskBase[4]);                                      \
    dup_mask[5] = set2x128(maskBase[5]);

#define PREPARE_MASKS_4                                                       \
    PREPARE_MASKS_3                                                           \
    dup_mask[6] = set2x128(maskBase[6]);                                      \
    dup_mask[7] = set2x128(maskBase[7]);

#define PREPARE_MASKS(n)                                                      \
    m256 lo_mask = set32x8(0xf);                                              \
    m256 dup_mask[n * 2];                                                     \
    PREPARE_MASKS_##n

#else // not defined __AVX2__

#define FDR_EXEC_TEDDY_RES_OLD_1

#define FDR_EXEC_TEDDY_RES_OLD_2                                              \
    m128 res_old_1 = zeroes128();

#define FDR_EXEC_TEDDY_RES_OLD_3                                              \
    m128 res_old_1 = zeroes128();                                             \
    m128 res_old_2 = zeroes128();

#define FDR_EXEC_TEDDY_RES_OLD_4                                              \
    m128 res_old_1 = zeroes128();                                             \
    m128 res_old_2 = zeroes128();                                             \
    m128 res_old_3 = zeroes128();

#define FDR_EXEC_TEDDY_RES_OLD(n) FDR_EXEC_TEDDY_RES_OLD_##n

#define PREP_CONF_FN_1(mask_base, val)                                        \
    prep_conf_teddy_m1(mask_base, val)

#define PREP_CONF_FN_2(mask_base, val)                                        \
    prep_conf_teddy_m2(mask_base, &res_old_1, val)

#define PREP_CONF_FN_3(mask_base, val)                                        \
    prep_conf_teddy_m3(mask_base, &res_old_1, &res_old_2, val)

#define PREP_CONF_FN_4(mask_base, val)                                        \
    prep_conf_teddy_m4(mask_base, &res_old_1, &res_old_2, &res_old_3, val)

#define PREP_CONF_FN(mask_base, val, n)                                       \
    PREP_CONF_FN_##n(mask_base, val)
#endif // __AVX2__


#if defined(__AVX2__) // reinforced teddy
#define FDR_EXEC_TEDDY(fdr, a, control, n_msk, conf_fn)                       \
do {                                                                          \
    const u8 *buf_end = a->buf + a->len;                                      \
    const u8 *ptr = a->buf + a->start_offset;                                 \
    u32 floodBackoff = FLOOD_BACKOFF_START;                                   \
    const u8 *tryFloodDetect = a->firstFloodDetect;                           \
    u32 last_match = (u32)-1;                                                 \
    const struct Teddy *teddy = (const struct Teddy *)fdr;                    \
    const size_t iterBytes = 64;                                              \
    DEBUG_PRINTF("params: buf %p len %zu start_offset %zu\n",                 \
                 a->buf, a->len, a->start_offset);                            \
                                                                              \
    const m128 *maskBase = getMaskBase(teddy);                                \
    PREPARE_MASKS(n_msk);                                                     \
    const u32 *confBase = getConfBase(teddy);                                 \
                                                                              \
    const u64a *r_msk_base = getReinforcedMaskBase(teddy, n_msk);             \
    u32 c_0 = 0x100;                                                          \
    u32 c_128 = 0x100;                                                        \
    const u8 *mainStart = ROUNDUP_PTR(ptr, 32);                               \
    DEBUG_PRINTF("derive: ptr: %p mainstart %p\n", ptr, mainStart);           \
    if (ptr < mainStart) {                                                    \
        ptr = mainStart - 32;                                                 \
        m256 p_mask;                                                          \
        m256 val_0 = vectoredLoad256(&p_mask, ptr, a->start_offset,           \
                                     a->buf, buf_end,                         \
                                     a->buf_history, a->len_history, n_msk);  \
        m256 r_0 = PREP_CONF_FN_NO_REINFORCEMENT(val_0, n_msk);               \
        c_0 = *(ptr + 31);                                                    \
        r_0 = or256(r_0, p_mask);                                             \
        CONFIRM_TEDDY(r_0, 8, 0, VECTORING, conf_fn);                         \
        ptr += 32;                                                            \
    }                                                                         \
                                                                              \
    if (ptr + 32 <= buf_end) {                                                \
        m256 r_0 = PREP_CONF_FN(ptr, n_msk);                                  \
        CONFIRM_TEDDY(r_0, 8, 0, VECTORING, conf_fn);                         \
        ptr += 32;                                                            \
    }                                                                         \
                                                                              \
    for (; ptr + iterBytes <= buf_end; ptr += iterBytes) {                    \
        __builtin_prefetch(ptr + (iterBytes * 4));                            \
        CHECK_FLOOD;                                                          \
        m256 r_0 = PREP_CONF_FN(ptr, n_msk);                                  \
        CONFIRM_TEDDY(r_0, 8, 0, NOT_CAUTIOUS, conf_fn);                      \
        m256 r_1 = PREP_CONF_FN(ptr + 32, n_msk);                             \
        CONFIRM_TEDDY(r_1, 8, 32, NOT_CAUTIOUS, conf_fn);                     \
    }                                                                         \
                                                                              \
    if (ptr + 32 <= buf_end) {                                                \
        m256 r_0 = PREP_CONF_FN(ptr, n_msk);                                  \
        CONFIRM_TEDDY(r_0, 8, 0, NOT_CAUTIOUS, conf_fn);                      \
        ptr += 32;                                                            \
    }                                                                         \
                                                                              \
    assert(ptr + 32 > buf_end);                                               \
    if (ptr < buf_end) {                                                      \
        m256 p_mask;                                                          \
        m256 val_0 = vectoredLoad256(&p_mask, ptr, 0, ptr, buf_end,           \
                                     a->buf_history, a->len_history, n_msk);  \
        m256 r_0 = PREP_CONF_FN_NO_REINFORCEMENT(val_0, n_msk);               \
        r_0 = or256(r_0, p_mask);                                             \
        CONFIRM_TEDDY(r_0, 8, 0, VECTORING, conf_fn);                         \
    }                                                                         \
                                                                              \
    return HWLM_SUCCESS;                                                      \
} while(0)
#else // not defined __AVX2__
#define FDR_EXEC_TEDDY(fdr, a, control, n_msk, conf_fn)                       \
do {                                                                          \
    const u8 *buf_end = a->buf + a->len;                                      \
    const u8 *ptr = a->buf + a->start_offset;                                 \
    u32 floodBackoff = FLOOD_BACKOFF_START;                                   \
    const u8 *tryFloodDetect = a->firstFloodDetect;                           \
    u32 last_match = (u32)-1;                                                 \
    const struct Teddy *teddy = (const struct Teddy *)fdr;                    \
    const size_t iterBytes = 32;                                              \
    DEBUG_PRINTF("params: buf %p len %zu start_offset %zu\n",                 \
                 a->buf, a->len, a->start_offset);                            \
                                                                              \
    const m128 *maskBase = getMaskBase(teddy);                                \
    const u32 *confBase = getConfBase(teddy);                                 \
                                                                              \
    FDR_EXEC_TEDDY_RES_OLD(n_msk);                                            \
    const u8 *mainStart = ROUNDUP_PTR(ptr, 16);                               \
    DEBUG_PRINTF("derive: ptr: %p mainstart %p\n", ptr, mainStart);           \
    if (ptr < mainStart) {                                                    \
        ptr = mainStart - 16;                                                 \
        m128 p_mask;                                                          \
        m128 val_0 = vectoredLoad128(&p_mask, ptr, a->start_offset,           \
                                     a->buf, buf_end,                         \
                                     a->buf_history, a->len_history, n_msk);  \
        m128 r_0 = PREP_CONF_FN(maskBase, val_0, n_msk);                      \
        r_0 = or128(r_0, p_mask);                                             \
        CONFIRM_TEDDY(r_0, 8, 0, VECTORING, conf_fn);                         \
        ptr += 16;                                                            \
    }                                                                         \
                                                                              \
    if (ptr + 16 <= buf_end) {                                                \
        m128 r_0 = PREP_CONF_FN(maskBase, load128(ptr), n_msk);               \
        CONFIRM_TEDDY(r_0, 8, 0, VECTORING, conf_fn);                         \
        ptr += 16;                                                            \
    }                                                                         \
                                                                              \
    for (; ptr + iterBytes <= buf_end; ptr += iterBytes) {                    \
        __builtin_prefetch(ptr + (iterBytes * 4));                            \
        CHECK_FLOOD;                                                          \
        m128 r_0 = PREP_CONF_FN(maskBase, load128(ptr), n_msk);               \
        CONFIRM_TEDDY(r_0, 8, 0, NOT_CAUTIOUS, conf_fn);                      \
        m128 r_1 = PREP_CONF_FN(maskBase, load128(ptr + 16), n_msk);          \
        CONFIRM_TEDDY(r_1, 8, 16, NOT_CAUTIOUS, conf_fn);                     \
    }                                                                         \
                                                                              \
    if (ptr + 16 <= buf_end) {                                                \
        m128 r_0 = PREP_CONF_FN(maskBase, load128(ptr), n_msk);               \
        CONFIRM_TEDDY(r_0, 8, 0, NOT_CAUTIOUS, conf_fn);                      \
        ptr += 16;                                                            \
    }                                                                         \
                                                                              \
    assert(ptr + 16 > buf_end);                                               \
    if (ptr < buf_end) {                                                      \
        m128 p_mask;                                                          \
        m128 val_0 = vectoredLoad128(&p_mask, ptr, 0, ptr, buf_end,           \
                                     a->buf_history, a->len_history, n_msk);  \
        m128 r_0 = PREP_CONF_FN(maskBase, val_0, n_msk);                      \
        r_0 = or128(r_0, p_mask);                                             \
        CONFIRM_TEDDY(r_0, 8, 0, VECTORING, conf_fn);                         \
    }                                                                         \
                                                                              \
    return HWLM_SUCCESS;                                                      \
} while(0)
#endif // __AVX2__

hwlm_error_t fdr_exec_teddy_msks1(const struct FDR *fdr,
                                  const struct FDR_Runtime_Args *a,
                                  hwlm_group_t control) {
    FDR_EXEC_TEDDY(fdr, a, control, 1, do_confWithBit_teddy);
}

hwlm_error_t fdr_exec_teddy_msks1_pck(const struct FDR *fdr,
                                      const struct FDR_Runtime_Args *a,
                                      hwlm_group_t control) {
    FDR_EXEC_TEDDY(fdr, a, control, 1, do_confWithBit_teddy);
}

hwlm_error_t fdr_exec_teddy_msks2(const struct FDR *fdr,
                                  const struct FDR_Runtime_Args *a,
                                  hwlm_group_t control) {
    FDR_EXEC_TEDDY(fdr, a, control, 2, do_confWithBit_teddy);
}

hwlm_error_t fdr_exec_teddy_msks2_pck(const struct FDR *fdr,
                                      const struct FDR_Runtime_Args *a,
                                      hwlm_group_t control) {
    FDR_EXEC_TEDDY(fdr, a, control, 2, do_confWithBit_teddy);
}

hwlm_error_t fdr_exec_teddy_msks3(const struct FDR *fdr,
                                  const struct FDR_Runtime_Args *a,
                                  hwlm_group_t control) {
    FDR_EXEC_TEDDY(fdr, a, control, 3, do_confWithBit_teddy);
}

hwlm_error_t fdr_exec_teddy_msks3_pck(const struct FDR *fdr,
                                      const struct FDR_Runtime_Args *a,
                                      hwlm_group_t control) {
    FDR_EXEC_TEDDY(fdr, a, control, 3, do_confWithBit_teddy);
}

hwlm_error_t fdr_exec_teddy_msks4(const struct FDR *fdr,
                                  const struct FDR_Runtime_Args *a,
                                  hwlm_group_t control) {
    FDR_EXEC_TEDDY(fdr, a, control, 4, do_confWithBit_teddy);
}

hwlm_error_t fdr_exec_teddy_msks4_pck(const struct FDR *fdr,
                                      const struct FDR_Runtime_Args *a,
                                      hwlm_group_t control) {
    FDR_EXEC_TEDDY(fdr, a, control, 4, do_confWithBit_teddy);
}
