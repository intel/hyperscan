/*
 * Copyright (c) 2016-2017, Intel Corporation
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
 * \brief Teddy literal matcher: AVX2 engine runtime.
 */

#include "fdr_internal.h"
#include "flood_runtime.h"
#include "teddy.h"
#include "teddy_internal.h"
#include "teddy_runtime_common.h"
#include "util/arch.h"
#include "util/simd_utils.h"

#if defined(HAVE_AVX2)

#ifdef ARCH_64_BIT
#define CONFIRM_FAT_TEDDY(var, bucket, offset, reason, conf_fn)             \
do {                                                                        \
    if (unlikely(isnonzero256(var))) {                                      \
        m256 swap = swap128in256(var);                                      \
        m256 r = interleave256lo(var, swap);                                \
        u64a part1 = extractlow64from256(r);                                \
        u64a part2 = extract64from256(r, 1);                                \
        r = interleave256hi(var, swap);                                     \
        u64a part3 = extractlow64from256(r);                                \
        u64a part4 = extract64from256(r, 1);                                \
        if (unlikely(part1)) {                                              \
            conf_fn(&part1, bucket, offset, confBase, reason, a, ptr,       \
                    &control, &last_match);                                 \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
        if (unlikely(part2)) {                                              \
            conf_fn(&part2, bucket, offset + 4, confBase, reason, a, ptr,   \
                    &control, &last_match);                                 \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
        if (unlikely(part3)) {                                              \
            conf_fn(&part3, bucket, offset + 8, confBase, reason, a, ptr,   \
                    &control, &last_match);                                 \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
        if (unlikely(part4)) {                                              \
            conf_fn(&part4, bucket, offset + 12, confBase, reason, a, ptr,  \
                    &control, &last_match);                                 \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
    }                                                                       \
} while (0);
#else
#define CONFIRM_FAT_TEDDY(var, bucket, offset, reason, conf_fn)             \
do {                                                                        \
    if (unlikely(isnonzero256(var))) {                                      \
        m256 swap = swap128in256(var);                                      \
        m256 r = interleave256lo(var, swap);                                \
        u32 part1 = extractlow32from256(r);                                 \
        u32 part2 = extract32from256(r, 1);                                 \
        u32 part3 = extract32from256(r, 2);                                 \
        u32 part4 = extract32from256(r, 3);                                 \
        r = interleave256hi(var, swap);                                     \
        u32 part5 = extractlow32from256(r);                                 \
        u32 part6 = extract32from256(r, 1);                                 \
        u32 part7 = extract32from256(r, 2);                                 \
        u32 part8 = extract32from256(r, 3);                                 \
        if (unlikely(part1)) {                                              \
            conf_fn(&part1, bucket, offset, confBase, reason, a, ptr,       \
                    &control, &last_match);                                 \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
        if (unlikely(part2)) {                                              \
            conf_fn(&part2, bucket, offset + 2, confBase, reason, a, ptr,   \
                    &control, &last_match);                                 \
        }                                                                   \
        if (unlikely(part3)) {                                              \
            conf_fn(&part3, bucket, offset + 4, confBase, reason, a, ptr,   \
                    &control, &last_match);                                 \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
        if (unlikely(part4)) {                                              \
            conf_fn(&part4, bucket, offset + 6, confBase, reason, a, ptr,   \
                    &control, &last_match);                                 \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
        if (unlikely(part5)) {                                              \
            conf_fn(&part5, bucket, offset + 8, confBase, reason, a, ptr,   \
                    &control, &last_match);                                 \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
        if (unlikely(part6)) {                                              \
            conf_fn(&part6, bucket, offset + 10, confBase, reason, a, ptr,  \
                    &control, &last_match);                                 \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
        if (unlikely(part7)) {                                              \
            conf_fn(&part7, bucket, offset + 12, confBase, reason, a, ptr,  \
                    &control, &last_match);                                 \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
        if (unlikely(part8)) {                                              \
            conf_fn(&part8, bucket, offset + 14, confBase, reason, a, ptr,  \
                    &control, &last_match);                                 \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
    }                                                                       \
} while (0);
#endif

static really_inline
m256 vectoredLoad2x128(m256 *p_mask, const u8 *ptr, const u8 *lo, const u8 *hi,
                       const u8 *buf_history, size_t len_history,
                       const u32 nMasks) {
    m128 p_mask128;
    m256 ret = set2x128(vectoredLoad128(&p_mask128, ptr, lo, hi, buf_history,
                                        len_history, nMasks));
    *p_mask = set2x128(p_mask128);
    return ret;
}

static really_inline
m256 prep_conf_fat_teddy_m1(const m256 *maskBase, m256 val) {
    m256 mask = set32x8(0xf);
    m256 lo = and256(val, mask);
    m256 hi = and256(rshift64_m256(val, 4), mask);
    return and256(pshufb_m256(maskBase[0*2], lo),
                  pshufb_m256(maskBase[0*2+1], hi));
}

static really_inline
m256 prep_conf_fat_teddy_m2(const m256 *maskBase, m256 *old_1, m256 val) {
    m256 mask = set32x8(0xf);
    m256 lo = and256(val, mask);
    m256 hi = and256(rshift64_m256(val, 4), mask);
    m256 r = prep_conf_fat_teddy_m1(maskBase, val);

    m256 res_1 = and256(pshufb_m256(maskBase[1*2], lo),
                        pshufb_m256(maskBase[1*2+1], hi));
    m256 res_shifted_1 = vpalignr(res_1, *old_1, 16-1);
    *old_1 = res_1;
    return and256(r, res_shifted_1);
}

static really_inline
m256 prep_conf_fat_teddy_m3(const m256 *maskBase, m256 *old_1, m256 *old_2,
                            m256 val) {
    m256 mask = set32x8(0xf);
    m256 lo = and256(val, mask);
    m256 hi = and256(rshift64_m256(val, 4), mask);
    m256 r = prep_conf_fat_teddy_m2(maskBase, old_1, val);

    m256 res_2 = and256(pshufb_m256(maskBase[2*2], lo),
                        pshufb_m256(maskBase[2*2+1], hi));
    m256 res_shifted_2 = vpalignr(res_2, *old_2, 16-2);
    *old_2 = res_2;
    return and256(r, res_shifted_2);
}

static really_inline
m256 prep_conf_fat_teddy_m4(const m256 *maskBase, m256 *old_1, m256 *old_2,
                            m256 *old_3, m256 val) {
    m256 mask = set32x8(0xf);
    m256 lo = and256(val, mask);
    m256 hi = and256(rshift64_m256(val, 4), mask);
    m256 r = prep_conf_fat_teddy_m3(maskBase, old_1, old_2, val);

    m256 res_3 = and256(pshufb_m256(maskBase[3*2], lo),
                        pshufb_m256(maskBase[3*2+1], hi));
    m256 res_shifted_3 = vpalignr(res_3, *old_3, 16-3);
    *old_3 = res_3;
    return and256(r, res_shifted_3);
}

static really_inline
const m256 * getMaskBase_avx2(const struct Teddy *teddy) {
    return (const m256 *)((const u8 *)teddy + sizeof(struct Teddy));
}

static really_inline
const u32 * getConfBase_avx2(const struct Teddy *teddy, u8 numMask) {
    return (const u32 *)((const u8 *)teddy + sizeof(struct Teddy) +
                         (numMask*32*2));
}

hwlm_error_t fdr_exec_teddy_avx2_msks1_fat(const struct FDR *fdr,
                                           const struct FDR_Runtime_Args *a,
                                           hwlm_group_t control) {
    const u8 *buf_end = a->buf + a->len;
    const u8 *ptr = a->buf + a->start_offset;
    u32 floodBackoff = FLOOD_BACKOFF_START;
    const u8 *tryFloodDetect = a->firstFloodDetect;
    u32 last_match = (u32)-1;
    const struct Teddy *teddy = (const struct Teddy *)fdr;
    const size_t iterBytes = 32;
    DEBUG_PRINTF("params: buf %p len %zu start_offset %zu\n",
                 a->buf, a->len, a->start_offset);

    const m256 *maskBase = getMaskBase_avx2(teddy);
    const u32 *confBase = getConfBase_avx2(teddy, 1);

    const u8 *mainStart = ROUNDUP_PTR(ptr, 16);
    DEBUG_PRINTF("derive: ptr: %p mainstart %p\n", ptr, mainStart);
    if (ptr < mainStart) {
        ptr = mainStart - 16;
        m256 p_mask;
        m256 val_0 = vectoredLoad2x128(&p_mask, ptr, a->buf, buf_end,
                                       a->buf_history, a->len_history, 1);
        m256 r_0 = prep_conf_fat_teddy_m1(maskBase, val_0);
        r_0 = and256(r_0, p_mask);
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBit1_teddy);
        ptr += 16;
    }

    if (ptr + 16 < buf_end) {
        m256 r_0 = prep_conf_fat_teddy_m1(maskBase, load2x128(ptr));
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBit1_teddy);
        ptr += 16;
    }

    for ( ; ptr + iterBytes <= buf_end; ptr += iterBytes) {
        __builtin_prefetch(ptr + (iterBytes*4));
        CHECK_FLOOD;
        m256 r_0 = prep_conf_fat_teddy_m1(maskBase, load2x128(ptr));
        CONFIRM_FAT_TEDDY(r_0, 16, 0, NOT_CAUTIOUS, do_confWithBit1_teddy);
        m256 r_1 = prep_conf_fat_teddy_m1(maskBase, load2x128(ptr + 16));
        CONFIRM_FAT_TEDDY(r_1, 16, 16, NOT_CAUTIOUS, do_confWithBit1_teddy);
    }

    for (; ptr < buf_end; ptr += 16) {
        m256 p_mask;
        m256 val_0 = vectoredLoad2x128(&p_mask, ptr, a->buf, buf_end,
                                       a->buf_history, a->len_history, 1);
        m256 r_0 = prep_conf_fat_teddy_m1(maskBase, val_0);
        r_0 = and256(r_0, p_mask);
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBit1_teddy);
    }

    return HWLM_SUCCESS;
}

hwlm_error_t fdr_exec_teddy_avx2_msks1_pck_fat(const struct FDR *fdr,
                                               const struct FDR_Runtime_Args *a,
                                               hwlm_group_t control) {
    const u8 *buf_end = a->buf + a->len;
    const u8 *ptr = a->buf + a->start_offset;
    u32 floodBackoff = FLOOD_BACKOFF_START;
    const u8 *tryFloodDetect = a->firstFloodDetect;
    u32 last_match = (u32)-1;
    const struct Teddy *teddy = (const struct Teddy *)fdr;
    const size_t iterBytes = 32;
    DEBUG_PRINTF("params: buf %p len %zu start_offset %zu\n",
                 a->buf, a->len, a->start_offset);

    const m256 *maskBase = getMaskBase_avx2(teddy);
    const u32 *confBase = getConfBase_avx2(teddy, 1);

    const u8 *mainStart = ROUNDUP_PTR(ptr, 16);
    DEBUG_PRINTF("derive: ptr: %p mainstart %p\n", ptr, mainStart);
    if (ptr < mainStart) {
        ptr = mainStart - 16;
        m256 p_mask;
        m256 val_0 = vectoredLoad2x128(&p_mask, ptr, a->buf, buf_end,
                                       a->buf_history, a->len_history, 1);
        m256 r_0 = prep_conf_fat_teddy_m1(maskBase, val_0);
        r_0 = and256(r_0, p_mask);
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBit_teddy);
        ptr += 16;
    }

    if (ptr + 16 < buf_end) {
        m256 r_0 = prep_conf_fat_teddy_m1(maskBase, load2x128(ptr));
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBit_teddy);
        ptr += 16;
    }

    for ( ; ptr + iterBytes <= buf_end; ptr += iterBytes) {
        __builtin_prefetch(ptr + (iterBytes*4));
        CHECK_FLOOD;
        m256 r_0 = prep_conf_fat_teddy_m1(maskBase, load2x128(ptr));
        CONFIRM_FAT_TEDDY(r_0, 16, 0, NOT_CAUTIOUS, do_confWithBit_teddy);
        m256 r_1 = prep_conf_fat_teddy_m1(maskBase, load2x128(ptr + 16));
        CONFIRM_FAT_TEDDY(r_1, 16, 16, NOT_CAUTIOUS, do_confWithBit_teddy);
    }

    for (; ptr < buf_end; ptr += 16) {
        m256 p_mask;
        m256 val_0 = vectoredLoad2x128(&p_mask, ptr, a->buf, buf_end,
                                       a->buf_history, a->len_history, 1);
        m256 r_0 = prep_conf_fat_teddy_m1(maskBase, val_0);
        r_0 = and256(r_0, p_mask);
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBit_teddy);
    }

    return HWLM_SUCCESS;
}

hwlm_error_t fdr_exec_teddy_avx2_msks2_fat(const struct FDR *fdr,
                                           const struct FDR_Runtime_Args *a,
                                           hwlm_group_t control) {
    const u8 *buf_end = a->buf + a->len;
    const u8 *ptr = a->buf + a->start_offset;
    u32 floodBackoff = FLOOD_BACKOFF_START;
    const u8 *tryFloodDetect = a->firstFloodDetect;
    u32 last_match = (u32)-1;
    const struct Teddy *teddy = (const struct Teddy *)fdr;
    const size_t iterBytes = 32;
    DEBUG_PRINTF("params: buf %p len %zu start_offset %zu\n",
                 a->buf, a->len, a->start_offset);

    const m256 *maskBase = getMaskBase_avx2(teddy);
    const u32 *confBase = getConfBase_avx2(teddy, 2);

    m256 res_old_1 = ones256();
    const u8 *mainStart = ROUNDUP_PTR(ptr, 16);
    DEBUG_PRINTF("derive: ptr: %p mainstart %p\n", ptr, mainStart);
    if (ptr < mainStart) {
        ptr = mainStart - 16;
        m256 p_mask;
        m256 val_0 = vectoredLoad2x128(&p_mask, ptr, a->buf, buf_end,
                                       a->buf_history, a->len_history, 2);
        m256 r_0 = prep_conf_fat_teddy_m2(maskBase, &res_old_1, val_0);
        r_0 = and256(r_0, p_mask);
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBitMany_teddy);
        ptr += 16;
    }

    if (ptr + 16 < buf_end) {
        m256 r_0 = prep_conf_fat_teddy_m2(maskBase, &res_old_1, load2x128(ptr));
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBitMany_teddy);
        ptr += 16;
    }

    for ( ; ptr + iterBytes <= buf_end; ptr += iterBytes) {
        __builtin_prefetch(ptr + (iterBytes*4));
        CHECK_FLOOD;
        m256 r_0 = prep_conf_fat_teddy_m2(maskBase, &res_old_1, load2x128(ptr));
        CONFIRM_FAT_TEDDY(r_0, 16, 0, NOT_CAUTIOUS, do_confWithBitMany_teddy);
        m256 r_1 = prep_conf_fat_teddy_m2(maskBase, &res_old_1,
                                          load2x128(ptr + 16));
        CONFIRM_FAT_TEDDY(r_1, 16, 16, NOT_CAUTIOUS, do_confWithBitMany_teddy);
    }

    for (; ptr < buf_end; ptr += 16) {
        m256 p_mask;
        m256 val_0 = vectoredLoad2x128(&p_mask, ptr, a->buf, buf_end,
                                       a->buf_history, a->len_history, 2);
        m256 r_0 = prep_conf_fat_teddy_m2(maskBase, &res_old_1, val_0);
        r_0 = and256(r_0, p_mask);
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBitMany_teddy);
    }

    return HWLM_SUCCESS;
}

hwlm_error_t fdr_exec_teddy_avx2_msks2_pck_fat(const struct FDR *fdr,
                                               const struct FDR_Runtime_Args *a,
                                               hwlm_group_t control) {
    const u8 *buf_end = a->buf + a->len;
    const u8 *ptr = a->buf + a->start_offset;
    u32 floodBackoff = FLOOD_BACKOFF_START;
    const u8 *tryFloodDetect = a->firstFloodDetect;
    u32 last_match = (u32)-1;
    const struct Teddy *teddy = (const struct Teddy *)fdr;
    const size_t iterBytes = 32;
    DEBUG_PRINTF("params: buf %p len %zu start_offset %zu\n",
                 a->buf, a->len, a->start_offset);

    const m256 *maskBase = getMaskBase_avx2(teddy);
    const u32 *confBase = getConfBase_avx2(teddy, 2);

    m256 res_old_1 = ones256();
    const u8 *mainStart = ROUNDUP_PTR(ptr, 16);
    DEBUG_PRINTF("derive: ptr: %p mainstart %p\n", ptr, mainStart);
    if (ptr < mainStart) {
        ptr = mainStart - 16;
        m256 p_mask;
        m256 val_0 = vectoredLoad2x128(&p_mask, ptr, a->buf, buf_end,
                                       a->buf_history, a->len_history, 2);
        m256 r_0 = prep_conf_fat_teddy_m2(maskBase, &res_old_1, val_0);
        r_0 = and256(r_0, p_mask);
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBit_teddy);
        ptr += 16;
    }

    if (ptr + 16 < buf_end) {
        m256 r_0 = prep_conf_fat_teddy_m2(maskBase, &res_old_1, load2x128(ptr));
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBit_teddy);
        ptr += 16;
    }

    for ( ; ptr + iterBytes <= buf_end; ptr += iterBytes) {
        __builtin_prefetch(ptr + (iterBytes*4));
        CHECK_FLOOD;
        m256 r_0 = prep_conf_fat_teddy_m2(maskBase, &res_old_1, load2x128(ptr));
        CONFIRM_FAT_TEDDY(r_0, 16, 0, NOT_CAUTIOUS, do_confWithBit_teddy);
        m256 r_1 = prep_conf_fat_teddy_m2(maskBase, &res_old_1,
                                          load2x128(ptr + 16));
        CONFIRM_FAT_TEDDY(r_1, 16, 16, NOT_CAUTIOUS, do_confWithBit_teddy);
    }

    for (; ptr < buf_end; ptr += 16) {
        m256 p_mask;
        m256 val_0 = vectoredLoad2x128(&p_mask, ptr, a->buf, buf_end,
                                       a->buf_history, a->len_history, 2);
        m256 r_0 = prep_conf_fat_teddy_m2(maskBase, &res_old_1, val_0);
        r_0 = and256(r_0, p_mask);
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBit_teddy);
    }

    return HWLM_SUCCESS;
}

hwlm_error_t fdr_exec_teddy_avx2_msks3_fat(const struct FDR *fdr,
                                           const struct FDR_Runtime_Args *a,
                                           hwlm_group_t control) {
    const u8 *buf_end = a->buf + a->len;
    const u8 *ptr = a->buf + a->start_offset;
    u32 floodBackoff = FLOOD_BACKOFF_START;
    const u8 *tryFloodDetect = a->firstFloodDetect;
    u32 last_match = (u32)-1;
    const struct Teddy *teddy = (const struct Teddy *)fdr;
    const size_t iterBytes = 32;
    DEBUG_PRINTF("params: buf %p len %zu start_offset %zu\n",
                 a->buf, a->len, a->start_offset);

    const m256 *maskBase = getMaskBase_avx2(teddy);
    const u32 *confBase = getConfBase_avx2(teddy, 3);

    m256 res_old_1 = ones256();
    m256 res_old_2 = ones256();
    const u8 *mainStart = ROUNDUP_PTR(ptr, 16);
    DEBUG_PRINTF("derive: ptr: %p mainstart %p\n", ptr, mainStart);
    if (ptr < mainStart) {
        ptr = mainStart - 16;
        m256 p_mask;
        m256 val_0 = vectoredLoad2x128(&p_mask, ptr, a->buf, buf_end,
                                       a->buf_history, a->len_history, 3);
        m256 r_0 = prep_conf_fat_teddy_m3(maskBase, &res_old_1, &res_old_2,
                                          val_0);
        r_0 = and256(r_0, p_mask);
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBitMany_teddy);
        ptr += 16;
    }

    if (ptr + 16 < buf_end) {
        m256 r_0 = prep_conf_fat_teddy_m3(maskBase, &res_old_1, &res_old_2,
                                          load2x128(ptr));
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBitMany_teddy);
        ptr += 16;
    }

    for ( ; ptr + iterBytes <= buf_end; ptr += iterBytes) {
        __builtin_prefetch(ptr + (iterBytes*4));
        CHECK_FLOOD;
        m256 r_0 = prep_conf_fat_teddy_m3(maskBase, &res_old_1, &res_old_2,
                                          load2x128(ptr));
        CONFIRM_FAT_TEDDY(r_0, 16, 0, NOT_CAUTIOUS, do_confWithBitMany_teddy);
        m256 r_1 = prep_conf_fat_teddy_m3(maskBase, &res_old_1, &res_old_2,
                                          load2x128(ptr + 16));
        CONFIRM_FAT_TEDDY(r_1, 16, 16, NOT_CAUTIOUS, do_confWithBitMany_teddy);
    }

    for (; ptr < buf_end; ptr += 16) {
        m256 p_mask;
        m256 val_0 = vectoredLoad2x128(&p_mask, ptr, a->buf, buf_end,
                                       a->buf_history, a->len_history, 3);
        m256 r_0 = prep_conf_fat_teddy_m3(maskBase, &res_old_1, &res_old_2,
                                          val_0);
        r_0 = and256(r_0, p_mask);
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBitMany_teddy);
    }

    return HWLM_SUCCESS;
}

hwlm_error_t fdr_exec_teddy_avx2_msks3_pck_fat(const struct FDR *fdr,
                                               const struct FDR_Runtime_Args *a,
                                               hwlm_group_t control) {
    const u8 *buf_end = a->buf + a->len;
    const u8 *ptr = a->buf + a->start_offset;
    u32 floodBackoff = FLOOD_BACKOFF_START;
    const u8 *tryFloodDetect = a->firstFloodDetect;
    u32 last_match = (u32)-1;
    const struct Teddy *teddy = (const struct Teddy *)fdr;
    const size_t iterBytes = 32;
    DEBUG_PRINTF("params: buf %p len %zu start_offset %zu\n",
                 a->buf, a->len, a->start_offset);

    const m256 *maskBase = getMaskBase_avx2(teddy);
    const u32 *confBase = getConfBase_avx2(teddy, 3);

    m256 res_old_1 = ones256();
    m256 res_old_2 = ones256();
    const u8 *mainStart = ROUNDUP_PTR(ptr, 16);
    DEBUG_PRINTF("derive: ptr: %p mainstart %p\n", ptr, mainStart);
    if (ptr < mainStart) {
        ptr = mainStart - 16;
        m256 p_mask;
        m256 val_0 = vectoredLoad2x128(&p_mask, ptr, a->buf, buf_end,
                                       a->buf_history, a->len_history, 3);
        m256 r_0 = prep_conf_fat_teddy_m3(maskBase, &res_old_1, &res_old_2,
                                          val_0);
        r_0 = and256(r_0, p_mask);
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBit_teddy);
        ptr += 16;
    }

    if (ptr + 16 < buf_end) {
        m256 r_0 = prep_conf_fat_teddy_m3(maskBase, &res_old_1, &res_old_2,
                                          load2x128(ptr));
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBit_teddy);
        ptr += 16;
    }

    for ( ; ptr + iterBytes <= buf_end; ptr += iterBytes) {
        __builtin_prefetch(ptr + (iterBytes*4));
        CHECK_FLOOD;
        m256 r_0 = prep_conf_fat_teddy_m3(maskBase, &res_old_1, &res_old_2,
                                          load2x128(ptr));
        CONFIRM_FAT_TEDDY(r_0, 16, 0, NOT_CAUTIOUS, do_confWithBit_teddy);
        m256 r_1 = prep_conf_fat_teddy_m3(maskBase, &res_old_1, &res_old_2,
                                          load2x128(ptr + 16));
        CONFIRM_FAT_TEDDY(r_1, 16, 16, NOT_CAUTIOUS, do_confWithBit_teddy);
    }

    for (; ptr < buf_end; ptr += 16) {
        m256 p_mask;
        m256 val_0 = vectoredLoad2x128(&p_mask, ptr, a->buf, buf_end,
                                       a->buf_history, a->len_history, 3);
        m256 r_0 = prep_conf_fat_teddy_m3(maskBase, &res_old_1, &res_old_2,
                                          val_0);
        r_0 = and256(r_0, p_mask);
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBit_teddy);
    }

    return HWLM_SUCCESS;
}

hwlm_error_t fdr_exec_teddy_avx2_msks4_fat(const struct FDR *fdr,
                                           const struct FDR_Runtime_Args *a,
                                           hwlm_group_t control) {
    const u8 *buf_end = a->buf + a->len;
    const u8 *ptr = a->buf + a->start_offset;
    u32 floodBackoff = FLOOD_BACKOFF_START;
    const u8 *tryFloodDetect = a->firstFloodDetect;
    u32 last_match = (u32)-1;
    const struct Teddy *teddy = (const struct Teddy *)fdr;
    const size_t iterBytes = 32;
    DEBUG_PRINTF("params: buf %p len %zu start_offset %zu\n",
                 a->buf, a->len, a->start_offset);

    const m256 *maskBase = getMaskBase_avx2(teddy);
    const u32 *confBase = getConfBase_avx2(teddy, 4);

    m256 res_old_1 = ones256();
    m256 res_old_2 = ones256();
    m256 res_old_3 = ones256();
    const u8 *mainStart = ROUNDUP_PTR(ptr, 16);
    DEBUG_PRINTF("derive: ptr: %p mainstart %p\n", ptr, mainStart);
    if (ptr < mainStart) {
        ptr = mainStart - 16;
        m256 p_mask;
        m256 val_0 = vectoredLoad2x128(&p_mask, ptr, a->buf, buf_end,
                                       a->buf_history, a->len_history, 4);
        m256 r_0 = prep_conf_fat_teddy_m4(maskBase, &res_old_1, &res_old_2,
                                          &res_old_3, val_0);
        r_0 = and256(r_0, p_mask);
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBitMany_teddy);
        ptr += 16;
    }

    if (ptr + 16 < buf_end) {
        m256 r_0 = prep_conf_fat_teddy_m4(maskBase, &res_old_1, &res_old_2,
                                          &res_old_3, load2x128(ptr));
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBitMany_teddy);
        ptr += 16;
    }

    for ( ; ptr + iterBytes <= buf_end; ptr += iterBytes) {
        __builtin_prefetch(ptr + (iterBytes*4));
        CHECK_FLOOD;
        m256 r_0 = prep_conf_fat_teddy_m4(maskBase, &res_old_1, &res_old_2,
                                          &res_old_3, load2x128(ptr));
        CONFIRM_FAT_TEDDY(r_0, 16, 0, NOT_CAUTIOUS, do_confWithBitMany_teddy);
        m256 r_1 = prep_conf_fat_teddy_m4(maskBase, &res_old_1, &res_old_2,
                                          &res_old_3, load2x128(ptr + 16));
        CONFIRM_FAT_TEDDY(r_1, 16, 16, NOT_CAUTIOUS, do_confWithBitMany_teddy);
    }

    for (; ptr < buf_end; ptr += 16) {
        m256 p_mask;
        m256 val_0 = vectoredLoad2x128(&p_mask, ptr, a->buf, buf_end,
                                       a->buf_history, a->len_history, 4);
        m256 r_0 = prep_conf_fat_teddy_m4(maskBase, &res_old_1, &res_old_2,
                                          &res_old_3, val_0);
        r_0 = and256(r_0, p_mask);
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBitMany_teddy);
    }

    return HWLM_SUCCESS;
}

hwlm_error_t fdr_exec_teddy_avx2_msks4_pck_fat(const struct FDR *fdr,
                                               const struct FDR_Runtime_Args *a,
                                               hwlm_group_t control) {
    const u8 *buf_end = a->buf + a->len;
    const u8 *ptr = a->buf + a->start_offset;
    u32 floodBackoff = FLOOD_BACKOFF_START;
    const u8 *tryFloodDetect = a->firstFloodDetect;
    u32 last_match = (u32)-1;
    const struct Teddy *teddy = (const struct Teddy *)fdr;
    const size_t iterBytes = 32;
    DEBUG_PRINTF("params: buf %p len %zu start_offset %zu\n",
                 a->buf, a->len, a->start_offset);

    const m256 *maskBase = getMaskBase_avx2(teddy);
    const u32 *confBase = getConfBase_avx2(teddy, 4);

    m256 res_old_1 = ones256();
    m256 res_old_2 = ones256();
    m256 res_old_3 = ones256();
    const u8 *mainStart = ROUNDUP_PTR(ptr, 16);
    DEBUG_PRINTF("derive: ptr: %p mainstart %p\n", ptr, mainStart);
    if (ptr < mainStart) {
        ptr = mainStart - 16;
        m256 p_mask;
        m256 val_0 = vectoredLoad2x128(&p_mask, ptr, a->buf, buf_end,
                                       a->buf_history, a->len_history, 4);
        m256 r_0 = prep_conf_fat_teddy_m4(maskBase, &res_old_1, &res_old_2,
                                          &res_old_3, val_0);
        r_0 = and256(r_0, p_mask);
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBit_teddy);
        ptr += 16;
    }

    if (ptr + 16 < buf_end) {
        m256 r_0 = prep_conf_fat_teddy_m4(maskBase, &res_old_1, &res_old_2,
                                          &res_old_3, load2x128(ptr));
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBit_teddy);
        ptr += 16;
    }

    for ( ; ptr + iterBytes <= buf_end; ptr += iterBytes) {
        __builtin_prefetch(ptr + (iterBytes*4));
        CHECK_FLOOD;
        m256 r_0 = prep_conf_fat_teddy_m4(maskBase, &res_old_1, &res_old_2,
                                          &res_old_3, load2x128(ptr));
        CONFIRM_FAT_TEDDY(r_0, 16, 0, NOT_CAUTIOUS, do_confWithBit_teddy);
        m256 r_1 = prep_conf_fat_teddy_m4(maskBase, &res_old_1, &res_old_2,
                                          &res_old_3, load2x128(ptr + 16));
        CONFIRM_FAT_TEDDY(r_1, 16, 16, NOT_CAUTIOUS, do_confWithBit_teddy);
    }

    for (; ptr < buf_end; ptr += 16) {
        m256 p_mask;
        m256 val_0 = vectoredLoad2x128(&p_mask, ptr, a->buf, buf_end,
                                       a->buf_history, a->len_history, 4);
        m256 r_0 = prep_conf_fat_teddy_m4(maskBase, &res_old_1, &res_old_2,
                                          &res_old_3, val_0);
        r_0 = and256(r_0, p_mask);
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBit_teddy);
    }

    return HWLM_SUCCESS;
}

#endif // HAVE_AVX2
