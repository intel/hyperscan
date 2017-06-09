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
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}
};

#ifdef ARCH_64_BIT
#define CONFIRM_TEDDY(var, bucket, offset, reason, conf_fn)                 \
do {                                                                        \
    if (unlikely(isnonzero128(var))) {                                      \
        u64a lo = movq(var);                                                \
        u64a hi = movq(rshiftbyte_m128(var, 8));                            \
        if (unlikely(lo)) {                                                 \
            conf_fn(&lo, bucket, offset, confBase, reason, a, ptr,          \
                    &control, &last_match);                                 \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
        if (unlikely(hi)) {                                                 \
            conf_fn(&hi, bucket, offset + 8, confBase, reason, a, ptr,      \
                    &control, &last_match);                                 \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
    }                                                                       \
} while (0);
#else
#define CONFIRM_TEDDY(var, bucket, offset, reason, conf_fn)                 \
do {                                                                        \
    if (unlikely(isnonzero128(var))) {                                      \
        u32 part1 = movd(var);                                              \
        u32 part2 = movd(rshiftbyte_m128(var, 4));                          \
        u32 part3 = movd(rshiftbyte_m128(var, 8));                          \
        u32 part4 = movd(rshiftbyte_m128(var, 12));                         \
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
#endif

static really_inline
m128 prep_conf_teddy_m1(const m128 *maskBase, m128 val) {
    m128 mask = set16x8(0xf);
    m128 lo = and128(val, mask);
    m128 hi = and128(rshift64_m128(val, 4), mask);
    return and128(pshufb_m128(maskBase[0 * 2], lo),
                  pshufb_m128(maskBase[0 * 2 + 1], hi));
}

static really_inline
m128 prep_conf_teddy_m2(const m128 *maskBase, m128 *old_1, m128 val) {
    m128 mask = set16x8(0xf);
    m128 lo = and128(val, mask);
    m128 hi = and128(rshift64_m128(val, 4), mask);
    m128 r = prep_conf_teddy_m1(maskBase, val);

    m128 res_1 = and128(pshufb_m128(maskBase[1*2], lo),
                        pshufb_m128(maskBase[1*2+1], hi));
    m128 res_shifted_1 = palignr(res_1, *old_1, 16-1);
    *old_1 = res_1;
    return and128(r, res_shifted_1);
}

static really_inline
m128 prep_conf_teddy_m3(const m128 *maskBase, m128 *old_1, m128 *old_2,
                        m128 val) {
    m128 mask = set16x8(0xf);
    m128 lo = and128(val, mask);
    m128 hi = and128(rshift64_m128(val, 4), mask);
    m128 r = prep_conf_teddy_m2(maskBase, old_1, val);

    m128 res_2 = and128(pshufb_m128(maskBase[2*2], lo),
                        pshufb_m128(maskBase[2*2+1], hi));
    m128 res_shifted_2 = palignr(res_2, *old_2, 16-2);
    *old_2 = res_2;
    return and128(r, res_shifted_2);
}

static really_inline
m128 prep_conf_teddy_m4(const m128 *maskBase, m128 *old_1, m128 *old_2,
                        m128 *old_3, m128 val) {
    m128 mask = set16x8(0xf);
    m128 lo = and128(val, mask);
    m128 hi = and128(rshift64_m128(val, 4), mask);
    m128 r = prep_conf_teddy_m3(maskBase, old_1, old_2, val);

    m128 res_3 = and128(pshufb_m128(maskBase[3*2], lo),
                        pshufb_m128(maskBase[3*2+1], hi));
    m128 res_shifted_3 = palignr(res_3, *old_3, 16-3);
    *old_3 = res_3;
    return and128(r, res_shifted_3);
}

hwlm_error_t fdr_exec_teddy_msks1(const struct FDR *fdr,
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

    const m128 *maskBase = getMaskBase(teddy);
    const u32 *confBase = getConfBase(teddy, 1);

    const u8 *mainStart = ROUNDUP_PTR(ptr, 16);
    DEBUG_PRINTF("derive: ptr: %p mainstart %p\n", ptr, mainStart);
    if (ptr < mainStart) {
        ptr = mainStart - 16;
        m128 p_mask;
        m128 val_0 = vectoredLoad128(&p_mask, ptr, a->buf, buf_end,
                                     a->buf_history, a->len_history, 1);
        m128 r_0 = prep_conf_teddy_m1(maskBase, val_0);
        r_0 = and128(r_0, p_mask);
        CONFIRM_TEDDY(r_0, 8, 0, VECTORING, do_confWithBit1_teddy);
        ptr += 16;
    }

    if (ptr + 16 < buf_end) {
        m128 r_0 = prep_conf_teddy_m1(maskBase, load128(ptr));
        CONFIRM_TEDDY(r_0, 8, 0, VECTORING, do_confWithBit1_teddy);
        ptr += 16;
    }

    for (; ptr + iterBytes <= buf_end; ptr += iterBytes) {
        __builtin_prefetch(ptr + (iterBytes*4));
        CHECK_FLOOD;
        m128 r_0 = prep_conf_teddy_m1(maskBase, load128(ptr));
        CONFIRM_TEDDY(r_0, 8, 0, NOT_CAUTIOUS, do_confWithBit1_teddy);
        m128 r_1 = prep_conf_teddy_m1(maskBase, load128(ptr + 16));
        CONFIRM_TEDDY(r_1, 8, 16, NOT_CAUTIOUS, do_confWithBit1_teddy);
    }

    for (; ptr < buf_end; ptr += 16) {
        m128 p_mask;
        m128 val_0 = vectoredLoad128(&p_mask, ptr, a->buf, buf_end,
                                     a->buf_history, a->len_history, 1);
        m128 r_0 = prep_conf_teddy_m1(maskBase, val_0);
        r_0 = and128(r_0, p_mask);
        CONFIRM_TEDDY(r_0, 8, 0, VECTORING, do_confWithBit1_teddy);
    }

    return HWLM_SUCCESS;
}

hwlm_error_t fdr_exec_teddy_msks1_pck(const struct FDR *fdr,
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

    const m128 *maskBase = getMaskBase(teddy);
    const u32 *confBase = getConfBase(teddy, 1);

    const u8 *mainStart = ROUNDUP_PTR(ptr, 16);
    DEBUG_PRINTF("derive: ptr: %p mainstart %p\n", ptr, mainStart);
    if (ptr < mainStart) {
        ptr = mainStart - 16;
        m128 p_mask;
        m128 val_0 = vectoredLoad128(&p_mask, ptr, a->buf, buf_end,
                                     a->buf_history, a->len_history, 1);
        m128 r_0 = prep_conf_teddy_m1(maskBase, val_0);
        r_0 = and128(r_0, p_mask);
        CONFIRM_TEDDY(r_0, 8, 0, VECTORING, do_confWithBit_teddy);
        ptr += 16;
    }

    if (ptr + 16 < buf_end) {
        m128 r_0 = prep_conf_teddy_m1(maskBase, load128(ptr));
        CONFIRM_TEDDY(r_0, 8, 0, VECTORING, do_confWithBit_teddy);
        ptr += 16;
    }

    for (; ptr + iterBytes <= buf_end; ptr += iterBytes) {
        __builtin_prefetch(ptr + (iterBytes*4));
        CHECK_FLOOD;
        m128 r_0 = prep_conf_teddy_m1(maskBase, load128(ptr));
        CONFIRM_TEDDY(r_0, 8, 0, NOT_CAUTIOUS, do_confWithBit_teddy);
        m128 r_1 = prep_conf_teddy_m1(maskBase, load128(ptr + 16));
        CONFIRM_TEDDY(r_1, 8, 16, NOT_CAUTIOUS, do_confWithBit_teddy);
    }

    for (; ptr < buf_end; ptr += 16) {
        m128 p_mask;
        m128 val_0 = vectoredLoad128(&p_mask, ptr, a->buf, buf_end,
                                     a->buf_history, a->len_history, 1);
        m128 r_0 = prep_conf_teddy_m1(maskBase, val_0);
        r_0 = and128(r_0, p_mask);
        CONFIRM_TEDDY(r_0, 8, 0, VECTORING, do_confWithBit_teddy);
    }

    return HWLM_SUCCESS;
}

hwlm_error_t fdr_exec_teddy_msks2(const struct FDR *fdr,
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

    const m128 *maskBase = getMaskBase(teddy);
    const u32 *confBase = getConfBase(teddy, 2);

    m128 res_old_1 = ones128();
    const u8 *mainStart = ROUNDUP_PTR(ptr, 16);
    DEBUG_PRINTF("derive: ptr: %p mainstart %p\n", ptr, mainStart);
    if (ptr < mainStart) {
        ptr = mainStart - 16;
        m128 p_mask;
        m128 val_0 = vectoredLoad128(&p_mask, ptr, a->buf, buf_end,
                                     a->buf_history, a->len_history, 2);
        m128 r_0 = prep_conf_teddy_m2(maskBase, &res_old_1, val_0);
        r_0 = and128(r_0, p_mask);
        CONFIRM_TEDDY(r_0, 8, 0, VECTORING, do_confWithBitMany_teddy);
        ptr += 16;
    }

    if (ptr + 16 < buf_end) {
        m128 r_0 = prep_conf_teddy_m2(maskBase, &res_old_1, load128(ptr));
        CONFIRM_TEDDY(r_0, 8, 0, VECTORING, do_confWithBitMany_teddy);
        ptr += 16;
    }

    for (; ptr + iterBytes <= buf_end; ptr += iterBytes) {
        __builtin_prefetch(ptr + (iterBytes*4));
        CHECK_FLOOD;
        m128 r_0 = prep_conf_teddy_m2(maskBase, &res_old_1, load128(ptr));
        CONFIRM_TEDDY(r_0, 8, 0, NOT_CAUTIOUS, do_confWithBitMany_teddy);
        m128 r_1 = prep_conf_teddy_m2(maskBase, &res_old_1, load128(ptr + 16));
        CONFIRM_TEDDY(r_1, 8, 16, NOT_CAUTIOUS, do_confWithBitMany_teddy);
    }

    for (; ptr < buf_end; ptr += 16) {
        m128 p_mask;
        m128 val_0 = vectoredLoad128(&p_mask, ptr, a->buf, buf_end,
                                     a->buf_history, a->len_history, 2);
        m128 r_0 = prep_conf_teddy_m2(maskBase, &res_old_1, val_0);
        r_0 = and128(r_0, p_mask);
        CONFIRM_TEDDY(r_0, 8, 0, VECTORING, do_confWithBitMany_teddy);
    }

    return HWLM_SUCCESS;
}

hwlm_error_t fdr_exec_teddy_msks2_pck(const struct FDR *fdr,
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

    const m128 *maskBase = getMaskBase(teddy);
    const u32 *confBase = getConfBase(teddy, 2);

    m128 res_old_1 = ones128();
    const u8 *mainStart = ROUNDUP_PTR(ptr, 16);
    DEBUG_PRINTF("derive: ptr: %p mainstart %p\n", ptr, mainStart);
    if (ptr < mainStart) {
        ptr = mainStart - 16;
        m128 p_mask;
        m128 val_0 = vectoredLoad128(&p_mask, ptr, a->buf, buf_end,
                                     a->buf_history, a->len_history, 2);
        m128 r_0 = prep_conf_teddy_m2(maskBase, &res_old_1, val_0);
        r_0 = and128(r_0, p_mask);
        CONFIRM_TEDDY(r_0, 8, 0, VECTORING, do_confWithBit_teddy);
        ptr += 16;
    }

    if (ptr + 16 < buf_end) {
        m128 r_0 = prep_conf_teddy_m2(maskBase, &res_old_1, load128(ptr));
        CONFIRM_TEDDY(r_0, 8, 0, VECTORING, do_confWithBit_teddy);
        ptr += 16;
    }

    for (; ptr + iterBytes <= buf_end; ptr += iterBytes) {
        __builtin_prefetch(ptr + (iterBytes*4));
        CHECK_FLOOD;
        m128 r_0 = prep_conf_teddy_m2(maskBase, &res_old_1, load128(ptr));
        CONFIRM_TEDDY(r_0, 8, 0, NOT_CAUTIOUS, do_confWithBit_teddy);
        m128 r_1 = prep_conf_teddy_m2(maskBase, &res_old_1, load128(ptr + 16));
        CONFIRM_TEDDY(r_1, 8, 16, NOT_CAUTIOUS, do_confWithBit_teddy);
    }

    for (; ptr < buf_end; ptr += 16) {
        m128 p_mask;
        m128 val_0 = vectoredLoad128(&p_mask, ptr, a->buf, buf_end,
                                   a->buf_history, a->len_history, 2);
        m128 r_0 = prep_conf_teddy_m2(maskBase, &res_old_1, val_0);
        r_0 = and128(r_0, p_mask);
        CONFIRM_TEDDY(r_0, 8, 0, VECTORING, do_confWithBit_teddy);
    }

    return HWLM_SUCCESS;
}

hwlm_error_t fdr_exec_teddy_msks3(const struct FDR *fdr,
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

    const m128 *maskBase = getMaskBase(teddy);
    const u32 *confBase = getConfBase(teddy, 3);

    m128 res_old_1 = ones128();
    m128 res_old_2 = ones128();
    const u8 *mainStart = ROUNDUP_PTR(ptr, 16);
    DEBUG_PRINTF("derive: ptr: %p mainstart %p\n", ptr, mainStart);
    if (ptr < mainStart) {
        ptr = mainStart - 16;
        m128 p_mask;
        m128 val_0 = vectoredLoad128(&p_mask, ptr, a->buf, buf_end,
                                     a->buf_history, a->len_history, 3);
        m128 r_0 = prep_conf_teddy_m3(maskBase, &res_old_1, &res_old_2,
                                      val_0);
        r_0 = and128(r_0, p_mask);
        CONFIRM_TEDDY(r_0, 8, 0, VECTORING, do_confWithBitMany_teddy);
        ptr += 16;
    }

    if (ptr + 16 < buf_end) {
        m128 r_0 = prep_conf_teddy_m3(maskBase, &res_old_1, &res_old_2,
                                      load128(ptr));
        CONFIRM_TEDDY(r_0, 8, 0, VECTORING, do_confWithBitMany_teddy);
        ptr += 16;
    }

    for (; ptr + iterBytes <= buf_end; ptr += iterBytes) {
        __builtin_prefetch(ptr + (iterBytes*4));
        CHECK_FLOOD;
        m128 r_0 = prep_conf_teddy_m3(maskBase, &res_old_1, &res_old_2,
                                      load128(ptr));
        CONFIRM_TEDDY(r_0, 8, 0, NOT_CAUTIOUS, do_confWithBitMany_teddy);
        m128 r_1 = prep_conf_teddy_m3(maskBase, &res_old_1, &res_old_2,
                                      load128(ptr + 16));
        CONFIRM_TEDDY(r_1, 8, 16, NOT_CAUTIOUS, do_confWithBitMany_teddy);
    }

    for (; ptr < buf_end; ptr += 16) {
        m128 p_mask;
        m128 val_0 = vectoredLoad128(&p_mask, ptr, a->buf, buf_end,
                                     a->buf_history, a->len_history, 3);
        m128 r_0 = prep_conf_teddy_m3(maskBase, &res_old_1, &res_old_2, val_0);
        r_0 = and128(r_0, p_mask);
        CONFIRM_TEDDY(r_0, 8, 0, VECTORING, do_confWithBitMany_teddy);
    }

    return HWLM_SUCCESS;
}

hwlm_error_t fdr_exec_teddy_msks3_pck(const struct FDR *fdr,
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

    const m128 *maskBase = getMaskBase(teddy);
    const u32 *confBase = getConfBase(teddy, 3);

    m128 res_old_1 = ones128();
    m128 res_old_2 = ones128();
    const u8 *mainStart = ROUNDUP_PTR(ptr, 16);
    DEBUG_PRINTF("derive: ptr: %p mainstart %p\n", ptr, mainStart);
    if (ptr < mainStart) {
        ptr = mainStart - 16;
        m128 p_mask;
        m128 val_0 = vectoredLoad128(&p_mask, ptr, a->buf, buf_end,
                                     a->buf_history, a->len_history, 3);
        m128 r_0 = prep_conf_teddy_m3(maskBase, &res_old_1, &res_old_2,
                                      val_0);
        r_0 = and128(r_0, p_mask);
        CONFIRM_TEDDY(r_0, 8, 0, VECTORING, do_confWithBit_teddy);
        ptr += 16;
    }

    if (ptr + 16 < buf_end) {
        m128 r_0 = prep_conf_teddy_m3(maskBase, &res_old_1, &res_old_2,
                                      load128(ptr));
        CONFIRM_TEDDY(r_0, 8, 0, VECTORING, do_confWithBit_teddy);
        ptr += 16;
    }

    for (; ptr + iterBytes <= buf_end; ptr += iterBytes) {
        __builtin_prefetch(ptr + (iterBytes*4));
        CHECK_FLOOD;
        m128 r_0 = prep_conf_teddy_m3(maskBase, &res_old_1, &res_old_2,
                                      load128(ptr));
        CONFIRM_TEDDY(r_0, 8, 0, NOT_CAUTIOUS, do_confWithBit_teddy);
        m128 r_1 = prep_conf_teddy_m3(maskBase, &res_old_1, &res_old_2,
                                      load128(ptr + 16));
        CONFIRM_TEDDY(r_1, 8, 16, NOT_CAUTIOUS, do_confWithBit_teddy);
    }

    for (; ptr < buf_end; ptr += 16) {
        m128 p_mask;
        m128 val_0 = vectoredLoad128(&p_mask, ptr, a->buf, buf_end,
                                     a->buf_history, a->len_history, 3);
        m128 r_0 = prep_conf_teddy_m3(maskBase, &res_old_1, &res_old_2, val_0);
        r_0 = and128(r_0, p_mask);
        CONFIRM_TEDDY(r_0, 8, 0, VECTORING, do_confWithBit_teddy);
    }

    return HWLM_SUCCESS;
}

hwlm_error_t fdr_exec_teddy_msks4(const struct FDR *fdr,
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

    const m128 *maskBase = getMaskBase(teddy);
    const u32 *confBase = getConfBase(teddy, 4);

    m128 res_old_1 = ones128();
    m128 res_old_2 = ones128();
    m128 res_old_3 = ones128();
    const u8 *mainStart = ROUNDUP_PTR(ptr, 16);
    DEBUG_PRINTF("derive: ptr: %p mainstart %p\n", ptr, mainStart);
    if (ptr < mainStart) {
        ptr = mainStart - 16;
        m128 p_mask;
        m128 val_0 = vectoredLoad128(&p_mask, ptr, a->buf, buf_end,
                                     a->buf_history, a->len_history, 4);
        m128 r_0 = prep_conf_teddy_m4(maskBase, &res_old_1, &res_old_2,
                                      &res_old_3, val_0);
        r_0 = and128(r_0, p_mask);
        CONFIRM_TEDDY(r_0, 8, 0, VECTORING, do_confWithBitMany_teddy);
        ptr += 16;
    }

    if (ptr + 16 < buf_end) {
        m128 r_0 = prep_conf_teddy_m4(maskBase, &res_old_1, &res_old_2,
                                      &res_old_3, load128(ptr));
        CONFIRM_TEDDY(r_0, 8, 0, VECTORING, do_confWithBitMany_teddy);
        ptr += 16;
    }

    for (; ptr + iterBytes <= buf_end; ptr += iterBytes) {
        __builtin_prefetch(ptr + (iterBytes*4));
        CHECK_FLOOD;
        m128 r_0 = prep_conf_teddy_m4(maskBase, &res_old_1, &res_old_2,
                                      &res_old_3, load128(ptr));
        CONFIRM_TEDDY(r_0, 8, 0, NOT_CAUTIOUS, do_confWithBitMany_teddy);
        m128 r_1 = prep_conf_teddy_m4(maskBase, &res_old_1, &res_old_2,
                                      &res_old_3, load128(ptr + 16));
        CONFIRM_TEDDY(r_1, 8, 16, NOT_CAUTIOUS, do_confWithBitMany_teddy);
    }

    for (; ptr < buf_end; ptr += 16) {
        m128 p_mask;
        m128 val_0 = vectoredLoad128(&p_mask, ptr, a->buf, buf_end,
                                     a->buf_history, a->len_history, 4);
        m128 r_0 = prep_conf_teddy_m4(maskBase, &res_old_1, &res_old_2,
                                      &res_old_3, val_0);
        r_0 = and128(r_0, p_mask);
        CONFIRM_TEDDY(r_0, 8, 0, VECTORING, do_confWithBitMany_teddy);
    }

    return HWLM_SUCCESS;
}

hwlm_error_t fdr_exec_teddy_msks4_pck(const struct FDR *fdr,
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

    const m128 *maskBase = getMaskBase(teddy);
    const u32 *confBase = getConfBase(teddy, 4);

    m128 res_old_1 = ones128();
    m128 res_old_2 = ones128();
    m128 res_old_3 = ones128();
    const u8 *mainStart = ROUNDUP_PTR(ptr, 16);
    DEBUG_PRINTF("derive: ptr: %p mainstart %p\n", ptr, mainStart);
    if (ptr < mainStart) {
        ptr = mainStart - 16;
        m128 p_mask;
        m128 val_0 = vectoredLoad128(&p_mask, ptr, a->buf, buf_end,
                                     a->buf_history, a->len_history, 4);
        m128 r_0 = prep_conf_teddy_m4(maskBase, &res_old_1, &res_old_2,
                                      &res_old_3, val_0);
        r_0 = and128(r_0, p_mask);
        CONFIRM_TEDDY(r_0, 8, 0, VECTORING, do_confWithBit_teddy);
        ptr += 16;
    }

    if (ptr + 16 < buf_end) {
        m128 r_0 = prep_conf_teddy_m4(maskBase, &res_old_1, &res_old_2,
                                      &res_old_3, load128(ptr));
        CONFIRM_TEDDY(r_0, 8, 0, VECTORING, do_confWithBit_teddy);
        ptr += 16;
    }

    for (; ptr + iterBytes <= buf_end; ptr += iterBytes) {
        __builtin_prefetch(ptr + (iterBytes*4));
        CHECK_FLOOD;
        m128 r_0 = prep_conf_teddy_m4(maskBase, &res_old_1, &res_old_2,
                                      &res_old_3, load128(ptr));
        CONFIRM_TEDDY(r_0, 8, 0, NOT_CAUTIOUS, do_confWithBit_teddy);
        m128 r_1 = prep_conf_teddy_m4(maskBase, &res_old_1, &res_old_2,
                                      &res_old_3, load128(ptr + 16));
        CONFIRM_TEDDY(r_1, 8, 16, NOT_CAUTIOUS, do_confWithBit_teddy);
    }

    for (; ptr < buf_end; ptr += 16) {
        m128 p_mask;
        m128 val_0 = vectoredLoad128(&p_mask, ptr, a->buf, buf_end,
                                     a->buf_history, a->len_history, 4);
        m128 r_0 = prep_conf_teddy_m4(maskBase, &res_old_1, &res_old_2,
                                      &res_old_3, val_0);
        r_0 = and128(r_0, p_mask);
        CONFIRM_TEDDY(r_0, 8, 0, VECTORING, do_confWithBit_teddy);
    }

    return HWLM_SUCCESS;
}
