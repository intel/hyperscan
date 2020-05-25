/*
 * Copyright (c) 2015-2020, Intel Corporation
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

#if defined(HAVE_AVX512VBMI) // VBMI strong teddy

#define CONF_CHUNK_64(chunk, bucket, off, reason, pt, conf_fn)              \
do {                                                                        \
    if (unlikely(chunk != ones_u64a)) {                                     \
        chunk = ~chunk;                                                     \
        conf_fn(&chunk, bucket, off, confBase, reason, a, pt,               \
                &control, &last_match);                                     \
        CHECK_HWLM_TERMINATE_MATCHING;                                      \
    }                                                                       \
} while(0)

#define CONF_CHUNK_32(chunk, bucket, off, reason, pt, conf_fn)              \
do {                                                                        \
    if (unlikely(chunk != ones_u32)) {                                      \
        chunk = ~chunk;                                                     \
        conf_fn(&chunk, bucket, off, confBase, reason, a, pt,               \
                &control, &last_match);                                     \
        CHECK_HWLM_TERMINATE_MATCHING;                                      \
    }                                                                       \
} while(0)

#else

#define CONF_CHUNK_64(chunk, bucket, off, reason, conf_fn)                  \
do {                                                                        \
    if (unlikely(chunk != ones_u64a)) {                                     \
        chunk = ~chunk;                                                     \
        conf_fn(&chunk, bucket, off, confBase, reason, a, ptr,              \
                &control, &last_match);                                     \
        CHECK_HWLM_TERMINATE_MATCHING;                                      \
    }                                                                       \
} while(0)

#define CONF_CHUNK_32(chunk, bucket, off, reason, conf_fn)                  \
do {                                                                        \
    if (unlikely(chunk != ones_u32)) {                                      \
        chunk = ~chunk;                                                     \
        conf_fn(&chunk, bucket, off, confBase, reason, a, ptr,              \
                &control, &last_match);                                     \
        CHECK_HWLM_TERMINATE_MATCHING;                                      \
    }                                                                       \
} while(0)

#endif

#if defined(HAVE_AVX512VBMI) // VBMI strong teddy

#ifdef ARCH_64_BIT
#define CONFIRM_TEDDY(var, bucket, offset, reason, pt, conf_fn)             \
do {                                                                        \
    if (unlikely(diff512(var, ones512()))) {                                \
        m128 p128_0 = extract128from512(var, 0);                            \
        m128 p128_1 = extract128from512(var, 1);                            \
        m128 p128_2 = extract128from512(var, 2);                            \
        m128 p128_3 = extract128from512(var, 3);                            \
        u64a part1 = movq(p128_0);                                          \
        u64a part2 = movq(rshiftbyte_m128(p128_0, 8));                      \
        u64a part3 = movq(p128_1);                                          \
        u64a part4 = movq(rshiftbyte_m128(p128_1, 8));                      \
        u64a part5 = movq(p128_2);                                          \
        u64a part6 = movq(rshiftbyte_m128(p128_2, 8));                      \
        u64a part7 = movq(p128_3);                                          \
        u64a part8 = movq(rshiftbyte_m128(p128_3, 8));                      \
        CONF_CHUNK_64(part1, bucket, offset, reason, pt, conf_fn);          \
        CONF_CHUNK_64(part2, bucket, offset + 8, reason, pt, conf_fn);      \
        CONF_CHUNK_64(part3, bucket, offset + 16, reason, pt, conf_fn);     \
        CONF_CHUNK_64(part4, bucket, offset + 24, reason, pt, conf_fn);     \
        CONF_CHUNK_64(part5, bucket, offset + 32, reason, pt, conf_fn);     \
        CONF_CHUNK_64(part6, bucket, offset + 40, reason, pt, conf_fn);     \
        CONF_CHUNK_64(part7, bucket, offset + 48, reason, pt, conf_fn);     \
        CONF_CHUNK_64(part8, bucket, offset + 56, reason, pt, conf_fn);     \
    }                                                                       \
} while(0)
#else
#define CONFIRM_TEDDY(var, bucket, offset, reason, pt, conf_fn)             \
do {                                                                        \
    if (unlikely(diff512(var, ones512()))) {                                \
        m128 p128_0 = extract128from512(var, 0);                            \
        m128 p128_1 = extract128from512(var, 1);                            \
        m128 p128_2 = extract128from512(var, 2);                            \
        m128 p128_3 = extract128from512(var, 3);                            \
        u32 part1 = movd(p128_0);                                           \
        u32 part2 = movd(rshiftbyte_m128(p128_0, 4));                       \
        u32 part3 = movd(rshiftbyte_m128(p128_0, 8));                       \
        u32 part4 = movd(rshiftbyte_m128(p128_0, 12));                      \
        u32 part5 = movd(p128_1);                                           \
        u32 part6 = movd(rshiftbyte_m128(p128_1, 4));                       \
        u32 part7 = movd(rshiftbyte_m128(p128_1, 8));                       \
        u32 part8 = movd(rshiftbyte_m128(p128_1, 12));                      \
        u32 part9 = movd(p128_2);                                           \
        u32 part10 = movd(rshiftbyte_m128(p128_2, 4));                      \
        u32 part11 = movd(rshiftbyte_m128(p128_2, 8));                      \
        u32 part12 = movd(rshiftbyte_m128(p128_2, 12));                     \
        u32 part13 = movd(p128_3);                                          \
        u32 part14 = movd(rshiftbyte_m128(p128_3, 4));                      \
        u32 part15 = movd(rshiftbyte_m128(p128_3, 8));                      \
        u32 part16 = movd(rshiftbyte_m128(p128_3, 12));                     \
        CONF_CHUNK_32(part1, bucket, offset, reason, pt, conf_fn);          \
        CONF_CHUNK_32(part2, bucket, offset + 4, reason, pt, conf_fn);      \
        CONF_CHUNK_32(part3, bucket, offset + 8, reason, pt, conf_fn);      \
        CONF_CHUNK_32(part4, bucket, offset + 12, reason, pt, conf_fn);     \
        CONF_CHUNK_32(part5, bucket, offset + 16, reason, pt, conf_fn);     \
        CONF_CHUNK_32(part6, bucket, offset + 20, reason, pt, conf_fn);     \
        CONF_CHUNK_32(part7, bucket, offset + 24, reason, pt, conf_fn);     \
        CONF_CHUNK_32(part8, bucket, offset + 28, reason, pt, conf_fn);     \
        CONF_CHUNK_32(part9, bucket, offset + 32, reason, pt, conf_fn);     \
        CONF_CHUNK_32(part10, bucket, offset + 36, reason, pt, conf_fn);    \
        CONF_CHUNK_32(part11, bucket, offset + 40, reason, pt, conf_fn);    \
        CONF_CHUNK_32(part12, bucket, offset + 44, reason, pt, conf_fn);    \
        CONF_CHUNK_32(part13, bucket, offset + 48, reason, pt, conf_fn);    \
        CONF_CHUNK_32(part14, bucket, offset + 52, reason, pt, conf_fn);    \
        CONF_CHUNK_32(part15, bucket, offset + 56, reason, pt, conf_fn);    \
        CONF_CHUNK_32(part16, bucket, offset + 60, reason, pt, conf_fn);    \
    }                                                                       \
} while(0)
#endif

#define PREP_SHUF_MASK                                                      \
    m512 lo = and512(val, *lo_mask);                                        \
    m512 hi = and512(rshift64_m512(val, 4), *lo_mask)

#define TEDDY_VBMI_PSHUFB_OR_M1                              \
    m512 shuf_or_b0 = or512(pshufb_m512(dup_mask[0], lo),    \
                            pshufb_m512(dup_mask[1], hi));

#define TEDDY_VBMI_PSHUFB_OR_M2                              \
    TEDDY_VBMI_PSHUFB_OR_M1                                  \
    m512 shuf_or_b1 = or512(pshufb_m512(dup_mask[2], lo),    \
                            pshufb_m512(dup_mask[3], hi));

#define TEDDY_VBMI_PSHUFB_OR_M3                              \
    TEDDY_VBMI_PSHUFB_OR_M2                                  \
    m512 shuf_or_b2 = or512(pshufb_m512(dup_mask[4], lo),    \
                            pshufb_m512(dup_mask[5], hi));

#define TEDDY_VBMI_PSHUFB_OR_M4                              \
    TEDDY_VBMI_PSHUFB_OR_M3                                  \
    m512 shuf_or_b3 = or512(pshufb_m512(dup_mask[6], lo),    \
                            pshufb_m512(dup_mask[7], hi));

#define TEDDY_VBMI_SL1_MASK   0xfffffffffffffffeULL
#define TEDDY_VBMI_SL2_MASK   0xfffffffffffffffcULL
#define TEDDY_VBMI_SL3_MASK   0xfffffffffffffff8ULL

#define TEDDY_VBMI_SHIFT_M1

#define TEDDY_VBMI_SHIFT_M2                      \
    TEDDY_VBMI_SHIFT_M1                          \
    m512 sl1 = maskz_vpermb512(TEDDY_VBMI_SL1_MASK, sl_msk[0], shuf_or_b1);

#define TEDDY_VBMI_SHIFT_M3                      \
    TEDDY_VBMI_SHIFT_M2                          \
    m512 sl2 = maskz_vpermb512(TEDDY_VBMI_SL2_MASK, sl_msk[1], shuf_or_b2);

#define TEDDY_VBMI_SHIFT_M4                      \
    TEDDY_VBMI_SHIFT_M3                          \
    m512 sl3 = maskz_vpermb512(TEDDY_VBMI_SL3_MASK, sl_msk[2], shuf_or_b3);

#define SHIFT_OR_M1            \
    shuf_or_b0

#define SHIFT_OR_M2            \
    or512(sl1, SHIFT_OR_M1)

#define SHIFT_OR_M3            \
    or512(sl2, SHIFT_OR_M2)

#define SHIFT_OR_M4            \
    or512(sl3, SHIFT_OR_M3)

static really_inline
m512 prep_conf_teddy_m1(const m512 *lo_mask, const m512 *dup_mask,
                        UNUSED const m512 *sl_msk, const m512 val) {
    PREP_SHUF_MASK;
    TEDDY_VBMI_PSHUFB_OR_M1;
    TEDDY_VBMI_SHIFT_M1;
    return SHIFT_OR_M1;
}

static really_inline
m512 prep_conf_teddy_m2(const m512 *lo_mask, const m512 *dup_mask,
                        const m512 *sl_msk, const m512 val) {
    PREP_SHUF_MASK;
    TEDDY_VBMI_PSHUFB_OR_M2;
    TEDDY_VBMI_SHIFT_M2;
    return SHIFT_OR_M2;
}

static really_inline
m512 prep_conf_teddy_m3(const m512 *lo_mask, const m512 *dup_mask,
                        const m512 *sl_msk, const m512 val) {
    PREP_SHUF_MASK;
    TEDDY_VBMI_PSHUFB_OR_M3;
    TEDDY_VBMI_SHIFT_M3;
    return SHIFT_OR_M3;
}

static really_inline
m512 prep_conf_teddy_m4(const m512 *lo_mask, const m512 *dup_mask,
                        const m512 *sl_msk, const m512 val) {
    PREP_SHUF_MASK;
    TEDDY_VBMI_PSHUFB_OR_M4;
    TEDDY_VBMI_SHIFT_M4;
    return SHIFT_OR_M4;
}

#define PREP_CONF_FN(val, n)                                                  \
    prep_conf_teddy_m##n(&lo_mask, dup_mask, sl_msk, val)

const u8 ALIGN_DIRECTIVE p_sh_mask_arr[80] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
    0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f,
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f
};

#define TEDDY_VBMI_SL1_POS    15
#define TEDDY_VBMI_SL2_POS    14
#define TEDDY_VBMI_SL3_POS    13

#define TEDDY_VBMI_LOAD_SHIFT_MASK_M1

#define TEDDY_VBMI_LOAD_SHIFT_MASK_M2    \
    TEDDY_VBMI_LOAD_SHIFT_MASK_M1        \
    sl_msk[0] = loadu512(p_sh_mask_arr + TEDDY_VBMI_SL1_POS);

#define TEDDY_VBMI_LOAD_SHIFT_MASK_M3    \
    TEDDY_VBMI_LOAD_SHIFT_MASK_M2        \
    sl_msk[1] = loadu512(p_sh_mask_arr + TEDDY_VBMI_SL2_POS);

#define TEDDY_VBMI_LOAD_SHIFT_MASK_M4    \
    TEDDY_VBMI_LOAD_SHIFT_MASK_M3        \
    sl_msk[2] = loadu512(p_sh_mask_arr + TEDDY_VBMI_SL3_POS);

#define PREPARE_MASKS_1                                                       \
    dup_mask[0] = set4x128(maskBase[0]);                                      \
    dup_mask[1] = set4x128(maskBase[1]);

#define PREPARE_MASKS_2                                                       \
    PREPARE_MASKS_1                                                           \
    dup_mask[2] = set4x128(maskBase[2]);                                      \
    dup_mask[3] = set4x128(maskBase[3]);

#define PREPARE_MASKS_3                                                       \
    PREPARE_MASKS_2                                                           \
    dup_mask[4] = set4x128(maskBase[4]);                                      \
    dup_mask[5] = set4x128(maskBase[5]);

#define PREPARE_MASKS_4                                                       \
    PREPARE_MASKS_3                                                           \
    dup_mask[6] = set4x128(maskBase[6]);                                      \
    dup_mask[7] = set4x128(maskBase[7]);

#define PREPARE_MASKS(n)                                                      \
    m512 lo_mask = set64x8(0xf);                                              \
    m512 dup_mask[n * 2];                                                     \
    m512 sl_msk[n - 1];                                                       \
    PREPARE_MASKS_##n                                                         \
    TEDDY_VBMI_LOAD_SHIFT_MASK_M##n

#define TEDDY_VBMI_CONF_MASK_HEAD   (0xffffffffffffffffULL >> n_sh)
#define TEDDY_VBMI_CONF_MASK_FULL   (0xffffffffffffffffULL << n_sh)
#define TEDDY_VBMI_CONF_MASK_VAR(n) (0xffffffffffffffffULL >> (64 - n) << overlap)
#define TEDDY_VBMI_LOAD_MASK_PATCH  (0xffffffffffffffffULL >> (64 - n_sh))

#define FDR_EXEC_TEDDY(fdr, a, control, n_msk, conf_fn)                       \
do {                                                                          \
    const u8 *buf_end = a->buf + a->len;                                      \
    const u8 *ptr = a->buf + a->start_offset;                                 \
    u32 floodBackoff = FLOOD_BACKOFF_START;                                   \
    const u8 *tryFloodDetect = a->firstFloodDetect;                           \
    u32 last_match = ones_u32;                                                \
    const struct Teddy *teddy = (const struct Teddy *)fdr;                    \
    const size_t iterBytes = 64;                                              \
    u32 n_sh = n_msk - 1;                                                     \
    const size_t loopBytes = 64 - n_sh;                                       \
    DEBUG_PRINTF("params: buf %p len %zu start_offset %zu\n",                 \
                 a->buf, a->len, a->start_offset);                            \
                                                                              \
    const m128 *maskBase = getMaskBase(teddy);                                \
    PREPARE_MASKS(n_msk);                                                     \
    const u32 *confBase = getConfBase(teddy);                                 \
                                                                              \
    u64a k = TEDDY_VBMI_CONF_MASK_FULL;                                       \
    m512 p_mask = set_mask_m512(~k);                                          \
    u32 overlap = 0;                                                          \
    u64a patch = 0;                                                           \
    if (likely(ptr + loopBytes <= buf_end)) {                                 \
        m512 p_mask0 = set_mask_m512(~TEDDY_VBMI_CONF_MASK_HEAD);             \
        m512 r_0 = PREP_CONF_FN(loadu512(ptr), n_msk);                        \
        r_0 = or512(r_0, p_mask0);                                            \
        CONFIRM_TEDDY(r_0, 8, 0, VECTORING, ptr, conf_fn);                    \
        ptr += loopBytes;                                                     \
        overlap = n_sh;                                                       \
        patch = TEDDY_VBMI_LOAD_MASK_PATCH;                                   \
    }                                                                         \
                                                                              \
    for (; ptr + loopBytes <= buf_end; ptr += loopBytes) {                    \
        __builtin_prefetch(ptr - n_sh + (64 * 2));                            \
        CHECK_FLOOD;                                                          \
        m512 r_0 = PREP_CONF_FN(loadu512(ptr - n_sh), n_msk);                 \
        r_0 = or512(r_0, p_mask);                                             \
        CONFIRM_TEDDY(r_0, 8, 0, NOT_CAUTIOUS, ptr - n_sh, conf_fn);          \
    }                                                                         \
                                                                              \
    assert(ptr + loopBytes > buf_end);                                        \
    if (ptr < buf_end) {                                                      \
        u32 left = (u32)(buf_end - ptr);                                      \
        u64a k1 = TEDDY_VBMI_CONF_MASK_VAR(left);                             \
        m512 p_mask1 = set_mask_m512(~k1);                                    \
        m512 val_0 = loadu_maskz_m512(k1 | patch, ptr - overlap);             \
        m512 r_0 = PREP_CONF_FN(val_0, n_msk);                                \
        r_0 = or512(r_0, p_mask1);                                            \
        CONFIRM_TEDDY(r_0, 8, 0, VECTORING, ptr - overlap, conf_fn);          \
    }                                                                         \
                                                                              \
    return HWLM_SUCCESS;                                                      \
} while(0)

#elif defined(HAVE_AVX512) // AVX512 reinforced teddy

#ifdef ARCH_64_BIT
#define CONFIRM_TEDDY(var, bucket, offset, reason, conf_fn)                 \
do {                                                                        \
    if (unlikely(diff512(var, ones512()))) {                                \
        m128 p128_0 = extract128from512(var, 0);                            \
        m128 p128_1 = extract128from512(var, 1);                            \
        m128 p128_2 = extract128from512(var, 2);                            \
        m128 p128_3 = extract128from512(var, 3);                            \
        u64a part1 = movq(p128_0);                                          \
        u64a part2 = movq(rshiftbyte_m128(p128_0, 8));                      \
        u64a part3 = movq(p128_1);                                          \
        u64a part4 = movq(rshiftbyte_m128(p128_1, 8));                      \
        u64a part5 = movq(p128_2);                                          \
        u64a part6 = movq(rshiftbyte_m128(p128_2, 8));                      \
        u64a part7 = movq(p128_3);                                          \
        u64a part8 = movq(rshiftbyte_m128(p128_3, 8));                      \
        CONF_CHUNK_64(part1, bucket, offset, reason, conf_fn);              \
        CONF_CHUNK_64(part2, bucket, offset + 8, reason, conf_fn);          \
        CONF_CHUNK_64(part3, bucket, offset + 16, reason, conf_fn);         \
        CONF_CHUNK_64(part4, bucket, offset + 24, reason, conf_fn);         \
        CONF_CHUNK_64(part5, bucket, offset + 32, reason, conf_fn);         \
        CONF_CHUNK_64(part6, bucket, offset + 40, reason, conf_fn);         \
        CONF_CHUNK_64(part7, bucket, offset + 48, reason, conf_fn);         \
        CONF_CHUNK_64(part8, bucket, offset + 56, reason, conf_fn);         \
    }                                                                       \
} while(0)
#else
#define CONFIRM_TEDDY(var, bucket, offset, reason, conf_fn)                 \
do {                                                                        \
    if (unlikely(diff512(var, ones512()))) {                                \
        m128 p128_0 = extract128from512(var, 0);                            \
        m128 p128_1 = extract128from512(var, 1);                            \
        m128 p128_2 = extract128from512(var, 2);                            \
        m128 p128_3 = extract128from512(var, 3);                            \
        u32 part1 = movd(p128_0);                                           \
        u32 part2 = movd(rshiftbyte_m128(p128_0, 4));                       \
        u32 part3 = movd(rshiftbyte_m128(p128_0, 8));                       \
        u32 part4 = movd(rshiftbyte_m128(p128_0, 12));                      \
        u32 part5 = movd(p128_1);                                           \
        u32 part6 = movd(rshiftbyte_m128(p128_1, 4));                       \
        u32 part7 = movd(rshiftbyte_m128(p128_1, 8));                       \
        u32 part8 = movd(rshiftbyte_m128(p128_1, 12));                      \
        u32 part9 = movd(p128_2);                                           \
        u32 part10 = movd(rshiftbyte_m128(p128_2, 4));                      \
        u32 part11 = movd(rshiftbyte_m128(p128_2, 8));                      \
        u32 part12 = movd(rshiftbyte_m128(p128_2, 12));                     \
        u32 part13 = movd(p128_3);                                          \
        u32 part14 = movd(rshiftbyte_m128(p128_3, 4));                      \
        u32 part15 = movd(rshiftbyte_m128(p128_3, 8));                      \
        u32 part16 = movd(rshiftbyte_m128(p128_3, 12));                     \
        CONF_CHUNK_32(part1, bucket, offset, reason, conf_fn);              \
        CONF_CHUNK_32(part2, bucket, offset + 4, reason, conf_fn);          \
        CONF_CHUNK_32(part3, bucket, offset + 8, reason, conf_fn);          \
        CONF_CHUNK_32(part4, bucket, offset + 12, reason, conf_fn);         \
        CONF_CHUNK_32(part5, bucket, offset + 16, reason, conf_fn);         \
        CONF_CHUNK_32(part6, bucket, offset + 20, reason, conf_fn);         \
        CONF_CHUNK_32(part7, bucket, offset + 24, reason, conf_fn);         \
        CONF_CHUNK_32(part8, bucket, offset + 28, reason, conf_fn);         \
        CONF_CHUNK_32(part9, bucket, offset + 32, reason, conf_fn);         \
        CONF_CHUNK_32(part10, bucket, offset + 36, reason, conf_fn);        \
        CONF_CHUNK_32(part11, bucket, offset + 40, reason, conf_fn);        \
        CONF_CHUNK_32(part12, bucket, offset + 44, reason, conf_fn);        \
        CONF_CHUNK_32(part13, bucket, offset + 48, reason, conf_fn);        \
        CONF_CHUNK_32(part14, bucket, offset + 52, reason, conf_fn);        \
        CONF_CHUNK_32(part15, bucket, offset + 56, reason, conf_fn);        \
        CONF_CHUNK_32(part16, bucket, offset + 60, reason, conf_fn);        \
    }                                                                       \
} while(0)
#endif

#define PREP_SHUF_MASK_NO_REINFORCEMENT(val)                                \
    m512 lo = and512(val, *lo_mask);                                        \
    m512 hi = and512(rshift64_m512(val, 4), *lo_mask)

#define PREP_SHUF_MASK                                                      \
    PREP_SHUF_MASK_NO_REINFORCEMENT(load512(ptr));                          \
    *c_16 = *(ptr + 15);                                                    \
    *c_32 = *(ptr + 31);                                                    \
    *c_48 = *(ptr + 47);                                                    \
    m512 r_msk = set512_64(0ULL, r_msk_base[*c_48], 0ULL, r_msk_base[*c_32],\
                           0ULL, r_msk_base[*c_16], 0ULL, r_msk_base[*c_0]);\
    *c_0 = *(ptr + 63)

#define SHIFT_OR_M1                                                         \
    or512(pshufb_m512(dup_mask[0], lo), pshufb_m512(dup_mask[1], hi))

#define SHIFT_OR_M2                                                         \
    or512(lshift128_m512(or512(pshufb_m512(dup_mask[2], lo),                \
                               pshufb_m512(dup_mask[3], hi)),               \
                         1), SHIFT_OR_M1)

#define SHIFT_OR_M3                                                         \
    or512(lshift128_m512(or512(pshufb_m512(dup_mask[4], lo),                \
                               pshufb_m512(dup_mask[5], hi)),               \
                         2), SHIFT_OR_M2)

#define SHIFT_OR_M4                                                         \
    or512(lshift128_m512(or512(pshufb_m512(dup_mask[6], lo),                \
                               pshufb_m512(dup_mask[7], hi)),               \
                         3), SHIFT_OR_M3)

static really_inline
m512 prep_conf_teddy_no_reinforcement_m1(const m512 *lo_mask,
                                         const m512 *dup_mask,
                                         const m512 val) {
    PREP_SHUF_MASK_NO_REINFORCEMENT(val);
    return SHIFT_OR_M1;
}

static really_inline
m512 prep_conf_teddy_no_reinforcement_m2(const m512 *lo_mask,
                                         const m512 *dup_mask,
                                         const m512 val) {
    PREP_SHUF_MASK_NO_REINFORCEMENT(val);
    return SHIFT_OR_M2;
}

static really_inline
m512 prep_conf_teddy_no_reinforcement_m3(const m512 *lo_mask,
                                         const m512 *dup_mask,
                                         const m512 val) {
    PREP_SHUF_MASK_NO_REINFORCEMENT(val);
    return SHIFT_OR_M3;
}

static really_inline
m512 prep_conf_teddy_no_reinforcement_m4(const m512 *lo_mask,
                                         const m512 *dup_mask,
                                         const m512 val) {
    PREP_SHUF_MASK_NO_REINFORCEMENT(val);
    return SHIFT_OR_M4;
}

static really_inline
m512 prep_conf_teddy_m1(const m512 *lo_mask, const m512 *dup_mask,
                        const u8 *ptr, const u64a *r_msk_base,
                        u32 *c_0, u32 *c_16, u32 *c_32, u32 *c_48) {
    PREP_SHUF_MASK;
    return or512(SHIFT_OR_M1, r_msk);
}

static really_inline
m512 prep_conf_teddy_m2(const m512 *lo_mask, const m512 *dup_mask,
                        const u8 *ptr, const u64a *r_msk_base,
                        u32 *c_0, u32 *c_16, u32 *c_32, u32 *c_48) {
    PREP_SHUF_MASK;
    return or512(SHIFT_OR_M2, r_msk);
}

static really_inline
m512 prep_conf_teddy_m3(const m512 *lo_mask, const m512 *dup_mask,
                        const u8 *ptr, const u64a *r_msk_base,
                        u32 *c_0, u32 *c_16, u32 *c_32, u32 *c_48) {
    PREP_SHUF_MASK;
    return or512(SHIFT_OR_M3, r_msk);
}

static really_inline
m512 prep_conf_teddy_m4(const m512 *lo_mask, const m512 *dup_mask,
                        const u8 *ptr, const u64a *r_msk_base,
                        u32 *c_0, u32 *c_16, u32 *c_32, u32 *c_48) {
    PREP_SHUF_MASK;
    return or512(SHIFT_OR_M4, r_msk);
}

#define PREP_CONF_FN_NO_REINFORCEMENT(val, n)                                 \
    prep_conf_teddy_no_reinforcement_m##n(&lo_mask, dup_mask, val)

#define PREP_CONF_FN(ptr, n)                                                  \
    prep_conf_teddy_m##n(&lo_mask, dup_mask, ptr, r_msk_base,                 \
                         &c_0, &c_16, &c_32, &c_48)

#define PREPARE_MASKS_1                                                       \
    dup_mask[0] = set4x128(maskBase[0]);                                      \
    dup_mask[1] = set4x128(maskBase[1]);

#define PREPARE_MASKS_2                                                       \
    PREPARE_MASKS_1                                                           \
    dup_mask[2] = set4x128(maskBase[2]);                                      \
    dup_mask[3] = set4x128(maskBase[3]);

#define PREPARE_MASKS_3                                                       \
    PREPARE_MASKS_2                                                           \
    dup_mask[4] = set4x128(maskBase[4]);                                      \
    dup_mask[5] = set4x128(maskBase[5]);

#define PREPARE_MASKS_4                                                       \
    PREPARE_MASKS_3                                                           \
    dup_mask[6] = set4x128(maskBase[6]);                                      \
    dup_mask[7] = set4x128(maskBase[7]);

#define PREPARE_MASKS(n)                                                      \
    m512 lo_mask = set64x8(0xf);                                              \
    m512 dup_mask[n * 2];                                                     \
    PREPARE_MASKS_##n

#define FDR_EXEC_TEDDY(fdr, a, control, n_msk, conf_fn)                       \
do {                                                                          \
    const u8 *buf_end = a->buf + a->len;                                      \
    const u8 *ptr = a->buf + a->start_offset;                                 \
    u32 floodBackoff = FLOOD_BACKOFF_START;                                   \
    const u8 *tryFloodDetect = a->firstFloodDetect;                           \
    u32 last_match = ones_u32;                                                \
    const struct Teddy *teddy = (const struct Teddy *)fdr;                    \
    const size_t iterBytes = 128;                                             \
    DEBUG_PRINTF("params: buf %p len %zu start_offset %zu\n",                 \
                 a->buf, a->len, a->start_offset);                            \
                                                                              \
    const m128 *maskBase = getMaskBase(teddy);                                \
    PREPARE_MASKS(n_msk);                                                     \
    const u32 *confBase = getConfBase(teddy);                                 \
                                                                              \
    const u64a *r_msk_base = getReinforcedMaskBase(teddy, n_msk);             \
    u32 c_0 = 0x100;                                                          \
    u32 c_16 = 0x100;                                                         \
    u32 c_32 = 0x100;                                                         \
    u32 c_48 = 0x100;                                                         \
    const u8 *mainStart = ROUNDUP_PTR(ptr, 64);                               \
    DEBUG_PRINTF("derive: ptr: %p mainstart %p\n", ptr, mainStart);           \
    if (ptr < mainStart) {                                                    \
        ptr = mainStart - 64;                                                 \
        m512 p_mask;                                                          \
        m512 val_0 = vectoredLoad512(&p_mask, ptr, a->start_offset,           \
                                     a->buf, buf_end,                         \
                                     a->buf_history, a->len_history, n_msk);  \
        m512 r_0 = PREP_CONF_FN_NO_REINFORCEMENT(val_0, n_msk);               \
        r_0 = or512(r_0, p_mask);                                             \
        CONFIRM_TEDDY(r_0, 8, 0, VECTORING, conf_fn);                         \
        ptr += 64;                                                            \
    }                                                                         \
                                                                              \
    if (ptr + 64 <= buf_end) {                                                \
        m512 r_0 = PREP_CONF_FN(ptr, n_msk);                                  \
        CONFIRM_TEDDY(r_0, 8, 0, VECTORING, conf_fn);                         \
        ptr += 64;                                                            \
    }                                                                         \
                                                                              \
    for (; ptr + iterBytes <= buf_end; ptr += iterBytes) {                    \
        __builtin_prefetch(ptr + (iterBytes * 4));                            \
        CHECK_FLOOD;                                                          \
        m512 r_0 = PREP_CONF_FN(ptr, n_msk);                                  \
        CONFIRM_TEDDY(r_0, 8, 0, NOT_CAUTIOUS, conf_fn);                      \
        m512 r_1 = PREP_CONF_FN(ptr + 64, n_msk);                             \
        CONFIRM_TEDDY(r_1, 8, 64, NOT_CAUTIOUS, conf_fn);                     \
    }                                                                         \
                                                                              \
    if (ptr + 64 <= buf_end) {                                                \
        m512 r_0 = PREP_CONF_FN(ptr, n_msk);                                  \
        CONFIRM_TEDDY(r_0, 8, 0, NOT_CAUTIOUS, conf_fn);                      \
        ptr += 64;                                                            \
    }                                                                         \
                                                                              \
    assert(ptr + 64 > buf_end);                                               \
    if (ptr < buf_end) {                                                      \
        m512 p_mask;                                                          \
        m512 val_0 = vectoredLoad512(&p_mask, ptr, 0, ptr, buf_end,           \
                                     a->buf_history, a->len_history, n_msk);  \
        m512 r_0 = PREP_CONF_FN_NO_REINFORCEMENT(val_0, n_msk);               \
        r_0 = or512(r_0, p_mask);                                             \
        CONFIRM_TEDDY(r_0, 8, 0, VECTORING, conf_fn);                         \
    }                                                                         \
                                                                              \
    return HWLM_SUCCESS;                                                      \
} while(0)

#elif defined(HAVE_AVX2) // not HAVE_AVX512 but HAVE_AVX2 reinforced teddy

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
        CONF_CHUNK_64(part1, bucket, offset, reason, conf_fn);              \
        CONF_CHUNK_64(part2, bucket, offset + 8, reason, conf_fn);          \
        CONF_CHUNK_64(part3, bucket, offset + 16, reason, conf_fn);         \
        CONF_CHUNK_64(part4, bucket, offset + 24, reason, conf_fn);         \
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
        CONF_CHUNK_32(part1, bucket, offset, reason, conf_fn);              \
        CONF_CHUNK_32(part2, bucket, offset + 4, reason, conf_fn);          \
        CONF_CHUNK_32(part3, bucket, offset + 8, reason, conf_fn);          \
        CONF_CHUNK_32(part4, bucket, offset + 12, reason, conf_fn);         \
        CONF_CHUNK_32(part5, bucket, offset + 16, reason, conf_fn);         \
        CONF_CHUNK_32(part6, bucket, offset + 20, reason, conf_fn);         \
        CONF_CHUNK_32(part7, bucket, offset + 24, reason, conf_fn);         \
        CONF_CHUNK_32(part8, bucket, offset + 28, reason, conf_fn);         \
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

#define FDR_EXEC_TEDDY(fdr, a, control, n_msk, conf_fn)                       \
do {                                                                          \
    const u8 *buf_end = a->buf + a->len;                                      \
    const u8 *ptr = a->buf + a->start_offset;                                 \
    u32 floodBackoff = FLOOD_BACKOFF_START;                                   \
    const u8 *tryFloodDetect = a->firstFloodDetect;                           \
    u32 last_match = ones_u32;                                                \
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

#else // not defined HAVE_AVX2

#ifdef ARCH_64_BIT
#define CONFIRM_TEDDY(var, bucket, offset, reason, conf_fn)                 \
do {                                                                        \
    if (unlikely(diff128(var, ones128()))) {                                \
        u64a lo = movq(var);                                                \
        u64a hi = movq(rshiftbyte_m128(var, 8));                            \
        CONF_CHUNK_64(lo, bucket, offset, reason, conf_fn);                 \
        CONF_CHUNK_64(hi, bucket, offset + 8, reason, conf_fn);             \
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
        CONF_CHUNK_32(part1, bucket, offset, reason, conf_fn);              \
        CONF_CHUNK_32(part2, bucket, offset + 4, reason, conf_fn);          \
        CONF_CHUNK_32(part3, bucket, offset + 8, reason, conf_fn);          \
        CONF_CHUNK_32(part4, bucket, offset + 12, reason, conf_fn);         \
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

#define FDR_EXEC_TEDDY(fdr, a, control, n_msk, conf_fn)                       \
do {                                                                          \
    const u8 *buf_end = a->buf + a->len;                                      \
    const u8 *ptr = a->buf + a->start_offset;                                 \
    u32 floodBackoff = FLOOD_BACKOFF_START;                                   \
    const u8 *tryFloodDetect = a->firstFloodDetect;                           \
    u32 last_match = ones_u32;                                                \
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

#endif // HAVE_AVX2 HAVE_AVX512

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
