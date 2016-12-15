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
 * \brief Teddy literal matcher: common runtime procedures.
 */

#ifndef TEDDY_RUNTIME_COMMON_H_
#define TEDDY_RUNTIME_COMMON_H_

#include "fdr_confirm.h"
#include "fdr_confirm_runtime.h"
#include "ue2common.h"
#include "util/bitutils.h"
#include "util/simd_utils.h"

extern const u8 ALIGN_DIRECTIVE p_mask_arr[17][32];

#ifdef ARCH_64_BIT
#define TEDDY_CONF_TYPE u64a
#define TEDDY_FIND_AND_CLEAR_LSB(conf) findAndClearLSB_64(conf)
#else
#define TEDDY_CONF_TYPE u32
#define TEDDY_FIND_AND_CLEAR_LSB(conf) findAndClearLSB_32(conf)
#endif

#define CHECK_HWLM_TERMINATE_MATCHING                                       \
do {                                                                        \
    if (unlikely(control == HWLM_TERMINATE_MATCHING)) {                     \
        return HWLM_TERMINATED;                                             \
    }                                                                       \
} while (0);

#define CHECK_FLOOD                                                         \
do {                                                                        \
    if (unlikely(ptr > tryFloodDetect)) {                                   \
        tryFloodDetect = floodDetect(fdr, a, &ptr, tryFloodDetect,          \
                                     &floodBackoff, &control, iterBytes);   \
        CHECK_HWLM_TERMINATE_MATCHING;                                      \
    }                                                                       \
} while (0);

/*
 * \brief Copy a block of [0,15] bytes efficiently.
 *
 * This function is a workaround intended to stop some compilers from
 * synthesizing a memcpy function call out of the copy of a small number of
 * bytes that we do in vectoredLoad128.
 */
static really_inline
void copyRuntBlock128(u8 *dst, const u8 *src, size_t len) {
    switch (len) {
    case 0:
        break;
    case 1:
        *dst = *src;
        break;
    case 2:
        unaligned_store_u16(dst, unaligned_load_u16(src));
        break;
    case 3:
        unaligned_store_u16(dst, unaligned_load_u16(src));
        dst[2] = src[2];
        break;
    case 4:
        unaligned_store_u32(dst, unaligned_load_u32(src));
        break;
    case 5:
    case 6:
    case 7:
        /* Perform copy with two overlapping 4-byte chunks. */
        unaligned_store_u32(dst + len - 4, unaligned_load_u32(src + len - 4));
        unaligned_store_u32(dst, unaligned_load_u32(src));
        break;
    case 8:
        unaligned_store_u64a(dst, unaligned_load_u64a(src));
        break;
    default:
        /* Perform copy with two overlapping 8-byte chunks. */
        assert(len < 16);
        unaligned_store_u64a(dst + len - 8, unaligned_load_u64a(src + len - 8));
        unaligned_store_u64a(dst, unaligned_load_u64a(src));
        break;
    }
}

// Note: p_mask is an output param that initialises a poison mask.
static really_inline
m128 vectoredLoad128(m128 *p_mask, const u8 *ptr, const u8 *lo, const u8 *hi,
                     const u8 *buf_history, size_t len_history,
                     const u32 nMasks) {
    union {
        u8 val8[16];
        m128 val128;
    } u;
    u.val128 = zeroes128();

    uintptr_t copy_start;
    uintptr_t copy_len;

    if (ptr >= lo) {
        uintptr_t avail = (uintptr_t)(hi - ptr);
        if (avail >= 16) {
            *p_mask = load128(p_mask_arr[16] + 16);
            return loadu128(ptr);
        }
        *p_mask = load128(p_mask_arr[avail] + 16);
        copy_start = 0;
        copy_len = avail;
    } else {
        uintptr_t need = MIN((uintptr_t)(lo - ptr),
                             MIN(len_history, nMasks - 1));
        uintptr_t start = (uintptr_t)(lo - ptr);
        uintptr_t i;
        for (i = start - need; ptr + i < lo; i++) {
            u.val8[i] = buf_history[len_history - (lo - (ptr + i))];
        }
        uintptr_t end = MIN(16, (uintptr_t)(hi - ptr));
        *p_mask = loadu128(p_mask_arr[end - start] + 16 - start);
        copy_start = i;
        copy_len = end - i;
    }

    // Runt block from the buffer.
    copyRuntBlock128(&u.val8[copy_start], &ptr[copy_start], copy_len);

    return u.val128;
}

static really_inline
u64a getConfVal(const struct FDR_Runtime_Args *a, const u8 *ptr, u32 byte,
                CautionReason reason) {
    u64a confVal = 0;
    const u8 *buf = a->buf;
    size_t len = a->len;
    const u8 *confirm_loc = ptr + byte - 7;
    if (likely(reason == NOT_CAUTIOUS || confirm_loc >= buf)) {
        confVal = lv_u64a(confirm_loc, buf, buf + len);
    } else { // r == VECTORING, confirm_loc < buf
        u64a histBytes = a->histBytes;
        confVal = lv_u64a_ce(confirm_loc, buf, buf + len);
        // stitch together confVal and history
        u32 overhang = buf - confirm_loc;
        histBytes >>= 64 - (overhang * 8);
        confVal |= histBytes;
    }
    return confVal;
}

static really_inline
void do_confWithBit_teddy(TEDDY_CONF_TYPE *conf, u8 bucket, u8 offset,
                          const u32 *confBase, CautionReason reason,
                          const struct FDR_Runtime_Args *a, const u8 *ptr,
                          hwlmcb_rv_t *control, u32 *last_match) {
    do  {
        u32 bit = TEDDY_FIND_AND_CLEAR_LSB(conf);
        u32 byte = bit / bucket + offset;
        u32 idx  = bit % bucket;
        u32 cf = confBase[idx];
        if (!cf) {
            continue;
        }
        const struct FDRConfirm *fdrc = (const struct FDRConfirm *)
                                        ((const u8 *)confBase + cf);
        if (!(fdrc->groups & *control)) {
            continue;
        }
        u64a confVal = getConfVal(a, ptr, byte, reason);
        confWithBit(fdrc, a, ptr - a->buf + byte, control,
                    last_match, confVal);
    } while (unlikely(*conf));
}

static really_inline
void do_confWithBit1_teddy(TEDDY_CONF_TYPE *conf, u8 bucket, u8 offset,
                           const u32 *confBase, CautionReason reason,
                           const struct FDR_Runtime_Args *a, const u8 *ptr,
                           hwlmcb_rv_t *control, u32 *last_match) {
    do {
        u32 bit = TEDDY_FIND_AND_CLEAR_LSB(conf);
        u32 byte = bit / bucket + offset;
        u32 idx  = bit % bucket;
        u32 cf = confBase[idx];
        const struct FDRConfirm *fdrc = (const struct FDRConfirm *)
                                        ((const u8 *)confBase + cf);
        if (!(fdrc->groups & *control)) {
            continue;
        }
        u64a confVal = getConfVal(a, ptr, byte, reason);
        confWithBit1(fdrc, a, ptr - a->buf + byte, control, last_match,
                     confVal);
    } while (unlikely(*conf));
}

static really_inline
void do_confWithBitMany_teddy(TEDDY_CONF_TYPE *conf, u8 bucket, u8 offset,
                              const u32 *confBase, CautionReason reason,
                              const struct FDR_Runtime_Args *a, const u8 *ptr,
                              hwlmcb_rv_t *control, u32 *last_match) {
    do {
        u32 bit = TEDDY_FIND_AND_CLEAR_LSB(conf);
        u32 byte = bit / bucket + offset;
        u32 idx = bit % bucket;
        u32 cf = confBase[idx];
        const struct FDRConfirm *fdrc = (const struct FDRConfirm *)
                                        ((const u8 *)confBase + cf);
        if (!(fdrc->groups & *control)) {
            continue;
        }
        u64a confVal = getConfVal(a, ptr, byte, reason);
        confWithBitMany(fdrc, a, ptr - a->buf + byte, reason, control,
                        last_match, confVal);
    } while (unlikely(*conf));
}

static really_inline
const m128 * getMaskBase(const struct Teddy *teddy) {
    return (const m128 *)((const u8 *)teddy + sizeof(struct Teddy));
}

static really_inline
const u32 * getConfBase(const struct Teddy *teddy, u8 numMask) {
    return (const u32 *)((const u8 *)teddy + sizeof(struct Teddy) +
                         (numMask*32));
}

#endif /* TEDDY_RUNTIME_COMMON_H_ */
