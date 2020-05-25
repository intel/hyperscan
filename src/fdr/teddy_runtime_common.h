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
#include "util/uniform_ops.h"

extern const u8 ALIGN_DIRECTIVE p_mask_arr[17][32];
#if defined(HAVE_AVX2)
extern const u8 ALIGN_AVX_DIRECTIVE p_mask_arr256[33][64];
#endif

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
//       *p_mask = load128(p_mask_arr[n] + 16 - m) means:
//       m byte 0xff in the beginning, followed by n byte 0x00,
//       then followed by the rest bytes 0xff.
// ptr >= lo:
//     no history.
//     for end/short zone, ptr==lo and start_offset==0
//     for start zone, see below
//          lo         ptr                      hi           hi
//          |----------|-------|----------------|............|
//          -start     0       -start+offset    MIN(avail,16)
// p_mask              ffff..ff0000...........00ffff..........
// ptr < lo:
//     only start zone.
//             history
//          ptr        lo                       hi           hi
//          |----------|-------|----------------|............|
//          0          start   start+offset     end(<=16)
// p_mask   ffff.....ffffff..ff0000...........00ffff..........
static really_inline
m128 vectoredLoad128(m128 *p_mask, const u8 *ptr, const size_t start_offset,
                     const u8 *lo, const u8 *hi,
                     const u8 *buf_history, size_t len_history,
                     const u32 nMasks) {
    union {
        u8 val8[16];
        m128 val128;
    } u;
    u.val128 = zeroes128();

    uintptr_t copy_start;
    uintptr_t copy_len;

    if (ptr >= lo) { // short/end/start zone
        uintptr_t start = (uintptr_t)(ptr - lo);
        uintptr_t avail = (uintptr_t)(hi - ptr);
        if (avail >= 16) {
            assert(start_offset - start <= 16);
            *p_mask = loadu128(p_mask_arr[16 - start_offset + start]
                               + 16 - start_offset + start);
            return loadu128(ptr);
        }
        assert(start_offset - start <= avail);
        *p_mask = loadu128(p_mask_arr[avail - start_offset + start]
                           + 16 - start_offset + start);
        copy_start = 0;
        copy_len = avail;
    } else { // start zone
        uintptr_t need = MIN((uintptr_t)(lo - ptr),
                             MIN(len_history, nMasks - 1));
        uintptr_t start = (uintptr_t)(lo - ptr);
        uintptr_t i;
        for (i = start - need; i < start; i++) {
            u.val8[i] = buf_history[len_history - (start - i)];
        }
        uintptr_t end = MIN(16, (uintptr_t)(hi - ptr));
        assert(start + start_offset <= end);
        *p_mask = loadu128(p_mask_arr[end - start - start_offset]
                           + 16 - start - start_offset);
        copy_start = start;
        copy_len = end - start;
    }

    // Runt block from the buffer.
    copyRuntBlock128(&u.val8[copy_start], &ptr[copy_start], copy_len);

    return u.val128;
}

#if defined(HAVE_AVX2)
/*
 * \brief Copy a block of [0,31] bytes efficiently.
 *
 * This function is a workaround intended to stop some compilers from
 * synthesizing a memcpy function call out of the copy of a small number of
 * bytes that we do in vectoredLoad256.
 */
static really_inline
void copyRuntBlock256(u8 *dst, const u8 *src, size_t len) {
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
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
        /* Perform copy with two overlapping 8-byte chunks. */
        unaligned_store_u64a(dst + len - 8, unaligned_load_u64a(src + len - 8));
        unaligned_store_u64a(dst, unaligned_load_u64a(src));
        break;
    case 16:
        storeu128(dst, loadu128(src));
        break;
    default:
        /* Perform copy with two overlapping 16-byte chunks. */
        assert(len < 32);
        storeu128(dst + len - 16, loadu128(src + len - 16));
        storeu128(dst, loadu128(src));
        break;
    }
}

// Note: p_mask is an output param that initialises a poison mask.
//       *p_mask = load256(p_mask_arr256[n] + 32 - m) means:
//       m byte 0xff in the beginning, followed by n byte 0x00,
//       then followed by the rest bytes 0xff.
// ptr >= lo:
//     no history.
//     for end/short zone, ptr==lo and start_offset==0
//     for start zone, see below
//          lo         ptr                      hi           hi
//          |----------|-------|----------------|............|
//          -start     0       -start+offset    MIN(avail,32)
// p_mask              ffff..ff0000...........00ffff..........
// ptr < lo:
//     only start zone.
//             history
//          ptr        lo                       hi           hi
//          |----------|-------|----------------|............|
//          0          start   start+offset     end(<=32)
// p_mask   ffff.....ffffff..ff0000...........00ffff..........
static really_inline
m256 vectoredLoad256(m256 *p_mask, const u8 *ptr, const size_t start_offset,
                     const u8 *lo, const u8 *hi,
                     const u8 *buf_history, size_t len_history,
                     const u32 nMasks) {
    union {
        u8 val8[32];
        m256 val256;
    } u;
    u.val256 = zeroes256();

    uintptr_t copy_start;
    uintptr_t copy_len;

    if (ptr >= lo) { // short/end/start zone
        uintptr_t start = (uintptr_t)(ptr - lo);
        uintptr_t avail = (uintptr_t)(hi - ptr);
        if (avail >= 32) {
            assert(start_offset - start <= 32);
            *p_mask = loadu256(p_mask_arr256[32 - start_offset + start]
                               + 32 - start_offset + start);
            return loadu256(ptr);
        }
        assert(start_offset - start <= avail);
        *p_mask = loadu256(p_mask_arr256[avail - start_offset + start]
                           + 32 - start_offset + start);
        copy_start = 0;
        copy_len = avail;
    } else { //start zone
        uintptr_t need = MIN((uintptr_t)(lo - ptr),
                             MIN(len_history, nMasks - 1));
        uintptr_t start = (uintptr_t)(lo - ptr);
        uintptr_t i;
        for (i = start - need; i < start; i++) {
            u.val8[i] = buf_history[len_history - (start - i)];
        }
        uintptr_t end = MIN(32, (uintptr_t)(hi - ptr));
        assert(start + start_offset <= end);
        *p_mask = loadu256(p_mask_arr256[end - start - start_offset]
                           + 32 - start - start_offset);
        copy_start = start;
        copy_len = end - start;
    }

    // Runt block from the buffer.
    copyRuntBlock256(&u.val8[copy_start], &ptr[copy_start], copy_len);

    return u.val256;
}
#endif // HAVE_AVX2

#if defined(HAVE_AVX512)
// Note: p_mask is an output param that initialises a poison mask.
//       u64a k = ones_u64a << n' >> m'; // m' < n'
//       *p_mask = set_mask_m512(~k);
//       means p_mask is consist of:
//       (n' - m') poison bytes "0xff" at the beginning,
//       followed by (64 - n') valid bytes "0x00",
//       then followed by the rest m' poison bytes "0xff".
// ptr >= lo:
//     no history.
//     for end/short zone, ptr==lo and start_offset==0
//     for start zone, see below
//          lo         ptr                      hi           hi
//          |----------|-------|----------------|............|
//          -start     0       -start+offset    MIN(avail,64)
// p_mask              ffff..ff0000...........00ffff..........
// ptr < lo:
//     only start zone.
//             history
//          ptr        lo                       hi           hi
//          |----------|-------|----------------|............|
//          0          start   start+offset     end(<=64)
// p_mask   ffff.....ffffff..ff0000...........00ffff..........
static really_inline
m512 vectoredLoad512(m512 *p_mask, const u8 *ptr, const size_t start_offset,
                     const u8 *lo, const u8 *hi, const u8 *hbuf, size_t hlen,
                     const u32 nMasks) {
    m512 val;

    uintptr_t copy_start;
    uintptr_t copy_len;

    if (ptr >= lo) { // short/end/start zone
        uintptr_t start = (uintptr_t)(ptr - lo);
        uintptr_t avail = (uintptr_t)(hi - ptr);
        if (avail >= 64) {
            assert(start_offset - start <= 64);
            u64a k = ones_u64a << (start_offset - start);
            *p_mask = set_mask_m512(~k);
            return loadu512(ptr);
        }
        assert(start_offset - start <= avail);
        u64a k = ones_u64a << (64 - avail + start_offset - start)
                           >> (64 - avail);
        *p_mask = set_mask_m512(~k);
        copy_start = 0;
        copy_len = avail;
    } else { //start zone
        uintptr_t need = MIN((uintptr_t)(lo - ptr),
                             MIN(hlen, nMasks - 1));
        uintptr_t start = (uintptr_t)(lo - ptr);
        u64a j = 0x7fffffffffffffffULL >> (63 - need) << (start - need);
        val = loadu_maskz_m512(j, &hbuf[hlen - start]);
        uintptr_t end = MIN(64, (uintptr_t)(hi - ptr));
        assert(start + start_offset <= end);
        u64a k = ones_u64a << (64 - end + start + start_offset) >> (64 - end);
        *p_mask = set_mask_m512(~k);
        copy_start = start;
        copy_len = end - start;
    }

    assert(copy_len < 64);
    assert(copy_len > 0);
    u64a j = ones_u64a >> (64 - copy_len) << copy_start;
    val = loadu_mask_m512(val, j, ptr);

    return val;
}
#endif // HAVE_AVX512

static really_inline
u64a getConfVal(const struct FDR_Runtime_Args *a, const u8 *ptr, u32 byte,
                UNUSED CautionReason reason) {
    u64a confVal = 0;
    const u8 *buf = a->buf;
    size_t len = a->len;
    const u8 *confirm_loc = ptr + byte - 7;
#if defined(HAVE_AVX512VBMI)
    if (likely(confirm_loc >= buf)) {
#else
    if (likely(reason == NOT_CAUTIOUS || confirm_loc >= buf)) {
#endif
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
        u64a tmp = 0;
        u64a confVal = getConfVal(a, ptr, byte, reason);
        confWithBit(fdrc, a, ptr - a->buf + byte, control,
                    last_match, confVal, &tmp, 0);
    } while (unlikely(*conf));
}

static really_inline
const m128 *getMaskBase(const struct Teddy *teddy) {
    return (const m128 *)((const u8 *)teddy + ROUNDUP_CL(sizeof(struct Teddy)));
}

static really_inline
const u64a *getReinforcedMaskBase(const struct Teddy *teddy, u8 numMask) {
    return (const u64a *)((const u8 *)getMaskBase(teddy)
                          + ROUNDUP_CL(2 * numMask * sizeof(m128)));
}

static really_inline
const u32 *getConfBase(const struct Teddy *teddy) {
    return (const u32 *)((const u8 *)teddy + teddy->confOffset);
}

#endif /* TEDDY_RUNTIME_COMMON_H_ */
