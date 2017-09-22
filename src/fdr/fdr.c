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

#include "fdr.h"
#include "fdr_confirm.h"
#include "fdr_confirm_runtime.h"
#include "fdr_internal.h"
#include "fdr_loadval.h"
#include "flood_runtime.h"
#include "scratch.h"
#include "teddy.h"
#include "teddy_internal.h"
#include "util/arch.h"
#include "util/simd_utils.h"
#include "util/uniform_ops.h"

/** \brief number of bytes processed in each iteration */
#define ITER_BYTES          16

/** \brief total zone buffer size */
#define ZONE_TOTAL_SIZE     64

/** \brief maximum number of allowed zones */
#define ZONE_MAX            3

/** \brief zone information.
 *
 * Zone represents a region of data to scan in FDR.
 *
 * The incoming buffer is to split in multiple zones to ensure two properties:
 * 1: that we can read 8? bytes behind to generate a hash safely
 * 2: that we can read the 3 byte after the current byte (domain > 8)
 */
struct zone {
    /** \brief copied buffer, used only when it is a boundary zone. */
    u8 ALIGN_CL_DIRECTIVE buf[ZONE_TOTAL_SIZE];

    /** \brief shift amount for fdr state to avoid unwanted match. */
    u8 shift;

    /** \brief if boundary zone, start points into the zone buffer after the
     * pre-padding. Otherwise, points to the main buffer, appropriately. */
    const u8 *start;

    /** \brief if boundary zone, end points to the end of zone. Otherwise,
     * pointer to the main buffer, appropriately. */
    const u8 *end;

    /** \brief the amount to adjust to go from a pointer in the zones region
     * (between start and end) to a pointer in the original data buffer. */
    ptrdiff_t zone_pointer_adjust;

    /** \brief firstFloodDetect from FDR_Runtime_Args for non-boundary zones,
     * otherwise end of the zone buf. floodPtr always points inside the same
     * buffer as the start pointe. */
    const u8 *floodPtr;
};

static
const ALIGN_CL_DIRECTIVE u8 zone_or_mask[ITER_BYTES+1][ITER_BYTES] = {
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
      0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
      0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
      0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
      0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00 },
    { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
      0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00 },
    { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00 },
    { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
};

/* compilers don't reliably synthesize the 32-bit ANDN instruction here,
 * so we force its generation.
 */
static really_inline
u64a andn(const u32 a, const u8 *b) {
    u64a r;
#if defined(HAVE_BMI) && !defined(NO_ASM)
    __asm__ ("andn\t%2,%1,%k0" : "=r"(r) : "r"(a), "m"(*(const u32 *)b));
#else
    r = unaligned_load_u32(b) & ~a;
#endif
    return r;
}

/* generates an initial state mask based on the last byte-ish of history rather
 * than being all accepting. If there is no history to consider, the state is
 * generated based on the minimum length of each bucket in order to prevent
 * confirms.
 */
static really_inline
m128 getInitState(const struct FDR *fdr, u8 len_history, const u64a *ft,
                  const struct zone *z) {
    m128 s;
    if (len_history) {
        /* +1: the zones ensure that we can read the byte at z->end */
        u32 tmp = lv_u16(z->start + z->shift - 1, z->buf, z->end + 1);
        tmp &= fdr->domainMask;
        s = load_m128_from_u64a(ft + tmp);
        s = rshiftbyte_m128(s, 1);
    } else {
        s = fdr->start;
    }
    return s;
}

static really_inline
void get_conf_stride_1(const u8 *itPtr, UNUSED const u8 *start_ptr,
                       UNUSED const u8 *end_ptr, u32 domain_mask_flipped,
                       const u64a *ft, u64a *conf0, u64a *conf8, m128 *s) {
    /* +1: the zones ensure that we can read the byte at z->end */
    assert(itPtr >= start_ptr && itPtr + ITER_BYTES <= end_ptr);
    u64a reach0 = andn(domain_mask_flipped, itPtr);
    u64a reach1 = andn(domain_mask_flipped, itPtr + 1);
    u64a reach2 = andn(domain_mask_flipped, itPtr + 2);
    u64a reach3 = andn(domain_mask_flipped, itPtr + 3);

    m128 st0 = load_m128_from_u64a(ft + reach0);
    m128 st1 = load_m128_from_u64a(ft + reach1);
    m128 st2 = load_m128_from_u64a(ft + reach2);
    m128 st3 = load_m128_from_u64a(ft + reach3);

    u64a reach4 = andn(domain_mask_flipped, itPtr + 4);
    u64a reach5 = andn(domain_mask_flipped, itPtr + 5);
    u64a reach6 = andn(domain_mask_flipped, itPtr + 6);
    u64a reach7 = andn(domain_mask_flipped, itPtr + 7);

    m128 st4 = load_m128_from_u64a(ft + reach4);
    m128 st5 = load_m128_from_u64a(ft + reach5);
    m128 st6 = load_m128_from_u64a(ft + reach6);
    m128 st7 = load_m128_from_u64a(ft + reach7);

    st1 = lshiftbyte_m128(st1, 1);
    st2 = lshiftbyte_m128(st2, 2);
    st3 = lshiftbyte_m128(st3, 3);
    st4 = lshiftbyte_m128(st4, 4);
    st5 = lshiftbyte_m128(st5, 5);
    st6 = lshiftbyte_m128(st6, 6);
    st7 = lshiftbyte_m128(st7, 7);

    st0 = or128(st0, st1);
    st2 = or128(st2, st3);
    st4 = or128(st4, st5);
    st6 = or128(st6, st7);
    st0 = or128(st0, st2);
    st4 = or128(st4, st6);
    st0 = or128(st0, st4);
    *s = or128(*s, st0);

    *conf0 = movq(*s);
    *s = rshiftbyte_m128(*s, 8);
    *conf0 ^= ~0ULL;

    u64a reach8 = andn(domain_mask_flipped, itPtr + 8);
    u64a reach9 = andn(domain_mask_flipped, itPtr + 9);
    u64a reach10 = andn(domain_mask_flipped, itPtr + 10);
    u64a reach11 = andn(domain_mask_flipped, itPtr + 11);

    m128 st8 = load_m128_from_u64a(ft + reach8);
    m128 st9 = load_m128_from_u64a(ft + reach9);
    m128 st10 = load_m128_from_u64a(ft + reach10);
    m128 st11 = load_m128_from_u64a(ft + reach11);

    u64a reach12 = andn(domain_mask_flipped, itPtr + 12);
    u64a reach13 = andn(domain_mask_flipped, itPtr + 13);
    u64a reach14 = andn(domain_mask_flipped, itPtr + 14);
    u64a reach15 = andn(domain_mask_flipped, itPtr + 15);

    m128 st12 = load_m128_from_u64a(ft + reach12);
    m128 st13 = load_m128_from_u64a(ft + reach13);
    m128 st14 = load_m128_from_u64a(ft + reach14);
    m128 st15 = load_m128_from_u64a(ft + reach15);

    st9 = lshiftbyte_m128(st9, 1);
    st10 = lshiftbyte_m128(st10, 2);
    st11 = lshiftbyte_m128(st11, 3);
    st12 = lshiftbyte_m128(st12, 4);
    st13 = lshiftbyte_m128(st13, 5);
    st14 = lshiftbyte_m128(st14, 6);
    st15 = lshiftbyte_m128(st15, 7);

    st8 = or128(st8, st9);
    st10 = or128(st10, st11);
    st12 = or128(st12, st13);
    st14 = or128(st14, st15);
    st8 = or128(st8, st10);
    st12 = or128(st12, st14);
    st8 = or128(st8, st12);
    *s = or128(*s, st8);

    *conf8 = movq(*s);
    *s = rshiftbyte_m128(*s, 8);
    *conf8 ^= ~0ULL;
}

static really_inline
void get_conf_stride_2(const u8 *itPtr, UNUSED const u8 *start_ptr,
                       UNUSED const u8 *end_ptr, u32 domain_mask_flipped,
                       const u64a *ft, u64a *conf0, u64a *conf8, m128 *s) {
    assert(itPtr >= start_ptr && itPtr + ITER_BYTES <= end_ptr);
    u64a reach0 = andn(domain_mask_flipped, itPtr);
    u64a reach2 = andn(domain_mask_flipped, itPtr + 2);
    u64a reach4 = andn(domain_mask_flipped, itPtr + 4);
    u64a reach6 = andn(domain_mask_flipped, itPtr + 6);

    m128 st0 = load_m128_from_u64a(ft + reach0);
    m128 st2 = load_m128_from_u64a(ft + reach2);
    m128 st4 = load_m128_from_u64a(ft + reach4);
    m128 st6 = load_m128_from_u64a(ft + reach6);

    u64a reach8 = andn(domain_mask_flipped, itPtr + 8);
    u64a reach10 = andn(domain_mask_flipped, itPtr + 10);
    u64a reach12 = andn(domain_mask_flipped, itPtr + 12);
    u64a reach14 = andn(domain_mask_flipped, itPtr + 14);

    m128 st8 = load_m128_from_u64a(ft + reach8);
    m128 st10 = load_m128_from_u64a(ft + reach10);
    m128 st12 = load_m128_from_u64a(ft + reach12);
    m128 st14 = load_m128_from_u64a(ft + reach14);

    st2  = lshiftbyte_m128(st2, 2);
    st4  = lshiftbyte_m128(st4, 4);
    st6  = lshiftbyte_m128(st6, 6);

    *s = or128(*s, st0);
    *s = or128(*s, st2);
    *s = or128(*s, st4);
    *s = or128(*s, st6);

    *conf0 = movq(*s);
    *s = rshiftbyte_m128(*s, 8);
    *conf0 ^= ~0ULL;

    st10 = lshiftbyte_m128(st10, 2);
    st12 = lshiftbyte_m128(st12, 4);
    st14 = lshiftbyte_m128(st14, 6);

    *s = or128(*s, st8);
    *s = or128(*s, st10);
    *s = or128(*s, st12);
    *s = or128(*s, st14);

    *conf8 = movq(*s);
    *s = rshiftbyte_m128(*s, 8);
    *conf8 ^= ~0ULL;
}

static really_inline
void get_conf_stride_4(const u8 *itPtr, UNUSED const u8 *start_ptr,
                       UNUSED const u8 *end_ptr, u32 domain_mask_flipped,
                       const u64a *ft, u64a *conf0, u64a *conf8, m128 *s) {
    assert(itPtr >= start_ptr && itPtr + ITER_BYTES <= end_ptr);
    u64a reach0 = andn(domain_mask_flipped, itPtr);
    u64a reach4 = andn(domain_mask_flipped, itPtr + 4);
    u64a reach8 = andn(domain_mask_flipped, itPtr + 8);
    u64a reach12 = andn(domain_mask_flipped, itPtr + 12);

    m128 st0 = load_m128_from_u64a(ft + reach0);
    m128 st4 = load_m128_from_u64a(ft + reach4);
    m128 st8 = load_m128_from_u64a(ft + reach8);
    m128 st12 = load_m128_from_u64a(ft + reach12);

    st4 = lshiftbyte_m128(st4, 4);
    st12 = lshiftbyte_m128(st12, 4);

    *s = or128(*s, st0);
    *s = or128(*s, st4);
    *conf0 = movq(*s);
    *s = rshiftbyte_m128(*s, 8);
    *conf0 ^= ~0ULL;

    *s = or128(*s, st8);
    *s = or128(*s, st12);
    *conf8 = movq(*s);
    *s = rshiftbyte_m128(*s, 8);
    *conf8 ^= ~0ULL;
}

static really_inline
void do_confirm_fdr(u64a *conf, u8 offset, hwlmcb_rv_t *control,
                    const u32 *confBase, const struct FDR_Runtime_Args *a,
                    const u8 *ptr, u32 *last_match_id, struct zone *z) {
    const u8 bucket = 8;

    if (likely(!*conf)) {
        return;
    }

    /* ptr is currently referring to a location in the zone's buffer, we also
     * need a pointer in the original, main buffer for the final string compare.
     */
    const u8 *ptr_main = (const u8 *)((uintptr_t)ptr + z->zone_pointer_adjust);

    const u8 *confLoc = ptr;

    do  {
        u32 bit = findAndClearLSB_64(conf);
        u32 byte = bit / bucket + offset;
        u32 bitRem = bit % bucket;
        u32 idx = bitRem;
        u32 cf = confBase[idx];
        if (!cf) {
            continue;
        }
        const struct FDRConfirm *fdrc = (const struct FDRConfirm *)
                                        ((const u8 *)confBase + cf);
        if (!(fdrc->groups & *control)) {
            continue;
        }
        u64a confVal = unaligned_load_u64a(confLoc + byte - sizeof(u64a) + 1);
        confWithBit(fdrc, a, ptr_main - a->buf + byte, control,
                    last_match_id, confVal, conf, bit);
    } while (unlikely(!!*conf));
}

static really_inline
void dumpZoneInfo(UNUSED struct zone *z, UNUSED size_t zone_id) {
#ifdef DEBUG
    DEBUG_PRINTF("zone: zone=%zu, bufPtr=%p\n", zone_id, z->buf);
    DEBUG_PRINTF("zone: startPtr=%p, endPtr=%p, shift=%u\n",
                 z->start, z->end, z->shift);
    DEBUG_PRINTF("zone: zone_pointer_adjust=%zd, floodPtr=%p\n",
                 z->zone_pointer_adjust, z->floodPtr);
    DEBUG_PRINTF("zone buf:");
    for (size_t i = 0; i < ZONE_TOTAL_SIZE; i++) {
        if (i % 8 == 0) {
            printf("_");
        }
        if (z->buf[i]) {
            printf("%02x", z->buf[i]);
        } else {
            printf("..");
        }
    }
    printf("\n");
#endif
};

/**
 * \brief Updates attributes for non-boundary region zone.
 */
static really_inline
void createMainZone(const u8 *flood, const u8 *begin, const u8 *end,
                    struct zone *z) {
    z->zone_pointer_adjust = 0; /* zone buffer is the main buffer */
    z->start = begin;
    z->end = end;
    z->floodPtr = flood;
    z->shift = 0;
}

/**
 * \brief Create zone for short cases (<= ITER_BYTES).
 *
 * For this case we need to copy everything into the zone's internal buffer.
 *
 * We need to ensure that we run over real data if it exists (in history or
 * before zone begin). We also need to ensure 8 bytes before any data being
 * matched can be read (to perform a conf hash).
 *
 * We also need to ensure that the data at z->end can be read.
 *
 * Hence, the zone consists of:
 *     16 bytes of history,
 *     1 - 24 bytes of data form the buffer (ending at end),
 *     1 byte of final padding
 */
static really_inline
void createShortZone(const u8 *buf, const u8 *hend, const u8 *begin,
                     const u8 *end, struct zone *z) {
    /* the floodPtr for BOUNDARY zones are maximum of end of zone buf to avoid
     * the checks in boundary zone. */
    z->floodPtr = z->buf + ZONE_TOTAL_SIZE;

    ptrdiff_t z_len = end - begin;
    assert(z_len > 0);
    assert(z_len <= ITER_BYTES);

    z->shift = ITER_BYTES - z_len; /* ignore bytes outside region specified */

    static const size_t ZONE_SHORT_DATA_OFFSET = 16; /* after history */

    /* we are guaranteed to always have 16 initialised bytes at the end of
     * the history buffer (they may be garbage coming from the stream state
     * preceding hbuf, but bytes that don't correspond to actual history
     * shouldn't affect computations). */
    *(m128 *)z->buf = loadu128(hend - sizeof(m128));

    /* The amount of data we have to copy from main buffer. */
    size_t copy_len = MIN((size_t)(end - buf),
                          ITER_BYTES + sizeof(CONF_TYPE));

    u8 *zone_data = z->buf + ZONE_SHORT_DATA_OFFSET;
    switch (copy_len) {
    case 1:
        *zone_data = *(end - 1);
        break;
    case 2:
        *(u16 *)zone_data = unaligned_load_u16(end - 2);
        break;
    case 3:
        *(u16 *)zone_data = unaligned_load_u16(end - 3);
        *(zone_data + 2) = *(end - 1);
        break;
    case 4:
        *(u32 *)zone_data = unaligned_load_u32(end - 4);
        break;
    case 5:
    case 6:
    case 7:
        /* perform copy with 2 overlapping 4-byte chunks from buf. */
        *(u32 *)zone_data = unaligned_load_u32(end - copy_len);
        unaligned_store_u32(zone_data + copy_len - sizeof(u32),
                            unaligned_load_u32(end - sizeof(u32)));
        break;
    case 8:
        *(u64a *)zone_data = unaligned_load_u64a(end - 8);
        break;
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
        /* perform copy with 2 overlapping 8-byte chunks from buf. */
        *(u64a *)zone_data = unaligned_load_u64a(end - copy_len);
        unaligned_store_u64a(zone_data + copy_len - sizeof(u64a),
                             unaligned_load_u64a(end - sizeof(u64a)));
        break;
    case 16:
        /* copy 16-bytes from buf. */
        *(m128 *)zone_data = loadu128(end - 16);
        break;
    default:
        assert(copy_len <= sizeof(m128) + sizeof(u64a));

        /* perform copy with (potentially overlapping) 8-byte and 16-byte chunks.
         */
        *(u64a *)zone_data = unaligned_load_u64a(end - copy_len);
        storeu128(zone_data + copy_len - sizeof(m128),
                  loadu128(end - sizeof(m128)));
        break;
    }

    /* set the start and end location of the zone buf
     * to be scanned */
    u8 *z_end = z->buf + ZONE_SHORT_DATA_OFFSET + copy_len;
    assert(ZONE_SHORT_DATA_OFFSET + copy_len >= ITER_BYTES);

    /* copy the post-padding byte; this is required for domain > 8 due to
     * overhang */
    assert(ZONE_SHORT_DATA_OFFSET + copy_len + 3 < 64);
    *z_end = 0;

    z->end = z_end;
    z->start = z_end - ITER_BYTES;
    z->zone_pointer_adjust = (ptrdiff_t)((uintptr_t)end - (uintptr_t)z_end);
    assert(z->start + z->shift == z_end - z_len);
}

/**
 * \brief Create a zone for the start region.
 *
 * This function requires that there is > ITER_BYTES of data in the buffer to
 * scan. The start zone itself is always responsible for scanning exactly
 * ITER_BYTES of data - there are no warmup/junk bytes scanned.
 *
 * This zone ensures that the byte at z->end can be read and corresponds to
 * the next byte of data.
 *
 * 8 bytes of history data are provided before z->start to allow proper hash
 * generation in streaming mode. If buf != begin, upto 8 bytes of data
 * prior to begin is also provided.
 *
 * Although we are not interested in bare literals which start before begin
 * if buf != begin, lookarounds associated with the literal may require
 * the data prior to begin for hash purposes.
 */
static really_inline
void createStartZone(const u8 *buf, const u8 *hend, const u8 *begin,
                     struct zone *z) {
    assert(ITER_BYTES == sizeof(m128));
    assert(sizeof(CONF_TYPE) == 8);
    static const size_t ZONE_START_BEGIN = sizeof(CONF_TYPE);

    const u8 *end = begin + ITER_BYTES;

    /* set floodPtr to the end of zone buf to avoid checks in start zone */
    z->floodPtr = z->buf + ZONE_TOTAL_SIZE;

    z->shift = 0; /* we are processing ITER_BYTES of real data */

    /* we are guaranteed to always have 16 initialised bytes at the end of the
     * history buffer (they may be garbage coming from the stream state
     * preceding hbuf, but bytes that don't correspond to actual history
     * shouldn't affect computations). However, for start zones, history is only
     * required for conf hash purposes so we only need 8 bytes */
    unaligned_store_u64a(z->buf, unaligned_load_u64a(hend - sizeof(u64a)));

    /* The amount of data we have to copy from main buffer. */
    size_t copy_len = MIN((size_t)(end - buf),
                          ITER_BYTES + sizeof(CONF_TYPE));
    assert(copy_len >= 16);

    /* copy the post-padding byte; this is required for domain > 8 due to
     * overhang. The start requires that there is data after the zone so it
     * it safe to dereference end */
    z->buf[ZONE_START_BEGIN + copy_len] = *end;

    /* set the start and end location of the zone buf to be scanned */
    u8 *z_end = z->buf + ZONE_START_BEGIN + copy_len;
    z->end = z_end;
    z->start = z_end - ITER_BYTES;

    /* copy the first 8 bytes of the valid region */
    unaligned_store_u64a(z->buf + ZONE_START_BEGIN,
                         unaligned_load_u64a(end - copy_len));

    /* copy the last 16 bytes, may overlap with the previous 8 byte write */
    storeu128(z_end - sizeof(m128), loadu128(end - sizeof(m128)));

    z->zone_pointer_adjust = (ptrdiff_t)((uintptr_t)end - (uintptr_t)z_end);

    assert(ZONE_START_BEGIN + copy_len + 3 < 64);
}

/**
 * \brief Create a zone for the end region.
 *
 * This function requires that there is > ITER_BYTES of data in the buffer to
 * scan. The end zone is responsible for a scanning the <= ITER_BYTES rump of
 * data and optional ITER_BYTES. The main zone cannot handle the last 3 bytes
 * of the buffer. The end zone is required to handle an optional full
 * ITER_BYTES from main zone when there are less than 3 bytes to scan. The
 * main zone size is reduced by ITER_BYTES in this case.
 *
 * This zone ensures that the byte at z->end can be read by filling it with a
 * padding character.
 *
 * Upto 8 bytes of data prior to begin is also provided for the purposes of
 * generating hashes. History is not copied, as all locations which require
 * history for generating a hash are the responsiblity of the start zone.
 */
static really_inline
void createEndZone(const u8 *buf, const u8 *begin, const u8 *end,
                   struct zone *z) {
    /* the floodPtr for BOUNDARY zones are maximum of end of zone buf to avoid
     * the checks in boundary zone. */
    z->floodPtr = z->buf + ZONE_TOTAL_SIZE;

    ptrdiff_t z_len = end - begin;
    assert(z_len > 0);
    size_t iter_bytes_second = 0;
    size_t z_len_first = z_len;
    if (z_len > ITER_BYTES) {
        z_len_first = z_len - ITER_BYTES;
        iter_bytes_second = ITER_BYTES;
    }
    z->shift = ITER_BYTES - z_len_first;

    const u8 *end_first = end - iter_bytes_second;
    /* The amount of data we have to copy from main buffer for the
     * first iteration. */
    size_t copy_len_first = MIN((size_t)(end_first - buf),
                                ITER_BYTES + sizeof(CONF_TYPE));
    assert(copy_len_first >= 16);

    size_t total_copy_len = copy_len_first + iter_bytes_second;
    assert(total_copy_len + 3 < 64);

    /* copy the post-padding byte; this is required for domain > 8 due to
     * overhang */
    z->buf[total_copy_len] = 0;

    /* set the start and end location of the zone buf
     * to be scanned */
    u8 *z_end = z->buf + total_copy_len;
    z->end = z_end;
    z->start = z_end - ITER_BYTES - iter_bytes_second;
    assert(z->start + z->shift == z_end - z_len);

    u8 *z_end_first = z_end - iter_bytes_second;
    /* copy the first 8 bytes of the valid region */
    unaligned_store_u64a(z->buf,
                         unaligned_load_u64a(end_first - copy_len_first));

    /* copy the last 16 bytes, may overlap with the previous 8 byte write */
    storeu128(z_end_first - sizeof(m128), loadu128(end_first - sizeof(m128)));
    if (iter_bytes_second) {
        storeu128(z_end - sizeof(m128), loadu128(end - sizeof(m128)));
    }

    z->zone_pointer_adjust = (ptrdiff_t)((uintptr_t)end - (uintptr_t)z_end);
}

/**
 * \brief Prepare zones.
 *
 * This function prepares zones with actual buffer and some padded bytes.
 * The actual ITER_BYTES bytes in zone is preceded by main buf and/or
 * history buf and succeeded by padded bytes possibly from main buf,
 * if available.
 */
static really_inline
size_t prepareZones(const u8 *buf, size_t len, const u8 *hend,
                    size_t start, const u8 *flood, struct zone *zoneArr) {
    const u8 *ptr = buf + start;
    size_t remaining = len - start;

    if (remaining <= ITER_BYTES) {
        /* enough bytes to make only one zone */
        createShortZone(buf, hend, ptr, buf + len, &zoneArr[0]);
        return 1;
    }

    /* enough bytes to make more than one zone */

    size_t numZone = 0;
    createStartZone(buf, hend, ptr, &zoneArr[numZone++]);
    ptr += ITER_BYTES;

    assert(ptr < buf + len);

    /* find maximum buffer location that the main zone can scan
     * - must be a multiple of ITER_BYTES, and
     * - cannot contain the last 3 bytes (due to 3 bytes read behind the
         end of buffer in FDR main loop)
     */
    const u8 *main_end = buf + start + ROUNDDOWN_N(len - start - 3, ITER_BYTES);

    /* create a zone if multiple of ITER_BYTES are found */
    if (main_end > ptr) {
        createMainZone(flood, ptr, main_end, &zoneArr[numZone++]);
        ptr = main_end;
    }
    /* create a zone with rest of the data from the main buffer */
    createEndZone(buf, ptr, buf + len, &zoneArr[numZone++]);
    return numZone;
}

#define INVALID_MATCH_ID (~0U)

#define FDR_MAIN_LOOP(zz, s, get_conf_fn)                                   \
    do {                                                                    \
        const u8 *tryFloodDetect = zz->floodPtr;                            \
        const u8 *start_ptr = zz->start;                                    \
        const u8 *end_ptr = zz->end;                                        \
                                                                            \
        for (const u8 *itPtr = start_ptr; itPtr + ITER_BYTES <= end_ptr;    \
            itPtr += ITER_BYTES) {                                          \
            if (unlikely(itPtr > tryFloodDetect)) {                         \
                tryFloodDetect = floodDetect(fdr, a, &itPtr, tryFloodDetect,\
                                             &floodBackoff, &control,       \
                                             ITER_BYTES);                   \
                if (unlikely(control == HWLM_TERMINATE_MATCHING)) {         \
                    return HWLM_TERMINATED;                                 \
                }                                                           \
            }                                                               \
            __builtin_prefetch(itPtr + ITER_BYTES);                         \
            u64a conf0;                                                     \
            u64a conf8;                                                     \
            get_conf_fn(itPtr, start_ptr, end_ptr, domain_mask_flipped,     \
                        ft, &conf0, &conf8, &s);                            \
            do_confirm_fdr(&conf0, 0, &control, confBase, a, itPtr,         \
                           &last_match_id, zz);                             \
            do_confirm_fdr(&conf8, 8, &control, confBase, a, itPtr,         \
                           &last_match_id, zz);                             \
            if (unlikely(control == HWLM_TERMINATE_MATCHING)) {             \
                return HWLM_TERMINATED;                                     \
            }                                                               \
        } /* end for loop */                                                \
    } while (0)                                                             \

static never_inline
hwlm_error_t fdr_engine_exec(const struct FDR *fdr,
                             const struct FDR_Runtime_Args *a,
                             hwlm_group_t control) {
    assert(ISALIGNED_CL(fdr));

    u32 floodBackoff = FLOOD_BACKOFF_START;
    u32 last_match_id = INVALID_MATCH_ID;
    u32 domain_mask_flipped = ~fdr->domainMask;
    u8 stride = fdr->stride;
    const u64a *ft =
        (const u64a *)((const u8 *)fdr + ROUNDUP_CL(sizeof(struct FDR)));
    assert(ISALIGNED_CL(ft));
    const u32 *confBase = (const u32 *)((const u8 *)fdr + fdr->confOffset);
    assert(ISALIGNED_CL(confBase));
    struct zone zones[ZONE_MAX];
    assert(fdr->domain > 8 && fdr->domain < 16);

    size_t numZone = prepareZones(a->buf, a->len,
                                  a->buf_history + a->len_history,
                                  a->start_offset, a->firstFloodDetect, zones);
    assert(numZone <= ZONE_MAX);
    m128 state = getInitState(fdr, a->len_history, ft, &zones[0]);

    for (size_t curZone = 0; curZone < numZone; curZone++) {
        struct zone *z = &zones[curZone];
        dumpZoneInfo(z, curZone);

        /* When a zone contains less data than is processed in an iteration
         * of FDR_MAIN_LOOP(), we need to scan over some extra data.
         *
         * We have chosen to scan this extra data at the start of the
         * iteration. The extra data is either data we have already scanned or
         * garbage (if it is earlier than offset 0),
         *
         * As a result we need to shift the incoming state back so that it will
         * properly line up with the data being scanned.
         *
         * We also need to forbid reporting any matches in the data being
         * rescanned as they have already been reported (or are over garbage but
         * later stages should also provide that safety guarantee).
         */

        u8 shift = z->shift;

        state = variable_byte_shift_m128(state, shift);

        state = or128(state, load128(zone_or_mask[shift]));

        switch (stride) {
        case 1:
            FDR_MAIN_LOOP(z, state, get_conf_stride_1);
            break;
        case 2:
            FDR_MAIN_LOOP(z, state, get_conf_stride_2);
            break;
        case 4:
            FDR_MAIN_LOOP(z, state, get_conf_stride_4);
            break;
        default:
            break;
        }
    }

    return HWLM_SUCCESS;
}

#if defined(HAVE_AVX2)
#define ONLY_AVX2(func) func
#else
#define ONLY_AVX2(func) NULL
#endif

typedef hwlm_error_t (*FDRFUNCTYPE)(const struct FDR *fdr,
                                    const struct FDR_Runtime_Args *a,
                                    hwlm_group_t control);

static const FDRFUNCTYPE funcs[] = {
    fdr_engine_exec,
    NULL, /* old: fast teddy */
    NULL, /* old: fast teddy */
    ONLY_AVX2(fdr_exec_fat_teddy_msks1),
    ONLY_AVX2(fdr_exec_fat_teddy_msks1_pck),
    ONLY_AVX2(fdr_exec_fat_teddy_msks2),
    ONLY_AVX2(fdr_exec_fat_teddy_msks2_pck),
    ONLY_AVX2(fdr_exec_fat_teddy_msks3),
    ONLY_AVX2(fdr_exec_fat_teddy_msks3_pck),
    ONLY_AVX2(fdr_exec_fat_teddy_msks4),
    ONLY_AVX2(fdr_exec_fat_teddy_msks4_pck),
    fdr_exec_teddy_msks1,
    fdr_exec_teddy_msks1_pck,
    fdr_exec_teddy_msks2,
    fdr_exec_teddy_msks2_pck,
    fdr_exec_teddy_msks3,
    fdr_exec_teddy_msks3_pck,
    fdr_exec_teddy_msks4,
    fdr_exec_teddy_msks4_pck,
};

#define FAKE_HISTORY_SIZE 16
static const u8 fake_history[FAKE_HISTORY_SIZE];

hwlm_error_t fdrExec(const struct FDR *fdr, const u8 *buf, size_t len,
                     size_t start, HWLMCallback cb,
                     struct hs_scratch *scratch, hwlm_group_t groups) {
    // We guarantee (for safezone construction) that it is safe to read 16
    // bytes before the end of the history buffer.
    const u8 *hbuf = fake_history + FAKE_HISTORY_SIZE;

    const struct FDR_Runtime_Args a = {
        buf,
        len,
        hbuf,
        0,
        start,
        cb,
        scratch,
        nextFloodDetect(buf, len, FLOOD_BACKOFF_START),
        0
    };
    if (unlikely(a.start_offset >= a.len)) {
        return HWLM_SUCCESS;
    } else {
        assert(funcs[fdr->engineID]);
        return funcs[fdr->engineID](fdr, &a, groups);
    }
}

hwlm_error_t fdrExecStreaming(const struct FDR *fdr, const u8 *hbuf,
                              size_t hlen, const u8 *buf, size_t len,
                              size_t start, HWLMCallback cb,
                              struct hs_scratch *scratch,
                              hwlm_group_t groups) {
    struct FDR_Runtime_Args a = {
        buf,
        len,
        hbuf,
        hlen,
        start,
        cb,
        scratch,
        nextFloodDetect(buf, len, FLOOD_BACKOFF_START),
        /* we are guaranteed to always have 16 initialised bytes at the end of
         * the history buffer (they may be garbage). */
        hbuf ? unaligned_load_u64a(hbuf + hlen - sizeof(u64a)) : (u64a)0
    };

    hwlm_error_t ret;
    if (unlikely(a.start_offset >= a.len)) {
        ret = HWLM_SUCCESS;
    } else {
        assert(funcs[fdr->engineID]);
        ret = funcs[fdr->engineID](fdr, &a, groups);
    }

    return ret;
}
