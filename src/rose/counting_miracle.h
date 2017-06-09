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

#ifndef ROSE_COUNTING_MIRACLE_H
#define ROSE_COUNTING_MIRACLE_H

#include "ue2common.h"
#include "runtime.h"
#include "rose_internal.h"
#include "nfa/nfa_api_queue.h"
#include "util/simd_utils.h"

/** \brief Maximum number of bytes to scan when looking for a "counting miracle"
 * stop character. */
#define COUNTING_MIRACLE_LEN_MAX 256

static really_inline
char roseCountingMiracleScan(u8 c, const u8 *d, const u8 *d_end,
                             u32 target_count, u32 *count_inout,
                             const u8 **d_out) {
    assert(d <= d_end);

    u32 count = *count_inout;

    m128 chars = set16x8(c);

    for (; d + 16 <= d_end; d_end -= 16) {
        m128 data = loadu128(d_end - 16);
        u32 z1 = movemask128(eq128(chars, data));
        count += popcount32(z1);

        if (count >= target_count) {
            *d_out = d_end - 16;
            *count_inout = count;
            return 1;
        }
    }

    if (d != d_end) {
        char temp[sizeof(m128)];
        assert(d + sizeof(temp) > d_end);
        memset(temp, c + 1, sizeof(temp));
        memcpy(temp, d, d_end - d);
        m128 data = loadu128(temp);
        u32 z1 = movemask128(eq128(chars, data));
        count += popcount32(z1);

        if (count >= target_count) {
            *d_out = d;
            *count_inout = count;
            return 1;
        }
    }

    *count_inout = count;
    return 0;
}

#define GET_LO_4(chars) and128(chars, low4bits)
#define GET_HI_4(chars) rshift64_m128(andnot128(low4bits, chars), 4)

static really_inline
u32 roseCountingMiracleScanShufti(m128 mask_lo, m128 mask_hi, u8 poison,
                                  const u8 *d, const u8 *d_end,
                                  u32 target_count, u32 *count_inout,
                                  const u8 **d_out) {
    assert(d <= d_end);

    u32 count = *count_inout;

    const m128 zeroes = zeroes128();
    const m128 low4bits = _mm_set1_epi8(0xf);

    for (; d + 16 <= d_end; d_end -= 16) {
        m128 data = loadu128(d_end - 16);
        m128 c_lo  = pshufb_m128(mask_lo, GET_LO_4(data));
        m128 c_hi  = pshufb_m128(mask_hi, GET_HI_4(data));
        m128 t     = and128(c_lo, c_hi);
        u32 z1 = movemask128(eq128(t, zeroes));
        count += popcount32(z1 ^ 0xffff);

        if (count >= target_count) {
            *d_out = d_end - 16;
            *count_inout = count;
            return 1;
        }
    }

    if (d != d_end) {
        char temp[sizeof(m128)];
        assert(d + sizeof(temp) > d_end);
        memset(temp, poison, sizeof(temp));
        memcpy(temp, d, d_end - d);
        m128 data  = loadu128(temp);
        m128 c_lo  = pshufb_m128(mask_lo, GET_LO_4(data));
        m128 c_hi  = pshufb_m128(mask_hi, GET_HI_4(data));
        m128 t     = and128(c_lo, c_hi);
        u32 z1 = movemask128(eq128(t, zeroes));
        count += popcount32(z1 ^ 0xffff);

        if (count >= target_count) {
            *d_out = d;
            *count_inout = count;
            return 1;
        }
    }

    *count_inout = count;
    return 0;
}

/**
 * \brief "Counting Miracle" scan: If we see more than N instances of a
 * particular character class we know that the engine must be dead.
 *
 * Scans the buffer/history between relative locations \a begin_loc and \a
 * end_loc, and returns a miracle location (if any) that appears in the stream
 * after \a begin_loc.
 *
 * Returns 1 if some bytes can be skipped and sets \a miracle_loc
 * appropriately, 0 otherwise.
 */
static never_inline
int roseCountingMiracleOccurs(const struct RoseEngine *t,
                              const struct LeftNfaInfo *left,
                              const struct core_info *ci, s64a begin_loc,
                              const s64a end_loc, s64a *miracle_loc) {
    if (!left->countingMiracleOffset) {
        return 0;
    }

    const struct RoseCountingMiracle *cm
        = (const void *)((const char *)t + left->countingMiracleOffset);

    assert(!left->transient);
    assert(cm->count > 1); /* should be a normal miracle then */

    DEBUG_PRINTF("looking for counting miracle over [%lld,%lld], maxLag=%u\n",
                 begin_loc, end_loc, left->maxLag);
    DEBUG_PRINTF("ci->len=%zu, ci->hlen=%zu\n", ci->len, ci->hlen);

    assert(begin_loc <= end_loc);
    assert(begin_loc >= -(s64a)ci->hlen);
    assert(end_loc <= (s64a)ci->len);

    const s64a scan_end_loc = end_loc - left->maxLag;
    if (scan_end_loc <= begin_loc) {
        DEBUG_PRINTF("nothing to scan\n");
        return 0;
    }

    const s64a start = MAX(begin_loc, scan_end_loc - COUNTING_MIRACLE_LEN_MAX);
    DEBUG_PRINTF("scan [%lld..%lld]\n", start, scan_end_loc);

    u32 count = 0;

    s64a m_loc = start;

    if (!cm->shufti) {
        u8 c = cm->c;

        // Scan buffer.
        const s64a buf_scan_start = MAX(0, start);
        if (scan_end_loc > buf_scan_start) {
            const u8 *buf = ci->buf;
            const u8 *d = buf + scan_end_loc;
            const u8 *d_start = buf + buf_scan_start;
            const u8 *d_out;
            if (roseCountingMiracleScan(c, d_start, d, cm->count, &count,
                                        &d_out)) {
                assert(d_out >= d_start);
                m_loc = (d_out - d_start) + buf_scan_start;
                goto success;
            }
        }

        // Scan history.
        if (start < 0) {
            const u8 *hbuf_end = ci->hbuf + ci->hlen;
            const u8 *d = hbuf_end + MIN(0, scan_end_loc);
            const u8 *d_start = hbuf_end + start;
            const u8 *d_out;
            if (roseCountingMiracleScan(c, d_start, d, cm->count, &count,
                                        &d_out)) {
                assert(d_out >= d_start);
                m_loc = (d_out - d_start) + start;
                goto success;
            }
        }
    } else {
        m128 lo = cm->lo;
        m128 hi = cm->hi;
        u8 poison = cm->poison;

        // Scan buffer.
        const s64a buf_scan_start = MAX(0, start);
        if (scan_end_loc > buf_scan_start) {
            const u8 *buf = ci->buf;
            const u8 *d = buf + scan_end_loc;
            const u8 *d_start = buf + buf_scan_start;
            const u8 *d_out;
            if (roseCountingMiracleScanShufti(lo, hi, poison, d_start, d,
                                              cm->count, &count, &d_out)) {
                assert(d_out >= d_start);
                m_loc = (d_out - d_start) + buf_scan_start;
                goto success;
            }
        }

        // Scan history.
        if (start < 0) {
            const u8 *hbuf_end = ci->hbuf + ci->hlen;
            const u8 *d = hbuf_end + MIN(0, scan_end_loc);
            const u8 *d_start = hbuf_end + start;
            const u8 *d_out;
            if (roseCountingMiracleScanShufti(lo, hi, poison, d_start, d,
                                              cm->count, &count, &d_out)) {
                assert(d_out >= d_start);
                m_loc = (d_out - d_start) + start;
                goto success;
            }
        }
    }

    DEBUG_PRINTF("found %u/%u\n", count, cm->count);
    return 0;

success:
    DEBUG_PRINTF("found %u/%u\n", count, cm->count);
    assert(count >= cm->count);
    assert(m_loc < scan_end_loc);
    assert(m_loc >= start);

    *miracle_loc = m_loc;
    return 1;
}

#endif
