/*
 * Copyright (c) 2015, Intel Corporation
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

#ifndef ROSE_MIRACLE_H
#define ROSE_MIRACLE_H

#include "ue2common.h"
#include "runtime.h"
#include "rose_internal.h"

/** \brief Maximum number of bytes to scan when looking for a "miracle" stop
 * character. */
#define MIRACLE_LEN_MAX 32

static really_inline
u64a roseMiracleScan(const u8 *stop, const u8 *d, const u8 *d_start) {
    assert(d >= d_start);

    // Note: unrolling this loop manually does appear to reduce its
    // performance. I'm sick of tilting at this particular windmill.

    u32 mshift = 0;
    do {
        u64a s = (u64a)stop[*d];
        if (s) {
            s <<= mshift;
            return s;
        }
        mshift++;
    } while (--d >= d_start);
    return 0;
}

/**
 * \brief "Miracle" scan: uses stop table to check if we can skip forward to a
 * location where we know that the given rose engine will be in a known state.
 *
 * Scans the buffer/history between relative locations \a begin_loc and \a
 * end_loc, and returns a miracle location (if any) that appears in the stream
 * after \a begin_loc.
 *
 * Returns 1 if some bytes can be skipped and sets \a miracle_loc
 * appropriately, 0 otherwise.
 */
static rose_inline
char roseMiracleOccurs(const struct RoseEngine *t,
                       const struct LeftNfaInfo *left,
                       const struct core_info *ci, const s64a begin_loc,
                       const s64a end_loc, s64a *miracle_loc) {
    assert(!left->transient);
    assert(left->stopTable);

    DEBUG_PRINTF("looking for miracle over [%lld,%lld], maxLag=%u\n",
                 begin_loc, end_loc, left->maxLag);
    DEBUG_PRINTF("ci->len=%zu, ci->hlen=%zu\n", ci->len, ci->hlen);

    assert(begin_loc <= end_loc);
    assert(begin_loc >= -(s64a)ci->hlen);
    assert(end_loc <= (s64a)ci->len);

    const u8 *stop = getByOffset(t, left->stopTable);

    const s64a scan_end_loc = end_loc - left->maxLag;
    if (scan_end_loc <= begin_loc) {
        DEBUG_PRINTF("nothing to scan\n");
        return 0;
    }

    const s64a start = MAX(begin_loc, scan_end_loc - MIRACLE_LEN_MAX);
    DEBUG_PRINTF("scan [%lld..%lld]\n", start, scan_end_loc);

    u64a s = 0; // state, on bits are miracle locations

    // Scan buffer.
    const s64a buf_scan_start = MAX(0, start);
    if (scan_end_loc > buf_scan_start) {
        const u8 *buf = ci->buf;
        const u8 *d = buf + scan_end_loc - 1;
        const u8 *d_start = buf + buf_scan_start;
        s = roseMiracleScan(stop, d, d_start);
        if (s) {
            goto miracle_found;
        }
    }

    // Scan history.
    if (start < 0) {
        const u8 *hbuf_end = ci->hbuf + ci->hlen;
        const u8 *d = hbuf_end + MIN(0, scan_end_loc) - 1;
        const u8 *d_start = hbuf_end + start;
        s = roseMiracleScan(stop, d, d_start);
        if (scan_end_loc > 0) {
            // Shift s over to account for the buffer scan above.
            s <<= scan_end_loc;
        }
    }

    if (s) {
    miracle_found:
        DEBUG_PRINTF("s=0x%llx, ctz=%u\n", s, ctz64(s));
        s64a loc = end_loc - left->maxLag - ctz64(s) - 1;
        if (loc > begin_loc) {
            DEBUG_PRINTF("miracle at %lld\n", loc);
            *miracle_loc = loc;
            return 1;
        }
    }

    DEBUG_PRINTF("no viable miraculous stop characters found\n");
    return 0;
}

#endif // ROSE_MIRACLE_H
