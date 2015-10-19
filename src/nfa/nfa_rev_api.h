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

/** \file
 * \brief Reverse-acceleration optimizations for the NFA API block mode scans.
 */

#ifndef NFA_REV_API_H
#define NFA_REV_API_H

#include "accel.h"
#include "nfa_internal.h"
#include "vermicelli.h"
#include "util/unaligned.h"

static really_inline
size_t nfaRevAccel_i(const struct NFA *nfa, const u8 *buffer, size_t length) {
    DEBUG_PRINTF("checking rev accel mw %u\n", nfa->minWidth);
    assert(nfa->rAccelOffset >= 1);
    assert(nfa->rAccelOffset <= nfa->minWidth);

    const u8 *rv; // result for accel engine

    switch (nfa->rAccelType) {
    case ACCEL_RVERM:
        DEBUG_PRINTF("ACCEL_RVERM\n");
        if (length + 1 - nfa->rAccelOffset < 16) {
            break;
        }

        rv = rvermicelliExec(nfa->rAccelData.c, 0, buffer,
                             buffer + length + 1 - nfa->rAccelOffset);
        length = (size_t)(rv - buffer + nfa->rAccelOffset);
        break;
    case ACCEL_RVERM_NOCASE:
        DEBUG_PRINTF("ACCEL_RVERM_NOCASE\n");
        if (length + 1 - nfa->rAccelOffset < 16) {
            break;
        }

        rv = rvermicelliExec(nfa->rAccelData.c, 1, buffer,
                             buffer + length + 1 - nfa->rAccelOffset);
        length = (size_t)(rv - buffer + nfa->rAccelOffset);
        break;
    case ACCEL_RDVERM:
        DEBUG_PRINTF("ACCEL_RDVERM\n");
        if (length + 1 - nfa->rAccelOffset < 17) {
            break;
        }

        rv = rvermicelliDoubleExec(nfa->rAccelData.array[0],
                                   nfa->rAccelData.array[1], 0, buffer,
                                   buffer + length + 1 - nfa->rAccelOffset);
        length = (size_t)(rv - buffer + nfa->rAccelOffset);
        break;
    case ACCEL_RDVERM_NOCASE:
        DEBUG_PRINTF("ACCEL_RVERM_NOCASE\n");
        if (length + 1 - nfa->rAccelOffset < 17) {
            break;
        }

        rv = rvermicelliDoubleExec(nfa->rAccelData.array[0],
                                   nfa->rAccelData.array[1], 1, buffer,
                                   buffer + length + 1 - nfa->rAccelOffset);
        length = (size_t)(rv - buffer + nfa->rAccelOffset);
        break;
    case ACCEL_REOD:
        DEBUG_PRINTF("ACCEL_REOD\n");
        if (buffer[length - nfa->rAccelOffset] != nfa->rAccelData.c) {
            return 0;
        }
        break;
    case ACCEL_REOD_NOCASE:
        DEBUG_PRINTF("ACCEL_REOD_NOCASE\n");
        if ((buffer[length - nfa->rAccelOffset] & CASE_CLEAR) !=
            nfa->rAccelData.c) {
            return 0;
        }
        break;
    case ACCEL_RDEOD:
        DEBUG_PRINTF("ACCEL_RDEOD\n");
        if (unaligned_load_u16(buffer + length - nfa->rAccelOffset) !=
                nfa->rAccelData.dc) {
            return 0;
        }
        break;
    case ACCEL_RDEOD_NOCASE:
        DEBUG_PRINTF("ACCEL_RDEOD_NOCASE\n");
        if ((unaligned_load_u16(buffer + length - nfa->rAccelOffset) &
             DOUBLE_CASE_CLEAR) != nfa->rAccelData.dc) {
            return 0;
        }
        break;
    default:
        assert(!"not here");
    }

    if (nfa->minWidth > length) {
        DEBUG_PRINTF("post-accel, scan skipped: %zu < min %u bytes\n", length,
                     nfa->minWidth);
        return 0;
    }

    return length;
}

/** \brief Reverse acceleration check. Returns a new length for the block,
 * guaranteeing that a match cannot occur beyond that point. */
static really_inline
size_t nfaRevAccelCheck(const struct NFA *nfa, const u8 *buffer,
                        size_t length) {
    assert(nfa);

    // If this block is not long enough to satisfy the minimum width
    // constraint on this NFA, we can avoid the scan altogether.
    if (nfa->minWidth > length) {
        DEBUG_PRINTF("scan skipped: %zu < min %u bytes\n", length,
                     nfa->minWidth);
        return 0;
    }

    if (nfa->rAccelType == ACCEL_NONE) {
        DEBUG_PRINTF("no rev accel available\n");
        return length;
    }

    size_t rv_length = nfaRevAccel_i(nfa, buffer, length);
    assert(rv_length <= length);
    return rv_length;
}

#endif
