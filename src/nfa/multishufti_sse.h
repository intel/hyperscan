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

#include "shufti_common.h"

#include "ue2common.h"
#include "util/bitutils.h"
#include "util/simd_utils.h"
#include "util/simd_utils_ssse3.h"

/* Normal SSSE3 shufti */

static really_inline
const u8 *JOIN(MATCH_ALGO, fwdBlock)(m128 mask_lo, m128 mask_hi, m128 chars,
                                     const u8 *buf, const m128 low4bits,
                                     const m128 zeroes, const u8 run_len
#ifdef MULTIACCEL_DOUBLE
                                     , const u8 run_len2
#endif
                                             ) {
    // negate first 16 bits
    u32 z = block(mask_lo, mask_hi, chars, low4bits, zeroes) ^ 0xFFFF;
    return (*JOIN4(MATCH_ALGO, match_funcs, _, 32)[run_len])(buf, z
#ifdef MULTIACCEL_DOUBLE
            , run_len2
#endif
            );
}

/*
 * 16-byte pipeline, for smaller scans
 */
static
const u8 *JOIN(MATCH_ALGO, shuftiPipeline16)(m128 mask_lo, m128 mask_hi,
                                             const u8 *buf, const u8 *buf_end,
                                             const m128 low4bits,
                                             const m128 zeroes, const u8 run_len
#ifdef MULTIACCEL_DOUBLE
                                             , const u8 run_len2
#endif
                                             ) {
    const u8* ptr, *last_buf;
    u32 last_res;

    // pipeline prologue: scan first 16 bytes
    m128 data = load128(buf);
    u32 z = block(mask_lo, mask_hi, data, low4bits, zeroes) ^ 0xFFFF;
    last_buf = buf;
    last_res = z;
    buf += 16;

    // now, start the pipeline!
    assert((size_t)buf % 16 == 0);
    for (; buf + 15 < buf_end; buf += 16) {
        // scan more data
        data = load128(buf);
        z = block(mask_lo, mask_hi, data, low4bits, zeroes) ^ 0xFFFF;

        // do a comparison on previous result
        ptr = (*JOIN4(MATCH_ALGO, match_funcs, _, 32)[run_len])
                (last_buf, last_res
#ifdef MULTIACCEL_DOUBLE
                 , run_len2
#endif
                 );
        if (unlikely(ptr)) {
            return ptr;
        }
        last_buf = buf;
        last_res = z;
    }
    assert(buf <= buf_end && buf >= buf_end - 16);

    // epilogue: compare final results
    ptr = (*JOIN4(MATCH_ALGO, match_funcs, _, 32)[run_len])
            (last_buf, last_res
#ifdef MULTIACCEL_DOUBLE
             , run_len2
#endif
             );
    if (unlikely(ptr)) {
        return ptr;
    }

    return NULL;
}

/*
 * 32-byte pipeline, for bigger scans
 */
static
const u8 *JOIN(MATCH_ALGO, shuftiPipeline32)(m128 mask_lo, m128 mask_hi,
                                             const u8 *buf, const u8 *buf_end,
                                             const m128 low4bits,
                                             const m128 zeroes, const u8 run_len
#ifdef MULTIACCEL_DOUBLE
                                             , const u8 run_len2
#endif
                                             ) {
    const u8* ptr, *last_buf;
    u32 res;

    // pipeline prologue: scan first 32 bytes
    m128 data1 = load128(buf);
    u32 z1 = block(mask_lo, mask_hi, data1, low4bits, zeroes) ^ 0xFFFF;
    m128 data2 = load128(buf + 16);
    u32 z2 = block(mask_lo, mask_hi, data2, low4bits, zeroes) ^ 0xFFFF;

    // store the results
    u32 last_res = z1 | (z2 << 16);
    last_buf = buf;
    buf += 32;


    // now, start the pipeline!
    assert((size_t)buf % 16 == 0);
    for (; buf + 31 < buf_end; buf += 32) {
        // scan more data
        data1 = load128(buf);
        z1 = block(mask_lo, mask_hi, data1, low4bits, zeroes) ^ 0xFFFF;
        data2 = load128(buf + 16);
        z2 = block(mask_lo, mask_hi, data2, low4bits, zeroes) ^ 0xFFFF;
        res = z1 | (z2 << 16);

        // do a comparison on previous result
        ptr = (*JOIN4(MATCH_ALGO, match_funcs, _, 64)[run_len])
                (last_buf, last_res
#ifdef MULTIACCEL_DOUBLE
                 , run_len2
#endif
                 );
        if (unlikely(ptr)) {
            return ptr;
        }
        last_res = res;
        last_buf = buf;
    }

    // epilogue: compare final results
    ptr = (*JOIN4(MATCH_ALGO, match_funcs, _, 64)[run_len])
            (last_buf, last_res
#ifdef MULTIACCEL_DOUBLE
             , run_len2
#endif
             );
    if (unlikely(ptr)) {
        return ptr;
    }

    // if we still have some data left, scan it too
    for (; buf + 15 < buf_end; buf += 16) {
        m128 chars = load128(buf);
        ptr = JOIN(MATCH_ALGO, fwdBlock)(mask_lo, mask_hi, chars, buf,
                low4bits, zeroes, run_len
#ifdef MULTIACCEL_DOUBLE
                , run_len2
#endif
                );
        if (unlikely(ptr)) {
            return ptr;
        }
    }
    assert(buf <= buf_end && buf >= buf_end - 16);

    return NULL;
}

const u8 *JOIN(MATCH_ALGO, shuftiExec)(m128 mask_lo, m128 mask_hi,
                                       const u8 *buf,
                                       const u8 *buf_end, u8 run_len
#ifdef MULTIACCEL_DOUBLE
                                       , u8 run_len2
#endif
                                               ) {
    assert(buf && buf_end);
    assert(buf < buf_end);

    // Slow path for small cases.
    if (buf_end - buf < 16) {
        return shuftiFwdSlow((const u8 *)&mask_lo, (const u8 *)&mask_hi,
                buf, buf_end);
    }

    const m128 zeroes = zeroes128();
    const m128 low4bits = _mm_set1_epi8(0xf);
    const u8 *rv;

    size_t min = (size_t)buf % 16;
    assert(buf_end - buf >= 16);

    // Preconditioning: most of the time our buffer won't be aligned.
    m128 chars = loadu128(buf);
    rv = JOIN(MATCH_ALGO, fwdBlock)(mask_lo, mask_hi, chars, buf,
            low4bits, zeroes, run_len
#ifdef MULTIACCEL_DOUBLE
            , run_len2
#endif
            );
    if (rv) {
        return rv;
    }
    buf += (16 - min);

    // if we have enough data, run bigger pipeline; otherwise run smaller one
    if (buf_end - buf >= 128) {
        rv = JOIN(MATCH_ALGO, shuftiPipeline32)(mask_lo, mask_hi,
                buf, buf_end, low4bits, zeroes, run_len
#ifdef MULTIACCEL_DOUBLE
                , run_len2
#endif
                );
        if (unlikely(rv)) {
            return rv;
        }
    } else if (buf_end - buf >= 16){
        rv = JOIN(MATCH_ALGO, shuftiPipeline16)(mask_lo, mask_hi,
                buf, buf_end, low4bits, zeroes, run_len
#ifdef MULTIACCEL_DOUBLE
                , run_len2
#endif
                );
        if (unlikely(rv)) {
            return rv;
        }
    }

    // Use an unaligned load to mop up the last 16 bytes and get an accurate
    // picture to buf_end.
    chars = loadu128(buf_end - 16);
    rv = JOIN(MATCH_ALGO, fwdBlock)(mask_lo, mask_hi, chars,
            buf_end - 16, low4bits, zeroes, run_len
#ifdef MULTIACCEL_DOUBLE
            , run_len2
#endif
            );
    if (rv) {
        return rv;
    }

    return buf_end;
}
