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

static really_inline
const u8 *JOIN(MATCH_ALGO, fwdBlock)(m256 mask_lo, m256 mask_hi, m256 chars,
                                     const u8 *buf, const m256 low4bits,
                                     const m256 zeroes, const u8 run_len
#ifdef MULTIACCEL_DOUBLE
                                     , const u8 run_len2
#endif
                                     ) {
    u32 z = block(mask_lo, mask_hi, chars, low4bits, zeroes);
    return (*JOIN4(MATCH_ALGO, match_funcs, _, 64)[run_len])(buf, ~z
#ifdef MULTIACCEL_DOUBLE
                                                             , run_len2
#endif
                                                             );
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
    if (buf_end - buf < 32) {
        return shuftiFwdSlow((const u8 *)&mask_lo, (const u8 *)&mask_hi,
                             buf, buf_end);
    }

    const m256 zeroes = zeroes256();
    const m256 low4bits = set32x8(0xf);
    const m256 wide_mask_lo = set2x128(mask_lo);
    const m256 wide_mask_hi = set2x128(mask_hi);
    const u8 *rv;

    size_t min = (size_t)buf % 32;
    assert(buf_end - buf >= 32);

    // Preconditioning: most of the time our buffer won't be aligned.
    m256 chars = loadu256(buf);
    rv = JOIN(MATCH_ALGO, fwdBlock)(wide_mask_lo, wide_mask_hi, chars, buf,
                                            low4bits, zeroes, run_len
#ifdef MULTIACCEL_DOUBLE
                                            , run_len2
#endif
                                            );
    if (rv) {
        return rv;
    }
    buf += (32 - min);

    // Unrolling was here, but it wasn't doing anything but taking up space.
    // Reroll FTW.
    const u8 *last_block = buf_end - 32;
    while (buf < last_block) {
        m256 lchars = load256(buf);
        rv = JOIN(MATCH_ALGO, fwdBlock)(wide_mask_lo, wide_mask_hi, lchars, buf,
                                        low4bits, zeroes, run_len
#ifdef MULTIACCEL_DOUBLE
                                        , run_len2
#endif
                                        );
        if (rv) {
            return rv;
        }
        buf += 32;
    }

    // Use an unaligned load to mop up the last 32 bytes and get an accurate
    // picture to buf_end.
    assert(buf <= buf_end && buf >= buf_end - 32);
    chars = loadu256(buf_end - 32);
    rv = JOIN(MATCH_ALGO, fwdBlock)(wide_mask_lo, wide_mask_hi, chars, buf_end - 32,
                                    low4bits, zeroes, run_len
#ifdef MULTIACCEL_DOUBLE
                                    , run_len2
#endif
                                    );
    if (rv) {
        return rv;
    }

    return buf_end;
}
