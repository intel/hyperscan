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

/*
 * Matches a byte in a charclass using three shuffles
 */

#include "config.h"
#include "ue2common.h"
#include "multiaccel_common.h"

/*
 * include "block" function
 */
#include "truffle_common.h"

/*
 * single-byte truffle fwd match function, should only be defined when not
 * compiling multiaccel
 */
static really_inline
const u8 *JOIN(MATCH_ALGO, fwdBlock)(m256 shuf_mask_lo_highclear, m256 shuf_mask_lo_highset,
                                     m256 v, const u8 *buf, const u8 run_len
#ifdef MULTIACCEL_DOUBLE
                                     , const u8 run_len2
#endif
                                     ) {
    u64a z = (u64a) block(shuf_mask_lo_highclear, shuf_mask_lo_highset, v);
    return (*JOIN4(MATCH_ALGO, match_funcs, _, 64)[run_len])(buf, z ^ 0xFFFFFFFF
#ifdef MULTIACCEL_DOUBLE
                                                             , run_len2
#endif
                                                             );
}

const u8 *JOIN(MATCH_ALGO, truffleExec)(m128 shuf_mask_lo_highclear,
                                        m128 shuf_mask_lo_highset,
                                        const u8 *buf, const u8 *buf_end, const u8 run_len
#ifdef MULTIACCEL_DOUBLE
                                        , const u8 run_len2
#endif
                                        ) {
    DEBUG_PRINTF("run_len %zu\n", buf_end - buf);
    const m256 wide_clear = set2x128(shuf_mask_lo_highclear);
    const m256 wide_set = set2x128(shuf_mask_lo_highset);

    assert(buf && buf_end);
    assert(buf < buf_end);
    const u8 *rv;

    if (buf_end - buf < 32) {
        return truffleMini(wide_clear, wide_set, buf, buf_end);
    }

    size_t min = (size_t)buf % 32;
    assert(buf_end - buf >= 32);

    // Preconditioning: most of the time our buffer won't be aligned.
    m256 chars = loadu256(buf);
    rv = JOIN(MATCH_ALGO, fwdBlock)(wide_clear, wide_set, chars, buf, run_len
#ifdef MULTIACCEL_DOUBLE
                                    , run_len2
#endif
                                    );
    if (rv) {
        return rv;
    }
    buf += (32 - min);

    const u8 *last_block = buf_end - 32;
    while (buf < last_block) {
        m256 lchars = load256(buf);
        rv = JOIN(MATCH_ALGO, fwdBlock)(wide_clear, wide_set, lchars,
                                        buf, run_len
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
    rv = JOIN(MATCH_ALGO, fwdBlock)(wide_clear, wide_set, chars,
                                    buf_end - 32, run_len
#ifdef MULTIACCEL_DOUBLE
                                    , run_len2
#endif
                                    );
    if (rv) {
        return rv;
    }

    return buf_end;
}
