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

#include "util/bitutils.h"
#include "util/simd_utils.h"
#include "util/unaligned.h"

#include "multiaccel_common.h"

static really_inline
const u8 *JOIN(MATCH_ALGO, vermUnalignNocase)(m256 chars,
                                              const u8 *buf,
                                              const u8 run_len
#ifdef MULTIACCEL_DOUBLE
                                              , const u8 run_len2
#endif
                                              ) {
    m256 casemask = set32x8(CASE_CLEAR);
    const u8 *ptr;
    m256 data = loadu256(buf);
    u32 z = movemask256(eq256(chars, and256(casemask, data)));
    ptr = (*JOIN4(MATCH_ALGO, match_funcs, _, 64)[run_len])
            (buf, z
#ifdef MULTIACCEL_DOUBLE
            , run_len2
#endif
            );
    if (unlikely(ptr)) {
        return ptr;
    }
    return NULL;
}

static really_inline
const u8 *JOIN(MATCH_ALGO, vermUnalign)(m256 chars,
                                        const u8 *buf,
                                        const u8 run_len
#ifdef MULTIACCEL_DOUBLE
                                        , const u8 run_len2
#endif
                                        ) {
    const u8 *ptr;

    m256 data = loadu256(buf);
    u32 z = movemask256(eq256(chars, data));
    ptr = (*JOIN4(MATCH_ALGO, match_funcs, _, 64)[run_len])
            (buf, z
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
 * 32-byte pipeline
 */
static really_inline
const u8 *JOIN(MATCH_ALGO, vermPipeline)(m256 chars,
                                         const u8 *buf,
                                         const u8 *buf_end,
                                         const u8 run_len
#ifdef MULTIACCEL_DOUBLE
                                         , const u8 run_len2
#endif
                                         ) {
    const u8* ptr, *last_buf;
    u32 last_res;

    // pipeline prologue: scan first 32 bytes
    m256 data = load256(buf);
    u32 z = movemask256(eq256(chars, data));
    last_res = z;
    last_buf = buf;
    buf += 32;

    // now, start the pipeline!
    assert((size_t)buf % 32 == 0);
    for (; buf + 31 < buf_end; buf += 32) {
        // scan more data
        data = load256(buf);
        z = movemask256(eq256(chars, data));

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
        last_buf = buf;
        last_res = z;
    }
    assert(buf <= buf_end && buf >= buf_end - 32);

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

    return NULL;
}

/*
 * 32-byte caseless pipeline
 */
static really_inline
const u8 *JOIN(MATCH_ALGO, vermPipelineNocase)(m256 chars,
                                               const u8 *buf,
                                               const u8 *buf_end,
                                               const u8 run_len
#ifdef MULTIACCEL_DOUBLE
                                               , const u8 run_len2
#endif
                                               ) {
    m256 casemask = set32x8(CASE_CLEAR);
    const u8* ptr, *last_buf;
    u32 last_res;

    // pipeline prologue: scan first 32 bytes
    m256 data = load256(buf);
    u32 z = movemask256(eq256(chars, and256(casemask, data)));
    last_res = z;
    last_buf = buf;
    buf += 32;


    // now, start the pipeline!
    assert((size_t)buf % 32 == 0);
    for (; buf + 31 < buf_end; buf += 32) {
        // scan more data
        data = load256(buf);
        z = movemask256(eq256(chars, and256(casemask, data)));

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
        last_buf = buf;
        last_res = z;
    }
    assert(buf <= buf_end && buf >= buf_end - 32);

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

    return NULL;
}

const u8 *JOIN(MATCH_ALGO, vermicelliExec)(char c, char nocase,
                                           const u8 *buf,
                                           const u8 *buf_end,
                                           const u8 run_len
#ifdef MULTIACCEL_DOUBLE
                                           , const u8 run_len2
#endif
                                           ) {
    DEBUG_PRINTF("verm scan %s\\x%02hhx over %zu bytes\n",
                 nocase ? "nocase " : "", c, (size_t)(buf_end - buf));
    assert(buf < buf_end);

    const u8 *ptr;

    // Handle small scans.
    if (buf_end - buf < 32) {
        for (; buf < buf_end; buf++) {
            char cur = (char)*buf;
            if (nocase) {
                cur &= CASE_CLEAR;
            }
            if (cur == c) {
                break;
            }
        }
        return buf;
    }

    m256 chars = set32x8(c); /* nocase already uppercase */

    uintptr_t min = (uintptr_t)buf % 32;

    if (min) {
        ptr = nocase ? JOIN(MATCH_ALGO, vermUnalignNocase)(chars,
                buf, run_len
#ifdef MULTIACCEL_DOUBLE
                , run_len2
#endif
                ) : JOIN(MATCH_ALGO, vermUnalign)(chars,
                        buf, run_len
#ifdef MULTIACCEL_DOUBLE
                        , run_len2
#endif
                        );
        if (unlikely(ptr)) {
            return ptr;
        }
        buf += 32 - min;
    }

    if (buf_end - buf >= 32){
        ptr = nocase ? JOIN(MATCH_ALGO, vermPipelineNocase)(chars,
                buf, buf_end, run_len
#ifdef MULTIACCEL_DOUBLE
                , run_len2
#endif
                ) : JOIN(MATCH_ALGO, vermPipeline)(chars,
                        buf, buf_end, run_len
#ifdef MULTIACCEL_DOUBLE
                        , run_len2
#endif
                        );
        if (unlikely(ptr)) {
            return ptr;
        }
    }

    // final unaligned scan
    ptr = nocase ? JOIN(MATCH_ALGO, vermUnalignNocase)(chars,
            buf_end - 32, run_len
#ifdef MULTIACCEL_DOUBLE
            , run_len2
#endif
            ) : JOIN(MATCH_ALGO, vermUnalign)(chars,
                    buf_end - 32, run_len
#ifdef MULTIACCEL_DOUBLE
                    , run_len2
#endif
                    );

    // run our pipeline
    return ptr ? ptr : buf_end;
}
