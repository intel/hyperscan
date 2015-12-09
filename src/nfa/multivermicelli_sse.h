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

#define VERM_BOUNDARY 16
#define VERM_TYPE m128
#define VERM_SET_FN set16x8

#include "multiaccel_common.h"

static really_inline
const u8 *JOIN(MATCH_ALGO, vermUnalignNocase)(m128 chars,
                                              const u8 *buf,
                                              const u8 run_len
#ifdef MULTIACCEL_DOUBLE
                                              , const u8 run_len2
#endif
                                              ) {
    m128 casemask = set16x8(CASE_CLEAR);
    const u8 *ptr;
    m128 data = loadu128(buf);
    u32 z = movemask128(eq128(chars, and128(casemask, data)));
    ptr = (*JOIN4(MATCH_ALGO, match_funcs, _, 32)[run_len])
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
const u8 *JOIN(MATCH_ALGO, vermUnalign)(m128 chars,
                                        const u8 *buf,
                                        const u8 run_len
#ifdef MULTIACCEL_DOUBLE
                                        , const u8 run_len2
#endif
                                        ) {
    const u8 *ptr;

    m128 data = loadu128(buf);
    u32 z = movemask128(eq128(chars, data));
    ptr = (*JOIN4(MATCH_ALGO, match_funcs, _, 32)[run_len])
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
 * 16-byte pipeline, for smaller scans
 */
static
const u8 *JOIN(MATCH_ALGO, vermPipeline16)(m128 chars,
                                                 const u8 *buf,
                                                 const u8 *buf_end,
                                                 const u8 run_len
#ifdef MULTIACCEL_DOUBLE
                                                 , const u8 run_len2
#endif
                                                 ) {
    const u8* ptr, *last_buf;
    u32 last_res;

    // pipeline prologue: scan first 16 bytes
    m128 data = load128(buf);
    u32 z = movemask128(eq128(chars, data));
    last_buf = buf;
    last_res = z;
    buf += 16;

    // now, start the pipeline!
    assert((size_t)buf % 16 == 0);
    for (; buf + 15 < buf_end; buf += 16) {
        // scan more data
        data = load128(buf);
        z = movemask128(eq128(chars, data));

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
 * 16-byte pipeline, for smaller scans
 */
static
const u8 *JOIN(MATCH_ALGO, vermPipeline16Nocase)(m128 chars,
                                                         const u8 *buf,
                                                         const u8 *buf_end,
                                                         const u8 run_len
#ifdef MULTIACCEL_DOUBLE
                                                         , const u8 run_len2
#endif
                                                         ) {
    m128 casemask = set16x8(CASE_CLEAR);
    const u8* ptr, *last_buf;
    u32 last_res;

    // pipeline prologue: scan first 16 bytes
    m128 data = load128(buf);
    u32 z = movemask128(eq128(chars, and128(casemask, data)));
    last_buf = buf;
    last_res = z;
    buf += 16;

    // now, start the pipeline!
    assert((size_t)buf % 16 == 0);
    for (; buf + 15 < buf_end; buf += 16) {
        // scan more data
        data = load128(buf);
        z = movemask128(eq128(chars, and128(casemask, data)));

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
const u8 *JOIN(MATCH_ALGO, vermPipeline32)(m128 chars,
                                                   const u8 *buf,
                                                   const u8 *buf_end,
                                                   const u8 run_len
#ifdef MULTIACCEL_DOUBLE
                                                   , const u8 run_len2
#endif
                                                   ) {
    const u8* ptr, *last_buf;
    u32 res;

    // pipeline prologue: scan first 32 bytes
    m128 data1 = load128(buf);
    u32 z1 = movemask128(eq128(chars, data1));
    m128 data2 = load128(buf + 16);
    u32 z2 = movemask128(eq128(chars, data2));

    // store the results
    u32 last_res = z1 | (z2 << VERM_BOUNDARY);
    last_buf = buf;
    buf += 32;


    // now, start the pipeline!
    assert((size_t)buf % 16 == 0);
    for (; buf + 31 < buf_end; buf += 32) {
        // scan more data
        data1 = load128(buf);
        z1 = movemask128(eq128(chars, data1));
        data2 = load128(buf + 16);
        z2 = movemask128(eq128(chars, data2));
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
    if (buf + 15 < buf_end) {
        return JOIN(MATCH_ALGO, vermPipeline16)(chars, buf, buf_end, run_len
#ifdef MULTIACCEL_DOUBLE
                                                , run_len2
#endif
                                                );
    }
    assert(buf <= buf_end && buf >= buf_end - 16);

    return NULL;
}

/*
 * 32-byte caseless pipeline, for bigger scans
 */
static
const u8 *JOIN(MATCH_ALGO, vermPipeline32Nocase)(m128 chars,
                                                         const u8 *buf,
                                                         const u8 *buf_end,
                                                         const u8 run_len
#ifdef MULTIACCEL_DOUBLE
                                                         , const u8 run_len2
#endif
                                                         ) {
    m128 casemask = set16x8(CASE_CLEAR);
    const u8* ptr, *last_buf;
    u32 last_res;

    // pipeline prologue: scan first 32 bytes
    m128 data1 = load128(buf);
    u32 z1 = movemask128(eq128(chars, and128(casemask, data1)));
    m128 data2 = load128(buf + 16);
    u32 z2 = movemask128(eq128(chars, and128(casemask, data2)));
    u32 z = z1 | (z2 << VERM_BOUNDARY);

    last_res = z;
    last_buf = buf;
    buf += 32;

    // now, start the pipeline!
    assert((size_t)buf % 16 == 0);
    for (; buf + 31 < buf_end; buf += 32) {
        // scan more data
        data1 = load128(buf);
        z1 = movemask128(eq128(chars, and128(casemask, data1)));
        data2 = load128(buf + 16);
        z2 = movemask128(eq128(chars, and128(casemask, data2)));
        z = z1 | (z2 << 16);

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
        last_res = z;
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
    if (buf + 15 < buf_end) {
        return JOIN(MATCH_ALGO, vermPipeline16Nocase)(chars, buf, buf_end, run_len
#ifdef MULTIACCEL_DOUBLE
                                                              , run_len2
#endif
                                                              );
    }
    assert(buf <= buf_end && buf >= buf_end - 16);

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
    if (buf_end - buf < VERM_BOUNDARY) {
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

    VERM_TYPE chars = VERM_SET_FN(c); /* nocase already uppercase */

    uintptr_t min = (uintptr_t)buf % VERM_BOUNDARY;

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
        buf += VERM_BOUNDARY - min;
    }

    // if we have enough data, run bigger pipeline; otherwise run smaller one
    if (buf_end - buf >= 128) {
        ptr = nocase ? JOIN(MATCH_ALGO, vermPipeline32Nocase)(chars,
                buf, buf_end, run_len
#ifdef MULTIACCEL_DOUBLE
                , run_len2
#endif
                ) : JOIN(MATCH_ALGO, vermPipeline32)(chars,
                        buf, buf_end, run_len
#ifdef MULTIACCEL_DOUBLE
                        , run_len2
#endif
                        );
        if (unlikely(ptr)) {
            return ptr;
        }
    } else if (buf_end - buf >= 16){
        ptr = nocase ? JOIN(MATCH_ALGO, vermPipeline16Nocase)(chars,
                buf, buf_end, run_len
#ifdef MULTIACCEL_DOUBLE
                , run_len2
#endif
                ) : JOIN(MATCH_ALGO, vermPipeline16)(chars,
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
            buf_end - VERM_BOUNDARY, run_len
#ifdef MULTIACCEL_DOUBLE
            , run_len2
#endif
            ) : JOIN(MATCH_ALGO, vermUnalign)(chars,
                    buf_end - VERM_BOUNDARY, run_len
#ifdef MULTIACCEL_DOUBLE
                    , run_len2
#endif
                    );

    // run our pipeline
    return ptr ? ptr : buf_end;
}
