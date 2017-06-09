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

/** \file
 * \brief Noodle literal matcher: runtime.
 */
#include "hwlm.h"
#include "noodle_engine.h"
#include "noodle_internal.h"
#include "ue2common.h"
#include "util/arch.h"
#include "util/bitutils.h"
#include "util/compare.h"
#include "util/intrinsics.h"
#include "util/join.h"
#include "util/masked_move.h"
#include "util/simd_utils.h"

#include <ctype.h>
#include <stdbool.h>
#include <string.h>

/** \brief Noodle runtime context. */
struct cb_info {
    HWLMCallback cb; //!< callback function called on match
    u32 id; //!< ID to pass to callback on match
    void *ctx; //!< caller-supplied context to pass to callback
    size_t offsetAdj; //!< used in streaming mode
};

#if defined(HAVE_AVX512)
#define CHUNKSIZE 64
#define MASK_TYPE m512
#define Z_BITS 64
#define Z_TYPE u64a
#elif defined(HAVE_AVX2)
#define CHUNKSIZE 32
#define MASK_TYPE m256
#define Z_BITS 32
#define Z_TYPE u32
#else
#define CHUNKSIZE 16
#define MASK_TYPE m128
#define Z_BITS 32
#define Z_TYPE u32
#endif


#define RETURN_IF_TERMINATED(x)                                                \
    {                                                                          \
        if ((x) == HWLM_TERMINATED) {                                          \
            return HWLM_TERMINATED;                                            \
        }                                                                      \
    }

#define SINGLE_ZSCAN()                                                         \
    do {                                                                       \
        while (unlikely(z)) {                                                  \
            Z_TYPE pos = JOIN(findAndClearLSB_, Z_BITS)(&z);                   \
            size_t matchPos = d - buf + pos;                                   \
            DEBUG_PRINTF("match pos %zu\n", matchPos);                        \
            hwlmcb_rv_t rv = final(buf, len, key, 1, 0, 0, noCase, cbi,        \
                                   matchPos);                                  \
            RETURN_IF_TERMINATED(rv);                                          \
        }                                                                      \
    } while (0)

#define DOUBLE_ZSCAN()                                                         \
    do {                                                                       \
        while (unlikely(z)) {                                                  \
            Z_TYPE pos = JOIN(findAndClearLSB_, Z_BITS)(&z);                   \
            size_t matchPos = d - buf + pos - 1;                               \
            DEBUG_PRINTF("match pos %zu\n", matchPos);                        \
            hwlmcb_rv_t rv = final(buf, len, key, keyLen, keyOffset, 1,        \
                                   noCase, cbi, matchPos);                     \
            RETURN_IF_TERMINATED(rv);                                          \
        }                                                                      \
    } while (0)

static really_inline
u8 caseClear8(u8 x, bool noCase) {
    return (u8)(noCase ? (x & (u8)0xdf) : x);
}

// Make sure the rest of the string is there. The single character scanner
// is used only for single chars with case insensitivity used correctly,
// so it can go straight to the callback if we get this far.
static really_inline
hwlm_error_t final(const u8 *buf, size_t len, const u8 *key, size_t keyLen,
                   size_t keyOffset, bool is_double, bool noCase,
                   const struct cb_info *cbi, size_t pos) {
    pos -= keyOffset;
    if (is_double) {
        if (pos + keyLen > len) {
            return HWLM_SUCCESS;
        }
        if (cmpForward(buf + pos, key, keyLen, noCase)) { // ret 1 on mismatch
            return HWLM_SUCCESS;
        }
    }
    pos += cbi->offsetAdj;
    DEBUG_PRINTF("match @ %zu->%zu\n", pos, (pos + keyLen - 1));
    hwlmcb_rv_t rv = cbi->cb(pos, (pos + keyLen - 1), cbi->id, cbi->ctx);
    if (rv == HWLM_TERMINATE_MATCHING) {
        return HWLM_TERMINATED;
    }
    return HWLM_SUCCESS;
}

#if defined(HAVE_AVX512)
#define CHUNKSIZE 64
#define MASK_TYPE m512
#include "noodle_engine_avx512.c"
#elif defined(HAVE_AVX2)
#define CHUNKSIZE 32
#define MASK_TYPE m256
#include "noodle_engine_avx2.c"
#else
#define CHUNKSIZE 16
#define MASK_TYPE m128
#include "noodle_engine_sse.c"
#endif

static really_inline
hwlm_error_t scanSingleMain(const u8 *buf, size_t len, const u8 *key,
                            bool noCase, const struct cb_info *cbi) {

    const MASK_TYPE mask1 = getMask(key[0], noCase);
    const MASK_TYPE caseMask = getCaseMask();

#if !defined(HAVE_AVX512)
    hwlm_error_t rv;
    size_t end = len;

    if (len < CHUNKSIZE) {
        rv = scanSingleShort(buf, len, key, noCase, caseMask, mask1, cbi, 0, len);
        return rv;
    }

    if (len == CHUNKSIZE) {
        rv = scanSingleUnaligned(buf, len, 0, key, noCase, caseMask, mask1, cbi,
                                 0, len);
        return rv;
    }

    uintptr_t data = (uintptr_t)buf;
    uintptr_t s2Start = ROUNDUP_N(data, CHUNKSIZE) - data;
    uintptr_t last = data + end;
    uintptr_t s2End = ROUNDDOWN_N(last, CHUNKSIZE) - data;
    uintptr_t s3Start = len - CHUNKSIZE;

    if (s2Start) {
        // first scan out to the fast scan starting point
        DEBUG_PRINTF("stage 1: -> %zu\n", s2Start);
        rv = scanSingleUnaligned(buf, len, 0, key, noCase, caseMask, mask1, cbi,
                                 0, s2Start);
        RETURN_IF_TERMINATED(rv);
    }

    if (likely(s2Start != s2End)) {
        // scan as far as we can, bounded by the last point this key can
        // possibly match
        DEBUG_PRINTF("fast: ~ %zu -> %zu\n", s2Start, s2End);
        rv = scanSingleFast(buf, len, key, noCase, caseMask, mask1, cbi,
                            s2Start, s2End);
        RETURN_IF_TERMINATED(rv);
    }

    // if we are done bail out
    if (s2End == end) {
        return HWLM_SUCCESS;
    }

    DEBUG_PRINTF("stage 3: %zu -> %zu\n", s2End, end);
    rv = scanSingleUnaligned(buf, len, s3Start, key, noCase, caseMask, mask1,
                             cbi, s2End, end);

    return rv;
#else // HAVE_AVX512
    return scanSingle512(buf, len, key, noCase, caseMask, mask1, cbi);
#endif
}

static really_inline
hwlm_error_t scanDoubleMain(const u8 *buf, size_t len, const u8 *key,
                            size_t keyLen, size_t keyOffset, bool noCase,
                            const struct cb_info *cbi) {
    // we stop scanning for the key-fragment when the rest of the key can't
    // possibly fit in the remaining buffer
    size_t end = len - keyLen + keyOffset + 2;

    const MASK_TYPE caseMask = getCaseMask();
    const MASK_TYPE mask1 = getMask(key[keyOffset + 0], noCase);
    const MASK_TYPE mask2 = getMask(key[keyOffset + 1], noCase);

#if !defined(HAVE_AVX512)
    hwlm_error_t rv;

    if (end - keyOffset < CHUNKSIZE) {
        rv = scanDoubleShort(buf, len, key, keyLen, keyOffset, noCase, caseMask,
                             mask1, mask2, cbi, keyOffset, end);
        return rv;
    }
    if (end - keyOffset == CHUNKSIZE) {
        rv = scanDoubleUnaligned(buf, len, keyOffset, key, keyLen, keyOffset,
                                 noCase, caseMask, mask1, mask2, cbi, keyOffset,
                                 end);
        return rv;
    }

    uintptr_t data = (uintptr_t)buf;
    uintptr_t s2Start = ROUNDUP_N(data + keyOffset, CHUNKSIZE) - data;
    uintptr_t s1End = s2Start + 1;
    uintptr_t last = data + end;
    uintptr_t s2End = ROUNDDOWN_N(last, CHUNKSIZE) - data;
    uintptr_t s3Start = end - CHUNKSIZE;
    uintptr_t off = keyOffset;

    if (s2Start != keyOffset) {
        // first scan out to the fast scan starting point plus one char past to
        // catch the key on the overlap
        DEBUG_PRINTF("stage 1: -> %zu\n", s2Start);
        rv = scanDoubleUnaligned(buf, len, keyOffset, key, keyLen, keyOffset,
                                 noCase, caseMask, mask1, mask2, cbi, off,
                                 s1End);
        RETURN_IF_TERMINATED(rv);
    }
    off = s1End;

    if (s2Start >= end) {
        DEBUG_PRINTF("s2 == mL %zu\n", end);
        return HWLM_SUCCESS;
    }

    if (likely(s2Start != s2End)) {
        // scan as far as we can, bounded by the last point this key can
        // possibly match
        DEBUG_PRINTF("fast: ~ %zu -> %zu\n", s2Start, s3Start);
        rv = scanDoubleFast(buf, len, key, keyLen, keyOffset, noCase, caseMask,
                            mask1, mask2, cbi, s2Start, s2End);
        RETURN_IF_TERMINATED(rv);
        off = s2End;
    }

    // if there isn't enough data left to match the key, bail out
    if (s2End == end) {
        return HWLM_SUCCESS;
    }

    DEBUG_PRINTF("stage 3: %zu -> %zu\n", s3Start, end);
    rv = scanDoubleUnaligned(buf, len, s3Start, key, keyLen, keyOffset, noCase,
                             caseMask, mask1, mask2, cbi, off, end);

    return rv;
#else // AVX512
    return scanDouble512(buf, len, key, keyLen, keyOffset, noCase, caseMask,
                         mask1, mask2, cbi, keyOffset, end);
#endif // AVX512
}


static really_inline
hwlm_error_t scanSingleNoCase(const u8 *buf, size_t len, const u8 *key,
                              const struct cb_info *cbi) {
    return scanSingleMain(buf, len, key, 1, cbi);
}

static really_inline
hwlm_error_t scanSingleCase(const u8 *buf, size_t len, const u8 *key,
                            const struct cb_info *cbi) {
    return scanSingleMain(buf, len, key, 0, cbi);
}

// Single-character specialisation, used when keyLen = 1
static really_inline
hwlm_error_t scanSingle(const u8 *buf, size_t len, const u8 *key, bool noCase,
                        const struct cb_info *cbi) {
    if (!ourisalpha(key[0])) {
        noCase = 0; // force noCase off if we don't have an alphabetic char
    }

    // kinda ugly, but this forces constant propagation
    if (noCase) {
        return scanSingleNoCase(buf, len, key, cbi);
    } else {
        return scanSingleCase(buf, len, key, cbi);
    }
}


static really_inline
hwlm_error_t scanDoubleNoCase(const u8 *buf, size_t len, const u8 *key,
                              size_t keyLen, size_t keyOffset,
                              const struct cb_info *cbi) {
    return scanDoubleMain(buf, len, key, keyLen, keyOffset, 1, cbi);
}

static really_inline
hwlm_error_t scanDoubleCase(const u8 *buf, size_t len, const u8 *key,
                            size_t keyLen, size_t keyOffset,
                            const struct cb_info *cbi) {
    return scanDoubleMain(buf, len, key, keyLen, keyOffset, 0, cbi);
}


static really_inline
hwlm_error_t scanDouble(const u8 *buf, size_t len, const u8 *key, size_t keyLen,
                        size_t keyOffset, bool noCase,
                        const struct cb_info *cbi) {
    // kinda ugly, but this forces constant propagation
    if (noCase) {
        return scanDoubleNoCase(buf, len, key, keyLen, keyOffset, cbi);
    } else {
        return scanDoubleCase(buf, len, key, keyLen, keyOffset, cbi);
    }
}

// main entry point for the scan code
static really_inline
hwlm_error_t scan(const u8 *buf, size_t len, const u8 *key, size_t keyLen,
                  size_t keyOffset, bool noCase, const struct cb_info *cbi) {
    if (len < keyLen) {
        // can't find string of length keyLen in a shorter buffer
        return HWLM_SUCCESS;
    }

    if (keyLen == 1) {
        assert(keyOffset == 0);
        return scanSingle(buf, len, key, noCase, cbi);
    } else {
        return scanDouble(buf, len, key, keyLen, keyOffset, noCase, cbi);
    }
}

/** \brief Block-mode scanner. */
hwlm_error_t noodExec(const struct noodTable *n, const u8 *buf, size_t len,
                      size_t offset_adj, HWLMCallback cb, void *ctxt) {
    assert(n && buf);

    struct cb_info cbi = { cb, n->id, ctxt, offset_adj };
    DEBUG_PRINTF("nood scan of %zu bytes for %*s\n", len, n->len, n->str);
    return scan(buf, len, n->str, n->len, n->key_offset, n->nocase, &cbi);
}

/** \brief Streaming-mode scanner. */
hwlm_error_t noodExecStreaming(const struct noodTable *n, const u8 *hbuf,
                               size_t hlen, const u8 *buf, size_t len,
                               HWLMCallback cb, void *ctxt, u8 *temp_buf,
                               UNUSED size_t temp_buffer_size) {
    assert(n);

    struct cb_info cbi = {cb, n->id, ctxt, 0};
    hwlm_error_t rv;

    if (hlen) {
        assert(hbuf);

        size_t tl1 = MIN(n->len - 1, hlen);
        size_t tl2 = MIN(n->len - 1, len);
        size_t temp_len = tl1 + tl2;
        assert(temp_len < temp_buffer_size);
        memcpy(temp_buf, hbuf + hlen - tl1, tl1);
        memcpy(temp_buf + tl1, buf, tl2);

        cbi.offsetAdj = -tl1;
        rv = scan(temp_buf, temp_len, n->str, n->len, n->key_offset, n->nocase,
                  &cbi);
        if (rv == HWLM_TERMINATED) {
            return HWLM_TERMINATED;
        }
    }

    assert(buf);

    cbi.offsetAdj = 0;
    return scan(buf, len, n->str, n->len, n->key_offset, n->nocase, &cbi);
}
