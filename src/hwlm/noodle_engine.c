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
#include "scratch.h"
#include "ue2common.h"
#include "util/arch.h"
#include "util/bitutils.h"
#include "util/compare.h"
#include "util/intrinsics.h"
#include "util/join.h"
#include "util/masked_move.h"
#include "util/partial_store.h"
#include "util/simd_utils.h"

#include <ctype.h>
#include <stdbool.h>
#include <string.h>

/** \brief Noodle runtime context. */
struct cb_info {
    HWLMCallback cb; //!< callback function called on match
    u32 id; //!< ID to pass to callback on match
    struct hs_scratch *scratch; //!< scratch to pass to callback
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
            DEBUG_PRINTF("match pos %zu\n", matchPos);                         \
            hwlmcb_rv_t rv = final(n, buf, len, 1, cbi, matchPos);             \
            RETURN_IF_TERMINATED(rv);                                          \
        }                                                                      \
    } while (0)

#define DOUBLE_ZSCAN()                                                         \
    do {                                                                       \
        while (unlikely(z)) {                                                  \
            Z_TYPE pos = JOIN(findAndClearLSB_, Z_BITS)(&z);                   \
            size_t matchPos = d - buf + pos - 1;                               \
            DEBUG_PRINTF("match pos %zu\n", matchPos);                         \
            hwlmcb_rv_t rv = final(n, buf, len, 0, cbi, matchPos);             \
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
hwlm_error_t final(const struct noodTable *n, const u8 *buf, UNUSED size_t len,
                   char single, const struct cb_info *cbi, size_t pos) {
    if (single) {
        if (n->msk_len == 1) {
            goto match;
        }
    }
    assert(len >= n->msk_len);
    u64a v =
        partial_load_u64a(buf + pos + n->key_offset - n->msk_len, n->msk_len);
    DEBUG_PRINTF("v %016llx msk %016llx cmp %016llx\n", v, n->msk, n->cmp);
    if ((v & n->msk) != n->cmp) {
        /* mask didn't match */
        return HWLM_SUCCESS;
    }

match:
    pos -= cbi->offsetAdj;
    DEBUG_PRINTF("match @ %zu\n", pos + n->key_offset);
    hwlmcb_rv_t rv = cbi->cb(pos + n->key_offset - 1, cbi->id, cbi->scratch);
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
hwlm_error_t scanSingleMain(const struct noodTable *n, const u8 *buf,
                            size_t len, size_t start, bool noCase,
                            const struct cb_info *cbi) {

    const MASK_TYPE mask1 = getMask(n->key0, noCase);
    const MASK_TYPE caseMask = getCaseMask();

    size_t offset = start + n->msk_len - 1;
    size_t end = len;
    assert(offset < end);

#if !defined(HAVE_AVX512)
    hwlm_error_t rv;

    if (end - offset < CHUNKSIZE) {
        rv = scanSingleShort(n, buf, len, noCase, caseMask, mask1, cbi, offset,
                             end);
        return rv;
    }

    if (end - offset == CHUNKSIZE) {
        rv = scanSingleUnaligned(n, buf, len, offset, noCase, caseMask, mask1,
                                 cbi, offset, end);
        return rv;
    }

    uintptr_t data = (uintptr_t)buf;
    uintptr_t s2Start = ROUNDUP_N(data + offset, CHUNKSIZE) - data;
    uintptr_t last = data + end;
    uintptr_t s2End = ROUNDDOWN_N(last, CHUNKSIZE) - data;
    uintptr_t s3Start = end - CHUNKSIZE;

    if (offset != s2Start) {
        // first scan out to the fast scan starting point
        DEBUG_PRINTF("stage 1: -> %zu\n", s2Start);
        rv = scanSingleUnaligned(n, buf, len, offset, noCase, caseMask, mask1,
                                 cbi, offset, s2Start);
        RETURN_IF_TERMINATED(rv);
    }

    if (likely(s2Start != s2End)) {
        // scan as far as we can, bounded by the last point this key can
        // possibly match
        DEBUG_PRINTF("fast: ~ %zu -> %zu\n", s2Start, s2End);
        rv = scanSingleFast(n, buf, len, noCase, caseMask, mask1, cbi, s2Start,
                            s2End);
        RETURN_IF_TERMINATED(rv);
    }

    // if we are done bail out
    if (s2End == len) {
        return HWLM_SUCCESS;
    }

    DEBUG_PRINTF("stage 3: %zu -> %zu\n", s2End, len);
    rv = scanSingleUnaligned(n, buf, len, s3Start, noCase, caseMask, mask1, cbi,
                             s2End, len);

    return rv;
#else // HAVE_AVX512
    return scanSingle512(n, buf, len, noCase, caseMask, mask1, cbi, offset,
                         end);
#endif
}

static really_inline
hwlm_error_t scanDoubleMain(const struct noodTable *n, const u8 *buf,
                            size_t len, size_t start, bool noCase,
                            const struct cb_info *cbi) {
    // we stop scanning for the key-fragment when the rest of the key can't
    // possibly fit in the remaining buffer
    size_t end = len - n->key_offset + 2;

    // the first place the key can match
    size_t offset = start + n->msk_len - n->key_offset;

    const MASK_TYPE caseMask = getCaseMask();
    const MASK_TYPE mask1 = getMask(n->key0, noCase);
    const MASK_TYPE mask2 = getMask(n->key1, noCase);

#if !defined(HAVE_AVX512)
    hwlm_error_t rv;

    if (end - offset < CHUNKSIZE) {
        rv = scanDoubleShort(n, buf, len, noCase, caseMask, mask1, mask2, cbi,
                             offset, end);
        return rv;
    }
    if (end - offset == CHUNKSIZE) {
        rv = scanDoubleUnaligned(n, buf, len, offset, noCase, caseMask, mask1,
                                 mask2, cbi, offset, end);
        return rv;
    }

    uintptr_t data = (uintptr_t)buf;
    uintptr_t s2Start = ROUNDUP_N(data + offset, CHUNKSIZE) - data;
    uintptr_t s1End = s2Start + 1;
    uintptr_t last = data + end;
    uintptr_t s2End = ROUNDDOWN_N(last, CHUNKSIZE) - data;
    uintptr_t s3Start = end - CHUNKSIZE;
    uintptr_t off = offset;

    if (s2Start != off) {
        // first scan out to the fast scan starting point plus one char past to
        // catch the key on the overlap
        DEBUG_PRINTF("stage 1: %zu -> %zu\n", off, s2Start);
        rv = scanDoubleUnaligned(n, buf, len, offset, noCase, caseMask, mask1,
                                 mask2, cbi, off, s1End);
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
        rv = scanDoubleFast(n, buf, len, noCase, caseMask, mask1, mask2, cbi,
                            s2Start, s2End);
        RETURN_IF_TERMINATED(rv);
        off = s2End;
    }

    // if there isn't enough data left to match the key, bail out
    if (s2End == end) {
        return HWLM_SUCCESS;
    }

    DEBUG_PRINTF("stage 3: %zu -> %zu\n", s3Start, end);
    rv = scanDoubleUnaligned(n, buf, len, s3Start, noCase, caseMask, mask1,
                             mask2, cbi, off, end);

    return rv;
#else // AVX512
    return scanDouble512(n, buf, len, noCase, caseMask, mask1, mask2, cbi,
                         offset, end);
#endif // AVX512
}


static really_inline
hwlm_error_t scanSingleNoCase(const struct noodTable *n, const u8 *buf,
                              size_t len, size_t start,
                              const struct cb_info *cbi) {
    return scanSingleMain(n, buf, len, start, 1, cbi);
}

static really_inline
hwlm_error_t scanSingleCase(const struct noodTable *n, const u8 *buf,
                            size_t len, size_t start,
                            const struct cb_info *cbi) {
    return scanSingleMain(n, buf, len, start, 0, cbi);
}

// Single-character specialisation, used when keyLen = 1
static really_inline
hwlm_error_t scanSingle(const struct noodTable *n, const u8 *buf, size_t len,
                        size_t start, bool noCase, const struct cb_info *cbi) {
    if (!ourisalpha(n->key0)) {
        noCase = 0; // force noCase off if we don't have an alphabetic char
    }

    // kinda ugly, but this forces constant propagation
    if (noCase) {
        return scanSingleNoCase(n, buf, len, start, cbi);
    } else {
        return scanSingleCase(n, buf, len, start, cbi);
    }
}


static really_inline
hwlm_error_t scanDoubleNoCase(const struct noodTable *n, const u8 *buf,
                              size_t len, size_t start,
                              const struct cb_info *cbi) {
    return scanDoubleMain(n, buf, len, start, 1, cbi);
}

static really_inline
hwlm_error_t scanDoubleCase(const struct noodTable *n, const u8 *buf,
                            size_t len, size_t start,
                            const struct cb_info *cbi) {
    return scanDoubleMain(n, buf, len, start, 0, cbi);
}


static really_inline
hwlm_error_t scanDouble(const struct noodTable *n, const u8 *buf, size_t len,
                        size_t start, bool noCase, const struct cb_info *cbi) {
    // kinda ugly, but this forces constant propagation
    if (noCase) {
        return scanDoubleNoCase(n, buf, len, start, cbi);
    } else {
        return scanDoubleCase(n, buf, len, start, cbi);
    }
}

// main entry point for the scan code
static really_inline
hwlm_error_t scan(const struct noodTable *n, const u8 *buf, size_t len,
                  size_t start, char single, bool noCase,
                  const struct cb_info *cbi) {
    if (len - start < n->msk_len) {
        // can't find string of length keyLen in a shorter buffer
        return HWLM_SUCCESS;
    }

    if (single) {
        return scanSingle(n, buf, len, start, noCase, cbi);
    } else {
        return scanDouble(n, buf, len, start, noCase, cbi);
    }
}

/** \brief Block-mode scanner. */
hwlm_error_t noodExec(const struct noodTable *n, const u8 *buf, size_t len,
                      size_t start, HWLMCallback cb,
                      struct hs_scratch *scratch) {
    assert(n && buf);

    struct cb_info cbi = {cb, n->id, scratch, 0};
    DEBUG_PRINTF("nood scan of %zu bytes for %*s @ %p\n", len, n->msk_len,
                 (const char *)&n->cmp, buf);

    return scan(n, buf, len, start, n->single, n->nocase, &cbi);
}

/** \brief Streaming-mode scanner. */
hwlm_error_t noodExecStreaming(const struct noodTable *n, const u8 *hbuf,
                               size_t hlen, const u8 *buf, size_t len,
                               HWLMCallback cb, struct hs_scratch *scratch) {
    assert(n);

    if (len + hlen < n->msk_len) {
        DEBUG_PRINTF("not enough bytes for a match\n");
        return HWLM_SUCCESS;
    }

    struct cb_info cbi = {cb, n->id, scratch, 0};
    DEBUG_PRINTF("nood scan of %zu bytes (%zu hlen) for %*s @ %p\n", len, hlen,
                 n->msk_len, (const char *)&n->cmp, buf);

    if (hlen && n->msk_len > 1) {
        /*
         * we have history, so build up a buffer from enough of the history
         * buffer plus what we've been given to scan. Since this is relatively
         * short, just check against msk+cmp per byte offset for matches.
         */
        assert(hbuf);
        u8 ALIGN_DIRECTIVE temp_buf[HWLM_LITERAL_MAX_LEN * 2];
        memset(temp_buf, 0, sizeof(temp_buf));

        assert(n->msk_len);
        size_t tl1 = MIN((size_t)n->msk_len - 1, hlen);
        size_t tl2 = MIN((size_t)n->msk_len - 1, len);

        assert(tl1 + tl2 <= sizeof(temp_buf));
        assert(tl1 + tl2 >= n->msk_len);
        assert(tl1 <= sizeof(u64a));
        assert(tl2 <= sizeof(u64a));
        DEBUG_PRINTF("using %zu bytes of hist and %zu bytes of buf\n", tl1, tl2);

        unaligned_store_u64a(temp_buf,
                             partial_load_u64a(hbuf + hlen - tl1, tl1));
        unaligned_store_u64a(temp_buf + tl1, partial_load_u64a(buf, tl2));

        for (size_t i = 0; i <= tl1 + tl2 - n->msk_len; i++) {
            u64a v = unaligned_load_u64a(temp_buf + i);
            if ((v & n->msk) == n->cmp) {
                size_t m_end = -tl1 + i + n->msk_len - 1;
                DEBUG_PRINTF("match @ %zu (i %zu)\n", m_end, i);
                hwlmcb_rv_t rv = cb(m_end, n->id, scratch);
                if (rv == HWLM_TERMINATE_MATCHING) {
                    return HWLM_TERMINATED;
                }
            }
        }
    }

    assert(buf);

    cbi.offsetAdj = 0;
    return scan(n, buf, len, 0, n->single, n->nocase, &cbi);
}
