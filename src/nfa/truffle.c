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


#include "ue2common.h"
#include "truffle.h"
#include "util/bitutils.h"
#include "util/simd_utils.h"
#include "util/simd_utils_ssse3.h"

#include "truffle_common.h"

#if !defined(__AVX2__)

static really_inline
const u8 *lastMatch(const u8 *buf, u32 z) {
    if (unlikely(z != 0xffff)) {
        u32 pos = clz32(~z & 0xffff);
        assert(pos >= 16 && pos < 32);
        return buf + (31 - pos);
    }

    return NULL; // no match
}

static really_inline
const u8 *fwdBlock(m128 shuf_mask_lo_highclear, m128 shuf_mask_lo_highset,
                   m128 v, const u8 *buf) {
    u32 z = block(shuf_mask_lo_highclear, shuf_mask_lo_highset, v);
    return firstMatch(buf, z);
}

static really_inline
const u8 *revBlock(m128 shuf_mask_lo_highclear, m128 shuf_mask_lo_highset,
                   m128 v, const u8 *buf) {
    u32 z = block(shuf_mask_lo_highclear, shuf_mask_lo_highset, v);
    return lastMatch(buf, z);
}

const u8 *truffleExec(m128 shuf_mask_lo_highclear,
                      m128 shuf_mask_lo_highset,
                      const u8 *buf, const u8 *buf_end) {
    DEBUG_PRINTF("len %zu\n", buf_end - buf);

    assert(buf && buf_end);
    assert(buf < buf_end);
    const u8 *rv;

    if (buf_end - buf < 16) {
        return truffleMini(shuf_mask_lo_highclear, shuf_mask_lo_highset, buf,
                           buf_end);
    }

    size_t min = (size_t)buf % 16;
    assert(buf_end - buf >= 16);

    // Preconditioning: most of the time our buffer won't be aligned.
    m128 chars = loadu128(buf);
    rv = fwdBlock(shuf_mask_lo_highclear, shuf_mask_lo_highset, chars, buf);
    if (rv) {
        return rv;
    }
    buf += (16 - min);

    const u8 *last_block = buf_end - 16;
    while (buf < last_block) {
        m128 lchars = load128(buf);
        rv = fwdBlock(shuf_mask_lo_highclear, shuf_mask_lo_highset, lchars,
                      buf);
        if (rv) {
            return rv;
        }
        buf += 16;
    }

    // Use an unaligned load to mop up the last 16 bytes and get an accurate
    // picture to buf_end.
    assert(buf <= buf_end && buf >= buf_end - 16);
    chars = loadu128(buf_end - 16);
    rv = fwdBlock(shuf_mask_lo_highclear, shuf_mask_lo_highset, chars,
                  buf_end - 16);
    if (rv) {
        return rv;
    }

    return buf_end;
}

static
const u8 *truffleRevMini(m128 shuf_mask_lo_highclear,
                         m128 shuf_mask_lo_highset, const u8 *buf,
                         const u8 *buf_end) {
    uintptr_t len = buf_end - buf;
    assert(len < 16);

    m128 chars = zeroes128();
    memcpy(&chars, buf, len);

    u32 mask = (0xFFFF >> (16 - len)) ^ 0xFFFF;
    u32 z = block(shuf_mask_lo_highclear, shuf_mask_lo_highset, chars);
    const u8 *rv = lastMatch(buf, z | mask);

    if (rv) {
        return rv;
    }
    return buf - 1;
}

const u8 *rtruffleExec(m128 shuf_mask_lo_highclear,
                       m128 shuf_mask_lo_highset,
                       const u8 *buf, const u8 *buf_end) {
    assert(buf && buf_end);
    assert(buf < buf_end);
    const u8 *rv;

    DEBUG_PRINTF("len %zu\n", buf_end - buf);

    if (buf_end - buf < 16) {
        return truffleRevMini(shuf_mask_lo_highclear, shuf_mask_lo_highset, buf,
                              buf_end);
    }

    assert(buf_end - buf >= 16);

    // Preconditioning: most of the time our buffer won't be aligned.
    m128 chars = loadu128(buf_end - 16);
    rv = revBlock(shuf_mask_lo_highclear, shuf_mask_lo_highset, chars,
                  buf_end - 16);
    if (rv) {
        return rv;
    }
    buf_end = (const u8 *)((size_t)buf_end & ~((size_t)0xf));

    const u8 *last_block = buf + 16;
    while (buf_end > last_block) {
        buf_end -= 16;
        m128 lchars = load128(buf_end);
        rv = revBlock(shuf_mask_lo_highclear, shuf_mask_lo_highset, lchars,
                      buf_end);
        if (rv) {
            return rv;
        }
    }

    // Use an unaligned load to mop up the last 16 bytes and get an accurate
    // picture to buf_end.
    chars = loadu128(buf);
    rv = revBlock(shuf_mask_lo_highclear, shuf_mask_lo_highset, chars, buf);
    if (rv) {
        return rv;
    }

    return buf - 1;
}

#else

static really_inline
const u8 *lastMatch(const u8 *buf, u32 z) {
    if (unlikely(z != 0xffffffff)) {
        u32 pos = clz32(~z);
        assert(pos < 32);
        return buf + (31 - pos);
    }

    return NULL; // no match
}

static really_inline
const u8 *fwdBlock(m256 shuf_mask_lo_highclear, m256 shuf_mask_lo_highset,
                   m256 v, const u8 *buf) {
    u32 z = block(shuf_mask_lo_highclear, shuf_mask_lo_highset, v);
    return firstMatch(buf, z);
}

static really_inline
const u8 *revBlock(m256 shuf_mask_lo_highclear, m256 shuf_mask_lo_highset,
                   m256 v, const u8 *buf) {
    u32 z = block(shuf_mask_lo_highclear, shuf_mask_lo_highset, v);
    return lastMatch(buf, z);
}

const u8 *truffleExec(m128 shuf_mask_lo_highclear,
                      m128 shuf_mask_lo_highset,
                      const u8 *buf, const u8 *buf_end) {
    DEBUG_PRINTF("len %zu\n", buf_end - buf);
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
    rv = fwdBlock(wide_clear, wide_set, chars, buf);
    if (rv) {
        return rv;
    }
    buf += (32 - min);

    const u8 *last_block = buf_end - 32;
    while (buf < last_block) {
        m256 lchars = load256(buf);
        rv = fwdBlock(wide_clear, wide_set, lchars, buf);
        if (rv) {
            return rv;
        }
        buf += 32;
    }

    // Use an unaligned load to mop up the last 32 bytes and get an accurate
    // picture to buf_end.
    assert(buf <= buf_end && buf >= buf_end - 32);
    chars = loadu256(buf_end - 32);
    rv = fwdBlock(wide_clear, wide_set, chars, buf_end - 32);
    if (rv) {
        return rv;
    }
    return buf_end;
}

static
const u8 *truffleRevMini(m256 shuf_mask_lo_highclear,
                         m256 shuf_mask_lo_highset, const u8 *buf,
                         const u8 *buf_end) {
    uintptr_t len = buf_end - buf;
    assert(len < 32);

    m256 chars = zeroes256();
    memcpy(&chars, buf, len);

    u32 mask = (0xFFFFFFFF >> (32 - len)) ^ 0xFFFFFFFF;
    u32 z = block(shuf_mask_lo_highclear, shuf_mask_lo_highset, chars);
    const u8 *rv = lastMatch(buf, z | mask);

    if (rv) {
        return rv;
    }
    return buf - 1;
}


const u8 *rtruffleExec(m128 shuf_mask_lo_highclear,
                       m128 shuf_mask_lo_highset,
                       const u8 *buf, const u8 *buf_end) {
    const m256 wide_clear = set2x128(shuf_mask_lo_highclear);
    const m256 wide_set = set2x128(shuf_mask_lo_highset);
    assert(buf && buf_end);
    assert(buf < buf_end);
    const u8 *rv;

    DEBUG_PRINTF("len %zu\n", buf_end - buf);

    if (buf_end - buf < 32) {
        return truffleRevMini(wide_clear, wide_set, buf, buf_end);
    }

    assert(buf_end - buf >= 32);

    // Preconditioning: most of the time our buffer won't be aligned.
    m256 chars = loadu256(buf_end - 32);
    rv = revBlock(wide_clear, wide_set, chars,
                  buf_end - 32);
    if (rv) {
        return rv;
    }
    buf_end = (const u8 *)((size_t)buf_end & ~((size_t)0x1f));

    const u8 *last_block = buf + 32;
    while (buf_end > last_block) {
        buf_end -= 32;
        m256 lchars = load256(buf_end);
        rv = revBlock(wide_clear, wide_set, lchars, buf_end);
        if (rv) {
            return rv;
        }
    }

    // Use an unaligned load to mop up the last 32 bytes and get an accurate
    // picture to buf_end.
    chars = loadu256(buf);
    rv = revBlock(wide_clear, wide_set, chars, buf);
    if (rv) {
        return rv;
    }
    return buf - 1;
}

#endif
