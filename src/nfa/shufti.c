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
 * \brief Shufti: character class acceleration.
 *
 * Utilises the SSSE3 pshufb shuffle instruction
 */

#include "shufti.h"
#include "ue2common.h"
#include "util/bitutils.h"
#include "util/simd_utils.h"
#include "util/unaligned.h"

#include "shufti_common.h"

#include "util/simd_utils_ssse3.h"

/** \brief Naive byte-by-byte implementation. */
static really_inline
const u8 *shuftiRevSlow(const u8 *lo, const u8 *hi, const u8 *buf,
                        const u8 *buf_end) {
    assert(buf < buf_end);

    for (buf_end--; buf_end >= buf; buf_end--) {
        u8 c = *buf_end;
        if (lo[c & 0xf] & hi[c >> 4]) {
            break;
        }
    }
    return buf_end;
}

#if !defined(__AVX2__)
/* Normal SSSE3 shufti */

static really_inline
const u8 *firstMatch(const u8 *buf, u32 z) {
    if (unlikely(z != 0xffff)) {
        u32 pos = ctz32(~z & 0xffff);
        assert(pos < 16);
        return buf + pos;
    } else {
        return NULL; // no match
    }
}

static really_inline
const u8 *fwdBlock(m128 mask_lo, m128 mask_hi, m128 chars, const u8 *buf,
                   const m128 low4bits, const m128 zeroes) {
    u32 z = block(mask_lo, mask_hi, chars, low4bits, zeroes);

    return firstMatch(buf, z);
}

const u8 *shuftiExec(m128 mask_lo, m128 mask_hi, const u8 *buf,
                     const u8 *buf_end) {
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
    rv = fwdBlock(mask_lo, mask_hi, chars, buf, low4bits, zeroes);
    if (rv) {
        return rv;
    }
    buf += (16 - min);

    // Unrolling was here, but it wasn't doing anything but taking up space.
    // Reroll FTW.

    const u8 *last_block = buf_end - 16;
    while (buf < last_block) {
        m128 lchars = load128(buf);
        rv = fwdBlock(mask_lo, mask_hi, lchars, buf, low4bits, zeroes);
        if (rv) {
            return rv;
        }
        buf += 16;
    }

    // Use an unaligned load to mop up the last 16 bytes and get an accurate
    // picture to buf_end.
    assert(buf <= buf_end && buf >= buf_end - 16);
    chars = loadu128(buf_end - 16);
    rv = fwdBlock(mask_lo, mask_hi, chars, buf_end - 16, low4bits, zeroes);
    if (rv) {
        return rv;
    }

    return buf_end;
}

static really_inline
const u8 *lastMatch(const u8 *buf, m128 t, m128 compare) {
#ifdef DEBUG
    DEBUG_PRINTF("confirming match in:"); dumpMsk128(t); printf("\n");
#endif

    u32 z = movemask128(eq128(t, compare));
    if (unlikely(z != 0xffff)) {
        u32 pos = clz32(~z & 0xffff);
        DEBUG_PRINTF("buf=%p, pos=%u\n", buf, pos);
        assert(pos >= 16 && pos < 32);
        return buf + (31 - pos);
    } else {
        return NULL; // no match
    }
}


static really_inline
const u8 *revBlock(m128 mask_lo, m128 mask_hi, m128 chars, const u8 *buf,
                   const m128 low4bits, const m128 zeroes) {
    m128 c_lo  = pshufb(mask_lo, GET_LO_4(chars));
    m128 c_hi  = pshufb(mask_hi, GET_HI_4(chars));
    m128 t     = and128(c_lo, c_hi);

#ifdef DEBUG
    DEBUG_PRINTF(" chars: "); dumpMsk128AsChars(chars); printf("\n");
    DEBUG_PRINTF("  char: "); dumpMsk128(chars);        printf("\n");
    DEBUG_PRINTF("  c_lo: "); dumpMsk128(c_lo);         printf("\n");
    DEBUG_PRINTF("  c_hi: "); dumpMsk128(c_hi);         printf("\n");
    DEBUG_PRINTF("     t: "); dumpMsk128(t);            printf("\n");
#endif

    return lastMatch(buf, t, zeroes);
}

const u8 *rshuftiExec(m128 mask_lo, m128 mask_hi, const u8 *buf,
                      const u8 *buf_end) {
    assert(buf && buf_end);
    assert(buf < buf_end);

    // Slow path for small cases.
    if (buf_end - buf < 16) {
        return shuftiRevSlow((const u8 *)&mask_lo, (const u8 *)&mask_hi,
                             buf, buf_end);
    }

    const m128 zeroes = zeroes128();
    const m128 low4bits = _mm_set1_epi8(0xf);
    const u8 *rv;

    assert(buf_end - buf >= 16);

    // Preconditioning: most of the time our buffer won't be aligned.
    m128 chars = loadu128(buf_end - 16);
    rv = revBlock(mask_lo, mask_hi, chars, buf_end - 16, low4bits, zeroes);
    if (rv) {
        return rv;
    }
    buf_end = (const u8 *)((size_t)buf_end & ~((size_t)0xf));

    // Unrolling was here, but it wasn't doing anything but taking up space.
    // Reroll FTW.

    const u8 *last_block = buf + 16;
    while (buf_end > last_block) {
        buf_end -= 16;
        m128 lchars = load128(buf_end);
        rv = revBlock(mask_lo, mask_hi, lchars, buf_end, low4bits, zeroes);
        if (rv) {
            return rv;
        }
    }

    // Use an unaligned load to mop up the last 16 bytes and get an accurate
    // picture to buf.
    chars = loadu128(buf);
    rv = revBlock(mask_lo, mask_hi, chars, buf, low4bits, zeroes);
    if (rv) {
        return rv;
    }

    return buf - 1;
}

static really_inline
const u8 *fwdBlock2(m128 mask1_lo, m128 mask1_hi, m128 mask2_lo, m128 mask2_hi,
                    m128 chars, const u8 *buf, const m128 low4bits,
                    const m128 ones) {
    m128 chars_lo = GET_LO_4(chars);
    m128 chars_hi = GET_HI_4(chars);
    m128 c_lo  = pshufb(mask1_lo, chars_lo);
    m128 c_hi  = pshufb(mask1_hi, chars_hi);
    m128 t     = or128(c_lo, c_hi);

#ifdef DEBUG
    DEBUG_PRINTF(" chars: "); dumpMsk128AsChars(chars); printf("\n");
    DEBUG_PRINTF("  char: "); dumpMsk128(chars);        printf("\n");
    DEBUG_PRINTF("  c_lo: "); dumpMsk128(c_lo);         printf("\n");
    DEBUG_PRINTF("  c_hi: "); dumpMsk128(c_hi);         printf("\n");
    DEBUG_PRINTF("     t: "); dumpMsk128(t);            printf("\n");
#endif

    m128 c2_lo  = pshufb(mask2_lo, chars_lo);
    m128 c2_hi  = pshufb(mask2_hi, chars_hi);
    m128 t2     = or128(t, shiftRight8Bits(or128(c2_lo, c2_hi)));

#ifdef DEBUG
    DEBUG_PRINTF(" c2_lo: "); dumpMsk128(c2_lo);        printf("\n");
    DEBUG_PRINTF(" c2_hi: "); dumpMsk128(c2_hi);        printf("\n");
    DEBUG_PRINTF("    t2: "); dumpMsk128(t2);           printf("\n");
#endif

    u32 z = movemask128(eq128(t2, ones));
    return firstMatch(buf, z);
}

const u8 *shuftiDoubleExec(m128 mask1_lo, m128 mask1_hi,
                           m128 mask2_lo, m128 mask2_hi,
                           const u8 *buf, const u8 *buf_end) {
    const m128 ones = ones128();
    const m128 low4bits = _mm_set1_epi8(0xf);
    const u8 *rv;

    size_t min = (size_t)buf % 16;

    // Preconditioning: most of the time our buffer won't be aligned.
    m128 chars = loadu128(buf);
    rv = fwdBlock2(mask1_lo, mask1_hi, mask2_lo, mask2_hi,
                   chars, buf, low4bits, ones);
    if (rv) {
        return rv;
    }
    buf += (16 - min);

    // Unrolling was here, but it wasn't doing anything but taking up space.
    // Reroll FTW.

    const u8 *last_block = buf_end - 16;
    while (buf < last_block) {
        m128 lchars = load128(buf);
        rv = fwdBlock2(mask1_lo, mask1_hi, mask2_lo, mask2_hi,
                       lchars, buf, low4bits, ones);
        if (rv) {
            return rv;
        }
        buf += 16;
    }

    // Use an unaligned load to mop up the last 16 bytes and get an accurate
    // picture to buf_end.
    chars = loadu128(buf_end - 16);
    rv = fwdBlock2(mask1_lo, mask1_hi, mask2_lo, mask2_hi,
                   chars, buf_end - 16, low4bits, ones);
    if (rv) {
        return rv;
    }

    return buf_end;
}

#else // AVX2 - 256 wide shuftis

static really_inline
const u8 *firstMatch(const u8 *buf, u32 z) {
    if (unlikely(z != 0xffffffff)) {
        u32 pos = ctz32(~z);
        assert(pos < 32);
        return buf + pos;
    } else {
        return NULL; // no match
    }
}

static really_inline
const u8 *fwdBlock(m256 mask_lo, m256 mask_hi, m256 chars, const u8 *buf,
                   const m256 low4bits, const m256 zeroes) {
    u32 z = block(mask_lo, mask_hi, chars, low4bits, zeroes);

    return firstMatch(buf, z);
}

/* takes 128 bit masks, but operates on 256 bits of data */
const u8 *shuftiExec(m128 mask_lo, m128 mask_hi, const u8 *buf,
                     const u8 *buf_end) {
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
    rv = fwdBlock(wide_mask_lo, wide_mask_hi, chars, buf, low4bits, zeroes);
    if (rv) {
        return rv;
    }
    buf += (32 - min);

    // Unrolling was here, but it wasn't doing anything but taking up space.
    // Reroll FTW.

    const u8 *last_block = buf_end - 32;
    while (buf < last_block) {
        m256 lchars = load256(buf);
        rv = fwdBlock(wide_mask_lo, wide_mask_hi, lchars, buf, low4bits, zeroes);
        if (rv) {
            return rv;
        }
        buf += 32;
    }

    // Use an unaligned load to mop up the last 32 bytes and get an accurate
    // picture to buf_end.
    assert(buf <= buf_end && buf >= buf_end - 32);
    chars = loadu256(buf_end - 32);
    rv = fwdBlock(wide_mask_lo, wide_mask_hi, chars, buf_end - 32, low4bits, zeroes);
    if (rv) {
        return rv;
    }

    return buf_end;
}

static really_inline
const u8 *lastMatch(const u8 *buf, m256 t, m256 compare) {
#ifdef DEBUG
    DEBUG_PRINTF("confirming match in:"); dumpMsk256(t); printf("\n");
#endif

    u32 z = movemask256(eq256(t, compare));
    if (unlikely(z != 0xffffffff)) {
        u32 pos = clz32(~z);
        DEBUG_PRINTF("buf=%p, pos=%u\n", buf, pos);
        return buf + (31 - pos);
    } else {
        return NULL; // no match
    }
}

static really_inline
const u8 *revBlock(m256 mask_lo, m256 mask_hi, m256 chars, const u8 *buf,
                   const m256 low4bits, const m256 zeroes) {
    m256 c_lo  = vpshufb(mask_lo, GET_LO_4(chars));
    m256 c_hi  = vpshufb(mask_hi, GET_HI_4(chars));
    m256 t     = and256(c_lo, c_hi);

#ifdef DEBUG
    DEBUG_PRINTF(" chars: "); dumpMsk256AsChars(chars); printf("\n");
    DEBUG_PRINTF("  char: "); dumpMsk256(chars);        printf("\n");
    DEBUG_PRINTF("  c_lo: "); dumpMsk256(c_lo);         printf("\n");
    DEBUG_PRINTF("  c_hi: "); dumpMsk256(c_hi);         printf("\n");
    DEBUG_PRINTF("     t: "); dumpMsk256(t);            printf("\n");
#endif

    return lastMatch(buf, t, zeroes);
}

/* takes 128 bit masks, but operates on 256 bits of data */
const u8 *rshuftiExec(m128 mask_lo, m128 mask_hi, const u8 *buf,
                      const u8 *buf_end) {
    assert(buf && buf_end);
    assert(buf < buf_end);

    // Slow path for small cases.
    if (buf_end - buf < 64) {
        return shuftiRevSlow((const u8 *)&mask_lo, (const u8 *)&mask_hi,
                             buf, buf_end);
    }

    const m256 zeroes = zeroes256();
    const m256 low4bits = set32x8(0xf);
    const m256 wide_mask_lo = set2x128(mask_lo);
    const m256 wide_mask_hi = set2x128(mask_hi);
    const u8 *rv;

    assert(buf_end - buf >= 32);

    // Preconditioning: most of the time our buffer won't be aligned.
    m256 chars = loadu256(buf_end - 32);
    rv = revBlock(wide_mask_lo, wide_mask_hi, chars, buf_end - 32, low4bits, zeroes);
    if (rv) {
        return rv;
    }
    buf_end = (const u8 *)((size_t)buf_end & ~((size_t)0x1f));

    // Unrolling was here, but it wasn't doing anything but taking up space.
    // Reroll FTW.
    const u8 *last_block = buf + 32;
    while (buf_end > last_block) {
        buf_end -= 32;
        m256 lchars = load256(buf_end);
        rv = revBlock(wide_mask_lo, wide_mask_hi, lchars, buf_end, low4bits, zeroes);
        if (rv) {
            return rv;
        }
    }

    // Use an unaligned load to mop up the last 32 bytes and get an accurate
    // picture to buf.
    chars = loadu256(buf);
    rv = revBlock(wide_mask_lo, wide_mask_hi, chars, buf, low4bits, zeroes);
    if (rv) {
        return rv;
    }

    return buf - 1;
}

static really_inline
const u8 *fwdBlock2(m256 mask1_lo, m256 mask1_hi, m256 mask2_lo, m256 mask2_hi,
                    m256 chars, const u8 *buf, const m256 low4bits,
                    const m256 ones) {
    DEBUG_PRINTF("buf %p\n", buf);
    m256 chars_lo = GET_LO_4(chars);
    m256 chars_hi = GET_HI_4(chars);
    m256 c_lo  = vpshufb(mask1_lo, chars_lo);
    m256 c_hi  = vpshufb(mask1_hi, chars_hi);
    m256 t     = or256(c_lo, c_hi);

#ifdef DEBUG
    DEBUG_PRINTF(" chars: "); dumpMsk256AsChars(chars); printf("\n");
    DEBUG_PRINTF("  char: "); dumpMsk256(chars);        printf("\n");
    DEBUG_PRINTF("  c_lo: "); dumpMsk256(c_lo);         printf("\n");
    DEBUG_PRINTF("  c_hi: "); dumpMsk256(c_hi);         printf("\n");
    DEBUG_PRINTF("     t: "); dumpMsk256(t);            printf("\n");
#endif

    m256 c2_lo  = vpshufb(mask2_lo, chars_lo);
    m256 c2_hi  = vpshufb(mask2_hi, chars_hi);
    m256 t2     = or256(t, shift256Right8Bits(or256(c2_lo, c2_hi)));

#ifdef DEBUG
    DEBUG_PRINTF(" c2_lo: "); dumpMsk256(c2_lo);        printf("\n");
    DEBUG_PRINTF(" c2_hi: "); dumpMsk256(c2_hi);        printf("\n");
    DEBUG_PRINTF("    t2: "); dumpMsk256(t2);           printf("\n");
#endif
    u32 z = movemask256(eq256(t2, ones));

    return firstMatch(buf, z);
}

/* takes 128 bit masks, but operates on 256 bits of data */
const u8 *shuftiDoubleExec(m128 mask1_lo, m128 mask1_hi,
                           m128 mask2_lo, m128 mask2_hi,
                           const u8 *buf, const u8 *buf_end) {
    if (buf_end - buf < 32) {
        // not worth it
        return buf;
    }
    const m256 ones = ones256();
    const m256 low4bits = set32x8(0xf);
    const m256 wide_mask1_lo = set2x128(mask1_lo);
    const m256 wide_mask1_hi = set2x128(mask1_hi);
    const m256 wide_mask2_lo = set2x128(mask2_lo);
    const m256 wide_mask2_hi = set2x128(mask2_hi);
    const u8 *rv;

    size_t min = (size_t)buf % 32;

    // Preconditioning: most of the time our buffer won't be aligned.
    m256 chars = loadu256(buf);
    rv = fwdBlock2(wide_mask1_lo, wide_mask1_hi, wide_mask2_lo, wide_mask2_hi,
                   chars, buf, low4bits, ones);
    if (rv) {
        return rv;
    }
    buf += (32 - min);

    // Unrolling was here, but it wasn't doing anything but taking up space.
    // Reroll FTW.
    const u8 *last_block = buf_end - 32;
    while (buf < last_block) {
        m256 lchars = load256(buf);
        rv = fwdBlock2(wide_mask1_lo, wide_mask1_hi, wide_mask2_lo, wide_mask2_hi,
                       lchars, buf, low4bits, ones);
        if (rv) {
            return rv;
        }
        buf += 32;
    }

    // Use an unaligned load to mop up the last 32 bytes and get an accurate
    // picture to buf_end.
    chars = loadu256(buf_end - 32);
    rv = fwdBlock2(wide_mask1_lo, wide_mask1_hi, wide_mask2_lo, wide_mask2_hi,
                   chars, buf_end - 32, low4bits, ones);
    if (rv) {
        return rv;
    }

    return buf_end;
}

#endif //AVX2
