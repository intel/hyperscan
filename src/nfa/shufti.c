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
 * \brief Shufti: character class acceleration.
 *
 * Utilises the SSSE3 pshufb shuffle instruction
 */

#include "shufti.h"
#include "ue2common.h"
#include "util/arch.h"
#include "util/bitutils.h"
#include "util/simd_utils.h"
#include "util/unaligned.h"

#ifdef DEBUG
#include <ctype.h>

#define DUMP_MSK(_t)                                \
static UNUSED                                       \
void dumpMsk##_t(m##_t msk) {                       \
    u8 * mskAsU8 = (u8 *)&msk;                      \
    for (unsigned i = 0; i < sizeof(msk); i++) {    \
        u8 c = mskAsU8[i];                          \
        for (int j = 0; j < 8; j++) {               \
            if ((c >> (7-j)) & 0x1)                 \
                printf("1");                        \
            else                                    \
                printf("0");                        \
        }                                           \
        printf(" ");                                \
    }                                               \
}                                                   \
static UNUSED                                       \
void dumpMsk##_t##AsChars(m##_t msk) {              \
    u8 * mskAsU8 = (u8 *)&msk;                      \
    for (unsigned i = 0; i < sizeof(msk); i++) {    \
        u8 c = mskAsU8[i];                          \
        if (isprint(c))                             \
            printf("%c",c);                         \
        else                                        \
            printf(".");                            \
    }                                               \
}

#endif

/** \brief Naive byte-by-byte implementation. */
static really_inline
const u8 *shuftiFwdSlow(const u8 *lo, const u8 *hi, const u8 *buf,
                        const u8 *buf_end) {
    assert(buf < buf_end);

    for (; buf < buf_end; ++buf) {
        u8 c = *buf;
        if (lo[c & 0xf] & hi[c >> 4]) {
            break;
        }
    }
    return buf;
}

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

#if !defined(HAVE_AVX2)
/* Normal SSSE3 shufti */

#ifdef DEBUG
DUMP_MSK(128)
#endif

#define GET_LO_4(chars) and128(chars, low4bits)
#define GET_HI_4(chars) rshift64_m128(andnot128(low4bits, chars), 4)

static really_inline
u32 block(m128 mask_lo, m128 mask_hi, m128 chars, const m128 low4bits,
          const m128 compare) {
    m128 c_lo  = pshufb_m128(mask_lo, GET_LO_4(chars));
    m128 c_hi  = pshufb_m128(mask_hi, GET_HI_4(chars));
    m128 t     = and128(c_lo, c_hi);

#ifdef DEBUG
    DEBUG_PRINTF(" chars: "); dumpMsk128AsChars(chars); printf("\n");
    DEBUG_PRINTF("  char: "); dumpMsk128(chars);        printf("\n");
    DEBUG_PRINTF("  c_lo: "); dumpMsk128(c_lo);         printf("\n");
    DEBUG_PRINTF("  c_hi: "); dumpMsk128(c_hi);         printf("\n");
    DEBUG_PRINTF("     t: "); dumpMsk128(t);            printf("\n");
#endif
    return movemask128(eq128(t, compare));
}

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
    m128 c_lo  = pshufb_m128(mask_lo, GET_LO_4(chars));
    m128 c_hi  = pshufb_m128(mask_hi, GET_HI_4(chars));
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
    m128 c_lo  = pshufb_m128(mask1_lo, chars_lo);
    m128 c_hi  = pshufb_m128(mask1_hi, chars_hi);
    m128 t     = or128(c_lo, c_hi);

#ifdef DEBUG
    DEBUG_PRINTF(" chars: "); dumpMsk128AsChars(chars); printf("\n");
    DEBUG_PRINTF("  char: "); dumpMsk128(chars);        printf("\n");
    DEBUG_PRINTF("  c_lo: "); dumpMsk128(c_lo);         printf("\n");
    DEBUG_PRINTF("  c_hi: "); dumpMsk128(c_hi);         printf("\n");
    DEBUG_PRINTF("     t: "); dumpMsk128(t);            printf("\n");
#endif

    m128 c2_lo  = pshufb_m128(mask2_lo, chars_lo);
    m128 c2_hi  = pshufb_m128(mask2_hi, chars_hi);
    m128 t2     = or128(t, rshiftbyte_m128(or128(c2_lo, c2_hi), 1));

#ifdef DEBUG
    DEBUG_PRINTF(" c2_lo: "); dumpMsk128(c2_lo);        printf("\n");
    DEBUG_PRINTF(" c2_hi: "); dumpMsk128(c2_hi);        printf("\n");
    DEBUG_PRINTF("    t2: "); dumpMsk128(t2);           printf("\n");
#endif

    u32 z = movemask128(eq128(t2, ones));
    DEBUG_PRINTF("    z: 0x%08x\n", z);
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

#elif !defined(HAVE_AVX512)
// AVX2 - 256 wide shuftis

#ifdef DEBUG
DUMP_MSK(256)
#endif

#define GET_LO_4(chars) and256(chars, low4bits)
#define GET_HI_4(chars) rshift64_m256(andnot256(low4bits, chars), 4)

static really_inline
u32 block(m256 mask_lo, m256 mask_hi, m256 chars, const m256 low4bits,
          const m256 compare) {
    m256 c_lo  = pshufb_m256(mask_lo, GET_LO_4(chars));
    m256 c_hi  = pshufb_m256(mask_hi, GET_HI_4(chars));
    m256 t = and256(c_lo, c_hi);

#ifdef DEBUG
    DEBUG_PRINTF(" chars: "); dumpMsk256AsChars(chars); printf("\n");
    DEBUG_PRINTF("  char: "); dumpMsk256(chars); printf("\n");
    DEBUG_PRINTF("  c_lo: "); dumpMsk256(c_lo); printf("\n");
    DEBUG_PRINTF("  c_hi: "); dumpMsk256(c_hi); printf("\n");
    DEBUG_PRINTF("     t: "); dumpMsk256(t); printf("\n");
#endif

    return movemask256(eq256(t, compare));
}

static really_inline
const u8 *firstMatch(const u8 *buf, u32 z) {
    DEBUG_PRINTF("z 0x%08x\n", z);
    if (unlikely(z != 0xffffffff)) {
        u32 pos = ctz32(~z);
        assert(pos < 32);
        DEBUG_PRINTF("match @ pos %u\n", pos);
        return buf + pos;
    } else {
        return NULL; // no match
    }
}

static really_inline
const u8 *fwdBlockShort(m256 mask, m128 chars, const u8 *buf,
                        const m256 low4bits) {
    // do the hi and lo shuffles in the one avx register
    m256 c = combine2x128(rshift64_m128(chars, 4), chars);
    c = and256(c, low4bits);
    m256 c_shuf = pshufb_m256(mask, c);
    m128 t = and128(movdq_hi(c_shuf), cast256to128(c_shuf));
    // the upper 32-bits can't match
    u32 z = 0xffff0000U | movemask128(eq128(t, zeroes128()));

    return firstMatch(buf, z);
}

static really_inline
const u8 *shuftiFwdShort(m128 mask_lo, m128 mask_hi, const u8 *buf,
                         const u8 *buf_end, const m256 low4bits) {
    // run shufti over two overlapping 16-byte unaligned reads
    const m256 mask = combine2x128(mask_hi, mask_lo);
    m128 chars = loadu128(buf);
    const u8 *rv = fwdBlockShort(mask, chars, buf, low4bits);
    if (rv) {
        return rv;
    }

    chars = loadu128(buf_end - 16);
    rv = fwdBlockShort(mask, chars, buf_end - 16, low4bits);
    if (rv) {
        return rv;
    }
    return buf_end;
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
    DEBUG_PRINTF("shufti %p len %zu\n", buf, buf_end - buf);

    // Slow path for small cases.
    if (buf_end - buf < 16) {
        return shuftiFwdSlow((const u8 *)&mask_lo, (const u8 *)&mask_hi,
                             buf, buf_end);
    }

    const m256 low4bits = set32x8(0xf);

    if (buf_end - buf <= 32) {
        return shuftiFwdShort(mask_lo, mask_hi, buf, buf_end, low4bits);
    }

    const m256 zeroes = zeroes256();
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
const u8 *lastMatch(const u8 *buf, u32 z) {
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
    m256 c_lo  = pshufb_m256(mask_lo, GET_LO_4(chars));
    m256 c_hi  = pshufb_m256(mask_hi, GET_HI_4(chars));
    m256 t     = and256(c_lo, c_hi);

#ifdef DEBUG
    DEBUG_PRINTF(" chars: "); dumpMsk256AsChars(chars); printf("\n");
    DEBUG_PRINTF("  char: "); dumpMsk256(chars);        printf("\n");
    DEBUG_PRINTF("  c_lo: "); dumpMsk256(c_lo);         printf("\n");
    DEBUG_PRINTF("  c_hi: "); dumpMsk256(c_hi);         printf("\n");
    DEBUG_PRINTF("     t: "); dumpMsk256(t);            printf("\n");
#endif

    u32 z = movemask256(eq256(t, zeroes));
    return lastMatch(buf, z);
}

static really_inline
const u8 *revBlockShort(m256 mask, m128 chars, const u8 *buf,
                        const m256 low4bits) {
    // do the hi and lo shuffles in the one avx register
    m256 c = combine2x128(rshift64_m128(chars, 4), chars);
    c = and256(c, low4bits);
    m256 c_shuf = pshufb_m256(mask, c);
    m128 t = and128(movdq_hi(c_shuf), cast256to128(c_shuf));
    // the upper 32-bits can't match
    u32 z = 0xffff0000U | movemask128(eq128(t, zeroes128()));

    return lastMatch(buf, z);
}

static really_inline
const u8 *shuftiRevShort(m128 mask_lo, m128 mask_hi, const u8 *buf,
                         const u8 *buf_end, const m256 low4bits) {
    // run shufti over two overlapping 16-byte unaligned reads
    const m256 mask = combine2x128(mask_hi, mask_lo);

    m128 chars = loadu128(buf_end - 16);
    const u8 *rv = revBlockShort(mask, chars, buf_end - 16, low4bits);
    if (rv) {
        return rv;
    }

    chars = loadu128(buf);
    rv = revBlockShort(mask, chars, buf, low4bits);
    if (rv) {
        return rv;
    }
    return buf - 1;
}


/* takes 128 bit masks, but operates on 256 bits of data */
const u8 *rshuftiExec(m128 mask_lo, m128 mask_hi, const u8 *buf,
                      const u8 *buf_end) {
    assert(buf && buf_end);
    assert(buf < buf_end);

    // Slow path for small cases.
    if (buf_end - buf < 16) {
        return shuftiRevSlow((const u8 *)&mask_lo, (const u8 *)&mask_hi,
                             buf, buf_end);
    }

    const m256 low4bits = set32x8(0xf);

    if (buf_end - buf <= 32) {
        return shuftiRevShort(mask_lo, mask_hi, buf, buf_end, low4bits);
    }

    const m256 zeroes = zeroes256();
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
    m256 c_lo  = pshufb_m256(mask1_lo, chars_lo);
    m256 c_hi  = pshufb_m256(mask1_hi, chars_hi);
    m256 t     = or256(c_lo, c_hi);

#ifdef DEBUG
    DEBUG_PRINTF(" chars: "); dumpMsk256AsChars(chars); printf("\n");
    DEBUG_PRINTF("  char: "); dumpMsk256(chars);        printf("\n");
    DEBUG_PRINTF("  c_lo: "); dumpMsk256(c_lo);         printf("\n");
    DEBUG_PRINTF("  c_hi: "); dumpMsk256(c_hi);         printf("\n");
    DEBUG_PRINTF("     t: "); dumpMsk256(t);            printf("\n");
#endif

    m256 c2_lo  = pshufb_m256(mask2_lo, chars_lo);
    m256 c2_hi  = pshufb_m256(mask2_hi, chars_hi);
    m256 t2 = or256(t, rshift128_m256(or256(c2_lo, c2_hi), 1));

#ifdef DEBUG
    DEBUG_PRINTF(" c2_lo: "); dumpMsk256(c2_lo);        printf("\n");
    DEBUG_PRINTF(" c2_hi: "); dumpMsk256(c2_hi);        printf("\n");
    DEBUG_PRINTF("    t2: "); dumpMsk256(t2);           printf("\n");
#endif
    u32 z = movemask256(eq256(t2, ones));

    return firstMatch(buf, z);
}

static really_inline
const u8 *fwdBlockShort2(m256 mask1, m256 mask2, m128 chars, const u8 *buf,
                         const m256 low4bits) {
    // do the hi and lo shuffles in the one avx register
    m256 c = combine2x128(rshift64_m128(chars, 4), chars);
    c = and256(c, low4bits);
    m256 c_shuf1 = pshufb_m256(mask1, c);
    m256 c_shuf2 = rshift128_m256(pshufb_m256(mask2, c), 1);
    m256 t0 = or256(c_shuf1, c_shuf2);
    m128 t = or128(movdq_hi(t0), cast256to128(t0));
    // the upper 32-bits can't match
    u32 z = 0xffff0000U | movemask128(eq128(t, ones128()));

    return firstMatch(buf, z);
}

static really_inline
const u8 *shuftiDoubleShort(m128 mask1_lo, m128 mask1_hi, m128 mask2_lo,
                            m128 mask2_hi, const u8 *buf, const u8 *buf_end) {
    DEBUG_PRINTF("buf %p len %zu\n", buf, buf_end - buf);
    const m256 low4bits = set32x8(0xf);
    // run shufti over two overlapping 16-byte unaligned reads
    const m256 mask1 = combine2x128(mask1_hi, mask1_lo);
    const m256 mask2 = combine2x128(mask2_hi, mask2_lo);
    m128 chars = loadu128(buf);
    const u8 *rv = fwdBlockShort2(mask1, mask2, chars, buf, low4bits);
    if (rv) {
        return rv;
    }

    chars = loadu128(buf_end - 16);
    rv = fwdBlockShort2(mask1, mask2, chars, buf_end - 16, low4bits);
    if (rv) {
        return rv;
    }
    return buf_end;
}

/* takes 128 bit masks, but operates on 256 bits of data */
const u8 *shuftiDoubleExec(m128 mask1_lo, m128 mask1_hi,
                           m128 mask2_lo, m128 mask2_hi,
                           const u8 *buf, const u8 *buf_end) {
    /* we should always have at least 16 bytes */
    assert(buf_end - buf >= 16);
    DEBUG_PRINTF("buf %p len %zu\n", buf, buf_end - buf);

    if (buf_end - buf < 32) {
        return shuftiDoubleShort(mask1_lo, mask1_hi, mask2_lo, mask2_hi, buf,
                                 buf_end);
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

#else // defined(HAVE_AVX512)

#ifdef DEBUG
DUMP_MSK(512)
#endif

static really_inline
u64a block(m512 mask_lo, m512 mask_hi, m512 chars, const m512 low4bits,
           const m512 compare) {
    m512 c_lo = pshufb_m512(mask_lo, and512(chars, low4bits));
    m512 c_hi = pshufb_m512(mask_hi,
                            rshift64_m512(andnot512(low4bits, chars), 4));
    m512 t = and512(c_lo, c_hi);

#ifdef DEBUG
    DEBUG_PRINTF(" chars: "); dumpMsk512AsChars(chars); printf("\n");
    DEBUG_PRINTF("  char: "); dumpMsk512(chars); printf("\n");
    DEBUG_PRINTF("  c_lo: "); dumpMsk512(c_lo); printf("\n");
    DEBUG_PRINTF("  c_hi: "); dumpMsk512(c_hi); printf("\n");
    DEBUG_PRINTF("     t: "); dumpMsk512(t); printf("\n");
#endif

    return eq512mask(t, compare);
}
static really_inline
const u8 *firstMatch64(const u8 *buf, u64a z) {
    DEBUG_PRINTF("z 0x%016llx\n", z);
    if (unlikely(z != ~0ULL)) {
        u32 pos = ctz64(~z);
        DEBUG_PRINTF("match @ pos %u\n", pos);
        assert(pos < 64);
        return buf + pos;
    } else {
        return NULL; // no match
    }
}

static really_inline
const u8 *fwdBlock512(m512 mask_lo, m512 mask_hi, m512 chars, const u8 *buf,
                      const m512 low4bits, const m512 zeroes) {
    u64a z = block(mask_lo, mask_hi, chars, low4bits, zeroes);

    return firstMatch64(buf, z);
}

static really_inline
const u8 *shortShufti512(m512 mask_lo, m512 mask_hi, const u8 *buf,
                         const u8 *buf_end, const m512 low4bits,
                         const m512 zeroes) {
    DEBUG_PRINTF("short shufti %p len %zu\n", buf, buf_end - buf);
    uintptr_t len = buf_end - buf;
    assert(len <= 64);

    // load mask
    u64a k = (~0ULL) >> (64 - len);
    DEBUG_PRINTF("load mask 0x%016llx\n", k);

    m512 chars = loadu_maskz_m512(k, buf);

    u64a z = block(mask_lo, mask_hi, chars, low4bits, zeroes);

    // reuse the load mask to indicate valid bytes
    return firstMatch64(buf, z | ~k);
}

/* takes 128 bit masks, but operates on 512 bits of data */
const u8 *shuftiExec(m128 mask_lo, m128 mask_hi, const u8 *buf,
                     const u8 *buf_end) {
    assert(buf && buf_end);
    assert(buf < buf_end);
    DEBUG_PRINTF("shufti %p len %zu\n", buf, buf_end - buf);
    DEBUG_PRINTF("b %s\n", buf);

    const m512 low4bits = set64x8(0xf);
    const m512 zeroes = zeroes512();
    const m512 wide_mask_lo = set4x128(mask_lo);
    const m512 wide_mask_hi = set4x128(mask_hi);
    const u8 *rv;

    // small cases.
    if (buf_end - buf <= 64) {
        rv = shortShufti512(wide_mask_lo, wide_mask_hi, buf, buf_end, low4bits,
                            zeroes);
        return rv ? rv : buf_end;
    }

    assert(buf_end - buf >= 64);

    // Preconditioning: most of the time our buffer won't be aligned.
    if ((uintptr_t)buf % 64) {
        rv = shortShufti512(wide_mask_lo, wide_mask_hi, buf,
                            ROUNDUP_PTR(buf, 64), low4bits, zeroes);
        if (rv) {
            return rv;
        }
        buf = ROUNDUP_PTR(buf, 64);
    }

    const u8 *last_block = ROUNDDOWN_PTR(buf_end, 64);
    while (buf < last_block) {
        m512 lchars = load512(buf);
        rv = fwdBlock512(wide_mask_lo, wide_mask_hi, lchars, buf, low4bits,
                         zeroes);
        if (rv) {
            return rv;
        }
        buf += 64;
    }

    if (buf == buf_end) {
        goto done;
    }

    // Use an unaligned load to mop up the last 64 bytes and get an accurate
    // picture to buf_end.
    assert(buf <= buf_end && buf >= buf_end - 64);
    m512 chars = loadu512(buf_end - 64);
    rv = fwdBlock512(wide_mask_lo, wide_mask_hi, chars, buf_end - 64, low4bits,
                     zeroes);
    if (rv) {
        return rv;
    }
done:
    return buf_end;
}

static really_inline
const u8 *lastMatch64(const u8 *buf, u64a z) {
    DEBUG_PRINTF("z 0x%016llx\n", z);
    if (unlikely(z != ~0ULL)) {
        u32 pos = clz64(~z);
        DEBUG_PRINTF("buf=%p, pos=%u\n", buf, pos);
        return buf + (63 - pos);
    } else {
        return NULL; // no match
    }
}

static really_inline
const u8 *rshortShufti512(m512 mask_lo, m512 mask_hi, const u8 *buf,
                          const u8 *buf_end, const m512 low4bits,
                          const m512 zeroes) {
    DEBUG_PRINTF("short %p len %zu\n", buf, buf_end - buf);
    uintptr_t len = buf_end - buf;
    assert(len <= 64);

    // load mask
    u64a k = (~0ULL) >> (64 - len);
    DEBUG_PRINTF("load mask 0x%016llx\n", k);

    m512 chars = loadu_maskz_m512(k, buf);

    u64a z = block(mask_lo, mask_hi, chars, low4bits, zeroes);

    // reuse the load mask to indicate valid bytes
    return lastMatch64(buf, z | ~k);
}

static really_inline
const u8 *revBlock512(m512 mask_lo, m512 mask_hi, m512 chars, const u8 *buf,
                      const m512 low4bits, const m512 zeroes) {
    m512 c_lo  = pshufb_m512(mask_lo, and512(chars, low4bits));
    m512 c_hi  = pshufb_m512(mask_hi,
                             rshift64_m512(andnot512(low4bits, chars), 4));
    m512 t     = and512(c_lo, c_hi);

#ifdef DEBUG
    DEBUG_PRINTF(" chars: "); dumpMsk512AsChars(chars); printf("\n");
    DEBUG_PRINTF("  char: "); dumpMsk512(chars);        printf("\n");
    DEBUG_PRINTF("  c_lo: "); dumpMsk512(c_lo);         printf("\n");
    DEBUG_PRINTF("  c_hi: "); dumpMsk512(c_hi);         printf("\n");
    DEBUG_PRINTF("     t: "); dumpMsk512(t);            printf("\n");
#endif

    u64a z = eq512mask(t, zeroes);
    return lastMatch64(buf, z);
}

/* takes 128 bit masks, but operates on 512 bits of data */
const u8 *rshuftiExec(m128 mask_lo, m128 mask_hi, const u8 *buf,
                      const u8 *buf_end) {
    DEBUG_PRINTF("buf %p buf_end %p\n", buf, buf_end);
    assert(buf && buf_end);
    assert(buf < buf_end);

    const m512 low4bits = set64x8(0xf);
    const m512 zeroes = zeroes512();
    const m512 wide_mask_lo = set4x128(mask_lo);
    const m512 wide_mask_hi = set4x128(mask_hi);
    const u8 *rv;

    if (buf_end - buf < 64) {
        rv = rshortShufti512(wide_mask_lo, wide_mask_hi, buf, buf_end, low4bits,
                             zeroes);
        return rv ? rv : buf - 1;
    }

    if (ROUNDDOWN_PTR(buf_end, 64) != buf_end) {
        // peel off unaligned portion
        assert(buf_end - buf >= 64);
        DEBUG_PRINTF("start\n");
        rv = rshortShufti512(wide_mask_lo, wide_mask_hi,
                             ROUNDDOWN_PTR(buf_end, 64), buf_end, low4bits,
                             zeroes);
        if (rv) {
            return rv;
        }
        buf_end = ROUNDDOWN_PTR(buf_end, 64);
    }

    const u8 *last_block = ROUNDUP_PTR(buf, 64);
    while (buf_end > last_block) {
        buf_end -= 64;
        m512 lchars = load512(buf_end);
        rv = revBlock512(wide_mask_lo, wide_mask_hi, lchars, buf_end, low4bits,
                         zeroes);
        if (rv) {
            return rv;
        }
    }
    if (buf_end == buf) {
        goto done;
    }
    // Use an unaligned load to mop up the last 64 bytes and get an accurate
    // picture to buf.
    m512 chars = loadu512(buf);
    rv = revBlock512(wide_mask_lo, wide_mask_hi, chars, buf, low4bits, zeroes);
    if (rv) {
        return rv;
    }
done:
    return buf - 1;
}

static really_inline
const u8 *fwdBlock2(m512 mask1_lo, m512 mask1_hi, m512 mask2_lo, m512 mask2_hi,
                    m512 chars, const u8 *buf, const m512 low4bits,
                    const m512 ones, __mmask64 k) {
    DEBUG_PRINTF("buf %p %.64s\n", buf, buf);
    m512 chars_lo = and512(chars, low4bits);
    m512 chars_hi = rshift64_m512(andnot512(low4bits, chars), 4);
    m512 c_lo  = maskz_pshufb_m512(k, mask1_lo, chars_lo);
    m512 c_hi  = maskz_pshufb_m512(k, mask1_hi, chars_hi);
    m512 t     = or512(c_lo, c_hi);

#ifdef DEBUG
    DEBUG_PRINTF(" chars: "); dumpMsk512AsChars(chars); printf("\n");
    DEBUG_PRINTF("  char: "); dumpMsk512(chars);        printf("\n");
    DEBUG_PRINTF("  c_lo: "); dumpMsk512(c_lo);         printf("\n");
    DEBUG_PRINTF("  c_hi: "); dumpMsk512(c_hi);         printf("\n");
    DEBUG_PRINTF("     t: "); dumpMsk512(t);            printf("\n");
#endif

    m512 c2_lo  = maskz_pshufb_m512(k, mask2_lo, chars_lo);
    m512 c2_hi  = maskz_pshufb_m512(k, mask2_hi, chars_hi);
    m512 t2 = or512(t, rshift128_m512(or512(c2_lo, c2_hi), 1));

#ifdef DEBUG
    DEBUG_PRINTF(" c2_lo: "); dumpMsk512(c2_lo);        printf("\n");
    DEBUG_PRINTF(" c2_hi: "); dumpMsk512(c2_hi);        printf("\n");
    DEBUG_PRINTF("    t2: "); dumpMsk512(t2);           printf("\n");
#endif
    u64a z = eq512mask(t2, ones);

    return firstMatch64(buf, z | ~k);
}

static really_inline
const u8 *shortDoubleShufti512(m512 mask1_lo, m512 mask1_hi, m512 mask2_lo,
                               m512 mask2_hi, const u8 *buf, const u8 *buf_end,
                               const m512 low4bits, const m512 ones) {
    DEBUG_PRINTF("short %p len %zu\n", buf, buf_end - buf);
    uintptr_t len = buf_end - buf;
    assert(len <= 64);

    u64a k = (~0ULL) >> (64 - len);
    DEBUG_PRINTF("load mask 0x%016llx\n", k);

    m512 chars = loadu_mask_m512(ones, k, buf);

    const u8 *rv = fwdBlock2(mask1_lo, mask1_hi, mask2_lo, mask2_hi, chars, buf,
                             low4bits, ones, k);

    return rv;
}

/* takes 128 bit masks, but operates on 512 bits of data */
const u8 *shuftiDoubleExec(m128 mask1_lo, m128 mask1_hi,
                           m128 mask2_lo, m128 mask2_hi,
                           const u8 *buf, const u8 *buf_end) {
    /* we should always have at least 16 bytes */
    assert(buf_end - buf >= 16);
    DEBUG_PRINTF("buf %p len %zu\n", buf, buf_end - buf);

    const m512 ones = ones512();
    const m512 low4bits = set64x8(0xf);
    const m512 wide_mask1_lo = set4x128(mask1_lo);
    const m512 wide_mask1_hi = set4x128(mask1_hi);
    const m512 wide_mask2_lo = set4x128(mask2_lo);
    const m512 wide_mask2_hi = set4x128(mask2_hi);
    const u8 *rv;

    if (buf_end - buf <= 64) {
        rv = shortDoubleShufti512(wide_mask1_lo, wide_mask1_hi, wide_mask2_lo,
                                  wide_mask2_hi, buf, buf_end, low4bits, ones);
        DEBUG_PRINTF("rv %p\n", rv);
        return rv ? rv : buf_end;
    }

    // Preconditioning: most of the time our buffer won't be aligned.
    if ((uintptr_t)buf % 64) {
        rv = shortDoubleShufti512(wide_mask1_lo, wide_mask1_hi, wide_mask2_lo,
                                  wide_mask2_hi, buf, ROUNDUP_PTR(buf, 64),
                                  low4bits, ones);
        if (rv) {
            return rv;
        }

        buf = ROUNDUP_PTR(buf, 64);
    }

    const u8 *last_block = buf_end - 64;
    while (buf < last_block) {
        m512 lchars = load512(buf);
        rv = fwdBlock2(wide_mask1_lo, wide_mask1_hi, wide_mask2_lo,
                       wide_mask2_hi, lchars, buf, low4bits, ones, ~0);
        if (rv) {
            return rv;
        }
        buf += 64;
    }

    // Use an unaligned load to mop up the last 64 bytes and get an accurate
    // picture to buf_end.
    m512 chars = loadu512(buf_end - 64);
    rv = fwdBlock2(wide_mask1_lo, wide_mask1_hi, wide_mask2_lo, wide_mask2_hi,
                   chars, buf_end - 64, low4bits, ones, ~0);
    if (rv) {
        return rv;
    }

    return buf_end;
}
#endif
