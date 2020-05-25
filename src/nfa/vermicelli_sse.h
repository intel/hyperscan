/*
 * Copyright (c) 2015-2020, Intel Corporation
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
 * \brief Vermicelli: Intel SSE implementation.
 *
 * (users should include vermicelli.h)
 */

#if !defined(HAVE_AVX512)

#define VERM_BOUNDARY 16
#define VERM_TYPE m128
#define VERM_SET_FN set16x8

static really_inline
const u8 *vermSearchAligned(m128 chars, const u8 *buf, const u8 *buf_end,
                            char negate) {
    assert((size_t)buf % 16 == 0);
    for (; buf + 31 < buf_end; buf += 32) {
        m128 data = load128(buf);
        u32 z1 = movemask128(eq128(chars, data));
        m128 data2 = load128(buf + 16);
        u32 z2 = movemask128(eq128(chars, data2));
        u32 z = z1 | (z2 << 16);
        if (negate) {
            z = ~z;
        }
        if (unlikely(z)) {
            u32 pos = ctz32(z);
            return buf + pos;
        }
    }
    for (; buf + 15 < buf_end; buf += 16) {
        m128 data = load128(buf);
        u32 z = movemask128(eq128(chars, data));
        if (negate) {
            z = ~z & 0xffff;
        }
        if (unlikely(z)) {
            u32 pos = ctz32(z);
            return buf + pos;
        }
    }
    return NULL;
}

static really_inline
const u8 *vermSearchAlignedNocase(m128 chars, const u8 *buf,
                                  const u8 *buf_end, char negate) {
    assert((size_t)buf % 16 == 0);
    m128 casemask = set16x8(CASE_CLEAR);

    for (; buf + 31 < buf_end; buf += 32) {
        m128 data = load128(buf);
        u32 z1 = movemask128(eq128(chars, and128(casemask, data)));
        m128 data2 = load128(buf + 16);
        u32 z2 = movemask128(eq128(chars, and128(casemask, data2)));
        u32 z = z1 | (z2 << 16);
        if (negate) {
            z = ~z;
        }
        if (unlikely(z)) {
            u32 pos = ctz32(z);
            return buf + pos;
        }
    }

    for (; buf + 15 < buf_end; buf += 16) {
        m128 data = load128(buf);
        u32 z = movemask128(eq128(chars, and128(casemask, data)));
        if (negate) {
            z = ~z & 0xffff;
        }
        if (unlikely(z)) {
            u32 pos = ctz32(z);
            return buf + pos;
        }
    }
    return NULL;
}

// returns NULL if not found
static really_inline
const u8 *vermUnalign(m128 chars, const u8 *buf, char negate) {
    m128 data = loadu128(buf); // unaligned
    u32 z = movemask128(eq128(chars, data));
    if (negate) {
        z = ~z & 0xffff;
    }
    if (unlikely(z)) {
        return buf + ctz32(z);
    }
    return NULL;
}

// returns NULL if not found
static really_inline
const u8 *vermUnalignNocase(m128 chars, const u8 *buf, char negate) {
    m128 casemask = set16x8(CASE_CLEAR);
    m128 data = loadu128(buf); // unaligned
    u32 z = movemask128(eq128(chars, and128(casemask, data)));
    if (negate) {
        z = ~z & 0xffff;
    }
    if (unlikely(z)) {
        return buf + ctz32(z);
    }
    return NULL;
}

static really_inline
const u8 *dvermSearchAligned(m128 chars1, m128 chars2, u8 c1, u8 c2,
                             const u8 *buf, const u8 *buf_end) {
    for (; buf + 16 < buf_end; buf += 16) {
        m128 data = load128(buf);
        u32 z = movemask128(and128(eq128(chars1, data),
                                   rshiftbyte_m128(eq128(chars2, data), 1)));
        if (buf[15] == c1 && buf[16] == c2) {
            z |= (1 << 15);
        }
        if (unlikely(z)) {
            u32 pos = ctz32(z);
            return buf + pos;
        }
    }

    return NULL;
}

static really_inline
const u8 *dvermSearchAlignedNocase(m128 chars1, m128 chars2, u8 c1, u8 c2,
                                   const u8 *buf, const u8 *buf_end) {
    assert((size_t)buf % 16 == 0);
    m128 casemask = set16x8(CASE_CLEAR);

    for (; buf + 16 < buf_end; buf += 16) {
        m128 data = load128(buf);
        m128 v = and128(casemask, data);
        u32 z = movemask128(and128(eq128(chars1, v),
                                   rshiftbyte_m128(eq128(chars2, v), 1)));
        if ((buf[15] & CASE_CLEAR) == c1 && (buf[16] & CASE_CLEAR) == c2) {
            z |= (1 << 15);
        }
        if (unlikely(z)) {
            u32 pos = ctz32(z);
            return buf + pos;
        }
    }

    return NULL;
}

static really_inline
const u8 *dvermSearchAlignedMasked(m128 chars1, m128 chars2,
                                   m128 mask1, m128 mask2, u8 c1, u8 c2, u8 m1,
                                   u8 m2, const u8 *buf, const u8 *buf_end) {
    assert((size_t)buf % 16 == 0);

    for (; buf + 16 < buf_end; buf += 16) {
        m128 data = load128(buf);
        m128 v1 = eq128(chars1, and128(data, mask1));
        m128 v2 = eq128(chars2, and128(data, mask2));
        u32 z = movemask128(and128(v1, rshiftbyte_m128(v2, 1)));

        if ((buf[15] & m1) == c1 && (buf[16] & m2) == c2) {
            z |= (1 << 15);
        }
        if (unlikely(z)) {
            u32 pos = ctz32(z);
            return buf + pos;
        }
    }

    return NULL;
}

// returns NULL if not found
static really_inline
const u8 *dvermPrecondition(m128 chars1, m128 chars2, const u8 *buf) {
    m128 data = loadu128(buf); // unaligned
    u32 z = movemask128(and128(eq128(chars1, data),
                               rshiftbyte_m128(eq128(chars2, data), 1)));

    /* no fixup of the boundary required - the aligned run will pick it up */
    if (unlikely(z)) {
        u32 pos = ctz32(z);
        return buf + pos;
    }
    return NULL;
}

// returns NULL if not found
static really_inline
const u8 *dvermPreconditionNocase(m128 chars1, m128 chars2, const u8 *buf) {
    /* due to laziness, nonalphas and nocase having interesting behaviour */
    m128 casemask = set16x8(CASE_CLEAR);
    m128 data = loadu128(buf); // unaligned
    m128 v = and128(casemask, data);
    u32 z = movemask128(and128(eq128(chars1, v),
                               rshiftbyte_m128(eq128(chars2, v), 1)));

    /* no fixup of the boundary required - the aligned run will pick it up */
    if (unlikely(z)) {
        u32 pos = ctz32(z);
        return buf + pos;
    }
    return NULL;
}

// returns NULL if not found
static really_inline
const u8 *dvermPreconditionMasked(m128 chars1, m128 chars2,
                                  m128 mask1, m128 mask2, const u8 *buf) {
    m128 data = loadu128(buf); // unaligned
    m128 v1 = eq128(chars1, and128(data, mask1));
    m128 v2 = eq128(chars2, and128(data, mask2));
    u32 z = movemask128(and128(v1, rshiftbyte_m128(v2, 1)));

    /* no fixup of the boundary required - the aligned run will pick it up */
    if (unlikely(z)) {
        u32 pos = ctz32(z);
        return buf + pos;
    }
    return NULL;
}

static really_inline
const u8 *lastMatchOffset(const u8 *buf_end, u32 z) {
    assert(z);
    return buf_end - 16 + 31 - clz32(z);
}

static really_inline
const u8 *rvermSearchAligned(m128 chars, const u8 *buf, const u8 *buf_end,
                             char negate) {
    assert((size_t)buf_end % 16 == 0);
    for (; buf + 15 < buf_end; buf_end -= 16) {
        m128 data = load128(buf_end - 16);
        u32 z = movemask128(eq128(chars, data));
        if (negate) {
            z = ~z & 0xffff;
        }
        if (unlikely(z)) {
            return lastMatchOffset(buf_end, z);
        }
    }
    return NULL;
}

static really_inline
const u8 *rvermSearchAlignedNocase(m128 chars, const u8 *buf,
                                   const u8 *buf_end, char negate) {
    assert((size_t)buf_end % 16 == 0);
    m128 casemask = set16x8(CASE_CLEAR);

    for (; buf + 15 < buf_end; buf_end -= 16) {
        m128 data = load128(buf_end - 16);
        u32 z = movemask128(eq128(chars, and128(casemask, data)));
        if (negate) {
            z = ~z & 0xffff;
        }
        if (unlikely(z)) {
            return lastMatchOffset(buf_end, z);
        }
    }
    return NULL;
}

// returns NULL if not found
static really_inline
const u8 *rvermUnalign(m128 chars, const u8 *buf, char negate) {
    m128 data = loadu128(buf); // unaligned
    u32 z = movemask128(eq128(chars, data));
    if (negate) {
        z = ~z & 0xffff;
    }
    if (unlikely(z)) {
        return lastMatchOffset(buf + 16, z);
    }
    return NULL;
}

// returns NULL if not found
static really_inline
const u8 *rvermUnalignNocase(m128 chars, const u8 *buf, char negate) {
    m128 casemask = set16x8(CASE_CLEAR);
    m128 data = loadu128(buf); // unaligned
    u32 z = movemask128(eq128(chars, and128(casemask, data)));
    if (negate) {
        z = ~z & 0xffff;
    }
    if (unlikely(z)) {
        return lastMatchOffset(buf + 16, z);
    }
    return NULL;
}

static really_inline
const u8 *rdvermSearchAligned(m128 chars1, m128 chars2, u8 c1, u8 c2,
                              const u8 *buf, const u8 *buf_end) {
    assert((size_t)buf_end % 16 == 0);

    for (; buf + 16 < buf_end; buf_end -= 16) {
        m128 data = load128(buf_end - 16);
        u32 z = movemask128(and128(eq128(chars2, data),
                                   lshiftbyte_m128(eq128(chars1, data), 1)));
        if (buf_end[-17] == c1 && buf_end[-16] == c2) {
            z |= 1;
        }
        if (unlikely(z)) {
            return lastMatchOffset(buf_end, z);
        }
    }
    return buf_end;
}

static really_inline
const u8 *rdvermSearchAlignedNocase(m128 chars1, m128 chars2, u8 c1, u8 c2,
                                    const u8 *buf, const u8 *buf_end) {
    assert((size_t)buf_end % 16 == 0);
    m128 casemask = set16x8(CASE_CLEAR);

    for (; buf + 16 < buf_end; buf_end -= 16) {
        m128 data = load128(buf_end - 16);
        m128 v = and128(casemask, data);
        u32 z = movemask128(and128(eq128(chars2, v),
                                   lshiftbyte_m128(eq128(chars1, v), 1)));
        if ((buf_end[-17] & CASE_CLEAR) == c1
            && (buf_end[-16] & CASE_CLEAR) == c2) {
            z |= 1;
        }
        if (unlikely(z)) {
            return lastMatchOffset(buf_end, z);
        }
    }
    return buf_end;
}

// returns NULL if not found
static really_inline
const u8 *rdvermPrecondition(m128 chars1, m128 chars2, const u8 *buf) {
    m128 data = loadu128(buf);
    u32 z = movemask128(and128(eq128(chars2, data),
                               lshiftbyte_m128(eq128(chars1, data), 1)));

    /* no fixup of the boundary required - the aligned run will pick it up */
    if (unlikely(z)) {
        return lastMatchOffset(buf + 16, z);
    }

    return NULL;
}

// returns NULL if not found
static really_inline
const u8 *rdvermPreconditionNocase(m128 chars1, m128 chars2, const u8 *buf) {
    /* due to laziness, nonalphas and nocase having interesting behaviour */
    m128 casemask = set16x8(CASE_CLEAR);
    m128 data = loadu128(buf);
    m128 v = and128(casemask, data);
    u32 z = movemask128(and128(eq128(chars2, v),
                               lshiftbyte_m128(eq128(chars1, v), 1)));
    /* no fixup of the boundary required - the aligned run will pick it up */
    if (unlikely(z)) {
        return lastMatchOffset(buf + 16, z);
    }

    return NULL;
}

#else // HAVE_AVX512

#define VERM_BOUNDARY 64
#define VERM_TYPE m512
#define VERM_SET_FN set64x8

static really_inline
const u8 *vermMini(m512 chars, const u8 *buf, const u8 *buf_end, char negate) {
    uintptr_t len = buf_end - buf;
    __mmask64 mask = (~0ULL) >> (64 - len);
    m512 data = loadu_maskz_m512(mask, buf);

    u64a z = eq512mask(chars, data);

    if (negate) {
        z = ~z & mask;
    }
    z &= mask;
    if (unlikely(z)) {
        return buf + ctz64(z);
    }
    return NULL;
}

static really_inline
const u8 *vermMiniNocase(m512 chars, const u8 *buf, const u8 *buf_end,
                         char negate) {
    uintptr_t len = buf_end - buf;
    __mmask64 mask = (~0ULL) >> (64 - len);
    m512 data = loadu_maskz_m512(mask, buf);
    m512 casemask = set64x8(CASE_CLEAR);
    m512 v = and512(casemask, data);

    u64a z = eq512mask(chars, v);

    if (negate) {
        z = ~z & mask;
    }
    z &= mask;
    if (unlikely(z)) {
        return buf + ctz64(z);
    }
    return NULL;
}

static really_inline
const u8 *vermSearchAligned(m512 chars, const u8 *buf, const u8 *buf_end,
                            char negate) {
    assert((size_t)buf % 64 == 0);
    for (; buf + 63 < buf_end; buf += 64) {
        m512 data = load512(buf);
        u64a z = eq512mask(chars, data);
        if (negate) {
            z = ~z & ~0ULL;
        }
        if (unlikely(z)) {
            u64a pos = ctz64(z);
            return buf + pos;
        }
    }
    return NULL;
}

static really_inline
const u8 *vermSearchAlignedNocase(m512 chars, const u8 *buf,
                                  const u8 *buf_end, char negate) {
    assert((size_t)buf % 64 == 0);
    m512 casemask = set64x8(CASE_CLEAR);

    for (; buf + 63 < buf_end; buf += 64) {
        m512 data = load512(buf);
        u64a z = eq512mask(chars, and512(casemask, data));
        if (negate) {
            z = ~z & ~0ULL;
        }
        if (unlikely(z)) {
            u64a pos = ctz64(z);
            return buf + pos;
        }
    }
    return NULL;
}

// returns NULL if not found
static really_inline
const u8 *vermUnalign(m512 chars, const u8 *buf, char negate) {
    m512 data = loadu512(buf); // unaligned
    u64a z = eq512mask(chars, data);
    if (negate) {
        z = ~z & ~0ULL;
    }
    if (unlikely(z)) {
        return buf + ctz64(z);
    }
    return NULL;
}

// returns NULL if not found
static really_inline
const u8 *vermUnalignNocase(m512 chars, const u8 *buf, char negate) {
    m512 casemask = set64x8(CASE_CLEAR);
    m512 data = loadu512(buf); // unaligned
    u64a z = eq512mask(chars, and512(casemask, data));
    if (negate) {
        z = ~z & ~0ULL;
    }
    if (unlikely(z)) {
        return buf + ctz64(z);
    }
    return NULL;
}

static really_inline
const u8 *dvermMini(m512 chars1, m512 chars2, const u8 *buf,
                    const u8 *buf_end) {
    uintptr_t len = buf_end - buf;
    __mmask64 mask = (~0ULL) >> (64 - len);
    m512 data = loadu_maskz_m512(mask, buf);

    u64a z = eq512mask(chars1, data) & (eq512mask(chars2, data) >> 1);

    z &= mask;
    if (unlikely(z)) {
        u64a pos = ctz64(z);
        return buf + pos;
    }
    return NULL;
}

static really_inline
const u8 *dvermMiniNocase(m512 chars1, m512 chars2, const u8 *buf,
                          const u8 *buf_end) {
    uintptr_t len = buf_end - buf;
    __mmask64 mask = (~0ULL) >> (64 - len);
    m512 data = loadu_maskz_m512(mask, buf);
    m512 casemask = set64x8(CASE_CLEAR);
    m512 v = and512(casemask, data);

    u64a z = eq512mask(chars1, v) & (eq512mask(chars2, v) >> 1);

    z &= mask;
    if (unlikely(z)) {
        u64a pos = ctz64(z);
        return buf + pos;
    }
    return NULL;
}

static really_inline
const u8 *dvermMiniMasked(m512 chars1, m512 chars2, m512 mask1, m512 mask2,
                          const u8 *buf, const u8 *buf_end) {
    uintptr_t len = buf_end - buf;
    __mmask64 mask = (~0ULL) >> (64 - len);
    m512 data = loadu_maskz_m512(mask, buf);
    m512 v1 = and512(data, mask1);
    m512 v2 = and512(data, mask2);

    u64a z = eq512mask(chars1, v1) & (eq512mask(chars2, v2) >> 1);

    z &= mask;
    if (unlikely(z)) {
        u64a pos = ctz64(z);
        return buf + pos;
    }
    return NULL;
}

static really_inline
const u8 *dvermSearchAligned(m512 chars1, m512 chars2, u8 c1, u8 c2,
                             const u8 *buf, const u8 *buf_end) {
    for (; buf + 64 < buf_end; buf += 64) {
        m512 data = load512(buf);
        u64a z = eq512mask(chars1, data) & (eq512mask(chars2, data) >> 1);
        if (buf[63] == c1 && buf[64] == c2) {
            z |= (1ULL << 63);
        }
        if (unlikely(z)) {
            u64a pos = ctz64(z);
            return buf + pos;
        }
    }

    return NULL;
}

static really_inline
const u8 *dvermSearchAlignedNocase(m512 chars1, m512 chars2, u8 c1, u8 c2,
                                   const u8 *buf, const u8 *buf_end) {
    assert((size_t)buf % 64 == 0);
    m512 casemask = set64x8(CASE_CLEAR);

    for (; buf + 64 < buf_end; buf += 64) {
        m512 data = load512(buf);
        m512 v = and512(casemask, data);
        u64a z = eq512mask(chars1, v) & (eq512mask(chars2, v) >> 1);
        if ((buf[63] & CASE_CLEAR) == c1 && (buf[64] & CASE_CLEAR) == c2) {
            z |= (1ULL << 63);
        }
        if (unlikely(z)) {
            u64a pos = ctz64(z);
            return buf + pos;
        }
    }

    return NULL;
}

static really_inline
const u8 *dvermSearchAlignedMasked(m512 chars1, m512 chars2,
                                   m512 mask1, m512 mask2, u8 c1, u8 c2, u8 m1,
                                   u8 m2, const u8 *buf, const u8 *buf_end) {
    assert((size_t)buf % 64 == 0);

    for (; buf + 64 < buf_end; buf += 64) {
        m512 data = load512(buf);
        m512 v1 = and512(data, mask1);
        m512 v2 = and512(data, mask2);
        u64a z = eq512mask(chars1, v1) & (eq512mask(chars2, v2) >> 1);

        if ((buf[63] & m1) == c1 && (buf[64] & m2) == c2) {
            z |= (1ULL << 63);
        }
        if (unlikely(z)) {
            u64a pos = ctz64(z);
            return buf + pos;
        }
    }

    return NULL;
}

// returns NULL if not found
static really_inline
const u8 *dvermPrecondition(m512 chars1, m512 chars2, const u8 *buf) {
    m512 data = loadu512(buf); // unaligned
    u64a z = eq512mask(chars1, data) & (eq512mask(chars2, data) >> 1);

    /* no fixup of the boundary required - the aligned run will pick it up */
    if (unlikely(z)) {
        u64a pos = ctz64(z);
        return buf + pos;
    }
    return NULL;
}

// returns NULL if not found
static really_inline
const u8 *dvermPreconditionNocase(m512 chars1, m512 chars2, const u8 *buf) {
    /* due to laziness, nonalphas and nocase having interesting behaviour */
    m512 casemask = set64x8(CASE_CLEAR);
    m512 data = loadu512(buf); // unaligned
    m512 v = and512(casemask, data);
    u64a z = eq512mask(chars1, v) & (eq512mask(chars2, v) >> 1);

    /* no fixup of the boundary required - the aligned run will pick it up */
    if (unlikely(z)) {
        u64a pos = ctz64(z);
        return buf + pos;
    }
    return NULL;
}

// returns NULL if not found
static really_inline
const u8 *dvermPreconditionMasked(m512 chars1, m512 chars2,
                                  m512 mask1, m512 mask2, const u8 *buf) {
    m512 data = loadu512(buf); // unaligned
    m512 v1 = and512(data, mask1);
    m512 v2 = and512(data, mask2);
    u64a z = eq512mask(chars1, v1) & (eq512mask(chars2, v2) >> 1);

    /* no fixup of the boundary required - the aligned run will pick it up */
    if (unlikely(z)) {
        u64a pos = ctz64(z);
        return buf + pos;
    }
    return NULL;
}

static really_inline
const u8 *lastMatchOffset(const u8 *buf_end, u64a z) {
    assert(z);
    return buf_end - 64 + 63 - clz64(z);
}

static really_inline
const u8 *rvermMini(m512 chars, const u8 *buf, const u8 *buf_end, char negate) {
    uintptr_t len = buf_end - buf;
    __mmask64 mask = (~0ULL) >> (64 - len);
    m512 data = loadu_maskz_m512(mask, buf);

    u64a z = eq512mask(chars, data);

    if (negate) {
        z = ~z & mask;
    }
    z &= mask;
    if (unlikely(z)) {
        return lastMatchOffset(buf + 64, z);
    }
    return NULL;
}

static really_inline
const u8 *rvermMiniNocase(m512 chars, const u8 *buf, const u8 *buf_end,
                          char negate) {
    uintptr_t len = buf_end - buf;
    __mmask64 mask = (~0ULL) >> (64 - len);
    m512 data = loadu_maskz_m512(mask, buf);
    m512 casemask = set64x8(CASE_CLEAR);
    m512 v = and512(casemask, data);

    u64a z = eq512mask(chars, v);

    if (negate) {
        z = ~z & mask;
    }
    z &= mask;
    if (unlikely(z)) {
        return lastMatchOffset(buf + 64, z);
    }
    return NULL;
}

static really_inline
const u8 *rvermSearchAligned(m512 chars, const u8 *buf, const u8 *buf_end,
                             char negate) {
    assert((size_t)buf_end % 64 == 0);
    for (; buf + 63 < buf_end; buf_end -= 64) {
        m512 data = load512(buf_end - 64);
        u64a z = eq512mask(chars, data);
        if (negate) {
            z = ~z & ~0ULL;
        }
        if (unlikely(z)) {
            return lastMatchOffset(buf_end, z);
        }
    }
    return NULL;
}

static really_inline
const u8 *rvermSearchAlignedNocase(m512 chars, const u8 *buf,
                                   const u8 *buf_end, char negate) {
    assert((size_t)buf_end % 64 == 0);
    m512 casemask = set64x8(CASE_CLEAR);

    for (; buf + 63 < buf_end; buf_end -= 64) {
        m512 data = load512(buf_end - 64);
        u64a z = eq512mask(chars, and512(casemask, data));
        if (negate) {
            z = ~z & ~0ULL;
        }
        if (unlikely(z)) {
            return lastMatchOffset(buf_end, z);
        }
    }
    return NULL;
}

// returns NULL if not found
static really_inline
const u8 *rvermUnalign(m512 chars, const u8 *buf, char negate) {
    m512 data = loadu512(buf); // unaligned
    u64a z = eq512mask(chars, data);
    if (negate) {
        z = ~z & ~0ULL;
    }
    if (unlikely(z)) {
        return lastMatchOffset(buf + 64, z);
    }
    return NULL;
}

// returns NULL if not found
static really_inline
const u8 *rvermUnalignNocase(m512 chars, const u8 *buf, char negate) {
    m512 casemask = set64x8(CASE_CLEAR);
    m512 data = loadu512(buf); // unaligned
    u64a z = eq512mask(chars, and512(casemask, data));
    if (negate) {
        z = ~z & ~0ULL;
    }
    if (unlikely(z)) {
        return lastMatchOffset(buf + 64, z);
    }
    return NULL;
}

static really_inline
const u8 *rdvermMini(m512 chars1, m512 chars2, const u8 *buf,
                    const u8 *buf_end) {
    uintptr_t len = buf_end - buf;
    __mmask64 mask = (~0ULL) >> (64 - len);
    m512 data = loadu_maskz_m512(mask, buf);

    u64a z = eq512mask(chars2, data) & (eq512mask(chars1, data) << 1);

    z &= mask;
    if (unlikely(z)) {
        return lastMatchOffset(buf + 64, z);
    }
    return NULL;
}

static really_inline
const u8 *rdvermMiniNocase(m512 chars1, m512 chars2, const u8 *buf,
                           const u8 *buf_end) {
    uintptr_t len = buf_end - buf;
    __mmask64 mask = (~0ULL) >> (64 - len);
    m512 data = loadu_maskz_m512(mask, buf);
    m512 casemask = set64x8(CASE_CLEAR);
    m512 v = and512(casemask, data);

    u64a z = eq512mask(chars2, v) & (eq512mask(chars1, v) << 1);

    z &= mask;
    if (unlikely(z)) {
        return lastMatchOffset(buf + 64, z);
    }
    return NULL;
}

static really_inline
const u8 *rdvermSearchAligned(m512 chars1, m512 chars2, u8 c1, u8 c2,
                              const u8 *buf, const u8 *buf_end) {
    assert((size_t)buf_end % 64 == 0);

    for (; buf + 64 < buf_end; buf_end -= 64) {
        m512 data = load512(buf_end - 64);
        u64a z = eq512mask(chars2, data) & (eq512mask(chars1, data) << 1);
        if (buf_end[-65] == c1 && buf_end[-64] == c2) {
            z |= 1;
        }
        if (unlikely(z)) {
            return lastMatchOffset(buf_end, z);
        }
    }
    return buf_end;
}

static really_inline
const u8 *rdvermSearchAlignedNocase(m512 chars1, m512 chars2, u8 c1, u8 c2,
                                    const u8 *buf, const u8 *buf_end) {
    assert((size_t)buf_end % 64 == 0);
    m512 casemask = set64x8(CASE_CLEAR);

    for (; buf + 64 < buf_end; buf_end -= 64) {
        m512 data = load512(buf_end - 64);
        m512 v = and512(casemask, data);
        u64a z = eq512mask(chars2, v) & (eq512mask(chars1, v) << 1);
        if ((buf_end[-65] & CASE_CLEAR) == c1
            && (buf_end[-64] & CASE_CLEAR) == c2) {
            z |= 1;
        }
        if (unlikely(z)) {
            return lastMatchOffset(buf_end, z);
        }
    }
    return buf_end;
}

// returns NULL if not found
static really_inline
const u8 *rdvermPrecondition(m512 chars1, m512 chars2, const u8 *buf) {
    m512 data = loadu512(buf);
    u64a z = eq512mask(chars2, data) & (eq512mask(chars1, data) << 1);

    // no fixup of the boundary required - the aligned run will pick it up
    if (unlikely(z)) {
        return lastMatchOffset(buf + 64, z);
    }

    return NULL;
}

// returns NULL if not found
static really_inline
const u8 *rdvermPreconditionNocase(m512 chars1, m512 chars2, const u8 *buf) {
    // due to laziness, nonalphas and nocase having interesting behaviour
    m512 casemask = set64x8(CASE_CLEAR);
    m512 data = loadu512(buf);
    m512 v = and512(casemask, data);
    u64a z = eq512mask(chars2, v) & (eq512mask(chars1, v) << 1);
    // no fixup of the boundary required - the aligned run will pick it up
    if (unlikely(z)) {
        return lastMatchOffset(buf + 64, z);
    }

    return NULL;
}

#endif // HAVE_AVX512
