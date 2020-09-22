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
 * \brief Bit-twiddling primitives (ctz, compress etc)
 */

#ifndef BITUTILS_ARCH_COMMON_H
#define BITUTILS_ARCH_COMMON_H

#include "util/popcount.h"
#include "util/unaligned.h"

static really_inline
u32 clz32_impl_c(u32 x) {
    return (u32)__builtin_clz(x);
}

static really_inline
u32 clz64_impl_c(u64a x) {
    return (u32)__builtin_clzll(x);
}

// CTZ (count trailing zero) implementations.
static really_inline
u32 ctz32_impl_c(u32 x) {
    return (u32)__builtin_ctz(x);
}

static really_inline
u32 ctz64_impl_c(u64a x) {
    return (u32)__builtin_ctzll(x);
}

static really_inline
u32 lg2_impl_c(u32 x) {
    if (!x) {
        return 0;
    }
    return 31 - clz32_impl_c(x);
}

static really_inline
u64a lg2_64_impl_c(u64a x) {
    if (!x) {
        return 0;
    }
    return 63 - clz64_impl_c(x);
}

static really_inline
u32 findAndClearLSB_32_impl_c(u32 *v) {
    u32 val = *v;
    u32 offset = ctz32_impl_c(val);
    *v = val & (val - 1);

    assert(offset < 32);
    return offset;
}

static really_inline
u32 findAndClearLSB_64_impl_c(u64a *v) {
#ifdef ARCH_64_BIT
    // generic variant using gcc's builtin on 64-bit
    u64a val = *v, offset;
    offset = ctz64_impl_c(val);
    *v = val & (val - 1);
#else
    // fall back to doing things with two 32-bit cases, since gcc-4.1 doesn't
    // inline calls to __builtin_ctzll
    u32 v1 = (u32)*v;
    u32 v2 = (u32)(*v >> 32);
    u32 offset;
    if (v1) {
        offset = findAndClearLSB_32_impl_c(&v1);
        *v = (u64a)v1 | ((u64a)v2 << 32);
    } else {
        offset = findAndClearLSB_32_impl_c(&v2) + 32;
        *v = (u64a)v2 << 32;
    }
#endif

    assert(offset < 64);
    return (u32)offset;
}

static really_inline
u32 findAndClearMSB_32_impl_c(u32 *v) {
    u32 val = *v;
    u32 offset = 31 - clz32_impl_c(val);
    *v = val & ~(1 << offset);

    assert(offset < 32);
    return offset;
}

static really_inline
u32 findAndClearMSB_64_impl_c(u64a *v) {
#ifdef ARCH_64_BIT
    // generic variant using gcc's builtin on 64-bit
    u64a val = *v, offset;
    offset = 63 - clz64_impl_c(val);
    *v = val & ~(1ULL << offset);
#else
    // fall back to doing things with two 32-bit cases, since gcc-4.1 doesn't
    // inline calls to __builtin_ctzll
    u32 v1 = (u32)*v;
    u32 v2 = (*v >> 32);
    u32 offset;
    if (v2) {
        offset = findAndClearMSB_32_impl_c(&v2) + 32;
        *v = ((u64a)v2 << 32) | (u64a)v1;
    } else {
        offset = findAndClearMSB_32_impl_c(&v1);
        *v = (u64a)v1;
    }
#endif

    assert(offset < 64);
    return (u32)offset;
}

static really_inline
u32 compress32_impl_c(u32 x, u32 m) {

    // Return zero quickly on trivial cases
    if ((x & m) == 0) {
        return 0;
    }

    u32 mk, mp, mv, t;

    x &= m; // clear irrelevant bits

    mk = ~m << 1; // we will count 0's to right
    for (u32 i = 0; i < 5; i++) {
        mp = mk ^ (mk << 1);
        mp ^= mp << 2;
        mp ^= mp << 4;
        mp ^= mp << 8;
        mp ^= mp << 16;

        mv = mp & m; // bits to move
        m = (m ^ mv) | (mv >> (1 << i)); // compress m
        t = x & mv;
        x = (x ^ t) | (t >> (1 << i)); // compress x
        mk = mk & ~mp;
    }

    return x;
}

static really_inline
u64a compress64_impl_c(u64a x, u64a m) {
    // Return zero quickly on trivial cases
    if ((x & m) == 0) {
        return 0;
    }

    u64a mk, mp, mv, t;

    x &= m; // clear irrelevant bits

    mk = ~m << 1; // we will count 0's to right
    for (u32 i = 0; i < 6; i++) {
        mp = mk ^ (mk << 1);
        mp ^= mp << 2;
        mp ^= mp << 4;
        mp ^= mp << 8;
        mp ^= mp << 16;
        mp ^= mp << 32;

        mv = mp & m; // bits to move
        m = (m ^ mv) | (mv >> (1 << i)); // compress m
        t = x & mv;
        x = (x ^ t) | (t >> (1 << i)); // compress x
        mk = mk & ~mp;
    }

    return x;
}

static really_inline
u32 expand32_impl_c(u32 x, u32 m) {
    // Return zero quickly on trivial cases
    if (!x || !m) {
        return 0;
    }

    u32 m0, mk, mp, mv, t;
    u32 array[5];

    m0 = m; // save original mask
    mk = ~m << 1; // we will count 0's to right

    for (int i = 0; i < 5; i++) {
        mp = mk ^ (mk << 1); // parallel suffix
        mp = mp ^ (mp << 2);
        mp = mp ^ (mp << 4);
        mp = mp ^ (mp << 8);
        mp = mp ^ (mp << 16);
        mv = mp & m; // bits to move
        array[i] = mv;
        m = (m ^ mv) | (mv >> (1 << i)); // compress m
        mk = mk & ~mp;
    }

    for (int i = 4; i >= 0; i--) {
        mv = array[i];
        t = x << (1 << i);
        x = (x & ~mv) | (t & mv);
    }

    return x & m0; // clear out extraneous bits
}

static really_inline
u64a expand64_impl_c(u64a x, u64a m) {

    // Return zero quickly on trivial cases
    if (!x || !m) {
        return 0;
    }

    u64a m0, mk, mp, mv, t;
    u64a array[6];

    m0 = m; // save original mask
    mk = ~m << 1; // we will count 0's to right

    for (int i = 0; i < 6; i++) {
        mp = mk ^ (mk << 1); // parallel suffix
        mp = mp ^ (mp << 2);
        mp = mp ^ (mp << 4);
        mp = mp ^ (mp << 8);
        mp = mp ^ (mp << 16);
        mp = mp ^ (mp << 32);
        mv = mp & m; // bits to move
        array[i] = mv;
        m = (m ^ mv) | (mv >> (1 << i)); // compress m
        mk = mk & ~mp;
    }

    for (int i = 5; i >= 0; i--) {
        mv = array[i];
        t = x << (1 << i);
        x = (x & ~mv) | (t & mv);
    }

    return x & m0; // clear out extraneous bits
}


/* returns the first set bit after begin (if not ~0U). If no bit is set after
 * begin returns ~0U
 */
static really_inline
u32 bf64_iterate_impl_c(u64a bitfield, u32 begin) {
    if (begin != ~0U) {
        /* switch off all bits at or below begin. Note: not legal to shift by
         * by size of the datatype or larger. */
        assert(begin <= 63);
        bitfield &= ~((2ULL << begin) - 1);
    }

    if (!bitfield) {
        return ~0U;
    }

    return ctz64_impl_c(bitfield);
}

static really_inline
char bf64_set_impl_c(u64a *bitfield, u32 i) {
    u64a mask = 1ULL << i;
    char was_set = !!(*bitfield & mask);
    *bitfield |= mask;

    return was_set;
}

static really_inline
void bf64_unset_impl_c(u64a *bitfield, u32 i) {
    *bitfield &= ~(1ULL << i);
}

static really_inline
u32 rank_in_mask32_impl_c(u32 mask, u32 bit) {
    mask &= (u32)(1U << bit) - 1;
    return popcount32(mask);
}

static really_inline
u32 rank_in_mask64_impl_c(u64a mask, u32 bit) {
    mask &= (u64a)(1ULL << bit) - 1;
    return popcount64(mask);
}

static really_inline
u32 pext32_impl_c(u32 x, u32 mask) {

    u32 result = 0, num = 1;
    while (mask != 0) {
        u32 bit = findAndClearLSB_32_impl_c(&mask);
        if (x & (1U << bit)) {
            assert(num != 0); // more than 32 bits!
            result |= num;
        }
        num <<= 1;
    }
    return result;
}

static really_inline
u64a pext64_impl_c(u64a x, u64a mask) {

    u32 result = 0, num = 1;
    while (mask != 0) {
        u32 bit = findAndClearLSB_64_impl_c(&mask);
        if (x & (1ULL << bit)) {
            assert(num != 0); // more than 32 bits!
            result |= num;
        }
        num <<= 1;
    }
    return result;
}

/* compilers don't reliably synthesize the 32-bit ANDN instruction here,
 * so we force its generation.
 */
static really_inline
u64a andn_impl_c(const u32 a, const u8 *b) {
    return unaligned_load_u32(b) & ~a;
}

#endif // BITUTILS_ARCH_COMMON_H
