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

#ifndef BITUTILS_ARCH_X86_H
#define BITUTILS_ARCH_X86_H

#include "ue2common.h"
#include "util/popcount.h"
#include "util/arch.h"
#include "util/intrinsics.h"

#include "util/arch/common/bitutils.h"

static really_inline
u32 clz32_impl(u32 x) {
#if defined(_WIN32)
    unsigned long r;
    _BitScanReverse(&r, x);
    return 31 - r;
#else
    return clz32_impl_c(x);
#endif
}

static really_inline
u32 clz64_impl(u64a x) {
#if defined(_WIN64)
    unsigned long r;
    _BitScanReverse64(&r, x);
    return 63 - r;
#elif defined(_WIN32)
    unsigned long x1 = (u32)x;
    unsigned long x2 = (u32)(x >> 32);
    unsigned long r;
    if (x2) {
        _BitScanReverse(&r, x2);
        return (u32)(31 - r);
    }
    _BitScanReverse(&r, (u32)x1);
    return (u32)(63 - r);
#else
    return clz64_impl_c(x);
#endif
}

// CTZ (count trailing zero) implementations.
static really_inline
u32 ctz32_impl(u32 x) {
#if defined(_WIN32)
    unsigned long r;
    _BitScanForward(&r, x);
    return r;
#else
    return ctz32_impl_c(x);
#endif
}

static really_inline
u32 ctz64_impl(u64a x) {
#if defined(_WIN64)
    unsigned long r;
    _BitScanForward64(&r, x);
    return r;
#elif defined(_WIN32)
    unsigned long r;
    if (_BitScanForward(&r, (u32)x)) {
        return (u32)r;
    }
    _BitScanForward(&r, x >> 32);
    return (u32)(r + 32);
#else
    return ctz64_impl_c(x);
#endif
}

static really_inline
u32 lg2_impl(u32 x) {
    return lg2_impl_c(x);
}

static really_inline
u64a lg2_64_impl(u64a x) {
    return lg2_64_impl_c(x);
}

static really_inline
u32 findAndClearLSB_32_impl(u32 *v) {
#ifndef NO_ASM
    u32 val = *v, offset;
    __asm__ ("bsf %1, %0\n"
             "btr %0, %1\n"
             : "=r" (offset), "=r" (val)
             : "1" (val));
    *v = val;

    assert(offset < 32);
    return offset;
#else
    return findAndClearLSB_32_impl_c(v);
#endif

}

static really_inline
u32 findAndClearLSB_64_impl(u64a *v) {
#ifdef ARCH_64_BIT
#if !defined(NO_ASM)
    u64a val = *v, offset;
    __asm__ ("bsfq %1, %0\n"
             "btrq %0, %1\n"
             : "=r" (offset), "=r" (val)
             : "1" (val));
    *v = val;
#else
    // generic variant using gcc's builtin on 64-bit
    u64a val = *v, offset;
    offset = ctz64(val);
    *v = val & (val - 1);
#endif // ARCH_X86_64
    assert(offset < 64);
    return (u32)offset;
#else
    return findAndClearLSB_64_impl_c(v);
#endif
}

static really_inline
u32 findAndClearMSB_32_impl(u32 *v) {
#if !defined(NO_ASM)
    u32 val = *v, offset;
    __asm__ ("bsr %1, %0\n"
             "btr %0, %1\n"
             : "=r" (offset), "=r" (val)
             : "1" (val));
    *v = val;
#else
    u32 val = *v;
    u32 offset = 31 - clz32_impl(val);
    *v = val & ~(1 << offset);
#endif
    assert(offset < 32);
    return offset;
}

static really_inline
u32 findAndClearMSB_64_impl(u64a *v) {
#ifdef ARCH_64_BIT
#if !defined(NO_ASM)
    u64a val = *v, offset;
    __asm__ ("bsrq %1, %0\n"
             "btrq %0, %1\n"
             : "=r" (offset), "=r" (val)
             : "1" (val));
    *v = val;
#else
    // generic variant using gcc's builtin on 64-bit
    u64a val = *v, offset;
    offset = 63 - clz64_impl(val);
    *v = val & ~(1ULL << offset);
#endif // ARCH_X86_64
    assert(offset < 64);
    return (u32)offset;
#else
    return findAndClearMSB_64_impl_c(v);
#endif
}

static really_inline
u32 compress32_impl(u32 x, u32 m) {
#if defined(HAVE_BMI2)
    // BMI2 has a single instruction for this operation.
    return _pext_u32(x, m);
#else
    return compress32_impl_c(x, m);
#endif
}

static really_inline
u64a compress64_impl(u64a x, u64a m) {
#if defined(ARCH_X86_64) && defined(HAVE_BMI2)
    // BMI2 has a single instruction for this operation.
    return _pext_u64(x, m);
#else
    return compress64_impl_c(x, m);
#endif
}

static really_inline
m128 compress128_impl(m128 x, m128 m) {
    return compress128_impl_c(x, m);
}

static really_inline
u32 expand32_impl(u32 x, u32 m) {
#if defined(HAVE_BMI2)
    // BMI2 has a single instruction for this operation.
    return _pdep_u32(x, m);
#else
    return expand32_impl_c(x, m);
#endif
}

static really_inline
u64a expand64_impl(u64a x, u64a m) {
#if defined(ARCH_X86_64) && defined(HAVE_BMI2)
    // BMI2 has a single instruction for this operation.
    return _pdep_u64(x, m);
#else
    return expand64_impl_c(x, m);
#endif
}

/* returns the first set bit after begin (if not ~0U). If no bit is set after
 * begin returns ~0U
 */
static really_inline
u32 bf64_iterate_impl(u64a bitfield, u32 begin) {
    if (begin != ~0U) {
        /* switch off all bits at or below begin. Note: not legal to shift by
         * by size of the datatype or larger. */
        assert(begin <= 63);
        bitfield &= ~((2ULL << begin) - 1);
    }

    if (!bitfield) {
        return ~0U;
    }

    return ctz64_impl(bitfield);
}

static really_inline
char bf64_set_impl(u64a *bitfield, u32 i) {
    return bf64_set_impl_c(bitfield, i);
}

static really_inline
void bf64_unset_impl(u64a *bitfield, u32 i) {
    return bf64_unset_impl_c(bitfield, i);
}

static really_inline
u32 rank_in_mask32_impl(u32 mask, u32 bit) {
    return rank_in_mask32_impl_c(mask, bit);
}

static really_inline
u32 rank_in_mask64_impl(u64a mask, u32 bit) {
    return rank_in_mask64_impl_c(mask, bit);
}

static really_inline
u32 pext32_impl(u32 x, u32 mask) {
#if defined(HAVE_BMI2)
    // Intel BMI2 can do this operation in one instruction.
    return _pext_u32(x, mask);
#else
    return pext32_impl_c(x, mask);
#endif
}

static really_inline
u64a pext64_impl(u64a x, u64a mask) {
#if defined(HAVE_BMI2) && defined(ARCH_64_BIT)
    // Intel BMI2 can do this operation in one instruction.
    return _pext_u64(x, mask);
#else
    return pext64_impl_c(x, mask);
#endif
}

#if defined(HAVE_BMI2) && defined(ARCH_64_BIT)
static really_inline
u64a pdep64(u64a x, u64a mask) {
    return _pdep_u64(x, mask);
}
#endif

/* compilers don't reliably synthesize the 32-bit ANDN instruction here,
 * so we force its generation.
 */
static really_inline
u64a andn_impl(const u32 a, const u8 *b) {
#if defined(HAVE_BMI) && !defined(NO_ASM)
    u64a r;
    __asm__ ("andn\t%2,%1,%k0" : "=r"(r) : "r"(a), "m"(*(const u32 *)b));
    return r;
#else
    return andn_impl_c(a, b);
#endif
}

#endif // BITUTILS_ARCH_X86_H
