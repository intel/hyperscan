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

#ifndef BITUTILS_ARCH_ARM_H
#define BITUTILS_ARCH_ARM_H

#include "ue2common.h"
#include "util/popcount.h"
#include "util/arch.h"
#include "util/intrinsics.h"

#include "util/arch/common/bitutils.h"

static really_inline
u32 clz32_impl(u32 x) {
    return clz32_impl_c(x);
}

static really_inline
u32 clz64_impl(u64a x) {
    return clz64_impl_c(x);
}

static really_inline
u32 ctz32_impl(u32 x) {
    return ctz32_impl_c(x);
}

static really_inline
u32 ctz64_impl(u64a x) {
    return ctz64_impl_c(x);
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
    return findAndClearLSB_32_impl_c(v);
}

static really_inline
u32 findAndClearLSB_64_impl(u64a *v) {
    return findAndClearLSB_64_impl_c(v);
}

static really_inline
u32 findAndClearMSB_32_impl(u32 *v) {
    u32 val = *v;
    u32 offset = 31 - clz32_impl(val);
    *v = val & ~(1 << offset);
    assert(offset < 32);
    return offset;
}

static really_inline
u32 findAndClearMSB_64_impl(u64a *v) {
    return findAndClearMSB_64_impl_c(v);
}

static really_inline
u32 compress32_impl(u32 x, u32 m) {
    return compress32_impl_c(x, m);
}

static really_inline
u64a compress64_impl(u64a x, u64a m) {
    return compress64_impl_c(x, m);
}

static really_inline
u32 expand32_impl(u32 x, u32 m) {
    return expand32_impl_c(x, m);
}

static really_inline
u64a expand64_impl(u64a x, u64a m) {
    return expand64_impl_c(x, m);
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
    return pext32_impl_c(x, mask);
}

static really_inline
u64a pext64_impl(u64a x, u64a mask) {
    return pext64_impl_c(x, mask);
}

static really_inline
u64a pdep64(u64a x, u64a mask) {
    return pdep64_impl_c(x, mask);
}

/* compilers don't reliably synthesize the 32-bit ANDN instruction here,
 * so we force its generation.
 */
static really_inline
u64a andn_impl(const u32 a, const u8 *b) {
    return andn_impl_c(a, b);
}

#endif // BITUTILS_ARCH_ARM_H
