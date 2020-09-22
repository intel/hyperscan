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

#ifndef BITUTILS_H
#define BITUTILS_H

#include "config.h"
#include "ue2common.h"
#include "popcount.h"
#include "util/arch.h"
#include "util/intrinsics.h"

#define CASE_BIT          0x20
#define CASE_CLEAR        0xdf
#define DOUBLE_CASE_CLEAR 0xdfdf
#define OCTO_CASE_CLEAR   0xdfdfdfdfdfdfdfdfULL


#if defined(_WIN32) || defined(_WIN64) || defined(ARCH_IA32) || defined(ARCH_X86_64)
#include "util/arch/x86/bitutils.h"
#endif

static really_inline
u32 clz32(u32 x) {
    assert(x); // behaviour not defined for x == 0

    return clz32_impl(x);
}

static really_inline
u32 clz64(u64a x) {
    assert(x); // behaviour not defined for x == 0

    return clz64_impl(x);
}

// CTZ (count trailing zero) implementations.
static really_inline
u32 ctz32(u32 x) {
    assert(x); // behaviour not defined for x == 0

    return ctz32_impl(x);
}

static really_inline
u32 ctz64(u64a x) {
    assert(x); // behaviour not defined for x == 0

    return ctz64_impl(x);
}

static really_inline
u32 lg2(u32 x) {
    return lg2_impl(x);
}

static really_inline
u64a lg2_64(u64a x) {
    return lg2_64_impl(x);
}

static really_inline
u32 findAndClearLSB_32(u32 *v) {
    return findAndClearLSB_32_impl(v);
}

static really_inline
u32 findAndClearLSB_64(u64a *v) {
    return findAndClearLSB_64_impl(v);
}

static really_inline
u32 findAndClearMSB_32(u32 *v) {
    return findAndClearMSB_32_impl(v);
}

static really_inline
u32 findAndClearMSB_64(u64a *v) {
    return findAndClearMSB_64_impl(v);
}

static really_inline
u32 compress32(u32 x, u32 m) {
    return compress32_impl(x, m);
}

static really_inline
u64a compress64(u64a x, u64a m) {
    return compress64_impl(x, m);
}

static really_inline
u32 expand32(u32 x, u32 m) {
    return expand32_impl(x, m);
}

static really_inline
u64a expand64(u64a x, u64a m) {
    return expand64_impl(x, m);
}


/* returns the first set bit after begin (if not ~0U). If no bit is set after
 * begin returns ~0U
 */
static really_inline
u32 bf64_iterate(u64a bitfield, u32 begin) {
    return bf64_iterate_impl(bitfield, begin);
}

static really_inline
char bf64_set(u64a *bitfield, u32 i) {
    return bf64_set_impl(bitfield, i);
}

static really_inline
void bf64_unset(u64a *bitfield, u32 i) {
    return bf64_unset_impl(bitfield, i);
}

static really_inline
u32 rank_in_mask32(u32 mask, u32 bit) {
    return rank_in_mask32_impl(mask, bit);
}

static really_inline
u32 rank_in_mask64(u64a mask, u32 bit) {
    return rank_in_mask64_impl(mask, bit);
}

static really_inline
u32 pext32(u32 x, u32 mask) {
    return pext32_impl(x, mask);
}

static really_inline
u64a pext64(u64a x, u64a mask) {
    return pext64_impl(x, mask);
}

#endif // BITUTILS_H
