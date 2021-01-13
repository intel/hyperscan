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
 * \brief Uniformly-named primitives named by target type.
 *
 * The following are a set of primitives named by target type, so that we can
 * macro the hell out of all our NFA implementations. Hurrah!
 */

#ifndef UNIFORM_OPS_H
#define UNIFORM_OPS_H

#include "ue2common.h"
#include "simd_utils.h"
#include "unaligned.h"

// Aligned loads
#define load_u8(a)          (*(const u8 *)(a))
#define load_u16(a)         (*(const u16 *)(a))
#define load_u32(a)         (*(const u32 *)(a))
#define load_u64a(a)        (*(const u64a *)(a))
#define load_m128(a)        load128(a)
#define load_m256(a)        load256(a)
#define load_m384(a)        load384(a)
#define load_m512(a)        load512(a)

// Unaligned loads
#define loadu_u8(a)          (*(const u8 *)(a))
#define loadu_u16(a)         unaligned_load_u16((const u8 *)(a))
#define loadu_u32(a)         unaligned_load_u32((const u8 *)(a))
#define loadu_u64a(a)        unaligned_load_u64a((const u8 *)(a))
#define loadu_m128(a)        loadu128(a)
#define loadu_m256(a)        loadu256(a)
#define loadu_m384(a)        loadu384(a)
#define loadu_m512(a)        loadu512(a)

// Aligned stores
#define store_u8(ptr, a)    do { *(u8 *)(ptr) = (a); } while(0)
#define store_u16(ptr, a)   do { *(u16 *)(ptr) = (a); } while(0)
#define store_u32(ptr, a)   do { *(u32 *)(ptr) = (a); } while(0)
#define store_u64a(ptr, a)  do { *(u64a *)(ptr) = (a); } while(0)
#define store_m128(ptr, a)  store128(ptr, a)
#define store_m256(ptr, a)  store256(ptr, a)
#define store_m384(ptr, a)  store384(ptr, a)
#define store_m512(ptr, a)  store512(ptr, a)

// Unaligned stores
#define storeu_u8(ptr, a)    do { *(u8 *)(ptr) = (a); } while(0)
#define storeu_u16(ptr, a)   unaligned_store_u16(ptr, a)
#define storeu_u32(ptr, a)   unaligned_store_u32(ptr, a)
#define storeu_u64a(ptr, a)  unaligned_store_u64a(ptr, a)
#define storeu_m128(ptr, a)  storeu128(ptr, a)

#define zero_u8             0
#define zero_u32            0
#define zero_u64a           0
#define zero_m128           zeroes128()
#define zero_m256           zeroes256()
#define zero_m384           zeroes384()
#define zero_m512           zeroes512()

#define ones_u8             0xff
#define ones_u32            0xfffffffful
#define ones_u64a           0xffffffffffffffffull
#define ones_m128           ones128()
#define ones_m256           ones256()
#define ones_m384           ones384()
#define ones_m512           ones512()

#define or_u8(a, b)         ((a) | (b))
#define or_u32(a, b)        ((a) | (b))
#define or_u64a(a, b)       ((a) | (b))
#define or_m128(a, b)       (or128(a, b))
#define or_m256(a, b)       (or256(a, b))
#define or_m384(a, b)       (or384(a, b))
#define or_m512(a, b)       (or512(a, b))

#if defined(HAVE_AVX512VBMI)
#define expand_m128(a)      (expand128(a))
#define expand_m256(a)      (expand256(a))
#define expand_m384(a)      (expand384(a))
#define expand_m512(a)      (a)

#define shuffle_byte_m128(a, b)       (pshufb_m512(b, a))
#define shuffle_byte_m256(a, b)       (vpermb512(a, b))
#define shuffle_byte_m384(a, b)       (vpermb512(a, b))
#define shuffle_byte_m512(a, b)       (vpermb512(a, b))
#endif

#define and_u8(a, b)        ((a) & (b))
#define and_u32(a, b)       ((a) & (b))
#define and_u64a(a, b)      ((a) & (b))
#define and_m128(a, b)      (and128(a, b))
#define and_m256(a, b)      (and256(a, b))
#define and_m384(a, b)      (and384(a, b))
#define and_m512(a, b)      (and512(a, b))

#define not_u8(a)           (~(a))
#define not_u32(a)          (~(a))
#define not_u64a(a)         (~(a))
#define not_m128(a)         (not128(a))
#define not_m256(a)         (not256(a))
#define not_m384(a)         (not384(a))
#define not_m512(a)         (not512(a))

#define andnot_u8(a, b)     ((~(a)) & (b))
#define andnot_u32(a, b)    ((~(a)) & (b))
#define andnot_u64a(a, b)   ((~(a)) & (b))
#define andnot_m128(a, b)   (andnot128(a, b))
#define andnot_m256(a, b)   (andnot256(a, b))
#define andnot_m384(a, b)   (andnot384(a, b))
#define andnot_m512(a, b)   (andnot512(a, b))

#define lshift_u32(a, b)    ((a) << (b))
#define lshift_u64a(a, b)   ((a) << (b))
#define lshift_m128(a, b)   (lshift64_m128(a, b))
#define lshift_m256(a, b)   (lshift64_m256(a, b))
#define lshift_m384(a, b)   (lshift64_m384(a, b))
#define lshift_m512(a, b)   (lshift64_m512(a, b))

#define isZero_u8(a)        ((a) == 0)
#define isZero_u32(a)       ((a) == 0)
#define isZero_u64a(a)      ((a) == 0)
#define isZero_m128(a)      (!isnonzero128(a))
#define isZero_m256(a)      (!isnonzero256(a))
#define isZero_m384(a)      (!isnonzero384(a))
#define isZero_m512(a)      (!isnonzero512(a))

#define isNonZero_u8(a)     ((a) != 0)
#define isNonZero_u32(a)    ((a) != 0)
#define isNonZero_u64a(a)   ((a) != 0)
#define isNonZero_m128(a)   (isnonzero128(a))
#define isNonZero_m256(a)   (isnonzero256(a))
#define isNonZero_m384(a)   (isnonzero384(a))
#define isNonZero_m512(a)   (isnonzero512(a))

#define diffrich_u32(a, b)  ((a) != (b))
#define diffrich_u64a(a, b) ((a) != (b) ? 3 : 0) //TODO: impl 32bit granularity
#define diffrich_m128(a, b) (diffrich128(a, b))
#define diffrich_m256(a, b) (diffrich256(a, b))
#define diffrich_m384(a, b) (diffrich384(a, b))
#define diffrich_m512(a, b) (diffrich512(a, b))

#define diffrich64_u32(a, b)  ((a) != (b))
#define diffrich64_u64a(a, b) ((a) != (b) ? 1 : 0)
#define diffrich64_m128(a, b) (diffrich64_128(a, b))
#define diffrich64_m256(a, b) (diffrich64_256(a, b))
#define diffrich64_m384(a, b) (diffrich64_384(a, b))
#define diffrich64_m512(a, b) (diffrich64_512(a, b))

#define noteq_u8(a, b)      ((a) != (b))
#define noteq_u32(a, b)     ((a) != (b))
#define noteq_u64a(a, b)    ((a) != (b))
#define noteq_m128(a, b)    (diff128(a, b))
#define noteq_m256(a, b)    (diff256(a, b))
#define noteq_m384(a, b)    (diff384(a, b))
#define noteq_m512(a, b)    (diff512(a, b))

#define partial_store_m128(ptr, v, sz) storebytes128(ptr, v, sz)
#define partial_store_m256(ptr, v, sz) storebytes256(ptr, v, sz)
#define partial_store_m384(ptr, v, sz) storebytes384(ptr, v, sz)
#define partial_store_m512(ptr, v, sz) storebytes512(ptr, v, sz)

#define partial_load_m128(ptr, sz) loadbytes128(ptr, sz)
#define partial_load_m256(ptr, sz) loadbytes256(ptr, sz)
#define partial_load_m384(ptr, sz) loadbytes384(ptr, sz)
#define partial_load_m512(ptr, sz) loadbytes512(ptr, sz)

#define store_compressed_u32(ptr, x, m, len)  storecompressed32(ptr, x, m, len)
#define store_compressed_u64a(ptr, x, m, len) storecompressed64(ptr, x, m, len)
#define store_compressed_m128(ptr, x, m, len) storecompressed128(ptr, x, m, len)
#define store_compressed_m256(ptr, x, m, len) storecompressed256(ptr, x, m, len)
#define store_compressed_m384(ptr, x, m, len) storecompressed384(ptr, x, m, len)
#define store_compressed_m512(ptr, x, m, len) storecompressed512(ptr, x, m, len)

#define load_compressed_u32(x, ptr, m, len)   loadcompressed32(x, ptr, m, len)
#define load_compressed_u64a(x, ptr, m, len)  loadcompressed64(x, ptr, m, len)
#define load_compressed_m128(x, ptr, m, len)  loadcompressed128(x, ptr, m, len)
#define load_compressed_m256(x, ptr, m, len)  loadcompressed256(x, ptr, m, len)
#define load_compressed_m384(x, ptr, m, len)  loadcompressed384(x, ptr, m, len)
#define load_compressed_m512(x, ptr, m, len)  loadcompressed512(x, ptr, m, len)

static really_inline
void clearbit_u32(u32 *p, u32 n) {
    assert(n < sizeof(*p) * 8);
    *p &= ~(1U << n);
}

static really_inline
void clearbit_u64a(u64a *p, u32 n) {
    assert(n < sizeof(*p) * 8);
    *p &= ~(1ULL << n);
}

#define clearbit_m128(ptr, n)   (clearbit128(ptr, n))
#define clearbit_m256(ptr, n)   (clearbit256(ptr, n))
#define clearbit_m384(ptr, n)   (clearbit384(ptr, n))
#define clearbit_m512(ptr, n)   (clearbit512(ptr, n))

static really_inline
char testbit_u32(u32 val, u32 n) {
    assert(n < sizeof(val) * 8);
    return !!(val & (1U << n));
}

static really_inline
char testbit_u64a(u64a val, u32 n) {
    assert(n < sizeof(val) * 8);
    return !!(val & (1ULL << n));
}

#define testbit_m128(val, n)    (testbit128(val, n))
#define testbit_m256(val, n)    (testbit256(val, n))
#define testbit_m384(val, n)    (testbit384(val, n))
#define testbit_m512(val, n)    (testbit512(val, n))

#endif
