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
 * \brief SIMD types and primitive operations.
 */

#ifndef ARCH_COMMON_SIMD_UTILS_H
#define ARCH_COMMON_SIMD_UTILS_H

#include "ue2common.h"
#include "util/simd_types.h"
#include "util/unaligned.h"
#include "util/intrinsics.h"

#include <string.h> // for memcpy

#if !defined(HAVE_SIMD_128_BITS)
#error "You need at least a 128-bit capable SIMD engine!"
#endif // HAVE_SIMD_128_BITS

/****
 **** 256-bit Primitives
 ****/

#if !defined(HAVE_SIMD_256_BITS)

static really_really_inline
m256 lshift64_m256(m256 a, int b) {
    m256 rv = a;
    rv.lo = lshift64_m128(rv.lo, b);
    rv.hi = lshift64_m128(rv.hi, b);
    return rv;
}

static really_inline
m256 rshift64_m256(m256 a, int b) {
    m256 rv = a;
    rv.lo = rshift64_m128(rv.lo, b);
    rv.hi = rshift64_m128(rv.hi, b);
    return rv;
}

static really_inline
m256 eq256(m256 a, m256 b) {
    m256 rv;
    rv.lo = eq128(a.lo, b.lo);
    rv.hi = eq128(a.hi, b.hi);
    return rv;
}

static really_inline
u32 movemask256(m256 a) {
    u32 lo_mask = movemask128(a.lo);
    u32 hi_mask = movemask128(a.hi);
    return lo_mask | (hi_mask << 16);
}

static really_inline m256 set1_4x64(u64a c) {
    m128 a128 = set1_2x64(c);
    m256 rv = {a128, a128};
    return rv;
}

static really_inline
m256 set1_2x128(m128 a) {
    m256 rv = {a, a};
    return rv;
}

static really_inline m256 zeroes256(void) {
    m256 rv = {zeroes128(), zeroes128()};
    return rv;
}

static really_inline m256 ones256(void) {
    m256 rv = {ones128(), ones128()};
    return rv;
}

static really_inline m256 and256(m256 a, m256 b) {
    m256 rv;
    rv.lo = and128(a.lo, b.lo);
    rv.hi = and128(a.hi, b.hi);
    return rv;
}

static really_inline m256 or256(m256 a, m256 b) {
    m256 rv;
    rv.lo = or128(a.lo, b.lo);
    rv.hi = or128(a.hi, b.hi);
    return rv;
}

static really_inline m256 xor256(m256 a, m256 b) {
    m256 rv;
    rv.lo = xor128(a.lo, b.lo);
    rv.hi = xor128(a.hi, b.hi);
    return rv;
}

static really_inline m256 not256(m256 a) {
    m256 rv;
    rv.lo = not128(a.lo);
    rv.hi = not128(a.hi);
    return rv;
}

static really_inline m256 andnot256(m256 a, m256 b) {
    m256 rv;
    rv.lo = andnot128(a.lo, b.lo);
    rv.hi = andnot128(a.hi, b.hi);
    return rv;
}

static really_inline int diff256(m256 a, m256 b) {
    return diff128(a.lo, b.lo) || diff128(a.hi, b.hi);
}

static really_inline int isnonzero256(m256 a) {
    return isnonzero128(or128(a.lo, a.hi));
}

/**
 * "Rich" version of diff256(). Takes two vectors a and b and returns a 8-bit
 * mask indicating which 32-bit words contain differences.
 */
static really_inline
u32 diffrich256(m256 a, m256 b) {
    return diffrich128(a.lo, b.lo) | (diffrich128(a.hi, b.hi) << 4);
}

/**
 * "Rich" version of diff256(), 64-bit variant. Takes two vectors a and b and
 * returns an 8-bit mask indicating which 64-bit words contain differences.
 */
static really_inline u32 diffrich64_256(m256 a, m256 b) {
    u32 d = diffrich256(a, b);
    return (d | (d >> 1)) & 0x55555555;
}

// aligned load
static really_inline m256 load256(const void *ptr) {
    assert(ISALIGNED_N(ptr, alignof(m256)));
    m256 rv = { load128(ptr), load128((const char *)ptr + 16) };
    return rv;
}

// aligned load  of 128-bit value to low and high part of 256-bit value
static really_inline m256 load2x128(const void *ptr) {
    return set1_2x128(load128(ptr));
}

static really_inline m256 loadu2x128(const void *ptr) {
    return set1_2x128(loadu128(ptr));
}

// aligned store
static really_inline void store256(void *ptr, m256 a) {
    assert(ISALIGNED_N(ptr, alignof(m256)));
    ptr = assume_aligned(ptr, 16);
    *(m256 *)ptr = a;
}

// unaligned load
static really_inline m256 loadu256(const void *ptr) {
    m256 rv = { loadu128(ptr), loadu128((const char *)ptr + 16) };
    return rv;
}

// unaligned store
static really_inline void storeu256(void *ptr, m256 a) {
    storeu128(ptr, a.lo);
    storeu128((char *)ptr + 16, a.hi);
}

// packed unaligned store of first N bytes
static really_inline
void storebytes256(void *ptr, m256 a, unsigned int n) {
    assert(n <= sizeof(a));
    memcpy(ptr, &a, n);
}

// packed unaligned load of first N bytes, pad with zero
static really_inline
m256 loadbytes256(const void *ptr, unsigned int n) {
    m256 a = zeroes256();
    assert(n <= sizeof(a));
    memcpy(&a, ptr, n);
    return a;
}

static really_inline
m256 mask1bit256(unsigned int n) {
    assert(n < sizeof(m256) * 8);
    u32 mask_idx = ((n % 8) * 64) + 95;
    mask_idx -= n / 8;
    return loadu256(&simd_onebit_masks[mask_idx]);
}

static really_inline
m256 set1_32x8(u32 in) {
    m256 rv;
    rv.hi = set1_16x8(in);
    rv.lo = set1_16x8(in);
    return rv;
}

static really_inline
m256 set8x32(u32 hi_3, u32 hi_2, u32 hi_1, u32 hi_0, u32 lo_3, u32 lo_2, u32 lo_1, u32 lo_0) {
    m256 rv;
    rv.hi = set4x32(hi_3, hi_2, hi_1, hi_0);
    rv.lo = set4x32(lo_3, lo_2, lo_1, lo_0);
    return rv;
}

static really_inline
m256 set4x64(u64a hi_1, u64a hi_0, u64a lo_1, u64a lo_0) {
    m256 rv;
    rv.hi = set2x64(hi_1, hi_0);
    rv.lo = set2x64(lo_1, lo_0);
    return rv;
}

// switches on bit N in the given vector.
static really_inline
void setbit256(m256 *ptr, unsigned int n) {
    assert(n < sizeof(*ptr) * 8);
    m128 *sub;
    if (n < 128) {
        sub = &ptr->lo;
    } else {
        sub = &ptr->hi;
        n -= 128;
    }
    setbit128(sub, n);
}

// switches off bit N in the given vector.
static really_inline
void clearbit256(m256 *ptr, unsigned int n) {
    assert(n < sizeof(*ptr) * 8);
    m128 *sub;
    if (n < 128) {
        sub = &ptr->lo;
    } else {
        sub = &ptr->hi;
        n -= 128;
    }
    clearbit128(sub, n);
}

// tests bit N in the given vector.
static really_inline
char testbit256(m256 val, unsigned int n) {
    assert(n < sizeof(val) * 8);
    m128 sub;
    if (n < 128) {
        sub = val.lo;
    } else {
        sub = val.hi;
        n -= 128;
    }
    return testbit128(sub, n);
}

static really_really_inline
m128 movdq_hi(m256 x) {
    return x.hi;
}

static really_really_inline
m128 movdq_lo(m256 x) {
    return x.lo;
}

static really_inline
m256 combine2x128(m128 hi, m128 lo) {
    m256 rv = {lo, hi};
    return rv;
}

static really_inline
m256 pshufb_m256(m256 a, m256 b) {
    m256 rv;
    rv.lo = pshufb_m128(a.lo, b.lo);
    rv.hi = pshufb_m128(a.hi, b.hi);
    return rv;
}

#endif // HAVE_SIMD_256_BITS

/****
 **** 384-bit Primitives
 ****/

static really_inline m384 and384(m384 a, m384 b) {
    m384 rv;
    rv.lo = and128(a.lo, b.lo);
    rv.mid = and128(a.mid, b.mid);
    rv.hi = and128(a.hi, b.hi);
    return rv;
}

static really_inline m384 or384(m384 a, m384 b) {
    m384 rv;
    rv.lo = or128(a.lo, b.lo);
    rv.mid = or128(a.mid, b.mid);
    rv.hi = or128(a.hi, b.hi);
    return rv;
}

static really_inline m384 xor384(m384 a, m384 b) {
    m384 rv;
    rv.lo = xor128(a.lo, b.lo);
    rv.mid = xor128(a.mid, b.mid);
    rv.hi = xor128(a.hi, b.hi);
    return rv;
}
static really_inline m384 not384(m384 a) {
    m384 rv;
    rv.lo = not128(a.lo);
    rv.mid = not128(a.mid);
    rv.hi = not128(a.hi);
    return rv;
}
static really_inline m384 andnot384(m384 a, m384 b) {
    m384 rv;
    rv.lo = andnot128(a.lo, b.lo);
    rv.mid = andnot128(a.mid, b.mid);
    rv.hi = andnot128(a.hi, b.hi);
    return rv;
}

static really_really_inline
m384 lshift64_m384(m384 a, unsigned b) {
    m384 rv;
    rv.lo = lshift64_m128(a.lo, b);
    rv.mid = lshift64_m128(a.mid, b);
    rv.hi = lshift64_m128(a.hi, b);
    return rv;
}

static really_inline m384 zeroes384(void) {
    m384 rv = {zeroes128(), zeroes128(), zeroes128()};
    return rv;
}

static really_inline m384 ones384(void) {
    m384 rv = {ones128(), ones128(), ones128()};
    return rv;
}

static really_inline int diff384(m384 a, m384 b) {
    return diff128(a.lo, b.lo) || diff128(a.mid, b.mid) || diff128(a.hi, b.hi);
}

static really_inline int isnonzero384(m384 a) {
    return isnonzero128(or128(or128(a.lo, a.mid), a.hi));
}

#if defined(HAVE_SIMD_128_BITS) && !defined(ARCH_IA32) && !defined(ARCH_X86_64)
/**
 * "Rich" version of diff384(). Takes two vectors a and b and returns a 12-bit
 * mask indicating which 32-bit words contain differences.
 */
static really_inline
u32 diffrich384(m384 a, m384 b) {
    return diffrich128(a.lo, b.lo) | (diffrich128(a.mid, b.mid) << 4) | (diffrich128(a.hi, b.hi) << 8);
}
#endif

/**
 * "Rich" version of diff384(), 64-bit variant. Takes two vectors a and b and
 * returns a 12-bit mask indicating which 64-bit words contain differences.
 */
static really_inline u32 diffrich64_384(m384 a, m384 b) {
    u32 d = diffrich384(a, b);
    return (d | (d >> 1)) & 0x55555555;
}

// aligned load
static really_inline m384 load384(const void *ptr) {
    assert(ISALIGNED_16(ptr));
    m384 rv = { load128(ptr), load128((const char *)ptr + 16),
                load128((const char *)ptr + 32) };
    return rv;
}

// aligned store
static really_inline void store384(void *ptr, m384 a) {
    assert(ISALIGNED_16(ptr));
    ptr = assume_aligned(ptr, 16);
    *(m384 *)ptr = a;
}

// unaligned load
static really_inline m384 loadu384(const void *ptr) {
    m384 rv = { loadu128(ptr), loadu128((const char *)ptr + 16),
                loadu128((const char *)ptr + 32)};
    return rv;
}

// packed unaligned store of first N bytes
static really_inline
void storebytes384(void *ptr, m384 a, unsigned int n) {
    assert(n <= sizeof(a));
    memcpy(ptr, &a, n);
}

// packed unaligned load of first N bytes, pad with zero
static really_inline
m384 loadbytes384(const void *ptr, unsigned int n) {
    m384 a = zeroes384();
    assert(n <= sizeof(a));
    memcpy(&a, ptr, n);
    return a;
}

// switches on bit N in the given vector.
static really_inline
void setbit384(m384 *ptr, unsigned int n) {
    assert(n < sizeof(*ptr) * 8);
    m128 *sub;
    if (n < 128) {
        sub = &ptr->lo;
    } else if (n < 256) {
        sub = &ptr->mid;
    } else {
        sub = &ptr->hi;
    }
    setbit128(sub, n % 128);
}

// switches off bit N in the given vector.
static really_inline
void clearbit384(m384 *ptr, unsigned int n) {
    assert(n < sizeof(*ptr) * 8);
    m128 *sub;
    if (n < 128) {
        sub = &ptr->lo;
    } else if (n < 256) {
        sub = &ptr->mid;
    } else {
        sub = &ptr->hi;
    }
    clearbit128(sub, n % 128);
}

// tests bit N in the given vector.
static really_inline
char testbit384(m384 val, unsigned int n) {
    assert(n < sizeof(val) * 8);
    m128 sub;
    if (n < 128) {
        sub = val.lo;
    } else if (n < 256) {
        sub = val.mid;
    } else {
        sub = val.hi;
    }
    return testbit128(sub, n % 128);
}


/****
 **** 512-bit Primitives
 ****/

#if !defined(HAVE_SIMD_512_BITS)

static really_inline
m512 zeroes512(void) {
    m512 rv = {zeroes256(), zeroes256()};
    return rv;
}

static really_inline
m512 ones512(void) {
    m512 rv = {ones256(), ones256()};
    return rv;
}

static really_inline
m512 set1_64x8(u8 a) {
    m256 a256 = set1_32x8(a);
    m512 rv = {a256, a256};
    return rv;
}

static really_inline
m512 set1_8x64(u64a a) {
    m256 a256 = set1_4x64(a);
    m512 rv = {a256, a256};
    return rv;
}

static really_inline
m512 set8x64(u64a hi_3, u64a hi_2, u64a hi_1, u64a hi_0,
               u64a lo_3, u64a lo_2, u64a lo_1, u64a lo_0) {
    m512 rv;
    rv.lo = set4x64(lo_3, lo_2, lo_1, lo_0);
    rv.hi = set4x64(hi_3, hi_2, hi_1, hi_0);
    return rv;
}
/*
static really_inline
m512 swap256in512(m512 a) {
    m512 idx = set8x64(3ULL, 2ULL, 1ULL, 0ULL, 7ULL, 6ULL, 5ULL, 4ULL);
    return vpermq512(idx, a);
}*/

static really_inline
m512 set1_4x128(m128 a) {
    m256 a256 = set1_2x128(a);
    m512 rv = {a256, a256};
    return rv;
}


static really_inline
m512 and512(m512 a, m512 b) {
    m512 rv;
    rv.lo = and256(a.lo, b.lo);
    rv.hi = and256(a.hi, b.hi);
    return rv;
}

static really_inline
m512 or512(m512 a, m512 b) {
    m512 rv;
    rv.lo = or256(a.lo, b.lo);
    rv.hi = or256(a.hi, b.hi);
    return rv;
}

static really_inline
m512 xor512(m512 a, m512 b) {
    m512 rv;
    rv.lo = xor256(a.lo, b.lo);
    rv.hi = xor256(a.hi, b.hi);
    return rv;
}

static really_inline
m512 not512(m512 a) {
    m512 rv;
    rv.lo = not256(a.lo);
    rv.hi = not256(a.hi);
    return rv;
}

static really_inline
m512 andnot512(m512 a, m512 b) {
    m512 rv;
    rv.lo = andnot256(a.lo, b.lo);
    rv.hi = andnot256(a.hi, b.hi);
    return rv;
}

static really_really_inline
m512 lshift64_m512(m512 a, unsigned b) {
    m512 rv;
    rv.lo = lshift64_m256(a.lo, b);
    rv.hi = lshift64_m256(a.hi, b);
    return rv;
}

static really_inline
int diff512(m512 a, m512 b) {
    return diff256(a.lo, b.lo) || diff256(a.hi, b.hi);
}

static really_inline
int isnonzero512(m512 a) {
    m256 x = or256(a.lo, a.lo);
    m256 y = or256(a.hi, a.hi);
    return isnonzero256(or256(x, y));
}

/**
 * "Rich" version of diff512(). Takes two vectors a and b and returns a 16-bit
 * mask indicating which 32-bit words contain differences.
 */
static really_inline
u32 diffrich512(m512 a, m512 b) {
    return diffrich256(a.lo, b.lo) | (diffrich256(a.hi, b.hi) << 8);
}

/**
 * "Rich" version of diffrich(), 64-bit variant. Takes two vectors a and b and
 * returns a 16-bit mask indicating which 64-bit words contain differences.
 */
static really_inline
u32 diffrich64_512(m512 a, m512 b) {
    //TODO: cmp_epi64?
    u32 d = diffrich512(a, b);
    return (d | (d >> 1)) & 0x55555555;
}

// aligned load
static really_inline
m512 load512(const void *ptr) {
    assert(ISALIGNED_N(ptr, alignof(m256)));
    m512 rv = { load256(ptr), load256((const char *)ptr + 32) };
    return rv;
}

// aligned store
static really_inline
void store512(void *ptr, m512 a) {
    assert(ISALIGNED_N(ptr, alignof(m512)));
    m512 *x = (m512 *)ptr;
    store256(&x->lo, a.lo);
    store256(&x->hi, a.hi);
}

// unaligned load
static really_inline
m512 loadu512(const void *ptr) {
    m512 rv = { loadu256(ptr), loadu256((const char *)ptr + 32) };
    return rv;
}

/*static really_inline
m512 loadu_maskz_m512(__mmask64 k, const void *ptr) {
}

static really_inline
m512 loadu_mask_m512(m512 src, __mmask64 k, const void *ptr) {
}

static really_inline
m512 set_mask_m512(__mmask64 k) {
}*/

// packed unaligned store of first N bytes
static really_inline
void storebytes512(void *ptr, m512 a, unsigned int n) {
    assert(n <= sizeof(a));
    memcpy(ptr, &a, n);
}

// packed unaligned load of first N bytes, pad with zero
static really_inline
m512 loadbytes512(const void *ptr, unsigned int n) {
    m512 a = zeroes512();
    assert(n <= sizeof(a));
    memcpy(&a, ptr, n);
    return a;
}

static really_inline
m512 mask1bit512(unsigned int n) {
    assert(n < sizeof(m512) * 8);
    u32 mask_idx = ((n % 8) * 64) + 95;
    mask_idx -= n / 8;
    return loadu512(&simd_onebit_masks[mask_idx]);
}

// switches on bit N in the given vector.
static really_inline
void setbit512(m512 *ptr, unsigned int n) {
    assert(n < sizeof(*ptr) * 8);
    m256 *sub;
    if (n < 256) {
        sub = &ptr->lo;
    } else {
        sub = &ptr->hi;
        n -= 256;
    }
    setbit256(sub, n);
}

// switches off bit N in the given vector.
static really_inline
void clearbit512(m512 *ptr, unsigned int n) {
    assert(n < sizeof(*ptr) * 8);
    m256 *sub;
    if (n < 256) {
        sub = &ptr->lo;
    } else {
        sub = &ptr->hi;
        n -= 256;
    }
    clearbit256(sub, n);
}

// tests bit N in the given vector.
static really_inline
char testbit512(m512 val, unsigned int n) {
    assert(n < sizeof(val) * 8);
    m256 sub;
    if (n < 256) {
        sub = val.lo;
    } else {
        sub = val.hi;
        n -= 256;
    }
    return testbit256(sub, n);
}

#endif // HAVE_SIMD_512_BITS

#endif // ARCH_COMMON_SIMD_UTILS_H
