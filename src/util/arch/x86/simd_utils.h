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

#ifndef ARCH_X86_SIMD_UTILS_H
#define ARCH_X86_SIMD_UTILS_H

#include "x86.h"
#include "ue2common.h"
#include "util/simd_types.h"
#include "util/unaligned.h"
#include "util/intrinsics.h"

#include <string.h> // for memcpy

static really_inline m128 ones128(void) {
#if defined(__GNUC__) || defined(__INTEL_COMPILER)
    /* gcc gets this right */
    return _mm_set1_epi8(0xFF);
#else
    /* trick from Intel's optimization guide to generate all-ones.
     * ICC converts this to the single cmpeq instruction */
    return _mm_cmpeq_epi8(_mm_setzero_si128(), _mm_setzero_si128());
#endif
}

static really_inline m128 zeroes128(void) {
    return _mm_setzero_si128();
}

/** \brief Bitwise not for m128*/
static really_inline m128 not128(m128 a) {
    return _mm_xor_si128(a, ones128());
}

/** \brief Return 1 if a and b are different otherwise 0 */
static really_inline int diff128(m128 a, m128 b) {
    return (_mm_movemask_epi8(_mm_cmpeq_epi8(a, b)) ^ 0xffff);
}

static really_inline int isnonzero128(m128 a) {
    return !!diff128(a, zeroes128());
}

/**
 * "Rich" version of diff128(). Takes two vectors a and b and returns a 4-bit
 * mask indicating which 32-bit words contain differences.
 */
static really_inline u32 diffrich128(m128 a, m128 b) {
    a = _mm_cmpeq_epi32(a, b);
    return ~(_mm_movemask_ps(_mm_castsi128_ps(a))) & 0xf;
}

/**
 * "Rich" version of diff128(), 64-bit variant. Takes two vectors a and b and
 * returns a 4-bit mask indicating which 64-bit words contain differences.
 */
static really_inline u32 diffrich64_128(m128 a, m128 b) {
#if defined(HAVE_SSE41)
    a = _mm_cmpeq_epi64(a, b);
    return ~(_mm_movemask_ps(_mm_castsi128_ps(a))) & 0x5;
#else
    u32 d = diffrich128(a, b);
    return (d | (d >> 1)) & 0x5;
#endif
}

static really_really_inline
m128 lshift64_m128(m128 a, unsigned b) {
#if defined(HAVE__BUILTIN_CONSTANT_P)
    if (__builtin_constant_p(b)) {
        return _mm_slli_epi64(a, b);
    }
#endif
    m128 x = _mm_cvtsi32_si128(b);
    return _mm_sll_epi64(a, x);
}

#define rshift64_m128(a, b) _mm_srli_epi64((a), (b))
#define eq128(a, b)      _mm_cmpeq_epi8((a), (b))
#define movemask128(a)  ((u32)_mm_movemask_epi8((a)))

static really_inline m128 set1_16x8(u8 c) {
    return _mm_set1_epi8(c);
}

static really_inline m128 set1_4x32(u32 c) {
    return _mm_set1_epi32(c);
}

static really_inline m128 set1_2x64(u64a c) {
    return _mm_set1_epi64x(c);
}

static really_inline u32 movd(const m128 in) {
    return _mm_cvtsi128_si32(in);
}

static really_inline u64a movq(const m128 in) {
    return _mm_cvtsi128_si64(in);
}

/* another form of movq */
static really_inline
m128 load_m128_from_u64a(const u64a *p) {
    return _mm_set_epi64x(0LL, *p);
}

#define rshiftbyte_m128(a, count_immed) _mm_srli_si128(a, count_immed)
#define lshiftbyte_m128(a, count_immed) _mm_slli_si128(a, count_immed)

#if defined(HAVE_SSE41)
#define extract32from128(a, imm) _mm_extract_epi32(a, imm)
#define extract64from128(a, imm) _mm_extract_epi64(a, imm)
#else
#define extract32from128(a, imm) movd(_mm_srli_si128(a, imm << 2))
#define extract64from128(a, imm) movq(_mm_srli_si128(a, imm << 3))
#endif

#if !defined(HAVE_AVX2)
// TODO: this entire file needs restructuring - this carveout is awful
#define extractlow64from256(a) movq(a.lo)
#define extractlow32from256(a) movd(a.lo)
#if defined(HAVE_SSE41)
#define extract32from256(a, imm) _mm_extract_epi32((imm >> 2) ? a.hi : a.lo, imm % 4)
#define extract64from256(a, imm) _mm_extract_epi64((imm >> 1) ? a.hi : a.lo, imm % 2)
#else
#define extract32from256(a, imm) movd(_mm_srli_si128((imm >> 2) ? a.hi : a.lo, (imm % 4) * 4))
#define extract64from256(a, imm) movq(_mm_srli_si128((imm >> 1) ? a.hi : a.lo, (imm % 2) * 8))
#endif

#endif // !AVX2

static really_inline m128 and128(m128 a, m128 b) {
    return _mm_and_si128(a,b);
}

static really_inline m128 xor128(m128 a, m128 b) {
    return _mm_xor_si128(a,b);
}

static really_inline m128 or128(m128 a, m128 b) {
    return _mm_or_si128(a,b);
}

static really_inline m128 andnot128(m128 a, m128 b) {
    return _mm_andnot_si128(a, b);
}

// aligned load
static really_inline m128 load128(const void *ptr) {
    assert(ISALIGNED_N(ptr, alignof(m128)));
    ptr = assume_aligned(ptr, 16);
    return _mm_load_si128((const m128 *)ptr);
}

// aligned store
static really_inline void store128(void *ptr, m128 a) {
    assert(ISALIGNED_N(ptr, alignof(m128)));
    ptr = assume_aligned(ptr, 16);
    *(m128 *)ptr = a;
}

// unaligned load
static really_inline m128 loadu128(const void *ptr) {
    return _mm_loadu_si128((const m128 *)ptr);
}

// unaligned store
static really_inline void storeu128(void *ptr, m128 a) {
    _mm_storeu_si128 ((m128 *)ptr, a);
}

// packed unaligned store of first N bytes
static really_inline
void storebytes128(void *ptr, m128 a, unsigned int n) {
    assert(n <= sizeof(a));
    memcpy(ptr, &a, n);
}

// packed unaligned load of first N bytes, pad with zero
static really_inline
m128 loadbytes128(const void *ptr, unsigned int n) {
    m128 a = zeroes128();
    assert(n <= sizeof(a));
    memcpy(&a, ptr, n);
    return a;
}

#ifdef __cplusplus
extern "C" {
#endif
extern const u8 simd_onebit_masks[];
#ifdef __cplusplus
}
#endif

static really_inline
m128 mask1bit128(unsigned int n) {
    assert(n < sizeof(m128) * 8);
    u32 mask_idx = ((n % 8) * 64) + 95;
    mask_idx -= n / 8;
    return loadu128(&simd_onebit_masks[mask_idx]);
}

// switches on bit N in the given vector.
static really_inline
void setbit128(m128 *ptr, unsigned int n) {
    *ptr = or128(mask1bit128(n), *ptr);
}

// switches off bit N in the given vector.
static really_inline
void clearbit128(m128 *ptr, unsigned int n) {
    *ptr = andnot128(mask1bit128(n), *ptr);
}

// tests bit N in the given vector.
static really_inline
char testbit128(m128 val, unsigned int n) {
    const m128 mask = mask1bit128(n);
#if defined(HAVE_SSE41)
    return !_mm_testz_si128(mask, val);
#else
    return isnonzero128(and128(mask, val));
#endif
}

// offset must be an immediate
#define palignr(r, l, offset) _mm_alignr_epi8(r, l, offset)

static really_inline
m128 pshufb_m128(m128 a, m128 b) {
    m128 result;
    result = _mm_shuffle_epi8(a, b);
    return result;
}

static really_inline
m128 variable_byte_shift_m128(m128 in, s32 amount) {
    assert(amount >= -16 && amount <= 16);
    m128 shift_mask = loadu128(vbs_mask_data + 16 - amount);
    return pshufb_m128(in, shift_mask);
}

static really_inline
m128 max_u8_m128(m128 a, m128 b) {
    return _mm_max_epu8(a, b);
}

static really_inline
m128 min_u8_m128(m128 a, m128 b) {
    return _mm_min_epu8(a, b);
}

static really_inline
m128 sadd_u8_m128(m128 a, m128 b) {
    return _mm_adds_epu8(a, b);
}

static really_inline
m128 sub_u8_m128(m128 a, m128 b) {
    return _mm_sub_epi8(a, b);
}

static really_inline
m128 set4x32(u32 x3, u32 x2, u32 x1, u32 x0) {
    return _mm_set_epi32(x3, x2, x1, x0);
}

static really_inline
m128 set2x64(u64a hi, u64a lo) {
    return _mm_set_epi64x(hi, lo);
}

/****
 **** 256-bit Primitives
 ****/

#if defined(HAVE_SIMD_256_BITS) && defined(HAVE_AVX2)

static really_inline
m256 pshufb_m256(m256 a, m256 b) {
    return _mm256_shuffle_epi8(a, b);
}

static really_really_inline
m256 lshift64_m256(m256 a, unsigned b) {
#if defined(HAVE__BUILTIN_CONSTANT_P)
    if (__builtin_constant_p(b)) {
        return _mm256_slli_epi64(a, b);
    }
#endif
    m128 x = _mm_cvtsi32_si128(b);
    return _mm256_sll_epi64(a, x);
}

#define rshift64_m256(a, b) _mm256_srli_epi64((a), (b))

static really_inline m256 set1_4x64(u64a c) {
    return _mm256_set1_epi64x(c);
}

#define eq256(a, b)     _mm256_cmpeq_epi8((a), (b))
#define movemask256(a)  ((u32)_mm256_movemask_epi8((a)))

static really_inline
m256 set1_2x128(m128 a) {
    return _mm256_broadcastsi128_si256(a);
}

static really_inline m256 zeroes256(void) {
    return _mm256_setzero_si256();
}

static really_inline m256 ones256(void) {
    m256 rv = _mm256_set1_epi8(0xFF);
    return rv;
}

static really_inline m256 and256(m256 a, m256 b) {
    return _mm256_and_si256(a, b);
}

static really_inline m256 or256(m256 a, m256 b) {
    return _mm256_or_si256(a, b);
}

static really_inline m256 xor256(m256 a, m256 b) {
    return _mm256_xor_si256(a, b);
}

static really_inline m256 not256(m256 a) {
    return _mm256_xor_si256(a, ones256());
}

static really_inline m256 andnot256(m256 a, m256 b) {
    return _mm256_andnot_si256(a, b);
}

static really_inline int diff256(m256 a, m256 b) {
    return !!(_mm256_movemask_epi8(_mm256_cmpeq_epi8(a, b)) ^ (int)-1);
}

static really_inline int isnonzero256(m256 a) {
    return !!diff256(a, zeroes256());
}

/**
 * "Rich" version of diff256(). Takes two vectors a and b and returns an 8-bit
 * mask indicating which 32-bit words contain differences.
 */
static really_inline u32 diffrich256(m256 a, m256 b) {
    a = _mm256_cmpeq_epi32(a, b);
    return ~(_mm256_movemask_ps(_mm256_castsi256_ps(a))) & 0xFF;
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
    return _mm256_load_si256((const m256 *)ptr);
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
    _mm256_store_si256((m256 *)ptr, a);
}

// unaligned load
static really_inline m256 loadu256(const void *ptr) {
    return _mm256_loadu_si256((const m256 *)ptr);
}

// unaligned store
static really_inline void storeu256(void *ptr, m256 a) {
    _mm256_storeu_si256((m256 *)ptr, a);
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
    return _mm256_set1_epi8(in);
}

static really_inline
m256 set8x32(u32 hi_3, u32 hi_2, u32 hi_1, u32 hi_0, u32 lo_3, u32 lo_2, u32 lo_1, u32 lo_0) {
    return _mm256_set_epi32(hi_3, hi_2, hi_1, hi_0, lo_3, lo_2, lo_1, lo_0);
}

static really_inline
m256 set4x64(u64a hi_1, u64a hi_0, u64a lo_1, u64a lo_0) {
    return _mm256_set_epi64x(hi_1, hi_0, lo_1, lo_0);
}

// switches on bit N in the given vector.
static really_inline
void setbit256(m256 *ptr, unsigned int n) {
    *ptr = or256(mask1bit256(n), *ptr);
}

static really_inline
void clearbit256(m256 *ptr, unsigned int n) {
    *ptr = andnot256(mask1bit256(n), *ptr);
}

// tests bit N in the given vector.
static really_inline
char testbit256(m256 val, unsigned int n) {
    const m256 mask = mask1bit256(n);
    return !_mm256_testz_si256(mask, val);
}

static really_really_inline
m128 movdq_hi(m256 x) {
    return _mm256_extracti128_si256(x, 1);
}

static really_really_inline
m128 movdq_lo(m256 x) {
    return _mm256_extracti128_si256(x, 0);
}

#define cast256to128(a) _mm256_castsi256_si128(a)
#define cast128to256(a) _mm256_castsi128_si256(a)
#define swap128in256(a) _mm256_permute4x64_epi64(a, 0x4E)
#define insert128to256(a, b, imm) _mm256_inserti128_si256(a, b, imm)
#define rshift128_m256(a, count_immed) _mm256_srli_si256(a, count_immed)
#define lshift128_m256(a, count_immed) _mm256_slli_si256(a, count_immed)
#define extract64from256(a, imm) _mm_extract_epi64(_mm256_extracti128_si256(a, imm >> 1), imm % 2)
#define extract32from256(a, imm) _mm_extract_epi32(_mm256_extracti128_si256(a, imm >> 2), imm % 4)
#define extractlow64from256(a) _mm_cvtsi128_si64(cast256to128(a))
#define extractlow32from256(a) movd(cast256to128(a))
#define interleave256hi(a, b) _mm256_unpackhi_epi8(a, b)
#define interleave256lo(a, b) _mm256_unpacklo_epi8(a, b)
#define vpalignr(r, l, offset) _mm256_alignr_epi8(r, l, offset)

static really_inline
m256 combine2x128(m128 hi, m128 lo) {
#if defined(_mm256_set_m128i)
    return _mm256_set_m128i(hi, lo);
#else
    return insert128to256(cast128to256(lo), hi, 1);
#endif
}
#endif //AVX2

#if defined(HAVE_SIMD_128_BITS)
/**
 * "Rich" version of diff384(). Takes two vectors a and b and returns a 12-bit
 * mask indicating which 32-bit words contain differences.
 */

static really_inline u32 diffrich384(m384 a, m384 b) {
    m128 z = zeroes128();
    a.lo = _mm_cmpeq_epi32(a.lo, b.lo);
    a.mid = _mm_cmpeq_epi32(a.mid, b.mid);
    a.hi = _mm_cmpeq_epi32(a.hi, b.hi);
    m128 packed = _mm_packs_epi16(_mm_packs_epi32(a.lo, a.mid),
                                  _mm_packs_epi32(a.hi, z));
    return ~(_mm_movemask_epi8(packed)) & 0xfff;
}

#endif // HAVE_SIMD_128_BITS

/****
 **** 512-bit Primitives
 ****/

#if defined(HAVE_SIMD_512_BITS)

#define extract128from512(a, imm) _mm512_extracti32x4_epi32(a, imm)
#define interleave512hi(a, b) _mm512_unpackhi_epi8(a, b)
#define interleave512lo(a, b) _mm512_unpacklo_epi8(a, b)
#define set2x256(a) _mm512_broadcast_i64x4(a)
#define mask_set2x256(src, k, a) _mm512_mask_broadcast_i64x4(src, k, a)
#define vpermq512(idx, a) _mm512_permutexvar_epi64(idx, a)

static really_inline u32 movd512(const m512 in) {
    // NOTE: seems gcc doesn't support _mm512_cvtsi512_si32(in),
    //       so we use 2-step convertions to work around.
    return _mm_cvtsi128_si32(_mm512_castsi512_si128(in));
}

static really_inline
m512 pshufb_m512(m512 a, m512 b) {
    return _mm512_shuffle_epi8(a, b);
}

static really_inline
m512 maskz_pshufb_m512(__mmask64 k, m512 a, m512 b) {
    return _mm512_maskz_shuffle_epi8(k, a, b);
}

#if defined(HAVE_AVX512VBMI)
#define vpermb512(idx, a) _mm512_permutexvar_epi8(idx, a)
#define maskz_vpermb512(k, idx, a) _mm512_maskz_permutexvar_epi8(k, idx, a)
#endif

#define eq512mask(a, b) _mm512_cmpeq_epi8_mask((a), (b))
#define masked_eq512mask(k, a, b) _mm512_mask_cmpeq_epi8_mask((k), (a), (b))

static really_inline
m512 zeroes512(void) {
#if defined(HAVE_AVX512)
    return _mm512_setzero_si512();
#else
    m512 rv = {zeroes256(), zeroes256()};
    return rv;
#endif
}

static really_inline
m512 ones512(void) {
    return _mm512_set1_epi8(0xFF);
    //return _mm512_xor_si512(_mm512_setzero_si512(), _mm512_setzero_si512());
}

static really_inline
m512 set1_64x8(u8 a) {
    return _mm512_set1_epi8(a);
}

static really_inline
m512 set1_8x64(u64a a) {
    return _mm512_set1_epi64(a);
}

static really_inline
m512 set8x64(u64a hi_3, u64a hi_2, u64a hi_1, u64a hi_0,
               u64a lo_3, u64a lo_2, u64a lo_1, u64a lo_0) {
    return _mm512_set_epi64(hi_3, hi_2, hi_1, hi_0,
                            lo_3, lo_2, lo_1, lo_0);
}

static really_inline
m512 swap256in512(m512 a) {
    m512 idx = set512_64(3ULL, 2ULL, 1ULL, 0ULL, 7ULL, 6ULL, 5ULL, 4ULL);
    return vpermq512(idx, a);
}

static really_inline
m512 set1_4x128(m128 a) {
    return _mm512_broadcast_i32x4(a);
}

static really_inline
m512 and512(m512 a, m512 b) {
    return _mm512_and_si512(a, b);
}

static really_inline
m512 or512(m512 a, m512 b) {
    return _mm512_or_si512(a, b);
}

static really_inline
m512 xor512(m512 a, m512 b) {
    return _mm512_xor_si512(a, b);
}

static really_inline
m512 not512(m512 a) {
    return _mm512_xor_si512(a, ones512());
}

static really_inline
m512 andnot512(m512 a, m512 b) {
    return _mm512_andnot_si512(a, b);
}

static really_really_inline
m512 lshift64_m512(m512 a, unsigned b) {
#if defined(HAVE__BUILTIN_CONSTANT_P)
    if (__builtin_constant_p(b)) {
        return _mm512_slli_epi64(a, b);
    }
#endif
    m128 x = _mm_cvtsi32_si128(b);
    return _mm512_sll_epi64(a, x);
}

#define rshift64_m512(a, b) _mm512_srli_epi64((a), (b))
#define rshift128_m512(a, count_immed) _mm512_bsrli_epi128(a, count_immed)
#define lshift128_m512(a, count_immed) _mm512_bslli_epi128(a, count_immed)

#if !defined(_MM_CMPINT_NE)
#define _MM_CMPINT_NE 0x4
#endif

static really_inline
int diff512(m512 a, m512 b) {
    return !!_mm512_cmp_epi8_mask(a, b, _MM_CMPINT_NE);
}

static really_inline
int isnonzero512(m512 a) {
    return diff512(a, zeroes512());
}

/**
 * "Rich" version of diff512(). Takes two vectors a and b and returns a 16-bit
 * mask indicating which 32-bit words contain differences.
 */
static really_inline
u32 diffrich512(m512 a, m512 b) {
    return _mm512_cmp_epi32_mask(a, b, _MM_CMPINT_NE);
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
    return _mm512_load_si512(ptr);
}

// aligned store
static really_inline
void store512(void *ptr, m512 a) {
    assert(ISALIGNED_N(ptr, alignof(m512)));
    return _mm512_store_si512(ptr, a);
}

// unaligned load
static really_inline
m512 loadu512(const void *ptr) {
    return _mm512_loadu_si512(ptr);
}

static really_inline
m512 loadu_maskz_m512(__mmask64 k, const void *ptr) {
    return _mm512_maskz_loadu_epi8(k, ptr);
}

static really_inline
m512 loadu_mask_m512(m512 src, __mmask64 k, const void *ptr) {
    return _mm512_mask_loadu_epi8(src, k, ptr);
}

static really_inline
m512 set_mask_m512(__mmask64 k) {
    return _mm512_movm_epi8(k);
}

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
    *ptr = or512(mask1bit512(n), *ptr);
}

// switches off bit N in the given vector.
static really_inline
void clearbit512(m512 *ptr, unsigned int n) {
    assert(n < sizeof(*ptr) * 8);
    *ptr = andnot512(mask1bit512(n), *ptr);
}

// tests bit N in the given vector.
static really_inline
char testbit512(m512 val, unsigned int n) {
    assert(n < sizeof(val) * 8);
    const m512 mask = mask1bit512(n);
    return !!_mm512_test_epi8_mask(mask, val);
}

#endif // HAVE_SIMD_512_BITS

#endif // ARCH_X86_SIMD_UTILS_H
