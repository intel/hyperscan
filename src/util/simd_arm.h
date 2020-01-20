/*
 * Copyright (c) 2015-2017, Intel Corporation
 * 2020.01 - Use the neon instruction to implement the function of 128-bit operation.
 *           Huawei Technologies Co., Ltd.
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

#ifndef SIMD_ARM
#define SIMD_ARM

#include "config.h"
#include "simd_types.h"
#include "ue2common.h"
#include "unaligned.h"
#include "util/arch.h"
#include "util/intrinsics.h"

#include <string.h> // for memcpy

// Define a common assume_aligned using an appropriate compiler built-in, if
// it's available. Note that we need to handle C or C++ compilation.
#ifdef __cplusplus
#ifdef HAVE_CXX_BUILTIN_ASSUME_ALIGNED
#define assume_aligned(x, y) __builtin_assume_aligned((x), (y))
#endif
#else
#ifdef HAVE_CC_BUILTIN_ASSUME_ALIGNED
#define assume_aligned(x, y) __builtin_assume_aligned((x), (y))
#endif
#endif

// Fallback to identity case.
#ifndef assume_aligned
#define assume_aligned(x, y) (x)
#endif

#ifdef __cplusplus
extern "C" {
#endif
extern const char vbs_mask_data[];
#ifdef __cplusplus
}
#endif

/*
** extend 4.8.5 neon inline assembly functions
*/
__extension__ static __inline uint64x2_t __attribute__((__always_inline__))
vmvnq_u64(uint64x2_t a) {
    uint64x2_t result;
    __asm__("mvn %0.16b,%1.16b" : "=w"(result) : "w"(a) : /* No clobbers */);
    return result;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"

static really_inline m128 ones128(void) {
    m128 result;
    result.vect_s32 = vdupq_n_s32(0xFFFFFFFF);
    return result;
}

static really_inline m128 zeroes128(void) {
    m128 result;
    result.vect_s32 = vdupq_n_s32(0x0);
    return result;
}

/** \brief Return 1 if a and b are different otherwise 0 */
static really_inline int diff128(m128 a, m128 b) {
    return !!vaddlvq_s16(veorq_s16(a.vect_s16, b.vect_s16));
}

static really_inline int isnonzero128(m128 a) {
    return !!diff128(a, zeroes128());
}

/**
 * "Rich" version of diff128(). Takes two vectors a and b and returns a 4-bit
 * mask indicating which 32-bit words contain differences.
 */
static really_inline u32 diffrich128(m128 a, m128 b) {
    m128 tmp;
    tmp.vect_u32 = vmvnq_u32(vceqq_u32(a.vect_u32, b.vect_u32));
    return ((vgetq_lane_u32(tmp.vect_u32, 3) & 0x8) |
            (vgetq_lane_u32(tmp.vect_u32, 2) & 0x4) |
            (vgetq_lane_u32(tmp.vect_u32, 1) & 0x2) |
            (vgetq_lane_u32(tmp.vect_u32, 0) & 0x1));
}

/**
 * "Rich" version of diff128(), 64-bit variant. Takes two vectors a and b and
 * returns a 4-bit mask indicating which 64-bit words contain differences.
 */
static really_inline u32 diffrich64_128(m128 a, m128 b) {
    m128 tmp;
    tmp.vect_u64 = vmvnq_u64(vceqq_u64(a.vect_u64, b.vect_u64));
    return (u32)((vgetq_lane_u64(tmp.vect_u64, 1) & 0x4) |
                 (vgetq_lane_u64(tmp.vect_u64, 0) & 0x1));
}

static really_really_inline m128 lshift64_m128(m128 a, unsigned b) {
    assert(b <= 63);
    m128 result;
    result.vect_s64 = vshlq_n_s64(a.vect_s64, b);
    return result;
}

static really_really_inline m128 rshift64_m128(m128 a, int imm8) {
    assert(imm8 >= 0 && imm8 <= 63);
    if (unlikely(imm8 == 0)) {
        return a;
    }
    m128 result;
    result.vect_u64 = vshrq_n_u64(a.vect_u64, imm8);
    return result;
}

static really_really_inline m128 eq128(m128 a, m128 b) {
    m128 result;
    result.vect_u8 = vceqq_s8(a.vect_s8, b.vect_s8);
    return result;
}

static really_really_inline u32 movemask128(m128 a) {
    m128 result;
    result.vect_u8 = vshrq_n_u8(a.vect_u8, 7);
    result.vect_u16 = vsraq_n_u16(result.vect_u16, result.vect_u16, 7);
    result.vect_u32 = vsraq_n_u32(result.vect_u32, result.vect_u32, 14);
    result.vect_u64 = vsraq_n_u64(result.vect_u64, result.vect_u64, 28);
    return (u32)(vgetq_lane_u8(result.vect_u8, 0) |
                 ((u32)vgetq_lane_u8(result.vect_u8, 8) << 8));
}

static really_really_inline m128 rshiftbyte_m128(m128 a, int imm8) {
    assert(imm8 >= 0 && imm8 <= 15);
    m128 result;
    result.vect_s8 = vextq_s8(a.vect_s8, vdupq_n_s8(0), imm8);
    return result;
}

static really_really_inline m128 lshiftbyte_m128(m128 a, int imm8) {
    assert(imm8 >= 0 && imm8 <= 15);
    m128 result;
    if (unlikely(imm8 == 0)) {
        return a;
    }
    result.vect_s8 = vextq_s8(vdupq_n_s8(0), a.vect_s8, (16 - imm8));
    return result;
}

static really_inline m128 set16x8(u8 c) {
    m128 result;
    result.vect_s8 = vdupq_n_s8(c);
    return result;
}

static really_inline m128 set4x32(u32 c) {
    m128 result;
    result.vect_s32 = vdupq_n_s32(c);
    return result;
}

static really_inline m128 set2x64(u64a c) {
    m128 result;
    result.vect_u64 = vdupq_n_u64(c);
    return result;
}

static really_inline u32 movd(const m128 in) {
    u32 result;
    result = vgetq_lane_u32(in.vect_u32, 0);
    return result;
}

static really_inline u64a movq(const m128 in) {
    return vgetq_lane_u64(in.vect_u64, 0);
}

/* another form of movq */
static really_inline m128 load_m128_from_u64a(const u64a *p) {
    m128 result;
    __asm__ __volatile__("ldr %d0, %1         \n\t"
                         : "=w"(result)
                         : "Utv"(*p)
                         : /* No clobbers */
    );
    return result;
}

/*The x86 platform does not perform the lower 2 bit operation.
If the value of imm exceeds 2 bit, a compilation error occurs.*/
static really_inline u32 extract32from128(m128 a, int imm) {
    return vgetq_lane_s32(a.vect_s32, imm & 0x0003);
}

/*The x86 platform does not perform the lower 1 bit operation.
If the value of imm exceeds 1 bit, a compilation error occurs.*/
static really_inline u64a extract64from128(m128 a, int imm) {
    return vgetq_lane_s64(a.vect_s64, imm & 0x0001);
}

#define extractlow64from256(a) movq(a.lo)
#define extractlow32from256(a) movd(a.lo)

/*The x86 platform does not perform the lower 2 bit operation.
If the value of imm exceeds 2 bit, a compilation error occurs.*/
static really_inline u32 extract32from256(m256 a, int imm) {
    return vgetq_lane_s32((imm >> 2) ? a.hi.vect_s32 : a.lo.vect_s32,
                          imm & 0x0003);
}

/*The x86 platform does not perform the lower 1 bit operation.
If the value of imm exceeds 1 bit, a compilation error occurs.*/
static really_inline u64a extract64from256(m256 a, int imm) {
    return vgetq_lane_s64((imm >> 1) ? a.hi.vect_s64 : a.lo.vect_s64,
                          imm & 0x0001);
}

static really_inline m128 and128(m128 a, m128 b) {
    m128 result;
    result.vect_s32 = vandq_s32(a.vect_s32, b.vect_s32);
    return result;
}

static really_inline m128 not128(m128 a) {
    m128 result;
    result.vect_s32 = vmvnq_s32(a.vect_s32);
    return result;
}

static really_inline m128 xor128(m128 a, m128 b) {
    m128 result;
    result.vect_s32 = veorq_s32(a.vect_s32, b.vect_s32);
    return result;
}

static really_inline m128 or128(m128 a, m128 b) {
    m128 result;
    result.vect_s32 = vorrq_s32(a.vect_s32, b.vect_s32);
    return result;
}

static really_inline m128 andnot128(m128 a, m128 b) {
    m128 result;
    result.vect_s32 = vbicq_s32(b.vect_s32, a.vect_s32);
    return result;
}

// aligned load
static really_inline m128 load128(const void *ptr) {
    assert(ISALIGNED_N(ptr, alignof(m128)));
    ptr = assume_aligned(ptr, 16);
    m128 result;
    result.vect_s32 = vld1q_s32((const int32_t *)ptr);
    return result;
}

// aligned store
static really_inline void store128(void *ptr, m128 a) {
    assert(ISALIGNED_N(ptr, alignof(m128)));
    ptr = assume_aligned(ptr, 16);
    *(m128 *)ptr = a;
}

// unaligned load
static really_inline m128 loadu128(const void *ptr) {
    m128 result;
    result.vect_s32 = vld1q_s32((const int32_t *)ptr);
    return result;
}

// unaligned store
static really_inline void storeu128(void *ptr, m128 a) {
    vst1q_s32((int32_t *)ptr, a.vect_s32);
}

// packed unaligned store of first N bytes
static really_inline void storebytes128(void *ptr, m128 a, unsigned int n) {
    assert(n <= sizeof(a));
    memcpy(ptr, &a, n);
}

// packed unaligned load of first N bytes, pad with zero
static really_inline m128 loadbytes128(const void *ptr, unsigned int n) {
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

static really_inline m128 mask1bit128(unsigned int n) {
    assert(n < sizeof(m128) * 8);
    u32 mask_idx = ((n % 8) * 64) + 95;
    mask_idx -= n / 8;
    return loadu128(&simd_onebit_masks[mask_idx]);
}

// switches on bit N in the given vector.
static really_inline void setbit128(m128 *ptr, unsigned int n) {
    *ptr = or128(mask1bit128(n), *ptr);
}

// switches off bit N in the given vector.
static really_inline void clearbit128(m128 *ptr, unsigned int n) {
    *ptr = andnot128(mask1bit128(n), *ptr);
}

// tests bit N in the given vector.
static really_inline char testbit128(m128 val, unsigned int n) {
    const m128 mask = mask1bit128(n);
    return isnonzero128(and128(mask, val));
}

// offset must be an immediate
/*The x86 platform does not perform the lower 8 bit operation.
If the value of imm exceeds 8 bit, a compilation error occurs.*/
static really_inline m128 palignr(m128 a, m128 b, int count) {
    m128 result;
    count = count & 0xff;
    if (likely(count < 16)) {
        result.vect_s8 = vextq_s8(b.vect_s8, a.vect_s8, count);
    } else if (count < 32) {
        result.vect_s8 = vextq_s8(a.vect_s8, vdupq_n_s8(0x0), count - 16);
    } else {
        result.vect_s32 = vdupq_n_s32(0);
    }
    return result;
}

static really_inline m128 pshufb_m128(m128 a, m128 b) {
    m128 result;
    __asm__ __volatile__("movi v3.16b, 0x8f                   \n\t"
                         "and v3.16b, v3.16b, %2.16b          \n\t"
                         "tbl %0.16b, {%1.16b}, v3.16b        \n\t"
                         : "=w"(result)
                         : "w"(a), "w"(b)
                         : "v3");
    return result;
}

static really_inline m256 pshufb_m256(m256 a, m256 b) {
    m256 rv;
    rv.lo = pshufb_m128(a.lo, b.lo);
    rv.hi = pshufb_m128(a.hi, b.hi);
    return rv;
}

static really_inline m128 variable_byte_shift_m128(m128 in, s32 amount) {
    assert(amount >= -16 && amount <= 16);
    m128 shift_mask = loadu128(vbs_mask_data + 16 - amount);
    return pshufb_m128(in, shift_mask);
}

static really_inline m128 max_u8_m128(m128 a, m128 b) {
    m128 result;
    result.vect_u8 = vmaxq_u8(a.vect_u8, b.vect_u8);
    return result;
}

static really_inline m128 min_u8_m128(m128 a, m128 b) {
    m128 result;
    result.vect_u8 = vminq_u8(a.vect_u8, b.vect_u8);
    return result;
}

static really_inline m128 sadd_u8_m128(m128 a, m128 b) {
    m128 result;
    result.vect_u8 = vqaddq_u8(a.vect_u8, b.vect_u8);
    return result;
}

static really_inline m128 sub_u8_m128(m128 a, m128 b) {
    m128 result;
    result.vect_u8 = vsubq_u8(a.vect_u8, b.vect_u8);
    return result;
}

static really_inline m128 set64x2(int64_t hi, int64_t lo) {
    m128 result;
    result.vect_s64 = vsetq_lane_s64(hi, vdupq_n_s64(lo), 1);
    return result;
}

static really_inline m128 set32x4(int i3, int i2, int i1, int i0) {
    m128 result;
    result.vect_s32 = vsetq_lane_s32(
        i3, vsetq_lane_s32(i2, vsetq_lane_s32(i1, vdupq_n_s32(i0), 1), 2), 3);
    return result;
}

/****
 **** 256-bit Primitives
 ****/

static really_really_inline m256 lshift64_m256(m256 a, int b) {
    m256 rv = a;
    rv.lo = lshift64_m128(rv.lo, b);
    rv.hi = lshift64_m128(rv.hi, b);
    return rv;
}

static really_inline m256 rshift64_m256(m256 a, int b) {
    m256 rv = a;
    rv.lo = rshift64_m128(rv.lo, b);
    rv.hi = rshift64_m128(rv.hi, b);
    return rv;
}
static really_inline m256 set32x8(u32 in) {
    m256 rv;
    rv.lo = set16x8((u8)in);
    rv.hi = rv.lo;
    return rv;
}

static really_inline m256 eq256(m256 a, m256 b) {
    m256 rv;
    rv.lo = eq128(a.lo, b.lo);
    rv.hi = eq128(a.hi, b.hi);
    return rv;
}

static really_inline u32 movemask256(m256 a) {
    u32 lo_mask = movemask128(a.lo);
    u32 hi_mask = movemask128(a.hi);
    return lo_mask | (hi_mask << 16);
}

static really_inline m256 set2x128(m128 a) {
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
 * "Rich" version of diff256(). Takes two vectors a and b and returns an 8-bit
 * mask indicating which 32-bit words contain differences.
 */
static really_inline u32 diffrich256(m256 a, m256 b) {
    uint32x4_t x = vceqq_s32(a.lo.vect_s32, b.lo.vect_s32);
    uint32x4_t y = vceqq_s32(a.hi.vect_s32, b.hi.vect_s32);
    uint8x8_t lo = vqmovn_u16(vcombine_u16(vqmovn_u32(x), vqmovn_u32(y)));

    static const int8_t __attribute__((aligned(16)))
    xr[8] = {-7, -6, -5, -4, -3, -2, -1, 0};
    uint8x8_t mask_and = vdup_n_u8(0x80);
    int8x8_t mask_shift = vld1_s8(xr);

    lo = vand_u8(lo, mask_and);
    lo = vshl_u8(lo, mask_shift);

    lo = vpadd_u8(lo, lo);
    lo = vpadd_u8(lo, lo);
    lo = vpadd_u8(lo, lo);

    return ~(lo[0] & 0xFF) & 0xff;
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
    m256 rv = {load128(ptr), load128((const char *)ptr + 16)};
    return rv;
}

// aligned load  of 128-bit value to low and high part of 256-bit value
static really_inline m256 load2x128(const void *ptr) {
    assert(ISALIGNED_N(ptr, alignof(m128)));
    m256 rv;
    rv.hi = rv.lo = load128(ptr);
    return rv;
}

static really_inline m256 loadu2x128(const void *ptr) {
    return set2x128(loadu128(ptr));
}

// aligned store
static really_inline void store256(void *ptr, m256 a) {
    assert(ISALIGNED_N(ptr, alignof(m256)));
    ptr = assume_aligned(ptr, 16);
    *(m256 *)ptr = a;
}

// unaligned load
static really_inline m256 loadu256(const void *ptr) {
    m256 rv = {loadu128(ptr), loadu128((const char *)ptr + 16)};
    return rv;
}

// unaligned store
static really_inline void storeu256(void *ptr, m256 a) {
    storeu128(ptr, a.lo);
    storeu128((char *)ptr + 16, a.hi);
}

// packed unaligned store of first N bytes
static really_inline void storebytes256(void *ptr, m256 a, unsigned int n) {
    assert(n <= sizeof(a));
    memcpy(ptr, &a, n);
}

// packed unaligned load of first N bytes, pad with zero
static really_inline m256 loadbytes256(const void *ptr, unsigned int n) {
    m256 a = zeroes256();
    assert(n <= sizeof(a));
    memcpy(&a, ptr, n);
    return a;
}

static really_inline m256 mask1bit256(unsigned int n) {
    assert(n < sizeof(m256) * 8);
    u32 mask_idx = ((n % 8) * 64) + 95;
    mask_idx -= n / 8;
    return loadu256(&simd_onebit_masks[mask_idx]);
}

static really_inline m256 set64x4(u64a hi_1, u64a hi_0, u64a lo_1, u64a lo_0) {
    m256 rv;
    rv.hi = set64x2(hi_1, hi_0);
    rv.lo = set64x2(lo_1, lo_0);
    return rv;
}

// switches on bit N in the given vector.
static really_inline void setbit256(m256 *ptr, unsigned int n) {
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
static really_inline void clearbit256(m256 *ptr, unsigned int n) {
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
static really_inline char testbit256(m256 val, unsigned int n) {
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

static really_really_inline m128 movdq_hi(m256 x) { return x.hi; }

static really_really_inline m128 movdq_lo(m256 x) { return x.lo; }

static really_inline m256 combine2x128(m128 hi, m128 lo) {
    m256 rv = {lo, hi};
    return rv;
}

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

static really_really_inline m384 lshift64_m384(m384 a, unsigned b) {
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

/**
 * "Rich" version of diff384(). Takes two vectors a and b and returns a 12-bit
 * mask indicating which 32-bit words contain differences.
 */
static really_inline u32 diffrich384(m384 a, m384 b) {
    m128 z = zeroes128();
    uint32x4_t x = vceqq_s32(a.lo.vect_s32, b.lo.vect_s32);
    uint32x4_t y = vceqq_s32(a.mid.vect_s32, b.mid.vect_s32);
    uint32x4_t w = vceqq_s32(a.hi.vect_s32, b.hi.vect_s32);

    uint16x8_t q = vcombine_u16(vqmovn_u32(x), vqmovn_u32(y));
    uint16x8_t p = vcombine_u16(vqmovn_u32(w), vqmovn_u32(z.vect_u32));

    uint8x16_t input = vcombine_u8(vqmovn_u16(q), vqmovn_u16(p));

    static const int8_t __attribute__((aligned(16)))
    xr[8] = {-7, -6, -5, -4, -3, -2, -1, 0};
    uint8x8_t mask_and = vdup_n_u8(0x80);
    int8x8_t mask_shift = vld1_s8(xr);

    uint8x8_t lo = vget_low_u8(input);
    uint8x8_t hi = vget_high_u8(input);

    lo = vand_u8(lo, mask_and);
    lo = vshl_u8(lo, mask_shift);

    hi = vand_u8(hi, mask_and);
    hi = vshl_u8(hi, mask_shift);

    lo = vpadd_u8(lo, lo);
    lo = vpadd_u8(lo, lo);
    lo = vpadd_u8(lo, lo);

    hi = vpadd_u8(hi, hi);
    hi = vpadd_u8(hi, hi);
    hi = vpadd_u8(hi, hi);

    return ~((hi[0] << 8) | (lo[0] & 0xFF)) & 0xfff;
}

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
    m384 rv = {load128(ptr), load128((const char *)ptr + 16),
               load128((const char *)ptr + 32)};
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
    m384 rv = {loadu128(ptr), loadu128((const char *)ptr + 16),
               loadu128((const char *)ptr + 32)};
    return rv;
}

// packed unaligned store of first N bytes
static really_inline void storebytes384(void *ptr, m384 a, unsigned int n) {
    assert(n <= sizeof(a));
    memcpy(ptr, &a, n);
}

// packed unaligned load of first N bytes, pad with zero
static really_inline m384 loadbytes384(const void *ptr, unsigned int n) {
    m384 a = zeroes384();
    assert(n <= sizeof(a));
    memcpy(&a, ptr, n);
    return a;
}

// switches on bit N in the given vector.
static really_inline void setbit384(m384 *ptr, unsigned int n) {
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
static really_inline void clearbit384(m384 *ptr, unsigned int n) {
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
static really_inline char testbit384(m384 val, unsigned int n) {
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

static really_inline m512 zeroes512(void) {
    m512 rv = {zeroes256(), zeroes256()};
    return rv;
}

static really_inline m512 ones512(void) {
    m512 rv = {ones256(), ones256()};
    return rv;
}

static really_inline m512 and512(m512 a, m512 b) {
    m512 rv;
    rv.lo = and256(a.lo, b.lo);
    rv.hi = and256(a.hi, b.hi);
    return rv;
}

static really_inline m512 or512(m512 a, m512 b) {
    m512 rv;
    rv.lo = or256(a.lo, b.lo);
    rv.hi = or256(a.hi, b.hi);
    return rv;
}

static really_inline m512 xor512(m512 a, m512 b) {
    m512 rv;
    rv.lo = xor256(a.lo, b.lo);
    rv.hi = xor256(a.hi, b.hi);
    return rv;
}

static really_inline m512 not512(m512 a) {
    m512 rv;
    rv.lo = not256(a.lo);
    rv.hi = not256(a.hi);
    return rv;
}

static really_inline m512 andnot512(m512 a, m512 b) {
    m512 rv;
    rv.lo = andnot256(a.lo, b.lo);
    rv.hi = andnot256(a.hi, b.hi);
    return rv;
}

static really_really_inline m512 lshift64_m512(m512 a, unsigned b) {
    m512 rv;
    rv.lo = lshift64_m256(a.lo, b);
    rv.hi = lshift64_m256(a.hi, b);
    return rv;
}

static really_inline int diff512(m512 a, m512 b) {
    return diff256(a.lo, b.lo) || diff256(a.hi, b.hi);
}

static really_inline int isnonzero512(m512 a) {
    m128 x = or128(a.lo.lo, a.lo.hi);
    m128 y = or128(a.hi.lo, a.hi.hi);
    return isnonzero128(or128(x, y));
}

/**
 * "Rich" version of diff512(). Takes two vectors a and b and returns a 16-bit
 * mask indicating which 32-bit words contain differences.
 */
static really_inline u32 diffrich512(m512 a, m512 b) {
    uint32x4_t x = vceqq_s32(a.lo.lo.vect_s32, b.lo.lo.vect_s32);
    uint32x4_t y = vceqq_s32(a.lo.hi.vect_s32, b.lo.hi.vect_s32);
    uint32x4_t z = vceqq_s32(a.hi.lo.vect_s32, b.hi.lo.vect_s32);
    uint32x4_t w = vceqq_s32(a.hi.hi.vect_s32, b.hi.hi.vect_s32);
    uint16x8_t p = vcombine_u16(vqmovn_u32(x), vqmovn_u32(y));
    uint16x8_t q = vcombine_u16(vqmovn_u32(z), vqmovn_u32(w));

    uint8x16_t input = vcombine_u8(vqmovn_u16(p), vqmovn_u16(q));

    static const int8_t __attribute__((aligned(16)))
    xr[8] = {-7, -6, -5, -4, -3, -2, -1, 0};
    uint8x8_t mask_and = vdup_n_u8(0x80);
    int8x8_t mask_shift = vld1_s8(xr);

    uint8x8_t lo = vget_low_u8(input);
    uint8x8_t hi = vget_high_u8(input);

    lo = vand_u8(lo, mask_and);
    lo = vshl_u8(lo, mask_shift);

    hi = vand_u8(hi, mask_and);
    hi = vshl_u8(hi, mask_shift);

    lo = vpadd_u8(lo, lo);
    lo = vpadd_u8(lo, lo);
    lo = vpadd_u8(lo, lo);

    hi = vpadd_u8(hi, hi);
    hi = vpadd_u8(hi, hi);
    hi = vpadd_u8(hi, hi);

    return ~((hi[0] << 8) | (lo[0] & 0xFF)) & 0xffff;
}

/**
 * "Rich" version of diffrich(), 64-bit variant. Takes two vectors a and b and
 * returns a 16-bit mask indicating which 64-bit words contain differences.
 */
static really_inline u32 diffrich64_512(m512 a, m512 b) {
    u32 d = diffrich512(a, b);
    return (d | (d >> 1)) & 0x55555555;
}

// aligned load
static really_inline m512 load512(const void *ptr) {
    assert(ISALIGNED_N(ptr, alignof(m256)));
    m512 rv = {load256(ptr), load256((const char *)ptr + 32)};
    return rv;
}

// aligned store
static really_inline void store512(void *ptr, m512 a) {
    assert(ISALIGNED_N(ptr, alignof(m512)));
    ptr = assume_aligned(ptr, 16);
    *(m512 *)ptr = a;
}

// unaligned load
static really_inline m512 loadu512(const void *ptr) {
    m512 rv = {loadu256(ptr), loadu256((const char *)ptr + 32)};
    return rv;
}

// packed unaligned store of first N bytes
static really_inline void storebytes512(void *ptr, m512 a, unsigned int n) {
    assert(n <= sizeof(a));
    memcpy(ptr, &a, n);
}

// packed unaligned load of first N bytes, pad with zero
static really_inline m512 loadbytes512(const void *ptr, unsigned int n) {
    m512 a = zeroes512();
    assert(n <= sizeof(a));
    memcpy(&a, ptr, n);
    return a;
}

static really_inline m512 mask1bit512(unsigned int n) {
    assert(n < sizeof(m512) * 8);
    u32 mask_idx = ((n % 8) * 64) + 95;
    mask_idx -= n / 8;
    return loadu512(&simd_onebit_masks[mask_idx]);
}

// switches on bit N in the given vector.
static really_inline void setbit512(m512 *ptr, unsigned int n) {
    assert(n < sizeof(*ptr) * 8);
    m128 *sub;
    if (n < 128) {
        sub = &ptr->lo.lo;
    } else if (n < 256) {
        sub = &ptr->lo.hi;
    } else if (n < 384) {
        sub = &ptr->hi.lo;
    } else {
        sub = &ptr->hi.hi;
    }
    setbit128(sub, n % 128);
}

// switches off bit N in the given vector.
static really_inline void clearbit512(m512 *ptr, unsigned int n) {
    assert(n < sizeof(*ptr) * 8);
    m128 *sub;
    if (n < 128) {
        sub = &ptr->lo.lo;
    } else if (n < 256) {
        sub = &ptr->lo.hi;
    } else if (n < 384) {
        sub = &ptr->hi.lo;
    } else {
        sub = &ptr->hi.hi;
    }
    clearbit128(sub, n % 128);
}

// tests bit N in the given vector.
static really_inline char testbit512(m512 val, unsigned int n) {
    assert(n < sizeof(val) * 8);
    m128 sub;
    if (n < 128) {
        sub = val.lo.lo;
    } else if (n < 256) {
        sub = val.lo.hi;
    } else if (n < 384) {
        sub = val.hi.lo;
    } else {
        sub = val.hi.hi;
    }
    return testbit128(sub, n % 128);
}
#pragma GCC diagnostic pop

#endif
