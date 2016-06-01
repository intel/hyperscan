/*
 * Copyright (c) 2015-2016, Intel Corporation
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
 * \brief SIMD primitives specifically for Intel SSSE3 platforms.
 */

#ifndef SIMD_UTILS_SSSE3_H_E27DF795C9AA02
#define SIMD_UTILS_SSSE3_H_E27DF795C9AA02

#if !defined(_WIN32) && !defined(__SSSE3__)
#error SSSE3 instructions must be enabled
#endif

#include "simd_utils.h"
#include "ue2common.h"

// we may already have x86intrin.h
#if !defined(USE_X86INTRIN_H)
#if defined(HAVE_C_INTRIN_H)
#include <intrin.h>
#elif defined(HAVE_TMMINTRIN_H)
#include <tmmintrin.h> // SSSE3 intrinsics
#else
#define I_HAVE_BROKEN_INTRINSICS
#endif
#endif


#if !defined(I_HAVE_BROKEN_INTRINSICS)
// newish compilers get this right
#define palignr(r, l, offset) _mm_alignr_epi8(r, l, offset)
#else
// must be inline, even in weak-sauce debug builds.
// oldish compilers either don't have the intrinsic, or force one arg through memory
static really_really_inline
m128 palignr(m128 r, m128 l, const int offset) {
    __asm__ ("palignr   %2,%1,%0" : "+x"(r) : "x"(l), "i"(offset));
    return r;
}
#endif


static really_inline
m128 pshufb(m128 a, m128 b) {
    m128 result;
#if !defined(I_HAVE_BROKEN_INTRINSICS)
    result = _mm_shuffle_epi8(a, b);
#else
    __asm__("pshufb\t%1,%0" : "=x"(result) : "xm"(b), "0"(a));
#endif
    return result;
}

#ifdef __cplusplus
extern "C" {
#endif
extern const char vbs_mask_data[];
#ifdef __cplusplus
}
#endif

static really_inline
m128 variable_byte_shift_m128(m128 in, s32 amount) {
    assert(amount >= -16 && amount <= 16);
    m128 shift_mask = loadu128(vbs_mask_data + 16 - amount);
    return pshufb(in, shift_mask);
}

#if defined(__AVX2__)

static really_inline
m256 vpshufb(m256 a, m256 b) {
    return _mm256_shuffle_epi8(a, b);
}

#if defined(USE_GCC_COMPOUND_STATEMENTS)
#define vpalignr(r, l, offset) ({                   \
    m256 res = _mm256_alignr_epi8(r, l, offset);    \
    res;                                            \
})
#else
#define vpalignr(r, l, offset) _mm256_alignr_epi8(r, l, offset)
#endif

#else // not __AVX2__

static really_inline
m256 vpshufb(m256 a, m256 b) {
    m256 rv;
    rv.lo = pshufb(a.lo, b.lo);
    rv.hi = pshufb(a.hi, b.hi);
    return rv;
}

/* palignr requires the offset to be an immediate, which we can do with a
 * compound macro, otherwise we have to enumerate the offsets and hope the
 * compiler can throw the rest away. */
#if defined(USE_GCC_COMPOUND_STATEMENTS)
#define vpalignr(r, l, offset) ({           \
    m256 res;                               \
    res.lo = palignr(r.lo, l.lo, offset);   \
    res.hi = palignr(r.hi, l.hi, offset);   \
    res;                                    \
})
#else
#define VPALIGN_CASE(N) case N: \
		res.lo = palignr(r.lo, l.lo, N); \
		res.hi = palignr(r.hi, l.hi, N); \
		return res;
static really_inline
m256 vpalignr(m256 r, m256 l, const int offset) {
	m256 res;
	switch (offset) {
	VPALIGN_CASE(0)
	VPALIGN_CASE(1)
	VPALIGN_CASE(2)
	VPALIGN_CASE(3)
	VPALIGN_CASE(4)
	VPALIGN_CASE(5)
	VPALIGN_CASE(6)
	VPALIGN_CASE(7)
	VPALIGN_CASE(8)
	VPALIGN_CASE(9)
	VPALIGN_CASE(10)
	VPALIGN_CASE(11)
	VPALIGN_CASE(12)
	VPALIGN_CASE(13)
	VPALIGN_CASE(14)
	VPALIGN_CASE(15)
	default:
		assert(0);
		return zeroes256();
	}
}
#undef VPALIGN_CASE
#endif
#endif // __AVX2__

#endif /* SIMD_UTILS_SSSE3_H_E27DF795C9AA02 */

