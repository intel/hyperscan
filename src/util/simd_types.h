/*
 * Copyright (c) 2015, Intel Corporation
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

#ifndef SIMD_TYPES_H
#define SIMD_TYPES_H

#include "config.h"
#include "ue2common.h"

// more recent headers are bestest, but only if we can use them
#ifdef __cplusplus
# if defined(HAVE_CXX_X86INTRIN_H)
#  define USE_X86INTRIN_H
# endif
#else // C
# if defined(HAVE_C_X86INTRIN_H)
#  define USE_X86INTRIN_H
# endif
#endif

#ifdef __cplusplus
# if defined(HAVE_CXX_INTRIN_H)
#  define USE_INTRIN_H
# endif
#else // C
# if defined(HAVE_C_INTRIN_H)
#  define USE_INTRIN_H
# endif
#endif

#if defined(USE_X86INTRIN_H)
#include <x86intrin.h>
#elif defined(USE_INTRIN_H)
#include <intrin.h>
#else
#error no intrinsics!
#endif

typedef __m128i m128;
#if defined(__AVX2__)
typedef __m256i m256;
#else
typedef struct ALIGN_AVX_DIRECTIVE {m128 lo; m128 hi;} m256;
#endif

// these should align to 16 and 32 respectively
typedef struct {m128 lo; m128 mid; m128 hi;} m384;
typedef struct {m256 lo; m256 hi;} m512;

#endif /* SIMD_TYPES_H */

