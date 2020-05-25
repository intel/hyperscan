/*
 * Copyright (c) 2017-2020, Intel Corporation
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
 * \brief Per-platform architecture definitions
 */

#ifndef UTIL_ARCH_H_
#define UTIL_ARCH_H_

#if defined(__SSE2__) || defined(_M_X64) || (_M_IX86_FP >= 2)
#define HAVE_SSE2
#endif

#if defined(__SSE4_1__) || (defined(_WIN32) && defined(__AVX__))
#define HAVE_SSE41
#endif

#if defined(__SSE4_2__) || (defined(_WIN32) && defined(__AVX__))
#define HAVE_SSE42
#endif

#if defined(__AVX__)
#define HAVE_AVX
#endif

#if defined(__AVX2__)
#define HAVE_AVX2
#endif

#if defined(__AVX512BW__)
#define HAVE_AVX512
#endif

#if defined(__AVX512VBMI__)
#define HAVE_AVX512VBMI
#endif

/*
 * ICC and MSVC don't break out POPCNT or BMI/2 as separate pre-def macros
 */
#if defined(__POPCNT__) ||                                                     \
    (defined(__INTEL_COMPILER) && defined(__SSE4_2__)) ||                      \
    (defined(_WIN32) && defined(__AVX__))
#define HAVE_POPCOUNT_INSTR
#endif

#if defined(__BMI__) || (defined(_WIN32) && defined(__AVX2__)) ||              \
    (defined(__INTEL_COMPILER) && defined(__AVX2__))
#define HAVE_BMI
#endif

#if defined(__BMI2__) || (defined(_WIN32) && defined(__AVX2__)) ||             \
    (defined(__INTEL_COMPILER) && defined(__AVX2__))
#define HAVE_BMI2
#endif

/*
 * MSVC uses a different form of inline asm
 */
#if defined(_WIN32) && defined(_MSC_VER)
#define NO_ASM
#endif

#endif // UTIL_ARCH_H_
