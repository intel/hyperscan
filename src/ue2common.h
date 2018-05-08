/*
 * Copyright (c) 2015-2018, Intel Corporation
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
 * \brief Core UE2 global types, defines, utilities.
 *
 * NOTE WELL: this file is included into both C and C++ source code, so
 * be sure to remain compatible with both.
 */

#ifndef UE2COMMON_H
#define UE2COMMON_H

#include "config.h"

/* standard types used across ue2 */

// We use the size_t type all over the place, usually defined in stddef.h.
#include <stddef.h>
// stdint.h for things like uintptr_t and friends
#include <stdint.h>

/* ick */
#if defined(_WIN32)
#define ALIGN_ATTR(x) __declspec(align(x))
#else
#define ALIGN_ATTR(x) __attribute__((aligned((x))))
#endif

#define ALIGN_DIRECTIVE ALIGN_ATTR(16)
#define ALIGN_AVX_DIRECTIVE ALIGN_ATTR(32)
#define ALIGN_CL_DIRECTIVE ALIGN_ATTR(64)

typedef signed char s8;
typedef unsigned char u8;
typedef signed short s16;
typedef unsigned short u16;
typedef unsigned int u32;
typedef signed int s32;

/* We append the 'a' for aligned, since these aren't common, garden variety
 * 64 bit values. The alignment is necessary for structs on some platforms,
 * so we don't end up performing accidental unaligned accesses. */
#if defined(_WIN32) && ! defined(_WIN64)
typedef unsigned long long ALIGN_ATTR(4) u64a;
typedef signed long long ALIGN_ATTR(4) s64a;
#else
typedef unsigned long long ALIGN_ATTR(8) u64a;
typedef signed long long ALIGN_ATTR(8) s64a;
#endif

/* get the SIMD types */
#include "util/simd_types.h"

/** \brief Report identifier, used for internal IDs and external IDs (those
 * reported on match). */
typedef u32 ReportID;

/* Shorthand for attribute to mark a function as part of our public API.
 * Functions without this attribute will be hidden. */
#if !defined(_WIN32)
#define HS_PUBLIC_API     __attribute__((visibility("default")))
#else
// TODO: dllexport defines for windows
#define HS_PUBLIC_API
#endif

#define ARRAY_LENGTH(a) (sizeof(a)/sizeof((a)[0]))

/** \brief Shorthand for the attribute to shut gcc about unused parameters */
#if !defined(_WIN32)
#define UNUSED __attribute__ ((unused))
#else
#define UNUSED
#endif

/* really_inline forces inlining always */
#if !defined(_WIN32)
#if defined(HS_OPTIMIZE)
#define really_inline inline __attribute__ ((always_inline, unused))
#else
#define really_inline __attribute__ ((unused))
#endif

/** no, seriously, inline it, even if building in debug mode */
#define really_really_inline inline __attribute__ ((always_inline, unused))
#define never_inline __attribute__ ((noinline))
#define alignof __alignof
#define HAVE_TYPEOF 1

#else // ms windows
#define really_inline __forceinline
#define really_really_inline __forceinline
#define never_inline
#define __builtin_prefetch(...) do {} while(0)
#if defined(__cplusplus)
#define __typeof__ decltype
#define HAVE_TYPEOF 1
#else // C
/* msvc doesn't have decltype or typeof in C */
#define inline __inline
#define alignof __alignof
#endif
#endif


// We use C99-style "restrict".
#ifdef _WIN32
#ifdef __cplusplus
#define restrict
#else
#define restrict __restrict
#endif
#else
#define restrict __restrict
#endif


// Align to 16-byte boundary
#define ROUNDUP_16(a) (((a) + 0xf) & ~0xf)
#define ROUNDDOWN_16(a) ((a) & ~0xf)

// Align to N-byte boundary
#define ROUNDUP_N(a, n) (((a) + ((n)-1)) & ~((n)-1))
#define ROUNDDOWN_N(a, n) ((a) & ~((n)-1))

// Align to a cacheline - assumed to be 64 bytes
#define ROUNDUP_CL(a) ROUNDUP_N(a, 64)

// Align ptr to next N-byte boundary
#if defined(HAVE_TYPEOF)
#define ROUNDUP_PTR(ptr, n)   (__typeof__(ptr))(ROUNDUP_N((uintptr_t)(ptr), (n)))
#define ROUNDDOWN_PTR(ptr, n) (__typeof__(ptr))(ROUNDDOWN_N((uintptr_t)(ptr), (n)))
#else
#define ROUNDUP_PTR(ptr, n)   (void*)(ROUNDUP_N((uintptr_t)(ptr), (n)))
#define ROUNDDOWN_PTR(ptr, n) (void*)(ROUNDDOWN_N((uintptr_t)(ptr), (n)))
#endif

#define ISALIGNED_N(ptr, n) (((uintptr_t)(ptr) & ((n) - 1)) == 0)
#define ISALIGNED_16(ptr)   ISALIGNED_N((ptr), 16)
#define ISALIGNED_CL(ptr)   ISALIGNED_N((ptr), 64)
#if defined(HAVE_TYPEOF)
#define ISALIGNED(ptr)      ISALIGNED_N((ptr), alignof(__typeof__(*(ptr))))
#else
/* we should probably avoid using this test in C */
#define ISALIGNED(ptr)      (1)
#endif
#define N_CHARS 256

// Maximum offset representable in the 'unsigned long long' we use to return
// offset values.
#define MAX_OFFSET 0xffffffffffffffffULL

#if !defined(MIN)
  #define MIN(a,b)      ((a) < (b) ? (a) : (b))
#endif
#if !defined(MAX)
  #define MAX(a,b)      ((a) > (b) ? (a) : (b))
#endif

#define LIMIT_TO_AT_MOST(a, b) (*(a) = MIN(*(a),(b)))
#define ENSURE_AT_LEAST(a, b) (*(a) = MAX(*(a),(b)))

#ifndef _WIN32
#ifndef likely
  #define likely(x)     __builtin_expect(!!(x), 1)
#endif
#ifndef unlikely
  #define unlikely(x)   __builtin_expect(!!(x), 0)
#endif
#else
#define likely(x)   (x)
#define unlikely(x) (x)
#endif

#if !defined(RELEASE_BUILD) || defined(DEBUG)
#ifdef _WIN32
#define PATH_SEP '\\'
#else
#define PATH_SEP '/'
#endif
#endif

#if defined(DEBUG) && !defined(DEBUG_PRINTF)
#include <string.h>
#include <stdio.h>
#define DEBUG_PRINTF(format, ...) printf("%s:%s:%d:" format, \
                                         strrchr(__FILE__, PATH_SEP) + 1, \
                                         __func__, __LINE__,  ## __VA_ARGS__)
#elif !defined(DEBUG_PRINTF)
#define DEBUG_PRINTF(format, ...) do { } while(0)
#endif

#if !defined(RELEASE_BUILD)
#include <string.h>
#include <stdio.h>
#define ADEBUG_PRINTF(format, ...) printf("!%s:%s:%d:" format, \
                                          strrchr(__FILE__, PATH_SEP) + 1, \
                                          __func__, __LINE__,  ## __VA_ARGS__)
#else
#define ADEBUG_PRINTF(format, ...) do { } while(0)
#endif

#include <assert.h>

#endif
