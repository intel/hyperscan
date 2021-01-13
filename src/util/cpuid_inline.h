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

#ifndef CPUID_INLINE_H_
#define CPUID_INLINE_H_

#include "ue2common.h"
#include "cpuid_flags.h"

#if !defined(_WIN32) && !defined(CPUID_H_)
#include <cpuid.h>
/* system header doesn't have a header guard */
#define CPUID_H_
#endif

#ifdef __cplusplus
extern "C"
{
#endif

static inline
void cpuid(unsigned int op, unsigned int leaf, unsigned int *eax,
           unsigned int *ebx, unsigned int *ecx, unsigned int *edx) {
#ifndef _WIN32
    __cpuid_count(op, leaf, *eax, *ebx, *ecx, *edx);
#else
    int a[4];
    __cpuidex(a, op, leaf);
    *eax = a[0];
    *ebx = a[1];
    *ecx = a[2];
    *edx = a[3];
#endif
}

// ECX
#define CPUID_SSE3 (1 << 0)
#define CPUID_SSSE3 (1 << 9)
#define CPUID_SSE4_1 (1 << 19)
#define CPUID_SSE4_2 (1 << 20)
#define CPUID_POPCNT (1 << 23)
#define CPUID_XSAVE (1 << 27)
#define CPUID_AVX (1 << 28)

// EDX
#define CPUID_FXSAVE (1 << 24)
#define CPUID_SSE (1 << 25)
#define CPUID_SSE2 (1 << 26)
#define CPUID_HTT (1 << 28)

// Structured Extended Feature Flags Enumeration Leaf ECX values
#define CPUID_AVX512VBMI (1 << 1)

// Structured Extended Feature Flags Enumeration Leaf EBX values
#define CPUID_BMI (1 << 3)
#define CPUID_AVX2 (1 << 5)
#define CPUID_BMI2 (1 << 8)
#define CPUID_AVX512F (1 << 16)
#define CPUID_AVX512BW (1 << 30)

// Extended Control Register 0 (XCR0) values
#define CPUID_XCR0_SSE (1 << 1)
#define CPUID_XCR0_AVX (1 << 2)
#define CPUID_XCR0_OPMASK (1 << 5) // k-regs
#define CPUID_XCR0_ZMM_Hi256 (1 << 6) // upper 256 bits of ZMM0-ZMM15
#define CPUID_XCR0_Hi16_ZMM (1 << 7) // ZMM16-ZMM31

#define CPUID_XCR0_AVX512                                                      \
    (CPUID_XCR0_OPMASK | CPUID_XCR0_ZMM_Hi256 | CPUID_XCR0_Hi16_ZMM)

static inline
u64a xgetbv(u32 op) {
#if defined(_WIN32) || defined(__INTEL_COMPILER)
    return _xgetbv(op);
#else
    u32 a, d;
    __asm__ volatile (
            "xgetbv\n"
            : "=a"(a),
              "=d"(d)
            : "c"(op));
    return ((u64a)d << 32) + a;
#endif
}

static inline
int check_avx2(void) {
#if defined(__INTEL_COMPILER)
    return _may_i_use_cpu_feature(_FEATURE_AVX2);
#else
    unsigned int eax, ebx, ecx, edx;

    cpuid(1, 0, &eax, &ebx, &ecx, &edx);

    /* check AVX is supported and XGETBV is enabled by OS */
    if ((ecx & (CPUID_AVX | CPUID_XSAVE)) != (CPUID_AVX | CPUID_XSAVE)) {
        DEBUG_PRINTF("AVX and XSAVE not supported\n");
        return 0;
    }

    /* check that SSE and AVX registers are enabled by OS */
    u64a xcr0 = xgetbv(0);
    if ((xcr0 & (CPUID_XCR0_SSE | CPUID_XCR0_AVX)) !=
        (CPUID_XCR0_SSE | CPUID_XCR0_AVX)) {
        DEBUG_PRINTF("SSE and AVX registers not enabled\n");
        return 0;
    }

    /* ECX and EDX contain capability flags */
    ecx = 0;
    cpuid(7, 0, &eax, &ebx, &ecx, &edx);

    if (ebx & CPUID_AVX2) {
        DEBUG_PRINTF("AVX2 enabled\n");
        return 1;
    }

    return 0;
#endif
}

static inline
int check_avx512(void) {
    /*
     * For our purposes, having avx512 really means "can we use AVX512BW?"
     */
#if defined(__INTEL_COMPILER)
    return _may_i_use_cpu_feature(_FEATURE_AVX512BW | _FEATURE_AVX512VL);
#else
    unsigned int eax, ebx, ecx, edx;

    cpuid(1, 0, &eax, &ebx, &ecx, &edx);

    /* check XSAVE is enabled by OS */
    if (!(ecx & CPUID_XSAVE)) {
        DEBUG_PRINTF("AVX and XSAVE not supported\n");
        return 0;
    }

    /* check that AVX 512 registers are enabled by OS */
    u64a xcr0 = xgetbv(0);
    if ((xcr0 & CPUID_XCR0_AVX512) != CPUID_XCR0_AVX512) {
        DEBUG_PRINTF("AVX512 registers not enabled\n");
        return 0;
    }

    /* ECX and EDX contain capability flags */
    ecx = 0;
    cpuid(7, 0, &eax, &ebx, &ecx, &edx);

    if (!(ebx & CPUID_AVX512F)) {
        DEBUG_PRINTF("AVX512F (AVX512 Foundation) instructions not enabled\n");
        return 0;
    }

    if (ebx & CPUID_AVX512BW) {
        DEBUG_PRINTF("AVX512BW instructions enabled\n");
        return 1;
    }

    return 0;
#endif
}

static inline
int check_avx512vbmi(void) {
#if defined(__INTEL_COMPILER)
    return _may_i_use_cpu_feature(_FEATURE_AVX512VBMI);
#else
    unsigned int eax, ebx, ecx, edx;

    cpuid(1, 0, &eax, &ebx, &ecx, &edx);

    /* check XSAVE is enabled by OS */
    if (!(ecx & CPUID_XSAVE)) {
        DEBUG_PRINTF("AVX and XSAVE not supported\n");
        return 0;
    }

    /* check that AVX 512 registers are enabled by OS */
    u64a xcr0 = xgetbv(0);
    if ((xcr0 & CPUID_XCR0_AVX512) != CPUID_XCR0_AVX512) {
        DEBUG_PRINTF("AVX512 registers not enabled\n");
        return 0;
    }

    /* ECX and EDX contain capability flags */
    ecx = 0;
    cpuid(7, 0, &eax, &ebx, &ecx, &edx);

    if (!(ebx & CPUID_AVX512F)) {
        DEBUG_PRINTF("AVX512F (AVX512 Foundation) instructions not enabled\n");
        return 0;
    }

    if (!(ebx & CPUID_AVX512BW)) {
        DEBUG_PRINTF("AVX512BW instructions not enabled\n");
        return 0;
    }

    if (ecx & CPUID_AVX512VBMI) {
        DEBUG_PRINTF("AVX512VBMI instructions enabled\n");
        return 1;
    }

    return 0;
#endif
}

static inline
int check_ssse3(void) {
    unsigned int eax, ebx, ecx, edx;
    cpuid(1, 0, &eax, &ebx, &ecx, &edx);
    return !!(ecx & CPUID_SSSE3);
}

static inline
int check_sse42(void) {
    unsigned int eax, ebx, ecx, edx;
    cpuid(1, 0, &eax, &ebx, &ecx, &edx);
    return !!(ecx & CPUID_SSE4_2);
}

static inline
int check_popcnt(void) {
    unsigned int eax, ebx, ecx, edx;
    cpuid(1, 0, &eax, &ebx, &ecx, &edx);
    return !!(ecx & CPUID_POPCNT);
}

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* CPUID_INLINE_H_ */
