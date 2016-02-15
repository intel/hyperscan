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

#include "cpuid_flags.h"
#include "ue2common.h"
#include "hs_compile.h" // for HS_MODE_ flags
#include "hs_internal.h"

#ifndef _WIN32
#include <cpuid.h>
#endif

// ECX
#define SSE3 (1 << 0)
#define SSSE3 (1 << 9)
#define SSE4_1 (1 << 19)
#define SSE4_2 (1 << 20)
#define XSAVE (1 << 27)
#define AVX (1 << 28)

// EDX
#define SSE (1 << 25)
#define SSE2 (1 << 25)
#define HTT (1 << 28)

// Structured Extended Feature Flags Enumeration Leaf ECX values
#define BMI (1 << 3)
#define AVX2 (1 << 5)
#define BMI2 (1 << 8)

// Extended Control Register 0 (XCR0) values
#define XCR0_SSE (1 << 1)
#define XCR0_AVX (1 << 2)

static __inline
void cpuid(unsigned int op, unsigned int leaf, unsigned int *eax,
           unsigned int *ebx, unsigned int *ecx, unsigned int *edx) {
#ifndef _WIN32
    __cpuid_count(op, leaf, *eax, *ebx, *ecx, *edx);
#else
    unsigned int a[4];
    __cpuidex(a, op, leaf);
    *eax = a[0];
    *ebx = a[1];
    *ecx = a[2];
    *edx = a[3];
#endif
}

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

static
int check_avx2(void) {
#if defined(__INTEL_COMPILER)
    return _may_i_use_cpu_feature(_FEATURE_AVX2);
#else
    unsigned int eax, ebx, ecx, edx;

    cpuid(1, 0, &eax, &ebx, &ecx, &edx);

    /* check AVX is supported and XGETBV is enabled by OS */
    if ((ecx & (AVX | XSAVE)) != (AVX | XSAVE)) {
        DEBUG_PRINTF("AVX and XSAVE not supported\n");
        return 0;
    }

    /* check that SSE and AVX registers are enabled by OS */
    u64a xcr0 = xgetbv(0);
    if ((xcr0 & (XCR0_SSE | XCR0_AVX)) != (XCR0_SSE | XCR0_AVX)) {
        DEBUG_PRINTF("SSE and AVX registers not enabled\n");
        return 0;
    }

    /* ECX and EDX contain capability flags */
    ecx = 0;
    cpuid(7, 0, &eax, &ebx, &ecx, &edx);

    if (ebx & AVX2) {
        DEBUG_PRINTF("AVX2 enabled\n");
        return 1;
    }

    return 0;
#endif
}

u64a cpuid_flags(void) {
    u64a cap = 0;

    if (check_avx2()) {
        DEBUG_PRINTF("AVX2 enabled\n");
        cap |= HS_CPU_FEATURES_AVX2;
    }

#if !defined(__AVX2__)
    cap &= ~HS_CPU_FEATURES_AVX2;
#endif

    return cap;
}

struct family_id {
    u32 full_family;
    u32 full_model;
    u32 tune;
};

/* from table 35-1 of the Intel 64 and IA32 Arch. Software Developer's Manual
 * and "Intel Architecture and Processor Identification With CPUID Model and
 * Family Numbers" */
static const struct family_id known_microarch[] = {
    { 0x6, 0x37, HS_TUNE_FAMILY_SLM }, /* baytrail */
    { 0x6, 0x4D, HS_TUNE_FAMILY_SLM }, /* avoton, rangley */

    { 0x6, 0x3C, HS_TUNE_FAMILY_HSW }, /* haswell */
    { 0x6, 0x45, HS_TUNE_FAMILY_HSW }, /* haswell */
    { 0x6, 0x46, HS_TUNE_FAMILY_HSW }, /* haswell */
    { 0x6, 0x3F, HS_TUNE_FAMILY_HSW }, /* haswell */

    { 0x6, 0x3E, HS_TUNE_FAMILY_IVB }, /* ivybridge */
    { 0x6, 0x3A, HS_TUNE_FAMILY_IVB }, /* ivybridge */

    { 0x6, 0x2A, HS_TUNE_FAMILY_SNB }, /* sandybridge */
    { 0x6, 0x2D, HS_TUNE_FAMILY_SNB }, /* sandybridge */

    { 0x6, 0x3D, HS_TUNE_FAMILY_BDW }, /* broadwell Core-M */
    { 0x6, 0x4F, HS_TUNE_FAMILY_BDW }, /* broadwell xeon */
    { 0x6, 0x56, HS_TUNE_FAMILY_BDW }, /* broadwell xeon-d */

//    { 0x6, 0x25, HS_TUNE_FAMILY_GENERIC }, /* westmere */
//    { 0x6, 0x2C, HS_TUNE_FAMILY_GENERIC }, /* westmere */
//    { 0x6, 0x2F, HS_TUNE_FAMILY_GENERIC }, /* westmere */

//    { 0x6, 0x1E, HS_TUNE_FAMILY_GENERIC }, /* nehalem */
//    { 0x6, 0x1A, HS_TUNE_FAMILY_GENERIC }, /* nehalem */
//    { 0x6, 0x2E, HS_TUNE_FAMILY_GENERIC }, /* nehalem */

//    { 0x6, 0x17, HS_TUNE_FAMILY_GENERIC }, /* penryn */
//    { 0x6, 0x1D, HS_TUNE_FAMILY_GENERIC }, /* penryn */

};

#ifdef DUMP_SUPPORT
static UNUSED
const char *dumpTune(u32 tune) {
#define T_CASE(x) case x: return #x;
    switch (tune) {
        T_CASE(HS_TUNE_FAMILY_SLM);
        T_CASE(HS_TUNE_FAMILY_HSW);
        T_CASE(HS_TUNE_FAMILY_SNB);
        T_CASE(HS_TUNE_FAMILY_IVB);
        T_CASE(HS_TUNE_FAMILY_BDW);
    }
#undef T_CASE
    return "unknown";
}
#endif

u32 cpuid_tune(void) {
    unsigned int eax, ebx, ecx, edx;

    cpuid(1, 0, &eax, &ebx, &ecx, &edx);

    u32 family = (eax >> 8) & 0xf;
    u32 model = 0;

    if (family == 0x6 || family == 0xf) {
        model = ((eax >> 4) & 0xf) | ((eax >> 12) & 0xf0);
    } else {
        model = (eax >> 4) & 0xf;
    }

    DEBUG_PRINTF("family = %xh model = %xh\n", family, model);
    for (u32 i = 0; i < ARRAY_LENGTH(known_microarch); i++) {
        if (family != known_microarch[i].full_family) {
            continue;
        }

        if (model != known_microarch[i].full_model) {
            continue;
        }

        u32 tune = known_microarch[i].tune;
        DEBUG_PRINTF("found tune flag %s\n", dumpTune(tune) );
        return tune;
    }

    return HS_TUNE_FAMILY_GENERIC;
}
