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

#include "cpuid_flags.h"
#include "cpuid_inline.h"
#include "ue2common.h"
#include "hs_compile.h" // for HS_MODE_ flags
#include "hs_internal.h"
#include "util/arch.h"

#if !defined(_WIN32) && !defined(CPUID_H_)
#include <cpuid.h>
#endif

u64a cpuid_flags(void) {
    u64a cap = 0;

    if (check_avx2()) {
        DEBUG_PRINTF("AVX2 enabled\n");
        cap |= HS_CPU_FEATURES_AVX2;
    }

    if (check_avx512()) {
        DEBUG_PRINTF("AVX512 enabled\n");
        cap |= HS_CPU_FEATURES_AVX512;
    }

    if (check_avx512vbmi()) {
        DEBUG_PRINTF("AVX512VBMI enabled\n");
        cap |= HS_CPU_FEATURES_AVX512VBMI;
    }

#if !defined(FAT_RUNTIME) && !defined(HAVE_AVX2)
    cap &= ~HS_CPU_FEATURES_AVX2;
#endif

#if (!defined(FAT_RUNTIME) && !defined(HAVE_AVX512)) ||                        \
    (defined(FAT_RUNTIME) && !defined(BUILD_AVX512))
    cap &= ~HS_CPU_FEATURES_AVX512;
#endif

#if (!defined(FAT_RUNTIME) && !defined(HAVE_AVX512VBMI)) ||                    \
    (defined(FAT_RUNTIME) && !defined(BUILD_AVX512VBMI))
    cap &= ~HS_CPU_FEATURES_AVX512VBMI;
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
    { 0x6, 0x4A, HS_TUNE_FAMILY_SLM }, /* silvermont */
    { 0x6, 0x4C, HS_TUNE_FAMILY_SLM }, /* silvermont */
    { 0x6, 0x4D, HS_TUNE_FAMILY_SLM }, /* avoton, rangley */
    { 0x6, 0x5A, HS_TUNE_FAMILY_SLM }, /* silvermont */
    { 0x6, 0x5D, HS_TUNE_FAMILY_SLM }, /* silvermont */

    { 0x6, 0x5C, HS_TUNE_FAMILY_GLM }, /* goldmont */
    { 0x6, 0x5F, HS_TUNE_FAMILY_GLM }, /* denverton */

    { 0x6, 0x3C, HS_TUNE_FAMILY_HSW }, /* haswell */
    { 0x6, 0x45, HS_TUNE_FAMILY_HSW }, /* haswell */
    { 0x6, 0x46, HS_TUNE_FAMILY_HSW }, /* haswell */
    { 0x6, 0x3F, HS_TUNE_FAMILY_HSW }, /* haswell Xeon */

    { 0x6, 0x3E, HS_TUNE_FAMILY_IVB }, /* ivybridge Xeon */
    { 0x6, 0x3A, HS_TUNE_FAMILY_IVB }, /* ivybridge */

    { 0x6, 0x2A, HS_TUNE_FAMILY_SNB }, /* sandybridge */
    { 0x6, 0x2D, HS_TUNE_FAMILY_SNB }, /* sandybridge Xeon */

    { 0x6, 0x3D, HS_TUNE_FAMILY_BDW }, /* broadwell Core-M */
    { 0x6, 0x47, HS_TUNE_FAMILY_BDW }, /* broadwell */
    { 0x6, 0x4F, HS_TUNE_FAMILY_BDW }, /* broadwell xeon */
    { 0x6, 0x56, HS_TUNE_FAMILY_BDW }, /* broadwell xeon-d */

    { 0x6, 0x4E, HS_TUNE_FAMILY_SKL }, /* Skylake Mobile */
    { 0x6, 0x5E, HS_TUNE_FAMILY_SKL }, /* Skylake Core/E3 Xeon */
    { 0x6, 0x55, HS_TUNE_FAMILY_SKX }, /* Skylake Xeon */

    { 0x6, 0x8E, HS_TUNE_FAMILY_SKL }, /* Kabylake Mobile */
    { 0x6, 0x9E, HS_TUNE_FAMILY_SKL }, /* Kabylake desktop */

    { 0x6, 0x7D, HS_TUNE_FAMILY_ICL }, /* Icelake */
    { 0x6, 0x7E, HS_TUNE_FAMILY_ICL }, /* Icelake */
    { 0x6, 0x6A, HS_TUNE_FAMILY_ICX }, /* Icelake Xeon-D */
    { 0x6, 0x6C, HS_TUNE_FAMILY_ICX }, /* Icelake Xeon */

};

#ifdef DUMP_SUPPORT
static UNUSED
const char *dumpTune(u32 tune) {
#define T_CASE(x) case x: return #x;
    switch (tune) {
        T_CASE(HS_TUNE_FAMILY_SLM);
        T_CASE(HS_TUNE_FAMILY_GLM);
        T_CASE(HS_TUNE_FAMILY_HSW);
        T_CASE(HS_TUNE_FAMILY_SNB);
        T_CASE(HS_TUNE_FAMILY_IVB);
        T_CASE(HS_TUNE_FAMILY_BDW);
        T_CASE(HS_TUNE_FAMILY_SKL);
        T_CASE(HS_TUNE_FAMILY_SKX);
        T_CASE(HS_TUNE_FAMILY_ICL);
        T_CASE(HS_TUNE_FAMILY_ICX);
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
