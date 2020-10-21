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


#include "hs_compile.h" // for various hs_platform_info flags
#include "target_info.h"
#include "util/cpuid_flags.h"

namespace ue2 {

target_t get_current_target(void) {
    hs_platform_info p;
    p.cpu_features = cpuid_flags();
    p.tune = cpuid_tune();

    return target_t(p);
}

bool target_t::can_run_on_code_built_for(const target_t &code_target) const {
    if (!has_avx2() && code_target.has_avx2()) {
        return false;
    }

    if (!has_avx512() && code_target.has_avx512()) {
        return false;
    }

    if (!has_avx512vbmi() && code_target.has_avx512vbmi()) {
        return false;
    }

    return true;
}

target_t::target_t(const hs_platform_info &p)
    : tune(p.tune), cpu_features(p.cpu_features) {}

bool target_t::has_avx2(void) const {
    return cpu_features & HS_CPU_FEATURES_AVX2;
}

bool target_t::has_avx512(void) const {
    return cpu_features & HS_CPU_FEATURES_AVX512;
}

bool target_t::has_avx512vbmi(void) const {
    return cpu_features & HS_CPU_FEATURES_AVX512VBMI;
}

bool target_t::is_atom_class(void) const {
    return tune == HS_TUNE_FAMILY_SLM || tune == HS_TUNE_FAMILY_GLM;
}

} // namespace ue2
