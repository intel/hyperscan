/*
 * Copyright (c) 2015-2017, Intel Corporation
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

#include "config.h"

#include "cross_compile.h"
#include "src/ue2common.h"
#include "src/hs_compile.h"
#include "src/util/make_unique.h"

#include <sstream>
#include <string>

using namespace std;

struct XcompileMode {
    const string name;
    unsigned long long cpu_features;
};

static const XcompileMode xcompile_options[] = {
    { "avx512", HS_CPU_FEATURES_AVX512 },
    { "avx2", HS_CPU_FEATURES_AVX2 },
    { "base", 0 },
};

unique_ptr<hs_platform_info> xcompileReadMode(const char *s) {
    hs_platform_info rv;
    UNUSED hs_error_t err;
    err = hs_populate_platform(&rv);
    assert(!err);

    string str(s);
    string mode = str.substr(0, str.find(":"));
    string opt = str.substr(str.find(":")+1, str.npos);
    bool found_mode = false;

    if (!opt.empty()) {
        for (const auto &xcompile : xcompile_options) {
            if (opt == xcompile.name) {
                rv.cpu_features = xcompile.cpu_features;
                found_mode = true;
                break;
            }
        }
    }

    if (!found_mode) {
        return nullptr;
    } else {
        DEBUG_PRINTF("cpu_features %llx\n", rv.cpu_features);
        return ue2::make_unique<hs_platform_info>(rv);
    }
}

string to_string(const hs_platform_info &p) {
    ostringstream out;
    if (p.tune) {
        out << p.tune;
    }

    if (p.cpu_features) {
        u64a features = p.cpu_features;
        if (features & HS_CPU_FEATURES_AVX512) {
            out << " avx512";
            features &= ~HS_CPU_FEATURES_AVX512;
        }

        if (features & HS_CPU_FEATURES_AVX2) {
            out << " avx2";
            features &= ~HS_CPU_FEATURES_AVX2;
        }

        if (features) {
            out << " " << "?cpu_features?:" << features;
        }
    }

    return out.str();
}

string xcompileUsage(void) {
    string variants = "Instruction set options: ";
    const auto commaspace = ", ";
    auto sep = "";
    for (const auto &xcompile : xcompile_options) {
        variants += sep + xcompile.name;
        sep = commaspace;
    }
    return variants;
}
