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

#include "fdr_compile_internal.h"
#include "fdr_engine_description.h"
#include "hs_compile.h"
#include "util/target_info.h"
#include "util/compare.h" // for ourisalpha()
#include "util/make_unique.h"

#include <cassert>
#include <cstdlib>
#include <map>
#include <string>

using namespace std;

namespace ue2 {

FDREngineDescription::FDREngineDescription(const FDREngineDef &def)
    : EngineDescription(def.id, targetByArchFeatures(def.cpu_features),
                        def.numBuckets),
      schemeWidth(def.schemeWidth), stride(0), bits(0) {}

u32 FDREngineDescription::getDefaultFloodSuffixLength() const {
    // rounding up, so that scheme width 32 and 6 buckets is 6 not 5!
    // the +1 avoids pain due to various reach choices
    return ((getSchemeWidth() + getNumBuckets() - 1) / getNumBuckets()) + 1;
}

void getFdrDescriptions(vector<FDREngineDescription> *out) {
    static const FDREngineDef def = {0, 64, 8, 0};
    out->clear();
    out->emplace_back(def);
}

static
u32 findDesiredStride(size_t num_lits, size_t min_len, size_t min_len_count) {
    u32 desiredStride = 1; // always our safe fallback
    if (min_len > 1) {
        if (num_lits < 250) {
            // small cases we just go for it
            desiredStride = min_len;
        } else if (num_lits < 800) {
            // intermediate cases
            desiredStride = min_len - 1;
        } else if (num_lits < 5000) {
            // for larger but not huge sizes, go to stride 2 only if we have at
            // least minlen 3
            desiredStride = MIN(min_len - 1, 2);
        }
    }

    // patch if count is quite large - a ton of length 2 literals can
    // break things
#ifdef TRY_THIS_LATER
    if ((min_len == 2) && (desiredStride == 2) && (min_len_count > 20)) {
        desiredStride = 1;
    }
#endif

    // patch stuff just for the stride 4 case; don't let min_len=4,
    // desiredStride=4 through as even a few length 4 literals can break things
    // (far more fragile)
    if ((min_len == 4) && (desiredStride == 4) && (min_len_count > 2)) {
        desiredStride = 2;
    }

    return desiredStride;
}

unique_ptr<FDREngineDescription> chooseEngine(const target_t &target,
                                              const vector<hwlmLiteral> &vl,
                                              bool make_small) {
    vector<FDREngineDescription> allDescs;
    getFdrDescriptions(&allDescs);

    // find desired stride
    size_t count;
    size_t msl = minLenCount(vl, &count);
    u32 desiredStride = findDesiredStride(vl.size(), msl, count);

    DEBUG_PRINTF("%zu lits, msl=%zu, desiredStride=%u\n", vl.size(), msl,
                 desiredStride);

    FDREngineDescription *best = nullptr;
    u32 best_score = 0;

    FDREngineDescription &eng = allDescs[0];

    for (u32 domain = 9; domain <= 15; domain++) {
        for (size_t stride = 1; stride <= 4; stride *= 2) {
            // to make sure that domains >=14 have stride 1 according to origin
            if (domain > 13 && stride > 1) {
                continue;
            }
            if (!eng.isValidOnTarget(target)) {
                continue;
            }
            if (msl < stride) {
                continue;
            }

            u32 score = 100;

            score -= absdiff(desiredStride, stride);

            if (stride <= desiredStride) {
                score += stride;
            }

            u32 effLits = vl.size(); /* * desiredStride;*/
            u32 ideal;
            if (effLits < eng.getNumBuckets()) {
                if (stride == 1) {
                    ideal = 8;
                } else {
                    ideal = 10;
                }
            } else if (effLits < 20) {
                ideal = 10;
            } else if (effLits < 100) {
                ideal = 11;
            } else if (effLits < 1000) {
                ideal = 12;
            } else if (effLits < 10000) {
                ideal = 13;
            } else {
                ideal = 15;
            }

            if (ideal != 8 && eng.schemeWidth == 32) {
                ideal += 1;
            }

            if (make_small) {
                ideal -= 2;
            }

            if (stride > 1) {
                ideal++;
            }

            DEBUG_PRINTF("effLits %u\n", effLits);

            if (target.is_atom_class() && !make_small && effLits < 4000) {
                /* Unless it is a very heavy case, we want to build smaller
                 * tables on lightweight machines due to their small caches. */
                ideal -= 2;
            }

            score -= absdiff(ideal, domain);

            DEBUG_PRINTF("fdr %u: width=%u, domain=%u, buckets=%u, stride=%zu "
                         "-> score=%u\n",
                         eng.getID(), eng.schemeWidth, domain,
                         eng.getNumBuckets(), stride, score);

            if (!best || score > best_score) {
                eng.bits = domain;
                eng.stride = stride;
                best = &eng;
                best_score = score;
            }
        }
    }

    if (!best) {
        DEBUG_PRINTF("failed to find engine\n");
        return nullptr;
    }

    DEBUG_PRINTF("using engine %u\n", best->getID());
    return ue2::make_unique<FDREngineDescription>(*best);
}

SchemeBitIndex FDREngineDescription::getSchemeBit(BucketIndex b,
                                                  PositionInBucket p) const {
    assert(p < getBucketWidth(b));
    SchemeBitIndex sbi = p * getNumBuckets() + b;
    assert(sbi < getSchemeWidth());
    return sbi;
}

u32 FDREngineDescription::getBucketWidth(BucketIndex) const {
    u32 sw = getSchemeWidth();
    u32 nm = getNumBuckets();
    assert(sw % nm == 0);
    return sw/nm;
}

unique_ptr<FDREngineDescription> getFdrDescription(u32 engineID) {
    vector<FDREngineDescription> allDescs;
    getFdrDescriptions(&allDescs);

    if (engineID >= allDescs.size()) {
        return nullptr;
    }

    return ue2::make_unique<FDREngineDescription>(allDescs[engineID]);
}

} // namespace ue2
