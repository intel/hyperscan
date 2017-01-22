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

#include "fdr_internal.h"
#include "fdr_compile_internal.h"
#include "fdr_confirm.h"
#include "ue2common.h"
#include "hs_internal.h"
#include "fdr_engine_description.h"
#include "teddy_internal.h"
#include "teddy_engine_description.h"
#include "util/make_unique.h"

#include <cmath>

using namespace std;

namespace ue2 {

TeddyEngineDescription::TeddyEngineDescription(const TeddyEngineDef &def)
    : EngineDescription(def.id, targetByArchFeatures(def.cpu_features),
                        def.numBuckets),
      numMasks(def.numMasks), packed(def.packed) {}

u32 TeddyEngineDescription::getDefaultFloodSuffixLength() const {
    return numMasks;
}

void getTeddyDescriptions(vector<TeddyEngineDescription> *out) {
    static const TeddyEngineDef defns[] = {
        { 3, 0 | HS_CPU_FEATURES_AVX2, 1, 16, false },
        { 4, 0 | HS_CPU_FEATURES_AVX2, 1, 16, true },
        { 5, 0 | HS_CPU_FEATURES_AVX2, 2, 16, false },
        { 6, 0 | HS_CPU_FEATURES_AVX2, 2, 16, true },
        { 7, 0 | HS_CPU_FEATURES_AVX2, 3, 16, false },
        { 8, 0 | HS_CPU_FEATURES_AVX2, 3, 16, true },
        { 9, 0 | HS_CPU_FEATURES_AVX2, 4, 16, false },
        { 10, 0 | HS_CPU_FEATURES_AVX2, 4, 16, true },
        { 11, 0, 1, 8, false },
        { 12, 0, 1, 8, true },
        { 13, 0, 2, 8, false },
        { 14, 0, 2, 8, true },
        { 15, 0, 3, 8, false },
        { 16, 0, 3, 8, true },
        { 17, 0, 4, 8, false },
        { 18, 0, 4, 8, true },
    };
    out->clear();
    for (const auto &def : defns) {
        out->emplace_back(def);
    }
}

static
size_t maxFloodTailLen(const vector<hwlmLiteral> &vl) {
    size_t max_flood_tail = 0;
    for (const auto &lit : vl) {
        const string &s = lit.s;
        assert(!s.empty());
        size_t j;
        for (j = 1; j < s.length(); j++) {
            if (s[s.length() - j - 1] != s[s.length() - 1]) {
                break;
            }
        }
        max_flood_tail = max(max_flood_tail, j);
    }
    return max_flood_tail;
}

/**
 * \brief True if this Teddy engine is qualified to handle this set of literals
 * on this target.
 */
static
bool isAllowed(const vector<hwlmLiteral> &vl, const TeddyEngineDescription &eng,
               const size_t max_lit_len, const target_t &target) {
    if (!eng.isValidOnTarget(target)) {
        DEBUG_PRINTF("%u disallowed: not valid on target\n", eng.getID());
        return false;
    }
    if (eng.getNumBuckets() < vl.size() && !eng.packed) {
        DEBUG_PRINTF("%u disallowed: num buckets < num lits and not packed\n",
                     eng.getID());
        return false;
    }
    if (eng.getNumBuckets() * TEDDY_BUCKET_LOAD < vl.size()) {
        DEBUG_PRINTF("%u disallowed: too many lits for num buckets\n",
                     eng.getID());
        return false;
    }
    if (eng.numMasks > max_lit_len) {
        DEBUG_PRINTF("%u disallowed: more masks than max lit len (%zu)\n",
                     eng.getID(), max_lit_len);
        return false;
    }

    if (vl.size() > 40) {
        u32 n_small_lits = 0;
        for (const auto &lit : vl) {
            if (lit.s.length() < eng.numMasks) {
                n_small_lits++;
            }
        }
        if (n_small_lits * 5 > vl.size()) {
            DEBUG_PRINTF("too many short literals (%u)\n", n_small_lits);
            return false;
        }
    }

    return true;
}

unique_ptr<TeddyEngineDescription>
chooseTeddyEngine(const target_t &target, const vector<hwlmLiteral> &vl) {
    vector<TeddyEngineDescription> descs;
    getTeddyDescriptions(&descs);
    const TeddyEngineDescription *best = nullptr;

    const size_t max_lit_len = maxLen(vl);
    const size_t max_flood_tail = maxFloodTailLen(vl);
    DEBUG_PRINTF("%zu lits, max_lit_len=%zu, max_flood_tail=%zu\n", vl.size(),
                 max_lit_len, max_flood_tail);

    u32 best_score = 0;
    for (size_t engineID = 0; engineID < descs.size(); engineID++) {
        const TeddyEngineDescription &eng = descs[engineID];
        if (!isAllowed(vl, eng, max_lit_len, target)) {
            continue;
        }

        u32 score = 0;

        // We prefer unpacked Teddy models.
        if (!eng.packed) {
            score += 100;
        }

        // If we're heavily loaded, we prefer to have more masks.
        if (vl.size() > 4 * eng.getNumBuckets()) {
            score += eng.numMasks * 4;
        } else {
            // Lightly loaded cases are great.
            score += 100;
        }

        // We want enough masks to avoid becoming flood-prone.
        if (eng.numMasks > max_flood_tail) {
            score += 50;
        }

        // We prefer having 3 masks. 3 is just right.
        score += 6 / (abs(3 - (int)eng.numMasks) + 1);

        // We prefer cheaper, smaller Teddy models.
        score += 16 / eng.getNumBuckets();

        DEBUG_PRINTF("teddy %u: masks=%u, buckets=%u, packed=%u "
                     "-> score=%u\n",
                     eng.getID(), eng.numMasks, eng.getNumBuckets(),
                     eng.packed ? 1U : 0U, score);

        if (!best || score > best_score) {
            best = &eng;
            best_score = score;
        }
    }

    if (!best) {
        DEBUG_PRINTF("failed to find engine\n");
        return nullptr;
    }

    DEBUG_PRINTF("using engine %u\n", best->getID());
    return ue2::make_unique<TeddyEngineDescription>(*best);
}

unique_ptr<TeddyEngineDescription> getTeddyDescription(u32 engineID) {
    vector<TeddyEngineDescription> descs;
    getTeddyDescriptions(&descs);

    for (const auto &desc : descs) {
        if (desc.getID() == engineID) {
            return ue2::make_unique<TeddyEngineDescription>(desc);
        }
    }

    return nullptr;
}

} // namespace ue2
