/*
 * Copyright (c) 2016-2017, Intel Corporation
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

/**
 * \file
 * \brief Tamarama: container engine for exclusive engines, compiler code.
 */

#include "config.h"

#include "tamaramacompile.h"

#include "tamarama_internal.h"
#include "nfa_internal.h"
#include "nfa_api_queue.h"
#include "repeatcompile.h"
#include "util/container.h"
#include "util/verify_types.h"

using namespace std;

namespace ue2 {

static
void remapTops(const TamaInfo &tamaInfo,
               vector<u32> &top_base,
               map<pair<const NFA *, u32>, u32> &out_top_remap) {
    u32 i = 0;
    u32 cur = 0;
    for (const auto &sub : tamaInfo.subengines) {
        u32 base = cur;
        top_base.push_back(base + MQE_TOP_FIRST);
        DEBUG_PRINTF("subengine:%u\n", i);
        for (const auto &t : tamaInfo.tops[i++]) {
            cur = base + t;
            DEBUG_PRINTF("top remapping %u:%u\n", t ,cur);
            out_top_remap.emplace(make_pair(sub, t), cur++);
        }
    }
}

/**
 * update stream state and scratch state sizes and copy in
 * subengines in Tamarama.
 */
static
void copyInSubnfas(const char *base_offset, NFA &nfa,
                   const TamaInfo &tamaInfo, u32 *offsets,
                   char *sub_nfa_offset, const u32 activeIdxSize) {
    u32 maxStreamStateSize = 0;
    u32 maxScratchStateSize = 0;
    sub_nfa_offset = ROUNDUP_PTR(sub_nfa_offset, 64);
    bool infinite_max_width = false;
    for (auto &sub : tamaInfo.subengines) {
        u32 streamStateSize = verify_u32(sub->streamStateSize);
        u32 scratchStateSize = verify_u32(sub->scratchStateSize);
        maxStreamStateSize = max(maxStreamStateSize, streamStateSize);
        maxScratchStateSize = max(maxScratchStateSize, scratchStateSize);
        sub->queueIndex = nfa.queueIndex;

        memcpy(sub_nfa_offset, sub, sub->length);
        *offsets = verify_u32(sub_nfa_offset - base_offset);
        DEBUG_PRINTF("type:%u offsets:%u\n", sub->type, *offsets);
        ++offsets;
        sub_nfa_offset += ROUNDUP_CL(sub->length);

        // update nfa properties
        nfa.flags |= sub->flags;
        if (!sub->maxWidth) {
            infinite_max_width = true;
        } else if (!infinite_max_width) {
            nfa.maxWidth = max(nfa.maxWidth, sub->maxWidth);
        }
    }

    if (infinite_max_width) {
        nfa.maxWidth = 0;
    }
    nfa.maxBiAnchoredWidth = 0;
    nfa.streamStateSize = activeIdxSize + maxStreamStateSize;
    nfa.scratchStateSize = maxScratchStateSize;
}

/**
 * Take in a collection of exclusive sub engines and produces a tamarama, also
 * returns via out_top_remap, a mapping indicating how tops in the subengines in
 * relate to the tamarama's tops.
 */
bytecode_ptr<NFA>
buildTamarama(const TamaInfo &tamaInfo, const u32 queue,
              map<pair<const NFA *, u32>, u32> &out_top_remap) {
    vector<u32> top_base;
    remapTops(tamaInfo, top_base, out_top_remap);

    size_t subSize = tamaInfo.subengines.size();
    DEBUG_PRINTF("subSize:%zu\n", subSize);
    size_t total_size =
        sizeof(NFA) +               // initial NFA structure
        sizeof(Tamarama) +          // Tamarama structure
        sizeof(u32) * subSize +     // base top event value for subengines,
                                    // used for top remapping at runtime
        sizeof(u32) * subSize + 64; // offsets to subengines in bytecode and
                                    // padding for subengines

    for (const auto &sub : tamaInfo.subengines) {
        total_size += ROUNDUP_CL(sub->length);
    }

    // use subSize as a sentinel value for no active subengines,
    // so add one to subSize here
    u32 activeIdxSize = calcPackedBytes(subSize + 1);
    auto nfa = make_zeroed_bytecode_ptr<NFA>(total_size);
    nfa->type = verify_u8(TAMARAMA_NFA);
    nfa->length = verify_u32(total_size);
    nfa->queueIndex = queue;

    char *ptr = (char *)nfa.get() + sizeof(NFA);
    char *base_offset = ptr;
    Tamarama *t = (Tamarama *)ptr;
    t->numSubEngines = verify_u32(subSize);
    t->activeIdxSize = verify_u8(activeIdxSize);

    ptr += sizeof(Tamarama);
    copy_bytes(ptr, top_base);
    ptr += byte_length(top_base);

    u32 *offsets = (u32 *)ptr;
    char *sub_nfa_offset = ptr + sizeof(u32) * subSize;
    copyInSubnfas(base_offset, *nfa, tamaInfo, offsets, sub_nfa_offset,
                  activeIdxSize);
    assert((size_t)(sub_nfa_offset - (char *)nfa.get()) <= total_size);
    return nfa;
}

set<ReportID> all_reports(const TamaProto &proto) {
    return proto.reports;
}

void TamaInfo::add(NFA *sub, const set<u32> &top) {
    assert(subengines.size() < max_occupancy);
    subengines.push_back(sub);
    tops.push_back(top);
}

void TamaProto::add(const NFA *n, const u32 id, const u32 top,
                    const map<pair<const NFA *, u32>, u32> &out_top_remap) {
    top_remap.emplace(make_pair(id, top), out_top_remap.at(make_pair(n, top)));
}

} // namespace ue2

