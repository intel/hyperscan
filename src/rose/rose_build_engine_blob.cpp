/*
 * Copyright (c) 2017, Intel Corporation
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

#include "rose_build_engine_blob.h"

#include "rose_build_lookaround.h"
#include "util/charreach_util.h"

using namespace std;

namespace ue2 {

u32 lookaround_info::get_offset_of(const vector<vector<CharReach>> &reaches,
                                   RoseEngineBlob &blob) {
    assert(reaches.size() != 1);

    // Check the cache.
    auto it = multi_cache.find(reaches);
    if (it != multi_cache.end()) {
        DEBUG_PRINTF("reusing reach at idx %u\n", it->second);
        return it->second;
    }

    vector<u8> raw_reach(reaches.size() * MULTI_REACH_BITVECTOR_LEN);
    size_t off = 0;
    for (const auto &m : reaches) {
        u8 u = 0;
        assert(m.size() == MAX_LOOKAROUND_PATHS);
        for (size_t i = 0; i < m.size(); i++) {
            if (m[i].none()) {
                u |= (u8)1U << i;
            }
        }
        fill_n(raw_reach.data() + off, MULTI_REACH_BITVECTOR_LEN, u);

        for (size_t i = 0; i < m.size(); i++) {
            const CharReach &cr = m[i];
            if (cr.none()) {
                continue;
            }

            for (size_t c = cr.find_first(); c != cr.npos;
                 c = cr.find_next(c)) {
                raw_reach[c + off] |= (u8)1U << i;
            }
        }

        off += MULTI_REACH_BITVECTOR_LEN;
    }

    u32 reach_idx = blob.add_range(raw_reach);
    DEBUG_PRINTF("adding reach at idx %u\n", reach_idx);
    multi_cache.emplace(reaches, reach_idx);

    return reach_idx;
}

u32 lookaround_info::get_offset_of(const vector<CharReach> &reach,
                                   RoseEngineBlob &blob) {
    if (contains(rcache, reach)) {
        u32 offset = rcache[reach];
        DEBUG_PRINTF("reusing reach at idx %u\n", offset);
        return offset;
    }

    vector<u8> raw_reach(reach.size() * REACH_BITVECTOR_LEN);
    size_t off = 0;
    for (const auto &cr : reach) {
        assert(cr.any()); // Should be at least one character!
        fill_bitvector(cr, raw_reach.data() + off);
        off += REACH_BITVECTOR_LEN;
    }

    u32 offset = blob.add_range(raw_reach);
    rcache.emplace(reach, offset);
    return offset;
}

u32 lookaround_info::get_offset_of(const vector<s8> &look,
                                   RoseEngineBlob &blob) {
    if (contains(lcache, look)) {
        u32 offset = lcache[look];
        DEBUG_PRINTF("reusing look at idx %u\n", offset);
        return offset;
    }

    u32 offset = blob.add_range(look);
    lcache.emplace(look, offset);
    return offset;
}

} // namespace ue2
