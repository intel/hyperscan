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

#ifndef SLOT_MANAGER_INTERNAL_H
#define SLOT_MANAGER_INTERNAL_H

#include "nfagraph/ng.h"
#include "nfagraph/ng_is_equal.h"
#include "util/charreach.h"
#include "ue2common.h"

#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace ue2 {

struct InitialResetEntry {
    InitialResetEntry(std::shared_ptr<const NGHolder> sent_in,
                      std::shared_ptr<const NGHolder> body_in,
                      const std::unordered_map<NFAVertex, u32> &body_regions_in,
                      u32 sent_region_in, u32 first_bad_region_in)
        : sent(sent_in), body(body_in), body_regions(body_regions_in),
          sent_region(sent_region_in), first_bad_region(first_bad_region_in) {}

    std::shared_ptr<const NGHolder> sent;
    std::shared_ptr<const NGHolder> body;
    std::unordered_map<NFAVertex, u32> body_regions;
    u32 sent_region;
    u32 first_bad_region; /* ~0U if it must cover the whole g */
};

struct InitialResetInfo {
    explicit InitialResetInfo(u32 slot_in) : slot(slot_in) {}

    std::vector<InitialResetEntry> entries;
    u32 slot;
};

struct SlotCacheEntry {
    // We store our own copy of the prefix so we control its lifetime. A
    // pointer is used so that this entry can be placed in STL containers, as
    // NGHolder is not copy-constructible.
    SlotCacheEntry(const NGHolder &prefix_in, const CharReach &escapes_in,
                   u32 parent_in, bool is_reset_in, u32 slot_in);

    std::unique_ptr<const NGHolder> prefix;
    CharReach escapes;
    u32 parent_slot;
    bool is_reset;
    u32 slot;
};

struct SlotEntryHasher {
    size_t operator()(const SlotCacheEntry &e) const;
};

struct SlotEntryEqual {
    bool operator()(const SlotCacheEntry &a, const SlotCacheEntry &b) const;
};

struct SlotCache {
    typedef std::unordered_set<SlotCacheEntry, SlotEntryHasher,
                               SlotEntryEqual> CacheStore;

    void insert(const NGHolder &prefix, const CharReach &escapes,
                u32 parent_slot, bool is_reset, u32 slot);

    const SlotCacheEntry *find(const NGHolder &prefix, const CharReach &escapes,
                               u32 parent_slot, bool is_reset);

    CacheStore store;

    std::unordered_set<std::shared_ptr<const NGHolder>, NGHolderHasher,
                  NGHolderEqual> initial_prefixes;
    std::vector<InitialResetInfo> initial_resets;
};

} // namespace ue2

#endif
