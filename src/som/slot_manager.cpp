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

/**
 * \file
 * \brief SOM Slot Manager.
 */

#include "slot_manager.h"

#include "slot_manager_internal.h"
#include "ue2common.h"
#include "nfagraph/ng_holder.h"
#include "nfagraph/ng_is_equal.h"
#include "nfagraph/ng_som_util.h"
#include "nfagraph/ng_region.h"
#include "util/charreach.h"
#include "util/hash.h"
#include "util/make_unique.h"
#include "util/dump_charclass.h"
#include "util/verify_types.h"

#include <cassert>
#include <deque>
#include <utility>

using namespace std;

namespace ue2 {

/** \brief Define this to disable the cache and have everyone get their own
 * SOM slot. */
//#define NO_SLOT_CACHING

SlotCacheEntry::SlotCacheEntry(const NGHolder &prefix_in,
                               const CharReach &escapes_in, u32 parent_in,
                               bool is_reset_in, u32 slot_in)
    : prefix(cloneHolder(prefix_in)), escapes(escapes_in),
      parent_slot(parent_in), is_reset(is_reset_in), slot(slot_in) {}

size_t SlotEntryHasher::operator()(const SlotCacheEntry &e) const {
    assert(e.prefix);

    size_t v = hash_all(hash_holder(*e.prefix), e.parent_slot,
                        e.is_reset, e.escapes);

    DEBUG_PRINTF("%zu vertices, parent_slot=%u, escapes=%s, is_reset=%d "
                 "hashes to %zx\n", num_vertices(*e.prefix), e.parent_slot,
                 describeClass(e.escapes, 10, CC_OUT_TEXT).c_str(),
                 (int)e.is_reset, v);
    return v;
}

bool SlotEntryEqual::operator()(const SlotCacheEntry &a,
                                const SlotCacheEntry &b) const {
    assert(a.prefix);
    assert(b.prefix);
    return a.parent_slot == b.parent_slot
        && a.is_reset == b.is_reset
        && a.escapes == b.escapes
        && is_equal(*a.prefix, *b.prefix);
    // NOTE: slot not compared.
}

void SlotCache::insert(const NGHolder &prefix, const CharReach &escapes,
                       u32 parent_slot, bool is_reset, u32 slot) {
    store.emplace(prefix, escapes, parent_slot, is_reset, slot);
}

const SlotCacheEntry *SlotCache::find(const NGHolder &prefix,
                                      const CharReach &escapes, u32 parent_slot,
                                      bool is_reset) {
    SlotCacheEntry entry(prefix, escapes, parent_slot, is_reset,
                         0 /* unused for searching with SlotEntryEqual */);
    CacheStore::const_iterator it = store.find(entry);
    if (it != store.end()) {
        return &(*it);
    }
    return nullptr;
}

SomSlotManager::SomSlotManager(u8 p)
    : nextSomSlot(0), cache(ue2::make_unique<SlotCache>()), historyRequired(0),
      precision(p) {}

SomSlotManager::~SomSlotManager() { }

u32 SomSlotManager::getSomSlot(const NGHolder &prefix,
                               const CharReach &escapes, bool is_reset,
                               u32 parent_slot) {
    assert(parent_slot == NO_PARENT || parent_slot < nextSomSlot);

    DEBUG_PRINTF("prefix with %zu vertices, parent_slot=%u\n",
                 num_vertices(prefix), parent_slot);
    DEBUG_PRINTF("nextSomSlot=%u\n", nextSomSlot);

#ifdef NO_SLOT_CACHING
    return nextSomSlot++;
#endif

    const SlotCacheEntry *entry =
        cache->find(prefix, escapes, parent_slot, is_reset);
    if (entry) {
        DEBUG_PRINTF("cache hit: slot %u\n", entry->slot);
        return entry->slot;
    }

    DEBUG_PRINTF("cache miss: handing out new slot %u\n", nextSomSlot);
    cache->insert(prefix, escapes, parent_slot, is_reset, nextSomSlot);
    return nextSomSlot++;
}

u32 SomSlotManager::getInitialResetSomSlot(const NGHolder &prefix,
                const NGHolder &g,
                const unordered_map<NFAVertex, u32> &region_map,
                u32 last_sent_region, bool *prefix_already_implemented) {
    DEBUG_PRINTF("getting initial reset; last sent region %u\n",
                 last_sent_region);
    assert(last_sent_region);
    assert(!hasBigCycles(prefix));
    *prefix_already_implemented = false;

#ifdef NO_SLOT_CACHING
    return nextSomSlot++;
#endif

    shared_ptr<const NGHolder> pp = cloneHolder(prefix);
    assert(hash_holder(*pp) == hash_holder(prefix));

    auto hs_it = cache->initial_prefixes.find(pp);
    if (hs_it != cache->initial_prefixes.end()) {
        DEBUG_PRINTF("pulling from cache\n");
        pp = *hs_it;
    } else {
        DEBUG_PRINTF("storing in cache entry %zu, hash=%llu\n",
                     cache->initial_prefixes.size(), hash_holder(*pp));
        cache->initial_prefixes.insert(pp);
    }

    // Clone a copy of g (and its region map) that we will be able to store
    // later on.
    shared_ptr<NGHolder> gg = make_shared<NGHolder>();
    unordered_map<NFAVertex, NFAVertex> orig_to_copy;
    cloneHolder(*gg, g, &orig_to_copy);
    unordered_map<NFAVertex, u32> gg_region_map;
    for (const auto &m : region_map) {
        assert(contains(region_map, m.first));
        gg_region_map.emplace(orig_to_copy.at(m.first), m.second);
    }

    u32 first_bad_region = ~0U;
    UNUSED bool rv = sentClearsTail(g, region_map, *pp, last_sent_region,
                                    &first_bad_region);
    assert(!rv || first_bad_region == ~0U);

    InitialResetInfo *ir = nullptr;

    for (auto &reset : cache->initial_resets) {
        /* is this prefix already in our list? */
        auto has_prefix_func =
            [&pp](const InitialResetEntry &e) { return e.sent == pp; };
        bool already_seen_prefix =
            find_if(reset.entries.begin(), reset.entries.end(),
                    has_prefix_func) != reset.entries.end();

        for (auto &e : reset.entries) {
            u32 temp = 0;
            /* we don't need to test against sentinels which are identical to
             * our current one as races don't matter and we know it clears
             * sufficiently. */
            if (e.sent != pp &&
                !sentClearsTail(g, region_map, *e.sent, last_sent_region - 1,
                                &temp) &&
                (temp < first_bad_region || first_bad_region == ~0U)) {
                goto try_next;
            }

            /* if we have already seen the prefix it must be fine */
            if (!already_seen_prefix &&
                !sentClearsTail(*e.body, e.body_regions, prefix,
                                e.sent_region - 1, &temp) &&
                (temp < e.first_bad_region || e.first_bad_region == ~0U)) {
                goto try_next;
            }
        }
        DEBUG_PRINTF("sharing\n");
        if (already_seen_prefix) {
            /* if we have already created this prefix using this som slot, we
             * can avoid creating another copy of the prefix. */
            *prefix_already_implemented = true;
        }
        ir = &reset;
        goto found;
    try_next:;
    }

    cache->initial_resets.emplace_back(nextSomSlot++);
    ir = &cache->initial_resets.back();

found:
    ir->entries.emplace_back(pp, gg, gg_region_map, last_sent_region,
                             first_bad_region);
    return ir->slot;
}

u32 SomSlotManager::getPrivateSomSlot(void) {
    return nextSomSlot++;
}

void SomSlotManager::rollbackSomTo(u32 num) {
    assert(nextSomSlot >= num);
    nextSomSlot = num;
}

u32 SomSlotManager::numSomSlots() const {
    return nextSomSlot;
}

u32 SomSlotManager::addRevNfa(bytecode_ptr<NFA> nfa, u32 maxWidth) {
    u32 rv = verify_u32(rev_nfas.size());
    rev_nfas.push_back(move(nfa));

    // A rev nfa commits us to having enough history around to handle its
    // max width.
    historyRequired = max(historyRequired, maxWidth);

    return rv;
}

} // namespace ue2
