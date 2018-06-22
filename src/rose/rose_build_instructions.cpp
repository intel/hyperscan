/*
 * Copyright (c) 2017-2018, Intel Corporation
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

#include "rose_build_instructions.h"

#include "rose_build_engine_blob.h"
#include "util/multibit_build.h"
#include "util/verify_types.h"

#include <algorithm>

using namespace std;

namespace ue2 {
/* Destructors to avoid weak vtables. */

RoseInstruction::~RoseInstruction() = default;
RoseInstrCatchUp::~RoseInstrCatchUp() = default;
RoseInstrCatchUpMpv::~RoseInstrCatchUpMpv() = default;
RoseInstrSomZero::~RoseInstrSomZero() = default;
RoseInstrSuffixesEod::~RoseInstrSuffixesEod() = default;
RoseInstrMatcherEod::~RoseInstrMatcherEod() = default;
RoseInstrEnd::~RoseInstrEnd() = default;
RoseInstrClearWorkDone::~RoseInstrClearWorkDone() = default;
RoseInstrFlushCombination::~RoseInstrFlushCombination() = default;

using OffsetMap = RoseInstruction::OffsetMap;

static
u32 calc_jump(const OffsetMap &offset_map, const RoseInstruction *from,
              const RoseInstruction *to) {
    DEBUG_PRINTF("computing relative jump from %p to %p\n", from, to);
    assert(from && contains(offset_map, from));
    assert(to && contains(offset_map, to));

    u32 from_offset = offset_map.at(from);
    u32 to_offset = offset_map.at(to);
    DEBUG_PRINTF("offsets: %u -> %u\n", from_offset, to_offset);
    assert(from_offset <= to_offset);

    return to_offset - from_offset;
}

void RoseInstrAnchoredDelay::write(void *dest, RoseEngineBlob &blob,
                                   const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->groups = groups;
    inst->anch_id = anch_id;
    inst->done_jump = calc_jump(offset_map, this, target);
}

void RoseInstrCheckLitEarly::write(void *dest, RoseEngineBlob &blob,
                                   const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->min_offset = min_offset;
    inst->fail_jump = calc_jump(offset_map, this, target);
}

void RoseInstrCheckGroups::write(void *dest, RoseEngineBlob &blob,
                                 const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->groups = groups;
}

void RoseInstrCheckOnlyEod::write(void *dest, RoseEngineBlob &blob,
                                  const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->fail_jump = calc_jump(offset_map, this, target);
}

void RoseInstrCheckBounds::write(void *dest, RoseEngineBlob &blob,
                                 const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->min_bound = min_bound;
    inst->max_bound = max_bound;
    inst->fail_jump = calc_jump(offset_map, this, target);
}

void RoseInstrCheckNotHandled::write(void *dest, RoseEngineBlob &blob,
                                     const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->key = key;
    inst->fail_jump = calc_jump(offset_map, this, target);
}

void RoseInstrCheckSingleLookaround::write(void *dest, RoseEngineBlob &blob,
                                           const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->offset = offset;
    inst->reach_index = blob.lookaround_cache.get_offset_of({reach}, blob);
    inst->fail_jump = calc_jump(offset_map, this, target);
}

void RoseInstrCheckLookaround::write(void *dest, RoseEngineBlob &blob,
                                     const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    vector<s8> look_offsets;
    vector<CharReach> reaches;
    for (const auto &le : look) {
        look_offsets.push_back(le.offset);
        reaches.push_back(le.reach);
    }
    inst->look_index = blob.lookaround_cache.get_offset_of(look_offsets, blob);
    inst->reach_index = blob.lookaround_cache.get_offset_of(reaches, blob);
    inst->count = verify_u32(look.size());
    inst->fail_jump = calc_jump(offset_map, this, target);
}

void RoseInstrCheckMask::write(void *dest, RoseEngineBlob &blob,
                               const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->and_mask = and_mask;
    inst->cmp_mask = cmp_mask;
    inst->neg_mask = neg_mask;
    inst->offset = offset;
    inst->fail_jump = calc_jump(offset_map, this, target);
}

void RoseInstrCheckMask32::write(void *dest, RoseEngineBlob &blob,
                                 const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    copy(begin(and_mask), end(and_mask), inst->and_mask);
    copy(begin(cmp_mask), end(cmp_mask), inst->cmp_mask);
    inst->neg_mask = neg_mask;
    inst->offset = offset;
    inst->fail_jump = calc_jump(offset_map, this, target);
}

void RoseInstrCheckByte::write(void *dest, RoseEngineBlob &blob,
                               const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->and_mask = and_mask;
    inst->cmp_mask = cmp_mask;
    inst->negation = negation;
    inst->offset = offset;
    inst->fail_jump = calc_jump(offset_map, this, target);
}

void RoseInstrCheckShufti16x8::write(void *dest, RoseEngineBlob &blob,
                                     const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    copy(begin(nib_mask), end(nib_mask), inst->nib_mask);
    copy(begin(bucket_select_mask), end(bucket_select_mask),
         inst->bucket_select_mask);
    inst->neg_mask = neg_mask;
    inst->offset = offset;
    inst->fail_jump = calc_jump(offset_map, this, target);
}

void RoseInstrCheckShufti32x8::write(void *dest, RoseEngineBlob &blob,
                                     const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    copy(begin(hi_mask), end(hi_mask), inst->hi_mask);
    copy(begin(lo_mask), end(lo_mask), inst->lo_mask);
    copy(begin(bucket_select_mask), end(bucket_select_mask),
         inst->bucket_select_mask);

    inst->neg_mask = neg_mask;
    inst->offset = offset;
    inst->fail_jump = calc_jump(offset_map, this, target);
}

void RoseInstrCheckShufti16x16::write(void *dest, RoseEngineBlob &blob,
                                      const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    copy(begin(hi_mask), end(hi_mask), inst->hi_mask);
    copy(begin(lo_mask), end(lo_mask), inst->lo_mask);
    copy(begin(bucket_select_mask), end(bucket_select_mask),
         inst->bucket_select_mask);
    inst->neg_mask = neg_mask;
    inst->offset = offset;
    inst->fail_jump = calc_jump(offset_map, this, target);
}

void RoseInstrCheckShufti32x16::write(void *dest, RoseEngineBlob &blob,
                                      const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    copy(begin(hi_mask), end(hi_mask), inst->hi_mask);
    copy(begin(lo_mask), end(lo_mask), inst->lo_mask);
    copy(begin(bucket_select_mask_hi), end(bucket_select_mask_hi),
         inst->bucket_select_mask_hi);
    copy(begin(bucket_select_mask_lo), end(bucket_select_mask_lo),
         inst->bucket_select_mask_lo);
    inst->neg_mask = neg_mask;
    inst->offset = offset;
    inst->fail_jump = calc_jump(offset_map, this, target);
}

void RoseInstrCheckInfix::write(void *dest, RoseEngineBlob &blob,
                                const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->queue = queue;
    inst->lag = lag;
    inst->report = report;
    inst->fail_jump = calc_jump(offset_map, this, target);
}

void RoseInstrCheckPrefix::write(void *dest, RoseEngineBlob &blob,
                                 const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->queue = queue;
    inst->lag = lag;
    inst->report = report;
    inst->fail_jump = calc_jump(offset_map, this, target);
}

void RoseInstrPushDelayed::write(void *dest, RoseEngineBlob &blob,
                                 const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->delay = delay;
    inst->index = index;
}

void RoseInstrSomAdjust::write(void *dest, RoseEngineBlob &blob,
                               const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->distance = distance;
}

void RoseInstrSomLeftfix::write(void *dest, RoseEngineBlob &blob,
                                const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->queue = queue;
    inst->lag = lag;
}

void RoseInstrSomFromReport::write(void *dest, RoseEngineBlob &blob,
                                   const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->som = som;
}

void RoseInstrTriggerInfix::write(void *dest, RoseEngineBlob &blob,
                                  const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->cancel = cancel;
    inst->queue = queue;
    inst->event = event;
}

void RoseInstrTriggerSuffix::write(void *dest, RoseEngineBlob &blob,
                                   const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->queue = queue;
    inst->event = event;
}

void RoseInstrDedupe::write(void *dest, RoseEngineBlob &blob,
                            const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->quash_som = quash_som;
    inst->dkey = dkey;
    inst->offset_adjust = offset_adjust;
    inst->fail_jump = calc_jump(offset_map, this, target);
}

void RoseInstrDedupeSom::write(void *dest, RoseEngineBlob &blob,
                               const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->quash_som = quash_som;
    inst->dkey = dkey;
    inst->offset_adjust = offset_adjust;
    inst->fail_jump = calc_jump(offset_map, this, target);
}

void RoseInstrReportChain::write(void *dest, RoseEngineBlob &blob,
                                 const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->event = event;
    inst->top_squash_distance = top_squash_distance;
}

void RoseInstrReportSomInt::write(void *dest, RoseEngineBlob &blob,
                                  const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->som = som;
}

void RoseInstrReportSomAware::write(void *dest, RoseEngineBlob &blob,
                                    const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->som = som;
}

void RoseInstrReport::write(void *dest, RoseEngineBlob &blob,
                            const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->onmatch = onmatch;
    inst->offset_adjust = offset_adjust;
}

void RoseInstrReportExhaust::write(void *dest, RoseEngineBlob &blob,
                                   const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->onmatch = onmatch;
    inst->offset_adjust = offset_adjust;
    inst->ekey = ekey;
}

void RoseInstrReportSom::write(void *dest, RoseEngineBlob &blob,
                               const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->onmatch = onmatch;
    inst->offset_adjust = offset_adjust;
}

void RoseInstrReportSomExhaust::write(void *dest, RoseEngineBlob &blob,
                                      const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->onmatch = onmatch;
    inst->offset_adjust = offset_adjust;
    inst->ekey = ekey;
}

void RoseInstrDedupeAndReport::write(void *dest, RoseEngineBlob &blob,
                                     const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->quash_som = quash_som;
    inst->dkey = dkey;
    inst->onmatch = onmatch;
    inst->offset_adjust = offset_adjust;
    inst->fail_jump = calc_jump(offset_map, this, target);
}

void RoseInstrFinalReport::write(void *dest, RoseEngineBlob &blob,
                                 const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->onmatch = onmatch;
    inst->offset_adjust = offset_adjust;
}

void RoseInstrCheckExhausted::write(void *dest, RoseEngineBlob &blob,
                                    const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->ekey = ekey;
    inst->fail_jump = calc_jump(offset_map, this, target);
}

void RoseInstrCheckMinLength::write(void *dest, RoseEngineBlob &blob,
                                    const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->end_adj = end_adj;
    inst->min_length = min_length;
    inst->fail_jump = calc_jump(offset_map, this, target);
}

void RoseInstrSetState::write(void *dest, RoseEngineBlob &blob,
                              const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->index = index;
}

void RoseInstrSetGroups::write(void *dest, RoseEngineBlob &blob,
                              const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->groups = groups;
}

void RoseInstrSquashGroups::write(void *dest, RoseEngineBlob &blob,
                                  const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->groups = groups;
}

void RoseInstrCheckState::write(void *dest, RoseEngineBlob &blob,
                                const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->index = index;
    inst->fail_jump = calc_jump(offset_map, this, target);
}

void RoseInstrSparseIterBegin::write(void *dest, RoseEngineBlob &blob,
                                     const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->fail_jump = calc_jump(offset_map, this, target);

    // Resolve and write the multibit sparse iterator and the jump table.
    vector<u32> keys;
    vector<u32> jump_offsets;
    for (const auto &jump : jump_table) {
        keys.push_back(jump.first);
        assert(contains(offset_map, jump.second));
        jump_offsets.push_back(offset_map.at(jump.second));
    }

    auto iter = mmbBuildSparseIterator(keys, num_keys);
    assert(!iter.empty());
    inst->iter_offset = blob.add_iterator(iter);
    inst->jump_table = blob.add(jump_offsets.begin(), jump_offsets.end());

    // Store offsets for corresponding SPARSE_ITER_NEXT operations.
    is_written = true;
    iter_offset = inst->iter_offset;
    jump_table_offset = inst->jump_table;
}

void RoseInstrSparseIterNext::write(void *dest, RoseEngineBlob &blob,
                                    const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->state = state;
    inst->fail_jump = calc_jump(offset_map, this, target);

    // Use the same sparse iterator and jump table as the SPARSE_ITER_BEGIN
    // instruction.
    assert(begin);
    assert(contains(offset_map, begin));
    assert(begin->is_written);
    inst->iter_offset = begin->iter_offset;
    inst->jump_table = begin->jump_table_offset;
}

void RoseInstrSparseIterAny::write(void *dest, RoseEngineBlob &blob,
                                   const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->fail_jump = calc_jump(offset_map, this, target);

    // Write the multibit sparse iterator.
    auto iter = mmbBuildSparseIterator(keys, num_keys);
    assert(!iter.empty());
    inst->iter_offset = blob.add_iterator(iter);
}

void RoseInstrEnginesEod::write(void *dest, RoseEngineBlob &blob,
                                const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->iter_offset = iter_offset;
}

void RoseInstrCheckLongLit::write(void *dest, RoseEngineBlob &blob,
                                  const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    assert(!literal.empty());
    inst->lit_offset = blob.add(literal.c_str(), literal.size(), 1);
    inst->lit_length = verify_u32(literal.size());
    inst->fail_jump = calc_jump(offset_map, this, target);
}

void RoseInstrCheckLongLitNocase::write(void *dest, RoseEngineBlob &blob,
                                        const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    assert(!literal.empty());
    inst->lit_offset = blob.add(literal.c_str(), literal.size(), 1);
    inst->lit_length = verify_u32(literal.size());
    inst->fail_jump = calc_jump(offset_map, this, target);
}

void RoseInstrCheckMedLit::write(void *dest, RoseEngineBlob &blob,
                                 const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    assert(!literal.empty());
    inst->lit_offset = blob.add(literal.c_str(), literal.size(), 1);
    inst->lit_length = verify_u32(literal.size());
    inst->fail_jump = calc_jump(offset_map, this, target);
}

void RoseInstrCheckMedLitNocase::write(void *dest, RoseEngineBlob &blob,
                                       const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    assert(!literal.empty());
    inst->lit_offset = blob.add(literal.c_str(), literal.size(), 1);
    inst->lit_length = verify_u32(literal.size());
    inst->fail_jump = calc_jump(offset_map, this, target);
}

void RoseInstrMultipathLookaround::write(void *dest, RoseEngineBlob &blob,
                                         const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    auto &cache = blob.lookaround_cache;
    vector<s8> look_offsets;
    vector<vector<CharReach>> reaches;
    for (const auto &vle : multi_look) {
        reaches.push_back({});
        bool done_offset = false;

        for (const auto &le : vle) {
            reaches.back().push_back(le.reach);

            /* empty reaches don't have valid offsets */
            if (!done_offset && le.reach.any()) {
                look_offsets.push_back(le.offset);
                done_offset = true;
            }
        }
    }
    inst->look_index = cache.get_offset_of(look_offsets, blob);
    inst->reach_index = cache.get_offset_of(reaches, blob);
    inst->count = verify_u32(multi_look.size());
    inst->last_start = last_start;
    copy(begin(start_mask), end(start_mask), inst->start_mask);
    inst->fail_jump = calc_jump(offset_map, this, target);
}

void RoseInstrCheckMultipathShufti16x8::write(void *dest, RoseEngineBlob &blob,
                                          const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    copy(begin(nib_mask), end(nib_mask), inst->nib_mask);
    copy(begin(bucket_select_mask), begin(bucket_select_mask) + 16,
         inst->bucket_select_mask);
    copy(begin(data_select_mask), begin(data_select_mask) + 16,
         inst->data_select_mask);
    inst->hi_bits_mask = hi_bits_mask;
    inst->lo_bits_mask = lo_bits_mask;
    inst->neg_mask = neg_mask;
    inst->base_offset = base_offset;
    inst->last_start = last_start;
    inst->fail_jump = calc_jump(offset_map, this, target);
}

void RoseInstrCheckMultipathShufti32x8::write(void *dest, RoseEngineBlob &blob,
                                          const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    copy(begin(hi_mask), begin(hi_mask) + 16, inst->hi_mask);
    copy(begin(lo_mask), begin(lo_mask) + 16, inst->lo_mask);
    copy(begin(bucket_select_mask), begin(bucket_select_mask) + 32,
         inst->bucket_select_mask);
    copy(begin(data_select_mask), begin(data_select_mask) + 32,
         inst->data_select_mask);
    inst->hi_bits_mask = hi_bits_mask;
    inst->lo_bits_mask = lo_bits_mask;
    inst->neg_mask = neg_mask;
    inst->base_offset = base_offset;
    inst->last_start = last_start;
    inst->fail_jump = calc_jump(offset_map, this, target);
}

void RoseInstrCheckMultipathShufti32x16::write(void *dest, RoseEngineBlob &blob,
                                           const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    copy(begin(hi_mask), end(hi_mask), inst->hi_mask);
    copy(begin(lo_mask), end(lo_mask), inst->lo_mask);
    copy(begin(bucket_select_mask_hi), begin(bucket_select_mask_hi) + 32,
         inst->bucket_select_mask_hi);
    copy(begin(bucket_select_mask_lo), begin(bucket_select_mask_lo) + 32,
         inst->bucket_select_mask_lo);
    copy(begin(data_select_mask), begin(data_select_mask) + 32,
         inst->data_select_mask);
    inst->hi_bits_mask = hi_bits_mask;
    inst->lo_bits_mask = lo_bits_mask;
    inst->neg_mask = neg_mask;
    inst->base_offset = base_offset;
    inst->last_start = last_start;
    inst->fail_jump = calc_jump(offset_map, this, target);
}

void RoseInstrCheckMultipathShufti64::write(void *dest, RoseEngineBlob &blob,
                                            const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    copy(begin(hi_mask), begin(hi_mask) + 16, inst->hi_mask);
    copy(begin(lo_mask), begin(lo_mask) + 16, inst->lo_mask);
    copy(begin(bucket_select_mask), end(bucket_select_mask),
         inst->bucket_select_mask);
    copy(begin(data_select_mask), end(data_select_mask),
         inst->data_select_mask);
    inst->hi_bits_mask = hi_bits_mask;
    inst->lo_bits_mask = lo_bits_mask;
    inst->neg_mask = neg_mask;
    inst->base_offset = base_offset;
    inst->last_start = last_start;
    inst->fail_jump = calc_jump(offset_map, this, target);
}

void RoseInstrIncludedJump::write(void *dest, RoseEngineBlob &blob,
                                  const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->child_offset = child_offset;
    inst->squash = squash;
}

void RoseInstrSetLogical::write(void *dest, RoseEngineBlob &blob,
                                const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->lkey = lkey;
    inst->offset_adjust = offset_adjust;
}

void RoseInstrSetCombination::write(void *dest, RoseEngineBlob &blob,
                                    const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->ckey = ckey;
}

void RoseInstrSetExhaust::write(void *dest, RoseEngineBlob &blob,
                                const OffsetMap &offset_map) const {
    RoseInstrBase::write(dest, blob, offset_map);
    auto *inst = static_cast<impl_type *>(dest);
    inst->ekey = ekey;
}

}
