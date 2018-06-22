/*
 * Copyright (c) 2015-2018, Intel Corporation
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
 * \brief Rose runtime: code for catching up output-exposed engines.
 *
 * Rose has several components which run behind the main (floating table) clock
 * and need to be caught up before we report matches.
 *
 * Currently we have to deal with:
 * 1. Suffix/Outfix NFAs
 * 2. A single MPV NFA (chained), which may also be triggered by (1).
 *
 * The approach is to:
 * - (A) build a priority queue of the suffix/outfixes based on their first
 *       match location;
 * - (B) process the matches from the priority queue in order;
 * - (C) As we report matches from (B) we interleave matches from the MPV if it
 *       exists.
 */

#ifndef ROSE_CATCHUP_H
#define ROSE_CATCHUP_H

#include "hwlm/hwlm.h"
#include "runtime.h"
#include "scratch.h"
#include "rose.h"
#include "rose_common.h"
#include "rose_internal.h"
#include "ue2common.h"
#include "util/multibit.h"

hwlmcb_rv_t roseCatchUpAll(s64a loc, struct hs_scratch *scratch);

/* will only catch mpv up to last reported external match */
hwlmcb_rv_t roseCatchUpSuf(s64a loc, struct hs_scratch *scratch);

hwlmcb_rv_t roseCatchUpMPV_i(const struct RoseEngine *t, s64a loc,
                             struct hs_scratch *scratch);

void blockInitSufPQ(const struct RoseEngine *t, char *state,
                    struct hs_scratch *scratch, char is_small_block);
void streamInitSufPQ(const struct RoseEngine *t, char *state,
                     struct hs_scratch *scratch);

static really_inline
int canSkipCatchUpMPV(const struct RoseEngine *t, struct hs_scratch *scratch,
                      u64a cur_offset) {
    if (!has_chained_nfas(t)) {
        return 1;
    }

    /* note: we may have to run at less than tctxt.minMatchOffset as we may
     * have a full queue of postponed events that we need to flush */
    if (cur_offset < scratch->tctxt.next_mpv_offset) {
        DEBUG_PRINTF("skipping cur_offset %llu min %llu, mpv %llu\n",
                      cur_offset, scratch->tctxt.minMatchOffset,
                      scratch->tctxt.next_mpv_offset);
        return 1;
    }

    assert(t->activeArrayCount);

    DEBUG_PRINTF("cur offset offset: %llu\n", cur_offset);
    DEBUG_PRINTF("min match offset %llu\n", scratch->tctxt.minMatchOffset);

    assert(t->outfixBeginQueue == 1); /* if it exists mpv is queue 0 */

    const u8 *aa = getActiveLeafArray(t, scratch->core_info.state);
    return !mmbit_isset(aa, t->activeArrayCount, 0);
}

/** \brief Catches up the MPV. */
static really_inline
hwlmcb_rv_t roseCatchUpMPV(const struct RoseEngine *t, s64a loc,
                           struct hs_scratch *scratch) {
    u64a cur_offset = loc + scratch->core_info.buf_offset;
    assert(cur_offset >= scratch->tctxt.minMatchOffset);
    assert(!can_stop_matching(scratch));

    if (canSkipCatchUpMPV(t, scratch, cur_offset)) {
        if (t->flushCombProgramOffset) {
            if (roseRunFlushCombProgram(t, scratch, cur_offset)
                    == HWLM_TERMINATE_MATCHING) {
                return HWLM_TERMINATE_MATCHING;
            }
        }
        updateMinMatchOffsetFromMpv(&scratch->tctxt, cur_offset);
        return HWLM_CONTINUE_MATCHING;
    }

    /* Note: chained tails MUST not participate in the priority queue as
     * they may have events pushed on during this process which may be before
     * the catch up point */

    return roseCatchUpMPV_i(t, loc, scratch);
}

/** \brief Catches up NFAs and the MPV. */
static rose_inline
hwlmcb_rv_t roseCatchUpTo(const struct RoseEngine *t,
                          struct hs_scratch *scratch, u64a end) {
    /* no need to catch up if we are at the same offset as last time */
    if (end <= scratch->tctxt.minMatchOffset) {
        /* we must already be up to date */
        DEBUG_PRINTF("skip\n");
        return HWLM_CONTINUE_MATCHING;
    }

    char *state = scratch->core_info.state;
    s64a loc = end - scratch->core_info.buf_offset;

    if (end <= scratch->tctxt.minNonMpvMatchOffset) {
        /* only need to catch up the mpv */
        return roseCatchUpMPV(t, loc, scratch);
    }

    assert(scratch->tctxt.minMatchOffset >= scratch->core_info.buf_offset);
    hwlmcb_rv_t rv;
    if (!t->activeArrayCount
        || !mmbit_any(getActiveLeafArray(t, state), t->activeArrayCount)) {
        if (t->flushCombProgramOffset) {
            if (roseRunFlushCombProgram(t, scratch, end)
                    == HWLM_TERMINATE_MATCHING) {
                return HWLM_TERMINATE_MATCHING;
            }
        }
        updateMinMatchOffset(&scratch->tctxt, end);
        rv = HWLM_CONTINUE_MATCHING;
    } else {
        rv = roseCatchUpAll(loc, scratch);
    }

    assert(rv != HWLM_CONTINUE_MATCHING
           || scratch->tctxt.minMatchOffset == end);
    assert(rv != HWLM_CONTINUE_MATCHING
           || scratch->tctxt.minNonMpvMatchOffset == end);
    assert(!can_stop_matching(scratch) || rv == HWLM_TERMINATE_MATCHING);
    return rv;
}

/**
 * \brief Catches up anything which may add triggers on the MPV (suffixes and
 * outfixes).
 *
 * The MPV will be run only to intersperse matches in the output match stream
 * if external matches are raised.
 */
static rose_inline
hwlmcb_rv_t roseCatchUpMpvFeeders(const struct RoseEngine *t,
                                  struct hs_scratch *scratch, u64a end) {
    /* no need to catch up if we are at the same offset as last time */
    if (end <= scratch->tctxt.minNonMpvMatchOffset) {
        /* we must already be up to date */
        DEBUG_PRINTF("skip\n");
        return HWLM_CONTINUE_MATCHING;
    }

    s64a loc = end - scratch->core_info.buf_offset;

    assert(t->activeArrayCount); /* mpv is in active array */
    assert(scratch->tctxt.minMatchOffset >= scratch->core_info.buf_offset);

    if (!t->mpvTriggeredByLeaf) {
        /* no need to check as they never put triggers onto the mpv */
        return HWLM_CONTINUE_MATCHING;
    }

    /* sadly, this branch rarely gets taken as the mpv itself is usually
     * alive. */
    char *state = scratch->core_info.state;
    if (!mmbit_any(getActiveLeafArray(t, state), t->activeArrayCount)) {
        scratch->tctxt.minNonMpvMatchOffset = end;
        return HWLM_CONTINUE_MATCHING;
    }

    return roseCatchUpSuf(loc, scratch);
}

#endif
