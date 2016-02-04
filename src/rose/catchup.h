/*
 * Copyright (c) 2015-2016, Intel Corporation
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

#ifndef ROSE_CATCHUP_H
#define ROSE_CATCHUP_H

#include "hwlm/hwlm.h"
#include "runtime.h"
#include "scratch.h"
#include "rose_common.h"
#include "rose_internal.h"
#include "ue2common.h"
#include "nfa/nfa_internal.h"
#include "util/bitutils.h"
#include "util/multibit.h"

/*
 * Rose has several components which run behind the main (floating table) clock
 * and need to be caught up before we report matches.
 *
 * Currently we have to deal with:
 * 1) Stored matches from the anchored matcher
 * 2) Suffix/Outfix nfas
 * 3) a single MPV nfa (chained) (which may also be triggered by (1) and (2)).
 *
 * The approach is to:
 * A) build a priority queue of the suffix/outfixes based on their first match
 *    location
 * B) process the matches from the anchored matches in order
 * C) As we report a match from (B) we interleave matches from the suffixes
 * D) As we report matches from (B) and (C) we interleave matches from the
 *    mpv if it exists.
 */

/* Callbacks, defined in catchup.c */

hwlmcb_rv_t roseCatchUpSufAndChains(s64a loc, struct hs_scratch *scratch);

hwlmcb_rv_t roseCatchUpAll(s64a loc, struct hs_scratch *scratch);

hwlmcb_rv_t roseCatchUpAnchoredOnly(s64a loc, struct hs_scratch *scratch);


/* will only catch mpv upto last reported external match */
hwlmcb_rv_t roseCatchUpSuf(s64a loc, struct hs_scratch *scratch);

/* will only catch mpv upto last reported external match */
hwlmcb_rv_t roseCatchUpAnchoredAndSuf(s64a loc, struct hs_scratch *scratch);

hwlmcb_rv_t roseCatchUpMPV_i(const struct RoseEngine *t, s64a loc,
                             struct hs_scratch *scratch);

void blockInitSufPQ(const struct RoseEngine *t, char *state,
                    struct hs_scratch *scratch, char is_small_block);
void streamInitSufPQ(const struct RoseEngine *t, char *state,
                     struct hs_scratch *scratch);

static really_inline
hwlmcb_rv_t roseCatchUpMPV(const struct RoseEngine *t, s64a loc,
                           struct hs_scratch *scratch) {
    u64a cur_offset = loc + scratch->core_info.buf_offset;
    assert(cur_offset >= scratch->tctxt.minMatchOffset);

    if (0) {
    quick_exit:
        updateMinMatchOffsetFromMpv(&scratch->tctxt, cur_offset);
        return HWLM_CONTINUE_MATCHING;
    }

    if (!has_chained_nfas(t)) {
        goto quick_exit;
    }

    /* note: we may have to run at less than tctxt.minMatchOffset as we may
     * have a full queue of postponed events that we need to flush */
    if (cur_offset < scratch->tctxt.next_mpv_offset) {
        DEBUG_PRINTF("skipping cur_offset %lld min %lld, mpv %lld\n",
                      cur_offset, scratch->tctxt.minMatchOffset,
                      scratch->tctxt.next_mpv_offset);
        goto quick_exit;
    }

    assert(t->activeArrayCount);

    DEBUG_PRINTF("cur offset offset: %lld\n", cur_offset);
    DEBUG_PRINTF("min match offset %llu\n", scratch->tctxt.minMatchOffset);

    DEBUG_PRINTF("roseCatchUpMPV to %lld\n", loc);

    assert(t->outfixBeginQueue == 1); /* if it exists mpv is queue 0 */

    u8 *aa = getActiveLeafArray(t, scratch->core_info.state);
    u32 aaCount = t->activeArrayCount;

    if (!mmbit_isset(aa, aaCount, 0)){
        goto quick_exit;
    }

    /* Note: chained tails MUST not participate in the priority queue as
     * they may have events pushed on during this process which may be before
     * the catch up point */

    return roseCatchUpMPV_i(t, loc, scratch);
}

static really_inline
u64a currentAnchoredEnd(const struct RoseEngine *t, struct RoseContext *tctxt) {
    if (tctxt->curr_anchored_loc == MMB_INVALID) {
        return ANCHORED_MATCH_SENTINEL;
    } else {
        return tctxt->curr_anchored_loc + t->maxSafeAnchoredDROffset + 1;
    }
}

/* catches up nfas, anchored matches and the mpv */
static rose_inline
hwlmcb_rv_t roseCatchUpTo(const struct RoseEngine *t,
                          struct hs_scratch *scratch, u64a end,
                          char in_anchored) {
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
    u64a curr_anchored_end = currentAnchoredEnd(t, &scratch->tctxt);
    hwlmcb_rv_t rv;
    if (in_anchored
        || curr_anchored_end == ANCHORED_MATCH_SENTINEL
        || curr_anchored_end > end) {
        if (!t->activeArrayCount
            || !mmbit_any(getActiveLeafArray(t, state), t->activeArrayCount)) {
            updateMinMatchOffset(&scratch->tctxt, end);
            rv = HWLM_CONTINUE_MATCHING;
        } else {
            rv = roseCatchUpSufAndChains(loc, scratch);
        }
    } else {
        if (!t->activeArrayCount) {
            rv = roseCatchUpAnchoredOnly(loc, scratch);
        } else {
            rv = roseCatchUpAll(loc, scratch);
        }
    }

    assert(rv != HWLM_CONTINUE_MATCHING
           || scratch->tctxt.minMatchOffset == end);
    assert(rv != HWLM_CONTINUE_MATCHING
           || scratch->tctxt.minNonMpvMatchOffset == end);
    return rv;
}

/* Catches up anything which may add triggers on the mpv: anchored matches
 * and suf/outfixes. The MPV will be run only to intersperse matches in
 * the output match stream if external matches are raised. */
static rose_inline
hwlmcb_rv_t roseCatchUpMpvFeeders(const struct RoseEngine *t,
                                  struct hs_scratch *scratch, u64a end,
                                  char in_anchored) {
    /* no need to catch up if we are at the same offset as last time */
    if (end <= scratch->tctxt.minNonMpvMatchOffset) {
        /* we must already be up to date */
        DEBUG_PRINTF("skip\n");
        return HWLM_CONTINUE_MATCHING;
    }

    s64a loc = end - scratch->core_info.buf_offset;

    assert(t->activeArrayCount); /* mpv is in active array */
    assert(scratch->tctxt.minMatchOffset >= scratch->core_info.buf_offset);
    u64a curr_anchored_end = currentAnchoredEnd(t, &scratch->tctxt);
    if (in_anchored
        || curr_anchored_end == ANCHORED_MATCH_SENTINEL
        || curr_anchored_end > end) {
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
    } else {
        return roseCatchUpAnchoredAndSuf(loc, scratch);
    }
}

#endif
