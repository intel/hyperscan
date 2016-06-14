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

#include "catchup.h"
#include "match.h"
#include "program_runtime.h"
#include "rose.h"
#include "util/fatbit.h"

static really_inline
void initContext(const struct RoseEngine *t, u64a offset,
                 struct hs_scratch *scratch) {
    struct RoseContext *tctxt = &scratch->tctxt;
    /* TODO: diff groups for eod */
    tctxt->groups = loadGroups(t, scratch->core_info.state);
    tctxt->lit_offset_adjust = scratch->core_info.buf_offset
                             - scratch->core_info.hlen
                             + 1; // index after last byte
    tctxt->delayLastEndOffset = offset;
    tctxt->lastEndOffset = offset;
    tctxt->filledDelayedSlots = 0;
    tctxt->lastMatchOffset = 0;
    tctxt->minMatchOffset = offset;
    tctxt->minNonMpvMatchOffset = offset;
    tctxt->next_mpv_offset = offset;

    scratch->catchup_pq.qm_size = 0;
    scratch->al_log_sum = 0; /* clear the anchored logs */

    fatbit_clear(scratch->aqa);
}

void roseStreamEodExec(const struct RoseEngine *t, u64a offset,
                       struct hs_scratch *scratch) {
    assert(scratch);
    assert(t->requiresEodCheck);
    DEBUG_PRINTF("ci buf %p/%zu his %p/%zu\n", scratch->core_info.buf,
                 scratch->core_info.len, scratch->core_info.hbuf,
                 scratch->core_info.hlen);

    // We should not have been called if we've already been told to terminate
    // matching.
    assert(!told_to_stop_matching(scratch));

    if (t->maxBiAnchoredWidth != ROSE_BOUND_INF
        && offset > t->maxBiAnchoredWidth) {
        DEBUG_PRINTF("bailing, we are beyond max width\n");
        /* also some of the history/state may be stale */
        return;
    }

    if (!t->eodProgramOffset) {
        DEBUG_PRINTF("no eod program\n");
        return;
    }

    initContext(t, offset, scratch);

    DEBUG_PRINTF("running eod program at %u\n", t->eodProgramOffset);

    // There should be no pending delayed literals.
    assert(!scratch->tctxt.filledDelayedSlots);

    const u64a som = 0;
    const size_t match_len = 0;
    const char in_anchored = 0;
    const char in_catchup = 0;
    const char from_mpv = 0;
    const char skip_mpv_catchup = 1;

    // Note: we ignore the result, as this is the last thing to ever happen on
    // a scan.
    roseRunProgram(t, scratch, t->eodProgramOffset, som, offset, match_len,
                   in_anchored, in_catchup, from_mpv, skip_mpv_catchup);
}
