/*
 * Copyright (c) 2015, Intel Corporation
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

#ifndef ROSE_MATCH_H
#define ROSE_MATCH_H

#include "hwlm/hwlm.h"
#include "runtime.h"
#include "scratch.h"
#include "rose_common.h"
#include "rose_internal.h"
#include "ue2common.h"
#include "nfa/nfa_api.h"
#include "nfa/nfa_api_queue.h"
#include "nfa/nfa_api_util.h"
#include "som/som_runtime.h"
#include "util/bitutils.h"
#include "util/internal_report.h"
#include "util/multibit.h"

/* Callbacks, defined in catchup.c */

int roseNfaAdaptor(u64a offset, ReportID id, void *context);
int roseNfaAdaptorNoInternal(u64a offset, ReportID id, void *context);
int roseNfaSomAdaptor(u64a from_offset, u64a offset, ReportID id, void *context);

/* Callbacks, defined in match.c */

hwlmcb_rv_t roseCallback(size_t start, size_t end, u32 id, void *ctx);
hwlmcb_rv_t roseDelayRebuildCallback(size_t start, size_t end, u32 id,
                                     void *ctx);
int roseAnchoredCallback(u64a end, u32 id, void *ctx);
void roseRunEvent(size_t end, u32 id, struct RoseContext *tctxt);

/* Common code, used all over Rose runtime */

static rose_inline
void resetAnchoredLog(const struct RoseEngine *t, struct hs_scratch *scratch) {
    u8 **anchoredRows = getAnchoredLog(scratch);
    u32 region_width = t->anchoredMatches;
    struct RoseContext *tctxt = &scratch->tctxt;

    tctxt->curr_anchored_loc = bf64_iterate(scratch->am_log_sum, MMB_INVALID);
    if (tctxt->curr_anchored_loc != MMB_INVALID) {
        assert(tctxt->curr_anchored_loc < scratch->anchored_region_len);
        u8 *curr_row = anchoredRows[tctxt->curr_anchored_loc];
        tctxt->curr_row_offset = mmbit_iterate(curr_row, region_width,
                                               MMB_INVALID);
        assert(tctxt->curr_row_offset != MMB_INVALID);
    }
    DEBUG_PRINTF("AL reset --> %u, %u\n", tctxt->curr_anchored_loc,
                 tctxt->curr_row_offset);
}

hwlmcb_rv_t roseHandleChainMatch(const struct RoseEngine *t, ReportID r,
                                 u64a end, struct RoseContext *tctxt,
                                 char in_anchored, char in_catchup);

static really_inline
void initQueue(struct mq *q, u32 qi, const struct RoseEngine *t,
               struct RoseContext *tctxt) {
    struct hs_scratch *scratch = tctxtToScratch(tctxt);
    const struct NfaInfo *info = getNfaInfoByQueue(t, qi);
    assert(scratch->fullState);
    q->nfa = getNfaByInfo(t, info);
    q->end = 0;
    q->cur = 0;
    q->state = scratch->fullState + info->fullStateOffset;
    q->streamState = (char *)tctxt->state + info->stateOffset;
    q->offset = scratch->core_info.buf_offset;
    q->buffer = scratch->core_info.buf;
    q->length = scratch->core_info.len;
    q->history = scratch->core_info.hbuf;
    q->hlength = scratch->core_info.hlen;
    if (info->only_external) {
        q->cb = roseNfaAdaptorNoInternal;
    } else {
        q->cb = roseNfaAdaptor;
    }
    q->som_cb = roseNfaSomAdaptor;
    q->context = tctxt;
    q->report_current = 0;

    DEBUG_PRINTF("qi=%u, offset=%llu, fullState=%u, streamState=%u, "
                 "state=%u\n", qi, q->offset, info->fullStateOffset,
                 info->stateOffset, *(u32 *)q->state);
}

static really_inline
void initRoseQueue(const struct RoseEngine *t, u32 qi,
                   const struct LeftNfaInfo *left,
                   struct RoseContext *tctxt) {
    struct hs_scratch *scratch = tctxtToScratch(tctxt);
    struct mq *q = scratch->queues + qi;
    const struct NfaInfo *info = getNfaInfoByQueue(t, qi);
    q->nfa = getNfaByInfo(t, info);
    q->end = 0;
    q->cur = 0;
    q->state = scratch->fullState + info->fullStateOffset;

    // Transient roses don't have stream state, we use tstate in scratch
    // instead. The only reason we need this at ALL is for LimEx extended
    // regions, which assume that they have access to q->streamState +
    // compressedStateSize.
    if (left->transient) {
        q->streamState = (char *)scratch->tstate + info->stateOffset;
    } else {
        q->streamState = (char *)tctxt->state + info->stateOffset;
    }

    q->offset = scratch->core_info.buf_offset;
    q->buffer = scratch->core_info.buf;
    q->length = scratch->core_info.len;
    q->history = scratch->core_info.hbuf;
    q->hlength = scratch->core_info.hlen;
    q->cb = NULL;
    q->context = NULL;
    q->report_current = 0;

    DEBUG_PRINTF("qi=%u, offset=%llu, fullState=%u, streamState=%u, "
                 "state=%u\n", qi, q->offset, info->fullStateOffset,
                 info->stateOffset, *(u32 *)q->state);
}

/** returns 0 if space for two items (top and end) on the queue */
static really_inline
char isQueueFull(const struct mq *q) {
    return q->end + 2 > MAX_MQE_LEN;
}

static really_inline
void loadStreamState(const struct NFA *nfa, struct mq *q, s64a loc) {
    DEBUG_PRINTF("offset=%llu, length=%zu, hlength=%zu, loc=%lld\n",
                 q->offset, q->length, q->hlength, loc);
    nfaExpandState(nfa, q->state, q->streamState, q->offset + loc,
                   queue_prev_byte(q, loc));
}

static really_inline
void storeRoseDelay(const struct RoseEngine *t, u8 *state,
                    const struct LeftNfaInfo *left, u32 loc) {
    u32 di = left->lagIndex;
    if (di == ROSE_OFFSET_INVALID) {
        return;
    }

    assert(loc < 256); // ONE WHOLE BYTE!
    DEBUG_PRINTF("storing rose delay %u in slot %u\n", loc, di);
    u8 *leftfixDelay = getLeftfixLagTable(t, state);
    assert(loc <= MAX_STORED_LEFTFIX_LAG);
    leftfixDelay[di] = loc;
}

static really_inline
void setAsZombie(const struct RoseEngine *t, u8 *state,
                 const struct LeftNfaInfo *left) {
    u32 di = left->lagIndex;
    assert(di != ROSE_OFFSET_INVALID);
    if (di == ROSE_OFFSET_INVALID) {
        return;
    }

    u8 *leftfixDelay = getLeftfixLagTable(t, state);
    leftfixDelay[di] = OWB_ZOMBIE_ALWAYS_YES;
}

/* loadRoseDelay MUST NOT be called on the first stream write as it is only
 * initialized for running nfas on stream boundaries */
static really_inline
u32 loadRoseDelay(const struct RoseEngine *t, const u8 *state,
                  const struct LeftNfaInfo *left) {
    u32 di = left->lagIndex;
    if (di == ROSE_OFFSET_INVALID) {
        return 0;
    }

    const u8 *leftfixDelay = getLeftfixLagTableConst(t, state);
    u32 loc = leftfixDelay[di];
    DEBUG_PRINTF("read rose delay %u from slot %u\n", loc, di);
    return loc;
}

static really_inline
char isZombie(const struct RoseEngine *t, const u8 *state,
              const struct LeftNfaInfo *left) {
    u32 di = left->lagIndex;
    assert(di != ROSE_OFFSET_INVALID);
    if (di == ROSE_OFFSET_INVALID) {
        return 0;
    }

    const u8 *leftfixDelay = getLeftfixLagTableConst(t, state);
    DEBUG_PRINTF("read owb %hhu from slot %u\n", leftfixDelay[di], di);
    return leftfixDelay[di] == OWB_ZOMBIE_ALWAYS_YES;
}

hwlmcb_rv_t flushQueuedLiterals_i(struct RoseContext *tctxt, u64a end);

static really_inline
hwlmcb_rv_t flushQueuedLiterals(struct RoseContext *tctxt, u64a end) {
    if (tctxt->delayLastEndOffset == end) {
        DEBUG_PRINTF("no progress, no flush\n");
        return HWLM_CONTINUE_MATCHING;
    }

    if (!tctxt->filledDelayedSlots && !tctxtToScratch(tctxt)->al_log_sum) {
        tctxt->delayLastEndOffset = end;
        return HWLM_CONTINUE_MATCHING;
    }

    return flushQueuedLiterals_i(tctxt, end);
}

static really_inline
hwlmcb_rv_t cleanUpDelayed(size_t length, u64a offset, struct RoseContext *tctxt,
                           u8 *status) {
    if (can_stop_matching(tctxtToScratch(tctxt))) {
        return HWLM_TERMINATE_MATCHING;
    }

    if (flushQueuedLiterals(tctxt, length + offset)
        == HWLM_TERMINATE_MATCHING) {
        return HWLM_TERMINATE_MATCHING;
    }

    if (tctxt->filledDelayedSlots) {
        DEBUG_PRINTF("dirty\n");
        *status |= DELAY_FLOAT_DIRTY;
    } else {
        *status &= ~DELAY_FLOAT_DIRTY;
    }

    tctxt->filledDelayedSlots = 0;
    tctxt->delayLastEndOffset = offset;

    return HWLM_CONTINUE_MATCHING;
}

static really_inline
void update_depth(struct RoseContext *tctxt, u8 depth) {
    u8 d = MAX(tctxt->depth, depth + 1);
    assert(d >= tctxt->depth);
    DEBUG_PRINTF("depth now %hhu was %hhu\n", d, tctxt->depth);
    tctxt->depth = d;
}

/* Note: uses the stashed sparse iter state; cannot be called from
 * anybody else who is using it */
static rose_inline
void roseFlushLastByteHistory(const struct RoseEngine *t, u8 *state,
                              u64a currEnd, struct RoseContext *tctxt) {
    if (!t->lastByteHistoryIterOffset) {
        return;
    }

    struct hs_scratch *scratch = tctxtToScratch(tctxt);
    struct core_info *ci = &scratch->core_info;

    /* currEnd is last byte of string + 1 */
    if (tctxt->lastEndOffset == ci->buf_offset + ci->len
        || currEnd != ci->buf_offset + ci->len) {
        /* already flushed or it is not yet time to flush */
        return;
    }

    DEBUG_PRINTF("flushing\n");

    const struct mmbit_sparse_iter *it
        = (const void *)((const char *)t + t->lastByteHistoryIterOffset);
    const u32 numStates = t->rolesWithStateCount;
    void *role_state = getRoleState(state);

    mmbit_sparse_iter_unset(role_state, numStates, it,
                            scratch->sparse_iter_state);
}

hwlmcb_rv_t roseRunRoleProgram(const struct RoseEngine *t, u32 programOffset,
                               u64a end, u64a *som, struct RoseContext *tctxt,
                               int *work_done);

#endif
