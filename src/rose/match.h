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

#ifndef ROSE_MATCH_H
#define ROSE_MATCH_H

#include "catchup.h"
#include "runtime.h"
#include "scratch.h"
#include "report.h"
#include "rose_common.h"
#include "rose_internal.h"
#include "ue2common.h"
#include "hwlm/hwlm.h"
#include "nfa/nfa_api.h"
#include "nfa/nfa_api_queue.h"
#include "nfa/nfa_api_util.h"
#include "som/som_runtime.h"
#include "util/bitutils.h"
#include "util/exhaust.h"
#include "util/fatbit.h"
#include "util/multibit.h"

/* Callbacks, defined in catchup.c */

int roseNfaAdaptor(u64a start, u64a end, ReportID id, void *context);

/* Callbacks, defined in match.c */

hwlmcb_rv_t roseCallback(size_t end, u32 id, struct hs_scratch *scratch);
hwlmcb_rv_t roseFloatingCallback(size_t end, u32 id,
                                 struct hs_scratch *scratch);
hwlmcb_rv_t roseDelayRebuildCallback(size_t end, u32 id,
                                     struct hs_scratch *scratch);
int roseAnchoredCallback(u64a start, u64a end, u32 id, void *ctx);

/* Common code, used all over Rose runtime */

hwlmcb_rv_t roseHandleChainMatch(const struct RoseEngine *t,
                                 struct hs_scratch *scratch, u32 event,
                                 u64a top_squash_distance, u64a end,
                                 char in_catchup);

/** \brief Initialize the queue for a suffix/outfix engine. */
static really_inline
void initQueue(struct mq *q, u32 qi, const struct RoseEngine *t,
               struct hs_scratch *scratch) {
    const struct NfaInfo *info = getNfaInfoByQueue(t, qi);
    assert(scratch->fullState);
    q->nfa = getNfaByInfo(t, info);
    q->end = 0;
    q->cur = 0;
    q->state = scratch->fullState + info->fullStateOffset;
    q->streamState = scratch->core_info.state + info->stateOffset;
    q->offset = scratch->core_info.buf_offset;
    q->buffer = scratch->core_info.buf;
    q->length = scratch->core_info.len;
    q->history = scratch->core_info.hbuf;
    q->hlength = scratch->core_info.hlen;
    q->cb = roseNfaAdaptor;
    q->context = scratch;
    q->report_current = 0;

    DEBUG_PRINTF("qi=%u, offset=%llu, fullState=%u, streamState=%u, "
                 "state=%u\n", qi, q->offset, info->fullStateOffset,
                 info->stateOffset, *(u32 *)q->state);
}

/** \brief Initialize the queue for a leftfix (prefix/infix) engine. */
static really_inline
void initRoseQueue(const struct RoseEngine *t, u32 qi,
                   const struct LeftNfaInfo *left,
                   struct hs_scratch *scratch) {
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
        q->streamState = scratch->core_info.state + info->stateOffset;
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
void storeRoseDelay(const struct RoseEngine *t, char *state,
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
void setAsZombie(const struct RoseEngine *t, char *state,
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
u32 loadRoseDelay(const struct RoseEngine *t, const char *state,
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
char isZombie(const struct RoseEngine *t, const char *state,
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

hwlmcb_rv_t flushQueuedLiterals_i(const struct RoseEngine *t,
                                  struct hs_scratch *scratch, u64a end);

static really_inline
hwlmcb_rv_t flushQueuedLiterals(const struct RoseEngine *t,
                                struct hs_scratch *scratch, u64a end) {
    struct RoseContext *tctxt = &scratch->tctxt;

    if (tctxt->delayLastEndOffset == end) {
        DEBUG_PRINTF("no progress, no flush\n");
        return HWLM_CONTINUE_MATCHING;
    }

    if (!tctxt->filledDelayedSlots && !scratch->al_log_sum) {
        tctxt->delayLastEndOffset = end;
        return HWLM_CONTINUE_MATCHING;
    }

    return flushQueuedLiterals_i(t, scratch, end);
}

static really_inline
hwlmcb_rv_t cleanUpDelayed(const struct RoseEngine *t,
                           struct hs_scratch *scratch, size_t length,
                           u64a offset) {
    if (can_stop_matching(scratch)) {
        return HWLM_TERMINATE_MATCHING;
    }

    if (flushQueuedLiterals(t, scratch, length + offset)
        == HWLM_TERMINATE_MATCHING) {
        return HWLM_TERMINATE_MATCHING;
    }

    struct RoseContext *tctxt = &scratch->tctxt;
    if (tctxt->filledDelayedSlots) {
        DEBUG_PRINTF("dirty\n");
        scratch->core_info.status |= STATUS_DELAY_DIRTY;
    } else {
        scratch->core_info.status &= ~STATUS_DELAY_DIRTY;
    }

    tctxt->filledDelayedSlots = 0;
    tctxt->delayLastEndOffset = offset;

    return HWLM_CONTINUE_MATCHING;
}

static rose_inline
void roseFlushLastByteHistory(const struct RoseEngine *t,
                              struct hs_scratch *scratch, u64a currEnd) {
    if (!t->lastByteHistoryIterOffset) {
        return;
    }

    struct RoseContext *tctxt = &scratch->tctxt;
    struct core_info *ci = &scratch->core_info;

    /* currEnd is last byte of string + 1 */
    if (tctxt->lastEndOffset == ci->buf_offset + ci->len
        || currEnd != ci->buf_offset + ci->len) {
        /* already flushed or it is not yet time to flush */
        return;
    }

    DEBUG_PRINTF("flushing\n");

    const struct mmbit_sparse_iter *it =
        getByOffset(t, t->lastByteHistoryIterOffset);
    assert(ISALIGNED(it));

    const u32 numStates = t->rolesWithStateCount;
    void *role_state = getRoleState(scratch->core_info.state);

    struct mmbit_sparse_state si_state[MAX_SPARSE_ITER_STATES];

    mmbit_sparse_iter_unset(role_state, numStates, it, si_state);
}

static rose_inline
int roseHasInFlightMatches(const struct RoseEngine *t, char *state,
                           const struct hs_scratch *scratch) {
    if (scratch->al_log_sum) {
        DEBUG_PRINTF("anchored literals in log\n");
        return 1;
    }

    if (scratch->tctxt.filledDelayedSlots) {
        DEBUG_PRINTF("delayed literal\n");
        return 1;
    }

    if (mmbit_any(getRoleState(state), t->rolesWithStateCount)) {
        DEBUG_PRINTF("role state is set\n");
        return 1;
    }

    return 0;
}

static rose_inline
hwlmcb_rv_t roseHaltIfExhausted(const struct RoseEngine *t,
                                struct hs_scratch *scratch) {
    struct core_info *ci = &scratch->core_info;
    if (isAllExhausted(t, ci->exhaustionVector)) {
        ci->status |= STATUS_EXHAUSTED;
        scratch->tctxt.groups = 0;
        DEBUG_PRINTF("all exhausted, termination requested\n");
        return HWLM_TERMINATE_MATCHING;
    }

    return HWLM_CONTINUE_MATCHING;
}

static really_inline
hwlmcb_rv_t ensureQueueFlushed_i(const struct RoseEngine *t,
                                 struct hs_scratch *scratch, u32 qi, s64a loc,
                                 char is_mpv, char in_catchup) {
    struct RoseContext *tctxt = &scratch->tctxt;
    u8 *aa = getActiveLeafArray(t, scratch->core_info.state);
    struct fatbit *activeQueues = scratch->aqa;
    u32 aaCount = t->activeArrayCount;
    u32 qCount = t->queueCount;

    struct mq *q = &scratch->queues[qi];
    DEBUG_PRINTF("qcl %lld, loc: %lld, min (non mpv) match offset: %llu\n",
                 q_cur_loc(q), loc, tctxt->minNonMpvMatchOffset);
    if (q_cur_loc(q) == loc) {
        /* too many tops enqueued at the one spot; need to flatten this queue.
         * We can use the full catchups as it will short circuit as we are
         * already at this location. It also saves waking everybody up */
        pushQueueNoMerge(q, MQE_END, loc);
        nfaQueueExec(q->nfa, q, loc);
        q->cur = q->end = 0;
        pushQueueAt(q, 0, MQE_START, loc);
    } else if (!in_catchup) {
        if (is_mpv) {
            tctxt->next_mpv_offset = 0; /* force us to catch the mpv */
            if (loc + scratch->core_info.buf_offset
                <= tctxt->minNonMpvMatchOffset) {
                DEBUG_PRINTF("flushing chained\n");
                if (roseCatchUpMPV(t, loc, scratch) ==
                    HWLM_TERMINATE_MATCHING) {
                    return HWLM_TERMINATE_MATCHING;
                }
                goto done_queue_empty;
            }
        }

        if (roseCatchUpTo(t, scratch, loc + scratch->core_info.buf_offset) ==
            HWLM_TERMINATE_MATCHING) {
            return HWLM_TERMINATE_MATCHING;
        }
    } else {
        /* we must be a chained nfa */
        assert(is_mpv);
        DEBUG_PRINTF("flushing chained\n");
        tctxt->next_mpv_offset = 0; /* force us to catch the mpv */
        if (roseCatchUpMPV(t, loc, scratch) == HWLM_TERMINATE_MATCHING) {
            return HWLM_TERMINATE_MATCHING;
        }
    }
done_queue_empty:
    if (!mmbit_set(aa, aaCount, qi)) {
        initQueue(q, qi, t, scratch);
        nfaQueueInitState(q->nfa, q);
        pushQueueAt(q, 0, MQE_START, loc);
        fatbit_set(activeQueues, qCount, qi);
    }

    assert(!isQueueFull(q));

    return roseHaltIfExhausted(t, scratch);
}

static rose_inline
hwlmcb_rv_t ensureQueueFlushed(const struct RoseEngine *t,
                               struct hs_scratch *scratch, u32 qi, s64a loc) {
    return ensureQueueFlushed_i(t, scratch, qi, loc, 0, 0);
}

#endif
