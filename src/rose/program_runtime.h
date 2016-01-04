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

#ifndef PROGRAM_RUNTIME_H
#define PROGRAM_RUNTIME_H

#include "catchup.h"
#include "counting_miracle.h"
#include "infix.h"
#include "match.h"
#include "miracle.h"
#include "rose.h"
#include "rose_internal.h"
#include "rose_program.h"
#include "rose_types.h"
#include "runtime.h"
#include "scratch.h"
#include "ue2common.h"
#include "util/fatbit.h"
#include "util/multibit.h"

static rose_inline
char rosePrefixCheckMiracles(const struct RoseEngine *t,
                             const struct LeftNfaInfo *left,
                             struct core_info *ci, struct mq *q, u64a end) {
    if (left->transient) {
        // Miracles won't help us with transient leftfix engines; they only
        // scan for a limited time anyway.
        return 1;
    }

    if (!left->stopTable) {
        return 1;
    }

    DEBUG_PRINTF("looking for miracle on queue %u\n", q->nfa->queueIndex);

    const s64a begin_loc = q_cur_loc(q);
    const s64a end_loc = end - ci->buf_offset;

    s64a miracle_loc;
    if (roseMiracleOccurs(t, left, ci, begin_loc, end_loc, &miracle_loc)) {
        goto found_miracle;
    }

    if (roseCountingMiracleOccurs(t, left, ci, begin_loc, end_loc,
                                  &miracle_loc)) {
        goto found_miracle;
    }

    return 1;

found_miracle:
    DEBUG_PRINTF("miracle at %lld\n", miracle_loc);
    assert(miracle_loc >= begin_loc);

    // If we're a prefix, then a miracle effectively results in us needing to
    // re-init our state and start fresh.
    if (!left->infix) {
        if (miracle_loc != begin_loc) {
            DEBUG_PRINTF("re-init prefix state\n");
            q->cur = q->end = 0;
            pushQueueAt(q, 0, MQE_START, miracle_loc);
            pushQueueAt(q, 1, MQE_TOP, miracle_loc);
            nfaQueueInitState(q->nfa, q);
        }
        return 1;
    }

    // Otherwise, we're an infix. Remove tops before the miracle from the queue
    // and re-init at that location.

    q_skip_forward_to(q, miracle_loc);

    if (q_last_type(q) == MQE_START) {
        DEBUG_PRINTF("miracle caused infix to die\n");
        return 0;
    }

    DEBUG_PRINTF("re-init infix state\n");
    assert(q->items[q->cur].type == MQE_START);
    q->items[q->cur].location = miracle_loc;
    nfaQueueInitState(q->nfa, q);

    return 1;
}

static really_inline
hwlmcb_rv_t ensureQueueFlushed_i(const struct RoseEngine *t,
                                 struct hs_scratch *scratch, u32 qi, s64a loc,
                                 char is_mpv, char in_anchored,
                                 char in_catchup) {
    struct RoseContext *tctxt = &scratch->tctxt;
    u8 *aa = getActiveLeafArray(t, tctxt->state);
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
                if (roseCatchUpMPV(t, tctxt->state, loc, scratch)
                    == HWLM_TERMINATE_MATCHING) {
                    return HWLM_TERMINATE_MATCHING;
                }
                goto done_queue_empty;
            }
        }

        if (roseCatchUpTo(t, tctxt->state, loc + scratch->core_info.buf_offset,
                          scratch, in_anchored)
            == HWLM_TERMINATE_MATCHING) {
            return HWLM_TERMINATE_MATCHING;
        }
    } else {
        /* we must be a chained nfa */
        assert(is_mpv);
        DEBUG_PRINTF("flushing chained\n");
        tctxt->next_mpv_offset = 0; /* force us to catch the mpv */
        if (roseCatchUpMPV(t, tctxt->state, loc, scratch)
            == HWLM_TERMINATE_MATCHING) {
            return HWLM_TERMINATE_MATCHING;
        }
    }
done_queue_empty:
    if (!mmbit_set(aa, aaCount, qi)) {
        initQueue(q, qi, t, tctxt);
        nfaQueueInitState(q->nfa, q);
        pushQueueAt(q, 0, MQE_START, loc);
        fatbit_set(activeQueues, qCount, qi);
    }

    assert(!isQueueFull(q));

    if (isAllExhausted(t, scratch->core_info.exhaustionVector)) {
        if (!scratch->core_info.broken) {
            scratch->core_info.broken = BROKEN_EXHAUSTED;
        }
        tctxt->groups = 0;
        DEBUG_PRINTF("termination requested\n");
        return HWLM_TERMINATE_MATCHING;
    }

    return HWLM_CONTINUE_MATCHING;
}

static really_inline
hwlmcb_rv_t ensureQueueFlushed(const struct RoseEngine *t,
                               struct hs_scratch *scratch, u32 qi, s64a loc,
                               char in_anchored) {
    return ensureQueueFlushed_i(t, scratch, qi, loc, 0, in_anchored, 0);
}

static rose_inline
hwlmcb_rv_t roseHandleSuffixTrigger(const struct RoseEngine *t,
                                    u32 qi, u32 top, u64a som,
                                    u64a end, struct RoseContext *tctxt,
                                    char in_anchored) {
    DEBUG_PRINTF("suffix qi=%u, top event=%u\n", qi, top);

    u8 *aa = getActiveLeafArray(t, tctxt->state);
    struct hs_scratch *scratch = tctxtToScratch(tctxt);
    const u32 aaCount = t->activeArrayCount;
    const u32 qCount = t->queueCount;
    struct mq *q = &scratch->queues[qi];
    const struct NfaInfo *info = getNfaInfoByQueue(t, qi);
    const struct NFA *nfa = getNfaByInfo(t, info);

    struct core_info *ci = &scratch->core_info;
    s64a loc = (s64a)end - ci->buf_offset;
    assert(loc <= (s64a)ci->len && loc >= -(s64a)ci->hlen);

    if (!mmbit_set(aa, aaCount, qi)) {
        initQueue(q, qi, t, tctxt);
        nfaQueueInitState(nfa, q);
        pushQueueAt(q, 0, MQE_START, loc);
        fatbit_set(scratch->aqa, qCount, qi);
    } else if (info->no_retrigger) {
        DEBUG_PRINTF("yawn\n");
        /* nfa only needs one top; we can go home now */
        return HWLM_CONTINUE_MATCHING;
    } else if (!fatbit_set(scratch->aqa, qCount, qi)) {
        initQueue(q, qi, t, tctxt);
        loadStreamState(nfa, q, 0);
        pushQueueAt(q, 0, MQE_START, 0);
    } else if (isQueueFull(q)) {
        DEBUG_PRINTF("queue %u full -> catching up nfas\n", qi);
        if (info->eod) {
            /* can catch up suffix independently no pq */
            q->context = NULL;
            pushQueueNoMerge(q, MQE_END, loc);
            nfaQueueExecRose(q->nfa, q, MO_INVALID_IDX);
            q->cur = q->end = 0;
            pushQueueAt(q, 0, MQE_START, loc);
        } else if (ensureQueueFlushed(t, scratch, qi, loc, in_anchored)
            == HWLM_TERMINATE_MATCHING) {
            return HWLM_TERMINATE_MATCHING;
        }
    }

    assert(top == MQE_TOP || (top >= MQE_TOP_FIRST && top < MQE_INVALID));
    pushQueueSom(q, top, loc, som);

    if (q_cur_loc(q) == (s64a)ci->len && !info->eod) {
        /* we may not run the nfa; need to ensure state is fine  */
        DEBUG_PRINTF("empty run\n");
        pushQueueNoMerge(q, MQE_END, loc);
        char alive = nfaQueueExec(nfa, q, loc);
        if (alive) {
            q->cur = q->end = 0;
            pushQueueAt(q, 0, MQE_START, loc);
        } else {
            mmbit_unset(aa, aaCount, qi);
            fatbit_unset(scratch->aqa, qCount, qi);
        }
    }

    return HWLM_CONTINUE_MATCHING;
}

static really_inline
char roseTestLeftfix(const struct RoseEngine *t, u32 qi, u32 leftfixLag,
                     ReportID leftfixReport, u64a end,
                     struct RoseContext *tctxt) {
    struct hs_scratch *scratch = tctxtToScratch(tctxt);
    struct core_info *ci = &scratch->core_info;

    u32 ri = queueToLeftIndex(t, qi);
    const struct LeftNfaInfo *left = getLeftTable(t) + ri;

    DEBUG_PRINTF("testing %s %s %u/%u with lag %u (maxLag=%u)\n",
                 (left->transient ? "transient" : "active"),
                 (left->infix ? "infix" : "prefix"),
                 ri, qi, leftfixLag, left->maxLag);

    assert(leftfixLag <= left->maxLag);

    struct mq *q = scratch->queues + qi;
    u32 qCount = t->queueCount;
    u32 arCount = t->activeLeftCount;

    if (!mmbit_isset(getActiveLeftArray(t, tctxt->state), arCount, ri)) {
        DEBUG_PRINTF("engine is dead nothing to see here\n");
        return 0;
    }

    if (unlikely(end < leftfixLag)) {
        assert(0); /* lag is the literal length */
        return 0;
    }

    if (nfaSupportsZombie(getNfaByQueue(t, qi)) && ci->buf_offset
        && !fatbit_isset(scratch->aqa, qCount, qi)
        && isZombie(t, tctxt->state, left)) {
        DEBUG_PRINTF("zombie\n");
        return 1;
    }

    if (!fatbit_set(scratch->aqa, qCount, qi)) {
        DEBUG_PRINTF("initing q %u\n", qi);
        initRoseQueue(t, qi, left, tctxt);
        if (ci->buf_offset) { // there have been writes before us!
            s32 sp;
            if (left->transient) {
                sp = -(s32)ci->hlen;
            } else {
                sp = -(s32)loadRoseDelay(t, tctxt->state, left);
            }

            /* transient nfas are always started fresh -> state not maintained
             * at stream boundary */

            pushQueueAt(q, 0, MQE_START, sp);
            if (left->infix || (ci->buf_offset + sp > 0 && !left->transient)) {
                loadStreamState(q->nfa, q, sp);
            } else {
                pushQueueAt(q, 1, MQE_TOP, sp);
                nfaQueueInitState(q->nfa, q);
            }
        } else { // first write ever
            pushQueueAt(q, 0, MQE_START, 0);
            pushQueueAt(q, 1, MQE_TOP, 0);
            nfaQueueInitState(q->nfa, q);
        }
    }

    s64a loc = (s64a)end - ci->buf_offset - leftfixLag;
    assert(loc >= q_cur_loc(q));
    assert(leftfixReport != MO_INVALID_IDX);

    if (left->transient) {
        s64a start_loc = loc - left->transient;
        if (q_cur_loc(q) < start_loc) {
            q->cur = q->end = 0;
            pushQueueAt(q, 0, MQE_START, start_loc);
            pushQueueAt(q, 1, MQE_TOP, start_loc);
            nfaQueueInitState(q->nfa, q);
        }
    }

    if (q_cur_loc(q) < loc || q_last_type(q) != MQE_START) {
        if (left->infix) {
            if (infixTooOld(q, loc)) {
                DEBUG_PRINTF("infix %u died of old age\n", ri);
                scratch->tctxt.groups &= left->squash_mask;
                mmbit_unset(getActiveLeftArray(t, tctxt->state), arCount, ri);
                return 0;
            }

            reduceQueue(q, loc, left->maxQueueLen, q->nfa->maxWidth);
        }

        if (!rosePrefixCheckMiracles(t, left, ci, q, end)) {
            DEBUG_PRINTF("leftfix %u died due to miracle\n", ri);
            scratch->tctxt.groups &= left->squash_mask;
            mmbit_unset(getActiveLeftArray(t, tctxt->state), arCount, ri);
            return 0;
        }

#ifdef DEBUG
        debugQueue(q);
#endif

        pushQueueNoMerge(q, MQE_END, loc);

        char rv = nfaQueueExecRose(q->nfa, q, leftfixReport);
        if (!rv) { /* nfa is dead */
            DEBUG_PRINTF("leftfix %u died while trying to catch up\n", ri);
            mmbit_unset(getActiveLeftArray(t, tctxt->state), arCount, ri);
            assert(!mmbit_isset(getActiveLeftArray(t, tctxt->state), arCount,
                                ri));
            tctxt->groups &= left->squash_mask;
            return 0;
        }

        // Queue must have next start loc before we call nfaInAcceptState.
        q->cur = q->end = 0;
        pushQueueAt(q, 0, MQE_START, loc);

        DEBUG_PRINTF("checking for report %u\n", leftfixReport);
        DEBUG_PRINTF("leftfix done %hhd\n", (signed char)rv);
        return rv == MO_MATCHES_PENDING;
    } else {
        DEBUG_PRINTF("checking for report %u\n", leftfixReport);
        char rv = nfaInAcceptState(q->nfa, leftfixReport, q);
        DEBUG_PRINTF("leftfix done %hhd\n", (signed char)rv);
        return rv;
    }
}

static rose_inline
void roseSetRole(const struct RoseEngine *t, u8 *state,
                 struct RoseContext *tctxt, u32 stateIndex, u8 depth) {
    DEBUG_PRINTF("state idx=%u, depth=%u\n", stateIndex, depth);
    mmbit_set(getRoleState(state), t->rolesWithStateCount, stateIndex);
    update_depth(tctxt, depth);
}

static rose_inline
void roseTriggerInfix(const struct RoseEngine *t, u64a start, u64a end, u32 qi,
                      u32 topEvent, u8 cancel, struct RoseContext *tctxt) {
    struct core_info *ci = &tctxtToScratch(tctxt)->core_info;
    s64a loc = (s64a)end - ci->buf_offset;

    u32 ri = queueToLeftIndex(t, qi);
    assert(topEvent < MQE_INVALID);

    const struct LeftNfaInfo *left = getLeftInfoByQueue(t, qi);
    assert(!left->transient);

    DEBUG_PRINTF("rose %u (qi=%u) event %u\n", ri, qi, topEvent);

    struct mq *q = tctxtToScratch(tctxt)->queues + qi;
    const struct NfaInfo *info = getNfaInfoByQueue(t, qi);

    u8 *activeLeftArray = getActiveLeftArray(t, tctxt->state);
    const u32 arCount = t->activeLeftCount;
    char alive = mmbit_set(activeLeftArray, arCount, ri);

    if (alive && info->no_retrigger) {
        DEBUG_PRINTF("yawn\n");
        return;
    }

    struct fatbit *aqa = tctxtToScratch(tctxt)->aqa;
    const u32 qCount = t->queueCount;

    if (alive && nfaSupportsZombie(getNfaByInfo(t, info)) && ci->buf_offset &&
        !fatbit_isset(aqa, qCount, qi) && isZombie(t, tctxt->state, left)) {
        DEBUG_PRINTF("yawn - zombie\n");
        return;
    }

    if (cancel) {
        DEBUG_PRINTF("dominating top: (re)init\n");
        fatbit_set(aqa, qCount, qi);
        initRoseQueue(t, qi, left, tctxt);
        pushQueueAt(q, 0, MQE_START, loc);
        nfaQueueInitState(q->nfa, q);
    } else if (!fatbit_set(aqa, qCount, qi)) {
        DEBUG_PRINTF("initing %u\n", qi);
        initRoseQueue(t, qi, left, tctxt);
        if (alive) {
            s32 sp = -(s32)loadRoseDelay(t, tctxt->state, left);
            pushQueueAt(q, 0, MQE_START, sp);
            loadStreamState(q->nfa, q, sp);
        } else {
            pushQueueAt(q, 0, MQE_START, loc);
            nfaQueueInitState(q->nfa, q);
        }
    } else if (!alive) {
        q->cur = q->end = 0;
        pushQueueAt(q, 0, MQE_START, loc);
        nfaQueueInitState(q->nfa, q);
    } else if (isQueueFull(q)) {
        reduceQueue(q, loc, left->maxQueueLen, q->nfa->maxWidth);

        if (isQueueFull(q)) {
            /* still full - reduceQueue did nothing */
            DEBUG_PRINTF("queue %u full (%u items) -> catching up nfa\n", qi,
                         q->end - q->cur);
            pushQueueNoMerge(q, MQE_END, loc);
            nfaQueueExecRose(q->nfa, q, MO_INVALID_IDX);

            q->cur = q->end = 0;
            pushQueueAt(q, 0, MQE_START, loc);
        }
    }

    pushQueueSom(q, topEvent, loc, start);
}

/* handles the firing of external matches */
static rose_inline
hwlmcb_rv_t roseHandleMatch(const struct RoseEngine *t, u8 *state, ReportID id,
                            u64a end, struct RoseContext *tctxt,
                            char in_anchored) {
    struct hs_scratch *scratch = tctxtToScratch(tctxt);

    if (roseCatchUpTo(t, state, end, scratch, in_anchored)
        == HWLM_TERMINATE_MATCHING) {
        return HWLM_TERMINATE_MATCHING;
    }

    assert(end == tctxt->minMatchOffset);
    DEBUG_PRINTF("firing callback reportId=%u, end=%llu\n", id, end);
    updateLastMatchOffset(tctxt, end);

    int cb_rv = tctxt->cb(end, id, tctxt->userCtx);
    if (cb_rv == MO_HALT_MATCHING) {
        DEBUG_PRINTF("termination requested\n");
        return HWLM_TERMINATE_MATCHING;
    }

    if (cb_rv == ROSE_CONTINUE_MATCHING_NO_EXHAUST) {
        return HWLM_CONTINUE_MATCHING;
    }

    if (isAllExhausted(t, scratch->core_info.exhaustionVector)) {
        if (!scratch->core_info.broken) {
            scratch->core_info.broken = BROKEN_EXHAUSTED;
        }
        tctxt->groups = 0;
        DEBUG_PRINTF("termination requested\n");
        return HWLM_TERMINATE_MATCHING;
    }

    return HWLM_CONTINUE_MATCHING;
}

/* catches up engines enough to ensure any earlier mpv triggers are enqueued
 * and then adds the trigger to the mpv queue. Must not be called during catch
 * up */
static rose_inline
hwlmcb_rv_t roseCatchUpAndHandleChainMatch(const struct RoseEngine *t,
                                           u8 *state, ReportID r, u64a end,
                                           struct RoseContext *tctxt,
                                           char in_anchored) {
    struct hs_scratch *scratch = tctxtToScratch(tctxt);

    if (roseCatchUpMpvFeeders(t, state, end, scratch, in_anchored)
        == HWLM_TERMINATE_MATCHING) {
        return HWLM_TERMINATE_MATCHING;
    }

    return roseHandleChainMatch(t, r, end, tctxt, in_anchored, 0);
}

static rose_inline
hwlmcb_rv_t roseSomCatchup(const struct RoseEngine *t, u8 *state, u64a end,
                           struct RoseContext *tctxt, char in_anchored) {
    struct hs_scratch *scratch = tctxtToScratch(tctxt);

    // In SOM processing, we may be able to limit or entirely avoid catchup.

    DEBUG_PRINTF("entry\n");

    if (end == tctxt->minMatchOffset) {
        DEBUG_PRINTF("already caught up\n");
        return HWLM_CONTINUE_MATCHING;
    }

    DEBUG_PRINTF("catching up all NFAs\n");
    if (roseCatchUpTo(t, state, end, scratch, in_anchored)
        == HWLM_TERMINATE_MATCHING) {
        return HWLM_TERMINATE_MATCHING;
    }
    updateMinMatchOffset(tctxt, end);
    return HWLM_CONTINUE_MATCHING;
}

static really_inline
hwlmcb_rv_t roseHandleSom(const struct RoseEngine *t, u8 *state, ReportID id,
                          u64a end, struct RoseContext *tctxt,
                          char in_anchored) {
    struct hs_scratch *scratch = tctxtToScratch(tctxt);

    DEBUG_PRINTF("id=%u, end=%llu, minMatchOffset=%llu\n", id, end,
                  tctxt->minMatchOffset);

    // Reach into reports and handle internal reports that just manipulate SOM
    // slots ourselves, rather than going through the callback.

    if (roseSomCatchup(t, state, end, tctxt, in_anchored)
        == HWLM_TERMINATE_MATCHING) {
        return HWLM_TERMINATE_MATCHING;
    }

    const struct internal_report *ri = getInternalReport(t, id);
    handleSomInternal(scratch, ri, end);

    return HWLM_CONTINUE_MATCHING;
}

static rose_inline
hwlmcb_rv_t roseHandleSomMatch(const struct RoseEngine *t, u8 *state,
                               ReportID id, u64a start, u64a end,
                               struct RoseContext *tctxt, char in_anchored) {
    if (roseCatchUpTo(t, state, end, tctxtToScratch(tctxt), in_anchored)
        == HWLM_TERMINATE_MATCHING) {
        return HWLM_TERMINATE_MATCHING;
    }

    DEBUG_PRINTF("firing som callback reportId=%u, start=%llu end=%llu\n", id,
                 start, end);
    DEBUG_PRINTF("    last match %llu\n", tctxt->lastMatchOffset);
    assert(end == tctxt->minMatchOffset);

    updateLastMatchOffset(tctxt, end);
    int cb_rv = tctxt->cb_som(start, end, id, tctxt->userCtx);
    if (cb_rv == MO_HALT_MATCHING) {
        DEBUG_PRINTF("termination requested\n");
        return HWLM_TERMINATE_MATCHING;
    }

    if (cb_rv == ROSE_CONTINUE_MATCHING_NO_EXHAUST) {
        return HWLM_CONTINUE_MATCHING;
    }

    struct core_info *ci = &tctxtToScratch(tctxt)->core_info;
    if (isAllExhausted(t, ci->exhaustionVector)) {
        if (!ci->broken) {
            ci->broken = BROKEN_EXHAUSTED;
        }
        tctxt->groups = 0;
        DEBUG_PRINTF("termination requested\n");
        return HWLM_TERMINATE_MATCHING;
    }

    return HWLM_CONTINUE_MATCHING;
}

static rose_inline
hwlmcb_rv_t roseHandleSomSom(const struct RoseEngine *t, u8 *state, ReportID id,
                             u64a start, u64a end, struct RoseContext *tctxt,
                             char in_anchored) {
    DEBUG_PRINTF("id=%u, start=%llu, end=%llu, minMatchOffset=%llu\n",
                  id, start, end, tctxt->minMatchOffset);

    // Reach into reports and handle internal reports that just manipulate SOM
    // slots ourselves, rather than going through the callback.

    if (roseSomCatchup(t, state, end, tctxt, in_anchored)
        == HWLM_TERMINATE_MATCHING) {
        return HWLM_TERMINATE_MATCHING;
    }

    const struct internal_report *ri = getInternalReport(t, id);
    setSomFromSomAware(tctxtToScratch(tctxt), ri, start, end);
    return HWLM_CONTINUE_MATCHING;
}

static really_inline
int reachHasBit(const u8 *reach, u8 c) {
    return !!(reach[c / 8U] & (u8)1U << (c % 8U));
}

/**
 * \brief Scan around a literal, checking that that "lookaround" reach masks
 * are satisfied.
 */
static rose_inline
int roseCheckLookaround(const struct RoseEngine *t, u32 lookaroundIndex,
                        u32 lookaroundCount, u64a end,
                        struct RoseContext *tctxt) {
    assert(lookaroundIndex != MO_INVALID_IDX);
    assert(lookaroundCount > 0);

    const struct core_info *ci = &tctxtToScratch(tctxt)->core_info;
    DEBUG_PRINTF("end=%llu, buf_offset=%llu, buf_end=%llu\n", end,
                 ci->buf_offset, ci->buf_offset + ci->len);

    const u8 *base = (const u8 *)t;
    const s8 *look_base = (const s8 *)(base + t->lookaroundTableOffset);
    const s8 *look = look_base + lookaroundIndex;
    const s8 *look_end = look + lookaroundCount;
    assert(look < look_end);

    const u8 *reach_base = base + t->lookaroundReachOffset;
    const u8 *reach = reach_base + lookaroundIndex * REACH_BITVECTOR_LEN;

    // The following code assumes that the lookaround structures are ordered by
    // increasing offset.

    const s64a base_offset = end - ci->buf_offset;
    DEBUG_PRINTF("base_offset=%lld\n", base_offset);
    DEBUG_PRINTF("first look has offset %d\n", *look);

    // If our first check tells us we need to look at an offset before the
    // start of the stream, this role cannot match.
    if (unlikely(*look < 0 && (u64a)(0 - *look) > end)) {
        DEBUG_PRINTF("too early, fail\n");
        return 0;
    }

    // Skip over offsets that are before the history buffer.
    do {
        s64a offset = base_offset + *look;
        if (offset >= -(s64a)ci->hlen) {
            goto in_history;
        }
        DEBUG_PRINTF("look=%d before history\n", *look);
        look++;
        reach += REACH_BITVECTOR_LEN;
    } while (look < look_end);

    // History buffer.
    DEBUG_PRINTF("scan history (%zu looks left)\n", look_end - look);
    for (; look < look_end; ++look, reach += REACH_BITVECTOR_LEN) {
    in_history:
        ;
        s64a offset = base_offset + *look;
        DEBUG_PRINTF("reach=%p, rel offset=%lld\n", reach, offset);

        if (offset >= 0) {
            DEBUG_PRINTF("in buffer\n");
            goto in_buffer;
        }

        assert(offset >= -(s64a)ci->hlen && offset < 0);
        u8 c = ci->hbuf[ci->hlen + offset];
        if (!reachHasBit(reach, c)) {
            DEBUG_PRINTF("char 0x%02x failed reach check\n", c);
            return 0;
        }
    }
    // Current buffer.
    DEBUG_PRINTF("scan buffer (%zu looks left)\n", look_end - look);
    for (; look < look_end; ++look, reach += REACH_BITVECTOR_LEN) {
    in_buffer:
        ;
        s64a offset = base_offset + *look;
        DEBUG_PRINTF("reach=%p, rel offset=%lld\n", reach, offset);

        if (offset >= (s64a)ci->len) {
            DEBUG_PRINTF("in the future\n");
            break;
        }

        assert(offset >= 0 && offset < (s64a)ci->len);
        u8 c = ci->buf[offset];
        if (!reachHasBit(reach, c)) {
            DEBUG_PRINTF("char 0x%02x failed reach check\n", c);
            return 0;
        }
    }

    DEBUG_PRINTF("OK :)\n");
    return 1;
}

static
int roseNfaEarliestSom(u64a from_offset, UNUSED u64a offset, UNUSED ReportID id,
                       void *context) {
    u64a *som = context;
    *som = MIN(*som, from_offset);
    return MO_CONTINUE_MATCHING;
}

static rose_inline
u64a roseGetHaigSom(const struct RoseEngine *t, const u32 qi,
                    UNUSED const u32 leftfixLag,
                    struct RoseContext *tctxt) {
    u32 ri = queueToLeftIndex(t, qi);

    UNUSED const struct LeftNfaInfo *left = getLeftTable(t) + ri;

    DEBUG_PRINTF("testing %s prefix %u/%u with lag %u (maxLag=%u)\n",
                 left->transient ? "transient" : "active", ri, qi,
                 leftfixLag, left->maxLag);

    assert(leftfixLag <= left->maxLag);

    struct mq *q = tctxtToScratch(tctxt)->queues + qi;

    u64a start = ~0ULL;

    /* switch the callback + context for a fun one */
    q->som_cb = roseNfaEarliestSom;
    q->context = &start;

    nfaReportCurrentMatches(q->nfa, q);

    /* restore the old callback + context */
    q->som_cb = roseNfaSomAdaptor;
    q->context = NULL;
    DEBUG_PRINTF("earliest som is %llu\n", start);
    return start;
}

static rose_inline
char roseCheckRootBounds(u64a end, u32 min_bound, u32 max_bound) {
    assert(max_bound <= ROSE_BOUND_INF);
    assert(min_bound <= max_bound);

    if (end < min_bound) {
        return 0;
    }
    return max_bound == ROSE_BOUND_INF || end <= max_bound;
}


#define PROGRAM_CASE(name)                                                     \
    case ROSE_INSTR_##name: {                                                  \
        DEBUG_PRINTF("instruction: " #name " (%u)\n", ROSE_INSTR_##name);      \
        const struct ROSE_STRUCT_##name *ri =                                  \
            (const struct ROSE_STRUCT_##name *)pc;

#define PROGRAM_NEXT_INSTRUCTION                                               \
    pc += ROUNDUP_N(sizeof(*ri), ROSE_INSTR_MIN_ALIGN);                        \
    break;                                                                     \
    }

static really_inline
hwlmcb_rv_t roseRunProgram(const struct RoseEngine *t, u32 programOffset,
                           u64a end, struct RoseContext *tctxt,
                           char in_anchored, int *work_done) {
    DEBUG_PRINTF("program begins at offset %u\n", programOffset);

    assert(programOffset);
    assert(programOffset < t->size);

    const char *pc_base = getByOffset(t, programOffset);
    const char *pc = pc_base;

    u64a som = 0;

    // Local sparse iterator state for programs that use the SPARSE_ITER_BEGIN
    // and SPARSE_ITER_NEXT instructions.
    struct mmbit_sparse_state si_state[MAX_SPARSE_ITER_STATES];

    assert(*(const u8 *)pc != ROSE_INSTR_END);

    for (;;) {
        assert(ISALIGNED_N(pc, ROSE_INSTR_MIN_ALIGN));
        u8 code = *(const u8 *)pc;
        assert(code <= ROSE_INSTR_END);

        switch ((enum RoseInstructionCode)code) {
            PROGRAM_CASE(ANCHORED_DELAY) {
                if (in_anchored && end > t->floatingMinLiteralMatchOffset) {
                    DEBUG_PRINTF("delay until playback\n");
                    update_depth(tctxt, ri->depth);
                    tctxt->groups |= ri->groups;
                    *work_done = 1;
                    assert(ri->done_jump); // must progress
                    pc += ri->done_jump;
                    continue;
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_DEPTH) {
                DEBUG_PRINTF("current depth %u, check min depth %u\n",
                             tctxt->depth, ri->min_depth);
                if (ri->min_depth > tctxt->depth) {
                    DEBUG_PRINTF("failed depth check\n");
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    continue;
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_ONLY_EOD) {
                struct core_info *ci = &tctxtToScratch(tctxt)->core_info;
                if (end != ci->buf_offset + ci->len) {
                    DEBUG_PRINTF("should only match at end of data\n");
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    continue;
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_BOUNDS) {
                if (!in_anchored &&
                    !roseCheckRootBounds(end, ri->min_bound, ri->max_bound)) {
                    DEBUG_PRINTF("failed root bounds check\n");
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    continue;
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_NOT_HANDLED) {
                struct fatbit *handled = tctxtToScratch(tctxt)->handled_roles;
                if (fatbit_set(handled, t->handledKeyCount, ri->key)) {
                    DEBUG_PRINTF("key %u already set\n", ri->key);
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    continue;
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_LOOKAROUND) {
                if (!roseCheckLookaround(t, ri->index, ri->count, end, tctxt)) {
                    DEBUG_PRINTF("failed lookaround check\n");
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    continue;
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_LEFTFIX) {
                if (!roseTestLeftfix(t, ri->queue, ri->lag, ri->report, end,
                                     tctxt)) {
                    DEBUG_PRINTF("failed lookaround check\n");
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    continue;
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SOM_ADJUST) {
                assert(ri->distance <= end);
                som = end - ri->distance;
                DEBUG_PRINTF("som is (end - %u) = %llu\n", ri->distance, som);
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SOM_LEFTFIX) {
                som = roseGetHaigSom(t, ri->queue, ri->lag, tctxt);
                DEBUG_PRINTF("som from leftfix is %llu\n", som);
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(TRIGGER_INFIX) {
                roseTriggerInfix(t, som, end, ri->queue, ri->event, ri->cancel,
                                 tctxt);
                *work_done = 1;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(TRIGGER_SUFFIX) {
                if (roseHandleSuffixTrigger(t, ri->queue, ri->event, som, end,
                                            tctxt, in_anchored) ==
                    HWLM_TERMINATE_MATCHING) {
                    return HWLM_TERMINATE_MATCHING;
                }
                *work_done = 1;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(REPORT) {
                if (roseHandleMatch(t, tctxt->state, ri->report, end, tctxt,
                                    in_anchored) == HWLM_TERMINATE_MATCHING) {
                    return HWLM_TERMINATE_MATCHING;
                }
                *work_done = 1;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(REPORT_CHAIN) {
                if (roseCatchUpAndHandleChainMatch(t, tctxt->state, ri->report,
                                                   end, tctxt, in_anchored) ==
                    HWLM_TERMINATE_MATCHING) {
                    return HWLM_TERMINATE_MATCHING;
                }
                *work_done = 1;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(REPORT_EOD) {
                if (tctxt->cb(end, ri->report, tctxt->userCtx) ==
                    MO_HALT_MATCHING) {
                    return HWLM_TERMINATE_MATCHING;
                }
                *work_done = 1;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(REPORT_SOM_INT) {
                if (roseHandleSom(t, tctxt->state, ri->report, end, tctxt,
                                  in_anchored) == HWLM_TERMINATE_MATCHING) {
                    return HWLM_TERMINATE_MATCHING;
                }
                *work_done = 1;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(REPORT_SOM) {
                if (roseHandleSomSom(t, tctxt->state, ri->report, som, end,
                                     tctxt,
                                     in_anchored) == HWLM_TERMINATE_MATCHING) {
                    return HWLM_TERMINATE_MATCHING;
                }
                *work_done = 1;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(REPORT_SOM_KNOWN) {
                if (roseHandleSomMatch(t, tctxt->state, ri->report, som, end,
                                       tctxt, in_anchored) ==
                    HWLM_TERMINATE_MATCHING) {
                    return HWLM_TERMINATE_MATCHING;
                }
                *work_done = 1;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SET_STATE) {
                roseSetRole(t, tctxt->state, tctxt, ri->index, ri->depth);
                *work_done = 1;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SET_GROUPS) {
                tctxt->groups |= ri->groups;
                DEBUG_PRINTF("set groups 0x%llx -> 0x%llx\n", ri->groups,
                             tctxt->groups);
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SPARSE_ITER_BEGIN) {
                DEBUG_PRINTF("iter_offset=%u\n", ri->iter_offset);
                const struct mmbit_sparse_iter *it =
                    getByOffset(t, ri->iter_offset);
                assert(ISALIGNED(it));

                u32 idx = 0;
                u32 i = mmbit_sparse_iter_begin(getRoleState(tctxt->state),
                                                t->rolesWithStateCount, &idx,
                                                it, si_state);
                if (i == MMB_INVALID) {
                    DEBUG_PRINTF("no states in sparse iter are on\n");
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    continue;
                }

                struct hs_scratch *scratch = tctxtToScratch(tctxt);
                fatbit_clear(scratch->handled_roles);

                const u32 *jumps = getByOffset(t, ri->jump_table);
                DEBUG_PRINTF("state %u (idx=%u) is on, jump to %u\n", i, idx,
                             jumps[idx]);
                pc = pc_base + jumps[idx];
                continue;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SPARSE_ITER_NEXT) {
                DEBUG_PRINTF("iter_offset=%u, state=%u\n", ri->iter_offset,
                             ri->state);
                const struct mmbit_sparse_iter *it =
                    getByOffset(t, ri->iter_offset);
                assert(ISALIGNED(it));

                u32 idx = 0;
                u32 i = mmbit_sparse_iter_next(getRoleState(tctxt->state),
                                               t->rolesWithStateCount,
                                               ri->state, &idx, it, si_state);
                if (i == MMB_INVALID) {
                    DEBUG_PRINTF("no more states in sparse iter are on\n");
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    continue;
                }

                const u32 *jumps = getByOffset(t, ri->jump_table);
                DEBUG_PRINTF("state %u (idx=%u) is on, jump to %u\n", i, idx,
                             jumps[idx]);
                pc = pc_base + jumps[idx];
                continue;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(END) {
                DEBUG_PRINTF("finished\n");
                return HWLM_CONTINUE_MATCHING;
            }
            PROGRAM_NEXT_INSTRUCTION
        }
    }

    assert(0); // unreachable
    return HWLM_CONTINUE_MATCHING;
}

#undef PROGRAM_CASE
#undef PROGRAM_NEXT_INSTRUCTION

static rose_inline
void roseSquashGroup(struct RoseContext *tctxt, const struct RoseLiteral *tl) {
    assert(tl->squashesGroup);

    // we should be squashing a single group
    assert(popcount64(tl->groups) == 1);

    DEBUG_PRINTF("apply squash mask 0x%016llx, groups 0x%016llx -> 0x%016llx\n",
                 ~tl->groups, tctxt->groups, tctxt->groups & ~tl->groups);

    tctxt->groups &= ~tl->groups;
}

#endif // PROGRAM_RUNTIME_H
