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

#ifndef PROGRAM_RUNTIME_H
#define PROGRAM_RUNTIME_H

#include "catchup.h"
#include "counting_miracle.h"
#include "infix.h"
#include "match.h"
#include "miracle.h"
#include "report.h"
#include "rose.h"
#include "rose_internal.h"
#include "rose_program.h"
#include "rose_types.h"
#include "runtime.h"
#include "scratch.h"
#include "ue2common.h"
#include "util/compare.h"
#include "util/fatbit.h"
#include "util/multibit.h"

static rose_inline
int roseCheckBenefits(const struct core_info *ci, u64a end, u32 mask_rewind,
                      const u8 *and_mask, const u8 *exp_mask) {
    const u8 *data;

    // If the check works over part of the history and part of the buffer, we
    // create a temporary copy of the data in here so it's contiguous.
    u8 temp[MAX_MASK2_WIDTH];

    s64a buffer_offset = (s64a)end - ci->buf_offset;
    DEBUG_PRINTF("rel offset %lld\n", buffer_offset);
    if (buffer_offset >= mask_rewind) {
        data = ci->buf + buffer_offset - mask_rewind;
        DEBUG_PRINTF("all in one case data=%p buf=%p rewind=%u\n", data,
                     ci->buf, mask_rewind);
    } else if (buffer_offset <= 0) {
        data = ci->hbuf + ci->hlen + buffer_offset - mask_rewind;
        DEBUG_PRINTF("all in one case data=%p buf=%p rewind=%u\n", data,
                     ci->buf, mask_rewind);
    } else {
        u32 shortfall = mask_rewind - buffer_offset;
        DEBUG_PRINTF("shortfall of %u, rewind %u hlen %zu\n", shortfall,
                     mask_rewind, ci->hlen);
        data = temp;
        memcpy(temp, ci->hbuf + ci->hlen - shortfall, shortfall);
        memcpy(temp + shortfall, ci->buf, mask_rewind - shortfall);
    }

#ifdef DEBUG
    DEBUG_PRINTF("DATA: ");
    for (u32 i = 0; i < mask_rewind; i++) {
        printf("%c", ourisprint(data[i]) ? data[i] : '?');
    }
    printf(" (len=%u)\n", mask_rewind);
#endif

    u32 len = mask_rewind;
    while (len >= sizeof(u64a)) {
        u64a a = unaligned_load_u64a(data);
        a &= *(const u64a *)and_mask;
        if (a != *(const u64a *)exp_mask) {
            DEBUG_PRINTF("argh %016llx %016llx\n", a, *(const u64a *)exp_mask);
            return 0;
        }
        data += sizeof(u64a);
        and_mask += sizeof(u64a);
        exp_mask += sizeof(u64a);
        len -= sizeof(u64a);
    }

    while (len) {
        u8 a = *data;
        a &= *and_mask;
        if (a != *exp_mask) {
            DEBUG_PRINTF("argh d%02hhx =%02hhx am%02hhx  em%02hhx\n", a,
                          *data, *and_mask, *exp_mask);
            return 0;
        }
        data++;
        and_mask++;
        exp_mask++;
        len--;
    }

    return 1;
}

static rose_inline
void rosePushDelayedMatch(const struct RoseEngine *t,
                          struct hs_scratch *scratch, u32 delay,
                          u32 delay_index, u64a offset) {
    assert(delay);

    const u32 src_slot_index = delay;
    u32 slot_index = (src_slot_index + offset) & DELAY_MASK;

    struct RoseContext *tctxt = &scratch->tctxt;
    if (offset + src_slot_index <= tctxt->delayLastEndOffset) {
        DEBUG_PRINTF("skip too late\n");
        return;
    }

    const u32 delay_count = t->delay_count;
    struct fatbit **delaySlots = getDelaySlots(scratch);
    struct fatbit *slot = delaySlots[slot_index];

    DEBUG_PRINTF("pushing tab %u into slot %u\n", delay_index, slot_index);
    if (!(tctxt->filledDelayedSlots & (1U << slot_index))) {
        tctxt->filledDelayedSlots |= 1U << slot_index;
        fatbit_clear(slot);
    }

    fatbit_set(slot, delay_count, delay_index);
}

static rose_inline
char roseLeftfixCheckMiracles(const struct RoseEngine *t,
                              const struct LeftNfaInfo *left,
                              struct core_info *ci, struct mq *q, u64a end,
                              const char is_infix) {
    if (!is_infix && left->transient) {
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
    if (!is_infix) {
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

static rose_inline
hwlmcb_rv_t roseTriggerSuffix(const struct RoseEngine *t,
                              struct hs_scratch *scratch, u32 qi, u32 top,
                              u64a som, u64a end) {
    DEBUG_PRINTF("suffix qi=%u, top event=%u\n", qi, top);

    struct core_info *ci = &scratch->core_info;
    u8 *aa = getActiveLeafArray(t, ci->state);
    const u32 aaCount = t->activeArrayCount;
    const u32 qCount = t->queueCount;
    struct mq *q = &scratch->queues[qi];
    const struct NfaInfo *info = getNfaInfoByQueue(t, qi);
    const struct NFA *nfa = getNfaByInfo(t, info);

    s64a loc = (s64a)end - ci->buf_offset;
    assert(loc <= (s64a)ci->len && loc >= -(s64a)ci->hlen);

    if (!mmbit_set(aa, aaCount, qi)) {
        initQueue(q, qi, t, scratch);
        nfaQueueInitState(nfa, q);
        pushQueueAt(q, 0, MQE_START, loc);
        fatbit_set(scratch->aqa, qCount, qi);
    } else if (info->no_retrigger) {
        DEBUG_PRINTF("yawn\n");
        /* nfa only needs one top; we can go home now */
        return HWLM_CONTINUE_MATCHING;
    } else if (!fatbit_set(scratch->aqa, qCount, qi)) {
        initQueue(q, qi, t, scratch);
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
        } else if (ensureQueueFlushed(t, scratch, qi, loc)
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
char roseTestLeftfix(const struct RoseEngine *t, struct hs_scratch *scratch,
                     u32 qi, u32 leftfixLag, ReportID leftfixReport, u64a end,
                     const char is_infix) {
    struct core_info *ci = &scratch->core_info;

    u32 ri = queueToLeftIndex(t, qi);
    const struct LeftNfaInfo *left = getLeftTable(t) + ri;

    DEBUG_PRINTF("testing %s %s %u/%u with lag %u (maxLag=%u)\n",
                 (left->transient ? "transient" : "active"),
                 (is_infix ? "infix" : "prefix"),
                 ri, qi, leftfixLag, left->maxLag);

    assert(leftfixLag <= left->maxLag);
    assert(left->infix == is_infix);
    assert(!is_infix || !left->transient); // Only prefixes can be transient.

    struct mq *q = scratch->queues + qi;
    char *state = scratch->core_info.state;
    u8 *activeLeftArray = getActiveLeftArray(t, state);
    u32 qCount = t->queueCount;
    u32 arCount = t->activeLeftCount;

    if (!mmbit_isset(activeLeftArray, arCount, ri)) {
        DEBUG_PRINTF("engine is dead nothing to see here\n");
        return 0;
    }

    if (unlikely(end < leftfixLag)) {
        assert(0); /* lag is the literal length */
        return 0;
    }

    if (nfaSupportsZombie(getNfaByQueue(t, qi)) && ci->buf_offset
        && !fatbit_isset(scratch->aqa, qCount, qi)
        && isZombie(t, state, left)) {
        DEBUG_PRINTF("zombie\n");
        return 1;
    }

    if (!fatbit_set(scratch->aqa, qCount, qi)) {
        DEBUG_PRINTF("initing q %u\n", qi);
        initRoseQueue(t, qi, left, scratch);
        if (ci->buf_offset) { // there have been writes before us!
            s32 sp;
            if (!is_infix && left->transient) {
                sp = -(s32)ci->hlen;
            } else {
                sp = -(s32)loadRoseDelay(t, state, left);
            }

            /* transient nfas are always started fresh -> state not maintained
             * at stream boundary */

            pushQueueAt(q, 0, MQE_START, sp);
            if (is_infix || (ci->buf_offset + sp > 0 && !left->transient)) {
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

    if (!is_infix && left->transient) {
        s64a start_loc = loc - left->transient;
        if (q_cur_loc(q) < start_loc) {
            q->cur = q->end = 0;
            pushQueueAt(q, 0, MQE_START, start_loc);
            pushQueueAt(q, 1, MQE_TOP, start_loc);
            nfaQueueInitState(q->nfa, q);
        }
    }

    if (q_cur_loc(q) < loc || q_last_type(q) != MQE_START) {
        if (is_infix) {
            if (infixTooOld(q, loc)) {
                DEBUG_PRINTF("infix %u died of old age\n", ri);
                goto nfa_dead;
            }

            reduceInfixQueue(q, loc, left->maxQueueLen, q->nfa->maxWidth);
        }

        if (!roseLeftfixCheckMiracles(t, left, ci, q, end, is_infix)) {
            DEBUG_PRINTF("leftfix %u died due to miracle\n", ri);
            goto nfa_dead;
        }

#ifdef DEBUG
        debugQueue(q);
#endif

        pushQueueNoMerge(q, MQE_END, loc);

        char rv = nfaQueueExecRose(q->nfa, q, leftfixReport);
        if (!rv) { /* nfa is dead */
            DEBUG_PRINTF("leftfix %u died while trying to catch up\n", ri);
            goto nfa_dead;
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

nfa_dead:
    mmbit_unset(activeLeftArray, arCount, ri);
    scratch->tctxt.groups &= left->squash_mask;
    return 0;
}

static rose_inline
char roseTestPrefix(const struct RoseEngine *t, struct hs_scratch *scratch,
                    u32 qi, u32 leftfixLag, ReportID leftfixReport, u64a end) {
    return roseTestLeftfix(t, scratch, qi, leftfixLag, leftfixReport, end, 0);
}

static rose_inline
char roseTestInfix(const struct RoseEngine *t, struct hs_scratch *scratch,
                   u32 qi, u32 leftfixLag, ReportID leftfixReport, u64a end) {
    return roseTestLeftfix(t, scratch, qi, leftfixLag, leftfixReport, end, 1);
}

static rose_inline
void roseTriggerInfix(const struct RoseEngine *t, struct hs_scratch *scratch,
                      u64a start, u64a end, u32 qi, u32 topEvent, u8 cancel) {
    struct core_info *ci = &scratch->core_info;
    s64a loc = (s64a)end - ci->buf_offset;

    u32 ri = queueToLeftIndex(t, qi);
    assert(topEvent < MQE_INVALID);

    const struct LeftNfaInfo *left = getLeftInfoByQueue(t, qi);
    assert(!left->transient);

    DEBUG_PRINTF("rose %u (qi=%u) event %u\n", ri, qi, topEvent);

    struct mq *q = scratch->queues + qi;
    const struct NfaInfo *info = getNfaInfoByQueue(t, qi);

    char *state = ci->state;
    u8 *activeLeftArray = getActiveLeftArray(t, state);
    const u32 arCount = t->activeLeftCount;
    char alive = mmbit_set(activeLeftArray, arCount, ri);

    if (alive && info->no_retrigger) {
        DEBUG_PRINTF("yawn\n");
        return;
    }

    struct fatbit *aqa = scratch->aqa;
    const u32 qCount = t->queueCount;

    if (alive && nfaSupportsZombie(getNfaByInfo(t, info)) && ci->buf_offset &&
        !fatbit_isset(aqa, qCount, qi) && isZombie(t, state, left)) {
        DEBUG_PRINTF("yawn - zombie\n");
        return;
    }

    if (cancel) {
        DEBUG_PRINTF("dominating top: (re)init\n");
        fatbit_set(aqa, qCount, qi);
        initRoseQueue(t, qi, left, scratch);
        pushQueueAt(q, 0, MQE_START, loc);
        nfaQueueInitState(q->nfa, q);
    } else if (!fatbit_set(aqa, qCount, qi)) {
        DEBUG_PRINTF("initing %u\n", qi);
        initRoseQueue(t, qi, left, scratch);
        if (alive) {
            s32 sp = -(s32)loadRoseDelay(t, state, left);
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
        reduceInfixQueue(q, loc, left->maxQueueLen, q->nfa->maxWidth);

        if (isQueueFull(q)) {
            /* still full - reduceInfixQueue did nothing */
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

static rose_inline
hwlmcb_rv_t roseReport(const struct RoseEngine *t, struct hs_scratch *scratch,
                       u64a end, ReportID onmatch, s32 offset_adjust,
                       u32 ekey) {
    assert(!t->needsCatchup || end == scratch->tctxt.minMatchOffset);
    DEBUG_PRINTF("firing callback onmatch=%u, end=%llu\n", onmatch, end);
    updateLastMatchOffset(&scratch->tctxt, end);

    int cb_rv = roseDeliverReport(end, onmatch, offset_adjust, scratch, ekey);
    if (cb_rv == MO_HALT_MATCHING) {
        DEBUG_PRINTF("termination requested\n");
        return HWLM_TERMINATE_MATCHING;
    }

    if (ekey == INVALID_EKEY || cb_rv == ROSE_CONTINUE_MATCHING_NO_EXHAUST) {
        return HWLM_CONTINUE_MATCHING;
    }

    return roseHaltIfExhausted(t, scratch);
}

/* catches up engines enough to ensure any earlier mpv triggers are enqueued
 * and then adds the trigger to the mpv queue. Must not be called during catch
 * up */
static rose_inline
hwlmcb_rv_t roseCatchUpAndHandleChainMatch(const struct RoseEngine *t,
                                           struct hs_scratch *scratch,
                                           u32 event, u64a top_squash_distance,
                                           u64a end, const char in_catchup) {
    if (!in_catchup &&
        roseCatchUpMpvFeeders(t, scratch, end) == HWLM_TERMINATE_MATCHING) {
        return HWLM_TERMINATE_MATCHING;
    }
    return roseHandleChainMatch(t, scratch, event, top_squash_distance, end,
                                in_catchup);
}

static rose_inline
void roseHandleSom(UNUSED const struct RoseEngine *t,
                   struct hs_scratch *scratch, const struct som_operation *sr,
                   u64a end) {
    DEBUG_PRINTF("end=%llu, minMatchOffset=%llu\n", end,
                 scratch->tctxt.minMatchOffset);

    assert(!t->needsCatchup || end == scratch->tctxt.minMatchOffset);
    updateLastMatchOffset(&scratch->tctxt, end);
    handleSomInternal(scratch, sr, end);
}

static rose_inline
hwlmcb_rv_t roseReportSom(const struct RoseEngine *t,
                          struct hs_scratch *scratch, u64a start, u64a end,
                          ReportID onmatch, s32 offset_adjust, u32 ekey) {
    assert(!t->needsCatchup || end == scratch->tctxt.minMatchOffset);
    DEBUG_PRINTF("firing som callback onmatch=%u, start=%llu, end=%llu\n",
                 onmatch, start, end);
    updateLastMatchOffset(&scratch->tctxt, end);

    int cb_rv = roseDeliverSomReport(start, end, onmatch, offset_adjust,
                                     scratch, ekey);
    if (cb_rv == MO_HALT_MATCHING) {
        DEBUG_PRINTF("termination requested\n");
        return HWLM_TERMINATE_MATCHING;
    }

    if (ekey == INVALID_EKEY || cb_rv == ROSE_CONTINUE_MATCHING_NO_EXHAUST) {
        return HWLM_CONTINUE_MATCHING;
    }

    return roseHaltIfExhausted(t, scratch);
}

static rose_inline
void roseHandleSomSom(UNUSED const struct RoseEngine *t,
                      struct hs_scratch *scratch,
                      const struct som_operation *sr, u64a start, u64a end) {
    DEBUG_PRINTF("start=%llu, end=%llu, minMatchOffset=%llu\n", start, end,
                 scratch->tctxt.minMatchOffset);

    assert(!t->needsCatchup || end == scratch->tctxt.minMatchOffset);
    updateLastMatchOffset(&scratch->tctxt, end);
    setSomFromSomAware(scratch, sr, start, end);
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
int roseCheckLookaround(const struct RoseEngine *t,
                        const struct hs_scratch *scratch, u32 lookaroundIndex,
                        u32 lookaroundCount, u64a end) {
    assert(lookaroundIndex != MO_INVALID_IDX);
    assert(lookaroundCount > 0);

    const struct core_info *ci = &scratch->core_info;
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
u64a roseGetHaigSom(const struct RoseEngine *t, struct hs_scratch *scratch,
                    const u32 qi, UNUSED const u32 leftfixLag) {
    u32 ri = queueToLeftIndex(t, qi);

    UNUSED const struct LeftNfaInfo *left = getLeftTable(t) + ri;

    DEBUG_PRINTF("testing %s prefix %u/%u with lag %u (maxLag=%u)\n",
                 left->transient ? "transient" : "active", ri, qi,
                 leftfixLag, left->maxLag);

    assert(leftfixLag <= left->maxLag);

    struct mq *q = scratch->queues + qi;

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
char roseCheckBounds(u64a end, u64a min_bound, u64a max_bound) {
    DEBUG_PRINTF("check offset=%llu against bounds [%llu,%llu]\n", end,
                 min_bound, max_bound);
    assert(min_bound <= max_bound);
    return end >= min_bound && end <= max_bound;
}

static
void updateSeqPoint(struct RoseContext *tctxt, u64a offset,
                    const char from_mpv) {
    if (from_mpv) {
        updateMinMatchOffsetFromMpv(tctxt, offset);
    } else {
        updateMinMatchOffset(tctxt, offset);
    }
}

#define PROGRAM_CASE(name)                                                     \
    case ROSE_INSTR_##name: {                                                  \
        DEBUG_PRINTF("instruction: " #name " (pc=%u)\n",                       \
                     programOffset + (u32)(pc - pc_base));                     \
        const struct ROSE_STRUCT_##name *ri =                                  \
            (const struct ROSE_STRUCT_##name *)pc;

#define PROGRAM_NEXT_INSTRUCTION                                               \
    pc += ROUNDUP_N(sizeof(*ri), ROSE_INSTR_MIN_ALIGN);                        \
    break;                                                                     \
    }

static rose_inline
hwlmcb_rv_t roseRunProgram(const struct RoseEngine *t,
                           struct hs_scratch *scratch, u32 programOffset,
                           u64a som, u64a end, size_t match_len,
                           char in_anchored, char in_catchup, char from_mpv,
                           char skip_mpv_catchup) {
    DEBUG_PRINTF("program=%u, offsets [%llu,%llu]\n", programOffset, som, end);

    assert(programOffset >= sizeof(struct RoseEngine));
    assert(programOffset < t->size);

    const char *pc_base = getByOffset(t, programOffset);
    const char *pc = pc_base;

    // Local sparse iterator state for programs that use the SPARSE_ITER_BEGIN
    // and SPARSE_ITER_NEXT instructions.
    struct mmbit_sparse_state si_state[MAX_SPARSE_ITER_STATES];

    // If this program has an effect, work_done will be set to one (which may
    // allow the program to squash groups).
    int work_done = 0;

    struct RoseContext *tctxt = &scratch->tctxt;

    assert(*(const u8 *)pc != ROSE_INSTR_END);

    for (;;) {
        assert(ISALIGNED_N(pc, ROSE_INSTR_MIN_ALIGN));
        assert(pc >= pc_base);
        assert((size_t)(pc - pc_base) < t->size);
        const u8 code = *(const u8 *)pc;
        assert(code <= ROSE_INSTR_END);

        switch ((enum RoseInstructionCode)code) {
            PROGRAM_CASE(ANCHORED_DELAY) {
                if (in_anchored && end > t->floatingMinLiteralMatchOffset) {
                    DEBUG_PRINTF("delay until playback\n");
                    tctxt->groups |= ri->groups;
                    work_done = 1;
                    assert(ri->done_jump); // must progress
                    pc += ri->done_jump;
                    continue;
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_LIT_MASK) {
                assert(match_len);
                struct core_info *ci = &scratch->core_info;
                if (!roseCheckBenefits(ci, end, match_len, ri->and_mask.a8,
                                       ri->cmp_mask.a8)) {
                    DEBUG_PRINTF("halt: failed mask check\n");
                    return HWLM_CONTINUE_MATCHING;
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_LIT_EARLY) {
                if (end < t->floatingMinLiteralMatchOffset) {
                    DEBUG_PRINTF("halt: too soon, min offset=%u\n",
                                 t->floatingMinLiteralMatchOffset);
                    return HWLM_CONTINUE_MATCHING;
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_GROUPS) {
                DEBUG_PRINTF("groups=0x%llx, checking instr groups=0x%llx\n",
                             tctxt->groups, ri->groups);
                if (!(ri->groups & tctxt->groups)) {
                    DEBUG_PRINTF("halt: no groups are set\n");
                    return HWLM_CONTINUE_MATCHING;
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_ONLY_EOD) {
                struct core_info *ci = &scratch->core_info;
                if (end != ci->buf_offset + ci->len) {
                    DEBUG_PRINTF("should only match at end of data\n");
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    continue;
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_BOUNDS) {
                if (!roseCheckBounds(end, ri->min_bound, ri->max_bound)) {
                    DEBUG_PRINTF("failed bounds check\n");
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    continue;
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_NOT_HANDLED) {
                struct fatbit *handled = scratch->handled_roles;
                if (fatbit_set(handled, t->handledKeyCount, ri->key)) {
                    DEBUG_PRINTF("key %u already set\n", ri->key);
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    continue;
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_LOOKAROUND) {
                if (!roseCheckLookaround(t, scratch, ri->index, ri->count,
                                         end)) {
                    DEBUG_PRINTF("failed lookaround check\n");
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    continue;
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_INFIX) {
                if (!roseTestInfix(t, scratch, ri->queue, ri->lag, ri->report,
                                   end)) {
                    DEBUG_PRINTF("failed infix check\n");
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    continue;
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_PREFIX) {
                if (!roseTestPrefix(t, scratch, ri->queue, ri->lag, ri->report,
                                    end)) {
                    DEBUG_PRINTF("failed prefix check\n");
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    continue;
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(PUSH_DELAYED) {
                rosePushDelayedMatch(t, scratch, ri->delay, ri->index, end);
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CATCH_UP) {
                if (roseCatchUpTo(t, scratch, end) == HWLM_TERMINATE_MATCHING) {
                    return HWLM_TERMINATE_MATCHING;
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CATCH_UP_MPV) {
                if (from_mpv || skip_mpv_catchup) {
                    DEBUG_PRINTF("skipping mpv catchup\n");
                } else if (roseCatchUpMPV(t,
                                          end - scratch->core_info.buf_offset,
                                          scratch) == HWLM_TERMINATE_MATCHING) {
                    return HWLM_TERMINATE_MATCHING;
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
                som = roseGetHaigSom(t, scratch, ri->queue, ri->lag);
                DEBUG_PRINTF("som from leftfix is %llu\n", som);
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SOM_FROM_REPORT) {
                som = handleSomExternal(scratch, &ri->som, end);
                DEBUG_PRINTF("som from report %u is %llu\n", ri->som.onmatch,
                             som);
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SOM_ZERO) {
                DEBUG_PRINTF("setting SOM to zero\n");
                som = 0;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(TRIGGER_INFIX) {
                roseTriggerInfix(t, scratch, som, end, ri->queue, ri->event,
                                 ri->cancel);
                work_done = 1;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(TRIGGER_SUFFIX) {
                if (roseTriggerSuffix(t, scratch, ri->queue, ri->event, som,
                                      end) == HWLM_TERMINATE_MATCHING) {
                    return HWLM_TERMINATE_MATCHING;
                }
                work_done = 1;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(DEDUPE) {
                updateSeqPoint(tctxt, end, from_mpv);
                const char do_som = t->hasSom; // TODO: constant propagate
                const char is_external_report = 1;
                enum DedupeResult rv =
                    dedupeCatchup(t, scratch, end, som, end + ri->offset_adjust,
                                  ri->dkey, ri->offset_adjust,
                                  is_external_report, ri->quash_som, do_som);
                switch (rv) {
                case DEDUPE_HALT:
                    return HWLM_TERMINATE_MATCHING;
                case DEDUPE_SKIP:
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    continue;
                case DEDUPE_CONTINUE:
                    break;
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(DEDUPE_SOM) {
                updateSeqPoint(tctxt, end, from_mpv);
                const char is_external_report = 0;
                const char do_som = 1;
                enum DedupeResult rv =
                    dedupeCatchup(t, scratch, end, som, end + ri->offset_adjust,
                                  ri->dkey, ri->offset_adjust,
                                  is_external_report, ri->quash_som, do_som);
                switch (rv) {
                case DEDUPE_HALT:
                    return HWLM_TERMINATE_MATCHING;
                case DEDUPE_SKIP:
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    continue;
                case DEDUPE_CONTINUE:
                    break;
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(REPORT_CHAIN) {
                // Note: sequence points updated inside this function.
                if (roseCatchUpAndHandleChainMatch(
                        t, scratch, ri->event, ri->top_squash_distance, end,
                        in_catchup) == HWLM_TERMINATE_MATCHING) {
                    return HWLM_TERMINATE_MATCHING;
                }
                work_done = 1;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(REPORT_SOM_INT) {
                updateSeqPoint(tctxt, end, from_mpv);
                roseHandleSom(t, scratch, &ri->som, end);
                work_done = 1;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(REPORT_SOM_AWARE) {
                updateSeqPoint(tctxt, end, from_mpv);
                roseHandleSomSom(t, scratch, &ri->som, som, end);
                work_done = 1;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(REPORT) {
                updateSeqPoint(tctxt, end, from_mpv);
                if (roseReport(t, scratch, end, ri->onmatch, ri->offset_adjust,
                               INVALID_EKEY) == HWLM_TERMINATE_MATCHING) {
                    return HWLM_TERMINATE_MATCHING;
                }
                work_done = 1;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(REPORT_EXHAUST) {
                updateSeqPoint(tctxt, end, from_mpv);
                if (roseReport(t, scratch, end, ri->onmatch, ri->offset_adjust,
                               ri->ekey) == HWLM_TERMINATE_MATCHING) {
                    return HWLM_TERMINATE_MATCHING;
                }
                work_done = 1;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(REPORT_SOM) {
                updateSeqPoint(tctxt, end, from_mpv);
                if (roseReportSom(t, scratch, som, end, ri->onmatch,
                                  ri->offset_adjust,
                                  INVALID_EKEY) == HWLM_TERMINATE_MATCHING) {
                    return HWLM_TERMINATE_MATCHING;
                }
                work_done = 1;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(REPORT_SOM_EXHAUST) {
                updateSeqPoint(tctxt, end, from_mpv);
                if (roseReportSom(t, scratch, som, end, ri->onmatch,
                                  ri->offset_adjust,
                                  ri->ekey) == HWLM_TERMINATE_MATCHING) {
                    return HWLM_TERMINATE_MATCHING;
                }
                work_done = 1;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(DEDUPE_AND_REPORT) {
                updateSeqPoint(tctxt, end, from_mpv);
                const char do_som = t->hasSom; // TODO: constant propagate
                const char is_external_report = 1;
                enum DedupeResult rv =
                    dedupeCatchup(t, scratch, end, som, end + ri->offset_adjust,
                                  ri->dkey, ri->offset_adjust,
                                  is_external_report, ri->quash_som, do_som);
                switch (rv) {
                case DEDUPE_HALT:
                    return HWLM_TERMINATE_MATCHING;
                case DEDUPE_SKIP:
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    continue;
                case DEDUPE_CONTINUE:
                    break;
                }

                const u32 ekey = INVALID_EKEY;
                if (roseReport(t, scratch, end, ri->onmatch, ri->offset_adjust,
                               ekey) == HWLM_TERMINATE_MATCHING) {
                    return HWLM_TERMINATE_MATCHING;
                }
                work_done = 1;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(FINAL_REPORT) {
                updateSeqPoint(tctxt, end, from_mpv);
                if (roseReport(t, scratch, end, ri->onmatch, ri->offset_adjust,
                               INVALID_EKEY) == HWLM_TERMINATE_MATCHING) {
                    return HWLM_TERMINATE_MATCHING;
                }
                /* One-shot specialisation: this instruction always terminates
                 * execution of the program. */
                return HWLM_CONTINUE_MATCHING;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_EXHAUSTED) {
                DEBUG_PRINTF("check ekey %u\n", ri->ekey);
                assert(ri->ekey != INVALID_EKEY);
                assert(ri->ekey < t->ekeyCount);
                const char *evec = scratch->core_info.exhaustionVector;
                if (isExhausted(t, evec, ri->ekey)) {
                    DEBUG_PRINTF("ekey %u already set, match is exhausted\n",
                                 ri->ekey);
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    continue;
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_MIN_LENGTH) {
                DEBUG_PRINTF("check min length %llu (adj %d)\n", ri->min_length,
                             ri->end_adj);
                assert(ri->min_length > 0);
                assert(ri->end_adj == 0 || ri->end_adj == -1);
                assert(som == HS_OFFSET_PAST_HORIZON || som <= end);
                if (som != HS_OFFSET_PAST_HORIZON &&
                    ((end + ri->end_adj) - som < ri->min_length)) {
                    DEBUG_PRINTF("failed check, match len %llu\n",
                                 (u64a)((end + ri->end_adj) - som));
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    continue;
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SET_STATE) {
                DEBUG_PRINTF("set state index %u\n", ri->index);
                mmbit_set(getRoleState(scratch->core_info.state),
                          t->rolesWithStateCount, ri->index);
                work_done = 1;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SET_GROUPS) {
                tctxt->groups |= ri->groups;
                DEBUG_PRINTF("set groups 0x%llx -> 0x%llx\n", ri->groups,
                             tctxt->groups);
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SQUASH_GROUPS) {
                assert(popcount64(ri->groups) == 63); // Squash only one group.
                if (work_done) {
                    tctxt->groups &= ri->groups;
                    DEBUG_PRINTF("squash groups 0x%llx -> 0x%llx\n", ri->groups,
                                 tctxt->groups);
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_STATE) {
                DEBUG_PRINTF("check state %u\n", ri->index);
                const u8 *roles = getRoleState(scratch->core_info.state);
                if (!mmbit_isset(roles, t->rolesWithStateCount, ri->index)) {
                    DEBUG_PRINTF("state not on\n");
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    continue;
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SPARSE_ITER_BEGIN) {
                DEBUG_PRINTF("iter_offset=%u\n", ri->iter_offset);
                const struct mmbit_sparse_iter *it =
                    getByOffset(t, ri->iter_offset);
                assert(ISALIGNED(it));

                const u8 *roles = getRoleState(scratch->core_info.state);

                u32 idx = 0;
                u32 i = mmbit_sparse_iter_begin(roles, t->rolesWithStateCount,
                                                &idx, it, si_state);
                if (i == MMB_INVALID) {
                    DEBUG_PRINTF("no states in sparse iter are on\n");
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    continue;
                }

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

                const u8 *roles = getRoleState(scratch->core_info.state);

                u32 idx = 0;
                u32 i = mmbit_sparse_iter_next(roles, t->rolesWithStateCount,
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

#endif // PROGRAM_RUNTIME_H
