/*
 * Copyright (c) 2015-2019, Intel Corporation
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
 * \brief Rose runtime: program interpreter.
 */

#include "program_runtime.h"

#include "catchup.h"
#include "counting_miracle.h"
#include "infix.h"
#include "match.h"
#include "miracle.h"
#include "report.h"
#include "rose_common.h"
#include "rose_internal.h"
#include "rose_program.h"
#include "rose_types.h"
#include "validate_mask.h"
#include "validate_shufti.h"
#include "runtime.h"
#include "util/compare.h"
#include "util/copybytes.h"
#include "util/fatbit.h"
#include "util/multibit.h"

/* Inline implementation follows. */

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
void recordAnchoredLiteralMatch(const struct RoseEngine *t,
                                struct hs_scratch *scratch, u32 anch_id,
                                u64a end) {
    assert(end);

    if (end <= t->floatingMinLiteralMatchOffset) {
        return;
    }

    struct fatbit **anchoredLiteralRows = getAnchoredLiteralLog(scratch);

    DEBUG_PRINTF("record %u (of %u) @ %llu\n", anch_id, t->anchored_count, end);

    if (!bf64_set(&scratch->al_log_sum, end - 1)) {
        // first time, clear row
        DEBUG_PRINTF("clearing %llu/%u\n", end - 1, t->anchored_count);
        fatbit_clear(anchoredLiteralRows[end - 1]);
    }

    assert(anch_id < t->anchored_count);
    fatbit_set(anchoredLiteralRows[end - 1], t->anchored_count, anch_id);
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
    assert(loc >= q_cur_loc(q) || left->eager);
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
    } else if (q_cur_loc(q) > loc) {
        /* an eager leftfix may have already progressed past loc if there is no
         * match at loc. */
        assert(left->eager);
        return 0;
    } else {
        assert(q_cur_loc(q) == loc);
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
 * and then adds the trigger to the mpv queue. */
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
void roseHandleSom(struct hs_scratch *scratch, const struct som_operation *sr,
                   u64a end) {
    DEBUG_PRINTF("end=%llu, minMatchOffset=%llu\n", end,
                 scratch->tctxt.minMatchOffset);

    updateLastMatchOffset(&scratch->tctxt, end);
    handleSomInternal(scratch, sr, end);
}

static rose_inline
hwlmcb_rv_t roseReportSom(const struct RoseEngine *t,
                          struct hs_scratch *scratch, u64a start, u64a end,
                          ReportID onmatch, s32 offset_adjust, u32 ekey) {
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
void roseHandleSomSom(struct hs_scratch *scratch,
                      const struct som_operation *sr, u64a start, u64a end) {
    DEBUG_PRINTF("start=%llu, end=%llu, minMatchOffset=%llu\n", start, end,
                 scratch->tctxt.minMatchOffset);

    updateLastMatchOffset(&scratch->tctxt, end);
    setSomFromSomAware(scratch, sr, start, end);
}

static rose_inline
hwlmcb_rv_t roseSetExhaust(const struct RoseEngine *t,
                           struct hs_scratch *scratch, u32 ekey) {
    assert(scratch);
    assert(scratch->magic == SCRATCH_MAGIC);

    struct core_info *ci = &scratch->core_info;

    assert(!can_stop_matching(scratch));
    assert(!isExhausted(ci->rose, ci->exhaustionVector, ekey));

    markAsMatched(ci->rose, ci->exhaustionVector, ekey);

    return roseHaltIfExhausted(t, scratch);
}

static really_inline
int reachHasBit(const u8 *reach, u8 c) {
    return !!(reach[c / 8U] & (u8)1U << (c % 8U));
}

/*
 * Generate a 8-byte valid_mask with #high bytes 0 from the highest side
 * and #low bytes 0 from the lowest side
 * and (8 - high - low) bytes '0xff' in the middle.
 */
static rose_inline
u64a generateValidMask(const s32 high, const s32 low) {
    assert(high + low < 8);
    DEBUG_PRINTF("high %d low %d\n", high, low);
    const u64a ones = ~0ull;
    return (ones << ((high + low) * 8)) >> (high * 8);
}

/*
 * Do the single-byte check if only one lookaround entry exists
 * and it's a single mask.
 * Return success if the byte is in the future or before history
 * (offset is greater than (history) buffer length).
 */
static rose_inline
int roseCheckByte(const struct core_info *ci, u8 and_mask, u8 cmp_mask,
                  u8 negation, s32 checkOffset, u64a end) {
    DEBUG_PRINTF("end=%llu, buf_offset=%llu, buf_end=%llu\n", end,
                 ci->buf_offset, ci->buf_offset + ci->len);
    if (unlikely(checkOffset < 0 && (u64a)(0 - checkOffset) > end)) {
        DEBUG_PRINTF("too early, fail\n");
        return 0;
    }

    const s64a base_offset = end - ci->buf_offset;
    s64a offset = base_offset + checkOffset;
    DEBUG_PRINTF("checkOffset=%d offset=%lld\n", checkOffset, offset);
    u8 c;
    if (offset >= 0) {
        if (offset >= (s64a)ci->len) {
            DEBUG_PRINTF("in the future\n");
            return 1;
        } else {
            assert(offset < (s64a)ci->len);
            DEBUG_PRINTF("check byte in buffer\n");
            c = ci->buf[offset];
        }
    } else {
        if (offset >= -(s64a) ci->hlen) {
            DEBUG_PRINTF("check byte in history\n");
            c = ci->hbuf[ci->hlen + offset];
        } else {
            DEBUG_PRINTF("before history and return\n");
            return 1;
        }
    }

    if (((and_mask & c) != cmp_mask) ^ negation) {
        DEBUG_PRINTF("char 0x%02x at offset %lld failed byte check\n",
                     c, offset);
        return 0;
    }

    DEBUG_PRINTF("real offset=%lld char=%02x\n", offset, c);
    DEBUG_PRINTF("OK :)\n");
    return 1;
}

static rose_inline
int roseCheckMask(const struct core_info *ci, u64a and_mask, u64a cmp_mask,
                  u64a neg_mask, s32 checkOffset, u64a end) {
    const s64a base_offset = (s64a)end - ci->buf_offset;
    s64a offset = base_offset + checkOffset;
    DEBUG_PRINTF("rel offset %lld\n",base_offset);
    DEBUG_PRINTF("checkOffset %d offset %lld\n", checkOffset, offset);
    if (unlikely(checkOffset < 0 && (u64a)(0 - checkOffset) > end)) {
        DEBUG_PRINTF("too early, fail\n");
        return 0;
    }

    u64a data = 0;
    u64a valid_data_mask = ~0ULL; // mask for validate check.
    //A 0xff byte means that this byte is in the buffer.
    s32 shift_l = 0; // size of bytes in the future.
    s32 shift_r = 0; // size of bytes before the history.
    s32 h_len = 0; // size of bytes in the history buffer.
    s32 c_len = 8; // size of bytes in the current buffer.
    if (offset < 0) {
        // in or before history buffer.
        if (offset + 8 <= -(s64a)ci->hlen) {
            DEBUG_PRINTF("before history and return\n");
            return 1;
        }
        const u8 *h_start = ci->hbuf; // start pointer in history buffer.
        if (offset < -(s64a)ci->hlen) {
            // some bytes are before history.
            shift_r = -(offset + (s64a)ci->hlen);
            DEBUG_PRINTF("shift_r %d", shift_r);
        } else {
            h_start += ci->hlen + offset;
        }
        if (offset + 7 < 0) {
            DEBUG_PRINTF("all in history buffer\n");
            data = partial_load_u64a(h_start, 8 - shift_r);
        } else {
            // history part
            c_len = offset + 8;
            h_len = -offset - shift_r;
            DEBUG_PRINTF("%d bytes in history\n", h_len);
            s64a data_h = 0;
            data_h = partial_load_u64a(h_start, h_len);
            // current part
            if (c_len > (s64a)ci->len) {
                shift_l = c_len - ci->len;
                c_len = ci->len;
            }
            data = partial_load_u64a(ci->buf, c_len);
            data <<= h_len << 3;
            data |= data_h;
        }
        if (shift_r) {
            data <<= shift_r << 3;
        }
    } else {
        // current buffer.
        if (offset + c_len > (s64a)ci->len) {
            if (offset >= (s64a)ci->len) {
                DEBUG_PRINTF("all in the future\n");
                return 1;
            }
            // some  bytes in the future.
            shift_l = offset + c_len - ci->len;
            c_len = ci->len - offset;
            data = partial_load_u64a(ci->buf + offset, c_len);
        } else {
            data = unaligned_load_u64a(ci->buf + offset);
        }
    }

    if (shift_l || shift_r) {
        valid_data_mask = generateValidMask(shift_l, shift_r);
    }
    DEBUG_PRINTF("valid_data_mask %llx\n", valid_data_mask);

    if (validateMask(data, valid_data_mask,
                     and_mask, cmp_mask, neg_mask)) {
        DEBUG_PRINTF("check mask successfully\n");
        return 1;
    } else {
        return 0;
    }
}

static rose_inline
int roseCheckMask32(const struct core_info *ci, const u8 *and_mask,
                    const u8 *cmp_mask, const u32 neg_mask,
                    s32 checkOffset, u64a end) {
    const s64a base_offset = (s64a)end - ci->buf_offset;
    s64a offset = base_offset + checkOffset;
    DEBUG_PRINTF("end %lld base_offset %lld\n", end, base_offset);
    DEBUG_PRINTF("checkOffset %d offset %lld\n", checkOffset, offset);

    if (unlikely(checkOffset < 0 && (u64a)(0 - checkOffset) > end)) {
        DEBUG_PRINTF("too early, fail\n");
        return 0;
    }

    m256 data = zeroes256(); // consists of the following four parts.
    s32 c_shift = 0; // blank bytes after current.
    s32 h_shift = 0; // blank bytes before history.
    s32 h_len = 32; // number of bytes from history buffer.
    s32 c_len = 0; // number of bytes from current buffer.
    /* h_shift + h_len + c_len + c_shift = 32 need to be hold.*/

    if (offset < 0) {
        s32 h_offset = 0; // the start offset in history buffer.
        if (offset < -(s64a)ci->hlen) {
            if (offset + 32 <= -(s64a)ci->hlen) {
                DEBUG_PRINTF("all before history\n");
                return 1;
            }
            h_shift = -(offset + (s64a)ci->hlen);
            h_len = 32 - h_shift;
        } else {
            h_offset = ci->hlen + offset;
        }
        if (offset + 32 > 0) {
            // part in current buffer.
            c_len = offset + 32;
            h_len = -(offset + h_shift);
            if (c_len > (s64a)ci->len) {
                // out of current buffer.
                c_shift = c_len - ci->len;
                c_len = ci->len;
            }
            copy_upto_32_bytes((u8 *)&data - offset, ci->buf, c_len);
        }
        assert(h_shift + h_len + c_len + c_shift == 32);
        copy_upto_32_bytes((u8 *)&data + h_shift, ci->hbuf + h_offset, h_len);
    } else {
        if (offset + 32 > (s64a)ci->len) {
            if (offset >= (s64a)ci->len) {
                DEBUG_PRINTF("all in the future.\n");
                return 1;
            }
            c_len = ci->len - offset;
            c_shift = 32 - c_len;
            copy_upto_32_bytes((u8 *)&data, ci->buf + offset, c_len);
        } else {
            data = loadu256(ci->buf + offset);
        }
    }
    DEBUG_PRINTF("h_shift %d c_shift %d\n", h_shift, c_shift);
    DEBUG_PRINTF("h_len %d c_len %d\n", h_len, c_len);
    // we use valid_data_mask to blind bytes before history/in the future.
    u32 valid_data_mask;
    valid_data_mask = (~0u) << (h_shift + c_shift) >> (c_shift);

    m256 and_mask_m256 = loadu256(and_mask);
    m256 cmp_mask_m256 = loadu256(cmp_mask);
    if (validateMask32(data, valid_data_mask, and_mask_m256,
                       cmp_mask_m256, neg_mask)) {
        DEBUG_PRINTF("Mask32 passed\n");
        return 1;
    }
    return 0;
}

// get 128/256 bits data from history and current buffer.
// return data and valid_data_mask.
static rose_inline
u32 getBufferDataComplex(const struct core_info *ci, const s64a loc,
                         u8 *data, const u32 data_len) {
    assert(data_len == 16 || data_len == 32);
    s32 c_shift = 0; // blank bytes after current.
    s32 h_shift = 0; // blank bytes before history.
    s32 h_len = data_len; // number of bytes from history buffer.
    s32 c_len = 0; // number of bytes from current buffer.
    if (loc < 0) {
        s32 h_offset = 0; // the start offset in history buffer.
        if (loc < -(s64a)ci->hlen) {
            if (loc + data_len <= -(s64a)ci->hlen) {
                DEBUG_PRINTF("all before history\n");
                return 0;
            }
            h_shift = -(loc + (s64a)ci->hlen);
            h_len = data_len - h_shift;
        } else {
            h_offset = ci->hlen + loc;
        }
        if (loc + data_len > 0) {
            // part in current buffer.
            c_len = loc + data_len;
            h_len = -(loc + h_shift);
            if (c_len > (s64a)ci->len) {
                // out of current buffer.
                c_shift = c_len - ci->len;
                c_len = ci->len;
            }
            copy_upto_32_bytes(data - loc, ci->buf, c_len);
        }
        assert(h_shift + h_len + c_len + c_shift == (s32)data_len);
        copy_upto_32_bytes(data + h_shift, ci->hbuf + h_offset, h_len);
    } else {
        if (loc + data_len > (s64a)ci->len) {
            if (loc >= (s64a)ci->len) {
                DEBUG_PRINTF("all in the future.\n");
                return 0;
            }
            c_len = ci->len - loc;
            c_shift = data_len - c_len;
            copy_upto_32_bytes(data, ci->buf + loc, c_len);
        } else {
            if (data_len == 16) {
                storeu128(data, loadu128(ci->buf + loc));
                return 0xffff;
            } else {
                storeu256(data, loadu256(ci->buf + loc));
                return 0xffffffff;
            }
        }
    }
    DEBUG_PRINTF("h_shift %d c_shift %d\n", h_shift, c_shift);
    DEBUG_PRINTF("h_len %d c_len %d\n", h_len, c_len);

    if (data_len == 16) {
        return (u16)(0xffff << (h_shift + c_shift)) >> c_shift;
    } else {
        return (~0u) << (h_shift + c_shift) >> c_shift;
    }
}

static rose_inline
m128 getData128(const struct core_info *ci, s64a offset, u32 *valid_data_mask) {
    if (offset > 0 && offset + sizeof(m128) <= ci->len) {
        *valid_data_mask = 0xffff;
        return loadu128(ci->buf + offset);
    }
    ALIGN_DIRECTIVE u8 data[sizeof(m128)];
    *valid_data_mask = getBufferDataComplex(ci, offset, data, 16);
    return *(m128 *)data;
}

static rose_inline
m256 getData256(const struct core_info *ci, s64a offset, u32 *valid_data_mask) {
    if (offset > 0 && offset + sizeof(m256) <= ci->len) {
        *valid_data_mask = ~0u;
        return loadu256(ci->buf + offset);
    }
    ALIGN_AVX_DIRECTIVE u8 data[sizeof(m256)];
    *valid_data_mask = getBufferDataComplex(ci, offset, data, 32);
    return *(m256 *)data;
}

static rose_inline
int roseCheckShufti16x8(const struct core_info *ci, const u8 *nib_mask,
                        const u8 *bucket_select_mask, u32 neg_mask,
                        s32 checkOffset, u64a end) {
    const s64a base_offset = (s64a)end - ci->buf_offset;
    s64a offset = base_offset + checkOffset;
    DEBUG_PRINTF("end %lld base_offset %lld\n", end, base_offset);
    DEBUG_PRINTF("checkOffset %d offset %lld\n", checkOffset, offset);

    if (unlikely(checkOffset < 0 && (u64a)(0 - checkOffset) > end)) {
        DEBUG_PRINTF("too early, fail\n");
        return 0;
    }

    u32 valid_data_mask = 0;
    m128 data = getData128(ci, offset, &valid_data_mask);
    if (unlikely(!valid_data_mask)) {
        return 1;
    }

    m256 nib_mask_m256 = loadu256(nib_mask);
    m128 bucket_select_mask_m128 = loadu128(bucket_select_mask);
    if (validateShuftiMask16x8(data, nib_mask_m256,
                               bucket_select_mask_m128,
                               neg_mask, valid_data_mask)) {
        DEBUG_PRINTF("check shufti 16x8 successfully\n");
        return 1;
    } else {
        return 0;
    }
}

static rose_inline
int roseCheckShufti16x16(const struct core_info *ci, const u8 *hi_mask,
                         const u8 *lo_mask, const u8 *bucket_select_mask,
                         u32 neg_mask, s32 checkOffset, u64a end) {
    const s64a base_offset = (s64a)end - ci->buf_offset;
    s64a offset = base_offset + checkOffset;
    DEBUG_PRINTF("end %lld base_offset %lld\n", end, base_offset);
    DEBUG_PRINTF("checkOffset %d offset %lld\n", checkOffset, offset);

    if (unlikely(checkOffset < 0 && (u64a)(0 - checkOffset) > end)) {
        DEBUG_PRINTF("too early, fail\n");
        return 0;
    }

    u32 valid_data_mask = 0;
    m128 data = getData128(ci, offset, &valid_data_mask);
    if (unlikely(!valid_data_mask)) {
        return 1;
    }

    m256 data_m256 = set2x128(data);
    m256 hi_mask_m256 = loadu256(hi_mask);
    m256 lo_mask_m256 = loadu256(lo_mask);
    m256 bucket_select_mask_m256 = loadu256(bucket_select_mask);
    if (validateShuftiMask16x16(data_m256, hi_mask_m256, lo_mask_m256,
                                bucket_select_mask_m256,
                                neg_mask, valid_data_mask)) {
        DEBUG_PRINTF("check shufti 16x16 successfully\n");
        return 1;
    } else {
        return 0;
    }
}

static rose_inline
int roseCheckShufti32x8(const struct core_info *ci, const u8 *hi_mask,
                        const u8 *lo_mask, const u8 *bucket_select_mask,
                        u32 neg_mask, s32 checkOffset, u64a end) {
    const s64a base_offset = (s64a)end - ci->buf_offset;
    s64a offset = base_offset + checkOffset;
    DEBUG_PRINTF("end %lld base_offset %lld\n", end, base_offset);
    DEBUG_PRINTF("checkOffset %d offset %lld\n", checkOffset, offset);

    if (unlikely(checkOffset < 0 && (u64a)(0 - checkOffset) > end)) {
        DEBUG_PRINTF("too early, fail\n");
        return 0;
    }

    u32 valid_data_mask = 0;
    m256 data = getData256(ci, offset, &valid_data_mask);
    if (unlikely(!valid_data_mask)) {
        return 1;
    }

    m128 hi_mask_m128 = loadu128(hi_mask);
    m128 lo_mask_m128 = loadu128(lo_mask);
    m256 hi_mask_m256 = set2x128(hi_mask_m128);
    m256 lo_mask_m256 = set2x128(lo_mask_m128);
    m256 bucket_select_mask_m256 = loadu256(bucket_select_mask);
    if (validateShuftiMask32x8(data, hi_mask_m256, lo_mask_m256,
                               bucket_select_mask_m256,
                               neg_mask, valid_data_mask)) {
        DEBUG_PRINTF("check shufti 32x8 successfully\n");
        return 1;
    } else {
        return 0;
    }
}

static rose_inline
int roseCheckShufti32x16(const struct core_info *ci, const u8 *hi_mask,
                         const u8 *lo_mask, const u8 *bucket_select_mask_hi,
                         const u8 *bucket_select_mask_lo, u32 neg_mask,
                         s32 checkOffset, u64a end) {
    const s64a base_offset = (s64a)end - ci->buf_offset;
    s64a offset = base_offset + checkOffset;
    DEBUG_PRINTF("end %lld base_offset %lld\n", end, base_offset);
    DEBUG_PRINTF("checkOffset %d offset %lld\n", checkOffset, offset);

    if (unlikely(checkOffset < 0 && (u64a)(0 - checkOffset) > end)) {
        DEBUG_PRINTF("too early, fail\n");
        return 0;
    }

    u32 valid_data_mask = 0;
    m256 data = getData256(ci, offset, &valid_data_mask);
    if (unlikely(!valid_data_mask)) {
        return 1;
    }

    m256 hi_mask_1 = loadu2x128(hi_mask);
    m256 hi_mask_2 = loadu2x128(hi_mask + 16);
    m256 lo_mask_1 = loadu2x128(lo_mask);
    m256 lo_mask_2 = loadu2x128(lo_mask + 16);

    m256 bucket_mask_hi = loadu256(bucket_select_mask_hi);
    m256 bucket_mask_lo = loadu256(bucket_select_mask_lo);
    if (validateShuftiMask32x16(data, hi_mask_1, hi_mask_2,
                                lo_mask_1, lo_mask_2, bucket_mask_hi,
                                bucket_mask_lo, neg_mask, valid_data_mask)) {
        DEBUG_PRINTF("check shufti 32x16 successfully\n");
        return 1;
    } else {
        return 0;
    }
}

static rose_inline
int roseCheckSingleLookaround(const struct RoseEngine *t,
                              const struct hs_scratch *scratch,
                              s8 checkOffset, u32 lookaroundReachIndex,
                              u64a end) {
    assert(lookaroundReachIndex != MO_INVALID_IDX);
    const struct core_info *ci = &scratch->core_info;
    DEBUG_PRINTF("end=%llu, buf_offset=%llu, buf_end=%llu\n", end,
                 ci->buf_offset, ci->buf_offset + ci->len);

    const s64a base_offset = end - ci->buf_offset;
    const s64a offset = base_offset + checkOffset;
    DEBUG_PRINTF("base_offset=%lld\n", base_offset);
    DEBUG_PRINTF("checkOffset=%d offset=%lld\n", checkOffset, offset);

    if (unlikely(checkOffset < 0 && (u64a)(0 - checkOffset) > end)) {
        DEBUG_PRINTF("too early, fail\n");
        return 0;
    }

    const u8 *reach = getByOffset(t, lookaroundReachIndex);

    u8 c;
    if (offset >= 0 && offset < (s64a)ci->len) {
        c = ci->buf[offset];
    } else if (offset < 0 && offset >= -(s64a)ci->hlen) {
        c = ci->hbuf[ci->hlen + offset];
    } else {
        return 1;
    }

    if (!reachHasBit(reach, c)) {
        DEBUG_PRINTF("char 0x%02x failed reach check\n", c);
        return 0;
    }

    DEBUG_PRINTF("OK :)\n");
    return 1;
}

/**
 * \brief Scan around a literal, checking that that "lookaround" reach masks
 * are satisfied.
 */
static rose_inline
int roseCheckLookaround(const struct RoseEngine *t,
                        const struct hs_scratch *scratch,
                        u32 lookaroundLookIndex, u32 lookaroundReachIndex,
                        u32 lookaroundCount, u64a end) {
    assert(lookaroundLookIndex != MO_INVALID_IDX);
    assert(lookaroundReachIndex != MO_INVALID_IDX);
    assert(lookaroundCount > 0);

    const struct core_info *ci = &scratch->core_info;
    DEBUG_PRINTF("end=%llu, buf_offset=%llu, buf_end=%llu\n", end,
                 ci->buf_offset, ci->buf_offset + ci->len);

    const s8 *look = getByOffset(t, lookaroundLookIndex);
    const s8 *look_end = look + lookaroundCount;
    assert(look < look_end);

    const u8 *reach = getByOffset(t, lookaroundReachIndex);

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

/**
 * \brief Trying to find a matching path by the corresponding path mask of
 * every lookaround location.
 */
static rose_inline
int roseMultipathLookaround(const struct RoseEngine *t,
                            const struct hs_scratch *scratch,
                            u32 multipathLookaroundLookIndex,
                            u32 multipathLookaroundReachIndex,
                            u32 multipathLookaroundCount,
                            s32 last_start, const u8 *start_mask,
                            u64a end) {
    assert(multipathLookaroundCount > 0);

    const struct core_info *ci = &scratch->core_info;
    DEBUG_PRINTF("end=%llu, buf_offset=%llu, buf_end=%llu\n", end,
                 ci->buf_offset, ci->buf_offset + ci->len);

    const s8 *look = getByOffset(t, multipathLookaroundLookIndex);
    const s8 *look_end = look + multipathLookaroundCount;
    assert(look < look_end);

    const u8 *reach = getByOffset(t, multipathLookaroundReachIndex);

    const s64a base_offset = (s64a)end - ci->buf_offset;
    DEBUG_PRINTF("base_offset=%lld\n", base_offset);

    u8 path = 0xff;

    assert(last_start < 0);

    if (unlikely((u64a)(0 - last_start) > end)) {
        DEBUG_PRINTF("too early, fail\n");
        return 0;
    }

    s8 base_look_offset = *look;
    do {
        s64a offset = base_offset + *look;
        u32 start_offset = (u32)(*look - base_look_offset);
        DEBUG_PRINTF("start_mask[%u] = %x\n", start_offset,
                     start_mask[start_offset]);
        path = start_mask[start_offset];
        if (offset >= -(s64a)ci->hlen) {
            break;
        }
        DEBUG_PRINTF("look=%d before history\n", *look);
        look++;
        reach += MULTI_REACH_BITVECTOR_LEN;
    } while (look < look_end);

    DEBUG_PRINTF("scan history (%zu looks left)\n", look_end - look);
    for (; look < look_end; ++look, reach += MULTI_REACH_BITVECTOR_LEN) {
        s64a offset = base_offset + *look;
        DEBUG_PRINTF("reach=%p, rel offset=%lld\n", reach, offset);

        if (offset >= 0) {
            DEBUG_PRINTF("in buffer\n");
            break;
        }

        assert(offset >= -(s64a)ci->hlen && offset < 0);
        u8 c = ci->hbuf[ci->hlen + offset];
        path &= reach[c];
        DEBUG_PRINTF("reach[%x] = %02x path = %0xx\n", c, reach[c],  path);
        if (!path) {
            DEBUG_PRINTF("char 0x%02x failed reach check\n", c);
            return 0;
        }
    }

    DEBUG_PRINTF("scan buffer (%zu looks left)\n", look_end - look);
    for(; look < look_end; ++look, reach += MULTI_REACH_BITVECTOR_LEN) {
        s64a offset = base_offset + *look;
        DEBUG_PRINTF("reach=%p, rel offset=%lld\n", reach, offset);

        if (offset >= (s64a)ci->len) {
            DEBUG_PRINTF("in the future\n");
            break;
        }

        assert(offset >= 0 && offset < (s64a)ci->len);
        u8 c = ci->buf[offset];
        path &= reach[c];
        DEBUG_PRINTF("reach[%x] = %02x path = %0xx\n", c, reach[c],  path);
        if (!path) {
            DEBUG_PRINTF("char 0x%02x failed reach check\n", c);
            return 0;
        }
    }

    DEBUG_PRINTF("OK :)\n");
    return 1;
}

static never_inline
int roseCheckMultipathShufti16x8(const struct hs_scratch *scratch,
                       const struct ROSE_STRUCT_CHECK_MULTIPATH_SHUFTI_16x8 *ri,
                                 u64a end) {
    const struct core_info *ci = &scratch->core_info;
    s32 checkOffset = ri->base_offset;
    const s64a base_offset = (s64a)end - ci->buf_offset;
    s64a offset = base_offset + checkOffset;
    DEBUG_PRINTF("end %lld base_offset %lld\n", end, base_offset);
    DEBUG_PRINTF("checkOffset %d offset %lld\n", checkOffset, offset);

    assert(ri->last_start <= 0);
    if (unlikely(checkOffset < 0 && (u64a)(0 - checkOffset) > end)) {
        if ((u64a)(0 - ri->last_start) > end) {
            DEBUG_PRINTF("too early, fail\n");
            return 0;
        }
    }

    u32 valid_data_mask;
    m128 data_init = getData128(ci, offset, &valid_data_mask);
    m128 data_select_mask = loadu128(ri->data_select_mask);

    u32 valid_path_mask = 0;
    if (unlikely(!(valid_data_mask & 1))) {
        DEBUG_PRINTF("lose part of backward data\n");
        DEBUG_PRINTF("valid_data_mask %x\n", valid_data_mask);

        m128 expand_valid;
        u64a expand_mask = 0x8080808080808080ULL;
        u64a valid_lo = expand64(valid_data_mask & 0xff, expand_mask);
        u64a valid_hi = expand64(valid_data_mask >> 8, expand_mask);
        DEBUG_PRINTF("expand_hi %llx\n", valid_hi);
        DEBUG_PRINTF("expand_lo %llx\n", valid_lo);
        expand_valid = set64x2(valid_hi, valid_lo);
        valid_path_mask = ~movemask128(pshufb_m128(expand_valid,
                                               data_select_mask));
    }

    m128 data = pshufb_m128(data_init, data_select_mask);
    m256 nib_mask = loadu256(ri->nib_mask);
    m128 bucket_select_mask = loadu128(ri->bucket_select_mask);

    u32 hi_bits_mask = ri->hi_bits_mask;
    u32 lo_bits_mask = ri->lo_bits_mask;
    u32 neg_mask = ri->neg_mask;

    if (validateMultipathShuftiMask16x8(data, nib_mask,
                                        bucket_select_mask,
                                        hi_bits_mask, lo_bits_mask,
                                        neg_mask, valid_path_mask)) {
        DEBUG_PRINTF("check multi-path shufti-16x8 successfully\n");
        return 1;
    } else {
        return 0;
    }
}

static never_inline
int roseCheckMultipathShufti32x8(const struct hs_scratch *scratch,
                       const struct ROSE_STRUCT_CHECK_MULTIPATH_SHUFTI_32x8 *ri,
                                 u64a end) {
    const struct core_info *ci = &scratch->core_info;
    s32 checkOffset = ri->base_offset;
    const s64a base_offset = (s64a)end - ci->buf_offset;
    s64a offset = base_offset + checkOffset;
    DEBUG_PRINTF("end %lld base_offset %lld\n", end, base_offset);
    DEBUG_PRINTF("checkOffset %d offset %lld\n", checkOffset, offset);

    assert(ri->last_start <= 0);
    if (unlikely(checkOffset < 0 && (u64a)(0 - checkOffset) > end)) {
        if ((u64a)(0 - ri->last_start) > end) {
            DEBUG_PRINTF("too early, fail\n");
            return 0;
        }
    }

    u32 valid_data_mask;
    m128 data_m128 = getData128(ci, offset, &valid_data_mask);
    m256 data_double = set2x128(data_m128);
    m256 data_select_mask = loadu256(ri->data_select_mask);

    u32 valid_path_mask = 0;
    m256 expand_valid;
    if (unlikely(!(valid_data_mask & 1))) {
        DEBUG_PRINTF("lose part of backward data\n");
        DEBUG_PRINTF("valid_data_mask %x\n", valid_data_mask);

        u64a expand_mask = 0x8080808080808080ULL;
        u64a valid_lo = expand64(valid_data_mask & 0xff, expand_mask);
        u64a valid_hi = expand64(valid_data_mask >> 8, expand_mask);
        DEBUG_PRINTF("expand_hi %llx\n", valid_hi);
        DEBUG_PRINTF("expand_lo %llx\n", valid_lo);
        expand_valid = set64x4(valid_hi, valid_lo, valid_hi,
                                         valid_lo);
        valid_path_mask = ~movemask256(pshufb_m256(expand_valid,
                                                  data_select_mask));
    }

    m256 data = pshufb_m256(data_double, data_select_mask);
    m256 hi_mask = loadu2x128(ri->hi_mask);
    m256 lo_mask = loadu2x128(ri->lo_mask);
    m256 bucket_select_mask = loadu256(ri->bucket_select_mask);

    u32 hi_bits_mask = ri->hi_bits_mask;
    u32 lo_bits_mask = ri->lo_bits_mask;
    u32 neg_mask = ri->neg_mask;

    if (validateMultipathShuftiMask32x8(data, hi_mask, lo_mask,
                                        bucket_select_mask,
                                        hi_bits_mask, lo_bits_mask,
                                        neg_mask, valid_path_mask)) {
        DEBUG_PRINTF("check multi-path shufti-32x8 successfully\n");
        return 1;
    } else {
        return 0;
    }
}

static never_inline
int roseCheckMultipathShufti32x16(const struct hs_scratch *scratch,
                      const struct ROSE_STRUCT_CHECK_MULTIPATH_SHUFTI_32x16 *ri,
                                  u64a end) {
    const struct core_info *ci = &scratch->core_info;
    const s64a base_offset = (s64a)end - ci->buf_offset;
    s32 checkOffset = ri->base_offset;
    s64a offset = base_offset + checkOffset;
    DEBUG_PRINTF("end %lld base_offset %lld\n", end, base_offset);
    DEBUG_PRINTF("checkOffset %d offset %lld\n", checkOffset, offset);

    assert(ri->last_start <= 0);
    if (unlikely(checkOffset < 0 && (u64a)(0 - checkOffset) > end)) {
        if ((u64a)(0 - ri->last_start) > end) {
            DEBUG_PRINTF("too early, fail\n");
            return 0;
        }
    }

    u32 valid_data_mask;
    m128 data_m128 = getData128(ci, offset, &valid_data_mask);
    m256 data_double = set2x128(data_m128);
    m256 data_select_mask = loadu256(ri->data_select_mask);

    u32 valid_path_mask = 0;
    m256 expand_valid;
    if (unlikely(!(valid_data_mask & 1))) {
        DEBUG_PRINTF("lose part of backward data\n");
        DEBUG_PRINTF("valid_data_mask %x\n", valid_data_mask);

        u64a expand_mask = 0x8080808080808080ULL;
        u64a valid_lo = expand64(valid_data_mask & 0xff, expand_mask);
        u64a valid_hi = expand64(valid_data_mask >> 8, expand_mask);
        DEBUG_PRINTF("expand_hi %llx\n", valid_hi);
        DEBUG_PRINTF("expand_lo %llx\n", valid_lo);
        expand_valid = set64x4(valid_hi, valid_lo, valid_hi,
                                         valid_lo);
        valid_path_mask = ~movemask256(pshufb_m256(expand_valid,
                                                   data_select_mask));
    }

    m256 data = pshufb_m256(data_double, data_select_mask);

    m256 hi_mask_1 = loadu2x128(ri->hi_mask);
    m256 hi_mask_2 = loadu2x128(ri->hi_mask + 16);
    m256 lo_mask_1 = loadu2x128(ri->lo_mask);
    m256 lo_mask_2 = loadu2x128(ri->lo_mask + 16);

    m256 bucket_select_mask_hi = loadu256(ri->bucket_select_mask_hi);
    m256 bucket_select_mask_lo = loadu256(ri->bucket_select_mask_lo);

    u32 hi_bits_mask = ri->hi_bits_mask;
    u32 lo_bits_mask = ri->lo_bits_mask;
    u32 neg_mask = ri->neg_mask;

    if (validateMultipathShuftiMask32x16(data, hi_mask_1, hi_mask_2,
                                         lo_mask_1, lo_mask_2,
                                         bucket_select_mask_hi,
                                         bucket_select_mask_lo,
                                         hi_bits_mask, lo_bits_mask,
                                         neg_mask, valid_path_mask)) {
        DEBUG_PRINTF("check multi-path shufti-32x16 successfully\n");
        return 1;
    } else {
        return 0;
    }
}

static never_inline
int roseCheckMultipathShufti64(const struct hs_scratch *scratch,
                         const struct ROSE_STRUCT_CHECK_MULTIPATH_SHUFTI_64 *ri,
                               u64a end) {
    const struct core_info *ci = &scratch->core_info;
    const s64a base_offset = (s64a)end - ci->buf_offset;
    s32 checkOffset = ri->base_offset;
    s64a offset = base_offset + checkOffset;
    DEBUG_PRINTF("end %lld base_offset %lld\n", end, base_offset);
    DEBUG_PRINTF("checkOffset %d offset %lld\n", checkOffset, offset);

    if (unlikely(checkOffset < 0 && (u64a)(0 - checkOffset) > end)) {
        if ((u64a)(0 - ri->last_start) > end) {
            DEBUG_PRINTF("too early, fail\n");
            return 0;
        }
    }

    u32 valid_data_mask;
    m128 data_m128 = getData128(ci, offset, &valid_data_mask);
    m256 data_m256 = set2x128(data_m128);
    m256 data_select_mask_1 = loadu256(ri->data_select_mask);
    m256 data_select_mask_2 = loadu256(ri->data_select_mask + 32);

    u64a valid_path_mask = 0;
    m256 expand_valid;
    if (unlikely(!(valid_data_mask & 1))) {
        DEBUG_PRINTF("lose part of backward data\n");
        DEBUG_PRINTF("valid_data_mask %x\n", valid_data_mask);

        u64a expand_mask = 0x8080808080808080ULL;
        u64a valid_lo = expand64(valid_data_mask & 0xff, expand_mask);
        u64a valid_hi = expand64(valid_data_mask >> 8, expand_mask);
        DEBUG_PRINTF("expand_hi %llx\n", valid_hi);
        DEBUG_PRINTF("expand_lo %llx\n", valid_lo);
        expand_valid = set64x4(valid_hi, valid_lo, valid_hi,
                                         valid_lo);
        u32 valid_path_1 = movemask256(pshufb_m256(expand_valid,
                                                   data_select_mask_1));
        u32 valid_path_2 = movemask256(pshufb_m256(expand_valid,
                                                   data_select_mask_2));
        valid_path_mask = ~((u64a)valid_path_1 | (u64a)valid_path_2 << 32);
    }

    m256 data_1 = pshufb_m256(data_m256, data_select_mask_1);
    m256 data_2 = pshufb_m256(data_m256, data_select_mask_2);

    m256 hi_mask = loadu2x128(ri->hi_mask);
    m256 lo_mask = loadu2x128(ri->lo_mask);

    m256 bucket_select_mask_1 = loadu256(ri->bucket_select_mask);
    m256 bucket_select_mask_2 = loadu256(ri->bucket_select_mask + 32);

    u64a hi_bits_mask = ri->hi_bits_mask;
    u64a lo_bits_mask = ri->lo_bits_mask;
    u64a neg_mask = ri->neg_mask;

    if (validateMultipathShuftiMask64(data_1, data_2, hi_mask, lo_mask,
                                      bucket_select_mask_1,
                                      bucket_select_mask_2, hi_bits_mask,
                                      lo_bits_mask, neg_mask,
                                      valid_path_mask)) {
        DEBUG_PRINTF("check multi-path shufti-64 successfully\n");
        return 1;
    } else {
        return 0;
    }
}

static rose_inline
int roseNfaEarliestSom(u64a start, UNUSED u64a end, UNUSED ReportID id,
                       void *context) {
    assert(context);
    u64a *som = context;
    *som = MIN(*som, start);
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
    q->cb = roseNfaEarliestSom;
    q->context = &start;

    nfaReportCurrentMatches(q->nfa, q);

    /* restore the old callback + context */
    q->cb = roseNfaAdaptor;
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

static rose_inline
hwlmcb_rv_t roseEnginesEod(const struct RoseEngine *rose,
                           struct hs_scratch *scratch, u64a offset,
                           u32 iter_offset) {
    const char is_streaming = rose->mode != HS_MODE_BLOCK;

    /* data, len is used for state decompress, should be full available data */
    u8 key = 0;
    if (is_streaming) {
        const u8 *eod_data = scratch->core_info.hbuf;
        size_t eod_len = scratch->core_info.hlen;
        key = eod_len ? eod_data[eod_len - 1] : 0;
    }

    const u8 *aa = getActiveLeafArray(rose, scratch->core_info.state);
    const u32 aaCount = rose->activeArrayCount;
    const u32 qCount = rose->queueCount;
    struct fatbit *aqa = scratch->aqa;

    const struct mmbit_sparse_iter *it = getByOffset(rose, iter_offset);
    assert(ISALIGNED(it));

    u32 idx = 0;
    struct mmbit_sparse_state si_state[MAX_SPARSE_ITER_STATES];

    for (u32 qi = mmbit_sparse_iter_begin(aa, aaCount, &idx, it, si_state);
         qi != MMB_INVALID;
         qi = mmbit_sparse_iter_next(aa, aaCount, qi, &idx, it, si_state)) {
        DEBUG_PRINTF("checking nfa %u\n", qi);
        struct mq *q = scratch->queues + qi;
        if (!fatbit_set(aqa, qCount, qi)) {
            initQueue(q, qi, rose, scratch);
        }

        assert(q->nfa == getNfaByQueue(rose, qi));
        assert(nfaAcceptsEod(q->nfa));

        if (is_streaming) {
            // Decompress stream state.
            nfaExpandState(q->nfa, q->state, q->streamState, offset, key);
        }

        if (nfaCheckFinalState(q->nfa, q->state, q->streamState, offset,
                               roseReportAdaptor,
                               scratch) == MO_HALT_MATCHING) {
            DEBUG_PRINTF("user instructed us to stop\n");
            return HWLM_TERMINATE_MATCHING;
        }
    }

    return HWLM_CONTINUE_MATCHING;
}

static rose_inline
hwlmcb_rv_t roseSuffixesEod(const struct RoseEngine *rose,
                            struct hs_scratch *scratch, u64a offset) {
    const u8 *aa = getActiveLeafArray(rose, scratch->core_info.state);
    const u32 aaCount = rose->activeArrayCount;

    for (u32 qi = mmbit_iterate(aa, aaCount, MMB_INVALID); qi != MMB_INVALID;
         qi = mmbit_iterate(aa, aaCount, qi)) {
        DEBUG_PRINTF("checking nfa %u\n", qi);
        struct mq *q = scratch->queues + qi;
        assert(q->nfa == getNfaByQueue(rose, qi));
        assert(nfaAcceptsEod(q->nfa));

        /* We have just been triggered. */
        assert(fatbit_isset(scratch->aqa, rose->queueCount, qi));

        pushQueueNoMerge(q, MQE_END, scratch->core_info.len);
        q->context = NULL;

        /* rose exec is used as we don't want to / can't raise matches in the
         * history buffer. */
        if (!nfaQueueExecRose(q->nfa, q, MO_INVALID_IDX)) {
            DEBUG_PRINTF("nfa is dead\n");
            continue;
        }
        if (nfaCheckFinalState(q->nfa, q->state, q->streamState, offset,
                               roseReportAdaptor,
                               scratch) == MO_HALT_MATCHING) {
            DEBUG_PRINTF("user instructed us to stop\n");
            return HWLM_TERMINATE_MATCHING;
        }
    }
    return HWLM_CONTINUE_MATCHING;
}

static rose_inline
hwlmcb_rv_t roseMatcherEod(const struct RoseEngine *rose,
                           struct hs_scratch *scratch, u64a offset) {
    assert(rose->ematcherOffset);
    assert(rose->ematcherRegionSize);

    // Clear role state and active engines, since we have already handled all
    // outstanding work there.
    DEBUG_PRINTF("clear role state and active leaf array\n");
    char *state = scratch->core_info.state;
    mmbit_clear(getRoleState(state), rose->rolesWithStateCount);
    mmbit_clear(getActiveLeafArray(rose, state), rose->activeArrayCount);

    const char is_streaming = rose->mode != HS_MODE_BLOCK;

    size_t eod_len;
    const u8 *eod_data;
    if (!is_streaming) { /* Block */
        eod_data = scratch->core_info.buf;
        eod_len = scratch->core_info.len;
    } else { /* Streaming */
        eod_len = scratch->core_info.hlen;
        eod_data = scratch->core_info.hbuf;
    }

    assert(eod_data);
    assert(eod_len);

    DEBUG_PRINTF("%zu bytes of eod data to scan at offset %llu\n", eod_len,
                 offset);

    // If we don't have enough bytes to produce a match from an EOD table scan,
    // there's no point scanning.
    if (eod_len < rose->eodmatcherMinWidth) {
        DEBUG_PRINTF("too short for min width %u\n", rose->eodmatcherMinWidth);
        return HWLM_CONTINUE_MATCHING;
    }

    // Ensure that we only need scan the last N bytes, where N is the length of
    // the eod-anchored matcher region.
    size_t adj = eod_len - MIN(eod_len, rose->ematcherRegionSize);

    const struct HWLM *etable = getByOffset(rose, rose->ematcherOffset);
    hwlmExec(etable, eod_data, eod_len, adj, roseCallback, scratch,
             scratch->tctxt.groups);

    // We may need to fire delayed matches.
    if (cleanUpDelayed(rose, scratch, 0, offset) == HWLM_TERMINATE_MATCHING) {
        DEBUG_PRINTF("user instructed us to stop\n");
        return HWLM_TERMINATE_MATCHING;
    }

    roseFlushLastByteHistory(rose, scratch, offset);
    return HWLM_CONTINUE_MATCHING;
}

static rose_inline
int roseCheckLongLiteral(const struct RoseEngine *t,
                         const struct hs_scratch *scratch, u64a end,
                         u32 lit_offset, u32 lit_length, char nocase) {
    const struct core_info *ci = &scratch->core_info;
    const u8 *lit = getByOffset(t, lit_offset);

    DEBUG_PRINTF("check lit at %llu, length %u\n", end, lit_length);
    DEBUG_PRINTF("base buf_offset=%llu\n", ci->buf_offset);

    if (end < lit_length) {
        DEBUG_PRINTF("too short!\n");
        return 0;
    }

    // If any portion of the literal matched in the current buffer, check it.
    if (end > ci->buf_offset) {
        u32 scan_len = MIN(end - ci->buf_offset, lit_length);
        u64a scan_start = end - ci->buf_offset - scan_len;
        DEBUG_PRINTF("checking suffix (%u bytes) in buf[%llu:%llu]\n", scan_len,
                     scan_start, end);
        if (cmpForward(ci->buf + scan_start, lit + lit_length - scan_len,
                       scan_len, nocase)) {
            DEBUG_PRINTF("cmp of suffix failed\n");
            return 0;
        }
    }

    // If the entirety of the literal was in the current block, we are done.
    if (end - lit_length >= ci->buf_offset) {
        DEBUG_PRINTF("literal confirmed in current block\n");
        return 1;
    }

    // We still have a prefix which we must test against the buffer prepared by
    // the long literal table. This is only done in streaming mode.

    assert(t->mode != HS_MODE_BLOCK);

    const u8 *ll_buf;
    size_t ll_len;
    if (nocase) {
        ll_buf = scratch->tctxt.ll_buf_nocase;
        ll_len = scratch->tctxt.ll_len_nocase;
    } else {
        ll_buf = scratch->tctxt.ll_buf;
        ll_len = scratch->tctxt.ll_len;
    }

    assert(ll_buf);

    u64a lit_start_offset = end - lit_length;
    u32 prefix_len = MIN(lit_length, ci->buf_offset - lit_start_offset);
    u32 hist_rewind = ci->buf_offset - lit_start_offset;
    DEBUG_PRINTF("ll_len=%zu, hist_rewind=%u\n", ll_len, hist_rewind);
    if (hist_rewind > ll_len) {
        DEBUG_PRINTF("not enough history\n");
        return 0;
    }

    DEBUG_PRINTF("check prefix len=%u from hist (len %zu, rewind %u)\n",
                 prefix_len, ll_len, hist_rewind);
    assert(hist_rewind <= ll_len);
    if (cmpForward(ll_buf + ll_len - hist_rewind, lit, prefix_len, nocase)) {
        DEBUG_PRINTF("cmp of prefix failed\n");
        return 0;
    }

    DEBUG_PRINTF("cmp succeeded\n");
    return 1;
}

static rose_inline
int roseCheckMediumLiteral(const struct RoseEngine *t,
                           const struct hs_scratch *scratch, u64a end,
                           u32 lit_offset, u32 lit_length, char nocase) {
    const struct core_info *ci = &scratch->core_info;
    const u8 *lit = getByOffset(t, lit_offset);

    DEBUG_PRINTF("check lit at %llu, length %u\n", end, lit_length);
    DEBUG_PRINTF("base buf_offset=%llu\n", ci->buf_offset);

    if (end < lit_length) {
        DEBUG_PRINTF("too short!\n");
        return 0;
    }

    // If any portion of the literal matched in the current buffer, check it.
    if (end > ci->buf_offset) {
        u32 scan_len = MIN(end - ci->buf_offset, lit_length);
        u64a scan_start = end - ci->buf_offset - scan_len;
        DEBUG_PRINTF("checking suffix (%u bytes) in buf[%llu:%llu]\n", scan_len,
                     scan_start, end);
        if (cmpForward(ci->buf + scan_start, lit + lit_length - scan_len,
                       scan_len, nocase)) {
            DEBUG_PRINTF("cmp of suffix failed\n");
            return 0;
        }
    }

    // If the entirety of the literal was in the current block, we are done.
    if (end - lit_length >= ci->buf_offset) {
        DEBUG_PRINTF("literal confirmed in current block\n");
        return 1;
    }

    // We still have a prefix which we must test against the history buffer.
    assert(t->mode != HS_MODE_BLOCK);

    u64a lit_start_offset = end - lit_length;
    u32 prefix_len = MIN(lit_length, ci->buf_offset - lit_start_offset);
    u32 hist_rewind = ci->buf_offset - lit_start_offset;
    DEBUG_PRINTF("hlen=%zu, hist_rewind=%u\n", ci->hlen, hist_rewind);

    // History length check required for confirm in the EOD and delayed
    // rebuild paths.
    if (hist_rewind > ci->hlen) {
        DEBUG_PRINTF("not enough history\n");
        return 0;
    }

    DEBUG_PRINTF("check prefix len=%u from hist (len %zu, rewind %u)\n",
                 prefix_len, ci->hlen, hist_rewind);
    assert(hist_rewind <= ci->hlen);
    if (cmpForward(ci->hbuf + ci->hlen - hist_rewind, lit, prefix_len,
                   nocase)) {
        DEBUG_PRINTF("cmp of prefix failed\n");
        return 0;
    }

    DEBUG_PRINTF("cmp succeeded\n");
    return 1;
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

static rose_inline
hwlmcb_rv_t flushActiveCombinations(const struct RoseEngine *t,
                                    struct hs_scratch *scratch) {
    u8 *cvec = (u8 *)scratch->core_info.combVector;
    if (!mmbit_any(cvec, t->ckeyCount)) {
        return HWLM_CONTINUE_MATCHING;
    }
    u64a end = scratch->tctxt.lastCombMatchOffset;
    for (u32 i = mmbit_iterate(cvec, t->ckeyCount, MMB_INVALID);
         i != MMB_INVALID; i = mmbit_iterate(cvec, t->ckeyCount, i)) {
        const struct CombInfo *combInfoMap = (const struct CombInfo *)
            ((const char *)t + t->combInfoMapOffset);
        const struct CombInfo *ci = combInfoMap + i;
        if ((ci->min_offset != 0) && (end < ci->min_offset)) {
            DEBUG_PRINTF("halt: before min_offset=%llu\n", ci->min_offset);
            continue;
        }
        if ((ci->max_offset != MAX_OFFSET) && (end > ci->max_offset)) {
            DEBUG_PRINTF("halt: after max_offset=%llu\n", ci->max_offset);
            continue;
        }

        DEBUG_PRINTF("check ekey %u\n", ci->ekey);
        if (ci->ekey != INVALID_EKEY) {
            assert(ci->ekey < t->ekeyCount);
            const char *evec = scratch->core_info.exhaustionVector;
            if (isExhausted(t, evec, ci->ekey)) {
                DEBUG_PRINTF("ekey %u already set, match is exhausted\n",
                             ci->ekey);
                continue;
            }
        }

        DEBUG_PRINTF("check ckey %u\n", i);
        char *lvec = scratch->core_info.logicalVector;
        if (!isLogicalCombination(t, lvec, ci->start, ci->result)) {
            DEBUG_PRINTF("Logical Combination Failed!\n");
            continue;
        }

        DEBUG_PRINTF("Logical Combination Passed!\n");
        if (roseReport(t, scratch, end, ci->id, 0,
                       ci->ekey) == HWLM_TERMINATE_MATCHING) {
            return HWLM_TERMINATE_MATCHING;
        }
    }
    clearCvec(t, (char *)cvec);
    return HWLM_CONTINUE_MATCHING;
}

#if !defined(_WIN32)
#define PROGRAM_CASE(name)                                                     \
    case ROSE_INSTR_##name: {                                                  \
    LABEL_ROSE_INSTR_##name:                                                   \
        DEBUG_PRINTF("instruction: " #name " (pc=%u)\n",                       \
                     programOffset + (u32)(pc - pc_base));                     \
        const struct ROSE_STRUCT_##name *ri =                                  \
            (const struct ROSE_STRUCT_##name *)pc;

#define PROGRAM_NEXT_INSTRUCTION                                               \
    pc += ROUNDUP_N(sizeof(*ri), ROSE_INSTR_MIN_ALIGN);                        \
    goto *(next_instr[*(const u8 *)pc]);                                       \
    }

#define PROGRAM_NEXT_INSTRUCTION_JUMP                                          \
    goto *(next_instr[*(const u8 *)pc]);
#else
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

#define PROGRAM_NEXT_INSTRUCTION_JUMP continue;
#endif

hwlmcb_rv_t roseRunProgram(const struct RoseEngine *t,
                           struct hs_scratch *scratch, u32 programOffset,
                           u64a som, u64a end, u8 prog_flags) {
    DEBUG_PRINTF("program=%u, offsets [%llu,%llu], flags=%u\n", programOffset,
                 som, end, prog_flags);

    assert(programOffset != ROSE_INVALID_PROG_OFFSET);
    assert(programOffset >= sizeof(struct RoseEngine));
    assert(programOffset < t->size);

    const char in_anchored = prog_flags & ROSE_PROG_FLAG_IN_ANCHORED;
    const char in_catchup = prog_flags & ROSE_PROG_FLAG_IN_CATCHUP;
    const char from_mpv = prog_flags & ROSE_PROG_FLAG_FROM_MPV;
    const char skip_mpv_catchup = prog_flags & ROSE_PROG_FLAG_SKIP_MPV_CATCHUP;

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

#if !defined(_WIN32)
    static const void *next_instr[] = {
        &&LABEL_ROSE_INSTR_END,               //!< End of program.
        &&LABEL_ROSE_INSTR_ANCHORED_DELAY,    //!< Delay until after anchored matcher.
        &&LABEL_ROSE_INSTR_CHECK_LIT_EARLY,   //!< Skip matches before floating min offset.
        &&LABEL_ROSE_INSTR_CHECK_GROUPS,      //!< Check that literal groups are on.
        &&LABEL_ROSE_INSTR_CHECK_ONLY_EOD,    //!< Role matches only at EOD.
        &&LABEL_ROSE_INSTR_CHECK_BOUNDS,      //!< Bounds on distance from offset 0.
        &&LABEL_ROSE_INSTR_CHECK_NOT_HANDLED, //!< Test & set role in "handled".
        &&LABEL_ROSE_INSTR_CHECK_SINGLE_LOOKAROUND, //!< Single lookaround check.
        &&LABEL_ROSE_INSTR_CHECK_LOOKAROUND,  //!< Lookaround check.
        &&LABEL_ROSE_INSTR_CHECK_MASK,        //!< 8-bytes mask check.
        &&LABEL_ROSE_INSTR_CHECK_MASK_32,     //!< 32-bytes and/cmp/neg mask check.
        &&LABEL_ROSE_INSTR_CHECK_BYTE,        //!< Single Byte check.
        &&LABEL_ROSE_INSTR_CHECK_SHUFTI_16x8, //!< Check 16-byte data by 8-bucket shufti.
        &&LABEL_ROSE_INSTR_CHECK_SHUFTI_32x8, //!< Check 32-byte data by 8-bucket shufti.
        &&LABEL_ROSE_INSTR_CHECK_SHUFTI_16x16, //!< Check 16-byte data by 16-bucket shufti.
        &&LABEL_ROSE_INSTR_CHECK_SHUFTI_32x16, //!< Check 32-byte data by 16-bucket shufti.
        &&LABEL_ROSE_INSTR_CHECK_INFIX,       //!< Infix engine must be in accept state.
        &&LABEL_ROSE_INSTR_CHECK_PREFIX,      //!< Prefix engine must be in accept state.
        &&LABEL_ROSE_INSTR_PUSH_DELAYED,      //!< Push delayed literal matches.
        &&LABEL_ROSE_INSTR_DUMMY_NOP,         //!< NOP. Should not exist in build programs.
        &&LABEL_ROSE_INSTR_CATCH_UP,          //!< Catch up engines, anchored matches.
        &&LABEL_ROSE_INSTR_CATCH_UP_MPV,      //!< Catch up the MPV.
        &&LABEL_ROSE_INSTR_SOM_ADJUST,        //!< Set SOM from a distance to EOM.
        &&LABEL_ROSE_INSTR_SOM_LEFTFIX,       //!< Acquire SOM from a leftfix engine.
        &&LABEL_ROSE_INSTR_SOM_FROM_REPORT,   //!< Acquire SOM from a som_operation.
        &&LABEL_ROSE_INSTR_SOM_ZERO,          //!< Set SOM to zero.
        &&LABEL_ROSE_INSTR_TRIGGER_INFIX,     //!< Trigger an infix engine.
        &&LABEL_ROSE_INSTR_TRIGGER_SUFFIX,    //!< Trigger a suffix engine.
        &&LABEL_ROSE_INSTR_DEDUPE,            //!< Run deduplication for report.
        &&LABEL_ROSE_INSTR_DEDUPE_SOM,        //!< Run deduplication for SOM report.
        &&LABEL_ROSE_INSTR_REPORT_CHAIN,      //!< Fire a chained report (MPV).
        &&LABEL_ROSE_INSTR_REPORT_SOM_INT,    //!< Manipulate SOM only.
        &&LABEL_ROSE_INSTR_REPORT_SOM_AWARE,  //!< Manipulate SOM from SOM-aware source.
        &&LABEL_ROSE_INSTR_REPORT,
        &&LABEL_ROSE_INSTR_REPORT_EXHAUST,
        &&LABEL_ROSE_INSTR_REPORT_SOM,
        &&LABEL_ROSE_INSTR_REPORT_SOM_EXHAUST,
        &&LABEL_ROSE_INSTR_DEDUPE_AND_REPORT,
        &&LABEL_ROSE_INSTR_FINAL_REPORT,
        &&LABEL_ROSE_INSTR_CHECK_EXHAUSTED,   //!< Check if an ekey has already been set.
        &&LABEL_ROSE_INSTR_CHECK_MIN_LENGTH,  //!< Check (EOM - SOM) against min length.
        &&LABEL_ROSE_INSTR_SET_STATE,         //!< Switch a state index on.
        &&LABEL_ROSE_INSTR_SET_GROUPS,        //!< Set some literal group bits.
        &&LABEL_ROSE_INSTR_SQUASH_GROUPS,     //!< Conditionally turn off some groups.
        &&LABEL_ROSE_INSTR_CHECK_STATE,       //!< Test a single bit in the state multibit.
        &&LABEL_ROSE_INSTR_SPARSE_ITER_BEGIN, //!< Begin running a sparse iter over states.
        &&LABEL_ROSE_INSTR_SPARSE_ITER_NEXT,  //!< Continue running sparse iter over states.
        &&LABEL_ROSE_INSTR_SPARSE_ITER_ANY,   //!< Test for any bit in the sparse iterator.
        &&LABEL_ROSE_INSTR_ENGINES_EOD,
        &&LABEL_ROSE_INSTR_SUFFIXES_EOD,
        &&LABEL_ROSE_INSTR_MATCHER_EOD,
        &&LABEL_ROSE_INSTR_CHECK_LONG_LIT,
        &&LABEL_ROSE_INSTR_CHECK_LONG_LIT_NOCASE,
        &&LABEL_ROSE_INSTR_CHECK_MED_LIT,
        &&LABEL_ROSE_INSTR_CHECK_MED_LIT_NOCASE,
        &&LABEL_ROSE_INSTR_CLEAR_WORK_DONE,
        &&LABEL_ROSE_INSTR_MULTIPATH_LOOKAROUND,
        &&LABEL_ROSE_INSTR_CHECK_MULTIPATH_SHUFTI_16x8,
        &&LABEL_ROSE_INSTR_CHECK_MULTIPATH_SHUFTI_32x8,
        &&LABEL_ROSE_INSTR_CHECK_MULTIPATH_SHUFTI_32x16,
        &&LABEL_ROSE_INSTR_CHECK_MULTIPATH_SHUFTI_64,
        &&LABEL_ROSE_INSTR_INCLUDED_JUMP,
        &&LABEL_ROSE_INSTR_SET_LOGICAL,
        &&LABEL_ROSE_INSTR_SET_COMBINATION,
        &&LABEL_ROSE_INSTR_FLUSH_COMBINATION,
        &&LABEL_ROSE_INSTR_SET_EXHAUST
    };
#endif

    for (;;) {
        assert(ISALIGNED_N(pc, ROSE_INSTR_MIN_ALIGN));
        assert(pc >= pc_base);
        assert((size_t)(pc - pc_base) < t->size);
        const u8 code = *(const u8 *)pc;
        assert(code <= LAST_ROSE_INSTRUCTION);

        switch ((enum RoseInstructionCode)code) {
            PROGRAM_CASE(END) {
                DEBUG_PRINTF("finished\n");
                return HWLM_CONTINUE_MATCHING;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(ANCHORED_DELAY) {
                if (in_anchored && end > t->floatingMinLiteralMatchOffset) {
                    DEBUG_PRINTF("delay until playback\n");
                    tctxt->groups |= ri->groups;
                    work_done = 1;
                    recordAnchoredLiteralMatch(t, scratch, ri->anch_id, end);

                    assert(ri->done_jump); // must progress
                    pc += ri->done_jump;
                    PROGRAM_NEXT_INSTRUCTION_JUMP
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_LIT_EARLY) {
                if (end < ri->min_offset) {
                    DEBUG_PRINTF("halt: before min_offset=%u\n",
                                 ri->min_offset);
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    PROGRAM_NEXT_INSTRUCTION_JUMP
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
                    PROGRAM_NEXT_INSTRUCTION_JUMP
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_BOUNDS) {
                if (!roseCheckBounds(end, ri->min_bound, ri->max_bound)) {
                    DEBUG_PRINTF("failed bounds check\n");
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    PROGRAM_NEXT_INSTRUCTION_JUMP
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_NOT_HANDLED) {
                struct fatbit *handled = scratch->handled_roles;
                if (fatbit_set(handled, t->handledKeyCount, ri->key)) {
                    DEBUG_PRINTF("key %u already set\n", ri->key);
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    PROGRAM_NEXT_INSTRUCTION_JUMP
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_SINGLE_LOOKAROUND) {
                if (!roseCheckSingleLookaround(t, scratch, ri->offset,
                                               ri->reach_index, end)) {
                    DEBUG_PRINTF("failed lookaround check\n");
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    PROGRAM_NEXT_INSTRUCTION_JUMP
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_LOOKAROUND) {
                if (!roseCheckLookaround(t, scratch, ri->look_index,
                                         ri->reach_index, ri->count, end)) {
                    DEBUG_PRINTF("failed lookaround check\n");
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    PROGRAM_NEXT_INSTRUCTION_JUMP
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_MASK) {
                struct core_info *ci = &scratch->core_info;
                if (!roseCheckMask(ci, ri->and_mask, ri->cmp_mask,
                                   ri->neg_mask, ri->offset, end)) {
                    DEBUG_PRINTF("failed mask check\n");
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    PROGRAM_NEXT_INSTRUCTION_JUMP
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_MASK_32) {
                struct core_info *ci = &scratch->core_info;
                if (!roseCheckMask32(ci, ri->and_mask, ri->cmp_mask,
                                     ri->neg_mask, ri->offset, end)) {
                    assert(ri->fail_jump);
                    pc += ri->fail_jump;
                    PROGRAM_NEXT_INSTRUCTION_JUMP
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_BYTE) {
                const struct core_info *ci = &scratch->core_info;
                if (!roseCheckByte(ci, ri->and_mask, ri->cmp_mask,
                                   ri->negation, ri->offset, end)) {
                    DEBUG_PRINTF("failed byte check\n");
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    PROGRAM_NEXT_INSTRUCTION_JUMP
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_SHUFTI_16x8) {
                const struct core_info *ci = &scratch->core_info;
                if (!roseCheckShufti16x8(ci, ri->nib_mask,
                                         ri->bucket_select_mask,
                                         ri->neg_mask, ri->offset, end)) {
                    assert(ri->fail_jump);
                    pc += ri-> fail_jump;
                    PROGRAM_NEXT_INSTRUCTION_JUMP
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_SHUFTI_32x8) {
                const struct core_info *ci = &scratch->core_info;
                if (!roseCheckShufti32x8(ci, ri->hi_mask, ri->lo_mask,
                                         ri->bucket_select_mask,
                                         ri->neg_mask, ri->offset, end)) {
                    assert(ri->fail_jump);
                    pc += ri-> fail_jump;
                    PROGRAM_NEXT_INSTRUCTION_JUMP
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_SHUFTI_16x16) {
                const struct core_info *ci = &scratch->core_info;
                if (!roseCheckShufti16x16(ci, ri->hi_mask, ri->lo_mask,
                                          ri->bucket_select_mask,
                                          ri->neg_mask, ri->offset, end)) {
                    assert(ri->fail_jump);
                    pc += ri-> fail_jump;
                    PROGRAM_NEXT_INSTRUCTION_JUMP
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_SHUFTI_32x16) {
                const struct core_info *ci = &scratch->core_info;
                if (!roseCheckShufti32x16(ci, ri->hi_mask, ri->lo_mask,
                                          ri->bucket_select_mask_hi,
                                          ri->bucket_select_mask_lo,
                                          ri->neg_mask, ri->offset, end)) {
                    assert(ri->fail_jump);
                    pc += ri-> fail_jump;
                    PROGRAM_NEXT_INSTRUCTION_JUMP
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_INFIX) {
                if (!roseTestInfix(t, scratch, ri->queue, ri->lag, ri->report,
                                   end)) {
                    DEBUG_PRINTF("failed infix check\n");
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    PROGRAM_NEXT_INSTRUCTION_JUMP
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_PREFIX) {
                if (!roseTestPrefix(t, scratch, ri->queue, ri->lag, ri->report,
                                    end)) {
                    DEBUG_PRINTF("failed prefix check\n");
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    PROGRAM_NEXT_INSTRUCTION_JUMP
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(PUSH_DELAYED) {
                rosePushDelayedMatch(t, scratch, ri->delay, ri->index, end);
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(DUMMY_NOP) {
                assert(0);
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
                    PROGRAM_NEXT_INSTRUCTION_JUMP
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
                    PROGRAM_NEXT_INSTRUCTION_JUMP
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
                roseHandleSom(scratch, &ri->som, end);
                work_done = 1;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(REPORT_SOM_AWARE) {
                updateSeqPoint(tctxt, end, from_mpv);
                roseHandleSomSom(scratch, &ri->som, som, end);
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
                    PROGRAM_NEXT_INSTRUCTION_JUMP
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
                    PROGRAM_NEXT_INSTRUCTION_JUMP
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
                    PROGRAM_NEXT_INSTRUCTION_JUMP
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
                    PROGRAM_NEXT_INSTRUCTION_JUMP
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
                    PROGRAM_NEXT_INSTRUCTION_JUMP
                }

                fatbit_clear(scratch->handled_roles);

                const u32 *jumps = getByOffset(t, ri->jump_table);
                DEBUG_PRINTF("state %u (idx=%u) is on, jump to %u\n", i, idx,
                             jumps[idx]);
                pc = pc_base + jumps[idx];
                PROGRAM_NEXT_INSTRUCTION_JUMP
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
                    PROGRAM_NEXT_INSTRUCTION_JUMP
                }

                const u32 *jumps = getByOffset(t, ri->jump_table);
                DEBUG_PRINTF("state %u (idx=%u) is on, jump to %u\n", i, idx,
                             jumps[idx]);
                pc = pc_base + jumps[idx];
                PROGRAM_NEXT_INSTRUCTION_JUMP
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SPARSE_ITER_ANY) {
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
                    PROGRAM_NEXT_INSTRUCTION_JUMP
                }
                DEBUG_PRINTF("state %u (idx=%u) is on\n", i, idx);
                fatbit_clear(scratch->handled_roles);
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(ENGINES_EOD) {
                if (roseEnginesEod(t, scratch, end, ri->iter_offset) ==
                    HWLM_TERMINATE_MATCHING) {
                    return HWLM_TERMINATE_MATCHING;
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SUFFIXES_EOD) {
                if (roseSuffixesEod(t, scratch, end) ==
                    HWLM_TERMINATE_MATCHING) {
                    return HWLM_TERMINATE_MATCHING;
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(MATCHER_EOD) {
                if (roseMatcherEod(t, scratch, end) ==
                    HWLM_TERMINATE_MATCHING) {
                    return HWLM_TERMINATE_MATCHING;
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_LONG_LIT) {
                const char nocase = 0;
                if (!roseCheckLongLiteral(t, scratch, end, ri->lit_offset,
                                          ri->lit_length, nocase)) {
                    DEBUG_PRINTF("failed long lit check\n");
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    PROGRAM_NEXT_INSTRUCTION_JUMP
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_LONG_LIT_NOCASE) {
                const char nocase = 1;
                if (!roseCheckLongLiteral(t, scratch, end, ri->lit_offset,
                                          ri->lit_length, nocase)) {
                    DEBUG_PRINTF("failed nocase long lit check\n");
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    PROGRAM_NEXT_INSTRUCTION_JUMP
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_MED_LIT) {
                const char nocase = 0;
                if (!roseCheckMediumLiteral(t, scratch, end, ri->lit_offset,
                                            ri->lit_length, nocase)) {
                    DEBUG_PRINTF("failed lit check\n");
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    PROGRAM_NEXT_INSTRUCTION_JUMP
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_MED_LIT_NOCASE) {
                const char nocase = 1;
                if (!roseCheckMediumLiteral(t, scratch, end, ri->lit_offset,
                                            ri->lit_length, nocase)) {
                    DEBUG_PRINTF("failed long lit check\n");
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    PROGRAM_NEXT_INSTRUCTION_JUMP
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CLEAR_WORK_DONE) {
                DEBUG_PRINTF("clear work_done flag\n");
                work_done = 0;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(MULTIPATH_LOOKAROUND) {
                if (!roseMultipathLookaround(t, scratch, ri->look_index,
                                             ri->reach_index, ri->count,
                                             ri->last_start, ri->start_mask,
                                             end)) {
                    DEBUG_PRINTF("failed multi-path lookaround check\n");
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    PROGRAM_NEXT_INSTRUCTION_JUMP
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_MULTIPATH_SHUFTI_16x8) {
                if (!roseCheckMultipathShufti16x8(scratch, ri, end)) {
                    DEBUG_PRINTF("failed multi-path shufti 16x8 check\n");
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    PROGRAM_NEXT_INSTRUCTION_JUMP
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_MULTIPATH_SHUFTI_32x8) {
                if (!roseCheckMultipathShufti32x8(scratch, ri, end)) {
                    DEBUG_PRINTF("failed multi-path shufti 32x8 check\n");
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    PROGRAM_NEXT_INSTRUCTION_JUMP
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_MULTIPATH_SHUFTI_32x16) {
                if (!roseCheckMultipathShufti32x16(scratch, ri, end)) {
                    DEBUG_PRINTF("failed multi-path shufti 32x16 check\n");
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    PROGRAM_NEXT_INSTRUCTION_JUMP
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_MULTIPATH_SHUFTI_64) {
                if (!roseCheckMultipathShufti64(scratch, ri, end)) {
                    DEBUG_PRINTF("failed multi-path shufti 64 check\n");
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    PROGRAM_NEXT_INSTRUCTION_JUMP
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(INCLUDED_JUMP) {
                if (scratch->fdr_conf) {
                    // squash the bucket of included literal
                    u8 shift = scratch->fdr_conf_offset & ~7U;
                    u64a mask = ((~(u64a)ri->squash) << shift);
                    *(scratch->fdr_conf) &= mask;

                    pc = getByOffset(t, ri->child_offset);
                    pc_base = pc;
                    programOffset = (const u8 *)pc_base -(const u8 *)t;
                    DEBUG_PRINTF("pc_base %p pc %p child_offset %u squash %u\n",
                                 pc_base, pc, ri->child_offset, ri->squash);
                    work_done = 0;
                    PROGRAM_NEXT_INSTRUCTION_JUMP
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SET_LOGICAL) {
                DEBUG_PRINTF("set logical value of lkey %u, offset_adjust=%d\n",
                             ri->lkey, ri->offset_adjust);
                assert(ri->lkey != INVALID_LKEY);
                assert(ri->lkey < t->lkeyCount);
                char *lvec = scratch->core_info.logicalVector;
                setLogicalVal(t, lvec, ri->lkey, 1);
                updateLastCombMatchOffset(tctxt, end + ri->offset_adjust);
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SET_COMBINATION) {
                DEBUG_PRINTF("set ckey %u as active\n", ri->ckey);
                assert(ri->ckey != INVALID_CKEY);
                assert(ri->ckey < t->ckeyCount);
                char *cvec = scratch->core_info.combVector;
                setCombinationActive(t, cvec, ri->ckey);
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(FLUSH_COMBINATION) {
                assert(end >= tctxt->lastCombMatchOffset);
                if (end > tctxt->lastCombMatchOffset) {
                    if (flushActiveCombinations(t, scratch)
                            == HWLM_TERMINATE_MATCHING) {
                        return HWLM_TERMINATE_MATCHING;
                    }
                }
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SET_EXHAUST) {
                updateSeqPoint(tctxt, end, from_mpv);
                if (roseSetExhaust(t, scratch, ri->ekey)
                        == HWLM_TERMINATE_MATCHING) {
                    return HWLM_TERMINATE_MATCHING;
                }
                work_done = 1;
            }
            PROGRAM_NEXT_INSTRUCTION
        }
    }

    assert(0); // unreachable
    return HWLM_CONTINUE_MATCHING;
}

#define L_PROGRAM_CASE(name)                                                   \
    case ROSE_INSTR_##name: {                                                  \
        DEBUG_PRINTF("l_instruction: " #name " (pc=%u)\n",                     \
                     programOffset + (u32)(pc - pc_base));                     \
        const struct ROSE_STRUCT_##name *ri =                                  \
            (const struct ROSE_STRUCT_##name *)pc;

#define L_PROGRAM_NEXT_INSTRUCTION                                             \
    pc += ROUNDUP_N(sizeof(*ri), ROSE_INSTR_MIN_ALIGN);                        \
    break;                                                                     \
    }

#define L_PROGRAM_NEXT_INSTRUCTION_JUMP continue;

hwlmcb_rv_t roseRunProgram_l(const struct RoseEngine *t,
                             struct hs_scratch *scratch, u32 programOffset,
                             u64a som, u64a end, u8 prog_flags) {
    DEBUG_PRINTF("program=%u, offsets [%llu,%llu], flags=%u\n", programOffset,
                 som, end, prog_flags);

    assert(programOffset != ROSE_INVALID_PROG_OFFSET);
    assert(programOffset >= sizeof(struct RoseEngine));
    assert(programOffset < t->size);

    const char from_mpv = prog_flags & ROSE_PROG_FLAG_FROM_MPV;

    const char *pc_base = getByOffset(t, programOffset);
    const char *pc = pc_base;

    struct RoseContext *tctxt = &scratch->tctxt;

    assert(*(const u8 *)pc != ROSE_INSTR_END);

    for (;;) {
        assert(ISALIGNED_N(pc, ROSE_INSTR_MIN_ALIGN));
        assert(pc >= pc_base);
        assert((size_t)(pc - pc_base) < t->size);
        const u8 code = *(const u8 *)pc;
        assert(code <= LAST_ROSE_INSTRUCTION);

        switch ((enum RoseInstructionCode)code) {
            L_PROGRAM_CASE(END) {
                DEBUG_PRINTF("finished\n");
                return HWLM_CONTINUE_MATCHING;
            }
            L_PROGRAM_NEXT_INSTRUCTION

            L_PROGRAM_CASE(CATCH_UP) {
                if (roseCatchUpTo(t, scratch, end) == HWLM_TERMINATE_MATCHING) {
                    return HWLM_TERMINATE_MATCHING;
                }
            }
            L_PROGRAM_NEXT_INSTRUCTION

            L_PROGRAM_CASE(SOM_FROM_REPORT) {
                som = handleSomExternal(scratch, &ri->som, end);
                DEBUG_PRINTF("som from report %u is %llu\n", ri->som.onmatch,
                             som);
            }
            L_PROGRAM_NEXT_INSTRUCTION

            L_PROGRAM_CASE(DEDUPE) {
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
                    L_PROGRAM_NEXT_INSTRUCTION_JUMP
                case DEDUPE_CONTINUE:
                    break;
                }
            }
            L_PROGRAM_NEXT_INSTRUCTION

            L_PROGRAM_CASE(DEDUPE_SOM) {
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
                    L_PROGRAM_NEXT_INSTRUCTION_JUMP
                case DEDUPE_CONTINUE:
                    break;
                }
            }
            L_PROGRAM_NEXT_INSTRUCTION

            L_PROGRAM_CASE(REPORT) {
                updateSeqPoint(tctxt, end, from_mpv);
                if (roseReport(t, scratch, end, ri->onmatch, ri->offset_adjust,
                               INVALID_EKEY) == HWLM_TERMINATE_MATCHING) {
                    return HWLM_TERMINATE_MATCHING;
                }
            }
            L_PROGRAM_NEXT_INSTRUCTION

            L_PROGRAM_CASE(REPORT_EXHAUST) {
                updateSeqPoint(tctxt, end, from_mpv);
                if (roseReport(t, scratch, end, ri->onmatch, ri->offset_adjust,
                               ri->ekey) == HWLM_TERMINATE_MATCHING) {
                    return HWLM_TERMINATE_MATCHING;
                }
            }
            L_PROGRAM_NEXT_INSTRUCTION

            L_PROGRAM_CASE(REPORT_SOM) {
                updateSeqPoint(tctxt, end, from_mpv);
                if (roseReportSom(t, scratch, som, end, ri->onmatch,
                                  ri->offset_adjust,
                                  INVALID_EKEY) == HWLM_TERMINATE_MATCHING) {
                    return HWLM_TERMINATE_MATCHING;
                }
            }
            L_PROGRAM_NEXT_INSTRUCTION

            L_PROGRAM_CASE(DEDUPE_AND_REPORT) {
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
                    L_PROGRAM_NEXT_INSTRUCTION_JUMP
                case DEDUPE_CONTINUE:
                    break;
                }

                const u32 ekey = INVALID_EKEY;
                if (roseReport(t, scratch, end, ri->onmatch, ri->offset_adjust,
                               ekey) == HWLM_TERMINATE_MATCHING) {
                    return HWLM_TERMINATE_MATCHING;
                }
            }
            L_PROGRAM_NEXT_INSTRUCTION

            L_PROGRAM_CASE(FINAL_REPORT) {
                updateSeqPoint(tctxt, end, from_mpv);
                if (roseReport(t, scratch, end, ri->onmatch, ri->offset_adjust,
                               INVALID_EKEY) == HWLM_TERMINATE_MATCHING) {
                    return HWLM_TERMINATE_MATCHING;
                }
                /* One-shot specialisation: this instruction always terminates
                 * execution of the program. */
                return HWLM_CONTINUE_MATCHING;
            }
            L_PROGRAM_NEXT_INSTRUCTION

            L_PROGRAM_CASE(CHECK_EXHAUSTED) {
                DEBUG_PRINTF("check ekey %u\n", ri->ekey);
                assert(ri->ekey != INVALID_EKEY);
                assert(ri->ekey < t->ekeyCount);
                const char *evec = scratch->core_info.exhaustionVector;
                if (isExhausted(t, evec, ri->ekey)) {
                    DEBUG_PRINTF("ekey %u already set, match is exhausted\n",
                                 ri->ekey);
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    L_PROGRAM_NEXT_INSTRUCTION_JUMP
                }
            }
            L_PROGRAM_NEXT_INSTRUCTION

            L_PROGRAM_CASE(CHECK_LONG_LIT) {
                const char nocase = 0;
                if (!roseCheckLongLiteral(t, scratch, end, ri->lit_offset,
                                          ri->lit_length, nocase)) {
                    DEBUG_PRINTF("failed long lit check\n");
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    L_PROGRAM_NEXT_INSTRUCTION_JUMP
                }
            }
            L_PROGRAM_NEXT_INSTRUCTION

            L_PROGRAM_CASE(CHECK_LONG_LIT_NOCASE) {
                const char nocase = 1;
                if (!roseCheckLongLiteral(t, scratch, end, ri->lit_offset,
                                          ri->lit_length, nocase)) {
                    DEBUG_PRINTF("failed nocase long lit check\n");
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    L_PROGRAM_NEXT_INSTRUCTION_JUMP
                }
            }
            L_PROGRAM_NEXT_INSTRUCTION

            L_PROGRAM_CASE(CHECK_MED_LIT) {
                const char nocase = 0;
                if (!roseCheckMediumLiteral(t, scratch, end, ri->lit_offset,
                                            ri->lit_length, nocase)) {
                    DEBUG_PRINTF("failed lit check\n");
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    L_PROGRAM_NEXT_INSTRUCTION_JUMP
                }
            }
            L_PROGRAM_NEXT_INSTRUCTION

            L_PROGRAM_CASE(CHECK_MED_LIT_NOCASE) {
                const char nocase = 1;
                if (!roseCheckMediumLiteral(t, scratch, end, ri->lit_offset,
                                            ri->lit_length, nocase)) {
                    DEBUG_PRINTF("failed long lit check\n");
                    assert(ri->fail_jump); // must progress
                    pc += ri->fail_jump;
                    L_PROGRAM_NEXT_INSTRUCTION_JUMP
                }
            }
            L_PROGRAM_NEXT_INSTRUCTION

            L_PROGRAM_CASE(SET_LOGICAL) {
                DEBUG_PRINTF("set logical value of lkey %u, offset_adjust=%d\n",
                             ri->lkey, ri->offset_adjust);
                assert(ri->lkey != INVALID_LKEY);
                assert(ri->lkey < t->lkeyCount);
                char *lvec = scratch->core_info.logicalVector;
                setLogicalVal(t, lvec, ri->lkey, 1);
                updateLastCombMatchOffset(tctxt, end + ri->offset_adjust);
            }
            L_PROGRAM_NEXT_INSTRUCTION

            L_PROGRAM_CASE(SET_COMBINATION) {
                DEBUG_PRINTF("set ckey %u as active\n", ri->ckey);
                assert(ri->ckey != INVALID_CKEY);
                assert(ri->ckey < t->ckeyCount);
                char *cvec = scratch->core_info.combVector;
                setCombinationActive(t, cvec, ri->ckey);
            }
            L_PROGRAM_NEXT_INSTRUCTION

            L_PROGRAM_CASE(FLUSH_COMBINATION) {
                assert(end >= tctxt->lastCombMatchOffset);
                if (end > tctxt->lastCombMatchOffset) {
                    if (flushActiveCombinations(t, scratch)
                            == HWLM_TERMINATE_MATCHING) {
                        return HWLM_TERMINATE_MATCHING;
                    }
                }
            }
            L_PROGRAM_NEXT_INSTRUCTION

            L_PROGRAM_CASE(SET_EXHAUST) {
                updateSeqPoint(tctxt, end, from_mpv);
                if (roseSetExhaust(t, scratch, ri->ekey)
                        == HWLM_TERMINATE_MATCHING) {
                    return HWLM_TERMINATE_MATCHING;
                }
            }
            L_PROGRAM_NEXT_INSTRUCTION

            default: {
                assert(0); // unreachable
            }
        }
    }

    assert(0); // unreachable
    return HWLM_CONTINUE_MATCHING;
}

#undef L_PROGRAM_CASE
#undef L_PROGRAM_NEXT_INSTRUCTION
#undef L_PROGRAM_NEXT_INSTRUCTION_JUMP

#undef PROGRAM_CASE
#undef PROGRAM_NEXT_INSTRUCTION
#undef PROGRAM_NEXT_INSTRUCTION_JUMP
