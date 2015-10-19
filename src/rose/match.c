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

#include "catchup.h"
#include "counting_miracle.h"
#include "infix.h"
#include "match.h"
#include "miracle.h"
#include "rose_sidecar_runtime.h"
#include "rose.h"
#include "som/som_runtime.h"
#include "util/bitutils.h"
#include "util/fatbit.h"

#if defined(DEBUG) || defined(DUMP_SUPPORT)
#include "util/compare.h"
/** A debugging crutch: print a hex-escaped version of the match for our
 * perusal. The start and end offsets are stream offsets. */
static UNUSED
void printMatch(const struct core_info *ci, u64a start, u64a end) {
    assert(start <= end);
    assert(end <= ci->buf_offset + ci->len);

    printf("'");
    u64a i = start;
    for (; i <= MIN(ci->buf_offset, end); i++) {
        u64a h_idx = ci->buf_offset - i;
        u8 c = h_idx >= ci->hlen ? '?' : ci->hbuf[ci->hlen - h_idx - 1];
        if (ourisprint(c) && c != '\'') {
            printf("%c", c);
        } else {
            printf("\\x%02x", c);
        }
    }
    for (; i <= end; i++) {
        u64a b_idx = i - ci->buf_offset - 1;
        u8 c = b_idx >= ci->len ? '?' : ci->buf[b_idx];
        if (ourisprint(c) && c != '\'') {
            printf("%c", c);
        } else {
            printf("\\x%02x", c);
        }
    }
    printf("'");
}
#endif

static rose_inline
int roseCheckBenefits(struct RoseContext *tctxt, u64a end, u32 mask_rewind,
                      const u8 *and_mask, const u8 *exp_mask) {
    DEBUG_PRINTF("am offset = %zu, em offset = %zu\n",
                 and_mask - (const u8 *)tctxt->t,
                 exp_mask - (const u8 *)tctxt->t);
    const u8 *data;

    // If the check works over part of the history and part of the buffer, we
    // create a temporary copy of the data in here so it's contiguous.
    u8 temp[MAX_MASK2_WIDTH];

    struct core_info *ci = &tctxtToScratch(tctxt)->core_info;
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

static
int roseCheckLiteralBenefits(u64a end, size_t mask_rewind, u32 id,
                             struct RoseContext *tctxt) {
    const struct RoseEngine *t = tctxt->t;
    const struct lit_benefits *lbi = getLiteralBenefitsTable(t) + id;
    return roseCheckBenefits(tctxt, end, mask_rewind, lbi->and_mask.a8,
                             lbi->expected.e8);
}

static rose_inline
void pushDelayedMatches(const struct RoseLiteral *tl, u64a offset,
                        struct RoseContext *tctxt) {
    u32 delay_mask = tl->delay_mask;
    if (!delay_mask) {
        return;
    }

    u32 delay_count = tctxt->t->delay_count;
    u8 *delaySlotBase = getDelaySlots(tctxtToScratch(tctxt));
    size_t delaySlotSize = tctxt->t->delay_slot_size;
    assert(tl->delayIdsOffset != ROSE_OFFSET_INVALID);
    const u32 *delayIds = getByOffset(tctxt->t, tl->delayIdsOffset);
    assert(ISALIGNED(delayIds));

    while (delay_mask) {
        u32 src_slot_index = findAndClearLSB_32(&delay_mask);
        u32 slot_index = (src_slot_index + offset) & DELAY_MASK;
        u8 *slot = delaySlotBase + delaySlotSize * slot_index;

        if (offset + src_slot_index <= tctxt->delayLastEndOffset) {
            DEBUG_PRINTF("skip too late\n");
            goto next;
        }

        DEBUG_PRINTF("pushing tab %u into slot %u\n", *delayIds, slot_index);
        if (!(tctxt->filledDelayedSlots & (1U << slot_index))) {
            tctxt->filledDelayedSlots |= 1U << slot_index;
            mmbit_clear(slot, delay_count);
        }

        mmbit_set(slot, delay_count, *delayIds);
    next:
        delayIds++;
    }
}

hwlmcb_rv_t roseDelayRebuildCallback(size_t start, size_t end, u32 id,
                                     void *ctx) {
    struct hs_scratch *scratch = ctx;
    struct RoseContext *tctx = &scratch->tctxt;
    const struct RoseEngine *t = tctx->t;
    struct core_info *ci = &scratch->core_info;
    size_t rb_len = MIN(ci->hlen, t->delayRebuildLength);

    u64a real_end = ci->buf_offset - rb_len + end + 1; // index after last byte

#ifdef DEBUG
    DEBUG_PRINTF("REBUILD MATCH id=%u offsets=[%llu,%llu]: ", id,
                 start + ci->buf_offset - rb_len, real_end);
    printMatch(ci, start + ci->buf_offset - rb_len, real_end);
    printf("\n");
#endif

    DEBUG_PRINTF("STATE depth=%u, groups=0x%016llx\n", tctx->depth,
                 tctx->groups);

    if (isLiteralDR(id)) {
        return tctx->groups;
    }

    if (id < t->nonbenefits_base_id
        && !roseCheckLiteralBenefits(real_end, end - start + 1, id, tctx)) {
        return tctx->groups;
    }

    assert(id < t->literalCount);
    const struct RoseLiteral *tl = &getLiteralTable(t)[id];

    DEBUG_PRINTF("literal id=%u, minDepth=%u, groups=0x%016llx\n",
                 id, tl->minDepth, tl->groups);

    pushDelayedMatches(tl, real_end, tctx);

    /* we are just repopulating the delay queue, groups and depths should be
     * already set from the original scan. */

    return tctx->groups;
}

/* Note: uses the stashed sparse iter state; cannot be called from
 * anybody else who is using it
 */
static never_inline
void roseSquashStates(const struct RoseEngine *t, const struct RoseSide *tsb,
                      struct RoseContext *tctxt) {
    DEBUG_PRINTF("attempting to squash states\n");

    struct mmbit_sparse_state *s = tctxtToScratch(tctxt)->sparse_iter_state;
    u8 *state = tctxt->state;
    void *role_state = getRoleState(state);
    u32 role_count = t->rolesWithStateCount;
    const struct mmbit_sparse_iter *it = getByOffset(t, tsb->squashIterOffset);
    assert(ISALIGNED(it));

    /* we can squash willy-nilly */
    DEBUG_PRINTF("squashing iter off = %u\n", tsb->squashIterOffset);
    mmbit_sparse_iter_unset(role_state, role_count, it, s);
    DEBUG_PRINTF("squashing groups with = %016llx\n", tsb->squashGroupMask);
    tctxt->groups &= tsb->squashGroupMask;
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

static really_inline
hwlmcb_rv_t ensureMpvQueueFlushed(const struct RoseEngine *t,
                                  struct hs_scratch *scratch, u32 qi, s64a loc,
                                  char in_anchored, char in_chained) {
    return ensureQueueFlushed_i(t, scratch, qi, loc, 1, in_anchored,
                                in_chained);
}

static rose_inline
hwlmcb_rv_t roseHandleSuffixTrigger(const struct RoseEngine *t,
                                    const struct RoseRole *tr, u64a som,
                                    u64a end, struct RoseContext *tctxt,
                                    char in_anchored) {
    DEBUG_PRINTF("woot we have a mask/follower/suffix/... role\n");

    assert(tr->suffixOffset);

    const struct NFA *nfa
        = (const struct NFA *)((const char *)t + tr->suffixOffset);
    u8 *aa = getActiveLeafArray(t, tctxt->state);
    struct hs_scratch *scratch = tctxtToScratch(tctxt);
    u32 aaCount = t->activeArrayCount;
    u32 qCount = t->queueCount;
    u32 qi = nfa->queueIndex;
    struct mq *q = &scratch->queues[qi];
    const struct NfaInfo *info = getNfaInfoByQueue(t, qi);

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

    enum mqe_event top = tr->suffixEvent;
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

static rose_inline
void recordAnchoredMatch(struct RoseContext *tctxt, ReportID reportId,
                         u64a end) {
    const struct RoseEngine *t = tctxt->t;
    struct hs_scratch *scratch = tctxtToScratch(tctxt);
    u8 **anchoredRows = getAnchoredLog(scratch);

    DEBUG_PRINTF("record %u @ %llu\n", reportId, end);
    assert(end - t->maxSafeAnchoredDROffset >= 1);
    u32 adj_end = end - t->maxSafeAnchoredDROffset - 1;
    DEBUG_PRINTF("adjusted location %u/%u\n", adj_end,
                  scratch->anchored_region_len);

    if (!bf64_set(&scratch->am_log_sum, adj_end)) {
        // first time, clear row
        mmbit_clear(anchoredRows[adj_end], t->anchoredMatches);
    }

    u32 idx = getAnchoredInverseMap(t)[reportId];
    DEBUG_PRINTF("record %u @ %llu index %u\n", reportId, end, idx);
    assert(idx < t->anchoredMatches);
    mmbit_set(anchoredRows[adj_end], t->anchoredMatches, idx);
}

static rose_inline
void recordAnchoredLiteralMatch(struct RoseContext *tctxt, u32 literal_id,
                                u64a end) {
    assert(end);
    const struct RoseEngine *t = tctxt->t;
    struct hs_scratch *scratch = tctxtToScratch(tctxt);
    u8 **anchoredLiteralRows = getAnchoredLiteralLog(scratch);

    DEBUG_PRINTF("record %u @ %llu\n", literal_id, end);

    if (!bf64_set(&scratch->al_log_sum, end - 1)) {
        // first time, clear row
        DEBUG_PRINTF("clearing %llu/%u\n", end - 1, t->anchored_count);
        mmbit_clear(anchoredLiteralRows[end - 1], t->anchored_count);
    }

    u32 rel_idx = literal_id - t->anchored_base_id;
    DEBUG_PRINTF("record %u @ %llu index %u/%u\n", literal_id, end, rel_idx,
                 t->anchored_count);
    assert(rel_idx < t->anchored_count);
    mmbit_set(anchoredLiteralRows[end - 1], t->anchored_count, rel_idx);
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

hwlmcb_rv_t roseHandleChainMatch(const struct RoseEngine *t, ReportID r,
                                 u64a end, struct RoseContext *tctxt,
                                 char in_anchored, char in_catchup) {
    struct hs_scratch *scratch = tctxtToScratch(tctxt);
    struct core_info *ci = &scratch->core_info;

    u8 *aa = getActiveLeafArray(t, tctxt->state);
    u32 aaCount = t->activeArrayCount;
    struct fatbit *activeQueues = scratch->aqa;
    u32 qCount = t->queueCount;

    const struct internal_report *ri = getInternalReport(t, r);
    assert(ri->type == INTERNAL_ROSE_CHAIN);

    u32 qi = 0; /* MPV is always queue 0 if it exists */
    u32 event = ri->onmatch;
    assert(event == MQE_TOP || event >= MQE_TOP_FIRST);

    /* TODO: populate INTERNAL_ROSE_CHAIN internal reports with offset where
     * possible */
    if (end < ri->minOffset || (ri->maxOffset && end > ri->maxOffset)) {
        return HWLM_CONTINUE_MATCHING;
    }
    struct mq *q = &scratch->queues[qi];
    const struct NfaInfo *info = getNfaInfoByQueue(t, qi);

    s64a loc = (s64a)end - ci->buf_offset;
    assert(loc <= (s64a)ci->len && loc >= -(s64a)ci->hlen);

    if (!mmbit_set(aa, aaCount, qi)) {
        initQueue(q, qi, t, tctxt);
        nfaQueueInitState(q->nfa, q);
        pushQueueAt(q, 0, MQE_START, loc);
        fatbit_set(activeQueues, qCount, qi);
    } else if (info->no_retrigger) {
        DEBUG_PRINTF("yawn\n");
        /* nfa only needs one top; we can go home now */
        return HWLM_CONTINUE_MATCHING;
    } else if (!fatbit_set(activeQueues, qCount, qi)) {
        initQueue(q, qi, t, tctxt);
        loadStreamState(q->nfa, q, 0);
        pushQueueAt(q, 0, MQE_START, 0);
    } else if (isQueueFull(q)) {
        DEBUG_PRINTF("queue %u full -> catching up nfas\n", qi);
        /* we know it is a chained nfa and the suffixes/outfixes must already
         * be known to be consistent */
        if (ensureMpvQueueFlushed(t, scratch, qi, loc, in_anchored, in_catchup)
            == HWLM_TERMINATE_MATCHING) {
            return HWLM_TERMINATE_MATCHING;
        }
    }

    if (ri->aux.topSquashDistance) {
        assert(q->cur != q->end);
        struct mq_item *last = &q->items[q->end - 1];
        if (last->type == event
            && last->location >= loc - (s64a)ri->aux.topSquashDistance) {
            last->location = loc;
            goto event_enqueued;
        }
    }

    pushQueue(q, event, loc);

event_enqueued:
    if (q_cur_loc(q) == (s64a)ci->len) {
        /* we may not run the nfa; need to ensure state is fine  */
        DEBUG_PRINTF("empty run\n");
        pushQueueNoMerge(q, MQE_END, loc);
        char alive = nfaQueueExec(q->nfa, q, loc);
        if (alive) {
            tctxt->mpv_inactive = 0;
            q->cur = q->end = 0;
            pushQueueAt(q, 0, MQE_START, loc);
        } else {
            mmbit_unset(aa, aaCount, qi);
            fatbit_unset(scratch->aqa, qCount, qi);
        }
    }

    DEBUG_PRINTF("added mpv event at %lld\n", loc);
    tctxt->next_mpv_offset = 0; /* the top event may result in matches earlier
                                 * than expected */
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

    if (q->items[q->end - 1].type == MQE_START) {
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
char roseTestLeftfix(const struct RoseEngine *t, const struct RoseRole *tr,
                     u64a end, struct RoseContext *tctxt) {
    struct hs_scratch *scratch = tctxtToScratch(tctxt);
    struct core_info *ci = &scratch->core_info;
    assert(tr->flags & ROSE_ROLE_FLAG_ROSE);

    u32 qi = tr->leftfixQueue;

    u32 ri = queueToLeftIndex(t, qi);
    const struct LeftNfaInfo *left = getLeftTable(t) + ri;

    DEBUG_PRINTF("testing %s %s %u/%u with lag %u (maxLag=%u)\n",
                 (left->transient ? "transient" : "active"),
                 (left->infix ? "infix" : "prefix"),
                 ri, qi, tr->leftfixLag, left->maxLag);

    assert(tr->leftfixLag <= left->maxLag);

    struct mq *q = scratch->queues + qi;
    u32 qCount = t->queueCount;
    u32 arCount = t->activeLeftCount;

    if (!mmbit_isset(getActiveLeftArray(t, tctxt->state), arCount, ri)) {
        DEBUG_PRINTF("engine is dead nothing to see here\n");
        return 0;
    }

    if (unlikely(end < tr->leftfixLag)) {
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

    s64a loc = (s64a)end - ci->buf_offset - tr->leftfixLag;
    assert(loc >= q_cur_loc(q));
    assert(tr->leftfixReport != MO_INVALID_IDX);

    if (left->transient) {
        s64a start_loc = loc - left->transient;
        if (q_cur_loc(q) < start_loc) {
            q->cur = q->end = 0;
            pushQueueAt(q, 0, MQE_START, start_loc);
            pushQueueAt(q, 1, MQE_TOP, start_loc);
            nfaQueueInitState(q->nfa, q);
        }
    }

    if (q_cur_loc(q) < loc || q->items[q->end - 1].type != MQE_START) {
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

        char rv = nfaQueueExecRose(q->nfa, q, tr->leftfixReport);
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

        DEBUG_PRINTF("checking for report %u\n", tr->leftfixReport);
        DEBUG_PRINTF("leftfix done %hhd\n", (signed char)rv);
        return rv == MO_MATCHES_PENDING;
    } else {
        DEBUG_PRINTF("checking for report %u\n", tr->leftfixReport);
        char rv = nfaInAcceptState(q->nfa, tr->leftfixReport, q);
        DEBUG_PRINTF("leftfix done %hhd\n", (signed char)rv);
        return rv;
    }
}

static rose_inline
void roseSetRole(const struct RoseEngine *t, u8 *state,
                 struct RoseContext *tctxt, const struct RoseRole *tr) {
    DEBUG_PRINTF("set role %u on: idx=%u, depth=%u, groups=0x%016llx\n",
                 (u32)(tr - getRoleTable(t)),
                 tr->stateIndex, tr->depth, tr->groups);
    void *role_state = getRoleState(state);

    assert(tr < getRoleTable(t) + t->roleCount);

    int leafNode = !!(tr->stateIndex == MMB_INVALID);

    // If this role is a leaf node, it doesn't have a state index to switch
    // on and it doesn't need any history stored or other work done. So we can
    // bail.
    /* may be a ghost role; still need to set groups */
    if (leafNode) {
        tctxt->groups |= tr->groups;
        DEBUG_PRINTF("role %u is a leaf node, no work to do.\n",
                (u32)(tr - getRoleTable(t)));
        return;
    }

    // Switch this role on in the state bitvector, checking whether it was set
    // already.
    char alreadySet = mmbit_set(role_state, t->rolesWithStateCount,
                                tr->stateIndex);

    // Roles that we've already seen have had most of their bookkeeping done:
    // all we need to do is update the offset table if this is an
    // offset-tracking role.
    if (alreadySet) {
        DEBUG_PRINTF("role already set\n");
        if (tr->sidecarEnableOffset) {
            enable_sidecar(tctxt, tr);
        }
        return;
    }

    // If this role's depth is greater than the current depth, update it
    update_depth(tctxt, tr);

    // Switch on this role's groups
    tctxt->groups |= tr->groups;

    if (tr->sidecarEnableOffset) {
        // We have to enable some sidecar literals
        enable_sidecar(tctxt, tr);
    }
}

static rose_inline
void roseTriggerInfixes(const struct RoseEngine *t, const struct RoseRole *tr,
                        u64a start, u64a end, struct RoseContext *tctxt) {
    struct core_info *ci = &tctxtToScratch(tctxt)->core_info;

    DEBUG_PRINTF("infix time! @%llu\t(s%llu)\n", end, start);

    assert(tr->infixTriggerOffset);

    u32 qCount = t->queueCount;
    u32 arCount = t->activeLeftCount;
    struct fatbit *aqa = tctxtToScratch(tctxt)->aqa;
    u8 *activeLeftArray = getActiveLeftArray(t, tctxt->state);
    s64a loc = (s64a)end - ci->buf_offset;

    const struct RoseTrigger *curr_r = (const struct RoseTrigger *)
        ((const char *)t + tr->infixTriggerOffset);
    assert(ISALIGNED_N(curr_r, alignof(struct RoseTrigger)));
    assert(curr_r->queue != MO_INVALID_IDX); /* shouldn't be here if no
                                              * triggers */
    do {
        u32 qi = curr_r->queue;
        u32 ri = queueToLeftIndex(t, qi);
        enum mqe_event topEvent = curr_r->event;
        u8 cancel = curr_r->cancel_prev_top;
        assert(topEvent < MQE_INVALID);

        const struct LeftNfaInfo *left = getLeftInfoByQueue(t, qi);
        assert(!left->transient);

        DEBUG_PRINTF("rose %u (qi=%u) event %u\n", ri, qi, (u32)topEvent);

        struct mq *q = tctxtToScratch(tctxt)->queues + qi;
        const struct NfaInfo *info = getNfaInfoByQueue(t, qi);

        char alive = mmbit_set(activeLeftArray, arCount, ri);

        if (alive && info->no_retrigger) {
            DEBUG_PRINTF("yawn\n");
            goto next_infix;
        }

        if (alive && nfaSupportsZombie(getNfaByInfo(t, info)) && ci->buf_offset
            && !fatbit_isset(aqa, qCount, qi)
            && isZombie(t, tctxt->state, left)) {
            DEBUG_PRINTF("yawn - zombie\n");
            goto next_infix;
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
                DEBUG_PRINTF("queue %u full (%u items) -> catching up nfa\n",
                             qi, q->end - q->cur);
                pushQueueNoMerge(q, MQE_END, loc);
                nfaQueueExecRose(q->nfa, q, MO_INVALID_IDX);

                q->cur = q->end = 0;
                pushQueueAt(q, 0, MQE_START, loc);
            }
        }

        pushQueueSom(q, topEvent, loc, start);
    next_infix:
        ++curr_r;
    } while (curr_r->queue != MO_INVALID_IDX);
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
int roseCheckLookaround(const struct RoseEngine *t, const struct RoseRole *tr,
                        u64a end, struct RoseContext *tctxt) {
    assert(tr->lookaroundIndex != MO_INVALID_IDX);
    assert(tr->lookaroundCount > 0);

    const struct core_info *ci = &tctxtToScratch(tctxt)->core_info;
    DEBUG_PRINTF("end=%llu, buf_offset=%llu, buf_end=%llu\n", end,
                 ci->buf_offset, ci->buf_offset + ci->len);

    const u8 *base = (const u8 *)t;
    const s8 *look_base = (const s8 *)(base + t->lookaroundTableOffset);
    const s8 *look = look_base + tr->lookaroundIndex;
    const s8 *look_end = look + tr->lookaroundCount;
    assert(look < look_end);

    const u8 *reach_base = base + t->lookaroundReachOffset;
    const u8 *reach = reach_base + tr->lookaroundIndex * REACH_BITVECTOR_LEN;

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

static rose_inline
int roseCheckRolePreconditions(const struct RoseEngine *t,
                               const struct RoseRole *tr, u64a end,
                               struct RoseContext *tctxt) {
    // If this role can only match at end-of-block, then check that it's so.
    if (tr->flags & ROSE_ROLE_FLAG_ONLY_AT_END) {
        struct core_info *ci = &tctxtToScratch(tctxt)->core_info;
        if (end != ci->buf_offset + ci->len) {
            DEBUG_PRINTF("role %u should only match at end of data, skipping\n",
                         (u32)(tr - getRoleTable(t)));
            return 0;
        }
    }

    if (tr->lookaroundIndex != MO_INVALID_IDX) {
        if (!roseCheckLookaround(t, tr, end, tctxt)) {
            DEBUG_PRINTF("failed lookaround check\n");
            return 0;
        }
    }

    assert(!tr->leftfixQueue || (tr->flags & ROSE_ROLE_FLAG_ROSE));
    if (tr->flags & ROSE_ROLE_FLAG_ROSE) {
        if (!roseTestLeftfix(t, tr, end, tctxt)) {
            DEBUG_PRINTF("failed leftfix check\n");
            return 0;
        }
    }

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
u64a roseGetHaigSom(const struct RoseEngine *t, const struct RoseRole *tr,
                    UNUSED u64a end, struct RoseContext *tctxt) {
    assert(tr->flags & ROSE_ROLE_FLAG_ROSE);

    u32 qi = tr->leftfixQueue;
    u32 ri = queueToLeftIndex(t, qi);

    UNUSED const struct LeftNfaInfo *left = getLeftTable(t) + ri;

    DEBUG_PRINTF("testing %s prefix %u/%u with lag %u (maxLag=%u)\n",
                 left->transient ? "transient" : "active", ri, qi,
                 tr->leftfixLag, left->maxLag);

    assert(tr->leftfixLag <= left->maxLag);

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

static really_inline
hwlmcb_rv_t roseHandleRoleEffects(const struct RoseEngine *t,
                                  const struct RoseRole *tr, u64a end,
                                  struct RoseContext *tctxt, char in_anchored,
                                  int *work_done) {
    u64a som = 0ULL;
    if (tr->flags & ROSE_ROLE_FLAG_SOM_ADJUST) {
        som = end - tr->somAdjust;
        DEBUG_PRINTF("som requested som %llu = %llu - %u\n", som, end,
                     tr->somAdjust);
    } else if (tr->flags & ROSE_ROLE_FLAG_SOM_ROSEFIX) {
        som = roseGetHaigSom(t, tr, end, tctxt);
        DEBUG_PRINTF("som from rosefix %llu\n", som);
    }

    if (tr->infixTriggerOffset) {
        roseTriggerInfixes(t, tr, som, end, tctxt);
        tctxt->groups |= tr->groups; /* groups may have been cleared by infix
                                      * going quiet before */
    }

    if (tr->suffixOffset) {
        hwlmcb_rv_t rv = roseHandleSuffixTrigger(t, tr, som, end, tctxt,
                                                 in_anchored);
        if (rv != HWLM_CONTINUE_MATCHING) {
            return rv;
        }
    }

    if (tr->reportId != MO_INVALID_IDX) {
        hwlmcb_rv_t rv;
        if (tr->flags & ROSE_ROLE_FLAG_REPORT_START) {
            /* rose role knows its start offset */
            assert(tr->flags & ROSE_ROLE_FLAG_SOM_ROSEFIX);
            assert(!(tr->flags & ROSE_ROLE_FLAG_CHAIN_REPORT));
            if (tr->flags & ROSE_ROLE_FLAG_SOM_REPORT) {
                rv = roseHandleSomSom(t, tctxt->state, tr->reportId, som, end,
                                      tctxt, in_anchored);
            } else {
                rv = roseHandleSomMatch(t, tctxt->state, tr->reportId, som, end,
                                        tctxt, in_anchored);
            }
        } else {
            if (tr->flags & ROSE_ROLE_FLAG_SOM_REPORT) {
                /* do som management */
                rv = roseHandleSom(t, tctxt->state, tr->reportId, end, tctxt,
                                  in_anchored);
            } else if (tr->flags & ROSE_ROLE_FLAG_CHAIN_REPORT) {
                rv = roseCatchUpAndHandleChainMatch(t, tctxt->state,
                                                    tr->reportId, end, tctxt,
                                                    in_anchored);
            } else {
                rv = roseHandleMatch(t, tctxt->state, tr->reportId, end, tctxt,
                                     in_anchored);
            }
        }

        if (rv != HWLM_CONTINUE_MATCHING) {
            return HWLM_TERMINATE_MATCHING;
        }
    }

    roseSetRole(t, tctxt->state, tctxt, tr);

    *work_done = 1;

    return HWLM_CONTINUE_MATCHING;
}

static really_inline
hwlmcb_rv_t roseHandleRole(const struct RoseEngine *t,
                           const struct RoseRole *tr, u64a end,
                           struct RoseContext *tctxt, char in_anchored,
                           int *work_done) {
    DEBUG_PRINTF("hi role %zd (flags %08x)\n", tr - getRoleTable(t),
                  tr->flags);
    if (in_anchored && end > t->floatingMinLiteralMatchOffset) {
        DEBUG_PRINTF("delay until playback, just do groups/depth now\n");
        update_depth(tctxt, tr);
        tctxt->groups |= tr->groups;
        *work_done = 1;
        return HWLM_CONTINUE_MATCHING;
    }

    if (!roseCheckRolePreconditions(t, tr, end, tctxt)) {
        return HWLM_CONTINUE_MATCHING;
    }

    /* We now know the role has matched. We can now trigger things that need to
     * be triggered and record things that need to be recorded.*/

    return roseHandleRoleEffects(t, tr, end, tctxt, in_anchored, work_done);
}

static really_inline
void roseSquashGroup(struct RoseContext *tctxt, const struct RoseLiteral *tl) {
    assert(tl->squashesGroup);

    // we should be squashing a single group
    assert(popcount64(tl->groups) == 1);

    DEBUG_PRINTF("apply squash mask 0x%016llx, groups 0x%016llx -> 0x%016llx\n",
                 ~tl->groups, tctxt->groups, tctxt->groups & ~tl->groups);

    tctxt->groups &= ~tl->groups;
}

// Run the sparse iterator for this literal and use that to discover which
// roles to consider.
/* Note: uses the stashed sparse iter state; cannot be called from
 * anybody else who is using it */
/* Note: uses the handled role mmbit; cannot be called from
 * anybody else who is using it (nobody else should be) */
/* non-root roles should not occur in any anchored context */
static really_inline
hwlmcb_rv_t roseWalkSparseIterator(const struct RoseEngine *t,
                                   const struct RoseLiteral *tl, u64a end,
                                   struct RoseContext *tctxt) {
    /* assert(!tctxt->in_anchored); */
    /* assert(!tctxt->in_anch_playback); */
    const struct RoseRole *roleTable = getRoleTable(t);
    const struct RosePred *predTable = getPredTable(t);
    const struct RoseIterMapping *iterMapBase
        = getByOffset(t, tl->iterMapOffset);
    const struct mmbit_sparse_iter *it = getByOffset(t, tl->iterOffset);
    assert(ISALIGNED(iterMapBase));
    assert(ISALIGNED(it));

    // Sparse iterator state was allocated earlier
    struct mmbit_sparse_state *s = tctxtToScratch(tctxt)->sparse_iter_state;
    struct fatbit *handled_roles = tctxtToScratch(tctxt)->handled_roles;

    const u32 numStates = t->rolesWithStateCount;

    void *role_state = getRoleState(tctxt->state);
    u32 idx = 0;
    int work_done = 0; // set to 1 if we actually process any roles
    u32 i = mmbit_sparse_iter_begin(role_state, numStates, &idx, it, s);

    fatbit_clear(handled_roles);

    for (; i != MMB_INVALID;
           i = mmbit_sparse_iter_next(role_state, numStates, i, &idx, it, s)) {
        DEBUG_PRINTF("pred state %u (iter idx=%u) is on\n", i, idx);
        const struct RoseIterMapping *iterMap = iterMapBase + idx;
        const struct RoseIterRole *roles = getByOffset(t, iterMap->offset);
        assert(ISALIGNED(roles));

        DEBUG_PRINTF("%u roles to consider\n", iterMap->count);
        for (u32 j = 0; j != iterMap->count; j++) {
            u32 role = roles[j].role;
            assert(role < t->roleCount);
            DEBUG_PRINTF("checking role %u, pred %u:\n", role, roles[j].pred);
            const struct RoseRole *tr = roleTable + role;

            if (fatbit_isset(handled_roles, t->roleCount, role)) {
                DEBUG_PRINTF("role %u already handled by the walk, skip\n",
                             role);
                continue;
            }

            // Special case: if this role is a trivial case (pred type simple)
            // we don't need to check any history and we already know the pred
            // role is on.
            if (tr->flags & ROSE_ROLE_PRED_SIMPLE) {
                DEBUG_PRINTF("pred type is simple, no need for further"
                             " checks\n");
            } else {
                assert(roles[j].pred < t->predCount);
                const struct RosePred *tp = predTable + roles[j].pred;
                if (!roseCheckPredHistory(tp, end)) {
                    continue;
                }
            }

            /* mark role as handled so we don't touch it again in this walk */
            fatbit_set(handled_roles, t->roleCount, role);

            hwlmcb_rv_t rv = roseHandleRole(t, tr, end, tctxt,
                                            0 /* in_anchored */, &work_done);
            if (rv == HWLM_TERMINATE_MATCHING) {
                return HWLM_TERMINATE_MATCHING;
            }
        }
    }

    // If we've actually handled any roles, we might need to apply this
    // literal's squash mask to our groups as well.
    if (work_done && tl->squashesGroup) {
        roseSquashGroup(tctxt, tl);
    }

    return HWLM_CONTINUE_MATCHING;
}

// Check that the predecessor bounds are satisfied for a root role with special
// requirements (anchored, or unanchored but with preceding dots).
static rose_inline
char roseCheckRootBounds(const struct RoseEngine *t, const struct RoseRole *tr,
                         u64a end) {
    assert(tr->predOffset != ROSE_OFFSET_INVALID);
    const struct RosePred *tp = getPredTable(t) + tr->predOffset;
    assert(tp->role == MO_INVALID_IDX);

    // Check history. We only use a subset of our history types for root or
    // anchored root roles.
    assert(tp->historyCheck == ROSE_ROLE_HISTORY_NONE ||
           tp->historyCheck == ROSE_ROLE_HISTORY_ANCH);

    return roseCheckPredHistory(tp, end);
}

// Walk the set of root roles (roles with depth 1) associated with this literal
// and set them on.
static really_inline
char roseWalkRootRoles_i(const struct RoseEngine *t,
                         const struct RoseLiteral *tl, u64a end,
                         struct RoseContext *tctxt, char in_anchored) {
    /* main entry point ensures that there is at least two root roles */
    int work_done = 0;

    assert(tl->rootRoleOffset + tl->rootRoleCount <= t->rootRoleCount);
    assert(tl->rootRoleCount > 1);

    const u32 *rootRole = getRootRoleTable(t) + tl->rootRoleOffset;
    const u32 *rootRoleEnd = rootRole + tl->rootRoleCount;
    for (; rootRole < rootRoleEnd; rootRole++) {
        u32 role_offset = *rootRole;
        const struct RoseRole *tr = getRoleByOffset(t, role_offset);

        if (!in_anchored && (tr->flags & ROSE_ROLE_PRED_ROOT)
            && !roseCheckRootBounds(t, tr, end)) {
            continue;
        }

        if (roseHandleRole(t, tr, end, tctxt, in_anchored, &work_done)
            == HWLM_TERMINATE_MATCHING) {
            return 0;
        }
    };

    // If we've actually handled any roles, we might need to apply this
    // literal's squash mask to our groups as well.
    if (work_done && tl->squashesGroup) {
        roseSquashGroup(tctxt, tl);
    }

    return 1;
}

static never_inline
char roseWalkRootRoles_A(const struct RoseEngine *t,
                         const struct RoseLiteral *tl, u64a end,
                         struct RoseContext *tctxt) {
    return roseWalkRootRoles_i(t, tl, end, tctxt, 1);
}

static never_inline
char roseWalkRootRoles_N(const struct RoseEngine *t,
                         const struct RoseLiteral *tl, u64a end,
                         struct RoseContext *tctxt) {
    return roseWalkRootRoles_i(t, tl, end, tctxt, 0);
}

static really_inline
char roseWalkRootRoles_i1(const struct RoseEngine *t,
                          const struct RoseLiteral *tl, u64a end,
                          struct RoseContext *tctxt, char in_anchored) {
    /* main entry point ensures that there is exactly one root role */
    int work_done = 0;
    u32 role_offset = tl->rootRoleOffset;
    const struct RoseRole *tr = getRoleByOffset(t, role_offset);

    if (!in_anchored && (tr->flags & ROSE_ROLE_PRED_ROOT)
        && !roseCheckRootBounds(t, tr, end)) {
        return 1;
    }

    hwlmcb_rv_t rv = roseHandleRole(t, tr, end, tctxt, in_anchored, &work_done);
    if (rv == HWLM_TERMINATE_MATCHING) {
        return 0;
    }

    // If we've actually handled any roles, we might need to apply this
    // literal's squash mask to our groups as well.
    if (work_done && tl->squashesGroup) {
        roseSquashGroup(tctxt, tl);
    }

    return 1;
}

static never_inline
char roseWalkRootRoles_A1(const struct RoseEngine *t,
                          const struct RoseLiteral *tl, u64a end,
                          struct RoseContext *tctxt) {
    return roseWalkRootRoles_i1(t, tl, end, tctxt, 1);
}

static never_inline
char roseWalkRootRoles_N1(const struct RoseEngine *t,
                          const struct RoseLiteral *tl, u64a end,
                          struct RoseContext *tctxt) {
    return roseWalkRootRoles_i1(t, tl, end, tctxt, 0);
}


static really_inline
char roseWalkRootRoles(const struct RoseEngine *t,
                       const struct RoseLiteral *tl, u64a end,
                       struct RoseContext *tctxt, char in_anchored,
                       char in_anch_playback) {
    DEBUG_PRINTF("literal has %u root roles\n", tl->rootRoleCount);

    assert(!in_anch_playback || tl->rootRoleCount);
    if (!in_anch_playback && !tl->rootRoleCount) {
        return 1;
    }

    if (in_anchored) {
        if (tl->rootRoleCount == 1) {
            return roseWalkRootRoles_A1(t, tl, end, tctxt);
        } else {
            return roseWalkRootRoles_A(t, tl, end, tctxt);
        }
    } else {
        if (tl->rootRoleCount == 1) {
            return roseWalkRootRoles_N1(t, tl, end, tctxt);
        } else {
            return roseWalkRootRoles_N(t, tl, end, tctxt);
        }
    }
}

void roseSidecarCallback(UNUSED u64a offset, u32 side_id, void *context) {
    struct RoseContext *tctxt = context;
    const struct RoseEngine *t = tctxt->t;

    DEBUG_PRINTF("SIDE MATCH side_id=%u offset=[%llu, %llu]\n", side_id,
                 offset, offset + 1);
    assert(side_id < t->sideCount);

    const struct RoseSide *side = &getSideEntryTable(t)[side_id];
    roseSquashStates(t, side, tctxt);

    DEBUG_PRINTF("done with sc\n");
}

/* handles catchup, som, cb, etc */
static really_inline
hwlmcb_rv_t roseHandleReport(const struct RoseEngine *t, u8 *state,
                             struct RoseContext *tctxt, ReportID id, u64a offset,
                             char in_anchored) {
    const struct internal_report *ri = getInternalReport(t, id);

    if (ri) {
        // Mildly cheesy performance hack: if this report is already exhausted,
        // we can quash the match here.
        if (ri->ekey != INVALID_EKEY) {
            const struct hs_scratch *scratch = tctxtToScratch(tctxt);
            if (isExhausted(scratch->core_info.exhaustionVector, ri->ekey)) {
                DEBUG_PRINTF("eating exhausted match (report %u, ekey %u)\n",
                             ri->onmatch, ri->ekey);
                return HWLM_CONTINUE_MATCHING;
            }
        }

        if (isInternalSomReport(ri)) {
            return roseHandleSom(t, state, id, offset, tctxt, in_anchored);
        } else if (ri->type == INTERNAL_ROSE_CHAIN) {
            return roseCatchUpAndHandleChainMatch(t, state, id, offset, tctxt,
                                                  in_anchored);
        }
    }
    return roseHandleMatch(t, state, id, offset, tctxt, in_anchored);
}

static really_inline
hwlmcb_rv_t roseHandleAnchoredDirectReport(const struct RoseEngine *t,
                                           u8 *state, struct RoseContext *tctxt,
                                           u64a real_end, ReportID report) {
    DEBUG_PRINTF("direct report %u, real_end=%llu\n", report, real_end);

    if (real_end > t->maxSafeAnchoredDROffset) {
        DEBUG_PRINTF("match in overlapped anchored region --> stash\n");
        recordAnchoredMatch(tctxt, report, real_end);
        return HWLM_CONTINUE_MATCHING;
    }

    return roseHandleReport(t, state, tctxt, report, real_end,
                            1 /* in anchored */);
}

int roseAnchoredCallback(u64a end, u32 id, void *ctx) {
    struct RoseContext *tctxt = ctx;
    const struct RoseEngine *t = tctxt->t;
    u8 *state = tctxt->state;
    struct core_info *ci = &tctxtToScratch(tctxt)->core_info;

    u64a real_end = ci->buf_offset + end; // index after last byte

    DEBUG_PRINTF("MATCH id=%u offsets=[???,%llu]\n", id, real_end);
    DEBUG_PRINTF("STATE depth=%u, groups=0x%016llx\n", tctxt->depth,
                 tctxt->groups);

    if (can_stop_matching(tctxtToScratch(tctxt))) {
        DEBUG_PRINTF("received a match when we're already dead!\n");
        return MO_HALT_MATCHING;
    }

    hwlmcb_rv_t rv = HWLM_CONTINUE_MATCHING;

    /* delayed literals need to be delivered before real literals; however
     * delayed literals only come from the floating table so if we are going
     * to deliver a literal here it must be too early for a delayed literal */

    /* no history checks from anchored region and we are before the flush
     * boundary */

    if (isLiteralMDR(id)) {
        // Multi-direct report, list of reports indexed by the ID.
        u32 mdr_offset = id & ~LITERAL_MDR_FLAG;
        const ReportID *report =
            (const ReportID *)((const char *)t + t->multidirectOffset) +
            mdr_offset;
        for (; *report != MO_INVALID_IDX; report++) {
            rv = roseHandleAnchoredDirectReport(t, state, tctxt, real_end,
                                                *report);
            if (rv == HWLM_TERMINATE_MATCHING) {
                return MO_HALT_MATCHING;
            }
        }
        return MO_CONTINUE_MATCHING;
    } else if (isLiteralDR(id)) {
        // Single direct report.
        ReportID report = literalToReport(id);
        rv = roseHandleAnchoredDirectReport(t, state, tctxt, real_end, report);
        if (rv == HWLM_TERMINATE_MATCHING) {
            return MO_HALT_MATCHING;
        }
        return MO_CONTINUE_MATCHING;
    }

    assert(id < t->literalCount);
    const struct RoseLiteral *tl = &getLiteralTable(t)[id];
    assert(tl->rootRoleCount > 0);
    assert(!tl->delay_mask);

    DEBUG_PRINTF("literal id=%u, minDepth=%u, groups=0x%016llx, "
                 "rootRoleCount=%u\n",
                 id, tl->minDepth, tl->groups, tl->rootRoleCount);

    if (real_end <= t->floatingMinLiteralMatchOffset) {
        roseFlushLastByteHistory(t, state, real_end, tctxt);
        tctxt->lastEndOffset = real_end;
    }

    if (tl->requires_side
        && real_end <= t->floatingMinLiteralMatchOffset) {
        /* Catch up the sidecar to the literal location. This means that all
         * squashing events are delivered before any 'side involved' literal
         * matches at a given location. */

        catchup_sidecar(tctxt, real_end);
    }

    /* anchored literals are root only */
    if (!roseWalkRootRoles(t, tl, real_end, tctxt, 1, 0)) {
        rv = HWLM_TERMINATE_MATCHING;
    }

    DEBUG_PRINTF("DONE depth=%u, groups=0x%016llx\n", tctxt->depth,
                 tctxt->groups);

    if (rv == HWLM_TERMINATE_MATCHING) {
        assert(can_stop_matching(tctxtToScratch(tctxt)));
        DEBUG_PRINTF("caller requested termination\n");
        return MO_HALT_MATCHING;
    }

    if (real_end > t->floatingMinLiteralMatchOffset) {
        recordAnchoredLiteralMatch(tctxt, id, real_end);
    }

    return MO_CONTINUE_MATCHING;
}

// Rose match-processing workhorse
/* assumes not in_anchored */
static really_inline
hwlmcb_rv_t roseProcessMatch_i(const struct RoseEngine *t, u64a end, u32 id,
                               struct RoseContext *tctxt, char do_group_check,
                               char in_delay_play, char in_anch_playback) {
    /* assert(!tctxt->in_anchored); */
    u8 *state = tctxt->state;

    DEBUG_PRINTF("id=%u\n", id);

    if (!in_anch_playback && !in_delay_play) {
        if (isLiteralMDR(id)) {
            // Multi-direct report, list of reports indexed by the ID.
            u32 mdr_offset = id & ~LITERAL_MDR_FLAG;
            const ReportID *report =
                (const ReportID *)((const char *)t + t->multidirectOffset) +
                mdr_offset;
            for (; *report != MO_INVALID_IDX; report++) {
                DEBUG_PRINTF("handle multi-direct report %u\n", *report);
                hwlmcb_rv_t rv = roseHandleReport(t, state, tctxt, *report, end,
                                                  0 /* in anchored */);
                if (rv == HWLM_TERMINATE_MATCHING) {
                    return HWLM_TERMINATE_MATCHING;
                }
            }
            return HWLM_CONTINUE_MATCHING;
        } else if (isLiteralDR(id)) {
            // Single direct report.
            ReportID report = literalToReport(id);
            DEBUG_PRINTF("handle direct report %u\n", report);
            return roseHandleReport(t, state, tctxt, report, end,
                                    0 /* in anchored */);
        }
    }

    assert(id < t->literalCount);
    const struct RoseLiteral *tl = &getLiteralTable(t)[id];
    DEBUG_PRINTF("lit id=%u, minDepth=%u, groups=0x%016llx, rootRoleCount=%u\n",
                 id, tl->minDepth, tl->groups, tl->rootRoleCount);

    if (do_group_check && !(tl->groups & tctxt->groups)) {
        DEBUG_PRINTF("IGNORE: none of this literal's groups are set.\n");
        return HWLM_CONTINUE_MATCHING;
    }

    assert(!in_delay_play || !tl->delay_mask);
    if (!in_delay_play) {
        pushDelayedMatches(tl, end, tctxt);
    }

    if (end < t->floatingMinLiteralMatchOffset) {
        DEBUG_PRINTF("too soon\n");
        assert(!in_delay_play); /* should not have been enqueued */
        /* continuing on may result in pushing global time back */
        return HWLM_CONTINUE_MATCHING;
    }

    // If the current literal requires sidecar support, run to current
    // location.
    if (tl->requires_side) {
        /* Catch up the sidecar to the literal location. This means that all
         * squashing events are delivered before any 'side involved' literal
         * matches at a given location. */

        if (tl->rootRoleCount || tl->minDepth <= tctxt->depth) {
            catchup_sidecar(tctxt, end);
        }
    }

    if (tl->minDepth > tctxt->depth) {
        DEBUG_PRINTF("IGNORE: minDepth=%u > %u\n", tl->minDepth, tctxt->depth);
        goto root_roles;
    }

    /* the depth checks will normally prevent roles without a spare iterator
     * from reaching here (root roles) (and only root roles should be seen
     * during anch play back). */
    assert(tl->iterOffset == ROSE_OFFSET_INVALID || !in_anch_playback);
    if (tl->iterOffset != ROSE_OFFSET_INVALID && !in_anch_playback) {
        hwlmcb_rv_t rv = roseWalkSparseIterator(t, tl, end, tctxt);

        if (rv == HWLM_TERMINATE_MATCHING) {
            return HWLM_TERMINATE_MATCHING;
        }
    }

root_roles:
    // Process "root roles", i.e. depth 1 roles for this literal
    if (!roseWalkRootRoles(t, tl, end, tctxt, 0 /* in_anchored */,
                           in_anch_playback)) {
        return HWLM_TERMINATE_MATCHING;
    }

    return HWLM_CONTINUE_MATCHING;
}


static never_inline
hwlmcb_rv_t roseProcessDelayedMatch(const struct RoseEngine *t, u64a end, u32 id,
                                    struct RoseContext *tctxt) {
    return roseProcessMatch_i(t, end, id, tctxt, 1, 1, 0);
}

static never_inline
hwlmcb_rv_t roseProcessDelayedAnchoredMatch(const struct RoseEngine *t, u64a end,
                                            u32 id, struct RoseContext *tctxt) {
    return roseProcessMatch_i(t, end, id, tctxt, 0, 0, 1);
}

static really_inline
hwlmcb_rv_t roseProcessMainMatch(const struct RoseEngine *t, u64a end, u32 id,
                                 struct RoseContext *tctxt) {
    return roseProcessMatch_i(t, end, id, tctxt, 1, 0, 0);
}

static rose_inline
hwlmcb_rv_t playDelaySlot(struct RoseContext *tctxt, const u8 *delaySlotBase,
                          size_t delaySlotSize, u32 vicIndex, u64a offset) {
    /* assert(!tctxt->in_anchored); */
    assert(vicIndex < DELAY_SLOT_COUNT);
    const u8 *vicSlot = delaySlotBase + delaySlotSize * vicIndex;
    u32 delay_count = tctxt->t->delay_count;

    if (offset < tctxt->t->floatingMinLiteralMatchOffset) {
        DEBUG_PRINTF("too soon\n");
        return HWLM_CONTINUE_MATCHING;
    }

    roseFlushLastByteHistory(tctxt->t, tctxt->state, offset, tctxt);
    tctxt->lastEndOffset = offset;

    for (u32 it = mmbit_iterate(vicSlot, delay_count, MMB_INVALID);
         it != MMB_INVALID; it = mmbit_iterate(vicSlot, delay_count, it)) {
        u32 literal_id = tctxt->t->delay_base_id + it;

        UNUSED rose_group old_groups = tctxt->groups;

        DEBUG_PRINTF("DELAYED MATCH id=%u offset=%llu\n", literal_id, offset);
        hwlmcb_rv_t rv = roseProcessDelayedMatch(tctxt->t, offset, literal_id,
                                                 tctxt);
        DEBUG_PRINTF("DONE depth=%u, groups=0x%016llx\n", tctxt->depth,
                     tctxt->groups);

        /* delayed literals can't safely set groups, squashing may from side.
         * However we may be setting groups that successors already have
         * worked out that we don't need to match the group */
        DEBUG_PRINTF("groups in %016llx out %016llx\n", old_groups,
                     tctxt->groups);

        if (rv == HWLM_TERMINATE_MATCHING) {
            return HWLM_TERMINATE_MATCHING;
        }
    }

    return HWLM_CONTINUE_MATCHING;
}

static really_inline
hwlmcb_rv_t flushAnchoredLiteralAtLoc(struct RoseContext *tctxt, u32 curr_loc) {
    u8 *curr_row = getAnchoredLiteralLog(tctxtToScratch(tctxt))[curr_loc - 1];
    u32 region_width = tctxt->t->anchored_count;

    DEBUG_PRINTF("report matches at curr loc\n");
    for (u32 it = mmbit_iterate(curr_row, region_width, MMB_INVALID);
         it != MMB_INVALID; it = mmbit_iterate(curr_row, region_width, it)) {
        DEBUG_PRINTF("it = %u/%u\n", it, region_width);
        u32 literal_id = tctxt->t->anchored_base_id + it;

        rose_group old_groups = tctxt->groups;
        DEBUG_PRINTF("ANCH REPLAY MATCH id=%u offset=%u\n", literal_id,
                     curr_loc);
        hwlmcb_rv_t rv = roseProcessDelayedAnchoredMatch(tctxt->t, curr_loc,
                                                         literal_id, tctxt);
        DEBUG_PRINTF("DONE depth=%u, groups=0x%016llx\n", tctxt->depth,
                     tctxt->groups);

        /* anchored literals can't safely set groups, squashing may from
         * side. However we may be setting groups that successors already
         * have worked out that we don't need to match the group */
        DEBUG_PRINTF("groups in %016llx out %016llx\n", old_groups,
                     tctxt->groups);
        tctxt->groups &= old_groups;

        if (rv == HWLM_TERMINATE_MATCHING) {
            return HWLM_TERMINATE_MATCHING;
        }
    }

    /* clear row; does not invalidate iteration */
    struct hs_scratch *scratch = tctxtToScratch(tctxt);
    bf64_unset(&scratch->al_log_sum, curr_loc - 1);

    return HWLM_CONTINUE_MATCHING;
}

static really_inline
u32 anchored_it_begin(struct RoseContext *tctxt) {
    struct hs_scratch *scratch = tctxtToScratch(tctxt);
    if (tctxt->lastEndOffset >= scratch->anchored_literal_region_len) {
        return MMB_INVALID;
    }
    u32 begin = tctxt->lastEndOffset;
    begin--;

    return bf64_iterate(tctxtToScratch(tctxt)->al_log_sum, begin);
}

static really_inline
hwlmcb_rv_t flushAnchoredLiterals(struct RoseContext *tctxt,
                                  u32 *anchored_it_param, u64a to_off) {
    struct hs_scratch *scratch = tctxtToScratch(tctxt);
    u32 anchored_it = *anchored_it_param;
    /* catch up any remaining anchored matches */
    for (; anchored_it != MMB_INVALID && anchored_it < to_off;
         anchored_it = bf64_iterate(scratch->al_log_sum, anchored_it)) {
        assert(anchored_it < scratch->anchored_literal_region_len);
        DEBUG_PRINTF("loc_it = %u\n", anchored_it);
        u32 curr_off = anchored_it + 1;
        roseFlushLastByteHistory(tctxt->t, tctxt->state, curr_off, tctxt);
        tctxt->lastEndOffset = curr_off;

        if (flushAnchoredLiteralAtLoc(tctxt, curr_off)
            == HWLM_TERMINATE_MATCHING) {
            return HWLM_TERMINATE_MATCHING;
        }
    }

    *anchored_it_param = anchored_it;
    return HWLM_CONTINUE_MATCHING;
}

static really_inline
hwlmcb_rv_t playVictims(struct RoseContext *tctxt, u32 *anchored_it,
                        u64a lastEnd, u64a victimDelaySlots, u8 *delaySlotBase,
                        size_t delaySlotSize) {
    /* assert (!tctxt->in_anchored); */

    while (victimDelaySlots) {
        u32 vic = findAndClearLSB_64(&victimDelaySlots);
        DEBUG_PRINTF("vic = %u\n", vic);
        u64a vicOffset = vic + (lastEnd & ~(u64a)DELAY_MASK);

        if (flushAnchoredLiterals(tctxt, anchored_it, vicOffset)
            == HWLM_TERMINATE_MATCHING) {
            return HWLM_TERMINATE_MATCHING;
        }

        if (playDelaySlot(tctxt, delaySlotBase, delaySlotSize,
                          vic % DELAY_SLOT_COUNT, vicOffset)
            == HWLM_TERMINATE_MATCHING) {
            return HWLM_TERMINATE_MATCHING;
        }
    }

    return HWLM_CONTINUE_MATCHING;
}

/* call flushQueuedLiterals instead */
hwlmcb_rv_t flushQueuedLiterals_i(struct RoseContext *tctxt, u64a currEnd) {
    /* assert(!tctxt->in_anchored); */
    u64a lastEnd = tctxt->delayLastEndOffset;
    DEBUG_PRINTF("flushing backed up matches @%llu up from %llu\n", currEnd,
                 lastEnd);

    assert(currEnd != lastEnd); /* checked in main entry point */

    u32 anchored_it = anchored_it_begin(tctxt);

    if (!tctxt->filledDelayedSlots) {
        DEBUG_PRINTF("no delayed, no flush\n");
        goto anchored_leftovers;
    }

    {
        u8 *delaySlotBase = getDelaySlots(tctxtToScratch(tctxt));
        size_t delaySlotSize = tctxt->t->delay_slot_size;

        u32 lastIndex = lastEnd & DELAY_MASK;
        u32 currIndex = currEnd & DELAY_MASK;

        int wrapped = (lastEnd | DELAY_MASK) < currEnd;

        u64a victimDelaySlots; /* needs to be twice as wide as the number of
                                * slots. */

        DEBUG_PRINTF("hello %08x\n", tctxt->filledDelayedSlots);
        if (!wrapped) {
            victimDelaySlots = tctxt->filledDelayedSlots;

            DEBUG_PRINTF("unwrapped %016llx %08x\n", victimDelaySlots,
                         tctxt->filledDelayedSlots);
            /* index vars < 32 so 64bit shifts are safe */

            /* clear all slots at last index and below, */
            victimDelaySlots &= ~((1LLU << (lastIndex + 1)) - 1);

            /* clear all slots above curr index */
            victimDelaySlots &= (1LLU << (currIndex + 1)) - 1;

            tctxt->filledDelayedSlots &= ~victimDelaySlots;

            DEBUG_PRINTF("unwrapped %016llx %08x\n", victimDelaySlots,
                         tctxt->filledDelayedSlots);
        } else {
            DEBUG_PRINTF("wrapped %08x\n", tctxt->filledDelayedSlots);

            /* 1st half: clear all slots at last index and below, */
            u64a first_half = tctxt->filledDelayedSlots;
            first_half &= ~((1ULL << (lastIndex + 1)) - 1);
            tctxt->filledDelayedSlots &= (1ULL << (lastIndex + 1)) - 1;

            u64a second_half = tctxt->filledDelayedSlots;

            if (currEnd > lastEnd + DELAY_SLOT_COUNT) {
                /* 2nd half: clear all slots above last index */
                second_half &= (1ULL << (lastIndex + 1)) - 1;
            } else {
                /* 2nd half: clear all slots above curr index */
                second_half &= (1ULL << (currIndex + 1)) - 1;
            }
            tctxt->filledDelayedSlots &= ~second_half;

            victimDelaySlots = first_half | (second_half << DELAY_SLOT_COUNT);

            DEBUG_PRINTF("-- %016llx %016llx = %016llx (li %u)\n", first_half,
                         second_half, victimDelaySlots, lastIndex);
        }

        if (playVictims(tctxt, &anchored_it, lastEnd, victimDelaySlots,
                        delaySlotBase, delaySlotSize)
            == HWLM_TERMINATE_MATCHING) {
            return HWLM_TERMINATE_MATCHING;
        }
    }

anchored_leftovers:;
    hwlmcb_rv_t rv = flushAnchoredLiterals(tctxt, &anchored_it, currEnd);
    tctxt->delayLastEndOffset = currEnd;
    return rv;
}

hwlmcb_rv_t roseCallback(size_t start, size_t end, u32 id, void *ctxt) {
    struct RoseContext *tctx = ctxt;
    u64a real_end = end + tctx->lit_offset_adjust;

#if defined(DEBUG)
    struct core_info *ci = &tctxtToScratch(tctx)->core_info;
    DEBUG_PRINTF("MATCH id=%u offsets=[%llu,%llu]: ", id,
                 start + tctx->lit_offset_adjust, real_end);
    printMatch(ci, start + tctx->lit_offset_adjust, real_end);
    printf("\n");
#endif
    DEBUG_PRINTF("last end %llu\n", tctx->lastEndOffset);

    DEBUG_PRINTF("STATE depth=%u, groups=0x%016llx\n", tctx->depth,
                 tctx->groups);

    if (can_stop_matching(tctxtToScratch(tctx))) {
        DEBUG_PRINTF("received a match when we're already dead!\n");
        return HWLM_TERMINATE_MATCHING;
    }

    if (id < tctx->t->nonbenefits_base_id
        && !roseCheckLiteralBenefits(real_end, end - start + 1, id, tctx)) {
        return tctx->groups;
    }

    hwlmcb_rv_t rv = flushQueuedLiterals(tctx, real_end);
    /* flushDelayed may have advanced tctx->lastEndOffset */

    if (real_end >= tctx->t->floatingMinLiteralMatchOffset) {
        roseFlushLastByteHistory(tctx->t, tctx->state, real_end, tctx);
        tctx->lastEndOffset = real_end;
    }

    if (rv == HWLM_TERMINATE_MATCHING) {
        return HWLM_TERMINATE_MATCHING;
    }

    rv = roseProcessMainMatch(tctx->t, real_end, id, tctx);

    DEBUG_PRINTF("DONE depth=%hhu, groups=0x%016llx\n", tctx->depth,
                 tctx->groups);

    if (rv != HWLM_TERMINATE_MATCHING) {
        return tctx->groups;
    }

    assert(can_stop_matching(tctxtToScratch(tctx)));
    DEBUG_PRINTF("user requested halt\n");
    return HWLM_TERMINATE_MATCHING;
}

// Specialised cut-down roseCallback for running ROSE_EVENT "literals", like the
// EOD one.
void roseRunEvent(size_t end, u32 id, struct RoseContext *tctxt) {
    const struct RoseEngine *t = tctxt->t;
    struct core_info *ci = &tctxtToScratch(tctxt)->core_info;
    u64a real_end = ci->buf_offset - ci->hlen + end;

    DEBUG_PRINTF("EVENT id=%u offset=%llu\n", id, real_end);

    // Caller should guard against broken stream.
    assert(!can_stop_matching(tctxtToScratch(tctxt)));

    // Shouldn't be here if we're a real literal with benefits.
    assert(id >= t->nonbenefits_base_id);

    // At the moment, this path is only used for the EOD event.
    assert(id == t->eodLiteralId);

    // There should be no pending delayed literals.
    assert(!tctxt->filledDelayedSlots);

    // Note: we throw away the return value.
    roseProcessMatch_i(t, real_end, id, tctxt, 0, 0, 0);

    DEBUG_PRINTF("DONE depth=%hhu, groups=0x%016llx\n", tctxt->depth,
                 tctxt->groups);
}
