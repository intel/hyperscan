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
#include "program_runtime.h"
#include "rose_program.h"
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

    DEBUG_PRINTF("STATE groups=0x%016llx\n", tctx->groups);

    if (isLiteralDR(id)) {
        return tctx->groups;
    }

    if (id < t->nonbenefits_base_id
        && !roseCheckLiteralBenefits(real_end, end - start + 1, id, tctx)) {
        return tctx->groups;
    }

    assert(id < t->literalCount);
    const struct RoseLiteral *tl = &getLiteralTable(t)[id];

    DEBUG_PRINTF("literal id=%u, groups=0x%016llx\n", id, tl->groups);

    pushDelayedMatches(tl, real_end, tctx);

    /* we are just repopulating the delay queue, groups should be
     * already set from the original scan. */

    return tctx->groups;
}

static really_inline
hwlmcb_rv_t ensureMpvQueueFlushed(const struct RoseEngine *t,
                                  struct hs_scratch *scratch, u32 qi, s64a loc,
                                  char in_anchored, char in_chained) {
    return ensureQueueFlushed_i(t, scratch, qi, loc, 1, in_anchored,
                                in_chained);
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
    DEBUG_PRINTF("STATE groups=0x%016llx\n", tctxt->groups);

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
    assert(tl->programOffset);
    assert(!tl->delay_mask);

    DEBUG_PRINTF("literal id=%u, groups=0x%016llx\n", id, tl->groups);

    if (real_end <= t->floatingMinLiteralMatchOffset) {
        roseFlushLastByteHistory(t, state, real_end, tctxt);
        tctxt->lastEndOffset = real_end;
    }

    int work_done = 0;
    if (roseRunProgram(t, tl->programOffset, real_end, tctxt, 1, &work_done) ==
        HWLM_TERMINATE_MATCHING) {
        assert(can_stop_matching(tctxtToScratch(tctxt)));
        DEBUG_PRINTF("caller requested termination\n");
        return MO_HALT_MATCHING;
    }

    // If we've actually handled any roles, we might need to apply this
    // literal's squash mask to our groups as well.
    if (work_done && tl->squashesGroup) {
        roseSquashGroup(tctxt, tl);
    }

    DEBUG_PRINTF("DONE groups=0x%016llx\n", tctxt->groups);

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
    DEBUG_PRINTF("lit id=%u, groups=0x%016llx\n", id, tl->groups);

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

    int work_done = 0;

    if (tl->programOffset) {
        DEBUG_PRINTF("running program at %u\n", tl->programOffset);
        if (roseRunProgram(t, tl->programOffset, end, tctxt, 0, &work_done) ==
            HWLM_TERMINATE_MATCHING) {
            return HWLM_TERMINATE_MATCHING;
        }

    }

    // If we've actually handled any roles, we might need to apply this
    // literal's squash mask to our groups as well.
    if (work_done && tl->squashesGroup) {
        roseSquashGroup(tctxt, tl);
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
        DEBUG_PRINTF("DONE groups=0x%016llx\n", tctxt->groups);

        /* delayed literals can't safely set groups.
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
        DEBUG_PRINTF("DONE groups=0x%016llx\n", tctxt->groups);

        /* anchored literals can't safely set groups.
         * However we may be setting groups that successors already
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

    DEBUG_PRINTF("STATE groups=0x%016llx\n", tctx->groups);

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

    DEBUG_PRINTF("DONE groups=0x%016llx\n", tctx->groups);

    if (rv != HWLM_TERMINATE_MATCHING) {
        return tctx->groups;
    }

    assert(can_stop_matching(tctxtToScratch(tctx)));
    DEBUG_PRINTF("user requested halt\n");
    return HWLM_TERMINATE_MATCHING;
}
