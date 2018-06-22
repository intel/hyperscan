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
 */

#include "catchup.h"
#include "match.h"
#include "program_runtime.h"
#include "rose.h"
#include "nfa/nfa_rev_api.h"
#include "nfa/mpv.h"
#include "som/som_runtime.h"
#include "util/fatbit.h"
#include "report.h"

typedef struct queue_match PQ_T;
#define PQ_COMP(pqc_items, a, b) ((pqc_items)[a].loc < (pqc_items)[b].loc)
#define PQ_COMP_B(pqc_items, a, b_fixed) ((pqc_items)[a].loc < (b_fixed).loc)

#include "util/pqueue.h"

static really_inline
int roseNfaRunProgram(const struct RoseEngine *rose, struct hs_scratch *scratch,
                      u64a som, u64a offset, ReportID id, const char from_mpv) {
    const u32 program = id;
    u8 flags = ROSE_PROG_FLAG_IN_CATCHUP;
    if (from_mpv) {
        flags |= ROSE_PROG_FLAG_FROM_MPV;
    }

    roseRunProgram(rose, scratch, program, som, offset, flags);

    return can_stop_matching(scratch) ? MO_HALT_MATCHING : MO_CONTINUE_MATCHING;
}

static rose_inline
char roseSuffixInfoIsExhausted(const struct RoseEngine *rose,
                               const struct NfaInfo *info,
                               const char *exhausted) {
    if (!info->ekeyListOffset) {
        return 0;
    }

    DEBUG_PRINTF("check exhaustion -> start at %u\n", info->ekeyListOffset);

    /* INVALID_EKEY terminated list */
    const u32 *ekeys = getByOffset(rose, info->ekeyListOffset);
    while (*ekeys != INVALID_EKEY) {
        DEBUG_PRINTF("check %u\n", *ekeys);
        if (!isExhausted(rose, exhausted, *ekeys)) {
            DEBUG_PRINTF("not exhausted -> alive\n");
            return 0;
        }
        ++ekeys;
    }

    DEBUG_PRINTF("all ekeys exhausted -> dead\n");
    return 1;
}

static really_inline
char roseSuffixIsExhausted(const struct RoseEngine *rose, u32 qi,
                           const char *exhausted) {
    DEBUG_PRINTF("check queue %u\n", qi);
    const struct NfaInfo *info = getNfaInfoByQueue(rose, qi);
    return roseSuffixInfoIsExhausted(rose, info, exhausted);
}

static really_inline
void deactivateQueue(const struct RoseEngine *t, u8 *aa, u32 qi,
                     struct hs_scratch *scratch) {
    u32 aaCount = t->activeArrayCount;
    u32 qCount = t->queueCount;

    /* this is sailing close to the wind with regards to invalidating an
     * iteration. We are saved by the fact that unsetting does not clear the
     * summary bits -> the block under the gun remains valid
     */
    DEBUG_PRINTF("killing off zombie queue %u\n", qi);
    mmbit_unset(aa, aaCount, qi);
    fatbit_unset(scratch->aqa, qCount, qi);
}

static really_inline
void ensureQueueActive(const struct RoseEngine *t, u32 qi, u32 qCount,
                       struct mq *q, struct hs_scratch *scratch) {
    if (!fatbit_set(scratch->aqa, qCount, qi)) {
        DEBUG_PRINTF("initing %u\n", qi);
        initQueue(q, qi, t, scratch);
        loadStreamState(q->nfa, q, 0);
        pushQueueAt(q, 0, MQE_START, 0);
    }
}

static really_inline
void pq_replace_top_with(struct catchup_pq *pq,
                         UNUSED struct hs_scratch *scratch, u32 queue,
                         s64a loc) {
    DEBUG_PRINTF("inserting q%u in pq at %lld\n", queue, loc);
    struct queue_match temp = {
        .queue = queue,
        .loc = (size_t)loc
    };

    assert(loc > 0);
    assert(pq->qm_size);
    assert(loc <= (s64a)scratch->core_info.len);
    pq_replace_top(pq->qm, pq->qm_size, temp);
}

static really_inline
void pq_insert_with(struct catchup_pq *pq,
                    UNUSED struct hs_scratch *scratch, u32 queue, s64a loc) {
    DEBUG_PRINTF("inserting q%u in pq at %lld\n", queue, loc);
    struct queue_match temp = {
        .queue = queue,
        .loc = (size_t)loc
    };

    assert(loc > 0);
    assert(loc <= (s64a)scratch->core_info.len);
    pq_insert(pq->qm, pq->qm_size, temp);
    ++pq->qm_size;
}

static really_inline
void pq_pop_nice(struct catchup_pq *pq) {
    pq_pop(pq->qm, pq->qm_size);
    pq->qm_size--;
}

static really_inline
s64a pq_top_loc(struct catchup_pq *pq) {
    assert(pq->qm_size);
    return (s64a)pq_top(pq->qm)->loc;
}

/* requires that we are the top item on the pq */
static really_inline
hwlmcb_rv_t runExistingNfaToNextMatch(const struct RoseEngine *t, u32 qi,
                                      struct mq *q, s64a loc,
                                      struct hs_scratch *scratch, u8 *aa,
                                      char report_curr) {
    assert(pq_top(scratch->catchup_pq.qm)->queue == qi);
    assert(scratch->catchup_pq.qm_size);
    assert(!q->report_current);
    if (report_curr) {
        DEBUG_PRINTF("need to report matches\n");
        q->report_current = 1;
    }

    DEBUG_PRINTF("running queue from %u:%lld to %lld\n", q->cur, q_cur_loc(q),
                 loc);

    assert(q_cur_loc(q) <= loc);

    char alive = nfaQueueExecToMatch(q->nfa, q, loc);

    /* exit via gift shop */
    if (alive == MO_MATCHES_PENDING) {
        /* we have pending matches */
        assert(q_cur_loc(q) + scratch->core_info.buf_offset
               >= scratch->tctxt.minMatchOffset);
        pq_replace_top_with(&scratch->catchup_pq, scratch, qi, q_cur_loc(q));
        return HWLM_CONTINUE_MATCHING;
    } else if (!alive) {
        if (report_curr && can_stop_matching(scratch)) {
            DEBUG_PRINTF("bailing\n");
            return HWLM_TERMINATE_MATCHING;
        }

        deactivateQueue(t, aa, qi, scratch);
    } else if (q->cur == q->end) {
        DEBUG_PRINTF("queue %u finished, nfa lives\n", qi);
        q->cur = q->end = 0;
        pushQueueAt(q, 0, MQE_START, loc);
    } else {
        DEBUG_PRINTF("queue %u unfinished, nfa lives\n", qi);
        u32 i = 0;
        while (q->cur < q->end) {
            q->items[i] = q->items[q->cur++];
            DEBUG_PRINTF("q[%u] = %u:%lld\n", i, q->items[i].type,
                         q->items[i].location);
            assert(q->items[i].type != MQE_END);
            i++;
        }
        q->cur = 0;
        q->end = i;
    }

    pq_pop_nice(&scratch->catchup_pq);

    return HWLM_CONTINUE_MATCHING;
}

static really_inline
hwlmcb_rv_t runNewNfaToNextMatch(const struct RoseEngine *t, u32 qi,
                                 struct mq *q, s64a loc,
                                 struct hs_scratch *scratch, u8 *aa,
                                 s64a report_ok_loc) {
    assert(!q->report_current);
    DEBUG_PRINTF("running queue from %u:%lld to %lld\n", q->cur, q_cur_loc(q),
                 loc);
    DEBUG_PRINTF("min match offset %llu\n", scratch->tctxt.minMatchOffset);

    char alive = 1;

restart:
    alive = nfaQueueExecToMatch(q->nfa, q, loc);

    if (alive == MO_MATCHES_PENDING) {
        DEBUG_PRINTF("we have pending matches at %lld\n", q_cur_loc(q));
        s64a qcl = q_cur_loc(q);

        if (qcl == report_ok_loc) {
            assert(q->cur != q->end); /* the queue shouldn't be empty if there
                                       * are pending matches. */
            q->report_current = 1;
            DEBUG_PRINTF("restarting...\n");
            goto restart;
        }
        assert(qcl + scratch->core_info.buf_offset
               >= scratch->tctxt.minMatchOffset);
        pq_insert_with(&scratch->catchup_pq, scratch, qi, qcl);
    } else if (!alive) {
        if (can_stop_matching(scratch)) {
            DEBUG_PRINTF("bailing\n");
            return HWLM_TERMINATE_MATCHING;
        }

        deactivateQueue(t, aa, qi, scratch);
    } else if (q->cur == q->end) {
        DEBUG_PRINTF("queue %u finished, nfa lives\n", qi);
        q->cur = q->end = 0;
        pushQueueAt(q, 0, MQE_START, loc);
    } else {
        DEBUG_PRINTF("queue %u unfinished, nfa lives\n", qi);
        u32 i = 0;
        while (q->cur < q->end) {
            q->items[i] = q->items[q->cur++];
            DEBUG_PRINTF("q[%u] = %u:%lld\n", i, q->items[i].type,
                         q->items[i].location);
            assert(q->items[i].type != MQE_END);
            i++;
        }
        q->cur = 0;
        q->end = i;
    }

    return HWLM_CONTINUE_MATCHING;
}

/* for use by mpv (chained) only */
static
int roseNfaFinalBlastAdaptor(u64a start, u64a end, ReportID id, void *context) {
    struct hs_scratch *scratch = context;
    assert(scratch && scratch->magic == SCRATCH_MAGIC);
    const struct RoseEngine *t = scratch->core_info.rose;

    DEBUG_PRINTF("id=%u matched at [%llu,%llu]\n", id, start, end);

    int cb_rv = roseNfaRunProgram(t, scratch, start, end, id, 1);
    if (cb_rv == MO_HALT_MATCHING) {
        return MO_HALT_MATCHING;
    } else if (cb_rv == ROSE_CONTINUE_MATCHING_NO_EXHAUST) {
        return MO_CONTINUE_MATCHING;
    } else {
        assert(cb_rv == MO_CONTINUE_MATCHING);
        return !roseSuffixIsExhausted(t, 0,
                                      scratch->core_info.exhaustionVector);
    }
}

static really_inline
void ensureEnd(struct mq *q, UNUSED u32 qi, s64a final_loc) {
    DEBUG_PRINTF("ensure MQE_END %lld for queue %u\n", final_loc, qi);
    if (final_loc >= q_last_loc(q)) {
        /* TODO: ensure situation does not arise */
        assert(q_last_type(q) != MQE_END);
        pushQueueNoMerge(q, MQE_END, final_loc);
    }
}

static really_inline
hwlmcb_rv_t add_to_queue(const struct RoseEngine *t, struct mq *queues,
                         u32 qCount, u8 *aa, struct hs_scratch *scratch,
                         s64a loc, u32 qi, s64a report_ok_loc) {
    struct mq *q = queues + qi;
    const struct NfaInfo *info = getNfaInfoByQueue(t, qi);

    if (roseSuffixInfoIsExhausted(t, info,
                                  scratch->core_info.exhaustionVector)) {
        deactivateQueue(t, aa, qi, scratch);
        return HWLM_CONTINUE_MATCHING;
    }

    ensureQueueActive(t, qi, qCount, q, scratch);

    if (unlikely(loc < q_cur_loc(q))) {
        DEBUG_PRINTF("err loc %lld < location %lld\n", loc, q_cur_loc(q));
        return HWLM_CONTINUE_MATCHING;
    }

    ensureEnd(q, qi, loc);

    return runNewNfaToNextMatch(t, qi, q, loc, scratch, aa, report_ok_loc);
}

static really_inline
s64a findSecondPlace(struct catchup_pq *pq, s64a loc_limit) {
    assert(pq->qm_size); /* we are still on the pq and we are first place */

    /* we know (*cough* encapsulation) that second place will either be in
     * pq->qm[1] or pq->qm[2] (we are pq->qm[0]) */
    switch (pq->qm_size) {
    case 0:
    case 1:
        return (s64a)loc_limit;
    case 2:
        return MIN((s64a)pq->qm[1].loc, loc_limit);
    default:;
        size_t best = MIN(pq->qm[1].loc, pq->qm[2].loc);
        return MIN((s64a)best, loc_limit);
    }
}

hwlmcb_rv_t roseCatchUpMPV_i(const struct RoseEngine *t, s64a loc,
                             struct hs_scratch *scratch) {
    char *state = scratch->core_info.state;
    struct mq *queues = scratch->queues;
    u8 *aa = getActiveLeafArray(t, state);
    UNUSED u32 aaCount = t->activeArrayCount;
    u32 qCount = t->queueCount;

    /* find first match of each pending nfa */
    DEBUG_PRINTF("aa=%p, aaCount=%u\n", aa, aaCount);

    assert(t->outfixBeginQueue == 1);

    u32 qi = 0;
    assert(mmbit_isset(aa, aaCount, 0)); /* caller should have already bailed */

    DEBUG_PRINTF("catching up qi=%u to loc %lld\n", qi, loc);

    struct mq *q = queues + qi;
    const struct NfaInfo *info = getNfaInfoByQueue(t, qi);
    u64a mpv_exec_end = scratch->core_info.buf_offset + loc;
    u64a next_pos_match_loc = 0;

    if (roseSuffixInfoIsExhausted(t, info,
                                  scratch->core_info.exhaustionVector)) {
        deactivateQueue(t, aa, qi, scratch);
        goto done;
    }

    ensureQueueActive(t, qi, qCount, q, scratch);

    if (unlikely(loc < q_cur_loc(q))) {
        DEBUG_PRINTF("err loc %lld < location %lld\n", loc, q_cur_loc(q));
        goto done;
    }

    ensureEnd(q, qi, loc);

    assert(!q->report_current);

    q->cb = roseNfaFinalBlastAdaptor;

    DEBUG_PRINTF("queue %u blasting, %u/%u [%lld/%lld]\n",
                  qi, q->cur, q->end, q->items[q->cur].location, loc);

    scratch->tctxt.mpv_inactive = 0;

    /* we know it is going to be an mpv, skip the indirection */
    next_pos_match_loc = nfaExecMpv_QueueExecRaw(q->nfa, q, loc);
    assert(!q->report_current);

    if (!next_pos_match_loc) { /* 0 means dead */
        DEBUG_PRINTF("mpv is pining for the fjords\n");
        if (can_stop_matching(scratch)) {
            deactivateQueue(t, aa, qi, scratch);
            return HWLM_TERMINATE_MATCHING;
        }

        next_pos_match_loc = scratch->core_info.len;
        scratch->tctxt.mpv_inactive = 1;
    }

    if (q->cur == q->end) {
        DEBUG_PRINTF("queue %u finished, nfa lives [%lld]\n", qi, loc);
        q->cur = 0;
        q->end = 0;
        pushQueueAt(q, 0, MQE_START, loc);
    } else {
        DEBUG_PRINTF("queue %u not finished, nfa lives [%lld]\n", qi, loc);
    }

done:
    if (t->flushCombProgramOffset) {
        if (roseRunFlushCombProgram(t, scratch, mpv_exec_end)
                == HWLM_TERMINATE_MATCHING) {
            return HWLM_TERMINATE_MATCHING;
        }
    }
    updateMinMatchOffsetFromMpv(&scratch->tctxt, mpv_exec_end);
    scratch->tctxt.next_mpv_offset
        = MAX(next_pos_match_loc + scratch->core_info.buf_offset,
              mpv_exec_end + 1);

    DEBUG_PRINTF("next match loc %lld (off %llu)\n", next_pos_match_loc,
                  scratch->tctxt.next_mpv_offset);
    return can_stop_matching(scratch) ? HWLM_TERMINATE_MATCHING
                                      : HWLM_CONTINUE_MATCHING;
}

static really_inline
char in_mpv(const struct RoseEngine *rose, const struct hs_scratch *scratch) {
    const struct RoseContext *tctxt = &scratch->tctxt;
    assert(tctxt->curr_qi < rose->queueCount);
    if (tctxt->curr_qi < rose->outfixBeginQueue) {
        assert(getNfaByQueue(rose, tctxt->curr_qi)->type == MPV_NFA);
        return 1;
    }
    return 0;
}

static
int roseNfaBlastAdaptor(u64a start, u64a end, ReportID id, void *context) {
    struct hs_scratch *scratch = context;
    assert(scratch && scratch->magic == SCRATCH_MAGIC);
    const struct RoseEngine *t = scratch->core_info.rose;

    DEBUG_PRINTF("id=%u matched at [%llu,%llu]\n", id, start, end);

    const char from_mpv = in_mpv(t, scratch);
    int cb_rv = roseNfaRunProgram(t, scratch, start, end, id, from_mpv);
    if (cb_rv == MO_HALT_MATCHING) {
        return MO_HALT_MATCHING;
    } else if (cb_rv == ROSE_CONTINUE_MATCHING_NO_EXHAUST) {
        return MO_CONTINUE_MATCHING;
    } else {
        assert(cb_rv == MO_CONTINUE_MATCHING);
        return !roseSuffixIsExhausted(t, scratch->tctxt.curr_qi,
                                      scratch->core_info.exhaustionVector);
    }
}

int roseNfaAdaptor(u64a start, u64a end, ReportID id, void *context) {
    struct hs_scratch *scratch = context;
    assert(scratch && scratch->magic == SCRATCH_MAGIC);

    DEBUG_PRINTF("id=%u matched at [%llu,%llu]\n", id, start, end);

    /* must be a external report as haig cannot directly participate in chain */
    return roseNfaRunProgram(scratch->core_info.rose, scratch, start, end, id,
                             0);
}

static really_inline
char blast_queue(struct hs_scratch *scratch, struct mq *q, u32 qi, s64a to_loc,
                 char report_current) {
    scratch->tctxt.curr_qi = qi;
    q->cb = roseNfaBlastAdaptor;
    q->report_current = report_current;
    DEBUG_PRINTF("queue %u blasting, %u/%u [%lld/%lld]\n", qi, q->cur, q->end,
                 q_cur_loc(q), to_loc);
    char alive = nfaQueueExec(q->nfa, q, to_loc);
    q->cb = roseNfaAdaptor;
    assert(!q->report_current);

    return alive;
}

static really_inline
hwlmcb_rv_t buildSufPQ_final(const struct RoseEngine *t, s64a report_ok_loc,
                             s64a second_place_loc, s64a final_loc,
                             struct hs_scratch *scratch, u8 *aa, u32 a_qi) {
    struct mq *q = scratch->queues + a_qi;
    const struct NfaInfo *info = getNfaInfoByQueue(t, a_qi);
    DEBUG_PRINTF("blasting qi=%u to %lld [final %lld]\n", a_qi, second_place_loc,
                 final_loc);

    if (roseSuffixInfoIsExhausted(t, info,
                                  scratch->core_info.exhaustionVector)) {
        deactivateQueue(t, aa, a_qi, scratch);
        return HWLM_CONTINUE_MATCHING;
    }

    ensureQueueActive(t, a_qi, t->queueCount, q, scratch);

    if (unlikely(final_loc < q_cur_loc(q))) {
        DEBUG_PRINTF("err loc %lld < location %lld\n", final_loc, q_cur_loc(q));
        return HWLM_CONTINUE_MATCHING;
    }

    ensureEnd(q, a_qi, final_loc);

    char alive = blast_queue(scratch, q, a_qi, second_place_loc, 0);

    /* We have three possible outcomes:
     * (1) the nfa died
     * (2) we completed the queue (implies that second_place_loc == final_loc)
     * (3) the queue ran to second_place_loc and stopped. In this case we need
     *     to find the next match location.
     */

    if (!alive) {
        if (can_stop_matching(scratch)) {
            DEBUG_PRINTF("roseCatchUpNfas done as bailing\n");
            return HWLM_TERMINATE_MATCHING;
        }

        deactivateQueue(t, aa, a_qi, scratch);
    } else if (q->cur == q->end) {
        DEBUG_PRINTF("queue %u finished, nfa lives [%lld]\n", a_qi, final_loc);

        assert(second_place_loc == final_loc);

        q->cur = q->end = 0;
        pushQueueAt(q, 0, MQE_START, final_loc);
    } else {
        DEBUG_PRINTF("queue %u not finished, %u/%u [%lld/%lld]\n", a_qi, q->cur,
                     q->end, q_cur_loc(q), final_loc);
        DEBUG_PRINTF("finding next match location\n");

        assert(second_place_loc < final_loc);
        assert(q_cur_loc(q) >= second_place_loc);

        if (runNewNfaToNextMatch(t, a_qi, q, final_loc, scratch, aa,
                                 report_ok_loc) == HWLM_TERMINATE_MATCHING) {
            DEBUG_PRINTF("roseCatchUpNfas done\n");
            return HWLM_TERMINATE_MATCHING;
        }
    }

    return HWLM_CONTINUE_MATCHING;
}

void streamInitSufPQ(const struct RoseEngine *t, char *state,
                     struct hs_scratch *scratch) {
    assert(scratch->catchup_pq.qm_size == 0);
    assert(t->outfixBeginQueue != t->outfixEndQueue);

    DEBUG_PRINTF("initSufPQ: outfixes [%u,%u)\n", t->outfixBeginQueue,
                 t->outfixEndQueue);

    u32 qCount = t->queueCount;
    u8 *aa = getActiveLeafArray(t, state);
    u32 aaCount = t->activeArrayCount;
    struct mq *queues = scratch->queues;
    size_t length = scratch->core_info.len;

    u32 qi = mmbit_iterate_bounded(aa, aaCount, t->outfixBeginQueue,
                                   t->outfixEndQueue);
    for (; qi < t->outfixEndQueue;) {
        DEBUG_PRINTF("adding qi=%u\n", qi);
        struct mq *q = queues + qi;

        ensureQueueActive(t, qi, qCount, q, scratch);
        ensureEnd(q, qi, length);

        char alive = nfaQueueExecToMatch(q->nfa, q, length);

        if (alive == MO_MATCHES_PENDING) {
            DEBUG_PRINTF("we have pending matches at %lld\n", q_cur_loc(q));
            s64a qcl = q_cur_loc(q);

            pq_insert_with(&scratch->catchup_pq, scratch, qi, qcl);
        } else if (!alive) {
            deactivateQueue(t, aa, qi, scratch);
        } else {
            assert(q->cur == q->end);
            /* TODO: can this be simplified? the nfa will never produce any
             * matches for this block. */
            DEBUG_PRINTF("queue %u finished, nfa lives\n", qi);
            q->cur = q->end = 0;
            pushQueueAt(q, 0, MQE_START, length);
        }

        qi = mmbit_iterate_bounded(aa, aaCount, qi + 1, t->outfixEndQueue);
    }
}

void blockInitSufPQ(const struct RoseEngine *t, char *state,
                    struct hs_scratch *scratch, char is_small_block) {
    DEBUG_PRINTF("initSufPQ: outfixes [%u,%u)\n", t->outfixBeginQueue,
                 t->outfixEndQueue);

    assert(scratch->catchup_pq.qm_size == 0);
    assert(t->outfixBeginQueue != t->outfixEndQueue);

    struct mq *queues = scratch->queues;
    u8 *aa = getActiveLeafArray(t, state);
    struct fatbit *aqa = scratch->aqa;
    u32 aaCount = t->activeArrayCount;
    u32 qCount = t->queueCount;
    size_t length = scratch->core_info.len;

    for (u32 qi = t->outfixBeginQueue; qi < t->outfixEndQueue; qi++) {
        const struct NfaInfo *info = getNfaInfoByQueue(t, qi);

        if (is_small_block && info->in_sbmatcher) {
            DEBUG_PRINTF("skip outfix %u as it's in the SB matcher\n", qi);
            continue;
        }

        const struct NFA *nfa = getNfaByInfo(t, info);
        DEBUG_PRINTF("testing minwidth %u > len %zu\n", nfa->minWidth,
                      length);
        size_t len = nfaRevAccelCheck(nfa, scratch->core_info.buf, length);
        if (!len) {
            continue;
        }
        mmbit_set(aa, aaCount, qi);
        fatbit_set(aqa, qCount, qi);
        struct mq *q = queues + qi;
        initQueue(q, qi, t, scratch);
        q->length = len; /* adjust for rev_accel */
        nfaQueueInitState(nfa, q);
        pushQueueAt(q, 0, MQE_START, 0);
        pushQueueAt(q, 1, MQE_TOP, 0);
        pushQueueAt(q, 2, MQE_END, length);

        DEBUG_PRINTF("adding qi=%u to pq\n", qi);

        char alive = nfaQueueExecToMatch(q->nfa, q, length);

        if (alive == MO_MATCHES_PENDING) {
            DEBUG_PRINTF("we have pending matches at %lld\n", q_cur_loc(q));
            s64a qcl = q_cur_loc(q);

            pq_insert_with(&scratch->catchup_pq, scratch, qi, qcl);
        } else if (!alive) {
            deactivateQueue(t, aa, qi, scratch);
        } else {
            assert(q->cur == q->end);
            /* TODO: can this be simplified? the nfa will never produce any
             * matches for this block. */
            DEBUG_PRINTF("queue %u finished, nfa lives\n", qi);
            q->cur = q->end = 0;
            pushQueueAt(q, 0, MQE_START, length);
        }
    }
}

/**
 * safe_loc is ???
 */
static rose_inline
hwlmcb_rv_t buildSufPQ(const struct RoseEngine *t, char *state, s64a safe_loc,
                       s64a final_loc, struct hs_scratch *scratch) {
    assert(scratch->catchup_pq.qm_size <= t->outfixEndQueue);

    struct RoseContext *tctxt = &scratch->tctxt;
    assert(t->activeArrayCount);

    assert(scratch->core_info.buf_offset + final_loc
           > tctxt->minNonMpvMatchOffset);
    DEBUG_PRINTF("buildSufPQ final loc %lld (safe %lld)\n", final_loc,
                 safe_loc);
    assert(safe_loc <= final_loc);

    u8 *aa = getActiveLeafArray(t, state);
    u32 aaCount = t->activeArrayCount;

    /* find first match of each pending nfa */
    DEBUG_PRINTF("aa=%p, aaCount=%u\n", aa, aaCount);

    /* Note: mpv MUST not participate in the main priority queue as
     * they may have events pushed on during this process which may be before
     * the catch up point. Outfixes are remain in the pq between catchup events
     * as they never have any incoming events to worry about.
     */
    if (aaCount == t->outfixEndQueue) {
        return HWLM_CONTINUE_MATCHING;
    }

    DEBUG_PRINTF("mib %u/%u\n", t->outfixBeginQueue, aaCount);

    u32 a_qi = mmbit_iterate_bounded(aa, aaCount, t->outfixEndQueue, aaCount);

    if (a_qi == MMB_INVALID) {
        return HWLM_CONTINUE_MATCHING;
    }

    s64a report_ok_loc = tctxt->minNonMpvMatchOffset + 1
        - scratch->core_info.buf_offset;

    hwlmcb_rv_t rv = roseCatchUpMPV(t, report_ok_loc, scratch);
    if (rv != HWLM_CONTINUE_MATCHING) {
        DEBUG_PRINTF("terminating...\n");
        return rv;
    }

    while (a_qi != MMB_INVALID) {
        DEBUG_PRINTF("catching up qi=%u to %lld\n", a_qi, final_loc);
        u32 n_qi = mmbit_iterate(aa, aaCount, a_qi);

        s64a second_place_loc
            = scratch->catchup_pq.qm_size ? pq_top_loc(&scratch->catchup_pq)
                                          : safe_loc;
        second_place_loc = MIN(second_place_loc, safe_loc);
        if (n_qi == MMB_INVALID && report_ok_loc <= second_place_loc) {
            if (buildSufPQ_final(t, report_ok_loc, second_place_loc, final_loc,
                                 scratch, aa, a_qi)
                == HWLM_TERMINATE_MATCHING) {
                return HWLM_TERMINATE_MATCHING;
            }
            break;
        }

        if (add_to_queue(t, scratch->queues, t->queueCount, aa, scratch,
                         final_loc, a_qi, report_ok_loc)
            == HWLM_TERMINATE_MATCHING) {
            DEBUG_PRINTF("roseCatchUpNfas done\n");
            return HWLM_TERMINATE_MATCHING;
        }

        a_qi = n_qi;
    }

    DEBUG_PRINTF("PQ BUILD %u items\n", scratch->catchup_pq.qm_size);
    return HWLM_CONTINUE_MATCHING;
}

static never_inline
hwlmcb_rv_t roseCatchUpNfas(const struct RoseEngine *t, s64a loc,
                            s64a final_loc, struct hs_scratch *scratch) {
    assert(t->activeArrayCount);

    DEBUG_PRINTF("roseCatchUpNfas offset=%llu + %lld/%lld\n",
                 scratch->core_info.buf_offset, loc, final_loc);
    DEBUG_PRINTF("min non mpv match offset %llu\n",
                 scratch->tctxt.minNonMpvMatchOffset);

    struct RoseContext *tctxt = &scratch->tctxt;
    assert(scratch->core_info.buf_offset + loc >= tctxt->minNonMpvMatchOffset);

    char *state = scratch->core_info.state;
    struct mq *queues = scratch->queues;
    u8 *aa = getActiveLeafArray(t, state);

    /* fire off earliest nfa match and catchup anchored matches to that point */
    while (scratch->catchup_pq.qm_size) {
        s64a match_loc = pq_top_loc(&scratch->catchup_pq);
        u32 qi = pq_top(scratch->catchup_pq.qm)->queue;

        DEBUG_PRINTF("winrar q%u@%lld loc %lld\n", qi, match_loc, loc);
        assert(match_loc + scratch->core_info.buf_offset
               >= scratch->tctxt.minNonMpvMatchOffset);

        if (match_loc > loc) {
            /* we have processed all the matches at or before rose's current
             * location; only things remaining on the pq should be outfixes. */
            DEBUG_PRINTF("saving for later\n");
            goto exit;
        }

        /* catch up char matches to this point */
        if (roseCatchUpMPV(t, match_loc, scratch)
            == HWLM_TERMINATE_MATCHING) {
            DEBUG_PRINTF("roseCatchUpNfas done\n");
            return HWLM_TERMINATE_MATCHING;
        }

        assert(match_loc + scratch->core_info.buf_offset
               >= scratch->tctxt.minNonMpvMatchOffset);

        struct mq *q = queues + qi;

        /* outfixes must be advanced all the way as they persist in the pq
         * between catchup events */
        s64a q_final_loc = qi >= t->outfixEndQueue ? final_loc
                                                 : (s64a)scratch->core_info.len;

        /* fire nfa matches, and find next place this nfa match */
        DEBUG_PRINTF("reporting matches %u@%llu [q->cur %u/%u]\n", qi,
                     match_loc, q->cur, q->end);

        /* we then need to catch this nfa up to next earliest nfa match. These
         * matches can be fired directly from the callback. The callback needs
         * to ensure that the anchored matches remain in sync though */
        s64a second_place_loc = findSecondPlace(&scratch->catchup_pq, loc);
        DEBUG_PRINTF("second place %lld loc %lld\n", second_place_loc, loc);

        if (second_place_loc == q_cur_loc(q)) {
            if (runExistingNfaToNextMatch(t, qi, q, q_final_loc, scratch, aa, 1)
                == HWLM_TERMINATE_MATCHING) {
                return HWLM_TERMINATE_MATCHING;
            }
            continue;
        }

        char alive = blast_queue(scratch, q, qi, second_place_loc, 1);

        if (!alive) {
            if (can_stop_matching(scratch)) {
                DEBUG_PRINTF("roseCatchUpNfas done as bailing\n");
                return HWLM_TERMINATE_MATCHING;
            }

            deactivateQueue(t, aa, qi, scratch);
            pq_pop_nice(&scratch->catchup_pq);
        } else if (q->cur == q->end) {
            DEBUG_PRINTF("queue %u finished, nfa lives [%lld]\n", qi, loc);
            q->cur = q->end = 0;
            pushQueueAt(q, 0, MQE_START, loc);
            pq_pop_nice(&scratch->catchup_pq);
        } else if (second_place_loc == q_final_loc) {
            DEBUG_PRINTF("queue %u on hold\n", qi);
            pq_pop_nice(&scratch->catchup_pq);
            break;
        } else {
            DEBUG_PRINTF("queue %u not finished, %u/%u [%lld/%lld]\n",
                          qi, q->cur, q->end, q->items[q->cur].location, loc);
            runExistingNfaToNextMatch(t, qi, q, q_final_loc, scratch, aa, 0);
        }
    }
exit:;
    tctxt->minNonMpvMatchOffset = scratch->core_info.buf_offset + loc;
    DEBUG_PRINTF("roseCatchUpNfas done\n");
    return HWLM_CONTINUE_MATCHING;
}

hwlmcb_rv_t roseCatchUpAll(s64a loc, struct hs_scratch *scratch) {
    /* just need suf/outfixes and mpv */
    DEBUG_PRINTF("loc %lld mnmmo %llu mmo %llu\n", loc,
                 scratch->tctxt.minNonMpvMatchOffset,
                 scratch->tctxt.minMatchOffset);
    assert(scratch->core_info.buf_offset + loc
           > scratch->tctxt.minNonMpvMatchOffset);

    const struct RoseEngine *t = scratch->core_info.rose;
    char *state = scratch->core_info.state;

    hwlmcb_rv_t rv = buildSufPQ(t, state, loc, loc, scratch);
    if (rv != HWLM_CONTINUE_MATCHING) {
        return rv;
    }

    rv = roseCatchUpNfas(t, loc, loc, scratch);
    if (rv != HWLM_CONTINUE_MATCHING) {
        return rv;
    }

    rv = roseCatchUpMPV(t, loc, scratch);
    assert(rv != HWLM_CONTINUE_MATCHING
           || scratch->catchup_pq.qm_size <= t->outfixEndQueue);
    assert(!can_stop_matching(scratch) || rv == HWLM_TERMINATE_MATCHING);
    return rv;
}

hwlmcb_rv_t roseCatchUpSuf(s64a loc, struct hs_scratch *scratch) {
    /* just need suf/outfixes. mpv will be caught up only to last reported
     * external match */
    assert(scratch->core_info.buf_offset + loc
           > scratch->tctxt.minNonMpvMatchOffset);

    const struct RoseEngine *t = scratch->core_info.rose;
    char *state = scratch->core_info.state;

    hwlmcb_rv_t rv = buildSufPQ(t, state, loc, loc, scratch);
    if (rv != HWLM_CONTINUE_MATCHING) {
        return rv;
    }

    rv = roseCatchUpNfas(t, loc, loc, scratch);
    assert(rv != HWLM_CONTINUE_MATCHING ||
           scratch->catchup_pq.qm_size <= t->outfixEndQueue);

    return rv;
}
