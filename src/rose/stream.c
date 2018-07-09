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

#include "catchup.h"
#include "counting_miracle.h"
#include "infix.h"
#include "match.h"
#include "miracle.h"
#include "program_runtime.h"
#include "rose.h"
#include "rose_internal.h"
#include "stream_long_lit.h"
#include "hwlm/hwlm.h"
#include "nfa/mcclellan.h"
#include "nfa/nfa_api.h"
#include "nfa/nfa_api_queue.h"
#include "nfa/nfa_internal.h"
#include "util/fatbit.h"

static rose_inline
void runAnchoredTableStream(const struct RoseEngine *t, const void *atable,
                            size_t alen, u64a offset,
                            struct hs_scratch *scratch) {
    char *state_base = scratch->core_info.state + t->stateOffsets.anchorState;
    const struct anchored_matcher_info *curr = atable;

    do {
        DEBUG_PRINTF("--anchored nfa (+%u) no %u so %u\n",
                     curr->anchoredMinDistance, curr->next_offset,
                     curr->state_offset);
        const struct NFA *nfa
            = (const struct NFA *)((const char *)curr + sizeof(*curr));
        assert(ISALIGNED_CL(nfa));
        assert(isMcClellanType(nfa->type));

        char *state = state_base + curr->state_offset;

        char start = 0;
        size_t adj = 0;

        if (offset <= curr->anchoredMinDistance) {
            adj = curr->anchoredMinDistance - offset;
            if (adj >= alen) {
                goto next_nfa;
            }

            start = 1;
        } else {
            // (No state decompress necessary.)
            if (nfa->type == MCCLELLAN_NFA_8) {
                if (!*(u8 *)state) {
                    goto next_nfa;
                }
            } else {
                if (!unaligned_load_u16(state)) {
                    goto next_nfa;
                }
            }
        }

        if (nfa->type == MCCLELLAN_NFA_8) {
            nfaExecMcClellan8_SimpStream(nfa, state, scratch->core_info.buf,
                                         start, adj, alen, roseAnchoredCallback,
                                         scratch);
        } else {
            nfaExecMcClellan16_SimpStream(nfa, state, scratch->core_info.buf,
                                          start, adj, alen,
                                          roseAnchoredCallback, scratch);
        }

    next_nfa:
        if (!curr->next_offset) {
            break;
        }

        curr = (const void *)((const char *)curr + curr->next_offset);
    } while (1);
}


static really_inline
void saveStreamState(const struct NFA *nfa, struct mq *q, s64a loc) {
    DEBUG_PRINTF("offset=%llu, length=%zu, hlength=%zu, loc=%lld\n",
                 q->offset, q->length, q->hlength, loc);
    nfaQueueCompressState(nfa, q, loc);
}

static really_inline
u8 getByteBefore(const struct core_info *ci, s64a sp) {
    if (sp > 0) { // in main buffer
        assert(sp <= (s64a)ci->len);
        return ci->buf[sp - 1];
    }
    // in history buffer
    assert(-sp < (s64a)ci->hlen);
    return ci->hbuf[ci->hlen + sp - 1];
}

/** \brief Return value for \ref roseScanForMiracles. */
enum MiracleAction {
    MIRACLE_DEAD, //!< kill off this engine
    MIRACLE_SAVED, //!< engine has been caught up and state saved
    MIRACLE_CONTINUE //!< continue running and catch up engine
};

static really_inline
enum MiracleAction roseScanForMiracles(const struct RoseEngine *t, char *state,
                                       struct hs_scratch *scratch, u32 qi,
                                       const struct LeftNfaInfo *left,
                                       const struct NFA *nfa) {
    struct core_info *ci = &scratch->core_info;
    const u32 qCount = t->queueCount;
    struct mq *q = scratch->queues + qi;

    const char q_active = fatbit_isset(scratch->aqa, qCount, qi);
    DEBUG_PRINTF("q_active=%d\n", q_active);

    const s64a begin_loc = q_active ? q_cur_loc(q) : 0;
    const s64a end_loc = ci->len;

    s64a miracle_loc;
    if (roseMiracleOccurs(t, left, ci, begin_loc, end_loc, &miracle_loc)) {
        goto found_miracle;
    }

    if (roseCountingMiracleOccurs(t, left, ci, begin_loc, end_loc,
                                  &miracle_loc)) {
        goto found_miracle;
    }

    DEBUG_PRINTF("no miracle\n");
    return MIRACLE_CONTINUE;

found_miracle:
    DEBUG_PRINTF("miracle at %lld\n", miracle_loc);

    if (left->infix) {
        if (!q_active) {
            DEBUG_PRINTF("killing infix\n");
            return MIRACLE_DEAD;
        }

        DEBUG_PRINTF("skip q forward, %lld to %lld\n", begin_loc, miracle_loc);
        q_skip_forward_to(q, miracle_loc);
        if (q_last_type(q) == MQE_START) {
            DEBUG_PRINTF("miracle caused infix to die\n");
            return MIRACLE_DEAD;
        }

        DEBUG_PRINTF("re-init infix state\n");
        assert(q->items[q->cur].type == MQE_START);
        q->items[q->cur].location = miracle_loc;
        nfaQueueInitState(q->nfa, q);
    } else {
        if (miracle_loc > end_loc - t->historyRequired) {
            char *streamState = state + getNfaInfoByQueue(t, qi)->stateOffset;
            u64a offset = ci->buf_offset + miracle_loc;
            u8 key = offset ? getByteBefore(ci, miracle_loc) : 0;
            DEBUG_PRINTF("init state, key=0x%02x, offset=%llu\n", key, offset);
            if (!nfaInitCompressedState(nfa, offset, streamState, key)) {
                return MIRACLE_DEAD;
            }
            storeRoseDelay(t, state, left, (s64a)ci->len - miracle_loc);
            return MIRACLE_SAVED;
        }

        DEBUG_PRINTF("re-init prefix (skip %lld->%lld)\n", begin_loc,
                     miracle_loc);
        if (!q_active) {
            fatbit_set(scratch->aqa, qCount, qi);
            initRoseQueue(t, qi, left, scratch);
        }
        q->cur = q->end = 0;
        pushQueueAt(q, 0, MQE_START, miracle_loc);
        pushQueueAt(q, 1, MQE_TOP, miracle_loc);
        nfaQueueInitState(q->nfa, q);
    }

    return MIRACLE_CONTINUE;
}


static really_inline
char roseCatchUpLeftfix(const struct RoseEngine *t, char *state,
                        struct hs_scratch *scratch, u32 qi,
                        const struct LeftNfaInfo *left) {
    assert(!left->transient); // active roses only

    struct core_info *ci = &scratch->core_info;
    const u32 qCount = t->queueCount;
    struct mq *q = scratch->queues + qi;
    const struct NFA *nfa = getNfaByQueue(t, qi);

    if (nfaSupportsZombie(nfa)
        && ci->buf_offset /* prefix can be alive with no q */
        && !fatbit_isset(scratch->aqa, qCount, qi)
        && isZombie(t, state, left)) {
        DEBUG_PRINTF("yawn - zombie\n");
        return 1;
    }

    if (left->stopTable) {
        enum MiracleAction mrv =
            roseScanForMiracles(t, state, scratch, qi, left, nfa);
        switch (mrv) {
        case MIRACLE_DEAD:
            return 0;
        case MIRACLE_SAVED:
            return 1;
        default:
            assert(mrv == MIRACLE_CONTINUE);
            break;
        }
    }

    if (!fatbit_set(scratch->aqa, qCount, qi)) {
        initRoseQueue(t, qi, left, scratch);

        s32 sp;
        if (ci->buf_offset) {
            sp = -(s32)loadRoseDelay(t, state, left);
        } else {
            sp = 0;
        }

        DEBUG_PRINTF("ci->len=%zu, sp=%d, historyRequired=%u\n", ci->len, sp,
                     t->historyRequired);

        if ( ci->len - sp + 1 < t->historyRequired) {
            // we'll end up safely in the history region.
            DEBUG_PRINTF("safely in history, skipping\n");
            storeRoseDelay(t, state, left, (s64a)ci->len - sp);
            return 1;
        }

        pushQueueAt(q, 0, MQE_START, sp);
        if (left->infix || ci->buf_offset + sp > 0) {
            loadStreamState(nfa, q, sp);
        } else {
            pushQueueAt(q, 1, MQE_TOP, sp);
            nfaQueueInitState(nfa, q);
        }
    } else {
        DEBUG_PRINTF("queue already active\n");
        if (q->end - q->cur == 1 && q_cur_type(q) == MQE_START) {
            DEBUG_PRINTF("empty queue, start loc=%lld\n", q_cur_loc(q));
            s64a last_loc = q_cur_loc(q);
            if (ci->len - last_loc + 1 < t->historyRequired) {
                // we'll end up safely in the history region.
                DEBUG_PRINTF("safely in history, saving state and skipping\n");
                saveStreamState(nfa, q, last_loc);
                storeRoseDelay(t, state, left, (s64a)ci->len - last_loc);
                return 1;
            }
        }
    }

    // Determine whether the byte before last_loc will be in the history
    // buffer on the next stream write.
    s64a last_loc = q_last_loc(q);
    s64a leftovers = ci->len - last_loc;
    if (leftovers + 1 >= t->historyRequired) {
        u32 catchup_offset = left->maxLag ? left->maxLag - 1 : 0;
        last_loc = (s64a)ci->len - catchup_offset;
    }

    if (left->infix) {
        if (infixTooOld(q, last_loc)) {
            DEBUG_PRINTF("infix died of old age\n");
            return 0;
        }
        reduceInfixQueue(q, last_loc, left->maxQueueLen, q->nfa->maxWidth);
    }

    DEBUG_PRINTF("end scan at %lld\n", last_loc);
    pushQueueNoMerge(q, MQE_END, last_loc);

#ifdef DEBUG
    debugQueue(q);
#endif

    char rv = nfaQueueExecRose(nfa, q, MO_INVALID_IDX);
    if (!rv) { /* nfa is dead */
        DEBUG_PRINTF("died catching up to stream boundary\n");
        return 0;
    } else {
        DEBUG_PRINTF("alive, saving stream state\n");
        if (nfaSupportsZombie(nfa) &&
            nfaGetZombieStatus(nfa, q, last_loc) == NFA_ZOMBIE_ALWAYS_YES) {
            DEBUG_PRINTF("not so fast - zombie\n");
            setAsZombie(t, state, left);
        } else {
            saveStreamState(nfa, q, last_loc);
            storeRoseDelay(t, state, left, (s64a)ci->len - last_loc);
        }
    }

    return 1;
}

static rose_inline
void roseCatchUpLeftfixes(const struct RoseEngine *t, char *state,
                          struct hs_scratch *scratch) {
    if (!t->activeLeftIterOffset) {
        // No sparse iter, no non-transient roses.
        return;
    }

    // As per UE-1629, we catch up leftfix engines to:
    //  * current position (last location in the queue, or last location we
    //    executed to if the queue is empty) if that position (and the byte
    //    before so we can decompress the stream state) will be in the history
    //    buffer on the next stream write; OR
    //  * (stream_boundary - max_delay) other

    u8 *ara = getActiveLeftArray(t, state); /* indexed by offsets into
                                             * left_table */
    const u32 arCount = t->activeLeftCount;
    const struct LeftNfaInfo *left_table = getLeftTable(t);
    const struct mmbit_sparse_iter *it = getActiveLeftIter(t);

    struct mmbit_sparse_state si_state[MAX_SPARSE_ITER_STATES];

    u32 idx = 0;
    u32 ri = mmbit_sparse_iter_begin(ara, arCount, &idx, it, si_state);
    for (; ri != MMB_INVALID;
           ri = mmbit_sparse_iter_next(ara, arCount, ri, &idx, it, si_state)) {
        const struct LeftNfaInfo *left = left_table + ri;
        u32 qi = ri + t->leftfixBeginQueue;
        DEBUG_PRINTF("leftfix %u of %u, maxLag=%u, infix=%d\n", ri, arCount,
                     left->maxLag, (int)left->infix);
        if (!roseCatchUpLeftfix(t, state, scratch, qi, left)) {
            DEBUG_PRINTF("removing rose %u from active list\n", ri);
            DEBUG_PRINTF("groups old=%016llx mask=%016llx\n",
                         scratch->tctxt.groups, left->squash_mask);
            scratch->tctxt.groups &= left->squash_mask;
            mmbit_unset(ara, arCount, ri);
        }
    }
}

// Saves out stream state for all our active suffix NFAs.
static rose_inline
void roseSaveNfaStreamState(const struct RoseEngine *t, char *state,
                            struct hs_scratch *scratch) {
    struct mq *queues = scratch->queues;
    u8 *aa = getActiveLeafArray(t, state);
    u32 aaCount = t->activeArrayCount;

    if (scratch->tctxt.mpv_inactive) {
        DEBUG_PRINTF("mpv is dead as a doornail\n");
        /* mpv if it exists is queue 0 */
        mmbit_unset(aa, aaCount, 0);
    }

    for (u32 qi = mmbit_iterate(aa, aaCount, MMB_INVALID); qi != MMB_INVALID;
         qi = mmbit_iterate(aa, aaCount, qi)) {
        DEBUG_PRINTF("saving stream state for qi=%u\n", qi);

        struct mq *q = queues + qi;

        // If it's active, it should have an active queue (as we should have
        // done some work!)
        assert(fatbit_isset(scratch->aqa, t->queueCount, qi));

        const struct NFA *nfa = getNfaByQueue(t, qi);
        saveStreamState(nfa, q, q_cur_loc(q));
    }
}

static rose_inline
void ensureStreamNeatAndTidy(const struct RoseEngine *t, char *state,
                             struct hs_scratch *scratch, size_t length,
                             u64a offset) {
    struct RoseContext *tctxt = &scratch->tctxt;

    if (roseCatchUpTo(t, scratch, length + scratch->core_info.buf_offset) ==
        HWLM_TERMINATE_MATCHING) {
        return; /* dead; no need to clean up state. */
    }
    roseSaveNfaStreamState(t, state, scratch);
    roseCatchUpLeftfixes(t, state, scratch);
    roseFlushLastByteHistory(t, scratch, offset + length);
    tctxt->lastEndOffset = offset + length;
    storeGroups(t, state, tctxt->groups);
    storeLongLiteralState(t, state, scratch);
}

static really_inline
void do_rebuild(const struct RoseEngine *t, struct hs_scratch *scratch) {
    assert(t->drmatcherOffset);
    assert(!can_stop_matching(scratch));

    const struct HWLM *hwlm = getByOffset(t, t->drmatcherOffset);
    size_t len = MIN(scratch->core_info.hlen, t->delayRebuildLength);
    const u8 *buf = scratch->core_info.hbuf + scratch->core_info.hlen - len;
    DEBUG_PRINTF("BEGIN FLOATING REBUILD over %zu bytes\n", len);

    scratch->core_info.status &= ~STATUS_DELAY_DIRTY;

    hwlmExec(hwlm, buf, len, 0, roseDelayRebuildCallback, scratch,
             scratch->tctxt.groups);
    assert(!can_stop_matching(scratch));
}

static rose_inline
void runEagerPrefixesStream(const struct RoseEngine *t,
                            struct hs_scratch *scratch) {
    if (!t->eagerIterOffset
        || scratch->core_info.buf_offset >= EAGER_STOP_OFFSET) {
        return;
    }

    char *state = scratch->core_info.state;
    u8 *ara = getActiveLeftArray(t, state); /* indexed by offsets into
                                             * left_table */
    const u32 arCount = t->activeLeftCount;
    const u32 qCount = t->queueCount;
    const struct LeftNfaInfo *left_table = getLeftTable(t);
    const struct mmbit_sparse_iter *it = getByOffset(t, t->eagerIterOffset);

    struct mmbit_sparse_state si_state[MAX_SPARSE_ITER_STATES];

    u32 idx = 0;
    u32 ri = mmbit_sparse_iter_begin(ara, arCount, &idx, it, si_state);
    for (; ri != MMB_INVALID;
           ri = mmbit_sparse_iter_next(ara, arCount, ri, &idx, it, si_state)) {
        const struct LeftNfaInfo *left = left_table + ri;
        u32 qi = ri + t->leftfixBeginQueue;
        DEBUG_PRINTF("leftfix %u of %u, maxLag=%u\n", ri, arCount, left->maxLag);

        assert(!fatbit_isset(scratch->aqa, qCount, qi));
        assert(left->eager);
        assert(!left->infix);

        struct mq *q = scratch->queues + qi;
        const struct NFA *nfa = getNfaByQueue(t, qi);
        s64a loc = MIN(scratch->core_info.len,
                       EAGER_STOP_OFFSET - scratch->core_info.buf_offset);

        fatbit_set(scratch->aqa, qCount, qi);
        initRoseQueue(t, qi, left, scratch);

        if (scratch->core_info.buf_offset) {
            s64a sp = left->transient ? -(s64a)scratch->core_info.hlen
                                      : -(s64a)loadRoseDelay(t, state, left);
            pushQueueAt(q, 0, MQE_START, sp);
            if (scratch->core_info.buf_offset + sp > 0) {
                loadStreamState(nfa, q, sp);
                /* if the leftfix fix is currently in a match state, we cannot
                 * advance it. */
                if (nfaInAnyAcceptState(nfa, q)) {
                    continue;
                }
                pushQueueAt(q, 1, MQE_END, loc);
            } else {
                pushQueueAt(q, 1, MQE_TOP, sp);
                pushQueueAt(q, 2, MQE_END, loc);
                nfaQueueInitState(q->nfa, q);
            }
        } else {
            pushQueueAt(q, 0, MQE_START, 0);
            pushQueueAt(q, 1, MQE_TOP, 0);
            pushQueueAt(q, 2, MQE_END, loc);
            nfaQueueInitState(nfa, q);
        }

        char alive = nfaQueueExecToMatch(q->nfa, q, loc);

        if (!alive) {
            DEBUG_PRINTF("queue %u dead, squashing\n", qi);
            mmbit_unset(ara, arCount, ri);
            fatbit_unset(scratch->aqa, qCount, qi);
            scratch->tctxt.groups &= left->squash_mask;
        } else if (q->cur == q->end) {
            assert(alive != MO_MATCHES_PENDING);
            /* unlike in block mode we cannot squash groups if there is no match
             * in this block as we need the groups on for later stream writes */
            /* TODO: investigate possibility of a method to suppress groups for
             * a single stream block. */
            DEBUG_PRINTF("queue %u finished, nfa lives\n", qi);
            q->cur = q->end = 0;
            pushQueueAt(q, 0, MQE_START, loc);
        } else {
            assert(alive == MO_MATCHES_PENDING);
            DEBUG_PRINTF("queue %u unfinished, nfa lives\n", qi);
            q->end--; /* remove end item */
        }
    }
}

static really_inline
int can_never_match(const struct RoseEngine *t, char *state,
                    struct hs_scratch *scratch, size_t length, u64a offset) {
    struct RoseContext *tctxt = &scratch->tctxt;

    if (tctxt->groups) {
        DEBUG_PRINTF("still has active groups\n");
        return 0;
    }

    if (offset + length <= t->anchoredDistance) { /* not < as may have eod */
        DEBUG_PRINTF("still in anchored region\n");
        return 0;
    }

    if (t->lastByteHistoryIterOffset) { /* last byte history is hard */
        DEBUG_PRINTF("last byte history\n");
        return 0;
    }

    if (mmbit_any(getActiveLeafArray(t, state), t->activeArrayCount)) {
        DEBUG_PRINTF("active leaf\n");
        return 0;
    }

    return 1;
}

void roseStreamExec(const struct RoseEngine *t, struct hs_scratch *scratch) {
    DEBUG_PRINTF("OH HAI [%llu, %llu)\n", scratch->core_info.buf_offset,
                 scratch->core_info.buf_offset + (u64a)scratch->core_info.len);
    assert(t);
    assert(scratch->core_info.hbuf);
    assert(scratch->core_info.buf);

    // We should not have been called if we've already been told to terminate
    // matching.
    assert(!told_to_stop_matching(scratch));

    assert(mmbit_sparse_iter_state_size(t->rolesWithStateCount)
           < MAX_SPARSE_ITER_STATES);

    size_t length = scratch->core_info.len;
    u64a offset = scratch->core_info.buf_offset;

    // We may have a maximum width (for engines constructed entirely
    // of bi-anchored patterns). If this write would result in us progressing
    // beyond this point, we cannot possibly match.
    if (t->maxBiAnchoredWidth != ROSE_BOUND_INF
        && offset + length > t->maxBiAnchoredWidth) {
        DEBUG_PRINTF("bailing, write would progress beyond maxBAWidth\n");
        return;
    }

    char *state = scratch->core_info.state;

    struct RoseContext *tctxt = &scratch->tctxt;
    tctxt->mpv_inactive = 0;
    tctxt->groups = loadGroups(t, state);
    tctxt->lit_offset_adjust = offset + 1; // index after last byte
    tctxt->delayLastEndOffset = offset;
    tctxt->lastEndOffset = offset;
    tctxt->filledDelayedSlots = 0;
    tctxt->lastMatchOffset = 0;
    tctxt->lastCombMatchOffset = offset;
    tctxt->minMatchOffset = offset;
    tctxt->minNonMpvMatchOffset = offset;
    tctxt->next_mpv_offset = 0;

    DEBUG_PRINTF("BEGIN: history len=%zu, buffer len=%zu groups=%016llx\n",
                 scratch->core_info.hlen, scratch->core_info.len, tctxt->groups);

    fatbit_clear(scratch->aqa);
    scratch->al_log_sum = 0;
    scratch->catchup_pq.qm_size = 0;

    if (t->outfixBeginQueue != t->outfixEndQueue) {
        streamInitSufPQ(t, state, scratch);
    }

    runEagerPrefixesStream(t, scratch);

    u32 alen = t->anchoredDistance > offset ?
        MIN(length + offset, t->anchoredDistance) - offset : 0;

    const struct anchored_matcher_info *atable = getALiteralMatcher(t);
    if (atable && alen) {
        DEBUG_PRINTF("BEGIN ANCHORED %zu/%u\n", scratch->core_info.hlen, alen);
        runAnchoredTableStream(t, atable, alen, offset, scratch);

        if (can_stop_matching(scratch)) {
            goto exit;
        }
    }

    const struct HWLM *ftable = getFLiteralMatcher(t);
    if (ftable) {
        // Load in long literal table state and set up "fake history" buffers
        // (ll_buf, etc, used by the CHECK_LONG_LIT instruction). Note that this
        // must be done here in order to ensure that it happens before any path
        // that leads to storeLongLiteralState(), which relies on these buffers.
        loadLongLiteralState(t, state, scratch);

        if (t->noFloatingRoots && !roseHasInFlightMatches(t, state, scratch)) {
            DEBUG_PRINTF("skip FLOATING: no inflight matches\n");
            goto flush_delay_and_exit;
        }

        size_t flen = length;
        if (t->floatingDistance != ROSE_BOUND_INF) {
            flen = t->floatingDistance > offset ?
                MIN(t->floatingDistance, length + offset) - offset : 0;
        }

        size_t hlength = scratch->core_info.hlen;

        char rebuild = hlength &&
                       (scratch->core_info.status & STATUS_DELAY_DIRTY) &&
                       (t->maxFloatingDelayedMatch == ROSE_BOUND_INF ||
                        offset < t->maxFloatingDelayedMatch);
        DEBUG_PRINTF("**rebuild %hhd status %hhu mfdm %u, offset %llu\n",
                     rebuild, scratch->core_info.status,
                     t->maxFloatingDelayedMatch, offset);

        if (rebuild) { /* rebuild floating delayed match stuff */
            do_rebuild(t, scratch);
        }

        if (!flen) {
            goto flush_delay_and_exit;
        }

        if (flen + offset <= t->floatingMinDistance) {
            DEBUG_PRINTF("skip FLOATING: before floating min\n");
            goto flush_delay_and_exit;
        }

        size_t start = 0;
        if (offset < t->floatingMinDistance) {
            // This scan crosses the floating min distance, so we can use that
            // to set HWLM's "start" offset.
            start = t->floatingMinDistance - offset;
        }
        DEBUG_PRINTF("start=%zu\n", start);

        DEBUG_PRINTF("BEGIN FLOATING (over %zu/%zu)\n", flen, length);
        hwlmExecStreaming(ftable, flen, start, roseFloatingCallback, scratch,
                          tctxt->groups & t->floating_group_mask);
    }

flush_delay_and_exit:
    DEBUG_PRINTF("flushing floating\n");
    if (cleanUpDelayed(t, scratch, length, offset) == HWLM_TERMINATE_MATCHING) {
        return;
    }

exit:
    DEBUG_PRINTF("CLEAN UP TIME\n");
    if (!can_stop_matching(scratch)) {
        ensureStreamNeatAndTidy(t, state, scratch, length, offset);
    }

    if (!told_to_stop_matching(scratch)
        && can_never_match(t, state, scratch, length, offset)) {
        DEBUG_PRINTF("PATTERN SET IS EXHAUSTED\n");
        scratch->core_info.status = STATUS_EXHAUSTED;
        return;
    }

    DEBUG_PRINTF("DONE STREAMING SCAN, status = %u\n",
                 scratch->core_info.status);
    return;
}

static rose_inline
void roseStreamInitEod(const struct RoseEngine *t, u64a offset,
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
    tctxt->lastCombMatchOffset = offset; /* DO NOT set 0 here! */
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

    roseStreamInitEod(t, offset, scratch);

    DEBUG_PRINTF("running eod program at %u\n", t->eodProgramOffset);

    // There should be no pending delayed literals.
    assert(!scratch->tctxt.filledDelayedSlots);

    const u64a som = 0;
    const u8 flags = ROSE_PROG_FLAG_SKIP_MPV_CATCHUP;

    // Note: we ignore the result, as this is the last thing to ever happen on
    // a scan.
    roseRunProgram(t, scratch, t->eodProgramOffset, som, offset, flags);
}
