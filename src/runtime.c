/*
 * Copyright (c) 2015-2022, Intel Corporation
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

/** \file
 * \brief Runtime functions.
 */

#include <stdlib.h>
#include <string.h>

#include "allocator.h"
#include "hs_compile.h" /* for HS_MODE_* flags */
#include "hs_runtime.h"
#include "hs_internal.h"
#include "hwlm/hwlm.h"
#include "nfa/mcclellan.h"
#include "nfa/nfa_api.h"
#include "nfa/nfa_api_util.h"
#include "nfa/nfa_internal.h"
#include "nfa/nfa_rev_api.h"
#include "nfa/sheng.h"
#include "smallwrite/smallwrite_internal.h"
#include "rose/rose.h"
#include "rose/runtime.h"
#include "database.h"
#include "report.h"
#include "scratch.h"
#include "som/som_runtime.h"
#include "som/som_stream.h"
#include "state.h"
#include "stream_compress.h"
#include "ue2common.h"
#include "util/exhaust.h"
#include "util/multibit.h"

static really_inline
void prefetch_data(const char *data, unsigned length) {
    __builtin_prefetch(data);
    __builtin_prefetch(data + length/2);
    __builtin_prefetch(data + length - 24);
}

/** dummy event handler for use when user does not provide one */
static
int HS_CDECL null_onEvent(UNUSED unsigned id, UNUSED unsigned long long from,
                          UNUSED unsigned long long to, UNUSED unsigned flags,
                          UNUSED void *ctxt) {
    return 0;
}

static really_inline
u32 getHistoryAmount(const struct RoseEngine *t, u64a offset) {
    return MIN(t->historyRequired, offset);
}

static really_inline
u8 *getHistory(char *state, const struct RoseEngine *t, u64a offset) {
    return (u8 *)state + t->stateOffsets.history + t->historyRequired
        - MIN(t->historyRequired, offset);
}

/** \brief Sanity checks for scratch space.
 *
 * Although more at home in scratch.c, it is located here to be closer to its
 * callers.
 */
static really_inline
char validScratch(const struct RoseEngine *t, const struct hs_scratch *s) {
    if (!ISALIGNED_CL(s)) {
        DEBUG_PRINTF("bad alignment %p\n", s);
        return 0;
    }

    if (s->magic != SCRATCH_MAGIC) {
        DEBUG_PRINTF("bad magic 0x%x\n", s->magic);
        return 0;
    }

    if (t->mode == HS_MODE_BLOCK && t->stateOffsets.end > s->bStateSize) {
        DEBUG_PRINTF("bad state size\n");
        return 0;
    }

    if (t->queueCount > s->queueCount) {
        DEBUG_PRINTF("bad queue count\n");
        return 0;
    }

    /* TODO: add quick rose sanity checks */

    return 1;
}

static really_inline
void populateCoreInfo(struct hs_scratch *s, const struct RoseEngine *rose,
                      char *state, match_event_handler onEvent, void *userCtx,
                      const char *data, size_t length, const u8 *history,
                      size_t hlen, u64a offset, u8 status,
                      UNUSED unsigned int flags) {
    assert(rose);
    s->core_info.userContext = userCtx;
    s->core_info.userCallback = onEvent ? onEvent : null_onEvent;
    s->core_info.rose = rose;
    s->core_info.state = state; /* required for chained queues + evec */

    s->core_info.exhaustionVector = state + rose->stateOffsets.exhausted;
    s->core_info.status = status;
    s->core_info.buf = (const u8 *)data;
    s->core_info.len = length;
    s->core_info.hbuf = history;
    s->core_info.hlen = hlen;
    s->core_info.buf_offset = offset;

    /* and some stuff not actually in core info */
    s->som_set_now_offset = ~0ULL;
    s->deduper.current_report_offset = ~0ULL;
    s->deduper.som_log_dirty = 1; /* som logs have not been cleared */
    s->fdr_conf = NULL;

    // Rose program execution (used for some report paths) depends on these
    // values being initialised.
    s->tctxt.lastMatchOffset = 0;
    s->tctxt.minMatchOffset = offset;
    s->tctxt.minNonMpvMatchOffset = offset;
}

#define STATUS_VALID_BITS                                                      \
    (STATUS_TERMINATED | STATUS_EXHAUSTED | STATUS_DELAY_DIRTY | STATUS_ERROR)

/** \brief Retrieve status bitmask from stream state. */
static really_inline
u8 getStreamStatus(const char *state) {
    u8 status = *(const u8 *)(state + ROSE_STATE_OFFSET_STATUS_FLAGS);
    assert((status & ~STATUS_VALID_BITS) == 0);
    return status;
}

/** \brief Store status bitmask to stream state. */
static really_inline
void setStreamStatus(char *state, u8 status) {
    assert((status & ~STATUS_VALID_BITS) == 0);
    *(u8 *)(state + ROSE_STATE_OFFSET_STATUS_FLAGS) = status;
}

/** \brief Initialise SOM state. Used in both block and streaming mode. */
static really_inline
void initSomState(const struct RoseEngine *rose, char *state) {
    assert(rose && state);
    const u32 somCount = rose->somLocationCount;
    mmbit_clear((u8 *)state + rose->stateOffsets.somValid, somCount);
    mmbit_clear((u8 *)state + rose->stateOffsets.somWritable, somCount);
}

static really_inline
void rawBlockExec(const struct RoseEngine *rose, struct hs_scratch *scratch) {
    assert(rose);
    assert(scratch);

    initSomState(rose, scratch->core_info.state);

    DEBUG_PRINTF("blockmode scan len=%zu\n", scratch->core_info.len);

    roseBlockExec(rose, scratch);
}

static really_inline
void pureLiteralInitScratch(struct hs_scratch *scratch, u64a offset) {
    // Some init has already been done.
    assert(offset == scratch->core_info.buf_offset);

    scratch->tctxt.lit_offset_adjust = offset + 1;
    scratch->tctxt.lastEndOffset = offset;
    scratch->tctxt.delayLastEndOffset = offset;
    scratch->tctxt.filledDelayedSlots = 0;
    scratch->al_log_sum = 0;
}

static really_inline
void pureLiteralBlockExec(const struct RoseEngine *rose,
                          struct hs_scratch *scratch) {
    assert(rose);
    assert(scratch);

    const struct HWLM *ftable = getFLiteralMatcher(rose);
    initSomState(rose, scratch->core_info.state);
    const u8 *buffer = scratch->core_info.buf;
    size_t length = scratch->core_info.len;
    DEBUG_PRINTF("rose engine %d\n", rose->runtimeImpl);

    pureLiteralInitScratch(scratch, 0);
    scratch->tctxt.groups = rose->initialGroups;

    hwlmExec(ftable, buffer, length, 0, roseCallback, scratch,
             rose->initialGroups & rose->floating_group_mask);
}

static really_inline
void initOutfixQueue(struct mq *q, u32 qi, const struct RoseEngine *t,
                     struct hs_scratch *scratch) {
    const struct NfaInfo *info = getNfaInfoByQueue(t, qi);
    q->nfa = getNfaByInfo(t, info);
    q->end = 0;
    q->cur = 0;
    q->state = scratch->fullState + info->fullStateOffset;
    q->streamState = (char *)scratch->core_info.state + info->stateOffset;
    q->offset = scratch->core_info.buf_offset;
    q->buffer = scratch->core_info.buf;
    q->length = scratch->core_info.len;
    q->history = scratch->core_info.hbuf;
    q->hlength = scratch->core_info.hlen;
    q->cb = roseReportAdaptor;
    q->context = scratch;
    q->report_current = 0;

    DEBUG_PRINTF("qi=%u, offset=%llu, fullState=%u, streamState=%u, "
                 "state=%u\n", qi, q->offset, info->fullStateOffset,
                 info->stateOffset, *(u32 *)q->state);
}

static never_inline
void soleOutfixBlockExec(const struct RoseEngine *t,
                         struct hs_scratch *scratch) {
    assert(t);
    assert(scratch);

    initSomState(t, scratch->core_info.state);
    assert(t->outfixEndQueue == 1);
    assert(!t->amatcherOffset);
    assert(!t->ematcherOffset);
    assert(!t->fmatcherOffset);

    const struct NFA *nfa = getNfaByQueue(t, 0);

    size_t len = nfaRevAccelCheck(nfa, scratch->core_info.buf,
                                  scratch->core_info.len);
    if (!len) {
        return;
    }

    struct mq *q = scratch->queues;
    initOutfixQueue(q, 0, t, scratch);
    q->length = len; /* adjust for rev_accel */
    nfaQueueInitState(nfa, q);
    pushQueueAt(q, 0, MQE_START, 0);
    pushQueueAt(q, 1, MQE_TOP, 0);
    pushQueueAt(q, 2, MQE_END, scratch->core_info.len);

    char rv = nfaQueueExec(q->nfa, q, scratch->core_info.len);

    if (rv && nfaAcceptsEod(nfa) && len == scratch->core_info.len) {
        nfaCheckFinalState(nfa, q->state, q->streamState, q->length, q->cb,
                           scratch);
    }
}

static rose_inline
void runSmallWriteEngine(const struct SmallWriteEngine *smwr,
                         struct hs_scratch *scratch) {
    assert(smwr);
    assert(scratch);

    const u8 *buffer = scratch->core_info.buf;
    size_t length = scratch->core_info.len;

    DEBUG_PRINTF("USING SMALL WRITE\n");

    if (length <= smwr->start_offset) {
        DEBUG_PRINTF("too short\n");
        return;
    }

    const struct NFA *nfa = getSmwrNfa(smwr);

    size_t local_alen = length - smwr->start_offset;
    const u8 *local_buffer = buffer + smwr->start_offset;

    assert(isDfaType(nfa->type));
    if (nfa->type == MCCLELLAN_NFA_8) {
        nfaExecMcClellan8_B(nfa, smwr->start_offset, local_buffer,
                            local_alen, roseReportAdaptor, scratch);
    } else if (nfa->type == MCCLELLAN_NFA_16) {
        nfaExecMcClellan16_B(nfa, smwr->start_offset, local_buffer,
                             local_alen, roseReportAdaptor, scratch);
    } else {
        nfaExecSheng_B(nfa, smwr->start_offset, local_buffer,
                       local_alen, roseReportAdaptor, scratch);
    }
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_scan(const hs_database_t *db, const char *data,
                            unsigned length, unsigned flags,
                            hs_scratch_t *scratch, match_event_handler onEvent,
                            void *userCtx) {
    if (unlikely(!scratch || !data)) {
        return HS_INVALID;
    }

    hs_error_t err = validDatabase(db);
    if (unlikely(err != HS_SUCCESS)) {
        return err;
    }

    const struct RoseEngine *rose = hs_get_bytecode(db);
    if (unlikely(!ISALIGNED_16(rose))) {
        return HS_INVALID;
    }

    if (unlikely(rose->mode != HS_MODE_BLOCK)) {
        return HS_DB_MODE_ERROR;
    }

    if (unlikely(!validScratch(rose, scratch))) {
        return HS_INVALID;
    }

    if (unlikely(markScratchInUse(scratch))) {
        return HS_SCRATCH_IN_USE;
    }

    if (rose->minWidth > length) {
        DEBUG_PRINTF("minwidth=%u > length=%u\n", rose->minWidth, length);
        unmarkScratchInUse(scratch);
        return HS_SUCCESS;
    }

    prefetch_data(data, length);

    /* populate core info in scratch */
    populateCoreInfo(scratch, rose, scratch->bstate, onEvent, userCtx, data,
                     length, NULL, 0, 0, 0, flags);

    clearEvec(rose, scratch->core_info.exhaustionVector);
    if (rose->ckeyCount) {
        scratch->core_info.logicalVector = scratch->bstate +
                                           rose->stateOffsets.logicalVec;
        scratch->core_info.combVector = scratch->bstate +
                                        rose->stateOffsets.combVec;
        scratch->tctxt.lastCombMatchOffset = 0;
        clearLvec(rose, scratch->core_info.logicalVector,
                  scratch->core_info.combVector);
    }

    if (!length) {
        if (rose->boundary.reportZeroEodOffset) {
            roseRunBoundaryProgram(rose, rose->boundary.reportZeroEodOffset, 0,
                                   scratch);
        }
        goto set_retval;
    }

    if (rose->boundary.reportZeroOffset) {
        int rv = roseRunBoundaryProgram(rose, rose->boundary.reportZeroOffset,
                                        0, scratch);
        if (rv == MO_HALT_MATCHING) {
            goto set_retval;
        }
    }

    if (rose->minWidthExcludingBoundaries > length) {
        DEBUG_PRINTF("minWidthExcludingBoundaries=%u > length=%u\n",
                     rose->minWidthExcludingBoundaries, length);
        goto done_scan;
    }

    // Similarly, we may have a maximum width (for engines constructed entirely
    // of bi-anchored patterns).
    if (rose->maxBiAnchoredWidth != ROSE_BOUND_INF
        && length > rose->maxBiAnchoredWidth) {
        DEBUG_PRINTF("block len=%u longer than maxBAWidth=%u\n", length,
                     rose->maxBiAnchoredWidth);
        goto done_scan;
    }

    // Is this a small write case?
    if (rose->smallWriteOffset) {
        const struct SmallWriteEngine *smwr = getSmallWrite(rose);
        assert(smwr);

        // Apply the small write engine if and only if the block (buffer) is
        // small enough. Otherwise, we allow rose &co to deal with it.
        if (length < smwr->largestBuffer) {
            DEBUG_PRINTF("Attempting small write of block %u bytes long.\n",
                         length);
            runSmallWriteEngine(smwr, scratch);
            goto done_scan;
        }
    }

    switch (rose->runtimeImpl) {
    default:
        assert(0);
    case ROSE_RUNTIME_FULL_ROSE:
        rawBlockExec(rose, scratch);
        break;
    case ROSE_RUNTIME_PURE_LITERAL:
        pureLiteralBlockExec(rose, scratch);
        break;
    case ROSE_RUNTIME_SINGLE_OUTFIX:
        soleOutfixBlockExec(rose, scratch);
        break;
    }

done_scan:
    if (unlikely(internal_matching_error(scratch))) {
        unmarkScratchInUse(scratch);
        return HS_UNKNOWN_ERROR;
    } else if (told_to_stop_matching(scratch)) {
        unmarkScratchInUse(scratch);
        return HS_SCAN_TERMINATED;
    }

    if (rose->hasSom) {
        int halt = flushStoredSomMatches(scratch, ~0ULL);
        if (halt) {
            unmarkScratchInUse(scratch);
            return HS_SCAN_TERMINATED;
        }
    }

    if (rose->boundary.reportEodOffset) {
        roseRunBoundaryProgram(rose, rose->boundary.reportEodOffset, length,
                               scratch);
    }

set_retval:
    if (unlikely(internal_matching_error(scratch))) {
        unmarkScratchInUse(scratch);
        return HS_UNKNOWN_ERROR;
    }

    if (rose->lastFlushCombProgramOffset) {
        if (roseRunLastFlushCombProgram(rose, scratch, length)
            == MO_HALT_MATCHING) {
            if (unlikely(internal_matching_error(scratch))) {
                unmarkScratchInUse(scratch);
                return HS_UNKNOWN_ERROR;
            }
            unmarkScratchInUse(scratch);
            return HS_SCAN_TERMINATED;
        }
    }

    DEBUG_PRINTF("done. told_to_stop_matching=%d\n",
                 told_to_stop_matching(scratch));
    hs_error_t rv = told_to_stop_matching(scratch) ? HS_SCAN_TERMINATED
                                                   : HS_SUCCESS;
    unmarkScratchInUse(scratch);
    return rv;
}

static really_inline
void maintainHistoryBuffer(const struct RoseEngine *rose, char *state,
                           const char *buffer, size_t length) {
    if (!rose->historyRequired) {
        return;
    }

    // Hopefully few of our users are scanning no data.
    if (unlikely(length == 0)) {
        DEBUG_PRINTF("zero-byte scan\n");
        return;
    }

    char *his_state = state + rose->stateOffsets.history;

    if (length < rose->historyRequired) {
        size_t shortfall = rose->historyRequired - length;
        memmove(his_state, his_state + rose->historyRequired - shortfall,
                shortfall);
    }
    size_t amount = MIN(rose->historyRequired, length);

    memcpy(his_state + rose->historyRequired - amount, buffer + length - amount,
           amount);
#ifdef DEBUG_HISTORY
    printf("History [%u] : ", rose->historyRequired);
    for (size_t i = 0; i < rose->historyRequired; i++) {
        printf(" %02hhx", his_state[i]);
    }
    printf("\n");
#endif
}

static really_inline
void init_stream(struct hs_stream *s, const struct RoseEngine *rose,
                 char init_history) {
    char *state = getMultiState(s);

    if (init_history) {
        // Make absolutely sure that the 16 bytes leading up to the end of the
        // history buffer are initialised, as we rely on this (regardless of the
        // actual values used) in FDR.
        char *hist_end =
            state + rose->stateOffsets.history + rose->historyRequired;
        assert(hist_end - 16 >= (const char *)s);
        memset(hist_end - 16, 0x5a, 16);
    }

    s->rose = rose;
    s->offset = 0;

    setStreamStatus(state, 0);
    roseInitState(rose, state);

    clearEvec(rose, state + rose->stateOffsets.exhausted);
    if (rose->ckeyCount) {
        clearLvec(rose, state + rose->stateOffsets.logicalVec,
                  state + rose->stateOffsets.combVec);
    }

    // SOM state multibit structures.
    initSomState(rose, state);
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_open_stream(const hs_database_t *db,
                                   UNUSED unsigned flags,
                                   hs_stream_t **stream) {
    if (unlikely(!stream)) {
        return HS_INVALID;
    }

    *stream = NULL;

    hs_error_t err = validDatabase(db);
    if (unlikely(err != HS_SUCCESS)) {
        return err;
    }

    const struct RoseEngine *rose = hs_get_bytecode(db);
    if (unlikely(!ISALIGNED_16(rose))) {
        return HS_INVALID;
    }

    if (unlikely(rose->mode != HS_MODE_STREAM)) {
        return HS_DB_MODE_ERROR;
    }

    size_t stateSize = rose->stateOffsets.end;
    struct hs_stream *s = hs_stream_alloc(sizeof(struct hs_stream) + stateSize);
    if (unlikely(!s)) {
        return HS_NOMEM;
    }

    init_stream(s, rose, 1);

    *stream = s;
    return HS_SUCCESS;
}


static really_inline
void rawEodExec(hs_stream_t *id, hs_scratch_t *scratch) {
    const struct RoseEngine *rose = id->rose;

    if (can_stop_matching(scratch)) {
        DEBUG_PRINTF("stream already broken\n");
        return;
    }

    if (isAllExhausted(rose, scratch->core_info.exhaustionVector)) {
        DEBUG_PRINTF("stream exhausted\n");
        return;
    }

    roseStreamEodExec(rose, id->offset, scratch);
}

static never_inline
void soleOutfixEodExec(hs_stream_t *id, hs_scratch_t *scratch) {
    const struct RoseEngine *t = id->rose;

    if (can_stop_matching(scratch)) {
        DEBUG_PRINTF("stream already broken\n");
        return;
    }

    if (isAllExhausted(t, scratch->core_info.exhaustionVector)) {
        DEBUG_PRINTF("stream exhausted\n");
        return;
    }

    assert(t->outfixEndQueue == 1);
    assert(!t->amatcherOffset);
    assert(!t->ematcherOffset);
    assert(!t->fmatcherOffset);

    const struct NFA *nfa = getNfaByQueue(t, 0);

    struct mq *q = scratch->queues;
    initOutfixQueue(q, 0, t, scratch);
    if (!scratch->core_info.buf_offset) {
        DEBUG_PRINTF("buf_offset is zero\n");
        return; /* no vacuous engines */
    }

    nfaExpandState(nfa, q->state, q->streamState, q->offset,
                   queue_prev_byte(q, 0));

    assert(nfaAcceptsEod(nfa));
    nfaCheckFinalState(nfa, q->state, q->streamState, q->offset, q->cb,
                       scratch);
}

static really_inline
void report_eod_matches(hs_stream_t *id, hs_scratch_t *scratch,
                        match_event_handler onEvent, void *context) {
    DEBUG_PRINTF("--- report eod matches at offset %llu\n", id->offset);
    assert(onEvent);

    const struct RoseEngine *rose = id->rose;
    char *state = getMultiState(id);
    u8 status = getStreamStatus(state);

    if (status & (STATUS_TERMINATED | STATUS_EXHAUSTED | STATUS_ERROR)) {
        DEBUG_PRINTF("stream is broken, just freeing storage\n");
        return;
    }

    populateCoreInfo(scratch, rose, state, onEvent, context, NULL, 0,
                     getHistory(state, rose, id->offset),
                     getHistoryAmount(rose, id->offset), id->offset, status, 0);

    if (rose->ckeyCount) {
        scratch->core_info.logicalVector = state +
                                           rose->stateOffsets.logicalVec;
        scratch->core_info.combVector = state + rose->stateOffsets.combVec;
        if (!id->offset) {
            scratch->tctxt.lastCombMatchOffset = id->offset;
        }
    }

    if (rose->somLocationCount) {
        loadSomFromStream(scratch, id->offset);
    }

    if (!id->offset) {
        if (rose->boundary.reportZeroEodOffset) {
            int rv = roseRunBoundaryProgram(
                rose, rose->boundary.reportZeroEodOffset, 0, scratch);
            if (rv == MO_HALT_MATCHING) {
                return;
            }
        }
    } else {
        if (rose->boundary.reportEodOffset) {
            int rv = roseRunBoundaryProgram(
                rose, rose->boundary.reportEodOffset, id->offset, scratch);
            if (rv == MO_HALT_MATCHING) {
                return;
            }
        }

        if (rose->requiresEodCheck) {
            switch (rose->runtimeImpl) {
            default:
            case ROSE_RUNTIME_PURE_LITERAL:
                assert(0);
            case ROSE_RUNTIME_FULL_ROSE:
                rawEodExec(id, scratch);
                break;
            case ROSE_RUNTIME_SINGLE_OUTFIX:
                soleOutfixEodExec(id, scratch);
                break;
            }
        }
    }

    if (rose->hasSom && !told_to_stop_matching(scratch)) {
        int halt = flushStoredSomMatches(scratch, ~0ULL);
        if (halt) {
            DEBUG_PRINTF("told to stop matching\n");
            scratch->core_info.status |= STATUS_TERMINATED;
        }
    }

    if (rose->lastFlushCombProgramOffset && !told_to_stop_matching(scratch)) {
        if (roseRunLastFlushCombProgram(rose, scratch, id->offset)
            == MO_HALT_MATCHING) {
            DEBUG_PRINTF("told to stop matching\n");
            scratch->core_info.status |= STATUS_TERMINATED;
        }
    }
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_copy_stream(hs_stream_t **to_id,
                                   const hs_stream_t *from_id) {
    if (!to_id) {
        return HS_INVALID;
    }

    *to_id = NULL;

    if (!from_id || !from_id->rose) {
        return HS_INVALID;
    }

    const struct RoseEngine *rose = from_id->rose;
    size_t stateSize = sizeof(struct hs_stream) + rose->stateOffsets.end;

    struct hs_stream *s = hs_stream_alloc(stateSize);
    if (!s) {
        return HS_NOMEM;
    }

    memcpy(s, from_id, stateSize);

    *to_id = s;

    return HS_SUCCESS;
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_reset_and_copy_stream(hs_stream_t *to_id,
                                             const hs_stream_t *from_id,
                                             hs_scratch_t *scratch,
                                             match_event_handler onEvent,
                                             void *context) {
    if (!from_id || !from_id->rose) {
        return HS_INVALID;
    }

    if (!to_id || to_id->rose != from_id->rose) {
        return HS_INVALID;
    }

    if (to_id == from_id) {
        return HS_INVALID;
    }

    if (onEvent) {
        if (!scratch || !validScratch(to_id->rose, scratch)) {
            return HS_INVALID;
        }
        if (unlikely(markScratchInUse(scratch))) {
            return HS_SCRATCH_IN_USE;
        }
        report_eod_matches(to_id, scratch, onEvent, context);
        if (unlikely(internal_matching_error(scratch))) {
            unmarkScratchInUse(scratch);
            return HS_UNKNOWN_ERROR;
        }
        unmarkScratchInUse(scratch);
    }

    size_t stateSize
        = sizeof(struct hs_stream) + from_id->rose->stateOffsets.end;

    memcpy(to_id, from_id, stateSize);

    return HS_SUCCESS;
}

static really_inline
void rawStreamExec(struct hs_stream *stream_state, struct hs_scratch *scratch) {
    assert(stream_state);
    assert(scratch);
    assert(!can_stop_matching(scratch));

    DEBUG_PRINTF("::: streaming rose ::: offset = %llu len = %zu\n",
                 stream_state->offset, scratch->core_info.len);

    const struct RoseEngine *rose = stream_state->rose;
    assert(rose);
    roseStreamExec(rose, scratch);

    if (!told_to_stop_matching(scratch) &&
        isAllExhausted(rose, scratch->core_info.exhaustionVector)) {
        DEBUG_PRINTF("stream exhausted\n");
        scratch->core_info.status |= STATUS_EXHAUSTED;
    }
}

static really_inline
void pureLiteralStreamExec(struct hs_stream *stream_state,
                           struct hs_scratch *scratch) {
    assert(stream_state);
    assert(scratch);
    assert(!can_stop_matching(scratch));

    const struct RoseEngine *rose = stream_state->rose;
    const struct HWLM *ftable = getFLiteralMatcher(rose);

    size_t len2 = scratch->core_info.len;

    DEBUG_PRINTF("::: streaming rose ::: offset = %llu len = %zu\n",
                 stream_state->offset, scratch->core_info.len);

    pureLiteralInitScratch(scratch, stream_state->offset);
    scratch->tctxt.groups = loadGroups(rose, scratch->core_info.state);

    // Pure literal cases don't have floatingMinDistance set, so we always
    // start the match region at zero.
    const size_t start = 0;

    hwlmExecStreaming(ftable, len2, start, roseCallback, scratch,
                      rose->initialGroups & rose->floating_group_mask);

    if (!told_to_stop_matching(scratch) &&
        isAllExhausted(rose, scratch->core_info.exhaustionVector)) {
        DEBUG_PRINTF("stream exhausted\n");
        scratch->core_info.status |= STATUS_EXHAUSTED;
    }
}

static never_inline
void soleOutfixStreamExec(struct hs_stream *stream_state,
                          struct hs_scratch *scratch) {
    assert(stream_state);
    assert(scratch);
    assert(!can_stop_matching(scratch));

    const struct RoseEngine *t = stream_state->rose;
    assert(t->outfixEndQueue == 1);
    assert(!t->amatcherOffset);
    assert(!t->ematcherOffset);
    assert(!t->fmatcherOffset);

    const struct NFA *nfa = getNfaByQueue(t, 0);

    struct mq *q = scratch->queues;
    initOutfixQueue(q, 0, t, scratch);
    if (!scratch->core_info.buf_offset) {
        nfaQueueInitState(nfa, q);
        pushQueueAt(q, 0, MQE_START, 0);
        pushQueueAt(q, 1, MQE_TOP, 0);
        pushQueueAt(q, 2, MQE_END, scratch->core_info.len);
    } else {
        nfaExpandState(nfa, q->state, q->streamState, q->offset,
                       queue_prev_byte(q, 0));
        pushQueueAt(q, 0, MQE_START, 0);
        pushQueueAt(q, 1, MQE_END, scratch->core_info.len);
    }

    if (nfaQueueExec(q->nfa, q, scratch->core_info.len)) {
        nfaQueueCompressState(nfa, q, scratch->core_info.len);
    } else if (!told_to_stop_matching(scratch)) {
        scratch->core_info.status |= STATUS_EXHAUSTED;
    }
}

static inline
hs_error_t hs_scan_stream_internal(hs_stream_t *id, const char *data,
                                   unsigned length, UNUSED unsigned flags,
                                   hs_scratch_t *scratch,
                                   match_event_handler onEvent, void *context) {
    assert(id);
    assert(scratch);

    if (unlikely(!data)) {
        return HS_INVALID;
    }

    const struct RoseEngine *rose = id->rose;
    char *state = getMultiState(id);

    u8 status = getStreamStatus(state);
    if (status & (STATUS_TERMINATED | STATUS_EXHAUSTED | STATUS_ERROR)) {
        DEBUG_PRINTF("stream is broken, halting scan\n");
        if (status & STATUS_ERROR) {
            return HS_UNKNOWN_ERROR;
        } else if (status & STATUS_TERMINATED) {
            return HS_SCAN_TERMINATED;
        } else {
            return HS_SUCCESS;
        }
    }

    // We avoid doing any work if the user has given us zero bytes of data to
    // scan. Arguably we should define some semantics for how we treat vacuous
    // cases here.
    if (unlikely(length == 0)) {
        DEBUG_PRINTF("zero length block\n");
        return HS_SUCCESS;
    }

    u32 historyAmount = getHistoryAmount(rose, id->offset);
    populateCoreInfo(scratch, rose, state, onEvent, context, data, length,
                     getHistory(state, rose, id->offset), historyAmount,
                     id->offset, status, flags);
    if (rose->ckeyCount) {
        scratch->core_info.logicalVector = state +
                                           rose->stateOffsets.logicalVec;
        scratch->core_info.combVector = state + rose->stateOffsets.combVec;
        if (!id->offset) {
            scratch->tctxt.lastCombMatchOffset = id->offset;
        }
    }
    assert(scratch->core_info.hlen <= id->offset
           && scratch->core_info.hlen <= rose->historyRequired);

    prefetch_data(data, length);

    if (rose->somLocationCount) {
        loadSomFromStream(scratch, id->offset);
    }

    if (!id->offset && rose->boundary.reportZeroOffset) {
        DEBUG_PRINTF("zero reports\n");
        int rv = roseRunBoundaryProgram(rose, rose->boundary.reportZeroOffset,
                                        0, scratch);
        if (rv == MO_HALT_MATCHING) {
            DEBUG_PRINTF("halting scan\n");
            setStreamStatus(state, scratch->core_info.status);
            if (told_to_stop_matching(scratch)) {
                return HS_SCAN_TERMINATED;
            } else {
                assert(scratch->core_info.status & STATUS_EXHAUSTED);
                return HS_SUCCESS;
            }
        }
    }

    switch (rose->runtimeImpl) {
    default:
        assert(0);
    case ROSE_RUNTIME_FULL_ROSE:
        rawStreamExec(id, scratch);
        break;
    case ROSE_RUNTIME_PURE_LITERAL:
        pureLiteralStreamExec(id, scratch);
        break;
    case ROSE_RUNTIME_SINGLE_OUTFIX:
        soleOutfixStreamExec(id, scratch);
    }

    if (rose->hasSom && !told_to_stop_matching(scratch)) {
        int halt = flushStoredSomMatches(scratch, ~0ULL);
        if (halt) {
            scratch->core_info.status |= STATUS_TERMINATED;
        }
    }

    setStreamStatus(state, scratch->core_info.status);

    if (unlikely(internal_matching_error(scratch))) {
        return HS_UNKNOWN_ERROR;
    } else if (likely(!can_stop_matching(scratch))) {
        maintainHistoryBuffer(rose, state, data, length);
        id->offset += length; /* maintain offset */

        if (rose->somLocationCount) {
            storeSomToStream(scratch, id->offset);
        }
    } else if (told_to_stop_matching(scratch)) {
        return HS_SCAN_TERMINATED;
    }

    return HS_SUCCESS;
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_scan_stream(hs_stream_t *id, const char *data,
                                   unsigned length, unsigned flags,
                                   hs_scratch_t *scratch,
                                   match_event_handler onEvent, void *context) {
    if (unlikely(!id || !scratch || !data ||
                 !validScratch(id->rose, scratch))) {
        return HS_INVALID;
    }

    if (unlikely(markScratchInUse(scratch))) {
        return HS_SCRATCH_IN_USE;
    }
    hs_error_t rv = hs_scan_stream_internal(id, data, length, flags, scratch,
                                            onEvent, context);
    unmarkScratchInUse(scratch);
    return rv;
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_close_stream(hs_stream_t *id, hs_scratch_t *scratch,
                                    match_event_handler onEvent,
                                    void *context) {
    if (!id) {
        return HS_INVALID;
    }

    if (onEvent) {
        if (!scratch || !validScratch(id->rose, scratch)) {
            return HS_INVALID;
        }
        if (unlikely(markScratchInUse(scratch))) {
            return HS_SCRATCH_IN_USE;
        }
        report_eod_matches(id, scratch, onEvent, context);
        if (unlikely(internal_matching_error(scratch))) {
            unmarkScratchInUse(scratch);
            hs_stream_free(id);
            return HS_UNKNOWN_ERROR;
        }
        unmarkScratchInUse(scratch);
    }

    hs_stream_free(id);

    return HS_SUCCESS;
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_reset_stream(hs_stream_t *id, UNUSED unsigned int flags,
                                    hs_scratch_t *scratch,
                                    match_event_handler onEvent,
                                    void *context) {
    if (!id) {
        return HS_INVALID;
    }

    if (onEvent) {
        if (!scratch || !validScratch(id->rose, scratch)) {
            return HS_INVALID;
        }
        if (unlikely(markScratchInUse(scratch))) {
            return HS_SCRATCH_IN_USE;
        }
        report_eod_matches(id, scratch, onEvent, context);
        if (unlikely(internal_matching_error(scratch))) {
            unmarkScratchInUse(scratch);
            return HS_UNKNOWN_ERROR;
        }
        unmarkScratchInUse(scratch);
    }

    // history already initialised
    init_stream(id, id->rose, 0);

    return HS_SUCCESS;
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_stream_size(const hs_database_t *db,
                                   size_t *stream_size) {
    if (!stream_size) {
        return HS_INVALID;
    }

    hs_error_t ret = validDatabase(db);
    if (ret != HS_SUCCESS) {
        return ret;
    }

    const struct RoseEngine *rose = hs_get_bytecode(db);
    if (!ISALIGNED_16(rose)) {
        return HS_INVALID;
    }

    if (rose->mode != HS_MODE_STREAM) {
        return HS_DB_MODE_ERROR;
    }

    u32 base_stream_size = rose->stateOffsets.end;

    // stream state plus the hs_stream struct itself
    *stream_size = base_stream_size + sizeof(struct hs_stream);

    return HS_SUCCESS;
}

#if defined(DEBUG) || defined(DUMP_SUPPORT)
#include "util/compare.h"
// A debugging crutch: print a hex-escaped version of the match for our
// perusal.
static UNUSED
void dumpData(const char *data, size_t len) {
    DEBUG_PRINTF("BUFFER:");
    for (size_t i = 0; i < len; i++) {
        u8 c = data[i];
        if (ourisprint(c) && c != '\'') {
            printf("%c", c);
        } else {
            printf("\\x%02x", c);
        }
    }
    printf("\n");
}
#endif

HS_PUBLIC_API
hs_error_t HS_CDECL hs_scan_vector(const hs_database_t *db,
                                   const char * const * data,
                                   const unsigned int *length,
                                   unsigned int count,
                                   UNUSED unsigned int flags,
                                   hs_scratch_t *scratch,
                                   match_event_handler onEvent, void *context) {
    if (unlikely(!scratch || !data || !length)) {
        return HS_INVALID;
    }

    hs_error_t err = validDatabase(db);
    if (unlikely(err != HS_SUCCESS)) {
        return err;
    }

    const struct RoseEngine *rose = hs_get_bytecode(db);
    if (unlikely(!ISALIGNED_16(rose))) {
        return HS_INVALID;
    }

    if (unlikely(rose->mode != HS_MODE_VECTORED)) {
        return HS_DB_MODE_ERROR;
    }

    if (unlikely(!validScratch(rose, scratch))) {
        return HS_INVALID;
    }

    if (unlikely(markScratchInUse(scratch))) {
        return HS_SCRATCH_IN_USE;
    }

    hs_stream_t *id = (hs_stream_t *)(scratch->bstate);

    init_stream(id, rose, 1); /* open stream */

    for (u32 i = 0; i < count; i++) {
        DEBUG_PRINTF("block %u/%u offset=%llu len=%u\n", i, count, id->offset,
                     length[i]);
#ifdef DEBUG
        dumpData(data[i], length[i]);
#endif
        hs_error_t ret
            = hs_scan_stream_internal(id, data[i], length[i], 0, scratch,
                                      onEvent, context);
        if (ret != HS_SUCCESS) {
            unmarkScratchInUse(scratch);
            return ret;
        }
    }

    /* close stream */
    if (onEvent) {
        report_eod_matches(id, scratch, onEvent, context);

        if (unlikely(internal_matching_error(scratch))) {
            unmarkScratchInUse(scratch);
            return HS_UNKNOWN_ERROR;
        } else if (told_to_stop_matching(scratch)) {
            unmarkScratchInUse(scratch);
            return HS_SCAN_TERMINATED;
        }
    }

    unmarkScratchInUse(scratch);

    return HS_SUCCESS;
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_compress_stream(const hs_stream_t *stream, char *buf,
                                       size_t buf_space, size_t *used_space) {
    if (unlikely(!stream || !used_space)) {
        return HS_INVALID;
    }

    if (unlikely(buf_space && !buf)) {
        return HS_INVALID;
    }

    const struct RoseEngine *rose = stream->rose;

    size_t stream_size = size_compress_stream(rose, stream);

    DEBUG_PRINTF("require %zu [orig %zu]\n", stream_size,
                 rose->stateOffsets.end + sizeof(struct hs_stream));
    *used_space = stream_size;

    if (buf_space < stream_size) {
        return HS_INSUFFICIENT_SPACE;
    }
    compress_stream(buf, stream_size, rose, stream);

    return HS_SUCCESS;
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_expand_stream(const hs_database_t *db,
                                     hs_stream_t **stream,
                                     const char *buf, size_t buf_size) {
    if (unlikely(!stream || !buf)) {
        return HS_INVALID;
    }

    *stream = NULL;

    hs_error_t err = validDatabase(db);
    if (unlikely(err != HS_SUCCESS)) {
        return err;
    }

    const struct RoseEngine *rose = hs_get_bytecode(db);
    if (unlikely(!ISALIGNED_16(rose))) {
        return HS_INVALID;
    }

    if (unlikely(rose->mode != HS_MODE_STREAM)) {
        return HS_DB_MODE_ERROR;
    }

    size_t stream_size = rose->stateOffsets.end + sizeof(struct hs_stream);

    struct hs_stream *s = hs_stream_alloc(stream_size);
    if (unlikely(!s)) {
        return HS_NOMEM;
    }

    if (!expand_stream(s, rose, buf, buf_size)) {
        hs_stream_free(s);
        return HS_INVALID;
    }

    *stream = s;
    return HS_SUCCESS;
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_reset_and_expand_stream(hs_stream_t *to_stream,
                                               const char *buf, size_t buf_size,
                                               hs_scratch_t *scratch,
                                               match_event_handler onEvent,
                                               void *context) {
    if (unlikely(!to_stream || !buf)) {
        return HS_INVALID;
    }

    const struct RoseEngine *rose = to_stream->rose;

    if (onEvent) {
        if (!scratch || !validScratch(to_stream->rose, scratch)) {
            return HS_INVALID;
        }
        if (unlikely(markScratchInUse(scratch))) {
            return HS_SCRATCH_IN_USE;
        }
        report_eod_matches(to_stream, scratch, onEvent, context);
        if (unlikely(internal_matching_error(scratch))) {
            unmarkScratchInUse(scratch);
            return HS_UNKNOWN_ERROR;
        }
        unmarkScratchInUse(scratch);
    }

    if (expand_stream(to_stream, rose, buf, buf_size)) {
        return HS_SUCCESS;
    } else {
        return HS_INVALID;
    }
}
