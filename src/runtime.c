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
#include "smallwrite/smallwrite_internal.h"
#include "rose/rose.h"
#include "rose/runtime.h"
#include "database.h"
#include "scratch.h"
#include "som/som_runtime.h"
#include "som/som_stream.h"
#include "state.h"
#include "ue2common.h"
#include "util/exhaust.h"
#include "util/fatbit.h"
#include "util/multibit.h"

#define DEDUPE_MATCHES

static really_inline
void prefetch_data(const char *data, unsigned length) {
    __builtin_prefetch(data);
    __builtin_prefetch(data + length/2);
    __builtin_prefetch(data + length - 24);
}

/** dummy event handler for use when user does not provide one */
static
int null_onEvent(UNUSED unsigned id, UNUSED unsigned long long from,
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
                      size_t hlen, u64a offset, UNUSED unsigned int flags) {
    assert(rose);
    s->core_info.userContext = userCtx;
    s->core_info.userCallback = onEvent ? onEvent : null_onEvent;
    s->core_info.rose = rose;
    s->core_info.state = state; /* required for chained queues + evec */

    s->core_info.exhaustionVector = state + rose->stateOffsets.exhausted;
    s->core_info.broken = NOT_BROKEN;
    s->core_info.buf = (const u8 *)data;
    s->core_info.len = length;
    s->core_info.hbuf = history;
    s->core_info.hlen = hlen;
    s->core_info.buf_offset = offset;

    /* and some stuff not actually in core info */
    s->som_set_now_offset = ~0ULL;
    s->deduper.current_report_offset = ~0ULL;
    s->deduper.som_log_dirty = 1; /* som logs have not been cleared */
}

/** \brief Query whether this stream is broken.
 *
 * A broken stream is one on which scanning has stopped, either because the
 * user has told us to (via the return value from a match callback) or because
 * we have exhausted all reports.
 *
 * \return NOT_BROKEN, BROKEN_FROM_USER or BROKEN_EXHAUSTED.
 */
static really_inline
u8 getBroken(const char *state) {
    const struct RoseRuntimeState *ts = (const void *)state;
    assert(ts->broken == NOT_BROKEN || ts->broken == BROKEN_FROM_USER
           || ts->broken == BROKEN_EXHAUSTED);
    return ts->broken;
}

/** \brief Mark this stream with the given broken flag.
 *
 * Possible values: NOT_BROKEN, BROKEN_FROM_USER, BROKEN_EXHAUSTED.
 */
static really_inline
void setBroken(char *state, u8 broken) {
    DEBUG_PRINTF("set broken=%d\n", broken);
    assert(broken == NOT_BROKEN || broken == BROKEN_FROM_USER
           || broken == BROKEN_EXHAUSTED);
    struct RoseRuntimeState *ts = (void *)state;
    ts->broken = broken;
}

static really_inline
int roseAdaptor_i(u64a offset, ReportID id, void *context, char is_simple,
                  char do_som) {
    assert(id != MO_INVALID_IDX); // Should never get an invalid ID.

    struct hs_scratch *scratch = (struct hs_scratch *)context;
    struct core_info *ci = &scratch->core_info;
    const struct RoseEngine *rose = ci->rose;
    DEBUG_PRINTF("internal report %u\n", id);
    const struct internal_report *ri = getInternalReport(rose, id);

    assert(isExternalReport(ri)); /* only external reports should reach here */

    s32 offset_adj = ri->offsetAdjust;
    UNUSED u32 dkey = ri->dkey;
    u64a to_offset = offset;
    u64a from_offset = 0;
    UNUSED u32 dkeyCount = rose->dkeyCount;

    u32 flags = 0;
#ifndef RELEASE_BUILD
    if (offset_adj) {
        // alert testing tools that we've got adjusted matches
        flags |= HS_MATCH_FLAG_ADJUSTED;
    }
#endif

    DEBUG_PRINTF("internal match at %llu: IID=%u type=%hhu RID=%u "
                 "offsetAdj=%d\n", offset, id, ri->type, ri->onmatch,
                 offset_adj);

    if (unlikely(can_stop_matching(scratch))) { /* ok - we are from rose */
        DEBUG_PRINTF("pre broken - halting\n");
        return MO_HALT_MATCHING;
    }

    if (!is_simple && ri->hasBounds) {
        assert(ri->minOffset || ri->minLength || ri->maxOffset < MAX_OFFSET);
        assert(ri->minOffset <= ri->maxOffset);
        if (offset < ri->minOffset || offset > ri->maxOffset) {
            DEBUG_PRINTF("match fell outside valid range %llu !: [%llu,%llu]\n",
                         offset, ri->minOffset, ri->maxOffset);
            return ROSE_CONTINUE_MATCHING_NO_EXHAUST;
        }
    }

    if (!is_simple && unlikely(isExhausted(ci->exhaustionVector, ri->ekey))) {
        DEBUG_PRINTF("ate exhausted match\n");
        return MO_CONTINUE_MATCHING;
    }

    if (ri->type == EXTERNAL_CALLBACK) {
        from_offset = 0;
    } else if (do_som) {
        from_offset = handleSomExternal(scratch, ri, to_offset);
    }

    to_offset += offset_adj;
    assert(from_offset == HS_OFFSET_PAST_HORIZON || from_offset <= to_offset);

    if (do_som && ri->minLength) {
        if (from_offset != HS_OFFSET_PAST_HORIZON &&
                (to_offset - from_offset < ri->minLength)) {
            return ROSE_CONTINUE_MATCHING_NO_EXHAUST;
        }
        if (ri->quashSom) {
            from_offset = 0;
        }
    }

    DEBUG_PRINTF(">> reporting match @[%llu,%llu] for sig %u ctxt %p <<\n",
                 from_offset, to_offset, ri->onmatch, ci->userContext);

    int halt = 0;

    if (do_som || dkey != MO_INVALID_IDX) {
        if (offset != scratch->deduper.current_report_offset) {
            assert(scratch->deduper.current_report_offset == ~0ULL ||
                   scratch->deduper.current_report_offset < offset);
            if (offset == scratch->deduper.current_report_offset + 1) {
                fatbit_clear(scratch->deduper.log[offset % 2]);
            } else {
                fatbit_clear(scratch->deduper.log[0]);
                fatbit_clear(scratch->deduper.log[1]);
            }

            DEBUG_PRINTF("adj dedupe offset %hhd\n", do_som);
            if (do_som) {
                halt = flushStoredSomMatches(scratch, offset);
                if (halt) {
                    goto exit;
                }
            }
            scratch->deduper.current_report_offset = offset;
        }
    }

#ifdef DEDUPE_MATCHES
    if (dkey != MO_INVALID_IDX) {
        if (ri->type == EXTERNAL_CALLBACK || ri->quashSom) {
            DEBUG_PRINTF("checking dkey %u at offset %llu\n", dkey, to_offset);
            assert(offset_adj == 0 || offset_adj == -1);
            if (fatbit_set(scratch->deduper.log[to_offset % 2], dkeyCount,
                           dkey)) {
                /* we have already raised this report at this offset, squash dupe
                 * match. */
                DEBUG_PRINTF("dedupe\n");
                goto exit;
            }
        } else if (do_som) {
            /* SOM external event */
            DEBUG_PRINTF("checking dkey %u at offset %llu\n", dkey, to_offset);
            assert(offset_adj == 0 || offset_adj == -1);
            u64a *starts = scratch->deduper.som_start_log[to_offset % 2];
            if (fatbit_set(scratch->deduper.som_log[to_offset % 2], dkeyCount,
                           dkey)) {
                starts[dkey] = MIN(starts[dkey], from_offset);
            } else {
                starts[dkey] = from_offset;
            }

            if (offset_adj) {
                scratch->deduper.som_log_dirty |= 1;
            } else {
                scratch->deduper.som_log_dirty |= 2;
            }

            goto exit;
        }
    }
#endif

    halt = ci->userCallback((unsigned int)ri->onmatch, from_offset, to_offset,
                            flags, ci->userContext);
#ifdef DEDUPE_MATCHES
exit:
#endif
    if (halt) {
        DEBUG_PRINTF("callback requested to terminate matches\n");

        setBroken(ci->state, BROKEN_FROM_USER);
        ci->broken = BROKEN_FROM_USER;

        return MO_HALT_MATCHING;
    }

    if (!is_simple && ri->ekey != END_EXHAUST) {
        markAsMatched(ci->exhaustionVector, ri->ekey);
        return MO_CONTINUE_MATCHING;
    } else {
        return ROSE_CONTINUE_MATCHING_NO_EXHAUST;
    }
}

static really_inline
int roseSomAdaptor_i(u64a from_offset, u64a to_offset, ReportID id,
                     void *context, char is_simple) {
    assert(id != MO_INVALID_IDX); // Should never get an invalid ID.

    u32 flags = 0;

    struct hs_scratch *scratch = (struct hs_scratch *)context;
    struct core_info *ci = &scratch->core_info;
    const struct RoseEngine *rose = ci->rose;
    const struct internal_report *ri = getInternalReport(rose, id);

    /* internal events should be handled by rose directly */
    assert(ri->type == EXTERNAL_CALLBACK);

    DEBUG_PRINTF("internal match at %llu: IID=%u type=%hhu RID=%u "
                 "offsetAdj=%d\n", to_offset, id, ri->type, ri->onmatch,
                 ri->offsetAdjust);

    if (unlikely(can_stop_matching(scratch))) {
        DEBUG_PRINTF("pre broken - halting\n");
        return MO_HALT_MATCHING;
    }

    if (!is_simple && ri->hasBounds) {
        assert(ri->minOffset || ri->minLength || ri->maxOffset < MAX_OFFSET);
        if (to_offset < ri->minOffset || to_offset > ri->maxOffset) {
            DEBUG_PRINTF("match fell outside valid range %llu !: [%llu,%llu]\n",
                         to_offset, ri->minOffset, ri->maxOffset);
            return MO_CONTINUE_MATCHING;
        }
    }

    int halt = 0;

    if (!is_simple && unlikely(isExhausted(ci->exhaustionVector, ri->ekey))) {
        DEBUG_PRINTF("ate exhausted match\n");
        goto do_return;
    }

#ifdef DEDUPE_MATCHES
    u64a offset = to_offset;
#endif

    to_offset += ri->offsetAdjust;
    assert(from_offset == HS_OFFSET_PAST_HORIZON || from_offset <= to_offset);

    if (!is_simple && ri->minLength) {
        if (from_offset != HS_OFFSET_PAST_HORIZON &&
                (to_offset - from_offset < ri->minLength)) {
            return MO_CONTINUE_MATCHING;
        }
        if (ri->quashSom) {
            from_offset = 0;
        }
    }

    DEBUG_PRINTF(">> reporting match @[%llu,%llu] for sig %u ctxt %p <<\n",
                 from_offset, to_offset, ri->onmatch, ci->userContext);

#ifndef RELEASE_BUILD
    if (ri->offsetAdjust != 0) {
        // alert testing tools that we've got adjusted matches
        flags |= HS_MATCH_FLAG_ADJUSTED;
    }
#endif

#ifdef DEDUPE_MATCHES
    u32 dkeyCount = rose->dkeyCount;

    if (offset != scratch->deduper.current_report_offset) {

        assert(scratch->deduper.current_report_offset == ~0ULL
               || scratch->deduper.current_report_offset < offset);
        if (offset == scratch->deduper.current_report_offset + 1) {
            fatbit_clear(scratch->deduper.log[offset % 2]);
        } else {
            fatbit_clear(scratch->deduper.log[0]);
            fatbit_clear(scratch->deduper.log[1]);
        }

        halt = flushStoredSomMatches(scratch, offset);
        if (halt) {
            goto do_return;
        }

        scratch->deduper.current_report_offset = offset;
    }

    u32 dkey = ri->dkey;
    if (dkey != MO_INVALID_IDX) {
        if (ri->quashSom) {
            DEBUG_PRINTF("checking dkey %u at offset %llu\n", dkey, to_offset);
            assert(ri->offsetAdjust == 0 || ri->offsetAdjust == -1);
            if (fatbit_set(scratch->deduper.log[to_offset % 2], dkeyCount,
                           dkey)) {
                /* we have already raised this report at this offset, squash
                 * dupe match. */
                DEBUG_PRINTF("dedupe\n");
                goto do_return;
            }
        } else {
            /* SOM external event */
            DEBUG_PRINTF("checking dkey %u at offset %llu\n", dkey, to_offset);
            assert(ri->offsetAdjust == 0 || ri->offsetAdjust == -1);
            u64a *starts = scratch->deduper.som_start_log[to_offset % 2];
            if (fatbit_set(scratch->deduper.som_log[to_offset % 2], dkeyCount,
                           dkey)) {
                starts[dkey] = MIN(starts[dkey], from_offset);
            } else {
                starts[dkey] = from_offset;
            }

            if (ri->offsetAdjust) {
                scratch->deduper.som_log_dirty |= 1;
            } else {
                scratch->deduper.som_log_dirty |= 2;
            }

            goto do_return;
        }
    }
#endif

    halt = ci->userCallback((unsigned int)ri->onmatch, from_offset, to_offset,
                            flags, ci->userContext);

    if (!is_simple) {
        markAsMatched(ci->exhaustionVector, ri->ekey);
    }

do_return:
    if (halt) {
        DEBUG_PRINTF("callback requested to terminate matches\n");

        setBroken(ci->state, BROKEN_FROM_USER);
        ci->broken = BROKEN_FROM_USER;

        return MO_HALT_MATCHING;
    }

    return MO_CONTINUE_MATCHING;
}

static really_inline
hwlmcb_rv_t multiDirectAdaptor(u64a real_end, ReportID direct_id, void *context,
                               struct core_info *ci, char is_simple,
                               char do_som) {
    // Multi-direct report, list of reports indexed by the ID.
    u32 mdr_offset = direct_id & ~LITERAL_MDR_FLAG;
    const struct RoseEngine *t = ci->rose;
    const ReportID *id
        = (const ReportID *)((const char *)t + t->multidirectOffset)
        + mdr_offset;
    for (; *id != MO_INVALID_IDX; id++) {
        int rv = roseAdaptor_i(real_end, *id, context, is_simple, do_som);
        if (rv == MO_HALT_MATCHING) {
            return HWLM_TERMINATE_MATCHING;
        }
    }
    return HWLM_CONTINUE_MATCHING;
}

static
int roseAdaptor(u64a offset, ReportID id, void *context) {
    return roseAdaptor_i(offset, id, context, 0, 0);
}

static
hwlmcb_rv_t hwlmAdaptor(UNUSED size_t start, size_t end, u32 direct_id,
                        void *context) {
    struct hs_scratch *scratch = (struct hs_scratch *)context;
    struct core_info *ci = &scratch->core_info;
    u64a real_end = (u64a)end + ci->buf_offset + 1;

    if (isLiteralMDR(direct_id)) {
        return multiDirectAdaptor(real_end, direct_id, context, ci, 0, 0);
    }

    ReportID id = literalToReport(direct_id);
    int rv = roseAdaptor_i(real_end, id, context, 0, 0);
    if (rv == MO_CONTINUE_MATCHING || rv == ROSE_CONTINUE_MATCHING_NO_EXHAUST) {
        return HWLM_CONTINUE_MATCHING;
    } else {
        return HWLM_TERMINATE_MATCHING;
    }
}

static
int roseSimpleAdaptor(u64a offset, ReportID id, void *context) {
    return roseAdaptor_i(offset, id, context, 1, 0);
}

static
hwlmcb_rv_t hwlmSimpleAdaptor(UNUSED size_t start, size_t end, u32 direct_id,
                              void *context) {
    struct hs_scratch *scratch = (struct hs_scratch *)context;
    struct core_info *ci = &scratch->core_info;
    u64a real_end = (u64a)end + ci->buf_offset + 1;

    if (isLiteralMDR(direct_id)) {
        return multiDirectAdaptor(real_end, direct_id, context, ci, 1, 0);
    }

    // Single direct report.
    ReportID id = literalToReport(direct_id);
    int rv = roseAdaptor_i(real_end, id, context, 1, 0);
    if (rv == MO_CONTINUE_MATCHING || rv == ROSE_CONTINUE_MATCHING_NO_EXHAUST) {
        return HWLM_CONTINUE_MATCHING;
    } else {
        return HWLM_TERMINATE_MATCHING;
    }
}

static
int roseSomAdaptor(u64a offset, ReportID id, void *context) {
    return roseAdaptor_i(offset, id, context, 0, 1);
}

static
hwlmcb_rv_t hwlmSomAdaptor(UNUSED size_t start, size_t end, u32 direct_id,
                           void *context) {
    struct hs_scratch *scratch = (struct hs_scratch *)context;
    struct core_info *ci = &scratch->core_info;
    u64a real_end = (u64a)end + ci->buf_offset + 1;

    if (isLiteralMDR(direct_id)) {
        return multiDirectAdaptor(real_end, direct_id, context, ci, 0, 1);
    }

    ReportID id = literalToReport(direct_id);
    int rv = roseAdaptor_i(real_end, id, context, 0, 1);
    if (rv == MO_CONTINUE_MATCHING || rv == ROSE_CONTINUE_MATCHING_NO_EXHAUST) {
        return HWLM_CONTINUE_MATCHING;
    } else {
        return HWLM_TERMINATE_MATCHING;
    }
}

static
int roseSimpleSomAdaptor(u64a offset, ReportID id, void *context) {
    return roseAdaptor_i(offset, id, context, 1, 1);
}

static
hwlmcb_rv_t hwlmSimpleSomAdaptor(UNUSED size_t start, size_t end, u32 direct_id,
                                 void *context) {
    struct hs_scratch *scratch = (struct hs_scratch *)context;
    struct core_info *ci = &scratch->core_info;
    u64a real_end = (u64a)end + ci->buf_offset + 1;

    if (isLiteralMDR(direct_id)) {
        return multiDirectAdaptor(real_end, direct_id, context, ci, 1, 1);
    }

    ReportID id = literalToReport(direct_id);
    int rv = roseAdaptor_i(real_end, id, context, 1, 1);
    if (rv == MO_CONTINUE_MATCHING || rv == ROSE_CONTINUE_MATCHING_NO_EXHAUST) {
        return HWLM_CONTINUE_MATCHING;
    } else {
        return HWLM_TERMINATE_MATCHING;
    }
}

static really_inline
RoseCallback selectAdaptor(const struct RoseEngine *rose) {
    const char is_simple = rose->simpleCallback;
    const char do_som = rose->hasSom;

    if (do_som) {
        return is_simple ? roseSimpleSomAdaptor : roseSomAdaptor;
    } else {
        return is_simple ? roseSimpleAdaptor : roseAdaptor;
    }
}

static really_inline
HWLMCallback selectHwlmAdaptor(const struct RoseEngine *rose) {
    const char is_simple = rose->simpleCallback;
    const char do_som = rose->hasSom;

    if (do_som) {
        return is_simple ? hwlmSimpleSomAdaptor : hwlmSomAdaptor;
    } else {
        return is_simple ? hwlmSimpleAdaptor : hwlmAdaptor;
    }
}

static
int roseSomSomAdaptor(u64a from_offset, u64a to_offset, ReportID id,
                       void *context) {
    return roseSomAdaptor_i(from_offset, to_offset, id, context, 0);
}

static
int roseSimpleSomSomAdaptor(u64a from_offset, u64a to_offset, ReportID id,
                             void *context) {
    return roseSomAdaptor_i(from_offset, to_offset, id, context, 1);
}

static really_inline
RoseCallbackSom selectSomAdaptor(const struct RoseEngine *rose) {
    const char is_simple = rose->simpleCallback;

    return is_simple ? roseSimpleSomSomAdaptor : roseSomSomAdaptor;
}

static never_inline
void processReportList(const struct RoseEngine *rose, u32 base_offset,
                       u64a stream_offset, hs_scratch_t *scratch) {
    DEBUG_PRINTF("running report list at offset %u\n", base_offset);

    if (told_to_stop_matching(scratch)) {
        DEBUG_PRINTF("matching has been terminated\n");
        return;
    }

    if (rose->hasSom && scratch->deduper.current_report_offset == ~0ULL) {
        /* we cannot delay the initialization of the som deduper logs any longer
         * as we are reporting matches. This is done explicitly as we are
         * shortcutting the som handling in the vacuous repeats as we know they
         * all come from non-som patterns. */

        fatbit_clear(scratch->deduper.som_log[0]);
        fatbit_clear(scratch->deduper.som_log[1]);
        scratch->deduper.som_log_dirty = 0;
    }

    const ReportID *report =
        (const ReportID *)((const char *)rose + base_offset);

    /* never required to do som as vacuous reports are always external */

    if (rose->simpleCallback) {
        for (; *report != MO_INVALID_IDX; report++) {
            roseSimpleAdaptor(stream_offset, *report, scratch);
        }
    } else {
        for (; *report != MO_INVALID_IDX; report++) {
            roseAdaptor(stream_offset, *report, scratch);
        }
    }
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

    roseBlockExec(rose, scratch, selectAdaptor(rose),
                  selectSomAdaptor(rose), scratch);
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

    hwlmExec(ftable, buffer, length, 0, selectHwlmAdaptor(rose), scratch,
             rose->initialGroups);
}

static really_inline
void initQueue(struct mq *q, u32 qi, const struct RoseEngine *t,
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
    q->cb = selectAdaptor(t);
    q->som_cb = selectSomAdaptor(t);
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
    initQueue(q, 0, t, scratch);
    q->length = len; /* adjust for rev_accel */
    nfaQueueInitState(nfa, q);
    pushQueueAt(q, 0, MQE_START, 0);
    pushQueueAt(q, 1, MQE_TOP, 0);
    pushQueueAt(q, 2, MQE_END, scratch->core_info.len);

    char rv = nfaQueueExec(q->nfa, q, scratch->core_info.len);

    if (rv && nfaAcceptsEod(nfa) && len == scratch->core_info.len) {
        nfaCheckFinalState(nfa, q->state, q->streamState, q->length,
                        q->cb, q->som_cb, scratch);
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

    const struct RoseEngine *rose = scratch->core_info.rose;

    size_t local_alen = length - smwr->start_offset;
    const u8 *local_buffer = buffer + smwr->start_offset;

    assert(isMcClellanType(nfa->type));
    if (nfa->type == MCCLELLAN_NFA_8) {
        nfaExecMcClellan8_B(nfa, smwr->start_offset, local_buffer,
                            local_alen, selectAdaptor(rose), scratch);
    } else {
        nfaExecMcClellan16_B(nfa, smwr->start_offset, local_buffer,
                             local_alen, selectAdaptor(rose), scratch);
    }
}

HS_PUBLIC_API
hs_error_t hs_scan(const hs_database_t *db, const char *data, unsigned length,
                   unsigned flags, hs_scratch_t *scratch,
                   match_event_handler onEvent, void *userCtx) {
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

    if (rose->minWidth > length) {
        DEBUG_PRINTF("minwidth=%u > length=%u\n", rose->minWidth, length);
        return HS_SUCCESS;
    }

    prefetch_data(data, length);

    /* populate core info in scratch */
    populateCoreInfo(scratch, rose, scratch->bstate, onEvent, userCtx, data,
                     length, NULL, 0, 0, flags);

    clearEvec(scratch->core_info.exhaustionVector, rose);

    if (!length) {
        if (rose->boundary.reportZeroEodOffset) {
            processReportList(rose, rose->boundary.reportZeroEodOffset, 0,
                              scratch);
        }
        goto set_retval;
    }

    if (rose->boundary.reportZeroOffset) {
        processReportList(rose, rose->boundary.reportZeroOffset, 0, scratch);
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
    if (told_to_stop_matching(scratch)) {
        return HS_SCAN_TERMINATED;
    }

    if (rose->hasSom) {
        int halt = flushStoredSomMatches(scratch, ~0ULL);
        if (halt) {
            return HS_SCAN_TERMINATED;
        }
    }

    if (rose->boundary.reportEodOffset) {
        processReportList(rose, rose->boundary.reportEodOffset, length, scratch);
    }

set_retval:
    DEBUG_PRINTF("done. told_to_stop_matching=%d\n",
                 told_to_stop_matching(scratch));
    return told_to_stop_matching(scratch) ? HS_SCAN_TERMINATED : HS_SUCCESS;
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
void init_stream(struct hs_stream *s, const struct RoseEngine *rose) {
    s->rose = rose;
    s->offset = 0;

    char *state = getMultiState(s);

    roseInitState(rose, state);

    clearEvec((char *)state + rose->stateOffsets.exhausted, rose);

    // SOM state multibit structures.
    initSomState(rose, state);
}

HS_PUBLIC_API
hs_error_t hs_open_stream(const hs_database_t *db, UNUSED unsigned flags,
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

    init_stream(s, rose);

    *stream = s;
    return HS_SUCCESS;
}


static really_inline
void rawEodExec(hs_stream_t *id, hs_scratch_t *scratch) {
    const struct RoseEngine *rose = id->rose;
    char *state = getMultiState(id);
    u8 broken = getBroken(state);

    if (broken) {
        DEBUG_PRINTF("stream already broken\n");
        assert(broken == BROKEN_FROM_USER || broken == BROKEN_EXHAUSTED);
        return;
    }

    if (isAllExhausted(rose, scratch->core_info.exhaustionVector)) {
        DEBUG_PRINTF("stream exhausted\n");
        return;
    }

    roseEodExec(rose, id->offset, scratch, selectAdaptor(rose),
                selectSomAdaptor(rose), scratch);
}

static never_inline
void soleOutfixEodExec(hs_stream_t *id, hs_scratch_t *scratch) {
    const struct RoseEngine *t = id->rose;
    char *state = getMultiState(id);
    u8 broken = getBroken(state);

    if (broken) {
        DEBUG_PRINTF("stream already broken\n");
        assert(broken == BROKEN_FROM_USER || broken == BROKEN_EXHAUSTED);
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
    initQueue(q, 0, t, scratch);
    if (!scratch->core_info.buf_offset) {
        DEBUG_PRINTF("buf_offset is zero\n");
        return; /* no vacuous engines */
    }

    nfaExpandState(nfa, q->state, q->streamState, q->offset,
                   queue_prev_byte(q, 0));

    assert(nfaAcceptsEod(nfa));
    nfaCheckFinalState(nfa, q->state, q->streamState, q->offset, q->cb,
                       q->som_cb, scratch);
}

static really_inline
void report_eod_matches(hs_stream_t *id, hs_scratch_t *scratch,
                        match_event_handler onEvent, void *context) {
    DEBUG_PRINTF("--- report eod matches at offset %llu\n", id->offset);
    assert(onEvent);

    const struct RoseEngine *rose = id->rose;
    char *state = getMultiState(id);

    if (getBroken(state)) {
        DEBUG_PRINTF("stream is broken, just freeing storage\n");
        return;
    }

    populateCoreInfo(scratch, rose, state, onEvent, context, NULL, 0,
                     getHistory(state, rose, id->offset),
                     getHistoryAmount(rose, id->offset), id->offset, 0);

    if (rose->somLocationCount) {
        loadSomFromStream(scratch, id->offset);
    }

    if (!id->offset) {
        if (rose->boundary.reportZeroEodOffset) {
            processReportList(rose, rose->boundary.reportZeroEodOffset, 0,
                              scratch);
        }
    } else {
        if (rose->boundary.reportEodOffset) {
            processReportList(rose, rose->boundary.reportEodOffset,
                              id->offset, scratch);
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
            scratch->core_info.broken = BROKEN_FROM_USER;
            DEBUG_PRINTF("broken = %hhd\n", scratch->core_info.broken);
        }
    }
}

HS_PUBLIC_API
hs_error_t hs_copy_stream(hs_stream_t **to_id, const hs_stream_t *from_id) {
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
hs_error_t hs_reset_and_copy_stream(hs_stream_t *to_id,
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
        report_eod_matches(to_id, scratch, onEvent, context);
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

    assert(!getBroken(getMultiState(stream_state)));

    DEBUG_PRINTF("::: streaming rose ::: offset = %llu len = %zu\n",
                 stream_state->offset, scratch->core_info.len);

    const struct RoseEngine *rose = stream_state->rose;
    assert(rose);
    roseStreamExec(rose, scratch, selectAdaptor(rose), selectSomAdaptor(rose),
                   scratch);

    if (!told_to_stop_matching(scratch) &&
        isAllExhausted(rose, scratch->core_info.exhaustionVector)) {
        DEBUG_PRINTF("stream exhausted\n");
        scratch->core_info.broken = BROKEN_EXHAUSTED;
    }
}

static really_inline
void pureLiteralStreamExec(struct hs_stream *stream_state,
                           struct hs_scratch *scratch) {
    assert(stream_state);
    assert(scratch);

    char *state = getMultiState(stream_state);
    assert(!getBroken(state));

    const struct RoseEngine *rose = stream_state->rose;
    const struct HWLM *ftable = getFLiteralMatcher(rose);

    size_t len2 = scratch->core_info.len;

    u8 *hwlm_stream_state;
    if (rose->floatingStreamState) {
        hwlm_stream_state = getFloatingMatcherState(rose, state);
    } else {
        hwlm_stream_state = NULL;
    }

    DEBUG_PRINTF("::: streaming rose ::: offset = %llu len = %zu\n",
                 stream_state->offset, scratch->core_info.len);

    // Pure literal cases don't have floatingMinDistance set, so we always
    // start the match region at zero.
    const size_t start = 0;

    hwlmExecStreaming(ftable, scratch, len2, start, selectHwlmAdaptor(rose),
                      scratch, rose->initialGroups, hwlm_stream_state);

    if (!told_to_stop_matching(scratch) &&
        isAllExhausted(rose, scratch->core_info.exhaustionVector)) {
        DEBUG_PRINTF("stream exhausted\n");
        scratch->core_info.broken = BROKEN_EXHAUSTED;
    }
}

static never_inline
void soleOutfixStreamExec(struct hs_stream *stream_state,
                          struct hs_scratch *scratch) {
    assert(stream_state);
    assert(scratch);

    const struct RoseEngine *t = stream_state->rose;
    assert(t->outfixEndQueue == 1);
    assert(!t->amatcherOffset);
    assert(!t->ematcherOffset);
    assert(!t->fmatcherOffset);

    const struct NFA *nfa = getNfaByQueue(t, 0);

    struct mq *q = scratch->queues;
    initQueue(q, 0, t, scratch);
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
        scratch->core_info.broken = BROKEN_EXHAUSTED;
    }
}

static inline
hs_error_t hs_scan_stream_internal(hs_stream_t *id, const char *data,
                                   unsigned length, UNUSED unsigned flags,
                                   hs_scratch_t *scratch,
                                   match_event_handler onEvent, void *context) {
    if (unlikely(!id || !scratch || !data || !validScratch(id->rose, scratch))) {
        return HS_INVALID;
    }

    const struct RoseEngine *rose = id->rose;
    char *state = getMultiState(id);

    u8 broken = getBroken(state);
    if (broken) {
        DEBUG_PRINTF("stream is broken, halting scan\n");
        if (broken == BROKEN_FROM_USER) {
            return HS_SCAN_TERMINATED;
        } else {
            assert(broken == BROKEN_EXHAUSTED);
            return HS_SUCCESS;
        }
    }

    // We avoid doing any work if the user has given us zero bytes of data to
    // scan. Arguably we should define some semantics for how we treat vacuous
    // cases here.
    if (unlikely(length == 0)) {
        DEBUG_PRINTF("zero length block\n");
        assert(getBroken(state) != BROKEN_FROM_USER);
        return HS_SUCCESS;
    }

    u32 historyAmount = getHistoryAmount(rose, id->offset);
    populateCoreInfo(scratch, rose, state, onEvent, context, data, length,
                     getHistory(state, rose, id->offset), historyAmount,
                     id->offset, flags);
    assert(scratch->core_info.hlen <= id->offset
           && scratch->core_info.hlen <= rose->historyRequired);

    prefetch_data(data, length);

    if (rose->somLocationCount) {
        loadSomFromStream(scratch, id->offset);
    }

    if (!id->offset && rose->boundary.reportZeroOffset) {
        DEBUG_PRINTF("zero reports\n");
        processReportList(rose, rose->boundary.reportZeroOffset, 0, scratch);
        broken = getBroken(state);
        if (unlikely(broken)) {
            DEBUG_PRINTF("stream is broken, halting scan\n");
            if (broken == BROKEN_FROM_USER) {
                return HS_SCAN_TERMINATED;
            } else {
                assert(broken == BROKEN_EXHAUSTED);
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
            setBroken(state, BROKEN_FROM_USER);
            scratch->core_info.broken = BROKEN_FROM_USER;
        }
    }

    if (likely(!can_stop_matching(scratch))) {
        maintainHistoryBuffer(id->rose, getMultiState(id), data, length);
        id->offset += length; /* maintain offset */

        if (rose->somLocationCount) {
            storeSomToStream(scratch, id->offset);
        }
    } else if (told_to_stop_matching(scratch)) {
        return HS_SCAN_TERMINATED;
    } else { /* exhausted */
        setBroken(state, BROKEN_EXHAUSTED);
    }

    return HS_SUCCESS;
}

HS_PUBLIC_API
hs_error_t hs_scan_stream(hs_stream_t *id, const char *data, unsigned length,
                          unsigned flags, hs_scratch_t *scratch,
                          match_event_handler onEvent, void *context) {
    return hs_scan_stream_internal(id, data, length, flags, scratch,
                                       onEvent, context);
}

HS_PUBLIC_API
hs_error_t hs_close_stream(hs_stream_t *id, hs_scratch_t *scratch,
                           match_event_handler onEvent, void *context) {
    if (!id) {
        return HS_INVALID;
    }

    if (onEvent) {
        if (!scratch || !validScratch(id->rose, scratch)) {
            return HS_INVALID;
        }
        report_eod_matches(id, scratch, onEvent, context);
    }

    hs_stream_free(id);

    return HS_SUCCESS;
}

HS_PUBLIC_API
hs_error_t hs_reset_stream(hs_stream_t *id, UNUSED unsigned int flags,
                           hs_scratch_t *scratch, match_event_handler onEvent,
                           void *context) {
    if (!id) {
        return HS_INVALID;
    }

    if (onEvent) {
        if (!scratch || !validScratch(id->rose, scratch)) {
            return HS_INVALID;
        }
        report_eod_matches(id, scratch, onEvent, context);
    }

    init_stream(id, id->rose);

    return HS_SUCCESS;
}

HS_PUBLIC_API
hs_error_t hs_stream_size(const hs_database_t *db, size_t *stream_size) {
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
hs_error_t hs_scan_vector(const hs_database_t *db, const char * const * data,
                          const unsigned int *length, unsigned int count,
                          UNUSED unsigned int flags, hs_scratch_t *scratch,
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

    hs_stream_t *id = (hs_stream_t *)(scratch->bstate);

    init_stream(id, rose); /* open stream */

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
            return ret;
        }
    }

    /* close stream */
    if (onEvent) {
        report_eod_matches(id, scratch, onEvent, context);

        if (told_to_stop_matching(scratch)) {
            return HS_SCAN_TERMINATED;
        }
    }

    return HS_SUCCESS;
}
