/*
 * Copyright (c) 2016, Intel Corporation
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
 * \brief Runtime functions to do with reports, inlined into callers.
 */

#ifndef REPORT_H
#define REPORT_H

#include "hs_internal.h"
#include "hs_runtime.h"
#include "scratch.h"
#include "ue2common.h"
#include "nfa/callback.h"
#include "nfa/nfa_internal.h"
#include "rose/runtime.h"
#include "som/som_runtime.h"
#include "util/exhaust.h"
#include "util/fatbit.h"
#include "util/internal_report.h"

static really_inline
int satisfiesMinLength(u64a min_len, u64a from_offset,
                       u64a to_offset) {
    assert(min_len);

    if (from_offset == HS_OFFSET_PAST_HORIZON) {
        DEBUG_PRINTF("SOM beyond horizon\n");
        return 1;
    }

    DEBUG_PRINTF("match len=%llu, min len=%llu\n", to_offset - from_offset,
                 min_len);
    return to_offset - from_offset >= min_len;
}

enum DedupeResult {
    DEDUPE_CONTINUE, //!< Continue with match, not a dupe.
    DEDUPE_SKIP, //!< Don't report this match, dupe or delayed due to SOM.
    DEDUPE_HALT //!< User instructed us to stop matching.
};

static really_inline
enum DedupeResult dedupeCatchup(const struct RoseEngine *rose,
                                const struct internal_report *ir,
                                struct hs_scratch *scratch, u64a offset,
                                u64a from_offset, u64a to_offset,
                                const char do_som) {
    DEBUG_PRINTF("offset=%llu, match=[%llu,%llu], dkey=%u, do_som=%d\n", offset,
                 from_offset, to_offset, ir->dkey, do_som);
    DEBUG_PRINTF("report type=%u, quashSom=%d\n", ir->type, ir->quashSom);
    const u32 dkey = ir->dkey;

    // We should not have been called if there's no dedupe work to do.
    assert(do_som || dkey != MO_INVALID_IDX);

    struct match_deduper *deduper = &scratch->deduper;
    if (offset != deduper->current_report_offset) {
        assert(deduper->current_report_offset == ~0ULL ||
               deduper->current_report_offset < offset);
        if (offset == deduper->current_report_offset + 1) {
            fatbit_clear(deduper->log[offset % 2]);
        } else {
            fatbit_clear(deduper->log[0]);
            fatbit_clear(deduper->log[1]);
        }

        if (do_som && flushStoredSomMatches(scratch, offset)) {
            return DEDUPE_HALT;
        }
        deduper->current_report_offset = offset;
    }

    if (dkey != MO_INVALID_IDX) {
        const u32 dkeyCount = rose->dkeyCount;
        const s32 offset_adj = ir->offsetAdjust;
        if (ir->type == EXTERNAL_CALLBACK || ir->quashSom) {
            DEBUG_PRINTF("checking dkey %u at offset %llu\n", dkey, to_offset);
            assert(offset_adj == 0 || offset_adj == -1);
            if (fatbit_set(deduper->log[to_offset % 2], dkeyCount, dkey)) {
                /* we have already raised this report at this offset, squash
                 * dupe match. */
                DEBUG_PRINTF("dedupe\n");
                return DEDUPE_SKIP;
            }
        } else if (do_som) {
            /* SOM external event */
            DEBUG_PRINTF("checking dkey %u at offset %llu\n", dkey, to_offset);
            assert(offset_adj == 0 || offset_adj == -1);
            u64a *starts = deduper->som_start_log[to_offset % 2];
            if (fatbit_set(deduper->som_log[to_offset % 2], dkeyCount, dkey)) {
                starts[dkey] = MIN(starts[dkey], from_offset);
            } else {
                starts[dkey] = from_offset;
            }
            DEBUG_PRINTF("starts[%u]=%llu\n", dkey, starts[dkey]);

            if (offset_adj) {
                deduper->som_log_dirty |= 1;
            } else {
                deduper->som_log_dirty |= 2;
            }

            return DEDUPE_SKIP;
        }
    }

    return DEDUPE_CONTINUE;
}

static really_inline
enum DedupeResult dedupeCatchupSom(const struct RoseEngine *rose,
                                   const struct internal_report *ir,
                                   struct hs_scratch *scratch, u64a offset,
                                   u64a from_offset, u64a to_offset) {
    DEBUG_PRINTF("offset=%llu, match=[%llu,%llu], dkey=%u\n", offset,
                 from_offset, to_offset, ir->dkey);
    DEBUG_PRINTF("report type=%u, quashSom=%d\n", ir->type, ir->quashSom);

    struct match_deduper *deduper = &scratch->deduper;
    if (offset != deduper->current_report_offset) {
        assert(deduper->current_report_offset == ~0ULL ||
               deduper->current_report_offset < offset);
        if (offset == deduper->current_report_offset + 1) {
            fatbit_clear(deduper->log[offset % 2]);
        } else {
            fatbit_clear(deduper->log[0]);
            fatbit_clear(deduper->log[1]);
        }

        if (flushStoredSomMatches(scratch, offset)) {
            return DEDUPE_HALT;
        }
        deduper->current_report_offset = offset;
    }

    const u32 dkey = ir->dkey;
    if (dkey != MO_INVALID_IDX) {
        const u32 dkeyCount = rose->dkeyCount;
        const s32 offset_adj = ir->offsetAdjust;
        if (ir->quashSom) {
            DEBUG_PRINTF("checking dkey %u at offset %llu\n", dkey, to_offset);
            assert(offset_adj == 0 || offset_adj == -1);
            if (fatbit_set(deduper->log[to_offset % 2], dkeyCount, dkey)) {
                /* we have already raised this report at this offset, squash
                 * dupe match. */
                DEBUG_PRINTF("dedupe\n");
                return DEDUPE_SKIP;
            }
        } else {
            /* SOM external event */
            DEBUG_PRINTF("checking dkey %u at offset %llu\n", dkey, to_offset);
            assert(offset_adj == 0 || offset_adj == -1);
            u64a *starts = deduper->som_start_log[to_offset % 2];
            if (fatbit_set(deduper->som_log[to_offset % 2], dkeyCount, dkey)) {
                starts[dkey] = MIN(starts[dkey], from_offset);
            } else {
                starts[dkey] = from_offset;
            }
            DEBUG_PRINTF("starts[%u]=%llu\n", dkey, starts[dkey]);

            if (offset_adj) {
                deduper->som_log_dirty |= 1;
            } else {
                deduper->som_log_dirty |= 2;
            }

            return DEDUPE_SKIP;
        }
    }

    return DEDUPE_CONTINUE;
}

static really_inline
int roseAdaptor_i(u64a offset, ReportID id, struct hs_scratch *scratch,
                  char is_simple, char do_som) {
    assert(id != MO_INVALID_IDX); // Should never get an invalid ID.
    assert(scratch);
    assert(scratch->magic == SCRATCH_MAGIC);

    struct core_info *ci = &scratch->core_info;
    const struct RoseEngine *rose = ci->rose;
    DEBUG_PRINTF("internal report %u\n", id);
    const struct internal_report *ir = getInternalReport(rose, id);

    assert(isExternalReport(ir)); /* only external reports should reach here */

    s32 offset_adj = ir->offsetAdjust;
    u64a to_offset = offset;
    u64a from_offset = 0;

    u32 flags = 0;
#ifndef RELEASE_BUILD
    if (offset_adj) {
        // alert testing tools that we've got adjusted matches
        flags |= HS_MATCH_FLAG_ADJUSTED;
    }
#endif

    DEBUG_PRINTF("internal match at %llu: IID=%u type=%hhu RID=%u "
                 "offsetAdj=%d\n", offset, id, ir->type, ir->onmatch,
                 offset_adj);

    if (unlikely(can_stop_matching(scratch))) { /* ok - we are from rose */
        DEBUG_PRINTF("pre broken - halting\n");
        return MO_HALT_MATCHING;
    }

    if (!is_simple && ir->hasBounds) {
        assert(ir->minOffset || ir->minLength || ir->maxOffset < MAX_OFFSET);
        assert(ir->minOffset <= ir->maxOffset);
        if (offset < ir->minOffset || offset > ir->maxOffset) {
            DEBUG_PRINTF("match fell outside valid range %llu !: [%llu,%llu]\n",
                         offset, ir->minOffset, ir->maxOffset);
            return ROSE_CONTINUE_MATCHING_NO_EXHAUST;
        }
    }

    if (!is_simple && ir->ekey != INVALID_EKEY &&
        unlikely(isExhausted(ci->rose, ci->exhaustionVector, ir->ekey))) {
        DEBUG_PRINTF("ate exhausted match\n");
        return MO_CONTINUE_MATCHING;
    }

    if (ir->type == EXTERNAL_CALLBACK) {
        from_offset = 0;
    } else if (do_som) {
        from_offset = handleSomExternal(scratch, ir, to_offset);
    }

    to_offset += offset_adj;
    assert(from_offset == HS_OFFSET_PAST_HORIZON || from_offset <= to_offset);

    if (do_som && ir->minLength) {
        if (!satisfiesMinLength(ir->minLength, from_offset, to_offset)) {
            return ROSE_CONTINUE_MATCHING_NO_EXHAUST;
        }
        if (ir->quashSom) {
            from_offset = 0;
        }
    }

    DEBUG_PRINTF(">> reporting match @[%llu,%llu] for sig %u ctxt %p <<\n",
                 from_offset, to_offset, ir->onmatch, ci->userContext);

    int halt = 0;

    if (do_som || ir->dkey != MO_INVALID_IDX) {
        enum DedupeResult dedupe_rv = dedupeCatchup(rose, ir, scratch, offset,
                                                from_offset, to_offset, do_som);
        switch (dedupe_rv) {
        case DEDUPE_HALT:
            halt = 1;
            goto exit;
        case DEDUPE_SKIP:
            halt = 0;
            goto exit;
        case DEDUPE_CONTINUE:
            break;
        }
    }

    halt = ci->userCallback((unsigned int)ir->onmatch, from_offset, to_offset,
                            flags, ci->userContext);
exit:
    if (halt) {
        DEBUG_PRINTF("callback requested to terminate matches\n");
        ci->status |= STATUS_TERMINATED;
        return MO_HALT_MATCHING;
    }

    if (!is_simple && ir->ekey != INVALID_EKEY) {
        markAsMatched(ci->rose, ci->exhaustionVector, ir->ekey);
        return MO_CONTINUE_MATCHING;
    } else {
        return ROSE_CONTINUE_MATCHING_NO_EXHAUST;
    }
}

/**
 * \brief Deliver the given report to the user callback.
 *
 * Assumes all preconditions (bounds, exhaustion etc) have been checked and
 * that dedupe catchup has been done.
 */
static really_inline
int roseDeliverReport(u64a offset, UNUSED ReportID id, ReportID onmatch,
                      s32 offset_adjust, struct hs_scratch *scratch, u32 ekey) {
    assert(scratch);
    assert(scratch->magic == SCRATCH_MAGIC);

    struct core_info *ci = &scratch->core_info;

    u32 flags = 0;
#ifndef RELEASE_BUILD
    if (offset_adjust) {
        // alert testing tools that we've got adjusted matches
        flags |= HS_MATCH_FLAG_ADJUSTED;
    }
#endif

#ifndef NDEBUG
    // Assertions for development builds.
    UNUSED const struct internal_report *ir = getInternalReport(ci->rose, id);
    assert(isExternalReport(ir)); /* only external reports should reach here */

    assert(!can_stop_matching(scratch));
    assert(!ir->hasBounds ||
           (offset >= ir->minOffset && offset <= ir->maxOffset));
    assert(ir->type == EXTERNAL_CALLBACK);
    assert(!ir->minLength);
    assert(!ir->quashSom);
#endif

    assert(ekey == INVALID_EKEY ||
           !isExhausted(ci->rose, ci->exhaustionVector, ekey));

    u64a from_offset = 0;
    u64a to_offset = offset + offset_adjust;

    DEBUG_PRINTF(">> reporting match @[%llu,%llu] for sig %u ctxt %p <<\n",
                 from_offset, to_offset, onmatch, ci->userContext);

    int halt = ci->userCallback(onmatch, from_offset, to_offset, flags,
                                ci->userContext);
    if (halt) {
        DEBUG_PRINTF("callback requested to terminate matches\n");
        ci->status |= STATUS_TERMINATED;
        return MO_HALT_MATCHING;
    }

    if (ekey != INVALID_EKEY) {
        markAsMatched(ci->rose, ci->exhaustionVector, ekey);
        return MO_CONTINUE_MATCHING;
    } else {
        return ROSE_CONTINUE_MATCHING_NO_EXHAUST;
    }
}

static really_inline
int roseSomAdaptor_i(u64a from_offset, u64a to_offset, ReportID id,
                     struct hs_scratch *scratch, char is_simple) {
    assert(id != MO_INVALID_IDX); // Should never get an invalid ID.
    assert(scratch);
    assert(scratch->magic == SCRATCH_MAGIC);

    u32 flags = 0;

    struct core_info *ci = &scratch->core_info;
    const struct RoseEngine *rose = ci->rose;
    const struct internal_report *ir = getInternalReport(rose, id);

    /* internal events should be handled by rose directly */
    assert(ir->type == EXTERNAL_CALLBACK);

    DEBUG_PRINTF("internal match at %llu: IID=%u type=%hhu RID=%u "
                 "offsetAdj=%d\n", to_offset, id, ir->type, ir->onmatch,
                 ir->offsetAdjust);

    if (unlikely(can_stop_matching(scratch))) {
        DEBUG_PRINTF("pre broken - halting\n");
        return MO_HALT_MATCHING;
    }

    if (!is_simple && ir->hasBounds) {
        assert(ir->minOffset || ir->minLength || ir->maxOffset < MAX_OFFSET);
        if (to_offset < ir->minOffset || to_offset > ir->maxOffset) {
            DEBUG_PRINTF("match fell outside valid range %llu !: [%llu,%llu]\n",
                         to_offset, ir->minOffset, ir->maxOffset);
            return MO_CONTINUE_MATCHING;
        }
    }

    int halt = 0;

    if (!is_simple && ir->ekey != INVALID_EKEY &&
        unlikely(isExhausted(ci->rose, ci->exhaustionVector, ir->ekey))) {
        DEBUG_PRINTF("ate exhausted match\n");
        goto exit;
    }

    u64a offset = to_offset;

    to_offset += ir->offsetAdjust;
    assert(from_offset == HS_OFFSET_PAST_HORIZON || from_offset <= to_offset);

    if (!is_simple && ir->minLength) {
        if (!satisfiesMinLength(ir->minLength, from_offset, to_offset)) {
            return MO_CONTINUE_MATCHING;
        }
        if (ir->quashSom) {
            from_offset = 0;
        }
    }

    DEBUG_PRINTF(">> reporting match @[%llu,%llu] for sig %u ctxt %p <<\n",
                 from_offset, to_offset, ir->onmatch, ci->userContext);

#ifndef RELEASE_BUILD
    if (ir->offsetAdjust != 0) {
        // alert testing tools that we've got adjusted matches
        flags |= HS_MATCH_FLAG_ADJUSTED;
    }
#endif

    enum DedupeResult dedupe_rv =
        dedupeCatchupSom(rose, ir, scratch, offset, from_offset, to_offset);
    switch (dedupe_rv) {
    case DEDUPE_HALT:
        halt = 1;
        goto exit;
    case DEDUPE_SKIP:
        halt = 0;
        goto exit;
    case DEDUPE_CONTINUE:
        break;
    }

    halt = ci->userCallback((unsigned int)ir->onmatch, from_offset, to_offset,
                            flags, ci->userContext);

    if (!is_simple && ir->ekey != INVALID_EKEY) {
        markAsMatched(ci->rose, ci->exhaustionVector, ir->ekey);
    }

exit:
    if (halt) {
        DEBUG_PRINTF("callback requested to terminate matches\n");
        ci->status |= STATUS_TERMINATED;
        return MO_HALT_MATCHING;
    }

    return MO_CONTINUE_MATCHING;
}

/**
 * \brief Deliver the given SOM report to the user callback.
 *
 * Assumes all preconditions (bounds, exhaustion etc) have been checked and
 * that dedupe catchup has been done.
 */
static really_inline
int roseDeliverSomReport(u64a from_offset, u64a to_offset,
                         const struct internal_report *ir,
                         struct hs_scratch *scratch, char is_exhaustible) {
    assert(scratch);
    assert(scratch->magic == SCRATCH_MAGIC);
    assert(isExternalReport(ir)); /* only external reports should reach here */

    struct core_info *ci = &scratch->core_info;

    u32 flags = 0;
#ifndef RELEASE_BUILD
    if (ir->offsetAdjust != 0) {
        // alert testing tools that we've got adjusted matches
        flags |= HS_MATCH_FLAG_ADJUSTED;
    }
#endif

    assert(!can_stop_matching(scratch));
    assert(!ir->hasBounds ||
           (to_offset >= ir->minOffset && to_offset <= ir->maxOffset));
    assert(ir->ekey == INVALID_EKEY ||
           !isExhausted(ci->rose, ci->exhaustionVector, ir->ekey));

    to_offset += ir->offsetAdjust;
    assert(from_offset == HS_OFFSET_PAST_HORIZON || from_offset <= to_offset);

    assert(!ir->minLength ||
           satisfiesMinLength(ir->minLength, from_offset, to_offset));
    assert(!ir->quashSom || from_offset == 0);

    DEBUG_PRINTF(">> reporting match @[%llu,%llu] for sig %u ctxt %p <<\n",
                 from_offset, to_offset, ir->onmatch, ci->userContext);


    int halt = ci->userCallback((unsigned int)ir->onmatch, from_offset,
                                to_offset, flags, ci->userContext);

    if (halt) {
        DEBUG_PRINTF("callback requested to terminate matches\n");
        ci->status |= STATUS_TERMINATED;
        return MO_HALT_MATCHING;
    }

    if (is_exhaustible) {
        assert(ir->ekey != INVALID_EKEY);
        markAsMatched(ci->rose, ci->exhaustionVector, ir->ekey);
        return MO_CONTINUE_MATCHING;
    } else {
        return ROSE_CONTINUE_MATCHING_NO_EXHAUST;
    }
}

#endif // REPORT_H
