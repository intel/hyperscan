/*
 * Copyright (c) 2016-2018, Intel Corporation
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
#include "util/logical.h"
#include "util/fatbit.h"

enum DedupeResult {
    DEDUPE_CONTINUE, //!< Continue with match, not a dupe.
    DEDUPE_SKIP, //!< Don't report this match, dupe or delayed due to SOM.
    DEDUPE_HALT //!< User instructed us to stop matching.
};

static really_inline
enum DedupeResult dedupeCatchup(const struct RoseEngine *rose,
                                struct hs_scratch *scratch, u64a offset,
                                u64a from_offset, u64a to_offset, u32 dkey,
                                s32 offset_adjust, char is_external_report,
                                char quash_som, const char do_som) {
    DEBUG_PRINTF("offset=%llu, match=[%llu,%llu], dkey=%u, do_som=%d\n", offset,
                 from_offset, to_offset, dkey, do_som);

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
        if (is_external_report || quash_som) {
            DEBUG_PRINTF("checking dkey %u at offset %llu\n", dkey, to_offset);
            assert(offset_adjust == 0 || offset_adjust == -1);
            if (fatbit_set(deduper->log[to_offset % 2], dkeyCount, dkey)) {
                /* we have already raised this report at this offset, squash
                 * dupe match. */
                DEBUG_PRINTF("dedupe\n");
                return DEDUPE_SKIP;
            }
        } else if (do_som) {
            /* SOM external event */
            DEBUG_PRINTF("checking dkey %u at offset %llu\n", dkey, to_offset);
            assert(offset_adjust == 0 || offset_adjust == -1);
            u64a *starts = deduper->som_start_log[to_offset % 2];
            if (fatbit_set(deduper->som_log[to_offset % 2], dkeyCount, dkey)) {
                starts[dkey] = MIN(starts[dkey], from_offset);
            } else {
                starts[dkey] = from_offset;
            }
            DEBUG_PRINTF("starts[%u]=%llu\n", dkey, starts[dkey]);

            if (offset_adjust) {
                deduper->som_log_dirty |= 1;
            } else {
                deduper->som_log_dirty |= 2;
            }

            return DEDUPE_SKIP;
        }
    }

    return DEDUPE_CONTINUE;
}

/** \brief Test whether the given key (\a ekey) is set in the exhaustion vector
 * \a evec. */
static really_inline
int isExhausted(const struct RoseEngine *rose, const char *evec, u32 ekey) {
    DEBUG_PRINTF("checking exhaustion %p %u\n", evec, ekey);
    assert(ekey != INVALID_EKEY);
    assert(ekey < rose->ekeyCount);
    return mmbit_isset((const u8 *)evec, rose->ekeyCount, ekey);
}

/** \brief Returns 1 if all exhaustion keys in the bitvector are on. */
static really_inline
int isAllExhausted(const struct RoseEngine *rose, const char *evec) {
    if (!rose->canExhaust) {
        return 0; /* pattern set is inexhaustible */
    }

    return mmbit_all((const u8 *)evec, rose->ekeyCount);
}

/** \brief Mark key \a ekey on in the exhaustion vector. */
static really_inline
void markAsMatched(const struct RoseEngine *rose, char *evec, u32 ekey) {
    DEBUG_PRINTF("marking as exhausted key %u\n", ekey);
    assert(ekey != INVALID_EKEY);
    assert(ekey < rose->ekeyCount);
    mmbit_set((u8 *)evec, rose->ekeyCount, ekey);
}

/** \brief Clear all keys in the exhaustion vector. */
static really_inline
void clearEvec(const struct RoseEngine *rose, char *evec) {
    DEBUG_PRINTF("clearing evec %p %u\n", evec, rose->ekeyCount);
    mmbit_clear((u8 *)evec, rose->ekeyCount);
}

/** \brief Test whether the given key (\a lkey) is set in the logical vector
 * \a lvec. */
static really_inline
char getLogicalVal(const struct RoseEngine *rose, const char *lvec, u32 lkey) {
    DEBUG_PRINTF("checking lkey matching %p %u\n", lvec, lkey);
    assert(lkey != INVALID_LKEY);
    assert(lkey < rose->lkeyCount + rose->lopCount);
    return mmbit_isset((const u8 *)lvec, rose->lkeyCount + rose->lopCount,
                       lkey);
}

/** \brief Mark key \a lkey on in the logical vector. */
static really_inline
void setLogicalVal(const struct RoseEngine *rose, char *lvec, u32 lkey,
                   char val) {
    DEBUG_PRINTF("marking as matched logical key %u\n", lkey);
    assert(lkey != INVALID_LKEY);
    assert(lkey < rose->lkeyCount + rose->lopCount);
    switch (val) {
    case 0:
        mmbit_unset((u8 *)lvec, rose->lkeyCount + rose->lopCount, lkey);
        break;
    default:
        mmbit_set((u8 *)lvec, rose->lkeyCount + rose->lopCount, lkey);
        break;
    }
}

/** \brief Mark key \a ckey on in the combination vector. */
static really_inline
void setCombinationActive(const struct RoseEngine *rose, char *cvec, u32 ckey) {
    DEBUG_PRINTF("marking as active combination key %u\n", ckey);
    assert(ckey != INVALID_CKEY);
    assert(ckey < rose->ckeyCount);
    mmbit_set((u8 *)cvec, rose->ckeyCount, ckey);
}

/** \brief Returns 1 if compliant to all logical combinations. */
static really_inline
char isLogicalCombination(const struct RoseEngine *rose, char *lvec,
                          u32 start, u32 result) {
    const struct LogicalOp *logicalTree = (const struct LogicalOp *)
        ((const char *)rose + rose->logicalTreeOffset);
    assert(start >= rose->lkeyCount);
    assert(start <= result);
    assert(result < rose->lkeyCount + rose->lopCount);
    for (u32 i = start; i <= result; i++) {
        const struct LogicalOp *op = logicalTree + (i - rose->lkeyCount);
        assert(i == op->id);
        assert(op->op <= LAST_LOGICAL_OP);
        switch ((enum LogicalOpType)op->op) {
        case LOGICAL_OP_NOT:
            setLogicalVal(rose, lvec, op->id,
                          !getLogicalVal(rose, lvec, op->ro));
            break;
        case LOGICAL_OP_AND:
            setLogicalVal(rose, lvec, op->id,
                          getLogicalVal(rose, lvec, op->lo) &
                          getLogicalVal(rose, lvec, op->ro)); // &&
            break;
        case LOGICAL_OP_OR:
            setLogicalVal(rose, lvec, op->id,
                          getLogicalVal(rose, lvec, op->lo) |
                          getLogicalVal(rose, lvec, op->ro)); // ||
            break;
        }
    }
    return getLogicalVal(rose, lvec, result);
}

/** \brief Clear all keys in the logical vector. */
static really_inline
void clearLvec(const struct RoseEngine *rose, char *lvec, char *cvec) {
    DEBUG_PRINTF("clearing lvec %p %u\n", lvec,
                 rose->lkeyCount + rose->lopCount);
    DEBUG_PRINTF("clearing cvec %p %u\n", cvec, rose->ckeyCount);
    mmbit_clear((u8 *)lvec, rose->lkeyCount + rose->lopCount);
    mmbit_clear((u8 *)cvec, rose->ckeyCount);
}

/** \brief Clear all keys in the combination vector. */
static really_inline
void clearCvec(const struct RoseEngine *rose, char *cvec) {
    DEBUG_PRINTF("clearing cvec %p %u\n", cvec, rose->ckeyCount);
    mmbit_clear((u8 *)cvec, rose->ckeyCount);
}

/**
 * \brief Deliver the given report to the user callback.
 *
 * Assumes all preconditions (bounds, exhaustion etc) have been checked and
 * that dedupe catchup has been done.
 */
static really_inline
int roseDeliverReport(u64a offset, ReportID onmatch, s32 offset_adjust,
                      struct hs_scratch *scratch, u32 ekey) {
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

    assert(!can_stop_matching(scratch));
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

/**
 * \brief Deliver the given SOM report to the user callback.
 *
 * Assumes all preconditions (bounds, exhaustion etc) have been checked and
 * that dedupe catchup has been done.
 */
static really_inline
int roseDeliverSomReport(u64a from_offset, u64a to_offset, ReportID onmatch,
                         s32 offset_adjust, struct hs_scratch *scratch,
                         u32 ekey) {
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

    assert(!can_stop_matching(scratch));
    assert(ekey == INVALID_EKEY ||
           !isExhausted(ci->rose, ci->exhaustionVector, ekey));

    to_offset += offset_adjust;
    assert(from_offset == HS_OFFSET_PAST_HORIZON || from_offset <= to_offset);

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

#endif // REPORT_H
