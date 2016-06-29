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
 * \brief SOM runtime code.
 *
 *
 * Runtime code for SOM handling called by the Rose callback adaptors.
 *
 * Note:
 * Races between escapes making a som loc writeable and attempts to write to it
 * at the same to_offset are always resolved as if the escape arrived first
 * and then the request to write to that location.
 */

#include "hs_internal.h"
#include "som_operation.h"
#include "som_runtime.h"
#include "scratch.h"
#include "ue2common.h"
#include "rose/rose_internal.h"
#include "nfa/nfa_api.h"
#include "nfa/nfa_internal.h"
#include "util/fatbit.h"
#include "util/multibit.h"

static really_inline
void setSomLoc(struct fatbit *som_set_now, u64a *som_store, u32 som_store_count,
               const struct som_operation *ri, u64a to_offset) {
    /* validity handled by callers */
    assert(to_offset >= ri->aux.somDistance);
    u64a start_offset = to_offset - ri->aux.somDistance;
    u32 som_loc = ri->onmatch;

    /* resolve any races for matches at this point in favour of the earliest som
     */
    if (!fatbit_set(som_set_now, som_store_count, som_loc)) {
        som_store[som_loc] = start_offset;
    } else {
        LIMIT_TO_AT_MOST(&som_store[som_loc], start_offset);
    }

    DEBUG_PRINTF("som_store[%u] set to %llu\n", som_loc, som_store[som_loc]);
}

static really_inline
char ok_and_mark_if_write(u8 *som_store_valid, struct fatbit *som_set_now,
                          u8 *som_store_writable, u32 som_store_count,
                          u32 loc) {
    return !mmbit_set(som_store_valid, som_store_count, loc) /* unwritten */
        || fatbit_isset(som_set_now, som_store_count, loc) /* write here, need
                                                            * to resolve race */
        || mmbit_isset(som_store_writable, som_store_count, loc); /* writable */
}

static really_inline
char ok_and_mark_if_unset(u8 *som_store_valid, struct fatbit *som_set_now,
                          u32 som_store_count, u32 loc) {
    return !mmbit_set(som_store_valid, som_store_count, loc) /* unwritten */
        || fatbit_isset(som_set_now, som_store_count, loc); /* write here, need
                                                            * to resolve race */
}

static
int somRevCallback(UNUSED u64a start, u64a end, ReportID id, void *ctx) {
    DEBUG_PRINTF("offset=%llu, id=%u\n", end, id);

    // We use the id to store the offset adjustment (for assertions like a
    // leading \b or multiline mode).
    assert(id <= 1);
    u64a *from_offset = ctx;
    LIMIT_TO_AT_MOST(from_offset, end + id);
    return 1; // continue matching.
}

static really_inline
const struct NFA *getSomRevNFA(const struct RoseEngine *t, u32 i) {
    assert(t->somRevOffsetOffset);
    const u32 *rev_offsets
        = (const u32 *)((const u8 *)t + t->somRevOffsetOffset);
    u32 nfa_offset = rev_offsets[i];
    assert(nfa_offset && nfa_offset < t->size);
    const struct NFA *n = (const struct NFA *)(((const u8 *)t + nfa_offset));
    assert(ISALIGNED(n));

    return n;
}

static
void runRevNfa(struct hs_scratch *scratch, const struct som_operation *ri,
               const u64a to_offset, u64a *from_offset) {
    struct core_info *ci = &scratch->core_info;

    DEBUG_PRINTF("buf has %zu bytes total, history has %zu\n",
                 ci->len, ci->hlen);

    u32 nfa_idx = ri->aux.revNfaIndex;
    DEBUG_PRINTF("run rev nfa %u from to_offset=%llu\n", nfa_idx, to_offset);
    const struct NFA *nfa = getSomRevNFA(ci->rose, nfa_idx);

    assert(nfa->maxWidth); // No inf width rev NFAs.

    size_t buf_bytes = to_offset - ci->buf_offset;
    size_t history_bytes = ci->hlen;

    DEBUG_PRINTF("nfa min/max widths [%u,%u], %zu in buffer, %zu in history\n",
                 nfa->minWidth, nfa->maxWidth,  buf_bytes, history_bytes);
    assert(nfa->minWidth <= buf_bytes + history_bytes);

    const u8 *buf = ci->buf;
    const u8 *hbuf = ci->hbuf;

    // Work out if we need to scan any history as well.
    if (history_bytes && buf_bytes < nfa->maxWidth) {
        assert(hbuf);
        size_t remainder = nfa->maxWidth - buf_bytes;
        if (remainder < history_bytes) {
            hbuf += history_bytes - remainder;
            history_bytes = remainder;
        }
    }

    DEBUG_PRINTF("scanning %zu from buffer and %zu from history\n", buf_bytes,
                 history_bytes);

    *from_offset = to_offset;

    nfaBlockExecReverse(nfa, to_offset, buf, buf_bytes, hbuf, history_bytes,
                        somRevCallback, from_offset);

    assert(*from_offset <= to_offset);
}

static really_inline
void setSomLocRevNfa(struct hs_scratch *scratch, struct fatbit *som_set_now,
                     u64a *som_store, u32 som_store_count,
                     const struct som_operation *ri, u64a to_offset) {
    /* validity handled by callers */
    u64a from_offset = 0;
    runRevNfa(scratch, ri, to_offset, &from_offset);

    u32 som_loc = ri->onmatch;

    /* resolve any races for matches at this point in favour of the earliest som
     */
    if (!fatbit_set(som_set_now, som_store_count, som_loc)) {
        som_store[som_loc] = from_offset;
    } else {
        LIMIT_TO_AT_MOST(&som_store[som_loc], from_offset);
    }

    DEBUG_PRINTF("som_store[%u] set to %llu\n", som_loc, som_store[som_loc]);
}

void handleSomInternal(struct hs_scratch *scratch,
                       const struct som_operation *ri, const u64a to_offset) {
    assert(scratch);
    assert(ri);
    DEBUG_PRINTF("-->som action required at %llu\n", to_offset);

    // SOM handling at scan time operates on data held in scratch. In
    // streaming mode, this data is read from / written out to stream state at
    // stream write boundaries.

    struct core_info *ci = &scratch->core_info;
    const struct RoseEngine *rose = ci->rose;
    assert(rose->hasSom);

    const u32 som_store_count = rose->somLocationCount;
    u8 *som_store_valid = (u8 *)ci->state + rose->stateOffsets.somValid;
    u8 *som_store_writable = (u8 *)ci->state + rose->stateOffsets.somWritable;
    struct fatbit *som_set_now = scratch->som_set_now;
    struct fatbit *som_attempted_set = scratch->som_attempted_set;
    u64a *som_store = scratch->som_store;
    u64a *som_failed_store = scratch->som_attempted_store;

    if (to_offset != scratch->som_set_now_offset) {
        assert(scratch->som_set_now_offset == ~0ULL
               || to_offset > scratch->som_set_now_offset);
        DEBUG_PRINTF("setting som_set_now_offset=%llu\n", to_offset);
        fatbit_clear(som_set_now);
        fatbit_clear(som_attempted_set);
        scratch->som_set_now_offset = to_offset;
    }

    switch (ri->type) {
    case SOM_INTERNAL_LOC_SET:
        DEBUG_PRINTF("SOM_INTERNAL_LOC_SET\n");
        mmbit_set(som_store_valid, som_store_count, ri->onmatch);
        setSomLoc(som_set_now, som_store, som_store_count, ri, to_offset);
        return;
    case SOM_INTERNAL_LOC_SET_IF_UNSET:
        DEBUG_PRINTF("SOM_INTERNAL_LOC_SET_IF_UNSET\n");
        if (ok_and_mark_if_unset(som_store_valid, som_set_now, som_store_count,
                                 ri->onmatch)) {
            setSomLoc(som_set_now, som_store, som_store_count, ri, to_offset);
        }
        return;
    case SOM_INTERNAL_LOC_SET_IF_WRITABLE: {
        u32 slot = ri->onmatch;
        DEBUG_PRINTF("SOM_INTERNAL_LOC_SET_IF_WRITABLE\n");
        if (ok_and_mark_if_write(som_store_valid, som_set_now,
                                 som_store_writable, som_store_count, slot)) {
            setSomLoc(som_set_now, som_store, som_store_count, ri, to_offset);
            mmbit_unset(som_store_writable, som_store_count, slot);
        } else {
            /* not writable, stash as an attempted write in case we are
             * racing our escape. */
            DEBUG_PRINTF("not writable, stashing attempt\n");
            assert(to_offset >= ri->aux.somDistance);
            u64a start_offset = to_offset - ri->aux.somDistance;

            if (!fatbit_set(som_attempted_set, som_store_count, slot)) {
                som_failed_store[slot] = start_offset;
            } else {
                LIMIT_TO_AT_MOST(&som_failed_store[slot], start_offset);
            }
            DEBUG_PRINTF("som_failed_store[%u] = %llu\n", slot,
                         som_failed_store[slot]);
        }
        return;
    }
    case SOM_INTERNAL_LOC_SET_REV_NFA:
        DEBUG_PRINTF("SOM_INTERNAL_LOC_SET_REV_NFA\n");
        mmbit_set(som_store_valid, som_store_count, ri->onmatch);
        setSomLocRevNfa(scratch, som_set_now, som_store, som_store_count, ri,
                        to_offset);
        return;
    case SOM_INTERNAL_LOC_SET_REV_NFA_IF_UNSET:
        DEBUG_PRINTF("SOM_INTERNAL_LOC_SET_REV_NFA_IF_UNSET\n");
        if (ok_and_mark_if_unset(som_store_valid, som_set_now, som_store_count,
                                 ri->onmatch)) {
            setSomLocRevNfa(scratch, som_set_now, som_store, som_store_count,
                            ri, to_offset);
        }
        return;
    case SOM_INTERNAL_LOC_SET_REV_NFA_IF_WRITABLE: {
        u32 slot = ri->onmatch;
        DEBUG_PRINTF("SOM_INTERNAL_LOC_SET_IF_WRITABLE\n");
        if (ok_and_mark_if_write(som_store_valid, som_set_now,
                                 som_store_writable, som_store_count, slot)) {
            setSomLocRevNfa(scratch, som_set_now, som_store, som_store_count,
                            ri, to_offset);
            mmbit_unset(som_store_writable, som_store_count, slot);
        } else {
            /* not writable, stash as an attempted write in case we are
             * racing our escape. */
            DEBUG_PRINTF("not writable, stashing attempt\n");

            u64a from_offset = 0;
            runRevNfa(scratch, ri, to_offset, &from_offset);

            if (!fatbit_set(som_attempted_set, som_store_count, slot)) {
                som_failed_store[slot] = from_offset;
            } else {
                LIMIT_TO_AT_MOST(&som_failed_store[slot], from_offset);
            }
            DEBUG_PRINTF("som_failed_store[%u] = %llu\n", slot,
                         som_failed_store[slot]);
        }
        return;
    }
    case SOM_INTERNAL_LOC_COPY: {
        u32 slot_in = ri->aux.somDistance;
        u32 slot_out = ri->onmatch;
        DEBUG_PRINTF("SOM_INTERNAL_LOC_COPY S[%u] = S[%u]\n", slot_out,
                     slot_in);
        assert(mmbit_isset(som_store_valid, som_store_count, slot_in));
        mmbit_set(som_store_valid, som_store_count, slot_out);
        fatbit_set(som_set_now, som_store_count, slot_out);
        som_store[slot_out] = som_store[slot_in];

        return;
    }
    case SOM_INTERNAL_LOC_COPY_IF_WRITABLE: {
        u32 slot_in = ri->aux.somDistance;
        u32 slot_out = ri->onmatch;
        DEBUG_PRINTF("SOM_INTERNAL_LOC_COPY_IF_WRITABLE S[%u] = S[%u]\n",
                     slot_out, slot_in);
        assert(mmbit_isset(som_store_valid, som_store_count, slot_in));
        if (ok_and_mark_if_write(som_store_valid, som_set_now,
                                 som_store_writable, som_store_count,
                                 slot_out)) {
            DEBUG_PRINTF("copy, set som_store[%u]=%llu\n", slot_out,
                         som_store[slot_in]);
            som_store[slot_out] = som_store[slot_in];
            fatbit_set(som_set_now, som_store_count, slot_out);
            mmbit_unset(som_store_writable, som_store_count, slot_out);
        } else {
            /* not writable, stash as an attempted write in case we are
             * racing our escape */
            DEBUG_PRINTF("not writable, stashing attempt\n");
            fatbit_set(som_attempted_set, som_store_count, slot_out);
            som_failed_store[slot_out] = som_store[slot_in];
            DEBUG_PRINTF("som_failed_store[%u] = %llu\n", slot_out,
                         som_failed_store[slot_out]);
        }
        return;
    }
    case SOM_INTERNAL_LOC_MAKE_WRITABLE: {
        u32 slot = ri->onmatch;
        DEBUG_PRINTF("SOM_INTERNAL_LOC_MAKE_WRITABLE\n");
        /* if just written to the loc, ignore the racing escape */
        if (fatbit_isset(som_set_now, som_store_count, slot)) {
            DEBUG_PRINTF("just written\n");
            return;
        }
        if (fatbit_isset(som_attempted_set, som_store_count, slot)) {
            /* writes were waiting for an escape to arrive */
            DEBUG_PRINTF("setting som_store[%u] = %llu from "
                         "som_failed_store[%u]\n", slot, som_failed_store[slot],
                         slot);
            som_store[slot] = som_failed_store[slot];
            fatbit_set(som_set_now, som_store_count, slot);
            return;
        }
        mmbit_set(som_store_writable, som_store_count, slot);
        return;
    }
    default:
        DEBUG_PRINTF("unknown report type!\n");
        break;
    }

    // All valid som_operation types should be handled and returned above.
    assert(0);
    return;
}

// Returns the SOM offset.
u64a handleSomExternal(struct hs_scratch *scratch,
                       const struct som_operation *ri,
                       const u64a to_offset) {
    assert(scratch);
    assert(ri);

    // SOM handling at scan time operates on data held in scratch. In
    // streaming mode, this data is read from / written out to stream state at
    // stream write boundaries.

    struct core_info *ci = &scratch->core_info;
    const struct RoseEngine *rose = ci->rose;
    assert(rose->hasSom);

    switch (ri->type) {
    case SOM_EXTERNAL_CALLBACK_REL:
        DEBUG_PRINTF("SOM_EXTERNAL_CALLBACK_REL: som is %llu chars back\n",
                     ri->aux.somDistance);
        assert(to_offset >= ri->aux.somDistance);
        return to_offset - ri->aux.somDistance;
    case SOM_EXTERNAL_CALLBACK_ABS:
        DEBUG_PRINTF("SOM_EXTERNAL_CALLBACK_ABS: som is at %llu\n",
                     ri->aux.somDistance);
        assert(to_offset >= ri->aux.somDistance);
        return ri->aux.somDistance;
    case SOM_EXTERNAL_CALLBACK_STORED: {
        const u64a *som_store = scratch->som_store;
        u32 slot = ri->aux.somDistance;
        DEBUG_PRINTF("SOM_EXTERNAL_CALLBACK_STORED: <- som_store[%u]=%llu\n",
                     slot, som_store[slot]);

        UNUSED const u32 som_store_count = rose->somLocationCount;
        UNUSED const u8 *som_store_valid = (u8 *)ci->state
            + rose->stateOffsets.somValid;

        assert(mmbit_isset(som_store_valid, som_store_count, slot));
        return som_store[slot];
    }
    case SOM_EXTERNAL_CALLBACK_REV_NFA: {
        DEBUG_PRINTF("SOM_EXTERNAL_CALLBACK_REV_NFA\n");
        u64a from_offset = 0;
        runRevNfa(scratch, ri, to_offset, &from_offset);
        return from_offset;
    }
    default:
        DEBUG_PRINTF("unknown report type!\n");
        break;
    }

    // All valid som_operation types should be handled and returned above.
    assert(0);
    return 0;
}

void setSomFromSomAware(struct hs_scratch *scratch,
                        const struct som_operation *ri, u64a from_offset,
                        u64a to_offset) {
    assert(scratch);
    assert(ri);
    assert(to_offset);
    assert(ri->type == SOM_INTERNAL_LOC_SET_FROM
           || ri->type == SOM_INTERNAL_LOC_SET_FROM_IF_WRITABLE);

    struct core_info *ci = &scratch->core_info;
    const struct RoseEngine *rose = ci->rose;
    assert(rose->hasSom);

    const u32 som_store_count = rose->somLocationCount;
    u8 *som_store_valid = (u8 *)ci->state + rose->stateOffsets.somValid;
    u8 *som_store_writable = (u8 *)ci->state + rose->stateOffsets.somWritable;
    struct fatbit *som_set_now = scratch->som_set_now;
    struct fatbit *som_attempted_set = scratch->som_attempted_set;
    u64a *som_store = scratch->som_store;
    u64a *som_failed_store = scratch->som_attempted_store;

    if (to_offset != scratch->som_set_now_offset) {
        DEBUG_PRINTF("setting som_set_now_offset=%llu\n", to_offset);
        fatbit_clear(som_set_now);
        fatbit_clear(som_attempted_set);
        scratch->som_set_now_offset = to_offset;
    }

    if (ri->type == SOM_INTERNAL_LOC_SET_FROM) {
        DEBUG_PRINTF("SOM_INTERNAL_LOC_SET_FROM\n");
        mmbit_set(som_store_valid, som_store_count, ri->onmatch);
        setSomLoc(som_set_now, som_store, som_store_count, ri, from_offset);
    } else {
        DEBUG_PRINTF("SOM_INTERNAL_LOC_SET_FROM_IF_WRITABLE\n");
        if (ok_and_mark_if_write(som_store_valid, som_set_now,
                                 som_store_writable, som_store_count,
                                 ri->onmatch)) {
            setSomLoc(som_set_now, som_store, som_store_count, ri, from_offset);
            mmbit_unset(som_store_writable, som_store_count, ri->onmatch);
        } else {
            /* not writable, stash as an attempted write in case we are
             * racing our escape. */
            DEBUG_PRINTF("not writable, stashing attempt\n");
            assert(to_offset >= ri->aux.somDistance);
            u32 som_loc = ri->onmatch;

            if (!fatbit_set(som_attempted_set, som_store_count, ri->onmatch)) {
                som_failed_store[som_loc] = from_offset;
            } else {
                LIMIT_TO_AT_MOST(&som_failed_store[som_loc], from_offset);
            }
            DEBUG_PRINTF("som_failed_store[%u] = %llu\n", som_loc,
                         som_failed_store[som_loc]);
        }
    }
}

static really_inline
int clearSomLog(struct hs_scratch *scratch, u64a offset, struct fatbit *log,
                const u64a *starts) {
    DEBUG_PRINTF("at %llu\n", offset);
    struct core_info *ci = &scratch->core_info;
    const struct RoseEngine *rose = ci->rose;
    const u32 dkeyCount = rose->dkeyCount;
    const u32 *dkey_to_report = (const u32 *)
        ((const char *)rose + rose->invDkeyOffset);
    u32 flags = 0;
#ifndef RELEASE_BUILD
    if (scratch->deduper.current_report_offset != offset) {
        flags |= HS_MATCH_FLAG_ADJUSTED;
    }
#endif

    for (u32 it = fatbit_iterate(log, dkeyCount, MMB_INVALID);
             it != MMB_INVALID; it = fatbit_iterate(log, dkeyCount, it)) {
        u64a from_offset = starts[it];
        u32 onmatch = dkey_to_report[it];
        int halt = ci->userCallback(onmatch, from_offset, offset, flags,
                                    ci->userContext);
        if (halt) {
            ci->status |= STATUS_TERMINATED;
            return 1;
        }
    }
    fatbit_clear(log);
    return 0;
}

int flushStoredSomMatches_i(struct hs_scratch *scratch, u64a offset) {
    DEBUG_PRINTF("flush som matches\n");
    int halt = 0;

    assert(!told_to_stop_matching(scratch));

    if (scratch->deduper.current_report_offset == ~0ULL) {
        /* no matches recorded yet; just need to clear the logs */
        fatbit_clear(scratch->deduper.som_log[0]);
        fatbit_clear(scratch->deduper.som_log[1]);
        scratch->deduper.som_log_dirty = 0;
        return 0;
    }

    /* fire any reports from the logs and clear them */
    if (offset == scratch->deduper.current_report_offset + 1) {
        struct fatbit *done_log = scratch->deduper.som_log[offset % 2];
        u64a *done_starts = scratch->deduper.som_start_log[offset % 2];

        halt = clearSomLog(scratch, scratch->deduper.current_report_offset - 1,
                           done_log, done_starts);
        scratch->deduper.som_log_dirty >>= 1;
    } else {
        /* need to report both logs */
        u64a f_offset = scratch->deduper.current_report_offset - 1;
        u64a s_offset = scratch->deduper.current_report_offset;
        struct fatbit *first_log = scratch->deduper.som_log[f_offset % 2];
        u64a *first_starts = scratch->deduper.som_start_log[f_offset % 2];
        struct fatbit *second_log = scratch->deduper.som_log[s_offset % 2];
        u64a *second_starts = scratch->deduper.som_start_log[s_offset % 2];

        halt = clearSomLog(scratch, f_offset, first_log, first_starts) ||
               clearSomLog(scratch, s_offset, second_log, second_starts);
        scratch->deduper.som_log_dirty = 0;
    }

    return halt;
}
