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

#ifndef ROSE_H
#define ROSE_H

#include "rose_types.h"
#include "rose_internal.h"
#include "runtime.h"
#include "scratch.h"
#include "ue2common.h"
#include "util/multibit.h"

// Initialise state space for engine use.
void roseInitState(const struct RoseEngine *t, char *state);

void roseBlockEodExec(const struct RoseEngine *t, u64a offset,
                      struct hs_scratch *scratch);
void roseBlockExec_i(const struct RoseEngine *t, struct hs_scratch *scratch);

static really_inline
int roseBlockHasEodWork(const struct RoseEngine *t,
                        struct hs_scratch *scratch) {
    if (t->ematcherOffset) {
        DEBUG_PRINTF("eod matcher to run\n");
        return 1;
    }

    if (t->eodProgramOffset) {
        DEBUG_PRINTF("has eod program\n");
        return 1;
    }

    void *state = scratch->core_info.state;
    if (mmbit_any(getActiveLeafArray(t, state), t->activeArrayCount)) {
        DEBUG_PRINTF("active outfix/suffix engines\n");
        return 1;
    }

    if (t->eodIterOffset) {
        u32 idx;
        const struct mmbit_sparse_iter *it = getByOffset(t, t->eodIterOffset);
        struct mmbit_sparse_state si_state[MAX_SPARSE_ITER_STATES];
        if (mmbit_sparse_iter_begin(getRoleState(state), t->rolesWithStateCount,
                                    &idx, it, si_state) != MMB_INVALID) {
            DEBUG_PRINTF("eod iter has states on\n");
            return 1;
        }
    }

    return 0;
}

/* assumes core_info in scratch has been init to point to data */
static really_inline
void roseBlockExec(const struct RoseEngine *t, struct hs_scratch *scratch) {
    assert(t);
    assert(scratch);
    assert(scratch->core_info.buf);

    // We should not have been called if we've already been told to terminate
    // matching.
    assert(!told_to_stop_matching(scratch));

    // If this block is shorter than our minimum width, then no pattern in this
    // RoseEngine could match.
    /* minWidth checks should have already been performed by the caller */
    const size_t length = scratch->core_info.len;
    assert(length >= t->minWidth);

    // Similarly, we may have a maximum width (for engines constructed entirely
    // of bi-anchored patterns).
    /* This check is now handled by the interpreter */
    assert(t->maxBiAnchoredWidth == ROSE_BOUND_INF
           || length <= t->maxBiAnchoredWidth);

    roseBlockExec_i(t, scratch);

    if (!t->requiresEodCheck) {
        return;
    }

    if (can_stop_matching(scratch)) {
        DEBUG_PRINTF("bailing, already halted\n");
        return;
    }

    if (!roseBlockHasEodWork(t, scratch)) {
        DEBUG_PRINTF("no eod work\n");
        return;
    }

    roseBlockEodExec(t, length, scratch);
}

/* assumes core_info in scratch has been init to point to data */
void roseStreamExec(const struct RoseEngine *t, struct hs_scratch *scratch);

void roseEodExec(const struct RoseEngine *t, u64a offset,
                 struct hs_scratch *scratch);

hwlmcb_rv_t rosePureLiteralCallback(size_t start, size_t end, u32 id,
                                    void *context);

int roseReportAdaptor(u64a offset, ReportID id, void *context);
int roseReportSomAdaptor(u64a som, u64a offset, ReportID id, void *context);

int roseRunBoundaryProgram(const struct RoseEngine *rose, u32 program,
                           u64a stream_offset, struct hs_scratch *scratch);

#endif // ROSE_H
