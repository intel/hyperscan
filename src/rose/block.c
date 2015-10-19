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
#include "init.h"
#include "match.h"
#include "nfa/nfa_api.h"
#include "nfa/nfa_internal.h"
#include "nfa/nfa_rev_api.h"
#include "nfa/mcclellan.h"
#include "util/fatbit.h"
#include "rose_sidecar_runtime.h"
#include "rose.h"
#include "rose_common.h"

static rose_inline
void runAnchoredTableBlock(const struct RoseEngine *t, const void *atable,
                           struct hs_scratch *scratch) {
    const u8 *buffer = scratch->core_info.buf;
    size_t length = scratch->core_info.len;
    size_t alen = MIN(length, t->anchoredDistance);
    const struct anchored_matcher_info *curr = atable;

    DEBUG_PRINTF("BEGIN ANCHORED (over %zu/%zu)\n", alen, length);

    do {
        const struct NFA *nfa
            = (const struct NFA *)((const char *)curr + sizeof(*curr));

        assert(t->anchoredDistance > curr->anchoredMinDistance);
        if (length >= curr->anchoredMinDistance) {
            size_t local_alen = alen - curr->anchoredMinDistance;
            const u8 *local_buffer = buffer + curr->anchoredMinDistance;

            DEBUG_PRINTF("--anchored nfa (+%u)\n", curr->anchoredMinDistance);
            assert(isMcClellanType(nfa->type));
            if (nfa->type == MCCLELLAN_NFA_8) {
                nfaExecMcClellan8_B(nfa, curr->anchoredMinDistance,
                                    local_buffer, local_alen,
                                    roseAnchoredCallback, &scratch->tctxt);
            } else {
                nfaExecMcClellan16_B(nfa, curr->anchoredMinDistance,
                                     local_buffer, local_alen,
                                     roseAnchoredCallback, &scratch->tctxt);
            }
        }

        if (!curr->next_offset) {
            break;
        }

        curr = (const void *)((const char *)curr + curr->next_offset);
    } while (1);
}

static really_inline
void init_sidecar(const struct RoseEngine *t, struct hs_scratch *scratch) {
    if (!t->smatcherOffset) {
        return;
    }

    DEBUG_PRINTF("welcome to the sidecar\n");
    assert(t->initSideEnableOffset);
    // We have to enable some sidecar literals
    const char *template = (const char *)t + t->initSideEnableOffset;

    memcpy(&scratch->side_enabled, template, t->stateOffsets.sidecar_size);
}

static really_inline
void init_state_for_block(const struct RoseEngine *t, u8 *state) {
    assert(t);
    assert(state);

    DEBUG_PRINTF("init for Rose %p with %u roles (%u with state indices)\n",
                 t, t->roleCount, t->rolesWithStateCount);

    // Rose is guaranteed 8-aligned state
    assert(ISALIGNED_N(state, 8));

    init_state(t, state);
}

static really_inline
void init_outfixes_for_block(const struct RoseEngine *t,
                             struct hs_scratch *scratch, u8 *state,
                             char is_small_block) {
    /* active leaf array has been cleared by the init scatter */

    if (t->initMpvNfa != MO_INVALID_IDX) {
        assert(t->initMpvNfa == 0);
        const struct NFA *nfa = getNfaByQueue(t, 0);
        DEBUG_PRINTF("testing minwidth %u > len %zu\n", nfa->minWidth,
                      scratch->core_info.len);
        size_t len = nfaRevAccelCheck(nfa, scratch->core_info.buf,
                                      scratch->core_info.len);
        if (len) {
            struct RoseContext *tctxt = &scratch->tctxt;
            u8 *activeArray = getActiveLeafArray(t, state);
            const u32 activeArraySize = t->activeArrayCount;
            const u32 qCount = t->queueCount;

            mmbit_set(activeArray, activeArraySize, 0);
            fatbit_set(scratch->aqa, qCount, 0);

            struct mq *q = scratch->queues;
            initQueue(q, 0, t, tctxt);
            q->length = len; /* adjust for rev_accel */
            nfaQueueInitState(nfa, q);
            pushQueueAt(q, 0, MQE_START, 0);
            pushQueueAt(q, 1, MQE_TOP, 0);
        }
    }

    if (is_small_block && !t->hasOutfixesInSmallBlock) {
        DEBUG_PRINTF("all outfixes in small block table\n");
        return;
    }

    if (t->outfixBeginQueue != t->outfixEndQueue) {
        blockInitSufPQ(t, state, scratch, is_small_block);
    }
}

static really_inline
void init_for_block(const struct RoseEngine *t, struct hs_scratch *scratch,
                    RoseCallback callback, RoseCallbackSom som_callback,
                    void *ctxt, u8 *state, char is_small_block) {
    init_state_for_block(t, state);

    struct RoseContext *tctxt = &scratch->tctxt;

    tctxt->t = t;
    tctxt->depth = 1;
    tctxt->groups = t->initialGroups;
    tctxt->lit_offset_adjust = 1; // index after last byte
    tctxt->delayLastEndOffset = 0;
    tctxt->lastEndOffset = 0;
    tctxt->filledDelayedSlots = 0;
    tctxt->state = state;
    tctxt->cb = callback;
    tctxt->cb_som = som_callback;
    tctxt->userCtx = ctxt;
    tctxt->lastMatchOffset = 0;
    tctxt->minMatchOffset = 0;
    tctxt->minNonMpvMatchOffset = 0;
    tctxt->next_mpv_offset = 0;
    tctxt->curr_anchored_loc = MMB_INVALID;
    tctxt->curr_row_offset = 0;
    tctxt->side_curr = 0;

    scratch->am_log_sum = 0; /* clear the anchored logs */
    scratch->al_log_sum = 0;

    fatbit_clear(scratch->aqa);

    init_sidecar(t, scratch); /* Init the sidecar enabled state */

    scratch->catchup_pq.qm_size = 0;

    init_outfixes_for_block(t, scratch, state, is_small_block);
}

void roseBlockExec_i(const struct RoseEngine *t, struct hs_scratch *scratch,
                     RoseCallback callback, RoseCallbackSom som_callback,
                     void *ctx) {
    assert(t);
    assert(scratch);
    assert(scratch->core_info.buf);
    assert(mmbit_sparse_iter_state_size(t->rolesWithStateCount)
           < MAX_SPARSE_ITER_STATES);

    const size_t length = scratch->core_info.len;

    // We have optimizations for small block scans: we run a single coalesced
    // HWLM scan instead of running the anchored and floating matchers. Some
    // outfixes are disabled as well (for SEP scans of single-byte literals,
    // which are also run in the HWLM scan).
    const char is_small_block =
        (length < ROSE_SMALL_BLOCK_LEN && t->sbmatcherOffset);

    u8 *state = (u8 *)scratch->core_info.state;

    init_for_block(t, scratch, callback, som_callback, ctx, state,
                   is_small_block);

    struct RoseContext *tctxt = &scratch->tctxt;

    if (is_small_block) {
        const void *sbtable = getSBLiteralMatcher(t);
        assert(sbtable);

        size_t sblen = MIN(length, t->smallBlockDistance);

        DEBUG_PRINTF("BEGIN SMALL BLOCK (over %zu/%zu)\n", sblen, length);
        DEBUG_PRINTF("-- %016llx\n", tctxt->groups);
        hwlmExec(sbtable, scratch->core_info.buf, sblen, 0, roseCallback,
                 tctxt, tctxt->groups);
        goto exit;
    }

    const void *atable = getALiteralMatcher(t);

    if (atable) {
        if (t->amatcherMaxBiAnchoredWidth != ROSE_BOUND_INF
            && length > t->amatcherMaxBiAnchoredWidth) {
            goto skip_atable;
        }

        if (length < t->amatcherMinWidth) {
            goto skip_atable;
        }


        runAnchoredTableBlock(t, atable, scratch);

        if (can_stop_matching(scratch)) {
            goto exit;
        }

        resetAnchoredLog(t, scratch);
    skip_atable:;
    }

    const struct HWLM *ftable = getFLiteralMatcher(t);
    if (ftable) {
        DEBUG_PRINTF("ftable fd=%u fmd %u\n", t->floatingDistance,
            t->floatingMinDistance);
        if (t->noFloatingRoots && tctxt->depth == 1) {
            DEBUG_PRINTF("skip FLOATING: no inflight matches\n");
            goto exit;
        }

        if (t->fmatcherMaxBiAnchoredWidth != ROSE_BOUND_INF
            && length > t->fmatcherMaxBiAnchoredWidth) {
            goto exit;
        }

        if (length < t->fmatcherMinWidth) {
            goto exit;
        }

        const u8 *buffer = scratch->core_info.buf;
        size_t flen = length;
        if (t->floatingDistance != ROSE_BOUND_INF) {
            flen = MIN(t->floatingDistance, length);
        }
        if (flen <= t->floatingMinDistance) {
            goto exit;
        }

        DEBUG_PRINTF("BEGIN FLOATING (over %zu/%zu)\n", flen, length);
        DEBUG_PRINTF("-- %016llx\n", tctxt->groups);
        hwlmExec(ftable, buffer, flen, t->floatingMinDistance,
                 roseCallback, tctxt, tctxt->groups);
    }

exit:;
    u8 dummy_delay_mask = 0;
    if (cleanUpDelayed(length, 0, tctxt, &dummy_delay_mask)
        == HWLM_TERMINATE_MATCHING) {
        return;
    }

    assert(!can_stop_matching(scratch));

    roseCatchUpTo(t, state, length, scratch, 0);
}
