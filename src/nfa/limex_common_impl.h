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

#include "repeat.h"
#include "util/join.h"

/* impl of limex functions which depend only on state size */

#if !defined(SIZE) || !defined(STATE_T) || !defined(LOAD_FROM_ENG) \
    || !defined(INLINE_ATTR)
#  error Must define SIZE, STATE_T, LOAD_FROM_ENG and INLINE_ATTR in includer.
#endif

#define IMPL_NFA_T          JOIN(struct LimExNFA, SIZE)

#define TESTEOD_FN          JOIN(moNfaTestEod, SIZE)
#define LIMEX_INACCEPT_FN   JOIN(limexInAccept, SIZE)
#define LIMEX_INANYACCEPT_FN   JOIN(limexInAnyAccept, SIZE)
#define EXPIRE_ESTATE_FN    JOIN(limexExpireExtendedState, SIZE)
#define REPORTCURRENT_FN    JOIN(moNfaReportCurrent, SIZE)
#define INITIAL_FN          JOIN(moNfaInitial, SIZE)
#define TOP_FN              JOIN(moNfaTop, SIZE)
#define TOPN_FN             JOIN(moNfaTopN, SIZE)
#define PROCESS_ACCEPTS_IMPL_FN  JOIN(moProcessAcceptsImpl, SIZE)
#define PROCESS_ACCEPTS_FN  JOIN(moProcessAccepts, SIZE)
#define PROCESS_ACCEPTS_NOSQUASH_FN  JOIN(moProcessAcceptsNoSquash, SIZE)
#define CONTEXT_T           JOIN(NFAContext, SIZE)
#define ONES_STATE          JOIN(ones_, STATE_T)
#define AND_STATE           JOIN(and_, STATE_T)
#define OR_STATE            JOIN(or_, STATE_T)
#define ANDNOT_STATE        JOIN(andnot_, STATE_T)
#define CLEARBIT_STATE      JOIN(clearbit_, STATE_T)
#define TESTBIT_STATE       JOIN(testbit_, STATE_T)
#define ISNONZERO_STATE     JOIN(isNonZero_, STATE_T)
#define ISZERO_STATE        JOIN(isZero_, STATE_T)
#define SQUASH_UNTUG_BR_FN  JOIN(lazyTug, SIZE)
#define GET_NFA_REPEAT_INFO_FN JOIN(getNfaRepeatInfo, SIZE)

#if defined(ARCH_64_BIT) && (SIZE >= 64)
#define CHUNK_T u64a
#define FIND_AND_CLEAR_FN findAndClearLSB_64
#define POPCOUNT_FN popcount64
#define RANK_IN_MASK_FN rank_in_mask64
#else
#define CHUNK_T u32
#define FIND_AND_CLEAR_FN findAndClearLSB_32
#define POPCOUNT_FN popcount32
#define RANK_IN_MASK_FN rank_in_mask32
#endif

#define NUM_STATE_CHUNKS (sizeof(STATE_T) / sizeof(CHUNK_T))

static really_inline
void SQUASH_UNTUG_BR_FN(const IMPL_NFA_T *limex,
                        const union RepeatControl *repeat_ctrl,
                        const char *repeat_state, u64a offset,
                        STATE_T *accstate) {
    // switch off cyclic tug-accepts which aren't tuggable right now.

    /* TODO: might be nice to work which br to examine based on accstate rather
     * than iterating overall br */

    if (!limex->repeatCount) {
        return;
    }

    assert(repeat_ctrl);
    assert(repeat_state);

    for (u32 i = 0; i < limex->repeatCount; i++) {
        const struct NFARepeatInfo *info = GET_NFA_REPEAT_INFO_FN(limex, i);

        u32 cyclicState = info->cyclicState;
        if (!TESTBIT_STATE(*accstate, cyclicState)) {
            continue;
        }

        DEBUG_PRINTF("repeat %u (cyclic state %u) is active\n", i, cyclicState);
        DEBUG_PRINTF("checking if offset %llu would match\n", offset);

        const union RepeatControl *ctrl = repeat_ctrl + i;
        const char *state = repeat_state + info->stateOffset;
        const struct RepeatInfo *repeat = getRepeatInfo(info);
        if (repeatHasMatch(repeat, ctrl, state, offset) != REPEAT_MATCH) {
            DEBUG_PRINTF("not ready to accept yet\n");
            CLEARBIT_STATE(accstate, cyclicState);
        }
    }
}

static really_inline
char PROCESS_ACCEPTS_IMPL_FN(const IMPL_NFA_T *limex, const STATE_T *s,
                             STATE_T *squash, const STATE_T *acceptMask,
                             const struct NFAAccept *acceptTable, u64a offset,
                             NfaCallback callback, void *context) {
    assert(s);
    assert(limex);
    assert(callback);

    const STATE_T accept_mask = *acceptMask;
    STATE_T accepts = AND_STATE(*s, accept_mask);

    // Caller must ensure that we have at least one accept state on.
    assert(ISNONZERO_STATE(accepts));

    CHUNK_T chunks[NUM_STATE_CHUNKS];
    memcpy(chunks, &accepts, sizeof(accepts));

    CHUNK_T mask_chunks[NUM_STATE_CHUNKS];
    memcpy(mask_chunks, &accept_mask, sizeof(accept_mask));

    u32 base_index = 0; // Cumulative sum of mask popcount up to current chunk.
    for (u32 i = 0; i < NUM_STATE_CHUNKS; i++) {
        CHUNK_T chunk = chunks[i];
        while (chunk != 0) {
            u32 bit = FIND_AND_CLEAR_FN(&chunk);
            u32 local_idx = RANK_IN_MASK_FN(mask_chunks[i], bit);
            u32 idx = local_idx + base_index;
            const struct NFAAccept *a = &acceptTable[idx];
            DEBUG_PRINTF("state %u: firing report list=%u, offset=%llu\n",
                         bit + i * (u32)sizeof(chunk) * 8, a->reports, offset);
            int rv = limexRunAccept((const char *)limex, a, callback, context,
                                    offset);
            if (unlikely(rv == MO_HALT_MATCHING)) {
                return 1;
            }
            if (squash != NULL && a->squash != MO_INVALID_IDX) {
                DEBUG_PRINTF("applying squash mask at offset %u\n", a->squash);
                const ENG_STATE_T *sq =
                    (const ENG_STATE_T *)((const char *)limex + a->squash);
                *squash = AND_STATE(*squash, LOAD_FROM_ENG(sq));
            }
        }
        base_index += POPCOUNT_FN(mask_chunks[i]);
    }

    return 0;
}

static never_inline
char PROCESS_ACCEPTS_FN(const IMPL_NFA_T *limex, STATE_T *s,
                        const STATE_T *acceptMask,
                        const struct NFAAccept *acceptTable, u64a offset,
                        NfaCallback callback, void *context) {
    // We have squash masks we might have to apply after firing reports.
    STATE_T squash = ONES_STATE;
    return PROCESS_ACCEPTS_IMPL_FN(limex, s, &squash, acceptMask, acceptTable,
                                   offset, callback, context);

    *s = AND_STATE(*s, squash);
}

static never_inline
char PROCESS_ACCEPTS_NOSQUASH_FN(const IMPL_NFA_T *limex, const STATE_T *s,
                                 const STATE_T *acceptMask,
                                 const struct NFAAccept *acceptTable,
                                 u64a offset, NfaCallback callback,
                                 void *context) {
    STATE_T *squash = NULL;
    return PROCESS_ACCEPTS_IMPL_FN(limex, s, squash, acceptMask, acceptTable,
                                   offset, callback, context);
}

// Run EOD accepts. Note that repeat_ctrl and repeat_state may be NULL if this
// LimEx contains no repeat structures.
static really_inline
char TESTEOD_FN(const IMPL_NFA_T *limex, const STATE_T *s,
                const union RepeatControl *repeat_ctrl,
                const char *repeat_state, u64a offset,
                NfaCallback callback, void *context) {
    assert(limex && s);

    // There may not be any EOD accepts in this NFA.
    if (!limex->acceptEodCount) {
        return MO_CONTINUE_MATCHING;
    }

    const STATE_T acceptEodMask = LOAD_FROM_ENG(&limex->acceptAtEOD);
    STATE_T foundAccepts = AND_STATE(*s, acceptEodMask);

    SQUASH_UNTUG_BR_FN(limex, repeat_ctrl, repeat_state,
                       offset + 1 /* EOD 'symbol' */, &foundAccepts);

    if (unlikely(ISNONZERO_STATE(foundAccepts))) {
        const struct NFAAccept *acceptEodTable = getAcceptEodTable(limex);
        if (PROCESS_ACCEPTS_NOSQUASH_FN(limex, &foundAccepts, &acceptEodMask,
                                        acceptEodTable, offset, callback,
                                        context)) {
            return MO_HALT_MATCHING;
        }
    }

    return MO_CONTINUE_MATCHING;
}

// Run accepts corresponding to current state.
static really_inline
char REPORTCURRENT_FN(const IMPL_NFA_T *limex, const struct mq *q) {
    assert(limex && q);
    assert(q->state);
    assert(q_cur_type(q) == MQE_START);

    STATE_T s = *(STATE_T *)q->state;
    STATE_T acceptMask = LOAD_FROM_ENG(&limex->accept);
    STATE_T foundAccepts = AND_STATE(s, acceptMask);

    if (unlikely(ISNONZERO_STATE(foundAccepts))) {
        DEBUG_PRINTF("found accepts\n");
        DEBUG_PRINTF("for nfa %p\n", limex);
        const struct NFAAccept *acceptTable = getAcceptTable(limex);
        u64a offset = q_cur_offset(q);

        if (PROCESS_ACCEPTS_NOSQUASH_FN(limex, &foundAccepts, &acceptMask,
                                        acceptTable, offset, q->cb,
                                        q->context)) {
            return MO_HALT_MATCHING;
        }
    }

    return MO_CONTINUE_MATCHING;
}

static really_inline
STATE_T INITIAL_FN(const IMPL_NFA_T *impl, char onlyDs) {
    return LOAD_FROM_ENG(onlyDs ? &impl->initDS : &impl->init);
}

static really_inline
STATE_T TOP_FN(const IMPL_NFA_T *impl, char onlyDs, STATE_T state) {
    return OR_STATE(INITIAL_FN(impl, onlyDs), state);
}

static really_inline
STATE_T TOPN_FN(const IMPL_NFA_T *limex, STATE_T state, u32 n) {
    assert(n < limex->topCount);
    const ENG_STATE_T *topsptr =
        (const ENG_STATE_T *)((const char *)limex + limex->topOffset);
    STATE_T top = LOAD_FROM_ENG(&topsptr[n]);
    return OR_STATE(top, state);
}

static really_inline
void EXPIRE_ESTATE_FN(const IMPL_NFA_T *limex, struct CONTEXT_T *ctx,
                      u64a offset) {
    assert(limex);
    assert(ctx);

    if (!limex->repeatCount) {
        return;
    }

    DEBUG_PRINTF("expire estate at offset %llu\n", offset);

    const STATE_T cyclics
        = AND_STATE(ctx->s, LOAD_FROM_ENG(&limex->repeatCyclicMask));
    if (ISZERO_STATE(cyclics)) {
        DEBUG_PRINTF("no cyclic states are on\n");
        return;
    }

    for (u32 i = 0; i < limex->repeatCount; i++) {
        const struct NFARepeatInfo *info = GET_NFA_REPEAT_INFO_FN(limex, i);

        u32 cyclicState = info->cyclicState;
        if (!TESTBIT_STATE(cyclics, cyclicState)) {
            continue;
        }

        DEBUG_PRINTF("repeat %u (cyclic state %u) is active\n", i,
                     cyclicState);

        const struct RepeatInfo *repeat = getRepeatInfo(info);
        if (repeat->repeatMax == REPEAT_INF) {
            continue; // can't expire
        }

        const union RepeatControl *repeat_ctrl = ctx->repeat_ctrl + i;
        const char *repeat_state = ctx->repeat_state + info->stateOffset;
        u64a last_top = repeatLastTop(repeat, repeat_ctrl, repeat_state);
        assert(repeat->repeatMax < REPEAT_INF);
        DEBUG_PRINTF("offset %llu, last_top %llu repeatMax %u\n", offset,
                     last_top, repeat->repeatMax);
        u64a adj = 0;
        /* if the cycle's tugs are active at repeat max, it is still alive */
        if (TESTBIT_STATE(LOAD_FROM_ENG(&limex->accept), cyclicState) ||
            TESTBIT_STATE(LOAD_FROM_ENG(&limex->acceptAtEOD), cyclicState)) {
            DEBUG_PRINTF("lazy tug possible - may still be inspected\n");
            adj = 1;
        } else {
            const ENG_STATE_T *tug_mask =
                (const ENG_STATE_T *)((const char *)info + info->tugMaskOffset);
            if (ISNONZERO_STATE(AND_STATE(ctx->s, LOAD_FROM_ENG(tug_mask)))) {
                DEBUG_PRINTF("tug possible - may still be inspected\n");
                adj = 1;
            }
        }

        if (offset >= last_top + repeat->repeatMax + adj) {
            DEBUG_PRINTF("repeat state is stale, squashing state %u\n",
                         cyclicState);
            CLEARBIT_STATE(&ctx->s, cyclicState);
        }
    }
}

// Specialised inAccept call: LimEx NFAs with the "lazy tug" optimisation (see
// UE-1636) need to guard cyclic tug-accepts as well.
static really_inline
char LIMEX_INACCEPT_FN(const IMPL_NFA_T *limex, STATE_T state,
                       union RepeatControl *repeat_ctrl, char *repeat_state,
                       u64a offset, ReportID report) {
    assert(limex);

    const STATE_T accept_mask = LOAD_FROM_ENG(&limex->accept);
    STATE_T accepts = AND_STATE(state, accept_mask);

    // Are we in an accept state?
    if (ISZERO_STATE(accepts)) {
        DEBUG_PRINTF("no accept states are on\n");
        return 0;
    }

    SQUASH_UNTUG_BR_FN(limex, repeat_ctrl, repeat_state, offset, &accepts);

    DEBUG_PRINTF("looking for report %u\n", report);

    const struct NFAAccept *acceptTable = getAcceptTable(limex);

    CHUNK_T chunks[NUM_STATE_CHUNKS];
    memcpy(chunks, &accepts, sizeof(accepts));

    CHUNK_T mask_chunks[NUM_STATE_CHUNKS];
    memcpy(mask_chunks, &accept_mask, sizeof(accept_mask));

    u32 base_index = 0; // Cumulative sum of mask popcount up to current chunk.
    for (u32 i = 0; i < NUM_STATE_CHUNKS; i++) {
        CHUNK_T chunk = chunks[i];
        while (chunk != 0) {
            u32 bit = FIND_AND_CLEAR_FN(&chunk);
            u32 local_idx = RANK_IN_MASK_FN(mask_chunks[i], bit);
            u32 idx = local_idx + base_index;
            assert(idx < limex->acceptCount);
            const struct NFAAccept *a = &acceptTable[idx];
            DEBUG_PRINTF("state %u is on, report list at %u\n",
                         bit + i * (u32)sizeof(chunk) * 8, a->reports);

            if (limexAcceptHasReport((const char *)limex, a, report)) {
                DEBUG_PRINTF("report %u is on\n", report);
                return 1;
            }
        }
        base_index += POPCOUNT_FN(mask_chunks[i]);
    }

    return 0;
}

static really_inline
char LIMEX_INANYACCEPT_FN(const IMPL_NFA_T *limex, STATE_T state,
                          union RepeatControl *repeat_ctrl, char *repeat_state,
                          u64a offset) {
    assert(limex);

    const STATE_T acceptMask = LOAD_FROM_ENG(&limex->accept);
    STATE_T accstate = AND_STATE(state, acceptMask);

    // Are we in an accept state?
    if (ISZERO_STATE(accstate)) {
        DEBUG_PRINTF("no accept states are on\n");
        return 0;
    }

    SQUASH_UNTUG_BR_FN(limex, repeat_ctrl, repeat_state, offset, &accstate);

    return ISNONZERO_STATE(accstate);
}

#undef TESTEOD_FN
#undef REPORTCURRENT_FN
#undef EXPIRE_ESTATE_FN
#undef LIMEX_INACCEPT_FN
#undef LIMEX_INANYACCEPT_FN
#undef INITIAL_FN
#undef TOP_FN
#undef TOPN_FN
#undef CONTEXT_T
#undef IMPL_NFA_T
#undef ONES_STATE
#undef AND_STATE
#undef OR_STATE
#undef ANDNOT_STATE
#undef CLEARBIT_STATE
#undef TESTBIT_STATE
#undef ISNONZERO_STATE
#undef ISZERO_STATE
#undef PROCESS_ACCEPTS_IMPL_FN
#undef PROCESS_ACCEPTS_FN
#undef PROCESS_ACCEPTS_NOSQUASH_FN
#undef SQUASH_UNTUG_BR_FN
#undef GET_NFA_REPEAT_INFO_FN

#undef CHUNK_T
#undef FIND_AND_CLEAR_FN
#undef POPCOUNT_FN
#undef RANK_IN_MASK_FN
#undef NUM_STATE_CHUNKS
