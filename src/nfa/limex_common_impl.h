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

#include "repeat.h"
#include "util/join.h"

/* impl of limex functions which depend only on state size */

#if !defined(SIZE) || !defined(STATE_T) || !defined(INLINE_ATTR)
#  error Must define SIZE and STATE_T and INLINE_ATTR in includer.
#endif

#define IMPL_NFA_T          JOIN(struct LimExNFA, SIZE)

#define TESTEOD_FN          JOIN(moNfaTestEod, SIZE)
#define TESTEOD_REV_FN      JOIN(moNfaRevTestEod, SIZE)
#define LIMEX_INACCEPT_FN   JOIN(limexInAccept, SIZE)
#define EXPIRE_ESTATE_FN    JOIN(limexExpireExtendedState, SIZE)
#define REPORTCURRENT_FN    JOIN(moNfaReportCurrent, SIZE)
#define INITIAL_FN          JOIN(moNfaInitial, SIZE)
#define TOP_FN              JOIN(moNfaTop, SIZE)
#define TOPN_FN             JOIN(moNfaTopN, SIZE)
#define PROCESS_ACCEPTS_FN  JOIN(moProcessAccepts, SIZE)
#define PROCESS_ACCEPTS_NOSQUASH_FN  JOIN(moProcessAcceptsNoSquash, SIZE)
#define CONTEXT_T           JOIN(NFAContext, SIZE)
#define ONES_STATE          JOIN(ones_, STATE_T)
#define LOAD_STATE          JOIN(load_, STATE_T)
#define STORE_STATE         JOIN(store_, STATE_T)
#define AND_STATE           JOIN(and_, STATE_T)
#define OR_STATE            JOIN(or_, STATE_T)
#define ANDNOT_STATE        JOIN(andnot_, STATE_T)
#define CLEARBIT_STATE      JOIN(clearbit_, STATE_T)
#define TESTBIT_STATE       JOIN(testbit_, STATE_T)
#define ISNONZERO_STATE     JOIN(isNonZero_, STATE_T)
#define ISZERO_STATE        JOIN(isZero_, STATE_T)
#define SQUASH_UNTUG_BR_FN  JOIN(lazyTug, SIZE)
#define GET_NFA_REPEAT_INFO_FN JOIN(getNfaRepeatInfo, SIZE)

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
        if (!TESTBIT_STATE(accstate, cyclicState)) {
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

static never_inline
char PROCESS_ACCEPTS_FN(const IMPL_NFA_T *limex, STATE_T *s,
                        const struct NFAAccept *acceptTable, u32 acceptCount,
                        u64a offset, NfaCallback callback, void *context) {
    assert(s);
    assert(limex);
    assert(callback);
    assert(acceptCount);

    // We have squash masks we might have to apply after firing reports.
    STATE_T squash = ONES_STATE;
    const STATE_T *squashMasks = (const STATE_T *)
        ((const char *)limex + limex->squashOffset);

    for (u32 i = 0; i < acceptCount; i++) {
        const struct NFAAccept *a = &acceptTable[i];
        if (TESTBIT_STATE(s, a->state)) {
            DEBUG_PRINTF("state %u is on, firing report id=%u, offset=%llu\n",
                         a->state, a->externalId, offset);
            int rv = callback(offset, a->externalId, context);
            if (unlikely(rv == MO_HALT_MATCHING)) {
                return 1;
            }
            if (a->squash != MO_INVALID_IDX) {
                assert(a->squash < limex->squashCount);
                const STATE_T *sq = &squashMasks[a->squash];
                DEBUG_PRINTF("squash mask %u @ %p\n", a->squash, sq);
                squash = AND_STATE(squash, LOAD_STATE(sq));
            }
        }
    }

    STORE_STATE(s, AND_STATE(LOAD_STATE(s), squash));
    return 0;
}

static never_inline
char PROCESS_ACCEPTS_NOSQUASH_FN(const STATE_T *s,
                                 const struct NFAAccept *acceptTable,
                                 u32 acceptCount, u64a offset,
                                 NfaCallback callback, void *context) {
    assert(s);
    assert(callback);
    assert(acceptCount);

    for (u32 i = 0; i < acceptCount; i++) {
        const struct NFAAccept *a = &acceptTable[i];
        if (TESTBIT_STATE(s, a->state)) {
            DEBUG_PRINTF("state %u is on, firing report id=%u, offset=%llu\n",
                         a->state, a->externalId, offset);
            int rv = callback(offset, a->externalId, context);
            if (unlikely(rv == MO_HALT_MATCHING)) {
                return 1;
            }
        }
    }
    return 0;
}

// Run EOD accepts.
static really_inline
char TESTEOD_FN(const IMPL_NFA_T *limex, const STATE_T *s,
                const union RepeatControl *repeat_ctrl,
                const char *repeat_state, u64a offset, char do_br,
                NfaCallback callback, void *context) {
    assert(limex && s);

    // There may not be any EOD accepts in this NFA.
    if (!limex->acceptEodCount) {
        return MO_CONTINUE_MATCHING;
    }

    const STATE_T acceptEodMask = LOAD_STATE(&limex->acceptAtEOD);
    STATE_T foundAccepts = AND_STATE(LOAD_STATE(s), acceptEodMask);

    if (do_br) {
        SQUASH_UNTUG_BR_FN(limex, repeat_ctrl, repeat_state,
                           offset + 1 /* EOD 'symbol' */, &foundAccepts);
    } else {
        assert(!limex->repeatCount);
    }

    if (unlikely(ISNONZERO_STATE(foundAccepts))) {
        const struct NFAAccept *acceptEodTable = getAcceptEodTable(limex);
        if (PROCESS_ACCEPTS_NOSQUASH_FN(&foundAccepts, acceptEodTable,
                                        limex->acceptEodCount, offset, callback,
                                        context)) {
            return MO_HALT_MATCHING;
        }
    }

    return MO_CONTINUE_MATCHING;
}

static really_inline
char TESTEOD_REV_FN(const IMPL_NFA_T *limex, const STATE_T *s, u64a offset,
                    NfaCallback callback, void *context) {
    assert(limex && s);

    // There may not be any EOD accepts in this NFA.
    if (!limex->acceptEodCount) {
        return MO_CONTINUE_MATCHING;
    }

    STATE_T acceptEodMask = LOAD_STATE(&limex->acceptAtEOD);
    STATE_T foundAccepts = AND_STATE(LOAD_STATE(s), acceptEodMask);

    assert(!limex->repeatCount);

    if (unlikely(ISNONZERO_STATE(foundAccepts))) {
        const struct NFAAccept *acceptEodTable = getAcceptEodTable(limex);
        if (PROCESS_ACCEPTS_NOSQUASH_FN(&foundAccepts, acceptEodTable,
                                        limex->acceptEodCount, offset, callback,
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

    STATE_T s = LOAD_STATE(q->state);
    STATE_T acceptMask = LOAD_STATE(&limex->accept);
    STATE_T foundAccepts = AND_STATE(s, acceptMask);

    if (unlikely(ISNONZERO_STATE(foundAccepts))) {
        DEBUG_PRINTF("found accepts\n");
        DEBUG_PRINTF("for nfa %p\n", limex);
        const struct NFAAccept *acceptTable = getAcceptTable(limex);
        u64a offset = q_cur_offset(q);

        if (PROCESS_ACCEPTS_NOSQUASH_FN(&foundAccepts, acceptTable,
                                        limex->acceptCount, offset, q->cb,
                                        q->context)) {
            return MO_HALT_MATCHING;
        }
    }

    return MO_CONTINUE_MATCHING;
}

static really_inline
STATE_T INITIAL_FN(const IMPL_NFA_T *impl, char onlyDs) {
    return LOAD_STATE(onlyDs ? &impl->initDS : &impl->init);
}

static really_inline
STATE_T TOP_FN(const IMPL_NFA_T *impl, char onlyDs, STATE_T state) {
    return OR_STATE(INITIAL_FN(impl, onlyDs), state);
}

static really_inline
STATE_T TOPN_FN(const IMPL_NFA_T *limex, STATE_T state, u32 n) {
    assert(n < limex->topCount);
    const STATE_T *topsptr =
        (const STATE_T *)((const char *)limex + limex->topOffset);
    STATE_T top = LOAD_STATE(&topsptr[n]);
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

    const STATE_T cyclics =
        AND_STATE(LOAD_STATE(&ctx->s), LOAD_STATE(&limex->repeatCyclicMask));
    if (ISZERO_STATE(cyclics)) {
        DEBUG_PRINTF("no cyclic states are on\n");
        return;
    }

    for (u32 i = 0; i < limex->repeatCount; i++) {
        const struct NFARepeatInfo *info = GET_NFA_REPEAT_INFO_FN(limex, i);

        u32 cyclicState = info->cyclicState;
        if (!TESTBIT_STATE(&cyclics, cyclicState)) {
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
        if (TESTBIT_STATE((const STATE_T *)&limex->accept, cyclicState) ||
            TESTBIT_STATE((const STATE_T *)&limex->acceptAtEOD, cyclicState)) {
            DEBUG_PRINTF("lazy tug possible - may still be inspected\n");
            adj = 1;
        } else {
            const STATE_T *tug_mask =
                (const STATE_T *)((const char *)info + info->tugMaskOffset);
            if (ISNONZERO_STATE(AND_STATE(ctx->s, LOAD_STATE(tug_mask)))) {
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

    const STATE_T acceptMask = LOAD_STATE(&limex->accept);
    STATE_T accstate = AND_STATE(state, acceptMask);

    // Are we in an accept state?
    if (ISZERO_STATE(accstate)) {
        DEBUG_PRINTF("no accept states are on\n");
        return 0;
    }

    SQUASH_UNTUG_BR_FN(limex, repeat_ctrl, repeat_state, offset, &accstate);

    DEBUG_PRINTF("looking for report %u\n", report);

#ifdef DEBUG
    DEBUG_PRINTF("accept states that are on: ");
    for (u32 i = 0; i < sizeof(STATE_T) * 8; i++) {
        if (TESTBIT_STATE(&accstate, i)) printf("%u ", i);
    }
    printf("\n");
#endif

    // Does one of our states match the given report ID?
    const struct NFAAccept *acceptTable = getAcceptTable(limex);
    for (u32 i = 0; i < limex->acceptCount; i++) {
        const struct NFAAccept *a = &acceptTable[i];
        DEBUG_PRINTF("checking idx=%u, externalId=%u\n", a->state,
                     a->externalId);
        if (a->externalId == report && TESTBIT_STATE(&accstate, a->state)) {
            DEBUG_PRINTF("report is on!\n");
            return 1;
        }
    }

    return 0;
}

#undef TESTEOD_FN
#undef TESTEOD_REV_FN
#undef REPORTCURRENT_FN
#undef EXPIRE_ESTATE_FN
#undef LIMEX_INACCEPT_FN
#undef INITIAL_FN
#undef TOP_FN
#undef TOPN_FN
#undef CONTEXT_T
#undef IMPL_NFA_T
#undef ONES_STATE
#undef LOAD_STATE
#undef STORE_STATE
#undef AND_STATE
#undef OR_STATE
#undef ANDNOT_STATE
#undef CLEARBIT_STATE
#undef TESTBIT_STATE
#undef ISNONZERO_STATE
#undef ISZERO_STATE
#undef PROCESS_ACCEPTS_FN
#undef PROCESS_ACCEPTS_NOSQUASH_FN
#undef SQUASH_UNTUG_BR_FN
#undef GET_NFA_REPEAT_INFO_FN

#undef SIZE
#undef STATE_T
#undef INLINE_ATTR
