/*
 * Copyright (c) 2015-2017, Intel Corporation
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

#include "util/join.h"
#include <string.h>

/** \file
  * \brief Limex Execution Engine Or:
  * How I Learned To Stop Worrying And Love The Preprocessor
  *
  * Version 2.0: now with X-Macros, so you get line numbers in your debugger.
  */


#if !defined(SIZE) || !defined(STATE_T) || !defined(LOAD_FROM_ENG)
#  error Must define SIZE, STATE_T, LOAD_FROM_ENG in includer.
#endif

#define LIMEX_API_ROOT   JOIN(nfaExecLimEx, SIZE)

#define IMPL_NFA_T          JOIN(struct LimExNFA, SIZE)

#define TESTEOD_FN          JOIN(moNfaTestEod, SIZE)
#define INITIAL_FN          JOIN(moNfaInitial, SIZE)
#define TOP_FN              JOIN(moNfaTop, SIZE)
#define TOPN_FN             JOIN(moNfaTopN, SIZE)
#define REPORTCURRENT_FN    JOIN(moNfaReportCurrent, SIZE)
#define COMPRESS_FN         JOIN(moNfaCompressState, SIZE)
#define EXPAND_FN           JOIN(moNfaExpandState, SIZE)
#define COMPRESS_REPEATS_FN JOIN(LIMEX_API_ROOT, _Compress_Repeats)
#define EXPAND_REPEATS_FN   JOIN(LIMEX_API_ROOT, _Expand_Repeats)
#define PROCESS_ACCEPTS_FN  JOIN(moProcessAccepts, SIZE)
#define PROCESS_ACCEPTS_NOSQUASH_FN  JOIN(moProcessAcceptsNoSquash, SIZE)
#define GET_NFA_REPEAT_INFO_FN JOIN(getNfaRepeatInfo, SIZE)
#define RUN_ACCEL_FN        JOIN(LIMEX_API_ROOT, _Run_Accel)
#define RUN_EXCEPTIONS_FN   JOIN(LIMEX_API_ROOT, _Run_Exceptions)
#define REV_STREAM_FN       JOIN(LIMEX_API_ROOT, _Rev_Stream)
#define LOOP_NOACCEL_FN     JOIN(LIMEX_API_ROOT, _Loop_No_Accel)
#define STREAM_FN           JOIN(LIMEX_API_ROOT, _Stream)
#define STREAMCB_FN         JOIN(LIMEX_API_ROOT, _Stream_CB)
#define STREAMFIRST_FN      JOIN(LIMEX_API_ROOT, _Stream_First)
#define STREAMSILENT_FN     JOIN(LIMEX_API_ROOT, _Stream_Silent)
#define CONTEXT_T           JOIN(NFAContext, SIZE)
#define EXCEPTION_T         JOIN(struct NFAException, SIZE)
#define AND_STATE           JOIN(and_, STATE_T)
#define ANDNOT_STATE        JOIN(andnot_, STATE_T)
#define OR_STATE            JOIN(or_, STATE_T)
#define LSHIFT_STATE        JOIN(lshift_, STATE_T)
#define TESTBIT_STATE       JOIN(testbit_, STATE_T)
#define CLEARBIT_STATE      JOIN(clearbit_, STATE_T)
#define ZERO_STATE          JOIN(zero_, STATE_T)
#define ISNONZERO_STATE     JOIN(isNonZero_, STATE_T)
#define ISZERO_STATE        JOIN(isZero_, STATE_T)
#define NOTEQ_STATE         JOIN(noteq_, STATE_T)

// Pick an appropriate diffrich function for this platform.
#ifdef ARCH_64_BIT
#define DIFFRICH_STATE JOIN(diffrich64_, STATE_T)
#else
#define DIFFRICH_STATE JOIN(diffrich_, STATE_T)
#endif

#define EXPIRE_ESTATE_FN    JOIN(limexExpireExtendedState, SIZE)
#define SQUASH_UNTUG_BR_FN  JOIN(lazyTug, SIZE)

// Acceleration and exception masks: we load them on the fly for really big
// models.
#if SIZE < 256
#define ACCEL_MASK              accelMask
#define ACCEL_AND_FRIENDS_MASK  accel_and_friendsMask
#define EXCEPTION_MASK          exceptionMask
#else
#define ACCEL_MASK              LOAD_FROM_ENG(&limex->accel)
#define ACCEL_AND_FRIENDS_MASK  LOAD_FROM_ENG(&limex->accel_and_friends)
#define EXCEPTION_MASK          LOAD_FROM_ENG(&limex->exceptionMask)
#endif

// Run exception processing, if necessary. Returns 0 if scanning should
// continue, 1 if an accept was fired and the user instructed us to halt.
static really_inline
char RUN_EXCEPTIONS_FN(const IMPL_NFA_T *limex, const EXCEPTION_T *exceptions,
                       STATE_T s, const STATE_T emask, size_t i, u64a offset,
                       STATE_T *succ, u64a *final_loc, struct CONTEXT_T *ctx,
                       const char flags, const char in_rev,
                       const char first_match) {
    STATE_T estate = AND_STATE(s, emask);
    u32 diffmask = DIFFRICH_STATE(ZERO_STATE, estate);
    if (likely(!diffmask)) {
        return 0; // No exceptions to process.
    }

    if (first_match && i) {
        STATE_T acceptMask = LOAD_FROM_ENG(&limex->accept);
        STATE_T foundAccepts = AND_STATE(s, acceptMask);
        if (unlikely(ISNONZERO_STATE(foundAccepts))) {
            DEBUG_PRINTF("first match at %zu\n", i);
            DEBUG_PRINTF("for nfa %p\n", limex);
            assert(final_loc);
            ctx->s = s;
            *final_loc = i;
            return 1; // Halt matching.
        }
    }

    u64a callback_offset = i + offset;
    char localflags = (!i && !in_rev) ? NO_OUTPUT | FIRST_BYTE : flags;

    int rv = JOIN(processExceptional, SIZE)(
        pass_state, pass_estate, diffmask, succ, limex, exceptions,
        callback_offset, ctx, in_rev, localflags);
    if (rv == PE_RV_HALT) {
        return 1; // Halt matching.
    }

    return 0;
}

static really_inline
size_t RUN_ACCEL_FN(const STATE_T s, UNUSED const STATE_T accelMask,
                    UNUSED const IMPL_NFA_T *limex, const u8 *accelTable,
                    const union AccelAux *accelAux, const u8 *input, size_t i,
                    size_t length) {
    size_t j;
#if SIZE < 128
    // For small cases, we pass the state by value.
    j = JOIN(doAccel, SIZE)(s, accelMask, accelTable, accelAux, input, i,
                            length);
#else
    j = JOIN(doAccel, SIZE)(&s, limex, accelTable, accelAux, input, i, length);
#endif

    assert(j >= i);
    assert(i <= length);
    return j;
}

// Shift macros for Limited NFAs. Defined in terms of uniform ops.
// LimExNFAxxx ptr in 'limex' and the current state in 's'
#define NFA_EXEC_LIM_SHIFT(limex_m, curr_m, shift_idx)                         \
    LSHIFT_STATE(AND_STATE(curr_m, LOAD_FROM_ENG(&limex_m->shift[shift_idx])), \
                 limex_m->shiftAmount[shift_idx])

// Calculate the (limited model) successors for a number of variable shifts.
// Assumes current state in 'curr_m' and places the successors in 'succ_m'.
#define NFA_EXEC_GET_LIM_SUCC(limex_m, curr_m, succ_m)                         \
    do {                                                                       \
        succ_m = NFA_EXEC_LIM_SHIFT(limex_m, curr_m, 0);                       \
        switch (limex_m->shiftCount) {                                         \
        case 8:                                                                \
            succ_m = OR_STATE(succ_m, NFA_EXEC_LIM_SHIFT(limex_m, curr_m, 7)); \
            /* fallthrough */                                                  \
        case 7:                                                                \
            succ_m = OR_STATE(succ_m, NFA_EXEC_LIM_SHIFT(limex_m, curr_m, 6)); \
            /* fallthrough */                                                  \
        case 6:                                                                \
            succ_m = OR_STATE(succ_m, NFA_EXEC_LIM_SHIFT(limex_m, curr_m, 5)); \
            /* fallthrough */                                                  \
        case 5:                                                                \
            succ_m = OR_STATE(succ_m, NFA_EXEC_LIM_SHIFT(limex_m, curr_m, 4)); \
            /* fallthrough */                                                  \
        case 4:                                                                \
            succ_m = OR_STATE(succ_m, NFA_EXEC_LIM_SHIFT(limex_m, curr_m, 3)); \
            /* fallthrough */                                                  \
        case 3:                                                                \
            succ_m = OR_STATE(succ_m, NFA_EXEC_LIM_SHIFT(limex_m, curr_m, 2)); \
            /* fallthrough */                                                  \
        case 2:                                                                \
            succ_m = OR_STATE(succ_m, NFA_EXEC_LIM_SHIFT(limex_m, curr_m, 1)); \
            /* fallthrough */                                                  \
        case 1:                                                                \
            /* fallthrough */                                                  \
        case 0:                                                                \
            ;                                                                  \
        }                                                                      \
    } while (0)

/**
 * \brief LimEx NFAS inner loop without accel.
 *
 * Note that the "all zeroes" early death check is only performed if can_die is
 * true.
 *
 */
static really_inline
char LOOP_NOACCEL_FN(const IMPL_NFA_T *limex, const u8 *input, size_t *loc,
                     size_t length, STATE_T *s_ptr, struct CONTEXT_T *ctx,
                     u64a offset, const char flags, u64a *final_loc,
                     const char first_match, const char can_die) {
    const ENG_STATE_T *reach = get_reach_table(limex);
#if SIZE < 256
    const STATE_T exceptionMask = LOAD_FROM_ENG(&limex->exceptionMask);
#endif
    const EXCEPTION_T *exceptions = getExceptionTable(EXCEPTION_T, limex);
    STATE_T s = *s_ptr;

    size_t i = *loc;
    for (; i != length; i++) {
        DUMP_INPUT(i);
        if (can_die && ISZERO_STATE(s)) {
            DEBUG_PRINTF("no states are switched on, early exit\n");
            break;
        }

        STATE_T succ;
        NFA_EXEC_GET_LIM_SUCC(limex, s, succ);

        if (RUN_EXCEPTIONS_FN(limex, exceptions, s, EXCEPTION_MASK, i, offset,
                              &succ, final_loc, ctx, flags, 0, first_match)) {
            return MO_HALT_MATCHING;
        }

        u8 c = input[i];
        s = AND_STATE(succ, LOAD_FROM_ENG(&reach[limex->reachMap[c]]));
    }

    *loc = i;
    *s_ptr = s;
    return MO_CONTINUE_MATCHING;
}

static really_inline
char STREAM_FN(const IMPL_NFA_T *limex, const u8 *input, size_t length,
               struct CONTEXT_T *ctx, u64a offset, const char flags,
               u64a *final_loc, const char first_match) {
    const ENG_STATE_T *reach = get_reach_table(limex);
#if SIZE < 256
    const STATE_T accelMask = LOAD_FROM_ENG(&limex->accel);
    const STATE_T accel_and_friendsMask
        = LOAD_FROM_ENG(&limex->accel_and_friends);
    const STATE_T exceptionMask = LOAD_FROM_ENG(&limex->exceptionMask);
#endif
    const u8 *accelTable =
        (const u8 *)((const char *)limex + limex->accelTableOffset);
    const union AccelAux *accelAux =
        (const union AccelAux *)((const char *)limex + limex->accelAuxOffset);
    const EXCEPTION_T *exceptions = getExceptionTable(EXCEPTION_T, limex);
    STATE_T s = ctx->s;

    /* assert(ISALIGNED_16(exceptions)); */
    /* assert(ISALIGNED_16(reach)); */

    size_t i = 0;
    size_t min_accel_offset = 0;
    if (!limex->accelCount || length < ACCEL_MIN_LEN) {
        min_accel_offset = length;
        goto without_accel;
    } else {
        goto with_accel;
    }

without_accel:
    if (limex->flags & LIMEX_FLAG_CANNOT_DIE) {
        const char can_die = 0;
        if (LOOP_NOACCEL_FN(limex, input, &i, min_accel_offset, &s, ctx, offset,
                            flags, final_loc, first_match,
                            can_die) == MO_HALT_MATCHING) {
            return MO_HALT_MATCHING;
        }
    } else {
        const char can_die = 1;
        if (LOOP_NOACCEL_FN(limex, input, &i, min_accel_offset, &s, ctx, offset,
                            flags, final_loc, first_match,
                            can_die) == MO_HALT_MATCHING) {
            return MO_HALT_MATCHING;
        }
    }

with_accel:
    for (; i != length; i++) {
        DUMP_INPUT(i);
        if (i + 16 <= length &&
            ISZERO_STATE(ANDNOT_STATE(ACCEL_AND_FRIENDS_MASK, s))) {
            DEBUG_PRINTF("current states are all accelerable\n");
            assert(i + 16 <= length);
            size_t post_idx =
                RUN_ACCEL_FN(s, ACCEL_MASK, limex, accelTable, accelAux, input,
                             i, length);
            if (post_idx != i) {
                /* squashing any friends as they may no longer be valid;
                 * offset back off should ensure they weren't doing anything
                 * important */
                s = AND_STATE(ACCEL_MASK, s);
            }

            if (i && post_idx < min_accel_offset + BAD_ACCEL_DIST) {
                min_accel_offset = post_idx + BIG_ACCEL_PENALTY;
            } else {
                min_accel_offset = post_idx + SMALL_ACCEL_PENALTY;
            }

            if (min_accel_offset >= length - ACCEL_MIN_LEN) {
                min_accel_offset = length;
            }

            DEBUG_PRINTF("advanced %zd, next accel chance in %zd/%zd\n",
                         post_idx - i, min_accel_offset - post_idx,
                         length - post_idx);

            i = post_idx;
            if (i == length) {
                break; /* all chars eaten, break out of loop */
            }
            goto without_accel;
        }

        STATE_T succ;
        NFA_EXEC_GET_LIM_SUCC(limex, s, succ);

        if (RUN_EXCEPTIONS_FN(limex, exceptions, s, EXCEPTION_MASK, i, offset,
                              &succ, final_loc, ctx, flags, 0, first_match)) {
            return MO_HALT_MATCHING;
        }

        u8 c = input[i];
        s = AND_STATE(succ, LOAD_FROM_ENG(&reach[limex->reachMap[c]]));
    }

    ctx->s = s;

    if ((first_match || (flags & CALLBACK_OUTPUT)) && limex->acceptCount) {
        STATE_T acceptMask = LOAD_FROM_ENG(&limex->accept);
        const struct NFAAccept *acceptTable = getAcceptTable(limex);
        STATE_T foundAccepts = AND_STATE(s, acceptMask);
        if (unlikely(ISNONZERO_STATE(foundAccepts))) {
            if (first_match) {
                ctx->s = s;
                assert(final_loc);
                *final_loc = length;
                return MO_HALT_MATCHING;
            } else if (PROCESS_ACCEPTS_FN(limex, &ctx->s, &acceptMask,
                                          acceptTable, offset + length,
                                          ctx->callback, ctx->context)) {
                return MO_HALT_MATCHING;
            }
        }
    }
    if (first_match) {
        assert(final_loc);
        *final_loc = length;
    }
    return MO_CONTINUE_MATCHING;
}

static never_inline
char REV_STREAM_FN(const IMPL_NFA_T *limex, const u8 *input, size_t length,
                   struct CONTEXT_T *ctx, u64a offset) {
    const ENG_STATE_T *reach = get_reach_table(limex);
#if SIZE < 256
    const STATE_T exceptionMask = LOAD_FROM_ENG(&limex->exceptionMask);
#endif
    const EXCEPTION_T *exceptions = getExceptionTable(EXCEPTION_T, limex);
    STATE_T s = ctx->s;

    /* assert(ISALIGNED_16(exceptions)); */
    /* assert(ISALIGNED_16(reach)); */
    const char flags = CALLBACK_OUTPUT;
    u64a *final_loc = NULL;

    for (size_t i = length; i != 0; i--) {
        DUMP_INPUT(i - 1);
        if (ISZERO_STATE(s)) {
            DEBUG_PRINTF("no states are switched on, early exit\n");
            ctx->s = s;
            return MO_CONTINUE_MATCHING;
        }

        STATE_T succ;
        NFA_EXEC_GET_LIM_SUCC(limex, s, succ);

        if (RUN_EXCEPTIONS_FN(limex, exceptions, s, EXCEPTION_MASK, i, offset,
                              &succ, final_loc, ctx, flags, 1, 0)) {
            return MO_HALT_MATCHING;
        }

        u8 c = input[i - 1];
        s = AND_STATE(succ, LOAD_FROM_ENG(&reach[limex->reachMap[c]]));
    }

    ctx->s = s;

    STATE_T acceptMask = LOAD_FROM_ENG(&limex->accept);
    const struct NFAAccept *acceptTable = getAcceptTable(limex);
    const u32 acceptCount = limex->acceptCount;
    assert(flags & CALLBACK_OUTPUT);
    if (acceptCount) {
        STATE_T foundAccepts = AND_STATE(s, acceptMask);
        if (unlikely(ISNONZERO_STATE(foundAccepts))) {
            if (PROCESS_ACCEPTS_NOSQUASH_FN(limex, &ctx->s, &acceptMask,
                                            acceptTable, offset, ctx->callback,
                                            ctx->context)) {
                return MO_HALT_MATCHING;
            }
        }
    }
    return MO_CONTINUE_MATCHING;
}

static really_inline
void COMPRESS_REPEATS_FN(const IMPL_NFA_T *limex, void *dest, void *src,
                         u64a offset) {
    if (!limex->repeatCount) {
        return;
    }

    STATE_T s = *(STATE_T *)src;

    if (ISZERO_STATE(AND_STATE(LOAD_FROM_ENG(&limex->repeatCyclicMask), s))) {
        DEBUG_PRINTF("no cyclics are on\n");
        return;
    }

    const union RepeatControl *ctrl =
        getRepeatControlBaseConst((const char *)src, sizeof(STATE_T));
    char *state_base = (char *)dest + limex->stateSize;

    for (u32 i = 0; i < limex->repeatCount; i++) {
        DEBUG_PRINTF("repeat %u\n", i);
        const struct NFARepeatInfo *info = GET_NFA_REPEAT_INFO_FN(limex, i);

        const ENG_STATE_T *tug_mask =
            (const ENG_STATE_T *)((const char *)info + info->tugMaskOffset);
        /* repeat may still be inspected if its tug state is on */
        if (!TESTBIT_STATE(s, info->cyclicState)
            && ISZERO_STATE(AND_STATE(s, LOAD_FROM_ENG(tug_mask)))) {
            DEBUG_PRINTF("is dead\n");
            continue;
        }

        const struct RepeatInfo *repeat = getRepeatInfo(info);
        DEBUG_PRINTF("packing state (packedCtrlOffset=%u)\n",
                     info->packedCtrlOffset);
        repeatPack(state_base + info->packedCtrlOffset, repeat, &ctrl[i],
                   offset);
    }

    *(STATE_T *)src = s;
}

char JOIN(LIMEX_API_ROOT, _queueCompressState)(const struct NFA *n,
                                               const struct mq *q, s64a loc) {
    void *dest = q->streamState;
    void *src = q->state;
    u8 key = queue_prev_byte(q, loc);
    const IMPL_NFA_T *limex = getImplNfa(n);
    COMPRESS_REPEATS_FN(limex, dest, src, q->offset + loc);
    COMPRESS_FN(limex, dest, src, key);
    return 0;
}

static really_inline
void EXPAND_REPEATS_FN(const IMPL_NFA_T *limex, void *dest, const void *src,
                       u64a offset) {
    if (!limex->repeatCount) {
        return;
    }

    // Note: state has already been expanded into 'dest'.
    const STATE_T cyclics =
        AND_STATE(*(STATE_T *)dest, LOAD_FROM_ENG(&limex->repeatCyclicMask));
    if (ISZERO_STATE(cyclics)) {
        DEBUG_PRINTF("no cyclics are on\n");
        return;
    }

    union RepeatControl *ctrl =
        getRepeatControlBase((char *)dest, sizeof(STATE_T));
    const char *state_base = (const char *)src + limex->stateSize;

    for (u32 i = 0; i < limex->repeatCount; i++) {
        DEBUG_PRINTF("repeat %u\n", i);
        const struct NFARepeatInfo *info = GET_NFA_REPEAT_INFO_FN(limex, i);
        const ENG_STATE_T *tug_mask =
            (const ENG_STATE_T *)((const char *)info + info->tugMaskOffset);

        if (!TESTBIT_STATE(cyclics, info->cyclicState)
            && ISZERO_STATE(AND_STATE(cyclics, LOAD_FROM_ENG(tug_mask)))) {
            DEBUG_PRINTF("is dead\n");
            continue;
        }

        DEBUG_PRINTF("unpacking state (packedCtrlOffset=%u)\n",
                     info->packedCtrlOffset);
        const struct RepeatInfo *repeat = getRepeatInfo(info);
        repeatUnpack(state_base + info->packedCtrlOffset, repeat, offset,
                     &ctrl[i]);
    }
}

char JOIN(LIMEX_API_ROOT, _expandState)(const struct NFA *n, void *dest,
                                           const void *src, u64a offset,
                                           u8 key) {
    const IMPL_NFA_T *limex = getImplNfa(n);
    EXPAND_FN(limex, dest, src, key);
    EXPAND_REPEATS_FN(limex, dest, src, offset);
    return 0;
}

char JOIN(LIMEX_API_ROOT, _queueInitState)(const struct NFA *n, struct mq *q) {
    *(STATE_T *)q->state = ZERO_STATE;

    // Zero every bounded repeat control block in state.
    const IMPL_NFA_T *limex = getImplNfa(n);
    union RepeatControl *ctrl = getRepeatControlBase(q->state, sizeof(STATE_T));
    for (u32 i = 0; i < limex->repeatCount; i++) {
        memset(&ctrl[i], 0, sizeof(*ctrl));
    }

    return 0;
}

char JOIN(LIMEX_API_ROOT, _initCompressedState)(const struct NFA *n,
                                                   u64a offset, void *state,
                                                   u8 key) {
    const IMPL_NFA_T *limex = getImplNfa(n);

    STATE_T s = INITIAL_FN(limex, !!offset);
    if (ISZERO_STATE(s)) {
        DEBUG_PRINTF("state went to zero\n");
        return 0;
    }

    // NFA is still active, compress its state and ship it out.
    COMPRESS_FN(limex, state, &s, key);

    // Zero every packed bounded repeat control block in stream state.
    char *repeat_region = (char *)state + limex->stateSize;
    for (u32 i = 0; i < limex->repeatCount; i++) {
        const struct NFARepeatInfo *info = GET_NFA_REPEAT_INFO_FN(limex, i);
        const struct RepeatInfo *repeat = getRepeatInfo(info);

        memset(repeat_region + info->packedCtrlOffset, 0,
               repeat->packedCtrlSize);
    }

    return 1;
}

// Helper for history buffer scans, which catch up the NFA state but don't emit
// matches.
static never_inline
void STREAMSILENT_FN(const IMPL_NFA_T *limex, const u8 *input, size_t length,
                     struct CONTEXT_T *ctx, u64a offset) {
    const char first_match = 0;

    UNUSED char rv = STREAM_FN(limex, input, length, ctx, offset, NO_OUTPUT,
                               NULL, first_match);
    assert(rv != MO_HALT_MATCHING);
}

static never_inline
char STREAMCB_FN(const IMPL_NFA_T *limex, const u8 *input, size_t length,
                 struct CONTEXT_T *ctx, u64a offset) {
    const char first_match = 0;
    assert(ISALIGNED_CL(ctx));
    return STREAM_FN(limex, input, length, ctx, offset, CALLBACK_OUTPUT, NULL,
                     first_match);
}

static never_inline
char STREAMFIRST_FN(const IMPL_NFA_T *limex, const u8 *input, size_t length,
                    struct CONTEXT_T *ctx, u64a offset, u64a *final_loc) {
    const char first_match = 1; // Run to first match and stop, no callbacks.
    return STREAM_FN(limex, input, length, ctx, offset, NO_OUTPUT, final_loc,
                     first_match);
}

// Common code for handling the current event on the queue.
static really_inline
void JOIN(LIMEX_API_ROOT, _HandleEvent)(const IMPL_NFA_T *limex,
                                           struct mq *q, struct CONTEXT_T *ctx,
                                           u64a sp) {
#define DEFINE_CASE(ee)                                                        \
    case ee:                                                                   \
        DEBUG_PRINTF(#ee "\n");

    u32 e = q->items[q->cur].type;
    switch (e) {
        DEFINE_CASE(MQE_TOP)
            ctx->s = TOP_FN(limex, !!sp, ctx->s);
            break;
        DEFINE_CASE(MQE_START)
            break;
        DEFINE_CASE(MQE_END)
            break;
        default:
            assert(e >= MQE_TOP_FIRST);
            assert(e < MQE_INVALID);
            DEBUG_PRINTF("MQE_TOP + %d\n", ((int)e - MQE_TOP_FIRST));
            ctx->s = TOPN_FN(limex, ctx->s, e - MQE_TOP_FIRST);
    }
#undef DEFINE_CASE
}

// "Classic" queue call, used by outfixes
char JOIN(LIMEX_API_ROOT, _Q)(const struct NFA *n, struct mq *q, s64a end) {
    const IMPL_NFA_T *limex = getImplNfa(n);

    if (q->report_current) {
        char rv = REPORTCURRENT_FN(limex, q);

        q->report_current = 0;

        if (rv == MO_HALT_MATCHING) {
            return MO_HALT_MATCHING;
        }
    }

    if (q->cur == q->end) {
        return 1;
    }

    assert(q->cur + 1 < q->end); /* require at least two items */

    struct CONTEXT_T ctx;
    ctx.repeat_ctrl = getRepeatControlBase(q->state, sizeof(STATE_T));
    ctx.repeat_state = q->streamState + limex->stateSize;
    ctx.callback = q->cb;
    ctx.context = q->context;
    ctx.cached_estate = ZERO_STATE;
    ctx.cached_br = 0;

    assert(q->items[q->cur].location >= 0);
    DEBUG_PRINTF("LOAD STATE\n");
    ctx.s = *(STATE_T *)q->state;
    assert(q->items[q->cur].type == MQE_START);

    u64a offset = q->offset;
    u64a sp = offset + q->items[q->cur].location;
    u64a end_abs = offset + end;
    q->cur++;

    while (q->cur < q->end && sp <= end_abs) {
        u64a ep = offset + q->items[q->cur].location;
        ep = MIN(ep, end_abs);
        assert(ep >= sp);

        assert(sp >= offset); // We no longer do history buffer scans here.

        if (sp >= ep) {
            goto scan_done;
        }

        /* do main buffer region */
        DEBUG_PRINTF("MAIN BUFFER SCAN\n");
        assert(ep - offset <= q->length);
        if (STREAMCB_FN(limex, q->buffer + sp - offset, ep - sp, &ctx, sp)
                == MO_HALT_MATCHING) {
            *(STATE_T *)q->state = ZERO_STATE;
            return 0;
        }

        DEBUG_PRINTF("SCAN DONE\n");
    scan_done:
        sp = ep;

       if (sp != offset + q->items[q->cur].location) {
           assert(q->cur);
           DEBUG_PRINTF("bail: sp = %llu end_abs == %llu offset == %llu\n",
                        sp, end_abs, offset);
           assert(sp == end_abs);
           q->cur--;
           q->items[q->cur].type = MQE_START;
           q->items[q->cur].location = sp - offset;
           DEBUG_PRINTF("bailing q->cur %u q->end %u\n", q->cur, q->end);
           *(STATE_T *)q->state = ctx.s;
           return MO_ALIVE;
       }

        JOIN(LIMEX_API_ROOT, _HandleEvent)(limex, q, &ctx, sp);

        q->cur++;
    }

    EXPIRE_ESTATE_FN(limex, &ctx, sp);

    DEBUG_PRINTF("END\n");
    *(STATE_T *)q->state = ctx.s;

    if (q->cur != q->end) {
        q->cur--;
        q->items[q->cur].type = MQE_START;
        q->items[q->cur].location = sp - offset;
        return MO_ALIVE;
    }

    return ISNONZERO_STATE(ctx.s);
}

/* used by suffix execution in Rose */
char JOIN(LIMEX_API_ROOT, _Q2)(const struct NFA *n, struct mq *q, s64a end) {
    const IMPL_NFA_T *limex = getImplNfa(n);

    if (q->report_current) {
        char rv = REPORTCURRENT_FN(limex, q);

        q->report_current = 0;

        if (rv == MO_HALT_MATCHING) {
            return MO_HALT_MATCHING;
        }
    }

    if (q->cur == q->end) {
        return 1;
    }

    assert(q->cur + 1 < q->end); /* require at least two items */

    struct CONTEXT_T ctx;
    ctx.repeat_ctrl = getRepeatControlBase(q->state, sizeof(STATE_T));
    ctx.repeat_state = q->streamState + limex->stateSize;
    ctx.callback = q->cb;
    ctx.context = q->context;
    ctx.cached_estate = ZERO_STATE;
    ctx.cached_br = 0;

    DEBUG_PRINTF("LOAD STATE\n");
    ctx.s = *(STATE_T *)q->state;
    assert(q->items[q->cur].type == MQE_START);

    u64a offset = q->offset;
    u64a sp = offset + q->items[q->cur].location;
    u64a end_abs = offset + end;
    q->cur++;

    while (q->cur < q->end && sp <= end_abs) {
        u64a ep = offset + q->items[q->cur].location;
        DEBUG_PRINTF("sp = %llu, ep = %llu, end_abs = %llu\n",
                     sp, ep, end_abs);
        ep = MIN(ep, end_abs);
        assert(ep >= sp);

        if (sp < offset) {
            DEBUG_PRINTF("HISTORY BUFFER SCAN\n");
            assert(offset - sp <= q->hlength);
            u64a local_ep = MIN(offset, ep);
            u64a final_look = 0;
            /* we are starting inside the history buffer */
            if (STREAMFIRST_FN(limex, q->history + q->hlength + sp - offset,
                               local_ep - sp, &ctx, sp,
                               &final_look) == MO_HALT_MATCHING) {
                DEBUG_PRINTF("final_look:%llu sp:%llu end_abs:%llu "
                             "offset:%llu\n", final_look, sp, end_abs, offset);
                assert(q->cur);
                q->cur--;
                q->items[q->cur].type = MQE_START;
                q->items[q->cur].location = sp + final_look - offset;
                *(STATE_T *)q->state = ctx.s;
                return MO_MATCHES_PENDING;
            }

            sp = local_ep;
        }

        if (sp >= ep) {
            goto scan_done;
        }

        /* do main buffer region */
        u64a final_look = 0;
        assert(ep - offset <= q->length);
        if (STREAMFIRST_FN(limex, q->buffer + sp - offset, ep - sp, &ctx, sp,
                           &final_look) == MO_HALT_MATCHING) {
            DEBUG_PRINTF("final_look:%llu sp:%llu end_abs:%llu offset:%llu\n",
                         final_look, sp, end_abs, offset);
            assert(q->cur);
            q->cur--;
            q->items[q->cur].type = MQE_START;
            q->items[q->cur].location = sp + final_look - offset;
            *(STATE_T *)q->state = ctx.s;
            return MO_MATCHES_PENDING;
        }

    scan_done:
        sp = ep;

        if (sp != offset + q->items[q->cur].location) {
            assert(q->cur);
            DEBUG_PRINTF("bail: sp = %llu end_abs == %llu offset == %llu\n",
                         sp, end_abs, offset);
            assert(sp == end_abs);
            q->cur--;
            q->items[q->cur].type = MQE_START;
            q->items[q->cur].location = sp - offset;
            DEBUG_PRINTF("bailing q->cur %u q->end %u\n", q->cur, q->end);
            *(STATE_T *)q->state = ctx.s;
            return MO_ALIVE;
        }

        JOIN(LIMEX_API_ROOT, _HandleEvent)(limex, q, &ctx, sp);

        q->cur++;
    }

    EXPIRE_ESTATE_FN(limex, &ctx, sp);

    DEBUG_PRINTF("END\n");
    *(STATE_T *)q->state = ctx.s;

    if (q->cur != q->end) {
        q->cur--;
        q->items[q->cur].type = MQE_START;
        q->items[q->cur].location = sp - offset;
        return MO_ALIVE;
    }

    return ISNONZERO_STATE(ctx.s);
}

// Used for execution Rose prefix/infixes.
char JOIN(LIMEX_API_ROOT, _QR)(const struct NFA *n, struct mq *q,
                                  ReportID report) {
    const IMPL_NFA_T *limex = getImplNfa(n);

    if (q->cur == q->end) {
        return 1;
    }

    assert(q->cur + 1 < q->end); /* require at least two items */

    struct CONTEXT_T ctx;
    ctx.repeat_ctrl = getRepeatControlBase(q->state, sizeof(STATE_T));
    ctx.repeat_state = q->streamState + limex->stateSize;
    ctx.callback = NULL;
    ctx.context = NULL;
    ctx.cached_estate = ZERO_STATE;
    ctx.cached_br = 0;

    DEBUG_PRINTF("LOAD STATE\n");
    ctx.s = *(STATE_T *)q->state;
    assert(q->items[q->cur].type == MQE_START);

    u64a offset = q->offset;
    u64a sp = offset + q->items[q->cur].location;
    q->cur++;

    while (q->cur < q->end) {
        u64a ep = offset + q->items[q->cur].location;
        if (n->maxWidth) {
            if (ep - sp > n->maxWidth) {
                sp = ep - n->maxWidth;
                ctx.s = INITIAL_FN(limex, !!sp);
            }
        }
        assert(ep >= sp);

        if (sp < offset) {
            DEBUG_PRINTF("HISTORY BUFFER SCAN\n");
            assert(offset - sp <= q->hlength);
            u64a local_ep = MIN(offset, ep);
            /* we are starting inside the history buffer */
            STREAMSILENT_FN(limex, q->history + q->hlength + sp - offset,
                            local_ep - sp, &ctx, sp);

            sp = local_ep;
        }

        if (sp >= ep) {
            goto scan_done;
        }

        /* do main buffer region */
        DEBUG_PRINTF("MAIN BUFFER SCAN\n");
        assert(ep - offset <= q->length);
        STREAMSILENT_FN(limex, q->buffer + sp - offset, ep - sp, &ctx, sp);

        DEBUG_PRINTF("SCAN DONE\n");
    scan_done:
        sp = ep;

        JOIN(LIMEX_API_ROOT, _HandleEvent)(limex, q, &ctx, sp);

        q->cur++;
    }

    EXPIRE_ESTATE_FN(limex, &ctx, sp);

    DEBUG_PRINTF("END, nfa is %s\n",
                 ISNONZERO_STATE(ctx.s) ? "still alive" : "dead");

    *(STATE_T *)q->state = ctx.s;

    if (JOIN(limexInAccept, SIZE)(limex, ctx.s, ctx.repeat_ctrl,
                                  ctx.repeat_state, sp + 1, report)) {
        return MO_MATCHES_PENDING;
    }

    return ISNONZERO_STATE(ctx.s);
}

char JOIN(LIMEX_API_ROOT, _testEOD)(const struct NFA *n, const char *state,
                                    const char *streamState, u64a offset,
                                    NfaCallback callback, void *context) {
    assert(n && state);

    const IMPL_NFA_T *limex = getImplNfa(n);
    const STATE_T *sptr = (const STATE_T *)state;
    const union RepeatControl *repeat_ctrl =
        getRepeatControlBaseConst(state, sizeof(STATE_T));
    const char *repeat_state = streamState + limex->stateSize;
    return TESTEOD_FN(limex, sptr, repeat_ctrl, repeat_state, offset, callback,
                      context);
}

char JOIN(LIMEX_API_ROOT, _reportCurrent)(const struct NFA *n, struct mq *q) {
    const IMPL_NFA_T *limex = getImplNfa(n);
    REPORTCURRENT_FN(limex, q);
    return 1;
}

// Block mode reverse scan.
char JOIN(LIMEX_API_ROOT, _B_Reverse)(const struct NFA *n, u64a offset,
                                      const u8 *buf, size_t buflen,
                                      const u8 *hbuf, size_t hlen,
                                      NfaCallback cb, void *context) {
    assert(buf || hbuf);
    assert(buflen || hlen);

    struct CONTEXT_T ctx;
    ctx.repeat_ctrl = NULL;
    ctx.repeat_state = NULL;
    ctx.callback = cb;
    ctx.context = context;
    ctx.cached_estate = ZERO_STATE;
    ctx.cached_br = 0;

    const IMPL_NFA_T *limex = getImplNfa(n);
    ctx.s = INITIAL_FN(limex, 0); // always anchored

    // 'buf' may be null, for example when we're scanning at EOD time.
    if (buflen) {
        assert(buf);
        DEBUG_PRINTF("MAIN BUFFER SCAN, %zu bytes\n", buflen);
        offset -= buflen;
        REV_STREAM_FN(limex, buf, buflen, &ctx, offset);
    }

    if (hlen) {
        assert(hbuf);
        DEBUG_PRINTF("HISTORY BUFFER SCAN, %zu bytes\n", hlen);
        offset -= hlen;
        REV_STREAM_FN(limex, hbuf, hlen, &ctx, offset);
    }

    if (offset == 0 && limex->acceptEodCount && ISNONZERO_STATE(ctx.s)) {
        const union RepeatControl *repeat_ctrl = NULL;
        const char *repeat_state = NULL;
        TESTEOD_FN(limex, &ctx.s, repeat_ctrl, repeat_state, offset, cb,
                   context);
    }

    // NOTE: return value is unused.
    return 0;
}

char JOIN(LIMEX_API_ROOT, _inAccept)(const struct NFA *nfa,
                                        ReportID report, struct mq *q) {
    assert(nfa && q);
    assert(q->state && q->streamState);

    const IMPL_NFA_T *limex = getImplNfa(nfa);
    union RepeatControl *repeat_ctrl =
        getRepeatControlBase(q->state, sizeof(STATE_T));
    char *repeat_state = q->streamState + limex->stateSize;
    STATE_T state = *(STATE_T *)q->state;
    u64a offset = q->offset + q_last_loc(q) + 1;

    return JOIN(limexInAccept, SIZE)(limex, state, repeat_ctrl, repeat_state,
                                     offset, report);
}

char JOIN(LIMEX_API_ROOT, _inAnyAccept)(const struct NFA *nfa, struct mq *q) {
    assert(nfa && q);
    assert(q->state && q->streamState);

    const IMPL_NFA_T *limex = getImplNfa(nfa);
    union RepeatControl *repeat_ctrl =
        getRepeatControlBase(q->state, sizeof(STATE_T));
    char *repeat_state = q->streamState + limex->stateSize;
    STATE_T state = *(STATE_T *)q->state;
    u64a offset = q->offset + q_last_loc(q) + 1;

    return JOIN(limexInAnyAccept, SIZE)(limex, state, repeat_ctrl, repeat_state,
                                        offset);
}

enum nfa_zombie_status JOIN(LIMEX_API_ROOT, _zombie_status)(
                                                         const struct NFA *nfa,
                                                         struct mq *q,
                                                         s64a loc) {
    assert(nfa->flags & NFA_ZOMBIE);
    const IMPL_NFA_T *limex = getImplNfa(nfa);
    STATE_T state = *(STATE_T *)q->state;
    STATE_T zmask = LOAD_FROM_ENG(&limex->zombieMask);

    if (limex->repeatCount) {
        u64a offset = q->offset + loc + 1;
        union RepeatControl *repeat_ctrl =
            getRepeatControlBase(q->state, sizeof(STATE_T));
        char *repeat_state = q->streamState + limex->stateSize;
        SQUASH_UNTUG_BR_FN(limex, repeat_ctrl, repeat_state, offset, &state);
    }

    if (ISNONZERO_STATE(AND_STATE(state, zmask))) {
        return NFA_ZOMBIE_ALWAYS_YES;
    }

    return NFA_ZOMBIE_NO;
}

#undef TESTEOD_FN
#undef INITIAL_FN
#undef TOP_FN
#undef TOPN_FN
#undef REPORTCURRENT_FN
#undef COMPRESS_FN
#undef EXPAND_FN
#undef COMPRESS_REPEATS_FN
#undef EXPAND_REPEATS_FN
#undef PROCESS_ACCEPTS_FN
#undef PROCESS_ACCEPTS_NOSQUASH_FN
#undef GET_NFA_REPEAT_INFO_FN
#undef RUN_ACCEL_FN
#undef RUN_EXCEPTIONS_FN
#undef REV_STREAM_FN
#undef LOOP_NOACCEL_FN
#undef STREAM_FN
#undef STREAMCB_FN
#undef STREAMFIRST_FN
#undef STREAMSILENT_FN
#undef CONTEXT_T
#undef EXCEPTION_T
#undef AND_STATE
#undef ANDNOT_STATE
#undef OR_STATE
#undef LSHIFT_STATE
#undef TESTBIT_STATE
#undef CLEARBIT_STATE
#undef ZERO_STATE
#undef ISNONZERO_STATE
#undef ISZERO_STATE
#undef NOTEQ_STATE
#undef DIFFRICH_STATE
#undef INLINE_ATTR_INT
#undef IMPL_NFA_T
#undef SQUASH_UNTUG_BR_FN
#undef ACCEL_MASK
#undef ACCEL_AND_FRIENDS_MASK
#undef EXCEPTION_MASK
#undef LIMEX_API_ROOT
