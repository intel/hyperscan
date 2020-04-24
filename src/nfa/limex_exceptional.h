/*
 * Copyright (c) 2015-2020, Intel Corporation
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
 * \brief LimEx NFA: runtime exception processing code.
 *
 * X-macro generic impl, included into the various LimEx model implementations.
 */

#if !defined(SIZE) || !defined(STATE_T) || !defined(LOAD_FROM_ENG)
#  error Must define SIZE, STATE_T, LOAD_FROM_ENG in includer.
#endif

#include "config.h"
#include "limex_ring.h"
#include "util/join.h"
#include "util/uniform_ops.h"

#define PE_FN                   JOIN(processExceptional, SIZE)
#define RUN_EXCEPTION_FN        JOIN(runException, SIZE)
#define ZERO_STATE              JOIN(zero_, STATE_T)
#define AND_STATE               JOIN(and_, STATE_T)
#define EQ_STATE(a, b)          (!JOIN(noteq_, STATE_T)((a), (b)))
#define OR_STATE                JOIN(or_, STATE_T)
#define EXPAND_STATE            JOIN(expand_, STATE_T)
#define SHUFFLE_BYTE_STATE      JOIN(shuffle_byte_, STATE_T)
#define TESTBIT_STATE           JOIN(testbit_, STATE_T)
#define EXCEPTION_T             JOIN(struct NFAException, SIZE)
#define CONTEXT_T               JOIN(NFAContext, SIZE)
#define IMPL_NFA_T              JOIN(LimExNFA, SIZE)
#define GET_NFA_REPEAT_INFO_FN  JOIN(getNfaRepeatInfo, SIZE)

#ifdef ESTATE_ON_STACK
#define ESTATE_ARG STATE_T estate
#else
#define ESTATE_ARG const STATE_T *estatep
#define estate (*estatep)
#endif

#ifdef STATE_ON_STACK
#define STATE_ARG_NAME s
#define STATE_ARG STATE_T STATE_ARG_NAME
#define STATE_ARG_P &s
#else
#define STATE_ARG_NAME sp
#define STATE_ARG const STATE_T *STATE_ARG_NAME
#define STATE_ARG_P sp
#endif

#ifndef STATE_ON_STACK
#define BIG_MODEL
#endif

#ifdef ARCH_64_BIT
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

/** \brief Process a single exception. Returns 1 if exception handling should
 * continue, 0 if an accept callback has instructed us to halt. */
static really_inline
int RUN_EXCEPTION_FN(const EXCEPTION_T *e, STATE_ARG,
                     STATE_T *succ,
#ifndef BIG_MODEL
                     STATE_T *local_succ,
#endif
                     const struct IMPL_NFA_T *limex,
                     u64a offset,
                     struct CONTEXT_T *ctx,
                     struct proto_cache *new_cache,
                     enum CacheResult *cacheable,
                     char in_rev,
                     const char flags) {
    assert(e);

#ifdef DEBUG_EXCEPTIONS
    printf("EXCEPTION e=%p reports=%u trigger=", e, e->reports);
    if (e->trigger == LIMEX_TRIGGER_NONE) {
        printf("none");
    } else if (e->trigger == LIMEX_TRIGGER_POS) {
        printf("pos");
    } else if (e->trigger == LIMEX_TRIGGER_TUG) {
        printf("tug");
    } else {
        printf("unknown!");
    }
    printf("\n");
#endif

    // Trigger exceptions, used in bounded repeats.
    assert(!in_rev || e->trigger == LIMEX_TRIGGER_NONE);
    if (!in_rev && e->trigger != LIMEX_TRIGGER_NONE) {
        assert(e->repeatOffset != MO_INVALID_IDX);
        const struct NFARepeatInfo *info =
            (const struct NFARepeatInfo *)((const char *)limex +
                                           e->repeatOffset);
        const struct RepeatInfo *repeat = getRepeatInfo(info);
        assert(ctx->repeat_ctrl && ctx->repeat_state);
        union RepeatControl *repeat_ctrl = ctx->repeat_ctrl + info->ctrlIndex;
        char *repeat_state = ctx->repeat_state + info->stateOffset;

        if (e->trigger == LIMEX_TRIGGER_POS) {
            char cyclic_on = TESTBIT_STATE(*STATE_ARG_P, info->cyclicState);
            processPosTrigger(repeat, repeat_ctrl, repeat_state, offset,
                              cyclic_on);
            *cacheable = DO_NOT_CACHE_RESULT_AND_FLUSH_BR_ENTRIES;
        } else {
            assert(e->trigger == LIMEX_TRIGGER_TUG);
            enum TriggerResult rv =
                processTugTrigger(repeat, repeat_ctrl, repeat_state, offset);
            if (rv == TRIGGER_FAIL) {
                *cacheable = DO_NOT_CACHE_RESULT_AND_FLUSH_BR_ENTRIES;
                DEBUG_PRINTF("tug found no valid matches in repeat state\n");
                return 1; // continue
            } else if (rv == TRIGGER_STALE) {
                *cacheable = DO_NOT_CACHE_RESULT_AND_FLUSH_BR_ENTRIES;
                DEBUG_PRINTF("stale history, squashing cyclic state\n");
                assert(e->hasSquash == LIMEX_SQUASH_TUG);
                *succ = AND_STATE(*succ, LOAD_FROM_ENG(&e->squash));
                return 1; // continue
            } else if (rv == TRIGGER_SUCCESS_CACHE) {
                new_cache->br = 1;
            } else {
                assert(rv == TRIGGER_SUCCESS);
                *cacheable = DO_NOT_CACHE_RESULT_AND_FLUSH_BR_ENTRIES;
            }
        }
    }

    // Some exceptions fire accepts.
    if (e->reports != MO_INVALID_IDX) {
        if (flags & CALLBACK_OUTPUT) {
            const ReportID *reports =
                (const ReportID *)((const char *)limex + e->reports);
            if (unlikely(limexRunReports(reports, ctx->callback,
                            ctx->context, offset)
                        == MO_HALT_MATCHING)) {
                DEBUG_PRINTF("callback instructed us to stop\n");
                return 0; // halt
            }
            if (*cacheable == CACHE_RESULT) {
                if (!new_cache->reports || new_cache->reports == reports) {
                    new_cache->reports = reports;
                } else {
                    *cacheable = DO_NOT_CACHE_RESULT;
                }
            }
        } else {
            if ((flags & FIRST_BYTE) && *cacheable == CACHE_RESULT) {
                *cacheable = DO_NOT_CACHE_RESULT;
            } /* otherwise we can cache as we never care about accepts */
        }
    }

    // Most exceptions have a set of successors to switch on. `local_succ' is
    // ORed into `succ' at the end of the caller's loop.
#ifndef BIG_MODEL
    *local_succ = OR_STATE(*local_succ, LOAD_FROM_ENG(&e->successors));
#else
    ctx->local_succ = OR_STATE(ctx->local_succ, LOAD_FROM_ENG(&e->successors));
#endif

    // Some exceptions squash states behind them. Note that we squash states in
    // 'succ', not local_succ.
    if (e->hasSquash == LIMEX_SQUASH_CYCLIC
        || e->hasSquash == LIMEX_SQUASH_REPORT) {
        *succ = AND_STATE(*succ, LOAD_FROM_ENG(&e->squash));
        if (*cacheable == CACHE_RESULT) {
            *cacheable = DO_NOT_CACHE_RESULT;
        }
    }

    return 1; // continue
}

#ifndef RUN_EXCEPTION_FN_ONLY

/** \brief Process all of the exceptions associated with the states in the \a
 * estate. */
static really_inline
int PE_FN(STATE_ARG, ESTATE_ARG, UNUSED u32 diffmask, STATE_T *succ,
          const struct IMPL_NFA_T *limex, const EXCEPTION_T *exceptions,
          u64a offset, struct CONTEXT_T *ctx, char in_rev, char flags) {
    assert(diffmask > 0); // guaranteed by caller macro

    if (EQ_STATE(estate, ctx->cached_estate)) {
        DEBUG_PRINTF("using cached succ from previous state\n");
        *succ = OR_STATE(*succ, ctx->cached_esucc);
        if (ctx->cached_reports && (flags & CALLBACK_OUTPUT)) {
            DEBUG_PRINTF("firing cached reports from previous state\n");
            if (unlikely(limexRunReports(ctx->cached_reports, ctx->callback,
                                         ctx->context, offset)
                        == MO_HALT_MATCHING)) {
                return PE_RV_HALT; // halt;
            }
        }
        return 0;
    }

#ifndef BIG_MODEL
    STATE_T local_succ = ZERO_STATE;
#else
    ctx->local_succ = ZERO_STATE;
#endif

    struct proto_cache new_cache = {0, NULL};
    enum CacheResult cacheable = CACHE_RESULT;

#if defined(HAVE_AVX512VBMI) && SIZE > 64
    if (likely(limex->flags & LIMEX_FLAG_EXTRACT_EXP)) {
        m512 emask = EXPAND_STATE(*STATE_ARG_P);
        emask = SHUFFLE_BYTE_STATE(load_m512(&limex->exceptionShufMask), emask);
        emask = and512(emask, load_m512(&limex->exceptionAndMask));
        u64a word = eq512mask(emask, load_m512(&limex->exceptionBitMask));

        do {
            u32 bit = FIND_AND_CLEAR_FN(&word);
            const EXCEPTION_T *e = &exceptions[bit];

            if (!RUN_EXCEPTION_FN(e, STATE_ARG_NAME, succ,
#ifndef BIG_MODEL
                                  &local_succ,
#endif
                                  limex, offset, ctx, &new_cache, &cacheable,
                                  in_rev, flags)) {
                return PE_RV_HALT;
            }
        } while (word);
    } else {
        // A copy of the estate as an array of GPR-sized chunks.
        CHUNK_T chunks[sizeof(STATE_T) / sizeof(CHUNK_T)];
        CHUNK_T emask_chunks[sizeof(STATE_T) / sizeof(CHUNK_T)];
#ifdef ESTATE_ON_STACK
        memcpy(chunks, &estate, sizeof(STATE_T));
#else
        memcpy(chunks, estatep, sizeof(STATE_T));
#endif
        memcpy(emask_chunks, &limex->exceptionMask, sizeof(STATE_T));

        u32 base_index[sizeof(STATE_T) / sizeof(CHUNK_T)];
        base_index[0] = 0;
        for (s32 i = 0; i < (s32)ARRAY_LENGTH(base_index) - 1; i++) {
            base_index[i + 1] = base_index[i] + POPCOUNT_FN(emask_chunks[i]);
        }

        do {
            u32 t = findAndClearLSB_32(&diffmask);
#ifdef ARCH_64_BIT
            t >>= 1; // Due to diffmask64, which leaves holes in the bitmask.
#endif
            assert(t < ARRAY_LENGTH(chunks));
            CHUNK_T word = chunks[t];
            assert(word != 0);
            do {
                u32 bit = FIND_AND_CLEAR_FN(&word);
                u32 local_index = RANK_IN_MASK_FN(emask_chunks[t], bit);
                u32 idx = local_index + base_index[t];
                const EXCEPTION_T *e = &exceptions[idx];

                if (!RUN_EXCEPTION_FN(e, STATE_ARG_NAME, succ,
#ifndef BIG_MODEL
                                      &local_succ,
#endif
                                      limex, offset, ctx, &new_cache, &cacheable,
                                      in_rev, flags)) {
                    return PE_RV_HALT;
                }
            } while (word);
        } while (diffmask);
    }
#else
    // A copy of the estate as an array of GPR-sized chunks.
    CHUNK_T chunks[sizeof(STATE_T) / sizeof(CHUNK_T)];
    CHUNK_T emask_chunks[sizeof(STATE_T) / sizeof(CHUNK_T)];
#ifdef ESTATE_ON_STACK
    memcpy(chunks, &estate, sizeof(STATE_T));
#else
    memcpy(chunks, estatep, sizeof(STATE_T));
#endif
    memcpy(emask_chunks, &limex->exceptionMask, sizeof(STATE_T));

    u32 base_index[sizeof(STATE_T) / sizeof(CHUNK_T)];
    base_index[0] = 0;
    for (s32 i = 0; i < (s32)ARRAY_LENGTH(base_index) - 1; i++) {
        base_index[i + 1] = base_index[i] + POPCOUNT_FN(emask_chunks[i]);
    }

    do {
        u32 t = findAndClearLSB_32(&diffmask);
#ifdef ARCH_64_BIT
        t >>= 1; // Due to diffmask64, which leaves holes in the bitmask.
#endif
        assert(t < ARRAY_LENGTH(chunks));
        CHUNK_T word = chunks[t];
        assert(word != 0);
        do {
            u32 bit = FIND_AND_CLEAR_FN(&word);
            u32 local_index = RANK_IN_MASK_FN(emask_chunks[t], bit);
            u32 idx = local_index + base_index[t];
            const EXCEPTION_T *e = &exceptions[idx];

            if (!RUN_EXCEPTION_FN(e, STATE_ARG_NAME, succ,
#ifndef BIG_MODEL
                                  &local_succ,
#endif
                                  limex, offset, ctx, &new_cache, &cacheable,
                                  in_rev, flags)) {
                return PE_RV_HALT;
            }
        } while (word);
    } while (diffmask);
#endif

#ifndef BIG_MODEL
    *succ = OR_STATE(*succ, local_succ);
#else
    *succ = OR_STATE(*succ, ctx->local_succ);
#endif

    if (cacheable == CACHE_RESULT) {
        ctx->cached_estate = estate;
#ifndef BIG_MODEL
        ctx->cached_esucc = local_succ;
#else
        ctx->cached_esucc = ctx->local_succ;
#endif
        ctx->cached_reports = new_cache.reports;
        ctx->cached_br = new_cache.br;
    } else if (cacheable == DO_NOT_CACHE_RESULT_AND_FLUSH_BR_ENTRIES) {
        if (ctx->cached_br) {
            ctx->cached_estate = ZERO_STATE;
        }
    }

    return 0;
}

#endif

#undef ZERO_STATE
#undef AND_STATE
#undef EQ_STATE
#undef OR_STATE
#undef EXPAND_STATE
#undef SHUFFLE_BYTE_STATE
#undef TESTBIT_STATE
#undef PE_FN
#undef RUN_EXCEPTION_FN
#undef CONTEXT_T
#undef EXCEPTION_T

#ifdef estate
#undef estate
#endif

#ifdef BIG_MODEL
#undef BIG_MODEL
#endif

#undef STATE_ARG
#undef STATE_ARG_NAME
#undef STATE_ARG_P

#undef IMPL_NFA_T

#undef CHUNK_T
#undef FIND_AND_CLEAR_FN
#undef POPCOUNT_FN
#undef RANK_IN_MASK_FN
