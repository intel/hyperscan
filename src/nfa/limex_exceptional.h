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

/** \file
 * \brief LimEx NFA: runtime exception processing code.
 *
 * X-macro generic impl, included into the various LimEx model implementations.
 */

#if !defined(SIZE) || !defined(STATE_T)
#  error Must define SIZE and STATE_T in includer.
#endif

#include "config.h"
#include "limex_ring.h"
#include "util/join.h"
#include "util/uniform_ops.h"

#define PE_FN                   JOIN(processExceptional, SIZE)
#define RUN_EXCEPTION_FN        JOIN(runException, SIZE)
#define ZERO_STATE              JOIN(zero_, STATE_T)
#define LOAD_STATE              JOIN(load_, STATE_T)
#define STORE_STATE             JOIN(store_, STATE_T)
#define AND_STATE               JOIN(and_, STATE_T)
#define EQ_STATE(a, b)          (!JOIN(noteq_, STATE_T)((a), (b)))
#define OR_STATE                JOIN(or_, STATE_T)
#define TESTBIT_STATE           JOIN(testbit_, STATE_T)
#define EXCEPTION_T             JOIN(struct NFAException, SIZE)
#define CONTEXT_T               JOIN(NFAContext, SIZE)
#define IMPL_NFA_T              JOIN(LimExNFA, SIZE)
#define GET_NFA_REPEAT_INFO_FN  JOIN(getNfaRepeatInfo, SIZE)

#ifdef ESTATE_ON_STACK
#define ESTATE_ARG STATE_T estate
#else
#define ESTATE_ARG const STATE_T *estatep
#define estate LOAD_STATE(estatep)
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
#else
#define CHUNK_T u32
#define FIND_AND_CLEAR_FN findAndClearLSB_32
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
                     const ReportID *exReports,
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
            char cyclic_on = TESTBIT_STATE(STATE_ARG_P, info->cyclicState);
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
                STORE_STATE(succ, AND_STATE(LOAD_STATE(succ),
                            LOAD_STATE(&e->squash)));
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
            const ReportID *reports = exReports + e->reports;
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
    *local_succ = OR_STATE(*local_succ, LOAD_STATE(&e->successors));
#else
    STORE_STATE(&ctx->local_succ, OR_STATE(LOAD_STATE(&ctx->local_succ),
                LOAD_STATE(&e->successors)));
#endif

    // Some exceptions squash states behind them. Note that we squash states in
    // 'succ', not local_succ.
    if (e->hasSquash == LIMEX_SQUASH_CYCLIC ||
                e->hasSquash == LIMEX_SQUASH_REPORT) {
        STORE_STATE(succ, AND_STATE(LOAD_STATE(succ),
                    LOAD_STATE(&e->squash)));
        if (*cacheable == CACHE_RESULT) {
            *cacheable = DO_NOT_CACHE_RESULT;
        }
    }

    return 1; // continue
}

#ifndef RUN_EXCEPTION_FN_ONLY

/** \brief Process all of the exceptions associated with the states in the \a estate. */
static really_inline
int PE_FN(STATE_ARG, ESTATE_ARG, u32 diffmask, STATE_T *succ,
          const struct IMPL_NFA_T *limex,
          const u32 *exceptionMap, const EXCEPTION_T *exceptions,
          const ReportID *exReports,
          u64a offset, struct CONTEXT_T *ctx, char in_rev, char flags) {
    assert(diffmask > 0); // guaranteed by caller macro

    if (EQ_STATE(estate, LOAD_STATE(&ctx->cached_estate))) {
        DEBUG_PRINTF("using cached succ from previous state\n");
        STORE_STATE(succ, OR_STATE(LOAD_STATE(succ), LOAD_STATE(&ctx->cached_esucc)));
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
    STORE_STATE(&ctx->local_succ, ZERO_STATE);
#endif

    // A copy of the estate as an array of GPR-sized chunks.
    CHUNK_T chunks[sizeof(STATE_T) / sizeof(CHUNK_T)];
#ifdef ESTATE_ON_STACK
    memcpy(chunks, &estate, sizeof(STATE_T));
#else
    memcpy(chunks, estatep, sizeof(STATE_T));
#endif

    struct proto_cache new_cache = {0, NULL};
    enum CacheResult cacheable = CACHE_RESULT;

    do {
        u32 t = findAndClearLSB_32(&diffmask);
#ifdef ARCH_64_BIT
        t >>= 1; // Due to diffmask64, which leaves holes in the bitmask.
#endif
        assert(t < ARRAY_LENGTH(chunks));
        CHUNK_T word = chunks[t];
        assert(word != 0);
        u32 base = t * sizeof(CHUNK_T) * 8;
        do {
            u32 bit = FIND_AND_CLEAR_FN(&word) + base;
            u32 idx = exceptionMap[bit];
            const EXCEPTION_T *e = &exceptions[idx];

            if (!RUN_EXCEPTION_FN(e, STATE_ARG_NAME, succ,
#ifndef BIG_MODEL
                                  &local_succ,
#endif
                                  limex, exReports, offset, ctx, &new_cache,
                                  &cacheable, in_rev, flags)) {
                return PE_RV_HALT;
            }
        } while (word);
    } while (diffmask);

#ifndef BIG_MODEL
    STORE_STATE(succ, OR_STATE(LOAD_STATE(succ), local_succ));
#else
    STORE_STATE(succ, OR_STATE(LOAD_STATE(succ), ctx->local_succ));
#endif

    if (cacheable == CACHE_RESULT) {
        STORE_STATE(&ctx->cached_estate, estate);
#ifndef BIG_MODEL
        ctx->cached_esucc = local_succ;
#else
        STORE_STATE(&ctx->cached_esucc, LOAD_STATE(&ctx->local_succ));
#endif
        ctx->cached_reports = new_cache.reports;
        ctx->cached_br = new_cache.br;
    } else if (cacheable == DO_NOT_CACHE_RESULT_AND_FLUSH_BR_ENTRIES) {
        if (ctx->cached_br) {
            STORE_STATE(&ctx->cached_estate, ZERO_STATE);
        }
    }

    return 0;
}

#endif

#undef ZERO_STATE
#undef AND_STATE
#undef EQ_STATE
#undef OR_STATE
#undef TESTBIT_STATE
#undef LOAD_STATE
#undef STORE_STATE
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

#undef CHUNK_T
#undef FIND_AND_CLEAR_FN
#undef IMPL_NFA_T
#undef GET_NFA_REPEAT_INFO_FN

// Parameters.
#undef SIZE
#undef STATE_T
