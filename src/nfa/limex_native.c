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
 * \brief LimEx NFA: native GPR runtime implementations.
 */

//#define DEBUG
//#define DEBUG_INPUT
//#define DEBUG_EXCEPTIONS

#include "limex.h"

#include "accel.h"
#include "limex_internal.h"
#include "nfa_internal.h"
#include "ue2common.h"
#include "util/bitutils.h"

// Common code
#define STATE_ON_STACK
#define ESTATE_ON_STACK

#include "limex_runtime.h"

// Other implementation code from X-Macro impl.
#define SIZE          32
#define STATE_T       u32
#define ENG_STATE_T   u32
#define LOAD_FROM_ENG load_u32

#include "limex_state_impl.h"

#define INLINE_ATTR really_inline
#include "limex_common_impl.h"

////////////////////////////////////////////////////////////////////////////
// LimEx NFA implementation code - general purpose registers
////////////////////////////////////////////////////////////////////////////

// Process exceptional states

#define STATE_ON_STACK
#define ESTATE_ON_STACK
#define RUN_EXCEPTION_FN_ONLY
#include "limex_exceptional.h"

static really_inline
int processExceptional32(u32 s, u32 estate, UNUSED u32 diffmask, u32 *succ,
                         const struct LimExNFA32 *limex,
                         const struct NFAException32 *exceptions, u64a offset,
                         struct NFAContext32 *ctx, char in_rev, char flags) {
    assert(estate != 0); // guaranteed by calling macro

    if (estate == ctx->cached_estate) {
        DEBUG_PRINTF("using cached succ from previous state\n");
        *succ |= ctx->cached_esucc;
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

    u32 orig_estate = estate; // for caching
    u32 local_succ = 0;
    struct proto_cache new_cache = {0, NULL};
    enum CacheResult cacheable = CACHE_RESULT;

    /* Note that only exception-states that consist of exceptions that _only_
     * set successors (not fire accepts or squash states) are cacheable. */

    do {
        u32 bit = findAndClearLSB_32(&estate);
        u32 idx = rank_in_mask32(limex->exceptionMask, bit);
        const struct NFAException32 *e = &exceptions[idx];
        if (!runException32(e, s, succ, &local_succ, limex, offset, ctx,
                            &new_cache, &cacheable, in_rev, flags)) {
            return PE_RV_HALT;
        }
    } while (estate != 0);

    *succ |= local_succ;

    if (cacheable == CACHE_RESULT) {
        ctx->cached_estate = orig_estate;
        ctx->cached_esucc = local_succ;
        ctx->cached_reports = new_cache.reports;
        ctx->cached_br = new_cache.br;
    } else if (cacheable == DO_NOT_CACHE_RESULT_AND_FLUSH_BR_ENTRIES) {
        if (ctx->cached_br) {
            ctx->cached_estate = 0U;
        }
    }

    return 0;
}

// 32-bit models.
#include "limex_runtime_impl.h"
