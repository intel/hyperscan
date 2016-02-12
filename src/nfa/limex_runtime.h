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
    \brief Limex Execution Engine Or:
    How I Learned To Stop Worrying And Love The Preprocessor

    This file includes utility functions which do not depend on the state size or
    shift masks directly.
*/

#ifndef LIMEX_RUNTIME_H
#define LIMEX_RUNTIME_H

#include "limex_accel.h"
#include "limex_context.h"
#include "limex_internal.h"
#include "nfa_api_util.h"
#include "nfa_internal.h"
#include "util/uniform_ops.h"

////////////////////////////////////////////////////////////////////////////
// LimEx NFA implementation code - common macros
////////////////////////////////////////////////////////////////////////////

#ifdef DEBUG_INPUT
#include <ctype.h>
#define DUMP_INPUT(index) DEBUG_PRINTF("input %p i=%zu: %02hhx (%c)\n", \
                            &input[index], index, input[index],         \
                            isprint(input[index]) ? input[index] : ' ')
#else
#define DUMP_INPUT(index) do { } while(0)
#endif

#define NO_OUTPUT       0
#define CALLBACK_OUTPUT 1
#define FIRST_BYTE      16

enum CacheResult {
    DO_NOT_CACHE_RESULT,
    CACHE_RESULT,
    DO_NOT_CACHE_RESULT_AND_FLUSH_BR_ENTRIES
};

struct proto_cache {
    char br;
    const ReportID *reports;
};

// Shift macros for Limited NFAs. Defined in terms of uniform ops.
#define NFA_EXEC_LIM_SHIFT(nels_type, nels_i)                                  \
    (JOIN(shift_, nels_type)(                                                  \
        JOIN(and_, nels_type)(s,                                               \
                              JOIN(load_, nels_type)(&limex->shift[nels_i])),  \
        nels_i))

// Calculate the (limited model) successors for a given max shift. Assumes
// LimExNFAxxx ptr in 'l', current state in 's' and successors in 'succ'.

#define NFA_EXEC_GET_LIM_SUCC(gls_type, gls_shift)                             \
    do {                                                                       \
        succ =                                                                 \
            JOIN(and_, gls_type)(s, JOIN(load_, gls_type)(&limex->shift[0]));  \
        switch (gls_shift) {                                                   \
        case 7:                                                                \
            succ = JOIN(or_, gls_type)(succ, NFA_EXEC_LIM_SHIFT(gls_type, 7)); \
        case 6:                                                                \
            succ = JOIN(or_, gls_type)(succ, NFA_EXEC_LIM_SHIFT(gls_type, 6)); \
        case 5:                                                                \
            succ = JOIN(or_, gls_type)(succ, NFA_EXEC_LIM_SHIFT(gls_type, 5)); \
        case 4:                                                                \
            succ = JOIN(or_, gls_type)(succ, NFA_EXEC_LIM_SHIFT(gls_type, 4)); \
        case 3:                                                                \
            succ = JOIN(or_, gls_type)(succ, NFA_EXEC_LIM_SHIFT(gls_type, 3)); \
        case 2:                                                                \
            succ = JOIN(or_, gls_type)(succ, NFA_EXEC_LIM_SHIFT(gls_type, 2)); \
        case 1:                                                                \
            succ = JOIN(or_, gls_type)(succ, NFA_EXEC_LIM_SHIFT(gls_type, 1)); \
        case 0:                                                                \
            ;                                                                  \
        }                                                                      \
    } while (0)

#define PE_RV_HALT 1

#ifdef STATE_ON_STACK
#define pass_state s
#else
#define pass_state &s
#endif

#ifdef ESTATE_ON_STACK
#define pass_estate estate
#else
#define pass_estate &estate
#endif

static really_inline
int limexRunReports(const ReportID *reports, NfaCallback callback,
                    void *context, u64a offset) {
    assert(reports);
    assert(callback);

    for (; *reports != MO_INVALID_IDX; ++reports) {
        DEBUG_PRINTF("firing report for id %u at offset %llu\n",
                     *reports, offset);
        int rv = callback(offset, *reports, context);
        if (rv == MO_HALT_MATCHING) {
            return MO_HALT_MATCHING;
        }
    }
    return MO_CONTINUE_MATCHING; // continue
}

/** \brief Return a (correctly typed) pointer to the exception table. */
#define getExceptionTable(exc_type, lim)                                       \
    ((const exc_type *)((const char *)(lim) + (lim)->exceptionOffset))

/** \brief Return a pointer to the exceptional reports list. */
#define getExReports(lim)                                                      \
    ((const ReportID *)((const char *)(lim) + (lim)->exReportOffset))

/** \brief Return a pointer to the ordinary accepts table. */
#define getAcceptTable(lim)                                                    \
    ((const struct NFAAccept *)((const char *)(lim) + (lim)->acceptOffset))

/** \brief Return a pointer to the EOD accepts table. */
#define getAcceptEodTable(lim)                                                 \
    ((const struct NFAAccept *)((const char *)(lim) + (lim)->acceptEodOffset))

#define MAKE_GET_NFA_REPEAT_INFO(size)                                         \
    static really_inline const struct NFARepeatInfo *getNfaRepeatInfo##size(   \
        const struct LimExNFA##size *limex, unsigned num) {                    \
        assert(num < limex->repeatCount);                                      \
                                                                               \
        const char *base = (const char *)limex;                                \
        const u32 *repeatOffset = (const u32 *)(base + limex->repeatOffset);   \
        assert(ISALIGNED(repeatOffset));                                       \
                                                                               \
        const struct NFARepeatInfo *info =                                     \
            (const struct NFARepeatInfo *)(base + repeatOffset[num]);          \
        assert(ISALIGNED(info));                                               \
        return info;                                                           \
    }

MAKE_GET_NFA_REPEAT_INFO(32)
MAKE_GET_NFA_REPEAT_INFO(128)
MAKE_GET_NFA_REPEAT_INFO(256)
MAKE_GET_NFA_REPEAT_INFO(384)
MAKE_GET_NFA_REPEAT_INFO(512)

static really_inline
const struct RepeatInfo *getRepeatInfo(const struct NFARepeatInfo *info) {
    const struct RepeatInfo *repeat =
        (const struct RepeatInfo *)((const char *)info + sizeof(*info));
    assert(ISALIGNED(repeat));
    return repeat;
}

static really_inline
union RepeatControl *getRepeatControlBase(char *state, size_t nfa_state_size) {
    union RepeatControl *ctrl_base =
        (union RepeatControl *)(state +
                                ROUNDUP_N(nfa_state_size,
                                          alignof(union RepeatControl)));
    assert(ISALIGNED(ctrl_base));
    return ctrl_base;
}

static really_inline
const union RepeatControl *getRepeatControlBaseConst(const char *state,
                                                     size_t nfa_state_size) {
    const union RepeatControl *ctrl_base =
        (const union RepeatControl *)(state +
                                      ROUNDUP_N(nfa_state_size,
                                                alignof(union RepeatControl)));
    assert(ISALIGNED(ctrl_base));
    return ctrl_base;
}

#endif
