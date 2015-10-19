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
 * \brief Bounded Repeat implementation for the LimEx NFA.
 */

#ifndef LIMEX_RING_H
#define LIMEX_RING_H

#include "ue2common.h"
#include "repeat.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** \brief Return values from \ref processTugTrigger, used to provide feedback
 * about a bounded repeat to the caller.
 *
 * TRIGGER_FAIL does not get cached as we prefer to use TRIGGER_STALE which
 * allows the exception to squash the cyclic state as well. */
enum TriggerResult {
    TRIGGER_FAIL,    /**< no valid matches, but history still valid */
    TRIGGER_SUCCESS, /**< valid match found */
    TRIGGER_STALE,   /**< no valid matches and history is invalid (stale) */
    TRIGGER_SUCCESS_CACHE /**< valid match found; can cache as the repeat has no
                             upper bound. */
};

/** \brief Handle a TUG trigger: given an \p offset, returns whether a repeat
 * matches or not. */
static really_inline
enum TriggerResult processTugTrigger(const struct RepeatInfo *info,
                                     const union RepeatControl *ctrl,
                                     const char *state, u64a offset) {
    DEBUG_PRINTF("tug trigger, %s history, repeat={%u,%u}, offset=%llu, "
                 "ctrl=%p, state=%p\n",
                 repeatTypeName(info->type), info->repeatMin, info->repeatMax,
                 offset, ctrl, state);

    assert(ISALIGNED(ctrl));

    enum RepeatMatch rv = repeatHasMatch(info, ctrl, state, offset);
    switch (rv) {
    case REPEAT_NOMATCH:
        return TRIGGER_FAIL;
    case REPEAT_STALE:
        return TRIGGER_STALE;
    case REPEAT_MATCH:
        if (info->repeatMax == REPEAT_INF) {
            // {N,} repeats can be cached.
            return TRIGGER_SUCCESS_CACHE;
        } else {
            return TRIGGER_SUCCESS;
        }
    }

    assert(0); // unreachable
    return TRIGGER_FAIL;
}

/** \brief Handle a POS trigger: stores a top in the repeat. */
static really_inline
void processPosTrigger(const struct RepeatInfo *info, union RepeatControl *ctrl,
                       char *state, u64a offset, char is_alive) {
    DEBUG_PRINTF("pos trigger, %s history, repeat={%u,%u}, offset=%llu, "
                 "is_alive=%d\n", repeatTypeName(info->type),
                 info->repeatMin, info->repeatMax, offset, is_alive);

    assert(ISALIGNED(ctrl));

    repeatStore(info, ctrl, state, offset, is_alive);
}

#ifdef __cplusplus
}
#endif

#endif
