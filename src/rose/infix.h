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

#ifndef INFIX_H
#define INFIX_H

#include "ue2common.h"
#include "nfa/nfa_api.h"
#include "nfa/nfa_api_queue.h"
#include "nfa/nfa_internal.h"

static really_inline
int infixTooOld(struct mq *q, s64a curr_loc) {
    u32 maxAge = q->nfa->maxWidth;

    if (!maxAge) {
        return 0;
    }

    return q_last_loc(q) + maxAge < curr_loc;
}

static really_inline
int canReduceQueue(const struct mq *q, s64a curr_loc, u32 maxTops, u32 maxAge) {
    u32 qlen = q->end - q->cur; /* includes MQE_START */

    if (maxAge && q->items[q->cur].location + maxAge < curr_loc) {
        return 1;
    }

    if (qlen - 1 > maxTops) {
        return 1;
    }

    if (qlen - 1 == maxTops
        && q->items[q->cur].location != q->items[q->cur + 1].location) {
        /* we can advance start to the first top location */
        return 1;
    }

    return 0;
}

/**
 * Removes tops which are known not to affect the final state from the queue.
 * May also reinitialise the engine state if it is unneeded.
 *
 * maxAge is the maximum width of the infix. Any tops/state before this can be
 * ignored. 0 is used to indicate that there is no upper bound on the width of
 * the pattern.
 *
 * maxTops is the maximum number of locations of tops that can affect the top.
 * It is only possible for the last maxTops tops to affect the final state -
 * earlier ones can be safely removed. Also, any state before the max tops may
 * be ignored.
 *
 * This code assumes/requires that there are not multiple tops at the same
 * location in the queue. This code also assumes that it is not a multitop
 * engine.
 */
static really_inline
void reduceInfixQueue(struct mq *q, s64a curr_loc, u32 maxTops, u32 maxAge) {
    assert(q->end > q->cur);
    assert(maxTops);
    u32 qlen = q->end - q->cur; /* includes MQE_START */
    DEBUG_PRINTF("q=%p, len=%u, maxTops=%u maxAge=%u\n", q, qlen, maxTops,
                 maxAge);

    if (!canReduceQueue(q, curr_loc, maxTops, maxAge)) {
        DEBUG_PRINTF("nothing to do\n");
        return;
    }

#ifdef DEBUG
    debugQueue(q);
#endif

    char drop_state = qlen - 1 >= maxTops
        || (maxAge && q->items[q->cur].location + maxAge < curr_loc);

    LIMIT_TO_AT_MOST(&maxTops, qlen - 1);

    // We leave our START where it is, at the front of the queue.
    assert(q->items[q->cur].type == MQE_START);

    // We want to shuffle maxQueueLen items from the end of the queue to just
    // after the start, effectively dequeuing old items. We could use memmove
    // for this, but it's probably not a good idea to take the cost of the
    // function call.
    const struct mq_item *src = &q->items[q->cur + qlen - maxTops];

    q->items[0] = q->items[q->cur]; /* shift start event to 0 slot */
    q->cur = 0;
    q->end = 1;
    struct mq_item *dst = &q->items[1];
    u32 i = 0;
    if (maxAge) {
        /* any event which is older than maxAge can be dropped */
        for (; i < maxTops; i++, src++) {
            if (src->location >= curr_loc - maxAge) {
                break;
            }
        }
    }

    for (; i < maxTops; i++) {
        *dst = *src;
        src++;
        dst++;
        q->end++;
    }

    if (drop_state) {
        /* clear state and shift start up to first top */
        s64a new_loc;
        if (q->end > 1) {
            new_loc = q->items[1].location;
        } else {
            DEBUG_PRINTF("no tops\n");
            new_loc = curr_loc;
        }

        DEBUG_PRINTF("advancing start from %lld to %lld\n",
                     q->items[0].location, new_loc);
        assert(new_loc > q->items[0].location);
        q->items[0].location = new_loc;
        nfaQueueInitState(q->nfa, q);
    }

    DEBUG_PRINTF("reduced queue to len=%u\n", q->end - q->cur);
#ifdef DEBUG
    debugQueue(q);
#endif
}

#endif
