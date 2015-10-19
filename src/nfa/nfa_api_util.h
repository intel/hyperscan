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

#ifndef NFA_API_UTIL_H
#define NFA_API_UTIL_H

#include "nfa_api_queue.h"
#include "ue2common.h"

/* returns the byte prior to the given location, NUL if not available */
static really_inline
u8 queue_prev_byte(const struct mq *q, s64a loc) {
    if (loc <= 0) {
        if (1LL - loc > (s64a)q->hlength) {
            return 0; /* assume NUL for start of stream write */
        }
        // In the history buffer.
        assert(q->history);
        assert(q->hlength >= (u64a)(loc * -1));
        return q->history[q->hlength - 1 + loc];
    } else {
        // In the stream write buffer.
        assert(q->buffer);
        assert(q->length >= (u64a)loc);
        return q->buffer[loc - 1];
    }
}

/* this is a modified version of pushQueue where we statically know the state of
 * the queue. Does not attempt to merge and inserts at the given queue
 * position. */
static really_inline
void pushQueueAt(struct mq * restrict q, u32 pos, u32 e, s64a loc) {
    assert(pos == q->end);
    DEBUG_PRINTF("pushing %u@%lld -> %u\n", e, loc, q->end);
    assert(q->end < MAX_MQE_LEN);
    assert(e < MQE_INVALID);
/* stop gcc getting too smart for its own good */
/*     assert(!q->end || q->items[q->end - 1].location <= loc); */
    assert(q->end || e == MQE_START);

#ifndef NDEBUG
    // We assert that the event is different from its predecessor. If it's a
    // dupe, you should have used the ordinary pushQueue call.
    if (q->end) {
        UNUSED struct mq_item *prev = &q->items[q->end - 1];
        assert(prev->type != e || prev->location != loc);
    }
#endif

    struct mq_item *item = &q->items[pos];
    item->type = e;
    item->location = loc;
    item->som = 0;
    q->end = pos + 1;
}
#endif
