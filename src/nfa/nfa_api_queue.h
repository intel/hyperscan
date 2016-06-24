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

#ifndef NFA_API_QUEUE_H
#define NFA_API_QUEUE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "ue2common.h"
#include "callback.h"

/** Size of mq::items, max elements on a queue. */
#define MAX_MQE_LEN 10

/** Queue events */

/** Queue event: begin scanning. Note: stateless engines will start from this
 * location. */
#define MQE_START 0U

/** Queue event: stop scanning. */
#define MQE_END 1U

/** Queue event: enable start and start-dot-star. */
#define MQE_TOP 2U

/** Queue event: first event corresponding to a numbered TOP. Additional tops
 * (in multi-top engines) use the event values from MQE_TOP_FIRST to
 * MQE_INVALID - 1. */
#define MQE_TOP_FIRST 4U

/** Invalid queue event */
#define MQE_INVALID (~0U)

/** Queue item */
struct mq_item {
    u32 type; /**< event type, from MQE_* */
    s64a location; /**< relative to the start of the current buffer */
    u64a som; /**< pattern start-of-match corresponding to a top, only used
               * by som engines. */
};

// Forward decl.
struct NFA;

/**
 * Queue of events to control engine execution.  mq::cur is index of first
 * valid event, mq::end is one past the index of last valid event.
 */
struct mq {
    const struct NFA *nfa; /**< nfa corresponding to the queue */
    u32 cur; /**< index of the first valid item in the queue */
    u32 end; /**< index one past the last valid item in the queue */
    char *state; /**< uncompressed stream state; lives in scratch */
    char *streamState; /**<
                        * real stream state; used to access structures which
                        * not duplicated the scratch state (bounded repeats,
                        * etc) */
    u64a offset; /**< base offset of the buffer */
    const u8 *buffer; /**< buffer to scan */
    size_t length; /**< length of buffer */
    const u8 *history; /**<
                        * history buffer; (logically) immediately before the
                        * main buffer */
    size_t hlength; /**< length of the history buffer */
    struct hs_scratch *scratch; /**< global scratch space */
    char report_current; /**<
                          * report_current matches at starting offset through
                          * callback. If true, the queue must be located at a
                          * point where MO_MATCHES_PENDING was returned */
    NfaCallback cb; /**< callback to trigger on matches */
    void *context; /**< context to pass along with a callback */
    struct mq_item items[MAX_MQE_LEN]; /**< queue items */
};


/**
 * Pushes an (event, location, som) item onto a queue. If it is identical to the
 * previous item on the queue, it is not added to the queue.
 * @param q queue
 * @param e event
 * @param som som marker
 * @param loc event location
 */
static really_inline
void pushQueueSom(struct mq * restrict q, u32 e, s64a loc, u64a som) {
    DEBUG_PRINTF("pushing %u@%lld -> %u [som = %llu]\n", e, loc, q->end, som);
    assert(q->end < MAX_MQE_LEN);
    assert(e < MQE_INVALID);
/* stop gcc getting too smart for its own good */
/*     assert(!q->end || q->items[q->end - 1].location <= loc); */
    assert(q->end || e == MQE_START);

    // Avoid duplicate items on the queue.
    if (q->end) {
        struct mq_item *item = &q->items[q->end - 1];
        if (item->type == e && item->location == loc) {
            DEBUG_PRINTF("dropping duplicate item\n");
            LIMIT_TO_AT_MOST(&item->som, som); /* take lower som */
            return;
        }
    }

    u32 end = q->end;
    struct mq_item *item = &q->items[end];
    item->type = e;
    item->location = loc;
    item->som = som;
    q->end = end + 1;
}

/**
 * Pushes an (event, location) item onto a queue. If it is identical to the
 * previous item on the queue, it is not added to the queue.
 * @param q queue
 * @param e event
 * @param loc event location
 */
static really_inline
void pushQueue(struct mq * restrict q, u32 e, s64a loc) {
    pushQueueSom(q, e, loc, 0);
}

/**
 * Pushes an (event, location) item onto a queue.
 * This version of @ref pushQueue does not check to ensure that the item being
 * added is not already on the queue. Used for events other than tops.
 */
static really_inline
void pushQueueNoMerge(struct mq * restrict q, u32 e, s64a loc) {
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

    u32 end = q->end;
    struct mq_item *item = &q->items[end];
    item->type = e;
    item->location = loc;
    item->som = 0;
    q->end = end + 1;
}

/** \brief Returns the type of the current queue event. */
static really_inline u32 q_cur_type(const struct mq *q) {
    assert(q->cur < q->end);
    assert(q->cur < MAX_MQE_LEN);
    return q->items[q->cur].type;
}

/** \brief Returns the location (relative to the beginning of the current data
 * buffer) of the current queue event. */
static really_inline s64a q_cur_loc(const struct mq *q) {
    assert(q->cur < q->end);
    assert(q->cur < MAX_MQE_LEN);
    return q->items[q->cur].location;
}

/** \brief Returns the type of the last event in the queue. */
static really_inline u32 q_last_type(const struct mq *q) {
    assert(q->cur < q->end);
    assert(q->end > 0);
    assert(q->end <= MAX_MQE_LEN);
    return q->items[q->end - 1].type;
}

/** \brief Returns the location (relative to the beginning of the current data
 * buffer) of the last event in the queue. */
static really_inline s64a q_last_loc(const struct mq *q) {
    assert(q->cur < q->end);
    assert(q->end > 0);
    assert(q->end <= MAX_MQE_LEN);
    return q->items[q->end - 1].location;
}

/** \brief Returns the absolute stream offset of the current queue event. */
static really_inline u64a q_cur_offset(const struct mq *q) {
    assert(q->cur < q->end);
    assert(q->cur < MAX_MQE_LEN);
    return q->offset + (u64a)q->items[q->cur].location;
}

/**
 * \brief Removes all events in the queue before the given location.
 */
static really_inline
void q_skip_forward_to(struct mq *q, s64a min_loc) {
    assert(q->cur < q->end);
    assert(q->cur < MAX_MQE_LEN);
    assert(q->items[q->cur].type == MQE_START);

    if (q_cur_loc(q) >= min_loc) {
        DEBUG_PRINTF("all events >= loc %lld\n", min_loc);
        return;
    }

    const u32 start_loc = q->cur;

    do {
        DEBUG_PRINTF("remove item with loc=%lld\n", q_cur_loc(q));
        q->cur++;
    } while (q->cur < q->end && q_cur_loc(q) < min_loc);

    if (q->cur > start_loc) {
        // Move original MQE_START item forward.
        q->cur--;
        q->items[q->cur] = q->items[start_loc];
    }
}

#ifdef DEBUG
// Dump the contents of the given queue.
static never_inline UNUSED
void debugQueue(const struct mq *q) {
    DEBUG_PRINTF("q=%p, nfa=%p\n", q, q->nfa);
    DEBUG_PRINTF("q offset=%llu, buf={%p, len=%zu}, history={%p, len=%zu}\n",
                 q->offset, q->buffer, q->length, q->history, q->hlength);
    DEBUG_PRINTF("q cur=%u, end=%u\n", q->cur, q->end);
    for (u32 cur = q->cur; cur < q->end; cur++) {
        const char *type = "UNKNOWN";
        u32 e = q->items[cur].type;
        switch (e) {
        case MQE_START:
            type = "MQE_START";
            break;
        case MQE_END:
            type = "MQE_END";
            break;
        case MQE_TOP:
            type = "MQE_TOP";
            break;
        case MQE_INVALID:
            type = "MQE_INVALID";
            break;
        default:
            assert(e >= MQE_TOP_FIRST && e < MQE_INVALID);
            type = "MQE_TOP_N";
            break;
        }
        DEBUG_PRINTF("\tq[%u] %lld %u:%s\n", cur, q->items[cur].location,
                     q->items[cur].type, type);
    }
}
#endif // DEBUG

#ifdef __cplusplus
}
#endif

#endif
