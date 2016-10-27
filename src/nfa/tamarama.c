/*
 * Copyright (c) 2016, Intel Corporation
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
    \brief Tamarama: container engine for exclusive engines, runtime code.
*/
#include "config.h"

#include "tamarama.h"

#include "tamarama_internal.h"
#include "nfa_api.h"
#include "nfa_api_queue.h"
#include "nfa_api_util.h"
#include "nfa_internal.h"
#include "scratch.h"
#include "util/partial_store.h"

static really_inline
u32 getSubOffset(const struct Tamarama *t, u32 num) {
    DEBUG_PRINTF("subengine:%u\n", num);
    assert(num < t->numSubEngines);
    const u32 *sub =
        (const u32 *)((const char *)t + sizeof(struct Tamarama) +
                      t->numSubEngines * sizeof(u32));
    assert(ISALIGNED(sub));
    return sub[num];
}

static
const struct NFA *getSubEngine(const struct Tamarama *t,
                               const u32 activeIdx) {
    const u32 offset = getSubOffset(t, activeIdx);
    DEBUG_PRINTF("activeIdx:%u offsets:%u\n", activeIdx, offset);
    const char *base = (const char *)t;
    return (const struct NFA *)(base + offset);
}

static
void storeActiveIdx(const struct Tamarama *t, char *state,
                    const u32 idx) {
    assert(idx <= t->numSubEngines);
    partial_store_u32(state, idx, t->activeIdxSize);
}

static
u32 loadActiveIdx(const char *state,
                  const u32 activeIdxSize) {
    return partial_load_u32(state, activeIdxSize);
}

static really_inline
void copyQueueProperties(const struct mq *q1, struct mq *q2,
                         const u32 activeIdxSize) {
    q2->state = q1->state;
    q2->streamState = q1->streamState + activeIdxSize;
    q2->offset = q1->offset;
    q2->buffer = q1->buffer;
    q2->length = q1->length;
    q2->history = q1->history;
    q2->hlength = q1->hlength;
    q2->cb = q1->cb;
    q2->context = q1->context;
    q2->scratch = q1->scratch;
    q2->report_current = q1->report_current;
}

static
void copyQueueItems(const struct Tamarama *t, const struct NFA *sub,
                    struct mq *q1, struct mq *q2, const u32 activeIdx) {
    const u32 *baseTop = (const u32 *)((const char *)t +
                                       sizeof(struct Tamarama));

    u32 lower = baseTop[activeIdx];
    u32 upper = activeIdx == t->numSubEngines - 1 ?
                    ~0U : baseTop[activeIdx + 1];
    u32 event_base = isMultiTopType(sub->type) ? MQE_TOP_FIRST : MQE_TOP;
    while (q1->cur < q1->end) {
        u32 type = q1->items[q1->cur].type;
        s64a loc = q1->items[q1->cur].location;
        DEBUG_PRINTF("type:%u lower:%u upper:%u\n", type, lower, upper);
        if (type >= lower && type < upper) {
            u32 event = event_base;
            if (event == MQE_TOP_FIRST) {
                event += type - lower;
            }
            pushQueue(q2, event, loc);
        } else {
            pushQueueNoMerge(q2, MQE_END, loc);
            break;
        }
        q1->cur++;
    }
}

static
void copyQueue(const struct Tamarama *t, const struct NFA *sub,
               struct mq *q1, struct mq *q2, const u32 activeIdx) {
    copyQueueProperties(q1, q2, t->activeIdxSize);

    // copy MQE_START item
    u32 cur = q1->cur++;
    q2->cur = cur;
    q2->items[cur] = q1->items[cur];
    q2->end = cur + 1;

    copyQueueItems(t, sub, q1, q2, activeIdx);
    // restore cur index of the main queue
    q1->cur = cur;
}

static
u32 findEngineForTop(const u32 *baseTop, const u32 cur,
                     const u32 numSubEngines) {
    u32 i;
    for (i = 0; i < numSubEngines; ++i) {
        DEBUG_PRINTF("cur:%u base:%u\n", cur, baseTop[i]);
        if (cur >= baseTop[i] &&
            (i == numSubEngines - 1 || cur < baseTop[i + 1])) {
            break;
        }
    }
    return i;
}

static
void initSubQueue(const struct Tamarama *t, struct mq *q1,
                  struct mq *q2, const u32 lastActiveIdx,
                  const u32 activeIdx) {
    // Push events to the new queue
    const struct NFA *sub = getSubEngine(t, activeIdx);
    assert(!isContainerType(sub->type));
    q2->nfa = sub;

    // Reinitialize state if the last active subengine is different
    // from current one
    if (lastActiveIdx == t->numSubEngines ||
        lastActiveIdx != activeIdx) {
        nfaQueueInitState(q2->nfa, q2);
    }

    copyQueueItems(t, sub, q1, q2, activeIdx);
    if (q1->items[q1->cur].type == MQE_END) {
        q1->cur++;
    }
    DEBUG_PRINTF("update lastIdx:%u\n", activeIdx);
    storeActiveIdx(t, q1->streamState, activeIdx);
}

static
void updateQueues(const struct Tamarama *t, struct mq *q1, struct mq *q2) {
    q2->cur = q2->end = 0;
    copyQueueProperties(q1, q2, t->activeIdxSize);

    const u32 numSubEngines = t->numSubEngines;
    u32 lastActiveIdx = loadActiveIdx(q1->streamState,
                                      t->activeIdxSize);
#ifdef DEBUG
    DEBUG_PRINTF("external queue\n");
    debugQueue(q1);
#endif

    // Push MQE_START event to the subqueue
    s64a loc = q1->items[q1->cur].location;
    pushQueueAt(q2, 0, MQE_START, loc);
    char hasStart = 0;
    if (q1->items[q1->cur].type == MQE_START) {
        hasStart = 1;
        q1->cur++;
    }

    u32 activeIdx = lastActiveIdx;
    // If we have top events in the main queue, update current active id
    if (q1->cur < q1->end - 1) {
        const u32 *baseTop = (const u32 *)((const char *)t +
                                           sizeof(struct Tamarama));
        u32 curTop = q1->items[q1->cur].type;
        activeIdx = findEngineForTop(baseTop, curTop, numSubEngines);
    }

    assert(activeIdx < numSubEngines);
    DEBUG_PRINTF("last id:%u, current id:%u, num of subengines:%u\n",
                 lastActiveIdx, activeIdx, numSubEngines);
    // Handle unfinished last alive subengine
    if (lastActiveIdx != activeIdx &&
        lastActiveIdx != numSubEngines && hasStart) {
        loc = q1->items[q1->cur].location;
        pushQueueNoMerge(q2, MQE_END, loc);
        q2->nfa = getSubEngine(t, lastActiveIdx);
        return;
    }

    initSubQueue(t, q1, q2, lastActiveIdx, activeIdx);
    DEBUG_PRINTF("finish queues\n");
}

// After processing subqueue items for subengines, we need to copy back
// remaining items in subqueue if there are any to Tamarama main queue
static
void copyBack(const struct  Tamarama *t, struct mq *q, struct mq *q1) {
    DEBUG_PRINTF("copy back %u, %u\n", q1->cur, q1->end);
    q->report_current = q1->report_current;
    if (q->cur >= q->end && q1->cur >= q1->end) {
        return;
    }

    const u32 *baseTop = (const u32 *)((const char *)t +
                                        sizeof(struct Tamarama));
    const u32 lastIdx = loadActiveIdx(q->streamState,
                                      t->activeIdxSize);
    u32 base = 0, event_base = 0;
    if (lastIdx != t->numSubEngines) {
        base = baseTop[lastIdx];
        const struct NFA *sub = getSubEngine(t, lastIdx);
        event_base = isMultiTopType(sub->type) ? MQE_TOP_FIRST : MQE_TOP;
    }

    u32 numItems = q1->end > q1->cur + 1 ? q1->end - q1->cur - 1 : 1;
    // Also need to copy MQE_END if the main queue is empty
    if (q->cur == q->end) {
        assert(q->cur > 1 && q1->items[q1->end - 1].type == MQE_END);
        q->items[--q->cur] = q1->items[q1->end - 1];
    }
    u32 cur = q->cur - numItems;
    q->items[cur] = q1->items[q1->cur++];
    q->items[cur].type = MQE_START;
    q->cur = cur++;
    for (u32 i = 0; i < numItems - 1; ++i) {
        assert(q1->cur < q1->end);
        u32 type = q1->items[q1->cur].type;
        if (type > MQE_END) {
            q1->items[q1->cur].type = type - event_base + base;
        }
        q->items[cur++] = q1->items[q1->cur++];
    }

#ifdef DEBUG
    DEBUG_PRINTF("external queue\n");
    debugQueue(q);
#endif
}

char nfaExecTamarama_testEOD(const struct NFA *n, const char *state,
                             const char *streamState, u64a offset,
                             NfaCallback callback, void *context) {
    const struct Tamarama *t = getImplNfa(n);
    u32 activeIdx = loadActiveIdx(streamState, t->activeIdxSize);
    if (activeIdx == t->numSubEngines) {
        return MO_CONTINUE_MATCHING;
    }

    const struct NFA *sub = getSubEngine(t, activeIdx);
    if (nfaAcceptsEod(sub)) {
        assert(!isContainerType(sub->type));
        const char *subStreamState = streamState + t->activeIdxSize;
        return nfaCheckFinalState(sub, state, subStreamState, offset, callback,
                                  context);
    }

    return MO_CONTINUE_MATCHING;
}

char nfaExecTamarama_QR(const struct NFA *n, struct mq *q, ReportID report) {
    DEBUG_PRINTF("exec rose\n");
    struct mq q1;
    q1.cur = q1.end = 0;
    char rv = 0;
    const struct Tamarama *t = getImplNfa(n);
    while (q->cur < q->end) {
        updateQueues(t, q, &q1);
    }

    if (q1.cur < q1.end) {
        rv = nfaQueueExecRose(q1.nfa, &q1, report);
    }

    DEBUG_PRINTF("exec rose rv:%u\n", rv);
    return rv;
}

char nfaExecTamarama_reportCurrent(const struct NFA *n, struct mq *q) {
    const struct Tamarama *t = getImplNfa(n);
    u32 activeIdx = loadActiveIdx(q->streamState, t->activeIdxSize);
    if (activeIdx == t->numSubEngines) {
        return 1;
    }

    const struct NFA *sub = getSubEngine(t, activeIdx);
    struct mq q1;
    copyQueue(t, sub, q, &q1, activeIdx);
    return nfaReportCurrentMatches(sub, &q1);
}

char nfaExecTamarama_inAccept(const struct NFA *n, ReportID report,
                              struct mq *q) {
    const struct Tamarama *t = getImplNfa(n);
    u32 activeIdx = loadActiveIdx(q->streamState, t->activeIdxSize);
    if (activeIdx == t->numSubEngines) {
        return 0;
    }
    const struct NFA *sub = getSubEngine(t, activeIdx);

    struct mq q1;
    copyQueue(t, sub, q, &q1, activeIdx);
    return nfaInAcceptState(sub, report, &q1);
}

char nfaExecTamarama_inAnyAccept(const struct NFA *n, struct mq *q) {
    const struct Tamarama *t = getImplNfa(n);
    u32 activeIdx = loadActiveIdx(q->streamState, t->activeIdxSize);
    if (activeIdx == t->numSubEngines) {
        return 0;
    }
    const struct NFA *sub = getSubEngine(t, activeIdx);

    struct mq q1;
    copyQueue(t, sub, q, &q1, activeIdx);
    return nfaInAnyAcceptState(sub, &q1);
}

char nfaExecTamarama_queueInitState(const struct NFA *n, struct mq *q) {
    DEBUG_PRINTF("init state\n");
    const struct Tamarama *t = getImplNfa(n);
    char *ptr = q->streamState;
    // Use activeIdxSize as a sentinel value and initialize the state to
    // an invalid engine as nothing has been triggered yet
    storeActiveIdx(t, ptr, t->numSubEngines);
    return 0;
}

char nfaExecTamarama_queueCompressState(const struct NFA *n, const struct mq *q,
                                        s64a loc) {
    const struct Tamarama *t = getImplNfa(n);
    u32 activeIdx = loadActiveIdx(q->streamState, t->activeIdxSize);
    if (activeIdx == t->numSubEngines) {
        return 0;
    }

    const struct NFA *sub = getSubEngine(t, activeIdx);

    struct mq q1;
    copyQueueProperties(q, &q1, t->activeIdxSize);
    return nfaQueueCompressState(sub, &q1, loc);
}

char nfaExecTamarama_expandState(const struct NFA *n, void *dest,
                                 const void *src, u64a offset, u8 key) {
    const struct Tamarama *t = getImplNfa(n);
    u32 activeIdx = loadActiveIdx(src, t->activeIdxSize);
    if (activeIdx == t->numSubEngines) {
        return 0;
    }

    const struct NFA *sub = getSubEngine(t, activeIdx);

    const char *subStreamState = (const char *)src + t->activeIdxSize;
    return nfaExpandState(sub, dest, subStreamState, offset, key);
}

enum nfa_zombie_status nfaExecTamarama_zombie_status(const struct NFA *n,
                                                     struct mq *q, s64a loc) {
    const struct Tamarama *t = getImplNfa(n);
    u32 activeIdx = loadActiveIdx(q->streamState, t->activeIdxSize);
    if (activeIdx == t->numSubEngines) {
        return NFA_ZOMBIE_NO;
    }
    const struct NFA *sub = getSubEngine(t, activeIdx);

    struct mq q1;
    copyQueue(t, sub, q, &q1, activeIdx);
    return nfaGetZombieStatus(sub, &q1, loc);
}

char nfaExecTamarama_Q(const struct NFA *n, struct mq *q, s64a end) {
    DEBUG_PRINTF("exec\n");
    struct mq q1;
    char rv = MO_ALIVE;
    char copy = 0;
    const struct Tamarama *t = getImplNfa(n);
    while (q->cur < q->end && q_cur_loc(q) <= end) {
        updateQueues(t, q, &q1);
        rv = nfaQueueExec_raw(q1.nfa, &q1, end);
        q->report_current = q1.report_current;
        copy = 1;
        if (can_stop_matching(q->scratch)) {
            break;
        }
    }
    if (copy) {
        copyBack(t, q, &q1);
    }
    return rv;
}

char nfaExecTamarama_Q2(const struct NFA *n, struct mq *q, s64a end) {
    DEBUG_PRINTF("exec to match\n");
    struct mq q1;
    char rv = 0;
    char copy = 0;
    const struct Tamarama *t = getImplNfa(n);
    while (q->cur < q->end && q_cur_loc(q) <= end &&
           rv != MO_MATCHES_PENDING) {
        updateQueues(t, q, &q1);
        rv = nfaQueueExec2_raw(q1.nfa, &q1, end);
        q->report_current = q1.report_current;
        copy = 1;
        if (can_stop_matching(q->scratch)) {
            break;
        }
    }
    if (copy) {
        copyBack(t, q, &q1);
    }
    return rv;
}

