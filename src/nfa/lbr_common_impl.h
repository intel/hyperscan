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
 * \brief Large Bounded Repeat (LBR) engine: runtime impl X-macros.
 */

#include "util/join.h"

#define ENGINE_EXEC_NAME JOIN(nfaExecLbr, ENGINE_ROOT_NAME)
#define EXEC_FN JOIN(lbrExec, ENGINE_ROOT_NAME)
#define FWDSCAN_FN JOIN(lbrFwdScan, ENGINE_ROOT_NAME)
#define REVSCAN_FN JOIN(lbrRevScan, ENGINE_ROOT_NAME)

char JOIN(ENGINE_EXEC_NAME, _queueCompressState)(const struct NFA *nfa,
                                                 const struct mq *q, s64a loc) {
    assert(nfa && q);
    assert(isLbrType(nfa->type));
    DEBUG_PRINTF("entry, q->offset=%llu, loc=%lld\n", q->offset, loc);

    const struct lbr_common *l = getImplNfa(nfa);
    const struct lbr_state *lstate = (const struct lbr_state *)q->state;

    u64a offset = q->offset + loc;
    lbrCompressState(l, offset, lstate, q->streamState);
    return 0;
}

char JOIN(ENGINE_EXEC_NAME, _expandState)(const struct NFA *nfa, void *dest,
                                          const void *src, u64a offset,
                                          UNUSED u8 key) {
    assert(nfa);
    assert(isLbrType(nfa->type));
    DEBUG_PRINTF("entry, offset=%llu\n", offset);

    const struct lbr_common *l = getImplNfa(nfa);
    struct lbr_state *lstate = (struct lbr_state *)dest;
    lbrExpandState(l, offset, src, lstate);
    return 0;
}

char JOIN(ENGINE_EXEC_NAME, _reportCurrent)(const struct NFA *nfa,
                                            struct mq *q) {
    assert(nfa && q);
    assert(isLbrType(nfa->type));

    const struct lbr_common *l = getImplNfa(nfa);
    u64a offset = q_cur_offset(q);
    DEBUG_PRINTF("firing match %u at %llu\n", l->report, offset);
    q->cb(0, offset, l->report, q->context);
    return 0;
}

char JOIN(ENGINE_EXEC_NAME, _inAccept)(const struct NFA *nfa,
                                       ReportID report, struct mq *q) {
    assert(nfa && q);
    assert(isLbrType(nfa->type));
    DEBUG_PRINTF("entry\n");

    const struct lbr_common *l = getImplNfa(nfa);
    const struct RepeatInfo *info = getRepeatInfo(l);
    const struct lbr_state *lstate = (const struct lbr_state *)q->state;
    if (repeatIsDead(info, lstate)) {
        DEBUG_PRINTF("repeat is dead\n");
        return 0;
    }

    u64a offset = q->offset + q_last_loc(q);
    return lbrInAccept(l, lstate, q->streamState, offset, report);
}

char JOIN(ENGINE_EXEC_NAME, _inAnyAccept)(const struct NFA *nfa, struct mq *q) {
    assert(nfa && q);
    assert(isLbrType(nfa->type));
    DEBUG_PRINTF("entry\n");

    const struct lbr_common *l = getImplNfa(nfa);
    return JOIN(ENGINE_EXEC_NAME, _inAccept)(nfa, l->report, q);
}

char JOIN(ENGINE_EXEC_NAME, _queueInitState)(const struct NFA *nfa,
                                             struct mq *q) {
    assert(nfa && q);
    assert(isLbrType(nfa->type));
    DEBUG_PRINTF("entry\n");

    const struct lbr_common *l = getImplNfa(nfa);
    const struct RepeatInfo *info = getRepeatInfo(l);

    assert(q->state);
    struct lbr_state *lstate = (struct lbr_state *)q->state;
    assert(ISALIGNED(lstate));

    lstate->lastEscape = 0;
    clearRepeat(info, lstate);

    return 0;
}

char JOIN(ENGINE_EXEC_NAME, _initCompressedState)(const struct NFA *nfa,
                                                  u64a offset,
                                                  void *state, UNUSED u8 key) {
    assert(nfa && state);
    assert(isLbrType(nfa->type));
    DEBUG_PRINTF("entry\n");

    const struct lbr_common *l = getImplNfa(nfa);
    const struct RepeatInfo *info = getRepeatInfo(l);
    struct lbr_state lstate; // temp control block on stack.
    clearRepeat(info, &lstate);
    lbrTop(l, &lstate, state, offset);
    lbrCompressState(l, offset, &lstate, state);

    return 1; // LBR is alive
}

// FIXME: this function could be much simpler for a Dot LBR, as all it needs to
// do is find the next top.
static really_inline
char JOIN(ENGINE_EXEC_NAME, _TopScan)(const struct NFA *nfa, struct mq *q,
                                      s64a end) {
    const struct lbr_common *l = getImplNfa(nfa);
    const struct RepeatInfo *info = getRepeatInfo(l);

    const u64a offset = q->offset;
    struct lbr_state *lstate = (struct lbr_state *)q->state;
    assert(ISALIGNED(lstate));

    assert(repeatIsDead(info, lstate));
    assert(q->cur < q->end);

    DEBUG_PRINTF("entry, end=%lld, offset=%llu, lastEscape=%llu\n", end,
                  offset, lstate->lastEscape);

    while (1) {
        // Find the next top with location >= the last escape we saw.
        for (; q->cur < q->end && q_cur_loc(q) <= end; q->cur++) {
            u32 event = q_cur_type(q);
            if ((event == MQE_TOP || event == MQE_TOP_FIRST) &&
                q_cur_offset(q) >= lstate->lastEscape) {
                goto found_top;
            }
            DEBUG_PRINTF("skip event type=%u offset=%lld\n", event, q_cur_offset(q));
        }

        // No more tops, we're done.
        break;

found_top:;
        assert(q->cur < q->end);

        u64a sp = q_cur_offset(q);
        u64a first_match = sp + info->repeatMin;
        DEBUG_PRINTF("first possible match is at %llu\n", first_match);

        u64a ep = MIN(MIN(end, (s64a)q->length) + offset, first_match);
        if (ep > sp && sp >= offset) {
            size_t eloc;
            DEBUG_PRINTF("rev b%llu e%llu/%zu\n", sp - offset, ep - offset,
                         q->length);
            assert(ep - offset <= q->length);
            if (REVSCAN_FN(nfa, q->buffer, sp - offset, ep - offset, &eloc)) {
                DEBUG_PRINTF("escape found at %llu\n", offset + eloc);
                lstate->lastEscape = eloc;
                q->cur++;
                continue;
            }
        }

        lbrTop(l, lstate, q->streamState, sp);
        return 1;
    }

    DEBUG_PRINTF("exhausted queue\n");
    return 0;
}

static really_inline
char JOIN(ENGINE_EXEC_NAME, _Q_i)(const struct NFA *nfa, struct mq *q,
                                  s64a end, enum MatchMode mode) {
    assert(nfa && q);
    assert(isLbrType(nfa->type));

    const struct lbr_common *l = getImplNfa(nfa);
    const struct RepeatInfo *info = getRepeatInfo(l);

    struct lbr_state *lstate = (struct lbr_state *)q->state;
    assert(ISALIGNED(lstate));


    if (q->report_current) {
        DEBUG_PRINTF("report_current: fire match at %llu\n", q_cur_offset(q));
        int rv = q->cb(0, q_cur_offset(q), l->report, q->context);
        q->report_current = 0;
        if (rv == MO_HALT_MATCHING) {
            return MO_HALT_MATCHING;
        }
    }

    if (q->cur == q->end) {
        return 1;
    }

    assert(q->cur + 1 < q->end); /* require at least two items */
    assert(q_cur_type(q) == MQE_START);
    u64a sp = q_cur_offset(q);
    q->cur++;
    DEBUG_PRINTF("sp=%llu, abs_end=%llu\n", sp, end + q->offset);

    while (q->cur < q->end) {
        DEBUG_PRINTF("q item type=%d offset=%llu\n", q_cur_type(q),
                     q_cur_offset(q));

        assert(sp >= q->offset); // not in history

        if (repeatIsDead(info, lstate)) {
            DEBUG_PRINTF("repeat is currently dead, skipping scan\n");
            goto scan_done;
        }

        u64a ep = q_cur_offset(q);
        ep = MIN(ep, q->offset + end);
        if (sp < ep) {
            size_t eloc = 0;
            char escape_found = 0;
            DEBUG_PRINTF("scanning from sp=%llu to ep=%llu\n", sp, ep);
            assert(sp >= q->offset && ep >= q->offset);
            if (FWDSCAN_FN(nfa, q->buffer, sp - q->offset, ep - q->offset, &eloc)) {
                escape_found = 1;
                ep = q->offset + eloc;
                DEBUG_PRINTF("escape found at %llu\n", ep);
                assert(ep >= sp);
            }

            assert(sp <= ep);

            if (mode == STOP_AT_MATCH) {
                size_t mloc;
                if (lbrFindMatch(l, sp, ep, lstate, q->streamState, &mloc)) {
                    DEBUG_PRINTF("storing match at %llu\n", sp + mloc);
                    q->cur--;
                    assert(q->cur < MAX_MQE_LEN);
                    q->items[q->cur].type = MQE_START;
                    q->items[q->cur].location = (s64a)(sp - q->offset) + mloc;
                    return MO_MATCHES_PENDING;
                }
            } else {
                assert(mode == CALLBACK_OUTPUT);
                char rv = lbrMatchLoop(l, sp, ep, lstate, q->streamState, q->cb,
                                       q->context);
                if (rv == MO_HALT_MATCHING) {
                    return MO_HALT_MATCHING;
                }
                assert(rv == MO_CONTINUE_MATCHING);
            }

            if (escape_found) {
                DEBUG_PRINTF("clearing repeat due to escape\n");
                clearRepeat(info, lstate);
            }
        }

    scan_done:
        if (q_cur_loc(q) > end) {
            q->cur--;
            assert(q->cur < MAX_MQE_LEN);
            q->items[q->cur].type = MQE_START;
            q->items[q->cur].location = end;
            return MO_ALIVE;
        }

        if (repeatIsDead(info, lstate)) {
            if (!JOIN(ENGINE_EXEC_NAME, _TopScan)(nfa, q, end)) {
                assert(repeatIsDead(info, lstate));
                if (q->cur < q->end && q_cur_loc(q) > end) {
                    q->cur--;
                    assert(q->cur < MAX_MQE_LEN);
                    q->items[q->cur].type = MQE_START;
                    q->items[q->cur].location = end;
                    return MO_ALIVE;
                }
                return 0;
            }
            DEBUG_PRINTF("cur offset = %llu\n", q_cur_offset(q));
        } else {
            switch (q_cur_type(q)) {
            case MQE_TOP:
            case MQE_TOP_FIRST:
                lbrTop(l, lstate, q->streamState, q_cur_offset(q));
                break;
            case MQE_START:
            case MQE_END:
                break;
            default:
                DEBUG_PRINTF("unhandled event %d!\n", q_cur_type(q));
                assert(0);
                break;
            }
        }

        sp = q_cur_offset(q);
        q->cur++;
    }

    return lbrIsAlive(l, lstate, q->streamState, sp);
}

char JOIN(ENGINE_EXEC_NAME, _Q)(const struct NFA *nfa, struct mq *q, s64a end) {
    DEBUG_PRINTF("entry, offset=%llu, end=%lld\n", q->offset, end);
    return JOIN(ENGINE_EXEC_NAME, _Q_i)(nfa, q, end, CALLBACK_OUTPUT);
}

char JOIN(ENGINE_EXEC_NAME, _Q2)(const struct NFA *nfa, struct mq *q, s64a end) {
    DEBUG_PRINTF("entry, offset=%llu, end=%lld\n", q->offset, end);
    return JOIN(ENGINE_EXEC_NAME, _Q_i)(nfa, q, end, STOP_AT_MATCH);
}

static really_inline
void JOIN(ENGINE_EXEC_NAME, _StreamSilent)(const struct NFA *nfa, struct mq *q,
                                           const u8 *buf, size_t length) {
    const struct lbr_common *l = getImplNfa(nfa);
    const struct RepeatInfo *info = getRepeatInfo(l);
    struct lbr_state *lstate = (struct lbr_state *)q->state;
    assert(ISALIGNED(lstate));

    assert(!repeatIsDead(info, lstate));

    // This call doesn't produce matches, so we elide the lbrMatchLoop call
    // entirely and just do escape scans to maintain the repeat.

    size_t eloc = 0;
    char escaped = FWDSCAN_FN(nfa, buf, 0, length, &eloc);
    if (escaped) {
        assert(eloc < length);
        DEBUG_PRINTF("escape found at %zu, clearing repeat\n", eloc);
        clearRepeat(info, lstate);
    }
}

// Rose infix path.
char JOIN(ENGINE_EXEC_NAME, _QR)(const struct NFA *nfa, struct mq *q,
                                 ReportID report) {
    assert(nfa && q);
    assert(isLbrType(nfa->type));

    if (q->cur == q->end) {
        return 1;
    }

    assert(q->cur + 1 < q->end); /* require at least two items */
    assert(q_cur_type(q) == MQE_START);
    u64a sp = q_cur_offset(q);
    q->cur++;
    DEBUG_PRINTF("sp=%llu\n", sp);

    const struct lbr_common *l = getImplNfa(nfa);
    const struct RepeatInfo *info = getRepeatInfo(l);
    struct lbr_state *lstate = (struct lbr_state *)q->state;
    assert(ISALIGNED(lstate));
    const s64a lastLoc = q_last_loc(q);

    while (q->cur < q->end) {
        DEBUG_PRINTF("q item type=%d offset=%llu\n", q_cur_type(q),
                     q_cur_offset(q));

        if (repeatIsDead(info, lstate)) {
            DEBUG_PRINTF("repeat is dead\n");
            goto scan_done;
        }

        u64a ep = q_cur_offset(q);

        if (sp < q->offset) {
            DEBUG_PRINTF("HISTORY BUFFER SCAN\n");
            assert(q->offset - sp <= q->hlength);
            u64a local_ep = MIN(q->offset, ep);
            const u8 *ptr = q->history + q->hlength + sp - q->offset;
            JOIN(ENGINE_EXEC_NAME, _StreamSilent)(nfa, q, ptr, local_ep - sp);
            sp = local_ep;
        }

        if (repeatIsDead(info, lstate)) {
            DEBUG_PRINTF("repeat is dead\n");
            goto scan_done;
        }

        if (sp < ep) {
            DEBUG_PRINTF("MAIN BUFFER SCAN\n");
            assert(ep - q->offset <= q->length);
            const u8 *ptr = q->buffer + sp - q->offset;
            JOIN(ENGINE_EXEC_NAME, _StreamSilent)(nfa, q, ptr, ep - sp);
        }

        if (repeatIsDead(info, lstate)) {
scan_done:
            if (!JOIN(ENGINE_EXEC_NAME, _TopScan)(nfa, q, lastLoc)) {
                assert(repeatIsDead(info, lstate));
                assert(q->cur == q->end);
                return 0;
            }
        } else {
            switch (q_cur_type(q)) {
            case MQE_TOP:
            case MQE_TOP_FIRST:
                lbrTop(l, lstate, q->streamState, q_cur_offset(q));
                break;
            case MQE_START:
            case MQE_END:
                break;
            default:
                DEBUG_PRINTF("unhandled event %d!\n", q_cur_type(q));
                assert(0);
                break;
            }
        }

        sp = q_cur_offset(q);
        q->cur++;
    }

    if (repeatIsDead(info, lstate)) {
        DEBUG_PRINTF("repeat is dead\n");
        return 0;
    }

    if (lbrInAccept(l, lstate, q->streamState, sp, report)) {
        return MO_MATCHES_PENDING;
    }

    return lbrIsActive(l, lstate, q->streamState, sp);
}

#undef ENGINE_EXEC_NAME
#undef EXEC_FN
#undef FWDSCAN_FN
#undef REVSCAN_FN
#undef ENGINE_ROOT_NAME
