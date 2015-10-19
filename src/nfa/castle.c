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
 * \brief Castle: multi-tenant repeat engine, runtime code.
 */

#include "castle.h"

#include "castle_internal.h"
#include "nfa_api.h"
#include "nfa_api_queue.h"
#include "nfa_internal.h"
#include "repeat.h"
#include "shufti.h"
#include "truffle.h"
#include "vermicelli.h"
#include "util/bitutils.h"
#include "util/multibit.h"
#include "util/partial_store.h"
#include "ue2common.h"

static really_inline
const struct SubCastle *getSubCastle(const struct Castle *c, u32 num) {
    assert(num < c->numRepeats);
    const struct SubCastle *sub =
        (const struct SubCastle *)((const char *)c + sizeof(struct Castle));
    assert(ISALIGNED(sub));
    return &sub[num];
}

static really_inline
const struct RepeatInfo *getRepeatInfo(const struct SubCastle *sub) {
    const struct RepeatInfo *repeatInfo =
        (const struct RepeatInfo *)((const char *)sub + sub->repeatInfoOffset);
    return repeatInfo;
}

static really_inline
union RepeatControl *getControl(char *full_state, const struct SubCastle *sub) {
    union RepeatControl *rctrl =
        (union RepeatControl *)(full_state + sub->fullStateOffset);
    assert(ISALIGNED(rctrl));
    return rctrl;
}

static really_inline
const union RepeatControl *getControlConst(const char *full_state,
                                           const struct SubCastle *sub) {
    const union RepeatControl *rctrl =
        (const union RepeatControl *)(full_state + sub->fullStateOffset);
    assert(ISALIGNED(rctrl));
    return rctrl;
}

enum MatchMode {
    CALLBACK_OUTPUT,
    STOP_AT_MATCH,
};

static really_inline
char subCastleReportCurrent(const struct Castle *c, struct mq *q,
                            const u64a offset, const u32 subIdx) {
    const struct SubCastle *sub = getSubCastle(c, subIdx);
    const struct RepeatInfo *info = getRepeatInfo(sub);

    union RepeatControl *rctrl = getControl(q->state, sub);
    char *rstate = (char *)q->streamState + sub->streamStateOffset +
                   info->packedCtrlSize;
    enum RepeatMatch match =
        repeatHasMatch(info, rctrl, rstate, offset);
    DEBUG_PRINTF("repeatHasMatch returned %d\n", match);
    if (match == REPEAT_MATCH) {
        DEBUG_PRINTF("firing match at %llu for sub %u\n", offset, subIdx);
        if (q->cb(offset, sub->report, q->context) == MO_HALT_MATCHING) {
            return MO_HALT_MATCHING;
        }
    }

    return MO_CONTINUE_MATCHING;
}

static really_inline
int castleReportCurrent(const struct Castle *c, struct mq *q) {
    const u64a offset = q_cur_offset(q);
    DEBUG_PRINTF("offset=%llu\n", offset);

    if (c->exclusive) {
        const u32 activeIdx = partial_load_u32(q->streamState,
                                               c->activeIdxSize);
        DEBUG_PRINTF("subcastle %u\n", activeIdx);
        if (activeIdx < c->numRepeats && subCastleReportCurrent(c, q,
                offset, activeIdx) == MO_HALT_MATCHING) {
            return MO_HALT_MATCHING;
        }
    }

    if (!c->pureExclusive) {
        const u8 *active = (const u8 *)q->streamState + c->activeIdxSize;
        for (u32 i = mmbit_iterate(active, c->numRepeats, MMB_INVALID);
             i != MMB_INVALID; i = mmbit_iterate(active, c->numRepeats, i)) {
            DEBUG_PRINTF("subcastle %u\n", i);
            if (subCastleReportCurrent(c, q, offset, i) == MO_HALT_MATCHING) {
                return MO_HALT_MATCHING;
            }
        }
    }

    return MO_CONTINUE_MATCHING;
}

static really_inline
char subCastleInAccept(const struct Castle *c, struct mq *q,
                       const ReportID report, const u64a offset,
                       const u32 subIdx) {
    const struct SubCastle *sub = getSubCastle(c, subIdx);

    if (sub->report != report) {
        return 0;
    }
    const struct RepeatInfo *info = getRepeatInfo(sub);

    union RepeatControl *rctrl = getControl(q->state, sub);
    char *rstate = (char *)q->streamState + sub->streamStateOffset +
                   info->packedCtrlSize;
    enum RepeatMatch match =
        repeatHasMatch(info, rctrl, rstate, offset);
    if (match == REPEAT_MATCH) {
        DEBUG_PRINTF("in an accept\n");
        return 1;
    }

    return 0;
}

static really_inline
char castleInAccept(const struct Castle *c, struct mq *q,
                    const ReportID report, const u64a offset) {
    DEBUG_PRINTF("offset=%llu\n", offset);

    if (c->exclusive) {
        const u32 activeIdx = partial_load_u32(q->streamState,
                                               c->activeIdxSize);
        if (activeIdx < c->numRepeats) {
            DEBUG_PRINTF("subcastle %u\n", activeIdx);
            if (subCastleInAccept(c, q, report, offset, activeIdx)) {
                return 1;
            }
        }
    }

    if (!c->pureExclusive) {
        const u8 *active = (const u8 *)q->streamState + c->activeIdxSize;
        for (u32 i = mmbit_iterate(active, c->numRepeats, MMB_INVALID);
             i != MMB_INVALID;
             i = mmbit_iterate(active, c->numRepeats, i)) {
            DEBUG_PRINTF("subcastle %u\n", i);
            if (subCastleInAccept(c, q, report, offset, i)) {
                return 1;
            }
        }
    }

    return 0;
}

static really_inline
void subCastleDeactivateStaleSubs(const struct Castle *c, const u64a offset,
                                  void *full_state, void *stream_state,
                                  const u32 subIdx) {
    u8 *active = (u8 *)stream_state;
    const struct SubCastle *sub = getSubCastle(c, subIdx);
    const struct RepeatInfo *info = getRepeatInfo(sub);

    union RepeatControl *rctrl = getControl(full_state, sub);
    char *rstate = (char *)stream_state + sub->streamStateOffset +
                       info->packedCtrlSize;

    if (repeatHasMatch(info, rctrl, rstate, offset) == REPEAT_STALE) {
        DEBUG_PRINTF("sub %u is stale at offset %llu\n", subIdx, offset);
        if (sub->exclusive) {
            partial_store_u32(stream_state, c->numRepeats, c->activeIdxSize);
        } else {
            mmbit_unset(active + c->activeIdxSize, c->numRepeats, subIdx);
        }
    }
}

static really_inline
void castleDeactivateStaleSubs(const struct Castle *c, const u64a offset,
                               void *full_state, void *stream_state) {
    DEBUG_PRINTF("offset=%llu\n", offset);

    if (c->exclusive) {
        const u32 activeIdx = partial_load_u32(stream_state, c->activeIdxSize);
        if (activeIdx < c->numRepeats) {
            DEBUG_PRINTF("subcastle %u\n", activeIdx);
            subCastleDeactivateStaleSubs(c, offset, full_state,
                                         stream_state, activeIdx);
        }
    }

    if (!c->pureExclusive) {
        const u8 *active = (const u8 *)stream_state + c->activeIdxSize;
        for (u32 i = mmbit_iterate(active, c->numRepeats, MMB_INVALID);
             i != MMB_INVALID;
             i = mmbit_iterate(active, c->numRepeats, i)) {
            DEBUG_PRINTF("subcastle %u\n", i);
            subCastleDeactivateStaleSubs(c, offset, full_state,
                                         stream_state, i);
        }
    }
}

static really_inline
void castleProcessTop(const struct Castle *c, const u32 top, const u64a offset,
                      void *full_state, void *stream_state) {
    assert(top < c->numRepeats);

    const struct SubCastle *sub = getSubCastle(c, top);
    const struct RepeatInfo *info = getRepeatInfo(sub);
    union RepeatControl *rctrl = getControl(full_state, sub);
    char *rstate = (char *)stream_state + sub->streamStateOffset +
                   info->packedCtrlSize;

    char is_alive = 0;
    if (sub->exclusive) {
        const u32 activeIdx = partial_load_u32(stream_state, c->activeIdxSize);
        is_alive = (activeIdx == top);
        partial_store_u32(stream_state, top, c->activeIdxSize);
    } else {
        u8 *active = (u8 *)stream_state + c->activeIdxSize;
        is_alive = mmbit_set(active, c->numRepeats, top);
    }

    if (!is_alive) {
        DEBUG_PRINTF("first top for inactive repeat %u\n", top);
    } else {
        DEBUG_PRINTF("repeat %u is already alive\n", top);
        // Caller should ensure we're not stale.
        assert(repeatHasMatch(info, rctrl, rstate, offset) !=
               REPEAT_STALE);

        // Ignore duplicate top events.
        u64a last = repeatLastTop(info, rctrl, rstate);

        assert(last <= offset);
        if (last == offset) {
            DEBUG_PRINTF("dupe top at %llu\n", offset);
            return;
        }
    }

    repeatStore(info, rctrl, rstate, offset, is_alive);
}

static really_inline
void subCastleFindMatch(const struct Castle *c, const u64a begin,
                        const u64a end, void *full_state, void *stream_state,
                        size_t *mloc, char *found, const u32 subIdx) {
    const struct SubCastle *sub = getSubCastle(c, subIdx);
    const struct RepeatInfo *info = getRepeatInfo(sub);
    union RepeatControl *rctrl = getControl(full_state, sub);
    char *rstate = (char *)stream_state + sub->streamStateOffset +
                   info->packedCtrlSize;

    u64a match = repeatNextMatch(info, rctrl, rstate, begin);
    if (match == 0) {
        DEBUG_PRINTF("no more matches for sub %u\n", subIdx);
        if (sub->exclusive) {
            partial_store_u32(stream_state, c->numRepeats,
                                  c->activeIdxSize);
        } else {
            u8 *active = (u8 *)stream_state + c->activeIdxSize;
            mmbit_unset(active, c->numRepeats, subIdx);
        }
        return;
    } else if (match > end) {
        DEBUG_PRINTF("next match for sub %u at %llu is > horizon\n", subIdx,
                     match);
        return;
    }
    DEBUG_PRINTF("sub %u earliest match at %llu\n", subIdx, match);
    size_t diff = match - begin;
    if (!(*found) || diff < *mloc) {
        *mloc = diff;
        DEBUG_PRINTF("mloc=%zu\n", *mloc);
    }
    *found = 1;
}

static really_inline
char castleFindMatch(const struct Castle *c, const u64a begin, const u64a end,
                     void *full_state, void *stream_state, size_t *mloc) {
    DEBUG_PRINTF("begin=%llu, end=%llu\n", begin, end);
    assert(begin <= end);

    if (begin == end) {
        DEBUG_PRINTF("no work to do\n");
        return 0;
    }

    char found = 0;
    *mloc = 0;

    if (c->exclusive) {
        const u32 activeIdx = partial_load_u32(stream_state, c->activeIdxSize);
        if (activeIdx < c->numRepeats) {
            DEBUG_PRINTF("subcastle %u\n", activeIdx);
            subCastleFindMatch(c, begin, end, full_state, stream_state, mloc,
                               &found, activeIdx);
        }
    }

    if (!c->pureExclusive) {
        u8 *active = (u8 *)stream_state + c->activeIdxSize;
        for (u32 i = mmbit_iterate(active, c->numRepeats, MMB_INVALID);
             i != MMB_INVALID;
             i = mmbit_iterate(active, c->numRepeats, i)) {
            DEBUG_PRINTF("subcastle %u\n", i);
            subCastleFindMatch(c, begin, end, full_state, stream_state, mloc,
                               &found, i);
        }
    }

    return found;
}

static really_inline
u64a subCastleNextMatch(const struct Castle *c, void *full_state,
                        void *stream_state, const u64a loc,
                        const u32 subIdx) {
    DEBUG_PRINTF("subcastle %u\n", subIdx);
    const struct SubCastle *sub = getSubCastle(c, subIdx);
    const struct RepeatInfo *info = getRepeatInfo(sub);
    const union RepeatControl *rctrl =
        getControlConst(full_state, sub);
    const char *rstate = (const char *)stream_state +
                         sub->streamStateOffset +
                         info->packedCtrlSize;

    return repeatNextMatch(info, rctrl, rstate, loc);
}

static really_inline
void subCastleMatchLoop(const struct Castle *c, void *full_state,
                        void *stream_state, const u64a end,
                        const u64a loc, u64a *offset) {
    u8 *active = (u8 *)stream_state + c->activeIdxSize;
    u8 *matching = full_state;
    mmbit_clear(matching, c->numRepeats);
    for (u32 i = mmbit_iterate(active, c->numRepeats, MMB_INVALID);
         i != MMB_INVALID; i = mmbit_iterate(active, c->numRepeats, i)) {
        u64a match = subCastleNextMatch(c, full_state, stream_state, loc, i);
        if (match == 0) {
            DEBUG_PRINTF("no more matches\n");
            mmbit_unset(active, c->numRepeats, i);
        } else if (match > end) {
            // If we had a local copy of the active mmbit, we could skip
            // looking at this repeat again. But we don't, so we just move
            // on.
        } else if (match == *offset) {
            mmbit_set(matching, c->numRepeats, i);
        } else if (match < *offset) {
            // New minimum offset.
            *offset = match;
            mmbit_clear(matching, c->numRepeats);
            mmbit_set(matching, c->numRepeats, i);
        }
    }
}

static really_inline
char subCastleFireMatch(const struct Castle *c, const void *full_state,
                        UNUSED const void *stream_state, NfaCallback cb,
                        void *ctx, const u64a offset) {
    const u8 *matching = full_state;

    // Fire all matching sub-castles at this offset.
    for (u32 i = mmbit_iterate(matching, c->numRepeats, MMB_INVALID);
         i != MMB_INVALID;
         i = mmbit_iterate(matching, c->numRepeats, i)) {
        const struct SubCastle *sub = getSubCastle(c, i);
        DEBUG_PRINTF("firing match at %llu for sub %u\n", offset, i);
        if (cb(offset, sub->report, ctx) == MO_HALT_MATCHING) {
            DEBUG_PRINTF("caller told us to halt\n");
            return MO_HALT_MATCHING;
        }
    }

    return MO_CONTINUE_MATCHING;
}

static really_inline
char castleMatchLoop(const struct Castle *c, const u64a begin, const u64a end,
                     void *full_state, void *stream_state, NfaCallback cb,
                     void *ctx) {
    DEBUG_PRINTF("begin=%llu, end=%llu\n", begin, end);
    assert(begin <= end);

    u8 *matching = full_state; // temp multibit

    u64a loc = begin;
    while (loc < end) {

        // Find minimum next offset for the next match(es) from amongst our
        // active sub-castles, and store the indices of the sub-castles that
        // match at that offset in the 'matching' mmbit, which is in the
        // full_state (scratch).

        u64a offset = end; // min offset of next match
        char found = 0;
        u32 activeIdx = 0;
        if (c->exclusive) {
            activeIdx = partial_load_u32(stream_state, c->activeIdxSize);
            if (activeIdx < c->numRepeats) {
                u32 i = activeIdx;
                DEBUG_PRINTF("subcastle %u\n", i);
                u64a match = subCastleNextMatch(c, full_state, stream_state,
                                                loc, i);

                if (match == 0) {
                    DEBUG_PRINTF("no more matches\n");
                    partial_store_u32(stream_state, c->numRepeats,
                                      c->activeIdxSize);
                } else if (match > end) {
                    // If we had a local copy of the active mmbit, we could skip
                    // looking at this repeat again. But we don't, so we just move
                    // on.
                } else if (match <= offset) {
                    if (match < offset) {
                        // New minimum offset.
                        offset = match;
                    }
                    found = 1;
                }
            }
        }

        const char hasMatch = found;
        u64a newOffset = offset;
        if (!c->pureExclusive) {
            subCastleMatchLoop(c, full_state, stream_state,
                               end, loc, &newOffset);

            DEBUG_PRINTF("offset=%llu\n", newOffset);
            if (mmbit_any(matching, c->numRepeats)) {
                found = 1;
                if (subCastleFireMatch(c, full_state, stream_state,
                        cb, ctx, newOffset) == MO_HALT_MATCHING) {
                    return MO_HALT_MATCHING;
                }
            }
        }

        if (!found) {
            break;
        } else if (hasMatch && offset == newOffset) {
            const struct SubCastle *sub = getSubCastle(c, activeIdx);
            DEBUG_PRINTF("firing match at %llu for sub %u\n", offset, activeIdx);
            if (cb(offset, sub->report, ctx) == MO_HALT_MATCHING) {
                DEBUG_PRINTF("caller told us to halt\n");
                return MO_HALT_MATCHING;
            }
        }
        loc = newOffset;
    }

    return MO_CONTINUE_MATCHING;
}

static really_inline
char castleScanVerm(const struct Castle *c, const u8 *buf, const size_t begin,
                    const size_t end, size_t *loc) {
    const u8 *ptr = vermicelliExec(c->u.verm.c, 0, buf + begin, buf + end);
    if (ptr == buf + end) {
        DEBUG_PRINTF("no escape found\n");
        return 0;
    }

    assert(loc);
    assert(ptr >= buf && ptr < buf + end);
    *loc = (size_t)(ptr - buf);
    DEBUG_PRINTF("escape found at offset %zu\n", *loc);
    return 1;
}

static really_inline
char castleScanNVerm(const struct Castle *c, const u8 *buf, const size_t begin,
                     const size_t end, size_t *loc) {
    const u8 *ptr = nvermicelliExec(c->u.verm.c, 0, buf + begin, buf + end);
    if (ptr == buf + end) {
        DEBUG_PRINTF("no escape found\n");
        return 0;
    }

    assert(loc);
    assert(ptr >= buf && ptr < buf + end);
    *loc = (size_t)(ptr - buf);
    DEBUG_PRINTF("escape found at offset %zu\n", *loc);
    return 1;
}

static really_inline
char castleScanShufti(const struct Castle *c, const u8 *buf, const size_t begin,
                      const size_t end, size_t *loc) {
    const m128 mask_lo = c->u.shuf.mask_lo;
    const m128 mask_hi = c->u.shuf.mask_hi;
    const u8 *ptr = shuftiExec(mask_lo, mask_hi, buf + begin, buf + end);
    if (ptr == buf + end) {
        DEBUG_PRINTF("no escape found\n");
        return 0;
    }

    assert(loc);
    assert(ptr >= buf && ptr < buf + end);
    *loc = (size_t)(ptr - buf);
    DEBUG_PRINTF("escape found at offset %zu\n", *loc);
    return 1;
}

static really_inline
char castleScanTruffle(const struct Castle *c, const u8 *buf, const size_t begin,
                      const size_t end, size_t *loc) {
    const u8 *ptr = truffleExec(c->u.truffle.mask1, c->u.truffle.mask2, buf + begin, buf + end);
    if (ptr == buf + end) {
        DEBUG_PRINTF("no escape found\n");
        return 0;
    }

    assert(loc);
    assert(ptr >= buf && ptr < buf + end);
    *loc = (size_t)(ptr - buf);
    DEBUG_PRINTF("escape found at offset %zu\n", *loc);
    return 1;
}

static really_inline
char castleScan(const struct Castle *c, const u8 *buf, const size_t begin,
                const size_t end, size_t *loc) {
    assert(begin <= end);

    if (begin == end) {
        return 0;
    }

    switch (c->type) {
    case CASTLE_DOT:
        // Nothing can stop a dot scan!
        return 0;
    case CASTLE_VERM:
        return castleScanVerm(c, buf, begin, end, loc);
    case CASTLE_NVERM:
        return castleScanNVerm(c, buf, begin, end, loc);
    case CASTLE_SHUFTI:
        return castleScanShufti(c, buf, begin, end, loc);
    case CASTLE_TRUFFLE:
        return castleScanTruffle(c, buf, begin, end, loc);
    default:
        DEBUG_PRINTF("unknown scan type!\n");
        assert(0);
        return 0;
    }
}

static really_inline
void castleHandleEvent(const struct Castle *c, struct mq *q, const u64a sp) {
    const u32 event = q->items[q->cur].type;
    switch (event) {
    case MQE_TOP:
        assert(0); // should be a numbered top
        break;
    case MQE_START:
    case MQE_END:
        break;
    default:
        assert(event >= MQE_TOP_FIRST);
        assert(event < MQE_INVALID);
        u32 top = event - MQE_TOP_FIRST;
        DEBUG_PRINTF("top %u at offset %llu\n", top, sp);
        castleProcessTop(c, top, sp, q->state, q->streamState);
        break;
    }
}

static really_inline
char nfaExecCastle0_Q_i(const struct NFA *n, struct mq *q, s64a end,
                        enum MatchMode mode) {
    assert(n && q);
    assert(n->type == CASTLE_NFA_0);

    DEBUG_PRINTF("state=%p, streamState=%p\n", q->state, q->streamState);

    const struct Castle *c = getImplNfa(n);

    if (q->report_current) {
        int rv = castleReportCurrent(c, q);
        q->report_current = 0;
        if (rv == MO_HALT_MATCHING) {
            return MO_HALT_MATCHING;
        }
    }

    if (q->cur == q->end) {
        return 1;
    }

    u8 *active = (u8 *)q->streamState + c->activeIdxSize; // active multibit

    assert(q->cur + 1 < q->end); // require at least two items
    assert(q_cur_type(q) == MQE_START);
    u64a sp = q_cur_offset(q);
    q->cur++;
    DEBUG_PRINTF("sp=%llu, abs_end=%llu\n", sp, end + q->offset);

    while (q->cur < q->end) {
        DEBUG_PRINTF("q item type=%d offset=%llu\n", q_cur_type(q),
                     q_cur_offset(q));

        char found = 0;
        if (c->exclusive) {
            const u32 activeIdx = partial_load_u32(q->streamState,
                                                   c->activeIdxSize);
            if (activeIdx < c->numRepeats) {
                found = 1;
            } else if (c->pureExclusive) {
                DEBUG_PRINTF("castle is dead\n");
                goto scan_done;
            }
        }

        if (!found && !mmbit_any(active, c->numRepeats)) {
            DEBUG_PRINTF("no repeats active, skipping scan\n");
            goto scan_done;
        }

        u64a ep = q_cur_offset(q);
        ep = MIN(ep, q->offset + end);
        if (sp < ep) {
            size_t eloc = 0;
            char escape_found = 0;
            DEBUG_PRINTF("scanning from sp=%llu to ep=%llu\n", sp, ep);
            assert(sp >= q->offset && ep >= q->offset);
            if (castleScan(c, q->buffer, sp - q->offset, ep - q->offset,
                           &eloc)) {
                escape_found = 1;
                ep = q->offset + eloc;
                DEBUG_PRINTF("escape found at %llu\n", ep);
                assert(ep >= sp);
            }

            assert(sp <= ep);

            if (mode == STOP_AT_MATCH) {
                size_t mloc;
                if (castleFindMatch(c, sp, ep, q->state, q->streamState,
                                    &mloc)) {
                    DEBUG_PRINTF("storing match at %llu\n", sp + mloc);
                    q->cur--;
                    assert(q->cur < MAX_MQE_LEN);
                    q->items[q->cur].type = MQE_START;
                    q->items[q->cur].location = (s64a)(sp - q->offset) + mloc;
                    return MO_MATCHES_PENDING;
                }
            } else {
                assert(mode == CALLBACK_OUTPUT);
                char rv = castleMatchLoop(c, sp, ep, q->state, q->streamState,
                                          q->cb, q->context);
                if (rv == MO_HALT_MATCHING) {
                    return MO_HALT_MATCHING;
                }
                assert(rv == MO_CONTINUE_MATCHING);
            }

            if (escape_found) {
                DEBUG_PRINTF("clearing active repeats due to escape\n");
                if (c->exclusive) {
                    partial_store_u32(q->streamState, c->numRepeats,
                                      c->activeIdxSize);
                }

                if (!c->pureExclusive) {
                    mmbit_clear(active, c->numRepeats);
                }
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

        sp = q_cur_offset(q);
        castleHandleEvent(c, q, sp);
        q->cur++;
    }

    if (c->exclusive) {
        const u32 activeIdx = partial_load_u32(q->streamState,
                                               c->activeIdxSize);
        if (c->pureExclusive || activeIdx < c->numRepeats) {
            return activeIdx < c->numRepeats;
        }
    }

    return mmbit_any_precise(active, c->numRepeats);
}

char nfaExecCastle0_Q(const struct NFA *n, struct mq *q, s64a end) {
    DEBUG_PRINTF("entry\n");
    return nfaExecCastle0_Q_i(n, q, end, CALLBACK_OUTPUT);
}

char nfaExecCastle0_Q2(const struct NFA *n, struct mq *q, s64a end) {
    DEBUG_PRINTF("entry\n");
    return nfaExecCastle0_Q_i(n, q, end, STOP_AT_MATCH);
}

static really_inline
void castleStreamSilent(const struct Castle *c, u8 *active, const u8 *buf,
                        size_t length) {
    DEBUG_PRINTF("entry\n");

    // This call doesn't produce matches, so we elide the castleMatchLoop call
    // entirely and just do escape scans to maintain the repeat.

    size_t eloc = 0;
    char escaped = castleScan(c, buf, 0, length, &eloc);
    if (escaped) {
        assert(eloc < length);
        DEBUG_PRINTF("escape found at %zu, clearing castle\n", eloc);
        if (c->exclusive) {
            partial_store_u32(active - c->activeIdxSize,
                              c->numRepeats, c->activeIdxSize);
        }

        if (!c->pureExclusive) {
            mmbit_clear(active, c->numRepeats);
        }
    }
}

char nfaExecCastle0_QR(const struct NFA *n, struct mq *q, ReportID report) {
    assert(n && q);
    assert(n->type == CASTLE_NFA_0);
    DEBUG_PRINTF("entry\n");

    if (q->cur == q->end) {
        return 1;
    }

    assert(q->cur + 1 < q->end); /* require at least two items */
    assert(q_cur_type(q) == MQE_START);
    u64a sp = q_cur_offset(q);
    q->cur++;
    DEBUG_PRINTF("sp=%llu\n", sp);

    const struct Castle *c = getImplNfa(n);
    u8 *active = (u8 *)q->streamState + c->activeIdxSize;
    char found = 0;
    while (q->cur < q->end) {
        DEBUG_PRINTF("q item type=%d offset=%llu\n", q_cur_type(q),
                     q_cur_offset(q));
        found = 0;
        if (c->exclusive) {
            const u32 activeIdx = partial_load_u32(q->streamState,
                                                   c->activeIdxSize);
            if (activeIdx < c->numRepeats) {
                found = 1;
            } else if (c->pureExclusive) {
                DEBUG_PRINTF("castle is dead\n");
                goto scan_done;
            }
        }

        if (!found && !mmbit_any(active, c->numRepeats)) {
            DEBUG_PRINTF("castle is dead\n");
            goto scan_done;
        }

        u64a ep = q_cur_offset(q);

        if (sp < q->offset) {
            DEBUG_PRINTF("HISTORY BUFFER SCAN\n");
            assert(q->offset - sp <= q->hlength);
            u64a local_ep = MIN(q->offset, ep);
            const u8 *ptr = q->history + q->hlength + sp - q->offset;
            castleStreamSilent(c, active, ptr, local_ep - sp);
            sp = local_ep;
        }

        found = 0;
        if (c->exclusive) {
            const u32 activeIdx = partial_load_u32(q->streamState,
                                                   c->activeIdxSize);
            if (activeIdx < c->numRepeats) {
                found = 1;
            } else if (c->pureExclusive) {
                DEBUG_PRINTF("castle is dead\n");
                goto scan_done;
            }
        }

        if (!found && !mmbit_any(active, c->numRepeats)) {
            DEBUG_PRINTF("castle is dead\n");
            goto scan_done;
        }

        if (sp < ep) {
            DEBUG_PRINTF("MAIN BUFFER SCAN\n");
            assert(ep - q->offset <= q->length);
            const u8 *ptr = q->buffer + sp - q->offset;
            castleStreamSilent(c, active, ptr, ep - sp);
        }

scan_done:
        sp = q_cur_offset(q);
        castleDeactivateStaleSubs(c, sp, q->state, q->streamState);
        castleHandleEvent(c, q, sp);
        q->cur++;
    }

    found = 0;
    if (c->exclusive) {
        const u32 activeIdx = partial_load_u32(q->streamState,
                                               c->activeIdxSize);
        if (activeIdx < c->numRepeats) {
            found = 1;
        } else if (c->pureExclusive) {
            DEBUG_PRINTF("castle is dead\n");
            return 0;
        }
    }

    if (!found && !mmbit_any_precise(active, c->numRepeats)) {
        DEBUG_PRINTF("castle is dead\n");
        return 0;
    }

    if (castleInAccept(c, q, report, sp)) {
        return MO_MATCHES_PENDING;
    }

    return 1;
}

char nfaExecCastle0_reportCurrent(const struct NFA *n, struct mq *q) {
    assert(n && q);
    assert(n->type == CASTLE_NFA_0);
    DEBUG_PRINTF("entry\n");

    const struct Castle *c = getImplNfa(n);
    castleReportCurrent(c, q);
    return 0;
}

char nfaExecCastle0_inAccept(const struct NFA *n, ReportID report,
                             struct mq *q) {
    assert(n && q);
    assert(n->type == CASTLE_NFA_0);
    DEBUG_PRINTF("entry\n");

    const struct Castle *c = getImplNfa(n);
    return castleInAccept(c, q, report, q_cur_offset(q));
}

char nfaExecCastle0_queueInitState(UNUSED const struct NFA *n, struct mq *q) {
    assert(n && q);
    assert(n->type == CASTLE_NFA_0);
    DEBUG_PRINTF("entry\n");

    const struct Castle *c = getImplNfa(n);
    assert(q->streamState);
    if (c->exclusive) {
        partial_store_u32(q->streamState, c->numRepeats, c->activeIdxSize);
    }

    if (!c->pureExclusive) {
        u8 *active = (u8 *)q->streamState + c->activeIdxSize;
        mmbit_clear(active, c->numRepeats);
    }
    return 0;
}

char nfaExecCastle0_initCompressedState(const struct NFA *n, UNUSED u64a offset,
                                        void *state, UNUSED u8 key) {
    assert(n && state);
    assert(n->type == CASTLE_NFA_0);
    DEBUG_PRINTF("entry\n");

    const struct Castle *c = getImplNfa(n);
    if (c->exclusive) {
        partial_store_u32(state, c->numRepeats, c->activeIdxSize);
    }

    if (!c->pureExclusive) {
        u8 *active = (u8 *)state + c->activeIdxSize;
        mmbit_clear(active, c->numRepeats);
    }
    return 0;
}

static really_inline
void subCastleQueueCompressState(const struct Castle *c, const u32 subIdx,
                                 const struct mq *q, const u64a offset) {
    const struct SubCastle *sub = getSubCastle(c, subIdx);
    const struct RepeatInfo *info = getRepeatInfo(sub);
    union RepeatControl *rctrl = getControl(q->state, sub);
    char *packed = (char *)q->streamState + sub->streamStateOffset;
    DEBUG_PRINTF("sub %u next match %llu\n", subIdx,
                 repeatNextMatch(info, rctrl,
                                 packed + info->packedCtrlSize, offset));
    repeatPack(packed, info, rctrl, offset);
}

char nfaExecCastle0_queueCompressState(const struct NFA *n, const struct mq *q,
                                       s64a loc) {
    assert(n && q);
    assert(n->type == CASTLE_NFA_0);
    DEBUG_PRINTF("entry, loc=%lld\n", loc);

    const struct Castle *c = getImplNfa(n);

    // Pack state for all active repeats.
    const u64a offset = q->offset + loc;
    DEBUG_PRINTF("offset=%llu\n", offset);
    if (c->exclusive) {
        const u32 activeIdx = partial_load_u32(q->streamState,
                                               c->activeIdxSize);
        if (activeIdx < c->numRepeats) {
            DEBUG_PRINTF("packing state for sub %u\n", activeIdx);
            subCastleQueueCompressState(c, activeIdx, q, offset);
        }
    }

    if (!c->pureExclusive) {
        const u8 *active = (const u8 *)q->streamState + c->activeIdxSize;
        for (u32 i = mmbit_iterate(active, c->numRepeats, MMB_INVALID);
             i != MMB_INVALID; i = mmbit_iterate(active, c->numRepeats, i)) {
            DEBUG_PRINTF("packing state for sub %u\n", i);
            subCastleQueueCompressState(c, i, q, offset);
        }
    }
    return 0;
}

static really_inline
void subCastleExpandState(const struct Castle *c, const u32 subIdx,
                          void *dest, const void *src, const u64a offset) {
    const struct SubCastle *sub = getSubCastle(c, subIdx);
    const struct RepeatInfo *info = getRepeatInfo(sub);
    DEBUG_PRINTF("unpacking state for sub %u\n", subIdx);
    union RepeatControl *rctrl = getControl(dest, sub);
    const char *packed = (const char *)src + sub->streamStateOffset;
    repeatUnpack(packed, info, offset, rctrl);
    DEBUG_PRINTF("sub %u next match %llu\n", subIdx,
                 repeatNextMatch(info, rctrl,
                                 packed + info->packedCtrlSize, offset));
}

char nfaExecCastle0_expandState(const struct NFA *n, void *dest,
                                const void *src, u64a offset,
                                UNUSED u8 key) {
    assert(n && dest && src);
    assert(n->type == CASTLE_NFA_0);
    DEBUG_PRINTF("entry, src=%p, dest=%p, offset=%llu\n", src, dest, offset);

    const struct Castle *c = getImplNfa(n);

    if (c->exclusive) {
        const u32 activeIdx = partial_load_u32(src, c->activeIdxSize);
        if (activeIdx < c->numRepeats) {
            subCastleExpandState(c, activeIdx, dest, src, offset);
        }
    }

    if (!c->pureExclusive) {
        // Unpack state for all active repeats.
        const u8 *active = (const u8 *)src + c->activeIdxSize;
        for (u32 i = mmbit_iterate(active, c->numRepeats, MMB_INVALID);
             i != MMB_INVALID; i = mmbit_iterate(active, c->numRepeats, i)) {
            subCastleExpandState(c, i, dest, src, offset);
        }
    }
    return 0;
}

