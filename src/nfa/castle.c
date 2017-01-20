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
        DEBUG_PRINTF("firing match at %llu for sub %u, report %u\n", offset,
                     subIdx, sub->report);
        if (q->cb(0, offset, sub->report, q->context) == MO_HALT_MATCHING) {
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
        u8 *active = (u8 *)q->streamState;
        u8 *groups = active + c->groupIterOffset;
        for (u32 i = mmbit_iterate(groups, c->numGroups, MMB_INVALID);
             i != MMB_INVALID; i = mmbit_iterate(groups, c->numGroups, i)) {
            u8 *cur = active + i * c->activeIdxSize;
            const u32 activeIdx = partial_load_u32(cur, c->activeIdxSize);
            DEBUG_PRINTF("subcastle %u\n", activeIdx);
            if (subCastleReportCurrent(c, q,
                    offset, activeIdx) == MO_HALT_MATCHING) {
                return MO_HALT_MATCHING;
            }
        }
    }

    if (c->exclusive != PURE_EXCLUSIVE) {
        const u8 *active = (const u8 *)q->streamState + c->activeOffset;
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
     /* ignore when just catching up due to full queue */
    if (report == MO_INVALID_IDX) {
        return 0;
    }

    if (c->exclusive) {
        u8 *active = (u8 *)q->streamState;
        u8 *groups = active + c->groupIterOffset;
        for (u32 i = mmbit_iterate(groups, c->numGroups, MMB_INVALID);
             i != MMB_INVALID; i = mmbit_iterate(groups, c->numGroups, i)) {
            u8 *cur = active + i * c->activeIdxSize;
            const u32 activeIdx = partial_load_u32(cur, c->activeIdxSize);
            DEBUG_PRINTF("subcastle %u\n", activeIdx);
            if (subCastleInAccept(c, q, report, offset, activeIdx)) {
                return 1;
            }
        }
    }

    if (c->exclusive != PURE_EXCLUSIVE) {
        const u8 *active = (const u8 *)q->streamState + c->activeOffset;
        for (u32 i = mmbit_iterate(active, c->numRepeats, MMB_INVALID);
             i != MMB_INVALID; i = mmbit_iterate(active, c->numRepeats, i)) {
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
    const struct SubCastle *sub = getSubCastle(c, subIdx);
    const struct RepeatInfo *info = getRepeatInfo(sub);

    union RepeatControl *rctrl = getControl(full_state, sub);
    char *rstate = (char *)stream_state + sub->streamStateOffset +
                       info->packedCtrlSize;

    if (repeatHasMatch(info, rctrl, rstate, offset) == REPEAT_STALE) {
        DEBUG_PRINTF("sub %u is stale at offset %llu\n", subIdx, offset);
        if (sub->exclusiveId < c->numRepeats) {
            u8 *active = (u8 *)stream_state;
            u8 *groups = active + c->groupIterOffset;
            mmbit_unset(groups, c->numGroups, sub->exclusiveId);
        } else {
            u8 *active = (u8 *)stream_state + c->activeOffset;
            mmbit_unset(active, c->numRepeats, subIdx);
        }
    }
}

static really_inline
void castleDeactivateStaleSubs(const struct Castle *c, const u64a offset,
                               void *full_state, void *stream_state) {
    DEBUG_PRINTF("offset=%llu\n", offset);

    if (!c->staleIterOffset) {
        DEBUG_PRINTF("{no repeats can go stale}\n");
        return; /* no subcastle can ever go stale */
    }

    if (c->exclusive) {
        u8 *active = (u8 *)stream_state;
        u8 *groups = active + c->groupIterOffset;
        for (u32 i = mmbit_iterate(groups, c->numGroups, MMB_INVALID);
             i != MMB_INVALID; i = mmbit_iterate(groups, c->numGroups, i)) {
            u8 *cur = active + i * c->activeIdxSize;
            const u32 activeIdx = partial_load_u32(cur, c->activeIdxSize);
            DEBUG_PRINTF("subcastle %u\n", activeIdx);
            subCastleDeactivateStaleSubs(c, offset, full_state,
                                         stream_state, activeIdx);
        }
    }

    if (c->exclusive != PURE_EXCLUSIVE) {
        const u8 *active = (const u8 *)stream_state + c->activeOffset;
        const struct mmbit_sparse_iter *it
            = (const void *)((const char *)c + c->staleIterOffset);

        struct mmbit_sparse_state si_state[MAX_SPARSE_ITER_STATES];
        u32 numRepeats = c->numRepeats;
        u32 idx = 0;

        u32 i = mmbit_sparse_iter_begin(active, numRepeats, &idx, it, si_state);
        while(i != MMB_INVALID) {
            DEBUG_PRINTF("subcastle %u\n", i);
            subCastleDeactivateStaleSubs(c, offset, full_state, stream_state, i);
            i = mmbit_sparse_iter_next(active, numRepeats, i, &idx, it,
                                       si_state);
        }
    }
}

static really_inline
void castleProcessTop(const struct Castle *c, const u32 top, const u64a offset,
                      void *full_state, void *stream_state,
                      UNUSED char stale_checked) {
    assert(top < c->numRepeats);

    const struct SubCastle *sub = getSubCastle(c, top);
    const struct RepeatInfo *info = getRepeatInfo(sub);
    union RepeatControl *rctrl = getControl(full_state, sub);
    char *rstate = (char *)stream_state + sub->streamStateOffset +
                   info->packedCtrlSize;

    char is_alive = 0;
    u8 *active = (u8 *)stream_state;
    if (sub->exclusiveId < c->numRepeats) {
        u8 *groups = active + c->groupIterOffset;
        active += sub->exclusiveId * c->activeIdxSize;
        if (mmbit_set(groups, c->numGroups, sub->exclusiveId)) {
            const u32 activeIdx = partial_load_u32(active, c->activeIdxSize);
            is_alive = (activeIdx == top);
        }

        if (!is_alive) {
            partial_store_u32(active, top, c->activeIdxSize);
        }
    } else {
        active += c->activeOffset;
        is_alive = mmbit_set(active, c->numRepeats, top);
    }

    if (!is_alive) {
        DEBUG_PRINTF("first top for inactive repeat %u\n", top);
    } else {
        DEBUG_PRINTF("repeat %u is already alive\n", top);
        // Caller should ensure we're not stale.
        assert(!stale_checked
               || repeatHasMatch(info, rctrl, rstate, offset) != REPEAT_STALE);

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
        if (sub->exclusiveId < c->numRepeats) {
            u8 *groups = (u8 *)stream_state + c->groupIterOffset;
            mmbit_unset(groups, c->numGroups, sub->exclusiveId);
        } else {
            u8 *active = (u8 *)stream_state + c->activeOffset;
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
        u8 *active = (u8 *)stream_state;
        u8 *groups = active + c->groupIterOffset;
        for (u32 i = mmbit_iterate(groups, c->numGroups, MMB_INVALID);
             i != MMB_INVALID; i = mmbit_iterate(groups, c->numGroups, i)) {
            u8 *cur = active + i * c->activeIdxSize;
            const u32 activeIdx = partial_load_u32(cur, c->activeIdxSize);
            DEBUG_PRINTF("subcastle %u\n", activeIdx);
            subCastleFindMatch(c, begin, end, full_state, stream_state, mloc,
                               &found, activeIdx);
        }
    }

    if (c->exclusive != PURE_EXCLUSIVE) {
        u8 *active = (u8 *)stream_state + c->activeOffset;
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
void set_matching(const struct Castle *c, const u64a match, u8 *active,
                  u8 *matching, const u32 active_size, const u32 active_id,
                  const u32 matching_id, u64a *offset, const u64a end) {
    if (match == 0) {
        DEBUG_PRINTF("no more matches\n");
        mmbit_unset(active, active_size, active_id);
    } else if (match > end) {
        // If we had a local copy of the active mmbit, we could skip
        // looking at this repeat again. But we don't, so we just move
        // on.
    } else if (match == *offset) {
        mmbit_set(matching, c->numRepeats, matching_id);
    } else if (match < *offset) {
        // New minimum offset.
        *offset = match;
        mmbit_clear(matching, c->numRepeats);
        mmbit_set(matching, c->numRepeats, matching_id);
    }
}

static really_inline
void subCastleMatchLoop(const struct Castle *c, void *full_state,
                        void *stream_state, const u64a end,
                        const u64a loc, u64a *offset) {
    u8 *active = (u8 *)stream_state + c->activeOffset;
    u8 *matching = full_state;
    for (u32 i = mmbit_iterate(active, c->numRepeats, MMB_INVALID);
         i != MMB_INVALID; i = mmbit_iterate(active, c->numRepeats, i)) {
        u64a match = subCastleNextMatch(c, full_state, stream_state, loc, i);
        set_matching(c, match, active, matching, c->numRepeats, i,
                     i, offset, end);
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
        if (cb(0, offset, sub->report, ctx) == MO_HALT_MATCHING) {
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
        u32 activeIdx = 0;
        mmbit_clear(matching, c->numRepeats);
        if (c->exclusive) {
            u8 *active = (u8 *)stream_state;
            u8 *groups = active + c->groupIterOffset;
            for (u32 i = mmbit_iterate(groups, c->numGroups, MMB_INVALID);
                 i != MMB_INVALID; i = mmbit_iterate(groups, c->numGroups, i)) {
                u8 *cur = active + i * c->activeIdxSize;
                activeIdx = partial_load_u32(cur, c->activeIdxSize);
                u64a match = subCastleNextMatch(c, full_state, stream_state,
                                                loc, activeIdx);
                set_matching(c, match, groups, matching, c->numGroups, i,
                             activeIdx, &offset, end);
            }
        }

        if (c->exclusive != PURE_EXCLUSIVE) {
            subCastleMatchLoop(c, full_state, stream_state,
                               end, loc, &offset);
        }
        DEBUG_PRINTF("offset=%llu\n", offset);
        if (!mmbit_any(matching, c->numRepeats)) {
            DEBUG_PRINTF("no more matches\n");
            break;
        }

        if (subCastleFireMatch(c, full_state, stream_state,
                cb, ctx, offset) == MO_HALT_MATCHING) {
            return MO_HALT_MATCHING;
        }
        loc = offset;
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
    const u8 *ptr = truffleExec(c->u.truffle.mask1, c->u.truffle.mask2,
                                buf + begin, buf + end);
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
char castleRevScanVerm(const struct Castle *c, const u8 *buf,
                       const size_t begin, const size_t end, size_t *loc) {
    const u8 *ptr = rvermicelliExec(c->u.verm.c, 0, buf + begin, buf + end);
    if (ptr == buf + begin - 1) {
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
char castleRevScanNVerm(const struct Castle *c, const u8 *buf,
                        const size_t begin, const size_t end, size_t *loc) {
    const u8 *ptr = rnvermicelliExec(c->u.verm.c, 0, buf + begin, buf + end);
    if (ptr == buf + begin - 1) {
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
char castleRevScanShufti(const struct Castle *c, const u8 *buf,
                         const size_t begin, const size_t end, size_t *loc) {
    const m128 mask_lo = c->u.shuf.mask_lo;
    const m128 mask_hi = c->u.shuf.mask_hi;
    const u8 *ptr = rshuftiExec(mask_lo, mask_hi, buf + begin, buf + end);
    if (ptr == buf + begin - 1) {
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
char castleRevScanTruffle(const struct Castle *c, const u8 *buf,
                          const size_t begin, const size_t end, size_t *loc) {
    const u8 *ptr = rtruffleExec(c->u.truffle.mask1, c->u.truffle.mask2,
                                 buf + begin, buf + end);
    if (ptr == buf + begin - 1) {
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
char castleRevScan(const struct Castle *c, const u8 *buf, const size_t begin,
                const size_t end, size_t *loc) {
    assert(begin <= end);
    DEBUG_PRINTF("scanning backwards over (%zu,%zu]\n", begin, end);
    if (begin == end) {
        return 0;
    }

    switch (c->type) {
    case CASTLE_DOT:
        // Nothing can stop a dot scan!
        return 0;
    case CASTLE_VERM:
        return castleRevScanVerm(c, buf, begin, end, loc);
    case CASTLE_NVERM:
        return castleRevScanNVerm(c, buf, begin, end, loc);
    case CASTLE_SHUFTI:
        return castleRevScanShufti(c, buf, begin, end, loc);
    case CASTLE_TRUFFLE:
        return castleRevScanTruffle(c, buf, begin, end, loc);
    default:
        DEBUG_PRINTF("unknown scan type!\n");
        assert(0);
        return 0;
    }
}

static really_inline
void castleHandleEvent(const struct Castle *c, struct mq *q, const u64a sp,
                       char stale_checked) {
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
        castleProcessTop(c, top, sp, q->state, q->streamState, stale_checked);
        break;
    }
}

static really_inline
void clear_repeats(const struct Castle *c, const struct mq *q, u8 *active) {
    DEBUG_PRINTF("clearing active repeats due to escape\n");
    if (c->exclusive) {
        u8 *groups = (u8 *)q->streamState + c->groupIterOffset;
        mmbit_clear(groups, c->numGroups);
    }

    if (c->exclusive != PURE_EXCLUSIVE) {
        mmbit_clear(active, c->numRepeats);
    }
}

static really_inline
char nfaExecCastle_Q_i(const struct NFA *n, struct mq *q, s64a end,
                       enum MatchMode mode) {
    assert(n && q);
    assert(n->type == CASTLE_NFA);

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

    u8 *active = (u8 *)q->streamState + c->activeOffset;// active multibit

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
            u8 *groups = (u8 *)q->streamState + c->groupIterOffset;
            found = mmbit_any(groups, c->numGroups);
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
                clear_repeats(c, q, active);
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
        castleHandleEvent(c, q, sp, 1);
        q->cur++;
    }

    if (c->exclusive) {
        u8 *groups = (u8 *)q->streamState + c->groupIterOffset;
        if (mmbit_any_precise(groups, c->numGroups)) {
            return 1;
        }
    }

    return mmbit_any_precise(active, c->numRepeats);
}

char nfaExecCastle_Q(const struct NFA *n, struct mq *q, s64a end) {
    DEBUG_PRINTF("entry\n");
    return nfaExecCastle_Q_i(n, q, end, CALLBACK_OUTPUT);
}

char nfaExecCastle_Q2(const struct NFA *n, struct mq *q, s64a end) {
    DEBUG_PRINTF("entry\n");
    return nfaExecCastle_Q_i(n, q, end, STOP_AT_MATCH);
}

static
s64a castleLastKillLoc(const struct Castle *c, struct mq *q) {
    assert(q_cur_type(q) == MQE_START);
    assert(q_last_type(q) == MQE_END);
    s64a sp = q_cur_loc(q);
    s64a ep = q_last_loc(q);

    DEBUG_PRINTF("finding final squash in (%lld, %lld]\n", sp, ep);

    size_t loc;

    if (ep > 0) {
        if (castleRevScan(c, q->buffer, sp > 0 ? sp : 0, ep, &loc)) {
            return (s64a)loc;
        }
        ep = 0;
    }

    if (sp < 0) {
        s64a hlen = q->hlength;

        if (castleRevScan(c, q->history, sp + hlen, ep + hlen, &loc)) {
            return (s64a)loc - hlen;
        }
        ep = 0;
    }

    return sp - 1; /* the repeats are never killed */
}

char nfaExecCastle_QR(const struct NFA *n, struct mq *q, ReportID report) {
    assert(n && q);
    assert(n->type == CASTLE_NFA);
    DEBUG_PRINTF("entry\n");

    if (q->cur == q->end) {
        return 1;
    }

    assert(q->cur + 1 < q->end); /* require at least two items */
    assert(q_cur_type(q) == MQE_START);

    const struct Castle *c = getImplNfa(n);
    u8 *active = (u8 *)q->streamState + c->activeOffset;

    u64a end_offset = q_last_loc(q) + q->offset;
    s64a last_kill_loc = castleLastKillLoc(c, q);
    DEBUG_PRINTF("all repeats killed at %lld (exec range %lld, %lld)\n",
                 last_kill_loc, q_cur_loc(q), q_last_loc(q));
    assert(last_kill_loc < q_last_loc(q));

    if (last_kill_loc != q_cur_loc(q) - 1) {
        clear_repeats(c, q, active);
    }

    q->cur++; /* skip start event */

    /* skip events prior to the repeats being squashed */
    while (q_cur_loc(q) <= last_kill_loc) {
        DEBUG_PRINTF("skipping moot event at %lld\n", q_cur_loc(q));
        q->cur++;
        assert(q->cur < q->end);
    }

    while (q->cur < q->end) {
        DEBUG_PRINTF("q item type=%d offset=%llu\n", q_cur_type(q),
                     q_cur_offset(q));
        u64a sp = q_cur_offset(q);
        castleHandleEvent(c, q, sp, 0);
        q->cur++;
    }

    castleDeactivateStaleSubs(c, end_offset, q->state, q->streamState);

    char found = 0;
    if (c->exclusive) {
        u8 *groups = (u8 *)q->streamState + c->groupIterOffset;
        found = mmbit_any_precise(groups, c->numGroups);

    }

    if (!found && !mmbit_any_precise(active, c->numRepeats)) {
        DEBUG_PRINTF("castle is dead\n");
        return 0;
    }

    if (castleInAccept(c, q, report, end_offset)) {
        return MO_MATCHES_PENDING;
    }

    return 1;
}

char nfaExecCastle_reportCurrent(const struct NFA *n, struct mq *q) {
    assert(n && q);
    assert(n->type == CASTLE_NFA);
    DEBUG_PRINTF("entry\n");

    const struct Castle *c = getImplNfa(n);
    castleReportCurrent(c, q);
    return 0;
}

char nfaExecCastle_inAccept(const struct NFA *n, ReportID report,
                            struct mq *q) {
    assert(n && q);
    assert(n->type == CASTLE_NFA);
    DEBUG_PRINTF("entry\n");

    const struct Castle *c = getImplNfa(n);
    return castleInAccept(c, q, report, q_cur_offset(q));
}

char nfaExecCastle_inAnyAccept(const struct NFA *n, struct mq *q) {
    assert(n && q);
    assert(n->type == CASTLE_NFA);
    DEBUG_PRINTF("entry\n");

    const struct Castle *c = getImplNfa(n);
    const u64a offset = q_cur_offset(q);
    DEBUG_PRINTF("offset=%llu\n", offset);

    if (c->exclusive) {
        u8 *active = (u8 *)q->streamState;
        u8 *groups = active + c->groupIterOffset;
        for (u32 i = mmbit_iterate(groups, c->numGroups, MMB_INVALID);
             i != MMB_INVALID; i = mmbit_iterate(groups, c->numGroups, i)) {
            u8 *cur = active + i * c->activeIdxSize;
            const u32 activeIdx = partial_load_u32(cur, c->activeIdxSize);
            DEBUG_PRINTF("subcastle %u\n", activeIdx);
            const struct SubCastle *sub = getSubCastle(c, activeIdx);
            if (subCastleInAccept(c, q, sub->report, offset, activeIdx)) {
                return 1;
            }
        }
    }

    if (c->exclusive != PURE_EXCLUSIVE) {
        const u8 *active = (const u8 *)q->streamState + c->activeOffset;
        for (u32 i = mmbit_iterate(active, c->numRepeats, MMB_INVALID);
             i != MMB_INVALID; i = mmbit_iterate(active, c->numRepeats, i)) {
            DEBUG_PRINTF("subcastle %u\n", i);
            const struct SubCastle *sub = getSubCastle(c, i);
            if (subCastleInAccept(c, q, sub->report, offset, i)) {
                return 1;
            }
        }
    }

    return 0;
}


char nfaExecCastle_queueInitState(UNUSED const struct NFA *n, struct mq *q) {
    assert(n && q);
    assert(n->type == CASTLE_NFA);
    DEBUG_PRINTF("entry\n");

    const struct Castle *c = getImplNfa(n);
    assert(q->streamState);
    if (c->exclusive) {
        u8 *groups = (u8 *)q->streamState + c->groupIterOffset;
        mmbit_clear(groups, c->numGroups);
    }

    if (c->exclusive != PURE_EXCLUSIVE) {
        u8 *active = (u8 *)q->streamState + c->activeOffset;
        mmbit_clear(active, c->numRepeats);
    }
    return 0;
}

char nfaExecCastle_initCompressedState(const struct NFA *n, UNUSED u64a offset,
                                       void *state, UNUSED u8 key) {
    assert(n && state);
    assert(n->type == CASTLE_NFA);
    DEBUG_PRINTF("entry\n");

    const struct Castle *c = getImplNfa(n);
    if (c->exclusive) {
        u8 *groups = (u8 *)state + c->groupIterOffset;
        mmbit_clear(groups, c->numGroups);
    }

    if (c->exclusive != PURE_EXCLUSIVE) {
        u8 *active = (u8 *)state + c->activeOffset;
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

char nfaExecCastle_queueCompressState(const struct NFA *n, const struct mq *q,
                                      s64a loc) {
    assert(n && q);
    assert(n->type == CASTLE_NFA);
    DEBUG_PRINTF("entry, loc=%lld\n", loc);

    const struct Castle *c = getImplNfa(n);

    // Pack state for all active repeats.
    const u64a offset = q->offset + loc;
    DEBUG_PRINTF("offset=%llu\n", offset);
    if (c->exclusive) {
        u8 *active = (u8 *)q->streamState;
        u8 *groups = active + c->groupIterOffset;
        for (u32 i = mmbit_iterate(groups, c->numGroups, MMB_INVALID);
             i != MMB_INVALID; i = mmbit_iterate(groups, c->numGroups, i)) {
            u8 *cur = active + i * c->activeIdxSize;
            const u32 activeIdx = partial_load_u32(cur, c->activeIdxSize);
            DEBUG_PRINTF("packing state for sub %u\n", activeIdx);
            subCastleQueueCompressState(c, activeIdx, q, offset);
        }
    }

    if (c->exclusive != PURE_EXCLUSIVE) {
        const u8 *active = (const u8 *)q->streamState + c->activeOffset;
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

char nfaExecCastle_expandState(const struct NFA *n, void *dest, const void *src,
                               u64a offset, UNUSED u8 key) {
    assert(n && dest && src);
    assert(n->type == CASTLE_NFA);
    DEBUG_PRINTF("entry, src=%p, dest=%p, offset=%llu\n", src, dest, offset);

    const struct Castle *c = getImplNfa(n);

    if (c->exclusive) {
        const u8 *active = (const u8 *)src;
        const u8 *groups = active + c->groupIterOffset;
        for (u32 i = mmbit_iterate(groups, c->numGroups, MMB_INVALID);
             i != MMB_INVALID; i = mmbit_iterate(groups, c->numGroups, i)) {
            const u8 *cur = active + i * c->activeIdxSize;
            const u32 activeIdx = partial_load_u32(cur, c->activeIdxSize);
            subCastleExpandState(c, activeIdx, dest, src, offset);
        }
    }

    if (c->exclusive != PURE_EXCLUSIVE) {
        // Unpack state for all active repeats.
        const u8 *active = (const u8 *)src + c->activeOffset;
        for (u32 i = mmbit_iterate(active, c->numRepeats, MMB_INVALID);
             i != MMB_INVALID; i = mmbit_iterate(active, c->numRepeats, i)) {
            subCastleExpandState(c, i, dest, src, offset);
        }
    }
    return 0;
}
