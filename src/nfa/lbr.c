/*
 * Copyright (c) 2015-2017, Intel Corporation
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
 * \brief Large Bounded Repeat (LBR) engine: runtime code.
 */
#include "lbr.h"

#include "lbr_internal.h"
#include "nfa_api.h"
#include "nfa_api_queue.h"
#include "nfa_internal.h"
#include "repeat.h"
#include "repeat_internal.h"
#include "shufti.h"
#include "truffle.h"
#include "vermicelli.h"
#include "util/partial_store.h"
#include "util/unaligned.h"

/** \brief Sentinel value used to indicate that a repeat is dead/empty/unused.
 *  * */
#define REPEAT_DEAD 0xffffffffffffffffull

enum MatchMode {
    CALLBACK_OUTPUT,
    STOP_AT_MATCH,
};

static really_inline
const struct RepeatInfo *getRepeatInfo(const struct lbr_common *l) {
    const struct RepeatInfo *repeatInfo =
        (const struct RepeatInfo *)((const char *)l + l->repeatInfoOffset);
    return repeatInfo;
}

static really_inline
void lbrCompressState(const struct lbr_common *l, u64a offset,
                      const struct lbr_state *lstate, char *stream_state) {
    assert(l && lstate && stream_state);
    assert(ISALIGNED(lstate));

    const struct RepeatInfo *info = getRepeatInfo(l);
    repeatPack(stream_state, info, &lstate->ctrl, offset);
}

static really_inline
void lbrExpandState(const struct lbr_common *l, u64a offset,
                    const char *stream_state, struct lbr_state *lstate) {
    assert(l && stream_state && lstate);
    assert(ISALIGNED(lstate));

    const struct RepeatInfo *info = getRepeatInfo(l);
    repeatUnpack(stream_state, info, offset, &lstate->ctrl);
    lstate->lastEscape = 0;
}

static really_inline
void clearRepeat(const struct RepeatInfo *info, struct lbr_state *lstate) {
    assert(info && lstate);

    DEBUG_PRINTF("clear repeat at %p\n", lstate);

    switch ((enum RepeatType)info->type) {
    case REPEAT_RING:
        lstate->ctrl.ring.offset = REPEAT_DEAD;
        break;
    case REPEAT_RANGE:
        lstate->ctrl.range.offset = REPEAT_DEAD;
        break;
    case REPEAT_FIRST:
    case REPEAT_LAST:
        lstate->ctrl.offset.offset = REPEAT_DEAD;
        break;
    case REPEAT_BITMAP:
        lstate->ctrl.bitmap.offset = REPEAT_DEAD;
        break;
    case REPEAT_SPARSE_OPTIMAL_P:
        lstate->ctrl.ring.offset = REPEAT_DEAD;
        break;
    case REPEAT_TRAILER:
        lstate->ctrl.trailer.offset = REPEAT_DEAD;
        break;
    default:
        assert(0);
        break;
    }
}

static really_inline
char repeatIsDead(const struct RepeatInfo *info,
                  const struct lbr_state *lstate) {
    assert(info && lstate);

    switch ((enum RepeatType)info->type) {
    case REPEAT_RING:
        return lstate->ctrl.ring.offset == REPEAT_DEAD;
    case REPEAT_RANGE:
        return lstate->ctrl.range.offset == REPEAT_DEAD;
    case REPEAT_FIRST:
    case REPEAT_LAST:
        return lstate->ctrl.offset.offset == REPEAT_DEAD;
    case REPEAT_BITMAP:
        return lstate->ctrl.bitmap.offset == REPEAT_DEAD;
    case REPEAT_SPARSE_OPTIMAL_P:
        return lstate->ctrl.ring.offset == REPEAT_DEAD;
    case REPEAT_TRAILER:
        return lstate->ctrl.trailer.offset == REPEAT_DEAD;
    case REPEAT_ALWAYS:
        assert(!"REPEAT_ALWAYS should only be used by Castle");
        return 0;
    }

    assert(0);
    return 1;
}

/** Returns true if the LBR can produce matches at offsets greater than the
 * given one. TODO: can this be combined with lbrIsActive? */
static really_inline
char lbrIsAlive(const struct lbr_common *l, const struct lbr_state *lstate,
                const char *state, u64a offset) {
    assert(l && lstate && state);

    const struct RepeatInfo *info = getRepeatInfo(l);
    if (repeatIsDead(info, lstate)) {
        DEBUG_PRINTF("repeat is dead\n");
        return 0;
    }

    if (info->repeatMax == REPEAT_INF) {
        DEBUG_PRINTF("active repeat with inf max bound, alive\n");
        return 1;
    }

    assert(info->repeatMax < REPEAT_INF);
    const char *repeatState = state + info->packedCtrlSize;
    u64a lastTop = repeatLastTop(info, &lstate->ctrl, repeatState);
    if (offset < lastTop + info->repeatMax) {
        DEBUG_PRINTF("alive, as we can still produce matches after %llu\n",
                     offset);
        return 1;
    }

    DEBUG_PRINTF("dead\n");
    return 0;
}

/** Returns true if the LBR is matching at the given offset or it could produce
 * a match in the future. */
static really_inline
char lbrIsActive(const struct lbr_common *l, const struct lbr_state *lstate,
                 const char *state, u64a offset) {
    assert(l && lstate && state);
    const struct RepeatInfo *info = getRepeatInfo(l);
    assert(!repeatIsDead(info, lstate)); // Guaranteed by caller.

    const char *repeatState = state + info->packedCtrlSize;
    if (repeatHasMatch(info, &lstate->ctrl, repeatState, offset) ==
        REPEAT_MATCH) {
        DEBUG_PRINTF("currently matching\n");
        return 1;
    }

    u64a i = repeatNextMatch(info, &lstate->ctrl, repeatState, offset);
    if (i != 0) {
        DEBUG_PRINTF("active, next match is at %llu\n", i);
        return 1;
    }

    DEBUG_PRINTF("no more matches\n");
    return 0;
}

static really_inline
void lbrTop(const struct lbr_common *l, struct lbr_state *lstate, char *state,
            u64a offset) {
    assert(l && lstate && state);
    DEBUG_PRINTF("top at %llu\n", offset);

    const struct RepeatInfo *info = getRepeatInfo(l);
    char *repeatState = state + info->packedCtrlSize;

    char is_alive = !repeatIsDead(info, lstate);
    if (is_alive) {
        // Ignore duplicate TOPs.
        u64a last = repeatLastTop(info, &lstate->ctrl, repeatState);
        assert(last <= offset);
        if (last == offset) {
            return;
        }
    }

    repeatStore(info, &lstate->ctrl, repeatState, offset, is_alive);
}

static really_inline
char lbrInAccept(const struct lbr_common *l, const struct lbr_state *lstate,
                 const char *state, u64a offset, ReportID report) {
    assert(l && lstate && state);
    DEBUG_PRINTF("offset=%llu, report=%u\n", offset, report);

    if (report != l->report) {
        DEBUG_PRINTF("report=%u is not LBR report %u\n", report, l->report);
        return 0;
    }

    const struct RepeatInfo *info = getRepeatInfo(l);
    assert(!repeatIsDead(info, lstate)); // Guaranteed by caller.

    const char *repeatState = state + info->packedCtrlSize;
    return repeatHasMatch(info, &lstate->ctrl, repeatState, offset) ==
           REPEAT_MATCH;
}

static really_inline
char lbrFindMatch(const struct lbr_common *l, const u64a begin, const u64a end,
                  const struct lbr_state *lstate, const char *state,
                  size_t *mloc) {
    DEBUG_PRINTF("begin=%llu, end=%llu\n", begin, end);
    assert(begin <= end);

    if (begin == end) {
        return 0;
    }

    const struct RepeatInfo *info = getRepeatInfo(l);
    const char *repeatState = state + info->packedCtrlSize;
    u64a i = repeatNextMatch(info, &lstate->ctrl, repeatState, begin);
    if (i == 0) {
        DEBUG_PRINTF("no more matches\n");
        return 0;
    }
    if (i > end) {
        DEBUG_PRINTF("next match at %llu is beyond the horizon\n", i);
        return 0;
    }

    DEBUG_PRINTF("stop at match at %llu\n", i);
    assert(mloc);
    *mloc = i - begin;
    return 1;
}

static really_inline
char lbrMatchLoop(const struct lbr_common *l, const u64a begin, const u64a end,
                  const struct lbr_state *lstate, const char *state,
                  NfaCallback cb, void *ctx) {
    DEBUG_PRINTF("begin=%llu, end=%llu\n", begin, end);
    assert(begin <= end);

    if (begin == end) {
        return MO_CONTINUE_MATCHING;
    }

    const struct RepeatInfo *info = getRepeatInfo(l);
    const char *repeatState = state + info->packedCtrlSize;

    u64a i = begin;
    for (;;) {
        i = repeatNextMatch(info, &lstate->ctrl, repeatState, i);
        if (i == 0) {
            DEBUG_PRINTF("no more matches\n");
            return MO_CONTINUE_MATCHING;
        }
        if (i > end) {
            DEBUG_PRINTF("next match at %llu is beyond the horizon\n", i);
            return MO_CONTINUE_MATCHING;
        }

        DEBUG_PRINTF("firing match at %llu\n", i);
        if (cb(0, i, l->report, ctx) == MO_HALT_MATCHING) {
            return MO_HALT_MATCHING;
        }
    }

    assert(0);
    return MO_CONTINUE_MATCHING;
}

static really_inline
char lbrRevScanDot(UNUSED const struct NFA *nfa, UNUSED const u8 *buf,
                   UNUSED size_t begin, UNUSED size_t end,
                   UNUSED size_t *loc) {
    assert(begin <= end);
    assert(nfa->type == LBR_NFA_DOT);
    // Nothing can kill a dot!
    return 0;
}

static really_inline
char lbrRevScanVerm(const struct NFA *nfa, const u8 *buf,
                    size_t begin, size_t end, size_t *loc) {
    assert(begin <= end);
    assert(nfa->type == LBR_NFA_VERM);
    const struct lbr_verm *l = getImplNfa(nfa);

    if (begin == end) {
        return 0;
    }

    const u8 *ptr = rvermicelliExec(l->c, 0, buf + begin, buf + end);
    if (ptr == buf + begin - 1) {
        DEBUG_PRINTF("no escape found\n");
        return 0;
    }

    assert(loc);
    *loc = (size_t)(ptr - buf);
    DEBUG_PRINTF("escape found at offset %zu\n", *loc);
    assert((char)*ptr == l->c);
    return 1;
}

static really_inline
char lbrRevScanNVerm(const struct NFA *nfa, const u8 *buf,
                     size_t begin, size_t end, size_t *loc) {
    assert(begin <= end);
    assert(nfa->type == LBR_NFA_NVERM);
    const struct lbr_verm *l = getImplNfa(nfa);

    if (begin == end) {
        return 0;
    }

    const u8 *ptr = rnvermicelliExec(l->c, 0, buf + begin, buf + end);
    if (ptr == buf + begin - 1) {
        DEBUG_PRINTF("no escape found\n");
        return 0;
    }

    assert(loc);
    *loc = (size_t)(ptr - buf);
    DEBUG_PRINTF("escape found at offset %zu\n", *loc);
    assert((char)*ptr != l->c);
    return 1;
}

static really_inline
char lbrRevScanShuf(const struct NFA *nfa, const u8 *buf,
                    size_t begin, size_t end,
                    size_t *loc) {
    assert(begin <= end);
    assert(nfa->type == LBR_NFA_SHUF);
    const struct lbr_shuf *l = getImplNfa(nfa);

    if (begin == end) {
        return 0;
    }

    const u8 *ptr = rshuftiExec(l->mask_lo, l->mask_hi, buf + begin, buf + end);
    if (ptr == buf + begin - 1) {
        DEBUG_PRINTF("no escape found\n");
        return 0;
    }

    assert(loc);
    *loc = (size_t)(ptr - buf);
    DEBUG_PRINTF("escape found at offset %zu\n", *loc);
    return 1;
}

static really_inline
char lbrRevScanTruf(const struct NFA *nfa, const u8 *buf,
                    size_t begin, size_t end,
                    size_t *loc) {
    assert(begin <= end);
    assert(nfa->type == LBR_NFA_TRUF);
    const struct lbr_truf *l = getImplNfa(nfa);

    if (begin == end) {
        return 0;
    }

    const u8 *ptr = rtruffleExec(l->mask1, l->mask2, buf + begin, buf + end);
    if (ptr == buf + begin - 1) {
        DEBUG_PRINTF("no escape found\n");
        return 0;
    }

    assert(loc);
    *loc = (size_t)(ptr - buf);
    DEBUG_PRINTF("escape found at offset %zu\n", *loc);
    return 1;
}

static really_inline
char lbrFwdScanDot(UNUSED const struct NFA *nfa, UNUSED const u8 *buf,
                   UNUSED size_t begin, UNUSED size_t end,
                   UNUSED size_t *loc) {
    assert(begin <= end);
    assert(nfa->type == LBR_NFA_DOT);
    // Nothing can kill a dot!
    return 0;
}

static really_inline
char lbrFwdScanVerm(const struct NFA *nfa, const u8 *buf,
                    size_t begin, size_t end, size_t *loc) {
    assert(begin <= end);
    assert(nfa->type == LBR_NFA_VERM);
    const struct lbr_verm *l = getImplNfa(nfa);

    if (begin == end) {
        return 0;
    }

    const u8 *ptr = vermicelliExec(l->c, 0, buf + begin, buf + end);
    if (ptr == buf + end) {
        DEBUG_PRINTF("no escape found\n");
        return 0;
    }

    assert(loc);
    *loc = (size_t)(ptr - buf);
    DEBUG_PRINTF("escape found at offset %zu\n", *loc);
    assert((char)*ptr == l->c);
    return 1;
}

static really_inline
char lbrFwdScanNVerm(const struct NFA *nfa, const u8 *buf,
                     size_t begin, size_t end, size_t *loc) {
    assert(begin <= end);
    assert(nfa->type == LBR_NFA_NVERM);
    const struct lbr_verm *l = getImplNfa(nfa);

    if (begin == end) {
        return 0;
    }

    const u8 *ptr = nvermicelliExec(l->c, 0, buf + begin, buf + end);
    if (ptr == buf + end) {
        DEBUG_PRINTF("no escape found\n");
        return 0;
    }

    assert(loc);
    *loc = (size_t)(ptr - buf);
    DEBUG_PRINTF("escape found at offset %zu\n", *loc);
    assert((char)*ptr != l->c);
    return 1;
}

static really_inline
char lbrFwdScanShuf(const struct NFA *nfa, const u8 *buf,
                    size_t begin, size_t end,
                    size_t *loc) {
    assert(begin <= end);
    assert(nfa->type == LBR_NFA_SHUF);
    const struct lbr_shuf *l = getImplNfa(nfa);

    if (begin == end) {
        return 0;
    }

    const u8 *ptr = shuftiExec(l->mask_lo, l->mask_hi, buf + begin, buf + end);
    if (ptr == buf + end) {
        DEBUG_PRINTF("no escape found\n");
        return 0;
    }

    assert(loc);
    *loc = (size_t)(ptr - buf);
    DEBUG_PRINTF("escape found at offset %zu\n", *loc);
    return 1;
}

static really_inline
char lbrFwdScanTruf(const struct NFA *nfa, const u8 *buf,
                    size_t begin, size_t end,
                    size_t *loc) {
    assert(begin <= end);
    assert(nfa->type == LBR_NFA_TRUF);
    const struct lbr_truf *l = getImplNfa(nfa);

    if (begin == end) {
        return 0;
    }

    const u8 *ptr = truffleExec(l->mask1, l->mask2, buf + begin, buf + end);
    if (ptr == buf + end) {
        DEBUG_PRINTF("no escape found\n");
        return 0;
    }

    assert(loc);
    *loc = (size_t)(ptr - buf);
    DEBUG_PRINTF("escape found at offset %zu\n", *loc);
    return 1;
}

#define ENGINE_ROOT_NAME Dot
#include "lbr_common_impl.h"

#define ENGINE_ROOT_NAME Verm
#include "lbr_common_impl.h"

#define ENGINE_ROOT_NAME NVerm
#include "lbr_common_impl.h"

#define ENGINE_ROOT_NAME Shuf
#include "lbr_common_impl.h"

#define ENGINE_ROOT_NAME Truf
#include "lbr_common_impl.h"
