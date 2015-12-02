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
 * \brief API for handling bounded repeats.
 *
 * This file provides an internal API for handling bounded repeats of character
 * classes. It is used by the Large Bounded Repeat (LBR) engine and by the
 * bounded repeat handling in the LimEx NFA engine as well.
 *
 * The state required by these functions is split into two regions:
 *
 * 1. Control block. This is a small structure (size varies with repeat mode)
 *    that may be copied around or compressed into stream state.
 * 2. Repeat state. This is a larger structure that can be quite big for large
 *    repeats, often containing a multibit ring or large vector of indices.
 *    This generally lives in stream state and is not copied.
 */

#ifndef REPEAT_H
#define REPEAT_H

#include "ue2common.h"
#include "repeat_internal.h"
#include "util/bitutils.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** Returns the offset of the most recent 'top' offset set in the repeat. */
static really_inline
u64a repeatLastTop(const struct RepeatInfo *info,
                   const union RepeatControl *ctrl, const void *state);

/** Returns the offset of the next match after 'offset', or zero if no further
 * matches are possible. */
static really_inline
u64a repeatNextMatch(const struct RepeatInfo *info,
                     const union RepeatControl *ctrl, const void *state,
                     u64a offset);

/** Stores a new top in the repeat. If is_alive is false, the repeat will be
 * initialised first and this top will become the first (and only) one. */
static really_inline
void repeatStore(const struct RepeatInfo *info, union RepeatControl *ctrl,
                 void *state, u64a offset, char is_alive);

/** Return type for repeatHasMatch. */
enum RepeatMatch {
    REPEAT_NOMATCH, /**< This offset is not a valid match. */
    REPEAT_MATCH, /**< This offset is a valid match. */
    REPEAT_STALE /**< This offset is not a valid match and no greater
                      offset will be (unless another top is stored). */
};

/** Query whether the repeat has a match at the given offset. Returns
 * ::REPEAT_STALE if it does not have a match at that offset _and_
 * no further matches are possible. */
static really_inline
enum RepeatMatch repeatHasMatch(const struct RepeatInfo *info,
                                const union RepeatControl *ctrl,
                                const void *state, u64a offset);

/** \brief Serialize a packed version of the repeat control block into stream
 * state. */
void repeatPack(char *dest, const struct RepeatInfo *info,
                const union RepeatControl *ctrl, u64a offset);

/** \brief Deserialize a packed version of the repeat control block. */
void repeatUnpack(const char *src, const struct RepeatInfo *info, u64a offset,
                  union RepeatControl *ctrl);

////
//// IMPLEMENTATION.
////

u64a repeatLastTopRing(const struct RepeatInfo *info,
                       const union RepeatControl *ctrl);

u64a repeatLastTopRange(const union RepeatControl *ctrl,
                        const void *state);

u64a repeatLastTopBitmap(const union RepeatControl *ctrl);

u64a repeatLastTopTrailer(const struct RepeatInfo *info,
                          const union RepeatControl *ctrl);

u64a repeatLastTopSparseOptimalP(const struct RepeatInfo *info,
                                 const union RepeatControl *ctrl,
                                 const void *state);

static really_inline
u64a repeatLastTop(const struct RepeatInfo *info,
                   const union RepeatControl *ctrl, const void *state) {
    assert(info && ctrl && state);

    switch ((enum RepeatType)info->type) {
    case REPEAT_RING:
        return repeatLastTopRing(info, ctrl);
    case REPEAT_FIRST:
    case REPEAT_LAST:
        return ctrl->offset.offset;
    case REPEAT_RANGE:
        return repeatLastTopRange(ctrl, state);
    case REPEAT_BITMAP:
        return repeatLastTopBitmap(ctrl);
    case REPEAT_SPARSE_OPTIMAL_P:
        return repeatLastTopSparseOptimalP(info, ctrl, state);
    case REPEAT_TRAILER:
        return repeatLastTopTrailer(info, ctrl);
    case REPEAT_ALWAYS:
        return 0;
    }

    DEBUG_PRINTF("bad repeat type %u\n", info->type);
    assert(0);
    return 0;
}

// Used for both FIRST and LAST models.
static really_inline
u64a repeatNextMatchOffset(const struct RepeatInfo *info,
                           const union RepeatControl *ctrl, u64a offset) {
    u64a first = ctrl->offset.offset + info->repeatMin;
    if (offset < first) {
        return first;
    }

    if (info->repeatMax == REPEAT_INF ||
        offset < ctrl->offset.offset + info->repeatMax) {
        return offset + 1;
    }

    return 0; // No more matches.
}

u64a repeatNextMatchRing(const struct RepeatInfo *info,
                         const union RepeatControl *ctrl,
                         const void *state, u64a offset);

u64a repeatNextMatchRange(const struct RepeatInfo *info,
                          const union RepeatControl *ctrl,
                          const void *state, u64a offset);

u64a repeatNextMatchBitmap(const struct RepeatInfo *info,
                           const union RepeatControl *ctrl, u64a offset);

u64a repeatNextMatchSparseOptimalP(const struct RepeatInfo *info,
                                   const union RepeatControl *ctrl,
                                   const void *state, u64a offset);

u64a repeatNextMatchTrailer(const struct RepeatInfo *info,
                            const union RepeatControl *ctrl, u64a offset);

static really_inline
u64a repeatNextMatch(const struct RepeatInfo *info,
                     const union RepeatControl *ctrl, const void *state,
                     u64a offset) {
    assert(info && ctrl && state);
    assert(ISALIGNED(info));
    assert(ISALIGNED(ctrl));

    switch ((enum RepeatType)info->type) {
    case REPEAT_RING:
        return repeatNextMatchRing(info, ctrl, state, offset);
    case REPEAT_FIRST:
    // fall through
    case REPEAT_LAST:
        return repeatNextMatchOffset(info, ctrl, offset);
    case REPEAT_RANGE:
        return repeatNextMatchRange(info, ctrl, state, offset);
    case REPEAT_BITMAP:
        return repeatNextMatchBitmap(info, ctrl, offset);
    case REPEAT_SPARSE_OPTIMAL_P:
        return repeatNextMatchSparseOptimalP(info, ctrl, state, offset);
    case REPEAT_TRAILER:
        return repeatNextMatchTrailer(info, ctrl, offset);
    case REPEAT_ALWAYS:
        return offset + 1;
    }

    DEBUG_PRINTF("bad repeat type %u\n", info->type);
    assert(0);
    return 0;
}

static really_inline
void repeatStoreFirst(union RepeatControl *ctrl, u64a offset,
                      char is_alive) {
    if (is_alive) {
        return;
    }
    ctrl->offset.offset = offset;
}

static really_inline
void repeatStoreLast(union RepeatControl *ctrl, u64a offset,
                     UNUSED char is_alive) {
    assert(!is_alive || offset >= ctrl->offset.offset);
    ctrl->offset.offset = offset;
}

void repeatStoreRing(const struct RepeatInfo *info,
                     union RepeatControl *ctrl, void *state, u64a offset,
                     char is_alive);

void repeatStoreRange(const struct RepeatInfo *info,
                      union RepeatControl *ctrl, void *state, u64a offset,
                      char is_alive);

void repeatStoreBitmap(const struct RepeatInfo *info,
                       union RepeatControl *ctrl, u64a offset,
                       char is_alive);

void repeatStoreSparseOptimalP(const struct RepeatInfo *info,
                               union RepeatControl *ctrl, void *state,
                               u64a offset, char is_alive);

void repeatStoreTrailer(const struct RepeatInfo *info,
                        union RepeatControl *ctrl, u64a offset,
                        char is_alive);

static really_inline
void repeatStore(const struct RepeatInfo *info, union RepeatControl *ctrl,
                 void *state, u64a offset, char is_alive) {
    assert(info && ctrl && state);
    assert(ISALIGNED(info));
    assert(ISALIGNED(ctrl));

    assert(info->repeatMin <= info->repeatMax);
    assert(info->repeatMax <= REPEAT_INF);

    switch ((enum RepeatType)info->type) {
    case REPEAT_RING:
        repeatStoreRing(info, ctrl, state, offset, is_alive);
        break;
    case REPEAT_FIRST:
        repeatStoreFirst(ctrl, offset, is_alive);
        break;
    case REPEAT_LAST:
        repeatStoreLast(ctrl, offset, is_alive);
        break;
    case REPEAT_RANGE:
        repeatStoreRange(info, ctrl, state, offset, is_alive);
        break;
    case REPEAT_BITMAP:
        repeatStoreBitmap(info, ctrl, offset, is_alive);
        break;
    case REPEAT_SPARSE_OPTIMAL_P:
        repeatStoreSparseOptimalP(info, ctrl, state, offset, is_alive);
        break;
    case REPEAT_TRAILER:
        repeatStoreTrailer(info, ctrl, offset, is_alive);
        break;
    case REPEAT_ALWAYS:
        /* nothing to do - no state */
        break;
    }
}

static really_inline
enum RepeatMatch repeatHasMatchFirst(const struct RepeatInfo *info,
                                     const union RepeatControl *ctrl,
                                     u64a offset) {
    if (offset < ctrl->offset.offset + info->repeatMin) {
        return REPEAT_NOMATCH;
    }

    // FIRST models are {N,} repeats, i.e. they always have inf max depth.
    assert(info->repeatMax == REPEAT_INF);
    return REPEAT_MATCH;
}

static really_inline
enum RepeatMatch repeatHasMatchLast(const struct RepeatInfo *info,
                                    const union RepeatControl *ctrl,
                                    u64a offset) {
    if (offset < ctrl->offset.offset + info->repeatMin) {
        return REPEAT_NOMATCH;
    }
    assert(info->repeatMax < REPEAT_INF);
    if (offset <= ctrl->offset.offset + info->repeatMax) {
        return REPEAT_MATCH;
    }
    return REPEAT_STALE;
}

enum RepeatMatch repeatHasMatchRing(const struct RepeatInfo *info,
                                    const union RepeatControl *ctrl,
                                    const void *state, u64a offset);

enum RepeatMatch repeatHasMatchRange(const struct RepeatInfo *info,
                                     const union RepeatControl *ctrl,
                                     const void *state, u64a offset);

enum RepeatMatch repeatHasMatchSparseOptimalP(const struct RepeatInfo *info,
                                              const union RepeatControl *ctrl,
                                              const void *state, u64a offset);

enum RepeatMatch repeatHasMatchBitmap(const struct RepeatInfo *info,
                                      const union RepeatControl *ctrl,
                                      u64a offset);

enum RepeatMatch repeatHasMatchTrailer(const struct RepeatInfo *info,
                                       const union RepeatControl *ctrl,
                                       u64a offset);

static really_inline
enum RepeatMatch repeatHasMatch(const struct RepeatInfo *info,
                                const union RepeatControl *ctrl,
                                const void *state, u64a offset) {
    assert(info && ctrl && state);
    assert(ISALIGNED(info));
    assert(ISALIGNED(ctrl));

    switch ((enum RepeatType)info->type) {
    case REPEAT_RING:
        return repeatHasMatchRing(info, ctrl, state, offset);
    case REPEAT_FIRST:
        return repeatHasMatchFirst(info, ctrl, offset);
    case REPEAT_LAST:
        return repeatHasMatchLast(info, ctrl, offset);
    case REPEAT_RANGE:
        return repeatHasMatchRange(info, ctrl, state, offset);
    case REPEAT_BITMAP:
        return repeatHasMatchBitmap(info, ctrl, offset);
    case REPEAT_SPARSE_OPTIMAL_P:
        return repeatHasMatchSparseOptimalP(info, ctrl, state, offset);
    case REPEAT_TRAILER:
        return repeatHasMatchTrailer(info, ctrl, offset);
    case REPEAT_ALWAYS:
        return REPEAT_MATCH;
    }

    assert(0);
    return REPEAT_NOMATCH;
}

#ifdef __cplusplus
}
#endif

#endif // REPEAT_H
