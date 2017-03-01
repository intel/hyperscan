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
 * \brief API for handling bounded repeats.
 *
 * This file provides an internal API for handling bounded repeats of character
 * classes. It is used by the Large Bounded Repeat (LBR) engine and by the
 * bounded repeat handling in the LimEx NFA engine as well.
 */
#include "repeat.h"
#include "util/bitutils.h"
#include "util/multibit.h"
#include "util/pack_bits.h"
#include "util/partial_store.h"
#include "util/unaligned.h"

#include <stdint.h>
#include <string.h>

/** \brief Returns the total capacity of the ring.
 * Note that it's currently one greater than repeatMax so that we can handle
 * cases where the tug and pos triggers overlap. */
static
u32 ringCapacity(const struct RepeatInfo *info) {
    return info->repeatMax + 1;
}

/** \brief Returns the number of elements currently in the ring. Note that if
 * the first and last indices are equal, the ring is full. */
static
u32 ringOccupancy(const struct RepeatRingControl *xs, const u32 ringSize) {
    if (xs->last > xs->first) {
        return xs->last - xs->first;
    } else { // wrapped
        return ringSize - (xs->first - xs->last);
    }
}

/** \brief Returns the offset of the _last_ top stored in the ring. */
static
u64a ringLastTop(const struct RepeatRingControl *xs, const u32 ringSize) {
    return xs->offset + ringOccupancy(xs, ringSize) - 1;
}

#if !defined(NDEBUG) || defined(DUMP_SUPPORT)
/** \brief For debugging: returns the total capacity of the range list. */
static UNUSED
u32 rangeListCapacity(const struct RepeatInfo *info) {
    u32 d = info->repeatMax - info->repeatMin;
    assert(d > 0); // should be in a RING model!
    return 2 * ((info->repeatMax / d) + 1);
}
#endif

#ifdef DEBUG
static
void dumpRing(const struct RepeatInfo *info, const struct RepeatRingControl *xs,
              const u8 *ring) {
    const u32 ringSize = ringCapacity(info);
    DEBUG_PRINTF("ring (occ %u/%u, %u->%u): ", ringOccupancy(xs, ringSize),
                 ringSize, xs->first, xs->last);

    u16 i = xs->first, n = 0;
    do {
        if (mmbit_isset(ring, ringSize, i)) {
            u64a ringOffset = xs->offset + n;
            printf("%llu ", ringOffset);
        }
        ++i, ++n;
        if (i == ringSize) {
            i = 0;
        }
    } while (i != xs->last);
    printf("\n");
}

static
void dumpRange(const struct RepeatInfo *info,
               const struct RepeatRangeControl *xs, const u16 *ring) {
    const u32 ringSize = rangeListCapacity(info);
    DEBUG_PRINTF("ring (occ %u/%u): ", xs->num, ringSize);

    if (xs->num) {
        for (u32 i = 0; i < xs->num; i++) {
            printf("%llu ", xs->offset + unaligned_load_u16(ring + i));
        }
    } else {
        printf("empty");
    }
    printf("\n");
}

static
void dumpBitmap(const struct RepeatBitmapControl *xs) {
    DEBUG_PRINTF("bitmap (base=%llu): ", xs->offset);
    u64a bitmap = xs->bitmap;
    while (bitmap) {
        printf("%llu ", xs->offset + findAndClearLSB_64(&bitmap));
    }
    printf("\n");
}

static
void dumpTrailer(const struct RepeatInfo *info,
                 const struct RepeatTrailerControl *xs) {
    const u64a m_width = info->repeatMax - info->repeatMin;
    DEBUG_PRINTF("trailer: current extent is [%llu,%llu]", xs->offset,
                 xs->offset + m_width);
    u64a bitmap = xs->bitmap;
    if (bitmap) {
        printf(", also matches at: ");
        while (bitmap) {
            u32 idx = findAndClearMSB_64(&bitmap);
            printf("%llu ", xs->offset - idx - 1);
        }
    } else {
        printf(", no earlier matches");
    }
    printf("\n");
}

#endif // DEBUG

#ifndef NDEBUG
/** \brief For debugging: returns true if the range is ordered with no dupes. */
static UNUSED
int rangeListIsOrdered(const struct RepeatRangeControl *xs, const u16 *ring) {
    for (u32 i = 1; i < xs->num; i++) {
        u16 a = unaligned_load_u16(ring + i - 1);
        u16 b = unaligned_load_u16(ring + i);
        if (a >= b) {
            return 0;
        }
    }
    return 1;
}
#endif

u64a repeatLastTopRing(const struct RepeatInfo *info,
                       const union RepeatControl *ctrl) {
    const u32 ringSize = ringCapacity(info);
    return ringLastTop(&ctrl->ring, ringSize);
}

u64a repeatLastTopRange(const union RepeatControl *ctrl, const void *state) {
    const u16 *ring = (const u16 *)state;
    const struct RepeatRangeControl *xs = &ctrl->range;
    assert(xs->num);
    return xs->offset + unaligned_load_u16(ring + xs->num - 1);
}

u64a repeatLastTopBitmap(const union RepeatControl *ctrl) {
    const struct RepeatBitmapControl *xs = &ctrl->bitmap;
    if (!xs->bitmap) {
        /* last top was too long ago */
        return 0;
    }
    return xs->offset + 63 - clz64(xs->bitmap);
}

u64a repeatLastTopTrailer(const struct RepeatInfo *info,
                          const union RepeatControl *ctrl) {
    const struct RepeatTrailerControl *xs = &ctrl->trailer;
    assert(xs->offset >= info->repeatMin);
    return xs->offset - info->repeatMin;
}

u64a repeatNextMatchRing(const struct RepeatInfo *info,
                         const union RepeatControl *ctrl, const void *state,
                         u64a offset) {
    const struct RepeatRingControl *xs = &ctrl->ring;
    const u8 *ring = (const u8 *)state;
    const u32 ringSize = ringCapacity(info);

    // We should have at least one top stored.
    assert(mmbit_any(ring, ringSize));
    assert(info->repeatMax < REPEAT_INF);

    // Increment offset, as we want the NEXT match.
    offset++;

    const u64a base_offset = xs->offset;
    DEBUG_PRINTF("offset=%llu, base_offset=%llu\n", offset, base_offset);

    u64a delta = offset - base_offset;
    if (offset < base_offset || delta < info->repeatMin) {
        DEBUG_PRINTF("before min repeat\n");
        return base_offset + info->repeatMin;
    }
    if (offset > ringLastTop(xs, ringSize) + info->repeatMax) {
        DEBUG_PRINTF("ring is stale\n");
        return 0; // no more matches
    }

    DEBUG_PRINTF("delta=%llu\n", delta);
    u64a lower = delta > info->repeatMax ? delta - info->repeatMax : 0;
    DEBUG_PRINTF("lower=%llu\n", lower);

    assert(lower < ringSize);

    // First scan, either to xs->last if there's no wrap-around or ringSize
    // (end of the underlying multibit) if we are wrapping.

    u32 begin = xs->first + lower;
    if (begin >= ringSize) {
        // This branch and sub tested a lot faster than using % (integer div).
        begin -= ringSize;
    }
    const u32 end = begin >= xs->last ? ringSize : xs->last;
    u32 i = mmbit_iterate_bounded(ring, ringSize, begin, end);
    if (i != MMB_INVALID) {
        u32 j = i - begin + lower;
        return MAX(offset, base_offset + j + info->repeatMin);
    }

    // A second scan is necessary if we need to cope with wrap-around in the
    // ring buffer.

    if (begin >= xs->last) {
        i = mmbit_iterate_bounded(ring, ringSize, 0, xs->last);
        if (i != MMB_INVALID) {
            u32 j = i + (ringSize - begin) + lower;
            return MAX(offset, base_offset + j + info->repeatMin);
        }
    }

    return 0;
}

u64a repeatNextMatchRange(const struct RepeatInfo *info,
                          const union RepeatControl *ctrl, const void *state,
                          u64a offset) {
    const struct RepeatRangeControl *xs = &ctrl->range;
    const u16 *ring = (const u16 *)state;

    assert(xs->num > 0);
    assert(xs->num <= rangeListCapacity(info));
    assert(rangeListIsOrdered(xs, ring));
    assert(info->repeatMax < REPEAT_INF);

    for (u32 i = 0; i < xs->num; i++) {
        u64a base = xs->offset + unaligned_load_u16(ring + i);
        u64a first = base + info->repeatMin;
        if (offset < first) {
            return first;
        }
        if (offset < base + info->repeatMax) {
            return offset + 1;
        }
    }

    return 0;
}

u64a repeatNextMatchBitmap(const struct RepeatInfo *info,
                           const union RepeatControl *ctrl, u64a offset) {
    const struct RepeatBitmapControl *xs = &ctrl->bitmap;
    const u64a base = xs->offset;
    u64a bitmap = xs->bitmap;

    // FIXME: quick exit if there is no match, based on last top in bitmap?

    while (bitmap) {
        u64a top = base + findAndClearLSB_64(&bitmap);
        if (offset < top + info->repeatMin) {
            return top + info->repeatMin;
        }
        if (offset < top + info->repeatMax) {
            return offset + 1;
        }
    }

    return 0; // No more matches.
}

u64a repeatNextMatchTrailer(const struct RepeatInfo *info,
                            const union RepeatControl *ctrl, u64a offset) {
    const struct RepeatTrailerControl *xs = &ctrl->trailer;
    const u32 m_width = info->repeatMax - info->repeatMin;

    DEBUG_PRINTF("offset=%llu, xs->offset=%llu\n", offset, xs->offset);
    DEBUG_PRINTF("{%u,%u} repeat, m_width=%u\n", info->repeatMin,
                 info->repeatMax, m_width);

    assert(xs->offset >= info->repeatMin);

    if (offset >= xs->offset + m_width) {
        DEBUG_PRINTF("no more matches\n");
        return 0;
    }

    if (offset >= xs->offset) {
        DEBUG_PRINTF("inside most recent match window, next match %llu\n",
                     offset + 1);
        return offset + 1;
    }

    // Offset is before the match window, we need to consult the bitmap of
    // earlier match offsets.
    u64a bitmap = xs->bitmap;

    u64a diff = xs->offset - offset;
    DEBUG_PRINTF("diff=%llu\n", diff);
    if (diff <= 64) {
        assert(diff);
        bitmap &= (1ULL << (diff - 1)) - 1;
    }
    DEBUG_PRINTF("bitmap = 0x%llx\n", bitmap);
    if (bitmap) {
        u32 idx = 63 - clz64(bitmap);
        DEBUG_PRINTF("clz=%u, idx = %u -> offset %llu\n", clz64(bitmap), idx,
                     xs->offset - idx);
        DEBUG_PRINTF("next match at %llu\n", xs->offset - idx - 1);
        u64a next_match = xs->offset - idx - 1;
        assert(next_match > offset);
        return next_match;
    }

    DEBUG_PRINTF("next match is start of match window, %llu\n", xs->offset);
    return xs->offset;
}

/** \brief Store the first top in the ring buffer. */
static
void storeInitialRingTop(struct RepeatRingControl *xs, u8 *ring,
                         u64a offset, const u32 ringSize) {
    DEBUG_PRINTF("ring=%p, ringSize=%u\n", ring, ringSize);
    xs->offset = offset;
    mmbit_clear(ring, ringSize);
    mmbit_set(ring, ringSize, 0);
    xs->first = 0;
    xs->last = 1;
}

static really_inline
char ringIsStale(const struct RepeatRingControl *xs, const u32 ringSize,
                 const u64a offset) {
    u64a finalMatch = ringLastTop(xs, ringSize);
    if (offset - finalMatch >= ringSize) {
        DEBUG_PRINTF("all matches in ring are stale\n");
        return 1;
    }

    return 0;
}

void repeatStoreRing(const struct RepeatInfo *info, union RepeatControl *ctrl,
                     void *state, u64a offset, char is_alive) {
    struct RepeatRingControl *xs = &ctrl->ring;
    u8 *ring = (u8 *)state;
    const u32 ringSize = ringCapacity(info);
    assert(ringSize > 0);

    DEBUG_PRINTF("storing top for offset %llu in ring\n", offset);

    if (!is_alive || ringIsStale(xs, ringSize, offset)) {
        storeInitialRingTop(xs, ring, offset, ringSize);
    } else {
        assert(offset > ringLastTop(xs, ringSize)); // Dupe or out of order.
        u32 occ = ringOccupancy(xs, ringSize);
        u64a diff = offset - xs->offset;
        DEBUG_PRINTF("diff=%llu, occ=%u\n", diff, occ);
        if (diff >= ringSize) {
            u32 push = diff - ringSize + 1;
            DEBUG_PRINTF("push ring %u\n", push);
            xs->first += push;
            if (xs->first >= ringSize) {
                xs->first -= ringSize;
            }
            xs->offset += push;
            diff -= push;
            occ -= push;
        }

        // There's now room in the ring for this top, so we write a run of
        // zeroes, then a one.
        DEBUG_PRINTF("diff=%llu, occ=%u\n", diff, occ);
        assert(diff < ringSize);
        assert(diff >= occ);
        u32 n = diff - occ;

        u32 i = xs->last + n;

        mmbit_unset_range(ring, ringSize, xs->last, MIN(i, ringSize));
        if (i >= ringSize) {
            i -= ringSize;
            mmbit_unset_range(ring, ringSize, 0, i);
        }

        assert(i != xs->first);
        DEBUG_PRINTF("set bit %u\n", i);
        mmbit_set(ring, ringSize, i);
        xs->last = i + 1;
        if (xs->last == ringSize) {
            xs->last = 0;
        }
    }

    // Our ring indices shouldn't have spiraled off into uncharted space.
    assert(xs->first < ringSize);
    assert(xs->last < ringSize);

#ifdef DEBUG
    DEBUG_PRINTF("post-store ring state\n");
    dumpRing(info, xs, ring);
#endif

    // The final top stored in our ring should be the one we just wrote in.
    assert(ringLastTop(xs, ringSize) == offset);
}

static really_inline
void storeInitialRangeTop(struct RepeatRangeControl *xs, u16 *ring,
                          u64a offset) {
    xs->offset = offset;
    xs->num = 1;
    unaligned_store_u16(ring, 0);
}

void repeatStoreRange(const struct RepeatInfo *info, union RepeatControl *ctrl,
                      void *state, u64a offset, char is_alive) {
    struct RepeatRangeControl *xs = &ctrl->range;
    u16 *ring = (u16 *)state;

    if (!is_alive) {
        DEBUG_PRINTF("storing initial top at %llu\n", offset);
        storeInitialRangeTop(xs, ring, offset);
        return;
    }

    DEBUG_PRINTF("storing top at %llu, list currently has %u/%u elements\n",
                 offset, xs->num, rangeListCapacity(info));

#ifdef DEBUG
    dumpRange(info, xs, ring);
#endif

    // Walk ring from front. Identify the number of stale elements, and shift
    // the whole ring to delete them.
    u32 i = 0;
    for (; i < xs->num; i++) {
        u64a this_offset = xs->offset + unaligned_load_u16(ring + i);
        DEBUG_PRINTF("this_offset=%llu, diff=%llu\n", this_offset,
                     offset - this_offset);
        if (offset - this_offset <= info->repeatMax) {
            break;
        }
    }

    if (i == xs->num) {
        DEBUG_PRINTF("whole ring is stale\n");
        storeInitialRangeTop(xs, ring, offset);
        return;
    } else if (i > 0) {
        DEBUG_PRINTF("expiring %u stale tops\n", i);
        u16 first_offset = unaligned_load_u16(ring + i); // first live top
        for (u32 j = 0; j < xs->num - i; j++) {
            u16 val = unaligned_load_u16(ring + i + j);
            assert(val >= first_offset);
            unaligned_store_u16(ring + j, val - first_offset);
        }
        xs->offset += first_offset;
        xs->num -= i;
    }

#ifdef DEBUG
    DEBUG_PRINTF("post-expire:\n");
    dumpRange(info, xs, ring);
#endif

    if (xs->num == 1) {
        goto append;
    }

    // Let d = repeatMax - repeatMin
    // Examine penultimate entry x[-2].
    // If (offset - x[-2] <= d), then last entry x[-1] can be replaced with
    // entry for offset.
    assert(xs->num >= 2);
    u32 d = info->repeatMax - info->repeatMin;
    u64a penultimate_offset =
        xs->offset + unaligned_load_u16(ring + xs->num - 2);
    if (offset - penultimate_offset <= d) {
        assert(offset - xs->offset <= (u16)-1);
        unaligned_store_u16(ring + xs->num - 1, offset - xs->offset);
        goto done;
    }

    // Otherwise, write a new entry for offset and return.

append:
    assert(offset - xs->offset <= (u16)-1);
    assert(xs->num < rangeListCapacity(info));
    unaligned_store_u16(ring + xs->num, offset - xs->offset);
    xs->num++;

done:
    assert(rangeListIsOrdered(xs, ring));
}

void repeatStoreBitmap(const struct RepeatInfo *info, union RepeatControl *ctrl,
                       u64a offset, char is_alive) {
    DEBUG_PRINTF("{%u,%u} repeat, storing top at %llu\n", info->repeatMin,
                 info->repeatMax, offset);

    struct RepeatBitmapControl *xs = &ctrl->bitmap;
    if (!is_alive || !xs->bitmap) {
        DEBUG_PRINTF("storing initial top at %llu\n", offset);
        xs->offset = offset;
        xs->bitmap = 1U;
        return;
    }

#ifdef DEBUG
    DEBUG_PRINTF("pre-store:\n");
    dumpBitmap(xs);
#endif

    assert(offset >= xs->offset);

    u64a last_top = xs->offset + 63 - clz64(xs->bitmap);
    if (offset > last_top + info->repeatMax) {
        DEBUG_PRINTF("bitmap stale, storing initial top\n");
        xs->offset = offset;
        xs->bitmap = 1U;
        return;
    }

    u64a diff = offset - xs->offset;
    if (diff >= info->repeatMax + 1) {
        DEBUG_PRINTF("need expire, diff=%llu\n", diff);
        u64a push = diff - info->repeatMax;
        xs->offset += push;
        xs->bitmap = push >= 64 ? 0 : xs->bitmap >> push;
        DEBUG_PRINTF("pushed xs->offset to %llu\n", xs->offset);
    }

    // Write a new entry.
    diff = offset - xs->offset;
    assert(diff < 64);
    xs->bitmap |= (1ULL << diff);

#ifdef DEBUG
    DEBUG_PRINTF("post-store:\n");
    dumpBitmap(xs);
#endif
}

/** \brief Returns 1 if the ring has a match between (logical) index \a lower
 * and \a upper, excluding \a upper. */
static
int ringHasMatch(const struct RepeatRingControl *xs, const u8 *ring,
                 const u32 ringSize, u32 lower, u32 upper) {
    assert(lower < upper);
    assert(lower < ringSize);
    assert(upper <= ringSize);

    u32 i = xs->first + lower;
    if (i >= ringSize) {
        i -= ringSize;
    }

    // Performance tweak: if we're looking at a fixed repeat, we can just use
    // mmbit_isset.
    if (lower + 1 == upper) {
        return mmbit_isset(ring, ringSize, i);
    }

    u32 end = xs->first + upper;
    if (end >= ringSize) {
        end -= ringSize;
    }

    // First scan, either to end if there's no wrap-around or ringSize (end of
    // the underlying multibit) if we are wrapping.

    u32 scan_end = i < end ? end : ringSize;
    u32 m = mmbit_iterate_bounded(ring, ringSize, i, scan_end);
    if (m != MMB_INVALID) {
        return 1;
    }

    // A second scan is necessary if we need to cope with wrap-around in the
    // ring buffer.

    if (i >= end) {
        m = mmbit_iterate_bounded(ring, ringSize, 0, end);
        return m != MMB_INVALID;
    }

    return 0;
}

/** Return a mask of ones in bit positions [0..v]. */
static really_inline
u64a mask_ones_to(u32 v) {
    if (v < 63) {
        return (1ULL << (v + 1)) - 1;
    } else {
        return ~(0ULL);
    }
}

void repeatStoreTrailer(const struct RepeatInfo *info,
                        union RepeatControl *ctrl, u64a offset, char is_alive) {
    DEBUG_PRINTF("{%u,%u} repeat, top at %llu\n", info->repeatMin,
                 info->repeatMax, offset);

    struct RepeatTrailerControl *xs = &ctrl->trailer;

    /* The TRAILER repeat model stores the following data in its control block:
     *
     *   1. offset, which is the min extent of the most recent match window
     *      (i.e. corresponding to the most recent top)
     *   2. bitmap, which is a bitmap of up to repeatMin matches before
     *      the min extent offset.
     */

    const u64a next_extent = offset + info->repeatMin;

    if (!is_alive) {
        xs->offset = next_extent;
        xs->bitmap = 0;
        DEBUG_PRINTF("initial top, set extent to %llu\n", next_extent);
        return;
    }

#ifdef DEBUG
    DEBUG_PRINTF("pre-store:\n");
    dumpTrailer(info, xs);
#endif

    const u32 m_width = info->repeatMax - info->repeatMin;
    DEBUG_PRINTF("most recent match window is [%llu,%llu]\n", xs->offset,
                 xs->offset + m_width);

    assert(next_extent > xs->offset);
    u64a diff = next_extent - xs->offset;
    DEBUG_PRINTF("diff=%llu, m_width=%u\n", diff, m_width);

    assert(diff);
    xs->bitmap = diff < 64 ? xs->bitmap << diff : 0;

    // Switch on bits in the bitmask corresponding to matches in the previous
    // match window.
    if (diff <= m_width) {
        u64a m = mask_ones_to(diff - 1);
        xs->bitmap |= m;
    } else {
        u64a shift = diff - m_width - 1;
        if (shift < 64) {
            u64a m = mask_ones_to(m_width);
            m <<= shift;
            xs->bitmap |= m;
        }
    }

    DEBUG_PRINTF("bitmap=0x%llx\n", xs->bitmap);

    // Update max extent.
    xs->offset = next_extent;

    // Trim stale history: we only need repeatMin bytes of history.
    if (info->repeatMin < 63) {
        u64a mask = (1ULL << (info->repeatMin + 1)) - 1;
        xs->bitmap &= mask;
    }

#ifdef DEBUG
    DEBUG_PRINTF("post-store:\n");
    dumpTrailer(info, xs);
#endif
}

enum RepeatMatch repeatHasMatchRing(const struct RepeatInfo *info,
                                    const union RepeatControl *ctrl,
                                    const void *state, u64a offset) {
    const struct RepeatRingControl *xs = &ctrl->ring;
    const u8 *ring = (const u8 *)state;
    const u32 ringSize = ringCapacity(info);

    assert(mmbit_any(ring, ringSize));
    assert(offset >= xs->offset);

    DEBUG_PRINTF("check: offset=%llu, repeat=[%u,%u]\n", offset,
                 info->repeatMin, info->repeatMax);
#ifdef DEBUG
    DEBUG_PRINTF("ring state\n");
    dumpRing(info, xs, ring);
#endif

    if (offset - xs->offset < info->repeatMin) {
        DEBUG_PRINTF("haven't even seen repeatMin bytes yet!\n");
        return REPEAT_NOMATCH;
    }

    if (offset - ringLastTop(xs, ringSize) >= ringSize) {
        DEBUG_PRINTF("ring is stale\n");
        return REPEAT_STALE;
    }

    // If we're not stale, delta fits in the range [repeatMin, lastTop +
    // repeatMax], which fits in a u32.
    assert(offset - xs->offset < UINT32_MAX);
    u32 delta = (u32)(offset - xs->offset);
    DEBUG_PRINTF("delta=%u\n", delta);

    // Find the bounds on possible matches in the ring buffer.
    u32 lower = delta > info->repeatMax ? delta - info->repeatMax : 0;
    u32 upper = MIN(delta - info->repeatMin + 1, ringOccupancy(xs, ringSize));

    if (lower >= upper) {
        DEBUG_PRINTF("no matches to check\n");
        return REPEAT_NOMATCH;
    }

    DEBUG_PRINTF("possible match indices=[%u,%u]\n", lower, upper);
    if (ringHasMatch(xs, ring, ringSize, lower, upper)) {
        return REPEAT_MATCH;
    }

    return REPEAT_NOMATCH;
}

enum RepeatMatch repeatHasMatchRange(const struct RepeatInfo *info,
                                     const union RepeatControl *ctrl,
                                     const void *state, u64a offset) {
    const struct RepeatRangeControl *xs = &ctrl->range;
    const u16 *ring = (const u16 *)state;

    assert(xs->num > 0);
    assert(xs->num <= rangeListCapacity(info));
    assert(rangeListIsOrdered(xs, ring));

    // Walk the ring. For each entry x:
    //   if (offset - x) falls inside repeat bounds, return success.

    // It may be worth doing tests on first and last elements first to bail
    // early if the whole ring is too young or stale.

    DEBUG_PRINTF("check %u (of %u) elements, offset %llu, bounds={%u,%u}\n",
                 xs->num, rangeListCapacity(info), offset,
                 info->repeatMin, info->repeatMax);
#ifdef DEBUG
    dumpRange(info, xs, ring);
#endif

    // Quick pre-check for minimum.
    assert(offset >= xs->offset);
    if (offset - xs->offset < info->repeatMin) {
        DEBUG_PRINTF("haven't even seen repeatMin bytes yet!\n");
        return REPEAT_NOMATCH;
    }

    // We check the most recent offset first, as we can establish staleness.
    u64a match = xs->offset + unaligned_load_u16(ring + xs->num - 1);
    assert(offset >= match);
    u64a diff = offset - match;
    if (diff > info->repeatMax) {
        DEBUG_PRINTF("range list is stale\n");
        return REPEAT_STALE;
    } else if (diff >= info->repeatMin && diff <= info->repeatMax) {
        return REPEAT_MATCH;
    }

    // Check the other offsets in the list.
    u32 count = xs->num - 1;
    for (u32 i = 0; i < count; i++) {
        match = xs->offset + unaligned_load_u16(ring + i);
        assert(offset >= match);
        diff = offset - match;
        if (diff >= info->repeatMin && diff <= info->repeatMax) {
            return REPEAT_MATCH;
        }
    }

    return REPEAT_NOMATCH;
}

enum RepeatMatch repeatHasMatchBitmap(const struct RepeatInfo *info,
                                      const union RepeatControl *ctrl,
                                      u64a offset) {
    const struct RepeatBitmapControl *xs = &ctrl->bitmap;

    DEBUG_PRINTF("checking if offset=%llu is a match\n", offset);

#ifdef DEBUG
    dumpBitmap(xs);
#endif

    u64a bitmap = xs->bitmap;
    if (!bitmap) {
        DEBUG_PRINTF("no tops; stale\n");
        return REPEAT_STALE;
    }

    // Quick pre-check for minimum.
    const u64a base = xs->offset;
    assert(offset >= base);
    if (offset - base < info->repeatMin) {
        DEBUG_PRINTF("haven't even seen repeatMin bytes yet!\n");
        return REPEAT_NOMATCH;
    }

    // We check the most recent offset first, as we can establish staleness.
    u64a match = base + findAndClearMSB_64(&bitmap);
    DEBUG_PRINTF("offset=%llu, last_match %llu\n", offset, match);
    assert(offset >= match);
    u64a diff = offset - match;
    if (diff > info->repeatMax) {
        DEBUG_PRINTF("stale\n");
        return REPEAT_STALE;
    } else if (diff >= info->repeatMin && diff <= info->repeatMax) {
        return REPEAT_MATCH;
    }

    while (bitmap) {
        match = base + findAndClearLSB_64(&bitmap);
        DEBUG_PRINTF("offset=%llu, last_match %llu\n", offset, match);
        assert(offset >= match);
        diff = offset - match;
        if (diff >= info->repeatMin && diff <= info->repeatMax) {
            return REPEAT_MATCH;
        }
    }

    return REPEAT_NOMATCH;
}

enum RepeatMatch repeatHasMatchTrailer(const struct RepeatInfo *info,
                                       const union RepeatControl *ctrl,
                                       u64a offset) {
    const struct RepeatTrailerControl *xs = &ctrl->trailer;
    const u32 m_width = info->repeatMax - info->repeatMin;

    DEBUG_PRINTF("offset=%llu, xs->offset=%llu, xs->bitmap=0x%llx\n", offset,
                 xs->offset, xs->bitmap);

    if (offset > xs->offset + m_width) {
        DEBUG_PRINTF("stale\n");
        return REPEAT_STALE;
    }

    if (offset >= xs->offset) {
        DEBUG_PRINTF("in match window\n");
        return REPEAT_MATCH;
    }

    if (offset >= xs->offset - info->repeatMin) {
        u32 idx = xs->offset - offset - 1;
        DEBUG_PRINTF("check bitmap idx %u\n", idx);
        assert(idx < 64);
        if (xs->bitmap & (1ULL << idx)) {
            DEBUG_PRINTF("match in bitmap\n");
            return REPEAT_MATCH;
        }
    }

    DEBUG_PRINTF("no match\n");
    return REPEAT_NOMATCH;
}

/** \brief True if the given value can be packed into len bytes.  */
static really_inline
int fits_in_len_bytes(u64a val, u32 len) {
    if (len >= 8) {
        return 1;
    }
    return val <= (1ULL << (len * 8));
}

static really_inline
void storePackedRelative(char *dest, u64a val, u64a offset, u64a max, u32 len) {
    assert(val <= offset);
    assert(fits_in_len_bytes(max, len));
    u64a delta = offset - val;
    if (delta >= max) {
        delta = max;
    }
    DEBUG_PRINTF("delta %llu\n", delta);
    assert(fits_in_len_bytes(delta, len));
    partial_store_u64a(dest, delta, len);
}

static
void repeatPackRing(char *dest, const struct RepeatInfo *info,
                    const union RepeatControl *ctrl, u64a offset) {
    const struct RepeatRingControl *xs = &ctrl->ring;
    const u32 ring_indices_len = info->repeatMax < 254 ? 2 : 4;
    const u32 offset_len = info->packedCtrlSize - ring_indices_len;

    // Write out packed relative base offset.
    assert(info->packedCtrlSize > ring_indices_len);
    storePackedRelative(dest, xs->offset, offset, info->horizon, offset_len);

    // Write out ring indices.
    if (ring_indices_len == 4) {
        unaligned_store_u16(dest + offset_len, xs->first);
        unaligned_store_u16(dest + offset_len + 2, xs->last);
    } else {
        assert(xs->first < 256 && xs->last < 256);
        u8 *indices = (u8 *)dest + offset_len;
        indices[0] = xs->first;
        indices[1] = xs->last;
    }
}

static
void repeatPackOffset(char *dest, const struct RepeatInfo *info,
                      const union RepeatControl *ctrl, u64a offset) {
    const struct RepeatOffsetControl *xs = &ctrl->offset;
    DEBUG_PRINTF("packing offset %llu [h %u]\n", xs->offset, info->horizon);
    if (!info->packedCtrlSize) {
        assert(info->type == REPEAT_ALWAYS);
        DEBUG_PRINTF("externally guarded .*\n");
        return;
    }
    storePackedRelative(dest, xs->offset, offset, info->horizon,
                        info->packedCtrlSize);
}

static
void repeatPackRange(char *dest, const struct RepeatInfo *info,
                     const union RepeatControl *ctrl, u64a offset) {
    const struct RepeatRangeControl *xs = &ctrl->range;

    // Write out packed relative base offset.
    assert(info->packedCtrlSize > 1);
    storePackedRelative(dest, xs->offset, offset, info->horizon,
                        info->packedCtrlSize - 1);

    // Write out range number of elements.
    dest[info->packedCtrlSize - 1] = xs->num;
}

static
void repeatPackBitmap(char *dest, const struct RepeatInfo *info,
                      const union RepeatControl *ctrl, u64a offset) {
    const struct RepeatBitmapControl *xs = &ctrl->bitmap;
    const u32 bound = info->repeatMax;

    assert(offset >= xs->offset);
    u64a new_base = offset > bound ? offset - bound : 0;

    // Shift bitmap to begin at new_base rather than xs->offset.
    u64a bitmap = xs->bitmap;
    if (new_base >= xs->offset) {
        u64a shift = new_base - xs->offset;
        bitmap = shift < 64 ? bitmap >> shift : 0;
    } else {
        u64a shift = xs->offset - new_base;
        bitmap = shift < 64 ? bitmap << shift : 0;
    }

    DEBUG_PRINTF("packing %llu into %u bytes\n", bitmap, info->packedCtrlSize);

    // Write out packed bitmap.
    assert(fits_in_len_bytes(bitmap, info->packedCtrlSize));
    partial_store_u64a(dest, bitmap, info->packedCtrlSize);
}

static
void repeatPackSparseOptimalP(char *dest, const struct RepeatInfo *info,
                             const union RepeatControl *ctrl, u64a offset) {
    const struct RepeatRingControl *xs = &ctrl->ring;
    // set ring index pointer according to patch count
    const u32 ring_indices_len = info->patchCount < 254 ? 2 : 4;
    const u32 offset_len = info->packedCtrlSize - ring_indices_len;

    // Write out packed relative base offset.
    assert(info->packedCtrlSize > ring_indices_len);
    storePackedRelative(dest, xs->offset, offset, info->horizon, offset_len);

    // Write out ring indices.
    if (ring_indices_len == 4) {
        unaligned_store_u16(dest + offset_len, xs->first);
        unaligned_store_u16(dest + offset_len + 2, xs->last);
    } else {
        assert(xs->first < 256 && xs->last < 256);
        u8 *indices = (u8 *)dest + offset_len;
        indices[0] = xs->first;
        indices[1] = xs->last;
    }

}

static
void repeatPackTrailer(char *dest, const struct RepeatInfo *info,
                       const union RepeatControl *ctrl, u64a offset) {
    const struct RepeatTrailerControl *xs = &ctrl->trailer;

    DEBUG_PRINTF("saving: offset=%llu, xs->offset=%llu, xs->bitmap=0x%llx\n",
                 offset, xs->offset, xs->bitmap);

    // XXX: xs->offset may be zero in the NFA path (effectively uninitialized).
    u64a top;
    if (xs->offset) {
        assert(xs->offset >= info->repeatMin);
        top = xs->offset - info->repeatMin;
    } else {
        top = 0;
    }

    top = offset - top; // Pack top relative to offset.

    u64a v[2];
    v[0] = MIN(top, info->horizon);
    v[1] = xs->bitmap;

    pack_bits_64(dest, v, info->packedFieldSizes, 2);
}

void repeatPack(char *dest, const struct RepeatInfo *info,
                const union RepeatControl *ctrl, u64a offset) {
    assert(dest && info && ctrl);

    switch ((enum RepeatType)info->type) {
    case REPEAT_RING:
        repeatPackRing(dest, info, ctrl, offset);
        break;
    case REPEAT_FIRST:
    case REPEAT_LAST:
        repeatPackOffset(dest, info, ctrl, offset);
        break;
    case REPEAT_RANGE:
        repeatPackRange(dest, info, ctrl, offset);
        break;
    case REPEAT_BITMAP:
        repeatPackBitmap(dest, info, ctrl, offset);
        break;
    case REPEAT_SPARSE_OPTIMAL_P:
        repeatPackSparseOptimalP(dest, info, ctrl, offset);
        break;
    case REPEAT_TRAILER:
        repeatPackTrailer(dest, info, ctrl, offset);
        break;
    case REPEAT_ALWAYS:
        /* nothing to do - no state */
        break;
    }
}

static really_inline
u64a loadPackedRelative(const char *src, u64a offset, u32 len) {
    u64a delta = partial_load_u64a(src, len);
    DEBUG_PRINTF("delta %llu\n", delta);
    assert(offset >= delta);
    return offset - delta;
}

static
void repeatUnpackRing(const char *src, const struct RepeatInfo *info,
                      u64a offset, union RepeatControl *ctrl) {
    struct RepeatRingControl *xs = &ctrl->ring;
    const u32 ring_indices_len = info->repeatMax < 254 ? 2 : 4;
    const u32 offset_len = info->packedCtrlSize - ring_indices_len;
    xs->offset = loadPackedRelative(src, offset, offset_len);
    if (ring_indices_len == 4) {
        xs->first = unaligned_load_u16(src + offset_len);
        xs->last = unaligned_load_u16(src + offset_len + 2);
    } else {
        const u8 *indices = (const u8 *)src + offset_len;
        xs->first = indices[0];
        xs->last = indices[1];
    }
}

static
void repeatUnpackOffset(const char *src, const struct RepeatInfo *info,
                        u64a offset, union RepeatControl *ctrl) {
    struct RepeatOffsetControl *xs = &ctrl->offset;
    if (!info->packedCtrlSize) {
        assert(info->type == REPEAT_ALWAYS);
        DEBUG_PRINTF("externally guarded .*\n");
        xs->offset = 0;
    } else {
        xs->offset = loadPackedRelative(src, offset, info->packedCtrlSize);
    }
    DEBUG_PRINTF("unpacking offset %llu [h%u]\n", xs->offset,
                 info->horizon);
}

static
void repeatUnpackRange(const char *src, const struct RepeatInfo *info,
                       u64a offset, union RepeatControl *ctrl) {
    struct RepeatRangeControl *xs = &ctrl->range;
    xs->offset = loadPackedRelative(src, offset, info->packedCtrlSize - 1);
    xs->num = src[info->packedCtrlSize - 1];
}

static
void repeatUnpackBitmap(const char *src, const struct RepeatInfo *info,
                        u64a offset, union RepeatControl *ctrl) {
    struct RepeatBitmapControl *xs = &ctrl->bitmap;
    xs->offset = offset > info->repeatMax ? offset - info->repeatMax : 0;
    xs->bitmap = partial_load_u64a(src, info->packedCtrlSize);
}

static
void repeatUnpackSparseOptimalP(const char *src, const struct RepeatInfo *info,
                                u64a offset, union RepeatControl *ctrl) {
    struct RepeatRingControl *xs = &ctrl->ring;
    const u32 ring_indices_len = info->patchCount < 254 ? 2 : 4;
    const u32 offset_len = info->packedCtrlSize - ring_indices_len;
    xs->offset = loadPackedRelative(src, offset, offset_len);
    if (ring_indices_len == 4) {
        xs->first = unaligned_load_u16(src + offset_len);
        xs->last = unaligned_load_u16(src + offset_len + 2);
    } else {
        const u8 *indices = (const u8 *)src + offset_len;
        xs->first = indices[0];
        xs->last = indices[1];
    }
}

static
void repeatUnpackTrailer(const char *src, const struct RepeatInfo *info,
                         u64a offset, union RepeatControl *ctrl) {
    struct RepeatTrailerControl *xs = &ctrl->trailer;

    u64a v[2];
    unpack_bits_64(v, (const u8 *)src, info->packedFieldSizes, 2);

    xs->offset = offset - v[0] + info->repeatMin;
    xs->bitmap = v[1];

    DEBUG_PRINTF("loaded: xs->offset=%llu, xs->bitmap=0x%llx\n", xs->offset,
                 xs->bitmap);
}

void repeatUnpack(const char *src, const struct RepeatInfo *info, u64a offset,
                  union RepeatControl *ctrl) {
    assert(src && info && ctrl);

    switch ((enum RepeatType)info->type) {
    case REPEAT_RING:
        repeatUnpackRing(src, info, offset, ctrl);
        break;
    case REPEAT_FIRST:
    case REPEAT_LAST:
        repeatUnpackOffset(src, info, offset, ctrl);
        break;
    case REPEAT_RANGE:
        repeatUnpackRange(src, info, offset, ctrl);
        break;
    case REPEAT_BITMAP:
        repeatUnpackBitmap(src, info, offset, ctrl);
        break;
    case REPEAT_SPARSE_OPTIMAL_P:
        repeatUnpackSparseOptimalP(src, info, offset, ctrl);
        break;
    case REPEAT_TRAILER:
        repeatUnpackTrailer(src, info, offset, ctrl);
        break;
    case REPEAT_ALWAYS:
        /* nothing to do - no state */
        break;
    }
}

static really_inline
const u64a *getImplTable(const struct RepeatInfo *info) {
    const u64a *table = ((const u64a *)(ROUNDUP_PTR(
                                        ((const char *)(info) +
                                        sizeof(*info)),
                                        alignof(u64a))));
    return table;
}

static
void storeInitialRingTopPatch(const struct RepeatInfo *info,
                              struct RepeatRingControl *xs,
                              u8 *state, u64a offset) {
    DEBUG_PRINTF("set the first patch, offset=%llu\n", offset);
    xs->offset = offset;

    u8 *active = state;
    u32 patch_count = info->patchCount;
    mmbit_clear(active, patch_count);
    mmbit_set(active, patch_count, 0);

    u8 *ring = active + info->patchesOffset;
    u32 encoding_size = info->encodingSize;
    partial_store_u64a(ring, 1ull, encoding_size);
    xs->first = 0;
    xs->last = 1;
}

static
u32 getSparseOptimalTargetValue(const struct RepeatInfo *info,
                                const u32 tval, u64a *val) {
    u32 patch_size = info->patchSize;
    const u64a *repeatTable = getImplTable(info);
    u32 loc = 0;
    DEBUG_PRINTF("val:%llu \n", *val);
    for (u32 i = 1; i <= patch_size - tval; i++) {
        u64a tmp = repeatTable[patch_size - i];
        if (*val >= tmp) {
            *val -= tmp;
            loc = i;
            i += (info->minPeriod - 1);
        }
    }

    return loc;
}

static
u64a sparseLastTop(const struct RepeatInfo *info,
                   const struct RepeatRingControl *xs, const u8 *state) {
    DEBUG_PRINTF("looking for last top\n");
    u32 patch_size = info->patchSize;
    u32 patch_count = info->patchCount;
    u32 encoding_size = info->encodingSize;

    u32 occ = ringOccupancy(xs, patch_count);
    u32 patch = xs->first + occ - 1;
    if (patch >= patch_count) {
        patch -= patch_count;
    }

    DEBUG_PRINTF("patch%u encoding_size%u occ%u\n", patch, encoding_size, occ);
    const u8 *ring = state + info->patchesOffset;
    u64a val = partial_load_u64a(ring + encoding_size * patch, encoding_size);

    DEBUG_PRINTF("val:%llu\n", val);
    const u64a *repeatTable = getImplTable(info);
    for (s32 i = patch_size - 1; i >= 0; i--) {
        if (val >= repeatTable[i]) {
            DEBUG_PRINTF("xs->offset%llu v%u p%llu\n",
                         xs->offset, i, repeatTable[i]);
            return xs->offset + i + (occ - 1) * patch_size;
        }
    }

    assert(0);
    return 0;
}

u64a repeatLastTopSparseOptimalP(const struct RepeatInfo *info,
                                 const union RepeatControl *ctrl,
                                 const void *state) {
    return sparseLastTop(info, &ctrl->ring, state);
}

u64a repeatNextMatchSparseOptimalP(const struct RepeatInfo *info,
                                   const union RepeatControl *ctrl,
                                   const void *state, u64a offset) {
    const struct RepeatRingControl *xs = &ctrl->ring;

    DEBUG_PRINTF("repeat [%u, %u] looking for match after %llu\n",
                 info->repeatMin, info->repeatMax, offset);

    assert(offset >= xs->offset);

    u64a nextOffset = offset + 1;

    u32 patch_size = info->patchSize;
    u32 patch;
    u32 tval;
    if (nextOffset <= xs->offset + info->repeatMin) {
        patch = xs->first;
        tval = 0;
    } else if (nextOffset > sparseLastTop(info, xs, state) + info->repeatMax) {
        DEBUG_PRINTF("ring is stale\n");
        return 0;
    } else {
        assert(nextOffset - xs->offset < UINT32_MAX); // ring is not stale
        u32 delta = (u32)(nextOffset - xs->offset);
        u32 lower = delta > info->repeatMax ? delta - info->repeatMax : 0;
        patch = lower / patch_size;
        tval = lower - patch * patch_size;
    }

    DEBUG_PRINTF("patch %u\n", patch);
    u32 patch_count = info->patchCount;
    if (patch >= patch_count) {
        return 0;
    }

    DEBUG_PRINTF("initial test for %u\n", tval);

    u32 begin = xs->first + patch;
    if (begin >= patch_count) {
        begin -= patch_count;
    }

    const u8 *active = (const u8 *)state;
    const u8 *ring = active + info->patchesOffset;
    u32 encoding_size = info->encodingSize;
    const u32 end = begin >= xs->last ? patch_count : xs->last;
    u32 low = tval;
    u64a diff = 0, loc = 0;
    DEBUG_PRINTF("begin %u end %u\n", begin, end);
    for (u32 p = mmbit_iterate_bounded(active, patch_count, begin, end);
         p != MMB_INVALID; p = mmbit_iterate_bounded(active, patch_count,
         p + 1, end)) {
        if (p != begin) {
            low = 0;
        }

        u64a val = partial_load_u64a(ring + encoding_size * p, encoding_size);
        u32 p1 = 0;
        if (p >= xs->first) {
            p1 = p - xs->first;
        } else {
            p1 = p + patch_count - xs->first;
        }

        if (val) {
            loc = getSparseOptimalTargetValue(info, low, &val);
            diff = (p1 + 1) * patch_size - loc;
        }
        if (loc) {
            u64a rv = MAX(nextOffset, xs->offset + info->repeatMin + diff);
            DEBUG_PRINTF("offset%llu next match at %llu\n", xs->offset, rv);
            return rv;
        }
        low = 0;
    }

    low = 0;
    if (begin >= xs->last) {
        for (u32 p = mmbit_iterate_bounded(active, patch_count, 0, xs->last);
             p != MMB_INVALID; p = mmbit_iterate_bounded(active, patch_count,
             p + 1, xs->last)) {

            u64a val = partial_load_u64a(ring + encoding_size * p,
                                         encoding_size);
            if (val) {
                loc = getSparseOptimalTargetValue(info, low, &val);
                diff = (p + 1) * patch_size - loc;
            }
            if (loc) {
                u64a rv = MAX(nextOffset, xs->offset + info->repeatMin +
                              diff + (end - xs->first) * patch_size);
                DEBUG_PRINTF("next match at %llu\n", rv);
                return rv;
            }
        }
    }

    DEBUG_PRINTF("next match\n");
    return 0;
}

void repeatStoreSparseOptimalP(const struct RepeatInfo *info,
                               union RepeatControl *ctrl, void *state,
                               u64a offset, char is_alive) {
    struct RepeatRingControl *xs = &ctrl->ring;
    u8 *active = (u8 *)state;

    DEBUG_PRINTF("offset: %llu encoding_size: %u\n", offset,
                 info->encodingSize);

    // If (a) this is the first top, or (b) the ring is stale, initialize the
    // ring and write this offset in as the first top.
    if (!is_alive ||
        offset > sparseLastTop(info, xs, state) + info->repeatMax) {
        storeInitialRingTopPatch(info, xs, active, offset);
        return;
    }

    // Tops should arrive in order, with no duplicates.
    assert(offset > sparseLastTop(info, xs, state));

    // As the ring is not stale, our delta should fit within a u32.
    assert(offset - xs->offset <= UINT32_MAX);
    u32 delta = (u32)(offset - xs->offset);
    u32 patch_size = info->patchSize;
    u32 patch_count = info->patchCount;
    u32 encoding_size = info->encodingSize;
    u32 patch = delta / patch_size;

    DEBUG_PRINTF("delta=%u, patch_size=%u, patch=%u\n", delta, patch_size,
                 patch);

    u8 *ring = active + info->patchesOffset;
    u32 occ = ringOccupancy(xs, patch_count);
    u64a val = 0;
    u32 idx;

    DEBUG_PRINTF("patch: %u patch_count: %u occ: %u\n",
                 patch, patch_count, occ);
    if (patch >= patch_count) {
        u32 patch_shift_count = patch - patch_count + 1;
        assert(patch >= patch_shift_count);
        DEBUG_PRINTF("shifting by %u\n", patch_shift_count);
        xs->offset += patch_size * patch_shift_count;
        xs->first += patch_shift_count;
        if (xs->first >= patch_count) {
            xs->first -= patch_count;
        }
        idx = xs->last + patch - occ;
        mmbit_unset_range(active, patch_count, xs->last,
                          MIN(idx, patch_count));
        if (idx >= patch_count) {
            idx -= patch_count;
            mmbit_unset_range(active, patch_count, 0, idx + 1);
        }
        xs->last = idx + 1;
        if (xs->last == patch_count) {
            xs->last = 0;
        }
    } else if (patch < occ) {
        assert(patch == occ - 1);
        idx = xs->last == 0 ? patch_count - 1 : (u32)xs->last - 1;
        val = partial_load_u64a(ring + encoding_size * idx, encoding_size);
    } else {
        idx = xs->last + patch - occ;
        mmbit_unset_range(active, patch_count, xs->last,
                          MIN(idx, patch_count));
        if (idx >= patch_count) {
            idx -= patch_count;
            mmbit_unset_range(active, patch_count, 0, idx + 1);
        }
        xs->last = idx + 1;
        if (xs->last == patch_count) {
            xs->last = 0;
        }
    }

    assert((u64a)patch * patch_size <= delta);
    u32 diff = delta - patch * patch_size;
    const u64a *repeatTable = getImplTable(info);
    val += repeatTable[diff];

    DEBUG_PRINTF("patch=%u, occ=%u\n", patch, occ);
    DEBUG_PRINTF("xs->first:%u xs->last:%u patch:%u\n",
                 xs->first, xs->last, patch);
    DEBUG_PRINTF("value:%llu\n", val);
    assert(fits_in_len_bytes(val, encoding_size));
    partial_store_u64a(ring + encoding_size * idx, val, encoding_size);
    mmbit_set(active, patch_count, idx);
}

static
char sparseHasMatch(const struct RepeatInfo *info, const u8 *state,
                    u32 lower, u32 upper) {
    u32 patch_size = info->patchSize;
    u32 patch_count = info->patchCount;
    u32 encoding_size = info->encodingSize;
    u32 patch_lower = lower / patch_size;
    u32 patch_upper = upper / patch_size;
    u32 diff = lower - patch_lower * patch_size;

    DEBUG_PRINTF("lower=%u, upper=%u\n", lower, upper);
    const u64a *repeatTable = getImplTable(info);

    const u8 *ring = state + info->patchesOffset;
    const u8 *active = state;
    u64a val;
    // test the first patch
    if (mmbit_isset(active, patch_count, patch_lower)) {
        val = partial_load_u64a(ring + encoding_size * patch_lower,
                                encoding_size);
        DEBUG_PRINTF("patch_size=%u, diff=%u, table=%llu\n",
                     patch_size, diff, repeatTable[diff]);
        DEBUG_PRINTF("patch_lower=%u, patch_upper=%u\n",
                     patch_lower, patch_upper);
        if (patch_upper == patch_lower) {
            u32 limit = upper - patch_lower * patch_size;
            getSparseOptimalTargetValue(info, limit + 1, &val);
        }
        if (val >= repeatTable[diff]) {
            return 1;
        }
    }

    if (patch_lower == patch_upper) {
        return 0;
    }

    // test the patches between first and last
    u32 m = mmbit_iterate_bounded(active, patch_count,
                                  patch_lower + 1, patch_upper);
    if (m != MMB_INVALID) {
        return 1;
    }

    if (patch_upper == patch_count) {
        return 0;
    }

    // test the last patch
    if (!mmbit_isset(active, patch_count, patch_upper)) {
        return 0;
    }
    diff = (patch_upper + 1) * patch_size - upper;
    DEBUG_PRINTF("diff=%u\n", diff);
    val = partial_load_u64a(ring + encoding_size * patch_upper, encoding_size);
    getSparseOptimalTargetValue(info, patch_size - diff + 1, &val);
    if (val) {
        DEBUG_PRINTF("last patch: val=%llu\n", val);
        return 1;
    }

    return 0;
}

enum RepeatMatch repeatHasMatchSparseOptimalP(const struct RepeatInfo *info,
                                              const union RepeatControl *ctrl,
                                              const void *state, u64a offset) {
    DEBUG_PRINTF("check for match at %llu corresponding to trigger "
                 "at [%llu, %llu]\n", offset, offset - info->repeatMax,
                 offset - info->repeatMin);

    const struct RepeatRingControl *xs = &ctrl->ring;
    const u8 *ring = (const u8 *)state;

    assert(offset >= xs->offset);

    if (offset < xs->offset + info->repeatMin) {
        DEBUG_PRINTF("too soon\n");
        return REPEAT_NOMATCH;
    } else if (offset > sparseLastTop(info, xs, state) + info->repeatMax) {
        DEBUG_PRINTF("stale\n");
        return REPEAT_STALE;
    }

    // Our delta between the base offset of the ring and the current offset
    // must fit within the range [repeatMin, lastPossibleTop + repeatMax]. This
    // range fits comfortably within a u32.
    assert(offset - xs->offset <= UINT32_MAX);

    u32 delta = (u32)(offset - xs->offset);
    u32 patch_size = info->patchSize;
    u32 patch_count = info->patchCount;
    u32 occ = ringOccupancy(xs, patch_count);

    u32 lower = delta > info->repeatMax ? delta - info->repeatMax : 0;
    u32 upper = MIN(delta - info->repeatMin, occ * patch_size - 1);

    DEBUG_PRINTF("lower=%u, upper=%u\n", lower, upper);
    u32 patch_lower = lower / patch_size;
    u32 patch_upper = upper / patch_size;

    if (patch_lower >= occ) {
        DEBUG_PRINTF("too late\n");
        return REPEAT_NOMATCH;
    }

    u32 remaining_lower = lower - patch_lower * patch_size;
    u32 remaining_upper = upper - patch_upper * patch_size;
    patch_lower += xs->first;
    patch_upper += xs->first;
    if (patch_lower >= patch_count) {
        patch_lower -= patch_count;
        patch_upper -= patch_count;
    } else if (patch_upper >= patch_count) {
        patch_upper -= patch_count;
    }

    DEBUG_PRINTF("xs->first:%u xs->last:%u patch_lower:%u, patch_upper:%u\n",
                 xs->first, xs->last, patch_lower, patch_upper);

    u32 scan_end;
    const char is_not_wrapped = (patch_lower <= patch_upper);
    if (is_not_wrapped) {
        scan_end = patch_upper * patch_size + remaining_upper;
    } else {
        scan_end = patch_count * patch_size;
    }

    lower = patch_lower * patch_size + remaining_lower;
    if (sparseHasMatch(info, ring, lower, scan_end)) {
        return REPEAT_MATCH;
    }

    if (!is_not_wrapped) {
        upper -= (patch_count - xs->first) * patch_size;
        if (sparseHasMatch(info, ring, 0, upper)) {
            return REPEAT_MATCH;
        }
    }

    return REPEAT_NOMATCH;
}
