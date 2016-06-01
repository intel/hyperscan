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

#ifndef REPEAT_INTERNAL_H
#define REPEAT_INTERNAL_H

#include "ue2common.h"

/** \file
 *   \brief Bounded Repeat models.
 *
 *  Used by the NFA, to represent bounded repeats managed via special POS and
 *  TUG exceptions, and by the LBR (limited bounded repeat) and Castle
 *  specialist engines.
 *
 *  We currently have a number of different kinds of bounded repeat model, for
 *  different kinds of {N,M} repeats, described by ::RepeatType.
 */

/** Different types of bounded repeats. */
enum RepeatType {
    /** General mechanism for tracking {N,M} repeats. Stores the first top as
     * an absolute offset, then subsequent tops in the {N,M} range as a ring of
     * relative top indices stored in a multibit. */
    REPEAT_RING,

    /** Used to track {N,} repeats. Uses the \ref RepeatOffsetControl structure,
     * since only the first top encountered needs to be stored. */
    REPEAT_FIRST,

    /** Used to track {0,N} repeats. Much like ::REPEAT_FIRST, except that we
     * store the most recent top encountered. */
    REPEAT_LAST,

    /** Like ::REPEAT_RING, this is also used for {N,M} repeats, but for cases
     * where there is a large difference between N and M, and developed to
     * reduce the state requirements of this case (relative to the RING model).
     * Uses a small ordered array of top indices relative to \ref
     * RepeatRangeControl::offset. */
    REPEAT_RANGE,

    /** Used for {N,M} repeats where 0 < M <= 64. Uses the \ref
     * RepeatBitmapControl structure at runtime. */
    REPEAT_BITMAP,

    /** Optimal mechanism for tracking {N,M} repeats when there is a bound on
     * how frequently they can be retriggered.
     * Assume f(repeat, min) representing the number of possible bit patterns
     * we can have for repeat size = repeat,  minimum period = min
     * We will have the following recurrence relation:
     * f(repeat, min) = f(repeat - 1, min) + f(repeat - min, min);
     * We use this recurrence to encode bit patterns with 64-bit values by
     * referencing a table that stores values from f(0, min) to f(repeat, min)
     * eg: repeat = 5, min = 2.  10001 => f(4,2) + f(0,2) = 9.
     * We search the optimal patch size between min and repeat in advance and
     * use the scheme above to do encoding and decoding to reduce stream state
     * size. */
    REPEAT_SPARSE_OPTIMAL_P,

    /** Used for {N,M} repeats where 0 < N < 64. Uses the
     * \ref RepeatTrailerControl structure at runtime. */
    REPEAT_TRAILER,

    /** Degenerate repeat that always returns true. Used by castle for pseudo
     * [^X]* repeats. */
    REPEAT_ALWAYS,
};

/**
 * \brief Value used to represent an unbounded max repeat.
 *
 * Note that we do not support \ref RepeatInfo::repeatMax values larger than
 * this.
 */
#define REPEAT_INF 65535

/** Max slots used by ::REPEAT_RANGE repeat model. */
#define REPEAT_RANGE_MAX_SLOTS 16

/** Structure describing a bounded repeat in the bytecode */
struct RepeatInfo {
    u8 type; //!< from enum RepeatType.
    u32 repeatMin; //!< minimum number of repeats.
    u32 repeatMax; //!< maximum number of repeats, or REPEAT_INF if unbounded.

    /** Maximum value that is required to be stored in the control block
     * counters. Any value greater than this will be capped at the horizon.
     */
    u32 horizon;

    /** Size of the compressed control block in bytes. This is what is written
     * out to stream state at stream boundaries. */
    u32 packedCtrlSize;

    /** Size of the repeat state block in bytes. This is where the REPEAT_RANGE
     * vector and REPEAT_RING multibit are stored, in stream state, and they
     * are manipulated directly (i.e. not copied at stream boundaries). */
    u32 stateSize;

    /** How soon after one trigger we can see the next trigger.
     * Used by REPEAT_SPARSE_OPTIMAL_P. */
    u32 minPeriod;

    /** Packed control block field sizes (in bits), used by REPEAT_TRAILER. */
    u32 packedFieldSizes[2];

    /* Number of patches, used by REPEAT_SPARSE_OPTIMAL_P. */
    u32 patchCount;

    /* Optimal patch length, used by REPEAT_SPARSE_OPTIMAL_P. */
    u32 patchSize;

    /* Encoding patch length in bytes, used by REPEAT_SPARSE_OPTIMAL_P. */
    u32 encodingSize;

    /* RepeatInfo struct length including table size. */
    u32 length;

    /** Offset of patches relative to the start of repeat stream state,
     * used by REPEAT_SPARSE_OPTIMAL_P. */
    u32 patchesOffset;
};

/** Runtime control block structure for ::REPEAT_RING and
 * ::REPEAT_SPARSE_OPTIMAL_P bounded repeats. Note that this struct is packed
 * (may not be aligned). */
struct RepeatRingControl {
    u64a offset; //!< index of first top.
    u16 first; //!< start index in ring.
    u16 last; //!< end index in ring.
};

/** Runtime control block structure for ::REPEAT_RANGE bounded repeats. Note
 * that this struct is packed (may not be aligned). */
struct RepeatRangeControl {
    u64a offset; //!< index of first top.
    u8 num; //!< number of elements in array.
};

/** Runtime control block structure for cases where only a single offset is
 * needed to track the repeat, both ::REPEAT_FIRST and ::REPEAT_LAST. Note that
 * this struct is packed (may not be aligned). */
struct RepeatOffsetControl {
    u64a offset; //!< index of a top.
};

/** Runtime control block structure for ::REPEAT_BITMAP bounded repeats. */
struct RepeatBitmapControl {
    u64a offset; //!< index of first top.
    u64a bitmap; //!< forward bitmap of tops relative to base offset.
};

/** Runtime control block structure for ::REPEAT_TRAILER bounded repeats. */
struct RepeatTrailerControl {
    u64a offset; //!< min extent of most recent match window.
    u64a bitmap; //!< trailing bitmap of earlier matches, relative to offset.
};

/** \brief Union of control block types, used at runtime. */
union RepeatControl {
    struct RepeatRingControl ring;
    struct RepeatRangeControl range;
    struct RepeatOffsetControl offset;
    struct RepeatBitmapControl bitmap;
    struct RepeatTrailerControl trailer;
};

/** For debugging, returns the name of a repeat model. */
static really_inline UNUSED
const char *repeatTypeName(u8 type) {
    switch ((enum RepeatType)type) {
    case REPEAT_RING:
        return "RING";
    case REPEAT_FIRST:
        return "FIRST";
    case REPEAT_LAST:
        return "LAST";
    case REPEAT_RANGE:
        return "RANGE";
    case REPEAT_BITMAP:
        return "BITMAP";
    case REPEAT_SPARSE_OPTIMAL_P:
        return "SPARSE_OPTIMAL_P";
    case REPEAT_TRAILER:
        return "TRAILER";
    case REPEAT_ALWAYS:
        return "ALWAYS";
    }
    assert(0);
    return "UNKNOWN";
}

#endif // REPEAT_INTERNAL_H
