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
 * \brief Definition of an internal_report, along with convenience functions.
 */

#ifndef INTERNAL_REPORT_H
#define INTERNAL_REPORT_H

#include "ue2common.h"

/* internal_report::type values */

#define EXTERNAL_CALLBACK                             0
#define EXTERNAL_CALLBACK_SOM_REL                     1
#define INTERNAL_SOM_LOC_SET                          2
#define INTERNAL_SOM_LOC_SET_IF_UNSET                 3
#define INTERNAL_SOM_LOC_SET_IF_WRITABLE              4
#define INTERNAL_SOM_LOC_SET_SOM_REV_NFA              5
#define INTERNAL_SOM_LOC_SET_SOM_REV_NFA_IF_UNSET     6
#define INTERNAL_SOM_LOC_SET_SOM_REV_NFA_IF_WRITABLE  7
#define INTERNAL_SOM_LOC_COPY                         8
#define INTERNAL_SOM_LOC_COPY_IF_WRITABLE             9
#define INTERNAL_SOM_LOC_MAKE_WRITABLE               10
#define EXTERNAL_CALLBACK_SOM_STORED                 11
#define EXTERNAL_CALLBACK_SOM_ABS                    12
#define EXTERNAL_CALLBACK_SOM_REV_NFA                13

/** set the som loc to the value in from_offset */
#define INTERNAL_SOM_LOC_SET_FROM                    14

/** set the som loc to the value in from_offset */
#define INTERNAL_SOM_LOC_SET_FROM_IF_WRITABLE        15
#define INTERNAL_ROSE_CHAIN                          16

/** Index meaning a given exhaustion key is invalid. */
#define INVALID_EKEY      0xffffffff

/** \brief All the data we use for handling a match, bytecode representation.
 *
 * Includes extparam constraints and bounds, exhaustion/dedupe keys, offset
 * adjustment and SOM information.
 *
 * At compile time, this data is in the ue2::Report structure, which is
 * converted to internal_report for layout in the bytecode.
 */
struct ALIGN_CL_DIRECTIVE internal_report {
    /** \brief from EXTERNAL_ and INTERNAL_ defined above. */
    u8 type;

    /** \brief do we actually use minOffset, maxOffset */
    u8 hasBounds;

    /** \brief use SOM for minLength, but don't report it to user callback. */
    u8 quashSom;

    /** \brief min offset in the stream at which this report can match. */
    u64a minOffset;

    /** \brief max offset in the stream at which this report can match. */
    u64a maxOffset;

    /** \brief min match length (start of match to current offset) */
    u64a minLength;

    /** \brief Exhaustion key.
     *
     * If exhaustible, the ekey to check before reporting a match.
     * Additionally after reporting a match the ekey will be set. If not
     * exhaustible, this will be INVALID_EKEY. */
    u32 ekey;

    /** \brief Dedupe key. */
    u32 dkey;

    /** \brief Adjustment to add to the match offset when we report a match.
     *
     * This is usually used for reports attached to states that form part of a
     * zero-width assertion, like '$'. */
    s32 offsetAdjust;

    /** \brief Match report ID, for external reports.
     *
     * - external callback -> external report id
     * - internal_som_* -> som loc to modify,
     * - INTERNAL_ROSE_CHAIN -> top event to push on
     * - otherwise target subnfa. */
    u32 onmatch;

    union {
        /** \brief SOM distance value, use varies according to type.
         *
         *  - for EXTERNAL_CALLBACK_SOM_REL, from-offset is this many bytes
         *    before the to-offset.
         *  - for EXTERNAL_CALLBACK_SOM_ABS, set from-offset to this value.
         *  - for INTERNAL_SOM_LOC_COPY*, som location read_from.
         */
        u64a somDistance;

        /** \brief Index of the reverse nfa.
         * Used by EXTERNAL_CALLBACK_SOM_REV_NFA and
         *  INTERNAL_SOM_LOC_SET_SOM_REV_NFA*
         */
        u64a revNfaIndex;

        /**
         * Used by INTERNAL_ROSE_CHAIN, Number of bytes behind us that we are
         * allowed to squash identical top events on the queue.
         */
        u64a topSquashDistance;
    } aux;
};

static really_inline
int isInternalSomReport(const struct internal_report *ri) {
    switch (ri->type) {
    case INTERNAL_SOM_LOC_SET:
    case INTERNAL_SOM_LOC_SET_IF_UNSET:
    case INTERNAL_SOM_LOC_SET_IF_WRITABLE:
    case INTERNAL_SOM_LOC_SET_SOM_REV_NFA:
    case INTERNAL_SOM_LOC_SET_SOM_REV_NFA_IF_UNSET:
    case INTERNAL_SOM_LOC_SET_SOM_REV_NFA_IF_WRITABLE:
    case INTERNAL_SOM_LOC_COPY:
    case INTERNAL_SOM_LOC_COPY_IF_WRITABLE:
    case INTERNAL_SOM_LOC_MAKE_WRITABLE:
    case INTERNAL_SOM_LOC_SET_FROM:
    case INTERNAL_SOM_LOC_SET_FROM_IF_WRITABLE:
        return 1;
    case EXTERNAL_CALLBACK:
    case EXTERNAL_CALLBACK_SOM_REL:
    case EXTERNAL_CALLBACK_SOM_STORED:
    case EXTERNAL_CALLBACK_SOM_ABS:
    case EXTERNAL_CALLBACK_SOM_REV_NFA:
    case INTERNAL_ROSE_CHAIN:
        return 0;
    default:
        break; // fall through
    }
    assert(0); // unknown?
    return 0;
}

#ifndef NDEBUG
/* used in asserts */
static UNUSED
char isExternalReport(const struct internal_report *ir) {
    switch (ir->type) {
    case INTERNAL_SOM_LOC_SET:
    case INTERNAL_SOM_LOC_SET_IF_UNSET:
    case INTERNAL_SOM_LOC_SET_IF_WRITABLE:
    case INTERNAL_SOM_LOC_SET_SOM_REV_NFA:
    case INTERNAL_SOM_LOC_SET_SOM_REV_NFA_IF_UNSET:
    case INTERNAL_SOM_LOC_SET_SOM_REV_NFA_IF_WRITABLE:
    case INTERNAL_SOM_LOC_COPY:
    case INTERNAL_SOM_LOC_COPY_IF_WRITABLE:
    case INTERNAL_SOM_LOC_MAKE_WRITABLE:
    case INTERNAL_SOM_LOC_SET_FROM:
    case INTERNAL_SOM_LOC_SET_FROM_IF_WRITABLE:
    case INTERNAL_ROSE_CHAIN:
        return 0;
    case EXTERNAL_CALLBACK:
    case EXTERNAL_CALLBACK_SOM_REL:
    case EXTERNAL_CALLBACK_SOM_STORED:
    case EXTERNAL_CALLBACK_SOM_ABS:
    case EXTERNAL_CALLBACK_SOM_REV_NFA:
        return 1;
    default:
        break; // fall through
    }
    assert(0); // unknown?
    return 1;
}
#endif

#endif // INTERNAL_REPORT_H
