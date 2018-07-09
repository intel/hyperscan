/*
 * Copyright (c) 2015-2018, Intel Corporation
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
 * \brief Report structure used to manage data associated with a report at
 * compile time.
 */

#ifndef UTIL_REPORT_H
#define UTIL_REPORT_H

#include "ue2common.h"
#include "util/exhaust.h" // for INVALID_EKEY
#include "util/logical.h" // for INVALID_LKEY
#include "util/hash.h"
#include "util/order_check.h"

#include <cassert>

namespace ue2 {

class ReportManager;

enum ReportType {
    EXTERNAL_CALLBACK,
    EXTERNAL_CALLBACK_SOM_REL,
    INTERNAL_SOM_LOC_SET,
    INTERNAL_SOM_LOC_SET_IF_UNSET,
    INTERNAL_SOM_LOC_SET_IF_WRITABLE,
    INTERNAL_SOM_LOC_SET_SOM_REV_NFA,
    INTERNAL_SOM_LOC_SET_SOM_REV_NFA_IF_UNSET,
    INTERNAL_SOM_LOC_SET_SOM_REV_NFA_IF_WRITABLE,
    INTERNAL_SOM_LOC_COPY,
    INTERNAL_SOM_LOC_COPY_IF_WRITABLE,
    INTERNAL_SOM_LOC_MAKE_WRITABLE,
    EXTERNAL_CALLBACK_SOM_STORED,
    EXTERNAL_CALLBACK_SOM_ABS,
    EXTERNAL_CALLBACK_SOM_REV_NFA,
    INTERNAL_SOM_LOC_SET_FROM,
    INTERNAL_SOM_LOC_SET_FROM_IF_WRITABLE,
    INTERNAL_ROSE_CHAIN,
    EXTERNAL_CALLBACK_SOM_PASS
};

/**
 * \brief All the data we use for handling a match.
 *
 * Includes extparam constraints and bounds, exhaustion/dedupe keys, offset
 * adjustment and SOM information.
 *
 * The data in this structure eventually becomes a list of Rose programs
 * instructions.
 */
struct Report {
    Report(ReportType type_in, u32 onmatch_in)
        : type(type_in), onmatch(onmatch_in) {}

    /** \brief True if this report has bounds from extended parameters, i.e.
     * min offset, max offset, min length. */
    bool hasBounds() const {
        return minOffset > 0 || maxOffset < MAX_OFFSET || minLength > 0;
    }

    /** \brief Type of this report. */
    ReportType type;

    /** \brief use SOM for minLength, but don't report it to user callback. */
    bool quashSom = false;

    /** \brief min offset in the stream at which this report can match. */
    u64a minOffset = 0;

    /** \brief max offset in the stream at which this report can match. */
    u64a maxOffset = MAX_OFFSET;

    /** \brief min match length (start of match to current offset) */
    u64a minLength = 0;

    /** \brief Exhaustion key.
     *
     * If exhaustible, the ekey to check before reporting a match.
     * Additionally after reporting a match the ekey will be set. If not
     * exhaustible, this will be INVALID_EKEY. */
    u32 ekey = INVALID_EKEY;

    /** \brief Logical Combination key in each combination.
     *
     * If in Logical Combination, the lkey to check before reporting a match.
     * Additionally before checking the lkey will be set. If not
     * in Logical Combination, this will be INVALID_LKEY. */
    u32 lkey = INVALID_LKEY;

    /** \brief Quiet flag for expressions in any logical combination. */
    bool quiet = false;

    /** \brief Adjustment to add to the match offset when we report a match.
     *
     * This is usually used for reports attached to states that form part of a
     * zero-width assertion, like '$'. */
    s32 offsetAdjust = 0;

    /** \brief Match report ID, for external reports.
     *
     * - external callback -> external report id
     * - internal_som_* -> som loc to modify
     * - INTERNAL_ROSE_CHAIN -> top event to push on
     * - otherwise -> target subnfa */
    u32 onmatch;

    /** \brief Index of the reverse nfa.
     *
     * Used by EXTERNAL_CALLBACK_SOM_REV_NFA and
     * INTERNAL_SOM_LOC_SET_SOM_REV_NFA*.
     */
    u32 revNfaIndex = 0;

    /** \brief SOM distance value, use varies according to type.
     *
     *  - for EXTERNAL_CALLBACK_SOM_REL, from-offset is this many bytes
     *    before the to-offset.
     *  - for EXTERNAL_CALLBACK_SOM_ABS, set from-offset to this value.
     *  - for INTERNAL_SOM_LOC_COPY*, som location read_from.
     */
    u64a somDistance = 0;

    /** \brief Number of bytes behind us that we are allowed to squash
     * identical top events on the queue.
     *
     * Used by INTERNAL_ROSE_CHAIN.
     */
    u64a topSquashDistance = 0;
};

static inline
bool isExternalReport(const Report &r) {
    switch (r.type) {
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
        return false;
    case EXTERNAL_CALLBACK:
    case EXTERNAL_CALLBACK_SOM_REL:
    case EXTERNAL_CALLBACK_SOM_STORED:
    case EXTERNAL_CALLBACK_SOM_ABS:
    case EXTERNAL_CALLBACK_SOM_REV_NFA:
    case EXTERNAL_CALLBACK_SOM_PASS:
        return true;
    default:
        break; // fall through
    }
    assert(0); // unknown?
    return true;
}

static inline
bool isExternalSomReport(const Report &r) {
    return r.type != EXTERNAL_CALLBACK && isExternalReport(r);
}

static inline
bool operator<(const Report &a, const Report &b) {
    ORDER_CHECK(type);
    ORDER_CHECK(quashSom);
    ORDER_CHECK(ekey);
    ORDER_CHECK(offsetAdjust);
    ORDER_CHECK(onmatch);
    ORDER_CHECK(minOffset);
    ORDER_CHECK(maxOffset);
    ORDER_CHECK(minLength);
    ORDER_CHECK(somDistance);
    ORDER_CHECK(revNfaIndex);
    ORDER_CHECK(topSquashDistance);
    return false;
}

inline
bool operator==(const Report &a, const Report &b) {
    return a.type == b.type && a.quashSom == b.quashSom &&
           a.minOffset == b.minOffset && a.maxOffset == b.maxOffset &&
           a.minLength == b.minLength && a.ekey == b.ekey &&
           a.offsetAdjust == b.offsetAdjust && a.onmatch == b.onmatch &&
           a.revNfaIndex == b.revNfaIndex && a.somDistance == b.somDistance &&
           a.topSquashDistance == b.topSquashDistance;
}

static inline
Report makeECallback(u32 report, s32 offsetAdjust, u32 ekey, bool quiet) {
    Report ir(EXTERNAL_CALLBACK, report);
    ir.offsetAdjust = offsetAdjust;
    ir.ekey = ekey;
    ir.quiet = (u8)quiet;
    return ir;
}

static inline
Report makeCallback(u32 report, s32 offsetAdjust) {
    return makeECallback(report, offsetAdjust, INVALID_EKEY, false);
}

static inline
Report makeSomRelativeCallback(u32 report, s32 offsetAdjust, u64a distance) {
    Report ir(EXTERNAL_CALLBACK_SOM_REL, report);
    ir.offsetAdjust = offsetAdjust;
    ir.ekey = INVALID_EKEY;
    ir.somDistance = distance;
    return ir;
}

static inline
Report makeMpvTrigger(u32 event, u64a squashDistance) {
    Report ir(INTERNAL_ROSE_CHAIN, event);
    ir.ekey = INVALID_EKEY;
    ir.topSquashDistance = squashDistance;
    return ir;
}

/** simple exhaustible: exhaustible and if the first attempted match does not
 * succeed, no later matches will succeed either  */
static inline
bool isSimpleExhaustible(const Report &ir) {
    if (ir.ekey == INVALID_EKEY) {
        return false;
    }

    if (ir.hasBounds() && (ir.minOffset || ir.minLength)) {
        return false;
    }

    if (!isExternalReport(ir)) {
        return false;
    }

    return true;
}

} // namespace ue2

namespace std {

template<>
struct hash<ue2::Report> {
    std::size_t operator()(const ue2::Report &r) const {
        return ue2::hash_all(r.type, r.quashSom, r.minOffset, r.maxOffset,
                             r.minLength, r.ekey, r.offsetAdjust, r.onmatch,
                             r.revNfaIndex, r.somDistance, r.topSquashDistance);
    }
};

} // namespace std

#endif // UTIL_REPORT_H
