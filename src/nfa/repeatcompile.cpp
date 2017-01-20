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
 * \brief Bounded repeat compile-time code.
 */
#include "repeatcompile.h"
#include "util/bitutils.h"
#include "util/charreach.h"
#include "util/depth.h"
#include "util/dump_charclass.h"
#include "util/multibit_build.h"
#include "util/verify_types.h"

#include <algorithm>
#include <cstring> // memset
#include <utility>

using namespace std;

namespace ue2 {

/** \brief Calculate the number of slots required to store the given repeat in
 * a RANGE model. */
static
u32 numRangeSlots(u32 repeatMin, u32 repeatMax) {
    assert(repeatMax > repeatMin);

    u32 d = repeatMax - repeatMin;
    u32 slots = 2 * ((repeatMax / d) + 1);
    return slots;
}

static
u32 calcPackedBits(u64a val) {
    assert(val);
    if (val <= 1) {
        return 1;
    }
    u32 bits = lg2_64(val - 1) + 1U; /* lg2 rounds down */
    DEBUG_PRINTF("packing %llu into %u bits\n", val, bits);
    return bits;
}

/* returns the min number of bytes required to represent val options */
u32 calcPackedBytes(u64a val) {
    u32 rv = (calcPackedBits(val) + 7U) / 8U;
    DEBUG_PRINTF("packing %llu into %u bytes\n", val, rv);
    return rv;
}

static
u32 repeatRecurTable(struct RepeatStateInfo *info, const depth &repeatMax,
                     const u32 minPeriod) {
    u32 repeatTmp = info->patchCount > 2 ? 64 : (u32)repeatMax;
    u32 repeat_index = repeatTmp < minPeriod ? repeatTmp : minPeriod;
    for (u32 i = 0; i <= repeat_index; i++) {
        info->table.push_back(i + 1);
    }
    for (u32 i = minPeriod + 1; i <= repeatTmp; i++) {
        info->table.push_back(info->table[i - 1] + info->table[i - minPeriod]);
        if (info->table[i] < info->table[i - 1]) {
            return i - 1;
        }
    }
    return 0;
}

static
u32 findOptimalPatchSize(struct RepeatStateInfo *info, const depth &repeatMax,
                         const u32 minPeriod, u32 rv) {
    u32 cnt = 0;
    u32 patch_bits = 0;
    u32 total_size = 0;
    u32 min = ~0U;
    u32 patch_len = 0;

    if (!rv) {
        rv = repeatMax;
    }

    for (u32 i = minPeriod; i <= rv; i++) {
        cnt = ((u32)repeatMax + (i - 1)) / i + 1;

        // no bit packing version
        patch_bits = calcPackedBits(info->table[i]);
        total_size = (patch_bits + 7U) / 8U * cnt;

        if (total_size < min) {
            patch_len = i;
            min = total_size;
            info->patchCount = cnt;
        }
    }
    return patch_len;
}

RepeatStateInfo::RepeatStateInfo(enum RepeatType type, const depth &repeatMin,
                                 const depth &repeatMax, u32 minPeriod)
    : stateSize(0), packedCtrlSize(0), horizon(0), patchCount(0),
      patchSize(0), encodingSize(0), patchesOffset(0) {
    assert(repeatMin <= repeatMax);
    assert(repeatMax.is_reachable());
    assert(minPeriod || type != REPEAT_SPARSE_OPTIMAL_P);

    switch (type) {
    case REPEAT_FIRST:
        assert(repeatMin.is_finite());
        stateSize = 0; // everything is in the control block.
        horizon = repeatMin;
        packedCtrlSize = calcPackedBytes(horizon + 1);
        break;
    case REPEAT_LAST:
        assert(repeatMax.is_finite());
        stateSize = 0; // everything is in the control block.
        horizon = repeatMax + 1;
        packedCtrlSize = calcPackedBytes(horizon + 1);
        break;
    case REPEAT_RING:
        assert(repeatMax.is_finite());
        stateSize = mmbit_size(repeatMax + 1);
        horizon = repeatMax * 2 + 1; /* TODO: investigate tightening */
        // Packed offset member, plus two bytes for each ring index, reduced to
        // one byte each if they'll fit in eight bits.
        {
            u32 offset_len = calcPackedBytes(horizon + 1);
            u32 ring_indices_len = repeatMax < depth(254) ? 2 : 4;
            packedCtrlSize = offset_len + ring_indices_len;
        }
        break;
    case REPEAT_RANGE:
        assert(repeatMax.is_finite());
        assert(repeatMin < repeatMax);
        stateSize = numRangeSlots(repeatMin, repeatMax) * sizeof(u16);
        horizon = repeatMax * 2 + 1;
        // Packed offset member, plus one byte for the number of range
        // elements.
        packedCtrlSize = calcPackedBytes(horizon + 1) + 1;
        break;
    case REPEAT_BITMAP:
        stateSize = 0; // everything is in the control block.
        horizon = 0;   // unused
        packedCtrlSize = ROUNDUP_N(repeatMax + 1, 8) / 8;
        break;
    case REPEAT_SPARSE_OPTIMAL_P:
        assert(minPeriod);
        assert(repeatMax.is_finite());
        {
            u32 rv = repeatRecurTable(this, repeatMax, minPeriod);
            u32 repeatTmp = 0;
            if ((u32)repeatMax < minPeriod) {
                repeatTmp = repeatMax;
                patchCount = 1;
            } else {
                // find optimal patch size
                repeatTmp =
                    findOptimalPatchSize(this, repeatMax, minPeriod, rv);
                assert(patchCount < 65536);
            }
            DEBUG_PRINTF("repeat[%u %u], period=%u\n", (u32)repeatMin,
                         (u32)repeatMax, minPeriod);
            u64a maxVal = table[repeatTmp];
            encodingSize = calcPackedBytes(maxVal);
            patchSize = repeatTmp;
            assert(encodingSize <= 64);

            patchesOffset = mmbit_size(patchCount);
            stateSize = patchesOffset + encodingSize * patchCount;
            horizon = (repeatTmp * patchCount) * 2 + 1;
            u32 ring_indices_len = patchCount < depth(254) ? 2 : 4;
            packedCtrlSize = calcPackedBytes(horizon + 1) + ring_indices_len;
        }
        break;
    case REPEAT_TRAILER:
        assert(repeatMax.is_finite());
        assert(repeatMin <= depth(64));
        stateSize = 0; // everything is in the control block.
        horizon = repeatMax + 1;
        packedFieldSizes.resize(2);
        packedFieldSizes[0] = calcPackedBits(horizon + 1);
        packedFieldSizes[1] = repeatMin;
        packedCtrlSize = (packedFieldSizes[0] + packedFieldSizes[1] + 7U) / 8U;
        break;
    case REPEAT_ALWAYS:
        assert(repeatMin == 0ULL);
        assert(repeatMax.is_infinite());
        stateSize = 0; // everything is in the control block.
        horizon = 0;
        packedCtrlSize = 0;
        break;
    }
    DEBUG_PRINTF("stateSize=%u, packedCtrlSize=%u, horizon=%u\n", stateSize,
                 packedCtrlSize, horizon);

    assert(packedCtrlSize <= sizeof(RepeatControl));
}

/** \brief Returns the packed control block size in bytes for a given bounded
 * repeat. */
static
u32 packedSize(enum RepeatType type, const depth &repeatMin,
               const depth &repeatMax, u32 minPeriod) {
    RepeatStateInfo rsi(type, repeatMin, repeatMax, minPeriod);
    return rsi.packedCtrlSize;
}

/** \brief Returns the stream state size in bytes for a given bounded
 * repeat. */
static
u32 streamStateSize(enum RepeatType type, const depth &repeatMin,
                    const depth &repeatMax, u32 minPeriod) {
    RepeatStateInfo rsi(type, repeatMin, repeatMax, minPeriod);
    return rsi.stateSize;
}

enum RepeatType chooseRepeatType(const depth &repeatMin, const depth &repeatMax,
                                 u32 minPeriod, bool is_reset,
                                 bool has_external_guard) {
    if (repeatMax.is_infinite()) {
        if (has_external_guard && !repeatMin) {
            return REPEAT_ALWAYS;
        } else {
            return REPEAT_FIRST;
        }
    }

    if (repeatMin == depth(0) || is_reset) {
        return REPEAT_LAST;
    }

    // Cases with max < 64 can be handled with either bitmap or trailer. We use
    // whichever has smaller packed state.

    if (repeatMax < depth(64)) {
        u32 bitmap_len =
            packedSize(REPEAT_BITMAP, repeatMin, repeatMax, minPeriod);
        u32 trailer_len =
            packedSize(REPEAT_TRAILER, repeatMin, repeatMax, minPeriod);
        return bitmap_len <= trailer_len ? REPEAT_BITMAP : REPEAT_TRAILER;
    }

    if (repeatMin <= depth(64)) {
        return REPEAT_TRAILER;
    }

    u32 range_len = ~0U;
    if (repeatMax > repeatMin &&
        numRangeSlots(repeatMin, repeatMax) <= REPEAT_RANGE_MAX_SLOTS) {
        assert(numRangeSlots(repeatMin, repeatMax) < 256); // stored in u8
        range_len =
        streamStateSize(REPEAT_RANGE, repeatMin, repeatMax, minPeriod);
    }

    assert(repeatMax.is_finite());

    u32 sparse_len = ~0U;
    if (minPeriod > 6) {
        sparse_len =
        streamStateSize(REPEAT_SPARSE_OPTIMAL_P, repeatMin, repeatMax, minPeriod);
    }

    if (range_len != ~0U || sparse_len != ~0U) {
        return range_len < sparse_len ? REPEAT_RANGE : REPEAT_SPARSE_OPTIMAL_P;
    }

    return REPEAT_RING;
}

bool matches(vector<CharReach>::const_iterator a_it,
             vector<CharReach>::const_iterator a_ite,
             vector<CharReach>::const_iterator b_it,
             UNUSED vector<CharReach>::const_iterator b_ite) {
    for (; a_it != a_ite; ++a_it, ++b_it) {
        assert(b_it != b_ite);
        if ((*a_it & *b_it).none()) {
            return false;
        }
    }
    assert(b_it == b_ite);
    return true;
}

static
u32 minDistAfterA(const vector<CharReach> &a, const vector<CharReach> &b) {
    /* we do not count the case where b can end at the same position as a */

    for (u32 i = 1; i < b.size(); i++) {
        u32 overlap_len = b.size() - i;
        if (overlap_len <= a.size()) {
            if (matches(a.end() - overlap_len, a.end(),
                        b.begin(), b.end() - i)) {
                return i;
            }
        } else {
            assert(overlap_len > a.size());
            if (matches(a.begin(), a.end(),
                        b.end() - i - a.size(), b.end() - i)) {
                return i;
            }
        }
    }

    return b.size();
}

vector<size_t> minResetDistToEnd(const vector<vector<CharReach>> &triggers,
                                 const CharReach &cr) {
    /* if a trigger does not reset the repeat, it gets a distance of trigger
       length */
    vector<size_t> out;
    for (const auto &trig : triggers) {
        size_t size = trig.size();
        size_t i = 0;
        for (; i < size; i++) {
            if ((trig[size - i - 1] & cr).none()) {
                break;
            }
        }
        out.push_back(i);
    }

    return out;
}

#if defined(DEBUG) || defined(DUMP_SUPPORT)

static UNUSED
string dumpTrigger(const vector<CharReach> &trigger) {
    string s;
    for (const auto &cr : trigger) {
        s += describeClass(cr);
    }
    return s;
}

#endif

u32 minPeriod(const vector<vector<CharReach>> &triggers, const CharReach &cr,
              bool *can_reset) {
    assert(!triggers.empty());

    u32 rv = ~0U;
    *can_reset = true;
    vector<size_t> min_reset_dist = minResetDistToEnd(triggers, cr);

    for (const auto &trigger : triggers) {
        DEBUG_PRINTF("trigger: %s\n", dumpTrigger(trigger).c_str());
        for (size_t j = 0; j < triggers.size(); j++) {
            u32 min_ext = minDistAfterA(trigger, triggers[j]);
            rv = min(rv, min_ext);
            if (min_ext <= min_reset_dist[j]) {
                *can_reset = false;
            }
        }
    }

    DEBUG_PRINTF("min period %u\n", rv);
    return rv;
}

} // namespace ue2
