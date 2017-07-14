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
 * \brief Shufti acceleration: compile code.
 */
#include "shufticompile.h"
#include "ue2common.h"
#include "util/charreach.h"
#include "util/container.h"
#include "util/flat_containers.h"

#include <array>
#include <cassert>
#include <cstring>
#include <map>

using namespace std;

namespace ue2 {

/** \brief Single-byte variant.
 *
 * Returns -1 if unable to construct masks, otherwise returns number of bits
 * used in the mask.
 *
 * Note: always able to construct masks for 8 or fewer characters.
 */
int shuftiBuildMasks(const CharReach &c, u8 *lo, u8 *hi) {
    /* Things could be packed much more optimally, but this should be able to
     * handle any set of characters entirely in the lower half.  */

    assert(c.count() < 256);
    assert(!c.none());

    map<u8, CharReach> by_hi; /* hi nibble -> set of matching lo nibbles */
    /* group matching characters by high nibble */
    for (size_t i = c.find_first(); i != CharReach::npos; i = c.find_next(i)) {
        u8 it_hi = i >> 4;
        u8 it_lo = i & 0xf;
        by_hi[it_hi].set(it_lo);
    }

    map<CharReach, CharReach> by_lo_set;
    /* group all hi nibbles with a common set of lo nibbles together */
    for (map<u8, CharReach>::const_iterator it = by_hi.begin();
         it != by_hi.end(); ++it) {
        by_lo_set[it->second].set(it->first);
    }

    if (by_lo_set.size() > 8) {
        /* too many char classes on the dance floor */
        assert(c.size() > 8);
        return -1;
    }

    u8 bit_index = 0;
    array<u8, 16> lo_a; lo_a.fill(0);
    array<u8, 16> hi_a; hi_a.fill(0);
    for (map<CharReach, CharReach>::const_iterator it = by_lo_set.begin();
         it != by_lo_set.end(); ++it) {
        const CharReach &lo_nibbles = it->first;
        const CharReach &hi_nibbles = it->second;

        /* set bits in low mask */
        for (size_t j = lo_nibbles.find_first(); j != CharReach::npos;
             j = lo_nibbles.find_next(j)) {
            lo_a[j] |= (1 << bit_index);
        }

        /* set bits in high mask */
        for (size_t j = hi_nibbles.find_first(); j != CharReach::npos;
             j = hi_nibbles.find_next(j)) {
            hi_a[j] |= (1 << bit_index);
        }

        bit_index++;
    }

    memcpy(lo, lo_a.data(), sizeof(m128));
    memcpy(hi, hi_a.data(), sizeof(m128));

    return bit_index;
}

static
array<u16, 4> or_array(array<u16, 4> a, const array<u16, 4> &b) {
    a[0] |= b[0];
    a[1] |= b[1];
    a[2] |= b[2];
    a[3] |= b[3];

    return a;
}


#define MAX_BUCKETS 8
static
void set_buckets_from_mask(u16 nibble_mask, u32 bucket,
                           array<u8, 16> &byte_mask) {
    assert(bucket < MAX_BUCKETS);

    u32 mask = nibble_mask;
    while (mask) {
        u32 n = findAndClearLSB_32(&mask);
        byte_mask[n]  &= ~(1 << bucket);
    }
}

bool shuftiBuildDoubleMasks(const CharReach &onechar,
                            const flat_set<pair<u8, u8>> &twochar,
                            u8 *lo1, u8 *hi1, u8 *lo2, u8 *hi2) {
    DEBUG_PRINTF("unibytes %zu dibytes %zu\n", onechar.size(),
                 twochar.size());
    array<u8, 16> lo1_a;
    array<u8, 16> lo2_a;
    array<u8, 16> hi1_a;
    array<u8, 16> hi2_a;

    lo1_a.fill(0xff);
    lo2_a.fill(0xff);
    hi1_a.fill(0xff);
    hi2_a.fill(0xff);

    // two-byte literals
    vector<array<u16, 4>> nibble_masks;
    for (const auto &p : twochar) {
        DEBUG_PRINTF("%02hhx %02hhx\n", p.first, p.second);
        u16 a_lo = 1U << (p.first  & 0xf);
        u16 a_hi = 1U << (p.first  >> 4);
        u16 b_lo = 1U << (p.second & 0xf);
        u16 b_hi = 1U << (p.second >> 4);
        nibble_masks.push_back({{a_lo, a_hi, b_lo, b_hi}});
    }

    // one-byte literals (second byte is a wildcard)
    for (size_t it = onechar.find_first(); it != CharReach::npos;
         it = onechar.find_next(it)) {
        DEBUG_PRINTF("%02hhx\n", (u8)it);
        u16 a_lo = 1U << (it & 0xf);
        u16 a_hi = 1U << (it >> 4);
        u16 wildcard = 0xffff;
        nibble_masks.push_back({{a_lo, a_hi, wildcard, wildcard}});
    }

    // try to merge strings into shared buckets
    for (u32 i = 0; i < 4; i++) {
        map<array<u16, 4>, array<u16, 4>> new_masks;
        for (const auto &a : nibble_masks) {
            auto key = a;
            key[i] = 0;
            if (!contains(new_masks, key)) {
                new_masks[key] = a;
            } else {
                new_masks[key] = or_array(new_masks[key], a);
            }
        }
        nibble_masks.clear();
        for (const auto &e : new_masks) {
            nibble_masks.push_back(e.second);
        }
    }

    if (nibble_masks.size() > MAX_BUCKETS) {
        DEBUG_PRINTF("too many buckets needed (%zu)\n", nibble_masks.size());
        return false;
    }

    u32 i = 0;
    for (const auto &a : nibble_masks) {
        set_buckets_from_mask(a[0], i, lo1_a);
        set_buckets_from_mask(a[1], i, hi1_a);
        set_buckets_from_mask(a[2], i, lo2_a);
        set_buckets_from_mask(a[3], i, hi2_a);
        i++;
    }

    memcpy(lo1, lo1_a.data(), sizeof(m128));
    memcpy(lo2, lo2_a.data(), sizeof(m128));
    memcpy(hi1, hi1_a.data(), sizeof(m128));
    memcpy(hi2, hi2_a.data(), sizeof(m128));

    return true;
}

#ifdef DUMP_SUPPORT

CharReach shufti2cr(const u8 *lo, const u8 *hi) {
    CharReach cr;
    for (u32 i = 0; i < 256; i++) {
        if (lo[(u8)i & 0xf] & hi[(u8)i >> 4]) {
            cr.set(i);
        }
    }
    return cr;
}

#endif // DUMP_SUPPORT

} // namespace ue2
