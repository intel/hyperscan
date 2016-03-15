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
 * \brief Shufti acceleration: compile code.
 */
#include "shufticompile.h"
#include "ue2common.h"
#include "util/charreach.h"
#include "util/ue2_containers.h"

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
int shuftiBuildMasks(const CharReach &c, m128 *lo, m128 *hi) {
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

void shuftiBuildDoubleMasks(const CharReach &onechar,
                            const flat_set<pair<u8, u8>> &twochar,
                            m128 *lo1, m128 *hi1, m128 *lo2, m128 *hi2) {
    DEBUG_PRINTF("unibytes %zu dibytes %zu\n", onechar.size(),
                 twochar.size());
    assert(onechar.count() + twochar.size() <= 8);

    array<u8, 16> lo1_a;
    array<u8, 16> lo2_a;
    array<u8, 16> hi1_a;
    array<u8, 16> hi2_a;

    lo1_a.fill(0xff);
    lo2_a.fill(0xff);
    hi1_a.fill(0xff);
    hi2_a.fill(0xff);

    u32 i = 0;

    // two-byte literals
    for (flat_set<pair<u8, u8>>::const_iterator it = twochar.begin();
         it != twochar.end(); ++it, i++) {
        DEBUG_PRINTF("%u: %02hhx %02hhx\n", i, it->first, it->second);
        u8 b1 = it->first & 0xf;
        u8 t1 = it->first >> 4;
        u8 b2 = it->second & 0xf;
        u8 t2 = it->second >> 4;

        lo1_a[b1] &= ~(1 << i);
        hi1_a[t1] &= ~(1 << i);
        lo2_a[b2] &= ~(1 << i);
        hi2_a[t2] &= ~(1 << i);
    }

    // one-byte literals (second byte is a wildcard)
    for (size_t it = onechar.find_first(); it != CharReach::npos;
         it = onechar.find_next(it), i++) {
        DEBUG_PRINTF("%u: %02hhx\n", i, (u8)it);
        u8 b1 = it & 0xf;
        u8 t1 = it >> 4;

        lo1_a[b1] &= ~(1 << i);
        hi1_a[t1] &= ~(1 << i);

        for (int j = 0; j < 16; j++) {
            lo2_a[j] &= ~(1 << i);
            hi2_a[j] &= ~(1 << i);
        }
    }

    memcpy(lo1, lo1_a.data(), sizeof(m128));
    memcpy(lo2, lo2_a.data(), sizeof(m128));
    memcpy(hi1, hi1_a.data(), sizeof(m128));
    memcpy(hi2, hi2_a.data(), sizeof(m128));
}

#ifdef DUMP_SUPPORT

CharReach shufti2cr(const m128 lo_in, const m128 hi_in) {
    const u8 *lo = (const u8 *)&lo_in;
    const u8 *hi = (const u8 *)&hi_in;
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
