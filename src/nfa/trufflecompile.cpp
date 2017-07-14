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
 * \brief Truffle compiler
 *
 * truffle is always able to represent an entire character class, providing a
 * backstop to other acceleration engines.
 */

#include "trufflecompile.h"

#include "ue2common.h"
#include "util/charreach.h"
#include "util/dump_mask.h"
#include "util/simd_types.h"

#include <cstring>

using namespace std;

namespace ue2 {

/*
 * To represent an entire charclass (256 chars), truffle uses two 128 bit
 * masks - the first is for chars that do not have the high bit/bit 7 set,
 * i.e. chars {0..127}. The second mask is for chars with bit 7 set.
 *
 * Each char to be represented is split into the low nibble (bits {0..3}) and
 * bits {4,5,6} - the low nibble is the offset into the mask and the value of
 * bits 456 is the bit that is set at that offset.
 */

void truffleBuildMasks(const CharReach &cr, u8 *shuf_mask_lo_highclear,
                       u8 *shuf_mask_lo_highset) {
    memset(shuf_mask_lo_highset, 0, sizeof(m128));
    memset(shuf_mask_lo_highclear, 0, sizeof(m128));

    for (size_t v = cr.find_first(); v != CharReach::npos;
         v = cr.find_next(v)) {
        DEBUG_PRINTF("adding 0x%02x to %s\n", (u8)v, (v & 0x80) ? "highset" : "highclear");
        u8 *change_mask = (v & 0x80) ? shuf_mask_lo_highset : shuf_mask_lo_highclear;
        u8 low_nibble = v & 0xf;
        u8 bits_456 = (v & 0x70) >> 4;
        change_mask[low_nibble] |= 1 << bits_456;
    }
}

/*
 * Reconstruct the charclass that the truffle masks represent
 */
CharReach truffle2cr(const u8 *highclear, const u8 *highset) {
    CharReach cr;
    for (u8 i = 0; i < 16; i++) {
        u32 bits_456 = highclear[i];
        while (bits_456) {
            u32 pos = findAndClearLSB_32(&bits_456);
            assert(pos < 8);
            cr.set(pos << 4 | i);
        }
        bits_456 = highset[i];
        while (bits_456) {
            u32 pos = findAndClearLSB_32(&bits_456);
            assert(pos < 8);
            cr.set(0x80 | pos << 4 | i);
        }
    }
    return cr;
}

} // namespc
