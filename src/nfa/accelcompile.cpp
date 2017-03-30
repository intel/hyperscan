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

#include "accel.h"
#include "accelcompile.h"
#include "shufticompile.h"
#include "trufflecompile.h"
#include "nfagraph/ng_limex_accel.h" /* for constants */
#include "util/bitutils.h"
#include "util/verify_types.h"

#include <map>
#include <set>
#include <vector>

using namespace std;

namespace ue2 {

static
void buildAccelSingle(const AccelInfo &info, AccelAux *aux) {
    assert(aux->accel_type == ACCEL_NONE);
    if (info.single_stops.all()) {
        return;
    }

    size_t outs = info.single_stops.count();
    DEBUG_PRINTF("%zu outs\n", outs);
    assert(outs && outs < 256);
    u32 offset = info.single_offset;

    if (outs == 1) {
        aux->accel_type = ACCEL_VERM;
        aux->verm.offset = offset;
        aux->verm.c = info.single_stops.find_first();
        DEBUG_PRINTF("building vermicelli caseful for 0x%02hhx\n", aux->verm.c);
        return;
    }

    if (outs == 2 && info.single_stops.isCaselessChar()) {
        aux->accel_type = ACCEL_VERM_NOCASE;
        aux->verm.offset = offset;
        aux->verm.c = info.single_stops.find_first() & CASE_CLEAR;
        DEBUG_PRINTF("building vermicelli caseless for 0x%02hhx\n",
                     aux->verm.c);
        return;
    }

    DEBUG_PRINTF("attempting shufti for %zu chars\n", outs);
    if (-1 != shuftiBuildMasks(info.single_stops, (u8 *)&aux->shufti.lo,
                               (u8 *)&aux->shufti.hi)) {
        aux->accel_type = ACCEL_SHUFTI;
        aux->shufti.offset = offset;
        DEBUG_PRINTF("shufti built OK\n");
        return;
    } else {
        DEBUG_PRINTF("shufti build failed, falling through\n");
    }

    if (outs <= ACCEL_MAX_STOP_CHAR) {
        DEBUG_PRINTF("building Truffle for %zu chars\n", outs);
        aux->accel_type = ACCEL_TRUFFLE;
        aux->truffle.offset = offset;
        truffleBuildMasks(info.single_stops, (u8 *)&aux->truffle.mask1,
                          (u8 *)&aux->truffle.mask2);
        return;
    }

    DEBUG_PRINTF("unable to accelerate case with %zu outs\n", outs);
}

bool buildDvermMask(const flat_set<pair<u8, u8>> &escape_set, u8 *m1_out,
                    u8 *m2_out) {
    u8 a1 = 0xff;
    u8 a2 = 0xff;
    u8 b1 = 0xff;
    u8 b2 = 0xff;

    for (const auto &e : escape_set) {
        DEBUG_PRINTF("%0hhx %0hhx\n", e.first, e.second);
        a1 &= e.first;
        b1 &= ~e.first;
        a2 &= e.second;
        b2 &= ~e.second;
    }

    u8 m1 = a1 | b1;
    u8 m2 = a2 | b2;

    u32 holes1 = 8 - popcount32(m1);
    u32 holes2 = 8 - popcount32(m2);

    DEBUG_PRINTF("aaaa %0hhx %0hhx\n", a1, a2);
    DEBUG_PRINTF("bbbb %0hhx %0hhx\n", b1, b2);
    DEBUG_PRINTF("mask %0hhx %0hhx\n", m1, m2);

    assert(holes1 <= 8 && holes2 <= 8);
    assert(escape_set.size() <= 1U << (holes1 + holes2));
    if (escape_set.size() != 1U << (holes1 + holes2)) {
        return false;
    }

    if (m1_out) {
        *m1_out = m1;
    }
    if (m2_out) {
        *m2_out = m2;
    }

    return true;
}

static
bool isCaselessDouble(const flat_set<pair<u8, u8>> &stop) {
    // test for vector containing <A,Z> <A,z> <a,Z> <a,z>
    if (stop.size() != 4) {
        return false;
    }
    const u8 a = stop.begin()->first & CASE_CLEAR;
    const u8 b = stop.begin()->second & CASE_CLEAR;

    flat_set<pair<u8, u8>>::const_iterator it, ite;
    for (it = stop.begin(), ite = stop.end(); it != ite; ++it) {
        if ((it->first & CASE_CLEAR) != a || (it->second & CASE_CLEAR) != b) {
            return false;
        }
    }

    return true;
}

static
void buildAccelDouble(const AccelInfo &info, AccelAux *aux) {
    size_t outs1 = info.double_stop1.count();
    size_t outs2 = info.double_stop2.size();

    u8 offset = verify_u8(info.double_offset);
    DEBUG_PRINTF("outs1=%zu, outs2=%zu\n", outs1, outs2);

    assert(aux->accel_type == ACCEL_NONE);

    if (!outs2) {
        /*  no double byte accel available */
        return;
    }

    // double-byte accel
    if (outs1 == 0 && outs2 == 1) {
        aux->accel_type = ACCEL_DVERM;
        aux->dverm.offset = offset;
        aux->dverm.c1 = info.double_stop2.begin()->first;
        aux->dverm.c2 = info.double_stop2.begin()->second;
        DEBUG_PRINTF("building double-vermicelli caseful for 0x%02hhx%02hhx\n",
                     aux->dverm.c1, aux->dverm.c2);
        return;
    }

    if (outs1 == 0 && isCaselessDouble(info.double_stop2)) {
        aux->accel_type = ACCEL_DVERM_NOCASE;
        aux->dverm.offset = offset;
        aux->dverm.c1 = info.double_stop2.begin()->first & CASE_CLEAR;
        aux->dverm.c2 = info.double_stop2.begin()->second & CASE_CLEAR;
        DEBUG_PRINTF("building double-vermicelli caseless for 0x%02hhx%02hhx\n",
                     aux->dverm.c1, aux->dverm.c2);
        return;
    }

    if (outs1 == 0) {
        u8 m1;
        u8 m2;

        if (buildDvermMask(info.double_stop2, &m1, &m2)) {
            aux->accel_type = ACCEL_DVERM_MASKED;
            aux->dverm.offset = offset;
            aux->dverm.c1 = info.double_stop2.begin()->first & m1;
            aux->dverm.c2 = info.double_stop2.begin()->second & m2;
            aux->dverm.m1 = m1;
            aux->dverm.m2 = m2;
            DEBUG_PRINTF("building maskeddouble-vermicelli for 0x%02hhx%02hhx\n",
                         aux->dverm.c1, aux->dverm.c2);
            return;
        }
    }

    if (outs1 < outs2 && outs1 <= 2) { // Heuristic from UE-438.
        DEBUG_PRINTF("building double-shufti for %zu one-byte and %zu"
                     " two-byte literals\n", outs1, outs2);
        aux->accel_type = ACCEL_DSHUFTI;
        aux->dshufti.offset = offset;
        if (shuftiBuildDoubleMasks(
                info.double_stop1, info.double_stop2, (u8 *)&aux->dshufti.lo1,
                (u8 *)&aux->dshufti.hi1, (u8 *)&aux->dshufti.lo2,
                (u8 *)&aux->dshufti.hi2)) {
            return;
        }
    }

    // drop back to attempt single-byte accel
    DEBUG_PRINTF("dropping back to single-byte acceleration\n");
    aux->accel_type = ACCEL_NONE;
}

bool buildAccelAux(const AccelInfo &info, AccelAux *aux) {
    assert(aux->accel_type == ACCEL_NONE);
    if (info.single_stops.none()) {
        DEBUG_PRINTF("picked red tape\n");
        aux->accel_type = ACCEL_RED_TAPE;
        aux->generic.offset = info.single_offset;
    }
    if (aux->accel_type == ACCEL_NONE) {
        buildAccelDouble(info, aux);
    }
    if (aux->accel_type == ACCEL_NONE) {
        buildAccelSingle(info, aux);
    }

    assert(aux->accel_type == ACCEL_NONE
           || aux->generic.offset == info.single_offset
           || aux->generic.offset == info.double_offset);
    return aux->accel_type != ACCEL_NONE;
}

} // namespace ue2
