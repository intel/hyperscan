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
 * \brief Acceleration: dump code.
 */

#include "config.h"

#include "accel.h"
#include "accel_dump.h"
#include "shufticompile.h"
#include "trufflecompile.h"
#include "ue2common.h"
#include "util/bitutils.h"
#include "util/charreach.h"
#include "util/dump_charclass.h"
#include "util/dump_mask.h"
#include "util/simd_types.h"

#include <cstdio>
#include <map>
#include <set>
#include <vector>

#ifndef DUMP_SUPPORT
#error No dump support!
#endif

using namespace std;

namespace ue2 {

static
const char *accelName(u8 accel_type) {
    switch (accel_type) {
    case ACCEL_NONE:
        return "none";
    case ACCEL_VERM:
        return "vermicelli";
    case ACCEL_VERM_NOCASE:
        return "vermicelli nocase";
    case ACCEL_DVERM:
        return "double-vermicelli";
    case ACCEL_DVERM_NOCASE:
        return "double-vermicelli nocase";
    case ACCEL_DVERM_MASKED:
        return "double-vermicelli masked";
    case ACCEL_RVERM:
        return "reverse vermicelli";
    case ACCEL_RVERM_NOCASE:
        return "reverse vermicelli nocase";
    case ACCEL_RDVERM:
        return "reverse double-vermicelli";
    case ACCEL_RDVERM_NOCASE:
        return "reverse double-vermicelli nocase";
    case ACCEL_REOD:
        return "reverse eod";
    case ACCEL_REOD_NOCASE:
        return "reverse eod nocase";
    case ACCEL_RDEOD:
        return "reverse double-eod";
    case ACCEL_RDEOD_NOCASE:
        return "reverse double-eod nocase";
    case ACCEL_SHUFTI:
        return "shufti";
    case ACCEL_DSHUFTI:
        return "double-shufti";
    case ACCEL_TRUFFLE:
        return "truffle";
    case ACCEL_RED_TAPE:
        return "red tape";
    default:
        return "unknown!";
    }
}

static
void dumpShuftiCharReach(FILE *f, const u8 *lo, const u8 *hi) {
    CharReach cr = shufti2cr(lo, hi);
    fprintf(f, "count %zu class %s\n", cr.count(),
            describeClass(cr).c_str());
}

static
vector<CharReach> dshufti2cr_array(const u8 *lo_in, const u8 *hi_in) {
    u8 lo[16];
    u8 hi[16];
    for (u32 i = 0; i < 16; i++) {
        lo[i] = ~lo_in[i];
        hi[i] = ~hi_in[i];
    }
    vector<CharReach> crs(8);
    for (u32 i = 0; i < 256; i++) {
        u32 combined = lo[(u8)i & 0xf] & hi[(u8)i >> 4];
        while (combined) {
            u32 j = findAndClearLSB_32(&combined);
            crs.at(j).set(i);
        }
    }
    return crs;
}

static
void dumpDShuftiCharReach(FILE *f, const u8 *lo1, const u8 *hi1,
                                   const u8 *lo2, const u8 *hi2) {
    vector<CharReach> cr1 = dshufti2cr_array(lo1, hi1);
    vector<CharReach> cr2 = dshufti2cr_array(lo2, hi2);
    map<CharReach, set<u32> > cr1_group;
    assert(cr1.size() == 8 && cr2.size() == 8);
    for (u32 i = 0; i < 8; i++) {
        if (!cr1[i].any()) {
            continue;
        }
        cr1_group[cr1[i]].insert(i);
    }
    map<CharReach, CharReach> rev;
    for (const auto &e : cr1_group) {
        CharReach rhs;
        for (u32 r : e.second) {
            rhs |= cr2.at(r);
        }

        rev[rhs] |= e.first;
    }
    fprintf(f, "escapes: {");
    for (auto it = rev.begin(); it != rev.end(); ++it) {
        const auto &e = *it;
        if (it != rev.begin()) {
            fprintf(f, ", ");
        }

        if (e.first.all()) {
            fprintf(f, "%s", describeClass(e.second).c_str());
        } else {
            fprintf(f, "%s%s", describeClass(e.second).c_str(),
                    describeClass(e.first).c_str());
        }
    }
    fprintf(f, "}\n");
}

static
void dumpShuftiMasks(FILE *f, const u8 *lo, const u8 *hi) {
    fprintf(f, "lo %s\n", dumpMask(lo, 128).c_str());
    fprintf(f, "hi %s\n", dumpMask(hi, 128).c_str());
}

static
void dumpTruffleCharReach(FILE *f, const u8 *hiset, const u8 *hiclear) {
    CharReach cr = truffle2cr(hiset, hiclear);
    fprintf(f, "count %zu class %s\n", cr.count(),
            describeClass(cr).c_str());
}

static
void dumpTruffleMasks(FILE *f, const u8 *hiset, const u8 *hiclear) {
    fprintf(f, "lo %s\n", dumpMask(hiset, 128).c_str());
    fprintf(f, "hi %s\n", dumpMask(hiclear, 128).c_str());
}


void dumpAccelInfo(FILE *f, const AccelAux &accel) {
    fprintf(f, " %s", accelName(accel.accel_type));
    if (accel.generic.offset) {
        fprintf(f, "+%hhu", accel.generic.offset);
    }

    switch (accel.accel_type) {
    case ACCEL_VERM:
    case ACCEL_VERM_NOCASE:
    case ACCEL_RVERM:
    case ACCEL_RVERM_NOCASE:
        fprintf(f, " [\\x%02hhx]\n", accel.verm.c);
        break;
    case ACCEL_DVERM:
    case ACCEL_DVERM_NOCASE:
    case ACCEL_RDVERM:
    case ACCEL_RDVERM_NOCASE:
        fprintf(f, " [\\x%02hhx\\x%02hhx]\n", accel.dverm.c1, accel.dverm.c2);
        break;
    case ACCEL_DVERM_MASKED:
        fprintf(f, " [\\x%02hhx\\x%02hhx] & [\\x%02hhx\\x%02hhx]\n",
                accel.dverm.c1, accel.dverm.c2, accel.dverm.m1, accel.dverm.m2);
        break;
    case ACCEL_SHUFTI: {
        fprintf(f, "\n");
        dumpShuftiMasks(f, (const u8 *)&accel.shufti.lo,
                        (const u8 *)&accel.shufti.hi);
        dumpShuftiCharReach(f, (const u8 *)&accel.shufti.lo,
                            (const u8 *)&accel.shufti.hi);
        break;
    }
    case ACCEL_DSHUFTI:
        fprintf(f, "\n");
        fprintf(f, "mask 1\n");
        dumpShuftiMasks(f, (const u8 *)&accel.dshufti.lo1,
                        (const u8 *)&accel.dshufti.hi1);
        fprintf(f, "mask 2\n");
        dumpShuftiMasks(f, (const u8 *)&accel.dshufti.lo2,
                        (const u8 *)&accel.dshufti.hi2);
        dumpDShuftiCharReach(f, (const u8 *)&accel.dshufti.lo1,
                             (const u8 *)&accel.dshufti.hi1,
                             (const u8 *)&accel.dshufti.lo2,
                             (const u8 *)&accel.dshufti.hi2);
        break;
    case ACCEL_TRUFFLE: {
        fprintf(f, "\n");
        dumpTruffleMasks(f, (const u8 *)&accel.truffle.mask1,
                         (const u8 *)&accel.truffle.mask2);
        dumpTruffleCharReach(f, (const u8 *)&accel.truffle.mask1,
                             (const u8 *)&accel.truffle.mask2);
        break;
    }
    default:
        fprintf(f, "\n");
        break;
    }
}

} // namespace ue2
