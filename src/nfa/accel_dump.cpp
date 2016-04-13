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
#include "util/simd_utils.h"

#include <cstdio>
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
    case ACCEL_MLVERM:
        return "multibyte long vermicelli";
    case ACCEL_MLVERM_NOCASE:
        return "multibyte long vermicelli nocase";
    case ACCEL_MLGVERM:
        return "multibyte long-grab vermicelli";
    case ACCEL_MLGVERM_NOCASE:
        return "multibyte long-grab vermicelli nocase";
    case ACCEL_MSVERM:
        return "multibyte shift vermicelli";
    case ACCEL_MSVERM_NOCASE:
        return "multibyte shift vermicelli nocase";
    case ACCEL_MSGVERM:
        return "multibyte shift-grab vermicelli";
    case ACCEL_MSGVERM_NOCASE:
        return "multibyte shift-grab vermicelli nocase";
    case ACCEL_MDSVERM:
        return "multibyte doubleshift vermicelli";
    case ACCEL_MDSVERM_NOCASE:
        return "multibyte doubleshift vermicelli nocase";
    case ACCEL_MDSGVERM:
        return "multibyte doubleshift-grab vermicelli";
    case ACCEL_MDSGVERM_NOCASE:
        return "multibyte doubleshift-grab vermicelli nocase";
    case ACCEL_MLSHUFTI:
        return "multibyte long shufti";
    case ACCEL_MLGSHUFTI:
        return "multibyte long-grab shufti";
    case ACCEL_MSSHUFTI:
        return "multibyte shift shufti";
    case ACCEL_MSGSHUFTI:
        return "multibyte shift-grab shufti";
    case ACCEL_MDSSHUFTI:
        return "multibyte doubleshift shufti";
    case ACCEL_MDSGSHUFTI:
        return "multibyte doubleshift-grab shufti";
    case ACCEL_MLTRUFFLE:
        return "multibyte long truffle";
    case ACCEL_MLGTRUFFLE:
        return "multibyte long-grab truffle";
    case ACCEL_MSTRUFFLE:
        return "multibyte shift truffle";
    case ACCEL_MSGTRUFFLE:
        return "multibyte shift-grab truffle";
    case ACCEL_MDSTRUFFLE:
        return "multibyte doubleshift truffle";
    case ACCEL_MDSGTRUFFLE:
        return "multibyte doubleshift-grab truffle";
    default:
        return "unknown!";
    }
}

static
void dumpShuftiCharReach(FILE *f, const m128 &lo, const m128 &hi) {
    CharReach cr = shufti2cr(lo, hi);
    fprintf(f, "count %zu class %s\n", cr.count(),
            describeClass(cr).c_str());
}

static
vector<CharReach> shufti2cr_array(const m128 lo_in, const m128 hi_in) {
    const u8 *lo = (const u8 *)&lo_in;
    const u8 *hi = (const u8 *)&hi_in;
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
void dumpDShuftiCharReach(FILE *f, const m128 &lo1, const m128 &hi1,
                                   const m128 &lo2, const m128 &hi2) {
    vector<CharReach> cr1 = shufti2cr_array(not128(lo1), not128(hi1));
    vector<CharReach> cr2 = shufti2cr_array(not128(lo2), not128(hi2));
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
void dumpShuftiMasks(FILE *f, const m128 &lo, const m128 &hi) {
    fprintf(f, "lo %s\n",
            dumpMask((const u8 *)&lo, 128).c_str());
    fprintf(f, "hi %s\n",
            dumpMask((const u8 *)&hi, 128).c_str());
}

static
void dumpTruffleCharReach(FILE *f, const m128 &hiset, const m128 &hiclear) {
    CharReach cr = truffle2cr(hiset, hiclear);
    fprintf(f, "count %zu class %s\n", cr.count(),
            describeClass(cr).c_str());
}

static
void dumpTruffleMasks(FILE *f, const m128 &hiset, const m128 &hiclear) {
    fprintf(f, "lo %s\n",
            dumpMask((const u8 *)&hiset, 128).c_str());
    fprintf(f, "hi %s\n",
            dumpMask((const u8 *)&hiclear, 128).c_str());
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
        dumpShuftiMasks(f, accel.shufti.lo, accel.shufti.hi);
        dumpShuftiCharReach(f, accel.shufti.lo, accel.shufti.hi);
        break;
    }
    case ACCEL_DSHUFTI:
        fprintf(f, "\n");
        fprintf(f, "mask 1\n");
        dumpShuftiMasks(f, accel.dshufti.lo1, accel.dshufti.hi1);
        fprintf(f, "mask 2\n");
        dumpShuftiMasks(f, accel.dshufti.lo2, accel.dshufti.hi2);
        dumpDShuftiCharReach(f, accel.dshufti.lo1, accel.dshufti.hi1,
                             accel.dshufti.lo2, accel.dshufti.hi2);
        break;
    case ACCEL_TRUFFLE: {
        fprintf(f, "\n");
        dumpTruffleMasks(f, accel.truffle.mask1, accel.truffle.mask2);
        dumpTruffleCharReach(f, accel.truffle.mask1, accel.truffle.mask2);
        break;
    }
    case ACCEL_MLVERM:
    case ACCEL_MLVERM_NOCASE:
    case ACCEL_MLGVERM:
    case ACCEL_MLGVERM_NOCASE:
    case ACCEL_MSVERM:
    case ACCEL_MSVERM_NOCASE:
    case ACCEL_MSGVERM:
    case ACCEL_MSGVERM_NOCASE:
        fprintf(f, " [\\x%02hhx] len:%u\n", accel.mverm.c, accel.mverm.len);
        break;
    case ACCEL_MDSVERM:
    case ACCEL_MDSVERM_NOCASE:
    case ACCEL_MDSGVERM:
    case ACCEL_MDSGVERM_NOCASE:
        fprintf(f, " [\\x%02hhx] len1:%u len2:%u\n", accel.mdverm.c, accel.mdverm.len1,
                accel.mdverm.len2);
        break;
    case ACCEL_MLSHUFTI:
    case ACCEL_MLGSHUFTI:
    case ACCEL_MSSHUFTI:
    case ACCEL_MSGSHUFTI:
        fprintf(f, " len:%u\n", accel.mshufti.len);
        dumpShuftiMasks(f, accel.mshufti.lo, accel.mshufti.hi);
        dumpShuftiCharReach(f, accel.mshufti.lo, accel.mshufti.hi);
        break;
    case ACCEL_MDSSHUFTI:
    case ACCEL_MDSGSHUFTI:
        fprintf(f, " len1:%u len2:%u\n", accel.mdshufti.len1, accel.mdshufti.len2);
        dumpShuftiMasks(f, accel.mdshufti.lo, accel.mdshufti.hi);
        dumpShuftiCharReach(f, accel.mdshufti.lo, accel.mdshufti.hi);
        break;
    case ACCEL_MLTRUFFLE:
    case ACCEL_MLGTRUFFLE:
    case ACCEL_MSTRUFFLE:
    case ACCEL_MSGTRUFFLE:
        fprintf(f, " len:%u\n", accel.mtruffle.len);
        dumpTruffleMasks(f, accel.mtruffle.mask1, accel.mtruffle.mask2);
        dumpTruffleCharReach(f, accel.mtruffle.mask1, accel.mtruffle.mask2);
        break;
    case ACCEL_MDSTRUFFLE:
    case ACCEL_MDSGTRUFFLE:
        fprintf(f, " len1:%u len2:%u\n", accel.mdtruffle.len1, accel.mdtruffle.len2);
        dumpTruffleMasks(f, accel.mdtruffle.mask1, accel.mdtruffle.mask2);
        dumpTruffleCharReach(f, accel.mdtruffle.mask1, accel.mdtruffle.mask2);
        break;
    default:
        fprintf(f, "\n");
        break;
    }
}

} // namespace ue2
