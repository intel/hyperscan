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
 * \brief Acceleration: dump code.
 */

#include "config.h"

#include "accel.h"
#include "accel_dump.h"
#include "shufticompile.h"
#include "trufflecompile.h"
#include "ue2common.h"
#include "util/charreach.h"
#include "util/dump_charclass.h"
#include "util/dump_mask.h"

#include <cstdio>

#ifndef DUMP_SUPPORT
#error No dump support!
#endif

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
    default:
        return "unknown!";
    }
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
    case ACCEL_SHUFTI: {
        fprintf(f, "\n");
        fprintf(f, "lo %s\n",
                dumpMask((const u8 *)&accel.shufti.lo, 128).c_str());
        fprintf(f, "hi %s\n",
                dumpMask((const u8 *)&accel.shufti.hi, 128).c_str());
        CharReach cr = shufti2cr(accel.shufti.lo, accel.shufti.hi);
        fprintf(f, "count %zu class %s\n", cr.count(),
                describeClass(cr).c_str());
        break;
    }
    case ACCEL_DSHUFTI:
        fprintf(f, "\n");
        fprintf(f, "lo1 %s\n",
                dumpMask((const u8 *)&accel.dshufti.lo1, 128).c_str());
        fprintf(f, "hi1 %s\n",
                dumpMask((const u8 *)&accel.dshufti.hi1, 128).c_str());
        fprintf(f, "lo2 %s\n",
                dumpMask((const u8 *)&accel.dshufti.lo2, 128).c_str());
        fprintf(f, "hi2 %s\n",
                dumpMask((const u8 *)&accel.dshufti.hi2, 128).c_str());
        break;
    case ACCEL_TRUFFLE: {
        fprintf(f, "\n");
        fprintf(f, "lo %s\n",
                dumpMask((const u8 *)&accel.truffle.mask1, 128).c_str());
        fprintf(f, "hi %s\n",
                dumpMask((const u8 *)&accel.truffle.mask2, 128).c_str());
        CharReach cr = truffle2cr(accel.truffle.mask1, accel.truffle.mask2);
        fprintf(f, "count %zu class %s\n", cr.count(),
                describeClass(cr).c_str());
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
    default:
        fprintf(f, "\n");
        break;
    }
}

} // namespace ue2
