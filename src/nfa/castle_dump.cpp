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
 * \brief Castle: multi-tenant repeat engine, dump code.
 */

#include "config.h"

#include "castle_dump.h"

#include "castle_internal.h"
#include "nfa_dump_internal.h"
#include "nfa_internal.h"
#include "shufticompile.h"
#include "trufflecompile.h"
#include "util/charreach.h"
#include "util/dump_util.h"
#include "util/dump_charclass.h"

#ifndef DUMP_SUPPORT
#error No dump support!
#endif

/* Note: No dot files for castle */

using namespace std;

namespace ue2 {

static
void dumpTextSubCastle(const SubCastle &sub, FILE *f) {
    const RepeatInfo *info =
        (const RepeatInfo *)((const char *)&sub + sub.repeatInfoOffset);
    fprintf(f, "  repeat model:          %s\n", repeatTypeName(info->type));
    fprintf(f, "  repeat bounds:         {%u, %u}\n", info->repeatMin,
            info->repeatMax);
    fprintf(f, "  min period:            %u\n", info->minPeriod);

    fprintf(f, "  report:                %u\n", sub.report);
    fprintf(f, "  full state offset:     %u\n", sub.fullStateOffset);
    fprintf(f, "  stream state offset:   %u\n", sub.streamStateOffset);
    fprintf(f, "\n");
}

void nfaExecCastle_dump(const struct NFA *nfa, const string &base) {
    const Castle *c = (const Castle *)getImplNfa(nfa);

    StdioFile f(base + ".txt", "w");

    fprintf(f, "Castle multi-tenant repeat engine\n");
    fprintf(f, "\n");
    fprintf(f, "Number of repeat tenants:  %u\n", c->numRepeats);
    fprintf(f, "Scan type:                 ");
    switch (c->type) {
    case CASTLE_DOT:
        fprintf(f, "dot\n");
        break;
    case CASTLE_VERM:
        fprintf(f, "verm, scanning for 0x%02x\n", c->u.verm.c);
        break;
    case CASTLE_NVERM:
        fprintf(f, "negated verm, scanning for 0x%02x\n", c->u.verm.c);
        break;
    case CASTLE_SHUFTI: {
        const CharReach cr = shufti2cr((const u8 *)&c->u.shuf.mask_lo,
                                       (const u8 *)&c->u.shuf.mask_hi);
        fprintf(f, "shufti, scanning for %s (%zu chars)\n",
                describeClass(cr).c_str(), cr.count());
        break;
    }
    case CASTLE_TRUFFLE: {
        const CharReach cr = truffle2cr((const u8 *)&c->u.truffle.mask1,
                                        (const u8 *)&c->u.truffle.mask2);
        fprintf(f, "truffle, scanning for %s (%zu chars)\n",
                describeClass(cr).c_str(), cr.count());
        break;
    }
    default:
        fprintf(f, "unknown type %u\n", c->type);
        break;
    }
    fprintf(f, "Stale Iter Offset:          %u\n", c->staleIterOffset);

    fprintf(f, "\n");
    dumpTextReverse(nfa, f);
    fprintf(f, "\n");

    const SubCastle *sub =
        (const SubCastle *)((const char *)c + sizeof(Castle));
    for (u32 i = 0; i < c->numRepeats; i++) {
        fprintf(f, "Sub %u:\n", i);
        dumpTextSubCastle(sub[i], f);
    }
}

} // namespace ue2
