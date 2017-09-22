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

#include "config.h"

#include "mpv_dump.h"

#include "mpv_internal.h"
#include "nfa_dump_internal.h"
#include "nfa_internal.h"
#include "ue2common.h"
#include "util/compare.h"
#include "util/dump_mask.h"
#include "util/dump_util.h"

#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cctype>

#ifndef DUMP_SUPPORT
#error No dump support!
#endif

/* Note: No dot files for MPV */

using namespace std;

namespace ue2 {

static really_inline
u32 largest_puff_repeat(const mpv *m, const mpv_kilopuff *kp) {
    return get_puff_array(m, kp)[kp->count - 1].repeats;
}

static
void dumpKilo(FILE *f, const mpv *m, const mpv_kilopuff *k) {
    if (k->auto_restart) {
        fprintf(f, "    AUTO RESTART\n");
    }

    fprintf(f, "    ");

    switch (k->type) {
    case MPV_DOT:
        fprintf(f, "dot\n");
        break;
    case MPV_VERM:
        if (!ourisprint(k->u.verm.c)) {
            fprintf(f, "verm 0x%02x\n", k->u.verm.c);
        } else {
            fprintf(f, "verm 0x%02x '%c'\n", k->u.verm.c, k->u.verm.c);
        }
        break;
    case MPV_SHUFTI:
        fprintf(f, "shufti\n");
        fprintf(f, "lo %s\n",
                dumpMask((const u8 *)&k->u.shuf.mask_lo, 128).c_str());
        fprintf(f, "hi %s\n",
                dumpMask((const u8 *)&k->u.shuf.mask_hi, 128).c_str());
        break;
    case MPV_TRUFFLE:
        fprintf(f, "truffle\n");
        break;
    case MPV_NVERM:
        if (!ourisprint(k->u.verm.c)) {
            fprintf(f, "nverm 0x%02x\n", k->u.verm.c);
        } else {
            fprintf(f, "nverm 0x%02x '%c'\n", k->u.verm.c, k->u.verm.c);
        }
        break;
    default:
        fprintf(f, "unknown type: %hhu\n", k->type);
    }

    fprintf(f, "    %u puffettes\n", k->count);
    fprintf(f, "    largest repeat %u\n", largest_puff_repeat(m, k));
    fprintf(f, "    dead point %llu\n", k->dead_point);
    fprintf(f, "    counter offset %u\n", k->counter_offset);
    fprintf(f, "    puffette offset %u\n", k->puffette_offset);

    const mpv_puffette *p = get_puff_array(m, k);
    for (u32 i = 0; i < k->count; i++) {
        fprintf(f, "\n");
        fprintf(f, "    Puffette %u\n", i);
        fprintf(f, "        repeats:   %u%s\n", p[i].repeats,
                p[i].unbounded ? "," : "");
        if (p[i].simple_exhaust) {
            fprintf(f, "        simple exhaustible\n");
        }
        fprintf(f, "        report id: %u\n", p[i].report);
    }

    fprintf(f, "\n");
}

static
void dumpCounter(FILE *f, const mpv_counter_info *c) {
    fprintf(f, "    max value %llu\n", c->max_counter);
    fprintf(f, "    state offset %u\n", c->counter_offset);
    fprintf(f, "    used by kilopuffs %u - %u\n", c->kilo_begin,
            c->kilo_end - 1);
    fprintf(f, "    bytes %u\n", c->counter_size);
    fprintf(f, "\n");
}

void nfaExecMpv_dump(const NFA *nfa, const string &base) {
    const mpv *m = (const mpv *)getImplNfa(nfa);

    StdioFile f(base + ".txt", "w");

    fprintf(f, "Puff the Magic Engines\n");
    fprintf(f, "\n");
    fprintf(f, "%u puffettes in %u kilopuffs\n", m->puffette_count,
            m->kilo_count);
    fprintf(f, "initial kilopuffs %u - %u\n", m->top_kilo_begin,
            m->top_kilo_end - 1);

    const mpv_kilopuff *k = (const mpv_kilopuff *)(m + 1);
    for (u32 i = 0; i < m->kilo_count; i++) {
        fprintf(f,  "\nKILOPUFF %u\n", i);
        dumpKilo(f, m, k++);
    }

    const mpv_counter_info *c = get_counter_info(m);
    for (u32 i = 0; i < m->counter_count; i++) {
        fprintf(f,  "\nCOUNTER %u\n", i);
        dumpCounter(f, c++);
    }

    dumpTextReverse(nfa, f);
}

} // namespace ue2
