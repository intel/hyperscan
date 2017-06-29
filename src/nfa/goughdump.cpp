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

#include "goughdump.h"

#include "gough_internal.h"
#include "mcclellandump.h"
#include "nfa_dump_internal.h"
#include "nfa_internal.h"
#include "ue2common.h"
#include "util/charreach.h"
#include "util/dump_charclass.h"
#include "util/dump_util.h"
#include "util/unaligned.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <set>
#include <vector>

#ifndef DUMP_SUPPORT
#error No dump support!
#endif

using namespace std;

namespace ue2 {

static
void goughGetTransitions(const NFA *n, u16 s, u16 *t) {
    assert(isGoughType(n->type));
    const mcclellan *m = (const mcclellan *)getImplNfa(n);
    const mstate_aux *aux = getAux(n, s);
    const u32 as = m->alphaShift;
    const char *sher_base
        = (const char *)m - sizeof(struct NFA) + m->sherman_offset;

    if (n->type == GOUGH_NFA_8) {
        const u8 *succ_table = (const u8 *)((const char *)m + sizeof(mcclellan));
        for (u16 c = 0; c < N_CHARS; c++) {
            t[c] = succ_table[((u32)s << as) + m->remap[c]];
        }
    } else {
        u16 base_s = s;

        if (s >= m->sherman_limit) {
            const char *state_base
                = findShermanState(m, sher_base, m->sherman_limit, s);
            base_s = *(const u16 *)(state_base + SHERMAN_DADDY_OFFSET);
        }

        const u16 *succ_table = (const u16 *)((const char *)m
                                              + sizeof(mcclellan));
        for (u16 c = 0; c < N_CHARS; c++) {
            const u8 *addr
                = (const u8*)(succ_table + (((u32)base_s << as) + m->remap[c]));
            t[c] = unaligned_load_u16(addr);
            t[c] &= STATE_MASK;
        }

        if (s >= m->sherman_limit) {
            const char *state_base
                = findShermanState(m, sher_base, m->sherman_limit, s);
            u8 len = *(const u8 *)(SHERMAN_LEN_OFFSET + state_base);
            const u8 *chars = (const u8 *)state_base + SHERMAN_CHARS_OFFSET;
            const u16 *states
                = (const u16 *)(state_base + SHERMAN_STATES_OFFSET(len));

            for (u8 i = 0; i < len; i++) {
                for (u16 c = 0; c < N_CHARS; c++) {
                    if (m->remap[c] != chars[i]) {
                        t[c] = unaligned_load_u16((const u8*)&states[i])
                             & STATE_MASK;
                    }
                }
            }
        }
    }

    t[TOP] = aux->top & STATE_MASK;
}

static
void describeNode(const NFA *n, const mcclellan *m, u16 i, FILE *f) {
    const mstate_aux *aux = getAux(n, i);

    bool isSherman = m->sherman_limit && i >= m->sherman_limit;
    const char *sher_base
        = (const char *)m - sizeof(NFA) + m->sherman_offset;

    fprintf(f, "%u [ width = 1, fixedsize = true, fontsize = 12, "
            "label = \"%u%s\" ]; \n", i, i, isSherman ? "w":"");

    if (aux->accel_offset) {
        dumpAccelDot(f, i,
          &((const gough_accel *)((const char *)m + aux->accel_offset))->accel);
    }

    if (aux->accept_eod) {
        fprintf(f, "%u [ color = darkorchid ];\n", i);
    }

    if (aux->accept) {
        fprintf(f, "%u [ shape = doublecircle ];\n", i);
    }

    if (aux->top && aux->top != i) {
        fprintf(f, "%u -> %u [color = darkgoldenrod weight=0.1 ]\n", i,
                aux->top);
    }

    if (i == m->start_anchored) {
        fprintf(f, "STARTA -> %u [color = blue ]\n", i);
    }

    if (i == m->start_floating) {
        fprintf(f, "STARTF -> %u [color = red ]\n", i);
    }

    if (isSherman) {
        const char *sherman_state
            = findShermanState(m, sher_base, m->sherman_limit, i);
        fprintf(f, "%u [ fillcolor = lightblue style=filled ];\n", i);
        u16 daddy = *(const u16 *)(sherman_state + SHERMAN_DADDY_OFFSET);
        if (daddy) {
            fprintf(f, "%u -> %u [ color=royalblue style=dashed weight=0.1]\n",
                    i, daddy);
        }
    }
}

static
void dump_program(FILE *f, const pair<u32, u32> &e, const gough_ins *prog) {
    fprintf(f, "edge_%u_%u:\n", e.first, e.second);
    for (const gough_ins *it = prog;; ++it) {
        fprintf(f, "\t");
        u32 s = it->src;
        u32 d = it->dest;
        switch (it->op) {
        case GOUGH_INS_END:
            fprintf(f, "END");
            fprintf(f, "\n");
            return;
        case GOUGH_INS_MOV:
            fprintf(f, "MOV %u %u", d, s);
            break;
        case GOUGH_INS_NEW:
            fprintf(f, "NEW-%u %u", s, d);
            break;
        case GOUGH_INS_MIN:
            fprintf(f, "MIN %u %u", d, s);
            break;
        default:
            fprintf(f, "<UNKNOWN>");
            fprintf(f, "\n");
            return;
        }
        fprintf(f, "\n");
    }
}

static
void dump_programs(FILE *f, const NFA *nfa,
                   const set<pair<pair<u32, u32>, u32 > > &prog_dump) {
    fprintf(f, "Edge Programs\n");
    fprintf(f, "-------------\n");
    for (set<pair<pair<u32, u32>, u32 > >::const_iterator it
             = prog_dump.begin(); it != prog_dump.end(); ++it) {
        assert(it->second);
        const gough_ins *p = (const gough_ins *)((const u8 *)nfa + it->second);
        dump_program(f, it->first, p);
    }
}

static
void dumpTransitions(const NFA *nfa, FILE *f,
                     set<pair<pair<u32, u32>, u32 > > *prog_dump) {
    const mcclellan *m = (const mcclellan *)getImplNfa(nfa);
    const gough_info *g = get_gough(m);
    u32 alphaSize = 1U << m->alphaShift;
    const u32 *prog_offset_table = (const u32 *)(g + 1);

    for (u16 i = 0; i < m->state_count; i++) {
        fprintf(f, "%05hu", i);
        const mstate_aux *aux = getAux(nfa, i);

        if (aux->accel_offset) {
            dumpAccelText(f, (const union AccelAux *)((const char *)m +
                                                      aux->accel_offset));
        }

        u16 trans[ALPHABET_SIZE];
        goughGetTransitions(nfa, i, trans);

        int rstart = 0;
        u16 prev = 0xffff;
        for (int j = 0; j < N_CHARS; j++) {
            u16 curr = trans[j];
            if (curr == prev) {
                continue;
            }

            if (prev != 0xffff) {
                if (j == rstart + 1) {
                    fprintf(f, " %02x->%hu", rstart, prev);
                } else {
                    fprintf(f, " [%02x - %02x]->%hu", rstart, j - 1, prev);
                }
            }

            prev = curr;
            rstart = j;

            u32 edge_index = i * alphaSize + m->remap[j];
            u32 prog_offset = prog_offset_table[edge_index];
            if (prog_offset) {
                prog_dump->insert(make_pair(make_pair((u32)i, (u32)trans[j]),
                                            prog_offset));
            }
        }
        if (N_CHARS == rstart + 1) {
            fprintf(f, " %02x->%hu", rstart, prev);
        } else {
            fprintf(f, " [%02x - %02x]->%hu", rstart, N_CHARS - 1, prev);
        }
        fprintf(f, " TOP->%hu\n", trans[TOP]);
        fprintf(f, "\n");
    }

    fprintf(f, "\n");
}

static
void nfaExecGough8_dumpDot(const struct NFA *nfa, FILE *f) {
    assert(nfa->type == GOUGH_NFA_8);
    const mcclellan *m = (const mcclellan *)getImplNfa(nfa);

    dumpDotPreambleDfa(f);

    for (u16 i = 1; i < m->state_count; i++) {
        describeNode(nfa, m, i, f);

        u16 t[ALPHABET_SIZE];

        goughGetTransitions(nfa, i, t);

        describeEdge(f, t, i);
    }

    fprintf(f, "}\n");
}

static
void nfaExecGough8_dumpText(const struct NFA *nfa, FILE *f) {

    assert(nfa->type == GOUGH_NFA_8);
    const mcclellan *m = (const mcclellan *)getImplNfa(nfa);

    fprintf(f, "gough 8\n");
    fprintf(f, "report: %u, states %u, length %u\n", m->arb_report,
            m->state_count, m->length);
    fprintf(f, "astart: %hu, fstart %hu\n", m->start_anchored,
            m->start_floating);
    fprintf(f, "accel_limit: %hu, accept_limit %hu\n", m->accel_limit_8,
            m->accept_limit_8);
    fprintf(f, "\n");

    describeAlphabet(f, m);

    set<pair<pair<u32, u32>, u32 > > prog_dump;

    dumpTransitions(nfa, f, &prog_dump);
    dump_programs(f, nfa, prog_dump);

    dumpTextReverse(nfa, f);
}

static
void nfaExecGough16_dumpDot(const struct NFA *nfa, FILE *f) {
    assert(nfa->type == GOUGH_NFA_16);
    const mcclellan *m = (const mcclellan *)getImplNfa(nfa);

    dumpDotPreambleDfa(f);

    for (u16 i = 1; i < m->state_count; i++) {
        describeNode(nfa, m, i, f);

        u16 t[ALPHABET_SIZE];

        goughGetTransitions(nfa, i, t);

        describeEdge(f, t, i);
    }

    fprintf(f, "}\n");
}

static
void nfaExecGough16_dumpText(const struct NFA *nfa, FILE *f) {
    assert(nfa->type == GOUGH_NFA_16);
    const mcclellan *m = (const mcclellan *)getImplNfa(nfa);
    //    const gough_info *h = get_gough(m);

    fprintf(f, "gough 16\n");
    fprintf(f, "report: %u, states: %u, length: %u\n", m->arb_report,
            m->state_count, m->length);
    fprintf(f, "astart: %hu, fstart: %hu\n", m->start_anchored,
            m->start_floating);
    fprintf(f, "single accept: %d\n", !!(int)m->flags & MCCLELLAN_FLAG_SINGLE);
    fprintf(f, "sherman_limit: %u, sherman_end: %u\n", m->sherman_limit,
            m->sherman_end);

    describeAlphabet(f, m);

    set<pair<pair<u32, u32>, u32 > > prog_dump;

    dumpTransitions(nfa, f, &prog_dump);
    dump_programs(f, nfa, prog_dump);

    fprintf(f, "\n");
    dumpTextReverse(nfa, f);
}

void nfaExecGough16_dump(const NFA *nfa, const string &base) {
    assert(nfa->type == GOUGH_NFA_16);
    nfaExecGough16_dumpText(nfa, StdioFile(base + ".txt", "w"));
    nfaExecGough16_dumpDot(nfa, StdioFile(base + ".dot", "w"));
}

void nfaExecGough8_dump(const NFA *nfa, const string &base) {
    assert(nfa->type == GOUGH_NFA_8);
    nfaExecGough8_dumpText(nfa, StdioFile(base + ".txt", "w"));
    nfaExecGough8_dumpDot(nfa, StdioFile(base + ".dot", "w"));
}

} // namespace ue2
