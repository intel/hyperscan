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

#include "mcclellandump.h"

#include "accel.h"
#include "accel_dump.h"
#include "nfa_dump_internal.h"
#include "nfa_internal.h"
#include "mcclellan_internal.h"
#include "rdfa.h"
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

const mstate_aux *getAux(const NFA *n, dstate_id_t i) {
    assert(n && isDfaType(n->type));

    const mcclellan *m = (const mcclellan *)getImplNfa(n);
    const mstate_aux *aux_base
        = (const mstate_aux *)((const char *)n + m->aux_offset);

    const mstate_aux *aux = aux_base + i;

    assert((const char *)aux < (const char *)n + m->length);
    return aux;
}

static
void mcclellanGetTransitions(const NFA *n, u16 s, u16 *t) {
    assert(isMcClellanType(n->type));
    const mcclellan *m = (const mcclellan *)getImplNfa(n);
    const mstate_aux *aux = getAux(n, s);
    const u32 as = m->alphaShift;

    if (n->type == MCCLELLAN_NFA_8) {
        const u8 *succ_table = (const u8 *)((const char *)m
                                            + sizeof(mcclellan));
        for (u16 c = 0; c < N_CHARS; c++) {
            t[c] = succ_table[((u32)s << as) + m->remap[c]];
        }
    } else {
        u16 base_s = s;
        const char *winfo_base = (const char *)n + m->sherman_offset;
        const char *state_base
                = winfo_base + SHERMAN_FIXED_SIZE * (s - m->sherman_limit);

        if (s >= m->sherman_limit) {
            base_s = unaligned_load_u16(state_base + SHERMAN_DADDY_OFFSET);
        }

        const u16 *succ_table = (const u16 *)((const char *)m
                                              + sizeof(mcclellan));
        for (u16 c = 0; c < N_CHARS; c++) {
            const u8 *addr = (const u8*)(succ_table + (((u32)base_s << as)
                                                       + m->remap[c]));
            t[c] = unaligned_load_u16(addr);
            t[c] &= STATE_MASK;
        }

        if (s >= m->sherman_limit) {
            UNUSED char type = *(state_base + SHERMAN_TYPE_OFFSET);
            assert(type == SHERMAN_STATE);
            u8 len = *(const u8 *)(SHERMAN_LEN_OFFSET + state_base);
            const char *chars = state_base + SHERMAN_CHARS_OFFSET;
            const u16 *states = (const u16 *)(state_base
                                              + SHERMAN_STATES_OFFSET(len));

            for (u8 i = 0; i < len; i++) {
                for (u16 c = 0; c < N_CHARS; c++) {
                    if (m->remap[c] == chars[i]) {
                        t[c] = unaligned_load_u16((const u8*)&states[i]) & STATE_MASK;
                    }
                }
            }
        }
    }

    t[TOP] = aux->top & STATE_MASK;
}

void describeEdge(FILE *f, const u16 *t, u16 i) {
    for (u16 s = 0; s < N_CHARS; s++) {
        if (!t[s]) {
            continue;
        }

        u16 ss;
        for (ss = 0; ss < s; ss++) {
            if (t[s] == t[ss]) {
                break;
            }
        }

        if (ss != s) {
            continue;
        }

        CharReach reach;
        for (ss = s; ss < 256; ss++) {
            if (t[s] == t[ss]) {
                reach.set(ss);
            }
        }

        fprintf(f, "%u -> %u [ label = \"", i, t[s]);

        describeClass(f, reach, 5, CC_OUT_DOT);

        fprintf(f, "\" ];\n");
    }
}

void dumpAccelText(FILE *f, const union AccelAux *accel) {
    switch(accel->accel_type) {
    case ACCEL_NONE:
        break;
    case ACCEL_VERM:
        fprintf(f, ":V");
        break;
    case ACCEL_DVERM:
        fprintf(f, ":VV");
        break;
    case ACCEL_VERM_NOCASE:
        fprintf(f, ":VN");
        break;
    case ACCEL_DVERM_NOCASE:
        fprintf(f, ":VVN");
        break;
    case ACCEL_SHUFTI:
        fprintf(f, ":S");
        break;
    case ACCEL_DSHUFTI:
        fprintf(f, ":SS");
        break;
    case ACCEL_TRUFFLE:
        fprintf(f, ":M");
        break;
    default:
        fprintf(f, ":??");
        break;
    }
}

void dumpAccelDot(FILE *f, u16 i, const union AccelAux *accel) {
    switch(accel->accel_type) {
    case ACCEL_NONE:
        break;
    case ACCEL_VERM:
    case ACCEL_VERM_NOCASE:
    case ACCEL_DVERM:
    case ACCEL_DVERM_NOCASE:
        fprintf(f, "%u [ color = forestgreen style=diagonals];\n", i);
        break;
    case ACCEL_SHUFTI:
    case ACCEL_DSHUFTI:
    case ACCEL_TRUFFLE:
        fprintf(f, "%u [ color = darkgreen style=diagonals ];\n", i);
        break;
    default:
        fprintf(f, "%u [ color = yellow style=diagonals ];\n", i);
        break;
    }
}

static
void describeNode(const NFA *n, const mcclellan *m, u16 i, FILE *f) {
    const mstate_aux *aux = getAux(n, i);

    bool isSherman = m->sherman_limit && i >= m->sherman_limit;

    fprintf(f, "%u [ width = 1, fixedsize = true, fontsize = 12, "
            "label = \"%u%s\" ]; \n", i, i, isSherman ? "w":"");

    if (aux->accel_offset) {
        dumpAccelDot(f, i, (const union AccelAux *)
                     ((const char *)m + aux->accel_offset));
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
        const char *winfo_base = (const char *)n + m->sherman_offset;
        const char *state_base
                = winfo_base + SHERMAN_FIXED_SIZE * (i - m->sherman_limit);
        assert(state_base < (const char *)m + m->length - sizeof(NFA));
        UNUSED u8 type = *(const u8 *)(state_base + SHERMAN_TYPE_OFFSET);
        assert(type == SHERMAN_STATE);
        fprintf(f, "%u [ fillcolor = lightblue style=filled ];\n", i);
        u16 daddy = *(const u16 *)(state_base + SHERMAN_DADDY_OFFSET);
        if (daddy) {
            fprintf(f, "%u -> %u [ color=royalblue style=dashed weight=0.1]\n",
                    i, daddy);
        }
    }
}

void dumpDotPreambleDfa(FILE *f) {
    dumpDotPreamble(f);

    // DFA specific additions.
    fprintf(f, "STARTF [style=invis];\n");
    fprintf(f, "STARTA [style=invis];\n");
    fprintf(f, "0 [style=invis];\n");
}

static
void nfaExecMcClellan16_dumpDot(const NFA *nfa, FILE *f) {
    assert(nfa->type == MCCLELLAN_NFA_16);
    const mcclellan *m = (const mcclellan *)getImplNfa(nfa);

    dumpDotPreambleDfa(f);

    u16 sherman_ceil = m->has_wide == 1 ? m->wide_limit : m->state_count;
    for (u16 i = 1; i < sherman_ceil; i++) {
        describeNode(nfa, m, i, f);

        u16 t[ALPHABET_SIZE];

        mcclellanGetTransitions(nfa, i, t);

        describeEdge(f, t, i);
    }

    fprintf(f, "}\n");
}

static
void nfaExecMcClellan8_dumpDot(const NFA *nfa, FILE *f) {
    assert(nfa->type == MCCLELLAN_NFA_8);
    const mcclellan *m = (const mcclellan *)getImplNfa(nfa);

    dumpDotPreambleDfa(f);

    for (u16 i = 1; i < m->state_count; i++) {
        describeNode(nfa, m, i, f);

        u16 t[ALPHABET_SIZE];

        mcclellanGetTransitions(nfa, i, t);

        describeEdge(f, t, i);
    }

    fprintf(f, "}\n");
}

static
void dumpAccelMasks(FILE *f, const mcclellan *m, const mstate_aux *aux) {
    fprintf(f, "\n");
    fprintf(f, "Acceleration\n");
    fprintf(f, "------------\n");

    u16 sherman_ceil = m->has_wide == 1 ? m->wide_limit : m->state_count;
    for (u16 i = 0; i < sherman_ceil; i++) {
        if (!aux[i].accel_offset) {
            continue;
        }

        const AccelAux *accel = (const AccelAux *)((const char *)m
                                                   + aux[i].accel_offset);
        fprintf(f, "%05hu ", i);
        dumpAccelInfo(f, *accel);
    }
}

void describeAlphabet(FILE *f, const mcclellan *m) {
    map<u8, CharReach> rev;

    for (u16 i = 0; i < N_CHARS; i++) {
        rev[m->remap[i]].clear();
    }

    for (u16 i = 0; i < N_CHARS; i++) {
        rev[m->remap[i]].set(i);
    }

    map<u8, CharReach>::const_iterator it;
    fprintf(f, "\nAlphabet\n");
    for (it = rev.begin(); it != rev.end(); ++it) {
        fprintf(f, "%3hhu: ", it->first);
        describeClass(f, it->second, 10240, CC_OUT_TEXT);
        fprintf(f, "\n");
    }
    fprintf(f, "\n");
}

static
void dumpCommonHeader(FILE *f, const mcclellan *m) {
    fprintf(f, "report: %u, states: %u, length: %u\n", m->arb_report,
            m->state_count, m->length);
    fprintf(f, "astart: %hu, fstart: %hu\n", m->start_anchored,
            m->start_floating);
    fprintf(f, "single accept: %d, has_accel: %d\n",
            !!(int)m->flags & MCCLELLAN_FLAG_SINGLE, m->has_accel);
}

static
void dumpTransitions(FILE *f, const NFA *nfa, const mcclellan *m,
                     const mstate_aux *aux) {
    u16 sherman_ceil = m->has_wide == 1 ? m->wide_limit : m->state_count;
    for (u16 i = 0; i < sherman_ceil; i++) {
        fprintf(f, "%05hu", i);
        if (aux[i].accel_offset) {
            dumpAccelText(f, (const union AccelAux *)((const char *)m +
                                                      aux[i].accel_offset));
        }

        u16 trans[ALPHABET_SIZE];
        mcclellanGetTransitions(nfa, i, trans);

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
        }
        if (N_CHARS == rstart + 1) {
            fprintf(f, " %02x->%hu", rstart, prev);
        } else {
            fprintf(f, " [%02x - %02x]->%hu", rstart, N_CHARS - 1, prev);
        }
        fprintf(f, "\n");
    }
}

static
void nfaExecMcClellan16_dumpText(const NFA *nfa, FILE *f) {
    assert(nfa->type == MCCLELLAN_NFA_16);
    const mcclellan *m = (const mcclellan *)getImplNfa(nfa);
    const mstate_aux *aux =
        (const mstate_aux *)((const char *)nfa + m->aux_offset);

    fprintf(f, "mcclellan 16\n");
    dumpCommonHeader(f, m);
    fprintf(f, "sherman_limit: %d, sherman_end: %d\n", (int)m->sherman_limit,
            (int)m->sherman_end);
    fprintf(f, "\n");

    describeAlphabet(f, m);
    dumpTransitions(f, nfa, m, aux);
    dumpAccelMasks(f, m, aux);

    fprintf(f, "\n");
    dumpTextReverse(nfa, f);
}

static
void nfaExecMcClellan8_dumpText(const NFA *nfa, FILE *f) {
    assert(nfa->type == MCCLELLAN_NFA_8);
    const mcclellan *m = (const mcclellan *)getImplNfa(nfa);
    const mstate_aux *aux =
        (const mstate_aux *)((const char *)nfa + m->aux_offset);

    fprintf(f, "mcclellan 8\n");
    dumpCommonHeader(f, m);
    fprintf(f, "accel_limit: %hu, accept_limit %hu\n", m->accel_limit_8,
            m->accept_limit_8);
    fprintf(f, "\n");

    describeAlphabet(f, m);
    dumpTransitions(f, nfa, m, aux);
    dumpAccelMasks(f, m, aux);

    fprintf(f, "\n");
    dumpTextReverse(nfa, f);
}

void nfaExecMcClellan16_dump(const NFA *nfa, const string &base) {
    assert(nfa->type == MCCLELLAN_NFA_16);
    nfaExecMcClellan16_dumpText(nfa, StdioFile(base + ".txt", "w"));
    nfaExecMcClellan16_dumpDot(nfa, StdioFile(base + ".dot", "w"));
}

void nfaExecMcClellan8_dump(const NFA *nfa, const string &base) {
    assert(nfa->type == MCCLELLAN_NFA_8);
    nfaExecMcClellan8_dumpText(nfa, StdioFile(base + ".txt", "w"));
    nfaExecMcClellan8_dumpDot(nfa, StdioFile(base + ".dot", "w"));
}

} // namespace ue2
