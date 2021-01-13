/*
 * Copyright (c) 2016-2020, Intel Corporation
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

#include "mcsheng_dump.h"

#include "accel.h"
#include "accel_dump.h"
#include "nfa_dump_internal.h"
#include "nfa_internal.h"
#include "mcsheng_internal.h"
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

#ifndef DUMP_SUPPORT
#error No dump support!
#endif

using namespace std;

namespace ue2 {

static
const mstate_aux *getAux(const NFA *n, dstate_id_t i) {
    auto *m = (const mcsheng *)getImplNfa(n);
    auto *aux_base = (const mstate_aux *)((const char *)n + m->aux_offset);

    const mstate_aux *aux = aux_base + i;

    assert((const char *)aux < (const char *)n + m->length);
    return aux;
}

static
void next_states(const NFA *n, u16 s, u16 *t) {
    const mcsheng *m = (const mcsheng *)getImplNfa(n);
    const mstate_aux *aux = getAux(n, s);
    const u32 as = m->alphaShift;
    assert(s != DEAD_STATE);

    if (s < m->sheng_end) {
        for (u16 c = 0; c < N_CHARS; c++) {
            u8 sheng_s = s - 1;
            auto trans_for_c = (const char *)&m->sheng_masks[c];
            assert(sheng_s < sizeof(m128));
            u8 raw_succ = trans_for_c[sheng_s];
            if (raw_succ == m->sheng_end - 1) {
                t[c] = DEAD_STATE;
            } else if (raw_succ < m->sheng_end) {
                t[c] = raw_succ + 1;
            } else {
                t[c] = raw_succ;
            }
        }
    } else  if (n->type == MCSHENG_NFA_8) {
        const u8 *succ_table = (const u8 *)((const char *)m + sizeof(mcsheng));
        for (u16 c = 0; c < N_CHARS; c++) {
            u32 normal_id = s - m->sheng_end;
            t[c] = succ_table[(normal_id << as) + m->remap[c]];
        }
    } else {
        u16 base_s = s;
        const char *winfo_base = (const char *)n + m->sherman_offset;
        const char *state_base
                = winfo_base + SHERMAN_FIXED_SIZE * (s - m->sherman_limit);

        if (s >= m->sherman_limit) {
            base_s = unaligned_load_u16(state_base + SHERMAN_DADDY_OFFSET);
            assert(base_s >= m->sheng_end);
        }

        const u16 *succ_table = (const u16 *)((const char *)m
                                              + sizeof(mcsheng));
        for (u16 c = 0; c < N_CHARS; c++) {
            u32 normal_id = base_s - m->sheng_end;
            t[c] = succ_table[(normal_id << as) + m->remap[c]];
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
                        t[c] = unaligned_load_u16((const u8*)&states[i]);
                    }
                }
            }
        }

        for (u16 c = 0; c < N_CHARS; c++) {
            t[c] &= STATE_MASK;
        }

    }

    t[TOP] = aux->top & STATE_MASK;
}

static
void describeEdge(FILE *f, const mcsheng *m, const u16 *t, u16 i) {
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

        fprintf(f, "%u -> %u [ ", i, t[s]);
        if (i < m->sheng_end && t[s] < m->sheng_end) {
            fprintf(f, "color = red, fontcolor = red ");
        }
        fprintf(f, "label = \"");
        describeClass(f, reach, 5, CC_OUT_DOT);

        fprintf(f, "\" ];\n");
    }
}

static
const mstate_aux *getAux64(const NFA *n, dstate_id_t i) {
    auto *m = (const mcsheng64 *)getImplNfa(n);
    auto *aux_base = (const mstate_aux *)((const char *)n + m->aux_offset);

    const mstate_aux *aux = aux_base + i;

    assert((const char *)aux < (const char *)n + m->length);
    return aux;
}

static
void next_states64(const NFA *n, u16 s, u16 *t) {
    const mcsheng64 *m = (const mcsheng64 *)getImplNfa(n);
    const mstate_aux *aux = getAux64(n, s);
    const u32 as = m->alphaShift;
    assert(s != DEAD_STATE);

    if (s < m->sheng_end) {
        for (u16 c = 0; c < N_CHARS; c++) {
            u8 sheng_s = s - 1;
            auto trans_for_c = (const char *)&m->sheng_succ_masks[c];
            assert(sheng_s < sizeof(m512));
            u8 raw_succ = trans_for_c[sheng_s];
            if (raw_succ == m->sheng_end - 1) {
                t[c] = DEAD_STATE;
            } else if (raw_succ < m->sheng_end) {
                t[c] = raw_succ + 1;
            } else {
                t[c] = raw_succ;
            }
        }
    } else  if (n->type == MCSHENG_64_NFA_8) {
        const u8 *succ_table = (const u8 *)((const char *)m + sizeof(mcsheng64));
        for (u16 c = 0; c < N_CHARS; c++) {
            u32 normal_id = s - m->sheng_end;
            t[c] = succ_table[(normal_id << as) + m->remap[c]];
        }
    } else {
        u16 base_s = s;
        const char *winfo_base = (const char *)n + m->sherman_offset;
        const char *state_base
                = winfo_base + SHERMAN_FIXED_SIZE * (s - m->sherman_limit);

        if (s >= m->sherman_limit) {
            base_s = unaligned_load_u16(state_base + SHERMAN_DADDY_OFFSET);
            assert(base_s >= m->sheng_end);
        }

        const u16 *succ_table = (const u16 *)((const char *)m
                                              + sizeof(mcsheng64));
        for (u16 c = 0; c < N_CHARS; c++) {
            u32 normal_id = base_s - m->sheng_end;
            t[c] = succ_table[(normal_id << as) + m->remap[c]];
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
                        t[c] = unaligned_load_u16((const u8*)&states[i]);
                    }
                }
            }
        }

        for (u16 c = 0; c < N_CHARS; c++) {
            t[c] &= STATE_MASK;
        }

    }

    t[TOP] = aux->top & STATE_MASK;
}

static
void describeEdge64(FILE *f, const mcsheng64 *m, const u16 *t, u16 i) {
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

        fprintf(f, "%u -> %u [ ", i, t[s]);
        if (i < m->sheng_end && t[s] < m->sheng_end) {
            fprintf(f, "color = red, fontcolor = red ");
        }
        fprintf(f, "label = \"");
        describeClass(f, reach, 5, CC_OUT_DOT);

        fprintf(f, "\" ];\n");
    }
}

static
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
void describeNode(const NFA *n, const mcsheng *m, u16 i, FILE *f) {
    const mstate_aux *aux = getAux(n, i);

    bool isSherman = m->sherman_limit && i >= m->sherman_limit;

    fprintf(f, "%u [ width = 1, fixedsize = true, fontsize = 12, "
            "label = \"%u%s\" ]; \n", i, i, isSherman ? "w":"");

    if (aux->accel_offset) {
        dumpAccelDot(f, i, (const union AccelAux *)
                     ((const char *)m + aux->accel_offset));
    }

    if (i && i < m->sheng_end) {
        fprintf(f, "%u [color = red, fontcolor = red]; \n", i);
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

    if (i && i < m->sheng_end) {
        fprintf(f, "subgraph cluster_sheng { %u } \n", i);
    }

}

static
void describeNode64(const NFA *n, const mcsheng64 *m, u16 i, FILE *f) {
    const mstate_aux *aux = getAux64(n, i);

    bool isSherman = m->sherman_limit && i >= m->sherman_limit;

    fprintf(f, "%u [ width = 1, fixedsize = true, fontsize = 12, "
            "label = \"%u%s\" ]; \n", i, i, isSherman ? "w":"");

    if (aux->accel_offset) {
        dumpAccelDot(f, i, (const union AccelAux *)
                     ((const char *)m + aux->accel_offset));
    }

    if (i && i < m->sheng_end) {
        fprintf(f, "%u [color = red, fontcolor = red]; \n", i);
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

    if (i && i < m->sheng_end) {
        fprintf(f, "subgraph cluster_sheng { %u } \n", i);
    }

}

static
void dumpDotPreambleDfa(FILE *f) {
    dumpDotPreamble(f);

    // DFA specific additions.
    fprintf(f, "STARTF [style=invis];\n");
    fprintf(f, "STARTA [style=invis];\n");
    fprintf(f, "0 [style=invis];\n");
    fprintf(f, "subgraph cluster_sheng { style = dashed }\n");
}

static
void dump_dot_16(const NFA *nfa, FILE *f) {
    auto  *m = (const mcsheng *)getImplNfa(nfa);

    dumpDotPreambleDfa(f);

    for (u16 i = 1; i < m->state_count; i++) {
        describeNode(nfa, m, i, f);

        u16 t[ALPHABET_SIZE];

        next_states(nfa, i, t);

        describeEdge(f, m, t, i);
    }

    fprintf(f, "}\n");
}

static
void dump_dot_8(const NFA *nfa, FILE *f) {
    auto m = (const mcsheng *)getImplNfa(nfa);

    dumpDotPreambleDfa(f);

    for (u16 i = 1; i < m->state_count; i++) {
        describeNode(nfa, m, i, f);

        u16 t[ALPHABET_SIZE];

        next_states(nfa, i, t);

        describeEdge(f, m, t, i);
    }

    fprintf(f, "}\n");
}

static
void dumpAccelMasks(FILE *f, const mcsheng *m, const mstate_aux *aux) {
    fprintf(f, "\n");
    fprintf(f, "Acceleration\n");
    fprintf(f, "------------\n");

    for (u16 i = 0; i < m->state_count; i++) {
        if (!aux[i].accel_offset) {
            continue;
        }

        auto accel = (const AccelAux *)((const char *)m + aux[i].accel_offset);
        fprintf(f, "%05hu ", i);
        dumpAccelInfo(f, *accel);
    }
}

static
void describeAlphabet(FILE *f, const mcsheng *m) {
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
void dumpCommonHeader(FILE *f, const mcsheng *m) {
    fprintf(f, "report: %u, states: %u, length: %u\n", m->arb_report,
            m->state_count, m->length);
    fprintf(f, "astart: %hu, fstart: %hu\n", m->start_anchored,
            m->start_floating);
    fprintf(f, "single accept: %d, has_accel: %d\n",
            !!(int)m->flags & MCSHENG_FLAG_SINGLE, m->has_accel);
    fprintf(f, "sheng_end:         %hu\n", m->sheng_end);
    fprintf(f, "sheng_accel_limit: %hu\n", m->sheng_accel_limit);
}

static
void dump_text_16(const NFA *nfa, FILE *f) {
    auto *m = (const mcsheng *)getImplNfa(nfa);
    auto *aux = (const mstate_aux *)((const char *)nfa + m->aux_offset);

    fprintf(f, "mcsheng 16\n");
    dumpCommonHeader(f, m);
    fprintf(f, "sherman_limit: %d, sherman_end: %d\n", (int)m->sherman_limit,
            (int)m->sherman_end);
    fprintf(f, "\n");

    describeAlphabet(f, m);
    dumpAccelMasks(f, m, aux);

    fprintf(f, "\n");
    dumpTextReverse(nfa, f);
}

static
void dump_text_8(const NFA *nfa, FILE *f) {
    auto m = (const mcsheng *)getImplNfa(nfa);
    auto aux = (const mstate_aux *)((const char *)nfa + m->aux_offset);

    fprintf(f, "mcsheng 8\n");
    dumpCommonHeader(f, m);
    fprintf(f, "accel_limit: %hu, accept_limit %hu\n", m->accel_limit_8,
            m->accept_limit_8);
    fprintf(f, "\n");

    describeAlphabet(f, m);
    dumpAccelMasks(f, m, aux);

    fprintf(f, "\n");
    dumpTextReverse(nfa, f);
}

static
void dump64_dot_16(const NFA *nfa, FILE *f) {
    auto  *m = (const mcsheng64 *)getImplNfa(nfa);

    dumpDotPreambleDfa(f);

    for (u16 i = 1; i < m->state_count; i++) {
        describeNode64(nfa, m, i, f);

        u16 t[ALPHABET_SIZE];

        next_states64(nfa, i, t);

        describeEdge64(f, m, t, i);
    }

    fprintf(f, "}\n");
}

static
void dump64_dot_8(const NFA *nfa, FILE *f) {
    auto m = (const mcsheng64 *)getImplNfa(nfa);

    dumpDotPreambleDfa(f);

    for (u16 i = 1; i < m->state_count; i++) {
        describeNode64(nfa, m, i, f);

        u16 t[ALPHABET_SIZE];

        next_states64(nfa, i, t);

        describeEdge64(f, m, t, i);
    }

    fprintf(f, "}\n");
}

static
void dumpAccelMasks64(FILE *f, const mcsheng64 *m, const mstate_aux *aux) {
    fprintf(f, "\n");
    fprintf(f, "Acceleration\n");
    fprintf(f, "------------\n");

    for (u16 i = 0; i < m->state_count; i++) {
        if (!aux[i].accel_offset) {
            continue;
        }

        auto accel = (const AccelAux *)((const char *)m + aux[i].accel_offset);
        fprintf(f, "%05hu ", i);
        dumpAccelInfo(f, *accel);
    }
}

static
void describeAlphabet64(FILE *f, const mcsheng64 *m) {
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
void dumpCommonHeader64(FILE *f, const mcsheng64 *m) {
    fprintf(f, "report: %u, states: %u, length: %u\n", m->arb_report,
            m->state_count, m->length);
    fprintf(f, "astart: %hu, fstart: %hu\n", m->start_anchored,
            m->start_floating);
    fprintf(f, "single accept: %d, has_accel: %d\n",
            !!(int)m->flags & MCSHENG_FLAG_SINGLE, m->has_accel);
    fprintf(f, "sheng_end:         %hu\n", m->sheng_end);
    fprintf(f, "sheng_accel_limit: %hu\n", m->sheng_accel_limit);
}

static
void dump64_text_8(const NFA *nfa, FILE *f) {
    auto m = (const mcsheng64 *)getImplNfa(nfa);
    auto aux = (const mstate_aux *)((const char *)nfa + m->aux_offset);

    fprintf(f, "mcsheng 64-8\n");
    dumpCommonHeader64(f, m);
    fprintf(f, "accel_limit: %hu, accept_limit %hu\n", m->accel_limit_8,
            m->accept_limit_8);
    fprintf(f, "\n");

    describeAlphabet64(f, m);
    dumpAccelMasks64(f, m, aux);

    fprintf(f, "\n");
    dumpTextReverse(nfa, f);
}

static
void dump64_text_16(const NFA *nfa, FILE *f) {
    auto *m = (const mcsheng64 *)getImplNfa(nfa);
    auto *aux = (const mstate_aux *)((const char *)nfa + m->aux_offset);

    fprintf(f, "mcsheng 64-16\n");
    dumpCommonHeader64(f, m);
    fprintf(f, "sherman_limit: %d, sherman_end: %d\n", (int)m->sherman_limit,
            (int)m->sherman_end);
    fprintf(f, "\n");

    describeAlphabet64(f, m);
    dumpAccelMasks64(f, m, aux);

    fprintf(f, "\n");
    dumpTextReverse(nfa, f);
}

void nfaExecMcSheng16_dump(const NFA *nfa, const string &base) {
    assert(nfa->type == MCSHENG_NFA_16);
    dump_text_16(nfa, StdioFile(base + ".txt", "w"));
    dump_dot_16(nfa, StdioFile(base + ".dot", "w"));
}

void nfaExecMcSheng8_dump(const NFA *nfa, const string &base) {
    assert(nfa->type == MCSHENG_NFA_8);
    dump_text_8(nfa, StdioFile(base + ".txt", "w"));
    dump_dot_8(nfa, StdioFile(base + ".dot", "w"));
}

void nfaExecMcSheng64_16_dump(UNUSED const NFA *nfa, UNUSED const string &base) {
    assert(nfa->type == MCSHENG_64_NFA_16);
    dump64_text_16(nfa, StdioFile(base + ".txt", "w"));
    dump64_dot_16(nfa, StdioFile(base + ".dot", "w"));
}

void nfaExecMcSheng64_8_dump(UNUSED const NFA *nfa, UNUSED const string &base) {
    assert(nfa->type == MCSHENG_64_NFA_8);
    dump64_text_8(nfa, StdioFile(base + ".txt", "w"));
    dump64_dot_8(nfa, StdioFile(base + ".dot", "w"));
}

} // namespace ue2
