/*
 * Copyright (c) 2015-2018, Intel Corporation
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

enum MatchMode {
    CALLBACK_OUTPUT,
    STOP_AT_MATCH,
    NO_MATCHES
};

static really_inline
const struct mstate_aux *get_aux(const struct mcclellan *m, u32 s) {
    const char *nfa = (const char *)m - sizeof(struct NFA);
    const struct mstate_aux *aux
        = s + (const struct mstate_aux *)(nfa + m->aux_offset);

    assert(ISALIGNED(aux));
    return aux;
}

static really_inline
u32 mcclellanEnableStarts(const struct mcclellan *m, u32 s) {
    const struct mstate_aux *aux = get_aux(m, s);

    DEBUG_PRINTF("enabling starts %u->%hu\n", s, aux->top);
    return aux->top;
}

static really_inline
u32 doSherman16(const char *sherman_state, u8 cprime, const u16 *succ_table,
                u32 as) {
    assert(ISALIGNED_N(sherman_state, 16));

    u8 len = *(const u8 *)(sherman_state + SHERMAN_LEN_OFFSET);

    if (len) {
        m128 ss_char = load128(sherman_state);
        m128 cur_char = set16x8(cprime);

        u32 z = movemask128(eq128(ss_char, cur_char));

        /* remove header cruft: type 1, len 1, daddy 2*/
        z &= ~0xf;
        z &= (1U << (len + 4)) - 1;

        if (z) {
            u32 i = ctz32(z & ~0xf) - 4;

            u32 s_out = unaligned_load_u16((const u8 *)sherman_state
                                           + SHERMAN_STATES_OFFSET(len)
                                           + sizeof(u16) * i);
            DEBUG_PRINTF("found sherman match at %u/%u for c'=%hhu s=%u\n", i,
                         len, cprime, s_out);
            return s_out;
        }
    }

    u32 daddy = *(const u16 *)(sherman_state + SHERMAN_DADDY_OFFSET);
    return succ_table[(daddy << as) + cprime];
}

static really_inline
u16 doWide16(const char *wide_entry, const u8 **c_inout, const u8 *end,
             const u8 *remap, const u16 *s, char *qstate, u16 *offset) {
    // Internal relative offset after the last visit of the wide state.
    if (qstate != NULL) { // stream mode
        *offset = unaligned_load_u16((const u16 *)(qstate + 2));
    }

    u8 successful = 0;
    const u8 *c = *c_inout;
    u32 len_c = end - c;

    u16 width = *(const u16 *)(wide_entry + WIDE_WIDTH_OFFSET);
    assert(width >= 8);
    const u8 *symbols = (const u8 *)(wide_entry + WIDE_SYMBOL_OFFSET16);
    const u16 *trans = (const u16 *)(wide_entry +
                                     WIDE_TRANSITION_OFFSET16(width));

    assert(*offset < width);
    u16 len_w = width - *offset;
    const u8 *sym = symbols + *offset;

    char tmp[16];
    u16 pos = 0;

    if (*offset == 0 && remap[*c] != *sym) {
        goto normal;
    }

    // both in (16, +oo).
    while (len_w >= 16 && len_c >= 16) {
        m128 str_w = loadu128(sym);
        for (size_t i = 0; i < 16; i++) {
            tmp[i] = remap[*(c + i)];
        }
        m128 str_c = loadu128(tmp);

        u32 z = movemask128(eq128(str_w, str_c));
        pos = ctz32(~z);
        assert(pos <= 16);

        if (pos < 16) {
            goto normal;
        }

        sym += 16;
        c += 16;
        len_w -= 16;
        len_c -= 16;
    }

    pos = 0;
    // at least one in (0, 16).
    u32 loadLength_w = MIN(len_w, 16);
    u32 loadLength_c = MIN(len_c, 16);
    m128 str_w = loadbytes128(sym, loadLength_w);
    for (size_t i = 0; i < loadLength_c; i++) {
        tmp[i] = remap[*(c + i)];
    }
    m128 str_c = loadbytes128(tmp, loadLength_c);

    u32 z = movemask128(eq128(str_w, str_c));
    pos = ctz32(~z);

    pos = MIN(pos, MIN(loadLength_w, loadLength_c));

    if (loadLength_w <= loadLength_c) {
        assert(pos <= loadLength_w);
        // successful matching.
        if (pos == loadLength_w) {
            c -= 1;
            successful = 1;
        }
        // failure, do nothing.
    } else {
        assert(pos <= loadLength_c);
        // successful partial matching.
        if (pos == loadLength_c) {
            c -= 1;
            goto partial;
        }
        // failure, do nothing.
    }

normal:
    *offset = 0;
    if (qstate != NULL) {
        // Internal relative offset.
        unaligned_store_u16(qstate + 2, *offset);
    }
    c += pos;
    *c_inout = c;
    return successful ? *trans : *(trans + 1 + remap[*c]);

partial:
    *offset = sym - symbols + pos;
    if (qstate != NULL) {
        // Internal relative offset.
        unaligned_store_u16(qstate + 2, *offset);
    }
    c += pos;
    *c_inout = c;
    return *s;
}
