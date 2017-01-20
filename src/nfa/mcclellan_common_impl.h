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
