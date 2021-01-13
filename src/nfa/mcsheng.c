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

#include "mcsheng.h"

#include "accel.h"
#include "mcsheng_internal.h"
#include "nfa_api.h"
#include "nfa_api_queue.h"
#include "nfa_internal.h"
#include "util/arch.h"
#include "util/bitutils.h"
#include "util/compare.h"
#include "util/simd_utils.h"
#include "ue2common.h"

enum MatchMode {
    CALLBACK_OUTPUT,
    STOP_AT_MATCH,
    NO_MATCHES
};

static really_inline
const struct mstate_aux *get_aux(const struct mcsheng *m, u32 s) {
    const char *nfa = (const char *)m - sizeof(struct NFA);
    const struct mstate_aux *aux
        = s + (const struct mstate_aux *)(nfa + m->aux_offset);

    assert(ISALIGNED(aux));
    return aux;
}

static really_inline
u32 mcshengEnableStarts(const struct mcsheng *m, u32 s) {
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
char doComplexReport(NfaCallback cb, void *ctxt, const struct mcsheng *m,
                     u32 s, u64a loc, char eod, u32 *cached_accept_state,
                     u32 *cached_accept_id) {
    DEBUG_PRINTF("reporting state = %u, loc=%llu, eod %hhu\n",
                 s & STATE_MASK, loc, eod);

    if (!eod && s == *cached_accept_state) {
        if (cb(0, loc, *cached_accept_id, ctxt) == MO_HALT_MATCHING) {
            return MO_HALT_MATCHING; /* termination requested */
        }

        return MO_CONTINUE_MATCHING; /* continue execution */
    }

    const struct mstate_aux *aux = get_aux(m, s);
    size_t offset = eod ? aux->accept_eod : aux->accept;

    assert(offset);
    const struct report_list *rl
        = (const void *)((const char *)m + offset - sizeof(struct NFA));
    assert(ISALIGNED(rl));

    DEBUG_PRINTF("report list size %u\n", rl->count);
    u32 count = rl->count;

    if (!eod && count == 1) {
        *cached_accept_state = s;
        *cached_accept_id = rl->report[0];

        DEBUG_PRINTF("reporting %u\n", rl->report[0]);
        if (cb(0, loc, rl->report[0], ctxt) == MO_HALT_MATCHING) {
            return MO_HALT_MATCHING; /* termination requested */
        }

        return MO_CONTINUE_MATCHING; /* continue execution */
    }

    for (u32 i = 0; i < count; i++) {
        DEBUG_PRINTF("reporting %u\n", rl->report[i]);
        if (cb(0, loc, rl->report[i], ctxt) == MO_HALT_MATCHING) {
            return MO_HALT_MATCHING; /* termination requested */
        }
    }

    return MO_CONTINUE_MATCHING; /* continue execution */
}

#define SHENG_CHUNK 8

static really_inline
u32 doSheng(const struct mcsheng *m, const u8 **c_inout, const u8 *soft_c_end,
            const u8 *hard_c_end, u32 s_in, char do_accel) {
    assert(s_in < m->sheng_end);
    assert(s_in); /* should not already be dead */
    assert(soft_c_end <= hard_c_end);
    DEBUG_PRINTF("s_in = %u (adjusted %u)\n", s_in, s_in - 1);
    m128 s = set16x8(s_in - 1);
    const u8 *c = *c_inout;
    const u8 *c_end = hard_c_end - SHENG_CHUNK + 1;
    if (!do_accel) {
        c_end = MIN(soft_c_end, hard_c_end - SHENG_CHUNK + 1);
    }
    const m128 *masks = m->sheng_masks;
    u8 sheng_limit = m->sheng_end - 1; /* - 1: no dead state */
    u8 sheng_stop_limit = do_accel ? m->sheng_accel_limit : sheng_limit;

    /* When we use movd to get a u32 containing our state, it will have 4 lanes
     * all duplicating the state. We can create versions of our limits with 4
     * copies to directly compare against, this prevents us generating code to
     * extract a single copy of the state from the u32 for checking. */
    u32 sheng_stop_limit_x4 = sheng_stop_limit * 0x01010101;

#if defined(HAVE_BMI2) && defined(ARCH_64_BIT)
    u32 sheng_limit_x4 = sheng_limit * 0x01010101;
    m128 simd_stop_limit = set4x32(sheng_stop_limit_x4);
    m128 accel_delta = set16x8(sheng_limit - sheng_stop_limit);
    DEBUG_PRINTF("end %hhu, accel %hu --> limit %hhu\n", sheng_limit,
                 m->sheng_accel_limit, sheng_stop_limit);
#endif

#define SHENG_SINGLE_ITER do {                                             \
        m128 shuffle_mask = masks[*(c++)];                                 \
        s = pshufb_m128(shuffle_mask, s);                                  \
        u32 s_gpr_x4 = movd(s); /* convert to u8 */                        \
        DEBUG_PRINTF("c %hhu (%c) --> s %u\n", c[-1], c[-1], s_gpr_x4);    \
        if (s_gpr_x4 >= sheng_stop_limit_x4) {                             \
            s_gpr = s_gpr_x4;                                              \
            goto exit;                                                     \
        }                                                                  \
    } while (0)

    u8 s_gpr;
    while (c < c_end) {
#if defined(HAVE_BMI2) && defined(ARCH_64_BIT)
        /* This version uses pext for efficiently bitbashing out scaled
         * versions of the bytes to process from a u64a */

        u64a data_bytes = unaligned_load_u64a(c);
        u64a cc0 = pdep64(data_bytes, 0xff0); /* extract scaled low byte */
        data_bytes &= ~0xffULL; /* clear low bits for scale space */
        m128 shuffle_mask0 = load128((const char *)masks + cc0);
        s = pshufb_m128(shuffle_mask0, s);
        m128 s_max = s;
        m128 s_max0 = s_max;
        DEBUG_PRINTF("c %02llx --> s %u\n", cc0 >> 4, movd(s));

#define SHENG_SINGLE_UNROLL_ITER(iter)                                  \
        assert(iter);                                                   \
        u64a cc##iter = pext64(data_bytes, mcsheng_pext_mask[iter]);    \
        assert(cc##iter == (u64a)c[iter] << 4);                         \
        m128 shuffle_mask##iter = load128((const char *)masks + cc##iter); \
        s = pshufb_m128(shuffle_mask##iter, s);                         \
        if (do_accel && iter == 7) {                                    \
            /* in the final iteration we also have to check against accel */ \
            m128 s_temp = sadd_u8_m128(s, accel_delta);                 \
            s_max = max_u8_m128(s_max, s_temp);                         \
        } else {                                                        \
            s_max = max_u8_m128(s_max, s);                              \
        }                                                               \
        m128 s_max##iter = s_max;                                       \
        DEBUG_PRINTF("c %02llx --> s %u max %u\n", cc##iter >> 4,       \
                     movd(s), movd(s_max));

        SHENG_SINGLE_UNROLL_ITER(1);

        SHENG_SINGLE_UNROLL_ITER(2);
        SHENG_SINGLE_UNROLL_ITER(3);

        SHENG_SINGLE_UNROLL_ITER(4);
        SHENG_SINGLE_UNROLL_ITER(5);

        SHENG_SINGLE_UNROLL_ITER(6);
        SHENG_SINGLE_UNROLL_ITER(7);

        if (movd(s_max7) >= sheng_limit_x4) {
            DEBUG_PRINTF("exit found\n");

            /* Explicitly check the last byte as it is more likely as it also
             * checks for acceleration. */
            if (movd(s_max6) < sheng_limit_x4) {
                c += SHENG_CHUNK;
                s_gpr = movq(s);
                assert(s_gpr >= sheng_stop_limit);
                goto exit;
            }

            /* use shift-xor to create a register containing all of the max
             * values */
            m128 blended = rshift64_m128(s_max0, 56);
            blended = xor128(blended, rshift64_m128(s_max1, 48));
            blended = xor128(blended, rshift64_m128(s_max2, 40));
            blended = xor128(blended, rshift64_m128(s_max3, 32));
            blended = xor128(blended, rshift64_m128(s_max4, 24));
            blended = xor128(blended, rshift64_m128(s_max5, 16));
            blended = xor128(blended, rshift64_m128(s_max6, 8));
            blended = xor128(blended, s);
            blended = xor128(blended, rshift64_m128(blended, 8));
            DEBUG_PRINTF("blended %016llx\n", movq(blended));

            m128 final = min_u8_m128(blended, simd_stop_limit);
            m128 cmp = sub_u8_m128(final, simd_stop_limit);
            u64a stops = ~movemask128(cmp);
            assert(stops);
            u32 earliest = ctz32(stops);
            DEBUG_PRINTF("stops %02llx, earliest %u\n", stops, earliest);
            assert(earliest < 8);
            c += earliest + 1;
            s_gpr = movq(blended) >> (earliest * 8);
            assert(s_gpr >= sheng_stop_limit);
            goto exit;
        } else {
            c += SHENG_CHUNK;
        }
#else
        SHENG_SINGLE_ITER;
        SHENG_SINGLE_ITER;
        SHENG_SINGLE_ITER;
        SHENG_SINGLE_ITER;

        SHENG_SINGLE_ITER;
        SHENG_SINGLE_ITER;
        SHENG_SINGLE_ITER;
        SHENG_SINGLE_ITER;
#endif
    }

    assert(c_end - c < SHENG_CHUNK);
    if (c < soft_c_end) {
        assert(soft_c_end - c < SHENG_CHUNK);
        switch (soft_c_end - c) {
        case 7:
            SHENG_SINGLE_ITER; // fallthrough
        case 6:
            SHENG_SINGLE_ITER; // fallthrough
        case 5:
            SHENG_SINGLE_ITER; // fallthrough
        case 4:
            SHENG_SINGLE_ITER; // fallthrough
        case 3:
            SHENG_SINGLE_ITER; // fallthrough
        case 2:
            SHENG_SINGLE_ITER; // fallthrough
        case 1:
            SHENG_SINGLE_ITER; // fallthrough
        }
    }

    assert(c >= soft_c_end);

    s_gpr = movd(s);
exit:
    assert(c <= hard_c_end);
    DEBUG_PRINTF("%zu from end; s %hhu\n", c_end - c, s_gpr);
    assert(c >= soft_c_end || s_gpr >= sheng_stop_limit);
    /* undo state adjustment to match mcclellan view */
    if (s_gpr == sheng_limit) {
        s_gpr = 0;
    } else if (s_gpr < sheng_limit) {
        s_gpr++;
    }

    *c_inout = c;
    return s_gpr;
}

static really_inline
const char *findShermanState(UNUSED const struct mcsheng *m,
                             const char *sherman_base_offset, u32 sherman_base,
                             u32 s) {
    const char *rv
        = sherman_base_offset + SHERMAN_FIXED_SIZE * (s - sherman_base);
    assert(rv < (const char *)m + m->length - sizeof(struct NFA));
    UNUSED u8 type = *(const u8 *)(rv + SHERMAN_TYPE_OFFSET);
    assert(type == SHERMAN_STATE);
    return rv;
}

static really_inline
const u8 *run_mcsheng_accel(const struct mcsheng *m,
                            const struct mstate_aux *aux, u32 s,
                            const u8 **min_accel_offset,
                            const u8 *c, const u8 *c_end) {
    DEBUG_PRINTF("skipping\n");
    u32 accel_offset = aux[s].accel_offset;

    assert(aux[s].accel_offset);
    assert(accel_offset >= m->aux_offset);
    assert(!m->sherman_offset || accel_offset < m->sherman_offset);

    const union AccelAux *aaux = (const void *)((const char *)m + accel_offset);
    const u8 *c2 = run_accel(aaux, c, c_end);

    if (c2 < *min_accel_offset + BAD_ACCEL_DIST) {
        *min_accel_offset = c2 + BIG_ACCEL_PENALTY;
    } else {
        *min_accel_offset = c2 + SMALL_ACCEL_PENALTY;
    }

    if (*min_accel_offset >= c_end - ACCEL_MIN_LEN) {
        *min_accel_offset = c_end;
    }

    DEBUG_PRINTF("advanced %zd, next accel chance in %zd/%zd\n",
                 c2 - c, *min_accel_offset - c2, c_end - c2);

    return c2;
}

static really_inline
u32 doNormal16(const struct mcsheng *m, const u8 **c_inout, const u8 *end,
               u32 s, char do_accel, enum MatchMode mode) {
    const u8 *c = *c_inout;

    const u16 *succ_table
        = (const u16 *)((const char *)m + sizeof(struct mcsheng));
    assert(ISALIGNED_N(succ_table, 2));
    u32 sheng_end = m->sheng_end;
    u32 sherman_base = m->sherman_limit;
    const char *sherman_base_offset
        = (const char *)m - sizeof(struct NFA) + m->sherman_offset;
    u32 as = m->alphaShift;

    /* Adjust start of succ table so we can index into using state id (rather
     * than adjust to normal id). As we will not be processing states with low
     * state ids, we will not be accessing data before the succ table. Note: due
     * to the size of the sheng tables, the succ_table pointer will still be
     * inside the engine.*/
    succ_table -= sheng_end << as;

    s &= STATE_MASK;

    while (c < end && s >= sheng_end) {
        u8 cprime = m->remap[*c];
        DEBUG_PRINTF("c: %02hhx '%c' cp:%02hhx (s=%u)\n", *c,
                     ourisprint(*c) ? *c : '?', cprime, s);
        if (s < sherman_base) {
            DEBUG_PRINTF("doing normal\n");
            assert(s < m->state_count);
            s = succ_table[(s << as) + cprime];
        } else {
            const char *sherman_state
                = findShermanState(m, sherman_base_offset, sherman_base, s);
            DEBUG_PRINTF("doing sherman (%u)\n", s);
            s = doSherman16(sherman_state, cprime, succ_table, as);
        }

        DEBUG_PRINTF("s: %u (%u)\n", s, s & STATE_MASK);
        c++;

        if (do_accel && (s & ACCEL_FLAG)) {
            break;
        }
        if (mode != NO_MATCHES && (s & ACCEPT_FLAG)) {
            break;
        }

        s &= STATE_MASK;
    }

    *c_inout = c;
    return s;
}

static really_inline
char mcshengExec16_i(const struct mcsheng *m, u32 *state, const u8 *buf,
                     size_t len, u64a offAdj, NfaCallback cb, void *ctxt,
                     char single, const u8 **c_final, enum MatchMode mode) {
    assert(ISALIGNED_N(state, 2));
    if (!len) {
        if (mode == STOP_AT_MATCH) {
            *c_final = buf;
        }
        return MO_ALIVE;
    }

    u32 s = *state;
    const u8 *c = buf;
    const u8 *c_end = buf + len;
    const u8 sheng_end = m->sheng_end;
    const struct mstate_aux *aux
        = (const struct mstate_aux *)((const char *)m + m->aux_offset
                                      - sizeof(struct NFA));

    s &= STATE_MASK;

    u32 cached_accept_id = 0;
    u32 cached_accept_state = 0;

    DEBUG_PRINTF("s: %u, len %zu\n", s, len);

    const u8 *min_accel_offset = c;
    if (!m->has_accel || len < ACCEL_MIN_LEN) {
        min_accel_offset = c_end;
        goto without_accel;
    }

    goto with_accel;

without_accel:
    do {
        assert(c < min_accel_offset);
        int do_accept;
        if (!s) {
            goto exit;
        } else if (s < sheng_end) {
            s = doSheng(m, &c, min_accel_offset, c_end, s, 0);
            do_accept = mode != NO_MATCHES && get_aux(m, s)->accept;
        } else {
            s = doNormal16(m, &c, min_accel_offset, s, 0, mode);

            do_accept = mode != NO_MATCHES && (s & ACCEPT_FLAG);
        }

        if (do_accept) {
            if (mode == STOP_AT_MATCH) {
                *state = s & STATE_MASK;
                *c_final = c - 1;
                return MO_MATCHES_PENDING;
            }

            u64a loc = (c - 1) - buf + offAdj + 1;

            if (single) {
                DEBUG_PRINTF("reporting %u\n", m->arb_report);
                if (cb(0, loc, m->arb_report, ctxt) == MO_HALT_MATCHING) {
                    return MO_DEAD; /* termination requested */
                }
            } else if (doComplexReport(cb, ctxt, m, s & STATE_MASK, loc, 0,
                                       &cached_accept_state, &cached_accept_id)
                       == MO_HALT_MATCHING) {
                return MO_DEAD;
            }
        }

        assert(c <= c_end); /* sheng is fuzzy for min_accel_offset */
    } while (c < min_accel_offset);

    if (c == c_end) {
        goto exit;
    }

with_accel:
    do {
        assert(c < c_end);
        int do_accept;

        if (!s) {
            goto exit;
        } else if (s < sheng_end) {
            if (s > m->sheng_accel_limit) {
                c = run_mcsheng_accel(m, aux, s, &min_accel_offset, c, c_end);
                if (c == c_end) {
                    goto exit;
                } else {
                    goto without_accel;
                }
            }
            s = doSheng(m, &c, c_end, c_end, s, 1);
            do_accept = mode != NO_MATCHES && get_aux(m, s)->accept;
        } else {
            if (s & ACCEL_FLAG) {
                DEBUG_PRINTF("skipping\n");
                s &= STATE_MASK;
                c = run_mcsheng_accel(m, aux, s, &min_accel_offset, c, c_end);
                if (c == c_end) {
                    goto exit;
                } else {
                    goto without_accel;
                }
            }

            s = doNormal16(m, &c, c_end, s, 1, mode);
            do_accept = mode != NO_MATCHES && (s & ACCEPT_FLAG);
        }

        if (do_accept) {
            if (mode == STOP_AT_MATCH) {
                *state = s & STATE_MASK;
                *c_final = c - 1;
                return MO_MATCHES_PENDING;
            }

            u64a loc = (c - 1) - buf + offAdj + 1;

            if (single) {
                DEBUG_PRINTF("reporting %u\n", m->arb_report);
                if (cb(0, loc, m->arb_report, ctxt) == MO_HALT_MATCHING) {
                    return MO_DEAD; /* termination requested */
                }
            } else if (doComplexReport(cb, ctxt, m, s & STATE_MASK, loc, 0,
                                       &cached_accept_state, &cached_accept_id)
                       == MO_HALT_MATCHING) {
                return MO_DEAD;
            }
        }

        assert(c <= c_end);
    } while (c < c_end);

exit:
    s &= STATE_MASK;

    if (mode == STOP_AT_MATCH) {
        *c_final = c_end;
    }
    *state = s;

    return MO_ALIVE;
}

static never_inline
char mcshengExec16_i_cb(const struct mcsheng *m, u32 *state, const u8 *buf,
                        size_t len, u64a offAdj, NfaCallback cb, void *ctxt,
                        char single, const u8 **final_point) {
    return mcshengExec16_i(m, state, buf, len, offAdj, cb, ctxt, single,
                           final_point, CALLBACK_OUTPUT);
}

static never_inline
char mcshengExec16_i_sam(const struct mcsheng *m, u32 *state, const u8 *buf,
                         size_t len, u64a offAdj, NfaCallback cb, void *ctxt,
                         char single, const u8 **final_point) {
    return mcshengExec16_i(m, state, buf, len, offAdj, cb, ctxt, single,
                           final_point, STOP_AT_MATCH);
}

static never_inline
char mcshengExec16_i_nm(const struct mcsheng *m, u32 *state, const u8 *buf,
                        size_t len, u64a offAdj, NfaCallback cb, void *ctxt,
                        char single, const u8 **final_point) {
    return mcshengExec16_i(m, state, buf, len, offAdj, cb, ctxt, single,
                           final_point, NO_MATCHES);
}

static really_inline
char mcshengExec16_i_ni(const struct mcsheng *m, u32 *state, const u8 *buf,
                        size_t len, u64a offAdj, NfaCallback cb, void *ctxt,
                        char single, const u8 **final_point,
                        enum MatchMode mode) {
    if (mode == CALLBACK_OUTPUT) {
        return mcshengExec16_i_cb(m, state, buf, len, offAdj, cb, ctxt,
                                  single, final_point);
    } else if (mode == STOP_AT_MATCH) {
        return mcshengExec16_i_sam(m, state, buf, len, offAdj, cb, ctxt,
                                   single, final_point);
    } else {
        assert (mode == NO_MATCHES);
        return mcshengExec16_i_nm(m, state, buf, len, offAdj, cb, ctxt,
                                  single, final_point);
    }
}

static really_inline
u32 doNormal8(const struct mcsheng *m, const u8 **c_inout, const u8 *end, u32 s,
              char do_accel, enum MatchMode mode) {
    const u8 *c = *c_inout;
    u32 sheng_end = m->sheng_end;
    u32 accel_limit = m->accel_limit_8;
    u32 accept_limit = m->accept_limit_8;

    const u32 as = m->alphaShift;
    const u8 *succ_table = (const u8 *)((const char *)m
                                        + sizeof(struct mcsheng));
    /* Adjust start of succ table so we can index into using state id (rather
     * than adjust to normal id). As we will not be processing states with low
     * state ids, we will not be accessing data before the succ table. Note: due
     * to the size of the sheng tables, the succ_table pointer will still be
     * inside the engine.*/
    succ_table -= sheng_end << as;

    assert(s >= sheng_end);

    while (c < end && s >= sheng_end) {
        u8 cprime = m->remap[*c];
        DEBUG_PRINTF("c: %02hhx '%c' cp:%02hhx\n", *c,
                     ourisprint(*c) ? *c : '?', cprime);
        s = succ_table[(s << as) + cprime];

        DEBUG_PRINTF("s: %u\n", s);
        c++;
        if (do_accel) {
            if (s >= accel_limit) {
                break;
            }
        } else {
            if (mode != NO_MATCHES && s >= accept_limit) {
                break;
            }
        }
    }
    *c_inout = c;
    return s;
}

static really_inline
char mcshengExec8_i(const struct mcsheng *m, u32 *state, const u8 *buf,
                    size_t len, u64a offAdj, NfaCallback cb, void *ctxt,
                    char single, const u8 **c_final, enum MatchMode mode) {
    if (!len) {
        *c_final = buf;
        return MO_ALIVE;
    }
    u32 s = *state;
    const u8 *c = buf;
    const u8 *c_end = buf + len;
    const u8 sheng_end = m->sheng_end;

    const struct mstate_aux *aux
        = (const struct mstate_aux *)((const char *)m + m->aux_offset
                                      - sizeof(struct NFA));
    u32 accept_limit = m->accept_limit_8;

    u32 cached_accept_id = 0;
    u32 cached_accept_state = 0;

    DEBUG_PRINTF("accel %hu, accept %u\n", m->accel_limit_8, accept_limit);

    DEBUG_PRINTF("s: %u, len %zu\n", s, len);

    const u8 *min_accel_offset = c;
    if (!m->has_accel || len < ACCEL_MIN_LEN) {
        min_accel_offset = c_end;
        goto without_accel;
    }

    goto with_accel;

without_accel:
    do {
        assert(c < min_accel_offset);
        if (!s) {
            goto exit;
        } else if (s < sheng_end) {
            s = doSheng(m, &c, min_accel_offset, c_end, s, 0);
        } else {
            s = doNormal8(m, &c, min_accel_offset, s, 0, mode);
            assert(c <= min_accel_offset);
        }

        if (mode != NO_MATCHES && s >= accept_limit) {
            if (mode == STOP_AT_MATCH) {
                DEBUG_PRINTF("match - pausing\n");
                *state = s;
                *c_final = c - 1;
                return MO_MATCHES_PENDING;
            }

            u64a loc = (c - 1) - buf + offAdj + 1;
            if (single) {
                DEBUG_PRINTF("reporting %u\n", m->arb_report);
                if (cb(0, loc, m->arb_report, ctxt) == MO_HALT_MATCHING) {
                    return MO_DEAD;
                }
            } else if (doComplexReport(cb, ctxt, m, s, loc, 0,
                                       &cached_accept_state, &cached_accept_id)
                       == MO_HALT_MATCHING) {
                return MO_DEAD;
            }
        }

        assert(c <= c_end); /* sheng is fuzzy for min_accel_offset */
    } while (c < min_accel_offset);

    if (c == c_end) {
        goto exit;
    }

with_accel:
    do {
        u32 accel_limit = m->accel_limit_8;

        assert(c < c_end);
        if (!s) {
            goto exit;
        } else if (s < sheng_end) {
            if (s > m->sheng_accel_limit) {
                c = run_mcsheng_accel(m, aux, s, &min_accel_offset, c, c_end);
                if (c == c_end) {
                    goto exit;
                } else {
                    goto without_accel;
                }
            }
            s = doSheng(m, &c, c_end, c_end, s, 1);
        } else {
            if (s >= accel_limit && aux[s].accel_offset) {
                c = run_mcsheng_accel(m, aux, s, &min_accel_offset, c, c_end);
                if (c == c_end) {
                    goto exit;
                } else {
                    goto without_accel;
                }
            }
            s = doNormal8(m, &c, c_end, s, 1, mode);
        }

        if (mode != NO_MATCHES && s >= accept_limit) {
            if (mode == STOP_AT_MATCH) {
                DEBUG_PRINTF("match - pausing\n");
                *state = s;
                *c_final = c - 1;
                return MO_MATCHES_PENDING;
            }

            u64a loc = (c - 1) - buf + offAdj + 1;
            if (single) {
                DEBUG_PRINTF("reporting %u\n", m->arb_report);
                if (cb(0, loc, m->arb_report, ctxt) == MO_HALT_MATCHING) {
                    return MO_DEAD;
                }
            } else if (doComplexReport(cb, ctxt, m, s, loc, 0,
                                       &cached_accept_state, &cached_accept_id)
                       == MO_HALT_MATCHING) {
                return MO_DEAD;
            }
        }

        assert(c <= c_end);
    } while (c < c_end);

exit:
    *state = s;
    if (mode == STOP_AT_MATCH) {
        *c_final = c_end;
    }
    return MO_ALIVE;
}

static never_inline
char mcshengExec8_i_cb(const struct mcsheng *m, u32 *state, const u8 *buf,
                       size_t len, u64a offAdj, NfaCallback cb, void *ctxt,
                       char single, const u8 **final_point) {
    return mcshengExec8_i(m, state, buf, len, offAdj, cb, ctxt, single,
                          final_point, CALLBACK_OUTPUT);
}

static never_inline
char mcshengExec8_i_sam(const struct mcsheng *m, u32 *state, const u8 *buf,
                        size_t len, u64a offAdj, NfaCallback cb, void *ctxt,
                        char single, const u8 **final_point) {
    return mcshengExec8_i(m, state, buf, len, offAdj, cb, ctxt, single,
                          final_point, STOP_AT_MATCH);
}

static never_inline
char mcshengExec8_i_nm(const struct mcsheng *m, u32 *state, const u8 *buf,
                       size_t len, u64a offAdj, NfaCallback cb, void *ctxt,
                       char single, const u8 **final_point) {
    return mcshengExec8_i(m, state, buf, len, offAdj, cb, ctxt, single,
                          final_point, NO_MATCHES);
}

static really_inline
char mcshengExec8_i_ni(const struct mcsheng *m, u32 *state, const u8 *buf,
                       size_t len, u64a offAdj, NfaCallback cb, void *ctxt,
                       char single, const u8 **final_point,
                       enum MatchMode mode) {
    if (mode == CALLBACK_OUTPUT) {
        return mcshengExec8_i_cb(m, state, buf, len, offAdj, cb, ctxt, single,
                                 final_point);
    } else if (mode == STOP_AT_MATCH) {
        return mcshengExec8_i_sam(m, state, buf, len, offAdj, cb, ctxt,
                                  single, final_point);
    } else {
        assert(mode == NO_MATCHES);
        return mcshengExec8_i_nm(m, state, buf, len, offAdj, cb, ctxt, single,
                                 final_point);
    }
}

static really_inline
char mcshengCheckEOD(const struct NFA *nfa, u32 s, u64a offset,
                     NfaCallback cb, void *ctxt) {
    const struct mcsheng *m = getImplNfa(nfa);
    const struct mstate_aux *aux = get_aux(m, s);

    if (!aux->accept_eod) {
        return MO_CONTINUE_MATCHING;
    }
    return doComplexReport(cb, ctxt, m, s, offset, 1, NULL, NULL);
}

static really_inline
char nfaExecMcSheng16_Q2i(const struct NFA *n, u64a offset, const u8 *buffer,
                          const u8 *hend, NfaCallback cb, void *context,
                          struct mq *q, char single, s64a end,
                          enum MatchMode mode) {
    assert(n->type == MCSHENG_NFA_16);
    const struct mcsheng *m = getImplNfa(n);
    s64a sp;

    assert(ISALIGNED_N(q->state, 2));
    u32 s = *(u16 *)q->state;

    if (q->report_current) {
        assert(s);
        assert(get_aux(m, s)->accept);

        int rv;
        if (single) {
            DEBUG_PRINTF("reporting %u\n", m->arb_report);
            rv = cb(0, q_cur_offset(q), m->arb_report, context);
        } else {
            u32 cached_accept_id = 0;
            u32 cached_accept_state = 0;

            rv = doComplexReport(cb, context, m, s, q_cur_offset(q), 0,
                                 &cached_accept_state, &cached_accept_id);
        }

        q->report_current = 0;

        if (rv == MO_HALT_MATCHING) {
            return MO_DEAD;
        }
    }

    sp = q_cur_loc(q);
    q->cur++;

    const u8 *cur_buf = sp < 0 ? hend : buffer;

    assert(q->cur);
    if (mode != NO_MATCHES && q->items[q->cur - 1].location > end) {
        DEBUG_PRINTF("this is as far as we go\n");
        q->cur--;
        q->items[q->cur].type = MQE_START;
        q->items[q->cur].location = end;
        *(u16 *)q->state = s;
        return MO_ALIVE;
    }

    while (1) {
        assert(q->cur < q->end);
        s64a ep = q->items[q->cur].location;
        if (mode != NO_MATCHES) {
            ep = MIN(ep, end);
        }

        assert(ep >= sp);

        s64a local_ep = ep;
        if (sp < 0) {
            local_ep = MIN(0, ep);
        }

        /* do main buffer region */
        const u8 *final_look;
        char rv = mcshengExec16_i_ni(m, &s, cur_buf + sp, local_ep - sp,
                                     offset + sp, cb, context, single,
                                     &final_look, mode);
        if (rv == MO_DEAD) {
            *(u16 *)q->state = 0;
            return MO_DEAD;
        }
        if (mode == STOP_AT_MATCH && rv == MO_MATCHES_PENDING) {
            DEBUG_PRINTF("this is as far as we go\n");
            DEBUG_PRINTF("state %u final_look %zd\n", s, final_look - cur_buf);

            assert(q->cur);
            assert(final_look != cur_buf + local_ep);

            q->cur--;
            q->items[q->cur].type = MQE_START;
            q->items[q->cur].location = final_look - cur_buf + 1; /* due to
                                                                   * early -1 */
            *(u16 *)q->state = s;
            return MO_MATCHES_PENDING;
        }

        assert(rv == MO_ALIVE);
        assert(q->cur);
        if (mode != NO_MATCHES && q->items[q->cur].location > end) {
            DEBUG_PRINTF("this is as far as we go\n");
            q->cur--;
            q->items[q->cur].type = MQE_START;
            q->items[q->cur].location = end;
            *(u16 *)q->state = s;
            return MO_ALIVE;
        }

        sp = local_ep;

        if (sp == 0) {
            cur_buf = buffer;
        }

        if (sp != ep) {
            continue;
        }

        switch (q->items[q->cur].type) {
        case MQE_TOP:
            assert(sp + offset || !s);
            if (sp + offset == 0) {
                s = m->start_anchored;
                break;
            }
            s = mcshengEnableStarts(m, s);
            break;
        case MQE_END:
            *(u16 *)q->state = s;
            q->cur++;
            return s ? MO_ALIVE : MO_DEAD;
        default:
            assert(!"invalid queue event");
        }

        q->cur++;
    }
}

static really_inline
char nfaExecMcSheng8_Q2i(const struct NFA *n, u64a offset, const u8 *buffer,
                         const u8 *hend, NfaCallback cb, void *context,
                         struct mq *q, char single, s64a end,
                         enum MatchMode mode) {
    assert(n->type == MCSHENG_NFA_8);
    const struct mcsheng *m = getImplNfa(n);
    s64a sp;

    u32 s = *(u8 *)q->state;

    if (q->report_current) {
        assert(s);
        assert(s >= m->accept_limit_8);

        int rv;
        if (single) {
            DEBUG_PRINTF("reporting %u\n", m->arb_report);
            rv = cb(0, q_cur_offset(q), m->arb_report, context);
        } else {
            u32 cached_accept_id = 0;
            u32 cached_accept_state = 0;

            rv = doComplexReport(cb, context, m, s, q_cur_offset(q), 0,
                                 &cached_accept_state, &cached_accept_id);
        }

        q->report_current = 0;

        if (rv == MO_HALT_MATCHING) {
            return MO_DEAD;
        }
    }

    sp = q_cur_loc(q);
    q->cur++;

    const u8 *cur_buf = sp < 0 ? hend : buffer;

    if (mode != NO_MATCHES && q->items[q->cur - 1].location > end) {
        DEBUG_PRINTF("this is as far as we go\n");
        q->cur--;
        q->items[q->cur].type = MQE_START;
        q->items[q->cur].location = end;
        *(u8 *)q->state = s;
        return MO_ALIVE;
    }

    while (1) {
        DEBUG_PRINTF("%s @ %llu\n", q->items[q->cur].type == MQE_TOP ? "TOP" :
                     q->items[q->cur].type == MQE_END ? "END" : "???",
                     q->items[q->cur].location + offset);
        assert(q->cur < q->end);
        s64a ep = q->items[q->cur].location;
        if (mode != NO_MATCHES) {
            ep = MIN(ep, end);
        }

        assert(ep >= sp);

        s64a local_ep = ep;
        if (sp < 0) {
            local_ep = MIN(0, ep);
        }

        const u8 *final_look;
        char rv = mcshengExec8_i_ni(m, &s, cur_buf + sp, local_ep - sp,
                                    offset + sp, cb, context, single,
                                    &final_look, mode);
        if (rv == MO_HALT_MATCHING) {
            *(u8 *)q->state = 0;
            return MO_DEAD;
        }
        if (mode == STOP_AT_MATCH && rv == MO_MATCHES_PENDING) {
            DEBUG_PRINTF("this is as far as we go\n");
            DEBUG_PRINTF("state %u final_look %zd\n", s, final_look - cur_buf);

            assert(q->cur);
            assert(final_look != cur_buf + local_ep);

            q->cur--;
            q->items[q->cur].type = MQE_START;
            q->items[q->cur].location = final_look - cur_buf + 1; /* due to
                                                                   * early -1 */
            *(u8 *)q->state = s;
            return MO_MATCHES_PENDING;
        }

        assert(rv == MO_ALIVE);
        assert(q->cur);
        if (mode != NO_MATCHES && q->items[q->cur].location > end) {
            DEBUG_PRINTF("this is as far as we go\n");
            assert(q->cur);
            q->cur--;
            q->items[q->cur].type = MQE_START;
            q->items[q->cur].location = end;
            *(u8 *)q->state = s;
            return MO_ALIVE;
        }

        sp = local_ep;

        if (sp == 0) {
            cur_buf = buffer;
        }

        if (sp != ep) {
            continue;
        }

        switch (q->items[q->cur].type) {
        case MQE_TOP:
            assert(sp + offset || !s);
            if (sp + offset == 0) {
                s = (u8)m->start_anchored;
                break;
            }
            s = mcshengEnableStarts(m, s);
            break;
        case MQE_END:
            *(u8 *)q->state = s;
            q->cur++;
            return s ? MO_ALIVE : MO_DEAD;
        default:
            assert(!"invalid queue event");
        }

        q->cur++;
    }
}

char nfaExecMcSheng8_Q(const struct NFA *n, struct mq *q, s64a end) {
    u64a offset = q->offset;
    const u8 *buffer = q->buffer;
    NfaCallback cb = q->cb;
    void *context = q->context;
    assert(n->type == MCSHENG_NFA_8);
    const struct mcsheng *m = getImplNfa(n);
    const u8 *hend = q->history + q->hlength;

    return nfaExecMcSheng8_Q2i(n, offset, buffer, hend, cb, context, q,
                               m->flags & MCSHENG_FLAG_SINGLE, end,
                               CALLBACK_OUTPUT);
}

char nfaExecMcSheng16_Q(const struct NFA *n, struct mq *q, s64a end) {
    u64a offset = q->offset;
    const u8 *buffer = q->buffer;
    NfaCallback cb = q->cb;
    void *context = q->context;
    assert(n->type == MCSHENG_NFA_16);
    const struct mcsheng *m = getImplNfa(n);
    const u8 *hend = q->history + q->hlength;

    return nfaExecMcSheng16_Q2i(n, offset, buffer, hend, cb, context, q,
                                m->flags & MCSHENG_FLAG_SINGLE, end,
                                CALLBACK_OUTPUT);
}

char nfaExecMcSheng8_reportCurrent(const struct NFA *n, struct mq *q) {
    const struct mcsheng *m = getImplNfa(n);
    NfaCallback cb = q->cb;
    void *ctxt = q->context;
    u32 s = *(u8 *)q->state;
    u8 single = m->flags & MCSHENG_FLAG_SINGLE;
    u64a offset = q_cur_offset(q);
    assert(q_cur_type(q) == MQE_START);
    assert(s);

    if (s >= m->accept_limit_8) {
        if (single) {
            DEBUG_PRINTF("reporting %u\n", m->arb_report);
            cb(0, offset, m->arb_report, ctxt);
        } else {
            u32 cached_accept_id = 0;
            u32 cached_accept_state = 0;

            doComplexReport(cb, ctxt, m, s, offset, 0, &cached_accept_state,
                            &cached_accept_id);
        }
    }

    return 0;
}

char nfaExecMcSheng16_reportCurrent(const struct NFA *n, struct mq *q) {
    const struct mcsheng *m = getImplNfa(n);
    NfaCallback cb = q->cb;
    void *ctxt = q->context;
    u32 s = *(u16 *)q->state;
    const struct mstate_aux *aux = get_aux(m, s);
    u8 single = m->flags & MCSHENG_FLAG_SINGLE;
    u64a offset = q_cur_offset(q);
    assert(q_cur_type(q) == MQE_START);
    DEBUG_PRINTF("state %u\n", s);
    assert(s);

    if (aux->accept) {
        if (single) {
            DEBUG_PRINTF("reporting %u\n", m->arb_report);
            cb(0, offset, m->arb_report, ctxt);
        } else {
            u32 cached_accept_id = 0;
            u32 cached_accept_state = 0;

            doComplexReport(cb, ctxt, m, s, offset, 0, &cached_accept_state,
                            &cached_accept_id);
        }
    }

    return 0;
}

static
char mcshengHasAccept(const struct mcsheng *m, const struct mstate_aux *aux,
                      ReportID report) {
    assert(m && aux);

    if (!aux->accept) {
        return 0;
    }

    const struct report_list *rl = (const struct report_list *)
            ((const char *)m + aux->accept - sizeof(struct NFA));
    assert(ISALIGNED_N(rl, 4));

    DEBUG_PRINTF("report list has %u entries\n", rl->count);

    for (u32 i = 0; i < rl->count; i++) {
        if (rl->report[i] == report) {
            return 1;
        }
    }

    return 0;
}

char nfaExecMcSheng8_inAccept(const struct NFA *n, ReportID report,
                              struct mq *q) {
    assert(n && q);

    const struct mcsheng *m = getImplNfa(n);
    u8 s = *(u8 *)q->state;
    DEBUG_PRINTF("checking accepts for %hhu\n", s);

    return mcshengHasAccept(m, get_aux(m, s), report);
}

char nfaExecMcSheng8_inAnyAccept(const struct NFA *n, struct mq *q) {
    assert(n && q);

    const struct mcsheng *m = getImplNfa(n);
    u8 s = *(u8 *)q->state;
    DEBUG_PRINTF("checking accepts for %hhu\n", s);

    return !!get_aux(m, s)->accept;
}

char nfaExecMcSheng16_inAccept(const struct NFA *n, ReportID report,
                               struct mq *q) {
    assert(n && q);

    const struct mcsheng *m = getImplNfa(n);
    u16 s = *(u16 *)q->state;
    DEBUG_PRINTF("checking accepts for %hu\n", s);

    return mcshengHasAccept(m, get_aux(m, s), report);
}

char nfaExecMcSheng16_inAnyAccept(const struct NFA *n, struct mq *q) {
    assert(n && q);

    const struct mcsheng *m = getImplNfa(n);
    u16 s = *(u16 *)q->state;
    DEBUG_PRINTF("checking accepts for %hu\n", s);

    return !!get_aux(m, s)->accept;
}

char nfaExecMcSheng8_Q2(const struct NFA *n, struct mq *q, s64a end) {
    u64a offset = q->offset;
    const u8 *buffer = q->buffer;
    NfaCallback cb = q->cb;
    void *context = q->context;
    assert(n->type == MCSHENG_NFA_8);
    const struct mcsheng *m = getImplNfa(n);
    const u8 *hend = q->history + q->hlength;

    return nfaExecMcSheng8_Q2i(n, offset, buffer, hend, cb, context, q,
                               m->flags & MCSHENG_FLAG_SINGLE, end,
                               STOP_AT_MATCH);
}

char nfaExecMcSheng16_Q2(const struct NFA *n, struct mq *q, s64a end) {
    u64a offset = q->offset;
    const u8 *buffer = q->buffer;
    NfaCallback cb = q->cb;
    void *context = q->context;
    assert(n->type == MCSHENG_NFA_16);
    const struct mcsheng *m = getImplNfa(n);
    const u8 *hend = q->history + q->hlength;

    return nfaExecMcSheng16_Q2i(n, offset, buffer, hend, cb, context, q,
                                m->flags & MCSHENG_FLAG_SINGLE, end,
                                STOP_AT_MATCH);
}

char nfaExecMcSheng8_QR(const struct NFA *n, struct mq *q, ReportID report) {
    u64a offset = q->offset;
    const u8 *buffer = q->buffer;
    NfaCallback cb = q->cb;
    void *context = q->context;
    assert(n->type == MCSHENG_NFA_8);
    const struct mcsheng *m = getImplNfa(n);
    const u8 *hend = q->history + q->hlength;

    char rv = nfaExecMcSheng8_Q2i(n, offset, buffer, hend, cb, context, q,
                                  m->flags & MCSHENG_FLAG_SINGLE, 0 /* end */,
                                  NO_MATCHES);
    if (rv && nfaExecMcSheng8_inAccept(n, report, q)) {
        return MO_MATCHES_PENDING;
    } else {
        return rv;
    }
}

char nfaExecMcSheng16_QR(const struct NFA *n, struct mq *q, ReportID report) {
    u64a offset = q->offset;
    const u8 *buffer = q->buffer;
    NfaCallback cb = q->cb;
    void *context = q->context;
    assert(n->type == MCSHENG_NFA_16);
    const struct mcsheng *m = getImplNfa(n);
    const u8 *hend = q->history + q->hlength;

    char rv = nfaExecMcSheng16_Q2i(n, offset, buffer, hend, cb, context, q,
                                   m->flags & MCSHENG_FLAG_SINGLE, 0 /* end */,
                                   NO_MATCHES);

    if (rv && nfaExecMcSheng16_inAccept(n, report, q)) {
        return MO_MATCHES_PENDING;
    } else {
        return rv;
    }
}

char nfaExecMcSheng8_initCompressedState(const struct NFA *nfa, u64a offset,
                                         void *state, UNUSED u8 key) {
    const struct mcsheng *m = getImplNfa(nfa);
    u8 s = offset ? m->start_floating : m->start_anchored;
    if (s) {
        *(u8 *)state = s;
        return 1;
    }
    return 0;
}

char nfaExecMcSheng16_initCompressedState(const struct NFA *nfa, u64a offset,
                                          void *state, UNUSED u8 key) {
    const struct mcsheng *m = getImplNfa(nfa);
    u16 s = offset ? m->start_floating : m->start_anchored;
    if (s) {
        unaligned_store_u16(state, s);
        return 1;
    }
    return 0;
}

char nfaExecMcSheng8_testEOD(const struct NFA *nfa, const char *state,
                             UNUSED const char *streamState, u64a offset,
                             NfaCallback callback, void *context) {
    return mcshengCheckEOD(nfa, *(const u8 *)state, offset, callback,
                           context);
}

char nfaExecMcSheng16_testEOD(const struct NFA *nfa, const char *state,
                              UNUSED const char *streamState, u64a offset,
                              NfaCallback callback, void *context) {
    assert(ISALIGNED_N(state, 2));
    return mcshengCheckEOD(nfa, *(const u16 *)state, offset, callback,
                           context);
}

char nfaExecMcSheng8_queueInitState(UNUSED const struct NFA *nfa, struct mq *q) {
    assert(nfa->scratchStateSize == 1);
    *(u8 *)q->state = 0;
    return 0;
}

char nfaExecMcSheng16_queueInitState(UNUSED const struct NFA *nfa, struct mq *q) {
    assert(nfa->scratchStateSize == 2);
    assert(ISALIGNED_N(q->state, 2));
    *(u16 *)q->state = 0;
    return 0;
}

char nfaExecMcSheng8_queueCompressState(UNUSED const struct NFA *nfa,
                                        const struct mq *q, UNUSED s64a loc) {
    void *dest = q->streamState;
    const void *src = q->state;
    assert(nfa->scratchStateSize == 1);
    assert(nfa->streamStateSize == 1);
    *(u8 *)dest = *(const u8 *)src;
    return 0;
}

char nfaExecMcSheng8_expandState(UNUSED const struct NFA *nfa, void *dest,
                                 const void *src, UNUSED u64a offset,
                                 UNUSED u8 key) {
    assert(nfa->scratchStateSize == 1);
    assert(nfa->streamStateSize == 1);
    *(u8 *)dest = *(const u8 *)src;
    return 0;
}

char nfaExecMcSheng16_queueCompressState(UNUSED const struct NFA *nfa,
                                         const struct mq *q,
                                         UNUSED s64a loc) {
    void *dest = q->streamState;
    const void *src = q->state;
    assert(nfa->scratchStateSize == 2);
    assert(nfa->streamStateSize == 2);
    assert(ISALIGNED_N(src, 2));
    unaligned_store_u16(dest, *(const u16 *)(src));
    return 0;
}

char nfaExecMcSheng16_expandState(UNUSED const struct NFA *nfa, void *dest,
                                  const void *src, UNUSED u64a offset,
                                  UNUSED u8 key) {
    assert(nfa->scratchStateSize == 2);
    assert(nfa->streamStateSize == 2);
    assert(ISALIGNED_N(dest, 2));
    *(u16 *)dest = unaligned_load_u16(src);
    return 0;
}

#if defined(HAVE_AVX512VBMI)
static really_inline
const struct mstate_aux *get_aux64(const struct mcsheng64 *m, u32 s) {
    const char *nfa = (const char *)m - sizeof(struct NFA);
    const struct mstate_aux *aux
        = s + (const struct mstate_aux *)(nfa + m->aux_offset);

    assert(ISALIGNED(aux));
    return aux;
}

static really_inline
u32 mcshengEnableStarts64(const struct mcsheng64 *m, u32 s) {
    const struct mstate_aux *aux = get_aux64(m, s);

    DEBUG_PRINTF("enabling starts %u->%hu\n", s, aux->top);
    return aux->top;
}

static really_inline
char doComplexReport64(NfaCallback cb, void *ctxt, const struct mcsheng64 *m,
                       u32 s, u64a loc, char eod, u32 *cached_accept_state,
                       u32 *cached_accept_id) {
    DEBUG_PRINTF("reporting state = %u, loc=%llu, eod %hhu\n",
                 s & STATE_MASK, loc, eod);

    if (!eod && s == *cached_accept_state) {
        if (cb(0, loc, *cached_accept_id, ctxt) == MO_HALT_MATCHING) {
            return MO_HALT_MATCHING; /* termination requested */
        }

        return MO_CONTINUE_MATCHING; /* continue execution */
    }

    const struct mstate_aux *aux = get_aux64(m, s);
    size_t offset = eod ? aux->accept_eod : aux->accept;

    assert(offset);
    const struct report_list *rl
        = (const void *)((const char *)m + offset - sizeof(struct NFA));
    assert(ISALIGNED(rl));

    DEBUG_PRINTF("report list size %u\n", rl->count);
    u32 count = rl->count;

    if (!eod && count == 1) {
        *cached_accept_state = s;
        *cached_accept_id = rl->report[0];

        DEBUG_PRINTF("reporting %u\n", rl->report[0]);
        if (cb(0, loc, rl->report[0], ctxt) == MO_HALT_MATCHING) {
            return MO_HALT_MATCHING; /* termination requested */
        }

        return MO_CONTINUE_MATCHING; /* continue execution */
    }

    for (u32 i = 0; i < count; i++) {
        DEBUG_PRINTF("reporting %u\n", rl->report[i]);
        if (cb(0, loc, rl->report[i], ctxt) == MO_HALT_MATCHING) {
            return MO_HALT_MATCHING; /* termination requested */
        }
    }

    return MO_CONTINUE_MATCHING; /* continue execution */
}

static really_inline
u32 doSheng64(const struct mcsheng64 *m, const u8 **c_inout, const u8 *soft_c_end,
              const u8 *hard_c_end, u32 s_in, char do_accel) {
    assert(s_in < m->sheng_end);
    assert(s_in); /* should not already be dead */
    assert(soft_c_end <= hard_c_end);
    DEBUG_PRINTF("s_in = %u (adjusted %u)\n", s_in, s_in - 1);
    m512 s = set64x8(s_in - 1);
    const u8 *c = *c_inout;
    const u8 *c_end = hard_c_end - SHENG_CHUNK + 1;
    if (!do_accel) {
        c_end = MIN(soft_c_end, hard_c_end - SHENG_CHUNK + 1);
    }

    const m512 *masks = m->sheng_succ_masks;
    u8 sheng_limit = m->sheng_end - 1; /* - 1: no dead state */
    u8 sheng_stop_limit = do_accel ? m->sheng_accel_limit : sheng_limit;

    /* When we use movd to get a u32 containing our state, it will have 4 lanes
     * all duplicating the state. We can create versions of our limits with 4
     * copies to directly compare against, this prevents us generating code to
     * extract a single copy of the state from the u32 for checking. */
    u32 sheng_stop_limit_x4 = sheng_stop_limit * 0x01010101;

#if defined(HAVE_BMI2) && defined(ARCH_64_BIT)
    u32 sheng_limit_x4 = sheng_limit * 0x01010101;
    m512 simd_stop_limit = set16x32(sheng_stop_limit_x4);
    m512 accel_delta = set64x8(sheng_limit - sheng_stop_limit);
    DEBUG_PRINTF("end %hhu, accel %hu --> limit %hhu\n", sheng_limit,
                 m->sheng_accel_limit, sheng_stop_limit);
#endif

#define SHENG64_SINGLE_ITER do {                                             \
        m512 succ_mask = masks[*(c++)];                                      \
        s = vpermb512(s, succ_mask);                                         \
        u32 s_gpr_x4 = movd512(s); /* convert to u8 */                       \
        DEBUG_PRINTF("c %hhu (%c) --> s %u\n", c[-1], c[-1], s_gpr_x4);      \
        if (s_gpr_x4 >= sheng_stop_limit_x4) {                               \
            s_gpr = s_gpr_x4;                                                \
            goto exit;                                                       \
        }                                                                    \
    } while (0)

    u8 s_gpr;
    while (c < c_end) {
#if defined(HAVE_BMI2) && defined(ARCH_64_BIT)
        /* This version uses pext for efficiently bitbashing out scaled
         * versions of the bytes to process from a u64a */

        u64a data_bytes = unaligned_load_u64a(c);
        u64a cc0 = pdep64(data_bytes, 0x3fc0); /* extract scaled low byte */
        data_bytes &= ~0xffULL; /* clear low bits for scale space */

        m512 succ_mask0 = load512((const char *)masks + cc0);
        s = vpermb512(s, succ_mask0);
        m512 s_max = s;
        m512 s_max0 = s_max;
        DEBUG_PRINTF("c %02llx --> s %u\n", cc0 >> 6, movd512(s));

#define SHENG64_SINGLE_UNROLL_ITER(iter)                                \
        assert(iter);                                                   \
        u64a cc##iter = pext64(data_bytes, mcsheng64_pext_mask[iter]);  \
        assert(cc##iter == (u64a)c[iter] << 6);                         \
        m512 succ_mask##iter = load512((const char *)masks + cc##iter); \
        s = vpermb512(s, succ_mask##iter);                              \
        if (do_accel && iter == 7) {                                    \
            /* in the final iteration we also have to check against accel */ \
            m512 s_temp = sadd_u8_m512(s, accel_delta);                 \
            s_max = max_u8_m512(s_max, s_temp);                         \
        } else {                                                        \
            s_max = max_u8_m512(s_max, s);                              \
        }                                                               \
        m512 s_max##iter = s_max;                                       \
        DEBUG_PRINTF("c %02llx --> s %u max %u\n", cc##iter >> 6,       \
                     movd512(s), movd512(s_max));

        SHENG64_SINGLE_UNROLL_ITER(1);
        SHENG64_SINGLE_UNROLL_ITER(2);
        SHENG64_SINGLE_UNROLL_ITER(3);
        SHENG64_SINGLE_UNROLL_ITER(4);
        SHENG64_SINGLE_UNROLL_ITER(5);
        SHENG64_SINGLE_UNROLL_ITER(6);
        SHENG64_SINGLE_UNROLL_ITER(7);

        if (movd512(s_max7) >= sheng_limit_x4) {
            DEBUG_PRINTF("exit found\n");

            /* Explicitly check the last byte as it is more likely as it also
             * checks for acceleration. */
            if (movd512(s_max6) < sheng_limit_x4) {
                c += SHENG_CHUNK;
                s_gpr = movq512(s);
                assert(s_gpr >= sheng_stop_limit);
                goto exit;
            }

            /* use shift-xor to create a register containing all of the max
             * values */
            m512 blended = rshift64_m512(s_max0, 56);
            blended = xor512(blended, rshift64_m512(s_max1, 48));
            blended = xor512(blended, rshift64_m512(s_max2, 40));
            blended = xor512(blended, rshift64_m512(s_max3, 32));
            blended = xor512(blended, rshift64_m512(s_max4, 24));
            blended = xor512(blended, rshift64_m512(s_max5, 16));
            blended = xor512(blended, rshift64_m512(s_max6, 8));
            blended = xor512(blended, s);
            blended = xor512(blended, rshift64_m512(blended, 8));
            DEBUG_PRINTF("blended %016llx\n", movq512(blended));

            m512 final = min_u8_m512(blended, simd_stop_limit);
            m512 cmp = sub_u8_m512(final, simd_stop_limit);
            m128 tmp = cast512to128(cmp);
            u64a stops = ~movemask128(tmp);
            assert(stops);
            u32 earliest = ctz32(stops);
            DEBUG_PRINTF("stops %02llx, earliest %u\n", stops, earliest);
            assert(earliest < 8);
            c += earliest + 1;
            s_gpr = movq512(blended) >> (earliest * 8);
            assert(s_gpr >= sheng_stop_limit);
            goto exit;
        } else {
            c += SHENG_CHUNK;
        }
#else
        SHENG64_SINGLE_ITER;
        SHENG64_SINGLE_ITER;
        SHENG64_SINGLE_ITER;
        SHENG64_SINGLE_ITER;

        SHENG64_SINGLE_ITER;
        SHENG64_SINGLE_ITER;
        SHENG64_SINGLE_ITER;
        SHENG64_SINGLE_ITER;
#endif
    }

    assert(c_end - c < SHENG_CHUNK);
    if (c < soft_c_end) {
        assert(soft_c_end - c < SHENG_CHUNK);
        switch (soft_c_end - c) {
        case 7:
            SHENG64_SINGLE_ITER; // fallthrough
        case 6:
            SHENG64_SINGLE_ITER; // fallthrough
        case 5:
            SHENG64_SINGLE_ITER; // fallthrough
        case 4:
            SHENG64_SINGLE_ITER; // fallthrough
        case 3:
            SHENG64_SINGLE_ITER; // fallthrough
        case 2:
            SHENG64_SINGLE_ITER; // fallthrough
        case 1:
            SHENG64_SINGLE_ITER; // fallthrough
        }
    }

    assert(c >= soft_c_end);

    s_gpr = movq512(s);
exit:
    assert(c <= hard_c_end);
    DEBUG_PRINTF("%zu from end; s %hhu\n", c_end - c, s_gpr);
    assert(c >= soft_c_end || s_gpr >= sheng_stop_limit);
    /* undo state adjustment to match mcclellan view */
    if (s_gpr == sheng_limit) {
        s_gpr = 0;
    } else if (s_gpr < sheng_limit) {
        s_gpr++;
    }

    *c_inout = c;
    return s_gpr;
}

static really_inline
const char *findShermanState64(UNUSED const struct mcsheng64 *m,
                               const char *sherman_base_offset,
                               u32 sherman_base, u32 s) {
    const char *rv
        = sherman_base_offset + SHERMAN_FIXED_SIZE * (s - sherman_base);
    assert(rv < (const char *)m + m->length - sizeof(struct NFA));
    UNUSED u8 type = *(const u8 *)(rv + SHERMAN_TYPE_OFFSET);
    assert(type == SHERMAN_STATE);
    return rv;
}

static really_inline
const u8 *run_mcsheng_accel64(const struct mcsheng64 *m,
                              const struct mstate_aux *aux, u32 s,
                              const u8 **min_accel_offset,
                              const u8 *c, const u8 *c_end) {
    DEBUG_PRINTF("skipping\n");
    u32 accel_offset = aux[s].accel_offset;

    assert(aux[s].accel_offset);
    assert(accel_offset >= m->aux_offset);
    assert(!m->sherman_offset || accel_offset < m->sherman_offset);

    const union AccelAux *aaux = (const void *)((const char *)m + accel_offset);
    const u8 *c2 = run_accel(aaux, c, c_end);

    if (c2 < *min_accel_offset + BAD_ACCEL_DIST) {
        *min_accel_offset = c2 + BIG_ACCEL_PENALTY;
    } else {
        *min_accel_offset = c2 + SMALL_ACCEL_PENALTY;
    }

    if (*min_accel_offset >= c_end - ACCEL_MIN_LEN) {
        *min_accel_offset = c_end;
    }

    DEBUG_PRINTF("advanced %zd, next accel chance in %zd/%zd\n",
                 c2 - c, *min_accel_offset - c2, c_end - c2);

    return c2;
}

static really_inline
u32 doNormal64_16(const struct mcsheng64 *m, const u8 **c_inout, const u8 *end,
                  u32 s, char do_accel, enum MatchMode mode) {
    const u8 *c = *c_inout;
    const u16 *succ_table
        = (const u16 *)((const char *)m + sizeof(struct mcsheng64));
    assert(ISALIGNED_N(succ_table, 2));
    u32 sheng_end = m->sheng_end;
    u32 sherman_base = m->sherman_limit;
    const char *sherman_base_offset
        = (const char *)m - sizeof(struct NFA) + m->sherman_offset;
    u32 as = m->alphaShift;

    /* Adjust start of succ table so we can index into using state id (rather
     * than adjust to normal id). As we will not be processing states with low
     * state ids, we will not be accessing data before the succ table. Note: due
     * to the size of the sheng tables, the succ_table pointer will still be
     * inside the engine.*/
    succ_table -= sheng_end << as;
    s &= STATE_MASK;
    while (c < end && s >= sheng_end) {
        u8 cprime = m->remap[*c];
        DEBUG_PRINTF("c: %02hhx '%c' cp:%02hhx (s=%u)\n", *c,
                     ourisprint(*c) ? *c : '?', cprime, s);
        if (s < sherman_base) {
            DEBUG_PRINTF("doing normal\n");
            assert(s < m->state_count);
            s = succ_table[(s << as) + cprime];
        } else {
            const char *sherman_state
                = findShermanState64(m, sherman_base_offset, sherman_base, s);
            DEBUG_PRINTF("doing sherman (%u)\n", s);
            s = doSherman16(sherman_state, cprime, succ_table, as);
        }

        DEBUG_PRINTF("s: %u (%u)\n", s, s & STATE_MASK);
        c++;

        if (do_accel && (s & ACCEL_FLAG)) {
            break;
        }
        if (mode != NO_MATCHES && (s & ACCEPT_FLAG)) {
            break;
        }

        s &= STATE_MASK;
    }

    *c_inout = c;
    return s;
}

static really_inline
char mcsheng64Exec16_i(const struct mcsheng64 *m, u32 *state, const u8 *buf,
                       size_t len, u64a offAdj, NfaCallback cb, void *ctxt,
                       char single, const u8 **c_final, enum MatchMode mode) {
    assert(ISALIGNED_N(state, 2));
    if (!len) {
        if (mode == STOP_AT_MATCH) {
            *c_final = buf;
        }
        return MO_ALIVE;
    }

    u32 s = *state;
    const u8 *c = buf;
    const u8 *c_end = buf + len;
    const u8 sheng_end = m->sheng_end;
    const struct mstate_aux *aux
        = (const struct mstate_aux *)((const char *)m + m->aux_offset
                                      - sizeof(struct NFA));

    s &= STATE_MASK;

    u32 cached_accept_id = 0;
    u32 cached_accept_state = 0;

    DEBUG_PRINTF("s: %u, len %zu\n", s, len);

    const u8 *min_accel_offset = c;
    if (!m->has_accel || len < ACCEL_MIN_LEN) {
        min_accel_offset = c_end;
        goto without_accel;
    }

    goto with_accel;

without_accel:
    do {
        assert(c < min_accel_offset);
        int do_accept;
        if (!s) {
            goto exit;
        } else if (s < sheng_end) {
            s = doSheng64(m, &c, min_accel_offset, c_end, s, 0);
            do_accept = mode != NO_MATCHES && get_aux64(m, s)->accept;
        } else {
            s = doNormal64_16(m, &c, min_accel_offset, s, 0, mode);

            do_accept = mode != NO_MATCHES && (s & ACCEPT_FLAG);
        }

        if (do_accept) {
            if (mode == STOP_AT_MATCH) {
                *state = s & STATE_MASK;
                *c_final = c - 1;
                return MO_MATCHES_PENDING;
            }

            u64a loc = (c - 1) - buf + offAdj + 1;

            if (single) {
                DEBUG_PRINTF("reporting %u\n", m->arb_report);
                if (cb(0, loc, m->arb_report, ctxt) == MO_HALT_MATCHING) {
                    return MO_DEAD; /* termination requested */
                }
            } else if (doComplexReport64(cb, ctxt, m, s & STATE_MASK, loc, 0,
                                         &cached_accept_state,
                                         &cached_accept_id)
                       == MO_HALT_MATCHING) {
                return MO_DEAD;
            }
        }

        assert(c <= c_end); /* sheng is fuzzy for min_accel_offset */
    } while (c < min_accel_offset);

    if (c == c_end) {
        goto exit;
    }

with_accel:
    do {
        assert(c < c_end);
        int do_accept;

        if (!s) {
            goto exit;
        } else if (s < sheng_end) {
            if (s > m->sheng_accel_limit) {
                c = run_mcsheng_accel64(m, aux, s, &min_accel_offset, c, c_end);
                if (c == c_end) {
                    goto exit;
                } else {
                    goto without_accel;
                }
            }
            s = doSheng64(m, &c, c_end, c_end, s, 1);
            do_accept = mode != NO_MATCHES && get_aux64(m, s)->accept;
        } else {
            if (s & ACCEL_FLAG) {
                DEBUG_PRINTF("skipping\n");
                s &= STATE_MASK;
                c = run_mcsheng_accel64(m, aux, s, &min_accel_offset, c, c_end);
                if (c == c_end) {
                    goto exit;
                } else {
                    goto without_accel;
                }
            }

            s = doNormal64_16(m, &c, c_end, s, 1, mode);
            do_accept = mode != NO_MATCHES && (s & ACCEPT_FLAG);
        }

        if (do_accept) {
            if (mode == STOP_AT_MATCH) {
                *state = s & STATE_MASK;
                *c_final = c - 1;
                return MO_MATCHES_PENDING;
            }

            u64a loc = (c - 1) - buf + offAdj + 1;

            if (single) {
                DEBUG_PRINTF("reporting %u\n", m->arb_report);
                if (cb(0, loc, m->arb_report, ctxt) == MO_HALT_MATCHING) {
                    return MO_DEAD; /* termination requested */
                }
            } else if (doComplexReport64(cb, ctxt, m, s & STATE_MASK, loc, 0,
                                         &cached_accept_state,
                                         &cached_accept_id)
                       == MO_HALT_MATCHING) {
                return MO_DEAD;
            }
        }

        assert(c <= c_end);
    } while (c < c_end);

exit:
    s &= STATE_MASK;

    if (mode == STOP_AT_MATCH) {
        *c_final = c_end;
    }
    *state = s;

    return MO_ALIVE;
}

static never_inline
char mcsheng64Exec16_i_cb(const struct mcsheng64 *m, u32 *state, const u8 *buf,
                          size_t len, u64a offAdj, NfaCallback cb, void *ctxt,
                          char single, const u8 **final_point) {
    return mcsheng64Exec16_i(m, state, buf, len, offAdj, cb, ctxt, single,
                             final_point, CALLBACK_OUTPUT);
}

static never_inline
char mcsheng64Exec16_i_sam(const struct mcsheng64 *m, u32 *state, const u8 *buf,
                           size_t len, u64a offAdj, NfaCallback cb, void *ctxt,
                           char single, const u8 **final_point) {
    return mcsheng64Exec16_i(m, state, buf, len, offAdj, cb, ctxt, single,
                             final_point, STOP_AT_MATCH);
}

static never_inline
char mcsheng64Exec16_i_nm(const struct mcsheng64 *m, u32 *state, const u8 *buf,
                          size_t len, u64a offAdj, NfaCallback cb, void *ctxt,
                          char single, const u8 **final_point) {
    return mcsheng64Exec16_i(m, state, buf, len, offAdj, cb, ctxt, single,
                             final_point, NO_MATCHES);
}

static really_inline
char mcsheng64Exec16_i_ni(const struct mcsheng64 *m, u32 *state, const u8 *buf,
                          size_t len, u64a offAdj, NfaCallback cb, void *ctxt,
                          char single, const u8 **final_point,
                          enum MatchMode mode) {
    if (mode == CALLBACK_OUTPUT) {
        return mcsheng64Exec16_i_cb(m, state, buf, len, offAdj, cb, ctxt,
                                    single, final_point);
    } else if (mode == STOP_AT_MATCH) {
        return mcsheng64Exec16_i_sam(m, state, buf, len, offAdj, cb, ctxt,
                                     single, final_point);
    } else {
        assert (mode == NO_MATCHES);
        return mcsheng64Exec16_i_nm(m, state, buf, len, offAdj, cb, ctxt,
                                    single, final_point);
    }
}

static really_inline
u32 doNormal64_8(const struct mcsheng64 *m, const u8 **c_inout, const u8 *end, u32 s,
                 char do_accel, enum MatchMode mode) {
    const u8 *c = *c_inout;
    u32 sheng_end = m->sheng_end;
    u32 accel_limit = m->accel_limit_8;
    u32 accept_limit = m->accept_limit_8;

    const u32 as = m->alphaShift;
    const u8 *succ_table = (const u8 *)((const char *)m
                                        + sizeof(struct mcsheng64));
    /* Adjust start of succ table so we can index into using state id (rather
     * than adjust to normal id). As we will not be processing states with low
     * state ids, we will not be accessing data before the succ table. Note: due
     * to the size of the sheng tables, the succ_table pointer will still be
     * inside the engine.*/
    succ_table -= sheng_end << as;

    assert(s >= sheng_end);
    while (c < end && s >= sheng_end) {
        u8 cprime = m->remap[*c];
        DEBUG_PRINTF("c: %02hhx '%c' cp:%02hhx\n", *c,
                     ourisprint(*c) ? *c : '?', cprime);
        s = succ_table[(s << as) + cprime];

        DEBUG_PRINTF("s: %u\n", s);
        c++;
        if (do_accel) {
            if (s >= accel_limit) {
                break;
            }
        } else {
            if (mode != NO_MATCHES && s >= accept_limit) {
                break;
            }
        }
    }
    *c_inout = c;
    return s;
}

static really_inline
char mcsheng64Exec8_i(const struct mcsheng64 *m, u32 *state, const u8 *buf,
                      size_t len, u64a offAdj, NfaCallback cb, void *ctxt,
                      char single, const u8 **c_final, enum MatchMode mode) {
    if (!len) {
        *c_final = buf;
        return MO_ALIVE;
    }
    u32 s = *state;
    const u8 *c = buf;
    const u8 *c_end = buf + len;
    const u8 sheng_end = m->sheng_end;

    const struct mstate_aux *aux
        = (const struct mstate_aux *)((const char *)m + m->aux_offset
                                      - sizeof(struct NFA));
    u32 accept_limit = m->accept_limit_8;

    u32 cached_accept_id = 0;
    u32 cached_accept_state = 0;

    DEBUG_PRINTF("accel %hu, accept %u\n", m->accel_limit_8, accept_limit);

    DEBUG_PRINTF("s: %u, len %zu\n", s, len);

    const u8 *min_accel_offset = c;
    if (!m->has_accel || len < ACCEL_MIN_LEN) {
        min_accel_offset = c_end;
        goto without_accel;
    }

    goto with_accel;

without_accel:
    do {
        assert(c < min_accel_offset);
        if (!s) {
            goto exit;
        } else if (s < sheng_end) {
            s = doSheng64(m, &c, min_accel_offset, c_end, s, 0);
        } else {
            s = doNormal64_8(m, &c, min_accel_offset, s, 0, mode);
            assert(c <= min_accel_offset);
        }

        if (mode != NO_MATCHES && s >= accept_limit) {
            if (mode == STOP_AT_MATCH) {
                DEBUG_PRINTF("match - pausing\n");
                *state = s;
                *c_final = c - 1;
                return MO_MATCHES_PENDING;
            }

            u64a loc = (c - 1) - buf + offAdj + 1;
            if (single) {
                DEBUG_PRINTF("reporting %u\n", m->arb_report);
                if (cb(0, loc, m->arb_report, ctxt) == MO_HALT_MATCHING) {
                    return MO_DEAD;
                }
            } else if (doComplexReport64(cb, ctxt, m, s, loc, 0,
                                         &cached_accept_state,
                                         &cached_accept_id)
                       == MO_HALT_MATCHING) {
                return MO_DEAD;
            }
        }

        assert(c <= c_end); /* sheng is fuzzy for min_accel_offset */
    } while (c < min_accel_offset);

    if (c == c_end) {
        goto exit;
    }

with_accel:
    do {
        u32 accel_limit = m->accel_limit_8;

        assert(c < c_end);
        if (!s) {
            goto exit;
        } else if (s < sheng_end) {
            if (s > m->sheng_accel_limit) {
                c = run_mcsheng_accel64(m, aux, s, &min_accel_offset, c, c_end);
                if (c == c_end) {
                    goto exit;
                } else {
                    goto without_accel;
                }
            }
            s = doSheng64(m, &c, c_end, c_end, s, 1);
        } else {
            if (s >= accel_limit && aux[s].accel_offset) {
                c = run_mcsheng_accel64(m, aux, s, &min_accel_offset, c, c_end);
                if (c == c_end) {
                    goto exit;
                } else {
                    goto without_accel;
                }
            }
            s = doNormal64_8(m, &c, c_end, s, 1, mode);
        }

        if (mode != NO_MATCHES && s >= accept_limit) {
            if (mode == STOP_AT_MATCH) {
                DEBUG_PRINTF("match - pausing\n");
                *state = s;
                *c_final = c - 1;
                return MO_MATCHES_PENDING;
            }

            u64a loc = (c - 1) - buf + offAdj + 1;
            if (single) {
                DEBUG_PRINTF("reporting %u\n", m->arb_report);
                if (cb(0, loc, m->arb_report, ctxt) == MO_HALT_MATCHING) {
                    return MO_DEAD;
                }
            } else if (doComplexReport64(cb, ctxt, m, s, loc, 0,
                                         &cached_accept_state,
                                         &cached_accept_id)
                       == MO_HALT_MATCHING) {
                return MO_DEAD;
            }
        }

        assert(c <= c_end);
    } while (c < c_end);

exit:
    *state = s;
    if (mode == STOP_AT_MATCH) {
        *c_final = c_end;
    }
    return MO_ALIVE;
}

static never_inline
char mcsheng64Exec8_i_cb(const struct mcsheng64 *m, u32 *state, const u8 *buf,
                         size_t len, u64a offAdj, NfaCallback cb, void *ctxt,
                         char single, const u8 **final_point) {
    return mcsheng64Exec8_i(m, state, buf, len, offAdj, cb, ctxt, single,
                            final_point, CALLBACK_OUTPUT);
}

static never_inline
char mcsheng64Exec8_i_sam(const struct mcsheng64 *m, u32 *state, const u8 *buf,
                          size_t len, u64a offAdj, NfaCallback cb, void *ctxt,
                          char single, const u8 **final_point) {
    return mcsheng64Exec8_i(m, state, buf, len, offAdj, cb, ctxt, single,
                            final_point, STOP_AT_MATCH);
}

static never_inline
char mcsheng64Exec8_i_nm(const struct mcsheng64 *m, u32 *state, const u8 *buf,
                         size_t len, u64a offAdj, NfaCallback cb, void *ctxt,
                         char single, const u8 **final_point) {
    return mcsheng64Exec8_i(m, state, buf, len, offAdj, cb, ctxt, single,
                            final_point, NO_MATCHES);
}

static really_inline
char mcsheng64Exec8_i_ni(const struct mcsheng64 *m, u32 *state, const u8 *buf,
                         size_t len, u64a offAdj, NfaCallback cb, void *ctxt,
                         char single, const u8 **final_point,
                         enum MatchMode mode) {
    if (mode == CALLBACK_OUTPUT) {
        return mcsheng64Exec8_i_cb(m, state, buf, len, offAdj, cb, ctxt, single,
                                   final_point);
    } else if (mode == STOP_AT_MATCH) {
        return mcsheng64Exec8_i_sam(m, state, buf, len, offAdj, cb, ctxt,
                                    single, final_point);
    } else {
        assert(mode == NO_MATCHES);
        return mcsheng64Exec8_i_nm(m, state, buf, len, offAdj, cb, ctxt, single,
                                   final_point);
    }
}

static really_inline
char mcshengCheckEOD64(const struct NFA *nfa, u32 s, u64a offset,
                       NfaCallback cb, void *ctxt) {
    const struct mcsheng64 *m = getImplNfa(nfa);
    const struct mstate_aux *aux = get_aux64(m, s);

    if (!aux->accept_eod) {
        return MO_CONTINUE_MATCHING;
    }
    return doComplexReport64(cb, ctxt, m, s, offset, 1, NULL, NULL);
}

static really_inline
char nfaExecMcSheng64_16_Q2i(const struct NFA *n, u64a offset, const u8 *buffer,
                             const u8 *hend, NfaCallback cb, void *context,
                             struct mq *q, char single, s64a end,
                             enum MatchMode mode) {
    assert(n->type == MCSHENG_64_NFA_16);
    const struct mcsheng64 *m = getImplNfa(n);
    s64a sp;

    assert(ISALIGNED_N(q->state, 2));
    u32 s = *(u16 *)q->state;

    if (q->report_current) {
        assert(s);
        assert(get_aux64(m, s)->accept);

        int rv;
        if (single) {
            DEBUG_PRINTF("reporting %u\n", m->arb_report);
            rv = cb(0, q_cur_offset(q), m->arb_report, context);
        } else {
            u32 cached_accept_id = 0;
            u32 cached_accept_state = 0;

            rv = doComplexReport64(cb, context, m, s, q_cur_offset(q), 0,
                                   &cached_accept_state, &cached_accept_id);
        }

        q->report_current = 0;

        if (rv == MO_HALT_MATCHING) {
            return MO_DEAD;
        }
    }

    sp = q_cur_loc(q);
    q->cur++;

    const u8 *cur_buf = sp < 0 ? hend : buffer;

    assert(q->cur);
    if (mode != NO_MATCHES && q->items[q->cur - 1].location > end) {
        DEBUG_PRINTF("this is as far as we go\n");
        q->cur--;
        q->items[q->cur].type = MQE_START;
        q->items[q->cur].location = end;
        *(u16 *)q->state = s;
        return MO_ALIVE;
    }

    while (1) {
        assert(q->cur < q->end);
        s64a ep = q->items[q->cur].location;
        if (mode != NO_MATCHES) {
            ep = MIN(ep, end);
        }

        assert(ep >= sp);

        s64a local_ep = ep;
        if (sp < 0) {
            local_ep = MIN(0, ep);
        }

        /* do main buffer region */
        const u8 *final_look;
        char rv = mcsheng64Exec16_i_ni(m, &s, cur_buf + sp, local_ep - sp,
                                       offset + sp, cb, context, single,
                                       &final_look, mode);
        if (rv == MO_DEAD) {
            *(u16 *)q->state = 0;
            return MO_DEAD;
        }
        if (mode == STOP_AT_MATCH && rv == MO_MATCHES_PENDING) {
            DEBUG_PRINTF("this is as far as we go\n");
            DEBUG_PRINTF("state %u final_look %zd\n", s, final_look - cur_buf);

            assert(q->cur);
            assert(final_look != cur_buf + local_ep);

            q->cur--;
            q->items[q->cur].type = MQE_START;
            q->items[q->cur].location = final_look - cur_buf + 1; /* due to
                                                                   * early -1 */
            *(u16 *)q->state = s;
            return MO_MATCHES_PENDING;
        }

        assert(rv == MO_ALIVE);
        assert(q->cur);
        if (mode != NO_MATCHES && q->items[q->cur].location > end) {
            DEBUG_PRINTF("this is as far as we go\n");
            q->cur--;
            q->items[q->cur].type = MQE_START;
            q->items[q->cur].location = end;
            *(u16 *)q->state = s;
            return MO_ALIVE;
        }

        sp = local_ep;

        if (sp == 0) {
            cur_buf = buffer;
        }

        if (sp != ep) {
            continue;
        }

        switch (q->items[q->cur].type) {
        case MQE_TOP:
            assert(sp + offset || !s);
            if (sp + offset == 0) {
                s = m->start_anchored;
                break;
            }
            s = mcshengEnableStarts64(m, s);
            break;
        case MQE_END:
            *(u16 *)q->state = s;
            q->cur++;
            return s ? MO_ALIVE : MO_DEAD;
        default:
            assert(!"invalid queue event");
        }

        q->cur++;
    }
}

static really_inline
char nfaExecMcSheng64_8_Q2i(const struct NFA *n, u64a offset, const u8 *buffer,
                            const u8 *hend, NfaCallback cb, void *context,
                            struct mq *q, char single, s64a end,
                            enum MatchMode mode) {
    assert(n->type == MCSHENG_64_NFA_8);
    const struct mcsheng64 *m = getImplNfa(n);
    s64a sp;

    u32 s = *(u8 *)q->state;

    if (q->report_current) {
        assert(s);
        assert(s >= m->accept_limit_8);

        int rv;
        if (single) {
            DEBUG_PRINTF("reporting %u\n", m->arb_report);

            rv = cb(0, q_cur_offset(q), m->arb_report, context);
        } else {
            u32 cached_accept_id = 0;
            u32 cached_accept_state = 0;

            rv = doComplexReport64(cb, context, m, s, q_cur_offset(q), 0,
                                   &cached_accept_state, &cached_accept_id);
        }

        q->report_current = 0;

        if (rv == MO_HALT_MATCHING) {
            return MO_DEAD;
        }
    }

    sp = q_cur_loc(q);
    q->cur++;

    const u8 *cur_buf = sp < 0 ? hend : buffer;

    if (mode != NO_MATCHES && q->items[q->cur - 1].location > end) {
        DEBUG_PRINTF("this is as far as we go\n");
        q->cur--;
        q->items[q->cur].type = MQE_START;
        q->items[q->cur].location = end;
        *(u8 *)q->state = s;
        return MO_ALIVE;
    }

    while (1) {
        DEBUG_PRINTF("%s @ %llu\n", q->items[q->cur].type == MQE_TOP ? "TOP" :
                     q->items[q->cur].type == MQE_END ? "END" : "???",
                     q->items[q->cur].location + offset);
        assert(q->cur < q->end);
        s64a ep = q->items[q->cur].location;
        if (mode != NO_MATCHES) {
            ep = MIN(ep, end);
        }

        assert(ep >= sp);

        s64a local_ep = ep;
        if (sp < 0) {
            local_ep = MIN(0, ep);
        }

        const u8 *final_look;
        char rv = mcsheng64Exec8_i_ni(m, &s, cur_buf + sp, local_ep - sp,
                                      offset + sp, cb, context, single,
                                      &final_look, mode);
        if (rv == MO_HALT_MATCHING) {
            *(u8 *)q->state = 0;
            return MO_DEAD;
        }
        if (mode == STOP_AT_MATCH && rv == MO_MATCHES_PENDING) {
            DEBUG_PRINTF("this is as far as we go\n");
            DEBUG_PRINTF("state %u final_look %zd\n", s, final_look - cur_buf);

            assert(q->cur);
            assert(final_look != cur_buf + local_ep);

            q->cur--;
            q->items[q->cur].type = MQE_START;
            q->items[q->cur].location = final_look - cur_buf + 1; /* due to
                                                                   * early -1 */
            *(u8 *)q->state = s;
            return MO_MATCHES_PENDING;
        }

        assert(rv == MO_ALIVE);
        assert(q->cur);
        if (mode != NO_MATCHES && q->items[q->cur].location > end) {
            DEBUG_PRINTF("this is as far as we go\n");
            assert(q->cur);
            q->cur--;
            q->items[q->cur].type = MQE_START;
            q->items[q->cur].location = end;
            *(u8 *)q->state = s;
            return MO_ALIVE;
        }

        sp = local_ep;

        if (sp == 0) {
            cur_buf = buffer;
        }

        if (sp != ep) {
            continue;
        }

        switch (q->items[q->cur].type) {
        case MQE_TOP:
            assert(sp + offset || !s);
            if (sp + offset == 0) {
                s = (u8)m->start_anchored;
                break;
            }
            s = mcshengEnableStarts64(m, s);
            break;
        case MQE_END:
            *(u8 *)q->state = s;
            q->cur++;
            return s ? MO_ALIVE : MO_DEAD;
        default:
            assert(!"invalid queue event");
        }

        q->cur++;
    }
}

char nfaExecMcSheng64_8_Q(const struct NFA *n, struct mq *q, s64a end) {
    u64a offset = q->offset;
    const u8 *buffer = q->buffer;
    NfaCallback cb = q->cb;
    void *context = q->context;
    assert(n->type == MCSHENG_64_NFA_8);
    const struct mcsheng64 *m = getImplNfa(n);
    const u8 *hend = q->history + q->hlength;

    return nfaExecMcSheng64_8_Q2i(n, offset, buffer, hend, cb, context, q,
                                  m->flags & MCSHENG_FLAG_SINGLE, end,
                                  CALLBACK_OUTPUT);
}

char nfaExecMcSheng64_16_Q(const struct NFA *n, struct mq *q, s64a end) {
    u64a offset = q->offset;
    const u8 *buffer = q->buffer;
    NfaCallback cb = q->cb;
    void *context = q->context;
    assert(n->type == MCSHENG_64_NFA_16);
    const struct mcsheng64 *m = getImplNfa(n);
    const u8 *hend = q->history + q->hlength;

    return nfaExecMcSheng64_16_Q2i(n, offset, buffer, hend, cb, context, q,
                                   m->flags & MCSHENG_FLAG_SINGLE, end,
                                   CALLBACK_OUTPUT);
}

char nfaExecMcSheng64_8_reportCurrent(const struct NFA *n, struct mq *q) {
    const struct mcsheng64 *m = getImplNfa(n);
    NfaCallback cb = q->cb;
    void *ctxt = q->context;
    u32 s = *(u8 *)q->state;
    u8 single = m->flags & MCSHENG_FLAG_SINGLE;
    u64a offset = q_cur_offset(q);
    assert(q_cur_type(q) == MQE_START);
    assert(s);

    if (s >= m->accept_limit_8) {
        if (single) {
            DEBUG_PRINTF("reporting %u\n", m->arb_report);
            cb(0, offset, m->arb_report, ctxt);
        } else {
            u32 cached_accept_id = 0;
            u32 cached_accept_state = 0;

            doComplexReport64(cb, ctxt, m, s, offset, 0, &cached_accept_state,
                              &cached_accept_id);
        }
    }

    return 0;
}

char nfaExecMcSheng64_16_reportCurrent(const struct NFA *n, struct mq *q) {
    const struct mcsheng64 *m = getImplNfa(n);
    NfaCallback cb = q->cb;
    void *ctxt = q->context;
    u32 s = *(u16 *)q->state;
    const struct mstate_aux *aux = get_aux64(m, s);
    u8 single = m->flags & MCSHENG_FLAG_SINGLE;
    u64a offset = q_cur_offset(q);
    assert(q_cur_type(q) == MQE_START);
    DEBUG_PRINTF("state %u\n", s);
    assert(s);

    if (aux->accept) {
        if (single) {
            DEBUG_PRINTF("reporting %u\n", m->arb_report);
            cb(0, offset, m->arb_report, ctxt);
        } else {
            u32 cached_accept_id = 0;
            u32 cached_accept_state = 0;

            doComplexReport64(cb, ctxt, m, s, offset, 0, &cached_accept_state,
                              &cached_accept_id);
        }
    }

    return 0;
}

static
char mcshengHasAccept64(const struct mcsheng64 *m, const struct mstate_aux *aux,
                        ReportID report) {
    assert(m && aux);

    if (!aux->accept) {
        return 0;
    }

    const struct report_list *rl = (const struct report_list *)
            ((const char *)m + aux->accept - sizeof(struct NFA));
    assert(ISALIGNED_N(rl, 4));

    DEBUG_PRINTF("report list has %u entries\n", rl->count);

    for (u32 i = 0; i < rl->count; i++) {
        if (rl->report[i] == report) {
            return 1;
        }
    }

    return 0;
}

char nfaExecMcSheng64_8_inAccept(const struct NFA *n, ReportID report,
                                 struct mq *q) {
    assert(n && q);

    const struct mcsheng64 *m = getImplNfa(n);
    u8 s = *(u8 *)q->state;
    DEBUG_PRINTF("checking accepts for %hhu\n", s);

    return mcshengHasAccept64(m, get_aux64(m, s), report);
}

char nfaExecMcSheng64_8_inAnyAccept(const struct NFA *n, struct mq *q) {
    assert(n && q);

    const struct mcsheng64 *m = getImplNfa(n);
    u8 s = *(u8 *)q->state;
    DEBUG_PRINTF("checking accepts for %hhu\n", s);

    return !!get_aux64(m, s)->accept;
}

char nfaExecMcSheng64_16_inAccept(const struct NFA *n, ReportID report,
                                  struct mq *q) {
    assert(n && q);

    const struct mcsheng64 *m = getImplNfa(n);
    u16 s = *(u16 *)q->state;
    DEBUG_PRINTF("checking accepts for %hu\n", s);

    return mcshengHasAccept64(m, get_aux64(m, s), report);
}

char nfaExecMcSheng64_16_inAnyAccept(const struct NFA *n, struct mq *q) {
    assert(n && q);

    const struct mcsheng64 *m = getImplNfa(n);
    u16 s = *(u16 *)q->state;
    DEBUG_PRINTF("checking accepts for %hu\n", s);

    return !!get_aux64(m, s)->accept;
}

char nfaExecMcSheng64_8_Q2(const struct NFA *n, struct mq *q, s64a end) {
    u64a offset = q->offset;
    const u8 *buffer = q->buffer;
    NfaCallback cb = q->cb;
    void *context = q->context;
    assert(n->type == MCSHENG_64_NFA_8);
    const struct mcsheng64 *m = getImplNfa(n);
    const u8 *hend = q->history + q->hlength;

    return nfaExecMcSheng64_8_Q2i(n, offset, buffer, hend, cb, context, q,
                                  m->flags & MCSHENG_FLAG_SINGLE, end,
                                  STOP_AT_MATCH);
}

char nfaExecMcSheng64_16_Q2(const struct NFA *n, struct mq *q, s64a end) {
    u64a offset = q->offset;
    const u8 *buffer = q->buffer;
    NfaCallback cb = q->cb;
    void *context = q->context;
    assert(n->type == MCSHENG_64_NFA_16);
    const struct mcsheng64 *m = getImplNfa(n);
    const u8 *hend = q->history + q->hlength;

    return nfaExecMcSheng64_16_Q2i(n, offset, buffer, hend, cb, context, q,
                                   m->flags & MCSHENG_FLAG_SINGLE, end,
                                   STOP_AT_MATCH);
}

char nfaExecMcSheng64_8_QR(const struct NFA *n, struct mq *q, ReportID report) {
    u64a offset = q->offset;
    const u8 *buffer = q->buffer;
    NfaCallback cb = q->cb;
    void *context = q->context;
    assert(n->type == MCSHENG_64_NFA_8);
    const struct mcsheng64 *m = getImplNfa(n);
    const u8 *hend = q->history + q->hlength;

    char rv = nfaExecMcSheng64_8_Q2i(n, offset, buffer, hend, cb, context, q,
                                     m->flags & MCSHENG_FLAG_SINGLE,
                                     0 /* end */, NO_MATCHES);
    if (rv && nfaExecMcSheng64_8_inAccept(n, report, q)) {
        return MO_MATCHES_PENDING;
    } else {
        return rv;
    }
}

char nfaExecMcSheng64_16_QR(const struct NFA *n, struct mq *q, ReportID report) {
    u64a offset = q->offset;
    const u8 *buffer = q->buffer;
    NfaCallback cb = q->cb;
    void *context = q->context;
    assert(n->type == MCSHENG_64_NFA_16);
    const struct mcsheng64 *m = getImplNfa(n);
    const u8 *hend = q->history + q->hlength;

    char rv = nfaExecMcSheng64_16_Q2i(n, offset, buffer, hend, cb, context, q,
                                      m->flags & MCSHENG_FLAG_SINGLE,
                                      0 /* end */, NO_MATCHES);

    if (rv && nfaExecMcSheng64_16_inAccept(n, report, q)) {
        return MO_MATCHES_PENDING;
    } else {
        return rv;
    }
}

char nfaExecMcSheng64_8_initCompressedState(const struct NFA *nfa, u64a offset,
                                            void *state, UNUSED u8 key) {
    const struct mcsheng64 *m = getImplNfa(nfa);
    u8 s = offset ? m->start_floating : m->start_anchored;
    if (s) {
        *(u8 *)state = s;
        return 1;
    }
    return 0;
}

char nfaExecMcSheng64_16_initCompressedState(const struct NFA *nfa, u64a offset,
                                             void *state, UNUSED u8 key) {
    const struct mcsheng64 *m = getImplNfa(nfa);
    u16 s = offset ? m->start_floating : m->start_anchored;
    if (s) {
        unaligned_store_u16(state, s);
        return 1;
    }
    return 0;
}

char nfaExecMcSheng64_8_testEOD(const struct NFA *nfa, const char *state,
                                UNUSED const char *streamState, u64a offset,
                                NfaCallback callback, void *context) {
    return mcshengCheckEOD64(nfa, *(const u8 *)state, offset, callback,
                             context);
}

char nfaExecMcSheng64_16_testEOD(const struct NFA *nfa, const char *state,
                                 UNUSED const char *streamState, u64a offset,
                                 NfaCallback callback, void *context) {
    assert(ISALIGNED_N(state, 2));
    return mcshengCheckEOD64(nfa, *(const u16 *)state, offset, callback,
                             context);
}

char nfaExecMcSheng64_8_queueInitState(UNUSED const struct NFA *nfa, struct mq *q) {
    assert(nfa->scratchStateSize == 1);
    *(u8 *)q->state = 0;
    return 0;
}

char nfaExecMcSheng64_16_queueInitState(UNUSED const struct NFA *nfa, struct mq *q) {
    assert(nfa->scratchStateSize == 2);
    assert(ISALIGNED_N(q->state, 2));
    *(u16 *)q->state = 0;
    return 0;
}

char nfaExecMcSheng64_8_queueCompressState(UNUSED const struct NFA *nfa,
                                           const struct mq *q, UNUSED s64a loc) {
    void *dest = q->streamState;
    const void *src = q->state;
    assert(nfa->scratchStateSize == 1);
    assert(nfa->streamStateSize == 1);
    *(u8 *)dest = *(const u8 *)src;
    return 0;
}

char nfaExecMcSheng64_8_expandState(UNUSED const struct NFA *nfa, void *dest,
                                    const void *src, UNUSED u64a offset,
                                    UNUSED u8 key) {
    assert(nfa->scratchStateSize == 1);
    assert(nfa->streamStateSize == 1);
    *(u8 *)dest = *(const u8 *)src;
    return 0;
}

char nfaExecMcSheng64_16_queueCompressState(UNUSED const struct NFA *nfa,
                                            const struct mq *q,
                                            UNUSED s64a loc) {
    void *dest = q->streamState;
    const void *src = q->state;
    assert(nfa->scratchStateSize == 2);
    assert(nfa->streamStateSize == 2);
    assert(ISALIGNED_N(src, 2));
    unaligned_store_u16(dest, *(const u16 *)(src));
    return 0;
}

char nfaExecMcSheng64_16_expandState(UNUSED const struct NFA *nfa, void *dest,
                                     const void *src, UNUSED u64a offset,
                                     UNUSED u8 key) {
    assert(nfa->scratchStateSize == 2);
    assert(nfa->streamStateSize == 2);
    assert(ISALIGNED_N(dest, 2));
    *(u16 *)dest = unaligned_load_u16(src);
    return 0;
}
#endif
