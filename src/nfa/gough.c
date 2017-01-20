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

#include "gough.h"

#include "accel.h"
#include "gough_internal.h"
#include "mcclellan.h"
#include "nfa_api.h"
#include "nfa_api_queue.h"
#include "nfa_internal.h"
#include "util/bitutils.h"
#include "util/compare.h"
#include "util/simd_utils.h"
#include "util/unaligned.h"
#include "ue2common.h"
#include <string.h>

#include "mcclellan_common_impl.h"

#define GOUGH_SOM_EARLY (~0ULL)

static really_inline
void compressSomValue(u32 comp_slot_width, u64a curr_offset,
                      void *dest_som_base, u32 i, u64a val) {
    void *dest_som = (u8 *)dest_som_base + i * comp_slot_width;
    /* gough does not initialise all slots, so may contain garbage */
    u64a delta = curr_offset - val;
    switch (comp_slot_width) {
    case 2:
        if (delta >= (u16)~0U) {
            delta = GOUGH_SOM_EARLY;
        }
        unaligned_store_u16(dest_som, delta);
        break;
    case 4:
        if (delta >= (u32)~0U) {
            delta = GOUGH_SOM_EARLY;
        }
        unaligned_store_u32(dest_som, delta);
        break;
    case 8:
        if (delta >= ~0ULL) {
            delta = GOUGH_SOM_EARLY;
        }
        unaligned_store_u64a(dest_som, delta);
        break;
    default:
        assert(0);
    }
}

static really_inline
u64a expandSomValue(u32 comp_slot_width, u64a curr_offset,
                    const void *src_som_base, u32 i) {
    /* Note: gough does not initialise all slots, so we may end up decompressing
     * garbage */

    const void *src_som = (const u8 *)src_som_base + i * comp_slot_width;
    u64a val = 0;
    switch (comp_slot_width) {
    case 2:
        val = unaligned_load_u16(src_som);
        if (val == (u16)~0U) {
            return GOUGH_SOM_EARLY;
        }
        break;
    case 4:
        val = unaligned_load_u32(src_som);
        if (val == (u32)~0U) {
            return GOUGH_SOM_EARLY;
        }
        break;
    case 8:
        val = unaligned_load_u64a(src_som);
        if (val == ~0ULL) {
            return GOUGH_SOM_EARLY;
        }
        break;

    default:
        assert(0);
    }
    return curr_offset - val;
}

static really_inline
char doReports(NfaCallback cb, void *ctxt, const struct mcclellan *m,
               const struct gough_som_info *som, u16 s, u64a loc,
               char eod, u16 * const cached_accept_state,
               u32 * const cached_accept_id, u32 * const cached_accept_som) {
    DEBUG_PRINTF("reporting state = %hu, loc=%llu, eod %hhu\n",
                 (u16)(s & STATE_MASK), loc, eod);

    if (!eod && s == *cached_accept_state) {
        u64a from = *cached_accept_som == INVALID_SLOT ? loc
                                               : som->slots[*cached_accept_som];
        if (cb(from, loc, *cached_accept_id, ctxt) == MO_HALT_MATCHING) {
            return MO_HALT_MATCHING; /* termination requested */
        }

        return MO_CONTINUE_MATCHING; /* continue execution */
    }

    const struct mstate_aux *aux = get_aux(m, s);
    size_t offset = eod ? aux->accept_eod : aux->accept;

    assert(offset);
    const struct gough_report_list *rl
        = (const void *)((const char *)m + offset - sizeof(struct NFA));
    assert(ISALIGNED(rl));

    DEBUG_PRINTF("report list size %u\n", rl->count);
    u32 count = rl->count;

    if (!eod && count == 1) {
        *cached_accept_state = s;
        *cached_accept_id = rl->report[0].r;
        *cached_accept_som = rl->report[0].som;

        u64a from = *cached_accept_som == INVALID_SLOT ? loc
                                               : som->slots[*cached_accept_som];
        DEBUG_PRINTF("reporting %u, using som[%u]=%llu\n", rl->report[0].r,
                     *cached_accept_som, from);
        if (cb(from, loc, *cached_accept_id, ctxt) == MO_HALT_MATCHING) {
            return MO_HALT_MATCHING; /* termination requested */
        }

        return MO_CONTINUE_MATCHING; /* continue execution */
    }

    for (u32 i = 0; i < count; i++) {
        u32 slot = rl->report[i].som;
        u64a from = slot == INVALID_SLOT ? loc : som->slots[slot];
        DEBUG_PRINTF("reporting %u, using som[%u] = %llu\n",
                     rl->report[i].r, slot, from);
        if (cb(from, loc, rl->report[i].r, ctxt) == MO_HALT_MATCHING) {
            return MO_HALT_MATCHING; /* termination requested */
        }
    }

    return MO_CONTINUE_MATCHING; /* continue execution */
}

#ifdef DUMP_SUPPORT
static UNUSED
const char *dump_op(u8 op) {
    switch (op) {
    case GOUGH_INS_END:
        return "END";
    case GOUGH_INS_MOV:
        return "MOV";
    case GOUGH_INS_NEW:
        return "NEW";
    case GOUGH_INS_MIN:
        return "MIN";
    default:
        return "???";
    }
}
#endif

static really_inline
void run_prog_i(UNUSED const struct NFA *nfa,
                const struct gough_ins *pc, u64a som_offset,
                struct gough_som_info *som) {
    DEBUG_PRINTF("run prog at som_offset of %llu\n", som_offset);
    while (1) {
        assert((const u8 *)pc >= (const u8 *)nfa);
        assert((const u8 *)pc < (const u8 *)nfa + nfa->length);
        u32 dest = pc->dest;
        u32 src = pc->src;
        assert(pc->op == GOUGH_INS_END
               || dest < (nfa->scratchStateSize - 16) / 8);
        DEBUG_PRINTF("%s %u %u\n", dump_op(pc->op), dest, src);
        switch (pc->op) {
        case GOUGH_INS_END:
            return;
        case GOUGH_INS_MOV:
            som->slots[dest] = som->slots[src];
            break;
        case GOUGH_INS_NEW:
            /* note: c has already been advanced */
            DEBUG_PRINTF("current offset %llu; adjust %u\n", som_offset,
                         pc->src);
            assert(som_offset >= pc->src);
            som->slots[dest] = som_offset - pc->src;
            break;
        case GOUGH_INS_MIN:
            /* TODO: shift all values along by one so that a normal min works
             */
            if (som->slots[src] == GOUGH_SOM_EARLY) {
                som->slots[dest] = som->slots[src];
            } else if (som->slots[dest] != GOUGH_SOM_EARLY) {
                LIMIT_TO_AT_MOST(&som->slots[dest], som->slots[src]);
            }
            break;
        default:
            assert(0);
            return;
        }
        DEBUG_PRINTF("dest slot[%u] = %llu\n", dest, som->slots[dest]);
        ++pc;
    }
}

static really_inline
void run_prog(const struct NFA *nfa, const u32 *edge_prog_table,
              const u8 *buf, u64a offAdj, const u8 *c, u32 edge_num,
              struct gough_som_info *som) {
    DEBUG_PRINTF("taking edge %u\n", edge_num);
    u32 prog_offset = edge_prog_table[edge_num];
    if (!prog_offset) {
        DEBUG_PRINTF("no prog on edge\n");
        return;
    }

    const struct gough_ins *pc = (const void *)((const u8 *)nfa + prog_offset);
    u64a curr_offset = (u64a)(c - buf) + offAdj - 1;
    run_prog_i(nfa, pc, curr_offset, som);
}

static never_inline
void run_accel_prog(const struct NFA *nfa, const struct gough_accel *gacc,
                    const u8 *buf, u64a offAdj, const u8 *c, const u8 *c2,
                    struct gough_som_info *som) {
    assert(gacc->prog_offset);
    assert(c2 > c);

    const struct gough_ins *pc
        = (const void *)((const u8 *)nfa + gacc->prog_offset);
    s64a margin_dist = gacc->margin_dist;

    DEBUG_PRINTF("run accel after skip %lld margin; advanced %zd\n",
                  margin_dist, c2 - c);

    if (c2 - c <= 2 * margin_dist) {
        while (c < c2) {
            u64a curr_offset = (u64a)(c - buf) + offAdj;
            run_prog_i(nfa, pc, curr_offset, som);
            c++;
        }
    } else {
        u64a curr_offset = (u64a)(c - buf) + offAdj;
        for (s64a i = 0; i < margin_dist; i++) {
            run_prog_i(nfa, pc, curr_offset + i, som);
        }

        curr_offset = (u64a)(c2 - buf) + offAdj - margin_dist;
        for (s64a i = 0; i < margin_dist; i++) {
            run_prog_i(nfa, pc, curr_offset + i, som);
        }
    }
}

static never_inline
u16 goughEnableStarts(const struct mcclellan *m, u16 s, u64a som_offset,
                      struct gough_som_info *som) {
    DEBUG_PRINTF("top triggered while at %hu\n", s);
    const struct mstate_aux *aux = get_aux(m, s);
    DEBUG_PRINTF("now going to state %hu\n", aux->top);

    const u32 *top_offsets = get_gough_top_offsets(m);
    if (!top_offsets) {
        return aux->top;
    }

    u32 prog_offset = top_offsets[s];
    if (!prog_offset) {
        return aux->top;
    }

    DEBUG_PRINTF("doing som for top\n");
    const struct NFA *nfa
        = (const struct NFA *)((const char *)m - sizeof(struct NFA));
    const struct gough_ins *pc = (const void *)((const u8 *)nfa
                                                + prog_offset);
    run_prog_i(nfa, pc, som_offset, som);
    return aux->top;
}

static really_inline
char goughExec16_i(const struct mcclellan *m, struct gough_som_info *som,
                   u16 *state, const u8 *buf, size_t len, u64a offAdj,
                   NfaCallback cb, void *ctxt, const u8 **c_final,
                   enum MatchMode mode) {
    assert(ISALIGNED_N(state, 2));

    u16 s = *state;
    const struct NFA *nfa
        = (const struct NFA *)((const char *)m - sizeof(struct NFA));
    const u8 *c = buf, *c_end = buf + len;
    const u16 *succ_table = (const u16 *)((const char *)m
                                          + sizeof(struct mcclellan));
    assert(ISALIGNED_N(succ_table, 2));
    const u16 sherman_base = m->sherman_limit;
    const char *sherman_base_offset
        = (const char *)nfa + m->sherman_offset;
    const u32 as = m->alphaShift;

    s &= STATE_MASK;

    u32 cached_accept_id = 0;
    u16 cached_accept_state = 0;
    u32 cached_accept_som = 0;

    const u32 *edge_prog_table = (const u32 *)(get_gough(m) + 1);

    DEBUG_PRINTF("s: %hu, len %zu\n", s, len);

    const u8 *min_accel_offset = c;
    if (!m->has_accel || len < ACCEL_MIN_LEN) {
        min_accel_offset = c_end;
        goto without_accel;
    }

    goto with_accel;

without_accel:
    while (c < min_accel_offset && s) {
        u8 cprime = m->remap[*(c++)];
        DEBUG_PRINTF("c: %02hhx cp:%02hhx (s=%hu)\n", *(c-1), cprime, s);

        u32 edge_num = ((u32)s << as) + cprime;
        run_prog(nfa, edge_prog_table, buf, offAdj, c, edge_num, som);
        if (s < sherman_base) {
            DEBUG_PRINTF("doing normal\n");
            assert(s < m->state_count);
            s = succ_table[edge_num];
        } else {
            const char *sherman_state
                = findShermanState(m, sherman_base_offset, sherman_base, s);
            DEBUG_PRINTF("doing sherman\n");
            s = doSherman16(sherman_state, cprime, succ_table, as);
        }
        DEBUG_PRINTF("s: %hu (%hu)\n", s, (u16)(s & STATE_MASK));

        if (mode != NO_MATCHES && (s & ACCEPT_FLAG)) {
            if (mode == STOP_AT_MATCH) {
                *state = s & STATE_MASK;
                *c_final = c - 1;
                return MO_CONTINUE_MATCHING;
            }

            u64a loc = (c - 1) - buf + offAdj + 1;
            if (doReports(cb, ctxt, m, som, s & STATE_MASK, loc, 0,
                                &cached_accept_state, &cached_accept_id,
                                &cached_accept_som) == MO_HALT_MATCHING) {
                return MO_HALT_MATCHING;
            }
        }

        s &= STATE_MASK;
    }

with_accel:
    while (c < c_end && s) {
        u8 cprime = m->remap[*(c++)];
        DEBUG_PRINTF("c: %02hhx cp:%02hhx (s=%hu)\n", *(c-1), cprime, s);

        u32 edge_num = ((u32)s << as) + cprime;
        run_prog(nfa, edge_prog_table, buf, offAdj, c, edge_num, som);
        if (s < sherman_base) {
            DEBUG_PRINTF("doing normal\n");
            assert(s < m->state_count);
            s = succ_table[edge_num];
        } else {
            const char *sherman_state
                = findShermanState(m, sherman_base_offset, sherman_base, s);
            DEBUG_PRINTF("doing sherman\n");
            s = doSherman16(sherman_state, cprime, succ_table, as);
        }
        DEBUG_PRINTF("s: %hu (%hu)\n", s, (u16)(s & STATE_MASK));

        if (mode != NO_MATCHES && (s & ACCEPT_FLAG)) {
            if (mode == STOP_AT_MATCH) {
                *state = s & STATE_MASK;
                *c_final = c - 1;
                return MO_CONTINUE_MATCHING;
            }

            u64a loc = (c - 1) - buf + offAdj + 1;

            if (doReports(cb, ctxt, m, som, s & STATE_MASK, loc, 0,
                          &cached_accept_state, &cached_accept_id,
                          &cached_accept_som)
                == MO_HALT_MATCHING) {
                return MO_HALT_MATCHING;
            }
        } else if (s & ACCEL_FLAG) {
            DEBUG_PRINTF("skipping\n");
            const struct mstate_aux *this_aux = get_aux(m, s & STATE_MASK);
            u32 accel_offset = this_aux->accel_offset;

            assert(accel_offset >= m->aux_offset);
            assert(accel_offset < m->sherman_offset);

            const struct gough_accel *gacc
                = (const void *)((const char *)m + accel_offset);
            assert(!gacc->prog_offset == !gacc->margin_dist);
            const u8 *c2 = run_accel(&gacc->accel, c, c_end);

            if (c2 != c && gacc->prog_offset) {
                run_accel_prog(nfa, gacc, buf, offAdj, c, c2, som);
            }

            if (c2 < min_accel_offset + BAD_ACCEL_DIST) {
                min_accel_offset = c2 + BIG_ACCEL_PENALTY;
            } else {
                min_accel_offset = c2 + SMALL_ACCEL_PENALTY;
            }

            if (min_accel_offset >= c_end - ACCEL_MIN_LEN) {
                min_accel_offset = c_end;
            }

            DEBUG_PRINTF("advanced %zd, next accel chance in %zd/%zd\n",
                         c2 - c, min_accel_offset - c2, c_end - c2);

            c = c2;
            s &= STATE_MASK;
            goto without_accel;
        }

        s &= STATE_MASK;
    }

    if (mode == STOP_AT_MATCH) {
        *c_final = c_end;
    }
    *state = s;

    return MO_CONTINUE_MATCHING;
}

static really_inline
char goughExec8_i(const struct mcclellan *m, struct gough_som_info *som,
                  u8 *state, const u8 *buf, size_t len, u64a offAdj,
                  NfaCallback cb, void *ctxt, const u8 **c_final,
                  enum MatchMode mode) {
    u8 s = *state;
    const u8 *c = buf, *c_end = buf + len;
    const u8 *succ_table = (const u8 *)((const char *)m
                                        + sizeof(struct mcclellan));
    const u32 as = m->alphaShift;
    const struct mstate_aux *aux;

    const struct NFA *nfa
        = (const struct NFA *)((const char *)m - sizeof(struct NFA));
    aux = (const struct mstate_aux *)((const char *)nfa + m->aux_offset);

    const u32 *edge_prog_table = (const u32 *)(get_gough(m) + 1);

    u16 accel_limit = m->accel_limit_8;
    u16 accept_limit = m->accept_limit_8;

    u32 cached_accept_id = 0;
    u16 cached_accept_state = 0;
    u32 cached_accept_som = 0;

    DEBUG_PRINTF("accel %hu, accept %hu\n", accel_limit, accept_limit);

    DEBUG_PRINTF("s: %hhu, len %zu\n", s, len);

    const u8 *min_accel_offset = c;
    if (!m->has_accel || len < ACCEL_MIN_LEN) {
        min_accel_offset = c_end;
        goto without_accel;
    }

    goto with_accel;

without_accel:
    while (c < min_accel_offset && s) {
        u8 cprime = m->remap[*(c++)];
        DEBUG_PRINTF("c: %02hhx '%c' cp:%02hhx\n", *(c-1),
                     ourisprint(*(c-1)) ? *(c-1) : '?', cprime);

        u32 edge_num = ((u32)s << as) + cprime;

        run_prog(nfa, edge_prog_table, buf, offAdj, c, edge_num, som);

        s = succ_table[edge_num];
        DEBUG_PRINTF("s: %hhu\n", s);

        if (mode != NO_MATCHES && s >= accept_limit) {
            if (mode == STOP_AT_MATCH) {
                DEBUG_PRINTF("match - pausing\n");
                *state = s;
                *c_final = c - 1;
                return MO_CONTINUE_MATCHING;
            }

            u64a loc = (c - 1) - buf + offAdj + 1;
            if (doReports(cb, ctxt, m, som, s, loc, 0,
                                &cached_accept_state, &cached_accept_id,
                                &cached_accept_som)
                       == MO_HALT_MATCHING) {
                return MO_HALT_MATCHING;
            }
        }
    }

with_accel:
    while (c < c_end && s) {
        u8 cprime = m->remap[*(c++)];
        DEBUG_PRINTF("c: %02hhx '%c' cp:%02hhx\n", *(c-1),
                     ourisprint(*(c-1)) ? *(c-1) : '?', cprime);

        u32 edge_num = ((u32)s << as) + cprime;

        run_prog(nfa, edge_prog_table, buf, offAdj, c, edge_num, som);

        s = succ_table[edge_num];
        DEBUG_PRINTF("s: %hhu\n", s);

        if (s >= accel_limit) { /* accept_limit >= accel_limit */
            if (mode != NO_MATCHES && s >= accept_limit) {
                if (mode == STOP_AT_MATCH) {
                    DEBUG_PRINTF("match - pausing\n");
                    *state = s;
                    *c_final = c - 1;
                    return MO_CONTINUE_MATCHING;
                }

                u64a loc = (c - 1) - buf + offAdj + 1;
                if (doReports(cb, ctxt, m, som, s, loc, 0,
                                    &cached_accept_state, &cached_accept_id,
                                    &cached_accept_som)
                           == MO_HALT_MATCHING) {
                    return MO_HALT_MATCHING;
                }
            } else if (aux[s].accel_offset) {
                DEBUG_PRINTF("skipping\n");

                const struct gough_accel *gacc
                    = (const void *)((const char *)m + aux[s].accel_offset);
                const u8 *c2 = run_accel(&gacc->accel, c, c_end);

                if (c2 != c && gacc->prog_offset) {
                    run_accel_prog(nfa, gacc, buf, offAdj, c, c2, som);
                }

                if (c2 < min_accel_offset + BAD_ACCEL_DIST) {
                    min_accel_offset = c2 + BIG_ACCEL_PENALTY;
                } else {
                    min_accel_offset = c2 + SMALL_ACCEL_PENALTY;
                }

                if (min_accel_offset >= c_end - ACCEL_MIN_LEN) {
                    min_accel_offset = c_end;
                }

                DEBUG_PRINTF("advanced %zd, next accel chance in %zd/%zd\n",
                             c2 - c, min_accel_offset - c2, c_end - c2);

                c = c2;
                goto without_accel;
            }
        }
    }

    *state = s;
    if (mode == STOP_AT_MATCH) {
        *c_final = c_end;
    }
    return MO_CONTINUE_MATCHING;
}

static never_inline
char goughExec8_i_ni(const struct mcclellan *m, struct gough_som_info *som,
                     u8 *state, const u8 *buf, size_t len, u64a offAdj,
                     NfaCallback cb, void *ctxt, const u8 **final_point,
                     enum MatchMode mode) {
    return goughExec8_i(m, som, state, buf, len, offAdj, cb, ctxt, final_point,
                        mode);
}

static never_inline
char goughExec16_i_ni(const struct mcclellan *m, struct gough_som_info *som,
                      u16 *state, const u8 *buf, size_t len, u64a offAdj,
                      NfaCallback cb, void *ctxt, const u8 **final_point,
                      enum MatchMode mode) {
    return goughExec16_i(m, som, state, buf, len, offAdj, cb, ctxt, final_point,
                         mode);
}

static really_inline
struct gough_som_info *getSomInfo(char *state_base) {
    return (struct gough_som_info *)(state_base + 16);
}

static really_inline
const struct gough_som_info *getSomInfoConst(const char *state_base) {
    return (const struct gough_som_info *)(state_base + 16);
}

static really_inline
char nfaExecGough8_Q2i(const struct NFA *n, u64a offset, const u8 *buffer,
                      const u8 *hend, NfaCallback cb, void *context,
                      struct mq *q, s64a end, enum MatchMode mode) {
    DEBUG_PRINTF("enter\n");
    struct gough_som_info *som = getSomInfo(q->state);
    assert(n->type == GOUGH_NFA_8);
    const struct mcclellan *m = (const struct mcclellan *)getImplNfa(n);
    s64a sp;
    u8 s = *(u8 *)q->state;

    if (q->report_current) {
        assert(s);
        assert(s >= m->accept_limit_8);

        u32 cached_accept_id = 0;
        u16 cached_accept_state = 0;
        u32 cached_accept_som = 0;

        int rv = doReports(cb, context, m, som, s, q_cur_offset(q), 0,
                           &cached_accept_state, &cached_accept_id,
                           &cached_accept_som);

        q->report_current = 0;

        if (rv == MO_HALT_MATCHING) {
            return MO_HALT_MATCHING;
        }
    }

    sp = q_cur_loc(q);
    q->cur++;

    const u8 *cur_buf = sp < 0 ? hend : buffer;

    if (mode != NO_MATCHES && q->items[q->cur - 1].location > end) {
        /* this is as far as we go */
        q->cur--;
        q->items[q->cur].type = MQE_START;
        q->items[q->cur].location = end;
        *(u8 *)q->state = s;
        return MO_ALIVE;
    }

    while (1) {
        DEBUG_PRINTF("%s @ %llu [som %llu]\n",
                     q->items[q->cur].type == MQE_TOP ? "TOP" :
                     q->items[q->cur].type == MQE_END ? "END" : "???",
                     q->items[q->cur].location + offset, q->items[q->cur].som);
        assert(q->cur < q->end);
        s64a ep = q->items[q->cur].location;
        if (mode != NO_MATCHES) {
            ep = MIN(ep, end);
        }

        assert(ep >= sp);
        DEBUG_PRINTF("run to %lld from %lld\n", ep, sp);

        s64a local_ep = ep;
        if (sp < 0) {
            local_ep = MIN(0, ep);
        }

        const u8 *final_look;
        if (goughExec8_i_ni(m, som, &s, cur_buf + sp, local_ep - sp,
                            offset + sp, cb, context, &final_look, mode)
            == MO_HALT_MATCHING) {
            *(u8 *)q->state = 0;
            return 0;
        }
        if (mode == STOP_AT_MATCH && final_look != cur_buf + local_ep) {
            /* found a match */
            DEBUG_PRINTF("found a match\n");
            assert(q->cur);
            q->cur--;
            q->items[q->cur].type = MQE_START;
            q->items[q->cur].location = final_look - cur_buf + 1; /* due to
                                                                   * early -1 */
            *(u8 *)q->state = s;
            return MO_MATCHES_PENDING;
        }

        assert(q->cur);
        if (mode != NO_MATCHES && q->items[q->cur].location > end) {
            /* this is as far as we go */
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
            assert(!s || sp + offset > 0);
            if (sp + offset == 0) {
                s = (u8)m->start_anchored;
                break;
            }
            s = goughEnableStarts(m, s, q->items[q->cur].som, som);
            break;
        case MQE_END:
            *(u8 *)q->state = s;
            q->cur++;
            return s ? MO_ALIVE : 0;
        default:
            assert(!"invalid queue event");
        }

        q->cur++;
    }
}


static really_inline
char nfaExecGough16_Q2i(const struct NFA *n, u64a offset, const u8 *buffer,
                       const u8 *hend, NfaCallback cb, void *context,
                       struct mq *q, s64a end, enum MatchMode mode) {
    struct gough_som_info *som = getSomInfo(q->state);
    assert(n->type == GOUGH_NFA_16);
    const struct mcclellan *m = (const struct mcclellan *)getImplNfa(n);
    s64a sp;

    assert(ISALIGNED_N(q->state, 2));
    u16 s = *(u16 *)q->state;

    if (q->report_current) {
        assert(s);
        assert(get_aux(m, s)->accept);

        u32 cached_accept_id = 0;
        u16 cached_accept_state = 0;
        u32 cached_accept_som = 0;

        int rv = doReports(cb, context, m, som, s, q_cur_offset(q), 0,
                                 &cached_accept_state, &cached_accept_id,
                                 &cached_accept_som);

        q->report_current = 0;

        if (rv == MO_HALT_MATCHING) {
            return MO_HALT_MATCHING;
        }
    }

    sp = q_cur_loc(q);
    q->cur++;

    const u8 *cur_buf = sp < 0 ? hend : buffer;

    assert(q->cur);
    if (mode != NO_MATCHES && q->items[q->cur - 1].location > end) {
        /* this is as far as we go */
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
        if (goughExec16_i_ni(m, som, &s, cur_buf + sp, local_ep - sp,
                             offset + sp, cb, context, &final_look, mode)
            == MO_HALT_MATCHING) {
            *(u16 *)q->state = 0;
            return 0;
        }
        if (mode == STOP_AT_MATCH && final_look != cur_buf + local_ep) {
            /* this is as far as we go */
            assert(q->cur);
            DEBUG_PRINTF("state %hu final_look %zd\n", s,
                          final_look - cur_buf);
            q->cur--;
            q->items[q->cur].type = MQE_START;
            q->items[q->cur].location = final_look - cur_buf + 1; /* due to
                                                                   * early -1 */
            *(u16 *)q->state = s;
            return MO_MATCHES_PENDING;
        }

        assert(q->cur);
        if (mode != NO_MATCHES && q->items[q->cur].location > end) {
            /* this is as far as we go */
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
            assert(!s || sp + offset > 0);
            if (sp + offset == 0) {
                s = m->start_anchored;
                break;
            }
            s = goughEnableStarts(m, s, q->items[q->cur].som, som);
            break;
        case MQE_END:
            *(u16 *)q->state = s;
            q->cur++;
            return s ? MO_ALIVE : 0;
        default:
            assert(!"invalid queue event");
        }

        q->cur++;
    }
}

char nfaExecGough8_Q(const struct NFA *n, struct mq *q, s64a end) {
    u64a offset = q->offset;
    const u8 *buffer = q->buffer;
    NfaCallback cb = q->cb;
    void *context = q->context;
    assert(n->type == GOUGH_NFA_8);
    const u8 *hend = q->history + q->hlength;

    return nfaExecGough8_Q2i(n, offset, buffer, hend, cb, context, q, end,
                            CALLBACK_OUTPUT);
}

char nfaExecGough16_Q(const struct NFA *n, struct mq *q, s64a end) {
    u64a offset = q->offset;
    const u8 *buffer = q->buffer;
    NfaCallback cb = q->cb;
    void *context = q->context;
    assert(n->type == GOUGH_NFA_16);
    const u8 *hend = q->history + q->hlength;

    return nfaExecGough16_Q2i(n, offset, buffer, hend, cb, context, q, end,
                              CALLBACK_OUTPUT);
}

char nfaExecGough8_Q2(const struct NFA *n, struct mq *q, s64a end) {
    u64a offset = q->offset;
    const u8 *buffer = q->buffer;
    NfaCallback cb = q->cb;
    void *context = q->context;
    assert(n->type == GOUGH_NFA_8);
    const u8 *hend = q->history + q->hlength;

    return nfaExecGough8_Q2i(n, offset, buffer, hend, cb, context, q, end,
                            STOP_AT_MATCH);
}

char nfaExecGough16_Q2(const struct NFA *n, struct mq *q, s64a end) {
    u64a offset = q->offset;
    const u8 *buffer = q->buffer;
    NfaCallback cb = q->cb;
    void *context = q->context;
    assert(n->type == GOUGH_NFA_16);
    const u8 *hend = q->history + q->hlength;

    return nfaExecGough16_Q2i(n, offset, buffer, hend, cb, context, q, end,
                              STOP_AT_MATCH);
}

char nfaExecGough8_QR(const struct NFA *n, struct mq *q, ReportID report) {
    u64a offset = q->offset;
    const u8 *buffer = q->buffer;
    NfaCallback cb = q->cb;
    void *context = q->context;
    assert(n->type == GOUGH_NFA_8);
    const u8 *hend = q->history + q->hlength;

    char rv = nfaExecGough8_Q2i(n, offset, buffer, hend, cb, context, q,
                                0 /* end */, NO_MATCHES);
    if (rv && nfaExecMcClellan8_inAccept(n, report, q)) {
        return MO_MATCHES_PENDING;
    } else {
        return rv;
    }
}

char nfaExecGough16_QR(const struct NFA *n, struct mq *q, ReportID report) {
    u64a offset = q->offset;
    const u8 *buffer = q->buffer;
    NfaCallback cb = q->cb;
    void *context = q->context;
    assert(n->type == GOUGH_NFA_16);
    const u8 *hend = q->history + q->hlength;

    char rv = nfaExecGough16_Q2i(n, offset, buffer, hend, cb, context, q,
                                 0 /* end */, NO_MATCHES);

    if (rv && nfaExecMcClellan16_inAccept(n, report, q)) {
        return MO_MATCHES_PENDING;
    } else {
        return rv;
    }
}

char nfaExecGough8_initCompressedState(const struct NFA *nfa, u64a offset,
                                       void *state, UNUSED u8 key) {
    const struct mcclellan *m = (const struct mcclellan *)getImplNfa(nfa);
    memset(state, 0, nfa->streamStateSize);
    u8 s = offset ? m->start_floating : m->start_anchored;
    if (s) {
        *(u8 *)state = s;
        return 1;
    }
    return 0;
}

char nfaExecGough16_initCompressedState(const struct NFA *nfa, u64a offset,
                                        void *state, UNUSED u8 key) {
    const struct mcclellan *m = (const struct mcclellan *)getImplNfa(nfa);
    memset(state, 0, nfa->streamStateSize);
    u16 s = offset ? m->start_floating : m->start_anchored;
    if (s) {
        unaligned_store_u16(state, s);
        return 1;
    }
    return 0;
}


char nfaExecGough8_reportCurrent(const struct NFA *n, struct mq *q) {
    const struct mcclellan *m = (const struct mcclellan *)getImplNfa(n);
    NfaCallback cb = q->cb;
    void *ctxt = q->context;
    u8 s = *(u8 *)q->state;
    u64a offset = q_cur_offset(q);
    struct gough_som_info *som = getSomInfo(q->state);
    assert(q_cur_type(q) == MQE_START);
    assert(s);

    if (s >= m->accept_limit_8) {
        u32 cached_accept_id = 0;
        u16 cached_accept_state = 0;
        u32 cached_accept_som = 0;

        doReports(cb, ctxt, m, som, s, offset, 0, &cached_accept_state,
                        &cached_accept_id, &cached_accept_som);
    }

    return 0;
}

char nfaExecGough16_reportCurrent(const struct NFA *n, struct mq *q) {
    const struct mcclellan *m = (const struct mcclellan *)getImplNfa(n);
    NfaCallback cb = q->cb;
    void *ctxt = q->context;
    u16 s = *(u16 *)q->state;
    const struct mstate_aux *aux = get_aux(m, s);
    u64a offset = q_cur_offset(q);
    struct gough_som_info *som = getSomInfo(q->state);
    assert(q_cur_type(q) == MQE_START);
    DEBUG_PRINTF("state %hu\n", s);
    assert(s);

    if (aux->accept) {
        u32 cached_accept_id = 0;
        u16 cached_accept_state = 0;
        u32 cached_accept_som = 0;

        doReports(cb, ctxt, m, som, s, offset, 0, &cached_accept_state,
                        &cached_accept_id, &cached_accept_som);
    }

    return 0;
}

char nfaExecGough8_inAccept(const struct NFA *n, ReportID report,
                            struct mq *q) {
    return nfaExecMcClellan8_inAccept(n, report, q);
}

char nfaExecGough16_inAccept(const struct NFA *n, ReportID report,
                             struct mq *q) {
    return nfaExecMcClellan16_inAccept(n, report, q);
}

char nfaExecGough8_inAnyAccept(const struct NFA *n, struct mq *q) {
    return nfaExecMcClellan8_inAnyAccept(n, q);
}

char nfaExecGough16_inAnyAccept(const struct NFA *n, struct mq *q) {
    return nfaExecMcClellan16_inAnyAccept(n, q);
}

static
char goughCheckEOD(const struct NFA *nfa, u16 s,
                   const struct gough_som_info *som,
                   u64a offset, NfaCallback cb, void *ctxt) {
    const struct mcclellan *m = (const struct mcclellan *)getImplNfa(nfa);
    const struct mstate_aux *aux = get_aux(m, s);

    if (!aux->accept_eod) {
        return MO_CONTINUE_MATCHING;
    }
    return doReports(cb, ctxt, m, som, s, offset, 1, NULL, NULL, NULL);
}

char nfaExecGough8_testEOD(const struct NFA *nfa, const char *state,
                           UNUSED const char *streamState, u64a offset,
                           NfaCallback callback, void *context) {
    const struct gough_som_info *som = getSomInfoConst(state);
    return goughCheckEOD(nfa, *(const u8 *)state, som, offset, callback,
                         context);
}

char nfaExecGough16_testEOD(const struct NFA *nfa, const char *state,
                            UNUSED const char *streamState, u64a offset,
                            NfaCallback callback, void *context) {
    assert(ISALIGNED_N(state, 8));
    const struct gough_som_info *som = getSomInfoConst(state);
    return goughCheckEOD(nfa, *(const u16 *)state, som, offset, callback,
                         context);
}

char nfaExecGough8_queueInitState(UNUSED const struct NFA *nfa, struct mq *q) {
    memset(q->state, 0, nfa->scratchStateSize);
    return 0;
}

char nfaExecGough16_queueInitState(UNUSED const struct NFA *nfa, struct mq *q) {
    memset(q->state, 0, nfa->scratchStateSize);
    assert(ISALIGNED_N(q->state, 2));
    return 0;
}

static really_inline
void compSomSpace(const struct NFA *nfa, u8 *dest_som_base,
                  const struct gough_som_info *src, u64a curr_offset) {
    const struct mcclellan *m = (const struct mcclellan *)getImplNfa(nfa);
    const struct gough_info *gi = get_gough(m);
    u32 count = gi->stream_som_loc_count;
    u32 width = gi->stream_som_loc_width;

    for (u32 i = 0; i < count; i++) {
        compressSomValue(width, curr_offset, dest_som_base, i, src->slots[i]);
    }
}

static really_inline
void expandSomSpace(const struct NFA *nfa, struct gough_som_info *som,
                    const u8 *src_som_base, u64a curr_offset) {
    const struct mcclellan *m = (const struct mcclellan *)getImplNfa(nfa);
    const struct gough_info *gi = get_gough(m);
    u32 count = gi->stream_som_loc_count;
    u32 width = gi->stream_som_loc_width;

    for (u32 i = 0; i < count; i++) {
        som->slots[i] = expandSomValue(width, curr_offset, src_som_base, i);
    }
}

char nfaExecGough8_queueCompressState(const struct NFA *nfa, const struct mq *q,
                                     s64a loc) {
    void *dest = q->streamState;
    const void *src = q->state;

    *(u8 *)dest = *(const u8 *)src;
    compSomSpace(nfa, (u8 *)dest + 1, getSomInfoConst(src), q->offset + loc);
    return 0;
}

char nfaExecGough8_expandState(const struct NFA *nfa, void *dest,
                              const void *src, u64a offset, UNUSED u8 key) {
    *(u8 *)dest = *(const u8 *)src;
    expandSomSpace(nfa, getSomInfo(dest), (const u8 *)src + 1, offset);
    return 0;
}

char nfaExecGough16_queueCompressState(const struct NFA *nfa,
                                       const struct mq *q, s64a loc) {
    void *dest = q->streamState;
    const void *src = q->state;

    assert(ISALIGNED_N(src, 2));
    unaligned_store_u16(dest, *(const u16 *)(src));
    compSomSpace(nfa, (u8 *)dest + 2, getSomInfoConst(src), q->offset + loc);
    return 0;
}

char nfaExecGough16_expandState(const struct NFA *nfa, void *dest,
                               const void *src, u64a offset, UNUSED u8 key) {
    assert(ISALIGNED_N(dest, 2));
    *(u16 *)dest = unaligned_load_u16(src);
    expandSomSpace(nfa, getSomInfo(dest), (const u8 *)src + 2, offset);
    return 0;
}
