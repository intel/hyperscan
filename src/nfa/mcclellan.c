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

#include "mcclellan.h"

#include "accel.h"
#include "mcclellan_internal.h"
#include "nfa_api.h"
#include "nfa_api_queue.h"
#include "nfa_internal.h"
#include "util/bitutils.h"
#include "util/compare.h"
#include "util/simd_utils.h"
#include "ue2common.h"

#include "mcclellan_common_impl.h"

static really_inline
char doComplexReport(NfaCallback cb, void *ctxt, const struct mcclellan *m,
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

static really_inline
const u8 *run_mcclellan_accel(const struct mcclellan *m,
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
u32 doNormal16(const struct mcclellan *m, const u8 **c_inout, const u8 *end,
               u32 s, char do_accel, enum MatchMode mode) {
    const u8 *c = *c_inout;

    const u16 *succ_table
        = (const u16 *)((const char *)m + sizeof(struct mcclellan));
    assert(ISALIGNED_N(succ_table, 2));
    u32 sherman_base = m->sherman_limit;
    const char *sherman_base_offset
        = (const char *)m - sizeof(struct NFA) + m->sherman_offset;
    u32 as = m->alphaShift;

    s &= STATE_MASK;

    while (c < end && s) {
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
u32 doNormalWide16(const struct mcclellan *m, const u8 **c_inout,
                   const u8 *end, u32 s, char *qstate, u16 *offset,
                   char do_accel, enum MatchMode mode) {
    const u8 *c = *c_inout;

    u32 wide_limit = m->wide_limit;
    const char *wide_base
        = (const char *)m - sizeof(struct NFA) + m->wide_offset;

    const u16 *succ_table
        = (const u16 *)((const char *)m + sizeof(struct mcclellan));
    assert(ISALIGNED_N(succ_table, 2));
    u32 sherman_base = m->sherman_limit;
    const char *sherman_base_offset
        = (const char *)m - sizeof(struct NFA) + m->sherman_offset;
    u32 as = m->alphaShift;

    s &= STATE_MASK;

    while (c < end && s) {
        u8 cprime = m->remap[*c];
        DEBUG_PRINTF("c: %02hhx '%c' cp:%02hhx (s=%u) &c: %p\n", *c,
                     ourisprint(*c) ? *c : '?', cprime, s, c);

        if (unlikely(s >= wide_limit)) {
            const char *wide_entry
                = findWideEntry16(m, wide_base, wide_limit, s);
            DEBUG_PRINTF("doing wide head (%u)\n", s);
            s = doWide16(wide_entry, &c, end, m->remap, (u16 *)&s, qstate,
                         offset);
        } else if (s >= sherman_base) {
            const char *sherman_state
                = findShermanState(m, sherman_base_offset, sherman_base, s);
            DEBUG_PRINTF("doing sherman (%u)\n", s);
            s = doSherman16(sherman_state, cprime, succ_table, as);
        } else {
            DEBUG_PRINTF("doing normal\n");
            s = succ_table[(s << as) + cprime];
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
char mcclellanExec16_i(const struct mcclellan *m, u32 *state, char *qstate,
                       const u8 *buf, size_t len, u64a offAdj, NfaCallback cb,
                       void *ctxt, char single, const u8 **c_final,
                       enum MatchMode mode) {
    assert(ISALIGNED_N(state, 2));
    if (!len) {
        if (mode == STOP_AT_MATCH) {
            *c_final = buf;
        }
        return MO_ALIVE;
    }

    u32 s = *state;
    u16 offset = 0;
    const u8 *c = buf;
    const u8 *c_end = buf + len;
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
        if (!s) {
            goto exit;
        }

        if (unlikely(m->has_wide)) {
            s = doNormalWide16(m, &c, min_accel_offset, s, qstate, &offset, 0,
                               mode);
        } else {
            s = doNormal16(m, &c, min_accel_offset, s, 0, mode);
        }

        if (mode != NO_MATCHES && (s & ACCEPT_FLAG)) {
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

        assert(c <= min_accel_offset);
    } while (c < min_accel_offset);

    s &= STATE_MASK;

    if (c == c_end) {
        goto exit;
    } else {
        goto with_accel;
    }

with_accel:
    do {
        assert(c < c_end);
        if (!s) {
            goto exit;
        }

        if (s & ACCEL_FLAG) {
            DEBUG_PRINTF("skipping\n");
            s &= STATE_MASK;
            c = run_mcclellan_accel(m, aux, s, &min_accel_offset, c, c_end);
            if (c == c_end) {
                goto exit;
            } else {
                goto without_accel;
            }
        }

        if (unlikely(m->has_wide)) {
            s = doNormalWide16(m, &c, c_end, s, qstate, &offset, 1, mode);
        } else {
            s = doNormal16(m, &c, c_end, s, 1, mode);
        }

        if (mode != NO_MATCHES && (s & ACCEPT_FLAG)) {
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
char mcclellanExec16_i_cb(const struct mcclellan *m, u32 *state, char *qstate,
                          const u8 *buf, size_t len, u64a offAdj,
                          NfaCallback cb, void *ctxt, char single,
                          const u8 **final_point) {
    return mcclellanExec16_i(m, state, qstate, buf, len, offAdj, cb, ctxt,
                             single, final_point, CALLBACK_OUTPUT);
}

static never_inline
char mcclellanExec16_i_sam(const struct mcclellan *m, u32 *state, char *qstate,
                           const u8 *buf, size_t len, u64a offAdj,
                           NfaCallback cb, void *ctxt, char single,
                           const u8 **final_point) {
    return mcclellanExec16_i(m, state, qstate, buf, len, offAdj, cb, ctxt,
                             single, final_point, STOP_AT_MATCH);
}

static never_inline
char mcclellanExec16_i_nm(const struct mcclellan *m, u32 *state, char *qstate,
                          const u8 *buf, size_t len, u64a offAdj,
                          NfaCallback cb, void *ctxt, char single,
                          const u8 **final_point) {
    return mcclellanExec16_i(m, state, qstate, buf, len, offAdj, cb, ctxt,
                             single, final_point, NO_MATCHES);
}

static really_inline
char mcclellanExec16_i_ni(const struct mcclellan *m, u32 *state, char *qstate,
                          const u8 *buf, size_t len, u64a offAdj,
                          NfaCallback cb, void *ctxt, char single,
                          const u8 **final_point, enum MatchMode mode) {
    if (mode == CALLBACK_OUTPUT) {
        return mcclellanExec16_i_cb(m, state, qstate, buf, len, offAdj, cb,
                                    ctxt, single, final_point);
    } else if (mode == STOP_AT_MATCH) {
        return mcclellanExec16_i_sam(m, state, qstate, buf, len, offAdj, cb,
                                     ctxt, single, final_point);
    } else {
        assert(mode == NO_MATCHES);
        return mcclellanExec16_i_nm(m, state, qstate, buf, len, offAdj, cb,
                                    ctxt, single, final_point);
    }
}

static really_inline
u32 doNormal8(const struct mcclellan *m, const u8 **c_inout, const u8 *end,
              u32 s, char do_accel, enum MatchMode mode) {
    const u8 *c = *c_inout;
    u32 accel_limit = m->accel_limit_8;
    u32 accept_limit = m->accept_limit_8;

    const u32 as = m->alphaShift;
    const u8 *succ_table = (const u8 *)((const char *)m
                                        + sizeof(struct mcclellan));
    while (c < end && s) {
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
char mcclellanExec8_i(const struct mcclellan *m, u32 *state, const u8 *buf,
                      size_t len, u64a offAdj, NfaCallback cb, void *ctxt,
                      char single, const u8 **c_final, enum MatchMode mode) {
    if (!len) {
        if (mode == STOP_AT_MATCH) {
            *c_final = buf;
        }
        return MO_ALIVE;
    }
    u32 s = *state;
    const u8 *c = buf;
    const u8 *c_end = buf + len;

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
        }

        s = doNormal8(m, &c, min_accel_offset, s, 0, mode);

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

        assert(c <= min_accel_offset);
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
        }

        if (s >= accel_limit && aux[s].accel_offset) {
            c = run_mcclellan_accel(m, aux, s, &min_accel_offset, c, c_end);
            if (c == c_end) {
                goto exit;
            } else {
                goto without_accel;
            }
        }
        s = doNormal8(m, &c, c_end, s, 1, mode);

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
char mcclellanExec8_i_cb(const struct mcclellan *m, u32 *state, const u8 *buf,
                         size_t len, u64a offAdj, NfaCallback cb, void *ctxt,
                         char single, const u8 **final_point) {
    return mcclellanExec8_i(m, state, buf, len, offAdj, cb, ctxt, single,
                          final_point, CALLBACK_OUTPUT);
}

static never_inline
char mcclellanExec8_i_sam(const struct mcclellan *m, u32 *state, const u8 *buf,
                          size_t len, u64a offAdj, NfaCallback cb, void *ctxt,
                          char single, const u8 **final_point) {
    return mcclellanExec8_i(m, state, buf, len, offAdj, cb, ctxt, single,
                          final_point, STOP_AT_MATCH);
}

static never_inline
char mcclellanExec8_i_nm(const struct mcclellan *m, u32 *state, const u8 *buf,
                         size_t len, u64a offAdj, NfaCallback cb, void *ctxt,
                         char single, const u8 **final_point) {
    return mcclellanExec8_i(m, state, buf, len, offAdj, cb, ctxt, single,
                          final_point, NO_MATCHES);
}

static really_inline
char mcclellanExec8_i_ni(const struct mcclellan *m, u32 *state, const u8 *buf,
                         size_t len, u64a offAdj, NfaCallback cb, void *ctxt,
                         char single, const u8 **final_point,
                         enum MatchMode mode) {
    if (mode == CALLBACK_OUTPUT) {
        return mcclellanExec8_i_cb(m, state, buf, len, offAdj, cb, ctxt, single,
                                   final_point);
    } else if (mode == STOP_AT_MATCH) {
        return mcclellanExec8_i_sam(m, state, buf, len, offAdj, cb, ctxt,
                                    single, final_point);
    } else {
        assert(mode == NO_MATCHES);
        return mcclellanExec8_i_nm(m, state, buf, len, offAdj, cb, ctxt, single,
                                   final_point);
    }
}

static really_inline
char mcclellanCheckEOD(const struct NFA *nfa, u32 s, u64a offset,
                       NfaCallback cb, void *ctxt) {
    const struct mcclellan *m = getImplNfa(nfa);
    const struct mstate_aux *aux = get_aux(m, s);

    if (m->has_wide == 1 && s >= m->wide_limit) {
        return MO_CONTINUE_MATCHING;
    }

    if (!aux->accept_eod) {
        return MO_CONTINUE_MATCHING;
    }
    return doComplexReport(cb, ctxt, m, s, offset, 1, NULL, NULL);
}

static really_inline
char nfaExecMcClellan16_Q2i(const struct NFA *n, u64a offset, const u8 *buffer,
                            const u8 *hend, NfaCallback cb, void *context,
                            struct mq *q, char single, s64a end,
                            enum MatchMode mode) {
    assert(n->type == MCCLELLAN_NFA_16);
    const struct mcclellan *m = getImplNfa(n);
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
        char rv = mcclellanExec16_i_ni(m, &s, q->state, cur_buf + sp,
                                       local_ep - sp, offset + sp, cb, context,
                                       single, &final_look, mode);
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
            s = mcclellanEnableStarts(m, s);
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
char nfaExecMcClellan16_Bi(const struct NFA *n, u64a offset, const u8 *buffer,
                           size_t length, NfaCallback cb, void *context,
                           char single) {
    assert(n->type == MCCLELLAN_NFA_16);
    const struct mcclellan *m = getImplNfa(n);
    u32 s = m->start_anchored;

    if (mcclellanExec16_i(m, &s, NULL, buffer, length, offset, cb, context,
                          single, NULL, CALLBACK_OUTPUT)
        == MO_DEAD) {
        return s ? MO_ALIVE : MO_DEAD;
    }

    if (m->has_wide == 1 && s >= m->wide_limit) {
        return MO_ALIVE;
    }

    const struct mstate_aux *aux = get_aux(m, s);

    if (aux->accept_eod) {
        doComplexReport(cb, context, m, s, offset + length, 1, NULL, NULL);
    }

    return MO_ALIVE;
}

static really_inline
char nfaExecMcClellan8_Q2i(const struct NFA *n, u64a offset, const u8 *buffer,
                           const u8 *hend, NfaCallback cb, void *context,
                           struct mq *q, char single, s64a end,
                           enum MatchMode mode) {
    assert(n->type == MCCLELLAN_NFA_8);
    const struct mcclellan *m = getImplNfa(n);
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
        char rv = mcclellanExec8_i_ni(m, &s, cur_buf + sp, local_ep - sp,
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
            s = mcclellanEnableStarts(m, s);
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

static really_inline
char nfaExecMcClellan8_Bi(const struct NFA *n, u64a offset, const u8 *buffer,
                          size_t length, NfaCallback cb, void *context,
                          char single) {
    assert(n->type == MCCLELLAN_NFA_8);
    const struct mcclellan *m = getImplNfa(n);
    u32 s = m->start_anchored;

    if (mcclellanExec8_i(m, &s, buffer, length, offset, cb, context, single,
                         NULL, CALLBACK_OUTPUT)
        == MO_DEAD) {
        return MO_DEAD;
    }

    const struct mstate_aux *aux = get_aux(m, s);

    if (aux->accept_eod) {
        doComplexReport(cb, context, m, s, offset + length, 1, NULL, NULL);
    }

    return s ? MO_ALIVE : MO_DEAD;
}

char nfaExecMcClellan8_B(const struct NFA *n, u64a offset, const u8 *buffer,
                         size_t length, NfaCallback cb, void *context) {
    assert(n->type == MCCLELLAN_NFA_8);
    const struct mcclellan *m = getImplNfa(n);

    if (m->flags & MCCLELLAN_FLAG_SINGLE) {
        return nfaExecMcClellan8_Bi(n, offset, buffer, length, cb, context, 1);
    } else {
        return nfaExecMcClellan8_Bi(n, offset, buffer, length, cb, context, 0);
    }
}

char nfaExecMcClellan8_Q(const struct NFA *n, struct mq *q, s64a end) {
    u64a offset = q->offset;
    const u8 *buffer = q->buffer;
    NfaCallback cb = q->cb;
    void *context = q->context;
    assert(n->type == MCCLELLAN_NFA_8);
    const struct mcclellan *m = getImplNfa(n);
    const u8 *hend = q->history + q->hlength;

    return nfaExecMcClellan8_Q2i(n, offset, buffer, hend, cb, context, q,
                                 m->flags & MCCLELLAN_FLAG_SINGLE, end,
                                 CALLBACK_OUTPUT);
}

char nfaExecMcClellan16_B(const struct NFA *n, u64a offset, const u8 *buffer,
                          size_t length, NfaCallback cb, void *context) {
    assert(n->type == MCCLELLAN_NFA_16);
    const struct mcclellan *m = getImplNfa(n);

    if (m->flags & MCCLELLAN_FLAG_SINGLE) {
        return nfaExecMcClellan16_Bi(n, offset, buffer, length, cb, context, 1);
    } else {
        return nfaExecMcClellan16_Bi(n, offset, buffer, length, cb, context, 0);
    }
}

char nfaExecMcClellan16_Q(const struct NFA *n, struct mq *q, s64a end) {
    u64a offset = q->offset;
    const u8 *buffer = q->buffer;
    NfaCallback cb = q->cb;
    void *context = q->context;
    assert(n->type == MCCLELLAN_NFA_16);
    const struct mcclellan *m = getImplNfa(n);
    const u8 *hend = q->history + q->hlength;

    return nfaExecMcClellan16_Q2i(n, offset, buffer, hend, cb, context, q,
                                  m->flags & MCCLELLAN_FLAG_SINGLE, end,
                                  CALLBACK_OUTPUT);
}

char nfaExecMcClellan8_reportCurrent(const struct NFA *n, struct mq *q) {
    const struct mcclellan *m = getImplNfa(n);
    NfaCallback cb = q->cb;
    void *ctxt = q->context;
    u32 s = *(u8 *)q->state;
    u8 single = m->flags & MCCLELLAN_FLAG_SINGLE;
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

char nfaExecMcClellan16_reportCurrent(const struct NFA *n, struct mq *q) {
    const struct mcclellan *m = getImplNfa(n);
    NfaCallback cb = q->cb;
    void *ctxt = q->context;
    u32 s = *(u16 *)q->state;
    const struct mstate_aux *aux = get_aux(m, s);
    u8 single = m->flags & MCCLELLAN_FLAG_SINGLE;
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
char mcclellanHasAccept(const struct mcclellan *m, const struct mstate_aux *aux,
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

char nfaExecMcClellan8_inAccept(const struct NFA *n, ReportID report,
                                struct mq *q) {
    assert(n && q);

    const struct mcclellan *m = getImplNfa(n);
    u8 s = *(u8 *)q->state;
    DEBUG_PRINTF("checking accepts for %hhu\n", s);
    if (s < m->accept_limit_8) {
        return 0;
    }

    return mcclellanHasAccept(m, get_aux(m, s), report);
}

char nfaExecMcClellan8_inAnyAccept(const struct NFA *n, struct mq *q) {
    assert(n && q);

    const struct mcclellan *m = getImplNfa(n);
    u8 s = *(u8 *)q->state;
    DEBUG_PRINTF("checking accepts for %hhu\n", s);
    assert(s < m->accept_limit_8 || get_aux(m, s)->accept);

    return s >= m->accept_limit_8;
}

char nfaExecMcClellan16_inAccept(const struct NFA *n, ReportID report,
                                 struct mq *q) {
    assert(n && q);

    const struct mcclellan *m = getImplNfa(n);
    u16 s = *(u16 *)q->state;
    DEBUG_PRINTF("checking accepts for %hu\n", s);

    return (m->has_wide == 1 && s >= m->wide_limit) ?
                0 : mcclellanHasAccept(m, get_aux(m, s), report);
}

char nfaExecMcClellan16_inAnyAccept(const struct NFA *n, struct mq *q) {
    assert(n && q);

    const struct mcclellan *m = getImplNfa(n);
    u16 s = *(u16 *)q->state;
    DEBUG_PRINTF("checking accepts for %hu\n", s);

    return (m->has_wide == 1 && s >= m->wide_limit) ?
                0 : !!get_aux(m, s)->accept;
}

char nfaExecMcClellan8_Q2(const struct NFA *n, struct mq *q, s64a end) {
    u64a offset = q->offset;
    const u8 *buffer = q->buffer;
    NfaCallback cb = q->cb;
    void *context = q->context;
    assert(n->type == MCCLELLAN_NFA_8);
    const struct mcclellan *m = getImplNfa(n);
    const u8 *hend = q->history + q->hlength;

    return nfaExecMcClellan8_Q2i(n, offset, buffer, hend, cb, context, q,
                                 m->flags & MCCLELLAN_FLAG_SINGLE, end,
                                 STOP_AT_MATCH);
}

char nfaExecMcClellan16_Q2(const struct NFA *n, struct mq *q, s64a end) {
    u64a offset = q->offset;
    const u8 *buffer = q->buffer;
    NfaCallback cb = q->cb;
    void *context = q->context;
    assert(n->type == MCCLELLAN_NFA_16);
    const struct mcclellan *m = getImplNfa(n);
    const u8 *hend = q->history + q->hlength;

    return nfaExecMcClellan16_Q2i(n, offset, buffer, hend, cb, context, q,
                                  m->flags & MCCLELLAN_FLAG_SINGLE, end,
                                  STOP_AT_MATCH);
}

char nfaExecMcClellan8_QR(const struct NFA *n, struct mq *q, ReportID report) {
    u64a offset = q->offset;
    const u8 *buffer = q->buffer;
    NfaCallback cb = q->cb;
    void *context = q->context;
    assert(n->type == MCCLELLAN_NFA_8);
    const struct mcclellan *m = getImplNfa(n);
    const u8 *hend = q->history + q->hlength;

    char rv = nfaExecMcClellan8_Q2i(n, offset, buffer, hend, cb, context, q,
                                m->flags & MCCLELLAN_FLAG_SINGLE, 0 /* end */,
                                NO_MATCHES);
    if (rv && nfaExecMcClellan8_inAccept(n, report, q)) {
        return MO_MATCHES_PENDING;
    } else {
        return rv;
    }
}

char nfaExecMcClellan16_QR(const struct NFA *n, struct mq *q, ReportID report) {
    u64a offset = q->offset;
    const u8 *buffer = q->buffer;
    NfaCallback cb = q->cb;
    void *context = q->context;
    assert(n->type == MCCLELLAN_NFA_16);
    const struct mcclellan *m = getImplNfa(n);
    const u8 *hend = q->history + q->hlength;

    char rv = nfaExecMcClellan16_Q2i(n, offset, buffer, hend, cb, context, q,
                                     m->flags & MCCLELLAN_FLAG_SINGLE,
                                     0 /* end */, NO_MATCHES);

    if (rv && nfaExecMcClellan16_inAccept(n, report, q)) {
        return MO_MATCHES_PENDING;
    } else {
        return rv;
    }
}

char nfaExecMcClellan8_initCompressedState(const struct NFA *nfa, u64a offset,
                                           void *state, UNUSED u8 key) {
    const struct mcclellan *m = getImplNfa(nfa);
    u8 s = offset ? m->start_floating : m->start_anchored;
    if (s) {
        *(u8 *)state = s;
        return 1;
    }
    return 0;
}

char nfaExecMcClellan16_initCompressedState(const struct NFA *nfa, u64a offset,
                                            void *state, UNUSED u8 key) {
    const struct mcclellan *m = getImplNfa(nfa);
    u16 s = offset ? m->start_floating : m->start_anchored;

    // new byte
    if (m->has_wide) {
        unaligned_store_u16((u16 *)state + 1, 0);
    }

    if (s) {
        unaligned_store_u16(state, s);
        return 1;
    }
    return 0;
}

void nfaExecMcClellan8_SimpStream(const struct NFA *nfa, char *state,
                                  const u8 *buf, char top, size_t start_off,
                                  size_t len, NfaCallback cb, void *ctxt) {
    const struct mcclellan *m = getImplNfa(nfa);

    u32 s = top ? m->start_anchored : *(u8 *)state;

    if (m->flags & MCCLELLAN_FLAG_SINGLE) {
        mcclellanExec8_i(m, &s, buf + start_off, len - start_off,
                         start_off, cb, ctxt, 1, NULL, CALLBACK_OUTPUT);
    } else {
        mcclellanExec8_i(m, &s, buf + start_off, len - start_off,
                         start_off, cb, ctxt, 0, NULL, CALLBACK_OUTPUT);
    }

    *(u8 *)state = s;
}

void nfaExecMcClellan16_SimpStream(const struct NFA *nfa, char *state,
                                   const u8 *buf, char top, size_t start_off,
                                   size_t len, NfaCallback cb, void *ctxt) {
    const struct mcclellan *m = getImplNfa(nfa);
    u32 s;

    if (top) {
        s = m->start_anchored;

        // new byte
        if (m->has_wide) {
            unaligned_store_u16((u16 *)state + 1, 0);
        }
    } else {
        s = unaligned_load_u16(state);
    }

    if (m->flags & MCCLELLAN_FLAG_SINGLE) {
        mcclellanExec16_i(m, &s, state, buf + start_off, len - start_off,
                          start_off, cb, ctxt, 1, NULL, CALLBACK_OUTPUT);
    } else {
        mcclellanExec16_i(m, &s, state, buf + start_off, len - start_off,
                          start_off, cb, ctxt, 0, NULL, CALLBACK_OUTPUT);
    }

    unaligned_store_u16(state, s);
}

char nfaExecMcClellan8_testEOD(const struct NFA *nfa, const char *state,
                               UNUSED const char *streamState, u64a offset,
                               NfaCallback callback, void *context) {
    return mcclellanCheckEOD(nfa, *(const u8 *)state, offset, callback,
                             context);
}

char nfaExecMcClellan16_testEOD(const struct NFA *nfa, const char *state,
                                UNUSED const char *streamState, u64a offset,
                                NfaCallback callback, void *context) {
    assert(ISALIGNED_N(state, 2));
    return mcclellanCheckEOD(nfa, *(const u16 *)state, offset, callback,
                             context);
}

char nfaExecMcClellan8_queueInitState(UNUSED const struct NFA *nfa,
                                      struct mq *q) {
    assert(nfa->scratchStateSize == 1);
    *(u8 *)q->state = 0;
    return 0;
}

char nfaExecMcClellan16_queueInitState(UNUSED const struct NFA *nfa,
                                       struct mq *q) {
    const struct mcclellan *m = getImplNfa(nfa);
    assert(m->has_wide == 1 ? nfa->scratchStateSize == 4
                            : nfa->scratchStateSize == 2);
    assert(ISALIGNED_N(q->state, 2));
    *(u16 *)q->state = 0;

    // new byte
    if (m->has_wide) {
        unaligned_store_u16((u16 *)q->state + 1, 0);
    }
    return 0;
}

char nfaExecMcClellan8_queueCompressState(UNUSED const struct NFA *nfa,
                                          const struct mq *q, UNUSED s64a loc) {
    void *dest = q->streamState;
    const void *src = q->state;
    assert(nfa->scratchStateSize == 1);
    assert(nfa->streamStateSize == 1);
    *(u8 *)dest = *(const u8 *)src;
    return 0;
}

char nfaExecMcClellan8_expandState(UNUSED const struct NFA *nfa, void *dest,
                                   const void *src, UNUSED u64a offset,
                                   UNUSED u8 key) {
    assert(nfa->scratchStateSize == 1);
    assert(nfa->streamStateSize == 1);
    *(u8 *)dest = *(const u8 *)src;
    return 0;
}

char nfaExecMcClellan16_queueCompressState(UNUSED const struct NFA *nfa,
                                           const struct mq *q,
                                           UNUSED s64a loc) {
    const struct mcclellan *m = getImplNfa(nfa);
    void *dest = q->streamState;
    const void *src = q->state;
    assert(m->has_wide == 1 ? nfa->scratchStateSize == 4
                            : nfa->scratchStateSize == 2);
    assert(m->has_wide == 1 ? nfa->streamStateSize == 4
                            : nfa->streamStateSize == 2);

    assert(ISALIGNED_N(src, 2));
    unaligned_store_u16(dest, *(const u16 *)(src));

    // new byte
    if (m->has_wide) {
        unaligned_store_u16((u16 *)dest + 1, *((const u16 *)src + 1));
    }
    return 0;
}

char nfaExecMcClellan16_expandState(UNUSED const struct NFA *nfa, void *dest,
                                    const void *src, UNUSED u64a offset,
                                    UNUSED u8 key) {
    const struct mcclellan *m = getImplNfa(nfa);
    assert(m->has_wide == 1 ? nfa->scratchStateSize == 4
                            : nfa->scratchStateSize == 2);
    assert(m->has_wide == 1 ? nfa->streamStateSize == 4
                            : nfa->streamStateSize == 2);

    assert(ISALIGNED_N(dest, 2));
    *(u16 *)dest = unaligned_load_u16(src);

    // new byte
    if (m->has_wide) {
        *((u16 *)dest + 1) =  unaligned_load_u16((const u16 *)src + 1);
    }
    return 0;
}
