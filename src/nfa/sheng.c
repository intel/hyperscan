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

#include "sheng.h"

#include "accel.h"
#include "sheng_internal.h"
#include "nfa_api.h"
#include "nfa_api_queue.h"
#include "nfa_internal.h"
#include "util/bitutils.h"
#include "util/compare.h"
#include "util/join.h"
#include "util/simd_utils.h"

enum MatchMode {
    CALLBACK_OUTPUT,
    STOP_AT_MATCH,
    NO_MATCHES
};

static really_inline
const struct sheng *get_sheng(const struct NFA *n) {
    return (const struct sheng *)getImplNfa(n);
}

static really_inline
const struct sstate_aux *get_aux(const struct sheng *sh, u8 id) {
    u32 offset = sh->aux_offset - sizeof(struct NFA) +
            (id & SHENG_STATE_MASK) * sizeof(struct sstate_aux);
    DEBUG_PRINTF("Getting aux for state %u at offset %llu\n",
                 id & SHENG_STATE_MASK, (u64a)offset + sizeof(struct NFA));
    return (const struct sstate_aux *)((const char *) sh + offset);
}

static really_inline
const union AccelAux *get_accel(const struct sheng *sh, u8 id) {
    const struct sstate_aux *saux = get_aux(sh, id);
    DEBUG_PRINTF("Getting accel aux at offset %u\n", saux->accel);
    const union AccelAux *aux = (const union AccelAux *)
            ((const char *)sh + saux->accel - sizeof(struct NFA));
    return aux;
}

static really_inline
const struct report_list *get_rl(const struct sheng *sh,
                                 const struct sstate_aux *aux) {
    DEBUG_PRINTF("Getting report list at offset %u\n", aux->accept);
    return (const struct report_list *)
        ((const char *)sh + aux->accept - sizeof(struct NFA));
}

static really_inline
const struct report_list *get_eod_rl(const struct sheng *sh,
                                     const struct sstate_aux *aux) {
    DEBUG_PRINTF("Getting EOD report list at offset %u\n", aux->accept);
    return (const struct report_list *)
        ((const char *)sh + aux->accept_eod - sizeof(struct NFA));
}

static really_inline
char shengHasAccept(const struct sheng *sh, const struct sstate_aux *aux,
                    ReportID report) {
    assert(sh && aux);

    const struct report_list *rl = get_rl(sh, aux);
    assert(ISALIGNED_N(rl, 4));

    DEBUG_PRINTF("report list has %u entries\n", rl->count);

    for (u32 i = 0; i < rl->count; i++) {
        if (rl->report[i] == report) {
            DEBUG_PRINTF("reporting %u\n", rl->report[i]);
            return 1;
        }
    }

    return 0;
}

static really_inline
char fireSingleReport(NfaCallback cb, void *ctxt, ReportID r, u64a loc) {
    DEBUG_PRINTF("reporting %u\n", r);
    if (cb(0, loc, r, ctxt) == MO_HALT_MATCHING) {
        return MO_HALT_MATCHING; /* termination requested */
    }
    return MO_CONTINUE_MATCHING; /* continue execution */
}

static really_inline
char fireReports(const struct sheng *sh, NfaCallback cb, void *ctxt,
                 const u8 state, u64a loc, u8 *const cached_accept_state,
                 ReportID *const cached_accept_id, char eod) {
    DEBUG_PRINTF("reporting matches @ %llu\n", loc);

    if (!eod && state == *cached_accept_state) {
        DEBUG_PRINTF("reporting %u\n", *cached_accept_id);
        if (cb(0, loc, *cached_accept_id, ctxt) == MO_HALT_MATCHING) {
            return MO_HALT_MATCHING; /* termination requested */
        }

        return MO_CONTINUE_MATCHING; /* continue execution */
    }
    const struct sstate_aux *aux = get_aux(sh, state);
    const struct report_list *rl = eod ? get_eod_rl(sh, aux) : get_rl(sh, aux);
    assert(ISALIGNED(rl));

    DEBUG_PRINTF("report list has %u entries\n", rl->count);
    u32 count = rl->count;

    if (!eod && count == 1) {
        *cached_accept_state = state;
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

#if defined(HAVE_AVX512VBMI)
// Sheng32
static really_inline
const struct sheng32 *get_sheng32(const struct NFA *n) {
    return (const struct sheng32 *)getImplNfa(n);
}

static really_inline
const struct sstate_aux *get_aux32(const struct sheng32 *sh, u8 id) {
    u32 offset = sh->aux_offset - sizeof(struct NFA) +
            (id & SHENG32_STATE_MASK) * sizeof(struct sstate_aux);
    DEBUG_PRINTF("Getting aux for state %u at offset %llu\n",
                 id & SHENG32_STATE_MASK, (u64a)offset + sizeof(struct NFA));
    return (const struct sstate_aux *)((const char *) sh + offset);
}

static really_inline
const union AccelAux *get_accel32(const struct sheng32 *sh, u8 id) {
    const struct sstate_aux *saux = get_aux32(sh, id);
    DEBUG_PRINTF("Getting accel aux at offset %u\n", saux->accel);
    const union AccelAux *aux = (const union AccelAux *)
            ((const char *)sh + saux->accel - sizeof(struct NFA));
    return aux;
}

static really_inline
const struct report_list *get_rl32(const struct sheng32 *sh,
                                   const struct sstate_aux *aux) {
    DEBUG_PRINTF("Getting report list at offset %u\n", aux->accept);
    return (const struct report_list *)
        ((const char *)sh + aux->accept - sizeof(struct NFA));
}

static really_inline
const struct report_list *get_eod_rl32(const struct sheng32 *sh,
                                       const struct sstate_aux *aux) {
    DEBUG_PRINTF("Getting EOD report list at offset %u\n", aux->accept);
    return (const struct report_list *)
        ((const char *)sh + aux->accept_eod - sizeof(struct NFA));
}

static really_inline
char sheng32HasAccept(const struct sheng32 *sh, const struct sstate_aux *aux,
                      ReportID report) {
    assert(sh && aux);

    const struct report_list *rl = get_rl32(sh, aux);
    assert(ISALIGNED_N(rl, 4));

    DEBUG_PRINTF("report list has %u entries\n", rl->count);

    for (u32 i = 0; i < rl->count; i++) {
        if (rl->report[i] == report) {
            DEBUG_PRINTF("reporting %u\n", rl->report[i]);
            return 1;
        }
    }

    return 0;
}

static really_inline
char fireReports32(const struct sheng32 *sh, NfaCallback cb, void *ctxt,
                   const u8 state, u64a loc, u8 *const cached_accept_state,
                   ReportID *const cached_accept_id, char eod) {
    DEBUG_PRINTF("reporting matches @ %llu\n", loc);

    if (!eod && state == *cached_accept_state) {
        DEBUG_PRINTF("reporting %u\n", *cached_accept_id);
        if (cb(0, loc, *cached_accept_id, ctxt) == MO_HALT_MATCHING) {
            return MO_HALT_MATCHING; /* termination requested */
        }

        return MO_CONTINUE_MATCHING; /* continue execution */
    }
    const struct sstate_aux *aux = get_aux32(sh, state);
    const struct report_list *rl = eod ? get_eod_rl32(sh, aux) :
                                         get_rl32(sh, aux);
    assert(ISALIGNED(rl));

    DEBUG_PRINTF("report list has %u entries\n", rl->count);
    u32 count = rl->count;

    if (!eod && count == 1) {
        *cached_accept_state = state;
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

// Sheng64
static really_inline
const struct sheng64 *get_sheng64(const struct NFA *n) {
    return (const struct sheng64 *)getImplNfa(n);
}

static really_inline
const struct sstate_aux *get_aux64(const struct sheng64 *sh, u8 id) {
    u32 offset = sh->aux_offset - sizeof(struct NFA) +
            (id & SHENG64_STATE_MASK) * sizeof(struct sstate_aux);
    DEBUG_PRINTF("Getting aux for state %u at offset %llu\n",
                 id & SHENG64_STATE_MASK, (u64a)offset + sizeof(struct NFA));
    return (const struct sstate_aux *)((const char *) sh + offset);
}

static really_inline
const struct report_list *get_rl64(const struct sheng64 *sh,
                                   const struct sstate_aux *aux) {
    DEBUG_PRINTF("Getting report list at offset %u\n", aux->accept);
    return (const struct report_list *)
        ((const char *)sh + aux->accept - sizeof(struct NFA));
}

static really_inline
const struct report_list *get_eod_rl64(const struct sheng64 *sh,
                                       const struct sstate_aux *aux) {
    DEBUG_PRINTF("Getting EOD report list at offset %u\n", aux->accept);
    return (const struct report_list *)
        ((const char *)sh + aux->accept_eod - sizeof(struct NFA));
}

static really_inline
char sheng64HasAccept(const struct sheng64 *sh, const struct sstate_aux *aux,
                      ReportID report) {
    assert(sh && aux);

    const struct report_list *rl = get_rl64(sh, aux);
    assert(ISALIGNED_N(rl, 4));

    DEBUG_PRINTF("report list has %u entries\n", rl->count);

    for (u32 i = 0; i < rl->count; i++) {
        if (rl->report[i] == report) {
            DEBUG_PRINTF("reporting %u\n", rl->report[i]);
            return 1;
        }
    }

    return 0;
}

static really_inline
char fireReports64(const struct sheng64 *sh, NfaCallback cb, void *ctxt,
                   const u8 state, u64a loc, u8 *const cached_accept_state,
                   ReportID *const cached_accept_id, char eod) {
    DEBUG_PRINTF("reporting matches @ %llu\n", loc);

    if (!eod && state == *cached_accept_state) {
        DEBUG_PRINTF("reporting %u\n", *cached_accept_id);
        if (cb(0, loc, *cached_accept_id, ctxt) == MO_HALT_MATCHING) {
            return MO_HALT_MATCHING; /* termination requested */
        }

        return MO_CONTINUE_MATCHING; /* continue execution */
    }
    const struct sstate_aux *aux = get_aux64(sh, state);
    const struct report_list *rl = eod ? get_eod_rl64(sh, aux) :
                                         get_rl64(sh, aux);
    assert(ISALIGNED(rl));

    DEBUG_PRINTF("report list has %u entries\n", rl->count);
    u32 count = rl->count;

    if (!eod && count == 1) {
        *cached_accept_state = state;
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
#endif // end of HAVE_AVX512VBMI

/* include Sheng function definitions */
#include "sheng_defs.h"

static really_inline
char runShengCb(const struct sheng *sh, NfaCallback cb, void *ctxt, u64a offset,
                u8 *const cached_accept_state, ReportID *const cached_accept_id,
                const u8 *cur_buf, const u8 *start, const u8 *end, u8 can_die,
                u8 has_accel, u8 single, const u8 **scanned, u8 *state) {
    DEBUG_PRINTF("Scanning %llu bytes (offset %llu) in callback mode\n",
                 (u64a)(end - start), offset);
    DEBUG_PRINTF("start: %lli end: %lli\n", (s64a)(start - cur_buf),
                 (s64a)(end - cur_buf));
    DEBUG_PRINTF("can die: %u has accel: %u single: %u\n", !!can_die,
                 !!has_accel, !!single);
    int rv;
    /* scan and report all matches */
    if (can_die) {
        if (has_accel) {
            rv = sheng4_coda(state, cb, ctxt, sh, cached_accept_state,
                             cached_accept_id, single, offset, cur_buf, start,
                             end, scanned);
        } else {
            rv = sheng4_cod(state, cb, ctxt, sh, cached_accept_state,
                            cached_accept_id, single, offset, cur_buf, start,
                            end, scanned);
        }
        if (rv == MO_HALT_MATCHING) {
            return MO_DEAD;
        }
        rv = sheng_cod(state, cb, ctxt, sh, cached_accept_state,
                       cached_accept_id, single, offset, cur_buf, *scanned, end,
                       scanned);
    } else {
        if (has_accel) {
            rv = sheng4_coa(state, cb, ctxt, sh, cached_accept_state,
                            cached_accept_id, single, offset, cur_buf, start,
                            end, scanned);
        } else {
            rv = sheng4_co(state, cb, ctxt, sh, cached_accept_state,
                           cached_accept_id, single, offset, cur_buf, start,
                           end, scanned);
        }
        if (rv == MO_HALT_MATCHING) {
            return MO_DEAD;
        }
        rv = sheng_co(state, cb, ctxt, sh, cached_accept_state,
                      cached_accept_id, single, offset, cur_buf, *scanned, end,
                      scanned);
    }
    if (rv == MO_HALT_MATCHING) {
        return MO_DEAD;
    }
    return MO_ALIVE;
}

static really_inline
void runShengNm(const struct sheng *sh, NfaCallback cb, void *ctxt, u64a offset,
                u8 *const cached_accept_state, ReportID *const cached_accept_id,
                const u8 *cur_buf, const u8 *start, const u8 *end, u8 can_die,
                u8 has_accel, u8 single, const u8 **scanned, u8 *state) {
    DEBUG_PRINTF("Scanning %llu bytes (offset %llu) in nomatch mode\n",
                 (u64a)(end - start), offset);
    DEBUG_PRINTF("start: %lli end: %lli\n", (s64a)(start - cur_buf),
                 (s64a)(end - cur_buf));
    DEBUG_PRINTF("can die: %u has accel: %u single: %u\n", !!can_die,
                 !!has_accel, !!single);
    /* just scan the buffer */
    if (can_die) {
        if (has_accel) {
            sheng4_nmda(state, cb, ctxt, sh, cached_accept_state,
                        cached_accept_id, single, offset, cur_buf, start, end,
                        scanned);
        } else {
            sheng4_nmd(state, cb, ctxt, sh, cached_accept_state,
                       cached_accept_id, single, offset, cur_buf, start, end,
                       scanned);
        }
        sheng_nmd(state, cb, ctxt, sh, cached_accept_state, cached_accept_id,
                  single, offset, cur_buf, *scanned, end, scanned);
    } else {
        sheng4_nm(state, cb, ctxt, sh, cached_accept_state, cached_accept_id,
                  single, offset, cur_buf, start, end, scanned);
        sheng_nm(state, cb, ctxt, sh, cached_accept_state, cached_accept_id,
                 single, offset, cur_buf, *scanned, end, scanned);
    }
}

static really_inline
char runShengSam(const struct sheng *sh, NfaCallback cb, void *ctxt,
                 u64a offset, u8 *const cached_accept_state,
                 ReportID *const cached_accept_id, const u8 *cur_buf,
                 const u8 *start, const u8 *end, u8 can_die, u8 has_accel,
                 u8 single, const u8 **scanned, u8 *state) {
    DEBUG_PRINTF("Scanning %llu bytes (offset %llu) in stop at match mode\n",
                 (u64a)(end - start), offset);
    DEBUG_PRINTF("start: %lli end: %lli\n", (s64a)(start - cur_buf),
                 (s64a)(end - cur_buf));
    DEBUG_PRINTF("can die: %u has accel: %u single: %u\n", !!can_die,
                 !!has_accel, !!single);
    int rv;
    /* scan until first match */
    if (can_die) {
        if (has_accel) {
            rv = sheng4_samda(state, cb, ctxt, sh, cached_accept_state,
                              cached_accept_id, single, offset, cur_buf, start,
                              end, scanned);
        } else {
            rv = sheng4_samd(state, cb, ctxt, sh, cached_accept_state,
                             cached_accept_id, single, offset, cur_buf, start,
                             end, scanned);
        }
        if (rv == MO_HALT_MATCHING) {
            return MO_DEAD;
        }
        /* if we stopped before we expected, we found a match */
        if (rv == MO_MATCHES_PENDING) {
            return MO_MATCHES_PENDING;
        }

        rv = sheng_samd(state, cb, ctxt, sh, cached_accept_state,
                        cached_accept_id, single, offset, cur_buf, *scanned,
                        end, scanned);
    } else {
        if (has_accel) {
            rv = sheng4_sama(state, cb, ctxt, sh, cached_accept_state,
                             cached_accept_id, single, offset, cur_buf, start,
                             end, scanned);
        } else {
            rv = sheng4_sam(state, cb, ctxt, sh, cached_accept_state,
                            cached_accept_id, single, offset, cur_buf, start,
                            end, scanned);
        }
        if (rv == MO_HALT_MATCHING) {
            return MO_DEAD;
        }
        /* if we stopped before we expected, we found a match */
        if (rv == MO_MATCHES_PENDING) {
            return MO_MATCHES_PENDING;
        }

        rv = sheng_sam(state, cb, ctxt, sh, cached_accept_state,
                       cached_accept_id, single, offset, cur_buf, *scanned, end,
                       scanned);
    }
    if (rv == MO_HALT_MATCHING) {
        return MO_DEAD;
    }
    /* if we stopped before we expected, we found a match */
    if (rv == MO_MATCHES_PENDING) {
        return MO_MATCHES_PENDING;
    }
    return MO_ALIVE;
}

static never_inline
char runSheng(const struct sheng *sh, struct mq *q, s64a b_end,
              enum MatchMode mode) {
    u8 state = *(u8 *)q->state;
    u8 can_die = sh->flags & SHENG_FLAG_CAN_DIE;
    u8 has_accel = sh->flags & SHENG_FLAG_HAS_ACCEL;
    u8 single = sh->flags & SHENG_FLAG_SINGLE_REPORT;

    u8 cached_accept_state = 0;
    ReportID cached_accept_id = 0;

    DEBUG_PRINTF("starting Sheng execution in state %u\n",
                 state & SHENG_STATE_MASK);

    if (q->report_current) {
        DEBUG_PRINTF("reporting current pending matches\n");
        assert(sh);

        q->report_current = 0;

        int rv;
        if (single) {
            rv = fireSingleReport(q->cb, q->context, sh->report,
                                  q_cur_offset(q));
        } else {
            rv = fireReports(sh, q->cb, q->context, state, q_cur_offset(q),
                             &cached_accept_state, &cached_accept_id, 0);
        }
        if (rv == MO_HALT_MATCHING) {
            DEBUG_PRINTF("exiting in state %u\n", state & SHENG_STATE_MASK);
            return MO_DEAD;
        }

        DEBUG_PRINTF("proceeding with matching\n");
    }

    assert(q_cur_type(q) == MQE_START);
    s64a start = q_cur_loc(q);

    DEBUG_PRINTF("offset: %lli, location: %lli, mode: %s\n", q->offset, start,
                 mode == CALLBACK_OUTPUT ? "CALLBACK OUTPUT" :
                     mode == NO_MATCHES ? "NO MATCHES" :
                         mode == STOP_AT_MATCH ? "STOP AT MATCH" : "???");

    DEBUG_PRINTF("processing event @ %lli: %s\n", q->offset + q_cur_loc(q),
                 q_cur_type(q) == MQE_START ? "START" :
                     q_cur_type(q) == MQE_TOP ? "TOP" :
                         q_cur_type(q) == MQE_END ? "END" : "???");

    const u8* cur_buf;
    if (start < 0) {
        DEBUG_PRINTF("negative location, scanning history\n");
        DEBUG_PRINTF("min location: %zd\n", -q->hlength);
        cur_buf = q->history + q->hlength;
    } else {
        DEBUG_PRINTF("positive location, scanning buffer\n");
        DEBUG_PRINTF("max location: %lli\n", b_end);
        cur_buf = q->buffer;
    }

    /* if we our queue event is past our end */
    if (mode != NO_MATCHES && q_cur_loc(q) > b_end) {
        DEBUG_PRINTF("current location past buffer end\n");
        DEBUG_PRINTF("setting q location to %llu\n", b_end);
        DEBUG_PRINTF("exiting in state %u\n", state & SHENG_STATE_MASK);
        q->items[q->cur].location = b_end;
        return MO_ALIVE;
    }

    q->cur++;

    s64a cur_start = start;

    while (1) {
        DEBUG_PRINTF("processing event @ %lli: %s\n", q->offset + q_cur_loc(q),
                     q_cur_type(q) == MQE_START ? "START" :
                             q_cur_type(q) == MQE_TOP ? "TOP" :
                                     q_cur_type(q) == MQE_END ? "END" : "???");
        s64a end = q_cur_loc(q);
        if (mode != NO_MATCHES) {
            end = MIN(end, b_end);
        }
        assert(end <= (s64a) q->length);
        s64a cur_end = end;

        /* we may cross the border between history and current buffer */
        if (cur_start < 0) {
            cur_end = MIN(0, cur_end);
        }

        DEBUG_PRINTF("start: %lli end: %lli\n", start, end);

        /* don't scan zero length buffer */
        if (cur_start != cur_end) {
            const u8 * scanned = cur_buf;
            char rv;

            if (mode == NO_MATCHES) {
                runShengNm(sh, q->cb, q->context, q->offset,
                           &cached_accept_state, &cached_accept_id, cur_buf,
                           cur_buf + cur_start, cur_buf + cur_end, can_die,
                           has_accel, single, &scanned, &state);
            } else if (mode == CALLBACK_OUTPUT) {
                rv = runShengCb(sh, q->cb, q->context, q->offset,
                                &cached_accept_state, &cached_accept_id,
                                cur_buf, cur_buf + cur_start, cur_buf + cur_end,
                                can_die, has_accel, single, &scanned, &state);
                if (rv == MO_DEAD) {
                    DEBUG_PRINTF("exiting in state %u\n",
                                 state & SHENG_STATE_MASK);
                    return MO_DEAD;
                }
            } else if (mode == STOP_AT_MATCH) {
                rv = runShengSam(sh, q->cb, q->context, q->offset,
                                 &cached_accept_state, &cached_accept_id,
                                 cur_buf, cur_buf + cur_start,
                                 cur_buf + cur_end, can_die, has_accel, single,
                                 &scanned, &state);
                if (rv == MO_DEAD) {
                    DEBUG_PRINTF("exiting in state %u\n",
                                 state & SHENG_STATE_MASK);
                    return rv;
                } else if (rv == MO_MATCHES_PENDING) {
                    assert(q->cur);
                    DEBUG_PRINTF("found a match, setting q location to %zd\n",
                                 scanned - cur_buf + 1);
                    q->cur--;
                    q->items[q->cur].type = MQE_START;
                    q->items[q->cur].location =
                            scanned - cur_buf + 1; /* due to exiting early */
                    *(u8 *)q->state = state;
                    DEBUG_PRINTF("exiting in state %u\n",
                                 state & SHENG_STATE_MASK);
                    return rv;
                }
            } else {
                assert(!"invalid scanning mode!");
            }
            assert(scanned == cur_buf + cur_end);

            cur_start = cur_end;
        }

        /* if we our queue event is past our end */
        if (mode != NO_MATCHES && q_cur_loc(q) > b_end) {
            DEBUG_PRINTF("current location past buffer end\n");
            DEBUG_PRINTF("setting q location to %llu\n", b_end);
            DEBUG_PRINTF("exiting in state %u\n", state & SHENG_STATE_MASK);
            q->cur--;
            q->items[q->cur].type = MQE_START;
            q->items[q->cur].location = b_end;
            *(u8 *)q->state = state;
            return MO_ALIVE;
        }

        /* crossing over into actual buffer */
        if (cur_start == 0) {
            DEBUG_PRINTF("positive location, scanning buffer\n");
            DEBUG_PRINTF("max offset: %lli\n", b_end);
            cur_buf = q->buffer;
        }

        /* continue scanning the same buffer */
        if (end != cur_end) {
            continue;
        }

        switch (q_cur_type(q)) {
        case MQE_END:
            *(u8 *)q->state = state;
            q->cur++;
            DEBUG_PRINTF("exiting in state %u\n", state & SHENG_STATE_MASK);
            if (can_die) {
                return (state & SHENG_STATE_DEAD) ? MO_DEAD : MO_ALIVE;
            }
            return MO_ALIVE;
        case MQE_TOP:
            if (q->offset + cur_start == 0) {
                DEBUG_PRINTF("Anchored start, going to state %u\n",
                             sh->anchored);
                state = sh->anchored;
            } else {
                u8 new_state = get_aux(sh, state)->top;
                DEBUG_PRINTF("Top event %u->%u\n", state & SHENG_STATE_MASK,
                             new_state & SHENG_STATE_MASK);
                state = new_state;
            }
            break;
        default:
            assert(!"invalid queue event");
            break;
        }
        q->cur++;
    }
}

char nfaExecSheng_B(const struct NFA *n, u64a offset, const u8 *buffer,
                    size_t length, NfaCallback cb, void *context) {
    DEBUG_PRINTF("smallwrite Sheng\n");
    assert(n->type == SHENG_NFA);
    const struct sheng *sh = getImplNfa(n);
    u8 state = sh->anchored;
    u8 can_die = sh->flags & SHENG_FLAG_CAN_DIE;
    u8 has_accel = sh->flags & SHENG_FLAG_HAS_ACCEL;
    u8 single = sh->flags & SHENG_FLAG_SINGLE_REPORT;
    u8 cached_accept_state = 0;
    ReportID cached_accept_id = 0;

    /* scan and report all matches */
    int rv;
    s64a end = length;
    const u8 *scanned;

    rv = runShengCb(sh, cb, context, offset, &cached_accept_state,
                    &cached_accept_id, buffer, buffer, buffer + end, can_die,
                    has_accel, single, &scanned, &state);
    if (rv == MO_DEAD) {
        DEBUG_PRINTF("exiting in state %u\n",
                     state & SHENG_STATE_MASK);
        return MO_DEAD;
    }

    DEBUG_PRINTF("%u\n", state & SHENG_STATE_MASK);

    const struct sstate_aux *aux = get_aux(sh, state);

    if (aux->accept_eod) {
        DEBUG_PRINTF("Reporting EOD matches\n");
        fireReports(sh, cb, context, state, end + offset, &cached_accept_state,
                    &cached_accept_id, 1);
    }

    return state & SHENG_STATE_DEAD ? MO_DEAD : MO_ALIVE;
}

char nfaExecSheng_Q(const struct NFA *n, struct mq *q, s64a end) {
    const struct sheng *sh = get_sheng(n);
    char rv = runSheng(sh, q, end, CALLBACK_OUTPUT);
    return rv;
}

char nfaExecSheng_Q2(const struct NFA *n, struct mq *q, s64a end) {
    const struct sheng *sh = get_sheng(n);
    char rv = runSheng(sh, q, end, STOP_AT_MATCH);
    return rv;
}

char nfaExecSheng_QR(const struct NFA *n, struct mq *q, ReportID report) {
    assert(q_cur_type(q) == MQE_START);

    const struct sheng *sh = get_sheng(n);
    char rv = runSheng(sh, q, 0 /* end */, NO_MATCHES);

    if (rv && nfaExecSheng_inAccept(n, report, q)) {
        return MO_MATCHES_PENDING;
    }
    return rv;
}

char nfaExecSheng_inAccept(const struct NFA *n, ReportID report, struct mq *q) {
    assert(n && q);

    const struct sheng *sh = get_sheng(n);
    u8 s = *(const u8 *)q->state;
    DEBUG_PRINTF("checking accepts for %u\n", (u8)(s & SHENG_STATE_MASK));

    const struct sstate_aux *aux = get_aux(sh, s);

    if (!aux->accept) {
        return 0;
    }

    return shengHasAccept(sh, aux, report);
}

char nfaExecSheng_inAnyAccept(const struct NFA *n, struct mq *q) {
    assert(n && q);

    const struct sheng *sh = get_sheng(n);
    u8 s = *(const u8 *)q->state;
    DEBUG_PRINTF("checking accepts for %u\n", (u8)(s & SHENG_STATE_MASK));

    const struct sstate_aux *aux = get_aux(sh, s);
    return !!aux->accept;
}

char nfaExecSheng_testEOD(const struct NFA *nfa, const char *state,
                          UNUSED const char *streamState, u64a offset,
                          NfaCallback cb, void *ctxt) {
    assert(nfa);

    const struct sheng *sh = get_sheng(nfa);
    u8 s = *(const u8 *)state;
    DEBUG_PRINTF("checking EOD accepts for %u\n", (u8)(s & SHENG_STATE_MASK));

    const struct sstate_aux *aux = get_aux(sh, s);

    if (!aux->accept_eod) {
        return MO_CONTINUE_MATCHING;
    }

    return fireReports(sh, cb, ctxt, s, offset, NULL, NULL, 1);
}

char nfaExecSheng_reportCurrent(const struct NFA *n, struct mq *q) {
    const struct sheng *sh = (const struct sheng *)getImplNfa(n);
    NfaCallback cb = q->cb;
    void *ctxt = q->context;
    u8 s = *(u8 *)q->state;
    const struct sstate_aux *aux = get_aux(sh, s);
    u64a offset = q_cur_offset(q);
    u8 cached_state_id = 0;
    ReportID cached_report_id = 0;
    assert(q_cur_type(q) == MQE_START);

    if (aux->accept) {
        if (sh->flags & SHENG_FLAG_SINGLE_REPORT) {
            fireSingleReport(cb, ctxt, sh->report, offset);
        } else {
            fireReports(sh, cb, ctxt, s, offset, &cached_state_id,
                        &cached_report_id, 0);
        }
    }

    return 0;
}

char nfaExecSheng_initCompressedState(const struct NFA *nfa, u64a offset,
                                      void *state, UNUSED u8 key) {
    const struct sheng *sh = get_sheng(nfa);
    u8 *s = (u8 *)state;
    *s = offset ? sh->floating: sh->anchored;
    return !(*s & SHENG_STATE_DEAD);
}

char nfaExecSheng_queueInitState(const struct NFA *nfa, struct mq *q) {
    assert(nfa->scratchStateSize == 1);

    /* starting in floating state */
    const struct sheng *sh = get_sheng(nfa);
    *(u8 *)q->state = sh->floating;
    DEBUG_PRINTF("starting in floating state\n");
    return 0;
}

char nfaExecSheng_queueCompressState(UNUSED const struct NFA *nfa,
                                     const struct mq *q, UNUSED s64a loc) {
    void *dest = q->streamState;
    const void *src = q->state;
    assert(nfa->scratchStateSize == 1);
    assert(nfa->streamStateSize == 1);
    *(u8 *)dest = *(const u8 *)src;
    return 0;
}

char nfaExecSheng_expandState(UNUSED const struct NFA *nfa, void *dest,
                              const void *src, UNUSED u64a offset,
                              UNUSED u8 key) {
    assert(nfa->scratchStateSize == 1);
    assert(nfa->streamStateSize == 1);
    *(u8 *)dest = *(const u8 *)src;
    return 0;
}

#if defined(HAVE_AVX512VBMI)
// Sheng32
static really_inline
char runSheng32Cb(const struct sheng32 *sh, NfaCallback cb, void *ctxt,
                  u64a offset, u8 *const cached_accept_state,
                  ReportID *const cached_accept_id, const u8 *cur_buf,
                  const u8 *start, const u8 *end, u8 can_die,
                  u8 has_accel, u8 single, const u8 **scanned, u8 *state) {
    DEBUG_PRINTF("Scanning %llu bytes (offset %llu) in callback mode\n",
                 (u64a)(end - start), offset);
    DEBUG_PRINTF("start: %lli end: %lli\n", (s64a)(start - cur_buf),
                 (s64a)(end - cur_buf));
    DEBUG_PRINTF("can die: %u has accel: %u single: %u\n", !!can_die,
                 !!has_accel, !!single);
    int rv;
    /* scan and report all matches */
    if (can_die) {
        if (has_accel) {
            rv = sheng32_4_coda(state, cb, ctxt, sh, cached_accept_state,
                                cached_accept_id, single, offset, cur_buf,
                                start, end, scanned);
        } else {
            rv = sheng32_4_cod(state, cb, ctxt, sh, cached_accept_state,
                               cached_accept_id, single, offset, cur_buf,
                               start, end, scanned);
        }
        if (rv == MO_HALT_MATCHING) {
            return MO_DEAD;
        }
        rv = sheng32_cod(state, cb, ctxt, sh, cached_accept_state,
                         cached_accept_id, single, offset, cur_buf,
                         *scanned, end, scanned);
    } else {
        if (has_accel) {
            rv = sheng32_4_coa(state, cb, ctxt, sh, cached_accept_state,
                               cached_accept_id, single, offset, cur_buf,
                               start, end, scanned);
        } else {
            rv = sheng32_4_co(state, cb, ctxt, sh, cached_accept_state,
                              cached_accept_id, single, offset, cur_buf,
                              start, end, scanned);
        }
        if (rv == MO_HALT_MATCHING) {
            return MO_DEAD;
        }
        rv = sheng32_co(state, cb, ctxt, sh, cached_accept_state,
                        cached_accept_id, single, offset, cur_buf,
                        *scanned, end, scanned);
    }
    if (rv == MO_HALT_MATCHING) {
        return MO_DEAD;
    }
    return MO_ALIVE;
}

static really_inline
void runSheng32Nm(const struct sheng32 *sh, NfaCallback cb, void *ctxt,
                  u64a offset, u8 *const cached_accept_state,
                  ReportID *const cached_accept_id, const u8 *cur_buf,
                  const u8 *start, const u8 *end, u8 can_die, u8 has_accel,
                  u8 single, const u8 **scanned, u8 *state) {
    DEBUG_PRINTF("Scanning %llu bytes (offset %llu) in nomatch mode\n",
                 (u64a)(end - start), offset);
    DEBUG_PRINTF("start: %lli end: %lli\n", (s64a)(start - cur_buf),
                 (s64a)(end - cur_buf));
    DEBUG_PRINTF("can die: %u has accel: %u single: %u\n", !!can_die,
                 !!has_accel, !!single);
    /* just scan the buffer */
    if (can_die) {
        if (has_accel) {
            sheng32_4_nmda(state, cb, ctxt, sh, cached_accept_state,
                           cached_accept_id, single, offset, cur_buf,
                           start, end, scanned);
        } else {
            sheng32_4_nmd(state, cb, ctxt, sh, cached_accept_state,
                          cached_accept_id, single, offset, cur_buf,
                          start, end, scanned);
        }
        sheng32_nmd(state, cb, ctxt, sh, cached_accept_state, cached_accept_id,
                    single, offset, cur_buf, *scanned, end, scanned);
    } else {
        sheng32_4_nm(state, cb, ctxt, sh, cached_accept_state, cached_accept_id,
                     single, offset, cur_buf, start, end, scanned);
        sheng32_nm(state, cb, ctxt, sh, cached_accept_state, cached_accept_id,
                   single, offset, cur_buf, *scanned, end, scanned);
    }
}

static really_inline
char runSheng32Sam(const struct sheng32 *sh, NfaCallback cb, void *ctxt,
                   u64a offset, u8 *const cached_accept_state,
                   ReportID *const cached_accept_id, const u8 *cur_buf,
                   const u8 *start, const u8 *end, u8 can_die, u8 has_accel,
                   u8 single, const u8 **scanned, u8 *state) {
    DEBUG_PRINTF("Scanning %llu bytes (offset %llu) in stop at match mode\n",
                 (u64a)(end - start), offset);
    DEBUG_PRINTF("start: %lli end: %lli\n", (s64a)(start - cur_buf),
                 (s64a)(end - cur_buf));
    DEBUG_PRINTF("can die: %u has accel: %u single: %u\n", !!can_die,
                 !!has_accel, !!single);
    int rv;
    /* scan until first match */
    if (can_die) {
        if (has_accel) {
            rv = sheng32_4_samda(state, cb, ctxt, sh, cached_accept_state,
                                 cached_accept_id, single, offset, cur_buf,
                                 start, end, scanned);
        } else {
            rv = sheng32_4_samd(state, cb, ctxt, sh, cached_accept_state,
                                cached_accept_id, single, offset, cur_buf,
                                start, end, scanned);
        }
        if (rv == MO_HALT_MATCHING) {
            return MO_DEAD;
        }
        /* if we stopped before we expected, we found a match */
        if (rv == MO_MATCHES_PENDING) {
            return MO_MATCHES_PENDING;
        }

        rv = sheng32_samd(state, cb, ctxt, sh, cached_accept_state,
                          cached_accept_id, single, offset, cur_buf,
                          *scanned, end, scanned);
    } else {
        if (has_accel) {
            rv = sheng32_4_sama(state, cb, ctxt, sh, cached_accept_state,
                                cached_accept_id, single, offset, cur_buf,
                                start, end, scanned);
        } else {
            rv = sheng32_4_sam(state, cb, ctxt, sh, cached_accept_state,
                               cached_accept_id, single, offset, cur_buf,
                               start, end, scanned);
        }
        if (rv == MO_HALT_MATCHING) {
            return MO_DEAD;
        }
        /* if we stopped before we expected, we found a match */
        if (rv == MO_MATCHES_PENDING) {
            return MO_MATCHES_PENDING;
        }

        rv = sheng32_sam(state, cb, ctxt, sh, cached_accept_state,
                         cached_accept_id, single, offset, cur_buf,
                         *scanned, end, scanned);
    }
    if (rv == MO_HALT_MATCHING) {
        return MO_DEAD;
    }
    /* if we stopped before we expected, we found a match */
    if (rv == MO_MATCHES_PENDING) {
        return MO_MATCHES_PENDING;
    }
    return MO_ALIVE;
}

static never_inline
char runSheng32(const struct sheng32 *sh, struct mq *q, s64a b_end,
                enum MatchMode mode) {
    u8 state = *(u8 *)q->state;
    u8 can_die = sh->flags & SHENG_FLAG_CAN_DIE;
    u8 has_accel = sh->flags & SHENG_FLAG_HAS_ACCEL;
    u8 single = sh->flags & SHENG_FLAG_SINGLE_REPORT;

    u8 cached_accept_state = 0;
    ReportID cached_accept_id = 0;

    DEBUG_PRINTF("starting Sheng32 execution in state %u\n",
                 state & SHENG32_STATE_MASK);

    if (q->report_current) {
        DEBUG_PRINTF("reporting current pending matches\n");
        assert(sh);

        q->report_current = 0;

        int rv;
        if (single) {
            rv = fireSingleReport(q->cb, q->context, sh->report,
                                  q_cur_offset(q));
        } else {
            rv = fireReports32(sh, q->cb, q->context, state, q_cur_offset(q),
                               &cached_accept_state, &cached_accept_id, 0);
        }
        if (rv == MO_HALT_MATCHING) {
            DEBUG_PRINTF("exiting in state %u\n", state & SHENG32_STATE_MASK);
            return MO_DEAD;
        }

        DEBUG_PRINTF("proceeding with matching\n");
    }

    assert(q_cur_type(q) == MQE_START);
    s64a start = q_cur_loc(q);

    DEBUG_PRINTF("offset: %lli, location: %lli, mode: %s\n", q->offset, start,
                 mode == CALLBACK_OUTPUT ? "CALLBACK OUTPUT" :
                     mode == NO_MATCHES ? "NO MATCHES" :
                         mode == STOP_AT_MATCH ? "STOP AT MATCH" : "???");

    DEBUG_PRINTF("processing event @ %lli: %s\n", q->offset + q_cur_loc(q),
                 q_cur_type(q) == MQE_START ? "START" :
                     q_cur_type(q) == MQE_TOP ? "TOP" :
                         q_cur_type(q) == MQE_END ? "END" : "???");

    const u8* cur_buf;
    if (start < 0) {
        DEBUG_PRINTF("negative location, scanning history\n");
        DEBUG_PRINTF("min location: %zd\n", -q->hlength);
        cur_buf = q->history + q->hlength;
    } else {
        DEBUG_PRINTF("positive location, scanning buffer\n");
        DEBUG_PRINTF("max location: %lli\n", b_end);
        cur_buf = q->buffer;
    }

    /* if we our queue event is past our end */
    if (mode != NO_MATCHES && q_cur_loc(q) > b_end) {
        DEBUG_PRINTF("current location past buffer end\n");
        DEBUG_PRINTF("setting q location to %llu\n", b_end);
        DEBUG_PRINTF("exiting in state %u\n", state & SHENG32_STATE_MASK);
        q->items[q->cur].location = b_end;
        return MO_ALIVE;
    }

    q->cur++;

    s64a cur_start = start;

    while (1) {
        DEBUG_PRINTF("processing event @ %lli: %s\n", q->offset + q_cur_loc(q),
                     q_cur_type(q) == MQE_START ? "START" :
                             q_cur_type(q) == MQE_TOP ? "TOP" :
                                     q_cur_type(q) == MQE_END ? "END" : "???");
        s64a end = q_cur_loc(q);
        if (mode != NO_MATCHES) {
            end = MIN(end, b_end);
        }
        assert(end <= (s64a) q->length);
        s64a cur_end = end;

        /* we may cross the border between history and current buffer */
        if (cur_start < 0) {
            cur_end = MIN(0, cur_end);
        }

        DEBUG_PRINTF("start: %lli end: %lli\n", start, end);

        /* don't scan zero length buffer */
        if (cur_start != cur_end) {
            const u8 * scanned = cur_buf;
            char rv;

            if (mode == NO_MATCHES) {
                runSheng32Nm(sh, q->cb, q->context, q->offset,
                             &cached_accept_state, &cached_accept_id, cur_buf,
                             cur_buf + cur_start, cur_buf + cur_end, can_die,
                             has_accel, single, &scanned, &state);
            } else if (mode == CALLBACK_OUTPUT) {
                rv = runSheng32Cb(sh, q->cb, q->context, q->offset,
                                  &cached_accept_state, &cached_accept_id,
                                  cur_buf, cur_buf + cur_start, cur_buf + cur_end,
                                  can_die, has_accel, single, &scanned, &state);
                if (rv == MO_DEAD) {
                    DEBUG_PRINTF("exiting in state %u\n",
                                 state & SHENG32_STATE_MASK);
                    return MO_DEAD;
                }
            } else if (mode == STOP_AT_MATCH) {
                rv = runSheng32Sam(sh, q->cb, q->context, q->offset,
                                   &cached_accept_state, &cached_accept_id,
                                   cur_buf, cur_buf + cur_start,
                                   cur_buf + cur_end, can_die, has_accel, single,
                                   &scanned, &state);
                if (rv == MO_DEAD) {
                    DEBUG_PRINTF("exiting in state %u\n",
                                 state & SHENG32_STATE_MASK);
                    return rv;
                } else if (rv == MO_MATCHES_PENDING) {
                    assert(q->cur);
                    DEBUG_PRINTF("found a match, setting q location to %zd\n",
                                 scanned - cur_buf + 1);
                    q->cur--;
                    q->items[q->cur].type = MQE_START;
                    q->items[q->cur].location =
                            scanned - cur_buf + 1; /* due to exiting early */
                    *(u8 *)q->state = state;
                    DEBUG_PRINTF("exiting in state %u\n",
                                 state & SHENG32_STATE_MASK);
                    return rv;
                }
            } else {
                assert(!"invalid scanning mode!");
            }
            assert(scanned == cur_buf + cur_end);

            cur_start = cur_end;
        }

        /* if we our queue event is past our end */
        if (mode != NO_MATCHES && q_cur_loc(q) > b_end) {
            DEBUG_PRINTF("current location past buffer end\n");
            DEBUG_PRINTF("setting q location to %llu\n", b_end);
            DEBUG_PRINTF("exiting in state %u\n", state & SHENG32_STATE_MASK);
            q->cur--;
            q->items[q->cur].type = MQE_START;
            q->items[q->cur].location = b_end;
            *(u8 *)q->state = state;
            return MO_ALIVE;
        }

        /* crossing over into actual buffer */
        if (cur_start == 0) {
            DEBUG_PRINTF("positive location, scanning buffer\n");
            DEBUG_PRINTF("max offset: %lli\n", b_end);
            cur_buf = q->buffer;
        }

        /* continue scanning the same buffer */
        if (end != cur_end) {
            continue;
        }

        switch (q_cur_type(q)) {
        case MQE_END:
            *(u8 *)q->state = state;
            q->cur++;
            DEBUG_PRINTF("exiting in state %u\n", state & SHENG32_STATE_MASK);
            if (can_die) {
                return (state & SHENG32_STATE_DEAD) ? MO_DEAD : MO_ALIVE;
            }
            return MO_ALIVE;
        case MQE_TOP:
            if (q->offset + cur_start == 0) {
                DEBUG_PRINTF("Anchored start, going to state %u\n",
                             sh->anchored);
                state = sh->anchored;
            } else {
                u8 new_state = get_aux32(sh, state)->top;
                DEBUG_PRINTF("Top event %u->%u\n", state & SHENG32_STATE_MASK,
                             new_state & SHENG32_STATE_MASK);
                state = new_state;
            }
            break;
        default:
            assert(!"invalid queue event");
            break;
        }
        q->cur++;
    }
}

char nfaExecSheng32_B(const struct NFA *n, u64a offset, const u8 *buffer,
                      size_t length, NfaCallback cb, void *context) {
    DEBUG_PRINTF("smallwrite Sheng32\n");
    assert(n->type == SHENG_NFA_32);
    const struct sheng32 *sh = getImplNfa(n);
    u8 state = sh->anchored;
    u8 can_die = sh->flags & SHENG_FLAG_CAN_DIE;
    u8 has_accel = sh->flags & SHENG_FLAG_HAS_ACCEL;
    u8 single = sh->flags & SHENG_FLAG_SINGLE_REPORT;
    u8 cached_accept_state = 0;
    ReportID cached_accept_id = 0;

    /* scan and report all matches */
    int rv;
    s64a end = length;
    const u8 *scanned;

    rv = runSheng32Cb(sh, cb, context, offset, &cached_accept_state,
                      &cached_accept_id, buffer, buffer, buffer + end, can_die,
                      has_accel, single, &scanned, &state);
    if (rv == MO_DEAD) {
        DEBUG_PRINTF("exiting in state %u\n",
                     state & SHENG32_STATE_MASK);
        return MO_DEAD;
    }

    DEBUG_PRINTF("%u\n", state & SHENG32_STATE_MASK);

    const struct sstate_aux *aux = get_aux32(sh, state);

    if (aux->accept_eod) {
        DEBUG_PRINTF("Reporting EOD matches\n");
        fireReports32(sh, cb, context, state, end + offset,
                      &cached_accept_state, &cached_accept_id, 1);
    }

    return state & SHENG32_STATE_DEAD ? MO_DEAD : MO_ALIVE;
}

char nfaExecSheng32_Q(const struct NFA *n, struct mq *q, s64a end) {
    const struct sheng32 *sh = get_sheng32(n);
    char rv = runSheng32(sh, q, end, CALLBACK_OUTPUT);
    return rv;
}

char nfaExecSheng32_Q2(const struct NFA *n, struct mq *q, s64a end) {
    const struct sheng32 *sh = get_sheng32(n);
    char rv = runSheng32(sh, q, end, STOP_AT_MATCH);
    return rv;
}

char nfaExecSheng32_QR(const struct NFA *n, struct mq *q, ReportID report) {
    assert(q_cur_type(q) == MQE_START);

    const struct sheng32 *sh = get_sheng32(n);
    char rv = runSheng32(sh, q, 0 /* end */, NO_MATCHES);

    if (rv && nfaExecSheng32_inAccept(n, report, q)) {
        return MO_MATCHES_PENDING;
    }
    return rv;
}

char nfaExecSheng32_inAccept(const struct NFA *n, ReportID report,
                             struct mq *q) {
    assert(n && q);

    const struct sheng32 *sh = get_sheng32(n);
    u8 s = *(const u8 *)q->state;
    DEBUG_PRINTF("checking accepts for %u\n", (u8)(s & SHENG32_STATE_MASK));

    const struct sstate_aux *aux = get_aux32(sh, s);

    if (!aux->accept) {
        return 0;
    }

    return sheng32HasAccept(sh, aux, report);
}

char nfaExecSheng32_inAnyAccept(const struct NFA *n, struct mq *q) {
    assert(n && q);

    const struct sheng32 *sh = get_sheng32(n);
    u8 s = *(const u8 *)q->state;
    DEBUG_PRINTF("checking accepts for %u\n", (u8)(s & SHENG32_STATE_MASK));

    const struct sstate_aux *aux = get_aux32(sh, s);
    return !!aux->accept;
}

char nfaExecSheng32_testEOD(const struct NFA *nfa, const char *state,
                            UNUSED const char *streamState, u64a offset,
                            NfaCallback cb, void *ctxt) {
    assert(nfa);

    const struct sheng32 *sh = get_sheng32(nfa);
    u8 s = *(const u8 *)state;
    DEBUG_PRINTF("checking EOD accepts for %u\n", (u8)(s & SHENG32_STATE_MASK));

    const struct sstate_aux *aux = get_aux32(sh, s);

    if (!aux->accept_eod) {
        return MO_CONTINUE_MATCHING;
    }

    return fireReports32(sh, cb, ctxt, s, offset, NULL, NULL, 1);
}

char nfaExecSheng32_reportCurrent(const struct NFA *n, struct mq *q) {
    const struct sheng32 *sh = (const struct sheng32 *)getImplNfa(n);
    NfaCallback cb = q->cb;
    void *ctxt = q->context;
    u8 s = *(u8 *)q->state;
    const struct sstate_aux *aux = get_aux32(sh, s);
    u64a offset = q_cur_offset(q);
    u8 cached_state_id = 0;
    ReportID cached_report_id = 0;
    assert(q_cur_type(q) == MQE_START);

    if (aux->accept) {
        if (sh->flags & SHENG_FLAG_SINGLE_REPORT) {
            fireSingleReport(cb, ctxt, sh->report, offset);
        } else {
            fireReports32(sh, cb, ctxt, s, offset, &cached_state_id,
                          &cached_report_id, 0);
        }
    }

    return 0;
}

char nfaExecSheng32_initCompressedState(const struct NFA *nfa, u64a offset,
                                        void *state, UNUSED u8 key) {
    const struct sheng32 *sh = get_sheng32(nfa);
    u8 *s = (u8 *)state;
    *s = offset ? sh->floating: sh->anchored;
    return !(*s & SHENG32_STATE_DEAD);
}

char nfaExecSheng32_queueInitState(const struct NFA *nfa, struct mq *q) {
    assert(nfa->scratchStateSize == 1);

    /* starting in floating state */
    const struct sheng32 *sh = get_sheng32(nfa);
    *(u8 *)q->state = sh->floating;
    DEBUG_PRINTF("starting in floating state\n");
    return 0;
}

char nfaExecSheng32_queueCompressState(UNUSED const struct NFA *nfa,
                                       const struct mq *q, UNUSED s64a loc) {
    void *dest = q->streamState;
    const void *src = q->state;
    assert(nfa->scratchStateSize == 1);
    assert(nfa->streamStateSize == 1);
    *(u8 *)dest = *(const u8 *)src;
    return 0;
}

char nfaExecSheng32_expandState(UNUSED const struct NFA *nfa, void *dest,
                                const void *src, UNUSED u64a offset,
                                UNUSED u8 key) {
    assert(nfa->scratchStateSize == 1);
    assert(nfa->streamStateSize == 1);
    *(u8 *)dest = *(const u8 *)src;
    return 0;
}

// Sheng64
static really_inline
char runSheng64Cb(const struct sheng64 *sh, NfaCallback cb, void *ctxt,
                  u64a offset, u8 *const cached_accept_state,
                  ReportID *const cached_accept_id, const u8 *cur_buf,
                  const u8 *start, const u8 *end, u8 can_die,
                  u8 single, const u8 **scanned, u8 *state) {
    DEBUG_PRINTF("Scanning %llu bytes (offset %llu) in callback mode\n",
                 (u64a)(end - start), offset);
    DEBUG_PRINTF("start: %lli end: %lli\n", (s64a)(start - cur_buf),
                 (s64a)(end - cur_buf));
    DEBUG_PRINTF("can die: %u single: %u\n", !!can_die, !!single);
    int rv;
    /* scan and report all matches */
    if (can_die) {
        rv = sheng64_4_cod(state, cb, ctxt, sh, cached_accept_state,
                           cached_accept_id, single, offset, cur_buf,
                           start, end, scanned);
        if (rv == MO_HALT_MATCHING) {
            return MO_DEAD;
        }
        rv = sheng64_cod(state, cb, ctxt, sh, cached_accept_state,
                         cached_accept_id, single, offset, cur_buf,
                         *scanned, end, scanned);
    } else {
        rv = sheng64_4_co(state, cb, ctxt, sh, cached_accept_state,
                          cached_accept_id, single, offset, cur_buf,
                          start, end, scanned);
        if (rv == MO_HALT_MATCHING) {
            return MO_DEAD;
        }
        rv = sheng64_co(state, cb, ctxt, sh, cached_accept_state,
                        cached_accept_id, single, offset, cur_buf,
                        *scanned, end, scanned);
    }
    if (rv == MO_HALT_MATCHING) {
        return MO_DEAD;
    }
    return MO_ALIVE;
}

static really_inline
void runSheng64Nm(const struct sheng64 *sh, NfaCallback cb, void *ctxt,
                  u64a offset, u8 *const cached_accept_state,
                  ReportID *const cached_accept_id, const u8 *cur_buf,
                  const u8 *start, const u8 *end, u8 can_die,
                  u8 single, const u8 **scanned, u8 *state) {
    DEBUG_PRINTF("Scanning %llu bytes (offset %llu) in nomatch mode\n",
                 (u64a)(end - start), offset);
    DEBUG_PRINTF("start: %lli end: %lli\n", (s64a)(start - cur_buf),
                 (s64a)(end - cur_buf));
    DEBUG_PRINTF("can die: %u single: %u\n", !!can_die, !!single);
    /* just scan the buffer */
    if (can_die) {
        sheng64_4_nmd(state, cb, ctxt, sh, cached_accept_state,
                      cached_accept_id, single, offset, cur_buf,
                      start, end, scanned);
        sheng64_nmd(state, cb, ctxt, sh, cached_accept_state, cached_accept_id,
                    single, offset, cur_buf, *scanned, end, scanned);
    } else {
        sheng64_4_nm(state, cb, ctxt, sh, cached_accept_state, cached_accept_id,
                     single, offset, cur_buf, start, end, scanned);
        sheng64_nm(state, cb, ctxt, sh, cached_accept_state, cached_accept_id,
                   single, offset, cur_buf, *scanned, end, scanned);
    }
}

static really_inline
char runSheng64Sam(const struct sheng64 *sh, NfaCallback cb, void *ctxt,
                   u64a offset, u8 *const cached_accept_state,
                   ReportID *const cached_accept_id, const u8 *cur_buf,
                   const u8 *start, const u8 *end, u8 can_die,
                   u8 single, const u8 **scanned, u8 *state) {
    DEBUG_PRINTF("Scanning %llu bytes (offset %llu) in stop at match mode\n",
                 (u64a)(end - start), offset);
    DEBUG_PRINTF("start: %lli end: %lli\n", (s64a)(start - cur_buf),
                 (s64a)(end - cur_buf));
    DEBUG_PRINTF("can die: %u single: %u\n", !!can_die, !!single);
    int rv;
    /* scan until first match */
    if (can_die) {
        rv = sheng64_4_samd(state, cb, ctxt, sh, cached_accept_state,
                            cached_accept_id, single, offset, cur_buf,
                            start, end, scanned);
        if (rv == MO_HALT_MATCHING) {
            return MO_DEAD;
        }
        /* if we stopped before we expected, we found a match */
        if (rv == MO_MATCHES_PENDING) {
            return MO_MATCHES_PENDING;
        }

        rv = sheng64_samd(state, cb, ctxt, sh, cached_accept_state,
                          cached_accept_id, single, offset, cur_buf,
                          *scanned, end, scanned);
    } else {
        rv = sheng64_4_sam(state, cb, ctxt, sh, cached_accept_state,
                           cached_accept_id, single, offset, cur_buf,
                           start, end, scanned);
        if (rv == MO_HALT_MATCHING) {
            return MO_DEAD;
        }
        /* if we stopped before we expected, we found a match */
        if (rv == MO_MATCHES_PENDING) {
            return MO_MATCHES_PENDING;
        }

        rv = sheng64_sam(state, cb, ctxt, sh, cached_accept_state,
                         cached_accept_id, single, offset, cur_buf,
                         *scanned, end, scanned);
    }
    if (rv == MO_HALT_MATCHING) {
        return MO_DEAD;
    }
    /* if we stopped before we expected, we found a match */
    if (rv == MO_MATCHES_PENDING) {
        return MO_MATCHES_PENDING;
    }
    return MO_ALIVE;
}

static never_inline
char runSheng64(const struct sheng64 *sh, struct mq *q, s64a b_end,
                enum MatchMode mode) {
    u8 state = *(u8 *)q->state;
    u8 can_die = sh->flags & SHENG_FLAG_CAN_DIE;
    u8 single = sh->flags & SHENG_FLAG_SINGLE_REPORT;

    u8 cached_accept_state = 0;
    ReportID cached_accept_id = 0;

    DEBUG_PRINTF("starting Sheng64 execution in state %u\n",
                 state & SHENG64_STATE_MASK);

    if (q->report_current) {
        DEBUG_PRINTF("reporting current pending matches\n");
        assert(sh);

        q->report_current = 0;

        int rv;
        if (single) {
            rv = fireSingleReport(q->cb, q->context, sh->report,
                                  q_cur_offset(q));
        } else {
            rv = fireReports64(sh, q->cb, q->context, state, q_cur_offset(q),
                               &cached_accept_state, &cached_accept_id, 0);
        }
        if (rv == MO_HALT_MATCHING) {
            DEBUG_PRINTF("exiting in state %u\n", state & SHENG64_STATE_MASK);
            return MO_DEAD;
        }

        DEBUG_PRINTF("proceeding with matching\n");
    }

    assert(q_cur_type(q) == MQE_START);
    s64a start = q_cur_loc(q);

    DEBUG_PRINTF("offset: %lli, location: %lli, mode: %s\n", q->offset, start,
                 mode == CALLBACK_OUTPUT ? "CALLBACK OUTPUT" :
                     mode == NO_MATCHES ? "NO MATCHES" :
                         mode == STOP_AT_MATCH ? "STOP AT MATCH" : "???");

    DEBUG_PRINTF("processing event @ %lli: %s\n", q->offset + q_cur_loc(q),
                 q_cur_type(q) == MQE_START ? "START" :
                     q_cur_type(q) == MQE_TOP ? "TOP" :
                         q_cur_type(q) == MQE_END ? "END" : "???");

    const u8* cur_buf;
    if (start < 0) {
        DEBUG_PRINTF("negative location, scanning history\n");
        DEBUG_PRINTF("min location: %zd\n", -q->hlength);
        cur_buf = q->history + q->hlength;
    } else {
        DEBUG_PRINTF("positive location, scanning buffer\n");
        DEBUG_PRINTF("max location: %lli\n", b_end);
        cur_buf = q->buffer;
    }

    /* if we our queue event is past our end */
    if (mode != NO_MATCHES && q_cur_loc(q) > b_end) {
        DEBUG_PRINTF("current location past buffer end\n");
        DEBUG_PRINTF("setting q location to %llu\n", b_end);
        DEBUG_PRINTF("exiting in state %u\n", state & SHENG64_STATE_MASK);
        q->items[q->cur].location = b_end;
        return MO_ALIVE;
    }

    q->cur++;

    s64a cur_start = start;

    while (1) {
        DEBUG_PRINTF("processing event @ %lli: %s\n", q->offset + q_cur_loc(q),
                     q_cur_type(q) == MQE_START ? "START" :
                             q_cur_type(q) == MQE_TOP ? "TOP" :
                                     q_cur_type(q) == MQE_END ? "END" : "???");
        s64a end = q_cur_loc(q);
        if (mode != NO_MATCHES) {
            end = MIN(end, b_end);
        }
        assert(end <= (s64a) q->length);
        s64a cur_end = end;

        /* we may cross the border between history and current buffer */
        if (cur_start < 0) {
            cur_end = MIN(0, cur_end);
        }

        DEBUG_PRINTF("start: %lli end: %lli\n", start, end);

        /* don't scan zero length buffer */
        if (cur_start != cur_end) {
            const u8 * scanned = cur_buf;
            char rv;

            if (mode == NO_MATCHES) {
                runSheng64Nm(sh, q->cb, q->context, q->offset,
                             &cached_accept_state, &cached_accept_id, cur_buf,
                             cur_buf + cur_start, cur_buf + cur_end, can_die,
                             single, &scanned, &state);
            } else if (mode == CALLBACK_OUTPUT) {
                rv = runSheng64Cb(sh, q->cb, q->context, q->offset,
                                  &cached_accept_state, &cached_accept_id,
                                  cur_buf, cur_buf + cur_start, cur_buf + cur_end,
                                  can_die, single, &scanned, &state);
                if (rv == MO_DEAD) {
                    DEBUG_PRINTF("exiting in state %u\n",
                                 state & SHENG64_STATE_MASK);
                    return MO_DEAD;
                }
            } else if (mode == STOP_AT_MATCH) {
                rv = runSheng64Sam(sh, q->cb, q->context, q->offset,
                                   &cached_accept_state, &cached_accept_id,
                                   cur_buf, cur_buf + cur_start,
                                   cur_buf + cur_end, can_die, single,
                                   &scanned, &state);
                if (rv == MO_DEAD) {
                    DEBUG_PRINTF("exiting in state %u\n",
                                 state & SHENG64_STATE_MASK);
                    return rv;
                } else if (rv == MO_MATCHES_PENDING) {
                    assert(q->cur);
                    DEBUG_PRINTF("found a match, setting q location to %zd\n",
                                 scanned - cur_buf + 1);
                    q->cur--;
                    q->items[q->cur].type = MQE_START;
                    q->items[q->cur].location =
                            scanned - cur_buf + 1; /* due to exiting early */
                    *(u8 *)q->state = state;
                    DEBUG_PRINTF("exiting in state %u\n",
                                 state & SHENG64_STATE_MASK);
                    return rv;
                }
            } else {
                assert(!"invalid scanning mode!");
            }
            assert(scanned == cur_buf + cur_end);

            cur_start = cur_end;
        }

        /* if we our queue event is past our end */
        if (mode != NO_MATCHES && q_cur_loc(q) > b_end) {
            DEBUG_PRINTF("current location past buffer end\n");
            DEBUG_PRINTF("setting q location to %llu\n", b_end);
            DEBUG_PRINTF("exiting in state %u\n", state & SHENG64_STATE_MASK);
            q->cur--;
            q->items[q->cur].type = MQE_START;
            q->items[q->cur].location = b_end;
            *(u8 *)q->state = state;
            return MO_ALIVE;
        }

        /* crossing over into actual buffer */
        if (cur_start == 0) {
            DEBUG_PRINTF("positive location, scanning buffer\n");
            DEBUG_PRINTF("max offset: %lli\n", b_end);
            cur_buf = q->buffer;
        }

        /* continue scanning the same buffer */
        if (end != cur_end) {
            continue;
        }

        switch (q_cur_type(q)) {
        case MQE_END:
            *(u8 *)q->state = state;
            q->cur++;
            DEBUG_PRINTF("exiting in state %u\n", state & SHENG64_STATE_MASK);
            if (can_die) {
                return (state & SHENG64_STATE_DEAD) ? MO_DEAD : MO_ALIVE;
            }
            return MO_ALIVE;
        case MQE_TOP:
            if (q->offset + cur_start == 0) {
                DEBUG_PRINTF("Anchored start, going to state %u\n",
                             sh->anchored);
                state = sh->anchored;
            } else {
                u8 new_state = get_aux64(sh, state)->top;
                DEBUG_PRINTF("Top event %u->%u\n", state & SHENG64_STATE_MASK,
                             new_state & SHENG64_STATE_MASK);
                state = new_state;
            }
            break;
        default:
            assert(!"invalid queue event");
            break;
        }
        q->cur++;
    }
}

char nfaExecSheng64_B(const struct NFA *n, u64a offset, const u8 *buffer,
                      size_t length, NfaCallback cb, void *context) {
    DEBUG_PRINTF("smallwrite Sheng64\n");
    assert(n->type == SHENG_NFA_64);
    const struct sheng64 *sh = getImplNfa(n);
    u8 state = sh->anchored;
    u8 can_die = sh->flags & SHENG_FLAG_CAN_DIE;
    u8 single = sh->flags & SHENG_FLAG_SINGLE_REPORT;
    u8 cached_accept_state = 0;
    ReportID cached_accept_id = 0;

    /* scan and report all matches */
    int rv;
    s64a end = length;
    const u8 *scanned;

    rv = runSheng64Cb(sh, cb, context, offset, &cached_accept_state,
                      &cached_accept_id, buffer, buffer, buffer + end, can_die,
                      single, &scanned, &state);
    if (rv == MO_DEAD) {
        DEBUG_PRINTF("exiting in state %u\n",
                     state & SHENG64_STATE_MASK);
        return MO_DEAD;
    }

    DEBUG_PRINTF("%u\n", state & SHENG64_STATE_MASK);

    const struct sstate_aux *aux = get_aux64(sh, state);

    if (aux->accept_eod) {
        DEBUG_PRINTF("Reporting EOD matches\n");
        fireReports64(sh, cb, context, state, end + offset,
                      &cached_accept_state, &cached_accept_id, 1);
    }

    return state & SHENG64_STATE_DEAD ? MO_DEAD : MO_ALIVE;
}

char nfaExecSheng64_Q(const struct NFA *n, struct mq *q, s64a end) {
    const struct sheng64 *sh = get_sheng64(n);
    char rv = runSheng64(sh, q, end, CALLBACK_OUTPUT);
    return rv;
}

char nfaExecSheng64_Q2(const struct NFA *n, struct mq *q, s64a end) {
    const struct sheng64 *sh = get_sheng64(n);
    char rv = runSheng64(sh, q, end, STOP_AT_MATCH);
    return rv;
}

char nfaExecSheng64_QR(const struct NFA *n, struct mq *q, ReportID report) {
    assert(q_cur_type(q) == MQE_START);

    const struct sheng64 *sh = get_sheng64(n);
    char rv = runSheng64(sh, q, 0 /* end */, NO_MATCHES);

    if (rv && nfaExecSheng64_inAccept(n, report, q)) {
        return MO_MATCHES_PENDING;
    }
    return rv;
}

char nfaExecSheng64_inAccept(const struct NFA *n, ReportID report,
                             struct mq *q) {
    assert(n && q);

    const struct sheng64 *sh = get_sheng64(n);
    u8 s = *(const u8 *)q->state;
    DEBUG_PRINTF("checking accepts for %u\n", (u8)(s & SHENG64_STATE_MASK));

    const struct sstate_aux *aux = get_aux64(sh, s);

    if (!aux->accept) {
        return 0;
    }

    return sheng64HasAccept(sh, aux, report);
}

char nfaExecSheng64_inAnyAccept(const struct NFA *n, struct mq *q) {
    assert(n && q);

    const struct sheng64 *sh = get_sheng64(n);
    u8 s = *(const u8 *)q->state;
    DEBUG_PRINTF("checking accepts for %u\n", (u8)(s & SHENG64_STATE_MASK));

    const struct sstate_aux *aux = get_aux64(sh, s);
    return !!aux->accept;
}

char nfaExecSheng64_testEOD(const struct NFA *nfa, const char *state,
                            UNUSED const char *streamState, u64a offset,
                            NfaCallback cb, void *ctxt) {
    assert(nfa);

    const struct sheng64 *sh = get_sheng64(nfa);
    u8 s = *(const u8 *)state;
    DEBUG_PRINTF("checking EOD accepts for %u\n", (u8)(s & SHENG64_STATE_MASK));

    const struct sstate_aux *aux = get_aux64(sh, s);

    if (!aux->accept_eod) {
        return MO_CONTINUE_MATCHING;
    }

    return fireReports64(sh, cb, ctxt, s, offset, NULL, NULL, 1);
}

char nfaExecSheng64_reportCurrent(const struct NFA *n, struct mq *q) {
    const struct sheng64 *sh = (const struct sheng64 *)getImplNfa(n);
    NfaCallback cb = q->cb;
    void *ctxt = q->context;
    u8 s = *(u8 *)q->state;
    const struct sstate_aux *aux = get_aux64(sh, s);
    u64a offset = q_cur_offset(q);
    u8 cached_state_id = 0;
    ReportID cached_report_id = 0;
    assert(q_cur_type(q) == MQE_START);

    if (aux->accept) {
        if (sh->flags & SHENG_FLAG_SINGLE_REPORT) {
            fireSingleReport(cb, ctxt, sh->report, offset);
        } else {
            fireReports64(sh, cb, ctxt, s, offset, &cached_state_id,
                          &cached_report_id, 0);
        }
    }

    return 0;
}

char nfaExecSheng64_initCompressedState(const struct NFA *nfa, u64a offset,
                                        void *state, UNUSED u8 key) {
    const struct sheng64 *sh = get_sheng64(nfa);
    u8 *s = (u8 *)state;
    *s = offset ? sh->floating: sh->anchored;
    return !(*s & SHENG64_STATE_DEAD);
}

char nfaExecSheng64_queueInitState(const struct NFA *nfa, struct mq *q) {
    assert(nfa->scratchStateSize == 1);

    /* starting in floating state */
    const struct sheng64 *sh = get_sheng64(nfa);
    *(u8 *)q->state = sh->floating;
    DEBUG_PRINTF("starting in floating state\n");
    return 0;
}

char nfaExecSheng64_queueCompressState(UNUSED const struct NFA *nfa,
                                       const struct mq *q, UNUSED s64a loc) {
    void *dest = q->streamState;
    const void *src = q->state;
    assert(nfa->scratchStateSize == 1);
    assert(nfa->streamStateSize == 1);
    *(u8 *)dest = *(const u8 *)src;
    return 0;
}

char nfaExecSheng64_expandState(UNUSED const struct NFA *nfa, void *dest,
                                const void *src, UNUSED u64a offset,
                                UNUSED u8 key) {
    assert(nfa->scratchStateSize == 1);
    assert(nfa->streamStateSize == 1);
    *(u8 *)dest = *(const u8 *)src;
    return 0;
}
#endif // end of HAVE_AVX512VBMI
