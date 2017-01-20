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

#include "mpv.h"

#include "mpv_internal.h"
#include "nfa_api.h"
#include "nfa_api_queue.h"
#include "nfa_internal.h"
#include "shufti.h"
#include "truffle.h"
#include "ue2common.h"
#include "vermicelli.h"
#include "vermicelli_run.h"
#include "util/multibit.h"
#include "util/partial_store.h"
#include "util/simd_utils.h"
#include "util/unaligned.h"

#include <string.h>

#define MIN_SKIP_REPEAT 32

typedef struct mpv_pq_item PQ_T;
#define PQ_COMP(pqc_items, a, b) \
    ((pqc_items)[a].trigger_loc < (pqc_items)[b].trigger_loc)
#define PQ_COMP_B(pqc_items, a, b_fixed) \
    ((pqc_items)[a].trigger_loc < (b_fixed).trigger_loc)

#include "util/pqueue.h"

static really_inline
u64a *get_counter_n(struct mpv_decomp_state *s,
                    const struct mpv *m, u32 n) {
    return (u64a *)((char *)s + get_counter_info(m)[n].counter_offset);
}

static really_inline
u64a *get_counter_for_kilo(struct mpv_decomp_state *s,
                           const struct mpv_kilopuff *kp) {
    return (u64a *)((char *)s + kp->counter_offset);
}

static really_inline
u64a get_counter_value_for_kilo(struct mpv_decomp_state *s,
                                const struct mpv_kilopuff *kp) {
    return *get_counter_for_kilo(s, kp) + s->counter_adj;
}

static really_inline
const u64a *get_counter_for_kilo_c(const struct mpv_decomp_state *s,
                             const struct mpv_kilopuff *kp) {
    return (const u64a *)((const char *)s + kp->counter_offset);
}


static never_inline
void normalize_counters(struct mpv_decomp_state *dstate, const struct mpv *m) {
    u64a adj = dstate->counter_adj;
    u64a *counters = get_counter_n(dstate, m, 0);

    if (!adj) {
        return;
    }

    for (u32 i = 0; i < m->counter_count; i++) {
        /* update all counters - alive or dead */
        counters[i] += adj;
        DEBUG_PRINTF("counter %u: %llu\n", i, counters[i]);
    }

    dstate->counter_adj = 0;
}

static really_inline
char processReports(const struct mpv *m, u8 *reporters,
                    const struct mpv_decomp_state *dstate, u64a counter_adj,
                    u64a report_offset, NfaCallback cb, void *ctxt,
                    ReportID *rl, u32 *rl_count_out) {
    DEBUG_PRINTF("reporting at offset %llu\n", report_offset);
    const struct mpv_kilopuff *kp = (const void *)(m + 1);
    u32 rl_count = 0;

    for (u32 i = mmbit_iterate(reporters, m->kilo_count, MMB_INVALID);
         i != MMB_INVALID; i = mmbit_iterate(reporters, m->kilo_count, i)) {
        const struct mpv_puffette *curr = dstate->active[i].curr;
        u64a curr_counter_val = *get_counter_for_kilo_c(dstate, &kp[i])
                              + counter_adj;
        DEBUG_PRINTF("kilo %u, underlying counter: %llu current: %llu\n", i,
                     *get_counter_for_kilo_c(dstate, &kp[i]), curr_counter_val);
        assert(curr_counter_val != MPV_DEAD_VALUE); /* counter_adj should take
                                                     * care if underlying value
                                                     * is -1 */
        char did_stuff = 0;

        while (curr->report != INVALID_REPORT) {
            assert(curr_counter_val >= curr->repeats);
            if (curr->unbounded || curr_counter_val == curr->repeats) {
                DEBUG_PRINTF("report %u at %llu\n", curr->report,
                              report_offset);

                if (curr->unbounded && !curr->simple_exhaust) {
                    assert(rl_count < m->puffette_count);
                    *rl = curr->report;
                    ++rl;
                    rl_count++;
                }

                if (cb(0, report_offset, curr->report, ctxt) ==
                    MO_HALT_MATCHING) {
                    DEBUG_PRINTF("bailing\n");
                    return MO_HALT_MATCHING;
                }
                did_stuff = 1;
            }

            curr--;
        }

        if (!did_stuff) {
            mmbit_unset(reporters, m->kilo_count, i);
        }
    }

    *rl_count_out = rl_count;
    return MO_CONTINUE_MATCHING;
}

static
ReportID *get_report_list(const struct mpv *m, struct mpv_decomp_state *s) {
    return (ReportID *)((char *)s + m->report_list_offset);
}

static really_inline
char processReportsForRange(const struct mpv *m, u8 *reporters,
                            struct mpv_decomp_state *dstate, u64a first_offset,
                            size_t length, NfaCallback cb, void *ctxt) {
    if (!length) {
        return MO_CONTINUE_MATCHING;
    }

    u64a counter_adj = dstate->counter_adj;
    u32 rl_count = 0;
    ReportID *rl = get_report_list(m, dstate);
    char rv = processReports(m, reporters, dstate, 1 + counter_adj,
                             first_offset + 1, cb, ctxt, rl, &rl_count);
    if (rv != MO_CONTINUE_MATCHING) {
        DEBUG_PRINTF("bailing\n");
        return rv;
    }
    if (!rl_count) {
        return MO_CONTINUE_MATCHING;
    }

    DEBUG_PRINTF("length=%zu, rl_count=%u\n", length, rl_count);

    for (size_t i = 2; i <= length; i++) {
        for (u32 j = 0; j < rl_count; j++) {
            if (cb(0, first_offset + i, rl[j], ctxt) == MO_HALT_MATCHING) {
                DEBUG_PRINTF("bailing\n");
                return MO_HALT_MATCHING;
            }
        }
    }

    return MO_CONTINUE_MATCHING;
}

/* returns last puffette that we have satisfied */
static
const struct mpv_puffette *get_curr_puff(const struct mpv *m,
                                         const struct mpv_kilopuff *kp,
                                         struct mpv_decomp_state *dstate) {
    u64a counter = *get_counter_for_kilo(dstate, kp);
    assert(counter != MPV_DEAD_VALUE);

    const struct mpv_puffette *p = get_puff_array(m, kp);
    DEBUG_PRINTF("looking for current puffette (counter = %llu)\n", counter);
    DEBUG_PRINTF("next: (%u, %u)\n", p->repeats, p->report);
    while (counter + 1 >= p->repeats && p->report != INVALID_REPORT) {
        DEBUG_PRINTF("advancing\n");
        ++p;
        DEBUG_PRINTF("next: (%u, %u)\n", p->repeats, p->report);
    }

    return p - 1;
}

static
const struct mpv_puffette *get_init_puff(const struct mpv *m,
                                         const struct mpv_kilopuff *kp) {
    const struct mpv_puffette *p = get_puff_array(m, kp);
    while (p->repeats == 1) {
        ++p;
    }
    return p - 1;
}


/* returns the last puffette whose repeats have been satisfied */
static really_inline
const struct mpv_puffette *update_curr_puff(const struct mpv *m, u8 *reporters,
                                            u64a counter,
                                            const struct mpv_puffette *in,
                                            u32 kilo_index) {
    assert(counter != MPV_DEAD_VALUE);

    const struct mpv_puffette *p = in;
    DEBUG_PRINTF("looking for current puffette (counter = %llu)\n", counter);
    DEBUG_PRINTF("curr: (%u, %u)\n", p->repeats, p->report);
    while (counter + 1 >= p[1].repeats && p[1].report != INVALID_REPORT) {
        DEBUG_PRINTF("advancing\n");
        ++p;
        DEBUG_PRINTF("curr: (%u, %u)\n", p->repeats, p->report);
    }

    if (p != in) {
        mmbit_set(reporters, m->kilo_count, kilo_index);
    }

    return p;
}

static really_inline
size_t limitByReach(const struct mpv_kilopuff *kp, const u8 *buf,
                    size_t length) {
    if (kp->type == MPV_VERM) {
        return vermicelliExec(kp->u.verm.c, 0, buf, buf + length) - buf;
    } else if (kp->type == MPV_SHUFTI) {
        m128 mask_lo = kp->u.shuf.mask_lo;
        m128 mask_hi = kp->u.shuf.mask_hi;
        return shuftiExec(mask_lo, mask_hi, buf, buf + length) - buf;
    } else if (kp->type == MPV_TRUFFLE) {
        return truffleExec(kp->u.truffle.mask1, kp->u.truffle.mask2, buf, buf + length) - buf;
    } else if (kp->type == MPV_NVERM) {
        return nvermicelliExec(kp->u.verm.c, 0, buf, buf + length) - buf;
    }

    assert(kp->type == MPV_DOT);
    return length;
}

static never_inline
void fillLimits(const struct mpv *m, u8 *active, u8 *reporters,
                struct mpv_decomp_state *dstate, struct mpv_pq_item *pq,
                const u8 *buf, size_t length) {
    DEBUG_PRINTF("filling limits %zu\n", length);
    assert(!dstate->pq_size);

    if (!length) {
        DEBUG_PRINTF("0 length\n");
        return;
    }

    const struct mpv_kilopuff *kp = (const void *)(m + 1);

    for (u32 i = mmbit_iterate(active, m->kilo_count, MMB_INVALID);
         i != MMB_INVALID; i = mmbit_iterate(active, m->kilo_count, i)) {
        dstate->active[i].curr = get_curr_puff(m, &kp[i], dstate);
        if (dstate->active[i].curr->report != INVALID_REPORT) {
            /* this kilo puff may fire reports */
            mmbit_set(reporters, m->kilo_count, i);
        }

        u64a lim = limitByReach(&kp[i], buf, length);
        DEBUG_PRINTF("lim %llu/%zu\n", lim, length);

        if (kp[i].dead_point != MPV_DEAD_VALUE) {
            assert(!kp[i].auto_restart);
            u64a counter = get_counter_value_for_kilo(dstate, &kp[i]);
            u64a dp_trigger = kp[i].dead_point - counter;
            if (dp_trigger < lim) {
                DEBUG_PRINTF("dead point trigger %llu\n", dp_trigger);
                lim = dp_trigger;
            }
        }

        if (kp[i].auto_restart && !lim) {
            *get_counter_for_kilo(dstate, &kp[i]) = MPV_DEAD_VALUE;
            mmbit_unset(reporters, m->kilo_count, i);
            /* the counter value will cause the nex_trigger calculation below to
             * adjust correctly */
            if (length == 1) {
                dstate->active[i].limit = 0;
                continue;
            }

            lim = limitByReach(&kp[i], buf + 1, length - 1) + 1;


            /* restart active counters */
            dstate->active[i].curr = get_init_puff(m, &kp[i]);
            assert(dstate->active[i].curr[0].report == INVALID_REPORT);

            DEBUG_PRINTF("lim now %llu/%zu\n", lim, length);
        }

        dstate->active[i].limit = lim;
        if (!lim) {
            mmbit_unset(active, m->kilo_count, i);
            mmbit_unset(reporters, m->kilo_count, i);
            continue;
        }
        if (dstate->active[i].curr[1].report != INVALID_REPORT) {
            u32 next_trigger = dstate->active[i].curr[1].repeats - 1ULL
                             - *get_counter_for_kilo(dstate, &kp[i]);
            DEBUG_PRINTF("next trigger %u\n", next_trigger);
            lim = MIN(lim, next_trigger);
        }

        if (lim != length) {
            struct mpv_pq_item temp = {
                .trigger_loc = lim,
                .kilo = i
            };

            DEBUG_PRINTF("push for %u at %llu\n", i, lim);
            pq_insert(pq, dstate->pq_size, temp);
            ++dstate->pq_size;
        }

        assert(lim || kp[i].auto_restart);
    }

    DEBUG_PRINTF("filled\n");
    dstate->filled = 1;
}

static never_inline
void handleTopN(const struct mpv *m, s64a loc, u8 *active, u8 *reporters,
                struct mpv_decomp_state *dstate, struct mpv_pq_item *pq,
                const u8 *buf, size_t length, u32 i) {
    assert(i < m->kilo_count);
    DEBUG_PRINTF("MQE_TOP + %u @%lld\n", i, loc);
    if (mmbit_set(active, m->kilo_count, i)) {
        DEBUG_PRINTF("kilo is already alive and kicking\n");
        return;
    }

    const struct mpv_kilopuff *kp = (const struct mpv_kilopuff *)(m + 1);

    assert(!kp[i].auto_restart); /* handle later/never */

    /* we need to ensure that the counters are upto date */
    normalize_counters(dstate, m);

    /* reset counter */
    *get_counter_for_kilo(dstate, &kp[i]) = 0;

    if ((size_t)loc == length) {
        /* end of buffer, just make sure it is active */
        dstate->active[i].limit = loc;
        dstate->active[i].curr = get_init_puff(m, &kp[i]);
        return;
    }

    /* find the limit */
    u64a lim = limitByReach(&kp[i], buf + loc, length - loc) + loc;

    /* no need to worry about dead_point triggers here as kilopuff must first
     * update chain (to fire a report) before it goes dead. */

    if (lim == (u64a)loc) {
        DEBUG_PRINTF("dead on arrival\n");
        mmbit_unset(active, m->kilo_count, i);
        return;
    }
    dstate->active[i].limit = lim;

    /* setup puffette, find next trigger */
    dstate->active[i].curr = get_init_puff(m, &kp[i]);
    if (dstate->active[i].curr[1].report != INVALID_REPORT) {
        u32 next_trigger = dstate->active[i].curr[1].repeats - 1ULL + loc;
        lim = MIN(lim, next_trigger);
    }

    assert(dstate->active[i].curr[0].repeats == 1
           || dstate->active[i].curr[0].report == INVALID_REPORT);
    if (dstate->active[i].curr[0].repeats == 1) {
        DEBUG_PRINTF("yippee\n");
        mmbit_set(reporters, m->kilo_count, i);
    }

    assert(lim > (u64a)loc);

    /* add to pq */
    if (lim != length) {
        struct mpv_pq_item temp = {
            .trigger_loc = lim,
            .kilo = i
        };

        DEBUG_PRINTF("push for %u at %llu\n", i, lim);
        pq_insert(pq, dstate->pq_size, temp);
        ++dstate->pq_size;
    }
}

static really_inline
void killKilo(const struct mpv *m, u8 *active, u8 *reporters,
              struct mpv_decomp_state *dstate, struct mpv_pq_item *pq, u32 i) {
    DEBUG_PRINTF("squashing kilo %u (progress %llu, limit %llu)\n",
                 i, pq_top(pq)->trigger_loc, dstate->active[i].limit);
    mmbit_unset(active, m->kilo_count, i);
    mmbit_unset(reporters, m->kilo_count, i);

    pq_pop(pq, dstate->pq_size);
    dstate->pq_size--;
}

static really_inline
void updateKiloChains(const struct mpv *m, u8 *reporters,
                      struct mpv_decomp_state *dstate, struct mpv_pq_item *pq,
                      u64a curr_loc, size_t buf_length, u32 i) {
    const struct mpv_kilopuff *kp = (const void *)(m + 1);
    u64a counter = get_counter_value_for_kilo(dstate, &kp[i]);

    DEBUG_PRINTF("updating active puff for kilo %u\n", i);
    dstate->active[i].curr = update_curr_puff(m, reporters, counter,
                                              dstate->active[i].curr, i);

    u64a next_trigger = dstate->active[i].limit;

    if (dstate->active[i].curr[1].report != INVALID_REPORT) {
        u64a next_rep_trigger = dstate->active[i].curr[1].repeats - 1 - counter
                             + curr_loc;

        next_trigger = MIN(next_trigger, next_rep_trigger);
    } else if (kp[i].dead_point != MPV_DEAD_VALUE) {
        u64a dp_trigger = kp[i].dead_point - counter + curr_loc;
        DEBUG_PRINTF("dead point trigger %llu\n", dp_trigger);
        if (dp_trigger < dstate->active[i].limit) {
            dstate->active[i].limit = dp_trigger;
            next_trigger = dp_trigger;
        }
    }

    DEBUG_PRINTF("next trigger location is %llu\n", next_trigger);

    if (next_trigger < buf_length) {
        assert(dstate->pq_size <= m->kilo_count);
        assert(next_trigger > pq_top(pq)->trigger_loc);
        struct mpv_pq_item temp = {
            .trigger_loc = next_trigger,
            .kilo = i
        };

        DEBUG_PRINTF("(replace) push for %u at %llu\n", i, next_trigger);
        pq_replace_top(pq, dstate->pq_size, temp);
    } else {
        pq_pop(pq, dstate->pq_size);
        dstate->pq_size--;
        DEBUG_PRINTF("PQ_POP\n");
    }
    DEBUG_PRINTF("pq size now %u next top %llu\n", dstate->pq_size,
                 pq_top(pq)->trigger_loc);
}

static really_inline
u8 do_single_shufti(const m128 l, const m128 h, u8 c) {
    const u8 *lo = (const u8 *)&l;
    const u8 *hi = (const u8 *)&h;
    return lo[c & 0xf] & hi[c >> 4];
}

static really_inline
size_t find_last_bad(const struct mpv_kilopuff *kp, const u8 *buf,
                     size_t length, size_t curr, u32 min_rep) {
    assert(kp->type != MPV_DOT);

    DEBUG_PRINTF("repeats = %u\n", min_rep);
    /* TODO: this should be replace by some sort of simd stuff */

    if (kp->type == MPV_VERM) {
        if (min_rep < MIN_SKIP_REPEAT) {
            return find_nverm_run(kp->u.verm.c, 0, min_rep, buf, buf + curr,
                                  buf + length) - buf - 1;
        }

    verm_restart:;
        assert(buf[curr] == kp->u.verm.c);
        size_t test = curr;
        if (curr + min_rep < length) {
            test = curr + min_rep;
        } else {
            test = length - 1;
        }

        while (test > curr) {
            if (buf[test] == kp->u.verm.c) {
                curr = test;
                if (curr == length - 1) {
                    return curr;
                }
                goto verm_restart;
            }
            --test;
        }
    } else if (kp->type == MPV_SHUFTI) {
        m128 lo = kp->u.shuf.mask_lo;
        m128 hi = kp->u.shuf.mask_hi;
    shuf_restart:
        assert(do_single_shufti(lo, hi, buf[curr]));
        size_t test = curr;
        if (curr + min_rep < length) {
            test = curr + min_rep;
        } else {
            test = length - 1;
        }

        while (test > curr) {
            if (do_single_shufti(lo, hi, buf[test])) {
                DEBUG_PRINTF("updating curr from %zu to %zu\n", curr, test);
                curr = test;
                if (curr == length - 1) {
                    return curr;
                }
                goto shuf_restart;
            }
            --test;
        }
    } else if (kp->type == MPV_TRUFFLE) {
        const m128 mask1 = kp->u.truffle.mask1;
        const m128 mask2 = kp->u.truffle.mask2;
    truffle_restart:;
        size_t test = curr;
        if (curr + min_rep < length) {
            test = curr + min_rep;
        } else {
            test = length - 1;
        }

        while (test > curr) {
            const u8 *rv = truffleExec(mask1, mask2, buf + test, buf + test + 1);
            if (rv == buf + test) {
                curr = test;
                if (curr == length - 1) {
                    return curr;
                }
                goto truffle_restart;
            }
            --test;
        }
    } else if (kp->type == MPV_NVERM) {
        if (min_rep < MIN_SKIP_REPEAT) {
            return find_verm_run(kp->u.verm.c, 0, min_rep, buf, buf + curr,
                                 buf + length) - buf - 1;
        }

    nverm_restart:;
        assert(buf[curr] != kp->u.verm.c);
        size_t test = curr;
        if (curr + min_rep < length) {
            test = curr + min_rep;
        } else {
            test = length - 1;
        }

        while (test > curr) {
            if (buf[test] != kp->u.verm.c) {
                curr = test;
                if (curr == length - 1) {
                    return curr;
                }
                goto nverm_restart;
            }
            --test;
        }
    } else {
        assert(0);
    }

    return curr;
}

static really_inline
void restartKilo(const struct mpv *m, UNUSED u8 *active, u8 *reporters,
                 struct mpv_decomp_state *dstate, struct mpv_pq_item *pq,
                 const u8 *buf, u64a prev_limit, size_t buf_length, u32 i) {
    const struct mpv_kilopuff *kp = (const void *)(m + 1);
    assert(kp[i].auto_restart);
    assert(mmbit_isset(active, m->kilo_count, i));

    DEBUG_PRINTF("we got to %llu,%llu\n", prev_limit, dstate->active[i].limit);
    assert(prev_limit == dstate->active[i].limit);

    DEBUG_PRINTF("resetting counter\n");

    /* we need to ensure that the counters are upto date */
    normalize_counters(dstate, m);

    /* current byte is dead, will wrap to 0 after processing this byte */
    assert(MPV_DEAD_VALUE + 1 == 0);
    *get_counter_for_kilo(dstate, &kp[i]) = MPV_DEAD_VALUE;

    DEBUG_PRINTF("resetting puffettes\n");
    dstate->active[i].curr = get_init_puff(m, &kp[i]);

    assert(dstate->active[i].curr[0].report == INVALID_REPORT);
    /* TODO: handle restart .{1,}s */

    mmbit_unset(reporters, m->kilo_count, i);

    if (prev_limit != buf_length - 1) {
        size_t last_bad = find_last_bad(&kp[i], buf, buf_length, prev_limit,
                                        dstate->active[i].curr[1].repeats);
        assert(last_bad >= prev_limit && last_bad < buf_length);
        if (last_bad != prev_limit) {
            /* there is no point in getting restarted at this location */
            dstate->active[i].limit = last_bad;
            assert(dstate->pq_size <= m->kilo_count);
            struct mpv_pq_item temp = {
                .trigger_loc = last_bad,
                .kilo = i
            };

            pq_replace_top(pq, dstate->pq_size, temp);
            return;
        }
    }

    /* TODO: skipping would really come in handy about now */
    u64a lim;
    if (buf_length > prev_limit + 1) {
        lim = limitByReach(&kp[i], buf + prev_limit + 1,
                           buf_length - (prev_limit + 1)) +
              prev_limit + 1;
    } else {
        assert(buf_length == prev_limit + 1);
        lim = buf_length;
    }
    DEBUG_PRINTF("next limit is %llu\n", lim);

    assert(lim > prev_limit);

    dstate->active[i].limit = lim;

    if (dstate->active[i].curr[1].report != INVALID_REPORT) {
        u32 next_trigger = dstate->active[i].curr[1].repeats + prev_limit;
        lim = MIN(lim, next_trigger);
    }

    DEBUG_PRINTF("next trigger for kilo at %llu\n", lim);

    if (lim < buf_length) {
        assert(dstate->pq_size <= m->kilo_count);
        assert(lim >= prev_limit);
        struct mpv_pq_item temp = {
            .trigger_loc = lim,
            .kilo = i
        };

        pq_replace_top(pq, dstate->pq_size, temp);
    } else {
        pq_pop(pq, dstate->pq_size);
        dstate->pq_size--;
    }
}

static really_inline
void handle_events(const struct mpv *m, u8 *active, u8 *reporters,
                   struct mpv_decomp_state *dstate, struct mpv_pq_item *pq,
                   u64a loc,  const u8 *buf, size_t buf_length) {
    const struct mpv_kilopuff *kp = (const void *)(m + 1);

    while (dstate->pq_size && pq_top(pq)->trigger_loc <= loc) {
        assert(pq_top(pq)->trigger_loc == loc);

        u32 kilo = pq_top(pq)->kilo;

        DEBUG_PRINTF("pop for kilo %u at %llu\n", kilo,
                     pq_top(pq)->trigger_loc);

        if (dstate->active[kilo].limit <= loc) {
           if (!kp[kilo].auto_restart) {
                killKilo(m, active, reporters, dstate, pq, kilo);
            } else {
                restartKilo(m, active, reporters, dstate, pq, buf, loc,
                            buf_length, kilo);
            }
        } else {
            updateKiloChains(m, reporters, dstate, pq, loc, buf_length, kilo);
        }
    }
}

static really_inline
u64a find_next_limit(const struct mpv *m, u8 *active, u8 *reporters,
                     struct mpv_decomp_state *dstate, struct mpv_pq_item *pq,
                     const u8 *buf, u64a prev_limit, u64a ep,
                     size_t buf_length) {
    u64a limit = ep;

    DEBUG_PRINTF("length %llu (prev %llu), pq %u\n", limit, prev_limit,
                 dstate->pq_size);

    handle_events(m, active, reporters, dstate, pq, prev_limit, buf,
                  buf_length);

    if (dstate->pq_size) {
        limit = MIN(pq_top(pq)->trigger_loc, limit);
        assert(limit > prev_limit);
    }

    DEBUG_PRINTF("limit now %llu\n", limit);
    return limit;
}

static really_inline
char mpvExec(const struct mpv *m, u8 *active, u8 *reporters,
             struct mpv_decomp_state *dstate, struct mpv_pq_item *pq,
             const u8 *buf, s64a start, size_t length, size_t buf_length,
             u64a offsetAdj, NfaCallback cb, void *ctxt) {
    DEBUG_PRINTF("running mpv (s %lliu, l %zu, o %llu)\n",
                 *get_counter_n(dstate, m, 0) + dstate->counter_adj, length,
                 offsetAdj);

    u64a progress = start; /* progress is relative to buffer offsets */

    while (progress < length) {
        DEBUG_PRINTF("progress %llu\n", progress);

        /* find next limit and update chains */
        u64a limit = find_next_limit(m, active, reporters, dstate, pq, buf,
                                     progress, length, buf_length);
        assert(limit != progress);
        u64a incr = limit - progress;
        DEBUG_PRINTF("incr = %llu\n", incr);

        /* report matches upto next limit */
        char rv = processReportsForRange(m, reporters, dstate,
                                         offsetAdj + progress, limit - progress,
                                         cb, ctxt);

        if (rv != MO_CONTINUE_MATCHING) {
            DEBUG_PRINTF("mpvExec done %llu/%zu\n", progress, length);
            return rv;
        }

        dstate->counter_adj += incr;
        progress = limit;
    }

    assert(progress == length);

    DEBUG_PRINTF("mpvExec done\n");
    return MO_CONTINUE_MATCHING;
}

static really_inline
void mpvLoadState(struct mpv_decomp_state *out, const struct NFA *n,
                  const char *state) {
    assert(16 >= sizeof(struct mpv_decomp_kilo));
    assert(sizeof(*out) <= n->scratchStateSize);
    assert(ISALIGNED(out));

    const struct mpv *m = getImplNfa(n);
    const struct mpv_counter_info *counter_info = get_counter_info(m);
    u64a *counters = get_counter_n(out, m, 0);
    const char *comp_counter = state;
    for (u32 i = 0; i < m->counter_count; i++) {
        u32 counter_size = counter_info[i].counter_size;
        counters[i] = partial_load_u64a(comp_counter, counter_size);
        DEBUG_PRINTF("loaded %llu counter %u\n", counters[i], i);
        comp_counter += counter_size;
    }

    out->filled = 0; /* _Q_i will fill limits, curr puffetes, and populate pq
                      * on first call */
    out->counter_adj = 0;
    out->pq_size = 0;

    u8 *reporters = (u8 *)out + m->reporter_offset;

    mmbit_clear(reporters, m->kilo_count);
}

static really_inline
void mpvStoreState(const struct NFA *n, char *state,
                   const struct mpv_decomp_state *in) {
    assert(ISALIGNED(in));
    const struct mpv *m = getImplNfa(n);
    const struct mpv_counter_info *counter_info = get_counter_info(m);

    const u64a *counters = (const u64a *)((const char *)in
                                       + get_counter_info(m)[0].counter_offset);
    u64a adj = in->counter_adj;
    char *comp_counter = state;
    for (u32 i = 0; i < m->counter_count; i++) {
        /* clamp counter to allow storage in smaller ints */
        u64a curr_counter = MIN(counters[i] + adj, counter_info[i].max_counter);

        u32 counter_size = counter_info[i].counter_size;
        partial_store_u64a(comp_counter, curr_counter, counter_size);
        DEBUG_PRINTF("stored %llu counter %u (orig %llu)\n", curr_counter, i,
                     counters[i]);
        /* assert(counters[i] != MPV_DEAD_VALUE); /\* should have process 1 byte */
        /*                                         * since a clear *\/ */
        comp_counter += counter_size;
    }
}

char nfaExecMpv_queueCompressState(const struct NFA *nfa, const struct mq *q,
                                   UNUSED s64a loc) {
    void *dest = q->streamState;
    const void *src = q->state;
    mpvStoreState(nfa, dest, src);
    return 0;
}

char nfaExecMpv_expandState(const struct NFA *nfa, void *dest, const void *src,
                            UNUSED u64a offset, UNUSED u8 key) {
    mpvLoadState(dest, nfa, src);
    return 0;
}

char nfaExecMpv_reportCurrent(const struct NFA *n, struct mq *q) {
    const struct mpv *m = getImplNfa(n);
    u64a offset = q_cur_offset(q);
    struct mpv_decomp_state *s = (struct mpv_decomp_state *)q->state;

    DEBUG_PRINTF("report current: offset %llu\n", offset);

    u8 *active = (u8 *)q->streamState + m->active_offset;
    u32 rl_count = 0;
    ReportID *rl = get_report_list(m, s);

    processReports(m, active, s, s->counter_adj, offset, q->cb, q->context, rl,
                   &rl_count);
    return 0;
}

char nfaExecMpv_queueInitState(const struct NFA *n, struct mq *q) {
    struct mpv_decomp_state *out = (void *)q->state;
    const struct mpv *m = getImplNfa(n);
    assert(sizeof(*out) <= n->scratchStateSize);

    DEBUG_PRINTF("queue init state\n");

    u64a *counters = get_counter_n(out, m, 0);
    for (u32 i = 0; i < m->counter_count; i++) {
        counters[i] = MPV_DEAD_VALUE;
    }

    out->filled = 0;
    out->counter_adj = 0;
    out->pq_size = 0;
    out->active[0].curr = NULL;

    assert(q->streamState);
    u8 *active_kpuff = (u8 *)q->streamState + m->active_offset;
    u8 *reporters = (u8 *)q->state + m->reporter_offset;
    mmbit_clear(active_kpuff, m->kilo_count);
    mmbit_clear(reporters, m->kilo_count);
    return 0;
}

char nfaExecMpv_initCompressedState(const struct NFA *n, u64a offset,
                                    void *state, UNUSED u8 key) {
    const struct mpv *m = getImplNfa(n);
    memset(state, 0, m->active_offset); /* active_offset marks end of comp
                                         * counters */
    u8 *active_kpuff = (u8 *)state + m->active_offset;
    if (!offset) {
        mmbit_init_range(active_kpuff, m->kilo_count, m->top_kilo_begin,
                         m->top_kilo_end);
        return 1;
    } else {
        return 0;
    }
}

static really_inline
char nfaExecMpv_Q_i(const struct NFA *n, struct mq *q, s64a end) {
    u64a offset = q->offset;
    const u8 *buffer = q->buffer;
    size_t length = q->length;
    NfaCallback cb = q->cb;
    void *context = q->context;
    s64a sp;
    const struct mpv *m = getImplNfa(n);
    struct mpv_decomp_state *s = (struct mpv_decomp_state *)q->state;
    u8 *active = (u8 *)q->streamState + m->active_offset;
    u8 *reporters = (u8 *)q->state + m->reporter_offset;
    struct mpv_pq_item *pq = (struct mpv_pq_item *)(q->state + m->pq_offset);

    if (!s->filled) {
        fillLimits(m, active, reporters, s, pq, q->buffer, q->length);
    }

    assert(!q->report_current);

    if (q->cur == q->end) {
        return 1;
    }

    assert(q->cur + 1 < q->end); /* require at least two items */

    assert(q_cur_type(q) == MQE_START);
    assert(q_cur_loc(q) >= 0);
    sp = q->items[q->cur].location;
    q->cur++;

    if (q->items[q->cur - 1].location > end) {
        /* this is as far as we go */
        q->cur--;
        q->items[q->cur].type = MQE_START;
        q->items[q->cur].location = end;
        return MO_ALIVE;
    }

    while (q->cur < q->end) {
        s64a ep = q->items[q->cur].location;

        ep = MIN(ep, end);

        assert(ep >= sp);

        assert(sp >= 0); /* mpv should be an outfix; outfixes are not lazy */

        if (sp >= ep) {
            goto scan_done;
        }

        /* do main buffer region */
        assert((u64a)ep <= length);
        char rv = mpvExec(m, active, reporters, s, pq, buffer, sp, ep, length,
                          offset, cb, context);
        if (rv == MO_HALT_MATCHING) {
            q->cur = q->end;
            return 0;
        }

    scan_done:
        if (q->items[q->cur].location > end) {
            /* this is as far as we go */
            q->cur--;
            q->items[q->cur].type = MQE_START;
            q->items[q->cur].location = end;
            return MO_ALIVE;
        }

        sp = ep;

        switch (q->items[q->cur].type) {
        case MQE_TOP:
            DEBUG_PRINTF("top %u %u\n", m->top_kilo_begin, m->top_kilo_end);
            /* MQE_TOP initialise all counters to 0; activates all kilos */
            {
                u64a *counters = get_counter_n(s, m, 0);
                assert(counters[0] == MPV_DEAD_VALUE);
                assert(!s->counter_adj);
                for (u32 i = 0; i < m->counter_count; i++) {
                    counters[i] = 0;
                }
                mmbit_init_range(active, m->kilo_count, m->top_kilo_begin,
                                 m->top_kilo_end);
                fillLimits(m, active, reporters, s, pq, buffer, length);
            }
            break;
        case MQE_START:
        case MQE_END:
            break;
        default:
            /* MQE_TOP_N --> switch on kilo puff N */
            assert(q->items[q->cur].type >= MQE_TOP_FIRST);
            assert(q->items[q->cur].type < MQE_INVALID);
            u32 i = q->items[q->cur].type - MQE_TOP_FIRST;
            handleTopN(m, sp, active, reporters, s, pq, buffer, length, i);
            break;
        }

        q->cur++;
    }

    char alive = 0;
    assert(q->items[q->cur - 1].type == MQE_END);
    if (q->items[q->cur - 1].location == (s64a)q->length) {
        normalize_counters(s, m);

        const struct mpv_kilopuff *kp = (const struct mpv_kilopuff *)(m + 1);
        for (u32 i = mmbit_iterate(active, m->kilo_count, MMB_INVALID);
             i != MMB_INVALID; i = mmbit_iterate(active, m->kilo_count, i)) {
            if (*get_counter_for_kilo(s, &kp[i]) >= kp[i].dead_point) {
                mmbit_unset(active, m->kilo_count, i);
            } else {
                alive = 1;
            }
        }
    } else {
        alive
            = mmbit_iterate(active, m->kilo_count, MMB_INVALID) != MMB_INVALID;
    }

    DEBUG_PRINTF("finished %d\n", (int)alive);
    return alive;
}

char nfaExecMpv_Q(const struct NFA *n, struct mq *q, s64a end) {
    DEBUG_PRINTF("_Q %lld\n", end);
    return nfaExecMpv_Q_i(n, q, end);
}

s64a nfaExecMpv_QueueExecRaw(const struct NFA *nfa, struct mq *q, s64a end) {
    DEBUG_PRINTF("nfa=%p end=%lld\n", nfa, end);
#ifdef DEBUG
    debugQueue(q);
#endif

    assert(nfa->type == MPV_NFA);
    assert(q && q->context && q->state);
    assert(end >= 0);
    assert(q->cur < q->end);
    assert(q->end <= MAX_MQE_LEN);
    assert(ISALIGNED_16(nfa) && ISALIGNED_16(getImplNfa(nfa)));
    assert(end < q->items[q->end - 1].location
           || q->items[q->end - 1].type == MQE_END);

    if (q->items[q->cur].location > end) {
        return 1;
    }

    char q_trimmed = 0;

    assert(end <= (s64a)q->length || !q->hlength);
    /* due to reverse accel in block mode some queues may work on a truncated
     * buffer */
    if (end > (s64a)q->length) {
        end = q->length;
        q_trimmed = 1;
    }

    /* TODO: restore max offset stuff, if/when _interesting_ max offset stuff
     * is filled in */

    char rv = nfaExecMpv_Q_i(nfa, q, end);

    assert(!q->report_current);
    DEBUG_PRINTF("returned rv=%d, q_trimmed=%d\n", rv, q_trimmed);
    if (q_trimmed || !rv) {
        return 0;
    } else {
        const struct mpv *m = getImplNfa(nfa);
        u8 *reporters = (u8 *)q->state + m->reporter_offset;

        if (mmbit_any_precise(reporters, m->kilo_count)) {
            DEBUG_PRINTF("next byte\n");
            return 1; /* need to match at next byte */
        } else {
            s64a next_event = q->length;
            s64a next_pq = q->length;

            if (q->cur < q->end) {
                next_event = q->items[q->cur].location;
            }

            struct mpv_decomp_state *s = (struct mpv_decomp_state *)q->state;
            struct mpv_pq_item *pq
                = (struct mpv_pq_item *)(q->state + m->pq_offset);
            if (s->pq_size) {
                next_pq = pq_top(pq)->trigger_loc;
            }

            assert(next_event);
            assert(next_pq);

            DEBUG_PRINTF("next pq %lld event %lld\n", next_pq, next_event);
            return MIN(next_pq, next_event);
        }
    }
}
