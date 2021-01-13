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

/*
 * In order to use this macro, the following things need to be defined:
 *
 *  - SHENG_IMPL        (name of the Sheng implementation function)
 *  - INTERESTING_FUNC  (name of the function checking for accept, accel or dead
 *                       states)
 *  - INNER_DEAD_FUNC   (name of the inner function checking for dead states)
 *  - OUTER_DEAD_FUNC   (name of the outer function checking for dead states)
 *  - INNER_ACCEL_FUNC  (name of the inner function checking for accel states)
 *  - OUTER_ACCEL_FUNC  (name of the outer function checking for accel states)
 *  - ACCEPT_FUNC       (name of the function checking for accept state)
 *  - STOP_AT_MATCH     (can be 1 or 0, enable or disable stop at match)
 */

/* unrolled 4-byte-at-a-time version.
 *
 * we put innerDeadFunc inside interestingFunc() block so that we don't pay for
 * dead states checking. however, if interestingFunc is dummy, innerDeadFunc
 * gets lost with it, so we need an additional check outside the
 * interestingFunc() branch - it's normally dummy so we don't pay for it, but
 * when interestingFunc is dummy, outerDeadFunc should be set if we want to
 * check for dead states.
 *
 * also, deadFunc only checks the last known state, but since we can't ever get
 * out of the dead state and we don't really care where we died, it's not a
 * problem.
 */
static really_inline
char SHENG_IMPL(u8 *state, NfaCallback cb, void *ctxt, const struct sheng *s,
                u8 *const cached_accept_state, ReportID *const cached_accept_id,
                u8 single, u64a base_offset, const u8 *buf, const u8 *start,
                const u8 *end, const u8 **scan_end) {
    DEBUG_PRINTF("Starting DFAx4 execution in state %u\n",
                 *state & SHENG_STATE_MASK);
    const u8 *cur_buf = start;
    const u8 *min_accel_dist = start;
    base_offset++;
    DEBUG_PRINTF("Scanning %llu bytes\n", (u64a)(end - start));

    if (INNER_ACCEL_FUNC(*state) || OUTER_ACCEL_FUNC(*state)) {
        DEBUG_PRINTF("Accel state reached @ 0\n");
        const union AccelAux *aaux = get_accel(s, *state & SHENG_STATE_MASK);
        const u8 *new_offset = run_accel(aaux, cur_buf, end);
        if (new_offset < cur_buf + BAD_ACCEL_DIST) {
            min_accel_dist = new_offset + BIG_ACCEL_PENALTY;
        } else {
            min_accel_dist = new_offset + SMALL_ACCEL_PENALTY;
        }
        DEBUG_PRINTF("Next accel chance: %llu\n",
                     (u64a)(min_accel_dist - start));
        DEBUG_PRINTF("Accel scanned %zu bytes\n", new_offset - cur_buf);
        cur_buf = new_offset;
        DEBUG_PRINTF("New offset: %lli\n", (s64a)(cur_buf - start));
    }
    if (INNER_DEAD_FUNC(*state) || OUTER_DEAD_FUNC(*state)) {
        DEBUG_PRINTF("Dead on arrival\n");
        *scan_end = end;
        return MO_CONTINUE_MATCHING;
    }

    m128 cur_state = set16x8(*state);
    const m128 *masks = s->shuffle_masks;

    while (likely(end - cur_buf >= 4)) {
        const u8 *b1 = cur_buf;
        const u8 *b2 = cur_buf + 1;
        const u8 *b3 = cur_buf + 2;
        const u8 *b4 = cur_buf + 3;
        const u8 c1 = *b1;
        const u8 c2 = *b2;
        const u8 c3 = *b3;
        const u8 c4 = *b4;

        const m128 shuffle_mask1 = masks[c1];
        cur_state = pshufb_m128(shuffle_mask1, cur_state);
        const u8 a1 = movd(cur_state);

        const m128 shuffle_mask2 = masks[c2];
        cur_state = pshufb_m128(shuffle_mask2, cur_state);
        const u8 a2 = movd(cur_state);

        const m128 shuffle_mask3 = masks[c3];
        cur_state = pshufb_m128(shuffle_mask3, cur_state);
        const u8 a3 = movd(cur_state);

        const m128 shuffle_mask4 = masks[c4];
        cur_state = pshufb_m128(shuffle_mask4, cur_state);
        const u8 a4 = movd(cur_state);

        DEBUG_PRINTF("c: %02hhx '%c'\n", c1, ourisprint(c1) ? c1 : '?');
        DEBUG_PRINTF("s: %u (hi: %u lo: %u)\n", a1, (a1 & 0xF0) >> 4, a1 & 0xF);

        DEBUG_PRINTF("c: %02hhx '%c'\n", c2, ourisprint(c2) ? c2 : '?');
        DEBUG_PRINTF("s: %u (hi: %u lo: %u)\n", a2, (a2 & 0xF0) >> 4, a2 & 0xF);

        DEBUG_PRINTF("c: %02hhx '%c'\n", c3, ourisprint(c3) ? c3 : '?');
        DEBUG_PRINTF("s: %u (hi: %u lo: %u)\n", a3, (a3 & 0xF0) >> 4, a3 & 0xF);

        DEBUG_PRINTF("c: %02hhx '%c'\n", c4, ourisprint(c4) ? c4 : '?');
        DEBUG_PRINTF("s: %u (hi: %u lo: %u)\n", a4, (a4 & 0xF0) >> 4, a4 & 0xF);

        if (unlikely(INTERESTING_FUNC(a1, a2, a3, a4))) {
            if (ACCEPT_FUNC(a1)) {
                u64a match_offset = base_offset + b1 - buf;
                DEBUG_PRINTF("Accept state %u reached\n",
                             a1 & SHENG_STATE_MASK);
                DEBUG_PRINTF("Match @ %llu\n", match_offset);
                if (STOP_AT_MATCH) {
                    DEBUG_PRINTF("Stopping at match @ %lli\n",
                                 (s64a)(b1 - start));
                    *scan_end = b1;
                    *state = a1;
                    return MO_MATCHES_PENDING;
                }
                if (single) {
                    if (fireSingleReport(cb, ctxt, s->report, match_offset) ==
                        MO_HALT_MATCHING) {
                        return MO_HALT_MATCHING;
                    }
                } else {
                    if (fireReports(s, cb, ctxt, a1, match_offset,
                                    cached_accept_state, cached_accept_id,
                                    0) == MO_HALT_MATCHING) {
                        return MO_HALT_MATCHING;
                    }
                }
            }
            if (ACCEPT_FUNC(a2)) {
                u64a match_offset = base_offset + b2 - buf;
                DEBUG_PRINTF("Accept state %u reached\n",
                             a2 & SHENG_STATE_MASK);
                DEBUG_PRINTF("Match @ %llu\n", match_offset);
                if (STOP_AT_MATCH) {
                    DEBUG_PRINTF("Stopping at match @ %lli\n",
                                 (s64a)(b2 - start));
                    *scan_end = b2;
                    *state = a2;
                    return MO_MATCHES_PENDING;
                }
                if (single) {
                    if (fireSingleReport(cb, ctxt, s->report, match_offset) ==
                        MO_HALT_MATCHING) {
                        return MO_HALT_MATCHING;
                    }
                } else {
                    if (fireReports(s, cb, ctxt, a2, match_offset,
                                    cached_accept_state, cached_accept_id,
                                    0) == MO_HALT_MATCHING) {
                        return MO_HALT_MATCHING;
                    }
                }
            }
            if (ACCEPT_FUNC(a3)) {
                u64a match_offset = base_offset + b3 - buf;
                DEBUG_PRINTF("Accept state %u reached\n",
                             a3 & SHENG_STATE_MASK);
                DEBUG_PRINTF("Match @ %llu\n", match_offset);
                if (STOP_AT_MATCH) {
                    DEBUG_PRINTF("Stopping at match @ %lli\n",
                                 (s64a)(b3 - start));
                    *scan_end = b3;
                    *state = a3;
                    return MO_MATCHES_PENDING;
                }
                if (single) {
                    if (fireSingleReport(cb, ctxt, s->report, match_offset) ==
                        MO_HALT_MATCHING) {
                        return MO_HALT_MATCHING;
                    }
                } else {
                    if (fireReports(s, cb, ctxt, a3, match_offset,
                                    cached_accept_state, cached_accept_id,
                                    0) == MO_HALT_MATCHING) {
                        return MO_HALT_MATCHING;
                    }
                }
            }
            if (ACCEPT_FUNC(a4)) {
                u64a match_offset = base_offset + b4 - buf;
                DEBUG_PRINTF("Accept state %u reached\n",
                             a4 & SHENG_STATE_MASK);
                DEBUG_PRINTF("Match @ %llu\n", match_offset);
                if (STOP_AT_MATCH) {
                    DEBUG_PRINTF("Stopping at match @ %lli\n",
                                 (s64a)(b4 - start));
                    *scan_end = b4;
                    *state = a4;
                    return MO_MATCHES_PENDING;
                }
                if (single) {
                    if (fireSingleReport(cb, ctxt, s->report, match_offset) ==
                        MO_HALT_MATCHING) {
                        return MO_HALT_MATCHING;
                    }
                } else {
                    if (fireReports(s, cb, ctxt, a4, match_offset,
                                    cached_accept_state, cached_accept_id,
                                    0) == MO_HALT_MATCHING) {
                        return MO_HALT_MATCHING;
                    }
                }
            }
            if (INNER_DEAD_FUNC(a4)) {
                DEBUG_PRINTF("Dead state reached @ %lli\n", (s64a)(b4 - buf));
                *scan_end = end;
                *state = a4;
                return MO_CONTINUE_MATCHING;
            }
            if (cur_buf > min_accel_dist && INNER_ACCEL_FUNC(a4)) {
                DEBUG_PRINTF("Accel state reached @ %lli\n", (s64a)(b4 - buf));
                const union AccelAux *aaux =
                    get_accel(s, a4 & SHENG_STATE_MASK);
                const u8 *new_offset = run_accel(aaux, cur_buf + 4, end);
                if (new_offset < cur_buf + 4 + BAD_ACCEL_DIST) {
                    min_accel_dist = new_offset + BIG_ACCEL_PENALTY;
                } else {
                    min_accel_dist = new_offset + SMALL_ACCEL_PENALTY;
                }
                DEBUG_PRINTF("Next accel chance: %llu\n",
                             (u64a)(min_accel_dist - start));
                DEBUG_PRINTF("Accel scanned %llu bytes\n",
                             (u64a)(new_offset - cur_buf - 4));
                cur_buf = new_offset;
                DEBUG_PRINTF("New offset: %llu\n", (u64a)(cur_buf - buf));
                continue;
            }
        }
        if (OUTER_DEAD_FUNC(a4)) {
            DEBUG_PRINTF("Dead state reached @ %lli\n", (s64a)(cur_buf - buf));
            *scan_end = end;
            *state = a4;
            return MO_CONTINUE_MATCHING;
        };
        if (cur_buf > min_accel_dist && OUTER_ACCEL_FUNC(a4)) {
            DEBUG_PRINTF("Accel state reached @ %lli\n", (s64a)(b4 - buf));
            const union AccelAux *aaux = get_accel(s, a4 & SHENG_STATE_MASK);
            const u8 *new_offset = run_accel(aaux, cur_buf + 4, end);
            if (new_offset < cur_buf + 4 + BAD_ACCEL_DIST) {
                min_accel_dist = new_offset + BIG_ACCEL_PENALTY;
            } else {
                min_accel_dist = new_offset + SMALL_ACCEL_PENALTY;
            }
            DEBUG_PRINTF("Next accel chance: %llu\n",
                         (u64a)(min_accel_dist - start));
            DEBUG_PRINTF("Accel scanned %llu bytes\n",
                         (u64a)(new_offset - cur_buf - 4));
            cur_buf = new_offset;
            DEBUG_PRINTF("New offset: %llu\n", (u64a)(cur_buf - buf));
            continue;
        };
        cur_buf += 4;
    }
    *state = movd(cur_state);
    *scan_end = cur_buf;
    return MO_CONTINUE_MATCHING;
}

#if defined(HAVE_AVX512VBMI)
static really_inline
char SHENG32_IMPL(u8 *state, NfaCallback cb, void *ctxt,
                  const struct sheng32 *s,
                  u8 *const cached_accept_state,
                  ReportID *const cached_accept_id,
                  u8 single, u64a base_offset, const u8 *buf, const u8 *start,
                  const u8 *end, const u8 **scan_end) {
    DEBUG_PRINTF("Starting DFAx4 execution in state %u\n",
                 *state & SHENG32_STATE_MASK);
    const u8 *cur_buf = start;
    const u8 *min_accel_dist = start;
    base_offset++;
    DEBUG_PRINTF("Scanning %llu bytes\n", (u64a)(end - start));

    if (INNER_ACCEL_FUNC32(*state) || OUTER_ACCEL_FUNC32(*state)) {
        DEBUG_PRINTF("Accel state reached @ 0\n");
        const union AccelAux *aaux =
            get_accel32(s, *state & SHENG32_STATE_MASK);
        const u8 *new_offset = run_accel(aaux, cur_buf, end);
        if (new_offset < cur_buf + BAD_ACCEL_DIST) {
            min_accel_dist = new_offset + BIG_ACCEL_PENALTY;
        } else {
            min_accel_dist = new_offset + SMALL_ACCEL_PENALTY;
        }
        DEBUG_PRINTF("Next accel chance: %llu\n",
                     (u64a)(min_accel_dist - start));
        DEBUG_PRINTF("Accel scanned %zu bytes\n", new_offset - cur_buf);
        cur_buf = new_offset;
        DEBUG_PRINTF("New offset: %lli\n", (s64a)(cur_buf - start));
    }
    if (INNER_DEAD_FUNC32(*state) || OUTER_DEAD_FUNC32(*state)) {
        DEBUG_PRINTF("Dead on arrival\n");
        *scan_end = end;
        return MO_CONTINUE_MATCHING;
    }

    m512 cur_state = set64x8(*state);
    const m512 *masks = s->succ_masks;

    while (likely(end - cur_buf >= 4)) {
        const u8 *b1 = cur_buf;
        const u8 *b2 = cur_buf + 1;
        const u8 *b3 = cur_buf + 2;
        const u8 *b4 = cur_buf + 3;
        const u8 c1 = *b1;
        const u8 c2 = *b2;
        const u8 c3 = *b3;
        const u8 c4 = *b4;

        const m512 succ_mask1 = masks[c1];
        cur_state = vpermb512(cur_state, succ_mask1);
        const u8 a1 = movd512(cur_state);

        const m512 succ_mask2 = masks[c2];
        cur_state = vpermb512(cur_state, succ_mask2);
        const u8 a2 = movd512(cur_state);

        const m512 succ_mask3 = masks[c3];
        cur_state = vpermb512(cur_state, succ_mask3);
        const u8 a3 = movd512(cur_state);

        const m512 succ_mask4 = masks[c4];
        cur_state = vpermb512(cur_state, succ_mask4);
        const u8 a4 = movd512(cur_state);

        DEBUG_PRINTF("c: %02hhx '%c'\n", c1, ourisprint(c1) ? c1 : '?');
        DEBUG_PRINTF("s: %u (flag: %u)\n", a1 & SHENG32_STATE_MASK,
                     a1 & SHENG32_STATE_FLAG_MASK);

        DEBUG_PRINTF("c: %02hhx '%c'\n", c2, ourisprint(c2) ? c2 : '?');
        DEBUG_PRINTF("s: %u (flag: %u)\n", a2 & SHENG32_STATE_MASK,
                     a2 & SHENG32_STATE_FLAG_MASK);

        DEBUG_PRINTF("c: %02hhx '%c'\n", c3, ourisprint(c3) ? c3 : '?');
        DEBUG_PRINTF("s: %u (flag: %u)\n", a3 & SHENG32_STATE_MASK,
                     a3 & SHENG32_STATE_FLAG_MASK);

        DEBUG_PRINTF("c: %02hhx '%c'\n", c4, ourisprint(c4) ? c4 : '?');
        DEBUG_PRINTF("s: %u (flag: %u)\n", a4 & SHENG32_STATE_MASK,
                     a4 & SHENG32_STATE_FLAG_MASK);

        if (unlikely(INTERESTING_FUNC32(a1, a2, a3, a4))) {
            if (ACCEPT_FUNC32(a1)) {
                u64a match_offset = base_offset + b1 - buf;
                DEBUG_PRINTF("Accept state %u reached\n",
                             a1 & SHENG32_STATE_MASK);
                DEBUG_PRINTF("Match @ %llu\n", match_offset);
                if (STOP_AT_MATCH) {
                    DEBUG_PRINTF("Stopping at match @ %lli\n",
                                 (s64a)(b1 - start));
                    *scan_end = b1;
                    *state = a1;
                    return MO_MATCHES_PENDING;
                }
                if (single) {
                    if (fireSingleReport(cb, ctxt, s->report, match_offset) ==
                        MO_HALT_MATCHING) {
                        return MO_HALT_MATCHING;
                    }
                } else {
                    if (fireReports32(s, cb, ctxt, a1, match_offset,
                                      cached_accept_state, cached_accept_id,
                                      0) == MO_HALT_MATCHING) {
                        return MO_HALT_MATCHING;
                    }
                }
            }
            if (ACCEPT_FUNC32(a2)) {
                u64a match_offset = base_offset + b2 - buf;
                DEBUG_PRINTF("Accept state %u reached\n",
                             a2 & SHENG32_STATE_MASK);
                DEBUG_PRINTF("Match @ %llu\n", match_offset);
                if (STOP_AT_MATCH) {
                    DEBUG_PRINTF("Stopping at match @ %lli\n",
                                 (s64a)(b2 - start));
                    *scan_end = b2;
                    *state = a2;
                    return MO_MATCHES_PENDING;
                }
                if (single) {
                    if (fireSingleReport(cb, ctxt, s->report, match_offset) ==
                        MO_HALT_MATCHING) {
                        return MO_HALT_MATCHING;
                    }
                } else {
                    if (fireReports32(s, cb, ctxt, a2, match_offset,
                                      cached_accept_state, cached_accept_id,
                                      0) == MO_HALT_MATCHING) {
                        return MO_HALT_MATCHING;
                    }
                }
            }
            if (ACCEPT_FUNC32(a3)) {
                u64a match_offset = base_offset + b3 - buf;
                DEBUG_PRINTF("Accept state %u reached\n",
                             a3 & SHENG32_STATE_MASK);
                DEBUG_PRINTF("Match @ %llu\n", match_offset);
                if (STOP_AT_MATCH) {
                    DEBUG_PRINTF("Stopping at match @ %lli\n",
                                 (s64a)(b3 - start));
                    *scan_end = b3;
                    *state = a3;
                    return MO_MATCHES_PENDING;
                }
                if (single) {
                    if (fireSingleReport(cb, ctxt, s->report, match_offset) ==
                        MO_HALT_MATCHING) {
                        return MO_HALT_MATCHING;
                    }
                } else {
                    if (fireReports32(s, cb, ctxt, a3, match_offset,
                                      cached_accept_state, cached_accept_id,
                                      0) == MO_HALT_MATCHING) {
                        return MO_HALT_MATCHING;
                    }
                }
            }
            if (ACCEPT_FUNC32(a4)) {
                u64a match_offset = base_offset + b4 - buf;
                DEBUG_PRINTF("Accept state %u reached\n",
                             a4 & SHENG32_STATE_MASK);
                DEBUG_PRINTF("Match @ %llu\n", match_offset);
                if (STOP_AT_MATCH) {
                    DEBUG_PRINTF("Stopping at match @ %lli\n",
                                 (s64a)(b4 - start));
                    *scan_end = b4;
                    *state = a4;
                    return MO_MATCHES_PENDING;
                }
                if (single) {
                    if (fireSingleReport(cb, ctxt, s->report, match_offset) ==
                        MO_HALT_MATCHING) {
                        return MO_HALT_MATCHING;
                    }
                } else {
                    if (fireReports32(s, cb, ctxt, a4, match_offset,
                                      cached_accept_state, cached_accept_id,
                                      0) == MO_HALT_MATCHING) {
                        return MO_HALT_MATCHING;
                    }
                }
            }
            if (INNER_DEAD_FUNC32(a4)) {
                DEBUG_PRINTF("Dead state reached @ %lli\n", (s64a)(b4 - buf));
                *scan_end = end;
                *state = a4;
                return MO_CONTINUE_MATCHING;
            }
            if (cur_buf > min_accel_dist && INNER_ACCEL_FUNC32(a4)) {
                DEBUG_PRINTF("Accel state reached @ %lli\n", (s64a)(b4 - buf));
                const union AccelAux *aaux =
                    get_accel32(s, a4 & SHENG32_STATE_MASK);
                const u8 *new_offset = run_accel(aaux, cur_buf + 4, end);
                if (new_offset < cur_buf + 4 + BAD_ACCEL_DIST) {
                    min_accel_dist = new_offset + BIG_ACCEL_PENALTY;
                } else {
                    min_accel_dist = new_offset + SMALL_ACCEL_PENALTY;
                }
                DEBUG_PRINTF("Next accel chance: %llu\n",
                             (u64a)(min_accel_dist - start));
                DEBUG_PRINTF("Accel scanned %llu bytes\n",
                             (u64a)(new_offset - cur_buf - 4));
                cur_buf = new_offset;
                DEBUG_PRINTF("New offset: %llu\n", (u64a)(cur_buf - buf));
                continue;
            }
        }
        if (OUTER_DEAD_FUNC32(a4)) {
            DEBUG_PRINTF("Dead state reached @ %lli\n", (s64a)(cur_buf - buf));
            *scan_end = end;
            *state = a4;
            return MO_CONTINUE_MATCHING;
        };
        if (cur_buf > min_accel_dist && OUTER_ACCEL_FUNC32(a4)) {
            DEBUG_PRINTF("Accel state reached @ %lli\n", (s64a)(b4 - buf));
            const union AccelAux *aaux =
                get_accel32(s, a4 & SHENG32_STATE_MASK);
            const u8 *new_offset = run_accel(aaux, cur_buf + 4, end);
            if (new_offset < cur_buf + 4 + BAD_ACCEL_DIST) {
                min_accel_dist = new_offset + BIG_ACCEL_PENALTY;
            } else {
                min_accel_dist = new_offset + SMALL_ACCEL_PENALTY;
            }
            DEBUG_PRINTF("Next accel chance: %llu\n",
                         (u64a)(min_accel_dist - start));
            DEBUG_PRINTF("Accel scanned %llu bytes\n",
                         (u64a)(new_offset - cur_buf - 4));
            cur_buf = new_offset;
            DEBUG_PRINTF("New offset: %llu\n", (u64a)(cur_buf - buf));
            continue;
        };
        cur_buf += 4;
    }
    *state = movd512(cur_state);
    *scan_end = cur_buf;
    return MO_CONTINUE_MATCHING;
}

#ifndef NO_SHENG64_IMPL
static really_inline
char SHENG64_IMPL(u8 *state, NfaCallback cb, void *ctxt,
                  const struct sheng64 *s,
                  u8 *const cached_accept_state,
                  ReportID *const cached_accept_id,
                  u8 single, u64a base_offset, const u8 *buf, const u8 *start,
                  const u8 *end, const u8 **scan_end) {
    DEBUG_PRINTF("Starting DFAx4 execution in state %u\n",
                 *state & SHENG64_STATE_MASK);
    const u8 *cur_buf = start;
    base_offset++;
    DEBUG_PRINTF("Scanning %llu bytes\n", (u64a)(end - start));

    if (INNER_DEAD_FUNC64(*state) || OUTER_DEAD_FUNC64(*state)) {
        DEBUG_PRINTF("Dead on arrival\n");
        *scan_end = end;
        return MO_CONTINUE_MATCHING;
    }

    m512 cur_state = set64x8(*state);
    const m512 *masks = s->succ_masks;

    while (likely(end - cur_buf >= 4)) {
        const u8 *b1 = cur_buf;
        const u8 *b2 = cur_buf + 1;
        const u8 *b3 = cur_buf + 2;
        const u8 *b4 = cur_buf + 3;
        const u8 c1 = *b1;
        const u8 c2 = *b2;
        const u8 c3 = *b3;
        const u8 c4 = *b4;

        const m512 succ_mask1 = masks[c1];
        cur_state = vpermb512(cur_state, succ_mask1);
        const u8 a1 = movd512(cur_state);

        const m512 succ_mask2 = masks[c2];
        cur_state = vpermb512(cur_state, succ_mask2);
        const u8 a2 = movd512(cur_state);

        const m512 succ_mask3 = masks[c3];
        cur_state = vpermb512(cur_state, succ_mask3);
        const u8 a3 = movd512(cur_state);

        const m512 succ_mask4 = masks[c4];
        cur_state = vpermb512(cur_state, succ_mask4);
        const u8 a4 = movd512(cur_state);

        DEBUG_PRINTF("c: %02hhx '%c'\n", c1, ourisprint(c1) ? c1 : '?');
        DEBUG_PRINTF("s: %u (flag: %u)\n", a1 & SHENG64_STATE_MASK,
                     a1 & SHENG64_STATE_FLAG_MASK);

        DEBUG_PRINTF("c: %02hhx '%c'\n", c2, ourisprint(c2) ? c2 : '?');
        DEBUG_PRINTF("s: %u (flag: %u)\n", a2 & SHENG64_STATE_MASK,
                     a2 & SHENG64_STATE_FLAG_MASK);

        DEBUG_PRINTF("c: %02hhx '%c'\n", c3, ourisprint(c3) ? c3 : '?');
        DEBUG_PRINTF("s: %u (flag: %u)\n", a3 & SHENG64_STATE_MASK,
                     a3 & SHENG64_STATE_FLAG_MASK);

        DEBUG_PRINTF("c: %02hhx '%c'\n", c4, ourisprint(c4) ? c4 : '?');
        DEBUG_PRINTF("s: %u (flag: %u)\n", a4 & SHENG64_STATE_MASK,
                     a4 & SHENG64_STATE_FLAG_MASK);

        if (unlikely(INTERESTING_FUNC64(a1, a2, a3, a4))) {
            if (ACCEPT_FUNC64(a1)) {
                u64a match_offset = base_offset + b1 - buf;
                DEBUG_PRINTF("Accept state %u reached\n",
                             a1 & SHENG64_STATE_MASK);
                DEBUG_PRINTF("Match @ %llu\n", match_offset);
                if (STOP_AT_MATCH) {
                    DEBUG_PRINTF("Stopping at match @ %lli\n",
                                 (s64a)(b1 - start));
                    *scan_end = b1;
                    *state = a1;
                    return MO_MATCHES_PENDING;
                }
                if (single) {
                    if (fireSingleReport(cb, ctxt, s->report, match_offset) ==
                        MO_HALT_MATCHING) {
                        return MO_HALT_MATCHING;
                    }
                } else {
                    if (fireReports64(s, cb, ctxt, a1, match_offset,
                                      cached_accept_state, cached_accept_id,
                                      0) == MO_HALT_MATCHING) {
                        return MO_HALT_MATCHING;
                    }
                }
            }
            if (ACCEPT_FUNC64(a2)) {
                u64a match_offset = base_offset + b2 - buf;
                DEBUG_PRINTF("Accept state %u reached\n",
                             a2 & SHENG64_STATE_MASK);
                DEBUG_PRINTF("Match @ %llu\n", match_offset);
                if (STOP_AT_MATCH) {
                    DEBUG_PRINTF("Stopping at match @ %lli\n",
                                 (s64a)(b2 - start));
                    *scan_end = b2;
                    *state = a2;
                    return MO_MATCHES_PENDING;
                }
                if (single) {
                    if (fireSingleReport(cb, ctxt, s->report, match_offset) ==
                        MO_HALT_MATCHING) {
                        return MO_HALT_MATCHING;
                    }
                } else {
                    if (fireReports64(s, cb, ctxt, a2, match_offset,
                                      cached_accept_state, cached_accept_id,
                                      0) == MO_HALT_MATCHING) {
                        return MO_HALT_MATCHING;
                    }
                }
            }
            if (ACCEPT_FUNC64(a3)) {
                u64a match_offset = base_offset + b3 - buf;
                DEBUG_PRINTF("Accept state %u reached\n",
                             a3 & SHENG64_STATE_MASK);
                DEBUG_PRINTF("Match @ %llu\n", match_offset);
                if (STOP_AT_MATCH) {
                    DEBUG_PRINTF("Stopping at match @ %lli\n",
                                 (s64a)(b3 - start));
                    *scan_end = b3;
                    *state = a3;
                    return MO_MATCHES_PENDING;
                }
                if (single) {
                    if (fireSingleReport(cb, ctxt, s->report, match_offset) ==
                        MO_HALT_MATCHING) {
                        return MO_HALT_MATCHING;
                    }
                } else {
                    if (fireReports64(s, cb, ctxt, a3, match_offset,
                                      cached_accept_state, cached_accept_id,
                                      0) == MO_HALT_MATCHING) {
                        return MO_HALT_MATCHING;
                    }
                }
            }
            if (ACCEPT_FUNC64(a4)) {
                u64a match_offset = base_offset + b4 - buf;
                DEBUG_PRINTF("Accept state %u reached\n",
                             a4 & SHENG64_STATE_MASK);
                DEBUG_PRINTF("Match @ %llu\n", match_offset);
                if (STOP_AT_MATCH) {
                    DEBUG_PRINTF("Stopping at match @ %lli\n",
                                 (s64a)(b4 - start));
                    *scan_end = b4;
                    *state = a4;
                    return MO_MATCHES_PENDING;
                }
                if (single) {
                    if (fireSingleReport(cb, ctxt, s->report, match_offset) ==
                        MO_HALT_MATCHING) {
                        return MO_HALT_MATCHING;
                    }
                } else {
                    if (fireReports64(s, cb, ctxt, a4, match_offset,
                                      cached_accept_state, cached_accept_id,
                                      0) == MO_HALT_MATCHING) {
                        return MO_HALT_MATCHING;
                    }
                }
            }
            if (INNER_DEAD_FUNC64(a4)) {
                DEBUG_PRINTF("Dead state reached @ %lli\n", (s64a)(b4 - buf));
                *scan_end = end;
                *state = a4;
                return MO_CONTINUE_MATCHING;
            }
        }
        if (OUTER_DEAD_FUNC64(a4)) {
            DEBUG_PRINTF("Dead state reached @ %lli\n", (s64a)(cur_buf - buf));
            *scan_end = end;
            *state = a4;
            return MO_CONTINUE_MATCHING;
        }
        cur_buf += 4;
    }
    *state = movd512(cur_state);
    *scan_end = cur_buf;
    return MO_CONTINUE_MATCHING;
}
#endif // !NO_SHENG64_IMPL
#endif
