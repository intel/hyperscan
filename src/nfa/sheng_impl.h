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
 *  - SHENG_IMPL    (name of the Sheng implementation function)
 *  - DEAD_FUNC     (name of the function checking for dead states)
 *  - ACCEPT_FUNC   (name of the function checking for accept state)
 *  - STOP_AT_MATCH (can be 1 or 0, enable or disable stop at match)
 */

/* byte-by-byte version. we don't do byte-by-byte death checking as it's
 * pretty pointless to do it over a buffer that's at most 3 bytes long */
static really_inline
char SHENG_IMPL(u8 *state, NfaCallback cb, void *ctxt, const struct sheng *s,
                u8 *const cached_accept_state, ReportID *const cached_accept_id,
                u8 single, u64a base_offset, const u8 *buf, const u8 *start,
                const u8 *end, const u8 **scan_end) {
    DEBUG_PRINTF("Starting DFA execution in state %u\n",
                 *state & SHENG_STATE_MASK);
    const u8 *cur_buf = start;
    if (DEAD_FUNC(*state)) {
        DEBUG_PRINTF("Dead on arrival\n");
        *scan_end = end;
        return MO_CONTINUE_MATCHING;
    }
    DEBUG_PRINTF("Scanning %lli bytes\n", (s64a)(end - start));

    m128 cur_state = set16x8(*state);
    const m128 *masks = s->shuffle_masks;

    while (likely(cur_buf != end)) {
        const u8 c = *cur_buf;
        const m128 shuffle_mask = masks[c];
        cur_state = pshufb_m128(shuffle_mask, cur_state);
        const u8 tmp = movd(cur_state);

        DEBUG_PRINTF("c: %02hhx '%c'\n", c, ourisprint(c) ? c : '?');
        DEBUG_PRINTF("s: %u (hi: %u lo: %u)\n", tmp, (tmp & 0xF0) >> 4,
                     tmp & 0xF);

        if (unlikely(ACCEPT_FUNC(tmp))) {
            DEBUG_PRINTF("Accept state %u reached\n", tmp & SHENG_STATE_MASK);
            u64a match_offset = base_offset + (cur_buf - buf) + 1;
            DEBUG_PRINTF("Match @ %llu\n", match_offset);
            if (STOP_AT_MATCH) {
                DEBUG_PRINTF("Stopping at match @ %lli\n",
                             (u64a)(cur_buf - start));
                *state = tmp;
                *scan_end = cur_buf;
                return MO_MATCHES_PENDING;
            }
            if (single) {
                if (fireSingleReport(cb, ctxt, s->report, match_offset) ==
                    MO_HALT_MATCHING) {
                    return MO_HALT_MATCHING;
                }
            } else {
                if (fireReports(s, cb, ctxt, tmp, match_offset,
                                cached_accept_state, cached_accept_id,
                                0) == MO_HALT_MATCHING) {
                    return MO_HALT_MATCHING;
                }
            }
        }
        cur_buf++;
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
    DEBUG_PRINTF("Starting DFA execution in state %u\n",
                 *state & SHENG32_STATE_MASK);
    const u8 *cur_buf = start;
    if (DEAD_FUNC32(*state)) {
        DEBUG_PRINTF("Dead on arrival\n");
        *scan_end = end;
        return MO_CONTINUE_MATCHING;
    }
    DEBUG_PRINTF("Scanning %lli bytes\n", (s64a)(end - start));

    m512 cur_state = set64x8(*state);
    const m512 *masks = s->succ_masks;

    while (likely(cur_buf != end)) {
        const u8 c = *cur_buf;
        const m512 succ_mask = masks[c];
        cur_state = vpermb512(cur_state, succ_mask);
        const u8 tmp = movd512(cur_state);

        DEBUG_PRINTF("c: %02hhx '%c'\n", c, ourisprint(c) ? c : '?');
        DEBUG_PRINTF("s: %u (flag: %u)\n", tmp & SHENG32_STATE_MASK,
                     tmp & SHENG32_STATE_FLAG_MASK);

        if (unlikely(ACCEPT_FUNC32(tmp))) {
            DEBUG_PRINTF("Accept state %u reached\n", tmp & SHENG32_STATE_MASK);
            u64a match_offset = base_offset + (cur_buf - buf) + 1;
            DEBUG_PRINTF("Match @ %llu\n", match_offset);
            if (STOP_AT_MATCH) {
                DEBUG_PRINTF("Stopping at match @ %lli\n",
                             (u64a)(cur_buf - start));
                *state = tmp;
                *scan_end = cur_buf;
                return MO_MATCHES_PENDING;
            }
            if (single) {
                if (fireSingleReport(cb, ctxt, s->report, match_offset) ==
                    MO_HALT_MATCHING) {
                    return MO_HALT_MATCHING;
                }
            } else {
                if (fireReports32(s, cb, ctxt, tmp, match_offset,
                                  cached_accept_state, cached_accept_id,
                                  0) == MO_HALT_MATCHING) {
                    return MO_HALT_MATCHING;
                }
            }
        }
        cur_buf++;
    }
    *state = movd512(cur_state);
    *scan_end = cur_buf;
    return MO_CONTINUE_MATCHING;
}

static really_inline
char SHENG64_IMPL(u8 *state, NfaCallback cb, void *ctxt,
                  const struct sheng64 *s,
                  u8 *const cached_accept_state,
                  ReportID *const cached_accept_id,
                  u8 single, u64a base_offset, const u8 *buf, const u8 *start,
                  const u8 *end, const u8 **scan_end) {
    DEBUG_PRINTF("Starting DFA execution in state %u\n",
                 *state & SHENG64_STATE_MASK);
    const u8 *cur_buf = start;
    if (DEAD_FUNC64(*state)) {
        DEBUG_PRINTF("Dead on arrival\n");
        *scan_end = end;
        return MO_CONTINUE_MATCHING;
    }
    DEBUG_PRINTF("Scanning %lli bytes\n", (s64a)(end - start));

    m512 cur_state = set64x8(*state);
    const m512 *masks = s->succ_masks;

    while (likely(cur_buf != end)) {
        const u8 c = *cur_buf;
        const m512 succ_mask = masks[c];
        cur_state = vpermb512(cur_state, succ_mask);
        const u8 tmp = movd512(cur_state);

        DEBUG_PRINTF("c: %02hhx '%c'\n", c, ourisprint(c) ? c : '?');
        DEBUG_PRINTF("s: %u (flag: %u)\n", tmp & SHENG64_STATE_MASK,
                     tmp & SHENG64_STATE_FLAG_MASK);

        if (unlikely(ACCEPT_FUNC64(tmp))) {
            DEBUG_PRINTF("Accept state %u reached\n", tmp & SHENG64_STATE_MASK);
            u64a match_offset = base_offset + (cur_buf - buf) + 1;
            DEBUG_PRINTF("Match @ %llu\n", match_offset);
            if (STOP_AT_MATCH) {
                DEBUG_PRINTF("Stopping at match @ %lli\n",
                             (u64a)(cur_buf - start));
                *state = tmp;
                *scan_end = cur_buf;
                return MO_MATCHES_PENDING;
            }
            if (single) {
                if (fireSingleReport(cb, ctxt, s->report, match_offset) ==
                    MO_HALT_MATCHING) {
                    return MO_HALT_MATCHING;
                }
            } else {
                if (fireReports64(s, cb, ctxt, tmp, match_offset,
                                  cached_accept_state, cached_accept_id,
                                  0) == MO_HALT_MATCHING) {
                    return MO_HALT_MATCHING;
                }
            }
        }
        cur_buf++;
    }
    *state = movd512(cur_state);
    *scan_end = cur_buf;
    return MO_CONTINUE_MATCHING;
}
#endif
