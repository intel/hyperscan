/*
 * Copyright (c) 2015, Intel Corporation
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

/* in param TAG, STATE_T */

#include "util/join.h"

#if TAG == 8
#define ISTATE_T u32
#else
#define ISTATE_T STATE_T
#endif

#define EXEC_FN JOIN(sidecarExec_, TAG)
#define EXEC_I_FN JOIN(sidecarExec_i_, TAG)
#define ENABLED_INIT_FN JOIN(sidecarEnabledInit_, TAG)
#define ENABLED_UNION_FN JOIN(sidecarEnabledUnion_, TAG)
#define ENABLED_STRUCT JOIN(struct sidecar_enabled_, TAG)
#define PLAY_CB_FB JOIN(sidecarPlayCallbacks_, STATE_T)
#define SIDECAR_STRUCT JOIN(struct sidecar_, TAG)
#define MR_STRUCT JOIN(struct sidecar_mr_, TAG)
#define load_state JOIN(load_, STATE_T)
#define store_state JOIN(store_, STATE_T)
#define and_state JOIN(and_, STATE_T)
#define iand_state JOIN(and_, ISTATE_T)
#define andnot_state JOIN(andnot_, STATE_T)
#define or_state JOIN(or_, STATE_T)
#define is_zero JOIN(isZero_, STATE_T)
#define iis_zero JOIN(isZero_, ISTATE_T)
#define is_not_zero JOIN(isNonZero_, ISTATE_T)
#define not_eq JOIN(noteq_, STATE_T)
#define inot_eq JOIN(noteq_, ISTATE_T)
#define find_and_clear_lsb JOIN(findAndClearLSB_, TAG)
#define zero_state JOIN(zero_, ISTATE_T)

#if TAG <= 64
#define TDEBUG_PRINTF(...) DEBUG_PRINTF(__VA_ARGS__)
#define ATDEBUG_PRINTF(...) ADEBUG_PRINTF(__VA_ARGS__)
#else
#define TDEBUG_PRINTF(...) do { } while(0)
#define ATDEBUG_PRINTF(...) do { } while(0)
#endif

MR_STRUCT {
    const u8 *loc;
    STATE_T mask;
};

static really_inline
void PLAY_CB_FB(const SIDECAR_STRUCT *n, const u8 *b, const MR_STRUCT *matches,
                u32 match_len, ENABLED_STRUCT *enabled, u64a base_offset,
                SidecarCallback cb, void *context) {
    const STATE_T *id_mask_map = sidecar_ids_to_mask_const(n);
    const struct sidecar_id_offset *id_map = n->id_list;

    STATE_T e_local = load_state(&enabled->bits);

    DEBUG_PRINTF("playing %u matches\n", match_len);
    TDEBUG_PRINTF("enabled %08llu\n", (u64a)enabled->bits);

    for (u32 i = 0; i < match_len; i++) {
        u64a offset = matches[i].loc - b + base_offset;
        DEBUG_PRINTF("match at %llu\n", offset);

        STATE_T local_m = andnot_state(load_state(&matches[i].mask), e_local);

        e_local = and_state(matches[i].mask, e_local);

        TDEBUG_PRINTF("%08llu=~%08llu^%08llu\n", (u64a)local_m,
                      (u64a)matches[i].mask, (u64a)e_local);

        while (is_not_zero(local_m)) {
            u32 bit = find_and_clear_lsb(&local_m);
            DEBUG_PRINTF("bit %u at %llu\n", bit, offset);
            const u32 *id_base = (const u32 *)
                ((const char *)n + id_map[bit].first_offset);
            assert(ISALIGNED_N(id_base, 4));
            u32 count = id_map[bit].count;
            for (u32 j = 0; j < count; ++j) {
                cb(offset, id_base[j], context);
                STATE_T u_local = and_state(id_mask_map[id_base[j]],
                                            load_state(&n->unshared_mask));
                DEBUG_PRINTF("squashing unshared???\n");
                e_local = andnot_state(u_local, e_local);
                local_m = andnot_state(u_local, local_m);
            }
        }
    }

    TDEBUG_PRINTF("enabled %08llu\n", (u64a)e_local);
    store_state(&enabled->bits, e_local);
}

/* returns count of match locations */
static really_inline
MR_STRUCT *EXEC_I_FN(const SIDECAR_STRUCT *n, const u8 *b, const u8 *b_end,
              STATE_T state_in, MR_STRUCT *matches) {
    DEBUG_PRINTF("running over %zu\n", b_end - b);
    const STATE_T *table = (const STATE_T *)&n->reach;
    ISTATE_T s = state_in;

    b_end--; /* last byte is unrolled at end of function */
    for (; b < b_end; b++) {
        u8 c = *b;
        ISTATE_T r = table[c];
        ISTATE_T s1 = iand_state(s, r);
        if (inot_eq(s1, s)) {
            TDEBUG_PRINTF("recording match %08llu\n", (u64a)s1);
            matches->loc = b;
            store_state(&matches->mask, s1);
            matches++;
            if (DO_DEAD_CHECK && iis_zero(s1)) {
                goto done;
            }
        }
        s = s1;
    }

    /* do final byte by itself; gain blessing from the gcc gods */
    u8 c = *b;
    ISTATE_T r = table[c];
    ISTATE_T s1 = iand_state(s, r);
    if (inot_eq(s1, s)) {
        TDEBUG_PRINTF("recording match %08llu\n", (u64a)s1);
        matches->loc = b;
        matches->mask = s1;
        matches++;
    }

done:
    return matches;
}

static never_inline
void EXEC_FN(const SIDECAR_STRUCT *n, const u8 *b, size_t len,
             ENABLED_STRUCT *enabled, struct sidecar_scratch *scratch,
             u64a base_offset, SidecarCallback cb, void *context) {
    STATE_T e_local = load_state(&enabled->bits);
    if (is_zero(e_local)) {
        return;
    }

    MR_STRUCT *matches = (MR_STRUCT *)scratch;
    DEBUG_PRINTF("running sidecar over %zu len\n", len);
    DEBUG_PRINTF("enabled %p scratch %p\n", enabled, scratch);
    TDEBUG_PRINTF("enabled %08llu\n", (u64a)enabled->bits);
    MR_STRUCT *matches_out = EXEC_I_FN(n, b, b + len, e_local, matches);
    TDEBUG_PRINTF("enabled %08llu\n", (u64a)enabled->bits);
    if (matches_out - matches) {
        PLAY_CB_FB(n, b, matches, matches_out - matches, enabled, base_offset,
                   cb, context);
    }

    TDEBUG_PRINTF("enabled %08llu\n", (u64a)enabled->bits);
}

static really_inline
void ENABLED_INIT_FN(struct sidecar_enabled *enabled) {
    ENABLED_STRUCT *e = (void *)enabled;
    store_state(&e->bits, zero_state);
}

static really_inline
void ENABLED_UNION_FN(struct sidecar_enabled *dest,
                           const struct sidecar_enabled *src) {
    ENABLED_STRUCT *d = (void *)dest;
    const ENABLED_STRUCT *s = (const void *)src;
    store_state(&d->bits, or_state(load_state(&d->bits), load_state(&s->bits)));
}


#undef ENABLED_STRUCT
#undef ENABLED_INIT_FN
#undef ENABLED_UNION_FN
#undef EXEC_FN
#undef EXEC_I_FN
#undef load_state
#undef MR_STRUCT
#undef PLAY_CB_FB
#undef SIDECAR_STRUCT
#undef store_state
#undef and_state
#undef iand_state
#undef andnot_state
#undef not_eq
#undef inot_eq
#undef or_state
#undef is_zero
#undef is_not_zero
#undef zero_state

#undef TDEBUG_PRINTF
#undef ATDEBUG_PRINTF

#undef ISTATE_T

#undef TAG
#undef STATE_T
