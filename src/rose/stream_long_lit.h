/*
 * Copyright (c) 2016, Intel Corporation
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

#ifndef STREAM_LONG_LIT_H
#define STREAM_LONG_LIT_H

#include "rose.h"
#include "rose_common.h"
#include "rose_internal.h"
#include "stream_long_lit_hash.h"
#include "util/copybytes.h"

static really_inline
const struct RoseLongLiteral *
getLitTab(const struct RoseLongLitTable *ll_table) {
    return (const struct RoseLongLiteral *)((const char *)ll_table +
            ROUNDUP_16(sizeof(struct RoseLongLitTable)));
}

static really_inline
u32 get_start_lit_idx(const struct RoseLongLitTable *ll_table,
                      const char nocase) {
    return nocase ? ll_table->boundaryCase : 0;
}

static really_inline
u32 get_end_lit_idx(const struct RoseLongLitTable *ll_table,
                    const char nocase) {
    return nocase ? ll_table->boundaryNocase : ll_table->boundaryCase;
}

// search for the literal index that contains the current state
static rose_inline
u32 findLitTabEntry(const struct RoseLongLitTable *ll_table,
                    u32 stateValue, const char nocase) {
    const struct RoseLongLiteral *litTab = getLitTab(ll_table);
    u32 lo = get_start_lit_idx(ll_table, nocase);
    u32 hi = get_end_lit_idx(ll_table, nocase);

    // Now move stateValue back by one so that we're looking for the
    // litTab entry that includes it the string, not the one 'one past' it
    stateValue -= 1;
    assert(lo != hi);
    assert(litTab[lo].offset <= stateValue);
    assert(litTab[hi].offset > stateValue);

    // binary search to find the entry e such that:
    // litTab[e].offsetToLiteral <= stateValue < litTab[e+1].offsetToLiteral
    while (lo + 1 < hi) {
        u32 mid = (lo + hi) / 2;
        if (litTab[mid].offset <= stateValue) {
            lo = mid;
        } else { // (litTab[mid].offset > stateValue) {
            hi = mid;
        }
    }
    assert(litTab[lo].offset <= stateValue);
    assert(litTab[hi].offset > stateValue);
    return lo;
}

// Reads from stream state and unpacks values into stream state table.
static really_inline
void loadLongLitStreamState(const struct RoseLongLitTable *ll_table,
                            const u8 *ll_state, u32 *state_case,
                            u32 *state_nocase) {
    assert(ll_table);
    assert(ll_state);
    assert(state_case && state_nocase);

    u8 ss_bytes = ll_table->streamStateBytes;
    u8 ssb = ll_table->streamStateBitsCase;
    UNUSED u8 ssb_nc = ll_table->streamStateBitsNocase;
    assert(ss_bytes == (ssb + ssb_nc + 7) / 8);

#if defined(ARCH_32_BIT)
    // On 32-bit hosts, we may be able to avoid having to do any u64a
    // manipulation at all.
    if (ss_bytes <= 4) {
        u32 ssb_mask = (1U << ssb) - 1;
        u32 streamVal = partial_load_u32(ll_state, ss_bytes);
        *state_case = (u32)(streamVal & ssb_mask);
        *state_nocase = (u32)(streamVal >> ssb);
        return;
    }
#endif

    u64a ssb_mask = (1ULL << ssb) - 1;
    u64a streamVal = partial_load_u64a(ll_state, ss_bytes);
    *state_case = (u32)(streamVal & ssb_mask);
    *state_nocase = (u32)(streamVal >> ssb);
}

static really_inline
u32 getBaseOffsetOfLits(const struct RoseLongLitTable *ll_table,
                        const char nocase) {
    u32 lit_idx = get_start_lit_idx(ll_table, nocase);
    return getLitTab(ll_table)[lit_idx].offset;
}

static really_inline
u32 unpackStateVal(const struct RoseLongLitTable *ll_table, const char nocase,
                   u32 v) {
    return v + getBaseOffsetOfLits(ll_table, nocase) - 1;
}

static really_inline
u32 packStateVal(const struct RoseLongLitTable *ll_table, const char nocase,
                 u32 v) {
    return v - getBaseOffsetOfLits(ll_table, nocase) + 1;
}

static rose_inline
void loadLongLiteralStateMode(struct hs_scratch *scratch,
                              const struct RoseLongLitTable *ll_table,
                              const struct RoseLongLiteral *litTab,
                              const u32 state, const char nocase) {
    if (!state) {
        DEBUG_PRINTF("no state for %s\n", nocase ? "caseless" : "caseful");
        return;
    }

    u32 stateValue = unpackStateVal(ll_table, nocase, state);
    u32 idx = findLitTabEntry(ll_table, stateValue, nocase);
    size_t found_offset = litTab[idx].offset;
    const u8 *found_buf = found_offset + (const u8 *)ll_table;
    size_t found_sz = stateValue - found_offset;

    struct RoseContext *tctxt = &scratch->tctxt;
    if (nocase) {
        tctxt->ll_buf_nocase = found_buf;
        tctxt->ll_len_nocase = found_sz;
    } else {
        tctxt->ll_buf = found_buf;
        tctxt->ll_len = found_sz;
    }
}

static rose_inline
void loadLongLiteralState(const struct RoseEngine *t, char *state,
                          struct hs_scratch *scratch) {
    if (!t->longLitTableOffset) {
        return;
    }

    scratch->tctxt.ll_buf = scratch->core_info.hbuf;
    scratch->tctxt.ll_len = scratch->core_info.hlen;
    scratch->tctxt.ll_buf_nocase = scratch->core_info.hbuf;
    scratch->tctxt.ll_len_nocase = scratch->core_info.hlen;

    const struct RoseLongLitTable *ll_table =
        getByOffset(t, t->longLitTableOffset);
    const struct RoseLongLiteral *litTab = getLitTab(ll_table);
    const u8 *ll_state = getLongLitState(t, state);

    u32 state_case;
    u32 state_nocase;
    loadLongLitStreamState(ll_table, ll_state, &state_case, &state_nocase);

    loadLongLiteralStateMode(scratch, ll_table, litTab, state_case, 0);
    loadLongLiteralStateMode(scratch, ll_table, litTab, state_nocase, 1);
}

static rose_inline
char confirmLongLiteral(const struct RoseLongLitTable *ll_table,
                        const hs_scratch_t *scratch, u32 hashState,
                        const char nocase) {
    const struct RoseLongLiteral *litTab = getLitTab(ll_table);
    u32 idx = findLitTabEntry(ll_table, hashState, nocase);
    size_t found_offset = litTab[idx].offset;
    const u8 *s = found_offset + (const u8 *)ll_table;
    assert(hashState > found_offset);
    size_t len = hashState - found_offset;
    const u8 *buf = scratch->core_info.buf;
    const size_t buf_len = scratch->core_info.len;

    if (len > buf_len) {
        const struct RoseContext *tctxt = &scratch->tctxt;
        const u8 *hist = nocase ? tctxt->ll_buf_nocase : tctxt->ll_buf;
        size_t hist_len = nocase ? tctxt->ll_len_nocase : tctxt->ll_len;

        if (len > buf_len + hist_len) {
            return 0; // Break out - not enough total history
        }

        size_t overhang = len - buf_len;
        assert(overhang <= hist_len);

        if (cmpForward(hist + hist_len - overhang, s, overhang, nocase)) {
            return 0;
        }
        s += overhang;
        len -= overhang;
    }

    // if we got here, we don't need history or we compared ok out of history
    assert(len <= buf_len);

    if (cmpForward(buf + buf_len - len, s, len, nocase)) {
        return 0;
    }

    DEBUG_PRINTF("confirmed hashState=%u\n", hashState);
    return 1;
}

static rose_inline
void calcStreamingHash(const struct core_info *ci,
                       const struct RoseLongLitTable *ll_table, u8 hash_len,
                       u32 *hash_case, u32 *hash_nocase) {
    assert(hash_len >= LONG_LIT_HASH_LEN);

    // Our hash function operates over LONG_LIT_HASH_LEN bytes, starting from
    // location (end of buffer - hash_len). If this block can be satisfied
    // entirely from either the current buffer or the history buffer, we pass
    // in the pointer directly; otherwise we must make a copy.

    u8 tempbuf[LONG_LIT_HASH_LEN];
    const u8 *base;

    if (hash_len > ci->len) {
        size_t overhang = hash_len - ci->len;
        if (overhang >= LONG_LIT_HASH_LEN) {
            // Can read enough to hash from inside the history buffer.
            assert(overhang <= ci->hlen);
            base = ci->hbuf + ci->hlen - overhang;
        } else {
            // Copy: first chunk from history buffer.
            assert(overhang <= ci->hlen);
            copy_upto_32_bytes(tempbuf, ci->hbuf + ci->hlen - overhang,
                               overhang);
            // Copy: second chunk from current buffer.
            size_t copy_buf_len = LONG_LIT_HASH_LEN - overhang;
            assert(copy_buf_len <= ci->len);
            copy_upto_32_bytes(tempbuf + overhang, ci->buf, copy_buf_len);
            // Read from our temporary buffer for the hash.
            base = tempbuf;
        }
    } else {
        // Can read enough to hash from inside the current buffer.
        base = ci->buf + ci->len - hash_len;
    }

    if (ll_table->hashNBitsCase) {
        *hash_case = hashLongLiteral(base, LONG_LIT_HASH_LEN, 0);
        DEBUG_PRINTF("caseful hash %u\n", *hash_case);
    }
    if (ll_table->hashNBitsNocase) {
        *hash_nocase = hashLongLiteral(base, LONG_LIT_HASH_LEN, 1);
        DEBUG_PRINTF("caseless hash %u\n", *hash_nocase);
    }
}

static really_inline
const struct RoseLongLitHashEntry *
getHashTableBase(const struct RoseLongLitTable *ll_table, const char nocase) {
    const u32 hashOffset = nocase ? ll_table->hashOffsetNocase
                                  : ll_table->hashOffsetCase;
    return (const struct RoseLongLitHashEntry *)((const char *)ll_table +
                                                 hashOffset);
}

static rose_inline
const struct RoseLongLitHashEntry *
getLongLitHashEnt(const struct RoseLongLitTable *ll_table, u32 h,
                  const char nocase) {
    u32 nbits = nocase ? ll_table->hashNBitsNocase : ll_table->hashNBitsCase;
    if (!nbits) {
        return NULL;
    }

    u32 h_ent = h & ((1 << nbits) - 1);
    u32 h_low = (h >> nbits) & 63;

    const struct RoseLongLitHashEntry *tab = getHashTableBase(ll_table, nocase);
    const struct RoseLongLitHashEntry *ent = tab + h_ent;

    if (!((ent->bitfield >> h_low) & 0x1)) {
        return NULL;
    }

    return ent;
}

static rose_inline
u32 storeLongLiteralStateMode(const struct hs_scratch *scratch,
                              const struct RoseLongLitTable *ll_table,
                              const struct RoseLongLitHashEntry *ent,
                              const char nocase) {
    assert(ent);
    assert(nocase ? ll_table->hashNBitsNocase : ll_table->hashNBitsCase);

    const struct RoseLongLitHashEntry *tab = getHashTableBase(ll_table, nocase);

    u32 packed_state = 0;
    while (1) {
        if (confirmLongLiteral(ll_table, scratch, ent->state, nocase)) {
            packed_state = packStateVal(ll_table, nocase, ent->state);
            DEBUG_PRINTF("set %s state to %u\n", nocase ? "nocase" : "case",
                         packed_state);
            break;
        }
        if (ent->link == LINK_INVALID) {
            break;
        }
        ent = tab + ent->link;
    }
    return packed_state;
}

#ifndef NDEBUG
// Defensive checking (used in assert) that these table values don't overflow
// the range available.
static really_inline
char streamingTableOverflow(u32 state_case, u32 state_nocase, u8 ssb,
                            u8 ssb_nc) {
    u32 ssb_mask = (1ULL << (ssb)) - 1;
    if (state_case & ~ssb_mask) {
        return 1;
    }
    u32 ssb_nc_mask = (1ULL << (ssb_nc)) - 1;
    if (state_nocase & ~ssb_nc_mask) {
        return 1;
    }
    return 0;
}
#endif

// Reads from stream state table and packs values into stream state.
static rose_inline
void storeLongLitStreamState(const struct RoseLongLitTable *ll_table,
                             u8 *ll_state, u32 state_case, u32 state_nocase) {
    assert(ll_table);
    assert(ll_state);

    u8 ss_bytes = ll_table->streamStateBytes;
    u8 ssb = ll_table->streamStateBitsCase;
    UNUSED u8 ssb_nc = ll_table->streamStateBitsNocase;
    assert(ss_bytes == ROUNDUP_N(ssb + ssb_nc, 8) / 8);
    assert(!streamingTableOverflow(state_case, state_nocase, ssb, ssb_nc));

#if defined(ARCH_32_BIT)
    // On 32-bit hosts, we may be able to avoid having to do any u64a
    // manipulation at all.
    if (ss_bytes <= 4) {
        u32 stagingStreamState = state_case;
        stagingStreamState |= (state_nocase << ssb);
        partial_store_u32(ll_state, stagingStreamState, ss_bytes);
        return;
    }
#endif

    u64a stagingStreamState = (u64a)state_case;
    stagingStreamState |= (u64a)state_nocase << ssb;
    partial_store_u64a(ll_state, stagingStreamState, ss_bytes);
}

static rose_inline
void storeLongLiteralState(const struct RoseEngine *t, char *state,
                           struct hs_scratch *scratch) {
    if (!t->longLitTableOffset) {
        DEBUG_PRINTF("no table\n");
        return;
    }

    struct core_info *ci = &scratch->core_info;
    const struct RoseLongLitTable *ll_table =
        getByOffset(t, t->longLitTableOffset);
    assert(ll_table->maxLen);

    DEBUG_PRINTF("maxLen=%u, len=%zu, hlen=%zu\n", ll_table->maxLen, ci->len,
                 ci->hlen);

    u32 state_case = 0;
    u32 state_nocase = 0;

    // If we don't have enough history, we don't need to do anything.
    if (ll_table->maxLen <= ci->len + ci->hlen) {
        u32 hash_case = 0;
        u32 hash_nocase = 0;

        calcStreamingHash(ci, ll_table, ll_table->maxLen, &hash_case,
                          &hash_nocase);

        const struct RoseLongLitHashEntry *ent_case =
            getLongLitHashEnt(ll_table, hash_case, 0);
        const struct RoseLongLitHashEntry *ent_nocase =
            getLongLitHashEnt(ll_table, hash_nocase, 1);

        DEBUG_PRINTF("ent_caseful=%p, ent_caseless=%p\n", ent_case, ent_nocase);

        if (ent_case) {
            state_case = storeLongLiteralStateMode(scratch, ll_table,
                                                   ent_case, 0);
        }

        if (ent_nocase) {
            state_nocase = storeLongLiteralStateMode(scratch, ll_table,
                                                     ent_nocase, 1);
        }
    }

    DEBUG_PRINTF("store {%u, %u}\n", state_case, state_nocase);

    u8 *ll_state = getLongLitState(t, state);
    storeLongLitStreamState(ll_table, ll_state, state_case, state_nocase);
}

#endif // STREAM_LONG_LIT_H
