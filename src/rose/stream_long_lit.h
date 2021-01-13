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
#include "util/compare.h"
#include "util/copybytes.h"

static really_inline
const struct RoseLongLitHashEntry *
getHashTableBase(const struct RoseLongLitTable *ll_table,
                 const struct RoseLongLitSubtable *ll_sub) {
    assert(ll_sub->hashOffset);
    return (const struct RoseLongLitHashEntry *)((const char *)ll_table +
                                                 ll_sub->hashOffset);
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
    u8 ssb = ll_table->caseful.streamStateBits;
    UNUSED u8 ssb_nc = ll_table->nocase.streamStateBits;
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

static rose_inline
void loadLongLiteralStateMode(struct hs_scratch *scratch,
                              const struct RoseLongLitTable *ll_table,
                              const struct RoseLongLitSubtable *ll_sub,
                              const u32 state, const char nocase) {
    if (!state) {
        DEBUG_PRINTF("no state for %s\n", nocase ? "caseless" : "caseful");
        return;
    }

    const struct RoseLongLitHashEntry *tab = getHashTableBase(ll_table, ll_sub);
    const struct RoseLongLitHashEntry *ent = tab + state - 1;

    assert(ent->str_offset + ent->str_len <= ll_table->size);
    const u8 *found_buf = (const u8 *)ll_table + ent->str_offset;
    size_t found_sz = ent->str_len;

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

    // If we don't have any long literals in play, these values must point to
    // the real history buffer so that CHECK_LONG_LIT instructions examine the
    // history buffer.
    scratch->tctxt.ll_buf = scratch->core_info.hbuf;
    scratch->tctxt.ll_len = scratch->core_info.hlen;
    scratch->tctxt.ll_buf_nocase = scratch->core_info.hbuf;
    scratch->tctxt.ll_len_nocase = scratch->core_info.hlen;

    if (!scratch->core_info.hlen) {
        return;
    }

    const struct RoseLongLitTable *ll_table =
        getByOffset(t, t->longLitTableOffset);
    const u8 *ll_state = getLongLitState(t, state);

    u32 state_case;
    u32 state_nocase;
    loadLongLitStreamState(ll_table, ll_state, &state_case, &state_nocase);

    DEBUG_PRINTF("loaded {%u, %u}\n", state_case, state_nocase);

    loadLongLiteralStateMode(scratch, ll_table, &ll_table->caseful,
                             state_case, 0);
    loadLongLiteralStateMode(scratch, ll_table, &ll_table->nocase,
                             state_nocase, 1);
}

static rose_inline
char confirmLongLiteral(const struct RoseLongLitTable *ll_table,
                        const struct hs_scratch *scratch,
                        const struct RoseLongLitHashEntry *ent,
                        const char nocase) {
    assert(ent->str_offset + ent->str_len <= ll_table->size);
    const u8 *s = (const u8 *)ll_table + ent->str_offset;
    size_t len = ent->str_len;
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

    return 1;
}

static rose_inline
const u8 *prepScanBuffer(const struct core_info *ci,
                         const struct RoseLongLitTable *ll_table, u8 *tempbuf) {
    const u8 hash_len = ll_table->maxLen;
    assert(hash_len >= LONG_LIT_HASH_LEN);

    // Our hash function operates over LONG_LIT_HASH_LEN bytes, starting from
    // location (end of buffer - hash_len). If this block can be satisfied
    // entirely from either the current buffer or the history buffer, we pass
    // in the pointer directly; otherwise we must make a copy.

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
            copy_upto_64_bytes(tempbuf, ci->hbuf + ci->hlen - overhang,
                               overhang);
            // Copy: second chunk from current buffer.
            size_t copy_buf_len = LONG_LIT_HASH_LEN - overhang;
            assert(copy_buf_len <= ci->len);
            copy_upto_64_bytes(tempbuf + overhang, ci->buf, copy_buf_len);
            // Read from our temporary buffer for the hash.
            base = tempbuf;
        }
    } else {
        // Can read enough to hash from inside the current buffer.
        base = ci->buf + ci->len - hash_len;
    }

    return base;
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
    u8 ssb = ll_table->caseful.streamStateBits;
    UNUSED u8 ssb_nc = ll_table->nocase.streamStateBits;
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

static really_inline
char has_bit(const u8 *data, u32 bit) {
    return (data[bit / 8] >> (bit % 8)) & 1;
}

static rose_inline
char bloomHasKey(const u8 *bloom, u32 bloom_mask, u32 hash) {
    return has_bit(bloom, hash & bloom_mask);
}

static rose_inline
char checkBloomFilter(const struct RoseLongLitTable *ll_table,
                      const struct RoseLongLitSubtable *ll_sub,
                      const u8 *scan_buf, char nocase) {
    assert(ll_sub->bloomBits);

    const u8 *bloom = (const u8 *)ll_table + ll_sub->bloomOffset;
    const u32 bloom_mask = (1U << ll_sub->bloomBits) - 1;

    char v = 1;
    v &= bloomHasKey(bloom, bloom_mask, bloomHash_1(scan_buf, nocase));
    v &= bloomHasKey(bloom, bloom_mask, bloomHash_2(scan_buf, nocase));
    v &= bloomHasKey(bloom, bloom_mask, bloomHash_3(scan_buf, nocase));
    return v;
}

/**
 * \brief Look for a hit in the hash table.
 *
 * Returns zero if not found, otherwise returns (bucket + 1).
 */
static rose_inline
u32 checkHashTable(const struct RoseLongLitTable *ll_table,
                   const struct RoseLongLitSubtable *ll_sub, const u8 *scan_buf,
                   const struct hs_scratch *scratch, char nocase) {
    const u32 nbits = ll_sub->hashBits;
    assert(nbits && nbits < 32);
    const u32 num_entries = 1U << nbits;

    const struct RoseLongLitHashEntry *tab = getHashTableBase(ll_table, ll_sub);

    u32 hash = hashLongLiteral(scan_buf, LONG_LIT_HASH_LEN, nocase);
    u32 bucket = hash & ((1U << nbits) - 1);

    while (tab[bucket].str_offset != 0) {
        DEBUG_PRINTF("checking bucket %u\n", bucket);
        if (confirmLongLiteral(ll_table, scratch, &tab[bucket], nocase)) {
            DEBUG_PRINTF("found hit for bucket %u\n", bucket);
            return bucket + 1;
        }

        if (++bucket == num_entries) {
            bucket = 0;
        }
    }

    return 0;
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
        u8 tempbuf[LONG_LIT_HASH_LEN];
        const u8 *scan_buf = prepScanBuffer(ci, ll_table, tempbuf);

        if (ll_table->caseful.hashBits &&
            checkBloomFilter(ll_table, &ll_table->caseful, scan_buf, 0)) {
            state_case = checkHashTable(ll_table, &ll_table->caseful, scan_buf,
                                        scratch, 0);
        }

        if (ll_table->nocase.hashBits &&
            checkBloomFilter(ll_table, &ll_table->nocase, scan_buf, 1)) {
            state_nocase = checkHashTable(ll_table, &ll_table->nocase, scan_buf,
                                          scratch, 1);
        }
    } else {
        DEBUG_PRINTF("not enough history (%zu bytes)\n", ci->len + ci->hlen);
    }

    DEBUG_PRINTF("store {%u, %u}\n", state_case, state_nocase);

    u8 *ll_state = getLongLitState(t, state);
    storeLongLitStreamState(ll_table, ll_state, state_case, state_nocase);
}

#endif // STREAM_LONG_LIT_H
