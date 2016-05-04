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

#ifndef FDR_STREAMING_RUNTIME_H
#define FDR_STREAMING_RUNTIME_H

#include "fdr_streaming_internal.h"
#include "util/partial_store.h"

#include <string.h>

static really_inline
const struct FDRSTableHeader * getSHDR(const struct FDR * fdr) {
    const u8 * linkPtr = ((const u8 *)fdr) + fdr->link;
    // test if it's not really a engineID, but a 'pseudo engine id'
    assert(*(const u32 *)linkPtr == 0xffffffff);
    assert(linkPtr);
    return (const struct FDRSTableHeader *)linkPtr;
}

// Reads from stream state and unpacks values into stream state table.
static really_inline
void getStreamStates(const struct FDRSTableHeader * streamingTable,
                     const u8 * stream_state, u32 * table) {
    assert(streamingTable);
    assert(stream_state);
    assert(table);

    u8 ss_bytes = streamingTable->streamStateBytes;
    u8 ssb = streamingTable->streamStateBits[CASEFUL];
    UNUSED u8 ssb_nc = streamingTable->streamStateBits[CASELESS];
    assert(ss_bytes == (ssb + ssb_nc + 7) / 8);

#if defined(ARCH_32_BIT)
    // On 32-bit hosts, we may be able to avoid having to do any u64a
    // manipulation at all.
    if (ss_bytes <= 4) {
        u32 ssb_mask = (1U << ssb) - 1;
        u32 streamVal = partial_load_u32(stream_state, ss_bytes);
        table[CASEFUL] = (u32)(streamVal & ssb_mask);
        table[CASELESS] = (u32)(streamVal >> ssb);
        return;
    }
#endif

    u64a ssb_mask = (1ULL << ssb) - 1;
    u64a streamVal = partial_load_u64a(stream_state, ss_bytes);
    table[CASEFUL] = (u32)(streamVal & ssb_mask);
    table[CASELESS] = (u32)(streamVal >> (u64a)ssb);
}

#ifndef NDEBUG
// Defensive checking (used in assert) that these table values don't overflow
// outside the range available.
static really_inline UNUSED
u32 streamingTableOverflow(u32 * table, u8 ssb, u8 ssb_nc) {
    u32 ssb_mask = (1ULL << (ssb)) - 1;
    if (table[CASEFUL] & ~ssb_mask) {
        return 1;
    }
    u32 ssb_nc_mask = (1ULL << (ssb_nc)) - 1;
    if (table[CASELESS] & ~ssb_nc_mask) {
        return 1;
    }
    return 0;
}
#endif

// Reads from stream state table and packs values into stream state.
static really_inline
void setStreamStates(const struct FDRSTableHeader * streamingTable,
                     u8 * stream_state, u32 * table) {
    assert(streamingTable);
    assert(stream_state);
    assert(table);

    u8 ss_bytes = streamingTable->streamStateBytes;
    u8 ssb = streamingTable->streamStateBits[CASEFUL];
    UNUSED u8 ssb_nc = streamingTable->streamStateBits[CASELESS];
    assert(ss_bytes == (ssb + ssb_nc + 7) / 8);
    assert(!streamingTableOverflow(table, ssb, ssb_nc));

#if defined(ARCH_32_BIT)
    // On 32-bit hosts, we may be able to avoid having to do any u64a
    // manipulation at all.
    if (ss_bytes <= 4) {
        u32 stagingStreamState = table[CASEFUL];
        stagingStreamState |= (table[CASELESS] << ssb);

        partial_store_u32(stream_state, stagingStreamState, ss_bytes);
        return;
    }
#endif

    u64a stagingStreamState = (u64a)table[CASEFUL];
    stagingStreamState |= (u64a)table[CASELESS] << ((u64a)ssb);
    partial_store_u64a(stream_state, stagingStreamState, ss_bytes);
}

u32 fdrStreamStateActive(const struct FDR * fdr, const u8 * stream_state) {
    if (!stream_state) {
        return 0;
    }
    const struct FDRSTableHeader * streamingTable = getSHDR(fdr);
    u8 ss_bytes = streamingTable->streamStateBytes;

    // We just care if there are any bits set, and the test below is faster
    // than a partial_load_u64a (especially on 32-bit hosts).
    for (u32 i = 0; i < ss_bytes; i++) {
        if (*stream_state) {
            return 1;
        }
        ++stream_state;
    }
    return 0;
}

// binary search for the literal index that contains the current state
static really_inline
u32 findLitTabEntry(const struct FDRSTableHeader * streamingTable,
                    u32 stateValue, MODES m) {
    const struct FDRSLiteral * litTab = getLitTab(streamingTable);
    u32 lo = get_start_lit_idx(streamingTable, m);
    u32 hi = get_end_lit_idx(streamingTable, m);

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
        } else { //(litTab[mid].offset > stateValue) {
            hi = mid;
        }
    }
    assert(litTab[lo].offset <= stateValue);
    assert(litTab[hi].offset > stateValue);
    return lo;
}

static really_inline
void fdrUnpackStateMode(struct FDR_Runtime_Args *a,
                        const struct FDRSTableHeader *streamingTable,
                        const struct FDRSLiteral * litTab,
                        const u32 *state_table,
                        const MODES m) {
    if (!state_table[m]) {
        return;
    }

    u32 stateValue = unpackStateVal(streamingTable, m, state_table[m]);
    u32 idx = findLitTabEntry(streamingTable, stateValue, m);
    size_t found_offset = litTab[idx].offset;
    const u8 * found_buf = found_offset + (const u8 *)streamingTable;
    size_t found_sz = stateValue - found_offset;
    if (m == CASEFUL) {
        a->buf_history = found_buf;
        a->len_history = found_sz;
    } else {
        a->buf_history_nocase = found_buf;
        a->len_history_nocase = found_sz;
    }
}

static really_inline
void fdrUnpackState(const struct FDR * fdr, struct FDR_Runtime_Args * a,
                    const u8 * stream_state) {
    // nothing to do if there's no stream state for the case
    if (!stream_state) {
        return;
    }

    const struct FDRSTableHeader * streamingTable = getSHDR(fdr);
    const struct FDRSLiteral * litTab = getLitTab(streamingTable);

    u32 state_table[MAX_MODES];
    getStreamStates(streamingTable, stream_state, state_table);

    fdrUnpackStateMode(a, streamingTable, litTab, state_table, CASEFUL);
    fdrUnpackStateMode(a, streamingTable, litTab, state_table, CASELESS);
}

static really_inline
u32 do_single_confirm(const struct FDRSTableHeader * streamingTable,
                      const struct FDR_Runtime_Args * a, u32 hashState, MODES m) {
    const struct FDRSLiteral * litTab = getLitTab(streamingTable);
    u32 idx = findLitTabEntry(streamingTable, hashState, m);
    size_t found_offset = litTab[idx].offset;
    const u8 * s1 = found_offset + (const u8 *)streamingTable;
    assert(hashState > found_offset);
    size_t l1 = hashState - found_offset;
    const u8 * buf = a->buf;
    size_t len = a->len;
    const char nocase = m != CASEFUL;

    if (l1 > len) {
        const u8 * hist = nocase ? a->buf_history_nocase : a->buf_history;
        size_t hist_len = nocase ? a->len_history_nocase : a->len_history;

        if (l1 > len+hist_len) {
            return 0; // Break out - not enough total history
        }

        size_t overhang = l1 - len;
        assert(overhang <= hist_len);

        if (cmpForward(hist + hist_len - overhang, s1, overhang, nocase)) {
            return 0;
        }
        s1 += overhang;
        l1 -= overhang;
    }
    // if we got here, we don't need history or we compared ok out of history
    assert(l1 <= len);

    if (cmpForward(buf + len - l1, s1, l1, nocase)) {
        return 0;
    }
    return hashState; // our new state
}

static really_inline
void fdrFindStreamingHash(const struct FDR_Runtime_Args *a,
                          const struct FDRSTableHeader *streamingTable,
                          u8 hash_len, u32 *hashes) {
    u8 tempbuf[128];
    const u8 *base;
    if (hash_len > a->len) {
        assert(hash_len <= 128);
        size_t overhang = hash_len - a->len;
        assert(overhang <= a->len_history);
        memcpy(tempbuf, a->buf_history + a->len_history - overhang, overhang);
        memcpy(tempbuf + overhang, a->buf, a->len);
        base = tempbuf;
    } else {
        assert(hash_len <= a->len);
        base = a->buf + a->len - hash_len;
    }

    if (streamingTable->hashNBits[CASEFUL]) {
        hashes[CASEFUL] = streaming_hash(base, hash_len, CASEFUL);
    }
    if (streamingTable->hashNBits[CASELESS]) {
        hashes[CASELESS] = streaming_hash(base, hash_len, CASELESS);
    }
}

static really_inline
const struct FDRSHashEntry *getEnt(const struct FDRSTableHeader *streamingTable,
                                   u32 h, const MODES m) {
    u32 nbits = streamingTable->hashNBits[m];
    if (!nbits) {
        return NULL;
    }

    u32 h_ent = h & ((1 << nbits) - 1);
    u32 h_low = (h >> nbits) & 63;

    const struct FDRSHashEntry *tab =
        (const struct FDRSHashEntry *)((const u8 *)streamingTable
                                       + streamingTable->hashOffset[m]);
    const struct FDRSHashEntry *ent = tab + h_ent;

    if (!has_bit(ent, h_low)) {
        return NULL;
    }

    return ent;
}

static really_inline
void fdrPackStateMode(u32 *state_table, const struct FDR_Runtime_Args *a,
                      const struct FDRSTableHeader *streamingTable,
                      const struct FDRSHashEntry *ent, const MODES m) {
    assert(ent);
    assert(streamingTable->hashNBits[m]);

    const struct FDRSHashEntry *tab =
        (const struct FDRSHashEntry *)((const u8 *)streamingTable
                                       + streamingTable->hashOffset[m]);

    while (1) {
        u32 tmp = 0;
        if ((tmp = do_single_confirm(streamingTable, a, ent->state, m))) {
            state_table[m] = packStateVal(streamingTable, m, tmp);
            break;
        }
        if (ent->link == LINK_INVALID) {
            break;
        }
        ent = tab + ent->link;
    }
}

static really_inline
void fdrPackState(const struct FDR *fdr, const struct FDR_Runtime_Args *a,
                  u8 *stream_state) {
    // nothing to do if there's no stream state for the case
    if (!stream_state) {
        return;
    }

    // get pointers to the streamer FDR and the tertiary structure
    const struct FDRSTableHeader *streamingTable = getSHDR(fdr);

    assert(streamingTable->N);

    u32 state_table[MAX_MODES] = {0, 0};

    // if we don't have enough history, we don't need to do anything
    if (streamingTable->N <= a->len + a->len_history) {
        u32 hashes[MAX_MODES] = {0, 0};

        fdrFindStreamingHash(a, streamingTable, streamingTable->N, hashes);

        const struct FDRSHashEntry *ent_ful = getEnt(streamingTable,
                                                    hashes[CASEFUL], CASEFUL);
        const struct FDRSHashEntry *ent_less = getEnt(streamingTable,
                                                    hashes[CASELESS], CASELESS);

        if (ent_ful) {
            fdrPackStateMode(state_table, a, streamingTable, ent_ful,
                             CASEFUL);
        }

        if (ent_less) {
            fdrPackStateMode(state_table, a, streamingTable, ent_less,
                             CASELESS);
        }
    }

    setStreamStates(streamingTable, stream_state, state_table);
}

#endif
