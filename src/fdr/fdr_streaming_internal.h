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

#ifndef FDR_STREAMING_INTERNAL_H
#define FDR_STREAMING_INTERNAL_H

#include "ue2common.h"
#include "fdr_internal.h"
#include "util/unaligned.h"

// tertiary table:
// a header (FDRSTableHeader)
// long_lits.size()+1 entries holding an offset to the string in the
//       'whole literal table' (FDRSLiteral structure)
// the whole literal table - every string packed in (freeform)
// hash table (caseful) (FDRSHashEntry)
// hash table (caseless) (FDRSHashEntry)

typedef enum {
    CASEFUL = 0,
    CASELESS = 1,
    MAX_MODES = 2
} MODES;

// We have one of these structures hanging off the 'link' of our secondary
// FDR table that handles streaming strings
struct FDRSTableHeader {
    u32 pseudoEngineID; // set to 0xffffffff to indicate this isn't an FDR

    // string id one beyond the maximum entry for this type of literal
    // boundary[CASEFUL] is the end of the caseful literals
    // boundary[CASELESS] is the end of the caseless literals and one beyond
    // the largest literal id (the size of the littab)
    u32 boundary[MAX_MODES];

    // offsets are 0 if no such table exists
    // offset from the base of the tertiary structure to the hash table
    u32 hashOffset[MAX_MODES];
    u32 hashNBits[MAX_MODES]; // lg2 of the size of the hash table

    u8 streamStateBits[MAX_MODES];
    u8 streamStateBytes; // total size of packed stream state in bytes
    u8 N; // prefix lengths
    u16 pad;
};

// One of these structures per literal entry in our secondary FDR table.
struct FDRSLiteral {
    u32 offset;
    // potentially - another u32 to point to the 'next lesser included literal'
    // which would be a literal that overlaps this one in such a way that a
    // failure to match _this_ literal can leave us in a state that we might
    // still match that literal. Offset information might also be called for,
    // in which case we might be wanting to use a FDRSLiteralOffset
};

typedef u32 FDRSLiteralOffset;

#define LINK_INVALID 0xffffffff

// One of these structures per hash table entry in our secondary FDR table
struct FDRSHashEntry {
    u64a bitfield;
    FDRSLiteralOffset state;
    u32 link;
};

static really_inline
u32 get_start_lit_idx(const struct FDRSTableHeader * h, MODES m) {
    return m == CASEFUL ? 0 : h->boundary[m-1];
}

static really_inline
u32 get_end_lit_idx(const struct FDRSTableHeader * h, MODES m) {
    return h->boundary[m];
}

static really_inline
const struct FDRSLiteral * getLitTab(const struct FDRSTableHeader * h) {
    return (const struct FDRSLiteral *) (((const u8 *)h) +
            ROUNDUP_16(sizeof(struct FDRSTableHeader)));
}

static really_inline
u32 getBaseOffsetOfLits(const struct FDRSTableHeader * h, MODES m) {
    return getLitTab(h)[get_start_lit_idx(h, m)].offset;
}

static really_inline
u32 packStateVal(const struct FDRSTableHeader * h, MODES m, u32 v) {
    return v - getBaseOffsetOfLits(h, m) + 1;
}

static really_inline
u32 unpackStateVal(const struct FDRSTableHeader * h, MODES m, u32 v) {
    return v + getBaseOffsetOfLits(h, m) - 1;
}

static really_inline
u32 has_bit(const struct FDRSHashEntry * ent, u32 bit) {
    return (ent->bitfield >> bit) & 0x1;
}

static really_inline
u32 streaming_hash(const u8 *ptr, UNUSED size_t len, MODES mode) {
    const u64a CASEMASK = 0xdfdfdfdfdfdfdfdfULL;
    const u64a MULTIPLIER = 0x0b4e0ef37bc32127ULL;
    assert(len >= 32);

    u64a v1 = unaligned_load_u64a(ptr);
    u64a v2 = unaligned_load_u64a(ptr + 8);
    u64a v3 = unaligned_load_u64a(ptr + 16);
    if (mode == CASELESS) {
        v1 &= CASEMASK;
        v2 &= CASEMASK;
        v3 &= CASEMASK;
    }
    v1 *= MULTIPLIER;
    v2 *= (MULTIPLIER*MULTIPLIER);
    v3 *= (MULTIPLIER*MULTIPLIER*MULTIPLIER);
    v1 >>= 32;
    v2 >>= 32;
    v3 >>= 32;
    return v1 ^ v2 ^ v3;
}

#endif
