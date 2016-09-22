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

#ifndef STREAM_LONG_LIT_HASH_H
#define STREAM_LONG_LIT_HASH_H

#include "ue2common.h"
#include "util/bitutils.h"
#include "util/unaligned.h"

/** \brief Length of the buffer operated on by \ref hashLongLiteral(). */
#define LONG_LIT_HASH_LEN 24

/** \brief Multiplier used by al the hash functions below. */
#define HASH_MULTIPLIER 0x0b4e0ef37bc32127ULL

/** \brief Hash function used for long literal table in streaming mode. */
static really_inline
u32 hashLongLiteral(const u8 *ptr, UNUSED size_t len, char nocase) {
    // We unconditionally hash LONG_LIT_HASH_LEN bytes; all use cases of this
    // hash are for strings longer than this.
    assert(len >= 24);

    u64a v1 = unaligned_load_u64a(ptr);
    u64a v2 = unaligned_load_u64a(ptr + 8);
    u64a v3 = unaligned_load_u64a(ptr + 16);
    if (nocase) {
        v1 &= OCTO_CASE_CLEAR;
        v2 &= OCTO_CASE_CLEAR;
        v3 &= OCTO_CASE_CLEAR;
    }
    v1 *= HASH_MULTIPLIER;
    v2 *= HASH_MULTIPLIER * HASH_MULTIPLIER;
    v3 *= HASH_MULTIPLIER * HASH_MULTIPLIER * HASH_MULTIPLIER;
    v1 >>= 32;
    v2 >>= 32;
    v3 >>= 32;
    return v1 ^ v2 ^ v3;
}

/**
 * \brief Internal, used by the bloom filter hash functions below. Hashes 16
 * bytes beginning at (ptr + offset).
 */
static really_inline
u32 bloomHash_i(const u8 *ptr, u32 offset, u64a multiplier, char nocase) {
    assert(offset + 16 <= LONG_LIT_HASH_LEN);

    u64a v = unaligned_load_u64a(ptr + offset);
    if (nocase) {
        v &= OCTO_CASE_CLEAR;
    }
    v *= multiplier;
    return v >> 32;
}

/*
 * We ensure that we see every byte the first LONG_LIT_HASH_LEN bytes of input
 * data (using at least one of the following functions).
 */

static really_inline
u32 bloomHash_1(const u8 *ptr, char nocase) {
    const u64a multiplier = HASH_MULTIPLIER;
    return bloomHash_i(ptr, 0, multiplier, nocase);
}

static really_inline
u32 bloomHash_2(const u8 *ptr, char nocase) {
    const u64a multiplier = HASH_MULTIPLIER * HASH_MULTIPLIER;
    return bloomHash_i(ptr, 4, multiplier, nocase);
}

static really_inline
u32 bloomHash_3(const u8 *ptr, char nocase) {
    const u64a multiplier = HASH_MULTIPLIER * HASH_MULTIPLIER * HASH_MULTIPLIER;
    return bloomHash_i(ptr, 8, multiplier, nocase);
}

#endif // STREAM_LONG_LIT_HASH_H
