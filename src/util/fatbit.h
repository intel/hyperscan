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

#ifndef FATBIT_H
#define FATBIT_H

/** \file
 * \brief Multibit: fast bitset structure for use in scratch.
 * Uses more space than mmbit, to avoid partial words for hopefully a taddy more
 * performance.
 *
 * API is also trimmed down.
 */

#include "multibit.h"
#include "ue2common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MIN_FAT_SIZE 32

struct fatbit {
    union {
        u64a flat[MIN_FAT_SIZE / sizeof(u64a)];
        u8 raw[MIN_FAT_SIZE];
    } fb_int;
    u64a tail[];
};

static really_inline
void fatbit_clear(struct fatbit *bits) {
    assert(ISALIGNED(bits));
    memset(bits, 0, sizeof(struct fatbit));
}

static really_inline
char fatbit_set(struct fatbit *bits, u32 total_bits, u32 key) {
    assert(ISALIGNED(bits));
    return mmbit_set(bits->fb_int.raw, total_bits, key);
}

static really_inline
void fatbit_unset(struct fatbit *bits, u32 total_bits, u32 key) {
    assert(ISALIGNED(bits));
     mmbit_unset(bits->fb_int.raw, total_bits, key);
}

static really_inline
char fatbit_isset(const struct fatbit *bits, u32 total_bits, u32 key) {
    assert(ISALIGNED(bits));
    return mmbit_isset(bits->fb_int.raw, total_bits, key);
}

static really_inline
u32 fatbit_iterate(const struct fatbit *bits, u32 total_bits, u32 it_in) {
    assert(ISALIGNED(bits));
    /* TODO: iterate_flat could be specialised as we don't have to worry about
     * partial blocks. */
    return mmbit_iterate(bits->fb_int.raw, total_bits, it_in);
}

#ifdef __cplusplus
} // extern "C"
#endif

#endif
