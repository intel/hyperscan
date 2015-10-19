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

#ifndef SIDECAR_INTERNAL_H
#define SIDECAR_INTERNAL_H

#include "ue2common.h"

#define SIDECAR_8   0
#define SIDECAR_32  1
#define SIDECAR_64  2
#define SIDECAR_128 3
#define SIDECAR_256 4
#define SIDECAR_N   5
#define SIDECAR_S   6

struct sidecar_id_offset {
    u32 first_offset; /* from base of sidecar */
    u32 count;
};

struct sidecar {
    u8 type;
    u32 size;
    u32 id_count;
    u32 mask_bit_count;
}; /* .. followed in memory by reach table */

#define SIDECAR_SPEC(bit_count, base_type)      \
struct sidecar_##bit_count {                    \
    struct sidecar header;                      \
    base_type reach[N_CHARS];                    \
    struct sidecar_id_offset id_list[bit_count];\
    base_type unshared_mask;                    \
};

struct sidecar_N {
    struct sidecar header;
    char c;
    char nocase;
    u32 report_count;
    u32 reports[];
};

struct sidecar_S {
    struct sidecar header;
    m128 hi;
    m128 lo;
    struct sidecar_id_offset id_list[8];
    u8 unshared_mask;
};

SIDECAR_SPEC(8,   u8)
SIDECAR_SPEC(32,  u32)
SIDECAR_SPEC(64,  u64a)
SIDECAR_SPEC(128, m128)
SIDECAR_SPEC(256, m256)

struct sidecar_enabled {
    u8 null;
};

struct sidecar_enabled_8 {
    u8 bits;
};

struct sidecar_enabled_32 {
    u32 bits;
};

struct sidecar_enabled_64 {
    u64a bits;
};

struct sidecar_enabled_128 {
    m128 bits;
};

struct sidecar_enabled_256 {
    m256 bits;
};

struct sidecar_enabled_N {
    u8 bits;
};

struct sidecar_enabled_S {
    u8 bits;
};

union sidecar_enabled_any {
    struct sidecar_enabled     arb;
    struct sidecar_enabled_8   e8;
    struct sidecar_enabled_32  e32;
    struct sidecar_enabled_64  e64;
    struct sidecar_enabled_128 e128;
    struct sidecar_enabled_256 e256;
    struct sidecar_enabled_N eN;
    struct sidecar_enabled_S eS;
};

/* ASCII ART TIME
 *
 * non-noodle sidecars
 *
 * ---------------------
 *    [  struct sidecar  ] ROUNDUP_16(sizeof(sidecar))
 *    ---------------------                                |
 *    [                   ]                                | Shufti: masks here
 *    [    reach table    ] sizeof(N) * N_CHARS            |
 *    [                   ]                                |
 *    ---------------------
 *    [ bit->id list head ] N * sizeof(sidecar_id_offset)
 *    ---------------------
 * --------------------- sizeof(sidecar_N)
 * [                   ]
 * [     id->masks     ] count(id) * sizeof(N)
 * [                   ]
 * ---------------------
 * [                   ]
 * [      id lists     ] complicated * sizeof(report)
 * [                   ]
 * ---------------------
 */

#define sidecar_ids_to_mask_const(side_struct)                          \
    ((const void *)((const char *)side_struct + sizeof(*side_struct)))



#endif
