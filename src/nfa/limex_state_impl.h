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

/** \file
 * \brief NFA stream state handling.
 */

#include "util/join.h"
#include "util/partial_store.h"
#include "util/state_compress.h"
#include <string.h>

#if !defined(SIZE) || !defined(STATE_T) || !defined(LOAD_FROM_ENG)
#  error Must define SIZE, STATE_T, LOAD_FROM_ENG in includer.
#endif

#define IMPL_NFA_T          JOIN(struct LimExNFA, SIZE)
#define COMMON_T            JOIN(NFACommon, SIZE)
#define REACHMASK_FN        JOIN(moNfaReachMask, SIZE)
#define COMPRESS_FN         JOIN(moNfaCompressState, SIZE)
#define EXPAND_FN           JOIN(moNfaExpandState, SIZE)
#define COMPRESSED_STORE_FN JOIN(store_compressed_, STATE_T)
#define COMPRESSED_LOAD_FN  JOIN(load_compressed_, STATE_T)
#define PARTIAL_STORE_FN    JOIN(partial_store_, STATE_T)
#define PARTIAL_LOAD_FN     JOIN(partial_load_, STATE_T)
#define OR_STATE            JOIN(or_, STATE_T)
#define AND_STATE           JOIN(and_, STATE_T)
#define ISZERO_STATE        JOIN(isZero_, STATE_T)

static really_inline
const ENG_STATE_T *get_reach_table(const IMPL_NFA_T *limex) {
    const ENG_STATE_T *reach
        = (const ENG_STATE_T *)((const char *)limex + sizeof(*limex));
    assert(ISALIGNED_N(reach, alignof(ENG_STATE_T)));
    return reach;
}

static really_inline
STATE_T REACHMASK_FN(const IMPL_NFA_T *limex, const u8 key) {
    const ENG_STATE_T *reach = get_reach_table(limex);
    return LOAD_FROM_ENG(&reach[limex->reachMap[key]]);
}

static really_inline
void COMPRESS_FN(const IMPL_NFA_T *limex, u8 *dest, const STATE_T *src,
                 u8 key) {
    assert(ISALIGNED_N(src, alignof(STATE_T)));
    STATE_T a_src = *src;

    DEBUG_PRINTF("compress state: %p -> %p\n", src, dest);

    if (!(limex->flags & LIMEX_FLAG_COMPRESS_STATE)) {
        // No key-based compression, just a partial store.
        DEBUG_PRINTF("store state into %u bytes\n", limex->stateSize);
        PARTIAL_STORE_FN(dest, a_src, limex->stateSize);
    } else {
        DEBUG_PRINTF("compress state, key=%hhx\n", key);

        STATE_T reachmask = REACHMASK_FN(limex, key);

        // Masked compression means that we mask off the initDs states and
        // provide a shortcut for the all-zeroes case. Note that these must be
        // switched on in the EXPAND call below.
        if (limex->flags & LIMEX_FLAG_COMPRESS_MASKED) {
            STATE_T s = AND_STATE(LOAD_FROM_ENG(&limex->compressMask), a_src);
            if (ISZERO_STATE(s)) {
                DEBUG_PRINTF("after compression mask, all states are zero\n");
                memset(dest, 0, limex->stateSize);
                return;
            }

            STATE_T mask = AND_STATE(LOAD_FROM_ENG(&limex->compressMask),
                                     reachmask);
            COMPRESSED_STORE_FN(dest, &s, &mask, limex->stateSize);
        } else {
            COMPRESSED_STORE_FN(dest, src, &reachmask, limex->stateSize);
        }
    }
}

static really_inline
void EXPAND_FN(const IMPL_NFA_T *limex, STATE_T *dest, const u8 *src, u8 key) {
    assert(ISALIGNED_N(dest, alignof(STATE_T)));
    DEBUG_PRINTF("expand state: %p -> %p\n", src, dest);

    if (!(limex->flags & LIMEX_FLAG_COMPRESS_STATE)) {
        // No key-based compression, just a partial load.
        DEBUG_PRINTF("load state from %u bytes\n", limex->stateSize);
        *dest = PARTIAL_LOAD_FN(src, limex->stateSize);
    } else {
        DEBUG_PRINTF("expand state, key=%hhx\n", key);
        STATE_T reachmask = REACHMASK_FN(limex, key);

        if (limex->flags & LIMEX_FLAG_COMPRESS_MASKED) {
            STATE_T mask = AND_STATE(LOAD_FROM_ENG(&limex->compressMask),
                                     reachmask);
            COMPRESSED_LOAD_FN(dest, src, &mask, limex->stateSize);
            *dest = OR_STATE(LOAD_FROM_ENG(&limex->initDS), *dest);
        } else {
            COMPRESSED_LOAD_FN(dest, src, &reachmask, limex->stateSize);
        }
    }
}

#undef IMPL_NFA_T
#undef COMMON_T
#undef REACHMASK_FN
#undef COMPRESS_FN
#undef EXPAND_FN
#undef COMPRESSED_STORE_FN
#undef COMPRESSED_LOAD_FN
#undef PARTIAL_STORE_FN
#undef PARTIAL_LOAD_FN
#undef OR_STATE
#undef AND_STATE
#undef ISZERO_STATE
