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

/** \file
 * \brief Inline functions for manipulating exhaustion vector.
 */

#ifndef EXHAUST_H
#define EXHAUST_H

#include "rose/rose_internal.h"
#include "ue2common.h"
#include <string.h>

/** \brief Sentinel value meaning no further exhaustion keys. */
#define END_EXHAUST (~(u32)0)

/** \brief Test whether the given key (\a eoff) is set in the exhaustion vector
 * \a evec. */
static really_inline
int isExhausted(const char *evec, u32 eoff) {
    DEBUG_PRINTF("checking exhaustion %p %u\n", evec, eoff);
    return eoff != END_EXHAUST && (evec[eoff >> 3] & (1 << (eoff % 8)));
}

/** \brief Returns 1 if all exhaustion keys in the bitvector are on. */
static really_inline
int isAllExhausted(const struct RoseEngine *t, const char *evec_in) {
    if (!t->canExhaust) {
        return 0; /* pattern set is inexhaustible */
    }

    const u8 *evec = (const u8 *)evec_in;

    u32 whole_bytes = t->ekeyCount / 8;
    for (u32 i = 0; i < whole_bytes; i++) {
        if (evec[i] != 0xff) {
            DEBUG_PRINTF("unexhausted pattern in byte %u\n", i);
            return 0;
        }
    }

    u32 rem = t->ekeyCount % 8;
    if (t->ekeyCount % 8) {
        u8 mask = (1 << rem) - 1;
        if (evec[whole_bytes] != (char)mask) {
            DEBUG_PRINTF("unexhausted pattern (%hhu) in final byte\n", mask);
            return 0;
        }
    }

    DEBUG_PRINTF("pattern set is exhausted\n");
    return 1;
}

/** \brief Mark key \a eoff on in the exhaustion vector. */
static really_inline
void markAsMatched(char *evec, u32 eoff) {
    if (eoff != END_EXHAUST) {
        DEBUG_PRINTF("marking as exhausted key %u\n", eoff);
        evec[eoff >> 3] |= 1 << (eoff % 8);
    }
}

/** \brief Clear all keys in the exhaustion vector. */
static really_inline
void clearEvec(char *ev, const struct RoseEngine *t) {
    size_t size = (t->ekeyCount + 7) / 8;
    DEBUG_PRINTF("clearing evec %p %zu\n", ev, size);
    memset(ev, 0, size);
}

#endif
