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
 * \brief Inline functions for manipulating exhaustion vector.
 */

#ifndef EXHAUST_H
#define EXHAUST_H

#include "rose/rose_internal.h"
#include "util/multibit.h"
#include "ue2common.h"

/** Index meaning a given exhaustion key is invalid. */
#define INVALID_EKEY    (~(u32)0)

/** \brief Test whether the given key (\a ekey) is set in the exhaustion vector
 * \a evec. */
static really_inline
int isExhausted(const struct RoseEngine *t, const char *evec, u32 ekey) {
    DEBUG_PRINTF("checking exhaustion %p %u\n", evec, ekey);
    assert(ekey != INVALID_EKEY);
    assert(ekey < t->ekeyCount);
    return mmbit_isset((const u8 *)evec, t->ekeyCount, ekey);
}

/** \brief Returns 1 if all exhaustion keys in the bitvector are on. */
static really_inline
int isAllExhausted(const struct RoseEngine *t, const char *evec) {
    if (!t->canExhaust) {
        return 0; /* pattern set is inexhaustible */
    }

    return mmbit_all((const u8 *)evec, t->ekeyCount);
}

/** \brief Mark key \a ekey on in the exhaustion vector. */
static really_inline
void markAsMatched(const struct RoseEngine *t, char *evec, u32 ekey) {
    DEBUG_PRINTF("marking as exhausted key %u\n", ekey);
    assert(ekey != INVALID_EKEY);
    assert(ekey < t->ekeyCount);
    mmbit_set((u8 *)evec, t->ekeyCount, ekey);
}

/** \brief Clear all keys in the exhaustion vector. */
static really_inline
void clearEvec(const struct RoseEngine *t, char *evec) {
    DEBUG_PRINTF("clearing evec %p %u\n", evec, t->ekeyCount);
    mmbit_clear((u8 *)evec, t->ekeyCount);
}

#endif
