/*
 * Copyright (c) 2015-2018, Intel Corporation
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
 * \brief Runtime functions shared between various Rose runtime code.
 */

#ifndef ROSE_RUNTIME_H
#define ROSE_RUNTIME_H

#include "rose_internal.h"
#include "scratch.h"
#include "util/partial_store.h"

/*
 * ROSE STATE LAYOUT:
 *
 * - runtime status byte (halt status, delay rebuild dirty, etc)
 * - rose state multibit
 * - active leaf array (multibit)
 * - active leftfix array (multibit)
 * - leftfix lag table
 * - anchored matcher state
 * - literal groups
 * - history buffer
 * - exhausted bitvector
 * - som slots, som multibit arrays
 * - nfa stream state (for each nfa)
 */

#define rose_inline really_inline

/* Maximum offset that we will eagerly run prefixes to. Beyond this point, eager
 * prefixes are always run in exactly the same way as normal prefixes. */
#define EAGER_STOP_OFFSET 64


static really_inline
const void *getByOffset(const struct RoseEngine *t, u32 offset) {
    assert(offset < t->size);
    return (const u8 *)t + offset;
}

static really_inline
void *getRoleState(char *state) {
    return state + ROSE_STATE_OFFSET_ROLE_MMBIT;
}

/** \brief Fetch the active array for suffix nfas. */
static really_inline
u8 *getActiveLeafArray(const struct RoseEngine *t, char *state) {
    return (u8 *)(state + t->stateOffsets.activeLeafArray);
}

/** \brief Fetch the active array for rose nfas. */
static really_inline
u8 *getActiveLeftArray(const struct RoseEngine *t, char *state) {
    return (u8 *)(state + t->stateOffsets.activeLeftArray);
}

static really_inline
rose_group loadGroups(const struct RoseEngine *t, const char *state) {
    return partial_load_u64a(state + t->stateOffsets.groups,
                             t->stateOffsets.groups_size);

}

static really_inline
void storeGroups(const struct RoseEngine *t, char *state, rose_group groups) {
    partial_store_u64a(state + t->stateOffsets.groups, groups,
                       t->stateOffsets.groups_size);
}

static really_inline
u8 *getLongLitState(const struct RoseEngine *t, char *state) {
    return (u8 *)(state + t->stateOffsets.longLitState);
}

static really_inline
u8 *getLeftfixLagTable(const struct RoseEngine *t, char *state) {
    return (u8 *)(state + t->stateOffsets.leftfixLagTable);
}

static really_inline
const u8 *getLeftfixLagTableConst(const struct RoseEngine *t,
                                  const char *state) {
    return (const u8 *)(state + t->stateOffsets.leftfixLagTable);
}

static really_inline
u32 has_chained_nfas(const struct RoseEngine *t) {
    return t->outfixBeginQueue;
}

static really_inline
void updateLastMatchOffset(struct RoseContext *tctxt, u64a offset) {
    DEBUG_PRINTF("match @%llu, last match @%llu\n", offset,
                 tctxt->lastMatchOffset);

    assert(offset >= tctxt->minMatchOffset);
    assert(offset >= tctxt->lastMatchOffset);
    tctxt->lastMatchOffset = offset;
}

static really_inline
void updateLastCombMatchOffset(struct RoseContext *tctxt, u64a offset) {
    DEBUG_PRINTF("match @%llu, last match @%llu\n", offset,
                 tctxt->lastCombMatchOffset);

    assert(offset >= tctxt->lastCombMatchOffset);
    tctxt->lastCombMatchOffset = offset;
}

static really_inline
void updateMinMatchOffset(struct RoseContext *tctxt, u64a offset) {
    DEBUG_PRINTF("min match now @%llu, was @%llu\n", offset,
                 tctxt->minMatchOffset);

    assert(offset >= tctxt->minMatchOffset);
    assert(offset >= tctxt->minNonMpvMatchOffset);
    tctxt->minMatchOffset = offset;
    tctxt->minNonMpvMatchOffset = offset;
}

static really_inline
void updateMinMatchOffsetFromMpv(struct RoseContext *tctxt, u64a offset) {
    DEBUG_PRINTF("min match now @%llu, was @%llu\n", offset,
                 tctxt->minMatchOffset);

    assert(offset >= tctxt->minMatchOffset);
    assert(tctxt->minNonMpvMatchOffset >= tctxt->minMatchOffset);
    tctxt->minMatchOffset = offset;
    tctxt->minNonMpvMatchOffset = MAX(tctxt->minNonMpvMatchOffset, offset);
}
#endif
