/*
 * Copyright (c) 2015-2023, Intel Corporation
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
 * \brief Functions for allocating and manipulating scratch space.
 */

#include <stdlib.h>
#include <string.h>

#include "allocator.h"
#include "hs_internal.h"
#include "hs_runtime.h"
#include "scratch.h"
#include "state.h"
#include "ue2common.h"
#include "database.h"
#include "nfa/nfa_api_queue.h"
#include "rose/rose_internal.h"
#include "util/fatbit.h"

/**
 * Determine the space required for a correctly aligned array of fatbit
 * structure, laid out as:
 *
 * - an array of num_entries pointers, each to a fatbit.
 * - an array of fatbit structures, each of size fatbit_len.
 *
 * fatbit_len should have been determined at compile time, via the
 * fatbit_size() call.
 */
static
size_t fatbit_array_size(u32 num_entries, u32 fatbit_len) {
    size_t len = 0;

    // Array of pointers to each fatbit entry.
    len += sizeof(struct fatbit *) * num_entries;

    // Fatbit entries themselves.
    len = ROUNDUP_N(len, alignof(struct fatbit));
    len += (size_t)fatbit_len * num_entries;

    return ROUNDUP_N(len, 8); // Round up for potential padding.
}

/** Used by hs_alloc_scratch and hs_clone_scratch to allocate a complete
 * scratch region from a prototype structure. */
static
hs_error_t alloc_scratch(const hs_scratch_t *proto, hs_scratch_t **scratch) {
    u32 queueCount = proto->queueCount;
    u32 activeQueueArraySize = proto->activeQueueArraySize;
    u32 deduperCount = proto->deduper.dkey_count;
    u32 deduperLogSize = proto->deduper.log_size;
    u32 bStateSize = proto->bStateSize;
    u32 tStateSize = proto->tStateSize;
    u32 fullStateSize = proto->fullStateSize;
    u32 anchored_literal_region_len = proto->anchored_literal_region_len;
    u32 anchored_literal_fatbit_size = proto->anchored_literal_fatbit_size;

    u32 som_store_size = proto->som_store_count * sizeof(u64a);
    u32 som_attempted_store_size = proto->som_store_count * sizeof(u64a);
    u32 som_now_size = proto->som_fatbit_size;
    u32 som_attempted_size = proto->som_fatbit_size;

    struct hs_scratch *s;
    struct hs_scratch *s_tmp;
    size_t queue_size = queueCount * sizeof(struct mq);
    size_t qmpq_size = queueCount * sizeof(struct queue_match);

    assert(anchored_literal_region_len < 8 * sizeof(s->al_log_sum));

    size_t anchored_literal_region_size = fatbit_array_size(
        anchored_literal_region_len, proto->anchored_literal_fatbit_size);
    size_t delay_region_size =
        fatbit_array_size(DELAY_SLOT_COUNT, proto->delay_fatbit_size);

    // the size is all the allocated stuff, not including the struct itself
    size_t size = queue_size + 63
                  + bStateSize + tStateSize
                  + fullStateSize + 63 /* cacheline padding */
                  + proto->handledKeyFatbitSize /* handled roles */
                  + activeQueueArraySize /* active queue array */
                  + 2 * deduperLogSize /* need odd and even logs */
                  + 2 * deduperLogSize /* ditto som logs */
                  + 2 * sizeof(u64a) * deduperCount /* start offsets for som */
                  + anchored_literal_region_size + qmpq_size
                  + delay_region_size
                  + som_store_size
                  + som_now_size
                  + som_attempted_size
                  + som_attempted_store_size + 15;

    /* the struct plus the allocated stuff plus padding for cacheline
     * alignment */
    const size_t alloc_size = sizeof(struct hs_scratch) + size + 256;
    s_tmp = hs_scratch_alloc(alloc_size);
    hs_error_t err = hs_check_alloc(s_tmp);
    if (err != HS_SUCCESS) {
        hs_scratch_free(s_tmp);
        *scratch = NULL;
        return err;
    }

    memset(s_tmp, 0, alloc_size);
    s = ROUNDUP_PTR(s_tmp, 64);
    DEBUG_PRINTF("allocated %zu bytes at %p but realigning to %p\n", alloc_size, s_tmp, s);
    DEBUG_PRINTF("sizeof %zu\n", sizeof(struct hs_scratch));
    *s = *proto;

    s->magic = SCRATCH_MAGIC;
    s->in_use = 0;
    s->scratchSize = alloc_size;
    s->scratch_alloc = (char *)s_tmp;
    s->fdr_conf = NULL;

    // each of these is at an offset from the previous
    char *current = (char *)s + sizeof(*s);

    // align current so that the following arrays are naturally aligned: this
    // is accounted for in the padding allocated
    current = ROUNDUP_PTR(current, 8);

    s->queues = (struct mq *)current;
    current += queue_size;

    assert(ISALIGNED_N(current, 8));
    s->som_store = (u64a *)current;
    current += som_store_size;

    s->som_attempted_store = (u64a *)current;
    current += som_attempted_store_size;

    current = ROUNDUP_PTR(current, alignof(struct fatbit *));
    s->delay_slots = (struct fatbit **)current;
    current += sizeof(struct fatbit *) * DELAY_SLOT_COUNT;
    current = ROUNDUP_PTR(current, alignof(struct fatbit));
    for (u32 i = 0; i < DELAY_SLOT_COUNT; i++) {
        s->delay_slots[i] = (struct fatbit *)current;
        assert(ISALIGNED(s->delay_slots[i]));
        current += proto->delay_fatbit_size;
    }

    current = ROUNDUP_PTR(current, alignof(struct fatbit *));
    s->al_log = (struct fatbit **)current;
    current += sizeof(struct fatbit *) * anchored_literal_region_len;
    current = ROUNDUP_PTR(current, alignof(struct fatbit));
    for (u32 i = 0; i < anchored_literal_region_len; i++) {
        s->al_log[i] = (struct fatbit *)current;
        assert(ISALIGNED(s->al_log[i]));
        current += anchored_literal_fatbit_size;
    }

    current = ROUNDUP_PTR(current, 8);
    s->catchup_pq.qm = (struct queue_match *)current;
    current += qmpq_size;

    s->bstate = (char *)current;
    s->bStateSize = bStateSize;
    current += bStateSize;

    s->tstate = (char *)current;
    s->tStateSize = tStateSize;
    current += tStateSize;

    current = ROUNDUP_PTR(current, 64);

    assert(ISALIGNED_N(current, 8));
    s->deduper.som_start_log[0] = (u64a *)current;
    current += sizeof(u64a) * deduperCount;

    s->deduper.som_start_log[1] = (u64a *)current;
    current += sizeof(u64a) * deduperCount;

    assert(ISALIGNED_N(current, 8));
    s->aqa = (struct fatbit *)current;
    current += activeQueueArraySize;

    s->handled_roles = (struct fatbit *)current;
    current += proto->handledKeyFatbitSize;

    s->deduper.log[0] = (struct fatbit *)current;
    current += deduperLogSize;

    s->deduper.log[1] = (struct fatbit *)current;
    current += deduperLogSize;

    s->deduper.som_log[0] = (struct fatbit *)current;
    current += deduperLogSize;

    s->deduper.som_log[1] = (struct fatbit *)current;
    current += deduperLogSize;

    s->som_set_now = (struct fatbit *)current;
    current += som_now_size;

    s->som_attempted_set = (struct fatbit *)current;
    current += som_attempted_size;

    current = ROUNDUP_PTR(current, 64);
    assert(ISALIGNED_CL(current));
    s->fullState = (char *)current;
    s->fullStateSize = fullStateSize;
    current += fullStateSize;

    *scratch = s;

    // Don't get too big for your boots
    assert((size_t)(current - (char *)s) <= alloc_size);

    // Init q->scratch ptr for every queue.
    for (struct mq *qi = s->queues; qi != s->queues + queueCount; ++qi) {
        qi->scratch = s;
    }

    return HS_SUCCESS;
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_alloc_scratch(const hs_database_t *db,
                                     hs_scratch_t **scratch) {
    if (!db || !scratch) {
        return HS_INVALID;
    }

    /* We need to do some real sanity checks on the database as some users mmap
     * in old deserialised databases, so this is the first real opportunity we
     * have to make sure it is sane.
     */
    hs_error_t rv = dbIsValid(db);
    if (rv != HS_SUCCESS) {
        return rv;
    }

    /* We can also sanity-check the scratch parameter: if it points to an
     * existing scratch area, that scratch should have valid magic bits. */
    if (*scratch != NULL) {
        /* has to be aligned before we can do anything with it */
        if (!ISALIGNED_CL(*scratch)) {
            return HS_INVALID;
        }
        if ((*scratch)->magic != SCRATCH_MAGIC) {
            return HS_INVALID;
        }
        if (markScratchInUse(*scratch)) {
            return HS_SCRATCH_IN_USE;
        }
    }

    const struct RoseEngine *rose = hs_get_bytecode(db);
    int resize = 0;

    hs_scratch_t *proto;
    hs_scratch_t *proto_tmp = hs_scratch_alloc(sizeof(struct hs_scratch) + 256);
    hs_error_t proto_ret = hs_check_alloc(proto_tmp);
    if (proto_ret != HS_SUCCESS) {
        hs_scratch_free(proto_tmp);
        if (*scratch) {
            hs_scratch_free((*scratch)->scratch_alloc);
        }
        *scratch = NULL;
        return proto_ret;
    }

    proto = ROUNDUP_PTR(proto_tmp, 64);

    if (*scratch) {
        *proto = **scratch;
    } else {
        memset(proto, 0, sizeof(*proto));
        resize = 1;
    }
    proto->scratch_alloc = (char *)proto_tmp;

    if (rose->anchoredDistance > proto->anchored_literal_region_len) {
        resize = 1;
        proto->anchored_literal_region_len = rose->anchoredDistance;
    }

    if (rose->anchored_fatbit_size > proto->anchored_literal_fatbit_size) {
        resize = 1;
        proto->anchored_literal_fatbit_size = rose->anchored_fatbit_size;
    }

    if (rose->delay_fatbit_size > proto->delay_fatbit_size) {
        resize = 1;
        proto->delay_fatbit_size = rose->delay_fatbit_size;
    }

    if (rose->handledKeyFatbitSize > proto->handledKeyFatbitSize) {
        resize = 1;
        proto->handledKeyFatbitSize = rose->handledKeyFatbitSize;
    }

    if (rose->tStateSize > proto->tStateSize) {
        resize = 1;
        proto->tStateSize = rose->tStateSize;
    }

    u32 som_store_count = rose->somLocationCount;
    if (som_store_count > proto->som_store_count) {
        resize = 1;
        proto->som_store_count = som_store_count;
    }

    if (rose->somLocationFatbitSize > proto->som_fatbit_size) {
        resize = 1;
        proto->som_fatbit_size = rose->somLocationFatbitSize;
    }

    u32 queueCount = rose->queueCount;
    if (queueCount > proto->queueCount) {
        resize = 1;
        proto->queueCount = queueCount;
    }

    if (rose->activeQueueArraySize > proto->activeQueueArraySize) {
        resize = 1;
        proto->activeQueueArraySize = rose->activeQueueArraySize;
    }

    u32 bStateSize = 0;
    if (rose->mode == HS_MODE_BLOCK) {
        bStateSize = rose->stateOffsets.end;
    } else if (rose->mode == HS_MODE_VECTORED) {
        /* vectoring database require a full stream state (inc header) */
        bStateSize = sizeof(struct hs_stream) + rose->stateOffsets.end;
    }

    if (bStateSize > proto->bStateSize) {
        resize = 1;
        proto->bStateSize = bStateSize;
    }

    u32 fullStateSize = rose->scratchStateSize;
    if (fullStateSize > proto->fullStateSize) {
        resize = 1;
        proto->fullStateSize = fullStateSize;
    }

    if (rose->dkeyCount > proto->deduper.dkey_count) {
        resize = 1;
        proto->deduper.dkey_count = rose->dkeyCount;
        proto->deduper.log_size = rose->dkeyLogSize;
    }

    if (resize) {
        if (*scratch) {
            hs_scratch_free((*scratch)->scratch_alloc);
        }

        hs_error_t alloc_ret = alloc_scratch(proto, scratch);
        hs_scratch_free(proto_tmp); /* kill off temp used for sizing */
        if (alloc_ret != HS_SUCCESS) {
            *scratch = NULL;
            return alloc_ret;
        }
    } else {
        hs_scratch_free(proto_tmp); /* kill off temp used for sizing */
        unmarkScratchInUse(*scratch);
    }

    assert(!(*scratch)->in_use);
    return HS_SUCCESS;
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_clone_scratch(const hs_scratch_t *src,
                                     hs_scratch_t **dest) {
    if (!dest || !src || !ISALIGNED_CL(src) || src->magic != SCRATCH_MAGIC) {
        return HS_INVALID;
    }

    *dest = NULL;
    hs_error_t ret = alloc_scratch(src, dest);
    if (ret != HS_SUCCESS) {
        *dest = NULL;
        return ret;
    }

    assert(!(*dest)->in_use);
    return HS_SUCCESS;
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_free_scratch(hs_scratch_t *scratch) {
    if (scratch) {
        /* has to be aligned before we can do anything with it */
        if (!ISALIGNED_CL(scratch)) {
            return HS_INVALID;
        }
        if (scratch->magic != SCRATCH_MAGIC) {
            return HS_INVALID;
        }
        if (markScratchInUse(scratch)) {
            return HS_SCRATCH_IN_USE;
        }

        scratch->magic = 0;
        assert(scratch->scratch_alloc);
        DEBUG_PRINTF("scratch %p is really at %p : freeing\n", scratch,
                     scratch->scratch_alloc);
        hs_scratch_free(scratch->scratch_alloc);
    }

    return HS_SUCCESS;
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_scratch_size(const hs_scratch_t *scratch, size_t *size) {
    if (!size || !scratch || !ISALIGNED_CL(scratch) ||
        scratch->magic != SCRATCH_MAGIC) {
        return HS_INVALID;
    }

    *size = scratch->scratchSize;

    return HS_SUCCESS;
}
