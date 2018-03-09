/*
 * Copyright (c) 2018, Intel Corporation
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
 * \brief Chimera: scratch space alloc.
 */

#include <string.h>

#include "allocator.h"
#include "ch.h"
#include "hs.h"
#include "hs_internal.h"
#include "ue2common.h"
#include "ch_alloc.h"
#include "ch_internal.h"
#include "ch_scratch.h"
#include "ch_database.h"

static
size_t getPatternDataSize(const ch_scratch_t *s) {
    size_t numCapturingStructs =
        s->patternCount * (s->maxCaptureGroups + 1);
    return (sizeof(struct ch_patterndata) * s->patternCount) +
           alignof(struct ch_capture) + // padding
           (sizeof(struct ch_capture) * numCapturingStructs);
}

static
void initPatternData(const ch_scratch_t *s) {
    // ch_capture array is aligned, directly after the patterndata array.
    char *ptr = (char *)s->patternData +
            (sizeof(struct ch_patterndata) * s->patternCount);
    struct ch_capture *cap = (struct ch_capture *)
            (ROUNDUP_PTR(ptr, alignof(struct ch_capture)));

    for (u32 i = 0; i < s->patternCount; i++) {
        struct ch_patterndata *pd = &s->patternData[i];
        pd->match = cap;
        DEBUG_PRINTF("pattern %u: pd=%p, match=%p\n", i, pd, pd->match);
        cap += (s->maxCaptureGroups + 1);
    }
}

static
ch_error_t alloc_scratch(const ch_scratch_t *proto, ch_scratch_t **scratch) {
    size_t ovectorSize = (proto->maxCaptureGroups + 1) * sizeof(int) * 3;
    size_t capturedSize =
        sizeof(struct ch_capture) * (proto->maxCaptureGroups + 1);
    size_t patternDataSize = getPatternDataSize(proto);
    size_t activeSize = proto->activeSize;
    size_t queueSize = proto->patternCount * sizeof(struct queue_item);

    // max padding for alignment below.
    size_t padding = alignof(int) + alignof(struct ch_capture) +
                     alignof(struct ch_patterndata) +
                     alignof(struct queue_item);

    size_t allocSize = sizeof(ch_scratch_t) + ovectorSize + capturedSize +
                       patternDataSize + activeSize + queueSize + padding
                       + 256; /* padding for cacheline alignment */
    ch_scratch_t *s;
    ch_scratch_t *s_tmp = ch_scratch_alloc(allocSize);
    ch_error_t err = ch_check_alloc(s_tmp);
    if (err != CH_SUCCESS) {
        ch_scratch_free(s_tmp);
        *scratch = NULL;
        return err;
    }

    memset(s_tmp, 0, allocSize);
    s = ROUNDUP_PTR(s_tmp, 64);
    // Set ordinary members.
    *s = *proto;

    s->magic = CH_SCRATCH_MAGIC;
    s->in_use = 0;
    s->scratch_alloc = (char *)s_tmp;

    // Set pointers internal to allocation.

    char *ptr = (char *)s + sizeof(*s);
    ptr = ROUNDUP_PTR(ptr, alignof(int));
    s->ovector = (int *)ptr;
    ptr += ovectorSize;

    ptr = ROUNDUP_PTR(ptr, alignof(struct ch_capture));
    s->captured = (struct ch_capture *)ptr;
    ptr += capturedSize;

    ptr = ROUNDUP_PTR(ptr, alignof(struct ch_patterndata));
    s->patternData = (struct ch_patterndata *)ptr;
    ptr += patternDataSize;

    // Pre-fill pattern data, setting captureOffsets
    initPatternData(s);

    ptr = ROUNDUP_PTR(ptr, alignof(struct queue_item));
    s->pq.item = (struct queue_item *)ptr;
    ptr += queueSize;

    s->active = (u8 *)ptr;

    // Store size.
    s->scratchSize = allocSize;

    // We should never overrun our allocation.
    assert((ptr + activeSize) - (char *)s <= (ptrdiff_t)allocSize);

    *scratch = s;
    return CH_SUCCESS;
}

HS_PUBLIC_API
ch_error_t HS_CDECL ch_alloc_scratch(const ch_database_t *hydb,
                                     ch_scratch_t **scratch) {
    if (!hydb || !scratch) {
        DEBUG_PRINTF("invalid args\n");
        return CH_INVALID;
    }

    DEBUG_PRINTF("hydb=%p, &scratch=%p\n", hydb, scratch);
    ch_error_t rv = hydbIsValid(hydb);
    if (rv != CH_SUCCESS) {
        DEBUG_PRINTF("invalid database\n");
        return rv;
    }

    if (*scratch != NULL) {
        /* has to be aligned before we can do anything with it */
        if (!ISALIGNED_CL(*scratch)) {
            return CH_INVALID;
        }
        if ((*scratch)->magic != CH_SCRATCH_MAGIC) {
            return CH_INVALID;
        }
        if (markScratchInUse(*scratch)) {
            return CH_SCRATCH_IN_USE;
        }
    }

    // We allocate a prototype of the scratch header to do our sizing with.
    ch_scratch_t *proto;
    ch_scratch_t *proto_tmp = ch_scratch_alloc(sizeof(ch_scratch_t) + 256);
    ch_error_t proto_ret = ch_check_alloc(proto_tmp);
    if (proto_ret != CH_SUCCESS) {
        ch_scratch_free(proto_tmp);
        ch_scratch_free(*scratch);
        *scratch = NULL;
        return proto_ret;
    }

    proto = ROUNDUP_PTR(proto_tmp, 64);

    int resize = 0;
    if (*scratch) {
        *proto = **scratch;
    } else {
        memset(proto, 0, sizeof(*proto));
        resize = 1;
    }
    proto->scratch_alloc = (char *)proto_tmp;

    const struct ch_bytecode *db = ch_get_bytecode(hydb);

    if (db->maxCaptureGroups > proto->maxCaptureGroups) {
        proto->maxCaptureGroups = db->maxCaptureGroups;
        resize = 1;
    }

    if (db->patternCount > proto->patternCount) {
        proto->patternCount = db->patternCount;
        proto->activeSize = db->activeSize;
        resize = 1;
    }

    if (resize) {
        if (*scratch) {
            ch_scratch_free((*scratch)->scratch_alloc);
        }

        ch_error_t alloc_ret = alloc_scratch(proto, scratch);
        ch_scratch_free(proto_tmp);
        if (alloc_ret != CH_SUCCESS) {
            *scratch = NULL;
            return alloc_ret;
        }
    } else {
        ch_scratch_free(proto_tmp);
        unmarkScratchInUse(*scratch);
    }

    if (db->flags & CHIMERA_FLAG_NO_MULTIMATCH) {
        (*scratch)->multi_scratch = NULL;
        return CH_SUCCESS;
    }

    // We may still have to realloc the underlying Hyperscan scratch.
    rv = hs_alloc_scratch(getHyperscanDatabase(db),
                           &(*scratch)->multi_scratch);
    if (rv != HS_SUCCESS) {
        DEBUG_PRINTF("hs_alloc_scratch for multi_scratch failed\n");
        hs_free_scratch((*scratch)->multi_scratch);
        ch_scratch_free((*scratch)->scratch_alloc);
        *scratch = NULL;
        return rv;
    }

    return CH_SUCCESS;
}

HS_PUBLIC_API
ch_error_t HS_CDECL ch_clone_scratch(const ch_scratch_t *src,
                                     ch_scratch_t **dest) {
    if (!dest || !src || !ISALIGNED_CL(src) ||
        src->magic != CH_SCRATCH_MAGIC) {
        DEBUG_PRINTF("scratch invalid\n");
        return CH_INVALID;
    }

    ch_error_t ret = alloc_scratch(src, dest);
    if (ret != CH_SUCCESS) {
        DEBUG_PRINTF("alloc_scratch failed\n");
        *dest = NULL;
        return ret;
    }

    if (src->multi_scratch) {
        (*dest)->multi_scratch = NULL;
        ret = hs_clone_scratch(src->multi_scratch, &(*dest)->multi_scratch);
        if (ret != HS_SUCCESS) {
            DEBUG_PRINTF("hs_clone_scratch(multi_scratch,...) failed\n");
            ch_scratch_free(*dest);
            return ret;
        }
    }

    return CH_SUCCESS;
}

HS_PUBLIC_API
ch_error_t HS_CDECL ch_free_scratch(ch_scratch_t *scratch) {
    ch_error_t ret = CH_SUCCESS;
    if (scratch) {
        /* has to be aligned before we can do anything with it */
        if (!ISALIGNED_CL(scratch)) {
            return CH_INVALID;
        }
        if (scratch->magic != CH_SCRATCH_MAGIC) {
            return CH_INVALID;
        }
        if (markScratchInUse(scratch)) {
            return CH_SCRATCH_IN_USE;
        }

        if (scratch->multi_scratch) {
            ret = hs_free_scratch(scratch->multi_scratch);
        }

        scratch->magic = 0;
        assert(scratch->scratch_alloc);
        DEBUG_PRINTF("scratch %p is really at %p : freeing\n", scratch,
                     scratch->scratch_alloc);
        ch_scratch_free(scratch->scratch_alloc);
    }

    return ret;
}

/** Not public, but used for info from our internal tools. Note that in the
 * hybrid matcher the scratch is definitely not a contiguous memory region. */
HS_PUBLIC_API
ch_error_t HS_CDECL ch_scratch_size(const ch_scratch_t *scratch, size_t *size) {
    ch_error_t ret = CH_SUCCESS;
    if (!size || !scratch || !ISALIGNED_CL(scratch) ||
        scratch->magic != CH_SCRATCH_MAGIC) {
        return CH_INVALID;
    } else {
        size_t multi_size = 0;

        if (scratch->multi_scratch) {
            ret = hs_scratch_size(scratch->multi_scratch, &multi_size);
        }
        if (ret) {
            multi_size = 0;
        }

        *size = scratch->scratchSize + multi_size;
    }

    return ret;
}
