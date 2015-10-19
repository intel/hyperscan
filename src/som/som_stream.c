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
 * \brief SOM streaming runtime code.
 *
 * Code in this file handles storing and loading SOM slot information from
 * stream state.
 */

#include "scratch.h"
#include "som_stream.h"
#include "rose/rose_internal.h"
#include "util/multibit.h"

// Sentinel values stored in stream state and used to represent an SOM distance
// that is too far in the past to be stored in the available space in stream
// state.

#define SOM_SENTINEL_LARGE  (~0ull)
#define SOM_SENTINEL_MEDIUM (~0u)
#define SOM_SENTINEL_SMALL  ((u16)~0u)

static really_inline
void storeSomValue(void *stream_som_store, u64a som_value,
                   u64a stream_offset, u8 som_size) {
    // Special case for sentinel value.
    if (som_value == SOM_SENTINEL_LARGE) {
        switch (som_size) {
        case 2:
            *(u16 *)stream_som_store = SOM_SENTINEL_SMALL;
            break;
        case 4:
            *(u32 *)stream_som_store = SOM_SENTINEL_MEDIUM;
            break;
        case 8:
            *(u64a *)stream_som_store = SOM_SENTINEL_LARGE;
            break;
        default:
            break;
        }
        return;
    }

    assert(som_value <= stream_offset);
    u64a rel_offset = stream_offset - som_value;
    DEBUG_PRINTF("rel_offset=%llu\n", rel_offset);

    switch (som_size) {
    case 2:
        rel_offset = MIN(rel_offset, SOM_SENTINEL_SMALL);
        assert(ISALIGNED_N(stream_som_store, alignof(u16)));
        *(u16 *)stream_som_store = rel_offset;
        break;
    case 4:
        rel_offset = MIN(rel_offset, SOM_SENTINEL_MEDIUM);
        assert(ISALIGNED_N(stream_som_store, alignof(u32)));
        *(u32 *)stream_som_store = rel_offset;
        break;
    case 8:
        assert(ISALIGNED_N(stream_som_store, alignof(u64a)));
        *(u64a *)stream_som_store = rel_offset;
        break;
    default:
        assert(0);
        break;
    }
}

void storeSomToStream(struct hs_scratch *scratch, const u64a offset) {
    assert(scratch);
    DEBUG_PRINTF("stream offset %llu\n", offset);

    struct core_info *ci = &scratch->core_info;
    const struct RoseEngine *rose = ci->rose;

    const u32 som_store_count = rose->somLocationCount;
    assert(som_store_count); // Caller should ensure that we have work to do.

    u8 *som_store_valid = (u8 *)ci->state + rose->stateOffsets.somValid;
    char *stream_som_store = ci->state + rose->stateOffsets.somLocation;
    const u64a *som_store = scratch->som_store;
    const u8 som_size = rose->somHorizon;

    for (u32 i = mmbit_iterate(som_store_valid, som_store_count, MMB_INVALID);
         i != MMB_INVALID;
         i = mmbit_iterate(som_store_valid, som_store_count, i)) {
        DEBUG_PRINTF("storing %llu in %u\n", som_store[i], i);
        storeSomValue(stream_som_store + (i * som_size), som_store[i],
                      offset, som_size);
    }
}

static really_inline
u64a loadSomValue(const void *stream_som_store, u64a stream_offset,
                  u8 som_size) {
    u64a rel_offset;
    switch (som_size) {
    case 2:
        assert(ISALIGNED_N(stream_som_store, alignof(u16)));
        rel_offset = *(const u16 *)stream_som_store;
        if (rel_offset == SOM_SENTINEL_SMALL) {
            return SOM_SENTINEL_LARGE;
        }
        break;
    case 4:
        assert(ISALIGNED_N(stream_som_store, alignof(u32)));
        rel_offset = *(const u32 *)stream_som_store;
        if (rel_offset == SOM_SENTINEL_MEDIUM) {
            return SOM_SENTINEL_LARGE;
        }
        break;
    case 8:
        assert(ISALIGNED_N(stream_som_store, alignof(u64a)));
        rel_offset = *(const u64a *)stream_som_store;
        break;
    default:
        assert(0);
        rel_offset = 0;
        break;
    }

    DEBUG_PRINTF("rel_offset=%llu\n", rel_offset);
    return stream_offset - rel_offset;
}

void loadSomFromStream(struct hs_scratch *scratch, const u64a offset) {
    assert(scratch);
    DEBUG_PRINTF("stream offset %llu\n", offset);

    struct core_info *ci = &scratch->core_info;
    const struct RoseEngine *rose = ci->rose;

    const u32 som_store_count = rose->somLocationCount;
    assert(som_store_count); // Caller should ensure that we have work to do.

    const u8 *som_store_valid = (u8 *)ci->state + rose->stateOffsets.somValid;
    const char *stream_som_store = ci->state + rose->stateOffsets.somLocation;
    u64a *som_store = scratch->som_store;
    const u8 som_size = rose->somHorizon;

    for (u32 i = mmbit_iterate(som_store_valid, som_store_count, MMB_INVALID);
         i != MMB_INVALID;
         i = mmbit_iterate(som_store_valid, som_store_count, i)) {
        som_store[i] = loadSomValue(stream_som_store + (i*som_size), offset,
                                    som_size);
        DEBUG_PRINTF("loaded %llu from %u\n", som_store[i], i);
    }
}
