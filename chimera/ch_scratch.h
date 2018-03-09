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
 * \brief Scratch and associated data structures.
 *
 * This header gets pulled into many places (many deep, slow to compile
 * places). Try to keep the included headers under control.
 */

#ifndef CH_SCRATCH_H_
#define CH_SCRATCH_H_

#include "ch_common.h"
#include "ch_runtime.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define CH_SCRATCH_MAGIC 0x554F4259 //!< Magic number stored in \ref ch_scratch

struct queue_item {
    int from; /** \brief used to store the start location. */
    int to; /** \brief used to store the current location. */
    u32 id; /**< pattern index. */
};

struct match_pq {
    struct queue_item *item;
    u32 size; /**< current size of the priority queue */
};

/** \brief Information about a pattern stored at runtime when a match is
 * encountered. */
struct ch_patterndata {
    struct ch_capture *match; //!< buffered group info
    u32 groupCount; //!< number of capturing groups
    u32 scanStart; //!< start of match window (still to be single-scanned).
};

/** \brief Scratch space header for Chimera. */
struct ch_scratch {
    u32 magic; //!< must be \ref CH_SCRATCH_MAGIC
    u8 in_use; /**< non-zero when being used by an API call. */
    struct hs_scratch *multi_scratch; //!< for hyperscan scatch.
    int *ovector; //!< maximally-sized ovector for PCRE usage.
    struct ch_capture *captured; //!< max-sized capture group struct.
    u8 *active; //!< active multibit.
    struct ch_patterndata *patternData; //!< per-pattern match data, indexed by
                                        // pattern ID.
    struct match_pq pq; //!< priority queue to ensure matching ordering
    u32 patternCount; //!< number of patterns, used to size active multibit
    u32 activeSize;   //!< size of active multibit
    u32 maxCaptureGroups; //!< largest num of capturing groups required
    u32 scratchSize; //!< size of allocation
    int ret;  //!< return value in Hyperscan callback
    char *scratch_alloc; /* user allocated scratch object */
};

/**
 * \brief Mark scratch as in use.
 *
 * Returns non-zero if it was already in use, zero otherwise.
 */
static really_inline
char markScratchInUse(struct ch_scratch *scratch) {
    DEBUG_PRINTF("marking scratch as in use\n");
    assert(scratch && scratch->magic == CH_SCRATCH_MAGIC);
    if (scratch->in_use) {
        DEBUG_PRINTF("scratch already in use!\n");
        return 1;
    }
    scratch->in_use = 1;
    return 0;
}

/**
 * \brief Mark scratch as no longer in use.
 */
static really_inline
void unmarkScratchInUse(struct ch_scratch *scratch) {
    DEBUG_PRINTF("marking scratch as not in use\n");
    assert(scratch && scratch->magic == CH_SCRATCH_MAGIC);
    assert(scratch->in_use == 1);
    scratch->in_use = 0;
}

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* CH_SCRATCH_H_ */
