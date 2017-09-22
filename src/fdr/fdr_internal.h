/*
 * Copyright (c) 2015-2017, Intel Corporation
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
 * \brief FDR literal matcher: data structures.
 */

#ifndef FDR_INTERNAL_H
#define FDR_INTERNAL_H

#include "ue2common.h"
#include "hwlm/hwlm.h" // for hwlm_group_t, HWLMCallback

struct hs_scratch;

typedef enum {
    NOT_CAUTIOUS, //!< not near a boundary (quantify?)
    VECTORING     //!< potentially vectoring
} CautionReason;

/** \brief number of different ids that can be triggered by floods of any given
 * character. */
#define FDR_FLOOD_MAX_IDS 16

struct FDRFlood {
    hwlm_group_t allGroups; //!< all the groups or'd together
    u32 suffix;

    /** \brief 0 to FDR_FLOOD_MAX_IDS-1 ids that are generated once per char on
     * a flood.
     * If larger we won't handle this through the flood path at all. */
    u16 idCount;

    u32 ids[FDR_FLOOD_MAX_IDS]; //!< the ids
    hwlm_group_t groups[FDR_FLOOD_MAX_IDS]; //!< group ids to go with string ids
};

/** \brief FDR structure.
 *
 * 1. struct as-is
 * 2. primary matching table
 * 3. confirm stuff
 */
struct FDR {
    u32 engineID;
    u32 size;
    u32 maxStringLen;
    u32 numStrings;
    u32 confOffset;
    u32 floodOffset;
    u8 stride; /* stride - how frequently the data is consulted by the first
                * stage matcher */
    u8 domain; /* number of bits used to index into main FDR table. This value
                * is used only of debugging/asserts. */
    u16 domainMask; /* pre-computed domain mask */
    u32 tabSize; /* pre-computed hashtable size in bytes */
    m128 start; /* initial start state to use at offset 0. The state has been
                 * set up based on the min length of buckets to reduce the need
                 * for pointless confirms. */
};

/** \brief FDR runtime arguments.
 *
 * This structure handles read-only things that are passed extensively around
 * the FDR run-time functions. They are set by the API, passed by value into
 * the main function, then a pointer is passed around to all the various
 * sub-functions (confirm & flood). */
struct FDR_Runtime_Args {
    const u8 *buf;
    size_t len;
    const u8 *buf_history;
    size_t len_history;
    size_t start_offset;
    HWLMCallback cb;
    struct hs_scratch *scratch;
    const u8 *firstFloodDetect;
    const u64a histBytes;
};

#endif
