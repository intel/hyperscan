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
 * \brief Multibit: data structures.
 *
 * If all you need is the sizes of multibit's few structures, then including
 * this file is a much better idea than including all of multibit.h.
 */
#ifndef MULTIBIT_INTERNAL_H
#define MULTIBIT_INTERNAL_H

#include "ue2common.h"

#ifdef __cplusplus
extern "C" {
#endif

/** \brief Sentinel value meaning "no key found". */
#define MMB_INVALID 0xffffffffu

typedef u64a MMB_TYPE; /**< Basic block type for mmbit operations. */
#define MMB_MAX_LEVEL 6 /**< Maximum level in the mmbit pyramid. */

/** \brief Maximum number of keys (bits) in a multibit. */
#define MMB_MAX_BITS (1U << 31)

/** \brief Sparse iterator record type.
 *
 * A sparse iterator is a tree of these records, where val identifies the
 * offset of the result for leaf nodes and points to the next record for
 * intermediate nodes. Built by the code in multibit_build.cpp.
 */
struct mmbit_sparse_iter {
    MMB_TYPE mask;
    u32 val;
};

/** \brief Sparse iterator runtime state type.
 *
 * An array of these records (one per "level" in the multibit pyramid) is used
 * to store the current iteration state.
 */
struct mmbit_sparse_state {
    MMB_TYPE mask; //!< \brief masked last block read at this level.
    u32 itkey;     //!< \brief iterator offset for this level.
};

/** \brief Maximum number of \ref mmbit_sparse_state that could be needed. */
#define MAX_SPARSE_ITER_STATES (6 + 1)

/**
 * \brief Runtime equivalent of mmbit_size() for validation at scan/load time.
 *
 * Computes the minimum number of bytes needed for a multibit with
 * \a total_bits entries, matching the compile-time mmbit_size() from
 * multibit_build.h.  This function is safe to call from runtime code
 * (C translation units) that cannot include the compile-time C++ headers.
 *
 * The "rt_" prefix avoids collisions with the compile-time mmbit_size()
 * defined in multibit_build.h / multibit_build.cpp.
 */
static really_inline
u32 rt_mmbit_size(u32 total_bits) {
    if (total_bits == 0) {
        return 0;
    }
    /* Reject counts beyond the compile-time limit. */
    if (total_bits > MMB_MAX_BITS) {
        return UINT32_MAX;
    }
    /* Flat model: simple bit vector. */
    if (total_bits <= 256) {
        return (total_bits + 7) / 8;
    }
    /* Hierarchical model: sum of all level block counts * 8 bytes. */
    u64a current_level = 1;
    u64a total = 0;
    while (current_level * 64 < total_bits) {
        total += current_level;
        current_level <<= 6; /* MMB_KEY_SHIFT */
    }
    u64a last_level = ((u64a)total_bits + 63) / 64;
    total += last_level;
    u64a byte_size = total * sizeof(u64a);
    /* Saturate to UINT32_MAX on overflow to guarantee validators reject. */
    if (byte_size > UINT32_MAX) {
        return UINT32_MAX;
    }
    return (u32)byte_size;
}

/**
 * \brief Runtime equivalent of fatbit_size() for validation at scan/load time.
 *
 * Returns the minimum allocation size (in bytes) for a fatbit that stores
 * \a total_bits logical entries.  Uses MIN_FAT_SIZE (== sizeof(struct fatbit))
 * as the floor, matching the compile-time fatbit_size().  The "rt_" prefix
 * avoids collisions with the compile-time fatbit_size().
 */
#ifndef MIN_FAT_SIZE
#define MIN_FAT_SIZE 32
#endif
static really_inline
u32 rt_fatbit_size(u32 total_bits) {
    u32 mmsz = rt_mmbit_size(total_bits);
    return mmsz > MIN_FAT_SIZE ? mmsz : MIN_FAT_SIZE;
}

#ifdef __cplusplus
} // extern "C"
#endif

#endif // MULTIBIT_INTERNAL_H
