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

#ifdef __cplusplus
} // extern "C"
#endif

#endif // MULTIBIT_INTERNAL_H
