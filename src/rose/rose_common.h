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

#ifndef ROSE_COMMON_H
#define ROSE_COMMON_H

// Common defs available to build-time clients as well as runtime.

#define ROSE_BOUND_INF (~0U)
#define MAX_MASK2_WIDTH 32

// Max block width to use the combined small-block matcher on, instead of
// running the floating and anchored tables.
#define ROSE_SMALL_BLOCK_LEN 32

/** \brief Length in bytes of a reach bitvector, used by the lookaround code. */
#define REACH_BITVECTOR_LEN 32

/** \brief Length in bytes of a reach bitvector for multi-path lookaround. */
#define MULTI_REACH_BITVECTOR_LEN 256

/**
 * \brief The max offset from the leftmost byte to the rightmost byte in
 * multi-path lookaround.
 */
#define MULTIPATH_MAX_LEN 16

/** \brief Value used to represent an invalid Rose program offset. */
#define ROSE_INVALID_PROG_OFFSET 0

#endif // ROSE_COMMON_H
