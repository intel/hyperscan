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
 * \brief Multibit: build code (for sparse iterators)
 */

#ifndef MULTIBIT_BUILD_H
#define MULTIBIT_BUILD_H

#include "hs_common.h"
#include "multibit_internal.h"
#include "hash.h"

#include <vector>

inline
bool operator==(const mmbit_sparse_iter &a, const mmbit_sparse_iter &b) {
    return a.mask == b.mask && a.val == b.val;
}

namespace std {

template<>
struct hash<mmbit_sparse_iter> {
    size_t operator()(const mmbit_sparse_iter &iter) const {
        return ue2::hash_all(iter.mask, iter.val);
    }
};

} // namespace std

namespace ue2 {

/**
 * \brief Return the size in bytes of a multibit that can store the given
 * number of bits.
 *
 * This will throw a resource limit assertion if the requested mmbit is too
 * large.
 */
u32 mmbit_size(u32 total_bits);

/** \brief Construct a sparse iterator over the values in \a bits for a
 * multibit of size \a total_bits. */
std::vector<mmbit_sparse_iter>
mmbBuildSparseIterator(const std::vector<u32> &bits, u32 total_bits);

struct scatter_plan_raw;

void mmbBuildInitRangePlan(u32 total_bits, u32 begin, u32 end,
                           scatter_plan_raw *out);
void mmbBuildClearPlan(u32 total_bits, scatter_plan_raw *out);

} // namespace ue2

#endif // MULTIBIT_BUILD_H
