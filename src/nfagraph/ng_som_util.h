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
 * \brief Utility functions related to SOM ("Start of Match").
 */

#ifndef NG_SOM_UTIL_H
#define NG_SOM_UTIL_H

#include "ng_util.h"
#include "util/depth.h"

#include <map>
#include <unordered_map>
#include <vector>

namespace ue2 {

class NGHolder;

/**
 * Returns min/max distance from start of match, index by vertex_id.
 */
std::vector<DepthMinMax> getDistancesFromSOM(const NGHolder &g);

/**
 * Returns true if the first match by end-offset must always be the first match
 * by start-offset.
 */
bool firstMatchIsFirst(const NGHolder &p);

struct smgb_cache : public mbsb_cache {
    explicit smgb_cache(const NGHolder &gg) : mbsb_cache(gg) {}
    std::map<NFAVertex, bool> smgb;
};

bool somMayGoBackwards(NFAVertex u, const NGHolder &g,
                       const std::unordered_map<NFAVertex, u32> &region_map,
                       smgb_cache &cache);

/**
 * Returns true if matching 'sent' causes all tail states in the main graph \a
 * g to go dead. A tail state is any state with a region greater than
 * \a last_head_region.
 *
 * - The graph \a sent must be a "kinda-DAG", where the only back-edges present
 *   are self-loops.
 * - If the result is false, \a bad_region will be updated with the smallest
 *   region ID associated with a tail state that is still on.
 */
bool sentClearsTail(const NGHolder &g,
                    const std::unordered_map<NFAVertex, u32> &region_map,
                    const NGHolder &sent, u32 last_head_region,
                    u32 *bad_region);

} // namespace ue2

#endif // NG_SOM_UTIL_H
