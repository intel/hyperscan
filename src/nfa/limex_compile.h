/*
 * Copyright (c) 2015-2020, Intel Corporation
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

/**
 * \file
 * \brief Main NFA build code.
 */

#ifndef LIMEX_COMPILE_H
#define LIMEX_COMPILE_H

#include "nfagraph/ng_holder.h"
#include "nfagraph/ng_squash.h" // for NFAStateSet
#include "ue2common.h"
#include "util/bytecode_ptr.h"

#include <set>
#include <map>
#include <memory>
#include <unordered_map>
#include <vector>

struct NFA;

namespace ue2 {

struct BoundedRepeatData;
struct CompileContext;

/**
 * \brief Construct a LimEx NFA from an NGHolder.
 *
 * \param g Input NFA graph. Must have state IDs assigned.
 * \param repeats Bounded repeat information, if any.
 * \param reportSquashMap Single-match mode squash map.
 * \param squashMap More general squash map.
 * \param tops Tops and their start vertices,
 * \param zombies The set of zombifying states.
 * \param do_accel Calculate acceleration schemes.
 * \param stateCompression Allow (and calculate masks for) state compression.
 * \param hint If not INVALID_NFA, this allows a particular LimEx NFA model
               to be requested.
 * \param cc Compile context.
 * \return a built NFA, or nullptr if no NFA could be constructed for this
 * graph.
 */
bytecode_ptr<NFA> generate(NGHolder &g,
            const std::unordered_map<NFAVertex, u32> &states,
            const std::vector<BoundedRepeatData> &repeats,
            const std::unordered_map<NFAVertex, NFAStateSet> &reportSquashMap,
            const std::unordered_map<NFAVertex, NFAStateSet> &squashMap,
            const std::map<u32, std::set<NFAVertex>> &tops,
            const std::set<NFAVertex> &zombies,
            bool do_accel,
            bool stateCompression,
            bool &fast,
            u32 hint,
            const CompileContext &cc);

/**
 * \brief For a given graph, count the number of accelerable states it has.
 *
 * Note that this number may be greater than the number that are actually
 * implementable.
 */
u32 countAccelStates(NGHolder &h,
            const std::unordered_map<NFAVertex, u32> &states,
            const std::vector<BoundedRepeatData> &repeats,
            const std::unordered_map<NFAVertex, NFAStateSet> &reportSquashMap,
            const std::unordered_map<NFAVertex, NFAStateSet> &squashMap,
            const std::map<u32, std::set<NFAVertex>> &tops,
            const std::set<NFAVertex> &zombies,
            const CompileContext &cc);

} // namespace ue2

#endif
