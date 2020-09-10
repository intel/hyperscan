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
 * \brief Limex NFA construction code.
 */

#ifndef NG_LIMEX_H
#define NG_LIMEX_H

#include "ue2common.h"
#include "som/som.h"
#include "util/bytecode_ptr.h"

#include <map>
#include <memory>
#include <vector>

struct NFA;

namespace ue2 {

class CharReach;
class NG;
class NGHolder;
class ReportManager;
struct CompileContext;

/**
 * \brief Determine if the given graph is implementable as an NFA.
 *
 * Returns zero if the NFA is not implementable (usually because it has too
 * many states for any of our models). Otherwise returns the number of states.
 *
 * ReportManager is used by NFA_SUFFIX and NFA_OUTFIX only. NFA_PREFIX and
 * NFA_INFIX use unmanaged rose-local reports.
 */
u32 isImplementableNFA(const NGHolder &g, const ReportManager *rm,
                       const CompileContext &cc);

/**
 * \brief Late-stage graph reductions.
 *
 * This will call \ref removeRedundancy and apply its changes to the given
 * holder only if it is implementable afterwards.
 */
void reduceImplementableGraph(NGHolder &g, som_type som,
                              const ReportManager *rm,
                              const CompileContext &cc);

/**
 * \brief For a given graph, count the number of accel states it will have in
 * an implementation.
 *
 * \return the number of accel states, or NFA_MAX_ACCEL_STATES + 1 if an
 * implementation would not be constructible.
 */
u32 countAccelStates(const NGHolder &g, const ReportManager *rm,
                     const CompileContext &cc);

/**
 * \brief Construct an NFA from the given graph.
 *
 * Returns zero if the NFA is not implementable (usually because it has too
 * many states for any of our models). Otherwise returns the number of states.
 *
 * ReportManager is used by NFA_SUFFIX and NFA_OUTFIX only. NFA_PREFIX and
 * NFA_INFIX use unmanaged rose-local reports.
 *
 * Note: this variant of the function allows a model to be specified with the
 * \a hint parameter.
 */
bytecode_ptr<NFA>
constructNFA(const NGHolder &g, const ReportManager *rm,
             const std::map<u32, u32> &fixed_depth_tops,
             const std::map<u32, std::vector<std::vector<CharReach>>> &triggers,
             bool compress_state, bool &fast, const CompileContext &cc);

/**
 * \brief Build a reverse NFA from the graph given, which should have already
 * been reversed.
 *
 * Used for reverse NFAs used in SOM mode.
 */
bytecode_ptr<NFA> constructReversedNFA(const NGHolder &h,
                                       const CompileContext &cc);

#ifndef RELEASE_BUILD

/**
 * \brief Construct an NFA (with model type hint) from the given graph.
 *
 * Returns zero if the NFA is not implementable (usually because it has too
 * many states for any of our models). Otherwise returns the number of states.
 *
 * ReportManager is used by NFA_SUFFIX and NFA_OUTFIX only. NFA_PREFIX and
 * NFA_INFIX use unmanaged rose-local reports.
 *
 * Note: this variant of the function allows a model to be specified with the
 * \a hint parameter.
 */
bytecode_ptr<NFA>
constructNFA(const NGHolder &g, const ReportManager *rm,
             const std::map<u32, u32> &fixed_depth_tops,
             const std::map<u32, std::vector<std::vector<CharReach>>> &triggers,
             bool compress_state, bool &fast, u32 hint, const CompileContext &cc);

/**
 * \brief Build a reverse NFA (with model type hint) from the graph given,
 * which should have already been reversed.
 *
 * Used for reverse NFAs used in SOM mode.
 */
bytecode_ptr<NFA> constructReversedNFA(const NGHolder &h, u32 hint,
                                       const CompileContext &cc);

#endif // RELEASE_BUILD

} // namespace ue2

#endif // NG_METEOR_H
