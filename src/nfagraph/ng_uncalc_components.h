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
 * \brief NFA graph merging ("uncalc")
 */

#ifndef NG_UNCALC_COMPONENTS_H
#define NG_UNCALC_COMPONENTS_H

#include <map>
#include <vector>

#include "nfagraph/ng_graph.h"
#include "util/ue2_containers.h"

namespace ue2 {

struct CompileContext;
struct Grey;
class NGHolder;
class ReportManager;

/**
 * \brief Returns the common prefix length for a pair of graphs.
 *
 * The CPL is calculated based the topological ordering given by the state
 * indices for each graph.
 */
u32 commonPrefixLength(const NGHolder &ga,
                       const ue2::unordered_map<NFAVertex, u32> &a_state_ids,
                       const NGHolder &gb,
                       const ue2::unordered_map<NFAVertex, u32> &b_state_ids);

/**
 * \brief Merge the group of graphs in \p cluster where possible.
 *
 * The (from, to) mapping of merged graphs is returned in \p merged.
 */
void mergeNfaCluster(const std::vector<NGHolder *> &cluster,
                     const ReportManager *rm,
                     std::map<NGHolder *, NGHolder *> &merged,
                     const CompileContext &cc);

/**
 * \brief Merge graph \p ga into graph \p gb.
 *
 * Returns false on failure. On success, \p gb is reduced via \ref
 * reduceImplementableGraph and renumbered.
 */
bool mergeNfaPair(NGHolder &ga, NGHolder &gb, const ReportManager *rm,
                  const CompileContext &cc);

} // namespace ue2

#endif
