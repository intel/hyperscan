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
 * \brief Rose Build: functions for reducing the number of engines in a Rose
 * graph through merging or deduplicating engines.
 */

#ifndef ROSE_BUILD_MERGE_H
#define ROSE_BUILD_MERGE_H

#include "rose_graph.h"

#include <deque>
#include <set>

namespace ue2 {

class NGHolder;
class RoseBuildImpl;

bool dedupeLeftfixes(RoseBuildImpl &tbi);
void mergeLeftfixesVariableLag(RoseBuildImpl &tbi);
void dedupeLeftfixesVariableLag(RoseBuildImpl &tbi);
void dedupeSuffixes(RoseBuildImpl &tbi);

void mergeAcyclicSuffixes(RoseBuildImpl &tbi);
void mergeSmallSuffixes(RoseBuildImpl &tbi);
void mergeSmallLeftfixes(RoseBuildImpl &tbi);
void mergeCastleLeftfixes(RoseBuildImpl &tbi);
void mergeOutfixes(RoseBuildImpl &tbi);
void mergePuffixes(RoseBuildImpl &tbi);
void mergeCastleSuffixes(RoseBuildImpl &tbi);

bool mergeableRoseVertices(const RoseBuildImpl &tbi, RoseVertex u,
                           RoseVertex v);
bool mergeableRoseVertices(const RoseBuildImpl &tbi,
                           const std::set<RoseVertex> &v1,
                           const std::set<RoseVertex> &v2);
bool setDistinctRoseTops(RoseGraph &g, NGHolder &h1, const NGHolder &h2,
                         const std::deque<RoseVertex> &verts1);

} // namespace ue2

#endif // ROSE_BUILD_MERGE_H
