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

#ifndef ROSE_BUILD_CASTLE_H
#define ROSE_BUILD_CASTLE_H

#include "rose_graph.h"

#include <set>

namespace ue2 {

class RoseBuildImpl;
struct left_id;
struct ue2_literal;

/**
 * Runs over all rose infix/suffix engines and converts those that are pure
 * repeats with one report into CastleProto engines.
 */
void makeCastles(RoseBuildImpl &tbi);

/**
 * Identifies all the CastleProto prototypes that are small enough that they
 * would be better implemented as NFAs, and converts them back to NGHolder
 * prototypes.
 *
 * Returns true if any changes were made.
 */
bool unmakeCastles(RoseBuildImpl &tbi);

/**
 * Runs over all the Castle engine prototypes in the graph and ensures that
 * they have tops in a contiguous range, ready for construction.
 */
void remapCastleTops(RoseBuildImpl &tbi);

bool triggerKillsRoseCastle(const RoseBuildImpl &tbi, const left_id &left,
                            const std::set<ue2_literal> &all_lits,
                            const RoseEdge &e);

}

#endif
