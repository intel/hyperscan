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

#ifndef ROSE_BUILD_ANCHORED
#define ROSE_BUILD_ANCHORED

#include "ue2common.h"
#include "rose_build_impl.h"
#include "nfagraph/ng_holder.h"
#include "util/bytecode_ptr.h"

#include <map>
#include <vector>
#include <set>

struct anchored_matcher_info;

namespace ue2 {

class RoseBuildImpl;
struct raw_dfa;
struct LitFragment;

/**
 * \brief Construct a set of anchored DFAs from our anchored literals/engines.
 */
std::vector<raw_dfa> buildAnchoredDfas(RoseBuildImpl &build,
                                    const std::vector<LitFragment> &fragments);

/**
 * \brief Construct an anchored_matcher_info runtime structure from the given
 * set of DFAs.
 *
 * Remap the literal final_ids used for raw_dfa reports to the program offsets
 * given in litPrograms.
 */
bytecode_ptr<anchored_matcher_info>
buildAnchoredMatcher(RoseBuildImpl &build,
                     const std::vector<LitFragment> &fragments,
                     std::vector<raw_dfa> &dfas);

u32 anchoredStateSize(const anchored_matcher_info &atable);

#define ANCHORED_FAIL    0
#define ANCHORED_SUCCESS 1
#define ANCHORED_REMAP   2

int addAnchoredNFA(RoseBuildImpl &build, const NGHolder &wrapper,
                   const std::map<NFAVertex, std::set<u32>> &reportMap);

int addToAnchoredMatcher(RoseBuildImpl &build, const NGHolder &anchored,
                         u32 exit_id, ReportID *remap);

} // namespace ue2

#endif
