/*
 * Copyright (c) 2016-2017, Intel Corporation
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
 * \brief Rose build: code for constructing literal tables.
 */

#ifndef ROSE_BUILD_MATCHERS_H
#define ROSE_BUILD_MATCHERS_H

#include "rose_build_impl.h"
#include "util/bytecode_ptr.h"

#include <vector>

struct Grey;
struct HWLM;

namespace ue2 {

struct LitFragment {
    LitFragment(u32 fragment_id_in, rose_group groups_in, u32 lit_id)
    : fragment_id(fragment_id_in), groups(groups_in), lit_ids({lit_id}) {}
    LitFragment(u32 fragment_id_in, rose_group groups_in,
                std::vector<u32> lit_ids_in)
    : fragment_id(fragment_id_in), groups(groups_in),
        lit_ids(std::move(lit_ids_in)) {}
    u32 fragment_id;
    rose_group groups;
    std::vector<u32> lit_ids;
    u32 lit_program_offset = ROSE_INVALID_PROG_OFFSET;
    u32 delay_program_offset = ROSE_INVALID_PROG_OFFSET;
};

bytecode_ptr<HWLM>
buildFloatingMatcher(const RoseBuildImpl &build,
                     const std::vector<LitFragment> &fragments,
                     size_t longLitLengthThreshold, rose_group *fgroups,
                     size_t *historyRequired);

bytecode_ptr<HWLM>
buildDelayRebuildMatcher(const RoseBuildImpl &build,
                         const std::vector<LitFragment> &fragments,
                         size_t longLitLengthThreshold);

bytecode_ptr<HWLM>
buildSmallBlockMatcher(const RoseBuildImpl &build,
                       const std::vector<LitFragment> &fragments);

bytecode_ptr<HWLM>
buildEodAnchoredMatcher(const RoseBuildImpl &build,
                        const std::vector<LitFragment> &fragments);

void findMoreLiteralMasks(RoseBuildImpl &build);

} // namespace ue2

#endif // ROSE_BUILD_MATCHERS_H
