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

#ifndef FDR_COMPILE_INTERNAL_H
#define FDR_COMPILE_INTERNAL_H

#include "ue2common.h"
#include "hwlm/hwlm_literal.h"
#include "util/bytecode_ptr.h"

#include <map>
#include <utility>
#include <vector>

struct FDRConfirm;
struct LitInfo;

namespace ue2 {

// a pile of decorative typedefs
// good for documentation purposes more than anything else
typedef u32 LiteralIndex;
typedef u32 SuffixPositionInString; // zero is last byte, counting back
                                    // into the string
typedef u32 BucketIndex;
typedef u32 SchemeBitIndex;
typedef u32 PositionInBucket;  // zero is 'we are matching right now!",
                               // counting towards future matches

class EngineDescription;
class FDREngineDescription;
struct hwlmStreamingControl;
struct Grey;

bytecode_ptr<u8> setupFullConfs(
      const std::vector<hwlmLiteral> &lits,
      const EngineDescription &eng,
      const std::map<BucketIndex, std::vector<LiteralIndex>> &bucketToLits,
      bool make_small);

// all suffixes include an implicit max_bucket_width suffix to ensure that
// we always read a full-scale flood "behind" us in terms of what's in our
// state; if we don't have a flood that's long enough we won't be in the
// right state yet to allow blindly advancing
bytecode_ptr<u8> setupFDRFloodControl(const std::vector<hwlmLiteral> &lits,
                                      const EngineDescription &eng,
                                      const Grey &grey);

bytecode_ptr<u8>
fdrBuildTableStreaming(const std::vector<hwlmLiteral> &lits,
                       hwlmStreamingControl &stream_control);

static constexpr u32 HINT_INVALID = 0xffffffff;

// fdr_compile_util.cpp utilities
size_t maxLen(const std::vector<hwlmLiteral> &lits);
size_t minLenCount(const std::vector<hwlmLiteral> &lits, size_t *count);
u32 absdiff(u32 i, u32 j);

} // namespace ue2

#endif
