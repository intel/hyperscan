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

#ifndef ENGINE_DESCRIPTION_H
#define ENGINE_DESCRIPTION_H

#include "ue2common.h"
#include "util/target_info.h"

namespace ue2 {

class EngineDescription {
    u32 id;
    target_t code_target; // the target that we built this code for
    u32 numBuckets;

public:
    EngineDescription(u32 id_in, const target_t &code_target_in,
                      u32 numBuckets_in)
        : id(id_in), code_target(code_target_in), numBuckets(numBuckets_in) {}

    virtual ~EngineDescription();

    u32 getID() const { return id; }
    u32 getNumBuckets() const { return numBuckets; }

    bool isValidOnTarget(const target_t &target_in) const;
    virtual u32 getDefaultFloodSuffixLength() const = 0;
};

/** Returns a target given a CPU feature set value. */
target_t targetByArchFeatures(u64a cpu_features);

} // namespace ue2

#endif
