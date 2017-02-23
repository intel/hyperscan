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

#ifndef ROSE_BUILD_SCATTER_H
#define ROSE_BUILD_SCATTER_H

#include "rose_internal.h"
#include "util/scatter.h"

#include <vector>

namespace ue2 {

class RoseBuildImpl;

struct scatter_plan_raw {
    std::vector<scatter_unit_u64a> p_u64a;
    std::vector<scatter_unit_u32>  p_u32;
    std::vector<scatter_unit_u16>  p_u16;
    std::vector<scatter_unit_u8>   p_u8;
};

scatter_plan_raw buildStateScatterPlan(u32 role_state_offset,
            u32 role_state_count, u32 left_array_count, u32 left_prefix_count,
            const RoseStateOffsets &stateOffsets, bool streaming,
            u32 leaf_array_count, u32 outfix_begin, u32 outfix_end);

u32 aux_size(const scatter_plan_raw &raw);

void write_out(scatter_full_plan *plan_out, void *aux_out,
               const scatter_plan_raw &raw, u32 aux_base_offset);

} // namespace ue2

#endif
