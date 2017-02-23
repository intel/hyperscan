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

#include "rose_build_scatter.h"
#include "ue2common.h"
#include "util/container.h"
#include "util/multibit_build.h"

#include <cstring> // memset
#include <set>

using namespace std;

namespace ue2 {

template<typename T>
static
void rebase(vector<T> *p, u32 adj) {
    for (typename vector<T>::iterator it = p->begin(); it != p->end(); ++it) {
        DEBUG_PRINTF("=%u+%u\n", it->offset, adj);
        it->offset += adj;
    }
}

static
void rebase(scatter_plan_raw *raw, u32 adj) {
    rebase(&raw->p_u64a, adj);
    rebase(&raw->p_u32,  adj);
    rebase(&raw->p_u16,  adj);
    rebase(&raw->p_u8,   adj);
}

static
void merge_in(scatter_plan_raw *out, const scatter_plan_raw &in) {
    insert(&out->p_u64a, out->p_u64a.end(), in.p_u64a);
    insert(&out->p_u32,  out->p_u32.end(),  in.p_u32);
    insert(&out->p_u16,  out->p_u16.end(),  in.p_u16);
    insert(&out->p_u8,   out->p_u8.end(),   in.p_u8);
}

scatter_plan_raw buildStateScatterPlan(u32 role_state_offset,
            u32 role_state_count, u32 left_array_count, u32 left_prefix_count,
            const RoseStateOffsets &stateOffsets, bool streaming,
            u32 leaf_array_count, u32 outfix_begin, u32 outfix_end) {
    scatter_plan_raw out;

    /* init role array */
    scatter_plan_raw spr_role;
    mmbBuildClearPlan(role_state_count, &spr_role);
    rebase(&spr_role, role_state_offset);
    merge_in(&out, spr_role);

    /* init rose array: turn on prefixes */
    u32 rose_array_offset = stateOffsets.activeLeftArray;
    scatter_plan_raw spr_rose;
    mmbBuildInitRangePlan(left_array_count, 0, left_prefix_count, &spr_rose);
    rebase(&spr_rose, rose_array_offset);
    merge_in(&out, spr_rose);

    /* suffix/outfix array */
    scatter_plan_raw spr_leaf;
    if (streaming) {
        mmbBuildInitRangePlan(leaf_array_count, outfix_begin, outfix_end,
                              &spr_leaf);
    } else {
        mmbBuildClearPlan(leaf_array_count, &spr_leaf);
    }
    rebase(&spr_leaf, stateOffsets.activeLeafArray);
    merge_in(&out, spr_leaf);

    return out;
}

u32 aux_size(const scatter_plan_raw &raw) {
    u32 rv = 0;

    rv += byte_length(raw.p_u64a);
    rv += byte_length(raw.p_u32);
    rv += byte_length(raw.p_u16);
    rv += byte_length(raw.p_u8);

    return rv;
}

void write_out(scatter_full_plan *plan_out, void *aux_out,
               const scatter_plan_raw &raw, u32 aux_base_offset) {
    memset(plan_out, 0, sizeof(*plan_out));

#define DO_CASE(t)                                                      \
    if (!raw.p_##t.empty()) {                                           \
        plan_out->s_##t##_offset = aux_base_offset;                     \
        plan_out->s_##t##_count = raw.p_##t.size();                     \
        assert(ISALIGNED_N((char *)aux_out + aux_base_offset,           \
                           alignof(scatter_unit_##t)));                 \
        memcpy((char *)aux_out + aux_base_offset, raw.p_##t.data(),     \
               byte_length(raw.p_##t));                                 \
        aux_base_offset += byte_length(raw.p_##t);                      \
    }

    DO_CASE(u64a);
    DO_CASE(u32);
    DO_CASE(u16);
    DO_CASE(u8);
}

} // namespace ue2
