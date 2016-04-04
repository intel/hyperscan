/*
 * Copyright (c) 2015-2016, Intel Corporation
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
 * \brief NFA acceleration analysis code.
 */

#ifndef NG_LIMEX_ACCEL_H
#define NG_LIMEX_ACCEL_H

#include "ng_holder.h"
#include "ng_misc_opt.h"
#include "ue2common.h"
#include "nfa/accelcompile.h"
#include "util/charreach.h"
#include "util/order_check.h"
#include "util/ue2_containers.h"

#include <map>
#include <vector>

namespace ue2 {

/* compile time accel defs */
#define MAX_ACCEL_DEPTH 4
#define MAX_MERGED_ACCEL_STOPS 200
#define ACCEL_MAX_STOP_CHAR 24
#define ACCEL_MAX_FLOATING_STOP_CHAR 192 /* accelerating sds is important */
#define MULTIACCEL_MIN_LEN 3
#define MULTIACCEL_MAX_LEN_SSE 15
#define MULTIACCEL_MAX_LEN_AVX2 31

// forward-declaration of CompileContext
struct CompileContext;

void findAccelFriends(const NGHolder &g, NFAVertex v,
                  const std::map<NFAVertex, BoundedRepeatSummary> &br_cyclic,
                      u32 offset,
                      ue2::flat_set<NFAVertex> *friends);

#define DOUBLE_SHUFTI_LIMIT 20

struct AccelScheme {
    AccelScheme(const CharReach &cr_in, u32 offset_in)
        : cr(cr_in), offset(offset_in) {
        assert(offset <= MAX_ACCEL_DEPTH);
    }
    AccelScheme() : cr(CharReach::dot()), offset(MAX_ACCEL_DEPTH + 1) {}

    bool operator<(const AccelScheme &b) const {
        const AccelScheme &a = *this;

        // Don't use ORDER_CHECK as it will (stupidly) eval count() too many
        // times.
        size_t a_dcount = double_cr.count();
        size_t b_dcount = b.double_cr.count();

        bool feasible_double_a = !a.double_byte.empty()
            && a.double_byte.size() <= DOUBLE_SHUFTI_LIMIT;
        bool feasible_double_b = !b.double_byte.empty()
            && b.double_byte.size() <= DOUBLE_SHUFTI_LIMIT;

        if (feasible_double_a != feasible_double_b) {
            return feasible_double_a > feasible_double_b;
        }

        if (feasible_double_a) {
            if (a_dcount != b_dcount) {
                return a_dcount < b_dcount;
            }

            if ((a.double_byte.size() == 1) != (b.double_byte.size() == 1)) {
                return a.double_byte.size() < b.double_byte.size();
            }

            if (!a_dcount) {
                bool cd_a = buildDvermMask(a.double_byte);
                bool cd_b = buildDvermMask(b.double_byte);
                if (cd_a != cd_b) {
                    return cd_a > cd_b;
                }
            }
            ORDER_CHECK(double_byte.size());
            ORDER_CHECK(double_offset);
        }

        const size_t a_count = cr.count(), b_count = b.cr.count();
        if (a_count != b_count) {
            return a_count < b_count;
        }

        /* TODO: give bonus if one is a 'caseless' character */
        ORDER_CHECK(offset);
        ORDER_CHECK(cr);
        ORDER_CHECK(double_byte);
        ORDER_CHECK(double_cr);
        ORDER_CHECK(double_offset);
        return false;
    }

    bool operator>(const AccelScheme &b) const {
        return b < *this;
    }

    ue2::flat_set<std::pair<u8, u8> > double_byte;
    CharReach cr;
    CharReach double_cr;
    u32 offset;
    u32 double_offset = 0;
};

NFAVertex get_sds_or_proxy(const NGHolder &g);

AccelScheme nfaFindAccel(const NGHolder &g, const std::vector<NFAVertex> &verts,
                    const std::vector<CharReach> &refined_cr,
                    const std::map<NFAVertex, BoundedRepeatSummary> &br_cyclic,
                    bool allow_wide, bool look_for_double_byte = false);

AccelScheme findBestAccelScheme(std::vector<std::vector<CharReach> > paths,
                                const CharReach &terminating,
                                bool look_for_double_byte = false);

/** \brief Check if vertex \a v is an accelerable state (for a limex NFA). If a
 *  single byte accel scheme is found it is placed into *as
 */
bool nfaCheckAccel(const NGHolder &g, NFAVertex v,
                   const std::vector<CharReach> &refined_cr,
                   const std::map<NFAVertex, BoundedRepeatSummary> &br_cyclic,
                   AccelScheme *as, bool allow_wide);

/** \brief Check if vertex \a v is a multi accelerable state (for a limex NFA). */
MultibyteAccelInfo nfaCheckMultiAccel(const NGHolder &g,
                                      const std::vector<NFAVertex> &verts,
                                      const CompileContext &cc);

} // namespace ue2

#endif
