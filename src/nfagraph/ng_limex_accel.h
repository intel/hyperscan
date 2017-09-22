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
 * \brief NFA acceleration analysis code.
 */

#ifndef NG_LIMEX_ACCEL_H
#define NG_LIMEX_ACCEL_H

#include "ng_holder.h"
#include "ng_misc_opt.h"
#include "ue2common.h"
#include "nfa/accelcompile.h"
#include "util/accel_scheme.h"
#include "util/charreach.h"
#include "util/flat_containers.h"
#include "util/order_check.h"

#include <map>
#include <vector>

namespace ue2 {

/* compile time accel defs */
#define MAX_MERGED_ACCEL_STOPS 200
#define ACCEL_MAX_STOP_CHAR 24
#define ACCEL_MAX_FLOATING_STOP_CHAR 192 /* accelerating sds is important */

// forward-declaration of CompileContext
struct CompileContext;

void findAccelFriends(const NGHolder &g, NFAVertex v,
                  const std::map<NFAVertex, BoundedRepeatSummary> &br_cyclic,
                  u32 offset, flat_set<NFAVertex> *friends);

#define DOUBLE_SHUFTI_LIMIT 20

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


} // namespace ue2

#endif
