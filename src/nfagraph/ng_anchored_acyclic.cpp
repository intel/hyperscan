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

/** \file
 * \brief Anchored acyclic graph -> DFA analysis.
 */
#include "ng_anchored_acyclic.h"

#include "ng_holder.h"
#include "ng_reports.h"
#include "ng_util.h"
#include "ue2common.h"
#include "rose/rose_build.h"
#include "util/compile_context.h"

namespace ue2 {

bool splitOffAnchoredAcyclic(RoseBuild &rose, const NGHolder &h,
                             const CompileContext &cc) {
    if (!cc.grey.allowAnchoredAcyclic) {
        return false;
    }

    if (!isAnchored(h)) {
        DEBUG_PRINTF("fail, not anchored\n");
        return false;
    }

    if (!isAcyclic(h)) {
        DEBUG_PRINTF("fail, not acyclic\n");
        return false;
    }

    if (rose.addAnchoredAcyclic(h)) {
        return true;
    } else {
        DEBUG_PRINTF("failed to add anchored nfa\n");
        return false;
    }
}

} // namespace ue2
