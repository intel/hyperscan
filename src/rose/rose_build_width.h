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

#ifndef ROSE_BUILD_WIDTH_H
#define ROSE_BUILD_WIDTH_H

#include "rose_build_impl.h"
#include "ue2common.h"

namespace ue2 {

class RoseBuildImpl;

/* returns a lower bound on the minimum number of bytes required for match to be
 * raised up to the user which requires the given literal table to be used
 *
 * returns ROSE_BOUND_INF if the table can never produce matches */
u32 findMinWidth(const RoseBuildImpl &tbi, enum rose_literal_table table);

/* returns an upper bound on the maximum length of a buffer that can result in
 * matches. If there are any patterns which are not bianchored (start and end
 * anchored), then there is no such limit and ROSE_BOUND_INF is returned.
 */
u32 findMaxBAWidth(const RoseBuildImpl &tbi);

/* returns an upper bound on the maximum length of a buffer that can result in
 * matches and requires that the given table to be used. If there are any
 * patterns which are not bianchored (start and end anchored), then there is no
 * such limit and ROSE_BOUND_INF is returned.
 */
u32 findMaxBAWidth(const RoseBuildImpl &tbi, enum rose_literal_table table);

/**
 * Note: there is no function for determining the min width of the whole rose
 * as this is more easily done by the NG layer which has access to the full
 * nfagraphs before they are chopped into little pieces.
 */

} // namespace ue2

#endif
