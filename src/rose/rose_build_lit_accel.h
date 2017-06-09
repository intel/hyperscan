/*
 * Copyright (c) 2017, Intel Corporation
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

#ifndef ROSE_BUILD_LIT_ACCEL_H
#define ROSE_BUILD_LIT_ACCEL_H

#include "hwlm/hwlm.h"

#include <string>
#include <tuple>
#include <utility>
#include <vector>

struct HWLM;

namespace ue2 {

struct AccelString {
    AccelString(std::string s_in, bool nocase_in, std::vector<u8> msk_in,
                std::vector<u8> cmp_in, hwlm_group_t groups_in)
        : s(std::move(s_in)), nocase(nocase_in), msk(std::move(msk_in)),
          cmp(std::move(cmp_in)), groups(groups_in) {}

    std::string s;
    bool nocase;
    std::vector<u8> msk;
    std::vector<u8> cmp;
    hwlm_group_t groups;

    bool operator==(const AccelString &a) const {
        return s == a.s && nocase == a.nocase && msk == a.msk && cmp == a.cmp &&
               groups == a.groups;
    }

    bool operator<(const AccelString &a) const {
        return std::tie(s, nocase, msk, cmp, groups) <
               std::tie(a.s, a.nocase, a.msk, a.cmp, a.groups);
    }
};

void buildForwardAccel(HWLM *h, const std::vector<AccelString> &lits,
                       hwlm_group_t expected_groups);

} // namespace ue2

#endif // ROSE_BUILD_LIT_ACCEL_H
