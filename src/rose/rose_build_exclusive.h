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

/** \file
 * \brief exclusive analysis for infix and suffix engines.
 * Two engines are considered as exclusive if they can never be alive
 * at the same time. This analysis takes advantage of the property of
 * triggering literal + engine graph. If the triggering literals of
 * two engines can make all the states dead in each other's graph,
 * then they are exclusive.
 */
#ifndef ROSE_BUILD_EXCLUSIVE_H
#define ROSE_BUILD_EXCLUSIVE_H

#include "ue2common.h"

#include "rose_build_impl.h"
#include "util/alloc.h"
#include "util/charreach.h"

#include <map>
#include <set>
#include <vector>

namespace ue2 {

/** \brief role info structure for exclusive analysis */
template<typename role_id>
struct RoleInfo {
    RoleInfo(role_id role_in, u32 id_in) : role(role_in), id(id_in) {}
    bool operator==(const RoleInfo &b) const {
        return id == b.id;
    }
    bool operator!=(const RoleInfo &b) const { return !(*this == b); }
    bool operator<(const RoleInfo &b) const {
        const RoleInfo &a = *this;
        if (a.score != b.score) {
            return a.score > b.score;
        }
        ORDER_CHECK(id);
        return false;
    }

    std::vector<std::vector<CharReach>> literals; // prefix literals
    CharReach prefix_cr; // reach of prefix literals
    CharReach last_cr; // reach of the last character of literals
    CharReach cr; // reach of engine graph
    const role_id role; // infix or suffix info
    const u32 id; // infix or suffix id
    u32 score = ~0U; // score for exclusive analysis
};

/**
 * \brief add triggering literals to infix info.
 */
bool setTriggerLiteralsInfix(RoleInfo<left_id> &roleInfo,
        const std::map<u32, std::vector<std::vector<CharReach>>> &triggers);

/**
 * \brief add triggering literals to suffix info.
 */
bool setTriggerLiteralsSuffix(RoleInfo<suffix_id> &roleInfo,
        const std::map<u32, std::vector<std::vector<CharReach>>> &triggers);

/**
 * Exclusive analysis for infix engines.
 *
 * @param build rose build info mainly used to set exclusive chunk size here
 * @param vertex_map mapping between engine id and rose vertices
 *        related to this engine
 * @param roleInfoSet structure contains role properties including infix info,
 *        triggering literals and literal reachabilities.
 *        Used for exclusive analysis.
 * @param exclusive_roles output mapping between engine id and its exclusive
 *        group id
 */
void exclusiveAnalysisInfix(const RoseBuildImpl &build,
               const std::map<u32, std::vector<RoseVertex>> &vertex_map,
               std::set<RoleInfo<left_id>> &roleInfoSet,
               std::vector<std::vector<u32>> &exclusive_roles);

/**
 * Exclusive analysis for suffix engines.
 *
 * @param build rose build info mainly used to set exclusive chunk size here
 * @param vertex_map mapping between engine id and rose vertices
 *        related to this engine
 * @param roleInfoSet structure contains role properties including suffix info,
 *        triggering literals and literal reachabilities.
 *        Used for exclusive analysis.
 * @param exclusive_roles output mapping between engine id and its exclusive
 *        group id
 */
void exclusiveAnalysisSuffix(const RoseBuildImpl &build,
               const std::map<u32, std::vector<RoseVertex>> &vertex_map,
               std::set<RoleInfo<suffix_id>> &roleInfoSet,
               std::vector<std::vector<u32>> &exclusive_roles);

} // namespace ue2

#endif //ROSE_BUILD_EXCLUSIVE_H

