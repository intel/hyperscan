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
 * \brief Rose compile-time analysis for lookaround masks.
 */
#ifndef ROSE_ROSE_BUILD_LOOKAROUND_H
#define ROSE_ROSE_BUILD_LOOKAROUND_H

#include "rose_graph.h"
#include "util/hash.h"

#include <vector>

/** \brief Max path number for multi-path lookaround. */
#define MAX_LOOKAROUND_PATHS 8

namespace ue2 {

class CharReach;
class RoseBuildImpl;

/** \brief Lookaround entry prototype, describing the reachability at a given
 * distance from the end of a role match. */
struct LookEntry {
    LookEntry() : offset(0) {}
    LookEntry(s8 offset_in, const CharReach &reach_in)
        : offset(offset_in), reach(reach_in) {}
    s8 offset; //!< offset from role match location.
    CharReach reach; //!< reachability at given offset.

    bool operator==(const LookEntry &other) const {
        return offset == other.offset && reach == other.reach;
    }
};

void findLookaroundMasks(const RoseBuildImpl &tbi, const RoseVertex v,
                         std::vector<LookEntry> &look_more);

/**
 * \brief If possible, render the prefix of the given vertex as a lookaround.
 *
 * Given a prefix, returns true (and fills the lookaround vector) if
 * it can be satisfied with a lookaround alone.
 */
bool makeLeftfixLookaround(const RoseBuildImpl &build, const RoseVertex v,
                           std::vector<std::vector<LookEntry>> &lookaround);

void mergeLookaround(std::vector<LookEntry> &lookaround,
                     const std::vector<LookEntry> &more_lookaround);

} // namespace ue2

namespace std {

template<>
struct hash<ue2::LookEntry> {
    size_t operator()(const ue2::LookEntry &l) const {
        return ue2::hash_all(l.offset, l.reach);
    }
};

} // namespace std

#endif // ROSE_ROSE_BUILD_LOOKAROUND_H
