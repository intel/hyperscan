/*
 * Copyright (c) 2018, Intel Corporation
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
 * \brief Parse and build ParsedLogical::logicalTree and combInfoMap.
 */

#ifndef LOGICAL_COMBINATION_H
#define LOGICAL_COMBINATION_H

#include "util/logical.h"

#include <map>
#include <set>
#include <vector>

namespace ue2 {

class ParsedLogical {
    friend class ReportManager;
public:
    /** \brief Parse 1 logical expression \a logical, assign temporary ckey. */
    void parseLogicalCombination(unsigned id, const char *logical, u32 ekey,
                                 u64a min_offset, u64a max_offset);

    /** \brief Check if all sub-expression id in combinations are valid. */
    void validateSubIDs(const unsigned *ids, const char *const *expressions,
                        const unsigned *flags, unsigned elements);

    /** \brief Renumber and assign final lkey for each logical operation
     * after parsed all logical expressions. */
    void logicalKeyRenumber();

    /** \brief Fetch the lkey associated with the given expression id,
     * assigning one if necessary. */
    u32 getLogicalKey(u32 expressionId);

    /** \brief Fetch the ckey associated with the given expression id,
     * assigning one if necessary. */
    u32 getCombKey(u32 expressionId);

    /** \brief Add lkey's corresponding combination id. */
    void addRelateCKey(u32 lkey, u32 ckey);

    /** \brief Add one Logical Operation. */
    u32 logicalTreeAdd(u32 op, u32 left, u32 right);

    /** \brief Assign the combination info associated with the given ckey. */
    void combinationInfoAdd(u32 ckey, u32 id, u32 ekey, u32 lkey_start,
                            u32 lkey_result, u64a min_offset, u64a max_offset);

    const std::map<u32, u32> &getLkeyMap() const {
        return toLogicalKeyMap;
    }

    const std::vector<LogicalOp> &getLogicalTree() const {
        return logicalTree;
    }

    CombInfo getCombInfoById(u32 id) const {
        u32 ckey = toCombKeyMap.at(id);
        assert(ckey < combInfoMap.size());
        return combInfoMap.at(ckey);
    }

private:
    /** \brief Mapping from ckey to combination info. */
    std::vector<CombInfo> combInfoMap;

    /** \brief Mapping from combination expression id to combination key,
     * combination key is used in combination bit-vector cache. */
    std::map<u32, u32> toCombKeyMap;

    /** \brief Mapping from expression id to logical key, logical key is used
     * as index in LogicalOp array. */
    std::map<u32, u32> toLogicalKeyMap;

    /** \brief Mapping from logical key to related combination keys. */
    std::map<u32, std::set<u32>> lkey2ckeys;

    /** \brief Logical constraints, each operation from postfix notation. */
    std::vector<LogicalOp> logicalTree;
};

} // namespace ue2

#endif
