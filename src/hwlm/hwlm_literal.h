/*
 * Copyright (c) 2015-2019, Intel Corporation
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
 * \brief Hamster Wheel Literal Matcher: literal representation at build time.
 */

#ifndef HWLM_LITERAL_H
#define HWLM_LITERAL_H

#include "hwlm.h"
#include "ue2common.h"

#include <string>
#include <tuple>
#include <vector>

namespace ue2 {

/** \brief Max length of the hwlmLiteral::msk and hwlmLiteral::cmp vectors. */
#define HWLM_MASKLEN 8

#define INVALID_LIT_ID ~0U

/** \brief Class representing a literal, fed to \ref hwlmBuild. */
struct hwlmLiteral {
    std::string s; //!< \brief The literal itself.

    /** \brief The ID to pass to the callback if this literal matches.
     *
     * Note that the special value 0xFFFFFFFF is reserved for internal use and
     * should not be used. */
    u32 id;

    bool nocase; //!< \brief True if literal is case-insensitive.

    /** \brief Matches for runs of this literal can be quashed.
     *
     * Advisory flag meaning that there is no value in returning runs of
     * additional matches for a literal after the first one, so such matches
     * can be quashed by the literal matcher. */
    bool noruns;

    /** \brief included literal id. */
    u32 included_id = INVALID_LIT_ID;

    /** \brief Squash mask for FDR's confirm mask for included literals.
     *
     * In FDR confirm, if we have included literal in another bucket,
     * we can use this mask to squash the bit for the bucket in FDR confirm
     * mask and then run programs of included literal directly and avoid
     * confirm work.
     *
     * This value is calculated in FDR compile code once bucket assignment is
     * completed
     */
    u8 squash = 0;

    /** \brief Set of groups that literal belongs to.
     *
     * Use \ref HWLM_ALL_GROUPS for a literal that could match regardless of
     * the groups that are switched on. */
    hwlm_group_t groups;

    /** \brief Supplementary comparison mask.
     *
     * These two values add a supplementary comparison that is done over the
     * final 8 bytes of the string -- if v is those bytes, then the string must
     * match as well as (v & msk) == cmp.
     *
     * An empty msk is the safe way of not adding any comparison to the string
     * unnecessarily filling in msk may turn off optimizations.
     *
     * The msk/cmp mechanism must NOT place a value into the literal that
     * conflicts with the contents of the string, but can be allowed to add
     * additional power within the string -- for example, to allow some case
     * sensitivity within a case-insensitive string.

     * Values are stored in memory order -- i.e. the last byte of the mask
     * corresponds to the last byte of the string. Both vectors must be the
     * same size, and must not exceed \ref HWLM_MASKLEN in length.
     */
    std::vector<u8> msk;

    /** \brief Supplementary comparison value.
     *
     * See documentation for \ref msk.
     */
    std::vector<u8> cmp;

    bool pure; //!< \brief The pass-on of pure flag from LitFragment.

    /** \brief Complete constructor, takes group information and msk/cmp.
     *
     * This constructor takes a msk/cmp pair. Both must be vectors of length <=
     * \ref HWLM_MASKLEN. */
    hwlmLiteral(const std::string &s_in, bool nocase_in, bool noruns_in,
                u32 id_in, hwlm_group_t groups_in,
                const std::vector<u8> &msk_in, const std::vector<u8> &cmp_in,
                bool pure_in = false);

    /** \brief Simple constructor: no group information, no msk/cmp.
     *
     * This constructor is only used in internal unit test. */
    hwlmLiteral(const std::string &s_in, bool nocase_in, u32 id_in)
        : hwlmLiteral(s_in, nocase_in, false, id_in, HWLM_ALL_GROUPS, {}, {}) {}
};

inline
bool operator<(const hwlmLiteral &a, const hwlmLiteral &b) {
    return std::tie(a.id, a.s, a.nocase, a.noruns, a.groups, a.msk, a.cmp) <
           std::tie(b.id, b.s, b.nocase, b.noruns, b.groups, b.msk, b.cmp);
}

inline
bool operator==(const hwlmLiteral &a, const hwlmLiteral &b) {
    return a.id == b.id && a.s == b.s && a.nocase == b.nocase &&
           a.noruns == b.noruns && a.groups == b.groups && a.msk == b.msk &&
           a.cmp == b.cmp;
}

/**
 * Consistency test; returns false if the given msk/cmp test can never match
 * the literal string s.
 */
bool maskIsConsistent(const std::string &s, bool nocase,
                      const std::vector<u8> &msk, const std::vector<u8> &cmp);

} // namespace ue2

#endif // HWLM_LITERAL_H
