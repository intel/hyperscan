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
#include "hwlm_literal.h"
#include "util/bitutils.h" // for CASE_BIT
#include "util/compare.h" // for ourisalpha
#include "util/ue2string.h" // for escapeString

#include <algorithm>
#include <iomanip>
#include <sstream>

using namespace std;

namespace ue2 {

#ifdef DEBUG
static UNUSED
std::string dumpMask(const vector<u8> &v) {
    ostringstream oss;
    vector<u8>::const_iterator it, ite;
    for (it = v.begin(), ite = v.end(); it != ite; ++it) {
        oss << setfill('0') << setw(2) << hex << (unsigned int)*it;
    }
    return oss.str();
}
#endif

bool maskIsConsistent(const std::string &s, bool nocase, const vector<u8> &msk,
                      const vector<u8> &cmp) {
    string::const_reverse_iterator si = s.rbegin();
    vector<u8>::const_reverse_iterator mi = msk.rbegin(), ci = cmp.rbegin();

    for (; si != s.rend() && mi != msk.rend(); ++si, ++mi, ++ci) {
        u8 c = *si, m = *mi, v = *ci;
        if (nocase && ourisalpha(c)) {
            m &= ~CASE_BIT;
            v &= ~CASE_BIT;
        }

        assert(ci != cmp.rend());
        if ((c & m) != v) {
            DEBUG_PRINTF("c = %02hhx; *ci = %02hhx m =%02hhx\n", c, *ci, m);
            DEBUG_PRINTF("s = %s; dist = %zd\n", s.c_str(), si - s.rbegin());
            return false;
        }
    }

    return true;
}

/** \brief Complete constructor, takes group information and msk/cmp.
 *
 * This constructor takes a msk/cmp pair. Both must be vectors of length <=
 * \ref HWLM_MASKLEN. */
hwlmLiteral::hwlmLiteral(const std::string &s_in, bool nocase_in,
                         bool noruns_in, u32 id_in, hwlm_group_t groups_in,
                         const vector<u8> &msk_in, const vector<u8> &cmp_in,
                         bool pure_in)
    : s(s_in), id(id_in), nocase(nocase_in), noruns(noruns_in),
      groups(groups_in), msk(msk_in), cmp(cmp_in), pure(pure_in) {
    assert(s.size() <= HWLM_LITERAL_MAX_LEN);
    assert(msk.size() <= HWLM_MASKLEN);
    assert(msk.size() == cmp.size());

    // If we've been handled a nocase literal, all letter characters must be
    // upper-case.
    if (nocase) {
        upperString(s);
    }

    DEBUG_PRINTF("literal '%s'%s, msk=%s, cmp=%s\n", escapeString(s).c_str(),
                 nocase ? " (nocase)" : "", dumpMask(msk).c_str(),
                 dumpMask(cmp).c_str());


    // Mask and compare vectors MUST be the same size.
    assert(msk.size() == cmp.size());

    // We must have been passed a msk/cmp that can be applied to s.
    assert(maskIsConsistent(s, nocase, msk, cmp));

    // In the name of good hygiene, zap msk/cmp if msk is all zeroes.
    if (all_of(begin(msk), end(msk), [](u8 val) { return val == 0; })) {
        msk.clear();
        cmp.clear();
    }
}

} // namespace ue2
