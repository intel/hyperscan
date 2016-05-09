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

#include "Utf8ComponentClass.h"

#include <algorithm>

using namespace std;

namespace ue2 {

#define UCP_FN(cat)                                                     \
CodePointSet getUcp##cat(void) {                                        \
    CodePointSet rv;                                                    \
    for (u32 i = 0; i < ARRAY_LENGTH(ucp_##cat##_def); i += 2) {        \
        rv.setRange(ucp_##cat##_def[i], ucp_##cat##_def[i + 1]);        \
    }                                                                   \
    return rv;                                                          \
}

struct unicase {
    unichar base;
    unichar caseless;
};

} // namespace ue2

#define UCP_TABLE_DEFINE_FN
#include "ucp_table.h"

namespace ue2 {

static
bool operator<(const unicase &a, const unicase &b) {
    if (a.base < b.base) {
        return true;
    }

    if (a.base > b.base) {
        return false;
    }

    return a.caseless < b.caseless;
}

void make_caseless(CodePointSet *cps) {
    assert(cps);
    DEBUG_PRINTF("hello\n");
    // Cheap optimisation: if we are empty or a dot, we're already caseless.
    if (cps->begin() == cps->end()) {
        DEBUG_PRINTF("empty\n");
        return;
    }
    if (lower(*cps->begin()) == 0 && upper(*cps->begin()) == MAX_UNICODE) {
        DEBUG_PRINTF("dot\n");
        return;
    }

    CodePointSet base = *cps;

    auto uc_begin = begin(ucp_caseless_def);
    auto uc_end = end(ucp_caseless_def);
    DEBUG_PRINTF("uc len %zd\n", distance(uc_begin, uc_end));

    for (const auto &elem : base) {
        unichar b = lower(elem);
        unichar e = upper(elem) + 1;

        for (; b < e; b++) {
            DEBUG_PRINTF("decasing %x\n", b);
            unicase test = {b, 0}; /* NUL is not a caseless version of anything,
                                    * so we are ok */
            uc_begin = lower_bound(uc_begin, uc_end, test);
            if (uc_begin == uc_end) {
                DEBUG_PRINTF("EOL\n");
                return;
            }
            while (uc_begin != uc_end && uc_begin->base == b) {
                DEBUG_PRINTF("at {%x,%x}\n", uc_begin->base, uc_begin->caseless);
                cps->set(uc_begin->caseless);
                ++uc_begin;
            }
        }
    }
}

/** \brief Flip the case of the codepoint in c, if possible.
 *
 * Note that this assumes a one-to-one case mapping, which (though not
 * realistic) is what PCRE does. */
bool flip_case(unichar *c) {
    assert(c);

    const unicase test = { *c, 0 };

    const auto uc_begin = begin(ucp_caseless_def);
    const auto uc_end = end(ucp_caseless_def);
    const auto f = lower_bound(uc_begin, uc_end, test);
    if (f != uc_end && f->base == *c) {
        DEBUG_PRINTF("flipped c=%x to %x\n", *c, f->caseless);
        *c = f->caseless;
        return true;
    }
    return false;
}

} // namespace ue2
