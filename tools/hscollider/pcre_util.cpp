/*
 * Copyright (c) 2015-2018, Intel Corporation
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

#include "pcre_util.h"

#include "hs.h"

#include <assert.h>
#include <pcre.h> /* for pcre flags */

bool getPcreFlags(unsigned int hs_flags, unsigned int *flags,
                  bool *highlander, bool *prefilter, bool *som,
                  bool *combination, bool *quiet) {
    assert(flags);
    assert(highlander);
    assert(prefilter);
    assert(som);
    *flags = 0;
    *highlander = false;
    *prefilter = false;
    *som = false;

    if (hs_flags & HS_FLAG_CASELESS) {
        *flags |= PCRE_CASELESS;
        hs_flags &= ~HS_FLAG_CASELESS;
    }
    if (hs_flags & HS_FLAG_DOTALL) {
        *flags |= PCRE_DOTALL;
        hs_flags &= ~HS_FLAG_DOTALL;
    }
    if (hs_flags & HS_FLAG_MULTILINE) {
        *flags |= PCRE_MULTILINE;
        hs_flags &= ~HS_FLAG_MULTILINE;
    }
    if (hs_flags & HS_FLAG_UCP) {
        *flags |= PCRE_UCP;
        hs_flags &= ~HS_FLAG_UCP;
    }
    if (hs_flags & HS_FLAG_UTF8) {
        *flags |= PCRE_UTF8;
        hs_flags &= ~HS_FLAG_UTF8;
    }
    if (hs_flags & HS_FLAG_SINGLEMATCH) {
        *highlander = true;
        hs_flags &= ~HS_FLAG_SINGLEMATCH;
    }
    if (hs_flags & HS_FLAG_PREFILTER) {
        *prefilter = true;
        hs_flags &= ~HS_FLAG_PREFILTER;
    }
    if (hs_flags & HS_FLAG_SOM_LEFTMOST) {
        *som = true;
        hs_flags &= ~HS_FLAG_SOM_LEFTMOST;
    }
    if (hs_flags & HS_FLAG_COMBINATION) {
        *combination = true;
        hs_flags &= ~HS_FLAG_COMBINATION;
    }
    if (hs_flags & HS_FLAG_QUIET) {
        *quiet = true;
        hs_flags &= ~HS_FLAG_QUIET;
    }

    // Flags that are irrelevant to PCRE.
    hs_flags &= ~HS_FLAG_ALLOWEMPTY;

    if (hs_flags) {
        // You've added new flags, haven't you?
        assert(0);
        return false;
    }

    return true;
}
