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

#include "config.h"

#include "smallwrite_dump.h"
#include "smallwrite_internal.h"
#include "ue2common.h"
#include "nfa/nfa_build_util.h"
#include "nfa/nfa_dump_api.h"
#include "nfa/nfa_internal.h"
#include "util/dump_util.h"

#include <cstdio>
#include <string>

#ifndef DUMP_SUPPORT
#error No dump support!
#endif

using namespace std;

namespace ue2 {

void smwrDumpText(const SmallWriteEngine *smwr, FILE *f) {
    if (!smwr) {
        fprintf(f, "<< no smallwrite >>\n");
        return;
    }

    const struct NFA *n = getSmwrNfa(smwr);

    fprintf(f, "SmallWrite:\n\n");
    fprintf(f, "%s\n", describe(*n).c_str());
    fprintf(f, "States: %u\n", n->nPositions);
    fprintf(f, "Length: %u\n", n->length);
    fprintf(f, "Largest Short Buffer: %u\n", smwr->largestBuffer);
    fprintf(f, "Start Offset: %u\n", smwr->start_offset);
}

void smwrDumpNFA(const SmallWriteEngine *smwr, bool dump_raw,
                 const string &base) {
    if (!smwr) {
        DEBUG_PRINTF("no smallwrite\n");
        return;
    }

    const struct NFA *n = getSmwrNfa(smwr);

    nfaGenerateDumpFiles(n, base + "smallwrite_nfa");

    if (dump_raw) {
        StdioFile f(base + "smallwrite_nfa.raw", "w");
        fwrite(n, 1, n->length, f);
    }
}

} // namespace ue2
