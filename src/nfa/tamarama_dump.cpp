/*
 * Copyright (c) 2016, Intel Corporation
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
 * \brief Tamarama: container engine for exclusive engines, dump code.
 */

#include "config.h"

#include "tamarama_dump.h"

#include "tamarama_internal.h"
#include "nfa_dump_api.h"
#include "nfa_dump_internal.h"
#include "nfa_internal.h"
#include "util/dump_util.h"

#include <string>
#include <sstream>

#ifndef DUMP_SUPPORT
#error No dump support!
#endif

using namespace std;

namespace ue2 {

void nfaExecTamarama_dump(const struct NFA *nfa, const string &base) {
    const Tamarama *t = (const Tamarama *)getImplNfa(nfa);

    StdioFile f(base + ".txt", "w");

    fprintf(f, "Tamarama container engine\n");
    fprintf(f, "\n");
    fprintf(f, "Number of subengine tenants:  %u\n", t->numSubEngines);

    fprintf(f, "\n");
    dumpTextReverse(nfa, f);
    fprintf(f, "\n");

    const u32 *subOffset =
        (const u32 *)((const char *)t + sizeof(struct Tamarama) +
                      t->numSubEngines * sizeof(u32));
    for (u32 i = 0; i < t->numSubEngines; i++) {
        const NFA *sub = (const struct NFA *)((const char *)t + subOffset[i]);

        stringstream sssub;
        sssub << base << "_sub_" << i;
        nfaGenerateDumpFiles(sub, sssub.str());
    }
}

} // namespace ue2
