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
 * \brief Hamster Wheel Literal Matcher: dump code.
 */

#include "config.h"

#include "hwlm_dump.h"
#include "hwlm_internal.h"
#include "noodle_build.h"
#include "ue2common.h"
#include "fdr/fdr_dump.h"
#include "nfa/accel_dump.h"
#include "util/dump_util.h"

#ifndef DUMP_SUPPORT
#error No dump support!
#endif

using namespace std;

namespace ue2 {

void hwlmGenerateDumpFiles(const HWLM *h, const string &base) {
    StdioFile f(base + ".txt", "w");

    switch (h->type) {
    case HWLM_ENGINE_NOOD:
        noodPrintStats((const noodTable *)HWLM_C_DATA(h), f);
        break;
    case HWLM_ENGINE_FDR:
        fdrPrintStats((const FDR *)HWLM_C_DATA(h), f);
        break;
    default:
        fprintf(f, "<unknown hwlm subengine>\n");
    }

    fprintf(f, "accel1_groups: %016llx\n", h->accel1_groups);

    fprintf(f, "accel1:");
    dumpAccelInfo(f, h->accel1);
    fprintf(f, "accel0:");
    dumpAccelInfo(f, h->accel0);
}

} // namespace ue2
