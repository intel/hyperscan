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

#include "config.h"

#include "fdr_compile.h"
#include "fdr_compile_internal.h"
#include "fdr_dump.h"
#include "fdr_engine_description.h"
#include "fdr_internal.h"
#include "teddy_engine_description.h"
#include "ue2common.h"

#include <cstdio>
#include <memory>

#ifndef DUMP_SUPPORT
#error No dump support!
#endif

using std::unique_ptr;

namespace ue2 {

static
bool fdrIsTeddy(const FDR *fdr) {
    assert(fdr);
    u32 engine = fdr->engineID;

    /* teddys don't have an fdr engine description (which is why the dump code
     * is so broken). */

    return !getFdrDescription(engine);
}

void fdrPrintStats(const FDR *fdr, FILE *f) {
    const bool isTeddy = fdrIsTeddy(fdr);

    if (isTeddy) {
        fprintf(f, "TEDDY:         %u\n", fdr->engineID);
    } else {
        fprintf(f, "FDR:           %u\n", fdr->engineID);
    }

    if (isTeddy) {
        auto des = getTeddyDescription(fdr->engineID);
        if (des) {
            fprintf(f, "    masks      %u\n", des->numMasks);
            fprintf(f, "    buckets    %u\n", des->getNumBuckets());
            fprintf(f, "    packed     %s\n", des->packed ? "true" : "false");
        } else {
            fprintf(f, "   <unknown engine>\n");
        }
    } else {
        fprintf(f, "    domain     %u\n", fdr->domain);
        fprintf(f, "    stride     %u\n", fdr->stride);
    }

    fprintf(f, "    strings    ???\n");
    fprintf(f, "    size       %zu bytes\n", fdrSize(fdr));
    fprintf(f, "    max length %u\n", fdr->maxStringLen);
    fprintf(f, "    floodoff   %u (%x)\n", fdr->floodOffset, fdr->floodOffset);
}

} // namespace ue2
