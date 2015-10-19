/*
 * Copyright (c) 2015, Intel Corporation
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

#include "nfa_dump_internal.h"

#include "accel.h"
#include "nfa_internal.h"
#include "ue2common.h"

#include <cctype> // for isprint
#include <sstream>

#ifndef DUMP_SUPPORT
#error No dump support!
#endif

namespace ue2 {

void dumpDotPreamble(FILE *dotFile) {
    fprintf(dotFile, "digraph NFA {\n");
    fprintf(dotFile, "rankdir=LR;\n");
    fprintf(dotFile, "size=\"11.5,8\"\n");
    fprintf(dotFile, "node [ shape = circle ];\n");
    fprintf(dotFile, "START [style=invis];\n");
}

void dumpDotTrailer(FILE *dotFile) {
    fprintf(dotFile, "}\n");
}

static
void dumpFlags(const struct NFA *nfa, FILE *f) {
    fprintf(f, "Flags                : ");
    if (nfa->flags & NFA_ACCEPTS_EOD) {
        fprintf(f, "ACCEPTS_EOD ");
    }
    if (nfa->flags & NFA_ZOMBIE) {
        fprintf(f, "ZOMBIE ");
    }
    fprintf(f, "\n");
}

// Render an unsigned integer as a string, returning "inf/unknown" if it's
// zero.
static
std::string value_or_inf(const u64a v) {
    std::ostringstream oss;
    if (v) {
        oss << v;
    } else {
        oss << "inf/unknown";
    }
    return oss.str();
}

void dumpTextReverse(const struct NFA *nfa, FILE *f) {
    bool twofer = false;

    fprintf(f, "Queue                : %u\n", nfa->queueIndex);
    fprintf(f, "Length               : %u bytes\n", nfa->length);
    fprintf(f, "Num Positions        : %u\n", nfa->nPositions);
    fprintf(f, "Scratch State        : %u bytes\n", nfa->scratchStateSize);
    fprintf(f, "Stream State         : %u bytes\n", nfa->streamStateSize);
    dumpFlags(nfa, f);
    fprintf(f, "Max Width            : %s\n", value_or_inf(nfa->maxWidth).c_str());
    fprintf(f, "Min Width            : %u\n", nfa->minWidth);
    fprintf(f, "BiAnchored Width     : %s\n",
            value_or_inf(nfa->maxBiAnchoredWidth).c_str());
    fprintf(f, "Max Offset           : %s\n", value_or_inf(nfa->maxOffset).c_str());
    fprintf(f, "Reverse Acceleration : ");

    switch (nfa->rAccelType) {
    case ACCEL_NONE:
        fprintf(f, "NONE\n");
        return;
    case ACCEL_RVERM:
        fprintf(f, "R VERM");
        break;
    case ACCEL_RVERM_NOCASE:
        fprintf(f, "R VERM NOCASE");
        break;
    case ACCEL_RDVERM:
        fprintf(f, "R VERM x2");
        twofer = true;
        break;
    case ACCEL_RDVERM_NOCASE:
        fprintf(f, "R VERM NOCASE x2");
        twofer = true;
        break;
    case ACCEL_REOD:
        fprintf(f, "R EOD");
        break;
    case ACCEL_REOD_NOCASE:
        fprintf(f, "R EOD NOCASE");
        break;
    case ACCEL_RDEOD:
        fprintf(f, "R EOD x2");
        twofer = true;
        break;
    case ACCEL_RDEOD_NOCASE:
        fprintf(f, "R EOD x2 NOCASE");
        twofer = true;
        break;
    default:
        fprintf(f, "UNKNOWN\n");
        return;
    }

    char c1 = nfa->rAccelData.array[0];
    char c2 = nfa->rAccelData.array[1];

    if (!twofer) {
        fprintf(f, " \\x%02hhx (%c) ", c1, isprint(c1) ? c1 : '?');
    } else {
        fprintf(f, " \\x%02hhx\\x%02hhx (%c%c) ", c1, c2,
                isprint(c1) ? c1 : '?', isprint(c2) ? c2 : '?');
    }

    fprintf(f, "offset %hhd\n", nfa->rAccelOffset);
}

} // namespace ue2
