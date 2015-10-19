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

#include "sidecar_dump.h"
#include "sidecar_internal.h"
#include "ue2common.h"

#include <cstdio>

#ifndef DUMP_SUPPORT
#error No dump support!
#endif

namespace ue2 {

static
void dumpSideShuf(const sidecar_S *s, FILE *f) {
    fprintf(f, "lo:");
    for (u32 i = 0; i < 16; i++) {
        fprintf(f, " %02hhx", ((const u8 *)&s->lo)[i]);
    }
    fprintf(f, "\n");

    fprintf(f, "hi:");
    for (u32 i = 0; i < 16; i++) {
        fprintf(f, " %02hhx", ((const u8 *)&s->hi)[i]);
    }
    fprintf(f, "\n");

    const u8 *enables = (const u8 *)sidecar_ids_to_mask_const(s);
    fprintf(f, "shufti masks per id\n");
    for (u32 i = 0; i < s->header.id_count; i++) {
        fprintf(f, "%u: %02hhx\n", i, enables[i]);
    }
}

void sidecarDump(const sidecar *s, FILE *f) {
    const char *type = "?";
    switch(s->type) {
    case SIDECAR_8:
        type = "8";
        break;
    case SIDECAR_32:
        type = "32";
        break;
    case SIDECAR_64:
        type = "64";
        break;
    case SIDECAR_128:
        type = "128";
        break;
    case SIDECAR_256:
        type = "256";
        break;
    case SIDECAR_N:
        type = "N";
        break;
    case SIDECAR_S:
        type = "S";
        break;
    default:
        assert(0);
    }

    fprintf(f, "Sidecar:           %s\n", type);
    fprintf(f, "    size:          %u\n", s->size);
    fprintf(f, "    used bits:     %u\n", s->mask_bit_count);
    fprintf(f, "    ids:           %u\n", s->id_count);
    if (s->type == SIDECAR_S) {
        dumpSideShuf((const sidecar_S *)s, f);
    }
}

} // namespace ue2
