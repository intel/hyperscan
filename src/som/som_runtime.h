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

/**
 * \file
 * \brief SOM runtime code.
 *
 * Runtime code for SOM handling called by the Rose callback adaptors.
 */

#ifndef SOM_RUNTIME_H
#define SOM_RUNTIME_H

#include "scratch.h"
#include "ue2common.h"

struct som_operation;

void handleSomInternal(struct hs_scratch *scratch,
                       const struct som_operation *ri, const u64a to_offset);

// Returns the from_offset.
u64a handleSomExternal(struct hs_scratch *scratch,
                       const struct som_operation *ri, const u64a to_offset);

void setSomFromSomAware(struct hs_scratch *scratch,
                        const struct som_operation *ri, u64a from_offset,
                        u64a to_offset);

int flushStoredSomMatches_i(struct hs_scratch *scratch, u64a offset);

static really_inline
int flushStoredSomMatches(struct hs_scratch *scratch, u64a offset) {
    if (scratch->deduper.som_log_dirty) {
        return flushStoredSomMatches_i(scratch, offset);
    } else {
        return 0;
    }
}

#endif // SOM_RUNTIME_H

