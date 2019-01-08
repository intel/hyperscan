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

#ifndef FDR_CONFIRM_RUNTIME_H
#define FDR_CONFIRM_RUNTIME_H

#include "scratch.h"
#include "fdr_internal.h"
#include "fdr_loadval.h"
#include "hwlm/hwlm.h"
#include "ue2common.h"
#include "util/bitutils.h"
#include "util/compare.h"

// this is ordinary confirmation function which runs through
// the whole confirmation procedure
static really_inline
void confWithBit(const struct FDRConfirm *fdrc, const struct FDR_Runtime_Args *a,
                 size_t i, hwlmcb_rv_t *control, u32 *last_match,
                 u64a conf_key, u64a *conf, u8 bit) {
    assert(i < a->len);
    assert(i >= a->start_offset);
    assert(ISALIGNED(fdrc));

    const u8 * buf = a->buf;
    u32 c = CONF_HASH_CALL(conf_key, fdrc->andmsk, fdrc->mult,
                           fdrc->nBits);
    u32 start = getConfirmLitIndex(fdrc)[c];
    if (likely(!start)) {
        return;
    }

    const struct LitInfo *li
        = (const struct LitInfo *)((const u8 *)fdrc + start);

    struct hs_scratch *scratch = a->scratch;
    assert(!scratch->fdr_conf);
    scratch->fdr_conf = conf;
    scratch->fdr_conf_offset = bit;
    u8 oldNext; // initialized in loop
    do {
        assert(ISALIGNED(li));
        scratch->pure = li->pure;

        if (unlikely((conf_key & li->msk) != li->v)) {
            goto out;
        }

        if ((*last_match == li->id) && (li->flags & FDR_LIT_FLAG_NOREPEAT)) {
            goto out;
        }

        const u8 *loc = buf + i - li->size + 1;

        if (loc < buf) {
            u32 full_overhang = buf - loc;
            size_t len_history = a->len_history;

            // can't do a vectored confirm either if we don't have
            // the bytes
            if (full_overhang > len_history) {
                goto out;
            }
        }
        assert(li->size <= sizeof(CONF_TYPE));

        if (unlikely(!(li->groups & *control))) {
            goto out;
        }

        *last_match = li->id;
        *control = a->cb(i, li->id, scratch);
    out:
        oldNext = li->next; // oldNext is either 0 or an 'adjust' value
        li++;
    } while (oldNext);
    scratch->fdr_conf = NULL;
    scratch->pure = 0;
}

#endif
