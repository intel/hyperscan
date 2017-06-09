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

#ifndef FDR_CONFIRM_RUNTIME_H
#define FDR_CONFIRM_RUNTIME_H

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
                 u64a conf_key) {
    assert(i < a->len);
    assert(ISALIGNED(fdrc));

    const u8 * buf = a->buf;
    u32 c = CONF_HASH_CALL(conf_key, fdrc->andmsk, fdrc->mult,
                           fdrc->nBitsOrSoleID);
    u32 start = getConfirmLitIndex(fdrc)[c];
    if (likely(!start)) {
        return;
    }

    const struct LitInfo *li
        = (const struct LitInfo *)((const u8 *)fdrc + start);

    u8 oldNext; // initialized in loop
    do {
        assert(ISALIGNED(li));

        if (unlikely((conf_key & li->msk) != li->v)) {
            goto out;
        }

        if ((*last_match == li->id) && (li->flags & NoRepeat)) {
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

        if (unlikely(li->flags & ComplexConfirm)) {
            const u8 *loc2 = buf + i - li->extended_size + 1;
            if (loc2 < buf) {
                u32 full_overhang = buf - loc2;
                size_t len_history = a->len_history;
                if (full_overhang > len_history) {
                    goto out;
                }
            }
        }

        *last_match = li->id;
        *control = a->cb(loc - buf, i, li->id, a->ctxt);
    out:
        oldNext = li->next; // oldNext is either 0 or an 'adjust' value
        li++;
    } while (oldNext);
}

// 'light-weight' confirmation function which is used by 1-mask Teddy;
// in the 'confirmless' case it simply calls callback function,
// otherwise it calls 'confWithBit' function for the full confirmation procedure
static really_inline
void confWithBit1(const struct FDRConfirm *fdrc,
                  const struct FDR_Runtime_Args *a, size_t i,
                  hwlmcb_rv_t *control, u32 *last_match, u64a conf_key) {
    assert(i < a->len);
    assert(ISALIGNED(fdrc));

    if (unlikely(fdrc->mult)) {
        confWithBit(fdrc, a, i, control, last_match, conf_key);
        return;
    } else {
        u32 id = fdrc->nBitsOrSoleID;

        if ((*last_match == id) && (fdrc->flags & NoRepeat)) {
            return;
        }
        *last_match = id;
        *control = a->cb(i, i, id, a->ctxt);
    }
}

// This is 'light-weight' confirmation function which is used by 2-3-4-mask Teddy
// In the 'confirmless' case it makes fast 32-bit comparison,
// otherwise it calls 'confWithBit' function for the full confirmation procedure
static really_inline
void confWithBitMany(const struct FDRConfirm *fdrc,
                     const struct FDR_Runtime_Args *a, size_t i, CautionReason r,
                     hwlmcb_rv_t *control, u32 *last_match, u64a conf_key) {
    assert(i < a->len);
    assert(ISALIGNED(fdrc));

    if (i < a->start_offset) {
        return;
    }

    if (unlikely(fdrc->mult)) {
        confWithBit(fdrc, a, i, control, last_match, conf_key);
        return;
    } else {
        const u32 id = fdrc->nBitsOrSoleID;
        const u32 len = fdrc->soleLitSize;

        if ((*last_match == id) && (fdrc->flags & NoRepeat)) {
            return;
        }

        if (r == VECTORING && len > i - a->start_offset) {
            if (len > i + a->len_history) {
                return;
            }

            u32 cmp = (u32)a->buf[i] << 24;

            if (len <= i) {
                for (u32 j = 1; j <= len; j++) {
                    cmp |= (u32)a->buf[i - j] << (24 - (j * 8));
                }
            } else {
                for (u32 j = 1; j <= i; j++) {
                    cmp |= (u32)a->buf[i - j] << (24 - (j * 8));
                }
                cmp |= (u32)(a->histBytes >> (40 + i * 8));
            }

            if ((fdrc->soleLitMsk & cmp) != fdrc->soleLitCmp) {
               return;
            }
        }
        *last_match = id;
        *control = a->cb(i - len, i, id, a->ctxt);
    }
}

#endif
