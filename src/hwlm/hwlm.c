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
 * \brief Hamster Wheel Literal Matcher: runtime.
 */
#include "hwlm.h"
#include "hwlm_internal.h"
#include "noodle_engine.h"
#include "scratch.h"
#include "ue2common.h"
#include "fdr/fdr.h"
#include "nfa/accel.h"
#include "nfa/shufti.h"
#include "nfa/truffle.h"
#include "nfa/vermicelli.h"
#include <string.h>

#define MIN_ACCEL_LEN_BLOCK  16
#define MIN_ACCEL_LEN_STREAM 16

static really_inline
const u8 *run_hwlm_accel(const union AccelAux *aux, const u8 *ptr,
                         const u8 *end) {
    switch (aux->accel_type) {
    case ACCEL_VERM:
        DEBUG_PRINTF("single vermicelli for 0x%02hhx\n", aux->verm.c);
        return vermicelliExec(aux->verm.c, 0, ptr, end);
    case ACCEL_VERM_NOCASE:
        DEBUG_PRINTF("single vermicelli-nocase for 0x%02hhx\n", aux->verm.c);
        return vermicelliExec(aux->verm.c, 1, ptr, end);
    case ACCEL_DVERM:
        DEBUG_PRINTF("double vermicelli for 0x%02hhx%02hhx\n", aux->dverm.c1,
                     aux->dverm.c2);
        return vermicelliDoubleExec(aux->dverm.c1, aux->dverm.c2, 0, ptr, end);
    case ACCEL_DVERM_NOCASE:
        DEBUG_PRINTF("double vermicelli-nocase for 0x%02hhx%02hhx\n",
                     aux->dverm.c1, aux->dverm.c2);
        return vermicelliDoubleExec(aux->dverm.c1, aux->dverm.c2, 1, ptr, end);
    case ACCEL_SHUFTI:
        DEBUG_PRINTF("single shufti\n");
        return shuftiExec(aux->shufti.lo, aux->shufti.hi, ptr, end);
    case ACCEL_TRUFFLE:
        DEBUG_PRINTF("truffle\n");
        return truffleExec(aux->truffle.mask1, aux->truffle.mask2, ptr, end);
    default:
        /* no acceleration, fall through and return current ptr */
        DEBUG_PRINTF("no accel; %u\n", (int)aux->accel_type);
        assert(aux->accel_type == ACCEL_NONE);
        return ptr;
    }
}

static really_inline
void do_accel_block(const union AccelAux *aux, const u8 *buf, size_t len,
                    size_t *start) {
    if (len - *start < MIN_ACCEL_LEN_BLOCK) {
        return;
    }

    const u8 *ptr = buf + *start;
    const u8 *end = buf + len;
    const u8 offset = aux->generic.offset;
    ptr = run_hwlm_accel(aux, ptr, end);

    if (offset) {
        ptr -= offset;
        if (ptr < buf) {
            ptr = buf;
        }
    }
    assert(ptr >= buf);
    *start = ptr - buf;
}

static really_inline
int inaccurate_accel(u8 type) {
    /* accels which don't always catch up to the boundary
     * DSHUFTI is also inaccurate but it is not used by the hamsters */
    return type == ACCEL_DVERM_NOCASE || type == ACCEL_DVERM;
}

static never_inline
void do_accel_streaming(const union AccelAux *aux, const u8 *hbuf, size_t hlen,
                        const u8 *buf, size_t len, size_t *start) {
    if (aux->accel_type == ACCEL_NONE || len - *start < MIN_ACCEL_LEN_STREAM) {
        return;
    }

    const u8 offset = aux->generic.offset;

    DEBUG_PRINTF("using accel %hhu offset %hhu\n", aux->accel_type, offset);

    // Scan history buffer, but only if the start offset (which always refers to
    // buf) is zero.

    if (!*start && hlen) {
        const u8 *ptr1 = hbuf;
        const u8 *end1 = hbuf + hlen;
        if (hlen >= 16) {
            ptr1 = run_hwlm_accel(aux, ptr1, end1);
        }

        if ((hlen <= 16 || inaccurate_accel(aux->accel_type))
            && end1 != ptr1 && end1 - ptr1 <= 16) {
            DEBUG_PRINTF("already scanned %zu/%zu\n", ptr1 - hbuf, hlen);
            /* see if we can finish off the history buffer completely */
            u8 ALIGN_DIRECTIVE temp[17];
            ptrdiff_t tlen = end1 - ptr1;
            memcpy(temp, ptr1, tlen);
            memset(temp + tlen, 0, 17 - tlen);
            if (len) { /* for dverm */
                temp[end1 - ptr1] = *buf;
            }

            const u8 *tempp = run_hwlm_accel(aux, temp, temp + 17);

            if (tempp - temp >= tlen) {
                ptr1 = end1;
            }
            DEBUG_PRINTF("got %zu\n", tempp - temp);
        }

        if (ptr1 != end1) {
            DEBUG_PRINTF("bailing in history\n");
            return;
        }
    }

    DEBUG_PRINTF("scanning main buffer, start=%zu, len=%zu\n", *start, len);

    const u8 *ptr2 = buf + *start;
    const u8 *end2 = buf + len;

    const u8 *found = run_hwlm_accel(aux, ptr2, end2);

    if (found >= ptr2 + offset) {
        size_t delta = found - offset - ptr2;
        DEBUG_PRINTF("got %zu/%zu in 2nd buffer\n", delta, len);
        *start += delta;
    } else if (hlen) {
        UNUSED size_t remaining = offset + ptr2 - found;
        DEBUG_PRINTF("got %zu/%zu remaining in 1st buffer\n", remaining, hlen);
    }
}

hwlm_error_t hwlmExec(const struct HWLM *t, const u8 *buf, size_t len,
                      size_t start, HWLMCallback cb, struct hs_scratch *scratch,
                      hwlm_group_t groups) {
    assert(t);

    DEBUG_PRINTF("buf len=%zu, start=%zu, groups=%llx\n", len, start, groups);
    if (!groups) {
        DEBUG_PRINTF("groups all off\n");
        return HWLM_SUCCESS;
    }

    assert(start < len);

    if (t->type == HWLM_ENGINE_NOOD) {
        DEBUG_PRINTF("calling noodExec\n");
        return noodExec(HWLM_C_DATA(t), buf, len, start, cb, scratch);
    }

    assert(t->type == HWLM_ENGINE_FDR);
    const union AccelAux *aa = &t->accel0;
    if ((groups & ~t->accel1_groups) == 0) {
        DEBUG_PRINTF("using hq accel %hhu\n", t->accel1.accel_type);
        aa = &t->accel1;
    }
    do_accel_block(aa, buf, len, &start);
    DEBUG_PRINTF("calling frankie (groups=%08llx, start=%zu)\n", groups, start);
    return fdrExec(HWLM_C_DATA(t), buf, len, start, cb, scratch, groups);
}

hwlm_error_t hwlmExecStreaming(const struct HWLM *t, size_t len, size_t start,
                               HWLMCallback cb, struct hs_scratch *scratch,
                               hwlm_group_t groups) {
    assert(t);
    assert(scratch);

    const u8 *hbuf = scratch->core_info.hbuf;
    const size_t hlen = scratch->core_info.hlen;
    const u8 *buf = scratch->core_info.buf;

    DEBUG_PRINTF("hbuf len=%zu, buf len=%zu, start=%zu, groups=%llx\n", hlen,
                 len, start, groups);

    if (!groups) {
        return HWLM_SUCCESS;
    }

    assert(start < len);

    if (t->type == HWLM_ENGINE_NOOD) {
        DEBUG_PRINTF("calling noodExec\n");
        // If we've been handed a start offset, we can use a block mode scan at
        // that offset.
        if (start) {
            return noodExec(HWLM_C_DATA(t), buf, len, start, cb, scratch);
        } else {
            return noodExecStreaming(HWLM_C_DATA(t), hbuf, hlen, buf, len, cb,
                                     scratch);
        }
    }

    assert(t->type == HWLM_ENGINE_FDR);
    const union AccelAux *aa = &t->accel0;
    if ((groups & ~t->accel1_groups) == 0) {
        DEBUG_PRINTF("using hq accel %hhu\n", t->accel1.accel_type);
        aa = &t->accel1;
    }
    do_accel_streaming(aa, hbuf, hlen, buf, len, &start);
    DEBUG_PRINTF("calling frankie (groups=%08llx, start=%zu)\n", groups, start);
    return fdrExecStreaming(HWLM_C_DATA(t), hbuf, hlen, buf, len, start, cb,
                            scratch, groups);
}
