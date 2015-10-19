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

#include "util/simd_utils.h"

#define P0(cnd) unlikely(cnd)

#include "fdr.h"
#include "fdr_internal.h"
#include "teddy_internal.h"

#include "flood_runtime.h"

#include "fdr_confirm.h"
#include "fdr_confirm_runtime.h"
#include "fdr_streaming_runtime.h"
#include "fdr_loadval.h"

static really_inline UNUSED
u32 getPreStartVal(const struct FDR_Runtime_Args *a, u32 numBits) {
    u32 r = 0;
    if (a->start_offset == 0) {
        if (numBits <= 8) {
            r = a->buf_history[a->len_history - 1];
        } else {
            r = a->buf_history[a->len_history - 1];
            r |= (a->buf[0] << 8);
        }
    } else {
        if (numBits <= 8) {
            r = a->buf[a->start_offset - 1];
        } else {
            r = lv_u16(a->buf + a->start_offset - 1, a->buf, a->buf + a->len);
        }
    }
    return r & ((1 << numBits) - 1);
}

#include "fdr_autogen.c"

#define FAKE_HISTORY_SIZE 16
static const u8 fake_history[FAKE_HISTORY_SIZE];

hwlm_error_t fdrExec(const struct FDR *fdr, const u8 *buf, size_t len, size_t start,
                     HWLMCallback cb, void *ctxt, hwlm_group_t groups) {

    const struct FDR_Runtime_Args a = {
        buf,
        len,
        fake_history,
        0,
        fake_history, // nocase
        0,
        start,
        cb,
        ctxt,
        &groups,
        nextFloodDetect(buf, len, FLOOD_BACKOFF_START),
        0
    };
    if (unlikely(a.start_offset >= a.len)) {
        return HWLM_SUCCESS;
    } else {
        assert(funcs[fdr->engineID]);
        return funcs[fdr->engineID](fdr, &a);
    }
}

hwlm_error_t fdrExecStreaming(const struct FDR *fdr, const u8 *hbuf,
                              size_t hlen, const u8 *buf, size_t len,
                              size_t start, HWLMCallback cb, void *ctxt,
                              hwlm_group_t groups, u8 * stream_state) {
    struct FDR_Runtime_Args a = {
        buf,
        len,
        hbuf,
        hlen,
        hbuf, // nocase - start same as caseful, override later if needed
        hlen, // nocase
        start,
        cb,
        ctxt,
        &groups,
        nextFloodDetect(buf, len, FLOOD_BACKOFF_START),
        hbuf ? CONF_LOADVAL_CALL_CAUTIOUS(hbuf + hlen - 8, hbuf, hbuf + hlen)
             : (u64a)0

    };
    fdrUnpackState(fdr, &a, stream_state);

    hwlm_error_t ret;
    if (unlikely(a.start_offset >= a.len)) {
        ret = HWLM_SUCCESS;
    } else {
        assert(funcs[fdr->engineID]);
        ret = funcs[fdr->engineID](fdr, &a);
    }

    fdrPackState(fdr, &a, stream_state);
    return ret;
}
