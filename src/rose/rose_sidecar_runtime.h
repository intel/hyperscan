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

#ifndef ROSE_SIDECAR_RUNTIME_H_1F746F6F237176
#define ROSE_SIDECAR_RUNTIME_H_1F746F6F237176

#include "hwlm/hwlm.h"
#include "scratch.h"
#include "sidecar/sidecar.h"
#include "rose_common.h"
#include "ue2common.h"

// Callback defined in match.c
void roseSidecarCallback(u64a offset, u32 side_id, void *context);

static really_inline
void catchup_sidecar(struct RoseContext *tctxt, u64a end) {
    DEBUG_PRINTF("catching up the sidecar from %llu to %llu\n",
                 tctxt->side_curr, end);
    const struct sidecar *sidecar = getSLiteralMatcher(tctxt->t);
    struct hs_scratch *scratch = tctxtToScratch(tctxt);
    struct core_info *ci = &scratch->core_info;

    if (!sidecar || tctxt->side_curr == end) {
        return;
    }

    const u8 *start;
    if (tctxt->side_curr >=  ci->buf_offset) {
        start = ci->buf + tctxt->side_curr - ci->buf_offset;
        assert(end <= ci->buf_offset + ci->len);
    } else {
        /* at eod time we are called running over the histroy */
        start = ci->hbuf + tctxt->side_curr - ci->buf_offset + ci->hlen;
        assert(end <= ci->buf_offset);
    }
    size_t len = end - tctxt->side_curr;

    DEBUG_PRINTF("enabled-->%02hhx\n",  *(u8 *)&scratch->side_enabled.arb);
    sidecarExec(sidecar, start, len, &scratch->side_enabled.arb,
                scratch->side_scratch, tctxt->side_curr, roseSidecarCallback,
                tctxt);
    tctxt->side_curr = end;

    DEBUG_PRINTF("finished catching up the sidecar to %llu\n", end);
}

static rose_inline
void enable_sidecar(struct RoseContext *tctxt, const struct RoseRole *tr) {
    assert(tr->sidecarEnableOffset);
    const struct sidecar *sidecar = getSLiteralMatcher(tctxt->t);
    assert(sidecar);
    struct hs_scratch *scratch = tctxtToScratch(tctxt);
    DEBUG_PRINTF("welcome to the sidecar\n");
    sidecarEnabledUnion(sidecar, &scratch->side_enabled.arb,
           (const void *)((const char *)tctxt->t + tr->sidecarEnableOffset));
}

static really_inline
void sidecar_enabled_populate(const struct RoseEngine *t,
                              struct hs_scratch *scratch, const u8 *state) {
    DEBUG_PRINTF("enabled-->%02hhx\n",  *(state + t->stateOffsets.sidecar));
    memcpy(&scratch->side_enabled, state + t->stateOffsets.sidecar,
           t->stateOffsets.sidecar_size);
    DEBUG_PRINTF("enabled-->%02hhx\n",  *(u8 *)&scratch->side_enabled.arb);
}

static really_inline
void sidecar_enabled_preserve(const struct RoseEngine *t,
                              const struct hs_scratch *scratch, u8 *state) {
    memcpy(state + t->stateOffsets.sidecar, &scratch->side_enabled,
           t->stateOffsets.sidecar_size);
}


#endif /* ROSE_SIDECAR_RUNTIME_H_1F746F6F237176 */
