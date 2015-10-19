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

#ifndef SIDECAR_H
#define SIDECAR_H

#include "ue2common.h"

#ifdef __cplusplus
extern "C" {
#endif

struct sidecar;
struct sidecar_enabled;
struct sidecar_scratch;

/*
 * Sidecar is guaranteed to return the first match of a given id. However, in
 * various cases later matches may also be returned, as may matches for disabled
 * ids
 */
typedef void (*SidecarCallback)(u64a offset, u32 id, void *context);

void sidecarExec(const struct sidecar *n, const u8 *buffer, size_t len,
                 struct sidecar_enabled *enabled,
                 struct sidecar_scratch *sidecar_scratch,
                 u64a base_offset, SidecarCallback cb, void *context);

u32 sidecarScratchSize(const struct sidecar *n);

void sidecarEnabledInit(const struct sidecar *n,
                        struct sidecar_enabled *enabled);

/* Note: sidecar literals need to be reenabled after they match.
 * This is purely because this behaviour is handy for rose.
 * In rose, they always set their roles when fired (never have to postpone due
 * to history) and if cleared their preds are also cleared so a pred would also
 * have to match again before we need to care about them again
 */
void sidecarEnabledUnion(const struct sidecar *n, struct sidecar_enabled *dest,
                         const struct sidecar_enabled *src);

#define ID_TERMINATOR (~0U)

#ifdef __cplusplus
}
#endif

#endif
