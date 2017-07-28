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
 * \brief FDR literal matcher: runtime API.
 */

#ifndef FDR_H
#define FDR_H

#include "ue2common.h"
#include "hwlm/hwlm.h"

// C linkage in the API
#ifdef __cplusplus
extern "C" {
#endif

struct FDR;
struct hs_scratch;

/**
 * \brief Block-mode scan.
 *
 * \param fdr FDR matcher engine.
 * \param buf Buffer to scan.
 * \param len Length of buffer to scan.
 * \param start First offset in buf at which a match may start.
 * \param cb Callback to call when a match is found.
 * \param scratch Scratch supplied to callback on match.
 * \param groups Initial groups mask.
 */
hwlm_error_t fdrExec(const struct FDR *fdr, const u8 *buf, size_t len,
                     size_t start, HWLMCallback cb, struct hs_scratch *scratch,
                     hwlm_group_t groups);

/**
 * \brief Streaming-mode scan.
 *
 * \param fdr FDR matcher engine.
 * \param hbuf History buffer.
 * \param hlen Length of history buffer (hbuf).
 * \param buf Buffer to scan.
 * \param len Length of buffer to scan (buf).
 * \param start First offset in buf at which a match may start.
 * \param cb Callback to call when a match is found.
 * \param scratch Scratch supplied to callback on match.
 * \param groups Initial groups mask.
 */
hwlm_error_t fdrExecStreaming(const struct FDR *fdr, const u8 *hbuf,
                              size_t hlen, const u8 *buf, size_t len,
                              size_t start, HWLMCallback cb,
                              struct hs_scratch *scratch,
                              hwlm_group_t groups);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // FDR_H
