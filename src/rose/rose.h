/*
 * Copyright (c) 2015-2018, Intel Corporation
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

#ifndef ROSE_H
#define ROSE_H

#include "ue2common.h"

struct RoseEngine;
struct hs_scratch;

// Initialise state space for engine use.
void roseInitState(const struct RoseEngine *t, char *state);

/* assumes core_info in scratch has been init to point to data */
void roseBlockExec(const struct RoseEngine *t, struct hs_scratch *scratch);

/* assumes core_info in scratch has been init to point to data */
void roseStreamExec(const struct RoseEngine *t, struct hs_scratch *scratch);

void roseStreamEodExec(const struct RoseEngine *t, u64a offset,
                       struct hs_scratch *scratch);

hwlmcb_rv_t roseCallback(size_t end, u32 id, struct hs_scratch *scratch);

int roseReportAdaptor(u64a start, u64a end, ReportID id, void *context);

int roseRunBoundaryProgram(const struct RoseEngine *rose, u32 program,
                           u64a stream_offset, struct hs_scratch *scratch);

int roseRunFlushCombProgram(const struct RoseEngine *rose,
                            struct hs_scratch *scratch, u64a end);

#endif // ROSE_H
