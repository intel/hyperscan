/*
 * Copyright (c) 2015-2023, Intel Corporation
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
 * \brief Stream state data structures.
 */

#ifndef STATE_H
#define STATE_H

#include "hs_runtime.h" /* match_event_handler */
#include "ue2common.h"

#ifdef __cplusplus
extern "C"
{
#endif

struct RoseEngine;

/** \brief Stream context: allocated for each stream.
 *
 * struct hs_stream is followed in memory by the main Rose state: history,
 * exhaustion, individual states, etc. The RoseEngine has the offsets required
 * to correctly index into the main state structure. The offsets used by the
 * RoseEngine are based on the end of the hs_stream struct as its size may
 * vary from platform to platform.
 */
struct hs_stream {
    /** \brief The RoseEngine that this stream is matching against. */
    const struct RoseEngine *rose;

    /** \brief The current stream offset. */
    u64a offset;
};

#define getMultiState(hs_s)      ((char *)(hs_s) + sizeof(*(hs_s)))
#define getMultiStateConst(hs_s) ((const char *)(hs_s) + sizeof(*(hs_s)))

#ifdef __cplusplus
}
#endif

#endif
