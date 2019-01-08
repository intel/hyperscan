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

/**
 * \file
 * \brief Rose runtime: program interpreter.
 */

#ifndef PROGRAM_RUNTIME_H
#define PROGRAM_RUNTIME_H

#include "hwlm/hwlm.h" // for hwlmcb_rv_t
#include "rose.h"
#include "scratch.h"
#include "ue2common.h"

/*
 * Program context flags, which control the behaviour of some instructions at
 * based on runtime contexts (whether the program is triggered by the anchored
 * matcher, engine catchup, etc).
 */

#define ROSE_PROG_FLAG_IN_ANCHORED          1
#define ROSE_PROG_FLAG_IN_CATCHUP           2
#define ROSE_PROG_FLAG_FROM_MPV             4
#define ROSE_PROG_FLAG_SKIP_MPV_CATCHUP     8

hwlmcb_rv_t roseRunProgram(const struct RoseEngine *t,
                           struct hs_scratch *scratch, u32 programOffset,
                           u64a som, u64a end, u8 prog_flags);

hwlmcb_rv_t roseRunProgram_l(const struct RoseEngine *t,
                             struct hs_scratch *scratch, u32 programOffset,
                             u64a som, u64a end, u8 prog_flags);

#endif // PROGRAM_RUNTIME_H
