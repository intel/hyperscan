/*
 * Copyright (c) 2015-2016, Intel Corporation
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
 * \brief Rose runtime types (callbacks, etc).
 */

#ifndef ROSE_TYPES_H
#define ROSE_TYPES_H

#include "ue2common.h"

struct hs_scratch;

/**
 * \brief Continue without checking for exhaustion.
 *
 * \ref RoseCallback return value indicating that execution should continue and
 * that it is not necessary to check if all reports have been exhausted.
 */
#define ROSE_CONTINUE_MATCHING_NO_EXHAUST 2

/**
 * \brief The type for a Rose callback.
 *
 * \return
 *  - \ref MO_HALT_MATCHING if matching should terminate;
 *  - \ref MO_CONTINUE_MATCHING if matching should continue;
 *  - \ref ROSE_CONTINUE_MATCHING_NO_EXHAUST if matching should continue and no
 *    exhaustion is possible.
 */
typedef int (*RoseCallback)(u64a offset, ReportID id,
                            struct hs_scratch *scratch);

/**
 * \brief The type for a Rose callback which also tracks start of match.
 *
 * Behaves just like \ref RoseCallback except that it is provided with both a
 * start and an end offset.
 *
 * \see RoseCallback
 */
typedef int (*RoseCallbackSom)(u64a from_offset, u64a to_offset, ReportID id,
                               struct hs_scratch *scratch);

#endif
