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
 * \brief Hamster Wheel Literal Matcher: runtime API.
 */

#ifndef HWLM_H
#define HWLM_H

#include "ue2common.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** \brief Error return type for exec functions. */
typedef int hwlm_error_t;

/** \brief Type representing a set of groups as a bitmap. */
typedef u64a hwlm_group_t;

/** \brief HWLM callback return type. */
typedef hwlm_group_t hwlmcb_rv_t;

/** \brief Value representing all possible literal groups. */
#define HWLM_ALL_GROUPS         ((hwlm_group_t)~0ULL)

/** \brief Callback return value indicating that we should continue matching. */
#define HWLM_CONTINUE_MATCHING  HWLM_ALL_GROUPS

/** \brief Callback return value indicating that we should halt matching. */
#define HWLM_TERMINATE_MATCHING 0

/** \brief Matching finished without being terminated by the user. */
#define HWLM_SUCCESS       0

/** \brief The user terminated matching by returning HWLM_TERMINATE_MATCHING
 * from the match callback. */
#define HWLM_TERMINATED    1

/** \brief An error occurred during matching.
 *
 * This should only be used if an unsupported engine was called (like one
 * designed for a different architecture). */
#define HWLM_ERROR_UNKNOWN 2

/** \brief Max length of the literal passed to HWLM. */
#define HWLM_LITERAL_MAX_LEN 8

struct hs_scratch;
struct HWLM;

/** \brief The type for an HWLM callback.
 *
 * This callback receives an end-of-match offset, the ID of the match and
 * the context pointer that was passed into \ref hwlmExec or
 * \ref hwlmExecStreaming.
 *
 * A callback return of \ref HWLM_TERMINATE_MATCHING will stop matching.
 *
 * A callback return of \ref HWLM_CONTINUE_MATCHING continues matching.
 *
 * An arbitrary group mask may be given as the return value. This will be taken
 * as a hint by the underlying engine that only literals with groups
 * overlapping the provided mask need to be reported.
 *
 * The underlying engine may choose not to report a match if there is no group
 * belonging to the literal which was active at the when the end match location
 * was first reached.
 */
typedef hwlmcb_rv_t (*HWLMCallback)(size_t end, u32 id,
                     struct hs_scratch *scratch);

/** \brief Match strings in table.
 *
 * If a match occurs, the callback function given will be called with the index
 * of the last character in the string and the \p context (passed through
 * without interpretation).
 *
 * Returns \ref HWLM_TERMINATED if scanning is cancelled due to the callback
 * returning \ref HWLM_TERMINATE_MATCHING.
 *
 * \p start is the first offset at which a match may start. Note: match
 * starts may include masks overhanging the main literal.
 *
 * The underlying engine may choose not to report any match which starts before
 * the first possible match of a literal which is in the initial group mask.
 */
hwlm_error_t hwlmExec(const struct HWLM *tab, const u8 *buf, size_t len,
                      size_t start, HWLMCallback callback,
                      struct hs_scratch *scratch, hwlm_group_t groups);

/** \brief As for \ref hwlmExec, but a streaming case across two buffers.
 *
 * \p len is the length of the main buffer to be scanned.
 *
 * \p start is an advisory hint representing the first offset at which a match
 * may start. Some underlying literal matches may not respect it. Note: match
 * starts may include masks overhanging the main literal.
 *
 * \p scratch is used to access the history buffer, history length and
 * the main buffer.
 *
 * Two buffers/lengths are provided. Matches that occur entirely within
 * the history buffer will not be reported by this function. The offsets
 * reported for the main buffer are relative to the start of that buffer (a
 * match at byte 10 of the main buffer is reported as 10). Matches that start
 * in the history buffer will have starts reported with 'negative' values.
 */
hwlm_error_t hwlmExecStreaming(const struct HWLM *tab, size_t len, size_t start,
                               HWLMCallback callback,
                               struct hs_scratch *scratch, hwlm_group_t groups);

#ifdef __cplusplus
}       /* extern "C" */
#endif

#endif
