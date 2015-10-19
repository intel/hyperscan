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

/** \file
 * \brief Hamster Wheel Literal Matcher: build API.
 */

#ifndef HWLM_BUILD_H
#define HWLM_BUILD_H

#include "hwlm.h"
#include "hwlm_literal.h"
#include "ue2common.h"
#include "util/alloc.h"

#include <memory>
#include <vector>

struct HWLM;

namespace ue2 {

struct CompileContext;
struct Grey;
struct target_t;

/** \brief Structure gathering together the input/output parameters related to
 * streaming mode operation. */
struct hwlmStreamingControl {
    /** \brief IN parameter: Upper limit on the amount of history that can be
     * requested. */
    size_t history_max;

    /** \brief IN parameter: History already known to be used before literal
     * analysis. */
    size_t history_min;

    /** \brief OUT parameter: History required by the literal matcher to
     * correctly match all literals. */
    size_t literal_history_required;

    /** OUT parameter: Stream state required by literal matcher in bytes. Can
     * be zero, and generally will be small (0-8 bytes). */
    size_t literal_stream_state_required;
};

/** \brief Build an \ref HWLM literal matcher runtime structure for a group of
 * literals.
 *
 * \param lits The group of literals.
 * \param stream_control Streaming control parameters. If the matcher will
 *        operate in non-streaming (block) mode, this pointer should be NULL.
 * \param make_small Optimise matcher for small size.
 * \param cc Compile context.
 * \param expected_groups FIXME: document me!
 *
 * Build failures are generally a result of memory allocation failure. These
 * may result in a nullptr return value, or a std::bad_alloc exception being
 * thrown.
 */
aligned_unique_ptr<HWLM>
hwlmBuild(const std::vector<hwlmLiteral> &lits,
          hwlmStreamingControl *stream_control, bool make_small,
          const CompileContext &cc,
          hwlm_group_t expected_groups = HWLM_ALL_GROUPS);

/**
 * Returns an estimate of the number of repeated characters on the end of a
 * literal that will make a literal set of size \a numLiterals suffer
 * performance degradation.
 */
size_t hwlmFloodProneSuffixLen(size_t numLiterals, const CompileContext &cc);

/** \brief Return the size in bytes of an HWLM structure. */
size_t hwlmSize(const HWLM *h);

} // namespace

#endif // HWLM_BUILD_H
