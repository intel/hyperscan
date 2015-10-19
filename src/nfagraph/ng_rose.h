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
 * \brief Rose construction from NGHolder.
 */

#ifndef NG_ROSE_H
#define NG_ROSE_H

#include "ue2common.h"

namespace ue2 {

class NGHolder;
class ReportManager;
class RoseBuild;

struct CompileContext;
struct ue2_literal;

/** \brief Attempt to consume the entire pattern in graph \a h with Rose.
 * Returns true if successful. */
bool splitOffRose(RoseBuild &rose, const NGHolder &h, bool prefilter,
                  const CompileContext &cc);

/** \brief Attempt to consume the entire pattern in graph \a h with Rose.
 * This is the last attempt to handle a pattern before we resort to an outfix.
 * Returns true if successful. */
bool finalChanceRose(RoseBuild &rose, const NGHolder &h, bool prefilter,
                     const CompileContext &cc);

/** \brief True if the pattern in \a h is consumable by Rose. This function
 * may be conservative (return false even if supported) for efficiency. */
bool checkRose(const ReportManager &rm, const NGHolder &h, bool prefilter,
               const CompileContext &cc);

/** \brief Returns the delay or MO_INVALID_IDX if the graph cannot match with
 * the trailing literal. */
u32 removeTrailingLiteralStates(NGHolder &g, const ue2_literal &lit,
                                u32 max_delay, bool overhang_ok = true);

} // namespace ue2

#endif // NG_ROSE_H
