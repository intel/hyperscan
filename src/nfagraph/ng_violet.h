/*
 * Copyright (c) 2016-2017, Intel Corporation
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
 * \brief Violet method of rose construction from NGHolder.
 */

#ifndef NG_VIOLET_H
#define NG_VIOLET_H

#include "ue2common.h"

namespace ue2 {

class NGHolder;
class RoseBuild;

struct CompileContext;
class ReportManager;
struct RoseInGraph;

/** \brief Attempt to consume the entire pattern in graph \a h with Rose.
 * Returns true if successful. */
bool doViolet(RoseBuild &rose, const NGHolder &h, bool prefilter,
              bool last_chance, const ReportManager &rm,
              const CompileContext &cc);

bool ensureImplementable(RoseBuild &rose, RoseInGraph &vg, bool allow_changes,
                         bool final_chance, const ReportManager &rm,
                         const CompileContext &cc);

/** \brief True if the pattern in \a h is consumable by Rose/Violet. This
 * function may be conservative (return false even if supported) for
 * efficiency. */
bool checkViolet(const ReportManager &rm, const NGHolder &h, bool prefilter,
                 const CompileContext &cc);

} // namespace ue2

#endif
