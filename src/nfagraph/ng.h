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

/** \file
 * \brief NG declaration.
 */

#ifndef NG_H
#define NG_H

#include "ng_holder.h"
#include "ue2common.h"
#include "parser/position.h"
#include "som/slot_manager.h"
#include "som/som.h"
#include "util/boundary_reports.h"
#include "util/compile_context.h"
#include "util/depth.h"
#include "util/graph.h"
#include "util/noncopyable.h"
#include "util/report_manager.h"

#include <deque>
#include <map>
#include <memory>
#include <utility>
#include <vector>

namespace ue2 {

struct CompileContext;
struct ue2_literal;

class ExpressionInfo;
class RoseBuild;
class SmallWriteBuild;

class NG : noncopyable {
public:
    NG(const CompileContext &in_cc, size_t num_patterns,
       unsigned in_somPrecision);
    ~NG();

    /** \brief Consumes a pattern, returns false or throws a CompileError
     * exception if the graph cannot be consumed. */
    bool addGraph(ExpressionInfo &expr, std::unique_ptr<NGHolder> g_ptr);

    /** \brief Consumes a graph, cut-down version of addGraph for use by SOM
     * processing. */
    bool addHolder(NGHolder &h);

    /** \brief Adds a literal to Rose, used by literal shortcut passes (instead
     * of using \ref addGraph) */
    bool addLiteral(const ue2_literal &lit, u32 expr_index, u32 external_report,
                    bool highlander, som_type som, bool quiet);

    /** \brief Maximum history in bytes available for use by SOM reverse NFAs,
     * a hack for pattern support (see UE-1903). This is always set to the max
     * "lookbehind" length. */
    const u32 maxSomRevHistoryAvailable;

    /** \brief The length of the shortest corpus which can match a pattern
     * contained in the NG (excluding the boundary reports used by vacuous
     * patterns, which give an effective minWidth of zero). */
    depth minWidth;

    ReportManager rm;
    SomSlotManager ssm;
    BoundaryReports boundary;
    const CompileContext cc;

    const std::unique_ptr<SmallWriteBuild> smwr; //!< SmallWrite builder.
    const std::unique_ptr<RoseBuild> rose; //!< Rose builder.
};

/** \brief Run graph reduction passes.
 *
 * Shared with the small write compiler.
 */
void reduceGraph(NGHolder &g, som_type som, bool utf8,
                 const CompileContext &cc);

} // namespace ue2

#endif
