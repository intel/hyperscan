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

#ifndef NFAGRAPH_COMMON_H
#define NFAGRAPH_COMMON_H

#include "compiler/compiler.h"
#include "grey.h"
#include "nfagraph/ng_builder.h"
#include "nfagraph/ng.h"
#include "nfagraph/ng_width.h"
#include "util/target_info.h"

namespace ue2 {

// Helper function: construct a graph from an expression, flags and context.
inline
std::unique_ptr<NGHolder> constructGraphWithCC(const std::string &expr,
                                               CompileContext &cc,
                                               unsigned flags) {
    ReportManager rm(cc.grey);
    ParsedExpression parsed(0, expr.c_str(), flags, 0);
    auto built_expr = buildGraph(rm, cc, parsed);
    return std::move(built_expr.g);
}

// Helper function: construct a graph from an expression and its flags.
inline
std::unique_ptr<NGHolder> constructGraph(const std::string &expr,
                                         unsigned flags) {
    CompileContext cc(false, false, get_current_target(), Grey());
    return constructGraphWithCC(expr, cc, flags);
}

} // namespace ue2

#endif // NFAGRAPH_COMMON_H
