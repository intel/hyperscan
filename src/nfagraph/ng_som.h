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
 * \brief SOM ("Start of Match") analysis.
 */

#ifndef NG_SOM_H
#define NG_SOM_H

#include "som/som.h"
#include "ue2common.h"

namespace ue2 {

class ExpressionInfo;
class NG;
class NGHolder;
class ReportManager;
struct Grey;

enum sombe_rv {
    SOMBE_FAIL,
    SOMBE_HANDLED_INTERNAL,
    SOMBE_HANDLED_ALL
};

/** \brief Perform SOM analysis on the given graph.
 *
 * This function will replace report IDs and mutate the graph, then return
 * SOMBE_HANDLED_INTERNAL if SOM can be established and the full graph still
 * needs to be handled (rose, etc).
 *
 * Returns SOMBE_HANDLED_ALL if everything has been done and the pattern has
 * been handled in all its glory.
 *
 * Returns SOMBE_FAIL and does not mutate the graph if SOM cannot be
 * established.
 *
 *  May throw a "Pattern too large" exception if prefixes of the
 * pattern are too large to compile.
 */
sombe_rv doSom(NG &ng, NGHolder &h, const ExpressionInfo &expr, u32 comp_id,
               som_type som);

/** Returns SOMBE_FAIL (and the original graph) if SOM cannot be established.
 * May also throw pattern too large if prefixes of the pattern are too large to
 * compile. */
sombe_rv doSomWithHaig(NG &ng, NGHolder &h, const ExpressionInfo &expr,
                       u32 comp_id, som_type som);

void makeReportsSomPass(ReportManager &rm, NGHolder &g);

} // namespace ue2

#endif // NG_SOM_H
