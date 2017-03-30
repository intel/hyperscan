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
 * \brief: NFA Graph Builder: used by Glushkov construction to construct an
 * NGHolder from a parsed expression.
 */

#ifndef NG_BUILDER_H
#define NG_BUILDER_H

#include "ue2common.h"

#include "parser/position.h"
#include "util/noncopyable.h"

#include <memory>

namespace ue2 {

class CharReach;
class ReportManager;
struct BuiltExpression;
struct CompileContext;

class ParsedExpression;

/** \brief Abstract builder interface. Use \ref makeNFABuilder to construct
 * one. Used by GlushkovBuildState. */
class NFABuilder : noncopyable {
public:
    virtual ~NFABuilder();

    virtual Position makePositions(size_t nPositions) = 0;
    virtual Position getStart() const = 0;
    virtual Position getStartDotStar() const = 0;
    virtual Position getAccept() const = 0;
    virtual Position getAcceptEOD() const = 0;

    virtual bool isSpecialState(Position p) const = 0;

    virtual void setNodeReportID(Position position, int offsetAdjust) = 0;
    virtual void addCharReach(Position position, const CharReach &cr) = 0;

    /* or-in vertex assertions */
    virtual void setAssertFlag(Position position, u32 flag) = 0;
    virtual u32 getAssertFlag(Position position) = 0;

    virtual void addVertex(Position p) = 0;

    virtual void addEdge(Position start, Position end) = 0;

    virtual bool hasEdge(Position start, Position end) const = 0;

    virtual u32 numVertices() const = 0;

    virtual void cloneRegion(Position first, Position last,
                             unsigned posOffset) = 0;

    /**
     * \brief Returns the built NGHolder graph and ExpressionInfo.
     * Note that this builder cannot be used after this call.
     */
    virtual BuiltExpression getGraph() = 0;
};

/** Construct a usable NFABuilder. */
std::unique_ptr<NFABuilder> makeNFABuilder(ReportManager &rm,
                                           const CompileContext &cc,
                                           const ParsedExpression &expr);

} // namespace ue2

#endif
