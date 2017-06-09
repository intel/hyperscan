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

#include "ng_builder.h"

#include "grey.h"
#include "ng.h"
#include "ng_util.h"
#include "ue2common.h"
#include "compiler/compiler.h" // for ParsedExpression
#include "util/compile_error.h"
#include "util/make_unique.h"

#include <cassert>

using namespace std;

namespace ue2 {

namespace {

/** Concrete implementation of NFABuilder interface. */
class NFABuilderImpl : public NFABuilder {
public:
    NFABuilderImpl(ReportManager &rm, const Grey &grey,
                   const ParsedExpression &expr);

    ~NFABuilderImpl() override;

    Position makePositions(size_t nPositions) override;
    Position getStart() const override;
    Position getStartDotStar() const override;
    Position getAccept() const override;
    Position getAcceptEOD() const override;

    bool isSpecialState(Position p) const override;

    void setNodeReportID(Position position, int offsetAdjust) override;
    void addCharReach(Position position, const CharReach &cr) override;
    void setAssertFlag(Position position, u32 flag) override;
    u32 getAssertFlag(Position position) override;

    void addVertex(Position p) override;

    void addEdge(Position start, Position end) override;

    bool hasEdge(Position start, Position end) const override;

    u32 numVertices() const override { return vertIdx; }

    void cloneRegion(Position first, Position last,
                     unsigned posOffset) override;

    BuiltExpression getGraph() override;

private:
    /** fetch a vertex given its Position ID. */
    NFAVertex getVertex(Position pos) const;

    /** \brief Internal convenience function to add an edge (u, v). */
    pair<NFAEdge, bool> addEdge(NFAVertex u, NFAVertex v);

    /** \brief We use the ReportManager to hand out new internal reports. */
    ReportManager &rm;

    /** \brief Greybox: used for resource limits. */
    const Grey &grey;

    /** \brief Underlying graph. */
    unique_ptr<NGHolder> graph;

    /** \brief Underlying expression info. */
    ExpressionInfo expr;

    /** \brief mapping from position to vertex. Use \ref getVertex for access.
     * */
    vector<NFAVertex> id2vertex;

    /** \brief Index of next vertex. */
    u32 vertIdx;
}; // class NFABuilderImpl

} // namespace

NFABuilderImpl::NFABuilderImpl(ReportManager &rm_in, const Grey &grey_in,
                               const ParsedExpression &parsed)
    : rm(rm_in), grey(grey_in), graph(ue2::make_unique<NGHolder>()),
      expr(parsed.expr), vertIdx(N_SPECIALS) {

    // Reserve space for a reasonably-sized NFA
    id2vertex.reserve(64);
    id2vertex.resize(N_SPECIALS);
    id2vertex[NODE_START] = graph->start;
    id2vertex[NODE_START_DOTSTAR] = graph->startDs;
    id2vertex[NODE_ACCEPT] = graph->accept;
    id2vertex[NODE_ACCEPT_EOD] = graph->acceptEod;
}

NFABuilderImpl::~NFABuilderImpl() {
    // empty
}

NFAVertex NFABuilderImpl::getVertex(Position pos) const {
    assert(id2vertex.size() >= pos);
    const NFAVertex v = id2vertex[pos];
    assert(v != NGHolder::null_vertex());
    assert((*graph)[v].index == pos);
    return v;
}

void NFABuilderImpl::addVertex(Position pos) {
    // Enforce resource limit.
    if (pos > grey.limitGraphVertices) {
        throw CompileError("Pattern too large.");
    }

    NFAVertex v = add_vertex(*graph);
    if (id2vertex.size() <= pos) {
        id2vertex.resize(pos + 1);
    }
    id2vertex[pos] = v;
    (*graph)[v].index = pos;
}

BuiltExpression NFABuilderImpl::getGraph() {
    DEBUG_PRINTF("built graph has %zu vertices and %zu edges\n",
                 num_vertices(*graph), num_edges(*graph));

    if (num_edges(*graph) > grey.limitGraphEdges) {
        throw CompileError("Pattern too large.");
    }
    if (num_vertices(*graph) > grey.limitGraphVertices) {
        throw CompileError("Pattern too large.");
    }

    return { expr, move(graph) };
}

void NFABuilderImpl::setNodeReportID(Position pos, int offsetAdjust) {
    Report ir = rm.getBasicInternalReport(expr, offsetAdjust);
    DEBUG_PRINTF("setting report id on %u = (%u, %d, %u)\n",
                 pos, expr.report, offsetAdjust, ir.ekey);

    NFAVertex v = getVertex(pos);
    auto &reports = (*graph)[v].reports;
    reports.clear();
    reports.insert(rm.getInternalId(ir));
}

void NFABuilderImpl::addCharReach(Position pos, const CharReach &cr) {
    NFAVertex v = getVertex(pos);
    (*graph)[v].char_reach |= cr;
}

void NFABuilderImpl::setAssertFlag(Position pos, u32 flag) {
    NFAVertex v = getVertex(pos);
    (*graph)[v].assert_flags |= flag;
}

u32 NFABuilderImpl::getAssertFlag(Position pos) {
    NFAVertex v = getVertex(pos);
    return (*graph)[v].assert_flags;
}

pair<NFAEdge, bool> NFABuilderImpl::addEdge(NFAVertex u, NFAVertex v) {
    // assert that the edge doesn't already exist
    assert(edge(u, v, *graph).second == false);

    return add_edge(u, v, *graph);
}

void NFABuilderImpl::addEdge(Position startPos, Position endPos) {
    DEBUG_PRINTF("%u -> %u\n", startPos, endPos);
    assert(startPos < vertIdx);
    assert(endPos < vertIdx);

    NFAVertex u = getVertex(startPos);
    NFAVertex v = getVertex(endPos);

    if ((u == graph->start || u == graph->startDs) && v == graph->startDs) {
        /* standard special -> special edges already exist */
        assert(edge(u, v, *graph).second == true);
        return;
    }

    assert(edge(u, v, *graph).second == false);
    addEdge(u, v);
}

bool NFABuilderImpl::hasEdge(Position startPos, Position endPos) const {
    return edge(getVertex(startPos), getVertex(endPos), *graph).second;
}

Position NFABuilderImpl::getStart() const {
    return NODE_START;
}

Position NFABuilderImpl::getStartDotStar() const {
    return NODE_START_DOTSTAR;
}

Position NFABuilderImpl::getAccept() const {
    return NODE_ACCEPT;
}

Position NFABuilderImpl::getAcceptEOD() const {
    return NODE_ACCEPT_EOD;
}

bool NFABuilderImpl::isSpecialState(Position p) const {
    return (p == NODE_START || p == NODE_START_DOTSTAR ||
            p == NODE_ACCEPT || p == NODE_ACCEPT_EOD);
}

Position NFABuilderImpl::makePositions(size_t nPositions) {
    Position base = vertIdx;
    for (size_t i = 0; i < nPositions; i++) {
        addVertex(vertIdx++);
    }
    DEBUG_PRINTF("built %zu positions from base %u\n", nPositions, base);
    return base;
}

void NFABuilderImpl::cloneRegion(Position first, Position last, unsigned posOffset) {
    NGHolder &g = *graph;
    assert(posOffset > 0);

    // walk the nodes between first and last and copy their vertex properties
    DEBUG_PRINTF("cloning nodes in [%u, %u], offset %u\n", first, last,
                 posOffset);
    for (Position i = first; i <= last; ++i) {
        NFAVertex orig = getVertex(i);
        Position destIdx = i + posOffset;
        assert(destIdx < vertIdx);
        NFAVertex dest = getVertex(destIdx);
        g[dest] = g[orig]; // all properties
        g[dest].index = destIdx;
    }
}

unique_ptr<NFABuilder> makeNFABuilder(ReportManager &rm, const CompileContext &cc,
                           const ParsedExpression &expr) {
    return ue2::make_unique<NFABuilderImpl>(rm, cc.grey, expr);
}

NFABuilder::~NFABuilder() { }

} // namespace ue2
