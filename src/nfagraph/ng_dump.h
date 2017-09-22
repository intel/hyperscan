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
 * \brief Dump code for NFA graphs.
 */

#ifndef NG_DUMP_H
#define NG_DUMP_H

#include "grey.h"
#include "ng_holder.h" // for graph types
#include "ue2common.h"

#include <unordered_map>

#ifdef DUMP_SUPPORT
#include <fstream>
#endif

struct RoseEngine;

namespace ue2 {

class NGHolder;
class NG;
class ExpressionInfo;
class ReportManager;

// Implementations for stubs below -- all have the suffix "Impl".

#ifdef DUMP_SUPPORT

template <typename GraphT>
void dumpGraphImpl(const char *name, const GraphT &g);

template <typename GraphT>
void dumpGraphImpl(const char *name, const GraphT &g, const ReportManager &rm);

void dumpDotWrapperImpl(const NGHolder &g, const ExpressionInfo &expr,
                        const char *name, const Grey &grey);

void dumpComponentImpl(const NGHolder &g, const char *name, u32 expr, u32 comp,
                       const Grey &grey);

void dumpSomSubComponentImpl(const NGHolder &g, const char *name, u32 expr,
                             u32 comp, u32 plan, const Grey &grey);

void dumpHolderImpl(const NGHolder &h, unsigned int stageNumber,
                    const char *stageName, const Grey &grey);

// Variant that takes a region map as well.
void dumpHolderImpl(const NGHolder &h,
                    const std::unordered_map<NFAVertex, u32> &region_map,
                    unsigned int stageNumber, const char *stageName,
                    const Grey &grey);

template <typename GraphT>
static inline void dumpGraph(UNUSED const char *name, UNUSED const GraphT &g) {
    dumpGraphImpl(name, g);
}

#endif // DUMP_SUPPORT

// Stubs which call through to dump code if compiled in.

UNUSED static inline
void dumpDotWrapper(UNUSED const NGHolder &g, UNUSED const ExpressionInfo &expr,
                    UNUSED const char *name, UNUSED const Grey &grey) {
#ifdef DUMP_SUPPORT
    dumpDotWrapperImpl(g, expr, name, grey);
#endif
}

UNUSED static inline
void dumpComponent(UNUSED const NGHolder &h, UNUSED const char *name,
                   UNUSED u32 expr, UNUSED u32 comp, UNUSED const Grey &grey) {
#ifdef DUMP_SUPPORT
    dumpComponentImpl(h, name, expr, comp, grey);
#endif
}

UNUSED static inline
void dumpSomSubComponent(UNUSED const NGHolder &h, UNUSED const char *name,
                         UNUSED u32 expr, UNUSED u32 comp, UNUSED u32 plan,
                         UNUSED const Grey &grey) {
#ifdef DUMP_SUPPORT
    dumpSomSubComponentImpl(h, name, expr, comp, plan, grey);
#endif
}

UNUSED static inline
void dumpHolder(UNUSED const NGHolder &h, UNUSED unsigned int stageNumber,
                UNUSED const char *name, UNUSED const Grey &grey) {
#ifdef DUMP_SUPPORT
    dumpHolderImpl(h, stageNumber, name, grey);
#endif
}

UNUSED static inline
void dumpHolder(UNUSED const NGHolder &h,
                UNUSED const std::unordered_map<NFAVertex, u32> &region_map,
                UNUSED unsigned int stageNumber, UNUSED const char *name,
                UNUSED const Grey &grey) {
#ifdef DUMP_SUPPORT
    dumpHolderImpl(h, region_map, stageNumber, name, grey);
#endif
}

#ifdef DUMP_SUPPORT
void dumpReportManager(const ReportManager &rm, const Grey &grey);
void dumpSmallWrite(const RoseEngine *rose, const Grey &grey);
#else
static UNUSED
void dumpReportManager(const ReportManager &, const Grey &) {
}
static UNUSED
void dumpSmallWrite(const RoseEngine *, const Grey &) {
}
#endif

#ifdef DUMP_SUPPORT
// replace boost's graphviz writer
template <typename GraphT, typename WriterT, typename VertexID>
static void writeGraphviz(std::ostream &out, const GraphT &g, WriterT w,
                          const VertexID &vertex_id) {
    const std::string delimiter(" -> ");
    out << "digraph G {" << std::endl;

    typename boost::graph_traits<GraphT>::vertex_iterator i, end;
    for(boost::tie(i,end) = vertices(g); i != end; ++i) {
        out << get(vertex_id, *i);
        w(out, *i); // print vertex attributes
        out << ";" << std::endl;
    }
    typename boost::graph_traits<GraphT>::edge_iterator ei, edge_end;
    for(boost::tie(ei, edge_end) = edges(g); ei != edge_end; ++ei) {
        out << (get(vertex_id, source(*ei, g))) << delimiter
            << (get(vertex_id, target(*ei, g))) << " ";
        w(out, *ei); // print edge attributes
        out << ";" << std::endl;
    }
    out << "}" << std::endl;
}

#endif // DUMP_SUPPORT

} // namespace ue2

#endif // NG_DUMP_H
