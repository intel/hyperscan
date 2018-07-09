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
 * \brief Rose Input Graph: Used for ng_violet -> rose_build_add communication.
 *
 * The input graph MUST be a DAG.
 * There MUST be exactly 1 START or ANCHORED_START vertex.
 * The edges MUST be of the form START->LITERAL, LITERAL->LITERAL,
 *                               LITERAL->ACCEPT or LITERAL->ACCEPT_EOD.
 * Every non START/ANCHORED_START vertex MUST have an in-edge.
 * Every non ACCEPT/ACCEPT_EOD vertex MUST have an out-edge.
 *
 * Edges are either a graph or have bounds associated with them.
 * Graphs on edges to accepts use their internal report ids.
 */

#ifndef ROSE_IN_GRAPH_H
#define ROSE_IN_GRAPH_H

#include "ue2common.h"
#include "rose/rose_common.h"
#include "util/flat_containers.h"
#include "util/ue2_graph.h"
#include "util/ue2string.h"

#include <memory>

namespace ue2 {

class NGHolder;
struct raw_som_dfa;
struct raw_dfa;

enum RoseInVertexType {
    RIV_LITERAL,
    RIV_START,
    RIV_ANCHORED_START,
    RIV_ACCEPT,
    RIV_ACCEPT_EOD
};

struct RoseInVertexProps {
    RoseInVertexProps()
        : type(RIV_LITERAL), delay(0), min_offset(0),
          max_offset(ROSE_BOUND_INF) {}

private:
    template <class ReportContainer>
    RoseInVertexProps(RoseInVertexType type_in, const ue2_literal &s_in,
                      const ReportContainer &reports_in, u32 min_offset_in,
                      u32 max_offset_in)
        : type(type_in), s(s_in), delay(0),
          reports(begin(reports_in), end(reports_in)),
          min_offset(min_offset_in), max_offset(max_offset_in) {}

    // Constructor for a vertex with no reports.
    RoseInVertexProps(RoseInVertexType type_in, const ue2_literal &s_in,
                      u32 min_offset_in, u32 max_offset_in)
        : type(type_in), s(s_in), delay(0), min_offset(min_offset_in),
          max_offset(max_offset_in) {}

public:
    static RoseInVertexProps makeLiteral(const ue2_literal &lit) {
        DEBUG_PRINTF("making literal %s\n", dumpString(lit).c_str());
        return RoseInVertexProps(RIV_LITERAL, lit, 0, ROSE_BOUND_INF);
    }

    template <class ReportContainer>
    static RoseInVertexProps makeAccept(const ReportContainer &rep) {
        DEBUG_PRINTF("making accept for %zu reports\n", rep.size());
        return RoseInVertexProps(RIV_ACCEPT, ue2_literal(), rep, 0,
                                 ROSE_BOUND_INF);
    }

    template <class ReportContainer>
    static RoseInVertexProps makeAcceptEod(const ReportContainer &rep) {
        DEBUG_PRINTF("making accept-eod for %zu reports\n", rep.size());
        return RoseInVertexProps(RIV_ACCEPT_EOD, ue2_literal(), rep, 0,
                                 ROSE_BOUND_INF);
    }

    /* for when there is a suffix graph which handles the reports */
    static RoseInVertexProps makeAcceptEod() {
        return RoseInVertexProps(RIV_ACCEPT_EOD, ue2_literal(), 0,
                                 ROSE_BOUND_INF);
    }

    static RoseInVertexProps makeStart(bool anchored) {
        DEBUG_PRINTF("making %s\n", anchored ? "anchored start" : "start");
        if (anchored) {
            return RoseInVertexProps(RIV_ANCHORED_START, ue2_literal(), 0, 0);
        } else {
            return RoseInVertexProps(RIV_START, ue2_literal(), 0,
                                     ROSE_BOUND_INF);
        }
    }

    RoseInVertexType type; /* polymorphic vertices are probably a bad idea */
    ue2_literal s;   /**< for RIV_LITERAL */
    u32 delay;       /**< for RIV_LITERAL, delay applied to literal. */
    flat_set<ReportID> reports; /**< for RIV_ACCEPT/RIV_ACCEPT_EOD */
    u32 min_offset; /**< Minimum offset at which this vertex can match. */
    u32 max_offset; /**< Maximum offset at which this vertex can match. */
    size_t index = 0; /**< \brief Unique vertex index. */
};

struct RoseInEdgeProps {
    RoseInEdgeProps()
        : minBound(0), maxBound(0), graph(), haig(), graph_lag(0) {}

    RoseInEdgeProps(u32 min_in, u32 max_in)
        : minBound(min_in), maxBound(max_in), graph(), graph_lag(0) {
        assert(minBound <= maxBound);
        assert(minBound != ROSE_BOUND_INF);
    }

    /* haig rosefixes (prefix/infix) require their corresponding holders */
    RoseInEdgeProps(std::shared_ptr<NGHolder> g, std::shared_ptr<raw_som_dfa> h,
                    u32 lag)
        : minBound(0), maxBound(ROSE_BOUND_INF), graph(g), haig(h),
          graph_lag(lag) {
        assert(graph);
        assert(haig);
    }

    /* haig suffixes do not require their corresponding holders */
    explicit RoseInEdgeProps(std::shared_ptr<raw_som_dfa> h)
        : minBound(0), maxBound(ROSE_BOUND_INF), haig(h), graph_lag(0) {
        assert(haig);
    }

    RoseInEdgeProps(std::shared_ptr<NGHolder> g, u32 lag)
        : minBound(0), maxBound(ROSE_BOUND_INF), graph(g), graph_lag(lag) {
        assert(graph);
    }

    /** \brief Minimum bound on 'dot' repeat between literals. ie pred end ->
     * succ begin. */
    u32 minBound;

    /** \brief Maximum bound on 'dot' repeat between literals. */
    u32 maxBound;

    /** \brief Graph on edge. Graph is end to (end - lag). */
    std::shared_ptr<NGHolder> graph;

    /** \brief DFA version of graph, if we have already determinised. */
    std::shared_ptr<raw_dfa> dfa;

    /** \brief Haig version of graph, if required. */
    std::shared_ptr<raw_som_dfa> haig;

    /**
     * \brief Distance behind the match offset for the literal in the target
     * vertex that the leftfix needs to be checked at.
     */
    u32 graph_lag;

    /** \brief Unique edge index. */
    size_t index = 0;
};

struct RoseInGraph
    : public ue2_graph<RoseInGraph, RoseInVertexProps, RoseInEdgeProps> {
};
typedef RoseInGraph::vertex_descriptor RoseInVertex;
typedef RoseInGraph::edge_descriptor RoseInEdge;

} // namespace ue2

#endif
