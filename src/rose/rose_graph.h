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
 * \brief BGL graph structures used internally by the Rose build process.
 *
 * BGL graph structures used internally by the build-time portion of Rose. The
 * graph used for input is in rose_in_graph.h since it's part of the RoseBuild
 * external API.
 */

#ifndef ROSE_GRAPH_H
#define ROSE_GRAPH_H

#include "ue2common.h"
#include "rose_build.h"
#include "rose_internal.h"
#include "nfa/nfa_internal.h" // for MO_INVALID_IDX
#include "util/depth.h"
#include "util/flat_containers.h"
#include "util/ue2_graph.h"

#include <memory>
#include <set>

namespace ue2 {

struct CastleProto;
struct raw_dfa;
struct raw_som_dfa;
struct TamaProto;

/** \brief Table type for a literal. */
enum rose_literal_table {
    ROSE_ANCHORED,             //!< literals anchored to start
    ROSE_FLOATING,             //!< general floating literals
    ROSE_EOD_ANCHORED,         //!< literals that match near EOD
    ROSE_ANCHORED_SMALL_BLOCK, //!< anchored literals for small block table
    ROSE_EVENT                 //!< "literal-like" events, such as EOD
};

/** \brief Edge history types. */
enum RoseRoleHistory {
    ROSE_ROLE_HISTORY_NONE,      //!< no special history
    ROSE_ROLE_HISTORY_ANCH,      //!< previous role is at a fixed offset
    ROSE_ROLE_HISTORY_LAST_BYTE, //!< previous role can only match at EOD
    ROSE_ROLE_HISTORY_INVALID    //!< history not yet assigned
};

#include "util/order_check.h"

/** \brief Provides information about the (pre|in)fix engine to the left of a
 * role. */
struct LeftEngInfo {
    std::shared_ptr<NGHolder> graph;
    std::shared_ptr<CastleProto> castle;
    std::shared_ptr<raw_dfa> dfa;
    std::shared_ptr<raw_som_dfa> haig;
    std::shared_ptr<TamaProto> tamarama;
    u32 lag = 0U;
    ReportID leftfix_report = MO_INVALID_IDX;
    depth dfa_min_width{0};
    depth dfa_max_width = depth::infinity();

    bool operator==(const LeftEngInfo &other) const {
        return other.graph == graph
            && other.castle == castle
            && other.dfa == dfa
            && other.haig == haig
            && other.tamarama == tamarama
            && other.lag == lag
            && other.leftfix_report == leftfix_report;
    }
    bool operator!=(const LeftEngInfo &other) const {
        return !(*this == other);
    }
    bool operator<(const LeftEngInfo &b) const {
        const LeftEngInfo &a = *this;
        ORDER_CHECK(graph);
        ORDER_CHECK(castle);
        ORDER_CHECK(dfa);
        ORDER_CHECK(haig);
        ORDER_CHECK(tamarama);
        ORDER_CHECK(lag);
        ORDER_CHECK(leftfix_report);
        return false;
    }
    size_t hash() const;
    void reset(void);
    explicit operator bool() const;
    bool tracksSom() const { return !!haig; }
};

/** \brief Provides information about the suffix engine to the right of a
 * role. */
struct RoseSuffixInfo {
    u32 top = 0;
    std::shared_ptr<NGHolder> graph; /* if triggers a trailing nfa */
    std::shared_ptr<CastleProto> castle;
    std::shared_ptr<raw_som_dfa> haig;
    std::shared_ptr<raw_dfa> rdfa;
    std::shared_ptr<TamaProto> tamarama;
    depth dfa_min_width{0};
    depth dfa_max_width = depth::infinity();

    bool operator==(const RoseSuffixInfo &b) const;
    bool operator!=(const RoseSuffixInfo &b) const { return !(*this == b); }
    bool operator<(const RoseSuffixInfo &b) const;
    size_t hash() const;
    void reset(void);
    explicit operator bool() const { return graph || castle || haig || rdfa || tamarama; }
};

/** \brief Properties attached to each Rose graph vertex. */
struct RoseVertexProps {
    /** \brief Unique dense vertex index. Used for BGL algorithms. */
    size_t index = ~size_t{0};

    /** \brief IDs of literals in the Rose literal map. */
    flat_set<u32> literals;

    /**
     * \brief If true, this vertex is a virtual vertex for firing reports at
     * EOD. These vertices must have reports and have no associated literals.
     */
    bool eod_accept = false;

    /** \brief Report IDs to fire. */
    flat_set<ReportID> reports;

    /** \brief Bitmask of groups that this role sets. */
    rose_group groups = 0;

    /** \brief Minimum role (end of literal) offset depth in bytes. */
    u32 min_offset = ~u32{0};

    /** \brief Maximum role (end of literal) offset depth in bytes */
    u32 max_offset = 0;

    /** \brief SOM for the role is offset from end match offset */
    u32 som_adjust = 0;

    /** \brief Prefix/infix engine to the left of this role. */
    LeftEngInfo left;

    /**
     * \brief Suffix engine to the right of this role.
     *
     * Note: information about triggered infixes is associated with the left of
     * the destination role.
     */
    RoseSuffixInfo suffix;

    bool isBoring(void) const;
    bool fixedOffset(void) const;
};

/** \brief Properties attached to each Rose graph edge. */
/* bounds are distance from end of prev to start of the next */
struct RoseEdgeProps {
    /** \brief Unique dense vertex index. Used for BGL algorithms. */
    size_t index = ~size_t{0};

    /**
     * \brief Minimum distance from the end of the source role's match to the
     * start of the target role's match.
     *
     * Not used when the target has a left engine (as the engine represents
     * bounds).
     */
    u32 minBound = 0;

    /**
     * \brief Maximum distance from the end of the source role's match to the
     * start of the target role's match.
     *
     * Not used when the target has a left engine (as the engine represents
     * bounds).
     */
    u32 maxBound = 0;

    /** \brief Which top to trigger on the target role's left engine. */
    u32 rose_top = 0;

    /** \brief True if the rose_top can clear all other previous tops. */
    u8 rose_cancel_prev_top = false;

    /** \brief History required by this edge. */
    RoseRoleHistory history = ROSE_ROLE_HISTORY_INVALID;
};

bool operator<(const RoseEdgeProps &a, const RoseEdgeProps &b);

/**
 * \brief Core Rose graph structure.
 */
struct RoseGraph : public ue2_graph<RoseGraph, RoseVertexProps, RoseEdgeProps> {
    friend class RoseBuildImpl; /* to allow index renumbering */
};
using RoseVertex = RoseGraph::vertex_descriptor;
using RoseEdge = RoseGraph::edge_descriptor;

} // namespace ue2

#endif // ROSE_GRAPH_H
