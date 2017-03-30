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
 * \brief Glushkov construction.
 */

#ifndef BUILDSTATE_H
#define BUILDSTATE_H

#include "ue2common.h"
#include "position.h"
#include "util/noncopyable.h"

#include <memory>
#include <vector>

namespace ue2 {

class NFABuilder;
class PositionInfo;

/** \brief Machinery for Glushkov construction.
 *
 * Abstract base class; use \ref makeGlushkovBuildState to get one of these you
 * can use. */
class GlushkovBuildState : noncopyable {
public:
    /** \brief Represents an uninitialized state. */
    static const Position POS_UNINITIALIZED;

    /** \brief Represents an epsilon transition in the firsts of a component. */
    static const Position POS_EPSILON;

    virtual ~GlushkovBuildState();

    /** \brief Returns a reference to the NFABuilder being used. */
    virtual NFABuilder &getBuilder() = 0;

    /** \brief Returns a const reference to the NFABuilder being used. */
    virtual const NFABuilder &getBuilder() const = 0;

    /** \brief Wire up edges from the lasts of one component to the firsts of
     * another. */
    virtual void connectRegions(const std::vector<PositionInfo> &lasts,
                                const std::vector<PositionInfo> &firsts) = 0;

    /** \brief Wire the lasts of the main sequence to accepts. */
    virtual void connectAccepts(const std::vector<PositionInfo> &lasts) = 0;

    /** \brief Wire up a pair of positions. */
    virtual void addSuccessor(Position from, Position to) = 0;

    /** \brief Clone the vertex properties and edges of all vertices between
     * two positions. */
    virtual void cloneFollowSet(Position from, Position to, u32 offset) = 0;

    /** \brief Build the prioritised list of edges out of our successor map. */
    virtual void buildEdges() = 0;
};

/** \brief Returns a new GlushkovBuildState object. */
std::unique_ptr<GlushkovBuildState> makeGlushkovBuildState(NFABuilder &b,
                                                           bool prefilter);

/** \brief Replace all epsilons with the given positions. */
void replaceEpsilons(std::vector<PositionInfo> &target,
                     const std::vector<PositionInfo> &source);

/** \brief Eliminate lower-priority duplicate PositionInfo entries.
 *
 * Scans through a list of positions and retains only the highest priority
 * version of a given (position, flags) entry. */
void cleanupPositions(std::vector<PositionInfo> &a);

} // namespace ue2

#endif
