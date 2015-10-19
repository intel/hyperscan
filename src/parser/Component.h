/*
 * Copyright (c) 2015, Intel Corporation
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
 * \brief Base class for all components.
 */

#ifndef _RE_COMPONENT_H_
#define _RE_COMPONENT_H_

#include "ComponentVisitor.h"
#include "ConstComponentVisitor.h"

#include "position.h"
#include "ue2common.h"

#include <set>
#include <string>
#include <vector>

namespace ue2 {

class GlushkovBuildState;
class PositionInfo;

enum EmptyPathType {
    NOT_EMPTY,       /**< component must consume characters */
    EPS_ONLY_PATHS,  /**< eps path with no overhanging asserts */
    BOUNDARY_PATHS   /**< eps paths some with overhanging asserts */
};

/** \brief Base class for regular expression parse tree components. */
class Component {
    friend class DumpVisitor;
public:
    /** \brief Constructor. */
    Component();

    /** \brief Destructor. */
    virtual ~Component();

    /** \brief Returns a newly-allocated deep copy of this component. */
    virtual Component *clone() const = 0;

    /** \brief Apply the given visitor functor. */
    virtual Component *accept(ComponentVisitor &v) = 0;

    /** \brief Apply the given const visitor functor. */
    virtual void accept(ConstComponentVisitor &v) const = 0;

    /** \brief Glushkov construction First() function.
     * \return set of initial positions in this component. */
    virtual std::vector<PositionInfo> first() const = 0;

    /** \brief Glushkov construction Last() function.
     * \return set of final positions in this component. */
    virtual std::vector<PositionInfo> last() const = 0;

    /** \brief Glushkov construction Empty() function.
     * \return true iff the component accepts epsilon.
     *
     * Note: ^, $, etc are considered empty. */
    virtual bool empty() const = 0;

    /** \brief True iff epsilon can pass through the component.
     *
     * Note: ^, $, etc are not vacuous everywhere. */
    virtual bool vacuous_everywhere(void) const;

    /** \brief True iff the component is repeatable on its own, without being
     * encapsulated in a sequence first.
     *
     * This is true for most components, but not for repeats, anchors and word
     * boundaries. */
    virtual bool repeatable() const;

    /** \brief Optimisation pass on the component tree.
     *
     * Called before \ref notePositions. May modify to the component tree.
     * Assumes no start of match information is required.
     */
    virtual void optimise(bool connected_to_sds);

    /** \brief Informs the Glushkov build process of the positions used by this
     * component. */
    virtual void notePositions(GlushkovBuildState &bs) = 0;

    /** \brief Glushkov construction Follow() function.
     *
     * Constructs (in \a bs) the set of positions in this component reachable
     * from the positions in \a lastPos.
     *
     * \throw ParseError on failure
     */
    virtual void buildFollowSet(GlushkovBuildState &bs,
                                const std::vector<PositionInfo> &lastPos) = 0;

    /** \brief Return value is used for chaining, throws if finds embedded
     * anchor. */
    virtual bool checkEmbeddedStartAnchor(bool at_start) const;

    /* \brief Return value is used for chaining, throws if finds embedded
     * anchor. */
    virtual bool checkEmbeddedEndAnchor(bool at_end) const;

protected:
    /** \brief Called during \ref notePositions. */
    void recordPosBounds(u32 b, u32 e);

    u32 pos_begin;
    u32 pos_end;

    // Protected copy ctor. Use clone instead.
    Component(const Component &other)
        : pos_begin(other.pos_begin), pos_end(other.pos_end) {}
};

} // namespace ue2

#endif
