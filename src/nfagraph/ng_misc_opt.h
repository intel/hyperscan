/*
 * Copyright (c) 2015-2016, Intel Corporation
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
 * \brief Miscellaneous optimisations.
 */

#ifndef NG_MISC_OPT_H
#define NG_MISC_OPT_H

#include <map>
#include <vector>

#include "ng_holder.h"
#include "som/som.h"
#include "util/depth.h"

namespace ue2 {

/** Small structure describing the bounds on a repeat. */
struct BoundedRepeatSummary {
    BoundedRepeatSummary(void) : repeatMin(0), repeatMax(depth::infinity()) {}
    BoundedRepeatSummary(const depth &min_in, const depth &max_in)
        : repeatMin(min_in), repeatMax(max_in) {
        assert(repeatMin <= repeatMax);
        assert(repeatMax.is_reachable());
    }
    bool unbounded(void) const { return repeatMax.is_infinite(); }

    depth repeatMin; //!< minimum repeat bound.
    depth repeatMax; //!< maximum repeat bound.
};

/* returns true if anything changed */
bool improveGraph(NGHolder &g, som_type som);

/** Sometimes the reach of a vertex is greater than it needs to be to reduce
 * stop chars for the benefit of the rest of our code base (accel, etc). In
 * these circumstances, we can treat the reach as the smaller one as
 * the graphs are equivalent. */
CharReach reduced_cr(NFAVertex v, const NGHolder &g,
        const std::map<NFAVertex, BoundedRepeatSummary> &br_cyclic);

std::vector<CharReach> reduced_cr(const NGHolder &g,
           const std::map<NFAVertex, BoundedRepeatSummary> &br_cyclic);

/** Remove cyclic stars connected to start */
bool mergeCyclicDotStars(NGHolder &g);

/**
 * Given a cyclic state 'c' with a broad reach and a later state 'v' that is
 * only reachable if c is still on, then any edges to a successor of a direct
 * successor of c with reach a superset of v are redundant.
 */
bool prunePathsRedundantWithSuccessorOfCyclics(NGHolder &h, som_type som);

} // namespace ue2

#endif
