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
 * \brief NFA graph vertex depth calculations.
 */

#ifndef STRUCTURAL_ANALYSIS_H
#define STRUCTURAL_ANALYSIS_H

#include "nfagraph/ng_holder.h"
#include "ue2common.h"
#include "util/depth.h"

#include <vector>

namespace ue2 {

class NGHolder;

/**
 * \brief Encapsulates min/max depths relative to the start and startDs
 * vertices.
 */
struct NFAVertexDepth {
    DepthMinMax fromStart;
    DepthMinMax fromStartDotStar;
};

/**
 * \brief Encapsulates min/max depths relative to the accept and acceptEod
 * vertices.
 */
struct NFAVertexRevDepth {
    DepthMinMax toAccept;
    DepthMinMax toAcceptEod;
};

/**
 * \brief Encapsulates min/max depths relative to all of our special vertices.
 */
struct NFAVertexBidiDepth {
    DepthMinMax fromStart;
    DepthMinMax fromStartDotStar;
    DepthMinMax toAccept;
    DepthMinMax toAcceptEod;
};

/**
 * \brief Calculate depths from start and startDs.
 * Fills the vector \p depths (indexed by \p vertex_index).
 */
void calcDepths(const NGHolder &g, std::vector<NFAVertexDepth> &depths);

/**
 * \brief Calculate depths to accept and acceptEod.
 * Fills the vector \p depths (indexed by \p vertex_index).
 */
void calcDepths(const NGHolder &g, std::vector<NFAVertexRevDepth> &depths);

/**
 * \brief Calculate depths to/from all special vertices.
 * Fills the vector \p depths (indexed by \p vertex_index).
 */
void calcDepths(const NGHolder &g, std::vector<NFAVertexBidiDepth> &depths);

/** Calculate the (min, max) depths from the given \p src to every vertex in
 * the graph and return them in a vector, indexed by \p vertex_index. */
void calcDepthsFrom(const NGHolder &g, const NFAVertex src,
                    std::vector<DepthMinMax> &depths);

} // namespace ue2

#endif
