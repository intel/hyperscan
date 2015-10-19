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
 * \brief Word Boundary Assertion (\\b or \\B)
 */
#include "ComponentWordBoundary.h"
#include "buildstate.h"
#include "parse_error.h"
#include "Parser.h"
#include "position_info.h"
#include "nfagraph/ng_builder.h"

using namespace std;

namespace ue2 {

ComponentWordBoundary::ComponentWordBoundary(u32 loc_in, bool neg,
                                             const ParseMode &mode)
    : loc(loc_in), position(GlushkovBuildState::POS_UNINITIALIZED),
      negated(neg), ucp(mode.ucp), prefilter(false) {}

ComponentWordBoundary::~ComponentWordBoundary() {
    // empty
}

ComponentWordBoundary * ComponentWordBoundary::clone() const {
    return new ComponentWordBoundary(*this);
}

vector<PositionInfo> ComponentWordBoundary::first() const {
    vector<PositionInfo> firsts;
    firsts.push_back(position);
    return firsts;
}

vector<PositionInfo> ComponentWordBoundary::last() const {
    // Same as firsts
    return first();
}

bool ComponentWordBoundary::empty() const {
    return false;
}

bool ComponentWordBoundary::repeatable() const {
    return false;
}

void ComponentWordBoundary::notePositions(GlushkovBuildState &bs) {
    NFABuilder &builder = bs.getBuilder();
    position = builder.makePositions(1);

    if (ucp) {
        assert(prefilter); // only in prefiltering mode!
        if (negated) {
            builder.setAssertFlag(position, POS_FLAG_ASSERT_WORD_TO_WORD_UCP
                                  | POS_FLAG_ASSERT_NONWORD_TO_NONWORD_UCP);
        } else {
            builder.setAssertFlag(position, POS_FLAG_ASSERT_WORD_TO_NONWORD_UCP
                                  | POS_FLAG_ASSERT_NONWORD_TO_WORD_UCP);
        }
    } else {
        if (negated) {
            builder.setAssertFlag(position, POS_FLAG_ASSERT_WORD_TO_WORD
                                  | POS_FLAG_ASSERT_NONWORD_TO_NONWORD);
        } else {
            builder.setAssertFlag(position, POS_FLAG_ASSERT_WORD_TO_NONWORD
                                  | POS_FLAG_ASSERT_NONWORD_TO_WORD);
        }
    }
    recordPosBounds(position, position + 1);
}

void ComponentWordBoundary::buildFollowSet(GlushkovBuildState&,
                                           const vector<PositionInfo>&) {
    // No internal connections, nowt to do
}

} // namespace ue2
