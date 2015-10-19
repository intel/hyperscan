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
 * \brief Represents an empty regex element, like (?m)
 */
#include <cassert>

#include "ComponentEmpty.h"
#include "position.h"
#include "position_info.h"
#include "buildstate.h"
#include "ue2common.h"

using namespace std;

namespace ue2 {

ComponentEmpty::ComponentEmpty() {
    // Surprise, it's EMPTY!
}

ComponentEmpty::~ComponentEmpty() {
    // Surprise, it's EMPTY!
}

ComponentEmpty *ComponentEmpty::clone() const { return new ComponentEmpty(); }

bool ComponentEmpty::empty() const {
    return true;
}

bool ComponentEmpty::vacuous_everywhere(void) const {
    return true;
}

bool ComponentEmpty::repeatable() const {
    // This is the whole point of this class. Empty constructs like '(?m)' are
    // not repeatable.
    return false;
}

vector<PositionInfo> ComponentEmpty::first() const {
    return vector<PositionInfo>(1, GlushkovBuildState::POS_EPSILON);
}

vector<PositionInfo> ComponentEmpty::last() const {
    return vector<PositionInfo>();
}

void ComponentEmpty::notePositions(GlushkovBuildState &) {
    // Nothing to do.
}

void ComponentEmpty::buildFollowSet(GlushkovBuildState &,
                                    const vector<PositionInfo> &) {
    // Nothing to do.
}

bool ComponentEmpty::checkEmbeddedStartAnchor(bool at_start) const {
    return at_start;
}

bool ComponentEmpty::checkEmbeddedEndAnchor(bool at_end) const {
    return at_end;
}

} // namespace ue2
