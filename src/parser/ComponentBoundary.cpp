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
 * \brief Boundary assertions (^, $, \\A, \\Z, \\z)
 */


#include "ComponentBoundary.h"

#include "buildstate.h"
#include "parse_error.h"
#include "position.h"
#include "position_info.h"
#include "Parser.h"
#include "util/charreach.h"
#include "nfagraph/ng_builder.h"

#include <cassert>

using namespace std;

namespace ue2 {

ComponentBoundary::ComponentBoundary(enum Boundary bound)
    : m_bound(bound), m_newline(GlushkovBuildState::POS_UNINITIALIZED) {}

ComponentBoundary::~ComponentBoundary() {
}

ComponentBoundary::ComponentBoundary(const ComponentBoundary &other)
    : Component(other), m_bound(other.m_bound), m_newline(other.m_newline),
      m_first(other.m_first), m_last(other.m_last) {}

ComponentBoundary * ComponentBoundary::clone() const {
    return new ComponentBoundary(*this);
}

vector<PositionInfo> ComponentBoundary::first() const {
    return m_first;
}

vector<PositionInfo> ComponentBoundary::last() const {
    return m_last;
}

bool ComponentBoundary::empty() const {
    return true;
}

bool ComponentBoundary::repeatable() const {
    return false;
}

static
Position makeNewline(GlushkovBuildState &bs) {
    NFABuilder &builder = bs.getBuilder();
    Position newline = builder.makePositions(1);
    builder.addCharReach(newline, CharReach('\n'));
    return newline;
}

void ComponentBoundary::notePositions(GlushkovBuildState & bs) {
    NFABuilder &builder = bs.getBuilder();
    const Position startState = builder.getStart();

    switch (m_bound) {
    case BEGIN_STRING: // beginning of data stream ('^')
    {
        PositionInfo epsilon(GlushkovBuildState::POS_EPSILON);
        epsilon.flags = POS_FLAG_NOFLOAT;
        m_first.push_back(epsilon);

        // We have the start vertex in firsts so that we can discourage
        // the mid-pattern use of boundaries.
        m_first.push_back(startState);

        break;
    }
    case BEGIN_LINE: // multiline anchor: beginning of stream or a newline
    {
        PositionInfo epsilon(GlushkovBuildState::POS_EPSILON);
        epsilon.flags = POS_FLAG_NOFLOAT;
        m_first.push_back(epsilon);

        // We have the start vertex in firsts so that we can discourage
        // the mid-pattern use of boundaries.
        m_first.push_back(startState);

        // Newline
        m_newline = makeNewline(bs);
        builder.setAssertFlag(m_newline, POS_FLAG_MULTILINE_START);
        builder.setAssertFlag(m_newline, POS_FLAG_VIRTUAL_START);
        PositionInfo nl(m_newline);
        nl.flags = POS_FLAG_MUST_FLOAT | POS_FLAG_FIDDLE_ACCEPT;
        m_first.push_back(nl);
        m_last.push_back(nl);
        recordPosBounds(m_newline, m_newline + 1);
        break;
    }
    case END_STRING: // end of data stream ('\z')
    {
        PositionInfo epsilon(GlushkovBuildState::POS_EPSILON);
        epsilon.flags = POS_FLAG_WIRE_EOD | POS_FLAG_NO_NL_EOD |
                        POS_FLAG_NO_NL_ACCEPT | POS_FLAG_ONLY_ENDS;
        m_first.push_back(epsilon);
        break;
    }
    case END_STRING_OPTIONAL_LF: // end of data with optional LF ('$')
    {
        PositionInfo epsilon(GlushkovBuildState::POS_EPSILON);
        epsilon.flags = POS_FLAG_WIRE_EOD | POS_FLAG_WIRE_NL_EOD |
                        POS_FLAG_NO_NL_ACCEPT | POS_FLAG_ONLY_ENDS;
        m_first.push_back(epsilon);
        break;
    }
    case END_LINE: // multiline anchor: end of data or a newline
    {
        PositionInfo epsilon(GlushkovBuildState::POS_EPSILON);
        epsilon.flags = POS_FLAG_WIRE_EOD | POS_FLAG_WIRE_NL_EOD |
                        POS_FLAG_WIRE_NL_ACCEPT | POS_FLAG_ONLY_ENDS;
        m_first.push_back(epsilon);
        break;
    }
    default:
        // unsupported
        assert(0);
        break;
    }
}

void ComponentBoundary::buildFollowSet(GlushkovBuildState &,
                                       const vector<PositionInfo> &) {

}

bool ComponentBoundary::checkEmbeddedStartAnchor(bool at_start) const {
    if (at_start) {
        return at_start;
    }

    if (m_bound == BEGIN_STRING || m_bound == BEGIN_LINE) {
        throw ParseError("Embedded start anchors not supported.");
    }

    return at_start;
}

bool ComponentBoundary::checkEmbeddedEndAnchor(bool at_end) const {
    if (at_end) {
        return at_end;
    }

    if (m_bound != BEGIN_STRING && m_bound != BEGIN_LINE) {
        throw ParseError("Embedded end anchors not supported.");
    }

    return at_end;
}

} // namespace
