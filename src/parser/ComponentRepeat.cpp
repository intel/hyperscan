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
 * \brief Repeats ('*', '+', '?', '{M,N}', etc)
 */


#include "ComponentRepeat.h"

#include "buildstate.h"
#include "nfagraph/ng_builder.h"
#include "parse_error.h"
#include "Parser.h"
#include "position.h"
#include "position_dump.h"
#include "position_info.h"
#include "ue2common.h"
#include "util/make_unique.h"

#include <algorithm>
#include <cassert>

using namespace std;

namespace ue2 {

/** \brief Hard limit on the maximum repeat for bounded repeats. */
static constexpr u32 MAX_REPEAT = 32767;

/** \brief If expanding a repeat would lead to this many positions being
 * generated, we fail the pattern. */
static constexpr u32 MAX_POSITIONS_EXPANDED = 500000; // arbitrarily huge

/* no edge priorities means that if our subcomponent can be empty, our min
 * extent is effectively zero. */
ComponentRepeat::ComponentRepeat(unique_ptr<Component> sub_comp_in, u32 min,
                                 u32 max, enum RepeatType t)
    : type(t), sub_comp(move(sub_comp_in)), m_min(min), m_max(max),
      posFirst(GlushkovBuildState::POS_UNINITIALIZED),
      posLast(GlushkovBuildState::POS_UNINITIALIZED) {
    assert(sub_comp);
    assert(max > 0);
    assert(m_min <= m_max);

    if (m_min > MAX_REPEAT) {
        throw ParseError("Bounded repeat is too large.");
    }
    if (m_max != NoLimit && m_max > MAX_REPEAT) {
        throw ParseError("Bounded repeat is too large.");
    }
}

ComponentRepeat::~ComponentRepeat() {}

ComponentRepeat *ComponentRepeat::clone() const {
    return new ComponentRepeat(*this);
}

ComponentRepeat::ComponentRepeat(const ComponentRepeat &other)
    : Component(other),
      type(other.type), sub_comp(unique_ptr<Component>(other.sub_comp->clone())),
      m_min(other.m_min), m_max(other.m_max),
      m_firsts(other.m_firsts), m_lasts(other.m_lasts),
      posFirst(other.posFirst), posLast(other.posLast) {}

bool ComponentRepeat::empty() const {
    return m_min == 0 || sub_comp->empty();
}

bool ComponentRepeat::repeatable() const {
    return false;
}

static
void addBase(Position base, vector<PositionInfo> &firsts,
             vector<PositionInfo> &lasts) {
    for (auto &e : firsts) {
        if (e.pos != GlushkovBuildState::POS_EPSILON) {
            e.pos += base;
        }
    }
    for (auto &e : lasts) {
        e.pos += base;
    }
}

static
void checkPositions(vector<PositionInfo> &v, const GlushkovBuildState &bs) {
    const NFABuilder& builder = bs.getBuilder();
    for (const auto &e : v) {
        if (builder.isSpecialState(e.pos)) {
            throw ParseError("Embedded anchors not supported.");
        }
    }
}

void ComponentRepeat::notePositions(GlushkovBuildState &bs) {
    assert(m_max > 0);
    assert(m_max == NoLimit || m_max < MAX_REPEAT);

    /* Note: We can construct smaller subgraphs if we're not maintaining edge
     * priorities. */

    // We create one copy only through a recursive call to notePositions(),
    // first() and last(). Then we clone its positions and store the
    // appropriate firsts and lasts values for the copies.
    posFirst = bs.getBuilder().numVertices();
    sub_comp->notePositions(bs);

    u32 copies = m_max < NoLimit ? m_max : MAX(m_min, 1);
    DEBUG_PRINTF("building %u copies of repeated region\n", copies);
    m_firsts.clear();
    m_lasts.clear();
    m_firsts.resize(copies);
    m_lasts.resize(copies);

    m_firsts[0] = sub_comp->first();
    m_lasts[0] = sub_comp->last();

    postSubNotePositionHook();

    posLast = bs.getBuilder().numVertices() - 1;
    u32 vcount = posLast + 1 - posFirst;

    // If we're making more than one copy, then our firsts and lasts must only
    // contain vertices inside [posFirst, posLast]: anything else means we have
    // an embedded anchor or otherwise weird situation.
    if (copies > 1) {
        checkPositions(m_firsts[0], bs);
        checkPositions(m_lasts[0], bs);
    }

    // Avoid enormous expansions
    if (vcount * copies > MAX_POSITIONS_EXPANDED) {
        throw ParseError("Bounded repeat is too large.");
    }

    // Add positions for the rest of the copies
    size_t copyPositions = vcount * (copies - 1);
    bs.getBuilder().makePositions(copyPositions);

    // Calculate our firsts and lasts for the copies
    for (u32 i = 1; i < copies; ++i) {
        m_firsts[i] = m_firsts[0];
        m_lasts[i] = m_lasts[0];
        u32 base = i * vcount;
        addBase(base, m_firsts[i], m_lasts[i]);
    }

    recordPosBounds(posFirst, bs.getBuilder().numVertices());

    // Each optional repeat has an epsilon at the end of its firsts list.
    for (u32 i = m_min; i < m_firsts.size(); i++) {
        m_firsts[i].push_back(GlushkovBuildState::POS_EPSILON);
    }

}

vector<PositionInfo> ComponentRepeat::first() const {
    if (!m_max) {
        return {};
    }

    assert(!m_firsts.empty()); // notePositions should already have run
    const vector<PositionInfo> &firsts = m_firsts.front();
    DEBUG_PRINTF("firsts = %s\n",
                 dumpPositions(begin(firsts), end(firsts)).c_str());
    return firsts;
}

void ComponentRepeat::buildFollowSet(GlushkovBuildState &bs,
                                     const vector<PositionInfo> &lastPos) {
    if (!m_max) {
        return;
    }
    DEBUG_PRINTF("enter\n");

    // Wire up the first (the "real") entry

    DEBUG_PRINTF("initial repeat\n");
    sub_comp->buildFollowSet(bs, lastPos);

    // Clone the subgraph we just added N times, where N is the minimum extent
    // of the graph minus one, wiring them up in a linear sequence

    u32 copies = m_firsts.size();
    DEBUG_PRINTF("cloning %u copies of repeat\n", copies - 1);
    for (u32 rep = 1; rep < copies; rep++) {
        u32 offset = (posLast + 1 - posFirst) * rep;
        if (offset > 0) {
            bs.cloneFollowSet(posFirst, posLast, offset);
        }
    }

    wireRepeats(bs);

    DEBUG_PRINTF("leave\n");
}

void ComponentRepeat::optimise(bool connected_to_sds) {
    DEBUG_PRINTF("opt %d\n", (int)connected_to_sds);
    if (!connected_to_sds) {
        return;
    }

    DEBUG_PRINTF("setting m_max to %u\n", m_min);
    m_max = m_min;
}

bool ComponentRepeat::vacuous_everywhere() const {
    return !m_min || sub_comp->vacuous_everywhere();
}

bool ComponentRepeat::checkEmbeddedStartAnchor(bool at_start) const {
    at_start = sub_comp->checkEmbeddedStartAnchor(at_start);

    if (m_max > 1) {
        at_start = sub_comp->checkEmbeddedStartAnchor(at_start);
    }

    return at_start;
}

bool ComponentRepeat::checkEmbeddedEndAnchor(bool at_end) const {
    at_end = sub_comp->checkEmbeddedEndAnchor(at_end);

    if (m_max > 1) {
        at_end = sub_comp->checkEmbeddedEndAnchor(at_end);
    }

    return at_end;
}

Component *ComponentRepeat::accept(ComponentVisitor &v) {
    Component *c = v.visit(this);
    if (c != this) {
        v.post(this);
        return c;
    }

    c = sub_comp->accept(v);
    if (c != sub_comp.get()) {
        sub_comp.reset(c);
    }

    v.post(this);
    return !sub_comp ? nullptr : this;
}

void ComponentRepeat::accept(ConstComponentVisitor &v) const {
    v.pre(*this);
    sub_comp->accept(v);
    v.post(*this);
}

vector<PositionInfo> ComponentRepeat::last() const {
    vector<PositionInfo> lasts;
    if (!m_max) {
        return lasts;
    }

    assert(!m_firsts.empty()); // notePositions should already have run
    assert(!m_lasts.empty());

    const auto &l = m_min ? m_lasts[m_min - 1] : m_lasts[0];
    lasts.insert(lasts.end(), l.begin(), l.end());

    if (!m_min || m_min != m_lasts.size()) {
        lasts.insert(lasts.end(), m_lasts.back().begin(), m_lasts.back().end());
    }

    DEBUG_PRINTF("lasts = %s\n",
                 dumpPositions(lasts.begin(), lasts.end()).c_str());
    return lasts;
}

void ComponentRepeat::wireRepeats(GlushkovBuildState &bs) {
    /* note: m_lasts[0] already valid */
    u32 copies = m_firsts.size();
    const bool isEmpty = sub_comp->empty();
    const vector<PositionInfo> &optLasts =
        m_min ? m_lasts[m_min - 1] : m_lasts[0];

    if (!copies) {
        goto inf_check;
    }

    DEBUG_PRINTF("wiring up %u mand repeats\n", m_min);
    for (u32 rep = 1; rep < m_min; rep++) {
        bs.connectRegions(m_lasts[rep - 1], m_firsts[rep]);

        if (isEmpty) {
            m_lasts[rep].insert(m_lasts[rep].end(), m_lasts[rep - 1].begin(),
                                m_lasts[rep - 1].end());
        }
    }

    DEBUG_PRINTF("wiring up %d optional repeats\n", copies - m_min);
    for (u32 rep = MAX(m_min, 1); rep < copies; rep++) {
        vector<PositionInfo> lasts = m_lasts[rep - 1];
        if (rep != m_min) {
            lasts.insert(lasts.end(), optLasts.begin(), optLasts.end());
            sort(lasts.begin(), lasts.end());
            lasts.erase(unique(lasts.begin(), lasts.end()), lasts.end());
        }
        bs.connectRegions(lasts, m_firsts[rep]);
    }

inf_check:
    // If we have no max bound, we need a self-loop as well.
    if (m_max == NoLimit) {
        DEBUG_PRINTF("final repeat self-loop\n");
        bs.connectRegions(m_lasts.back(), m_firsts.back());
    }
}

static
bool hasPositionFlags(const Component &c) {
    for (const auto &e : c.first()) {
        if (e.flags) {
            return true;
        }
    }
    return false;
}

void ComponentRepeat::postSubNotePositionHook() {
    // UE-444 optimization: we can REWRITE m_min under various circumstances,
    // so that we create smaller NFA graphs. Note that this is _not_ possible
    // if our subcomponent contains a flagged position, e.g. nofloat.
    if (!hasPositionFlags(*sub_comp) && sub_comp->empty()) {
        m_min = 0;
    }
}

unique_ptr<ComponentRepeat> makeComponentRepeat(unique_ptr<Component> sub_comp,
                                                u32 min, u32 max,
                                                ComponentRepeat::RepeatType t) {
    return ue2::make_unique<ComponentRepeat>(move(sub_comp), min, max, t);
}

} // namespace ue2
