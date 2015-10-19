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
 * \brief Lookahead/lookbehind zero-width assertions.
 */
#include "ComponentAssertion.h"
#include "buildstate.h"
#include "position.h"
#include "position_info.h"
#include "ue2common.h"

#include <cassert>
#include <algorithm>

using namespace std;

namespace ue2 {

ComponentAssertion::ComponentAssertion(enum Direction dir, enum Sense sense)
    : m_dir(dir), m_sense(sense) {}

ComponentAssertion::~ComponentAssertion() { }

ComponentAssertion *ComponentAssertion::clone() const {
    return new ComponentAssertion(*this);
}

Component * ComponentAssertion::accept(ComponentVisitor &v) {
    Component *c = v.visit(this);
    if (c != this) {
        v.post(this);
        return c;
    }

    for (auto i = children.begin(), e = children.end(); i != e; ++i) {
        Component *child = i->get();
        c = (*i)->accept(v);
        if (c != child) {
            // Child has been replaced (new Component pointer) or we've been
            // instructed to delete it (null).
            i->reset(c);
        }
    }

    // Remove deleted children.
    children.erase(remove(children.begin(), children.end(), nullptr),
                   children.end());

    v.post(this);
    return this;
}

void ComponentAssertion::accept(ConstComponentVisitor &v) const {
    v.pre(*this);
    for (auto i = children.begin(), e = children.end(); i != e; ++i) {
        (*i)->accept(v);
        if (i + 1 != e) {
            v.during(*this);
        }
    }

    v.post(*this);
}

vector<PositionInfo> ComponentAssertion::first() const {
    assert(0);
    return vector<PositionInfo>();
}

vector<PositionInfo> ComponentAssertion::last() const {
    assert(0);
    return vector<PositionInfo>();
}

bool ComponentAssertion::empty() const {
    return true;
}

void ComponentAssertion::notePositions(GlushkovBuildState &) {
    assert(0);
}

void ComponentAssertion::buildFollowSet(GlushkovBuildState &,
                                        const vector<PositionInfo> &) {
    assert(0);
}

bool ComponentAssertion::repeatable() const {
    // If this assertion has no children (it's an empty sequence, like that
    // produced by '(?!)') then PCRE would throw a "nothing to repeat" error.
    // So we do as well.
    return !children.empty();
}

} // namespace ue2
