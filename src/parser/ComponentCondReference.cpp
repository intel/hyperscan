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
 * \brief Conditional reference.
 */
#include "ComponentCondReference.h"
#include "ComponentAlternation.h"
#include "ComponentAssertion.h"
#include "parse_error.h"
#include "position_info.h"

#include <algorithm>
#include <cassert>
#include <memory>

using namespace std;

namespace ue2 {

ComponentCondReference::ComponentCondReference(unsigned ref)
    : kind(CONDITION_NUMBER), ref_id(ref), hasBothBranches(false) {}

ComponentCondReference::ComponentCondReference(const string &name)
    : kind(CONDITION_NAME), ref_id(0), ref_name(name), hasBothBranches(false) {}

ComponentCondReference::ComponentCondReference(unique_ptr<Component> c)
    : kind(CONDITION_ASSERTION), ref_id(0), assertion(move(c)),
      hasBothBranches(false) {}

ComponentCondReference::~ComponentCondReference() {}

ComponentCondReference::ComponentCondReference(
    const ComponentCondReference &other)
    : ComponentSequence(other), kind(other.kind), ref_id(other.ref_id),
      ref_name(other.ref_name), hasBothBranches(other.hasBothBranches) {
    if (kind == CONDITION_ASSERTION) {
        assert(other.assertion);
        assertion.reset(other.assertion->clone());
    } else {
        assert(!other.assertion);
    }
}

ComponentCondReference *ComponentCondReference::clone() const {
    return new ComponentCondReference(*this);
}

Component *ComponentCondReference::accept(ComponentVisitor &v) {
    Component *c = v.visit(this);
    if (c != this) {
        v.post(this);
        return c;
    }

    if (kind == CONDITION_ASSERTION) {
        Component *a = assertion.get();
        c = assertion->accept(v);
        if (c != a) {
            assertion.reset(c);
        }
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

void ComponentCondReference::accept(ConstComponentVisitor &v) const {
    v.pre(*this);

    if (kind == CONDITION_ASSERTION) {
        assertion->accept(v);
        v.during(*this); // FIXME: a good idea?
    }

    for (auto i = children.begin(), e = children.end(); i != e; ++i) {
        (*i)->accept(v);
        if (i + 1 != e) {
            v.during(*this);
        }
    }

    v.post(*this);
}

void ComponentCondReference::addAlternation() {
    if (alternation) {
        if (ref_name == "DEFINE") {
            throw LocatedParseError("DEFINE conditional group with more than "
                                    "one branch");
        }

        if (alternation->numBranches() >= 2) {
            throw LocatedParseError("Conditional with more than two branches");
        }
    }
    hasBothBranches = true;
    ComponentSequence::addAlternation();
}

vector<PositionInfo> ComponentCondReference::first() const {
    assert(0);
    return vector<PositionInfo>();
}

vector<PositionInfo> ComponentCondReference::last() const {
    assert(0);
    return vector<PositionInfo>();
}

bool ComponentCondReference::empty() const { return true; }

void ComponentCondReference::notePositions(GlushkovBuildState &) { assert(0); }

void ComponentCondReference::buildFollowSet(GlushkovBuildState &,
                                            const vector<PositionInfo> &) {
    assert(0);
}

bool ComponentCondReference::repeatable() const {
    // If this assertion has no children (it's an empty sequence, like that
    // produced by '(?!)') then PCRE would throw a "nothing to repeat" error.
    // So we do as well.
    return !children.empty();
}

} // namespace ue2
