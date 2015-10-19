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
 * \brief Alternations (foo|bar|baz).
 */


#include "ComponentAlternation.h"

#include "buildstate.h"
#include "position.h"
#include "position_info.h"
#include "nfagraph/ng_builder.h"
#include "ue2common.h"

#include <algorithm>

using namespace std;

namespace ue2 {

ComponentAlternation::ComponentAlternation() {
    // empty
}

ComponentAlternation::~ComponentAlternation() {
    // empty
}

ComponentAlternation::ComponentAlternation(const ComponentAlternation &other)
    : Component(other) {
    for (const auto &c : other.children) {
        assert(c);
        children.push_back(unique_ptr<Component>(c->clone()));
    }
}

ComponentAlternation * ComponentAlternation::clone() const {
    return new ComponentAlternation(*this);
}

Component *ComponentAlternation::accept(ComponentVisitor &v) {
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

void ComponentAlternation::accept(ConstComponentVisitor &v) const {
    v.pre(*this);
    for (auto i = children.begin(), e = children.end(); i != e; ++i) {
        (*i)->accept(v);
        if (i + 1 != e) {
            v.during(*this);
        }
    }

    v.post(*this);
}

void ComponentAlternation::append(unique_ptr<Component> component) {
    children.push_back(move(component));
}

vector<PositionInfo> ComponentAlternation::first() const {
    // firsts come from all our subcomponents in position order. This will
    // maintain left-to-right priority order.
    vector<PositionInfo> firsts, subfirsts;

    for (const auto &c : children) {
        subfirsts = c->first();
        firsts.insert(firsts.end(), subfirsts.begin(), subfirsts.end());
    }
    return firsts;
}

vector<PositionInfo> ComponentAlternation::last() const {
    vector<PositionInfo> lasts, sublasts;

    for (const auto &c : children) {
        sublasts = c->last();
        lasts.insert(lasts.end(), sublasts.begin(), sublasts.end());
    }
    return lasts;
}

bool ComponentAlternation::empty(void) const {
    // an alternation can be empty if any of its components are empty
    for (const auto &c : children) {
        if (c->empty()) {
            return true;
        }
    }

    return false;
}

void ComponentAlternation::notePositions(GlushkovBuildState &bs) {
    u32 pb = bs.getBuilder().numVertices();
    for (auto &c : children) {
        c->notePositions(bs);
    }
    recordPosBounds(pb, bs.getBuilder().numVertices());
}

void ComponentAlternation::buildFollowSet(GlushkovBuildState &bs,
                                          const vector<PositionInfo> &lastPos) {
    for (auto &c : children) {
        c->buildFollowSet(bs, lastPos);
    }
}

bool ComponentAlternation::checkEmbeddedStartAnchor(bool at_start) const {
    bool rv = at_start;
    for (const auto &c : children) {
        rv &= c->checkEmbeddedStartAnchor(at_start);
    }

    return rv;
}

bool ComponentAlternation::checkEmbeddedEndAnchor(bool at_end) const {
    bool rv = at_end;
    for (const auto &c : children) {
        rv &= c->checkEmbeddedEndAnchor(at_end);
    }

    return rv;
}

bool ComponentAlternation::vacuous_everywhere(void) const {
    for (const auto &c : children) {
        if (c->vacuous_everywhere()) {
            return true;
        }
    }
    return false;
}

void ComponentAlternation::optimise(bool connected_to_sds) {
    for (auto &c : children) {
        c->optimise(connected_to_sds);
    }
}

} // namespace ue2
