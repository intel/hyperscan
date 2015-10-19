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
 * \brief Sequence of Component objects.
 */


#include "ComponentSequence.h"

#include "buildstate.h"
#include "ComponentAlternation.h"
#include "ComponentRepeat.h"
#include "Parser.h"
#include "ue2common.h"
#include "parse_error.h"
#include "position_dump.h"
#include "position_info.h"
#include "nfagraph/ng_builder.h"
#include "util/container.h"
#include "util/make_unique.h"

#include <algorithm>
#include <cassert>

using namespace std;

namespace ue2 {

ComponentSequence::ComponentSequence() : capture_index(NOT_CAPTURED) {}

ComponentSequence::~ComponentSequence() {}

ComponentSequence::ComponentSequence(const ComponentSequence &other)
    : Component(other), capture_index(other.capture_index) {
    // Deep copy children.
    for (const auto &c : other.children) {
        assert(c);
        children.push_back(unique_ptr<Component>(c->clone()));
    }
    if (other.alternation) {
        const ComponentAlternation &c = *other.alternation;
        alternation.reset(c.clone());
    }
}

ComponentSequence *ComponentSequence::clone() const {
    return new ComponentSequence(*this);
}

Component *ComponentSequence::accept(ComponentVisitor &v) {
    assert(!alternation); // Sequence must be finalized first.

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

void ComponentSequence::accept(ConstComponentVisitor &v) const {
    assert(!alternation); // Sequence must be finalized first.

    v.pre(*this);

    for (auto i = children.begin(), e = children.end(); i != e; ++i) {
        (*i)->accept(v);

        if (i + 1 != e) {
            v.during(*this);
        }
    }

    v.post(*this);
}

void ComponentSequence::addComponent(unique_ptr<Component> comp) {
    children.push_back(move(comp));
}

bool ComponentSequence::addRepeat(u32 min, u32 max,
                                  ComponentRepeat::RepeatType type) {
    if (children.empty() || min > max || max == 0) {
        return false;
    }

    // We can't apply a repeat to some types of component.
    assert(children.back());
    if (!children.back()->repeatable()) {
        return false;
    }

    children.back() = makeComponentRepeat(move(children.back()), min, max,
                                          type);
    assert(children.back());
    return true;
}

void ComponentSequence::addAlternation() {
    if (!alternation) {
        alternation = ue2::make_unique<ComponentAlternation>();
    }

    auto seq = ue2::make_unique<ComponentSequence>();
    seq->children.swap(children);
    alternation->append(move(seq));
}

void ComponentSequence::finalize() {
    if (alternation) {
        addAlternation();
        assert(children.empty());
        children.push_back(move(alternation));
        alternation = nullptr;
    }
}

vector<PositionInfo> ComponentSequence::first() const {
    vector<PositionInfo> firsts, subfirsts;

    for (const auto &c : children) {
        subfirsts = c->first();
        replaceEpsilons(firsts, subfirsts);
        if (!c->empty()) {
            break;
        }
    }

    if (firsts.empty()) {
        DEBUG_PRINTF("trivial empty sequence %zu\n", firsts.size());
        assert(children.empty());
        firsts.push_back(GlushkovBuildState::POS_EPSILON);
    }

    DEBUG_PRINTF("%zu firsts\n", firsts.size());
    return firsts;
}

namespace {
struct eps_info {
    eps_info() : flags(0U) {}
    u32 flags;
};
}

static
void epsilonVisit(vector<eps_info> *info, const vector<PositionInfo> &f) {
    vector<eps_info> out;
    out.reserve(info->size());

    set<u32> seen_flags;

    assert(!info->empty());
    for (auto eps = find(f.begin(), f.end(), GlushkovBuildState::POS_EPSILON);
         eps != f.end();
         eps = find(eps + 1, f.end(), GlushkovBuildState::POS_EPSILON)) {
        for (auto it = info->begin(); it != info->end(); ++it) {
            u32 flags = it->flags | eps->flags;
            if (contains(seen_flags, flags)) {
                continue;
            }

            out.push_back(*it);
            out.back().flags = flags;
            seen_flags.insert(flags);
        }
    }

    info->swap(out);
    assert(!info->empty());
}

static
void applyEpsilonVisits(vector<PositionInfo> &lasts,
                        const vector<eps_info> &eps_visits) {
    vector<PositionInfo> out;
    out.reserve(lasts.size() * eps_visits.size());

    for (const auto &last : lasts) {
        for (const auto &e : eps_visits) {
            out.push_back(last);
            out.back().flags |= e.flags;
        }
    }

    cleanupPositions(out);
    lasts.swap(out);
}

vector<PositionInfo> ComponentSequence::last() const {
    vector<PositionInfo> lasts, sublasts;
    vector<eps_info> visits(1);

    auto i = children.rbegin(), e = children.rend();
    for (; i != e; ++i) {
        sublasts = (*i)->last();
        applyEpsilonVisits(sublasts, visits);
        lasts.insert(lasts.end(), sublasts.begin(), sublasts.end());
        if ((*i)->empty()) {
            // this epsilon's flags should propagate to subsequent lasts'
            // enter/exit lists
            epsilonVisit(&visits, (*i)->first());
        } else {
            break;
        }
    }

    DEBUG_PRINTF("lasts = %s\n",
                 dumpPositions(lasts.begin(), lasts.end()).c_str());
    return lasts;
}

bool ComponentSequence::empty(void) const {
    // a sequence can be empty if all its subcomponents can be empty
    for (const auto &c : children) {
        if (!c->empty()) {
            return false;
        }
    }
    return true;
}

void ComponentSequence::notePositions(GlushkovBuildState &bs) {
    u32 pb = bs.getBuilder().numVertices();
    for (auto &c : children) {
        c->notePositions(bs);
    }
    recordPosBounds(pb, bs.getBuilder().numVertices());
}

void ComponentSequence::buildFollowSet(GlushkovBuildState &bs,
                                       const vector<PositionInfo> &lastPos) {
    DEBUG_PRINTF("sequence of %zu components\n", children.size());

    // If no components, no work to do.
    if (children.empty()) {
        return;
    }

    // First element
    children.front()->buildFollowSet(bs, lastPos);
    if (children.size() == 1) {
        // If our sequence contains precisely one component, then we've done
        // all our work. Hooking up its firsts and lasts will be done by our
        // parent component.
        return;
    }

    // Remaining elements, wiring last to first in sequence.

    vector<PositionInfo> prevLasts = children.front()->last();

    for (auto it = next(children.begin()), ite = children.end(); it != ite; ++it) {
        assert(*it);
        Component &c = *(*it);

        // Build subcomponent follow set
        c.buildFollowSet(bs, prevLasts);

        // FIRST(curr)
        vector<PositionInfo> currFirsts(c.first());

        // LAST(prev) => FIRST(curr)
        DEBUG_PRINTF("connecting lasts (|| %zu) to firsts of comp %zd\n",
                     prevLasts.size(), it - children.begin());
        bs.connectRegions(prevLasts, currFirsts);

        // Generate a new LAST(prev) for the next iteration; either c->last()
        // on its own if it can't be empty or c->last unioned with the previous
        // last if c can be empty
        vector<PositionInfo> currLasts(c.last());

        if (!c.empty()) {
            // Current component can't be empty, so use its lasts only
            prevLasts.swap(currLasts);
            DEBUG_PRINTF("swapped lasts\n");
        } else {
            // Add current lasts to previous lasts
            DEBUG_PRINTF("doing stuff for empty comp\n");
            prevLasts.insert(prevLasts.end(), currLasts.begin(), currLasts.end());
            DEBUG_PRINTF("done stuff for empty comp\n");
        }
    }
}

bool ComponentSequence::checkEmbeddedStartAnchor(bool at_start) const {
    for (const auto &c : children) {
        at_start = c->checkEmbeddedStartAnchor(at_start);
    }

    return at_start;
}

bool ComponentSequence::checkEmbeddedEndAnchor(bool at_end) const {
    // Note reversed ordering.
    for (auto i = children.rbegin(), e = children.rend(); i != e; ++i) {
        at_end = (*i)->checkEmbeddedEndAnchor(at_end);
    }

    return at_end;
}

bool ComponentSequence::vacuous_everywhere() const {
    for (const auto &c : children) {
        if (!c->vacuous_everywhere()) {
            return false;
        }
    }
    return true;
}

void ComponentSequence::optimise(bool connected_to_sds) {
    DEBUG_PRINTF("opt %d\n", (int)connected_to_sds);
    for (u32 i = 0; i < children.size();) {
        DEBUG_PRINTF("opt %u: ctsds: %d\n", i, (int)connected_to_sds);
        Component &sub = *children[i];

        sub.optimise(connected_to_sds);

        bool vacuous = sub.vacuous_everywhere();

        if (connected_to_sds && vacuous) {
            DEBUG_PRINTF("delete opt %u\n", i);
            auto it = children.begin() + i;
            children.erase(it);
            continue;
        }

        connected_to_sds = connected_to_sds && vacuous;
        i++;
    }
}

} // namespace ue2
