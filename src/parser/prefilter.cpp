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
 * \brief Prefiltering component tree transformation.
 */
#include "ComponentAssertion.h"
#include "ComponentAtomicGroup.h"
#include "ComponentBackReference.h"
#include "ComponentBoundary.h"
#include "ComponentClass.h"
#include "ComponentCondReference.h"
#include "ComponentRepeat.h"
#include "ComponentSequence.h"
#include "ComponentVisitor.h"
#include "ComponentWordBoundary.h"
#include "ConstComponentVisitor.h"
#include "Parser.h"
#include "prefilter.h"

#include <algorithm>
#include <stack>

using namespace std;

namespace ue2 {

/** \brief Max number of positions a referent can have to be considered safe to
 * replace a reference in prefiltering mode. */
static const size_t MAX_REFERENT_POSITIONS = 1;

/** \brief Constructs a \ref ComponentClass that matches a dot (any
 * byte/codepoint, depending on whether UTF-8). */
static
unique_ptr<ComponentClass> makeDotClass(const ParseMode &mode_in) {
    ParseMode mode(mode_in);
    mode.dotall = true;
    return generateComponent(CLASS_ANY, false, mode);
}

namespace {

/**
 * \brief Visitor used to determine if a given referent component is safe to
 * replace its reference in prefiltering mode. Throws
 * SafeReferentVisitor::Unsafe to terminate early on unsafe cases. */
class SafeReferentVisitor : public DefaultConstComponentVisitor {
public:
    struct Unsafe {};

    SafeReferentVisitor() : numPositions(0) {}

    bool is_safe() const {
        DEBUG_PRINTF("numPositions = %zu\n", numPositions);
        return numPositions <= MAX_REFERENT_POSITIONS;
    }

    using DefaultConstComponentVisitor::pre;
    using DefaultConstComponentVisitor::post;

    void pre(const AsciiComponentClass &) override {
        numPositions++;
    }

    void pre(const UTF8ComponentClass &) override {
        // FIXME: we should be able to tell precisely how many positions this
        // class will use. Right now, use the worst case.
        numPositions += 4;
    }

    void pre(const ComponentBoundary &) override {
        numPositions++;
    }

    void pre(const ComponentByte &) override {
        numPositions++;
    }

    void pre(const ComponentEUS &) override {
        numPositions++;
    }

    void pre(const ComponentRepeat &) override {
        // Record the number of positions used before we visit the contents of
        // the repeat.
        countStack.push(numPositions);
    }

    void post(const ComponentRepeat &c) override {
        assert(!countStack.empty());
        size_t before = countStack.top();
        countStack.pop();
        assert(before <= numPositions);

        std::pair<u32, u32> bounds = c.getBounds();
        size_t subPositions = numPositions - before;
        size_t copies = bounds.second < ComponentRepeat::NoLimit
                            ? bounds.second
                            : max(bounds.first, 1U);
        numPositions = before + (subPositions * copies);
    }

    void pre(const ComponentWordBoundary &) override {
        // not quite accurate, as these are expanded out in assert
        // resolution...
        numPositions++;
    }

    void pre(const ComponentBackReference &) override {
        throw Unsafe();
    }

    void pre(const ComponentCondReference &) override {
        throw Unsafe();
    }

private:
    size_t numPositions;

    // For temporary use
    std::stack<size_t> countStack;
};

static
bool isSafeReferent(const Component &c) {
    try {
        SafeReferentVisitor vis;
        c.accept(vis);
        return vis.is_safe();
    }
    catch (const SafeReferentVisitor::Unsafe &) {
        return false;
    }
}

/**
 * \brief Visitor to find the \ref ComponentSequence with a given reference ID
 * or name: if found, the visitor will throw a const ptr to it.
 */
class FindSequenceVisitor : public DefaultConstComponentVisitor {
public:
    explicit FindSequenceVisitor(unsigned ref_id) : id(ref_id) {}
    explicit FindSequenceVisitor(const std::string &s) : name(s) {}

    using DefaultConstComponentVisitor::pre;

    void pre(const ComponentSequence &c) override {
        if (!name.empty()) {
            if (c.getCaptureName() == name) {
                throw &c;
            }
        } else if (c.getCaptureIndex() == id) {
            throw &c;
        }
    }
private:
    const std::string name;
    const unsigned id = 0;
};

static
const ComponentSequence *findCapturingGroup(const Component *root,
                                            FindSequenceVisitor &vis) {
    try {
        root->accept(vis);
        DEBUG_PRINTF("group not found\n");
        return nullptr;
    } catch (const ComponentSequence *seq) {
        return seq;
    }
}

} // namespace

/**
 * \brief Visitor to apply prefilter reductions, swapping components for which
 * we don't have real implementations with implementable ones. Any such
 * replacement should produce a superset of the matches that would be produced
 * by the original.
 */
class PrefilterVisitor : public DefaultComponentVisitor {
public:
    PrefilterVisitor(Component *c, const ParseMode &m) : root(c), mode(m) {}
    ~PrefilterVisitor() override;

    using DefaultComponentVisitor::visit;

    /** \brief Calls the visitor (recursively) on a new replacement component
     * we've just created. Takes care of freeing it if the sequence is itself
     * replaced. */
    template<class T>
    Component *visit_replacement(T *r) {
        Component *c = r->accept(*this);
        if (c != r) {
            delete r;
        }
        return c;
    }

    Component *visit(ComponentBackReference *c) override {
        assert(c);

        // If the referent is simple (represents a single position), then we
        // replace the back-reference with a copy of it.
        const ComponentSequence *ref = nullptr;
        const std::string &ref_name = c->getRefName();
        const unsigned ref_id = c->getRefID();
        if (!ref_name.empty()) {
            FindSequenceVisitor vis(ref_name);
            ref = findCapturingGroup(root, vis);
        } else if (ref_id > 0) {
            FindSequenceVisitor vis(ref_id);
            ref = findCapturingGroup(root, vis);
        }

        if (ref && isSafeReferent(*ref)) {
            DEBUG_PRINTF("found safe ref %p\n", ref);
            ComponentSequence *seq = ref->clone();
            // Remove labels from cloned sequence.
            seq->setCaptureName("");
            seq->setCaptureIndex(ComponentSequence::NOT_CAPTURED);

            return visit_replacement(seq);
        }

        // Replace with ".*".
        auto rep = makeComponentRepeat(makeDotClass(mode), 0,
                                       ComponentRepeat::NoLimit,
                                       ComponentRepeat::REPEAT_GREEDY);
        return rep.release(); // FIXME: owning raw ptr
    }

    Component *visit(UNUSED ComponentAssertion *c) override {
        assert(c);
        // Replace with an empty sequence.
        return new ComponentSequence();
    }

    Component *visit(ComponentRepeat *c) override {
        assert(c);
        // Possessive repeats become greedy.
        if (c->type == ComponentRepeat::REPEAT_POSSESSIVE) {
            c->type = ComponentRepeat::REPEAT_GREEDY;
        }
        return c;
    }

    Component *visit(ComponentAtomicGroup *c) override {
        assert(c);
        // Replace with a plain sequence containing the atomic group's
        // children.
        ComponentSequence *seq = new ComponentSequence();
        const auto &children = c->getChildren();
        for (const auto &child : children) {
            assert(child);
            seq->addComponent(unique_ptr<Component>(child->clone()));
        }

        return visit_replacement(seq);
    }

    Component *visit(UNUSED ComponentEUS *c) override {
        assert(c);
        // Replace with ".+".
        auto rep = makeComponentRepeat(makeDotClass(mode), 1,
                                       ComponentRepeat::NoLimit,
                                       ComponentRepeat::REPEAT_GREEDY);
        return rep.release(); // FIXME: owning raw ptr
    }

    Component *visit(ComponentWordBoundary *c) override {
        assert(c);

        // TODO: Right now, we do not have correct code for resolving these
        // when prefiltering is on, UCP is on, and UTF-8 is *off*. For now, we
        // just replace with an empty sequence (as that will return a superset
        // of matches).
        if (mode.ucp && !mode.utf8) {
            return new ComponentSequence();
        }

        // All other cases can be prefiltered.
        c->setPrefilter(true);
        return c;
    }

    Component *visit(ComponentCondReference *c) override {
        assert(c);
        // Replace with a plain sequence containing the conditional reference's
        // children.
        ComponentSequence *seq = new ComponentSequence();
        const auto &children = c->getChildren();

        // Empty children is accepted by PCRE as a "do nothing" case.
        if (children.empty()) {
            return seq;
        }

        for (const auto &child : children) {
            assert(child);
            seq->addComponent(unique_ptr<Component>(child->clone()));
        }

        // If the conditional reference had just a YES branch, we want this to
        // be an alternation with an empty sequence (the NO branch).
        if (!c->hasBothBranches) {
            seq->addAlternation();
            seq->finalize();
        }

        return visit_replacement(seq);
    }

private:
    Component *root;
    const ParseMode &mode;
};

PrefilterVisitor::~PrefilterVisitor() {}

void prefilterTree(unique_ptr<Component> &root, const ParseMode &mode) {
    assert(root);
    PrefilterVisitor vis(root.get(), mode);

    Component *c = root->accept(vis);
    if (c != root.get()) {
        root.reset(c);
    }
}

} // namespace ue2
