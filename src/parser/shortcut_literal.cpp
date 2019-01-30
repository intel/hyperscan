/*
 * Copyright (c) 2015-2019, Intel Corporation
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
 * \brief Shortcut literal pass: directly add literal components to Rose.
 */
#include "AsciiComponentClass.h"
#include "Utf8ComponentClass.h"
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
#include "parse_error.h"
#include "shortcut_literal.h"
#include "grey.h"
#include "nfagraph/ng.h"
#include "compiler/compiler.h"
#include "util/ue2string.h"
#include "ue2common.h"

#include <stack>

using namespace std;

namespace ue2 {

/**
 * \brief Visitor that constructs a ue2_literal from a component tree.
 *
 * If a component that can't be part of a literal is encountered, this visitor
 * will throw ConstructLiteralVisitor::NotLiteral.
 */
class ConstructLiteralVisitor : public ConstComponentVisitor {
public:
    ~ConstructLiteralVisitor() override;

    /** \brief Thrown if this component does not represent a literal. */
    struct NotLiteral {};

    void pre(const AsciiComponentClass &c) override {
        const CharReach &cr = c.cr;
        const size_t width = cr.count();
        if (width == 1) {
            lit.push_back(cr.find_first(), false);
        } else if (width == 2 && cr.isCaselessChar()) {
            lit.push_back(cr.find_first(), true);
        } else {
            throw NotLiteral();
        }
    }

    void pre(const ComponentRepeat &c) override {
        if (c.m_min == 0 || c.m_min != c.m_max) {
            throw NotLiteral();
        }

        if (c.m_max < ComponentRepeat::NoLimit && c.m_max > 32767) {
            throw ParseError("Bounded repeat is too large.");
        }

        // Store the current length of the literal; in this repeat's post()
        // call we will append N-1 more copies of [index..end].
        repeat_stack.push(lit.length());
    }

    void post(const ComponentRepeat &c) override {
        // Add N-1 copies of the string between the entry to the repeat and the
        // current end of the literal.
        assert(!repeat_stack.empty());
        const ue2_literal suffix = lit.substr(repeat_stack.top());
        repeat_stack.pop();

        for (unsigned i = 1; i < c.m_min; i++) {
            lit += suffix;
        }
    }

    void pre(const ComponentSequence &) override {
        // Pass through.
    }

    void pre(const ComponentAlternation &) override { throw NotLiteral(); }
    void pre(const ComponentAssertion &) override { throw NotLiteral(); }
    void pre(const ComponentAtomicGroup &) override { throw NotLiteral(); }
    void pre(const ComponentBackReference &) override { throw NotLiteral(); }
    void pre(const ComponentBoundary &) override { throw NotLiteral(); }
    void pre(const ComponentByte &) override { throw NotLiteral(); }
    void pre(const ComponentCondReference &) override { throw NotLiteral(); }
    void pre(const ComponentEmpty &) override { throw NotLiteral(); }
    void pre(const ComponentEUS &) override { throw NotLiteral(); }
    void pre(const ComponentWordBoundary &) override { throw NotLiteral(); }
    void pre(const UTF8ComponentClass &) override { throw NotLiteral(); }

    void during(const AsciiComponentClass &) override {}
    void during(const ComponentAlternation &) override {}
    void during(const ComponentAssertion &) override {}
    void during(const ComponentAtomicGroup &) override {}
    void during(const ComponentBackReference &) override {}
    void during(const ComponentBoundary &) override {}
    void during(const ComponentByte &) override {}
    void during(const ComponentCondReference &) override {}
    void during(const ComponentEmpty &) override {}
    void during(const ComponentEUS &) override {}
    void during(const ComponentRepeat &) override {}
    void during(const ComponentSequence &) override {}
    void during(const ComponentWordBoundary &) override {}
    void during(const UTF8ComponentClass &) override {}

    void post(const AsciiComponentClass &) override {}
    void post(const ComponentAlternation &) override {}
    void post(const ComponentAssertion &) override {}
    void post(const ComponentAtomicGroup &) override {}
    void post(const ComponentBackReference &) override {}
    void post(const ComponentBoundary &) override {}
    void post(const ComponentByte &) override {}
    void post(const ComponentCondReference &) override {}
    void post(const ComponentEmpty &) override {}
    void post(const ComponentEUS &) override {}
    void post(const ComponentSequence &) override {}
    void post(const ComponentWordBoundary &) override {}
    void post(const UTF8ComponentClass &) override {}

    ue2_literal lit;
    stack<size_t> repeat_stack; //!< index of entry to repeat.
};

ConstructLiteralVisitor::~ConstructLiteralVisitor() {}

/** \brief True if the literal expression \a expr could be added to Rose. */
bool shortcutLiteral(NG &ng, const ParsedExpression &pe) {
    assert(pe.component);

    if (!ng.cc.grey.allowLiteral) {
        return false;
    }

    const auto &expr = pe.expr;

    // XXX: don't shortcut literals with extended params (yet)
    if (expr.min_offset || expr.max_offset != MAX_OFFSET || expr.min_length ||
        expr.edit_distance || expr.hamm_distance) {
        DEBUG_PRINTF("extended params not allowed\n");
        return false;
    }

    ConstructLiteralVisitor vis;
    try {
        assert(pe.component);
        pe.component->accept(vis);
        assert(vis.repeat_stack.empty());
    } catch (const ConstructLiteralVisitor::NotLiteral&) {
        DEBUG_PRINTF("not a literal\n");
        return false;
    }

    vis.lit.set_pure();
    const ue2_literal &lit = vis.lit;

    if (lit.empty()) {
        DEBUG_PRINTF("empty literal\n");
        return false;
    }

    if (expr.highlander && lit.length() <= 1) {
        DEBUG_PRINTF("not shortcutting SEP literal\n");
        return false;
    }

    DEBUG_PRINTF("constructed literal %s\n", dumpString(lit).c_str());
    return ng.addLiteral(lit, expr.index, expr.report, expr.highlander,
                         expr.som, expr.quiet);
}

} // namespace ue2
