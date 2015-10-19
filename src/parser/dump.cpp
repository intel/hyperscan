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

#include "config.h"

#include "dump.h"
#include "position.h"
#include "ConstComponentVisitor.h"
#include "ComponentBackReference.h"
#include "ComponentClass.h"
#include "ComponentCondReference.h"
#include "ComponentRepeat.h"
#include "ComponentAlternation.h"
#include "ComponentAssertion.h"
#include "ComponentAtomicGroup.h"
#include "ComponentBoundary.h"
#include "ComponentByte.h"
#include "ComponentEmpty.h"
#include "ComponentEUS.h"
#include "ComponentSequence.h"
#include "ComponentWordBoundary.h"
#include "Utf8ComponentClass.h"
#include "AsciiComponentClass.h"
#include "util/charreach.h"
#include "util/dump_charclass.h"

#include <ostream>
#include <string>

#ifndef DUMP_SUPPORT
#error No dump support!
#endif

using std::ostream;
using std::string;
using std::endl;

namespace ue2 {

class DumpVisitor : public ConstComponentVisitor {
private:
    void indent() { level++; }
    void outdent() {
        assert(level > 0);
        level--;
    }
    std::string filler() const { return string(level * 2, ' '); }

public:
    explicit DumpVisitor(ostream &s) : os(s), level(0) {}
    ~DumpVisitor() override;

    void pre(const AsciiComponentClass &c) override {
        os << filler() << "ASCII CLASS" << endl << filler() << "  ";
        describeClass(os, c.cr, 256, CC_OUT_TEXT);
        os << endl;
        indent();
    }

    void post(const AsciiComponentClass &) override { outdent(); }

    void pre(const ComponentAlternation &) override {
        os << filler() << "ALTERNATION" << endl;
        indent();
    }

    void post(const ComponentAlternation &) override {
        outdent();
    }

    void pre(const ComponentAssertion &c) override {
        os << filler() << "ASSERTION (";
        switch (c.m_sense) {
        case ComponentAssertion::POS:
            os << "POSITIVE ";
            break;
        case ComponentAssertion::NEG:
            os << "NEGATIVE ";
            break;
        }

        switch (c.m_dir) {
        case ComponentAssertion::LOOKAHEAD:
            os << "LOOKAHEAD";
            break;
        case ComponentAssertion::LOOKBEHIND:
            os << "LOOKBEHIND";
            break;
        }

        os << ")" << endl;
        indent();
    }

    void post(const ComponentAssertion &) override { outdent(); }

    void pre(const ComponentAtomicGroup &) override {
        os << filler() << "ATOMIC GROUP" << endl;
        indent();
    }

    void post(const ComponentAtomicGroup &) override { outdent(); }

    void pre(const ComponentBackReference &c) override {
        if (!c.name.empty()) {
            os << filler() << "BACKREF " << c.name << std::endl;
        } else {
            os << filler() << "BACKREF " << c.ref_id << std::endl;
        }
        indent();
    }

    void post(const ComponentBackReference &) override { outdent(); }

    void pre(const ComponentBoundary &c) override {
        os << filler() << "BOUNDARY" << endl << filler() << "  ";
        switch (c.m_bound) {
        case ComponentBoundary::BEGIN_STRING:
            os << "ComponentBoundary::BEGIN_STRING";
            break;
        case ComponentBoundary::END_STRING:
            os << "ComponentBoundary::END_STRING";
            break;
        case ComponentBoundary::END_STRING_OPTIONAL_LF:
            os << "ComponentBoundary::END_STRING_OPTIONAL_LF";
            break;
        case ComponentBoundary::BEGIN_LINE:
            os << "ComponentBoundary::BEGIN_LINE";
            break;
        case ComponentBoundary::END_LINE:
            os << "ComponentBoundary::END_LINE";
            break;
        }
        os << endl;
        indent();
    }

    void post(const ComponentBoundary &) override { outdent(); }

    void pre(const ComponentByte &) override {
        os << filler() << "BYTE" << endl;
        indent();
    }

    void post(const ComponentByte &) override { outdent(); }

    void pre(const ComponentCondReference &c) override {
        os << filler() << "CONDITIONAL REFERENCE" << endl;
        switch (c.kind) {
        case ComponentCondReference::CONDITION_NUMBER:
            os << filler() << "REFERENCES GROUP WITH NUMBER " << c.ref_id
               << endl;
            break;
        case ComponentCondReference::CONDITION_NAME:
            os << filler() << "REFERENCES GROUP WITH NAME " << c.ref_name
               << endl;
            break;
        case ComponentCondReference::CONDITION_ASSERTION:
            os << filler() << "REFERENCES FOLLOWING ASSERTION" << endl;
            break;
        }
        indent();
    }

    void post(const ComponentCondReference &) override { outdent(); }

    void pre(const ComponentEmpty &) override {
        os << filler() << "EMPTY" << endl;
        indent();
    }

    void post(const ComponentEmpty &) override { outdent(); }

    void pre(const ComponentEUS &) override {
        os << filler() << "EUS" << endl;
        indent();
    }

    void post(const ComponentEUS &) override { outdent(); }

    void pre(const ComponentRepeat &c) override {
        os << filler() << "REPEAT (" << c.m_min << ", ";
        if (c.m_max == ComponentRepeat::NoLimit) {
            os << "NoLimit";
        } else {
            os << c.m_max;
        }
        os << ") ";
        switch (c.type) {
            case ComponentRepeat::REPEAT_NONGREEDY:
                os << "non-greedy";
                break;
            case ComponentRepeat::REPEAT_GREEDY:
                os << "greedy";
                break;
            case ComponentRepeat::REPEAT_POSSESSIVE:
                os << "possessive";
                break;
        }
        os << endl;
        indent();
    }

    void post(const ComponentRepeat &) override { outdent(); }

    void pre(const ComponentSequence &c) override {
        os << filler() << "SEQUENCE ";
        if (c.capture_index == ComponentSequence::NOT_CAPTURED) {
            os << "(not captured) ";
        } else {
            os << "(capture index " << c.capture_index << ") ";
        }
        if (!c.capture_name.empty()) {
            os << "(capture name '" << c.capture_name << "')";
        }
        os << endl;
        indent();
        if (c.children.empty()) {
            os << filler() << " <empty>" << endl;
        }
    }

    void post(const ComponentSequence &) override { outdent(); }

    void pre(const ComponentWordBoundary &c) override {
        os << filler() << (c.negated ? "NON-WORD-BOUNDARY ('\\B')"
                                      : "WORD-BOUNDARY ('\\b')") << endl;
        indent();
    }

    void post(const ComponentWordBoundary &) override { outdent(); }

    void pre(const UTF8ComponentClass &c) override {
        os << filler() << "UTF8 CLASS" << endl << filler() << "  ";
        if (c.cps.none()) {
            os << "<none>";
        } else {
            for (auto it = c.cps.begin(), ite = c.cps.end(); it != ite; ++it) {
                os << std::hex << *it << " ";
            }
        }
        os << endl;

        indent();
    }

    void post(const UTF8ComponentClass &) override { outdent(); }

    // not used
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

private:
    ostream &os;
    unsigned level;
};

DumpVisitor::~DumpVisitor() {}

void dumpTree(ostream &os, const Component *const root) {
    assert(root);
    DumpVisitor vis(os);
    root->accept(vis);
}

} // namespace ue2
