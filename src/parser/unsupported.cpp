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
 * \brief Checks component trees for unsupported components.
  */
#include "ConstComponentVisitor.h"
#include "ComponentEUS.h"
#include "ComponentRepeat.h"
#include "ComponentWordBoundary.h"
#include "parse_error.h"
#include "unsupported.h"

#include <sstream>

namespace ue2 {

/** \brief Visitor class that throws a ParseError exception when it encounters
 * an unsupported component. */
class UnsupportedVisitor : public DefaultConstComponentVisitor {
public:
    ~UnsupportedVisitor() override;
    using DefaultConstComponentVisitor::pre;
    void pre(const ComponentAssertion &) override {
        throw ParseError("Zero-width assertions are not supported.");
    }
    void pre(const ComponentAtomicGroup &) override {
        throw ParseError("Atomic groups are unsupported.");
    }
    void pre(const ComponentBackReference &) override {
        throw ParseError("Back-references are unsupported.");
    }
    void pre(const ComponentCondReference &) override {
        throw ParseError("Conditional references are not supported.");
    }
    void pre(const ComponentEUS &c) override {
        std::ostringstream str;
        str << "\\X unsupported at index " << c.loc << ".";
        throw ParseError(str.str());
    }
    void pre(const ComponentRepeat &c) override {
        if (c.type == ComponentRepeat::REPEAT_POSSESSIVE) {
            throw ParseError("Possessive quantifiers are not supported.");
        }
    }
    void pre(const ComponentWordBoundary &c) override {
        if (c.ucp && !c.prefilter) {
            std::ostringstream str;
            str << (!c.negated ? "\\b" : "\\B")
                << " unsupported in UCP mode at index " << c.loc << ".";
            throw ParseError(str.str());
        }
    }
};

UnsupportedVisitor::~UnsupportedVisitor() {}

void checkUnsupported(const Component &root) {
    UnsupportedVisitor vis;
    root.accept(vis);
}

} // namespace ue2
