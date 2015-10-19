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
 * \brief Back-references (/([a-f]{3}).*\\1/)
 */

#ifndef _RE_COMPONENTBACKREFERENCE_H_
#define _RE_COMPONENTBACKREFERENCE_H_

#include "Component.h"
#include <string>

namespace ue2 {

class ComponentBackReference : public Component {
    friend class DumpVisitor;
    friend class PrintVisitor;
    friend class ReferenceVisitor;
public:
    explicit ComponentBackReference(unsigned int id);
    explicit ComponentBackReference(const std::string &s);
    ~ComponentBackReference() override {}
    ComponentBackReference *clone() const override;

    Component *accept(ComponentVisitor &v) override {
        Component *c = v.visit(this);
        v.post(this);
        return c;
    }

    void accept(ConstComponentVisitor &v) const override {
        v.pre(*this);
        v.during(*this);
        v.post(*this);
    }

    unsigned int getRefID() const { return ref_id; }
    const std::string &getRefName() const { return name; }

    std::vector<PositionInfo> first() const override;
    std::vector<PositionInfo> last() const override;
    bool empty(void) const override;
    void notePositions(GlushkovBuildState &bs) override;
    void buildFollowSet(GlushkovBuildState &bs,
                        const std::vector<PositionInfo> &lastPos) override;

private:
    // Private copy ctor. Use clone instead.
    ComponentBackReference(const ComponentBackReference &other)
        : Component(other), name(other.name), ref_id(other.ref_id) {}

    std::string name;
    unsigned int ref_id;
};

} // namespace ue2

#endif
