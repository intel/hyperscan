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
 * \brief Extended Unicode sequences (\\X)
 */

#ifndef _RE_COMPONENTEXTENDEDUNICODESEQUENCE_H_
#define _RE_COMPONENTEXTENDEDUNICODESEQUENCE_H_

#include "Component.h"

namespace ue2 {

struct ParseMode;

class ComponentEUS : public Component {
    friend class DumpVisitor;
    friend class UnsupportedVisitor;
public:
    ComponentEUS(u32 loc, const ParseMode &mode);
    ~ComponentEUS() override;
    ComponentEUS *clone() const override;

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

    std::vector<PositionInfo> first() const override;
    std::vector<PositionInfo> last() const override;

    bool empty() const override { return false; }

    void notePositions(GlushkovBuildState &bs) override;
    void buildFollowSet(GlushkovBuildState &,
                        const std::vector<PositionInfo> &) override {
        // all follow set construction is handled by firsts/lasts
        return;
    }

private:
    u32 loc;
    bool utf8;
    Position position;

    ComponentEUS(const ComponentEUS &other)
        : Component(other), loc(other.loc), utf8(other.utf8),
          position(other.position) {}
};

} // namespace ue2

#endif
