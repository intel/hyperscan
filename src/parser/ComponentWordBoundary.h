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
 * \brief Word Boundary Assertion (\\b or \\B)
 */

#ifndef _RE_COMPONENTWORDBOUNDARY_H_
#define _RE_COMPONENTWORDBOUNDARY_H_

#include "Component.h"
#include "position.h"

namespace ue2 {

struct ParseMode;

/** \brief Encapsulates a positive (\\b) or negative (\\B) word boundary
 * assertion. */
class ComponentWordBoundary : public Component {
    friend class DumpVisitor;
    friend class PrintVisitor;
    friend class UnsupportedVisitor;
public:
    ComponentWordBoundary(u32 loc, bool negated, const ParseMode &mode);
    ~ComponentWordBoundary() override;
    ComponentWordBoundary *clone() const override;

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
    bool empty() const override;
    bool repeatable() const override;
    void notePositions(GlushkovBuildState &bs) override;
    void buildFollowSet(GlushkovBuildState &bs,
                        const std::vector<PositionInfo> &lastPos) override;

    void setPrefilter(bool p) { prefilter = p; }

private:
    u32 loc; //!< location in pattern for error reporting.
    Position position;
    bool negated;
    bool ucp;
    bool prefilter; //!< set by PrefilterVisitor, this is ugly

    ComponentWordBoundary(const ComponentWordBoundary &other)
        : Component(other), loc(other.loc), position(other.position),
          negated(other.negated), ucp(other.ucp), prefilter(other.prefilter) {}
};

} // namespace ue2

#endif
