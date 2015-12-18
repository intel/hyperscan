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
 * \brief Character class in UTF-8 mode.
 */

#ifndef UTF8_COMPONENT_CLASS_H
#define UTF8_COMPONENT_CLASS_H

#include "ComponentClass.h"
#include "ue2common.h"
#include "util/unicode_set.h"

#include <map>
#include <set>
#include <string>
#include <vector>

namespace ue2 {

class UTF8ComponentClass : public ComponentClass {
    friend class DumpVisitor;
    friend class PrintVisitor;
    friend class CaselessVisitor;
    friend class SimplifyVisitor;
    friend class SimplifyCandidatesVisitor;
public:
    explicit UTF8ComponentClass(const ParseMode &mode);
     ~UTF8ComponentClass() override {}
    UTF8ComponentClass *clone() const override;

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

    bool class_empty(void) const override;
    void add(PredefinedClass c, bool negative) override;
    void add(unichar c) override;
    void finalize(void) override;
    void notePositions(GlushkovBuildState &bs) override;
    void buildFollowSet(GlushkovBuildState &bs,
                        const std::vector<PositionInfo> &) override;
    std::vector<PositionInfo> first(void) const override;
    std::vector<PositionInfo> last(void) const override;

protected:
    void createRange(unichar to) override;

private:
    Position getHead(NFABuilder &builder, u8 first_byte);
    void addToTail(GlushkovBuildState &bs, std::map<Position, Position> &finals,
                   Position prev, unichar b, unichar e);
    void ensureDotTrailer(GlushkovBuildState &bs);
    void ensureTwoDotTrailer(GlushkovBuildState &bs);
    void ensureThreeDotTrailer(GlushkovBuildState &bs);
    void buildOneByte(GlushkovBuildState &bs);
    void buildTwoByte(GlushkovBuildState &bs);
    void buildThreeByte(GlushkovBuildState &bs);
    void buildFourByte(GlushkovBuildState &bs);

    CodePointSet cps;

    std::map<u8, Position> heads;
    Position single_pos;
    Position one_dot_trailer;
    Position two_dot_trailer;
    Position three_dot_trailer;

    Position two_char_dot_head;
    Position three_char_dot_head;
    Position four_char_dot_head;
    std::set<Position> tails;
};

PredefinedClass translateForUcpMode(PredefinedClass in, const ParseMode &mode);

CodePointSet getPredefinedCodePointSet(PredefinedClass c,
                                       const ParseMode &mode);

} // namespace

#endif // UTF8_COMPONENT_CLASS_H
