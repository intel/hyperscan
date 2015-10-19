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
 * \brief Boundary assertions (^, $, \\A, \\Z, \\z)
 */

#ifndef _RE_COMPONENTBOUNDARY_H_
#define _RE_COMPONENTBOUNDARY_H_

#include "Component.h"
#include "position.h"

namespace ue2 {

/** \brief Encapsulates a line/string boundary assertion. */
class ComponentBoundary : public Component {
    friend class DumpVisitor;
    friend class PrintVisitor;
    friend class UnsafeBoundsVisitor;
    friend class MultilineVisitor;
public:
    enum Boundary {
        BEGIN_STRING,           //!< beginning of data stream
        END_STRING,             //!< end of data stream
        END_STRING_OPTIONAL_LF, //!< end of data stream with an optional
                                //   linefeed
        BEGIN_LINE,             //!< '(^|\\n)'
        END_LINE                //!< '($|\\n)'
    };

    explicit ComponentBoundary(enum Boundary bound);
    ~ComponentBoundary() override;
    ComponentBoundary *clone() const override;

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
    bool checkEmbeddedStartAnchor(bool at_start) const override;
    bool checkEmbeddedEndAnchor(bool at_end) const override;

private:
    enum Boundary m_bound; //!< \brief which assertion is that?
    Position m_newline; //!< \brief special newline state
    std::vector<PositionInfo> m_first; //!< \brief positions returned for first()
    std::vector<PositionInfo> m_last; //!< \brief positions returned for last()

    ComponentBoundary(const ComponentBoundary &other);
};

} // namespace ue2

#endif
