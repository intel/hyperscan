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

#ifndef COMPONENT_SEQUENCE_H
#define COMPONENT_SEQUENCE_H

#include "Component.h"
#include "ComponentRepeat.h" // for ComponentRepeat::RepeatType
#include "ue2common.h"

#include <memory>
#include <set>
#include <vector>

namespace ue2 {

class ComponentAlternation;
class GlushkovBuildState;

// Encapsulates a number of sub expressions to be applied sequentially
class ComponentSequence : public Component {
    friend class DumpVisitor;
    friend class PrintVisitor;
    friend class SimplifyVisitor;
public:
    /** \brief capture index representing a sequence that ISN'T capturing */
    static constexpr unsigned int NOT_CAPTURED = 65536;

    ComponentSequence();
    ~ComponentSequence() override;
    ComponentSequence *clone() const override;
    Component *accept(ComponentVisitor &v) override;
    void accept(ConstComponentVisitor &v) const override;

    bool addRepeat(u32 min, u32 max, ComponentRepeat::RepeatType type);

    // overridden by ComponentCondReference, which can only have 1 or 2
    // branches.
    virtual void addAlternation();

    virtual void finalize();

    void addComponent(std::unique_ptr<Component> comp);

    std::vector<PositionInfo> first() const override;
    std::vector<PositionInfo> last() const override;
    bool empty(void) const override;
    bool vacuous_everywhere() const override;
    void notePositions(GlushkovBuildState &bs) override;
    void buildFollowSet(GlushkovBuildState &bs,
                        const std::vector<PositionInfo> &lastPos) override;
    bool checkEmbeddedStartAnchor(bool at_start) const override;
    bool checkEmbeddedEndAnchor(bool at_end) const override;

    void optimise(bool connected_to_sds) override;

    void setCaptureIndex(unsigned int idx) { capture_index = idx; }
    unsigned int getCaptureIndex() const { return capture_index; }
    void setCaptureName(const std::string &s) { capture_name = s; }
    const std::string &getCaptureName() const { return capture_name; }

    virtual const std::vector<std::unique_ptr<Component>> &getChildren() const {
        return children;
    }

protected:
    ComponentSequence(const ComponentSequence &other);

    std::vector<std::unique_ptr<Component>> children;
    std::unique_ptr<ComponentAlternation> alternation;

private:
    unsigned int capture_index;
    std::string capture_name; //!< empty means no name
};

} // namespace ue2

#endif
