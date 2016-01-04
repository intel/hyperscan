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
 * \brief Repeats ('*', '+', '?', '{M,N}', etc)
 */

#ifndef RE_COMPONENTREPEAT_H
#define RE_COMPONENTREPEAT_H

#include "Component.h"
#include "position.h"
#include "ue2common.h"

#include <memory>
#include <utility>

namespace ue2 {

/**
 * \brief Encapsulates a repeat of a subexpression ('*', '+', '?', '{M,N}',
 * etc).
 *
 * ASCII Art Time:
 *
 * Our standard representation of standard repeats. Other constructions (fan-in
 * vs fan-out) would also be possible and equivalent for our purposes.
 *
 * {n,m}
 *
 *     S->M->M->M->O->O->O->T
 *              |     ^  ^  ^
 *              |     |  |  |
 *              \-----------/
 *
 * {0,m}
 *
 *     /-----------\
 *     |           |
 *     |           V
 *     S->O->O->O->T
 *        |  ^  ^  ^
 *        |  |  |  |
 *        \--------/
 *
 */
class ComponentRepeat : public Component {
    friend class ConstructLiteralVisitor;
    friend class DumpVisitor;
    friend class PrintVisitor;
    friend class SimplifyVisitor;
public:
    /** \brief Value representing no maximum bound. */
    static constexpr u32 NoLimit = 0xffffffff;

    /** \brief Type of this repeat, characterising its
     * greediness/possessiveness. */
    enum RepeatType {
        /** Minimising repeat, like 'a*?'. */
        REPEAT_NONGREEDY,
        /** Maximising repeat, like 'a*'. This is the default in PCRE. */
        REPEAT_GREEDY,
        /** Possessive, maximising repeat, like 'a*+'. Possessive repeats are
         * only currently supported in prefiltering mode, where we treat them
         * the same way we treat normal greedy repeats. */
        REPEAT_POSSESSIVE,
    };

    ComponentRepeat(std::unique_ptr<Component> sub_comp, u32 min, u32 max,
                    RepeatType t);
    ~ComponentRepeat() override;
    ComponentRepeat *clone() const override;
    Component *accept(ComponentVisitor &v) override;
    void accept(ConstComponentVisitor &v) const override;

    std::vector<PositionInfo> first() const override;
    std::vector<PositionInfo> last() const override;
    bool empty() const override;
    bool repeatable() const override;
    bool vacuous_everywhere() const override;
    void notePositions(GlushkovBuildState &bs) override;
    void buildFollowSet(GlushkovBuildState &bs,
                        const std::vector<PositionInfo> &lastPos) override;
    bool checkEmbeddedStartAnchor(bool at_start) const override;
    bool checkEmbeddedEndAnchor(bool at_end) const override;

    void optimise(bool connected_to_sds) override;

    virtual std::pair<u32, u32> getBounds() const {
        return std::make_pair(m_min, m_max);
    }

    /** \brief From declared behaviour (not taking into account the
     * sub-component).  */
    enum RepeatType type;

protected:
    void postSubNotePositionHook();
    void wireRepeats(GlushkovBuildState &bs);

    std::unique_ptr<Component> sub_comp;
    u32 m_min;
    u32 m_max;

    std::vector<std::vector<PositionInfo> > m_firsts;
    std::vector<std::vector<PositionInfo> > m_lasts;
    Position posFirst;
    Position posLast;

    ComponentRepeat(const ComponentRepeat &other);
};

std::unique_ptr<ComponentRepeat>
makeComponentRepeat(std::unique_ptr<Component> sub_comp, u32 min, u32 max,
                    ComponentRepeat::RepeatType t);

} // namespace ue2

#endif // _RE_COMPONENTREPEAT_H_
