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
 * \brief Character classes and their mnemonics.
 */

#ifndef COMPONENTCLASS_H
#define COMPONENTCLASS_H

#include <string>
#include <vector>
#include <utility>

#include "Component.h"
#include "Parser.h"
#include "util/charreach.h"
#include "util/unicode_def.h"
#include "ue2common.h"

namespace ue2 {

enum PredefinedClass {
    CLASS_ALNUM,
    CLASS_ALPHA,
    CLASS_ANY, /* dot, not quite any when not in dotall mode */
    CLASS_ASCII,
    CLASS_BLANK,
    CLASS_CNTRL,
    CLASS_DIGIT,
    CLASS_GRAPH,
    CLASS_HORZ,
    CLASS_LOWER,
    CLASS_PRINT,
    CLASS_PUNCT,
    CLASS_SPACE, /* has vertical tab */
    CLASS_UPPER,
    CLASS_VERT,
    CLASS_WORD,
    CLASS_XDIGIT,
    CLASS_XGRAPH, /* [:graph:] in UCP mode */
    CLASS_XPRINT, /* [:print:] in UCP mode */
    CLASS_XPUNCT, /* [:punct:] in UCP mode */
    CLASS_UCP_C,
    CLASS_UCP_CC,
    CLASS_UCP_CF,
    CLASS_UCP_CN, /* unallocated code points */
    CLASS_UCP_CO,
    CLASS_UCP_CS, /* does not contain valid unicode codepoints */
    CLASS_UCP_L,
    CLASS_UCP_LL,
    CLASS_UCP_LM,
    CLASS_UCP_LO,
    CLASS_UCP_LT,
    CLASS_UCP_LU,
    CLASS_UCP_L_AND, /* L& = LL+LU+LT */
    CLASS_UCP_M,
    CLASS_UCP_MC,
    CLASS_UCP_ME,
    CLASS_UCP_MN,
    CLASS_UCP_N,
    CLASS_UCP_ND,
    CLASS_UCP_NL,
    CLASS_UCP_NO,
    CLASS_UCP_P,
    CLASS_UCP_PC,
    CLASS_UCP_PD,
    CLASS_UCP_PE,
    CLASS_UCP_PF,
    CLASS_UCP_PI,
    CLASS_UCP_PO,
    CLASS_UCP_PS,
    CLASS_UCP_S,
    CLASS_UCP_SC,
    CLASS_UCP_SK,
    CLASS_UCP_SM,
    CLASS_UCP_SO,
    CLASS_UCP_Z,
    CLASS_UCP_ZL,
    CLASS_UCP_ZP,
    CLASS_UCP_ZS,
    CLASS_UCP_XAN,
    CLASS_UCP_XPS, /* CLASS_SPACE */
    CLASS_UCP_XSP,
    CLASS_UCP_XWD,
    CLASS_SCRIPT_ARABIC,
    CLASS_SCRIPT_ARMENIAN,
    CLASS_SCRIPT_AVESTAN,
    CLASS_SCRIPT_BALINESE,
    CLASS_SCRIPT_BAMUM,
    CLASS_SCRIPT_BATAK,
    CLASS_SCRIPT_BENGALI,
    CLASS_SCRIPT_BOPOMOFO,
    CLASS_SCRIPT_BRAHMI,
    CLASS_SCRIPT_BRAILLE,
    CLASS_SCRIPT_BUGINESE,
    CLASS_SCRIPT_BUHID,
    CLASS_SCRIPT_CANADIAN_ABORIGINAL,
    CLASS_SCRIPT_CARIAN,
    CLASS_SCRIPT_CHAM,
    CLASS_SCRIPT_CHEROKEE,
    CLASS_SCRIPT_COMMON,
    CLASS_SCRIPT_COPTIC,
    CLASS_SCRIPT_CUNEIFORM,
    CLASS_SCRIPT_CYPRIOT,
    CLASS_SCRIPT_CYRILLIC,
    CLASS_SCRIPT_DESERET,
    CLASS_SCRIPT_DEVANAGARI,
    CLASS_SCRIPT_EGYPTIAN_HIEROGLYPHS,
    CLASS_SCRIPT_ETHIOPIC,
    CLASS_SCRIPT_GEORGIAN,
    CLASS_SCRIPT_GLAGOLITIC,
    CLASS_SCRIPT_GOTHIC,
    CLASS_SCRIPT_GREEK,
    CLASS_SCRIPT_GUJARATI,
    CLASS_SCRIPT_GURMUKHI,
    CLASS_SCRIPT_HAN,
    CLASS_SCRIPT_HANGUL,
    CLASS_SCRIPT_HANUNOO,
    CLASS_SCRIPT_HEBREW,
    CLASS_SCRIPT_HIRAGANA,
    CLASS_SCRIPT_IMPERIAL_ARAMAIC,
    CLASS_SCRIPT_INHERITED,
    CLASS_SCRIPT_INSCRIPTIONAL_PAHLAVI,
    CLASS_SCRIPT_INSCRIPTIONAL_PARTHIAN,
    CLASS_SCRIPT_JAVANESE,
    CLASS_SCRIPT_KAITHI,
    CLASS_SCRIPT_KANNADA,
    CLASS_SCRIPT_KATAKANA,
    CLASS_SCRIPT_KAYAH_LI,
    CLASS_SCRIPT_KHAROSHTHI,
    CLASS_SCRIPT_KHMER,
    CLASS_SCRIPT_LAO,
    CLASS_SCRIPT_LATIN,
    CLASS_SCRIPT_LEPCHA,
    CLASS_SCRIPT_LIMBU,
    CLASS_SCRIPT_LINEAR_B,
    CLASS_SCRIPT_LISU,
    CLASS_SCRIPT_LYCIAN,
    CLASS_SCRIPT_LYDIAN,
    CLASS_SCRIPT_MALAYALAM,
    CLASS_SCRIPT_MANDAIC,
    CLASS_SCRIPT_MEETEI_MAYEK,
    CLASS_SCRIPT_MONGOLIAN,
    CLASS_SCRIPT_MYANMAR,
    CLASS_SCRIPT_NEW_TAI_LUE,
    CLASS_SCRIPT_NKO,
    CLASS_SCRIPT_OGHAM,
    CLASS_SCRIPT_OL_CHIKI,
    CLASS_SCRIPT_OLD_ITALIC,
    CLASS_SCRIPT_OLD_PERSIAN,
    CLASS_SCRIPT_OLD_SOUTH_ARABIAN,
    CLASS_SCRIPT_OLD_TURKIC,
    CLASS_SCRIPT_ORIYA,
    CLASS_SCRIPT_OSMANYA,
    CLASS_SCRIPT_PHAGS_PA,
    CLASS_SCRIPT_PHOENICIAN,
    CLASS_SCRIPT_REJANG,
    CLASS_SCRIPT_RUNIC,
    CLASS_SCRIPT_SAMARITAN,
    CLASS_SCRIPT_SAURASHTRA,
    CLASS_SCRIPT_SHAVIAN,
    CLASS_SCRIPT_SINHALA,
    CLASS_SCRIPT_SUNDANESE,
    CLASS_SCRIPT_SYLOTI_NAGRI,
    CLASS_SCRIPT_SYRIAC,
    CLASS_SCRIPT_TAGALOG,
    CLASS_SCRIPT_TAGBANWA,
    CLASS_SCRIPT_TAI_LE,
    CLASS_SCRIPT_TAI_THAM,
    CLASS_SCRIPT_TAI_VIET,
    CLASS_SCRIPT_TAMIL,
    CLASS_SCRIPT_TELUGU,
    CLASS_SCRIPT_THAANA,
    CLASS_SCRIPT_THAI,
    CLASS_SCRIPT_TIBETAN,
    CLASS_SCRIPT_TIFINAGH,
    CLASS_SCRIPT_UGARITIC,
    CLASS_SCRIPT_VAI,
    CLASS_SCRIPT_YI,
    CLASS_UCP_ANY
};

CharReach getPredefinedCharReach(PredefinedClass c, const ParseMode &mode);

class ComponentClass;
class NFABuilder;

/* Caller is responsible for lifecycle management, class finalized */
std::unique_ptr<ComponentClass>
generateComponent(PredefinedClass c, bool negated, const ParseMode &mode);

/* Caller is responsible for lifecycle management, class open */
std::unique_ptr<ComponentClass> getComponentClass(const ParseMode &mode);

/** Common case: generate a component for a single literal character, possibly
 * in caseless mode. Caller is responsible for lifecycle management. */
std::unique_ptr<ComponentClass> getLiteralComponentClass(unsigned char c,
                                                         bool nocase);

class ComponentClass : public Component {
    friend class DumpVisitor;
protected:
    explicit ComponentClass(const ParseMode &mode_in);
public:
    ~ComponentClass() override;
    ComponentClass *clone() const override = 0;

    Component *accept(ComponentVisitor &v) override = 0;
    void accept(ConstComponentVisitor &v) const override = 0;

    /** \brief True if the class contains no members (i.e. it will not match
     * against anything). This function can only be called on a finalized
     * class.
     *
     * Note: This is a different concept to Component::empty.
     */
    virtual bool class_empty(void) const = 0;

    virtual void add(PredefinedClass c, bool negated) = 0;
    virtual void add(unichar c) = 0; /* may throw LocatedParseError */
    void addDash(void);

    void negate(void);
    virtual void finalize(void) = 0;

    bool isNegated() const { return m_negate; }

    std::vector<PositionInfo> first() const override = 0;
    std::vector<PositionInfo> last() const override = 0;
    bool empty() const override { return false; } /* always 1 codepoint wide */

    void notePositions(GlushkovBuildState &bs) override = 0;
    void buildFollowSet(GlushkovBuildState &bs,
                        const std::vector<PositionInfo> &) override = 0;

protected:
    bool m_negate;
    const ParseMode mode;
    bool in_cand_range;
    unichar range_start;
    bool finalized;

    virtual void createRange(unichar) = 0;

    // Protected copy ctor. Use clone instead.
    ComponentClass(const ComponentClass &other)
        : Component(other), m_negate(other.m_negate), mode(other.mode),
          in_cand_range(other.in_cand_range), range_start(other.range_start),
          finalized(other.finalized) {}
};

} // namespace ue2

#endif // COMPONENTCLASS_H
