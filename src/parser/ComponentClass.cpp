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
#include "Parser.h"
#include "ComponentClass.h"
#include "AsciiComponentClass.h"
#include "ucp_table.h"
#include "Utf8ComponentClass.h"
#include "util/charreach.h"
#include "util/make_unique.h"

#include <boost/icl/interval_set.hpp>

using namespace std;

namespace ue2 {

static
CharReach to_cr(const CodePointSet &cps) {
    CharReach cr;
    for (const auto &cp : cps) {
        if (lower(cp) >= CharReach::npos) {
            break;
        }

        cr.setRange(lower(cp), MIN(upper(cp), CharReach::npos - 1));
    }

    return cr;
}

CharReach getPredefinedCharReach(PredefinedClass c, const ParseMode &mode) {
    const CharReach lower('a', 'z');
    const CharReach upper('A', 'Z');
    const CharReach number('0', '9');
    switch (c) {
    case CLASS_ALNUM:
        return lower | upper | number;
    case CLASS_ALPHA:
        return lower | upper;
    case CLASS_ANY:
        if (mode.dotall) {
            return ~CharReach();
        } else {
            return ~CharReach('\n');
        }
    case CLASS_ASCII:
        return CharReach(0, 127);
    case CLASS_BLANK:
        return CharReach(" \t");
    case CLASS_CNTRL:
        return CharReach(0, 31) | CharReach(127 /* del */);
    case CLASS_DIGIT:
        return number;
    case CLASS_GRAPH:
        return CharReach(0x21, 0x7e);
    case CLASS_XGRAPH:
        return to_cr(getPredefinedCodePointSet(c, mode));
    case CLASS_HORZ:
        return CharReach("\x09\x20\xA0");
    case CLASS_LOWER:
        if (mode.caseless) {
            return lower | upper;
        } else {
            return lower;
        }
    case CLASS_PRINT:
        return CharReach(0x20, 0x7e);
    case CLASS_XPRINT:
        return to_cr(getPredefinedCodePointSet(c, mode));
    case CLASS_PUNCT:
        return CharReach(0x21, '0' - 1)
            | CharReach('9' + 1, 'A' - 1)
            | CharReach('Z' + 1, 'a' - 1)
            | CharReach('z' + 1, 126);
    case CLASS_XPUNCT:
        return to_cr(getPredefinedCodePointSet(c, mode));
    case CLASS_SPACE:
        return CharReach("\x09\x0a\x0c\x0b\x0d\x20");
    case CLASS_UPPER:
        if (mode.caseless) {
            return lower | upper;
        } else {
            return upper;
        }
    case CLASS_VERT:
        return CharReach("\x0a\x0b\x0c\x0d\x85");
    case CLASS_WORD:
        return lower | upper | number | CharReach('_');
    case CLASS_XDIGIT:
        return CharReach("0123456789abcdefABCDEF");
    case CLASS_UCP_C:
        return to_cr(getUcpC());
    case CLASS_UCP_CC:
        return to_cr(getUcpCc());
    case CLASS_UCP_CF:
        return to_cr(getUcpCf());
    case CLASS_UCP_CN:
        return to_cr(getUcpCn());
    case CLASS_UCP_CO:
        return to_cr(getUcpCo());
    case CLASS_UCP_CS:
        return to_cr(getUcpCs());
    case CLASS_UCP_L:
        return to_cr(getUcpL());
    case CLASS_UCP_L_AND:
        return to_cr(getUcpL_and());
    case CLASS_UCP_LL:
        return to_cr(getUcpLl());
    case CLASS_UCP_LM:
        return to_cr(getUcpLm());
    case CLASS_UCP_LO:
        return to_cr(getUcpLo());
    case CLASS_UCP_LT:
        return to_cr(getUcpLt());
    case CLASS_UCP_LU:
        return to_cr(getUcpLu());
    case CLASS_UCP_M:
        return to_cr(getUcpM());
    case CLASS_UCP_MC:
        return to_cr(getUcpMc());
    case CLASS_UCP_ME:
        return to_cr(getUcpMe());
    case CLASS_UCP_MN:
        return to_cr(getUcpMn());
    case CLASS_UCP_N:
        return to_cr(getUcpN());
    case CLASS_UCP_ND:
        return to_cr(getUcpNd());
    case CLASS_UCP_NL:
        return to_cr(getUcpNl());
    case CLASS_UCP_NO:
        return to_cr(getUcpNo());
    case CLASS_UCP_P:
        return to_cr(getUcpP());
    case CLASS_UCP_PC:
        return to_cr(getUcpPc());
    case CLASS_UCP_PD:
        return to_cr(getUcpPd());
    case CLASS_UCP_PE:
        return to_cr(getUcpPe());
    case CLASS_UCP_PF:
        return to_cr(getUcpPf());
    case CLASS_UCP_PI:
        return to_cr(getUcpPi());
    case CLASS_UCP_PO:
        return to_cr(getUcpPo());
    case CLASS_UCP_PS:
        return to_cr(getUcpPs());
    case CLASS_UCP_S:
        return to_cr(getUcpS());
    case CLASS_UCP_SC:
        return to_cr(getUcpSc());
    case CLASS_UCP_SK:
        return to_cr(getUcpSk());
    case CLASS_UCP_SM:
        return to_cr(getUcpSm());
    case CLASS_UCP_SO:
        return to_cr(getUcpSo());
    case CLASS_UCP_XAN:
        return to_cr(getUcpXan());
    case CLASS_UCP_XPS:
    case CLASS_UCP_XSP:
        return getPredefinedCharReach(CLASS_VERT, mode) | getPredefinedCharReach(CLASS_HORZ, mode);
    case CLASS_UCP_XWD:
        return to_cr(getUcpXwd());
    case CLASS_UCP_Z:
        return to_cr(getUcpZ());
    case CLASS_UCP_ZL:
        return to_cr(getUcpZl());
    case CLASS_UCP_ZP:
        return to_cr(getUcpZp());
    case CLASS_UCP_ZS:
        return to_cr(getUcpZs());
    case CLASS_SCRIPT_ARABIC:
        return to_cr(getUcpArabic());
    case CLASS_SCRIPT_ARMENIAN:
        return to_cr(getUcpArmenian());
    case CLASS_SCRIPT_AVESTAN:
        return to_cr(getUcpAvestan());
    case CLASS_SCRIPT_BALINESE:
        return to_cr(getUcpBalinese());
    case CLASS_SCRIPT_BAMUM:
        return to_cr(getUcpBamum());
    case CLASS_SCRIPT_BATAK:
        return to_cr(getUcpBatak());
    case CLASS_SCRIPT_BENGALI:
        return to_cr(getUcpBengali());
    case CLASS_SCRIPT_BOPOMOFO:
        return to_cr(getUcpBopomofo());
    case CLASS_SCRIPT_BRAHMI:
        return to_cr(getUcpBrahmi());
    case CLASS_SCRIPT_BRAILLE:
        return to_cr(getUcpBraille());
    case CLASS_SCRIPT_BUGINESE:
        return to_cr(getUcpBuginese());
    case CLASS_SCRIPT_BUHID:
        return to_cr(getUcpBuhid());
    case CLASS_SCRIPT_CANADIAN_ABORIGINAL:
        return to_cr(getUcpCanadian_Aboriginal());
    case CLASS_SCRIPT_CARIAN:
        return to_cr(getUcpCarian());
    case CLASS_SCRIPT_CHAM:
        return to_cr(getUcpCham());
    case CLASS_SCRIPT_CHEROKEE:
        return to_cr(getUcpCherokee());
    case CLASS_SCRIPT_COMMON:
        return to_cr(getUcpCommon());
    case CLASS_SCRIPT_COPTIC:
        return to_cr(getUcpCoptic());
    case CLASS_SCRIPT_CUNEIFORM:
        return to_cr(getUcpCuneiform());
    case CLASS_SCRIPT_CYPRIOT:
        return to_cr(getUcpCypriot());
    case CLASS_SCRIPT_CYRILLIC:
        return to_cr(getUcpCyrillic());
    case CLASS_SCRIPT_DESERET:
        return to_cr(getUcpDeseret());
    case CLASS_SCRIPT_DEVANAGARI:
        return to_cr(getUcpDevanagari());
    case CLASS_SCRIPT_EGYPTIAN_HIEROGLYPHS:
        return to_cr(getUcpEgyptian_Hieroglyphs());
    case CLASS_SCRIPT_ETHIOPIC:
        return to_cr(getUcpEthiopic());
    case CLASS_SCRIPT_GEORGIAN:
        return to_cr(getUcpGeorgian());
    case CLASS_SCRIPT_GLAGOLITIC:
        return to_cr(getUcpGlagolitic());
    case CLASS_SCRIPT_GOTHIC:
        return to_cr(getUcpGothic());
    case CLASS_SCRIPT_GREEK:
        return to_cr(getUcpGreek());
    case CLASS_SCRIPT_GUJARATI:
        return to_cr(getUcpGujarati());
    case CLASS_SCRIPT_GURMUKHI:
        return to_cr(getUcpGurmukhi());
    case CLASS_SCRIPT_HAN:
        return to_cr(getUcpHan());
    case CLASS_SCRIPT_HANGUL:
        return to_cr(getUcpHangul());
    case CLASS_SCRIPT_HANUNOO:
        return to_cr(getUcpHanunoo());
    case CLASS_SCRIPT_HEBREW:
        return to_cr(getUcpHebrew());
    case CLASS_SCRIPT_HIRAGANA:
        return to_cr(getUcpHiragana());
    case CLASS_SCRIPT_IMPERIAL_ARAMAIC:
        return to_cr(getUcpImperial_Aramaic());
    case CLASS_SCRIPT_INHERITED:
        return to_cr(getUcpInherited());
    case CLASS_SCRIPT_INSCRIPTIONAL_PAHLAVI:
        return to_cr(getUcpInscriptional_Pahlavi());
    case CLASS_SCRIPT_INSCRIPTIONAL_PARTHIAN:
        return to_cr(getUcpInscriptional_Parthian());
    case CLASS_SCRIPT_JAVANESE:
        return to_cr(getUcpJavanese());
    case CLASS_SCRIPT_KAITHI:
        return to_cr(getUcpKaithi());
    case CLASS_SCRIPT_KANNADA:
        return to_cr(getUcpKannada());
    case CLASS_SCRIPT_KATAKANA:
        return to_cr(getUcpKatakana());
    case CLASS_SCRIPT_KAYAH_LI:
        return to_cr(getUcpKayah_Li());
    case CLASS_SCRIPT_KHAROSHTHI:
        return to_cr(getUcpKharoshthi());
    case CLASS_SCRIPT_KHMER:
        return to_cr(getUcpKhmer());
    case CLASS_SCRIPT_LAO:
        return to_cr(getUcpLao());
    case CLASS_SCRIPT_LATIN:
        return to_cr(getUcpLatin());
    case CLASS_SCRIPT_LEPCHA:
        return to_cr(getUcpLepcha());
    case CLASS_SCRIPT_LIMBU:
        return to_cr(getUcpLimbu());
    case CLASS_SCRIPT_LINEAR_B:
        return to_cr(getUcpLinear_B());
    case CLASS_SCRIPT_LISU:
        return to_cr(getUcpLisu());
    case CLASS_SCRIPT_LYCIAN:
        return to_cr(getUcpLycian());
    case CLASS_SCRIPT_LYDIAN:
        return to_cr(getUcpLydian());
    case CLASS_SCRIPT_MALAYALAM:
        return to_cr(getUcpMalayalam());
    case CLASS_SCRIPT_MANDAIC:
        return to_cr(getUcpMandaic());
    case CLASS_SCRIPT_MEETEI_MAYEK:
        return to_cr(getUcpMeetei_Mayek());
    case CLASS_SCRIPT_MONGOLIAN:
        return to_cr(getUcpMongolian());
    case CLASS_SCRIPT_MYANMAR:
        return to_cr(getUcpMyanmar());
    case CLASS_SCRIPT_NEW_TAI_LUE:
        return to_cr(getUcpNew_Tai_Lue());
    case CLASS_SCRIPT_NKO:
        return to_cr(getUcpNko());
    case CLASS_SCRIPT_OGHAM:
        return to_cr(getUcpOgham());
    case CLASS_SCRIPT_OL_CHIKI:
        return to_cr(getUcpOl_Chiki());
    case CLASS_SCRIPT_OLD_ITALIC:
        return to_cr(getUcpOld_Italic());
    case CLASS_SCRIPT_OLD_PERSIAN:
        return to_cr(getUcpOld_Persian());
    case CLASS_SCRIPT_OLD_SOUTH_ARABIAN:
        return to_cr(getUcpOld_South_Arabian());
    case CLASS_SCRIPT_OLD_TURKIC:
        return to_cr(getUcpOld_Turkic());
    case CLASS_SCRIPT_ORIYA:
        return to_cr(getUcpOriya());
    case CLASS_SCRIPT_OSMANYA:
        return to_cr(getUcpOsmanya());
    case CLASS_SCRIPT_PHAGS_PA:
        return to_cr(getUcpPhags_Pa());
    case CLASS_SCRIPT_PHOENICIAN:
        return to_cr(getUcpPhoenician());
    case CLASS_SCRIPT_REJANG:
        return to_cr(getUcpRejang());
    case CLASS_SCRIPT_RUNIC:
        return to_cr(getUcpRunic());
    case CLASS_SCRIPT_SAMARITAN:
        return to_cr(getUcpSamaritan());
    case CLASS_SCRIPT_SAURASHTRA:
        return to_cr(getUcpSaurashtra());
    case CLASS_SCRIPT_SHAVIAN:
        return to_cr(getUcpShavian());
    case CLASS_SCRIPT_SINHALA:
        return to_cr(getUcpSinhala());
    case CLASS_SCRIPT_SUNDANESE:
        return to_cr(getUcpSundanese());
    case CLASS_SCRIPT_SYLOTI_NAGRI:
        return to_cr(getUcpSyloti_Nagri());
    case CLASS_SCRIPT_SYRIAC:
        return to_cr(getUcpSyriac());
    case CLASS_SCRIPT_TAGALOG:
        return to_cr(getUcpTagalog());
    case CLASS_SCRIPT_TAGBANWA:
        return to_cr(getUcpTagbanwa());
    case CLASS_SCRIPT_TAI_LE:
        return to_cr(getUcpTai_Le());
    case CLASS_SCRIPT_TAI_THAM:
        return to_cr(getUcpTai_Tham());
    case CLASS_SCRIPT_TAI_VIET:
        return to_cr(getUcpTai_Viet());
    case CLASS_SCRIPT_TAMIL:
        return to_cr(getUcpTamil());
    case CLASS_SCRIPT_TELUGU:
        return to_cr(getUcpTelugu());
    case CLASS_SCRIPT_THAANA:
        return to_cr(getUcpThaana());
    case CLASS_SCRIPT_THAI:
        return to_cr(getUcpThai());
    case CLASS_SCRIPT_TIBETAN:
        return to_cr(getUcpTibetan());
    case CLASS_SCRIPT_TIFINAGH:
        return to_cr(getUcpTifinagh());
    case CLASS_SCRIPT_UGARITIC:
        return to_cr(getUcpUgaritic());
    case CLASS_SCRIPT_VAI:
        return to_cr(getUcpVai());
    case CLASS_SCRIPT_YI:
        return to_cr(getUcpYi());
    case CLASS_UCP_ANY: /* always include newline */
        return ~CharReach();
    }
    assert(0);
    return CharReach();
}

unique_ptr<ComponentClass> getComponentClass(const ParseMode &mode) {
    if (mode.utf8) {
        return ue2::make_unique<UTF8ComponentClass>(mode);
    } else {
        return ue2::make_unique<AsciiComponentClass>(mode);
    }
}

unique_ptr<ComponentClass> generateComponent(PredefinedClass c, bool negate,
                                             const ParseMode &mode) {
    auto cc = getComponentClass(mode);
    cc->add(c, negate);
    cc->finalize();
    return cc;
}

unique_ptr<ComponentClass> getLiteralComponentClass(unsigned char c,
                                                    bool nocase) {
    ParseMode mode;
    mode.caseless = nocase;
    auto cc = getComponentClass(mode);
    cc->add(c);
    cc->finalize();
    return cc;
}

ComponentClass::ComponentClass(const ParseMode &mode_in)
    : m_negate(false), mode(mode_in), in_cand_range(false),
      range_start(INVALID_UNICODE), finalized(false) {}

ComponentClass::~ComponentClass() { }

void ComponentClass::addDash(void) {
    if (!in_cand_range) {
        // this could be the start of a range
        if (range_start != INVALID_UNICODE) {
            in_cand_range = true;
        } else {
            /* no possible start character for range, this is just a literal */
            add('-');
        }
    } else {
        // already creating a range, so this must be literal '-'
        in_cand_range = false;
        createRange('-');
    }
}

void ComponentClass::negate() {
    m_negate = true;
}

} // namespace ue2
