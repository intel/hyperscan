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


#include "Utf8ComponentClass.h"

#include "buildstate.h"
#include "Parser.h"
#include "parse_error.h"
#include "position.h"
#include "position_info.h"
#include "nfagraph/ng_builder.h"
#include "util/compare.h"
#include "util/unicode_def.h"

#include <cstring>

#include "ucp_table.h"

using namespace std;

namespace ue2 {

PredefinedClass translateForUcpMode(PredefinedClass in, const ParseMode &mode) {
    /* Note: the mapping used here for mapping posix character classes
     * matches the observed behaviour of PCRE (lower and upper going to \p{L}
     * is not documented by pcre).
     *
     * Note: this mapping is quite different from both of the mappings
     * recommended in the unicode regex tech report (TR-18) appendix C
     */
    switch (in) {
    case CLASS_ALNUM:
        return CLASS_UCP_XAN;
    case CLASS_ALPHA:
        return CLASS_UCP_L;
    case CLASS_BLANK:
        return CLASS_HORZ;
    case CLASS_DIGIT:
        return CLASS_UCP_ND;
    case CLASS_GRAPH:
        return CLASS_XGRAPH;
    case CLASS_LOWER:
        if (mode.caseless) { /* we also pick up uppercase titlecase and others */
            return CLASS_UCP_L;
        } else {
            return CLASS_UCP_LL;
        }
    case CLASS_PRINT:
        return CLASS_XPRINT;
    case CLASS_PUNCT:
        return CLASS_XPUNCT;
    case CLASS_SPACE:
        return CLASS_UCP_XPS;
    case CLASS_UPPER:
        if (mode.caseless) { /* we also pick up lowercase titlecase and others */
            return CLASS_UCP_L;
        } else {
            return CLASS_UCP_LU;
        }
    case CLASS_WORD:
        return CLASS_UCP_XWD;
    default:
        return in;
    }
}

CodePointSet getPredefinedCodePointSet(PredefinedClass c,
                                       const ParseMode &mode) {
    /* TODO: support properly PCRE_UCP mode and non PCRE_UCP mode */
    switch (c) {
    case CLASS_ANY:
        if (mode.dotall) {
            return CodePointSet(CodePointSet::interval(0, MAX_UNICODE));
        } else {
            CodePointSet rv;
            rv.set('\n');
            rv.flip();
            return rv;
        }
    case CLASS_XGRAPH: {
        CodePointSet rv;
        rv = getUcpZ();
        rv |= getUcpC();
        rv.flip();
        // most of Cf, except for ...
        CodePointSet cf = getUcpCf();
        cf.unset(0x061c);
        cf.unset(0x180e);
        cf.unsetRange(0x2066, 0x2069);
        rv |= cf;
        return rv;
    }
    case CLASS_XPRINT: {
        // Same as graph, plus everything with the Zs property.
        CodePointSet rv = getPredefinedCodePointSet(CLASS_XGRAPH, mode);
        rv |= getUcpZs();
        rv.set(0x180e); // Also included in this class by PCRE 8.38.
        return rv;
    }
    case CLASS_XPUNCT: {
        // Everything with the P (punctuation) property, plus code points in S
        // (symbols) that are < 128.
        CodePointSet rv = getUcpP();
        CodePointSet symbols = getUcpS();
        symbols.unsetRange(128, MAX_UNICODE);
        rv |= symbols;
        return rv;
    }
    case CLASS_HORZ: {
        CodePointSet rv;
        rv.set(0x0009); /* Horizontal tab */
        rv.set(0x0020); /* Space */
        rv.set(0x00A0); /* Non-break space */
        rv.set(0x1680); /* Ogham space mark */
        rv.set(0x180E); /* Mongolian vowel separator */
        rv.set(0x2000); /* En quad */
        rv.set(0x2001); /* Em quad */
        rv.set(0x2002); /* En space */
        rv.set(0x2003); /* Em space */
        rv.set(0x2004); /* Three-per-em space */
        rv.set(0x2005); /* Four-per-em space */
        rv.set(0x2006); /* Six-per-em space */
        rv.set(0x2007); /* Figure space */
        rv.set(0x2008); /* Punctuation space */
        rv.set(0x2009); /* Thin space */
        rv.set(0x200A); /* Hair space */
        rv.set(0x202F); /* Narrow no-break space */
        rv.set(0x205F); /* Medium mathematical space */
        rv.set(0x3000); /* Ideographic space */
        return rv;
    }
    case CLASS_VERT: {
        CodePointSet rv;
        rv.set(0x000A); /* Linefeed */
        rv.set(0x000B); /* Vertical tab */
        rv.set(0x000C); /* Formfeed */
        rv.set(0x000D); /* Carriage return */
        rv.set(0x0085); /* Next line */
        rv.set(0x2028); /* Line separator */
        rv.set(0x2029); /* Paragraph separator */
        return rv;
    }
    case CLASS_UCP_XPS:
    case CLASS_UCP_XSP: {
        CodePointSet rv;
        rv.set(0x0009); /* Horizontal tab */
        rv.set(0x0020); /* Space */
        rv.set(0x00A0); /* Non-break space */
        rv.set(0x1680); /* Ogham space mark */
        rv.set(0x180E); /* Mongolian vowel separator */
        rv.set(0x2000); /* En quad */
        rv.set(0x2001); /* Em quad */
        rv.set(0x2002); /* En space */
        rv.set(0x2003); /* Em space */
        rv.set(0x2004); /* Three-per-em space */
        rv.set(0x2005); /* Four-per-em space */
        rv.set(0x2006); /* Six-per-em space */
        rv.set(0x2007); /* Figure space */
        rv.set(0x2008); /* Punctuation space */
        rv.set(0x2009); /* Thin space */
        rv.set(0x200A); /* Hair space */
        rv.set(0x202F); /* Narrow no-break space */
        rv.set(0x205F); /* Medium mathematical space */
        rv.set(0x3000); /* Ideographic space */
        rv.set(0x000A); /* Linefeed */
        rv.set(0x000B); /* Vertical tab */
        rv.set(0x000C); /* Formfeed */
        rv.set(0x000D); /* Carriage return */
        rv.set(0x0085); /* Next line */
        rv.set(0x2028); /* Line separator */
        rv.set(0x2029); /* Paragraph separator */
        return rv;
    }
    case CLASS_UCP_C:
        return getUcpC();
    case CLASS_UCP_CC:
        return getUcpCc();
    case CLASS_UCP_CF:
        return getUcpCf();
    case CLASS_UCP_CN:
        return getUcpCn();
    case CLASS_UCP_CO:
        return getUcpCo();
    case CLASS_UCP_CS:
        return getUcpCs();
    case CLASS_UCP_L:
        return getUcpL();
    case CLASS_UCP_L_AND:
        return getUcpL_and();
    case CLASS_UCP_LL:
        return getUcpLl();
    case CLASS_UCP_LM:
        return getUcpLm();
    case CLASS_UCP_LO:
        return getUcpLo();
    case CLASS_UCP_LT:
        return getUcpLt();
    case CLASS_UCP_LU:
        return getUcpLu();
    case CLASS_UCP_M:
        return getUcpM();
    case CLASS_UCP_MC:
        return getUcpMc();
    case CLASS_UCP_ME:
        return getUcpMe();
    case CLASS_UCP_MN:
        return getUcpMn();
    case CLASS_UCP_N:
        return getUcpN();
    case CLASS_UCP_ND:
        return getUcpNd();
    case CLASS_UCP_NL:
        return getUcpNl();
    case CLASS_UCP_NO:
        return getUcpNo();
    case CLASS_UCP_P:
        return getUcpP();
    case CLASS_UCP_PC:
        return getUcpPc();
    case CLASS_UCP_PD:
        return getUcpPd();
    case CLASS_UCP_PE:
        return getUcpPe();
    case CLASS_UCP_PF:
        return getUcpPf();
    case CLASS_UCP_PI:
        return getUcpPi();
    case CLASS_UCP_PO:
        return getUcpPo();
    case CLASS_UCP_PS:
        return getUcpPs();
    case CLASS_UCP_S:
        return getUcpS();
    case CLASS_UCP_SC:
        return getUcpSc();
    case CLASS_UCP_SK:
        return getUcpSk();
    case CLASS_UCP_SM:
        return getUcpSm();
    case CLASS_UCP_SO:
        return getUcpSo();
    case CLASS_UCP_XAN:
        return getUcpXan();
    case CLASS_UCP_XWD:
        return getUcpXwd();
    case CLASS_UCP_Z:
        return getUcpZ();
    case CLASS_UCP_ZL:
        return getUcpZl();
    case CLASS_UCP_ZP:
        return getUcpZp();
    case CLASS_UCP_ZS:
        return getUcpZs();
    case CLASS_SCRIPT_ARABIC:
        return getUcpArabic();
    case CLASS_SCRIPT_ARMENIAN:
        return getUcpArmenian();
    case CLASS_SCRIPT_AVESTAN:
        return getUcpAvestan();
    case CLASS_SCRIPT_BALINESE:
        return getUcpBalinese();
    case CLASS_SCRIPT_BAMUM:
        return getUcpBamum();
    case CLASS_SCRIPT_BATAK:
        return getUcpBatak();
    case CLASS_SCRIPT_BENGALI:
        return getUcpBengali();
    case CLASS_SCRIPT_BOPOMOFO:
        return getUcpBopomofo();
    case CLASS_SCRIPT_BRAHMI:
        return getUcpBrahmi();
    case CLASS_SCRIPT_BRAILLE:
        return getUcpBraille();
    case CLASS_SCRIPT_BUGINESE:
        return getUcpBuginese();
    case CLASS_SCRIPT_BUHID:
        return getUcpBuhid();
    case CLASS_SCRIPT_CANADIAN_ABORIGINAL:
        return getUcpCanadian_Aboriginal();
    case CLASS_SCRIPT_CARIAN:
        return getUcpCarian();
    case CLASS_SCRIPT_CHAM:
        return getUcpCham();
    case CLASS_SCRIPT_CHEROKEE:
        return getUcpCherokee();
    case CLASS_SCRIPT_COMMON:
        return getUcpCommon();
    case CLASS_SCRIPT_COPTIC:
        return getUcpCoptic();
    case CLASS_SCRIPT_CUNEIFORM:
        return getUcpCuneiform();
    case CLASS_SCRIPT_CYPRIOT:
        return getUcpCypriot();
    case CLASS_SCRIPT_CYRILLIC:
        return getUcpCyrillic();
    case CLASS_SCRIPT_DESERET:
        return getUcpDeseret();
    case CLASS_SCRIPT_DEVANAGARI:
        return getUcpDevanagari();
    case CLASS_SCRIPT_EGYPTIAN_HIEROGLYPHS:
        return getUcpEgyptian_Hieroglyphs();
    case CLASS_SCRIPT_ETHIOPIC:
        return getUcpEthiopic();
    case CLASS_SCRIPT_GEORGIAN:
        return getUcpGeorgian();
    case CLASS_SCRIPT_GLAGOLITIC:
        return getUcpGlagolitic();
    case CLASS_SCRIPT_GOTHIC:
        return getUcpGothic();
    case CLASS_SCRIPT_GREEK:
        return getUcpGreek();
    case CLASS_SCRIPT_GUJARATI:
        return getUcpGujarati();
    case CLASS_SCRIPT_GURMUKHI:
        return getUcpGurmukhi();
    case CLASS_SCRIPT_HAN:
        return getUcpHan();
    case CLASS_SCRIPT_HANGUL:
        return getUcpHangul();
    case CLASS_SCRIPT_HANUNOO:
        return getUcpHanunoo();
    case CLASS_SCRIPT_HEBREW:
        return getUcpHebrew();
    case CLASS_SCRIPT_HIRAGANA:
        return getUcpHiragana();
    case CLASS_SCRIPT_IMPERIAL_ARAMAIC:
        return getUcpImperial_Aramaic();
    case CLASS_SCRIPT_INHERITED:
        return getUcpInherited();
    case CLASS_SCRIPT_INSCRIPTIONAL_PAHLAVI:
        return getUcpInscriptional_Pahlavi();
    case CLASS_SCRIPT_INSCRIPTIONAL_PARTHIAN:
        return getUcpInscriptional_Parthian();
    case CLASS_SCRIPT_JAVANESE:
        return getUcpJavanese();
    case CLASS_SCRIPT_KAITHI:
        return getUcpKaithi();
    case CLASS_SCRIPT_KANNADA:
        return getUcpKannada();
    case CLASS_SCRIPT_KATAKANA:
        return getUcpKatakana();
    case CLASS_SCRIPT_KAYAH_LI:
        return getUcpKayah_Li();
    case CLASS_SCRIPT_KHAROSHTHI:
        return getUcpKharoshthi();
    case CLASS_SCRIPT_KHMER:
        return getUcpKhmer();
    case CLASS_SCRIPT_LAO:
        return getUcpLao();
    case CLASS_SCRIPT_LATIN:
        return getUcpLatin();
    case CLASS_SCRIPT_LEPCHA:
        return getUcpLepcha();
    case CLASS_SCRIPT_LIMBU:
        return getUcpLimbu();
    case CLASS_SCRIPT_LINEAR_B:
        return getUcpLinear_B();
    case CLASS_SCRIPT_LISU:
        return getUcpLisu();
    case CLASS_SCRIPT_LYCIAN:
        return getUcpLycian();
    case CLASS_SCRIPT_LYDIAN:
        return getUcpLydian();
    case CLASS_SCRIPT_MALAYALAM:
        return getUcpMalayalam();
    case CLASS_SCRIPT_MANDAIC:
        return getUcpMandaic();
    case CLASS_SCRIPT_MEETEI_MAYEK:
        return getUcpMeetei_Mayek();
    case CLASS_SCRIPT_MONGOLIAN:
        return getUcpMongolian();
    case CLASS_SCRIPT_MYANMAR:
        return getUcpMyanmar();
    case CLASS_SCRIPT_NEW_TAI_LUE:
        return getUcpNew_Tai_Lue();
    case CLASS_SCRIPT_NKO:
        return getUcpNko();
    case CLASS_SCRIPT_OGHAM:
        return getUcpOgham();
    case CLASS_SCRIPT_OL_CHIKI:
        return getUcpOl_Chiki();
    case CLASS_SCRIPT_OLD_ITALIC:
        return getUcpOld_Italic();
    case CLASS_SCRIPT_OLD_PERSIAN:
        return getUcpOld_Persian();
    case CLASS_SCRIPT_OLD_SOUTH_ARABIAN:
        return getUcpOld_South_Arabian();
    case CLASS_SCRIPT_OLD_TURKIC:
        return getUcpOld_Turkic();
    case CLASS_SCRIPT_ORIYA:
        return getUcpOriya();
    case CLASS_SCRIPT_OSMANYA:
        return getUcpOsmanya();
    case CLASS_SCRIPT_PHAGS_PA:
        return getUcpPhags_Pa();
    case CLASS_SCRIPT_PHOENICIAN:
        return getUcpPhoenician();
    case CLASS_SCRIPT_REJANG:
        return getUcpRejang();
    case CLASS_SCRIPT_RUNIC:
        return getUcpRunic();
    case CLASS_SCRIPT_SAMARITAN:
        return getUcpSamaritan();
    case CLASS_SCRIPT_SAURASHTRA:
        return getUcpSaurashtra();
    case CLASS_SCRIPT_SHAVIAN:
        return getUcpShavian();
    case CLASS_SCRIPT_SINHALA:
        return getUcpSinhala();
    case CLASS_SCRIPT_SUNDANESE:
        return getUcpSundanese();
    case CLASS_SCRIPT_SYLOTI_NAGRI:
        return getUcpSyloti_Nagri();
    case CLASS_SCRIPT_SYRIAC:
        return getUcpSyriac();
    case CLASS_SCRIPT_TAGALOG:
        return getUcpTagalog();
    case CLASS_SCRIPT_TAGBANWA:
        return getUcpTagbanwa();
    case CLASS_SCRIPT_TAI_LE:
        return getUcpTai_Le();
    case CLASS_SCRIPT_TAI_THAM:
        return getUcpTai_Tham();
    case CLASS_SCRIPT_TAI_VIET:
        return getUcpTai_Viet();
    case CLASS_SCRIPT_TAMIL:
        return getUcpTamil();
    case CLASS_SCRIPT_TELUGU:
        return getUcpTelugu();
    case CLASS_SCRIPT_THAANA:
        return getUcpThaana();
    case CLASS_SCRIPT_THAI:
        return getUcpThai();
    case CLASS_SCRIPT_TIBETAN:
        return getUcpTibetan();
    case CLASS_SCRIPT_TIFINAGH:
        return getUcpTifinagh();
    case CLASS_SCRIPT_UGARITIC:
        return getUcpUgaritic();
    case CLASS_SCRIPT_VAI:
        return getUcpVai();
    case CLASS_SCRIPT_YI:
        return getUcpYi();
    case CLASS_UCP_ANY:
        return CodePointSet(CodePointSet::interval(0, MAX_UNICODE));

    default: { /* currently uses ascii defns */
        CharReach cr = getPredefinedCharReach(c, mode);
        CodePointSet rv;
        for (u32 i = cr.find_first(); i != CharReach::npos;
             i = cr.find_next(i)) {
            rv.set(i);
        }
        return rv;
    }
    }
}

UTF8ComponentClass::UTF8ComponentClass(const ParseMode &mode_in)
    : ComponentClass(mode_in),
      single_pos(         GlushkovBuildState::POS_UNINITIALIZED),
      one_dot_trailer(    GlushkovBuildState::POS_UNINITIALIZED),
      two_dot_trailer(    GlushkovBuildState::POS_UNINITIALIZED),
      three_dot_trailer(  GlushkovBuildState::POS_UNINITIALIZED),
      two_char_dot_head(  GlushkovBuildState::POS_UNINITIALIZED),
      three_char_dot_head(GlushkovBuildState::POS_UNINITIALIZED),
      four_char_dot_head( GlushkovBuildState::POS_UNINITIALIZED) {
    assert(mode.utf8);
}

UTF8ComponentClass *UTF8ComponentClass::clone() const {
    return new UTF8ComponentClass(*this);
}

bool UTF8ComponentClass::class_empty(void) const {
    assert(finalized);
    return cps.none();
}

void UTF8ComponentClass::createRange(unichar to) {
    assert(range_start != INVALID_UNICODE);
    unichar from = range_start;
    if (from > to) {
        throw LocatedParseError("Range out of order in character class");
    }

    in_cand_range = false;
    CodePointSet ncps;
    ncps.setRange(from, to);
    if (mode.caseless) {
        make_caseless(&ncps);
    }
    cps |= ncps;
    range_start = INVALID_UNICODE;
}

void UTF8ComponentClass::add(PredefinedClass c, bool negative) {
    if (in_cand_range) { // can't form a range here
        throw LocatedParseError("Invalid range in character class");
    }

    if (mode.ucp) {
        c = translateForUcpMode(c, mode);
    }

    // caselessness is handled inside this call - don't apply make_caseless
    // to the result
    CodePointSet pcps = getPredefinedCodePointSet(c, mode);
    if (negative) {
        pcps.flip();
    }

    cps |= pcps;

    range_start = INVALID_UNICODE;
    in_cand_range = false;
}

void UTF8ComponentClass::add(unichar c) {
    DEBUG_PRINTF("adding \\x%08x\n", c);
    if (c > MAX_UNICODE) { // too big!
        throw LocatedParseError("Hexadecimal value is greater than \\x10FFFF");
    }

    if (in_cand_range) {
        createRange(c);
        return;
    }

    CodePointSet ncps;
    ncps.set(c);
    if (mode.caseless) {
        make_caseless(&ncps);
    }
    cps |= ncps;
    range_start = c;
}

void UTF8ComponentClass::finalize() {
    if (finalized) {
        return;
    }

    // Handle unclosed ranges, like '[a-]' and '[a-\Q\E]' -- in these cases the
    // dash is a literal dash.
    if (in_cand_range) {
        cps.set('-');
        in_cand_range = false;
    }

    if (m_negate) {
        cps.flip();
    }

    finalized = true;
}

Position UTF8ComponentClass::getHead(NFABuilder &builder, u8 first_byte) {
    map<u8, Position>::const_iterator it = heads.find(first_byte);
    if (it != heads.end()) {
        return it->second;
    }

    Position head = builder.makePositions(1);
    assert(heads.find(first_byte) == heads.end());
    builder.addCharReach(head, CharReach(first_byte));
    /* no report id as head can not be directly wired to accept */

    heads[first_byte] = head;
    return head;
}

void UTF8ComponentClass::ensureDotTrailer(GlushkovBuildState &bs) {
    NFABuilder &builder = bs.getBuilder();
    if (one_dot_trailer != GlushkovBuildState::POS_UNINITIALIZED) {
        return;
    }

    one_dot_trailer = builder.makePositions(1);
    builder.setNodeReportID(one_dot_trailer, 0);
    builder.addCharReach(one_dot_trailer, CharReach(0x80, 0xbf));
    tails.insert(one_dot_trailer);
}

void UTF8ComponentClass::ensureTwoDotTrailer(GlushkovBuildState &bs) {
    NFABuilder &builder = bs.getBuilder();
    if (two_dot_trailer != GlushkovBuildState::POS_UNINITIALIZED) {
        return;
    }

    ensureDotTrailer(bs);

    two_dot_trailer = builder.makePositions(1);
    builder.addCharReach(two_dot_trailer, CharReach(0x80, 0xbf));
    bs.addSuccessor(two_dot_trailer, one_dot_trailer);
}

void UTF8ComponentClass::ensureThreeDotTrailer(GlushkovBuildState &bs) {
    NFABuilder &builder = bs.getBuilder();
    if (three_dot_trailer != GlushkovBuildState::POS_UNINITIALIZED) {
        return;
    }

    ensureTwoDotTrailer(bs);

    three_dot_trailer = builder.makePositions(1);
    builder.addCharReach(three_dot_trailer, CharReach(0x80, 0xbf));
    bs.addSuccessor(three_dot_trailer, two_dot_trailer);
}

void UTF8ComponentClass::buildOneByte(GlushkovBuildState &bs) {
    NFABuilder &builder = bs.getBuilder();
    for (CodePointSet::const_iterator it = cps.begin(); it != cps.end(); ++it) {
        unichar b = lower(*it);
        unichar e = upper(*it) + 1;
        if (b >= UTF_2CHAR_MIN) {
            continue;
        }

        DEBUG_PRINTF("building vertices for [%u, %u)\n", b, e);

        if (single_pos == GlushkovBuildState::POS_UNINITIALIZED) {
            single_pos = builder.makePositions(1);
            builder.setNodeReportID(single_pos, 0 /* offset adj */);
            tails.insert(single_pos);
        }
        CharReach cr(b, MIN(e, UTF_2CHAR_MIN) - 1);
        builder.addCharReach(single_pos, cr);
    }
}

void UTF8ComponentClass::addToTail(GlushkovBuildState &bs,
                                   map<Position, Position> &finals,
                                   Position prev, unichar b, unichar e) {
    NFABuilder &builder = bs.getBuilder();
    Position tail;
    if (finals.find(prev) == finals.end()) {
        tail = builder.makePositions(1);
        builder.setNodeReportID(tail, 0 /* offset adj */);
        bs.addSuccessor(prev, tail);
        finals[prev] = tail;
        tails.insert(tail);
    } else {
        tail = finals[prev];
    }

    u8 bb = makeContByte(b);
    u8 ee = makeContByte(e - 1);
    builder.addCharReach(tail, CharReach(bb, ee));
}

void UTF8ComponentClass::buildTwoByte(GlushkovBuildState &bs) {
    NFABuilder &builder = bs.getBuilder();
    map<Position, Position> finals;

    for (auto it = cps.begin(); it != cps.end(); ++it) {
        unichar b = lower(*it);
        unichar e = upper(*it) + 1;

        b = MAX(b, UTF_2CHAR_MIN);
        e = MIN(e, UTF_3CHAR_MIN);

        if (b >= e) {
            continue; /* we're done here */
        }

        /* raise b to the start of the next tail byte boundary */
        if (b & UTF_CONT_BYTE_VALUE_MASK) {
            unichar bb = MIN(e, ROUNDUP_N(b, UTF_CONT_BYTE_RANGE));
            u8 first_byte = UTF_TWO_BYTE_HEADER | (b >> UTF_CONT_SHIFT);
            assert(first_byte > 0xc1 && first_byte <= 0xdf);

            Position head = getHead(builder, first_byte);
            addToTail(bs, finals, head, b, bb);

            b = bb;
        }

        if (b == e) {
            continue; /* we're done here */
        }
        assert(b < e);

        /* lower e to the end of a tail byte boundary */
        if (e & UTF_CONT_BYTE_VALUE_MASK) {
            unichar ee = e & ~UTF_CONT_BYTE_VALUE_MASK;
            assert(ee >= b);

            u8 first_byte = UTF_TWO_BYTE_HEADER | (ee >> UTF_CONT_SHIFT);
            assert(first_byte > 0xc1 && first_byte <= 0xdf);

            Position head = getHead(builder, first_byte);
            addToTail(bs, finals, head, ee, e);

            e = ee;
        }

        if (b == e) {
            continue; /* we're done here */
        }
        assert(b < e);

        /* middle section just goes to a common full vertex */
        ensureDotTrailer(bs);

        if (two_char_dot_head == GlushkovBuildState::POS_UNINITIALIZED) {
            two_char_dot_head = builder.makePositions(1);
            bs.addSuccessor(two_char_dot_head, one_dot_trailer);
        }

        u8 min_first_byte =  UTF_TWO_BYTE_HEADER | (b >> UTF_CONT_SHIFT);
        u8 max_first_byte =  UTF_TWO_BYTE_HEADER | ((e - 1) >> UTF_CONT_SHIFT);

        assert(min_first_byte > 0xc1 && min_first_byte <= 0xdf);
        assert(max_first_byte > 0xc1 && max_first_byte <= 0xdf);

        builder.addCharReach(two_char_dot_head,
                             CharReach(min_first_byte, max_first_byte));
    }
}

static
Position getMid(GlushkovBuildState &bs, map<Position, map<u8, Position> > &mids,
                const Position &prev, u8 byte_val) {
    NFABuilder &builder = bs.getBuilder();
    map<u8, Position> &by_byte = mids[prev];

    map<u8, Position>::const_iterator it = by_byte.find(byte_val);
    if (it != by_byte.end()) {
        return it->second;
    }

    Position mid = builder.makePositions(1);
    builder.addCharReach(mid, CharReach(byte_val));
    bs.addSuccessor(prev, mid);
    /* no report id as mid can not be directly wired to accept */

    by_byte[byte_val] = mid;
    return mid;
}

void UTF8ComponentClass::buildThreeByte(GlushkovBuildState &bs) {
    NFABuilder &builder = bs.getBuilder();

    map<Position, map<u8, Position> > mids;
    map<Position, Position> finals;

    for (auto it = cps.begin(); it != cps.end(); ++it) {
        unichar b = lower(*it);
        unichar e = upper(*it) + 1;

        b = MAX(b, UTF_3CHAR_MIN);
        e = MIN(e, UTF_4CHAR_MIN);

        if (b >= e) {
            continue; /* we're done here */
        }

        /* raise b to the start of the next tail byte boundary */
        if (b & UTF_CONT_BYTE_VALUE_MASK) {
            unichar bb = MIN(e, ROUNDUP_N(b, UTF_CONT_BYTE_RANGE));

            u8 first_byte = UTF_THREE_BYTE_HEADER | (b >> (2 * UTF_CONT_SHIFT));
            assert(first_byte >= 0xe0 && first_byte <= 0xef);
            Position head = getHead(builder, first_byte);

            u8 second_byte = makeContByte(b >> UTF_CONT_SHIFT);
            Position mid = getMid(bs, mids, head, second_byte);

            addToTail(bs, finals, mid, b, bb);

            b = bb;
        }

        if (b == e) {
            continue; /* we're done here */
        }
        assert(b < e);

        /* lower e to the end of a tail byte boundary */
        if (e & UTF_CONT_BYTE_VALUE_MASK) {
            unichar ee = e & ~UTF_CONT_BYTE_VALUE_MASK;
            assert(ee >= b);

            u8 first_byte = UTF_THREE_BYTE_HEADER
                          | (ee >> (2 * UTF_CONT_SHIFT));
            assert(first_byte >= 0xe0 && first_byte <= 0xef);
            Position head = getHead(builder, first_byte);

            u8 second_byte = makeContByte(ee >> UTF_CONT_SHIFT);
            Position mid = getMid(bs, mids, head, second_byte);

            addToTail(bs, finals, mid, ee, e);

            e = ee;
        }

        if (b == e) {
            continue; /* we're done here */
        }
        assert(b < e);

        /* from here on in the last byte is always full */
        ensureDotTrailer(bs);

        /* raise b to the start of the next mid byte boundary */
        if (b & ((1 << (2 * UTF_CONT_SHIFT)) - 1)) {
            unichar bb = MIN(e, ROUNDUP_N(b, 1 << (2 * UTF_CONT_SHIFT)));

            u8 first_byte = UTF_THREE_BYTE_HEADER | (b >> (2 * UTF_CONT_SHIFT));
            Position head = getHead(builder, first_byte);

            Position mid = builder.makePositions(1);
            bs.addSuccessor(head, mid);
            bs.addSuccessor(mid, one_dot_trailer);
            /* no report id as mid can not be directly wired to accept,
             * not adding to mids as we are completely filling its downstream */
            u8 second_min = makeContByte(b >> UTF_CONT_SHIFT);
            u8 second_max = makeContByte((bb - 1) >> UTF_CONT_SHIFT);

            builder.addCharReach(mid, CharReach(second_min, second_max));

            b = bb;
        }

        if (b == e) {
            continue; /* we're done here */
        }
        assert(b < e);

        /* lower e to the end of a mid byte boundary */
        if (e & ((1 << (2 * UTF_CONT_SHIFT)) - 1)) {
            unichar ee = e & ~((1 << (2 * UTF_CONT_SHIFT)) - 1);
            assert(ee >= b);

            u8 first_byte = UTF_THREE_BYTE_HEADER
                          | (ee >> (2 * UTF_CONT_SHIFT));
            Position head = getHead(builder, first_byte);

            Position mid = builder.makePositions(1);
            bs.addSuccessor(head, mid);
            bs.addSuccessor(mid, one_dot_trailer);
            /* no report id as mid can not be directly wired to accept,
             * not adding to mids as we are completely filling its downstream */
            u8 second_min = makeContByte(ee >> UTF_CONT_SHIFT);
            u8 second_max = makeContByte((e - 1) >> UTF_CONT_SHIFT);

            builder.addCharReach(mid, CharReach(second_min, second_max));

            e = ee;
        }

        if (b == e) {
            continue; /* we're done here */
        }
        assert(b < e);

        /* now we just have to wire head to a common dot trailer */
        ensureTwoDotTrailer(bs);
        if (three_char_dot_head == GlushkovBuildState::POS_UNINITIALIZED) {
            three_char_dot_head = builder.makePositions(1);
            bs.addSuccessor(three_char_dot_head, two_dot_trailer);
        }

        u8 min_first_byte =  UTF_THREE_BYTE_HEADER
                          | (b >> (2 * UTF_CONT_SHIFT));
        u8 max_first_byte =  UTF_THREE_BYTE_HEADER
                          | ((e - 1) >> (2 * UTF_CONT_SHIFT));

        assert(min_first_byte > 0xdf && min_first_byte <= 0xef);
        assert(max_first_byte > 0xdf && max_first_byte <= 0xef);

        builder.addCharReach(three_char_dot_head,
                             CharReach(min_first_byte, max_first_byte));
    }
}

static
u8 makeFirstByteOfFour(unichar raw) {
    u8 first_byte = UTF_FOUR_BYTE_HEADER | (raw >> (3 * UTF_CONT_SHIFT));
    assert(first_byte > 0xef && first_byte <= 0xf7);
    return first_byte;
}

static
bool isTwoContAligned(unichar raw) {
    return !(raw & ((1 << (2 * UTF_CONT_SHIFT)) - 1));
}

static
bool isThreeContAligned(unichar raw) {
    return !(raw & ((1 << (3 * UTF_CONT_SHIFT)) - 1));
}

void UTF8ComponentClass::buildFourByte(GlushkovBuildState &bs) {
    NFABuilder &builder = bs.getBuilder();
    map<Position, map<u8, Position> > mids;
    map<Position, Position> finals;

    for (auto it = cps.begin(); it != cps.end(); ++it) {
        unichar b = lower(*it);
        unichar e = upper(*it) + 1;

        b = MAX(b, UTF_4CHAR_MIN);
        e = MIN(e, MAX_UNICODE + 1);

        if (b >= e) {
            continue;
        }

        /* raise b to the start of the next tail byte boundary */
        if (b & UTF_CONT_BYTE_VALUE_MASK) {
            unichar bb = MIN(e, ROUNDUP_N(b, UTF_CONT_BYTE_RANGE));

            u8 first_byte = makeFirstByteOfFour(b);
            Position head = getHead(builder, first_byte);

            u8 second_byte = makeContByte(b >> (2 * UTF_CONT_SHIFT));
            Position mid1 = getMid(bs, mids, head, second_byte);

            u8 third_byte = makeContByte(b >> UTF_CONT_SHIFT);
            Position mid2 = getMid(bs, mids, mid1, third_byte);

            addToTail(bs, finals, mid2, b, bb);

            b = bb;
        }

        if (b == e) {
            continue; /* we're done here */
        }
        assert(b < e);

        /* lower e to the end of a tail byte boundary */
        if (e & UTF_CONT_BYTE_VALUE_MASK) {
            unichar ee = e & ~UTF_CONT_BYTE_VALUE_MASK;
            assert(ee >= b);

            u8 first_byte = makeFirstByteOfFour(ee);
            Position head = getHead(builder, first_byte);

            u8 second_byte = makeContByte(ee >> (2 * UTF_CONT_SHIFT));
            Position mid1 = getMid(bs, mids, head, second_byte);

            u8 third_byte = makeContByte(ee >> UTF_CONT_SHIFT);
            Position mid2 = getMid(bs, mids, mid1, third_byte);

            addToTail(bs, finals, mid2, ee, e);

            e = ee;
        }

        if (b == e) {
            continue; /* we're done here */
        }
        assert(b < e);

        /* from here on in the last byte is always full */
        ensureDotTrailer(bs);

        /* raise b to the start of the next mid byte boundary */
        if (!isTwoContAligned(b)) {
            unichar bb = MIN(e, ROUNDUP_N(b, 1 << (2 * UTF_CONT_SHIFT)));

            u8 first_byte = makeFirstByteOfFour(b);
            Position head = getHead(builder, first_byte);

            u8 second_byte = makeContByte(b >> (2 * UTF_CONT_SHIFT));
            Position mid1 = getMid(bs, mids, head, second_byte);

            Position mid2 = builder.makePositions(1);
            bs.addSuccessor(mid1, mid2);
            bs.addSuccessor(mid2, one_dot_trailer);
            /* no report id as mid can not be directly wired to accept,
             * not adding to mids as we are completely filling its downstream */
            u8 byte_min = makeContByte(b >> UTF_CONT_SHIFT);
            u8 byte_max = makeContByte((bb - 1) >> UTF_CONT_SHIFT);

            builder.addCharReach(mid2, CharReach(byte_min, byte_max));

            b = bb;
        }

        if (b == e) {
            continue; /* we're done here */
        }
        assert(b < e);

        /* lower e to the end of a mid byte boundary */
        if (!isTwoContAligned(e)) {
            unichar ee = e & ~((1 << (2 * UTF_CONT_SHIFT)) - 1);
            assert(ee >= b);

            u8 first_byte = makeFirstByteOfFour(ee);
            Position head = getHead(builder, first_byte);

            u8 second_byte = makeContByte(ee >> (2 * UTF_CONT_SHIFT));
            Position mid1 = getMid(bs, mids, head, second_byte);

            Position mid2 = builder.makePositions(1);
            bs.addSuccessor(mid1, mid2);
            bs.addSuccessor(mid2, one_dot_trailer);
            /* no report id as mid can not be directly wired to accept,
             * not adding to mids as we are completely filling its downstream */
            u8 byte_min = makeContByte(ee >> UTF_CONT_SHIFT);
            u8 byte_max = makeContByte((e - 1) >> UTF_CONT_SHIFT);

            builder.addCharReach(mid2, CharReach(byte_min, byte_max));

            e = ee;
        }

        if (b == e) {
            continue; /* we're done here */
        }
        assert(b < e);

        ensureTwoDotTrailer(bs);

        /* raise b to the next byte boundary */
        if (!isThreeContAligned(b)) {
            unichar bb = MIN(e, ROUNDUP_N(b, 1 << (3 * UTF_CONT_SHIFT)));

            u8 first_byte = makeFirstByteOfFour(b);
            Position head = getHead(builder, first_byte);

            Position mid1 = builder.makePositions(1);
            bs.addSuccessor(head, mid1);
            bs.addSuccessor(mid1, two_dot_trailer);
            /* no report id as mid can not be directly wired to accept,
             * not adding to mids as we are completely filling its downstream */
            u8 byte_min = makeContByte(b >> (2 * UTF_CONT_SHIFT));
            u8 byte_max = makeContByte((bb - 1) >> (2 * UTF_CONT_SHIFT));

            builder.addCharReach(mid1, CharReach(byte_min, byte_max));

            b = bb;
        }

        if (b == e) {
            continue; /* we're done here */
        }
        assert(b < e);

        /* lower e to the next byte boundary */
        if (!isThreeContAligned(e)) {
            unichar ee = e & ~((1 << (3 * UTF_CONT_SHIFT)) - 1);
            assert(ee >= b);

            u8 first_byte = makeFirstByteOfFour(ee);
            Position head = getHead(builder, first_byte);
            Position mid1 = builder.makePositions(1);
            bs.addSuccessor(head, mid1);
            bs.addSuccessor(mid1, two_dot_trailer);
            /* no report id as mid can not be directly wired to accept,
             * not adding to mids as we are completely filling its downstream */
            u8 byte_min = makeContByte(ee >> (2 * UTF_CONT_SHIFT));
            u8 byte_max = makeContByte((e - 1) >> (2 * UTF_CONT_SHIFT));

            builder.addCharReach(mid1, CharReach(byte_min, byte_max));

            e = ee;
        }

        if (b == e) {
            continue; /* we're done here */
        }
        assert(b < e);

        /* now we just have to wire head to a common dot trailer */
        ensureThreeDotTrailer(bs);
        if (four_char_dot_head == GlushkovBuildState::POS_UNINITIALIZED) {
            four_char_dot_head = builder.makePositions(1);
            bs.addSuccessor(four_char_dot_head, three_dot_trailer);
        }

        u8 min_first_byte = makeFirstByteOfFour(b);
        u8 max_first_byte = makeFirstByteOfFour(e - 1);

        builder.addCharReach(four_char_dot_head,
                             CharReach(min_first_byte, max_first_byte));
    }
}

void UTF8ComponentClass::notePositions(GlushkovBuildState &bs) {
    // We should always be finalized by now.
    assert(finalized);

    // An empty class is a special case; this would be generated by something
    // like /[\s\S]/8, which can never match. We treat these like we do the non
    // UTF-8 version: add a vertex with empty reach (to ensure we create a
    // connected graph) and pick it up later on.
    if (class_empty()) {
        DEBUG_PRINTF("empty class!\n");
        assert(single_pos == GlushkovBuildState::POS_UNINITIALIZED);
        NFABuilder &builder = bs.getBuilder();
        single_pos = builder.makePositions(1);
        builder.setNodeReportID(single_pos, 0 /* offset adj */);
        builder.addCharReach(single_pos, CharReach());
        tails.insert(single_pos);
        return;
    }

    buildOneByte(bs);
    buildTwoByte(bs);
    buildThreeByte(bs);
    buildFourByte(bs);
}

void UTF8ComponentClass::buildFollowSet(GlushkovBuildState &,
                                        const vector<PositionInfo> &) {
    /* states are wired in notePositions as all belong to this component. */
}

vector<PositionInfo> UTF8ComponentClass::first(void) const {
    vector<PositionInfo> rv;
    if (single_pos != GlushkovBuildState::POS_UNINITIALIZED) {
        rv.push_back(single_pos);
    }
    if (two_char_dot_head != GlushkovBuildState::POS_UNINITIALIZED) {
        rv.push_back(two_char_dot_head);
    }
    if (three_char_dot_head != GlushkovBuildState::POS_UNINITIALIZED) {
        rv.push_back(three_char_dot_head);
    }
    if (four_char_dot_head != GlushkovBuildState::POS_UNINITIALIZED) {
        rv.push_back(four_char_dot_head);
    }

    for (auto it = heads.begin(); it != heads.end(); ++it) {
        rv.push_back(it->second);
    }
    return rv;
}

vector<PositionInfo> UTF8ComponentClass::last(void) const {
    vector<PositionInfo> rv;

    rv.insert(rv.end(), tails.begin(), tails.end());
    return rv;
}

} // namespace ue2
