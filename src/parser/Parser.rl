/*
 * Copyright (c) 2015-2017, Intel Corporation
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
 * \brief Parser code (generated with Ragel from Parser.rl).
 */

#include "config.h"

/* Parser.cpp is a built source, may not be in same dir as parser files */
#include "parser/check_refs.h"
#include "parser/control_verbs.h"
#include "parser/ComponentAlternation.h"
#include "parser/ComponentAssertion.h"
#include "parser/ComponentAtomicGroup.h"
#include "parser/ComponentBackReference.h"
#include "parser/ComponentBoundary.h"
#include "parser/ComponentByte.h"
#include "parser/ComponentClass.h"
#include "parser/ComponentCondReference.h"
#include "parser/ComponentEmpty.h"
#include "parser/ComponentEUS.h"
#include "parser/Component.h"
#include "parser/ComponentRepeat.h"
#include "parser/ComponentSequence.h"
#include "parser/ComponentWordBoundary.h"
#include "parser/parse_error.h"
#include "parser/Parser.h"
#include "ue2common.h"
#include "util/compare.h"
#include "util/flat_containers.h"
#include "util/make_unique.h"
#include "util/unicode_def.h"
#include "util/verify_types.h"

#include <cassert>
#include <cctype>
#include <cstring>
#include <cstdlib>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

namespace ue2 {

#define PUSH_SEQUENCE do {\
        sequences.push_back(ExprState(currentSeq, (size_t)(ts - ptr), \
                mode)); \
    } while(0)
#define POP_SEQUENCE do {\
        currentSeq = sequences.back().seq; \
        mode = sequences.back().mode; \
        sequences.pop_back(); \
    } while(0)

namespace {

/** \brief Structure representing current state as we're parsing (current
 * sequence, current options). Stored in the 'sequences' vector. */
struct ExprState {
    ExprState(ComponentSequence *seq_in, size_t offset,
              const ParseMode &mode_in) :
        seq(seq_in), seqOffset(offset), mode(mode_in) {}

    ComponentSequence *seq; //!< current sequence
    size_t seqOffset; //!< offset seq was entered, for error reporting
    ParseMode mode; //!< current mode flags
};

} // namespace

static
unsigned parseAsDecimal(unsigned oct) {
    // The input was parsed as octal, but should have been parsed as decimal.
    // Deconstruct the octal number and reconstruct into decimal
    unsigned ret = 0;
    unsigned multiplier = 1;
    while (oct) {
        ret += (oct & 0x7) * multiplier;
        oct >>= 3;
        multiplier *= 10;
    }
    return ret;
}

/** \brief Maximum value for a positive integer. We use INT_MAX, as that's what
 * PCRE uses. */
static constexpr u32 MAX_NUMBER = INT_MAX;

static
void pushDec(u32 *acc, char raw_digit) {
    assert(raw_digit >= '0' && raw_digit <= '9');
    u32 digit_val = raw_digit - '0';

    // Ensure that we don't overflow.
    u64a val = ((u64a)*acc * 10) + digit_val;
    if (val > MAX_NUMBER) {
        throw LocatedParseError("Number is too big");
    }

    *acc = verify_u32(val);
}

static
void pushOct(u32 *acc, char raw_digit) {
    assert(raw_digit >= '0' && raw_digit <= '7');
    u32 digit_val = raw_digit - '0';

    // Ensure that we don't overflow.
    u64a val = ((u64a)*acc * 8) + digit_val;
    if (val > MAX_NUMBER) {
        throw LocatedParseError("Number is too big");
    }

    *acc = verify_u32(val);
}

static
void throwInvalidRepeat(void) {
    throw LocatedParseError("Invalid repeat");
}

static
void throwInvalidUtf8(void) {
    throw ParseError("Expression is not valid UTF-8.");
}

/**
 * Adds the given child component to the parent sequence, returning a pointer
 * to the new (child) "current sequence".
 */
static
ComponentSequence *enterSequence(ComponentSequence *parent,
                                 unique_ptr<ComponentSequence> child) {
    assert(parent);
    assert(child);

    ComponentSequence *seq = child.get();
    parent->addComponent(move(child));
    return seq;
}

static
void addLiteral(ComponentSequence *currentSeq, char c, const ParseMode &mode) {
    if (mode.utf8 && mode.caseless) {
        /* leverage ComponentClass to generate the vertices */
        auto cc = getComponentClass(mode);
        assert(cc);
        cc->add(c);
        cc->finalize();
        currentSeq->addComponent(move(cc));
    } else {
        currentSeq->addComponent(getLiteralComponentClass(c, mode.caseless));
    }
}

static
void addEscaped(ComponentSequence *currentSeq, unichar accum,
                const ParseMode &mode, const char *err_msg) {
    if (mode.utf8) {
        /* leverage ComponentClass to generate the vertices */
        auto cc = getComponentClass(mode);
        assert(cc);
        cc->add(accum);
        cc->finalize();
        currentSeq->addComponent(move(cc));
    } else {
        if (accum > 255) {
            throw LocatedParseError(err_msg);
        }
        addLiteral(currentSeq, (char)accum, mode);
    }
}

static
void addEscapedOctal(ComponentSequence *currentSeq, unichar accum,
                     const ParseMode &mode) {
    addEscaped(currentSeq, accum, mode, "Octal value is greater than \\377");
}

static
void addEscapedHex(ComponentSequence *currentSeq, unichar accum,
                   const ParseMode &mode) {
    addEscaped(currentSeq, accum, mode,
               "Hexadecimal value is greater than \\xFF");
}

#define SLASH_C_ERROR "\\c must be followed by an ASCII character"

static
u8 decodeCtrl(char raw) {
    if (raw & 0x80) {
        throw LocatedParseError(SLASH_C_ERROR);
    }
    return mytoupper(raw) ^ 0x40;
}

static
unichar readUtf8CodePoint2c(const char *s) {
    auto *ts = (const u8 *)s;
    assert(ts[0] >= 0xc0 && ts[0] < 0xe0);
    assert(ts[1] >= 0x80 && ts[1] < 0xc0);
    unichar val = ts[0] & 0x1f;
    val <<= 6;
    val |= ts[1] & 0x3f;
    DEBUG_PRINTF("utf8 %02hhx %02hhx ->\\x{%x}\n", ts[0],
                 ts[1], val);
    return val;
}

static
unichar readUtf8CodePoint3c(const char *s) {
    auto *ts = (const u8 *)s;
    assert(ts[0] >= 0xe0 && ts[0] < 0xf0);
    assert(ts[1] >= 0x80 && ts[1] < 0xc0);
    assert(ts[2] >= 0x80 && ts[2] < 0xc0);
    unichar val = ts[0] & 0x0f;
    val <<= 6;
    val |= ts[1] & 0x3f;
    val <<= 6;
    val |= ts[2] & 0x3f;
    DEBUG_PRINTF("utf8 %02hhx %02hhx %02hhx ->\\x{%x}\n", ts[0],
                 ts[1], ts[2], val);
    return val;
}

static
unichar readUtf8CodePoint4c(const char *s) {
    auto *ts = (const u8 *)s;
    assert(ts[0] >= 0xf0 && ts[0] < 0xf8);
    assert(ts[1] >= 0x80 && ts[1] < 0xc0);
    assert(ts[2] >= 0x80 && ts[2] < 0xc0);
    assert(ts[3] >= 0x80 && ts[3] < 0xc0);
    unichar val = ts[0] & 0x07;
    val <<= 6;
    val |= ts[1] & 0x3f;
    val <<= 6;
    val |= ts[2] & 0x3f;
    val <<= 6;
    val |= ts[3] & 0x3f;
    DEBUG_PRINTF("utf8 %02hhx %02hhx %02hhx %02hhx ->\\x{%x}\n", ts[0],
                 ts[1], ts[2], ts[3], val);
    return val;
}

%%{
    machine regex;

    action throwUnsupportedEscape {
        ostringstream str;
        str << "'\\" << *(ts + 1) << "' at index " << ts - ptr
            << " not supported in a character class.";
        throw ParseError(str.str());
    }
    action unsupportedProperty {
        throw LocatedParseError("Character property not supported");
    }
    action clearLabel { label.clear();}
    action appendLabelCharacter { label.push_back(fc);}
    action clearOctAccumulator { octAccumulator = 0;}
    action clearAccumulator { accumulator = 0;}
    action setOctAccumulator {
        octAccumulator = 0;
        pushOct(&octAccumulator, fc);
    }
    action setDecAccumulator {
        accumulator = 0;
        pushDec(&accumulator, fc);
    }
    action clearNM { repeatN = 0; repeatM = 0; }
    action appendN { pushDec(&repeatN, fc); }
    action appendM { pushDec(&repeatM, fc); }
    action appendAccumulatorOctDigit { pushOct(&octAccumulator, fc); }
    action appendAccumulatorDecDigit { pushDec(&accumulator, fc); }
    action appendAccumulatorHexDigit {
        accumulator *= 16;
        accumulator += fc - '0';
    }
    action appendAccumulatorHexL {
        accumulator *= 16;
        accumulator += 10 + fc - 'a';
    }
    action appendAccumulatorHexU {
        accumulator *= 16;
        accumulator += 10 + fc - 'A';
    }

    # enter a comment group, where we just scan for a close paren.
    action enterComment { 
        inComment = true;
        fgoto readComment;
    }

    # enter an extended mode comment, where we just scan for a newline.
    action enterNewlineTerminatedComment {
        inComment = true;
        fgoto readNewlineTerminatedComment;
    }

    # enter a CAPTURING group ( e.g. '(blah)' )
    action enterCapturingGroup {
        PUSH_SEQUENCE;
        auto seq = ue2::make_unique<ComponentSequence>();
        seq->setCaptureIndex(groupIndex++);
        currentSeq = enterSequence(currentSeq, move(seq));
    }

    # enter a NAMED CAPTURING group ( e.g. (?'<hatstand>blah) )
    action enterNamedGroup {
        assert(!label.empty()); // should be guaranteed by machine
        char c = *label.begin();
        if (c >= '0' && c <= '9') {
            throw LocatedParseError("Group name cannot begin with a digit");
        }
        if (!groupNames.insert(label).second) {
            throw LocatedParseError("Two named subpatterns use the name '" + label + "'");
        }
        PUSH_SEQUENCE;
        auto seq = ue2::make_unique<ComponentSequence>();
        seq->setCaptureIndex(groupIndex++);
        seq->setCaptureName(label);
        currentSeq = enterSequence(currentSeq, move(seq));
    }

    # enter a NON-CAPTURING group where we're modifying flags
    # ( e.g. '(?i:blah)' ). Standard non-capturing groups use this path
    # as well.
    action enterModifiedGroup {
        PUSH_SEQUENCE;
        mode = newMode;
        currentSeq =
            enterSequence(currentSeq, ue2::make_unique<ComponentSequence>());
    }

    action exitGroup {
        if (sequences.empty()) {
            throw LocatedParseError("Unmatched parentheses");
        }
        currentSeq->finalize();
        POP_SEQUENCE;
    }
    action enterZWLookAhead {
        PUSH_SEQUENCE;
        currentSeq = enterSequence(currentSeq,
            ue2::make_unique<ComponentAssertion>(ComponentAssertion::LOOKAHEAD,
                                                 ComponentAssertion::POS));
    }
    action enterZWNegLookAhead {
        PUSH_SEQUENCE;
        currentSeq = enterSequence(currentSeq,
            ue2::make_unique<ComponentAssertion>(ComponentAssertion::LOOKAHEAD,
                                                 ComponentAssertion::NEG));
    }
    action enterZWLookBehind {
        PUSH_SEQUENCE;
        currentSeq = enterSequence(currentSeq,
            ue2::make_unique<ComponentAssertion>(ComponentAssertion::LOOKBEHIND,
                                                 ComponentAssertion::POS));
    }
    action enterZWNegLookBehind {
        PUSH_SEQUENCE;
        currentSeq = enterSequence(currentSeq,
            ue2::make_unique<ComponentAssertion>(ComponentAssertion::LOOKBEHIND,
                                                 ComponentAssertion::NEG));
    }
    action enterEmbeddedCode {
        throw LocatedParseError("Embedded code is not supported");
    }
    action enterConditionUnsupported {
        throw LocatedParseError("Conditional subpattern unsupported");
    }
    action enterReferenceUnsupported {
        throw LocatedParseError("Subpattern reference unsupported");
    }
    action enterNumberedConditionalRef {
        if (accumulator == 0) {
            throw LocatedParseError("Numbered reference cannot be zero");
        }
        PUSH_SEQUENCE;
        currentSeq = enterSequence(currentSeq,
                ue2::make_unique<ComponentCondReference>(accumulator));
    }
    action enterNamedConditionalRef {
        PUSH_SEQUENCE;
        assert(!label.empty());
        currentSeq = enterSequence(currentSeq,
                ue2::make_unique<ComponentCondReference>(label));
    }
    action enterAtomicGroup {
        PUSH_SEQUENCE;
        currentSeq = enterSequence(currentSeq,
                                   ue2::make_unique<ComponentAtomicGroup>());
    }
    action eatClass {
        assert(!currentCls);
        assert(!inCharClass); // not reentrant
        currentCls = getComponentClass(mode);
        inCharClass = true;
        inCharClassEarly = true;
        currentClsBegin = ts;
        fgoto readClass;
    }
    action resetModifiers {
        newMode = mode;
    }
    action applyModifiers {
        mode = newMode;
        currentSeq->addComponent(ue2::make_unique<ComponentEmpty>());
    }
    action modifyMatchPositive {
        switch (fc) {
            case 'i':
                newMode.caseless = true;
                break;
            case 'm':
                newMode.multiline = true;
                break;
            case 's':
                newMode.dotall = true;
                break;
            case 'x':
                newMode.ignore_space = true;
                break;
            default:
                assert(0); // this action only called for [imsx]
                break;
        }
    }
    action modifyMatchNegative {
        switch (fc) {
            case 'i':
                newMode.caseless = false;
                break;
            case 'm':
                newMode.multiline = false;
                break;
            case 's':
                newMode.dotall = false;
                break;
            case 'x':
                newMode.ignore_space = false;
                break;
            default:
                assert(0); // this action only called for [imsx]
                break;
        }
    }
    action is_utf8 { mode.utf8 }
    action is_ignore_space { mode.ignore_space }
    action is_early_charclass { inCharClassEarly }

    action addNumberedBackRef {
        if (accumulator == 0) {
            throw LocatedParseError("Numbered reference cannot be zero");
        }
        currentSeq->addComponent(ue2::make_unique<ComponentBackReference>(accumulator));
    }

    action addNegativeNumberedBackRef {
        // Accumulator is a negative offset.
        if (accumulator == 0) {
            throw LocatedParseError("Numbered reference cannot be zero");
        }
        if (accumulator >= groupIndex) {
            throw LocatedParseError("Invalid reference");
        }
        unsigned idx = groupIndex - accumulator;
        currentSeq->addComponent(ue2::make_unique<ComponentBackReference>(idx));
    }

    action addNamedBackRef {
        currentSeq->addComponent(ue2::make_unique<ComponentBackReference>(label));
    }

    escapedOctal0 = '\\0' @clearOctAccumulator [0-7]{0,2} $appendAccumulatorOctDigit;
    escapedOctal2 = '\\' [1-7] $setOctAccumulator [0-7]{1,2} $appendAccumulatorOctDigit;
    escapedOctal2c = '\\' [1-7] $setOctAccumulator [0-7]{0,2} $appendAccumulatorOctDigit;
    backRefIdSingle = [1-7] $setDecAccumulator;
    backRefId = [1-9] $setDecAccumulator [0-9]+ $appendAccumulatorDecDigit;
    escapedHex = '\\x' @clearAccumulator ([0-9] $appendAccumulatorHexDigit | [a-f] $appendAccumulatorHexL | [A-F] $appendAccumulatorHexU){0,2};
    escapedCtrl = '\\c' any?;
    escapedUnsupported = '\\' [NluLU];
    repeatNM1 = '\{' @clearNM [0-9]+ $appendN ('}' @{repeatM = repeatN;} | ',' '\}' @{repeatM = ComponentRepeat::NoLimit;} | ',' [0-9]+ $appendM '}');

    backReferenceG = '\\g' @clearAccumulator [0-9]{1,3} $appendAccumulatorDecDigit;
    backReferenceGNegative = '\\g-' @clearAccumulator [0-9]{1,3} $appendAccumulatorDecDigit;
    backReferenceGBracket = '\\g{' @clearAccumulator [0-9]{1,3} $appendAccumulatorDecDigit '}';
    backReferenceGBracket2 = '\\g{-' @clearAccumulator [0-9]{1,3} $appendAccumulatorDecDigit '}';
    backReferenceGBracketName = '\\g{' @clearLabel [A-Za-z0-9_]+ $appendLabelCharacter '}';
    backReferenceKBracketName = '\\k{' @clearLabel [A-Za-z0-9_]+ $appendLabelCharacter '}';
    backReferenceKBracketName2 = '\\k<' @clearLabel [A-Za-z0-9_]+ $appendLabelCharacter '>';
    backReferenceKBracketName3 = '\\k\'' @clearLabel [A-Za-z0-9_]+ $appendLabelCharacter '\'';
    backReferenceP = '(?P=' @clearLabel [A-Za-z0-9_]+ $appendLabelCharacter ')';

    namedGroup1 = '(?<' @clearLabel [A-Za-z0-9_]+ $appendLabelCharacter '>';
    namedGroup2 = '(?\'' @clearLabel [A-Za-z0-9_]+ $appendLabelCharacter '\'';
    namedGroup3 = '(?P<' @clearLabel [A-Za-z0-9_]+ $appendLabelCharacter '>';

    namedConditionalRef1 = '(?(<' @clearLabel [A-Za-z0-9_]+ $appendLabelCharacter '>)';
    namedConditionalRef2 = '(?(\'' @clearLabel [A-Za-z0-9_]+ $appendLabelCharacter '\')';
    namedConditionalRef3 = '(?(' @clearLabel [A-Za-z0-9_]+ $appendLabelCharacter ')';

    numberedSubExpression = '(?' [+\-]? [0-9]+ ')';
    namedSubExpression = '(?' ('&'|'P>') [A-Za-z0-9_]+ ')';

    positiveMatchModifiers = [imsx]+ $modifyMatchPositive;
    negativeMatchModifiers = '-' [imsx]+ $modifyMatchNegative;
    matchModifiers = positiveMatchModifiers ? negativeMatchModifiers ?;

    utf8_cont = 0x80..0xbf;
    utf8_2c   = 0xc0..0xdf utf8_cont;
    utf8_3c   = 0xe0..0xef utf8_cont utf8_cont;
    utf8_4c   = 0xf0..0xf7 utf8_cont utf8_cont utf8_cont;
    hi_byte   = 0x80..0xff;

    whitespace = [\t\n\v\f\r ];

    #############################################################
    # Trivial parser to read Perl 5.10+ control verbs, introduced
    # by '(*'.
    #############################################################
    readVerb := |*
        'UTF8)' => {
            throw LocatedParseError("(*UTF8) must be at start of "
                                    "expression, encountered");
        };
        'UTF)' => {
            throw LocatedParseError("(*UTF) must be at start of "
                                    "expression, encountered");
        };
        'UCP)' => {
            throw LocatedParseError("(*UCP) must be at start of "
                                    "expression, encountered");
        };
        # Use the control verb mini-parser to report an error for this
        # unsupported/unknown verb.
        [^)]+ ')' => {
            ParseMode temp_mode;
            assert(ts - 2 >= ptr); // parser needs the '(*' at the start too.
            read_control_verbs(ts - 2, te, (ts - 2 - ptr), temp_mode);
            assert(0); // Should have thrown a parse error.
            throw LocatedParseError("Unknown control verb");
        };
        any => {
            throw LocatedParseError("Unknown control verb");
        };
    *|;

    #############################################################
    # Parser to read UCP
    #############################################################
    readUCP := |*
        'C'   => { currentCls->add(CLASS_UCP_C, negated); fret; };
        'Cc'  => { currentCls->add(CLASS_UCP_CC, negated); fret; };
        'Cf'  => { currentCls->add(CLASS_UCP_CF, negated); fret; };
        'Cn'  => { currentCls->add(CLASS_UCP_CN, negated); fret; };
        'Co'  => { currentCls->add(CLASS_UCP_CO, negated); fret; };
        'Cs'  => { currentCls->add(CLASS_UCP_CS, negated); fret; };
        'L'   => { currentCls->add(CLASS_UCP_L, negated); fret; };
        'Ll'  => { currentCls->add(CLASS_UCP_LL, negated); fret; };
        'Lm'  => { currentCls->add(CLASS_UCP_LM, negated); fret; };
        'Lo'  => { currentCls->add(CLASS_UCP_LO, negated); fret; };
        'Lt'  => { currentCls->add(CLASS_UCP_LT, negated); fret; };
        'Lu'  => { currentCls->add(CLASS_UCP_LU, negated); fret; };
        'L&'  => { currentCls->add(CLASS_UCP_L_AND, negated); fret; };
        'M'   => { currentCls->add(CLASS_UCP_M, negated); fret; };
        'Mc'  => { currentCls->add(CLASS_UCP_MC, negated); fret; };
        'Me'  => { currentCls->add(CLASS_UCP_ME, negated); fret; };
        'Mn'  => { currentCls->add(CLASS_UCP_MN, negated); fret; };
        'N'   => { currentCls->add(CLASS_UCP_N, negated); fret; };
        'Nd'  => { currentCls->add(CLASS_UCP_ND, negated); fret; };
        'Nl'  => { currentCls->add(CLASS_UCP_NL, negated); fret; };
        'No'  => { currentCls->add(CLASS_UCP_NO, negated); fret; };
        'P'   => { currentCls->add(CLASS_UCP_P, negated); fret; };
        'Pc'  => { currentCls->add(CLASS_UCP_PC, negated); fret; };
        'Pd'  => { currentCls->add(CLASS_UCP_PD, negated); fret; };
        'Pe'  => { currentCls->add(CLASS_UCP_PE, negated); fret; };
        'Pf'  => { currentCls->add(CLASS_UCP_PF, negated); fret; };
        'Pi'  => { currentCls->add(CLASS_UCP_PI, negated); fret; };
        'Po'  => { currentCls->add(CLASS_UCP_PO, negated); fret; };
        'Ps'  => { currentCls->add(CLASS_UCP_PS, negated); fret; };
        'S'   => { currentCls->add(CLASS_UCP_S, negated); fret; };
        'Sc'  => { currentCls->add(CLASS_UCP_SC, negated); fret; };
        'Sk'  => { currentCls->add(CLASS_UCP_SK, negated); fret; };
        'Sm'  => { currentCls->add(CLASS_UCP_SM, negated); fret; };
        'So'  => { currentCls->add(CLASS_UCP_SO, negated); fret; };
        'Z'   => { currentCls->add(CLASS_UCP_Z, negated); fret; };
        'Zl'  => { currentCls->add(CLASS_UCP_ZL, negated); fret; };
        'Zp'  => { currentCls->add(CLASS_UCP_ZP, negated); fret; };
        'Zs'  => { currentCls->add(CLASS_UCP_ZS, negated); fret; };
        'Xan' => { currentCls->add(CLASS_UCP_XAN, negated); fret; };
        'Xps' => { currentCls->add(CLASS_UCP_XPS, negated); fret; };
        'Xsp' => { currentCls->add(CLASS_UCP_XSP, negated); fret; };
        'Xwd' => { currentCls->add(CLASS_UCP_XWD, negated); fret; };
        'Arabic' => { currentCls->add(CLASS_SCRIPT_ARABIC, negated); fret; };
        'Armenian' => { currentCls->add(CLASS_SCRIPT_ARMENIAN, negated); fret; };
        'Avestan' => { currentCls->add(CLASS_SCRIPT_AVESTAN, negated); fret; };
        'Balinese' => { currentCls->add(CLASS_SCRIPT_BALINESE, negated); fret; };
        'Bamum' => { currentCls->add(CLASS_SCRIPT_BAMUM, negated); fret; };
        'Batak' => { currentCls->add(CLASS_SCRIPT_BATAK, negated); fret; };
        'Bengali' => { currentCls->add(CLASS_SCRIPT_BENGALI, negated); fret; };
        'Bopomofo' => { currentCls->add(CLASS_SCRIPT_BOPOMOFO, negated); fret; };
        'Brahmi' => { currentCls->add(CLASS_SCRIPT_BRAHMI, negated); fret; };
        'Braille' => { currentCls->add(CLASS_SCRIPT_BRAILLE, negated); fret; };
        'Buginese' => { currentCls->add(CLASS_SCRIPT_BUGINESE, negated); fret; };
        'Buhid' => { currentCls->add(CLASS_SCRIPT_BUHID, negated); fret; };
        'Canadian_Aboriginal' => { currentCls->add(CLASS_SCRIPT_CANADIAN_ABORIGINAL, negated); fret; };
        'Carian' => { currentCls->add(CLASS_SCRIPT_CARIAN, negated); fret; };
        'Cham' => { currentCls->add(CLASS_SCRIPT_CHAM, negated); fret; };
        'Cherokee' => { currentCls->add(CLASS_SCRIPT_CHEROKEE, negated); fret; };
        'Common' => { currentCls->add(CLASS_SCRIPT_COMMON, negated); fret; };
        'Coptic' => { currentCls->add(CLASS_SCRIPT_COPTIC, negated); fret; };
        'Cuneiform' => { currentCls->add(CLASS_SCRIPT_CUNEIFORM, negated); fret; };
        'Cypriot' => { currentCls->add(CLASS_SCRIPT_CYPRIOT, negated); fret; };
        'Cyrillic' => { currentCls->add(CLASS_SCRIPT_CYRILLIC, negated); fret; };
        'Deseret' => { currentCls->add(CLASS_SCRIPT_DESERET, negated); fret; };
        'Devanagari' => { currentCls->add(CLASS_SCRIPT_DEVANAGARI, negated); fret; };
        'Egyptian_Hieroglyphs' => { currentCls->add(CLASS_SCRIPT_EGYPTIAN_HIEROGLYPHS, negated); fret; };
        'Ethiopic' => { currentCls->add(CLASS_SCRIPT_ETHIOPIC, negated); fret; };
        'Georgian' => { currentCls->add(CLASS_SCRIPT_GEORGIAN, negated); fret; };
        'Glagolitic' => { currentCls->add(CLASS_SCRIPT_GLAGOLITIC, negated); fret; };
        'Gothic' => { currentCls->add(CLASS_SCRIPT_GOTHIC, negated); fret; };
        'Greek' => { currentCls->add(CLASS_SCRIPT_GREEK, negated); fret; };
        'Gujarati' => { currentCls->add(CLASS_SCRIPT_GUJARATI, negated); fret; };
        'Gurmukhi' => { currentCls->add(CLASS_SCRIPT_GURMUKHI, negated); fret; };
        'Han' => { currentCls->add(CLASS_SCRIPT_HAN, negated); fret; };
        'Hangul' => { currentCls->add(CLASS_SCRIPT_HANGUL, negated); fret; };
        'Hanunoo' => { currentCls->add(CLASS_SCRIPT_HANUNOO, negated); fret; };
        'Hebrew' => { currentCls->add(CLASS_SCRIPT_HEBREW, negated); fret; };
        'Hiragana' => { currentCls->add(CLASS_SCRIPT_HIRAGANA, negated); fret; };
        'Imperial_Aramaic' => { currentCls->add(CLASS_SCRIPT_IMPERIAL_ARAMAIC, negated); fret; };
        'Inherited' => { currentCls->add(CLASS_SCRIPT_INHERITED, negated); fret; };
        'Inscriptional_Pahlavi' => { currentCls->add(CLASS_SCRIPT_INSCRIPTIONAL_PAHLAVI, negated); fret; };
        'Inscriptional_Parthian' => { currentCls->add(CLASS_SCRIPT_INSCRIPTIONAL_PARTHIAN, negated); fret; };
        'Javanese' => { currentCls->add(CLASS_SCRIPT_JAVANESE, negated); fret; };
        'Kaithi' => { currentCls->add(CLASS_SCRIPT_KAITHI, negated); fret; };
        'Kannada' => { currentCls->add(CLASS_SCRIPT_KANNADA, negated); fret; };
        'Katakana' => { currentCls->add(CLASS_SCRIPT_KATAKANA, negated); fret; };
        'Kayah_Li' => { currentCls->add(CLASS_SCRIPT_KAYAH_LI, negated); fret; };
        'Kharoshthi' => { currentCls->add(CLASS_SCRIPT_KHAROSHTHI, negated); fret; };
        'Khmer' => { currentCls->add(CLASS_SCRIPT_KHMER, negated); fret; };
        'Lao' => { currentCls->add(CLASS_SCRIPT_LAO, negated); fret; };
        'Latin' => { currentCls->add(CLASS_SCRIPT_LATIN, negated); fret; };
        'Lepcha' => { currentCls->add(CLASS_SCRIPT_LEPCHA, negated); fret; };
        'Limbu' => { currentCls->add(CLASS_SCRIPT_LIMBU, negated); fret; };
        'Linear_B' => { currentCls->add(CLASS_SCRIPT_LINEAR_B, negated); fret; };
        'Lisu' => { currentCls->add(CLASS_SCRIPT_LISU, negated); fret; };
        'Lycian' => { currentCls->add(CLASS_SCRIPT_LYCIAN, negated); fret; };
        'Lydian' => { currentCls->add(CLASS_SCRIPT_LYDIAN, negated); fret; };
        'Malayalam' => { currentCls->add(CLASS_SCRIPT_MALAYALAM, negated); fret; };
        'Mandaic' => { currentCls->add(CLASS_SCRIPT_MANDAIC, negated); fret; };
        'Meetei_Mayek' => { currentCls->add(CLASS_SCRIPT_MEETEI_MAYEK, negated); fret; };
        'Mongolian' => { currentCls->add(CLASS_SCRIPT_MONGOLIAN, negated); fret; };
        'Myanmar' => { currentCls->add(CLASS_SCRIPT_MYANMAR, negated); fret; };
        'New_Tai_Lue' => { currentCls->add(CLASS_SCRIPT_NEW_TAI_LUE, negated); fret; };
        'Nko' => { currentCls->add(CLASS_SCRIPT_NKO, negated); fret; };
        'Ogham' => { currentCls->add(CLASS_SCRIPT_OGHAM, negated); fret; };
        'Ol_Chiki' => { currentCls->add(CLASS_SCRIPT_OL_CHIKI, negated); fret; };
        'Old_Italic' => { currentCls->add(CLASS_SCRIPT_OLD_ITALIC, negated); fret; };
        'Old_Persian' => { currentCls->add(CLASS_SCRIPT_OLD_PERSIAN, negated); fret; };
        'Old_South_Arabian' => { currentCls->add(CLASS_SCRIPT_OLD_SOUTH_ARABIAN, negated); fret; };
        'Old_Turkic' => { currentCls->add(CLASS_SCRIPT_OLD_TURKIC, negated); fret; };
        'Oriya' => { currentCls->add(CLASS_SCRIPT_ORIYA, negated); fret; };
        'Osmanya' => { currentCls->add(CLASS_SCRIPT_OSMANYA, negated); fret; };
        'Phags_Pa' => { currentCls->add(CLASS_SCRIPT_PHAGS_PA, negated); fret; };
        'Phoenician' => { currentCls->add(CLASS_SCRIPT_PHOENICIAN, negated); fret; };
        'Rejang' => { currentCls->add(CLASS_SCRIPT_REJANG, negated); fret; };
        'Runic' => { currentCls->add(CLASS_SCRIPT_RUNIC, negated); fret; };
        'Samaritan' => { currentCls->add(CLASS_SCRIPT_SAMARITAN, negated); fret; };
        'Saurashtra' => { currentCls->add(CLASS_SCRIPT_SAURASHTRA, negated); fret; };
        'Shavian' => { currentCls->add(CLASS_SCRIPT_SHAVIAN, negated); fret; };
        'Sinhala' => { currentCls->add(CLASS_SCRIPT_SINHALA, negated); fret; };
        'Sundanese' => { currentCls->add(CLASS_SCRIPT_SUNDANESE, negated); fret; };
        'Syloti_Nagri' => { currentCls->add(CLASS_SCRIPT_SYLOTI_NAGRI, negated); fret; };
        'Syriac' => { currentCls->add(CLASS_SCRIPT_SYRIAC, negated); fret; };
        'Tagalog' => { currentCls->add(CLASS_SCRIPT_TAGALOG, negated); fret; };
        'Tagbanwa' => { currentCls->add(CLASS_SCRIPT_TAGBANWA, negated); fret; };
        'Tai_Le' => { currentCls->add(CLASS_SCRIPT_TAI_LE, negated); fret; };
        'Tai_Tham' => { currentCls->add(CLASS_SCRIPT_TAI_THAM, negated); fret; };
        'Tai_Viet' => { currentCls->add(CLASS_SCRIPT_TAI_VIET, negated); fret; };
        'Tamil' => { currentCls->add(CLASS_SCRIPT_TAMIL, negated); fret; };
        'Telugu' => { currentCls->add(CLASS_SCRIPT_TELUGU, negated); fret; };
        'Thaana' => { currentCls->add(CLASS_SCRIPT_THAANA, negated); fret; };
        'Thai' => { currentCls->add(CLASS_SCRIPT_THAI, negated); fret; };
        'Tibetan' => { currentCls->add(CLASS_SCRIPT_TIBETAN, negated); fret; };
        'Tifinagh' => { currentCls->add(CLASS_SCRIPT_TIFINAGH, negated); fret; };
        'Ugaritic' => { currentCls->add(CLASS_SCRIPT_UGARITIC, negated); fret; };
        'Vai' => { currentCls->add(CLASS_SCRIPT_VAI, negated); fret; };
        'Yi' => { currentCls->add(CLASS_SCRIPT_YI, negated); fret; };
        'Any' => { currentCls->add(CLASS_UCP_ANY, negated); fret; };
        any => { throw LocatedParseError("Unknown property"); };
               *|;

    readBracedUCP := ('{'
                     ('^' ${ negated = !negated; }) ?
                     ([^^] ${ fhold; fcall readUCP; })
                      '}' ${ if (!inCharClass) { // not inside [..]
                                 currentCls->finalize();
                                 currentSeq->addComponent(move(currentCls));
                             }
                             fret; 
                           })
                          $^{ throw LocatedParseError("Malformed property"); };

    readUCPSingle := |*
        'C' => { 
            currentCls->add(CLASS_UCP_C, negated); 
            if (!inCharClass) {
                currentCls->finalize();
                currentSeq->addComponent(move(currentCls));
            }
            fret; 
        };
        'L' => { 
            currentCls->add(CLASS_UCP_L, negated); 
            if (!inCharClass) {
                currentCls->finalize();
                currentSeq->addComponent(move(currentCls));
            }
            fret; 
        };
        'M' => { 
            currentCls->add(CLASS_UCP_M, negated); 
            if (!inCharClass) {
                currentCls->finalize();
                currentSeq->addComponent(move(currentCls));
            }
            fret; 
        };
        'N' => {
            currentCls->add(CLASS_UCP_N, negated); 
            if (!inCharClass) {
                currentCls->finalize();
                currentSeq->addComponent(move(currentCls));
            }
            fret;
        };
        'P' => { 
            currentCls->add(CLASS_UCP_P, negated); 
            if (!inCharClass) {
                currentCls->finalize();
                currentSeq->addComponent(move(currentCls));
            }
            fret; 
        };
        'S' => { 
            currentCls->add(CLASS_UCP_S, negated); 
            if (!inCharClass) {
                currentCls->finalize();
                currentSeq->addComponent(move(currentCls));
            }
            fret; 
        };
        'Z' => { 
            currentCls->add(CLASS_UCP_Z, negated); 
            if (!inCharClass) {
                currentCls->finalize();
                currentSeq->addComponent(move(currentCls));
            }
            fret; 
        };

        any => { throw LocatedParseError("Unknown property"); };
                     *|;
    charClassGuts := |*
              # We don't support POSIX collating elements (neither does PCRE
              # or Perl). These look like [.ch.] or [=ch=].
              '\[\.' ( '\\]' | [^\]] )* '\.\]' |
              '\[=' ( '\\]' | [^\]] )* '=\]' => {
                  throw LocatedParseError("Unsupported POSIX collating "
                                          "element");
              };
              # Named sets
              # Adding these may cause the charclass to close, hence the
              # finalized check - UE-2276
              '[:alnum:]' => {
                  currentCls->add(CLASS_ALNUM, false);
              };
              '[:^alnum:]' => {
                  currentCls->add(CLASS_ALNUM, true);
              };
              '[:alpha:]' => {
                  currentCls->add(CLASS_ALPHA, false);
              };
              '[:^alpha:]' => {
                  currentCls->add(CLASS_ALPHA, true);
              };
              '[:ascii:]' => {
                  currentCls->add(CLASS_ASCII, false);
              };
              '[:^ascii:]' => {
                  currentCls->add(CLASS_ASCII, true);
              };
              '[:blank:]' => {
                  currentCls->add(CLASS_BLANK, false);
              };
              '[:^blank:]' => {
                  currentCls->add(CLASS_BLANK, true);
              };
              '[:cntrl:]' => {
                  currentCls->add(CLASS_CNTRL, false);
              };
              '[:^cntrl:]' => {
                  currentCls->add(CLASS_CNTRL, true);
              };
              '[:digit:]' => {
                  currentCls->add(CLASS_DIGIT, false);
              };
              '[:^digit:]' => {
                  currentCls->add(CLASS_DIGIT, true);
              };
              '[:graph:]' => {
                  currentCls->add(CLASS_GRAPH, false);
              };
              '[:^graph:]' => {
                  currentCls->add(CLASS_GRAPH, true);
              };
              '[:lower:]' => {
                  currentCls->add(CLASS_LOWER, false);
              };
              '[:^lower:]' => {
                  currentCls->add(CLASS_LOWER, true);
              };
              '[:print:]' => {
                  currentCls->add(CLASS_PRINT, false);
              };
              '[:^print:]' => {
                  currentCls->add(CLASS_PRINT, true);
              };
              '[:punct:]' => {
                  currentCls->add(CLASS_PUNCT, false);
              };
              '[:^punct:]' => {
                  currentCls->add(CLASS_PUNCT, true);
              };
              # Posix SPACE covers 9, 10, 11, 12, 13, 32
              '[:space:]' => {
                  currentCls->add(CLASS_SPACE, false);
              };
              '[:^space:]' => {
                  currentCls->add(CLASS_SPACE, true);
              };
              '[:upper:]' => {
                  currentCls->add(CLASS_UPPER, false);
              };
              '[:^upper:]' => {
                  currentCls->add(CLASS_UPPER, true);
              };
              '[:word:]' => {
                  currentCls->add(CLASS_WORD, false);
              };
              '[:^word:]' => {
                  currentCls->add(CLASS_WORD, true);
              };
              '[:xdigit:]' => {
                  currentCls->add(CLASS_XDIGIT, false);
              };
              '[:^xdigit:]' => {
                  currentCls->add(CLASS_XDIGIT, true);
              };
              # Anything else between "[:" and ":]" is an invalid POSIX class.
              # Note that "\]" counts as a literal char here.
              '\[:' ( '\\]' | [^\]] )* ':\]' => {
                  throw LocatedParseError("Invalid POSIX named class");
              };
              '\\Q' => {
                  fcall readQuotedClass;
              };
              '\\E' => { /*noop*/};
              # Backspace (this is only valid for \b in char classes)
              '\\b' => {
                  currentCls->add('\x08');
              };
              # Tab
              '\\t' => {
                  currentCls->add('\x09');
              };
              # Newline
              '\\n' => {
                  currentCls->add('\x0a');
              };
              # Carriage return
              '\\r' => {
                  currentCls->add('\x0d');
              };
              # Form feed
              '\\f' => {
                  currentCls->add('\x0c');
              };
              # Bell
              '\\a' => {
                  currentCls->add('\x07');
              };
              # Escape
              '\\e' => {
                  currentCls->add('\x1b');
              };
              # Horizontal whitespace
              '\\h' => {
                  currentCls->add(CLASS_HORZ, false);
              };
              # Not horizontal whitespace
              '\\H' => {
                  currentCls->add(CLASS_HORZ, true);
              };
              # Vertical whitespace
              '\\v' => {
                  currentCls->add(CLASS_VERT, false);
              };
              # Not vertical whitespace
              '\\V' => {
                  currentCls->add(CLASS_VERT, true);
              };

              '\\p{' => {
                  negated = false;
                  fhold;
                  fcall readBracedUCP;
              };

              '\\p' any => {
                  negated = false;
                  fhold;
                  fcall readUCPSingle;
              };

              '\\P{' => {
                  negated = true;
                  fhold;
                  fcall readBracedUCP;
              };

              '\\P'any => {
                  negated = true;
                  fhold;
                  fcall readUCPSingle;
              };

              '\\P' => { throw LocatedParseError("Malformed property"); };
              '\\p' => { throw LocatedParseError("Malformed property"); };

              # Octal
              escapedOctal0 => {
                  currentCls->add(octAccumulator);
              };
              escapedOctal2c => {
                  currentCls->add(octAccumulator);
              };

              '\\o{' [0-7]+ '}' => {
                  string oct(ts + 3, te - ts - 4);
                  unsigned long val;
                  try {
                      val = stoul(oct, nullptr, 8);
                  } catch (const std::out_of_range &) {
                      val = MAX_UNICODE + 1;
                  }
                  if ((!mode.utf8 && val > 255) || val > MAX_UNICODE) {
                      throw LocatedParseError("Value in \\o{...} sequence is too large");
                  }
                  currentCls->add((unichar)val);
              };

              # And for when it goes wrong
              '\\o' => {
                  throw LocatedParseError("Value in \\o{...} sequence is non-octal or missing braces");
              };

              # Hex
              escapedHex => {
                  currentCls->add(accumulator);
              };
              # not a back-ref, not octal, just PCRE madness
              '\\' [89] => {
                  // whatever we found here
                  currentCls->add(*(ts + 1));

              };
              # Unicode Hex
              '\\x{' xdigit+ '}' => {
                  string hex(ts + 3, te - ts - 4);
                  unsigned long val;
                  try {
                      val = stoul(hex, nullptr, 16);
                  } catch (const std::out_of_range &) {
                      val = MAX_UNICODE + 1;
                  }
                  if (val > MAX_UNICODE) {
                      throw LocatedParseError("Value in \\x{...} sequence is too large");
                  }
                  currentCls->add((unichar)val);
              };
              # And for when it goes wrong
              '\\x{' => {
                  throw LocatedParseError("Value in \\x{...} sequence is non-hex or missing }");
              };
              # Control characters
              escapedCtrl => {
                  if (te - ts < 3) {
                      assert(te - ts == 2);
                      throw LocatedParseError(SLASH_C_ERROR);
                  } else {
                      assert(te - ts == 3);
                      currentCls->add(decodeCtrl(ts[2]));
                  }
              };
              # Word character
              '\\w' => {
                  currentCls->add(CLASS_WORD, false);
              };
              # Non word character
              '\\W' => {
                  currentCls->add(CLASS_WORD, true);
              };
              # Whitespace character (except VT)
              '\\s' => {
                  currentCls->add(CLASS_SPACE, false);
              };
              # Non whitespace character
              '\\S' => {
                  currentCls->add(CLASS_SPACE, true);
              };
              # Digit character
              '\\d' => {
                  currentCls->add(CLASS_DIGIT, false);
              };
              # Non digit character
              '\\D' => {
                  currentCls->add(CLASS_DIGIT, true);
              };
              '\-' => {
                  currentCls->addDash();
              };

              # A bunch of unsupported (for now) escapes
              escapedUnsupported - '\\X' => throwUnsupportedEscape;

              # PCRE appears to discard escaped g in a char class (a backref bug?)
              '\\g' => throwUnsupportedEscape;

              # the too-hard basket: UE-944, UE-1134, UE-1157
              # many escaped single char literals shold be benign, but PCRE
              # breaks with them when adding to ranges, so unless they have
              # defined special meaning in a char-class we reject them to be
              # safe.
              '\\' alpha => throwUnsupportedEscape;

              '\\' any => {
                  // add the literal char
                  currentCls->add(*(ts + 1));
              };

              #unicode chars
              utf8_2c when is_utf8 => {
                  assert(mode.utf8);
                  currentCls->add(readUtf8CodePoint2c(ts));
              };

              utf8_3c when is_utf8 => {
                  assert(mode.utf8);
                  currentCls->add(readUtf8CodePoint3c(ts));
              };

              utf8_4c when is_utf8 => {
                  assert(mode.utf8);
                  currentCls->add(readUtf8CodePoint4c(ts));
              };

              hi_byte when is_utf8 => {
                  assert(mode.utf8);
                  throwInvalidUtf8();
              };

              # Literal character
              (any - ']') => {
                  currentCls->add((u8)*ts);
              };

              ']' => {
                  currentCls->finalize();
                  currentSeq->addComponent(move(currentCls));
                  inCharClass = false;
                  fgoto main;
              };
              *|;

    #############################################################
    # Parser to read stuff from a character class
    #############################################################
    readClass := |*
        # A caret at the beginning of the class means that the rest of the
        # class is negated.
        '\^' when is_early_charclass => {
            if (currentCls->isNegated()) {
                // Already seen a caret; the second one is not a meta-character.
                inCharClassEarly = false;
                fhold; fgoto charClassGuts;
            } else {
                currentCls->negate();
                // Note: we cannot switch off inCharClassEarly here, as /[^]]/
                // needs to use the right square bracket path below.
            }
        };
        # A right square bracket before anything "real" is interpreted as a
        # literal right square bracket.
        ']' when is_early_charclass => {
            currentCls->add(']');
            inCharClassEarly = false;
        };
        # if we hit a quote before anything "real", handle it
        '\\Q' => { fcall readQuotedClass; };
        '\\E' => { /*noop*/};

        # time for the real work to happen
        any => {
            inCharClassEarly = false;
            fhold;
            fgoto charClassGuts;
        };
        *|;

    #############################################################
    # Parser to read a quoted literal
    #############################################################
    readQuotedLiteral := |*
              # Escape sequence
              '\\E' => {
                  fgoto main;
              };

              #unicode chars
              utf8_2c when is_utf8 => {
                  assert(mode.utf8);
                  /* leverage ComponentClass to generate the vertices */
                  auto cc = getComponentClass(mode);
                  cc->add(readUtf8CodePoint2c(ts));
                  cc->finalize();
                  currentSeq->addComponent(move(cc));
              };

              utf8_3c when is_utf8 => {
                  assert(mode.utf8);
                  /* leverage ComponentClass to generate the vertices */
                  auto cc = getComponentClass(mode);
                  cc->add(readUtf8CodePoint3c(ts));
                  cc->finalize();
                  currentSeq->addComponent(move(cc));
              };

              utf8_4c when is_utf8 => {
                  assert(mode.utf8);
                  /* leverage ComponentClass to generate the vertices */
                  auto cc = getComponentClass(mode);
                  cc->add(readUtf8CodePoint4c(ts));
                  cc->finalize();
                  currentSeq->addComponent(move(cc));
              };

              hi_byte when is_utf8 => {
                  assert(mode.utf8);
                  throwInvalidUtf8();
              };

              # Literal character
              any => {
                  addLiteral(currentSeq, *ts, mode);
              };
            *|;

    #############################################################
    # Parser to read a quoted class
    #############################################################
    readQuotedClass := |*
              # Escape sequence
              '\\E' => {
                  fret;
              };

              #unicode chars
              utf8_2c when is_utf8 => {
                  assert(mode.utf8);
                  currentCls->add(readUtf8CodePoint2c(ts));
                  inCharClassEarly = false;
              };

              utf8_3c when is_utf8 => {
                  assert(mode.utf8);
                  currentCls->add(readUtf8CodePoint3c(ts));
                  inCharClassEarly = false;
              };

              utf8_4c when is_utf8 => {
                  assert(mode.utf8);
                  currentCls->add(readUtf8CodePoint4c(ts));
                  inCharClassEarly = false;
              };

              hi_byte when is_utf8 => {
                  assert(mode.utf8);
                  throwInvalidUtf8();
              };

              # Literal character
              any => {
                  currentCls->add(*ts);
                  inCharClassEarly = false;
              };
            *|;


    #############################################################
    # Parser to read (and ignore) a comment block
    #############################################################
    readComment := |*
                     # Right paren
                     '\)' => { inComment = false; fgoto main; };

                     # absolutely everything gets ignored until we see a right
                     # paren
                     any;
                   *|;

    #############################################################
    # Parser to read (and ignore) a newline-terminated comment
    # block
    #############################################################
    readNewlineTerminatedComment := |*
                     '\n' => { inComment = false; fgoto main; };

                     # absolutely everything gets ignored until we see a
                     # newline
                     any;
                   *|;

    #############################################################
    # Parser for standard components
    #############################################################
    main := |*
              #############################################################
              # Standard components
              #############################################################
              # Begin capturing group (non-capturing handled further down)
              '\(' => enterCapturingGroup;
              # End group
              '\)' => exitGroup;
              # Mark alternation
              '\|' => {
                  currentSeq->addAlternation();
              };
              # POSIX named elements should only be used inside a class. Note
              # that we need to be able to reject /[:\]:]/ here.
              '\[:' ( '\\]' | [^\]] )* ':\]' => {
                  throw LocatedParseError("POSIX named classes are only "
                                          "supported inside a class");
              };
              # We don't support POSIX collating elements (neither does PCRE
              # or Perl). These look like [.ch.] or [=ch=].
              '\[\.' ( '\\]' | [^\]] )* '\.\]' |
              '\[=' ( '\\]' | [^\]] )* '=\]' => {
                  throw LocatedParseError("Unsupported POSIX collating "
                                          "element");
              };
              # Begin eating characters for class
              '\[' => eatClass;
              # Begin quoted literal
              '\\Q' => {
                  fgoto readQuotedLiteral;
              };
              # An \E that is not preceded by a \Q is ignored
              '\\E' => { /* noop */ };
              # Match any character
              '\.' => {
                  currentSeq->addComponent(generateComponent(CLASS_ANY, false, mode));
              };
              # Match one byte
              '\\C' => {
                  if (mode.utf8) {
                      throw LocatedParseError("\\C is unsupported in UTF8");
                  }
                  currentSeq->addComponent(ue2::make_unique<ComponentByte>());
              };
              # Match 0 or more times (greedy)
              '\*' => {
                  if (!currentSeq->addRepeat(0, ComponentRepeat::NoLimit,
                                             ComponentRepeat::REPEAT_GREEDY)) {
                      throwInvalidRepeat();
                  }
              };
              # Match 0 or more times (non-greedy)
              '\*\?' => {
                  if (!currentSeq->addRepeat(0, ComponentRepeat::NoLimit,
                                        ComponentRepeat::REPEAT_NONGREEDY)) {
                      throwInvalidRepeat();
                  }
              };
              # Match 0 or more times (possessive)
              '\*\+' => {
                  if (!currentSeq->addRepeat(0, ComponentRepeat::NoLimit,
                                        ComponentRepeat::REPEAT_POSSESSIVE)) {
                      throwInvalidRepeat();
                  }
              };
              # Match 1 or more times (greedy)
              '\+' => {
                  if (!currentSeq->addRepeat(1, ComponentRepeat::NoLimit,
                                             ComponentRepeat::REPEAT_GREEDY)) {
                      throwInvalidRepeat();
                  }
              };
              # Match 1 or more times (non-greedy)
              '\+\?' => {
                  if (!currentSeq->addRepeat(1, ComponentRepeat::NoLimit,
                                        ComponentRepeat::REPEAT_NONGREEDY)) {
                      throwInvalidRepeat();
                  }
              };
              # Match 1 or more times (possessive)
              '\+\+' => {
                  if (!currentSeq->addRepeat(1, ComponentRepeat::NoLimit,
                                        ComponentRepeat::REPEAT_POSSESSIVE)) {
                      throwInvalidRepeat();
                  }
              };
              # Match 0 or 1 times (greedy)
              '\?' => {
                  if (!currentSeq->addRepeat(
                           0, 1, ComponentRepeat::REPEAT_GREEDY)) {
                      throwInvalidRepeat();
                  }
              };
              # Match 0 or 1 times (non-greedy)
              '\?\?' => {
                  if (!currentSeq->addRepeat(
                           0, 1, ComponentRepeat::REPEAT_NONGREEDY)) {
                      throwInvalidRepeat();
                  }
              };
              # Match 0 or 1 times (possessive)
              '\?\+' => {
                  if (!currentSeq->addRepeat(
                           0, 1, ComponentRepeat::REPEAT_POSSESSIVE)) {
                      throwInvalidRepeat();
                  }
              };
              # Match {n}|{n,}|{n,m} times (greedy)
              repeatNM1 => {
                  if (repeatN > repeatM || repeatM == 0) {
                      throwInvalidRepeat();
                  } else if (!currentSeq->addRepeat(
                                  repeatN, repeatM,
                                  ComponentRepeat::REPEAT_GREEDY)) {
                      throwInvalidRepeat();
                  }
              };
              # Match {n}|{n,}|{n,m} times (non-greedy)
              repeatNM1 '\?' => {
                  if (repeatN > repeatM || repeatM == 0) {
                      throwInvalidRepeat();
                  } else if (!currentSeq->addRepeat(
                                  repeatN, repeatM,
                                  ComponentRepeat::REPEAT_NONGREEDY)) {
                      throwInvalidRepeat();
                  }
              };
              # Match {n}|{n,}|{n,m} times (possessive)
              repeatNM1 '\+' => {
                  if (repeatN > repeatM || repeatM == 0) {
                      throwInvalidRepeat();
                  } else if (!currentSeq->addRepeat(
                                  repeatN, repeatM,
                                  ComponentRepeat::REPEAT_POSSESSIVE)) {
                      throwInvalidRepeat();
                  }
              };

              # In ignore_space mode, an unescaped # character introduces a
              # comment that runs until the next newline or the end of the
              # pattern.
              '\#' when is_ignore_space => enterNewlineTerminatedComment;

              # Perl 5.10 Special Backtracking Control Verbs: we support
              # UTF8/UCP, none of the others
              '(*' [^)] => { fhold; fcall readVerb; };

              # Earlier parser code checked for the terminating NULL and exited
              # explicitly.
              '\0' => { assert(0); fbreak; };

              #############################################################
              # Boundaries
              #############################################################

              # Start of data; also after internal newline in multiline mode
              '\^' => {
                  auto bound = mode.multiline ? ComponentBoundary::BEGIN_LINE
                                              : ComponentBoundary::BEGIN_STRING;
                  currentSeq->addComponent(ue2::make_unique<ComponentBoundary>(bound));
              };
              # End of data (with optional internal newline); also before
              # internal newline in multiline mode
              '\$' => {
                  auto bound = mode.multiline ? ComponentBoundary::END_LINE
                                              : ComponentBoundary::END_STRING_OPTIONAL_LF;
                  currentSeq->addComponent(ue2::make_unique<ComponentBoundary>(bound));
              };
              # Beginning of data
              '\\A' => {
                  auto bound = ComponentBoundary::BEGIN_STRING;
                  currentSeq->addComponent(ue2::make_unique<ComponentBoundary>(bound));
              };
              # End of data (with optional internal newline)
              '\\Z' => {
                  auto bound = ComponentBoundary::END_STRING_OPTIONAL_LF;
                  currentSeq->addComponent(ue2::make_unique<ComponentBoundary>(bound));
              };
              # End of data
              '\\z' => {
                  auto bound = ComponentBoundary::END_STRING;
                  currentSeq->addComponent(ue2::make_unique<ComponentBoundary>(bound));
              };
              # Word boundary
              '\\b' => {
                  currentSeq->addComponent(
                      ue2::make_unique<ComponentWordBoundary>(ts - ptr, false, mode));
              };
              # Non-word boundary
              '\\B' => {
                  currentSeq->addComponent(
                      ue2::make_unique<ComponentWordBoundary>(ts - ptr, true, mode));
              };

              #############################################################
              # Escaped chars
              #############################################################

              # Tab
              '\\t' => {
                  addLiteral(currentSeq, '\x09', mode);
              };
              # Newline
              '\\n' => {
                  addLiteral(currentSeq, '\x0a', mode);
              };
              # Carriage return
              '\\r' => {
                  addLiteral(currentSeq, '\x0d', mode);
              };
              # Form feed
              '\\f' => {
                  addLiteral(currentSeq, '\x0c', mode);
              };
              # Bell
              '\\a' => {
                  addLiteral(currentSeq, '\x07', mode);
              };
              # Escape
              '\\e' => {
                  addLiteral(currentSeq, '\x1b', mode);
              };
              # Octal
              escapedOctal0 => {
                  addLiteral(currentSeq, octAccumulator, mode);
              };
              escapedOctal2 => {
                  // If there are enough capturing sub expressions, this may be
                  // a back reference
                  accumulator = parseAsDecimal(octAccumulator);
                  if (accumulator < groupIndex) {
                      currentSeq->addComponent(ue2::make_unique<ComponentBackReference>(accumulator));
                  } else {
                      addEscapedOctal(currentSeq, octAccumulator, mode);
                  }
              };

              # Numeric back reference
              # everything less than 8 is a straight up back ref, even if
              # it is a forwards backward reference (aieeee!)
              # Note that \8 and \9 are the literal chars '8' and '9'.
              '\\' backRefIdSingle => addNumberedBackRef;
              # otherwise we need to munge through the possible backref
              '\\' backRefId => {
                  // if there are enough left parens to this point, back ref
                  if (accumulator < groupIndex) {
                      currentSeq->addComponent(ue2::make_unique<ComponentBackReference>(accumulator));
                  } else {
                      // Otherwise, we interpret the first three digits as an
                      // octal escape, and the remaining characters stand for
                      // themselves as literals.
                      const char *s = ts;
                      unsigned int accum = 0;
                      unsigned int oct_digits = 0;
                      assert(*s == '\\'); // token starts at backslash
                      for (++s; s < te && oct_digits < 3; ++oct_digits, ++s) {
                          u8 digit = *s - '0';
                          if (digit < 8) {
                              accum = digit + accum * 8;
                          } else {
                              break;
                          }
                      }

                      if (oct_digits > 0) {
                          addEscapedOctal(currentSeq, accum, mode);
                      }

                      // And then the rest of the digits, if any, are literal.
                      for (; s < te; ++s) {
                          addLiteral(currentSeq, *s, mode);
                      }
                  }
              };
              backReferenceG => addNumberedBackRef;
              backReferenceGNegative => addNegativeNumberedBackRef;
              backReferenceGBracket => addNumberedBackRef;
              backReferenceGBracket2 => addNegativeNumberedBackRef;
              backReferenceGBracketName => addNamedBackRef;
              backReferenceKBracketName => addNamedBackRef;
              backReferenceKBracketName2 => addNamedBackRef;
              backReferenceKBracketName3 => addNamedBackRef;
              backReferenceP => addNamedBackRef;
              # Oniguruma - either angle braces or single quotes for this one
              ('\\g<' [^>]*? '>'|'\\g\'' [^\']*? '\'') => {
                  ostringstream str;
                  str << "Onigiruma subroutine call at index " << ts - ptr <<
                         " not supported.";
                  throw ParseError(str.str());
              };
              # Fallthrough: a \g that hasn't been caught by one of the above
              # is invalid syntax. Without this rule, we would accept /A\g/.
              '\\g' => {
                  throw LocatedParseError("Invalid reference after \\g");
              };
              '\\o{' [0-7]+ '}' => {
                  string oct(ts + 3, te - ts - 4);
                  unsigned long val;
                  try {
                      val = stoul(oct, nullptr, 8);
                  } catch (const std::out_of_range &) {
                      val = MAX_UNICODE + 1;
                  }
                  if ((!mode.utf8 && val > 255) || val > MAX_UNICODE) {
                      throw LocatedParseError("Value in \\o{...} sequence is too large");
                  }
                  addEscapedOctal(currentSeq, (unichar)val, mode);
              };
              # And for when it goes wrong
              '\\o' => {
                  throw LocatedParseError("Value in \\o{...} sequence is non-octal or missing braces");
              };
              # Hex
              escapedHex => {
                  addEscapedHex(currentSeq, accumulator, mode);
              };
              # Unicode Hex
              '\\x{' xdigit+ '}' => {
                  string hex(ts + 3, te - ts - 4);
                  unsigned long val;
                  try {
                      val = stoul(hex, nullptr, 16);
                  } catch (const std::out_of_range &) {
                      val = MAX_UNICODE + 1;
                  }
                  if (val > MAX_UNICODE) {
                      throw LocatedParseError("Value in \\x{...} sequence is too large");
                  }
                  addEscapedHex(currentSeq, (unichar)val, mode);
              };
              # And for when it goes wrong
              '\\x{' => {
                  throw LocatedParseError("Value in \\x{...} sequence is non-hex or missing }");
              };
              # Control characters
              escapedCtrl => {
                  if (te - ts < 3) {
                      assert(te - ts == 2);
                      throw LocatedParseError(SLASH_C_ERROR);
                  } else {
                      assert(te - ts == 3);
                      addLiteral(currentSeq, decodeCtrl(ts[2]), mode);
                  }
              };
              # A bunch of unsupported (for now) escapes
              escapedUnsupported => {
                  ostringstream str;
                  str << "'\\" << *(ts + 1) << "' at index " << ts - ptr
                      << " not supported.";
                  throw ParseError(str.str());
              };

              # Word character
              '\\w' => {
                  auto cc = generateComponent(CLASS_WORD, false, mode);
                  currentSeq->addComponent(move(cc));
              };
              # Non word character
              '\\W' => {
                  auto cc = generateComponent(CLASS_WORD, true, mode);
                  currentSeq->addComponent(move(cc));
              };
              # Whitespace character
              '\\s' => {
                  auto cc = generateComponent(CLASS_SPACE, false, mode);
                  currentSeq->addComponent(move(cc));
              };
              # Non whitespace character
              '\\S' => {
                  auto cc = generateComponent(CLASS_SPACE, true, mode);
                  currentSeq->addComponent(move(cc));
              };
              # Digit character
              '\\d' => {
                  auto cc = generateComponent(CLASS_DIGIT, false, mode);
                  currentSeq->addComponent(move(cc));
              };
              # Non digit character
              '\\D' => {
                  auto cc = generateComponent(CLASS_DIGIT, true, mode);
                  currentSeq->addComponent(move(cc));
              };
              # Horizontal whitespace
              '\\h' => {
                  auto cc = generateComponent(CLASS_HORZ, false, mode);
                  currentSeq->addComponent(move(cc));
              };
              # Not horizontal whitespace
              '\\H' => {
                  auto cc = generateComponent(CLASS_HORZ, true, mode);
                  currentSeq->addComponent(move(cc));
              };
              # Vertical whitespace
              '\\v' => {
                  auto cc = generateComponent(CLASS_VERT, false, mode);
                  currentSeq->addComponent(move(cc));
              };
              # Not vertical whitespace
              '\\V' => {
                  auto cc = generateComponent(CLASS_VERT, true, mode);
                  currentSeq->addComponent(move(cc));
              };

              '\\p{' => {
                  assert(!currentCls && !inCharClass);
                  currentCls = getComponentClass(mode);
                  negated = false;
                  fhold;
                  fcall readBracedUCP;
              };

              '\\p' any => {
                  assert(!currentCls && !inCharClass);
                  currentCls = getComponentClass(mode);
                  negated = false;
                  fhold;
                  fcall readUCPSingle;
              };

              '\\P{' => {
                  assert(!currentCls && !inCharClass);
                  currentCls = getComponentClass(mode);
                  negated = true;
                  fhold;
                  fcall readBracedUCP;
              };

              '\\P' any => {
                  assert(!currentCls && !inCharClass);
                  currentCls = getComponentClass(mode);
                  negated = true;
                  fhold;
                  fcall readUCPSingle;
              };

              '\\P' => { throw LocatedParseError("Malformed property"); };
              '\\p' => { throw LocatedParseError("Malformed property"); };

              # Newline sequence, hairy semantics that we don't do
              '\\R' => {
                  ostringstream str;
                  str << "\\R at index " << ts - ptr << " not supported.";
                  throw ParseError(str.str());
              };

              # Reset start of match, also hairy semantics that we don't do
              '\\K' => {
                  ostringstream str;
                  str << "\\K at index " << ts - ptr << " not supported.";
                  throw ParseError(str.str());
              };

              # \k without a backref is bugged in PCRE so we have no
              # idea what our semantics should be on it
              '\\k' => {
                  ostringstream str;
                  str << "\\k at index " << ts - ptr << " not supported.";
                  throw ParseError(str.str());
              };

              # \G is more hairy pcre-api stuff, DO NOT WANT
              '\\G' => {
                  ostringstream str;
                  str << "\\G at index " << ts - ptr << " not supported.";
                  throw ParseError(str.str());
              };

              '\\X' => {
                  currentSeq->addComponent(ue2::make_unique<ComponentEUS>(ts - ptr, mode));
              };

              # Fall through general escaped character
              '\\' any => {
                  addLiteral(currentSeq, *(ts + 1), mode);
              };

              # A backslash with no follower is not allowed
              '\\' => {
                  assert(ts + 1 == pe);
                  ostringstream str;
                  str << "Unescaped \\ at end of input, index " << ts - ptr << ".";
                  throw ParseError(str.str());
              };

              #############################################################
              # Extended patterns
              #############################################################

              # Comment
              '\(\?\#' => enterComment;
              # Match modifiers
              '\(\?' matchModifiers >resetModifiers ')' => applyModifiers;
              # Non-capturing group, with flag modifiers
              '\(\?' matchModifiers >resetModifiers ':' => enterModifiedGroup;
              # Zero width look ahead assertion
              '\(\?=' => enterZWLookAhead;
              # Zero width negative look ahead assertion
              '\(\?\!' => enterZWNegLookAhead;
              # Zero width look behind assertion
              '\(\?\<=' => enterZWLookBehind;
              # Zero width negative look behind assertion
              '\(\?\<\!' => enterZWNegLookBehind;
              # Code (TOTALLY unsupported... for good reason)
              '\(\?\{' => enterEmbeddedCode;
              '\(\?\?\{' => enterEmbeddedCode;
              # Atomic group
              '\(\?\>' => enterAtomicGroup;

              # Named capturing groups
              ( namedGroup1 |
                namedGroup2 |
                namedGroup3 ) => enterNamedGroup;

              # named/numbered subroutine references
              numberedSubExpression => enterReferenceUnsupported;
              namedSubExpression => enterReferenceUnsupported;

              # Conditional reference with a positive lookahead assertion
              '(?(?=' => {
                  auto a = ue2::make_unique<ComponentAssertion>(
                        ComponentAssertion::LOOKAHEAD, ComponentAssertion::POS);
                  ComponentAssertion *a_seq = a.get();
                  PUSH_SEQUENCE;
                  currentSeq = enterSequence(currentSeq,
                        ue2::make_unique<ComponentCondReference>(move(a)));
                  PUSH_SEQUENCE;
                  currentSeq = a_seq;
              };
              # Conditional reference with a negative lookahead assertion
              '(?(?!' => {
                  auto a = ue2::make_unique<ComponentAssertion>(
                        ComponentAssertion::LOOKAHEAD, ComponentAssertion::NEG);
                  ComponentAssertion *a_seq = a.get();
                  PUSH_SEQUENCE;
                  currentSeq = enterSequence(currentSeq,
                        ue2::make_unique<ComponentCondReference>(move(a)));
                  PUSH_SEQUENCE;
                  currentSeq = a_seq;
              };
              # Conditional reference with a positive lookbehind assertion
              '(?(?<=' => {
                  auto a = ue2::make_unique<ComponentAssertion>(
                      ComponentAssertion::LOOKBEHIND, ComponentAssertion::POS);
                  ComponentAssertion *a_seq = a.get();
                  PUSH_SEQUENCE;
                  currentSeq = enterSequence(currentSeq,
                        ue2::make_unique<ComponentCondReference>(move(a)));
                  PUSH_SEQUENCE;
                  currentSeq = a_seq;
              };
              # Conditional reference with a negative lookbehind assertion
              '(?(?<!' => {
                  auto a = ue2::make_unique<ComponentAssertion>(
                      ComponentAssertion::LOOKBEHIND, ComponentAssertion::NEG);
                  ComponentAssertion *a_seq = a.get();
                  PUSH_SEQUENCE;
                  currentSeq = enterSequence(currentSeq,
                        ue2::make_unique<ComponentCondReference>(move(a)));
                  PUSH_SEQUENCE;
                  currentSeq = a_seq;
              };

              # Recursive conditional references (unsupported)
              '(?(R' ( [0-9]+ | ('&' [A-Za-z0-9_]+) ) ? ')' => {
                  throw LocatedParseError("Pattern recursion not supported");
              };

              # Conditional references
              # numbered
              '\(\?\(' (backRefIdSingle | backRefId) ')' => enterNumberedConditionalRef;
              # named
              ( namedConditionalRef1 |
                namedConditionalRef2 |
                namedConditionalRef3 ) => enterNamedConditionalRef;

              # Conditions (unsupported)
              '\(\?\(' => enterConditionUnsupported;

              # Callouts (unsupported)
              '\(\?C' [0-9]* '\)' => {
                  ostringstream str;
                  str << "Callout at index " << ts - ptr << " not supported.";
                  throw ParseError(str.str());
              };

              # Any other char after '(?' is a pattern modifier we don't
              # recognise.
              '\(\?' any => {
                  throw LocatedParseError("Unrecognised character after (?");
              };

              #unicode chars
              utf8_2c when is_utf8 => {
                  assert(mode.utf8);
                  /* leverage ComponentClass to generate the vertices */
                  auto cc = getComponentClass(mode);
                  cc->add(readUtf8CodePoint2c(ts));
                  cc->finalize();
                  currentSeq->addComponent(move(cc));
              };

              utf8_3c when is_utf8 => {
                  assert(mode.utf8);
                  /* leverage ComponentClass to generate the vertices */
                  auto cc = getComponentClass(mode);
                  cc->add(readUtf8CodePoint3c(ts));
                  cc->finalize();
                  currentSeq->addComponent(move(cc));
              };

              utf8_4c when is_utf8 => {
                  assert(mode.utf8);
                  /* leverage ComponentClass to generate the vertices */
                  auto cc = getComponentClass(mode);
                  cc->add(readUtf8CodePoint4c(ts));
                  cc->finalize();
                  currentSeq->addComponent(move(cc));
              };

              hi_byte when is_utf8 => {
                  assert(mode.utf8);
                  throwInvalidUtf8();
              };

              #############################################################
              # Literal character
              #############################################################
              # literal character
              whitespace => {
                  if (mode.ignore_space == false) {
                      addLiteral(currentSeq, *ts, mode);
                  }
              };
              any => {
                  addLiteral(currentSeq, *ts, mode);
              };
           *|;

    prepush {
        DEBUG_PRINTF("stack %zu top %d\n", stack.size(), top);
        if ((int)stack.size() == top) {
            stack.resize(2 * (top + 1));
        }
    }
}%%

%% write data nofinal;

/** \brief Main parser call, returns root Component or nullptr. */
unique_ptr<Component> parse(const char *ptr, ParseMode &globalMode) {
    assert(ptr);

    const char *p = ptr;
    const char *pe = ptr + strlen(ptr);

    // First, read the control verbs, set any global mode flags and move the
    // ptr forward.
    p = read_control_verbs(p, pe, 0, globalMode);

    const char *eof = pe;
    int cs;
    UNUSED int act;
    int top;
    vector<int> stack;
    const char *ts, *te;
    unichar accumulator = 0;
    unichar octAccumulator = 0; /* required as we are also accumulating for
                                 * back ref when looking for octals */
    unsigned repeatN = 0;
    unsigned repeatM = 0;
    string label;

    ParseMode mode = globalMode;
    ParseMode newMode;

    bool negated = false;
    bool inComment = false;

    // Stack of sequences and flags used to store state when we enter
    // sub-sequences.
    vector<ExprState> sequences;

    // Index of the next capturing group. Note that zero is reserved for the
    // root sequence.
    unsigned groupIndex = 1;

    // Set storing group names that are currently in use.
    flat_set<string> groupNames;

    // Root sequence.
    unique_ptr<ComponentSequence> rootSeq = ue2::make_unique<ComponentSequence>();
    rootSeq->setCaptureIndex(0);

    // Current sequence being appended to
    ComponentSequence *currentSeq = rootSeq.get();

    // The current character class being appended to. This is used as the
    // accumulator for both character class and UCP properties.
    unique_ptr<ComponentClass> currentCls;

    // True if the machine is currently inside a character class, i.e. square
    // brackets [..].
    bool inCharClass = false;

    // True if the machine is inside a character class but it has not processed
    // any "real" elements yet, i.e. it's still processing meta-characters like
    // '^'.
    bool inCharClassEarly = false;

    // Location at which the current character class began.
    const char *currentClsBegin = p;

    // We throw exceptions on various parsing failures beyond this point: we
    // use a try/catch block here to clean up our allocated memory before we
    // re-throw the exception to the caller.
    try {
        // Embed the Ragel machine here
        %% write init;
        %% write exec;

        if (p != pe && *p != '\0') {
            // didn't make it to the end of our input, but we didn't throw a ParseError?
            assert(0);
            ostringstream str;
            str << "Parse error at index " << (p - ptr) << ".";
            throw ParseError(str.str());
        }

        if (currentCls) {
            assert(inCharClass);
            assert(currentClsBegin);
            ostringstream oss;
            oss << "Unterminated character class starting at index "
                << currentClsBegin - ptr << ".";
            throw ParseError(oss.str());
        }

        if (inComment) {
            throw ParseError("Unterminated comment.");
        }

        if (!sequences.empty()) {
            ostringstream str;
            str << "Missing close parenthesis for group started at index "
                << sequences.back().seqOffset << ".";
            throw ParseError(str.str());
        }

        // Unlikely, but possible
        if (groupIndex > 65535) {
            throw ParseError("The maximum number of capturing subexpressions is 65535.");
        }

        // Finalize the top-level sequence, which will take care of any
        // top-level alternation.
        currentSeq->finalize();
        assert(currentSeq == rootSeq.get());

        // Ensure that all references are valid.
        checkReferences(*rootSeq, groupIndex, groupNames);

        return move(rootSeq);
    } catch (LocatedParseError &error) {
        if (ts >= ptr && ts <= pe) {
            error.locate(ts - ptr);
        } else {
            error.locate(0);
        }
        throw;
    }
}

} // namespace ue2
