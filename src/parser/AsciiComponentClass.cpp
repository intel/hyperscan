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
#include "AsciiComponentClass.h"
#include "Utf8ComponentClass.h"
#include "buildstate.h"
#include "parse_error.h"
#include "position.h"
#include "position_info.h"
#include "nfagraph/ng_builder.h"
#include "util/charreach_util.h"

using namespace std;

namespace ue2 {

AsciiComponentClass::AsciiComponentClass(const ParseMode &mode_in)
    : ComponentClass(mode_in), position(GlushkovBuildState::POS_UNINITIALIZED) {
    assert(!mode.utf8);
}

AsciiComponentClass *AsciiComponentClass::clone() const {
    return new AsciiComponentClass(*this);
}

bool AsciiComponentClass::class_empty(void) const {
    assert(finalized);
    return cr.none();
}

void AsciiComponentClass::createRange(unichar to) {
    assert(range_start <= 0xff);
    unsigned char from = (u8)range_start;
    if (from > to) {
        throw LocatedParseError("Range out of order in character class");
    }

    in_cand_range = false;
    CharReach ncr(from, to);
    if (mode.caseless) {
        make_caseless(&ncr);
    }
    cr |= ncr;
    range_start = INVALID_UNICODE;
}

void AsciiComponentClass::notePositions(GlushkovBuildState &bs) {
    // We should always be finalized by now.
    assert(finalized);

    NFABuilder &builder = bs.getBuilder();
    position = builder.makePositions(1);

    builder.addCharReach(position, cr);
    builder.setNodeReportID(position, 0 /* offset adj */);
    recordPosBounds(position, position + 1);
}

void AsciiComponentClass::buildFollowSet(GlushkovBuildState &,
                                         const vector<PositionInfo> &) {
    // all follow set construction is handled by firsts/lasts
}

void AsciiComponentClass::add(PredefinedClass c, bool negative) {
    if (in_cand_range) { // can't form a range here
        throw LocatedParseError("Invalid range in character class");
    }
    DEBUG_PRINTF("getting %u %s\n", (u32)c, negative ? "^" : "");

    if (mode.ucp) {
        c = translateForUcpMode(c, mode);
    }

    // Note: caselessness is handled by getPredefinedCharReach.
    CharReach pcr = getPredefinedCharReach(c, mode);
    if (negative) {
        pcr.flip();
    }

    cr |= pcr;
    range_start = INVALID_UNICODE;
    in_cand_range = false;
}

void AsciiComponentClass::add(unichar c) {
    DEBUG_PRINTF("adding \\x%02x\n", c);
    if (c > 0xff) { // too big!
        throw LocatedParseError("Hexadecimal value is greater than \\xFF");
    }

    if (in_cand_range) {
        createRange(c);
        return;
    }

    CharReach ncr(c, c);
    if (mode.caseless) {
        make_caseless(&ncr);
    }

    cr |= ncr;
    range_start = c;
}

void AsciiComponentClass::finalize() {
    if (finalized) {
        return;
    }

    // Handle unclosed ranges, like '[a-]' and '[a-\Q\E]' -- in these cases the
    // dash is a literal dash.
    if (in_cand_range) {
        cr.set('-');
        in_cand_range = false;
    }

    if (m_negate) {
        cr.flip();
    }

    finalized = true;
}

vector<PositionInfo> AsciiComponentClass::first(void) const {
    return vector<PositionInfo>(1, PositionInfo(position));
}

vector<PositionInfo> AsciiComponentClass::last(void) const {
    return vector<PositionInfo>(1, PositionInfo(position));
}

} // namespace ue2
