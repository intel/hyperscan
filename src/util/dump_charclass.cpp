/*
 * Copyright (c) 2015-2018, Intel Corporation
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
 * \brief Dump code for character classes (expressed as CharReach objects).
 */

#include "config.h"

// Everything in this file is dump code
#if defined(DUMP_SUPPORT)

#include "charreach.h"
#include "dump_charclass.h"

#include <cctype>
#include <iomanip>
#include <ostream>
#include <sstream>

using std::string;
using std::ostream;

namespace ue2 {

static
void describeChar(ostream &os, char c, enum cc_output_t out_type) {
    // these characters must always be escaped
    static const string escaped("^-[].");
    static const string dot_single_escaped("\"\'");

    const string backslash((out_type == CC_OUT_DOT ? 2 : 1), '\\');

#ifdef _WIN32
    if (c >= 0x21 && c < 0x7F && c != '\\') {
#else
    if (isgraph(c) && c != '\\') {
#endif
        if (escaped.find(c) != string::npos) {
            os << backslash << c;
        } else if (out_type == CC_OUT_DOT
                   && dot_single_escaped.find(c) != string::npos) {
            os << '\\' << c;
        } else {
            os << c;
        }
    } else if (c == 0x09) {
        os << backslash << 't';
    } else if (c == 0x0a) {
        os << backslash << 'n';
    } else if (c == 0x0d) {
        os << backslash << 'r';
    } else {
        auto fmt(os.flags());
        os << backslash << 'x' << std::hex << std::setw(2)
           << std::setfill('0') << (unsigned)(c & 0xff);
        os.flags(fmt);
    }
}

static
void describeRange(ostream &os, unsigned char c1, unsigned char c2, enum cc_output_t out_type) {
    assert(c1 <= c2);
    if (c1 == c2) {
        describeChar(os, (char)c1, out_type);
    }
    else if (c2 - c1 < 4) {
        // render as individual chars
        do {
            describeChar(os, (char)c1, out_type);
        } while (c1++ != c2);
    } else {
        // range
        describeChar(os, (char)c1, out_type);
        os << '-';
        describeChar(os, (char)c2, out_type);
    }
}

static
bool extractMnemonic(ostream &os, CharReach &cr, enum cc_output_t out_type) {
    const string backslash((out_type == CC_OUT_DOT ? 2 : 1), '\\');

    // \w (word characters: any letter, digit, or underscore)
    static const CharReach words(string("_0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWYXZ"));
    if (words == (cr & words)) {
        cr &= ~words;
        os << backslash << 'w';
        return true;
    }

    // \d (digits)
    static const CharReach digits(string("0123456789"));
    if (digits == (cr & digits)) {
        cr &= ~digits;
        os << backslash << 'd';
        return true;
    }

    // \s (whitespace)
    static const CharReach whitespace(string("\x09\x0a\x0b\x0c\x0d\x20", 6));
    if (whitespace == (cr & whitespace)) {
        cr &= ~whitespace;
        os << backslash << 's';
        return true;
    }

    return false;
}

static
bool isContiguous(const CharReach &cr, size_t &first, size_t &last) {
    first = cr.find_first();
    size_t c = first;
    while (c != CharReach::npos) {
        last = c;
        c = cr.find_next(c);
        if (c == CharReach::npos) {
            break;
        }
        if (c != last + 1) {
            return false;
        }
    }
    return true;
}

static
size_t describeClassInt(ostream &os, const CharReach &incr, size_t maxLength,
        enum cc_output_t out_type) {

    // Approx size of output
    size_t i = 0;
    // one we can break
    CharReach cr(incr);

    // If we can be rendered as a single range, do it
    size_t first = 0, last = 0;
    if (isContiguous(cr, first, last)) {
        describeRange(os, first, last, out_type);
        i = 2;
        return i;
    }

    // Extract any mnemonics
    while (extractMnemonic(os, cr, out_type)) {
        if (++i == maxLength) {
            os << "...]";
            return maxLength;
        }
    }

    if (cr.none()) {
        // all mnemonics, all the time
        return i;
    }

    // Render charclass as a series of ranges
    size_t c_start = cr.find_first();
    size_t c = c_start, c_last = 0;
    while (c != CharReach::npos) {
        c_last = c;
        c = cr.find_next(c);
        if (c != c_last + 1 || c_last == 0xff) {
            describeRange(os, c_start, c_last, out_type);
            c_start = c;
            if (++i == maxLength && c != CharReach::npos) {
                os << "...]";
                return maxLength;
            }
        }
    }
    return i;
}

////
//// Functions exported outside this unit.
////

// C++ iostreams interface
void describeClass(ostream &os, const CharReach &incr, size_t maxLength,
                   enum cc_output_t out_type) {
    if (incr.all()) {
        os << "<any>";
        return;
    }

    if (incr.none()) {
        os << "<empty>";
        return;
    }

    if (incr.count() == 1) {
        describeChar(os, (char)incr.find_first(), out_type);
        return;
    }
    if ((~incr).count() == 1) {
        os << "[^";
        describeChar(os, (char)(~incr).find_first(), out_type);
        os << ']';
        return;
    }

    // build up a normal string and a negated one, and see which is shorter
    std::ostringstream out;
    int out_count = describeClassInt(out, incr, maxLength, out_type);

    std::ostringstream neg;
    UNUSED int neg_count = describeClassInt(neg, ~incr, maxLength, out_type);

    if (out.tellp() <= neg.tellp()) {
        if (out_count > 1) {
            os << '[' << out.str() << ']';
        } else {
            os << out.str();
        }
    } else {
        // TODO: negated single mnemonics
        os << "[^" << neg.str() << ']';
    }
}

// Version that returns a string, for convenience.
string describeClass(const CharReach &cr, size_t maxLength,
                     enum cc_output_t out_type) {
    std::ostringstream oss;
    describeClass(oss, cr, maxLength, out_type);
    return oss.str();
}

// C stdio wrapper
void describeClass(FILE *f, const CharReach &cr, size_t maxLength,
                   enum cc_output_t out_type) {
    fprintf(f, "%s", describeClass(cr, maxLength, out_type).c_str());
}

} // namespace ue2

#endif // DUMP_SUPPORT
