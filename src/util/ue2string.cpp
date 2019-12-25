/*
 * Copyright (c) 2015-2019, Intel Corporation
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
 * \brief Tools for string manipulation, ue2_literal definition.
 */

#include "ue2string.h"

#include "charreach.h"
#include "compare.h"
#include "hash_dynamic_bitset.h"

#include <algorithm>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <string>

using namespace std;

namespace ue2 {

#if defined(DUMP_SUPPORT) || defined(DEBUG)

// Escape a string so that it's screen-printable
string escapeString(const string &s) {
    ostringstream os;
    for (unsigned int i = 0; i < s.size(); ++i) {
        char c = s[i];
        if (0x20 <= c && c <= 0x7e && c != '\\') {
            os << c;
        } else if (c == '\n') {
            os << "\\n";
        } else if (c == '\r') {
            os << "\\r";
        } else if (c == '\t') {
            os << "\\t";
        } else {
            os << "\\x" << hex << setw(2) << setfill('0')
                << (unsigned)(c & 0xff) << dec;
        }
    }
    return os.str();
}

string escapeString(const ue2_literal &lit) {
    ostringstream os;
    for (ue2_literal::const_iterator it = lit.begin(); it != lit.end(); ++it) {
        char c = it->c;
        if (0x20 <= c && c <= 0x7e && c != '\\') {
            os << c;
        } else if (c == '\n') {
            os << "\\n";
        } else {
            os << "\\x" << hex << setw(2) << setfill('0')
                << (unsigned)(c & 0xff) << dec;
        }
    }
    return os.str();
}

// escape any metacharacters in a literal string
string escapeStringMeta(const string &s) {
    ostringstream os;
    for (unsigned int i = 0; i < s.size(); ++i) {
        char c = s[i];
        switch (c) {
            case '#': case '$': case '(': case ')':
            case '*': case '+': case '.': case '/':
            case '?': case '[': case ']': case '^':
            case '|':
                os << "\\" << c; break;
            default:
                os << c; break;
        }
    }
    return os.str();
}

string dotEscapeString(const string &s) {
    string ss = escapeString(s);
    string out;
    out.reserve(ss.size());
    for (size_t i = 0; i != ss.size(); i++) {
        char c = ss[i];
        switch (c) {
        case '\"':
        case '\\':
            out.push_back('\\');
        // fall through
        default:
            out.push_back(c);
            break;
        }
    }
    return out;
}

string dumpString(const ue2_literal &lit) {
    string s = escapeString(lit.get_string());
    if (lit.any_nocase()) {
        s += " (nocase)";
    }

    return s;
}
#endif

void upperString(string &s) {
    for (auto &c : s) {
        c = mytoupper(c);
    }
}

size_t maxStringOverlap(const string &a, const string &b, bool nocase) {
    size_t lena = a.length(), lenb = b.length();
    const char *astart = a.c_str();
    const char *bstart = b.c_str();
    const char *aend = astart + lena;
    size_t i = lenb;

    for (; i > lena; i--) {
        if (!cmp(astart, bstart + i - lena, lena, nocase)) {
            return i;
        }
    }

    for (; i && cmp(aend - i, bstart, i, nocase); i--) {
        ;
    }

    return i;
}

size_t maxStringOverlap(const ue2_literal &a, const ue2_literal &b) {
    /* todo: handle nocase better */
    return maxStringOverlap(a.get_string(), b.get_string(),
                            a.any_nocase() || b.any_nocase());
}

size_t maxStringSelfOverlap(const string &a, bool nocase) {
    size_t lena = a.length();
    const char *astart = a.c_str();
    const char *bstart = a.c_str();
    const char *aend = astart + lena;
    size_t i = lena - 1;

    for (; i && cmp(aend - i, bstart, i, nocase); i--) {
        ;
    }

    return i;
}

u32 cmp(const char *a, const char *b, size_t len, bool nocase) {
    if (!nocase) {
        return memcmp(a, b, len);
    }

    for (const auto *a_end = a + len; a < a_end; a++, b++) {
        if (mytoupper(*a) != mytoupper(*b)) {
            return 1;
        }
    }
    return 0;
}

case_iter::case_iter(const ue2_literal &ss) : s(ss.get_string()),
                                              s_orig(ss.get_string()) {
    for (ue2_literal::const_iterator it = ss.begin(); it != ss.end(); ++it) {
        nocase.push_back(it->nocase);
    }
}

case_iter caseIterateBegin(const ue2_literal &s) {
    return case_iter(s);
}

case_iter caseIterateEnd() {
    return case_iter(ue2_literal());
}

case_iter &case_iter::operator++ () {
    for (size_t i = s.length(); i != 0; i--) {
        char lower = mytolower(s[i - 1]);
        if (nocase[i - 1] && lower != s[i - 1]) {
            s[i - 1] = lower;
            copy(s_orig.begin() + i, s_orig.end(), s.begin() + i);
            return *this;
        }
    }

    s.clear();
    return *this;
}

static
string toUpperString(string s) {
    upperString(s);
    return s;
}

ue2_literal::elem::operator CharReach () const {
    if (!nocase) {
        return CharReach(c);
    } else {
        CharReach rv;
        rv.set(mytoupper(c));
        rv.set(mytolower(c));
        return rv;
    }
}

const ue2_literal::size_type ue2_literal::npos = std::string::npos;

ue2_literal::ue2_literal(const std::string &s_in, bool nc_in)
    : s(nc_in ? toUpperString(s_in) : s_in), nocase(s_in.size()) {
    if (nc_in) {
        // Switch on nocase bit for all alpha characters.
        for (size_t i = 0; i < s.length(); i++) {
            if (ourisalpha(s[i])) {
                nocase.set(i);
            }
        }
    }
}

ue2_literal::ue2_literal(char c, bool nc)
    : s(1, nc ? mytoupper(c) : c), nocase(1, ourisalpha(c) ? nc : false) {}

ue2_literal ue2_literal::substr(size_type pos, size_type n) const {
    ue2_literal rv;
    rv.s = s.substr(pos, n);
    size_type upper = nocase.size();
    if (n != npos && n + pos < nocase.size()) {
        upper = n + pos;
    }

    rv.nocase.resize(upper - pos, false);
    for (size_t i = pos; i < upper; i++) {
        rv.nocase.set(i - pos, nocase.test(i));
    }
    assert(s.size() == nocase.size());
    return rv;
}

ue2_literal &ue2_literal::erase(size_type pos, size_type n) {
    s.erase(pos, n);

    if (n != npos) {
        for (size_type i = pos + n; i < nocase.size(); i++) {
            nocase.set(i - n, nocase.test(i));
        }
    }
    nocase.resize(s.size());
    return *this;
}

void ue2_literal::push_back(char c, bool nc) {
    if (nc) {
        c = mytoupper(c);
    }
    nocase.push_back(nc);
    s.push_back(c);
}

void ue2_literal::reverse() {
    std::reverse(s.begin(), s.end());

    const size_t len = nocase.size();
    for (size_t i = 0; i < len / 2; i++) {
        size_t j = len - i - 1;
        bool a = nocase.test(i);
        bool b = nocase.test(j);
        nocase.set(i, b);
        nocase.set(j, a);
    }
}

// Return a copy of this literal in reverse order.
ue2_literal reverse_literal(const ue2_literal &in) {
    auto out = in;
    out.reverse();
    return out;
}

bool ue2_literal::operator<(const ue2_literal &b) const {
    if (s < b.s) {
        return true;
    }
    if (s > b.s) {
        return false;
    }
    return nocase < b.nocase;
}

void ue2_literal::operator+=(const ue2_literal &b) {
    s += b.s;
    size_t prefix = nocase.size();
    nocase.resize(prefix + b.nocase.size());
    for (size_t i = 0; i < b.nocase.size(); i++) {
        nocase.set(prefix + i, b.nocase[i]);
    }
}

bool ue2_literal::any_nocase() const {
    return nocase.any();
}

size_t ue2_literal::hash() const {
    return hash_all(s, hash_dynamic_bitset()(nocase));
}

void make_nocase(ue2_literal *lit) {
    ue2_literal rv;

    for (const auto &elem: *lit) {
        rv.push_back(elem.c, ourisalpha(elem.c));
    }

    lit->swap(rv);
}

static
bool testchar(char c, const CharReach &cr, bool nocase) {
    if (nocase) {
        return cr.test((unsigned char)mytolower(c))
            || cr.test((unsigned char)mytoupper(c));
    } else {
        return cr.test((unsigned char)c);
    }
}

// Returns true if the given literal contains a char in the given CharReach
bool contains(const ue2_literal &s, const CharReach &cr) {
    for (ue2_literal::const_iterator it = s.begin(), ite = s.end();
         it != ite; ++it) {
        if (testchar(it->c, cr, it->nocase)) {
            return true;
        }
    }
    return false;
}

size_t maxStringSelfOverlap(const ue2_literal &a) {
    /* overly conservative if only part of the string is nocase, TODO: fix */
    return maxStringSelfOverlap(a.get_string(), a.any_nocase());
}

size_t minStringPeriod(const ue2_literal &a) {
    return a.length() - maxStringSelfOverlap(a);
}

// Returns true if `a' is a suffix of (or equal to) `b'.
bool isSuffix(const ue2_literal &a, const ue2_literal &b) {
    size_t alen = a.length(), blen = b.length();
    if (alen > blen) {
        return false;
    }
    return equal(a.begin(), a.end(), b.begin() + (blen - alen));
}

bool is_flood(const ue2_literal &s) {
    assert(!s.empty());

    ue2_literal::const_iterator it = s.begin(), ite = s.end();
    ue2_literal::elem f = *it;
    for (++it; it != ite; ++it) {
        if (*it != f) {
            return false;
        }
    }

    return true;
}

} // namespace ue2
