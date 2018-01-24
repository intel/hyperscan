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

#include "config.h"
#include "gtest/gtest.h"

#include "nfagraph_common.h"

#include "compiler/compiler.h"
#include "grey.h"
#include "nfagraph/ng_builder.h"
#include "nfagraph/ng.h"
#include "nfagraph/ng_asserts.h"
#include "hs_compile.h"
#include "util/ng_find_matches.h"
#include "util/target_info.h"

using namespace std;
using namespace testing;
using namespace ue2;

struct MatchesTestParams {
    string pattern;
    string input;
    vector<pair<size_t, size_t>> matches;
    unsigned flags;
    bool notEod;
    bool som;
};

// teach google-test how to print a param
void PrintTo(const MatchesTestParams &p, ::std::ostream *os) {
    *os << "( \"" << p.pattern << "\", "
        << "\"" << p.input << "\", "
        << "{";
    for (const auto &match : p.matches) {
        *os << "P(" << match.first << ',' << match.second << "),";
    }
    *os << "}, ";
    *os << "flags(" << p.flags << "), "
        << p.notEod << ", "
        << p.som
        << ")\n";
}

class MatchesTest: public TestWithParam<MatchesTestParams> {
};

#define P(x, y) pair<size_t, size_t>((x), (y))

static const MatchesTestParams matchesTests[] = {
    // EOD and anchored patterns

    // these should produce no matches
    { "^foobar", "foolish", {}, 0, false, true},
    { "^foobar$", "ze foobar", {}, 0, false, true},
    { "^foobar$", "foobar ", {}, 0, false, true},
    { "^abc\\b", "abcde", {}, 0, false, true},
    { "^a\\b", "aa", {}, 0, false, true},
    { "^foobar\\b", "foobarz", {}, 0, false, true},
    { "^foobar", "fooq", {}, 0, false, true},
    { "^foobar", "foo", {}, 0, false, true},
    { "^foobar", "fooba", {}, 0, false, true},
    { "^foo *bar", "foolishness bar none ", {}, 0, false, true},
    { "^(abc\\s+dex?y)?(ghij|klmn).*?x?y", "abc p", {}, 0, false, true},
    { "^(abc\\s+dex?y)?(ghij|klmn).*?x?y", "abc dez", {}, 0, false, true},
    { "^(abc\\s+dex?y)?(ghij|klmn).*?x?y", "abc ghi", {}, 0, false, true},
    { "^(abc\\s+dex?y)?(ghij|klmn).*?x?y", "abc hij", {}, 0, false, true},
    { "^(abc\\s+dex?y)?(ghij|klmn).*?x?y", "abc klm", {}, 0, false, true},
    { "^(abc\\s+dex?y)?(ghij|klmn).*?x?y", "abcklmn", {}, 0, false, true},
    { "^.*foobar", "foobaz", {}, 0, false, true},
    { "^.*foobar", "foobaz\n", {}, 0, false, true},
    { "^(foo)|(bar)", "fo baz", {}, 0, false, true},
    { "^((foo)|(bar))", "fo baz", {}, 0, false, true},
    { "aaaaaaaa$", "AAaaAAaa", {}, 0, false, true},
    { "^foo\\z", "foo\n", {}, 0, false, true},
    { "^(foo){2,}", "foo", {}, 0, false, true},

    // these should match
    { "^abc\\B", "abcde", { P(0,3) }, 0, false, true},
    { "^abc\\b", "abc de", { P(0, 3) }, 0, false, true},
    { "^foobar", "foobar", { P(0, 6) }, 0, false, true},
    { "^foobar$", "foobar", { P(0, 6) }, 0, false, true},
    { "^foobar", "foobarq", { P(0, 6) }, 0, false, true},
    { "^foobar\\B", "foobarz", { P(0, 6) }, 0, false, true},
    { "^foo.*bar", "foobar none ", { P(0, 6) }, 0, false, true},
    { "^foo.*bar", "foo bar none ", { P(0, 7) }, 0, false, true},
    { "^foo.*bar", "foo    bar none ", { P(0, 10) }, 0, false, true},
    { "^foo.*bar", "foolishness bar none ", { P(0, 15) }, 0, false, true},
    { "^(abc\\s+dex?y)?(ghij|klmn).*?x?y", "klmny", { P(0, 5) }, 0, false, true},
    { "^(abc\\s+dex?y)?(ghij|klmn).*?x?y", "abc    dexyklmnxy", { P(0, 17) }, 0, false, true},
    { "^.*foobar", "abcfoobar", { P(0, 9) }, 0, false, true},
    { "^((foo)|(bar))", "foobar", { P(0, 3) }, 0, false, true},
    { "^((foo)|(bar))", "foo bar", { P(0, 3) }, 0, false, true},
    { "^(foo)|(bar)", "foobaz", { P(0, 3) }, 0, false, true},
    { "^(foo)|(bar)", "foo baz", { P(0, 3) }, 0, false, true},
    { "^(f[o0]+o)|(bar)", "fo0o baz", { P(0, 4) }, 0, false, true},
    { "aaaaaaaa$", "AAaaAAaa", { P(0, 8) }, HS_FLAG_CASELESS, false, true},
    { "^foo\\z", "foo", { P(0, 3) }, 0, false, true},
    { "^foo\\Z", "foo", { P(0, 3) }, 0, false, true},
    { "^foo\\Z", "foo\n", { P(0, 3) }, 0, false, true},
    { "^(foo){2,}", "foofoofoo", { P(0, 6), P(0, 9)}, 0, false, true},

    // try multiple matches per pattern
    { "^(foo)|(bar)", "foo bar", { P(0, 3), P(4, 7) }, 0, false, true},
    { "^(foo)|(bar)", "foobar", { P(0, 3), P(3, 6) }, 0, false, true},
    { "^(foo)+|(bar)", "foo foobar", { P(0, 3), P(7, 10) }, 0, false, true},
    { "^(foo)+|(bar)", "foofoo bar", { P(0, 3), P(0, 6), P(7, 10) }, 0, false, true},
    { "^(foo)|(bar)", "foobarbaz", { P(0, 3), P(3, 6) }, 0, false, true},
    { "^(f[o0]+o)", "foo0obar", { P(0, 3), P(0, 5) }, 0, false, true},
    { "^(f[o0]+)", "foo0obar", { P(0, 2), P(0, 3), P(0, 4), P(0, 5) }, 0, false, true},

    // unanchored patterns
    { "\\b4\\B", "444", { P(0, 1) }, 0, false, true},
    { "\\b\\w+\\b", "444 555", { P(0, 3), P(4, 7) }, 0, false, true},
    { "foobar", "veryfoolish", {}, 0, false, true},
    { "foo.*bar", "extreme foolishness bar none ", { P(8, 23) }, 0, false, true},
    { "(abc\\s+dex?y)?(ghij|klmn).*?x?y", "abc deyghijfy", { P(0, 13) }, 0, false, true},
    { "(abc\\s+dex?y)?(ghij|klmn).*?x?y", "wegf5tgghij34xy", { P(7, 15) }, 0, false, true},
    { ".*foobar", "verylongfoobaz", {}, 0, false, true},
    { ".*foobar", "foobaz\n", {}, 0, false, true},
    { "(foo)|(bar)", "verylongfo baz", {}, 0, false, true},
    { "(foo)?bar", "foobar", { P(0, 6) }, 0, false, true},
    { "foo?bar", "foobar", { P(0, 6) }, 0, false, true},
    { "(abc)|(bcd)", "abcd", { P(0, 3), P(1, 4) }, 0, false, true},
    { "(abcd)|(bc)", "abcd", { P(0, 4), P(1, 3) }, 0, false, true},
    { "(ab|cd)ef", "abcdef cdabef", { P(2, 6), P(9, 13) }, 0, false, true},
    { "(foo)|(bar)", "verylongfoobbarbaz", { P(8, 11), P(12, 15) }, 0, false, true},
    { "(a[aaaa]aa?((\\B)|[aa])){1,9}", "aaaaa", { P(0, 3), P(0, 4), P(0, 5) }, 0, false, true},
    { "bar\\Z", "foobar\n", { P(3, 6) }, 0, false, true},

    // multi-line patterns
    { "^foo$", "foo\nbar", { P(0, 3) }, HS_FLAG_MULTILINE, false, true},
    { "^bar$", "foo\nbar", { P(4, 7) }, HS_FLAG_MULTILINE, false, true},
    { "^foo$", "big foo\nbar", {}, HS_FLAG_MULTILINE, false, true},
    { "^foo$", "foo\nfoo", { P(0, 3), P(4, 7) }, HS_FLAG_MULTILINE, false, true},
    { "\\bfoo$", "big foo\nbar", { P(4, 7) }, HS_FLAG_MULTILINE, false, true},
    { "\\Bfoo$", "bigfoo\nbar", { P(3, 6) }, HS_FLAG_MULTILINE, false, true},
    { "^foo\\z", "big\nfoo", { P(4, 7) }, HS_FLAG_MULTILINE, false, true},
    { "^foo\\Z", "big\nfoo\n", { P(4, 7) }, HS_FLAG_MULTILINE, false, true},

    // utf8 patterns
    { "ab+", "\x61\x62", { P(0, 2) }, HS_FLAG_UTF8, false, true},
    { "ab.+d", "\x61\x62\xf0\xa4\xad\xa2\x64", { P(0, 7) }, HS_FLAG_UTF8, false, true},

    // noteod patterns
    { "^foobar$", "foobar", {}, 0, true, true},
    { "aaaaaaaa$", "AAaaAAaa", {}, HS_FLAG_CASELESS, true, true},
    { "^foo\\z", "foo", {}, 0, true, true},
    { "^foo\\Z", "foo", {}, 0, true, true},
    { "^foo\\Z", "foo\n", {}, 0, true, true},

    // vacuous patterns (with multiline, utf8 and caseless flags)
    // vacuous patterns have SOM turned off, so all SOM are zero
    { "b*", "abc", { P(0, 0), P(0, 1), P(0, 2), P(0, 3) }, 0, false, false},
    { "b*", "", { P(0, 0) }, 0, false, false},
    { "(aa|b*)", "a", { P(0, 0), P(0, 1) }, 0, false, false},
    { "b*", "bBb", { P(0, 0), P(0, 1), P(0, 2), P(0, 3) }, HS_FLAG_CASELESS, false, false},
    { "b*", "abc", { P(0, 0), P(0, 1), P(0, 2), P(0, 3) }, HS_FLAG_MULTILINE, false, false},
    { "b*", "", { P(0, 0) }, HS_FLAG_MULTILINE, false, false},
    { "(aa|b*)", "a", { P(0, 0), P(0, 1) }, HS_FLAG_MULTILINE, false, false},
    { "b*", "bBb", { P(0, 0), P(0, 1), P(0, 2), P(0, 3) }, HS_FLAG_MULTILINE | HS_FLAG_CASELESS, false, false},
    { "b*", "abc", { P(0, 0), P(0, 1), P(0, 2), P(0, 3) }, HS_FLAG_UTF8, false, false},
    { "b*", "", { P(0, 0) }, HS_FLAG_UTF8, false, false},
    { "(aa|b*)", "a", { P(0, 0), P(0, 1) }, HS_FLAG_UTF8, false, false},
    { "b*", "bBb", { P(0, 0), P(0, 1), P(0, 2), P(0, 3) }, HS_FLAG_UTF8 | HS_FLAG_CASELESS, false, false},
    { "b*", "bBb", { P(0, 0), P(0, 1), P(0, 2), P(0, 3) }, HS_FLAG_UTF8 | HS_FLAG_CASELESS, false, false},
    { ".*", "\x61\xf0\xa4\xad\xa2\x64", { P(0, 0), P(0, 1), P(0, 5), P(0, 6) }, HS_FLAG_UTF8, false, false},
    { ".*", "\xf0\xa4\xad\xa2\xf0\xa4\xad\xa3\x64", { P(0, 0), P(0, 4), P(0, 8), P(0, 9) }, HS_FLAG_UTF8, false, false},

    // special patterns for detecting various bugs
    { "(\\B.|a)*a", "\xf0\xa4\xad\xa2\x61", { P(0, 5) }, HS_FLAG_UTF8, false, true},
    { ".*", "\xf0\xa4\xad", { P(0, 0), P(0, 1), P(0, 2), P(0, 3) }, 0, false, true},
    { "\\Bfoo", "foo", {}, 0, false, true},
    { "fo\\B", "fo_", { P(0, 2)}, 0, false, true},
    { "^.*", "\xee\x80\x80\n\xee\x80\x80", { P(0, 0), P(0, 3), P(0, 4), P(0, 7)}, HS_FLAG_UTF8 | HS_FLAG_MULTILINE, false, false},

    // ignore highlander patterns as they can't be easily checked
};

TEST_P(MatchesTest, Check) {
    const MatchesTestParams &t = GetParam();
    CompileContext cc(false, false, get_current_target(), Grey());
    ReportManager rm(cc.grey);
    ParsedExpression parsed(0, t.pattern.c_str(), t.flags, 0);
    auto built_expr = buildGraph(rm, cc, parsed);
    const auto &g = built_expr.g;
    bool utf8 = (t.flags & HS_FLAG_UTF8) > 0;

    set<pair<size_t, size_t>> matches;
    bool success = findMatches(*g, rm, t.input, matches, 0, 0, t.notEod, utf8);
    ASSERT_TRUE(success);

    set<pair<size_t, size_t>> expected(begin(t.matches), end(t.matches));

    // findMatches returns matches with SOM, so zero them out if not SOM
    if (!t.som) {
        set<pair<size_t, size_t>> new_matches;
        for (auto &m : matches) {
            new_matches.emplace(0, m.second);
        }
        matches.swap(new_matches);
    }

    ASSERT_EQ(expected, matches) << "Pattern '" << t.pattern
                                 << "' against input '" << t.input << "'";
}

INSTANTIATE_TEST_CASE_P(ng_find_matches, MatchesTest, ValuesIn(matchesTests));
