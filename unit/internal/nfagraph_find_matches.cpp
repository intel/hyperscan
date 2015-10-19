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

#include "config.h"
#include "gtest/gtest.h"

#include "compiler/compiler.h"
#include "grey.h"
#include "nfagraph/ng_builder.h"
#include "nfagraph/ng.h"
#include "nfagraph/ng_asserts.h"
#include "util/target_info.h"
#include "hs_compile.h"
#include "ng_find_matches.h"

using namespace std;
using namespace testing;
using namespace ue2;

#define NUM_MATCHES 4U
#define P(x,y) pair<size_t, size_t>(x, y)
#define NO_MATCH P(~0U, ~0U)

struct MatchesTestParams {
    string pattern;
    string input;
    // max 4 matches per pattern, P(-1,-1) is "no match"
    pair<size_t, size_t> matches[NUM_MATCHES];
    unsigned flags;
    bool notEod;
    bool som;
};

// teach google-test how to print a param
void PrintTo(const MatchesTestParams &p, ::std::ostream *os) {
    pair<size_t, size_t> *matches = const_cast<pair<size_t, size_t> *>(p.matches);

    *os << "( \"" << p.pattern << "\", "
        << "\"" << p.input << "\", "
        << "{";
    for (int i = 0; i < 4; i++) {
        if (matches[i] == NO_MATCH) {
            *os << "NO_MATCH,";
            break;
        } else {
            *os << "P(" << matches[i].first << ',' << matches[i].second << "),";
        }
    }
    *os << "}, ";
    *os << "flags(" << p.flags << "), "
        << p.notEod << ", "
        << p.som
        << ")\n";
}

class MatchesTest: public TestWithParam<MatchesTestParams> {
};

static const MatchesTestParams matchesTests[] = {
    // EOD and anchored patterns

	// these should produce no matches
    { "^foobar", "foolish", {NO_MATCH}, 0, false, true},
    { "^foobar$", "ze foobar", {NO_MATCH}, 0, false, true},
    { "^foobar$", "foobar ", {NO_MATCH}, 0, false, true},
    { "^abc\\b", "abcde", {NO_MATCH}, 0, false, true},
    { "^a\\b", "aa", {NO_MATCH}, 0, false, true},
    { "^foobar\\b", "foobarz", {NO_MATCH}, 0, false, true},
    { "^foobar", "fooq", {NO_MATCH}, 0, false, true},
    { "^foobar", "foo", {NO_MATCH}, 0, false, true},
    { "^foobar", "fooba", {NO_MATCH}, 0, false, true},
    { "^foo *bar", "foolishness bar none ", {NO_MATCH}, 0, false, true},
    { "^(abc\\s+dex?y)?(ghij|klmn).*?x?y", "abc p", {NO_MATCH}, 0, false, true},
    { "^(abc\\s+dex?y)?(ghij|klmn).*?x?y", "abc dez", {NO_MATCH}, 0, false, true},
    { "^(abc\\s+dex?y)?(ghij|klmn).*?x?y", "abc ghi", {NO_MATCH}, 0, false, true},
    { "^(abc\\s+dex?y)?(ghij|klmn).*?x?y", "abc hij", {NO_MATCH}, 0, false, true},
    { "^(abc\\s+dex?y)?(ghij|klmn).*?x?y", "abc klm", {NO_MATCH}, 0, false, true},
    { "^(abc\\s+dex?y)?(ghij|klmn).*?x?y", "abcklmn", {NO_MATCH}, 0, false, true},
    { "^.*foobar", "foobaz", {NO_MATCH}, 0, false, true},
    { "^.*foobar", "foobaz\n", {NO_MATCH}, 0, false, true},
    { "^(foo)|(bar)", "fo baz", {NO_MATCH}, 0, false, true},
    { "^((foo)|(bar))", "fo baz", {NO_MATCH}, 0, false, true},
    { "aaaaaaaa$", "AAaaAAaa", {NO_MATCH}, 0, false, true},
    { "^foo\\z", "foo\n", {NO_MATCH}, 0, false, true},
    { "^(foo){2,}", "foo", {NO_MATCH}, 0, false, true},

    // these should match
    { "^abc\\B", "abcde", { P(0,3), NO_MATCH }, 0, false, true},
    { "^abc\\b", "abc de", { P(0, 3), NO_MATCH }, 0, false, true},
    { "^foobar", "foobar", { P(0, 6), NO_MATCH }, 0, false, true},
    { "^foobar$", "foobar", { P(0, 6), NO_MATCH }, 0, false, true},
    { "^foobar", "foobarq", { P(0, 6), NO_MATCH }, 0, false, true},
    { "^foobar\\B", "foobarz", { P(0, 6), NO_MATCH }, 0, false, true},
    { "^foo.*bar", "foobar none ", { P(0, 6), NO_MATCH }, 0, false, true},
    { "^foo.*bar", "foo bar none ", { P(0, 7), NO_MATCH }, 0, false, true},
    { "^foo.*bar", "foo    bar none ", { P(0, 10), NO_MATCH }, 0, false, true},
    { "^foo.*bar", "foolishness bar none ", { P(0, 15), NO_MATCH }, 0, false, true},
    { "^(abc\\s+dex?y)?(ghij|klmn).*?x?y", "klmny", { P(0, 5), NO_MATCH }, 0, false, true},
    { "^(abc\\s+dex?y)?(ghij|klmn).*?x?y", "abc    dexyklmnxy", { P(0, 17), NO_MATCH }, 0, false, true},
    { "^.*foobar", "abcfoobar", { P(0, 9), NO_MATCH }, 0, false, true},
    { "^((foo)|(bar))", "foobar", { P(0, 3), NO_MATCH }, 0, false, true},
    { "^((foo)|(bar))", "foo bar", { P(0, 3), NO_MATCH }, 0, false, true},
    { "^(foo)|(bar)", "foobaz", { P(0, 3), NO_MATCH }, 0, false, true},
    { "^(foo)|(bar)", "foo baz", { P(0, 3), NO_MATCH }, 0, false, true},
    { "^(f[o0]+o)|(bar)", "fo0o baz", { P(0, 4), NO_MATCH }, 0, false, true},
    { "aaaaaaaa$", "AAaaAAaa", { P(0, 8), NO_MATCH }, HS_FLAG_CASELESS, false, true},
    { "^foo\\z", "foo", { P(0, 3), NO_MATCH }, 0, false, true},
    { "^foo\\Z", "foo", { P(0, 3), NO_MATCH }, 0, false, true},
    { "^foo\\Z", "foo\n", { P(0, 3), NO_MATCH }, 0, false, true},
    { "^(foo){2,}", "foofoofoo", { P(0, 6), P(0, 9), NO_MATCH}, 0, false, true},

    // try multiple matches per pattern
    { "^(foo)|(bar)", "foo bar", { P(0, 3), P(4, 7), NO_MATCH }, 0, false, true},
    { "^(foo)|(bar)", "foobar", { P(0, 3), P(3, 6), NO_MATCH }, 0, false, true},
    { "^(foo)+|(bar)", "foo foobar", { P(0, 3), P(7, 10), NO_MATCH }, 0, false, true},
    { "^(foo)+|(bar)", "foofoo bar", { P(0, 3), P(0, 6), P(7, 10), NO_MATCH }, 0, false, true},
    { "^(foo)|(bar)", "foobarbaz", { P(0, 3), P(3, 6), NO_MATCH }, 0, false, true},
    { "^(f[o0]+o)", "foo0obar", { P(0, 3), P(0, 5), NO_MATCH }, 0, false, true},
    { "^(f[o0]+)", "foo0obar", { P(0, 2), P(0, 3), P(0, 4), P(0, 5) }, 0, false, true},

    // unanchored patterns
    { "\\b4\\B", "444", { P(0, 1), NO_MATCH }, 0, false, true},
    { "\\b\\w+\\b", "444 555", { P(0, 3), P(4, 7), NO_MATCH }, 0, false, true},
    { "foobar", "veryfoolish", {NO_MATCH}, 0, false, true},
    { "foo.*bar", "extreme foolishness bar none ", { P(8, 23), NO_MATCH }, 0, false, true},
    { "(abc\\s+dex?y)?(ghij|klmn).*?x?y", "abc deyghijfy", { P(0, 13), NO_MATCH }, 0, false, true},
    { "(abc\\s+dex?y)?(ghij|klmn).*?x?y", "wegf5tgghij34xy", { P(7, 15), NO_MATCH }, 0, false, true},
    { ".*foobar", "verylongfoobaz", {NO_MATCH}, 0, false, true},
    { ".*foobar", "foobaz\n", {NO_MATCH}, 0, false, true},
    { "(foo)|(bar)", "verylongfo baz", {NO_MATCH}, 0, false, true},
    { "(foo)?bar", "foobar", { P(0, 6), NO_MATCH }, 0, false, true},
    { "foo?bar", "foobar", { P(0, 6), NO_MATCH }, 0, false, true},
    { "(abc)|(bcd)", "abcd", { P(0, 3), P(1, 4), NO_MATCH }, 0, false, true},
    { "(abcd)|(bc)", "abcd", { P(0, 4), P(1, 3), NO_MATCH }, 0, false, true},
    { "(ab|cd)ef", "abcdef cdabef", { P(2, 6), P(9, 13), NO_MATCH }, 0, false, true},
    { "(foo)|(bar)", "verylongfoobbarbaz", { P(8, 11), P(12, 15), NO_MATCH }, 0, false, true},
    { "(a[aaaa]aa?((\\B)|[aa])){1,9}", "aaaaa", { P(0, 3), P(0, 4), P(0, 5), NO_MATCH }, 0, false, true},
    { "bar\\Z", "foobar\n", { P(3, 6), NO_MATCH }, 0, false, true},

    // multi-line patterns
    { "^foo$", "foo\nbar", { P(0, 3), NO_MATCH }, HS_FLAG_MULTILINE, false, true},
    { "^bar$", "foo\nbar", { P(4, 7), NO_MATCH }, HS_FLAG_MULTILINE, false, true},
    { "^foo$", "big foo\nbar", {NO_MATCH}, HS_FLAG_MULTILINE, false, true},
    { "^foo$", "foo\nfoo", { P(0, 3), P(4, 7), NO_MATCH }, HS_FLAG_MULTILINE, false, true},
    { "\\bfoo$", "big foo\nbar", { P(4, 7), NO_MATCH }, HS_FLAG_MULTILINE, false, true},
    { "\\Bfoo$", "bigfoo\nbar", { P(3, 6), NO_MATCH }, HS_FLAG_MULTILINE, false, true},
    { "^foo\\z", "big\nfoo", { P(4, 7), NO_MATCH }, HS_FLAG_MULTILINE, false, true},
    { "^foo\\Z", "big\nfoo\n", { P(4, 7), NO_MATCH }, HS_FLAG_MULTILINE, false, true},

    // utf8 patterns
    { "ab+", "\x61\x62", { P(0, 2), NO_MATCH }, HS_FLAG_UTF8, false, true},
    { "ab.+d", "\x61\x62\xf0\xa4\xad\xa2\x64", { P(0, 7), NO_MATCH }, HS_FLAG_UTF8, false, true},

    // noteod patterns
    { "^foobar$", "foobar", { NO_MATCH }, 0, true, true},
    { "aaaaaaaa$", "AAaaAAaa", { NO_MATCH }, HS_FLAG_CASELESS, true, true},
    { "^foo\\z", "foo", { NO_MATCH }, 0, true, true},
    { "^foo\\Z", "foo", { NO_MATCH }, 0, true, true},
    { "^foo\\Z", "foo\n", { NO_MATCH }, 0, true, true},

    // vacuous patterns (with multiline, utf8 and caseless flags)
    // vacuous patterns have SOM turned off, so all SOM are zero
    { "b*", "abc", { P(0, 0), P(0, 1), P(0, 2), P(0, 3) }, 0, false, false},
    { "b*", "", { P(0, 0), NO_MATCH }, 0, false, false},
    { "(aa|b*)", "a", { P(0, 0), P(0, 1), NO_MATCH }, 0, false, false},
    { "b*", "bBb", { P(0, 0), P(0, 1), P(0, 2), P(0, 3) }, HS_FLAG_CASELESS, false, false},
    { "b*", "abc", { P(0, 0), P(0, 1), P(0, 2), P(0, 3) }, HS_FLAG_MULTILINE, false, false},
    { "b*", "", { P(0, 0), NO_MATCH }, HS_FLAG_MULTILINE, false, false},
    { "(aa|b*)", "a", { P(0, 0), P(0, 1), NO_MATCH }, HS_FLAG_MULTILINE, false, false},
    { "b*", "bBb", { P(0, 0), P(0, 1), P(0, 2), P(0, 3) }, HS_FLAG_MULTILINE | HS_FLAG_CASELESS, false, false},
    { "b*", "abc", { P(0, 0), P(0, 1), P(0, 2), P(0, 3) }, HS_FLAG_UTF8, false, false},
    { "b*", "", { P(0, 0), NO_MATCH }, HS_FLAG_UTF8, false, false},
    { "(aa|b*)", "a", { P(0, 0), P(0, 1), NO_MATCH }, HS_FLAG_UTF8, false, false},
    { "b*", "bBb", { P(0, 0), P(0, 1), P(0, 2), P(0, 3) }, HS_FLAG_UTF8 | HS_FLAG_CASELESS, false, false},
    { "b*", "bBb", { P(0, 0), P(0, 1), P(0, 2), P(0, 3) }, HS_FLAG_UTF8 | HS_FLAG_CASELESS, false, false},
    { ".*", "\x61\xf0\xa4\xad\xa2\x64", { P(0, 0), P(0, 1), P(0, 5), P(0, 6) }, HS_FLAG_UTF8, false, false},
    { ".*", "\xf0\xa4\xad\xa2\xf0\xa4\xad\xa3\x64", { P(0, 0), P(0, 4), P(0, 8), P(0, 9) }, HS_FLAG_UTF8, false, false},

    // special patterns for detecting various bugs
    { "(\\B.|a)*a", "\xf0\xa4\xad\xa2\x61", { P(0, 5), NO_MATCH }, HS_FLAG_UTF8, false, true},
    { ".*", "\xf0\xa4\xad", { P(0, 0), P(0, 1), P(0, 2), P(0, 3) }, 0, false, true},
    { "\\Bfoo", "foo", {NO_MATCH}, 0, false, true},
    { "fo\\B", "fo_", { P(0, 2), NO_MATCH}, 0, false, true},
    { "^.*", "\xee\x80\x80\n\xee\x80\x80", { P(0, 0), P(0, 3), P(0, 4), P(0, 7)}, HS_FLAG_UTF8 | HS_FLAG_MULTILINE, false, false},

    // ignore highlander patterns as they can't be easily checked
};

// by default, all matches initialize to zeroes. this makes it impossible to
// test vacuous patterns, among other things, unless we specify every single
// match, which is tedious. instead, we set "no match" to NO_MATCH. if the matches
// start with NO_MATCH, everything else is set to NO_MATCH. if matches start
// with something else, look for the next NO_MATCH and replace everything after
// it with NO_MATCH's.
static
void fixMatches(const pair<size_t, size_t> in_matches[],
                pair<size_t, size_t> out_matches[], unsigned size) {
    bool end_matches = false;
    for (unsigned i = 0; i < size; i++) {
        if (in_matches[i] == NO_MATCH) {
            end_matches = true;
        }
        if (end_matches) {
            out_matches[i] = NO_MATCH;
        }
        else {
            out_matches[i] = in_matches[i];
        }
    }
}

TEST_P(MatchesTest, Check) {
    const MatchesTestParams &t = GetParam();
    CompileContext cc(false, false, get_current_target(), Grey());
    ReportManager rm(cc.grey);
    ParsedExpression parsed(0, t.pattern.c_str(), t.flags, 0);
    unique_ptr<NGWrapper> g = buildWrapper(rm, cc, parsed);

    set<pair<size_t, size_t> > results, tmp, matches;
    bool utf8 = (t.flags & HS_FLAG_UTF8) > 0;

    findMatches(*g, rm, t.input, matches, t.notEod, t.som, utf8);

    // fix matches and make a set out of them
    pair<size_t, size_t> tmp_matches[NUM_MATCHES];
    fixMatches(t.matches, tmp_matches, NUM_MATCHES);
    tmp = set<pair<size_t, size_t> >(tmp_matches, tmp_matches + NUM_MATCHES);
    tmp.erase(NO_MATCH);

    // create a superset of pattern results and matches to pick up unexpected
    // matches as well as missed matches.
    results.insert(tmp.begin(), tmp.end());
    results.insert(matches.begin(), matches.end());

    // check if we have the same number of matches as in expected results
    ASSERT_EQ(results.size(), tmp.size())<< "Pattern '" << t.pattern
    << "' against input '" << t.input << "': wrong results count";

    // we already know that size of two sets is the same, so now check matches.
    for (set<pair<size_t, size_t> >::const_iterator it = results.begin();
            it != results.end(); ++it) {
        ASSERT_EQ(1, matches.count(*it))<< "Pattern '" << t.pattern
        << "' against input '" << t.input << "'";
    }
}

INSTANTIATE_TEST_CASE_P(ng_find_matches, MatchesTest, ValuesIn(matchesTests));
