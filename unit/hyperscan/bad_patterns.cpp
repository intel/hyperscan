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
#include "hs.h"

#include <fstream>
#include <string>
#include <boost/algorithm/string/trim.hpp>

#include "util/expressions.h"
#include "util/ExpressionParser.h"

using namespace std;
using namespace testing;

class HyperscanFailBadPattern
    : public TestWithParam<tuple<bool, const char*>> {
protected:
    virtual void SetUp() {
        // compiles a database for this test instantiation
        bool isStreaming;
        tie(isStreaming, pattern) = GetParam();
        err = hs_compile(pattern, 0,
                         isStreaming ? HS_MODE_STREAM : HS_MODE_BLOCK, nullptr,
                         &db, &compile_err);
    }

    virtual void TearDown() {
        hs_free_database(db);
        hs_free_compile_error(compile_err);
    }

    hs_error_t err;
    const char *pattern;
    hs_compile_error_t *compile_err;
    hs_database_t *db;
};

static const char *badPatterns[] = {
    // vacuous patterns
    "",
    "a?",
    "a*",
    "(foo)?",
    "(foo)*(bar)*",
    "^arg|(foo)*(bar)*",
    "foo|bar|",
    "a*(b?)*c*",
    "a{0,3}",
    "[\\s\\S]*",
    "[^\\s\\S]*",
    // too big for our largest NFA
    "(ewh|m?uit|f|snmv.g.gx[yofl]|.[^g][hbd])((.h|((y|vypfw|dfg{4}|x+|o.|y{8,}))+|k{9}t|cgp...gsk+)){17,}",
    // illegal bounds
    "fooa{0}",
    "a{4,3}",
    "a{2,1}",
    // nothing to repeat
    "a++",
    "a+?+",
    "a??",
    "a?+",
    "?qa",
    "*abc",
    "+abc",
    // repeating boundaries is not allowed (UE-1007)
    "^?0",
    "^*0",
    "^+0",
    "^{1,3}0",
    "0$?",
    "0$*",
    "0$+",
    "0${1,3}",
    // zero width asserts ("lookarounds")
    "[a-z]+(?=;)", // positive lookahead
    ".((?!\\x00\\x00)..)*?foo", // negative lookahead
    "(?<=bullock|donkey)", // positive lookbehind
    "(?<!foo)bar", // negative lookbehind
    // embedded anchors
    "foo^bar",
    "foo$bar",
    "(foo^bar|other)",
    "(foo$bar|other)",
    "$test",
    "test^",
    "foo(a|^)bar",
    "a$^b",
    "(^foo)+bar",
    "foo(bar$)+",
    "a^{3}=",
    // atomic groups
    "foobar(?>.{3,})bar",
    // possessive quantifiers
    "\\d++foo",
    "(abc|xyz){2,3}+",
    // back-reference inside a repeat (also too big, actually)
    "^..\x02.{10,522}([^\00])\1{16}",
    // char classes
    "[]",
    "[]foobar",
    "[`-\\80",
    "[A-\\K]",
    // bad named classes
    "[[:foo:]]",
    "[[:1234:]]",
    "[[:f\\oo:]]",
    "[[: :]]",
    "[[:...:]]",
    "[[:l\\ower:]]",
    "[[:abc\\:]]",
    "[abc[:x\\]pqr:]]",
    "[[:a\\dz:]]",
    // unhandled subroutines and backrefs
    "foo\\g''bar",
    "foo\\g'45'bar",
    "foo\\g'hatstand'bar",
    "foo\\g<>bar",
    "foo\\g<45>bar",
    "foo\\g<teakettle>bar",
    "((?i)rah)\\s+\\1",
    "(?<p1>(?i)rah)\\s+\\k<p1>",
    "(?'p1'(?i)rah)\\s+\\k{p1}",
    "(?P<p1>(?i)rah)\\s+(?P=p1)",
    "(?<p1>(?i)rah)\\s+\\g{p1}",
    // truly enormous and with complicated assert resolution (UE-1107)
    "((c(p|p)h{2,}bh.|p|((((cq|j|c|(\\b)|.[^nbgn]|(\\B)[qfh]a)){10,12}|ih|a|mnde[pa].|.g)){5,8})){21,29}",
    // conditional subpatterns
    "(?(?=[^a-z]*[a-z])\\d{2}-[a-z]{3}-\\d{2}|\\d{2}-\\d{2}-\\d{2)}",
    // unmatched parens
    "(foo",
    "foo)",
    "((foo)",
    "(foo))",
    // unterminated comment
    "/foo(?#comment/",
    // bogus \g backrefs
    "A\\g",
    "A(.*)\\ga",
    // malformed \g backrefs (see UE-950)
    "^(a)\\g",
    "^(a)\\g{3",
    "\\g{A",
    "[\\g6666666666]",
    "(^(a|b\\g<-1'c))",
    // oniguruma subroutine calls (UE-950 as well)
    "^(?<name>a|b\\g'name'c)",
    "^(a|b\\g'1'c)",
    "^(a|b\\g'-1'c)",
    // backtracking control verbs
    "A((?:A|B(*ACCEPT)|C)D)",
    "(*FAIL)",
    "(*F)",
    "a+(*COMMIT)b",
    "(*PRUNE)",
    "a+(*SKIP)b",
    // other unsupported PCRE features
    "\\R",
    "foo\\Kbar",
    "\\Gfoo",
    "(?|(Sat)ur|(Sun))day", // duplicate subpatterns, see UE-958
    "foobar\\", // trailing unescaped backslash
    "(?x)abc(#i)def" // unterminated extended-mode comment
};

// Did we correctly fail the compile?
TEST_P(HyperscanFailBadPattern, Compile) {
    ASSERT_NE(HS_SUCCESS, err) << "Compile should have failed for expr: " << pattern;
    EXPECT_EQ(HS_COMPILER_ERROR, err);
    EXPECT_TRUE(db == nullptr);
    EXPECT_TRUE(compile_err != nullptr);
    // We shouldn't fail with the following messagess
    EXPECT_STRNE("An invalid flag was specified.", compile_err->message);
    EXPECT_STRNE("Unable to allocate memory.", compile_err->message);
    EXPECT_STRNE("Internal error.", compile_err->message);
    EXPECT_STRNE("Match can be raised on EOD", compile_err->message);
}

INSTANTIATE_TEST_CASE_P(CompileBadPatterns,
                        HyperscanFailBadPattern,
                        Combine(Bool(), ValuesIn(badPatterns)));

struct BadPatternParam {
    BadPatternParam(const string &expr_in, unsigned int flags_in,
                    const hs_expr_ext &ext_in,
                    const string &expected_error_in)
        : expr(expr_in), flags(flags_in), ext(ext_in),
          expected_error(expected_error_in) {}
    string expr;
    unsigned int flags;
    hs_expr_ext ext;
    string expected_error;

    // Wrap hs_compile_ext_multi for single patterns.
    hs_error_t compile(unsigned int mode, hs_database_t **db,
                       hs_compile_error_t **compile_err) const {
        const char *regex = expr.c_str();
        const hs_expr_ext *extp = &ext;
        return hs_compile_ext_multi(&regex, &flags, nullptr, &extp, 1,
                                    mode, nullptr, db, compile_err);
    }
};

void PrintTo(const BadPatternParam &p, ::std::ostream *os) {
    *os << "expr: \"" << p.expr << "\", expected error: " << p.expected_error;
}

class BadPattern : public TestWithParam<BadPatternParam> {
};

TEST_P(BadPattern, Block) {
    const BadPatternParam &p = GetParam();
    SCOPED_TRACE(p.expr);

    hs_compile_error_t *compile_err;
    hs_database_t *db;
    hs_error_t err = p.compile(HS_MODE_NOSTREAM, &db, &compile_err);
    EXPECT_EQ(HS_COMPILER_ERROR, err);
    EXPECT_TRUE(db == nullptr);
    EXPECT_TRUE(compile_err != nullptr);
    if (compile_err) {
        EXPECT_STREQ(p.expected_error.c_str(), compile_err->message);
    }

    hs_free_database(db);
    hs_free_compile_error(compile_err);
}

TEST_P(BadPattern, Stream) {
    const BadPatternParam &p = GetParam();
    SCOPED_TRACE(p.expr);

    hs_compile_error_t *compile_err;
    hs_database_t *db;
    hs_error_t err = p.compile(HS_MODE_STREAM | HS_MODE_SOM_HORIZON_LARGE, &db,
                               &compile_err);
    EXPECT_EQ(HS_COMPILER_ERROR, err);
    EXPECT_TRUE(db == nullptr);
    EXPECT_TRUE(compile_err != nullptr);
    if (compile_err) {
        EXPECT_STREQ(p.expected_error.c_str(), compile_err->message);
    }

    hs_free_database(db);
    hs_free_compile_error(compile_err);
}

// happy fun preprocessor hoop jumping
#define xstr(s) str(s)
#define str(s) #s

#define SRCDIR_PREFIX xstr(SRCDIR)

static
vector<BadPatternParam> getBadPatterns() {
    string filename = "unit/hyperscan/bad_patterns.txt";

    ifstream f;
    f.open(filename.c_str(), ifstream::in);
    if (!f.good()) {
        // try it with the src prefix
        f.open((string(SRCDIR_PREFIX) + "/" + filename).c_str(), ifstream::in);
    }

    vector<BadPatternParam> rv;
    if (!f.good()) {
        string expr("couldn't find input file:" + filename);
        cerr << expr << endl;
        abort();
        return rv;
    }

    string line;
    while (f.good()) {
        getline(f, line);
        if (line.empty()) {
            continue;
        }

        size_t hashIdx = line.find_first_of('#');
        size_t colonIdx = line.find_first_of(':');
        assert(hashIdx != string::npos);
        assert(colonIdx != string::npos);
        if (!hashIdx) {
            continue;
        }

        string error = line.substr(hashIdx + 1);
        string expr = line.substr(colonIdx + 1, hashIdx - colonIdx - 1);
        boost::trim(expr);

        unsigned int flags;
        string regex;
        hs_expr_ext ext;
        if (!readExpression(expr, regex, &flags, &ext)) {
            cerr << expr << " failed in readExpression" << endl;
            abort();
        }
        rv.push_back(BadPatternParam(regex, flags, ext, error));
    }
    f.close();

    return rv;
}

INSTANTIATE_TEST_CASE_P(Hyperscan, BadPattern, ValuesIn(getBadPatterns()));

TEST(ResourceLimits, longPattern) {
#define LONG_PATTERN_LEN 16384

    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;

    char pattern[LONG_PATTERN_LEN];
    memset(pattern, 'a', LONG_PATTERN_LEN);
    pattern[LONG_PATTERN_LEN - 1] = 0;

    hs_error_t err = hs_compile(pattern, HS_FLAG_DOTALL, HS_MODE_BLOCK, nullptr,
                                &db, &compile_err);

    EXPECT_EQ(HS_COMPILER_ERROR, err);
    EXPECT_TRUE(db == nullptr);
    EXPECT_TRUE(compile_err != nullptr);
    if (compile_err) {
        EXPECT_STREQ("Pattern length exceeds limit.", compile_err->message);
    }

    hs_free_database(db);
    hs_free_compile_error(compile_err);
}

TEST(ResourceLimits, longPatternInfo) {
#define LONG_PATTERN_LEN 16384

    hs_expr_info_t *info = nullptr;
    hs_compile_error_t *compile_err = nullptr;

    char pattern[LONG_PATTERN_LEN];
    memset(pattern, 'a', LONG_PATTERN_LEN);
    pattern[LONG_PATTERN_LEN - 1] = 0;

    hs_error_t err =
        hs_expression_info(pattern, HS_FLAG_DOTALL, &info, &compile_err);

    EXPECT_EQ(HS_COMPILER_ERROR, err);
    EXPECT_TRUE(compile_err != nullptr);
    if (compile_err) {
        EXPECT_STREQ("Pattern length exceeds limit.", compile_err->message);
    }

    free(info);
    hs_free_compile_error(compile_err);
}

TEST(ResourceLimits, longLiteral) {
#define LONG_PATTERN_LEN 16384

    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;

    const char *pattern = "(abcd){4096}";

    hs_error_t err = hs_compile(pattern, HS_FLAG_DOTALL, HS_MODE_BLOCK, nullptr,
                                &db, &compile_err);

    EXPECT_EQ(HS_COMPILER_ERROR, err);
    EXPECT_TRUE(db == nullptr);
    EXPECT_TRUE(compile_err != nullptr);
    if (compile_err) {
        EXPECT_STREQ("Resource limit exceeded.", compile_err->message);
    }

    hs_free_database(db);
    hs_free_compile_error(compile_err);
}
