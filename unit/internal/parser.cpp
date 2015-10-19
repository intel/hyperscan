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
#include "parser/Component.h"
#include "parser/Parser.h"
#include "parser/dump.h"
#include "parser/prefilter.h"
#include "parser/unsupported.h"
#include "hs.h"

#include <sstream>

using namespace ue2;
using std::ostringstream;
using std::unique_ptr;

struct PatternInfo {
    const char *expr;
    unsigned int flags;
};

class ParserTest : public testing::TestWithParam<PatternInfo> {};

TEST_P(ParserTest, clone) {
    const PatternInfo &info = GetParam();
    SCOPED_TRACE(info.expr);

    // This tests that we can parse expr and then clone it, then verifies that
    // the cloned component tree matches the original (using dump code for
    // serialisation)

    ParseMode mode(info.flags);
    unique_ptr<Component> c(parse(info.expr, mode));
    ASSERT_TRUE(c != nullptr);

#if defined(DUMP_SUPPORT)
    ostringstream oss;
    dumpTree(oss, c.get());
#endif

    unique_ptr<Component> c_copy(c->clone());
    ASSERT_TRUE(c_copy != nullptr);

    c.reset(); // free original c

#if defined(DUMP_SUPPORT)
    ostringstream oss_copy;
    dumpTree(oss_copy, c_copy.get());
    ASSERT_EQ(oss.str(), oss_copy.str());
#endif
}

TEST_P(ParserTest, prefilter) {
    const PatternInfo &info = GetParam();
    SCOPED_TRACE(info.expr);

    ParseMode mode(info.flags);
    unique_ptr<Component> c(parse(info.expr, mode));
    ASSERT_TRUE(c != nullptr);

    prefilterTree(c, mode);
    ASSERT_TRUE(c != nullptr);

    // Should now be supported.
    ASSERT_NO_THROW(checkUnsupported(*c));
}

static const PatternInfo patterns[] = {
    { "easy literal", 0 },
    { "foo|bar|baz", 0 },
    { "(?>\\d+)foo", 0 },
    { "\\bhello\\b", 0 },
    { "a+ b+? c++", 0 },
    { "^bi-anchored$", 0 },
    { "\\C", 0 },
    { "\\X",  HS_FLAG_UTF8 },
    { "\\w+", HS_FLAG_UTF8 | HS_FLAG_UCP },
    { "foo(?!bar).*baz", 0 },
    { "(hatstand.*teakettle){2,} badgerbrush(es)? \\1", 0 },
    { "a (pine)?(?(1)apple)", 0 },
    { "^(?(?=hello)[a-z]+|[a-z]{3})", 0 },
};

INSTANTIATE_TEST_CASE_P(Parser, ParserTest, testing::ValuesIn(patterns));

// teach google-test how to print a param
void PrintTo(const PatternInfo &p, ::std::ostream *os) {
    *os << p.expr << ":" << p.flags;
}
