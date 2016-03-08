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

#include "nfagraph_common.h"

#include "gtest/gtest.h"
#include "compiler/compiler.h"
#include "nfagraph/ng.h"
#include "nfagraph/ng_builder.h"
#include "nfagraph/ng_width.h"
#include "grey.h"
#include "util/target_info.h"
#include "util/depth.h"

using namespace std;
using namespace testing;
using namespace ue2;

struct WidthTest {
    string pattern;
    depth minWidth;
    depth maxWidth;
};

class NFAWidthTest : public TestWithParam<WidthTest> { };

static const WidthTest widthTests[] = {
    { "()", 0, 0 },
    { "a", 1, 1 },
    { "a?b", 1, 2 },
    { "foobar", 6, 6 },
    { "foo(bar)?", 3, 6 },
    { "(a|ab|abc|abcd)", 1, 4 },
    { "foo.*bar", 6, depth::infinity() },
    { "foo(bar)*", 3, depth::infinity() },
    { "foo(bar)+", 6, depth::infinity() },
    { "foo(bar){1,3}", 6, 12 },
    { "(abcd)+", 4, depth::infinity() },
    { "foo\\z", 3, 3 },
    { "^foo", 3, 3 },
    { "^foo|bar.*baz", 3, depth::infinity() },
    { "^foobar.*|baz", 3, depth::infinity() },
    { "foo(\\z|bar)", 3, 6 },
    { "foo(|bar\\z)", 3, 6 },
    { "foo.{0,15}bar", 6, 21 },
    { "foo.{0,15}.*bar", 6, depth::infinity() },
    { "(?smi)^(aa[^a]aa$|a|a+\\Z|a)", 1, depth::infinity() }
};

INSTANTIATE_TEST_CASE_P(NFAWidth, NFAWidthTest, ValuesIn(widthTests));

TEST_P(NFAWidthTest, Check) {
    const WidthTest &t = GetParam();
    SCOPED_TRACE(testing::Message() << "Pattern: " << t.pattern);
    unique_ptr<NGWrapper> w(constructGraph(t.pattern, 0));

    ASSERT_EQ(t.minWidth, findMinWidth(*w));
    ASSERT_EQ(t.maxWidth, findMaxWidth(*w));
}

// for google test
void PrintTo(const WidthTest &w, ::std::ostream *os) {
    *os << "WidthTest: " << w.pattern << "{" << w.minWidth << ',';
    if (w.maxWidth == depth::infinity()) {
        *os << "inf";
    } else {
        *os << w.maxWidth;
    }
    *os << '}';
}
