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

#include <limits.h>
#include <vector>

#include "gtest/gtest.h"
#include "hs.h"
#include "test_util.h"

using namespace std;
using namespace testing;

namespace /* anonymous */ {

struct expected_info {
    const char *pattern;
    unsigned min;
    unsigned max;
    char unordered_matches;
    char matches_at_eod;
    char matches_only_at_eod;
};

class ExprInfop : public TestWithParam<expected_info> {
};

TEST_P(ExprInfop, width) {
    const expected_info &ei = GetParam();
    SCOPED_TRACE(ei.pattern);

    hs_expr_info_t *info = nullptr;
    hs_compile_error_t *c_err = nullptr;
    hs_error_t err = hs_expression_info(ei.pattern, 0, &info, &c_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(info != nullptr);
    ASSERT_TRUE(c_err == nullptr);

    EXPECT_EQ(ei.min, info->min_width);
    EXPECT_EQ(ei.max, info->max_width);
    EXPECT_EQ(ei.unordered_matches, info->unordered_matches);
    EXPECT_EQ(ei.matches_at_eod, info->matches_at_eod);
    EXPECT_EQ(ei.matches_only_at_eod, info->matches_only_at_eod);

    free(info);
}

static const expected_info ei_test[] = {
    {"abc", 3, 3, 0, 0, 0},
    {"abc.*def", 6, UINT_MAX, 0, 0, 0},
    {"abc|defghi", 3, 6, 0, 0, 0},
    {"abc(def)?", 3, 6, 0, 0, 0},
    {"abc(def){0,3}", 3, 12, 0, 0, 0},
    {"abc(def){1,4}", 6, 15, 0, 0, 0},
    {"", 0, 0, 0, 0, 0},
    {"^", 0, 0, 0, 0, 0},
    {"^\\b", 0, 0, 1, 0, 0},
    {"\\b$", 0, 0, 1, 1, 1},
    {"(?m)\\b$", 0, 0, 1, 1, 0},
    {"\\A", 0, 0, 0, 0, 0},
    {"\\z", 0, 0, 0, 1, 1},
    {"\\Z", 0, 0, 1, 1, 1},
    {"$", 0, 0, 1, 1, 1},
    {"(?m)$", 0, 0, 1, 1, 0},
    {"^foo", 3, 3, 0, 0, 0},
    {"^foo.*bar", 6, UINT_MAX, 0, 0, 0},
    {"^foo.*bar?", 5, UINT_MAX, 0, 0, 0},
    {"^foo.*bar$", 6, UINT_MAX, 1, 1, 1},
    {"^foobar$", 6, 6, 1, 1, 1},
    {"foobar$", 6, 6, 1, 1, 1},
    {"^.*foo", 3, UINT_MAX, 0, 0, 0},
    {"foo\\b", 3, 3, 1, 1, 0},
    {"foo.{1,13}bar", 7, 19, 0, 0, 0},
    {"foo.{10,}bar", 16, UINT_MAX, 0, 0, 0},
    {"foo.{0,10}bar", 6, 16, 0, 0, 0},
    {"foo.{,10}bar", 12, 12, 0, 0, 0},
    {"foo.{10}bar", 16, 16, 0, 0, 0},
    {"(^|\n)foo", 3, 4, 0, 0, 0},
    {"(^\n|)foo", 3, 4, 0, 0, 0},
    {"(?m)^foo", 3, 3, 0, 0, 0},
    {"\\bfoo", 3, 3, 0, 0, 0},
    {"^\\bfoo", 3, 3, 0, 0, 0},
    {"(?m)^\\bfoo", 3, 3, 0, 0, 0},
    {"\\Bfoo", 3, 3, 0, 0, 0},
    {"(foo|bar\\z)", 3, 3, 0, 1, 0},
    {"(foo|bar)\\z", 3, 3, 0, 1, 1},
};

INSTANTIATE_TEST_CASE_P(ExprInfo, ExprInfop, ValuesIn(ei_test));

}
