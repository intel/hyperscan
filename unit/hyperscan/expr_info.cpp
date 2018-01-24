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
    hs_expr_ext ext;

    unsigned min;
    unsigned max;
    char unordered_matches;
    char matches_at_eod;
    char matches_only_at_eod;
};

ostream& operator<<(ostream &os, const hs_expr_ext &ext) {
    if (!ext.flags) {
        return os;
    }
    bool first = true;
    if (ext.flags & HS_EXT_FLAG_MIN_OFFSET) {
        if (!first) {
            os << ", ";
        }
        os << "min_offset=" << ext.min_offset;
        first = false;
    }
    if (ext.flags & HS_EXT_FLAG_MAX_OFFSET) {
        if (!first) {
            os << ", ";
        }
        os << "max_offset=" << ext.max_offset;
        first = false;
    }
    if (ext.flags & HS_EXT_FLAG_MIN_LENGTH) {
        if (!first) {
            os << ", ";
        }
        os << "min_length=" << ext.min_length;
        first = false;
    }
    if (ext.flags & HS_EXT_FLAG_EDIT_DISTANCE) {
        if (!first) {
            os << ", ";
        }
        os << "edit_distance=" << ext.edit_distance;
        first = false;
    }
    if (ext.flags & HS_EXT_FLAG_HAMMING_DISTANCE) {
        if (!first) {
            os << ", ";
        }
        os << "hamming_distance=" << ext.hamming_distance;
        first = false;
    }
    return os;
}

// For Google Test.
void PrintTo(const expected_info &ei, ostream *os) {
    *os << "expected_info: "
        << "pattern=\"" << ei.pattern << "\""
        << ", ext={" << ei.ext << "}"
        << ", min=" << ei.min << ", max=" << ei.max
        << ", unordered_matches=" << (ei.unordered_matches ? 1 : 0)
        << ", matches_at_eod=" << (ei.matches_at_eod ? 1 : 0)
        << ", matches_only_at_eod=" << (ei.matches_only_at_eod ? 1 : 0);
}

class ExprInfop : public TestWithParam<expected_info> {
};

static
void check_info(const expected_info &ei, const hs_expr_info_t *info) {
    EXPECT_EQ(ei.min, info->min_width);
    EXPECT_EQ(ei.max, info->max_width);
    EXPECT_EQ(ei.unordered_matches, info->unordered_matches);
    EXPECT_EQ(ei.matches_at_eod, info->matches_at_eod);
    EXPECT_EQ(ei.matches_only_at_eod, info->matches_only_at_eod);
}

// Check with hs_expression_info function.
TEST_P(ExprInfop, check_no_ext) {
    const expected_info &ei = GetParam();
    SCOPED_TRACE(ei.pattern);

    if (ei.ext.flags) {
        // This is an extparam test, skip it.
        return;
    }

    hs_expr_info_t *info = nullptr;
    hs_compile_error_t *c_err = nullptr;
    hs_error_t err = hs_expression_info(ei.pattern, 0, &info, &c_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(info != nullptr);
    ASSERT_TRUE(c_err == nullptr);

    check_info(ei, info);
    free(info);
}

// Check with hs_expression_ext_info function.
TEST_P(ExprInfop, check_ext) {
    const expected_info &ei = GetParam();
    SCOPED_TRACE(ei.pattern);

    hs_expr_info_t *info = nullptr;
    hs_compile_error_t *c_err = nullptr;
    hs_error_t err =
        hs_expression_ext_info(ei.pattern, 0, &ei.ext, &info, &c_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(info != nullptr);
    ASSERT_TRUE(c_err == nullptr);

    check_info(ei, info);
    free(info);
}

// Check with hs_expression_ext_info function and a nullptr ext param, for
// cases where ext.flags == 0. Functionally identical to check_no_ext above.
TEST_P(ExprInfop, check_ext_null) {
    const expected_info &ei = GetParam();
    SCOPED_TRACE(ei.pattern);

    if (ei.ext.flags) {
        // This is an extparam test, skip it.
        return;
    }

    hs_expr_info_t *info = nullptr;
    hs_compile_error_t *c_err = nullptr;
    hs_error_t err =
        hs_expression_ext_info(ei.pattern, 0, nullptr, &info, &c_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(info != nullptr);
    ASSERT_TRUE(c_err == nullptr);

    check_info(ei, info);
    free(info);
}

static const hs_expr_ext NO_EXT_PARAM = { 0, 0, 0, 0, 0, 0 };

static const expected_info ei_test[] = {
    {"abc", NO_EXT_PARAM, 3, 3, 0, 0, 0},
    {"abc.*def", NO_EXT_PARAM, 6, UINT_MAX, 0, 0, 0},
    {"abc|defghi", NO_EXT_PARAM, 3, 6, 0, 0, 0},
    {"abc(def)?", NO_EXT_PARAM, 3, 6, 0, 0, 0},
    {"abc(def){0,3}", NO_EXT_PARAM, 3, 12, 0, 0, 0},
    {"abc(def){1,4}", NO_EXT_PARAM, 6, 15, 0, 0, 0},
    {"", NO_EXT_PARAM, 0, 0, 0, 0, 0},
    {"^", NO_EXT_PARAM, 0, 0, 0, 0, 0},
    {"^\\b", NO_EXT_PARAM, 0, 0, 1, 0, 0},
    {"\\b$", NO_EXT_PARAM, 0, 0, 1, 1, 1},
    {"(?m)\\b$", NO_EXT_PARAM, 0, 0, 1, 1, 0},
    {"\\A", NO_EXT_PARAM, 0, 0, 0, 0, 0},
    {"\\z", NO_EXT_PARAM, 0, 0, 0, 1, 1},
    {"\\Z", NO_EXT_PARAM, 0, 0, 1, 1, 1},
    {"$", NO_EXT_PARAM, 0, 0, 1, 1, 1},
    {"(?m)$", NO_EXT_PARAM, 0, 0, 1, 1, 0},
    {"^foo", NO_EXT_PARAM, 3, 3, 0, 0, 0},
    {"^foo.*bar", NO_EXT_PARAM, 6, UINT_MAX, 0, 0, 0},
    {"^foo.*bar?", NO_EXT_PARAM, 5, UINT_MAX, 0, 0, 0},
    {"^foo.*bar$", NO_EXT_PARAM, 6, UINT_MAX, 1, 1, 1},
    {"^foobar$", NO_EXT_PARAM, 6, 6, 1, 1, 1},
    {"foobar$", NO_EXT_PARAM, 6, 6, 1, 1, 1},
    {"^.*foo", NO_EXT_PARAM, 3, UINT_MAX, 0, 0, 0},
    {"foo\\b", NO_EXT_PARAM, 3, 3, 1, 1, 0},
    {"foo.{1,13}bar", NO_EXT_PARAM, 7, 19, 0, 0, 0},
    {"foo.{10,}bar", NO_EXT_PARAM, 16, UINT_MAX, 0, 0, 0},
    {"foo.{0,10}bar", NO_EXT_PARAM, 6, 16, 0, 0, 0},
    {"foo.{,10}bar", NO_EXT_PARAM, 12, 12, 0, 0, 0},
    {"foo.{10}bar", NO_EXT_PARAM, 16, 16, 0, 0, 0},
    {"(^|\n)foo", NO_EXT_PARAM, 3, 4, 0, 0, 0},
    {"(^\n|)foo", NO_EXT_PARAM, 3, 4, 0, 0, 0},
    {"(?m)^foo", NO_EXT_PARAM, 3, 3, 0, 0, 0},
    {"\\bfoo", NO_EXT_PARAM, 3, 3, 0, 0, 0},
    {"^\\bfoo", NO_EXT_PARAM, 3, 3, 0, 0, 0},
    {"(?m)^\\bfoo", NO_EXT_PARAM, 3, 3, 0, 0, 0},
    {"\\Bfoo", NO_EXT_PARAM, 3, 3, 0, 0, 0},
    {"(foo|bar\\z)", NO_EXT_PARAM, 3, 3, 0, 1, 0},
    {"(foo|bar)\\z", NO_EXT_PARAM, 3, 3, 0, 1, 1},

    // Some cases with extended parameters.
    {"^abc.*def", {HS_EXT_FLAG_MAX_OFFSET, 0, 10, 0, 0, 0}, 6, 10, 0, 0, 0},
    {"^abc.*def", {HS_EXT_FLAG_MIN_LENGTH, 0, 0, 100, 0, 0}, 100, UINT_MAX, 0, 0, 0},
    {"abc.*def", {HS_EXT_FLAG_MAX_OFFSET, 0, 10, 0, 0, 0}, 6, 10, 0, 0, 0},
    {"abc.*def", {HS_EXT_FLAG_MIN_LENGTH, 0, 0, 100, 0, 0}, 100, UINT_MAX, 0, 0, 0},
    {"abc.*def", {HS_EXT_FLAG_MIN_LENGTH, 0, 0, 5, 0, 0}, 6, UINT_MAX, 0, 0, 0},

    {"abc.*def", {HS_EXT_FLAG_EDIT_DISTANCE, 0, 0, 0, 1, 0}, 5, UINT_MAX, 0, 0, 0},
    {"abc.*def", {HS_EXT_FLAG_EDIT_DISTANCE, 0, 0, 0, 2, 0}, 4, UINT_MAX, 0, 0, 0},
    {"abc.*def", {HS_EXT_FLAG_EDIT_DISTANCE | HS_EXT_FLAG_MIN_LENGTH, 0, 0, 10, 2, 0},
                10, UINT_MAX, 0, 0, 0},
    {"abc.*def", {HS_EXT_FLAG_EDIT_DISTANCE | HS_EXT_FLAG_MIN_OFFSET, 6, 0, 0, 2, 0},
                4, UINT_MAX, 0, 0, 0},
    {"abc.*def", {HS_EXT_FLAG_EDIT_DISTANCE | HS_EXT_FLAG_MAX_OFFSET, 0, 6, 0, 2, 0},
                4, 6, 0, 0, 0},

    {"^abc.*def", {HS_EXT_FLAG_EDIT_DISTANCE, 0, 0, 0, 1, 0}, 5, UINT_MAX, 0, 0, 0},
    {"^abc.*def", {HS_EXT_FLAG_EDIT_DISTANCE, 0, 0, 0, 2, 0}, 4, UINT_MAX, 0, 0, 0},
    {"^abc.*def", {HS_EXT_FLAG_EDIT_DISTANCE | HS_EXT_FLAG_MIN_LENGTH, 0, 0, 10, 2, 0},
                10, UINT_MAX, 0, 0, 0},
    {"^abc.*def", {HS_EXT_FLAG_EDIT_DISTANCE | HS_EXT_FLAG_MIN_OFFSET, 6, 0, 0, 2, 0},
                4, UINT_MAX, 0, 0, 0},
    {"^abc.*def", {HS_EXT_FLAG_EDIT_DISTANCE | HS_EXT_FLAG_MAX_OFFSET, 0, 6, 0, 2, 0},
                4, 6, 0, 0, 0},

    {"^abcdef", {HS_EXT_FLAG_EDIT_DISTANCE, 0, 0, 0, 1, 0}, 5, 7, 0, 0, 0},
    {"^abcdef", {HS_EXT_FLAG_EDIT_DISTANCE, 0, 0, 0, 2, 0}, 4, 8, 0, 0, 0},
    {"^abcdef", {HS_EXT_FLAG_EDIT_DISTANCE | HS_EXT_FLAG_MIN_LENGTH, 0, 0, 8, 2, 0},
                8, 8, 0, 0, 0},
    {"^abcdef", {HS_EXT_FLAG_EDIT_DISTANCE | HS_EXT_FLAG_MIN_OFFSET, 6, 0, 0, 2, 0},
                4, 8, 0, 0, 0},
    {"^abcdef", {HS_EXT_FLAG_EDIT_DISTANCE | HS_EXT_FLAG_MAX_OFFSET, 0, 6, 0, 2, 0},
                4, 6, 0, 0, 0},

    {"abc.*def", {HS_EXT_FLAG_HAMMING_DISTANCE, 0, 0, 0, 0, 1}, 6, UINT_MAX, 0, 0, 0},
    {"abc.*def", {HS_EXT_FLAG_HAMMING_DISTANCE, 0, 0, 0, 0, 2}, 6, UINT_MAX, 0, 0, 0},
    {"abc.*def", {HS_EXT_FLAG_HAMMING_DISTANCE, 0, 0, 0, 0, 5}, 6, UINT_MAX, 0, 0, 0},
    {"abc.*def", {HS_EXT_FLAG_HAMMING_DISTANCE | HS_EXT_FLAG_MIN_LENGTH, 0, 0, 10, 0, 2},
                10, UINT_MAX, 0, 0, 0},
    {"abc.*def", {HS_EXT_FLAG_HAMMING_DISTANCE | HS_EXT_FLAG_MIN_OFFSET, 6, 0, 0, 0, 2},
                6, UINT_MAX, 0, 0, 0},
    {"abc.*def", {HS_EXT_FLAG_HAMMING_DISTANCE | HS_EXT_FLAG_MAX_OFFSET, 0, 6, 0, 0, 2},
                6, 6, 0, 0, 0},

    {"^abc.*def", {HS_EXT_FLAG_HAMMING_DISTANCE, 0, 0, 0, 0, 1}, 6, UINT_MAX, 0, 0, 0},
    {"^abc.*def", {HS_EXT_FLAG_HAMMING_DISTANCE, 0, 0, 0, 0, 2}, 6, UINT_MAX, 0, 0, 0},
    {"^abc.*def", {HS_EXT_FLAG_HAMMING_DISTANCE, 0, 0, 0, 0, 5}, 6, UINT_MAX, 0, 0, 0},
    {"^abc.*def", {HS_EXT_FLAG_HAMMING_DISTANCE | HS_EXT_FLAG_MIN_LENGTH, 0, 0, 10, 0, 2},
                10, UINT_MAX, 0, 0, 0},
    {"^abc.*def", {HS_EXT_FLAG_HAMMING_DISTANCE | HS_EXT_FLAG_MIN_OFFSET, 6, 0, 0, 0, 2},
                6, UINT_MAX, 0, 0, 0},
    {"^abc.*def", {HS_EXT_FLAG_HAMMING_DISTANCE | HS_EXT_FLAG_MAX_OFFSET, 0, 6, 0, 0, 2},
                6, 6, 0, 0, 0},

    {"^abcdef", {HS_EXT_FLAG_HAMMING_DISTANCE, 0, 0, 0, 0, 1}, 6, 6, 0, 0, 0},
    {"^abcdef", {HS_EXT_FLAG_HAMMING_DISTANCE, 0, 0, 0, 0, 2}, 6, 6, 0, 0, 0},
    {"^abcdef", {HS_EXT_FLAG_HAMMING_DISTANCE, 0, 0, 0, 0, 5}, 6, 6, 0, 0, 0},
    {"^abcdef", {HS_EXT_FLAG_HAMMING_DISTANCE | HS_EXT_FLAG_MIN_LENGTH, 0, 0, 6, 0, 2},
                6, 6, 0, 0, 0},
    {"^abcdef", {HS_EXT_FLAG_HAMMING_DISTANCE | HS_EXT_FLAG_MIN_OFFSET, 6, 0, 0, 0, 2},
                6, 6, 0, 0, 0},
    {"^abcdef", {HS_EXT_FLAG_HAMMING_DISTANCE | HS_EXT_FLAG_MAX_OFFSET, 0, 6, 0, 0, 2},
                6, 6, 0, 0, 0},
};

INSTANTIATE_TEST_CASE_P(ExprInfo, ExprInfop, ValuesIn(ei_test));

}
