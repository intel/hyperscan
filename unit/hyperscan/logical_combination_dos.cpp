/*
 * Copyright (c) 2026, Intel Corporation
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

/**
 * \file
 * \brief Regression tests for compile-time denial of service via
 * oversized logical-combination expressions.
 *
 * The root cause is that when HS_FLAG_COMBINATION is set, addExpression()
 * bypasses the normal pattern-length guard and forwards the string to
 * ParsedLogical::parseLogicalCombination(), which walks the entire expression
 * and grows multiple vectors/maps with no dedicated size cap.
 *
 * These tests verify that:
 * 1. hs_compile_multi rejects extremely large combination expressions
 * 2. hs_compile_multi rejects deeply nested combination expressions
 * 3. Normal-sized logical combinations still compile and work correctly
 */

#include "config.h"

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "hs.h"
#include "hs_compile.h"
#include "test_util.h"

namespace {

/**
 * Helper: build a combination expression of the form "1|2|3|...|N"
 * referencing N leaf sub-expression IDs.
 */
static std::string make_combination_expr(size_t leaves) {
    std::string out;
    out.reserve(leaves * 8);
    for (size_t i = 1; i <= leaves; i++) {
        if (i > 1) {
            out += '|';
        }
        out += std::to_string(i);
    }
    return out;
}

/**
 * Helper: build a deeply nested combination expression with N levels of
 * parenthesized AND: "(((...(1 & 2) & 3) & 4) & ... & N)"
 */
static std::string make_nested_combination_expr(size_t depth) {
    if (depth < 2) {
        return "1";
    }
    std::string out;
    out.reserve(depth * 8);
    /* Build left-nested: (((1 & 2) & 3) & 4) ... */
    for (size_t i = 0; i + 2 < depth; i++) {
        out += '(';
    }
    out += "1 & 2";
    for (size_t i = 3; i <= depth; i++) {
        out += ") & ";
        out += std::to_string(i);
    }
    return out;
}

// ============================================================================
// Test 1: A combination expression with enough leaf references to exceed
// the length limit must be rejected by the compile API.
// The combination string "1|2|3|...|5000" exceeds the configured length limit.
// ============================================================================
TEST(LogicalCombinationDoS, RejectsOversizedCombination) {
    const size_t LEAF_COUNT = 5000;

    /* Build leaf patterns */
    std::vector<std::string> leaf_strs;
    std::vector<const char *> exprs;
    std::vector<unsigned> flags;
    std::vector<unsigned> ids;

    leaf_strs.reserve(LEAF_COUNT);
    for (size_t i = 0; i < LEAF_COUNT; i++) {
        leaf_strs.push_back("a" + std::to_string(i + 1));
    }
    for (size_t i = 0; i < LEAF_COUNT; i++) {
        exprs.push_back(leaf_strs[i].c_str());
        flags.push_back(0);
        ids.push_back(static_cast<unsigned>(i + 1));
    }

    /* Add the oversized combination expression */
    std::string combo = make_combination_expr(LEAF_COUNT);
    exprs.push_back(combo.c_str());
    flags.push_back(HS_FLAG_COMBINATION);
    ids.push_back(static_cast<unsigned>(LEAF_COUNT + 1));

    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;

    hs_error_t err = hs_compile_multi(
        exprs.data(), flags.data(), ids.data(),
        static_cast<unsigned>(exprs.size()), HS_MODE_BLOCK, nullptr, &db,
        &compile_err);

    /* After the fix, the compile API must reject this with a compile error
     * rather than consuming excessive CPU/memory processing 200K operands.
     * The error can be HS_COMPILER_ERROR (expression too long) or any
     * non-success code. */
    EXPECT_NE(HS_SUCCESS, err)
        << "hs_compile_multi should reject an oversized logical combination "
           "with " << LEAF_COUNT << " operands";

    if (compile_err) {
        hs_free_compile_error(compile_err);
    }
    if (db) {
        hs_free_database(db);
    }
}

// ============================================================================
// Test 2: A large combination string that is extremely long (> 10MB).
// Even without many distinct leaves, a huge string can cause DoS during
// parsing.
// ============================================================================
TEST(LogicalCombinationDoS, RejectsVeryLongCombinationString) {
    /* Create two simple leaf patterns */
    const char *exprs[] = {"abc", "def", nullptr};
    unsigned flags_arr[] = {0, 0, HS_FLAG_COMBINATION};
    unsigned ids_arr[] = {1, 2, 3};

    /* Build a very long combination: "1 | 2 | 1 | 2 | ..." repeated */
    const size_t target_len = 10 * 1024 * 1024; /* 10 MB */
    std::string combo;
    combo.reserve(target_len + 16);
    combo = "1";
    while (combo.size() < target_len) {
        combo += " | 2 | 1";
    }

    const char *expr_ptrs[] = {"abc", "def", combo.c_str()};

    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;

    hs_error_t err = hs_compile_multi(
        expr_ptrs, flags_arr, ids_arr, 3, HS_MODE_BLOCK, nullptr, &db,
        &compile_err);

    EXPECT_NE(HS_SUCCESS, err)
        << "hs_compile_multi should reject a 10MB logical combination string";

    if (compile_err) {
        hs_free_compile_error(compile_err);
    }
    if (db) {
        hs_free_database(db);
    }
}

// ============================================================================
// Test 3: A deeply nested combination expression should be rejected or at
// least handled safely.
// ============================================================================
TEST(LogicalCombinationDoS, RejectsDeeplyNestedCombination) {
    const size_t DEPTH = 50000;

    /* Build leaf patterns for each depth level */
    std::vector<std::string> leaf_strs;
    std::vector<const char *> exprs;
    std::vector<unsigned> flags;
    std::vector<unsigned> ids;

    leaf_strs.reserve(DEPTH);
    for (size_t i = 0; i < DEPTH; i++) {
        leaf_strs.push_back("a" + std::to_string(i + 1));
    }
    for (size_t i = 0; i < DEPTH; i++) {
        exprs.push_back(leaf_strs[i].c_str());
        flags.push_back(0);
        ids.push_back(static_cast<unsigned>(i + 1));
    }

    std::string combo = make_nested_combination_expr(DEPTH);
    exprs.push_back(combo.c_str());
    flags.push_back(HS_FLAG_COMBINATION);
    ids.push_back(static_cast<unsigned>(DEPTH + 1));

    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;

    hs_error_t err = hs_compile_multi(
        exprs.data(), flags.data(), ids.data(),
        static_cast<unsigned>(exprs.size()), HS_MODE_BLOCK, nullptr, &db,
        &compile_err);

    EXPECT_NE(HS_SUCCESS, err)
        << "hs_compile_multi should reject a deeply nested combination "
           "expression with depth=" << DEPTH;

    if (compile_err) {
        hs_free_compile_error(compile_err);
    }
    if (db) {
        hs_free_database(db);
    }
}

// ============================================================================
// Test 4: A reasonably sized logical combination must still work correctly.
// This is a false-positive control test.
// ============================================================================
TEST(LogicalCombinationDoS, SmallCombinationStillWorks) {
    const char *expr[] = {"abc", "def", "ghi", "(1 & 2) | 3"};
    unsigned flags[] = {0, 0, 0, HS_FLAG_COMBINATION};
    unsigned ids[] = {1, 2, 3, 100};

    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile_multi(expr, flags, ids, 4, HS_MODE_BLOCK,
                                      nullptr, &db, &compile_err);
    ASSERT_EQ(HS_SUCCESS, err) << "Small logical combination should compile";
    ASSERT_NE(nullptr, db);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(nullptr, scratch);

    /* Input that matches "abc" and "def" → combination (1 & 2) fires */
    const char input[] = "xxxabcxxxdefxxx";
    CallBackContext c;
    c.halt = false;
    err = hs_scan(db, input, sizeof(input) - 1, 0, scratch, record_cb,
                  (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    /* Check that the combination ID matched */
    bool combo_matched = false;
    for (const auto &m : c.matches) {
        if (m.id == 100) {
            combo_matched = true;
            break;
        }
    }
    EXPECT_TRUE(combo_matched) << "Small logical combination (1 & 2) should "
                                  "match when both sub-patterns match";

    hs_free_scratch(scratch);
    hs_free_database(db);
}

// ============================================================================
// Test 5: A moderate-size combination (100 operands) should still work.
// This ensures the limit is not set too aggressively.
// ============================================================================
TEST(LogicalCombinationDoS, ModerateCombinationStillWorks) {
    const size_t LEAF_COUNT = 100;

    std::vector<std::string> leaf_strs;
    std::vector<const char *> exprs;
    std::vector<unsigned> flags;
    std::vector<unsigned> ids;

    for (size_t i = 0; i < LEAF_COUNT; i++) {
        leaf_strs.push_back("pattern" + std::to_string(i + 1));
    }
    for (size_t i = 0; i < LEAF_COUNT; i++) {
        exprs.push_back(leaf_strs[i].c_str());
        flags.push_back(0);
        ids.push_back(static_cast<unsigned>(i + 1));
    }

    std::string combo = make_combination_expr(LEAF_COUNT);
    exprs.push_back(combo.c_str());
    flags.push_back(HS_FLAG_COMBINATION);
    ids.push_back(static_cast<unsigned>(LEAF_COUNT + 1));

    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;

    hs_error_t err = hs_compile_multi(
        exprs.data(), flags.data(), ids.data(),
        static_cast<unsigned>(exprs.size()), HS_MODE_BLOCK, nullptr, &db,
        &compile_err);
    ASSERT_EQ(HS_SUCCESS, err)
        << "Moderate logical combination (100 operands) should compile";
    ASSERT_NE(nullptr, db);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    /* Input that matches the first leaf pattern */
    const char input[] = "xpattern1x";
    unsigned match_count = 0;
    err = hs_scan(db, input, sizeof(input) - 1, 0, scratch,
                  [](unsigned, unsigned long long, unsigned long long,
                     unsigned, void *ctx) -> int {
                      ++(*static_cast<unsigned *>(ctx));
                      return 0;
                  },
                  &match_count);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_GT(match_count, 0u) << "At least one match expected";

    hs_free_scratch(scratch);
    hs_free_database(db);

    if (compile_err) {
        hs_free_compile_error(compile_err);
    }
}

// ============================================================================
// Test 6: Stream mode — oversized combination must be rejected.
// ============================================================================
TEST(LogicalCombinationDoS, RejectsOversizedCombinationStreamMode) {
    const char *expr[] = {"abc", "def", nullptr};
    unsigned flags_arr[] = {0, 0, HS_FLAG_COMBINATION};
    unsigned ids_arr[] = {1, 2, 3};

    /* Build a combination expression exceeding the 16,000 char limit */
    std::string combo;
    combo.reserve(20000);
    combo = "1";
    while (combo.size() < 17000) {
        combo += " | 2 | 1";
    }

    const char *expr_ptrs[] = {"abc", "def", combo.c_str()};

    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;

    hs_error_t err = hs_compile_multi(
        expr_ptrs, flags_arr, ids_arr, 3, HS_MODE_STREAM, nullptr, &db,
        &compile_err);

    EXPECT_NE(HS_SUCCESS, err)
        << "hs_compile_multi should reject oversized combination in stream mode";

    if (compile_err) hs_free_compile_error(compile_err);
    if (db) hs_free_database(db);
}

// ============================================================================
// Test 7: Vectored mode — oversized combination must be rejected.
// ============================================================================
TEST(LogicalCombinationDoS, RejectsOversizedCombinationVectoredMode) {
    const char *expr[] = {"abc", "def", nullptr};
    unsigned flags_arr[] = {0, 0, HS_FLAG_COMBINATION};
    unsigned ids_arr[] = {1, 2, 3};

    std::string combo;
    combo.reserve(20000);
    combo = "1";
    while (combo.size() < 17000) {
        combo += " | 2 | 1";
    }

    const char *expr_ptrs[] = {"abc", "def", combo.c_str()};

    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;

    hs_error_t err = hs_compile_multi(
        expr_ptrs, flags_arr, ids_arr, 3, HS_MODE_VECTORED, nullptr, &db,
        &compile_err);

    EXPECT_NE(HS_SUCCESS, err)
        << "hs_compile_multi should reject oversized combination in vectored mode";

    if (compile_err) hs_free_compile_error(compile_err);
    if (db) hs_free_database(db);
}

// ============================================================================
// Test 8: Stream mode — small combination works correctly.
// ============================================================================
TEST(LogicalCombinationDoS, SmallCombinationStreamMode) {
    const char *expr[] = {"abc", "def", "ghi", "(1 & 2) | 3"};
    unsigned flags[] = {0, 0, 0, HS_FLAG_COMBINATION};
    unsigned ids[] = {1, 2, 3, 100};

    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile_multi(expr, flags, ids, 4, HS_MODE_STREAM,
                                      nullptr, &db, &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(nullptr, db);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_stream_t *stream = nullptr;
    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);

    CallBackContext c;
    const char chunk1[] = "xxxabc";
    err = hs_scan_stream(stream, chunk1, sizeof(chunk1) - 1, 0, scratch,
                         record_cb, (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);

    const char chunk2[] = "xxxdefxxx";
    err = hs_scan_stream(stream, chunk2, sizeof(chunk2) - 1, 0, scratch,
                         record_cb, (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_close_stream(stream, scratch, record_cb, (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);

    bool combo_matched = false;
    for (const auto &m : c.matches) {
        if (m.id == 100) {
            combo_matched = true;
            break;
        }
    }
    EXPECT_TRUE(combo_matched)
        << "Small combination (1 & 2) should match in stream mode";

    hs_free_scratch(scratch);
    hs_free_database(db);
}

// ============================================================================
// Test 9: Vectored mode — small combination works correctly.
// ============================================================================
TEST(LogicalCombinationDoS, SmallCombinationVectoredMode) {
    const char *expr[] = {"abc", "def", "ghi", "(1 & 2) | 3"};
    unsigned flags[] = {0, 0, 0, HS_FLAG_COMBINATION};
    unsigned ids[] = {1, 2, 3, 100};

    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile_multi(expr, flags, ids, 4, HS_MODE_VECTORED,
                                      nullptr, &db, &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(nullptr, db);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    const char part1[] = "xxxabc";
    const char part2[] = "xxxdefxxx";
    const char *data[] = {part1, part2};
    unsigned int lengths[] = {(unsigned)(sizeof(part1) - 1),
                              (unsigned)(sizeof(part2) - 1)};

    CallBackContext c;
    err = hs_scan_vector(db, data, lengths, 2, 0, scratch,
                         record_cb, (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);

    bool combo_matched = false;
    for (const auto &m : c.matches) {
        if (m.id == 100) {
            combo_matched = true;
            break;
        }
    }
    EXPECT_TRUE(combo_matched)
        << "Small combination (1 & 2) should match in vectored mode";

    hs_free_scratch(scratch);
    hs_free_database(db);
}

} // namespace
