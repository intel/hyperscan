/*
 * Copyright (c) 2015-2016, Intel Corporation
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
#include "test_util.h"

#include <cstdlib>
#include <string>

using std::string;

static void *null_malloc(size_t) { return nullptr; }

TEST(CustomAllocator, DatabaseInfoBadAlloc) {
    hs_database_t *db = buildDB("foobar", 0, 0, HS_MODE_BLOCK);
    ASSERT_TRUE(db != nullptr);

    hs_set_allocator(null_malloc, nullptr);

    char *info = nullptr;
    hs_error_t err = hs_database_info(db, &info);
    ASSERT_EQ(HS_NOMEM, err);

    hs_set_allocator(nullptr, nullptr);
    hs_free_database(db);
}

static
void * two_aligned_malloc(size_t len) {
    void *mem = malloc(len + 2);
    if (!mem) {
        return nullptr;
    }
    return (char *)mem + 2;
}

static
void two_aligned_free(void *mem) {
    if (!mem) {
        return;
    }
    // Allocated with two_aligned_malloc above.
    free((char *)mem - 2);
}

TEST(CustomAllocator, TwoAlignedCompile) {
    hs_set_database_allocator(two_aligned_malloc, two_aligned_free);

    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    const hs_platform_info_t *platform = nullptr;
    hs_error_t err =
        hs_compile("foobar", 0, HS_MODE_BLOCK, platform, &db, &compile_err);
    ASSERT_EQ(HS_COMPILER_ERROR, err);
    ASSERT_EQ(nullptr, db);
    ASSERT_NE(nullptr, compile_err);
    hs_free_compile_error(compile_err);
    hs_set_database_allocator(nullptr, nullptr);
}

TEST(CustomAllocator, TwoAlignedCompileError) {
    hs_set_misc_allocator(two_aligned_malloc, two_aligned_free);

    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    const hs_platform_info_t *platform = nullptr;
    hs_error_t err =
        hs_compile("\\1", 0, HS_MODE_BLOCK, platform, &db, &compile_err);
    ASSERT_EQ(HS_COMPILER_ERROR, err);
    ASSERT_EQ(nullptr, db);
    ASSERT_NE(nullptr, compile_err);
    EXPECT_STREQ("Allocator returned misaligned memory.", compile_err->message);
    hs_free_compile_error(compile_err);
    hs_set_database_allocator(nullptr, nullptr);
}

TEST(CustomAllocator, TwoAlignedDatabaseInfo) {
    hs_database_t *db = buildDB("foobar", 0, 0, HS_MODE_BLOCK);
    ASSERT_TRUE(db != nullptr);

    hs_set_misc_allocator(two_aligned_malloc, two_aligned_free);

    char *info = nullptr;
    hs_error_t err = hs_database_info(db, &info);
    ASSERT_EQ(HS_BAD_ALLOC, err);

    hs_set_misc_allocator(nullptr, nullptr);
    hs_free_database(db);
}

TEST(CustomAllocator, TwoAlignedSerialize) {
    hs_database_t *db = buildDB("foobar", 0, 0, HS_MODE_BLOCK);
    ASSERT_TRUE(db != nullptr);

    hs_set_misc_allocator(two_aligned_malloc, two_aligned_free);

    char *bytes = nullptr;
    size_t serialized_len = 0;
    hs_error_t err = hs_serialize_database(db, &bytes, &serialized_len);
    ASSERT_EQ(HS_BAD_ALLOC, err);

    hs_set_misc_allocator(nullptr, nullptr);
    hs_free_database(db);
}

TEST(CustomAllocator, TwoAlignedDeserialize) {
    hs_database_t *db = buildDB("foobar", 0, 0, HS_MODE_BLOCK);
    ASSERT_TRUE(db != nullptr);

    char *bytes = nullptr;
    size_t serialized_len = 0;
    hs_error_t err = hs_serialize_database(db, &bytes, &serialized_len);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(nullptr, bytes);

    hs_free_database(db);
    db = nullptr;

    hs_set_database_allocator(two_aligned_malloc, two_aligned_free);

    err = hs_deserialize_database(bytes, serialized_len, &db);
    ASSERT_EQ(HS_BAD_ALLOC, err);
    ASSERT_EQ(nullptr, db);

    hs_set_database_allocator(nullptr, nullptr);

    free(bytes);
}

TEST(CustomAllocator, TwoAlignedAllocScratch) {
    hs_database_t *db = buildDB("foobar", 0, 0, HS_MODE_BLOCK);
    ASSERT_TRUE(db != nullptr);

    hs_set_scratch_allocator(two_aligned_malloc, two_aligned_free);

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_BAD_ALLOC, err);

    hs_set_scratch_allocator(nullptr, nullptr);
    hs_free_database(db);
}

TEST(CustomAllocator, NullMallocExpressionInfo) {
    hs_set_allocator(null_malloc, nullptr);

    string pattern = "foobar";
    hs_expr_info_t *info = nullptr;
    hs_compile_error_t *c_err = nullptr;
    hs_error_t err = hs_expression_info(pattern.c_str(), 0, &info, &c_err);
    ASSERT_EQ(HS_COMPILER_ERROR, err);
    ASSERT_NE(nullptr, c_err);
    hs_free_compile_error(c_err);
    hs_set_allocator(nullptr, nullptr);
}

TEST(CustomAllocator, TwoAlignedExpressionInfo) {
    hs_set_misc_allocator(two_aligned_malloc, two_aligned_free);

    string pattern = "\\1";
    hs_expr_info_t *info = nullptr;
    hs_compile_error_t *c_err = nullptr;
    hs_error_t err = hs_expression_info(pattern.c_str(), 0, &info, &c_err);
    ASSERT_EQ(HS_COMPILER_ERROR, err);
    ASSERT_NE(nullptr, c_err);
    EXPECT_STREQ("Allocator returned misaligned memory.", c_err->message);
    hs_free_compile_error(c_err);
    hs_set_allocator(nullptr, nullptr);
}
