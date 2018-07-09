/*
 * Copyright (c) 2018, Intel Corporation
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
#include "chimera/ch.h"

#include <cstdlib>
#include <string>

using std::string;

static void *null_malloc(size_t) { return nullptr; }

// Helper: correctly construct a simple database.
static
void makeDatabase(ch_database_t **hydb) {
    static const char *expr[] = { "foobar" };
    ch_database_t *db = nullptr;
    ch_compile_error_t *compile_err = nullptr;
    ch_error_t err;

    err = ch_compile_multi(expr, nullptr, nullptr, 1, 0, nullptr, &db,
                           &compile_err);

    ASSERT_EQ(CH_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    *hydb = db;
}

TEST(HybridAllocator, DatabaseInfoBadAlloc) {
    ch_database_t *db = nullptr;
    makeDatabase(&db);
    ASSERT_TRUE(db != nullptr);

    ch_set_allocator(null_malloc, nullptr);

    char *info = nullptr;
    ch_error_t err = ch_database_info(db, &info);
    ASSERT_EQ(CH_NOMEM, err);

    ch_set_allocator(nullptr, nullptr);
    ch_free_database(db);
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

TEST(HybridAllocator, TwoAlignedCompile) {
    ch_set_database_allocator(two_aligned_malloc, two_aligned_free);

    ch_database_t *db = nullptr;
    ch_compile_error_t *compile_err = nullptr;
    const hs_platform_info_t *platform = nullptr;
    ch_error_t err =
        ch_compile("foobar", 0, CH_MODE_GROUPS, platform, &db, &compile_err);
    ASSERT_EQ(CH_COMPILER_ERROR, err);
    ASSERT_EQ(nullptr, db);
    ASSERT_NE(nullptr, compile_err);
    ch_free_compile_error(compile_err);
    ch_set_database_allocator(nullptr, nullptr);
}

TEST(HybridAllocator, TwoAlignedCompileError) {
    ch_set_misc_allocator(two_aligned_malloc, two_aligned_free);

    ch_database_t *db = nullptr;
    ch_compile_error_t *compile_err = nullptr;
    const hs_platform_info_t *platform = nullptr;
    ch_error_t err =
        ch_compile("\\1", 0, CH_MODE_GROUPS, platform, &db, &compile_err);
    ASSERT_EQ(CH_COMPILER_ERROR, err);
    ASSERT_EQ(nullptr, db);
    ASSERT_NE(nullptr, compile_err);
    EXPECT_STREQ("Allocator returned misaligned memory.", compile_err->message);
    ch_free_compile_error(compile_err);
    ch_set_database_allocator(nullptr, nullptr);
    ch_set_misc_allocator(nullptr, nullptr);
}

TEST(HybridAllocator, TwoAlignedDatabaseInfo) {
    ch_database_t *db = nullptr;
    makeDatabase(&db);

    ch_set_misc_allocator(two_aligned_malloc, two_aligned_free);

    char *info = nullptr;
    ch_error_t err = ch_database_info(db, &info);
    ASSERT_EQ(CH_BAD_ALLOC, err);

    ch_set_misc_allocator(nullptr, nullptr);
    ch_free_database(db);
}

TEST(HybridAllocator, TwoAlignedAllocScratch) {
    ch_database_t *db = nullptr;
    makeDatabase(&db);

    ch_set_scratch_allocator(two_aligned_malloc, two_aligned_free);

    ch_scratch_t *scratch = nullptr;
    ch_error_t err = ch_alloc_scratch(db, &scratch);
    ASSERT_EQ(CH_BAD_ALLOC, err);

    ch_set_scratch_allocator(nullptr, nullptr);
    ch_free_database(db);
}
