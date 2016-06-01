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

#include <stdlib.h>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "hs.h"
#include "test_util.h"

using namespace std;
using namespace testing;

namespace /* anonymous */ {

static size_t last_alloc_size;

static void *log_malloc(size_t n) {
    last_alloc_size = n;
    return malloc(n);
}

static void *bad_alloc(size_t) { return nullptr; }

TEST(scratch, testAlloc) {
    vector<pattern> patterns;
    patterns.push_back(pattern("aa", HS_FLAG_DOTALL, 1));
    patterns.push_back(pattern("aa.", HS_FLAG_DOTALL, 2));
    patterns.push_back(pattern("aa..", HS_FLAG_DOTALL, 3));
    patterns.push_back(pattern("^.{0,4}aa..", HS_FLAG_DOTALL, 4));
    patterns.push_back(pattern("^.{0,4}aa", HS_FLAG_DOTALL, 5));

    hs_database_t *db = buildDB(patterns, HS_MODE_NOSTREAM);
    ASSERT_NE(nullptr, db);

    hs_error_t err;

    err = hs_set_allocator(log_malloc, free);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    size_t curr_size;
    err = hs_scratch_size(scratch, &curr_size);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(last_alloc_size, curr_size);

    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
    hs_set_allocator(nullptr, nullptr);
}

TEST(scratch, testScratchAlloc) {
    vector<pattern> patterns;

    allocated_count = 0;
    allocated_count_b = 0;
    hs_error_t err = hs_set_allocator(count_malloc_b, count_free_b);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_set_scratch_allocator(count_malloc, count_free);
    ASSERT_EQ(HS_SUCCESS, err);

    patterns.push_back(pattern("aa", HS_FLAG_DOTALL, 1));
    patterns.push_back(pattern("aa.", HS_FLAG_DOTALL, 2));
    patterns.push_back(pattern("aa..", HS_FLAG_DOTALL, 3));
    patterns.push_back(pattern("^.{0,4}aa..", HS_FLAG_DOTALL, 4));
    patterns.push_back(pattern("^.{0,4}aa", HS_FLAG_DOTALL, 5));

    hs_database_t *db = buildDB(patterns, HS_MODE_NOSTREAM);
    ASSERT_NE(nullptr, db);

    ASSERT_EQ(0, allocated_count);
    ASSERT_NE(0, allocated_count_b);
    size_t old_b = allocated_count_b;

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    size_t curr_size;
    err = hs_scratch_size(scratch, &curr_size);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(allocated_count, curr_size);
    ASSERT_EQ(allocated_count_b, old_b);

    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
    hs_set_allocator(nullptr, nullptr);

    ASSERT_EQ(0, allocated_count);
    ASSERT_EQ(0, allocated_count_b);
}


TEST(scratch, badAlloc) {
    vector<pattern> patterns;
    patterns.push_back(pattern("aa", HS_FLAG_DOTALL, 1));
    patterns.push_back(pattern("aa.", HS_FLAG_DOTALL, 2));
    patterns.push_back(pattern("aa..", HS_FLAG_DOTALL, 3));
    patterns.push_back(pattern("^.{0,4}aa..", HS_FLAG_DOTALL, 4));
    patterns.push_back(pattern("^.{0,4}aa", HS_FLAG_DOTALL, 5));

    hs_database_t *db = buildDB(patterns, HS_MODE_NOSTREAM);
    ASSERT_NE(nullptr, db);

    hs_error_t err;

    err = hs_set_scratch_allocator(bad_alloc, free);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_NE(HS_SUCCESS, err);
    ASSERT_TRUE(scratch == nullptr);

    hs_free_database(db);
    hs_set_scratch_allocator(nullptr, nullptr);
}

TEST(scratch, testScratchRealloc) {
    vector<pattern> patterns;
    patterns.push_back(pattern("aa", HS_FLAG_DOTALL, 1));
    patterns.push_back(pattern("aa.", HS_FLAG_DOTALL, 2));
    patterns.push_back(pattern("aa..", HS_FLAG_DOTALL, 3));
    patterns.push_back(pattern("^.{0,4}aa..", HS_FLAG_DOTALL, 4));
    patterns.push_back(pattern("^.{0,4}aa", HS_FLAG_DOTALL, 5));

    hs_database_t *db = buildDB(patterns, HS_MODE_NOSTREAM);
    ASSERT_NE(nullptr, db);

    patterns.push_back(pattern("^.{0,4}aa{0,4}", HS_FLAG_DOTALL, 6));
    patterns.push_back(pattern("^.{0,4}aa{0,4}a..", HS_FLAG_DOTALL, 7));
    hs_database_t *db2 = buildDB(patterns, HS_MODE_NOSTREAM);
    ASSERT_NE(nullptr, db2);

    hs_error_t err;

    err = hs_set_scratch_allocator(log_malloc, free);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    size_t curr_size;
    err = hs_scratch_size(scratch, &curr_size);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(last_alloc_size, curr_size);

    err = hs_alloc_scratch(db2, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_scratch_size(scratch, &curr_size);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(last_alloc_size, curr_size);

    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
    hs_free_database(db2);
    hs_set_scratch_allocator(nullptr, nullptr);
}

TEST(scratch, tooSmallForDatabase) {
    hs_database_t *db1 = buildDB("foobar", 0, 0, HS_MODE_BLOCK, nullptr);
    ASSERT_NE(nullptr, db1);

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db1, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_free_database(db1);

    hs_database_t *db2 =
        buildDB("(a.?b.?c.?d.?e.?f.?g)|(hatstand(..)+teakettle)", 0, 0,
                HS_MODE_BLOCK, nullptr);
    ASSERT_NE(nullptr, db2);

    // Try and scan some data using db2 and a scratch that has only been
    // allocated for db1.
    err = hs_scan(db2, "somedata", 8, 0, scratch, dummy_cb, nullptr);
    ASSERT_EQ(HS_INVALID, err);

    // Alloc scratch correctly and try again.
    err = hs_alloc_scratch(db2, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    err = hs_scan(db2, "somedata", 8, 0, scratch, dummy_cb, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db2);
}

TEST(scratch, tooSmallForDatabase2) {
    hs_database_t *db1 = buildDB("foobar", 0, 0, HS_MODE_STREAM, nullptr);
    ASSERT_NE(nullptr, db1);

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db1, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_free_database(db1);

    hs_database_t *db2 =
        buildDB("(a.?b.?c.?d.?e.?f.?g)|(hatstand(..)+teakettle)", 0, 0,
                HS_MODE_STREAM, nullptr);
    ASSERT_NE(nullptr, db2);

    // Try and scan some data using db2 and a scratch that has only been
    // allocated for db1.
    hs_stream_t *stream = nullptr;
    err = hs_open_stream(db2, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    err = hs_scan_stream(stream, "somedata", 8, 0, scratch, dummy_cb, nullptr);
    ASSERT_EQ(HS_INVALID, err);

    // Alloc scratch correctly and try again.
    err = hs_alloc_scratch(db2, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    err = hs_scan_stream(stream, "somedata", 8, 0, scratch, dummy_cb, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    err = hs_close_stream(stream, scratch, nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db2);
}

TEST(scratch, damagedScratch) {
    hs_database_t *db = buildDB("foobar", 0, 0, HS_MODE_BLOCK, nullptr);
    ASSERT_NE(nullptr, db);

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    size_t scratch_size = 0;
    err = hs_scratch_size(scratch, &scratch_size);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_LT(4, scratch_size);

    // Take a temp copy and then scribble over the first four bytes.
    char tmp[4];
    memcpy(tmp, scratch, 4);
    memset(scratch, 0xff, 4);

    err = hs_scan(db, "somedata", 8, 0, scratch, dummy_cb, nullptr);
    ASSERT_EQ(HS_INVALID, err);

    // Restore first four bytes so we can free.
    memcpy(scratch, tmp, 4);

    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_free_database(db);
}

} // namespace
