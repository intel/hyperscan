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
#include <algorithm>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "hs.h"
#include "test_util.h"

using namespace std;
using namespace testing;

namespace /* anonymous */ {

bool matchesOrdered(const vector<MatchRecord> &matches) {
    unsigned long lastMatch = 0;
    for (vector<MatchRecord>::const_iterator it = matches.begin();
         it != matches.end(); ++it) {
        if (lastMatch > it->to) {
            return false;
        }
        lastMatch = it->to;
    }
    return true;
}

unsigned countMatchesById(const vector<MatchRecord> &matches, int id) {
    unsigned count = 0;
    for (vector<MatchRecord>::const_iterator it = matches.begin();
         it != matches.end(); ++it) {
        if (id == it->id) {
            count++;
        }
    }
    return count;
}

TEST(order, ordering1) {
    vector<pattern> patterns;
    patterns.push_back(pattern("aa", HS_FLAG_DOTALL, 1));
    patterns.push_back(pattern("aa.", HS_FLAG_DOTALL, 2));
    patterns.push_back(pattern("aa..", HS_FLAG_DOTALL, 3));
    patterns.push_back(pattern("^.{0,4}aa..", HS_FLAG_DOTALL, 4));
    patterns.push_back(pattern("^.{0,4}aa", HS_FLAG_DOTALL, 5));

    const char *data = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";

    hs_database_t *db = buildDB(patterns, HS_MODE_NOSTREAM);
    ASSERT_NE(nullptr, db);

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    CallBackContext c;
    err = hs_scan(db, data, strlen(data), 0, scratch, record_cb,
                  (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);

    EXPECT_EQ(31U, countMatchesById(c.matches, 1));
    EXPECT_EQ(30U, countMatchesById(c.matches, 2));
    EXPECT_EQ(29U, countMatchesById(c.matches, 3));
    EXPECT_EQ(5U, countMatchesById(c.matches, 4));
    EXPECT_EQ(5U, countMatchesById(c.matches, 5));
    ASSERT_TRUE(matchesOrdered(c.matches));
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(order, ordering2) {
    vector<pattern> patterns;
    patterns.push_back(pattern("aa.", HS_FLAG_DOTALL, 2));
    patterns.push_back(pattern("aa..", HS_FLAG_DOTALL, 3));
    patterns.push_back(pattern("^.{0,4}aa..", HS_FLAG_DOTALL, 4));
    patterns.push_back(pattern("^.{0,4}aa", HS_FLAG_DOTALL, 5));

    const char *data = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";

    hs_database_t *db = buildDB(patterns, HS_MODE_NOSTREAM);
    ASSERT_NE(nullptr, db);

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    CallBackContext c;
    err = hs_scan(db, data, strlen(data), 0, scratch, record_cb,
                  (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);

    EXPECT_EQ(0U, countMatchesById(c.matches, 1));
    EXPECT_EQ(30U, countMatchesById(c.matches, 2));
    EXPECT_EQ(29U, countMatchesById(c.matches, 3));
    EXPECT_EQ(5U, countMatchesById(c.matches, 4));
    EXPECT_EQ(5U, countMatchesById(c.matches, 5));
    ASSERT_TRUE(matchesOrdered(c.matches));
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(order, ordering3) {
    vector<pattern> patterns;
    patterns.push_back(pattern("aa.", HS_FLAG_DOTALL, 2));
    patterns.push_back(pattern("aa..", HS_FLAG_DOTALL, 3));
    patterns.push_back(pattern("^.{0,4}aa..", HS_FLAG_DOTALL, 4));

    const char *data = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";

    hs_database_t *db = buildDB(patterns, HS_MODE_NOSTREAM);
    ASSERT_NE(nullptr, db);

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    CallBackContext c;
    err = hs_scan(db, data, strlen(data), 0, scratch, record_cb,
                  (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);

    EXPECT_EQ(0U, countMatchesById(c.matches, 1));
    EXPECT_EQ(30U, countMatchesById(c.matches, 2));
    EXPECT_EQ(29U, countMatchesById(c.matches, 3));
    EXPECT_EQ(5U, countMatchesById(c.matches, 4));
    EXPECT_EQ(0U, countMatchesById(c.matches, 5));
    ASSERT_TRUE(matchesOrdered(c.matches));
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(order, ordering4) {
    vector<pattern> patterns;
    patterns.push_back(pattern("aa", HS_FLAG_DOTALL, 1));
    patterns.push_back(pattern("aa.", HS_FLAG_DOTALL, 2));
    patterns.push_back(pattern("aa..", HS_FLAG_DOTALL, 3));

    const char *data = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";

    hs_database_t *db = buildDB(patterns, HS_MODE_NOSTREAM);
    ASSERT_NE(nullptr, db);

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    CallBackContext c;
    err = hs_scan(db, data, strlen(data), 0, scratch, record_cb,
                  (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);

    EXPECT_EQ(31U, countMatchesById(c.matches, 1));
    EXPECT_EQ(30U, countMatchesById(c.matches, 2));
    EXPECT_EQ(29U, countMatchesById(c.matches, 3));
    EXPECT_EQ(0U, countMatchesById(c.matches, 4));
    EXPECT_EQ(0U, countMatchesById(c.matches, 5));
    ASSERT_TRUE(matchesOrdered(c.matches));
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(order, ordering5) {
    vector<pattern> patterns;
    patterns.push_back(pattern("aa", HS_FLAG_DOTALL, 1));
    patterns.push_back(pattern("aa.", HS_FLAG_DOTALL, 2));
    patterns.push_back(pattern("aa..", HS_FLAG_DOTALL, 3));
    patterns.push_back(pattern("^.{0,4}aa..", HS_FLAG_DOTALL, 4));
    patterns.push_back(pattern("^.{0,4}aa", HS_FLAG_DOTALL, 5));

    const char *data = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";

    hs_database_t *db = buildDB(patterns, HS_MODE_STREAM);
    ASSERT_NE(nullptr, db);

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_stream_t *stream = nullptr;
    CallBackContext c;

    for (size_t jump = 1; jump <= 8; jump++) {
        err = hs_open_stream(db, 0, &stream);
        ASSERT_EQ(HS_SUCCESS, err);
        ASSERT_TRUE(stream != nullptr);

        for (unsigned i = 0; i < strlen(data); i += jump) {
            err = hs_scan_stream(stream, data + i, min(jump, strlen(data) - i),
                                 0, scratch, record_cb, (void *)&c);
            ASSERT_EQ(HS_SUCCESS, err);
        }
        EXPECT_EQ(31U, countMatchesById(c.matches, 1));
        EXPECT_EQ(30U, countMatchesById(c.matches, 2));
        EXPECT_EQ(29U, countMatchesById(c.matches, 3));
        EXPECT_EQ(5U, countMatchesById(c.matches, 4));
        EXPECT_EQ(5U, countMatchesById(c.matches, 5));
        ASSERT_TRUE(matchesOrdered(c.matches));
        c.matches.clear();
        hs_close_stream(stream, scratch, record_cb, (void *)&c);
    }
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(order, ordering6) {
    vector<pattern> patterns;
    patterns.push_back(pattern("aa.", HS_FLAG_DOTALL, 2));
    patterns.push_back(pattern("aa..", HS_FLAG_DOTALL, 3));
    patterns.push_back(pattern("^.{0,4}aa..", HS_FLAG_DOTALL, 4));
    patterns.push_back(pattern("^.{0,4}aa", HS_FLAG_DOTALL, 5));

    const char *data = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";

    hs_database_t *db = buildDB(patterns, HS_MODE_STREAM);
    ASSERT_NE(nullptr, db);

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_stream_t *stream = nullptr;
    CallBackContext c;

    for (size_t jump = 1; jump <= 8; jump++) {
        err = hs_open_stream(db, 0, &stream);
        ASSERT_EQ(HS_SUCCESS, err);
        ASSERT_TRUE(stream != nullptr);

        for (unsigned i = 0; i < strlen(data); i += jump) {
            err = hs_scan_stream(stream, data + i, min(jump, strlen(data) - i),
                                 0, scratch, record_cb, (void *)&c);
            ASSERT_EQ(HS_SUCCESS, err);
        }
        EXPECT_EQ(0U, countMatchesById(c.matches, 1));
        EXPECT_EQ(30U, countMatchesById(c.matches, 2));
        EXPECT_EQ(29U, countMatchesById(c.matches, 3));
        EXPECT_EQ(5U, countMatchesById(c.matches, 4));
        EXPECT_EQ(5U, countMatchesById(c.matches, 5));
        ASSERT_TRUE(matchesOrdered(c.matches));
        c.matches.clear();
        hs_close_stream(stream, scratch, record_cb, (void *)&c);
    }
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(order, ordering7) {
    vector<pattern> patterns;
    patterns.push_back(pattern("aa.", HS_FLAG_DOTALL, 2));
    patterns.push_back(pattern("aa..", HS_FLAG_DOTALL, 3));
    patterns.push_back(pattern("^.{0,4}aa..", HS_FLAG_DOTALL, 4));

    const char *data = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";

    hs_database_t *db = buildDB(patterns, HS_MODE_STREAM);
    ASSERT_NE(nullptr, db);

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_stream_t *stream = nullptr;
    CallBackContext c;

    for (size_t jump = 1; jump <= 8; jump++) {
        err = hs_open_stream(db, 0, &stream);
        ASSERT_EQ(HS_SUCCESS, err);
        ASSERT_TRUE(stream != nullptr);

        for (unsigned i = 0; i < strlen(data); i += jump) {
            err = hs_scan_stream(stream, data + i, min(jump, strlen(data) - i),
                                 0, scratch, record_cb, (void *)&c);
            ASSERT_EQ(HS_SUCCESS, err);
        }
        EXPECT_EQ(0U, countMatchesById(c.matches, 1));
        EXPECT_EQ(30U, countMatchesById(c.matches, 2));
        EXPECT_EQ(29U, countMatchesById(c.matches, 3));
        EXPECT_EQ(5U, countMatchesById(c.matches, 4));
        EXPECT_EQ(0U, countMatchesById(c.matches, 5));
        ASSERT_TRUE(matchesOrdered(c.matches));
        c.matches.clear();
        hs_close_stream(stream, scratch, record_cb, (void *)&c);
    }
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(order, ordering8) {
    vector<pattern> patterns;
    patterns.push_back(pattern("aa", HS_FLAG_DOTALL, 1));
    patterns.push_back(pattern("aa.", HS_FLAG_DOTALL, 2));
    patterns.push_back(pattern("aa..", HS_FLAG_DOTALL, 3));

    const char *data = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";

    hs_database_t *db = buildDB(patterns, HS_MODE_STREAM);
    ASSERT_NE(nullptr, db);

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_stream_t *stream = nullptr;
    CallBackContext c;

    for (size_t jump = 1; jump <= 8; jump++) {
        err = hs_open_stream(db, 0, &stream);
        ASSERT_EQ(HS_SUCCESS, err);
        ASSERT_TRUE(stream != nullptr);

        for (unsigned i = 0; i < strlen(data); i += jump) {
            err = hs_scan_stream(stream, data + i, min(jump, strlen(data) - i),
                                 0, scratch, record_cb, (void *)&c);
            ASSERT_EQ(HS_SUCCESS, err);
        }
        EXPECT_EQ(31U, countMatchesById(c.matches, 1));
        EXPECT_EQ(30U, countMatchesById(c.matches, 2));
        EXPECT_EQ(29U, countMatchesById(c.matches, 3));
        EXPECT_EQ(0U, countMatchesById(c.matches, 4));
        EXPECT_EQ(0U, countMatchesById(c.matches, 5));
        ASSERT_TRUE(matchesOrdered(c.matches));
        c.matches.clear();
        hs_close_stream(stream, scratch, record_cb, (void *)&c);
    }
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

}
