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

#include <cstring>

#include "gtest/gtest.h"
#include "hs.h"
#include "test_util.h"

using namespace std;
using namespace testing;

TEST(ExtParam, LargeMinOffset) {
    hs_expr_ext ext;
    memset(&ext, 0, sizeof(ext));
    ext.min_offset = 100000;
    ext.flags = HS_EXT_FLAG_MIN_OFFSET;

    pattern p("hatstand.*teakettle", 0, 0, ext);
    hs_database_t *db = buildDB(p, HS_MODE_NOSTREAM);
    ASSERT_TRUE(db != nullptr);

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    CallBackContext c;

    // First, scan a matching corpus that's shorter than our min_offset and
    // ensure it doesn't match.
    string corpus = "hatstand" + string(80000, '_') + "teakettle";
    err = hs_scan(db, corpus.c_str(), corpus.length(), 0, scratch, record_cb,
                  (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(0U, c.matches.size());

    // Try exactly at the min_offset.
    corpus = "hatstand" + string(99983, '_') + "teakettle";
    err = hs_scan(db, corpus.c_str(), corpus.length(), 0, scratch, record_cb,
                  (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(1U, c.matches.size());
    ASSERT_EQ(MatchRecord(100000, 0), c.matches[0]);

    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(ExtParam, LargeExactOffset) {
    hs_expr_ext ext;
    memset(&ext, 0, sizeof(ext));
    ext.min_offset = 200000;
    ext.max_offset = 200000;
    ext.flags = HS_EXT_FLAG_MIN_OFFSET | HS_EXT_FLAG_MAX_OFFSET;

    pattern p("hatstand.*teakettle", 0, 0, ext);
    hs_database_t *db = buildDB(p, HS_MODE_NOSTREAM);
    ASSERT_TRUE(db != nullptr);

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    CallBackContext c;

    // First, scan a matching corpus that's shorter than our min_offset and
    // ensure it doesn't match.
    string corpus = "hatstand" + string(199982, '_') + "teakettle";
    err = hs_scan(db, corpus.c_str(), corpus.length(), 0, scratch, record_cb,
                  (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(0U, c.matches.size());

    // Try the exact match.
    corpus = "hatstand" + string(199983, '_') + "teakettle";
    err = hs_scan(db, corpus.c_str(), corpus.length(), 0, scratch, record_cb,
                  (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(1U, c.matches.size());
    ASSERT_EQ(MatchRecord(200000, 0), c.matches[0]);

    // Try one byte too far.
    c.clear();
    corpus = "hatstand" + string(199984, '_') + "teakettle";
    err = hs_scan(db, corpus.c_str(), corpus.length(), 0, scratch, record_cb,
                  (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(0U, c.matches.size());

    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(ExtParam, LargeMinLength) {
    hs_expr_ext ext;
    memset(&ext, 0, sizeof(ext));
    ext.min_length = 100000;
    ext.flags = HS_EXT_FLAG_MIN_LENGTH;

    pattern p("hatstand.*teakettle", 0, 0, ext);
    hs_database_t *db = buildDB(p, HS_MODE_NOSTREAM);
    ASSERT_TRUE(db != nullptr);

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    CallBackContext c;

    // First, scan a matching corpus that contains a match that's a bit too
    // short.
    string corpus = string(10000, '_') + "hatstand" + string(80000, '_') + "teakettle";
    err = hs_scan(db, corpus.c_str(), corpus.length(), 0, scratch, record_cb,
                  (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(0U, c.matches.size());

    // Now, a match of the right length.
    corpus = string(10000, '_') + "hatstand" + string(99983, '_') + "teakettle";
    err = hs_scan(db, corpus.c_str(), corpus.length(), 0, scratch, record_cb,
                  (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(1U, c.matches.size());
    ASSERT_EQ(MatchRecord(110000, 0), c.matches[0]);

    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}
