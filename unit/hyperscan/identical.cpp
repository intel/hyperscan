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
#include "test_util.h"
#include "hs.h"

namespace {

struct PatternInfo {
    std::string expr;
    unsigned flags;
    std::string corpus;
    unsigned long long match;
};

class IdenticalTest : public testing::TestWithParam<PatternInfo> {};

TEST_P(IdenticalTest, Block) {
    const PatternInfo &info = GetParam();

    std::vector<pattern> patterns;
    for (unsigned i = 0; i < 100; i++) {
        patterns.push_back(pattern(info.expr, info.flags, i));
    }

    hs_database_t *db = buildDB(patterns, HS_MODE_BLOCK);
    ASSERT_NE(nullptr, db);

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(nullptr, scratch);

    CallBackContext cb;
    err = hs_scan(db, info.corpus.c_str(), info.corpus.size(), 0, scratch,
                  record_cb, &cb);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);

    ASSERT_EQ(patterns.size(), cb.matches.size());

    std::set<unsigned> ids;
    for (size_t i = 0; i < cb.matches.size(); i++) {
        ASSERT_EQ(info.match, cb.matches[i].to);
        ids.insert(cb.matches[i].id);
    }

    ASSERT_EQ(patterns.size(), ids.size());
    ASSERT_EQ(0, *ids.begin());
    ASSERT_EQ(patterns.size() - 1, *ids.rbegin());
}

TEST_P(IdenticalTest, Stream) {
    const PatternInfo &info = GetParam();

    std::vector<pattern> patterns;
    for (unsigned i = 0; i < 100; i++) {
        patterns.push_back(pattern(info.expr, info.flags, i));
    }

    hs_database_t *db =
        buildDB(patterns, HS_MODE_STREAM | HS_MODE_SOM_HORIZON_LARGE);
    ASSERT_NE(nullptr, db);

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(nullptr, scratch);

    CallBackContext cb;
    hs_stream_t *stream = nullptr;

    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(nullptr, stream);

    err = hs_scan_stream(stream, info.corpus.c_str(), info.corpus.size(), 0,
                         scratch, record_cb, &cb);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_close_stream(stream, scratch, record_cb, &cb);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);

    ASSERT_EQ(patterns.size(), cb.matches.size());

    std::set<unsigned> ids;
    for (size_t i = 0; i < cb.matches.size(); i++) {
        ASSERT_EQ(info.match, cb.matches[i].to);
        ids.insert(cb.matches[i].id);
    }

    ASSERT_EQ(patterns.size(), ids.size());
    ASSERT_EQ(0, *ids.begin());
    ASSERT_EQ(patterns.size() - 1, *ids.rbegin());
}

TEST_P(IdenticalTest, Vectored) {
    const PatternInfo &info = GetParam();

    std::vector<pattern> patterns;
    for (unsigned i = 0; i < 100; i++) {
        patterns.push_back(pattern(info.expr, info.flags, i));
    }

    hs_database_t *db = buildDB(patterns, HS_MODE_VECTORED);
    ASSERT_NE(nullptr, db);

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(nullptr, scratch);

    CallBackContext cb;

    const char * const data[] = { info.corpus.c_str() };
    const unsigned datalen[] = { (unsigned)info.corpus.size() };

    err = hs_scan_vector(db, data, datalen, 1, 0, scratch, record_cb, &cb);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);

    ASSERT_EQ(patterns.size(), cb.matches.size());

    std::set<unsigned> ids;
    for (size_t i = 0; i < cb.matches.size(); i++) {
        ASSERT_EQ(info.match, cb.matches[i].to);
        ids.insert(cb.matches[i].id);
    }

    ASSERT_EQ(patterns.size(), ids.size());
    ASSERT_EQ(0, *ids.begin());
    ASSERT_EQ(patterns.size() - 1, *ids.rbegin());
}

static const PatternInfo patterns[] = {
    { "a", 0, "a", 1 },
    { "a", HS_FLAG_SINGLEMATCH, "a", 1 },
    { "handbasket", 0, "__handbasket__", 12 },
    { "handbasket", HS_FLAG_SINGLEMATCH, "__handbasket__", 12 },
    { "handbasket", HS_FLAG_SOM_LEFTMOST, "__handbasket__", 12 },
    { "foo.*bar", 0, "a foolish embarrassment", 15 },
    { "foo.*bar", HS_FLAG_SINGLEMATCH, "a foolish embarrassment", 15 },
    { "foo.*bar", HS_FLAG_SOM_LEFTMOST, "a foolish embarrassment", 15 },
    { "\\bword\\b(..)+\\d{3,7}", 0, "    word    012", 15 },
    { "\\bword\\b(..)+\\d{3,7}", HS_FLAG_SINGLEMATCH, "    word    012", 15 },
    { "\\bword\\b(..)+\\d{3,7}", HS_FLAG_SOM_LEFTMOST, "    word    012", 15 },
    { "eod\\z", 0, "eod", 3 },
    { "eod\\z", HS_FLAG_SINGLEMATCH, "eod", 3 },
    { "eod\\z", HS_FLAG_SOM_LEFTMOST, "eod", 3 },
};

INSTANTIATE_TEST_CASE_P(Identical, IdenticalTest, testing::ValuesIn(patterns));

// teach google-test how to print a param
void PrintTo(const PatternInfo &p, ::std::ostream *os) {
    *os << p.expr << ":" << p.flags << ", " << p.corpus;
}

} // namespace
