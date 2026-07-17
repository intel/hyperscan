/*
 * Copyright (c) 2017, Intel Corporation
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
#include "test_util.h"
#include "gtest/gtest.h"

#include <iomanip>
#include <sstream>
#include <string>
#include <boost/random.hpp>

using namespace std;
using namespace testing;

class HyperscanLiteralTest
    : public TestWithParam<tuple<unsigned /* hyperscan mode */,
                                 unsigned /* flags to apply to all patterns */,
                                 unsigned /* number of literals */,
                                 pair<unsigned, unsigned> /* len min,max */,
                                 bool /* add non-literal case */>> {
protected:
    virtual void SetUp() {
        tie(mode, all_flags, num, bounds, add_non_literal) = GetParam();
        rng.seed(29785643);

        if (mode & HS_MODE_STREAM && all_flags & HS_FLAG_SOM_LEFTMOST) {
            mode |= HS_MODE_SOM_HORIZON_LARGE;
        }
    }

    // Returns (regex, corpus)
    pair<string, string> random_lit(unsigned min_len, unsigned max_len) {
        boost::random::uniform_int_distribution<> len_dist(min_len, max_len);
        size_t len = len_dist(rng);

        // Limit alphabet to [a-z] so that caseless tests include only alpha
        // chars and can be entirely caseless.
        boost::random::uniform_int_distribution<> dist('a', 'z');

        ostringstream oss;
        string corpus;
        for (size_t i = 0; i < len; i++) {
            char c = dist(rng);
            oss << "\\x" << std::hex << std::setw(2) << std::setfill('0')
                << ((unsigned)c & 0xff);
            corpus.push_back(c);
        }
        return {oss.str(), corpus};
    }

    virtual void TearDown() {}

    boost::random::mt19937 rng;
    unsigned mode;
    unsigned all_flags;
    unsigned num;
    pair<unsigned, unsigned> bounds;
    bool add_non_literal;
};

static
int count_cb(unsigned, unsigned long long, unsigned long long, unsigned,
             void *ctxt) {
    size_t *count = (size_t *)ctxt;
    (*count)++;
    return 0;
}

static
void do_scan_block(const vector<string> &corpora, const hs_database_t *db,
                   hs_scratch_t *scratch) {
    size_t count = 0;
    for (const auto &s : corpora) {
        size_t before = count;
        hs_error_t err =
            hs_scan(db, s.c_str(), s.size(), 0, scratch, count_cb, &count);
        ASSERT_EQ(HS_SUCCESS, err);
        ASSERT_LT(before, count);
    }
}

static
void do_scan_stream(const vector<string> &corpora, const hs_database_t *db,
                    hs_scratch_t *scratch) {
    size_t count = 0;
    for (const auto &s : corpora) {
        size_t before = count;
        hs_stream_t *stream = nullptr;
        hs_error_t err = hs_open_stream(db, 0, &stream);
        ASSERT_EQ(HS_SUCCESS, err);
        err = hs_scan_stream(stream, s.c_str(), s.size(), 0, scratch, count_cb,
                             &count);
        ASSERT_EQ(HS_SUCCESS, err);
        ASSERT_LT(before, count);
        err = hs_close_stream(stream, scratch, dummy_cb, nullptr);
        ASSERT_EQ(HS_SUCCESS, err);
    }
}

static
void do_scan_vectored(const vector<string> &corpora, const hs_database_t *db,
                      hs_scratch_t *scratch) {
    size_t count = 0;
    for (const auto &s : corpora) {
        size_t before = count;
        const char *const data[] = {s.c_str()};
        const unsigned int data_len[] = {(unsigned int)s.size()};
        hs_error_t err = hs_scan_vector(db, data, data_len, 1, 0, scratch,
                                        count_cb, &count);
        ASSERT_EQ(HS_SUCCESS, err);
        ASSERT_LT(before, count);
    }
}

static
void do_scan(unsigned mode, const vector<string> &corpora,
             const hs_database_t *db) {
    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    if (mode & HS_MODE_BLOCK) {
        do_scan_block(corpora, db, scratch);
    } else if (mode & HS_MODE_STREAM) {
        do_scan_stream(corpora, db, scratch);
    } else if (mode & HS_MODE_VECTORED) {
        do_scan_vectored(corpora, db, scratch);
    }

    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
}

TEST_P(HyperscanLiteralTest, Caseful) {
    vector<pattern> patterns;
    vector<string> corpora;
    for (unsigned i = 0; i < num; i++) {
        auto r = random_lit(bounds.first, bounds.second);
        unsigned flags = all_flags;
        patterns.emplace_back(std::move(r.first), flags, i);
        corpora.emplace_back(std::move(r.second));
    }

    if (add_non_literal) {
        patterns.emplace_back("hatstand.*teakettle", 0, num + 1);
        corpora.push_back("hatstand teakettle");
    }

    auto *db = buildDB(patterns, mode);
    ASSERT_TRUE(db != nullptr);

    do_scan(mode, corpora, db);

    hs_free_database(db);
}

TEST_P(HyperscanLiteralTest, Caseless) {
    vector<pattern> patterns;
    vector<string> corpora;
    for (unsigned i = 0; i < num; i++) {
        auto r = random_lit(bounds.first, bounds.second);
        unsigned flags = all_flags | HS_FLAG_CASELESS;
        patterns.emplace_back(std::move(r.first), flags, i);
        corpora.emplace_back(std::move(r.second));
    }

    if (add_non_literal) {
        patterns.emplace_back("hatstand.*teakettle", 0, num + 1);
        corpora.push_back("hatstand teakettle");
    }

    auto *db = buildDB(patterns, mode);
    ASSERT_TRUE(db != nullptr);

    do_scan(mode, corpora, db);

    hs_free_database(db);
}

TEST_P(HyperscanLiteralTest, MixedCase) {
    vector<pattern> patterns;
    vector<string> corpora;
    for (unsigned i = 0; i < num; i++) {
        auto r = random_lit(bounds.first, bounds.second);
        unsigned flags = all_flags;
        if (i % 2) {
            flags |= HS_FLAG_CASELESS;
        }
        patterns.emplace_back(std::move(r.first), flags, i);
        corpora.emplace_back(std::move(r.second));
    }

    if (add_non_literal) {
        patterns.emplace_back("hatstand.*teakettle", 0, num + 1);
        corpora.push_back("hatstand teakettle");
    }

    auto *db = buildDB(patterns, mode);
    ASSERT_TRUE(db != nullptr);

    do_scan(mode, corpora, db);

    hs_free_database(db);
}

static const unsigned test_modes[] = {HS_MODE_BLOCK, HS_MODE_STREAM,
                                      HS_MODE_VECTORED};

static const unsigned test_flags[] = {0, HS_FLAG_SINGLEMATCH,
                                      HS_FLAG_SOM_LEFTMOST};

static const unsigned test_sizes[] = {1, 10, 100, 500, 10000};

static const pair<unsigned, unsigned> test_bounds[] = {{3u, 10u}, {10u, 100u}};

INSTANTIATE_TEST_CASE_P(LiteralTest, HyperscanLiteralTest,
                        Combine(ValuesIn(test_modes), ValuesIn(test_flags),
                                ValuesIn(test_sizes), ValuesIn(test_bounds),
                                Bool()));
