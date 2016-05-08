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
#include <vector>

#include "gtest/gtest.h"
#include "hs.h"
#include "test_util.h"

using namespace std;
using namespace testing;

namespace {
struct Match {
    Match(unsigned i, unsigned long long f, unsigned long long t) :
                id(i), from(f), to(t) {}
    unsigned int id;
    unsigned long long from;
    unsigned long long to;
};
}

static
int vectorCallback(unsigned id, unsigned long long from,
                   unsigned long long to, unsigned, void *ctx) {
    //printf("match id %u at (%llu,%llu)\n", id, from, to);
    vector<Match> *matches = (vector<Match> *)ctx;
    matches->push_back(Match(id, from, to));
    return 0;
}

class SomTest : public TestWithParam<unsigned int> {
protected:
    virtual void SetUp() {
        som_mode = GetParam();
    }

    unsigned long long getSomHorizon() const {
        switch (som_mode) {
            case HS_MODE_SOM_HORIZON_SMALL:
                return 1ULL << 16;
            case HS_MODE_SOM_HORIZON_MEDIUM:
                return 1ULL << 32;
            default:
                return ~(0ULL);
        }
    }

    unsigned int som_mode;
};

TEST_P(SomTest, PastHorizon) {
    hs_database_t *db = buildDB("foo.*bar", HS_FLAG_SOM_LEFTMOST, 1000,
                                HS_MODE_STREAM | som_mode);
    ASSERT_TRUE(db != nullptr);

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_TRUE(scratch != nullptr);

    vector<Match> matches;

    hs_stream_t *stream = nullptr;
    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    const string prefix(" foo");
    const string suffix("bar");
    const string filler(4096, 'X');

    unsigned long long scanned_len = 0;

    err = hs_scan_stream(stream, prefix.c_str(), prefix.length(), 0, scratch,
                         vectorCallback, &matches);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(0, matches.size());
    scanned_len += prefix.length();

    const unsigned long long blocks = getSomHorizon() / filler.length();
    for (unsigned long long i = 0; i < blocks; i += 1) {
        err = hs_scan_stream(stream, filler.c_str(), filler.length(), 0,
                             scratch, vectorCallback, &matches);
        ASSERT_EQ(HS_SUCCESS, err);
        ASSERT_EQ(0, matches.size());
        scanned_len += filler.length();
    }

    err = hs_scan_stream(stream, suffix.c_str(), suffix.length(), 0, scratch,
                         vectorCallback, &matches);
    ASSERT_EQ(HS_SUCCESS, err);
    scanned_len += suffix.length();

    // We receive one match with the correct 'to' offset, but a sentinel value
    // for 'from'.

    ASSERT_EQ(1, matches.size());
    ASSERT_EQ(1000, matches[0].id);
    ASSERT_EQ(HS_OFFSET_PAST_HORIZON, matches[0].from);
    ASSERT_EQ(scanned_len, matches[0].to);

    err = hs_close_stream(stream, scratch, vectorCallback, &matches);
    ASSERT_EQ(HS_SUCCESS, err);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST_P(SomTest, NearHorizon) {
    hs_database_t *db = buildDB("foo.*bar", HS_FLAG_SOM_LEFTMOST, 1000,
                                HS_MODE_STREAM | som_mode);
    ASSERT_TRUE(db != nullptr);

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_TRUE(scratch != nullptr);

    vector<Match> matches;

    hs_stream_t *stream = nullptr;
    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    const string prefix(" foo");
    const string suffix("bar");
    const string filler(4096, 'X');

    unsigned long long scanned_len = 0;

    err = hs_scan_stream(stream, prefix.c_str(), prefix.length(), 0, scratch,
                         vectorCallback, &matches);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(0, matches.size());
    scanned_len += prefix.length();

    const unsigned long long blocks = getSomHorizon() / filler.length() - 1;
    for (unsigned long long i = 0; i < blocks; i += 1) {
        err = hs_scan_stream(stream, filler.c_str(), filler.length(), 0,
                             scratch, vectorCallback, &matches);
        ASSERT_EQ(HS_SUCCESS, err);
        ASSERT_EQ(0, matches.size());
        scanned_len += filler.length();
    }

    err = hs_scan_stream(stream, suffix.c_str(), suffix.length(), 0, scratch,
                         vectorCallback, &matches);
    ASSERT_EQ(HS_SUCCESS, err);
    scanned_len += suffix.length();

    // Both 'to' and 'from' should be accurate.

    ASSERT_EQ(1, matches.size());
    ASSERT_EQ(1000, matches[0].id);
    ASSERT_EQ(1, matches[0].from);
    ASSERT_EQ(scanned_len, matches[0].to);

    err = hs_close_stream(stream, scratch, vectorCallback, &matches);
    ASSERT_EQ(HS_SUCCESS, err);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

INSTANTIATE_TEST_CASE_P(Som, SomTest,
                        Values(HS_MODE_SOM_HORIZON_SMALL,
                               HS_MODE_SOM_HORIZON_MEDIUM));

