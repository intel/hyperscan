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
#include <algorithm>
#include <climits>
#include <sstream>
#include <string>

#include "gtest/gtest.h"
#include "hs.h"
#include "test_util.h"

using namespace std;
using namespace testing;

namespace /* anonymous */ {

// Dummy callback: does nothing, returns 0 (keep matching)
int dummyHandler(unsigned, unsigned long long, unsigned long long,
                 unsigned, void *) {
    // empty
    return 0;
}

unsigned lastMatchId = 0;
unsigned long long lastMatchFrom = 0;
unsigned long long lastMatchTo = 0;
unsigned lastMatchFlags = 0;
void *lastMatchCtx = nullptr;

// Single match Callback: record all the details from a single match
int singleHandler(unsigned id, unsigned long long from,
                  unsigned long long to, unsigned flags, void *ctx) {
    lastMatchId = id;
    lastMatchFrom = from;
    lastMatchTo = to;
    lastMatchFlags = flags;
    lastMatchCtx = ctx;
    return 0;
}

unsigned matchCount = 0;

// Counter callback: just counts the number of calls
int countHandler(unsigned, unsigned long long, unsigned long long,
                 unsigned, void *) {
    matchCount++;
    return 0;
}

// Stop handler: take one match and tell Hyperscan to not deliver any more.
int stopHandler(unsigned, unsigned long long, unsigned long long,
                unsigned, void *) {
    matchCount++;
    return 1;
}

// Can we scan 5GB of data (pushing past the uint32_t limit)
TEST(HyperscanTestBehaviour, ScanSeveralGigabytesNoMatch) {
    hs_error_t err;
    const size_t datalen = 1024 * 1024;
    size_t megabytes = 5 * 1024;
    vector<char> data(datalen, 'X');

    // build a database
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    err = hs_compile("hatstand.*teakettle.*badgerbrush",
                     HS_FLAG_DOTALL, HS_MODE_STREAM, nullptr, &db, &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);

    // alloc some scratch
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_TRUE(scratch != nullptr);

    // streaming mode scan of our megabyte of data 5K times
    hs_stream_t *stream = nullptr;

    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    while (megabytes-- > 0) {
        err = hs_scan_stream(stream, data.data(), data.size(), 0, scratch,
                             dummyHandler, nullptr);
        ASSERT_EQ(HS_SUCCESS, err);
    }

    err = hs_close_stream(stream, scratch, dummyHandler, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

struct HugeScanMatchingData {
    const char *pattern;
    unsigned int flags;
    const char *preBlock; // scan before 5GB of X's
    const char *postBlock; // scan after 5GB of X's, single match at end
};

class HyperscanScanGigabytesMatch : public TestWithParam<HugeScanMatchingData> {
};

// Can we scan 5GB of data (pushing past the uint32_t limit) and get a real match
TEST_P(HyperscanScanGigabytesMatch, StreamingMatch) {
    const HugeScanMatchingData &params = GetParam();
    SCOPED_TRACE(params.pattern);

    hs_error_t err;
    const size_t datalen = 1024*1024;
    vector<char> data(datalen, 'X');

    // build a database
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    err = hs_compile(params.pattern, params.flags, HS_MODE_STREAM, nullptr, &db,
                     &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);

    // alloc some scratch
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_TRUE(scratch != nullptr);

    // gb is the number of gigabytes to scan between pre-block and post-block
    // run over 1,2,4,8 gb
    for (unsigned long long gb = 1; gb <= 8; gb *= 2) {
        SCOPED_TRACE(gb);

        hs_stream_t *stream = nullptr;

        err = hs_open_stream(db, 0, &stream);
        ASSERT_EQ(HS_SUCCESS, err);
        ASSERT_TRUE(stream != nullptr);

        // scan our pre-block
        lastMatchTo = 0;
        err = hs_scan_stream(stream, params.preBlock, strlen(params.preBlock),
                             0, scratch, singleHandler, nullptr);
        ASSERT_EQ(HS_SUCCESS, err);
        ASSERT_EQ(0ULL, lastMatchTo);

        // streaming mode scan of our megabyte of data gb*1024 times
        unsigned long remaining = gb * 1024;
        while (remaining-- > 0) {
            err = hs_scan_stream(stream, data.data(), data.size(), 0, scratch,
                                 singleHandler, nullptr);
            ASSERT_EQ(HS_SUCCESS, err);
            ASSERT_EQ(0ULL, lastMatchTo);
        }

        // scan post-block and close stream
        err = hs_scan_stream(stream, params.postBlock, strlen(params.postBlock),
                             0, scratch, singleHandler, nullptr);
        ASSERT_EQ(HS_SUCCESS, err);

        err = hs_close_stream(stream, scratch, singleHandler, nullptr);
        ASSERT_EQ(HS_SUCCESS, err);

        // make sure we got our match
        unsigned long long totalLen = strlen(params.preBlock) +
                                      gb * 1024 * datalen +
                                      strlen(params.postBlock);
        ASSERT_EQ(totalLen, lastMatchTo);
    }

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

// Helper function to actually perform scans for BlockMatch test below
void allocAndScanBlock(const HugeScanMatchingData &params,
                       const hs_database_t *db, hs_scratch_t *scratch,
                       unsigned int len) {
    SCOPED_TRACE(len);

    char *data = (char *)calloc(len, 1);
    if (data == nullptr) {
        printf("Can't allocate %u bytes, skipping test\n", len);
        return;
    }

    // write pre-block at the start and post-block at the end
    memcpy(data, params.preBlock, strlen(params.preBlock));
    memcpy(data+len - strlen(params.postBlock), params.postBlock,
            strlen(params.postBlock));

    lastMatchTo = 0;
    hs_error_t err;
    err = hs_scan(db, data, len, 0, scratch, singleHandler, nullptr);
    free(data);
    ASSERT_EQ(HS_SUCCESS, err);

    // make sure we got our match
    ASSERT_EQ(len, lastMatchTo);
}

// Define to run even more huge block tests
//#define BIG_BLOCKS

// Can we scan gigabytes of data in a block and get a real match
TEST_P(HyperscanScanGigabytesMatch, BlockMatch) {
    const HugeScanMatchingData &params = GetParam();
    SCOPED_TRACE(params.pattern);

    hs_error_t err;

    // build a database
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    err = hs_compile(params.pattern, params.flags, HS_MODE_BLOCK, nullptr, &db,
                     &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);

    // alloc some scratch
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_TRUE(scratch != nullptr);

    // try various data sizes (in kilobytes)
    size_t blockSizes[] = {
        // kilobytes
        1, 4, 8, 16, 32, 64, 128, 256, 512,
        // megabytes
        1*1024,
#ifdef BIG_BLOCKS
        4*1024, 32*1024, 128*1024, 512*1024,
        // gigabytes
        1024*1024,
#ifdef ARCH_X86_64
        // big cases for big beefy machines
        2048*1024, 3072*1024
#endif // ARCH_X86_64
#endif // BIG_BLOCKS
    };

    for (const auto size : blockSizes) {
        size_t bytes = size * 1024;

        // The block API only accepts blocks up to UINT_MAX in size.
        ASSERT_GE(UINT_MAX, bytes);
        ASSERT_GE(UINT_MAX, bytes + strlen(params.postBlock));

        // First test: block size exactly.
        allocAndScanBlock(params, db, scratch, bytes);

        // Second test: block size plus four bytes, to make our post-block
        // straddle the block size boundary.
        allocAndScanBlock(params, db, scratch, bytes + 4);

        // Third test: block size plus the length of post-block.
        allocAndScanBlock(params, db, scratch, bytes + strlen(params.postBlock));
    }

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

static const HugeScanMatchingData gigTests[] = {
    { "foobar", 0, "flibble", "foobar" },
    { "longliteralislongerthanlong", 0, "precursor", "longliteralislongerthanlong" },
    { "foobar\\z", 0, "flibble", "foobar" },
    { "hatstand.*teakettle.*badgerbrush", HS_FLAG_DOTALL, "hatstand teakettle", "_badgerbrush" },
    { "hatstand.*teakettle.*badgerbrush\\z", HS_FLAG_DOTALL, "hatstand teakettle", "_badgerbrush" },
    { "a.*(([0123][56789]){3,6}|flibble|xyz{1,2}y)", 0, "a", "051629" },
    { "^a.*(([0123][56789]){3,6}|flibble|xyz{1,2}y)", 0, "a", "051629" },
    { "(badger.*){3,}mushroom.*mushroom", HS_FLAG_DOTALL, "badger badger badger", "mushroom! mushroom" },
    { "(badger.*){3,}mushroom.*mushroom$", HS_FLAG_DOTALL, "badger badger badger", "mushroom! mushroom" },
    { "foo[^X]{16}", HS_FLAG_SINGLEMATCH, "preblock", "foo0123456789abcdef" },
    { "foobar.*bazbaz[^X]{16}", HS_FLAG_DOTALL|HS_FLAG_SINGLEMATCH, "foobar", "bazbaz0123456789abcdef" },
    { "t?((w.(u|t|.)){6,10}|[dqnt]|e|va)", 0, "~~~~", "tva" },
    { "HTTP.*foobar.*blah", HS_FLAG_DOTALL, "bing", "HTTP foobar blah" },
};

INSTANTIATE_TEST_CASE_P(HyperscanTestBehaviour, HyperscanScanGigabytesMatch,
                        ValuesIn(gigTests));

// Do we actually only get a single match back from HS_FLAG_SINGLEMATCH?
TEST(HyperscanTestBehaviour, StreamingThereCanBeOnlyOne) {
    hs_error_t err;
    const std::string data1("hackhackHACKhackHACkhAcK");
    const std::string data2("KHACKhackHACKhaCkhAcKhaaaaaaaaaaaaaaaaaaaackk");

    // build a database
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    err = hs_compile(".ck",
                     HS_FLAG_CASELESS | HS_FLAG_SINGLEMATCH,
                     HS_MODE_STREAM, nullptr, &db, &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);

    // alloc some scratch
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_TRUE(scratch != nullptr);

    hs_stream_t *stream = nullptr;
    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    matchCount = 0;
    err = hs_scan_stream(stream, data1.c_str(), data1.size(), 0, scratch,
                         countHandler, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_EQ(1U, matchCount); // only one match
    err = hs_scan_stream(stream, data2.c_str(), data2.size(), 0, scratch,
                         countHandler, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_EQ(1U, matchCount); // just the one from the first call above

    err = hs_close_stream(stream, scratch, countHandler, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

// Do we actually only get a single match back from HS_FLAG_SINGLEMATCH?
TEST(HyperscanTestBehaviour, BlockThereCanBeOnlyOne) {
    hs_error_t err;
    const std::string data1("hackhackHACKhackHACkhAcKzofunvuynlslijlnikshvb ar yhtkubq45ytvb iuyh "
                            "ackeruniou viytdfjhg nvldkrjgnal");

    // build a database
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    err = hs_compile(".ck",
                     HS_FLAG_CASELESS | HS_FLAG_SINGLEMATCH,
                     HS_MODE_BLOCK, nullptr, &db, &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);

    // alloc some scratch
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_TRUE(scratch != nullptr);

    matchCount = 0;
    err = hs_scan(db, data1.c_str(), data1.size(), 0, scratch, countHandler, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_EQ(1U, matchCount); // only one match

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

class HyperscanLiteralLengthTest : public TestWithParam<size_t> {
protected:
    virtual void SetUp() {
        literal_len = GetParam();
    }

    size_t literal_len;
};

TEST_P(HyperscanLiteralLengthTest, FloatingBlock) {
    SCOPED_TRACE(literal_len);

    const string pattern(literal_len, 'a');
    const string data(literal_len + 4, 'a');
    SCOPED_TRACE(pattern);

    // build a database
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err;
    err = hs_compile(pattern.c_str(), 0, HS_MODE_BLOCK, nullptr, &db,
                     &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);

    // alloc some scratch
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_TRUE(scratch != nullptr);

    matchCount = 0;
    err = hs_scan(db, data.c_str(), data.size(), 0, scratch, countHandler, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_EQ(5U, matchCount); // five matches

    // Should see no match from five bytes in
    matchCount = 0;
    err = hs_scan(db, data.c_str() + 5, data.size() - 5, 0, scratch, countHandler, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_EQ(0U, matchCount); // no matches

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST_P(HyperscanLiteralLengthTest, AnchoredBlock) {
    SCOPED_TRACE(literal_len);

    const string pattern = "^" + string(literal_len, 'a');
    const string data(literal_len + 4, 'a');
    SCOPED_TRACE(pattern);

    // build a database
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err;
    err = hs_compile(pattern.c_str(), 0, HS_MODE_BLOCK, nullptr, &db,
                     &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);

    // alloc some scratch
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_TRUE(scratch != nullptr);

    matchCount = 0;
    err = hs_scan(db, data.c_str(), data.size(), 0, scratch, countHandler, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_EQ(1U, matchCount); // one match

    // Should see no match from five bytes in
    matchCount = 0;
    err = hs_scan(db, data.c_str() + 5, data.size() - 5, 0, scratch, countHandler, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_EQ(0U, matchCount); // no matches


    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

INSTANTIATE_TEST_CASE_P(HyperscanLiteralLength, HyperscanLiteralLengthTest,
                        Values(1, 2, 3, 4, 8, 16, 17, 32, 100, 200, 400, 1000,
                               4096, 8192, 15000, 15999));

struct CallbackStopData {
   const char *pattern;
   unsigned int flags;
   const char *corpus;
};

class CallbackReturnStop : public TestWithParam<CallbackStopData> {
};

TEST_P(CallbackReturnStop, Block) {
    const CallbackStopData &params = GetParam();
    SCOPED_TRACE(params.pattern);

    // build a database
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err;
    err = hs_compile(params.pattern, params.flags,
                     HS_MODE_NOSTREAM, nullptr, &db, &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);

    // alloc some scratch
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_TRUE(scratch != nullptr);

    matchCount = 0;
    err = hs_scan(db, params.corpus, strlen(params.corpus), 0, scratch,
                  stopHandler, nullptr);
    ASSERT_EQ(1U, matchCount) << "One match exactly should be recorded.";
    ASSERT_EQ(HS_SCAN_TERMINATED, err);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST_P(CallbackReturnStop, Streaming) {
    const CallbackStopData &params = GetParam();
    SCOPED_TRACE(params.pattern);

    // build a database
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err;
    err = hs_compile(params.pattern, params.flags,
                     HS_MODE_STREAM, nullptr, &db, &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);

    // alloc some scratch
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_TRUE(scratch != nullptr);

    hs_stream_t *stream = nullptr;
    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    matchCount = 0;

    err = hs_scan_stream(stream, params.corpus, strlen(params.corpus),
                         0, scratch, stopHandler, nullptr);
    ASSERT_EQ(1U, matchCount) << "One match exactly should be recorded.";
    ASSERT_EQ(HS_SCAN_TERMINATED, err);

    err = hs_close_stream(stream, scratch, stopHandler, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST_P(CallbackReturnStop, Vectored) {
    const CallbackStopData &params = GetParam();
    SCOPED_TRACE(params.pattern);

    // build a database
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err;
    err = hs_compile(params.pattern, params.flags,
                     HS_MODE_VECTORED, nullptr, &db, &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);

    // alloc some scratch
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_TRUE(scratch != nullptr);

    matchCount = 0;

    const char *data[] = { params.corpus };
    unsigned int len[] = { (unsigned int)strlen(params.corpus) };

    err = hs_scan_vector(db, data, len, 1, 0, scratch, stopHandler, nullptr);
    ASSERT_EQ(1U, matchCount) << "One match exactly should be recorded.";
    ASSERT_EQ(HS_SCAN_TERMINATED, err);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

static const CallbackStopData callbackStopTests[] = {
    { "foobar", 0, "xxxfoobarxxxfoobarxxxfoobar" },
    { "a", 0, "xxxaaaaaaaaaaaaaaaaaaa" },
    { "a", HS_FLAG_CASELESS, "xxxAaAaAaAa" },
    { ".", 0, "acbdef" },
    { ".", HS_FLAG_DOTALL, "abcdef" },
    { "foo.*bar.*foo", HS_FLAG_DOTALL, "foobarfoobarfoobarfoobarfoo" },
    { "(badger.*){3,}", HS_FLAG_DOTALL, "badger badger badger badger badger badger badger badger" },
    { "foobar.*", 0, "foobarxxx" },
    { "[012]{3,7}.*abba", 0, "0120120120abbaabba" },
};

INSTANTIATE_TEST_CASE_P(HyperscanTestBehaviour, CallbackReturnStop,
                        ValuesIn(callbackStopTests));

// Serialization, then deserialization: A simple case, with just a single
// trivial pattern.
TEST(HyperscanTestBehaviour, SerializedDogfood1) {
    // build database
    hs_database_t *db = buildDB("puppy treats!", 0, 0, HS_MODE_BLOCK);
    ASSERT_TRUE(db != nullptr);

    hs_error_t err;
    size_t origSize;
    err = hs_database_size(db, &origSize);
    ASSERT_EQ(HS_SUCCESS, err);

    // serialise to buffer
    char *bytes = nullptr;
    size_t length = 0;
    err = hs_serialize_database(db, &bytes, &length);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(bytes != nullptr);
    ASSERT_LT(0U, length);

    // free original database
    hs_free_database(db);
    db = nullptr;

    // deserialize
    hs_database_t *db2 = nullptr;
    err = hs_deserialize_database(bytes, length, &db2);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db2 != nullptr);
    free(bytes);

    // tests
    size_t newSize;
    err = hs_database_size(db2, &newSize);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(newSize, origSize) << "Database lengths should match.";

    // check that we can get a match out of it
    lastMatchTo = 0;
    const char *data = "delicious puppy treats!";
    unsigned int len = strlen(data);
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db2, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    err = hs_scan(db2, data, len, 0, scratch, singleHandler, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(len, lastMatchTo);

    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db2);
}

// A more complicated case, with several patterns
TEST(HyperscanTestBehaviour, SerializedDogfood2) {
    // build database
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err;

    static const vector<const char *> expressions {
        "puppy treats!",
        "foobar",
        "foo.*bar.*foo",
        "(badger.*){3,}",
        "[012]{3,7}.*abba"
    };

    static const vector<unsigned int> flags {
        0,
        0,
        HS_FLAG_DOTALL,
        HS_FLAG_DOTALL,
        0
    };

    static const vector<unsigned int> ids {
        1000,
        1001,
        1002,
        1003,
        1004
    };

    size_t num = expressions.size();
    ASSERT_EQ(num, flags.size());
    ASSERT_EQ(num, ids.size());

    err = hs_compile_multi(expressions.data(), flags.data(), ids.data(), num,
                           HS_MODE_NOSTREAM, nullptr, &db, &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    size_t origSize;
    err = hs_database_size(db, &origSize);
    ASSERT_EQ(HS_SUCCESS, err);

    // serialise to buffer
    char *bytes = nullptr;
    size_t length = 0;
    err = hs_serialize_database(db, &bytes, &length);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(bytes != nullptr);
    ASSERT_LT(0U, length);

    // free original database
    hs_free_database(db);
    db = nullptr;

    // deserialize
    hs_database_t *db2 = nullptr;
    err = hs_deserialize_database(bytes, length, &db2);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db2 != nullptr);
    free(bytes);

    // tests
    size_t newSize;
    err = hs_database_size(db2, &newSize);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(newSize, origSize) << "Database lengths should match.";

    // check that we can get a match out of it
    lastMatchTo = 0;
    lastMatchId = 0;
    const char *data = "delicious puppy treats!";
    unsigned int len = strlen(data);
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db2, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    err = hs_scan(db2, data, len, 0, scratch, singleHandler, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(len, lastMatchTo);
    ASSERT_EQ(ids[0], lastMatchId);

    // check another pattern
    const char *data2 = "badger badger badger";
    unsigned int len2 = strlen(data2);
    err = hs_scan(db2, data2, len2, 0, scratch, singleHandler, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(len2, lastMatchTo);
    ASSERT_EQ(ids[3], lastMatchId);

    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db2);
}

// Serialization, then deserialization into a preallocated memory region. A
// simple case, with just a single trivial pattern.
TEST(HyperscanTestBehaviour, SerializedDogfood3) {
    // build database
    hs_database_t *db = buildDB("puppy treats!", 0, 0, HS_MODE_BLOCK);
    ASSERT_TRUE(db != nullptr);

    hs_error_t err;
    size_t origSize;
    err = hs_database_size(db, &origSize);
    ASSERT_EQ(HS_SUCCESS, err);

    // serialise to buffer
    char *bytes = nullptr;
    size_t length = 0;
    err = hs_serialize_database(db, &bytes, &length);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(bytes != nullptr);
    ASSERT_LT(0U, length);

    // free original database
    hs_free_database(db);
    db = nullptr;

    // find out how much space we need
    size_t slength;
    err = hs_serialized_database_size(bytes, length, &slength);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(slength, origSize) << "Database lengths should match.";

    // fail tests: try to deserialize into an incorrectly aligned region.
    char *mem = (char *)malloc(slength + 16);
    ASSERT_TRUE(mem != nullptr);
    ASSERT_EQ(0U, (ptrdiff_t)mem % 8);

    // add one for epic fail
    err = hs_deserialize_database_at(bytes, length, (hs_database_t *)(mem + 1));
    ASSERT_EQ(HS_BAD_ALIGN, err);

    // add four, which should give us 4-byte alignment which isn't good enough.
    err = hs_deserialize_database_at(bytes, length, (hs_database_t *)(mem + 4));
    ASSERT_EQ(HS_BAD_ALIGN, err);

    // deserialize for real
    err = hs_deserialize_database_at(bytes, length, (hs_database_t *)mem);
    ASSERT_EQ(HS_SUCCESS, err);
    free(bytes);

    const hs_database_t *db2 = (const hs_database_t *)mem;

    // tests
    size_t newSize;
    err = hs_database_size(db2, &newSize);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(newSize, origSize) << "Database lengths should match.";

    // check that we can get a match out of it
    lastMatchTo = 0;
    const char *data = "delicious puppy treats!";
    unsigned int len = strlen(data);
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db2, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    err = hs_scan(db2, data, len, 0, scratch, singleHandler, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(len, lastMatchTo);

    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    free(mem);
}

// Test that we do get a pending match on stream close.
TEST(HyperscanTestBehaviour, CloseStreamMatch) {
    hs_error_t err;

    // build a database
    hs_database_t *db = buildDB("foo.*bar$", 0, 0, HS_MODE_STREAM);
    ASSERT_TRUE(db != nullptr);

    // alloc some scratch
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_TRUE(scratch != nullptr);

    hs_stream_t *stream = nullptr;
    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    matchCount = 0;
    const string data("foo        bar");
    err = hs_scan_stream(stream, data.c_str(), data.size(), 0, scratch,
                         countHandler, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_EQ(0U, matchCount); // hasn't matched until stream end

    err = hs_close_stream(stream, scratch, countHandler, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_EQ(1U, matchCount); // our match was returned

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

/* Test that if we scan without a callback during main phase that we are still
 * tracking stuff internally */
TEST(HyperscanTestBehaviour, NoMainCB) {
    hs_error_t err;

    // build a database
    hs_database_t *db = buildDB("foo|foo.*bar$", 0, 0, HS_MODE_STREAM);
    ASSERT_TRUE(db != nullptr);

    // alloc some scratch
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_TRUE(scratch != nullptr);

    hs_stream_t *stream = nullptr;
    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    matchCount = 0;
    const string data("foo        bar");
    err = hs_scan_stream(stream, data.c_str(), data.size(), 0, scratch,
                         nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_EQ(0U, matchCount); // hasn't matched until stream end

    err = hs_close_stream(stream, scratch, countHandler, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_EQ(1U, matchCount); // our match was returned

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}


// Test that we don't get a match that's pending on stream close when we use
// hs_close_stream with no event handler.
TEST(HyperscanTestBehaviour, CloseStreamNoMatch) {
    hs_error_t err;

    // build a database
    hs_database_t *db = buildDB("foo.*bar$", 0, 0, HS_MODE_STREAM);
    ASSERT_TRUE(db != nullptr);

    // alloc some scratch
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_TRUE(scratch != nullptr);

    hs_stream_t *stream = nullptr;
    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    matchCount = 0;
    const string data("foo        bar");
    err = hs_scan_stream(stream, data.c_str(), data.size(), 0, scratch,
                         countHandler, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_EQ(0U, matchCount); // hasn't matched until stream end

    err = hs_close_stream(stream, scratch, nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_EQ(0U, matchCount); // no match was returned

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

// Test that we can close a stream correctly after we've terminated matching.
TEST(HyperscanTestBehaviour, CloseStreamAfterTermination) {
    hs_error_t err;

    // build a database
    hs_database_t *db = buildDB("foo.*(bar|baz\\z)", 0, 0, HS_MODE_STREAM);
    ASSERT_TRUE(db != nullptr);

    // alloc some scratch
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_TRUE(scratch != nullptr);

    hs_stream_t *stream = nullptr;
    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    matchCount = 0;
    const string data("foo        bar     baz");
    err = hs_scan_stream(stream, data.c_str(), data.size(), 0, scratch,
                         stopHandler, nullptr);
    ASSERT_EQ(HS_SCAN_TERMINATED, err);
    EXPECT_EQ(1U, matchCount); // after "bar"

    err = hs_close_stream(stream, scratch, stopHandler, nullptr);
    EXPECT_EQ(1U, matchCount); // no match for baz
    ASSERT_EQ(HS_SUCCESS, err);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(HyperscanTestBehaviour, Vectored1) {
    hs_error_t err;

    // build a database
    hs_database_t *db = buildDB("^foo.*bar", HS_FLAG_DOTALL, 0,
                                HS_MODE_VECTORED);
    ASSERT_TRUE(db != nullptr);

    // alloc some scratch
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_TRUE(scratch != nullptr);

    const char *data[] = { "foo", "   ", "bar" };
    unsigned int len[] = { 3, 3, 3};

    matchCount = 0;
    err = hs_scan_vector(db, data, len, 3, 0, scratch, countHandler, nullptr);

    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_EQ(1U, matchCount);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(HyperscanTestBehaviour, Vectored2) {
    hs_error_t err;

    // build a database
    hs_database_t *db = buildDB("^foo.*bar", HS_FLAG_DOTALL, 0,
                                HS_MODE_VECTORED);
    ASSERT_TRUE(db != nullptr);

    // alloc some scratch
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_TRUE(scratch != nullptr);

    const char *data[] = { "", "foo", "   ", "bar" };
    unsigned int len[] = { 0, 3, 3, 3};

    matchCount = 0;
    err = hs_scan_vector(db, data, len, 4, 0, scratch, countHandler, nullptr);

    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_EQ(1U, matchCount);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(HyperscanTestBehaviour, Vectored3) {
    hs_error_t err;

    // build a database
    hs_database_t *db = buildDB("^foo.*bar", HS_FLAG_DOTALL, 0,
                                HS_MODE_VECTORED);
    ASSERT_TRUE(db != nullptr);

    // alloc some scratch
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_TRUE(scratch != nullptr);

    const char *data[] = { "foo", "   ", "", "bar" };
    unsigned int len[] = { 3, 3, 0, 3};

    matchCount = 0;
    err = hs_scan_vector(db, data, len, 4, 0, scratch, countHandler, nullptr);

    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_EQ(1U, matchCount);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(HyperscanTestBehaviour, Vectored4) {
    hs_error_t err;

    // build a database
    hs_database_t *db = buildDB("^foo.*bar", HS_FLAG_DOTALL, 0,
                                HS_MODE_VECTORED);
    ASSERT_TRUE(db != nullptr);

    // alloc some scratch
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_TRUE(scratch != nullptr);

    const char *data[] = { "foo", "   ", "bar", "" };
    unsigned int len[] = { 3, 3, 3, 0};

    matchCount = 0;
    err = hs_scan_vector(db, data, len, 4, 0, scratch, countHandler, nullptr);

    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_EQ(1U, matchCount);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(HyperscanTestBehaviour, Vectored5) {
    hs_error_t err;

    // build a database
    hs_database_t *db = buildDB("^foo.*bar", HS_FLAG_DOTALL, 0,
                                HS_MODE_VECTORED);
    ASSERT_TRUE(db != nullptr);

    // alloc some scratch
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_TRUE(scratch != nullptr);

    const char *data[] = { "", "foo", "   ", "bar" };
    unsigned int len[] = { 0, 3, 3, 3};

    matchCount = 0;
    err = hs_scan_vector(db, data, len, 0, 0, scratch, countHandler, nullptr);

    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_EQ(0U, matchCount);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(HyperscanTestBehaviour, Vectored6) {
    hs_error_t err;

    // build a database
    hs_database_t *db = buildDB("^", HS_FLAG_DOTALL | HS_FLAG_ALLOWEMPTY, 0,
                                HS_MODE_VECTORED);
    ASSERT_TRUE(db != nullptr);

    // alloc some scratch
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_TRUE(scratch != nullptr);

    const char *data[] = { "", "foo", "   ", "bar" };
    unsigned int len[] = { 0, 3, 3, 3};

    matchCount = 0;
    err = hs_scan_vector(db, data, len, 0, 0, scratch, countHandler, nullptr);

    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_EQ(1U, matchCount);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(HyperscanTestBehaviour, Vectored7) {
    hs_error_t err;

    // build a database
    hs_database_t *db = buildDB("$", HS_FLAG_DOTALL | HS_FLAG_ALLOWEMPTY, 0,
                                HS_MODE_VECTORED);
    ASSERT_TRUE(db != nullptr);

    // alloc some scratch
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_TRUE(scratch != nullptr);

    const char *data[] = { "", "foo", "   ", "bar" };
    unsigned int len[] = { 0, 3, 3, 3};

    matchCount = 0;
    err = hs_scan_vector(db, data, len, 0, 0, scratch, countHandler, nullptr);

    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_EQ(1U, matchCount);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(HyperscanTestBehaviour, MultiStream1) {
    hs_error_t err;

    // build a database
    hs_database_t *db = buildDB("foo.*bar.*\\b", 0, 0, HS_MODE_STREAM);
    ASSERT_TRUE(db != nullptr);

    // alloc some scratch
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_TRUE(scratch != nullptr);

    hs_stream_t *stream = nullptr;
    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    hs_stream_t *stream2 = nullptr;
    err = hs_open_stream(db, 0, &stream2);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream2 != nullptr);

    matchCount = 0;
    const string data("foo        bara");
    err = hs_scan_stream(stream, data.c_str(), data.size(), 0, scratch,
                         countHandler, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_EQ(0U, matchCount); // hasn't matched until stream end

    const string data2("foo        bar ");
    err = hs_scan_stream(stream2, data2.c_str(), data2.size(), 0, scratch,
                         nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_EQ(0U, matchCount);

    err = hs_close_stream(stream, scratch, countHandler, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_EQ(1U, matchCount);

    err = hs_close_stream(stream2, scratch, countHandler, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_EQ(1U, matchCount);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(HyperscanTestBehaviour, MultiStream2) {
    hs_error_t err;

    // build a database
    hs_database_t *db = buildDB("foo.*bar.*\\b", 0, 0, HS_MODE_STREAM);
    ASSERT_TRUE(db != nullptr);

    // alloc some scratch
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_TRUE(scratch != nullptr);

    hs_stream_t *stream = nullptr;
    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    hs_stream_t *stream2 = nullptr;
    err = hs_open_stream(db, 0, &stream2);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream2 != nullptr);

    matchCount = 0;
    const string data2("foo        bar ");
    err = hs_scan_stream(stream2, data2.c_str(), data2.size(), 0, scratch,
                         nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_EQ(0U, matchCount);

    const string data("foo        bara");
    err = hs_scan_stream(stream, data.c_str(), data.size(), 0, scratch,
                         countHandler, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_EQ(0U, matchCount); // hasn't matched until stream end

    err = hs_close_stream(stream2, scratch, countHandler, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_EQ(0U, matchCount);

    err = hs_close_stream(stream, scratch, countHandler, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_EQ(1U, matchCount);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(regression, UE_1005) {
    hs_error_t err;
    vector<pattern> patterns;
    patterns.push_back(pattern("match[^Z]*", HS_FLAG_DOTALL|HS_FLAG_SINGLEMATCH,
                               1));
    patterns.push_back(pattern("[^X]+\\z", HS_FLAG_DOTALL|HS_FLAG_SINGLEMATCH,
                               2));
    patterns.push_back(pattern("[^Y]+\\z", HS_FLAG_DOTALL|HS_FLAG_SINGLEMATCH,
                               3));
    hs_database_t *db = buildDB(patterns, HS_MODE_STREAM);
    ASSERT_NE(nullptr, db);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_stream_t *stream = nullptr;

    CallBackContext c;
    const char dataA[] = "match";

    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    err = hs_scan_stream(stream, dataA, strlen(dataA), 0, scratch, record_cb,
                         (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_close_stream(stream, scratch, record_cb, (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);

    ASSERT_EQ(3U, c.matches.size());
    ASSERT_TRUE(find(c.matches.begin(), c.matches.end(), MatchRecord(5, 1))
              != c.matches.end());
    ASSERT_TRUE(find(c.matches.begin(), c.matches.end(), MatchRecord(5, 2))
              != c.matches.end());
    ASSERT_TRUE(find(c.matches.begin(), c.matches.end(), MatchRecord(5, 3))
              != c.matches.end());

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(regression, UE_2425) {
    const char regex[] = "(b|[cd](\\B|a){14}|[ba]cd.[^ece]b.[da]cbe|d[cad]cb.[da](cd|[abedc])|\\ba.edbac){18}";
    unsigned flags = HS_FLAG_DOTALL | HS_FLAG_CASELESS | HS_FLAG_SINGLEMATCH |
                     HS_FLAG_UTF8 | HS_FLAG_PREFILTER;
    vector<pattern> patterns;
    patterns.push_back(pattern(regex, flags, 1));

    // This bug was an assertion failure at Puff construction time (Puff
    // generated by prefilter reductions.)

    hs_database_t *db = buildDB(patterns, HS_MODE_STREAM);
    ASSERT_NE(nullptr, db);
    hs_free_database(db);
}

TEST(regression, UE_2485) {
    const char regex[] = "(?:(.EeEa|((a{2}BD[bc]Bd[eae]|[DCd]|c|ebCa|d)){7,21})(E{5,}A{4,}[Cc].cc{3,6}|eCec|e+CaBEd|[Bb])){10}DB(a|[AAda])..A?DE?E";
    unsigned flags = HS_FLAG_DOTALL | HS_FLAG_CASELESS | HS_FLAG_UTF8 |
                     HS_FLAG_PREFILTER;
    vector<pattern> patterns;
    patterns.push_back(pattern(regex, flags, 1));

    // This bug was an assertion failure at compile time due to
    // removeRedundancy rendering a graph unimplementable after a merge, for
    // the prefiltered pattern.

    hs_database_t *db = buildDB(patterns, HS_MODE_BLOCK);
    ASSERT_NE(nullptr, db);
    hs_free_database(db);
}

TEST(regression, UE_2452) {
    const char regex[] = "/ab.b[bca]{2,}ca((?:c|(abc(?sxmi-xm)){10,14}|c|b|[abcb])){4,23}acbcbb*ba((?:(a|.{4,}|.|[acba])){3,16}a)+";
    unsigned flags = HS_FLAG_MULTILINE | HS_FLAG_CASELESS | HS_FLAG_UTF8 |
                     HS_FLAG_UCP | HS_FLAG_PREFILTER;
    vector<pattern> patterns;
    patterns.push_back(pattern(regex, flags, 1));

    // This bug was an interaction between bounded repeat analysis,
    // prefiltering and edge redundancy.

    hs_database_t *db = buildDB(patterns, HS_MODE_STREAM);
    ASSERT_NE(nullptr, db);
    hs_free_database(db);
}

TEST(regression, UE_2595) {
    const char regex[] = "(?:(?:acAa|c[EAA]aEb|((?:CC[bdd].cE((?x-msix)BE){32}(?:\\B)){16,19}CdD.E(E|E|B)){3,6}|E(a|d|.)(?:(?xs-isxm)|b|.|C))){17,}";
    unsigned flags = HS_FLAG_MULTILINE | HS_FLAG_CASELESS |
                     HS_FLAG_SINGLEMATCH | HS_FLAG_PREFILTER;
    vector<pattern> patterns;
    patterns.push_back(pattern(regex, flags, 1));

    // This pattern has some very large depth values and isn't compilable by
    // PCRE at all.

    hs_database_t *db = buildDB(patterns, HS_MODE_STREAM);
    ASSERT_NE(nullptr, db);
    hs_free_database(db);
}

TEST(regression, UE_2762) {
    const vector<pattern> patterns = {
        pattern("\\Aa\\z", HS_FLAG_MULTILINE, 1),
        pattern("^a", HS_FLAG_MULTILINE, 2),
        pattern("a|^a", HS_FLAG_MULTILINE | HS_FLAG_SOM_LEFTMOST, 3)};

    hs_database_t *db =
        buildDB(patterns, HS_MODE_STREAM | HS_MODE_SOM_HORIZON_LARGE);
    ASSERT_NE(nullptr, db);

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_stream_t *stream = nullptr;

    CallBackContext c;
    const char dataA[] = "a";

    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(nullptr, stream);

    err = hs_scan_stream(stream, dataA, strlen(dataA), 0, scratch, record_cb,
                         (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_close_stream(stream, scratch, record_cb, (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);

    ASSERT_EQ(3U, c.matches.size());
    ASSERT_TRUE(find(c.matches.begin(), c.matches.end(), MatchRecord(1, 1)) !=
                c.matches.end());
    ASSERT_TRUE(find(c.matches.begin(), c.matches.end(), MatchRecord(1, 2)) !=
                c.matches.end());
    ASSERT_TRUE(find(c.matches.begin(), c.matches.end(), MatchRecord(1, 3)) !=
                c.matches.end());

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(regression, UE_2763) {
    const vector<pattern> patterns = {
        pattern("aaa.a+$", HS_FLAG_CASELESS, 1),
        pattern("aaa.a", HS_FLAG_CASELESS | HS_FLAG_SINGLEMATCH | HS_FLAG_UTF8,
                2)};

    hs_database_t *db = buildDB(patterns, HS_MODE_STREAM);
    ASSERT_NE(nullptr, db);

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_stream_t *stream = nullptr;

    CallBackContext c;
    const char dataA[] = "aAasaA";

    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(nullptr, stream);

    err = hs_scan_stream(stream, dataA, strlen(dataA), 0, scratch, record_cb,
                         (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_close_stream(stream, scratch, record_cb, (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);

    ASSERT_EQ(2U, c.matches.size());
    ASSERT_TRUE(find(c.matches.begin(), c.matches.end(), MatchRecord(6, 1)) !=
                c.matches.end());
    ASSERT_TRUE(find(c.matches.begin(), c.matches.end(), MatchRecord(5, 2)) !=
                c.matches.end());

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(regression, UE_2798) {
    const vector<pattern> patterns = {
        pattern("([ab]b|aab+)$", HS_FLAG_DOTALL, 1),
        pattern("ab+", HS_FLAG_SOM_LEFTMOST, 2),
        pattern("a(b.)?ba+b", 0, 3)};

    hs_database_t *db =
        buildDB(patterns, HS_MODE_STREAM | HS_MODE_SOM_HORIZON_LARGE);
    ASSERT_NE(nullptr, db);

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_stream_t *stream = nullptr;

    CallBackContext c;
    const char dataA[] = "ab_baab\n";

    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(nullptr, stream);

    err = hs_scan_stream(stream, dataA, strlen(dataA), 0, scratch, record_cb,
                         (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_close_stream(stream, scratch, record_cb, (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);

    ASSERT_EQ(4U, c.matches.size());
    ASSERT_TRUE(find(c.matches.begin(), c.matches.end(), MatchRecord(7, 1)) !=
                c.matches.end());
    ASSERT_TRUE(find(c.matches.begin(), c.matches.end(), MatchRecord(2, 2)) !=
                c.matches.end());
    ASSERT_TRUE(find(c.matches.begin(), c.matches.end(), MatchRecord(7, 2)) !=
                c.matches.end());
    ASSERT_TRUE(find(c.matches.begin(), c.matches.end(), MatchRecord(7, 3)) !=
                c.matches.end());

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(PcreSpace, NewPcre) {
    const char regex[] = "\\s";
    const string data = "\x09\x0a\x0b\x0c\x0d\x20"; /* aka "\t\n\v\f\r " */
    unsigned flags = 0;
    vector<pattern> patterns;
    patterns.push_back(pattern(regex, flags, 1));

    hs_error_t err;
    hs_database_t *db = buildDB(patterns, HS_MODE_BLOCK);
    ASSERT_NE(nullptr, db);
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_TRUE(scratch != nullptr);

    matchCount = 0;
    err = hs_scan(db, data.c_str(), data.size(), 0, scratch, countHandler, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_EQ(data.size(), matchCount); // all are spaces

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(PcreSpace, NewPcreClass) {
    const char regex[] = "[\\s]";
    const string data = "\x09\x0a\x0b\x0c\x0d\x20"; /* aka "\t\n\v\f\r " */
    unsigned flags = 0;
    vector<pattern> patterns;
    patterns.push_back(pattern(regex, flags, 1));

    hs_error_t err;
    hs_database_t *db = buildDB(patterns, HS_MODE_BLOCK);
    ASSERT_NE(nullptr, db);
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_TRUE(scratch != nullptr);

    matchCount = 0;
    err = hs_scan(db, data.c_str(), data.size(), 0, scratch, countHandler, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_EQ(data.size(), matchCount); // all are spaces

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(PcreSpace, NewPcreNeg) {
    const char regex[] = "\\S";
    const string data = "\x09\x0a\x0b\x0c\x0d\x20"; /* aka "\t\n\v\f\r " */
    unsigned flags = 0;
    vector<pattern> patterns;
    patterns.push_back(pattern(regex, flags, 1));

    hs_error_t err;
    hs_database_t *db = buildDB(patterns, HS_MODE_BLOCK);
    ASSERT_NE(nullptr, db);
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_TRUE(scratch != nullptr);

    matchCount = 0;
    err = hs_scan(db, data.c_str(), data.size(), 0, scratch, countHandler, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_EQ(0, matchCount); // no matches, all are spaces

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(PcreSpace, NewPcreClassNeg) {
    const char regex[] = "[\\S]";
    const string data = "\x09\x0a\x0b\x0c\x0d\x20"; /* aka "\t\n\v\f\r " */
    unsigned flags = 0;
    vector<pattern> patterns;
    patterns.push_back(pattern(regex, flags, 1));

    hs_error_t err;
    hs_database_t *db = buildDB(patterns, HS_MODE_BLOCK);
    ASSERT_NE(nullptr, db);
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_TRUE(scratch != nullptr);

    matchCount = 0;
    err = hs_scan(db, data.c_str(), data.size(), 0, scratch, countHandler, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_EQ(0, matchCount); // no matches, all are spaces

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(Parser, NewlineTerminatedComment) {
    // Extended mode comments and whitespace should be stripped.
    const vector<pattern> patterns = {
        pattern("(?x)foo # initial comment \n bar # second comment \n baz",
                0, 1)};
    const string data = "foobarbaz";

    hs_database_t *db = buildDB(patterns, HS_MODE_BLOCK);
    ASSERT_NE(nullptr, db);
    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_TRUE(scratch != nullptr);

    matchCount = 0;
    err = hs_scan(db, data.c_str(), data.size(), 0, scratch, countHandler, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_EQ(1, matchCount);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

} // namespace
