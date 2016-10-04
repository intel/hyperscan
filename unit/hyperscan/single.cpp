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

#include <string>
#include <tuple>

using namespace std;
using namespace testing;

namespace /* anonymous */ {

class HyperscanSingleBase {
public:
    virtual ~HyperscanSingleBase() {}

    // Simplest case: did we successfully produce a non-zero length database
    void compile() {
        SCOPED_TRACE("Compile");
        ASSERT_EQ(HS_SUCCESS, err);
        EXPECT_TRUE(db != nullptr);
        EXPECT_TRUE(compile_err == nullptr);
        size_t db_size;
        err = hs_database_size(db, &db_size);
        EXPECT_EQ(HS_SUCCESS, err);
        EXPECT_NE(0U, db_size);
    }

    // Can we query the database for various information?
    void queryDatabase() {
        SCOPED_TRACE("QueryDatabase");

        // info about the database, should begin with "Version:"
        char *info;
        err = hs_database_info(db, &info);
        ASSERT_EQ(HS_SUCCESS, err);
        ASSERT_NE(nullptr, info);
        ASSERT_STREQ("Version:", string(info).substr(0, 8).c_str());
        free(info);

        // stream size, which is non-zero if streaming
        size_t stream_size;
        err = hs_stream_size(db, &stream_size);
        if (mode & HS_MODE_STREAM) {
            ASSERT_EQ(HS_SUCCESS, err);
            ASSERT_LT(0U, stream_size);
            ASSERT_GT(100000, stream_size); /* more than 100k of stream size
                                             * and we are probably returning
                                             * rubbish. */
        } else {
            ASSERT_EQ(HS_DB_MODE_ERROR, err);
        }
    }

protected:
    unsigned int mode; // HS_MODE_* mode flags
    hs_error_t err;
    hs_compile_error_t *compile_err;
    hs_database_t *db;
};

class HyperscanTestRuntime
    : public HyperscanSingleBase,
      public TestWithParam<
          tuple<const char *, unsigned int, unsigned int, unsigned long long>> {
protected:
    virtual void SetUp() {
        // compiles a database for this test instantiation
        const char *pattern;
        unsigned int flags;
        unsigned long long feature_mask;
        tie(pattern, flags, mode, feature_mask) = GetParam();

        hs_platform_info_t plat;
        err = hs_populate_platform(&plat);
        ASSERT_EQ(HS_SUCCESS, err);
        plat.cpu_features &= feature_mask;

        err = hs_compile(pattern, flags, mode, &plat, &db, &compile_err);
    }

    virtual void TearDown() {
        hs_free_database(db);
        hs_free_compile_error(compile_err);
    }

    // does a simple scan with a dummy handler (ignores matches)
    virtual void simpleScan() {
        const size_t datalen = 2048;
        char data[datalen];
        memset(data, 'X', datalen);

        hs_scratch_t *scratch = nullptr;
        err = hs_alloc_scratch(db, &scratch);
        ASSERT_EQ(HS_SUCCESS, err);
        EXPECT_TRUE(scratch != nullptr);

        if (mode & HS_MODE_STREAM) {
            // streaming mode scan
            hs_stream_t *stream = nullptr;

            err = hs_open_stream(db, 0, &stream);
            ASSERT_EQ(HS_SUCCESS, err);
            ASSERT_TRUE(stream != nullptr);

            // just for fun, scan a zero-byte block.
            err = hs_scan_stream(stream, data, 0, 0, scratch, dummy_cb,
                                 nullptr);
            ASSERT_EQ(HS_SUCCESS, err);

            err = hs_scan_stream(stream, data, datalen, 0, scratch,
                                 dummy_cb, nullptr);
            ASSERT_EQ(HS_SUCCESS, err);

            err = hs_close_stream(stream, scratch, dummy_cb, nullptr);
            ASSERT_EQ(HS_SUCCESS, err);
        } else if (mode & HS_MODE_VECTORED) {
            const char * const vec_bufs[] = { data };
            const unsigned int vec_lens[] = { datalen };
            err = hs_scan_vector(db, vec_bufs, vec_lens, 1, 0, scratch, dummy_cb, nullptr);
            ASSERT_EQ(HS_SUCCESS, err);
        } else {
            // block mode scan
            ASSERT_TRUE(mode & HS_MODE_BLOCK);
            err = hs_scan(db, data, datalen, 0, scratch, dummy_cb, nullptr);
            ASSERT_EQ(HS_SUCCESS, err);
        }

        // teardown
        err = hs_free_scratch(scratch);
        ASSERT_EQ(HS_SUCCESS, err);
    }

    virtual void zeroLengthScan() {
        const size_t datalen = 20;
        char data[datalen];
        memset(data, 'X', datalen);

        hs_scratch_t *scratch = nullptr;
        err = hs_alloc_scratch(db, &scratch);
        ASSERT_EQ(HS_SUCCESS, err);
        EXPECT_TRUE(scratch != nullptr);

        if (mode & HS_MODE_STREAM) {
            // streaming mode scan
            hs_stream_t *stream = nullptr;
            err = hs_open_stream(db, 0, &stream);
            ASSERT_EQ(HS_SUCCESS, err);
            ASSERT_TRUE(stream != nullptr);

            err = hs_scan_stream(stream, data, 0, 0, scratch, dummy_cb, nullptr);
            ASSERT_EQ(HS_SUCCESS, err);

            err = hs_close_stream(stream, scratch, dummy_cb, nullptr);
            ASSERT_EQ(HS_SUCCESS, err);
        } else if (mode & HS_MODE_VECTORED) {
            const char * const vec_bufs[] = { data };
            const unsigned int vec_lens[] = { 0 };
            err = hs_scan_vector(db, vec_bufs, vec_lens, 1, 0, scratch, dummy_cb, nullptr);
            ASSERT_EQ(HS_SUCCESS, err);
        } else {
            // block mode scan
            ASSERT_TRUE(mode & HS_MODE_BLOCK);
            err = hs_scan(db, data, 0, 0, scratch, dummy_cb, nullptr);
            ASSERT_EQ(HS_SUCCESS, err);
        }

        // teardown
        err = hs_free_scratch(scratch);
        ASSERT_EQ(HS_SUCCESS, err);
    }

    // Can we allocate and clone scratch
    void allocateScratch() {
        SCOPED_TRACE("AllocateScratch");
        hs_scratch_t *scratch = nullptr;
        err = hs_alloc_scratch(db, &scratch);
        ASSERT_EQ(HS_SUCCESS, err);
        EXPECT_TRUE(scratch != nullptr);

        // Try cloning the scratch
        hs_scratch_t *cloned = nullptr;
        err = hs_clone_scratch(scratch, &cloned);
        ASSERT_EQ(HS_SUCCESS, err);
        EXPECT_TRUE(cloned != nullptr);

        err = hs_free_scratch(scratch);
        EXPECT_EQ(HS_SUCCESS, err);
        err = hs_free_scratch(cloned);
        EXPECT_EQ(HS_SUCCESS, err);
    }

    // Can we scan with the database (ignoring the matches, and using
    // streaming/block mode appropriately)
    void scanIgnoreMatches() {
        SCOPED_TRACE("ScanIgnoreMatches");
        simpleScan();
    }

    // Can we serialize and deserialize the database
    void serialiseAndDeserializeCombo() {
        SCOPED_TRACE("SerialiseAndDeserializeCombo");
        char *bytes = nullptr;
        size_t len = 0;

        // serialise
        err = hs_serialize_database(db, &bytes, &len);
        ASSERT_EQ(HS_SUCCESS, err);
        EXPECT_TRUE(bytes != nullptr);
        EXPECT_NE(0U, len);

        // destroy original database
        size_t origSize;
        err = hs_database_size(db, &origSize);
        ASSERT_EQ(HS_SUCCESS, err);
        memset(db, 0xff, origSize);
        free(db); /* hs_free_database not used as it is no longer a valid db */

        // relocate to 16 different alignments, ensuring that we can
        // deserialize from any string
        char *buffer = new char[len + 16];
        for (size_t i = 0; i < 16; i++) {
            size_t newSize;
            memset(buffer, 0, len + 16);

            // relocate
            char *copy = buffer + i;
            memcpy(copy, bytes, len);

            // deserialise
            hs_database_t *newdb;
            err = hs_deserialize_database(copy, len, &newdb);
            ASSERT_EQ(HS_SUCCESS, err);
            EXPECT_TRUE(newdb != nullptr);
            err = hs_database_size(newdb, &newSize);
            ASSERT_EQ(HS_SUCCESS, err);
            EXPECT_EQ(origSize, newSize);

            // deserialiseAt
            // alloc a new place for this DB
            size_t buflen;
            err = hs_serialized_database_size(copy, len, &buflen);
            ASSERT_EQ(HS_SUCCESS, err);
            ASSERT_NE(0U, buflen);
            char *buf = new char[buflen];
            memset(buf, 0, buflen);

            // deserialise to our buffer
            hs_database_t *newdbAt = (hs_database_t *)buf;
            err = hs_deserialize_database_at(copy, len, newdbAt);
            ASSERT_EQ(HS_SUCCESS, err);
            err = hs_database_size(newdb, &newSize);
            ASSERT_EQ(HS_SUCCESS, err);
            EXPECT_EQ(origSize, newSize);

            {
                SCOPED_TRACE("DeserializeAt");
                // Replace test harness DB and run a simple scan as a test
                db = newdbAt;
                simpleScan();
                delete[] buf;
                db = nullptr;
            }

            {
                SCOPED_TRACE("Deserialize");
                // Replace test harness DB and run a simple scan as a test
                db = newdb;
                simpleScan();
                hs_free_database(newdb);
                db = nullptr;
            }

        }

        delete[] buffer;
        free(bytes);
    }
};

TEST_P(HyperscanTestRuntime, Combo) {
    compile();
    queryDatabase();
    allocateScratch();
    zeroLengthScan();
    scanIgnoreMatches();
    serialiseAndDeserializeCombo();
}

static const char *validPatterns[] = {
        "foobar",
        "abd.*def",
        "abc[123]def",
        "[pqr]",
        ".",
        "\\s",
        "hatstand.*(teakettle|badgerbrush)",
        "abc{1,3}",
        "abc",
        "^.{1,10}flibble",
        "(foobar)+",
        "(foo){2,5}",
        "((foo){2}){3}",
        "^.*test[a-f]{3}pattern.*$",
        "(([^u]|.){16}|x){1,2}", // from bug UE-500

        // UE-1553: these should compile without eating all your RAM.
        "fooa?a?a?a?a?a?a?a?a?a?a?a?a?a?a?a?a?a?a?a?a?a?a?a?",
        "fooa?a?a?a?a?a?a?a?a?a?a?a?a?a?a?a?a?a?a?a?a?a?a?a?a?a?a?a?a?a?a?a?a?a?",

        // UE-1816 (makes libpcre hit its match limit)
        "((aaa|[aaa]aa|aa[^aaaa]a|a.aaa.|a)){27}",

        // UE-2370 (makes libpcre hit its match limit)
        "^.{0,40}((e{0,9}|b){16}a.A|[da]..ecbcbcc[^e]de{10,20}[Bb]{3}[dEe]*){2}",
};

static const unsigned validFlags[] = {
        0,
        HS_FLAG_CASELESS,
        HS_FLAG_DOTALL,
        HS_FLAG_MULTILINE,
        HS_FLAG_CASELESS | HS_FLAG_DOTALL | HS_FLAG_MULTILINE,
        HS_FLAG_SINGLEMATCH,
};

static const unsigned validModes[] = {
    HS_MODE_BLOCK,
    HS_MODE_STREAM,
    HS_MODE_VECTORED,
};

// Mode bits for switching off various architecture features
static const unsigned long long featureMask[] = {
    ~0ULL, /* native */
    ~(HS_CPU_FEATURES_AVX2 | HS_CPU_FEATURES_AVX512), /* no avx2 */
    ~HS_CPU_FEATURES_AVX512, /* no avx512 */
};

INSTANTIATE_TEST_CASE_P(Single,
                        HyperscanTestRuntime,
                        Combine(ValuesIn(validPatterns),
                                ValuesIn(validFlags),
                                ValuesIn(validModes),
                                ValuesIn(featureMask)));

struct TestPlatform {
    TestPlatform() : features(0ULL) {}
    TestPlatform(unsigned long long f) : features(f) {}
    unsigned long long features;
};

class HyperscanTestCrossCompile
    : public HyperscanSingleBase,
      public TestWithParam<
          tuple<const char *, unsigned, unsigned, TestPlatform>> {
protected:
    virtual void SetUp() {
        // compiles a database for this test instantiation
        const char *pattern;
        unsigned flags;
        TestPlatform tp;
        tie(pattern, flags, mode, tp) = GetParam();

        hs_platform_info_t plat;
        plat.tune = HS_TUNE_FAMILY_GENERIC;
        plat.cpu_features = tp.features;

        err = hs_compile(pattern, flags, mode, &plat, &db, &compile_err);
    }

    virtual void TearDown() {
        hs_free_database(db);
        hs_free_compile_error(compile_err);
    }

    // Attempt to allocate a scratch region. This should succeed for the local
    // platform and return HS_DB_PLATFORM_ERROR for all the others.
    void attemptScratchAlloc() {
        SCOPED_TRACE("AttemptScratchAlloc");

        hs_scratch_t *scratch = nullptr;
        err = hs_alloc_scratch(db, &scratch);

        // XXX: it'd be nice to be a bit less cavalier about this and actually
        // find out what the local platform is, then check explicitly that only
        // that platform produces HS_SUCCESS.
        if (err == HS_SUCCESS) {
            // host platform.
            err = hs_free_scratch(scratch);
            ASSERT_EQ(HS_SUCCESS, err);
        } else {
            ASSERT_EQ(HS_DB_PLATFORM_ERROR, err);
            ASSERT_TRUE(!scratch);
        }
    }
};

TEST_P(HyperscanTestCrossCompile, Build) {
    compile();
    queryDatabase();
    attemptScratchAlloc();
}

static const TestPlatform validPlatforms[] = {
    TestPlatform(0),
    TestPlatform(HS_CPU_FEATURES_AVX2),
};

INSTANTIATE_TEST_CASE_P(Single,
                        HyperscanTestCrossCompile,
                        Combine(ValuesIn(validPatterns),
                                ValuesIn(validFlags),
                                ValuesIn(validModes),
                                ValuesIn(validPlatforms)));

// Also instantiate some normal and cross-compile tests for SOM patterns.
static const char *validSomPatterns[] = {
    "foobar",
    "abd.*def",
    "abc[123]def",
    "[pqr]",
    ".",
    "\\s",
    "hatstand.*(teakettle|badgerbrush)",
    "abc{1,3}",
    "abc",
    "^.{1,10}flibble",
    "(foobar)+",
    "(foo){2,5}",
    "((foo){2}){3}",
    "^.*test[a-f]{3}pattern.*$",
};
static const unsigned validSomFlags[] = {
    HS_FLAG_SOM_LEFTMOST,
};
static const unsigned validSomModes[] = {
    HS_MODE_BLOCK,
    HS_MODE_STREAM | HS_MODE_SOM_HORIZON_LARGE,
    HS_MODE_VECTORED,
};
INSTANTIATE_TEST_CASE_P(SingleSom, HyperscanTestRuntime,
                        Combine(ValuesIn(validSomPatterns),
                                ValuesIn(validSomFlags),
                                ValuesIn(validSomModes),
                                ValuesIn(featureMask)));
INSTANTIATE_TEST_CASE_P(SingleSom, HyperscanTestCrossCompile,
                        Combine(ValuesIn(validSomPatterns),
                                ValuesIn(validSomFlags),
                                ValuesIn(validSomModes),
                                ValuesIn(validPlatforms)));

struct TerminateMatchData {
    TerminateMatchData(const char *pattern_, unsigned int flags_,
                       const char *corpus_) : pattern(pattern_), flags(flags_),
                                              corpus(corpus_) {}
    const char *pattern;
    unsigned int flags;
    const char *corpus;
};

class HyperscanTestMatchTerminate : public TestWithParam<TerminateMatchData> {
    // empty
};

// Simple non-terminating callback: increments the int pointed to by the context
// and tells the matcher to keep going.
int countHandler(unsigned, unsigned long long, unsigned long long,
                 unsigned, void *ctx) {
    int *count = (int *)ctx;
    (*count)++;
    return 0; // keep going
}

// Simple terminating callback: increments the int pointed to by the context
// and tells the matcher to stop.
int terminateHandler(unsigned, unsigned long long, unsigned long long,
                     unsigned, void *ctx) {
    int *count = (int *)ctx;
    (*count)++;
    return 1; // stop matching
}

TEST_P(HyperscanTestMatchTerminate, MoreThanOne) {
    const TerminateMatchData &data = GetParam();

    hs_error_t err;
    hs_scratch_t *scratch = nullptr;
    hs_database_t *db = buildDBAndScratch(data.pattern, data.flags, 0,
                                          HS_MODE_BLOCK, &scratch);

    int count = 0;
    err = hs_scan(db, data.corpus, strlen(data.corpus), 0, scratch,
                  countHandler, &count);
    ASSERT_EQ(HS_SUCCESS, err) << "hs_scan didn't return HS_SCAN_TERMINATED";
    ASSERT_LT(1, count) << "Number of matches returned was not greater than 1.";

    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST_P(HyperscanTestMatchTerminate, Block) {
    const TerminateMatchData &data = GetParam();

    hs_error_t err;
    hs_scratch_t *scratch = nullptr;
    hs_database_t *db = buildDBAndScratch(data.pattern, data.flags, 0,
                                          HS_MODE_BLOCK, &scratch);

    int count = 0;
    err = hs_scan(db, data.corpus, strlen(data.corpus), 0, scratch,
                  terminateHandler, &count);
    ASSERT_EQ(HS_SCAN_TERMINATED, err)
        << "hs_scan didn't return HS_SCAN_TERMINATED";
    ASSERT_EQ(1, count) << "Number of matches returned was not 1.";

    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST_P(HyperscanTestMatchTerminate, StreamWhole) {
    const TerminateMatchData& data = GetParam();

    hs_error_t err;
    hs_scratch_t *scratch = nullptr;
    hs_database_t *db = buildDBAndScratch(data.pattern, data.flags, 0,
                                          HS_MODE_STREAM, &scratch);

    int count = 0;
    hs_stream_t *stream = nullptr;
    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err) << "Stream open failed";
    ASSERT_TRUE(stream != nullptr);

    err = hs_scan_stream(stream, data.corpus, strlen(data.corpus), 0, scratch,
                         terminateHandler, &count);
    ASSERT_TRUE(err == HS_SUCCESS || err == HS_SCAN_TERMINATED);

    err = hs_close_stream(stream, scratch, terminateHandler, &count);
    ASSERT_EQ(1, count) << "Number of matches returned was not 1.";

    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST_P(HyperscanTestMatchTerminate, StreamByteByByte) {
    const TerminateMatchData& data = GetParam();

    hs_error_t err;
    hs_scratch_t *scratch = nullptr;
    hs_database_t *db = buildDBAndScratch(data.pattern, data.flags, 0,
                                          HS_MODE_STREAM, &scratch);

    int count = 0;
    hs_stream_t *stream = nullptr;
    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err) << "Stream open failed";
    ASSERT_TRUE(stream != nullptr);

    size_t len = strlen(data.corpus);
    for (size_t i = 0; i < len; i++) {
        err = hs_scan_stream(stream, data.corpus + i, 1, 0, scratch,
                             terminateHandler, &count);
        ASSERT_TRUE(err == HS_SUCCESS || err == HS_SCAN_TERMINATED);
    }

    err = hs_close_stream(stream, scratch, terminateHandler, &count);
    ASSERT_EQ(1, count) << "Number of matches returned was not 1.";

    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

// Each of these cases must match multiple times
const TerminateMatchData terminateCases[] = {
    TerminateMatchData("foobar", 0, "foobarfoobarfoobar"),
    TerminateMatchData("a", 0, "a a a a a a a a a a a"),
    TerminateMatchData(".", 0, "zzzzzzzzzzzzaaaaaaaaaaaaa"),
    TerminateMatchData("...", 0, "zzzzzzzzzzzzaaaaaaaaaaaaa"),
    TerminateMatchData("[a-z]{3,7}", 0, "zzzzzzzzzzzzaaaaaaaaaaaaa"),
    TerminateMatchData("a", HS_FLAG_CASELESS, "   xAaAa"),
    TerminateMatchData("xyzzy", HS_FLAG_CASELESS, "abcdef XYZZy xyzzy XyZzY"),
    TerminateMatchData("abc.*def", 0, "abc   abc   abc   def def"),
    TerminateMatchData("abc.*def", HS_FLAG_DOTALL, "abc   abc   abc   def def"),
    TerminateMatchData("(01234|abcde).*(foo|bar)", 0, "abcde  xxxx   bar foo abcde foo"),
    TerminateMatchData("(01234|abcde).*(foo|bar)", HS_FLAG_DOTALL, "abcde  xxxx   bar foo abcde foo"),
    TerminateMatchData("[0-9a-f]{4,10}.*(foobar|bazbaz)", 0, "0123456789abcdef  bazbaz foobar"),
    TerminateMatchData("[0-9a-f]{4,10}.*(foobar|bazbaz)", HS_FLAG_DOTALL, "0123456789abcdef  bazbaz foobar"),
    TerminateMatchData("^foobar[^z]{20,}", 0, "foobarxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"),
    TerminateMatchData("hatstand|teakettle|badgerbrush|mnemosyne", 0, "hatstand teakettle badgerbrush mnemosyne"),
    TerminateMatchData("a|b|c|d", 0, "a b c d a b c d a b c d"),
    TerminateMatchData("bat|cat|mat|rat|fat|sat|pat|hat|vat", HS_FLAG_CASELESS, "VAt hat pat sat fat rat mat caT BAT"),
};

INSTANTIATE_TEST_CASE_P(Single, HyperscanTestMatchTerminate, ValuesIn(terminateCases));

} // namespace

