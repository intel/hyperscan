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

#include "gtest/gtest.h"
#include "hs.h"
#include "test_util.h"

static char garbage[] = "TEST(HyperscanArgChecks, DatabaseSizeNoDatabase) {" \
                        "    size_t sz = hs_database_size(0);" \
                        "    ASSERT_EQ(0, sz);";

static unsigned lastMatchId = 0;
static unsigned long long lastMatchFrom = 0;
static unsigned long long lastMatchTo = 0;
static unsigned lastMatchFlags = 0;
static void *lastMatchCtx = nullptr;

// Single match callback: record all the details from a single match
static
int singleHandler(unsigned id, unsigned long long from,
                  unsigned long long to, unsigned flags, void *ctx) {
    lastMatchId = id;
    lastMatchFrom = from;
    lastMatchTo = to;
    lastMatchFlags = flags;
    lastMatchCtx = ctx;
    return 0;
}

namespace /* anonymous */ {

// Break the magic number of the given database.
void breakDatabaseMagic(hs_database *db) {
    // database magic should be 0xdbdb at the start
    ASSERT_TRUE(memcmp("\xdb\xdb", db, 2) == 0);
    *(char *)db = 0xdc;
}

// Break the version number of the given database.
void breakDatabaseVersion(hs_database *db) {
    // database version is the second u32
    *((char *)db + 4) += 1;
}

// Break the platform data of the given database.
void breakDatabasePlatform(hs_database *db) {
    // database platform is an aligned u64a 16 bytes in
    memset((char *)db + 16, 0xff, 8);
}

// Break the alignment of the bytecode for the given database.
void breakDatabaseBytecode(hs_database *db) {
    // bytecode ptr is a u32, 36 bytes in
    unsigned int *bytecode = (unsigned int *)((char *)db + 36);
    ASSERT_NE(0U, *bytecode);
    ASSERT_EQ(0U, (size_t)((char *)db + *bytecode) % 16U);
    *bytecode += 3;
}

// Check that hs_valid_platform says we can run here
TEST(HyperscanArgChecks, ValidPlatform) {
    hs_error_t error = hs_valid_platform();
    ASSERT_EQ(HS_SUCCESS, error) << "hs_valid_platform should return zero";
}

// Check that hs_version gives us a reasonable string back
TEST(HyperscanArgChecks, Version) {
    const char *version = hs_version();
    ASSERT_TRUE(version != nullptr);
    ASSERT_TRUE(version[0] >= '0' && version[0] <= '9') << "First byte should be a digit.";
    ASSERT_EQ('.', version[1]) << "Second byte should be a dot.";
}

// hs_compile: Compile a NULL pattern (block mode)
TEST(HyperscanArgChecks, SingleCompileBlockNoPattern) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile(nullptr, 0, HS_MODE_NOSTREAM, nullptr, &db,
                                &compile_err);
    EXPECT_EQ(HS_COMPILER_ERROR, err);
    EXPECT_TRUE(db == nullptr);
    EXPECT_TRUE(compile_err != nullptr);
    hs_free_compile_error(compile_err);
}

// hs_compile: Compile a NULL pattern (streaming mode)
TEST(HyperscanArgChecks, SingleCompileStreamingNoPattern) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile(nullptr, 0, HS_MODE_STREAM, nullptr, &db,
                                &compile_err);
    EXPECT_EQ(HS_COMPILER_ERROR, err);
    EXPECT_TRUE(db == nullptr);
    EXPECT_TRUE(compile_err != nullptr);
    hs_free_compile_error(compile_err);
}

// hs_compile: Compile a pattern to a NULL database ptr (block mode)
TEST(HyperscanArgChecks, SingleCompileBlockNoDatabase) {
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_NOSTREAM, nullptr, nullptr,
                                &compile_err);
    EXPECT_EQ(HS_COMPILER_ERROR, err);
    EXPECT_TRUE(compile_err != nullptr);
    hs_free_compile_error(compile_err);
}

// hs_compile: Compile a pattern to a NULL database ptr (streaming mode)
TEST(HyperscanArgChecks, SingleCompileStreamingNoDatabase) {
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_STREAM, nullptr, nullptr,
                                &compile_err);
    EXPECT_EQ(HS_COMPILER_ERROR, err);
    EXPECT_TRUE(compile_err != nullptr);
    hs_free_compile_error(compile_err);
}

// hs_compile: Compile a pattern with no mode set
TEST(HyperscanArgChecks, SingleCompileNoMode) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, 0, nullptr, &db, &compile_err);
    EXPECT_EQ(HS_COMPILER_ERROR, err);
    EXPECT_TRUE(compile_err != nullptr);
    hs_free_compile_error(compile_err);
}

// hs_compile: Compile a pattern with several modes set
TEST(HyperscanArgChecks, SingleCompileSeveralModes1) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_STREAM | HS_MODE_NOSTREAM,
                                nullptr, &db, &compile_err);
    EXPECT_EQ(HS_COMPILER_ERROR, err);
    EXPECT_TRUE(compile_err != nullptr);
    hs_free_compile_error(compile_err);
}

// hs_compile: Compile a pattern with bogus flags
TEST(HyperscanArgChecks, SingleCompileBogusFlags) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0xdeadbeef, HS_MODE_NOSTREAM,
                                nullptr, &db, &compile_err);
    EXPECT_EQ(HS_COMPILER_ERROR, err);
    EXPECT_TRUE(compile_err != nullptr);
    EXPECT_STREQ("only HS_FLAG_QUIET and HS_FLAG_SINGLEMATCH "
                 "are supported in combination "
                 "with HS_FLAG_COMBINATION.", compile_err->message);

    hs_free_compile_error(compile_err);
}

// hs_compile: Compile a pattern with bogus mode flags set.
TEST(HyperscanArgChecks, SingleCompileBogusMode1) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;

    unsigned mode = HS_MODE_STREAM | (1U << 30);

    hs_error_t err = hs_compile("foobar", 0, mode, nullptr, &db, &compile_err);
    EXPECT_EQ(HS_COMPILER_ERROR, err);
    ASSERT_TRUE(compile_err != nullptr);
    ASSERT_STREQ("Invalid parameter: unrecognised mode flags.",
                 compile_err->message);
    hs_free_compile_error(compile_err);
}

TEST(HyperscanArgChecks, SingleCompileBadTune) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_platform_info_t plat;
    plat.cpu_features = 0;
    plat.tune = 42;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_NOSTREAM, &plat, &db,
                                &compile_err);
    EXPECT_EQ(HS_COMPILER_ERROR, err);
    EXPECT_TRUE(compile_err != nullptr);
    EXPECT_STREQ("Invalid tuning value specified in the platform information.",
                 compile_err->message);

    hs_free_compile_error(compile_err);
}

TEST(HyperscanArgChecks, SingleCompileBadFeatures) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_platform_info_t plat;
    plat.cpu_features = 42;
    plat.tune = 0;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_NOSTREAM, &plat, &db,
                                &compile_err);
    EXPECT_EQ(HS_COMPILER_ERROR, err);
    EXPECT_TRUE(compile_err != nullptr);
    EXPECT_STREQ("Invalid cpu features specified in the platform information.",
                 compile_err->message);

    hs_free_compile_error(compile_err);
}

// hs_compile: Check SOM flag validation.
TEST(HyperscanArgChecks, SingleCompileSOMFlag) {

    // try using the flags without a SOM mode.
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", HS_FLAG_SOM_LEFTMOST, HS_MODE_STREAM,
                                nullptr, &db, &compile_err);
    ASSERT_EQ(HS_COMPILER_ERROR, err) << "should have failed with flag "
                                      << HS_FLAG_SOM_LEFTMOST;
    ASSERT_TRUE(compile_err != nullptr);
    hs_free_compile_error(compile_err);
}

// hs_compile: Check SOM mode validation.
TEST(HyperscanArgChecks, SingleCompileSOMModes) {
    static const unsigned som_modes[] = {
            HS_MODE_SOM_HORIZON_LARGE,
            HS_MODE_SOM_HORIZON_MEDIUM,
            HS_MODE_SOM_HORIZON_SMALL
    };
    const size_t num_modes = sizeof(som_modes)/sizeof(som_modes[0]);

    // compilation of a trivial case with a single mode set should be fine.
    for (size_t i = 0; i < num_modes; i++) {
        hs_database_t *db = nullptr;
        hs_compile_error_t *compile_err = nullptr;
        hs_error_t err = hs_compile("foobar", HS_FLAG_SOM_LEFTMOST,
                                    HS_MODE_STREAM | som_modes[i], nullptr, &db,
                                    &compile_err);
        ASSERT_EQ(HS_SUCCESS, err);
        hs_free_database(db);
    }

    // you can only use one of the SOM modes at a time.
    for (size_t i = 0; i < num_modes; i++) {
        for (size_t j = i + 1; j < num_modes; j++) {
            hs_database_t *db = nullptr;
            hs_compile_error_t *compile_err = nullptr;
            unsigned int mode = som_modes[i] | som_modes[j];
            hs_error_t err = hs_compile("foobar", HS_FLAG_SOM_LEFTMOST,
                                        HS_MODE_STREAM | mode, nullptr, &db,
                                        &compile_err);
            ASSERT_EQ(HS_COMPILER_ERROR, err)
                << "should have failed with mode " << mode;
            ASSERT_TRUE(compile_err != nullptr);
            hs_free_compile_error(compile_err);
        }
    }
}

// hs_compile_multi: Compile a NULL pattern set (block mode)
TEST(HyperscanArgChecks, MultiCompileBlockNoPattern) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile_multi(nullptr, nullptr, nullptr, 1, HS_MODE_NOSTREAM,
                                      nullptr, &db, &compile_err);
    EXPECT_EQ(HS_COMPILER_ERROR, err);
    EXPECT_TRUE(db == nullptr);
    EXPECT_TRUE(compile_err != nullptr);
    hs_free_compile_error(compile_err);
}

// hs_compile_multi: Compile a NULL pattern set (streaming mode)
TEST(HyperscanArgChecks, MultiCompileStreamingNoPattern) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile_multi(nullptr, nullptr, nullptr, 1, HS_MODE_STREAM,
                                      nullptr, &db, &compile_err);
    EXPECT_EQ(HS_COMPILER_ERROR, err);
    EXPECT_TRUE(db == nullptr);
    EXPECT_TRUE(compile_err != nullptr);
    hs_free_compile_error(compile_err);
}

// hs_compile_multi: Compile a set of zero patterns
TEST(HyperscanArgChecks, MultiCompileZeroPatterns) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    const char *expr[] = {"foobar"};
    hs_error_t err = hs_compile_multi(expr, nullptr, nullptr, 0, HS_MODE_NOSTREAM,
                                      nullptr, &db, &compile_err);
    EXPECT_EQ(HS_COMPILER_ERROR, err);
    EXPECT_TRUE(db == nullptr);
    EXPECT_TRUE(compile_err != nullptr);
    hs_free_compile_error(compile_err);
}

// hs_compile_multi: Compile a pattern to a NULL database ptr (block mode)
TEST(HyperscanArgChecks, MultiCompileBlockNoDatabase) {
    hs_compile_error_t *compile_err = nullptr;
    const char *expr[] = {"foobar"};
    hs_error_t err = hs_compile_multi(expr, nullptr, nullptr, 1, HS_MODE_NOSTREAM,
                                      nullptr, nullptr, &compile_err);
    EXPECT_EQ(HS_COMPILER_ERROR, err);
    EXPECT_TRUE(compile_err != nullptr);
    hs_free_compile_error(compile_err);
}

// hs_compile_multi: Compile a pattern to a NULL database ptr (streaming mode)
TEST(HyperscanArgChecks, MultiCompileStreamingNoDatabase) {
    hs_compile_error_t *compile_err = nullptr;
    const char *expr[] = {"foobar"};
    hs_error_t err = hs_compile_multi(expr, nullptr, nullptr, 1, HS_MODE_STREAM,
                                      nullptr, nullptr, &compile_err);
    EXPECT_EQ(HS_COMPILER_ERROR, err);
    EXPECT_TRUE(compile_err != nullptr);
    hs_free_compile_error(compile_err);
}

// hs_open_stream: Open a stream with a NULL database ptr
TEST(HyperscanArgChecks, OpenStreamNoDatabase) {
    hs_stream_t *stream = nullptr;
    hs_error_t err = hs_open_stream(nullptr, 0, &stream);
    EXPECT_NE(HS_SUCCESS, err);
    EXPECT_TRUE(stream == nullptr);
}

// hs_open_stream: Open a stream with a NULL stream ptr
TEST(HyperscanArgChecks, OpenStreamNoStreamId) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_STREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_open_stream(db, 0, nullptr);
    ASSERT_EQ(HS_INVALID, err);

    // teardown
    hs_free_database(db);
}

// hs_open_stream: Open a stream with a non-streaming database
TEST(HyperscanArgChecks, OpenStreamWithBlockDatabase) {
    hs_stream_t *stream = nullptr;
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_NOSTREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_open_stream(db, 0, &stream);
    EXPECT_NE(HS_SUCCESS, err);
    EXPECT_TRUE(stream == nullptr);

    // teardown
    hs_free_database(db);
}

// hs_open_stream: Open a stream with a broken database bytecode
TEST(HyperscanArgChecks, OpenStreamWithBrokenDatabaseBytecode) {
    hs_stream_t *stream = nullptr;
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_STREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);

    breakDatabaseBytecode(db);

    err = hs_open_stream(db, 0, &stream);
    EXPECT_NE(HS_SUCCESS, err);
    EXPECT_TRUE(stream == nullptr);

    // teardown
    hs_free_database(db);
}

// hs_scan_stream: Call with no stream ID
TEST(HyperscanArgChecks, ScanStreamNoStreamID) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_STREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    err = hs_scan_stream(nullptr, "data", 4, 0, scratch, dummy_cb, nullptr);
    EXPECT_NE(HS_SUCCESS, err);
    EXPECT_NE(HS_SCAN_TERMINATED, err);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

// hs_scan_stream: Call with no data
TEST(HyperscanArgChecks, ScanStreamNoData) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_STREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);
    hs_stream_t *stream = nullptr;
    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    err = hs_scan_stream(stream, nullptr, 4, 0, scratch, dummy_cb, nullptr);
    EXPECT_NE(HS_SUCCESS, err);
    EXPECT_NE(HS_SCAN_TERMINATED, err);

    // teardown
    err = hs_close_stream(stream, scratch, dummy_cb, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

// hs_scan_stream: Call with no scratch
TEST(HyperscanArgChecks, ScanStreamNoScratch) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_STREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);
    hs_stream_t *stream = nullptr;
    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    err = hs_scan_stream(stream, nullptr, 4, 0, scratch, dummy_cb, nullptr);
    EXPECT_NE(HS_SUCCESS, err);
    EXPECT_NE(HS_SCAN_TERMINATED, err);

    // teardown
    err = hs_close_stream(stream, scratch, dummy_cb, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

// hs_close_stream: Call with no stream
TEST(HyperscanArgChecks, CloseStreamNoStream) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_STREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    err = hs_close_stream(nullptr, scratch, dummy_cb, nullptr);
    ASSERT_NE(HS_SUCCESS, err);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

// hs_close_stream: Call with no scratch
TEST(HyperscanArgChecks, CloseStreamNoScratch) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_STREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);
    hs_stream_t *stream = nullptr;
    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    err = hs_close_stream(stream, nullptr, dummy_cb, nullptr);
    EXPECT_NE(HS_SUCCESS, err);

    // We happen to know we can free the stream here, and we do it so that
    // this doesn't appear as a memory leak in valgrind
    free(stream);

    // teardown
    hs_free_database(db);
}

// hs_close_stream: Call with no scratch and no callback (allowed)
TEST(HyperscanArgChecks, CloseStreamNoScratchNoCallback) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_STREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);
    hs_stream_t *stream = nullptr;
    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    err = hs_close_stream(stream, nullptr, nullptr, nullptr);
    EXPECT_EQ(HS_SUCCESS, err);

    // teardown
    hs_free_database(db);
}

// hs_close_stream_nomatch: Call with no stream
TEST(HyperscanArgChecks, CloseStreamNoMatchNoStream) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_STREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_close_stream(nullptr, scratch, nullptr, nullptr);
    ASSERT_NE(HS_SUCCESS, err);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

// hs_set_stream_context: change context
TEST(HyperscanArgChecks, ChangeStreamContext) {
    const char *str = "foobar";
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile(str, 0, HS_MODE_STREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    // we'll start with the context pointer set to the scratch space ...
    hs_stream_t *stream = nullptr;
    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);
    err = hs_scan_stream(stream, str, strlen(str), 0, scratch, singleHandler,
                         (void *)scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    ASSERT_EQ(lastMatchTo, strlen(str));
    ASSERT_EQ(lastMatchCtx, scratch);

    // ... and then change it to the stream ptr itself.
    err = hs_scan_stream(stream, str, strlen(str), 0, scratch, singleHandler,
                         (void *)stream);
    ASSERT_EQ(HS_SUCCESS, err);

    ASSERT_EQ(lastMatchTo, 2*strlen(str));
    ASSERT_EQ(lastMatchCtx, stream);

    err = hs_close_stream(stream, scratch, nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);

    // teardown
    hs_free_database(db);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
}

// hs_reset_stream: Call with no stream id
TEST(HyperscanArgChecks, ResetStreamNoId) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_STREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_reset_stream(nullptr, 0, scratch, dummy_cb, nullptr);
    ASSERT_EQ(HS_INVALID, err);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

// hs_reset_stream: Call with no scratch
TEST(HyperscanArgChecks, ResetStreamNoScratch) {
    hs_stream_t *stream = nullptr;
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_STREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    err = hs_reset_stream(stream, 0, nullptr, dummy_cb, nullptr);
    ASSERT_EQ(HS_INVALID, err);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_close_stream(stream, scratch, nullptr, nullptr);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

// hs_copy_stream: Call with no from_id
TEST(HyperscanArgChecks, CopyStreamNoFromId) {
    hs_stream_t *to;
    hs_error_t err = hs_copy_stream(&to, nullptr);
    ASSERT_EQ(HS_INVALID, err);
}

// hs_copy_stream: Call with no to_id
TEST(HyperscanArgChecks, CopyStreamNoToId) {
    hs_stream_t *stream = nullptr;
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_STREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    err = hs_copy_stream(nullptr, stream);
    ASSERT_EQ(HS_INVALID, err);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_close_stream(stream, scratch, nullptr, nullptr);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(HyperscanArgChecks, ResetAndCopyStreamNoToId) {
    hs_stream_t *stream = nullptr;
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_STREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    err = hs_reset_and_copy_stream(nullptr, stream, scratch, nullptr, nullptr);
    ASSERT_EQ(HS_INVALID, err);

    err = hs_close_stream(stream, scratch, nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(HyperscanArgChecks, ResetAndCopyStreamNoFromId) {
    hs_stream_t *stream = nullptr;
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_STREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    err = hs_reset_and_copy_stream(stream, nullptr, scratch, nullptr, nullptr);
    ASSERT_EQ(HS_INVALID, err);

    err = hs_close_stream(stream, scratch, nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(HyperscanArgChecks, ResetAndCopyStreamSameToId) {
    hs_stream_t *stream = nullptr;
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_STREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    err = hs_reset_and_copy_stream(stream, stream, scratch, nullptr, nullptr);
    ASSERT_EQ(HS_INVALID, err);

    err = hs_close_stream(stream, scratch, nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

// hs_reset_and_copy_stream: You're allowed to reset and copy a stream with no
// scratch and no callback.
TEST(HyperscanArgChecks, ResetAndCopyStreamNoCallbackOrScratch) {
    hs_stream_t *stream = nullptr;
    hs_stream_t *stream_to = nullptr;
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_STREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_open_stream(db, 0, &stream_to);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_reset_and_copy_stream(stream_to, stream, nullptr, nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_close_stream(stream_to, scratch, nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    err = hs_close_stream(stream, scratch, nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

// hs_reset_and_copy_stream: If you specify a callback, you must provide
// scratch.
TEST(HyperscanArgChecks, ResetAndCopyStreamNoScratch) {
    hs_stream_t *stream = nullptr;
    hs_stream_t *stream_to = nullptr;
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_STREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_open_stream(db, 0, &stream_to);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_reset_and_copy_stream(stream_to, stream, nullptr, dummy_cb,
                                   nullptr);
    ASSERT_EQ(HS_INVALID, err);

    err = hs_close_stream(stream_to, scratch, nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    err = hs_close_stream(stream, scratch, nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(HyperscanArgChecks, ResetAndCopyStreamDiffDb) {
    hs_stream_t *stream = nullptr;
    hs_stream_t *stream_to = nullptr;
    hs_database_t *db = nullptr;
    hs_database_t *db2 = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_STREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_compile("barfoo", 0, HS_MODE_STREAM, nullptr, &db2, &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    err = hs_alloc_scratch(db2, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_open_stream(db2, 0, &stream_to);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_reset_and_copy_stream(stream_to, stream, scratch, nullptr,
                                   nullptr);
    ASSERT_EQ(HS_INVALID, err);

    err = hs_close_stream(stream_to, scratch, nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    err = hs_close_stream(stream, scratch, nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
    hs_free_database(db2);
}

// hs_scan: Call with no database
TEST(HyperscanArgChecks, ScanBlockNoDatabase) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_NOSTREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);
    err = hs_scan(nullptr, "data", 4, 0, scratch, dummy_cb, nullptr);
    ASSERT_NE(HS_SUCCESS, err);
    EXPECT_NE(HS_SCAN_TERMINATED, err);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

// hs_scan: Call with a database with broken magic
TEST(HyperscanArgChecks, ScanBlockBrokenDatabaseMagic) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_NOSTREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    // break the database here, after scratch alloc
    breakDatabaseMagic(db);

    err = hs_scan(db, "data", 4, 0, scratch, dummy_cb, nullptr);
    ASSERT_EQ(HS_INVALID, err);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    free(db);
}

// hs_scan: Call with a database with broken version
TEST(HyperscanArgChecks, ScanBlockBrokenDatabaseVersion) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_NOSTREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    // break the database here, after scratch alloc
    breakDatabaseVersion(db);

    err = hs_scan(db, "data", 4, 0, scratch, dummy_cb, nullptr);
    ASSERT_EQ(HS_DB_VERSION_ERROR, err);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

// hs_scan: Call with a database with broken bytecode offset
TEST(HyperscanArgChecks, ScanBlockBrokenDatabaseBytecode) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_NOSTREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    // break the database here, after scratch alloc
    breakDatabaseBytecode(db);

    err = hs_scan(db, "data", 4, 0, scratch, dummy_cb, nullptr);
    ASSERT_EQ(HS_INVALID, err);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

// hs_scan: Call with a database built for streaming mode
TEST(HyperscanArgChecks, ScanBlockStreamingDatabase) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_STREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    err = hs_scan(db, "data", 4, 0, scratch, dummy_cb, nullptr);
    ASSERT_EQ(HS_DB_MODE_ERROR, err);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(HyperscanArgChecks, ScanBlockVectoredDatabase) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_VECTORED, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    err = hs_scan(db, "data", 4, 0, scratch, dummy_cb, nullptr);
    ASSERT_EQ(HS_DB_MODE_ERROR, err);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}


// hs_scan: Call with no data
TEST(HyperscanArgChecks, ScanBlockNoData) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_NOSTREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);
    err = hs_scan(db, nullptr, 4, 0, scratch, dummy_cb, nullptr);
    ASSERT_NE(HS_SUCCESS, err);
    EXPECT_NE(HS_SCAN_TERMINATED, err);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

// hs_scan: Call with no scratch
TEST(HyperscanArgChecks, ScanBlockNoScratch) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_NOSTREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);
    err = hs_scan(db, "data", 4, 0, nullptr, dummy_cb, nullptr);
    ASSERT_NE(HS_SUCCESS, err);
    EXPECT_NE(HS_SCAN_TERMINATED, err);

    // teardown
    hs_free_database(db);
}

// hs_scan: Call with no event handler
TEST(HyperscanArgChecks, ScanBlockNoHandler) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_NOSTREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);
    err = hs_scan(db, "data", 4, 0, scratch, nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_NE(HS_SCAN_TERMINATED, err);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

// hs_scan_vector: Call with no database
TEST(HyperscanArgChecks, ScanVectorNoDatabase) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_VECTORED, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);
    const char *data[] = {"data", "data"};
    unsigned int len[] = {4, 4};
    err = hs_scan_vector(nullptr, data, len, 2, 0, scratch, dummy_cb, nullptr);
    ASSERT_NE(HS_SUCCESS, err);
    EXPECT_NE(HS_SCAN_TERMINATED, err);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

// hs_scan_vector: Call with a database with broken magic
TEST(HyperscanArgChecks, ScanVectorBrokenDatabaseMagic) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_VECTORED, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    // break the database here, after scratch alloc
    breakDatabaseMagic(db);

    const char *data[] = {"data", "data"};
    unsigned int len[] = {4, 4};
    err = hs_scan_vector(db, data, len, 2, 0, scratch, dummy_cb, nullptr);
    ASSERT_EQ(HS_INVALID, err);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    free(db);
}

// hs_scan_vector: Call with a database with broken version
TEST(HyperscanArgChecks, ScanVectorBrokenDatabaseVersion) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_VECTORED, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    // break the database here, after scratch alloc
    breakDatabaseVersion(db);

    const char *data[] = {"data", "data"};
    unsigned int len[] = {4, 4};
    err = hs_scan_vector(db, data, len, 2, 0, scratch, dummy_cb, nullptr);
    ASSERT_EQ(HS_DB_VERSION_ERROR, err);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

// hs_scan_vector: Call with a database with broken bytecode offset
TEST(HyperscanArgChecks, ScanVectorBrokenDatabaseBytecode) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_VECTORED, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    // break the database here, after scratch alloc
    breakDatabaseBytecode(db);

    const char *data[] = {"data", "data"};
    unsigned int len[] = {4, 4};
    err = hs_scan_vector(db, data, len, 2, 0, scratch, dummy_cb, nullptr);
    ASSERT_EQ(HS_INVALID, err);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

// hs_scan_vector: Call with a database built for streaming mode
TEST(HyperscanArgChecks, ScanVectorStreamingDatabase) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_STREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    const char *data[] = {"data", "data"};
    unsigned int len[] = {4, 4};
    err = hs_scan_vector(db, data, len, 2, 0, scratch, dummy_cb, nullptr);
    ASSERT_EQ(HS_DB_MODE_ERROR, err);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(HyperscanArgChecks, ScanVectorBlockDatabase) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_BLOCK, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    const char *data[] = {"data", "data"};
    unsigned int len[] = {4, 4};
    err = hs_scan_vector(db, data, len, 2, 0, scratch, dummy_cb, nullptr);
    ASSERT_EQ(HS_DB_MODE_ERROR, err);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

// hs_scan_vector: Call with null data
TEST(HyperscanArgChecks, ScanVectorNoDataArray) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_VECTORED, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    unsigned int len[] = {4, 4};
    err = hs_scan_vector(db, nullptr, len, 2, 0, scratch, dummy_cb, nullptr);
    ASSERT_NE(HS_SUCCESS, err);
    EXPECT_NE(HS_SCAN_TERMINATED, err);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(HyperscanArgChecks, ScanVectorNoDataBlock) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_VECTORED, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    const char *data[] = {"data", nullptr};
    unsigned int len[] = {4, 4};
    err = hs_scan_vector(db, data, len, 2, 0, scratch, dummy_cb, nullptr);
    ASSERT_NE(HS_SUCCESS, err);
    EXPECT_NE(HS_SCAN_TERMINATED, err);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(HyperscanArgChecks, ScanVectorNoLenArray) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_VECTORED, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    const char *data[] = {"data", "data"};
    err = hs_scan_vector(db, data, nullptr, 2, 0, scratch, dummy_cb, nullptr);
    ASSERT_NE(HS_SUCCESS, err);
    EXPECT_NE(HS_SCAN_TERMINATED, err);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

// hs_scan_vector: Call with no scratch
TEST(HyperscanArgChecks, ScanVectorNoScratch) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_VECTORED, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);
    const char *data[] = {"data", "data"};
    unsigned int len[] = {4, 4};
    err = hs_scan_vector(db, data, len, 2, 0, nullptr, dummy_cb, nullptr);
    ASSERT_NE(HS_SUCCESS, err);
    EXPECT_NE(HS_SCAN_TERMINATED, err);

    // teardown
    hs_free_database(db);
}

// hs_scan_vector: Call with no event handler
TEST(HyperscanArgChecks, ScanVectorNoHandler) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_VECTORED, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);
    const char *data[] = {"data", "data"};
    unsigned int len[] = {4, 4};
    err = hs_scan_vector(db, data, len, 2, 0, scratch, nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_NE(HS_SCAN_TERMINATED, err);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

// hs_alloc_scratch: Call with no database
TEST(HyperscanArgChecks, AllocScratchNoDatabase) {
    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(nullptr, &scratch);
    EXPECT_NE(HS_SUCCESS, err);
    EXPECT_TRUE(scratch == nullptr);
}

// hs_alloc_scratch: Call with NULL ptr-to-scratch
TEST(HyperscanArgChecks, AllocScratchNullScratchPtr) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_NOSTREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    err = hs_alloc_scratch(db, nullptr);
    ASSERT_EQ(HS_INVALID, err);

    // teardown
    hs_free_database(db);
}

// hs_alloc_scratch: Call with bogus scratch
TEST(HyperscanArgChecks, AllocScratchBogusScratch) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_NOSTREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);
    hs_scratch_t *blah = (hs_scratch_t *)malloc(100);
    ASSERT_TRUE(blah != nullptr);
    memset(blah, 0xf0, 100);
    err = hs_alloc_scratch(db, &blah);
    ASSERT_EQ(HS_INVALID, err);

    // teardown
    free(blah);
    hs_free_database(db);
}

// hs_alloc_scratch: Call with broken database magic
TEST(HyperscanArgChecks, AllocScratchBadDatabaseMagic) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_NOSTREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    breakDatabaseMagic(db);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_INVALID, err);

    // teardown
    free(db);
}

// hs_alloc_scratch: Call with broken database version
TEST(HyperscanArgChecks, AllocScratchBadDatabaseVersion) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_NOSTREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    breakDatabaseVersion(db);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_DB_VERSION_ERROR, err);

    // teardown
    hs_free_database(db);
}

// hs_alloc_scratch: Call with broken database platform
TEST(HyperscanArgChecks, AllocScratchBadDatabasePlatform) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_NOSTREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    breakDatabasePlatform(db);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_DB_PLATFORM_ERROR, err);

    // teardown
    hs_free_database(db);
}

// hs_alloc_scratch: Call with broken database bytecode alignment
TEST(HyperscanArgChecks, AllocScratchBadDatabaseBytecode) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_NOSTREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    breakDatabaseBytecode(db);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_INVALID, err);

    // teardown
    hs_free_database(db);
}

// hs_alloc_scratch: Call with broken database CRC
TEST(HyperscanArgChecks, AllocScratchBadDatabaseCRC) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_NOSTREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    size_t len = 0;
    err = hs_database_size(db, &len);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(0U, len);

    // ensure that we're able to alloc scratch for the unbroken db.
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    // for want of a better case, corrupt the "middle byte" of the database.
    char *mid = (char *)db + len/2;
    *mid += 17;

    scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_INVALID, err);

    // teardown
    hs_free_database(db);
}

// hs_clone_scratch: Call with no source scratch
TEST(HyperscanArgChecks, CloneScratchNoSource) {
    hs_scratch_t *scratch = nullptr, *scratch2 = nullptr;
    hs_error_t err = hs_clone_scratch(scratch, &scratch2);
    EXPECT_NE(HS_SUCCESS, err);
    EXPECT_TRUE(scratch2 == nullptr);
}

// hs_serialize_database: Call with no database
TEST(HyperscanArgChecks, SerializeNoDatabase) {
    char *bytes = nullptr;
    size_t length = 0;
    hs_error_t err = hs_serialize_database(nullptr, &bytes, &length);
    EXPECT_NE(HS_SUCCESS, err);
    EXPECT_TRUE(bytes == nullptr);
}

// hs_serialize_database: Call with no bytes ptr
TEST(HyperscanArgChecks, SerializeNoBuffer) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_NOSTREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    size_t length = 0;
    err = hs_serialize_database(db, nullptr, &length);
    EXPECT_NE(HS_SUCCESS, err);
    EXPECT_EQ(0U, length);

    // teardown
    hs_free_database(db);
}

// hs_serialize_database: Call with no bytes ptr
TEST(HyperscanArgChecks, SerializeNoLength) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_NOSTREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    char *buf = nullptr;
    err = hs_serialize_database(db, &buf, nullptr);
    EXPECT_NE(HS_SUCCESS, err);
    EXPECT_TRUE(buf == nullptr);

    // teardown
    hs_free_database(db);
}

// hs_stream_size: Call with no database
TEST(HyperscanArgChecks, StreamSizeNoDatabase) {
    size_t sz;
    hs_error_t err = hs_stream_size(nullptr, &sz);
    ASSERT_EQ(HS_INVALID, err);
}

// hs_stream_size: Call with invalid database
TEST(HyperscanArgChecks, StreamSizeBogusDatabase) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_NOSTREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);
    size_t len;
    err = hs_database_size(db, &len);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_LT(0U, len);

    memset(db, 0xf0, len);

    size_t sz;
    err = hs_stream_size(db, &sz);
    ASSERT_EQ(HS_INVALID, err);

    free(db);
}

// hs_stream_size: Call with a block-mode database
TEST(HyperscanArgChecks, StreamSizeBlockDatabase) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_NOSTREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    size_t sz;
    err = hs_stream_size(db, &sz);
    ASSERT_EQ(HS_DB_MODE_ERROR, err);

    hs_free_database(db);
}

TEST(HyperscanArgChecks, StreamSizeVectoredDatabase) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_VECTORED, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    size_t sz;
    err = hs_stream_size(db, &sz);
    ASSERT_EQ(HS_DB_MODE_ERROR, err);

    hs_free_database(db);
}

TEST(HyperscanArgChecks, OpenStreamBlockDatabase) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_NOSTREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    hs_stream_t *id;
    err = hs_open_stream(db, 0, &id);
    ASSERT_EQ(HS_DB_MODE_ERROR, err);

    hs_free_database(db);
}

TEST(HyperscanArgChecks, OpenStreamVectoredDatabase) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_VECTORED, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    hs_stream_t *id;
    err = hs_open_stream(db, 0, &id);
    ASSERT_EQ(HS_DB_MODE_ERROR, err);

    hs_free_database(db);
}


// hs_stream_size: Call with a real database
TEST(HyperscanArgChecks, StreamSizeRealDatabase) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_STREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    size_t sz;
    err = hs_stream_size(db, &sz);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(0U, sz)
        << "Stream-mode database should have non-zero stream size";

    hs_free_database(db);
}

TEST(HyperscanArgChecks, StreamSizeNoSize) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_STREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    err = hs_stream_size(db, nullptr);
    ASSERT_EQ(HS_INVALID, err);

    hs_free_database(db);
}

// hs_database_size: Call with no database
TEST(HyperscanArgChecks, DatabaseSizeNoDatabase) {
    size_t sz;
    hs_error_t err = hs_database_size(nullptr, &sz);
    ASSERT_EQ(HS_INVALID, err);
}

TEST(HyperscanArgChecks, DatabaseSizeNoSize) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_STREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    err = hs_database_size(db, nullptr);
    ASSERT_EQ(HS_INVALID, err);

    hs_free_database(db);
}

TEST(HyperscanArgChecks, DatabaseSizeBadDb) {
    hs_database_t *db = (hs_database_t *)garbage;
    size_t sz;

    hs_error_t err = hs_database_size(db, &sz);
    ASSERT_EQ(HS_INVALID, err);
}

TEST(HyperscanArgChecks, DatabaseInfoBadDb) {
    hs_database_t *db = (hs_database_t *)garbage;
    char *info = garbage;
    hs_error_t err = hs_database_info(db, &info);
    ASSERT_EQ(HS_INVALID, err);
    ASSERT_TRUE(nullptr == info);
}

TEST(HyperscanArgChecks, DatabaseInfoNullDb) {
    char *info = garbage;
    hs_error_t err = hs_database_info(nullptr, &info);
    ASSERT_EQ(HS_INVALID, err);
    ASSERT_TRUE(nullptr == info);
}

TEST(HyperscanArgChecks, DatabaseInfoNullInfo) {
    hs_database_t *db;
    hs_compile_error_t *c_err;
    static const char *pattern = "hatstand.*(badgerbrush|teakettle)";

    hs_error_t err = hs_compile(pattern, 0, HS_MODE_NOSTREAM, nullptr, &db, &c_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    err = hs_database_info(db, nullptr);
    ASSERT_EQ(HS_INVALID, err);

    hs_free_database(db);
}

TEST(HyperscanArgChecks, SerializedDatabaseSizeBadLen) {
    hs_database_t *db;
    hs_compile_error_t *c_err;
    static const char *pattern = "hatstand.*(badgerbrush|teakettle)";

    hs_error_t err = hs_compile(pattern, 0, HS_MODE_NOSTREAM, nullptr, &db, &c_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    size_t db_len;
    err = hs_database_size(db, &db_len);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(0, db_len);

    char *bytes = nullptr;
    size_t bytes_len = 0;

    err = hs_serialize_database(db, &bytes, &bytes_len);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(0, bytes_len);

    hs_free_database(db);

    size_t ser_len;
    err = hs_serialized_database_size(bytes, 16, &ser_len);
    ASSERT_EQ(HS_INVALID, err);

   free(bytes);
}

TEST(HyperscanArgChecks, SerializedDatabaseSizeNoSize) {
    hs_database_t *db;
    hs_compile_error_t *c_err;
    static const char *pattern = "hatstand.*(badgerbrush|teakettle)";

    hs_error_t err = hs_compile(pattern, 0, HS_MODE_NOSTREAM, nullptr, &db, &c_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    size_t db_len;
    err = hs_database_size(db, &db_len);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(0, db_len);

    char *bytes = nullptr;
    size_t bytes_len = 0;

    err = hs_serialize_database(db, &bytes, &bytes_len);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(0, bytes_len);

    hs_free_database(db);

    err = hs_serialized_database_size(bytes, bytes_len, nullptr);
    ASSERT_EQ(HS_INVALID, err);

   free(bytes);
}

TEST(HyperscanArgChecks, SerializedDatabaseSizeNoBytes) {
    size_t sz;
    hs_error_t err = hs_serialized_database_size(nullptr, 1024, &sz);
    ASSERT_EQ(HS_INVALID, err);
}

TEST(HyperscanArgChecks, SerializedDatabaseInfoBadLen) {
    hs_database_t *db;
    hs_compile_error_t *c_err;
    static const char *pattern = "hatstand.*(badgerbrush|teakettle)";

    hs_error_t err = hs_compile(pattern, 0, HS_MODE_NOSTREAM, nullptr, &db, &c_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    size_t db_len;
    err = hs_database_size(db, &db_len);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(0, db_len);

    char *bytes = nullptr;
    size_t bytes_len = 0;

    err = hs_serialize_database(db, &bytes, &bytes_len);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(0, bytes_len);

    hs_free_database(db);

    char *info = garbage;
    err = hs_serialized_database_info(bytes, 16, &info);
    ASSERT_EQ(HS_INVALID, err);
    ASSERT_TRUE(nullptr == info);

   free(bytes);
}

TEST(HyperscanArgChecks, SerializedDatabaseInfoNoInfo) {
    hs_database_t *db;
    hs_compile_error_t *c_err;
    static const char *pattern = "hatstand.*(badgerbrush|teakettle)";

    hs_error_t err = hs_compile(pattern, 0, HS_MODE_NOSTREAM, nullptr, &db, &c_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    size_t db_len;
    err = hs_database_size(db, &db_len);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(0, db_len);

    char *bytes = nullptr;
    size_t bytes_len = 0;

    err = hs_serialize_database(db, &bytes, &bytes_len);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(0, bytes_len);

    hs_free_database(db);

    err = hs_serialized_database_info(bytes, bytes_len, nullptr);
    ASSERT_EQ(HS_INVALID, err);

   free(bytes);
}

TEST(HyperscanArgChecks, SerializedDatabaseInfoNoBytes) {
    char *info = garbage;
    hs_error_t err = hs_serialized_database_info(nullptr, 1024, &info);
    ASSERT_EQ(HS_INVALID, err);
    ASSERT_TRUE(nullptr == info);
}

TEST(HyperscanArgChecks, DeserializeDatabaseNoBytes) {
    hs_database_t *db = nullptr;
    hs_error_t err = hs_deserialize_database(nullptr, 2048, &db);
    ASSERT_EQ(HS_INVALID, err);
}

TEST(HyperscanArgChecks, DeserializeDatabaseAtNoBytes) {
    hs_database_t *db = nullptr;
    hs_error_t err = hs_deserialize_database_at(nullptr, 2048, db);
    ASSERT_EQ(HS_INVALID, err);
}

static
void makeSerializedDatabase(char **bytes, size_t *length) {
    hs_database_t *db = buildDB("(foo.*bar){3,}", 0, 0, HS_MODE_BLOCK);
    ASSERT_NE(nullptr, db);
    hs_error_t err = hs_serialize_database(db, bytes, length);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(HyperscanArgChecks, DeserializeDatabaseNoDb) {
    char *bytes = nullptr;
    size_t length = 0;
    makeSerializedDatabase(&bytes, &length);
    hs_error_t err = hs_deserialize_database(bytes, length, nullptr);
    ASSERT_EQ(HS_INVALID, err);
    free(bytes);
}

TEST(HyperscanArgChecks, DeserializeDatabaseBadLen) {
    char *bytes = nullptr;
    size_t length = 0;
    makeSerializedDatabase(&bytes, &length);
    hs_database_t *db = nullptr;
    hs_error_t err = hs_deserialize_database(bytes, length - 1, &db);
    ASSERT_EQ(HS_INVALID, err);
    free(bytes);
}

TEST(HyperscanArgChecks, DeserializeDatabaseBadLen2) {
    char *bytes = nullptr;
    size_t length = 0;
    makeSerializedDatabase(&bytes, &length);
    hs_database_t *db = nullptr;
    hs_error_t err = hs_deserialize_database(bytes, length + 1, &db);
    ASSERT_EQ(HS_INVALID, err);
    free(bytes);
}

TEST(HyperscanArgChecks, DeserializeDatabaseBadBytes) {
    char *bytes = nullptr;
    size_t length = 0;
    makeSerializedDatabase(&bytes, &length);
    memset(bytes, 0xff, length); // scribble
    hs_database_t *db = nullptr;
    hs_error_t err = hs_deserialize_database(bytes, length, &db);
    ASSERT_EQ(HS_INVALID, err);
    free(bytes);
}

TEST(HyperscanArgChecks, DeserializeDatabaseBadBytes2) {
    char *bytes = nullptr;
    size_t length = 0;
    makeSerializedDatabase(&bytes, &length);
    memset(bytes, 0, length); // scribble
    hs_database_t *db = nullptr;
    hs_error_t err = hs_deserialize_database(bytes, length, &db);
    ASSERT_EQ(HS_INVALID, err);
    free(bytes);
}

TEST(HyperscanArgChecks, DeserializeDatabaseAtNoDb) {
    char *bytes = nullptr;
    size_t length = 0;
    makeSerializedDatabase(&bytes, &length);
    hs_error_t err = hs_deserialize_database_at(bytes, length, nullptr);
    ASSERT_EQ(HS_INVALID, err);
    free(bytes);
}

TEST(HyperscanArgChecks, DeserializeDatabaseAtBadLen) {
    char *bytes = nullptr;
    size_t length = 0;
    makeSerializedDatabase(&bytes, &length);
    size_t db_length = 0;
    hs_error_t err = hs_serialized_database_size(bytes, length, &db_length);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_database_t *db = (hs_database_t *)malloc(db_length);
    err = hs_deserialize_database_at(bytes, length - 1, db);
    ASSERT_EQ(HS_INVALID, err);
    free(bytes);
    free(db);
}

TEST(HyperscanArgChecks, DeserializeDatabaseAtBadLen2) {
    char *bytes = nullptr;
    size_t length = 0;
    makeSerializedDatabase(&bytes, &length);
    size_t db_length = 0;
    hs_error_t err = hs_serialized_database_size(bytes, length, &db_length);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_database_t *db = (hs_database_t *)malloc(db_length);
    err = hs_deserialize_database_at(bytes, length + 1, db);
    ASSERT_EQ(HS_INVALID, err);
    free(bytes);
    free(db);
}

TEST(HyperscanArgChecks, DeserializeDatabaseAtBadBytes) {
    char *bytes = nullptr;
    size_t length = 0;
    makeSerializedDatabase(&bytes, &length);
    size_t db_length = 0;
    hs_error_t err = hs_serialized_database_size(bytes, length, &db_length);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_database_t *db = (hs_database_t *)malloc(db_length);
    memset(bytes, 0xff, length); // scribble
    err = hs_deserialize_database_at(bytes, length, db);
    ASSERT_EQ(HS_INVALID, err);
    free(bytes);
    free(db);
}

TEST(HyperscanArgChecks, DeserializeDatabaseAtBadBytes2) {
    char *bytes = nullptr;
    size_t length = 0;
    makeSerializedDatabase(&bytes, &length);
    size_t db_length = 0;
    hs_error_t err = hs_serialized_database_size(bytes, length, &db_length);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_database_t *db = (hs_database_t *)malloc(db_length);
    memset(bytes, 0, length); // scribble
    err = hs_deserialize_database_at(bytes, length, db);
    ASSERT_EQ(HS_INVALID, err);
    free(bytes);
    free(db);
}

TEST(HyperscanArgChecks, ScratchSizeNoSize) {
    hs_error_t err;

    // build a database
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    err = hs_compile("foo.*bar$", 0, HS_MODE_STREAM, nullptr, &db, &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);

    // alloc some scratch
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_TRUE(scratch != nullptr);

    err = hs_scratch_size(scratch, nullptr);
    ASSERT_EQ(HS_INVALID, err);

    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(HyperscanArgChecks, ScratchSizeNoScratch) {
    size_t size;
    hs_error_t err = hs_scratch_size(nullptr, &size);
    ASSERT_EQ(HS_INVALID, err);
}

TEST(HyperscanArgChecks, ScratchSizeBadScratch) {
    hs_scratch_t *scratch = (hs_scratch_t *)garbage;
    size_t size;
    hs_error_t err = hs_scratch_size(scratch, &size);
    ASSERT_EQ(HS_INVALID, err);
}

// hs_clone_scratch: bad scratch arg
TEST(HyperscanArgChecks, CloneBadScratch) {
    // Try cloning the scratch
    void *local_garbage = malloc(sizeof(garbage));
    ASSERT_TRUE(local_garbage != nullptr);
    memcpy(local_garbage, garbage, sizeof(garbage));
    hs_scratch_t *cloned = nullptr;
    hs_scratch_t *scratch = (hs_scratch_t *)local_garbage;
    hs_error_t err = hs_clone_scratch(scratch, &cloned);
    free(local_garbage);
    ASSERT_EQ(HS_INVALID, err);
}

// hs_scan: bad scratch arg
TEST(HyperscanArgChecks, ScanBadScratch) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_NOSTREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    void *local_garbage = malloc(sizeof(garbage));
    ASSERT_TRUE(local_garbage != nullptr);
    memcpy(local_garbage, garbage, sizeof(garbage));

    hs_scratch_t *scratch = (hs_scratch_t *)local_garbage;
    err = hs_scan(db, "data", 4, 0, scratch, dummy_cb, nullptr);
    free(local_garbage);
    ASSERT_EQ(HS_INVALID, err);

    // teardown
    hs_free_database(db);
}

// hs_scan_stream: bad scratch arg
TEST(HyperscanArgChecks, ScanStreamBadScratch) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_STREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);
    void *local_garbage = malloc(sizeof(garbage));
    ASSERT_TRUE(local_garbage != nullptr);
    memcpy(local_garbage, garbage, sizeof(garbage));
    hs_scratch_t *scratch = (hs_scratch_t *)local_garbage;

    hs_stream_t *stream = nullptr;
    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    err = hs_scan_stream(stream, "data", 4, 0, scratch, nullptr, nullptr);
    EXPECT_NE(HS_SUCCESS, err);
    EXPECT_NE(HS_SCAN_TERMINATED, err);

    // teardown
    err = hs_close_stream(stream, scratch, dummy_cb, nullptr);
    EXPECT_EQ(HS_INVALID, err);
    scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);
    hs_close_stream(stream, scratch, nullptr, nullptr);
    hs_free_database(db);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    free(local_garbage);
}

// hs_reset_stream: bad scratch arg
TEST(HyperscanArgChecks, ResetStreamBadScratch) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_STREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);
    void *local_garbage = malloc(sizeof(garbage));
    ASSERT_TRUE(local_garbage != nullptr);
    memcpy(local_garbage, garbage, sizeof(garbage));
    hs_scratch_t *scratch = (hs_scratch_t *)local_garbage;

    hs_stream_t *stream = nullptr;
    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    err = hs_reset_stream(stream, 0, scratch, dummy_cb, nullptr);
    EXPECT_NE(HS_SUCCESS, err);
    EXPECT_NE(HS_SCAN_TERMINATED, err);

    // teardown
    err = hs_close_stream(stream, scratch, dummy_cb, nullptr);
    EXPECT_EQ(HS_INVALID, err);
    scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);
    hs_close_stream(stream, scratch, nullptr, nullptr);
    hs_free_database(db);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    free(local_garbage);
}

// hs_scan_stream: bad scratch arg
TEST(HyperscanArgChecks, ScanVectorBadScratch) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_VECTORED, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);
    void *local_garbage = malloc(sizeof(garbage));
    ASSERT_TRUE(local_garbage != nullptr);
    memcpy(local_garbage, garbage, sizeof(garbage));
    hs_scratch_t *scratch = (hs_scratch_t *)local_garbage;

    const char *data[] = { "data" };
    unsigned int len[] = { 4 };

    err = hs_scan_vector(db, data, len, 1, 0, scratch, dummy_cb, nullptr);

    EXPECT_NE(HS_SUCCESS, err);
    EXPECT_NE(HS_SCAN_TERMINATED, err);

    // teardown
    hs_free_database(db);
    free(local_garbage);
}

// hs_scan: bad (freed) scratch arg
// disabled as, unsurprisingly, valgrind complains
#ifdef TEST_FREED_MEM
TEST(HyperscanArgChecks, ScanFreedScratch) {
    hs_database_t *db = 0;
    hs_compile_error_t *compile_err = 0;
    hs_error_t err = hs_compile("foobar", 0, HS_MODE_NOSTREAM, NULL, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != NULL);
    hs_scratch_t *scratch = 0;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != NULL);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    err = hs_scan(db, "data", 4, 0, scratch, dummy_cb, 0);
    ASSERT_EQ(HS_INVALID, err);
    EXPECT_NE(HS_SCAN_TERMINATED, err);

    // teardown
    hs_free_database(db);
}
#endif // TEST_FREED_MEM

// hs_expression_info: Compile a NULL pattern
TEST(HyperscanArgChecks, ExprInfoNullExpression) {
    hs_expr_info_t *info = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_expression_info(nullptr, 0, &info, &compile_err);
    EXPECT_EQ(HS_COMPILER_ERROR, err);
    EXPECT_TRUE(info == nullptr);
    EXPECT_TRUE(compile_err != nullptr);
    hs_free_compile_error(compile_err);
}

// hs_expression_info: NULL info block ptr
TEST(HyperscanArgChecks, ExprInfoNullInfoPtr) {
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_expression_info("foobar", 0, nullptr, &compile_err);
    EXPECT_EQ(HS_COMPILER_ERROR, err);
    EXPECT_TRUE(compile_err != nullptr);
    hs_free_compile_error(compile_err);
}

// hs_expression_info: No compiler error block
TEST(HyperscanArgChecks, ExprInfoNullErrPtr) {
    hs_expr_info_t *info = nullptr;
    hs_error_t err = hs_expression_info("foobar", 0, &info, nullptr);
    EXPECT_EQ(HS_COMPILER_ERROR, err);
    EXPECT_TRUE(info == nullptr);
}

// hs_expression_ext_info: Compile a NULL pattern
TEST(HyperscanArgChecks, ExprExtInfoNullExpression) {
    hs_expr_info_t *info = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err =
        hs_expression_ext_info(nullptr, 0, nullptr, &info, &compile_err);
    EXPECT_EQ(HS_COMPILER_ERROR, err);
    EXPECT_TRUE(info == nullptr);
    EXPECT_TRUE(compile_err != nullptr);
    hs_free_compile_error(compile_err);
}

// hs_expression_ext_info: NULL info block ptr
TEST(HyperscanArgChecks, ExprExtInfoNullInfoPtr) {
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err =
        hs_expression_ext_info("foobar", 0, nullptr, nullptr, &compile_err);
    EXPECT_EQ(HS_COMPILER_ERROR, err);
    EXPECT_TRUE(compile_err != nullptr);
    hs_free_compile_error(compile_err);
}

// hs_expression_ext_info: No compiler error block
TEST(HyperscanArgChecks, ExprExtInfoNullErrPtr) {
    hs_expr_info_t *info = nullptr;
    hs_error_t err =
        hs_expression_ext_info("foobar", 0, nullptr, &info, nullptr);
    EXPECT_EQ(HS_COMPILER_ERROR, err);
    EXPECT_TRUE(info == nullptr);
}

TEST(HyperscanArgChecks, hs_free_database_null) {
    hs_error_t err = hs_free_database(nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
}

TEST(HyperscanArgChecks, hs_free_database_garbage) {
    hs_error_t err = hs_free_database((hs_database_t *)garbage);
    ASSERT_EQ(HS_INVALID, err);
}

TEST(HyperscanArgChecks, hs_free_scratch_null) {
    hs_error_t err = hs_free_scratch(nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
}

TEST(HyperscanArgChecks, hs_free_scratch_garbage) {
    hs_error_t err = hs_free_scratch((hs_scratch_t *)garbage);
    ASSERT_EQ(HS_INVALID, err);
}

TEST(HyperscanArgChecks, hs_free_compile_error_null) {
    hs_error_t err = hs_free_compile_error(nullptr);
    ASSERT_EQ(HS_SUCCESS, err);
}

TEST(HyperscanArgChecks, multicompile_mix_highlander_1) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    const char *expr[] = {"aoo[A-K]", "bar[L-Z]"};
    unsigned flags[] = {HS_FLAG_SINGLEMATCH, 0};
    unsigned ids[] = {30, 30};
    hs_error_t err = hs_compile_multi(expr, flags, ids, 2, HS_MODE_NOSTREAM,
                                      nullptr, &db, &compile_err);
    ASSERT_EQ(HS_COMPILER_ERROR, err);
    hs_free_compile_error(compile_err);
}

TEST(HyperscanArgChecks, multicompile_mix_highlander_2) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    const char *expr[] = {"aoo[A-K]", "bar[L-Z]"};
    unsigned flags[] = {0, HS_FLAG_SINGLEMATCH};
    unsigned ids[] = {30, 30};
    hs_error_t err = hs_compile_multi(expr, flags, ids, 2, HS_MODE_NOSTREAM,
                                      nullptr, &db, &compile_err);
    ASSERT_EQ(HS_COMPILER_ERROR, err);
    hs_free_compile_error(compile_err);
}

TEST(HyperscanArgChecks, multicompile_nomix_highlander_1) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    const char *expr[] = {"aoo[A-K]", "bar[L-Z]"};
    unsigned flags[] = {HS_FLAG_SINGLEMATCH, HS_FLAG_SINGLEMATCH};
    unsigned ids[] = {30, 30};
    hs_error_t err = hs_compile_multi(expr, flags, ids, 2, HS_MODE_NOSTREAM,
                                      nullptr, &db, &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(HyperscanArgChecks, multicompile_nomix_highlander_2) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    const char *expr[] = {"aoo[A-K]", "bar[L-Z]"};
    unsigned flags[] = {0, 0};
    unsigned ids[] = {30, 30};
    hs_error_t err = hs_compile_multi(expr, flags, ids, 2, HS_MODE_NOSTREAM,
                                      nullptr, &db, &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(HyperscanArgChecks, hs_populate_platform_null) {
    hs_error_t err = hs_populate_platform(nullptr);
    ASSERT_EQ(HS_INVALID, err);
}

TEST(HyperscanArgChecks, CompressStreamNoStream) {
    char buf[100];
    size_t used;
    hs_error_t err = hs_compress_stream(nullptr, buf, sizeof(buf), &used);
    ASSERT_EQ(HS_INVALID, err);
}

TEST(HyperscanArgChecks, CompressStreamNoUsed) {
    hs_database_t *db = buildDB("(foo.*bar){3,}", 0, 0, HS_MODE_STREAM);
    ASSERT_NE(nullptr, db);

    hs_stream_t *stream;
    hs_error_t err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);

    char buf[100];
    err = hs_compress_stream(stream, buf, sizeof(buf), nullptr);
    ASSERT_EQ(HS_INVALID, err);

    err = hs_close_stream(stream, nullptr, nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_free_database(db);
    ASSERT_EQ(HS_SUCCESS, err);
}

TEST(HyperscanArgChecks, CompressStreamNoBuf) {
    hs_database_t *db = buildDB("(foo.*bar){3,}", 0, 0, HS_MODE_STREAM);
    ASSERT_NE(nullptr, db);

    hs_stream_t *stream;
    hs_error_t err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);

    char buf[100];
    size_t used;
    err = hs_compress_stream(stream, nullptr, sizeof(buf), &used);
    ASSERT_EQ(HS_INVALID, err);

    err = hs_close_stream(stream, nullptr, nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_free_database(db);
    ASSERT_EQ(HS_SUCCESS, err);
}

TEST(HyperscanArgChecks, CompressStreamSmallBuff) {
    hs_database_t *db = buildDB("(foo.*bar){3,}", 0, 0, HS_MODE_STREAM);
    ASSERT_NE(nullptr, db);

    hs_stream_t *stream;
    hs_error_t err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);

    char buf[100];
    size_t used = 0;
    err = hs_compress_stream(stream, buf, 1, &used);
    ASSERT_EQ(HS_INSUFFICIENT_SPACE, err);
    ASSERT_LT(0, used);

    err = hs_close_stream(stream, nullptr, nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_free_database(db);
    ASSERT_EQ(HS_SUCCESS, err);
}

TEST(HyperscanArgChecks, ExpandNoDb) {
    hs_database_t *db = buildDB("(foo.*bar){3,}", 0, 0, HS_MODE_STREAM);
    ASSERT_NE(nullptr, db);

    hs_stream_t *stream1;
    hs_error_t err = hs_open_stream(db, 0, &stream1);
    ASSERT_EQ(HS_SUCCESS, err);

    char buf[2000];
    size_t used = 0;
    err = hs_compress_stream(stream1, buf, sizeof(buf), &used);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_stream_t *stream2;
    err = hs_expand_stream(nullptr, &stream2, buf, used);
    ASSERT_EQ(HS_INVALID, err);

    err = hs_close_stream(stream1, nullptr, nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_free_database(db);
    ASSERT_EQ(HS_SUCCESS, err);
}

TEST(HyperscanArgChecks, ExpandNoTo) {
    hs_database_t *db = buildDB("(foo.*bar){3,}", 0, 0, HS_MODE_STREAM);
    ASSERT_NE(nullptr, db);

    hs_stream_t *stream1;
    hs_error_t err = hs_open_stream(db, 0, &stream1);
    ASSERT_EQ(HS_SUCCESS, err);

    char buf[2000];
    size_t used = 0;
    err = hs_compress_stream(stream1, buf, sizeof(buf), &used);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_stream_t *stream2;
    err = hs_expand_stream(db, nullptr, buf, used);
    ASSERT_EQ(HS_INVALID, err);

    err = hs_close_stream(stream1, nullptr, nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_free_database(db);
    ASSERT_EQ(HS_SUCCESS, err);
}

TEST(HyperscanArgChecks, ExpandNoBuf) {
    hs_database_t *db = buildDB("(foo.*bar){3,}", 0, 0, HS_MODE_STREAM);
    ASSERT_NE(nullptr, db);

    hs_stream_t *stream1;
    hs_error_t err = hs_open_stream(db, 0, &stream1);
    ASSERT_EQ(HS_SUCCESS, err);

    char buf[2000];
    size_t used = 0;
    err = hs_compress_stream(stream1, buf, sizeof(buf), &used);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_stream_t *stream2;
    err = hs_expand_stream(db, &stream2, nullptr, used);
    ASSERT_EQ(HS_INVALID, err);

    err = hs_close_stream(stream1, nullptr, nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_free_database(db);
    ASSERT_EQ(HS_SUCCESS, err);
}

TEST(HyperscanArgChecks, ExpandSmallBuf) {
    hs_database_t *db = buildDB("(foo.*bar){3,}", 0, 0, HS_MODE_STREAM);
    ASSERT_NE(nullptr, db);

    hs_stream_t *stream1;
    hs_error_t err = hs_open_stream(db, 0, &stream1);
    ASSERT_EQ(HS_SUCCESS, err);

    char buf[2000];
    size_t used = 0;
    err = hs_compress_stream(stream1, buf, sizeof(buf), &used);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_stream_t *stream2;
    err = hs_expand_stream(db, &stream2, buf, used / 2);
    ASSERT_EQ(HS_INVALID, err);

    err = hs_close_stream(stream1, nullptr, nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_free_database(db);
    ASSERT_EQ(HS_SUCCESS, err);
}

TEST(HyperscanArgChecks, ResetAndExpandNoStream) {
    hs_database_t *db = buildDB("(foo.*bar){3,}", 0, 0, HS_MODE_STREAM);
    ASSERT_NE(nullptr, db);

    hs_stream_t *stream1;
    hs_error_t err = hs_open_stream(db, 0, &stream1);
    ASSERT_EQ(HS_SUCCESS, err);

    char buf[2000];
    size_t used = 0;
    err = hs_compress_stream(stream1, buf, sizeof(buf), &used);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_reset_and_expand_stream(nullptr, buf, used, nullptr, nullptr,
                                     nullptr);
    ASSERT_EQ(HS_INVALID, err);

    err = hs_close_stream(stream1, nullptr, nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_free_database(db);
    ASSERT_EQ(HS_SUCCESS, err);
}

TEST(HyperscanArgChecks, ResetAndExpandNoBuf) {
    hs_database_t *db = buildDB("(foo.*bar){3,}", 0, 0, HS_MODE_STREAM);
    ASSERT_NE(nullptr, db);

    hs_stream_t *stream1;
    hs_error_t err = hs_open_stream(db, 0, &stream1);
    ASSERT_EQ(HS_SUCCESS, err);

    char buf[2000];
    size_t used = 0;
    err = hs_compress_stream(stream1, buf, sizeof(buf), &used);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_stream_t *stream2;
    err = hs_open_stream(db, 0, &stream2);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_reset_and_expand_stream(stream2, nullptr, used, nullptr, nullptr,
                                     nullptr);
    ASSERT_EQ(HS_INVALID, err);

    err = hs_close_stream(stream1, nullptr, nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_close_stream(stream2, nullptr, nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_free_database(db);
    ASSERT_EQ(HS_SUCCESS, err);
}


TEST(HyperscanArgChecks, ResetAndExpandSmallBuf) {
    hs_database_t *db = buildDB("(foo.*bar){3,}", 0, 0, HS_MODE_STREAM);
    ASSERT_NE(nullptr, db);

    hs_stream_t *stream1;
    hs_error_t err = hs_open_stream(db, 0, &stream1);
    ASSERT_EQ(HS_SUCCESS, err);

    char buf[2000];
    size_t used = 0;
    err = hs_compress_stream(stream1, buf, sizeof(buf), &used);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_stream_t *stream2;
    err = hs_open_stream(db, 0, &stream2);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_reset_and_expand_stream(stream2, buf, used / 2, nullptr, nullptr,
                                     nullptr);
    ASSERT_EQ(HS_INVALID, err);

    err = hs_close_stream(stream1, nullptr, nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_close_stream(stream2, nullptr, nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_free_database(db);
    ASSERT_EQ(HS_SUCCESS, err);
}

TEST(HyperscanArgChecks, ResetAndExpandNoScratch) {
    hs_database_t *db = buildDB("(foo.*bar){3,}", 0, 0, HS_MODE_STREAM);
    ASSERT_NE(nullptr, db);

    hs_stream_t *stream1;
    hs_error_t err = hs_open_stream(db, 0, &stream1);
    ASSERT_EQ(HS_SUCCESS, err);

    char buf[2000];
    size_t used = 0;
    err = hs_compress_stream(stream1, buf, sizeof(buf), &used);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_stream_t *stream2;
    err = hs_open_stream(db, 0, &stream2);
    ASSERT_EQ(HS_SUCCESS, err);

    int temp;

    err = hs_reset_and_expand_stream(stream2, buf, used, nullptr, singleHandler,
                                     &temp);
    ASSERT_EQ(HS_INVALID, err);

    err = hs_close_stream(stream1, nullptr, nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_close_stream(stream2, nullptr, nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_free_database(db);
    ASSERT_EQ(HS_SUCCESS, err);
}

class BadModeTest : public testing::TestWithParam<unsigned> {};

// hs_compile: Compile a pattern with bogus mode flags set.
TEST_P(BadModeTest, FailCompile) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;

    unsigned mode = GetParam();
    SCOPED_TRACE(mode);

    hs_error_t err = hs_compile("foo", 0, mode, nullptr, &db, &compile_err);
    ASSERT_EQ(HS_COMPILER_ERROR, err);
    ASSERT_TRUE(compile_err != nullptr);
    ASSERT_TRUE(compile_err->message != nullptr);
    hs_free_compile_error(compile_err);
}

static const unsigned badModeValues[] = {
    // Multiple modes at once.
    HS_MODE_BLOCK | HS_MODE_STREAM | HS_MODE_VECTORED,
    HS_MODE_BLOCK | HS_MODE_STREAM,
    HS_MODE_BLOCK | HS_MODE_VECTORED,
    HS_MODE_STREAM | HS_MODE_VECTORED,
    // SOM horizon modes are only accepted in streaming mode.
    HS_MODE_BLOCK | HS_MODE_SOM_HORIZON_LARGE,
    HS_MODE_BLOCK | HS_MODE_SOM_HORIZON_MEDIUM,
    HS_MODE_BLOCK | HS_MODE_SOM_HORIZON_SMALL,
    HS_MODE_VECTORED | HS_MODE_SOM_HORIZON_LARGE,
    HS_MODE_VECTORED | HS_MODE_SOM_HORIZON_MEDIUM,
    HS_MODE_VECTORED | HS_MODE_SOM_HORIZON_SMALL,
    // Can't specify more than one SOM horizon.
    HS_MODE_STREAM | HS_MODE_SOM_HORIZON_LARGE | HS_MODE_SOM_HORIZON_MEDIUM | HS_MODE_SOM_HORIZON_SMALL,
    HS_MODE_STREAM | HS_MODE_SOM_HORIZON_LARGE | HS_MODE_SOM_HORIZON_SMALL,
    HS_MODE_STREAM | HS_MODE_SOM_HORIZON_LARGE | HS_MODE_SOM_HORIZON_MEDIUM,
    HS_MODE_STREAM | HS_MODE_SOM_HORIZON_MEDIUM | HS_MODE_SOM_HORIZON_SMALL,
};

INSTANTIATE_TEST_CASE_P(HyperscanArgChecks, BadModeTest,
                        testing::ValuesIn(badModeValues));

} // namespace

