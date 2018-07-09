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

#include "gtest/gtest.h"
#include "chimera/ch.h"

static char garbage[] = "TEST(HybridArgChecks, DatabaseSizeNoDatabase) {" \
                        "    size_t sz = ch_database_size(0);" \
                        "    ASSERT_EQ(0, sz);";

namespace /* anonymous */ {

// Dummy callback: does nothing, returns 0 (keep matching)
ch_callback_t dummyHandler(unsigned, unsigned long long,
                           unsigned long long, unsigned, unsigned,
                           const ch_capture_t *, void *) {
    // empty
    return CH_CALLBACK_CONTINUE;
}

// Helper: correctly construct a simple database.
static
void makeDatabase(ch_database_t **hydb) {
    static const char *expr[] = { "foo.*bar" };
    ch_database_t *db = nullptr;
    ch_compile_error_t *compile_err = nullptr;
    ch_error_t err;

    err = ch_compile_multi(expr, nullptr, nullptr, 1, 0, nullptr, &db,
                           &compile_err);

    ASSERT_EQ(CH_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    *hydb = db;
}

// Helper: given a database, build me some scratch.
static
void makeScratch(const ch_database_t *db,
                 ch_scratch_t **scratch) {
    ch_error_t err = ch_alloc_scratch(db, scratch);
    ASSERT_EQ(CH_SUCCESS, err);
    ASSERT_TRUE(*scratch != nullptr);
}

// Break the magic number of the given database.
void breakDatabaseMagic(ch_database *db) {
    // database magic should be 0xdbdb at the start
    ASSERT_TRUE(memcmp("\xde\xde", db, 2) == 0);
    *(char *)db = 0xdc;
}

// Break the version number of the given database.
void breakDatabaseVersion(ch_database *db) {
    // database version is the second u32
    *((char *)db + 4) += 1;
}

// Check that CH_version gives us a reasonable string back
TEST(HybridArgChecks, Version) {
    const char *version = ch_version();
    ASSERT_TRUE(version != nullptr);
    ASSERT_TRUE(version[0] >= '0' && version[0] <= '9')
        << "First byte should be a digit.";
    ASSERT_EQ('.', version[1]) << "Second byte should be a dot.";
}

// ch_compile: Hand the compiler a bogus flag.
TEST(HybridArgChecks, SingleBogusFlags) {
    ch_database_t *db = nullptr;
    ch_compile_error_t *compile_err = nullptr;
    ch_error_t err;

    static const unsigned int badflags[] = {
        0xffffffff,
        16,
        128,
        256,
        512,
    };

    for (size_t i = 0; i < sizeof(badflags)/sizeof(badflags[0]); i++) {
        const char expr[]  = "foobar";
        err = ch_compile(expr, badflags[i], 0, nullptr, &db, &compile_err);
        EXPECT_EQ(CH_COMPILER_ERROR, err);
        EXPECT_TRUE(db == nullptr);
        EXPECT_TRUE(compile_err != nullptr);
        EXPECT_STREQ("Unrecognized flag used.", compile_err->message);
        ch_free_compile_error(compile_err);
    }
}

// ch_compile: Hand the compiler a bogus mode.
TEST(HybridArgChecks, SingleBogusMode) {
    ch_database_t *db = nullptr;
    ch_compile_error_t *compile_err = nullptr;
    ch_error_t err;

    static const unsigned int badModes[] = {
        0xffffffff,
        1,
        2,
        CH_MODE_GROUPS << 1, // this was our largest mode flag
    };

    for (size_t i = 0; i < sizeof(badModes)/sizeof(badModes[0]); i++) {
        const char expr[]  = "foobar";
        err = ch_compile(expr, 0,  badModes[i], nullptr, &db, &compile_err);
        EXPECT_EQ(CH_COMPILER_ERROR, err);
        EXPECT_TRUE(db == nullptr);
        EXPECT_TRUE(compile_err != nullptr);
        EXPECT_STREQ("Invalid mode flag supplied.", compile_err->message);
        ch_free_compile_error(compile_err);
    }
}

// ch_compile: Compile a nullptr pattern set)
TEST(HybridArgChecks, SingleCompileBlockNoPattern) {
    ch_database_t *db = nullptr;
    ch_compile_error_t *compile_err = nullptr;
    ch_error_t err;
    err = ch_compile(nullptr, 0, 0, nullptr, &db, &compile_err);
    EXPECT_EQ(CH_COMPILER_ERROR, err);
    EXPECT_TRUE(db == nullptr);
    EXPECT_TRUE(compile_err != nullptr);
    ch_free_compile_error(compile_err);
}

// ch_compile: Compile a pattern to a nullptr database ptr
TEST(HybridArgChecks, SingleCompileBlockNoDatabase) {
    ch_compile_error_t *compile_err = nullptr;
    const char expr[] = "foobar";
    ch_error_t err;
    err = ch_compile(expr, 0, 0, nullptr, nullptr, &compile_err);
    EXPECT_EQ(CH_COMPILER_ERROR, err);
    EXPECT_TRUE(compile_err != nullptr);
    ch_free_compile_error(compile_err);
}

// ch_compile_multi: Hand the compiler a bogus flag.
TEST(HybridArgChecks, MultiBogusFlags) {
    ch_database_t *db = nullptr;
    ch_compile_error_t *compile_err = nullptr;
    ch_error_t err;

    static const unsigned int badflags[] = {
        0xffffffff,
        16, // HS_FLAG_ERROREOD
        128,
        256,
        512,
    };

    for (size_t i = 0; i < sizeof(badflags)/sizeof(badflags[0]); i++) {
        const char *expr[]  = { "foobar" };
        err = ch_compile_multi(expr, &badflags[i], nullptr, 1, 0, nullptr, &db,
                               &compile_err);
        EXPECT_EQ(CH_COMPILER_ERROR, err);
        EXPECT_TRUE(db == nullptr);
        EXPECT_TRUE(compile_err != nullptr);
        EXPECT_STREQ("Unrecognized flag used.", compile_err->message);
        ch_free_compile_error(compile_err);
    }
}

// ch_compile_multi: Hand the ch_compile_multi a bogus mode.
TEST(HybridArgChecks, MultiBogusMode) {
    ch_database_t *db = nullptr;
    ch_compile_error_t *compile_err = nullptr;
    ch_error_t err;

    static const unsigned int badModes[] = {
        0xffffffff,
        1,
        2,
        CH_MODE_GROUPS << 1, // this was our largest mode flag
    };

    for (size_t i = 0; i < sizeof(badModes)/sizeof(badModes[0]); i++) {
        const char *expr[]  = { "foobar" };
        err = ch_compile_multi(expr, nullptr, nullptr, 1, badModes[i], nullptr,
                               &db, &compile_err);
        EXPECT_EQ(CH_COMPILER_ERROR, err);
        EXPECT_TRUE(db == nullptr);
        EXPECT_TRUE(compile_err != nullptr);
        EXPECT_STREQ("Invalid mode flag supplied.", compile_err->message);
        ch_free_compile_error(compile_err);
    }
}

// ch_compile_multi: Compile a nullptr pattern set (block mode)
TEST(HybridArgChecks, MultiCompileBlockNoPattern) {
    ch_database_t *db = nullptr;
    ch_compile_error_t *compile_err = nullptr;
    ch_error_t err;
    err = ch_compile_multi(nullptr, nullptr, nullptr, 1, 0, nullptr, &db,
                           &compile_err);
    EXPECT_EQ(CH_COMPILER_ERROR, err);
    EXPECT_TRUE(db == nullptr);
    EXPECT_TRUE(compile_err != nullptr);
    ch_free_compile_error(compile_err);
}

// ch_compile_multi: Compile a set of zero patterns
TEST(HybridArgChecks, MultiCompileZeroPatterns) {
    ch_database_t *db = nullptr;
    ch_compile_error_t *compile_err = nullptr;
    const char *expr[] = {"foobar"};
    ch_error_t err;
    err = ch_compile_multi(expr, nullptr, nullptr, 0, 0, nullptr, &db,
                           &compile_err);
    EXPECT_EQ(CH_COMPILER_ERROR, err);
    EXPECT_TRUE(db == nullptr);
    EXPECT_TRUE(compile_err != nullptr);
    ch_free_compile_error(compile_err);
}

// ch_compile_multi: Compile a pattern to a nullptr database ptr
TEST(HybridArgChecks, MultiCompileBlockNoDatabase) {
    ch_compile_error_t *compile_err = nullptr;
    const char *expr[] = {"foobar"};
    ch_error_t err;
    err = ch_compile_multi(expr, nullptr, nullptr, 1, 0, nullptr, nullptr,
                           &compile_err);
    EXPECT_EQ(CH_COMPILER_ERROR, err);
    EXPECT_TRUE(compile_err != nullptr);
    ch_free_compile_error(compile_err);
}

// ch_compile_ext_multi: Hand the compiler a bogus flag.
TEST(HybridArgChecks, ExtMultiBogusFlags) {
    ch_database_t *db = nullptr;
    ch_compile_error_t *compile_err = nullptr;
    ch_error_t err;

    static const unsigned int badflags[] = {
        0xffffffff,
        16, // HS_FLAG_ERROREOD
        128,
        256,
        512,
    };

    for (size_t i = 0; i < sizeof(badflags)/sizeof(badflags[0]); i++) {
        const char *expr[]  = { "foobar" };
        err = ch_compile_ext_multi(expr, &badflags[i], nullptr, 1, 0,
                                   10000000, 8000, nullptr, &db, &compile_err);
        EXPECT_EQ(CH_COMPILER_ERROR, err);
        EXPECT_TRUE(db == nullptr);
        EXPECT_TRUE(compile_err != nullptr);
        EXPECT_STREQ("Unrecognized flag used.", compile_err->message);
        ch_free_compile_error(compile_err);
    }
}

// ch_compile_ext_multi: Hand the ch_compile_multi a bogus mode.
TEST(HybridArgChecks, ExtMultiBogusMode) {
    ch_database_t *db = nullptr;
    ch_compile_error_t *compile_err = nullptr;
    ch_error_t err;

    static const unsigned int badModes[] = {
        0xffffffff,
        1,
        2,
        CH_MODE_GROUPS << 1, // this was our largest mode flag
    };

    for (size_t i = 0; i < sizeof(badModes)/sizeof(badModes[0]); i++) {
        const char *expr[]  = { "foobar" };
        err = ch_compile_ext_multi(expr, nullptr, nullptr, 1, badModes[i],
                                   10000000, 8000, nullptr, &db, &compile_err);
        EXPECT_EQ(CH_COMPILER_ERROR, err);
        EXPECT_TRUE(db == nullptr);
        EXPECT_TRUE(compile_err != nullptr);
        EXPECT_STREQ("Invalid mode flag supplied.", compile_err->message);
        ch_free_compile_error(compile_err);
    }
}

// ch_compile_ext_multi: Compile a nullptr pattern set (block mode)
TEST(HybridArgChecks, ExtMultiCompileBlockNoPattern) {
    ch_database_t *db = nullptr;
    ch_compile_error_t *compile_err = nullptr;
    ch_error_t err;
    err = ch_compile_ext_multi(nullptr, nullptr, nullptr, 1, 0, 10000000,
                               8000, nullptr, &db, &compile_err);
    EXPECT_EQ(CH_COMPILER_ERROR, err);
    EXPECT_TRUE(db == nullptr);
    EXPECT_TRUE(compile_err != nullptr);
    ch_free_compile_error(compile_err);
}

// ch_compile_ext_multi: Compile a set of zero patterns
TEST(HybridArgChecks, ExtMultiCompileZeroPatterns) {
    ch_database_t *db = nullptr;
    ch_compile_error_t *compile_err = nullptr;
    const char *expr[] = {"foobar"};
    ch_error_t err;
    err = ch_compile_ext_multi(expr, nullptr, nullptr, 0, 0, 10000000,
                               8000, nullptr, &db, &compile_err);
    EXPECT_EQ(CH_COMPILER_ERROR, err);
    EXPECT_TRUE(db == nullptr);
    EXPECT_TRUE(compile_err != nullptr);
    ch_free_compile_error(compile_err);
}

// ch_compile_ext_multi: Compile a pattern to a nullptr database ptr
TEST(HybridArgChecks, ExtMultiCompileBlockNoDatabase) {
    ch_compile_error_t *compile_err = nullptr;
    const char *expr[] = {"foobar"};
    ch_error_t err;
    err = ch_compile_ext_multi(expr, nullptr, nullptr, 1, 0, 10000000,
                           8000, nullptr, nullptr, &compile_err);
    EXPECT_EQ(CH_COMPILER_ERROR, err);
    EXPECT_TRUE(compile_err != nullptr);
    ch_free_compile_error(compile_err);
}

// ch_scan: Call with no database
TEST(HybridArgChecks, ScanBlockNoDatabase) {
    ch_database_t *db = nullptr;
    makeDatabase(&db);
    ch_scratch_t *scratch = nullptr;
    makeScratch(db, &scratch);

    ch_error_t err = ch_scan(nullptr, "data", 4, 0, scratch,
                             dummyHandler, nullptr, nullptr);
    ASSERT_NE(CH_SUCCESS, err);
    EXPECT_NE(CH_SCAN_TERMINATED, err);

    // teardown
    err = ch_free_scratch(scratch);
    ASSERT_EQ(CH_SUCCESS, err);
    ch_free_database(db);
}

// ch_scan: Call with a database with broken magic
TEST(HybridArgChecks, ScanBlockBrokenDatabaseMagic) {
    ch_database_t *db = nullptr;
    makeDatabase(&db);
    ch_scratch_t *scratch = nullptr;
    makeScratch(db, &scratch);

    // break the database here, after scratch alloc
    breakDatabaseMagic(db);

    ch_error_t err = ch_scan(db, "data", 4, 0, scratch,
                             dummyHandler, nullptr, nullptr);
    ASSERT_EQ(CH_INVALID, err);

    // teardown
    err = ch_free_scratch(scratch);
    ASSERT_EQ(CH_SUCCESS, err);
    free(db);
}

// ch_scan: Call with a database with broken version
TEST(HybridArgChecks, ScanBlockBrokenDatabaseVersion) {
    ch_database_t *db = nullptr;
    makeDatabase(&db);
    ch_scratch_t *scratch = nullptr;
    makeScratch(db, &scratch);

    // break the database here, after scratch alloc
    breakDatabaseVersion(db);

    ch_error_t err = ch_scan(db, "data", 4, 0, scratch,
                             dummyHandler, nullptr, nullptr);
    ASSERT_EQ(CH_DB_VERSION_ERROR, err);

    // teardown
    err = ch_free_scratch(scratch);
    ASSERT_EQ(CH_SUCCESS, err);
    ch_free_database(db);
}

// ch_scan: Call with no data
TEST(HybridArgChecks, ScanBlockNoData) {
    ch_database_t *db = nullptr;
    makeDatabase(&db);
    ch_scratch_t *scratch = nullptr;
    makeScratch(db, &scratch);

    ch_error_t err = ch_scan(db, nullptr, 4, 0, scratch, dummyHandler,
                             nullptr, nullptr);
    ASSERT_NE(CH_SUCCESS, err);
    EXPECT_NE(CH_SCAN_TERMINATED, err);

    // teardown
    err = ch_free_scratch(scratch);
    ASSERT_EQ(CH_SUCCESS, err);
    ch_free_database(db);
}

// ch_scan: Call with no scratch
TEST(HybridArgChecks, ScanBlockNoScratch) {
    ch_database_t *db = nullptr;
    makeDatabase(&db);

    ch_error_t err = ch_scan(db, "data", 4, 0, nullptr, dummyHandler,
                             nullptr, nullptr);
    ASSERT_NE(CH_SUCCESS, err);
    EXPECT_NE(CH_SCAN_TERMINATED, err);

    // teardown
    ch_free_database(db);
}

// ch_scan: Call with no event handler
TEST(HybridArgChecks, ScanBlockNoHandler) {
    ch_database_t *db = nullptr;
    makeDatabase(&db);
    ch_scratch_t *scratch = nullptr;
    makeScratch(db, &scratch);

    ch_error_t err = ch_scan(db, "data", 4, 0, scratch, nullptr, nullptr,
                             nullptr);
    ASSERT_EQ(CH_SUCCESS, err);
    EXPECT_NE(CH_SCAN_TERMINATED, err);

    // teardown
    err = ch_free_scratch(scratch);
    ASSERT_EQ(CH_SUCCESS, err);
    ch_free_database(db);
}

// ch_alloc_scratch: Call with no database
TEST(HybridArgChecks, AllocScratchNoDatabase) {
    ch_scratch_t *scratch = nullptr;
    ch_error_t err = ch_alloc_scratch(nullptr, &scratch);
    EXPECT_NE(CH_SUCCESS, err);
    EXPECT_TRUE(scratch == nullptr);
}

// ch_alloc_scratch: Call with nullptr ptr-to-scratch
TEST(HybridArgChecks, AllocScratchNullScratchPtr) {
    ch_database_t *db = nullptr;
    makeDatabase(&db);

    ch_error_t err = ch_alloc_scratch(db, nullptr);
    ASSERT_EQ(CH_INVALID, err);

    // teardown
    ch_free_database(db);
}

// ch_alloc_scratch: Call with bogus scratch
TEST(HybridArgChecks, AllocScratchBogusScratch) {
    ch_database_t *db = nullptr;
    makeDatabase(&db);

    ch_scratch_t *blah = (ch_scratch_t *)malloc(100);
    memset(blah, 0xf0, 100);
    ch_error_t err = ch_alloc_scratch(db, &blah);
    ASSERT_EQ(CH_INVALID, err);

    // teardown
    free(blah);
    ch_free_database(db);
}

// ch_alloc_scratch: Call with broken database magic
TEST(HybridArgChecks, AllocScratchBadDatabaseMagic) {
    ch_database_t *db = nullptr;
    makeDatabase(&db);

    breakDatabaseMagic(db);

    ch_scratch_t *scratch = nullptr;
    ch_error_t err = ch_alloc_scratch(db, &scratch);
    ASSERT_EQ(CH_INVALID, err);

    // teardown
    free(db);
}

// ch_alloc_scratch: Call with broken database version
TEST(HybridArgChecks, AllocScratchBadDatabaseVersion) {
    ch_database_t *db = nullptr;
    makeDatabase(&db);

    breakDatabaseVersion(db);

    ch_scratch_t *scratch = nullptr;
    ch_error_t err = ch_alloc_scratch(db, &scratch);
    ASSERT_EQ(CH_DB_VERSION_ERROR, err);

    // teardown
    ch_free_database(db);
}

// ch_clone_scratch: Call with no source scratch
TEST(HybridArgChecks, CloneScratchNoSource) {
    ch_scratch_t *scratch = nullptr, *scratch2 = nullptr;
    ch_error_t err = ch_clone_scratch(scratch, &scratch2);
    EXPECT_NE(CH_SUCCESS, err);
    EXPECT_TRUE(scratch2 == nullptr);
}

// ch_database_size: Call with no database
TEST(HybridArgChecks, DatabaseSizeNoDatabase) {
    size_t sz = 0;
    ch_error_t err = ch_database_size(0, &sz);
    ASSERT_EQ(CH_INVALID, err);
    ASSERT_EQ(0U, sz);
}

// ch_clone_scratch: bad scratch arg
TEST(HybridArgChecks, CloneBadScratch) {
    // Try cloning the scratch
    void *local_garbage = malloc(sizeof(garbage));
    memcpy(local_garbage, garbage, sizeof(garbage));
    ch_scratch_t *cloned = nullptr;
    ch_scratch_t *scratch = (ch_scratch_t *)local_garbage;
    ch_error_t err = ch_clone_scratch(scratch, &cloned);
    free(local_garbage);
    ASSERT_EQ(CH_INVALID, err);
}

// ch_scan: bad scratch arg
TEST(HybridArgChecks, ScanBadScratch) {
    ch_database_t *db = nullptr;
    makeDatabase(&db);

    void *local_garbage = malloc(sizeof(garbage));
    memcpy(local_garbage, garbage, sizeof(garbage));

    ch_scratch_t *scratch = (ch_scratch_t *)local_garbage;
    ch_error_t err = ch_scan(db, "data", 4, 0, scratch,
                             dummyHandler, nullptr, nullptr);
    free(local_garbage);
    ASSERT_EQ(CH_INVALID, err);

    // teardown
    ch_free_database(db);
}

TEST(HybridArgChecks, ch_free_database_null) {
    ch_error_t err = ch_free_database(nullptr);
    ASSERT_EQ(CH_SUCCESS, err);
}

TEST(HybridArgChecks, ch_free_database_garbage) {
    ch_error_t err = ch_free_database((ch_database_t *)garbage);
    ASSERT_EQ(CH_INVALID, err);
}

TEST(HybridArgChecks, ch_free_scratch_null) {
    ch_error_t err = ch_free_scratch(nullptr);
    ASSERT_EQ(CH_SUCCESS, err);
}

TEST(HybridArgChecks, ch_free_scratch_garbage) {
    ch_error_t err = ch_free_scratch((ch_scratch_t *)garbage);
    ASSERT_EQ(CH_INVALID, err);
}

TEST(HybridArgChecks, ch_free_compile_error_null) {
    ch_error_t err = ch_free_compile_error(nullptr);
    ASSERT_EQ(CH_SUCCESS, err);
}

} // namespace

