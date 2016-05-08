/*
 * Copyright (c) 2016, Intel Corporation
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

#include "hs.h"
#include "gtest/gtest.h"

#include <memory>

using namespace std;

struct RescanContext {
    RescanContext(const hs_database_t *db_in, hs_scratch_t *scratch_in)
        : db(db_in), scratch(scratch_in) {}
    const hs_database_t *db;
    hs_scratch_t *scratch;
    size_t matches = 0;
};

struct HyperscanDatabaseDeleter {
    void operator()(hs_database_t *db) const {
        hs_error_t err = hs_free_database(db);
        EXPECT_EQ(HS_SUCCESS, err);
    }
};

unique_ptr<hs_database_t, HyperscanDatabaseDeleter>
makeDatabase(const char *expression, unsigned int flags, unsigned int mode) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile(expression, flags, mode, nullptr, &db,
                                &compile_err);
    EXPECT_EQ(HS_SUCCESS, err);

    return unique_ptr<hs_database_t, HyperscanDatabaseDeleter>(db);
}

// Generic block mode test that uses the given scan callback.
static
void runBlockTest(match_event_handler cb_func) {
    auto db = makeDatabase("foo.*bar", 0, HS_MODE_BLOCK);
    ASSERT_NE(nullptr, db.get());

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db.get(), &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    RescanContext rc(db.get(), scratch);
    const string data = "___foo___bar_";

    err = hs_scan(db.get(), data.c_str(), data.length(), 0, scratch,
                  cb_func, &rc);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(1, rc.matches);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
}

// Generic streaming mode test that uses the given scan callback.
static
void runStreamingTest(match_event_handler cb_func) {
    auto db = makeDatabase("foo.*bar", 0, HS_MODE_STREAM);
    ASSERT_NE(nullptr, db.get());

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db.get(), &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    hs_stream_t *stream = nullptr;
    err = hs_open_stream(db.get(), 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    RescanContext rc(db.get(), scratch);
    const string data = "___foo___bar_";

    err = hs_scan_stream(stream, data.c_str(), data.length(), 0, scratch,
                         cb_func, &rc);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(1, rc.matches);

    // teardown
    hs_close_stream(stream, scratch, nullptr, nullptr);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
}

// Generic vectored mode test that uses the given scan callback.
static
void runVectoredTest(match_event_handler cb_func) {
    auto db = makeDatabase("foo.*bar", 0, HS_MODE_VECTORED);
    ASSERT_NE(nullptr, db.get());

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db.get(), &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    RescanContext rc(db.get(), scratch);
    const string data1 = "___foo_";
    const string data2 = "bar_";

    const char *vec[] = {data1.c_str(), data2.c_str()};
    const unsigned int len[] = {unsigned(data1.length()),
                                unsigned(data2.length())};

    err = hs_scan_vector(db.get(), vec, len, 2, 0, scratch, cb_func, &rc);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(1, rc.matches);

    // teardown
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
}

static
int rescan_block_cb(unsigned, unsigned long long, unsigned long long, unsigned,
                    void *ctx) {
    RescanContext *rctx = (RescanContext *)ctx;
    rctx->matches++;

    const string data = "___foo___bar_";

    hs_error_t err = hs_scan(rctx->db, data.c_str(), data.length(), 0,
                             rctx->scratch, dummy_cb, nullptr);
    EXPECT_EQ(HS_SCRATCH_IN_USE, err);
    return 0;
}


// Attempt to use in-use scratch inside block mode callback.
TEST(ScratchInUse, Block) {
    runBlockTest(rescan_block_cb);
}

static
int rescan_stream_cb(unsigned, unsigned long long, unsigned long long, unsigned,
                     void *ctx) {
    RescanContext *rctx = (RescanContext *)ctx;
    rctx->matches++;

    const string data = "___foo___bar_";

    hs_stream_t *stream = nullptr;
    hs_error_t err = hs_open_stream(rctx->db, 0, &stream);
    EXPECT_EQ(HS_SUCCESS, err);
    EXPECT_TRUE(stream != nullptr);
    if (stream == nullptr) {
        return 1;
    }

    err = hs_scan_stream(stream, data.c_str(), data.length(), 0,
                         rctx->scratch, dummy_cb, nullptr);
    EXPECT_EQ(HS_SCRATCH_IN_USE, err);

    hs_close_stream(stream, nullptr, nullptr, nullptr);
    return 0;
}

// Attempt to use in-use scratch inside streaming mode callback.
TEST(ScratchInUse, Streaming) {
    runStreamingTest(rescan_stream_cb);
}

static
int rescan_vector_cb(unsigned, unsigned long long, unsigned long long, unsigned,
                    void *ctx) {
    RescanContext *rctx = (RescanContext *)ctx;
    rctx->matches++;

    const string data1 = "___foo_";
    const string data2 = "bar_";

    const char *vec[] = {data1.c_str(), data2.c_str()};
    const unsigned int len[] = {unsigned(data1.length()),
                                unsigned(data2.length())};

    hs_error_t err = hs_scan_vector(rctx->db, vec, len, 2, 0, rctx->scratch,
                                    dummy_cb, nullptr);
    EXPECT_EQ(HS_SCRATCH_IN_USE, err);
    return 0;
}

// Attempt to use in-use scratch inside vectored mode callback.
TEST(ScratchInUse, Vectored) {
    runVectoredTest(rescan_vector_cb);
}

static
int rescan_realloc_cb(unsigned, unsigned long long, unsigned long long,
                      unsigned, void *ctx) {
    RescanContext *rctx = (RescanContext *)ctx;
    rctx->matches++;

    auto db = makeDatabase("another db", 0, HS_MODE_BLOCK);
    hs_error_t err = hs_alloc_scratch(db.get(), &rctx->scratch);
    EXPECT_EQ(HS_SCRATCH_IN_USE, err);
    return 0;
}

// Attempt to use hs_alloc_scratch on in-use scratch inside callback (block
// scan).
TEST(ScratchInUse, ReallocScratchBlock) {
    runBlockTest(rescan_realloc_cb);
}

// Attempt to use hs_alloc_scratch on in-use scratch inside callback (streaming
// scan).
TEST(ScratchInUse, ReallocScratchStreaming) {
    runStreamingTest(rescan_realloc_cb);
}

// Attempt to use hs_alloc_scratch on in-use scratch inside callback (vectored
// scan).
TEST(ScratchInUse, ReallocScratchVector) {
    runVectoredTest(rescan_realloc_cb);
}

static
int rescan_free_cb(unsigned, unsigned long long, unsigned long long,
                      unsigned, void *ctx) {
    RescanContext *rctx = (RescanContext *)ctx;
    rctx->matches++;

    hs_error_t err = hs_free_scratch(rctx->scratch);
    EXPECT_EQ(HS_SCRATCH_IN_USE, err);
    return 0;
}

// Attempt to use hs_free_scratch on in-use scratch inside callback (block
// scan).
TEST(ScratchInUse, FreeScratchBlock) {
    runBlockTest(rescan_free_cb);
}

// Attempt to use hs_free_scratch on in-use scratch inside callback (streaming
// scan).
TEST(ScratchInUse, FreeScratchStreaming) {
    runStreamingTest(rescan_free_cb);
}

// Attempt to use hs_free_scratch on in-use scratch inside callback (vectored
// scan).
TEST(ScratchInUse, FreeScratchVector) {
    runVectoredTest(rescan_free_cb);
}
