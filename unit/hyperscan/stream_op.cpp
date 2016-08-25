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

#include <vector>

#include "gtest/gtest.h"
#include "hs.h"
#include "test_util.h"

using namespace std;

namespace /* anonymous */ {

int record_cb2(unsigned id, unsigned long long, unsigned long long to,
                 unsigned, void *ctxt) {
    CallBackContext *c = (CallBackContext *)ctxt;

    c->matches.push_back(MatchRecord(to + 1000, id));

    return (int)c->halt;
}

static const char data1[] = "barfoobar";

TEST(StreamUtil, reset1) {
    hs_error_t err;
    hs_scratch_t *scratch = nullptr;
    hs_database_t *db = buildDBAndScratch("foo.*bar", 0, 0, HS_MODE_STREAM,
                                          &scratch);

    hs_stream_t *stream = nullptr;

    CallBackContext c, c2;

    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    err = hs_scan_stream(stream, data1, sizeof(data1), 0, scratch, record_cb,
                         (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(1U, c.matches.size());
    ASSERT_EQ(MatchRecord(9, 0), c.matches[0]);

    c.matches.clear();

    // Note: we do not need matches from this reset operation, so we do not
    // need to supply a callback or scratch space.
    err = hs_reset_stream(stream, 0, nullptr, nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_scan_stream(stream, data1, sizeof(data1), 0, scratch, record_cb2,
                         (void *)&c2);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(0U, c.matches.size());
    ASSERT_EQ(1U, c2.matches.size());
    ASSERT_EQ(MatchRecord(1009, 0), c2.matches[0]);

    hs_close_stream(stream, scratch, nullptr, nullptr);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(StreamUtil, reset2) {
    hs_error_t err;
    hs_scratch_t *scratch = nullptr;
    hs_database_t *db = buildDBAndScratch("foo.*bar", 0, 0, HS_MODE_STREAM,
                                          &scratch);

    hs_stream_t *stream = nullptr;

    CallBackContext c, c2;

    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    c.halt = 1;
    err = hs_scan_stream(stream, data1, sizeof(data1), 0, scratch, record_cb,
                         (void *)&c);
    ASSERT_EQ(HS_SCAN_TERMINATED, err);
    ASSERT_EQ(1U, c.matches.size());
    ASSERT_EQ(MatchRecord(9, 0), c.matches[0]);

    c.matches.clear();

    // Note: we do not need matches from this reset operation, so we do not
    // need to supply a callback or scratch space.
    err = hs_reset_stream(stream, 0, nullptr, nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_scan_stream(stream, data1, sizeof(data1), 0, scratch, record_cb,
                         (void *)&c2);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(0U, c.matches.size());
    ASSERT_EQ(1U, c2.matches.size());
    ASSERT_EQ(MatchRecord(9, 0), c2.matches[0]);

    hs_close_stream(stream, scratch, nullptr, nullptr);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(StreamUtil, reset_matches) {
    hs_error_t err;
    hs_scratch_t *scratch = nullptr;
    hs_database_t *db = buildDBAndScratch("foo.*bar$", 0, 0, HS_MODE_STREAM,
                                          &scratch);

    hs_stream_t *stream = nullptr;
    CallBackContext c;

    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    err = hs_scan_stream(stream, data1, strlen(data1), 0, scratch, record_cb,
                         (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(0U, c.matches.size());

    err = hs_reset_stream(stream, 0, scratch, record_cb, (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(1U, c.matches.size());
    ASSERT_EQ(MatchRecord(9, 0), c.matches[0]);

    hs_close_stream(stream, scratch, nullptr, nullptr);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(StreamUtil, reset_close) {
    hs_error_t err;
    hs_scratch_t *scratch = nullptr;
    hs_database_t *db = buildDBAndScratch("foo", 0, 0, HS_MODE_STREAM,
                                          &scratch);

    hs_stream_t *stream = nullptr;
    CallBackContext c;

    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    // break matching string over two stream writes
    const char part1[] = "---f";
    err = hs_scan_stream(stream, part1, strlen(part1), 0, scratch, record_cb,
                         (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(0U, c.matches.size());

    const char part2[] = "oo--";
    err = hs_scan_stream(stream, part2, strlen(part2), 0, scratch, record_cb,
                         (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(1U, c.matches.size());
    ASSERT_EQ(MatchRecord(6, 0), c.matches[0]);

    err = hs_reset_stream(stream, 0, scratch, record_cb, (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    // still only one match
    ASSERT_EQ(1U, c.matches.size());

    err = hs_close_stream(stream, scratch, record_cb, (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(StreamUtil, copy1) {
    hs_error_t err;
    hs_scratch_t *scratch = nullptr;
    hs_database_t *db = buildDBAndScratch("foo.*bar", 0, 0, HS_MODE_STREAM,
                                          &scratch);

    hs_stream_t *stream = nullptr;
    hs_stream_t *stream2 = nullptr;

    CallBackContext c;

    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    err = hs_scan_stream(stream, data1, sizeof(data1), 0, scratch, record_cb,
                         (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(1U, c.matches.size());
    ASSERT_EQ(MatchRecord(9, 0), c.matches[0]);

    c.matches.clear();

    err = hs_copy_stream(&stream2, stream);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_scan_stream(stream, data1, sizeof(data1), 0, scratch, record_cb,
                         (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(2U, c.matches.size());
    ASSERT_EQ(MatchRecord(13, 0), c.matches[0]);
    ASSERT_EQ(MatchRecord(19, 0), c.matches[1]);

    c.matches.clear();

    err = hs_scan_stream(stream2, data1, sizeof(data1), 0, scratch, record_cb,
                         (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(2U, c.matches.size());
    ASSERT_EQ(MatchRecord(13, 0), c.matches[0]);
    ASSERT_EQ(MatchRecord(19, 0), c.matches[1]);

    hs_close_stream(stream, scratch, nullptr, nullptr);
    hs_close_stream(stream2, scratch, nullptr, nullptr);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(StreamUtil, copy2) {
    hs_error_t err;
    hs_scratch_t *scratch = nullptr;
    hs_database_t *db = buildDBAndScratch("foo.*bar", 0, 0, HS_MODE_STREAM,
                                          &scratch);


    hs_stream_t *stream = nullptr;
    hs_stream_t *stream2 = nullptr;
    CallBackContext c;

    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    c.halt = 1;
    err = hs_scan_stream(stream, data1, sizeof(data1), 0, scratch, record_cb,
                         (void *)&c);
    ASSERT_EQ(HS_SCAN_TERMINATED, err);
    ASSERT_EQ(1U, c.matches.size());
    ASSERT_EQ(MatchRecord(9, 0), c.matches[0]);

    c.matches.clear();

    err = hs_copy_stream(&stream2, stream);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_scan_stream(stream, data1, sizeof(data1), 0, scratch, record_cb,
                         (void *)&c);
    ASSERT_EQ(HS_SCAN_TERMINATED, err);
    ASSERT_EQ(0U, c.matches.size());

    err = hs_scan_stream(stream2, data1, sizeof(data1), 0, scratch, record_cb,
                         (void *)&c);
    ASSERT_EQ(HS_SCAN_TERMINATED, err);
    ASSERT_EQ(0U, c.matches.size());

    hs_close_stream(stream, scratch, nullptr, nullptr);
    hs_close_stream(stream2, scratch, nullptr, nullptr);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(StreamUtil, copy_reset1) {
    hs_error_t err;
    hs_scratch_t *scratch = nullptr;
    hs_database_t *db = buildDBAndScratch("foo.*bar", 0, 0, HS_MODE_STREAM,
                                          &scratch);

    hs_stream_t *stream = nullptr;
    hs_stream_t *stream2 = nullptr;

    CallBackContext c, c2;

    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    err = hs_open_stream(db, 0, &stream2);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream2 != nullptr);

    err = hs_scan_stream(stream, data1, sizeof(data1), 0, scratch, record_cb,
                         (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(1U, c.matches.size());
    ASSERT_EQ(MatchRecord(9, 0), c.matches[0]);

    c.matches.clear();

    err = hs_reset_and_copy_stream(stream, stream2, nullptr, nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_scan_stream(stream, data1, sizeof(data1), 0, scratch, record_cb2,
                         (void *)&c2);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(0U, c.matches.size());
    ASSERT_EQ(1U, c2.matches.size());
    ASSERT_EQ(MatchRecord(1009, 0), c2.matches[0]);

    hs_close_stream(stream, scratch, nullptr, nullptr);
    hs_close_stream(stream2, scratch, nullptr, nullptr);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(StreamUtil, copy_reset2) {
    hs_error_t err;
    hs_scratch_t *scratch = nullptr;
    hs_database_t *db = buildDBAndScratch("foo.*bar", 0, 0, HS_MODE_STREAM,
                                          &scratch);

    hs_stream_t *stream = nullptr;
    hs_stream_t *stream2 = nullptr;

    CallBackContext c, c2;

    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    err = hs_open_stream(db, 0, &stream2);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream2 != nullptr);

    c.halt = 1;
    err = hs_scan_stream(stream, data1, sizeof(data1), 0, scratch, record_cb,
                         (void *)&c);
    ASSERT_EQ(HS_SCAN_TERMINATED, err);
    ASSERT_EQ(1U, c.matches.size());
    ASSERT_EQ(MatchRecord(9, 0), c.matches[0]);

    c.matches.clear();

    err = hs_reset_and_copy_stream(stream, stream2, nullptr, nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_scan_stream(stream, data1, sizeof(data1), 0, scratch, record_cb,
                         (void *)&c2);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(0U, c.matches.size());
    ASSERT_EQ(1U, c2.matches.size());
    ASSERT_EQ(MatchRecord(9, 0), c2.matches[0]);

    hs_close_stream(stream, scratch, nullptr, nullptr);
    hs_close_stream(stream2, scratch, nullptr, nullptr);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(StreamUtil, copy_reset3) {
    hs_error_t err;
    hs_scratch_t *scratch = nullptr;
    hs_database_t *db = buildDBAndScratch("foo.*bar", 0, 0, HS_MODE_STREAM,
                                          &scratch);

    hs_stream_t *stream = nullptr;
    hs_stream_t *stream2 = nullptr;

    CallBackContext c;

    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    err = hs_open_stream(db, 0, &stream2);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    err = hs_scan_stream(stream, data1, sizeof(data1), 0, scratch, record_cb,
                         (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(1U, c.matches.size());
    ASSERT_EQ(MatchRecord(9, 0), c.matches[0]);

    c.matches.clear();

    err = hs_reset_and_copy_stream(stream2, stream, nullptr, nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_scan_stream(stream, data1, sizeof(data1), 0, scratch, record_cb,
                         (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(2U, c.matches.size());
    ASSERT_EQ(MatchRecord(13, 0), c.matches[0]);
    ASSERT_EQ(MatchRecord(19, 0), c.matches[1]);

    c.matches.clear();

    err = hs_scan_stream(stream2, data1, sizeof(data1), 0, scratch, record_cb,
                         (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(2U, c.matches.size());
    ASSERT_EQ(MatchRecord(13, 0), c.matches[0]);
    ASSERT_EQ(MatchRecord(19, 0), c.matches[1]);

    hs_close_stream(stream, scratch, nullptr, nullptr);
    hs_close_stream(stream2, scratch, nullptr, nullptr);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(StreamUtil, copy_reset4) {
    hs_error_t err;
    hs_scratch_t *scratch = nullptr;
    hs_database_t *db = buildDBAndScratch("foo.*bar", 0, 0, HS_MODE_STREAM,
                                          &scratch);

    hs_stream_t *stream = nullptr;
    hs_stream_t *stream2 = nullptr;

    CallBackContext c;

    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    err = hs_open_stream(db, 0, &stream2);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream2 != nullptr);

    c.halt = 1;
    err = hs_scan_stream(stream, data1, sizeof(data1), 0, scratch, record_cb,
                         (void *)&c);
    ASSERT_EQ(HS_SCAN_TERMINATED, err);
    ASSERT_EQ(1U, c.matches.size());
    ASSERT_EQ(MatchRecord(9, 0), c.matches[0]);

    c.matches.clear();

    err = hs_reset_and_copy_stream(stream2, stream, nullptr, nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_scan_stream(stream, data1, sizeof(data1), 0, scratch, record_cb,
                         (void *)&c);
    ASSERT_EQ(HS_SCAN_TERMINATED, err);
    ASSERT_EQ(0U, c.matches.size());

    err = hs_scan_stream(stream2, data1, sizeof(data1), 0, scratch, record_cb,
                         (void *)&c);
    ASSERT_EQ(HS_SCAN_TERMINATED, err);
    ASSERT_EQ(0U, c.matches.size());

    hs_close_stream(stream, scratch, nullptr, nullptr);
    hs_close_stream(stream2, scratch, nullptr, nullptr);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(StreamUtil, copy_reset5) {
    hs_error_t err;
    hs_scratch_t *scratch = nullptr;
    hs_database_t *db = buildDBAndScratch("foo.*bar", 0, 0, HS_MODE_STREAM,
                                          &scratch);

    hs_stream_t *stream = nullptr;
    hs_stream_t *stream2 = nullptr;

    CallBackContext c;

    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    err = hs_open_stream(db, 0, &stream2);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    err = hs_scan_stream(stream, data1, sizeof(data1), 0, scratch, record_cb,
                         (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(1U, c.matches.size());
    ASSERT_EQ(MatchRecord(9, 0), c.matches[0]);

    c.matches.clear();

    err = hs_scan_stream(stream2, "foo", 3, 0, scratch, record_cb, (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(0U, c.matches.size());

    err = hs_reset_and_copy_stream(stream2, stream, nullptr, nullptr, nullptr);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_scan_stream(stream, data1, sizeof(data1), 0, scratch, record_cb,
                         (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(2U, c.matches.size());
    ASSERT_EQ(MatchRecord(13, 0), c.matches[0]);
    ASSERT_EQ(MatchRecord(19, 0), c.matches[1]);

    c.matches.clear();

    err = hs_scan_stream(stream2, data1, sizeof(data1), 0, scratch, record_cb,
                         (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(2U, c.matches.size());
    ASSERT_EQ(MatchRecord(13, 0), c.matches[0]);
    ASSERT_EQ(MatchRecord(19, 0), c.matches[1]);

    hs_close_stream(stream, scratch, nullptr, nullptr);
    hs_close_stream(stream2, scratch, nullptr, nullptr);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

TEST(StreamUtil, copy_reset_matches) {
    hs_error_t err;
    hs_scratch_t *scratch = nullptr;
    hs_database_t *db = buildDBAndScratch("foo.*bar$", 0, 0, HS_MODE_STREAM,
                                          &scratch);

    hs_stream_t *stream = nullptr;
    hs_stream_t *stream2 = nullptr;

    CallBackContext c;

    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);

    err = hs_open_stream(db, 0, &stream2);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream2 != nullptr);

    err = hs_scan_stream(stream, data1, strlen(data1), 0, scratch, record_cb,
                         (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(0U, c.matches.size());

    err = hs_reset_and_copy_stream(stream, stream2, scratch, record_cb,
                                   (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(1U, c.matches.size());
    ASSERT_EQ(MatchRecord(9, 0), c.matches[0]);

    hs_close_stream(stream, scratch, nullptr, nullptr);
    hs_close_stream(stream2, scratch, nullptr, nullptr);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

static size_t last_alloc;

static
void *wrap_m(size_t s) {
    last_alloc = s;
    return malloc(s);
}

TEST(StreamUtil, size) {
    hs_error_t err;
    hs_scratch_t *scratch = nullptr;
    hs_database_t *db = buildDBAndScratch("foo.*bar", 0, 0, HS_MODE_STREAM,
                                          &scratch);

    hs_stream_t *stream = nullptr;

    CallBackContext c, c2;

    hs_set_allocator(wrap_m, nullptr);
    last_alloc = 0;
    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);

    size_t stream_size;
    err = hs_stream_size(db, &stream_size);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(last_alloc, stream_size);

    hs_set_allocator(nullptr, nullptr);

    hs_close_stream(stream, scratch, nullptr, nullptr);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
}

int alloc_called = 0;
int alloc2_called = 0;
int alloc3_called = 0;

void *bad_alloc(size_t) {
    return nullptr;
}

void *my_alloc(size_t s) {
    ++alloc_called;
    return malloc(s);
}

void my_free(void *p) {
    free(p);
    --alloc_called;
}

void *my_alloc2(size_t s) {
    ++alloc2_called;
    return malloc(s);
}

void my_free2(void *p) {
    free(p);
    --alloc2_called;
}

void *my_alloc3(size_t s) {
    ++alloc3_called;
    return malloc(s);
}

void my_free3(void *p) {
    free(p);
    --alloc3_called;
}

TEST(StreamUtil, Alloc) {
    hs_compile_error_t *compile_err = nullptr;
    hs_database_t *db;

    hs_set_stream_allocator(my_alloc, my_free);
    alloc_called = 0;

    hs_error_t err = hs_compile("foo.*bar", 0, HS_MODE_STREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);

    ASSERT_EQ(HS_SUCCESS, err);

    hs_stream_t *stream = nullptr;

    CallBackContext c;

    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);
    ASSERT_NE(alloc_called, 0);

    err = hs_scan_stream(stream, data1, sizeof(data1), 0, scratch, record_cb,
                         (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(1U, c.matches.size());
    ASSERT_EQ(MatchRecord(9, 0), c.matches[0]);

    hs_close_stream(stream, scratch, record_cb, (void *)&c);
    ASSERT_EQ(alloc_called, 0);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
    hs_free_compile_error(compile_err);
    hs_set_allocator(nullptr, nullptr);
}

TEST(StreamUtil, MoreAlloc) {
    hs_compile_error_t *compile_err = nullptr;
    hs_database_t *db;

    hs_set_allocator(my_alloc, my_free);
    alloc_called = 0;

    hs_error_t err = hs_compile("foo.*bar", 0, HS_MODE_STREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(1, alloc_called);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(2, alloc_called);

    ASSERT_EQ(HS_SUCCESS, err);

    hs_stream_t *stream = nullptr;

    CallBackContext c;

    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(stream != nullptr);
    ASSERT_EQ(alloc_called, 3);

    err = hs_scan_stream(stream, data1, sizeof(data1), 0, scratch, record_cb,
                         (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(1U, c.matches.size());
    ASSERT_EQ(MatchRecord(9, 0), c.matches[0]);

    hs_close_stream(stream, scratch, nullptr, nullptr);
    ASSERT_EQ(alloc_called, 2);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(alloc_called, 1);
    hs_free_database(db);
    ASSERT_EQ(alloc_called, 0);
    hs_free_compile_error(compile_err);
    hs_set_allocator(nullptr, nullptr);
}

TEST(StreamUtil, BadStreamAlloc) {
    hs_compile_error_t *compile_err = nullptr;
    hs_database_t *db;

    hs_set_stream_allocator(bad_alloc, free);
    alloc_called = 0;

    hs_error_t err = hs_compile("foo.*bar", 0, HS_MODE_STREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);

    ASSERT_EQ(HS_SUCCESS, err);

    hs_stream_t *stream = nullptr;

    CallBackContext c;

    // should go boom
    err = hs_open_stream(db, 0, &stream);
    ASSERT_NE(HS_SUCCESS, err);

    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
    hs_free_compile_error(compile_err);
    hs_set_stream_allocator(nullptr, nullptr);
}

TEST(StreamUtil, StreamAllocUsage) {
    hs_compile_error_t *compile_err = nullptr;
    hs_database_t *db;

    hs_set_allocator(my_alloc, my_free);
    hs_set_stream_allocator(my_alloc2, my_free2);
    hs_set_scratch_allocator(my_alloc3, my_free3);
    alloc_called  = 0;
    alloc2_called = 0;
    alloc3_called = 0;

    hs_error_t err = hs_compile("foo.*bar", 0, HS_MODE_STREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);

    ASSERT_EQ(HS_SUCCESS, err);

    hs_stream_t *stream = nullptr;
    hs_stream_t *stream2 = nullptr;
    hs_stream_t *stream3 = nullptr;

    CallBackContext c;
    ASSERT_EQ(1, alloc_called);
    ASSERT_EQ(0, alloc2_called);
    ASSERT_EQ(1, alloc3_called);

    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(1, alloc_called);
    ASSERT_EQ(1, alloc2_called);
    ASSERT_EQ(1, alloc3_called);

    err = hs_copy_stream(&stream2, stream);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(1, alloc_called);
    ASSERT_EQ(2, alloc2_called);
    ASSERT_EQ(1, alloc3_called);

    err = hs_open_stream(db, 0, &stream3);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(1, alloc_called);
    ASSERT_EQ(3, alloc2_called);
    ASSERT_EQ(1, alloc3_called);

    hs_close_stream(stream, scratch, nullptr, nullptr);
    hs_close_stream(stream2, scratch, nullptr, nullptr);
    hs_close_stream(stream3, scratch, nullptr, nullptr);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
    hs_free_compile_error(compile_err);
    hs_set_allocator(nullptr, nullptr);

    ASSERT_EQ(0, alloc_called);
    ASSERT_EQ(0, alloc2_called);
    ASSERT_EQ(0, alloc3_called);
}

}
