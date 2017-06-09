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

/**
 * Unit tests for serialization functions.
 */
#include "config.h"

#include "gtest/gtest.h"
#include "hs.h"
#include "hs_internal.h"
#include "test_util.h"

#include <cstring>
#include <string>
#include <vector>

namespace {

using namespace std;
using namespace testing;

static const unsigned validModes[] = {
    HS_MODE_NOSTREAM,
    HS_MODE_STREAM | HS_MODE_SOM_HORIZON_LARGE,
    HS_MODE_VECTORED
};

static const pattern testPatterns[] = {
    pattern("hatstand.*teakettle.*badgerbrush", HS_FLAG_CASELESS, 1000),
    pattern("hatstand.*teakettle.*badgerbrush", HS_FLAG_DOTALL, 1001),
    pattern("hatstand|teakettle|badgerbrush", 0, 1002),
    pattern("^hatstand|teakettle|badgerbrush$", 0, 1003),
    pattern("foobar.{10,1000}xyzzy", HS_FLAG_DOTALL, 1004),
    pattern("foobar.{2,501}roobar", 0, 1005),
    pattern("abc.*def.*ghi", HS_FLAG_SOM_LEFTMOST, 1006),
    pattern("(\\p{L}){4}", HS_FLAG_UTF8|HS_FLAG_UCP, 1007),
    pattern("\\.(exe|pdf|gif|jpg|png|wav|riff|mp4)\\z", 0, 1008)
};

class SerializeP : public TestWithParam<tuple<unsigned, pattern>> {};

static
const char *getModeString(unsigned mode) {
    if (mode & HS_MODE_STREAM) {
        return "STREAM";
    }
    if (mode & HS_MODE_BLOCK) {
        return "BLOCK";
    }
    if (mode & HS_MODE_VECTORED) {
        return "VECTORED";
    }
    return "UNKNOWN";
}

// Check that we can deserialize from a char array at any alignment and the info
// is consistent
TEST_P(SerializeP, DeserializeFromAnyAlignment) {
    const unsigned mode = get<0>(GetParam());
    const pattern &pat = get<1>(GetParam());
    SCOPED_TRACE(mode);
    SCOPED_TRACE(pat);

    hs_error_t err;
    hs_database_t *db = buildDB(pat, mode);
    ASSERT_TRUE(db != nullptr) << "database build failed.";

    char *original_info = nullptr;
    err = hs_database_info(db, &original_info);
    ASSERT_EQ(HS_SUCCESS, err);

    const char *mode_string = getModeString(mode);

    ASSERT_NE(nullptr, original_info)
        << "hs_serialized_database_info returned null.";
    ASSERT_STREQ("Version:", string(original_info).substr(0, 8).c_str());
    ASSERT_TRUE(strstr(original_info, mode_string) != nullptr)
        << "Original info \"" << original_info
        << "\" does not contain " << mode_string;

    char *bytes = nullptr;
    size_t length = 0;
    err = hs_serialize_database(db, &bytes, &length);
    ASSERT_EQ(HS_SUCCESS, err) << "serialize failed.";
    ASSERT_NE(nullptr, bytes);
    ASSERT_LT(0U, length);

    hs_free_database(db);
    db = nullptr;

    const size_t maxalign = 16;
    char *copy = new char[length + maxalign];

    // Deserialize from char arrays at a range of alignments.
    for (size_t i = 0; i < maxalign; i++) {
        SCOPED_TRACE(i);
        memset(copy, 0, length + maxalign);

        char *mycopy = copy + i;
        memcpy(mycopy, bytes, length);

        // We should be able to call hs_serialized_database_info and get back a
        // reasonable string.
        char *info;
        err = hs_serialized_database_info(mycopy, length, &info);
        ASSERT_EQ(HS_SUCCESS, err);
        ASSERT_NE(nullptr, original_info);
        ASSERT_STREQ(original_info, info);
        free(info);

        // We should be able to deserialize as well.
        err = hs_deserialize_database(mycopy, length, &db);
        ASSERT_EQ(HS_SUCCESS, err) << "deserialize failed.";
        ASSERT_TRUE(db != nullptr);

        // And the info there should match.
        err = hs_database_info(db, &info);
        ASSERT_EQ(HS_SUCCESS, err);
        ASSERT_STREQ(original_info, info);
        free(info);

        hs_free_database(db);
        db = nullptr;
    }

    free(original_info);
    free(bytes);
    delete[] copy;
}

// Check that we can deserialize_at from a char array at any alignment and the
// info is consistent
TEST_P(SerializeP, DeserializeAtFromAnyAlignment) {
    const unsigned mode = get<0>(GetParam());
    const pattern &pat = get<1>(GetParam());
    SCOPED_TRACE(mode);
    SCOPED_TRACE(pat);

    hs_error_t err;
    hs_database_t *db = buildDB(pat, mode);
    ASSERT_TRUE(db != nullptr) << "database build failed.";

    char *original_info;
    err = hs_database_info(db, &original_info);
    ASSERT_EQ(HS_SUCCESS, err);

    const char *mode_string = getModeString(mode);

    ASSERT_NE(nullptr, original_info)
        << "hs_serialized_database_info returned null.";
    ASSERT_STREQ("Version:", string(original_info).substr(0, 8).c_str());
    ASSERT_TRUE(strstr(original_info, mode_string) != nullptr)
        << "Original info \"" << original_info
        << "\" does not contain " << mode_string;

    char *bytes = nullptr;
    size_t length = 0;
    err = hs_serialize_database(db, &bytes, &length);
    ASSERT_EQ(HS_SUCCESS, err) << "serialize failed.";
    ASSERT_NE(nullptr, bytes);
    ASSERT_LT(0U, length);

    hs_free_database(db);
    db = nullptr;

    size_t slength;
    err = hs_serialized_database_size(bytes, length, &slength);
    ASSERT_EQ(HS_SUCCESS, err);

    const size_t maxalign = 16;
    char *copy = new char[length + maxalign];
    char *mem = new char[slength];
    db = (hs_database_t *)mem;

    // Deserialize from char arrays at a range of alignments.
    for (size_t i = 0; i < maxalign; i++) {
        SCOPED_TRACE(i);
        memset(copy, 0, length + maxalign);

        char *mycopy = copy + i;
        memcpy(mycopy, bytes, length);

        // We should be able to call hs_serialized_database_info and get back a
        // reasonable string.
        char *info;
        err = hs_serialized_database_info(mycopy, length, &info);
        ASSERT_EQ(HS_SUCCESS, err);
        ASSERT_NE(nullptr, original_info);
        ASSERT_STREQ(original_info, info);
        free(info);

        // Scrub target memory.
        memset(mem, 0xff, length);

        // We should be able to deserialize as well.
        err = hs_deserialize_database_at(mycopy, length, db);
        ASSERT_EQ(HS_SUCCESS, err) << "deserialize failed.";
        ASSERT_TRUE(db != nullptr);

        // And the info there should match.
        err = hs_database_info(db, &info);
        ASSERT_EQ(HS_SUCCESS, err);
        ASSERT_TRUE(info != nullptr);
        ASSERT_STREQ(original_info, info);
        free(info);
    }

    free(original_info);
    free(bytes);
    delete[] copy;
    delete[] mem;
}

INSTANTIATE_TEST_CASE_P(Serialize, SerializeP,
                        Combine(ValuesIn(validModes), ValuesIn(testPatterns)));

// Attempt to reproduce the scenario in UE-1946.
TEST(Serialize, CrossCompileSom) {
    hs_platform_info plat;
    plat.cpu_features = 0;
    plat.tune = HS_TUNE_FAMILY_GENERIC;

    static const char *pat = "hatstand.*(badgerbrush|teakettle)";
    const unsigned mode = HS_MODE_STREAM
                          | HS_MODE_SOM_HORIZON_LARGE;
    hs_database_t *db = buildDB(pat, HS_FLAG_SOM_LEFTMOST, 1000, mode, &plat);
    ASSERT_TRUE(db != nullptr) << "database build failed.";

    size_t db_len;
    hs_error_t err = hs_database_size(db, &db_len);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(0, db_len);

    char *bytes = nullptr;
    size_t bytes_len = 0;
    err = hs_serialize_database(db, &bytes, &bytes_len);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(0, bytes_len);

    hs_free_database(db);

    // Relocate to misaligned block.
    char *copy = (char *)malloc(bytes_len + 1);
    ASSERT_TRUE(copy != nullptr);
    memcpy(copy + 1, bytes, bytes_len);
    free(bytes);

    size_t ser_len;
    err = hs_serialized_database_size(copy + 1, bytes_len, &ser_len);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(0, ser_len);
    ASSERT_EQ(db_len, ser_len);

    free(copy);
}

static void *null_malloc(size_t) {
    return nullptr;
}

static void *misaligned_malloc(size_t s) {
    char *c = (char *)malloc(s + 1);
    return (void *)(c + 1);
}

static void misaligned_free(void *p) {
    char *c = (char *)p;
    free(c - 1);
}

// make sure that serializing/deserializing to null or an unaligned address
// fails
TEST(Serialize, CompileNullMalloc) {
    hs_database_t *db;
    hs_compile_error_t *c_err;
    static const char *pat = "hatstand.*(badgerbrush|teakettle)";

    // mallocing null should fail compile
    hs_set_allocator(null_malloc, nullptr);
    hs_error_t err = hs_compile(pat, 0, HS_MODE_BLOCK, nullptr, &db, &c_err);
    ASSERT_NE(HS_SUCCESS, err);
    ASSERT_TRUE(db == nullptr);
    ASSERT_TRUE(c_err != nullptr);
    hs_free_compile_error(c_err);
    hs_set_allocator(nullptr, nullptr);
}

TEST(Serialize, CompileErrorAllocator) {
    hs_database_t *db;
    hs_compile_error_t *c_err;
    static const char *pat = "hatsta^nd.*(badgerbrush|teakettle)";

    // failing to compile should use the misc allocator
    allocated_count = 0;
    allocated_count_b = 0;
    hs_set_allocator(count_malloc_b, count_free_b);
    hs_set_misc_allocator(count_malloc, count_free);
    hs_error_t err = hs_compile(pat, 0, HS_MODE_BLOCK, nullptr, &db, &c_err);
    ASSERT_NE(HS_SUCCESS, err);
    ASSERT_TRUE(db == nullptr);
    ASSERT_TRUE(c_err != nullptr);
    ASSERT_EQ(0, allocated_count_b);
    ASSERT_NE(0, allocated_count);
    hs_free_compile_error(c_err);
    hs_set_allocator(nullptr, nullptr);
    ASSERT_EQ(0, allocated_count);
}

TEST(Serialize, AllocatorsUsed) {
    hs_database_t *db;
    hs_compile_error_t *c_err;
    static const char *pat = "hatstand.*(badgerbrush|teakettle)";

    allocated_count = 0;
    allocated_count_b = 0;
    hs_set_allocator(count_malloc_b, count_free_b);
    hs_set_database_allocator(count_malloc, count_free);
    hs_error_t err = hs_compile(pat, 0, HS_MODE_BLOCK, nullptr, &db, &c_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);
    ASSERT_TRUE(c_err == nullptr);
    ASSERT_EQ(0, allocated_count_b);
    ASSERT_NE(0, allocated_count);

    /* serialize should use the misc allocator */
    char *bytes = nullptr;
    size_t bytes_len = 0;
    err = hs_serialize_database(db, &bytes, &bytes_len);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(0, bytes_len);
    ASSERT_EQ(bytes_len, allocated_count_b);

    count_free_b(bytes);

    hs_free_database(db);
    hs_set_allocator(nullptr, nullptr);
    ASSERT_EQ(0, allocated_count);
    ASSERT_EQ(0, allocated_count_b);
}

TEST(Serialize, CompileUnalignedMalloc) {
    hs_database_t *db;
    hs_compile_error_t *c_err;
    static const char *pat = "hatstand.*(badgerbrush|teakettle)";

    // unaligned malloc should fail compile
    hs_set_allocator(misaligned_malloc, misaligned_free);
    hs_error_t err = hs_compile(pat, 0, HS_MODE_BLOCK, nullptr, &db, &c_err);
    ASSERT_NE(HS_SUCCESS, err);
    ASSERT_TRUE(db == nullptr);
    ASSERT_TRUE(c_err != nullptr);
    hs_free_compile_error(c_err);
    hs_set_allocator(nullptr, nullptr);
}

TEST(Serialize, SerializeNullMalloc) {
    hs_database_t *db;
    hs_compile_error_t *c_err;
    static const char *pat = "hatstand.*(badgerbrush|teakettle)";
    hs_error_t err = hs_compile(pat, 0, HS_MODE_BLOCK, nullptr, &db, &c_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    size_t db_len;
    err = hs_database_size(db, &db_len);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(0, db_len);

    char *bytes = nullptr;
    size_t bytes_len = 0;

    // fail when serialize gets a null malloc
    hs_set_allocator(null_malloc, nullptr);
    err = hs_serialize_database(db, &bytes, &bytes_len);
    ASSERT_NE(HS_SUCCESS, err);
    hs_set_allocator(nullptr, nullptr);
    hs_free_database(db);
}

// make sure that serializing/deserializing to null or an unaligned address
// fails
TEST(Serialize, SerializeUnalignedMalloc) {
    hs_database_t *db;
    hs_compile_error_t *c_err;
    static const char *pat= "hatstand.*(badgerbrush|teakettle)";

    hs_error_t err = hs_compile(pat, 0, HS_MODE_BLOCK, nullptr, &db, &c_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    size_t db_len;
    err = hs_database_size(db, &db_len);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(0, db_len);

    char *bytes = nullptr;
    size_t bytes_len = 0;

    // fail when serialize gets a misaligned malloc
    hs_set_allocator(misaligned_malloc, misaligned_free);
    err = hs_serialize_database(db, &bytes, &bytes_len);
    ASSERT_NE(HS_SUCCESS, err);

    hs_set_allocator(nullptr, nullptr);
    hs_free_database(db);
}

TEST(Serialize, DeserializeNullMalloc) {
    hs_database_t *db;
    hs_compile_error_t *c_err;
    static const char *pat = "hatstand.*(badgerbrush|teakettle)";

    hs_error_t err = hs_compile(pat, 0, HS_MODE_BLOCK, nullptr, &db, &c_err);
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
    err = hs_serialized_database_size(bytes, bytes_len, &ser_len);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(0, ser_len);

    err = hs_deserialize_database_at(bytes, ser_len, nullptr);
    ASSERT_NE(HS_SUCCESS, err);
    free(bytes);
}

TEST(Serialize, DeserializeUnalignedMalloc) {
    hs_database_t *db;
    hs_compile_error_t *c_err;
    static const char *pat = "hatstand.*(badgerbrush|teakettle)";

    hs_error_t err = hs_compile(pat, 0, HS_MODE_BLOCK, nullptr, &db, &c_err);
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
    err = hs_serialized_database_size(bytes, bytes_len, &ser_len);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(0, ser_len);

    // and now fail when deserialize addr is unaligned
    char *new_db = (char *)malloc(ser_len + 8);
    for (int i = 1; i < 8; i++) {
        err = hs_deserialize_database_at(bytes, ser_len,
                                         (hs_database_t *)(new_db + i));
        ASSERT_NE(HS_SUCCESS, err);
    }
    free(new_db);
    free(bytes);
}

TEST(Serialize, DeserializeGarbage) {
    hs_database_t *db;
    hs_compile_error_t *c_err;
    static const char *pat = "hatstand.*(badgerbrush|teakettle)";

    hs_error_t err = hs_compile(pat, 0, HS_MODE_BLOCK, nullptr, &db, &c_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    // determine database size for subsequent hs_deserialize_database_at
    size_t db_len;
    err = hs_database_size(db, &db_len);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(0, db_len);

    // serialize
    char *bytes = nullptr;
    size_t bytes_len = 0;

    err = hs_serialize_database(db, &bytes, &bytes_len);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(0, bytes_len);

    hs_free_database(db);

    // append '\0' byte to the serialized string to spoil it
    bytes = (char *)realloc(bytes, bytes_len + 1);
    ASSERT_NE(nullptr, bytes);
    bytes[bytes_len] = '\0';

    // create set of invalid serializations
    struct Arg {
        char *start;
        size_t len;
    };

    const Arg invalid_args[] = {
        {bytes + 1, bytes_len},
        {bytes + 1, bytes_len - 1},
        {bytes, bytes_len - 1},
        {bytes, bytes_len + 1},
    };

    for (const Arg &arg : invalid_args) {
        hs_database_t *a_db;
        err = hs_deserialize_database(arg.start, arg.len, &a_db);
        ASSERT_NE(HS_SUCCESS, err);

        char *new_db = (char *)malloc(db_len);
        ASSERT_NE(nullptr, new_db);
        err = hs_deserialize_database_at(arg.start, arg.len,
                                         (hs_database_t *)(new_db));
        ASSERT_NE(HS_SUCCESS, err);
        free(new_db);

        char *info;
        err = hs_serialized_database_info(arg.start, arg.len, &info);
        ASSERT_NE(HS_SUCCESS, err);

        size_t ser_len;
        err = hs_serialized_database_size(arg.start, arg.len, &ser_len);
        ASSERT_NE(HS_SUCCESS, err);
    }

    free(bytes);
}

}
