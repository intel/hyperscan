/*
 * Copyright (c) 2015-2026, Intel Corporation
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
#include "hs_compile.h"
#include "database.h"
#include "hs_db_hmac_key.h"
#include "rose/rose_internal.h"
#include <openssl/hmac.h>
#include <openssl/evp.h>
#include "test_util.h"

#include <climits>
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

// --- HMAC tamper-detection regression tests ---

// Helper: serialize a simple block-mode database.
static void makeSerializedDB(char **bytes, size_t *length) {
    hs_database_t *db = buildDB("hatstand.*teakettle", 0, 1000, HS_MODE_BLOCK);
    ASSERT_NE(nullptr, db);
    hs_error_t err = hs_serialize_database(db, bytes, length);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(nullptr, *bytes);
    ASSERT_LT(sizeof(struct hs_database), *length);
    hs_free_database(db);
}

TEST(Serialize, HmacTamperDetectDeserialize) {
    char *bytes = nullptr;
    size_t length = 0;
    makeSerializedDB(&bytes, &length);

    hs_database_t *db = nullptr;
    hs_error_t err = hs_deserialize_database(bytes, length, &db);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
    db = nullptr;

    size_t tamper_offset = sizeof(struct hs_database) + 1;
    ASSERT_LT(tamper_offset, length);
    bytes[tamper_offset] ^= 0x01;

    err = hs_deserialize_database(bytes, length, &db);
    ASSERT_EQ(HS_INVALID, err) << "tampered database was not rejected";
    ASSERT_TRUE(db == nullptr);

    free(bytes);
}

TEST(Serialize, HmacTamperDetectDeserializeAt) {
    char *bytes = nullptr;
    size_t length = 0;
    makeSerializedDB(&bytes, &length);

    size_t db_length = 0;
    hs_error_t err = hs_serialized_database_size(bytes, length, &db_length);
    ASSERT_EQ(HS_SUCCESS, err);

    char *mem = (char *)malloc(db_length);
    ASSERT_NE(nullptr, mem);
    hs_database_t *db = (hs_database_t *)mem;

    err = hs_deserialize_database_at(bytes, length, db);
    ASSERT_EQ(HS_SUCCESS, err);

    size_t tamper_offset = sizeof(struct hs_database) + 1;
    ASSERT_LT(tamper_offset, length);
    bytes[tamper_offset] ^= 0x01;

    memset(mem, 0, db_length);
    err = hs_deserialize_database_at(bytes, length, db);
    ASSERT_EQ(HS_INVALID, err) << "tampered database was not rejected";

    free(mem);
    free(bytes);
}

TEST(Serialize, HmacTamperMultipleOffsets) {
    char *bytes = nullptr;
    size_t length = 0;
    makeSerializedDB(&bytes, &length);

    const size_t bytecode_start = 4 + 4 + 4 + 8 + 32;
    const u32 *len_field = (const u32 *)(bytes + 8);
    const u32 bc_len = *len_field;
    ASSERT_GT(bc_len, 2u);

    const size_t offsets[] = {
        bytecode_start,
        bytecode_start + bc_len / 2,
        bytecode_start + bc_len - 1
    };

    for (size_t off : offsets) {
        SCOPED_TRACE(off);

        char *copy = (char *)malloc(length);
        ASSERT_NE(nullptr, copy);
        memcpy(copy, bytes, length);

        copy[off] ^= 0x80;

        hs_database_t *db = nullptr;
        hs_error_t err = hs_deserialize_database(copy, length, &db);
        ASSERT_EQ(HS_INVALID, err)
            << "tampered byte at offset " << off << " was not rejected";
        ASSERT_TRUE(db == nullptr);

        free(copy);
    }

    free(bytes);
}

TEST(Serialize, HmacFieldCorruption) {
    char *bytes = nullptr;
    size_t length = 0;
    makeSerializedDB(&bytes, &length);

    size_t hmac_offset = offsetof(struct hs_database, hmac);
    ASSERT_LT(hmac_offset + 32, length);

    bytes[hmac_offset] ^= 0x01;

    hs_database_t *db = nullptr;
    hs_error_t err = hs_deserialize_database(bytes, length, &db);
    ASSERT_EQ(HS_INVALID, err) << "corrupted HMAC was not rejected";
    ASSERT_TRUE(db == nullptr);

    free(bytes);
}

// --- PSIRT regression tests ---

static const size_t SERIAL_BYTECODE_OFF = 4 + 4 + 4 + 8 + 32;

static void recomputeHMAC(char *bytes, size_t length) {
    const u32 bc_len = *(const u32 *)(bytes + 8);
    ASSERT_LE(SERIAL_BYTECODE_OFF + bc_len, length);
    u8 hmac[32] = {0};
    unsigned int hmac_len = 32;
    HMAC(EVP_sha256(), HS_DB_HMAC_KEY, sizeof(HS_DB_HMAC_KEY),
         (const unsigned char *)(bytes + SERIAL_BYTECODE_OFF),
         bc_len, hmac, &hmac_len);
    memcpy(bytes + 20, hmac, 32);
}

static void makeSerializedDBPat(const char *pattern, unsigned flags,
                                unsigned mode, char **bytes, size_t *length) {
    hs_database_t *db = buildDB(pattern, flags, 1000, mode);
    ASSERT_NE(nullptr, db);
    hs_error_t err = hs_serialize_database(db, bytes, length);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(nullptr, *bytes);
    hs_free_database(db);
}

static void forgeBytecodeU32(char *bytes, size_t bc_offset, u32 value) {
    memcpy(bytes + SERIAL_BYTECODE_OFF + bc_offset, &value, sizeof(u32));
}

static u32 readBytecodeU32(const char *bytes, size_t bc_offset) {
    u32 v;
    memcpy(&v, bytes + SERIAL_BYTECODE_OFF + bc_offset, sizeof(u32));
    return v;
}

TEST(Serialize, PsirtAnchoredFatbitOverflow) {
    char *bytes = nullptr;
    size_t length = 0;
    makeSerializedDBPat("hatstand.*teakettle", 0, HS_MODE_BLOCK,
                        &bytes, &length);

    const size_t off_count = offsetof(struct RoseEngine, anchored_count);
    const size_t off_fbsize = offsetof(struct RoseEngine, anchored_fatbit_size);
    u32 orig_fbsize = readBytecodeU32(bytes, off_fbsize);

    u32 forged_count = 4160;
    forgeBytecodeU32(bytes, off_count, forged_count);
    if (orig_fbsize > 32) {
        forgeBytecodeU32(bytes, off_fbsize, 32);
    }
    recomputeHMAC(bytes, length);

    hs_database_t *db = nullptr;
    hs_error_t err = hs_deserialize_database(bytes, length, &db);
    ASSERT_EQ(HS_INVALID, err)
        << "forged anchored_count/fatbit mismatch was not rejected";
    ASSERT_TRUE(db == nullptr);

    free(bytes);
}

TEST(Serialize, PsirtNullDerefWalkStrawToCyclicRev) {
    static const char *patterns[] = {
        "(?:ab|cd){2,4}[a-z]{1,3}xyz",
        "([a-z]){3,5}[0-9]foo",
        "(?:a|b){2,10}(?:c|d){2,10}efgh",
        "(?:abc|def){1,3}[x-z]{2,5}ghi",
    };

    for (const char *pat : patterns) {
        SCOPED_TRACE(pat);
        hs_database_t *db = nullptr;
        hs_compile_error_t *comp_error = nullptr;
        hs_error_t err = hs_compile(pat, 0, HS_MODE_BLOCK, nullptr,
                                    &db, &comp_error);
        if (err == HS_SUCCESS) {
            ASSERT_NE(nullptr, db);
            hs_free_database(db);
        } else {
            if (comp_error) {
                hs_free_compile_error(comp_error);
            }
        }
    }
}

TEST(Serialize, PsirtLimExRepeatMetadataForge) {
    char *bytes = nullptr;
    size_t length = 0;
    makeSerializedDBPat("a{5,10}b", 0,
                        HS_MODE_STREAM | HS_MODE_SOM_HORIZON_LARGE,
                        &bytes, &length);

    const size_t off_nfaInfo = offsetof(struct RoseEngine, nfaInfoOffset);

    const size_t off_rose_size = offsetof(struct RoseEngine, size);
    u32 rose_size = readBytecodeU32(bytes, off_rose_size);

    forgeBytecodeU32(bytes, off_nfaInfo, rose_size + 1000);
    recomputeHMAC(bytes, length);

    hs_database_t *db = nullptr;
    hs_error_t err = hs_deserialize_database(bytes, length, &db);
    ASSERT_EQ(HS_INVALID, err)
        << "forged nfaInfoOffset past rose_size was not rejected";
    ASSERT_TRUE(db == nullptr);

    free(bytes);
}

TEST(Serialize, PsirtRepeatRangeResumeState) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *comp_error = nullptr;
    hs_error_t err = hs_compile("a{3,7}b", 0, HS_MODE_STREAM,
                                nullptr, &db, &comp_error);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(nullptr, db);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_stream_t *stream = nullptr;
    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);

    CallBackContext cb;
    err = hs_scan_stream(stream, "aaaa", 4, 0, scratch, record_cb, &cb);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_scan_stream(stream, "aaab", 4, 0, scratch, record_cb, &cb);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_close_stream(stream, scratch, record_cb, &cb);
    ASSERT_EQ(HS_SUCCESS, err);

    ASSERT_FALSE(cb.matches.empty()) << "expected at least one match";

    hs_free_scratch(scratch);
    hs_free_database(db);
}

TEST(Serialize, PsirtCompileLitMultiNullExpr) {
    const char *exprs[] = { nullptr };
    const unsigned flags[] = { 0 };
    const unsigned ids[] = { 1 };
    const size_t lens[] = { 4 };
    hs_database_t *db = nullptr;
    hs_compile_error_t *comp_error = nullptr;

    hs_error_t err = hs_compile_lit_multi(exprs, flags, ids, lens, 1,
                                          HS_MODE_BLOCK, nullptr,
                                          &db, &comp_error);
    ASSERT_EQ(HS_COMPILER_ERROR, err);
    ASSERT_TRUE(db == nullptr);
    ASSERT_NE(nullptr, comp_error);
    hs_free_compile_error(comp_error);
}

TEST(Serialize, PsirtCompileLitMultiZeroLen) {
    const char *exprs[] = { "test" };
    const unsigned flags[] = { 0 };
    const unsigned ids[] = { 1 };
    const size_t lens[] = { 0 };
    hs_database_t *db = nullptr;
    hs_compile_error_t *comp_error = nullptr;

    hs_error_t err = hs_compile_lit_multi(exprs, flags, ids, lens, 1,
                                          HS_MODE_BLOCK, nullptr,
                                          &db, &comp_error);
    ASSERT_EQ(HS_COMPILER_ERROR, err);
    ASSERT_TRUE(db == nullptr);
    ASSERT_NE(nullptr, comp_error);
    hs_free_compile_error(comp_error);
}

TEST(Serialize, PsirtCompileLitMultiOversizedLen) {
    const char *exprs[] = { "test" };
    const unsigned flags[] = { 0 };
    const unsigned ids[] = { 1 };
    const size_t lens[] = { 0xFFFFFFFF };
    hs_database_t *db = nullptr;
    hs_compile_error_t *comp_error = nullptr;

    hs_error_t err = hs_compile_lit_multi(exprs, flags, ids, lens, 1,
                                          HS_MODE_BLOCK, nullptr,
                                          &db, &comp_error);
    ASSERT_EQ(HS_COMPILER_ERROR, err);
    ASSERT_TRUE(db == nullptr);
    ASSERT_NE(nullptr, comp_error);
    hs_free_compile_error(comp_error);
}

TEST(Serialize, PsirtCompileLitMultiNullLens) {
    const char *exprs[] = { "test" };
    const unsigned flags[] = { 0 };
    const unsigned ids[] = { 1 };
    hs_database_t *db = nullptr;
    hs_compile_error_t *comp_error = nullptr;

    hs_error_t err = hs_compile_lit_multi(exprs, flags, ids, nullptr, 1,
                                          HS_MODE_BLOCK, nullptr,
                                          &db, &comp_error);
    ASSERT_EQ(HS_COMPILER_ERROR, err);
    ASSERT_TRUE(db == nullptr);
    ASSERT_NE(nullptr, comp_error);
    hs_free_compile_error(comp_error);
}

TEST(Serialize, PsirtCompileLitMultiNullExprs) {
    const unsigned flags[] = { 0 };
    const unsigned ids[] = { 1 };
    const size_t lens[] = { 4 };
    hs_database_t *db = nullptr;
    hs_compile_error_t *comp_error = nullptr;

    hs_error_t err = hs_compile_lit_multi(nullptr, flags, ids, lens, 1,
                                          HS_MODE_BLOCK, nullptr,
                                          &db, &comp_error);
    ASSERT_EQ(HS_COMPILER_ERROR, err);
    ASSERT_TRUE(db == nullptr);
    ASSERT_NE(nullptr, comp_error);
    hs_free_compile_error(comp_error);
}

TEST(Serialize, PsirtRoseOffsetFmatcherOOB) {
    char *bytes = nullptr;
    size_t length = 0;
    makeSerializedDBPat("hatstand.*teakettle", 0, HS_MODE_BLOCK,
                        &bytes, &length);

    const size_t off_rose_size = offsetof(struct RoseEngine, size);
    u32 rose_size = readBytecodeU32(bytes, off_rose_size);

    forgeBytecodeU32(bytes, offsetof(struct RoseEngine, fmatcherOffset),
                     rose_size + 4096);
    recomputeHMAC(bytes, length);

    hs_database_t *db = nullptr;
    hs_error_t err = hs_deserialize_database(bytes, length, &db);
    ASSERT_EQ(HS_INVALID, err)
        << "forged fmatcherOffset was not rejected";
    ASSERT_TRUE(db == nullptr);

    free(bytes);
}

TEST(Serialize, PsirtRoseOffsetSmallWriteOOB) {
    char *bytes = nullptr;
    size_t length = 0;
    makeSerializedDBPat("hatstand.*teakettle", 0, HS_MODE_BLOCK,
                        &bytes, &length);

    const size_t off_rose_size = offsetof(struct RoseEngine, size);
    u32 rose_size = readBytecodeU32(bytes, off_rose_size);

    forgeBytecodeU32(bytes, offsetof(struct RoseEngine, smallWriteOffset),
                     rose_size + 4096);
    recomputeHMAC(bytes, length);

    hs_database_t *db = nullptr;
    hs_error_t err = hs_deserialize_database(bytes, length, &db);
    ASSERT_EQ(HS_INVALID, err)
        << "forged smallWriteOffset was not rejected";
    ASSERT_TRUE(db == nullptr);

    free(bytes);
}

TEST(Serialize, PsirtRoseOffsetReportProgramOOB) {
    char *bytes = nullptr;
    size_t length = 0;
    makeSerializedDBPat("hatstand.*teakettle", 0, HS_MODE_BLOCK,
                        &bytes, &length);

    const size_t off_rose_size = offsetof(struct RoseEngine, size);
    u32 rose_size = readBytecodeU32(bytes, off_rose_size);

    forgeBytecodeU32(bytes, offsetof(struct RoseEngine, reportProgramOffset),
                     rose_size + 4096);
    recomputeHMAC(bytes, length);

    hs_database_t *db = nullptr;
    hs_error_t err = hs_deserialize_database(bytes, length, &db);
    ASSERT_EQ(HS_INVALID, err)
        << "forged reportProgramOffset was not rejected";
    ASSERT_TRUE(db == nullptr);

    free(bytes);
}

TEST(Serialize, PsirtRoseSizeTooSmall) {
    char *bytes = nullptr;
    size_t length = 0;
    makeSerializedDBPat("hatstand.*teakettle", 0, HS_MODE_BLOCK,
                        &bytes, &length);

    forgeBytecodeU32(bytes, offsetof(struct RoseEngine, size), 16);
    recomputeHMAC(bytes, length);

    hs_database_t *db = nullptr;
    hs_error_t err = hs_deserialize_database(bytes, length, &db);
    ASSERT_EQ(HS_INVALID, err)
        << "impossibly small rose->size was not rejected";
    ASSERT_TRUE(db == nullptr);

    free(bytes);
}

TEST(Serialize, PsirtDeserializeOverflow) {
    char *bytes = nullptr;
    size_t length = 0;
    makeSerializedDBPat("hatstand.*teakettle", 0, HS_MODE_BLOCK,
                        &bytes, &length);

    u32 forged_length = 0xFFFFFFF0u;
    memcpy(bytes + 8, &forged_length, sizeof(u32));

    hs_database_t *db = nullptr;
    hs_error_t err = hs_deserialize_database(bytes, length, &db);
    ASSERT_NE(HS_SUCCESS, err)
        << "overflowing length field was not rejected";
    ASSERT_TRUE(db == nullptr);

    size_t at_buf_len = 4096;
    char *mem = (char *)malloc(at_buf_len);
    ASSERT_NE(nullptr, mem);
    err = hs_deserialize_database_at(bytes, length, (hs_database_t *)mem);
    ASSERT_NE(HS_SUCCESS, err)
        << "overflowing length field was not rejected by _at variant";
    free(mem);

    free(bytes);
}

TEST(Serialize, PsirtDeserializeMaxSize) {
    char *bytes = nullptr;
    size_t length = 0;
    makeSerializedDBPat("hatstand.*teakettle", 0, HS_MODE_BLOCK,
                        &bytes, &length);

    u32 forged_length = 0xFFFFF000u;
    memcpy(bytes + 8, &forged_length, sizeof(u32));

    hs_database_t *db = nullptr;
    hs_error_t err = hs_deserialize_database(bytes, length, &db);
    ASSERT_NE(HS_SUCCESS, err)
        << "length exceeding MAX_DATABASE_SIZE was not rejected";
    ASSERT_TRUE(db == nullptr);

    free(bytes);
}

TEST(Serialize, PsirtValidDbRoundTrip) {
    hs_database_t *db = buildDB("hatstand.*teakettle", 0, 1000,
                                HS_MODE_BLOCK);
    ASSERT_NE(nullptr, db);

    char *bytes = nullptr;
    size_t length = 0;
    hs_error_t err = hs_serialize_database(db, &bytes, &length);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
    db = nullptr;

    err = hs_deserialize_database(bytes, length, &db);
    ASSERT_EQ(HS_SUCCESS, err)
        << "valid database was rejected after PSIRT validation";
    ASSERT_NE(nullptr, db);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    CallBackContext cb;
    const char *data = "hatstand foobar teakettle";
    err = hs_scan(db, data, (unsigned)strlen(data), 0, scratch,
                  record_cb, &cb);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_FALSE(cb.matches.empty()) << "no match on valid deserialized DB";

    hs_free_scratch(scratch);
    hs_free_database(db);
    free(bytes);
}

TEST(Serialize, PsirtValidStreamRoundTrip) {
    hs_database_t *db = buildDB("a{3,7}b", 0, 1000, HS_MODE_STREAM);
    ASSERT_NE(nullptr, db);

    char *bytes = nullptr;
    size_t length = 0;
    hs_error_t err = hs_serialize_database(db, &bytes, &length);
    ASSERT_EQ(HS_SUCCESS, err);
    hs_free_database(db);
    db = nullptr;

    err = hs_deserialize_database(bytes, length, &db);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(nullptr, db);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_stream_t *stream = nullptr;
    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);

    CallBackContext cb;
    err = hs_scan_stream(stream, "aaaaa", 5, 0, scratch, record_cb, &cb);
    ASSERT_EQ(HS_SUCCESS, err);
    err = hs_scan_stream(stream, "b", 1, 0, scratch, record_cb, &cb);
    ASSERT_EQ(HS_SUCCESS, err);
    err = hs_close_stream(stream, scratch, record_cb, &cb);
    ASSERT_EQ(HS_SUCCESS, err);

    ASSERT_FALSE(cb.matches.empty()) << "no match on valid streaming DB";

    hs_free_scratch(scratch);
    hs_free_database(db);
    free(bytes);
}

TEST(Serialize, PsirtCompileLitMultiValid) {
    const char *exprs[] = { "foobar", "bazqux" };
    const unsigned flags[] = { 0, 0 };
    const unsigned ids[] = { 1, 2 };
    const size_t lens[] = { 6, 6 };
    hs_database_t *db = nullptr;
    hs_compile_error_t *comp_error = nullptr;

    hs_error_t err = hs_compile_lit_multi(exprs, flags, ids, lens, 2,
                                          HS_MODE_BLOCK, nullptr,
                                          &db, &comp_error);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(nullptr, db);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    CallBackContext cb;
    const char *data = "hello foobar world bazqux end";
    err = hs_scan(db, data, (unsigned)strlen(data), 0, scratch,
                  record_cb, &cb);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(2u, cb.matches.size()) << "expected 2 matches from lit_multi";

    hs_free_scratch(scratch);
    hs_free_database(db);
}

}
