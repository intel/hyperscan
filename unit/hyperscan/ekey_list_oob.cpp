/*
 * Copyright (c) 2026, Intel Corporation
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

#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "gtest/gtest.h"

#include "hs.h"
#include "hs_compile.h"
#include "hs_runtime.h"
#include "database.h"
#include "ue2common.h"
#include "rose/rose_internal.h"
#include "util/exhaust.h"
#include "hs_db_hmac_key.h"

#include <openssl/hmac.h>
#include <openssl/evp.h>

namespace {

// Serialized layout: magic(4) + version(4) + length(4) + platform(8) +
// hmac(32) + hmac_hdr(32) = 84 bytes, then `length` bytes of bytecode.
constexpr size_t kHmacOffset = 3 * sizeof(u32) + sizeof(u64a); // 20
constexpr size_t kBytecodeStart = kHmacOffset + 32 + 32;       // 84

static void reseal_bytecode_hmac(char *serialized, size_t bytecode_len) {
    u8 new_hmac[32] = {0};
    unsigned int hmac_len = 32;
    HMAC(EVP_sha256(), HS_DB_HMAC_KEY, sizeof(HS_DB_HMAC_KEY),
         reinterpret_cast<const unsigned char *>(serialized + kBytecodeStart),
         bytecode_len, new_hmac, &hmac_len);
    memcpy(serialized + kHmacOffset, new_hmac, sizeof(new_hmac));
}

// Build a SINGLEMATCH (exhaustible) multi-pattern database whose suffix
// engines carry ekey lists, and serialize it. Returns false if the current
// build cannot produce such a database.
static bool make_serialized_ekey_db(char **bytes, size_t *length) {
    // Two alternation-heavy exhaustible patterns, as in the upstream PoC:
    // these produce suffix NFAs with populated ekey lists.
    const char *const exprs[] = {"foo.*bar.*baz", "qux.*quux.*quuz"};
    const unsigned flags[] = {HS_FLAG_SINGLEMATCH, HS_FLAG_SINGLEMATCH};
    const unsigned ids[] = {100, 101};

    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile_multi(exprs, flags, ids, 2, HS_MODE_STREAM,
                                      nullptr, &db, &compile_err);
    if (err != HS_SUCCESS) {
        if (compile_err) {
            hs_free_compile_error(compile_err);
        }
        return false;
    }

    err = hs_serialize_database(db, bytes, length);
    hs_free_database(db);
    return err == HS_SUCCESS && *bytes != nullptr;
}

// Locate the first NfaInfo entry with a non-zero ekeyListOffset. Returns the
// byte offset (within the serialized blob) of that ekeyListOffset field, or 0.
static size_t find_ekey_list_field(const char *bytes, u32 *rose_size_out) {
    const auto *rose =
        reinterpret_cast<const struct RoseEngine *>(bytes + kBytecodeStart);
    *rose_size_out = rose->size;

    if (!rose->nfaInfoOffset || !rose->queueCount) {
        return 0;
    }

    for (u32 qi = 0; qi < rose->queueCount; qi++) {
        size_t info_rel = rose->nfaInfoOffset + qi * sizeof(struct NfaInfo);
        const auto *info = reinterpret_cast<const struct NfaInfo *>(
            bytes + kBytecodeStart + info_rel);
        if (info->ekeyListOffset) {
            return kBytecodeStart + info_rel +
                   offsetof(struct NfaInfo, ekeyListOffset);
        }
    }
    return 0;
}

// gtest ASSERT_* returns from the enclosing function, so every resource is
// held by an RAII wrapper to keep those early exits leak-free.
struct SerializedDb {
    char *bytes = nullptr;
    size_t length = 0;
    ~SerializedDb() { free(bytes); }
};

struct DbHolder {
    hs_database_t *db = nullptr;
    ~DbHolder() {
        if (db) {
            hs_free_database(db);
        }
    }
};

struct ScratchHolder {
    hs_scratch_t *scratch = nullptr;
    ~ScratchHolder() {
        if (scratch) {
            hs_free_scratch(scratch);
        }
    }
};

struct StreamHolder {
    hs_stream_t *stream = nullptr;
    ~StreamHolder() {
        if (stream) {
            hs_close_stream(stream, nullptr, nullptr, nullptr);
        }
    }
    hs_stream_t *release() {
        hs_stream_t *s = stream;
        stream = nullptr;
        return s;
    }
};

// ============================================================================
// Baseline / false-positive control: the unmodified exhaustible database must
// deserialize and scan cleanly.
// ============================================================================
TEST(EkeyListOob, BaselineUnmodifiedDatabaseWorks) {
    SerializedDb ser;
    if (!make_serialized_ekey_db(&ser.bytes, &ser.length)) {
        return; // unsupported in this build
    }

    DbHolder db;
    ASSERT_EQ(HS_SUCCESS,
              hs_deserialize_database(ser.bytes, ser.length, &db.db));

    ScratchHolder scratch;
    ASSERT_EQ(HS_SUCCESS, hs_alloc_scratch(db.db, &scratch.scratch));

    StreamHolder stream;
    ASSERT_EQ(HS_SUCCESS, hs_open_stream(db.db, 0, &stream.stream));

    int matches = 0;
    auto cb = [](unsigned int, unsigned long long, unsigned long long,
                 unsigned int, void *ctx) -> int {
        (*static_cast<int *>(ctx))++;
        return 0;
    };
    const char *d1 = "qux11quux22";
    EXPECT_EQ(HS_SUCCESS,
              hs_scan_stream(stream.stream, d1,
                             static_cast<unsigned int>(strlen(d1)), 0,
                             scratch.scratch, cb, &matches));
    const char *d2 = "foo33bar44baz";
    EXPECT_EQ(HS_SUCCESS,
              hs_scan_stream(stream.stream, d2,
                             static_cast<unsigned int>(strlen(d2)), 0,
                             scratch.scratch, cb, &matches));

    // Close through the real callback so EOD matches are delivered.
    hs_close_stream(stream.release(), scratch.scratch, cb, &matches);

    EXPECT_GT(matches, 0) << "baseline produced no match; the test input does "
                             "not exercise the suffix catch-up path";
}

// ============================================================================
// Exploit: point ekeyListOffset at the last u32 of the engine blob and store a
// valid (non-terminator) key there. The list then has no INVALID_EKEY
// terminator inside the allocation, so roseSuffixInfoIsExhausted() walks off
// the end during a streaming scan.
//
// Before the fix: hs_deserialize_database() accepts the blob and the scan
// reads past the engine allocation (ASan: heap-buffer-overflow READ of size 4
// at catchup.c:76 in roseSuffixInfoIsExhausted).
// After the fix: hs_deserialize_database() rejects the blob with HS_INVALID.
// ============================================================================
TEST(EkeyListOob, DeserializeRejectsUnterminatedEkeyList) {
    SerializedDb ser;
    if (!make_serialized_ekey_db(&ser.bytes, &ser.length)) {
        return; // unsupported in this build
    }

    u32 rose_size = 0;
    size_t field_off = find_ekey_list_field(ser.bytes, &rose_size);
    if (!field_off) {
        return; // no NfaInfo with an ekey list in this build
    }
    ASSERT_GE(rose_size, sizeof(u32));

    // Last u32 slot inside the engine blob.
    u32 bad_offset = (rose_size - sizeof(u32)) & ~(sizeof(u32) - 1);
    memcpy(ser.bytes + field_off, &bad_offset, sizeof(bad_offset));

    // Store a valid exhaustion key (0) there so the walk does not stop: the
    // next read is already outside the allocation.
    u32 first_key = 0;
    memcpy(ser.bytes + kBytecodeStart + bad_offset, &first_key,
           sizeof(first_key));

    u32 bytecode_len;
    memcpy(&bytecode_len, ser.bytes + 2 * sizeof(u32), sizeof(bytecode_len));
    reseal_bytecode_hmac(ser.bytes, bytecode_len);

    DbHolder db;
    hs_error_t err = hs_deserialize_database(ser.bytes, ser.length, &db.db);
    EXPECT_EQ(HS_INVALID, err)
        << "forged ekeyListOffset=" << bad_offset << " (rose_size="
        << rose_size << ") was accepted; roseSuffixInfoIsExhausted() would "
           "read past the engine allocation";
}

// ============================================================================
// Exploit variant: ekeyListOffset pointing outside the engine blob entirely.
// ============================================================================
TEST(EkeyListOob, DeserializeRejectsOutOfBoundsEkeyListOffset) {
    SerializedDb ser;
    if (!make_serialized_ekey_db(&ser.bytes, &ser.length)) {
        return; // unsupported in this build
    }

    u32 rose_size = 0;
    size_t field_off = find_ekey_list_field(ser.bytes, &rose_size);
    if (!field_off) {
        return; // no NfaInfo with an ekey list in this build
    }

    u32 bad_offset = rose_size + 4096;
    memcpy(ser.bytes + field_off, &bad_offset, sizeof(bad_offset));

    u32 bytecode_len;
    memcpy(&bytecode_len, ser.bytes + 2 * sizeof(u32), sizeof(bytecode_len));
    reseal_bytecode_hmac(ser.bytes, bytecode_len);

    DbHolder db;
    hs_error_t err = hs_deserialize_database(ser.bytes, ser.length, &db.db);
    EXPECT_EQ(HS_INVALID, err)
        << "ekeyListOffset past the end of the engine blob was accepted";
}

// ============================================================================
// Exploit variant: a terminated, in-bounds list holding an exhaustion key
// beyond ekeyCount. isExhausted() would index the ekeyCount-sized multibit
// out of range (guarded only by an assert in release builds).
// ============================================================================
TEST(EkeyListOob, DeserializeRejectsEkeyBeyondEkeyCount) {
    SerializedDb ser;
    if (!make_serialized_ekey_db(&ser.bytes, &ser.length)) {
        return; // unsupported in this build
    }

    u32 rose_size = 0;
    size_t field_off = find_ekey_list_field(ser.bytes, &rose_size);
    if (!field_off) {
        return; // no NfaInfo with an ekey list in this build
    }

    const auto *rose = reinterpret_cast<const struct RoseEngine *>(
        ser.bytes + kBytecodeStart);
    u32 ekey_count = rose->ekeyCount;

    // Overwrite the first entry of the existing (valid) list with an
    // out-of-range key, keeping the terminator in place.
    u32 list_off;
    memcpy(&list_off, ser.bytes + field_off, sizeof(list_off));
    u32 bad_key = ekey_count + 1000;
    memcpy(ser.bytes + kBytecodeStart + list_off, &bad_key, sizeof(bad_key));

    u32 bytecode_len;
    memcpy(&bytecode_len, ser.bytes + 2 * sizeof(u32), sizeof(bytecode_len));
    reseal_bytecode_hmac(ser.bytes, bytecode_len);

    DbHolder db;
    hs_error_t err = hs_deserialize_database(ser.bytes, ser.length, &db.db);
    EXPECT_EQ(HS_INVALID, err)
        << "ekey " << bad_key << " >= ekeyCount " << ekey_count
        << " was accepted; isExhausted() would index the exhaustion multibit "
           "out of range";
}

} // namespace
