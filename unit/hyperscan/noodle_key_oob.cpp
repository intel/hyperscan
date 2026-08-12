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

/**
 * \file
 * \brief Regression tests for out-of-bounds read via forged
 * Noodle key_offset / msk_len in a deserialized database.
 *
 * The root cause is that noodExec() trusts the serialized noodTable fields
 * key_offset and msk_len when computing a load address:
 *   partial_load_u64a(buf + pos + key_offset - msk_len, msk_len)
 * A forged database with key_offset=0 and msk_len=8 causes the subtraction
 * to underflow the scan buffer pointer, triggering an OOB read.
 *
 * These tests verify that:
 * 1. hs_alloc_scratch / hs_scan rejects databases with invalid noodle metadata
 * 2. A normal noodle-class literal pattern compiled legitimately still works
 */

#include "config.h"

#include <cstdint>
#include <cstdlib>
#include <cstring>

#ifdef HAVE__ALIGNED_MALLOC
#include <malloc.h>
#endif

#include "gtest/gtest.h"

#include "hs.h"
#include "hs_runtime.h"
#include "database.h"
#include "ue2common.h"
#include "nfa/accel.h"
#include "hwlm/hwlm_internal.h"
#include "hwlm/noodle_internal.h"
#include "rose/rose_internal.h"
#include "rose/rose_common.h"
#include "hs_db_hmac_key.h"

#include <openssl/hmac.h>
#include <openssl/evp.h>

#ifndef HS_PLATFORM_ALL
#define HS_PLATFORM_ALL (0xabcdef11abcdef00ULL)
#endif

namespace {

/**
 * Helper: compute HMAC-SHA256 over the bytecode portion of a database.
 */
static void compute_db_hmac(hs_database_t *db) {
    char *bytecode = (char *)db + db->bytecode;
    unsigned int hmac_len = 32;
    HMAC(EVP_sha256(), HS_DB_HMAC_KEY, sizeof(HS_DB_HMAC_KEY),
         (const unsigned char *)bytecode, db->length, db->hmac, &hmac_len);
}

/**
 * Helper: compute HMAC-SHA256 over header fields.
 */
static void compute_db_hmac_hdr(hs_database_t *db) {
    unsigned char hdr_buf[20];
    memcpy(hdr_buf + 0, &db->magic, 4);
    memcpy(hdr_buf + 4, &db->version, 4);
    memcpy(hdr_buf + 8, &db->length, 4);
    memcpy(hdr_buf + 12, &db->platform, sizeof(db->platform));
    unsigned int hmac_len = 32;
    HMAC(EVP_sha256(), HS_DB_HMAC_KEY, sizeof(HS_DB_HMAC_KEY),
         hdr_buf, sizeof(hdr_buf), db->hmac_hdr, &hmac_len);
}

/**
 * Build a minimal forged in-memory database with a HWLM_ENGINE_NOOD matcher
 * containing the specified key_offset and msk_len values. Layout:
 *   [hs_database_t header][RoseEngine][HWLM][noodTable]
 */
static hs_database_t *make_forged_noodle_db(u8 key_offset, u8 msk_len) {
    const size_t bytecode_offset = ROUNDUP_N(sizeof(hs_database_t), 64);
    const size_t rose_size = ROUNDUP_N(sizeof(struct RoseEngine), 64);
    const size_t hwlm_size = ROUNDUP_N(sizeof(struct HWLM), 64);
    const size_t nood_offset = rose_size + hwlm_size;
    const size_t nood_size = ROUNDUP_N(sizeof(struct noodTable), 64);
    const size_t bytecode_len = nood_offset + nood_size;
    const size_t total_size = bytecode_offset + bytecode_len;

    void *mem = nullptr;
#if defined(HAVE_POSIX_MEMALIGN)
    int rc = posix_memalign(&mem, 64, total_size);
    if (rc != 0 || mem == nullptr) {
        return nullptr;
    }
#elif defined(HAVE__ALIGNED_MALLOC)
    mem = _aligned_malloc(total_size, 64);
    if (mem == nullptr) {
        return nullptr;
    }
#else
#error "No aligned allocation function available"
#endif

    hs_database_t *db = static_cast<hs_database_t *>(mem);
    memset(db, 0, total_size);
    db->magic = HS_DB_MAGIC;
    db->version = HS_DB_VERSION;
    db->length = static_cast<u32>(bytecode_len);
    db->bytecode = static_cast<u32>(bytecode_offset);
    db->platform = HS_PLATFORM_ALL;

    char *bytecode = static_cast<char *>(mem) + bytecode_offset;

    /* RoseEngine */
    struct RoseEngine *rose = reinterpret_cast<struct RoseEngine *>(bytecode);
    memset(rose, 0, bytecode_len);

    rose->runtimeImpl = ROSE_RUNTIME_PURE_LITERAL;
    rose->mode = HS_MODE_BLOCK;
    rose->size = static_cast<u32>(bytecode_len);
    rose->maxBiAnchoredWidth = ROSE_BOUND_INF;
    rose->initialGroups = 1;
    rose->floating_group_mask = 1;
    rose->fmatcherOffset = static_cast<u32>(rose_size);
    rose->queueCount = 0;

    /* Fatbit / scratch sizes */
    rose->dkeyLogSize = 32;
    rose->somLocationFatbitSize = 32;
    rose->scratchStateSize = 0;
    rose->activeQueueArraySize = 32;
    rose->handledKeyFatbitSize = 32;
    rose->delay_fatbit_size = 32;
    rose->anchored_fatbit_size = 32;
    rose->stateOffsets.end = 1;

    /* HWLM header */
    struct HWLM *hwlm = reinterpret_cast<struct HWLM *>(bytecode + rose_size);
    hwlm->type = HWLM_ENGINE_NOOD;
    hwlm->accel0.accel_type = ACCEL_NONE;

    /* Noodle table with forged fields */
    struct noodTable *nood =
        reinterpret_cast<struct noodTable *>(bytecode + nood_offset);
    nood->id = 0x51515151u;
    nood->msk = UINT64_MAX;
    nood->cmp = 0;
    nood->msk_len = msk_len;
    nood->key_offset = key_offset;
    nood->nocase = 0;
    nood->single = 1;
    nood->key0 = 'A';
    nood->key1 = 0;

    /* Compute valid HMACs */
    compute_db_hmac(db);
    compute_db_hmac_hdr(db);

    return db;
}

static void free_forged_db(hs_database_t *db) {
    if (!db) return;
#if defined(HAVE__ALIGNED_MALLOC) && !defined(HAVE_POSIX_MEMALIGN)
    _aligned_free(db);
#else
    free(db);
#endif
}

// ============================================================================
// Test 1: key_offset=0, msk_len=8 — the exact PoC values.
// When pos=7 (last byte of 8-byte input matches 'A'):
//   load address = buf + 7 + 0 - 8 = buf - 1 → OOB read before the buffer.
// ============================================================================
TEST(NoodleOob, ScanRejectsForgedKeyOffsetMskLen) {
    hs_database_t *db = make_forged_noodle_db(/*key_offset=*/0, /*msk_len=*/8);
    ASSERT_NE(nullptr, db) << "Failed to allocate forged database";

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);

    if (err == HS_SUCCESS && scratch != nullptr) {
        /* Reproduce the PoC: 8-byte input with 'A' at end */
        unsigned char input[8] = {0};
        input[7] = 'A';

        err = hs_scan(db, reinterpret_cast<const char *>(input), sizeof(input),
                      0, scratch, nullptr, nullptr);

        /* After the fix, either alloc_scratch or hs_scan must reject */
        EXPECT_NE(HS_SUCCESS, err)
            << "hs_scan should reject a database with key_offset=0, msk_len=8 "
               "(pointer underflow: buf + pos + 0 - 8 reads before buffer)";

        hs_free_scratch(scratch);
    } else {
        EXPECT_NE(HS_SUCCESS, err);
    }

    free_forged_db(db);
}

// ============================================================================
// Test 2: msk_len > 8 — invalid since msk/cmp are u64a (8 bytes max).
// ============================================================================
TEST(NoodleOob, ScanRejectsOversizedMskLen) {
    hs_database_t *db = make_forged_noodle_db(/*key_offset=*/1, /*msk_len=*/16);
    ASSERT_NE(nullptr, db) << "Failed to allocate forged database";

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);

    if (err == HS_SUCCESS && scratch != nullptr) {
        const char input[] = "AAAAAAAAAAAAAAAA";
        err = hs_scan(db, input, sizeof(input) - 1, 0, scratch, nullptr,
                      nullptr);
        EXPECT_NE(HS_SUCCESS, err)
            << "hs_scan should reject msk_len > 8";
        hs_free_scratch(scratch);
    } else {
        EXPECT_NE(HS_SUCCESS, err);
    }

    free_forged_db(db);
}

// ============================================================================
// Test 3: msk_len=0 — should be rejected (zero-length mask is meaningless).
// ============================================================================
TEST(NoodleOob, ScanRejectsZeroMskLen) {
    hs_database_t *db = make_forged_noodle_db(/*key_offset=*/0, /*msk_len=*/0);
    ASSERT_NE(nullptr, db) << "Failed to allocate forged database";

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);

    if (err == HS_SUCCESS && scratch != nullptr) {
        const char input[] = "A";
        err = hs_scan(db, input, sizeof(input) - 1, 0, scratch, nullptr,
                      nullptr);
        /* A zero msk_len is invalid and should be caught by validation */
        EXPECT_NE(HS_SUCCESS, err)
            << "hs_scan should reject msk_len=0";
        hs_free_scratch(scratch);
    } else {
        EXPECT_NE(HS_SUCCESS, err);
    }

    free_forged_db(db);
}

// ============================================================================
// Test 4: key_offset > msk_len — the invariant key_offset <= msk_len is
// required for correct operation; violation should be rejected.
// ============================================================================
TEST(NoodleOob, ScanRejectsKeyOffsetExceedingMskLen) {
    hs_database_t *db = make_forged_noodle_db(/*key_offset=*/7, /*msk_len=*/2);
    ASSERT_NE(nullptr, db) << "Failed to allocate forged database";

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);

    if (err == HS_SUCCESS && scratch != nullptr) {
        const char input[] = "AAAAAAAA";
        err = hs_scan(db, input, sizeof(input) - 1, 0, scratch, nullptr,
                      nullptr);
        EXPECT_NE(HS_SUCCESS, err)
            << "hs_scan should reject key_offset > msk_len";
        hs_free_scratch(scratch);
    } else {
        EXPECT_NE(HS_SUCCESS, err);
    }

    free_forged_db(db);
}

// ============================================================================
// Test 5: A legitimately compiled single-literal pattern (Noodle path) must
// still work correctly. Single short literal typically uses Noodle engine.
// ============================================================================
TEST(NoodleOob, LegitNoodlePatternStillWorks) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("A", 0, HS_MODE_BLOCK, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(nullptr, db);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(nullptr, scratch);

    const char input[] = "hello A world";
    unsigned match_count = 0;
    err = hs_scan(db, input, sizeof(input) - 1, 0, scratch,
                  [](unsigned, unsigned long long, unsigned long long,
                     unsigned, void *ctx) -> int {
                      ++(*static_cast<unsigned *>(ctx));
                      return 0;
                  },
                  &match_count);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_GT(match_count, 0u) << "Legitimate noodle pattern should match";

    hs_free_scratch(scratch);
    hs_free_database(db);
}

// ============================================================================
// Test 6: Stream mode — single literal pattern via Noodle must work.
// ============================================================================
TEST(NoodleOob, LegitNoodlePatternStreamMode) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("A", 0, HS_MODE_STREAM, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(nullptr, db);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_stream_t *stream = nullptr;
    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);

    unsigned match_count = 0;
    auto cb = [](unsigned, unsigned long long, unsigned long long,
                 unsigned, void *ctx) -> int {
        ++(*static_cast<unsigned *>(ctx));
        return 0;
    };

    const char chunk1[] = "hello ";
    err = hs_scan_stream(stream, chunk1, sizeof(chunk1) - 1, 0, scratch,
                         cb, &match_count);
    ASSERT_EQ(HS_SUCCESS, err);

    const char chunk2[] = "A world";
    err = hs_scan_stream(stream, chunk2, sizeof(chunk2) - 1, 0, scratch,
                         cb, &match_count);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_close_stream(stream, scratch, cb, &match_count);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_GT(match_count, 0u) << "Noodle pattern should match in stream mode";

    hs_free_scratch(scratch);
    hs_free_database(db);
}

// ============================================================================
// Test 7: Vectored mode — single literal pattern via Noodle must work.
// ============================================================================
TEST(NoodleOob, LegitNoodlePatternVectoredMode) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("A", 0, HS_MODE_VECTORED, nullptr, &db,
                                &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(nullptr, db);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    const char part1[] = "hello ";
    const char part2[] = "A world";
    const char *data[] = {part1, part2};
    unsigned int lengths[] = {(unsigned)(sizeof(part1) - 1),
                              (unsigned)(sizeof(part2) - 1)};

    unsigned match_count = 0;
    err = hs_scan_vector(db, data, lengths, 2, 0, scratch,
                         [](unsigned, unsigned long long, unsigned long long,
                            unsigned, void *ctx) -> int {
                             ++(*static_cast<unsigned *>(ctx));
                             return 0;
                         },
                         &match_count);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_GT(match_count, 0u) << "Noodle pattern should match in vectored mode";

    hs_free_scratch(scratch);
    hs_free_database(db);
}

} // namespace
