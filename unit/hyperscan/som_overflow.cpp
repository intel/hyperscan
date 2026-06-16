/*
 * Copyright (c) 2026 Intel Corporation
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
#include <climits>

#ifdef HAVE__ALIGNED_MALLOC
#include <malloc.h>
#endif

#include "gtest/gtest.h"

#include "hs.h"
#include "hs_compile.h"
#include "hs_runtime.h"
#include "database.h"
#include "scratch.h"
#include "ue2common.h"
#include "rose/rose_internal.h"
#include "util/multibit_internal.h"
#include "hs_db_hmac_key.h"

#include <openssl/hmac.h>
#include <openssl/evp.h>

// HS_PLATFORM_ALL allows the database to run on any platform
#ifndef HS_PLATFORM_ALL
#define HS_PLATFORM_ALL (0xabcdef11abcdef00ULL)
#endif

namespace {

/**
 * Helper: compute HMAC-SHA256 over the bytecode portion of a database,
 * using the same key as Hyperscan's db_check_integrity.
 */
static void compute_db_hmac(hs_database_t *db) {
    char *bytecode = (char *)db + db->bytecode;
    unsigned int hmac_len = 32;
    HMAC(EVP_sha256(), HS_DB_HMAC_KEY, sizeof(HS_DB_HMAC_KEY),
         (const unsigned char *)bytecode, db->length, db->hmac, &hmac_len);
}

/**
 * Helper: allocate and initialize a minimal forged hs_database_t with a
 * RoseEngine that has the specified somLocationCount. All other fields are
 * set to minimal valid values that pass validate_queue_fatbits().
 */
static hs_database_t *make_forged_db(u32 som_location_count,
                                     size_t rose_size = 0x1000) {
    const size_t bytecode_offset = ROUNDUP_N(sizeof(hs_database_t), 64);
    const size_t total_size = bytecode_offset + rose_size;

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
    db->length = static_cast<u32>(rose_size);
    db->bytecode = static_cast<u32>(bytecode_offset);
    db->platform = HS_PLATFORM_ALL;
    struct RoseEngine *rose =
        reinterpret_cast<struct RoseEngine *>(static_cast<char *>(mem) +
                                              bytecode_offset);
    memset(rose, 0, rose_size);
    rose->size = static_cast<u32>(rose_size);
    rose->mode = HS_MODE_BLOCK;

    // Set somLocationCount to the attacker-controlled value
    rose->somLocationCount = som_location_count;

    // somLocationFatbitSize must be >= rt_fatbit_size(somLocationCount)
    // to pass validate_queue_fatbits()
    rose->somLocationFatbitSize = rt_fatbit_size(som_location_count);

    // Set other fatbit fields to consistent values.
    // rt_fatbit_size(0) returns 8 (minimum fatbit struct size), so
    // even with zero counts we must set sizes >= 8 to pass validation.
    rose->queueCount = 0;
    rose->activeQueueArraySize = rt_fatbit_size(0);
    rose->handledKeyCount = 0;
    rose->handledKeyFatbitSize = rt_fatbit_size(0);
    rose->delay_count = 0;
    rose->delay_fatbit_size = rt_fatbit_size(0);

    // Ensure literal matcher offsets are 0 (returns NULL, skips lit path)
    rose->fmatcherOffset = 0;
    rose->ematcherOffset = 0;

    // Compute valid HMAC so the database passes integrity checks
    compute_db_hmac(db);

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
// Test 1: hs_alloc_scratch must reject a database with somLocationCount that
// causes integer overflow in som_store_size computation.
//
// somLocationCount = 0x20000001:
//   0x20000001 * 8 = 0x100000008 → truncates to 0x8 as u32
//   This would allocate only 8 bytes for som_store but the runtime writes
//   based on somLocationCount = 0x20000001 entries.
// ============================================================================
TEST(PsirtSomOverflow, AllocScratchRejectsOverflowingSomCount) {
    hs_database_t *db = make_forged_db(0x20000001u);
    ASSERT_NE(nullptr, db) << "Failed to allocate forged database";

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);

    // After the fix: hs_alloc_scratch must detect the integer overflow and
    // return HS_INVALID.
    // Before the fix: it returns HS_SUCCESS with undersized scratch (OOB
    // write will occur on hs_scan).
    if (err == HS_SUCCESS && scratch != nullptr) {
        hs_free_scratch(scratch);
        free_forged_db(db);
        FAIL() << "hs_alloc_scratch should reject somLocationCount=0x20000001 "
                  "(integer overflow: 0x20000001*8 truncates to 8 in u32)";
    } else {
        EXPECT_NE(HS_SUCCESS, err);
    }

    free_forged_db(db);
}

// ============================================================================
// Test 2: Verify other overflow-inducing somLocationCount values are rejected.
// Any value where (somLocationCount * 8) overflows u32 must be caught.
// Boundary: overflow occurs when somLocationCount > 0x1FFFFFFF
//   0x1FFFFFFF * 8 = 0xFFFFFFF8 (fits u32)
//   0x20000000 * 8 = 0x100000000 (overflows u32 → 0)
// ============================================================================
TEST(PsirtSomOverflow, AllocScratchRejectsBoundarySomCount) {
    // 0x20000000 * 8 = 0x100000000 → truncates to 0
    hs_database_t *db = make_forged_db(0x20000000u);
    ASSERT_NE(nullptr, db) << "Failed to allocate forged database";

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);

    if (err == HS_SUCCESS && scratch != nullptr) {
        hs_free_scratch(scratch);
        free_forged_db(db);
        FAIL() << "hs_alloc_scratch should reject somLocationCount=0x20000000 "
                  "(integer overflow: 0x20000000*8 = 0 in u32)";
    } else {
        EXPECT_NE(HS_SUCCESS, err);
    }

    free_forged_db(db);
}

// ============================================================================
// Test 3: Smaller overflow value (somLocationCount = 0x80000001)
//   0x80000001 * 8 = 0x400000008 → truncates to 8
// ============================================================================
TEST(PsirtSomOverflow, AllocScratchRejectsLargeOverflowSomCount) {
    hs_database_t *db = make_forged_db(0x80000001u);
    ASSERT_NE(nullptr, db) << "Failed to allocate forged database";

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);

    if (err == HS_SUCCESS && scratch != nullptr) {
        hs_free_scratch(scratch);
        free_forged_db(db);
        FAIL() << "hs_alloc_scratch should reject somLocationCount=0x80000001 "
                  "(integer overflow: 0x80000001*8 truncates to 8 in u32)";
    } else {
        EXPECT_NE(HS_SUCCESS, err);
    }

    free_forged_db(db);
}

// ============================================================================
// Test 4: Verify that a legitimate small somLocationCount still works.
// somLocationCount = 1 is the common case for a simple SOM pattern.
// This is a false-positive control — must pass without error.
// ============================================================================
TEST(PsirtSomOverflow, AllocScratchAcceptsSmallSomCount) {
    hs_database_t *db = make_forged_db(1u);
    ASSERT_NE(nullptr, db) << "Failed to allocate forged database";

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);

    // A somLocationCount of 1 should be accepted (1*8=8, no overflow)
    EXPECT_EQ(HS_SUCCESS, err);
    if (scratch) {
        hs_free_scratch(scratch);
    }

    free_forged_db(db);
}

// ============================================================================
// Test 5: Verify max non-overflowing somLocationCount is accepted.
// 0x1FFFFFFF * 8 = 0xFFFFFFF8 — fits in u32 (no overflow).
// Note: allocation may fail due to size, but that's HS_NOMEM, not HS_INVALID.
// ============================================================================
TEST(PsirtSomOverflow, AllocScratchAcceptsMaxNonOverflowSomCount) {
    // 0x1FFFFFFF * 8 = 0xFFFFFFF8, fits u32 but is ~4GB of storage.
    // hs_alloc_scratch may return HS_NOMEM for this size, which is acceptable.
    // It must NOT silently truncate and return HS_SUCCESS with undersized
    // scratch.
    hs_database_t *db = make_forged_db(0x1FFFFFFFu);
    ASSERT_NE(nullptr, db) << "Failed to allocate forged database";

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);

    // Either HS_SUCCESS (huge allocation succeeded) or HS_NOMEM (too large)
    // or HS_INVALID (if validation rejects unreasonably large counts).
    // The key assertion: if it returns HS_SUCCESS, scratch must not be null
    // and must be properly sized.
    if (err == HS_SUCCESS) {
        EXPECT_NE(nullptr, scratch);
        if (scratch) {
            hs_free_scratch(scratch);
        }
    } else {
        // HS_NOMEM or HS_INVALID are acceptable for ~4GB request
        EXPECT_TRUE(err == HS_NOMEM || err == HS_INVALID);
    }

    free_forged_db(db);
}

// ============================================================================
// Test 6: End-to-end test using hs_compile + hs_serialize + forge + deserialize
// to replicate the exact attack chain from the PoC.
// Compiles a real SOM pattern, serializes it, forges somLocationCount in the
// serialized blob, recomputes HMAC, deserializes, and calls hs_alloc_scratch.
// ============================================================================
TEST(PsirtSomOverflow, SerializeForgeDeserializeRejectsOverflow) {
    // Step 1: Compile a real SOM pattern
    hs_database_t *real_db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("x", HS_FLAG_SOM_LEFTMOST, HS_MODE_BLOCK,
                                nullptr, &real_db, &compile_err);
    if (err != HS_SUCCESS) {
        if (compile_err) {
            hs_free_compile_error(compile_err);
        }
        // SOM may not be supported in all configurations
        return;
    }

    // Step 2: Serialize the database
    char *serialized = nullptr;
    size_t ser_len = 0;
    err = hs_serialize_database(real_db, &serialized, &ser_len);
    hs_free_database(real_db);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(nullptr, serialized);
    ASSERT_GT(ser_len, static_cast<size_t>(0));

    // Step 3: Deserialize a clean copy to find field offsets
    hs_database_t *clean_db = nullptr;
    err = hs_deserialize_database(serialized, ser_len, &clean_db);
    ASSERT_EQ(HS_SUCCESS, err);

    // Verify the real somLocationCount is small
    const struct RoseEngine *clean_rose =
        static_cast<const struct RoseEngine *>(hs_get_bytecode(clean_db));
    u32 original_som_count = clean_rose->somLocationCount;
    EXPECT_LE(original_som_count, 16u)
        << "Expected small somLocationCount for simple SOM pattern";

    // Calculate somLocationCount offset within the bytecode
    size_t som_offset_in_rose = offsetof(struct RoseEngine, somLocationCount);
    size_t som_fatbit_offset_in_rose = offsetof(struct RoseEngine,
                                                 somLocationFatbitSize);

    // In the serialized format produced by hs_serialize_database(), the bytecode
    // is written immediately after: magic/version/length (3*u32), platform (u64a)
    // and hmac (32 bytes). The remaining header space up to sizeof(hs_database_t)
    // is reserved padding.
    const size_t hmac_offset_in_serialized = 3 * sizeof(u32) + sizeof(u64a);
    const size_t bytecode_start_in_serialized = hmac_offset_in_serialized + 32;
    size_t som_offset_in_serialized = bytecode_start_in_serialized +
                                       som_offset_in_rose;
    size_t som_fatbit_offset_in_serialized = bytecode_start_in_serialized +
                                              som_fatbit_offset_in_rose;

    hs_free_database(clean_db);

    // Step 4: Make a forged copy of the serialized blob
    char *forged = static_cast<char *>(malloc(ser_len));
    ASSERT_NE(nullptr, forged);
    memcpy(forged, serialized, ser_len);

    // Patch somLocationCount to 0x20000001 (triggers integer overflow)
    u32 poison_som_count = 0x20000001u;
    memcpy(forged + som_offset_in_serialized, &poison_som_count, sizeof(u32));

    // Patch somLocationFatbitSize to match (must pass validate_queue_fatbits)
    u32 new_fatbit_size = rt_fatbit_size(poison_som_count);
    memcpy(forged + som_fatbit_offset_in_serialized, &new_fatbit_size,
           sizeof(u32));

    // Step 5: Recompute HMAC over the modified bytecode
    // HMAC covers bytes [bytecode_start_in_serialized ... +length]
    u32 bytecode_len;
    memcpy(&bytecode_len, forged + offsetof(hs_database_t, length),
           sizeof(u32));

    u8 new_hmac[32];
    unsigned int hmac_len = 32;
    HMAC(EVP_sha256(), HS_DB_HMAC_KEY, sizeof(HS_DB_HMAC_KEY),
         reinterpret_cast<const unsigned char *>(forged +
                                                  bytecode_start_in_serialized),
         bytecode_len, new_hmac, &hmac_len);
    memcpy(forged + hmac_offset_in_serialized, new_hmac, sizeof(new_hmac));

    // Step 6: Deserialize the forged blob
    hs_database_t *forged_db = nullptr;
    err = hs_deserialize_database(forged, ser_len, &forged_db);

    if (err != HS_SUCCESS) {
        // Good: deserialization rejected the forged blob (additional validation)
        free(forged);
        free(serialized);
        SUCCEED() << "hs_deserialize_database rejected forged blob (err="
                  << err << ")";
        return;
    }

    // Step 7: Try hs_alloc_scratch on the forged database
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(forged_db, &scratch);

    if (err == HS_SUCCESS && scratch != nullptr) {
        // Before fix: this succeeds with undersized scratch → OOB write on scan
        hs_free_scratch(scratch);
        hs_free_database(forged_db);
        free(forged);
        free(serialized);
        FAIL() << "hs_alloc_scratch accepted forged DB with "
                  "somLocationCount=0x20000001 (integer overflow in "
                  "som_store_size computation)";
    } else {
        // After fix: correctly rejected
        EXPECT_NE(HS_SUCCESS, err);
    }

    hs_free_database(forged_db);
    free(forged);
    free(serialized);
}

// ============================================================================
// Test 7: Verify that a legitimately compiled SOM database works end-to-end.
// This is the baseline false-positive control — compile, serialize,
// deserialize, alloc scratch, and scan must all succeed.
// ============================================================================
TEST(PsirtSomOverflow, BaselineSomDatabaseWorksCorrectly) {
    // Compile a real SOM pattern
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foo", HS_FLAG_SOM_LEFTMOST, HS_MODE_BLOCK,
                                nullptr, &db, &compile_err);
    if (err != HS_SUCCESS) {
        if (compile_err) {
            hs_free_compile_error(compile_err);
        }
        // SOM may not be supported in all configurations
        return;
    }

    // Allocate scratch
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(nullptr, scratch);

    // Scan — should succeed without crash
    const char *input = "hello foo world";
    int match_count = 0;
    auto cb = [](unsigned int id, unsigned long long from,
                 unsigned long long to, unsigned int flags,
                 void *ctx) -> int {
        (void)id; (void)from; (void)to; (void)flags;
        (*static_cast<int *>(ctx))++;
        return 0;
    };

    err = hs_scan(db, input, strlen(input), 0, scratch, cb, &match_count);
    EXPECT_EQ(HS_SUCCESS, err);
    EXPECT_GT(match_count, 0) << "Expected at least one match for 'foo'";

    hs_free_scratch(scratch);
    hs_free_database(db);
}

// ============================================================================
// Test 8: somLocationCount = 0 (no SOM) — must work fine (no overflow possible)
// ============================================================================
TEST(PsirtSomOverflow, ZeroSomCountAccepted) {
    hs_database_t *db = make_forged_db(0u);
    ASSERT_NE(nullptr, db) << "Failed to allocate forged database";

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);

    // somLocationCount=0 means no SOM — should be accepted
    EXPECT_EQ(HS_SUCCESS, err);
    if (scratch) {
        hs_free_scratch(scratch);
    }

    free_forged_db(db);
}

} // namespace
