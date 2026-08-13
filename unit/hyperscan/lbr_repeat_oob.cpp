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
 * \brief Regression tests for out-of-bounds read via forged LBR
 * repeatInfoOffset in a deserialized database.
 *
 * The root cause is that nfaExecLbrDot_queueInitState() trusts the serialized
 * lbr_common.repeatInfoOffset to locate RepeatInfo within the NFA image.
 * A forged offset (e.g. 0x70000000) causes getRepeatInfo() to derive a
 * pointer outside the NFA blob, leading to an OOB read in clearRepeat().
 *
 * These tests verify that:
 * 1. hs_alloc_scratch / hs_scan rejects databases with OOB repeatInfoOffset
 * 2. A normal LBR pattern compiled legitimately still works
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
#include "nfa/nfa_internal.h"
#include "nfa/lbr_internal.h"
#include "nfa/accel.h"
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
 * Helper: compute HMAC-SHA256 over header fields (magic, version, length,
 * platform) to populate hmac_hdr.
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
 * Build a minimal forged in-memory database containing an LBR_NFA_DOT engine
 * with the specified repeatInfoOffset. The database is structured as:
 *   [hs_database_t header][RoseEngine][NfaInfo][NFA + lbr_dot body]
 */
static hs_database_t *make_forged_lbr_db(u32 repeat_info_offset,
                                          size_t rose_size = 0x1000) {
    const size_t bytecode_offset = ROUNDUP_N(sizeof(hs_database_t), 64);
    const size_t nfa_info_offset =
        ROUNDUP_N(sizeof(struct RoseEngine), alignof(struct NfaInfo));
    const size_t nfa_offset =
        ROUNDUP_N(nfa_info_offset + sizeof(struct NfaInfo), 64);
    const size_t nfa_body_size = 64;

    /* Ensure rose_size is big enough */
    size_t min_rose = ROUNDUP_N(nfa_offset + sizeof(struct NFA) + nfa_body_size, 64);
    if (rose_size < min_rose) {
        rose_size = min_rose;
    }

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

    char *bytecode = static_cast<char *>(mem) + bytecode_offset;
    struct RoseEngine *rose = reinterpret_cast<struct RoseEngine *>(bytecode);
    memset(rose, 0, rose_size);

    rose->mode = HS_MODE_BLOCK;
    rose->runtimeImpl = ROSE_RUNTIME_SINGLE_OUTFIX;
    rose->size = static_cast<u32>(rose_size);
    rose->maxBiAnchoredWidth = ROSE_BOUND_INF;
    rose->queueCount = 1;
    rose->activeArrayCount = 1;
    rose->outfixEndQueue = 1;
    rose->initialGroups = 1;
    rose->floating_group_mask = 1;
    rose->nfaInfoOffset = static_cast<u32>(nfa_info_offset);

    /* Fatbit / scratch sizes */
    rose->dkeyLogSize = 32;
    rose->somLocationFatbitSize = 32;
    rose->scratchStateSize = 64;
    rose->activeQueueArraySize = 32;
    rose->handledKeyFatbitSize = 32;
    rose->delay_fatbit_size = 32;
    rose->anchored_fatbit_size = 32;

    /* State offsets */
    rose->stateOffsets.activeLeafArray = 8;
    rose->stateOffsets.activeLeafArray_size = 1;
    rose->stateOffsets.groups = 1;
    rose->stateOffsets.groups_size = 0;
    rose->stateOffsets.history = 1;
    rose->stateOffsets.exhausted = 1;
    rose->stateOffsets.exhausted_size = 0;
    rose->stateOffsets.logicalVec = 1;
    rose->stateOffsets.logicalVec_size = 0;
    rose->stateOffsets.combVec = 1;
    rose->stateOffsets.combVec_size = 0;
    rose->stateOffsets.somValid = 1;
    rose->stateOffsets.somWritable = 1;
    rose->stateOffsets.somMultibit_size = 0;
    rose->stateOffsets.end = 32;

    /* NfaInfo */
    struct NfaInfo *info = reinterpret_cast<struct NfaInfo *>(bytecode + nfa_info_offset);
    memset(info, 0, sizeof(*info));
    info->nfaOffset = static_cast<u32>(nfa_offset);
    info->stateOffset = 0;
    info->fullStateOffset = 0;

    /* NFA header */
    struct NFA *nfa = reinterpret_cast<struct NFA *>(bytecode + nfa_offset);
    memset(nfa, 0, sizeof(struct NFA) + nfa_body_size);
    nfa->type = LBR_NFA_DOT;
    nfa->length = static_cast<u32>(sizeof(struct NFA) + nfa_body_size);
    nfa->scratchStateSize = sizeof(struct lbr_state);
    nfa->streamStateSize = 0;
    nfa->rAccelType = ACCEL_NONE;

    /* LBR body with forged offset */
    struct lbr_dot *lbr = reinterpret_cast<struct lbr_dot *>(
        reinterpret_cast<char *>(nfa) + sizeof(struct NFA));
    lbr->common.repeatInfoOffset = repeat_info_offset;
    lbr->common.report = 1;

    /* Compute valid HMACs so the database passes integrity checks */
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
// Test 1: A forged repeatInfoOffset of 0x70000000 must be rejected.
// ============================================================================
TEST(LbrRepeatOob, ScanRejectsForgedRepeatInfoOffset) {
    hs_database_t *db = make_forged_lbr_db(0x70000000u);
    ASSERT_NE(nullptr, db) << "Failed to allocate forged database";

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);

    if (err == HS_SUCCESS && scratch != nullptr) {
        /* If scratch allocation passes, hs_scan must still reject the db */
        const char input[] = "x";
        err = hs_scan(db, input, sizeof(input) - 1, 0, scratch, nullptr, nullptr);

        /* After the fix, either hs_alloc_scratch or hs_scan must reject
         * the forged database. Before the fix, hs_scan drives an OOB read
         * in clearRepeat(). */
        EXPECT_NE(HS_SUCCESS, err)
            << "hs_scan should reject a database with forged "
               "repeatInfoOffset=0x70000000 (OOB read in getRepeatInfo)";

        hs_free_scratch(scratch);
    } else {
        /* hs_alloc_scratch rejecting the database is also acceptable */
        EXPECT_NE(HS_SUCCESS, err);
    }

    free_forged_db(db);
}

// ============================================================================
// Test 2: repeatInfoOffset pointing just past the NFA end must be rejected.
// NFA body is 64 bytes; offset of (sizeof(lbr_dot) + 64) exceeds the image.
// ============================================================================
TEST(LbrRepeatOob, ScanRejectsOffsetPastNfaEnd) {
    /* Offset larger than nfa_body_size will land past the NFA image */
    hs_database_t *db = make_forged_lbr_db(0x1000u);
    ASSERT_NE(nullptr, db) << "Failed to allocate forged database";

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);

    if (err == HS_SUCCESS && scratch != nullptr) {
        const char input[] = "x";
        err = hs_scan(db, input, sizeof(input) - 1, 0, scratch, nullptr, nullptr);
        EXPECT_NE(HS_SUCCESS, err)
            << "hs_scan should reject a database with repeatInfoOffset past "
               "NFA image end";
        hs_free_scratch(scratch);
    } else {
        EXPECT_NE(HS_SUCCESS, err);
    }

    free_forged_db(db);
}

// ============================================================================
// Test 3: repeatInfoOffset of UINT32_MAX (0xFFFFFFFF) must be rejected.
// ============================================================================
TEST(LbrRepeatOob, ScanRejectsMaxRepeatInfoOffset) {
    hs_database_t *db = make_forged_lbr_db(0xFFFFFFFFu);
    ASSERT_NE(nullptr, db) << "Failed to allocate forged database";

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);

    if (err == HS_SUCCESS && scratch != nullptr) {
        const char input[] = "x";
        err = hs_scan(db, input, sizeof(input) - 1, 0, scratch, nullptr, nullptr);
        EXPECT_NE(HS_SUCCESS, err)
            << "hs_scan should reject a database with "
               "repeatInfoOffset=0xFFFFFFFF";
        hs_free_scratch(scratch);
    } else {
        EXPECT_NE(HS_SUCCESS, err);
    }

    free_forged_db(db);
}

// ============================================================================
// Test 4: A legitimately compiled LBR pattern must still work correctly.
// The pattern ".{100,200}" compiles to an LBR engine internally.
// ============================================================================
TEST(LbrRepeatOob, LegitLbrPatternStillWorks) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile(".{100,200}", 0, HS_MODE_BLOCK, nullptr,
                                &db, &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(nullptr, db);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(nullptr, scratch);

    /* Build a 150-byte input string (within the {100,200} range) */
    std::string input(150, 'a');
    unsigned match_count = 0;
    err = hs_scan(db, input.c_str(), input.size(), 0, scratch,
                  [](unsigned, unsigned long long, unsigned long long,
                     unsigned, void *ctx) -> int {
                      ++(*static_cast<unsigned *>(ctx));
                      return 0;
                  },
                  &match_count);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_GT(match_count, 0u) << "Legitimate LBR pattern should match";

    hs_free_scratch(scratch);
    hs_free_database(db);
}

// ============================================================================
// Test 5: Stream mode — legitimate LBR pattern must work in streaming mode.
// ============================================================================
TEST(LbrRepeatOob, LegitLbrPatternStreamMode) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile(".{100,200}", 0, HS_MODE_STREAM, nullptr,
                                &db, &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(nullptr, db);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    hs_stream_t *stream = nullptr;
    err = hs_open_stream(db, 0, &stream);
    ASSERT_EQ(HS_SUCCESS, err);

    /* Feed 150 bytes in two chunks */
    std::string chunk1(75, 'a');
    std::string chunk2(75, 'a');
    unsigned match_count = 0;
    auto cb = [](unsigned, unsigned long long, unsigned long long,
                 unsigned, void *ctx) -> int {
        ++(*static_cast<unsigned *>(ctx));
        return 0;
    };

    err = hs_scan_stream(stream, chunk1.c_str(), chunk1.size(), 0, scratch,
                         cb, &match_count);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_scan_stream(stream, chunk2.c_str(), chunk2.size(), 0, scratch,
                         cb, &match_count);
    ASSERT_EQ(HS_SUCCESS, err);

    err = hs_close_stream(stream, scratch, cb, &match_count);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_GT(match_count, 0u) << "LBR pattern should match in stream mode";

    hs_free_scratch(scratch);
    hs_free_database(db);
}

// ============================================================================
// Test 6: Vectored mode — legitimate LBR pattern must work in vectored mode.
// ============================================================================
TEST(LbrRepeatOob, LegitLbrPatternVectoredMode) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile(".{100,200}", 0, HS_MODE_VECTORED, nullptr,
                                &db, &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(nullptr, db);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    std::string part1(75, 'a');
    std::string part2(75, 'a');
    const char *data[] = {part1.c_str(), part2.c_str()};
    unsigned int lengths[] = {(unsigned)part1.size(), (unsigned)part2.size()};

    unsigned match_count = 0;
    err = hs_scan_vector(db, data, lengths, 2, 0, scratch,
                         [](unsigned, unsigned long long, unsigned long long,
                            unsigned, void *ctx) -> int {
                             ++(*static_cast<unsigned *>(ctx));
                             return 0;
                         },
                         &match_count);
    ASSERT_EQ(HS_SUCCESS, err);
    EXPECT_GT(match_count, 0u) << "LBR pattern should match in vectored mode";

    hs_free_scratch(scratch);
    hs_free_database(db);
}

} // namespace
