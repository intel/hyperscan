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
 * \brief Regression tests for out-of-bounds write via forged
 * Castle NFA stream-state layout fields in a deserialized database.
 *
 * The root cause is that Castle::activeOffset, Castle::groupIterOffset,
 * Castle::numRepeats and Castle::numGroups are read straight from the
 * serialized blob and used to address the per-stream state with no bounds
 * check:
 *
 *   hs_open_stream() -> init_stream -> roseInitState -> init_outfixes
 *     -> nfaExecCastle_initCompressedState()
 *       -> mmbit_clear(state + castle->activeOffset, castle->numRepeats)
 *
 * mmbit_clear() memsets mmbit_size(numRepeats) bytes at the forged offset,
 * giving an attacker-controlled heap write that needs no scan input at all --
 * deserializing the database and opening a stream is enough.
 *
 * These tests verify that:
 * 1. hs_deserialize_database() rejects databases whose Castle layout fields
 *    reach outside the NFA's declared stream state.
 * 2. hs_alloc_scratch() rejects the same databases presented in memory.
 * 3. Legitimately compiled Castle patterns still compile and scan.
 *
 * Without the fix the forged databases are accepted by every validator and
 * these tests fail (and, under AddressSanitizer, opening a stream aborts with
 * a heap-buffer-overflow inside nfaExecCastle_initCompressedState).
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
#include "nfa/castle_internal.h"
#include "rose/rose_internal.h"
#include "hs_db_hmac_key.h"

#include <openssl/hmac.h>
#include <openssl/evp.h>

#ifndef HS_PLATFORM_ALL
#define HS_PLATFORM_ALL (0xabcdef11abcdef00ULL)
#endif

namespace {

/* Stream state declared by the forged NFA. Any Castle field addressing state
 * at or beyond this is out of bounds. */
const u32 CASTLE_STREAM_STATE = 8;

/* Per-SubCastle packed control block size in the forged Castle. */
const u32 SUB_CTRL_SIZE = 1;

/* rose->stateOffsets.end for the forged database. */
const u32 FORGED_STATE_END = 64;

static void compute_db_hmac(hs_database_t *db) {
    char *bytecode = (char *)db + db->bytecode;
    unsigned int hmac_len = 32;
    HMAC(EVP_sha256(), HS_DB_HMAC_KEY, sizeof(HS_DB_HMAC_KEY),
         (const unsigned char *)bytecode, db->length, db->hmac, &hmac_len);
}

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
 * Build a forged streaming-mode database holding a single Castle outfix.
 *
 * Every field that existing validators inspect is given a legitimate value, so
 * the database passes dbIsValid(), db_validate_rose_offsets() and
 * validateStateLayout(). Only the Castle-internal layout fields are forged.
 *
 * Each SubCastle gets a RepeatInfo laid out as buildCastle() does (after the
 * SubCastle array), with SUB_CTRL_SIZE bytes of stream state at
 * \a sub_stream_offset. \a repeat_info_offset overrides every SubCastle's
 * repeatInfoOffset when not ~0U.
 */
static hs_database_t *make_forged_castle_db(u32 num_repeats, u32 num_groups,
                                            u8 exclusive, u32 active_offset,
                                            u32 group_iter_offset,
                                            u32 sub_stream_offset = 0,
                                            u32 repeat_info_offset = ~0U) {
    /* Room for the SubCastle array that follows the Castle header. Absurd
     * counts are left unallocated: those are rejected on the count itself. */
    const u32 alloc_subs = num_repeats <= 4096 ? num_repeats : 0;
    const size_t castle_body =
        sizeof(struct Castle) + (size_t)alloc_subs * sizeof(struct SubCastle) +
        (size_t)alloc_subs * sizeof(struct RepeatInfo);

    const size_t bytecode_offset = ROUNDUP_N(sizeof(hs_database_t), 64);
    const size_t nfa_info_offset =
        ROUNDUP_N(sizeof(struct RoseEngine), alignof(struct NfaInfo));
    const size_t nfa_offset =
        ROUNDUP_N(nfa_info_offset + sizeof(struct NfaInfo), 64);

    size_t min_rose =
        ROUNDUP_N(nfa_offset + sizeof(struct NFA) + castle_body, 64);
    size_t rose_size = min_rose > 0x1000 ? min_rose : 0x1000;
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

    rose->mode = HS_MODE_STREAM;
    rose->runtimeImpl = ROSE_RUNTIME_FULL_ROSE;
    rose->size = static_cast<u32>(rose_size);
    rose->historyRequired = 16;
    rose->maxBiAnchoredWidth = ROSE_BOUND_INF;
    rose->floatingDistance = ROSE_BOUND_INF;
    rose->queueCount = 1;
    rose->activeArrayCount = 1;
    rose->outfixBeginQueue = 0;
    rose->outfixEndQueue = 1;
    rose->leftfixBeginQueue = 1;
    rose->initMpvNfa = MO_INVALID_IDX;
    rose->initialGroups = 1;
    rose->floating_group_mask = 1;
    rose->nfaInfoOffset = static_cast<u32>(nfa_info_offset);

    rose->dkeyLogSize = 32;
    rose->somLocationFatbitSize = 32;
    rose->scratchStateSize = 64;
    rose->activeQueueArraySize = 32;
    rose->handledKeyFatbitSize = 32;
    rose->delay_fatbit_size = 32;
    rose->anchored_fatbit_size = 32;

    rose->stateOffsets.activeLeafArray = 1;
    rose->stateOffsets.activeLeafArray_size = 1;
    rose->stateOffsets.groups = 8;
    rose->stateOffsets.groups_size = 8;
    rose->stateOffsets.history = 16;
    rose->stateOffsets.exhausted = 1;
    rose->stateOffsets.exhausted_size = 0;
    rose->stateOffsets.logicalVec = 1;
    rose->stateOffsets.logicalVec_size = 0;
    rose->stateOffsets.combVec = 1;
    rose->stateOffsets.combVec_size = 0;
    rose->stateOffsets.activeLeftArray = 1;
    rose->stateOffsets.activeLeftArray_size = 0;
    rose->stateOffsets.leftfixLagTable = 1;
    rose->stateOffsets.somValid = 1;
    rose->stateOffsets.somWritable = 1;
    rose->stateOffsets.somMultibit_size = 0;
    rose->stateOffsets.end = FORGED_STATE_END;

    struct NfaInfo *info =
        reinterpret_cast<struct NfaInfo *>(bytecode + nfa_info_offset);
    memset(info, 0, sizeof(*info));
    info->nfaOffset = static_cast<u32>(nfa_offset);
    info->stateOffset = 0;
    info->fullStateOffset = 0;

    struct NFA *nfa = reinterpret_cast<struct NFA *>(bytecode + nfa_offset);
    memset(nfa, 0, sizeof(struct NFA) + castle_body);
    nfa->type = CASTLE_NFA;
    nfa->length = static_cast<u32>(sizeof(struct NFA) + castle_body);
    nfa->queueIndex = 0;
    nfa->scratchStateSize = 32;
    nfa->streamStateSize = CASTLE_STREAM_STATE;

    struct Castle *castle = reinterpret_cast<struct Castle *>(
        reinterpret_cast<char *>(nfa) + sizeof(struct NFA));
    memset(castle, 0, sizeof(*castle));
    castle->numRepeats = num_repeats;
    castle->numGroups = num_groups;
    castle->type = CASTLE_DOT;
    castle->exclusive = exclusive;
    castle->activeIdxSize = 0;
    castle->activeOffset = active_offset;
    castle->staleIterOffset = 0;
    castle->groupIterOffset = group_iter_offset;

    struct SubCastle *subs = reinterpret_cast<struct SubCastle *>(
        reinterpret_cast<char *>(castle) + sizeof(struct Castle));
    for (u32 i = 0; i < alloc_subs; i++) {
        u32 ri_off = static_cast<u32>(
            sizeof(struct SubCastle) * (alloc_subs - i) +
            sizeof(struct RepeatInfo) * i);
        subs[i].streamStateOffset = sub_stream_offset;
        subs[i].repeatInfoOffset =
            repeat_info_offset == ~0U ? ri_off : repeat_info_offset;
        struct RepeatInfo *ri = reinterpret_cast<struct RepeatInfo *>(
            reinterpret_cast<char *>(&subs[i]) + ri_off);
        ri->packedCtrlSize = SUB_CTRL_SIZE;
        ri->stateSize = 0;
    }

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

/**
 * Serialize a forged database into the on-disk form expected by
 * hs_deserialize_database(). Caller frees with free().
 */
static char *serialize_forged_db(hs_database_t *db, size_t *out_len) {
    const size_t total = sizeof(hs_database) + db->length;
    char *bytes = static_cast<char *>(malloc(total));
    if (!bytes) {
        return nullptr;
    }

    memset(bytes, 0, total);
    size_t off = 0;
    memcpy(bytes + off, &db->magic, sizeof(db->magic)); off += 4;
    memcpy(bytes + off, &db->version, sizeof(db->version)); off += 4;
    memcpy(bytes + off, &db->length, sizeof(db->length)); off += 4;
    memcpy(bytes + off, &db->platform, sizeof(db->platform)); off += 8;
    memcpy(bytes + off, db->hmac, sizeof(db->hmac)); off += 32;
    memcpy(bytes + off, db->hmac_hdr, sizeof(db->hmac_hdr)); off += 32;
    memcpy(bytes + off, (char *)db + db->bytecode, db->length);

    *out_len = total;
    return bytes;
}

/**
 * The forged database must be refused by the deserialization gate. Before the
 * fix it is accepted, which is the vulnerability.
 */
static void expect_deserialize_rejected(hs_database_t *db, const char *what) {
    ASSERT_NE(nullptr, db);

    size_t ser_len = 0;
    char *bytes = serialize_forged_db(db, &ser_len);
    ASSERT_NE(nullptr, bytes);

    hs_database_t *db2 = nullptr;
    hs_error_t err = hs_deserialize_database(bytes, ser_len, &db2);
    free(bytes);

    if (err == HS_SUCCESS) {
        /* Deliberately do NOT open a stream here: in a vulnerable build that
         * is the out-of-bounds write, which would corrupt the test process. */
        hs_free_database(db2);
        FAIL() << "hs_deserialize_database accepted forged Castle database ("
               << what << "); Castle layout fields are unvalidated";
    }

    EXPECT_EQ(HS_INVALID, err) << what;
    EXPECT_EQ(nullptr, db2) << what;
}

/**
 * The same database handed to hs_alloc_scratch() (which runs dbIsValid())
 * must also be refused.
 */
static void expect_alloc_scratch_rejected(hs_database_t *db, const char *what) {
    ASSERT_NE(nullptr, db);

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);

    if (err == HS_SUCCESS) {
        hs_free_scratch(scratch);
        FAIL() << "hs_alloc_scratch accepted forged Castle database (" << what
               << "); Castle layout fields are unvalidated";
    }

    EXPECT_EQ(HS_INVALID, err) << what;
    EXPECT_EQ(nullptr, scratch) << what;
}

// ============================================================================
// Test 1: activeOffset = 0x200 lies far past the 8-byte Castle stream state,
// so mmbit_clear() would memset 32 bytes at state + 0x200 -- 512 bytes beyond
// the allocation.
// ============================================================================
TEST(CastleOob, DeserializeRejectsForgedActiveOffset) {
    hs_database_t *db = make_forged_castle_db(256, 256, NOT_EXCLUSIVE,
                                              0x200, 0);
    expect_deserialize_rejected(db, "activeOffset=0x200 numRepeats=256");
    free_forged_db(db);
}

TEST(CastleOob, AllocScratchRejectsForgedActiveOffset) {
    hs_database_t *db = make_forged_castle_db(256, 256, NOT_EXCLUSIVE,
                                              0x200, 0);
    expect_alloc_scratch_rejected(db, "activeOffset=0x200 numRepeats=256");
    free_forged_db(db);
}

// ============================================================================
// Test 2: activeOffset sits inside the stream state but the multibit it
// addresses runs off the end. This is the off-by-a-little case a bounds check
// on activeOffset alone would miss.
// ============================================================================
TEST(CastleOob, DeserializeRejectsActiveMultibitOverrun) {
    /* numRepeats=256 needs 32 bytes; only 8 are available. */
    hs_database_t *db = make_forged_castle_db(256, 256, NOT_EXCLUSIVE,
                                              0, 0);
    expect_deserialize_rejected(db, "activeOffset=0 numRepeats=256");
    free_forged_db(db);
}

// ============================================================================
// Test 3: numRepeats forged to a huge value. mmbit_size() saturates at
// UINT32_MAX, and mmbit_clear() would attempt an enormous memset.
// ============================================================================
TEST(CastleOob, DeserializeRejectsHugeNumRepeats) {
    hs_database_t *db = make_forged_castle_db(0xffffffffu, 1, NOT_EXCLUSIVE,
                                              0, 0);
    expect_deserialize_rejected(db, "numRepeats=0xffffffff");
    free_forged_db(db);
}

// ============================================================================
// Test 4: the exclusive branch. groupIterOffset feeds a second, independent
// mmbit_clear() guarded by castle->exclusive.
// ============================================================================
TEST(CastleOob, DeserializeRejectsForgedGroupIterOffset) {
    hs_database_t *db = make_forged_castle_db(1, 64, EXCLUSIVE,
                                              0, 0x200);
    expect_deserialize_rejected(db, "groupIterOffset=0x200 numGroups=64");
    free_forged_db(db);
}

TEST(CastleOob, DeserializeRejectsGroupMultibitOverrun) {
    /* numGroups=256 needs 32 bytes at offset 0; only 8 are available. */
    hs_database_t *db = make_forged_castle_db(1, 256, EXCLUSIVE, 0, 0);
    expect_deserialize_rejected(db, "groupIterOffset=0 numGroups=256");
    free_forged_db(db);
}

// ============================================================================
// Test 5: PURE_EXCLUSIVE skips the activeOffset branch at runtime, so a wild
// activeOffset there is harmless and must NOT be rejected -- the fix should
// not be stricter than the runtime requires. The group multibit still has to
// fit.
// ============================================================================
TEST(CastleOob, PureExclusiveIgnoresActiveOffset) {
    hs_database_t *db = make_forged_castle_db(1, 1, PURE_EXCLUSIVE,
                                              0xdeadbeef, 0);
    ASSERT_NE(nullptr, db);

    size_t ser_len = 0;
    char *bytes = serialize_forged_db(db, &ser_len);
    ASSERT_NE(nullptr, bytes);

    hs_database_t *db2 = nullptr;
    hs_error_t err = hs_deserialize_database(bytes, ser_len, &db2);
    free(bytes);

    EXPECT_EQ(HS_SUCCESS, err)
        << "PURE_EXCLUSIVE never reads activeOffset; rejecting it would break "
           "legitimate databases";
    if (err == HS_SUCCESS) {
        hs_free_database(db2);
    }
    free_forged_db(db);
}

// ============================================================================
// SubCastle state ranges: the runtime accesses packedCtrlSize + stateSize
// bytes at q->streamState + streamStateOffset, through a RepeatInfo located
// at SubCastle + repeatInfoOffset. The control case is accepted; each forged
// case differs from it in a single field.
// ============================================================================
TEST(CastleOob, SubCastleValidLayoutAccepted) {
    hs_database_t *db = make_forged_castle_db(1, 0, NOT_EXCLUSIVE, 0, 0,
                                              CASTLE_STREAM_STATE -
                                                  SUB_CTRL_SIZE);
    ASSERT_NE(nullptr, db);

    size_t ser_len = 0;
    char *bytes = serialize_forged_db(db, &ser_len);
    free_forged_db(db);
    ASSERT_NE(nullptr, bytes);

    hs_database_t *db2 = nullptr;
    hs_error_t err = hs_deserialize_database(bytes, ser_len, &db2);
    free(bytes);
    EXPECT_EQ(HS_SUCCESS, err);
    hs_free_database(db2);
}

TEST(CastleOob, DeserializeRejectsSubCastleStreamStateOverrun) {
    /* Control block ends one byte past the 8-byte Castle stream state. */
    hs_database_t *db = make_forged_castle_db(1, 0, NOT_EXCLUSIVE, 0, 0,
                                              CASTLE_STREAM_STATE -
                                                  SUB_CTRL_SIZE + 1);
    expect_deserialize_rejected(db, "SubCastle streamStateOffset overrun");
    expect_alloc_scratch_rejected(db, "SubCastle streamStateOffset overrun");
    free_forged_db(db);
}

TEST(CastleOob, DeserializeRejectsSubCastleHugeStreamStateOffset) {
    hs_database_t *db = make_forged_castle_db(1, 0, NOT_EXCLUSIVE, 0, 0,
                                              0xfffffff0u);
    expect_deserialize_rejected(db, "SubCastle streamStateOffset=0xfffffff0");
    free_forged_db(db);
}

TEST(CastleOob, DeserializeRejectsSubCastleRepeatInfoOffsetOob) {
    hs_database_t *db = make_forged_castle_db(1, 0, NOT_EXCLUSIVE, 0, 0, 0,
                                              0x10000);
    expect_deserialize_rejected(db, "SubCastle repeatInfoOffset=0x10000");
    free_forged_db(db);
}

// ============================================================================
// Test 6: legitimately compiled Castle patterns must keep working, in both
// block and streaming mode, so the new validation does not over-reject.
// ============================================================================
TEST(CastleOob, LegitCastlePatternBlockMode) {
    static const char *const patterns[] = {
        "abc{2,4}def",
        "foo[^X]{4,10}bar",
        "a.{5,20}b",
    };

    for (const char *pat : patterns) {
        hs_database_t *db = nullptr;
        hs_compile_error_t *compile_err = nullptr;
        hs_error_t err = hs_compile(pat, 0, HS_MODE_BLOCK, nullptr, &db,
                                    &compile_err);
        ASSERT_EQ(HS_SUCCESS, err) << "pattern: " << pat;
        ASSERT_NE(nullptr, db) << "pattern: " << pat;

        hs_scratch_t *scratch = nullptr;
        ASSERT_EQ(HS_SUCCESS, hs_alloc_scratch(db, &scratch)) << "pattern: " << pat;

        const char input[] = "xxabcccdef foozzzzzzbar aqqqqqqqqb";
        unsigned match_count = 0;
        auto cb = [](unsigned, unsigned long long, unsigned long long,
                     unsigned, void *ctx) -> int {
            ++(*static_cast<unsigned *>(ctx));
            return 0;
        };
        EXPECT_EQ(HS_SUCCESS, hs_scan(db, input, sizeof(input) - 1, 0, scratch,
                                      cb, &match_count)) << "pattern: " << pat;

        hs_free_scratch(scratch);
        hs_free_database(db);
    }
}

TEST(CastleOob, LegitCastlePatternStreamMode) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("abc{2,4}def", 0, HS_MODE_STREAM, nullptr,
                                &db, &compile_err);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(nullptr, db);

    hs_scratch_t *scratch = nullptr;
    ASSERT_EQ(HS_SUCCESS, hs_alloc_scratch(db, &scratch));

    hs_stream_t *stream = nullptr;
    ASSERT_EQ(HS_SUCCESS, hs_open_stream(db, 0, &stream));

    const char input[] = "xxabcccdef";
    unsigned match_count = 0;
    auto cb = [](unsigned, unsigned long long, unsigned long long,
                 unsigned, void *ctx) -> int {
        ++(*static_cast<unsigned *>(ctx));
        return 0;
    };
    ASSERT_EQ(HS_SUCCESS, hs_scan_stream(stream, input, sizeof(input) - 1, 0,
                                         scratch, cb, &match_count));
    ASSERT_EQ(HS_SUCCESS, hs_close_stream(stream, scratch, cb, &match_count));
    EXPECT_EQ(1u, match_count);

    hs_free_scratch(scratch);
    hs_free_database(db);
}

/**
 * A legitimately compiled database must survive a serialize/deserialize round
 * trip: this is the regression guard against the new Castle check rejecting
 * real engines.
 */
TEST(CastleOob, LegitCastlePatternSurvivesRoundTrip) {
    static const char *const patterns[] = {
        "abc{2,4}def",
        "foo[^X]{4,10}bar",
        "(abc){3,8}",
        "x[0-9]{2,32}y",
    };

    for (const char *pat : patterns) {
        hs_database_t *db = nullptr;
        hs_compile_error_t *compile_err = nullptr;
        ASSERT_EQ(HS_SUCCESS, hs_compile(pat, 0, HS_MODE_STREAM, nullptr, &db,
                                         &compile_err)) << "pattern: " << pat;

        char *bytes = nullptr;
        size_t len = 0;
        ASSERT_EQ(HS_SUCCESS, hs_serialize_database(db, &bytes, &len))
            << "pattern: " << pat;

        hs_database_t *db2 = nullptr;
        EXPECT_EQ(HS_SUCCESS, hs_deserialize_database(bytes, len, &db2))
            << "legit Castle database rejected after round trip, pattern: "
            << pat;

        if (db2) {
            hs_stream_t *stream = nullptr;
            hs_scratch_t *scratch = nullptr;
            EXPECT_EQ(HS_SUCCESS, hs_alloc_scratch(db2, &scratch))
                << "pattern: " << pat;
            EXPECT_EQ(HS_SUCCESS, hs_open_stream(db2, 0, &stream))
                << "pattern: " << pat;
            if (stream) {
                auto cb = [](unsigned, unsigned long long, unsigned long long,
                             unsigned, void *) -> int { return 0; };
                hs_close_stream(stream, scratch, cb, nullptr);
            }
            hs_free_scratch(scratch);
            hs_free_database(db2);
        }

        free(bytes);
        hs_free_database(db);
    }
}

} // namespace
