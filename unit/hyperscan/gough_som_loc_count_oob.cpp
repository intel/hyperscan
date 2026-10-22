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
 * \brief Regression tests for heap buffer overflow via unvalidated
 * stream_som_loc_count in Gough NFA engine during deserialization.
 *
 * The root cause is that GoughInfo::stream_som_loc_count is read straight from
 * the serialized blob and used to index the som->slots array with no bounds
 * check:
 *
 *   hs_scan_stream() -> nfaExecGough*() -> compSomSpace() / expandSomSpace()
 *     -> for (u32 i = 0; i < count; i++) som->slots[i] = ...
 *
 * If stream_som_loc_count is forged to be larger than the actual allocated
 * stream SOM slots, accessing som->slots[i] for i >= num_allocated_slots
 * causes a heap buffer overflow.
 *
 * These tests verify that:
 * 1. hs_deserialize_database() rejects databases whose Gough stream_som_loc_count
 *    exceeds the allocated stream state for SOM information.
 * 2. Legitimately compiled Gough patterns still compile and scan.
 *
 * Without the fix, the forged databases are accepted by the deserializer and
 * these tests demonstrate the heap overflow when scanning with the stream.
 */

#include "config.h"

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <functional>
#include <vector>

#ifdef HAVE__ALIGNED_MALLOC
#include <malloc.h>
#endif

#include "gtest/gtest.h"

#include "hs.h"
#include "hs_runtime.h"
#include "database.h"
#include "ue2common.h"
#include "nfa/nfa_internal.h"
#include "nfa/mcclellan_internal.h"
#include "nfa/gough_internal.h"
#include "rose/rose_internal.h"
#include "hs_db_hmac_key.h"

#include <openssl/hmac.h>
#include <openssl/evp.h>

#ifndef HS_PLATFORM_ALL
#define HS_PLATFORM_ALL (0xabcdef11abcdef00ULL)
#endif

namespace {

/* Stream state size for the forged Gough NFA. */
const u32 GOUGH_STREAM_STATE = 128;

/* rose->stateOffsets.end for the forged database. */
const u32 FORGED_STATE_END = 256;

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
 * Build a forged streaming-mode database holding a single Gough outfix.
 *
 * The Gough engine is vulnerable when stream_som_loc_count is forged to exceed
 * the actual SOM storage allocated in the stream state.
 */
static hs_database_t *make_forged_gough_db(u32 stream_som_loc_count) {
    const size_t bytecode_offset = ROUNDUP_N(sizeof(hs_database_t), 64);
    const size_t nfa_info_offset =
        ROUNDUP_N(sizeof(struct RoseEngine), alignof(struct NfaInfo));
    const size_t nfa_offset =
        ROUNDUP_N(nfa_info_offset + sizeof(struct NfaInfo), 64);

    /* Minimal Gough NFA structure: NFA + mcclellan + gough_info */
    const size_t gough_body = sizeof(struct mcclellan) + sizeof(struct gough_info);

    size_t min_rose =
        ROUNDUP_N(nfa_offset + sizeof(struct NFA) + gough_body, 64);
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
    memset(nfa, 0, sizeof(struct NFA) + gough_body);
    nfa->type = GOUGH_NFA_8;
    nfa->length = static_cast<u32>(sizeof(struct NFA) + gough_body);
    nfa->queueIndex = 0;
    nfa->scratchStateSize = 64;
    nfa->streamStateSize = GOUGH_STREAM_STATE;

    /* Forge the mcclellan+gough structure with excessive stream_som_loc_count */
    struct mcclellan *m = reinterpret_cast<struct mcclellan *>(
        reinterpret_cast<char *>(nfa) + sizeof(struct NFA));
    memset(m, 0, sizeof(*m));
    m->state_count = 8;
    m->start_anchored = 0;
    m->start_floating = 0;
    m->alphaShift = 3;
    /* haig_offset is relative to the start of the NFA, not the mcclellan. */
    m->haig_offset = sizeof(struct NFA) + sizeof(struct mcclellan);

    struct gough_info *gi = reinterpret_cast<struct gough_info *>(
        reinterpret_cast<char *>(m) + sizeof(struct mcclellan));
    memset(gi, 0, sizeof(*gi));
    gi->stream_som_loc_width = 4;
    /* FORGE: Set stream_som_loc_count to exceed allocated stream state */
    gi->stream_som_loc_count = stream_som_loc_count;

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
        /* Deliberately do NOT scan with this database here: in a vulnerable
         * build that could trigger the heap buffer overflow. */
        hs_free_database(db2);
        FAIL() << "hs_deserialize_database accepted forged Gough database ("
               << what << "); stream_som_loc_count field is unvalidated";
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
        FAIL() << "hs_alloc_scratch accepted forged Gough database (" << what
               << "); stream_som_loc_count field is unvalidated";
    }

    EXPECT_EQ(HS_INVALID, err) << what;
    EXPECT_EQ(nullptr, scratch) << what;
}

/* SOM-leftmost "(foobar)+" compiles to a Gough outfix in streaming mode. */
static hs_database_t *compile_gough_db() {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("(foobar)+", HS_FLAG_SOM_LEFTMOST,
                                HS_MODE_STREAM | HS_MODE_SOM_HORIZON_LARGE,
                                nullptr, &db, &compile_err);
    if (err != HS_SUCCESS) {
        hs_free_compile_error(compile_err);
        return nullptr;
    }
    return db;
}

/* Return the first Gough NFA in the Rose bytecode, or nullptr. */
static const struct NFA *find_gough_nfa(const char *bytecode) {
    const struct RoseEngine *rose =
        reinterpret_cast<const struct RoseEngine *>(bytecode);
    const struct NfaInfo *infos = reinterpret_cast<const struct NfaInfo *>(
        bytecode + rose->nfaInfoOffset);
    for (u32 qi = 0; qi < rose->queueCount; qi++) {
        const struct NFA *nfa = reinterpret_cast<const struct NFA *>(
            bytecode + infos[qi].nfaOffset);
        if (isGoughType(nfa->type)) {
            return nfa;
        }
    }
    return nullptr;
}

using GoughMutator =
    std::function<void(struct NFA *, struct mcclellan *, struct gough_info *)>;

/**
 * Serialize a real Gough database, apply \a mutate to its Gough NFA, re-sign
 * it and return the result of hs_deserialize_database().
 */
static hs_error_t deserialize_mutated_gough(const GoughMutator &mutate) {
    hs_database_t *db = compile_gough_db();
    if (!db) {
        ADD_FAILURE() << "failed to compile Gough pattern";
        return HS_UNKNOWN_ERROR;
    }
    char *bytes = nullptr;
    size_t len = 0;
    hs_error_t err = hs_serialize_database(db, &bytes, &len);
    hs_free_database(db);
    if (err != HS_SUCCESS) {
        ADD_FAILURE() << "hs_serialize_database failed: " << err;
        return err;
    }

    /* Serialized layout: magic, version, length, platform, hmac, hmac_hdr,
     * then the bytecode. Work on an aligned copy of the bytecode. */
    const size_t hmac_pos = 4 + 4 + 4 + 8;
    const size_t hdr_len = hmac_pos + 32 + 32;
    u32 body_len = 0;
    memcpy(&body_len, bytes + 8, sizeof(body_len));
    std::vector<u64a> body((body_len + sizeof(u64a) - 1) / sizeof(u64a));
    char *bc = reinterpret_cast<char *>(body.data());
    memcpy(bc, bytes + hdr_len, body_len);

    struct NFA *nfa = const_cast<struct NFA *>(find_gough_nfa(bc));
    if (!nfa) {
        free(bytes);
        ADD_FAILURE() << "compiled database contains no Gough NFA";
        return HS_UNKNOWN_ERROR;
    }
    struct mcclellan *m = reinterpret_cast<struct mcclellan *>(
        reinterpret_cast<char *>(nfa) + sizeof(struct NFA));
    struct gough_info *gi = reinterpret_cast<struct gough_info *>(
        reinterpret_cast<char *>(nfa) + m->haig_offset);
    mutate(nfa, m, gi);

    memcpy(bytes + hdr_len, bc, body_len);
    unsigned int hmac_len = 32;
    HMAC(EVP_sha256(), HS_DB_HMAC_KEY, sizeof(HS_DB_HMAC_KEY),
         reinterpret_cast<const unsigned char *>(bc), body_len,
         reinterpret_cast<unsigned char *>(bytes + hmac_pos), &hmac_len);

    hs_database_t *out = nullptr;
    err = hs_deserialize_database(bytes, len, &out);
    free(bytes);
    hs_free_database(out);
    return err;
}

// ============================================================================
// Test 1: stream_som_loc_count far exceeds stream state allocation.
// With only 128 bytes of stream state and ~32 bytes of overhead, there's no
// room for many SOM slots. Forging stream_som_loc_count = 1000 would attempt
// to access som->slots[0..999], causing a heap buffer overflow in
// expandSomSpace() and compSomSpace().
// ============================================================================
TEST(GoughSomLocCountOob, DeserializeRejectsForgedSomLocCount_Excessive) {
    hs_database_t *db = make_forged_gough_db(1000);
    expect_deserialize_rejected(db, "stream_som_loc_count=1000 (excessive)");
    expect_alloc_scratch_rejected(db, "stream_som_loc_count=1000 (excessive)");
    free_forged_db(db);
}

// ============================================================================
// Test 2: stream_som_loc_count moderately exceeds stream state allocation.
// Forging stream_som_loc_count = 64 might still exceed the actual allocation
// in the 128-byte stream state.
// ============================================================================
TEST(GoughSomLocCountOob, DeserializeRejectsForgedSomLocCount_Moderate) {
    hs_database_t *db = make_forged_gough_db(64);
    expect_deserialize_rejected(db, "stream_som_loc_count=64 (moderate overflow)");
    expect_alloc_scratch_rejected(db, "stream_som_loc_count=64 (moderate overflow)");
    free_forged_db(db);
}

// ============================================================================
// Test 3: stream_som_loc_count equals a valid but suspicious value.
// stream_som_loc_count = 32 might still overflow depending on the actual
// SOM allocation in the stream state.
// ============================================================================
TEST(GoughSomLocCountOob, DeserializeRejectsForgedSomLocCount_Boundary) {
    hs_database_t *db = make_forged_gough_db(32);
    expect_deserialize_rejected(db, "stream_som_loc_count=32 (boundary case)");
    expect_alloc_scratch_rejected(db, "stream_som_loc_count=32 (boundary case)");
    free_forged_db(db);
}

// ============================================================================
// Test 4: Baseline - a legitimately compiled Gough database must still be
// accepted and scan. The pattern is asserted to produce a Gough NFA so this
// acts as a positive regression guard for the validator.
// ============================================================================
TEST(GoughSomLocCountOob, ValidGoughPatternCompiles) {
    hs_database_t *db = compile_gough_db();
    ASSERT_NE(nullptr, db);
    ASSERT_NE(nullptr, find_gough_nfa(
        static_cast<const char *>(hs_get_bytecode(db))))
        << "pattern no longer compiles to a Gough NFA";

    size_t ser_len = 0;
    char *ser_bytes = nullptr;
    hs_error_t err = hs_serialize_database(db, &ser_bytes, &ser_len);
    hs_free_database(db);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(nullptr, ser_bytes);

    hs_database_t *db2 = nullptr;
    err = hs_deserialize_database(ser_bytes, ser_len, &db2);
    free(ser_bytes);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(nullptr, db2);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db2, &scratch);
    EXPECT_EQ(HS_SUCCESS, err);

    hs_stream_t *stream = nullptr;
    if (scratch) {
        err = hs_open_stream(db2, 0, &stream);
        EXPECT_EQ(HS_SUCCESS, err);
    }

    if (stream) {
        int matches = 0;
        auto match_callback = [](unsigned int, unsigned long long,
                                 unsigned long long, unsigned int,
                                 void *context) -> int {
            (*static_cast<int *>(context))++;
            return 0;
        };

        /* Split across writes so SOM slots are compressed/expanded. */
        err = hs_scan_stream(stream, "xxfoo", 5, 0, scratch, match_callback,
                             &matches);
        EXPECT_EQ(HS_SUCCESS, err);
        err = hs_scan_stream(stream, "barfoobar", 9, 0, scratch,
                             match_callback, &matches);
        EXPECT_EQ(HS_SUCCESS, err);

        hs_close_stream(stream, scratch, match_callback, &matches);
        EXPECT_LT(0, matches);
    }

    hs_free_scratch(scratch);
    hs_free_database(db2);
}

// ============================================================================
// Tests 5-9: mutate a single field of a real compiled Gough database (so every
// other validator passes) and check the Gough validator alone decides.
// ============================================================================
TEST(GoughSomLocCountOob, RealGough_Unmodified_Accepted) {
    EXPECT_EQ(HS_SUCCESS, deserialize_mutated_gough(
        [](struct NFA *, struct mcclellan *, struct gough_info *) {}));
}

TEST(GoughSomLocCountOob, RealGough_ZeroHaigOffset_Rejected) {
    EXPECT_EQ(HS_INVALID, deserialize_mutated_gough(
        [](struct NFA *, struct mcclellan *m, struct gough_info *) {
            m->haig_offset = 0;
        }));
}

TEST(GoughSomLocCountOob, RealGough_OutOfRangeHaigOffset_Rejected) {
    EXPECT_EQ(HS_INVALID, deserialize_mutated_gough(
        [](struct NFA *nfa, struct mcclellan *m, struct gough_info *) {
            m->haig_offset = nfa->length;
        }));
}

TEST(GoughSomLocCountOob, RealGough_UnsupportedWidth_Rejected) {
    /* 0 is only legitimate in block mode; this database is streaming. */
    for (u8 width : {0, 1, 3, 5, 6, 7}) {
        SCOPED_TRACE(width);
        EXPECT_EQ(HS_INVALID, deserialize_mutated_gough(
            [width](struct NFA *, struct mcclellan *, struct gough_info *gi) {
                gi->stream_som_loc_width = width;
            }));
    }
}

TEST(GoughSomLocCountOob, RealGough_StreamSlotsOverrun_Rejected) {
    EXPECT_EQ(HS_INVALID, deserialize_mutated_gough(
        [](struct NFA *nfa, struct mcclellan *, struct gough_info *gi) {
            /* One more slot than the stream state can hold. */
            u32 base = nfa->type == GOUGH_NFA_8 ? 1 : 2;
            gi->stream_som_loc_count =
                (nfa->streamStateSize - base) / gi->stream_som_loc_width + 1;
        }));
}

TEST(GoughSomLocCountOob, RealGough_ScratchSlotsOverrun_Rejected) {
    EXPECT_EQ(HS_INVALID, deserialize_mutated_gough(
        [](struct NFA *nfa, struct mcclellan *, struct gough_info *gi) {
            /* Stream slots still fit; scratch is one byte short. */
            nfa->scratchStateSize =
                16 + gi->stream_som_loc_count * (u32)sizeof(u64a) - 1;
        }));
}

} // namespace
