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

/**
 * \file
 * \brief Regression tests for the state_init scatter plan (CWE-787 / CWE-125).
 *
 * roseInitState() -> init_state() -> scatter() (util/scatter_runtime.h) reads
 * a table of {offset, val} entries from `rose + s_<type>_offset` and writes
 * each one into the freshly allocated per-stream state buffer:
 *
 *     storeu_u8((char *)out + item->offset, item->val);
 *
 * Two distinct fields are attacker controlled in a forged database:
 *
 *  1. each entry's destination `offset`, which must stay within
 *     stateOffsets.end (the size of the state allocation), and
 *  2. the location of the table itself (`s_<type>_offset` / `_count`), which
 *     must stay within the Rose blob so that merely *validating* the entries
 *     does not itself read out of bounds.
 *
 * validateStateLayout() is reached both from db_validate_rose_offsets() at
 * deserialize time and directly from hs_open_stream() / hs_scan_vector(), and
 * the latter paths do not run the deserializer's VALIDATE_SCATTER. Both the
 * table bounds and the per-entry destination therefore have to be checked
 * inside validateStateLayout() itself.
 *
 * These tests start from a real, fully valid compiled streaming database and
 * mutate exactly one field of the in-memory engine, so every other field
 * stays legitimate and validateStateLayout() cannot reject the database for
 * an unrelated reason. The baseline test pins that property down.
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
#include "util/scatter.h"

namespace {

// Hyperscan mmaps database memory and marks it PROT_READ once the HMAC has
// been computed (src/db_protect.c). Installing a custom database allocator
// opts out of that protection, which is what lets these tests model a forged
// engine by mutating it in place. RAII so the global allocator is always
// restored, even on assertion failure.
static void *test_db_alloc(size_t n) { return malloc(n); }
static void test_db_free(void *p) { free(p); }

class WritableDbAllocator {
public:
    WritableDbAllocator() {
        hs_set_database_allocator(test_db_alloc, test_db_free);
    }
    ~WritableDbAllocator() { hs_set_database_allocator(nullptr, nullptr); }
};

// Compile a real streaming database. Returns nullptr if unsupported.
static hs_database_t *build_stream_db() {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("hatstand.*teakettle", 0, HS_MODE_STREAM,
                                nullptr, &db, &compile_err);
    if (err != HS_SUCCESS) {
        if (compile_err) {
            hs_free_compile_error(compile_err);
        }
        return nullptr;
    }
    return db;
}

// Models a forged database that has already been loaded: mutate the engine in
// place rather than re-signing a serialized blob.
static struct RoseEngine *mutable_rose(hs_database_t *db) {
    return const_cast<struct RoseEngine *>(
        static_cast<const struct RoseEngine *>(hs_get_bytecode(db)));
}

// ============================================================================
// Baseline / false-positive control: the untouched database must open a
// stream successfully, so an HS_INVALID below is attributable to the mutation
// rather than to some unrelated validation failure.
// ============================================================================
TEST(ScatterDestOob, BaselineUnmodifiedDatabaseOpensStream) {
    WritableDbAllocator alloc_guard;
    hs_database_t *db = build_stream_db();
    if (!db) {
        return; // unsupported in this build
    }

    hs_stream_t *stream = nullptr;
    EXPECT_EQ(HS_SUCCESS, hs_open_stream(db, 0, &stream));
    EXPECT_NE(nullptr, stream);

    if (stream) {
        hs_close_stream(stream, nullptr, nullptr, nullptr);
    }
    hs_free_database(db);
}

// ============================================================================
// Forge one scatter entry's destination offset past the end of the per-stream
// state buffer.
//
// Before the fix: hs_open_stream() succeeds and roseInitState() writes past
// the stream state allocation.
// After the fix: validateStateLayout() rejects it with HS_INVALID.
// ============================================================================
TEST(ScatterDestOob, OpenStreamRejectsForgedScatterDestOffset) {
    WritableDbAllocator alloc_guard;
    hs_database_t *db = build_stream_db();
    if (!db) {
        return; // unsupported in this build
    }

    struct RoseEngine *rose = mutable_rose(db);

    // The compiler populates the u8 sub-plan for this pattern; the other
    // widths are empty. Assert rather than skip so this test cannot silently
    // become a no-op if the generated plan changes.
    ASSERT_NE(0u, rose->state_init.s_u8_offset)
        << "no u8 scatter entries in this build; test would be vacuous";
    ASSERT_NE(0u, rose->state_init.s_u8_count);

    auto *tbl = reinterpret_cast<struct scatter_unit_u8 *>(
        reinterpret_cast<char *>(rose) + rose->state_init.s_u8_offset);

    // Sanity: the untouched entry must be a legitimate in-range destination.
    ASSERT_LE(tbl[0].offset, rose->stateOffsets.end);

    u32 state_end = rose->stateOffsets.end;
    tbl[0].offset = 0x100000; // far past stateOffsets.end

    hs_stream_t *stream = nullptr;
    hs_error_t err = hs_open_stream(db, 0, &stream);

    EXPECT_EQ(HS_INVALID, err)
        << "forged scatter destination offset was accepted; roseInitState() "
           "would write past the stream state allocation (end=" << state_end
        << ")";

    if (stream) {
        hs_close_stream(stream, nullptr, nullptr, nullptr);
    }
    hs_free_database(db);
}

// ============================================================================
// Forge the location of the scatter table itself.
//
// hs_open_stream() reaches validateStateLayout() without the deserializer's
// VALIDATE_SCATTER having run, so the entry traversal must bound the table
// (offset, alignment, count, offset + count * sizeof(entry) <= rose->size)
// before dereferencing it. Otherwise the validation loop itself reads far
// outside the Rose blob -- an out-of-bounds read that a plain build may not
// fault on, but that AddressSanitizer reports deterministically.
// ============================================================================
TEST(ScatterDestOob, OpenStreamRejectsForgedScatterTableBounds) {
    WritableDbAllocator alloc_guard;
    hs_database_t *db = build_stream_db();
    if (!db) {
        return; // unsupported in this build
    }

    struct RoseEngine *rose = mutable_rose(db);

    ASSERT_NE(0u, rose->state_init.s_u8_offset)
        << "no u8 scatter entries in this build; test would be vacuous";

    // Table pushed past the end of the engine blob, with a large entry count
    // so any unbounded traversal walks far outside the allocation.
    u32 rose_size = rose->size;
    rose->state_init.s_u8_offset = rose_size + 0x10000;
    rose->state_init.s_u8_count = 0x10000;

    hs_stream_t *stream = nullptr;
    hs_error_t err = hs_open_stream(db, 0, &stream);

    EXPECT_EQ(HS_INVALID, err)
        << "forged scatter table location was accepted; the validation loop "
           "would dereference outside the Rose blob (rose->size=" << rose_size
        << ")";

    if (stream) {
        hs_close_stream(stream, nullptr, nullptr, nullptr);
    }
    hs_free_database(db);
}

// ============================================================================
// A non-zero table offset with a zero count must be rejected: scatter()
// asserts on that combination, so release builds must not reach it.
// ============================================================================
TEST(ScatterDestOob, OpenStreamRejectsScatterOffsetWithZeroCount) {
    WritableDbAllocator alloc_guard;
    hs_database_t *db = build_stream_db();
    if (!db) {
        return; // unsupported in this build
    }

    struct RoseEngine *rose = mutable_rose(db);

    ASSERT_NE(0u, rose->state_init.s_u8_offset)
        << "no u8 scatter entries in this build; test would be vacuous";

    rose->state_init.s_u8_count = 0;

    hs_stream_t *stream = nullptr;
    hs_error_t err = hs_open_stream(db, 0, &stream);

    EXPECT_EQ(HS_INVALID, err)
        << "scatter table with non-zero offset and zero count was accepted";

    if (stream) {
        hs_close_stream(stream, nullptr, nullptr, nullptr);
    }
    hs_free_database(db);
}

} // namespace
