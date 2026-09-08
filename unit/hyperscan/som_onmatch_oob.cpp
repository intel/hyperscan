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

#include "gtest/gtest.h"

#include "hs.h"
#include "hs_compile.h"
#include "hs_runtime.h"
#include "database.h"
#include "ue2common.h"
#include "rose/rose_program.h"
#include "som/som_operation.h"
#include "hs_db_hmac_key.h"

#include <openssl/hmac.h>
#include <openssl/evp.h>

namespace {

// Layout written by hs_serialize_database() / read by db_decode_header():
// magic(4) + version(4) + length(4) + platform(8) + hmac(32) + hmac_hdr(32)
// = 84 bytes of header, followed by `length` bytes of bytecode.
constexpr size_t kMagicVersionLengthPlatform = 3 * sizeof(u32) + sizeof(u64a);
constexpr size_t kHmacOffset = kMagicVersionLengthPlatform;         // 20
constexpr size_t kHmacHdrOffset = kHmacOffset + 32;                 // 52
constexpr size_t kBytecodeStart = kHmacHdrOffset + 32;              // 84

// Recompute the bytecode-content HMAC (header fields are untouched by this
// test, so hmac_hdr does not need to be recomputed).
static void reseal_bytecode_hmac(char *serialized, size_t bytecode_len) {
    u8 new_hmac[32];
    unsigned int hmac_len = 32;
    HMAC(EVP_sha256(), HS_DB_HMAC_KEY, sizeof(HS_DB_HMAC_KEY),
         reinterpret_cast<const unsigned char *>(serialized + kBytecodeStart),
         bytecode_len, new_hmac, &hmac_len);
    memcpy(serialized + kHmacOffset, new_hmac, sizeof(new_hmac));
}

// Scan the serialized bytecode for a REPORT_SOM_INT instruction whose
// `onmatch` operand currently names a valid (in-bounds) SOM slot, mirroring
// the byte-level search used by the original PoC. Returns the byte offset
// (within `serialized`) of the `onmatch` field, or 0 if none was found.
static size_t find_report_som_int_onmatch(char *serialized, size_t total_len,
                                          u32 som_store_count) {
    // struct ROSE_STRUCT_REPORT_SOM_INT { u8 code; struct som_operation som; }
    // som_operation { u8 type; u32 onmatch (at +4 within som, so +8 overall
    // once the leading `code` byte's alignment padding is accounted for);
    // union { u64a } aux; }, alignof(som_operation) == 8.
    const size_t instr_size = 24; // code + pad(7) + type + pad(3) + onmatch + aux
    for (size_t i = 0; i + instr_size <= total_len; i++) {
        auto *p = reinterpret_cast<unsigned char *>(serialized + i);
        if (p[0] != ROSE_INSTR_REPORT_SOM_INT) {
            continue;
        }
        unsigned char type = p[8];
        if (type < SOM_EXTERNAL_CALLBACK_REL ||
            type > SOM_INTERNAL_LOC_SET_FROM_IF_WRITABLE) {
            continue;
        }
        u32 onmatch;
        memcpy(&onmatch, p + 12, sizeof(onmatch));
        if (onmatch < som_store_count) {
            return i + 12;
        }
    }
    return 0;
}

struct CallbackCtx {
    int match_count = 0;
};

static int onMatch(unsigned int, unsigned long long, unsigned long long,
                   unsigned int, void *ctx) {
    static_cast<CallbackCtx *>(ctx)->match_count++;
    return 0;
}

// ============================================================================
// Baseline / false-positive control: the unmodified, legitimately compiled
// SOM database must scan cleanly and report a match.
// ============================================================================
TEST(SomOnmatchOob, BaselineUnmodifiedDatabaseScansCleanly) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foo+", HS_FLAG_SOM_LEFTMOST, HS_MODE_BLOCK,
                                nullptr, &db, &compile_err);
    if (err != HS_SUCCESS) {
        if (compile_err) {
            hs_free_compile_error(compile_err);
        }
        return;
    }

    hs_scratch_t *scratch = nullptr;
    ASSERT_EQ(HS_SUCCESS, hs_alloc_scratch(db, &scratch));

    CallbackCtx ctx;
    const char *input = "xxfoooyy";
    err = hs_scan(db, input, static_cast<unsigned int>(strlen(input)), 0,
                  scratch, onMatch, &ctx);
    EXPECT_EQ(HS_SUCCESS, err);
    EXPECT_GT(ctx.match_count, 0);

    hs_free_scratch(scratch);
    hs_free_database(db);
}

// ============================================================================
// Exploit: forge the `onmatch` operand of a REPORT_SOM_INT instruction to an
// out-of-range SOM slot, re-sign the HMAC with the public key, deserialize,
// and scan matching input.
//
// Without the fix in som_runtime.c, this drives:
//     som_store[onmatch] = start_offset;
// with `onmatch` far outside the `somLocationCount`-sized scratch array --
// an attacker-controlled heap out-of-bounds write (CWE-787). ASan builds
// abort here before the fix; this is the "fails without the fix" half of
// this regression test.
// ============================================================================
TEST(SomOnmatchOob, ForgedOnmatchOperandHandledSafely) {
    hs_database_t *real_db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err = hs_compile("foo+", HS_FLAG_SOM_LEFTMOST, HS_MODE_BLOCK,
                                nullptr, &real_db, &compile_err);
    if (err != HS_SUCCESS) {
        if (compile_err) {
            hs_free_compile_error(compile_err);
        }
        return;
    }

    char *serialized = nullptr;
    size_t ser_len = 0;
    err = hs_serialize_database(real_db, &serialized, &ser_len);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_NE(nullptr, serialized);

    // Learn the real somLocationCount / bytecode length via a clean
    // deserialize, so we know both the valid slot range and the bytecode
    // length to re-hash.
    hs_database_t *clean_db = nullptr;
    ASSERT_EQ(HS_SUCCESS,
              hs_deserialize_database(serialized, ser_len, &clean_db));
    u32 bytecode_len = clean_db->length;
    hs_free_database(clean_db);
    hs_free_database(real_db);

    // A simple pattern like "foo+" needs only a handful of SOM slots, so
    // restrict the search to instructions whose current `onmatch` names a
    // slot in that small range -- this is what makes the match specific to
    // the real REPORT_SOM_INT instruction rather than incidental bytes.
    size_t onmatch_off = find_report_som_int_onmatch(serialized, ser_len, 16u);
    ASSERT_NE(static_cast<size_t>(0), onmatch_off)
        << "could not locate a REPORT_SOM_INT instruction to forge; "
           "pattern/build may have changed the generated program";

    // Patch onmatch to a value far outside any plausible somLocationCount.
    u32 poison_onmatch = 0x10000u;
    memcpy(serialized + onmatch_off, &poison_onmatch, sizeof(poison_onmatch));

    // Re-sign the content HMAC exactly as an attacker with the public key
    // (hs_db_hmac_key.h) would.
    reseal_bytecode_hmac(serialized, bytecode_len);

    hs_database_t *forged_db = nullptr;
    err = hs_deserialize_database(serialized, ser_len, &forged_db);
    ASSERT_EQ(HS_SUCCESS, err)
        << "forged blob was rejected before reaching the vulnerable runtime "
           "path -- validators now cover this field, nothing left to test";

    hs_scratch_t *scratch = nullptr;
    ASSERT_EQ(HS_SUCCESS, hs_alloc_scratch(forged_db, &scratch));

    CallbackCtx ctx;
    const char *input = "xxfoooyy";
    // Before the fix: attacker-controlled OOB heap write inside setSomLoc()
    // (som_runtime.c) -- aborts under ASan.
    // After the fix: the out-of-range operand is detected and the SOM
    // operation is safely dropped; the scan completes normally.
    err = hs_scan(forged_db, input, static_cast<unsigned int>(strlen(input)),
                  0, scratch, onMatch, &ctx);
    EXPECT_EQ(HS_SUCCESS, err);

    hs_free_scratch(scratch);
    hs_free_database(forged_db);
    free(serialized);
}

} // namespace
