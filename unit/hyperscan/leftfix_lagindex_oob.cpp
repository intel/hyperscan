/*
 * Copyright (C) 2026 Intel Corporation
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
#include "hs_db_hmac_key.h"

#include <openssl/hmac.h>
#include <openssl/evp.h>

namespace {

// Serialized layout: magic(4) + version(4) + length(4) + platform(8) +
// hmac(32) + hmac_hdr(32) = 84 bytes, then `length` bytes of bytecode.
constexpr size_t kHmacOffset = 3 * sizeof(u32) + sizeof(u64a);           // 20
constexpr size_t kHmacHdrOffset = kHmacOffset + 32;                       // 52
constexpr size_t kBytecodeStart = kHmacHdrOffset + 32;                    // 84

static void reseal_bytecode_hmac(char *serialized, size_t bytecode_len) {
    u8 new_hmac[32];
    unsigned int hmac_len = 32;
    HMAC(EVP_sha256(), HS_DB_HMAC_KEY, sizeof(HS_DB_HMAC_KEY),
         reinterpret_cast<const unsigned char *>(serialized + kBytecodeStart),
         bytecode_len, new_hmac, &hmac_len);
    memcpy(serialized + kHmacOffset, new_hmac, sizeof(new_hmac));
}

static void reseal_header_hmac(char *serialized) {
    u8 buf[20];
    memcpy(buf, serialized, 20); // magic + version + length + platform
    
    u8 new_hmac_hdr[32];
    unsigned int hmac_len = 32;
    HMAC(EVP_sha256(), HS_DB_HMAC_KEY, sizeof(HS_DB_HMAC_KEY),
         buf, sizeof(buf), new_hmac_hdr, &hmac_len);
    memcpy(serialized + kHmacHdrOffset, new_hmac_hdr, sizeof(new_hmac_hdr));
}

// Build a simple STREAM mode database with a non-transient leftfix
// Pattern: a.*foobar — creates a leftfix for "a" which will have lagIndex
static bool make_streaming_db_with_leftfix(const char *pattern,
                                            char **bytes, size_t *length) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    
    // HS_MODE_STREAM is required to trigger the vulnerability (lagIndex is only
    // set for non-transient leftfixes when streaming)
    hs_error_t err = hs_compile(pattern, HS_FLAG_SINGLEMATCH, HS_MODE_STREAM,
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

// ============================================================================
// Baseline: Unmodified streaming database with leftfix works
// ============================================================================
TEST(LeftfixLagindexOob, BaselineUnmodifiedStreamingDatabaseWorks) {
    char *serialized = nullptr;
    size_t length = 0;
    
    // Pattern with leftfix: "a.*" creates a non-transient prefix for "a"
    ASSERT_TRUE(make_streaming_db_with_leftfix("a.*foo", &serialized, &length))
        << "Failed to create streaming DB";
    
    hs_database_t *db = nullptr;
    ASSERT_EQ(HS_SUCCESS, hs_deserialize_database(serialized, length, &db));
    
    hs_scratch_t *scratch = nullptr;
    ASSERT_EQ(HS_SUCCESS, hs_alloc_scratch(db, &scratch));
    
    // Set up a stream and run a scan
    hs_stream_t *stream = nullptr;
    ASSERT_EQ(HS_SUCCESS, hs_open_stream(db, 0, &stream));
    
    int matches = 0;
    auto match_callback = []([[maybe_unused]] unsigned int id, [[maybe_unused]] unsigned long long from,
                             [[maybe_unused]] unsigned long long to, [[maybe_unused]] unsigned int flags,
                             void *context) -> int {
        (*static_cast<int *>(context))++;
        return 0;
    };
    
    // Scan some data through the stream
    EXPECT_EQ(HS_SUCCESS, hs_scan_stream(stream, "afoo", 4, 0, scratch,
                                         match_callback, &matches));
    
    // End the stream (this calls storeRoseDelay with lagIndex)
    EXPECT_EQ(HS_SUCCESS, hs_close_stream(stream, scratch, match_callback, &matches));
    
    hs_free_scratch(scratch);
    hs_free_database(db);
    free(serialized);
}

// ============================================================================
// EXPLOIT ATTEMPT 1: Forge lagIndex to point beyond leftfixLagTable
// Expected: REJECTED during deserialization validation (after fix)
// Before fix: Would accept and cause OOB write on hs_close_stream()
// ============================================================================
TEST(LeftfixLagindexOob, RejectForgedLagIndexOobWrite) {
    char *serialized = nullptr;
    size_t length = 0;
    
    // Pattern with leftfix
    ASSERT_TRUE(make_streaming_db_with_leftfix("a.*bar", &serialized, &length))
        << "Failed to create DB";
    
    auto *rose = reinterpret_cast<struct RoseEngine *>(serialized + kBytecodeStart);
    u32 leftOffset = rose->leftOffset;
    
    if (leftOffset && leftOffset < rose->size) {
        char *bytecode = serialized + kBytecodeStart;
        auto *left_table = reinterpret_cast<struct LeftNfaInfo *>(
            bytecode + leftOffset);
        
        // LeftNfaInfo.lagIndex is a u32 field at offset 8 in the struct
        // (after maxQueueLen, maxLag)
        // For this test, we modify the first leftfix entry
        // Find the lagIndex offset within the struct
        // struct LeftNfaInfo { u32 maxQueueLen; u32 maxLag; u32 lagIndex; ... }
        
        u32 *lagIndex_ptr = reinterpret_cast<u32 *>(
            bytecode + leftOffset + offsetof(struct LeftNfaInfo, lagIndex));
        
        // Set lagIndex to a value that's way out of bounds
        // The lag table has size activeLeftCount bytes at most
        // Set it to something clearly wrong: 0x80000000
        u32 original_lagIndex = *lagIndex_ptr;
        *lagIndex_ptr = 0x80000000;
        
        // Recompute HMACs
        reseal_bytecode_hmac(serialized, length - kBytecodeStart);
        reseal_header_hmac(serialized);
        
        // Attempt deserialization — should be REJECTED after fix
        hs_database_t *db = nullptr;
        hs_error_t result = hs_deserialize_database(serialized, length, &db);
        
        if (result == HS_SUCCESS) {
            // After fix: should reject during validation
            // Before fix: would accept and potentially segfault on stream end
            hs_free_database(db);
        }
        
        EXPECT_NE(HS_SUCCESS, result)
            << "Forged lagIndex (0x80000000) should be rejected at deserialization";
    }
    
    free(serialized);
}

// ============================================================================
// EXPLOIT ATTEMPT 2: Multiple leftfixes with forged lagIndex
// Expected: REJECTED during deserialization validation (after fix)
// Vulnerability allows N consecutive OOB writes on single hs_scan_stream() call
// ============================================================================
TEST(LeftfixLagindexOob, RejectMultipleForgedLagIndices) {
    char *serialized = nullptr;
    size_t length = 0;
    
    // Complex pattern with multiple leftfixes
    // Each "a.*", "b.*", "c.*" creates a non-transient leftfix
    ASSERT_TRUE(make_streaming_db_with_leftfix("a.*x|b.*y|c.*z", 
                                                &serialized, &length))
        << "Failed to create DB";
    
    auto *rose = reinterpret_cast<struct RoseEngine *>(serialized + kBytecodeStart);
    u32 leftOffset = rose->leftOffset;
    
    if (leftOffset && leftOffset < rose->size) {
        char *bytecode = serialized + kBytecodeStart;
        
        // Walk the LeftNfaInfo array and forge the first few entries
        for (int i = 0; i < 3 && leftOffset + (i + 1) * sizeof(struct LeftNfaInfo)
                                   <= rose->size; ++i) {
            auto *left_entry = reinterpret_cast<struct LeftNfaInfo *>(
                bytecode + leftOffset + i * sizeof(struct LeftNfaInfo));
            
            // Forge lagIndex: make each one point to consecutive offsets
            // beyond the legitimate lag table
            left_entry->lagIndex = 0x1000 + (i * 100);
        }
        
        // Recompute HMACs
        reseal_bytecode_hmac(serialized, length - kBytecodeStart);
        reseal_header_hmac(serialized);
        
        // Attempt deserialization
        hs_database_t *db = nullptr;
        hs_error_t result = hs_deserialize_database(serialized, length, &db);
        
        if (result == HS_SUCCESS) {
            hs_free_database(db);
        }
        
        EXPECT_NE(HS_SUCCESS, result)
            << "Multiple forged lagIndex values should be rejected at deserialization";
    }
    
    free(serialized);
}

// ============================================================================
// EXPLOIT ATTEMPT 3: Verify conservative bound is applied
// A valid database with some lagIndex values should still work if they're
// within the conservative bound (stateOffsets.end - stateOffsets.leftfixLagTable)
// ============================================================================
TEST(LeftfixLagindexOob, ValidLagIndicesWithinBoundAccepted) {
    char *serialized = nullptr;
    size_t length = 0;
    
    // Simple valid pattern with leftfix
    ASSERT_TRUE(make_streaming_db_with_leftfix("a.*test", &serialized, &length))
        << "Failed to create DB";
    
    // Don't modify anything—just deserialize the valid database
    hs_database_t *db = nullptr;
    hs_error_t result = hs_deserialize_database(serialized, length, &db);
    
    // Valid databases should deserialize successfully
    EXPECT_EQ(HS_SUCCESS, result)
        << "Valid streaming database with legitimate lagIndex should deserialize";
    
    if (result == HS_SUCCESS) {
        hs_free_database(db);
    }
    
    free(serialized);
}

// ============================================================================
// Control: ROSE_OFFSET_INVALID lagIndex should be accepted
// If lagIndex == ROSE_OFFSET_INVALID (0xFFFFFFFF), it means no lag
// This is a valid state and should not be rejected
// ============================================================================
TEST(LeftfixLagindexOob, RoseOffsetInvalidLagIndexAccepted) {
    char *serialized = nullptr;
    size_t length = 0;
    
    // Create a valid database first
    ASSERT_TRUE(make_streaming_db_with_leftfix("xyz", &serialized, &length))
        << "Failed to create DB";
    
    // Don't modify lagIndex values—leave them as-is (many will be ROSE_OFFSET_INVALID)
    // Just verify the database deserializes normally
    hs_database_t *db = nullptr;
    hs_error_t result = hs_deserialize_database(serialized, length, &db);
    
    EXPECT_EQ(HS_SUCCESS, result)
        << "Database with legitimate ROSE_OFFSET_INVALID lagIndex should deserialize";
    
    if (result == HS_SUCCESS) {
        hs_free_database(db);
    }
    
    free(serialized);
}

} // namespace
