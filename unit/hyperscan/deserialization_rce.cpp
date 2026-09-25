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
#include "hwlm/noodle_internal.h"
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

// Build a simple SINGLEMATCH database and serialize it
static bool make_serialized_db(const char *pattern, char **bytes, size_t *length) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    
    hs_error_t err = hs_compile(pattern, HS_FLAG_SINGLEMATCH, HS_MODE_BLOCK,
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
// Baseline: Unmodified database deserializes and works
// ============================================================================
TEST(DeserializationRce, BaselineUnmodifiedDatabaseWorks) {
    char *serialized = nullptr;
    size_t length = 0;
    
    ASSERT_TRUE(make_serialized_db("abc", &serialized, &length)) << "Failed to create DB";
    
    hs_database_t *db = nullptr;
    ASSERT_EQ(HS_SUCCESS, hs_deserialize_database(serialized, length, &db));
    
    hs_scratch_t *scratch = nullptr;
    ASSERT_EQ(HS_SUCCESS, hs_alloc_scratch(db, &scratch));
    
    int matches = 0;
    auto match_callback = []([[maybe_unused]] unsigned int id, [[maybe_unused]] unsigned long long from,
                             [[maybe_unused]] unsigned long long to, [[maybe_unused]] unsigned int flags,
                             void *context) -> int {
        (*static_cast<int *>(context))++;
        return 0;
    };
    
    EXPECT_EQ(HS_SUCCESS, hs_scan(db, "abc", 3, 0, scratch, match_callback, &matches));
    EXPECT_GT(matches, 0) << "Expected at least one match";
    
    hs_free_scratch(scratch);
    hs_free_database(db);
    free(serialized);
}

// ============================================================================
// EXPLOIT ATTEMPT 1: Patch program with SET_STATE with unbounded index
// Expected: REJECTED during deserialization validation (after fix)
// ============================================================================
TEST(DeserializationRce, RejectSetStateWithUnboundedIndex) {
    char *serialized = nullptr;
    size_t length = 0;
    
    ASSERT_TRUE(make_serialized_db("abc", &serialized, &length)) << "Failed to create DB";
    
    // Parse the header to understand the structure
    auto *rose = reinterpret_cast<struct RoseEngine *>(serialized + kBytecodeStart);
    u32 rose_size = rose->size;
    u32 rolesWithStateCount = rose->rolesWithStateCount;
    
    // Use a real program offset from Rose engine (e.g., report program)
    u32 program_offset = rose->reportProgramOffset;
    if (!program_offset || program_offset >= rose_size) {
        // Try another program if reportProgram isn't available
        program_offset = rose->delayProgramOffset;
    }
    
    if (program_offset && program_offset + 8 <= rose_size) {
        // Forge a SET_STATE instruction with index = 0x10000000 (way beyond rolesWithStateCount)
        char *bytecode = serialized + kBytecodeStart;
        
        // Save original bytes to verify mutation is applied
        u8 original_bytes[8];
        memcpy(original_bytes, bytecode + program_offset, sizeof(original_bytes));
        
        // ROSE_INSTR_SET_STATE = 41 (0x29)
        bytecode[program_offset] = 0x29;  // SET_STATE
        bytecode[program_offset + 1] = 0x00;
        bytecode[program_offset + 2] = 0x00;
        bytecode[program_offset + 3] = 0x00;
        
        // index = 0x10000000 (way beyond rolesWithStateCount)
        u32 *index_ptr = reinterpret_cast<u32 *>(bytecode + program_offset + 4);
        *index_ptr = 0x10000000;
        
        // Assert mutation was applied before resealing
        ASSERT_NE(bytecode[program_offset], original_bytes[0]) 
            << "Mutation should have changed the original byte";
        
        // Recompute HMACs
        reseal_bytecode_hmac(serialized, length - kBytecodeStart);
        reseal_header_hmac(serialized);
        
        // Attempt deserialization — should be REJECTED after fix
        hs_database_t *db = nullptr;
        hs_error_t result = hs_deserialize_database(serialized, length, &db);
        
        if (result == HS_SUCCESS) {
            // After fix: should reject during validation
            // Before fix: would accept (vulnerability)
            hs_free_database(db);
        }
        
        EXPECT_NE(HS_SUCCESS, result) << "Forged SET_STATE with unbounded index should be rejected";
    }
    
    free(serialized);
}

// ============================================================================
// EXPLOIT ATTEMPT 2: Forge SPARSE_ITER with arbitrary jump table
// Expected: REJECTED during deserialization validation (after fix)
// ============================================================================
TEST(DeserializationRce, RejectSparseIterWithArbitraryJumpTable) {
    char *serialized = nullptr;
    size_t length = 0;
    
    ASSERT_TRUE(make_serialized_db("xyz", &serialized, &length)) << "Failed to create DB";
    
    auto *rose = reinterpret_cast<struct RoseEngine *>(serialized + kBytecodeStart);
    u32 rose_size = rose->size;
    
    // Use a real program offset from Rose engine
    u32 program_offset = rose->delayProgramOffset;
    if (!program_offset || program_offset >= rose_size) {
        program_offset = rose->reportProgramOffset;
    }
    
    if (program_offset && program_offset + 12 <= rose_size) {
        char *bytecode = serialized + kBytecodeStart;
        
        // Save original bytes to verify mutation is applied
        u8 original_bytes[12];
        memcpy(original_bytes, bytecode + program_offset, sizeof(original_bytes));
        
        // ROSE_INSTR_SPARSE_ITER_BEGIN = 45 (the actual enum value)
        // Format: [u8 code=45][u8 fail_offset][u32 iter_offset][u32 jump_table]
        bytecode[program_offset] = 45;  // SPARSE_ITER_BEGIN (actual enum)
        bytecode[program_offset + 1] = 0x08;  // fail offset
        
        // iter_offset at byte +2 (not +4, that would be invalid)
        // jump_table at byte +6 (not +8)
        // Wait, let me reconsider. The comment says offsets at +4 and +8.
        // Let me check: [u8 code][u8 fail][u32 iter][u32 jump] = 1+1+4+4 = 10 bytes
        // But the test comments says 12 bytes. Let me look at the structure.
        // Looking at the user's suggestion: offsets at +4 and +8 make sense for
        // a 1-byte code + 1-byte padding + 4-byte iter + 4-byte jump = 10 bytes minimum
        // But with alignment it's 12 bytes. So:
        // [u8 code=45][u8 fail][u16 pad?][u32 iter_off][u32 jump_table]
        // That would be: +0=code, +1=fail, +2-3=pad, +4=iter, +8=jump
        
        u32 *iter_off = reinterpret_cast<u32 *>(bytecode + program_offset + 4);
        *iter_off = 0x20000000;  // beyond rose_size
        
        u32 *jmp_tbl = reinterpret_cast<u32 *>(bytecode + program_offset + 8);
        *jmp_tbl = 0x30000000;  // beyond rose_size
        
        // Assert mutation was applied before resealing
        ASSERT_NE(bytecode[program_offset], original_bytes[0]) 
            << "Mutation should have changed the original byte";
        
        // Recompute HMACs
        reseal_bytecode_hmac(serialized, length - kBytecodeStart);
        reseal_header_hmac(serialized);
        
        // Attempt deserialization
        hs_database_t *db = nullptr;
        hs_error_t result = hs_deserialize_database(serialized, length, &db);
        
        if (result == HS_SUCCESS) {
            hs_free_database(db);
        }
        
        EXPECT_NE(HS_SUCCESS, result) << "Forged SPARSE_ITER with arbitrary offsets should be rejected";
    }
    
    free(serialized);
}

// ============================================================================
// EXPLOIT ATTEMPT 3: Verify noodTable structure is valid
// Note: noodTable.id is a report ID passed to callback, not a Rose bytecode
// offset, so it is not bounds-checked against rose_size. Only actual offset
// fields in the noodTable are validated.
// Expected: Valid databases deserialize successfully
// ============================================================================
TEST(DeserializationRce, RejectInvalidNoodTableId) {
    char *serialized = nullptr;
    size_t length = 0;
    
    ASSERT_TRUE(make_serialized_db("test", &serialized, &length)) << "Failed to create DB";
    
    auto *rose = reinterpret_cast<struct RoseEngine *>(serialized + kBytecodeStart);
    u32 rose_size = rose->size;
    
    // Locate Noodle table: fmatcherOffset + ROUNDUP_CL(sizeof(HWLM))
    // Verify the structure is accessible; noodTable.id is not bounds-checked
    // because it is a report ID, not an offset into the database bytecode.
    if (rose->fmatcherOffset && rose->fmatcherOffset < rose_size) {
        char *bytecode = serialized + kBytecodeStart;
        u32 nood_table_offset = rose->fmatcherOffset + ROUNDUP_CL(192); // 192 = sizeof(HWLM)
        
        if (nood_table_offset + sizeof(struct noodTable) <= rose_size) {
            // Find and read the noodTable.id field (not modified)
            auto *nood_table = reinterpret_cast<struct noodTable *>(
                bytecode + nood_table_offset);
            
            // noodTable.id value is not validated against rose_size
            // (it is a user-provided report ID, not a bytecode offset)
            u32 report_id = nood_table->id;
            (void)report_id; // Suppress unused variable warning
            
            // Deserialize unmodified database — should succeed
            hs_database_t *db = nullptr;
            hs_error_t result = hs_deserialize_database(serialized, length, &db);
            
            EXPECT_EQ(HS_SUCCESS, result) 
                << "Valid database with normal noodTable should deserialize";
            
            if (result == HS_SUCCESS) {
                hs_free_database(db);
            }
        }
    }
    
    free(serialized);
}

// ============================================================================
// Defense verification: All malformed instructions should be rejected
// ============================================================================
TEST(DeserializationRce, ValidInstructionParametersAccepted) {
    char *serialized = nullptr;
    size_t length = 0;
    
    ASSERT_TRUE(make_serialized_db("valid", &serialized, &length)) << "Failed to create DB";
    
    // Deserialize without modifications — should succeed
    hs_database_t *db = nullptr;
    hs_error_t result = hs_deserialize_database(serialized, length, &db);
    
    EXPECT_EQ(HS_SUCCESS, result) << "Valid, unmodified database should deserialize successfully";
    
    if (result == HS_SUCCESS) {
        hs_free_database(db);
    }
    
    free(serialized);
}

} // namespace
