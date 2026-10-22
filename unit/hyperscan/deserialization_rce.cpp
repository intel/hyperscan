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
#include "hwlm/hwlm_internal.h"
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

static void reseal_bytecode_hmac(char *serialized) {
    // The HMAC covers exactly the `length` bytes named in the header; the
    // serialized blob is longer than that due to alignment padding.
    u32 bc_len;
    memcpy(&bc_len, serialized + 2 * sizeof(u32), sizeof(bc_len));
    u8 new_hmac[32] = {0};
    unsigned int hmac_len = 32;
    HMAC(EVP_sha256(), HS_DB_HMAC_KEY, sizeof(HS_DB_HMAC_KEY),
         reinterpret_cast<const unsigned char *>(serialized + kBytecodeStart),
         bc_len, new_hmac, &hmac_len);
    memcpy(serialized + kHmacOffset, new_hmac, sizeof(new_hmac));
}

static void reseal_header_hmac(char *serialized) {
    u8 buf[20];
    memcpy(buf, serialized, 20); // magic + version + length + platform
    
    u8 new_hmac_hdr[32] = {0};
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
    
    // Find a valid program offset - prefer eod program as it's likely to exist
    u32 program_offset = rose->eodProgramOffset;
    if (!program_offset || program_offset >= rose_size || program_offset < sizeof(struct RoseEngine)) {
        program_offset = 0; // Skip test if no valid program
    }
    
    if (program_offset && program_offset + 8 <= rose_size) {
        // Forge a SET_STATE instruction with index = 0x10000000 (way beyond rolesWithStateCount)
        char *bytecode = serialized + kBytecodeStart;
        
        // Save original bytes to verify mutation is applied
        u8 original_bytes[8];
        memcpy(original_bytes, bytecode + program_offset, sizeof(original_bytes));
        
        // ROSE_INSTR_SET_STATE = 41 (0x29)
        // Structure: [u8 code=41][u8 pad[3]][u32 index]
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
        reseal_bytecode_hmac(serialized);
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
    
    // Find a valid program offset - prefer eod program
    u32 program_offset = rose->eodProgramOffset;
    if (!program_offset || program_offset >= rose_size || program_offset < sizeof(struct RoseEngine)) {
        program_offset = 0; // Skip test if no valid program
    }
    
    if (program_offset && program_offset + 16 <= rose_size) {
        char *bytecode = serialized + kBytecodeStart;
        
        // Save original bytes to verify mutation is applied
        u8 original_bytes[16];
        memcpy(original_bytes, bytecode + program_offset, sizeof(original_bytes));
        
        // ROSE_INSTR_SPARSE_ITER_BEGIN = 45
        // Structure (with alignment):
        // [u8 code=45][u8 pad[3]][u32 iter_offset][u32 jump_table][u32 fail_jump]
        // Offsets: +0=code, +1-3=pad, +4=iter, +8=jump, +12=fail_jump
        
        bytecode[program_offset] = 45;  // ROSE_INSTR_SPARSE_ITER_BEGIN
        bytecode[program_offset + 1] = 0x00;
        bytecode[program_offset + 2] = 0x00;
        bytecode[program_offset + 3] = 0x00;
        
        // iter_offset at +4: set to out-of-bounds
        u32 *iter_off = reinterpret_cast<u32 *>(bytecode + program_offset + 4);
        *iter_off = 0x20000000;  // way beyond rose_size
        
        // jump_table at +8: set to out-of-bounds
        u32 *jmp_tbl = reinterpret_cast<u32 *>(bytecode + program_offset + 8);
        *jmp_tbl = 0x30000000;  // way beyond rose_size
        
        // Assert mutation was applied before resealing
        ASSERT_NE(bytecode[program_offset], original_bytes[0]) 
            << "Mutation should have changed the original byte";
        
        // Recompute HMACs
        reseal_bytecode_hmac(serialized);
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
// EXPLOIT ATTEMPT 3: Forge noodTable.id to invalid program offset
// noodTable.id is used as a Rose program entry point passed to roseRunProgram().
// A forged misaligned or out-of-bounds value can trigger OOB read/memory corruption.
// Expected: REJECTED during deserialization validation (after fix)
// ============================================================================
TEST(DeserializationRce, RejectInvalidNoodTableId) {
    char *serialized = nullptr;
    size_t length = 0;
    
    ASSERT_TRUE(make_serialized_db("test", &serialized, &length)) << "Failed to create DB";
    
    auto *rose = reinterpret_cast<struct RoseEngine *>(serialized + kBytecodeStart);
    u32 rose_size = rose->size;
    
    // Locate Noodle table: fmatcherOffset + ROUNDUP_CL(sizeof(HWLM))
    // noodTable.id MUST be a valid program offset (aligned, in bounds)
    if (rose->fmatcherOffset && rose->fmatcherOffset < rose_size &&
        rose->fmatcherOffset + sizeof(struct HWLM) <= rose_size) {
        const struct HWLM *hwlm = 
            reinterpret_cast<const struct HWLM *>(serialized + kBytecodeStart + rose->fmatcherOffset);
        
        if (hwlm->type == HWLM_ENGINE_NOOD) {
            char *bytecode = serialized + kBytecodeStart;
            u32 nood_table_offset = rose->fmatcherOffset + ROUNDUP_CL(192); // 192 = sizeof(HWLM)
            
            if (nood_table_offset + sizeof(struct noodTable) <= rose_size) {
                auto *nood_table = reinterpret_cast<struct noodTable *>(
                    bytecode + nood_table_offset);
                
                // Save original ID to verify mutation is applied
                u32 original_id = nood_table->id;
                
                // Mutate noodTable.id to an invalid/misaligned program offset
                // Option 1: Out of bounds (rose_size + 100)
                nood_table->id = rose_size + 100;
                
                // Assert mutation was applied before resealing
                ASSERT_NE(nood_table->id, original_id) 
                    << "Mutation should have changed noodTable.id";
                
                // Recompute HMACs to match the forged data
                reseal_bytecode_hmac(serialized);
                reseal_header_hmac(serialized);
                
                // Attempt deserialization with forged noodTable.id
                hs_database_t *db = nullptr;
                hs_error_t result = hs_deserialize_database(serialized, length, &db);
                
                if (result == HS_SUCCESS) {
                    hs_free_database(db);
                }
                
                // Should REJECT with HS_INVALID because noodTable.id is out of bounds
                EXPECT_NE(HS_SUCCESS, result) 
                    << "Forged noodTable.id (out of bounds) should be rejected";
            }
        }
    }
    
    free(serialized);
}

// ============================================================================
// Additional test: Verify noodTable metadata validation
// Expected: Invalid msk_len or key_offset should be rejected
// ============================================================================
TEST(DeserializationRce, RejectInvalidNoodTableMetadata) {
    char *serialized = nullptr;
    size_t length = 0;
    
    ASSERT_TRUE(make_serialized_db("test", &serialized, &length)) << "Failed to create DB";
    
    auto *rose = reinterpret_cast<struct RoseEngine *>(serialized + kBytecodeStart);
    u32 rose_size = rose->size;
    
    if (rose->fmatcherOffset && rose->fmatcherOffset < rose_size &&
        rose->fmatcherOffset + sizeof(struct HWLM) <= rose_size) {
        const struct HWLM *hwlm = 
            reinterpret_cast<const struct HWLM *>(serialized + kBytecodeStart + rose->fmatcherOffset);
        
        if (hwlm->type == HWLM_ENGINE_NOOD) {
            char *bytecode = serialized + kBytecodeStart;
            u32 nood_table_offset = rose->fmatcherOffset + ROUNDUP_CL(192);
            
            if (nood_table_offset + sizeof(struct noodTable) <= rose_size) {
                auto *nood_table = reinterpret_cast<struct noodTable *>(
                    bytecode + nood_table_offset);
                
                // Save original metadata
                u8 original_msk_len = nood_table->msk_len;
                
                // Mutate msk_len to invalid value (0 or > 8)
                nood_table->msk_len = 0;  // Invalid: must be in [1, 8]
                
                // Recompute HMACs
                reseal_bytecode_hmac(serialized);
                reseal_header_hmac(serialized);
                
                // Attempt deserialization
                hs_database_t *db = nullptr;
                hs_error_t result = hs_deserialize_database(serialized, length, &db);
                
                if (result == HS_SUCCESS) {
                    hs_free_database(db);
                }
                
                // Should REJECT because msk_len is invalid
                EXPECT_NE(HS_SUCCESS, result) 
                    << "Invalid noodTable.msk_len should be rejected";
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
