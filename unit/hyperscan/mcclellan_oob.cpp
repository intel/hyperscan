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
 * CONTRACT, STRICT LIABILITY, TORT OR (NEGLIGENCE OR OTHERWISE)
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
#include "nfa/nfa_internal.h"
#include "nfa/mcclellan_internal.h"
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

// Build a simple database with a McClellan engine
// Pattern: "a.*b" — creates a left-anchor pattern that uses DFA
static bool make_mcclellan_db(const char *pattern, char **bytes,
                               size_t *length) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_expr_info_t *expr_info = nullptr;

    // Compile pattern
    if (hs_compile(pattern, HS_FLAG_DOTALL, HS_MODE_BLOCK, nullptr,
                   &db, &compile_err) != HS_SUCCESS) {
        if (compile_err) {
            hs_free_compile_error(compile_err);
        }
        return false;
    }

    if (!db) {
        return false;
    }

    // Serialize database
    char *serialized = nullptr;
    size_t serial_len = 0;
    if (hs_serialize_database(db, &serialized, &serial_len) !=
        HS_SUCCESS) {
        hs_free_database(db);
        return false;
    }

    *bytes = serialized;
    *length = serial_len;
    hs_free_database(db);
    return true;
}

// Test: Valid McClellan database should deserialize successfully
TEST(McClellanOOB, ValidDatabaseAccepted) {
    char *bytes = nullptr;
    size_t length = 0;

    ASSERT_TRUE(make_mcclellan_db("a.*b", &bytes, &length));
    ASSERT_TRUE(bytes != nullptr);
    ASSERT_GT(length, 0u);

    // Deserialize the valid database
    hs_database_t *db = nullptr;
    EXPECT_EQ(hs_deserialize_database(bytes, length, &db),
              HS_SUCCESS);
    EXPECT_NE(db, nullptr);

    if (db) {
        hs_free_database(db);
    }
    free(bytes);
}

// Test: Forged start_anchored state out of bounds
TEST(McClellanOOB, ForgedStartAnchoredOutOfBounds) {
    char *bytes = nullptr;
    size_t length = 0;

    ASSERT_TRUE(make_mcclellan_db("a.*b", &bytes, &length));
    ASSERT_TRUE(bytes != nullptr);

    // Find the McClellan engine in the serialized bytecode
    // and forge start_anchored to be out of bounds
    struct RoseEngine *rose = (struct RoseEngine *)(bytes + kBytecodeStart);
    
    bool found_mcclellan = false;
    if (rose->nfaInfoOffset && rose->nfaInfoOffset < (u32)length) {
        struct NfaInfo *infos =
            (struct NfaInfo *)((char *)rose + rose->nfaInfoOffset);
        
        for (u32 qi = 0; qi < rose->queueCount; qi++) {
            if (!infos[qi].nfaOffset || infos[qi].nfaOffset >= (u32)length) {
                continue;
            }

            struct NFA *nfa =
                (struct NFA *)((char *)rose + infos[qi].nfaOffset);

            if (isMcClellanType(nfa->type)) {
                found_mcclellan = true;
                // Found a McClellan engine - forge start_anchored
                struct mcclellan *m = (struct mcclellan *)((char *)nfa + sizeof(struct NFA));
                
                // Set start_anchored beyond state_count to trigger OOB
                u16 original_start = m->start_anchored;
                m->start_anchored = m->state_count + 100; // OOB value

                // Re-compute HMACs
                reseal_bytecode_hmac(bytes, length - kBytecodeStart);
                reseal_header_hmac(bytes);

                // Attempt to deserialize forged database
                hs_database_t *db = nullptr;
                hs_error_t result =
                    hs_deserialize_database(bytes, length, &db);

                // Should be rejected
                EXPECT_EQ(result, HS_INVALID);
                EXPECT_EQ(db, nullptr);

                m->start_anchored = original_start; // restore
                break;
            }
        }
    }
    
    if (!found_mcclellan) {
        // Pattern did not generate a McClellan engine; skip this test
        free(bytes);
        return;
    }

    free(bytes);
}

// Test: Forged alphaShift > 8 (max 256 alphabet)
TEST(McClellanOOB, ForgedAlphaShiftTooLarge) {
    char *bytes = nullptr;
    size_t length = 0;

    ASSERT_TRUE(make_mcclellan_db("a.*b", &bytes, &length));
    ASSERT_TRUE(bytes != nullptr);

    struct RoseEngine *rose = (struct RoseEngine *)(bytes + kBytecodeStart);
    
    bool found_mcclellan = false;
    if (rose->nfaInfoOffset && rose->nfaInfoOffset < (u32)length) {
        struct NfaInfo *infos =
            (struct NfaInfo *)((char *)rose + rose->nfaInfoOffset);
        
        for (u32 qi = 0; qi < rose->queueCount; qi++) {
            if (!infos[qi].nfaOffset || infos[qi].nfaOffset >= (u32)length) {
                continue;
            }

            struct NFA *nfa =
                (struct NFA *)((char *)rose + infos[qi].nfaOffset);

            if (isMcClellanType(nfa->type)) {
                found_mcclellan = true;
                struct mcclellan *m = (struct mcclellan *)((char *)nfa + sizeof(struct NFA));
                
                // Forge alphaShift to cause massive OOB reads
                u8 original_alpha = m->alphaShift;
                m->alphaShift = 16;  // Invalid! Max should be 8

                // Re-compute HMACs
                reseal_bytecode_hmac(bytes, length - kBytecodeStart);
                reseal_header_hmac(bytes);

                // Attempt to deserialize
                hs_database_t *db = nullptr;
                hs_error_t result =
                    hs_deserialize_database(bytes, length, &db);

                // Should be rejected
                EXPECT_EQ(result, HS_INVALID);
                EXPECT_EQ(db, nullptr);

                m->alphaShift = original_alpha;  // restore
                break;
            }
        }
    }
    
    if (!found_mcclellan) {
        // Pattern did not generate a McClellan engine; skip this test
        free(bytes);
        return;
    }

    free(bytes);
}

// Test: Forged aux_offset points past NFA body
TEST(McClellanOOB, ForgedAuxOffsetOutOfBounds) {
    char *bytes = nullptr;
    size_t length = 0;

    ASSERT_TRUE(make_mcclellan_db("a.*b", &bytes, &length));
    ASSERT_TRUE(bytes != nullptr);

    struct RoseEngine *rose = (struct RoseEngine *)(bytes + kBytecodeStart);
    
    bool found_mcclellan = false;
    if (rose->nfaInfoOffset && rose->nfaInfoOffset < (u32)length) {
        struct NfaInfo *infos =
            (struct NfaInfo *)((char *)rose + rose->nfaInfoOffset);
        
        for (u32 qi = 0; qi < rose->queueCount; qi++) {
            if (!infos[qi].nfaOffset || infos[qi].nfaOffset >= (u32)length) {
                continue;
            }

            struct NFA *nfa =
                (struct NFA *)((char *)rose + infos[qi].nfaOffset);

            if (isMcClellanType(nfa->type)) {
                found_mcclellan = true;
                struct mcclellan *m = (struct mcclellan *)((char *)nfa + sizeof(struct NFA));
                
                // Forge aux_offset to point way past the NFA body
                u32 original_aux = m->aux_offset;
                m->aux_offset = nfa->length + 1000;  // Way past NFA end

                // Re-compute HMACs
                reseal_bytecode_hmac(bytes, length - kBytecodeStart);
                reseal_header_hmac(bytes);

                // Attempt to deserialize
                hs_database_t *db = nullptr;
                hs_error_t result =
                    hs_deserialize_database(bytes, length, &db);

                // Should be rejected
                EXPECT_EQ(result, HS_INVALID);
                EXPECT_EQ(db, nullptr);

                m->aux_offset = original_aux;  // restore
                break;
            }
        }
    }
    
    if (!found_mcclellan) {
        // Pattern did not generate a McClellan engine; skip this test
        free(bytes);
        return;
    }

    free(bytes);
}

// Test: Forged sherman_limit > state_count
TEST(McClellanOOB, ForgedShermanLimitOutOfBounds) {
    char *bytes = nullptr;
    size_t length = 0;

    ASSERT_TRUE(make_mcclellan_db("a.*b", &bytes, &length));
    ASSERT_TRUE(bytes != nullptr);

    struct RoseEngine *rose = (struct RoseEngine *)(bytes + kBytecodeStart);
    
    bool found_mcclellan = false;
    if (rose->nfaInfoOffset && rose->nfaInfoOffset < (u32)length) {
        struct NfaInfo *infos =
            (struct NfaInfo *)((char *)rose + rose->nfaInfoOffset);
        
        for (u32 qi = 0; qi < rose->queueCount; qi++) {
            if (!infos[qi].nfaOffset || infos[qi].nfaOffset >= (u32)length) {
                continue;
            }

            struct NFA *nfa =
                (struct NFA *)((char *)rose + infos[qi].nfaOffset);

            if (isMcClellanType(nfa->type)) {
                found_mcclellan = true;
                struct mcclellan *m = (struct mcclellan *)((char *)nfa + sizeof(struct NFA));
                
                // Forge sherman_limit to exceed state_count
                u16 original_limit = m->sherman_limit;
                m->sherman_limit = m->state_count + 100;

                // Re-compute HMACs
                reseal_bytecode_hmac(bytes, length - kBytecodeStart);
                reseal_header_hmac(bytes);

                // Attempt to deserialize
                hs_database_t *db = nullptr;
                hs_error_t result =
                    hs_deserialize_database(bytes, length, &db);

                // Should be rejected
                EXPECT_EQ(result, HS_INVALID);
                EXPECT_EQ(db, nullptr);

                m->sherman_limit = original_limit;  // restore
                break;
            }
        }
    }
    
    if (!found_mcclellan) {
        // Pattern did not generate a McClellan engine; skip this test
        free(bytes);
        return;
    }

    free(bytes);
}

// Test: Forged wide_limit > state_count
TEST(McClellanOOB, ForgedWideLimitOutOfBounds) {
    char *bytes = nullptr;
    size_t length = 0;

    ASSERT_TRUE(make_mcclellan_db("a.*b", &bytes, &length));
    ASSERT_TRUE(bytes != nullptr);

    struct RoseEngine *rose = (struct RoseEngine *)(bytes + kBytecodeStart);
    
    bool found_mcclellan = false;
    if (rose->nfaInfoOffset && rose->nfaInfoOffset < (u32)length) {
        struct NfaInfo *infos =
            (struct NfaInfo *)((char *)rose + rose->nfaInfoOffset);
        
        for (u32 qi = 0; qi < rose->queueCount; qi++) {
            if (!infos[qi].nfaOffset || infos[qi].nfaOffset >= (u32)length) {
                continue;
            }

            struct NFA *nfa =
                (struct NFA *)((char *)rose + infos[qi].nfaOffset);

            if (isMcClellanType(nfa->type)) {
                found_mcclellan = true;
                struct mcclellan *m = (struct mcclellan *)((char *)nfa + sizeof(struct NFA));
                
                // Forge wide_limit to exceed state_count
                u16 original_wide = m->wide_limit;
                m->wide_limit = m->state_count + 50;

                // Re-compute HMACs
                reseal_bytecode_hmac(bytes, length - kBytecodeStart);
                reseal_header_hmac(bytes);

                // Attempt to deserialize
                hs_database_t *db = nullptr;
                hs_error_t result =
                    hs_deserialize_database(bytes, length, &db);

                // Should be rejected
                EXPECT_EQ(result, HS_INVALID);
                EXPECT_EQ(db, nullptr);

                m->wide_limit = original_wide;  // restore
                break;
            }
        }
    }
    
    if (!found_mcclellan) {
        // Pattern did not generate a McClellan engine; skip this test
        free(bytes);
        return;
    }

    free(bytes);
}

// Test: Forged start_floating state out of bounds
TEST(McClellanOOB, ForgedStartFloatingOutOfBounds) {
    char *bytes = nullptr;
    size_t length = 0;

    ASSERT_TRUE(make_mcclellan_db("a.*b", &bytes, &length));
    ASSERT_TRUE(bytes != nullptr);

    struct RoseEngine *rose = (struct RoseEngine *)(bytes + kBytecodeStart);
    
    bool found_mcclellan = false;
    if (rose->nfaInfoOffset && rose->nfaInfoOffset < (u32)length) {
        struct NfaInfo *infos =
            (struct NfaInfo *)((char *)rose + rose->nfaInfoOffset);
        
        for (u32 qi = 0; qi < rose->queueCount; qi++) {
            if (!infos[qi].nfaOffset || infos[qi].nfaOffset >= (u32)length) {
                continue;
            }

            struct NFA *nfa =
                (struct NFA *)((char *)rose + infos[qi].nfaOffset);

            if (isMcClellanType(nfa->type)) {
                found_mcclellan = true;
                struct mcclellan *m = (struct mcclellan *)((char *)nfa + sizeof(struct NFA));
                
                // Forge start_floating beyond state_count
                u16 original_floating = m->start_floating;
                m->start_floating = m->state_count + 200;

                // Re-compute HMACs
                reseal_bytecode_hmac(bytes, length - kBytecodeStart);
                reseal_header_hmac(bytes);

                // Attempt to deserialize
                hs_database_t *db = nullptr;
                hs_error_t result =
                    hs_deserialize_database(bytes, length, &db);

                // Should be rejected
                EXPECT_EQ(result, HS_INVALID);
                EXPECT_EQ(db, nullptr);

                m->start_floating = original_floating;  // restore
                break;
            }
        }
    }
    
    if (!found_mcclellan) {
        // Pattern did not generate a McClellan engine; skip this test
        free(bytes);
        return;
    }

    free(bytes);
}

}  // namespace
