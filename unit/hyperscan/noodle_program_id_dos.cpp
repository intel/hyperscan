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
 * \brief Regression tests for DoS caused by a forged noodTable->id.
 *
 * A forged noodTable->id redirects the Rose program interpreter to
 * an attacker-chosen bytecode offset. The literal still matches, but the
 * computed-goto interpreter then executes garbage, which hangs the scan
 * (DoS). This is the gap left by the CVE-2023-28711 fix, which only hardened
 * the explicit program-offset fields.
 */

#include "config.h"

#include "hs.h"
#include "hs_runtime.h"
#include "database.h"
#include "ue2common.h"
#include "rose/rose_internal.h"
#include "rose/rose_program.h"
#include "hwlm/hwlm_internal.h"
#include "hwlm/noodle_internal.h"
#include "hs_db_hmac_key.h"

#include <openssl/hmac.h>
#include <openssl/evp.h>

#include "gtest/gtest.h"

namespace {

constexpr size_t kHmacOffset = 3 * sizeof(u32) + sizeof(u64a);
constexpr size_t kHmacHdrOffset = kHmacOffset + 32;
constexpr size_t kBytecodeStart = kHmacHdrOffset + 32;

void reseal_bytecode_hmac(char *serialized, size_t) {
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

void reseal_header_hmac(char *serialized) {
    u8 buf[20];
    memcpy(buf, serialized, sizeof(buf));
    u8 new_hmac_hdr[32] = {0};
    unsigned int hmac_len = 32;
    HMAC(EVP_sha256(), HS_DB_HMAC_KEY, sizeof(HS_DB_HMAC_KEY),
         buf, sizeof(buf), new_hmac_hdr, &hmac_len);
    memcpy(serialized + kHmacHdrOffset, new_hmac_hdr, sizeof(new_hmac_hdr));
}

bool make_literal_db(char **bytes, size_t *length) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *cerr = nullptr;
    if (hs_compile("hello", HS_FLAG_DOTALL, HS_MODE_BLOCK, nullptr, &db,
                   &cerr) != HS_SUCCESS) {
        if (cerr) {
            hs_free_compile_error(cerr);
        }
        return false;
    }
    if (hs_serialize_database(db, bytes, length) != HS_SUCCESS) {
        hs_free_database(db);
        return false;
    }
    hs_free_database(db);
    return true;
}

struct noodTable *get_nood_table(char *bytes) {
    struct RoseEngine *rose = (struct RoseEngine *)(bytes + kBytecodeStart);
    if (!rose->fmatcherOffset) {
        return nullptr;
    }
    struct HWLM *hwlm = (struct HWLM *)((char *)rose + rose->fmatcherOffset);
    if (hwlm->type != HWLM_ENGINE_NOOD) {
        return nullptr;
    }
    u32 nood_rel = (u32)ROUNDUP_CL(sizeof(struct HWLM));
    return (struct noodTable *)((char *)hwlm + nood_rel);
}

TEST(NoodleProgramIdDos, BaselineLiteralDatabaseDeserializes) {
    char *bytes = nullptr;
    size_t length = 0;
    ASSERT_TRUE(make_literal_db(&bytes, &length));

    hs_database_t *db = nullptr;
    EXPECT_EQ(HS_SUCCESS, hs_deserialize_database(bytes, length, &db));
    EXPECT_TRUE(db != nullptr);
    if (db) {
        hs_free_database(db);
    }
    free(bytes);
}

TEST(NoodleProgramIdDos, RejectForgedNoodleProgramId) {
    char *bytes = nullptr;
    size_t length = 0;
    ASSERT_TRUE(make_literal_db(&bytes, &length));

    struct noodTable *nood = get_nood_table(bytes);
    if (!nood) {
        free(bytes);
        return; // not a noodle matcher in this build
    }

    // In-bounds but arbitrary program offset, as used by the PoC.
    nood->id = 200;
    reseal_bytecode_hmac(bytes, length - kBytecodeStart);
    reseal_header_hmac(bytes);

    hs_database_t *db = nullptr;
    EXPECT_EQ(HS_INVALID, hs_deserialize_database(bytes, length, &db))
        << "forged noodTable->id was not rejected";
    EXPECT_TRUE(db == nullptr);
    if (db) {
        hs_free_database(db);
    }
    free(bytes);
}

TEST(NoodleProgramIdDos, RejectZeroNoodleProgramId) {
    char *bytes = nullptr;
    size_t length = 0;
    ASSERT_TRUE(make_literal_db(&bytes, &length));

    struct noodTable *nood = get_nood_table(bytes);
    if (!nood) {
        free(bytes);
        return;
    }

    nood->id = 0;
    reseal_bytecode_hmac(bytes, length - kBytecodeStart);
    reseal_header_hmac(bytes);

    hs_database_t *db = nullptr;
    EXPECT_EQ(HS_INVALID, hs_deserialize_database(bytes, length, &db))
        << "zero noodTable->id was not rejected";
    EXPECT_TRUE(db == nullptr);
    if (db) {
        hs_free_database(db);
    }
    free(bytes);
}

} // namespace
