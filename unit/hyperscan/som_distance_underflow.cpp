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
 * \brief Regression tests for SOM from_offset underflow via a forged
 * som_operation.somDistance.
 *
 * A SOM_FROM_REPORT instruction embedded in Rose bytecode carries a
 * somDistance that is subtracted from to_offset. The only guard was an
 * assert(), compiled out in release builds, so a forged distance underflowed
 * and produced from_offset > to_offset at the user callback.
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
#include "som/som_operation.h"
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

struct MatchRecord {
    bool fired = false;
    unsigned long long from = 0;
    unsigned long long to = 0;
};

int HS_CDECL onMatch(unsigned, unsigned long long from, unsigned long long to,
                     unsigned, void *ctx) {
    MatchRecord *rec = static_cast<MatchRecord *>(ctx);
    rec->fired = true;
    rec->from = from;
    rec->to = to;
    return 0;
}

bool make_som_db(char **bytes, size_t *length) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *cerr = nullptr;
    if (hs_compile("needle", HS_FLAG_SOM_LEFTMOST, HS_MODE_BLOCK, nullptr,
                   &db, &cerr) != HS_SUCCESS) {
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

// Locate the SOM_FROM_REPORT instruction reached from the noodle literal
// program, if this build lays the database out that way.
struct ROSE_STRUCT_SOM_FROM_REPORT *find_som_from_report(char *bytes) {
    struct RoseEngine *rose = (struct RoseEngine *)(bytes + kBytecodeStart);
    if (!rose->fmatcherOffset) {
        return nullptr;
    }
    struct HWLM *hwlm = (struct HWLM *)((char *)rose + rose->fmatcherOffset);
    if (hwlm->type != HWLM_ENGINE_NOOD) {
        return nullptr;
    }
    u32 nood_rel = (u32)ROUNDUP_CL(sizeof(struct HWLM));
    struct noodTable *nood = (struct noodTable *)((char *)hwlm + nood_rel);
    if (!nood->id || nood->id >= rose->size) {
        return nullptr;
    }
    struct ROSE_STRUCT_SOM_FROM_REPORT *ins =
        (struct ROSE_STRUCT_SOM_FROM_REPORT *)((char *)rose + nood->id);
    if (ins->code != ROSE_INSTR_SOM_FROM_REPORT) {
        return nullptr;
    }
    return ins;
}

const char kData[] = "this buffer contains the word needle in it";

TEST(SomDistanceUnderflow, BaselineSomReportHasSaneOffsets) {
    char *bytes = nullptr;
    size_t length = 0;
    ASSERT_TRUE(make_som_db(&bytes, &length));

    hs_database_t *db = nullptr;
    ASSERT_EQ(HS_SUCCESS, hs_deserialize_database(bytes, length, &db));
    ASSERT_TRUE(db != nullptr);

    hs_scratch_t *scratch = nullptr;
    ASSERT_EQ(HS_SUCCESS, hs_alloc_scratch(db, &scratch));

    MatchRecord rec;
    ASSERT_EQ(HS_SUCCESS,
              hs_scan(db, kData, (unsigned)strlen(kData), 0, scratch, onMatch,
                      &rec));
    EXPECT_TRUE(rec.fired);
    EXPECT_LE(rec.from, rec.to);

    hs_free_scratch(scratch);
    hs_free_database(db);
    free(bytes);
}

TEST(SomDistanceUnderflow, ForgedSomDistanceDoesNotUnderflow) {
    char *bytes = nullptr;
    size_t length = 0;
    ASSERT_TRUE(make_som_db(&bytes, &length));

    struct ROSE_STRUCT_SOM_FROM_REPORT *ins = find_som_from_report(bytes);
    if (!ins) {
        free(bytes);
        return; // layout differs in this build; nothing to forge
    }

    // Far larger than any to_offset for our tiny scan buffer.
    ins->som.aux.somDistance = 999999999ULL;
    reseal_bytecode_hmac(bytes, length - kBytecodeStart);
    reseal_header_hmac(bytes);

    hs_database_t *db = nullptr;
    hs_error_t err = hs_deserialize_database(bytes, length, &db);
    if (err != HS_SUCCESS) {
        // Rejected at deserialize time is also an acceptable outcome.
        EXPECT_EQ(HS_INVALID, err);
        free(bytes);
        return;
    }
    ASSERT_TRUE(db != nullptr);

    hs_scratch_t *scratch = nullptr;
    ASSERT_EQ(HS_SUCCESS, hs_alloc_scratch(db, &scratch));

    MatchRecord rec;
    ASSERT_EQ(HS_SUCCESS,
              hs_scan(db, kData, (unsigned)strlen(kData), 0, scratch, onMatch,
                      &rec));

    if (rec.fired) {
        EXPECT_LE(rec.from, rec.to)
            << "forged somDistance produced from_offset > to_offset, "
               "violating the SOM API contract";
        EXPECT_LE(rec.to, (unsigned long long)strlen(kData));
    }

    hs_free_scratch(scratch);
    hs_free_database(db);
    free(bytes);
}

} // namespace
