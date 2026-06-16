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

#include "config.h"

#include <cstdint>
#include <cstdlib>
#include <cstring>

#ifdef HAVE__ALIGNED_MALLOC
#include <malloc.h>
#endif

#include "gtest/gtest.h"

#include "chimera/ch.h"
#include "ue2common.h"
#include "database.h"      // hs_database_t, HS_DB_MAGIC, HS_DB_VERSION
#include "chimera/ch_scratch.h"

namespace {

// Local mirror of the chimera bytecode header. Kept here so the test does not
// have to pull in <pcre.h> via chimera/ch_database.h. Must stay in sync with
// chimera/ch_database.h::struct ch_database / struct ch_bytecode.
struct ch_database_layout {
    u32 magic;
    u32 version;
    u32 length;
    u32 reserved0;
    u32 reserved1;
    u32 bytecode;
    u32 padding[16];
    char bytes[];
};

struct ch_bytecode_layout {
    u32 length;
    u32 flags;
    u32 patternCount;
    u32 activeSize;
    u32 databaseOffset;
    u32 patternOffset;
    u32 unguardedOffset;
    u32 unguardedCount;
    u32 maxCaptureGroups;
};

#ifndef CH_DB_MAGIC
#define CH_DB_MAGIC 0xdedededeU
#endif

// Mirror of chimera/ch_internal.h::CHIMERA_FLAG_NO_MULTIMATCH. Setting this
// flag short-circuits ch_alloc_scratch() before it calls hs_alloc_scratch()
// on the (unforged) inner Hyperscan database, isolating the activeSize vs.
// patternCount sizing validation that this regression test targets.
#define PSIRT_CHIMERA_FLAG_NO_MULTIMATCH 1u

// coverity[RULE_OF_ZERO_THREE_FIVE:FALSE]
TEST(PsirtActiveSize, AllocScratchRejectsForgedBytecode) {
    const u32 pattern_count = 8192;
    const u32 forged_active_size = 1; // far smaller than mmbit_size(8192)

    const size_t bytecode_offset =
        ROUNDUP_N(sizeof(ch_database_layout), 64);
    const size_t bytecode_size = sizeof(ch_bytecode_layout) + 256;
    const size_t total_size = bytecode_offset + bytecode_size;

    void *mem = nullptr;
#if defined(HAVE_POSIX_MEMALIGN)
    int rc = posix_memalign(&mem, 64, total_size);
    if (rc != 0 || mem == nullptr) {
        FAIL() << "posix_memalign failed with rc=" << rc;
        return;
    }
#elif defined(HAVE__ALIGNED_MALLOC)
    mem = _aligned_malloc(total_size, 64);
    if (mem == nullptr) {
        FAIL() << "_aligned_malloc failed";
        return;
    }
#else
#error "No aligned allocation function available"
#endif

    auto *hydb = static_cast<ch_database_layout *>(mem);
    memset(hydb, 0, total_size);
    hydb->magic = CH_DB_MAGIC;
    hydb->version = HS_VERSION_32BIT;
    hydb->length = static_cast<u32>(total_size - sizeof(ch_database_layout));
    hydb->bytecode = static_cast<u32>(bytecode_offset);

    auto *db = reinterpret_cast<ch_bytecode_layout *>(
        static_cast<char *>(mem) + bytecode_offset);
    db->length = static_cast<u32>(bytecode_size);
    db->flags = PSIRT_CHIMERA_FLAG_NO_MULTIMATCH;
    db->patternCount = pattern_count;
    db->activeSize = forged_active_size;
    db->databaseOffset = 0;
    db->patternOffset = 0;
    db->unguardedOffset = 0;
    db->unguardedCount = 0;
    db->maxCaptureGroups = 0;

    ch_scratch_t *scratch = nullptr;
    ch_error_t err = ch_alloc_scratch(
        reinterpret_cast<const ch_database_t *>(hydb), &scratch);

    // After the fix, the forged bytecode must be rejected before scratch is
    // sized from the mismatched activeSize. Before the fix this returns
    // CH_SUCCESS and a subsequent ch_scan() would overflow the active bitset.
    if (err == CH_SUCCESS && scratch != nullptr) {
        // Fix is NOT in place — ch_alloc_scratch should have rejected the
        // forged DB (activeSize=1, patternCount=8192).
        ch_free_scratch(scratch);
        FAIL() << "ch_alloc_scratch should have rejected the forged DB "
                  "(activeSize=1, patternCount=8192)";
    } else {
        // Fix is in place: ch_alloc_scratch properly rejected the forged DB.
        EXPECT_NE(CH_SUCCESS, err);
    }

#if defined(HAVE__ALIGNED_MALLOC) && !defined(HAVE_POSIX_MEMALIGN)
    _aligned_free(mem);
#else
    free(mem);
#endif
}

} // namespace
