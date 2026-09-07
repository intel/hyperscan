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

#ifdef HAVE__ALIGNED_MALLOC
#include <malloc.h>
#endif

#include "gtest/gtest.h"

#include "hs.h"
#include "hs_runtime.h"
#include "database.h"
#include "ue2common.h"
#include "rose/rose_internal.h"
#include "util/scatter.h"

namespace {

TEST(ScatterDestOob, OpenStreamRejectsForgedScatterDestOffset) {
    const size_t bytecode_offset = ROUNDUP_N(sizeof(hs_database_t), 64);
    const size_t rose_size = 0x500;
    const size_t total_size = bytecode_offset + rose_size;

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

    hs_database_t *db = static_cast<hs_database_t *>(mem);
    memset(db, 0, total_size);
    db->magic = HS_DB_MAGIC;
    db->version = HS_DB_VERSION;
    db->length = static_cast<u32>(total_size - sizeof(hs_database_t));
    db->bytecode = static_cast<u32>(bytecode_offset);

    struct RoseEngine *rose =
        reinterpret_cast<struct RoseEngine *>(static_cast<char *>(mem) +
                                              bytecode_offset);
    memset(rose, 0, rose_size);
    rose->size = static_cast<u32>(rose_size);
    rose->mode = HS_MODE_STREAM;

    // A small stream state -- everything else in stateOffsets/counts is left
    // at zero so every other region check in validateStateLayout() is
    // trivially satisfied (offset=0, size=0).
    rose->stateOffsets.end = 64;

    // Place a one-entry scatter_unit_u64a table at a properly (8-byte)
    // aligned offset inside the rose bytecode, well within rose_size.
    const u32 table_offset = 0x300;
    rose->state_init.s_u64a_offset = table_offset;
    rose->state_init.s_u64a_count = 1;

    auto *entry = reinterpret_cast<struct scatter_unit_u64a *>(
        reinterpret_cast<char *>(rose) + table_offset);
    entry->offset = 0x1000; // far past stateOffsets.end (64) -> OOB write
    entry->val = 0x4141414141414141ULL;

    hs_stream_t *stream = nullptr;
    hs_error_t err = hs_open_stream(db, 0, &stream);

    // After the fix, hs_open_stream must reject the forged scatter plan.
    // Before the fix, this call succeeds and roseInitState() performs an
    // 8-byte OOB heap write at state+0x1000 against a ~64+sizeof(hs_stream)
    // byte allocation; ASan-enabled builds will abort here.
    EXPECT_NE(HS_SUCCESS, err);

    if (stream) {
        hs_close_stream(stream, nullptr, nullptr, nullptr);
    }
#if defined(HAVE__ALIGNED_MALLOC) && !defined(HAVE_POSIX_MEMALIGN)
    _aligned_free(mem);
#else
    free(mem);
#endif
}

} // namespace
