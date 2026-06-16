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

#include "hs.h"
#include "hs_runtime.h"
#include "database.h"
#include "state.h"
#include "ue2common.h"
#include "nfa/nfa_internal.h"
#include "rose/rose_internal.h"
#include "util/multibit_internal.h"

namespace {

// coverity[RULE_OF_ZERO_THREE_FIVE:FALSE]
TEST(PsirtLeftfixOob, ExpandStreamRejectsForgedLeftfix) {
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
    rose->size = rose_size;
    rose->mode = HS_MODE_STREAM;
    rose->stateOffsets.end = 16;
    rose->stateOffsets.activeLeftArray = 1;
    rose->stateOffsets.leftfixLagTable = 0;
    rose->activeLeftCount = 1;
    rose->activeLeftIterOffset = 0x200;
    rose->leftOffset = 0x220;
    rose->leftfixBeginQueue = 0;
    rose->nfaInfoOffset = 0x240;

    auto *it = reinterpret_cast<struct mmbit_sparse_iter *>(
        reinterpret_cast<char *>(rose) + rose->activeLeftIterOffset);
    memset(it, 0, sizeof(*it));
    it->mask = 1;
    it->val = 0;

    auto *left = reinterpret_cast<struct LeftNfaInfo *>(
        reinterpret_cast<char *>(rose) + rose->leftOffset);
    memset(left, 0, sizeof(*left));
    left->lagIndex = 0;

    auto *info = reinterpret_cast<struct NfaInfo *>(
        reinterpret_cast<char *>(rose) + rose->nfaInfoOffset);
    memset(info, 0, sizeof(*info));
    info->stateOffset = 12;
    info->nfaOffset = 0x280;

    auto *nfa = reinterpret_cast<struct NFA *>(
        reinterpret_cast<char *>(rose) + info->nfaOffset);
    memset(nfa, 0, sizeof(*nfa));
    nfa->streamStateSize = 8;

    // Match the serialized stream layout produced by the PoC:
    // [u64a offset][1B status flags][1B activeLeftArray bitset, ri=0 set]
    // [8B forged leftfix NFA stream state][1B lag byte].
    char buf[64];
    memset(buf, 0, sizeof(buf));
    u64a offset = 0;
    memcpy(buf, &offset, sizeof(offset));
    buf[sizeof(offset)] = 0;
    buf[sizeof(offset) + 1] = 1;
    memset(buf + sizeof(offset) + 2, 'S', 8);
    buf[sizeof(offset) + 2 + 8] = 0;

    hs_stream_t *stream = nullptr;
    hs_error_t err =
        hs_expand_stream(db, &stream, buf, sizeof(offset) + 2 + 8 + 1);

    // After the fix, hs_expand_stream must reject the forged database.
    // Before the fix, this call silently performs an OOB heap write inside
    // sc_left_expand(); ASan-enabled builds will abort here.
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
