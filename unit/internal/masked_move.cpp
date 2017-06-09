/*
 * Copyright (c) 2015-2017, Intel Corporation
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

#include <cstring>

#include "gtest/gtest.h"
#include "util/arch.h"
#include "util/masked_move.h"

namespace {

#if defined(HAVE_AVX2)

bool try_mask_len(const u8 *buf, u8 *target, size_t len) {
    memset(target, 0, 32);
    memcpy(target, buf, len);
    m256 mask = masked_move256_len(buf, len);
    return (0 == memcmp((u8 *)&mask, target, 32));
}

static const char *alpha = "0123456789abcdefghijklmnopqrstuvwxyz0123456789abcde"
                           "fghijklmnopqrstuvwxyz0123456789abcdefghijklmnopqrst"
                           "uvwxyz";

TEST(MaskedMove, tymm) {
    const u8 *buf = (const u8 *)alpha;
    u8 target[32];

    for (int len = 4; len <= 32; len++) {
        EXPECT_TRUE(try_mask_len(buf, target, len)) << "len: " << len;
    }
}

#endif

} // namespace
