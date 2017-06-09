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

#include "hs.h"
#include "compiler/compiler.h"
#include "crc32.h"
#include "database.h"
#include "ue2common.h"
#include "util/arch.h"
#include "util/target_info.h"

#include "gtest/gtest.h"

#include <array>

using namespace ue2;

TEST(DB, flagsToPlatform) {
    hs_platform_info p;
    memset(&p, 0, sizeof(p));

    p.cpu_features = 0;

#if defined(HAVE_AVX2)
    p.cpu_features |= HS_CPU_FEATURES_AVX2;
#endif

#if defined(HAVE_AVX512)
    p.cpu_features |= HS_CPU_FEATURES_AVX512;
#endif

    platform_t pp = target_to_platform(target_t(p));
    ASSERT_EQ(pp, hs_current_platform);
}

TEST(CRC, alignments) {
    std::array<u8, 4096> a;
    a.fill('a');

    // test the crc32c function at different alignments
    for (u8 i = 0; i < 32; i++) {
        u32 crc = Crc32c_ComputeBuf(0, (u8 *)a.data() + i, 4000);
        ASSERT_EQ(crc, 0x94f04377U);
    }
}
