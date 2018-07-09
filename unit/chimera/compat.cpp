/*
 * Copyright (c) 2018, Intel Corporation
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

#include "gtest/gtest.h"
#include "chimera/ch.h"
#include "hs.h"

// We currently depend on our common (meaning) hash defines having the same
// values.
TEST(HybridCompat, Defines) {
    // flags
    EXPECT_EQ(HS_FLAG_CASELESS, CH_FLAG_CASELESS);
    EXPECT_EQ(HS_FLAG_DOTALL, CH_FLAG_DOTALL);
    EXPECT_EQ(HS_FLAG_MULTILINE, CH_FLAG_MULTILINE);
    EXPECT_EQ(HS_FLAG_SINGLEMATCH, CH_FLAG_SINGLEMATCH);
    EXPECT_EQ(HS_FLAG_UTF8, CH_FLAG_UTF8);
    EXPECT_EQ(HS_FLAG_UCP, CH_FLAG_UCP);

    // errors
    EXPECT_EQ(HS_SUCCESS, CH_SUCCESS);
    EXPECT_EQ(HS_INVALID, CH_INVALID);
    EXPECT_EQ(HS_NOMEM, CH_NOMEM);
    EXPECT_EQ(HS_SCAN_TERMINATED, CH_SCAN_TERMINATED);
    EXPECT_EQ(HS_COMPILER_ERROR, CH_COMPILER_ERROR);
    EXPECT_EQ(HS_DB_VERSION_ERROR, CH_DB_VERSION_ERROR);
    EXPECT_EQ(HS_DB_PLATFORM_ERROR, CH_DB_PLATFORM_ERROR);
    EXPECT_EQ(HS_DB_MODE_ERROR, CH_DB_MODE_ERROR);
    EXPECT_EQ(HS_BAD_ALIGN, CH_BAD_ALIGN);
    EXPECT_EQ(HS_BAD_ALLOC, CH_BAD_ALLOC);
    EXPECT_EQ(HS_SCRATCH_IN_USE, CH_SCRATCH_IN_USE);
}
