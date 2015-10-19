/*
 * Copyright (c) 2015, Intel Corporation
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

#include "gtest/gtest.h"
#include "util/unaligned.h"

TEST(Unaligned, StoreLoad16) {
    char data[3];
    memset(data, 0, sizeof(data));
    char *ptr = &data[1];

    unaligned_store_u16(ptr, (u16)~0u);
    ASSERT_EQ((u16)~0u, unaligned_load_u16(ptr));
    unaligned_store_u16(ptr, 0u);
    ASSERT_EQ(0u, unaligned_load_u16(ptr));
    unaligned_store_u16(ptr, (u16)0xf0f0u);
    ASSERT_EQ(0xf0f0u, unaligned_load_u16(ptr));

    for (u32 i = 0; i < 16; i++) {
        const u16 val = 0x1u << i;
        memset(data, 0xff, sizeof(data));
        unaligned_store_u16(ptr, val);
        ASSERT_EQ(val, unaligned_load_u16(ptr));
    }
}

TEST(Unaligned, StoreLoad32) {
    char data[5];
    memset(data, 0, sizeof(data));
    char *ptr = &data[1];

    unaligned_store_u32(ptr, ~0u);
    ASSERT_EQ(~0u, unaligned_load_u32(ptr));
    unaligned_store_u32(ptr, 0u);
    ASSERT_EQ(0u, unaligned_load_u32(ptr));
    unaligned_store_u32(ptr, 0xf0f0f0f0u);
    ASSERT_EQ(0xf0f0f0f0u, unaligned_load_u32(ptr));

    for (u32 i = 0; i < 32; i++) {
        const u32 val = 0x1u << i;
        memset(data, 0xff, sizeof(data));
        unaligned_store_u32(ptr, val);
        ASSERT_EQ(val, unaligned_load_u32(ptr));
    }
}

TEST(Unaligned, StoreLoad64) {
    char data[9];
    memset(data, 0, sizeof(data));
    char *ptr = &data[1];

    unaligned_store_u64a(ptr, ~0ull);
    ASSERT_EQ(~0ull, unaligned_load_u64a(ptr));
    unaligned_store_u64a(ptr, 0ull);
    ASSERT_EQ(0ull, unaligned_load_u64a(ptr));
    unaligned_store_u64a(ptr, 0xf0f0f0f0f0f0f0f0ull);
    ASSERT_EQ(0xf0f0f0f0f0f0f0f0ull, unaligned_load_u64a(ptr));

    for (u32 i = 0; i < 64; i++) {
        const u64a val = 0x1ull << i;
        memset(data, 0xff, sizeof(data));
        unaligned_store_u64a(ptr, val);
        ASSERT_EQ(val, unaligned_load_u64a(ptr));
    }
}
