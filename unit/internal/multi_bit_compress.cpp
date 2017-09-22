/*
 * Copyright (c) 2017, Intel Corporation
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
#include "ue2common.h"
#include "util/compile_error.h"
#include "util/make_unique.h"
#include "util/multibit.h"
#include "util/multibit_build.h"
#include "util/multibit_compress.h"

using namespace std;
using namespace testing;
using namespace ue2;

/** \brief Print mmbit structure block by block. */
UNUSED
static
void mmbit_display(const u8 *bits, u32 total_bits) {
    for (u32 i = 0; i < mmbit_size(total_bits); i += 8) {
        printf("block %d:", i / 8);
        for (s32 j = 7; j >= 0; j--) {
            u8 a = (*(bits + i + j));
            printf(" %02x", a);
        }
        printf("\n");
    }
    printf("\n");
}

/** \brief Print an MMB_TYPE block. */
UNUSED
static
void mmbit_display_block(const u8 *bits) {
    for (s32 j = 7; j >= 0; j--) {
        u8 a = (*(bits + j));
        printf(" %02x", a);
    }
    printf("\n");
}

/** \brief Print mmbit structure block by block. */
UNUSED
static
void mmbit_display_comp(const u8 *bits, u32 comp_size) {
    for (u32 i = 0; i < comp_size; i += 8) {
        printf("block %d:", i / 8);
        for (s32 j = 7; j >= 0; j--) {
            u8 a = (*(bits + i + j));
            printf(" %02x", a);
        }
        printf("\n");
    }
    printf("\n");
}

namespace {
class mmbit_holder {
public:
    mmbit_holder() {}
    explicit mmbit_holder(u32 num_bits, u32 excess = 0)
        : data(ue2::make_unique<u8[]>(mmbit_size(num_bits) + 7 + excess)) {}
    void init(u32 num_bits) {
        assert(!data);
        data = ue2::make_unique<u8[]>(mmbit_size(num_bits) + 7);
    }
    operator u8 *() {
        assert(data);
        return data.get() + 7;
    }
    operator const u8 *() const {
        assert(data);
        return data.get() + 7;
    }

private:
    unique_ptr<u8[]> data = nullptr;
};

class comp_holder {
public:
    comp_holder() {}
    explicit comp_holder(u32 length)
        : data(ue2::make_unique<u8[]>(length + 7)) {}
    void init(u32 length) {
        assert(!data);
        data = ue2::make_unique<u8[]>(length + 7);
    }
    operator u8 *() {
        assert(data);
        return data.get() + 7;
    }
    operator const u8 *() const {
        assert(data);
        return data.get() + 7;
    }

private:
    unique_ptr<u8[]> data = nullptr;
};
}

static
void fill_mmbit(u8 *ba, u32 test_size) {
    fill_n(ba, mmbit_size(test_size), 0xff);
}

// We provide both test size and stride so that larger tests don't take forever
// checking every single key.
struct MultiBitCompTestParam {
    u32 size;
    u32 stride;
};

// Parameterized test case for bounded iterator, rather that propagating
// copypasta. Allocates space as given.
class MultiBitCompTest : public TestWithParam<MultiBitCompTestParam> {
protected:
    virtual void SetUp() {
        const MultiBitCompTestParam &p = GetParam();
        test_size = p.size;
        stride = p.stride;
        ba.init(test_size);
        // blast with ones for the lulz
        fill_mmbit(ba, test_size);
    }

    virtual void TearDown() {}

    u32 test_size; // number of bits in the multibit
    u32 stride; // stride to use for scans
    mmbit_holder ba; // multibit storage
};

TEST(MultiBitComp, CompCompsizeSparse) {
    static const u32 test_set[] = {
        257,
        4097,
        (1U << 18) + 1,
        (1U << 24) + 1,
        (1U << 30) + 1
    };
    for (u32 i = 0; i < 5; i++) {
        u32 test_size = test_set[i];
        mmbit_holder ba(test_size);

        // Clear all.
        mmbit_clear(ba, test_size);
        ASSERT_EQ(sizeof(MMB_TYPE), mmbit_compsize(ba, test_size));

        // Switch 3 bits on.
        mmbit_set(ba, test_size, 0);
        mmbit_set(ba, test_size, test_size / 2);
        mmbit_set(ba, test_size, test_size - 1);

        switch(test_size){
        case 257:
            ASSERT_EQ(sizeof(MMB_TYPE) * 4, mmbit_compsize(ba, test_size));
            break;
        case 4097:
            ASSERT_EQ(sizeof(MMB_TYPE) * 6, mmbit_compsize(ba, test_size));
            break;
        case (1U << 18) + 1:
            ASSERT_EQ(sizeof(MMB_TYPE) * 9, mmbit_compsize(ba, test_size));
            break;
        case (1U << 24) + 1:
            ASSERT_EQ(sizeof(MMB_TYPE) * 12, mmbit_compsize(ba, test_size));
            break;
        case (1U << 30) + 1:
            ASSERT_EQ(sizeof(MMB_TYPE) * 15, mmbit_compsize(ba, test_size));
            break;
        }
        size_t comp_size = mmbit_compsize(ba, test_size);

        // Switch 3 bits off.
        mmbit_unset(ba, test_size, 0);
        mmbit_unset(ba, test_size, test_size / 2);
        mmbit_unset(ba, test_size, test_size - 1);

        ASSERT_TRUE(mmbit_any(ba, test_size));
        ASSERT_FALSE(mmbit_any_precise(ba, test_size));

        ASSERT_EQ(comp_size, mmbit_compsize(ba, test_size));

        // Clear all again.
        mmbit_clear(ba, test_size);

        ASSERT_FALSE(mmbit_any(ba, test_size));
        ASSERT_FALSE(mmbit_any_precise(ba, test_size));

        ASSERT_EQ(sizeof(MMB_TYPE), mmbit_compsize(ba, test_size));
    }
}

TEST(MultiBitComp, CompCompsizeDense) {
    static const u32 test_set[] = {
        257,
        4097,
        (1U << 18) + 1,
        (1U << 24) + 1,
        (1U << 30) + 1
    };
    for (u32 i = 0; i < 5; i++) {
        u32 test_size = test_set[i];
        mmbit_holder ba(test_size);

        // Fill all. (fill_mmbit() is not feasible.)
        //fill_mmbit(ba, test_size);
        mmbit_init_range(ba, test_size, 0, test_size);

        switch(test_size){
        case 257:
            ASSERT_EQ(sizeof(MMB_TYPE) * (1 + 5),
                      mmbit_compsize(ba, test_size));
            break;
        case 4097:
            ASSERT_EQ(sizeof(MMB_TYPE) * (3 + (1 + 64)),
                      mmbit_compsize(ba, test_size));
            break;
        case (1U << 18) + 1:
            ASSERT_EQ(sizeof(MMB_TYPE) * (4 + (1 + 64 + 4096)),
                      mmbit_compsize(ba, test_size));
            break;
        case (1U << 24) + 1:
            ASSERT_EQ(sizeof(MMB_TYPE) * (5 + (1 + 64 + 4096 + (1U << 18))),
                      mmbit_compsize(ba, test_size));
            break;
        case (1U << 30) + 1:
            ASSERT_EQ(sizeof(MMB_TYPE) * (6 + (1 + 64 + 4096 + (1U << 18) +
                      (1U << 24))), mmbit_compsize(ba, test_size));
            break;
        }
        size_t comp_size = mmbit_compsize(ba, test_size);

        // Switch 3 bits off.
        mmbit_unset(ba, test_size, 0);
        mmbit_unset(ba, test_size, test_size / 2);
        mmbit_unset(ba, test_size, test_size - 1);

        ASSERT_EQ(comp_size, mmbit_compsize(ba, test_size));

        // Switch all bits off, not a clear-up.
        mmbit_unset_range(ba, test_size, 0, test_size);

        ASSERT_TRUE(mmbit_any(ba, test_size));
        ASSERT_FALSE(mmbit_any_precise(ba, test_size));

        ASSERT_EQ(comp_size, mmbit_compsize(ba, test_size));
    }
}

TEST_P(MultiBitCompTest, CompCompressDecompressSparse) {
    SCOPED_TRACE(test_size);
    ASSERT_TRUE(ba != nullptr);

    // 1st active range --> empty
    mmbit_clear(ba, test_size);

    // op 2.
    // 2nd active range --> [1/5, 1/3)
    u64a begin = test_size / 5;
    u64a end = test_size / 3;
    for (u64a i = begin; i < end; i++) {
        mmbit_set(ba, test_size, i);
    }

    // op 3.
    // 3rd active range --> [1/5, 1/2)
    begin = test_size / 4;
    end = test_size / 2;
    for (u64a i = begin; i < end; i++) {
        mmbit_set(ba, test_size, i);
    }

    // op 4.
    // 4th active range --> [1/5, 1/4) and [1/3, 1/2)
    begin = test_size / 4;
    end = test_size / 3;
    mmbit_unset_range(ba, test_size, begin, end);

    // op 5.
    // 5th active range --> empty
    mmbit_clear(ba, test_size);

    // op 6.
    // 6th active range --> [1/4, 1/3)
    for (u64a i = begin; i < end; i++) {
        mmbit_set(ba, test_size, i);
    }

    // Initialize compression space.
    size_t comp_size = mmbit_compsize(ba, test_size);
    comp_holder ca(comp_size);
    ASSERT_EQ(1, mmbit_compress(ba, test_size, ca, &comp_size, comp_size));

    // Initialize decompression space.
    mmbit_holder ba_1(test_size);
    fill_mmbit(ba_1, test_size); // Dirty decompression space.
    ASSERT_EQ(1, mmbit_decompress(ba_1, test_size, ca, &comp_size, comp_size));

    // Correctness checking, should be [1/4, 1/3).
    // And now, begin = test_size / 4, end = test_size / 3.
    for (u64a i = 0; i < test_size; i += stride) {
        if (i >= begin && i < end) {
            ASSERT_TRUE(mmbit_isset(ba_1, test_size, i));
        } else {
            ASSERT_FALSE(mmbit_isset(ba_1, test_size, i));
        }
    }
}

TEST_P(MultiBitCompTest, CompCompressDecompressDense) {
    SCOPED_TRACE(test_size);
    ASSERT_TRUE(ba != nullptr);

    ASSERT_TRUE(mmbit_all(ba, test_size));

    // Sequence of set/unset/clear operations.
    // op 1.
    // 1st active range --> [0, 1/4) and [1/3, 1)
    u64a begin = test_size / 4;
    u64a end = test_size / 3;
    mmbit_unset_range(ba, test_size, begin, end);

    // op 2.
    // 2st active range --> empty
    mmbit_clear(ba, test_size);

    // op 3.
    // 3rd active range --> [1/5, 1/2)
    begin = test_size / 5;
    end = test_size / 2;
    for (u64a i = begin; i < end; i++) {
        mmbit_set(ba, test_size, i);
    }

    // op 4.
    // 4th active range --> [1/3, 1/2)
    end = test_size / 3;
    mmbit_unset_range(ba, test_size, begin, end);

    // op 5.
    //5th active range --> empty
    begin = test_size / 4;
    end = test_size / 2;
    mmbit_unset_range(ba, test_size, begin, end);

    // Initialize compression space.
    size_t comp_size = mmbit_compsize(ba, test_size);
    comp_holder ca(comp_size);
    ASSERT_EQ(1, mmbit_compress(ba, test_size, ca, &comp_size, comp_size));

    // Initialize decompression space.
    mmbit_holder ba_1(test_size);
    fill_mmbit(ba_1, test_size); // Dirty decompression space.
    ASSERT_EQ(1, mmbit_decompress(ba_1, test_size, ca, &comp_size, comp_size));

    // Correctness checking, should be empty.
    if (test_size <= MMB_FLAT_MAX_BITS) {
        ASSERT_FALSE(mmbit_any(ba, test_size));
        ASSERT_FALSE(mmbit_any(ba_1, test_size));
    } else {
        ASSERT_TRUE(mmbit_any(ba, test_size));
        ASSERT_TRUE(mmbit_any(ba_1, test_size));
    }
    ASSERT_FALSE(mmbit_any_precise(ba, test_size));
    ASSERT_FALSE(mmbit_any_precise(ba_1, test_size));
}

TEST(MultiBitComp, CompIntegration1) {
    // 256 + 1 --> smallest 2-level mmbit
    u32 total_size = mmbit_size(257);
    mmbit_holder ba(257);

    //-------------------- 1 -----------------------//
    // Operate on mmbit
    mmbit_init_range(ba, 257, 0, 100);
    // Compress
    size_t comp_size = mmbit_compsize(ba, 257);
    comp_holder ca(comp_size);
    ASSERT_EQ(1, mmbit_compress(ba, 257, ca, &comp_size, comp_size));
    // Decompress
    mmbit_holder ba_1(257);
    ASSERT_EQ(1, mmbit_decompress(ba_1, 257, ca, &comp_size, comp_size));
    // Check set range: [0,100)
    for (u64a i = 0; i < 257; i++) {
        if (i < 100) {
            ASSERT_TRUE(mmbit_isset(ba_1, 257, i));
        } else {
            ASSERT_FALSE(mmbit_isset(ba_1, 257, i));
        }
    }

    //-------------------- 2 -----------------------//
    // Operate on mmbit
    for (u64a i = 190; i < 257; i++) {
        mmbit_set(ba_1, 257, i);
    }
    // Compress
    size_t comp_size_1 = mmbit_compsize(ba_1, 257);
    comp_holder ca_1(comp_size_1);
    ASSERT_EQ(1, mmbit_compress(ba_1, 257, ca_1, &comp_size_1, comp_size_1));
    // Decompress
    mmbit_holder ba_2(257);
    ASSERT_EQ(1, mmbit_decompress(ba_2, 257, ca_1, &comp_size_1, comp_size_1));
    // Check set range: [0,100) and [190,257)
    for (u64a i = 0; i < 257; i++) {
        if (i < 100 || i >= 190) {
            ASSERT_TRUE(mmbit_isset(ba_2, 257, i));
        } else {
            ASSERT_FALSE(mmbit_isset(ba_2, 257, i));
        }
    }

    //-------------------- 3 -----------------------//
    // Operate on mmbit
    mmbit_unset_range(ba_2, 257, 190, 192);
    // Compress
    size_t comp_size_2 = mmbit_compsize(ba_2, 257);
    comp_holder ca_2(comp_size_2);
    ASSERT_EQ(1, mmbit_compress(ba_2, 257, ca_2, &comp_size_2, comp_size_2));
    // Decompress
    mmbit_holder ba_3(257);
    ASSERT_EQ(1, mmbit_decompress(ba_3, 257, ca_2, &comp_size_2, comp_size_2));
    // Check set range: [0,100) and [192,257)
    for (u64a i = 0; i < 257; i++) {
        if (i < 100 || i >= 192) {
            ASSERT_TRUE(mmbit_isset(ba_3, 257, i));
        } else {
            ASSERT_FALSE(mmbit_isset(ba_3, 257, i));
        }
    }

    //-------------------- 4 -----------------------//
    // Operate on mmbit
    for (u64a i = 100; i < 200; i++) {
        mmbit_set(ba_3, 257, i);
    }
    // Compress
    size_t comp_size_3 = mmbit_compsize(ba_3, 257);
    comp_holder ca_3(comp_size_3);
    ASSERT_EQ(1, mmbit_compress(ba_3, 257, ca_3, &comp_size_3, comp_size_3));
    // Decompress
    mmbit_holder ba_4(257);
    ASSERT_EQ(1, mmbit_decompress(ba_4, 257, ca_3, &comp_size_3, comp_size_3));
    // Check set range: full
    ASSERT_TRUE(mmbit_all(ba_4, 257));

    //-------------------- 5 -----------------------//
    // Operate on mmbit
    mmbit_clear(ba_4, 257);
    // Compress
    size_t comp_size_4 = mmbit_compsize(ba_4, 257);
    comp_holder ca_4(comp_size_4);
    ASSERT_EQ(1, mmbit_compress(ba_4, 257, ca_4, &comp_size_4, comp_size_4));
    // Decompress
    mmbit_holder ba_5(257);
    ASSERT_EQ(1, mmbit_decompress(ba_5, 257, ca_4, &comp_size_4, comp_size_4));
    // Check set range: empty
    ASSERT_FALSE(mmbit_any(ba_5, 257));
    ASSERT_FALSE(mmbit_any_precise(ba_5, 257));

    //-------------------- 6 -----------------------//
    // Operate on mmbit
    for (u64a i = 100; i < 200; i++) {
        mmbit_set(ba_5, 257, i);
    }
    // Compress
    size_t comp_size_5 = mmbit_compsize(ba_5, 257);
    comp_holder ca_5(comp_size_5);
    ASSERT_EQ(1, mmbit_compress(ba_5, 257, ca_5, &comp_size_5, comp_size_5));
    // Decompress
    mmbit_holder ba_6(257);
    ASSERT_EQ(1, mmbit_decompress(ba_6, 257, ca_5, &comp_size_5, comp_size_5));
    // Check set range: [100,200)
    for (u64a i = 0; i < 257; i++) {
        if (i >= 100 && i < 200) {
            ASSERT_TRUE(mmbit_isset(ba_6, 257, i));
        } else {
            ASSERT_FALSE(mmbit_isset(ba_6, 257, i));
        }
    }
}

TEST(MultiBitComp, CompIntegration2) {
    // 64^2 + 1 --> smallest 3-level mmbit
    u32 total_size = mmbit_size(4097);
    mmbit_holder ba(4097);

    //-------------------- 1 -----------------------//
    // Operate on mmbit
    mmbit_init_range(ba, 4097, 0, 3200);
    // Compress
    size_t comp_size = mmbit_compsize(ba, 4097);
    comp_holder ca(comp_size);
    ASSERT_EQ(1, mmbit_compress(ba, 4097, ca, &comp_size, comp_size));
    // Decompress
    mmbit_holder ba_1(4097);
    ASSERT_EQ(1, mmbit_decompress(ba_1, 4097, ca, &comp_size, comp_size));
    // Check set range: [0, 3200)
    for (u64a i = 0; i < 4097; i++) {
        if (i < 3200) {
            ASSERT_TRUE(mmbit_isset(ba_1, 4097, i));
        } else {
            ASSERT_FALSE(mmbit_isset(ba_1, 4097, i));
        }
    }

    //-------------------- 2 -----------------------//
    // Operate on mmbit
    mmbit_unset_range(ba_1, 4097, 320, 640);
    // Compress
    size_t comp_size_1 = mmbit_compsize(ba_1, 4097);
    comp_holder ca_1(comp_size_1);
    ASSERT_EQ(1, mmbit_compress(ba_1, 4097, ca_1, &comp_size_1, comp_size_1));
    // Decompress
    mmbit_holder ba_2(4097);
    ASSERT_EQ(1,
              mmbit_decompress(ba_2, 4097, ca_1, &comp_size_1, comp_size_1));
    // Check set range: [0, 320) and [640, 3200)
    for (u64a i = 0; i < 4097; i++) {
        if (i < 320 || (i >= 640 && i < 3200)) {
            ASSERT_TRUE(mmbit_isset(ba_2, 4097, i));
        } else {
            ASSERT_FALSE(mmbit_isset(ba_2, 4097, i));
        }
    }

    //-------------------- 3 -----------------------//
    // Operate on mmbit
    for (u64a i = 3000; i < 4000; i++) {
        mmbit_set(ba_2, 4097, i);
    }
    // Compress
    size_t comp_size_2 = mmbit_compsize(ba_2, 4097);
    comp_holder ca_2(comp_size_2);
    ASSERT_EQ(1, mmbit_compress(ba_2, 4097, ca_2, &comp_size_2, comp_size_2));
    // Decompress
    mmbit_holder ba_3(4097);
    ASSERT_EQ(1,
              mmbit_decompress(ba_3, 4097, ca_2, &comp_size_2, comp_size_2));
    // Check set range: [0, 320) and [640, 4000)
    for (u64a i = 0; i < 4097; i++) {
        if (i < 320 || (i >= 640 && i < 4000)) {
            ASSERT_TRUE(mmbit_isset(ba_3, 4097, i));
        } else {
            ASSERT_FALSE(mmbit_isset(ba_3, 4097, i));
        }
    }

    //-------------------- 4 -----------------------//
    // Operate on mmbit
    mmbit_unset(ba_3, 4097, 64);
    mmbit_unset(ba_3, 4097, 3200);
    // Compress
    size_t comp_size_3 = mmbit_compsize(ba_3, 4097);
    comp_holder ca_3(comp_size_3);
    ASSERT_EQ(1, mmbit_compress(ba_3, 4097, ca_3, &comp_size_3, comp_size_3));
    // Decompress
    mmbit_holder ba_4(4097);
    ASSERT_EQ(1,
              mmbit_decompress(ba_4, 4097, ca_3, &comp_size_3, comp_size_3));
    // Check set range: [0,64) and [65, 320) and [640, 3200) and [3201, 4000)
    for (u64a i = 0; i < 4097; i++) {
        if (i < 64 || (i >= 65 && i < 320) || (i >= 640 && i < 3200) ||
            (i >= 3201 && i < 4000)) {
            ASSERT_TRUE(mmbit_isset(ba_4, 4097, i));
        } else {
            ASSERT_FALSE(mmbit_isset(ba_4, 4097, i));
        }
    }

    //-------------------- 5 -----------------------//
    // Operate on mmbit
    for (u64a i = 0; i < 4097; i++) {
        if (i < 64 || (i >= 65 && i < 320) || (i >= 640 && i < 3200) ||
            (i >= 3201 && i < 4000)) {
            mmbit_unset(ba_4, 4097, i);
        }
    }
    // Compress
    size_t comp_size_4 = mmbit_compsize(ba_4, 4097);
    comp_holder ca_4(comp_size_4);
    ASSERT_EQ(1, mmbit_compress(ba_4, 4097, ca_4, &comp_size_4, comp_size_4));
    // Decompress
    mmbit_holder ba_5(4097);
    ASSERT_EQ(1,
              mmbit_decompress(ba_5, 4097, ca_4, &comp_size_4, comp_size_4));
    // Check set range: empty
    ASSERT_TRUE(mmbit_any(ba_5, 4097));
    ASSERT_FALSE(mmbit_any_precise(ba_5, 4097));

    //-------------------- 6 -----------------------//
    // Operate on mmbit
    mmbit_set(ba_5, 4097, 4096);
    // Compress
    size_t comp_size_5 = mmbit_compsize(ba_5, 4097);
    comp_holder ca_5(comp_size_5);
    ASSERT_EQ(1, mmbit_compress(ba_5, 4097, ca_5, &comp_size_5, comp_size_5));
    // Decompress
    mmbit_holder ba_6(4097);
    ASSERT_EQ(1,
              mmbit_decompress(ba_6, 4097, ca_5, &comp_size_5, comp_size_5));
    // Check set range: [4096, 4096]
    for (u64a i = 0; i < 4097; i++) {
        if (i == 4096) {
             ASSERT_TRUE(mmbit_isset(ba_6, 4097, i));
        } else {
             ASSERT_FALSE(mmbit_isset(ba_6, 4097, i));
        }
    }
}

TEST(MultiBitComp, CompIntegration3) {
    // 64^3 + 1 --> smallest 4-level mmbit
    u32 total_size = mmbit_size(262145);
    mmbit_holder ba(262145);

    //-------------------- 1 -----------------------//
    // Operate on mmbit
    mmbit_init_range(ba, 262145, 0, 262145);
    // Compress
    size_t comp_size = mmbit_compsize(ba, 262145);
    comp_holder ca(comp_size);
    ASSERT_EQ(1, mmbit_compress(ba, 262145, ca, &comp_size, comp_size));
    // Decompress
    mmbit_holder ba_1(262145);
    ASSERT_EQ(1, mmbit_decompress(ba_1, 262145, ca, &comp_size, comp_size));
    // Check set range: full
    ASSERT_TRUE(mmbit_all(ba_1, 262145));

    //-------------------- 2 -----------------------//
    // Operate on mmbit
    mmbit_unset_range(ba_1, 262145, 0, 64000);
    // Compress
    size_t comp_size_1 = mmbit_compsize(ba_1, 262145);
    comp_holder ca_1(comp_size_1);
    ASSERT_EQ(1,
              mmbit_compress(ba_1, 262145, ca_1, &comp_size_1, comp_size_1));
    // Decompress
    mmbit_holder ba_2(262145);
    ASSERT_EQ(1,
              mmbit_decompress(ba_2, 262145, ca_1, &comp_size_1, comp_size_1));
    // Check set range: [64000, 262145)
    for (u64a i = 0; i < 262145; i++) {
        if (i < 64000) {
            ASSERT_FALSE(mmbit_isset(ba_2, 262145, i));
        } else {
            ASSERT_TRUE(mmbit_isset(ba_2, 262145, i));
        }
    }

    //-------------------- 3 -----------------------//
    // Operate on mmbit
    mmbit_unset_range(ba_2, 262145, 64001, 256000);
    // Compress
    size_t comp_size_2 = mmbit_compsize(ba_2, 262145);
    comp_holder ca_2(comp_size_2);
    ASSERT_EQ(1,
              mmbit_compress(ba_2, 262145, ca_2, &comp_size_2, comp_size_2));
    // Decompress
    mmbit_holder ba_3(262145);
    ASSERT_EQ(1,
              mmbit_decompress(ba_3, 262145, ca_2, &comp_size_2, comp_size_2));
    // Check set range: [64000, 64000] and [256000, 262145)
    for (u64a i = 0; i < 262145; i++) {
        if (i == 64000 || i >= 256000) {
            ASSERT_TRUE(mmbit_isset(ba_3, 262145, i));
        } else {
            ASSERT_FALSE(mmbit_isset(ba_3, 262145, i));
        }
    }

    //-------------------- 4 -----------------------//
    // Operate on mmbit
    mmbit_unset_range(ba_3, 262145, 256001, 262145);
    // Compress
    size_t comp_size_3 = mmbit_compsize(ba_3, 262145);
    comp_holder ca_3(comp_size_3);
    ASSERT_EQ(1,
              mmbit_compress(ba_3, 262145, ca_3, &comp_size_3, comp_size_3));
    // Decompress
    mmbit_holder ba_4(262145);
    ASSERT_EQ(1,
              mmbit_decompress(ba_4, 262145, ca_3, &comp_size_3, comp_size_3));
    // Check set range: [64000, 64000] and [256000, 256000]
    ASSERT_EQ(64000, mmbit_iterate(ba_4, 262145, MMB_INVALID));
    ASSERT_EQ(256000, mmbit_iterate(ba_4, 262145, 64000));
    ASSERT_EQ(MMB_INVALID, mmbit_iterate(ba_4, 262145, 256000));

    //-------------------- 5 -----------------------//
    // Operate on mmbit
    mmbit_unset(ba_4, 262145, 64000);
    mmbit_unset(ba_4, 262145, 256000);
    // Compress
    size_t comp_size_4 = mmbit_compsize(ba_4, 262145);
    comp_holder ca_4(comp_size_4);
    ASSERT_EQ(1,
              mmbit_compress(ba_4, 262145, ca_4, &comp_size_4, comp_size_4));
    // Decompress
    mmbit_holder ba_5(262145);
    ASSERT_EQ(1,
              mmbit_decompress(ba_5, 262145, ca_4, &comp_size_4, comp_size_4));
    // Check set range: empty
    ASSERT_TRUE(mmbit_any(ba_5, 262145));
    ASSERT_FALSE(mmbit_any_precise(ba_5, 262145));
}

static const MultiBitCompTestParam multibitCompTests[] = {
    // We provide both test size and stride so that larger tests don't take
    // forever checking every single key.

    // Small cases, stride 1.
	{ 4, 1 },
    { 7, 1 },
    { 8, 1 },
    { 13, 1 },
    { 16, 1 },
    { 17, 1 },
    { 32, 1 },
    { 33, 1 },
    { 57, 1 },
    { 64, 1 },
    { 65, 1 },
    { 100, 1 },
    { 128, 1 },
    { 200, 1 },
    { 256, 1 },
    { 257, 1 },     // 257 = 256 + 1
    { 302, 1 },
    { 1024, 1 },
    { 1025, 1 },
    { 2099, 1 },    // 4097 = 64 ^ 2 + 1
    { 4097, 1 },
    { 10000, 1 },
    { 32768, 1 },
    { 32769, 1 },
    { 200000, 1 },
    { 262145, 1 },  // 262145 = 64 * 3 + 1

    // Larger cases, bigger strides.
    { 1U << 19, 3701 },
    { 1U << 20, 3701 },
    { 1U << 21, 3701 },
    { 1U << 22, 3701 },
    { 1U << 23, 3701 },
    { 1U << 24, 3701 },
    { 1U << 25, 3701 },
    { 1U << 26, 3701 },
    { 1U << 27, 7919 },
    { 1U << 28, 15073 },
    { 1U << 29, 24413 },
    { 1U << 30, 50377 },
    { 1U << 31, 104729 },
};

INSTANTIATE_TEST_CASE_P(MultiBitComp, MultiBitCompTest,
                        ValuesIn(multibitCompTests));
