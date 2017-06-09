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

#include "gtest/gtest.h"
#include "ue2common.h"
#include "rose/rose_build_scatter.h"
#include "util/compile_error.h"
#include "util/make_unique.h"
#include "util/multibit.h"
#include "util/multibit_build.h"

#include <algorithm>
#include <memory>
#include <vector>

using namespace std;
using namespace testing;
using namespace ue2;

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
}

static
void fill_mmbit(u8 *ba, u32 test_size) {
    fill_n(ba, mmbit_size(test_size), 0xff);
}

TEST(MultiBit, Set1) {
    const u32 test_size = 32;
    mmbit_holder ba(test_size);

    fill_mmbit(ba, test_size);
    mmbit_clear(ba, test_size);

    ASSERT_FALSE(mmbit_any(ba, 32));

    ASSERT_FALSE(mmbit_isset(ba, 32, 0));
    mmbit_set(ba, 32, 0);
    ASSERT_TRUE(mmbit_isset(ba, 32, 0));

    ASSERT_TRUE(mmbit_any(ba, 32));

    ASSERT_FALSE(mmbit_isset(ba, 32, 31));
    mmbit_set(ba, 32, 31);
    ASSERT_TRUE(mmbit_isset(ba, 32, 31));
}

TEST(MultiBit, Set2) {
    const u32 test_size = 64;
    mmbit_holder ba(test_size);

    fill_mmbit(ba, test_size);
    mmbit_clear(ba, test_size);

    ASSERT_FALSE(mmbit_any(ba, 64));

    for (u32 i = 0; i < 64; i++) {
        ASSERT_FALSE(mmbit_isset(ba, 64, i));
    }

    ASSERT_FALSE(mmbit_isset(ba, 64, 0));
    mmbit_set(ba, 64, 0);
    ASSERT_TRUE(mmbit_isset(ba, 64, 0));
    for (u32 i = 1; i < 64; i++) {
        ASSERT_FALSE(mmbit_isset(ba, 64, i));
    }

    ASSERT_TRUE(mmbit_any(ba, 64));

    ASSERT_FALSE(mmbit_isset(ba, 64, 31));
    mmbit_set(ba, 64, 31);
    ASSERT_TRUE(mmbit_isset(ba, 64, 31));

    ASSERT_FALSE(mmbit_isset(ba, 64, 32));
    mmbit_set(ba, 64, 32);
    ASSERT_TRUE(mmbit_isset(ba, 64, 32));

    ASSERT_FALSE(mmbit_isset(ba, 64, 48));
    mmbit_set(ba, 64, 48);
    ASSERT_TRUE(mmbit_isset(ba, 64, 48));
}

TEST(MultiBit, Set3) {
    const u32 test_size = 1030;
    mmbit_holder ba(test_size);

    fill_mmbit(ba, test_size);
    mmbit_clear(ba, test_size);

    ASSERT_FALSE(mmbit_any(ba, test_size));

    ASSERT_FALSE(mmbit_isset(ba, test_size, 0));
    mmbit_set(ba, test_size, 0);
    ASSERT_TRUE(mmbit_isset(ba, test_size, 0));

    ASSERT_TRUE(mmbit_any(ba, test_size));

    ASSERT_FALSE(mmbit_isset(ba, test_size, 31));
    mmbit_set(ba, test_size, 31);
    ASSERT_TRUE(mmbit_isset(ba, test_size, 31));

    ASSERT_FALSE(mmbit_isset(ba, test_size, 32));
    mmbit_set(ba, test_size, 32);
    ASSERT_TRUE(mmbit_isset(ba, test_size, 32));

    ASSERT_FALSE(mmbit_isset(ba, test_size, 48));
    mmbit_set(ba, test_size, 48);
    ASSERT_TRUE(mmbit_isset(ba, test_size, 48));

    ASSERT_FALSE(mmbit_isset(ba, test_size, 60));
    mmbit_set(ba, test_size, 60);
    ASSERT_TRUE(mmbit_isset(ba, test_size, 60));

    ASSERT_FALSE(mmbit_isset(ba, test_size, 1029));
    mmbit_set(ba, test_size, 1029);
    ASSERT_TRUE(mmbit_isset(ba, test_size, 1029));
}

TEST(MultiBit, Set4) {
    const u32 test_size = 1025;
    mmbit_holder ba(test_size);

    mmbit_clear(ba, test_size);

    ASSERT_FALSE(mmbit_any(ba, test_size));

    for (u32 i = 0; i < test_size; i++) {
        ASSERT_FALSE(mmbit_isset(ba, test_size, i));
        mmbit_set(ba, test_size, i);
        ASSERT_TRUE(mmbit_isset(ba, test_size, i));

        ASSERT_EQ(i, mmbit_iterate(ba, test_size, MMB_INVALID));
        ASSERT_EQ(MMB_INVALID, mmbit_iterate(ba, test_size, i));

        mmbit_unset(ba, test_size, i);
        ASSERT_FALSE(mmbit_isset(ba, test_size, i));
    }
}

TEST(MultiBit, Set5) {
    const u32 test_size = 33000;
    mmbit_holder ba(test_size);

    mmbit_clear(ba, test_size);

    ASSERT_FALSE(mmbit_any(ba, test_size));

    for (u32 i = 0; i < test_size; i += 8) {
        ASSERT_FALSE(mmbit_isset(ba, test_size, i));
        mmbit_set(ba, test_size, i);
        ASSERT_TRUE(mmbit_isset(ba, test_size, i));

        ASSERT_EQ(i, mmbit_iterate(ba, test_size, MMB_INVALID));
        ASSERT_EQ(MMB_INVALID, mmbit_iterate(ba, test_size, i));

        mmbit_unset(ba, test_size, i);
        ASSERT_FALSE(mmbit_isset(ba, test_size, i));
    }
}

TEST(MultiBit, Set8) {
    const u32 test_size = 8;
    mmbit_holder ba(test_size);

    mmbit_clear(ba, test_size);

    ASSERT_FALSE(mmbit_any(ba, test_size));

    for (u32 i = 0; i < test_size; i++) {
        ASSERT_FALSE(mmbit_isset(ba, test_size, i));
        mmbit_set(ba, test_size, i);
        ASSERT_TRUE(mmbit_isset(ba, test_size, i));

        ASSERT_EQ(i, mmbit_iterate(ba, test_size, MMB_INVALID));
        ASSERT_EQ(MMB_INVALID, mmbit_iterate(ba, test_size, i));

        mmbit_unset(ba, test_size, i);
        ASSERT_FALSE(mmbit_isset(ba, test_size, i));
    }
}

TEST(MultiBit, Set16) {
    const u32 test_size = 16;
    mmbit_holder ba(test_size);

    mmbit_clear(ba, test_size);

    ASSERT_FALSE(mmbit_any(ba, test_size));

    for (u32 i = 0; i < test_size; i++) {
        ASSERT_FALSE(mmbit_isset(ba, test_size, i));
        mmbit_set(ba, test_size, i);
        ASSERT_TRUE(mmbit_isset(ba, test_size, i));

        ASSERT_EQ(i, mmbit_iterate(ba, test_size, MMB_INVALID));
        ASSERT_EQ(MMB_INVALID, mmbit_iterate(ba, test_size, i));

        mmbit_unset(ba, test_size, i);
        ASSERT_FALSE(mmbit_isset(ba, test_size, i));
    }
}


TEST(MultiBit, It1) {
    const u32 test_size = 32;
    mmbit_holder ba(test_size);

    mmbit_clear(ba, test_size);

    ASSERT_EQ(MMB_INVALID, mmbit_iterate(ba, test_size, MMB_INVALID));

    for (u32 i = 0; i < test_size; i+=3) {
        mmbit_set(ba, test_size, i);
    }

    ASSERT_EQ(0U, mmbit_iterate(ba, test_size, MMB_INVALID));
    for (u32 i = 0; i < 29; i++) {
        ASSERT_EQ(i/3 * 3 + 3, mmbit_iterate(ba, test_size, i));
    }

    for (u32 i = 0; i < test_size; i++) {
        mmbit_set(ba, test_size, i);
    }

    ASSERT_EQ(0U, mmbit_iterate(ba, test_size, MMB_INVALID));
    for (u32 i = 1; i < test_size; i++) {
        ASSERT_EQ(i, mmbit_iterate(ba, test_size, i - 1));
    }
}

TEST(MultiBit, It2) {
    mmbit_holder ba(64);

    mmbit_clear(ba, 64);

    ASSERT_EQ(MMB_INVALID, mmbit_iterate(ba, 64, MMB_INVALID));

    for (u32 i = 0; i < 64; i+=3) {
        mmbit_set(ba, 64, i);
    }

    ASSERT_EQ(0U, mmbit_iterate(ba, 64, MMB_INVALID));
    for (u32 i = 0; i < 62; i++) {
        ASSERT_EQ(i/3 * 3 + 3, mmbit_iterate(ba, 64, i));
    }

    for (u32 i = 0; i < 64; i++) {
        mmbit_set(ba, 64, i);
    }

    ASSERT_EQ(0U, mmbit_iterate(ba, 64, MMB_INVALID));
    for (u32 i = 1; i < 64; i++) {
        ASSERT_EQ(i, mmbit_iterate(ba, 64, i - 1));
    }
}

TEST(MultiBit, It3) {
    const size_t test_size = 60;
    mmbit_holder ba(test_size, 4);

    fill_n((u8 *)ba, mmbit_size(test_size) + 4, 0xff);

    mmbit_clear(ba, test_size);

    ASSERT_EQ(MMB_INVALID, mmbit_iterate(ba, test_size, MMB_INVALID));

    for (u32 i = 0; i < test_size; i+=3) {
        mmbit_set(ba, test_size, i);
    }

    ASSERT_EQ(0U, mmbit_iterate(ba, test_size, MMB_INVALID));
    for (u32 i = 0; i < test_size - 3; i++) {
        ASSERT_EQ(i/3 * 3 + 3, mmbit_iterate(ba, test_size, i));
    }

    for (u32 i = 0; i < test_size; i++) {
        mmbit_set(ba, test_size, i);
    }

    ASSERT_EQ(0U, mmbit_iterate(ba, test_size, MMB_INVALID));
    for (u32 i = 1; i < test_size; i++) {
        ASSERT_EQ(i, mmbit_iterate(ba, test_size, i - 1));
    }
}

// We provide both test size and stride so that larger tests don't take forever
// checking every single key.
struct MultiBitTestParam {
    u32 size;
    u32 stride;
};

// Parameterized test case for bounded iterator, rather that propagating
// copypasta. Allocates space as given.
class MultiBitTest : public TestWithParam<MultiBitTestParam> {
protected:
    virtual void SetUp() {
        const MultiBitTestParam &p = GetParam();
        test_size = p.size;
        stride = p.stride;
        ba.init(test_size);
        // blast with ones for the lulz
        fill_mmbit(ba, test_size);
    }

    virtual void TearDown() {
    }

    u32 test_size; // number of bits in the multibit
    u32 stride; // stride to use for scans
    mmbit_holder ba; // multibit storage
};

TEST_P(MultiBitTest, BoundedIteratorSingle) {
    SCOPED_TRACE(test_size);
    ASSERT_TRUE(ba != nullptr);

    // Set one bit on and run some checks.
    for (u64a i = 0; i < test_size; i += stride) {
        SCOPED_TRACE(i);

        mmbit_clear(ba, test_size);
        mmbit_set(ba, test_size, i);

        // Scanning from the start to our bit should find nothing.
        ASSERT_EQ(MMB_INVALID, mmbit_iterate_bounded(ba, test_size, i, i));

        // Scanning from the start to the end should find our bit.
        ASSERT_EQ(i, mmbit_iterate_bounded(ba, test_size, 0, test_size));

        // Scanning from our bit to one past it should find itself.
        ASSERT_EQ(i, mmbit_iterate_bounded(ba, test_size, i, i + 1));

        // Scanning from our bit to the end should find itself.
        ASSERT_EQ(i, mmbit_iterate_bounded(ba, test_size, i, test_size));

        // Scanning from one past our bit to the end should find nothing.
        if (i != test_size - 1) {
            // Ordinary iterator.
            ASSERT_EQ(MMB_INVALID, mmbit_iterate(ba, test_size, i));

            // Bounded iterator.
            ASSERT_EQ(MMB_INVALID,
                      mmbit_iterate_bounded(ba, test_size, i + 1, test_size));
        }
    }
}

TEST_P(MultiBitTest, BoundedIteratorAll) {
    SCOPED_TRACE(test_size);
    ASSERT_TRUE(ba != nullptr);

    // Switch everything on.
    fill_mmbit(ba, test_size);

    for (u64a i = 0; i < test_size; i += stride) {
        if (i != 0) {
            ASSERT_EQ(0U, mmbit_iterate_bounded(ba, test_size, 0, i));
        }
        ASSERT_EQ(i, mmbit_iterate_bounded(ba, test_size, i, i+1));
        ASSERT_EQ(i, mmbit_iterate_bounded(ba, test_size, i, test_size));
    }
}

TEST_P(MultiBitTest, BoundedIteratorEven) {
    SCOPED_TRACE(test_size);
    ASSERT_TRUE(ba != nullptr);

    // Set every even-numbered bit and see what we can see.
    mmbit_clear(ba, test_size);
    for (u64a i = 0; i < test_size; i += 2) {
        mmbit_set(ba, test_size, i);
    }

    u32 even_stride = stride % 2 ? stride + 1 : stride;

    for (u64a i = 0; i < test_size; i += even_stride) {
        // Scanning from each even bit to the end should find itself.
        ASSERT_EQ(i, mmbit_iterate_bounded(ba, test_size, i, test_size));

        // Scanning from i to i+1 should find itself.
        ASSERT_EQ(i, mmbit_iterate_bounded(ba, test_size, i, i+1));

        // Scanning from i+1 to i+2 should find nothing.
        if (i+1 < test_size) {
            ASSERT_EQ(MMB_INVALID, mmbit_iterate_bounded(ba, test_size, i+1, i+2));
        }

        // Scanning from i+1 to the end should find i+2.
        if (i+2 < test_size) {
            ASSERT_EQ(i+2, mmbit_iterate_bounded(ba, test_size, i+1, test_size));
        }
    }
}

TEST_P(MultiBitTest, BoundedIteratorOdd) {
    SCOPED_TRACE(test_size);
    ASSERT_TRUE(ba != nullptr);

    // Set every odd-numbered bit and see what we can see.
    mmbit_clear(ba, test_size);
    for (u64a i = 1; i < test_size; i += 2) {
        mmbit_set(ba, test_size, i);
    }

    u32 even_stride = stride % 2 ? stride + 1 : stride;

    for (u64a i = 0; i < test_size; i += even_stride) {
        // Scanning from each even bit to the end should find i+1.
        if (i+1 < test_size) {
            ASSERT_EQ(i+1, mmbit_iterate_bounded(ba, test_size, i, test_size));
        }

        // Scanning from i to i+1 should find nothing.
        ASSERT_EQ(MMB_INVALID, mmbit_iterate_bounded(ba, test_size, i, i+1));

        // Scanning from i+1 to i+2 should find i+1.
        if (i+1 < test_size) {
            ASSERT_EQ(i+1, mmbit_iterate_bounded(ba, test_size, i+1, i+2));
        }

        // Scanning from i+1 to the end should find i+1.
        if (i+1 < test_size) {
            ASSERT_EQ(i+1, mmbit_iterate_bounded(ba, test_size, i+1, test_size));
        }
    }
}

TEST_P(MultiBitTest, Set) {
    SCOPED_TRACE(test_size);
    ASSERT_TRUE(ba != nullptr);

    mmbit_clear(ba, test_size);
    ASSERT_FALSE(mmbit_any(ba, test_size));

    for (u64a i = 0; i < test_size; i += stride) {
        SCOPED_TRACE(i);

        // set a bit that wasn't set before
        ASSERT_FALSE(mmbit_isset(ba, test_size, i));
        ASSERT_FALSE(mmbit_set(ba, test_size, i));
        ASSERT_TRUE(mmbit_isset(ba, test_size, i));

        // set it again, should return true
        ASSERT_TRUE(mmbit_set(ba, test_size, i));

        // clear it and make sure it's gone
        mmbit_unset(ba, test_size, i);
        ASSERT_FALSE(mmbit_isset(ba, test_size, i));

        // set it again, should return false
        ASSERT_FALSE(mmbit_set(ba, test_size, i));
    }
}

TEST_P(MultiBitTest, Iter) {
    SCOPED_TRACE(test_size);
    ASSERT_TRUE(ba != nullptr);

    mmbit_clear(ba, test_size);
    ASSERT_EQ(MMB_INVALID, mmbit_iterate(ba, test_size, MMB_INVALID));

    for (u64a i = 0; i < test_size; i += stride) {
        SCOPED_TRACE(i);
        mmbit_clear(ba, test_size);
        mmbit_set(ba, test_size, i);
        ASSERT_EQ(i, mmbit_iterate(ba, test_size, MMB_INVALID));
        ASSERT_EQ(MMB_INVALID, mmbit_iterate(ba, test_size, i));
    }
}

TEST_P(MultiBitTest, IterAll) {
    SCOPED_TRACE(test_size);
    ASSERT_TRUE(ba != nullptr);

    mmbit_clear(ba, test_size);
    ASSERT_EQ(MMB_INVALID, mmbit_iterate(ba, test_size, MMB_INVALID));

    // Set all bits.
    for (u64a i = 0; i < test_size; i += stride) {
        mmbit_set(ba, test_size, i);
    }

    // Find all bits.
    u32 it = MMB_INVALID;
    for (u64a i = 0; i < test_size; i += stride) {
        ASSERT_EQ(i, mmbit_iterate(ba, test_size, it));
        it = i;
    }
}

TEST_P(MultiBitTest, AnyPrecise) {
    SCOPED_TRACE(test_size);
    ASSERT_TRUE(ba != nullptr);

    mmbit_clear(ba, test_size);
    ASSERT_FALSE(mmbit_any_precise(ba, test_size));

    for (u64a i = 0; i < test_size; i += stride) {
        SCOPED_TRACE(i);
        mmbit_clear(ba, test_size);
        mmbit_set(ba, test_size, i);
        ASSERT_TRUE(mmbit_any_precise(ba, test_size));
    }
}

TEST_P(MultiBitTest, Any) {
    SCOPED_TRACE(test_size);
    ASSERT_TRUE(ba != nullptr);

    mmbit_clear(ba, test_size);
    ASSERT_FALSE(mmbit_any(ba, test_size));

    for (u64a i = 0; i < test_size; i += stride) {
        SCOPED_TRACE(i);
        mmbit_clear(ba, test_size);
        mmbit_set(ba, test_size, i);
        ASSERT_TRUE(mmbit_any(ba, test_size));
    }
}

TEST_P(MultiBitTest, All) {
    SCOPED_TRACE(test_size);
    ASSERT_TRUE(ba != nullptr);

    mmbit_clear(ba, test_size);
    ASSERT_FALSE(mmbit_all(ba, test_size));

    for (u64a i = 0; i < test_size - 1; i += stride) {
        SCOPED_TRACE(i);
        mmbit_set(ba, test_size, i);
        ASSERT_FALSE(mmbit_all(ba, test_size));
    }

    // Set all bits.
    fill_mmbit(ba, test_size);
    ASSERT_TRUE(mmbit_all(ba, test_size));
}

TEST_P(MultiBitTest, UnsetRange1) {
    SCOPED_TRACE(test_size);
    ASSERT_TRUE(ba != nullptr);

    // Set all bits.
    fill_mmbit(ba, test_size);

    // Use mmbit_unset_range to switch off any single bit.
    for (u64a i = 0; i < test_size; i += stride) {
        SCOPED_TRACE(i);
        ASSERT_TRUE(mmbit_isset(ba, test_size, i));
        mmbit_unset_range(ba, test_size, i, i + 1);
        ASSERT_FALSE(mmbit_isset(ba, test_size, i));
    }
}

TEST_P(MultiBitTest, UnsetRange2) {
    SCOPED_TRACE(test_size);
    ASSERT_TRUE(ba != nullptr);

    // XXX: slow on large cases, so we skip those.
    if (stride > 1) {
        return;
    }

    // Set all bits.
    fill_mmbit(ba, test_size);

    // Use mmbit_unset_range to switch off all bits.
    mmbit_unset_range(ba, test_size, 0, test_size);

    for (u64a i = 0; i < test_size; i += stride) {
        SCOPED_TRACE(i);
        ASSERT_FALSE(mmbit_isset(ba, test_size, i));
    }
}

TEST_P(MultiBitTest, UnsetRange3) {
    SCOPED_TRACE(test_size);
    ASSERT_TRUE(ba != nullptr);

    // Use mmbit_unset_range to switch off bits in chunks of 3.
    for (u64a i = 0; i < test_size - 3; i += stride) {
        // Switch on the bit before, the bits in question, and the bit after.
        if (i > 0) {
            mmbit_set(ba, test_size, i - 1);
        }
        for (u64a j = i; j < min(i + 4, (u64a)test_size); j++) {
            mmbit_set(ba, test_size, j);
        }

        // Switch off the chunk.
        mmbit_unset_range(ba, test_size, i, i + 3);

        ASSERT_FALSE(mmbit_isset(ba, test_size, i));
        ASSERT_FALSE(mmbit_isset(ba, test_size, i + 1));
        ASSERT_FALSE(mmbit_isset(ba, test_size, i + 2));

        // Bits on either side should be intact.
        if (i > 0) {
            ASSERT_TRUE(mmbit_isset(ba, test_size, i - 1));
        }
        if (i + 3 < test_size) {
            ASSERT_TRUE(mmbit_isset(ba, test_size, i + 3));
        }
    }
}

TEST_P(MultiBitTest, InitRangeAll) {
    SCOPED_TRACE(test_size);
    ASSERT_TRUE(ba != nullptr);

    // Use mmbit_init_range to set all the bits.
    mmbit_init_range(ba, test_size, 0, test_size);

    // Make sure they're all set.
    for (u64a i = 0; i < test_size; i += stride) {
        SCOPED_TRACE(i);
        ASSERT_TRUE(mmbit_isset(ba, test_size, i));
    }
}

TEST_P(MultiBitTest, InitRangeNone) {
    SCOPED_TRACE(test_size);
    ASSERT_TRUE(ba != nullptr);

    // Use mmbit_set_range to set all the bits.
    mmbit_init_range(ba, test_size, 0, 0);

    // Make sure nothing's set.
    ASSERT_EQ(MMB_INVALID, mmbit_iterate(ba, test_size, MMB_INVALID));
}

TEST_P(MultiBitTest, InitRangeOne) {
    SCOPED_TRACE(test_size);
    ASSERT_TRUE(ba != nullptr);

    for (u64a i = 0; i < test_size; i += stride) {
        mmbit_init_range(ba, test_size, i, i + 1);

        // Only bit 'i' should be on.
        ASSERT_EQ(i, mmbit_iterate(ba, test_size, MMB_INVALID));
        ASSERT_EQ(MMB_INVALID, mmbit_iterate(ba, test_size, i));
    }
}

TEST_P(MultiBitTest, InitRangeChunked) {
    SCOPED_TRACE(test_size);
    ASSERT_TRUE(ba != nullptr);

    // Init ranges chunk by chunk.

    for (u32 n = 2; n <= 10; n++) {
        u32 chunk_size = test_size / n;
        if (chunk_size == 0) {
            break;
        }

        for (u32 k = 0; k < n; k++) {
            u32 chunk_begin = k * chunk_size;
            u32 chunk_end = min(test_size, (k + 1) * chunk_size);

            mmbit_init_range(ba, test_size, chunk_begin, chunk_end);

            // First bit set should be chunk_begin.
            ASSERT_EQ(chunk_begin, mmbit_iterate(ba, test_size, MMB_INVALID));

            // All bits in the chunk should be on.
            for (u64a i = chunk_begin; i < chunk_end; i += stride) {
                SCOPED_TRACE(i);
                ASSERT_TRUE(mmbit_isset(ba, test_size, i));
            }

            // Last bit on is chunk_end - 1.
            if (chunk_end) {
                ASSERT_EQ(MMB_INVALID, mmbit_iterate(ba, test_size, chunk_end - 1));
            }
        }
    }
}

static
void apply(const scatter_plan_raw &sp, u8 *out) {
    for (const auto &e : sp.p_u64a) {
        memcpy(out + e.offset, &e.val, sizeof(e.val));
    }
    for (const auto &e : sp.p_u32) {
        memcpy(out + e.offset, &e.val, sizeof(e.val));
    }
    for (const auto &e : sp.p_u16) {
        memcpy(out + e.offset, &e.val, sizeof(e.val));
    }
    for (const auto &e : sp.p_u8) {
        memcpy(out + e.offset, &e.val, sizeof(e.val));
    }
}

TEST_P(MultiBitTest, InitRangePlanChunked) {
    SCOPED_TRACE(test_size);
    ASSERT_TRUE(ba != nullptr);

    // Init ranges chunk by chunk.

    for (u32 n = 2; n <= 10; n++) {
        u32 chunk_size = test_size / n;
        if (chunk_size == 0) {
            break;
        }

        for (u32 k = 0; k < n; k++) {
            u32 chunk_begin = k * chunk_size;
            u32 chunk_end = min(test_size, (k + 1) * chunk_size);

            scatter_plan_raw sp;
            mmbBuildInitRangePlan(test_size, chunk_begin, chunk_end, &sp);
            memset(ba, 0xaa, mmbit_size(test_size));
            apply(sp, ba);

            // First bit set should be chunk_begin.
            ASSERT_EQ(chunk_begin, mmbit_iterate(ba, test_size, MMB_INVALID));

            // All bits in the chunk should be on.
            for (u64a i = chunk_begin; i < chunk_end; i += stride) {
                SCOPED_TRACE(i);
                ASSERT_TRUE(mmbit_isset(ba, test_size, i));
            }

            // Last bit on is chunk_end - 1.
            if (chunk_end) {
                ASSERT_EQ(MMB_INVALID, mmbit_iterate(ba, test_size, chunk_end - 1));
            }
        }
    }
}

TEST(MultiBit, SparseIteratorBegin1) {
    const u32 test_size = 100;
    vector<u32> bits;

    bits.push_back(1);
    bits.push_back(5);
    bits.push_back(27);
    bits.push_back(35);
    bits.push_back(68);

    auto it = mmbBuildSparseIterator(bits, test_size);
    //ASSERT_EQ(4U, it.size());

    // Trivial initial test: all bits in 'bits' are on, all others are off
    mmbit_holder ba(test_size);
    mmbit_clear(ba, test_size);
    for (size_t i = 0; i < bits.size(); i++) {
        mmbit_set(ba, test_size, bits[i]);
    }

    vector<mmbit_sparse_state> state(mmbit_sparse_iter_state_size(test_size));

    for (u32 i = 0; i < bits.size(); i++) {
        u32 val, idx = 0xffffffff;
        val = mmbit_sparse_iter_begin(ba, test_size, &idx, &it[0], &state[0]);
        ASSERT_EQ(bits[i], val);
        ASSERT_EQ(i, idx);
        mmbit_unset(ba, test_size, bits[i]);
        ASSERT_FALSE(mmbit_isset(ba, test_size, bits[i]));
    }

    // All clear, begin should be invalid
    u32 val, idx = 0xffffffff;
    val = mmbit_sparse_iter_begin(ba, test_size, &idx, &it[0], &state[0]);
    ASSERT_EQ(MMB_INVALID, val);
}

TEST(MultiBit, SparseIteratorBegin2) {
    const u32 test_size = 40000;
    vector<u32> bits;

    bits.push_back(1);
    bits.push_back(256);
    bits.push_back(257);
    bits.push_back(1024);
    bits.push_back(8920);
    bits.push_back(37000);

    auto it = mmbBuildSparseIterator(bits, test_size);
    //ASSERT_EQ(12U, it.size());

    // Trivial initial test: all bits in 'bits' are on, all others are off
    mmbit_holder ba(test_size);
    mmbit_clear(ba, test_size);
    for (size_t i = 0; i < bits.size(); i++) {
        mmbit_set(ba, test_size, bits[i]);
    }

    vector<mmbit_sparse_state> state(mmbit_sparse_iter_state_size(test_size));

    for (u32 i = 0; i < bits.size(); i++) {
        u32 val, idx = 0xffffffff;
        val = mmbit_sparse_iter_begin(ba, test_size, &idx, &it[0], &state[0]);
        ASSERT_EQ(bits[i], val);
        ASSERT_EQ(i, idx);
        mmbit_unset(ba, test_size, bits[i]);
        ASSERT_FALSE(mmbit_isset(ba, test_size, bits[i]));
    }

    // All clear, begin should be invalid
    u32 val, idx = 0xffffffff;
    val = mmbit_sparse_iter_begin(ba, test_size, &idx, &it[0], &state[0]);
    ASSERT_EQ(MMB_INVALID, val);
}

TEST(MultiBit, SparseIteratorNext1) {
    const u32 test_size = 100;
    vector<u32> bits;

    bits.push_back(1);
    bits.push_back(5);
    bits.push_back(27);
    bits.push_back(35);
    bits.push_back(68);

    auto it = mmbBuildSparseIterator(bits, test_size);

    // Trivial initial test: all bits in 'bits' are on, all others are off
    mmbit_holder ba(test_size);
    mmbit_clear(ba, test_size);
    for (size_t i = 0; i < bits.size(); i++) {
        mmbit_set(ba, test_size, bits[i]);
    }

    vector<mmbit_sparse_state> state(mmbit_sparse_iter_state_size(test_size));
    u32 val, idx = 0xffffffff;
    val = mmbit_sparse_iter_begin(ba, test_size, &idx, &it[0], &state[0]);
    ASSERT_EQ(bits[0], val);
    ASSERT_EQ(0U, idx);

    for (u32 i = 1; i < bits.size(); i++) {
        val = mmbit_sparse_iter_next(ba, test_size, val, &idx, &it[0], &state[0]);
        ASSERT_EQ(bits[i], val);
        ASSERT_EQ(i, idx);
    }

    // Same test with all bits on
    for (size_t i = 0; i < test_size; i++) {
        mmbit_set(ba, test_size, i);
    }

    val = mmbit_sparse_iter_begin(ba, test_size, &idx, &it[0], &state[0]);
    ASSERT_EQ(bits[0], val);
    ASSERT_EQ(0U, idx);

    for (u32 i = 1; i < bits.size(); i++) {
        val = mmbit_sparse_iter_next(ba, test_size, val, &idx, &it[0], &state[0]);
        ASSERT_EQ(bits[i], val);
        ASSERT_EQ(i, idx);
    }

    // Test with only the last half on
    mmbit_clear(ba, test_size);
    size_t last_half = bits.size()/2;
    for (size_t i = last_half; i < bits.size(); i++) {
        mmbit_set(ba, test_size, bits[i]);
    }

    val = mmbit_sparse_iter_begin(ba, test_size, &idx, &it[0], &state[0]);
    ASSERT_EQ(bits[last_half], val);
    ASSERT_EQ(last_half, idx);

    for (u32 i = last_half + 1; i < bits.size(); i++) {
        val = mmbit_sparse_iter_next(ba, test_size, val, &idx, &it[0], &state[0]);
        ASSERT_EQ(bits[i], val);
        ASSERT_EQ(i, idx);
    }
}

TEST(MultiBit, SparseIteratorNext2) {
    const u32 test_size = 40000;
    vector<u32> bits;

    bits.push_back(1);
    bits.push_back(256);
    bits.push_back(257);
    bits.push_back(1024);
    bits.push_back(1025);
    bits.push_back(4095);
    bits.push_back(4096);
    bits.push_back(4097);
    bits.push_back(8920);
    bits.push_back(37000);
    bits.push_back(39999);

    auto it = mmbBuildSparseIterator(bits, test_size);

    // Trivial initial test: all bits in 'bits' are on, all others are off
    mmbit_holder ba(test_size);
    mmbit_clear(ba, test_size);
    for (size_t i = 0; i < bits.size(); i++) {
        mmbit_set(ba, test_size, bits[i]);
    }

    vector<mmbit_sparse_state> state(mmbit_sparse_iter_state_size(test_size));
    u32 val, idx = 0xffffffff;
    val = mmbit_sparse_iter_begin(ba, test_size, &idx, &it[0], &state[0]);
    ASSERT_EQ(bits[0], val);
    ASSERT_EQ(0U, idx);

    for (u32 i = 1; i < bits.size(); i++) {
        val = mmbit_sparse_iter_next(ba, test_size, val, &idx, &it[0], &state[0]);
        ASSERT_EQ(bits[i], val);
        ASSERT_EQ(i, idx);
    }

    // Same test with all bits on
    for (size_t i = 0; i < test_size; i++) {
        mmbit_set(ba, test_size, i);
    }

    val = mmbit_sparse_iter_begin(ba, test_size, &idx, &it[0], &state[0]);
    ASSERT_EQ(bits[0], val);
    ASSERT_EQ(0U, idx);

    for (u32 i = 1; i < bits.size(); i++) {
        val = mmbit_sparse_iter_next(ba, test_size, val, &idx, &it[0], &state[0]);
        ASSERT_EQ(bits[i], val);
        ASSERT_EQ(i, idx);
    }

    // Test with only the last half on
    mmbit_clear(ba, test_size);
    size_t last_half = bits.size()/2;
    for (size_t i = last_half; i < bits.size(); i++) {
        mmbit_set(ba, test_size, bits[i]);
    }

    val = mmbit_sparse_iter_begin(ba, test_size, &idx, &it[0], &state[0]);
    ASSERT_EQ(bits[last_half], val);
    ASSERT_EQ(last_half, idx);

    for (u32 i = last_half + 1; i < bits.size(); i++) {
        val = mmbit_sparse_iter_next(ba, test_size, val, &idx, &it[0], &state[0]);
        ASSERT_EQ(bits[i], val);
        ASSERT_EQ(i, idx);
    }
}

TEST(MultiBit, SparseIteratorNextSmall) {
    const u32 test_size = 15;
    vector<u32> bits;

    bits.push_back(1);
    bits.push_back(3);
    bits.push_back(6);
    bits.push_back(7);
    bits.push_back(12);
    bits.push_back(14);

    auto it = mmbBuildSparseIterator(bits, test_size);

    // Trivial initial test: all bits in 'bits' are on, all others are off
    mmbit_holder ba(test_size);
    mmbit_clear(ba, test_size);
    for (size_t i = 0; i < bits.size(); i++) {
        mmbit_set(ba, test_size, bits[i]);
    }

    vector<mmbit_sparse_state> state(mmbit_sparse_iter_state_size(test_size));
    u32 val, idx = 0xffffffff;
    val = mmbit_sparse_iter_begin(ba, test_size, &idx, &it[0], &state[0]);
    ASSERT_EQ(bits[0], val);
    ASSERT_EQ(0U, idx);

    for (u32 i = 1; i < bits.size(); i++) {
        val = mmbit_sparse_iter_next(ba, test_size, val, &idx, &it[0], &state[0]);
        ASSERT_EQ(bits[i], val);
        ASSERT_EQ(i, idx);
    }

    // Same test with all bits on
    for (size_t i = 0; i < test_size; i++) {
        mmbit_set(ba, test_size, i);
    }

    val = mmbit_sparse_iter_begin(ba, test_size, &idx, &it[0], &state[0]);
    ASSERT_EQ(bits[0], val);
    ASSERT_EQ(0U, idx);

    for (u32 i = 1; i < bits.size(); i++) {
        val = mmbit_sparse_iter_next(ba, test_size, val, &idx, &it[0], &state[0]);
        ASSERT_EQ(bits[i], val);
        ASSERT_EQ(i, idx);
    }

    // Test with only the last half on
    mmbit_clear(ba, test_size);
    size_t last_half = bits.size()/2;
    for (size_t i = last_half; i < bits.size(); i++) {
        mmbit_set(ba, test_size, bits[i]);
    }

    val = mmbit_sparse_iter_begin(ba, test_size, &idx, &it[0], &state[0]);
    ASSERT_EQ(bits[last_half], val);
    ASSERT_EQ(last_half, idx);

    for (u32 i = last_half + 1; i < bits.size(); i++) {
        val = mmbit_sparse_iter_next(ba, test_size, val, &idx, &it[0], &state[0]);
        ASSERT_EQ(bits[i], val);
        ASSERT_EQ(i, idx);
    }
}

TEST_P(MultiBitTest, SparseIteratorBeginAll) {
    SCOPED_TRACE(test_size);
    ASSERT_TRUE(ba != nullptr);

    // Put all our bits into the sparse iterator.
    vector<u32> bits;
    bits.reserve(test_size / stride);
    for (u64a i = 0; i < test_size; i += stride) {
        bits.push_back(i);
    }
    auto it = mmbBuildSparseIterator(bits, test_size);

    // Switch all bits on in state.
    mmbit_clear(ba, test_size);
    ASSERT_FALSE(mmbit_any(ba, test_size));
    fill_mmbit(ba, test_size);

    // Iterate over all bits; clear the first one and ensure that the iterator
    // reports the next bit as the start bit.
    vector<mmbit_sparse_state> state(mmbit_sparse_iter_state_size(test_size));
    u32 val, idx = 0xffffffff;

    for (size_t i = 0, num_keys = bits.size(); i < num_keys; i++) {
        val = mmbit_sparse_iter_begin(ba, test_size, &idx, &it[0], &state[0]);
        ASSERT_EQ(bits[i], val);
        ASSERT_EQ(i, idx);
        mmbit_unset(ba, test_size, bits[i]);
    }

    // Cleared all bits, iterator should return invalid.
    val = mmbit_sparse_iter_begin(ba, test_size, &idx, &it[0], &state[0]);
    ASSERT_EQ(MMB_INVALID, val);
}

TEST_P(MultiBitTest, SparseIteratorBeginThirds) {
    SCOPED_TRACE(test_size);
    ASSERT_TRUE(ba != nullptr);

    // XXX: only run on stride=1 cases for speed.
    if (stride > 1) {
        return;
    }

    // Put all our bits into the sparse iterator
    vector<u32> bits(test_size);
    for (u32 i = 0; i != test_size; i++) {
        bits[i] = i;
    }
    auto it = mmbBuildSparseIterator(bits, test_size);

    // Switch every third bits on in state
    mmbit_clear(ba, test_size);
    ASSERT_FALSE(mmbit_any(ba, test_size));
    for (u64a i = 0; i < test_size; i += 3) {
        mmbit_set(ba, test_size, i);
    }

    // Iterate over all bits; clear the first on bit and ensure that the
    // iterator reports the next on bit as the start bit.
    vector<mmbit_sparse_state> state(mmbit_sparse_iter_state_size(test_size));
    u32 val, idx = 0xffffffff;
    val = mmbit_sparse_iter_begin(ba, test_size, &idx, &it[0], &state[0]);
    ASSERT_EQ(0U, val);
    ASSERT_EQ(0U, idx);

    for (u64a i = 0; i < test_size - 3; i += 3) {
        mmbit_unset(ba, test_size, i);
        val = mmbit_sparse_iter_begin(ba, test_size, &idx, &it[0], &state[0]);
        ASSERT_EQ(i+3, val);
        ASSERT_EQ(i+3, idx);
    }
}

TEST_P(MultiBitTest, SparseIteratorNextAll) {
    SCOPED_TRACE(test_size);
    ASSERT_TRUE(ba != nullptr);

    // Put all our bits into the sparse iterator.
    vector<u32> bits;
    bits.reserve(test_size / stride);
    for (u64a i = 0; i < test_size; i += stride) {
        bits.push_back(i);
    }
    auto it = mmbBuildSparseIterator(bits, test_size);

    // Switch all bits on in state
    mmbit_clear(ba, test_size);
    ASSERT_FALSE(mmbit_any(ba, test_size));
    fill_mmbit(ba, test_size);

    // Iterate over all bits.
    vector<mmbit_sparse_state> state(mmbit_sparse_iter_state_size(test_size));
    u32 val, idx = 0xffffffff;
    val = mmbit_sparse_iter_begin(ba, test_size, &idx, &it[0], &state[0]);
    ASSERT_EQ(0U, val);
    ASSERT_EQ(0U, idx);

    for (u32 i = 1, num_keys = bits.size(); i < num_keys; i++) {
        val = mmbit_sparse_iter_next(ba, test_size, val, &idx, &it[0], &state[0]);
        ASSERT_EQ(bits[i], val);
        ASSERT_EQ(i, idx);
    }

    // All bits tested, we should now return invalid
    val = mmbit_sparse_iter_next(ba, test_size, bits.back(), &idx, &it[0], &state[0]);
    ASSERT_EQ(MMB_INVALID, val);
}

TEST_P(MultiBitTest, SparseIteratorNextExactStrided) {
    SCOPED_TRACE(test_size);
    ASSERT_TRUE(ba != nullptr);

    if (stride == 1) {
        // This test is the same as SparseIteratorNextAll for stride 1
        return;
    }

    // Put all our bits into the sparse iterator and switch them on in the
    // state.
    mmbit_clear(ba, test_size);
    vector<u32> bits;
    bits.reserve(test_size / stride);
    for (u64a i = 0; i < test_size; i += stride) {
        bits.push_back(i);
        mmbit_set(ba, test_size, i);
    }
    auto it = mmbBuildSparseIterator(bits, test_size);

    // Iterate over all bits.
    vector<mmbit_sparse_state> state(mmbit_sparse_iter_state_size(test_size));
    u32 val, idx = 0xffffffff;
    val = mmbit_sparse_iter_begin(ba, test_size, &idx, &it[0], &state[0]);
    ASSERT_EQ(0U, val);
    ASSERT_EQ(0U, idx);

    for (u32 i = 1, num_keys = bits.size(); i < num_keys; i++) {
        val = mmbit_sparse_iter_next(ba, test_size, val, &idx, &it[0], &state[0]);
        ASSERT_EQ(bits[i], val);
        ASSERT_EQ(i, idx);
    }

    // All bits tested, we should now return invalid
    val = mmbit_sparse_iter_next(ba, test_size, bits.back(), &idx, &it[0], &state[0]);
    ASSERT_EQ(MMB_INVALID, val);
}

TEST_P(MultiBitTest, SparseIteratorNextNone) {
    SCOPED_TRACE(test_size);
    ASSERT_TRUE(ba != nullptr);

    // Put all our bits into the sparse iterator.
    vector<u32> bits;
    bits.reserve(test_size / stride);
    for (u64a i = 0; i < test_size; i += stride) {
        bits.push_back(i);
    }
    auto it = mmbBuildSparseIterator(bits, test_size);

    // Switch only the first bit on
    mmbit_clear(ba, test_size);
    mmbit_set(ba, test_size, 0);

    // Begin should find first bit
    vector<mmbit_sparse_state> state(mmbit_sparse_iter_state_size(test_size));
    u32 val, idx = 0xffffffff;
    val = mmbit_sparse_iter_begin(ba, test_size, &idx, &it[0], &state[0]);
    ASSERT_EQ(0U, val);
    ASSERT_EQ(0U, idx);

    // Next should return invalid
    val = mmbit_sparse_iter_next(ba, test_size, val, &idx, &it[0], &state[0]);
    ASSERT_EQ(MMB_INVALID, val);
}

TEST_P(MultiBitTest, SparseIteratorUnsetAll) {
    SCOPED_TRACE(test_size);
    ASSERT_TRUE(ba != nullptr);

    // Put all our bits into the sparse iterator
    vector<u32> bits;
    bits.reserve(test_size / stride);
    for (u64a i = 0; i < test_size; i += stride) {
        bits.push_back(i);
    }
    auto it = mmbBuildSparseIterator(bits, test_size);

    // Switch all bits on
    mmbit_clear(ba, test_size);
    fill_mmbit(ba, test_size);

    // Run iterator unset, which should clear all bits
    struct mmbit_sparse_state sparse_iter_state[MAX_SPARSE_ITER_STATES];
    mmbit_sparse_iter_unset(ba, test_size, &it[0], sparse_iter_state);

    // All bits should now be off
    for (u32 i = 0, num_keys = bits.size(); i < num_keys; i++) {
        ASSERT_FALSE(mmbit_isset(ba, test_size, bits[i]));
    }
}

TEST_P(MultiBitTest, SparseIteratorUnsetHalves) {
    SCOPED_TRACE(test_size);
    ASSERT_TRUE(ba != nullptr);

    // TODO: write useful striding version of this test.
    if (stride > 1) {
        return;
    }

    // Two sparse iterators: one for even bits, one for odd ones
    vector<u32> even, odd;
    for (u64a i = 0; i < test_size; i += 2) {
        even.push_back(i);
    }
    for (u64a i = 1; i < test_size; i += 2) {
        odd.push_back(i);
    }

    auto it_even = mmbBuildSparseIterator(even, test_size);
    auto it_odd = mmbBuildSparseIterator(odd, test_size);

    // Switch all bits on
    mmbit_clear(ba, test_size);
    for (u32 i = 0; i != test_size; i++) {
        mmbit_set(ba, test_size, i);
        ASSERT_TRUE(mmbit_isset(ba, test_size, i));
    }

    // Run even iterator unset, which should clear all even bits
    struct mmbit_sparse_state sparse_iter_state[MAX_SPARSE_ITER_STATES];
    mmbit_sparse_iter_unset(ba, test_size, &it_even[0], sparse_iter_state);

    // All even bits are off, all odd bits are on
    for (u32 i = 0; i != test_size; i++) {
        if ((i % 2) == 0) {
            ASSERT_FALSE(mmbit_isset(ba, test_size, i));
        } else {
            ASSERT_TRUE(mmbit_isset(ba, test_size, i));
        }
    }

    // Run odd iterator unset, clearing the odd bits
    mmbit_sparse_iter_unset(ba, test_size, &it_odd[0], sparse_iter_state);

    // All bits should now be off
    for (u32 i = 0; i != test_size; i++) {
        ASSERT_FALSE(mmbit_isset(ba, test_size, i));
    }
}

static const MultiBitTestParam multibitTests[] = {
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
    { 302, 1 },
    { 1024, 1 },
    { 1025, 1 },
    { 2099, 1 },
    { 10000, 1 },
    { 32768, 1 },
    { 32769, 1 },
    { 200000, 1 },

    // Larger cases, bigger strides.
    { 1U << 18, 3701 },
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

INSTANTIATE_TEST_CASE_P(MultiBit, MultiBitTest, ValuesIn(multibitTests));

TEST(MultiBit, SizeTooBig) {
    ASSERT_NO_THROW(mmbit_size(MMB_MAX_BITS));
    ASSERT_THROW(mmbit_size(MMB_MAX_BITS + 1), ResourceLimitError);
}
