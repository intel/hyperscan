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

#include "nfa/limex_ring.h"
#include "nfa/repeat.h"
#include "nfa/repeatcompile.h"
#include "util/depth.h"
#include "util/make_unique.h"

#include <algorithm>
#include <memory>
#include <vector>

using namespace std;
using namespace testing;
using namespace ue2;

struct RepeatTestInfo {
    enum RepeatType type;
    depth repeatMin;
    depth repeatMax;
};

static
ostream& operator<<(ostream &os, const RepeatInfo &info) {
    os << "{" << info.repeatMin;
    if (info.repeatMin == info.repeatMax) {
        os << "}";
    } else if (info.repeatMax == REPEAT_INF) {
        os << ",}";
    } else {
        os << "," << info.repeatMax << "}";
    }
    os << " " << repeatTypeName(info.type);
    os << " (period " << info.minPeriod << ")";
    return os;
}

class RepeatTest : public TestWithParam<RepeatTestInfo> {
protected:
    virtual void SetUp() {
        test_info = GetParam();

        info.type = test_info.type;
        info.repeatMin = test_info.repeatMin;
        info.repeatMax = test_info.repeatMax.is_finite()
                             ? (u32)test_info.repeatMax
                             : REPEAT_INF;
        info.minPeriod = 0;

        RepeatStateInfo rsi(test_info.type, test_info.repeatMin,
                            test_info.repeatMax, 0);
        info.packedCtrlSize = rsi.packedCtrlSize;
        info.stateSize = rsi.stateSize;
        info.horizon = rsi.horizon;
        std::copy(rsi.packedFieldSizes.begin(), rsi.packedFieldSizes.end(),
                  info.packedFieldSizes);

        ctrl = new RepeatControl();
        state_int = new char[info.stateSize + 7]; /* state may have mmbits */
        state = state_int + 7;
    }

    virtual void TearDown() {
        delete ctrl;
        delete [] state_int;
    }

    RepeatTestInfo test_info; // Test params
    RepeatInfo info; // Repeat info structure
    RepeatControl *ctrl;
    char *state;
private:
    char *state_int;

};

static const RepeatTestInfo repeatTests[] = {
    // Fixed repeats -- ring model
    { REPEAT_RING, depth(2), depth(2) },
    { REPEAT_RING, depth(4), depth(4) },
    { REPEAT_RING, depth(10), depth(10) },
    { REPEAT_RING, depth(16), depth(16) },
    { REPEAT_RING, depth(20), depth(20) },
    { REPEAT_RING, depth(30), depth(30) },
    { REPEAT_RING, depth(50), depth(50) },
    { REPEAT_RING, depth(64), depth(64) },
    { REPEAT_RING, depth(65), depth(65) },
    { REPEAT_RING, depth(100), depth(100) },
    { REPEAT_RING, depth(200), depth(200) },
    { REPEAT_RING, depth(1000), depth(1000) },
    { REPEAT_RING, depth(4100), depth(4100) },
    { REPEAT_RING, depth(16000), depth(16000) },
    // {0, N} repeats -- last model
    { REPEAT_LAST, depth(0), depth(4) },
    { REPEAT_LAST, depth(0), depth(10) },
    { REPEAT_LAST, depth(0), depth(20) },
    { REPEAT_LAST, depth(0), depth(30) },
    { REPEAT_LAST, depth(0), depth(50) },
    { REPEAT_LAST, depth(0), depth(100) },
    { REPEAT_LAST, depth(0), depth(200) },
    { REPEAT_LAST, depth(0), depth(1000) },
    { REPEAT_LAST, depth(0), depth(16000) },
    // {0, N} repeats -- ring model (though we use 'last' model in practice)
    { REPEAT_RING, depth(0), depth(2) },
    { REPEAT_RING, depth(0), depth(4) },
    { REPEAT_RING, depth(0), depth(10) },
    { REPEAT_RING, depth(0), depth(20) },
    { REPEAT_RING, depth(0), depth(30) },
    { REPEAT_RING, depth(0), depth(50) },
    { REPEAT_RING, depth(0), depth(64) },
    { REPEAT_RING, depth(0), depth(65) },
    { REPEAT_RING, depth(0), depth(100) },
    { REPEAT_RING, depth(0), depth(200) },
    { REPEAT_RING, depth(0), depth(1000) },
    { REPEAT_RING, depth(0), depth(16000) },
    // {N, M} repeats -- ring model
    { REPEAT_RING, depth(2), depth(3) },
    { REPEAT_RING, depth(1), depth(4) },
    { REPEAT_RING, depth(5), depth(10) },
    { REPEAT_RING, depth(10), depth(20) },
    { REPEAT_RING, depth(10), depth(50) },
    { REPEAT_RING, depth(50), depth(60) },
    { REPEAT_RING, depth(100), depth(200) },
    { REPEAT_RING, depth(1), depth(200) },
    { REPEAT_RING, depth(10), depth(16000) },
    { REPEAT_RING, depth(10000), depth(16000) },
    // {N, M} repeats -- range model
    { REPEAT_RANGE, depth(1), depth(4) },
    { REPEAT_RANGE, depth(5), depth(10) },
    { REPEAT_RANGE, depth(10), depth(20) },
    { REPEAT_RANGE, depth(10), depth(50) },
    { REPEAT_RANGE, depth(50), depth(60) },
    { REPEAT_RANGE, depth(100), depth(200) },
    { REPEAT_RANGE, depth(1), depth(200) },
    { REPEAT_RANGE, depth(10), depth(16000) },
    { REPEAT_RANGE, depth(10000), depth(16000) },
    // {N,M} repeats -- small bitmap model
    { REPEAT_BITMAP, depth(1), depth(2) },
    { REPEAT_BITMAP, depth(5), depth(10) },
    { REPEAT_BITMAP, depth(10), depth(20) },
    { REPEAT_BITMAP, depth(20), depth(40) },
    { REPEAT_BITMAP, depth(1), depth(63) },
    { REPEAT_BITMAP, depth(50), depth(63) },
    // {N,M} repeats -- trailer model
    { REPEAT_TRAILER, depth(1), depth(2) },
    { REPEAT_TRAILER, depth(8), depth(8) },
    { REPEAT_TRAILER, depth(0), depth(8) },
    { REPEAT_TRAILER, depth(10), depth(20) },
    { REPEAT_TRAILER, depth(1), depth(32) },
    { REPEAT_TRAILER, depth(64), depth(64) },
    { REPEAT_TRAILER, depth(1), depth(64) },
    { REPEAT_TRAILER, depth(1), depth(100) },
    { REPEAT_TRAILER, depth(1), depth(2000) },
    { REPEAT_TRAILER, depth(50), depth(200) },
    { REPEAT_TRAILER, depth(50), depth(1000) },
    { REPEAT_TRAILER, depth(64), depth(1024) },
    // {N,} repeats -- first model
    { REPEAT_FIRST, depth(0), depth::infinity() },
    { REPEAT_FIRST, depth(1), depth::infinity() },
    { REPEAT_FIRST, depth(4), depth::infinity() },
    { REPEAT_FIRST, depth(10), depth::infinity() },
    { REPEAT_FIRST, depth(50), depth::infinity() },
    { REPEAT_FIRST, depth(100), depth::infinity() },
    { REPEAT_FIRST, depth(1000), depth::infinity() },
    { REPEAT_FIRST, depth(3000), depth::infinity() },
    { REPEAT_FIRST, depth(10000), depth::infinity() },
    // {,} repeats -- always
    { REPEAT_ALWAYS, depth(0), depth::infinity() },
};

INSTANTIATE_TEST_CASE_P(Repeat, RepeatTest, ValuesIn(repeatTests));

TEST_P(RepeatTest, MatchSuccess) {
    SCOPED_TRACE(testing::Message() << "Repeat: " << info);

    u64a offset = 1000;
    repeatStore(&info, ctrl, state, offset, 0);

    for (u32 i = info.repeatMin; i <= info.repeatMax; i++) {
        enum TriggerResult rv = processTugTrigger(&info, ctrl, state, offset + i);
        if (rv == TRIGGER_SUCCESS_CACHE) {
            rv = TRIGGER_SUCCESS;
        }
        ASSERT_EQ(TRIGGER_SUCCESS, rv);
    }
}

TEST_P(RepeatTest, MatchNegSuccess) {
    SCOPED_TRACE(testing::Message() << "Repeat: " << info);

    // Write a top at offset 1000.
    const u64a offset = 1000;
    repeatStore(&info, ctrl, state, offset, 0);

    // Write another match at offset 1002, using is_alive=0 (i.e. the repeat
    // was killed between these two tops).
    repeatStore(&info, ctrl, state, offset + 2, 0);

    enum TriggerResult rv;

    // Match at offset + repeatMin should fail, while offset + repeatMin + 2
    // should succeed.
    if (info.repeatMin > 2) {
        rv = processTugTrigger(&info, ctrl, state, offset + info.repeatMin);
        ASSERT_EQ(TRIGGER_FAIL, rv);
    }
    rv = processTugTrigger(&info, ctrl, state, offset + info.repeatMin + 2);
    if (rv == TRIGGER_SUCCESS_CACHE) {
        rv = TRIGGER_SUCCESS;
    }
    ASSERT_EQ(TRIGGER_SUCCESS, rv);
}

TEST_P(RepeatTest, MatchFail) {
    SCOPED_TRACE(testing::Message() << "Repeat: " << info);

    // Write a match at offset 1000.
    const u64a offset = 1000;
    repeatStore(&info, ctrl, state, offset, 0);

    // Test for a match, if possible, at offset + repeatMin - 1.
    enum TriggerResult rv;

    if (info.repeatMin > 0) {
        u64a testOffset = offset + info.repeatMin - 1;
        rv = processTugTrigger(&info, ctrl, state, testOffset);
        ASSERT_EQ(TRIGGER_FAIL, rv);
    }

    // Match after repeatMax should fail as well.
    if (info.repeatMax != REPEAT_INF) {
        u64a testOffset = offset + info.repeatMax + 1;
        rv = processTugTrigger(&info, ctrl, state, testOffset);
        ASSERT_EQ(TRIGGER_STALE, rv);
    }
}

// Fill the ring with matches.
TEST_P(RepeatTest, FillRing) {
    if (info.repeatMax == REPEAT_INF) {
        return;
    }

    const u64a offset = 1000;
    repeatStore(&info, ctrl, state, offset, 0);
    for (u64a i = 1; i <= info.repeatMax; i++) {
        repeatStore(&info, ctrl, state, offset + i, 1);
    }

    // We should be able to see matches for all of these (beyond the last top offset).
    enum TriggerResult rv;
    for (u64a i = offset + info.repeatMax;
            i <= offset + info.repeatMax + info.repeatMin; i++) {
        rv = processTugTrigger(&info, ctrl, state, i);
        if (rv == TRIGGER_SUCCESS_CACHE) {
            rv = TRIGGER_SUCCESS;
        }
        ASSERT_EQ(TRIGGER_SUCCESS, rv);
    }
}

TEST_P(RepeatTest, FindTops) {
    SCOPED_TRACE(testing::Message() << "Repeat: " << info);
    /* REPEAT_ALWAYS has no state and so does not track top locations */
    if (info.type == REPEAT_ALWAYS) {
        return;
    }

    repeatStore(&info, ctrl, state, 1000, 0);
    ASSERT_EQ(1000, repeatLastTop(&info, ctrl, state));

    repeatStore(&info, ctrl, state, 2000, 1);
    if (info.type == REPEAT_FIRST) {
        ASSERT_EQ(1000, repeatLastTop(&info, ctrl, state));
    } else {
        ASSERT_EQ(2000, repeatLastTop(&info, ctrl, state));
    }

    repeatStore(&info, ctrl, state, 3000, 0);
    ASSERT_EQ(3000, repeatLastTop(&info, ctrl, state));
}

TEST_P(RepeatTest, NextMatch) {
    SCOPED_TRACE(testing::Message() << "Repeat: " << info);

    u64a top = 10000000ULL;
    repeatStore(&info, ctrl, state, top, 0);

    u64a i = top + info.repeatMin;

    if (info.repeatMin != 0) {
        // First match after the top should be at top+min.
        ASSERT_EQ(i, repeatNextMatch(&info, ctrl, state, top));
    }

    while (i < info.repeatMax) {
        ASSERT_EQ(i + 1, repeatNextMatch(&info, ctrl, state, i));
        i++;
    }

    if (info.repeatMax != REPEAT_INF) {
        ASSERT_EQ(0, repeatNextMatch(&info, ctrl, state, top + info.repeatMax));
    }
}

TEST_P(RepeatTest, NextMatchFilledRepeat) {
    // This test is only really appropriate for repeat models that store more
    // than one top.
    if (info.type != REPEAT_RING && info.type != REPEAT_RANGE) {
        return;
    }

    SCOPED_TRACE(testing::Message() << "Repeat: " << info);

    u64a top = 10000000ULL;
    repeatStore(&info, ctrl, state, top, 0);
    for (u64a i = 1; i <= info.repeatMax; i++) {
        repeatStore(&info, ctrl, state, top + i, 1);
    }

    u64a last_top = top + info.repeatMax;
    u64a i = top + info.repeatMin;

    if (info.repeatMin != 0) {
        // First match after the initial top should be at top+min.
        ASSERT_EQ(i, repeatNextMatch(&info, ctrl, state, top));
    }

    while (i < last_top + info.repeatMax) {
        ASSERT_EQ(i + 1, repeatNextMatch(&info, ctrl, state, i));
        i++;
    }

    if (info.repeatMax != REPEAT_INF) {
        ASSERT_EQ(0, repeatNextMatch(&info, ctrl, state, last_top + info.repeatMax));
    }
}

TEST_P(RepeatTest, TwoTops) {
    SCOPED_TRACE(testing::Message() << "Repeat: " << info);

    // Only appropriate for tests that store more than one top.
    if (info.type == REPEAT_FIRST || info.type == REPEAT_LAST
        || info.type == REPEAT_ALWAYS) {
        return;
    }

    const u32 top_range = info.repeatMax;

    // Limit the scope of this test for runtime brevity. For small cases, we
    // check every offset, but for bigger ones we use a bigger step.
    const u32 iter_step = std::max(1u, top_range / 256u);

    const u64a top1 = 10000ull;

    for (u32 i = 1; i < top_range; i += iter_step) {
        const u64a top2 = top1 + i;
        SCOPED_TRACE(testing::Message() << "Tops at offsets " << top1 << " and "
                                        << top2);

        repeatStore(&info, ctrl, state, top1, 0);
        ASSERT_EQ(top1, repeatLastTop(&info, ctrl, state));
        repeatStore(&info, ctrl, state, top2, 1);
        ASSERT_EQ(top2, repeatLastTop(&info, ctrl, state));

        // We should have those matches from the top1 match window that are
        // greater than or equal to top2.
        for (u64a j = std::max(top1 + info.repeatMin, top2);
             j <= top1 + info.repeatMax; j++) {
            ASSERT_EQ(REPEAT_MATCH, repeatHasMatch(&info, ctrl, state, j))
                << j << " should be a match due to top 1";
        }

        // If the two match windows don't overlap, we should have some
        // non-matching positions between them.
        for (u64a j = top1 + info.repeatMax + 1; j < top2 + info.repeatMin;
             j++) {
            ASSERT_EQ(REPEAT_NOMATCH, repeatHasMatch(&info, ctrl, state, j))
                << j << " should not be a match";
        }

        // We should have all the matches in the match window from top 2.
        for (u64a j = top2 + info.repeatMin; j <= top2 + info.repeatMax; j++) {
            ASSERT_EQ(REPEAT_MATCH, repeatHasMatch(&info, ctrl, state, j))
                << j << " should be a match due to top 2";
        }

        // One past the end should be stale.
        u64a past_end = top2 + info.repeatMax + 1;
        ASSERT_EQ(REPEAT_STALE, repeatHasMatch(&info, ctrl, state, past_end))
            << "repeat should be stale at " << past_end;
    }
}

TEST_P(RepeatTest, Pack) {
    SCOPED_TRACE(testing::Message() << "Repeat: " << info);
    u64a offset = 1000;

    repeatStore(&info, ctrl, state, offset, 0);

    // We should be able to pack and then unpack the control block at any
    // offset up to repeatMin and get a match at both the min and max repeats.

    unique_ptr<char[]> packed = ue2::make_unique<char[]>(info.packedCtrlSize);

    for (u32 i = 0; i < info.repeatMax; i++) {
        SCOPED_TRACE(testing::Message() << "i=" << i);
        const u64a pack_offset = offset + i;

        memset(packed.get(), 0xff, info.packedCtrlSize);
        repeatPack(packed.get(), &info, ctrl, pack_offset);
        memset(ctrl, 0xff, sizeof(*ctrl));
        repeatUnpack(packed.get(), &info, pack_offset, ctrl);

        // We should have a match at every offset in [offset + repeatMin,
        // offset + repeatMax]. For brevity, we just check the first one and
        // the last one.

        u64a first = offset + std::max(i, info.repeatMin);
        u64a last = offset + info.repeatMax;
        ASSERT_EQ(REPEAT_MATCH, repeatHasMatch(&info, ctrl, state, first))
            << "repeat should have match at " << first;
        ASSERT_EQ(REPEAT_MATCH, repeatHasMatch(&info, ctrl, state, last))
            << "repeat should have match at " << last;
    }
}

TEST_P(RepeatTest, LargeGap) {
    SCOPED_TRACE(testing::Message() << "Repeat: " << info);

    if (info.repeatMax == REPEAT_INF) {
        return; // Test not valid for FIRST-type repeats.
    }

    for (int i = 0; i < 64; i++) {
        u64a top1 = 1000;
        repeatStore(&info, ctrl, state, top1, 0); // first top
        ASSERT_EQ(top1, repeatLastTop(&info, ctrl, state));

        // Add a second top after a gap of 2^i bytes.
        u64a top2 = top1 + (1ULL << i);
        repeatStore(&info, ctrl, state, top2, 1); // second top
        ASSERT_EQ(top2, repeatLastTop(&info, ctrl, state));
    }
}

static
const u32 sparsePeriods[] = {
    2,
    4,
    6,
    8,
    10,
    12,
    15,
    18,
    20,
    22,
    24,
    26,
    28,
    30,
/*    40,
    50,
    60,
    80,
    100,
    120,
    150,
    180,
    200,
    250,
    300,
    350,
    400,*/
};

static
const RepeatTestInfo sparseRepeats[] = {
    // Fixed repeats
    { REPEAT_SPARSE_OPTIMAL_P, depth(10), depth(10) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(20), depth(20) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(40), depth(40) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(80), depth(80) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(100), depth(100) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(150), depth(150) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(200), depth(200) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(250), depth(250) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(300), depth(300) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(350), depth(350) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(400), depth(400) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(500), depth(500) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(600), depth(600) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(800), depth(800) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(1000), depth(1000) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(1500), depth(1500) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(2000), depth(2000) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(2500), depth(2500) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(3000), depth(3000) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(3500), depth(3500) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(4000), depth(4000) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(4500), depth(4500) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(5000), depth(5000) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(65534), depth(65534) },
    // {N, M} repeats
    { REPEAT_SPARSE_OPTIMAL_P, depth(10), depth(20) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(20), depth(40) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(40), depth(80) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(80), depth(100) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(100), depth(120) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(150), depth(180) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(200), depth(400) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(250), depth(500) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(300), depth(400) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(350), depth(500) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(400), depth(500) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(500), depth(600) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(600), depth(700) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(800), depth(1000) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(1000), depth(1200) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(1500), depth(1800) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(2000), depth(4000) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(2500), depth(3000) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(3000), depth(3500) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(3500), depth(4000) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(4000), depth(8000) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(4500), depth(8000) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(5000), depth(5001) },
    { REPEAT_SPARSE_OPTIMAL_P, depth(60000), depth(65534) }
};

static
void test_sparse2entry(const RepeatInfo *info, RepeatControl *ctrl,
                       char *state, u64a second) {
    SCOPED_TRACE(testing::Message() << "Repeat: " << *info);
    SCOPED_TRACE(second);

    if (second > info->repeatMax || second < info->minPeriod) {
        return;
    }

    u64a offset = 1000;
    u64a exit = offset + info->repeatMin;
    repeatStore(info, ctrl, state, offset, 0);
    ASSERT_EQ(offset, repeatLastTop(info, ctrl, state));

    u64a offset2 = 1000 + second;
    u64a exit2 = offset2 + info->repeatMin;
    repeatStore(info, ctrl, state, offset2, 1);
    ASSERT_EQ(offset2, repeatLastTop(info, ctrl, state));

    u32 range = info->repeatMax - info->repeatMin;
    for (u32 i = offset2; i < offset + info->repeatMax * 2 + 100; i++) {
        SCOPED_TRACE(i);
        RepeatMatch r = repeatHasMatch(info, ctrl, state, i);
        if ((i >= exit && i <= exit + range) ||
            (i >= exit2 && i <= exit2 + range)) {
            ASSERT_EQ(REPEAT_MATCH, r);
        } else if (i > exit2 + range) {
            ASSERT_EQ(REPEAT_STALE, r);
        } else {
            ASSERT_EQ(REPEAT_NOMATCH, r);
        }
    }

    ASSERT_EQ(MAX(exit + range + 1, exit2),
              repeatNextMatch(info, ctrl, state, exit + range));
    ASSERT_EQ(MAX(exit + range + 2, exit2),
              repeatNextMatch(info, ctrl, state, exit + range + 1));
    ASSERT_EQ(exit2, repeatNextMatch(info, ctrl, state, exit2 - 1));
    ASSERT_EQ(0, repeatNextMatch(info, ctrl, state, exit2 + range));
}

static
void test_sparse3entry(const RepeatInfo *info, RepeatControl *ctrl,
                       char *state, u64a diff) {
    SCOPED_TRACE(testing::Message() << "Repeat:" << *info);
    SCOPED_TRACE(diff);

    if (diff * 2 > info->repeatMax || diff < info->minPeriod) {
        return;
    }

    u64a offset = 1000;
    u64a exit = offset + info->repeatMin;
    repeatStore(info, ctrl, state, offset, 0);
    ASSERT_EQ(offset, repeatLastTop(info, ctrl, state));

    u64a offset2 = 1000 + diff;
    u64a exit2 = offset2 + info->repeatMin;
    repeatStore(info, ctrl, state, offset2, 1);
    ASSERT_EQ(offset2, repeatLastTop(info, ctrl, state));

    u64a offset3 = 1000 + 2 * diff;
    u64a exit3 = offset3 + info->repeatMin;
    repeatStore(info, ctrl, state, offset3, 1);
    ASSERT_EQ(offset3, repeatLastTop(info, ctrl, state));

    u32 range = info->repeatMax - info->repeatMin;
    for (u32 i = offset2; i < offset + info->repeatMax * 2 + 100; i++) {
        SCOPED_TRACE(i);
        RepeatMatch r = repeatHasMatch(info, ctrl, state, i);
        if((i >= exit && i <= exit + range)||
           (i >= exit2 && i <= exit2 + range) ||
           (i >= exit3 && i <= exit3 + range)) {
            ASSERT_EQ(REPEAT_MATCH, r);
        } else if (i > exit3 + range) {
            ASSERT_EQ(REPEAT_STALE, r);
        } else {
            ASSERT_EQ(REPEAT_NOMATCH, r);
        }
    }

    ASSERT_EQ(MAX(exit + range + 1, exit2),
              repeatNextMatch(info, ctrl, state, exit + range));
    ASSERT_EQ(MAX(exit + range + 2, exit2),
              repeatNextMatch(info, ctrl, state, exit + range + 1));
    ASSERT_EQ(exit2, repeatNextMatch(info, ctrl, state, exit2 - 1));
    ASSERT_EQ(MAX(exit2 + range + 1, exit3),
              repeatNextMatch(info, ctrl, state, exit2 + range));
    ASSERT_EQ(MAX(exit2 + range + 2, exit3),
              repeatNextMatch(info, ctrl, state, exit2 + range + 1));
    ASSERT_EQ(exit3, repeatNextMatch(info, ctrl, state, exit3 - 1));
    ASSERT_EQ(0, repeatNextMatch(info, ctrl, state, exit3 + range));
}

static
void test_sparse3entryNeg(const RepeatInfo *info, RepeatControl *ctrl,
                          char *state, u64a diff) {
    SCOPED_TRACE(testing::Message() << "Repeat:" << *info);
    SCOPED_TRACE(diff);

    if (diff * 2 > info->repeatMax || diff < info->minPeriod) {
        return;
    }

    u64a offset = 1000;
    repeatStore(info, ctrl, state, offset, 0);
    ASSERT_EQ(offset, repeatLastTop(info, ctrl, state));

    u64a offset2 = 1000 + diff;
    repeatStore(info, ctrl, state, offset2, 0);
    ASSERT_EQ(offset2, repeatLastTop(info, ctrl, state));

    u64a offset3 = 1000 + 2 * diff;
    u64a exit3 = offset3 + info->repeatMin;
    repeatStore(info, ctrl, state, offset3, 0);
    ASSERT_EQ(offset3, repeatLastTop(info, ctrl, state));

    u32 range = info->repeatMax - info->repeatMin;
    for (u32 i = offset3; i < offset + info->repeatMax * 2 + 100; i++) {
        SCOPED_TRACE(i);
        RepeatMatch r = repeatHasMatch(info, ctrl, state, i);
        if(i >= exit3 && i <= exit3 + range) {
            ASSERT_EQ(REPEAT_MATCH, r);
        } else if (i > exit3 + range) {
            ASSERT_EQ(REPEAT_STALE, r);
        } else {
            ASSERT_EQ(REPEAT_NOMATCH, r);
        }
    }
}

static
void test_sparse3entryExpire(const RepeatInfo *info, RepeatControl *ctrl,
                             char *state, u64a diff) {
    SCOPED_TRACE(testing::Message() << "Repeat:" << *info);
    SCOPED_TRACE(diff);

    if (diff * 2 > info->repeatMax || diff < info->minPeriod) {
        return;
    }

    u64a offset = 1000;
    repeatStore(info, ctrl, state, offset, 0);
    ASSERT_EQ(offset, repeatLastTop(info, ctrl, state));

    u64a offset2 = 1000 + diff;
    repeatStore(info, ctrl, state, offset2, 1);
    ASSERT_EQ(offset2, repeatLastTop(info, ctrl, state));

    u64a offset3 = 1000 + 2 * info->repeatMax;
    u64a exit3 = offset3 + info->repeatMin;
    repeatStore(info, ctrl, state, offset3, 1);
    ASSERT_EQ(offset3, repeatLastTop(info, ctrl, state));

    u32 range = info->repeatMax - info->repeatMin;
    for (u32 i = offset3; i < offset3 + info->repeatMax + 100; i++) {
        SCOPED_TRACE(i);
        RepeatMatch r = repeatHasMatch(info, ctrl, state, i);
        if(i >= exit3 && i <= exit3 + range) {
            ASSERT_EQ(REPEAT_MATCH, r);
        } else if (i > exit3 + range) {
            ASSERT_EQ(REPEAT_STALE, r);
        } else {
            ASSERT_EQ(REPEAT_NOMATCH, r);
        }
    }
}

class SparseOptimalTest : public TestWithParam<tuple<u32, RepeatTestInfo> > {
protected:
    virtual void SetUp() {
        u32 period;
        tie(period, test_info) = GetParam();

        RepeatStateInfo rsi(REPEAT_SPARSE_OPTIMAL_P, test_info.repeatMin,
                            test_info.repeatMax, period);

        ptr = new char[sizeof(RepeatInfo) +
                       sizeof(u64a) * (rsi.patchSize + 2)];

        info = (struct RepeatInfo *)ptr;

        info->type = REPEAT_SPARSE_OPTIMAL_P;
        info->repeatMin = test_info.repeatMin;
        info->repeatMax = test_info.repeatMax;
        info->minPeriod = period;

        info->packedCtrlSize = rsi.packedCtrlSize;
        info->stateSize = rsi.stateSize;
        info->horizon = rsi.horizon;
        std::copy(rsi.packedFieldSizes.begin(), rsi.packedFieldSizes.end(),
                  info->packedFieldSizes);
        info->patchCount = rsi.patchCount;
        info->patchSize = rsi.patchSize;
        info->encodingSize = rsi.encodingSize;
        info->patchesOffset = rsi.patchesOffset;

        u32 repeatMax = info->patchSize;
        u64a *table = (u64a *)(ROUNDUP_PTR((ptr + sizeof(RepeatInfo)),
                                           alignof(u64a)));
        for (u32 i = 0; i < repeatMax + 1; i++) {
            table[i] = rsi.table[i];
        }

        ctrl = new RepeatControl();
        state_int = new char[info->stateSize + 7]; /* state may have mmbits */
        state = state_int + 7;
    }

    virtual void TearDown() {
        delete ctrl;
        delete[] state_int;
        delete[] ptr;
    }

    RepeatTestInfo test_info; // Test params
    RepeatInfo *info; // Repeat info structure
    RepeatControl *ctrl;
    char *state;
private:
    char *ptr;
    char *state_int;

};

TEST_P(SparseOptimalTest, Simple1) {
    SCOPED_TRACE(testing::Message() << "Repeat: " << *info);

    u64a offset = 1000;
    repeatStore(info, ctrl, state, offset, 0);

    ASSERT_EQ(1000U, repeatLastTop(info, ctrl, state));

    for (u32 i = 0; i < info->repeatMax * 2 + 100; i++) {
        SCOPED_TRACE(i);
        RepeatMatch r = repeatHasMatch(info, ctrl, state, offset + i);
        if (i >= info->repeatMin && i <= info->repeatMax) {
            ASSERT_EQ(REPEAT_MATCH, r);
        } else if (i > info->repeatMax) {
            ASSERT_EQ(REPEAT_STALE, r);
        } else {
            ASSERT_EQ(REPEAT_NOMATCH, r);
        }
    }

    u64a exp = 1000 + info->repeatMin;
    ASSERT_EQ(exp, repeatNextMatch(info, ctrl, state, 1000));
    ASSERT_EQ(exp, repeatNextMatch(info, ctrl, state, 1001));
    ASSERT_EQ(exp, repeatNextMatch(info, ctrl, state, 1002));
    ASSERT_EQ(exp, repeatNextMatch(info, ctrl, state, 1003));
    ASSERT_EQ(exp, repeatNextMatch(info, ctrl, state,
                                   1000 + info->repeatMin - 5));
    ASSERT_EQ(exp, repeatNextMatch(info, ctrl, state,
                                   1000 + info->repeatMin - 4));
    ASSERT_EQ(exp, repeatNextMatch(info, ctrl, state,
                                   1000 + info->repeatMin - 3));
    ASSERT_EQ(exp, repeatNextMatch(info, ctrl, state,
                                   1000 + info->repeatMin - 2));
    ASSERT_EQ(exp, repeatNextMatch(info, ctrl, state,
                                   1000 + info->repeatMin - 1));
    ASSERT_EQ(0U, repeatNextMatch(info, ctrl, state,
                                  1000 + info->repeatMax));
    ASSERT_EQ(0U, repeatNextMatch(info, ctrl, state,
                                  1000 + info->repeatMax + 1));
    ASSERT_EQ(0U, repeatNextMatch(info, ctrl, state,
                                  1000 + info->repeatMax * 2 - 1));
    ASSERT_EQ(0U, repeatNextMatch(info, ctrl, state,
                                  1000 + info->repeatMax * 2));
    ASSERT_EQ(0U, repeatNextMatch(info, ctrl, state,
                                  1000 + info->repeatMax * 2 + 1));
    ASSERT_EQ(0U, repeatNextMatch(info, ctrl, state, 100000));
}

TEST_P(SparseOptimalTest, TwoTopsNeg) {
    SCOPED_TRACE(testing::Message() << "Repeat: " << *info);

    u32 patch_count = info->patchCount;
    u32 patch_size = info->patchSize;
    u64a offset = 1000;
    u64a exit = offset + info->repeatMin;
    repeatStore(info, ctrl, state, offset, 0);
    ASSERT_EQ(offset, repeatLastTop(info, ctrl, state));

    u64a offset2 = 1000 + patch_count * patch_size;
    u64a exit2 = offset2 + info->repeatMin;
    repeatStore(info, ctrl, state, offset2, 1);
    ASSERT_EQ(offset2, repeatLastTop(info, ctrl, state));

    u32 range = info->repeatMax - info->repeatMin;
    for (u32 i = offset2; i < offset + info->repeatMax * 2 + 100; i++) {
        SCOPED_TRACE(i);
        RepeatMatch r = repeatHasMatch(info, ctrl, state, i);
        if (i >= exit2 && i <= exit2 + range) {
            ASSERT_EQ(REPEAT_MATCH, r);
        } else if (i > exit2 + range) {
            ASSERT_EQ(REPEAT_STALE, r);
        } else {
            ASSERT_EQ(REPEAT_NOMATCH, r);
        }
    }

    const struct RepeatRingControl *xs = (const struct RepeatRingControl *)
                                         ctrl;
    ASSERT_EQ(exit2, repeatNextMatch(info, ctrl, state,
                                     MAX(xs->offset, exit)));
    ASSERT_EQ(exit2, repeatNextMatch(info, ctrl, state,
                                     MAX(xs->offset, exit + 1)));
    ASSERT_EQ(exit2, repeatNextMatch(info, ctrl, state, exit2 - 1));
    ASSERT_EQ(0, repeatNextMatch(info, ctrl, state, exit2 + range));
}

TEST_P(SparseOptimalTest, Simple2a) {
    test_sparse2entry(info, ctrl, state, info->minPeriod);
}

TEST_P(SparseOptimalTest, Simple2b) {
    test_sparse2entry(info, ctrl, state, info->repeatMax - 1);
}

TEST_P(SparseOptimalTest, Simple2c) {
    test_sparse2entry(info, ctrl, state, info->repeatMax);
}

TEST_P(SparseOptimalTest, Simple2d) {
    test_sparse2entry(info, ctrl, state, info->minPeriod + 1);
}

TEST_P(SparseOptimalTest, Simple2e) {
    test_sparse2entry(info, ctrl, state, 2 * info->minPeriod - 1);
}

TEST_P(SparseOptimalTest, Simple3a) {
    test_sparse3entry(info, ctrl, state, info->minPeriod);
    test_sparse3entryNeg(info, ctrl, state, info->minPeriod);
    test_sparse3entryExpire(info, ctrl, state, info->minPeriod);
}

TEST_P(SparseOptimalTest, Simple3b) {
    test_sparse3entry(info, ctrl, state, info->repeatMax / 2 - 1);
    test_sparse3entryNeg(info, ctrl, state, info->repeatMax / 2 - 1);
    test_sparse3entryExpire(info, ctrl, state, info->repeatMax / 2 - 1);
}

TEST_P(SparseOptimalTest, Simple3c) {
    test_sparse3entry(info, ctrl, state, info->repeatMax / 2);
    test_sparse3entryNeg(info, ctrl, state, info->repeatMax / 2);
    test_sparse3entryExpire(info, ctrl, state, info->repeatMax / 2);
}

TEST_P(SparseOptimalTest, Simple3d) {
    test_sparse3entry(info, ctrl, state, info->minPeriod + 1);
    test_sparse3entryNeg(info, ctrl, state, info->minPeriod + 1);
    test_sparse3entryExpire(info, ctrl, state, info->minPeriod + 1);
}

TEST_P(SparseOptimalTest, Simple3e) {
    test_sparse3entry(info, ctrl, state, 2 * info->minPeriod - 1);
    test_sparse3entryNeg(info, ctrl, state, 2 * info->minPeriod - 1);
    test_sparse3entryExpire(info, ctrl, state, 2 * info->minPeriod - 1);
}

TEST_P(SparseOptimalTest, LargeGap) {
    SCOPED_TRACE(testing::Message() << "Repeat: " << *info);

    for (int i = 0; i < 64; i++) {
        u64a top1 = 1000;
        repeatStore(info, ctrl, state, top1, 0); // first top
        ASSERT_EQ(top1, repeatLastTop(info, ctrl, state));

        // Add a second top after a gap of 2^i bytes.
        u64a top2 = top1 + (1ULL << i);
        if (top2 - top1 < info->minPeriod) {
            continue; // not a valid top
        }
        repeatStore(info, ctrl, state, top2, 1); // second top
        ASSERT_EQ(top2, repeatLastTop(info, ctrl, state));
    }
}

TEST_P(SparseOptimalTest, ThreeTops) {
    SCOPED_TRACE(testing::Message() << "Repeat: " << *info);

    u32 patch_count = info->patchCount;
    u32 patch_size = info->patchSize;
    if (patch_count < 3) {
        return;
    }

    u64a offset = 1000;
    repeatStore(info, ctrl, state, offset, 0);
    ASSERT_EQ(offset, repeatLastTop(info, ctrl, state));

    u64a offset2 = 1000 + 2 * patch_size - 1;
    u64a exit2 = offset2 + info->repeatMin;
    repeatStore(info, ctrl, state, offset2, 1);
    ASSERT_EQ(offset2, repeatLastTop(info, ctrl, state));

    u64a offset3 = 1000 + patch_count * patch_size;
    u64a exit3 = offset3 + info->repeatMin;
    repeatStore(info, ctrl, state, offset3, 1);
    ASSERT_EQ(offset3, repeatLastTop(info, ctrl, state));


    for (u32 i = offset2 + info->repeatMin;
         i <= offset2 + info->repeatMax; i++) {
        ASSERT_EQ(REPEAT_MATCH, repeatHasMatch(info, ctrl, state, i));
    }

    for (u32 i = offset2 + info->repeatMax + 1;
         i < offset3 + info->repeatMin; i++) {
        ASSERT_EQ(REPEAT_NOMATCH, repeatHasMatch(info, ctrl, state, i));
    }

    for (u32 i = offset3 + info->repeatMin;
         i <= offset3 + info->repeatMax; i++) {
        ASSERT_EQ(REPEAT_MATCH, repeatHasMatch(info, ctrl, state, i));
    }

    u32 range = info->repeatMax - info->repeatMin;
    ASSERT_EQ(MAX(exit2 + range + 1, exit3),
              repeatNextMatch(info, ctrl, state, exit2 + range));
    ASSERT_EQ(MAX(exit2 + range + 2, exit3),
              repeatNextMatch(info, ctrl, state, exit2 + range + 1));
    ASSERT_EQ(exit3, repeatNextMatch(info, ctrl, state, exit3 - 1));
    ASSERT_EQ(0, repeatNextMatch(info, ctrl, state, exit3 + range));
}

TEST_P(SparseOptimalTest, FillTops) {
    SCOPED_TRACE(testing::Message() << "Repeat: " << *info);

    u32 patch_count = info->patchCount;
    u32 patch_size = info->patchSize;
    u32 min_period = info->minPeriod;

    u64a offset = 1000;
    u64a exit = offset + info->repeatMin;
    repeatStore(info, ctrl, state, offset, 0);
    ASSERT_EQ(offset, repeatLastTop(info, ctrl, state));

    u64a offset2;
    for (u32 i = min_period; i < patch_count * patch_size; i += min_period) {
        offset2 = offset + i;
        repeatStore(info, ctrl, state, offset2, 1);
        ASSERT_EQ(offset2, repeatLastTop(info, ctrl, state));
    }

    u64a exit2;
    for (u32 i = 0; i < patch_count * patch_size; i += min_period) {
        exit2 = exit + i;
        for (u32 j = exit2 + info->repeatMin;
             j <= offset + info->repeatMax; j++) {
            ASSERT_EQ(REPEAT_MATCH, repeatHasMatch(info, ctrl, state, j));
        }
    }
}

INSTANTIATE_TEST_CASE_P(SparseOptimal, SparseOptimalTest,
                        Combine(ValuesIn(sparsePeriods),
                        ValuesIn(sparseRepeats)));
