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

#include "nfagraph_common.h"

#include "gtest/gtest.h"
#include "nfagraph/ng_repeat.h"
#include "nfagraph/ng_util.h"
#include "util/depth.h"
#include "hs_compile.h"

using namespace std;
using namespace testing;
using namespace ue2;

static UNUSED
std::ostream& operator<<(std::ostream &os, const depth &d) {
    // We reimplement depth::str, as it's only available if dump support is
    // compiled in.
    if (d.is_unreachable()) {
        os << "unr";
        return os;
    } else if (d.is_infinite()) {
        os << "inf";
        return os;
    }

    u32 val = d; // finite val
    os << val;
    return os;
}

struct PureRepeatTest {
    string pattern;
    depth minBound;
    depth maxBound;
};

class NFAPureRepeatTest : public TestWithParam<PureRepeatTest> { };

static const PureRepeatTest pureRepeatTests[] = {
    { "^.*", depth(0), depth::infinity() },
    { "^.+", depth(1), depth::infinity() },
    { "^.", depth(1), depth(1) },
    { "^..", depth(2), depth(2) },
    { "^.?.", depth(1), depth(2) },
    { "^.{1,2}", depth(1), depth(2) },
    { "^.{1,3}", depth(1), depth(3) },
    { "^.{1,10}", depth(1), depth(10) },
    { "^.{1,200}", depth(1), depth(200) },
    { "^.{200}", depth(200), depth(200) },
    { "^.{0,}", depth(0), depth::infinity() },
    { "^.{1,}", depth(1), depth::infinity() },
    { "^.{2,}", depth(2), depth::infinity() },
    { "^.{10,}", depth(10), depth::infinity() },
    { "^.{200,}", depth(200), depth::infinity() },
    { "^.{5000,}", depth(5000), depth::infinity() },
    { "^.{0,1}", depth(0), depth(1) },
    { "^.{0,2}", depth(0), depth(2) },
    { "^.{0,100}", depth(0), depth(100) },
    { "^.{0,5000}", depth(0), depth(5000) },
    { "^x{10}x{20,30}", depth(30), depth(40) },
    { "^..?..?..?..?..?", depth(5), depth(10) }
};

INSTANTIATE_TEST_CASE_P(PureRepeat, NFAPureRepeatTest,
                        ValuesIn(pureRepeatTests));

TEST_P(NFAPureRepeatTest, Check) {
    const PureRepeatTest &t = GetParam();
    SCOPED_TRACE(testing::Message() << "Pattern: " << t.pattern);
    auto w = constructGraph(t.pattern, HS_FLAG_ALLOWEMPTY);
    ASSERT_TRUE(w != nullptr);
    clearReports(*w);

    PureRepeat repeat;
    bool result = isPureRepeat(*w, repeat);

    ASSERT_EQ(true, result);
    ASSERT_EQ(t.minBound, repeat.bounds.min);
    ASSERT_EQ(t.maxBound, repeat.bounds.max);
}

// for google test
void PrintTo(const PureRepeatTest &p, ::std::ostream *os) {
    *os << "PureRepeatTest: " << p.pattern
        << "{" << p.minBound << ',' << p.maxBound << '}';
}
