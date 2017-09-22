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

#include "ue2common.h"
#include "grey.h"
#include "fdr/fdr.h"
#include "fdr/fdr_compile.h"
#include "fdr/fdr_compile_internal.h"
#include "fdr/fdr_engine_description.h"
#include "fdr/teddy_compile.h"
#include "fdr/teddy_engine_description.h"
#include "hwlm/hwlm_internal.h"
#include "util/alloc.h"

#include "database.h"
#include "scratch.h"
#include "gtest/gtest.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <boost/random.hpp>

using namespace std;
using namespace testing;
using namespace ue2;

#define NO_TEDDY_FAIL_ALLOWED 0

#if(NO_TEDDY_FAIL_ALLOWED)
#define CHECK_WITH_TEDDY_OK_TO_FAIL(fdr, hint) ASSERT_TRUE(fdr != nullptr)
#else
#define CHECK_WITH_TEDDY_OK_TO_FAIL(fdr, hint)                                 \
    {                                                                          \
        auto descr = getTeddyDescription(hint);                                \
        if (descr && fdr == nullptr) {                                         \
            return; /* cannot build Teddy for this set of literals */          \
        } else {                                                               \
            ASSERT_TRUE(fdr != nullptr);                                       \
        }                                                                      \
    }
#endif

namespace {

struct match {
    size_t end;
    u32 id;
    match(size_t end_in, u32 id_in)
        : end(end_in), id(id_in) {}
    bool operator==(const match &b) const {
        return end == b.end && id == b.id;
    }
    bool operator<(const match &b) const {
        return tie(id, end) < tie(b.id, b.end);
    }
    match operator+(size_t adj) {
        return match(end + adj, id);
    }
};

vector<match> matches;

extern "C" {

static
hwlmcb_rv_t decentCallback(size_t end, u32 id,
                           UNUSED struct hs_scratch *scratch) {
    DEBUG_PRINTF("match @%zu : %u\n", end, id);

    matches.push_back(match(end, id));
    return HWLM_CONTINUE_MATCHING;
}

static
hwlmcb_rv_t decentCallbackT(size_t end, u32 id,
                            UNUSED struct hs_scratch *scratch) {
    matches.push_back(match(end, id));
    return HWLM_TERMINATE_MATCHING;
}

} // extern "C"

} // namespace

static
vector<u32> getValidFdrEngines() {
    const auto target = get_current_target();

    vector<u32> ret;

    vector<FDREngineDescription> fdr_descriptions;
    getFdrDescriptions(&fdr_descriptions);
    for (const FDREngineDescription &d : fdr_descriptions) {
        if (d.isValidOnTarget(target)) {
            ret.push_back(d.getID());
        }
    }

    vector<TeddyEngineDescription> teddy_descriptions;
    getTeddyDescriptions(&teddy_descriptions);
    for (const TeddyEngineDescription &d : teddy_descriptions) {
        if (d.isValidOnTarget(target)) {
            ret.push_back(d.getID());
        }
    }

    return ret;
}


static
bytecode_ptr<FDR> buildFDREngineHinted(std::vector<hwlmLiteral> &lits,
                                       bool make_small, u32 hint,
                                       const target_t &target,
                                       const Grey &grey) {
    auto proto = fdrBuildProtoHinted(HWLM_ENGINE_FDR, lits, make_small, hint,
                                     target, grey);
    if (!proto) {
        return nullptr;
    }
    return fdrBuildTable(*proto, grey);
}

static
bytecode_ptr<FDR> buildFDREngine(std::vector<hwlmLiteral> &lits,
                                 bool make_small, const target_t &target,
                                 const Grey &grey) {
    auto proto = fdrBuildProto(HWLM_ENGINE_FDR, lits, make_small, target, grey);
    if (!proto) {
        return nullptr;
    }
    return fdrBuildTable(*proto, grey);
}

class FDRp : public TestWithParam<u32> {
};

TEST_P(FDRp, Simple) {
    const u32 hint = GetParam();
    SCOPED_TRACE(hint);

    const char data[] = "mnopqrabcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ12345678901234567890mnopqr";

    vector<hwlmLiteral> lits;
    lits.push_back(hwlmLiteral("mnopqr", 0, 0));

    auto fdr = buildFDREngineHinted(lits, false, hint, get_current_target(),
                                    Grey());
    CHECK_WITH_TEDDY_OK_TO_FAIL(fdr, hint);

    struct hs_scratch scratch;
    scratch.fdr_conf = NULL;
    fdrExec(fdr.get(), (const u8 *)data, sizeof(data), 0, decentCallback,
            &scratch, HWLM_ALL_GROUPS);

    ASSERT_EQ(3U, matches.size());
    EXPECT_EQ(match(5, 0), matches[0]);
    EXPECT_EQ(match(23, 0), matches[1]);
    EXPECT_EQ(match(83, 0), matches[2]);
    matches.clear();
}

TEST_P(FDRp, SimpleSingle) {
    const u32 hint = GetParam();
    SCOPED_TRACE(hint);

    const char data[] = "mnopqrabcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ12345678901234567890m0m";

    vector<hwlmLiteral> lits;
    lits.push_back(hwlmLiteral("m", 0, 0));

    auto fdr = buildFDREngineHinted(lits, false, hint, get_current_target(),
                                    Grey());
    CHECK_WITH_TEDDY_OK_TO_FAIL(fdr, hint);

    struct hs_scratch scratch;
    scratch.fdr_conf = NULL;
    fdrExec(fdr.get(), (const u8 *)data, sizeof(data) - 1 /* skip nul */, 0,
            decentCallback, &scratch, HWLM_ALL_GROUPS);

    ASSERT_EQ(4U, matches.size());
    EXPECT_EQ(match(0, 0), matches[0]);
    EXPECT_EQ(match(18, 0), matches[1]);
    EXPECT_EQ(match(78, 0), matches[2]);
    EXPECT_EQ(match(80, 0), matches[3]);
    matches.clear();
}

TEST_P(FDRp, MultiLocation) {
    const u32 hint = GetParam();
    SCOPED_TRACE(hint);

    vector<hwlmLiteral> lits;
    lits.push_back(hwlmLiteral("abc", 0, 1));

    auto fdr = buildFDREngineHinted(lits, false, hint, get_current_target(),
                                    Grey());
    CHECK_WITH_TEDDY_OK_TO_FAIL(fdr, hint);

    const u32 testSize = 128;

    vector<u8> data(testSize, 0);

    struct hs_scratch scratch;
    scratch.fdr_conf = NULL;
    for (u32 i = 0; i < testSize - 3; i++) {
        memcpy(data.data() + i, "abc", 3);
        fdrExec(fdr.get(), data.data(), testSize, 0, decentCallback, &scratch,
                HWLM_ALL_GROUPS);
        ASSERT_EQ(1U, matches.size());
        EXPECT_EQ(match(i + 2, 1), matches[0]);
        memset(data.data() + i, 0, 3);
        matches.clear();
    }
}

TEST_P(FDRp, NoRepeat1) {
    const u32 hint = GetParam();
    SCOPED_TRACE(hint);

    const char data[] = "mnopqrabcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ12345678901234567890m0m";

    vector<hwlmLiteral> lits
        = { hwlmLiteral("m", 0, 1, 0, HWLM_ALL_GROUPS, {}, {}) };

    auto fdr = buildFDREngineHinted(lits, false, hint, get_current_target(),
                                    Grey());
    CHECK_WITH_TEDDY_OK_TO_FAIL(fdr, hint);

    struct hs_scratch scratch;
    scratch.fdr_conf = NULL;
    fdrExec(fdr.get(), (const u8 *)data, sizeof(data) - 1 /* skip nul */, 0,
            decentCallback, &scratch, HWLM_ALL_GROUPS);

    ASSERT_EQ(1U, matches.size());
    EXPECT_EQ(match(0, 0), matches[0]);
    matches.clear();
}

TEST_P(FDRp, NoRepeat2) {
    const u32 hint = GetParam();
    SCOPED_TRACE(hint);

    const char data[] = "mnopqrabcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ12345678901234567890m0m";

    vector<hwlmLiteral> lits
        = { hwlmLiteral("m", 0, 1, 0, HWLM_ALL_GROUPS, {}, {}),
            hwlmLiteral("A", 0, 42) };

    auto fdr = buildFDREngineHinted(lits, false, hint, get_current_target(),
                                    Grey());
    CHECK_WITH_TEDDY_OK_TO_FAIL(fdr, hint);

    struct hs_scratch scratch;
    scratch.fdr_conf = NULL;
    fdrExec(fdr.get(), (const u8 *)data, sizeof(data) - 1 /* skip nul */, 0,
            decentCallback, &scratch, HWLM_ALL_GROUPS);

    ASSERT_EQ(3U, matches.size());
    EXPECT_EQ(match(0, 0), matches[0]);
    EXPECT_EQ(match(78, 0), matches[2]);
    matches.clear();
}

TEST_P(FDRp, NoRepeat3) {
    const u32 hint = GetParam();
    SCOPED_TRACE(hint);

    const char data[] = "mnopqrabcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ12345678901234567890m0m";

    vector<hwlmLiteral> lits
        = { hwlmLiteral("90m", 0, 1, 0, HWLM_ALL_GROUPS, {}, {}),
            hwlmLiteral("zA", 0, 1, 0, HWLM_ALL_GROUPS, {}, {}) };

    auto fdr = buildFDREngineHinted(lits, false, hint, get_current_target(),
                                    Grey());
    CHECK_WITH_TEDDY_OK_TO_FAIL(fdr, hint);

    struct hs_scratch scratch;
    scratch.fdr_conf = NULL;
    fdrExec(fdr.get(), (const u8 *)data, sizeof(data) - 1 /* skip nul */, 0,
            decentCallback, &scratch, HWLM_ALL_GROUPS);

    ASSERT_EQ(1U, matches.size());
    EXPECT_EQ(match(32, 0), matches[0]);
    matches.clear();
}

/**
 * \brief Helper function wrapping the FDR streaming call that ensures it is
 * always safe to read 16 bytes before the end of the history buffer.
 */
static
hwlm_error_t safeExecStreaming(const FDR *fdr, const u8 *hbuf, size_t hlen,
                               const u8 *buf, size_t len, size_t start,
                               HWLMCallback cb, hwlm_group_t groups) {
    array<u8, 16> wrapped_history = {{'0', '1', '2', '3', '4', '5', '6', '7',
                                      '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'}};
    if (hlen < 16) {
        u8 *new_hbuf = wrapped_history.data() + 16 - hlen;
        memcpy(new_hbuf, hbuf, hlen);
        hbuf = new_hbuf;
    }
    struct hs_scratch scratch;
    scratch.fdr_conf = NULL;
    return fdrExecStreaming(fdr, hbuf, hlen, buf, len, start, cb, &scratch,
                            groups);
}

TEST_P(FDRp, SmallStreaming) {
    const u32 hint = GetParam();
    SCOPED_TRACE(hint);

    vector<hwlmLiteral> lits = {hwlmLiteral("a", 1, 1),
                                hwlmLiteral("aardvark", 0, 10)};

    auto fdr = buildFDREngineHinted(lits, false, hint, get_current_target(),
                                    Grey());
    CHECK_WITH_TEDDY_OK_TO_FAIL(fdr, hint);

    vector<match> expected;
    expected.push_back(match(0, 1));
    expected.push_back(match(1, 1));
    expected.push_back(match(2, 1));

    safeExecStreaming(fdr.get(), (const u8 *)"", 0, (const u8 *)"aaar", 4, 0,
                      decentCallback, HWLM_ALL_GROUPS);
    for (u32 i = 0; i < MIN(expected.size(), matches.size()); i++) {
        EXPECT_EQ(expected[i], matches[i]);
    }
    ASSERT_TRUE(expected.size() == matches.size());
    expected.clear();
    matches.clear();

    expected.push_back(match(6, 1));
    expected.push_back(match(8, 10));

    safeExecStreaming(fdr.get(), (const u8 *)"aaar", 4, (const u8 *)"dvark", 5,
                      0, decentCallback, HWLM_ALL_GROUPS);

    for (u32 i = 0; i < MIN(expected.size(), matches.size()); i++) {
        EXPECT_EQ(expected[i], matches[i] + 4);
    }
    ASSERT_EQ(expected.size(), matches.size());
    matches.clear();
}

TEST_P(FDRp, SmallStreaming2) {
    const u32 hint = GetParam();
    SCOPED_TRACE(hint);

    vector<hwlmLiteral> lits = {hwlmLiteral("a", 1, 1),
                                hwlmLiteral("kk", 1, 2),
                                hwlmLiteral("aardvark", 0, 10)};

    auto fdr = buildFDREngineHinted(lits, false, hint, get_current_target(),
                                    Grey());
    CHECK_WITH_TEDDY_OK_TO_FAIL(fdr, hint);

    vector<match> expected;
    expected.push_back(match(6,1));
    expected.push_back(match(7,1));
    expected.push_back(match(11,1));
    expected.push_back(match(13,10));
    expected.push_back(match(14,2));
    expected.push_back(match(15,2));

    safeExecStreaming(fdr.get(), (const u8 *)"foobar", 6,
                      (const u8 *)"aardvarkkk", 10, 0, decentCallback,
                      HWLM_ALL_GROUPS);

    for (u32 i = 0; i < MIN(expected.size(), matches.size()); i++) {
        EXPECT_EQ(expected[i], matches[i] + 6);
    }
    ASSERT_EQ(expected.size(), matches.size());
    matches.clear();
}

TEST_P(FDRp, moveByteStream) {
    const u32 hint = GetParam();
    SCOPED_TRACE(hint);
    const char data[] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ12345678901234567890";
    size_t data_len = strlen(data);

    vector<hwlmLiteral> lits;
    lits.push_back(hwlmLiteral("mnopqr", 0, 0));

    auto fdrTable0 = buildFDREngineHinted(lits, false, hint,
                                          get_current_target(), Grey());
    CHECK_WITH_TEDDY_OK_TO_FAIL(fdrTable0, hint);

    size_t size = fdrSize(fdrTable0.get());

    auto fdrTable = make_bytecode_ptr<FDR>(size, 64);
    EXPECT_NE(nullptr, fdrTable);

    memcpy(fdrTable.get(), fdrTable0.get(), size);

    //  bugger up original
    for (size_t i = 0 ; i < size; i++) {
        ((char *)fdrTable0.get())[i] = (i % 2) ? 0xCA : 0xFE;
    }

    // check matches
    struct hs_scratch scratch;
    scratch.fdr_conf = NULL;

    hwlm_error_t fdrStatus = fdrExec(fdrTable.get(), (const u8 *)data,
                                     data_len, 0, decentCallback, &scratch,
                                     HWLM_ALL_GROUPS);
    ASSERT_EQ(0, fdrStatus);

    ASSERT_EQ(1U, matches.size());
    EXPECT_EQ(match(17, 0), matches[0]);
    matches.clear();
}

TEST_P(FDRp, Stream1) {
    const u32 hint = GetParam();
    SCOPED_TRACE(hint);
    const char data1[] = "fffffffffffffffff";
    const char data2[] = "ffffuuuuuuuuuuuuu";
    size_t data_len1 = strlen(data1);
    size_t data_len2 = strlen(data2);
    hwlm_error_t fdrStatus = 1;

    vector<hwlmLiteral> lits;
    lits.push_back(hwlmLiteral("f", 0, 0));
    lits.push_back(hwlmLiteral("literal", 0, 1));

    auto fdr = buildFDREngineHinted(lits, false, hint, get_current_target(),
                                    Grey());
    CHECK_WITH_TEDDY_OK_TO_FAIL(fdr, hint);

    // check matches

    fdrStatus = safeExecStreaming(fdr.get(), (const u8 *)data1, data_len1,
                                  (const u8 *)data2, data_len2, 0,
                                  decentCallback, HWLM_ALL_GROUPS);
    ASSERT_EQ(0, fdrStatus);

    ASSERT_EQ(4U, matches.size());
    for (size_t i = 0; i < matches.size(); i++) {
        EXPECT_EQ(match(i, 0), matches[i]);
    }
    matches.clear();
}

INSTANTIATE_TEST_CASE_P(FDR, FDRp, ValuesIn(getValidFdrEngines()));

typedef struct {
    string pattern;
    unsigned char alien; // character not present in pattern
} pattern_alien_t;

// gtest helper
void PrintTo(const pattern_alien_t &t, ::std::ostream *os) {
    *os << "(" << t.pattern << ", " << t.alien << ")";
}

class FDRpp : public TestWithParam<tuple<u32, pattern_alien_t>> {};

// This test will check if matcher detects properly literals at the beginning
// and at the end of unaligned buffer. It will check as well that match does
// not happen if literal is partially (from 1 character up to full literal
// length) is out of searched buffer - "too early" and "too late" conditions
TEST_P(FDRpp, AlignAndTooEarly) {
    const size_t buf_alignment = 32;
    // Buffer should be big enough to hold two instances of matching literals
    // (up to 64 bytes each) and room for offset (up to 32 bytes)
    const size_t data_len = 5 * buf_alignment;

    const u32 hint = get<0>(GetParam());
    SCOPED_TRACE(hint);

    // pattern which is used to generate literals of variable size - from 1 to 8
    const string &pattern = get<1>(GetParam()).pattern;
    const size_t patLen = pattern.size();
    const unsigned char alien = get<1>(GetParam()).alien;

    // allocate aligned buffer
    auto dataBufAligned = shared_ptr<char>(
        (char *)aligned_malloc_internal(data_len, buf_alignment),
        aligned_free_internal);

    vector<hwlmLiteral> lits;
    struct hs_scratch scratch;
    scratch.fdr_conf = NULL;
    for (size_t litLen = 1; litLen <= patLen; litLen++) {

        // building literal from pattern substring of variable length 1-patLen
        lits.push_back(hwlmLiteral(string(pattern, 0, litLen), 0, 0));
        auto fdr = buildFDREngineHinted(lits, false, hint, get_current_target(),
                                        Grey());
        CHECK_WITH_TEDDY_OK_TO_FAIL(fdr, hint);

        // check with buffer offset from aligned start from 0 to 31
        for (size_t i = 0; i < buf_alignment; i++) {
            // fill the whole buffer with 'alien' character
            memset(dataBufAligned.get(), alien, data_len);
            // put the matching substring to the beginning of unaligned buffer
            memcpy(dataBufAligned.get() + i, pattern.data(), litLen);
            // put the matching substring to the end of unaligned buffer
            memcpy(dataBufAligned.get() + i + 4 * buf_alignment - litLen,
                        pattern.data(), litLen);

            for (size_t j = 0; j <= litLen; j++) {
                hwlm_error_t fdrStatus = fdrExec(fdr.get(),
                        (const u8 *)dataBufAligned.get() + i + j,
                        4 * buf_alignment - j * 2, 0, decentCallback,
                        &scratch, HWLM_ALL_GROUPS);
                ASSERT_EQ(0, fdrStatus);
                // j == 0 means that start and end matches are entirely within
                // searched buffer. Otherwise they are out of buffer boundaries
                // by j number of bytes - "too early" or "too late" conditions
                // j == litLen means that matches are completely put of searched buffer
                if (j == 0) {
                    // we should get two and only two matches - at the beginning and
                    // at the end of unaligned buffer
                    ASSERT_EQ(2U, matches.size());
                    ASSERT_EQ(match(litLen - 1, 0), matches[0]);
                    ASSERT_EQ(match(4 * buf_alignment - 1, 0), matches[1]);
                    matches.clear();
                } else {
                    // "Too early" / "too late" condition - should not match anything
                    ASSERT_EQ(0U, matches.size());
                }
            }
        }
        lits.clear();
    }
}

static const pattern_alien_t test_pattern[] = {
        {"abaabaaa", 'x'},
        {"zzzyyzyz", (unsigned char)'\x99'},
        {"abcdef l", '\0'}
};

INSTANTIATE_TEST_CASE_P(FDR, FDRpp, Combine(ValuesIn(getValidFdrEngines()),
                                            ValuesIn(test_pattern)));

// This test generates an exhaustive set of short input buffers of length from
// 1 to 6 (1092 buffers) and 2750 buffers of length from 7 to >64 constructed
// from arbitrary set of short buffers. All buffers contain 3 characters from
// the alphabet given as a parameter to the test.
// Then it generates an exhaustive set of literals of length from 1 to 8
// containing first two characters from the same alphabet (510 literals)
// Literals are grouped by 32 to run search on each and every buffer.
// All resulting matches are checked.

// Fibonacci sequence is used to generate arbitrary buffers
unsigned long long fib (int n) {
    unsigned long long fib0 = 1, fib1 = 1, fib2 = 1;
    for (int i = 0; i < n; i++) {
        fib2 = fib1 + fib0;
        fib0 = fib1;
        fib1 = fib2;
    }
    return fib2;
}

class FDRpa : public TestWithParam<tuple<u32, array<unsigned char, 3>>> {};

TEST_P(FDRpa, ShortWritings) {
    const u32 hint = get<0>(GetParam());
    SCOPED_TRACE(hint);
    vector<string> bufs;

    // create exhaustive buffer set for up to 6 literals:

    const array<unsigned char, 3> &alphabet = get<1>(GetParam());

    for (int len = 1; len <= 6; len++) {
        for (int j = 0; j < (int)pow((double)3, len); j++) {
            string s;
            for (int k = 0; k < len; k++) {
                s += alphabet[(j / (int)pow((double)3, k) % 3)];
            }
            bufs.push_back(s);
        }
    }
    size_t buflen = bufs.size();

    // create arbitrary buffers from exhaustive set of previously generated 'short'

    for (int len = 7; len < 64; len++) {
        for (int i = 0; i < 10; i++) {
            string s;
            for(int j = 0; (int)s.size() < len; j++) {
                s += bufs[fib(i * 5 + j + (len - 6) * 10) % buflen];
            }
            bufs.push_back(s);
        }
    }

    // generate exhaustive set of literals of length from 1 to 8
    vector<string> pats;
    for (int len = 1; len <= 8; len++) {
        for (int j = 0; j < (int)pow((double)2, len); j++) {
            string s;
            for (int k = 0; k < len; k++) {
                s += alphabet[(j >> k) & 1];
            }
            pats.push_back(s);
        }
    }

    // run the literal matching through all generated literals
    struct hs_scratch scratch;
    scratch.fdr_conf = NULL;
    for (size_t patIdx = 0; patIdx < pats.size();) {
        // group them in the sets of 32
        vector<hwlmLiteral> testSigs;
        for(int i = 0; i < 32 && patIdx < pats.size(); i++, patIdx++) {
            testSigs.push_back(hwlmLiteral(pats[patIdx], false, patIdx));
        }

        auto fdr = buildFDREngineHinted(testSigs, false, hint,
                                        get_current_target(), Grey());

        CHECK_WITH_TEDDY_OK_TO_FAIL(fdr, hint);

        // run the literal matching for the prepared set of 32 literals
        // on each generated buffer
        for (size_t bufIdx = 0; bufIdx < bufs.size(); bufIdx++) {
            const string &buf = bufs[bufIdx];
            size_t bufLen = buf.size();

            hwlm_error_t fdrStatus = fdrExec(fdr.get(), (const u8 *)buf.data(),
                        bufLen, 0, decentCallback, &scratch, HWLM_ALL_GROUPS);
            ASSERT_EQ(0, fdrStatus);

            // build the set of expected matches using standard
            // stl::string::compare() function
            vector<match> expMatches;
            for (size_t pIdx = 0; pIdx < testSigs.size(); pIdx++) {

                const string &pat = testSigs[pIdx].s;
                size_t patLen = pat.size();

                for (int j = 0; j <= (int)bufLen - (int)patLen; j++) {
                    if (!buf.compare(j, patLen, pat)) {
                        expMatches.push_back(match(j + patLen - 1,
                                                testSigs[pIdx].id));
                    }
                }
            }
            // compare the set obtained matches against expected ones
            sort(expMatches.begin(), expMatches.end());
            sort(matches.begin(), matches.end());
            ASSERT_EQ(expMatches, matches);
            matches.clear();
        }
    }
}

static const array<unsigned char, 3> test_alphabet[] = {
    { { 'a', 'b', 'x' } },
    { { 'x', 'y', 'z' } },
    { { '\0', 'A', '\x20' } },
    { { 'a', '\x20', (unsigned char)'\x99' } }
};

INSTANTIATE_TEST_CASE_P(FDR, FDRpa, Combine(ValuesIn(getValidFdrEngines()),
                                            ValuesIn(test_alphabet)));

TEST(FDR, FDRTermS) {
    const char data1[] = "fffffffffffffffff";
    const char data2[] = "ffffuuuuuuuuuuuuu";
    size_t data_len1 = strlen(data1);
    size_t data_len2 = strlen(data2);
    hwlm_error_t fdrStatus = 0;

    vector<hwlmLiteral> lits;
    lits.push_back(hwlmLiteral("f", 0, 0));
    lits.push_back(hwlmLiteral("ff", 0, 1));

    auto fdr = buildFDREngine(lits, false, get_current_target(), Grey());
    ASSERT_TRUE(fdr != nullptr);

    // check matches

    fdrStatus = safeExecStreaming(fdr.get(), (const u8 *)data1, data_len1,
                                  (const u8 *)data2, data_len2, 0,
                                  decentCallbackT, HWLM_ALL_GROUPS);
    ASSERT_EQ(HWLM_TERMINATED, fdrStatus);

    ASSERT_EQ(1U, matches.size());
    matches.clear();
}

TEST(FDR, FDRTermB) {
    const char data1[] = "fffffffffffffffff";
    size_t data_len1 = strlen(data1);
    hwlm_error_t fdrStatus = 0;

    vector<hwlmLiteral> lits;
    lits.push_back(hwlmLiteral("f", 0, 0));
    lits.push_back(hwlmLiteral("ff", 0, 1));

    auto fdr = buildFDREngine(lits, false, get_current_target(), Grey());
    ASSERT_TRUE(fdr != nullptr);

    // check matches
    struct hs_scratch scratch;
    scratch.fdr_conf = NULL;

    fdrStatus = fdrExec(fdr.get(), (const u8 *)data1, data_len1,
                        0, decentCallbackT, &scratch, HWLM_ALL_GROUPS);
    ASSERT_EQ(HWLM_TERMINATED, fdrStatus);

    ASSERT_EQ(1U, matches.size());
    matches.clear();
}
