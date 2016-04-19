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


extern "C" {
#include "nfa/accel.h" // wrapping in extern C to make sure run_accel works
}

#include "config.h"
#include "src/ue2common.h"

#include "gtest/gtest.h"
#include "nfagraph/ng_limex_accel.h"
#include "nfa/accelcompile.h"
#include "nfa/multivermicelli.h"
#include "nfa/multishufti.h"
#include "nfa/multitruffle.h"
#include "util/alloc.h"
#include "util/charreach.h"

#include <string>
#include <vector>
#include <algorithm>
#include <stdlib.h>
#include <iostream>

using namespace ue2;
using namespace std;
using namespace testing;

/*
 * Static functions needed for this test's wellbeing
 */

// char generator
static inline
char getChar(const CharReach &cr, bool match) {
    char result;
    do {
        result = rand() % CharReach::npos;
    } while (cr.test(result) != match);
    return result;
}

// appends a string with matches/unmatches according to input match pattern
static
void getMatch(u8 *result, u32 start, const string &pattern,
              const CharReach &cr) {
    for (const auto &c : pattern) {
        result[start++] = getChar(cr, c == '1');
    }
}

// appends non-matching noise of certain lengths
static
void getNoise(u8 *result, u32 start, u32 len, const CharReach &cr) {
    for (unsigned i = 0; i < len; i++) {
        result[start + i] = getChar(cr, false);
    }
}

// test parameters structure
struct MultiaccelTestParam {
    string match_pattern;
    u32 match_pattern_start_idx;
    u32 match_idx;
    bool test_all_offsets;
    u8 match_len1;
    u8 match_len2;
    MultibyteAccelInfo::multiaccel_type type;
};

// buffer size is constant
static const u32 BUF_SIZE = 200;

// strings, out of which CharReach will be generated
static const string VERM_CR = "a";
static const string V_NC_CR = "aA";
static const string SHUF_CR = "abcdefghijklmnopqrstuvwxyz";
static const string TRUF_CR = "\x11\x22\x33\x44\x55\x66\x77\x88\x99";

// Parameterized test case for multiaccel patterns.
class MultiaccelTest : public TestWithParam<MultiaccelTestParam> {
protected:
    virtual void SetUp() {
        // set up is deferred until the actual test, since we can't compile
        // any accel schemes unless we know CharReach
        const MultiaccelTestParam &p = GetParam();

        // reserve space in our buffer
        buffer = (u8 *)aligned_zmalloc(BUF_SIZE);

        // store the index where we expect to see the match. note that it may
        // be different from where the match pattern has started since we may
        // have a flooded match (i.e. a match preceded by almost-match) or a
        // no-match (in which case "match" index is at the end of the buffer).
        match_idx = p.match_idx;

        // make note if we need to test all offsets - sometimes we don't, for
        // example when testing partial or no-match.
        test_all_offsets = p.test_all_offsets;
    }

    // deferred buffer generation, as we don't know CharReach before we run the test
    void GenerateBuffer(const CharReach &cr) {
        const MultiaccelTestParam &p = GetParam();

        // step 1: fill prefix with non-matching noise
        u32 start = 0;
        getNoise(buffer, start, p.match_pattern_start_idx, cr);

        // step 2: add a match
        start += p.match_pattern_start_idx;
        getMatch(buffer, start, p.match_pattern, cr);

        // step 3: fill in the rest of the buffer with non-matching noise
        start += p.match_pattern.size();
        getNoise(buffer, start, BUF_SIZE - p.match_pattern.size() -
                 p.match_pattern_start_idx, cr);
    }

    // deferred accel scheme generation, as we don't know CharReach before we run the test
    void CompileAccelScheme(const CharReach &cr, AccelAux *aux) {
        const MultiaccelTestParam &p = GetParam();

        AccelInfo ai;
        ai.single_stops = cr; // dummy CharReach to prevent red tape accel
        ai.ma_len1 = p.match_len1;
        ai.ma_len2 = p.match_len2;
        ai.multiaccel_stops = cr;
        ai.ma_type = p.type;

        buildAccelAux(ai, aux);

        // now, verify we've successfully built our accel scheme, *and* that it's
        // a multibyte scheme
        ASSERT_TRUE(aux->accel_type >= ACCEL_MLVERM &&
                    aux->accel_type <= ACCEL_MDSGTRUFFLE);
    }

    virtual void TearDown() {
        aligned_free(buffer);
    }

    u32 match_idx;
    u8 *buffer;
    bool test_all_offsets;
};

static
void runTest(const u8 *buffer, AccelAux *aux, unsigned match_idx,
             bool test_all_offsets) {
    const u8 *start = buffer;
    const u8 *end = start + BUF_SIZE;
    const u8 *match = start + match_idx;

    // comparing indexes into the buffer is easier to understand than pointers
    if (test_all_offsets) {
        // run_accel can only scan >15 byte buffers
        u32 end_offset = min(match_idx, BUF_SIZE - 15);

        for (unsigned offset = 0; offset < end_offset; offset++) {
            const u8 *ptr = run_accel(aux, (start + offset), end);
            unsigned idx = ptr - start;
            ASSERT_EQ(match_idx, idx);
        }
    } else {
        const u8 *ptr = run_accel(aux, start, end);
        unsigned idx = ptr - start;
        ASSERT_EQ(match_idx, idx);
    }
}

TEST_P(MultiaccelTest, TestVermicelli) {
    AccelAux aux = {0};
    CharReach cr(VERM_CR);

    GenerateBuffer(cr);

    CompileAccelScheme(cr, &aux);

    runTest(buffer, &aux, match_idx, test_all_offsets);
}

TEST_P(MultiaccelTest, TestVermicelliNocase) {
    AccelAux aux = {0};
    CharReach cr(V_NC_CR);

    GenerateBuffer(cr);

    CompileAccelScheme(cr, &aux);

    runTest(buffer, &aux, match_idx, test_all_offsets);
}

TEST_P(MultiaccelTest, TestShufti) {
    AccelAux aux = {0};
    CharReach cr(SHUF_CR);

    GenerateBuffer(cr);

    CompileAccelScheme(cr, &aux);

    runTest(buffer, &aux, match_idx, test_all_offsets);
}

TEST_P(MultiaccelTest, TestTruffle) {
    AccelAux aux = {0};
    CharReach cr(TRUF_CR);

    GenerateBuffer(cr);

    CompileAccelScheme(cr, &aux);

    runTest(buffer, &aux, match_idx, test_all_offsets);
}

static const MultiaccelTestParam multiaccelTests[] = {
    // long matcher

    // full, partial, flooded, nomatch
    {"11111", 180, 180, true, 5, 0, MultibyteAccelInfo::MAT_LONG},
    {"111", 197, 197, true, 5, 0, MultibyteAccelInfo::MAT_LONG},
    {"1111011111", 177, 182, false, 5, 0, MultibyteAccelInfo::MAT_LONG},
    {"1111011110", 177, 200, false, 5, 0, MultibyteAccelInfo::MAT_LONG},

    // long-grab matcher

    // full, partial, flooded, nomatch
    {"111110", 180, 180, true, 5, 0, MultibyteAccelInfo::MAT_LONGGRAB},
    {"111", 197, 197, true, 5, 0, MultibyteAccelInfo::MAT_LONGGRAB},
    {"11111111110", 177, 182, false, 5, 0, MultibyteAccelInfo::MAT_LONGGRAB},
    {"11110111101", 177, 200, false, 5, 0, MultibyteAccelInfo::MAT_LONGGRAB},

    // shift matcher

    // full, partial, flooded, nomatch
    {"11001", 180, 180, true, 4, 0, MultibyteAccelInfo::MAT_SHIFT},
    {"110", 197, 197, true, 4, 0, MultibyteAccelInfo::MAT_SHIFT},
    {"1001011001", 177, 182, false, 4, 0, MultibyteAccelInfo::MAT_SHIFT},
    {"1101001011", 177, 200, false, 4, 0, MultibyteAccelInfo::MAT_SHIFT},

    // shift-grab matcher

    // full, partial, flooded, nomatch
    {"10111", 180, 180, true, 4, 0, MultibyteAccelInfo::MAT_SHIFTGRAB},
    {"101", 197, 197, true, 4, 0, MultibyteAccelInfo::MAT_SHIFTGRAB},
    {"1110010111", 177, 182, false, 4, 0, MultibyteAccelInfo::MAT_SHIFTGRAB},
    {"1100101100", 177, 200, false, 4, 0, MultibyteAccelInfo::MAT_SHIFTGRAB},

    // doubleshift matcher

    // full, partial (one and two shifts), flooded, nomatch
    {"110111", 180, 180, true, 3, 2, MultibyteAccelInfo::MAT_DSHIFT},
    {"110", 197, 197, true, 3, 2, MultibyteAccelInfo::MAT_DSHIFT},
    {"1101", 196, 196, true, 3, 2, MultibyteAccelInfo::MAT_DSHIFT},
    {"1100100101", 178, 182, false, 3, 2, MultibyteAccelInfo::MAT_DSHIFT},
    {"1101001101", 177, 200, false, 3, 2, MultibyteAccelInfo::MAT_DSHIFT},

    // doubleshift-grab matcher

    // full, partial (one and two shifts), flooded, nomatch
    {"100101", 180, 180, true, 3, 2, MultibyteAccelInfo::MAT_DSHIFTGRAB},
    {"100", 197, 197, true, 3, 2, MultibyteAccelInfo::MAT_DSHIFTGRAB},
    {"1011", 196, 196, true, 3, 2, MultibyteAccelInfo::MAT_DSHIFTGRAB},
    {"11111101101", 177, 182, false, 3, 2, MultibyteAccelInfo::MAT_DSHIFTGRAB},
    {"1111110111", 177, 200, false, 3, 2, MultibyteAccelInfo::MAT_DSHIFTGRAB},
};

INSTANTIATE_TEST_CASE_P(Multiaccel, MultiaccelTest, ValuesIn(multiaccelTests));

// boring stuff for google test
void PrintTo(const MultiaccelTestParam &p, ::std::ostream *os) {
    *os << "MultiaccelTestParam: " << p.match_pattern;
}
