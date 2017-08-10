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

#include "grey.h"
#include "hs_compile.h" /* for controlling ssse3 usage */
#include "compiler/compiler.h"
#include "nfa/lbr.h"
#include "nfa/nfa_api.h"
#include "nfa/nfa_api_util.h"
#include "nfa/nfa_internal.h"
#include "nfagraph/ng.h"
#include "nfagraph/ng_lbr.h"
#include "nfagraph/ng_util.h"
#include "util/bytecode_ptr.h"
#include "util/charreach.h"
#include "util/compile_context.h"
#include "util/target_info.h"

#include <ostream>

using namespace std;
using namespace testing;
using namespace ue2;

static constexpr u32 MATCH_REPORT = 1024;

struct LbrTestParams {
    CharReach reach;
    u32 min;
    u32 max;

    // Ugly but simple.
    string make_pattern() {
        std::ostringstream oss;
        oss << "^[";
        for (size_t i = reach.find_first(); i != CharReach::npos;
             i = reach.find_next(i)) {
            oss << "\\x" << std::hex << std::setw(2) << std::setfill('0')
                << (unsigned)(i & 0xff) << std::dec;
        }
        oss << "]{" << min << "," << max << "}";
        return oss.str();
    }
};

static
int onMatch(u64a, u64a, ReportID, void *ctx) {
    unsigned *matches = (unsigned *)ctx;
    (*matches)++;
    return MO_CONTINUE_MATCHING;
}

class LbrTest : public TestWithParam<LbrTestParams> {
protected:
    virtual void SetUp() {
        params = GetParam();

        hs_platform_info plat;
        hs_error_t err = hs_populate_platform(&plat);
        ASSERT_EQ(HS_SUCCESS, err);

        target_t target(plat);

        string pattern = params.make_pattern();

        const unsigned flags = 0;
        const Grey grey;
        const CompileContext cc(true, false, target, grey);
        ReportManager rm(cc.grey);
        ParsedExpression parsed(0, pattern.c_str(), flags, 0);
        auto built_expr = buildGraph(rm, cc, parsed);
        const auto &g = built_expr.g;
        ASSERT_TRUE(g != nullptr);
        clearReports(*g);

        rm.setProgramOffset(0, MATCH_REPORT);

        /* LBR triggered by dot */
        vector<vector<CharReach>> triggers = {{CharReach::dot()}};
        nfa = constructLBR(*g, triggers, cc, rm);
        ASSERT_TRUE(nfa != nullptr);

        full_state = make_bytecode_ptr<char>(nfa->scratchStateSize, 64);
        stream_state = make_bytecode_ptr<char>(nfa->streamStateSize);
    }

    virtual void initQueue() {
        q.nfa = nfa.get();
        q.cur = 0;
        q.end = 0;
        q.state = full_state.get();
        q.streamState = stream_state.get();
        q.offset = 0;
        q.buffer = nullptr; // filled in by test
        q.length = 0; // filled in by test
        q.history = nullptr;
        q.hlength = 0;
        q.scratch = nullptr; // not needed by LBR
        q.report_current = 0;
        q.cb = onMatch;
        q.context = &matches;
    }

    virtual string matchingCorpus(size_t len) const {
        string s;
        s.reserve(len);
        size_t c = params.reach.find_first();
        for (size_t i = 0; i < len; i++) {
            s.push_back(c);
            c = params.reach.find_next(c);
            if (c == CharReach::npos) {
                c = params.reach.find_first();
            }
        }
        return s;
    }

    // Params.
    LbrTestParams params;

    // Match count
    unsigned matches;

    // Compiled NFA structure.
    bytecode_ptr<NFA> nfa;

    // Aligned space for full state.
    bytecode_ptr<char> full_state;

    // Space for stream state.
    bytecode_ptr<char> stream_state;

    // Queue structure.
    struct mq q;
};

static const LbrTestParams params[] = {
    { CharReach::dot(), 100, 100 },
    { CharReach::dot(), 10, 100 },
    { CharReach::dot(), 99, 100 },
    { CharReach::dot(), 20, 40 },
    { CharReach("A"), 32, 64 },
    { CharReach("Aa"), 32, 64 },
    { CharReach("0123456789ABCDEFabcdef"), 1000, 1000 },
    { CharReach("0123456789ABCDEFabcdef"), 100, 1000 },
    { CharReach("0123456789ABCDEFabcdef"), 950, 1000 },
    { ~CharReach("a"), 128, 128 },
    { ~CharReach('\x00'), 128, 256 },
    { ~CharReach('\xff'), 128, 256 },
    // set enough bits to create a truffle
    { CharReach("abcXYZ012") | CharReach('\x80') | CharReach('\x91') |
      CharReach('\xAA') | CharReach('\xDE') | CharReach('\xFF'), 32, 64 },
    { CharReach("abcXYZ012") | CharReach('\x80') | CharReach('\x91') |
      CharReach('\xAA') | CharReach('\xDE') | CharReach('\xFF'), 100, 1000 },
};

INSTANTIATE_TEST_CASE_P(Lbr, LbrTest, ValuesIn(params));

TEST_P(LbrTest, MatchMin) {
    ASSERT_FALSE(params.reach.none());

    // string of min length consisting entirely of matching chars.
    const string corpus = matchingCorpus(params.min);

    initQueue();
    q.buffer = (const u8 *)corpus.c_str();
    q.length = corpus.length();
    u64a end = corpus.length();

    nfaQueueInitState(nfa.get(), &q);
    pushQueue(&q, MQE_START, 0);
    pushQueue(&q, MQE_TOP, 0);
    pushQueue(&q, MQE_END, end);

    matches = 0;
    nfaQueueExec(nfa.get(), &q, end);
    ASSERT_EQ(1, matches);
}

TEST_P(LbrTest, MatchMax) {
    ASSERT_FALSE(params.reach.none());

    // string of min length consisting entirely of matching chars.
    const string corpus = matchingCorpus(params.max);

    initQueue();
    q.buffer = (const u8 *)corpus.c_str();
    q.length = corpus.length();
    u64a end = corpus.length();

    nfaQueueInitState(nfa.get(), &q);
    pushQueue(&q, MQE_START, 0);
    pushQueue(&q, MQE_TOP, 0);
    pushQueue(&q, MQE_END, end);

    matches = 0;
    nfaQueueExec(nfa.get(), &q, end);
    ASSERT_EQ(params.max - params.min + 1, matches);
}

TEST_P(LbrTest, InitCompressedState0) {
    char rv = nfaInitCompressedState(nfa.get(), 0, stream_state.get(), '\0');
    ASSERT_NE(0, rv);
}

TEST_P(LbrTest, QueueExecToMatch) {
    ASSERT_FALSE(params.reach.none());

    // string of min length consisting entirely of matching chars.
    const string corpus = matchingCorpus(params.min);

    initQueue();
    q.buffer = (const u8 *)corpus.c_str();
    q.length = corpus.length();
    u64a end = corpus.length();

    nfaQueueInitState(nfa.get(), &q);
    pushQueue(&q, MQE_START, 0);
    pushQueue(&q, MQE_TOP, 0);
    pushQueue(&q, MQE_END, end);

    matches = 0;
    char rv = nfaQueueExecToMatch(nfa.get(), &q, end);
    ASSERT_EQ(MO_MATCHES_PENDING, rv);
    ASSERT_EQ(0, matches);
    ASSERT_NE(0, nfaInAcceptState(nfa.get(), MATCH_REPORT, &q));
    nfaReportCurrentMatches(nfa.get(), &q);
    ASSERT_EQ(1, matches);
}
