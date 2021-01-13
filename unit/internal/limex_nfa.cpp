/*
 * Copyright (c) 2015-2020, Intel Corporation
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
#include "compiler/compiler.h"
#include "nfa/limex_internal.h"
#include "nfa/nfa_api.h"
#include "nfa/nfa_api_util.h"
#include "nfa/nfa_internal.h"
#include "nfagraph/ng.h"
#include "nfagraph/ng_limex.h"
#include "nfagraph/ng_util.h"
#include "util/bytecode_ptr.h"
#include "util/target_info.h"

using namespace std;
using namespace testing;
using namespace ue2;

static const string SCAN_DATA = "___foo______\n___foofoo_foo_^^^^^^^^^^^^^^^^^^"
                                "^^^^__bar_bar______0_______z_____bar";
static const u32 MATCH_REPORT = 1024;

static
int onMatch(u64a, u64a, ReportID, void *ctx) {
    unsigned *matches = (unsigned *)ctx;
    (*matches)++;
    return MO_CONTINUE_MATCHING;
}

// Parameterized with LimEx model and target flags.
class LimExModelTest : public TestWithParam<int> {
protected:
    virtual void SetUp() {
        type = GetParam();
        hs_platform_info plat;
        hs_error_t err = hs_populate_platform(&plat);
        ASSERT_EQ(HS_SUCCESS, err);

        target_t target(plat);
        matches = 0;

        const string expr = "(foo.*bar)|end\\z";
        const unsigned flags = 0;
        CompileContext cc(false, false, target, Grey());
        ReportManager rm(cc.grey);
        ParsedExpression parsed(0, expr.c_str(), flags, 0);
        auto built_expr = buildGraph(rm, cc, parsed);
        const auto &g = built_expr.g;
        ASSERT_TRUE(g != nullptr);
        clearReports(*g);

        rm.setProgramOffset(0, MATCH_REPORT);

        const map<u32, u32> fixed_depth_tops;
        const map<u32, vector<vector<CharReach>>> triggers;
        bool compress_state = false;
        bool fast_nfa = false;

        nfa = constructNFA(*g, &rm, fixed_depth_tops, triggers, compress_state,
                           fast_nfa, type, cc);
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
        q.buffer = (const u8 *)SCAN_DATA.c_str();
        q.length = SCAN_DATA.size();
        q.history = nullptr;
        q.hlength = 0;
        q.scratch = nullptr; /* limex does not use scratch */
        q.report_current = 0;
        q.cb = onMatch;
        q.context = &matches;
    }

    // NFA type (enum NFAEngineType)
    int type;

    // Match count
    unsigned matches;

    // Compiled NFA structure.
    bytecode_ptr<NFA> nfa;

    // Space for full state.
    bytecode_ptr<char> full_state;

    // Space for stream state.
    bytecode_ptr<char> stream_state;

    // Queue structure.
    struct mq q;
};

INSTANTIATE_TEST_CASE_P(
    LimEx, LimExModelTest,
    Range((int)LIMEX_NFA_32, (int)LIMEX_NFA_512));

TEST_P(LimExModelTest, StateSize) {
    ASSERT_TRUE(nfa != nullptr);

    hs_platform_info plat;
    hs_error_t err = hs_populate_platform(&plat);
    ASSERT_EQ(HS_SUCCESS, err);

    target_t target(plat);

    // About all we can say is that any NFA should require at least one byte of
    // state space.

    EXPECT_LT(0, nfa->scratchStateSize);
    EXPECT_LT(0, nfa->streamStateSize);
}

TEST_P(LimExModelTest, QueueExec) {
    ASSERT_TRUE(nfa != nullptr);
    initQueue();
    nfaQueueInitState(nfa.get(), &q);

    u64a end = SCAN_DATA.size();
    pushQueue(&q, MQE_START, 0);
    pushQueue(&q, MQE_TOP, 0);
    pushQueue(&q, MQE_END, end);

    nfaQueueExec(nfa.get(), &q, end);

    ASSERT_EQ(3, matches);
}

TEST_P(LimExModelTest, CompressExpand) {
    ASSERT_TRUE(nfa != nullptr);

    u32 real_state_size = nfa->scratchStateSize;
    /* Only look at 8 bytes for limex 64 (rather than the padding) */
    if (nfa->type == LIMEX_NFA_64) {
        real_state_size = sizeof(u64a);
    }

    initQueue();
    nfaQueueInitState(nfa.get(), &q);

    // Do some scanning.
    u64a end = SCAN_DATA.size();
    pushQueue(&q, MQE_START, 0);
    pushQueue(&q, MQE_TOP, 0);
    pushQueue(&q, MQE_END, end);
    nfaQueueExec(nfa.get(), &q, end);

    // Compress state.
    nfaQueueCompressState(nfa.get(), &q, end);

    // Expand state into a new copy and check that it matches the original
    // uncompressed state.
    auto state_copy = make_bytecode_ptr<char>(nfa->scratchStateSize, 64);
    char *dest = state_copy.get();
    memset(dest, 0xff, nfa->scratchStateSize);
    nfaExpandState(nfa.get(), dest, q.streamState, q.offset,
                   queue_prev_byte(&q, end));
    ASSERT_TRUE(std::equal(dest, dest + real_state_size, full_state.get()));
}

TEST_P(LimExModelTest, InitCompressedState0) {
    ASSERT_TRUE(nfa != nullptr);

    // 64-bit NFAs assume during compression that they have >= 5 bytes of
    // compressed NFA state, which isn't true for our 8-state test pattern. We
    // skip this test for just these models.
    if (nfa->scratchStateSize == 8) {
        return;
    }

    // Trivial case: init at zero, like we do with outfixes.
    char rv = nfaInitCompressedState(nfa.get(), 0, stream_state.get(), '\0');
    ASSERT_NE(0, rv);
}

TEST_P(LimExModelTest, QueueExecToMatch) {
    ASSERT_TRUE(nfa != nullptr);
    initQueue();
    nfaQueueInitState(nfa.get(), &q);

    u64a end = SCAN_DATA.size();
    pushQueue(&q, MQE_START, 0);
    pushQueue(&q, MQE_TOP, 0);
    pushQueue(&q, MQE_END, end);

    // FIRST MATCH (of three).

    char rv = nfaQueueExecToMatch(nfa.get(), &q, end);
    ASSERT_EQ(MO_MATCHES_PENDING, rv);
    ASSERT_EQ(0, matches);
    ASSERT_NE(0, nfaInAcceptState(nfa.get(), MATCH_REPORT, &q));
    nfaReportCurrentMatches(nfa.get(), &q);
    ASSERT_EQ(1, matches);

    // SECOND MATCH (of three).

    rv = nfaQueueExecToMatch(nfa.get(), &q, end);
    ASSERT_EQ(MO_MATCHES_PENDING, rv);
    ASSERT_EQ(1, matches);
    ASSERT_NE(0, nfaInAcceptState(nfa.get(), MATCH_REPORT, &q));
    nfaReportCurrentMatches(nfa.get(), &q);
    ASSERT_EQ(2, matches);

    // THIRD MATCH (of three).

    rv = nfaQueueExecToMatch(nfa.get(), &q, end);
    ASSERT_EQ(MO_MATCHES_PENDING, rv);
    ASSERT_EQ(2, matches);
    ASSERT_NE(0, nfaInAcceptState(nfa.get(), MATCH_REPORT, &q));
    nfaReportCurrentMatches(nfa.get(), &q);
    ASSERT_EQ(3, matches);

    // No more.

    rv = nfaQueueExecToMatch(nfa.get(), &q, end);
    ASSERT_EQ(MO_ALIVE, rv);
    ASSERT_EQ(3, matches);
}

TEST_P(LimExModelTest, QueueExecRose) {
    ASSERT_TRUE(nfa != nullptr);
    initQueue();

    // For rose, there's no callback or context.
    q.cb = nullptr;
    q.context = nullptr;

    nfaQueueInitState(nfa.get(), &q);

    u64a end = SCAN_DATA.size();
    pushQueue(&q, MQE_START, 0);
    pushQueue(&q, MQE_TOP, 0);
    pushQueue(&q, MQE_END, end);

    char rv = nfaQueueExecRose(nfa.get(), &q, MATCH_REPORT);
    ASSERT_EQ(MO_MATCHES_PENDING, rv);
    pushQueue(&q, MQE_START, end);
    ASSERT_NE(0, nfaInAcceptState(nfa.get(), MATCH_REPORT, &q));
}

TEST_P(LimExModelTest, CheckFinalState) {
    ASSERT_TRUE(nfa != nullptr);
    initQueue();
    nfaQueueInitState(nfa.get(), &q);

    // Do some scanning.
    u64a end = SCAN_DATA.size();
    pushQueue(&q, MQE_START, 0);
    pushQueue(&q, MQE_TOP, 0);
    pushQueue(&q, MQE_END, end);
    nfaQueueExec(nfa.get(), &q, end);
    ASSERT_EQ(3, matches);

    // Check for EOD matches.
    char rv = nfaCheckFinalState(nfa.get(), full_state.get(),
                                 stream_state.get(), end, onMatch, &matches);
    ASSERT_EQ(MO_CONTINUE_MATCHING, rv);
}

// For testing the _B_Reverse backwards-scanning block-mode path.
class LimExReverseTest : public TestWithParam<int> {
protected:
    virtual void SetUp() {
        type = GetParam();
        matches = 0;

        const string expr = "foo.*bar";
        const unsigned flags = 0;
        CompileContext cc(false, false, get_current_target(), Grey());
        ReportManager rm(cc.grey);
        ParsedExpression parsed(0, expr.c_str(), flags, 0);
        auto built_expr = buildGraph(rm, cc, parsed);
        const auto &g = built_expr.g;
        ASSERT_TRUE(g != nullptr);
        clearReports(*g);

        // Reverse the graph and add some reports on the accept vertices.
        NGHolder g_rev(NFA_REV_PREFIX);
        reverseHolder(*g, g_rev);
        clearReports(g_rev);
        for (NFAVertex v : inv_adjacent_vertices_range(g_rev.accept, g_rev)) {
            g_rev[v].reports.insert(0);
        }

        nfa = constructReversedNFA(g_rev, type, cc);
        ASSERT_TRUE(nfa != nullptr);
    }

    // NFA type (enum NFAEngineType)
    int type;

    // Match count
    unsigned matches;

    // Compiled NFA structure.
    bytecode_ptr<NFA> nfa;
};

INSTANTIATE_TEST_CASE_P(LimExReverse, LimExReverseTest,
                        Range((int)LIMEX_NFA_32, (int)LIMEX_NFA_512));

TEST_P(LimExReverseTest, BlockExecReverse) {
    ASSERT_TRUE(nfa != nullptr);

    u64a offset = 0;
    const u8 *buf = (const u8 *)SCAN_DATA.c_str();
    const size_t buflen = SCAN_DATA.size();
    const u8 *hbuf = nullptr;
    const size_t hlen = 0;

    nfaBlockExecReverse(nfa.get(), offset, buf, buflen, hbuf, hlen,
                        onMatch, &matches);

    ASSERT_EQ(3, matches);
}

// Test the ZOMBIE path.

static const string ZOMBIE_SCAN_DATA = "braaaiiiiiins!!!!!";

class LimExZombieTest : public TestWithParam<int> {
protected:
    virtual void SetUp() {
        type = GetParam();
        matches = 0;

        const string expr = "bra+i+ns.*";
        const unsigned flags = HS_FLAG_DOTALL;
        CompileContext cc(true, false, get_current_target(), Grey());
        ParsedExpression parsed(0, expr.c_str(), flags, 0);
        ReportManager rm(cc.grey);
        auto built_expr = buildGraph(rm, cc, parsed);
        const auto &g = built_expr.g;
        ASSERT_TRUE(g != nullptr);
        clearReports(*g);

        rm.setProgramOffset(0, MATCH_REPORT);

        const map<u32, u32> fixed_depth_tops;
        const map<u32, vector<vector<CharReach>>> triggers;
        bool compress_state = false;
        bool fast_nfa = false;

        nfa = constructNFA(*g, &rm, fixed_depth_tops, triggers, compress_state,
                           fast_nfa, type, cc);
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
        q.buffer = (const u8 *)ZOMBIE_SCAN_DATA.c_str();
        q.length = ZOMBIE_SCAN_DATA.length();
        q.history = nullptr;
        q.hlength = 0;
        q.scratch = nullptr; /* limex does not use scratch */
        q.report_current = 0;
        q.cb = onMatch;
        q.context = &matches;
    }

    // NFA type (enum NFAEngineType)
    int type;

    // Match count
    unsigned matches;

    // Compiled NFA structure.
    bytecode_ptr<NFA> nfa;

    // Space for full state.
    bytecode_ptr<char> full_state;

    // Space for stream state.
    bytecode_ptr<char> stream_state;

    // Queue structure.
    struct mq q;
};

INSTANTIATE_TEST_CASE_P(LimExZombie, LimExZombieTest,
                        Range((int)LIMEX_NFA_32, (int)LIMEX_NFA_512));

TEST_P(LimExZombieTest, GetZombieStatus) {
    ASSERT_TRUE(nfa != nullptr);
    ASSERT_TRUE(nfa->flags & NFA_ZOMBIE);

    initQueue();
    nfaQueueInitState(nfa.get(), &q);

    // Not a zombie yet
    ASSERT_EQ(NFA_ZOMBIE_NO, nfaGetZombieStatus(nfa.get(), &q, 0));

    u64a end = q.length;
    pushQueue(&q, MQE_START, 0);
    pushQueue(&q, MQE_TOP, 0);
    pushQueue(&q, MQE_END, end);

    nfaQueueExec(nfa.get(), &q, end);

    ASSERT_EQ(6, matches); // one plus the number of '!' chars

    // The .* at the end of the pattern should have turned us into a zombie...
    ASSERT_EQ(NFA_ZOMBIE_ALWAYS_YES, nfaGetZombieStatus(nfa.get(), &q, end));
}
