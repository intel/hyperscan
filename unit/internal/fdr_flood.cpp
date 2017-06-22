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
#include "scratch.h"
#include "util/alloc.h"
#include "util/bitutils.h"

#include "gtest/gtest.h"

using namespace std;
using namespace testing;
using namespace ue2;

#define NO_TEDDY_FAIL_ALLOWED 0

#if(NO_TEDDY_FAIL_ALLOWED)
#define CHECK_WITH_TEDDY_OK_TO_FAIL(fdr, hint) ASSERT_TRUE(fdr)
#else
#define CHECK_WITH_TEDDY_OK_TO_FAIL(fdr, hint)                                 \
    {                                                                          \
        auto descr = getTeddyDescription(hint);                                \
        if (descr && fdr != nullptr) {                                         \
            return;                                                            \
        } else {                                                               \
            ASSERT_TRUE(fdr != nullptr);                                       \
        }                                                                      \
    }
#endif

namespace {

struct match {
    size_t end;
    u32 id;
    match(size_t end_in, u32 id_in) : end(end_in), id(id_in) {}
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

template<typename T>
T &operator<<(T &a, const match &b) {
    a << "(" << b.end << ", " << b.id << ")";
    return a;
}

template<typename T>
T &operator<<(T &a, const vector<match> &b) {
    a << "(";
    for (size_t i = 0; i < b.size(); i++) {
        a << b[i];
    }
    a << ")";
    return a;
}

map<u32, int> matchesCounts;

extern "C" {

static hwlmcb_rv_t countCallback(UNUSED size_t end, u32 id,
                                 UNUSED struct hs_scratch *scratch) {
    matchesCounts[id]++;
    return HWLM_CONTINUE_MATCHING;
}

} // extern "C"

} // namespace

static vector<u32> getValidFdrEngines() {
    vector<u32> ret;

    vector<FDREngineDescription> des;
    getFdrDescriptions(&des);
    for (vector<FDREngineDescription>::const_iterator it = des.begin();
         it != des.end(); ++it) {
        if (it->isValidOnTarget(get_current_target())) {
            ret.push_back(it->getID());
        }
    }
    vector<TeddyEngineDescription> tDes;
    getTeddyDescriptions(&tDes);
    for (vector<TeddyEngineDescription>::const_iterator it = tDes.begin();
         it != tDes.end(); ++it) {
        if (it->isValidOnTarget(get_current_target())) {
            ret.push_back(it->getID());
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
    return fdrBuildTable(*proto, grey);
}

class FDRFloodp : public TestWithParam<u32> {
};

TEST_P(FDRFloodp, NoMask) {
    const u32 hint = GetParam();
    SCOPED_TRACE(hint);
    const size_t dataSize = 1024;
    vector<u8> data(dataSize);
    u8 c = 0;

    struct hs_scratch scratch;
    scratch.fdr_conf = NULL;
    while (1) {
        SCOPED_TRACE((unsigned int)c);
        u8 bit = 1 << (c & 0x7);
        u8 cAlt = c ^ bit;
        memset(&data[0], c, dataSize);

        vector<hwlmLiteral> lits;

        // build literals of type "aaaa", "aaab", "baaa"
        // of lengths 1, 2, 4, 8, both case-less and case-sensitive
        for (int i = 0; i < 4; i++) {
            string s(1 << i, c);
            lits.push_back(hwlmLiteral(s, false, i * 8 + 0));
            s[0] = cAlt;
            lits.push_back(hwlmLiteral(s, false, i * 8 + 1));
            lits.push_back(hwlmLiteral(s, true,  i * 8 + 2));
            s[0] = c;
            s[s.size() - 1] = cAlt;
            lits.push_back(hwlmLiteral(s, false, i * 8 + 3));
            lits.push_back(hwlmLiteral(s, true,  i * 8 + 4));
            string sAlt(1 << i, cAlt);
            lits.push_back(hwlmLiteral(sAlt, true,  i * 8 + 5));
            sAlt[0] = c;
            lits.push_back(hwlmLiteral(sAlt, true,  i * 8 + 6));
            lits.push_back(hwlmLiteral(sAlt, false, i * 8 + 7));
        }

        auto fdr = buildFDREngineHinted(lits, false, hint, get_current_target(),
                                        Grey());
        CHECK_WITH_TEDDY_OK_TO_FAIL(fdr, hint);

        hwlm_error_t fdrStatus = fdrExec(fdr.get(), &data[0], dataSize,
                    0, countCallback, &scratch, HWLM_ALL_GROUPS);
        ASSERT_EQ(0, fdrStatus);

        for (u8 i = 0; i < 4; i++) {
            u32 cnt = dataSize - (1 << i) + 1;
            ASSERT_EQ(cnt, matchesCounts[i * 8 + 0]);
            ASSERT_EQ(0, matchesCounts[i * 8 + 1]);
            ASSERT_EQ(0, matchesCounts[i * 8 + 3]);
            ASSERT_EQ(i == 0 ? cnt : 0, matchesCounts[i * 8 + 7]);
            if (isalpha(c) && (bit == CASE_BIT)) {
                ASSERT_EQ(cnt, matchesCounts[i * 8 + 2]);
                ASSERT_EQ(cnt, matchesCounts[i * 8 + 4]);
                ASSERT_EQ(cnt, matchesCounts[i * 8 + 5]);
                ASSERT_EQ(cnt, matchesCounts[i * 8 + 6]);
            } else {
                ASSERT_EQ(0, matchesCounts[i * 8 + 2]);
                ASSERT_EQ(0, matchesCounts[i * 8 + 4]);
                ASSERT_EQ(0, matchesCounts[i * 8 + 5]);
                ASSERT_EQ(i == 0 ? cnt : 0, matchesCounts[i * 8 + 6]);
            }
        }

        matchesCounts.clear();
        memset(&data[0], cAlt, dataSize);
        fdrStatus = fdrExec(fdr.get(), &data[0], dataSize,
                    0, countCallback, &scratch, HWLM_ALL_GROUPS);
        ASSERT_EQ(0, fdrStatus);

        for (u8 i = 0; i < 4; i++) {
            u32 cnt = dataSize - (1 << i) + 1;
            ASSERT_EQ(0, matchesCounts[i * 8 + 0]);
            ASSERT_EQ(i == 0 ? cnt : 0, matchesCounts[i * 8 + 1]);
            ASSERT_EQ(i == 0 ? cnt : 0, matchesCounts[i * 8 + 3]);
            ASSERT_EQ(cnt, matchesCounts[i * 8 + 5]);
            ASSERT_EQ(0, matchesCounts[i * 8 + 7]);
            if (isalpha(c) && (bit == CASE_BIT)) {
                ASSERT_EQ(cnt, matchesCounts[i * 8 + 2]);
                ASSERT_EQ(cnt, matchesCounts[i * 8 + 4]);
                ASSERT_EQ(cnt, matchesCounts[i * 8 + 6]);
            } else {
                ASSERT_EQ(i == 0 ? cnt : 0, matchesCounts[i * 8 + 2]);
                ASSERT_EQ(i == 0 ? cnt : 0, matchesCounts[i * 8 + 4]);
                ASSERT_EQ(0, matchesCounts[i * 8 + 6]);
            }
        }
        matchesCounts.clear();

        if (++c == 0) {
            break;
        }
    }
}

TEST_P(FDRFloodp, WithMask) {
    const u32 hint = GetParam();
    SCOPED_TRACE(hint);
    const size_t dataSize = 1024;
    vector<u8> data(dataSize);
    u8 c = '\0';

    struct hs_scratch scratch;
    scratch.fdr_conf = NULL;
    while (1) {
        u8 bit = 1 << (c & 0x7);
        u8 cAlt = c ^ bit;
        SCOPED_TRACE((unsigned int)c);
        memset(&data[0], c, dataSize);

        vector<hwlmLiteral> lits;

        // literals of type "aaaa" of length 4, 12, and mask of length
        // 1, 2, 4, 8, 0f type "b.......", "b......b" both case-less and case-sensitive
        string s4(4, c);
        string s4Alt(4, cAlt);

        for (int i = 0; i < 4 ; i++) {
            u32 mskLen = 1 << i;
            vector<u8> msk(mskLen, '\0');
            vector<u8> cmp(mskLen, '\0');

            cmp[0] = cAlt;
            msk[0] = '\xff';
            // msk[f0000000] cmp[c0000000] lit[aaaa]
            if (mskLen > s4.length()) {
                lits.push_back(hwlmLiteral(s4, false, false, i * 12 + 0,
                                                    HWLM_ALL_GROUPS, msk, cmp));
                lits.push_back(hwlmLiteral(s4, true, false, i * 12 + 1,
                                                    HWLM_ALL_GROUPS, msk, cmp));
            }
            // msk[f0000000] cmp[e0000000] lit[EEEE]
            if (bit == CASE_BIT && isalpha(c)) {
                lits.push_back(hwlmLiteral(s4, true, false, i * 12 + 2,
                                                    HWLM_ALL_GROUPS, msk, cmp));
            }
            // msk[E0000000] cmp[E0000000] lit[eeee]
            if ((cAlt & bit) == 0) {
                msk[0] = ~bit;
                lits.push_back(hwlmLiteral(s4, false, false, i * 12 + 3,
                                                    HWLM_ALL_GROUPS, msk, cmp));
                lits.push_back(hwlmLiteral(s4, true, false, i * 12 + 4,
                                                    HWLM_ALL_GROUPS, msk, cmp));
            }
            // msk[f0000000] cmp[a0000000] lit[aaaa]
            cmp[0] = c;
            msk[0] = '\xff';
            lits.push_back(hwlmLiteral(s4, false, false, i * 12 + 5,
                                                HWLM_ALL_GROUPS, msk, cmp));
            lits.push_back(hwlmLiteral(s4, true, false, i * 12 + 6,
                                                HWLM_ALL_GROUPS, msk, cmp));
            // msk[f0000000] cmp[a0000000] lit[cccc]
            if (mskLen > s4Alt.length()) {
                lits.push_back(hwlmLiteral(s4Alt, false, false, i * 12 + 7,
                                                    HWLM_ALL_GROUPS, msk, cmp));
                lits.push_back(hwlmLiteral(s4Alt, true, false, i * 12 + 8,
                                                    HWLM_ALL_GROUPS, msk, cmp));
            }
            if (bit == CASE_BIT && isalpha(c)) {
                // msk[f0000000] cmp[e0000000] lit[EEEE]
                lits.push_back(hwlmLiteral(s4Alt, true, false, i * 12 + 9,
                                                    HWLM_ALL_GROUPS, msk, cmp));

                // msk[f0000000] cmp[e000000E] lit[eeee]
                cmp[mskLen - 1] = cAlt;
                msk[mskLen - 1] = '\xff';
                lits.push_back(hwlmLiteral(s4, true, false, i * 12 + 10,
                                                    HWLM_ALL_GROUPS, msk, cmp));
                // msk[f0000000] cmp[E000000E] lit[eeee]
                cmp[0] = cAlt;
                lits.push_back(hwlmLiteral(s4, true, false, i * 12 + 11,
                                                    HWLM_ALL_GROUPS, msk, cmp));
            }
        }
        auto fdr = buildFDREngineHinted(lits, false, hint, get_current_target(),
                                        Grey());
        CHECK_WITH_TEDDY_OK_TO_FAIL(fdr, hint);

        hwlm_error_t fdrStatus = fdrExec(fdr.get(), &data[0], dataSize,
                             0, countCallback, &scratch, HWLM_ALL_GROUPS);
        ASSERT_EQ(0, fdrStatus);

        const u32 cnt4 = dataSize - 4 + 1;
        for (u8 i = 0; i < 4; i++) {
            u32 mskLen = 1 << i;
            u32 cntMask = MIN(cnt4, dataSize - mskLen + 1);
            ASSERT_EQ(0, matchesCounts[i * 12 + 0]);
            ASSERT_EQ(0, matchesCounts[i * 12 + 1]);
            ASSERT_EQ(0, matchesCounts[i * 12 + 2]);
            if ((cAlt & bit) == 0) {
                ASSERT_EQ(cntMask, matchesCounts[i * 12 + 3]);
                ASSERT_EQ(cntMask, matchesCounts[i * 12 + 4]);
            }
            if (mskLen > 4) {
                ASSERT_EQ(cntMask, matchesCounts[i * 12 + 5]);
                ASSERT_EQ(cntMask, matchesCounts[i * 12 + 6]);
                ASSERT_EQ(0, matchesCounts[i * 12 + 7]);
                if (bit == CASE_BIT && isalpha(c)) {
                    ASSERT_EQ(cntMask, matchesCounts[i * 12 + 8]);
                } else {
                    ASSERT_EQ(0, matchesCounts[i * 12 + 8]);
                }
            }
            else {
                ASSERT_EQ(cnt4, matchesCounts[i * 12 + 5]);
                ASSERT_EQ(cnt4, matchesCounts[i * 12 + 6]);
            }
            if (bit == CASE_BIT && isalpha(c)) {
                ASSERT_EQ(cntMask, matchesCounts[i * 12 + 9]);
                ASSERT_EQ(0, matchesCounts[i * 12 + 10]);
                ASSERT_EQ(0, matchesCounts[i * 12 + 11]);
            }
        }

        memset(&data[0], cAlt, dataSize);
        matchesCounts.clear();
        fdrStatus = fdrExec(fdr.get(), &data[0], dataSize,
                            0, countCallback, &scratch, HWLM_ALL_GROUPS);
        ASSERT_EQ(0, fdrStatus);

        for (u8 i = 0; i < 4; i++) {
            u32 mskLen = 1 << i;
            u32 cntMask = MIN(cnt4, dataSize - mskLen + 1);

            ASSERT_EQ(0, matchesCounts[i * 12 + 0]);
            ASSERT_EQ(0, matchesCounts[i * 12 + 3]);
            ASSERT_EQ(0, matchesCounts[i * 12 + 5]);
            ASSERT_EQ(0, matchesCounts[i * 12 + 6]);
            ASSERT_EQ(0, matchesCounts[i * 12 + 7]);
            ASSERT_EQ(0, matchesCounts[i * 12 + 8]);

            ASSERT_EQ(0, matchesCounts[i * 12 + 9]);
            if (bit == CASE_BIT && isalpha(c)) {
                ASSERT_EQ(mskLen > 4 ? cntMask : 0, matchesCounts[i * 12 + 1]);
                ASSERT_EQ(cntMask, matchesCounts[i * 12 + 2]);
                if(islower(c)) {
                    ASSERT_EQ(cntMask, matchesCounts[i * 12 + 4]);
                } else {
                    ASSERT_EQ(0, matchesCounts[i * 12 + 4]);
                }
                ASSERT_EQ(mskLen == 1 ? cnt4 : 0, matchesCounts[i * 12 + 10]);
                ASSERT_EQ(cntMask, matchesCounts[i * 12 + 11]);
            } else {
                ASSERT_EQ(0, matchesCounts[i * 12 + 1]);
                ASSERT_EQ(0, matchesCounts[i * 12 + 2]);
                ASSERT_EQ(0, matchesCounts[i * 12 + 4]);
                ASSERT_EQ(0, matchesCounts[i * 12 + 10]);
                ASSERT_EQ(0, matchesCounts[i * 12 + 11]);
            }
        }
        matchesCounts.clear();

        if (++c == '\0') {
            break;
        }
    }
}

TEST_P(FDRFloodp, StreamingMask) {
    const u32 hint = GetParam();
    SCOPED_TRACE(hint);
    const size_t fake_history_size = 16;
    const vector<u8> fake_history(fake_history_size, 0);
    const size_t dataSize = 1024;
    vector<u8> data(dataSize);
    vector<u8> tempdata(dataSize + fake_history_size); // headroom
    u8 c = '\0';

    struct hs_scratch scratch;
    scratch.fdr_conf = NULL;
    while (1) {
        u8 bit = 1 << (c & 0x7);
        u8 cAlt = c ^ bit;
        SCOPED_TRACE((unsigned int)c);
        memset(&data[0], c, dataSize);

        vector<hwlmLiteral> lits;

        // literals of type "aaaa" of length 4, 12, and mask of length
        // 1, 2, 4, 8, 0f type "b.......", "b......b" both case-less and case-sensitive
        string s4(4, c);
        string s4Alt(4, cAlt);

        for (int i = 0; i < 4 ; i++) {
            u32 mskLen = 1 << i;
            vector<u8> msk(mskLen, '\0');
            vector<u8> cmp(mskLen, '\0');

            cmp[0] = cAlt;
            msk[0] = '\xff';
            // msk[f0000000] cmp[c0000000] lit[aaaa]
            if (mskLen > s4.length()) {
                lits.push_back(hwlmLiteral(s4, false, false, i * 12 + 0,
                                                    HWLM_ALL_GROUPS, msk, cmp));
                lits.push_back(hwlmLiteral(s4, true, false, i * 12 + 1,
                                                    HWLM_ALL_GROUPS, msk, cmp));
            }
            // msk[f0000000] cmp[e0000000] lit[EEEE]
            if (bit == CASE_BIT && isalpha(c)) {
                lits.push_back(hwlmLiteral(s4, true, false, i * 12 + 2,
                                                    HWLM_ALL_GROUPS, msk, cmp));
            }
            // msk[E0000000] cmp[E0000000] lit[eeee]
            if ((cAlt & bit) == 0) {
                msk[0] = ~bit;
                lits.push_back(hwlmLiteral(s4, false, false, i * 12 + 3,
                                                    HWLM_ALL_GROUPS, msk, cmp));
                lits.push_back(hwlmLiteral(s4, true, false, i * 12 + 4,
                                                    HWLM_ALL_GROUPS, msk, cmp));
            }
            // msk[f0000000] cmp[a0000000] lit[aaaa]
            cmp[0] = c;
            msk[0] = '\xff';
            lits.push_back(hwlmLiteral(s4, false, false, i * 12 + 5,
                                                HWLM_ALL_GROUPS, msk, cmp));
            lits.push_back(hwlmLiteral(s4, true, false, i * 12 + 6,
                                                HWLM_ALL_GROUPS, msk, cmp));
            // msk[f0000000] cmp[a0000000] lit[cccc]
            if (mskLen > s4Alt.length()) {
                lits.push_back(hwlmLiteral(s4Alt, false, false, i * 12 + 7,
                                                    HWLM_ALL_GROUPS, msk, cmp));
                lits.push_back(hwlmLiteral(s4Alt, true, false, i * 12 + 8,
                                                    HWLM_ALL_GROUPS, msk, cmp));
            }
            if (bit == CASE_BIT && isalpha(c)) {
                // msk[f0000000] cmp[e0000000] lit[EEEE]
                lits.push_back(hwlmLiteral(s4Alt, true, false, i * 12 + 9,
                                                    HWLM_ALL_GROUPS, msk, cmp));

                // msk[f0000000] cmp[e000000E] lit[eeee]
                cmp[mskLen - 1] = cAlt;
                msk[mskLen - 1] = '\xff';
                lits.push_back(hwlmLiteral(s4, true, false, i * 12 + 10,
                                                    HWLM_ALL_GROUPS, msk, cmp));
                // msk[f0000000] cmp[E000000E] lit[eeee]
                cmp[0] = cAlt;
                lits.push_back(hwlmLiteral(s4, true, false, i * 12 + 11,
                                                    HWLM_ALL_GROUPS, msk, cmp));
            }
        }
        auto fdr = buildFDREngineHinted(lits, false, hint, get_current_target(),
                                        Grey());
        CHECK_WITH_TEDDY_OK_TO_FAIL(fdr, hint);

        hwlm_error_t fdrStatus;
        const u32 cnt4 = dataSize - 4 + 1;

        for (u32 streamChunk = 1; streamChunk <= 16; streamChunk *= 2) {
            matchesCounts.clear();
            const u8 *d = data.data();
            // reference past the end of fake history to allow headroom
            const u8 *fhist = fake_history.data() + fake_history_size;
            fdrStatus = fdrExecStreaming(fdr.get(), fhist, 0, d, streamChunk, 0,
                                         countCallback, &scratch,
                                         HWLM_ALL_GROUPS);
            ASSERT_EQ(0, fdrStatus);
            for (u32 j = streamChunk; j < dataSize; j += streamChunk) {
                if (j < 16) {
                    /* allow 16 bytes headroom on read to avoid invalid
                     * memory read during the FDR zone creation.*/
                    memset(tempdata.data(), c, dataSize + fake_history_size);
                    const u8 *tmp_d = tempdata.data() + fake_history_size;
                    fdrStatus = fdrExecStreaming(fdr.get(), tmp_d, j, tmp_d + j,
                                                 streamChunk, 0, countCallback,
                                                 &scratch, HWLM_ALL_GROUPS);
                } else {
                    fdrStatus = fdrExecStreaming(fdr.get(), d + j - 8, 8, d + j,
                                                 streamChunk, 0, countCallback,
                                                 &scratch, HWLM_ALL_GROUPS);
                }
                ASSERT_EQ(0, fdrStatus);
            }

            for (u8 i = 0; i < 4; i++) {
                u32 mskLen = 1 << i;
                u32 cntMask = MIN(cnt4, dataSize - mskLen + 1);
                ASSERT_EQ(0, matchesCounts[i * 12 + 0]);
                ASSERT_EQ(0, matchesCounts[i * 12 + 1]);
                ASSERT_EQ(0, matchesCounts[i * 12 + 2]);
                if ((cAlt & bit) == 0) {
                    ASSERT_EQ(cntMask, matchesCounts[i * 12 + 3]);
                    ASSERT_EQ(cntMask, matchesCounts[i * 12 + 4]);
                }
                if (mskLen > 4) {
                    ASSERT_EQ(cntMask, matchesCounts[i * 12 + 5]);
                    ASSERT_EQ(cntMask, matchesCounts[i * 12 + 6]);
                    ASSERT_EQ(0, matchesCounts[i * 12 + 7]);
                    if (bit == CASE_BIT && isalpha(c)) {
                        ASSERT_EQ(cntMask, matchesCounts[i * 12 + 8]);
                    } else {
                        ASSERT_EQ(0, matchesCounts[i * 12 + 8]);
                    }
                }
                else {
                    ASSERT_EQ(cnt4, matchesCounts[i * 12 + 5]);
                    ASSERT_EQ(cnt4, matchesCounts[i * 12 + 6]);
                }
                if (bit == CASE_BIT && isalpha(c)) {
                    ASSERT_EQ(cntMask, matchesCounts[i * 12 + 9]);
                    ASSERT_EQ(0, matchesCounts[i * 12 + 10]);
                    ASSERT_EQ(0, matchesCounts[i * 12 + 11]);
                }
            }
        }

        if (++c == '\0') {
            break;
        }
    }
    matchesCounts.clear();
}

INSTANTIATE_TEST_CASE_P(FDRFlood, FDRFloodp, ValuesIn(getValidFdrEngines()));

