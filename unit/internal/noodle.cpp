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
#include "hwlm/noodle_build.h"
#include "hwlm/noodle_engine.h"
#include "hwlm/hwlm.h"
#include "hwlm/hwlm_literal.h"
#include "scratch.h"
#include "util/alloc.h"
#include "util/ue2string.h"

#include <cstring>
#include <vector>
#include "gtest/gtest.h"

using std::unique_ptr;
using std::vector;
using namespace ue2;

struct hlmMatchEntry {
    size_t to;
    u32 id;
    hlmMatchEntry(size_t end, u32 identifier) :
            to(end), id(identifier) {}
};

vector<hlmMatchEntry> ctxt;

static
hwlmcb_rv_t hlmSimpleCallback(size_t to, u32 id,
                              UNUSED struct hs_scratch *scratch) {
    DEBUG_PRINTF("match @%zu = %u\n", to, id);

    ctxt.push_back(hlmMatchEntry(to, id));

    return HWLM_CONTINUE_MATCHING;
}

static
void noodleMatch(const u8 *data, size_t data_len, const char *lit_str,
                 size_t lit_len, char nocase, HWLMCallback cb) {
    u32 id = 1000;
    hwlmLiteral lit(std::string(lit_str, lit_len), nocase, id);
    auto n = noodBuildTable(lit);
    ASSERT_TRUE(n != nullptr);

    hwlm_error_t rv;
    struct hs_scratch scratch;
    rv = noodExec(n.get(), data, data_len, 0, cb, &scratch);
    ASSERT_EQ(HWLM_SUCCESS, rv);
}

TEST(Noodle, nood1) {
    const size_t data_len = 1024;
    unsigned int i, j;
    u8 data[data_len];

    memset(data, 'a', data_len);

    noodleMatch(data, data_len, "a", 1, 0, hlmSimpleCallback);
    ASSERT_EQ(1024U, ctxt.size());
    for (i = 0; i < 1024; i++) {
        ASSERT_EQ(i, ctxt[i].to);
    }

    ctxt.clear();
    noodleMatch(data, data_len, "A", 1, 0, hlmSimpleCallback);
    ASSERT_EQ(0U, ctxt.size());

    ctxt.clear();
    noodleMatch(data, data_len, "A", 1, 1, hlmSimpleCallback);
    ASSERT_EQ(1024U, ctxt.size());
    for (i = 0; i < 1024; i++) {
        ASSERT_EQ(i, ctxt[i].to);
    }

    for (j = 0; j < 16; j++) {
        ctxt.clear();
        noodleMatch(data + j, data_len - j, "A", 1, 1, hlmSimpleCallback);
        ASSERT_EQ(1024 - j, ctxt.size());
        for (i = 0; i < 1024 - j; i++) {
            ASSERT_EQ(i, ctxt[i].to);
        }

        ctxt.clear();
        noodleMatch(data, data_len - j, "A", 1, 1, hlmSimpleCallback);
        ASSERT_EQ(1024 - j, ctxt.size());
        for (i = 0; i < 1024 - j; i++) {
            ASSERT_EQ(i, ctxt[i].to);
        }
    }
    ctxt.clear();
}

TEST(Noodle, nood2) {
    const size_t data_len = 1024;
    unsigned int i, j;
    u8 data[data_len];

    memset(data, 'a', data_len);

    noodleMatch(data, data_len, "aa", 2, 0, hlmSimpleCallback);
    ASSERT_EQ(1023U, ctxt.size());
    for (i = 0; i < 1023; i++) {
        ASSERT_EQ(i + 1, ctxt[i].to);
    }

    ctxt.clear();
    noodleMatch(data, data_len, "aA", 2, 0, hlmSimpleCallback);
    ASSERT_EQ(0U, ctxt.size());

    ctxt.clear();
    noodleMatch(data, data_len, "AA", 2, 0, hlmSimpleCallback);
    ASSERT_EQ(0U, ctxt.size());

    ctxt.clear();
    noodleMatch(data, data_len, "aa", 2, 1, hlmSimpleCallback);
    ASSERT_EQ(1023U, ctxt.size());
    for (i = 0; i < 1023; i++) {
        ASSERT_EQ(i + 1, ctxt[i].to);
    }

    ctxt.clear();
    noodleMatch(data, data_len, "Aa", 2, 1, hlmSimpleCallback);
    ASSERT_EQ(1023U, ctxt.size());
    for (i = 0; i < 1023; i++) {
        ASSERT_EQ(i + 1, ctxt[i].to);
    }

    ctxt.clear();
    noodleMatch(data, data_len, "AA", 2, 1, hlmSimpleCallback);
    ASSERT_EQ(1023U, ctxt.size());
    for (i = 0; i < 1023; i++) {
        ASSERT_EQ(i + 1, ctxt[i].to);
    }

    for (j = 0; j < 16; j++) {
        ctxt.clear();
        noodleMatch(data + j, data_len - j, "Aa", 2, 1, hlmSimpleCallback);
        ASSERT_EQ(1023 - j, ctxt.size());
        for (i = 0; i < 1023 - j; i++) {
            ASSERT_EQ(i + 1, ctxt[i].to);
        }

        ctxt.clear();
        noodleMatch(data, data_len - j, "aA", 2, 1, hlmSimpleCallback);
        ASSERT_EQ(1023 - j, ctxt.size());
        for (i = 0; i < 1023 - j; i++) {
            ASSERT_EQ(i + 1, ctxt[i].to);
        }
    }
    ctxt.clear();
}

TEST(Noodle, noodLong) {
    const size_t data_len = 1024;
    unsigned int i, j;
    u8 data[data_len];

    memset(data, 'a', data_len);

    noodleMatch(data, data_len, "aaaa", 4, 0, hlmSimpleCallback);
    ASSERT_EQ(1021U, ctxt.size());
    for (i = 0; i < 1021; i++) {
        ASSERT_EQ(i + 3, ctxt[i].to);
    }

    ctxt.clear();
    noodleMatch(data, data_len, "aaAA", 4, 0, hlmSimpleCallback);
    ASSERT_EQ(0U, ctxt.size());

    ctxt.clear();
    noodleMatch(data, data_len, "aaAA", 4, 1, hlmSimpleCallback);
    ASSERT_EQ(1021U, ctxt.size());
    for (i = 0; i < 1021; i++) {
        ASSERT_EQ(i + 3, ctxt[i].to);
    }

    for (j = 0; j < 16; j++) {
        ctxt.clear();
        noodleMatch(data + j, data_len - j, "AAaa", 4, 1, hlmSimpleCallback);
        ASSERT_EQ(1021 - j, ctxt.size());
        for (i = 0; i < 1021 - j; i++) {
            ASSERT_EQ(i + 3, ctxt[i].to);
        }

        ctxt.clear();
        noodleMatch(data + j, data_len - j, "aaaA", 4, 1, hlmSimpleCallback);
        ASSERT_EQ(1021 - j, ctxt.size());
        for (i = 0; i < 1021 - j; i++) {
            ASSERT_EQ(i + 3, ctxt[i].to);
        }
    }
    ctxt.clear();
}

TEST(Noodle, noodCutoverSingle) {
    const size_t max_data_len = 128;
    u8 data[max_data_len + 15];

    memset(data, 'a', max_data_len + 15);

    for (u32 align = 0; align < 16; align++) {
        for (u32 len = 0; len < max_data_len; len++) {
            ctxt.clear();
            noodleMatch(data + align, len, "a", 1, 0, hlmSimpleCallback);
            EXPECT_EQ(len, ctxt.size());
            for (u32 i = 0; i < ctxt.size(); i++) {
                ASSERT_EQ(i, ctxt[i].to);
            }
        }
    }
    ctxt.clear();
}

TEST(Noodle, noodCutoverDouble) {
    const size_t max_data_len = 128;
    u8 data[max_data_len + 15];

    memset(data, 'a', max_data_len + 15);

    for (u32 align = 0; align < 16; align++) {
        for (u32 len = 0; len < max_data_len; len++) {
            ctxt.clear();
            noodleMatch(data + align, len, "aa", 2, 0, hlmSimpleCallback);
            EXPECT_EQ(len ? len - 1 : 0U, ctxt.size());
            for (u32 i = 0; i < ctxt.size(); i++) {
                ASSERT_EQ(i + 1, ctxt[i].to);
            }
        }
    }
    ctxt.clear();
}

