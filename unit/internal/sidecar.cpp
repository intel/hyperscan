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

#include "config.h"

#include "ue2common.h"
#include "sidecar/sidecar.h"
#include "sidecar/sidecar_compile.h"
#include "sidecar/sidecar_internal.h"
#include "util/alloc.h"
#include "util/charreach.h"

#include <tuple>
#include <vector>
#include "gtest/gtest.h"

using namespace testing;
using namespace ue2;
using std::vector;
using std::set;
using std::tie;
using std::tuple;

namespace {

void ns_cb(UNUSED u64a offset, u32 id, void *ctxt) {
    u32 *seen = (u32 *)ctxt;
    *seen |= 1U << id;
}

void set_cb(UNUSED u64a offset, u32 id, void *ctxt) {
    set<u32> *seen = (set<u32> *)ctxt;
    seen->insert(id);
}

TEST(Sidecar, ns1) {
    const size_t data_len = 1024;
    u8 data[data_len];

    CharReach c_1;
    c_1.set('f');
    vector<CharReach> charclasses;
    charclasses.push_back(c_1);
    auto ns = sidecarCompile(charclasses);

    ASSERT_TRUE(ns != nullptr);
    ASSERT_LT(0U, sidecarSize(ns.get()));

    struct sidecar_enabled *enabled
        = (struct sidecar_enabled *)aligned_zmalloc(sidecarEnabledSize(ns.get()));
    ASSERT_TRUE(enabled);
    sidecarEnabledInit(ns.get(), enabled);
    struct sidecar_scratch *scratch
        = (struct sidecar_scratch *)aligned_zmalloc(sidecarScratchSize(ns.get()));

    for (u32 i = 0; i < 256; i++) {
        SCOPED_TRACE(i);
        u32 seen = 0;
        memset(data, i, data_len);
        sidecarExec(ns.get(), data, data_len, enabled, scratch, 0, ns_cb, &seen);
        ASSERT_EQ(0U, seen);
    }

    sidecarEnabledAdd(ns.get(), enabled, 0);

    for (u32 i = 0; i < 256; i++) {
        SCOPED_TRACE(i);
        u32 seen = 0;
        memset(data, i, data_len);
        sidecarExec(ns.get(), data, data_len, enabled, scratch, 0, ns_cb, &seen);
        if (i == 'f') {
            ASSERT_EQ(1U, seen);
        } else {
            ASSERT_EQ(0U, seen);
        }
    }

    aligned_free(enabled);
    aligned_free(scratch);
}

const char* sidecarStrings[] = {
    "f",
    "a",
    "A",
    "ab",
    "\r\n", // an old favourite
    "\t\r\n",
    " \r\n",
    "xyz",
    "z0y1",
    "01234567", // 8 elements
    "!@#$%^&*()", // 10 elements
    "qwertyuiopasdfgh", // 16 elements
    "qwertyuiopasdfghj", // 17 elements
    "qwertyuiopasdfghjklzxcvb", // 24 elements
    "qwertyuiopasdfghjklzxcvbnm012345", // 32 elements
    "qwertyuiopasdfghjklzxcvbnm0123456" // 33 elements
};

const u32 sidecarModels[] = {
    SIDECAR_8,
    SIDECAR_32,
    SIDECAR_64,
    SIDECAR_128,
    SIDECAR_256,
    SIDECAR_N,
    SIDECAR_S
};

// Number of elements we can handle in each model
const u32 sidecarSizes[] = {
    8,
    32,
    64,
    128,
    256,
    1,
    8
};

// Parameterized test case for string of single-byte classes
class SidecarTest : public TestWithParam<tuple<u32, const char *>> {
protected:
    virtual void SetUp() {
        tie(model, chars) = GetParam();
        size_t num = strlen(chars);
        charclasses.resize(num);

        for (size_t i = 0; i < num; i++) {
            charclasses[i].set(chars[i]);
        }
    }

    virtual bool fitsModel() {
        for (size_t i = 0; i < ARRAY_LENGTH(sidecarModels); i++) {
            if (sidecarModels[i] == model) {
                return charclasses.size() <= sidecarSizes[i];
            }
        }
        return false;
    }

    u32 model;
    const char *chars;
    vector<CharReach> charclasses;
};

TEST_P(SidecarTest, Individual) {
    SCOPED_TRACE(chars);

    // Skip this test if the model is too small
    if (!fitsModel()) {
        return;
    }

    auto ns = sidecarCompile(charclasses, model);
    if (!ns && model == SIDECAR_S) { /* shufti is fussi */
        return;
    }
    ASSERT_TRUE(ns != nullptr);
    ASSERT_LT(0U, sidecarSize(ns.get()));

    struct sidecar_enabled *enabled
        = (struct sidecar_enabled *)aligned_zmalloc(sidecarEnabledSize(ns.get()));
    ASSERT_TRUE(enabled);
    sidecarEnabledInit(ns.get(), enabled);
    struct sidecar_enabled *local_enabled
        = (struct sidecar_enabled *)aligned_zmalloc(sidecarEnabledSize(ns.get()));
    struct sidecar_scratch *scratch
        = (struct sidecar_scratch *)aligned_zmalloc(sidecarScratchSize(ns.get()));

    const size_t data_len = 1024;
    u8 data[data_len];

    // with nothing enabled, nothing should fire
    for (u32 i = 0; i < 256; i++) {
        SCOPED_TRACE(i);
        memset(data, i, data_len);
        set<u32> seen;
        sidecarExec(ns.get(), data, data_len, enabled, scratch, 0, set_cb, &seen);
        ASSERT_TRUE(seen.empty());
    }

    // test that every char class fires when enabled separately
    for (u32 j = 0; j < charclasses.size(); j++) {
        u32 c = chars[j];
        SCOPED_TRACE(c);

        // build a "compile time" enabled structure and add class j to it.
        sidecarEnabledInit(ns.get(), local_enabled);
        sidecarEnabledAdd(ns.get(), local_enabled, j);

        // union class j into our runtime enabled structure.
        sidecarEnabledUnion(ns.get(), enabled, local_enabled);

        for (u32 i = 0; i < 256; i++) {
            SCOPED_TRACE(i);
            memset(data, i, data_len);
            set<u32> seen;
            sidecarExec(ns.get(), data, data_len, enabled, scratch, 0, set_cb, &seen);
            if (i == c) {
                ASSERT_EQ(1U, seen.size());
                ASSERT_EQ(j, *seen.begin());
            } else {
                ASSERT_TRUE(seen.empty());
            }
        }
    }

    aligned_free(local_enabled);
    aligned_free(enabled);
    aligned_free(scratch);
}

TEST_P(SidecarTest, Together) {
    SCOPED_TRACE(chars);

    // Skip this test if the model is too small
    if (!fitsModel()) {
        return;
    }

    auto ns = sidecarCompile(charclasses, model);
    if (!ns && model == SIDECAR_S) { /* shufti is fussi */
        return;
    }
    ASSERT_TRUE(ns != nullptr);
    ASSERT_LT(0U, sidecarSize(ns.get()));

    struct sidecar_enabled *enabled
        = (struct sidecar_enabled *)aligned_zmalloc(sidecarEnabledSize(ns.get()));
    ASSERT_TRUE(enabled);
    struct sidecar_enabled *local_enabled
        = (struct sidecar_enabled *)aligned_zmalloc(sidecarEnabledSize(ns.get()));
    struct sidecar_scratch *scratch
        = (struct sidecar_scratch *)aligned_zmalloc(sidecarScratchSize(ns.get()));

    const size_t data_len = 1024;
    u8 data[data_len];

    // with nothing enabled, nothing should fire
    for (u32 i = 0; i < 256; i++) {
        SCOPED_TRACE(i);
        memset(data, i, data_len);
        set<u32> seen;
        sidecarExec(ns.get(), data, data_len, enabled, scratch, 0, set_cb, &seen);
        ASSERT_TRUE(seen.empty());
    }

    // test that every char class fires
    for (u32 j = 0; j < charclasses.size(); j++) {
        // enable the whole lot
        sidecarEnabledInit(ns.get(), enabled);
        for (u32 i = 0; i < charclasses.size(); i++) {
            // build a "compile time" enabled structure and add class j to it.
            sidecarEnabledInit(ns.get(), local_enabled);
            sidecarEnabledAdd(ns.get(), local_enabled, i);

            // union class j into our runtime enabled structure.
            sidecarEnabledUnion(ns.get(), enabled, local_enabled);
        }

        u32 c = chars[j];
        SCOPED_TRACE(c);

        for (u32 i = 0; i < 256; i++) {
            SCOPED_TRACE(i);
            memset(data, i, data_len);
            set<u32> seen;
            sidecarExec(ns.get(), data, data_len, enabled, scratch, 0, set_cb, &seen);
            if (i == c) {
                // seen should contain only `c'
                ASSERT_EQ(1U, seen.size());
                ASSERT_FALSE(seen.end() == seen.find(j));
            } else {
                // seen should not contain `c', and either zero or one char can
                // have matched
                ASSERT_GT(2U, seen.size());
                ASSERT_TRUE(seen.end() == seen.find(j));
            }
        }
    }

    aligned_free(local_enabled);
    aligned_free(enabled);
    aligned_free(scratch);
}

INSTANTIATE_TEST_CASE_P(Sidecar, SidecarTest,
    Combine(ValuesIn(sidecarModels),
            ValuesIn(sidecarStrings)));

}
