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

/**
 * Unit tests for checking the calc comp code in nfagraph/ng_calc_components.cpp
 */

#include "config.h"
#include "gtest/gtest.h"
#include "nfagraph_common.h"
#include "nfagraph/ng.h"
#include "nfagraph/ng_calc_components.h"

using namespace std;
using namespace ue2;

TEST(NFAGraph, CalcComp1) {
    auto graph = constructGraph("abc|def|ghi", 0);
    ASSERT_TRUE(graph != nullptr);

    Grey grey;
    grey.calcComponents = true;
    auto comps = calcComponents(std::move(graph), grey);
    ASSERT_EQ(3, comps.size());
}

TEST(NFAGraph, CalcComp2) {
    auto graph = constructGraph("a|b|c|d|e|f|g|h|i", 0);
    ASSERT_TRUE(graph != nullptr);

    Grey grey;
    grey.calcComponents = true;
    auto comps = calcComponents(std::move(graph), grey);

    // We should be identifying this as a trivial case and not splitting it.
    ASSERT_EQ(1, comps.size());
}

TEST(NFAGraph, RecalcComp1) {
    deque<unique_ptr<NGHolder>> comps;
    comps.push_back(constructGraph("abc|def|ghi", 0));
    ASSERT_TRUE(comps.back() != nullptr);

    Grey grey;
    grey.calcComponents = true;
    recalcComponents(comps, grey);

    ASSERT_EQ(3, comps.size());
}
