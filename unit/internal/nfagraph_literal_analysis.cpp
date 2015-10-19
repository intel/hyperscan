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

#include <set>
#include <vector>

#include "gtest/gtest.h"
#include "nfagraph/ng_literal_analysis.h"

using namespace std;
using namespace boost;
using namespace ue2;

struct LiteralSetCompressParams {
    set<ue2_literal> input;
    set<ue2_literal> output;
};

void PrintTo(const LiteralSetCompressParams &p, ::std::ostream *os) {
    *os << "input: ";
    set<ue2_literal>::const_iterator it, ite;
    for (it = p.input.begin(), ite = p.input.end(); it != ite; ++it) {
        *os << it->get_string() << " ";
    }
    *os << "output: ";
    for (it = p.output.begin(), ite = p.output.end(); it != ite; ++it) {
        *os << it->get_string() << " ";
    }
}

static
vector<LiteralSetCompressParams> paramFactory() {
    vector<LiteralSetCompressParams> p;

    p.push_back(LiteralSetCompressParams());
    p.back().input.insert(ue2_literal("foobar", false));
    p.back().input.insert(ue2_literal("bar", false));
    p.back().output.insert(ue2_literal("bar", false));

    p.push_back(LiteralSetCompressParams());
    p.back().input.insert(ue2_literal("foobar", false));
    p.back().input.insert(ue2_literal("fooobar", false));
    p.back().input.insert(ue2_literal("foooobar", false));
    p.back().input.insert(ue2_literal("fooooobar", false));
    p.back().output.insert(ue2_literal("oobar", false));

    return p;
}

class LiteralSetCompressTest : public testing::TestWithParam<LiteralSetCompressParams> {
protected:
    LiteralSetCompressTest() {}
};

TEST_P(LiteralSetCompressTest, Run) {
    const LiteralSetCompressParams &p = GetParam();

    set<ue2_literal> lits(p.input.begin(), p.input.end());
    compressAndScore(lits);
    ASSERT_EQ(p.output, lits);
}

INSTANTIATE_TEST_CASE_P(NFAGraph, LiteralSetCompressTest, testing::ValuesIn(paramFactory()));
