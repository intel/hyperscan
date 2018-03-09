/*
 * Copyright (c) 2018, Intel Corporation
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

#include "gtest/gtest.h"
#include "chimera/ch.h"

using namespace testing;

class HybridCompile : public TestWithParam<const char *> {
    // empty
};

TEST_P(HybridCompile, BadPattern) {
    ch_error_t err;
    ch_compile_error_t *compile_err = nullptr;
    const char *pattern = GetParam();
    ch_database_t *db = nullptr;

    err = ch_compile_multi(&pattern, nullptr, nullptr, 1, 0, nullptr, &db,
                           &compile_err);
    ASSERT_NE(CH_SUCCESS, err) << "Compile should have failed for expr: "
        << pattern;
    ASSERT_TRUE(db == nullptr);
    ASSERT_TRUE(compile_err != nullptr);

    ch_free_compile_error(compile_err);
}

static
const char * BAD_PATTERNS[] = {
    // unmatched parens
    "(foo",
    "foo)",
    "((foo)",
    "(foo))",
    // nothing to repeat
    "a+++",
    "a+?+",
    "a???",
    "a??+",
    "?qa",
    "*abc",
    "+abc",
    // repeating boundaries is not allowed (UE-1007)
    "^?0",
    "^*0",
    "^+0",
    "^{1,3}0",
    "0$?",
    "0$*",
    "0$+",
    "0${1,3}",
    // char classes
    "[]",
    "[]foobar",
    "[`-\\80",
    // bad named classes
    "[[:foo:]]",
    "[[:1234:]]",
    "[[:f\\oo:]]",
    "[[: :]]",
    "[[:...:]]",
    "[[:l\\ower:]]",
    "[[:abc\\:]]",
    "[abc[:x\\]pqr:]]",
    "[[:a\\dz:]]",
    "foobar\\", // trailing unescaped backslash
};

INSTANTIATE_TEST_CASE_P(Compile, HybridCompile, ValuesIn(BAD_PATTERNS));
