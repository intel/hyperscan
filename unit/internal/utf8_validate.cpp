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

#include "parser/utf8_validate.h"

#include "ue2common.h"
#include "util/string_util.h"

#include "gtest/gtest.h"

using namespace testing;
using namespace ue2;

struct ValidUtf8TestInfo {
    std::string str;
    bool is_valid;
};

// Helper for gtest.
static
void PrintTo(const ValidUtf8TestInfo &t, ::std::ostream *os) {
    *os << "(\"" << printable(t.str) << "\", " << t.is_valid << ")";
}

static ValidUtf8TestInfo valid_utf8_tests[] = {
    // Trivial ASCII.
    {"foobar", true},
    {"0", true},
    {"\x7e", true},
    {"hatstand\tteakettle\tbadgerbrush\n", true},

    // Some valid UTF-8: Movie titles!
    {"À bout de souffle", true},
    {"拳銃は俺のパスポート", true},
    {"大醉俠", true},
    {"龙门客栈", true},
    {"공동경비구역", true},
    {"জলসাঘর", true},

    // Invalid one-byte caseS.
    {"\x7f", false},

    // These bytes should never appear in a UTF-8 stream.
    {"\xc0", false},
    {"\xc1", false},
    {"\xf5", false},
    {"\xf6", false},
    {"\xf7", false},
    {"\xf8", false},
    {"\xf9", false},
    {"\xfa", false},
    {"\xfc", false},
    {"\xfd", false},
    {"\xfe", false},
    {"\xff", false},
    {"\xff", false},

    // Examples from RFC-3629 section 7.
    {"\x41\xe2\x89\xa2\xce\x91\x2e", true},
    {"\xed\x95\x9c\xea\xb5\xad\xec\x96\xb4", true},
    {"\xe6\x97\xa5\xe6\x9c\xac\xe8\xaa\x9e", true},
    {"\xef\xbb\xbf\xf0\xa3\x8e\xb4", true},

    // Examples from RFC-3629 section 10. (security concerns)
    {"/../", true},
    {"\x2f\xc0\xae\x2e\x2f", false}, // overlong

    // Overlong encodings
    {"\xc0\xc1", false}, // 'a' as two bytes
    {"\xe0\x80\xc1", false}, // 'a' as three bytes
    {"\xf0\x80\x80\xc1", false}, // 'a' as four bytes

    /* invalid continuing bytes */
    {"\xd1\xf1", false},
    {"\xef\xbf\xf1", false},
    {"\xef\xf1\xbf", false},
    {"\xf1\xf1\xf1\xf1", false},
    {"\xf1\xbf\xbf\xf1", false},
    {"\xf1\xbf\xf1\xbf", false},
    {"\xf1\xf1\xbf\xbf", false},

    // UTF-16 surrogates
    {"\xed\xa0\x80", false}, // U+D800
    {"\xed\xb2\x80", false}, // U+DC80
};

class ValidUtf8Test : public TestWithParam<ValidUtf8TestInfo> {};

INSTANTIATE_TEST_CASE_P(ValidUtf8, ValidUtf8Test, ValuesIn(valid_utf8_tests));

TEST_P(ValidUtf8Test, check) {
    const auto &info = GetParam();
    SCOPED_TRACE(testing::Message() << "String is: " << printable(info.str));
    ASSERT_EQ(info.is_valid, isValidUtf8(info.str.c_str(), info.str.size()));
}
