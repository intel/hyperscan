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

#include <algorithm>
#include <array>
#include <iostream>
#include <vector>

#include "gtest/gtest.h"
#include "hs.h"
#include "config.h"
#include "test_util.h"

using namespace std;

TEST(LogicalCombination, SingleComb1) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    CallBackContext c;
    string data = "abcdefxxfoobarrrghabcxdefxteakettleeeeexxxxijklmxxdef";
    const char *expr[] = {"abc", "def", "foobar.*gh", "teakettle{4,10}",
                          "ijkl[mMn]", "(101 & 102 & 103) | (104 & !105)"};
    unsigned flags[] = {0, 0, 0, 0, 0, HS_FLAG_COMBINATION};
    unsigned ids[] = {101, 102, 103, 104, 105, 1001};
    hs_error_t err = hs_compile_multi(expr, flags, ids, 6, HS_MODE_NOSTREAM,
                                      nullptr, &db, &compile_err);

    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    c.halt = 0;
    err = hs_scan(db, data.c_str(), data.size(), 0, scratch, record_cb,
                  (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(16U, c.matches.size());
    ASSERT_EQ(MatchRecord(3, 101), c.matches[0]);
    ASSERT_EQ(MatchRecord(6, 102), c.matches[1]);
    ASSERT_EQ(MatchRecord(18, 103), c.matches[2]);
    ASSERT_EQ(MatchRecord(18, 1001), c.matches[3]);
    ASSERT_EQ(MatchRecord(21, 101), c.matches[4]);
    ASSERT_EQ(MatchRecord(21, 1001), c.matches[5]);
    ASSERT_EQ(MatchRecord(25, 102), c.matches[6]);
    ASSERT_EQ(MatchRecord(25, 1001), c.matches[7]);
    ASSERT_EQ(MatchRecord(38, 104), c.matches[8]);
    ASSERT_EQ(MatchRecord(38, 1001), c.matches[9]);
    ASSERT_EQ(MatchRecord(39, 104), c.matches[10]);
    ASSERT_EQ(MatchRecord(39, 1001), c.matches[11]);
    ASSERT_EQ(MatchRecord(48, 105), c.matches[12]);
    ASSERT_EQ(MatchRecord(48, 1001), c.matches[13]);
    ASSERT_EQ(MatchRecord(53, 102), c.matches[14]);
    ASSERT_EQ(MatchRecord(53, 1001), c.matches[15]);

    hs_free_database(db);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
}

TEST(LogicalCombination, SingleCombQuietSub1) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    CallBackContext c;
    string data = "abcdefxxfoobarrrghabcxdefxteakettleeeeexxxxijklmxxdef";
    const char *expr[] = {"abc", "def", "foobar.*gh", "teakettle{4,10}",
                          "ijkl[mMn]", "(101 & 102 & 103) | (104 & !105)"};
    unsigned flags[] = {HS_FLAG_QUIET, HS_FLAG_QUIET, HS_FLAG_QUIET,
                        HS_FLAG_QUIET, 0, HS_FLAG_COMBINATION};
    unsigned ids[] = {101, 102, 103, 104, 105, 1001};
    hs_error_t err = hs_compile_multi(expr, flags, ids, 6, HS_MODE_NOSTREAM,
                                      nullptr, &db, &compile_err);

    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    c.halt = 0;
    err = hs_scan(db, data.c_str(), data.size(), 0, scratch, record_cb,
                  (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(8U, c.matches.size());
    ASSERT_EQ(MatchRecord(18, 1001), c.matches[0]);
    ASSERT_EQ(MatchRecord(21, 1001), c.matches[1]);
    ASSERT_EQ(MatchRecord(25, 1001), c.matches[2]);
    ASSERT_EQ(MatchRecord(38, 1001), c.matches[3]);
    ASSERT_EQ(MatchRecord(39, 1001), c.matches[4]);
    ASSERT_EQ(MatchRecord(48, 105), c.matches[5]);
    ASSERT_EQ(MatchRecord(48, 1001), c.matches[6]);
    ASSERT_EQ(MatchRecord(53, 1001), c.matches[7]);

    hs_free_database(db);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
}

TEST(LogicalCombination, MultiCombQuietSub1) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    CallBackContext c;
    string data = "abcdefxxfoobarrrghabcxdefxteakettleeeeexxxxijklmxxdef";
    const char *expr[] = {"abc", "def", "foobar.*gh", "teakettle{4,10}",
                          "ijkl[mMn]", "(101 & 102 & 103) | (104 & !105)",
                          "!101 & 102", "!(!101 | 102)", "101 & !102"};
    unsigned flags[] = {HS_FLAG_QUIET, HS_FLAG_QUIET, HS_FLAG_QUIET,
                        HS_FLAG_QUIET, 0, HS_FLAG_COMBINATION,
                        HS_FLAG_COMBINATION, HS_FLAG_COMBINATION,
                        HS_FLAG_COMBINATION};
    unsigned ids[] = {101, 102, 103, 104, 105, 1001, 1002, 1003, 1004};
    hs_error_t err = hs_compile_multi(expr, flags, ids, 9, HS_MODE_NOSTREAM,
                                      nullptr, &db, &compile_err);

    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    c.halt = 0;
    err = hs_scan(db, data.c_str(), data.size(), 0, scratch, record_cb,
                  (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(10U, c.matches.size());
    ASSERT_EQ(MatchRecord(3, 1003), c.matches[0]);
    ASSERT_EQ(MatchRecord(3, 1004), c.matches[1]);
    ASSERT_EQ(MatchRecord(18, 1001), c.matches[2]);
    ASSERT_EQ(MatchRecord(21, 1001), c.matches[3]);
    ASSERT_EQ(MatchRecord(25, 1001), c.matches[4]);
    ASSERT_EQ(MatchRecord(38, 1001), c.matches[5]);
    ASSERT_EQ(MatchRecord(39, 1001), c.matches[6]);
    ASSERT_EQ(MatchRecord(48, 105), c.matches[7]);
    ASSERT_EQ(MatchRecord(48, 1001), c.matches[8]);
    ASSERT_EQ(MatchRecord(53, 1001), c.matches[9]);

    hs_free_database(db);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
}

TEST(LogicalCombination, MultiHighlanderCombQuietSub1) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    CallBackContext c;
    string data = "abcdefxxfoobarrrghabcxdefxteakettleeeeexxxxijklmxxdef";
    const char *expr[] = {"abc", "def", "foobar.*gh", "teakettle{4,10}",
                          "ijkl[mMn]", "(101 & 102 & 103) | (104 & !105)",
                          "!101 & 102", "!(!101 | 102)", "101 & !102"};
    unsigned flags[] = {HS_FLAG_QUIET, HS_FLAG_QUIET, HS_FLAG_QUIET,
                        HS_FLAG_QUIET, 0,
                        HS_FLAG_COMBINATION | HS_FLAG_SINGLEMATCH,
                        HS_FLAG_COMBINATION,
                        HS_FLAG_COMBINATION | HS_FLAG_SINGLEMATCH,
                        HS_FLAG_COMBINATION | HS_FLAG_SINGLEMATCH};
    unsigned ids[] = {101, 102, 103, 104, 105, 1001, 1002, 1003, 1004};
    hs_error_t err = hs_compile_multi(expr, flags, ids, 9, HS_MODE_NOSTREAM,
                                      nullptr, &db, &compile_err);

    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    c.halt = 0;
    err = hs_scan(db, data.c_str(), data.size(), 0, scratch, record_cb,
                  (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(4U, c.matches.size());
    ASSERT_EQ(MatchRecord(3, 1003), c.matches[0]);
    ASSERT_EQ(MatchRecord(3, 1004), c.matches[1]);
    ASSERT_EQ(MatchRecord(18, 1001), c.matches[2]);
    ASSERT_EQ(MatchRecord(48, 105), c.matches[3]);

    hs_free_database(db);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
}

TEST(LogicalCombination, MultiQuietCombQuietSub1) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    CallBackContext c;
    string data = "abcdefxxfoobarrrghabcxdefxteakettleeeeexxxxijklmxxdef";
    const char *expr[] = {"abc", "def", "foobar.*gh", "teakettle{4,10}",
                          "ijkl[mMn]", "(101 & 102 & 103) | (104 & !105)",
                          "!101 & 102", "!(!101 | 102)", "101 & !102"};
    unsigned flags[] = {HS_FLAG_QUIET, HS_FLAG_QUIET, HS_FLAG_QUIET,
                        HS_FLAG_QUIET, 0, HS_FLAG_COMBINATION | HS_FLAG_QUIET,
                        HS_FLAG_COMBINATION, HS_FLAG_COMBINATION,
                        HS_FLAG_COMBINATION | HS_FLAG_QUIET};
    unsigned ids[] = {101, 102, 103, 104, 105, 1001, 1002, 1003, 1004};
    hs_error_t err = hs_compile_multi(expr, flags, ids, 9, HS_MODE_NOSTREAM,
                                      nullptr, &db, &compile_err);

    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    c.halt = 0;
    err = hs_scan(db, data.c_str(), data.size(), 0, scratch, record_cb,
                  (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(2U, c.matches.size());
    ASSERT_EQ(MatchRecord(3, 1003), c.matches[0]);
    ASSERT_EQ(MatchRecord(48, 105), c.matches[1]);

    hs_free_database(db);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
}

TEST(LogicalCombination, SingleComb2) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    CallBackContext c;
    string data = "abbdefxxfoobarrrghabcxdefxteakettleeeeexxxxijklmxxdef";
    const char *expr[] = {"abc", "def", "foobar.*gh", "teakettle{4,10}",
                          "ijkl[mMn]", "(201 | 202 & 203) & (!204 | 205)"};
    unsigned flags[] = {0, 0, 0, 0, 0, HS_FLAG_COMBINATION};
    unsigned ids[] = {201, 202, 203, 204, 205, 1002};
    hs_error_t err = hs_compile_multi(expr, flags, ids, 6, HS_MODE_NOSTREAM,
                                      nullptr, &db, &compile_err);

    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    c.halt = 0;
    err = hs_scan(db, data.c_str(), data.size(), 0, scratch, record_cb,
                  (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(13U, c.matches.size());
    ASSERT_EQ(MatchRecord(6, 202), c.matches[0]);
    ASSERT_EQ(MatchRecord(18, 203), c.matches[1]);
    ASSERT_EQ(MatchRecord(18, 1002), c.matches[2]);
    ASSERT_EQ(MatchRecord(21, 201), c.matches[3]);
    ASSERT_EQ(MatchRecord(21, 1002), c.matches[4]);
    ASSERT_EQ(MatchRecord(25, 202), c.matches[5]);
    ASSERT_EQ(MatchRecord(25, 1002), c.matches[6]);
    ASSERT_EQ(MatchRecord(38, 204), c.matches[7]);
    ASSERT_EQ(MatchRecord(39, 204), c.matches[8]);
    ASSERT_EQ(MatchRecord(48, 205), c.matches[9]);
    ASSERT_EQ(MatchRecord(48, 1002), c.matches[10]);
    ASSERT_EQ(MatchRecord(53, 202), c.matches[11]);
    ASSERT_EQ(MatchRecord(53, 1002), c.matches[12]);

    hs_free_database(db);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
}

TEST(LogicalCombination, SingleCombQuietSub2) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    CallBackContext c;
    string data = "abbdefxxfoobarrrghabcxdefxteakettleeeeexxxxijklmxxdef";
    const char *expr[] = {"abc", "def", "foobar.*gh", "teakettle{4,10}",
                          "ijkl[mMn]", "(201 | 202 & 203) & (!204 | 205)"};
    unsigned flags[] = {0, HS_FLAG_QUIET, HS_FLAG_QUIET, 0, HS_FLAG_QUIET,
                        HS_FLAG_COMBINATION};
    unsigned ids[] = {201, 202, 203, 204, 205, 1002};
    hs_error_t err = hs_compile_multi(expr, flags, ids, 6, HS_MODE_NOSTREAM,
                                      nullptr, &db, &compile_err);

    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    c.halt = 0;
    err = hs_scan(db, data.c_str(), data.size(), 0, scratch, record_cb,
                  (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(8U, c.matches.size());
    ASSERT_EQ(MatchRecord(18, 1002), c.matches[0]);
    ASSERT_EQ(MatchRecord(21, 201), c.matches[1]);
    ASSERT_EQ(MatchRecord(21, 1002), c.matches[2]);
    ASSERT_EQ(MatchRecord(25, 1002), c.matches[3]);
    ASSERT_EQ(MatchRecord(38, 204), c.matches[4]);
    ASSERT_EQ(MatchRecord(39, 204), c.matches[5]);
    ASSERT_EQ(MatchRecord(48, 1002), c.matches[6]);
    ASSERT_EQ(MatchRecord(53, 1002), c.matches[7]);

    hs_free_database(db);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
}

TEST(LogicalCombination, SingleComb3) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    CallBackContext c;
    string data = "abcijklndefxxfoobarrrghabcxdefxteakettleeeeexxxxijklnxxdef";
    const char *expr[] = {"abc", "def", "foobar.*gh", "teakettle{4,10}",
                          "ijkl[mMn]", "((301 | 302) & 303) & (304 | 305)"};
    unsigned flags[] = {0, 0, 0, 0, 0, HS_FLAG_COMBINATION};
    unsigned ids[] = {301, 302, 303, 304, 305, 1003};
    hs_error_t err = hs_compile_multi(expr, flags, ids, 6, HS_MODE_NOSTREAM,
                                      nullptr, &db, &compile_err);

    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    c.halt = 0;
    err = hs_scan(db, data.c_str(), data.size(), 0, scratch, record_cb,
                  (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(17U, c.matches.size());
    ASSERT_EQ(MatchRecord(3, 301), c.matches[0]);
    ASSERT_EQ(MatchRecord(8, 305), c.matches[1]);
    ASSERT_EQ(MatchRecord(11, 302), c.matches[2]);
    ASSERT_EQ(MatchRecord(23, 303), c.matches[3]);
    ASSERT_EQ(MatchRecord(23, 1003), c.matches[4]);
    ASSERT_EQ(MatchRecord(26, 301), c.matches[5]);
    ASSERT_EQ(MatchRecord(26, 1003), c.matches[6]);
    ASSERT_EQ(MatchRecord(30, 302), c.matches[7]);
    ASSERT_EQ(MatchRecord(30, 1003), c.matches[8]);
    ASSERT_EQ(MatchRecord(43, 304), c.matches[9]);
    ASSERT_EQ(MatchRecord(43, 1003), c.matches[10]);
    ASSERT_EQ(MatchRecord(44, 304), c.matches[11]);
    ASSERT_EQ(MatchRecord(44, 1003), c.matches[12]);
    ASSERT_EQ(MatchRecord(53, 305), c.matches[13]);
    ASSERT_EQ(MatchRecord(53, 1003), c.matches[14]);
    ASSERT_EQ(MatchRecord(58, 302), c.matches[15]);
    ASSERT_EQ(MatchRecord(58, 1003), c.matches[16]);

    hs_free_database(db);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
}

TEST(LogicalCombination, SingleCombQuietSub3) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    CallBackContext c;
    string data = "abcijklndefxxfoobarrrghabcxdefxteakettleeeeexxxxijklnxxdef";
    const char *expr[] = {"abc", "def", "foobar.*gh", "teakettle{4,10}",
                          "ijkl[mMn]", "((301 | 302) & 303) & (304 | 305)"};
    unsigned flags[] = {HS_FLAG_QUIET, HS_FLAG_QUIET, 0, HS_FLAG_QUIET,
                        HS_FLAG_QUIET, HS_FLAG_COMBINATION};
    unsigned ids[] = {301, 302, 303, 304, 305, 1003};
    hs_error_t err = hs_compile_multi(expr, flags, ids, 6, HS_MODE_NOSTREAM,
                                      nullptr, &db, &compile_err);

    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    c.halt = 0;
    err = hs_scan(db, data.c_str(), data.size(), 0, scratch, record_cb,
                  (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(8U, c.matches.size());
    ASSERT_EQ(MatchRecord(23, 303), c.matches[0]);
    ASSERT_EQ(MatchRecord(23, 1003), c.matches[1]);
    ASSERT_EQ(MatchRecord(26, 1003), c.matches[2]);
    ASSERT_EQ(MatchRecord(30, 1003), c.matches[3]);
    ASSERT_EQ(MatchRecord(43, 1003), c.matches[4]);
    ASSERT_EQ(MatchRecord(44, 1003), c.matches[5]);
    ASSERT_EQ(MatchRecord(53, 1003), c.matches[6]);
    ASSERT_EQ(MatchRecord(58, 1003), c.matches[7]);

    hs_free_database(db);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
}

TEST(LogicalCombination, MultiCombDupSub4) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    CallBackContext c;
    string data = "abbdefxxfoobarrrghabcxdefxteakettleeeeexxxxijklmxxdef";
    const char *expr[] = {"abc", "def", "foobar.*gh", "teakettle{4,10}",
                          "ijkl[mMn]", "(201 & 202 & 203) | (204 & !205)",
                          "(201 | 202 & 203) & (!204 | 205)",
                          "((201 | 202) & 203) & (204 | 205)"};
    unsigned flags[] = {0, 0, 0, 0, 0, HS_FLAG_COMBINATION,
                        HS_FLAG_COMBINATION, HS_FLAG_COMBINATION};
    unsigned ids[] = {201, 202, 203, 204, 205, 1001, 1002, 1003};
    hs_error_t err = hs_compile_multi(expr, flags, ids, 8, HS_MODE_NOSTREAM,
                                      nullptr, &db, &compile_err);

    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    c.halt = 0;
    err = hs_scan(db, data.c_str(), data.size(), 0, scratch, record_cb,
                  (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(23U, c.matches.size());
    ASSERT_EQ(MatchRecord(6, 202), c.matches[0]);
    ASSERT_EQ(MatchRecord(18, 203), c.matches[1]);
    ASSERT_EQ(MatchRecord(18, 1002), c.matches[2]);
    ASSERT_EQ(MatchRecord(21, 201), c.matches[3]);
    ASSERT_EQ(MatchRecord(21, 1001), c.matches[4]);
    ASSERT_EQ(MatchRecord(21, 1002), c.matches[5]);
    ASSERT_EQ(MatchRecord(25, 202), c.matches[6]);
    ASSERT_EQ(MatchRecord(25, 1001), c.matches[7]);
    ASSERT_EQ(MatchRecord(25, 1002), c.matches[8]);
    ASSERT_EQ(MatchRecord(38, 204), c.matches[9]);
    ASSERT_EQ(MatchRecord(38, 1001), c.matches[10]);
    ASSERT_EQ(MatchRecord(38, 1003), c.matches[11]);
    ASSERT_EQ(MatchRecord(39, 204), c.matches[12]);
    ASSERT_EQ(MatchRecord(39, 1001), c.matches[13]);
    ASSERT_EQ(MatchRecord(39, 1003), c.matches[14]);
    ASSERT_EQ(MatchRecord(48, 205), c.matches[15]);
    ASSERT_EQ(MatchRecord(48, 1001), c.matches[16]);
    ASSERT_EQ(MatchRecord(48, 1002), c.matches[17]);
    ASSERT_EQ(MatchRecord(48, 1003), c.matches[18]);
    ASSERT_EQ(MatchRecord(53, 202), c.matches[19]);
    ASSERT_EQ(MatchRecord(53, 1001), c.matches[20]);
    ASSERT_EQ(MatchRecord(53, 1002), c.matches[21]);
    ASSERT_EQ(MatchRecord(53, 1003), c.matches[22]);

    hs_free_database(db);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
}

TEST(LogicalCombination, MultiCombQuietDupSub4) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    CallBackContext c;
    string data = "abbdefxxfoobarrrghabcxdefxteakettleeeeexxxxijklmxxdef";
    const char *expr[] = {"abc", "def", "foobar.*gh", "teakettle{4,10}",
                          "ijkl[mMn]", "(201 & 202 & 203) | (204 & !205)",
                          "(201 | 202 & 203) & (!204 | 205)",
                          "((201 | 202) & 203) & (204 | 205)"};
    unsigned flags[] = {HS_FLAG_QUIET, HS_FLAG_QUIET, HS_FLAG_QUIET, 0,
                        HS_FLAG_QUIET, HS_FLAG_COMBINATION,
                        HS_FLAG_COMBINATION, HS_FLAG_COMBINATION};
    unsigned ids[] = {201, 202, 203, 204, 205, 1001, 1002, 1003};
    hs_error_t err = hs_compile_multi(expr, flags, ids, 8, HS_MODE_NOSTREAM,
                                      nullptr, &db, &compile_err);

    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    c.halt = 0;
    err = hs_scan(db, data.c_str(), data.size(), 0, scratch, record_cb,
                  (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(17U, c.matches.size());
    ASSERT_EQ(MatchRecord(18, 1002), c.matches[0]);
    ASSERT_EQ(MatchRecord(21, 1001), c.matches[1]);
    ASSERT_EQ(MatchRecord(21, 1002), c.matches[2]);
    ASSERT_EQ(MatchRecord(25, 1001), c.matches[3]);
    ASSERT_EQ(MatchRecord(25, 1002), c.matches[4]);
    ASSERT_EQ(MatchRecord(38, 204), c.matches[5]);
    ASSERT_EQ(MatchRecord(38, 1001), c.matches[6]);
    ASSERT_EQ(MatchRecord(38, 1003), c.matches[7]);
    ASSERT_EQ(MatchRecord(39, 204), c.matches[8]);
    ASSERT_EQ(MatchRecord(39, 1001), c.matches[9]);
    ASSERT_EQ(MatchRecord(39, 1003), c.matches[10]);
    ASSERT_EQ(MatchRecord(48, 1001), c.matches[11]);
    ASSERT_EQ(MatchRecord(48, 1002), c.matches[12]);
    ASSERT_EQ(MatchRecord(48, 1003), c.matches[13]);
    ASSERT_EQ(MatchRecord(53, 1001), c.matches[14]);
    ASSERT_EQ(MatchRecord(53, 1002), c.matches[15]);
    ASSERT_EQ(MatchRecord(53, 1003), c.matches[16]);

    hs_free_database(db);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
}

TEST(LogicalCombination, MultiCombUniSub5) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    CallBackContext c;
    string data = "abcdefxxfoobarrrghabcxdefxteakettleeeeexxxxijklmxxdef"
                  "-----------------------------------------------"
                  "cbbfedxxgoogleeecncbaxfedxhaystacksssssxxxxijkloxxfed"
                  "-----------------------------------------------"
                  "cabijklRfeexxgoobarrrjpcabxfeexshockwaveeeeexxxxijklsxxfee"
                  "------------------------------------------";
    const char *expr[] = {"abc", "def", "foobar.*gh", "teakettle{4,10}",
                          "ijkl[mMn]", "cba", "fed", "google.*cn",
                          "haystacks{4,8}", "ijkl[oOp]", "cab", "fee",
                          "goobar.*jp", "shockwave{4,6}", "ijkl[rRs]",
                          "(101 & 102 & 103) | (104 & !105)",
                          "(201 | 202 & 203) & (!204 | 205)",
                          "((301 | 302) & 303) & (304 | 305)"};
    unsigned flags[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        HS_FLAG_COMBINATION, HS_FLAG_COMBINATION,
                        HS_FLAG_COMBINATION};
    unsigned ids[] = {101, 102, 103, 104, 105, 201, 202, 203, 204, 205, 301,
                      302, 303, 304, 305, 1001, 1002, 1003};
    hs_error_t err = hs_compile_multi(expr, flags, ids, 18, HS_MODE_NOSTREAM,
                                      nullptr, &db, &compile_err);

    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    c.halt = 0;
    err = hs_scan(db, data.c_str(), data.size(), 0, scratch, record_cb,
                  (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(46U, c.matches.size());
    ASSERT_EQ(MatchRecord(3, 101), c.matches[0]);
    ASSERT_EQ(MatchRecord(6, 102), c.matches[1]);
    ASSERT_EQ(MatchRecord(18, 103), c.matches[2]);
    ASSERT_EQ(MatchRecord(18, 1001), c.matches[3]);
    ASSERT_EQ(MatchRecord(21, 101), c.matches[4]);
    ASSERT_EQ(MatchRecord(21, 1001), c.matches[5]);
    ASSERT_EQ(MatchRecord(25, 102), c.matches[6]);
    ASSERT_EQ(MatchRecord(25, 1001), c.matches[7]);
    ASSERT_EQ(MatchRecord(38, 104), c.matches[8]);
    ASSERT_EQ(MatchRecord(38, 1001), c.matches[9]);
    ASSERT_EQ(MatchRecord(39, 104), c.matches[10]);
    ASSERT_EQ(MatchRecord(39, 1001), c.matches[11]);
    ASSERT_EQ(MatchRecord(48, 105), c.matches[12]);
    ASSERT_EQ(MatchRecord(48, 1001), c.matches[13]);
    ASSERT_EQ(MatchRecord(53, 102), c.matches[14]);
    ASSERT_EQ(MatchRecord(53, 1001), c.matches[15]);
    ASSERT_EQ(MatchRecord(106, 202), c.matches[16]);
    ASSERT_EQ(MatchRecord(118, 203), c.matches[17]);
    ASSERT_EQ(MatchRecord(118, 1002), c.matches[18]);
    ASSERT_EQ(MatchRecord(121, 201), c.matches[19]);
    ASSERT_EQ(MatchRecord(121, 1002), c.matches[20]);
    ASSERT_EQ(MatchRecord(125, 202), c.matches[21]);
    ASSERT_EQ(MatchRecord(125, 1002), c.matches[22]);
    ASSERT_EQ(MatchRecord(138, 204), c.matches[23]);
    ASSERT_EQ(MatchRecord(139, 204), c.matches[24]);
    ASSERT_EQ(MatchRecord(148, 205), c.matches[25]);
    ASSERT_EQ(MatchRecord(148, 1002), c.matches[26]);
    ASSERT_EQ(MatchRecord(153, 202), c.matches[27]);
    ASSERT_EQ(MatchRecord(153, 1002), c.matches[28]);
    ASSERT_EQ(MatchRecord(203, 301), c.matches[29]);
    ASSERT_EQ(MatchRecord(208, 305), c.matches[30]);
    ASSERT_EQ(MatchRecord(211, 302), c.matches[31]);
    ASSERT_EQ(MatchRecord(223, 303), c.matches[32]);
    ASSERT_EQ(MatchRecord(223, 1003), c.matches[33]);
    ASSERT_EQ(MatchRecord(226, 301), c.matches[34]);
    ASSERT_EQ(MatchRecord(226, 1003), c.matches[35]);
    ASSERT_EQ(MatchRecord(230, 302), c.matches[36]);
    ASSERT_EQ(MatchRecord(230, 1003), c.matches[37]);
    ASSERT_EQ(MatchRecord(243, 304), c.matches[38]);
    ASSERT_EQ(MatchRecord(243, 1003), c.matches[39]);
    ASSERT_EQ(MatchRecord(244, 304), c.matches[40]);
    ASSERT_EQ(MatchRecord(244, 1003), c.matches[41]);
    ASSERT_EQ(MatchRecord(253, 305), c.matches[42]);
    ASSERT_EQ(MatchRecord(253, 1003), c.matches[43]);
    ASSERT_EQ(MatchRecord(258, 302), c.matches[44]);
    ASSERT_EQ(MatchRecord(258, 1003), c.matches[45]);

    hs_free_database(db);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
}

TEST(LogicalCombination, MultiCombQuietUniSub5) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    CallBackContext c;
    string data = "abcdefxxfoobarrrghabcxdefxteakettleeeeexxxxijklmxxdef"
                  "-----------------------------------------------"
                  "cbbfedxxgoogleeecncbaxfedxhaystacksssssxxxxijkloxxfed"
                  "-----------------------------------------------"
                  "cabijklRfeexxgoobarrrjpcabxfeexshockwaveeeeexxxxijklsxxfee"
                  "------------------------------------------";
    const char *expr[] = {"abc", "def", "foobar.*gh", "teakettle{4,10}",
                          "ijkl[mMn]", "cba", "fed", "google.*cn",
                          "haystacks{4,8}", "ijkl[oOp]", "cab", "fee",
                          "goobar.*jp", "shockwave{4,6}", "ijkl[rRs]",
                          "(101 & 102 & 103) | (104 & !105)",
                          "(201 | 202 & 203) & (!204 | 205)",
                          "((301 | 302) & 303) & (304 | 305)"};
    unsigned flags[] = {0, HS_FLAG_QUIET, HS_FLAG_QUIET, HS_FLAG_QUIET, 0,
                        HS_FLAG_QUIET, 0, HS_FLAG_QUIET, 0, HS_FLAG_QUIET,
                        HS_FLAG_QUIET, HS_FLAG_QUIET, 0, HS_FLAG_QUIET, 0,
                        HS_FLAG_COMBINATION, HS_FLAG_COMBINATION,
                        HS_FLAG_COMBINATION};
    unsigned ids[] = {101, 102, 103, 104, 105, 201, 202, 203, 204, 205, 301,
                      302, 303, 304, 305, 1001, 1002, 1003};
    hs_error_t err = hs_compile_multi(expr, flags, ids, 18, HS_MODE_NOSTREAM,
                                      nullptr, &db, &compile_err);

    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    c.halt = 0;
    err = hs_scan(db, data.c_str(), data.size(), 0, scratch, record_cb,
                  (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_EQ(30U, c.matches.size());
    ASSERT_EQ(MatchRecord(3, 101), c.matches[0]);
    ASSERT_EQ(MatchRecord(18, 1001), c.matches[1]);
    ASSERT_EQ(MatchRecord(21, 101), c.matches[2]);
    ASSERT_EQ(MatchRecord(21, 1001), c.matches[3]);
    ASSERT_EQ(MatchRecord(25, 1001), c.matches[4]);
    ASSERT_EQ(MatchRecord(38, 1001), c.matches[5]);
    ASSERT_EQ(MatchRecord(39, 1001), c.matches[6]);
    ASSERT_EQ(MatchRecord(48, 105), c.matches[7]);
    ASSERT_EQ(MatchRecord(48, 1001), c.matches[8]);
    ASSERT_EQ(MatchRecord(53, 1001), c.matches[9]);
    ASSERT_EQ(MatchRecord(106, 202), c.matches[10]);
    ASSERT_EQ(MatchRecord(118, 1002), c.matches[11]);
    ASSERT_EQ(MatchRecord(121, 1002), c.matches[12]);
    ASSERT_EQ(MatchRecord(125, 202), c.matches[13]);
    ASSERT_EQ(MatchRecord(125, 1002), c.matches[14]);
    ASSERT_EQ(MatchRecord(138, 204), c.matches[15]);
    ASSERT_EQ(MatchRecord(139, 204), c.matches[16]);
    ASSERT_EQ(MatchRecord(148, 1002), c.matches[17]);
    ASSERT_EQ(MatchRecord(153, 202), c.matches[18]);
    ASSERT_EQ(MatchRecord(153, 1002), c.matches[19]);
    ASSERT_EQ(MatchRecord(208, 305), c.matches[20]);
    ASSERT_EQ(MatchRecord(223, 303), c.matches[21]);
    ASSERT_EQ(MatchRecord(223, 1003), c.matches[22]);
    ASSERT_EQ(MatchRecord(226, 1003), c.matches[23]);
    ASSERT_EQ(MatchRecord(230, 1003), c.matches[24]);
    ASSERT_EQ(MatchRecord(243, 1003), c.matches[25]);
    ASSERT_EQ(MatchRecord(244, 1003), c.matches[26]);
    ASSERT_EQ(MatchRecord(253, 305), c.matches[27]);
    ASSERT_EQ(MatchRecord(253, 1003), c.matches[28]);
    ASSERT_EQ(MatchRecord(258, 1003), c.matches[29]);

    hs_free_database(db);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
}
