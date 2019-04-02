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

#include <algorithm>
#include <array>
#include <iostream>
#include <vector>

#include "gtest/gtest.h"
#include "hs.h"
#include "config.h"
#include "test_util.h"

using namespace std;

TEST(MMAdaptor, norm_cont1) { // UE-901
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    CallBackContext c;
    string data = "aooAaooAbarZ";
    const char *expr[] = {"aoo[A-K]", "bar[L-Z]"};
    unsigned flags[] = {0, 0};
    unsigned ids[] = {30, 31};
    hs_error_t err = hs_compile_multi(expr, flags, ids, 2, HS_MODE_NOSTREAM,
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
    ASSERT_EQ(3U, c.matches.size());
    ASSERT_EQ(MatchRecord(4, 30), c.matches[0]);
    ASSERT_EQ(MatchRecord(8, 30), c.matches[1]);
    ASSERT_EQ(MatchRecord(12, 31), c.matches[2]);

    hs_free_database(db);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
}

TEST(MMAdaptor, norm_cont2) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    CallBackContext c;
    string data = "aooAaooAbarZ                      ";
    const char *expr[] = {"aoo[A-K][^\n]{16}", "bar[L-Z][^\n]{16}"};
    unsigned flags[] = {0, 0};
    unsigned ids[] = {30, 31};
    hs_error_t err = hs_compile_multi(expr, flags, ids, 2, HS_MODE_NOSTREAM,
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
    ASSERT_EQ(3U, c.matches.size());
    ASSERT_TRUE(c.matches.end() != find(c.matches.begin(), c.matches.end(), MatchRecord(20, 30)));
    ASSERT_TRUE(c.matches.end() != find(c.matches.begin(), c.matches.end(), MatchRecord(24, 30)));
    ASSERT_TRUE(c.matches.end() != find(c.matches.begin(), c.matches.end(), MatchRecord(28, 31)));

    hs_free_database(db);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
}

TEST(MMAdaptor, norm_halt1) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    CallBackContext c;
    string data = "aooAaooAbarZ";
    const char *expr[] = {"aoo[A-K]", "bar[L-Z]"};
    unsigned flags[] = {0, 0};
    unsigned ids[] = {30, 31};
    hs_error_t err = hs_compile_multi(expr, flags, ids, 2, HS_MODE_NOSTREAM,
                                      nullptr, &db, &compile_err);

    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    c.halt = 1;
    err = hs_scan(db, data.c_str(), data.size(), 0, scratch, record_cb,
                  (void *)&c);
    ASSERT_EQ(HS_SCAN_TERMINATED, err);
    ASSERT_EQ(1U, c.matches.size());
    ASSERT_EQ(MatchRecord(4, 30), c.matches[0]);

    hs_free_database(db);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
}

TEST(MMAdaptor, norm_halt2) { // UE-901
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    CallBackContext c;
    string data = "aooAaooAbarZ                      ";
    const char *expr[] = {"aoo[A-K][^\n]{16}", "bar[L-Z][^\n]{16}"};
    unsigned flags[] = {0, 0};
    unsigned ids[] = {30, 31};
    hs_error_t err = hs_compile_multi(expr, flags, ids, 2, HS_MODE_NOSTREAM,
                                      nullptr, &db, &compile_err);

    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    c.halt = 1;
    err = hs_scan(db, data.c_str(), data.size(), 0, scratch, record_cb,
                  (void *)&c);
    ASSERT_EQ(HS_SCAN_TERMINATED, err);
    ASSERT_EQ(1U, c.matches.size());
    ASSERT_EQ(MatchRecord(20, 30), c.matches[0]);

    hs_free_database(db);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
}

TEST(MMAdaptor, high_cont1) { // UE-901
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    CallBackContext c;
    string data = "aooAaooAbarZ";
    const char *expr[] = {"aoo[A-K]", "bar[L-Z]"};
    unsigned flags[] = {HS_FLAG_SINGLEMATCH, 0};
    unsigned ids[] = {30, 31};
    hs_error_t err = hs_compile_multi(expr, flags, ids, 2, HS_MODE_NOSTREAM,
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
    ASSERT_TRUE(c.matches.end() != find(c.matches.begin(), c.matches.end(), MatchRecord(4, 30)));
    ASSERT_TRUE(c.matches.end() != find(c.matches.begin(), c.matches.end(), MatchRecord(12, 31)));

    hs_free_database(db);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
}

TEST(MMAdaptor, high_cont2) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    CallBackContext c;
    string data = "aooAaooAbarZ                      ";
    const char *expr[] = {"aoo[A-K][^\n]{16}", "bar[L-Z][^\n]{16}"};
    unsigned flags[] = {HS_FLAG_SINGLEMATCH, 0};
    unsigned ids[] = {30, 31};
    hs_error_t err = hs_compile_multi(expr, flags, ids, 2, HS_MODE_NOSTREAM,
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
    ASSERT_TRUE(c.matches.end() != find(c.matches.begin(), c.matches.end(), MatchRecord(20, 30)));
    ASSERT_TRUE(c.matches.end() != find(c.matches.begin(), c.matches.end(), MatchRecord(28, 31)));

    hs_free_database(db);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
}

TEST(MMAdaptor, high_halt1) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    CallBackContext c;
    string data = "aooAaooAbarZ";
    const char *expr[] = {"aoo[A-K]", "bar[L-Z]"};
    unsigned flags[] = {HS_FLAG_SINGLEMATCH, 0};
    unsigned ids[] = {30, 31};
    hs_error_t err = hs_compile_multi(expr, flags, ids, 2, HS_MODE_NOSTREAM,
                                      nullptr, &db, &compile_err);

    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    c.halt = 1;
    err = hs_scan(db, data.c_str(), data.size(), 0, scratch, record_cb,
                  (void *)&c);
    ASSERT_EQ(HS_SCAN_TERMINATED, err);
    ASSERT_EQ(1U, c.matches.size());
    ASSERT_EQ(MatchRecord(4, 30), c.matches[0]);

    hs_free_database(db);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
}

TEST(MMAdaptor, high_halt2) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    CallBackContext c;
    string data = "aooAaooAbarZbarZaooA                      ";
    const char *expr[] = {"aoo[A-K][^\n]{16}", "bar[L-Z][^\n]{16}"};
    unsigned flags[] = {HS_FLAG_SINGLEMATCH, 0};
    unsigned ids[] = {30, 31};
    hs_error_t err = hs_compile_multi(expr, flags, ids, 2, HS_MODE_NOSTREAM,
                                      nullptr, &db, &compile_err);

    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(db != nullptr);

    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);
    ASSERT_TRUE(scratch != nullptr);

    c.halt = 1;
    err = hs_scan(db, data.c_str(), data.size(), 0, scratch, record_cb,
                  (void *)&c);
    ASSERT_EQ(HS_SCAN_TERMINATED, err);
    ASSERT_EQ(1U, c.matches.size());
    ASSERT_TRUE(MatchRecord(20, 30) == c.matches[0]
                || MatchRecord(28, 31) == c.matches[0]);

    hs_free_database(db);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
}

TEST(MPV, UE_2395) {
    vector<pattern> patterns;
    patterns.push_back(pattern("^.{200}", HS_FLAG_DOTALL, 1));
    patterns.push_back(pattern(".{40,}", HS_FLAG_DOTALL, 2));
    patterns.push_back(pattern("aaa", HS_FLAG_DOTALL, 3));

    hs_database_t *db = buildDB(patterns, HS_MODE_BLOCK);
    ASSERT_NE(nullptr, db);

    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);
    ASSERT_EQ(HS_SUCCESS, err);

    array<char, 300> data;
    data.fill('a');

    CallBackContext c;
    err = hs_scan(db, data.data(), data.size(), 0, scratch, record_cb,
                  (void *)&c);
    ASSERT_EQ(HS_SUCCESS, err);

    unsigned seen = 39;
    for (vector<MatchRecord>::const_iterator it = c.matches.begin();
         it != c.matches.end(); ++it) {
        if (it->id != 2) {
            if (it->id == 1) {
                ASSERT_EQ(200, it->to);
            }
            continue;
        }
        ASSERT_EQ(seen + 1, it->to);
        seen = it->to;
    }

    ASSERT_EQ(300, seen);

    hs_free_database(db);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
}

TEST(MMRoseLiteralPath, issue_141) {
    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    CallBackContext c;
    string data = "/odezhda-dlya-bega/";
    const char *expr[] = {"/odezhda-dlya-bega/",
                          "kurtki-i-vetrovki-dlya-bega",
                          "futbolki-i-mayki-dlya-bega"};
    unsigned flags[] = {HS_FLAG_DOTALL | HS_FLAG_SINGLEMATCH,
                        HS_FLAG_DOTALL | HS_FLAG_SINGLEMATCH,
                        HS_FLAG_DOTALL | HS_FLAG_SINGLEMATCH};
    hs_error_t err = hs_compile_multi(expr, flags, nullptr, 3, HS_MODE_BLOCK,
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
    ASSERT_EQ(1U, c.matches.size());
    ASSERT_EQ(MatchRecord(19, 0), c.matches[0]);

    hs_free_database(db);
    err = hs_free_scratch(scratch);
    ASSERT_EQ(HS_SUCCESS, err);
}
