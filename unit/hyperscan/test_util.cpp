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

#include "hs.h"
#include "test_util.h"
#include "gtest/gtest.h"
#include "util/expressions.h"
#include "util/ExpressionParser.h"

#include <cstring>
#include <iostream>
#include <string>
#include <vector>

using namespace std;

int record_cb(unsigned id, unsigned long long, unsigned long long to,
              unsigned, void *ctxt) {
    CallBackContext *c = (CallBackContext *)ctxt;

    c->matches.emplace_back(to, id);

    return (int)c->halt;
}

std::ostream &operator<<(std::ostream &o, const MatchRecord &m) {
    return o << "[" << m.to << ", " << m.id << "]";
}

std::ostream &operator<<(std::ostream &o, const pattern &p) {
    return o << "[" << "expr=\"" << p.expression << "\", flags=" << p.flags
             << ", id=" << p.id << "]";
}

hs_database_t *buildDB(const vector<pattern> &patterns, unsigned int mode,
                       hs_platform_info *plat) {
    vector<const char *> expressions;
    vector<unsigned int> flags;
    vector<unsigned int> ids;
    vector<const hs_expr_ext *> ext;

    for (const auto &pat : patterns) {
        expressions.push_back(pat.expression.c_str());
        flags.push_back(pat.flags);
        ids.push_back(pat.id);
        ext.push_back(&pat.ext);
    }

    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err = nullptr;
    hs_error_t err;

    err = hs_compile_ext_multi(expressions.data(), flags.data(), ids.data(),
                               ext.data(), patterns.size(), mode, plat, &db,
                               &compile_err);

    if (err != HS_SUCCESS) {
        return nullptr;
    }

    return db;
}

hs_database_t *buildDB(const pattern &expr, unsigned int mode) {
    return buildDB(vector<pattern>({expr}), mode);
}

hs_database_t *buildDB(const char *expression, unsigned int flags,
                       unsigned int id, unsigned int mode,
                       hs_platform_info_t *plat) {
    return buildDB({pattern(expression, flags, id)}, mode, plat);
}

hs_database_t *buildDB(const char *filename, unsigned int mode,
                       unsigned int extra_flags) {
    vector<pattern> patterns;
    ExpressionMap expressions;
    loadExpressionsFromFile(filename, expressions);

    for (const auto &expr : expressions) {
        unsigned int flags = 0;
        string regex;
        hs_expr_ext ext;
        if (!readExpression(expr.second, regex, &flags, &ext)) {
            return nullptr;
        }
        patterns.emplace_back(regex, flags | extra_flags, expr.first, ext);
    }
    return buildDB(patterns, mode);
}

static
bool isOrdered(const string &expr, unsigned int flags) {
    // SOM doesn't produce ordered matches?
    if (flags & HS_FLAG_SOM_LEFTMOST) {
        return false;
    }

    hs_expr_info_t *info = nullptr;
    hs_compile_error_t *error = nullptr;
    hs_error_t err = hs_expression_info(expr.c_str(), flags, &info, &error);
    if (err != HS_SUCCESS) {
        // Expression will fail compilation and report error elsewhere.
        free(info);
        hs_free_compile_error(error);
        return false;
    }

    assert(info);

    // Any pattern that does not require offset adjustment should produce
    // matches in order.
    bool ordered = !info->unordered_matches;
    free(info);
    return ordered;
}

hs_database_t *buildDB(const char *filename, unsigned int mode,
                       bool check_ordering) {
    vector<pattern> patterns;
    ExpressionMap expressions;
    loadExpressionsFromFile(filename, expressions);

    for (const auto &expr : expressions) {
        unsigned int flags = 0;
        string regex;
        hs_expr_ext ext;
        bool must_be_ordered;
        if (!readExpression(expr.second, regex, &flags, &ext,
                            &must_be_ordered)) {
            return nullptr;
        }

        if (check_ordering && must_be_ordered && !isOrdered(regex, flags)) {
            return nullptr;
        }

        patterns.emplace_back(regex, flags, expr.first, ext);
    }
    return buildDB(patterns, mode);
}

hs_database_t *buildDBAndScratch(const char *expression, unsigned int flags,
                                 unsigned int id, unsigned int mode,
                                 hs_scratch_t **scratch) {
    hs_database_t *db = buildDB(expression, flags, id, mode);
    EXPECT_TRUE(db != nullptr);

    *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, scratch);
    EXPECT_EQ(HS_SUCCESS, err);
    EXPECT_TRUE(*scratch != nullptr);

    return db;
}

size_t allocated_count;
size_t allocated_count_b;

void *count_malloc(size_t n) {
    void *pp = malloc(n + 16);
    if (!pp) {
        return nullptr;
    }

    allocated_count += n;
    *(size_t *)pp = n;
    void *p = (char *)pp + 16;

    return p;
}

void count_free(void *p) {
    if (!p) {
        return;
    }

    void *pp = (char *)p - 16;
    size_t n = *(size_t *)pp;

    allocated_count -= n;

    free(pp);
}

void *count_malloc_b(size_t n) {
    void *pp = malloc(n + 32);
    if (!pp) {
        return nullptr;
    }

    allocated_count_b += n;
    *(size_t *)pp = n;
    void *p = (char *)pp + 32;

    return p;
}

void count_free_b(void *p) {
    if (!p) {
        return;
    }

    void *pp = (char *)p - 32;
    size_t n = *(size_t *)pp;

    allocated_count_b -= n;

    free(pp);
}
