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

#ifndef TEST_UTIL_H
#define TEST_UTIL_H

#include "hs.h"

#include <cstring>
#include <iosfwd>
#include <string>
#include <vector>

#ifndef UNUSED
#if defined(_WIN32) || defined(_WIN64)
#define UNUSED
#else
#define UNUSED __attribute__ ((unused))
#endif
#endif

struct MatchRecord {
    MatchRecord(unsigned long long t, int i) : to(t), id(i) {}
    bool operator==(const MatchRecord &o) const {
        return to == o.to && id == o.id;
    }
    unsigned long long to;
    int id;
};

std::ostream &operator<<(std::ostream &o, const MatchRecord &m);

struct CallBackContext {
    bool halt = false;
    std::vector<MatchRecord> matches;

    void clear() {
        halt = false;
        matches.clear();
    }
};

int record_cb(unsigned id, unsigned long long, unsigned long long to,
              unsigned, void *ctxt);

// Dummy callback: does nothing, returns 0 (keep matching)
static UNUSED
int dummy_cb(unsigned, unsigned long long, unsigned long long, unsigned,
             void *) {
    // empty
    return 0;
}

struct pattern {
    std::string expression;
    unsigned int flags = 0;
    unsigned int id = 0;
    hs_expr_ext ext;

    // We need a default constructor for combining in parameterised tests.
    pattern() {
        memset(&ext, 0, sizeof(ext));
    }

    explicit pattern(std::string expression_in,
                     unsigned int flags_in = 0, unsigned int id_in = 0)
        : expression(std::move(expression_in)), flags(flags_in), id(id_in) {
        memset(&ext, 0, sizeof(ext));
    }

    pattern(std::string expression_in, unsigned int flags_in,
            unsigned int id_in, hs_expr_ext ext_in)
        : expression(std::move(expression_in)), flags(flags_in), id(id_in),
          ext(std::move(ext_in)) {}
};

std::ostream &operator<<(std::ostream &o, const pattern &p);

hs_database_t *buildDB(const std::vector<pattern> &patterns, unsigned int mode,
                       hs_platform_info *plat = nullptr);
hs_database_t *buildDB(const pattern &pat, unsigned int mode);
hs_database_t *buildDB(const char *expression, unsigned int flags,
                       unsigned int id, unsigned int mode,
                       hs_platform_info *plat = nullptr);
hs_database_t *buildDB(const char *filename, unsigned int mode,
                       unsigned int extra_flags = 0);
hs_database_t *buildDB(const char *filename, unsigned int mode,
                       bool check_ordering);

hs_database_t *buildDBAndScratch(const char *expression, unsigned int flags,
                                 unsigned int id, unsigned int mode,
                                 hs_scratch_t **scratch);


extern size_t allocated_count;
extern size_t allocated_count_b;

void *count_malloc(size_t n);
void *count_malloc_b(size_t n);
void count_free(void *p);
void count_free_b(void *p);

#endif
