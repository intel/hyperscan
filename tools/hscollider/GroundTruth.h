/*
 * Copyright (c) 2015-2018, Intel Corporation
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

#ifndef GROUNDTRUTH_H
#define GROUNDTRUTH_H

#include "expressions.h"
#include "ResultSet.h"
#include "parser/logical_combination.h"

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <pcre.h>

#include <boost/core/noncopyable.hpp>

// Thrown by GroundTruth::compile in the event of a PCRE compile failure.
struct PcreCompileFailure {
    PcreCompileFailure(const std::string &msg_s) : msg(msg_s) {}
    std::string msg;
};

// Thrown in the event of a "soft" PCRE compile failure, one that we don't want
// to consider a ue2collider failure (e.g. "regular expression too large").
struct SoftPcreCompileFailure : PcreCompileFailure {
    SoftPcreCompileFailure(const std::string &msg_s)
        : PcreCompileFailure(msg_s) {}
};

// Struct to store everything about a PCRE. Note that the code assumes that
// once populated, the data in this structure will remain constant while tests
// are running, except for the bad flag (which is protected by a mutex).
class CompiledPcre : boost::noncopyable {
public:
    CompiledPcre() {}
    ~CompiledPcre() {
        free(bytecode);
    }

    bool is_bad() {
        std::lock_guard<std::mutex> lock(bad_mutex);
        bool val = bad;
        return val;
    }

    void mark_bad() {
        std::lock_guard<std::mutex> lock(bad_mutex);
        bad = true;
    }

    std::string expression;
    pcre *bytecode = nullptr;
    unsigned long long min_offset = 0;
    unsigned long long max_offset = ~0ULL;
    unsigned long long min_length = 0;
    int captureCount = 0;
    bool utf8 = false;
    bool highlander = false;
    bool prefilter = false;
    bool som = false;
    bool combination = false;
    bool quiet = false;

    // Parsed logical combinations.
    ue2::ParsedLogical pl;

    // Combination expression report id.
    unsigned report;

private:
    // If a PCRE has hit its match recursion limit when scanning a corpus, we
    // mark it as bad and skip the remaining tests for it for performance
    // reasons.
    bool bad = false;
    std::mutex bad_mutex; // serialised accesses to bad flag.
};

// Wrapper around libpcre to generate results for an expression and corpus.
class GroundTruth : boost::noncopyable {
public:
    GroundTruth(std::ostream &os, const ExpressionMap &expr,
                unsigned long limit, unsigned long limit_recursion);

    static void global_prep();

    std::unique_ptr<CompiledPcre> compile(unsigned id,
                                          bool no_callouts = false);

    bool run(unsigned id, const CompiledPcre &compiled,
             const std::string &buffer, ResultSet &rs, std::string &error);

private:
    // Output stream.
    std::ostream &out;

    // Our expression map
    const ExpressionMap &m_expr;

    // PCRE match limit
    const unsigned long int matchLimit;
    const unsigned long int matchLimitRecursion;

    // Persistent ovector used to run tests.
    std::vector<int> ovector;
};

#endif
