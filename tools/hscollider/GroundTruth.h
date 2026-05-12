/*
 * Copyright (C) 2025 Intel Corporation
 *
 * This software and the related documents are Intel copyrighted materials,
 * and your use of them is governed by the express license under which they were
 * provided to you ("License"). Unless the License provides otherwise,
 * you may not use, modify, copy, publish, distribute, disclose or transmit this
 * software or the related documents without Intel's prior written permission.
 *
 * This software and the related documents are provided as is, with no express or
 * implied warranties, other than those that are expressly stated in the License.
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
    unsigned report = 0;

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
