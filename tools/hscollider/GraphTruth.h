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

#ifndef GRAPHTRUTH_H
#define GRAPHTRUTH_H

#include "expressions.h"
#include "ResultSet.h"

#include "hs_compile.h" // for hs_expr_ext
#include "ue2common.h"

#include <memory>
#include <mutex>
#include <string>

#include <boost/core/noncopyable.hpp>

namespace ue2 {

class ReportManager;
struct BoundaryReports;

} // namespace ue2

struct NGCompileFailure {
    explicit NGCompileFailure(const std::string &msg_s) : msg(msg_s) {}
    std::string msg;
};

struct NGUnsupportedFailure {
    explicit NGUnsupportedFailure(const std::string &msg_s) : msg(msg_s) {}
    std::string msg;
};

// Struct to store the actual compiled NFA graph.
class CompiledNG;

// Struct to store the precompile information about the graph.
class CNGInfo : boost::noncopyable {
public:
    CNGInfo(unsigned id_in, const ExpressionMap &m_expr_in);
    ~CNGInfo();

    bool is_bad() {
        std::lock_guard<std::mutex> lock(bad_mutex);
        bool val = bad;
        return val;
    }

    void mark_bad() {
        std::lock_guard<std::mutex> lock(bad_mutex);
        bad = true;
    }

    const CompiledNG *get() {
        std::lock_guard<std::mutex> lock(cng_mutex);

        if (cng) {
            return cng.get();
        }

        // NFA graph hasn't been compiled yet.
        try {
            compile();
        } catch (NGCompileFailure &e) {
            throw NGCompileFailure(e);
        } catch (NGUnsupportedFailure &e) {
            throw NGCompileFailure(e.msg);
        }

        return cng.get();
    }

    u64a min_offset = 0;
    u64a max_offset = 0;
    u64a min_length = 0;
    u32 max_edit_distance = 0;
    u32 max_hamm_distance = 0;
    bool utf8 = false;
    bool highlander = false;
    bool prefilter = false;
    bool som = false;
    bool combination = false;
    bool quiet = false;

    unsigned id;
private:
    void compile();
    // If NFA graph scan failed for some reason, we mark it as bad and skip
    // the remaining tests for it for performance reasons.
    bool bad = false;
    std::mutex bad_mutex; // serialised accesses to bad flag.

    std::unique_ptr<CompiledNG> cng; // compiled NFA graph
    std::mutex cng_mutex; // serialised accesses to NFA graph

    // Our expression map
    const ExpressionMap &m_expr;
};


class GraphTruth : boost::noncopyable {
public:
    GraphTruth(std::ostream &os, const ExpressionMap &expr);

    bool run(unsigned id, const CompiledNG &cng, const CNGInfo &cngi,
             const std::string &buffer, ResultSet &rs, std::string &error);

    std::unique_ptr<CNGInfo> preprocess(unsigned id,
                                        bool ignoreUnsupported = false);

private:
    // Output stream.
    std::ostream &out;

    // Our expression map
    const ExpressionMap &m_expr;
};

#endif
