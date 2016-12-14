/*
 * Copyright (c) 2016, Intel Corporation
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

#ifndef ENGINEHYPERSCAN_H
#define ENGINEHYPERSCAN_H

#include "expressions.h"
#include "common.h"
#include "hs_runtime.h"

#include <memory>

/** Structure for the result of a single complete scan. */
struct ResultEntry {
    double seconds = 0;       //!< Time taken for scan.
    unsigned int matches = 0; //!< Count of matches found.
};

/** Engine context which is allocated on a per-thread basis. */
class EngineContext {
public:
    explicit EngineContext(const hs_database_t *db);
    ~EngineContext();

    hs_scratch_t *scratch = nullptr;
};

/** Streaming mode scans have persistent stream state associated with them. */
class EngineStream {
public:
    hs_stream_t *id;
    unsigned int sn;
    EngineContext *ctx;
};

/** Hyperscan Engine for scanning data. */
class EngineHyperscan {
public:
    explicit EngineHyperscan(hs_database_t *db);
    ~EngineHyperscan();

    std::unique_ptr<EngineContext> makeContext() const;

    void scan(const char *data, unsigned int len, unsigned int id,
              ResultEntry &result, EngineContext &ctx) const;

    void scan_vectored(const char *const *data, const unsigned int *len,
                       unsigned int count, unsigned int streamId,
                       ResultEntry &result, EngineContext &ctx) const;

    std::unique_ptr<EngineStream> streamOpen(EngineContext &ctx,
                                             unsigned id) const;

    void streamClose(std::unique_ptr<EngineStream> stream,
                     ResultEntry &result) const;

    void streamScan(EngineStream &stream, const char *data, unsigned int len,
                    unsigned int id, ResultEntry &result) const;

private:
    hs_database_t *db;
};

namespace ue2 {
struct Grey;
}

std::unique_ptr<EngineHyperscan>
buildEngineHyperscan(const ExpressionMap &expressions, ScanMode scan_mode,
                     const std::string &name, const ue2::Grey &grey);

#endif // ENGINEHYPERSCAN_H
