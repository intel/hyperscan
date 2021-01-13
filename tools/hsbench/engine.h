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

#ifndef ENGINE_H
#define ENGINE_H

#include "common.h"
#include "sqldb.h"

#include <memory>
#include <string>
#include <vector>

#include <boost/core/noncopyable.hpp>

// Engines have an engine context which is allocated on a per-thread basis.
class EngineContext : boost::noncopyable {
public:
    virtual ~EngineContext();
};

/** Streaming mode scans have persistent stream state associated with them. */
class EngineStream : boost::noncopyable {
public:
    virtual ~EngineStream();
    unsigned int sn;
};

// Benchmarking engine
class Engine : boost::noncopyable {
public:
    virtual ~Engine();

    // allocate an EngineContext
    virtual std::unique_ptr<EngineContext> makeContext() const = 0;

    // non-streaming scan
    virtual void scan(const char *data, unsigned len, unsigned blockId,
                      ResultEntry &results, EngineContext &ectx) const = 0;

    // vectoring scan
    virtual void scan_vectored(const char *const *data,
                               const unsigned int *len, unsigned int count,
                               unsigned int streamId, ResultEntry &result,
                               EngineContext &ectx) const = 0;

    // stream open
    virtual std::unique_ptr<EngineStream> streamOpen(EngineContext &ectx,
                                                     unsigned id) const = 0;

    // stream close
    virtual void streamClose(std::unique_ptr<EngineStream> stream,
                             ResultEntry &result) const = 0;

    // stream compress and expand
    virtual void streamCompressExpand(EngineStream &stream,
                                      std::vector<char> &temp) const = 0;

    // streaming scan
    virtual void streamScan(EngineStream &stream, const char *data,
                            unsigned int len, unsigned int id,
                            ResultEntry &result) const = 0;

    virtual void printStats() const = 0;

    virtual void printCsvStats() const = 0;

    virtual void sqlStats(SqlDB &db) const = 0;
};

#endif // ENGINE_H
