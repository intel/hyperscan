/*
 * Copyright (C) 2026 Intel Corporation
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

#ifndef ENGINEHYPERSCAN_H
#define ENGINEHYPERSCAN_H

#include "expressions.h"
#include "engine.h"
#include "hs_runtime.h"

#include <memory>
#include <string>
#include <vector>

/** Infomation about the database compile */
struct CompileHSStats {
    std::string sigs_name;
    std::string signatures;
    std::string db_info;
    size_t expressionCount = 0;
    size_t compiledSize = 0;
    std::string hmac_hex;
    bool streaming;
    size_t streamSize = 0;
    size_t scratchSize = 0;
    long double compileSecs = 0;
    unsigned int peakMemorySize = 0;
};

/** Engine context which is allocated on a per-thread basis. */
class EngineHSContext : public EngineContext {
public:
    explicit EngineHSContext(const hs_database_t *db);
    ~EngineHSContext();

    EngineHSContext(const EngineHSContext &) = delete;
    EngineHSContext &operator=(const EngineHSContext &) = delete;

    hs_scratch_t *scratch = nullptr;
};

/** Streaming mode scans have persistent stream state associated with them. */
class EngineHSStream : public EngineStream {
public:
    ~EngineHSStream();
    hs_stream_t *id = nullptr;
    EngineHSContext *ctx = nullptr;
};

/** Hyperscan Engine for scanning data. */
class EngineHyperscan : public Engine {
public:
    explicit EngineHyperscan(hs_database_t *db, const CompileHSStats &cs);
    ~EngineHyperscan();

    EngineHyperscan(const EngineHyperscan &) = delete;
    EngineHyperscan &operator=(const EngineHyperscan &) = delete;
    
    std::unique_ptr<EngineContext> makeContext() const;

    void scan(const char *data, unsigned int len, unsigned int id,
              ResultEntry &result, EngineContext &ectx) const;

    void scan_rlit(const char *data, unsigned int len, unsigned int id,
                   ResultEntry &result, EngineContext &ectx) const;

    void scan_vectored(const char *const *data, const unsigned int *len,
                       unsigned int count, unsigned int streamId,
                       ResultEntry &result, EngineContext &ectx) const;

    std::unique_ptr<EngineStream> streamOpen(EngineContext &ectx,
                                             unsigned id) const;

    void streamClose(std::unique_ptr<EngineStream> stream,
                     ResultEntry &result) const;

    void streamCompressExpand(EngineStream &stream,
                              std::vector<char> &temp) const;

    void streamScan(EngineStream &stream, const char *data, unsigned int len,
                    unsigned int id, ResultEntry &result) const;

    void printStats() const;

    void printCsvStats() const;

    void sqlStats(SqlDB &db) const;

private:
    hs_database_t *db;
    CompileHSStats compile_stats;
};

namespace ue2 {
struct Grey;
}

std::unique_ptr<EngineHyperscan>
buildEngineHyperscan(const ExpressionMap &expressions, ScanMode scan_mode,
                     const std::string &name, const std::string &sigs_name,
                     const ue2::Grey &grey);

#endif // ENGINEHYPERSCAN_H
