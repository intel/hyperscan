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

#ifndef ENGINEPCRE_H
#define ENGINEPCRE_H

#include "expressions.h"
#include "engine.h"

#include <pcre.h>

#include <memory>
#include <string>
#include <vector>

/** Infomation about the database compile */
struct CompilePCREStats {
    std::string sigs_name;
    std::string signatures;
    std::string db_info;
    size_t expressionCount = 0;
    size_t compiledSize = 0;
    size_t scratchSize = 0;
    long double compileSecs = 0;
    unsigned int peakMemorySize = 0;
};

/** Engine context which is allocated on a per-thread basis. */
class EnginePCREContext : public EngineContext{
public:
    explicit EnginePCREContext(int capture_cnt);
    ~EnginePCREContext();

    EnginePCREContext(const EnginePCREContext &) = delete;
    EnginePCREContext &operator=(const EnginePCREContext &) = delete;

    int *ovec = nullptr;
};

struct PcreDB {
    bool highlander = false;
    bool utf8 = false;
    u32 id = 0;
    pcre *db = nullptr;
    pcre_extra *extra = nullptr;
};

/** PCRE Engine for scanning data. */
class EnginePCRE : public Engine {
public:
    explicit EnginePCRE(std::vector<std::unique_ptr<PcreDB>> dbs_in,
                        const CompilePCREStats &cs, int capture_cnt_in);
    ~EnginePCRE();

    EnginePCRE(const EnginePCRE &) = delete;
    EnginePCRE &operator=(const EnginePCRE &) = delete;

    std::unique_ptr<EngineContext> makeContext() const;

    void scan(const char *data, unsigned int len, unsigned int id,
              ResultEntry &result, EngineContext &ectx) const;

    void scan_rlit(const char *data, unsigned int len, unsigned int id,
                   ResultEntry &result, EngineContext &ectx) const {
        scan(data, len, id, result, ectx);
    }

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
    std::vector<std::unique_ptr<PcreDB>> dbs;

    CompilePCREStats compile_stats;

    int capture_cnt;
};

std::unique_ptr<EnginePCRE>
buildEnginePcre(const ExpressionMap &expressions, const std::string &name,
                const std::string &sigs_name);

#endif // ENGINEPCRE_H
