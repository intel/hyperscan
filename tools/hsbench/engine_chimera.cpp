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

#include "config.h"

#include "ExpressionParser.h"
#include "common.h"
#include "engine_chimera.h"
#include "expressions.h"
#include "heapstats.h"
#include "sqldb.h"
#include "timer.h"

#include "chimera/ch_database.h"

#include "util/make_unique.h"

using namespace std;

EngineCHContext::EngineCHContext(const ch_database_t *db) {
    ch_alloc_scratch(db, &scratch);
    assert(scratch);
}

EngineCHContext::~EngineCHContext() {
    ch_free_scratch(scratch);
}

namespace /* anonymous */ {

/** Scan context structure passed to the onMatch callback function. */
struct ScanCHContext {
    ScanCHContext(unsigned id_in, ResultEntry &result_in)
        : id(id_in), result(result_in) {}
    unsigned id;
    ResultEntry &result;
};

} // namespace

/**
 * Callback function called for every match that Chimera produces, used when
 * "echo matches" is off.
 */
static
int HS_CDECL onMatch(unsigned int, unsigned long long, unsigned long long,
                     unsigned int, unsigned int, const ch_capture_t *,
                     void *ctx) {
    ScanCHContext *sc = static_cast<ScanCHContext *>(ctx);
    assert(sc);
    sc->result.matches++;

    return 0;
}

/**
 * Callback function called for every match that Chimera produces when "echo
 * matches" is enabled.
 */
static
int HS_CDECL onMatchEcho(unsigned int id, unsigned long long,
                         unsigned long long to, unsigned int, unsigned int,
                         const ch_capture_t *, void *ctx) {
    ScanCHContext *sc = static_cast<ScanCHContext *>(ctx);
    assert(sc);
    sc->result.matches++;

    printf("Match @%u:%llu for %u\n", sc->id, to, id);

    return 0;
}

EngineChimera::EngineChimera(ch_database_t *db_in, const CompileCHStats &cs)
    : db(db_in), compile_stats(cs) {
    assert(db);
}

EngineChimera::~EngineChimera() {
    ch_free_database(db);
}

unique_ptr<EngineContext> EngineChimera::makeContext() const {
    return ue2::make_unique<EngineCHContext>(db);
}

void EngineChimera::scan(const char *data, unsigned int len, unsigned int id,
                         ResultEntry &result, EngineContext &ectx) const {
    assert(data);

    auto &ctx = static_cast<EngineCHContext &>(ectx);
    ScanCHContext sc(id, result);
    auto callback = echo_matches ? onMatchEcho : onMatch;
    ch_error_t rv = ch_scan(db, data, len, 0, ctx.scratch, callback, nullptr,
                            &sc);

    if (rv != CH_SUCCESS) {
        printf("Fatal error: ch_scan returned error %d\n", rv);
        abort();
    }
}

// vectoring scan
void EngineChimera::scan_vectored(UNUSED const char *const *data,
                                  UNUSED const unsigned int *len,
                                  UNUSED unsigned int count,
                                  UNUSED unsigned int streamId,
                                  UNUSED ResultEntry &result,
                                  UNUSED EngineContext &ectx) const {
    printf("Hybrid matcher can't support vectored mode.\n");
    abort();
}

unique_ptr<EngineStream> EngineChimera::streamOpen(UNUSED EngineContext &ectx,
                                                   UNUSED unsigned id) const {
    printf("Hybrid matcher can't stream.\n");
    abort();
}

void EngineChimera::streamClose(UNUSED unique_ptr<EngineStream> stream,
                                UNUSED ResultEntry &result) const {
    printf("Hybrid matcher can't stream.\n");
    abort();
}

void EngineChimera::streamScan(UNUSED EngineStream &stream,
                               UNUSED const char *data,
                               UNUSED unsigned len, UNUSED unsigned id,
                               UNUSED ResultEntry &result) const {
    printf("Hybrid matcher can't stream.\n");
    abort();
}

void EngineChimera::streamCompressExpand(UNUSED EngineStream &stream,
                                         UNUSED vector<char> &temp) const {
    printf("Hybrid matcher can't stream.\n");
    abort();
}

void EngineChimera::printStats() const {
    // Output summary information.
    if (!compile_stats.sigs_name.empty()) {
        printf("Signature set:        %s\n", compile_stats.sigs_name.c_str());
    }
    printf("Signatures:        %s\n", compile_stats.signatures.c_str());
    printf("Chimera info:      %s\n", compile_stats.db_info.c_str());

    printf("Expression count:  %zu\n", compile_stats.expressionCount);
    printf("Bytecode size:     %zu bytes\n", compile_stats.compiledSize);
    printf("Scratch size:      %zu bytes\n", compile_stats.scratchSize);
    printf("Compile time:      %0.3Lf seconds\n", compile_stats.compileSecs);
    printf("Peak heap usage:   %u bytes\n", compile_stats.peakMemorySize);

}

void EngineChimera::printCsvStats() const {
    printf(",\"%s\"", compile_stats.signatures.c_str());
    printf(",\"%zu\"", compile_stats.expressionCount);
    printf(",\"%zu\"", compile_stats.compiledSize);
    printf(",\"%zu\"", compile_stats.scratchSize);
    printf(",\"%0.3Lf\"", compile_stats.compileSecs);
    printf(",\"%u\"", compile_stats.peakMemorySize);
}

void EngineChimera::sqlStats(SqlDB &sqldb) const {
    static const string Q =
        "INSERT INTO Compile ("
            "sigsName, signatures, dbInfo, exprCount, dbSize,"
            "scratchSize, compileSecs, peakMemory) "
        "VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8)";

    sqldb.insert_all(Q, compile_stats.sigs_name, compile_stats.signatures,
                     compile_stats.db_info, compile_stats.expressionCount,
                     compile_stats.compiledSize,
                     compile_stats.scratchSize, compile_stats.compileSecs,
                     compile_stats.peakMemorySize);
}

unique_ptr<EngineChimera>
buildEngineChimera(const ExpressionMap &expressions, const string &name,
                   const string &sigs_name) {
    if (expressions.empty()) {
        assert(0);
        return nullptr;
    }

    long double compileSecs = 0.0;
    size_t compiledSize = 0.0;
    size_t scratchSize = 0;
    unsigned int peakMemorySize = 0;
    string db_info;

    ch_database_t *db;
    ch_error_t err;

    const unsigned int count = expressions.size();

    vector<string> exprs;
    vector<unsigned int> flags, ids;
    vector<hs_expr_ext> ext;

    for (const auto &m : expressions) {
        string expr;
        unsigned int f = 0;
        hs_expr_ext extparam; // unused
        extparam.flags = 0;
        if (!readExpression(m.second, expr, &f, &extparam)) {
            printf("Error parsing PCRE: %s (id %u)\n", m.second.c_str(),
                   m.first);
            return nullptr;
        }

        if (extparam.flags) {
            printf("Error parsing PCRE with extended flags: %s (id %u)\n",
                   m.second.c_str(), m.first);
            return nullptr;
        }
        exprs.push_back(std::move(expr));
        ids.push_back(m.first);
        flags.push_back(f);
    }

    // Our compiler takes an array of plain ol' C strings.
    vector<const char *> patterns(count);
    for (unsigned int i = 0; i < count; i++) {
        patterns[i] = exprs[i].c_str();
    }

    Timer timer;
    timer.start();

    // Capture groups by default
    unsigned int mode = CH_MODE_GROUPS;
    ch_compile_error_t *compile_err;
    err = ch_compile_multi(patterns.data(), flags.data(), ids.data(),
                           count, mode, nullptr, &db, &compile_err);

    timer.complete();
    compileSecs = timer.seconds();
    peakMemorySize = getPeakHeap();

    if (err == CH_COMPILER_ERROR) {
        if (compile_err->expression >= 0) {
            printf("Compile error for signature #%u: %s\n",
                   compile_err->expression, compile_err->message);
        } else {
            printf("Compile error: %s\n", compile_err->message);
        }
        ch_free_compile_error(compile_err);
        return nullptr;
    }

    err = ch_database_size(db, &compiledSize);
    if (err != CH_SUCCESS) {
        return nullptr;
    }
    assert(compiledSize > 0);

    char *info;
    err = ch_database_info(db, &info);
    if (err != CH_SUCCESS) {
        return nullptr;
    } else {
        db_info = string(info);
        free(info);
    }

    // Allocate scratch temporarily to find its size: this is a good test
    // anyway.
    ch_scratch_t *scratch = nullptr;
    err = ch_alloc_scratch(db, &scratch);
    if (err != HS_SUCCESS) {
        return nullptr;
    }

    err = ch_scratch_size(scratch, &scratchSize);
    if (err != CH_SUCCESS) {
        return nullptr;
    }
    ch_free_scratch(scratch);

    // Collect summary information.
    CompileCHStats cs;
    cs.sigs_name = sigs_name;
    if (!sigs_name.empty()) {
        const auto pos = name.find_last_of('/');
        cs.signatures = name.substr(pos + 1);
    } else {
        cs.signatures = name;
    }
    cs.db_info = std::move(db_info);
    cs.expressionCount = expressions.size();
    cs.compiledSize = compiledSize;
    cs.scratchSize = scratchSize;
    cs.compileSecs = compileSecs;
    cs.peakMemorySize = peakMemorySize;

    return ue2::make_unique<EngineChimera>(db, std::move(cs));
}
