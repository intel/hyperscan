/*
 * Copyright (c) 2016-2019, Intel Corporation
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

#include "config.h"

#include "ExpressionParser.h"
#include "common.h"
#include "engine_hyperscan.h"
#include "expressions.h"
#include "heapstats.h"
#include "huge.h"
#include "sqldb.h"
#include "timer.h"

#include "database.h"
#include "hs_compile.h"
#include "hs_internal.h"
#include "hs_runtime.h"
#include "util/database_util.h"
#include "util/make_unique.h"

#include <cassert>
#include <cstring>
#include <functional>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <boost/crc.hpp>

using namespace std;

EngineHSContext::EngineHSContext(const hs_database_t *db) {
    hs_alloc_scratch(db, &scratch);
    assert(scratch);
}

EngineHSContext::~EngineHSContext() {
    hs_free_scratch(scratch);
}

EngineHSStream::~EngineHSStream() { }

namespace /* anonymous */ {

/** Scan context structure passed to the onMatch callback function. */
struct ScanHSContext {
    ScanHSContext(unsigned id_in, ResultEntry &result_in,
                const EngineStream *stream_in)
        : id(id_in), result(result_in), stream(stream_in) {}
    unsigned id;
    ResultEntry &result;
    const EngineStream *stream; // nullptr except in streaming mode.
};

} // namespace

/**
 * Callback function called for every match that Hyperscan produces, used when
 * "echo matches" is off.
 */
static
int HS_CDECL onMatch(unsigned int, unsigned long long,
                     unsigned long long, unsigned int, void *ctx) {
    ScanHSContext *sc = static_cast<ScanHSContext *>(ctx);
    assert(sc);
    sc->result.matches++;

    return 0;
}

/**
 * Callback function called for every match that Hyperscan produces when "echo
 * matches" is enabled.
 */
static
int HS_CDECL onMatchEcho(unsigned int id, unsigned long long,
                         unsigned long long to, unsigned int, void *ctx) {
    ScanHSContext *sc = static_cast<ScanHSContext *>(ctx);
    assert(sc);
    sc->result.matches++;

    if (sc->stream) {
        printf("Match @%u:%u:%llu for %u\n", sc->stream->sn, sc->id, to, id);
    } else {
        printf("Match @%u:%llu for %u\n", sc->id, to, id);
    }

    return 0;
}

EngineHyperscan::EngineHyperscan(hs_database_t *db_in, CompileHSStats cs)
    : db(db_in), compile_stats(std::move(cs)) {
    assert(db);
}

EngineHyperscan::~EngineHyperscan() {
    release_huge(db);
}

unique_ptr<EngineContext> EngineHyperscan::makeContext() const {
    return ue2::make_unique<EngineHSContext>(db);
}

void EngineHyperscan::scan(const char *data, unsigned int len, unsigned int id,
                           ResultEntry &result, EngineContext &ectx) const {
    assert(data);

    EngineHSContext &ctx = static_cast<EngineHSContext &>(ectx);
    ScanHSContext sc(id, result, nullptr);
    auto callback = echo_matches ? onMatchEcho : onMatch;
    hs_error_t rv = hs_scan(db, data, len, 0, ctx.scratch, callback, &sc);

    if (rv != HS_SUCCESS) {
        printf("Fatal error: hs_scan returned error %d\n", rv);
        abort();
    }
}

void EngineHyperscan::scan_vectored(const char *const *data,
                                    const unsigned int *len, unsigned int count,
                                    unsigned streamId, ResultEntry &result,
                                    EngineContext &ectx) const {
    assert(data);
    assert(len);

    EngineHSContext &ctx = static_cast<EngineHSContext &>(ectx);
    ScanHSContext sc(streamId, result, nullptr);
    auto callback = echo_matches ? onMatchEcho : onMatch;
    hs_error_t rv =
        hs_scan_vector(db, data, len, count, 0, ctx.scratch, callback, &sc);

    if (rv != HS_SUCCESS) {
        printf("Fatal error: hs_scan_vector returned error %d\n", rv);
        abort();
    }
}

unique_ptr<EngineStream> EngineHyperscan::streamOpen(EngineContext &ectx,
                                                     unsigned streamId) const {
    EngineHSContext &ctx = static_cast<EngineHSContext &>(ectx);
    auto stream = ue2::make_unique<EngineHSStream>();
    stream->ctx = &ctx;

    hs_open_stream(db, 0, &stream->id);
    if (!stream->id) {
        // an error occurred, propagate to caller
        return nullptr;
    }
    stream->sn = streamId;
    return move(stream);
}

void EngineHyperscan::streamClose(unique_ptr<EngineStream> stream,
                                  ResultEntry &result) const {
    assert(stream);

    auto &s = static_cast<EngineHSStream &>(*stream);
    EngineContext &ectx = *s.ctx;
    EngineHSContext &ctx = static_cast<EngineHSContext &>(ectx);

    ScanHSContext sc(0, result, &s);
    auto callback = echo_matches ? onMatchEcho : onMatch;

    assert(s.id);
    hs_close_stream(s.id, ctx.scratch, callback, &sc);
    s.id = nullptr;
}

void EngineHyperscan::streamScan(EngineStream &stream, const char *data,
                                 unsigned len, unsigned id,
                                 ResultEntry &result) const {
    assert(data);

    auto &s = static_cast<EngineHSStream &>(stream);
    EngineHSContext &ctx = *s.ctx;

    ScanHSContext sc(id, result, &s);
    auto callback = echo_matches ? onMatchEcho : onMatch;
    hs_error_t rv =
        hs_scan_stream(s.id, data, len, 0, ctx.scratch, callback, &sc);

    if (rv != HS_SUCCESS) {
        printf("Fatal error: hs_scan_stream returned error %d\n", rv);
        abort();
    }
}

void EngineHyperscan::streamCompressExpand(EngineStream &stream,
                                           vector<char> &temp) const {
    size_t used = 0;
    auto &s = static_cast<EngineHSStream &>(stream);
    hs_error_t err = hs_compress_stream(s.id, temp.data(), temp.size(),
                                        &used);
    if (err == HS_INSUFFICIENT_SPACE) {
        temp.resize(used);
        err = hs_compress_stream(s.id, temp.data(), temp.size(), &used);
    }

    if (err != HS_SUCCESS) {
        printf("Fatal error: hs_compress_stream returned error %d\n", err);
        abort();
    }

    if (printCompressSize) {
        printf("stream %u: compressed to %zu\n", s.sn, used);
    }

    err = hs_reset_and_expand_stream(s.id, temp.data(), temp.size(),
                                     nullptr, nullptr, nullptr);

    if (err != HS_SUCCESS) {
        printf("Fatal error: hs_reset_and expand_stream returned error %d\n",
               err);
        abort();
    }
}

void EngineHyperscan::printStats() const {
    // Output summary information.
    if (!compile_stats.sigs_name.empty()) {
        printf("Signature set:        %s\n", compile_stats.sigs_name.c_str());
    }
    printf("Signatures:        %s\n", compile_stats.signatures.c_str());
    printf("Hyperscan info:    %s\n", compile_stats.db_info.c_str());
#ifndef _WIN32
    printf("Expression count:  %'zu\n", compile_stats.expressionCount);
    printf("Bytecode size:     %'zu bytes\n", compile_stats.compiledSize);
#else
    printf("Expression count:  %zu\n", compile_stats.expressionCount);
    printf("Bytecode size:     %zu bytes\n", compile_stats.compiledSize);
#endif
    printf("Database CRC:      0x%x\n", compile_stats.crc32);
    if (compile_stats.streaming) {
#ifndef _WIN32
        printf("Stream state size: %'zu bytes\n", compile_stats.streamSize);
#else
        printf("Stream state size: %zu bytes\n", compile_stats.streamSize);
#endif
    }
#ifndef _WIN32
    printf("Scratch size:      %'zu bytes\n", compile_stats.scratchSize);
    printf("Compile time:      %'0.3Lf seconds\n", compile_stats.compileSecs);
    printf("Peak heap usage:   %'u bytes\n", compile_stats.peakMemorySize);
#else
    printf("Scratch size:      %zu bytes\n", compile_stats.scratchSize);
    printf("Compile time:      %0.3Lf seconds\n", compile_stats.compileSecs);
    printf("Peak heap usage:   %u bytes\n", compile_stats.peakMemorySize);
#endif
}

void EngineHyperscan::sqlStats(SqlDB &sqldb) const {
    ostringstream crc;
    crc << "0x" << hex << compile_stats.crc32;

    static const std::string Q =
        "INSERT INTO Compile ("
            "sigsName, signatures, dbInfo, exprCount, dbSize, crc, streaming,"
            "streamSize, scratchSize, compileSecs, peakMemory) "
        "VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10, ?11)";

    sqldb.insert_all(Q, compile_stats.sigs_name, compile_stats.signatures,
                     compile_stats.db_info, compile_stats.expressionCount,
                     compile_stats.compiledSize, crc.str(),
                     compile_stats.streaming ? "TRUE" : "FALSE",
                     compile_stats.streamSize, compile_stats.scratchSize,
                     compile_stats.compileSecs, compile_stats.peakMemorySize);
}


static
unsigned makeModeFlags(ScanMode scan_mode) {
    switch (scan_mode) {
    case ScanMode::BLOCK:
        return HS_MODE_BLOCK;
    case ScanMode::STREAMING:
        return HS_MODE_STREAM;
    case ScanMode::VECTORED:
        return HS_MODE_VECTORED;
    }
    assert(0);
    return HS_MODE_STREAM;
}

/**
 * Hash the settings used to compile a database, returning a string that can be
 * used as a filename.
 */
static
string dbSettingsHash(const string &filename, u32 mode) {
    ostringstream info_oss;

    info_oss << filename.c_str() << ' ';
    info_oss << mode << ' ';

    string info = info_oss.str();

    boost::crc_32_type crc;

    crc.process_bytes(info.data(), info.length());

    // return STL string with printable version of digest
    ostringstream oss;
    oss << hex << setw(8) << setfill('0') << crc.checksum() << dec;

    return oss.str();
}

static
string dbFilename(const std::string &name, unsigned mode) {
    ostringstream oss;
    oss << serializePath << '/' << dbSettingsHash(name, mode) << ".db";
    return oss.str();
}

std::unique_ptr<EngineHyperscan>
buildEngineHyperscan(const ExpressionMap &expressions, ScanMode scan_mode,
                     const std::string &name, const std::string &sigs_name,
                     UNUSED const ue2::Grey &grey) {
    if (expressions.empty()) {
        assert(0);
        return nullptr;
    }

    long double compileSecs = 0.0;
    size_t compiledSize = 0.0;
    size_t streamSize = 0;
    size_t scratchSize = 0;
    unsigned int peakMemorySize = 0;
    std::string db_info;

    unsigned int mode = makeModeFlags(scan_mode);

    hs_database_t *db;
    hs_error_t err;

    if (loadDatabases) {
        db = loadDatabase(dbFilename(name, mode).c_str());
        if (!db) {
            return nullptr;
        }
    } else {
        const unsigned int count = expressions.size();

        vector<string> exprs;
        vector<unsigned int> flags, ids;
        vector<hs_expr_ext> ext;

        for (const auto &m : expressions) {
            string expr;
            unsigned int f = 0;
            hs_expr_ext extparam;
            extparam.flags = 0;
            if (!readExpression(m.second, expr, &f, &extparam)) {
                printf("Error parsing PCRE: %s (id %u)\n", m.second.c_str(),
                       m.first);
                return nullptr;
            }
            if (forceEditDistance) {
                extparam.flags |= HS_EXT_FLAG_EDIT_DISTANCE;
                extparam.edit_distance = editDistance;
            }

            exprs.push_back(expr);
            ids.push_back(m.first);
            flags.push_back(f);
            ext.push_back(extparam);
        }

        unsigned full_mode = mode;
        if (mode == HS_MODE_STREAM) {
            full_mode |= somPrecisionMode;
        }

        // Our compiler takes an array of plain ol' C strings.
        vector<const char *> patterns(count);
        for (unsigned int i = 0; i < count; i++) {
            patterns[i] = exprs[i].c_str();
        }

        // Extended parameters are passed as pointers to hs_expr_ext structures.
        vector<const hs_expr_ext *> ext_ptr(count);
        for (unsigned int i = 0; i < count; i++) {
            ext_ptr[i] = &ext[i];
        }

        hs_compile_error_t *compile_err;
        Timer timer;

#ifndef RELEASE_BUILD
        if (useLiteralApi) {
            // Pattern length computation should be done before timer start.
            vector<size_t> lens(count);
            for (unsigned int i = 0; i < count; i++) {
                lens[i] = strlen(patterns[i]);
            }
            timer.start();
            err = hs_compile_lit_multi_int(patterns.data(), flags.data(),
                                           ids.data(), ext_ptr.data(),
                                           lens.data(), count, full_mode,
                                           nullptr, &db, &compile_err, grey);
            timer.complete();
        } else {
            timer.start();
            err = hs_compile_multi_int(patterns.data(), flags.data(),
                                       ids.data(), ext_ptr.data(), count,
                                       full_mode, nullptr, &db, &compile_err,
                                       grey);
            timer.complete();
        }
#else
        if (useLiteralApi) {
            // Pattern length computation should be done before timer start.
            vector<size_t> lens(count);
            for (unsigned int i = 0; i < count; i++) {
                lens[i] = strlen(patterns[i]);
            }
            timer.start();
            err = hs_compile_lit_multi(patterns.data(), flags.data(),
                                       ids.data(), lens.data(), count,
                                       full_mode, nullptr, &db, &compile_err);
            timer.complete();
        } else {
            timer.start();
            err = hs_compile_ext_multi(patterns.data(), flags.data(),
                                       ids.data(), ext_ptr.data(), count,
                                       full_mode, nullptr, &db, &compile_err);
            timer.complete();
        }
#endif

        compileSecs = timer.seconds();
        peakMemorySize = getPeakHeap();

        if (err == HS_COMPILER_ERROR) {
            if (compile_err->expression >= 0) {
                printf("Compile error for signature #%u: %s\n",
                       compile_err->expression, compile_err->message);
            } else {
                printf("Compile error: %s\n", compile_err->message);
            }
            hs_free_compile_error(compile_err);
            return nullptr;
        }
    }

    // copy the db into huge pages (where available) to reduce TLB pressure
    db = get_huge(db);
    if (!db) {
        return nullptr;
    }

    err = hs_database_size(db, &compiledSize);
    if (err != HS_SUCCESS) {
        return nullptr;
    }
    assert(compiledSize > 0);

    if (saveDatabases) {
        saveDatabase(db, dbFilename(name, mode).c_str());
    }

    if (mode & HS_MODE_STREAM) {
        err = hs_stream_size(db, &streamSize);
        if (err != HS_SUCCESS) {
            return nullptr;
        }
    } else {
        streamSize = 0;
    }

    char *info;
    err = hs_database_info(db, &info);
    if (err != HS_SUCCESS) {
        return nullptr;
    } else {
        db_info = string(info);
        free(info);
    }

    // Allocate scratch temporarily to find its size: this is a good test
    // anyway.
    hs_scratch_t *scratch = nullptr;
    err = hs_alloc_scratch(db, &scratch);
    if (err != HS_SUCCESS) {
        return nullptr;
    }

    err = hs_scratch_size(scratch, &scratchSize);
    if (err != HS_SUCCESS) {
        return nullptr;
    }
    hs_free_scratch(scratch);

    // Collect summary information.
    CompileHSStats cs;
    cs.sigs_name = sigs_name;
    if (!sigs_name.empty()) {
        const auto pos = name.find_last_of('/');
        cs.signatures = name.substr(pos + 1);
    } else {
        cs.signatures = name;
    }
    cs.db_info = db_info;
    cs.expressionCount = expressions.size();
    cs.compiledSize = compiledSize;
    cs.crc32 = db->crc32;
    cs.streaming = mode & HS_MODE_STREAM;
    cs.streamSize = streamSize;
    cs.scratchSize = scratchSize;
    cs.compileSecs = compileSecs;
    cs.peakMemorySize = peakMemorySize;

    return ue2::make_unique<EngineHyperscan>(db, std::move(cs));
}
