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

#ifdef _WIN32
#define PCRE_STATIC
#endif
#include "config.h"

#include "common.h"
#include "engine_pcre.h"
#include "heapstats.h"
#include "huge.h"
#include "sqldb.h"
#include "timer.h"

#include "util/make_unique.h"
#include "util/unicode_def.h"

#include <algorithm>

using namespace std;

EnginePCREContext::EnginePCREContext(int capture_cnt) {
    ovec = (int *)malloc((capture_cnt + 1)* sizeof(int) * 3);
}

EnginePCREContext::~EnginePCREContext() {
    free(ovec);
}

namespace /* anonymous */ {

/** Scan context structure passed to the onMatch callback function. */
struct ScanPCREContext {
    ScanPCREContext(unsigned id_in, ResultEntry &result_in)
        : id(id_in), result(result_in) {}
    unsigned id;
    ResultEntry &result;
};

} // namespace

/**
 * Function called for every match that PCRE produces, used when
 * "echo matches" is off.
 */
static
int onMatch(ScanPCREContext *sc) {
    assert(sc);
    sc->result.matches++;

    return 0;
}

/**
 * Function called for every match that PCRE produces when "echo
 * matches" is enabled.
 */
static
int onMatchEcho(unsigned int id, unsigned long long, unsigned long long to,
                ScanPCREContext *sc) {
    assert(sc);
    sc->result.matches++;

    printf("Match @%u:%llu for %u\n", sc->id, to, id);

    return 0;
}

EnginePCRE::EnginePCRE(vector<unique_ptr<PcreDB>> dbs_in, CompilePCREStats cs,
                       int capture_cnt_in)
    : dbs(move(dbs_in)), compile_stats(move(cs)),
      capture_cnt(capture_cnt_in) {}

EnginePCRE::~EnginePCRE() {
    for (auto &pcreDB : dbs) {
        free(pcreDB->extra);
        free(pcreDB->db);
    }
}

unique_ptr<EngineContext> EnginePCRE::makeContext() const {
    return ue2::make_unique<EnginePCREContext>(capture_cnt);
}

void EnginePCRE::scan(const char *data, unsigned int len, unsigned int id,
                      ResultEntry &result, EngineContext &ectx) const {
    assert(data);

    ScanPCREContext sc(id, result);
    auto &ctx = static_cast<EnginePCREContext &>(ectx);
    int *ovec = ctx.ovec;
    int ovec_size = (capture_cnt + 1) * 3;
    for (const auto &pcreDB : dbs) {
        int startoffset = 0;
        bool utf8 = pcreDB->utf8;
        bool highlander = pcreDB->highlander;

        int flags = 0;
        int ret;
        do {
            ret = pcre_exec(pcreDB->db, pcreDB->extra, data, len,
                            startoffset, flags, ovec, ovec_size);
            if (ret <= PCRE_ERROR_NOMATCH) {
                break;
            }

            int from = ovec[0];
            int to = ovec[1];
            assert(from <= to);

            if (echo_matches) {
                onMatchEcho(pcreDB->id, from, to, &sc);
            } else {
                onMatch(&sc);
            }

            // If we only wanted a single match, we're done.
            if (highlander) {
                break;
            }

            // Next scan starts at the first codepoint after the match. It's
            // possible that we have a vacuous match, in which case we must step
            // past it to ensure that we always progress.
            if (from != to) {
                startoffset = to;
            } else if (utf8) {
                startoffset = to + 1;
                while (startoffset < (int)len &&
                       ((data[startoffset] & 0xc0) == UTF_CONT_BYTE_HEADER)) {
                    ++startoffset;
                }
            } else {
                startoffset = to + 1;
            }
        } while (startoffset <= (int)len);

        if (ret < PCRE_ERROR_NOMATCH) {
            printf("Fatal error: pcre returned error %d\n", ret);
            abort();
        }
    }
}

// vectoring scan
void EnginePCRE::scan_vectored(UNUSED const char *const *data,
                               UNUSED const unsigned int *len,
                               UNUSED unsigned int count,
                               UNUSED unsigned int streamId,
                               UNUSED ResultEntry &result,
                               UNUSED EngineContext &ectx) const {
    printf("PCRE matcher can't support vectored mode.\n");
    abort();
}

unique_ptr<EngineStream> EnginePCRE::streamOpen(UNUSED EngineContext &ectx,
                                                UNUSED unsigned id) const {
    printf("PCRE matcher can't stream.\n");
    abort();
}

void EnginePCRE::streamClose(UNUSED unique_ptr<EngineStream> stream,
                             UNUSED ResultEntry &result) const {
    printf("PCRE matcher can't stream.\n");
    abort();
}

void EnginePCRE::streamScan(UNUSED EngineStream &stream,
                            UNUSED const char *data,
                            UNUSED unsigned len, UNUSED unsigned id,
                            UNUSED ResultEntry &result) const {
    printf("PCRE matcher can't stream.\n");
    abort();
}

void EnginePCRE::streamCompressExpand(UNUSED EngineStream &stream,
                                      UNUSED vector<char> &temp) const {
    printf("PCRE matcher can't stream.\n");
    abort();
}

void EnginePCRE::printStats() const {
    // Output summary information.
    if (!compile_stats.sigs_name.empty()) {
        printf("Signature set:        %s\n", compile_stats.sigs_name.c_str());
    }
    printf("Signatures:        %s\n", compile_stats.signatures.c_str());
    printf("PCRE info:         %s\n", compile_stats.db_info.c_str());
#ifndef _WIN32
    printf("Expression count:  %'zu\n", compile_stats.expressionCount);
    printf("Bytecode size:     %'zu bytes\n", compile_stats.compiledSize);
    printf("Scratch size:      %'zu bytes\n", compile_stats.scratchSize);
    printf("Compile time:      %'0.3Lf seconds\n", compile_stats.compileSecs);
    printf("Peak heap usage:   %'u bytes\n", compile_stats.peakMemorySize);
#else
    printf("Expression count:  %zu\n", compile_stats.expressionCount);
    printf("Bytecode size:     %zu bytes\n", compile_stats.compiledSize);
    printf("Scratch size:      %zu bytes\n", compile_stats.scratchSize);
    printf("Compile time:      %0.3Lf seconds\n", compile_stats.compileSecs);
    printf("Peak heap usage:   %u bytes\n", compile_stats.peakMemorySize);
#endif
}

void EnginePCRE::sqlStats(SqlDB &sqldb) const {
    ostringstream crc;

    static const string Q =
        "INSERT INTO Compile ("
            "sigsName, signatures, dbInfo, exprCount, dbSize, crc,"
            "scratchSize, compileSecs, peakMemory) "
        "VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9)";

    sqldb.insert_all(Q, compile_stats.sigs_name, compile_stats.signatures,
                     compile_stats.db_info, compile_stats.expressionCount,
                     compile_stats.compiledSize, crc.str(),
                     compile_stats.scratchSize, compile_stats.compileSecs,
                     compile_stats.peakMemorySize);
}

static
bool decodeExprPCRE(string &expr, unsigned *flags, struct PcreDB &db) {
    if (expr[0] != '/') {
        return false;
    }

    size_t end = expr.find_last_of('/');
    if (end == string::npos) {
        return false;
    }
    string strFlags = expr.substr(end + 1, expr.length() - end - 1);

    // strip starting and trailing slashes and the flags
    expr.erase(end, expr.length() - end);
    expr.erase(0, 1);

    // decode the flags
    *flags = 0;
    for (size_t i = 0; i != strFlags.length(); ++i) {
        switch (strFlags[i]) {
            case 's':
                *flags |= PCRE_DOTALL;
                break;
            case 'm':
                *flags |= PCRE_MULTILINE;
                break;
            case 'i':
                *flags |= PCRE_CASELESS;
                break;
            case '8':
                *flags |= PCRE_UTF8;
                db.utf8 = true;
                break;
            case 'W':
                *flags |= PCRE_UCP;
                break;
            case 'H':
                db.highlander = true;
                break;
            default:
                return false;
        }
    }

    return true;
}

unique_ptr<EnginePCRE>
buildEnginePcre(const ExpressionMap &expressions, const string &name,
                const string &sigs_name) {
    if (expressions.empty()) {
        assert(0);
        return nullptr;
    }

    long double compileSecs = 0.0;
    size_t compiledSize = 0.0;
    unsigned int peakMemorySize = 0;
    string db_info("Version: ");
    db_info += string(pcre_version());

    vector<unique_ptr<PcreDB>> dbs;
    int capture_cnt = 0;

    Timer timer;
    timer.start();

    for (const auto &m : expressions) {
        string expr(m.second);
        unsigned int flags = 0;
        auto pcreDB = ue2::make_unique<PcreDB>();
        if (!decodeExprPCRE(expr, &flags, *pcreDB)) {
            printf("Error parsing PCRE: %s (id %u)\n", m.second.c_str(),
                    m.first);
            return nullptr;
        }

        const char *errp;
        int erro;
        pcre *db = pcre_compile(expr.c_str(), flags, &errp, &erro, NULL);

        if (!db) {
            printf("Compile error %s\n", errp);
            return nullptr;
        }

        pcre_extra *extra = pcre_study(db, PCRE_STUDY_JIT_COMPILE, &errp);
        if (errp) {
            printf("PCRE could not be studied: %s\n", errp);
            return nullptr;
        }
        if (!extra) {
            extra = (pcre_extra *)malloc(sizeof(pcre_extra));
        }
        int cap = 0; // PCRE_INFO_CAPTURECOUNT demands an int
        if (pcre_fullinfo(db, extra, PCRE_INFO_CAPTURECOUNT, &cap)) {
            printf("PCRE fullinfo error\n");
            free(extra);
            free(db);
            return nullptr;
        }
        assert(cap >= 0);
        capture_cnt = max(capture_cnt, cap);

        size_t db_size = 0;
        if (pcre_fullinfo(db, extra, PCRE_INFO_SIZE, &db_size)) {
            printf("PCRE fullinfo error\n");
            free(extra);
            free(db);
            return nullptr;
        }

        size_t study_size = 0;
        if (pcre_fullinfo(db, extra, PCRE_INFO_STUDYSIZE,
            &study_size)) {
            printf("PCRE fullinfo error\n");
            free(extra);
            free(db);
            return nullptr;
        }
        compiledSize += db_size + study_size;

        pcreDB->id = m.first;
        pcreDB->db = db;

        extra->flags =
            PCRE_EXTRA_MATCH_LIMIT | PCRE_EXTRA_MATCH_LIMIT_RECURSION;
        extra->match_limit = 10000000;
        extra->match_limit_recursion = 1500;

        pcreDB->extra = extra;
        dbs.push_back(move(pcreDB));
    }

    timer.complete();
    compileSecs = timer.seconds();
    peakMemorySize = getPeakHeap();

    // Collect summary information.
    CompilePCREStats cs;
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
    cs.scratchSize = (capture_cnt  + 1) * sizeof(int) * 3;
    cs.compileSecs = compileSecs;
    cs.peakMemorySize = peakMemorySize;

    return ue2::make_unique<EnginePCRE>(move(dbs), move(cs), capture_cnt);
}
