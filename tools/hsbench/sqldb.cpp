/*
 * Copyright (c) 2017, Intel Corporation
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

#include "common.h"
#include "sqldb.h"
#include "ue2common.h"

#include <cassert>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <sqlite3.h>

using namespace std;

namespace {

static
sqlite3 *initDB(const string &filename) {
    sqlite3 *db;
    int status;
    status = sqlite3_open_v2(filename.c_str(), &db,
                        SQLITE_OPEN_CREATE | SQLITE_OPEN_READWRITE, nullptr);

    if (status != SQLITE_OK) {
        ostringstream oss;
        oss << "Unable to open database '" << filename
            << "': " << sqlite3_errmsg(db);
        status = sqlite3_close(db);
        assert(status == SQLITE_OK);
        throw SqlFailure(oss.str());
    }

    // create tables
    static const string c("CREATE TABLE Compile ("
                          "id INTEGER PRIMARY KEY,"
                          "sigsName TEXT, "
                          "signatures TEXT, "
                          "dbInfo TEXT, "
                          "exprCount INTEGER, "
                          "dbSize INTEGER,"
                          "crc TEXT, "
                          "streaming TEXT, "
                          "streamSize INTEGER, "
                          "scratchSize INTEGER, "
                          "compileSecs DOUBLE, "
                          "peakMemory INTEGER"
                          ");");

    static const string s("CREATE TABLE Scan (id INTEGER PRIMARY KEY,"
                          "corpusFile TEXT, scan_id INTEGER, "
                          "totalSecs DOUBLE, bytesPerRun INTEGER, "
                          "blockSize INTEGER, blockCount INTEGER, "
                          "totalBytes INTEGER, totalBlocks INTEGER, "
                          "matchesPerRun INTEGER, "
                          "matchRate DOUBLE, overallTput DOUBLE);");

    static const string sr(
        "CREATE TABLE ScanResults ( id INTEGER PRIMARY KEY, "
        "scan_id INTEGER, thread INTEGER, scan INTEGER, throughput DOUBLE );");

    static const string create_query = c + s + sr;

    sqlite3_stmt *statement;
    const char *pzTail = create_query.c_str();

    while (strlen(pzTail)) {
        status =
            sqlite3_prepare(db, pzTail, strlen(pzTail), &statement, &pzTail);
        if (status != SQLITE_OK) {
            goto fail;
        }
        status = sqlite3_step(statement);
        if (status != SQLITE_DONE && status != SQLITE_ROW) {
            goto fail;
        }
        status = sqlite3_finalize(statement);
        if (status != SQLITE_OK) {
            goto fail;
        }
    }

    return db;

fail:
    ostringstream oss;
    oss << "Unable to create tables: " << sqlite3_errmsg(db);
    status = sqlite3_close(db);
    assert(status == SQLITE_OK);
    throw SqlFailure(oss.str());
}
} // namespace

SqlDB::~SqlDB() {
    if (db) {
        sqlite3_close(db);
    }
    db = nullptr;
}

void SqlDB::open(const string &filename) {
    if (!ifstream(filename)) {
        // file doesn't exist, go set up some tables
        db = initDB(filename);
    } else {
        int status;
        status = sqlite3_open_v2(filename.c_str(), &db, SQLITE_OPEN_READWRITE,
                                 nullptr);

        if (status != SQLITE_OK) {
            ostringstream oss;
            oss << "Unable to open database '" << filename
                << "': " << sqlite3_errmsg(db);
            throw SqlFailure(oss.str());
        }
    }

    exec("PRAGMA synchronous = off;");
    exec("PRAGMA encoding = 'UTF-8';");
}

void SqlDB::exec(const string &query) {
    assert(db);
    int status;
    status = sqlite3_exec(db, query.c_str(), nullptr, nullptr, nullptr);
    if (status != SQLITE_OK) {
        ostringstream oss;
        oss << "Unable to run sqlite query: " << sqlite3_errmsg(db);
        sqlite3_close(db);
        throw SqlFailure(oss.str());
    }
}

u64a SqlDB::lastRowId() {
    assert(db);
    return sqlite3_last_insert_rowid(db);
}
