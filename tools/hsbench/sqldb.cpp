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
                          "hmac TEXT, "
                          "streaming TEXT, "
                          "streamSize INTEGER, "
                          "scratchSize INTEGER, "
                          "compileSecs DOUBLE, "
                          "peakMemory INTEGER"
                          ");");
#ifdef ENGINE_UPDATE_ON
    static const string s("CREATE TABLE Scan (id INTEGER PRIMARY KEY,"
                          "corpusFile TEXT, scan_id INTEGER, "
                          "totalSecs DOUBLE, bytesPerRun INTEGER, "
                          "blockSize INTEGER, blockCount INTEGER, "
                          "totalBytes INTEGER, totalBlocks INTEGER, "
                          "matchesPerRun INTEGER, "
                          "matchRate DOUBLE, overallTput DOUBLE,engType TEXT);");
#else
    static const string s("CREATE TABLE Scan (id INTEGER PRIMARY KEY,"
                          "corpusFile TEXT, scan_id INTEGER, "
                          "totalSecs DOUBLE, bytesPerRun INTEGER, "
                          "blockSize INTEGER, blockCount INTEGER, "
                          "totalBytes INTEGER, totalBlocks INTEGER, "
                          "matchesPerRun INTEGER, "
                          "matchRate DOUBLE, overallTput DOUBLE);");
#endif
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
