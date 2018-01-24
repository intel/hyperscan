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
#ifndef SQLDB_H_
#define SQLDB_H_

#include "ue2common.h"

#include "common.h"
#include "sqldb_bind.h"

#include <iostream>
#include <string>

#include <sqlite3.h>

class SqlDB {
public:
    SqlDB() : db(nullptr) {};
    ~SqlDB();
    void open(const std::string &filename);
    void exec(const std::string &query);
    u64a lastRowId();

    template <typename... Args>
    void insert_all(const std::string &query, Args&&... args) {
        sqlite3_stmt *stmt;
        const char *tail;

        int rc = sqlite3_prepare(db, query.c_str(), query.size(), &stmt, &tail);
        if (rc != SQLITE_OK) {
            std::ostringstream oss;
            oss << "Unable to prepare query: " << sqlite3_errmsg(db);
            throw SqlFailure(oss.str());
        }

        // only one statement per function call
        assert(strlen(tail) == 0);

        // perform templated binds to this statement
        ue2_sqlite::bind_args(stmt, 1, args...);

        rc = sqlite3_step(stmt);
        if (rc != SQLITE_DONE) {
            std::ostringstream oss;
            oss << "Unable to run insert: " << sqlite3_errmsg(db);
            throw SqlFailure(oss.str());
        }

        rc = sqlite3_finalize(stmt);
        if (rc != SQLITE_OK) {
            std::ostringstream oss;
            oss << "Unable to finalize statement: " << sqlite3_errmsg(db);
            throw SqlFailure(oss.str());
        }
    }

private:
    sqlite3 *db;
};

#endif /* SQLDB_H_ */
