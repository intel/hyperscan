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

    SqlDB(const SqlDB &) = delete;
    SqlDB &operator=(const SqlDB &) = delete;

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
