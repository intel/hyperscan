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
#ifndef SQLDB_BIND_H_
#define SQLDB_BIND_H_

#include "ue2common.h"

#include <iostream>
#include <sstream>
#include <string>

#include <sqlite3.h>

namespace ue2_sqlite {

inline
int bind_impl(sqlite3_stmt *stmt, int param, const unsigned long &val) {
    return sqlite3_bind_int64(stmt, param, val);
}

inline
int bind_impl(sqlite3_stmt *stmt, int param, const unsigned int &val) {
    return sqlite3_bind_int(stmt, param, val);
}

inline
int bind_impl(sqlite3_stmt *stmt, int param, const u64a &val) {
    return sqlite3_bind_int64(stmt, param, val);
}

inline
int bind_impl(sqlite3_stmt *stmt, int param, const double &val) {
    return sqlite3_bind_double(stmt, param, val);
}

inline
int bind_impl(sqlite3_stmt *stmt, int param, const long double &val) {
    return sqlite3_bind_double(stmt, param, val);
}

inline
int bind_impl(sqlite3_stmt *stmt, int param, const std::string &val) {
    return sqlite3_bind_text(stmt, param, val.c_str(), val.size(),
                             SQLITE_TRANSIENT);
}

template<typename T>
void bind_args(sqlite3_stmt *stmt, int param, T obj) {
    int rc = bind_impl(stmt, param, obj);
    if (rc != SQLITE_OK) {
        std::ostringstream oss;
        oss << "SQL value bind failed for param #: " << param;
        throw SqlFailure(oss.str());
    }
}

template<typename T, typename... Args>
void bind_args(sqlite3_stmt *stmt, int param, T obj, Args&&... args) {
    bind_args(stmt, param, obj);
    bind_args(stmt, param + 1, args...);
}

} // namespace

#endif /* SQLDB_BIND_H_ */
