/*
 * Copyright (c) 2015-2018, Intel Corporation
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

#ifndef UE2COLLIDER_DATABASEPROXY_H
#define UE2COLLIDER_DATABASEPROXY_H

#include "UltimateTruth.h"

#include <memory>
#include <mutex>
#include <set>
#include <string>

#include <boost/core/noncopyable.hpp>

/**
 * When a compile fails for the first time, we throw this exception so that a
 * compilation error can be reported to the user. Subsequent failures will
 * simply return nullptr rather than throwing this exception.
 */
struct CompileFailed {
public:
    explicit CompileFailed(const std::string &err) : error(err) {}
    std::string error;
};

class DatabaseProxy : boost::noncopyable {
public:
    explicit DatabaseProxy(const std::set<unsigned> &expr_ids)
        : ids(expr_ids) {}

    explicit DatabaseProxy(std::shared_ptr<BaseDB> built_db)
        : db(built_db) {}

    std::shared_ptr<BaseDB> get(const UltimateTruth &ultimate) {
        std::lock_guard<std::mutex> lock(mutex);
        if (failed) {
            // We have previously failed to compile this database.
            return nullptr;
        }
        if (db) {
            return db;
        }

        // Database hasn't been compiled yet.
        std::string error;
        db = ultimate.compile(ids, error);
        if (!db) {
            failed = true;
            throw CompileFailed(error);
        }

        return db;
    }

private:
    std::mutex mutex;
    std::shared_ptr<BaseDB> db;
    std::set<unsigned> ids;
    bool failed = false; // Database failed compilation.
};

#endif // UE2COLLIDER_DATABASEPROXY_H
