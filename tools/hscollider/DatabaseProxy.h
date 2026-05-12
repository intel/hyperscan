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
        : db(std::move(built_db)) {}

    std::shared_ptr<BaseDB> get(const UltimateTruth &ultimate) {
        std::lock_guard<std::mutex> lock(mutex);
        if (failed) {
            // We have previously failed to compile this database.
            throw CompileFailed("Unable to compile db previously.");
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
