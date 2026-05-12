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

#ifndef COMMON_H
#define COMMON_H

#include <string>

enum class ScanMode { BLOCK, STREAMING, VECTORED };

extern bool echo_matches;
extern bool saveDatabases;
extern bool loadDatabases;
extern std::string serializePath;
extern unsigned int somPrecisionMode;
extern bool forceEditDistance;
extern unsigned editDistance;
extern bool printCompressSize;
extern bool use_literal_api;
extern bool use_rliteral_api;
extern bool use_universal_database;

/** Structure for the result of a single complete scan. */
struct ResultEntry {
    double seconds = 0;       //!< Time taken for scan.
    unsigned int matches = 0; //!< Count of matches found.
#ifdef ENGINE_UPDATE_ON
    unsigned int engineType = 0;
#endif
};

struct SqlFailure {
    explicit SqlFailure(const std::string &s) : message(s) {}
    std::string message;
};

#endif // COMMON_H
