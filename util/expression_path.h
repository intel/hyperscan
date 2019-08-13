/*
 * Copyright (c) 2015-2016, Intel Corporation
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

#ifndef EXPRESSION_PATH_H
#define EXPRESSION_PATH_H

#include "ue2common.h"

#include <cerrno>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include <sys/stat.h>
#if !defined(_WIN32)
#include <unistd.h>
#include <libgen.h>
#endif

//
// Utility functions
//

/**
 * Given a path to a signature file, infer the path of the pcre directory.
 */
static inline
std::string inferExpressionPath(const std::string &sigFile) {
#ifndef _WIN32
    // POSIX variant.

    // dirname() may modify its argument, so we must make a copy.
    std::vector<char> path(sigFile.begin(), sigFile.end());
    path.push_back(0); // ensure null termination.

    std::string rv = dirname(path.data());
#else
    // Windows variant.
    if (sigFile.size() >= _MAX_DIR) {
        return std::string();
    }
    char path[_MAX_DIR];
    _splitpath(sigFile.c_str(), nullptr, path, nullptr, nullptr);
    std::string rv(path);
#endif

    rv += "/../pcre";
    return rv;
}

#if defined(_WIN32)
#define stat _stat
#define S_IFREG _S_IFREG
#endif

static inline
bool isDir(const std::string &filename) {
    struct stat s;

    if (stat(filename.c_str(), &s) == -1) {
        std::cerr << "stat: " << strerror(errno) << std::endl;
        return false;
    }

    return (S_IFDIR & s.st_mode);
}

static inline
bool isFile(const std::string &filename) {
    struct stat s;

    if (stat(filename.c_str(), &s) == -1) {
        std::cerr << "stat: " << strerror(errno) << std::endl;
        return false;
    }

    return (S_IFREG & s.st_mode);
}

#endif /* EXPRESSION_PATH_H */
