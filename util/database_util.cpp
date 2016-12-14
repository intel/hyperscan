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

#include "database_util.h"

#include "hs_common.h"

#include <cassert>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>

#if defined(HAVE_MMAP)
#include <sys/mman.h> // for mmap
#include <unistd.h> // for close
#include <sys/fcntl.h>
#include <sys/stat.h>
#endif

using namespace std;

bool saveDatabase(const hs_database_t *db, const char *filename, bool verbose) {
    assert(db);
    assert(filename);

    if (verbose) {
        cout << "Saving database to: " << filename << endl;
    }

    char *bytes = nullptr;
    size_t length = 0;
    hs_error_t err = hs_serialize_database(db, &bytes, &length);
    if (err != HS_SUCCESS) {
        return false;
    }

    assert(bytes);
    assert(length > 0);

    ofstream out(filename, ios::binary);
    out.write(bytes, length);
    out.close();

    ::free(bytes);

    return true;
}

hs_database_t * loadDatabase(const char *filename, bool verbose) {
    assert(filename);

    if (verbose) {
        cout << "Loading database from: " << filename << endl;
    }

    char *bytes = nullptr;

#if defined(HAVE_MMAP)
    // Use mmap to read the file
    int fd = open(filename, O_RDONLY);
    if (fd < 0) {
        return nullptr;
    }
    struct stat st;
    if (fstat(fd, &st) < 0) {
        close(fd);
        return nullptr;
    }
    size_t len = st.st_size;

    bytes = (char *)mmap(nullptr, len, PROT_READ, MAP_SHARED, fd, 0);
    if (bytes == MAP_FAILED) {
        cout << "mmap failed" << endl;
        close(fd);
        return nullptr;
    }
#else
    // Fall back on stream IO
    ifstream is;
    is.open(filename, ios::in | ios::binary);
    if (!is.is_open()) {
        return nullptr;
    }
    is.seekg(0, ios::end);
    size_t len = is.tellg();
    if (verbose) {
        cout << "Reading " << len << " bytes" << endl;
    }
    is.seekg(0, ios::beg);
    bytes = new char[len];
    is.read(bytes, len);
    is.close();
#endif

    assert(bytes);

    if (verbose) {
        char *info = nullptr;
        hs_error_t err = hs_serialized_database_info(bytes, len, &info);
        if (err) {
            cout << "Unable to decode serialized database info: " << err
                 << endl;
        } else if (info) {
            cout << "Serialized database info: " << info << endl;
            std::free(info);
        } else {
            cout << "Unable to decode serialized database info." << endl;
        }
    }

    hs_database_t *db = nullptr;
    hs_error_t err = hs_deserialize_database(bytes, len, &db);

#if defined(HAVE_MMAP)
    munmap(bytes, len);
    close(fd);
#else
    delete [] bytes;
#endif

    if (err != HS_SUCCESS) {
        cout << "hs_deserialize_database call failed: " << err << endl;
        return nullptr;
    }

    assert(db);

    return db;
}
