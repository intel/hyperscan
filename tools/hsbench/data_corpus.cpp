/*
 * Copyright (c) 2016, Intel Corporation
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

#include "data_corpus.h"

#include "util/container.h"
#include "ue2common.h"

#include <cassert>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <sqlite3.h>

using namespace std;
using namespace ue2;

static
void readRow(sqlite3_stmt *statement, vector<DataBlock> &blocks,
             map<unsigned int, unsigned int> &stream_indices) {
    unsigned int id = sqlite3_column_int(statement, 0);
    unsigned int stream_id = sqlite3_column_int(statement, 1);
    const char *blob = (const char *)sqlite3_column_blob(statement, 2);
    unsigned int bytes = sqlite3_column_bytes(statement, 2);

    if (!contains(stream_indices, stream_id)) {
        unsigned int internal_stream_index = stream_indices.size();
        stream_indices[stream_id] = internal_stream_index;
    }
    auto internal_stream_index = stream_indices[stream_id];

    assert(blob || bytes > 0);
    blocks.emplace_back(id, stream_id, internal_stream_index,
                        string(blob, blob + bytes));
}

vector<DataBlock> readCorpus(const string &filename) {
    int status;
    sqlite3 *db = nullptr;

    status = sqlite3_open_v2(filename.c_str(), &db, SQLITE_OPEN_READONLY,
                             nullptr);

    assert(db);
    if (status != SQLITE_OK) {
        ostringstream err;
        err << "Unable to open database '" << filename << "': "
            << sqlite3_errmsg(db);
        status = sqlite3_close(db);
        assert(status == SQLITE_OK);
        throw DataCorpusError(err.str());
    }

    static const string query("SELECT id, stream_id, data "
                              "FROM chunk ORDER BY id;");

    sqlite3_stmt *statement = nullptr;

    status = sqlite3_prepare_v2(db, query.c_str(), query.size(), &statement,
                                nullptr);
    if (status != SQLITE_OK) {
        status = sqlite3_finalize(statement);
        assert(status == SQLITE_OK);
        status = sqlite3_close(db);
        assert(status == SQLITE_OK);

        ostringstream oss;
        oss << "Query failed: " << query;
        throw DataCorpusError(oss.str());
    }

    vector<DataBlock> blocks;
    map<unsigned int, unsigned int> stream_indices;

    status = sqlite3_step(statement);
    while (status == SQLITE_ROW) {
        readRow(statement, blocks, stream_indices);
        status = sqlite3_step(statement);
    }

    if (status != SQLITE_DONE) {
        ostringstream oss;
        oss << "Error retrieving blocks from corpus: "
            << sqlite3_errmsg(db);

        status = sqlite3_finalize(statement);
        assert(status == SQLITE_OK);
        status = sqlite3_close(db);
        assert(status == SQLITE_OK);

        throw DataCorpusError(oss.str());
    }

    status = sqlite3_finalize(statement);
    assert(status == SQLITE_OK);
    status = sqlite3_close(db);
    assert(status == SQLITE_OK);

    if (blocks.empty()) {
        throw DataCorpusError("Database contains no blocks.");
    }

    return blocks;
}
