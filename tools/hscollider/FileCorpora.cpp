/*
 * Copyright (c) 2015-2017, Intel Corporation
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

#include "ColliderCorporaParser.h"
#include "FileCorpora.h"
#include "common.h"
#include "util/expression_path.h"

#include <iostream>
#include <fstream>

#include <boost/algorithm/string/trim.hpp>

using namespace std;

// Returns true if this line is empty or a comment and should be skipped
static
bool emptyLine(const string& line) {
    return line.empty() || line[0] == '#';
}

FileCorpora *FileCorpora::clone() const {
    FileCorpora *copy = new FileCorpora();
    copy->corpora_by_pat = corpora_by_pat;
    return copy;
}

bool FileCorpora::readLine(const string &line) {
    unsigned id = 0;
    Corpus c;
    bool rv = parseCorpus(line, c, id);
    if (rv) {
        corpora_by_pat[id].push_back(c);
        return true;
    } else {
        return false;
    }
}

bool FileCorpora::readFile(const string &filename) {
    ifstream f(filename.c_str());
    if (!f.good()) {
        return false;
    }

    unsigned lineNum = 0;
    string line;
    while (getline(f, line)) {
        lineNum++;

        boost::trim(line);

        if (emptyLine(line)) {
            continue;
        }
        if (!readLine(line)) {
            cerr << "Error in corpora file parsing line " << lineNum << endl;
            return false;
        }
    }
    return !corpora_by_pat.empty();
}

void FileCorpora::generate(unsigned id,
                           vector<Corpus> &data) {
    auto i = corpora_by_pat.find(id);
    if (i == corpora_by_pat.end() || i->second.empty()) {
        throw CorpusFailure("no corpora found for pattern.");
    }

    data.insert(data.end(), i->second.begin(), i->second.end());
}
