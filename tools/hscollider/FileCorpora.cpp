/*
 * Copyright (C) 2026 Intel Corporation
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
        corpora_by_pat[id].push_back(std::move(c));
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
