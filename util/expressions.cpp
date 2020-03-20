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
#include "expressions.h"

#include "hs.h"
#include "string_util.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include <sys/types.h>
#include <sys/stat.h>
#if !defined(_WIN32)
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>
#else
// Windows support is probably very fragile
#include <windows.h>
#endif

#include <boost/algorithm/string/trim.hpp>

using namespace std;

static
void failLine(unsigned lineNum, const string &file,
              const string &line, const string &error) {
    cerr << "Parse error in file " << file
        << " on line " << lineNum << ": " << error
        << endl << "Line is: '" << line << "'" << endl;
    exit(1);
}

static
void processLine(string &line, unsigned lineNum,
                 const string &file, ExpressionMap &exprMap) {
    // if line is empty, or a comment, we can skip it
    if (line.empty() || line[0] == '#') {
        return;
    }

    // cull any whitespace
    boost::trim(line);

    // otherwise, it should be ID:PCRE, e.g.
    //  10001:/foobar/is

    size_t colonIdx = line.find_first_of(':');
    if (colonIdx == string::npos) {
        failLine(lineNum, file, line, "Could not parse line.");
    }

    // we should have an unsigned int as an ID, before the colon
    unsigned id;
    if (!fromString(line.substr(0, colonIdx), id)) {
        failLine(lineNum, file, line, "Unable to parse ID.");
    }

    // rest of the expression is the PCRE
    const string pcre_str(line.substr(colonIdx + 1));

    //cout << "Inserting expr: id=" << id << ", pcre=" << pcre_str << endl;

    bool ins = exprMap.emplace(id, pcre_str).second;
    if (!ins) {
        failLine(lineNum, file, line, "Duplicate ID found.");
    }
}

#if defined(_WIN32)
#define stat _stat
#define S_ISDIR(st_m) (_S_IFDIR & (st_m))
#define S_ISREG(st_m) (_S_IFREG & (st_m))
#endif
void HS_CDECL loadExpressionsFromFile(const string &fname, ExpressionMap &exprMap) {
    struct stat st;
    if (stat(fname.c_str(), &st) != 0) {
        return;
    }
    if (!S_ISREG(st.st_mode)) {
        return;
    }
    ifstream f(fname.c_str());
    if (!f.good()) {
        throw runtime_error("Can't open file");
    }

    unsigned lineNum = 0;
    string line;
    while (getline(f, line)) {
        lineNum++;
        processLine(line, lineNum, fname, exprMap);
    }
}

static
bool isIgnorable(const std::string &f) {
    if (f.empty()) {
        return true;
    }

    // Editor backup files
    if (*f.rbegin() == '~') {
        return true;
    }

    // Ignore dotfiles
    if (*f.begin() == '.') {
        return true;
    }

    return false;
}

#ifndef _WIN32
void loadExpressions(const string &inPath, ExpressionMap &exprMap) {
    // Is our input path a file or a directory?
    struct stat st;
    if (stat(inPath.c_str(), &st) != 0) {
        cerr << "Can't stat path: '" << inPath << "'" << endl;
        exit(1);
    }
    if (S_ISREG(st.st_mode)) {
        // process file
        try {
            loadExpressionsFromFile(inPath, exprMap);
        } catch (runtime_error &e) {
            cerr << e.what() << ": '" << inPath << "'" << endl;
            exit(1);
        }
    } else if (S_ISDIR(st.st_mode)) {
        DIR *d = opendir(inPath.c_str());
        if (d == nullptr) {
            cerr << "Can't open directory: '" << inPath << "'" << endl;
            exit(1);
        }
        for (struct dirent *ent = readdir(d); ent; ent = readdir(d)) {
            string basename(ent->d_name);
            string fname(inPath);
            fname.push_back('/');
            fname.append(basename);

            // Ignore '.' and '..'
            if (basename == "." || basename == "..") {
                continue;
            }

            // Skip emacs backup files, dotfiles (such as VIM swap).
            if (isIgnorable(basename)) {
                cerr << "Ignoring signature file " << fname << endl;
                continue;
            }

            try {
                loadExpressionsFromFile(fname, exprMap);
            } catch (runtime_error &e) {
                cerr << e.what() << ": '" << fname << "'" << endl;
                exit(1);
            }
        }
        (void)closedir(d);
    } else {
        cerr << "Unsupported file type "
	    << hex << showbase << (st.st_mode & S_IFMT)
	    << " for path: '" << inPath << "'" << endl;
        exit(1);
    }
}
#else // windows TODO: improve
void HS_CDECL loadExpressions(const string &inPath, ExpressionMap &exprMap) {
    // Is our input path a file or a directory?
    struct stat st;
    if (stat(inPath.c_str(), &st) != 0) {
        cerr << "Can't stat path: '" << inPath << "'" << endl;
        exit(1);
    }
    if (S_ISREG(st.st_mode)) {
        // process file
        try {
            loadExpressionsFromFile(inPath, exprMap);
        } catch (runtime_error &e) {
            cerr << e.what() << ": '" << inPath << "'" << endl;
            exit(1);
        }
    } else if (S_ISDIR(st.st_mode)) {
        WIN32_FIND_DATA ffd;
        HANDLE hFind = INVALID_HANDLE_VALUE;
        string glob = inPath + "/*";
        hFind = FindFirstFile(glob.c_str(), &ffd);
        if (hFind == INVALID_HANDLE_VALUE) {
            cerr << "Can't open directory: '" << inPath << "'" << endl;
            exit(1);
        }
        do {
            string basename(ffd.cFileName);
            string fname(inPath);
            fname.push_back('/');
            fname.append(basename);

            // Ignore '.' and '..'
            if (basename == "." || basename == "..") {
                continue;
            }

            // Skip emacs backup files, dotfiles (such as VIM swap).
            if (isIgnorable(basename)) {
                cerr << "Ignoring signature file " << fname << endl;
                continue;
            }

            try {
                loadExpressionsFromFile(fname, exprMap);
            } catch (runtime_error &e) {
                cerr << e.what() << ": '" << fname << "'" << endl;
                exit(1);
            }
        } while (FindNextFile(hFind, &ffd) != 0);
        FindClose(hFind);
    } else {
        cerr << "Can't stat path: '" << inPath << "'" << endl;
        exit(1);
    }
}
#endif

void HS_CDECL loadSignatureList(const string &inFile,
                                SignatureSet &signatures) {
    ifstream f(inFile.c_str());
    if (!f.good()) {
        cerr << "Can't open file: '" << inFile << "'" << endl;
        exit(1);
    }

    unsigned lineNum = 0;
    string line;
    while (getline(f, line)) {
        lineNum++;

        // if line is empty, or a comment, we can skip it
        if (line.empty() || line[0] == '#') {
            continue;
        }

        unsigned id;
        if (fromString(line, id)) {
            signatures.push_back(id);
        } else {
            // Parse error occurred
            failLine(lineNum, inFile, line, "Unable to parse ID.");
        }
    }
}

ExpressionMap limitToSignatures(const ExpressionMap &exprMap,
                                const SignatureSet &signatures) {
    ExpressionMap keepers;

    for (auto id : signatures) {
        auto match = exprMap.find(id);
        if (match == exprMap.end()) {
            cerr << "Unable to find signature " << id
                    << " in expression set!" << endl;
            exit(1);
        }
        keepers.insert(*match);
    }

    return keepers;
}
