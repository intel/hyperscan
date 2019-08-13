/*
 * Copyright (c) 2015-2019, Intel Corporation
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

/**
 * \file
 * \brief Hyperscan compile dump tool
 *
 * Given a set of patterns, dump all available data from the compilation
 * process into a directory. This tool is intended to assist Hyperscan
 * developers with developement and debugging by providing insights into the
 * built bytecode.
 *
 * Note: requires that hyperscan is built with DUMP_SUPPORT enabled.
 */

#include "config.h"

#include "cross_compile.h"
#include "ExpressionParser.h"
#include "expressions.h"
#include "expression_path.h"
#include "string_util.h"

#include "grey.h"
#include "hs_compile.h"
#include "hs_internal.h"
#include "scratch_dump.h"

#include <cassert>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#ifndef _WIN32
#include <getopt.h>
#else
#include "win_getopt.h"
#endif
#include <sys/stat.h>

#ifndef _WIN32
#include <dirent.h>
#else
#include <direct.h>
#define stat _stat
#endif

#include <boost/ptr_container/ptr_vector.hpp>

using namespace std;
using namespace ue2;
using boost::ptr_vector;

namespace /* anonymous */ {

// Input pattern file
string patternfile;
// Output path
string dumpbase(".");
// Compile with streaming
bool streaming = true;
bool vectored = false;

bool echoSigs = false;
bool dump_db = false;
bool force_utf8 = false;
int force_prefilter = 0;

unsigned int onlyId;
u32 somFlags = 0;
unsigned somPrecisionMode = HS_MODE_SOM_HORIZON_LARGE;

bool singleId = false;
string signatureFile;

unique_ptr<hs_platform_info> plat_info;

bool dump_intermediate = true;
bool force_edit_distance = false;
u32 edit_distance = 0;

int use_literal_api = 0;

} // namespace

// Usage statement.
static
void usage(const char *name, const char *error) {
    printf("Usage: %s [OPTIONS...]\n\n", name);
    printf("Options:\n\n");
    printf("  -h              Display help and exit.\n");
    printf("  -G OVERRIDES    Overrides for the grey box.\n");
    printf("  -e PATH         Path to expression directory or file.\n");
    printf("  -s FILE         Signature file to use.\n");
    printf("  -z NUM          Signature ID to use.\n");
    printf("  -N, --block     Compile in block mode"
           " (default: streaming).\n");
    printf("  -V, --vectored  Compile in vectored mode"
           " (default: streaming).\n");
    printf("  -o, --output PATH\n");
    printf("                  Use data dump directory PATH (default: dump).\n");
    printf("                  WARNING: existing files in output directory are"
           " deleted.\n");
    printf("  -x NAME         Cross-compile for arch NAME\n");
    printf("  -D, --dump_db   Dump the final database.\n");
    printf("  -P, --print     Echo signature set to stdout.\n");
    printf("  -X, --no_intermediate\n");
    printf("                  Do not dump intermediate data.\n");
    printf("\n");
    printf("Pattern flags:\n");
    printf("  -d NUMBER       Set SOM precision mode (default: 8 (large)).\n");
    printf("  -E DISTANCE     Match all patterns within edit distance"
           " DISTANCE.\n");
    printf("  -8              Force UTF8 mode on all patterns.\n");
    printf("  -L              Apply HS_FLAG_SOM_LEFTMOST to all patterns.\n");
    printf(" --prefilter      Apply HS_FLAG_PREFILTER to all patterns.\n");
    printf(" --literal-on     Use Hyperscan pure literal matching API.\n");
    printf("\n");
    printf("Example:\n");
    printf("$ %s -e pattern.file -s sigfile\n", name);
    printf("\n");

    if (error) {
        printf("Error: %s\n", error);
    }
}

static
void processArgs(int argc, char *argv[], Grey &grey) {
    static const char *options = "d:De:E:G:hLNo:Ps:VXx:z:8";
    static struct option longOptions[] = {
        {"dump_db",             no_argument,        nullptr, 'D'},
        {"help",                no_argument,        nullptr, 'h'},
        {"output",              required_argument,  nullptr, 'o'},
        {"block",               no_argument,        nullptr, 'N'},
        {"no_intermediate",     no_argument,        nullptr, 'X'},
        {"vectored",            no_argument,        nullptr, 'V'},
        {"print",               no_argument,        nullptr, 'P'},
        {"utf8",                no_argument,        nullptr, '8'},
        {"prefilter",           no_argument,        &force_prefilter, 1},
        {"som-width",           required_argument,  nullptr, 'd'},
        {"literal-on",          no_argument,        &use_literal_api, 1},
        {nullptr, 0, nullptr, 0}
    };

    for (;;) {
        int c = getopt_long(argc, argv, options, longOptions, nullptr);

        if (c < 0) {
            break;
        }
        switch (c) {
        case 'D':
            dump_db = true;
            break;

        case 'd': {
            unsigned dist;
            if (!fromString(optarg, dist)) {
                usage(argv[0], "Must provide an integer argument to '-d' flag");
                exit(1);
            }
            switch (dist) {
            case 2:
                somPrecisionMode = HS_MODE_SOM_HORIZON_SMALL;
                break;
            case 4:
                somPrecisionMode = HS_MODE_SOM_HORIZON_MEDIUM;
                break;
            case 8:
                somPrecisionMode = HS_MODE_SOM_HORIZON_LARGE;
                break;
            default:
                usage(argv[0], "SOM precision must be 2, 4 or 8");
                exit(1);
            }
            break;
        }

        case 'h':
            usage(argv[0], nullptr);
            exit(0);

        case 'e':
            patternfile = optarg;
            break;

#ifndef RELEASE_BUILD
        case 'G':
            applyGreyOverrides(&grey, string(optarg));
            break;
#endif

        case 'L':
            somFlags |= HS_FLAG_SOM_LEFTMOST;
            break;

        case 'o':
            dumpbase = optarg;
            break;

        case 'P':
            echoSigs = true;
            break;

        case 's':
            signatureFile.assign(optarg);
            break;

        case 'N':
            streaming = false;
            break;

        case 'V':
            streaming = false;
            vectored = true;
            break;

        case 'X':
            dump_intermediate = false;
            break;

        case 'x':
            plat_info = xcompileReadMode(optarg);
            if (!plat_info) {
                usage(argv[0], xcompileUsage().c_str());
                exit(1);
            }
            break;

        case 'z':
            if (!fromString(optarg, onlyId)) {
                usage(argv[0], "Argument to '-z' flag must be an integer");
                exit(1);
            }
            singleId = true;
            break;
        case 'E': {
            u32 dist;
            if (!fromString(optarg, dist)) {
                usage(argv[0], "Argument to '-E' flag must be an integer");
                exit(1);
            }
            force_edit_distance = true;
            edit_distance = dist;
            break;
        }
        case '8':
            force_utf8 = true;
            break;
        case 0:
            break;
        default:
            usage(argv[0], "");
            exit(1);
        }
    }

    if (patternfile.empty() && !signatureFile.empty()) {
        /* attempt to infer an expression directory */
        patternfile = inferExpressionPath(signatureFile);
    }

    if (patternfile.size() == 0) {
        usage(argv[0], "No pattern file provided");
        exit(1);
    }
    if (dumpbase.size() == 0) {
        usage(argv[0], "No output directory provided");
        exit(1);
    }
}

static
void dumpDb(const struct hs_database *out, const Grey &grey) {
    char *bytes = nullptr;
    size_t len = 0;
    hs_error_t err = hs_serialize_database(out, &bytes, &len);
    if (err != HS_SUCCESS) {
        printf("ERROR: hs_serialize_database() failed with error %u\n", err);
        return;
    }

    FILE *f = fopen((grey.dumpPath + "db.raw").c_str(), "w");
    if (!f) {
        printf("ERROR: unable to write database out: %s", strerror(errno));
    } else {
        fwrite(bytes, 1, len, f);
        fclose(f);
    }
    free(bytes);
}

static
u32 buildDumpFlags(void) {
    u32 flags = 0;
    flags |= Grey::DUMP_BASICS;
    flags |= Grey::DUMP_IMPL;

    if (dump_intermediate) {
        flags |= Grey::DUMP_PARSE;
        flags |= Grey::DUMP_INT_GRAPH;
    }

    return flags;
}

#ifndef _WIN32
static
void clearDir(const string &path) {
    DIR *dir = opendir(path.c_str());
    if (!dir) {
        printf("ERROR: couldn't open location %s: %s\n", path.c_str(),
               strerror(errno));
        exit(1);
    }

    struct dirent *d_ent;
    while (nullptr != (d_ent = readdir(dir))) {
        string name(d_ent->d_name);
        if (name == "." || name == "..") {
            continue;
        }
        string f = path + '/' + name;
        if (unlink(f.c_str()) < 0) {
            printf("ERROR: couldn't remove file %s: %s\n", f.c_str(),
                   strerror(errno));
        }
    }
    closedir(dir);
}
#else // windows
static
void clearDir(const string &path) {
    WIN32_FIND_DATA ffd;
    HANDLE hFind = INVALID_HANDLE_VALUE;
    string glob = path + "/*";
    hFind = FindFirstFile(glob.c_str(), &ffd);
    if (hFind == INVALID_HANDLE_VALUE) {
        printf("ERROR: couldn't open location %s\n", path.c_str());
        exit(1);
    }
    do {
        string basename(ffd.cFileName);
        string fname(path);
        fname.push_back('/');
        fname.append(basename);

        // Ignore '.' and '..'
        if (basename == "." || basename == "..") {
            continue;
        }

        if (!DeleteFile(fname.c_str())) {
            printf("ERROR: couldn't remove file %s\n", fname.c_str());
        }

    } while (FindNextFile(hFind, &ffd) != 0);
    FindClose(hFind);
}
#endif

static
int makeDirectory(const string &dirName) {
#ifndef _WIN32
    mode_t mode = S_IRUSR | S_IWUSR | S_IXUSR | S_IRGRP | S_IXGRP |
                  S_IROTH | S_IXOTH;
    return mkdir(dirName.c_str(), mode);
#else
    return _mkdir(dirName.c_str());
#endif
}

static
void prepareDumpLoc(string parent, string path, u32 flags, Grey &grey) {
    struct stat st;
    if (stat(parent.c_str(), &st)) {
        // Create dump location if not found
        if (makeDirectory(parent) < 0) {
            printf("ERROR: could not create dump location %s: %s\n",
                   parent.c_str(), strerror(errno));
            exit(1);
        }
    }

    // If not separator terminated, add separator
    if (parent.back() != '/') {
        parent.push_back('/');
    }

    // Append path to parent
    path = parent.append(path);
    if (stat(path.c_str(), &st)) {
        // Create dump location if not found
        if (makeDirectory(path) < 0) {
            printf("ERROR: could not create dump location %s: %s\n",
                   path.c_str(), strerror(errno));
            exit(1);
        }
    }

    // remove anything in the dump dir - most likely stale
    clearDir(path);

    // If not separator terminated, add separator
    if (path.back() != '/') {
        path.push_back('/');
    }

    grey.dumpPath = path;
    grey.dumpFlags = flags;
}

static
unsigned buildMode() {
    unsigned mode = 0;
    if (streaming) {
        mode |= HS_MODE_STREAM;
        mode |= somPrecisionMode;
        assert(!vectored);
    } else if (vectored) {
        mode |= HS_MODE_VECTORED;
    } else {
        mode |= HS_MODE_BLOCK;
    }

    return mode;
}

static
void dumpScratch(const hs_database_t *db, const Grey &grey) {
    hs_scratch_t *scratch = nullptr;
    hs_error_t err = hs_alloc_scratch(db, &scratch);
    if (err == HS_SUCCESS) {
        FILE *f = fopen((grey.dumpPath + "scratch.txt").c_str(), "w");
        if (f) {
            dumpScratch(scratch, f);
            fclose(f);
        } else {
            printf("ERROR: could not open %s: %s\n",
                   (grey.dumpPath + "scratch.txt").c_str(), strerror(errno));
        }
    } else {
        printf("ERROR: hs_alloc_scratch() failed with error %u\n", err);
    }
    hs_free_scratch(scratch);
}

static
void dumpInfo(const hs_database_t *db, const Grey &grey) {
    char *info = nullptr;
    hs_error_t err = hs_database_info(db, &info);
    if (err == HS_SUCCESS) {
        FILE *f = fopen((grey.dumpPath + "db_info.txt").c_str(), "w");
        if (f) {
            fprintf(f, "%s\n", info);
            fclose(f);
        } else {
            printf("ERROR: could not open %s: %s\n",
                   (grey.dumpPath + "db_info.txt").c_str(), strerror(errno));
        }
    } else {
        printf("ERROR: hs_database_info() failed with error %u\n", err);
    }
    free(info);
}

static
unsigned int dumpDataMulti(const vector<const char *> &patterns,
                           const vector<unsigned> &flags,
                           const vector<unsigned> &ids,
                           ptr_vector<hs_expr_ext> &ext,
                           const Grey &grey) {
    unsigned mode = buildMode();

    printf("Compiling %zu patterns.\n", patterns.size());

    hs_database_t *db = nullptr;
    hs_compile_error_t *compile_err;

    hs_error_t err;
    const size_t count = patterns.size();
    if (use_literal_api) {
        // Compute length of each pattern.
        vector<size_t> lens(count);
        for (unsigned int i = 0; i < count; i++) {
            lens[i] = strlen(patterns[i]);
        }
        err = hs_compile_lit_multi_int(patterns.data(), flags.data(),
                                       ids.data(), ext.c_array(), lens.data(),
                                       count, mode, plat_info.get(), &db,
                                       &compile_err, grey);
    } else {
        err = hs_compile_multi_int(patterns.data(), flags.data(), ids.data(),
                                   ext.c_array(), count, mode, plat_info.get(),
                                   &db, &compile_err, grey);
    }

    if (err != HS_SUCCESS) {
        if (compile_err && compile_err->message) {
            printf("ERROR: Compile failed: %s\n", compile_err->message);
        } else {
            printf("ERROR: hs_compile_multi_int() returned error %u", err);
        }
        hs_free_compile_error(compile_err);
        return 1;
    }

    assert(db);
    dumpScratch(db, grey);
    dumpInfo(db, grey);

    if (dump_db) {
        dumpDb(db, grey);
    }

    hs_free_database(db);
    return 0;
}

static
unsigned int dumpData(const ExpressionMap &exprMap, Grey &grey) {
    u32 dump_flags = buildDumpFlags();
    string path = "dump";
    prepareDumpLoc(dumpbase, path, dump_flags, grey);
    printf("Dumping data for all patterns in '%s' to '%s/%s'\n",
           patternfile.c_str(), dumpbase.c_str(), path.c_str());

    string pat_name = grey.dumpPath + "patterns.txt";
    FILE *pat_out = fopen(pat_name.c_str(), "w");
    if (!pat_out) {
        printf("ERROR: unable to open %s\n", pat_name.c_str());
        return 1;
    }

    const size_t numPatterns = exprMap.size();
    vector<string> expressions(numPatterns);
    vector<unsigned> ids(numPatterns);
    vector<unsigned> flags(numPatterns);
    ptr_vector<hs_expr_ext> ext;
    ext.reserve(numPatterns);

    size_t n = 0;
    for (const auto &elem : exprMap) {
        const auto &id = elem.first;
        const auto &regex = elem.second;
        if (echoSigs) {
            printf("%u:%s\n", id, regex.c_str());
        }
        fprintf(pat_out, "%u:%s\n", id, regex.c_str());

        ext.push_back(new hs_expr_ext);
        ids[n] = id;
        if (!readExpression(regex, expressions[n], &flags[n], &ext[n])) {
            printf("ERROR: failed to parse expr: %s (id %u)\n",
                   regex.c_str(), id);
            fclose(pat_out);
            return 1;
        }

        if (force_edit_distance) {
            ext[n].flags |= HS_EXT_FLAG_EDIT_DISTANCE;
            ext[n].edit_distance = edit_distance;
        }

        flags[n] |= somFlags;
        if (force_utf8) {
            flags[n] |= HS_FLAG_UTF8;
        }
        if (force_prefilter) {
            flags[n] |= HS_FLAG_PREFILTER;
        }

        n++;
    }
    assert(n);

    // Our compiler takes an array of plain ol' C strings.
    vector<const char *> patterns(n);
    for (size_t i = 0; i < n; i++) {
        patterns[i] = expressions[i].c_str();
    }

    fclose(pat_out);
    return dumpDataMulti(patterns, flags, ids, ext, grey);
}

int HS_CDECL main(int argc, char *argv[]) {
    Grey grey;
    grey.dumpFlags = Grey::DUMP_BASICS;

    processArgs(argc, argv, grey);

    // Load patterns
    ExpressionMap exprMap;
    loadExpressions(patternfile, exprMap);

    if (!signatureFile.empty()) {
        SignatureSet sigs;
        loadSignatureList(signatureFile, sigs);
        exprMap = limitToSignatures(exprMap, sigs);
    }

    if (singleId) {
        exprMap = limitToSignatures(exprMap, {onlyId});
    }

    if (exprMap.empty()) {
        printf("No signatures.\n");
        return 1;
    }

    return dumpData(exprMap, grey);
}
