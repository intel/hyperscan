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
 * \brief hscheck: Tool to test regex compilation with Hyperscan.
 *
 * hscheck accepts a file of regular expressions in the form:
 * "ID:/regex/flags" and tests whether they can be compiled with Hyperscan,
 * reporting the error if compilation fails.
 *
 * For example, create the file "regex" containing:
 *
 *    1:/foo.*bar/s
 *    2:/hatstand|teakettle|badgerbrush/
 *
 * This can be checked with the following hscheck invocation:
 *
 *     $ bin/hscheck -e regex
 *
 * Use "hscheck -h" for complete usage information.
 */

#include "config.h"

#include "ExpressionParser.h"
#include "expressions.h"
#include "string_util.h"
#include "util/expression_path.h"
#include "util/make_unique.h"

#include "grey.h"
#include "hs_compile.h"
#include "hs_internal.h"
#include "ue2common.h"

#ifdef HS_HYBRID
#include <pcre.h>
#include "chimera/ch.h"
#endif

#include <cassert>
#include <fstream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#ifndef _WIN32
#include <getopt.h>
#else
#include "win_getopt.h"
#endif
#include <boost/algorithm/string/trim.hpp>

using namespace std;
using namespace ue2;

namespace /* anonymous */ {

// are we in streaming mode? (default: yes)
bool g_streaming = true;
bool g_vectored = false;
bool g_hybrid = false;
string g_exprPath("");
string g_signatureFile("");
bool g_allSignatures = false;
bool g_forceEditDistance = false;
bool build_sigs = false;
bool check_logical = false;
bool use_literal_api = false;
unsigned int g_signature;
unsigned int g_editDistance;
unsigned int globalFlags = 0;
unsigned int num_of_threads = 1;
unsigned int countFailures = 0;

class ParsedExpr {
public:
    ParsedExpr(string regex_in, unsigned int flags_in, hs_expr_ext ext_in)
        : regex(regex_in), flags(flags_in), ext(ext_in) {}
    ~ParsedExpr() {}
    string regex;
    unsigned int flags;
    hs_expr_ext ext;
};

typedef map<unsigned int, ParsedExpr> ExprExtMap;
ExprExtMap g_combs;
ExprExtMap g_validSubs;

// Iterator pointing to next logical expression to process.
ExprExtMap::const_iterator comb_read_it;

// Global greybox structure, used in non-release builds.
unique_ptr<Grey> g_grey;

// Global expression map.
ExpressionMap g_exprMap;

// Iterator pointing to next expression to process.
ExpressionMap::const_iterator read_it;

// Iterator pointing to next expression to print results.
ExpressionMap::const_iterator print_it;

// Mutex guarding access to read iterator.
std::mutex lk_read;

// Mutex serialising access to output map and stdout.
std::mutex lk_output;

// Mutex guarding access to write g_combs.
std::mutex lk_write_comb;

// Mutex guarding access to write g_validSubs.
std::mutex lk_write_sub;

// Possible values for pattern check results.
enum ExprStatus {NOT_PROCESSED, SUCCESS, FAILURE};

// Map for storing results.
map<unsigned int, pair<string, ExprStatus>> output;

} // namespace

static
bool getNextExpressionId(ExpressionMap::const_iterator &it) {
    lock_guard<mutex> lock(lk_read);
    if (read_it != g_exprMap.end()) {
        it = read_it;
        ++read_it;
        return true;
    } else {
        return false;
    }
}

static
bool getNextLogicalExpression(ExprExtMap::const_iterator &it) {
    lock_guard<mutex> lock(lk_read);
    if (comb_read_it != g_combs.end()) {
        it = comb_read_it;
        ++comb_read_it;
        return true;
    } else {
        return false;
    }
}

static
void cacheCombExpr(unsigned id, const string &regex, unsigned int flags,
                   const hs_expr_ext &ext) {
    lock_guard<mutex> lock(lk_write_comb);
    g_combs.emplace(id, ParsedExpr(regex, flags, ext));
}

static
void cacheSubExpr(unsigned id, const string &regex, unsigned int flags,
                  const hs_expr_ext &ext) {
    lock_guard<mutex> lock(lk_write_sub);
    g_validSubs.emplace(id, ParsedExpr(regex, flags, ext));
}

// This function prints the Pattern IDs order
// It creates the output for build sigs
// Caller is required to hold lk_output when calling this function
static
void printExpressionId(const ExpressionMap &exprMap) {
    while (print_it != exprMap.end()) {
        unsigned int id = print_it->first;
        const string &regex = print_it->second;
        const auto &result = output[id];
        if (result.second == NOT_PROCESSED) {
            break;
        }
        bool fail = result.second == FAILURE;
        if (!build_sigs) {
            if (fail) {
                cout << "FAIL (compile): " << id << ":" << regex << ": "
                     << result.first << endl;
            } else {
                cout << result.first << ' ' << id << ":" << regex << endl;
            }
        } else {
            if (fail) {
                cout << "# " << id << " # " << result.first << endl;
            } else {
                cout << id << endl;
            }
        }

        ++print_it;
    }
}

static
void recordFailure(const ExpressionMap &exprMap, unsigned int id,
                   const string &err) {
    lock_guard<mutex> lock(lk_output);
    output[id].first = err;
    output[id].second = FAILURE;
    countFailures++;
    printExpressionId(exprMap);
}

static
void recordSuccess(const ExpressionMap &exprMap, unsigned int id) {
    lock_guard<mutex> lock(lk_output);
    output[id].first = "OK:";
    output[id].second = SUCCESS;
    printExpressionId(exprMap);
}

static
void checkExpression(UNUSED void *threadarg) {
    unsigned int mode = g_streaming  ? HS_MODE_STREAM
                        : g_vectored ? HS_MODE_VECTORED
                                     : HS_MODE_BLOCK;
    if (g_streaming) {
        // Use SOM mode, for permissiveness' sake.
        mode |= HS_MODE_SOM_HORIZON_LARGE;
    }

    ExpressionMap::const_iterator it;
    while (getNextExpressionId(it)) {
        const string &line = it->second;

        // Initial slash char is required, but unused.
        if (line.empty() || line[0] != '/') {
            recordFailure(g_exprMap, it->first,
                             "Format required is \"ID:/REGEX/FLAGS\".");
            continue;
        }

        // Make a mutable copy and trim any whitespace on the right.
        string expr = line;
        boost::trim(expr);

        size_t flagsStart = expr.find_last_of('/');
        if (flagsStart == string::npos || flagsStart == 0) {
            recordFailure(g_exprMap, it->first, "No trailing '/' char.");
            continue;
        }

        string regex;
        unsigned int flags = 0;
        hs_expr_ext ext;
        if (!readExpression(expr, regex, &flags, &ext)) {
            recordFailure(g_exprMap, it->first, "Unsupported flag used.");
            continue;
        }

        flags |= globalFlags;
        if (g_forceEditDistance) {
            ext.edit_distance = g_editDistance;
            ext.flags |= HS_EXT_FLAG_EDIT_DISTANCE;
        }

        if (flags & HS_FLAG_COMBINATION) {
            if (check_logical) {
                cacheCombExpr(it->first, regex, flags, ext);
            } else {
                recordFailure(g_exprMap, it->first, "Unsupported flag used.");
            }
            continue;
        }

        // Try and compile a database.
        const char *regexp = regex.c_str();

        hs_error_t err;

        if (g_hybrid) {
#ifdef HS_HYBRID
            ch_compile_error_t *ch_compile_err;
            ch_database_t *hybrid_db = nullptr;
            err = ch_compile_multi(&regexp, &flags, nullptr, 1, CH_MODE_GROUPS,
                                   nullptr, &hybrid_db, &ch_compile_err);
            if (err == HS_SUCCESS) {
                assert(hybrid_db);
                recordSuccess(g_exprMap, it->first);
                ch_free_database(hybrid_db);
            } else {
                assert(!hybrid_db);
                assert(ch_compile_err);
                recordFailure(g_exprMap, it->first, ch_compile_err->message);
                ch_free_compile_error(ch_compile_err);
            }
#else
            cerr << "Hybrid mode not available in this build." << endl;
            exit(1);
#endif // HS_HYBRID
        } else {
            const hs_expr_ext *extp = &ext;
            hs_compile_error_t *compile_err;
            hs_database_t *db = nullptr;

#if !defined(RELEASE_BUILD)
            // This variant is available in non-release builds and allows us to
            // modify greybox settings.
            if (use_literal_api) {
                size_t len = strlen(regexp);
                err = hs_compile_lit_multi_int(&regexp, &flags, nullptr, &extp,
                                               &len, 1, mode, nullptr, &db,
                                               &compile_err, *g_grey);
            } else {
                err = hs_compile_multi_int(&regexp, &flags, nullptr, &extp, 1,
                                           mode, nullptr, &db, &compile_err,
                                           *g_grey);
            }
#else
            if (use_literal_api) {
                size_t len = strlen(regexp);
                err = hs_compile_lit_multi(&regexp, &flags, nullptr, &len, 1,
                                           mode, nullptr, &db, &compile_err);
            } else {
                err = hs_compile_ext_multi(&regexp, &flags, nullptr, &extp, 1,
                                           mode, nullptr, &db, &compile_err);
            }
#endif

            if (err == HS_SUCCESS) {
                assert(db);
                recordSuccess(g_exprMap, it->first);
                hs_free_database(db);
                if (check_logical) {
                    cacheSubExpr(it->first, regex, flags, ext);
                }
            } else {
                assert(!db);
                assert(compile_err);
                recordFailure(g_exprMap, it->first, compile_err->message);
                hs_free_compile_error(compile_err);
            }
        }
    }
}

static
bool fetchSubIds(const char *logical, vector<unsigned> &ids) {
    unsigned mult = 1;
    unsigned id = 0;
    for (int i = strlen(logical) - 1; i >= 0; i--) {
        if (isdigit(logical[i])) {
            if (mult > 100000000) {
                return false;
            }
            id += (logical[i] - '0') * mult;
            mult *= 10;
        } else if (mult > 1) {
                ids.push_back(id);
                mult = 1;
                id = 0;
        }
    }
    if (mult > 1) {
        ids.push_back(id);
    }
    return true;
}

static
void checkLogicalExpression(UNUSED void *threadarg) {
    unsigned int mode = g_streaming  ? HS_MODE_STREAM
                        : g_vectored ? HS_MODE_VECTORED
                                     : HS_MODE_BLOCK;
    if (g_streaming) {
        // Use SOM mode, for permissiveness' sake.
        mode |= HS_MODE_SOM_HORIZON_LARGE;
    }

    ExprExtMap::const_iterator it;
    while (getNextLogicalExpression(it)) {
        if (use_literal_api) {
            recordSuccess(g_exprMap, it->first);
            continue;
        }

        const ParsedExpr &comb = it->second;

        vector<unsigned> subIds;
        if (!fetchSubIds(comb.regex.c_str(), subIds)) {
            recordFailure(g_exprMap, it->first, "Sub-expression id too large.");
            continue;
        }

        vector<const char *> regexv;
        vector<unsigned> flagsv;
        vector<unsigned> idv;
        vector<const hs_expr_ext *> extv;
        bool valid = true;

        for (const auto i : subIds) {
            ExprExtMap::const_iterator jt = g_validSubs.find(i);
            if (jt != g_validSubs.end()) {
                const ParsedExpr &sub = jt->second;
                regexv.push_back(sub.regex.c_str());
                flagsv.push_back(sub.flags);
                idv.push_back(i);
                extv.push_back(&sub.ext);
            } else {
                valid = false;
                break;
            }
        }

        if (valid) {
            regexv.push_back(comb.regex.c_str());
            flagsv.push_back(comb.flags);
            idv.push_back(it->first);
            extv.push_back(&comb.ext);
        } else {
            recordFailure(g_exprMap, it->first, "Sub-expression id not valid.");
            continue;
        }

        // Try and compile a database.
        hs_error_t err;
        hs_compile_error_t *compile_err;
        hs_database_t *db = nullptr;

#if !defined(RELEASE_BUILD)
        // This variant is available in non-release builds and allows us to
        // modify greybox settings.
        err = hs_compile_multi_int(regexv.data(), flagsv.data(), idv.data(),
                                   extv.data(), regexv.size(), mode,
                                   nullptr, &db, &compile_err, *g_grey);
#else
        err = hs_compile_ext_multi(regexv.data(), flagsv.data(), idv.data(),
                                   extv.data(), regexv.size(), mode,
                                   nullptr, &db, &compile_err);
#endif

        if (err == HS_SUCCESS) {
            assert(db);
            recordSuccess(g_exprMap, it->first);
            hs_free_database(db);
        } else {
            assert(!db);
            assert(compile_err);
            recordFailure(g_exprMap, it->first, compile_err->message);
            hs_free_compile_error(compile_err);
        }
    }
}

static
void usage() {
    cout << "Usage: hscheck [OPTIONS...]"  << endl << endl
         << "  -e PATH         Path to expression directory." << endl
         << "  -s FILE         Signature file to use." << endl
         << "  -z NUM          Signature ID to use." << endl
         << "  -E DISTANCE     Force edit distance to DISTANCE for all patterns." << endl
#ifndef RELEASE_BUILD
         << "  -G OVERRIDES    Overrides for the grey." << endl
#endif
         << "  -V              Operate in vectored mode." << endl
         << "  -N              Operate in block mode (default: streaming)." << endl
#ifdef HS_HYBRID
         << "  -H              Operate in hybrid mode." << endl
#endif
         << "  -L              Pass HS_FLAG_SOM_LEFTMOST for all expressions (default: off)." << endl
         << "  -8              Force UTF8 mode on all patterns." << endl
         << "  -T NUM          Run with NUM threads." << endl
         << "  -h              Display this help." << endl
         << "  -B              Build signature set." << endl
         << "  -C              Check logical combinations (default: off)." << endl
         << "  --literal-on    Processing pure literals, no need to check." << endl
         << endl;
}

static
void processArgs(int argc, char *argv[], UNUSED unique_ptr<Grey> &grey) {
    const char options[] = "e:E:s:z:hHLNV8G:T:BC";
    bool signatureSet = false;
    int literalFlag = 0;

    static struct option longopts[] = {
        {"literal-on", no_argument, &literalFlag, 1},
        {nullptr, 0, nullptr, 0}
    };

    for (;;) {
        int c = getopt_long(argc, argv, options, longopts, nullptr);
        if (c < 0) {
            break;
        }
        switch (c) {
        case 'e':
            g_exprPath.assign(optarg);
            break;
        case 'h':
            usage();
            exit(0);
            break;
        case 's':
            g_signatureFile.assign(optarg);
            break;
        case 'E':
            if (!fromString(optarg, g_editDistance)) {
                usage();
                exit(1);
            }
            g_forceEditDistance = true;
            break;
        case 'z':
            if (!fromString(optarg, g_signature)) {
                usage();
                exit(1);
            }
            signatureSet = true;
            break;
        case '8':
            globalFlags |= HS_FLAG_UTF8;
            break;

#ifndef RELEASE_BUILD
        case 'G':
            applyGreyOverrides(grey.get(), string(optarg));
            break;
#endif
        case 'L':
            globalFlags |= HS_FLAG_SOM_LEFTMOST;
            break;
        case 'N':
            g_streaming = false;
            break;
        case 'V':
            g_streaming = false;
            g_vectored = true;
            break;
        case 'H':
            g_hybrid = true;
            break;
        case 'T':
            num_of_threads = atoi(optarg);
            break;
        case 'B':
            build_sigs = true;
            break;
        case 'C':
            check_logical = true;
            break;
        case 0:
        case 1:
            break;
        default:
            usage();
            exit(1);
        }
    }

    if (g_exprPath.empty() && !g_signatureFile.empty()) {
        /* attempt to infer an expression directory */
        g_exprPath = inferExpressionPath(g_signatureFile);
    }

    if (g_exprPath.empty()) {
        usage();
        exit(1);
    }

    if (!isDir(g_exprPath) && isFile(g_exprPath)
                           && g_signatureFile.empty() && !signatureSet) {
        g_allSignatures = true;
    }

    if (g_signatureFile.empty() && !signatureSet && !g_allSignatures) {
        usage();
        exit(1);
    }

    use_literal_api = (bool)literalFlag;
}

static
void failLine(unsigned lineNum, const string &file,
              const string &line, const string &error) {
    cerr << "Parse error in file " << file
        << " on line " << lineNum << ": " << error
        << endl << "Line is: '" << line << "'" << endl;
    exit(1);
}

// load a list of signature IDs if Build_sigs is enabled
// If a line is commented out, this function still loads the corresponding ID.
// The commented out line should have the format #<space>id<space>#<comment>
// It then prints out a signature file with the IDs that compile successfully.
static
void loadSignatureBuildSigs(const string &inFile,
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
        unsigned id;
        // if line is empty, we can skip it
        if (line.empty()) {
            continue;
        }
        // if line is commented out, try to locate the ID
        // Line is usually in the form #<space>id<space>#<comment>
        if (line[0] == '#') {
            string temp;
            // skip the opening hash and see if there is a second
            size_t comment = line.find_first_of('#', 1);
            if (comment) {
                temp = line.substr(1, comment - 1);
            } else {
                temp = line.substr(1, line.size());
            }
            // cull any whitespace
            boost::trim(temp);

            if (fromString(temp, id)) {
                signatures.push_back(id);
            } else {
                // couldn't be turned into an ID, dump to stdout
                cout << line << endl;
            }
        } else { // lines that don't begin with #
            if (fromString(line, id)) {
                signatures.push_back(id);
            } else {
                // Parse error occurred
                failLine(lineNum, inFile, line, "Unable to parse ID.");
            }
        }
    }
}

int HS_CDECL main(int argc, char **argv) {
    num_of_threads = max(1u, std::thread::hardware_concurrency());

#if !defined(RELEASE_BUILD)
    g_grey = make_unique<Grey>();
#endif
    processArgs(argc, argv, g_grey);

    if (num_of_threads == 0) {
        cout << "Error: Must have at least one thread." << endl;
        exit(1);
    }

    loadExpressions(g_exprPath, g_exprMap);

    if (!g_allSignatures) {
        SignatureSet signatures;
        if (!g_signatureFile.empty()) {
            if (!build_sigs) {
                loadSignatureList(g_signatureFile, signatures);
            } else {
                loadSignatureBuildSigs(g_signatureFile, signatures);
            }
        } else {
            signatures.push_back(g_signature);
        }

        g_exprMap = limitToSignatures(g_exprMap, signatures);
    }

    if (g_exprMap.empty()) {
        cout << "Warning: no signatures to scan. Exiting." << endl;
        exit(0);
    }

    read_it = g_exprMap.begin();
    print_it = g_exprMap.begin();
    vector<thread> threads(num_of_threads);

    for (unsigned int i = 0; i < num_of_threads; i++) {
        threads[i] = thread(checkExpression, nullptr);
    }

    for (unsigned int i = 0; i < num_of_threads; i++) {
        threads[i].join();
    }

    if (check_logical) {
        comb_read_it = g_combs.begin();

        for (unsigned int i = 0; i < num_of_threads; i++) {
            threads[i] = thread(checkLogicalExpression, nullptr);
        }

        for (unsigned int i = 0; i < num_of_threads; i++) {
            threads[i].join();
        }
    }

    if (!g_exprMap.empty() && !build_sigs) {
        cout << "SUMMARY: " << countFailures << " of "
             << g_exprMap.size() << " failed." << endl;
    }
    return 0;
}
