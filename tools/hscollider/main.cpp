/*
 * Copyright (c) 2015-2018, Intel Corporation
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

#include "BoundedQueue.h"
#include "DatabaseProxy.h"
#include "FileCorpora.h"
#include "GraphTruth.h"
#include "GroundTruth.h"
#include "NfaGeneratedCorpora.h"
#include "Thread.h"
#include "UltimateTruth.h"
#include "args.h"
#include "common.h"
#include "cross_compile.h"
#include "expressions.h"
#include "limit.h"
#include "ng_corpus_properties.h"
#include "sig.h"
#include "simple_timer.h"
#include "util/expression_path.h"
#include "util/string_util.h"

#include "grey.h"
#include "hs.h"
#include "parser/utf8_validate.h"
#include "ue2common.h"
#include "util/container.h"
#include "util/make_unique.h"

#include <algorithm>
#include <cassert>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <errno.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>

using namespace std;
using namespace ue2;

unsigned int numThreads = 1;
unsigned int numScannerThreads = 1;
unsigned int numGeneratorThreads = 1;
enum ColliderMode colliderMode = MODE_BLOCK;
bool echo_matches = false;
int g_quiet = 0;
bool g_verbose = false;
bool g_allSignatures = false;
string g_exprPath;
vector<string> g_signatureFiles;
string g_cmdline;
bool g_ue2CompileAll = false;
unsigned g_streamBlocks = 0;
unsigned long long g_streamOffset = 0;
unsigned multicompile_bands = 0;
vector<unsigned> g_signatures;
unsigned long int g_matchLimit = DEFAULT_PCRE_MATCH_LIMIT;
unsigned long int g_matchLimitRecursion = DEFAULT_PCRE_MATCH_RECURSION_LIMIT;
string g_corpora_prefix;
string g_corpora_suffix;
size_t g_memoryLimit = 1000; // megabytes per thread
unsigned int somFlags = 0;
bool loadDatabases = false;
bool saveDatabases = false;
bool saveCorpora = false;
string saveCorporaFile;
string serializePath;
bool force_utf8 = false;
int force_prefilter = 0;
int no_groups = 0;
unsigned somPrecisionMode = HS_MODE_SOM_HORIZON_LARGE;
unsigned limit_matches = 0;
unsigned randomSeed = 0;
bool use_random_alignment = false;
bool use_PCRE = true;
bool use_NFA = true;
bool use_UE2 = true;
bool use_copy_scratch = false;
bool use_copy_stream = false;
bool use_mangle_scratch = false;
bool use_compress_expand = false;
bool use_compress_reset_expand = false;
int abort_on_failure = 0;
int no_signal_handler = 0;
size_t max_scan_queue_len = 25000;
size_t max_generator_queue_len = 25000;
bool force_edit_distance = false;
unsigned edit_distance = 0;
CorpusProperties corpus_gen_prop;

// Semi constants
unsigned min_ue2_align = 0;
unsigned max_ue2_align = MAX_MAX_UE2_ALIGN;

#define DEDUPE_MATCHES

static
unsigned countCores() {
    unsigned count = std::thread::hardware_concurrency();
    return count ? count : 1;
}

// Detect the Address Sanitizer with either GCC or Clang.
#if defined(__SANITIZE_ADDRESS__)
#  define BUILT_WITH_ASAN
#elif defined(__has_feature)
#  if __has_feature(address_sanitizer)
#    define BUILT_WITH_ASAN
#  endif
#endif

// Set the default params that can be overridden with commandline args
static
void setDefaults() {
    // Seed random number generator for corpora
    randomSeed = time(nullptr);
    // Overcommit since we have generators and scanners running.
    numThreads = countCores() * 2;

#ifdef BUILT_WITH_ASAN
    cout << "NOTE: Built with AddressSanitizer.\n"
         << "Defaulting to no memory limit and no signal handler.\n"
         << endl;
    g_memoryLimit = 0;
    no_signal_handler = 1;
#endif
}

static
void exit_with_fail(void) {
    cout << "Failing cmdline was:\n  " << g_cmdline << endl;
    if (abort_on_failure) {
        cout << "Calling abort()" << endl;
        abort();
    }
    exit(1);
}

namespace /* anonymous */ {

// For saving corpora out if the -w flag is specified. Note that we need a
// mutex to serialise writes from different threads.
class CorpusWriter {
public:
    explicit CorpusWriter(const string &filename)
        : out(filename.c_str(), ios_base::trunc) {}

    void write(const string &str) {
        std::lock_guard<std::mutex> lock(mutex);
        out << str << flush;
    }

private:
    ofstream out;
    std::mutex mutex;
};

unique_ptr<CorpusWriter> corporaOut = nullptr;

// Encapsulates all of the data reported from a test
struct TestSummary {
    unsigned totalCorpora = 0;
    unsigned totalExpressions = 0;
    unsigned failCorpora = 0;
    unsigned failPcreCompile = 0;
    unsigned failNGCompile = 0;
    unsigned failUe2Compile = 0;
    unsigned failCompileDifference = 0; // failed in pcre but not ue2
    unsigned failPcreScan = 0;
    unsigned failNGScan = 0;
    unsigned failUe2Scan = 0;
    unsigned failDiff = 0;
    unsigned failNoGroundTruth = 0;
    set<unsigned> failIds;
    set<unsigned> nogtIds;

    // true if we've got a failure
    bool hasFailure() const {
        return failDiff != 0 || !failIds.empty() || failCompileDifference != 0;
    }

    void merge(const TestSummary &a) {
        totalCorpora += a.totalCorpora;
        totalExpressions += a.totalExpressions;
        failCorpora += a.failCorpora;
        failPcreCompile += a.failPcreCompile;
        failNGCompile += a.failNGCompile;
        failUe2Compile += a.failUe2Compile;
        failCompileDifference += a.failCompileDifference;
        failPcreScan += a.failPcreScan;
        failNGScan += a.failNGScan;
        failUe2Scan += a.failUe2Scan;
        failDiff += a.failDiff;
        failNoGroundTruth += a.failNoGroundTruth;
        failIds.insert(begin(a.failIds), end(a.failIds));
        nogtIds.insert(begin(a.nogtIds), end(a.nogtIds));
    }
};

enum TestResult {
    TEST_NO_GROUND_TRUTH,
    TEST_PASSED,
    TEST_SKIPPED,
    TEST_FAILED_COMPILE,
    TEST_FAILED
};

struct TestUnit {
    shared_ptr<CompiledPcre> pcre; // libpcre bytecode
    shared_ptr<CNGInfo> cngi; // NFA graph info (compilation is deferred)
    shared_ptr<DatabaseProxy> ue2; // ue2 bytecode
    Corpus corpus; // a local copy, as we may modify it

    unsigned id; // expression id
    unsigned corpus_id; // corpus id
    bool highlander; // single match flag
    bool prefilter; // prefilter flag
    bool som; // start of match flag
    bool multi; // if false, we're in single mode.
    bool utf8; // at least one of our patterns is utf8

    enum TestResult result;

    TestUnit(unsigned sig_id, unsigned c_id, const Corpus &c,
             shared_ptr<CompiledPcre> pcre_in, shared_ptr<CNGInfo> cngi_in,
             shared_ptr<DatabaseProxy> ue2_in, bool multi_in, bool utf8_in,
             bool highlander_in, bool prefilter_in, bool som_in)
        : pcre(pcre_in), cngi(cngi_in), ue2(ue2_in), corpus(c), id(sig_id),
          corpus_id(c_id), highlander(highlander_in), prefilter(prefilter_in),
          som(som_in), multi(multi_in), utf8(utf8_in),
          result(TEST_NO_GROUND_TRUTH) {}
};

} // namespace

// For ease of printing match sets
static
std::ostream &operator<<(std::ostream &os, const set<MatchResult> &v) {
    auto vi = v.begin(), ve = v.end();
    while (vi != ve) {
        // match offsets
        os << '(' << vi->from << ',' << vi->to << ')';
        if (++vi != ve) {
            os << ", ";
        }
    }
    return os;
}

static
void printCorpus(ostream &out, const Corpus &corpus) {
    // Print the offending corpus
    string corpus_data(corpus.data.begin() + g_corpora_prefix.size(),
                       corpus.data.end()   - g_corpora_suffix.size());
    bool trimmed = false;
    if (corpus_data.size() > 1000) {
        corpus_data.resize(1000);
        trimmed = true;
    }
    out << "  Corpus data: '" << printable(corpus_data) << "'";
    if (trimmed) {
        out << " ...";
    }
    out << "\n";
}

static
void printGroundTruthDifference(ostream &out, const ExpressionMap &exprMap,
                                const TestUnit &unit,
                                const ResultSet &pcre_results,
                                const ResultSet &ngw_results) {
    assert(contains(exprMap, unit.id));
    // Print the expression itself
    out << "  Expression: '" << exprMap.at(unit.id) << "'\n";
    printCorpus(out, unit.corpus);
    out << "  PCRE matches: " << pcre_results.matches << "\n";
    out << "  NFA matches: " << ngw_results.matches << "\n";

    vector<MatchResult> diff;

    set_difference(pcre_results.matches.begin(), pcre_results.matches.end(),
                   ngw_results.matches.begin(), ngw_results.matches.end(),
                   back_inserter(diff));

    for (const auto &match : diff) {
        out << "  PCRE only: match (" << match.from << "," << match.to << ")\n";
    }

    diff.clear();

    set_difference(ngw_results.matches.begin(), ngw_results.matches.end(),
                   pcre_results.matches.begin(), pcre_results.matches.end(),
                   back_inserter(diff));

    for (const auto &match : diff) {
        out << "  NFA only: match (" << match.from << "," << match.to << ")\n";
    }
    out.flush();
}

// Report the difference information when a pattern causes different matches in
// our engines.
static
void printDifference(ostream &out, const ExpressionMap &exprMap,
                     const TestUnit &unit, const ResultSet &gt_results,
                     const vector<ResultSet> &ue2_results,
                     const vector<bool> &pass) {
    assert(contains(exprMap, unit.id));
    // Print the expression itself
    out << "  Expression: '" << exprMap.at(unit.id) << "'\n";
    printCorpus(out, unit.corpus);
    out << "  " << gt_results.src << " matches: " << gt_results.matches << endl;

    for (u32 align = min_ue2_align; align < max_ue2_align; align++) {
        if (pass[align]) {
            continue;
        }

        u32 align_in = align;
        out << "  UE2 (" << align;
        while (align + 1 < max_ue2_align) {
            if (pass[align + 1] ||
                ue2_results[align] != ue2_results[align + 1]) {
                break;
            }
            align++;
        }

        if (align != align_in) {
            out << " - " << align;
        }

        out << ") matches: " << ue2_results[align].matches;
        out << endl;

        vector<MatchResult> only;

        // Print matches only returned by ground truth
        set_difference(gt_results.matches.begin(),
                       gt_results.matches.end(),
                       ue2_results[align].matches.begin(),
                       ue2_results[align].matches.end(),
                       back_inserter(only));
        for (const auto &match : only) {
            out << "  " << gt_results.src << " only: match ("
                << match.from << "," << match.to << ')' << endl;
        }

        // Print matches only returned by UE2
        only.clear();

        set_difference(ue2_results[align].matches.begin(),
                       ue2_results[align].matches.end(),
                       gt_results.matches.begin(),
                       gt_results.matches.end(),
                       back_inserter(only));

        for (const auto &match : only) {
            out << "  UE2 only: match (" << match.from << "," << match.to << ')'
                << endl;
        }

#ifdef DEDUPE_MATCHES
        for (const auto &match : ue2_results[align].dupe_matches) {
            out << "  UE2 dupe:  match (" << match.from << "," << match.to
                << ')' << endl;
        }
#endif

        if (ue2_results[align].uoom) {
            out << "  *** UE2 produced matches out of order" << endl;
        }
        if (ue2_results[align].match_after_halt) {
            out << "  *** UE2 produced matches after termination" << endl;
        }
        if (ue2_results[align].invalid_id) {
            out << "  *** UE2 produced matches for invalid ids" << endl;
        }
    }
}

static
void printMode(void) {
    if (!g_ue2CompileAll) {
        cout << "Single/";
    } else if  (!multicompile_bands) {
        cout << "Multi/";
    } else {
        cout << "Multi-" << multicompile_bands << "/";
    }

    switch (colliderMode) {
        case MODE_BLOCK:
            cout << "Block";
            break;
        case MODE_STREAMING:
            cout << "Streaming-" << g_streamBlocks;
            if (g_streamOffset) {
                cout << " offset " << g_streamOffset;
            }
            if (use_copy_stream) {
                cout << " [copy stream]";
            }
            if (use_compress_expand) {
                cout << " [compress]";
            }
            if (use_compress_reset_expand) {
                cout << " [compress+reset]";
            }
            break;
        case MODE_VECTORED:
            cout << "Vectored-" << g_streamBlocks;
            break;
        case MODE_HYBRID:
            cout << "Hybrid";
            break;
    }

    if (use_copy_scratch) {
        cout << " [copy scratch]";
    }
    if (use_mangle_scratch) {
        cout << " [mangle]";
    }
    cout << endl;
}

static
void printSummaryV(const TestSummary &sum) {
    cout << endl;
    cout << "Summary:" << endl;
    cout << "Mode:                           ";
    printMode();
    cout << "=========" << endl;
    cout << "Expressions processed:          " << sum.totalExpressions << endl;
    cout << "Corpora processed:              " << sum.totalCorpora << endl;
    cout << "Expressions with failures:      " << sum.failIds.size() << endl;
    cout << "  Corpora generation failures:  " << sum.failCorpora << endl;
    cout << "  Compilation failures:         ";
    cout << "pcre:" << sum.failPcreCompile << ", ";
    cout << "ng:" << sum.failNGCompile << ", ";
    cout << "ue2:" << sum.failUe2Compile << endl;

    cout << "  Matching failures:            ";
    cout << "pcre:" << sum.failPcreScan << ", ";
    cout << "ng:" << sum.failNGScan << ", ";
    cout << "ue2:" << sum.failUe2Scan << endl;
    cout << "  Match differences:            " << sum.failIds.size() << endl;
    cout << "  No ground truth:              " << sum.nogtIds.size() << endl;
    cout << "Total match differences:        " << sum.failDiff << endl;
}

static
void printSummaryQ(const TestSummary &sum) {
    cout << "Summary:     ";
    printMode();

    cout << "Processed:   " << sum.totalExpressions << " expressions, "
         << sum.totalCorpora << " corpora" << endl;
    cout << "Failures:    " << sum.failIds.size()
         << " (corpora:   " << sum.failCorpora << "; compile: ";
    cout << "pcre:" << sum.failPcreCompile << ", ";
    cout << "ng:" << sum.failNGCompile << ", ";
    cout << "ue2:" << sum.failUe2Compile << "; match: ";

    cout << "pcre:" << sum.failPcreScan << ", ";
    cout << "ng:" << sum.failNGScan << ", ";
    cout << "ue2:" << sum.failUe2Scan << ")" << endl;
    cout << "Differences: " << sum.failIds.size() << " expressions, "
         << sum.failDiff << " total" << endl;
    cout << "No ground truth: " << sum.nogtIds.size() << " expressions" << endl;
}

static
void printSummary(const TestSummary &sum) {
    if (g_quiet > 1) {
        printSummaryQ(sum);
    } else {
        printSummaryV(sum);
    }
}

// Returns true if this Highlander mode test succeeded.
static
bool checkSingleMatch(const ResultSet &ground_truth, const ResultSet &ue2) {
    // In Highlander (single-match) mode, UE2 must return only one of the
    // matches returned by PCRE/GraphTruth. It need not be the earliest one.
    if (ground_truth.matches.empty()) {
        return ue2.matches.empty();
    } else if (ue2.matches.size() != 1) {
        return false;
    } else {
        return contains(ground_truth.matches, *ue2.matches.begin());
    }
}

// Returns true if this prefiltering mode test succeeded.
static
bool checkPrefilterMatch(const ResultSet &ground_truth, const ResultSet &ue2,
                         bool highlander) {
    if (highlander) {
        // Highlander + prefilter is tricky. Best we can do is say that if PCRE
        // returns matches, UE2 must return a match, though it may not be one
        // of the ones returned by PCRE (it may be an earlier match).
        if (!ground_truth.matches.empty()) {
            return ue2.matches.size() == 1;
        }
        // We can't verify anything more.
        return true;
    } else if (!limit_matches || ue2.matches.size() < limit_matches) {
        // In prefilter mode, every match found by PCRE must be found by UE2,
        // but the UE2 set may be a superset of the PCRE match set.
        return std::includes(ue2.matches.begin(), ue2.matches.end(),
                ground_truth.matches.begin(), ground_truth.matches.end());
    }

    // Otherwise, we've hit our match limit. Prefilter mode is quite difficult
    // to verify in this case, so we just verify that "something happened".
    return true;
}

static
ResultSet makeEndOfMatchOnly(const ResultSet &rs) {
    ResultSet out(rs.src);
    for (const auto &match : rs.matches) {
        out.addMatch(0, match.to);
    }
    return out;
}

static
bool checkMultiMatch(const ResultSet &ground_truth, const ResultSet &ue2) {
    // If we had out-of-order matches or matches after termination, we have a
    // bug!
    if (ue2.uoom || ue2.match_after_halt || ue2.invalid_id) {
        return false;
    }

    // If we have more UE2 matches than our limit, we have a bug!
    if (limit_matches && ue2.matches.size() > limit_matches) {
        return false;
    }

    // If we have more UE2 matches than PCRE matches, we have a bug!
    if (ue2.matches.size() > ground_truth.matches.size()) {
        return false;
    }

    // If we've got fewer matches than our limit to test, then the match sets
    // must be identical.
    if (!limit_matches || ground_truth.matches.size() < limit_matches) {
        return ground_truth == ue2;
    }

    // We're in limit_matches mode _and_ we have hit the limit.  Every match in
    // 'ue2' must be in 'pcre'. (We can't just trim pcre and do an equality
    // test as matches may come out of UE2 a little out of order.)

    // In streaming mode, the limit may mean that we get a different SOM from
    // the leftmost one. So we compare only end offsets.
    if (colliderMode == MODE_STREAMING || colliderMode == MODE_VECTORED) {
        ResultSet gt_eom = makeEndOfMatchOnly(ground_truth);
        ResultSet ue2_eom = makeEndOfMatchOnly(ue2);
        return std::includes(gt_eom.matches.begin(), gt_eom.matches.end(),
                             ue2_eom.matches.begin(), ue2_eom.matches.end());
    }

    return std::includes(ground_truth.matches.begin(),
                         ground_truth.matches.end(),
                         ue2.matches.begin(), ue2.matches.end());
}

// Check results, returns true if there has any failure.
static
bool checkTestResults(ostream &out, TestSummary &summary,
                      const ExpressionMap &exprMap, TestUnit &unit,
                      const ResultSet &gt_results,
                      const vector<ResultSet> &ue2_results) {
    bool failed = false;
    bool any_fail = false;
    vector<bool> pass(max_ue2_align, false);

    for (unsigned align = min_ue2_align; align != max_ue2_align; ++align) {
        if (unit.prefilter) {
            failed = !checkPrefilterMatch(gt_results, ue2_results[align],
                                          unit.highlander);
        } else if (unit.highlander) {
            failed = !checkSingleMatch(gt_results, ue2_results[align]);
        } else {
            // In non-Highlander mode, the two result sets MUST be equal
            // don't check PCRE if the scan didn't succeed
            failed = !checkMultiMatch(gt_results, ue2_results[align]);
        }

#ifdef DEDUPE_MATCHES
        if (!failed) {
            failed |= !ue2_results[align].dupe_matches.empty();
        }
#endif

        pass[align] = !failed;

        any_fail |= failed;

        summary.failDiff += failed ? 1 : 0;

        if (g_verbose) {
            if (failed) {
                out << "FAILED: id " << unit.id << ", alignment " << align
                    << ", corpus " << unit.corpus_id << ", results differ"
                    << endl;
            } else {
                out << "PASSED: id " << unit.id << ", alignment " << align
                    << ", corpus " << unit.corpus_id
                    << " (matched "<< gt_results.src << ":"
                    << gt_results.matches.size()
                    << ", ue2:" << ue2_results[align].matches.size() << ")"
                    << endl;
            }
        }
    }

    if (!any_fail) {
        return false;
    }

    if (!g_verbose) {
        out << "FAILED: id " << unit.id << ", alignment";
        for (unsigned align = min_ue2_align; align != max_ue2_align; ++align) {
            if (!pass[align]) {
                out << " " << align;

                if (align + 1 < max_ue2_align && !pass[align + 1]) {
                    while (align + 1 < max_ue2_align && !pass[align + 1]) {
                        align++;
                    }

                    out << "-" << align;
                }
            }
        }

        out << ", corpus " << unit.corpus_id << ", results differ" << endl;
    }
    printDifference(out, exprMap, unit, gt_results, ue2_results, pass);

    return true;
}

// Construct a UE2 database, taking care of loading/saving to disk when
// appropriate
static
shared_ptr<DatabaseProxy> constructDatabase(const set<unsigned int> &ids,
                                            const UltimateTruth &ultimate) {
    assert(!ids.empty());

    if (loadDatabases) {
        string filename = ultimate.dbFilename(ids);
        shared_ptr<BaseDB> db = ultimate.loadDatabase(filename, ids);
        if (!db) {
            if (!g_quiet) {
                cout << "FAILED: could not load database " << filename << endl;
            }
            return nullptr;
        }
        return make_shared<DatabaseProxy>(db);
    }

    shared_ptr<DatabaseProxy> ue2 = make_shared<DatabaseProxy>(ids);

    try {
        // If we're not runnable (i.e. we're cross-compiling), let's at least
        // try to build the database.
        if (!ultimate.runnable()) {
            shared_ptr<BaseDB> db = ue2->get(ultimate);
            assert(db); // throws otherwise
        }

        // Compile and save if we've been told to.
        if (saveDatabases) {
            string filename = ultimate.dbFilename(ids);
            if (!ultimate.saveDatabase(*(ue2->get(ultimate)),
                                       filename.c_str())) {
                cout << "FAILED: could not save database to file: " << filename
                     << endl;
            }
        }
    } catch (const CompileFailed &fail) {
        if (!g_quiet) {
            cout << "FAILED: ue2 compile failed for " << *ids.begin() << ": "
                 << fail.error << endl;
        }
        // Return null database to indicate failure.
        ue2 = nullptr;
    }

    return ue2;
}

static
bool getGraphTruth(ostream &out, CNGInfo &cngi, GraphTruth &graph,
                    TestUnit &unit, ResultSet &ngw_results,
                    TestSummary &summary, const string &expression) {
    debug_stage = STAGE_GRAPH_RUN;

    // Skip patterns we've previously marked as bad.
    if (cngi.is_bad()) {
        summary.failNGScan++;
        return false;
    }

    // If we already have match information for this corpus, we don't need to
    // run PCRE at all. At the moment our on-disk format for corpora with match
    // information only includes the end-of-match offset, so we only use these
    // in non-som modes. If edit distance is forced, all bets are off so we
    // ignore this as well.
    if (!g_streamOffset && unit.corpus.hasMatches && !force_utf8 && !cngi.som &&
        !force_edit_distance) {
        if (g_verbose) {
            out << "Using corpus match set rather than NFA graph" << endl;
        }
        ngw_results = ResultSet(unit.corpus.matches, RESULT_FROM_GRAPH);
    } else {
        // compile the actual graph
        const CompiledNG *cng;
        try {
            debug_stage = STAGE_GRAPH_COMPILE;
            cng = cngi.get();
            debug_stage = STAGE_UNDEFINED;
        }
        catch (const NGCompileFailure &err) {
            debug_stage = STAGE_UNDEFINED;
            summary.failNGCompile++;
            summary.failNGScan++;
            cngi.mark_bad();
            if (!g_quiet) {
                cout << "FAILED: id " << unit.id
                     << ", NFA graph compile failed (" << err.msg << ")"
                     << endl;
            }
            return false;
        }
        debug_stage = STAGE_GRAPH_RUN;

        // Run NFA graph and collect match information.
        string error;
        assert(cng);
        if (!graph.run(unit.id, *cng, cngi, unit.corpus.data, ngw_results,
                       error)) {
            if (!g_quiet) {
                out << "FAILED: id " << unit.id
                    << ", NFA graph scan failed: " << error << "\n"
                    << "  Expression: '" << expression << "'\n"
                    << "  Corpus data: '" << printable(unit.corpus.data)
                    << "'\n"
                    << "  (note: marking bad, skipping subsequent tests)"
                    << endl;
            }
            summary.failNGScan++;
            cngi.mark_bad();
            return false;
        }
    }

    return true;
}

static
bool getGroundTruth(ostream &out, CompiledPcre &cpcre, GroundTruth &ground,
                    TestUnit &unit, ResultSet &pcre_results,
                    TestSummary &summary) {
    debug_stage = STAGE_PCRE_RUN;

    // Skip patterns we've previously marked as bad.
    if (cpcre.is_bad()) {
        summary.failPcreScan++;
        return false;
    }

    // If we already have match information for this corpus, we don't need to
    // run PCRE at all. At the moment our on-disk format for corpora with match
    // information only includes the end-of-match offset, so we only use these
    // in non-som modes. Also, we can't trust corpus matches if there was an
    // edit distance requested for all patterns.
    if (!g_streamOffset && unit.corpus.hasMatches && !force_utf8 && !cpcre.som
        && !force_edit_distance) {
        if (g_verbose) {
            out << "Using corpus match set rather than PCRE" << endl;
        }
        pcre_results = ResultSet(unit.corpus.matches, RESULT_FROM_PCRE);
    } else {
        // Run PCRE and collect match information.
        string error;
        if (!ground.run(unit.id, cpcre, unit.corpus.data, pcre_results,
                        error)) {
            if (!g_quiet) {
                out << "FAILED: id " << unit.id
                    << ", libpcre scan failed: " << error << "\n"
                    << "  Expression: '" << cpcre.expression << "'\n"
                    << "  Corpus data: '" << printable(unit.corpus.data)
                    << "'\n"
                    << "  (note: marking PCRE bad, skipping subsequent tests)"
                    << endl;
            }
            summary.failPcreScan++;
            cpcre.mark_bad();
            return false;
        }
    }

    return true;
}

static
void writeCorpus(unsigned id, const Corpus &corpus, const ResultSet &results) {
        assert(corporaOut);
        ostringstream oss;
        oss << id << "=\"" << printable(corpus.data) << "\": ";

        auto vi = results.matches.begin();
        auto ve = results.matches.end();

        // Print match end offsets only.
        while (vi != ve) {
            oss << vi->to;
            if (++vi != ve) {
                oss << ",";
            }
        }
        oss << "\n";
        corporaOut->write(oss.str());
}

static
void runTestUnit(ostream &out, GroundTruth &ground, GraphTruth &graph,
                 UltimateTruth &ultimate, TestUnit &unit, TestSummary &summary,
                 const ExpressionMap &exprMap) {
    assert(use_UE2);
    Corpus &corpus = unit.corpus;

    shared_ptr<const BaseDB> db;
    if (use_UE2) {
        // Acquire UE2 database.
        debug_stage = STAGE_UE2_COMPILE;
        try {
            db = unit.ue2->get(ultimate);
        } catch (const CompileFailed &fail) {
            summary.failUe2Compile++;
            if (!g_quiet) {
                out << "FAILED: ue2 compile failed for " << unit.id << ": "
                    << fail.error << endl;
                unit.result = TEST_FAILED_COMPILE;
                debug_stage = STAGE_UNDEFINED;
                return;
            }
        }
        debug_stage = STAGE_UNDEFINED;

        if (!db) {
            // Database previously failed compilation.
            unit.result = TEST_SKIPPED;
            return;
        }
    }

    // If the user has specified that they want prefix/suffix data added to
    // their corpora, we do it here; this is as local as possible to the
    // test, so we don't keep piles of HUGE corpora hanging around.
    if (!g_corpora_prefix.empty()) {
        corpus.data.insert(0, g_corpora_prefix);
        corpus.hasMatches = false;
    }
    if (!g_corpora_suffix.empty()) {
        corpus.data.append(g_corpora_suffix);
        corpus.hasMatches = false;
    }

    ResultSet gt_results(RESULT_FROM_PCRE);
    vector<ResultSet> ue2_results(max_ue2_align, ResultSet(RESULT_FROM_UE2));

    bool gt_done = false;

    // run PCRE test if enabled and if compile succeeded
    if (unit.pcre && use_PCRE) {
        gt_done = getGroundTruth(out, *unit.pcre, ground, unit, gt_results,
                                 summary);
    }

    // run NFA if PCRE failed (or wasn't run), or if we don't run UE2
    if (unit.cngi && (use_NFA && !gt_done)) {
        gt_done = getGraphTruth(out, *unit.cngi, graph, unit, gt_results,
                                summary, exprMap.find(unit.id)->second);
    }

    // both ground truth methods either failed or didn't run
    if (!gt_done) {
        unit.result = TEST_NO_GROUND_TRUTH;
        return;
    }

    // Write out corpora if we've been told to
    if (saveCorpora) {
        writeCorpus(unit.id, unit.corpus, gt_results);
    }

    debug_stage = STAGE_UE2_RUN;
    for (unsigned int align = min_ue2_align; align != max_ue2_align; ++align) {
        bool ok = ultimate.run(unit.id, db, corpus.data, !unit.multi, align,
                               ue2_results[align]);

        if (!ok) {
            if (!g_quiet) {
                out << "FAILED: id " << unit.id << ", ue2 scan at alignment "
                    << align << " failed" << endl;
            }
            unit.result = TEST_FAILED;
            debug_stage = STAGE_UNDEFINED;
            return;
        }
    }

    // if we're using UE2, check all the different results modes
    if (checkTestResults(out, summary, exprMap, unit, gt_results,
                         ue2_results)) {
        unit.result = TEST_FAILED;
    } else {
        unit.result = TEST_PASSED;
    }

    debug_stage = STAGE_UNDEFINED;
}

/* Used for testing the graph truth agains PCE */
static
void runGroundCompTestUnit(ostream &out, GroundTruth &ground, GraphTruth &graph,
                           TestUnit &unit, TestSummary &summary,
                           const ExpressionMap &exprMap) {
    assert(!use_UE2);
    assert(use_PCRE);
    assert(use_NFA);
    Corpus &corpus = unit.corpus;

    // If the user has specified that they want prefix/suffix data added to
    // their corpora, we do it here; this is as local as possible to the
    // test, so we don't keep piles of HUGE corpora hanging around.
    if (!g_corpora_prefix.empty()) {
        corpus.data.insert(0, g_corpora_prefix);
        corpus.hasMatches = false;
    }
    if (!g_corpora_suffix.empty()) {
        corpus.data.append(g_corpora_suffix);
        corpus.hasMatches = false;
    }

    ResultSet pcre_results(RESULT_FROM_PCRE);
    ResultSet ngw_results(RESULT_FROM_GRAPH);

    bool pcreResult = false;
    bool graphResult = false;

    if (unit.pcre) {
        pcreResult = getGroundTruth(out, *unit.pcre, ground, unit, pcre_results,
                                    summary);
    }

    if (unit.cngi) {
        graphResult = getGraphTruth(out, *unit.cngi, graph, unit, ngw_results,
                                    summary, exprMap.find(unit.id)->second);
    }

    // no ground truth found either NFA or PCRE failed
    if (!pcreResult || !graphResult) {
        unit.result = TEST_NO_GROUND_TRUTH;
        return;
    }

    // Write out corpora if we've been told to
    if (saveCorpora) {
        writeCorpus(unit.id, unit.corpus, pcre_results);
    }

    if (pcre_results.matches != ngw_results.matches) {
        unit.result = TEST_FAILED;
        out << "FAILED: id " << unit.id << ", corpus " << unit.corpus_id
            << ", results differ" << endl;

        printGroundTruthDifference(out, exprMap, unit, pcre_results,
                                   ngw_results);
    } else {
        unit.result = TEST_PASSED;
        if (g_verbose) {
            out << "PASSED: id " << unit.id << ", corpus " << unit.corpus_id
                << " (matched pcre:" << pcre_results.matches.size()
                << ", matched ng:" << ngw_results.matches.size() << ")" << endl;
        }
    }

    debug_stage = STAGE_UNDEFINED;
}

static
void addCorporaToQueue(ostream &out, BoundedQueue<TestUnit> &testq, unsigned id,
                       CorporaSource &corpora, TestSummary &summary,
                       shared_ptr<CompiledPcre> cpcre, shared_ptr<CNGInfo> cngi,
                       shared_ptr<DatabaseProxy> ue2, bool multi, bool utf8) {
    // build corpora
    vector<Corpus> c;
    try {
        corpora.generate(id, c);
    }
    catch (CorpusFailure &err) {
        if (!g_quiet) {
            out << "FAILED: id " << id << ", corpora failure: " << err.message
                << endl;
        }
        summary.failCorpora++;
        return;
    }

    const bool som = cpcre ? cpcre->som : cngi->som;
    const bool prefilter = cpcre ? cpcre->prefilter : cngi->prefilter;
    const bool highlander = cpcre ? cpcre->highlander : cngi->highlander;

    // If we're in UTF-8 mode and the corpus isn't valid UTF-8, skip it:
    // Hyperscan's behaviour when scanning invalid UTF-8 data in UTF-8 mode
    // is undefined.
    if (utf8) {
        auto is_invalid_utf8 = [](const Corpus &corpus) {
            return !isValidUtf8(corpus.data.c_str(), corpus.data.size());
        };
        c.erase(remove_if(begin(c), end(c), is_invalid_utf8), end(c));
    }

    // Collect together corpora units in a container so that we don't have to
    // repeatedly lock the queue.
    vector<unique_ptr<TestUnit>> tests;
    tests.reserve(c.size());

    size_t corpus_id = 0;
    for (const Corpus &corpus : c) {
        tests.push_back(ue2::make_unique<TestUnit>(id, corpus_id, corpus, cpcre,
                                                   cngi, ue2, multi, utf8,
                                                   highlander, prefilter, som));
        corpus_id++;
    }

    testq.push(begin(tests), end(tests));
}

namespace /* anonymous */ {

// A subclass of Thread that stores its own output in a stringstream, flushing
// it to cout when necessary.
class OutputThread : public Thread {
public:
    OutputThread(size_t id) : Thread(id) {}
    ~OutputThread() override {
        flush_output();
    }

protected:
    void flush_output() {
        const string &s = out.str();
        if (!s.empty()) {
            cout << s;
            out.str(""); // make empty
        }
    }

    // Output stream, flushed to cout after every test unit.
    stringstream out;
};

class ScanThread : public OutputThread {
public:
    ScanThread(size_t id, BoundedQueue<TestUnit> &testq, const ExpressionMap &e,
               const hs_platform_info *plat, const Grey &grey)
        : OutputThread(id), q(testq),
          ground(out, e, g_matchLimit, g_matchLimitRecursion), graph(out, e),
          ultimate(out, e, plat, grey, g_streamBlocks), exprMap(e) {}

    void run() override {
        DEBUG_PRINTF("thread %zu running\n", thread_id);
        for (;;) {
            const auto unit = q.pop(thread_id);
            if (!unit) {
                // Sentinel value, indicates that we have run out of units to
                // process.
                DEBUG_PRINTF("thread %zu stopped\n", thread_id);
                break;
            }

            assert(unit);
            assert(exprMap.find(unit->id) != exprMap.end());

            // Debug information is stored in TLS and (hopefully) printed out in
            // the event of a crash.
            debug_expr = unit->id;
            debug_corpus = unit->corpus_id;
            debug_corpus_ptr = unit->corpus.data.c_str();
            debug_corpus_len = unit->corpus.data.size();
            debug_expr_ptr = exprMap.find(unit->id)->second.c_str();

            if (use_UE2) {
                runTestUnit(out, ground, graph, ultimate, *unit, summary,
                            exprMap);
            } else {
                runGroundCompTestUnit(out, ground, graph, *unit, summary,
                                      exprMap);
            }

            if (unit->result == TEST_NO_GROUND_TRUTH) {
                summary.nogtIds.insert(unit->id);
                // this is fine, continue
            } else if (unit->result == TEST_FAILED) {
                summary.failIds.insert(unit->id);
            }

            count++;
            summary.totalCorpora++;
            flush_output();
        }
    }

    const TestSummary &getSummary() const { return summary; }

public:
    size_t count = 0; // number of units processed

private:
    // Shared queue.
    BoundedQueue<TestUnit> &q;

    // Thread-local data.
    GroundTruth ground; // independent copy
    GraphTruth graph; // independent copy
    UltimateTruth ultimate; // independent copy
    TestSummary summary;

    // Constant shared data.
    const ExpressionMap &exprMap;
};

/** Represent a work item for the corpus generation threads. This contains
 *  all information relating to an expression. The corpus generator will
 *  generate corpora for this expression and enqueue work items representing
 *  complete test cases for the scanning threads.
 */
struct CorpusGenUnit {
    CorpusGenUnit(unique_ptr<CNGInfo> cngi_in, unique_ptr<CompiledPcre> pcre_in,
               shared_ptr<DatabaseProxy> ue2_in, unsigned expr_id,
               bool multi_in, bool utf8_in)
        : cngi(move(cngi_in)), pcre(move(pcre_in)), ue2(ue2_in), id(expr_id),
          multi(multi_in), utf8(utf8_in) {}

    unique_ptr<CNGInfo> cngi;
    unique_ptr<CompiledPcre> pcre;

    /* ue2 shared_ptr as in multicompile and banded compile it is shared amongst
     * various corpus units (with differing expression ids). */
    shared_ptr<DatabaseProxy> ue2;

    unsigned id; // expression id
    bool multi; // ue2 contains more than one expression
    bool utf8; // ue2 can be run against utf8 corpora
};

class CorpusGenThread : public OutputThread {
public:
    CorpusGenThread(size_t id, BoundedQueue<TestUnit> &testq_in,
                    BoundedQueue<CorpusGenUnit> &corpq_in,
                    const CorporaSource &corpora_in)
        : OutputThread(id), testq(testq_in), corpq(corpq_in),
          corpora(corpora_in.clone()) {}

    void run() override {
        DEBUG_PRINTF("thread %zu running\n", thread_id);
        for (;;) {
            auto c = corpq.pop(thread_id);
            if (!c) {
                break;
            }

            addCorporaToQueue(out, testq, c->id, *corpora, summary,
                              move(c->pcre), move(c->cngi), c->ue2, c->multi,
                              c->utf8);

            count++;
            flush_output();
        }
    }

    const TestSummary &getSummary() const { return summary; }

public:
    size_t count = 0; // number of units processed

private:
    // Output queue, shared between threads.
    BoundedQueue<TestUnit> &testq;

    // Input queue, shared between corpus generator threads.
    BoundedQueue<CorpusGenUnit> &corpq;

    // Thread-local data.
    const unique_ptr<CorporaSource> corpora; // independent copy
    TestSummary summary;
};

} // namespace

static
unique_ptr<CNGInfo> makeNGInfo(const unsigned id, TestSummary &summary,
                               GraphTruth &graph, UltimateTruth &ultimate,
                               shared_ptr<DatabaseProxy> ue2) {
    string nfaErr;

    try {
        debug_stage = STAGE_GRAPH_PREPROCESS;
        auto cngi = graph.preprocess(id);
        debug_stage = STAGE_UNDEFINED;
        return cngi;
    }
    catch (const NGCompileFailure &err) {
        nfaErr = err.msg;
        debug_stage = STAGE_UNDEFINED;
        // fall through
    }
    catch (const NGUnsupportedFailure &err) {
        // unsupported error happens when the pattern appears to be valid, but
        // there are things that we don't yet support (e.g. SOM).
        // in this case, try again, suppressing the errors
        debug_stage = STAGE_UNDEFINED;
        summary.failNGCompile++;

        // try again and suppress unsupported errors
        try {
            debug_stage = STAGE_GRAPH_PREPROCESS;
            auto cngi = graph.preprocess(id, true);
            debug_stage = STAGE_UNDEFINED;

            // preprocess succeeded - that means the pattern itself is valid.
            // however, we can't use it, so we have to mark it as bad
            // only print the error in the following cases:
            // 1) if verbose is specified
            // 2) if we are not using UE2 and quiet is NOT specified
            if ((!use_UE2 && !g_quiet) || g_verbose) {
                cout << "FAILED: id " << id << ", NFA graph preprocess failed ("
                     << err.msg << ")" << endl;
            }
            cngi->mark_bad();
            return cngi;
        }
        catch (const NGCompileFailure &e) {
            // compile failed
            nfaErr = e.msg;
            debug_stage = STAGE_UNDEFINED;
            // fall through
        }
    }

    // We should ensure that we also fail compilation with UE2, otherwise we
    // likely have a pattern support bug.
    try {
        auto db = ue2->get(ultimate);
        if (db) {
            // if we made it this far, that means UE2 compile succeeded while
            // NFA compile failed.
            cout << "FAILED: id " << id << ", NFA graph preprocess failed ("
                 << nfaErr << ") but UE2 compile succeeded." << endl;
            summary.failNGCompile++;
            summary.failCompileDifference++;
            return nullptr;
        }
        // If db is nullptr, we have previously failed compilation of this
        // database.
    }
    catch (const CompileFailed &) {
        // Everything's OK: compilation failed in Hyperscan as well. Fall
        // through.
    }
    summary.failNGCompile++;
    if (!g_quiet) {
        cout << "FAILED: id " << id << ", NFA graph preprocess failed ("
             << nfaErr << ")" << endl;
    }
    return nullptr;
}

static
unique_ptr<CompiledPcre> makePcre(const unsigned id, TestSummary &summary,
                                  GroundTruth &ground, UltimateTruth &ultimate,
                                  shared_ptr<DatabaseProxy> ue2) {
    string pcreErr;

    try {
        debug_stage = STAGE_PCRE_COMPILE;
        auto cpcre = ground.compile(id);
        debug_stage = STAGE_UNDEFINED;
        return cpcre;
    }
    catch (const SoftPcreCompileFailure &err) {
        debug_stage = STAGE_UNDEFINED;
        summary.failPcreCompile++;
        if (g_verbose) {
            cout << "FAILED: id " << id
                 << ", libpcre compile failed with soft error: " << err.msg
                 << endl;
        }
        return nullptr;
    }
    catch (const PcreCompileFailure &err) {
        debug_stage = STAGE_UNDEFINED;
        pcreErr = err.msg;
        // fall through
    }

    // We should ensure that we also fail compilation with UE2, otherwise we
    // likely have a pattern support bug.
    try {
        auto db = ue2->get(ultimate);
        if (db) {
            // OK, so now we have a situation: PCRE failed but UE2 succeeded.
            // There is one situation where this is legal: patterns beginning
            // with (*UTF8), which will throw an error due to the callback
            // wrapping we do for PCRE. We can check these by trying to compile
            // an "unwrapped" PCRE.
            ground.compile(id, true);
            // If we didn't throw, PCRE failed above but succeeded when not
            // wrapped in a callback, and UE2 succeeded. Not worth reporting,
            // fall through.
        }
    }
    catch (const CompileFailed &) {
        // Everything's OK: compilation failed in Hyperscan as well. Fall
        // through.
    }
    catch (const PcreCompileFailure &) {
        cout << "FAILED: id " << id << ", libpcre compile failed (" << pcreErr
             << ") but UE2 compile succeeded." << endl;
        summary.failPcreCompile++;
        summary.failCompileDifference++;
        return nullptr;
    }

    if (!g_quiet) {
        cout << "FAILED: id " << id << ", libpcre compile failed: " << pcreErr
             << endl;
    }

    summary.failPcreCompile++;
    return nullptr;
}

static
void drainGenerators(BoundedQueue<CorpusGenUnit> &corpq,
                     vector<unique_ptr<CorpusGenThread>> &generators,
                     TestSummary &summary) {
    // Push a sentinel per thread.
    for (size_t i = 0; i < generators.size(); i++) {
        corpq.push(nullptr);
    }

    // Wait for workers to end and retrieve their results.
    for (auto &c : generators) {
        c->join();
        summary.merge(c->getSummary());
    }
}

// Note: In multi-pattern cases, utf8 is true if any pattern to be run against
// this corpus is in UTF-8 mode.
static
unique_ptr<CorpusGenUnit> makeCorpusGenUnit(unsigned id, TestSummary &summary,
                                            GroundTruth &ground,
                                            GraphTruth &graph,
                                            UltimateTruth &ultimate,
                                            shared_ptr<DatabaseProxy> ue2,
                                            bool multi, bool utf8) {
    unique_ptr<CompiledPcre> cpcre;
    unique_ptr<CNGInfo> cngi;

    // compile PCRE bytecode
    if (use_PCRE) {
        cpcre = makePcre(id, summary, ground, ultimate, ue2);
    }
    if (use_NFA) {
        cngi = makeNGInfo(id, summary, graph, ultimate, ue2);
    }

    // if both compiles failed, skip the test
    if (!cpcre && !cngi) {
        return nullptr;
    }

    // Caller may already have set the UTF-8 property (in multi cases)
    utf8 |= cpcre ? cpcre->utf8 : cngi->utf8;

    return ue2::make_unique<CorpusGenUnit>(move(cngi), move(cpcre), ue2, id,
                                           multi, utf8);
}

static
bool hasUTF8Pattern(GroundTruth &ground, ExpressionMap::const_iterator it,
                    ExpressionMap::const_iterator end) {
    /* note: we cannot just check the flags as utf8 can be enabled in the
     * pattern itself with (*UTF) */
    debug_stage = STAGE_PCRE_COMPILE;
    for (; it != end; ++it) {
        try {
            auto cpcre = ground.compile(it->first);
            assert(cpcre); // Would have thrown PcreCompileFailure otherwise.
            if (cpcre->utf8) {
                DEBUG_PRINTF("UTF8 mode\n");
                debug_stage = STAGE_UNDEFINED;
                return true;
            }
        }
        catch (const PcreCompileFailure &) {
            continue;
        }
    }
    debug_stage = STAGE_UNDEFINED;
    return false;
}

// Fill a test queue with single-pattern tests.
static
void buildSingle(BoundedQueue<CorpusGenUnit> &corpq, TestSummary &summary,
                 GroundTruth &ground, GraphTruth &graph,
                 UltimateTruth &ultimate, const ExpressionMap &exprMap) {
    for (const auto &m : exprMap) {
        unsigned id = m.first;
        debug_expr = id;
        debug_expr_ptr = m.second.c_str();

        shared_ptr<DatabaseProxy> ue2 = constructDatabase({id}, ultimate);
        if (!ue2) {
            summary.failUe2Compile++;
            continue;
        }

        // if we're cross-compiling, then we don't bother building PCRE and
        // running scans, we're just going to output the database bytecode.
        if (!ultimate.runnable()) {
            continue;
        }

        bool multi = false;
        bool utf8 = false;
        auto u = makeCorpusGenUnit(id, summary, ground, graph, ultimate, ue2,
                                   multi, utf8);
        if (u) {
            corpq.push(move(u));
        }
    }
}

// Fill a test queue with multi-pattern tests of size N, where N is the band
// size specified on the command line.
static
void buildBanded(BoundedQueue<CorpusGenUnit> &corpq, TestSummary &summary,
                 GroundTruth &ground, GraphTruth &graph,
                 UltimateTruth &ultimate, const ExpressionMap &exprMap) {
    for (auto i = exprMap.begin(), e = exprMap.end(); i != e;) {
        debug_expr = i->first;
        debug_expr_ptr = i->second.c_str();

        // Build a set of IDs in this band from the expression map
        set<unsigned> bandIds;

        if (g_verbose) {
            cout << "Building set:";
        }

        ExpressionMap::const_iterator band_end = i;
        for (u32 j = 0; j < multicompile_bands && band_end != e;
             j++, ++band_end) {
            bandIds.insert(bandIds.end(), band_end->first);
            if (g_verbose) {
                cout << " " << band_end->first;
            }
        }

        if (g_verbose) {
            cout << endl;
        }

        // compile UE2 bytecode
        shared_ptr<DatabaseProxy> ue2 = constructDatabase(bandIds, ultimate);
        if (!ue2) {
            summary.failUe2Compile++;
            i = band_end;
            continue;
        }

        // if we're cross-compiling, then we don't bother building PCRE and
        // running scans, we're just going to output the database bytecode.
        if (!ultimate.runnable()) {
            i = band_end;
            continue;
        }

        bool utf8 = hasUTF8Pattern(ground, i, band_end);

        for (; i != band_end; ++i) {
            unsigned id = i->first;
            bool multi = true;
            auto u = makeCorpusGenUnit(id, summary, ground, graph, ultimate,
                                       ue2, multi, utf8);
            if (u) {
                corpq.push(move(u));
            }
        }
    }
}

// Fill a test queue with multi-pattern tests.
static
void buildMulti(BoundedQueue<CorpusGenUnit> &corpq, TestSummary &summary,
                GroundTruth &ground, GraphTruth &graph, UltimateTruth &ultimate,
                const ExpressionMap &exprMap) {
    // Build a set of all IDs from the expression map
    set<unsigned> idsAll;
    for (const auto &e : exprMap) {
        idsAll.insert(e.first);
    }

    // Compile in UE2
    shared_ptr<DatabaseProxy> ue2 = constructDatabase(idsAll, ultimate);
    if (!ue2) {
        summary.failUe2Compile++;
        return;
    }

    // if we're cross-compiling, then we don't bother building PCRE and
    // running scans, we're just going to output the database bytecode.
    if (!ultimate.runnable()) {
        return;
    }

    bool utf8 = hasUTF8Pattern(ground, exprMap.begin(), exprMap.end());

    for (const auto &m : exprMap) {
        unsigned id = m.first;
        debug_expr = id;
        debug_expr_ptr = m.second.c_str();
        bool multi = true;
        auto u = makeCorpusGenUnit(id, summary, ground, graph, ultimate, ue2,
                                   multi, utf8);
        if (u) {
            corpq.push(move(u));
        }
    }
}

static
void generateTests(CorporaSource &corpora_src, const ExpressionMap &exprMap,
                   TestSummary &summary, const hs_platform_info *plat,
                   const Grey &grey, BoundedQueue<TestUnit> &testq) {
    GraphTruth graph(cout, exprMap);
    GroundTruth ground(cout, exprMap, g_matchLimit, g_matchLimitRecursion);
    UltimateTruth ultimate(cout, exprMap, plat, grey, g_streamBlocks);

    // Construct corpus generator queue and threads.
    BoundedQueue<CorpusGenUnit> corpq(numGeneratorThreads,
                                      max_generator_queue_len);
    vector<unique_ptr<CorpusGenThread>> generators;
    for (size_t i = 0; i < numGeneratorThreads; i++) {
        auto c = make_unique<CorpusGenThread>(i, testq, corpq, corpora_src);
        c->start();
        generators.push_back(move(c));
    }

    if (g_ue2CompileAll && multicompile_bands) {
        printf("Running single-pattern/banded-multi-compile test for %zu "
               "expressions.\n\n", exprMap.size());
        buildBanded(corpq, summary, ground, graph, ultimate, exprMap);
    } else if (g_ue2CompileAll) {
        printf("Running single-pattern/multi-compile test for %zu "
               "expressions.\n\n", exprMap.size());
        buildMulti(corpq, summary, ground, graph, ultimate, exprMap);
    } else {
        printf("Running single-pattern/single-compile test for %zu "
               "expressions.\n\n", exprMap.size());
        buildSingle(corpq, summary, ground, graph, ultimate, exprMap);
    }

    drainGenerators(corpq, generators, summary);
}

static
void printSettingsV(const vector<string> &corporaFiles,
                    const hs_platform_info *platform) {
    cout << "hscollider: The Pattern Collider Mark II\n\n"
         << "Number of threads:  " << numThreads << " (" << numScannerThreads
         << " scanner, " << numGeneratorThreads << " generator)\n"
         << "Expression path:    " << g_exprPath << "\n"
         << "Signature files:    ";
    if (g_signatureFiles.empty()) {
        cout << "none" << endl;
    } else {
        for (unsigned i = 0; i < g_signatureFiles.size(); i++) {
            string &fname = g_signatureFiles[i];
            if (i > 0) {
                cout << string(20, ' ');
            }
            cout << fname << endl;
        }
    }
    cout << "Mode of operation:  ";

    switch (colliderMode) {
        case MODE_BLOCK:        cout << "block mode"; break;
        case MODE_STREAMING:    cout << "streaming mode"; break;
        case MODE_VECTORED:     cout << "vectored mode"; break;
        case MODE_HYBRID:       cout << "hybrid mode"; break;
    }
    cout << endl;

    if (limit_matches) {
        cout << "Terminate scanning after " << limit_matches << " matches."
             << endl;
    }

    if (platform) {
        cout << "Cross-compile for:  " << to_string(*platform) << endl;
    }

    if (loadDatabases) {
        cout << "Loading DBs from:   " << serializePath << endl;
    }
    if (saveDatabases) {
        cout << "Saving DBs to:      " << serializePath << endl;
    }
    if (colliderMode == MODE_STREAMING) {
        cout << "Stream block count: " << g_streamBlocks << endl;
    }
    if (colliderMode == MODE_VECTORED) {
        cout << "Vectored block count: " << g_streamBlocks << endl;
    }

    if (use_UE2) {
        if (max_ue2_align == min_ue2_align + 1) {
            cout << "UE2 scan alignment: " << min_ue2_align << endl;
        } else {
            cout << "UE2 scan alignment: [" << min_ue2_align << ", "
                 << max_ue2_align << ")" << endl;
        }
    }

    if (!corporaFiles.empty()) {
        for (const auto &file : corporaFiles) {
            cout << "Corpora read from file: " << file << endl;
        }
    } else {
        cout << "Corpora properties: \n"
             << "  random seed:      " << corpus_gen_prop.getSeed() << "\n"
             << "  percentages:      " << corpus_gen_prop.percentMatch()
             << "% match, "
             << corpus_gen_prop.percentUnmatch() << "% unmatch, "
             << corpus_gen_prop.percentRandom() << "% random" << endl;

        // prefix and suffix info
        const min_max &prefixSpan = corpus_gen_prop.prefixRange;
        const min_max &suffixSpan = corpus_gen_prop.suffixRange;
        if (prefixSpan.max) {
            cout << "  random prefix:    " << prefixSpan.min << " to "
                 << prefixSpan.max << endl;
        } else {
            cout << "  random prefix:    none" << endl;
        }
        if (suffixSpan.max) {
            cout << "  random suffix:    " << suffixSpan.min
                 << " to " << suffixSpan.max << endl;
        } else {
            cout << "  random suffix:    none" << endl;
        }

        // cycle info
        pair<unsigned, unsigned> cycleSpan = corpus_gen_prop.getCycleLimit();
        cout << "  follow cycles:    " << cycleSpan.first << " to "
             << cycleSpan.second << " times" << endl;
    }

    if (saveCorpora) {
        cout << "Saving corpora to:  " << saveCorporaFile << endl;
    }

    cout << endl;
}

static
void printSettingsQ(const vector<string> &corporaFiles,
                    const hs_platform_info *platform) {
    cout << "Number of threads:  " << numThreads << endl
         << "Expression path:    " << g_exprPath << endl
         << "Signature files:    ";
    if (g_signatureFiles.empty()) {
        cout << "none" << endl;
    } else {
        for (unsigned i = 0; i < g_signatureFiles.size(); i++) {
            string &fname = g_signatureFiles[i];
            if (i > 0) {
                cout << string(20, ' ');
            }
            cout << fname << endl;
        }
    }
    cout << "Mode of operation:  ";

    switch (colliderMode) {
        case MODE_BLOCK:        cout << "block mode"; break;
        case MODE_STREAMING:    cout << "streaming mode"; break;
        case MODE_VECTORED:     cout << "vectored mode"; break;
        case MODE_HYBRID:       cout << "hybrid mode"; break;
    }
    cout << endl;

    if (limit_matches) {
        cout << "Terminate scanning after " << limit_matches << " matches."
             << endl;
    }

    if (platform) {
        cout << "Cross-compile for:  " << to_string(*platform) << endl;
    }

    if (colliderMode == MODE_STREAMING) {
        cout << "Stream block count: " << g_streamBlocks << endl;
    }
    if (colliderMode == MODE_VECTORED) {
        cout << "Vectored block count: " << g_streamBlocks << endl;
    }

    if (max_ue2_align == min_ue2_align + 1) {
        cout << "UE2 scan alignment: " << min_ue2_align << endl;
    } else {
        cout << "UE2 scan alignment: [" << min_ue2_align << ", "
             << max_ue2_align << ")" << endl;
    }

    if (!g_corpora_prefix.empty()) {
        cout << "Prefix of " << g_corpora_prefix.size() << "bytes" << endl;
    }
    if (!g_corpora_suffix.empty()) {
        cout << "Suffix of " << g_corpora_suffix.size() << "bytes" << endl;
    }

    if (!corporaFiles.empty()) {
        cout << "Corpora: from file" << endl;
    } else {
        cout << "Corpora: -R " << corpus_gen_prop.getSeed() << " -p "
             << corpus_gen_prop.percentMatch() << ","
             << corpus_gen_prop.percentUnmatch() << ","
             << corpus_gen_prop.percentRandom();

        // prefix and suffix info
        const min_max &prefixSpan = corpus_gen_prop.prefixRange;
        const min_max &suffixSpan = corpus_gen_prop.suffixRange;
        if (prefixSpan.max) {
            cout << " -P " << prefixSpan.min << "," << prefixSpan.max;
        }
        if (suffixSpan.max) {
            cout << " -S " << suffixSpan.min << "," << suffixSpan.max;
        }

        // cycle info
        pair<unsigned, unsigned> cycleSpan = corpus_gen_prop.getCycleLimit();
        cout << " -C " << cycleSpan.first << "," << cycleSpan.second;
        cout << endl;
    }
}

static
void printSettings(const vector<string> &c, const hs_platform_info *plat) {
    if (g_quiet > 1) {
        printSettingsQ(c, plat);
    } else {
        printSettingsV(c, plat);
    }
}

static
unique_ptr<CorporaSource> buildCorpora(const vector<string> &corporaFiles,
                                       const ExpressionMap &exprMap) {
    if (!corporaFiles.empty()) {
        auto c = ue2::make_unique<FileCorpora>();
        for (const auto &file : corporaFiles) {
            if (!c->readFile(file)) {
                cout << "Error reading corpora from file: " << file << endl;
                exit_with_fail();
            }
        }
        return move(c); /* move allows unique_ptr<CorporaSource> conversion */
    } else {
        auto c = ue2::make_unique<NfaGeneratedCorpora>(
            exprMap, corpus_gen_prop, force_utf8, force_prefilter);
        return move(c);
    }
}

static
bool needsQuotes(const char *s) {
    size_t len = strlen(s);

    if (len == 0) {
        return true;
    }
#ifndef _WIN32
    // don't confuse the correct isblank for the one in locale
    int (*blank)(int) = &std::isblank;
    if (find_if(s, s + len, blank) != s + len) {
#else
    if (find_if(s, s + len, [](unsigned char c){ return std::isblank(c); }) != s + len) {
#endif
        return true;
    }

    return false;
}

static
void storeCmdline(int argc, char **argv) {
    for (int i = 0; i < argc; i++) {
        const char *s = argv[i];
        if (needsQuotes(s)) {
            g_cmdline += '"';
            g_cmdline += s;
            g_cmdline += '"';
        } else {
            g_cmdline += s;
        }
        if (i != argc - 1) {
            g_cmdline += " ";
        }
    }
}

static
bool runTests(CorporaSource &corpora_source, const ExpressionMap &exprMap,
              const hs_platform_info *plat, const Grey &grey) {
    TestSummary summary;
    summary.totalExpressions = exprMap.size();
    BoundedQueue<TestUnit> testq(numScannerThreads, max_scan_queue_len);

    // Start scanning threads.
    vector<unique_ptr<ScanThread>> scanners;
    for (size_t i = 0; i < numScannerThreads; i++) {
        auto s = ue2::make_unique<ScanThread>(i, testq, exprMap, plat, grey);
        s->start();
        scanners.push_back(move(s));
    }

    generateTests(corpora_source, exprMap, summary, plat, grey, testq);

    // Push a sentinel per scanning thread to ensure that everyone finishes
    // work.
    for (size_t i = 0; i < scanners.size(); i++) {
        testq.push(nullptr);
    }

    // Wait for consumers to end and retrieve their results.
    for (size_t i = 0; i < scanners.size(); i++) {
        const auto &s = scanners[i];
        s->join();

        if (g_verbose) {
            cout << "Thread " << i << " processed " << s->count << " units."
                 << endl;
        }

        summary.merge(s->getSummary());
    }

    printSummary(summary);
    return !summary.hasFailure();
}

int HS_CDECL main(int argc, char *argv[]) {
    Grey grey;
    vector<string> corporaFiles;

    for (int i = 1; i < argc - 1; i++) {
        if (!strcmp(argv[i], "-G")) {
            cout << "Override: " << argv[i + 1] << endl;
        }
    }

    setDefaults();
    storeCmdline(argc, argv);
    unique_ptr<hs_platform_info> plat;
    corpus_gen_prop.seed(randomSeed);

    processArgs(argc, argv, corpus_gen_prop, &corporaFiles, &grey, &plat);

    // If the user has asked for a random alignment, we select it here (after
    // random number seed applied).
    if (use_random_alignment) {
        min_ue2_align = corpus_gen_prop.rand(0, 15);
        max_ue2_align = min_ue2_align + 1;
    }

    // Limit memory usage, unless the user has specified zero on the command
    // line or in a config file.
    if (g_memoryLimit) {
        setMemoryLimit(g_memoryLimit * numThreads);
    }

    // Split threads available up amongst scanner and generator threads.
    numGeneratorThreads = max(1u, static_cast<unsigned int>(numThreads * 0.5));
    numScannerThreads = max(1u, numThreads - numGeneratorThreads);

    ExpressionMap exprMap;
    loadExpressions(g_exprPath, exprMap);

    if (!g_allSignatures) {
        SignatureSet signatures;
        if (!g_signatureFiles.empty()) {
            for (string &fname : g_signatureFiles) {
                loadSignatureList(fname, signatures);
            }
        } else {
            signatures.insert(signatures.end(), g_signatures.begin(),
                              g_signatures.end());
        }

        exprMap = limitToSignatures(exprMap, signatures);
    }

    printSettings(corporaFiles, plat.get());

    if (exprMap.empty()) {
        cout << "Warning: no signatures to scan. Exiting." << endl;
        exit(0);
    }

    if (!no_signal_handler) {
        installSignalHandler();
    }

    if (saveDatabases || loadDatabases) {
        struct stat st;
        if (stat(serializePath.c_str(), &st) < 0) {
            cout << "Unable to stat serialize path '" <<  serializePath
                 << "': " << strerror(errno) << endl;
            exit_with_fail();
        }
    }

    // If we're saving corpora out, truncate the output file.
    if (saveCorpora) {
        corporaOut = ue2::make_unique<CorpusWriter>(saveCorporaFile);
    }

    GroundTruth::global_prep();

    auto corpora_source = buildCorpora(corporaFiles, exprMap);

    if (!g_verbose && g_quiet < 2) {
        cout << "Only failed tests are displayed." << endl;
    }

    SimpleTimer timer;
    bool success = runTests(*corpora_source, exprMap, plat.get(), grey);
    cout << "\nTotal elapsed time: " << timer.elapsed() << " secs." << endl;
    exprMap.clear();

    if (!success) {
        exit_with_fail();
    }

    return 0;
}
