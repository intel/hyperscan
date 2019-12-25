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

#include "config.h"

#include "ng_corpus_properties.h"
#include "args.h"
#include "common.h"
#include "cross_compile.h"
#include "util/expression_path.h"
#include "util/string_util.h"

#include "grey.h"
#include "ue2common.h"
#include "hs_compile.h" // for HS_MODE_*

#include <algorithm>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>
#ifndef _WIN32
#include <getopt.h>
#else
#include "win_getopt.h"
#endif

#define xstr(s) str(s)
#define str(s) #s

using namespace ue2;
using namespace std;

// display usage information, with an optional error
static
void usage(const char *name, const char *error) {
    printf("Usage: %s [OPTIONS...]\n\n", name);
    printf("General Options:\n\n");
    printf("  -h              Display help and exit.\n");
    printf("  -G OVERRIDES    Overrides for the grey box.\n");
    printf("  -e PATH         Path to expression directory or file.\n");
    printf("  -s FILE         Signature file to use.\n");
    printf("  -z NUM          Signature ID to use.\n");
    printf("  -c FILE         Load corpora from FILE rather than using "
           "generator.\n");
    printf("  -w FILE         After running, save corpora (with matches) to "
           "FILE.\n");
    printf("  -a [BAND]       Compile all expressions in UE2 (but still match "
           "singly).\n");
    printf("                  If BAND, compile patterns in groups of size "
           "BAND.\n");
    printf("  -t NUM          Use streaming mode, split data into ~NUM "
           "blocks.\n");
    printf("  -V NUM          Use vectored mode, split data into ~NUM "
           "blocks.\n");
    printf("  -H              Use hybrid mode.\n");
    printf("  -Z {R or 0-%d}  Only test one alignment, either as given or "
           "'R' for random.\n", MAX_MAX_UE2_ALIGN - 1);
    printf("  -q              Quiet; display only match differences, no other "
           "failures.\n");
    printf("  -v              Verbose; display successes as well as "
           "failures.\n");
    printf("\n");
    printf("Pattern flags:\n");
    printf("\n");
    printf("  -8              Force UTF8 mode on all patterns.\n");
    printf("  -L              Apply HS_FLAG_SOM_LEFTMOST to all patterns.\n");
    printf("  -E DISTANCE     Match all patterns within edit distance"
           " DISTANCE.\n");
    printf("  --prefilter     Apply HS_FLAG_PREFILTER to all patterns.\n");
    printf("  --no-groups     Disable capturing in Hybrid mode.\n");
    printf("\n");
    printf("Testing mode options:\n");
    printf("\n");
    printf("  -d NUM          Set SOM precision mode (default: 8 (large)).\n");
    printf("  -O NUM          In streaming mode, set initial offset to NUM.\n");
    printf("  -k NUM          Terminate callback after NUM matches per "
           "pattern.\n");
    printf("  --copy-scratch  Copy scratch after each scan call.\n");
    printf("  --copy-stream   Copy stream state after each scan call.\n");
    printf("  --compress-expand Compress and expand stream state after each "
           "scan call.\n");
    printf("  --compress-reset-expand Compress, reset and expand stream state "
           "after each scan call.\n");
    printf("  --mangle-scratch Mangle scratch space after each scan call.\n");
    printf("  --no-nfa        Disable NFA graph execution engine.\n");
    printf("  --no-pcre       Disable PCRE engine.\n");
    printf("  --test-nfa      Disable UE2 engine (test NFA against PCRE).\n");
    printf("  --abort-on-fail Abort, rather than exit, on failure.\n");
    printf("  --no-signal-handler Do not handle handle signals (to generate "
           "backtraces).\n");
    printf("  --literal-on    Use Hyperscan pure literal matching.\n");
    printf("\n");
    printf("Memory and resource control options:\n");
    printf("\n");
    printf("  -T NUM          Run with NUM threads.\n");
    printf("  -M NUM          Set maximum memory allocated to NUM megabytes per"
           " thread.\n");
    printf("                  (0 means no limit, default is 1000 MB).\n");
    printf("  -m NUM          Set PCRE_MATCH_LIMIT (default: %lu).\n",
           DEFAULT_PCRE_MATCH_LIMIT);
    printf("  -r NUM          Set PCRE_MATCH_LIMIT_RECURSION (default: %lu).\n",
           DEFAULT_PCRE_MATCH_RECURSION_LIMIT);
    printf("\n");
    printf("Cross-compiling:\n");
    printf("\n");
    printf("  -x NAME         Cross-compile for arch NAME.\n");
    printf("  -i DIR          Don't compile, load from files in DIR "
           "instead.\n");
    printf("  -o DIR          After compiling, save to files in DIR.\n");
    printf("\n");
    printf("Corpus generation options:\n");
    printf("\n");
    printf("  -n NUM          Max corpora to generate for a given signature "
           "(default: %u).\n", DEFAULT_CORPUS_GENERATOR_LIMIT);
    printf("  -R NUM          Random seed to use (default: seeded from "
           "time()).\n");
    printf("  -p NUM,NUM,NUM  Percentage probabilities of "
           "(match,unmatch,random) char.\n");
    printf("  -C NUM,NUM      Follow cycles (min,max) times.\n");
    printf("  -P NUM,NUM      Add a random prefix of length between "
           "(min,max).\n");
    printf("  -S NUM,NUM      Add a random suffix of length between "
           "(min,max).\n");
    printf("  -D NUM          Apply an edit distance (default: 0) to each "
           "corpus.\n");
    printf("  -b NUM          Limit alphabet to NUM characters, starting at "
           "lower-case 'a'.\n");
    printf("\n");

    if (error) {
        printf("Error: %s\n", error);
    }
}

void processArgs(int argc, char *argv[], CorpusProperties &corpus_gen_prop,
                 vector<string> *corpora, UNUSED Grey *grey,
                 unique_ptr<hs_platform_info> *plat_out) {
    static const char options[]
        = "-ab:cC:d:D:e:E:G:hHi:k:Lm:M:n:o:O:p:P:qr:R:S:s:t:T:vV:w:x:X:Y:z:Z:8";
    s32 in_multi = 0;
    s32 in_corpora = 0;
    int pcreFlag = 1;
    int nfaFlag = 1;
    int ue2Flag = 1;
    int copyScratch = 0;
    int copyStream = 0;
    int mangleScratch = 0;
    int compressFlag = 0;
    int compressResetFlag = 0;
    int literalFlag = 0;
    static const struct option longopts[] = {
        {"copy-scratch", 0, &copyScratch, 1},
        {"copy-stream", 0, &copyStream, 1},
        {"mangle-scratch", 0, &mangleScratch, 1},
        {"prefilter", 0, &force_prefilter, 1},
        {"no-pcre", 0, &pcreFlag, 0},
        {"no-nfa", 0, &nfaFlag, 0},
        {"test-nfa", 0, &ue2Flag, 0},
        {"abort-on-fail", 0, &abort_on_failure, 1},
        {"no-signal-handler", 0, &no_signal_handler, 1},
        {"compress-expand", 0, &compressFlag, 1},
        {"compress-reset-expand", 0, &compressResetFlag, 1},
        {"no-groups", 0, &no_groups, 1},
        {"literal-on", 0, &literalFlag, 1},
        {nullptr, 0, nullptr, 0}};

    for (;;) {
        int c = getopt_long(argc, argv, options, longopts, nullptr);
        if (c < 0) {
            break;
        }

        switch (c) {
            case 'a':
                g_ue2CompileAll = true;
                in_multi = 2;
                break;
            case 'b': {
                unsigned sz;
                if (!fromString(optarg, sz) || sz > 256) {
                    usage(argv[0], "Must provide an integer argument <= 256"
                          "to '-b' flag");
                    exit(1);
                }
                corpus_gen_prop.alphabetSize = sz;
                break;
            }
            case 'c':
                in_corpora = 2;
                break;
            case 'C': {
                vector<unsigned> nums;
                if (!strToList(optarg, nums) || nums.size() != 2
                    || nums[0] > nums[1]) {
                    usage(argv[0], "Cycle limit '-C' argument takes a list of "
                          " integers: MIN,MAX");
                    exit(1);
                }
                corpus_gen_prop.setCycleLimit(nums[0], nums[1]);
                break;
            }
            case 'd': {
                unsigned dist;
                if (!fromString(optarg, dist)) {
                    usage(argv[0],
                          "Must provide an integer argument to '-d' flag");
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
            case 'D': {
                unsigned dist;
                if (!fromString(optarg, dist)) {
                    usage(argv[0],
                          "Must provide an integer argument to '-D' flag");
                    exit(1);
                }
                corpus_gen_prop.editDistance = dist;
                break;
            }
            case 'e':
                g_exprPath.assign(optarg);
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
#ifndef RELEASE_BUILD
            case 'G':
                applyGreyOverrides(grey, string(optarg));
                break;
#endif
            case 'h':
                usage(argv[0], nullptr);
                exit(0);
            case 'H':
                if (colliderMode != MODE_BLOCK) {
                    usage(argv[0], "You can only use one mode at a time!");
                    exit(1);
                }
                colliderMode = MODE_HYBRID;
                // Disable graph truth in hybrid mode
                nfaFlag = 0;
                break;
            case 'i':
                loadDatabases = true;
                serializePath = optarg;
                break;
            case 'k':
                if (!fromString(optarg, limit_matches) || limit_matches < 1) {
                    usage(argv[0],
                          "Must provide a positive integer argument to '-k' "
                          "flag");
                    exit(1);
                }
                break;
            case 'L':
                somFlags = HS_FLAG_SOM_LEFTMOST;
                break;
            case 'm':
                if (!fromString(optarg, g_matchLimit) || g_matchLimit < 1) {
                    usage(argv[0],
                          "Must provide a positive integer argument to '-m' "
                          "flag");
                    exit(1);
                }
                break;
            case 'M':
                if (!fromString(optarg, g_memoryLimit)) {
                    usage(argv[0],
                          "Must provide a positive (or zero) integer argument "
                          "to '-M' flag");
                    exit(1);
                }
                break;
            case 'n': {
                unsigned int count;
                if (!fromString(optarg, count)) {
                    usage(argv[0], "Argument to '-n' flag must be an integer");
                    exit(1);
                }
                corpus_gen_prop.corpusLimit = count;
                break;
            }
            case 'o':
                saveDatabases = true;
                serializePath = optarg;
                break;
            case 'O':
                if (!fromString(optarg, g_streamOffset)) {
                    usage(argv[0],
                          "Argument '-O' flag must be a positive integer");
                    exit(1);
                }
                break;
            case 'p': {
                vector<unsigned> prob;
                if (!strToList(optarg, prob) || prob.size() != 3) {
                    usage(argv[0], "Probabilities '-p' argument takes a list "
                          "of three integers: MATCH,UNMATCH,RANDOM");
                    exit(1);
                }
                if (!corpus_gen_prop.setPercentages(prob[0], prob[1],
                                                    prob[2])) {
                    usage(argv[0],
                          "Unable to set corpus generator probabilities.");
                    exit(1);
                }
                break;
            }
            case 'P': {
                vector<unsigned> nums;
                if (!strToList(optarg, nums) || nums.size() != 2
                    || nums[0] > nums[1]) {
                    usage(argv[0], "Prefix '-P' argument takes a list of two"
                            " integers: MIN,MAX");
                    exit(1);
                }
                corpus_gen_prop.prefixRange = min_max(nums[0], nums[1]);
                break;
            }
            case 'q':
                g_quiet++;
                break;
            case 'r':
                if (!fromString(optarg, g_matchLimitRecursion)
                    || g_matchLimitRecursion < 1) {
                    usage(argv[0], "Must provide a positive integer argument "
                          "to '-r' flag");
                    exit(1);
                }
                break;
            case 'R': {
                if (!fromString(optarg, randomSeed)) {
                    usage(argv[0], "Argument to '-R' flag must be an integer");
                    exit(1);
                }
                corpus_gen_prop.seed(randomSeed);
                break;
            }
            case 's':
                g_signatureFiles.push_back(optarg);
                break;
            case 'S': {
                vector<unsigned> nums;
                if (!strToList(optarg, nums) || nums.size() != 2 ||
                        nums[0] > nums[1]) {
                    usage(argv[0], "Suffix '-S' argument takes a list of two"
                          " integers: MIN,MAX");
                    exit(1);
                }
                corpus_gen_prop.suffixRange = min_max(nums[0], nums[1]);
                break;
            }
            case 't':
                if (colliderMode != MODE_BLOCK) {
                    usage(argv[0], "You can only use one mode at a time!");
                    exit(1);
                }
                colliderMode = MODE_STREAMING;
                if (!fromString(optarg, g_streamBlocks) || g_streamBlocks < 1) {
                    usage(argv[0], "Must provide a positive integer argument "
                          "to '-t' flag");
                    exit(1);
                }
                break;
            case 'T':
                if (!fromString(optarg, numThreads) || numThreads < 1) {
                    usage(argv[0], "Must provide a positive integer argument "
                          "to '-T' flag");
                    exit(1);
                }
                break;
            case 'v':
                if (g_verbose) {
                    echo_matches = true;
                }
                g_verbose = true;
                break;
            case 'V':
                if (colliderMode != MODE_BLOCK) {
                    usage(argv[0], "You can only use one mode at a time!");
                    exit(1);
                }
                colliderMode = MODE_VECTORED;
                if (!fromString(optarg, g_streamBlocks) || g_streamBlocks < 1) {
                    usage(argv[0], "Must provide a positive integer argument "
                          "to '-t' flag");
                    exit(1);
                }
                break;
            case 'w':
                saveCorpora = true;
                saveCorporaFile = optarg;
                break;
            case 'x':
                *plat_out = xcompileReadMode(optarg);
                if (!*plat_out) {
                    usage(argv[0], xcompileUsage().c_str());
                    exit(1);
                }
                break;
            case 'X': {
                u32 count;
                if (!fromString(optarg, count)) {
                    usage(argv[0], "Argument to '-X' flag must be an integer");
                    exit(1);
                }
                g_corpora_prefix.insert(g_corpora_prefix.end(), count, '~');
                break;
            }
            case 'Y':
            {
                u32 count;
                if (!fromString(optarg, count)) {
                    usage(argv[0], "Argument to '-Y' flag must be an integer");
                    exit(1);
                }
                g_corpora_suffix.insert(g_corpora_suffix.end(), count, '~');
                break;
            }
            case 'z':
                if (!strToList(optarg, g_signatures)) {
                    usage(argv[0],
                          "Argument to '-z' flag must be a list of integers");
                    exit(1);
                }
                break;
            case 'Z': {     // Parentheses save VS C2360
                static constexpr unsigned ALIGN_LIMIT = MAX_MAX_UE2_ALIGN - 1;
                if (optarg == string("R")) {
                    // Random min alignment selected.
                    use_random_alignment = true;
                    break;
                } else if (!fromString(optarg, min_ue2_align)
                           || min_ue2_align > ALIGN_LIMIT) {
                    usage(argv[0], "Argument must be 'R' or numeric < "
                          xstr(MAX_MAX_UE2_ALIGN) " to '-Z'");
                    exit(1);
                }
                max_ue2_align = min_ue2_align + 1;
                break;
            }
            case '8':
                force_utf8 = true;
                break;
            case 1:
                if (in_multi) {
                    if (!fromString(optarg, multicompile_bands)) {
                        usage(argv[0],
                              "Argument to '-a' flag must be an integer");
                        exit(1);
                    }
                    break;
                } else if (in_corpora) {
                    corpora->push_back(optarg);
                    in_corpora = 2;
                    break;
                }
            case 0:
                break;
            default:
                usage(argv[0], "Unrecognised command line argument.");
                exit(1);
        }

        in_multi = MAX(0, in_multi - 1);
        in_corpora = MAX(0, in_corpora - 1);
    }

    if (g_streamOffset && !g_streamBlocks) {
        usage(argv[0], "stream offset requires streams");
        exit(1);
    }

    if (g_exprPath.empty() && !g_signatureFiles.empty()) {
        /* attempt to infer an expression directory */
        for (const auto &fname : g_signatureFiles) {
            string exprPath = inferExpressionPath(fname);
            if (!g_exprPath.empty() && exprPath != g_exprPath) {
                usage(argv[0], "Only one expression path is allowed.");
            }
            g_exprPath.assign(exprPath);
        }
    }

    // Must have a valid expression path
    if (g_exprPath.empty()) {
        usage(argv[0], "Must specify an expression path with the -e option.");
        exit(1);
    }

    // If we've been handed an expr file and no restrictions, use 'em all!
    if (!isDir(g_exprPath) && isFile(g_exprPath) && g_signatureFiles.empty()
        && g_signatures.empty()) {
        g_allSignatures = true;
    }

    // Must have a valid signature file
    if (g_signatureFiles.empty() && g_signatures.empty() && !g_allSignatures) {
        usage(argv[0], "Must specify a signature file with the -s option.");
        exit(1);
    }

    // Cannot ask for both loading and saving
    if (loadDatabases && saveDatabases) {
        usage(argv[0], "You cannot both load and save databases.");
        exit(1);
    }

    // Cannot ask for cross-compile and loading
    if (loadDatabases && *plat_out) {
        usage(argv[0], "You cannot both load and xcompile of databases.");
        exit(1);
    }

    if (colliderMode == MODE_HYBRID && !ue2Flag) {
        usage(argv[0], "You cannot disable UE2 engine in Hybrid mode.");
        exit(1);
    }

    // need at least two pattern engines active
    if (nfaFlag + pcreFlag + ue2Flag < 2) {
        usage(argv[0], "At least two pattern engines should be active.");
        exit(1);
    }

    if (copyStream && !g_streamBlocks) {
        usage(argv[0], "Copying streams only makes sense in streaming mode.");
        exit(1);
    }
    if (compressFlag && compressResetFlag) {
        usage(argv[0],
              "Only use one of --compress-expand and --compress-reset-expand.");
        exit(1);
    }

    // set booleans appropriately
    use_NFA = (bool) nfaFlag;
    use_PCRE = (bool) pcreFlag;
    use_UE2 = (bool) ue2Flag;
    use_copy_scratch = (bool) copyScratch;
    use_copy_stream = (bool) copyStream;
    use_mangle_scratch = (bool) mangleScratch;
    use_compress_expand = (bool)compressFlag;
    use_compress_reset_expand = (bool)compressResetFlag;
    use_literal_api = (bool)literalFlag;
}
