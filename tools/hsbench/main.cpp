/*
 * Copyright (c) 2016-2018, Intel Corporation
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

#include "common.h"
#include "data_corpus.h"
#include "engine_hyperscan.h"
#if defined(HS_HYBRID)
#include "engine_chimera.h"
#include "engine_pcre.h"
#endif
#include "expressions.h"
#include "sqldb.h"
#include "thread_barrier.h"
#include "timer.h"
#include "util/expression_path.h"
#include "util/string_util.h"

#include "grey.h"
#include "hs.h"
#include "ue2common.h"
#include "util/make_unique.h"

#include <algorithm>
#include <clocale>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <map>
#include <numeric>
#include <sstream>
#include <set>
#include <thread>

#ifndef _WIN32
#include <getopt.h>
#else
#include "win_getopt.h"
#endif
#ifndef _WIN32
#include <pthread.h>
#if defined(HAVE_PTHREAD_NP_H)
#include <pthread_np.h>
#endif
#include <unistd.h>
#endif

#include <boost/core/noncopyable.hpp>
#include <boost/range/adaptor/map.hpp>

using namespace std;
using namespace ue2;
using boost::adaptors::map_keys;

// Globals common to all files.
bool echo_matches = false;
bool saveDatabases = false;
bool loadDatabases = false;
string serializePath("");
unsigned int somPrecisionMode = HS_MODE_SOM_HORIZON_LARGE;
bool forceEditDistance = false;
unsigned editDistance = 0;
bool printCompressSize = false;

// Globals local to this file.
static bool compressStream = false;

namespace /* anonymous */ {

bool display_per_scan = false;
ScanMode scan_mode = ScanMode::STREAMING;
bool useHybrid = false;
bool usePcre = false;
unsigned repeats = 20;
string exprPath("");
string corpusFile("");
string sqloutFile("");
string sigName(""); // info only
vector<unsigned int> threadCores;
Timer totalTimer;
double totalSecs = 0;

SqlDB out_db;

typedef void (*thread_func_t)(void *context);

class ThreadContext : boost::noncopyable {
public:
    ThreadContext(unsigned num_in, const Engine &db_in,
                  thread_barrier &tb_in, thread_func_t function_in,
                  vector<DataBlock> corpus_data_in)
        : num(num_in), results(repeats), engine(db_in),
          enginectx(db_in.makeContext()), corpus_data(move(corpus_data_in)),
          tb(tb_in), function(function_in) {}

    // Start the thread.
    bool start(int cpu) {
        thr = thread(function, this);

        // affine if it's asked for
        if (cpu >= 0) {
            return affine(cpu);
        }
        return true;
    }

    // Wait for the thread to exit.
    void join() {
        thr.join();
    }

    // Serialise all threads on a global barrier.
    void barrier() {
        tb.wait();
    }

    // Apply processor affinity (if available) to this thread.
    bool affine(UNUSED int cpu) {

#if defined(_WIN32)
        SYSTEM_INFO system_info;
        GetSystemInfo(&system_info);
        assert(cpu >= 0 && (DWORD)cpu < system_info.dwNumberOfProcessors);
        DWORD_PTR mask = 1 << cpu;
        DWORD_PTR rv = SetThreadAffinityMask(thr.native_handle(), mask);
        return rv != 0;
#endif

#ifdef HAVE_DECL_PTHREAD_SETAFFINITY_NP
#if defined(__FreeBSD__)
        cpuset_t cpuset;
#else
        cpu_set_t cpuset;
#endif
        CPU_ZERO(&cpuset);
        assert(cpu >= 0 && cpu < CPU_SETSIZE);

        // The 'clang' compiler complains about an unused result here, so we
        // silence it.
        (void)CPU_SET(cpu, &cpuset);

        int rv = pthread_setaffinity_np(thr.native_handle(), sizeof(cpuset),
                                        &cpuset);
        return (rv == 0);
#endif
        return false;  // not available
    }

    unsigned num;
    Timer timer;
    vector<ResultEntry> results;
    const Engine &engine;
    unique_ptr<EngineContext> enginectx;
    vector<DataBlock> corpus_data;

protected:
    thread_barrier &tb; // shared barrier for time sync
    thread_func_t function;
    thread thr;
};

/** Display usage information, with an optional error. */
static
void usage(const char *error) {
    printf("Usage: hsbench [OPTIONS...]\n\n");
    printf("Options:\n\n");
    printf("  -h              Display help and exit.\n");
    printf("  -G OVERRIDES    Overrides for the grey box.\n");
    printf("  -e PATH         Path to expression directory.\n");
    printf("  -s FILE         Signature file to use.\n");
    printf("  -z NUM          Signature ID to use.\n");
    printf("  -c FILE         File to use as corpus.\n");
    printf("  -n NUMBER       Repeat scan NUMBER times (default 20).\n");
    printf("  -N              Benchmark in block mode"
           " (default: streaming).\n");
    printf("  -V              Benchmark in vectored mode"
           " (default: streaming).\n");
#if defined(HS_HYBRID)
    printf("  -H              Benchmark using Chimera (if supported).\n");
    printf("  -P              Benchmark using PCRE (if supported).\n");
#endif
#if defined(HAVE_DECL_PTHREAD_SETAFFINITY_NP) || defined(_WIN32)
    printf("  -T CPU,CPU,...  Benchmark with threads on these CPUs.\n");
#endif
    printf("  -i DIR          Don't compile, load from files in DIR"
           " instead.\n");
    printf("  -w DIR          After compiling, save to files in DIR.\n");
    printf("  -d NUMBER       Set SOM precision mode (default: 8 (large)).\n");
    printf("  -E DISTANCE     Match all patterns within edit distance"
           " DISTANCE.\n");
    printf("\n");
    printf("  --per-scan      Display per-scan Mbit/sec results.\n");
    printf("  --echo-matches  Display all matches that occur during scan.\n");
    printf("  --sql-out FILE  Output sqlite db.\n");
    printf("  -S NAME         Signature set name (for sqlite db).\n");
    printf("\n\n");

    if (error) {
        printf("Error: %s\n", error);
    }
}

/** Wraps up a name and the set of signature IDs it refers to. */
struct BenchmarkSigs {
    BenchmarkSigs(string name_in, SignatureSet sigs_in)
        : name(move(name_in)), sigs(move(sigs_in)) {}
    string name;
    SignatureSet sigs;
};

/** Process command-line arguments. Prints usage and exits on error. */
static
void processArgs(int argc, char *argv[], vector<BenchmarkSigs> &sigSets,
                 UNUSED unique_ptr<Grey> &grey) {
    const char options[] = "-b:c:Cd:e:E:G:hHi:n:No:p:PsS:Vw:z:"
#if defined(HAVE_DECL_PTHREAD_SETAFFINITY_NP) || defined(_WIN32)
        "T:" // add the thread flag
#endif
        ;
    int in_sigfile = 0;
    int do_per_scan = 0;
    int do_compress = 0;
    int do_compress_size = 0;
    int do_echo_matches = 0;
    int do_sql_output = 0;
    int option_index = 0;
    vector<string> sigFiles;

    static struct option longopts[] = {
        {"per-scan", no_argument, &do_per_scan, 1},
        {"echo-matches", no_argument, &do_echo_matches, 1},
        {"compress-stream", no_argument, &do_compress, 1},
        {"sql-out", required_argument, &do_sql_output, 1},
        {nullptr, 0, nullptr, 0}
    };

    for (;;) {
        int c = getopt_long(argc, argv, options, longopts, &option_index);
        if (c < 0) {
            break;
        }
        switch (c) {
        case 'c':
            corpusFile.assign(optarg);
            break;
        case 'd': {
            unsigned dist;
            if (!fromString(optarg, dist)) {
                usage("Must provide an integer argument to '-d' flag");
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
                usage("SOM precision must be 2, 4 or 8");
                exit(1);
            }
            break;
        }
        case 'e':
            exprPath.assign(optarg);
            break;
        case 'E':
            if (!fromString(optarg, editDistance)) {
                usage("Couldn't parse argument to -E flag, should be"
                      " a non-negative integer.");
                exit(1);
            }
            forceEditDistance = true;
            break;
#ifndef RELEASE_BUILD
        case 'G':
            applyGreyOverrides(grey.get(), string(optarg));
            break;
#endif
        case 'h':
            usage(nullptr);
            exit(0);
            break;
        case 'H':
#if defined(HS_HYBRID)
            useHybrid = true;
#else
            usage("Hybrid matcher not enabled in this build");
            exit(1);
#endif
            break;
        case 'n':
            if (!fromString(optarg, repeats) || repeats == 0) {
                usage("Couldn't parse argument to -n flag, should be"
                      " a positive integer.");
                exit(1);
            }
            break;
        case 'P':
#if defined(HS_HYBRID)
            usePcre = true;
#else
            usage("PCRE matcher not enabled in this build");
            exit(1);
#endif
            break;
        case 's':
            in_sigfile = 2;
            break;
        case 'N':
            scan_mode = ScanMode::BLOCK;
            break;
        case 'V':
            scan_mode = ScanMode::VECTORED;
            break;
        case 'S':
            sigName.assign(optarg);
            break;
#if defined(HAVE_DECL_PTHREAD_SETAFFINITY_NP) || defined(_WIN32)
        case 'T':
            if (!strToList(optarg, threadCores)) {
                usage("Couldn't parse argument to -T flag, should be"
                      " a list of positive integers.");
                exit(1);
            }
            break;
#endif
        case 'z': {
            unsigned int sinumber;
            if (!fromString(optarg, sinumber)) {
                usage("Argument to '-z' flag must be an integer");
                exit(1);
            }
            SignatureSet sigs = {sinumber};
            sigSets.emplace_back(string("-z ") + optarg, sigs);
            break;
        }
        case 'i':
            loadDatabases = true;
            serializePath = optarg;
            break;
        case 'w':
            saveDatabases = true;
            serializePath = optarg;
            break;
        case 0:
            if (do_sql_output) {
                sqloutFile.assign(optarg);
                do_sql_output = 0;
            }
            break;
        case 1:
            if (in_sigfile) {
                sigFiles.push_back(optarg);
                in_sigfile = 2;
                break;
            }
            /* fallthrough */
        default:
            usage("Unrecognised command line argument.");
            exit(1);
        }

        if (in_sigfile) {
            in_sigfile--;
        }
    }

    if (do_echo_matches) {
        echo_matches = true;
    }
    if (do_per_scan) {
        display_per_scan = true;
    }
    if (do_compress) {
        compressStream = true;
    }
    if (do_compress_size) {
        printCompressSize = true;
    }

    if (exprPath.empty() && !sigFiles.empty()) {
        /* attempt to infer an expression directory */
        auto si = sigFiles.begin();
        exprPath = inferExpressionPath(*si);
        for (++si; si != sigFiles.end(); ++si) {
            if (exprPath != inferExpressionPath(*si)) {
                usage("Unable to infer consistent expression directory");
                exit(1);
            }
        }
    }

    // Must have a valid expression path
    if (exprPath.empty()) {
        usage("Must specify an expression path with the -e option.");
        exit(1);
    }

    // Must have valid database to scan
    if (corpusFile.empty()) {
        usage("Must specify a corpus file with the -c option.");
        exit(1);
    }

    // Cannot ask for both loading and saving
    if (loadDatabases && saveDatabases) {
        usage("You cannot both load and save databases.");
        exit(1);
    }

    // Constraints on Chimera and PCRE engines
    if (useHybrid || usePcre) {
        if (useHybrid && usePcre) {
            usage("Can't run both Chimera and PCRE.");
            exit(1);
        }
        if (scan_mode != ScanMode::BLOCK) {
            usage("Must specify block mode in Chimera or PCRE with "
                  "the -N option.");
            exit(1);
        }

        if (forceEditDistance || loadDatabases || saveDatabases) {
            usage("No extended options are supported in Chimera or PCRE.");
            exit(1);
        }
    }

    // Read in any -s signature sets.
    for (const auto &file : sigFiles) {
        SignatureSet sigs;
        loadSignatureList(file, sigs);
        sigSets.emplace_back(file, move(sigs));
    }
}

/** Start the global timer. */
static
void startTotalTimer(ThreadContext *ctx) {
    if (ctx->num != 0) {
        return; // only runs in the first thread
    }
    totalTimer.start();
}

/** Stop the global timer and calculate totals. */
static
void stopTotalTimer(ThreadContext *ctx) {
    if (ctx->num != 0) {
        return; // only runs in the first thread
    }
    totalTimer.complete();
    totalSecs = totalTimer.seconds();
}

/** Run a benchmark over a given engine and corpus in block mode. */
static
void benchBlock(void *context) {
    ThreadContext *ctx = (ThreadContext *)context;

    // Synchronization point
    ctx->barrier();

    startTotalTimer(ctx);

    for (ResultEntry &r : ctx->results) {
        ctx->timer.start();

        for (const DataBlock &block : ctx->corpus_data) {
            ctx->engine.scan(block.payload.c_str(), block.payload.size(),
                             block.id, r, *ctx->enginectx);
        }

        ctx->timer.complete();
        r.seconds = ctx->timer.seconds();
    }

    // Synchronization point
    ctx->barrier();

    // Now that all threads are finished, we can stop the clock.
    stopTotalTimer(ctx);
}

/** Structure used to represent a stream. */
struct StreamInfo {
    unsigned int stream_id = ~0U;
    unsigned int first_block_id = ~0U;
    unsigned int last_block_id = 0;
    unique_ptr<EngineStream> eng_handle;
};

static
u64a count_streams(const vector<DataBlock> &corpus_blocks) {
    set<unsigned int> streams;
    for (const DataBlock &block : corpus_blocks) {
        streams.insert(block.stream_id);
    }

    return (u64a)streams.size();
}

/**
 * Take a ThreadContext and prepare a vector<StreamDataBlock> for streaming mode
 * scanning from it.
 */
static
vector<StreamInfo> prepStreamingData(const ThreadContext *ctx) {
    vector<StreamInfo> info(count_streams(ctx->corpus_data));
    for (const DataBlock &block : ctx->corpus_data) {
        assert(block.internal_stream_index < info.size());
        StreamInfo &si = info[block.internal_stream_index];

        /* check if this is the first time we have encountered this stream */
        if (si.first_block_id > si.last_block_id) {
            si.stream_id = block.stream_id;
            si.first_block_id = block.id;
            si.last_block_id = block.id;
        } else {
            assert(block.stream_id == si.stream_id);
            assert(block.id > si.last_block_id);
            assert(block.id > si.first_block_id);
            si.last_block_id = block.id;
        }
    }
    return info;
}

static
void benchStreamingInternal(ThreadContext *ctx, vector<StreamInfo> &streams,
                            bool do_compress) {
    assert(ctx);
    const Engine &e = ctx->engine;
    const vector<DataBlock> &blocks = ctx->corpus_data;
    vector<char> compress_buf(do_compress ? 1000 : 0);

    for (ResultEntry &r : ctx->results) {
        ctx->timer.start();

        for (const auto &b : blocks) {
            StreamInfo &stream = streams[b.internal_stream_index];
            assert(stream.stream_id == b.stream_id);

            // If this is the first block in the stream, open the stream
            // handle.
            if (b.id == stream.first_block_id) {
                assert(!stream.eng_handle);
                stream.eng_handle = e.streamOpen(*ctx->enginectx, b.stream_id);
                if (!stream.eng_handle) {
                    printf("Fatal error: stream open failed!\n");
                    exit(1);
                }
            } else if (do_compress) {
                e.streamCompressExpand(*stream.eng_handle, compress_buf);
            }

            assert(stream.eng_handle);

            e.streamScan(*stream.eng_handle, b.payload.c_str(),
                         b.payload.size(), b.id, r);

            // if this was the last block in the stream, close the stream handle
            if (b.id == stream.last_block_id) {
                e.streamClose(move(stream.eng_handle), r);
                stream.eng_handle = nullptr;
            }
        }

        ctx->timer.complete();
        r.seconds = ctx->timer.seconds();
    }
}

/** Run a benchmark over a given engine and corpus in streaming mode. */
static
void benchStreaming(void *context) {
    ThreadContext *ctx = (ThreadContext *)context;
    vector<StreamInfo> streams = prepStreamingData(ctx);

    // Synchronization point
    ctx->barrier();

    startTotalTimer(ctx);

    benchStreamingInternal(ctx, streams, false);

    // Synchronization point
    ctx->barrier();

    // Now that all threads are finished, we can stop the clock.
    stopTotalTimer(ctx);
}

static
void benchStreamingCompress(void *context) {
    ThreadContext *ctx = (ThreadContext *)context;
    vector<StreamInfo> streams = prepStreamingData(ctx);

    // Synchronization point
    ctx->barrier();

    startTotalTimer(ctx);

    benchStreamingInternal(ctx, streams, true);

    // Synchronization point
    ctx->barrier();

    // Now that all threads are finished, we can stop the clock.
    stopTotalTimer(ctx);
}


/** In-memory structure for a data block to be scanned in vectored mode. */
struct VectoredInfo {
    vector<const char *> data;
    vector<unsigned int> len;
    unsigned int stream_id;
};

/**
 * Take a ThreadContext and prepare a vector<VectoredInfo> for vectored mode
 * scanning from it.
 */
static
vector<VectoredInfo> prepVectorData(const ThreadContext *ctx) {
    vector<VectoredInfo> out(count_streams(ctx->corpus_data));
    for (const DataBlock &block : ctx->corpus_data) {
        VectoredInfo &vi = out[block.internal_stream_index];
        if (vi.data.empty()) {
            vi.stream_id = block.stream_id;
        } else {
            assert(vi.stream_id == block.stream_id);
        }
        vi.data.push_back(block.payload.c_str());
        vi.len.push_back(block.payload.size());
    }

    return out;
}

/** Run a benchmark over a given engine and corpus in vectored mode. */
static
void benchVectored(void *context) {
    ThreadContext *ctx = (ThreadContext *)context;

    vector<VectoredInfo> v_plans = prepVectorData(ctx);

    // Synchronization point
    ctx->barrier();

    startTotalTimer(ctx);

    for (ResultEntry &r : ctx->results) {
        ctx->timer.start();

        for (const VectoredInfo &v_plan : v_plans) {
            ctx->engine.scan_vectored(&v_plan.data[0], &v_plan.len[0],
                                      v_plan.data.size(), v_plan.stream_id, r,
                                      *ctx->enginectx);
        }

        ctx->timer.complete();
        r.seconds = ctx->timer.seconds();
    }

    // Synchronization point
    ctx->barrier();

    // Now that all threads are finished, we can stop the clock.
    stopTotalTimer(ctx);
}

/** Given a time and a size, compute the throughput in megabits/sec. */
static
long double calc_mbps(double seconds, u64a bytes) {
    assert(seconds > 0);
    return (long double)bytes / ((long double)seconds * 125000);
}

/** Dump per-scan throughput data to screen. */
static
void displayPerScanResults(const vector<unique_ptr<ThreadContext>> &threads,
                           u64a bytesPerRun) {
    for (const auto &t : threads) {
        const auto &results = t->results;
        for (size_t j = 0; j != results.size(); j++) {
            const auto &r = results[j];
            double mbps = calc_mbps(r.seconds, bytesPerRun);
#ifndef _WIN32
            printf("T %2u Scan %2zu: %'0.2f Mbit/sec\n", t->num, j, mbps);
#else
            printf("T %2u Scan %2zu: %0.2f Mbit/sec\n", t->num, j, mbps);
#endif
        }
    }
    printf("\n");
}

static
double fastestResult(const vector<unique_ptr<ThreadContext>> &threads) {
    double best = threads[0]->results[0].seconds;
    for (const auto &t : threads) {
        for (const auto &r : t->results) {
            best = min(best, r.seconds);
        }
    }
    return best;
}

static
u64a byte_size(const vector<DataBlock> &corpus_blocks) {
    u64a total = 0;
    for (const DataBlock &block : corpus_blocks) {
        total += block.payload.size();
    }

    return total;
}

/** Dump benchmark results to screen. */
static
void displayResults(const vector<unique_ptr<ThreadContext>> &threads,
                    const vector<DataBlock> &corpus_blocks) {
    u64a bytesPerRun = byte_size(corpus_blocks);
    u64a matchesPerRun = threads[0]->results[0].matches;

    // Sanity check: all of our results should have the same match count.
    for (const auto &t : threads) {
        if (!all_of(begin(t->results), end(t->results),
                    [&matchesPerRun](const ResultEntry &e) {
                        return e.matches == matchesPerRun;
                    })) {
            printf("\nWARNING: PER-SCAN MATCH COUNTS ARE INCONSISTENT!\n\n");
            break;
        }
    }

#ifndef _WIN32
    printf("Time spent scanning:       %'0.3f seconds\n", totalSecs);
    printf("Corpus size:               %'llu bytes ", bytesPerRun);
    switch (scan_mode) {
    case ScanMode::STREAMING:
        printf("(%'zu blocks in %'llu streams)\n", corpus_blocks.size(),
               count_streams(corpus_blocks));
        break;
    case ScanMode::VECTORED:
        printf("(%'zu blocks in %'llu vectors)\n", corpus_blocks.size(),
               count_streams(corpus_blocks));
        break;
    case ScanMode::BLOCK:
        printf("(%'zu blocks)\n", corpus_blocks.size());
        break;
    }
#else
    printf("Time spent scanning:       %0.3f seconds\n", totalSecs);
    printf("Corpus size:               %llu bytes ", bytesPerRun);
    switch (scan_mode) {
    case ScanMode::STREAMING:
        printf("(%zu blocks in %llu streams)\n", corpus_blocks.size(),
               count_streams(corpus_blocks));
        break;
    case ScanMode::VECTORED:
        printf("(%zu blocks in %llu vectors)\n", corpus_blocks.size(),
               count_streams(corpus_blocks));
        break;
    case ScanMode::BLOCK:
        printf("(%zu blocks)\n", corpus_blocks.size());
        break;
    }
#endif

    u64a totalBytes = bytesPerRun * repeats * threads.size();
    u64a totalBlocks = corpus_blocks.size() * repeats * threads.size();

    double matchRate = ((double)matchesPerRun * 1024) / bytesPerRun;
#ifndef _WIN32
    printf("Matches per iteration:     %'llu (%'0.3f matches/kilobyte)\n",
           matchesPerRun, matchRate);
#else
    printf("Matches per iteration:     %llu (%0.3f matches/kilobyte)\n",
           matchesPerRun, matchRate);
#endif

    double blockRate = (double)totalBlocks / (double)totalSecs;
#ifndef _WIN32
    printf("Overall block rate:        %'0.2f blocks/sec\n", blockRate);
    printf("Mean throughput (overall): %'0.2Lf Mbit/sec\n",
           calc_mbps(totalSecs, totalBytes));

#else
    printf("Overall block rate:        %0.2f blocks/sec\n", blockRate);
    printf("Mean throughput (overall): %0.2Lf Mbit/sec\n",
           calc_mbps(totalSecs, totalBytes));

#endif
    double lowestScanTime = fastestResult(threads);
#ifndef _WIN32
    printf("Max throughput (per core): %'0.2Lf Mbit/sec\n",
           calc_mbps(lowestScanTime, bytesPerRun));
#else
    printf("Max throughput (per core): %0.2Lf Mbit/sec\n",
           calc_mbps(lowestScanTime, bytesPerRun));
#endif
    printf("\n");

    if (display_per_scan) {
        displayPerScanResults(threads, bytesPerRun);
    }
}

/** Dump per-scan throughput data to sql. */
static
void sqlPerScanResults(const vector<unique_ptr<ThreadContext>> &threads,
                       u64a bytesPerRun, u64a scan_id) {
    static const std::string Q =
        "INSERT INTO ScanResults (scan_id, thread, scan, throughput) "
        "VALUES (?1, ?2, ?3, ?4)";

    for (const auto &t : threads) {
        const auto &results = t->results;
        for (size_t j = 0; j != results.size(); j++) {
            const auto &r = results[j];
            double mbps = calc_mbps(r.seconds, bytesPerRun);
            out_db.insert_all(Q, scan_id, t->num, j, mbps);
        }
    }
}

/** Dump benchmark results to sql. */
static
void sqlResults(const vector<unique_ptr<ThreadContext>> &threads,
                const vector<DataBlock> &corpus_blocks) {
    u64a bytesPerRun = byte_size(corpus_blocks);
    u64a matchesPerRun = threads[0]->results[0].matches;

    u64a scan_id = out_db.lastRowId();

    // Sanity check: all of our results should have the same match count.
    for (const auto &t : threads) {
        if (!all_of(begin(t->results), end(t->results),
                    [&matchesPerRun](const ResultEntry &e) {
                        return e.matches == matchesPerRun;
                    })) {
            printf("\nWARNING: PER-SCAN MATCH COUNTS ARE INCONSISTENT!\n\n");
            break;
        }
    }

    u64a totalBytes = bytesPerRun * repeats * threads.size();
    double matchRate = ((double)matchesPerRun * 1024) / bytesPerRun;

    const auto pos = corpusFile.find_last_of('/');
    const auto corpus = corpusFile.substr(pos + 1);

    static const std::string Q =
        "INSERT INTO Scan (scan_id, corpusFile, totalSecs, "
            "bytesPerRun, blockSize, blockCount, totalBytes, "
            "totalBlocks, matchesPerRun, matchRate, overallTput) "
        "VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10, ?11)";

    out_db.insert_all(
        Q, scan_id, corpus, totalSecs, bytesPerRun, corpus_blocks.size(),
        scan_mode == ScanMode::BLOCK ? 1 : count_streams(corpus_blocks),
        totalBytes, corpus_blocks.size() * repeats * threads.size(),
        matchesPerRun, matchRate, calc_mbps(totalSecs, totalBytes));

    if (display_per_scan) {
        sqlPerScanResults(threads, bytesPerRun, scan_id);
    }
}

/**
 * Construct a thread context for this scanning mode.
 *
 * Note: does not take blocks by reference. This is to give every thread their
 * own copy of the data. It would be unrealistic for every thread to be scanning
 * the same copy of the data.
 */
static
unique_ptr<ThreadContext> makeThreadContext(const Engine &db,
                                            const vector<DataBlock> &blocks,
                                            unsigned id,
                                            thread_barrier &sync_barrier) {
    thread_func_t fn = nullptr;
    switch (scan_mode) {
    case ScanMode::STREAMING:
        if (compressStream) {
            fn = benchStreamingCompress;
        } else {
            fn = benchStreaming;
        }
        break;
    case ScanMode::VECTORED:
        fn = benchVectored;
        break;
    case ScanMode::BLOCK:
        fn = benchBlock;
        break;
    }
    assert(fn);

    return ue2::make_unique<ThreadContext>(id, db, sync_barrier, fn, blocks);
}

/** Run the given benchmark. */
static
void runBenchmark(const Engine &db,
                  const vector<DataBlock> &corpus_blocks) {
    size_t numThreads;
    bool useAffinity = false;

    if (threadCores.empty()) {
        numThreads = 1;
    } else {
        numThreads = threadCores.size();
#if defined(HAVE_DECL_PTHREAD_SETAFFINITY_NP) || defined(_WIN32)
        useAffinity = true;
#else
        useAffinity = false;
#endif
    }

    // Initialise a barrier that will let us sync threads before/after scanning
    // for timer measurements.
    thread_barrier sync_barrier(numThreads);

    vector<unique_ptr<ThreadContext>> threads;

    for (unsigned i = 0; i < numThreads; i++) {
        auto t = makeThreadContext(db, corpus_blocks, i, sync_barrier);
        int core = useAffinity ? (int)threadCores[i] : -1;
        if (!t->start(core)) {
            printf("Unable to start processing thread %u\n", i);
            exit(1);
        }
        threads.push_back(move(t));
    }

    // Reap threads.
    for (auto &t : threads) {
        t->join();
    }

    if (sqloutFile.empty()) {
        // Display global results.
        displayResults(threads, corpus_blocks);
    } else {
        // write to sqlite file
        sqlResults(threads, corpus_blocks);
        out_db.exec("END");
    }
}
} // namespace

/** Main driver. */
int HS_CDECL main(int argc, char *argv[]) {
    unique_ptr<Grey> grey;
#if !defined(RELEASE_BUILD)
    grey = make_unique<Grey>();
#endif
    setlocale(LC_ALL, ""); // use the user's locale

#ifndef NDEBUG
    printf("\nWARNING: DO NOT BENCHMARK A HYPERSCAN BUILD WITH ASSERTIONS\n\n");
#endif

    vector<BenchmarkSigs> sigSets;
    processArgs(argc, argv, sigSets, grey);

    // read in and process our expressions
    ExpressionMap exprMapTemplate;
    loadExpressions(exprPath, exprMapTemplate);

    // If we have no signature sets, the user wants us to benchmark all the
    // known expressions together.
    if (sigSets.empty()) {
        SignatureSet sigs;
        sigs.reserve(exprMapTemplate.size());
        for (auto i : exprMapTemplate | map_keys) {
            sigs.push_back(i);
        }
        sigSets.emplace_back(exprPath, move(sigs));
    }

    // read in and process our corpus
    vector<DataBlock> corpus_blocks;
    try {
        corpus_blocks = readCorpus(corpusFile);
    } catch (const DataCorpusError &e) {
        printf("Corpus data error: %s\n", e.msg.c_str());
        return 1;
    }
    try {
        if (!sqloutFile.empty()) {
            out_db.open(sqloutFile);
        }

        for (const auto &s : sigSets) {
            auto exprMap = limitToSignatures(exprMapTemplate, s.sigs);
            if (exprMap.empty()) {
                continue;
            }

            unique_ptr<Engine> engine;
            if (useHybrid) {
#if defined(HS_HYBRID)
                engine = buildEngineChimera(exprMap, s.name, sigName);
            } else if (usePcre) {
                engine = buildEnginePcre(exprMap, s.name, sigName);
#endif
            } else {
                engine = buildEngineHyperscan(exprMap, scan_mode, s.name,
                                              sigName, *grey);
            }

            if (!engine) {
                printf("Error: expressions failed to compile.\n");
                exit(1);
            }

            if (sqloutFile.empty()) {
                // Display global results.
                engine->printStats();
                printf("\n");

            } else {
                out_db.exec("BEGIN");
                engine->sqlStats(out_db);
            }

            runBenchmark(*engine, corpus_blocks);
        }
    } catch (const SqlFailure &f) {
        cerr << f.message << '\n';
        return -1;
    }

    return 0;
}
