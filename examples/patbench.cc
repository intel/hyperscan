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

/*
 * Hyperscan pattern benchmarker.
 *
 * This program allows users to detect which signatures may be the most
 * expensive in a set of patterns. It is designed for use with small to medium
 * pattern set sizes (e.g. 5-500). If used with very large pattern sets it may
 * take a very long time - the number of recompiles done is g * O(lg2(n)) where
 * g is the number of generations and n is the number of patterns (assuming
 * that n >> g).
 *
 * This utility will return a cumulative series of removed patterns. The first
 * generation will find and remove a single pattern. The second generation will
 * begin with the first pattern removed and find another pattern to remove,
 * etc. So if we have 100 patterns and 15 generations, the final generation's
 * score will be a run over 85 patterns.
 *
 * This utility is probabilistic. It is possible that the pattern removed in a
 * generation is not a particularly expensive pattern. To reduce noise in the
 * results use 'taskset' and set the number of repeats to a level that still
 * completes in reasonable time (this will reduce the effect of random
 * measurement noise).
 *
 * The criterion for performance can be altered by use of the -C<x> flag where
 * <x> can be t,r,s,c,b, selecting pattern matching throughput, scratch size,
 * stream state size (only available in streaming mode), compile time and
 * bytecode size respectively.
 *
 * This utility will also not produce good results if all the patterns are
 * roughly equally expensive.
 *
 * Factor Group Size:
 *
 * If there are multiple expensive patterns that are very similar on the
 * left-hand-side or identical, this utility will typically not find these
 * groups unless the -F flag is used to search for a group size that is equal
 * to or larger than the size of the group of similar patterns.
 *
 * Otherwise, removing a portion of the similar patterns will have no or almost
 * no effect, and the search procedure used relies on the ability to remove all
 * of the similar patterns in at least one search case, something which will
 * only happen if the factor_group_size is large enough.
 *
 * This alters the operation of our tool so that instead of trying to find the
 * single pattern whose removal has the most effect by binary search (the
 * default with factor_group_size == 1), we attempt to find the N patterns
 * whose removal has the most effect by searching over N+1 evenly sized groups,
 * removing only 1/(N+1) of the search signatures per iteration.
 *
 * Note that the number of recompiles done greatly increases with increased
 * factor group size.  For example, with factor_group_size = 1, we do g * 2 *
 * lg2(n) recompiles, while with factor_group_size = 4, we do g * 4 *
 * log(5/4)(n). Informally the number of generations we require goes up as we
 * eliminate a smaller number of signatures and the we have to do more work per
 * generation.
 *
 *
 * Build instructions:
 *
 *     g++ -o patbench patbench.cc $(pkg-config --cflags --libs libhs) -lpcap
 *
 * Usage:
 *
 *     ./patbench [ -n repeats] [ -G generations] [ -C criterion ]
 *             [ -F factor_group_size ] [ -N | -S ] <pattern file> <pcap file>
 *
 *     -n repeats sets the number of times the PCAP is repeatedly scanned
 *        with the pattern
 *     -G generations sets the number of generations that the algorithm is
 *        run for
 *     -N sets non-streaming mode, -S sets streaming mode (default)
 *     -F sets the factor group size (must be >0); this allows the detection
 *        of multiple interacting factors
 *
 *     -C sets the "criterion", which can be either:
 *          t  throughput (the default) - this requires a pcap file
 *          r  scratch size
 *          s  stream state size
 *          c  compile time
 *          b  bytecode size
 *
 * We recommend the use of a utility like 'taskset' on multiprocessor hosts to
 * lock execution to a single processor: this will remove processor migration
 * by the scheduler as a source of noise in the results.
 *
 */

#include <algorithm>
#include <cstring>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <string>
#include <vector>
#include <unordered_map>

#include <unistd.h>

// We use the BSD primitives throughout as they exist on both BSD and Linux.
#define __FAVOR_BSD
#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <netinet/udp.h>
#include <netinet/ip_icmp.h>
#include <net/ethernet.h>
#include <arpa/inet.h>

#include <pcap.h>

#include <hs.h>

using std::cerr;
using std::cout;
using std::endl;
using std::ifstream;
using std::string;
using std::unordered_map;
using std::vector;
using std::set;
using std::min;
using std::max;
using std::copy;

enum Criterion {
    CRITERION_THROUGHPUT,
    CRITERION_BYTECODE_SIZE,
    CRITERION_COMPILE_TIME,
    CRITERION_STREAM_STATE,
    CRITERION_SCRATCH_SIZE
};

static bool higher_is_better(Criterion c) {
    return c == CRITERION_THROUGHPUT;
}

static void print_criterion(Criterion c, double val) {
    std::ios::fmtflags f(cout.flags());
    switch (c) {
    case CRITERION_THROUGHPUT:
        cout << std::fixed << std::setprecision(3) << val << " Megabits/s";
        break;
    case CRITERION_COMPILE_TIME:
        cout << std::fixed << std::setprecision(3) << val << " seconds";
        break;
    case CRITERION_BYTECODE_SIZE:
    case CRITERION_STREAM_STATE:
    case CRITERION_SCRATCH_SIZE:
    default:
        cout << static_cast<size_t>(val) << " bytes";
        break;
    }
    cout.flags(f);
}

// Key for identifying a stream in our pcap input data, using data from its IP
// headers.
struct FiveTuple {
    unsigned int protocol;
    unsigned int srcAddr;
    unsigned int srcPort;
    unsigned int dstAddr;
    unsigned int dstPort;

    // Construct a FiveTuple from a TCP or UDP packet.
    FiveTuple(const struct ip *iphdr) {
        // IP fields
        protocol = iphdr->ip_p;
        srcAddr = iphdr->ip_src.s_addr;
        dstAddr = iphdr->ip_dst.s_addr;

        // UDP/TCP ports
        const struct udphdr *uh = (const struct udphdr *)
                (((const char *)iphdr) + (iphdr->ip_hl * 4));
        srcPort = uh->uh_sport;
        dstPort = uh->uh_dport;
    }

    bool operator==(const FiveTuple &a) const {
        return protocol == a.protocol && srcAddr == a.srcAddr &&
               srcPort == a.srcPort && dstAddr == a.dstAddr &&
               dstPort == a.dstPort;
    }
};

// A *very* simple hash function, used when we create an unordered_map of
// FiveTuple objects.
struct FiveTupleHash {
    size_t operator()(const FiveTuple &x) const {
        return x.srcAddr ^ x.dstAddr ^ x.protocol ^ x.srcPort ^ x.dstPort;
    }
};

// Helper function. See end of file.
static bool payloadOffset(const unsigned char *pkt_data, unsigned int *offset,
                          unsigned int *length);

// Match event handler: called every time Hyperscan finds a match.
static
int onMatch(unsigned int id, unsigned long long from, unsigned long long to,
            unsigned int flags, void *ctx) {
    // Our context points to a size_t storing the match count
    size_t *matches = (size_t *)ctx;
    (*matches)++;
    return 0; // continue matching
}

// Simple timing class
class Clock {
public:
    void start() {
        time_start = std::chrono::system_clock::now();
    }

    void stop() {
        time_end = std::chrono::system_clock::now();
    }

    double seconds() const {
        std::chrono::duration<double> delta = time_end - time_start;
        return delta.count();
    }
private:
    std::chrono::time_point<std::chrono::system_clock> time_start, time_end;
};

// Class wrapping all state associated with the benchmark
class Benchmark {
private:
    // Packet data to be scanned
    vector<string> packets;

    // Stream ID for each packet
    vector<size_t> stream_ids;

    // Map used to construct stream_ids
    unordered_map<FiveTuple, size_t, FiveTupleHash> stream_map;

    // Hyperscan compiled database
    hs_database_t *db = nullptr;

    // Hyperscan temporary scratch space
    hs_scratch_t *scratch = nullptr;

    // Vector of Hyperscan stream state
    vector<hs_stream_t *> streams;

    // Count of matches found while scanning
    size_t matchCount = 0;
public:
    ~Benchmark() {
        hs_free_scratch(scratch);
        hs_free_database(db);
    }

    // Initialisation; after this call, Benchmark owns the database and will
    // ensure it is freed.
    void setDatabase(hs_database_t *hs_db) {
        hs_free_database(db); // Free previous database.
        db = hs_db;
        // (Re)allocate scratch to ensure that it is large enough to handle the
        // database.
        hs_error_t err = hs_alloc_scratch(db, &scratch);
        if (err != HS_SUCCESS) {
            cerr << "ERROR: could not allocate scratch space. Exiting." << endl;
            exit(-1);
        }
    }
    const hs_database_t *getDatabase() const {
        return db;
    }

    size_t getScratchSize() const {
        size_t scratch_size;
        hs_error_t err = hs_scratch_size(scratch, &scratch_size);
        if (err != HS_SUCCESS) {
            cerr << "ERROR: could not query scratch space size. Exiting."
                 << endl;
            exit(-1);
        }
        return scratch_size;
    }

    // Read a set of streams from a pcap file
    bool readStreams(const char *pcapFile) {
        // Open PCAP file for input
        char errbuf[PCAP_ERRBUF_SIZE];
        pcap_t *pcapHandle = pcap_open_offline(pcapFile, errbuf);
        if (pcapHandle == nullptr) {
            cerr << "ERROR: Unable to open pcap file \"" << pcapFile
                 << "\": " << errbuf << endl;
            return false;
        }

        struct pcap_pkthdr pktHeader;
        const unsigned char *pktData;
        while ((pktData = pcap_next(pcapHandle, &pktHeader)) != nullptr) {
            unsigned int offset = 0, length = 0;
            if (!payloadOffset(pktData, &offset, &length)) {
                continue;
            }

            // Valid TCP or UDP packet
            const struct ip *iphdr = (const struct ip *)(pktData
                    + sizeof(struct ether_header));
            const char *payload = (const char *)pktData + offset;

            size_t id = stream_map.insert(std::make_pair(FiveTuple(iphdr),
                                          stream_map.size())).first->second;

            packets.push_back(string(payload, length));
            stream_ids.push_back(id);
        }
        pcap_close(pcapHandle);

        return !packets.empty();
    }

    // Return the number of bytes scanned
    size_t bytes() const {
        size_t sum = 0;
        for (const auto &packet : packets) {
            sum += packet.size();
        }
        return sum;
    }

    // Return the number of matches found.
    size_t matches() const {
        return matchCount;
    }

    // Clear the number of matches found.
    void clearMatches() {
        matchCount = 0;
    }

    // Open a Hyperscan stream for each stream in stream_ids
    void openStreams() {
        streams.resize(stream_map.size());
        for (auto &stream : streams) {
            hs_error_t err = hs_open_stream(db, 0, &stream);
            if (err != HS_SUCCESS) {
                cerr << "ERROR: Unable to open stream. Exiting." << endl;
                exit(-1);
            }
        }
    }

    // Close all open Hyperscan streams (potentially generating any
    // end-anchored matches)
    void closeStreams() {
        for (auto &stream : streams) {
            hs_error_t err =
                hs_close_stream(stream, scratch, onMatch, &matchCount);
            if (err != HS_SUCCESS) {
                cerr << "ERROR: Unable to close stream. Exiting." << endl;
                exit(-1);
            }
        }
    }

    // Scan each packet (in the ordering given in the PCAP file) through
    // Hyperscan using the streaming interface.
    void scanStreams() {
        for (size_t i = 0; i != packets.size(); ++i) {
            const std::string &pkt = packets[i];
            hs_error_t err = hs_scan_stream(streams[stream_ids[i]],
                                            pkt.c_str(), pkt.length(), 0,
                                            scratch, onMatch, &matchCount);
            if (err != HS_SUCCESS) {
                cerr << "ERROR: Unable to scan packet. Exiting." << endl;
                exit(-1);
            }
        }
    }

    // Scan each packet (in the ordering given in the PCAP file) through
    // Hyperscan using the block-mode interface.
    void scanBlock() {
        for (size_t i = 0; i != packets.size(); ++i) {
            const std::string &pkt = packets[i];
            hs_error_t err = hs_scan(db, pkt.c_str(), pkt.length(), 0,
                                     scratch, onMatch, &matchCount);
            if (err != HS_SUCCESS) {
                cerr << "ERROR: Unable to scan packet. Exiting." << endl;
                exit(-1);
            }
        }
    }
};

// helper function - see end of file
static void parseFile(const char *filename, vector<string> &patterns,
                      vector<unsigned> &flags, vector<unsigned> &ids,
                      vector<string> &originals);

class Sigdata {
    vector<unsigned> flags;
    vector<unsigned> ids;
    vector<string> patterns;
    vector<string> originals;

public:
    Sigdata() {}
    Sigdata(const char *filename) {
        parseFile(filename, patterns, flags, ids, originals);

    }

    const string &get_original(unsigned index) const {
        return originals[index];
    }

    hs_database_t *compileDatabase(unsigned mode, double *compileTime) const {
        hs_database_t *db = nullptr;
        hs_compile_error_t *compileErr;

        // Turn our vector of strings into a vector of char*'s to pass in to
        // hs_compile_multi. (This is just using the vector of strings as
        // dynamic storage.)
        vector<const char *> cstrPatterns;
        cstrPatterns.reserve(patterns.size());
        for (const auto &pattern : patterns) {
            cstrPatterns.push_back(pattern.c_str());
        }

        Clock clock;
        clock.start();
        hs_error_t err = hs_compile_multi(cstrPatterns.data(), flags.data(),
                                          ids.data(), cstrPatterns.size(), mode,
                                          nullptr, &db, &compileErr);
        clock.stop();
        if (err != HS_SUCCESS) {
            if (compileErr->expression < 0) {
                // The error does not refer to a particular expression.
                cerr << "ERROR: " << compileErr->message << endl;
            } else {
                cerr << "ERROR: Pattern '"
                     << patterns[compileErr->expression]
                     << "' failed with error '" << compileErr->message << "'"
                     << endl;
            }
            // As the compileErr pointer points to dynamically allocated memory,
            // if we get an error, we must be sure to release it. This is not
            // necessary when no error is detected.
            hs_free_compile_error(compileErr);
            exit(-1);
        }

        *compileTime = clock.seconds();
        return db;
    }

    unsigned size() const {
        return patterns.size();
    }

    Sigdata cloneExclude(const set<unsigned> &excludeIndexSet) const {
        Sigdata c;
        for (unsigned i = 0, e = size(); i != e; ++i) {
            if (excludeIndexSet.find(i) == excludeIndexSet.end()) {
                c.flags.push_back(flags[i]);
                c.ids.push_back(ids[i]);
                c.patterns.push_back(patterns[i]);
                c.originals.push_back(originals[i]);
            }
        }
        return c;
    }
};

static
void usage(const char *) {
    cerr << "Usage:" << endl << endl;
    cerr << "  patbench [-n repeats] [ -G generations] [ -C criterion ]" << endl
         << "           [ -F factor_group_size ] [ -N | -S ] "
         << "<pattern file> <pcap file>" << endl << endl
         << "    -n repeats sets the number of times the PCAP is repeatedly "
            "scanned" << endl << "       with the pattern." << endl
         << "    -G generations sets the number of generations that the "
            "algorithm is" << endl << "       run for." << endl
         << "    -N sets non-streaming mode, -S sets streaming mode (default)."
         << endl << "    -F sets the factor group size (must be >0); this "
                    "allows the detection" << endl
         << "       of multiple interacting factors." << endl << "" << endl
         << "    -C sets the 'criterion', which can be either:" << endl
         << "         t  throughput (the default) - this requires a pcap file"
         << endl << "         r  scratch size" << endl
         << "         s  stream state size" << endl
         << "         c  compile time" << endl << "         b  bytecode size"
         << endl << endl
         << "We recommend the use of a utility like 'taskset' on "
            "multiprocessor hosts to" << endl
         << "lock execution to a single processor: this will remove processor "
            "migration" << endl
         << "by the scheduler as a source of noise in the results." << endl;
}

static
double measure_stream_time(Benchmark &bench, unsigned int repeatCount) {
    Clock clock;
    bench.clearMatches();
    clock.start();
    for (unsigned int i = 0; i < repeatCount; i++) {
        bench.openStreams();
        bench.scanStreams();
        bench.closeStreams();
    }
    clock.stop();
    double secsScan = clock.seconds();
    return secsScan;
}

static
double measure_block_time(Benchmark &bench, unsigned int repeatCount) {
    Clock clock;
    bench.clearMatches();
    clock.start();
    for (unsigned int i = 0; i < repeatCount; i++) {
        bench.scanBlock();
    }
    clock.stop();
    double secsScan = clock.seconds();
    return secsScan;
}

static
double eval_set(Benchmark &bench, Sigdata &sigs, unsigned int mode,
                unsigned repeatCount, Criterion criterion,
                bool diagnose = true) {
    double compileTime = 0;
    bench.setDatabase(sigs.compileDatabase(mode, &compileTime));

    switch (criterion) {
    case CRITERION_BYTECODE_SIZE: {
        size_t dbSize;
        hs_error_t err = hs_database_size(bench.getDatabase(), &dbSize);
        if (err != HS_SUCCESS) {
            cerr << "ERROR: could not retrieve bytecode size" << endl;
            exit(1);
        }
        return dbSize;
    }
    case CRITERION_COMPILE_TIME:
        return compileTime;
    case CRITERION_STREAM_STATE: {
        size_t streamStateSize;
        hs_error_t err = hs_stream_size(bench.getDatabase(), &streamStateSize);
        if (err != HS_SUCCESS) {
            cerr << "ERROR: could not retrieve stream state size" << endl;
            exit(1);
        }
        return streamStateSize;
    }
    case CRITERION_SCRATCH_SIZE:
        return bench.getScratchSize();
    case CRITERION_THROUGHPUT:
    default:
        break; // do nothing - we are THROUGHPUT
    }
    double scan_time;
    if (mode == HS_MODE_NOSTREAM) {
        scan_time = measure_block_time(bench, repeatCount);
    } else {
        scan_time = measure_stream_time(bench, repeatCount);
    }
    size_t bytes = bench.bytes();
    size_t matches = bench.matches();
    if (diagnose) {
        std::ios::fmtflags f(cout.flags());
        cout << "Scan time " << std::fixed << std::setprecision(3) << scan_time
             << " sec, Scanned " << bytes * repeatCount << " bytes, Throughput "
             << std::fixed << std::setprecision(3)
             << (bytes * 8 * repeatCount) / (scan_time * 1000000)
             << " Mbps, Matches " << matches << endl;
        cout.flags(f);
    }
    return (bytes * 8 * repeatCount) / (scan_time * 1000000);
}

// Main entry point.
int main(int argc, char **argv) {
    unsigned int repeatCount = 1;
    unsigned int mode = HS_MODE_STREAM;
    Criterion criterion = CRITERION_THROUGHPUT;
    unsigned int gen_max = 10;
    unsigned int factor_max = 1;
    // Process command line arguments.
    int opt;
    while ((opt = getopt(argc, argv, "SNn:G:F:C:")) != -1) {
        switch (opt) {
        case 'F':
            factor_max = atoi(optarg);
            break;
        case 'G':
            gen_max = atoi(optarg);
            break;
        case 'S':
            mode = HS_MODE_STREAM;
            break;
        case 'N':
            mode = HS_MODE_NOSTREAM;
            break;
        case 'C':
            switch (optarg[0]) {
            case 't':
                criterion = CRITERION_THROUGHPUT;
                break;
            case 'b':
                criterion = CRITERION_BYTECODE_SIZE;
                break;
            case 'c':
                criterion = CRITERION_COMPILE_TIME;
                break;
            case 's':
                criterion = CRITERION_STREAM_STATE;
                break;
            case 'r':
                criterion = CRITERION_SCRATCH_SIZE;
                break;
            default:
                cerr << "Unrecognised criterion: " << optarg[0] << endl;
                usage(argv[0]);
                exit(-1);
            }
            break;
        case 'n':
            repeatCount = atoi(optarg);
            break;
        default:
            usage(argv[0]);
            exit(-1);
        }
    }

    if (argc - optind != ((criterion == CRITERION_THROUGHPUT) ? 2 : 1)) {
        usage(argv[0]);
        exit(-1);
    }

    const char *patternFile = argv[optind];
    const char *pcapFile = argv[optind + 1];

    // Read our input PCAP file in
    Benchmark bench;
    if (criterion == CRITERION_THROUGHPUT) {
        if (!bench.readStreams(pcapFile)) {
            cerr << "Unable to read packets from PCAP file. Exiting." << endl;
            exit(-1);
        }
    }

    if ((criterion == CRITERION_STREAM_STATE) && (mode != HS_MODE_STREAM)) {
        cerr << "Cannot evaluate stream state for block mode compile. Exiting."
             << endl;
        exit(-1);
    }

    cout << "Base signatures: " << patternFile;
    if (pcapFile) {
        cout << "\tPCAP input file: " << pcapFile
             << "\tRepeat count: " << repeatCount;
    }
    if (mode == HS_MODE_STREAM) {
        cout << "\tMode: streaming";
    } else {
        cout << "\tMode: block";
    }
    cout << endl;

    Sigdata sigs(patternFile);

    // calculate and show a baseline
    eval_set(bench, sigs, mode, repeatCount, criterion);

    set<unsigned> work_sigs, exclude;

    for (unsigned i = 0; i < sigs.size(); ++i) {
        work_sigs.insert(i);
    }

    double score_base =
        eval_set(bench, sigs, mode, repeatCount, criterion, false);
    bool maximize = higher_is_better(criterion);
    cout << "Number of signatures: " << sigs.size() << endl;
    cout << "Base performance: ";
    print_criterion(criterion, score_base);
    cout << endl;

    unsigned generations = min(gen_max, (sigs.size() - 1) / factor_max);

    cout << "Cutting signatures cumulatively for " << generations
         << " generations" << endl;
    for (unsigned gen = 0; gen < generations; ++gen) {
        cout << "Generation " << gen << " ";
        set<unsigned> s(work_sigs.begin(), work_sigs.end());
        double best = maximize ? 0 : 1000000000000.0;
        unsigned count = 0;
        while (s.size() > factor_max) {
            count++;
            cout << "." << std::flush;
            vector<unsigned> sv(s.begin(), s.end());
            random_shuffle(sv.begin(), sv.end());
            unsigned groups = factor_max + 1;
            for (unsigned current_group = 0; current_group < groups;
                 current_group++) {
                unsigned sz = sv.size();
                unsigned lo = (current_group * sz) / groups;
                unsigned hi = ((current_group + 1) * sz) / groups;

                set<unsigned> s_part1(sv.begin(), sv.begin() + lo);
                set<unsigned> s_part2(sv.begin() + hi, sv.end());
                set<unsigned> s_tmp = s_part1;
                s_tmp.insert(s_part2.begin(), s_part2.end());
                set<unsigned> tmp = s_tmp;
                tmp.insert(exclude.begin(), exclude.end());
                Sigdata sigs_tmp = sigs.cloneExclude(tmp);
                double score = eval_set(bench, sigs_tmp, mode, repeatCount,
                                        criterion, false);

                if ((current_group == 0) ||
                    (!maximize ? (score < best) : (score > best))) {
                    s = s_tmp;
                    best = score;
                }
            }
        }
        for (unsigned i = count; i < 16; i++) {
            cout << " ";
        }
        std::ios::fmtflags out_f(cout.flags());
        cout << "Performance: ";
        print_criterion(criterion, best);
        cout << " (" << std::fixed << std::setprecision(3) << (best / score_base)
             << "x) after cutting:" << endl;
        cout.flags(out_f);

        // s now has factor_max signatures
        for (const auto &found : s) {
            exclude.insert(found);
            work_sigs.erase(found);
            cout << sigs.get_original(found) << endl;
        }

        cout << endl;
    }
    return 0;
}

/**
 * Helper function to locate the offset of the first byte of the payload in the
 * given ethernet frame. Offset into the packet, and the length of the payload
 * are returned in the arguments @a offset and @a length.
 */
static
bool payloadOffset(const unsigned char *pkt_data, unsigned int *offset,
                   unsigned int *length) {
    const ip *iph = (const ip *)(pkt_data + sizeof(ether_header));
    const tcphdr *th = nullptr;

    // Ignore packets that aren't IPv4
    if (iph->ip_v != 4) {
        return false;
    }

    // Ignore fragmented packets.
    if (iph->ip_off & htons(IP_MF | IP_OFFMASK)) {
        return false;
    }

    // IP header length, and transport header length.
    unsigned int ihlen = iph->ip_hl * 4;
    unsigned int thlen = 0;

    switch (iph->ip_p) {
    case IPPROTO_TCP:
        th = (const tcphdr *)((const char *)iph + ihlen);
        thlen = th->th_off * 4;
        break;
    case IPPROTO_UDP:
        thlen = sizeof(udphdr);
        break;
    default:
        return false;
    }

    *offset = sizeof(ether_header) + ihlen + thlen;
    *length = sizeof(ether_header) + ntohs(iph->ip_len) - *offset;

    return *length != 0;
}

static unsigned parseFlags(const string &flagsStr) {
    unsigned flags = 0;
    for (const auto &c : flagsStr) {
        switch (c) {
        case 'i':
            flags |= HS_FLAG_CASELESS; break;
        case 'm':
            flags |= HS_FLAG_MULTILINE; break;
        case 's':
            flags |= HS_FLAG_DOTALL; break;
        case 'H':
            flags |= HS_FLAG_SINGLEMATCH; break;
        case 'V':
            flags |= HS_FLAG_ALLOWEMPTY; break;
        case '8':
            flags |= HS_FLAG_UTF8; break;
        case 'W':
            flags |= HS_FLAG_UCP; break;
        case '\r': // stray carriage-return
            break;
        default:
            cerr << "Unsupported flag \'" << c << "\'" << endl;
            exit(-1);
        }
    }
    return flags;
}

static void parseFile(const char *filename, vector<string> &patterns,
                      vector<unsigned> &flags, vector<unsigned> &ids,
                      vector<string> &originals) {
    ifstream inFile(filename);
    if (!inFile.good()) {
        cerr << "ERROR: Can't open pattern file \"" << filename << "\"" << endl;
        exit(-1);
    }

    for (unsigned i = 1; !inFile.eof(); ++i) {
        string line;
        getline(inFile, line);

        // if line is empty, or a comment, we can skip it
        if (line.empty() || line[0] == '#') {
            continue;
        }

        // otherwise, it should be ID:PCRE, e.g.
        //  10001:/foobar/is

        size_t colonIdx = line.find_first_of(':');
        if (colonIdx == string::npos) {
            cerr << "ERROR: Could not parse line " << i << endl;
            exit(-1);
        }

        // we should have an unsigned int as an ID, before the colon
        unsigned id = std::stoi(line.substr(0, colonIdx).c_str());

        // rest of the expression is the PCRE
        const string expr(line.substr(colonIdx + 1));

        size_t flagsStart = expr.find_last_of('/');
        if (flagsStart == string::npos) {
            cerr << "ERROR: no trailing '/' char" << endl;
            exit(-1);
        }

        string pcre(expr.substr(1, flagsStart - 1));
        string flagsStr(expr.substr(flagsStart + 1, expr.size() - flagsStart));
        unsigned flag = parseFlags(flagsStr);

        originals.push_back(line);
        patterns.push_back(pcre);
        flags.push_back(flag);
        ids.push_back(id);
    }
}
