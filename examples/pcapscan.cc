/*
 * Copyright (c) 2015-2016, Intel Corporation
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
 * Hyperscan example program 2: pcapscan
 *
 * This example is a very simple packet scanning benchmark. It scans a given
 * PCAP file full of network traffic against a group of regular expressions and
 * returns some coarse performance measurements.  This example provides a quick
 * way to examine the performance achievable on a particular combination of
 * platform, pattern set and input data.
 *
 * Build instructions:
 *
 *     g++ -std=c++11 -O2 -o pcapscan pcapscan.cc $(pkg-config --cflags --libs libhs) -lpcap
 *
 * Usage:
 *
 *     ./pcapscan [-n repeats] <pattern file> <pcap file>
 *
 * We recommend the use of a utility like 'taskset' on multiprocessor hosts to
 * pin execution to a single processor: this will remove processor migration
 * by the scheduler as a source of noise in the results.
 *
 */

#include <cstring>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

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
        const struct udphdr *uh =
            (const struct udphdr *)(((const char *)iphdr) + (iphdr->ip_hl * 4));
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
    // Packet data to be scanned.
    vector<string> packets;

    // The stream ID to which each packet belongs
    vector<size_t> stream_ids;

    // Map used to construct stream_ids
    unordered_map<FiveTuple, size_t, FiveTupleHash> stream_map;

    // Hyperscan compiled database (streaming mode)
    const hs_database_t *db_streaming;

    // Hyperscan compiled database (block mode)
    const hs_database_t *db_block;

    // Hyperscan temporary scratch space (used in both modes)
    hs_scratch_t *scratch;

    // Vector of Hyperscan stream state (used in streaming mode)
    vector<hs_stream_t *> streams;

    // Count of matches found during scanning
    size_t matchCount;

public:
    Benchmark(const hs_database_t *streaming, const hs_database_t *block)
        : db_streaming(streaming), db_block(block), scratch(nullptr),
          matchCount(0) {
        // Allocate enough scratch space to handle either streaming or block
        // mode, so we only need the one scratch region.
        hs_error_t err = hs_alloc_scratch(db_streaming, &scratch);
        if (err != HS_SUCCESS) {
            cerr << "ERROR: could not allocate scratch space. Exiting." << endl;
            exit(-1);
        }
        // This second call will increase the scratch size if more is required
        // for block mode.
        err = hs_alloc_scratch(db_block, &scratch);
        if (err != HS_SUCCESS) {
            cerr << "ERROR: could not allocate scratch space. Exiting." << endl;
            exit(-1);
        }
    }

    ~Benchmark() {
        // Free scratch region
        hs_free_scratch(scratch);
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
            hs_error_t err = hs_open_stream(db_streaming, 0, &stream);
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
            hs_error_t err = hs_close_stream(stream, scratch, onMatch,
                                             &matchCount);
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
            hs_error_t err = hs_scan(db_block, pkt.c_str(), pkt.length(), 0,
                                     scratch, onMatch, &matchCount);
            if (err != HS_SUCCESS) {
                cerr << "ERROR: Unable to scan packet. Exiting." << endl;
                exit(-1);
            }
        }
    }

    // Display some information about the compiled database and scanned data.
    void displayStats() {
        size_t numPackets = packets.size();
        size_t numStreams = stream_map.size();
        size_t numBytes = bytes();
        hs_error_t err;

        cout << numPackets << " packets in " << numStreams
             << " streams, totalling " << numBytes << " bytes." << endl;
        cout << "Average packet length: " << numBytes / numPackets << " bytes."
             << endl;
        cout << "Average stream length: " << numBytes / numStreams << " bytes."
             << endl;
        cout << endl;

        size_t dbStream_size = 0;
        err = hs_database_size(db_streaming, &dbStream_size);
        if (err == HS_SUCCESS) {
            cout << "Streaming mode Hyperscan database size    : "
                 << dbStream_size << " bytes." << endl;
        } else {
            cout << "Error getting streaming mode Hyperscan database size"
                 << endl;
        }

        size_t dbBlock_size = 0;
        err = hs_database_size(db_block, &dbBlock_size);
        if (err == HS_SUCCESS) {
            cout << "Block mode Hyperscan database size        : "
                 << dbBlock_size << " bytes." << endl;
        } else {
            cout << "Error getting block mode Hyperscan database size"
                 << endl;
        }

        size_t stream_size = 0;
        err = hs_stream_size(db_streaming, &stream_size);
        if (err == HS_SUCCESS) {
            cout << "Streaming mode Hyperscan stream state size: "
                 << stream_size << " bytes (per stream)." << endl;
        } else {
            cout << "Error getting stream state size" << endl;
        }
    }
};

// helper function - see end of file
static void parseFile(const char *filename, vector<string> &patterns,
                      vector<unsigned> &flags, vector<unsigned> &ids);

static hs_database_t *buildDatabase(const vector<const char *> &expressions,
                                    const vector<unsigned> flags,
                                    const vector<unsigned> ids,
                                    unsigned int mode) {
    hs_database_t *db;
    hs_compile_error_t *compileErr;
    hs_error_t err;

    Clock clock;
    clock.start();

    err = hs_compile_multi(expressions.data(), flags.data(), ids.data(),
                           expressions.size(), mode, nullptr, &db, &compileErr);

    clock.stop();

    if (err != HS_SUCCESS) {
        if (compileErr->expression < 0) {
            // The error does not refer to a particular expression.
            cerr << "ERROR: " << compileErr->message << endl;
        } else {
            cerr << "ERROR: Pattern '" << expressions[compileErr->expression]
                 << "' failed compilation with error: " << compileErr->message
                 << endl;
        }
        // As the compileErr pointer points to dynamically allocated memory, if
        // we get an error, we must be sure to release it. This is not
        // necessary when no error is detected.
        hs_free_compile_error(compileErr);
        exit(-1);
    }

    cout << "Hyperscan " << (mode == HS_MODE_STREAM ? "streaming" : "block")
         << " mode database compiled in " << clock.seconds() << " seconds."
         << endl;

    return db;
}

/**
 * This function will read in the file with the specified name, with an
 * expression per line, ignoring lines starting with '#' and build a Hyperscan
 * database for it.
 */
static void databasesFromFile(const char *filename,
                              hs_database_t **db_streaming,
                              hs_database_t **db_block) {
    // hs_compile_multi requires three parallel arrays containing the patterns,
    // flags and ids that we want to work with. To achieve this we use
    // vectors and new entries onto each for each valid line of input from
    // the pattern file.
    vector<string> patterns;
    vector<unsigned> flags;
    vector<unsigned> ids;

    // do the actual file reading and string handling
    parseFile(filename, patterns, flags, ids);

    // Turn our vector of strings into a vector of char*'s to pass in to
    // hs_compile_multi. (This is just using the vector of strings as dynamic
    // storage.)
    vector<const char*> cstrPatterns;
    for (const auto &pattern : patterns) {
        cstrPatterns.push_back(pattern.c_str());
    }

    cout << "Compiling Hyperscan databases with " << patterns.size()
         << " patterns." << endl;

    *db_streaming = buildDatabase(cstrPatterns, flags, ids, HS_MODE_STREAM);
    *db_block = buildDatabase(cstrPatterns, flags, ids, HS_MODE_BLOCK);
}

static void usage(const char *prog) {
    cerr << "Usage: " << prog << " [-n repeats] <pattern file> <pcap file>" << endl;
}

// Main entry point.
int main(int argc, char **argv) {
    unsigned int repeatCount = 1;

    // Process command line arguments.
    int opt;
    while ((opt = getopt(argc, argv, "n:")) != -1) {
        switch (opt) {
        case 'n':
            repeatCount = atoi(optarg);
            break;
        default:
            usage(argv[0]);
            exit(-1);
        }
    }

    if (argc - optind != 2) {
        usage(argv[0]);
        exit(-1);
    }

    const char *patternFile = argv[optind];
    const char *pcapFile = argv[optind + 1];

    // Read our pattern set in and build Hyperscan databases from it.
    cout << "Pattern file: " << patternFile << endl;
    hs_database_t *db_streaming, *db_block;
    databasesFromFile(patternFile, &db_streaming, &db_block);

    // Read our input PCAP file in
    Benchmark bench(db_streaming, db_block);
    cout << "PCAP input file: " << pcapFile << endl;
    if (!bench.readStreams(pcapFile)) {
        cerr << "Unable to read packets from PCAP file. Exiting." << endl;
        exit(-1);
    }

    if (repeatCount != 1) {
        cout << "Repeating PCAP scan " << repeatCount << " times." << endl;
    }

    bench.displayStats();

    Clock clock;

    // Streaming mode scans.
    double secsStreamingScan = 0.0, secsStreamingOpenClose = 0.0;
    for (unsigned int i = 0; i < repeatCount; i++) {
        // Open streams.
        clock.start();
        bench.openStreams();
        clock.stop();
        secsStreamingOpenClose += clock.seconds();

        // Scan all our packets in streaming mode.
        clock.start();
        bench.scanStreams();
        clock.stop();
        secsStreamingScan += clock.seconds();

        // Close streams.
        clock.start();
        bench.closeStreams();
        clock.stop();
        secsStreamingOpenClose += clock.seconds();
    }

    // Collect data from streaming mode scans.
    size_t bytes = bench.bytes();
    double tputStreamScanning = (bytes * 8 * repeatCount) / secsStreamingScan;
    double tputStreamOverhead = (bytes * 8 * repeatCount) / (secsStreamingScan + secsStreamingOpenClose);
    size_t matchesStream = bench.matches();
    double matchRateStream = matchesStream / ((bytes * repeatCount) / 1024.0); // matches per kilobyte

    // Scan all our packets in block mode.
    bench.clearMatches();
    clock.start();
    for (unsigned int i = 0; i < repeatCount; i++) {
        bench.scanBlock();
    }
    clock.stop();
    double secsScanBlock = clock.seconds();

    // Collect data from block mode scans.
    double tputBlockScanning = (bytes * 8 * repeatCount) / secsScanBlock;
    size_t matchesBlock = bench.matches();
    double matchRateBlock = matchesBlock / ((bytes * repeatCount) / 1024.0); // matches per kilobyte

    cout << endl << "Streaming mode:" << endl << endl;
    cout << "  Total matches: " << matchesStream << endl;
    cout << std::fixed << std::setprecision(4);
    cout << "  Match rate:    " << matchRateStream << " matches/kilobyte" << endl;
    cout << std::fixed << std::setprecision(2);
    cout << "  Throughput (with stream overhead): "
              << tputStreamOverhead/1000000 << " megabits/sec" << endl;
    cout << "  Throughput (no stream overhead):   "
              << tputStreamScanning/1000000 << " megabits/sec" << endl;

    cout << endl << "Block mode:" << endl << endl;
    cout << "  Total matches: " << matchesBlock << endl;
    cout << std::fixed << std::setprecision(4);
    cout << "  Match rate:    " << matchRateBlock << " matches/kilobyte" << endl;
    cout << std::fixed << std::setprecision(2);
    cout << "  Throughput:    "
              << tputBlockScanning/1000000 << " megabits/sec" << endl;

    cout << endl;
    if (bytes < (2*1024*1024)) {
        cout << endl << "WARNING: Input PCAP file is less than 2MB in size." << endl
                  << "This test may have been too short to calculate accurate results." << endl;
    }

    // Close Hyperscan databases
    hs_free_database(db_streaming);
    hs_free_database(db_block);

    return 0;
}

/**
 * Helper function to locate the offset of the first byte of the payload in the
 * given ethernet frame. Offset into the packet, and the length of the payload
 * are returned in the arguments @a offset and @a length.
 */
static bool payloadOffset(const unsigned char *pkt_data, unsigned int *offset,
                          unsigned int *length) {
    const ip *iph = (const ip *)(pkt_data + sizeof(ether_header));
    const tcphdr *th = nullptr;

    // Ignore packets that aren't IPv4
    if (iph->ip_v != 4) {
        return false;
    }

    // Ignore fragmented packets.
    if (iph->ip_off & htons(IP_MF|IP_OFFMASK)) {
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
                      vector<unsigned> &flags, vector<unsigned> &ids) {
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

        patterns.push_back(pcre);
        flags.push_back(flag);
        ids.push_back(id);
    }
}

