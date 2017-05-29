/*
 * Copyright (c) 2016-2017, Intel Corporation
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

#include "rose_build_long_lit.h"

#include "rose_build_engine_blob.h"
#include "rose_build_impl.h"
#include "stream_long_lit_hash.h"
#include "util/bytecode_ptr.h"
#include "util/bitutils.h"
#include "util/verify_types.h"
#include "util/compile_context.h"

#include <algorithm>
#include <numeric>

using namespace std;

namespace ue2 {

/** \brief Minimum size for a non-empty hash table. Must be a power of two. */
static constexpr size_t MIN_HASH_TABLE_SIZE = 128;

/** \brief Maximum load factor (between zero and one) for a hash table. */
static constexpr double MAX_HASH_TABLE_LOAD = 0.7;

/** \brief Minimum size (in bits) for a bloom filter. Must be a power of two. */
static constexpr u32 MIN_BLOOM_FILTER_SIZE = 256;

/** \brief Maximum load factor (between zero and one) for a bloom filter. */
static constexpr double MAX_BLOOM_FILTER_LOAD = 0.25;

struct LongLitModeInfo {
    u32 num_literals = 0; //!< Number of strings for this mode.
    u32 hashed_positions = 0; //!< Number of hashable string positions.
};

struct LongLitInfo {
    LongLitModeInfo caseful;
    LongLitModeInfo nocase;
};

static
u32 roundUpToPowerOfTwo(u32 x) {
    assert(x != 0);
    u32 bits = lg2(x - 1) + 1;
    assert(bits < 32);
    return 1U << bits;
}

static
LongLitInfo analyzeLongLits(const vector<ue2_case_string> &lits,
                            size_t max_len) {
    LongLitInfo info;

    for (const auto &lit : lits) {
        auto &lit_info = lit.nocase ? info.nocase : info.caseful;
        assert(lit.s.size() > max_len);
        lit_info.num_literals++;
        lit_info.hashed_positions += lit.s.size() - max_len;
    }

    DEBUG_PRINTF("case: hashed %u positions\n", info.caseful.hashed_positions);
    DEBUG_PRINTF("nocase: hashed %u positions\n", info.nocase.hashed_positions);

    return info;
}

static
void addToBloomFilter(vector<u8> &bloom, const u8 *substr, bool nocase) {
    const u32 num_keys = verify_u32(bloom.size() * 8);
    const u32 key_mask = (1U << lg2(num_keys)) -1;

    const auto hash_functions = { bloomHash_1, bloomHash_2, bloomHash_3 };
    for (const auto &hash_func : hash_functions) {
        u32 hash = hash_func(substr, nocase);
        u32 key = hash & key_mask;
        DEBUG_PRINTF("set key %u (of %zu)\n", key, bloom.size() * 8);
        bloom[key / 8] |= 1U << (key % 8);
    }
}

static
size_t bloomOccupancy(const vector<u8> &bloom) {
    return accumulate(begin(bloom), end(bloom), 0,
                      [](const size_t &sum, const u8 &elem) {
                          return sum + popcount32(elem);
                      });
}

static
double bloomLoad(const vector<u8> &bloom) {
    return (double)bloomOccupancy(bloom) / (double)(bloom.size() * 8);
}

static
vector<u8> buildBloomFilter(const vector<ue2_case_string> &lits, size_t max_len,
                            size_t num_entries, bool nocase) {
    assert(num_entries % 8 == 0);
    assert((num_entries & (num_entries - 1)) == 0); // Must be power of two.

    vector<u8> bloom(num_entries / 8, 0);

    if (!num_entries) {
        return bloom;
    }

    for (const auto &lit : lits) {
        if (nocase != lit.nocase) {
            continue;
        }
        for (u32 offset = 1; offset < lit.s.size() - max_len + 1; offset++) {
            const u8 *substr = (const u8 *)lit.s.c_str() + offset;
            addToBloomFilter(bloom, substr, nocase);
        }
    }

    DEBUG_PRINTF("%s bloom filter occupancy %zu of %zu entries\n",
                 nocase ? "nocase" : "caseful", bloomOccupancy(bloom),
                 num_entries);

    return bloom;
}


static
vector<u8> makeBloomFilter(const vector<ue2_case_string> &lits,
                           size_t max_len, bool nocase) {
    vector<u8> bloom;

    size_t num_entries = MIN_BLOOM_FILTER_SIZE;
    for (;;) {
        bloom = buildBloomFilter(lits, max_len, num_entries, nocase);
        DEBUG_PRINTF("built %s bloom for %zu entries: load %f\n",
                     nocase ? "nocase" : "caseful", num_entries,
                     bloomLoad(bloom));
        if (bloomLoad(bloom) < MAX_BLOOM_FILTER_LOAD) {
            break;
        }
        num_entries *= 2;
    }
    return bloom;
}

static UNUSED
size_t hashTableOccupancy(const vector<RoseLongLitHashEntry> &tab) {
    return count_if(begin(tab), end(tab), [](const RoseLongLitHashEntry &ent) {
        return ent.str_offset != 0;
    });
}

static UNUSED
double hashTableLoad(const vector<RoseLongLitHashEntry> &tab) {
    return (double)hashTableOccupancy(tab) / (double)(tab.size());
}

using LitOffsetVector = small_vector<pair<u32, u32>, 1>;

static
vector<RoseLongLitHashEntry> buildHashTable(
               size_t max_len, const vector<u32> &litToOffsetVal,
               const map<u32, LitOffsetVector> &hashToLitOffPairs,
               size_t numEntries) {
    vector<RoseLongLitHashEntry> tab(numEntries, {0,0});

    if (!numEntries) {
        return tab;
    }

    for (const auto &m : hashToLitOffPairs) {
        u32 hash = m.first;
        const LitOffsetVector &d = m.second;

        u32 bucket = hash % numEntries;

        // Placement via linear probing.
        for (const auto &lit_offset : d) {
            while (tab[bucket].str_offset != 0) {
                bucket++;
                if (bucket == numEntries) {
                    bucket = 0;
                }
            }

            u32 lit_id = lit_offset.first;
            u32 offset = lit_offset.second;

            DEBUG_PRINTF("hash 0x%08x lit_id %u offset %u bucket %u\n", hash,
                         lit_id, offset, bucket);

            auto &entry = tab[bucket];
            entry.str_offset = verify_u32(litToOffsetVal.at(lit_id));
            assert(entry.str_offset != 0);
            entry.str_len = offset + max_len;
        }
    }

    DEBUG_PRINTF("hash table occupancy %zu of %zu entries\n",
                 hashTableOccupancy(tab), numEntries);

    return tab;
}

static
map<u32, LitOffsetVector> computeLitHashes(const vector<ue2_case_string> &lits,
                                           size_t max_len, bool nocase) {
    map<u32, LitOffsetVector> hashToLitOffPairs;

    for (u32 lit_id = 0; lit_id < lits.size(); lit_id++) {
        const ue2_case_string &lit = lits[lit_id];
        if (nocase != lit.nocase) {
            continue;
        }
        for (u32 offset = 1; offset < lit.s.size() - max_len + 1; offset++) {
            const u8 *substr = (const u8 *)lit.s.c_str() + offset;
            u32 hash = hashLongLiteral(substr, max_len, lit.nocase);
            hashToLitOffPairs[hash].emplace_back(lit_id, offset);
        }
    }

    for (auto &m : hashToLitOffPairs) {
        LitOffsetVector &d = m.second;
        if (d.size() == 1) {
            continue;
        }

        // Sort by (offset, string) so that we'll be able to remove identical
        // string prefixes.
        stable_sort(begin(d), end(d),
                    [&](const pair<u32, u32> &a, const pair<u32, u32> &b) {
                        const auto &str_a = lits[a.first].s;
                        const auto &str_b = lits[b.first].s;
                        return tie(a.second, str_a) < tie(b.second, str_b);
                    });

        // Remove entries that point to the same literal prefix.
        d.erase(unique(begin(d), end(d),
                       [&](const pair<u32, u32> &a, const pair<u32, u32> &b) {
                           if (a.second != b.second) {
                               return false;
                           }
                           const auto &str_a = lits[a.first].s;
                           const auto &str_b = lits[b.first].s;
                           const size_t len = max_len + a.second;
                           return equal(begin(str_a), begin(str_a) + len,
                                        begin(str_b));
                       }),
                end(d));

        // Sort d by distance of the residual string (len minus our depth into
        // the string). We need to put the 'furthest back' string first.
        stable_sort(begin(d), end(d),
                    [](const pair<u32, u32> &a, const pair<u32, u32> &b) {
                        if (a.second != b.second) {
                            return a.second > b.second; /* longest is first */
                        }
                        return a.first < b.first;
                    });
    }

    return hashToLitOffPairs;
}

static
vector<RoseLongLitHashEntry> makeHashTable(const vector<ue2_case_string> &lits,
                                           size_t max_len,
                                           const vector<u32> &litToOffsetVal,
                                           u32 numPositions, bool nocase) {
    // Compute lit substring hashes.
    const auto hashToLitOffPairs = computeLitHashes(lits, max_len, nocase);

    // Compute the size of the hash table: we need enough entries to satisfy
    // our max load constraint, and it must be a power of two.
    size_t num_entries = (double)numPositions / MAX_HASH_TABLE_LOAD + 1;
    num_entries = roundUpToPowerOfTwo(max(MIN_HASH_TABLE_SIZE, num_entries));

    auto tab = buildHashTable(max_len, litToOffsetVal, hashToLitOffPairs,
                              num_entries);
    DEBUG_PRINTF("built %s hash table for %zu entries: load %f\n",
                 nocase ? "nocase" : "caseful", num_entries,
                 hashTableLoad(tab));
    assert(hashTableLoad(tab) < MAX_HASH_TABLE_LOAD);

    return tab;
}

static
vector<u8> buildLits(const vector<ue2_case_string> &lits, u32 baseOffset,
                     vector<u32> &litToOffsetVal) {
    vector<u8> blob;
    litToOffsetVal.resize(lits.size(), 0);

    u32 lit_id = 0;
    for (const auto &lit : lits) {
        u32 offset = baseOffset + verify_u32(blob.size());
        blob.insert(blob.end(), begin(lit.s), end(lit.s));
        litToOffsetVal[lit_id] = offset;
        lit_id++;
    }

    DEBUG_PRINTF("built %zu bytes of strings\n", blob.size());
    return blob;
}

u32 buildLongLiteralTable(const RoseBuildImpl &build, RoseEngineBlob &blob,
                          vector<ue2_case_string> &lits,
                          size_t longLitLengthThreshold,
                          size_t *historyRequired,
                          size_t *longLitStreamStateRequired) {
    // Work in terms of history requirement (i.e. literal len - 1).
    const size_t max_len = longLitLengthThreshold - 1;

    // We should only be building the long literal hash table in streaming mode.
    if (!build.cc.streaming) {
        return 0;
    }

    if (lits.empty()) {
        DEBUG_PRINTF("no long literals\n");
        return 0;
    }

    // The last char of each literal is trimmed as we're not interested in full
    // matches, only partial matches.
    for (auto &lit : lits) {
        assert(!lit.s.empty());
        lit.s.pop_back();
    }

    // Sort by caseful/caseless and in lexicographical order.
    stable_sort(begin(lits), end(lits), [](const ue2_case_string &a,
                                           const ue2_case_string &b) {
        if (a.nocase != b.nocase) {
            return a.nocase < b.nocase;
        }
        return a.s < b.s;
    });

    // Find literals that are prefixes of other literals (including
    // duplicates). Note that we iterate in reverse, since we want to retain
    // only the longest string from a set of prefixes.
    auto it = unique(lits.rbegin(), lits.rend(), [](const ue2_case_string &a,
                                                    const ue2_case_string &b) {
        return a.nocase == b.nocase && a.s.size() >= b.s.size() &&
               equal(b.s.begin(), b.s.end(), a.s.begin());
    });

    // Erase dupes found by unique().
    lits.erase(lits.begin(), it.base());

    LongLitInfo info = analyzeLongLits(lits, max_len);

    vector<u32> litToOffsetVal;
    const size_t headerSize = ROUNDUP_16(sizeof(RoseLongLitTable));
    vector<u8> lit_blob = buildLits(lits, headerSize, litToOffsetVal);

    // Build caseful bloom filter and hash table.
    vector<u8> bloom_case;
    vector<RoseLongLitHashEntry> tab_case;
    if (info.caseful.num_literals) {
        bloom_case = makeBloomFilter(lits, max_len, false);
        tab_case = makeHashTable(lits, max_len, litToOffsetVal,
                                 info.caseful.hashed_positions, false);
    }

    // Build nocase bloom filter and hash table.
    vector<u8> bloom_nocase;
    vector<RoseLongLitHashEntry> tab_nocase;
    if (info.nocase.num_literals) {
        bloom_nocase = makeBloomFilter(lits, max_len, true);
        tab_nocase = makeHashTable(lits, max_len, litToOffsetVal,
                                   info.nocase.hashed_positions, true);
    }

    size_t wholeLitTabSize = ROUNDUP_16(byte_length(lit_blob));
    size_t htOffsetCase = headerSize + wholeLitTabSize;
    size_t htOffsetNocase = htOffsetCase + byte_length(tab_case);
    size_t bloomOffsetCase = htOffsetNocase + byte_length(tab_nocase);
    size_t bloomOffsetNocase = bloomOffsetCase + byte_length(bloom_case);

    size_t tabSize = ROUNDUP_16(bloomOffsetNocase + byte_length(bloom_nocase));

    // need to add +2 to both of these to allow space for the actual largest
    // value as well as handling the fact that we add one to the space when
    // storing out a position to allow zero to mean "no stream state value"
    u8 streamBitsCase = lg2(roundUpToPowerOfTwo(tab_case.size() + 2));
    u8 streamBitsNocase = lg2(roundUpToPowerOfTwo(tab_nocase.size() + 2));
    u32 tot_state_bytes = ROUNDUP_N(streamBitsCase + streamBitsNocase, 8) / 8;

    auto table = make_zeroed_bytecode_ptr<char>(tabSize, 16);
    assert(table); // otherwise would have thrown std::bad_alloc

    // Fill in the RoseLongLitTable header structure.
    RoseLongLitTable *header = (RoseLongLitTable *)(table.get());
    header->size = verify_u32(tabSize);
    header->maxLen = verify_u8(max_len); // u8 so doesn't matter; won't go > 255
    header->caseful.hashOffset = verify_u32(htOffsetCase);
    header->caseful.hashBits = lg2(tab_case.size());
    header->caseful.streamStateBits = streamBitsCase;
    header->caseful.bloomOffset = verify_u32(bloomOffsetCase);
    header->caseful.bloomBits = lg2(bloom_case.size() * 8);
    header->nocase.hashOffset = verify_u32(htOffsetNocase);
    header->nocase.hashBits = lg2(tab_nocase.size());
    header->nocase.streamStateBits = streamBitsNocase;
    header->nocase.bloomOffset = verify_u32(bloomOffsetNocase);
    header->nocase.bloomBits = lg2(bloom_nocase.size() * 8);
    assert(tot_state_bytes < sizeof(u64a));
    header->streamStateBytes = verify_u8(tot_state_bytes); // u8

    // Copy in the literal strings, hash tables and bloom filters,
    copy_bytes(table.get() + headerSize, lit_blob);
    copy_bytes(table.get() + htOffsetCase, tab_case);
    copy_bytes(table.get() + bloomOffsetCase, bloom_case);
    copy_bytes(table.get() + htOffsetNocase, tab_nocase);
    copy_bytes(table.get() + bloomOffsetNocase, bloom_nocase);

    DEBUG_PRINTF("built streaming table, size=%zu\n", tabSize);
    DEBUG_PRINTF("requires %zu bytes of history\n", max_len);
    DEBUG_PRINTF("requires %u bytes of stream state\n", tot_state_bytes);

    *historyRequired = max(*historyRequired, max_len);
    *longLitStreamStateRequired = tot_state_bytes;

    return blob.add(table);
}

} // namespace ue2
