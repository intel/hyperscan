/*
 * Copyright (c) 2016, Intel Corporation
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
#include "util/alloc.h"
#include "util/bitutils.h"
#include "util/verify_types.h"
#include "util/compile_context.h"

using namespace std;

namespace ue2 {

/** \brief Minimum size for a non-empty hash table. */
static constexpr u32 MIN_HASH_TABLE_SIZE = 4096;

struct LongLitModeInfo {
    u32 boundary = 0;    //!< One above the largest index for this mode.
    u32 positions = 0;   //!< Total number of string positions.
    u32 hashEntries = 0; //!< Number of hash table entries.
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
    u32 hashedPositionsCase = 0;
    u32 hashedPositionsNocase = 0;

    // Caseful boundary is the index of the first nocase literal, as we're
    // ordered (caseful, nocase).
    auto first_nocase = find_if(begin(lits), end(lits),
                [](const ue2_case_string &lit) { return lit.nocase; });
    info.caseful.boundary = verify_u32(distance(lits.begin(), first_nocase));

    // Nocase boundary is the size of the literal set.
    info.nocase.boundary = verify_u32(lits.size());

    for (const auto &lit : lits) {
        if (lit.nocase) {
            hashedPositionsNocase += lit.s.size() - max_len;
            info.nocase.positions += lit.s.size();
        } else {
            hashedPositionsCase += lit.s.size() - max_len;
            info.caseful.positions += lit.s.size();
        }
    }

    info.caseful.hashEntries = hashedPositionsCase
        ? roundUpToPowerOfTwo(max(MIN_HASH_TABLE_SIZE, hashedPositionsCase))
        : 0;
    info.nocase.hashEntries = hashedPositionsNocase
        ? roundUpToPowerOfTwo(max(MIN_HASH_TABLE_SIZE, hashedPositionsNocase))
        : 0;

    DEBUG_PRINTF("caseful:  boundary=%u, positions=%u, hashedPositions=%u, "
                 "hashEntries=%u\n",
                 info.caseful.boundary, info.caseful.positions,
                 hashedPositionsCase, info.caseful.hashEntries);
    DEBUG_PRINTF("nocase: boundary=%u, positions=%u, hashedPositions=%u, "
                 "hashEntries=%u\n",
                 info.nocase.boundary, info.nocase.positions,
                 hashedPositionsNocase, info.nocase.hashEntries);

    return info;
}

static
void fillHashes(const vector<ue2_case_string> &lits, size_t max_len,
                RoseLongLitHashEntry *tab, size_t numEntries, bool nocase,
                const map<u32, u32> &litToOffsetVal) {
    const u32 nbits = lg2(numEntries);
    map<u32, deque<pair<u32, u32>>> bucketToLitOffPairs;
    map<u32, u64a> bucketToBitfield;

    for (u32 lit_id = 0; lit_id < lits.size(); lit_id++) {
        const ue2_case_string &lit = lits[lit_id];
        if (nocase != lit.nocase) {
            continue;
        }
        for (u32 offset = 1; offset < lit.s.size() - max_len + 1; offset++) {
            const u8 *substr = (const u8 *)lit.s.c_str() + offset;
            u32 h = hashLongLiteral(substr, max_len, lit.nocase);
            u32 h_ent = h & ((1U << nbits) - 1);
            u32 h_low = (h >> nbits) & 63;
            bucketToLitOffPairs[h_ent].emplace_back(lit_id, offset);
            bucketToBitfield[h_ent] |= (1ULL << h_low);
        }
    }

    // this used to be a set<u32>, but a bitset is much much faster given that
    // we're using it only for membership testing.
    boost::dynamic_bitset<> filledBuckets(numEntries); // all zero by default.

    // sweep out bitfield entries and save the results swapped accordingly
    // also, anything with bitfield entries is put in filledBuckets
    for (const auto &m : bucketToBitfield) {
        const u32 &bucket = m.first;
        const u64a &contents = m.second;
        tab[bucket].bitfield = contents;
        filledBuckets.set(bucket);
    }

    // store out all our chains based on free values in our hash table.
    // find nearest free locations that are empty (there will always be more
    // entries than strings, at present)
    for (auto &m : bucketToLitOffPairs) {
        u32 bucket = m.first;
        deque<pair<u32, u32>> &d = m.second;

        // sort d by distance of the residual string (len minus our depth into
        // the string). We need to put the 'furthest back' string first...
        stable_sort(d.begin(), d.end(),
                    [](const pair<u32, u32> &a, const pair<u32, u32> &b) {
                        if (a.second != b.second) {
                            return a.second > b.second; /* longest is first */
                        }
                        return a.first < b.first;
                    });

        while (1) {
            // first time through is always at bucket, then we fill in links
            filledBuckets.set(bucket);
            RoseLongLitHashEntry *ent = &tab[bucket];
            u32 lit_id = d.front().first;
            u32 offset = d.front().second;

            ent->state = verify_u32(litToOffsetVal.at(lit_id) +
                                    offset + max_len);
            ent->link = (u32)LINK_INVALID;

            d.pop_front();
            if (d.empty()) {
                break;
            }
            // now, if there is another value
            // find a bucket for it and put in 'bucket' and repeat
            // all we really need to do is find something not in filledBuckets,
            // ideally something close to bucket
            // we search backward and forward from bucket, trying to stay as
            // close as possible.
            UNUSED bool found = false;
            int bucket_candidate = 0;
            for (u32 k = 1; k < numEntries * 2; k++) {
                bucket_candidate = bucket + (((k & 1) == 0)
                        ? (-(int)k / 2) : (k / 2));
                if (bucket_candidate < 0 ||
                    (size_t)bucket_candidate >= numEntries) {
                    continue;
                }
                if (!filledBuckets.test(bucket_candidate)) {
                    found = true;
                    break;
                }
            }

            assert(found);
            bucket = bucket_candidate;
            ent->link = bucket;
        }
    }
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

    // first assess the size and find our caseless threshold
    size_t headerSize = ROUNDUP_16(sizeof(RoseLongLitTable));

    size_t litTabOffset = headerSize;

    size_t litTabNumEntries = lits.size() + 1;
    size_t litTabSize = ROUNDUP_16(litTabNumEntries * sizeof(RoseLongLiteral));

    size_t wholeLitTabOffset = litTabOffset + litTabSize;
    size_t totalWholeLitTabSize =
        ROUNDUP_16(info.caseful.positions + info.nocase.positions);

    size_t htOffsetCase = wholeLitTabOffset + totalWholeLitTabSize;
    size_t htSizeCase = info.caseful.hashEntries * sizeof(RoseLongLitHashEntry);
    size_t htOffsetNocase = htOffsetCase + htSizeCase;
    size_t htSizeNocase =
        info.nocase.hashEntries * sizeof(RoseLongLitHashEntry);

    size_t tabSize = ROUNDUP_16(htOffsetNocase + htSizeNocase);

    // need to add +2 to both of these to allow space for the actual largest
    // value as well as handling the fact that we add one to the space when
    // storing out a position to allow zero to mean "no stream state value"
    u8 streamBitsCase = lg2(roundUpToPowerOfTwo(info.caseful.positions + 2));
    u8 streamBitsNocase = lg2(roundUpToPowerOfTwo(info.nocase.positions + 2));
    u32 tot_state_bytes = ROUNDUP_N(streamBitsCase + streamBitsNocase, 8) / 8;

    auto table = aligned_zmalloc_unique<char>(tabSize);
    assert(table); // otherwise would have thrown std::bad_alloc

    // then fill it in
    char *ptr = table.get();
    RoseLongLitTable *header = (RoseLongLitTable *)ptr;
    // fill in header
    header->maxLen = verify_u8(max_len); // u8 so doesn't matter; won't go > 255
    header->boundaryCase = info.caseful.boundary;
    header->hashOffsetCase = verify_u32(htOffsetCase);
    header->hashNBitsCase = lg2(info.caseful.hashEntries);
    header->streamStateBitsCase = streamBitsCase;
    header->boundaryNocase = info.nocase.boundary;
    header->hashOffsetNocase = verify_u32(htOffsetNocase);
    header->hashNBitsNocase = lg2(info.nocase.hashEntries);
    header->streamStateBitsNocase = streamBitsNocase;
    assert(tot_state_bytes < sizeof(u64a));
    header->streamStateBytes = verify_u8(tot_state_bytes); // u8

    ptr += headerSize;

    // now fill in the rest

    RoseLongLiteral *litTabPtr = (RoseLongLiteral *)ptr;
    ptr += litTabSize;

    map<u32, u32> litToOffsetVal;
    for (auto i = lits.begin(), e = lits.end(); i != e; ++i) {
        u32 entry = verify_u32(i - lits.begin());
        u32 offset = verify_u32(ptr - table.get());

        // point the table entry to the string location
        litTabPtr[entry].offset = offset;

        litToOffsetVal[entry] = offset;

        // copy the string into the string location
        const auto &s = i->s;
        memcpy(ptr, s.c_str(), s.size());

        ptr += s.size(); // and the string location
    }

    // fill in final lit table entry with current ptr (serves as end value)
    litTabPtr[lits.size()].offset = verify_u32(ptr - table.get());

    // fill hash tables
    ptr = table.get() + htOffsetCase;
    fillHashes(lits, max_len, (RoseLongLitHashEntry *)ptr,
               info.caseful.hashEntries, false, litToOffsetVal);
    ptr += htSizeCase;
    fillHashes(lits, max_len, (RoseLongLitHashEntry *)ptr,
               info.nocase.hashEntries, true, litToOffsetVal);
    ptr += htSizeNocase;

    assert(ptr <= table.get() + tabSize);

    DEBUG_PRINTF("built streaming table, size=%zu\n", tabSize);
    DEBUG_PRINTF("requires %zu bytes of history\n", max_len);
    DEBUG_PRINTF("requires %u bytes of stream state\n", tot_state_bytes);

    *historyRequired = max(*historyRequired, max_len);
    *longLitStreamStateRequired = tot_state_bytes;

    return blob.add(table.get(), tabSize, 16);
}

} // namespace ue2
