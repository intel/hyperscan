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

#include "fdr_internal.h"
#include "fdr_streaming_internal.h"
#include "fdr_compile_internal.h"
#include "hwlm/hwlm_build.h"
#include "util/alloc.h"
#include "util/bitutils.h"
#include "util/target_info.h"
#include "util/verify_types.h"

#include <algorithm>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <deque>
#include <set>

#include <boost/dynamic_bitset.hpp>

using namespace std;
using boost::dynamic_bitset;

namespace ue2 {

namespace {
struct LongLitOrder {
    bool operator()(const hwlmLiteral &i1, const hwlmLiteral &i2) const {
        if (i1.nocase != i2.nocase) {
            return i1.nocase < i2.nocase;
        } else {
            return i1.s < i2.s;
        }
    }
};
}

static
bool hwlmLitEqual(const hwlmLiteral &l1, const hwlmLiteral &l2) {
    return l1.s == l2.s && l1.nocase == l2.nocase;
}

static
u32 roundUpToPowerOfTwo(u32 x) {
    x -= 1;
    x |= (x >> 1);
    x |= (x >> 2);
    x |= (x >> 4);
    x |= (x >> 8);
    x |= (x >> 16);
    return x + 1;
}

/**
 * \brief Creates a long literals vector containing all literals of length > max_len.
 *
 * The last char of each literal is trimmed as we're not interested in full
 * matches, only partial matches.
 *
 * Literals are sorted (by caseful/caseless, then lexicographical order) and
 * made unique.
 *
 * The ID of each literal is set to its position in the vector.
 *
 * \return False if there aren't any long literals.
 */
static
bool setupLongLits(const vector<hwlmLiteral> &lits,
                   vector<hwlmLiteral> &long_lits, size_t max_len) {
    long_lits.reserve(lits.size());
    for (vector<hwlmLiteral>::const_iterator it = lits.begin();
         it != lits.end(); ++it) {
        if (it->s.length() > max_len) {
            hwlmLiteral tmp = *it; // copy
            tmp.s.erase(tmp.s.size() - 1, 1); // erase last char
            tmp.id = 0; // recalc later
            tmp.groups = 0; // filled in later by hash bucket(s)
            long_lits.push_back(tmp);
        }
    }

    if (long_lits.empty()) {
        return false;
    }

    // sort long_literals by caseful/caseless and in lexicographical order,
    // remove duplicates
    stable_sort(long_lits.begin(), long_lits.end(), LongLitOrder());
    vector<hwlmLiteral>::iterator new_end =
        unique(long_lits.begin(), long_lits.end(), hwlmLitEqual);
    long_lits.erase(new_end, long_lits.end());

    // fill in ids; not currently used
    for (vector<hwlmLiteral>::iterator i = long_lits.begin(),
                                       e = long_lits.end();
         i != e; ++i) {
        i->id = i - long_lits.begin();
    }
    return true;
}

// boundaries are the 'start' boundaries for each 'mode'
// so boundary[CASEFUL] is the index one above the largest caseful index
// positions[CASEFUL] is the # of positions in caseful strings (stream)
// hashedPositions[CASEFUL] is the # of positions in caseful strings
//                          (not returned - a temporary)
// hashEntries[CASEFUL] is the # of positions hashed for caseful strings
//                    (rounded up to the nearest power of two)
static
void analyzeLits(const vector<hwlmLiteral> &long_lits, size_t max_len,
                 u32 *boundaries, u32 *positions, u32 *hashEntries) {
    u32 hashedPositions[MAX_MODES];

    for (u32 m = CASEFUL; m < MAX_MODES; ++m) {
        boundaries[m] = verify_u32(long_lits.size());
        positions[m] = 0;
        hashedPositions[m] = 0;
    }

    for (vector<hwlmLiteral>::const_iterator i = long_lits.begin(),
                                             e = long_lits.end();
         i != e; ++i) {
        if (i->nocase) {
            boundaries[CASEFUL] = verify_u32(i - long_lits.begin());
            break;
        }
    }

    for (vector<hwlmLiteral>::const_iterator i = long_lits.begin(),
                                             e = long_lits.end();
         i != e; ++i) {
        MODES m = i->nocase ? CASELESS : CASEFUL;
        for (u32 j = 1; j < i->s.size() - max_len + 1; j++) {
            hashedPositions[m]++;
        }
        positions[m] += i->s.size();
    }

    for (u32 m = CASEFUL; m < MAX_MODES; m++) {
        hashEntries[m] = hashedPositions[m]
                ? roundUpToPowerOfTwo(MAX(4096, hashedPositions[m]))
                : 0;
    }

#ifdef DEBUG_COMPILE
    printf("analyzeLits:\n");
    for (MODES m = CASEFUL; m < MAX_MODES; m++) {
        printf("mode %s boundary %d positions %d hashedPositions %d "
               "hashEntries %d\n",
               (m == CASEFUL) ? "caseful" : "caseless", boundaries[m],
               positions[m], hashedPositions[m], hashEntries[m]);
    }
    printf("\n");
#endif
}

static
u32 hashLit(const hwlmLiteral &l, u32 offset, size_t max_len, MODES m) {
    return streaming_hash((const u8 *)l.s.c_str() + offset, max_len, m);
}

// sort by 'distance from start'
namespace {
struct OffsetIDFromEndOrder {
    const vector<hwlmLiteral> &lits; // not currently used
    explicit OffsetIDFromEndOrder(const vector<hwlmLiteral> &lits_in)
        : lits(lits_in) {}
    bool operator()(const pair<u32, u32> &i1, const pair<u32, u32> &i2) const {
        if (i1.second != i2.second) {
            // longest is 'first', so > not <
            return i1.second > i2.second;
        }
        return i1.first < i2.first;
    }
};
}

static
void fillHashes(const vector<hwlmLiteral> &long_lits, size_t max_len,
                FDRSHashEntry *tab, size_t numEntries, MODES m,
                map<u32, u32> &litToOffsetVal) {
    const u32 nbits = lg2(numEntries);
    map<u32, deque<pair<u32, u32> > > bucketToLitOffPairs;
    map<u32, u64a> bucketToBitfield;

    for (vector<hwlmLiteral>::const_iterator i = long_lits.begin(),
                                             e = long_lits.end();
         i != e; ++i) {
        const hwlmLiteral &l = *i;
        if ((m == CASELESS) != i->nocase) {
            continue;
        }
        for (u32 j = 1; j < i->s.size() - max_len + 1; j++) {
            u32 h = hashLit(l, j, max_len, m);
            u32 h_ent = h & ((1U << nbits) - 1);
            u32 h_low = (h >> nbits) & 63;
            bucketToLitOffPairs[h_ent].push_back(make_pair(i->id, j));
            bucketToBitfield[h_ent] |= (1ULL << h_low);
        }
    }

    // this used to be a set<u32>, but a bitset is much much faster given that
    // we're using it only for membership testing.
    dynamic_bitset<> filledBuckets(numEntries); // all bits zero by default.

    // sweep out bitfield entries and save the results swapped accordingly
    // also, anything with bitfield entries is put in filledBuckets
    for (map<u32, u64a>::const_iterator i = bucketToBitfield.begin(),
                                        e = bucketToBitfield.end();
         i != e; ++i) {
        u32 bucket = i->first;
        u64a contents = i->second;
        tab[bucket].bitfield = contents;
        filledBuckets.set(bucket);
    }

    // store out all our chains based on free values in our hash table.
    // find nearest free locations that are empty (there will always be more
    // entries than strings, at present)
    for (map<u32, deque<pair<u32, u32> > >::iterator
             i = bucketToLitOffPairs.begin(),
             e = bucketToLitOffPairs.end();
         i != e; ++i) {
        u32 bucket = i->first;
        deque<pair<u32, u32> > &d = i->second;

        // sort d by distance of the residual string (len minus our depth into
        // the string). We need to put the 'furthest back' string first...
        stable_sort(d.begin(), d.end(), OffsetIDFromEndOrder(long_lits));

        while (1) {
            // first time through is always at bucket, then we fill in links
            filledBuckets.set(bucket);
            FDRSHashEntry *ent = &tab[bucket];
            u32 lit_id = d.front().first;
            u32 offset = d.front().second;

            ent->state = verify_u32(litToOffsetVal[lit_id] + offset + max_len);
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

static
size_t maxMaskLen(const vector<hwlmLiteral> &lits) {
    size_t rv = 0;
    vector<hwlmLiteral>::const_iterator it, ite;
    for (it = lits.begin(), ite = lits.end(); it != ite; ++it) {
        rv = max(rv, it->msk.size());
    }
    return rv;
}

pair<u8 *, size_t>
fdrBuildTableStreaming(const vector<hwlmLiteral> &lits,
                       hwlmStreamingControl *stream_control) {
    // refuse to compile if we are forced to have smaller than minimum
    // history required for long-literal support, full stop
    // otherwise, choose the maximum of the preferred history quantity
    // (currently a fairly extravagant 32) or the already used history
    // quantity - subject to the limitation of stream_control->history_max

    const size_t MIN_HISTORY_REQUIRED = 32;

    if (MIN_HISTORY_REQUIRED > stream_control->history_max) {
        throw std::logic_error("Cannot set history to minimum history required");
    }

    size_t max_len =
        MIN(stream_control->history_max,
            MAX(MIN_HISTORY_REQUIRED, stream_control->history_min));
    assert(max_len >= MIN_HISTORY_REQUIRED);
    size_t max_mask_len = maxMaskLen(lits);

    vector<hwlmLiteral> long_lits;
    if (!setupLongLits(lits, long_lits, max_len) || false) {
        // "Don't need to do anything" path, not really a fail
        DEBUG_PRINTF("Streaming literal path produces no table\n");

        // we want enough history to manage the longest literal and the longest
        // mask.
        stream_control->literal_history_required =
                    max(maxLen(lits), max_mask_len) - 1;
        stream_control->literal_stream_state_required = 0;
        return make_pair(nullptr, size_t{0});
    }

    // Ensure that we have enough room for the longest mask.
    if (max_mask_len) {
        max_len = max(max_len, max_mask_len - 1);
    }

    u32 boundary[MAX_MODES];
    u32 positions[MAX_MODES];
    u32 hashEntries[MAX_MODES];

    analyzeLits(long_lits, max_len, boundary, positions, hashEntries);

    // first assess the size and find our caseless threshold
    size_t headerSize = ROUNDUP_16(sizeof(FDRSTableHeader));

    size_t litTabOffset = headerSize;

    size_t litTabNumEntries = long_lits.size() + 1;
    size_t litTabSize = ROUNDUP_16(litTabNumEntries * sizeof(FDRSLiteral));

    size_t wholeLitTabOffset = litTabOffset + litTabSize;
    size_t totalWholeLitTabSize = ROUNDUP_16(positions[CASEFUL] +
                                             positions[CASELESS]);

    size_t htOffset[MAX_MODES];
    size_t htSize[MAX_MODES];

    htOffset[CASEFUL] = wholeLitTabOffset + totalWholeLitTabSize;
    htSize[CASEFUL] = hashEntries[CASEFUL] * sizeof(FDRSHashEntry);
    htOffset[CASELESS] = htOffset[CASEFUL] + htSize[CASEFUL];
    htSize[CASELESS] = hashEntries[CASELESS] * sizeof(FDRSHashEntry);

    size_t tabSize = ROUNDUP_16(htOffset[CASELESS] + htSize[CASELESS]);

    // need to add +2 to both of these to allow space for the actual largest
    // value as well as handling the fact that we add one to the space when
    // storing out a position to allow zero to mean "no stream state value"
    u8 streamBits[MAX_MODES];
    streamBits[CASEFUL] = lg2(roundUpToPowerOfTwo(positions[CASEFUL] + 2));
    streamBits[CASELESS] = lg2(roundUpToPowerOfTwo(positions[CASELESS] + 2));
    u32 tot_state_bytes = (streamBits[CASEFUL] + streamBits[CASELESS] + 7) / 8;

    u8 * secondaryTable = (u8 *)aligned_zmalloc(tabSize);
    assert(secondaryTable); // otherwise would have thrown std::bad_alloc

    // then fill it in
    u8 * ptr = secondaryTable;
    FDRSTableHeader * header = (FDRSTableHeader *)ptr;
    // fill in header
    header->pseudoEngineID = (u32)0xffffffff;
    header->N = verify_u8(max_len); // u8 so doesn't matter; won't go > 255
    for (u32 m = CASEFUL; m < MAX_MODES; ++m) {
        header->boundary[m] = boundary[m];
        header->hashOffset[m] = verify_u32(htOffset[m]);
        header->hashNBits[m] = lg2(hashEntries[m]);
        header->streamStateBits[m] = streamBits[m];
    }
    assert(tot_state_bytes < sizeof(u64a));
    header->streamStateBytes = verify_u8(tot_state_bytes); // u8

    ptr += headerSize;

    // now fill in the rest

    FDRSLiteral * litTabPtr = (FDRSLiteral *)ptr;
    ptr += litTabSize;

    map<u32, u32> litToOffsetVal;
    for (vector<hwlmLiteral>::const_iterator i = long_lits.begin(),
                                             e = long_lits.end();
         i != e; ++i) {
        u32 entry = verify_u32(i - long_lits.begin());
        u32 offset = verify_u32(ptr - secondaryTable);

        // point the table entry to the string location
        litTabPtr[entry].offset = offset;

        litToOffsetVal[entry] = offset;

        // copy the string into the string location
        memcpy(ptr, i->s.c_str(), i->s.size());

        ptr += i->s.size(); // and the string location
    }

    // fill in final lit table entry with current ptr (serves as end value)
    litTabPtr[long_lits.size()].offset = verify_u32(ptr - secondaryTable);

    // fill hash tables
    ptr = secondaryTable + htOffset[CASEFUL];
    for (u32 m = CASEFUL; m < MAX_MODES; ++m) {
        fillHashes(long_lits, max_len, (FDRSHashEntry *)ptr, hashEntries[m],
                   (MODES)m, litToOffsetVal);
        ptr += htSize[m];
    }

    // tell the world what we did
    stream_control->literal_history_required = max_len;
    stream_control->literal_stream_state_required = tot_state_bytes;
    return make_pair(secondaryTable, tabSize);
}

} // namespace ue2
