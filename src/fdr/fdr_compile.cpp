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

/** \file
 * \brief FDR literal matcher: build API.
 */

#include "fdr_compile.h"

#include "fdr_internal.h"
#include "fdr_confirm.h"
#include "fdr_compile_internal.h"
#include "fdr_engine_description.h"
#include "teddy_compile.h"
#include "teddy_engine_description.h"
#include "grey.h"
#include "ue2common.h"
#include "hwlm/hwlm_build.h"
#include "util/compare.h"
#include "util/container.h"
#include "util/dump_mask.h"
#include "util/make_unique.h"
#include "util/math.h"
#include "util/noncopyable.h"
#include "util/target_info.h"
#include "util/ue2string.h"
#include "util/verify_types.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <boost/multi_array.hpp>

using namespace std;

namespace ue2 {

namespace {

class FDRCompiler : noncopyable {
private:
    const FDREngineDescription &eng;
    const Grey &grey;
    vector<u8> tab;
    vector<hwlmLiteral> lits;
    map<BucketIndex, std::vector<LiteralIndex> > bucketToLits;
    bool make_small;

    u8 *tabIndexToMask(u32 indexInTable);
#ifdef DEBUG
    void dumpMasks(const u8 *defaultMask);
#endif
    void setupTab();
    bytecode_ptr<FDR> setupFDR();
    void createInitialState(FDR *fdr);

public:
    FDRCompiler(vector<hwlmLiteral> lits_in,
                map<BucketIndex, std::vector<LiteralIndex>> bucketToLits_in,
                const FDREngineDescription &eng_in,
                bool make_small_in, const Grey &grey_in)
        : eng(eng_in), grey(grey_in), tab(eng_in.getTabSizeBytes()),
          lits(move(lits_in)), bucketToLits(move(bucketToLits_in)),
          make_small(make_small_in) {}

    bytecode_ptr<FDR> build();
};

u8 *FDRCompiler::tabIndexToMask(u32 indexInTable) {
    assert(indexInTable < tab.size());
    return &tab[0] + (indexInTable * (eng.getSchemeWidth() / 8));
}

static
void setbit(u8 *msk, u32 bit) {
    msk[bit / 8] |= 1U << (bit % 8);
}

static
void clearbit(u8 *msk, u32 bit) {
    msk[bit / 8] &= ~(1U << (bit % 8));
}

static
void andMask(u8 *dest, const u8 *a, const u8 *b, u32 num_bytes) {
    for (u32 i = 0; i < num_bytes; i++) {
        dest[i] = a[i] & b[i];
    }
}

void FDRCompiler::createInitialState(FDR *fdr) {
    u8 *start = (u8 *)&fdr->start;

    /* initial state should to be 1 in each slot in the bucket up to bucket
     * minlen - 1, and 0 thereafter */
    for (BucketIndex b = 0; b < eng.getNumBuckets(); b++) {
        // Find the minimum length for the literals in this bucket.
        const vector<LiteralIndex> &bucket_lits = bucketToLits[b];
        u32 min_len = ~0U;
        for (const LiteralIndex &lit_idx : bucket_lits) {
            min_len = min(min_len, verify_u32(lits[lit_idx].s.length()));
        }

        DEBUG_PRINTF("bucket %u has min_len=%u\n", b, min_len);
        assert(min_len);

        for (PositionInBucket i = 0; i < eng.getBucketWidth(b); i++) {
            if (i < min_len - 1) {
                setbit(start, eng.getSchemeBit(b, i));
            }
        }
    }
}

/**
 * \brief Lay out FDR structures in bytecode.
 *
 * Note that each major structure (header, table, confirm, flood control) is
 * cacheline-aligned.
 */
bytecode_ptr<FDR> FDRCompiler::setupFDR() {
    auto floodTable = setupFDRFloodControl(lits, eng, grey);
    auto confirmTable = setupFullConfs(lits, eng, bucketToLits, make_small);

    size_t headerSize = sizeof(FDR);
    size_t tabSize = eng.getTabSizeBytes();

    // Note: we place each major structure here on a cacheline boundary.
    size_t size = ROUNDUP_CL(headerSize) + ROUNDUP_CL(tabSize) +
                  ROUNDUP_CL(confirmTable.size()) + floodTable.size();

    DEBUG_PRINTF("sizes base=%zu tabSize=%zu confirm=%zu floodControl=%zu "
                 "total=%zu\n",
                 headerSize, tabSize, confirmTable.size(), floodTable.size(),
                 size);

    auto fdr = make_zeroed_bytecode_ptr<FDR>(size, 64);
    assert(fdr); // otherwise would have thrown std::bad_alloc

    u8 *fdr_base = (u8 *)fdr.get();

    // Write header.
    fdr->size = size;
    fdr->engineID = eng.getID();
    fdr->maxStringLen = verify_u32(maxLen(lits));
    fdr->numStrings = verify_u32(lits.size());
    assert(eng.bits > 8 && eng.bits < 16); // we allow domains 9 to 15 only
    fdr->domain = eng.bits;
    fdr->domainMask = (1 << eng.bits) - 1;
    fdr->tabSize = tabSize;
    fdr->stride = eng.stride;
    createInitialState(fdr.get());

    // Write table.
    u8 *ptr = fdr_base + ROUNDUP_CL(sizeof(FDR));
    assert(ISALIGNED_CL(ptr));
    copy(tab.begin(), tab.end(), ptr);
    ptr += ROUNDUP_CL(tabSize);

    // Write confirm structures.
    assert(ISALIGNED_CL(ptr));
    fdr->confOffset = verify_u32(ptr - fdr_base);
    memcpy(ptr, confirmTable.get(), confirmTable.size());
    ptr += ROUNDUP_CL(confirmTable.size());

    // Write flood control structures.
    assert(ISALIGNED_CL(ptr));
    fdr->floodOffset = verify_u32(ptr - fdr_base);
    memcpy(ptr, floodTable.get(), floodTable.size());
    ptr += floodTable.size(); // last write, no need to round up

    return fdr;
}

//#define DEBUG_ASSIGNMENT

/**
 * Utility class for computing:
 *
 *    score(count, len) = pow(count, 1.05) * pow(len, -3)
 *
 * Calling pow() is expensive. This is mitigated by using pre-computed LUTs for
 * small inputs and a cache for larger ones.
 */
class Scorer {
    unordered_map<u32, double> count_factor_cache;

    // LUT: pow(count, 1.05) for small values of count.
    static const array<double, 100> count_lut;

    double count_factor(u32 count) {
        if (count < count_lut.size()) {
            return count_lut[count];
        }

        auto it = count_factor_cache.find(count);
        if (it != count_factor_cache.end()) {
            return it->second;
        }
        double r = our_pow(count, 1.05);
        count_factor_cache.emplace(count, r);
        return r;
    }

    // LUT: pow(len, -3) for len in range [0,8].
    static const array<double, 9> len_lut;

    double len_factor(u32 len) {
        assert(len <= len_lut.size());
        return len_lut[len];
    }

public:
    double operator()(u32 len, u32 count) {
        if (len == 0) {
            return numeric_limits<double>::max();
        }
        return count_factor(count) * len_factor(len);
    }
};

const array<double, 100> Scorer::count_lut{{
    pow(0, 1.05),  pow(1, 1.05),  pow(2, 1.05),  pow(3, 1.05),  pow(4, 1.05),
    pow(5, 1.05),  pow(6, 1.05),  pow(7, 1.05),  pow(8, 1.05),  pow(9, 1.05),
    pow(10, 1.05), pow(11, 1.05), pow(12, 1.05), pow(13, 1.05), pow(14, 1.05),
    pow(15, 1.05), pow(16, 1.05), pow(17, 1.05), pow(18, 1.05), pow(19, 1.05),
    pow(20, 1.05), pow(21, 1.05), pow(22, 1.05), pow(23, 1.05), pow(24, 1.05),
    pow(25, 1.05), pow(26, 1.05), pow(27, 1.05), pow(28, 1.05), pow(29, 1.05),
    pow(30, 1.05), pow(31, 1.05), pow(32, 1.05), pow(33, 1.05), pow(34, 1.05),
    pow(35, 1.05), pow(36, 1.05), pow(37, 1.05), pow(38, 1.05), pow(39, 1.05),
    pow(40, 1.05), pow(41, 1.05), pow(42, 1.05), pow(43, 1.05), pow(44, 1.05),
    pow(45, 1.05), pow(46, 1.05), pow(47, 1.05), pow(48, 1.05), pow(49, 1.05),
    pow(50, 1.05), pow(51, 1.05), pow(52, 1.05), pow(53, 1.05), pow(54, 1.05),
    pow(55, 1.05), pow(56, 1.05), pow(57, 1.05), pow(58, 1.05), pow(59, 1.05),
    pow(60, 1.05), pow(61, 1.05), pow(62, 1.05), pow(63, 1.05), pow(64, 1.05),
    pow(65, 1.05), pow(66, 1.05), pow(67, 1.05), pow(68, 1.05), pow(69, 1.05),
    pow(70, 1.05), pow(71, 1.05), pow(72, 1.05), pow(73, 1.05), pow(74, 1.05),
    pow(75, 1.05), pow(76, 1.05), pow(77, 1.05), pow(78, 1.05), pow(79, 1.05),
    pow(80, 1.05), pow(81, 1.05), pow(82, 1.05), pow(83, 1.05), pow(84, 1.05),
    pow(85, 1.05), pow(86, 1.05), pow(87, 1.05), pow(88, 1.05), pow(89, 1.05),
    pow(90, 1.05), pow(91, 1.05), pow(92, 1.05), pow(93, 1.05), pow(94, 1.05),
    pow(95, 1.05), pow(96, 1.05), pow(97, 1.05), pow(98, 1.05), pow(99, 1.05),
}};

const array<double, 9> Scorer::len_lut{{
    0, pow(1, -3.0), pow(2, -3.0), pow(3, -3.0), pow(4, -3.0),
       pow(5, -3.0), pow(6, -3.0), pow(7, -3.0), pow(8, -3.0)}};

/**
 * Returns true if the two given literals should be placed in the same chunk as
 * they are identical except for a difference in caselessness.
 */
static
bool isEquivLit(const hwlmLiteral &a, const hwlmLiteral &b,
                const hwlmLiteral *last_nocase_lit) {
    const size_t a_len = a.s.size();
    const size_t b_len = b.s.size();

    if (a_len != b_len) {
        return false;
    }

    bool nocase = last_nocase_lit && a_len == last_nocase_lit->s.size() &&
                  !cmp(a.s.c_str(), last_nocase_lit->s.c_str(), a_len, true);
    return !cmp(a.s.c_str(), b.s.c_str(), a.s.size(), nocase);
}

struct Chunk {
    Chunk(u32 first_id_in, u32 count_in, u32 length_in)
        : first_id(first_id_in), count(count_in), length(length_in) {}
    u32 first_id; //!< first id in this chunk
    u32 count;    //!< how many are in this chunk
    u32 length;   //!< how long things in the chunk are
};

static
vector<Chunk> assignChunks(const vector<hwlmLiteral> &lits,
                           const map<u32, u32> &lenCounts) {
    const u32 CHUNK_MAX = 512;
    const u32 MAX_CONSIDERED_LENGTH = 16;

    // TODO: detailed early stage literal analysis for v. small cases (actually
    // look at lits) yes - after we factor this out and merge in the Teddy
    // style of building we can look at this, although the teddy merge
    // modelling is quite different. It's still probably adaptable to some
    // extent for this class of problem.

    vector<Chunk> chunks;
    chunks.reserve(CHUNK_MAX);

    const u32 maxPerChunk = lits.size() /
            (CHUNK_MAX - MIN(MAX_CONSIDERED_LENGTH, lenCounts.size())) + 1;

    u32 currentSize = 0;
    u32 chunkStartID = 0;
    const hwlmLiteral *last_nocase_lit = nullptr;

    for (u32 i = 0; i < lits.size() && chunks.size() < CHUNK_MAX - 1; i++) {
        const auto &lit = lits[i];

        DEBUG_PRINTF("i=%u, lit=%s%s\n", i, escapeString(lit.s).c_str(),
                      lit.nocase ? " (nocase)" : "");

        // If this literal is identical to the last one (aside from differences
        // in caselessness), keep going even if we will "overfill" a chunk; we
        // don't want to split identical literals into different buckets.
        if (i != 0 && isEquivLit(lit, lits[i - 1], last_nocase_lit)) {
            DEBUG_PRINTF("identical lit\n");
            goto next_literal;
        }

        if ((currentSize < MAX_CONSIDERED_LENGTH &&
             (lit.s.size() != currentSize)) ||
            (currentSize != 1 && ((i - chunkStartID) >= maxPerChunk))) {
            currentSize = lit.s.size();
            if (!chunks.empty()) {
                chunks.back().count = i - chunkStartID;
            }
            chunkStartID = i;
            chunks.emplace_back(i, 0, currentSize);
        }
next_literal:
        if (lit.nocase) {
            last_nocase_lit = &lit;
        }
    }

    assert(!chunks.empty());
    chunks.back().count = lits.size() - chunkStartID;
    // close off chunks with an empty row
    chunks.emplace_back(lits.size(), 0, 0);

#ifdef DEBUG_ASSIGNMENT
    for (size_t j = 0; j < chunks.size(); j++) {
        const auto &chunk = chunks[j];
        printf("chunk %zu first_id=%u count=%u length=%u\n", j, chunk.first_id,
               chunk.count, chunk.length);
    }
#endif

    DEBUG_PRINTF("built %zu chunks (%zu lits)\n", chunks.size(), lits.size());
    assert(chunks.size() <= CHUNK_MAX);
    return chunks;
}

static
map<BucketIndex, vector<LiteralIndex>> assignStringsToBuckets(
                                    vector<hwlmLiteral> &lits,
                                    const FDREngineDescription &eng) {
    const double MAX_SCORE = numeric_limits<double>::max();

    assert(!lits.empty()); // Shouldn't be called with no literals.

    // Count the number of literals for each length.
    map<u32, u32> lenCounts;
    for (const auto &lit : lits) {
        lenCounts[lit.s.size()]++;
    }

#ifdef DEBUG_ASSIGNMENT
    for (const auto &m : lenCounts) {
        printf("l<%u>:%u ", m.first, m.second);
    }
    printf("\n");
#endif

    // Sort literals by literal length. If tied on length, use lexicographic
    // ordering (of the reversed literals).
    stable_sort(lits.begin(), lits.end(),
                [](const hwlmLiteral &a, const hwlmLiteral &b) {
                    if (a.s.size() != b.s.size()) {
                        return a.s.size() < b.s.size();
                    }
                    auto p = mismatch(a.s.rbegin(), a.s.rend(), b.s.rbegin());
                    if (p.first != a.s.rend()) {
                        return *p.first < *p.second;
                    }
                    // Sort caseless variants first.
                    return a.nocase > b.nocase;
                });

    vector<Chunk> chunks = assignChunks(lits, lenCounts);

    const u32 numChunks = chunks.size();
    const u32 numBuckets = eng.getNumBuckets();

    // 2D array of (score, chunk index) pairs, indexed by
    // [chunk_index][bucket_index].
    boost::multi_array<pair<double, u32>, 2> t(
        boost::extents[numChunks][numBuckets]);

    Scorer scorer;

    for (u32 j = 0; j < numChunks; j++) {
        u32 cnt = 0;
        for (u32 k = j; k < numChunks; ++k) {
            cnt += chunks[k].count;
        }
        t[j][0] = {scorer(chunks[j].length, cnt), 0};
    }

    for (u32 i = 1; i < numBuckets; i++) {
        for (u32 j = 0; j < numChunks - 1; j++) { // don't do last, empty row
            pair<double, u32> best = {MAX_SCORE, 0};
            u32 cnt = chunks[j].count;
            for (u32 k = j + 1; k < numChunks - 1; k++) {
                auto score = scorer(chunks[j].length, cnt);
                if (score > best.first) {
                    break; // now worse locally than our best score, give up
                }
                score += t[k][i-1].first;
                if (score < best.first) {
                    best = {score, k};
                }
                cnt += chunks[k].count;
            }
            t[j][i] = best;
        }
        t[numChunks - 1][i] = {0,0}; // fill in empty final row for next iter
    }

#ifdef DEBUG_ASSIGNMENT
    for (u32 j = 0; j < numChunks; j++) {
        printf("%03u: ", j);
        for (u32 i = 0; i < numBuckets; i++) {
            const auto &v = t[j][i];
            printf("<%0.3f,%3d> ", v.first, v.second);
        }
        printf("\n");
    }
#endif

    // our best score is in t[0][N_BUCKETS-1] and we can follow the links
    // to find where our buckets should start and what goes into them
    vector<vector<LiteralIndex>> buckets;
    for (u32 i = 0, n = numBuckets; n && (i != numChunks - 1); n--) {
        u32 j = t[i][n - 1].second;
        if (j == 0) {
            j = numChunks - 1;
        }

        // put chunks between i - j into bucket (numBuckets - n).
        u32 first_id = chunks[i].first_id;
        u32 last_id = chunks[j].first_id;
        assert(first_id < last_id);
        UNUSED const auto &first_lit = lits[first_id];
        UNUSED const auto &last_lit = lits[last_id - 1];
        DEBUG_PRINTF("placing [%u-%u) in one bucket (%u lits, len %zu-%zu, "
                     "score %0.4f)\n",
                     first_id, last_id, last_id - first_id,
                     first_lit.s.length(), last_lit.s.length(),
                     scorer(first_lit.s.length(), last_id - first_id));

        vector<LiteralIndex> litIds;
        u32 cnt = last_id - first_id;
        // long literals first for included literals checking
        for (u32 k = 0; k < cnt; k++) {
            litIds.push_back(last_id - k - 1);
        }

        i = j;
        buckets.push_back(litIds);
    }

    // reverse bucket id, longer literals come first
    map<BucketIndex, vector<LiteralIndex>> bucketToLits;
    size_t bucketCnt = buckets.size();
    for (size_t i = 0; i < bucketCnt; i++) {
        bucketToLits.emplace(bucketCnt - i - 1, move(buckets[i]));
    }

    return bucketToLits;
}

#ifdef DEBUG
void FDRCompiler::dumpMasks(const u8 *defaultMask) {
    const size_t width = eng.getSchemeWidth();
    printf("default mask: %s\n", dumpMask(defaultMask, width).c_str());
    for (u32 i = 0; i < eng.getNumTableEntries(); i++) {
        u8 *m = tabIndexToMask(i);
        if (memcmp(m, defaultMask, width / 8)) {
            printf("tab %04x: %s\n", i, dumpMask(m, width).c_str());
        }
    }
}
#endif

static
bool getMultiEntriesAtPosition(const FDREngineDescription &eng,
                               const vector<LiteralIndex> &vl,
                               const vector<hwlmLiteral> &lits,
                               SuffixPositionInString pos,
                               map<u32, unordered_set<u32>> &m2) {
    assert(eng.bits < 32);

    u32 distance = 0;
    if (eng.bits <= 8) {
        distance = 1;
    } else if (eng.bits <= 16) {
        distance = 2;
    } else {
        distance = 4;
    }

    for (auto i = vl.begin(), e = vl.end(); i != e; ++i) {
        if (e - i > 5) {
            __builtin_prefetch(&lits[*(i + 5)]);
        }
        const hwlmLiteral &lit = lits[*i];
        const size_t sz = lit.s.size();
        u32 mask = 0;
        u32 dontCares = 0;
        for (u32 cnt = 0; cnt < distance; cnt++) {
            int newPos = pos - cnt;
            u8 dontCareByte = 0x0;
            u8 maskByte = 0x0;
            if (newPos < 0 || ((u32)newPos >= sz)) {
                dontCareByte = 0xff;
            } else {
                u8 c = lit.s[sz - newPos - 1];
                maskByte = c;
                u32 remainder = eng.bits - cnt * 8;
                assert(remainder != 0);
                if (remainder < 8) {
                    u8 cmask = (1U << remainder) - 1;
                    maskByte &= cmask;
                    dontCareByte |= ~cmask;
                }
                if (lit.nocase && ourisalpha(c)) {
                    maskByte &= 0xdf;
                    dontCareByte |= 0x20;
                }
            }
            u32 loc =  cnt * 8;
            mask |= maskByte << loc;
            dontCares |= dontCareByte << loc;
        }

        // truncate m and dc down to nBits
        mask &= (1U << eng.bits) - 1;
        dontCares &= (1U << eng.bits) - 1;
        if (dontCares == ((1U << eng.bits) - 1)) {
            return true;
        }
        m2[dontCares].insert(mask);
    }
    return false;
}

void FDRCompiler::setupTab() {
    const size_t mask_size = eng.getSchemeWidth() / 8;
    assert(mask_size);

    vector<u8> defaultMask(mask_size, 0xff);
    for (u32 i = 0; i < eng.getNumTableEntries(); i++) {
        memcpy(tabIndexToMask(i), &defaultMask[0], mask_size);
    }

    for (BucketIndex b = 0; b < eng.getNumBuckets(); b++) {
        const vector<LiteralIndex> &vl = bucketToLits[b];
        SuffixPositionInString pLimit = eng.getBucketWidth(b);
        for (SuffixPositionInString pos = 0; pos < pLimit; pos++) {
            u32 bit = eng.getSchemeBit(b, pos);
            map<u32, unordered_set<u32>> m2;
            bool done = getMultiEntriesAtPosition(eng, vl, lits, pos, m2);
            if (done) {
                clearbit(&defaultMask[0], bit);
                continue;
            }
            for (const auto &elem : m2) {
                u32 dc = elem.first;
                const unordered_set<u32> &mskSet = elem.second;
                u32 v = ~dc;
                do {
                    u32 b2 = v & dc;
                    for (const u32 &mskVal : mskSet) {
                        u32 val = (mskVal & ~dc) | b2;
                        clearbit(tabIndexToMask(val), bit);
                    }
                    v = (v + (dc & -dc)) | ~dc;
                } while (v != ~dc);
            }
        }
    }

    for (u32 i = 0; i < eng.getNumTableEntries(); i++) {
        u8 *m = tabIndexToMask(i);
        andMask(m, m, &defaultMask[0], mask_size);
    }
#ifdef DEBUG
    dumpMasks(&defaultMask[0]);
#endif
}

bytecode_ptr<FDR> FDRCompiler::build() {
    setupTab();
    return setupFDR();
}

static
bool isSuffix(const hwlmLiteral &lit1, const hwlmLiteral &lit2) {
    const auto &s1 = lit1.s;
    const auto &s2 = lit2.s;
    size_t len1 = s1.length();
    size_t len2 = s2.length();
    assert(len1 >= len2);

    if (lit1.nocase || lit2.nocase) {
        return equal(s2.begin(), s2.end(), s1.begin() + len1 - len2,
            [](char a, char b) { return mytoupper(a) == mytoupper(b); });
    } else {
        return equal(s2.begin(), s2.end(), s1.begin() + len1 - len2);
    }
}

/*
 * if lit2 is a suffix of lit1 but the case sensitivity, groups or mask info
 * of lit2 is a subset of lit1, then lit1 can't squash lit2 and lit2 can
 * possibly match when lit1 matches. In this case, we can't do bucket
 * squashing. e.g. AAA(no case) in bucket 0, AA(no case) and aa in bucket 1,
 * we can't squash bucket 1 if we have input like "aaa" as aa can also match.
 */
static
bool includedCheck(const hwlmLiteral &lit1, const hwlmLiteral &lit2) {
    /* lit1 is caseless and lit2 is case sensitive */
    if ((lit1.nocase && !lit2.nocase)) {
        return true;
    }

    /* lit2's group is a subset of lit1 */
    if (lit1.groups != lit2.groups &&
        (lit2.groups == (lit1.groups & lit2.groups))) {
        return true;
    }

    /* TODO: narrow down cases for mask check */
    if (lit1.cmp != lit2.cmp || lit1.msk != lit2.msk) {
        return true;
    }

    return false;
}

/*
 * if lit2 is an included literal of both lit0 and lit1, then lit0 and lit1
 * shouldn't match at the same offset, otherwise we give up squashing for lit1.
 * e.g. lit0:AAA(no case), lit1:aa, lit2:A(no case). We can have duplicate
 * matches for input "aaa" if lit0 and lit1 both squash lit2.
 */
static
bool checkParentLit(
            const vector<hwlmLiteral> &lits, u32 pos1,
            const unordered_set<u32> &parent_map,
            const unordered_map<u32, unordered_set<u32>> &exception_map) {
    assert(pos1 < lits.size());
    const auto &lit1 = lits[pos1];
    for (const auto pos2 : parent_map) {
        if (contains(exception_map, pos2)) {
            const auto &exception_pos = exception_map.at(pos2);
            if (contains(exception_pos, pos1)) {
                return false;
            }
        }

        /* if lit1 isn't an exception of lit2, then we have to do further
         * exclusive check.
         * TODO: More mask checks. Note if two literals are group exclusive,
         * it is possible that they match at the same offset. */
        assert(pos2 < lits.size());
        const auto &lit2 = lits[pos2];
        if (isSuffix(lit2, lit1)) {
            return false;
        }
    }

    return true;
}

static
void buildSquashMask(vector<hwlmLiteral> &lits, u32 id1, u32 bucket1,
                     size_t start, const vector<pair<u32, u32>> &group,
                     unordered_map<u32, unordered_set<u32>> &parent_map,
                     unordered_map<u32, unordered_set<u32>> &exception_map) {
    auto &lit1 = lits[id1];
    DEBUG_PRINTF("b:%u len:%zu\n", bucket1, lit1.s.length());

    size_t cnt = group.size();
    bool included = false;
    bool exception = false;
    u32 child_id = ~0U;
    for (size_t i = start; i < cnt; i++) {
        u32 bucket2 = group[i].first;
        assert(bucket2 >= bucket1);

        u32 id2 = group[i].second;
        auto &lit2 = lits[id2];
        // check if lit2 is a suffix of lit1
        if (isSuffix(lit1, lit2)) {
            /* if we have a included literal in the same bucket,
             * quit and let the included literal to do possible squashing */
            if (bucket1 == bucket2) {
                DEBUG_PRINTF("same bucket\n");
                return;
            }
            /* if lit2 is a suffix but doesn't pass included checks for
             * extra info, we give up sqaushing */
            if (includedCheck(lit1, lit2)) {
                DEBUG_PRINTF("find exceptional suffix %u\n", lit2.id);
                exception_map[id1].insert(id2);
                exception = true;
            } else if (checkParentLit(lits, id1, parent_map[id2],
                       exception_map)) {
                if (lit1.included_id == INVALID_LIT_ID) {
                    DEBUG_PRINTF("find suffix lit1 %u lit2 %u\n",
                                 lit1.id, lit2.id);
                    lit1.included_id = lit2.id;
                } else {
                    /* if we have multiple included literals in one bucket,
                     * give up squashing. */
                    DEBUG_PRINTF("multiple included literals\n");
                    lit1.included_id = INVALID_LIT_ID;
                    return;
                }
                child_id = id2;
                included = true;
            }
        }

        size_t next = i + 1;
        u32 nextBucket = next < cnt ? group[next].first : ~0U;
        if (bucket2 != nextBucket) {
            if (included) {
                if (exception) {
                    /* give up if we have exception literals
                     * in the same bucket as the included literal. */
                    lit1.included_id = INVALID_LIT_ID;
                } else {
                    parent_map[child_id].insert(id1);

                    lit1.squash |= 1U << bucket2;
                    DEBUG_PRINTF("build squash mask %2x for %u\n",
                                 lit1.squash, lit1.id);
                }
                return;
            }
            exception = false;
        }
    }
}

static constexpr u32 INCLUDED_LIMIT = 1000;

static
void findIncludedLits(vector<hwlmLiteral> &lits,
                      const vector<vector<pair<u32, u32>>> &lastCharMap) {
    /* Map for finding the positions of literal which includes a literal
     * in FDR hwlm literal vector. */
    unordered_map<u32, unordered_set<u32>> parent_map;

    /* Map for finding the positions of exception literals which could
     * sometimes match if a literal matches in FDR hwlm literal vector. */
    unordered_map<u32, unordered_set<u32>> exception_map;
    for (const auto &group : lastCharMap) {
        size_t cnt = group.size();
        if (cnt > INCLUDED_LIMIT) {
            continue;
        }
        for (size_t i = 0; i < cnt; i++) {
            u32 bucket1 = group[i].first;
            u32 id1 = group[i].second;
            buildSquashMask(lits, id1, bucket1, i + 1, group, parent_map,
                            exception_map);
        }
    }
}

static
void addIncludedInfo(
               vector<hwlmLiteral> &lits, u32 nBuckets,
               map<BucketIndex, vector<LiteralIndex>> &bucketToLits) {
    vector<vector<pair<u32, u32>>> lastCharMap(256);

    for (BucketIndex b = 0; b < nBuckets; b++) {
        if (!bucketToLits[b].empty()) {
            for (const LiteralIndex &lit_idx : bucketToLits[b]) {
                const auto &lit = lits[lit_idx];
                u8 c = mytoupper(lit.s.back());
                lastCharMap[c].emplace_back(b, lit_idx);
            }
        }
    }

    findIncludedLits(lits, lastCharMap);
}

} // namespace

static
unique_ptr<HWLMProto> fdrBuildProtoInternal(u8 engType,
                                            vector<hwlmLiteral> &lits,
                                            bool make_small,
                                            const target_t &target,
                                            const Grey &grey, u32 hint) {
    DEBUG_PRINTF("cpu has %s\n", target.has_avx2() ? "avx2" : "no-avx2");

    if (grey.fdrAllowTeddy) {
        auto proto = teddyBuildProtoHinted(engType, lits, make_small, hint,
                                           target);
        if (proto) {
            DEBUG_PRINTF("build with teddy succeeded\n");
            return proto;
        } else {
            DEBUG_PRINTF("build with teddy failed, will try with FDR\n");
        }
    }

    auto des = (hint == HINT_INVALID) ? chooseEngine(target, lits, make_small)
                                      : getFdrDescription(hint);
    if (!des) {
        return nullptr;
    }

    // temporary hack for unit testing
    if (hint != HINT_INVALID) {
        des->bits = 9;
        des->stride = 1;
    }

    auto bucketToLits = assignStringsToBuckets(lits, *des);
    addIncludedInfo(lits, des->getNumBuckets(), bucketToLits);
    auto proto =
        ue2::make_unique<HWLMProto>(engType, move(des), lits, bucketToLits,
                                    make_small);
    return proto;
}

unique_ptr<HWLMProto> fdrBuildProto(u8 engType, vector<hwlmLiteral> lits,
                                    bool make_small, const target_t &target,
                                    const Grey &grey) {
    return fdrBuildProtoInternal(engType, lits, make_small, target, grey,
                                 HINT_INVALID);
}

static
bytecode_ptr<FDR> fdrBuildTableInternal(const HWLMProto &proto,
                                        const Grey &grey) {

    if (proto.teddyEng) {
        return teddyBuildTable(proto, grey);
    }

    FDRCompiler fc(proto.lits, proto.bucketToLits, *(proto.fdrEng),
                   proto.make_small, grey);
    return fc.build();
}

bytecode_ptr<FDR> fdrBuildTable(const HWLMProto &proto, const Grey &grey) {
    return fdrBuildTableInternal(proto, grey);
}

#if !defined(RELEASE_BUILD)

unique_ptr<HWLMProto> fdrBuildProtoHinted(u8 engType,
                                          vector<hwlmLiteral> lits,
                                          bool make_small, u32 hint,
                                          const target_t &target,
                                          const Grey &grey) {
    return fdrBuildProtoInternal(engType, lits, make_small, target, grey,
                                 hint);
}

#endif

size_t fdrSize(const FDR *fdr) {
    assert(fdr);
    return fdr->size;
}

} // namespace ue2
