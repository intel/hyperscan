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
#include "util/dump_mask.h"
#include "util/math.h"
#include "util/noncopyable.h"
#include "util/target_info.h"
#include "util/ue2string.h"
#include "util/verify_types.h"

#include <algorithm>
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
    void assignStringsToBuckets();
#ifdef DEBUG
    void dumpMasks(const u8 *defaultMask);
#endif
    void setupTab();
    bytecode_ptr<FDR> setupFDR();
    void createInitialState(FDR *fdr);

public:
    FDRCompiler(vector<hwlmLiteral> lits_in, const FDREngineDescription &eng_in,
                bool make_small_in, const Grey &grey_in)
        : eng(eng_in), grey(grey_in), tab(eng_in.getTabSizeBytes()),
          lits(move(lits_in)), make_small(make_small_in) {}

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

bytecode_ptr<FDR> FDRCompiler::setupFDR() {
    size_t tabSize = eng.getTabSizeBytes();

    auto floodControlTmp = setupFDRFloodControl(lits, eng, grey);
    auto confirmTmp = setupFullConfs(lits, eng, bucketToLits, make_small);

    assert(ISALIGNED_16(tabSize));
    assert(ISALIGNED_16(confirmTmp.size()));
    assert(ISALIGNED_16(floodControlTmp.size()));
    size_t headerSize = ROUNDUP_16(sizeof(FDR));
    size_t size = ROUNDUP_16(headerSize + tabSize + confirmTmp.size() +
                             floodControlTmp.size());

    DEBUG_PRINTF("sizes base=%zu tabSize=%zu confirm=%zu floodControl=%zu "
                 "total=%zu\n",
                 headerSize, tabSize, confirmTmp.size(), floodControlTmp.size(),
                 size);

    auto fdr = make_zeroed_bytecode_ptr<FDR>(size, 64);
    assert(fdr); // otherwise would have thrown std::bad_alloc

    fdr->size = size;
    fdr->engineID = eng.getID();
    fdr->maxStringLen = verify_u32(maxLen(lits));
    createInitialState(fdr.get());

    u8 *fdr_base = (u8 *)fdr.get();
    u8 *ptr = fdr_base + ROUNDUP_16(sizeof(FDR));
    copy(tab.begin(), tab.end(), ptr);
    ptr += tabSize;

    memcpy(ptr, confirmTmp.get(), confirmTmp.size());
    ptr += confirmTmp.size();

    fdr->floodOffset = verify_u32(ptr - fdr_base);
    memcpy(ptr, floodControlTmp.get(), floodControlTmp.size());
    ptr += floodControlTmp.size();

    /*  we are allowing domains 9 to 15 only */
    assert(eng.bits > 8 && eng.bits < 16);
    fdr->domain = eng.bits;
    fdr->domainMask = (1 << eng.bits) - 1;
    fdr->tabSize = (1 << eng.bits) * (eng.schemeWidth / 8);
    fdr->stride = eng.stride;

    return fdr;
}

//#define DEBUG_ASSIGNMENT

static
double getScoreUtil(u32 len, u32 count) {
    return len == 0 ? numeric_limits<double>::max()
                    : our_pow(count, 1.05) * our_pow(len, -3.0);
}

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

void FDRCompiler::assignStringsToBuckets() {
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

    for (u32 j = 0; j < numChunks; j++) {
        u32 cnt = 0;
        for (u32 k = j; k < numChunks; ++k) {
            cnt += chunks[k].count;
        }
        t[j][0] = {getScoreUtil(chunks[j].length, cnt), 0};
    }

    for (u32 i = 1; i < numBuckets; i++) {
        for (u32 j = 0; j < numChunks - 1; j++) { // don't do last, empty row
            pair<double, u32> best = {MAX_SCORE, 0};
            u32 cnt = chunks[j].count;
            for (u32 k = j + 1; k < numChunks - 1; k++) {
                auto score = getScoreUtil(chunks[j].length, cnt);
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
    for (u32 i = 0, n = numBuckets; n && (i != numChunks - 1); n--) {
        u32 j = t[i][n - 1].second;
        if (j == 0) {
            j = numChunks - 1;
        }

        // put chunks between i - j into bucket (numBuckets - n).
        u32 first_id = chunks[i].first_id;
        u32 last_id = chunks[j].first_id;
        assert(first_id < last_id);
        u32 bucket = numBuckets - n;
        UNUSED const auto &first_lit = lits[first_id];
        UNUSED const auto &last_lit = lits[last_id - 1];
        DEBUG_PRINTF("placing [%u-%u) in bucket %u (%u lits, len %zu-%zu, "
                      "score %0.4f)\n",
                      first_id, last_id, bucket, last_id - first_id,
                      first_lit.s.length(), last_lit.s.length(),
                      getScoreUtil(first_lit.s.length(), last_id - first_id));

        auto &bucket_lits = bucketToLits[bucket];
        for (u32 k = first_id; k < last_id; k++) {
            bucket_lits.push_back(k);
        }
        i = j;
    }
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
                               std::map<u32, ue2::unordered_set<u32> > &m2) {
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
            map<u32, ue2::unordered_set<u32>> m2;
            bool done = getMultiEntriesAtPosition(eng, vl, lits, pos, m2);
            if (done) {
                clearbit(&defaultMask[0], bit);
                continue;
            }
            for (const auto &elem : m2) {
                u32 dc = elem.first;
                const ue2::unordered_set<u32> &mskSet = elem.second;
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
    assignStringsToBuckets();
    setupTab();
    return setupFDR();
}

} // namespace

static
bytecode_ptr<FDR> fdrBuildTableInternal(const vector<hwlmLiteral> &lits,
                                        bool make_small, const target_t &target,
                                        const Grey &grey, u32 hint) {
    DEBUG_PRINTF("cpu has %s\n", target.has_avx2() ? "avx2" : "no-avx2");

    if (grey.fdrAllowTeddy) {
        auto fdr = teddyBuildTableHinted(lits, make_small, hint, target, grey);
        if (fdr) {
            DEBUG_PRINTF("build with teddy succeeded\n");
            return fdr;
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

    FDRCompiler fc(lits, *des, make_small, grey);
    return fc.build();
}

bytecode_ptr<FDR> fdrBuildTable(const vector<hwlmLiteral> &lits,
                                bool make_small, const target_t &target,
                                const Grey &grey) {
    return fdrBuildTableInternal(lits, make_small, target, grey, HINT_INVALID);
}

#if !defined(RELEASE_BUILD)

bytecode_ptr<FDR> fdrBuildTableHinted(const vector<hwlmLiteral> &lits,
                                      bool make_small, u32 hint,
                                      const target_t &target,
                                      const Grey &grey) {
    return fdrBuildTableInternal(lits, make_small, target, grey, hint);
}

#endif

size_t fdrSize(const FDR *fdr) {
    assert(fdr);
    return fdr->size;
}

} // namespace ue2
