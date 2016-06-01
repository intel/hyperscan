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

/** \file
 * \brief FDR literal matcher: build API.
 */

#include "fdr_internal.h"
#include "fdr_compile.h"
#include "fdr_confirm.h"
#include "fdr_compile_internal.h"
#include "fdr_engine_description.h"
#include "teddy_compile.h"
#include "teddy_engine_description.h"
#include "grey.h"
#include "ue2common.h"
#include "util/alloc.h"
#include "util/compare.h"
#include "util/dump_mask.h"
#include "util/target_info.h"
#include "util/ue2string.h"
#include "util/verify_types.h"

#include <algorithm>
#include <cassert>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <boost/core/noncopyable.hpp>

using namespace std;

namespace ue2 {

namespace {

class FDRCompiler : boost::noncopyable {
private:
    const FDREngineDescription &eng;
    vector<u8> tab;
    const vector<hwlmLiteral> &lits;
    map<BucketIndex, std::vector<LiteralIndex> > bucketToLits;
    bool make_small;

    u8 *tabIndexToMask(u32 indexInTable);
    void assignStringToBucket(LiteralIndex l, BucketIndex b);
    void assignStringsToBuckets();
#ifdef DEBUG
    void dumpMasks(const u8 *defaultMask);
#endif
    void setupTab();
    aligned_unique_ptr<FDR> setupFDR(pair<u8 *, size_t> link);
    void createInitialState(FDR *fdr);

public:
    FDRCompiler(const vector<hwlmLiteral> &lits_in,
                const FDREngineDescription &eng_in, bool make_small_in)
        : eng(eng_in), tab(eng_in.getTabSizeBytes()), lits(lits_in),
          make_small(make_small_in) {}

    aligned_unique_ptr<FDR> build(pair<u8 *, size_t> link);
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
        for (vector<LiteralIndex>::const_iterator it = bucket_lits.begin(),
                                                  ite = bucket_lits.end();
             it != ite; ++it) {
            min_len = min(min_len, verify_u32(lits[*it].s.length()));
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

aligned_unique_ptr<FDR> FDRCompiler::setupFDR(pair<u8 *, size_t> link) {
    size_t tabSize = eng.getTabSizeBytes();

    pair<u8 *, size_t> floodControlTmp = setupFDRFloodControl(lits, eng);

    pair<u8 *, size_t> confirmTmp =
        setupFullMultiConfs(lits, eng, bucketToLits, make_small);

    assert(ISALIGNED_16(tabSize));
    assert(ISALIGNED_16(confirmTmp.second));
    assert(ISALIGNED_16(floodControlTmp.second));
    assert(ISALIGNED_16(link.second));
    size_t headerSize = ROUNDUP_16(sizeof(FDR));
    size_t size = ROUNDUP_16(headerSize + tabSize + confirmTmp.second +
                             floodControlTmp.second + link.second);

    DEBUG_PRINTF("sizes base=%zu tabSize=%zu confirm=%zu floodControl=%zu "
                 "total=%zu\n",
                 headerSize, tabSize, confirmTmp.second, floodControlTmp.second,
                 size);

    aligned_unique_ptr<FDR> fdr = aligned_zmalloc_unique<FDR>(size);
    assert(fdr); // otherwise would have thrown std::bad_alloc

    fdr->size = size;
    fdr->engineID = eng.getID();
    fdr->maxStringLen = verify_u32(maxLen(lits));
    createInitialState(fdr.get());

    u8 *fdr_base = (u8 *)fdr.get();
    u8 * ptr = fdr_base + ROUNDUP_16(sizeof(FDR));
    copy(tab.begin(), tab.end(), ptr);
    ptr += tabSize;

    memcpy(ptr, confirmTmp.first, confirmTmp.second);
    ptr += confirmTmp.second;
    aligned_free(confirmTmp.first);

    fdr->floodOffset = verify_u32(ptr - fdr_base);
    memcpy(ptr, floodControlTmp.first, floodControlTmp.second);
    ptr += floodControlTmp.second;
    aligned_free(floodControlTmp.first);

    /*  we are allowing domains 9 to 15 only */
    assert(eng.bits > 8 && eng.bits < 16);
    fdr->domain = eng.bits;
    fdr->domainMask = (1 << eng.bits) - 1;
    fdr->tabSize = (1 << eng.bits) * (eng.schemeWidth / 8);
    fdr->stride = eng.stride;

    if (link.first) {
        fdr->link = verify_u32(ptr - fdr_base);
        memcpy(ptr, link.first, link.second);
        aligned_free(link.first);
    } else {
        fdr->link = 0;
    }

    return fdr;
}

void FDRCompiler::assignStringToBucket(LiteralIndex l, BucketIndex b) {
    bucketToLits[b].push_back(l);
}

struct LitOrder {
    explicit LitOrder(const vector<hwlmLiteral> &vl_) : vl(vl_) {}
    bool operator()(const u32 &i1, const u32 &i2) const {
        const string &i1s = vl[i1].s;
        const string &i2s = vl[i2].s;

        size_t len1 = i1s.size(), len2 = i2s.size();

        if (len1 != len2) {
            return len1 < len2;
        } else {
            string::const_reverse_iterator it1, it2;
            tie(it1, it2) =
                std::mismatch(i1s.rbegin(), i1s.rend(), i2s.rbegin());
            if (it1 == i1s.rend()) {
                return false;
            }
            return *it1 < *it2;
        }
    }

private:
    const vector<hwlmLiteral> &vl;
};

static u64a getScoreUtil(u32 len, u32 count) {
    if (len == 0) {
        return (u64a)-1;
    }
    const u32 LEN_THRESH = 128;
    const u32 elen = (len > LEN_THRESH) ? LEN_THRESH : len;
    const u64a lenScore =
        (LEN_THRESH * LEN_THRESH * LEN_THRESH) / (elen * elen * elen);
    return count * lenScore; // deemphasize count - possibly more than needed
                             // this might be overkill in the other direction
}

//#define DEBUG_ASSIGNMENT
void FDRCompiler::assignStringsToBuckets() {
    typedef u64a SCORE; // 'Score' type
    const SCORE MAX_SCORE = (SCORE)-1;
    const u32 CHUNK_MAX = 512;
    const u32 BUCKET_MAX = 16;
    typedef pair<SCORE, u32> SCORE_INDEX_PAIR;

    u32 ls = verify_u32(lits.size());
    assert(ls); // Shouldn't be called with no literals.

    // make a vector that contains our literals as pointers or u32 LiteralIndex values
    vector<LiteralIndex> vli;
    vli.resize(ls);
    map<u32, u32> lenCounts;
    for (LiteralIndex l = 0; l < ls; l++) {
        vli[l] = l;
        lenCounts[lits[l].s.size()]++;
    }
    // sort vector by literal length + if tied on length, 'magic' criteria of some kind (tbd)
    stable_sort(vli.begin(), vli.end(), LitOrder(lits));

#ifdef DEBUG_ASSIGNMENT
    for (map<u32, u32>::iterator i = lenCounts.begin(), e = lenCounts.end();
         i != e; ++i) {
        printf("l<%d>:%d ", i->first, i->second);
    }
    printf("\n");
#endif

    // TODO: detailed early stage literal analysis for v. small cases (actually look at lits)
    // yes - after we factor this out and merge in the Teddy style of building we can look
    // at this, although the teddy merge modelling is quite different. It's still probably
    // adaptable to some extent for this class of problem

    u32 firstIds[CHUNK_MAX]; // how many are in this chunk (CHUNK_MAX - 1 contains 'last' bound)
    u32 count[CHUNK_MAX]; // how many are in this chunk
    u32 length[CHUNK_MAX]; // how long things in the chunk are

    const u32 MAX_CONSIDERED_LENGTH = 16;
    u32 currentChunk = 0;
    u32 currentSize = 0;
    u32 chunkStartID = 0;
    u32 maxPerChunk  = ls/(CHUNK_MAX - MIN(MAX_CONSIDERED_LENGTH, lenCounts.size())) + 1;

    for (u32 i = 0; i < ls && currentChunk < CHUNK_MAX - 1; i++) {
        LiteralIndex l = vli[i];
        if ((currentSize < MAX_CONSIDERED_LENGTH && (lits[l].s.size() != currentSize)) ||
            (currentSize != 1 && ((i - chunkStartID) >= maxPerChunk))) {
            currentSize = lits[l].s.size();
            if (currentChunk) {
                count[currentChunk - 1 ] = i - chunkStartID;
            }
            chunkStartID = firstIds[currentChunk] = i;
            length[currentChunk] = currentSize;
            currentChunk++;
        }
    }

    assert(currentChunk > 0);
    count[currentChunk - 1] = ls - chunkStartID;
    // close off chunks with an empty row
    firstIds[currentChunk] = ls;
    length[currentChunk] = 0;
    count[currentChunk] = 0;
    u32 nChunks = currentChunk + 1;

#ifdef DEBUG_ASSIGNMENT
    for (u32 j = 0; j < nChunks; j++) {
        printf("%d %d %d %d\n", j, firstIds[j], count[j], length[j]);
    }
#endif

    SCORE_INDEX_PAIR t[CHUNK_MAX][BUCKET_MAX]; // pair of score, index
    u32 nb = eng.getNumBuckets();

    for (u32 j = 0; j < nChunks; j++) {
        u32 cnt = 0;
        for (u32 k = j; k < nChunks; ++k) {
            cnt += count[k];
        }
        t[j][0] = make_pair(getScoreUtil(length[j], cnt), 0);
    }

    for (u32 i = 1; i < nb; i++) {
        for (u32 j = 0; j < nChunks - 1; j++) { // don't process last, empty row
            SCORE_INDEX_PAIR best = make_pair(MAX_SCORE, 0);
            u32 cnt = count[j];
            for (u32 k = j + 1; k < nChunks - 1; k++, cnt += count[k]) {
                SCORE score = getScoreUtil(length[j], cnt);
                if (score > best.first) {
                    break; // if we're now worse locally than our best score, give up
                }
                score += t[k][i-1].first;
                if (score < best.first) {
                    best = make_pair(score, k);
                }
            }
            t[j][i] = best;
        }
        t[nChunks - 1][i] = make_pair(0,0); // fill in empty final row for next iteration
    }

#ifdef DEBUG_ASSIGNMENT
    for (u32 j = 0; j < nChunks; j++) {
        for (u32 i = 0; i < nb; i++) {
            SCORE_INDEX_PAIR v = t[j][i];
            printf("<%7lld,%3d>", v.first, v.second);
        }
        printf("\n");
    }
#endif

    // our best score is in best[0][N_BUCKETS-1] and we can follow the links
    // to find where our buckets should start and what goes into them
    for (u32 i = 0, n = nb; n && (i != nChunks - 1); n--) {
        u32 j = t[i][n - 1].second;
        if (j == 0) {
            j = nChunks - 1;
        }
        // put chunks between i - j into bucket (NBUCKETS-1) - n
#ifdef DEBUG_ASSIGNMENT
        printf("placing from %d to %d in bucket %d\n", firstIds[i], firstIds[j],
               nb - n);
#endif
        for (u32 k = firstIds[i]; k < firstIds[j]; k++) {
            assignStringToBucket((LiteralIndex)vli[k], nb - n);
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

    for (vector<LiteralIndex>::const_iterator i = vl.begin(), e = vl.end();
         i != e; ++i) {
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

    typedef std::map<u32, ue2::unordered_set<u32> > M2SET;

    for (BucketIndex b = 0; b < eng.getNumBuckets(); b++) {
        const vector<LiteralIndex> &vl = bucketToLits[b];
        SuffixPositionInString pLimit = eng.getBucketWidth(b);
        for (SuffixPositionInString pos = 0; pos < pLimit; pos++) {
            u32 bit = eng.getSchemeBit(b, pos);
            M2SET m2;
            bool done = getMultiEntriesAtPosition(eng, vl, lits, pos, m2);
            if (done) {
                clearbit(&defaultMask[0], bit);
                continue;
            }
            for (M2SET::const_iterator i = m2.begin(), e = m2.end(); i != e;
                 ++i) {
                u32 dc = i->first;
                const ue2::unordered_set<u32> &mskSet = i->second;
                u32 v = ~dc;
                do {
                    u32 b2 = v & dc;
                    for (ue2::unordered_set<u32>::const_iterator
                             i2 = mskSet.begin(),
                             e2 = mskSet.end();
                         i2 != e2; ++i2) {
                        u32 val = (*i2 & ~dc) | b2;
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

aligned_unique_ptr<FDR> FDRCompiler::build(pair<u8 *, size_t> link) {
    assignStringsToBuckets();
    setupTab();
    return setupFDR(link);
}

} // namespace

static
aligned_unique_ptr<FDR>
fdrBuildTableInternal(const vector<hwlmLiteral> &lits, bool make_small,
                      const target_t &target, const Grey &grey, u32 hint,
                      hwlmStreamingControl *stream_control) {
    pair<u8 *, size_t> link(nullptr, 0);
    if (stream_control) {
        link = fdrBuildTableStreaming(lits, stream_control);
    }

    DEBUG_PRINTF("cpu has %s\n", target.has_avx2() ? "avx2" : "no-avx2");

    if (grey.fdrAllowTeddy) {
        aligned_unique_ptr<FDR> fdr
            = teddyBuildTableHinted(lits, make_small, hint, target, link);
        if (fdr) {
            DEBUG_PRINTF("build with teddy succeeded\n");
            return fdr;
        } else {
            DEBUG_PRINTF("build with teddy failed, will try with FDR\n");
        }
    }

    const unique_ptr<FDREngineDescription> des =
        (hint == HINT_INVALID) ? chooseEngine(target, lits, make_small)
                               : getFdrDescription(hint);

    if (!des) {
        return nullptr;
    }

    // temporary hack for unit testing
    if (hint != HINT_INVALID) {
        des->bits = 9;
        des->stride = 1;
    }

    FDRCompiler fc(lits, *des, make_small);
    return fc.build(link);
}

aligned_unique_ptr<FDR> fdrBuildTable(const vector<hwlmLiteral> &lits,
                                      bool make_small, const target_t &target,
                                      const Grey &grey,
                                      hwlmStreamingControl *stream_control) {
    return fdrBuildTableInternal(lits, make_small, target, grey, HINT_INVALID,
                                 stream_control);
}

#if !defined(RELEASE_BUILD)

aligned_unique_ptr<FDR>
fdrBuildTableHinted(const vector<hwlmLiteral> &lits, bool make_small, u32 hint,
                    const target_t &target, const Grey &grey,
                    hwlmStreamingControl *stream_control) {
    pair<u8 *, size_t> link(nullptr, 0);
    return fdrBuildTableInternal(lits, make_small, target, grey, hint,
                                 stream_control);
}

#endif

size_t fdrSize(const FDR *fdr) {
    assert(fdr);
    return fdr->size;
}

} // namespace ue2
