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

/**
 * \file
 * \brief FDR literal matcher: Teddy build code.
 */

#include "teddy_compile.h"

#include "fdr.h"
#include "fdr_internal.h"
#include "fdr_compile_internal.h"
#include "fdr_confirm.h"
#include "fdr_engine_description.h"
#include "teddy_internal.h"
#include "teddy_engine_description.h"
#include "grey.h"
#include "ue2common.h"
#include "util/alloc.h"
#include "util/compare.h"
#include "util/noncopyable.h"
#include "util/popcount.h"
#include "util/target_info.h"
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

using namespace std;

namespace ue2 {

namespace {

//#define TEDDY_DEBUG

class TeddyCompiler : noncopyable {
    const TeddyEngineDescription &eng;
    const Grey &grey;
    const vector<hwlmLiteral> &lits;
    bool make_small;

public:
    TeddyCompiler(const vector<hwlmLiteral> &lits_in,
                  const TeddyEngineDescription &eng_in, bool make_small_in,
                  const Grey &grey_in)
        : eng(eng_in), grey(grey_in), lits(lits_in), make_small(make_small_in) {
    }

    bytecode_ptr<FDR> build();
    bool pack(map<BucketIndex, std::vector<LiteralIndex> > &bucketToLits);
};

class TeddySet {
    u32 len;
    // nibbleSets is a series of bitfields over 16 predicates
    // that represent the whether shufti nibble set
    // so for num_masks = 4 we will represent our strings by
    // 8 u16s in the vector that indicate what a shufti bucket
    // would have to look like
    vector<u16> nibbleSets;
    set<u32> litIds;
public:
    explicit TeddySet(u32 len_in) : len(len_in), nibbleSets(len_in * 2, 0) {}
    const set<u32> & getLits() const { return litIds; }
    size_t litCount() const { return litIds.size(); }

    bool operator<(const TeddySet & s) const {
        return litIds < s.litIds;
    }

#ifdef TEDDY_DEBUG
    void dump() const {
        printf("TS: ");
        for (u32 i = 0; i < nibbleSets.size(); i++) {
            printf("%04x ", (u32)nibbleSets[i]);
        }
        printf("\nnlits: %zu\nLit ids: ", litCount());
        printf("Prob: %llu\n", probability());
        for (const auto &id : litIds) {
            printf("%u ", id);
        }
        printf("\n");
        printf("Flood prone : %s\n", isRunProne()?"yes":"no");
    }
#endif

    bool identicalTail(const TeddySet & ts) const {
        return nibbleSets == ts.nibbleSets;
    }

    void addLiteral(u32 lit_id, const hwlmLiteral &lit) {
        const string &s = lit.s;
        for (u32 i = 0; i < len; i++) {
            if (i < s.size()) {
                u8 c = s[s.size() - i - 1];
                u8 c_hi = (c >> 4) & 0xf;
                u8 c_lo = c & 0xf;
                nibbleSets[i*2] = 1 << c_lo;
                if (lit.nocase && ourisalpha(c)) {
                    nibbleSets[i*2+1] =  (1 << (c_hi&0xd)) | (1 << (c_hi|0x2));
                } else {
                    nibbleSets[i*2+1] =  1 << c_hi;
                }
            } else {
                nibbleSets[i*2] = nibbleSets[i*2+1] = 0xffff;
            }
        }
        litIds.insert(lit_id);
    }

    void merge(const TeddySet &ts) {
        for (u32 i = 0; i < nibbleSets.size(); i++) {
            nibbleSets[i] |= ts.nibbleSets[i];
        }
        litIds.insert(ts.litIds.begin(), ts.litIds.end());
    }

    // return a value p from 0 .. MAXINT64 that gives p/MAXINT64
    // likelihood of this TeddySet firing a first-stage accept
    // if it was given a bucket of its own and random data were
    // to be passed in
    u64a probability() const {
        u64a val = 1;
        for (size_t i = 0; i < nibbleSets.size(); i++) {
            val *= popcount32((u32)nibbleSets[i]);
        }
        return val;
    }

    // return a score based around the chance of this hitting times
    // a small fixed cost + the cost of traversing some sort of followup
    // (assumption is that the followup is linear)
    u64a heuristic() const {
        return probability() * (2+litCount());
    }

    bool isRunProne() const {
        u16 lo_and = 0xffff;
        u16 hi_and = 0xffff;
        for (u32 i = 0; i < len; i++) {
            lo_and &= nibbleSets[i*2];
            hi_and &= nibbleSets[i*2+1];
        }
        // we're not flood-prone if there's no way to get
        // through with a flood
        if (!lo_and || !hi_and) {
            return false;
        }
        return true;
    }
};

bool TeddyCompiler::pack(map<BucketIndex,
                             std::vector<LiteralIndex> > &bucketToLits) {
    set<TeddySet> sts;

    for (u32 i = 0; i < lits.size(); i++) {
        TeddySet ts(eng.numMasks);
        ts.addLiteral(i, lits[i]);
        sts.insert(ts);
    }

    while (1) {
#ifdef TEDDY_DEBUG
        printf("Size %zu\n", sts.size());
        for (const TeddySet &ts : sts) {
            printf("\n"); ts.dump();
        }
        printf("\n===============================================\n");
#endif

        auto m1 = sts.end(), m2 = sts.end();
        u64a best = 0xffffffffffffffffULL;

        for (auto i1 = sts.begin(), e1 = sts.end(); i1 != e1; ++i1) {
            const TeddySet &s1 = *i1;
            for (auto i2 = next(i1), e2 = sts.end(); i2 != e2; ++i2) {
                const TeddySet &s2 = *i2;

                // be more conservative if we don't absolutely need to
                // keep packing
                if ((sts.size() <= eng.getNumBuckets()) &&
                    !s1.identicalTail(s2)) {
                    continue;
                }

                TeddySet tmpSet(eng.numMasks);
                tmpSet.merge(s1);
                tmpSet.merge(s2);
                u64a newScore = tmpSet.heuristic();
                u64a oldScore = s1.heuristic() + s2.heuristic();
                if (newScore < oldScore) {
                    m1 = i1;
                    m2 = i2;
                    break;
                } else {
                    u64a score = newScore - oldScore;
                    bool oldRunProne = s1.isRunProne() && s2.isRunProne();
                    bool newRunProne = tmpSet.isRunProne();
                    if (newRunProne && !oldRunProne) {
                        continue;
                    }
                    if (score < best) {
                        best = score;
                        m1 = i1;
                        m2 = i2;
                    }
                }
            }
        }
        // if we didn't find a merge candidate, bail out
        if ((m1 == sts.end()) || (m2 == sts.end())) {
            break;
        }

        // do the merge
        TeddySet nts(eng.numMasks);
        nts.merge(*m1);
        nts.merge(*m2);
#ifdef TEDDY_DEBUG
        printf("Merging\n");
        printf("m1 = \n");
        m1->dump();
        printf("m2 = \n");
        m2->dump();
        printf("nts = \n");
        nts.dump();
        printf("\n===============================================\n");
#endif
        sts.erase(m1);
        sts.erase(m2);
        sts.insert(nts);
    }

    if (sts.size() > eng.getNumBuckets()) {
        return false;
    }

    u32 bucket_id = 0;
    for (const TeddySet &ts : sts) {
        const auto &ts_lits = ts.getLits();
        auto &bucket_lits = bucketToLits[bucket_id];
        bucket_lits.insert(end(bucket_lits), begin(ts_lits), end(ts_lits));
        bucket_id++;
    }
    return true;
}

bytecode_ptr<FDR> TeddyCompiler::build() {
    if (lits.size() > eng.getNumBuckets() * TEDDY_BUCKET_LOAD) {
        DEBUG_PRINTF("too many literals: %zu\n", lits.size());
        return nullptr;
    }

#ifdef TEDDY_DEBUG
    for (size_t i = 0; i < lits.size(); i++) {
        printf("lit %zu (len = %zu, %s) is ", i, lits[i].s.size(),
               lits[i].nocase ? "caseless" : "caseful");
        for (size_t j = 0; j < lits[i].s.size(); j++) {
            printf("%02x", ((u32)lits[i].s[j])&0xff);
        }
        printf("\n");
    }
#endif

    map<BucketIndex, std::vector<LiteralIndex> > bucketToLits;
    if(eng.needConfirm(lits)) {
        if (!pack(bucketToLits)) {
            DEBUG_PRINTF("more lits (%zu) than buckets (%u), can't pack.\n",
                         lits.size(), eng.getNumBuckets());
            return nullptr;
        }
    } else {
        for (u32 i = 0; i < lits.size(); i++) {
            bucketToLits[i].push_back(i);
        }
    }
    u32 maskWidth = eng.getNumBuckets() / 8;

    size_t maskLen = eng.numMasks * 16 * 2 * maskWidth;

    auto floodControlTmp = setupFDRFloodControl(lits, eng, grey);
    auto confirmTmp = setupFullConfs(lits, eng, bucketToLits, make_small);

    size_t size = ROUNDUP_N(sizeof(Teddy) +
                            maskLen +
                            confirmTmp.size() +
                            floodControlTmp.size(),
                            16 * maskWidth);

    auto fdr = make_zeroed_bytecode_ptr<FDR>(size, 64);
    assert(fdr); // otherwise would have thrown std::bad_alloc
    Teddy *teddy = (Teddy *)fdr.get(); // ugly
    u8 *teddy_base = (u8 *)teddy;

    teddy->size = size;
    teddy->engineID = eng.getID();
    teddy->maxStringLen = verify_u32(maxLen(lits));

    u8 *ptr = teddy_base + sizeof(Teddy) + maskLen;
    memcpy(ptr, confirmTmp.get(), confirmTmp.size());
    ptr += confirmTmp.size();

    teddy->floodOffset = verify_u32(ptr - teddy_base);
    memcpy(ptr, floodControlTmp.get(), floodControlTmp.size());
    ptr += floodControlTmp.size();

    u8 *baseMsk = teddy_base + sizeof(Teddy);

    for (const auto &b2l : bucketToLits) {
        const u32 &bucket_id = b2l.first;
        const vector<LiteralIndex> &ids = b2l.second;
        const u8 bmsk = 1U << (bucket_id % 8);

        for (const LiteralIndex &lit_id : ids) {
            const hwlmLiteral &l = lits[lit_id];
            DEBUG_PRINTF("putting lit %u into bucket %u\n", lit_id, bucket_id);
            const u32 sz = verify_u32(l.s.size());

            // fill in masks
            for (u32 j = 0; j < eng.numMasks; j++) {
                u32 msk_id_lo = j * 2 * maskWidth + (bucket_id  / 8);
                u32 msk_id_hi = (j * 2 + 1) * maskWidth + (bucket_id  / 8);

                // if we don't have a char at this position, fill in i
                // locations in these masks with '1'
                if (j >= sz) {
                    for (u32 n = 0; n < 16; n++) {
                        baseMsk[msk_id_lo * 16 + n] |= bmsk;
                        baseMsk[msk_id_hi * 16 + n] |= bmsk;
                    }
                } else {
                    u8 c = l.s[sz - 1 - j];
                    // if we do have a char at this position
                    const u32 hiShift = 4;
                    u32 n_hi = (c >> hiShift) & 0xf;
                    u32 n_lo = c & 0xf;

                    if (j < l.msk.size() && l.msk[l.msk.size() - 1 - j]) {
                        u8 m = l.msk[l.msk.size() - 1 - j];
                        u8 m_hi = (m >> hiShift) & 0xf;
                        u8 m_lo = m & 0xf;
                        u8 cmp = l.cmp[l.msk.size() - 1 - j];
                        u8 cmp_lo = cmp & 0xf;
                        u8 cmp_hi = (cmp >> hiShift) & 0xf;

                        for (u8 cm = 0; cm < 0x10; cm++) {
                            if ((cm & m_lo) == (cmp_lo & m_lo)) {
                                baseMsk[msk_id_lo * 16 + cm] |= bmsk;
                            }
                            if ((cm & m_hi) == (cmp_hi & m_hi)) {
                                baseMsk[msk_id_hi * 16 + cm] |= bmsk;
                            }
                        }
                    } else{
                        if (l.nocase && ourisalpha(c)) {
                            u32 cmHalfClear = (0xdf >> hiShift) & 0xf;
                            u32 cmHalfSet   = (0x20 >> hiShift) & 0xf;
                            baseMsk[msk_id_hi * 16 + (n_hi & cmHalfClear)] |= bmsk;
                            baseMsk[msk_id_hi * 16 + (n_hi | cmHalfSet  )] |= bmsk;
                        } else {
                            baseMsk[msk_id_hi * 16 + n_hi] |= bmsk;
                        }
                        baseMsk[msk_id_lo * 16 + n_lo] |= bmsk;
                    }
                }
            }
        }
    }


#ifdef TEDDY_DEBUG
    for (u32 i = 0; i < eng.numMasks * 2; i++) {
        for (u32 j = 0; j < 16; j++) {
            u8 val = baseMsk[i * 16 + j];
            for (u32 k = 0; k < 8; k++) {
                printf("%s", ((val >> k) & 0x1) ? "1" : "0");
            }
            printf(" ");
        }
        printf("\n");
    }
#endif

    return fdr;
}

} // namespace

bytecode_ptr<FDR> teddyBuildTableHinted(const vector<hwlmLiteral> &lits,
                                        bool make_small, u32 hint,
                                        const target_t &target,
                                        const Grey &grey) {
    unique_ptr<TeddyEngineDescription> des;
    if (hint == HINT_INVALID) {
        des = chooseTeddyEngine(target, lits);
    } else {
        des = getTeddyDescription(hint);
    }
    if (!des) {
        return nullptr;
    }
    TeddyCompiler tc(lits, *des, make_small, grey);
    return tc.build();
}

} // namespace ue2
