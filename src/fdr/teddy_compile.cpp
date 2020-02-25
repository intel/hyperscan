/*
 * Copyright (c) 2015-2020, Intel Corporation
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
#include "hwlm/hwlm_build.h"
#include "util/alloc.h"
#include "util/compare.h"
#include "util/container.h"
#include "util/make_unique.h"
#include "util/noncopyable.h"
#include "util/popcount.h"
#include "util/small_vector.h"
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

/** \brief Max number of Teddy masks we use. */
static constexpr size_t MAX_NUM_MASKS = 4;

class TeddyCompiler : noncopyable {
    const TeddyEngineDescription &eng;
    const Grey &grey;
    const vector<hwlmLiteral> &lits;
    map<BucketIndex, std::vector<LiteralIndex>> bucketToLits;
    bool make_small;

public:
    TeddyCompiler(const vector<hwlmLiteral> &lits_in,
                  map<BucketIndex, std::vector<LiteralIndex>> bucketToLits_in,
                  const TeddyEngineDescription &eng_in, bool make_small_in,
                  const Grey &grey_in)
        : eng(eng_in), grey(grey_in), lits(lits_in),
          bucketToLits(move(bucketToLits_in)), make_small(make_small_in) {}

    bytecode_ptr<FDR> build();
};

class TeddySet {
    /**
     * \brief Estimate of the max number of literals in a set, used to
     * minimise allocations.
     */
    static constexpr size_t LITS_PER_SET = 20;

    /** \brief Number of masks. */
    u32 len;

    /**
     * \brief A series of bitfields over 16 predicates that represent the
     * shufti nibble set.
     *
     * So for num_masks = 4 we will represent our strings by 8 u16s in the
     * vector that indicate what a shufti bucket would have to look like.
     */
    small_vector<u16, MAX_NUM_MASKS * 2> nibbleSets;

    /**
     * \brief Sorted, unique set of literals. We maintain our own set in a
     * sorted vector to minimise allocations.
     */
    small_vector<u32, LITS_PER_SET> litIds;

public:
    explicit TeddySet(u32 len_in) : len(len_in), nibbleSets(len_in * 2, 0) {}
    size_t litCount() const { return litIds.size(); }
    const small_vector<u32, LITS_PER_SET> &getLits() const { return litIds; }

    bool operator<(const TeddySet &s) const {
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
        printf("Flood prone : %s\n", isRunProne() ? "yes" : "no");
    }
#endif

    bool identicalTail(const TeddySet &ts) const {
        return nibbleSets == ts.nibbleSets;
    }

    void addLiteral(u32 lit_id, const hwlmLiteral &lit) {
        const string &s = lit.s;
        for (u32 i = 0; i < len; i++) {
            if (i < s.size()) {
                u8 c = s[s.size() - i - 1];
                u8 c_hi = (c >> 4) & 0xf;
                u8 c_lo = c & 0xf;
                nibbleSets[i * 2] = 1 << c_lo;
                if (lit.nocase && ourisalpha(c)) {
                    nibbleSets[i * 2 + 1] =
                        (1 << (c_hi & 0xd)) | (1 << (c_hi | 0x2));
                } else {
                    nibbleSets[i * 2 + 1] = 1 << c_hi;
                }
            } else {
                nibbleSets[i * 2] = nibbleSets[i * 2 + 1] = 0xffff;
            }
        }
        litIds.push_back(lit_id);
        sort_and_unique(litIds);
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
        return probability() * (2 + litCount());
    }

    bool isRunProne() const {
        u16 lo_and = 0xffff;
        u16 hi_and = 0xffff;
        for (u32 i = 0; i < len; i++) {
            lo_and &= nibbleSets[i * 2];
            hi_and &= nibbleSets[i * 2 + 1];
        }
        // we're not flood-prone if there's no way to get
        // through with a flood
        if (!lo_and || !hi_and) {
            return false;
        }
        return true;
    }

    friend TeddySet merge(const TeddySet &a, const TeddySet &b) {
        assert(a.nibbleSets.size() == b.nibbleSets.size());

        TeddySet m(a);

        for (size_t i = 0; i < m.nibbleSets.size(); i++) {
            m.nibbleSets[i] |= b.nibbleSets[i];
        }

        m.litIds.insert(m.litIds.end(), b.litIds.begin(), b.litIds.end());
        sort_and_unique(m.litIds);

        return m;
    }
};

static
bool pack(const vector<hwlmLiteral> &lits,
          const TeddyEngineDescription &eng,
          map<BucketIndex, std::vector<LiteralIndex>> &bucketToLits) {
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
            printf("\n");
            ts.dump();
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

                TeddySet tmpSet = merge(s1, s2);
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
        TeddySet nts = merge(*m1, *m2);
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

// this entry has all-zero mask to skip reinforcement
#define NO_REINFORCEMENT N_CHARS

// this means every entry in reinforcement table
#define ALL_CHAR_SET N_CHARS

// each item's reinforcement mask has REINFORCED_MSK_LEN bytes
#define REINFORCED_MSK_LEN 8

// reinforcement table size for each 8 buckets set
#define RTABLE_SIZE ((N_CHARS + 1) * REINFORCED_MSK_LEN)

static
void initReinforcedTable(u8 *rmsk) {
    u64a *mask = (u64a *)rmsk;
    fill_n(mask, N_CHARS, 0x00ffffffffffffffULL);
}

static
void fillReinforcedMskZero(u8 *rmsk) {
    u8 *mc = rmsk + NO_REINFORCEMENT * REINFORCED_MSK_LEN;
    fill_n(mc, REINFORCED_MSK_LEN, 0x00);
}

static
void fillReinforcedMsk(u8 *rmsk, u16 c, u32 j, u8 bmsk) {
    assert(j > 0);
    if (c == ALL_CHAR_SET) {
        for (size_t i = 0; i < N_CHARS; i++) {
            u8 *mc = rmsk + i * REINFORCED_MSK_LEN;
            mc[j - 1] &= ~bmsk;
        }
    } else {
        u8 *mc = rmsk + c * REINFORCED_MSK_LEN;
        mc[j - 1] &= ~bmsk;
    }
}

static
void fillDupNibbleMasks(const map<BucketIndex,
                                  vector<LiteralIndex>> &bucketToLits,
                        const vector<hwlmLiteral> &lits,
                        u32 numMasks, size_t maskLen,
                        u8 *baseMsk) {
    u32 maskWidth = 2;
    memset(baseMsk, 0xff, maskLen);

    for (const auto &b2l : bucketToLits) {
        const u32 &bucket_id = b2l.first;
        const vector<LiteralIndex> &ids = b2l.second;
        const u8 bmsk = 1U << (bucket_id % 8);

        for (const LiteralIndex &lit_id : ids) {
            const hwlmLiteral &l = lits[lit_id];
            DEBUG_PRINTF("putting lit %u into bucket %u\n", lit_id, bucket_id);
            const u32 sz = verify_u32(l.s.size());

            // fill in masks
            for (u32 j = 0; j < numMasks; j++) {
                const u32 msk_id_lo = j * 2 * maskWidth + (bucket_id / 8);
                const u32 msk_id_hi = (j * 2 + 1) * maskWidth + (bucket_id / 8);
                const u32 lo_base0 = msk_id_lo * 32;
                const u32 lo_base1 = msk_id_lo * 32 + 16;
                const u32 hi_base0 = msk_id_hi * 32;
                const u32 hi_base1 = msk_id_hi * 32 + 16;

                // if we don't have a char at this position, fill in i
                // locations in these masks with '1'
                if (j >= sz) {
                    for (u32 n = 0; n < 16; n++) {
                        baseMsk[lo_base0 + n] &= ~bmsk;
                        baseMsk[lo_base1 + n] &= ~bmsk;
                        baseMsk[hi_base0 + n] &= ~bmsk;
                        baseMsk[hi_base1 + n] &= ~bmsk;
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
                                baseMsk[lo_base0 + cm] &= ~bmsk;
                                baseMsk[lo_base1 + cm] &= ~bmsk;
                            }
                            if ((cm & m_hi) == (cmp_hi & m_hi)) {
                                baseMsk[hi_base0 + cm] &= ~bmsk;
                                baseMsk[hi_base1 + cm] &= ~bmsk;
                            }
                        }
                    } else {
                        if (l.nocase && ourisalpha(c)) {
                            u32 cmHalfClear = (0xdf >> hiShift) & 0xf;
                            u32 cmHalfSet = (0x20 >> hiShift) & 0xf;
                            baseMsk[hi_base0 + (n_hi & cmHalfClear)] &= ~bmsk;
                            baseMsk[hi_base1 + (n_hi & cmHalfClear)] &= ~bmsk;
                            baseMsk[hi_base0 + (n_hi | cmHalfSet)] &= ~bmsk;
                            baseMsk[hi_base1 + (n_hi | cmHalfSet)] &= ~bmsk;
                        } else {
                            baseMsk[hi_base0 + n_hi] &= ~bmsk;
                            baseMsk[hi_base1 + n_hi] &= ~bmsk;
                        }
                        baseMsk[lo_base0 + n_lo] &= ~bmsk;
                        baseMsk[lo_base1 + n_lo] &= ~bmsk;
                    }
                }
            }
        }
    }
}

static
void fillNibbleMasks(const map<BucketIndex,
                               vector<LiteralIndex>> &bucketToLits,
                     const vector<hwlmLiteral> &lits,
                     u32 numMasks, u32 maskWidth, size_t maskLen,
                     u8 *baseMsk) {
    memset(baseMsk, 0xff, maskLen);

    for (const auto &b2l : bucketToLits) {
        const u32 &bucket_id = b2l.first;
        const vector<LiteralIndex> &ids = b2l.second;
        const u8 bmsk = 1U << (bucket_id % 8);

        for (const LiteralIndex &lit_id : ids) {
            const hwlmLiteral &l = lits[lit_id];
            DEBUG_PRINTF("putting lit %u into bucket %u\n", lit_id, bucket_id);
            const u32 sz = verify_u32(l.s.size());

            // fill in masks
            for (u32 j = 0; j < numMasks; j++) {
                const u32 msk_id_lo = j * 2 * maskWidth + (bucket_id / 8);
                const u32 msk_id_hi = (j * 2 + 1) * maskWidth + (bucket_id / 8);
                const u32 lo_base = msk_id_lo * 16;
                const u32 hi_base = msk_id_hi * 16;

                // if we don't have a char at this position, fill in i
                // locations in these masks with '1'
                if (j >= sz) {
                    for (u32 n = 0; n < 16; n++) {
                        baseMsk[lo_base + n] &= ~bmsk;
                        baseMsk[hi_base + n] &= ~bmsk;
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
                                baseMsk[lo_base + cm] &= ~bmsk;
                            }
                            if ((cm & m_hi) == (cmp_hi & m_hi)) {
                                baseMsk[hi_base + cm] &= ~bmsk;
                            }
                        }
                    } else {
                        if (l.nocase && ourisalpha(c)) {
                            u32 cmHalfClear = (0xdf >> hiShift) & 0xf;
                            u32 cmHalfSet = (0x20 >> hiShift) & 0xf;
                            baseMsk[hi_base + (n_hi & cmHalfClear)] &= ~bmsk;
                            baseMsk[hi_base + (n_hi | cmHalfSet)] &= ~bmsk;
                        } else {
                            baseMsk[hi_base + n_hi] &= ~bmsk;
                        }
                        baseMsk[lo_base + n_lo] &= ~bmsk;
                    }
                }
            }
        }
    }
}

static
void fillReinforcedTable(const map<BucketIndex,
                                   vector<LiteralIndex>> &bucketToLits,
                         const vector<hwlmLiteral> &lits,
                         u8 *rtable_base, const u32 num_tables) {
    vector<u8 *> tables;
    for (u32 i = 0; i < num_tables; i++) {
        tables.push_back(rtable_base + i * RTABLE_SIZE);
    }

    for (auto t : tables) {
        initReinforcedTable(t);
    }

    for (const auto &b2l : bucketToLits) {
        const u32 &bucket_id = b2l.first;
        const vector<LiteralIndex> &ids = b2l.second;
        u8 *rmsk = tables[bucket_id / 8];
        const u8 bmsk = 1U << (bucket_id % 8);

        for (const LiteralIndex &lit_id : ids) {
            const hwlmLiteral &l = lits[lit_id];
            DEBUG_PRINTF("putting lit %u into bucket %u\n", lit_id, bucket_id);
            const u32 sz = verify_u32(l.s.size());

            // fill in reinforced masks
            for (u32 j = 1; j < REINFORCED_MSK_LEN; j++) {
                if (sz - 1 < j) {
                    fillReinforcedMsk(rmsk, ALL_CHAR_SET, j, bmsk);
                } else {
                    u8 c = l.s[sz - 1 - j];
                    if (l.nocase && ourisalpha(c)) {
                        u8 c_up = c & 0xdf;
                        fillReinforcedMsk(rmsk, c_up, j, bmsk);
                        u8 c_lo = c | 0x20;
                        fillReinforcedMsk(rmsk, c_lo, j, bmsk);
                    } else {
                        fillReinforcedMsk(rmsk, c, j, bmsk);
                    }
                }
            }
        }
    }

    for (auto t : tables) {
        fillReinforcedMskZero(t);
    }
}

bytecode_ptr<FDR> TeddyCompiler::build() {
    u32 maskWidth = eng.getNumBuckets() / 8;

    size_t headerSize = sizeof(Teddy);
    size_t maskLen = eng.numMasks * 16 * 2 * maskWidth;
    size_t reinforcedDupMaskLen = RTABLE_SIZE * maskWidth;
    if (maskWidth == 2) { // dup nibble mask table in Fat Teddy
        reinforcedDupMaskLen = maskLen * 2;
    }

    auto floodTable = setupFDRFloodControl(lits, eng, grey);
    auto confirmTable = setupFullConfs(lits, eng, bucketToLits, make_small);

    // Note: we place each major structure here on a cacheline boundary.
    size_t size = ROUNDUP_CL(headerSize) + ROUNDUP_CL(maskLen) +
                  ROUNDUP_CL(reinforcedDupMaskLen) +
                  ROUNDUP_CL(confirmTable.size()) + floodTable.size();

    auto fdr = make_zeroed_bytecode_ptr<FDR>(size, 64);
    assert(fdr); // otherwise would have thrown std::bad_alloc
    Teddy *teddy = (Teddy *)fdr.get(); // ugly
    u8 *teddy_base = (u8 *)teddy;

    // Write header.
    teddy->size = size;
    teddy->engineID = eng.getID();
    teddy->maxStringLen = verify_u32(maxLen(lits));
    teddy->numStrings = verify_u32(lits.size());

    // Write confirm structures.
    u8 *ptr = teddy_base + ROUNDUP_CL(headerSize) + ROUNDUP_CL(maskLen) +
              ROUNDUP_CL(reinforcedDupMaskLen);
    assert(ISALIGNED_CL(ptr));
    teddy->confOffset = verify_u32(ptr - teddy_base);
    memcpy(ptr, confirmTable.get(), confirmTable.size());
    ptr += ROUNDUP_CL(confirmTable.size());

    // Write flood control structures.
    assert(ISALIGNED_CL(ptr));
    teddy->floodOffset = verify_u32(ptr - teddy_base);
    memcpy(ptr, floodTable.get(), floodTable.size());
    ptr += floodTable.size();

    // Write teddy masks.
    u8 *baseMsk = teddy_base + ROUNDUP_CL(headerSize);
    fillNibbleMasks(bucketToLits, lits, eng.numMasks, maskWidth, maskLen,
                    baseMsk);

    if (maskWidth == 1) { // reinforcement table in Teddy
        // Write reinforcement masks.
        u8 *reinforcedMsk = baseMsk + ROUNDUP_CL(maskLen);
        fillReinforcedTable(bucketToLits, lits, reinforcedMsk, maskWidth);
    } else { // dup nibble mask table in Fat Teddy
        assert(maskWidth == 2);
        u8 *dupMsk = baseMsk + ROUNDUP_CL(maskLen);
        fillDupNibbleMasks(bucketToLits, lits, eng.numMasks,
			   reinforcedDupMaskLen, dupMsk);
    }

    return fdr;
}


static
bool assignStringsToBuckets(
                const vector<hwlmLiteral> &lits,
                TeddyEngineDescription &eng,
                map<BucketIndex, vector<LiteralIndex>> &bucketToLits) {
    assert(eng.numMasks <= MAX_NUM_MASKS);
    if (lits.size() > eng.getNumBuckets() * TEDDY_BUCKET_LOAD) {
        DEBUG_PRINTF("too many literals: %zu\n", lits.size());
        return false;
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

    if (!pack(lits, eng, bucketToLits)) {
        DEBUG_PRINTF("more lits (%zu) than buckets (%u), can't pack.\n",
                     lits.size(), eng.getNumBuckets());
        return false;
    }
    return true;
}

} // namespace

bytecode_ptr<FDR> teddyBuildTable(const HWLMProto &proto, const Grey &grey) {
    TeddyCompiler tc(proto.lits, proto.bucketToLits, *(proto.teddyEng),
                     proto.make_small, grey);
    return tc.build();
}


unique_ptr<HWLMProto> teddyBuildProtoHinted(
                        u8 engType, const vector<hwlmLiteral> &lits,
                        bool make_small, u32 hint, const target_t &target) {
    unique_ptr<TeddyEngineDescription> des;
    if (hint == HINT_INVALID) {
        des = chooseTeddyEngine(target, lits);
    } else {
        des = getTeddyDescription(hint);
    }
    if (!des) {
        return nullptr;
    }

    map<BucketIndex, std::vector<LiteralIndex>> bucketToLits;
    if (!assignStringsToBuckets(lits, *des, bucketToLits)) {
        return nullptr;
    }

    return ue2::make_unique<HWLMProto>(engType, move(des), lits,
                                       bucketToLits, make_small);
}

} // namespace ue2
