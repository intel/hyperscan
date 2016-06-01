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
#include "fdr_compile_internal.h"
#include "fdr_confirm.h"
#include "engine_description.h"
#include "teddy_engine_description.h"
#include "ue2common.h"
#include "util/alloc.h"
#include "util/bitutils.h"
#include "util/compare.h"
#include "util/verify_types.h"

#include <algorithm>
#include <cstring>
#include <set>

using namespace std;

namespace ue2 {

typedef u8 ConfSplitType;
typedef pair<BucketIndex, ConfSplitType> BucketSplitPair;
typedef map<BucketSplitPair, pair<FDRConfirm *, size_t> > BC2CONF;

// return the number of bytes beyond a length threshold in all strings in lits
static
size_t thresholdedSize(const vector<hwlmLiteral> &lits, size_t threshold) {
    size_t tot = 0;
    for (const auto &lit : lits) {
        size_t sz = lit.s.size();
        if (sz > threshold) {
            tot += ROUNDUP_N(sz - threshold, 8);
        }
    }
    return tot;
}

static
u64a make_u64a_mask(const vector<u8> &v) {
    assert(v.size() <= sizeof(u64a));
    if (v.size() > sizeof(u64a)) {
        throw std::exception();
    }

    u64a mask = 0;
    size_t vlen = v.size();
    size_t len = std::min(vlen, sizeof(mask));
    unsigned char *m = (unsigned char *)&mask;
    memcpy(m + sizeof(mask) - len, &v[vlen - len], len);
    return mask;
}

/**
 * Build a temporary vector of LitInfo structures (without the corresponding
 * pointers to the actual strings; these cannot be laid out yet). These
 * stay in 1:1 correspondence with the lits[] vector as that's the only
 * place we have to obtain our full strings.
 */
static
void fillLitInfo(const vector<hwlmLiteral> &lits, vector<LitInfo> &tmpLitInfo,
                 CONF_TYPE &andmsk) {
    const CONF_TYPE all_ones = ~(u64a)0;
    andmsk = all_ones; // fill in with 'and' of all literal masks

    for (LiteralIndex i = 0; i < lits.size(); i++) {
        const hwlmLiteral &lit = lits[i];
        LitInfo &info = tmpLitInfo[i];
        memset(&info, 0, sizeof(info));
        info.id = lit.id;
        u8 flags = NoFlags;
        if (lit.nocase) {
            flags |= Caseless;
        }
        if (lit.noruns) {
            flags |= NoRepeat;
        }
        if (lit.msk.size() > lit.s.size()) {
            flags |= ComplexConfirm;
            info.extended_size = verify_u8(lit.msk.size());
        }
        info.flags = flags;
        info.size = verify_u32(lit.s.size());
        info.groups = lit.groups;

        // these are built up assuming a LE machine
        CONF_TYPE msk = all_ones;
        CONF_TYPE val = 0;
        for (u32 j = 0; j < sizeof(CONF_TYPE); j++) {
            u32 shiftLoc = (sizeof(CONF_TYPE) - j - 1) * 8;
            if (j >= lit.s.size()) {
                msk &= ~((CONF_TYPE)0xff << shiftLoc);
            } else {
                u8 c = lit.s[lit.s.size() - j - 1];
                if (lit.nocase && ourisalpha(c)) {
                    msk &= ~((CONF_TYPE)CASE_BIT << shiftLoc);
                    val |= (CONF_TYPE)(c & CASE_CLEAR) << shiftLoc;
                } else {
                    val |= (CONF_TYPE)c << shiftLoc;
                }
            }
        }

        info.v = val;
        info.msk = msk;
        if (!lit.msk.empty()) {
            u64a l_msk = make_u64a_mask(lit.msk);
            u64a l_cmp = make_u64a_mask(lit.cmp);

            // test for consistency - if there's intersection, then v and msk
            // values must line up
            UNUSED u64a intersection = l_msk & info.msk;
            assert((info.v & intersection) == (l_cmp & intersection));

            // incorporate lit.msk, lit.cmp into v and msk
            info.msk |= l_msk;
            info.v |= l_cmp;
        }

        andmsk &= info.msk;
    }
}

//#define FDR_CONFIRM_DUMP 1

static
size_t getFDRConfirm(const vector<hwlmLiteral> &lits, FDRConfirm **fdrc_p,
                     bool applyOneCharOpt, bool make_small, bool make_confirm) {
    vector<LitInfo> tmpLitInfo(lits.size());
    CONF_TYPE andmsk;
    fillLitInfo(lits, tmpLitInfo, andmsk);

#ifdef FDR_CONFIRM_DUMP
    printf("-------------------\n");
#endif

    // just magic numbers and crude measures for now
    u32 nBits;
    if (make_small) {
        nBits = min(10U, lg2(lits.size()) + 1);
    } else {
        nBits = min(13U, lg2(lits.size()) + 4);
    }

    CONF_TYPE mult = (CONF_TYPE)0x0b4e0ef37bc32127ULL;
    u32 flags = 0;
    // we use next three variables for 'confirmless' case to speed-up
    // confirmation process
    u32 soleLitSize = 0;
    u32 soleLitCmp = 0;
    u32 soleLitMsk = 0;

    if ((applyOneCharOpt && lits.size() == 1 && lits[0].s.size() == 0 &&
            lits[0].msk.empty()) || make_confirm == false) {
        flags = FDRC_FLAG_NO_CONFIRM;
        if (lits[0].noruns) {
            flags |= NoRepeat; // messy - need to clean this up later as flags is sorta kinda obsoleted
        }
        mult = 0;
        soleLitSize = lits[0].s.size() - 1;
        // we can get to this point only in confirmless case;
        // it means that we have only one literal per FDRConfirm (no packing),
        // with no literal mask and size of literal is less or equal
        // to the number of masks of Teddy engine;
        // maximum number of masks for Teddy is 4, so the size of
        // literal is definitely less or equal to size of u32
        assert(lits[0].s.size() <= sizeof(u32));
        for (u32 i = 0; i < lits[0].s.size(); i++) {
            u32 shiftLoc = (sizeof(u32) - i - 1) * 8;
            u8 c = lits[0].s[lits[0].s.size() - i - 1];
            if (lits[0].nocase && ourisalpha(c)) {
                soleLitCmp |= (u32)(c & CASE_CLEAR) << shiftLoc;
                soleLitMsk |= (u32)CASE_CLEAR << shiftLoc;
            }
            else {
                soleLitCmp |= (u32)c << shiftLoc;
                soleLitMsk |= (u32)0xff << shiftLoc;
            }
        }
    }

    // we can walk the vector and assign elements from the vectors to a
    // map by hash value
    map<u32, vector<LiteralIndex> > res2lits;
    hwlm_group_t gm = 0;
    for (LiteralIndex i = 0; i < lits.size(); i++) {
        LitInfo & li = tmpLitInfo[i];
        u32 hash = CONF_HASH_CALL(li.v, andmsk, mult, nBits);
        DEBUG_PRINTF("%016llx --> %u\n", li.v, hash);
        res2lits[hash].push_back(i);
        gm |= li.groups;
    }

#ifdef FDR_CONFIRM_DUMP
    // print out the literals reversed - makes it easier to line up analyses
    // that are end-offset based
    for (map<u32, vector<LiteralIndex> >::iterator i = res2lits.begin(),
         e = res2lits.end(); i != e; ++i) {
        u32 hash = i->first;
        vector<LiteralIndex> & vlidx = i->second;
        if (vlidx.size() > 1) {
            printf("%x -> %zu literals\n", hash, vlidx.size());
            u32 min_len = lits[vlidx.front()].s.size();
            vector<set<u8> > vsl; // contains the set of chars at each location
                                  // reversed from the end
            vsl.resize(1024);
            u32 total_string_size = 0;
            for (vector<LiteralIndex>::iterator i2 = vlidx.begin(),
                 e2 = vlidx.end(); i2 != e2; ++i2) {
                LiteralIndex litIdx = *i2;
                total_string_size += lits[litIdx].s.size();
                for (u32 j = lits[litIdx].s.size(); j != 0 ; j--) {
                    vsl[lits[litIdx].s.size()-j].insert(lits[litIdx].s.c_str()[j - 1]);
                }
                min_len = MIN(min_len, lits[litIdx].s.size());
            }
            printf("common     ");
            for (u32 j = 0; j < min_len; j++) {
                if (vsl[j].size() == 1) {
                    printf("%02x", (u32)*vsl[j].begin());
                } else {
                    printf("__");
                }
            }
            printf("\n");
            for (vector<LiteralIndex>::iterator i2 = vlidx.begin(),
                 e2 = vlidx.end(); i2 != e2; ++i2) {
                LiteralIndex litIdx = *i2;
                printf("%8x  %c", lits[litIdx].id, lits[litIdx].nocase ? '!' : ' ');
                for (u32 j = lits[litIdx].s.size(); j != 0 ; j--) {
                    u32 dist_from_end = lits[litIdx].s.size() - j;
                    if (dist_from_end < min_len && vsl[dist_from_end].size() == 1) {
                        printf("__");
                    } else {
                        printf("%02x", (u32)lits[litIdx].s.c_str()[j-1]);
                    }
                }
                printf("\n");
            }
            u32 total_compares = 0;
            for (u32 j = 0; j < 1024; j++) { // naughty
                total_compares += vsl[j].size();
            }
            printf("Total compare load: %d Total string size: %d\n\n", total_compares, total_string_size);
        }
    }
#endif

    const size_t bitsToLitIndexSize = (1U << nBits) * sizeof(u32);
    const size_t totalLitSize = thresholdedSize(lits, sizeof(CONF_TYPE));

    // this size can now be a worst-case as we can always be a bit smaller
    size_t size = ROUNDUP_N(sizeof(FDRConfirm), alignof(u32)) +
                  ROUNDUP_N(bitsToLitIndexSize, alignof(LitInfo)) +
                  sizeof(LitInfo) * lits.size() + totalLitSize;
    size = ROUNDUP_N(size, alignof(FDRConfirm));

    FDRConfirm *fdrc = (FDRConfirm *)aligned_zmalloc(size);
    assert(fdrc); // otherwise would have thrown std::bad_alloc

    fdrc->andmsk = andmsk;
    fdrc->mult = mult;
    fdrc->nBitsOrSoleID = (flags & FDRC_FLAG_NO_CONFIRM) ? lits[0].id : nBits;
    fdrc->flags = flags;
    fdrc->soleLitSize = soleLitSize;
    fdrc->soleLitCmp = soleLitCmp;
    fdrc->soleLitMsk = soleLitMsk;

    fdrc->groups = gm;

    // After the FDRConfirm, we have the lit index array.
    u8 *fdrc_base = (u8 *)fdrc;
    u8 *ptr = fdrc_base + sizeof(*fdrc);
    ptr = ROUNDUP_PTR(ptr, alignof(u32));
    u32 *bitsToLitIndex = (u32 *)ptr;
    ptr += bitsToLitIndexSize;

    // After the lit index array, we have the LitInfo structures themselves,
    // which vary in size (as each may have a variable-length string after it).
    ptr = ROUNDUP_PTR(ptr, alignof(LitInfo));

    // Walk the map by hash value assigning indexes and laying out the
    // elements (and their associated string confirm material) in memory.
    for (std::map<u32, vector<LiteralIndex> >::const_iterator
             i = res2lits.begin(), e = res2lits.end(); i != e; ++i) {
        const u32 hash = i->first;
        const vector<LiteralIndex> &vlidx = i->second;
        bitsToLitIndex[hash] = verify_u32(ptr - (u8 *)fdrc);
        for (vector<LiteralIndex>::const_iterator i2 = vlidx.begin(),
             e2 = vlidx.end(); i2 != e2; ++i2) {
            LiteralIndex litIdx = *i2;

            // Write LitInfo header.
            u8 *oldPtr = ptr;
            LitInfo &finalLI = *(LitInfo *)ptr;
            finalLI = tmpLitInfo[litIdx];

            ptr += sizeof(LitInfo); // String starts directly after LitInfo.

            // Write literal prefix (everything before the last N characters,
            // as the last N are already confirmed).
            const string &t = lits[litIdx].s;
            if (t.size() > sizeof(CONF_TYPE)) {
                size_t prefix_len = t.size() - sizeof(CONF_TYPE);
                memcpy(&finalLI.s[0], t.c_str(), prefix_len);
                ptr = &finalLI.s[0] + prefix_len;
            }

            ptr = ROUNDUP_PTR(ptr, alignof(LitInfo));
            if (i2 + 1 == e2) {
                finalLI.next = 0x0;
            } else {
                // our next field represents an adjustment on top of
                // current address + the actual size of the literal
                // so we track any rounding up done for alignment and
                // add this in - that way we don't have to use bigger
                // than a u8 (for now)
                assert((size_t)(ptr - oldPtr) > t.size());
                finalLI.next = verify_u8(ptr - oldPtr - t.size());
            }
        }
        assert((size_t)(ptr - fdrc_base) <= size);
    }

    *fdrc_p = fdrc;

    // Return actual used size, not worst-case size. Must be rounded up to
    // FDRConfirm alignment so that the caller can lay out a sequence of these.
    size_t actual_size = ROUNDUP_N((size_t)(ptr - fdrc_base),
                                   alignof(FDRConfirm));
    assert(actual_size <= size);
    return actual_size;
}

static
u32 setupMultiConfirms(const vector<hwlmLiteral> &lits,
                       const EngineDescription &eng, BC2CONF &bc2Conf,
                       map<BucketIndex, vector<LiteralIndex> > &bucketToLits,
                       bool make_small) {
    u32 pullBack = eng.getConfirmPullBackDistance();
    u32 splitMask = eng.getConfirmTopLevelSplit() - 1;
    bool splitHasCase = splitMask & 0x20;

    bool makeConfirm = true;
    unique_ptr<TeddyEngineDescription> teddyDescr =
        getTeddyDescription(eng.getID());
    if (teddyDescr) {
        makeConfirm = teddyDescr->needConfirm(lits);
    }

    u32 totalConfirmSize = 0;
    for (BucketIndex b = 0; b < eng.getNumBuckets(); b++) {
        if (!bucketToLits[b].empty()) {
            vector<vector<hwlmLiteral> > vl(eng.getConfirmTopLevelSplit());
            for (vector<LiteralIndex>::const_iterator
                     i = bucketToLits[b].begin(),
                     e = bucketToLits[b].end();
                 i != e; ++i) {
                hwlmLiteral lit = lits[*i]; // copy
                // c is last char of this literal
                u8 c = *(lit.s.rbegin());

                bool suppressSplit = false;
                if (pullBack) {
                    // make a shorter string to work over if we're pulling back
                    // getFDRConfirm doesn't know about that stuff
                    assert(lit.s.size() >= pullBack);
                    lit.s.resize(lit.s.size() - pullBack);

                    u8 c_sub, c_sub_msk;
                    if (lit.msk.empty()) {
                        c_sub = 0;
                        c_sub_msk = 0;
                    } else {
                        c_sub = *(lit.cmp.rbegin());
                        c_sub_msk = *(lit.msk.rbegin());
                        size_t len = lit.msk.size() -
                                     min(lit.msk.size(), (size_t)pullBack);
                        lit.msk.resize(len);
                        lit.cmp.resize(len);
                    }

                    // if c_sub_msk is 0xff and lit.nocase
                    // resteer 'c' to an exact value and set suppressSplit
                    if ((c_sub_msk == 0xff) && (lit.nocase)) {
                        suppressSplit = true;
                        c = c_sub;
                    }
                }

                if (!suppressSplit && splitHasCase && lit.nocase &&
                    ourisalpha(c)) {
                    vl[(u8)(mytoupper(c) & splitMask)].push_back(lit);
                    vl[(u8)(mytolower(c) & splitMask)].push_back(lit);
                } else {
                    vl[c & splitMask].push_back(lit);
                }
            }

            for (u32 c = 0; c < eng.getConfirmTopLevelSplit(); c++) {
                if (!vl[c].empty()) {
                    DEBUG_PRINTF("b %d c %02x sz %zu\n", b, c, vl[c].size());
                    FDRConfirm *fdrc;
                    size_t size = getFDRConfirm(vl[c], &fdrc,
                                                eng.typicallyHoldsOneCharLits(),
                                                make_small, makeConfirm);
                    BucketSplitPair p = make_pair(b, c);
                    bc2Conf[p] = make_pair(fdrc, size);
                    totalConfirmSize += size;
                }
            }
        }
    }
    return totalConfirmSize;
}

pair<u8 *, size_t> setupFullMultiConfs(const vector<hwlmLiteral> &lits,
        const EngineDescription &eng,
        map<BucketIndex, vector<LiteralIndex> > &bucketToLits,
        bool make_small) {
    BC2CONF bc2Conf;
    u32 totalConfirmSize = setupMultiConfirms(lits, eng, bc2Conf, bucketToLits,
                                              make_small);

    u32 primarySwitch = eng.getConfirmTopLevelSplit();
    u32 nBuckets = eng.getNumBuckets();
    u32 totalConfSwitchSize = primarySwitch * nBuckets * sizeof(u32);
    u32 totalSize = ROUNDUP_16(totalConfSwitchSize + totalConfirmSize);

    u8 *buf = (u8 *)aligned_zmalloc(totalSize);
    assert(buf); // otherwise would have thrown std::bad_alloc

    u32 *confBase = (u32 *)buf;
    u8 *ptr = buf + totalConfSwitchSize;

    for (BC2CONF::const_iterator i = bc2Conf.begin(), e = bc2Conf.end(); i != e;
         ++i) {
        const pair<FDRConfirm *, size_t> &p = i->second;
        // confirm offset is relative to the base of this structure, now
        u32 confirm_offset = verify_u32(ptr - (u8 *)buf);
        memcpy(ptr, p.first, p.second);
        ptr += p.second;
        aligned_free(p.first);
        BucketIndex b = i->first.first;
        u8 c = i->first.second;
        u32 idx = c * nBuckets + b;
        confBase[idx] = confirm_offset;
    }
    return make_pair(buf, totalSize);
}

} // namespace ue2
