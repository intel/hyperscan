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

#include "smallwrite/smallwrite_build.h"

#include "grey.h"
#include "ue2common.h"
#include "nfa/mcclellancompile.h"
#include "nfa/mcclellancompile_util.h"
#include "nfa/nfa_internal.h"
#include "nfa/rdfa_merge.h"
#include "nfagraph/ng.h"
#include "nfagraph/ng_holder.h"
#include "nfagraph/ng_mcclellan.h"
#include "nfagraph/ng_util.h"
#include "nfagraph/ng_width.h"
#include "smallwrite/smallwrite_internal.h"
#include "util/alloc.h"
#include "util/charreach.h"
#include "util/compile_context.h"
#include "util/container.h"
#include "util/make_unique.h"
#include "util/ue2string.h"
#include "util/verify_types.h"

#include <map>
#include <set>
#include <vector>
#include <utility>

using namespace std;

namespace ue2 {

#define LITERAL_MERGE_CHUNK_SIZE 25
#define DFA_MERGE_MAX_STATES 8000

namespace { // unnamed

// Concrete impl class
class SmallWriteBuildImpl : public SmallWriteBuild {
public:
    SmallWriteBuildImpl(const ReportManager &rm, const CompileContext &cc);

    // Construct a runtime implementation.
    aligned_unique_ptr<SmallWriteEngine> build(u32 roseQuality) override;

    void add(const NGWrapper &w) override;
    void add(const ue2_literal &literal, ReportID r) override;

    bool determiniseLiterals();

    const ReportManager &rm;
    const CompileContext &cc;

    unique_ptr<raw_dfa> rdfa;
    vector<pair<ue2_literal, ReportID> > cand_literals;
    bool poisoned;
};

} // namespace

SmallWriteBuild::~SmallWriteBuild() { }

SmallWriteBuildImpl::SmallWriteBuildImpl(const ReportManager &rm_in,
                                         const CompileContext &cc_in)
    : rm(rm_in), cc(cc_in),
      /* small write is block mode only */
      poisoned(!cc.grey.allowSmallWrite || cc.streaming) {
}

void SmallWriteBuildImpl::add(const NGWrapper &w) {
    // If the graph is poisoned (i.e. we can't build a SmallWrite version),
    // we don't even try.
    if (poisoned) {
        return;
    }

    if (w.som || w.min_length || isVacuous(w)) { /* cannot support in smwr */
        poisoned = true;
        return;
    }

    DEBUG_PRINTF("w=%p\n", &w);

    // make a copy of the graph so that we can modify it for our purposes
    unique_ptr<NGHolder> h = cloneHolder(w);

    reduceGraph(*h, SOM_NONE, w.utf8, cc);

    // If the earliest match location is outside the small write region,
    // then we don't need to build a SmallWrite version.
    // However, we don't poison this case either, since it is simply a case,
    // where we know the resulting graph won't match.
    if (findMinWidth(*h) > depth(cc.grey.smallWriteLargestBuffer)) {
        return;
    }

    // Now we can actually build the McClellan DFA
    assert(h->kind == NFA_OUTFIX);
    auto r = buildMcClellan(*h, &rm, cc.grey);

    // If we couldn't build a McClellan DFA for this portion, we won't be able
    // build a smwr which represents the pattern set
    if (!r) {
        DEBUG_PRINTF("failed to determinise\n");
        poisoned = true;
        return;
    }

    prune_overlong(*r, cc.grey.smallWriteLargestBuffer);

    if (rdfa) {
        // do a merge of the new dfa with the existing dfa
        auto merged = mergeTwoDfas(rdfa.get(), r.get(), DFA_MERGE_MAX_STATES,
                                   &rm, cc.grey);
        if (!merged) {
            DEBUG_PRINTF("merge failed\n");
            poisoned = true;
            return;
        }
        DEBUG_PRINTF("merge succeeded, built %p\n", merged.get());
        rdfa = move(merged);
    } else {
        rdfa = move(r);
    }
}

void SmallWriteBuildImpl::add(const ue2_literal &literal, ReportID r) {
    // If the graph is poisoned (i.e. we can't build a SmallWrite version),
    // we don't even try.
    if (poisoned) {
        return;
    }

    if (literal.length() > cc.grey.smallWriteLargestBuffer) {
        return; /* too long */
    }

    cand_literals.push_back(make_pair(literal, r));
}

static
void lit_to_graph(NGHolder *h, const ue2_literal &literal, ReportID r) {
    NFAVertex u = h->startDs;
    for (const auto &c : literal) {
        NFAVertex v = add_vertex(*h);
        add_edge(u, v, *h);
        (*h)[v].char_reach = c;
        u = v;
    }
    (*h)[u].reports.insert(r);
    add_edge(u, h->accept, *h);
}

bool SmallWriteBuildImpl::determiniseLiterals() {
    DEBUG_PRINTF("handling literals\n");
    assert(!poisoned);

    if (cand_literals.empty()) {
        return true; /* nothing to do */
    }

    vector<unique_ptr<raw_dfa> > temp_dfas;

    for (const auto &cand : cand_literals) {
        NGHolder h;
        DEBUG_PRINTF("determinising %s\n", dumpString(cand.first).c_str());
        lit_to_graph(&h, cand.first, cand.second);
        temp_dfas.push_back(buildMcClellan(h, &rm, cc.grey));

        // If we couldn't build a McClellan DFA for this portion, then we
        // can't SmallWrite optimize the entire graph, so we can't
        // optimize any of it
        if (!temp_dfas.back()) {
            DEBUG_PRINTF("failed to determinise\n");
            poisoned = true;
            return false;
        }
    }

    if (!rdfa && temp_dfas.size() == 1) {
        /* no need to merge there is only one dfa */
        rdfa = move(temp_dfas[0]);
        return true;
    }

    /* do a merge of the new dfas */

    vector<const raw_dfa *> to_merge;

    if (rdfa) {/* also include the existing dfa */
        to_merge.push_back(rdfa.get());
    }

    for (const auto &d : temp_dfas) {
        to_merge.push_back(d.get());
    }

    assert(to_merge.size() > 1);

    while (to_merge.size() > LITERAL_MERGE_CHUNK_SIZE) {
        vector<const raw_dfa *> small_merge;
        small_merge.insert(small_merge.end(), to_merge.begin(),
                           to_merge.begin() + LITERAL_MERGE_CHUNK_SIZE);

        temp_dfas.push_back(
            mergeAllDfas(small_merge, DFA_MERGE_MAX_STATES, &rm, cc.grey));

        if (!temp_dfas.back()) {
            DEBUG_PRINTF("merge failed\n");
            poisoned = true;
            return false;
        }

        to_merge.erase(to_merge.begin(),
                       to_merge.begin() + LITERAL_MERGE_CHUNK_SIZE);
        to_merge.push_back(temp_dfas.back().get());
    }

    auto merged = mergeAllDfas(to_merge, DFA_MERGE_MAX_STATES, &rm, cc.grey);

    if (!merged) {
        DEBUG_PRINTF("merge failed\n");
        poisoned = true;
        return false;
    }

    DEBUG_PRINTF("merge succeeded, built %p\n", merged.get());

    // Replace our only DFA with the merged one
    rdfa = move(merged);

    return true;
}

#define MAX_GOOD_ACCEL_DEPTH 4

static
bool is_slow(const raw_dfa &rdfa, const set<dstate_id_t> &accel,
             u32 roseQuality) {
    /* we consider a dfa as slow if there is no way to quickly get into an accel
     * state/dead state. In these cases, it is more likely that we will be
     * running at our unaccelerated dfa speeds so the small write engine is only
     * competitive over a small region where start up costs are dominant. */

    if (roseQuality) {
        return true;
    }

    set<dstate_id_t> visited;
    set<dstate_id_t> next;
    set<dstate_id_t> curr;
    curr.insert(rdfa.start_anchored);

    u32 ialpha_size = rdfa.getImplAlphaSize();

    for (u32 i = 0; i < MAX_GOOD_ACCEL_DEPTH; i++) {
        next.clear();
        for (dstate_id_t s : curr) {
            if (contains(visited, s)) {
                continue;
            }
            visited.insert(s);
            if (s == DEAD_STATE || contains(accel, s)) {
                return false;
            }

            for (size_t j = 0; j < ialpha_size; j++) {
                next.insert(rdfa.states[s].next[j]);
            }
        }
        curr.swap(next);
    }

    return true;
}

static
aligned_unique_ptr<NFA> prepEngine(raw_dfa &rdfa, u32 roseQuality,
                                   const CompileContext &cc,
                                   const ReportManager &rm, u32 *start_offset,
                                   u32 *small_region) {
    *start_offset = remove_leading_dots(rdfa);

    // Unleash the McClellan!
    set<dstate_id_t> accel_states;

    auto nfa = mcclellanCompile(rdfa, cc, rm, &accel_states);
    if (!nfa) {
        DEBUG_PRINTF("mcclellan compile failed for smallwrite NFA\n");
        return nullptr;
    }

    if (is_slow(rdfa, accel_states, roseQuality)) {
        DEBUG_PRINTF("is slow\n");
        *small_region = cc.grey.smallWriteLargestBufferBad;
        if (*small_region <= *start_offset) {
            return nullptr;
        }
        if (prune_overlong(rdfa, *small_region - *start_offset)) {
            if (rdfa.start_anchored == DEAD_STATE) {
                DEBUG_PRINTF("all patterns pruned out\n");
                return nullptr;
            }

            nfa = mcclellanCompile(rdfa, cc, rm, &accel_states);
            if (!nfa) {
                DEBUG_PRINTF("mcclellan compile failed for smallwrite NFA\n");
                assert(0); /* able to build orig dfa but not the trimmed? */
                return nullptr;
            }
        }
    } else {
        *small_region = cc.grey.smallWriteLargestBuffer;
    }

    assert(isMcClellanType(nfa->type));
    if (nfa->length > cc.grey.limitSmallWriteOutfixSize
        || nfa->length > cc.grey.limitDFASize) {
        DEBUG_PRINTF("smallwrite outfix size too large\n");
        return nullptr; /* this is just a soft failure - don't build smwr */
    }

    nfa->queueIndex = 0; /* dummy, small write API does not use queue */
    return nfa;
}

// SmallWriteBuild factory
unique_ptr<SmallWriteBuild> makeSmallWriteBuilder(const ReportManager &rm,
                                                  const CompileContext &cc) {
    return ue2::make_unique<SmallWriteBuildImpl>(rm, cc);
}

aligned_unique_ptr<SmallWriteEngine>
SmallWriteBuildImpl::build(u32 roseQuality) {
    if (!rdfa && cand_literals.empty()) {
        DEBUG_PRINTF("no smallwrite engine\n");
        poisoned = true;
        return nullptr;
    }

    if (poisoned) {
        DEBUG_PRINTF("some pattern could not be made into a smallwrite dfa\n");
        return nullptr;
    }

    if (!determiniseLiterals()) {
        DEBUG_PRINTF("some literal could not be made into a smallwrite dfa\n");
        return nullptr;
    }

    DEBUG_PRINTF("building rdfa %p\n", rdfa.get());

    u32 start_offset;
    u32 small_region;
    auto nfa =
        prepEngine(*rdfa, roseQuality, cc, rm, &start_offset, &small_region);
    if (!nfa) {
        DEBUG_PRINTF("some smallwrite outfix could not be prepped\n");
        /* just skip the smallwrite optimization */
        poisoned = true;
        return nullptr;
    }

    u32 size = sizeof(SmallWriteEngine) + nfa->length;
    auto smwr = aligned_zmalloc_unique<SmallWriteEngine>(size);

    smwr->size = size;
    smwr->start_offset = start_offset;
    smwr->largestBuffer = small_region;

    /* copy in nfa after the smwr */
    assert(ISALIGNED_CL(smwr.get() + 1));
    memcpy(smwr.get() + 1, nfa.get(), nfa->length);

    DEBUG_PRINTF("smallwrite done %p\n", smwr.get());
    return smwr;
}

size_t smwrSize(const SmallWriteEngine *smwr) {
    assert(smwr);
    return smwr->size;
}

} // namespace ue2
