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
 * \brief Large Bounded Repeat (LBR) engine build code.
 */

#include "ng_lbr.h"

#include "grey.h"
#include "ng_holder.h"
#include "ng_repeat.h"
#include "ng_reports.h"
#include "nfa/castlecompile.h"
#include "nfa/lbr_internal.h"
#include "nfa/nfa_internal.h"
#include "nfa/repeatcompile.h"
#include "nfa/shufticompile.h"
#include "nfa/trufflecompile.h"
#include "util/alloc.h"
#include "util/bitutils.h" // for lg2
#include "util/compile_context.h"
#include "util/container.h"
#include "util/depth.h"
#include "util/dump_charclass.h"
#include "util/report_manager.h"
#include "util/verify_types.h"

using namespace std;

namespace ue2 {

static
u32 depth_to_u32(const depth &d) {
    assert(d.is_reachable());
    if (d.is_infinite()) {
        return REPEAT_INF;
    }

    u32 d_val = d;
    assert(d_val < REPEAT_INF);
    return d_val;
}

template<class LbrStruct> static
u64a* getTable(NFA *nfa) {
    char *ptr = (char *)nfa + sizeof(struct NFA) + sizeof(LbrStruct) +
                sizeof(RepeatInfo);
    ptr = ROUNDUP_PTR(ptr, alignof(u64a));
    return (u64a *)ptr;
}

template <class LbrStruct> static
void fillNfa(NFA *nfa, lbr_common *c, ReportID report, const depth &repeatMin,
             const depth &repeatMax, u32 minPeriod, enum RepeatType rtype) {
    assert(nfa);

    RepeatStateInfo rsi(rtype, repeatMin, repeatMax, minPeriod);

    DEBUG_PRINTF("selected %s model for {%s,%s} repeat\n",
                 repeatTypeName(rtype), repeatMin.str().c_str(),
                 repeatMax.str().c_str());

    // Fill the lbr_common structure first. Note that the RepeatInfo structure
    // directly follows the LbrStruct.
    const u32 info_offset = sizeof(LbrStruct);
    c->repeatInfoOffset = info_offset;
    c->report = report;

    RepeatInfo *info = (RepeatInfo *)((char *)c + info_offset);
    info->type = verify_u8(rtype);
    info->repeatMin = depth_to_u32(repeatMin);
    info->repeatMax = depth_to_u32(repeatMax);
    info->stateSize = rsi.stateSize;
    info->packedCtrlSize = rsi.packedCtrlSize;
    info->horizon = rsi.horizon;
    info->minPeriod = minPeriod;
    copy_bytes(&info->packedFieldSizes, rsi.packedFieldSizes);
    info->patchCount = rsi.patchCount;
    info->patchSize = rsi.patchSize;
    info->encodingSize = rsi.encodingSize;
    info->patchesOffset = rsi.patchesOffset;

    // Fill the NFA structure.
    nfa->nPositions = repeatMin;
    nfa->streamStateSize = verify_u32(rsi.packedCtrlSize + rsi.stateSize);
    nfa->scratchStateSize = (u32)sizeof(lbr_state);
    nfa->minWidth = verify_u32(repeatMin);
    nfa->maxWidth = repeatMax.is_finite() ? verify_u32(repeatMax) : 0;

    // Fill the lbr table for sparse lbr model.
    if (rtype == REPEAT_SPARSE_OPTIMAL_P) {
        u64a *table = getTable<LbrStruct>(nfa);
        // Adjust table length according to the optimal patch length.
        size_t len = nfa->length;
        assert((u32)repeatMax >= rsi.patchSize);
        len -= sizeof(u64a) * ((u32)repeatMax - rsi.patchSize);
        nfa->length = verify_u32(len);
        info->length = verify_u32(sizeof(RepeatInfo)
                                  + sizeof(u64a) * (rsi.patchSize + 1));
        copy_bytes(table, rsi.table);
    }
}

template <class LbrStruct> static
bytecode_ptr<NFA> makeLbrNfa(NFAEngineType nfa_type, enum RepeatType rtype,
                             const depth &repeatMax) {
    size_t tableLen = 0;
    if (rtype == REPEAT_SPARSE_OPTIMAL_P) {
        tableLen = sizeof(u64a) * (repeatMax + 1);
    }
    size_t len = sizeof(NFA) + sizeof(LbrStruct) + sizeof(RepeatInfo) +
                 tableLen + sizeof(u64a);
    auto nfa = make_zeroed_bytecode_ptr<NFA>(len);
    nfa->type = verify_u8(nfa_type);
    nfa->length = verify_u32(len);
    return nfa;
}

static
bytecode_ptr<NFA> buildLbrDot(const CharReach &cr, const depth &repeatMin,
                              const depth &repeatMax, u32 minPeriod,
                              bool is_reset, ReportID report) {
    if (!cr.all()) {
        return nullptr;
    }

    enum RepeatType rtype = chooseRepeatType(repeatMin, repeatMax, minPeriod,
                                             is_reset);
    auto nfa = makeLbrNfa<lbr_dot>(LBR_NFA_DOT, rtype, repeatMax);
    struct lbr_dot *ld = (struct lbr_dot *)getMutableImplNfa(nfa.get());

    fillNfa<lbr_dot>(nfa.get(), &ld->common, report, repeatMin, repeatMax,
                     minPeriod, rtype);

    DEBUG_PRINTF("built dot lbr\n");
    return nfa;
}

static
bytecode_ptr<NFA> buildLbrVerm(const CharReach &cr, const depth &repeatMin,
                               const depth &repeatMax, u32 minPeriod,
                               bool is_reset, ReportID report) {
    const CharReach escapes(~cr);

    if (escapes.count() != 1) {
        return nullptr;
    }

    enum RepeatType rtype = chooseRepeatType(repeatMin, repeatMax, minPeriod,
                                             is_reset);
    auto nfa = makeLbrNfa<lbr_verm>(LBR_NFA_VERM, rtype, repeatMax);
    struct lbr_verm *lv = (struct lbr_verm *)getMutableImplNfa(nfa.get());
    lv->c = escapes.find_first();

    fillNfa<lbr_verm>(nfa.get(), &lv->common, report, repeatMin, repeatMax,
                      minPeriod, rtype);

    DEBUG_PRINTF("built verm lbr\n");
    return nfa;
}

static
bytecode_ptr<NFA> buildLbrNVerm(const CharReach &cr, const depth &repeatMin,
                                const depth &repeatMax, u32 minPeriod,
                                bool is_reset, ReportID report) {
    const CharReach escapes(cr);

    if (escapes.count() != 1) {
        return nullptr;
    }

    enum RepeatType rtype = chooseRepeatType(repeatMin, repeatMax, minPeriod,
                                             is_reset);
    auto nfa = makeLbrNfa<lbr_verm>(LBR_NFA_NVERM, rtype, repeatMax);
    struct lbr_verm *lv = (struct lbr_verm *)getMutableImplNfa(nfa.get());
    lv->c = escapes.find_first();

    fillNfa<lbr_verm>(nfa.get(), &lv->common, report, repeatMin, repeatMax,
                      minPeriod, rtype);

    DEBUG_PRINTF("built negated verm lbr\n");
    return nfa;
}

static
bytecode_ptr<NFA> buildLbrShuf(const CharReach &cr, const depth &repeatMin,
                               const depth &repeatMax, u32 minPeriod,
                               bool is_reset, ReportID report) {
    enum RepeatType rtype = chooseRepeatType(repeatMin, repeatMax, minPeriod,
                                             is_reset);
    auto nfa = makeLbrNfa<lbr_shuf>(LBR_NFA_SHUF, rtype, repeatMax);
    struct lbr_shuf *ls = (struct lbr_shuf *)getMutableImplNfa(nfa.get());

    fillNfa<lbr_shuf>(nfa.get(), &ls->common, report, repeatMin, repeatMax,
                      minPeriod, rtype);

    if (shuftiBuildMasks(~cr, (u8 *)&ls->mask_lo, (u8 *)&ls->mask_hi) == -1) {
        return nullptr;
    }

    DEBUG_PRINTF("built shuf lbr\n");
    return nfa;
}

static
bytecode_ptr<NFA> buildLbrTruf(const CharReach &cr, const depth &repeatMin,
                               const depth &repeatMax, u32 minPeriod,
                               bool is_reset, ReportID report) {
    enum RepeatType rtype = chooseRepeatType(repeatMin, repeatMax, minPeriod,
                                             is_reset);
    auto nfa = makeLbrNfa<lbr_truf>(LBR_NFA_TRUF, rtype, repeatMax);
    struct lbr_truf *lc = (struct lbr_truf *)getMutableImplNfa(nfa.get());

    fillNfa<lbr_truf>(nfa.get(), &lc->common, report, repeatMin, repeatMax,
                      minPeriod, rtype);

    truffleBuildMasks(~cr, (u8 *)&lc->mask1, (u8 *)&lc->mask2);

    DEBUG_PRINTF("built truffle lbr\n");
    return nfa;
}

static
bytecode_ptr<NFA> constructLBR(const CharReach &cr, const depth &repeatMin,
                               const depth &repeatMax, u32 minPeriod,
                               bool is_reset, ReportID report) {
    DEBUG_PRINTF("bounds={%s,%s}, cr=%s (count %zu), report=%u\n",
                 repeatMin.str().c_str(), repeatMax.str().c_str(),
                 describeClass(cr, 20, CC_OUT_TEXT).c_str(), cr.count(),
                 report);
    assert(repeatMin <= repeatMax);
    assert(repeatMax.is_reachable());

    auto nfa =
        buildLbrDot(cr, repeatMin, repeatMax, minPeriod, is_reset, report);

    if (!nfa) {
        nfa = buildLbrVerm(cr, repeatMin, repeatMax, minPeriod, is_reset,
                           report);
    }
    if (!nfa) {
        nfa = buildLbrNVerm(cr, repeatMin, repeatMax, minPeriod, is_reset,
                            report);
    }
    if (!nfa) {
        nfa = buildLbrShuf(cr, repeatMin, repeatMax, minPeriod, is_reset,
                           report);
    }
    if (!nfa) {
        nfa = buildLbrTruf(cr, repeatMin, repeatMax, minPeriod, is_reset,
                           report);
    }

    if (!nfa) {
        assert(0);
        return nullptr;
    }

    return nfa;
}

bytecode_ptr<NFA> constructLBR(const CastleProto &proto,
                               const vector<vector<CharReach>> &triggers,
                               const CompileContext &cc,
                               const ReportManager &rm) {
    if (!cc.grey.allowLbr) {
        return nullptr;
    }

    if (proto.repeats.size() != 1) {
        return nullptr;
    }

    const PureRepeat &repeat = proto.repeats.begin()->second;
    assert(!repeat.reach.none());

    if (repeat.reports.size() != 1) {
        DEBUG_PRINTF("too many reports\n");
        return nullptr;
    }

    bool is_reset;
    u32 min_period = minPeriod(triggers, repeat.reach, &is_reset);

    if (depth(min_period) > repeat.bounds.max) {
        DEBUG_PRINTF("trigger is longer than repeat; only need one offset\n");
        is_reset = true;
    }

    ReportID report = *repeat.reports.begin();
    if (has_managed_reports(proto.kind)) {
        report = rm.getProgramOffset(report);
    }

    DEBUG_PRINTF("building LBR %s\n", repeat.bounds.str().c_str());
    return constructLBR(repeat.reach, repeat.bounds.min, repeat.bounds.max,
                        min_period, is_reset, report);
}

/** \brief Construct an LBR engine from the given graph \p g. */
bytecode_ptr<NFA> constructLBR(const NGHolder &g,
                               const vector<vector<CharReach>> &triggers,
                               const CompileContext &cc,
                               const ReportManager &rm) {
    if (!cc.grey.allowLbr) {
        return nullptr;
    }

    PureRepeat repeat;
    if (!isPureRepeat(g, repeat)) {
        return nullptr;
    }
    if (repeat.reports.size() != 1) {
        DEBUG_PRINTF("too many reports\n");
        return nullptr;
    }

    CastleProto proto(g.kind, repeat);
    return constructLBR(proto, triggers, cc, rm);
}

} // namespace ue2
