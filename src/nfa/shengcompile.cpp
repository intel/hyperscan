/*
 * Copyright (c) 2016-2020, Intel Corporation
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

#include "shengcompile.h"

#include "accel.h"
#include "accelcompile.h"
#include "shufticompile.h"
#include "trufflecompile.h"
#include "util/alloc.h"
#include "util/bitutils.h"
#include "util/charreach.h"
#include "util/compare.h"
#include "util/container.h"
#include "util/order_check.h"
#include "util/report_manager.h"
#include "util/unaligned.h"

#include "grey.h"
#include "nfa_internal.h"
#include "sheng_internal.h"
#include "ue2common.h"
#include "util/compile_context.h"
#include "util/make_unique.h"
#include "util/verify_types.h"
#include "util/simd_types.h"

#include <map>
#include <vector>
#include <sstream>

#include <boost/range/adaptor/map.hpp>

using namespace std;
using boost::adaptors::map_keys;

namespace ue2 {

#define ACCEL_DFA_MAX_OFFSET_DEPTH 4

/** Maximum tolerated number of escape character from an accel state.
 * This is larger than nfa, as we don't have a budget and the nfa cheats on stop
 * characters for sets of states */
#define ACCEL_DFA_MAX_STOP_CHAR 160

/** Maximum tolerated number of escape character from a sds accel state. Larger
 * than normal states as accelerating sds is important. Matches NFA value */
#define ACCEL_DFA_MAX_FLOATING_STOP_CHAR 192

struct dfa_info {
    accel_dfa_build_strat &strat;
    raw_dfa &raw;
    vector<dstate> &states;
    dstate &floating;
    dstate &anchored;
    bool can_die;

    explicit dfa_info(accel_dfa_build_strat &s)
        : strat(s), raw(strat.get_raw()), states(raw.states),
          floating(states[raw.start_floating]),
          anchored(states[raw.start_anchored]), can_die(dfaCanDie(raw)) {}

    // returns adjusted size
    size_t size() const {
        return can_die ? states.size() : states.size() - 1;
    }
    // expects adjusted index
    dstate &operator[](dstate_id_t idx) {
        return states[raw_id(idx)];
    }
    dstate &top(dstate_id_t idx) {
        if (isDead(idx)) {
            return floating;
        }
        return next(idx, TOP);
    }
    dstate &next(dstate_id_t idx, u16 chr) {
        auto &src = (*this)[idx];
        auto next_id = src.next[raw.alpha_remap[chr]];
        return states[next_id];
    }
    // get original idx from adjusted idx
    dstate_id_t raw_id(dstate_id_t idx) {
        assert(idx < size());
        // if DFA can't die, shift all indices left by 1
        return can_die ? idx : idx + 1;
    }
    bool isDead(dstate &state) {
        return raw_id(state.impl_id) == DEAD_STATE;
    }
    bool isDead(dstate_id_t idx) {
        return raw_id(idx) == DEAD_STATE;
    }

private:
    static bool dfaCanDie(raw_dfa &rdfa) {
        for (unsigned chr = 0; chr < 256; chr++) {
            for (dstate_id_t state = 0; state < rdfa.states.size(); state++) {
                auto succ = rdfa.states[state].next[rdfa.alpha_remap[chr]];
                if (succ == DEAD_STATE) {
                    return true;
                }
            }
        }
        return false;
    }
};

namespace {

struct raw_report_list {
    flat_set<ReportID> reports;

    raw_report_list(const flat_set<ReportID> &reports_in,
                    const ReportManager &rm, bool do_remap) {
        if (do_remap) {
            for (auto &id : reports_in) {
                reports.insert(rm.getProgramOffset(id));
            }
        } else {
            reports = reports_in;
        }
    }

    bool operator<(const raw_report_list &b) const {
        return reports < b.reports;
    }
};

struct raw_report_info_impl : public raw_report_info {
    vector<raw_report_list> rl;
    u32 getReportListSize() const override;
    size_t size() const override;
    void fillReportLists(NFA *n, size_t base_offset,
                         std::vector<u32> &ro /* out */) const override;
};
}

u32 raw_report_info_impl::getReportListSize() const {
    u32 rv = 0;

    for (const auto &reps : rl) {
        rv += sizeof(report_list);
        rv += sizeof(ReportID) * reps.reports.size();
    }

    return rv;
}

size_t raw_report_info_impl::size() const {
    return rl.size();
}

void raw_report_info_impl::fillReportLists(NFA *n, size_t base_offset,
                                           vector<u32> &ro) const {
    for (const auto &reps : rl) {
        ro.push_back(base_offset);

        report_list *p = (report_list *)((char *)n + base_offset);

        u32 i = 0;
        for (const ReportID report : reps.reports) {
            p->report[i++] = report;
        }
        p->count = verify_u32(reps.reports.size());

        base_offset += sizeof(report_list);
        base_offset += sizeof(ReportID) * reps.reports.size();
    }
}

unique_ptr<raw_report_info> sheng_build_strat::gatherReports(
                                                  vector<u32> &reports,
                                                  vector<u32> &reports_eod,
                                                  u8 *isSingleReport,
                                                  ReportID *arbReport) const {
    DEBUG_PRINTF("gathering reports\n");

    const bool remap_reports = has_managed_reports(rdfa.kind);

    auto ri = ue2::make_unique<raw_report_info_impl>();
    map<raw_report_list, u32> rev;

    for (const dstate &s : rdfa.states) {
        if (s.reports.empty()) {
            reports.push_back(MO_INVALID_IDX);
            continue;
        }

        raw_report_list rrl(s.reports, rm, remap_reports);
        DEBUG_PRINTF("non empty r\n");
        if (rev.find(rrl) != rev.end()) {
            reports.push_back(rev[rrl]);
        } else {
            DEBUG_PRINTF("adding to rl %zu\n", ri->size());
            rev[rrl] = ri->size();
            reports.push_back(ri->size());
            ri->rl.push_back(rrl);
        }
    }

    for (const dstate &s : rdfa.states) {
        if (s.reports_eod.empty()) {
            reports_eod.push_back(MO_INVALID_IDX);
            continue;
        }

        DEBUG_PRINTF("non empty r eod\n");
        raw_report_list rrl(s.reports_eod, rm, remap_reports);
        if (rev.find(rrl) != rev.end()) {
            reports_eod.push_back(rev[rrl]);
            continue;
        }

        DEBUG_PRINTF("adding to rl eod %zu\n", s.reports_eod.size());
        rev[rrl] = ri->size();
        reports_eod.push_back(ri->size());
        ri->rl.push_back(rrl);
    }

    assert(!ri->rl.empty()); /* all components should be able to generate
                                reports */
    if (!ri->rl.empty()) {
        *arbReport = *ri->rl.begin()->reports.begin();
    } else {
        *arbReport = 0;
    }

    /* if we have only a single report id generated from all accepts (not eod)
     * we can take some short cuts */
    set<ReportID> reps;

    for (u32 rl_index : reports) {
        if (rl_index == MO_INVALID_IDX) {
            continue;
        }
        assert(rl_index < ri->size());
        insert(&reps, ri->rl[rl_index].reports);
    }

    if (reps.size() == 1) {
        *isSingleReport = 1;
        *arbReport = *reps.begin();
        DEBUG_PRINTF("single -- %u\n", *arbReport);
    } else {
        *isSingleReport = 0;
    }

    return move(ri);
}

u32 sheng_build_strat::max_allowed_offset_accel() const {
    return ACCEL_DFA_MAX_OFFSET_DEPTH;
}

u32 sheng_build_strat::max_stop_char() const {
    return ACCEL_DFA_MAX_STOP_CHAR;
}

u32 sheng_build_strat::max_floating_stop_char() const {
    return ACCEL_DFA_MAX_FLOATING_STOP_CHAR;
}

size_t sheng_build_strat::accelSize() const {
    return sizeof(AccelAux);
}

#ifdef DEBUG
static really_inline
void dumpShuffleMask(const u8 chr, const u8 *buf, unsigned sz) {
    stringstream o;

    for (unsigned i = 0; i < sz; i++) {
        o.width(2);
        o << (buf[i] & SHENG_STATE_MASK) << " ";
    }
    DEBUG_PRINTF("chr %3u: %s\n", chr, o.str().c_str());
}

static really_inline
void dumpShuffleMask32(const u8 chr, const u8 *buf, unsigned sz) {
    stringstream o;

    for (unsigned i = 0; i < sz; i++) {
        o.width(2);
        o << (buf[i] & SHENG32_STATE_MASK) << " ";
    }
    DEBUG_PRINTF("chr %3u: %s\n", chr, o.str().c_str());
}

static really_inline
void dumpShuffleMask64(const u8 chr, const u8 *buf, unsigned sz) {
    stringstream o;

    for (unsigned i = 0; i < sz; i++) {
        o.width(2);
        o << (buf[i] & SHENG64_STATE_MASK) << " ";
    }
    DEBUG_PRINTF("chr %3u: %s\n", chr, o.str().c_str());
}
#endif

static
void fillAccelOut(const map<dstate_id_t, AccelScheme> &accel_escape_info,
                  set<dstate_id_t> *accel_states) {
    for (dstate_id_t i : accel_escape_info | map_keys) {
        accel_states->insert(i);
    }
}

template <typename T>
static
u8 getShengState(UNUSED dstate &state, UNUSED dfa_info &info,
                 UNUSED map<dstate_id_t, AccelScheme> &accelInfo) {
    return 0;
}

template <>
u8 getShengState<sheng>(dstate &state, dfa_info &info,
                        map<dstate_id_t, AccelScheme> &accelInfo) {
    u8 s = state.impl_id;
    if (!state.reports.empty()) {
        s |= SHENG_STATE_ACCEPT;
    }
    if (info.isDead(state)) {
        s |= SHENG_STATE_DEAD;
    }
    if (accelInfo.find(info.raw_id(state.impl_id)) != accelInfo.end()) {
        s |= SHENG_STATE_ACCEL;
    }
    return s;
}

template <>
u8 getShengState<sheng32>(dstate &state, dfa_info &info,
                          map<dstate_id_t, AccelScheme> &accelInfo) {
    u8 s = state.impl_id;
    if (!state.reports.empty()) {
        s |= SHENG32_STATE_ACCEPT;
    }
    if (info.isDead(state)) {
        s |= SHENG32_STATE_DEAD;
    }
    if (accelInfo.find(info.raw_id(state.impl_id)) != accelInfo.end()) {
        s |= SHENG32_STATE_ACCEL;
    }
    return s;
}

template <>
u8 getShengState<sheng64>(dstate &state, dfa_info &info,
                          UNUSED map<dstate_id_t, AccelScheme> &accelInfo) {
    u8 s = state.impl_id;
    if (!state.reports.empty()) {
        s |= SHENG64_STATE_ACCEPT;
    }
    if (info.isDead(state)) {
        s |= SHENG64_STATE_DEAD;
    }
    return s;
}

template <typename T>
static
void fillAccelAux(struct NFA *n, dfa_info &info,
                  map<dstate_id_t, AccelScheme> &accelInfo) {
    DEBUG_PRINTF("Filling accel aux structures\n");
    T *s = (T *)getMutableImplNfa(n);
    u32 offset = s->accel_offset;

    for (dstate_id_t i = 0; i < info.size(); i++) {
        dstate_id_t state_id = info.raw_id(i);
        if (accelInfo.find(state_id) != accelInfo.end()) {
            s->flags |= SHENG_FLAG_HAS_ACCEL;
            AccelAux *aux = (AccelAux *)((char *)n + offset);
            info.strat.buildAccel(state_id, accelInfo[state_id], aux);
            sstate_aux *saux =
                (sstate_aux *)((char *)n + s->aux_offset) + state_id;
            saux->accel = offset;
            DEBUG_PRINTF("Accel offset: %u\n", offset);
            offset += ROUNDUP_N(sizeof(AccelAux), alignof(AccelAux));
        }
    }
}

template <typename T>
static
void populateBasicInfo(UNUSED struct NFA *n, UNUSED dfa_info &info,
                       UNUSED map<dstate_id_t, AccelScheme> &accelInfo,
                       UNUSED u32 aux_offset, UNUSED u32 report_offset,
                       UNUSED u32 accel_offset, UNUSED u32 total_size,
                       UNUSED u32 dfa_size) {
}

template <>
void populateBasicInfo<sheng>(struct NFA *n, dfa_info &info,
                              map<dstate_id_t, AccelScheme> &accelInfo,
                              u32 aux_offset, u32 report_offset,
                              u32 accel_offset, u32 total_size,
                              u32 dfa_size) {
    n->length = total_size;
    n->scratchStateSize = 1;
    n->streamStateSize = 1;
    n->nPositions = info.size();
    n->type = SHENG_NFA;
    n->flags |= info.raw.hasEodReports() ? NFA_ACCEPTS_EOD : 0;

    sheng *s = (sheng *)getMutableImplNfa(n);
    s->aux_offset = aux_offset;
    s->report_offset = report_offset;
    s->accel_offset = accel_offset;
    s->n_states = info.size();
    s->length = dfa_size;
    s->flags |= info.can_die ? SHENG_FLAG_CAN_DIE : 0;

    s->anchored = getShengState<sheng>(info.anchored, info, accelInfo);
    s->floating = getShengState<sheng>(info.floating, info, accelInfo);
}

template <>
void populateBasicInfo<sheng32>(struct NFA *n, dfa_info &info,
                                map<dstate_id_t, AccelScheme> &accelInfo,
                                u32 aux_offset, u32 report_offset,
                                u32 accel_offset, u32 total_size,
                                u32 dfa_size) {
    n->length = total_size;
    n->scratchStateSize = 1;
    n->streamStateSize = 1;
    n->nPositions = info.size();
    n->type = SHENG_NFA_32;
    n->flags |= info.raw.hasEodReports() ? NFA_ACCEPTS_EOD : 0;

    sheng32 *s = (sheng32 *)getMutableImplNfa(n);
    s->aux_offset = aux_offset;
    s->report_offset = report_offset;
    s->accel_offset = accel_offset;
    s->n_states = info.size();
    s->length = dfa_size;
    s->flags |= info.can_die ? SHENG_FLAG_CAN_DIE : 0;

    s->anchored = getShengState<sheng32>(info.anchored, info, accelInfo);
    s->floating = getShengState<sheng32>(info.floating, info, accelInfo);
}

template <>
void populateBasicInfo<sheng64>(struct NFA *n, dfa_info &info,
                                map<dstate_id_t, AccelScheme> &accelInfo,
                                u32 aux_offset, u32 report_offset,
                                u32 accel_offset, u32 total_size,
                                u32 dfa_size) {
    n->length = total_size;
    n->scratchStateSize = 1;
    n->streamStateSize = 1;
    n->nPositions = info.size();
    n->type = SHENG_NFA_64;
    n->flags |= info.raw.hasEodReports() ? NFA_ACCEPTS_EOD : 0;

    sheng64 *s = (sheng64 *)getMutableImplNfa(n);
    s->aux_offset = aux_offset;
    s->report_offset = report_offset;
    s->accel_offset = accel_offset;
    s->n_states = info.size();
    s->length = dfa_size;
    s->flags |= info.can_die ? SHENG_FLAG_CAN_DIE : 0;

    s->anchored = getShengState<sheng64>(info.anchored, info, accelInfo);
    s->floating = getShengState<sheng64>(info.floating, info, accelInfo);
}

template <typename T>
static
void fillTops(NFA *n, dfa_info &info, dstate_id_t id,
              map<dstate_id_t, AccelScheme> &accelInfo) {
    T *s = (T *)getMutableImplNfa(n);
    u32 aux_base = s->aux_offset;

    DEBUG_PRINTF("Filling tops for state %u\n", id);

    sstate_aux *aux = (sstate_aux *)((char *)n + aux_base) + id;

    DEBUG_PRINTF("Aux structure for state %u, offset %zd\n", id,
                 (char *)aux - (char *)n);

    /* we could conceivably end up in an accept/dead state on a top event,
     * so mark top as accept/dead state if it indeed is.
     */
    auto &top_state = info.top(id);

    DEBUG_PRINTF("Top transition for state %u: %u\n", id, top_state.impl_id);

    aux->top = getShengState<T>(top_state, info, accelInfo);
}

template <typename T>
static
void fillAux(NFA *n, dfa_info &info, dstate_id_t id, vector<u32> &reports,
                 vector<u32> &reports_eod, vector<u32> &report_offsets) {
    T *s = (T *)getMutableImplNfa(n);
    u32 aux_base = s->aux_offset;
    auto raw_id = info.raw_id(id);

    auto &state = info[id];

    sstate_aux *aux = (sstate_aux *)((char *)n + aux_base) + id;

    DEBUG_PRINTF("Filling aux and report structures for state %u\n", id);
    DEBUG_PRINTF("Aux structure for state %u, offset %zd\n", id,
                 (char *)aux - (char *)n);

    aux->accept = state.reports.empty() ? 0 : report_offsets[reports[raw_id]];
    aux->accept_eod =
        state.reports_eod.empty() ? 0 : report_offsets[reports_eod[raw_id]];

    DEBUG_PRINTF("Report list offset: %u\n", aux->accept);
    DEBUG_PRINTF("EOD report list offset: %u\n", aux->accept_eod);
}

template <typename T>
static
void fillSingleReport(NFA *n, ReportID r_id) {
    T *s = (T *)getMutableImplNfa(n);

    DEBUG_PRINTF("Single report ID: %u\n", r_id);
    s->report = r_id;
    s->flags |= SHENG_FLAG_SINGLE_REPORT;
}

template <typename T>
static
bool createShuffleMasks(UNUSED T *s, UNUSED dfa_info &info,
                        UNUSED map<dstate_id_t, AccelScheme> &accelInfo) {
    return true;
}

template <>
bool createShuffleMasks<sheng>(sheng *s, dfa_info &info,
                               map<dstate_id_t, AccelScheme> &accelInfo) {
    for (u16 chr = 0; chr < 256; chr++) {
        u8 buf[16] = {0};

        for (dstate_id_t idx = 0; idx < info.size(); idx++) {
            auto &succ_state = info.next(idx, chr);

            buf[idx] = getShengState<sheng>(succ_state, info, accelInfo);
        }
#ifdef DEBUG
        dumpShuffleMask(chr, buf, sizeof(buf));
#endif
        memcpy(&s->shuffle_masks[chr], buf, sizeof(m128));
    }
    return true;
}

template <>
bool createShuffleMasks<sheng32>(sheng32 *s, dfa_info &info,
                                 map<dstate_id_t, AccelScheme> &accelInfo) {
    for (u16 chr = 0; chr < 256; chr++) {
        u8 buf[64] = {0};

        assert(info.size() <= 32);
        for (dstate_id_t idx = 0; idx < info.size(); idx++) {
            auto &succ_state = info.next(idx, chr);

            buf[idx] = getShengState<sheng32>(succ_state, info, accelInfo);
            buf[32 + idx] = buf[idx];
        }
#ifdef DEBUG
        dumpShuffleMask32(chr, buf, sizeof(buf));
#endif
        memcpy(&s->succ_masks[chr], buf, sizeof(m512));
    }
    return true;
}

template <>
bool createShuffleMasks<sheng64>(sheng64 *s, dfa_info &info,
                                 map<dstate_id_t, AccelScheme> &accelInfo) {
    for (u16 chr = 0; chr < 256; chr++) {
        u8 buf[64] = {0};

        assert(info.size() <= 64);
        for (dstate_id_t idx = 0; idx < info.size(); idx++) {
            auto &succ_state = info.next(idx, chr);

            if (accelInfo.find(info.raw_id(succ_state.impl_id))
                != accelInfo.end()) {
                return false;
            }
            buf[idx] = getShengState<sheng64>(succ_state, info, accelInfo);
        }
#ifdef DEBUG
        dumpShuffleMask64(chr, buf, sizeof(buf));
#endif
        memcpy(&s->succ_masks[chr], buf, sizeof(m512));
    }
    return true;
}

bool has_accel_sheng(const NFA *) {
    return true; /* consider the sheng region as accelerated */
}

template <typename T>
static
bytecode_ptr<NFA> shengCompile_int(raw_dfa &raw, const CompileContext &cc,
                                   set<dstate_id_t> *accel_states,
                                   sheng_build_strat &strat,
                                   dfa_info &info) {
    if (!cc.streaming) { /* TODO: work out if we can do the strip in streaming
                          * mode with our semantics */
        raw.stripExtraEodReports();
    }
    auto accelInfo = strat.getAccelInfo(cc.grey);

    // set impl_id of each dfa state
    for (dstate_id_t i = 0; i < info.size(); i++) {
        info[i].impl_id = i;
    }

    DEBUG_PRINTF("Anchored start state: %u, floating start state: %u\n",
                 info.anchored.impl_id, info.floating.impl_id);

    u32 nfa_size = ROUNDUP_16(sizeof(NFA) + sizeof(T));
    vector<u32> reports, eod_reports, report_offsets;
    u8 isSingle = 0;
    ReportID single_report = 0;

    auto ri =
        strat.gatherReports(reports, eod_reports, &isSingle, &single_report);

    u32 total_aux = sizeof(sstate_aux) * info.size();
    u32 total_accel = strat.accelSize() * accelInfo.size();
    u32 total_reports = ri->getReportListSize();

    u32 reports_offset = nfa_size + total_aux;
    u32 accel_offset =
        ROUNDUP_N(reports_offset + total_reports, alignof(AccelAux));
    u32 total_size = ROUNDUP_N(accel_offset + total_accel, 64);

    DEBUG_PRINTF("NFA: %u, aux: %u, reports: %u, accel: %u, total: %u\n",
                 nfa_size, total_aux, total_reports, total_accel, total_size);

    auto nfa = make_zeroed_bytecode_ptr<NFA>(total_size);

    populateBasicInfo<T>(nfa.get(), info, accelInfo, nfa_size,
                             reports_offset, accel_offset, total_size,
                             total_size - sizeof(NFA));

    DEBUG_PRINTF("Setting up aux and report structures\n");

    ri->fillReportLists(nfa.get(), reports_offset, report_offsets);

    for (dstate_id_t idx = 0; idx < info.size(); idx++) {
        fillTops<T>(nfa.get(), info, idx, accelInfo);
        fillAux<T>(nfa.get(), info, idx, reports, eod_reports,
                       report_offsets);
    }
    if (isSingle) {
        fillSingleReport<T>(nfa.get(), single_report);
    }

    fillAccelAux<T>(nfa.get(), info, accelInfo);

    if (accel_states) {
        fillAccelOut(accelInfo, accel_states);
    }

    if (!createShuffleMasks<T>((T *)getMutableImplNfa(nfa.get()), info, accelInfo)) {
        return nullptr;
    }

    return nfa;
}

bytecode_ptr<NFA> shengCompile(raw_dfa &raw, const CompileContext &cc,
                               const ReportManager &rm, bool only_accel_init,
                               set<dstate_id_t> *accel_states) {
    if (!cc.grey.allowSheng) {
        DEBUG_PRINTF("Sheng is not allowed!\n");
        return nullptr;
    }

    sheng_build_strat strat(raw, rm, only_accel_init);
    dfa_info info(strat);

    DEBUG_PRINTF("Trying to compile a %zu state Sheng\n", raw.states.size());

    DEBUG_PRINTF("Anchored start state id: %u, floating start state id: %u\n",
                 raw.start_anchored, raw.start_floating);

    DEBUG_PRINTF("This DFA %s die so effective number of states is %zu\n",
                 info.can_die ? "can" : "cannot", info.size());
    if (info.size() > 16) {
        DEBUG_PRINTF("Too many states\n");
        return nullptr;
    }

    return shengCompile_int<sheng>(raw, cc, accel_states, strat, info);
}

bytecode_ptr<NFA> sheng32Compile(raw_dfa &raw, const CompileContext &cc,
                                 const ReportManager &rm, bool only_accel_init,
                                 set<dstate_id_t> *accel_states) {
    if (!cc.grey.allowSheng) {
        DEBUG_PRINTF("Sheng is not allowed!\n");
        return nullptr;
    }

    if (!cc.target_info.has_avx512vbmi()) {
        DEBUG_PRINTF("Sheng32 failed, no HS_CPU_FEATURES_AVX512VBMI!\n");
        return nullptr;
    }

    sheng_build_strat strat(raw, rm, only_accel_init);
    dfa_info info(strat);

    DEBUG_PRINTF("Trying to compile a %zu state Sheng\n", raw.states.size());

    DEBUG_PRINTF("Anchored start state id: %u, floating start state id: %u\n",
                 raw.start_anchored, raw.start_floating);

    DEBUG_PRINTF("This DFA %s die so effective number of states is %zu\n",
                 info.can_die ? "can" : "cannot", info.size());
    assert(info.size() > 16);
    if (info.size() > 32) {
        DEBUG_PRINTF("Too many states\n");
        return nullptr;
    }

    return shengCompile_int<sheng32>(raw, cc, accel_states, strat, info);
}

bytecode_ptr<NFA> sheng64Compile(raw_dfa &raw, const CompileContext &cc,
                                 const ReportManager &rm, bool only_accel_init,
                                 set<dstate_id_t> *accel_states) {
    if (!cc.grey.allowSheng) {
        DEBUG_PRINTF("Sheng is not allowed!\n");
        return nullptr;
    }

    if (!cc.target_info.has_avx512vbmi()) {
        DEBUG_PRINTF("Sheng64 failed, no HS_CPU_FEATURES_AVX512VBMI!\n");
        return nullptr;
    }

    sheng_build_strat strat(raw, rm, only_accel_init);
    dfa_info info(strat);

    DEBUG_PRINTF("Trying to compile a %zu state Sheng\n", raw.states.size());

    DEBUG_PRINTF("Anchored start state id: %u, floating start state id: %u\n",
                 raw.start_anchored, raw.start_floating);

    DEBUG_PRINTF("This DFA %s die so effective number of states is %zu\n",
                 info.can_die ? "can" : "cannot", info.size());
    assert(info.size() > 32);
    if (info.size() > 64) {
        DEBUG_PRINTF("Too many states\n");
        return nullptr;
    }
    vector<dstate> old_states;
    old_states = info.states;
    auto nfa = shengCompile_int<sheng64>(raw, cc, accel_states, strat, info);
    if (!nfa) {
        info.states = old_states;
    }
    return nfa;
}

} // namespace ue2
