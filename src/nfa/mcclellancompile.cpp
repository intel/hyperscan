/*
 * Copyright (c) 2015-2018, Intel Corporation
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

#include "mcclellancompile.h"

#include "accel.h"
#include "accelcompile.h"
#include "grey.h"
#include "mcclellan_internal.h"
#include "mcclellancompile_util.h"
#include "nfa_internal.h"
#include "shufticompile.h"
#include "trufflecompile.h"
#include "ue2common.h"
#include "util/alloc.h"
#include "util/bitutils.h"
#include "util/charreach.h"
#include "util/compare.h"
#include "util/compile_context.h"
#include "util/container.h"
#include "util/make_unique.h"
#include "util/order_check.h"
#include "util/report_manager.h"
#include "util/flat_containers.h"
#include "util/unaligned.h"
#include "util/verify_types.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <queue>
#include <set>
#include <vector>

#include <boost/range/adaptor/map.hpp>

#include "mcclellandump.h"
#include "util/dump_util.h"
#include "util/dump_charclass.h"

using namespace std;
using boost::adaptors::map_keys;
using boost::dynamic_bitset;

#define ACCEL_DFA_MAX_OFFSET_DEPTH 4

/** Maximum tolerated number of escape character from an accel state.
 * This is larger than nfa, as we don't have a budget and the nfa cheats on stop
 * characters for sets of states */
#define ACCEL_DFA_MAX_STOP_CHAR 160

/** Maximum tolerated number of escape character from a sds accel state. Larger
 * than normal states as accelerating sds is important. Matches NFA value */
#define ACCEL_DFA_MAX_FLOATING_STOP_CHAR 192

namespace ue2 {

namespace /* anon */ {

struct dstate_extra {
    u16 daddytaken = 0;
    bool shermanState = false;
    bool wideState = false;
    bool wideHead = false;
};

struct dfa_info {
    accel_dfa_build_strat &strat;
    raw_dfa &raw;
    vector<dstate> &states;
    vector<dstate_extra> extra;
    vector<vector<dstate_id_t>> wide_state_chain;
    vector<vector<symbol_t>> wide_symbol_chain;
    const u16 alpha_size; /* including special symbols */
    const array<u16, ALPHABET_SIZE> &alpha_remap;
    const u16 impl_alpha_size;

    u8 getAlphaShift() const;

    explicit dfa_info(accel_dfa_build_strat &s)
                                : strat(s),
                                  raw(s.get_raw()),
                                  states(raw.states),
                                  extra(raw.states.size()),
                                  alpha_size(raw.alpha_size),
                                  alpha_remap(raw.alpha_remap),
                                  impl_alpha_size(raw.getImplAlphaSize()) {}

    dstate_id_t implId(dstate_id_t raw_id) const {
        return states[raw_id].impl_id;
    }

    bool is_sherman(dstate_id_t raw_id) const {
        return extra[raw_id].shermanState;
    }

    bool is_widestate(dstate_id_t raw_id) const {
        return extra[raw_id].wideState;
    }

    bool is_widehead(dstate_id_t raw_id) const {
        return extra[raw_id].wideHead;
    }

    size_t size(void) const { return states.size(); }
};

u8 dfa_info::getAlphaShift() const {
    if (impl_alpha_size < 2) {
        return 1;
    } else {
        /* log2 round up */
        return 32 - clz32(impl_alpha_size - 1);
    }
}

struct state_prev_info {
    vector<vector<dstate_id_t>> prev_vec;
    explicit state_prev_info(size_t alpha_size) : prev_vec(alpha_size) {}
};

struct DfaPrevInfo {
    u16 impl_alpha_size;
    u16 state_num;
    vector<state_prev_info> states;
    set<dstate_id_t> accepts;

    explicit DfaPrevInfo(raw_dfa &rdfa);
};

DfaPrevInfo::DfaPrevInfo(raw_dfa &rdfa)
    : impl_alpha_size(rdfa.getImplAlphaSize()), state_num(rdfa.states.size()),
      states(state_num, state_prev_info(impl_alpha_size)){
    for (size_t i = 0; i < states.size(); i++) {
        for (symbol_t sym = 0; sym < impl_alpha_size; sym++) {
            dstate_id_t curr = rdfa.states[i].next[sym];
            states[curr].prev_vec[sym].push_back(i);
        }
        if (!rdfa.states[i].reports.empty()
            || !rdfa.states[i].reports_eod.empty()) {
            DEBUG_PRINTF("accept raw state: %ld\n", i);
            accepts.insert(i);
        }
    }
}
} // namespace

static
mstate_aux *getAux(NFA *n, dstate_id_t i) {
    assert(isMcClellanType(n->type));

    mcclellan *m = (mcclellan *)getMutableImplNfa(n);
    mstate_aux *aux_base = (mstate_aux *)((char *)n + m->aux_offset);

    mstate_aux *aux = aux_base + i;
    assert((const char *)aux < (const char *)n + m->length);
    return aux;
}

static
void markEdges(NFA *n, u16 *succ_table, const dfa_info &info) {
    assert((size_t)succ_table % 2 == 0);
    assert(n->type == MCCLELLAN_NFA_16);
    u8 alphaShift = info.getAlphaShift();
    u16 alphaSize = info.impl_alpha_size;
    mcclellan *m = (mcclellan *)getMutableImplNfa(n);

    /* handle the normal states */
    for (u32 i = 0; i < m->sherman_limit; i++) {
        for (size_t j = 0; j < alphaSize; j++) {
            size_t c_prime = (i << alphaShift) + j;

            // wide state has no aux structure.
            if (m->has_wide && succ_table[c_prime] >= m->wide_limit) {
                continue;
            }

            mstate_aux *aux = getAux(n, succ_table[c_prime]);

            if (aux->accept) {
                succ_table[c_prime] |= ACCEPT_FLAG;
            }

            if (aux->accel_offset) {
                succ_table[c_prime] |= ACCEL_FLAG;
            }
        }
    }

    /* handle the sherman states */
    char *sherman_base_offset = (char *)n + m->sherman_offset;
    u16 sherman_ceil = m->has_wide == 1 ? m->wide_limit : m->state_count;
    for (u16 j = m->sherman_limit; j < sherman_ceil; j++) {
        char *sherman_cur
            = findMutableShermanState(sherman_base_offset, m->sherman_limit, j);
        assert(*(sherman_cur + SHERMAN_TYPE_OFFSET) == SHERMAN_STATE);
        u8 len = *(u8 *)(sherman_cur + SHERMAN_LEN_OFFSET);
        u16 *succs = (u16 *)(sherman_cur + SHERMAN_STATES_OFFSET(len));

        for (u8 i = 0; i < len; i++) {
            u16 succ_i = unaligned_load_u16((u8 *)&succs[i]);
            // wide state has no aux structure.
            if (m->has_wide && succ_i >= m->wide_limit) {
                continue;
            }

            mstate_aux *aux = getAux(n, succ_i);

            if (aux->accept) {
                succ_i |= ACCEPT_FLAG;
            }

            if (aux->accel_offset) {
                succ_i |= ACCEL_FLAG;
            }

            unaligned_store_u16((u8 *)&succs[i], succ_i);
        }
    }

    /* handle the wide states */
    if (m->has_wide) {
        u32 wide_limit = m->wide_limit;
        char *wide_base = (char *)n + m->wide_offset;
        assert(*wide_base == WIDE_STATE);
        u16 wide_number = verify_u16(info.wide_symbol_chain.size());
        // traverse over wide head states.
        for (u16 j = wide_limit; j < wide_limit + wide_number; j++) {
            char *wide_cur
                = findMutableWideEntry16(wide_base, wide_limit, j);
            u16 width = *(const u16 *)(wide_cur + WIDE_WIDTH_OFFSET);
            u16 *trans = (u16 *)(wide_cur + WIDE_TRANSITION_OFFSET16(width));

            // check successful transition
            u16 next = unaligned_load_u16((u8 *)trans);
            if (next < wide_limit) {
                mstate_aux *aux = getAux(n, next);
                if (aux->accept) {
                    next |= ACCEPT_FLAG;
                }
                if (aux->accel_offset) {
                    next |= ACCEL_FLAG;
                }
                unaligned_store_u16((u8 *)trans, next);
            }
            trans++;

            // check failure transition
            for (symbol_t k = 0; k < alphaSize; k++) {
                u16 next_k = unaligned_load_u16((u8 *)&trans[k]);
                if (next_k >= wide_limit) {
                    continue;
                }
                mstate_aux *aux_k = getAux(n, next_k);
                if (aux_k->accept) {
                    next_k |= ACCEPT_FLAG;
                }
                if (aux_k->accel_offset) {
                    next_k |= ACCEL_FLAG;
                }
                unaligned_store_u16((u8 *)&trans[k], next_k);
            }
        }
    }
}

u32 mcclellan_build_strat::max_allowed_offset_accel() const {
    return ACCEL_DFA_MAX_OFFSET_DEPTH;
}

u32 mcclellan_build_strat::max_stop_char() const {
    return ACCEL_DFA_MAX_STOP_CHAR;
}

u32 mcclellan_build_strat::max_floating_stop_char() const {
    return ACCEL_DFA_MAX_FLOATING_STOP_CHAR;
}

static
void populateBasicInfo(size_t state_size, const dfa_info &info,
                       u32 total_size, u32 aux_offset, u32 accel_offset,
                       u32 accel_count, ReportID arb, bool single, NFA *nfa) {
    assert(state_size == sizeof(u16) || state_size == sizeof(u8));

    nfa->length = total_size;
    nfa->nPositions = info.states.size();

    nfa->scratchStateSize = verify_u32(state_size);
    nfa->streamStateSize = verify_u32(state_size);

    if (state_size == sizeof(u8)) {
        nfa->type = MCCLELLAN_NFA_8;
    } else {
        nfa->type = MCCLELLAN_NFA_16;
    }

    mcclellan *m = (mcclellan *)getMutableImplNfa(nfa);
    for (u32 i = 0; i < 256; i++) {
        m->remap[i] = verify_u8(info.alpha_remap[i]);
    }
    m->alphaShift = info.getAlphaShift();
    m->length = total_size;
    m->aux_offset = aux_offset;
    m->accel_offset = accel_offset;
    m->arb_report = arb;
    m->state_count = verify_u16(info.size());
    m->start_anchored = info.implId(info.raw.start_anchored);
    m->start_floating = info.implId(info.raw.start_floating);
    m->has_accel = accel_count ? 1 : 0;
    m->has_wide = info.wide_state_chain.size() > 0 ? 1 : 0;

    if (state_size == sizeof(u8) && m->has_wide == 1) {
        // allocate 1 more byte for wide state use.
        nfa->scratchStateSize += sizeof(u8);
        nfa->streamStateSize += sizeof(u8);
    }

    if (state_size == sizeof(u16) && m->has_wide == 1) {
        // allocate 2 more bytes for wide state use.
        nfa->scratchStateSize += sizeof(u16);
        nfa->streamStateSize += sizeof(u16);
    }

    if (single) {
        m->flags |= MCCLELLAN_FLAG_SINGLE;
    }
}

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

unique_ptr<raw_report_info> mcclellan_build_strat::gatherReports(
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
        auto it = rev.find(rrl);
        if (it != rev.end()) {
            reports.push_back(it->second);
        } else {
            DEBUG_PRINTF("adding to rl %zu\n", ri->size());
            rev.emplace(rrl, ri->size());
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
        auto it = rev.find(rrl);
        if (it != rev.end()) {
            reports_eod.push_back(it->second);
            continue;
        }

        DEBUG_PRINTF("adding to rl eod %zu\n", s.reports_eod.size());
        rev.emplace(rrl, ri->size());
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
    flat_set<ReportID> reps;

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
        DEBUG_PRINTF("single -- %u\n",  *arbReport);
    } else {
        *isSingleReport = 0;
    }

    return move(ri);
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

static
void fillAccelOut(const map<dstate_id_t, AccelScheme> &accel_escape_info,
                  set<dstate_id_t> *accel_states) {
    for (dstate_id_t i : accel_escape_info | map_keys) {
        accel_states->insert(i);
    }
}

static
size_t calcShermanRegionSize(const dfa_info &info) {
    size_t rv = 0;

    for (size_t i = 0; i < info.size(); i++) {
        if (info.is_sherman(i)) {
            rv += SHERMAN_FIXED_SIZE;
        }
    }

    return ROUNDUP_16(rv);
}

static
size_t calcWideRegionSize(const dfa_info &info) {
    if (info.wide_state_chain.empty()) {
        return 0;
    }

    // wide info header
    size_t rv = info.wide_symbol_chain.size() * sizeof(u32) + 4;

    // wide info body
    for (const auto &chain : info.wide_symbol_chain) {
        rv += ROUNDUP_N(chain.size(), 2) +
              (info.impl_alpha_size + 1) * sizeof(u16) + 2;
    }

    return ROUNDUP_16(rv);
}

static
void fillInAux(mstate_aux *aux, dstate_id_t i, const dfa_info &info,
               const vector<u32> &reports, const vector<u32> &reports_eod,
               vector<u32> &reportOffsets) {
    const dstate &raw_state = info.states[i];
    aux->accept = raw_state.reports.empty() ? 0 : reportOffsets[reports[i]];
    aux->accept_eod = raw_state.reports_eod.empty() ? 0
                                              : reportOffsets[reports_eod[i]];
    aux->top = info.implId(i ? raw_state.next[info.alpha_remap[TOP]]
                             : info.raw.start_floating);
}

/* returns false on error */
static
bool allocateFSN16(dfa_info &info, dstate_id_t *sherman_base,
                   dstate_id_t *wide_limit) {
    info.states[0].impl_id = 0; /* dead is always 0 */

    vector<dstate_id_t> norm;
    vector<dstate_id_t> sherm;
    vector<dstate_id_t> wideHead;
    vector<dstate_id_t> wideState;

    if (info.size() > (1 << 16)) {
        DEBUG_PRINTF("too many states\n");
        *wide_limit = 0;
        return false;
    }

    for (u32 i = 1; i < info.size(); i++) {
        if (info.is_widehead(i)) {
            wideHead.push_back(i);
        } else if (info.is_widestate(i)) {
            wideState.push_back(i);
        } else if (info.is_sherman(i)) {
            sherm.push_back(i);
        } else {
            norm.push_back(i);
        }
    }

    dstate_id_t next = 1;
    for (const dstate_id_t &s : norm) {
        DEBUG_PRINTF("[norm] mapping state %u to %u\n", s, next);
        info.states[s].impl_id = next++;
    }

    *sherman_base = next;
    for (const dstate_id_t &s : sherm) {
        DEBUG_PRINTF("[sherm] mapping state %u to %u\n", s, next);
        info.states[s].impl_id = next++;
    }

    *wide_limit = next;
    for (const dstate_id_t &s : wideHead) {
        DEBUG_PRINTF("[widehead] mapping state %u to %u\n", s, next);
        info.states[s].impl_id = next++;
    }

    for (const dstate_id_t &s : wideState) {
        DEBUG_PRINTF("[wide] mapping state %u to %u\n", s, next);
        info.states[s].impl_id = next++;
    }

    /* Check to see if we haven't over allocated our states */
    DEBUG_PRINTF("next sherman %u masked %u\n", next,
                 (dstate_id_t)(next & STATE_MASK));
    return (next - 1) == ((next - 1) & STATE_MASK);
}

static
bytecode_ptr<NFA> mcclellanCompile16(dfa_info &info, const CompileContext &cc,
                                     set<dstate_id_t> *accel_states) {
    DEBUG_PRINTF("building mcclellan 16\n");

    vector<u32> reports; /* index in ri for the appropriate report list */
    vector<u32> reports_eod; /* as above */
    ReportID arb;
    u8 single;

    u8 alphaShift = info.getAlphaShift();
    assert(alphaShift <= 8);

    u16 count_real_states;
    u16 wide_limit;
    if (!allocateFSN16(info, &count_real_states, &wide_limit)) {
        DEBUG_PRINTF("failed to allocate state numbers, %zu states total\n",
                     info.size());
        return nullptr;
    }

    DEBUG_PRINTF("count_real_states: %d\n", count_real_states);
    DEBUG_PRINTF("non_wide_states: %d\n", wide_limit);

    auto ri = info.strat.gatherReports(reports, reports_eod, &single, &arb);
    map<dstate_id_t, AccelScheme> accel_escape_info
            = info.strat.getAccelInfo(cc.grey);

    size_t tran_size = (1 << info.getAlphaShift())
        * sizeof(u16) * count_real_states;

    size_t aux_size = sizeof(mstate_aux) * wide_limit;

    size_t aux_offset = ROUNDUP_16(sizeof(NFA) + sizeof(mcclellan) + tran_size);
    size_t accel_size = info.strat.accelSize() * accel_escape_info.size();
    size_t accel_offset = ROUNDUP_N(aux_offset + aux_size
                                    + ri->getReportListSize(), 32);
    size_t sherman_offset = ROUNDUP_16(accel_offset + accel_size);
    size_t sherman_size = calcShermanRegionSize(info);
    size_t wide_offset = ROUNDUP_16(sherman_offset + sherman_size);
    size_t wide_size = calcWideRegionSize(info);
    size_t total_size = wide_offset + wide_size;

    accel_offset -= sizeof(NFA); /* adj accel offset to be relative to m */
    assert(ISALIGNED_N(accel_offset, alignof(union AccelAux)));

    DEBUG_PRINTF("aux_offset %zu\n", aux_offset);
    DEBUG_PRINTF("aux_size %zu\n", aux_size);
    DEBUG_PRINTF("rl size %u\n", ri->getReportListSize());
    DEBUG_PRINTF("accel_offset %zu\n", accel_offset + sizeof(NFA));
    DEBUG_PRINTF("accel_size %zu\n", accel_size);
    DEBUG_PRINTF("sherman_offset %zu\n", sherman_offset);
    DEBUG_PRINTF("sherman_size %zu\n", sherman_size);
    DEBUG_PRINTF("wide_offset %zu\n", wide_offset);
    DEBUG_PRINTF("wide_size %zu\n", wide_size);
    DEBUG_PRINTF("total_size %zu\n", total_size);

    auto nfa = make_zeroed_bytecode_ptr<NFA>(total_size);
    char *nfa_base = (char *)nfa.get();

    populateBasicInfo(sizeof(u16), info, total_size, aux_offset, accel_offset,
                      accel_escape_info.size(), arb, single, nfa.get());

    vector<u32> reportOffsets;

    ri->fillReportLists(nfa.get(), aux_offset + aux_size, reportOffsets);

    u16 *succ_table = (u16 *)(nfa_base + sizeof(NFA) + sizeof(mcclellan));
    mstate_aux *aux = (mstate_aux *)(nfa_base + aux_offset);
    mcclellan *m = (mcclellan *)getMutableImplNfa(nfa.get());

    m->wide_limit = wide_limit;
    m->wide_offset = wide_offset;

    /* copy in the mc header information */
    m->sherman_offset = sherman_offset;
    m->sherman_end = total_size;
    m->sherman_limit = count_real_states;

    /* do normal states */
    for (size_t i = 0; i < info.size(); i++) {
        if (info.is_sherman(i) || info.is_widestate(i)) {
            continue;
        }

        u16 fs = info.implId(i);
        mstate_aux *this_aux = getAux(nfa.get(), fs);

        assert(fs < count_real_states);

        for (size_t j = 0; j < info.impl_alpha_size; j++) {
            succ_table[(fs << alphaShift) + j] =
                info.implId(info.states[i].next[j]);
        }

        fillInAux(&aux[fs], i, info, reports, reports_eod, reportOffsets);

        if (contains(accel_escape_info, i)) {
            this_aux->accel_offset = accel_offset;
            accel_offset += info.strat.accelSize();
            assert(accel_offset + sizeof(NFA) <= sherman_offset);
            assert(ISALIGNED_N(accel_offset, alignof(union AccelAux)));
            info.strat.buildAccel(i, accel_escape_info.at(i),
                                  (void *)((char *)m + this_aux->accel_offset));
        }
    }

    /* do sherman states */
    char *sherman_table = nfa_base + m->sherman_offset;
    assert(ISALIGNED_16(sherman_table));
    for (size_t i = 0; i < info.size(); i++) {
        if (!info.is_sherman(i)) {
            continue;
        }

        u16 fs = verify_u16(info.implId(i));
        mstate_aux *this_aux = getAux(nfa.get(), fs);

        assert(fs >= count_real_states);
        assert(fs < wide_limit);

        char *curr_sherman_entry
            = sherman_table + (fs - m->sherman_limit) * SHERMAN_FIXED_SIZE;
        assert(curr_sherman_entry <= nfa_base + m->length);

        fillInAux(this_aux, i, info, reports, reports_eod, reportOffsets);

        if (contains(accel_escape_info, i)) {
            this_aux->accel_offset = accel_offset;
            accel_offset += info.strat.accelSize();
            assert(accel_offset + sizeof(NFA) <= sherman_offset);
            assert(ISALIGNED_N(accel_offset, alignof(union AccelAux)));
            info.strat.buildAccel(i, accel_escape_info.at(i),
                                  (void *)((char *)m + this_aux->accel_offset));
        }

        u8 len = verify_u8(info.impl_alpha_size - info.extra[i].daddytaken);
        assert(len <= 9);
        dstate_id_t d = info.states[i].daddy;

        *(u8 *)(curr_sherman_entry + SHERMAN_TYPE_OFFSET) = SHERMAN_STATE;
        *(u8 *)(curr_sherman_entry + SHERMAN_LEN_OFFSET) = len;
        *(u16 *)(curr_sherman_entry + SHERMAN_DADDY_OFFSET) = info.implId(d);
        u8 *chars = (u8 *)(curr_sherman_entry + SHERMAN_CHARS_OFFSET);

        for (u16 s = 0; s < info.impl_alpha_size; s++) {
            if (info.states[i].next[s] != info.states[d].next[s]) {
                *(chars++) = (u8)s;
            }
        }

        u16 *states = (u16 *)(curr_sherman_entry + SHERMAN_STATES_OFFSET(len));
        for (u16 s = 0; s < info.impl_alpha_size; s++) {
            if (info.states[i].next[s] != info.states[d].next[s]) {
                DEBUG_PRINTF("s overrider %hu dad %hu char next %hu\n",
                             fs, info.implId(d),
                             info.implId(info.states[i].next[s]));
                unaligned_store_u16((u8 *)states++,
                                    info.implId(info.states[i].next[s]));
            }
        }
    }

    if (!info.wide_state_chain.empty()) {
        /* do wide states using info */
        u16 wide_number = verify_u16(info.wide_symbol_chain.size());
        char *wide_base = nfa_base + m->wide_offset;
        assert(ISALIGNED_16(wide_base));

        char *wide_top = wide_base;
        *(u8 *)(wide_top++) = WIDE_STATE;
        wide_top = ROUNDUP_PTR(wide_top, 2);
        *(u16 *)(wide_top) = wide_number;
        wide_top += 2;

        char *curr_wide_entry = wide_top + wide_number * sizeof(u32);
        u32 *wide_offset_list = (u32 *)wide_top;

        /* get the order of writing wide states */
        vector<size_t> order(wide_number);
        for (size_t i = 0; i < wide_number; i++) {
            dstate_id_t head = info.wide_state_chain[i].front();
            size_t pos = info.implId(head) - m->wide_limit;
            order[pos] = i;
        }

        for (size_t i : order) {
            vector<dstate_id_t> &state_chain = info.wide_state_chain[i];
            vector<symbol_t> &symbol_chain = info.wide_symbol_chain[i];

            u16 width = verify_u16(symbol_chain.size());
            *(u16 *)(curr_wide_entry + WIDE_WIDTH_OFFSET) = width;
            u8 *chars = (u8 *)(curr_wide_entry + WIDE_SYMBOL_OFFSET16);

            // store wide state symbol chain
            for (size_t j = 0; j < width; j++) {
                *(chars++) = verify_u8(symbol_chain[j]);
            }

            // store wide state transition table
            u16 *trans = (u16 *)(curr_wide_entry
                                + WIDE_TRANSITION_OFFSET16(width));
            dstate_id_t tail = state_chain[width - 1];
            symbol_t last = symbol_chain[width -1];
            dstate_id_t tran = info.states[tail].next[last];
            // 1. successful transition
            *trans++ = info.implId(tran);
            // 2. failure transition
            for (size_t j = 0; verify_u16(j) < width - 1; j++) {
                if (symbol_chain[j] != last) {
                    tran = info.states[state_chain[j]].next[last];
                }
            }
            for (symbol_t sym = 0; sym < info.impl_alpha_size; sym++) {
                if (sym != last) {
                    *trans++ = info.implId(info.states[tail].next[sym]);
                }
                else {
                    *trans++ = info.implId(tran);
                }
            }

            *wide_offset_list++ = verify_u32(curr_wide_entry - wide_base);

            curr_wide_entry = (char *)trans;
        }
    }

    markEdges(nfa.get(), succ_table, info);

    if (accel_states && nfa) {
        fillAccelOut(accel_escape_info, accel_states);
    }

    return nfa;
}

static
void fillInBasicState8(const dfa_info &info, mstate_aux *aux, u8 *succ_table,
                       const vector<u32> &reportOffsets,
                       const vector<u32> &reports,
                       const vector<u32> &reports_eod, u32 i) {
    dstate_id_t j = info.implId(i);
    u8 alphaShift = info.getAlphaShift();
    assert(alphaShift <= 8);

    for (size_t s = 0; s < info.impl_alpha_size; s++) {
        dstate_id_t raw_succ = info.states[i].next[s];
        succ_table[(j << alphaShift) + s] = info.implId(raw_succ);
    }

    aux[j].accept = 0;
    aux[j].accept_eod = 0;

    if (!info.states[i].reports.empty()) {
        DEBUG_PRINTF("i=%u r[i]=%u\n", i, reports[i]);
        assert(reports[i] != MO_INVALID_IDX);
        aux[j].accept = reportOffsets[reports[i]];
    }

    if (!info.states[i].reports_eod.empty()) {
        DEBUG_PRINTF("i=%u re[i]=%u\n", i, reports_eod[i]);
        aux[j].accept_eod = reportOffsets[reports_eod[i]];
    }

    dstate_id_t raw_top = i ? info.states[i].next[info.alpha_remap[TOP]]
                            : info.raw.start_floating;

    aux[j].top = info.implId(raw_top);
}

static
void allocateFSN8(dfa_info &info,
                  const map<dstate_id_t, AccelScheme> &accel_escape_info,
                  u16 *accel_limit, u16 *accept_limit) {
    info.states[0].impl_id = 0; /* dead is always 0 */

    vector<dstate_id_t> norm;
    vector<dstate_id_t> accel;
    vector<dstate_id_t> accept;

    assert(info.size() <= (1 << 8));

    for (u32 i = 1; i < info.size(); i++) {
        if (!info.states[i].reports.empty()) {
            accept.push_back(i);
        } else if (contains(accel_escape_info, i)) {
            accel.push_back(i);
        } else {
            norm.push_back(i);
        }
    }

    u32 j = 1; /* dead is already at 0 */
    for (const dstate_id_t &s : norm) {
        assert(j <= 256);
        DEBUG_PRINTF("mapping state %u to %u\n", s, j);
        info.states[s].impl_id = j++;
    }
    *accel_limit = j;
    for (const dstate_id_t &s : accel) {
        assert(j <= 256);
        DEBUG_PRINTF("mapping state %u to %u\n", s, j);
        info.states[s].impl_id = j++;
    }
    *accept_limit = j;
    for (const dstate_id_t &s : accept) {
        assert(j <= 256);
        DEBUG_PRINTF("mapping state %u to %u\n",  s, j);
        info.states[s].impl_id = j++;
    }
}

static
bytecode_ptr<NFA> mcclellanCompile8(dfa_info &info, const CompileContext &cc,
                                    set<dstate_id_t> *accel_states) {
    DEBUG_PRINTF("building mcclellan 8\n");

    vector<u32> reports;
    vector<u32> reports_eod;
    ReportID arb;
    u8 single;

    auto ri = info.strat.gatherReports(reports, reports_eod, &single, &arb);
    map<dstate_id_t, AccelScheme> accel_escape_info
        = info.strat.getAccelInfo(cc.grey);

    size_t tran_size = sizeof(u8) * (1 << info.getAlphaShift()) * info.size();
    size_t aux_size = sizeof(mstate_aux) * info.size();
    size_t aux_offset = ROUNDUP_16(sizeof(NFA) + sizeof(mcclellan) + tran_size);
    size_t accel_size = info.strat.accelSize() * accel_escape_info.size();
    size_t accel_offset = ROUNDUP_N(aux_offset + aux_size
                                     + ri->getReportListSize(), 32);
    size_t total_size = accel_offset + accel_size;

    DEBUG_PRINTF("aux_size %zu\n", aux_size);
    DEBUG_PRINTF("aux_offset %zu\n", aux_offset);
    DEBUG_PRINTF("rl size %u\n", ri->getReportListSize());
    DEBUG_PRINTF("accel_size %zu\n", accel_size);
    DEBUG_PRINTF("accel_offset %zu\n", accel_offset);
    DEBUG_PRINTF("total_size %zu\n", total_size);

    accel_offset -= sizeof(NFA); /* adj accel offset to be relative to m */
    assert(ISALIGNED_N(accel_offset, alignof(union AccelAux)));

    auto nfa = make_zeroed_bytecode_ptr<NFA>(total_size);
    char *nfa_base = (char *)nfa.get();

    mcclellan *m = (mcclellan *)getMutableImplNfa(nfa.get());

    allocateFSN8(info, accel_escape_info, &m->accel_limit_8,
                 &m->accept_limit_8);
    populateBasicInfo(sizeof(u8), info, total_size, aux_offset, accel_offset,
                      accel_escape_info.size(), arb, single, nfa.get());

    vector<u32> reportOffsets;

    ri->fillReportLists(nfa.get(), aux_offset + aux_size, reportOffsets);

    /* copy in the state information */
    u8 *succ_table = (u8 *)(nfa_base + sizeof(NFA) + sizeof(mcclellan));
    mstate_aux *aux = (mstate_aux *)(nfa_base + aux_offset);

    for (size_t i = 0; i < info.size(); i++) {
        if (contains(accel_escape_info, i)) {
            u32 j = info.implId(i);

            aux[j].accel_offset = accel_offset;
            accel_offset += info.strat.accelSize();

            info.strat.buildAccel(i, accel_escape_info.at(i),
                                  (void *)((char *)m + aux[j].accel_offset));
        }

        fillInBasicState8(info, aux, succ_table, reportOffsets, reports,
                          reports_eod, i);
    }

    assert(accel_offset + sizeof(NFA) <= total_size);

    DEBUG_PRINTF("rl size %zu\n", ri->size());

    if (accel_states && nfa) {
        fillAccelOut(accel_escape_info, accel_states);
    }

    return nfa;
}

#define MAX_SHERMAN_LIST_LEN 9

static
void addIfEarlier(flat_set<dstate_id_t> &dest, dstate_id_t candidate,
                  dstate_id_t max) {
    if (candidate < max) {
        dest.insert(candidate);
    }
}

static
void addSuccessors(flat_set<dstate_id_t> &dest, const dstate &source,
                   u16 alphasize, dstate_id_t curr_id) {
    for (symbol_t s = 0; s < alphasize; s++) {
        addIfEarlier(dest, source.next[s], curr_id);
    }
}

/* \brief Returns a set of states to search for a better daddy. */
static
flat_set<dstate_id_t> find_daddy_candidates(const dfa_info &info,
                                            dstate_id_t curr_id) {
    flat_set<dstate_id_t> hinted;

    addIfEarlier(hinted, 0, curr_id);
    addIfEarlier(hinted, info.raw.start_anchored, curr_id);
    addIfEarlier(hinted, info.raw.start_floating, curr_id);

    // Add existing daddy and his successors, then search back one generation.
    const u16 alphasize = info.impl_alpha_size;
    dstate_id_t daddy = info.states[curr_id].daddy;
    for (u32 level = 0; daddy && level < 2; level++) {
        addIfEarlier(hinted, daddy, curr_id);
        addSuccessors(hinted, info.states[daddy], alphasize, curr_id);
        daddy = info.states[daddy].daddy;
    }

    return hinted;
}

#define MAX_SHERMAN_SELF_LOOP 20

static
void find_better_daddy(dfa_info &info, dstate_id_t curr_id, bool using8bit,
                       bool any_cyclic_near_anchored_state,
                       bool trust_daddy_states, const Grey &grey) {
    if (!grey.allowShermanStates) {
        return;
    }

    const u16 width = using8bit ? sizeof(u8) : sizeof(u16);
    const u16 alphasize = info.impl_alpha_size;

    if (info.raw.start_anchored != DEAD_STATE
        && any_cyclic_near_anchored_state
        && curr_id < alphasize * 3) {
        /* crude attempt to prevent frequent states from being sherman'ed
         * depends on the fact that states are numbers are currently in bfs
         * order */
        DEBUG_PRINTF("%hu is banned\n", curr_id);
        return;
    }

    if (info.raw.start_floating != DEAD_STATE
        && curr_id >= info.raw.start_floating
        && curr_id < info.raw.start_floating + alphasize * 3) {
        /* crude attempt to prevent frequent states from being sherman'ed
         * depends on the fact that states are numbers are currently in bfs
         * order */
        DEBUG_PRINTF("%hu is banned (%hu)\n", curr_id, info.raw.start_floating);
        return;
    }

    const u16 full_state_size = width * alphasize;
    const u16 max_list_len = MIN(MAX_SHERMAN_LIST_LEN,
                           (full_state_size - 2)/(width + 1));
    u16 best_score = 0;
    dstate_id_t best_daddy = 0;
    dstate &currState = info.states[curr_id];

    flat_set<dstate_id_t> hinted;
    if (trust_daddy_states) {
        // Use the daddy already set for this state so long as it isn't already
        // a Sherman state.
        dstate_id_t daddy = currState.daddy;
        if (!info.is_sherman(daddy) && !info.is_widestate(daddy)) {
            hinted.insert(currState.daddy);
        } else {
            // Fall back to granddaddy, which has already been processed (due
            // to BFS ordering) and cannot be a Sherman state.
            dstate_id_t granddaddy = info.states[currState.daddy].daddy;
            if (info.is_widestate(granddaddy)) {
                return;
            }
            assert(!info.is_sherman(granddaddy));
            hinted.insert(granddaddy);
        }
    } else {
        hinted = find_daddy_candidates(info, curr_id);
    }

    for (const dstate_id_t &donor : hinted) {
        assert(donor < curr_id);
        u32 score = 0;

        if (info.is_sherman(donor) || info.is_widestate(donor)) {
            continue;
        }

        const dstate &donorState = info.states[donor];
        for (symbol_t s = 0; s < alphasize; s++) {
            if (currState.next[s] == donorState.next[s]) {
                score++;
            }
        }

        /* prefer lower ids to provide some stability amongst potential
         * siblings */
        if (score > best_score || (score == best_score && donor < best_daddy)) {
            best_daddy = donor;
            best_score = score;

            if (score == alphasize) {
                break;
            }
        }
    }

    currState.daddy = best_daddy;
    info.extra[curr_id].daddytaken = best_score;
    DEBUG_PRINTF("%hu -> daddy %hu: %u/%u BF\n", curr_id, best_daddy,
                 best_score, alphasize);

    if (best_score + max_list_len < alphasize) {
        return; /* ??? */
    }

    if (info.is_sherman(currState.daddy)) {
        return;
    }

    u32 self_loop_width = 0;
    const dstate &curr_raw = info.states[curr_id];
    for (unsigned i = 0; i < N_CHARS; i++) {
        if (curr_raw.next[info.alpha_remap[i]] == curr_id) {
            self_loop_width++;
        }
    }

    if (self_loop_width > MAX_SHERMAN_SELF_LOOP) {
        DEBUG_PRINTF("%hu is banned wide self loop (%u)\n", curr_id,
                     self_loop_width);
        return;
    }

    DEBUG_PRINTF("%hu is sherman\n", curr_id);
    info.extra[curr_id].shermanState = true;
}

static
bool is_cyclic_near(const raw_dfa &raw, dstate_id_t root) {
    symbol_t alphasize = raw.getImplAlphaSize();
    for (symbol_t s = 0; s < alphasize; s++) {
        dstate_id_t succ_id = raw.states[root].next[s];
        if (succ_id == DEAD_STATE) {
            continue;
        }

        const dstate &succ = raw.states[succ_id];
        for (symbol_t t = 0; t < alphasize; t++) {
            if (succ.next[t] == root || succ.next[t] == succ_id) {
                return true;
            }
        }
    }
    return false;
}

/* \brief Test for only-one-predecessor property. */
static
bool check_property1(const DfaPrevInfo &info, const u16 impl_alpha_size,
                     const dstate_id_t curr_id, dstate_id_t &prev_id,
                     symbol_t &prev_sym) {
    u32 num_prev = 0;
    bool test_p1 = false;

    for (symbol_t sym = 0; sym < impl_alpha_size; sym++) {
        num_prev += info.states[curr_id].prev_vec[sym].size();
        DEBUG_PRINTF("Check symbol: %u, with its vector size: %lu\n", sym,
                     info.states[curr_id].prev_vec[sym].size());
        if (num_prev == 1 && !test_p1) {
            test_p1 = true;
            prev_id = info.states[curr_id].prev_vec[sym].front(); //[0] for sure???
            prev_sym = sym;
        }
    }

    return num_prev == 1;
}

/* \brief Test for same-failure-action property. */
static
bool check_property2(const raw_dfa &rdfa, const u16 impl_alpha_size,
                     const dstate_id_t curr_id, const dstate_id_t prev_id,
                     const symbol_t curr_sym, const symbol_t prev_sym) {
    const dstate &prevState = rdfa.states[prev_id];
    const dstate &currState = rdfa.states[curr_id];

    // Compare transition tables between currState and prevState.
    u16 score = 0;
    for (symbol_t sym = 0; sym < impl_alpha_size; sym++) {
        if (currState.next[sym] == prevState.next[sym]
            && sym != curr_sym && sym != prev_sym) {
            score++;
        }
    }
    DEBUG_PRINTF("(Score: %u/%u)\n", score, impl_alpha_size);

    // 2 cases.
    if (curr_sym != prev_sym && score >= impl_alpha_size - 2
        && currState.next[prev_sym] == prevState.next[curr_sym]) {
        return true;
    } else if (curr_sym == prev_sym && score == impl_alpha_size - 1) {
        return true;
    }
    return false;
}

/* \brief Check whether adding current prev_id will generate a circle.*/
static
bool check_circle(const DfaPrevInfo &info, const u16 impl_alpha_size,
                  const vector<dstate_id_t> &chain, const dstate_id_t id) {
    const vector<vector<dstate_id_t>> &prev_vec = info.states[id].prev_vec;
    const dstate_id_t tail = chain.front();
    for (symbol_t sym = 0; sym < impl_alpha_size; sym++) {
        auto iter = find(prev_vec[sym].begin(), prev_vec[sym].end(), tail);
        if (iter != prev_vec[sym].end()) {
            // Tail is one of id's predecessors, forming a circle.
            return true;
        }
    }
    return false;
}

/* \brief Returns a chain of state ids and symbols. */
static
dstate_id_t find_chain_candidate(const raw_dfa &rdfa, const DfaPrevInfo &info,
                                 const dstate_id_t curr_id,
                                 const symbol_t curr_sym,
                                 vector<dstate_id_t> &temp_chain) {
    //Record current id first.
    temp_chain.push_back(curr_id);

    const u16 size = info.impl_alpha_size;

    // Stop when entering root cloud.
    if (rdfa.start_anchored != DEAD_STATE
        && is_cyclic_near(rdfa, rdfa.start_anchored)
        && curr_id < size) {
       return curr_id;
    }
    if (rdfa.start_floating != DEAD_STATE
        && curr_id >= rdfa.start_floating
        && curr_id < rdfa.start_floating + size * 3) {
        return curr_id;
    }

    // Stop when reaching anchored or floating.
    if (curr_id == rdfa.start_anchored || curr_id == rdfa.start_floating) {
        return curr_id;
    }

    dstate_id_t prev_id = 0;
    symbol_t prev_sym = ALPHABET_SIZE;

    // Check the only-one-predecessor property.
    if (!check_property1(info, size, curr_id, prev_id, prev_sym)) {
        return curr_id;
    }
    assert(prev_id != 0 && prev_sym != ALPHABET_SIZE);
    DEBUG_PRINTF("(P1 test passed.)\n");

    // Circle testing for the prev_id that passes the P1 test.
    if (check_circle(info, size, temp_chain, prev_id)) {
        DEBUG_PRINTF("(A circle is found.)\n");
        return curr_id;
    }

    // Check the same-failure-action property.
    if (!check_property2(rdfa, size, curr_id, prev_id, curr_sym, prev_sym)) {
        return curr_id;
    }
    DEBUG_PRINTF("(P2 test passed.)\n");

    if (!rdfa.states[prev_id].reports.empty()
        || !rdfa.states[prev_id].reports_eod.empty()) {
        return curr_id;
    } else {
        return find_chain_candidate(rdfa, info, prev_id, prev_sym, temp_chain);
    }
}

/* \brief Always store the non-extensible chains found till now. */
static
bool store_chain_longest(vector<vector<dstate_id_t>> &candidate_chain,
                         vector<dstate_id_t> &temp_chain,
                         dynamic_bitset<> &added, bool head_is_new) {
    dstate_id_t head = temp_chain.front();
    u16 length = temp_chain.size();

    if (head_is_new) {
        DEBUG_PRINTF("This is a new chain!\n");

        // Add this new chain and get it marked.
        candidate_chain.push_back(temp_chain);

        for (auto &id : temp_chain) {
            DEBUG_PRINTF("(Marking s%u ...)\n", id);
            added.set(id);
        }

        return true;
    }

    DEBUG_PRINTF("This is a longer chain!\n");
    assert(!candidate_chain.empty());

    auto chain = find_if(candidate_chain.begin(), candidate_chain.end(),
                         [&](const vector<dstate_id_t> &it) {
                            return it.front() == head;
                         });

    // Not a valid head, just do nothing and return.
    if (chain == candidate_chain.end()) {
        return false;
    }

    u16 len = chain->size();

    if (length > len) {
        // Find out the branch node first.
        size_t piv = 0;
        for (; piv < length; piv++) {
            if ((*chain)[piv] != temp_chain[piv]) {
                break;
            }
        }

        for (size_t j = piv + 1; j < length; j++) {
            DEBUG_PRINTF("(Marking s%u (new branch) ...)\n", temp_chain[j]);
            added.set(temp_chain[j]);
        }

        // Unmark old unuseful nodes.
        // (Except the tail node, which is in working queue)
        for (size_t j = piv + 1; j < verify_u16(len - 1); j++) {
            DEBUG_PRINTF("(UnMarking s%u (old branch)...)\n", (*chain)[j]);
            added.reset((*chain)[j]);
        }

        chain->assign(temp_chain.begin(), temp_chain.end());
    }

    return false;
}

/* \brief Generate wide_symbol_chain from wide_state_chain. */
static
void generate_symbol_chain(dfa_info &info, vector<symbol_t> &chain_tail) {
    raw_dfa &rdfa = info.raw;
    assert(chain_tail.size() == info.wide_state_chain.size());

    for (size_t i = 0; i < info.wide_state_chain.size(); i++) {
        vector<dstate_id_t> &state_chain = info.wide_state_chain[i];
        vector<symbol_t> symbol_chain;

        info.extra[state_chain[0]].wideHead = true;
        size_t width = state_chain.size() - 1;

        for (size_t j = 0; j < width; j++) {
            dstate_id_t curr_id = state_chain[j];
            dstate_id_t next_id = state_chain[j + 1];

            // The last state of the chain doesn't belong to a wide state.
            info.extra[curr_id].wideState = true;

            // The tail symbol comes from vector chain_tail;
            if (j == width - 1) {
                symbol_chain.push_back(chain_tail[i]);
            } else {
                for (symbol_t sym = 0; sym < info.impl_alpha_size; sym++) {
                    if (rdfa.states[curr_id].next[sym] == next_id) {
                        symbol_chain.push_back(sym);
                        break;
                    }
                }
            }
        }

        info.wide_symbol_chain.push_back(symbol_chain);
    }
}

/* \brief Find potential regions of states to be packed into wide states. */
static
void find_wide_state(dfa_info &info) {
    DfaPrevInfo dinfo(info.raw);
    queue<dstate_id_t> work_queue;

    dynamic_bitset<> added(info.raw.states.size());
    for (auto it : dinfo.accepts) {
        work_queue.push(it);
        added.set(it);
    }

    vector<symbol_t> chain_tail;
    while (!work_queue.empty()) {
        dstate_id_t curr_id = work_queue.front();
        work_queue.pop();
        DEBUG_PRINTF("Newly popped state: s%u\n", curr_id);

        for (symbol_t sym = 0; sym < dinfo.impl_alpha_size; sym++) {
            for (auto info_it : dinfo.states[curr_id].prev_vec[sym]) {
                if (added.test(info_it)) {
                    DEBUG_PRINTF("(s%u already marked.)\n", info_it);
                    continue;
                }

                vector<dstate_id_t> temp_chain;
                // Head is a state failing the test of the chain.
                dstate_id_t head = find_chain_candidate(info.raw, dinfo,
                                                        info_it, sym,
                                                        temp_chain);

                // A candidate chain should contain 8 substates at least.
                if (temp_chain.size() < 8) {
                    DEBUG_PRINTF("(Not enough substates, continue.)\n");
                    continue;
                }

                bool head_is_new = !added.test(head);
                if (head_is_new) {
                    added.set(head);
                    work_queue.push(head);
                    DEBUG_PRINTF("Newly pushed state: s%u\n", head);
                }

                reverse(temp_chain.begin(), temp_chain.end());
                temp_chain.push_back(curr_id);

                assert(head > 0 && head == temp_chain.front());
                if (store_chain_longest(info.wide_state_chain, temp_chain,
                                        added, head_is_new)) {
                    chain_tail.push_back(sym);
                }
            }
        }
    }

    generate_symbol_chain(info, chain_tail);
}

bytecode_ptr<NFA> mcclellanCompile_i(raw_dfa &raw, accel_dfa_build_strat &strat,
                                     const CompileContext &cc,
                                     bool trust_daddy_states,
                                     set<dstate_id_t> *accel_states) {
    assert(!is_dead(raw));

    dfa_info info(strat);
    bool using8bit = cc.grey.allowMcClellan8 && info.size() <= 256;

    if (!cc.streaming) { /* TODO: work out if we can do the strip in streaming
                          * mode with our semantics */
        raw.stripExtraEodReports();
    }

    bool has_eod_reports = raw.hasEodReports();

    bytecode_ptr<NFA> nfa;
    if (!using8bit) {
        if (cc.grey.allowWideStates && strat.getType() == McClellan
            && !is_triggered(raw.kind)) {
            find_wide_state(info);
        }

        u16 total_daddy = 0;
        bool any_cyclic_near_anchored_state
            = is_cyclic_near(raw, raw.start_anchored);

        for (u32 i = 0; i < info.size(); i++) {
            if (info.is_widestate(i)) {
                continue;
            }
            find_better_daddy(info, i, using8bit,
                              any_cyclic_near_anchored_state,
                              trust_daddy_states, cc.grey);
            total_daddy += info.extra[i].daddytaken;
        }

        DEBUG_PRINTF("daddy %hu/%zu states=%zu alpha=%hu\n", total_daddy,
                     info.size() * info.impl_alpha_size, info.size(),
                     info.impl_alpha_size);

        nfa = mcclellanCompile16(info, cc, accel_states);
    } else {
        nfa = mcclellanCompile8(info, cc, accel_states);
    }

    if (has_eod_reports) {
        nfa->flags |= NFA_ACCEPTS_EOD;
    }

    DEBUG_PRINTF("compile done\n");
    return nfa;
}

bytecode_ptr<NFA> mcclellanCompile(raw_dfa &raw, const CompileContext &cc,
                                   const ReportManager &rm,
                                   bool only_accel_init,
                                   bool trust_daddy_states,
                                   set<dstate_id_t> *accel_states) {
    mcclellan_build_strat mbs(raw, rm, only_accel_init);
    return mcclellanCompile_i(raw, mbs, cc, trust_daddy_states, accel_states);
}

size_t mcclellan_build_strat::accelSize(void) const {
    return sizeof(AccelAux); /* McClellan accel structures are just bare
                              * accelaux */
}

u32 mcclellanStartReachSize(const raw_dfa *raw) {
    if (raw->states.size() < 2) {
        return 0;
    }

    const dstate &ds = raw->states[raw->start_anchored];

    CharReach out;
    for (unsigned i = 0; i < N_CHARS; i++) {
        if (ds.next[raw->alpha_remap[i]] != DEAD_STATE) {
            out.set(i);
        }
    }

    return out.count();
}

bool has_accel_mcclellan(const NFA *nfa) {
    const mcclellan *m = (const mcclellan *)getImplNfa(nfa);
    return m->has_accel;
}

} // namespace ue2
