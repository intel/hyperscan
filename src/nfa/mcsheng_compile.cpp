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

#include "mcsheng_compile.h"

#include "accel.h"
#include "accelcompile.h"
#include "grey.h"
#include "mcclellancompile.h"
#include "mcclellancompile_util.h"
#include "mcsheng_internal.h"
#include "nfa_internal.h"
#include "rdfa_graph.h"
#include "shufticompile.h"
#include "trufflecompile.h"
#include "ue2common.h"
#include "util/alloc.h"
#include "util/bitutils.h"
#include "util/charreach.h"
#include "util/compare.h"
#include "util/compile_context.h"
#include "util/container.h"
#include "util/flat_containers.h"
#include "util/graph.h"
#include "util/graph_range.h"
#include "util/make_unique.h"
#include "util/order_check.h"
#include "util/report_manager.h"
#include "util/unaligned.h"
#include "util/unordered.h"
#include "util/verify_types.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <set>
#include <deque>
#include <vector>
#include <boost/range/adaptor/map.hpp>

using namespace std;
using boost::adaptors::map_keys;

namespace ue2 {

namespace /* anon */ {

#define MIN_SHENG_SIZE 6
#define INVALID_SHENG_ID 255

struct dstate_extra {
    u16 daddytaken = 0;
    bool shermanState = false;
    bool sheng_succ = false;
    u8 sheng_id = INVALID_SHENG_ID;
};

struct dfa_info {
    accel_dfa_build_strat &strat;
    raw_dfa &raw;
    vector<dstate> &states;
    vector<dstate_extra> extra;
    const u16 alpha_size; /* including special symbols */
    const array<u16, ALPHABET_SIZE> &alpha_remap;
    vector<CharReach> rev_alpha;
    const u16 impl_alpha_size;

    u8 getAlphaShift() const;

    explicit dfa_info(accel_dfa_build_strat &s)
                                : strat(s),
                                  raw(s.get_raw()),
                                  states(raw.states),
                                  extra(raw.states.size()),
                                  alpha_size(raw.alpha_size),
                                  alpha_remap(raw.alpha_remap),
                                  impl_alpha_size(raw.getImplAlphaSize()) {
        rev_alpha.resize(impl_alpha_size);
        for (u32 i = 0; i < N_CHARS; i++) {
            rev_alpha[alpha_remap[i]].set(i);
        }
    }

    dstate_id_t implId(dstate_id_t raw_id) const {
        return states[raw_id].impl_id;
    }

    bool is_sherman(dstate_id_t raw_id) const {
        return extra[raw_id].shermanState;
    }

    bool is_sheng(dstate_id_t raw_id) const {
        return extra[raw_id].sheng_id != INVALID_SHENG_ID;
    }

    bool is_sheng_succ(dstate_id_t raw_id) const {
        return extra[raw_id].sheng_succ;
    }

    /* states which use the normal transition/successor table */
    bool is_normal(dstate_id_t raw_id) const {
        return raw_id != DEAD_STATE && !is_sheng(raw_id) && !is_sherman(raw_id);
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

} // namespace

static
mstate_aux *getAux(NFA *n, dstate_id_t i) {
    mcsheng *m = (mcsheng *)getMutableImplNfa(n);
    mstate_aux *aux_base = (mstate_aux *)((char *)n + m->aux_offset);

    mstate_aux *aux = aux_base + i;
    assert((const char *)aux < (const char *)n + m->length);
    return aux;
}

static
void createShuffleMasks(mcsheng *m, const dfa_info &info,
                       dstate_id_t sheng_end,
                       const map<dstate_id_t, AccelScheme> &accel_escape_info) {
    DEBUG_PRINTF("using first %hu states for a sheng\n", sheng_end);
    assert(sheng_end > DEAD_STATE + 1);
    assert(sheng_end <= sizeof(m128) + 1);
    vector<array<u8, sizeof(m128)>> masks;
    masks.resize(info.alpha_size);
    /* -1 to avoid wasting a slot as we do not include dead state */
    vector<dstate_id_t> raw_ids;
    raw_ids.resize(sheng_end - 1);
    for (dstate_id_t s = DEAD_STATE + 1; s < info.states.size(); s++) {
        assert(info.implId(s)); /* should not map to DEAD_STATE */
        if (info.is_sheng(s)) {
            raw_ids[info.extra[s].sheng_id] = s;
        }
    }
    for (u32 i = 0; i < info.alpha_size; i++) {
        if (i == info.alpha_remap[TOP]) {
            continue;
        }
        auto &mask = masks[i];
        assert(sizeof(mask) == sizeof(m128));
        mask.fill(0);

        for (dstate_id_t sheng_id = 0; sheng_id < sheng_end - 1; sheng_id++) {
            dstate_id_t raw_id = raw_ids[sheng_id];
            dstate_id_t next_id = info.implId(info.states[raw_id].next[i]);
            if (next_id == DEAD_STATE) {
                next_id = sheng_end - 1;
            } else if (next_id < sheng_end) {
                next_id--;
            }
            DEBUG_PRINTF("%hu: %u->next %hu\n", sheng_id, i, next_id);
            mask[sheng_id] = verify_u8(next_id);
        }
    }
    for (u32 i = 0; i < N_CHARS; i++) {
        assert(info.alpha_remap[i] != info.alpha_remap[TOP]);
        memcpy((u8 *)&m->sheng_masks[i],
               (u8 *)masks[info.alpha_remap[i]].data(), sizeof(m128));
    }
    m->sheng_end = sheng_end;
    m->sheng_accel_limit = sheng_end - 1;

    for (dstate_id_t s : raw_ids) {
        if (contains(accel_escape_info, s)) {
            LIMIT_TO_AT_MOST(&m->sheng_accel_limit, info.extra[s].sheng_id);
        }
    }
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
        nfa->type = MCSHENG_NFA_8;
    } else {
        nfa->type = MCSHENG_NFA_16;
    }

    mcsheng *m = (mcsheng *)getMutableImplNfa(nfa);
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

    if (single) {
        m->flags |= MCSHENG_FLAG_SINGLE;
    }
}

static
mstate_aux *getAux64(NFA *n, dstate_id_t i) {
    mcsheng64 *m = (mcsheng64 *)getMutableImplNfa(n);
    mstate_aux *aux_base = (mstate_aux *)((char *)n + m->aux_offset);

    mstate_aux *aux = aux_base + i;
    assert((const char *)aux < (const char *)n + m->length);
    return aux;
}

static
void createShuffleMasks64(mcsheng64 *m, const dfa_info &info,
                      dstate_id_t sheng_end,
                      const map<dstate_id_t, AccelScheme> &accel_escape_info) {
    DEBUG_PRINTF("using first %hu states for a sheng\n", sheng_end);
    assert(sheng_end > DEAD_STATE + 1);
    assert(sheng_end <= sizeof(m512) + 1);
    vector<array<u8, sizeof(m512)>> masks;
    masks.resize(info.alpha_size);
    /* -1 to avoid wasting a slot as we do not include dead state */
    vector<dstate_id_t> raw_ids;
    raw_ids.resize(sheng_end - 1);
    for (dstate_id_t s = DEAD_STATE + 1; s < info.states.size(); s++) {
        assert(info.implId(s)); /* should not map to DEAD_STATE */
        if (info.is_sheng(s)) {
            raw_ids[info.extra[s].sheng_id] = s;
        }
    }
    for (u32 i = 0; i < info.alpha_size; i++) {
        if (i == info.alpha_remap[TOP]) {
            continue;
        }
        auto &mask = masks[i];
        assert(sizeof(mask) == sizeof(m512));
        mask.fill(0);

        for (dstate_id_t sheng_id = 0; sheng_id < sheng_end - 1; sheng_id++) {
            dstate_id_t raw_id = raw_ids[sheng_id];
            dstate_id_t next_id = info.implId(info.states[raw_id].next[i]);
            if (next_id == DEAD_STATE) {
                next_id = sheng_end - 1;
            } else if (next_id < sheng_end) {
                next_id--;
            }
            DEBUG_PRINTF("%hu: %u->next %hu\n", sheng_id, i, next_id);
            mask[sheng_id] = verify_u8(next_id);
        }
    }
    for (u32 i = 0; i < N_CHARS; i++) {
        assert(info.alpha_remap[i] != info.alpha_remap[TOP]);
        memcpy((u8 *)&m->sheng_succ_masks[i],
               (u8 *)masks[info.alpha_remap[i]].data(), sizeof(m512));
    }
    m->sheng_end = sheng_end;
    m->sheng_accel_limit = sheng_end - 1;

    for (dstate_id_t s : raw_ids) {
        if (contains(accel_escape_info, s)) {
            LIMIT_TO_AT_MOST(&m->sheng_accel_limit, info.extra[s].sheng_id);
        }
    }
}

static
void populateBasicInfo64(size_t state_size, const dfa_info &info,
                         u32 total_size, u32 aux_offset, u32 accel_offset,
                         u32 accel_count, ReportID arb, bool single, NFA *nfa) {
    assert(state_size == sizeof(u16) || state_size == sizeof(u8));

    nfa->length = total_size;
    nfa->nPositions = info.states.size();

    nfa->scratchStateSize = verify_u32(state_size);
    nfa->streamStateSize = verify_u32(state_size);

    if (state_size == sizeof(u8)) {
        nfa->type = MCSHENG_64_NFA_8;
    } else {
        nfa->type = MCSHENG_64_NFA_16;
    }

    mcsheng64 *m = (mcsheng64 *)getMutableImplNfa(nfa);
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

    if (single) {
        m->flags |= MCSHENG_FLAG_SINGLE;
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
void fillInAux(mstate_aux *aux, dstate_id_t i, const dfa_info &info,
               const vector<u32> &reports, const vector<u32> &reports_eod,
               const vector<u32> &reportOffsets) {
    const dstate &raw_state = info.states[i];
    aux->accept = raw_state.reports.empty() ? 0 : reportOffsets[reports[i]];
    aux->accept_eod = raw_state.reports_eod.empty() ? 0
                                              : reportOffsets[reports_eod[i]];
    aux->top = info.implId(i ? raw_state.next[info.alpha_remap[TOP]]
                             : info.raw.start_floating);
}

/* returns false on error */
static
bool allocateImplId16(dfa_info &info, dstate_id_t sheng_end,
                      dstate_id_t *sherman_base) {
    info.states[0].impl_id = 0; /* dead is always 0 */

    vector<dstate_id_t> norm;
    vector<dstate_id_t> sherm;
    vector<dstate_id_t> norm_sheng_succ;
    vector<dstate_id_t> sherm_sheng_succ;

    if (info.size() > (1 << 16)) {
        DEBUG_PRINTF("too many states\n");
        *sherman_base = 0;
        return false;
    }

    for (u32 i = 1; i < info.size(); i++) {
        if (info.is_sheng(i)) {
            continue; /* sheng impl ids have already been allocated */
        } if (info.is_sherman(i)) {
            if (info.is_sheng_succ(i)) {
                sherm_sheng_succ.push_back(i);
            } else {
                sherm.push_back(i);
            }
        } else {
            if (info.is_sheng_succ(i)) {
                norm_sheng_succ.push_back(i);
            } else {
                norm.push_back(i);
            }
        }
    }

    dstate_id_t next_norm = sheng_end;
    for (dstate_id_t s : norm_sheng_succ) {
        info.states[s].impl_id = next_norm++;
    }
    if (next_norm + norm.size() + sherm_sheng_succ.size() > UINT8_MAX) {
        /* we need to give sheng_succs ids which fit into a u8 -- demote these
         * to normal states */
        for (dstate_id_t s : sherm_sheng_succ) {
            info.states[s].impl_id = next_norm++;
            info.extra[s].shermanState = false;
        }
        sherm_sheng_succ.clear();
    }
    for (dstate_id_t s : norm) {
        info.states[s].impl_id = next_norm++;
    }

    *sherman_base = next_norm;
    dstate_id_t next_sherman = next_norm;

    for (dstate_id_t s : sherm_sheng_succ) {
        info.states[s].impl_id = next_sherman++;
    }

    for (dstate_id_t s : sherm) {
        info.states[s].impl_id = next_sherman++;
    }

    /* Check to see if we haven't over allocated our states */
    DEBUG_PRINTF("next sherman %u masked %u\n", next_sherman,
                 (dstate_id_t)(next_sherman & STATE_MASK));
    return (next_sherman - 1) == ((next_sherman - 1) & STATE_MASK);
}

typedef RdfaGraph::vertex_descriptor RdfaVertex;

static
bool mark_sheng_succs(const RdfaGraph &g, dfa_info &info,
                      const flat_set<RdfaVertex> &sheng_states) {
    u32 exit_count = 0;

    for (auto v : sheng_states) {
        dstate_id_t s = g[v].index;
        for (u32 i = 0; i != info.alpha_size; i++) {
            if (i == info.alpha_remap[TOP]) {
                continue;
            }
            dstate_id_t next = info.states[s].next[i];
            if (!next || info.is_sheng(next) || info.is_sheng_succ(next)) {
                continue;
            }
            exit_count++;
            info.extra[next].sheng_succ = true;
        }
    }

    if (exit_count + sheng_states.size() < UINT8_MAX) {
        return true;
    } else {
        DEBUG_PRINTF("fail: unable to fit %u exits in byte", exit_count);
        return false;
    }
}

static
CharReach get_edge_reach(dstate_id_t u, dstate_id_t v, const dfa_info &info) {
    CharReach rv;
    for (u32 i = 0; i < info.impl_alpha_size; i++) {
        if (info.raw.states[u].next[i] == v) {
            assert(info.rev_alpha[i].any());
            rv |= info.rev_alpha[i];
        }
    }
    assert(rv.any());
    return rv;
}

#define MAX_SHENG_STATES 16
#define MAX_SHENG64_STATES 64
#define MAX_SHENG_LEAKINESS 0.05

using LeakinessCache = ue2_unordered_map<pair<RdfaVertex, u32>, double>;

/**
 * Returns the proportion of strings of length 'depth' which will leave the
 * sheng region when starting at state 'u'.
 */
static
double leakiness(const RdfaGraph &g, dfa_info &info,
                 const flat_set<RdfaVertex> &sheng_states, RdfaVertex u,
                 u32 depth, LeakinessCache &cache) {
    double rv = 0;
    if (contains(cache, make_pair(u, depth))) {
        return cache[make_pair(u, depth)];
    }
    for (RdfaVertex v : adjacent_vertices_range(u, g)) {
        if (g[v].index == DEAD_STATE) {
            continue;
        }
        double width = get_edge_reach(g[u].index, g[v].index, info).count();
        width /= N_CHARS;

        double weight;
        if (!contains(sheng_states, v)) {
            weight = 1;
        } else if (depth > 1) {
             weight = leakiness(g, info, sheng_states, v, depth - 1, cache);
        } else {
            continue; /* weight = 0 */
        }
        rv += width * weight;
    }

    cache[make_pair(u, depth)] = rv;
    DEBUG_PRINTF("%zu [%u] q = %g\n", g[u].index, depth, rv);
    return rv;
}

/**
 * Returns the proportion of 8 byte strings which will leave the sheng region
 * when starting at state 'u'.
 */
static
double leakiness(const RdfaGraph &g, dfa_info &info,
                 const flat_set<RdfaVertex> &sheng_states, RdfaVertex u) {
    LeakinessCache cache;
    double rv = leakiness(g, info, sheng_states, u, 8, cache);
    return rv;
}

static
dstate_id_t find_sheng_states(dfa_info &info,
                              map<dstate_id_t, AccelScheme> &accel_escape_info,
                              size_t max_sheng_states) {
    RdfaGraph g(info.raw);
    auto cyclics = find_vertices_in_cycles(g);

    auto base_cyclic = RdfaGraph::null_vertex();
    for (const auto &v : cyclics) {
        if (g[v].index == DEAD_STATE) {
            continue;
        }
        DEBUG_PRINTF("considering cyclic %zu\n", g[v].index);
        /* get an estimate of stickness of the cyclic: assume any edges from
         * states with larger state ids are back edges */
        CharReach est_back_reach;
        for (const auto &u : inv_adjacent_vertices_range(v, g)) {
            if (g[u].index < g[v].index) {
                continue;
            }
            est_back_reach |= get_edge_reach(g[u].index, g[v].index, info);
        }

        if (est_back_reach.count() < 30) {
            continue;
        }
        base_cyclic = v;
        break;
    }
    if (!base_cyclic) {
        return DEAD_STATE;
    }

    flat_set<RdfaVertex> sheng_states;
    deque<RdfaVertex> to_consider = { base_cyclic };
    flat_set<dstate_id_t> considered = { DEAD_STATE };
    bool seen_back_edge = false;
    while (!to_consider.empty()
           && sheng_states.size() < max_sheng_states) {
        auto v = to_consider.front();
        to_consider.pop_front();
        if (!considered.insert(g[v].index).second) {
            continue;
        }

        assert(!contains(sheng_states, v));

        if (generates_callbacks(info.raw.kind)
            && !info.states[g[v].index].reports.empty()) {
            /* cannot raise callbacks from sheng region */
            continue;
        }

        sheng_states.insert(v);
        for (const auto &t : adjacent_vertices_range(v, g)) {
            if (!contains(considered, g[t].index)) {
                to_consider.push_back(t);
            }
            if (t == base_cyclic) {
                seen_back_edge = true;
            }
        }
    }

    /* allocate normal ids */
    dstate_id_t sheng_end = DEAD_STATE + 1;
    for (auto v : sheng_states) {
        dstate_id_t s = g[v].index;
        if (!contains(accel_escape_info, s)) {
            info.states[s].impl_id = sheng_end++;
            info.extra[s].sheng_id = info.states[s].impl_id - 1;
        }
    }

    /* allocate accel ids */
    for (auto v : sheng_states) {
        dstate_id_t s = g[v].index;
        if (contains(accel_escape_info, s)) {
            assert(!info.states[s].impl_id);
            info.states[s].impl_id = sheng_end++;
            info.extra[s].sheng_id = info.states[s].impl_id - 1;
        }
    }

    if (sheng_states.size() < MIN_SHENG_SIZE) {
        DEBUG_PRINTF("sheng region too small\n");
        return DEAD_STATE;
    }

    if (!seen_back_edge) {
        DEBUG_PRINTF("did not include cyclic\n");
        return DEAD_STATE;
    }

    double leak = leakiness(g, info, sheng_states, base_cyclic);
    if (leak > MAX_SHENG_LEAKINESS) {
        DEBUG_PRINTF("too leaky (%g)\n", leak);
        return DEAD_STATE;
    }

    if (!mark_sheng_succs(g, info, sheng_states)) {
        return DEAD_STATE;
    }

    /* TODO: ensure sufficiently 'sticky' */
    /* TODO: check not all states accel */
    DEBUG_PRINTF("sheng_end = %hu\n", sheng_end);
    return sheng_end;
}

static
void fill_in_aux_info(NFA *nfa, const dfa_info &info,
                      const map<dstate_id_t, AccelScheme> &accel_escape_info,
                      u32 accel_offset, UNUSED u32 accel_end_offset,
                      const vector<u32> &reports,
                      const vector<u32> &reports_eod,
                      u32 report_base_offset,
                      const raw_report_info &ri) {
    mcsheng *m = (mcsheng *)getMutableImplNfa(nfa);

    vector<u32> reportOffsets;

    ri.fillReportLists(nfa, report_base_offset, reportOffsets);

    for (u32 i = 0; i < info.size(); i++) {
        u16 impl_id = info.implId(i);
        mstate_aux *this_aux = getAux(nfa, impl_id);

        fillInAux(this_aux, i, info, reports, reports_eod, reportOffsets);
        if (contains(accel_escape_info, i)) {
            this_aux->accel_offset = accel_offset;
            accel_offset += info.strat.accelSize();
            assert(accel_offset <= accel_end_offset);
            assert(ISALIGNED_N(accel_offset, alignof(union AccelAux)));
            info.strat.buildAccel(i, accel_escape_info.at(i),
                                  (void *)((char *)m + this_aux->accel_offset));
        }
    }
}

static
u16 get_edge_flags(NFA *nfa, dstate_id_t target_impl_id) {
    mstate_aux *aux = getAux(nfa, target_impl_id);
    u16 flags = 0;

    if (aux->accept) {
        flags |= ACCEPT_FLAG;
    }

    if (aux->accel_offset) {
        flags |= ACCEL_FLAG;
    }

    return flags;
}

static
void fill_in_succ_table_16(NFA *nfa, const dfa_info &info,
                           dstate_id_t sheng_end,
                           UNUSED dstate_id_t sherman_base) {
    u16 *succ_table = (u16 *)((char *)nfa + sizeof(NFA) + sizeof(mcsheng));

    u8 alphaShift = info.getAlphaShift();
    assert(alphaShift <= 8);

    for (size_t i = 0; i < info.size(); i++) {
        if (!info.is_normal(i)) {
            assert(info.implId(i) < sheng_end || info.is_sherman(i));
            continue;
        }

        assert(info.implId(i) < sherman_base);
        u16 normal_id = verify_u16(info.implId(i) - sheng_end);

        for (size_t s = 0; s < info.impl_alpha_size; s++) {
            dstate_id_t raw_succ = info.states[i].next[s];
            u16 &entry = succ_table[((size_t)normal_id << alphaShift) + s];

            entry = info.implId(raw_succ);
            entry |= get_edge_flags(nfa, entry);
        }
    }
}

static
void fill_in_aux_info64(NFA *nfa, const dfa_info &info,
                        const map<dstate_id_t, AccelScheme> &accel_escape_info,
                        u32 accel_offset, UNUSED u32 accel_end_offset,
                        const vector<u32> &reports,
                        const vector<u32> &reports_eod,
                        u32 report_base_offset,
                        const raw_report_info &ri) {
    mcsheng64 *m = (mcsheng64 *)getMutableImplNfa(nfa);

    vector<u32> reportOffsets;

    ri.fillReportLists(nfa, report_base_offset, reportOffsets);

    for (u32 i = 0; i < info.size(); i++) {
        u16 impl_id = info.implId(i);
        mstate_aux *this_aux = getAux64(nfa, impl_id);

        fillInAux(this_aux, i, info, reports, reports_eod, reportOffsets);
        if (contains(accel_escape_info, i)) {
            this_aux->accel_offset = accel_offset;
            accel_offset += info.strat.accelSize();
            assert(accel_offset <= accel_end_offset);
            assert(ISALIGNED_N(accel_offset, alignof(union AccelAux)));
            info.strat.buildAccel(i, accel_escape_info.at(i),
                                  (void *)((char *)m + this_aux->accel_offset));
        }
    }
}

static
u16 get_edge_flags64(NFA *nfa, dstate_id_t target_impl_id) {
    mstate_aux *aux = getAux64(nfa, target_impl_id);
    u16 flags = 0;

    if (aux->accept) {
        flags |= ACCEPT_FLAG;
    }

    if (aux->accel_offset) {
        flags |= ACCEL_FLAG;
    }

    return flags;
}

static
void fill_in_succ_table_64_16(NFA *nfa, const dfa_info &info,
                              dstate_id_t sheng_end,
                              UNUSED dstate_id_t sherman_base) {
    u16 *succ_table = (u16 *)((char *)nfa + sizeof(NFA) + sizeof(mcsheng64));

    u8 alphaShift = info.getAlphaShift();
    assert(alphaShift <= 8);

    for (size_t i = 0; i < info.size(); i++) {
        if (!info.is_normal(i)) {
            assert(info.implId(i) < sheng_end || info.is_sherman(i));
            continue;
        }

        assert(info.implId(i) < sherman_base);
        u16 normal_id = verify_u16(info.implId(i) - sheng_end);

        for (size_t s = 0; s < info.impl_alpha_size; s++) {
            dstate_id_t raw_succ = info.states[i].next[s];
            u16 &entry = succ_table[((size_t)normal_id << alphaShift) + s];

            entry = info.implId(raw_succ);
            entry |= get_edge_flags64(nfa, entry);
        }
    }
}

#define MAX_SHERMAN_LIST_LEN 8

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
void find_better_daddy(dfa_info &info, dstate_id_t curr_id,
                       bool any_cyclic_near_anchored_state, const Grey &grey) {
    if (!grey.allowShermanStates) {
        return;
    }

    const u16 width = sizeof(u16);
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

    flat_set<dstate_id_t> hinted = find_daddy_candidates(info, curr_id);

    for (const dstate_id_t &donor : hinted) {
        assert(donor < curr_id);
        u32 score = 0;

        if (!info.is_normal(donor)) {
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

    if (best_daddy == DEAD_STATE) {
        return; /* No good daddy */
    }

    if (best_score + max_list_len < alphasize) {
        return; /* ??? */
    }

    assert(info.is_normal(currState.daddy));

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

    if (info.is_sheng(curr_id)) {
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

static
void fill_in_sherman(NFA *nfa, dfa_info &info, UNUSED u16 sherman_limit) {
    char *nfa_base = (char *)nfa;
    mcsheng *m = (mcsheng *)getMutableImplNfa(nfa);
    char *sherman_table = nfa_base + m->sherman_offset;

    assert(ISALIGNED_16(sherman_table));
    for (size_t i = 0; i < info.size(); i++) {
        if (!info.is_sherman(i)) {
            continue;
        }
        u16 fs = verify_u16(info.implId(i));
        DEBUG_PRINTF("building sherman %zu impl %hu\n", i, fs);

        assert(fs >= sherman_limit);

        char *curr_sherman_entry
            = sherman_table + (fs - m->sherman_limit) * SHERMAN_FIXED_SIZE;
        assert(curr_sherman_entry <= nfa_base + m->length);

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
                DEBUG_PRINTF("s overrider %hu dad %hu char next %hu\n", fs,
                             info.implId(d),
                             info.implId(info.states[i].next[s]));
                u16 entry_val = info.implId(info.states[i].next[s]);
                entry_val |= get_edge_flags(nfa, entry_val);
                unaligned_store_u16((u8 *)states++, entry_val);
            }
        }
    }
}

static
bytecode_ptr<NFA> mcshengCompile16(dfa_info &info, dstate_id_t sheng_end,
                        const map<dstate_id_t, AccelScheme> &accel_escape_info,
                        const Grey &grey) {
    DEBUG_PRINTF("building mcsheng 16\n");

    vector<u32> reports; /* index in ri for the appropriate report list */
    vector<u32> reports_eod; /* as above */
    ReportID arb;
    u8 single;

    assert(info.getAlphaShift() <= 8);

    // Sherman optimization
    if (info.impl_alpha_size > 16) {
        u16 total_daddy = 0;
        for (u32 i = 0; i < info.size(); i++) {
            find_better_daddy(info, i,
                              is_cyclic_near(info.raw, info.raw.start_anchored),
                              grey);
            total_daddy += info.extra[i].daddytaken;
        }

        DEBUG_PRINTF("daddy %hu/%zu states=%zu alpha=%hu\n", total_daddy,
                     info.size() * info.impl_alpha_size, info.size(),
                     info.impl_alpha_size);
    }

    u16 sherman_limit;
    if (!allocateImplId16(info, sheng_end, &sherman_limit)) {
        DEBUG_PRINTF("failed to allocate state numbers, %zu states total\n",
                     info.size());
        return nullptr;
    }
    u16 count_real_states = sherman_limit - sheng_end;

    auto ri = info.strat.gatherReports(reports, reports_eod, &single, &arb);

    size_t tran_size = (1 << info.getAlphaShift()) * sizeof(u16)
                     * count_real_states;

    size_t aux_size = sizeof(mstate_aux) * info.size();

    size_t aux_offset = ROUNDUP_16(sizeof(NFA) + sizeof(mcsheng) + tran_size);
    size_t accel_size = info.strat.accelSize() * accel_escape_info.size();
    size_t accel_offset = ROUNDUP_N(aux_offset + aux_size
                                    + ri->getReportListSize(), 32);
    size_t sherman_offset = ROUNDUP_16(accel_offset + accel_size);
    size_t sherman_size = calcShermanRegionSize(info);

    size_t total_size = sherman_offset + sherman_size;

    accel_offset -= sizeof(NFA); /* adj accel offset to be relative to m */
    assert(ISALIGNED_N(accel_offset, alignof(union AccelAux)));

    auto nfa = make_zeroed_bytecode_ptr<NFA>(total_size);
    mcsheng *m = (mcsheng *)getMutableImplNfa(nfa.get());

    populateBasicInfo(sizeof(u16), info, total_size, aux_offset, accel_offset,
                      accel_escape_info.size(), arb, single, nfa.get());
    createShuffleMasks(m, info, sheng_end, accel_escape_info);

    /* copy in the mc header information */
    m->sherman_offset = sherman_offset;
    m->sherman_end = total_size;
    m->sherman_limit = sherman_limit;

    DEBUG_PRINTF("%hu sheng, %hu norm, %zu total\n", sheng_end,
                 count_real_states, info.size());

    fill_in_aux_info(nfa.get(), info, accel_escape_info, accel_offset,
                     sherman_offset - sizeof(NFA), reports, reports_eod,
                     aux_offset + aux_size, *ri);

    fill_in_succ_table_16(nfa.get(), info, sheng_end, sherman_limit);

    fill_in_sherman(nfa.get(), info, sherman_limit);

    return nfa;
}

static
void fill_in_succ_table_8(NFA *nfa, const dfa_info &info,
                          dstate_id_t sheng_end) {
    u8 *succ_table = (u8 *)nfa + sizeof(NFA) + sizeof(mcsheng);

    u8 alphaShift = info.getAlphaShift();
    assert(alphaShift <= 8);

    for (size_t i = 0; i < info.size(); i++) {
        assert(!info.is_sherman(i));
        if (!info.is_normal(i)) {
            assert(info.implId(i) < sheng_end);
            continue;
        }
        u8 normal_id = verify_u8(info.implId(i) - sheng_end);

        for (size_t s = 0; s < info.impl_alpha_size; s++) {
            dstate_id_t raw_succ = info.states[i].next[s];
            succ_table[((size_t)normal_id << alphaShift) + s]
                = info.implId(raw_succ);
        }
    }
}

static
void fill_in_sherman64(NFA *nfa, dfa_info &info, UNUSED u16 sherman_limit) {
    char *nfa_base = (char *)nfa;
    mcsheng64 *m = (mcsheng64 *)getMutableImplNfa(nfa);
    char *sherman_table = nfa_base + m->sherman_offset;

    assert(ISALIGNED_16(sherman_table));
    for (size_t i = 0; i < info.size(); i++) {
        if (!info.is_sherman(i)) {
            continue;
        }
        u16 fs = verify_u16(info.implId(i));
        DEBUG_PRINTF("building sherman %zu impl %hu\n", i, fs);

        assert(fs >= sherman_limit);

        char *curr_sherman_entry
            = sherman_table + (fs - m->sherman_limit) * SHERMAN_FIXED_SIZE;
        assert(curr_sherman_entry <= nfa_base + m->length);

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
                DEBUG_PRINTF("s overrider %hu dad %hu char next %hu\n", fs,
                             info.implId(d),
                             info.implId(info.states[i].next[s]));
                u16 entry_val = info.implId(info.states[i].next[s]);
                entry_val |= get_edge_flags64(nfa, entry_val);
                unaligned_store_u16((u8 *)states++, entry_val);
            }
        }
    }
}

static
bytecode_ptr<NFA> mcsheng64Compile16(dfa_info&info, dstate_id_t sheng_end,
                         const map<dstate_id_t, AccelScheme>&accel_escape_info,
                         const Grey &grey) {
    DEBUG_PRINTF("building mcsheng 64-16\n");

    vector<u32> reports; /* index in ri for the appropriate report list */
    vector<u32> reports_eod; /* as above */
    ReportID arb;
    u8 single;

    assert(info.getAlphaShift() <= 8);

    // Sherman optimization
    if (info.impl_alpha_size > 16) {
        u16 total_daddy = 0;
        for (u32 i = 0; i < info.size(); i++) {
            find_better_daddy(info, i,
                              is_cyclic_near(info.raw, info.raw.start_anchored),
                              grey);
            total_daddy += info.extra[i].daddytaken;
        }

        DEBUG_PRINTF("daddy %hu/%zu states=%zu alpha=%hu\n", total_daddy,
                     info.size() * info.impl_alpha_size, info.size(),
                     info.impl_alpha_size);
    }

    u16 sherman_limit;
    if (!allocateImplId16(info, sheng_end, &sherman_limit)) {
        DEBUG_PRINTF("failed to allocate state numbers, %zu states total\n",
                     info.size());
        return nullptr;
    }
    u16 count_real_states = sherman_limit - sheng_end;

    auto ri = info.strat.gatherReports(reports, reports_eod, &single, &arb);

    size_t tran_size = (1 << info.getAlphaShift()) * sizeof(u16)
                     * count_real_states;

    size_t aux_size = sizeof(mstate_aux) * info.size();

    size_t aux_offset = ROUNDUP_16(sizeof(NFA) + sizeof(mcsheng64) + tran_size);
    size_t accel_size = info.strat.accelSize() * accel_escape_info.size();
    size_t accel_offset = ROUNDUP_N(aux_offset + aux_size
                                    + ri->getReportListSize(), 32);
    size_t sherman_offset = ROUNDUP_16(accel_offset + accel_size);
    size_t sherman_size = calcShermanRegionSize(info);

    size_t total_size = sherman_offset + sherman_size;

    accel_offset -= sizeof(NFA); /* adj accel offset to be relative to m */
    assert(ISALIGNED_N(accel_offset, alignof(union AccelAux)));

    auto nfa = make_zeroed_bytecode_ptr<NFA>(total_size);
    mcsheng64 *m = (mcsheng64 *)getMutableImplNfa(nfa.get());

    populateBasicInfo64(sizeof(u16), info, total_size, aux_offset, accel_offset,
                        accel_escape_info.size(), arb, single, nfa.get());
    createShuffleMasks64(m, info, sheng_end, accel_escape_info);

    /* copy in the mc header information */
    m->sherman_offset = sherman_offset;
    m->sherman_end = total_size;
    m->sherman_limit = sherman_limit;

    DEBUG_PRINTF("%hu sheng, %hu norm, %zu total\n", sheng_end,
                 count_real_states, info.size());

    fill_in_aux_info64(nfa.get(), info, accel_escape_info, accel_offset,
                       sherman_offset - sizeof(NFA), reports, reports_eod,
                       aux_offset + aux_size, *ri);

    fill_in_succ_table_64_16(nfa.get(), info, sheng_end, sherman_limit);

    fill_in_sherman64(nfa.get(), info, sherman_limit);

    return nfa;
}

static
void fill_in_succ_table_64_8(NFA *nfa, const dfa_info &info,
                             dstate_id_t sheng_end) {
    u8 *succ_table = (u8 *)nfa + sizeof(NFA) + sizeof(mcsheng64);

    u8 alphaShift = info.getAlphaShift();
    assert(alphaShift <= 8);

    for (size_t i = 0; i < info.size(); i++) {
        assert(!info.is_sherman(i));
        if (!info.is_normal(i)) {
            assert(info.implId(i) < sheng_end);
            continue;
        }
        u8 normal_id = verify_u8(info.implId(i) - sheng_end);

        for (size_t s = 0; s < info.impl_alpha_size; s++) {
            dstate_id_t raw_succ = info.states[i].next[s];
            succ_table[((size_t)normal_id << alphaShift) + s]
                = info.implId(raw_succ);
        }
    }
}

static
void allocateImplId8(dfa_info &info, dstate_id_t sheng_end,
                     const map<dstate_id_t, AccelScheme> &accel_escape_info,
                     u16 *accel_limit, u16 *accept_limit) {
    info.states[0].impl_id = 0; /* dead is always 0 */

    vector<dstate_id_t> norm;
    vector<dstate_id_t> accel;
    vector<dstate_id_t> accept;

    assert(info.size() <= (1 << 8));

    for (u32 i = 1; i < info.size(); i++) {
        if (info.is_sheng(i)) {
            continue; /* already allocated */
        } else if (!info.states[i].reports.empty()) {
            accept.push_back(i);
        } else if (contains(accel_escape_info, i)) {
            accel.push_back(i);
        } else {
            norm.push_back(i);
        }
    }

    u32 j = sheng_end;
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
bytecode_ptr<NFA> mcshengCompile8(dfa_info &info, dstate_id_t sheng_end,
                       const map<dstate_id_t, AccelScheme> &accel_escape_info) {
    DEBUG_PRINTF("building mcsheng 8\n");

    vector<u32> reports;
    vector<u32> reports_eod;
    ReportID arb;
    u8 single;

    auto ri = info.strat.gatherReports(reports, reports_eod, &single, &arb);

    size_t normal_count = info.size() - sheng_end;

    size_t tran_size = sizeof(u8) * (1 << info.getAlphaShift()) * normal_count;
    size_t aux_size = sizeof(mstate_aux) * info.size();
    size_t aux_offset = ROUNDUP_16(sizeof(NFA) + sizeof(mcsheng) + tran_size);
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
    mcsheng *m = (mcsheng *)getMutableImplNfa(nfa.get());

    allocateImplId8(info, sheng_end, accel_escape_info, &m->accel_limit_8,
                    &m->accept_limit_8);

    populateBasicInfo(sizeof(u8), info, total_size, aux_offset, accel_offset,
                      accel_escape_info.size(), arb, single, nfa.get());
    createShuffleMasks(m, info, sheng_end, accel_escape_info);

    fill_in_aux_info(nfa.get(), info, accel_escape_info, accel_offset,
                     total_size - sizeof(NFA), reports, reports_eod,
                     aux_offset + aux_size, *ri);

    fill_in_succ_table_8(nfa.get(), info, sheng_end);

    DEBUG_PRINTF("rl size %zu\n", ri->size());

    return nfa;
}

static
bytecode_ptr<NFA> mcsheng64Compile8(dfa_info &info, dstate_id_t sheng_end,
                      const map<dstate_id_t, AccelScheme> &accel_escape_info) {
    DEBUG_PRINTF("building mcsheng 64-8\n");

    vector<u32> reports;
    vector<u32> reports_eod;
    ReportID arb;
    u8 single;

    auto ri = info.strat.gatherReports(reports, reports_eod, &single, &arb);

    size_t normal_count = info.size() - sheng_end;

    size_t tran_size = sizeof(u8) * (1 << info.getAlphaShift()) * normal_count;
    size_t aux_size = sizeof(mstate_aux) * info.size();
    size_t aux_offset = ROUNDUP_16(sizeof(NFA) + sizeof(mcsheng64) + tran_size);
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
    mcsheng64 *m = (mcsheng64 *)getMutableImplNfa(nfa.get());

    allocateImplId8(info, sheng_end, accel_escape_info, &m->accel_limit_8,
                    &m->accept_limit_8);

    populateBasicInfo64(sizeof(u8), info, total_size, aux_offset, accel_offset,
                        accel_escape_info.size(), arb, single, nfa.get());
    createShuffleMasks64(m, info, sheng_end, accel_escape_info);

    fill_in_aux_info64(nfa.get(), info, accel_escape_info, accel_offset,
                       total_size - sizeof(NFA), reports, reports_eod,
                       aux_offset + aux_size, *ri);

    fill_in_succ_table_64_8(nfa.get(), info, sheng_end);
    DEBUG_PRINTF("rl size %zu\n", ri->size());

    return nfa;
}

bytecode_ptr<NFA> mcshengCompile(raw_dfa &raw, const CompileContext &cc,
                                 const ReportManager &rm) {
    if (!cc.grey.allowMcSheng) {
        return nullptr;
    }

    mcclellan_build_strat mbs(raw, rm, false);
    dfa_info info(mbs);
    bool using8bit = cc.grey.allowMcClellan8 && info.size() <= 256;

    if (!cc.streaming) { /* TODO: work out if we can do the strip in streaming
                          * mode with our semantics */
        raw.stripExtraEodReports();
    }

    bool has_eod_reports = raw.hasEodReports();

    map<dstate_id_t, AccelScheme> accel_escape_info
        = info.strat.getAccelInfo(cc.grey);
    auto old_states = info.states;
    dstate_id_t sheng_end = find_sheng_states(info, accel_escape_info, MAX_SHENG_STATES);

    if (sheng_end <= DEAD_STATE + 1) {
        info.states = old_states;
        return nullptr;
    }

    bytecode_ptr<NFA> nfa;

    if (!using8bit) {
        nfa = mcshengCompile16(info, sheng_end, accel_escape_info, cc.grey);
    } else {
        nfa = mcshengCompile8(info, sheng_end, accel_escape_info);
    }

    if (!nfa) {
        info.states = old_states;
        return nfa;
    }

    if (has_eod_reports) {
        nfa->flags |= NFA_ACCEPTS_EOD;
    }

    DEBUG_PRINTF("compile done\n");
    return nfa;
}

bytecode_ptr<NFA> mcshengCompile64(raw_dfa &raw, const CompileContext &cc,
                                   const ReportManager &rm) {
    if (!cc.grey.allowMcSheng) {
        return nullptr;
    }

    if (!cc.target_info.has_avx512vbmi()) {
        DEBUG_PRINTF("McSheng64 failed, no HS_CPU_FEATURES_AVX512VBMI!\n");
        return nullptr;
    }

    mcclellan_build_strat mbs(raw, rm, false);
    dfa_info info(mbs);
    bool using8bit = cc.grey.allowMcClellan8 && info.size() <= 256;

    if (!cc.streaming) { /* TODO: work out if we can do the strip in streaming
                          * mode with our semantics */
        raw.stripExtraEodReports();
    }

    bool has_eod_reports = raw.hasEodReports();

    map<dstate_id_t, AccelScheme> accel_escape_info
        = info.strat.getAccelInfo(cc.grey);
    bool using64state = false; /*default flag*/
    dstate_id_t sheng_end64;
    sheng_end64 = find_sheng_states(info, accel_escape_info, MAX_SHENG64_STATES);

    if (sheng_end64 <= DEAD_STATE + 1) {
        return nullptr;
    } else {
        using64state = true;
    }

    bytecode_ptr<NFA> nfa;

    if (using64state) {
        assert((sheng_end64 > 17) && (sheng_end64 <= 65));
        if (!using8bit) {
            nfa = mcsheng64Compile16(info, sheng_end64, accel_escape_info, cc.grey);
        } else {
            assert(using8bit);
            nfa = mcsheng64Compile8(info, sheng_end64, accel_escape_info);
            assert(nfa);
            assert(nfa->type == MCSHENG_64_NFA_8);
        }
    }

    if (!nfa) {
        return nfa;
    }

    if (has_eod_reports) {
        nfa->flags |= NFA_ACCEPTS_EOD;
    }

    DEBUG_PRINTF("compile done\n");
    return nfa;
}

bool has_accel_mcsheng(const NFA *) {
    return true; /* consider the sheng region as accelerated */
}

} // namespace ue2
