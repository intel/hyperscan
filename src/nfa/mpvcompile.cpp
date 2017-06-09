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

#include "mpvcompile.h"

#include "mpv_internal.h"
#include "nfa_api_queue.h"
#include "nfa_internal.h"
#include "shufticompile.h"
#include "trufflecompile.h"
#include "util/alloc.h"
#include "util/multibit_build.h"
#include "util/order_check.h"
#include "util/report_manager.h"
#include "util/verify_types.h"

#include <algorithm>
#include <iterator>
#include <map>

#include <boost/range/adaptor/map.hpp>

using namespace std;
using boost::adaptors::map_values;
using boost::adaptors::map_keys;

namespace ue2 {

namespace {
struct pcomp {
    bool operator()(const raw_puff &a, const raw_puff &b) const {
        return tie(a.repeats, a.unbounded, a.simple_exhaust, a.report) <
               tie(b.repeats, b.unbounded, b.simple_exhaust, b.report);
    }
};

struct ClusterKey {
    explicit ClusterKey(const raw_puff &src)
        : trigger_event(MQE_INVALID), reach(src.reach),
          auto_restart(src.auto_restart) {}
    ClusterKey(u32 event, const raw_puff &src)
        : trigger_event(event), reach(src.reach),
          auto_restart(src.auto_restart) {}

    u32 trigger_event;
    CharReach reach;
    bool auto_restart;

    bool operator<(const ClusterKey &b) const {
        const ClusterKey &a = *this;
        ORDER_CHECK(trigger_event); /* want triggered puffs first */
        ORDER_CHECK(auto_restart);
        ORDER_CHECK(reach);
        return false;
    }
};

} // namespace

static
void writePuffette(mpv_puffette *out, const raw_puff &rp,
                   const ReportManager &rm) {
    DEBUG_PRINTF("outputting %u %d %u to %p\n", rp.repeats, (int)rp.unbounded,
                 rp.report, out);
    out->repeats = rp.repeats;
    out->unbounded = rp.unbounded;
    out->simple_exhaust = rp.simple_exhaust;
    out->report = rm.getProgramOffset(rp.report);
}

static
void writeSentinel(mpv_puffette *out) {
    DEBUG_PRINTF("outputting sentinel to %p\n", out);
    memset(out, 0, sizeof(*out));
    out->report = INVALID_REPORT;
}

static
void writeDeadPoint(mpv_kilopuff *out, const vector<raw_puff> &puffs) {
    for (const auto &puff : puffs) {
        if (puff.unbounded) { /* mpv can never die */
            out->dead_point = MPV_DEAD_VALUE;
            return;
        }
    }

    out->dead_point = puffs.back().repeats + 1;
}

static
size_t calcSize(const map<ClusterKey, vector<raw_puff>> &raw,
                const vector<mpv_counter_info> &counters) {
    size_t len = sizeof(NFA) + sizeof(mpv);

    len += sizeof(mpv_kilopuff) * raw.size(); /* need a kilopuff for each
                                                 distinct reach */

    len += sizeof(mpv_counter_info) * counters.size();

    len += sizeof(mpv_puffette); /* initial sent */

    for (const vector<raw_puff> &puffs : raw | map_values) {
        len += sizeof(mpv_puffette) * puffs.size();
        len += sizeof(mpv_puffette); /* terminal sent */
    }

    return len;
}

static
void populateClusters(const vector<raw_puff> &puffs_in,
                      const vector<raw_puff> &triggered_puffs,
                      map<ClusterKey, vector<raw_puff>> *raw) {
    map<ClusterKey, vector<raw_puff>> &puff_clusters = *raw;

    u32 e = MQE_TOP_FIRST;
    for (const auto &puff : triggered_puffs) {
        puff_clusters[ClusterKey(e, puff)].push_back(puff);
        e++;
    }

    for (const auto &puff : puffs_in) {
        puff_clusters[ClusterKey(puff)].push_back(puff);
    }


    for (vector<raw_puff> &puffs : puff_clusters | map_values) {
        sort(puffs.begin(), puffs.end(), pcomp());
    }
}

static
void writeKiloPuff(const map<ClusterKey, vector<raw_puff>>::const_iterator &it,
                   const ReportManager &rm, u32 counter_offset, mpv *m,
                   mpv_kilopuff *kp, mpv_puffette **pa) {
    const CharReach &reach = it->first.reach;
    const vector<raw_puff> &puffs = it->second;

    kp->auto_restart = it->first.auto_restart;

    if (reach.all()) {
        kp->type = MPV_DOT;
    } else if (reach.count() == 255) {
        kp->type = MPV_VERM;
        size_t unset = (~reach).find_first();
        assert(unset != CharReach::npos);
        kp->u.verm.c = (char)unset;
    } else if (reach.count() == 1) {
        kp->type = MPV_NVERM;
        size_t set = reach.find_first();
        assert(set != CharReach::npos);
        kp->u.verm.c = (char)set;
    } else if (shuftiBuildMasks(~reach, (u8 *)&kp->u.shuf.mask_lo,
                                (u8 *)&kp->u.shuf.mask_hi) != -1) {
        kp->type = MPV_SHUFTI;
    } else {
        kp->type = MPV_TRUFFLE;
        truffleBuildMasks(~reach, (u8 *)&kp->u.truffle.mask1,
                          (u8 *)&kp->u.truffle.mask2);
    }

    kp->count = verify_u32(puffs.size());
    kp->counter_offset = counter_offset;

    /* start of real puffette array */
    kp->puffette_offset = verify_u32((char *)*pa - (char *)m);
    for (size_t i = 0; i < puffs.size(); i++) {
        assert(!it->first.auto_restart || puffs[i].unbounded);
        writePuffette(*pa + i, puffs[i], rm);
    }

    *pa += puffs.size();
    writeSentinel(*pa);
    ++*pa;

    writeDeadPoint(kp, puffs);
}

static
void writeCoreNfa(NFA *nfa, u32 len, u32 min_width, u32 max_counter,
                  u32 streamStateSize, u32 scratchStateSize) {
    assert(nfa);

    nfa->length = len;
    nfa->nPositions = max_counter - 1;
    nfa->type = MPV_NFA;
    nfa->streamStateSize = streamStateSize;
    assert(16 >= sizeof(mpv_decomp_kilo));
    nfa->scratchStateSize = scratchStateSize;
    nfa->minWidth = min_width;
}

static
void findCounterSize(map<ClusterKey, vector<raw_puff>>::const_iterator kp_it,
                     map<ClusterKey, vector<raw_puff>>::const_iterator kp_ite,
                     u64a *max_counter_out, u32 *counter_size) {
    u32 max_counter = 0; /* max counter that we may need to know about is one
                            more than largest repeat */
    for (; kp_it != kp_ite; ++kp_it) {
        max_counter = MAX(max_counter, kp_it->second.back().repeats + 1);
    }

    if (max_counter < (1U << 8)) {
        *counter_size = 1;
    } else if (max_counter < (1U << 16)) {
        *counter_size = 2;
    } else if (max_counter < (1U << 24)) {
        *counter_size = 3;
    } else {
        *counter_size = 4;
    }

    *max_counter_out = max_counter;
}

static
void fillCounterInfo(mpv_counter_info *out, u32 *curr_decomp_offset,
                    u32 *curr_comp_offset,
                    const map<ClusterKey, vector<raw_puff>> &kilopuffs,
                    map<ClusterKey, vector<raw_puff>>::const_iterator kp_it,
                    map<ClusterKey, vector<raw_puff>>::const_iterator kp_ite) {

    out->kilo_begin = distance(kilopuffs.begin(), kp_it);
    out->kilo_end = distance(kilopuffs.begin(), kp_ite);
    findCounterSize(kp_it, kp_ite, &out->max_counter, &out->counter_size);
    out->counter_offset = *curr_decomp_offset;
    *curr_decomp_offset += sizeof(u64a);
    *curr_comp_offset += out->counter_size;
}

static
void fillCounterInfos(vector<mpv_counter_info> *out, u32 *curr_decomp_offset,
                      u32 *curr_comp_offset,
                      const map<ClusterKey, vector<raw_puff>> &kilopuffs) {
    /* first the triggered puffs */
    map<ClusterKey, vector<raw_puff>>::const_iterator it = kilopuffs.begin();
    while (it != kilopuffs.end() && it->first.trigger_event != MQE_INVALID) {
        assert(!it->first.auto_restart);
        assert(it->first.trigger_event
               == MQE_TOP_FIRST + distance(kilopuffs.begin(), it));

        out->push_back(mpv_counter_info());
        map<ClusterKey, vector<raw_puff>>::const_iterator it_o = it;
        ++it;
        fillCounterInfo(&out->back(), curr_decomp_offset, curr_comp_offset,
                    kilopuffs, it_o, it);
    }

    /* we may have 2 sets of non triggered puffs:
     * 1) always started with no auto_restart
     * 2) always started with auto_restart
     */
    map<ClusterKey, vector<raw_puff>>::const_iterator trig_ite = it;
    while (it != kilopuffs.end() && !it->first.auto_restart) {
        assert(it->first.trigger_event == MQE_INVALID);

        ++it;
    }
    if (it != trig_ite) {
        out->push_back(mpv_counter_info());
        fillCounterInfo(&out->back(), curr_decomp_offset, curr_comp_offset,
                        kilopuffs, kilopuffs.begin(), it);
    }
    while (it != kilopuffs.end() && it->first.auto_restart) {
        assert(it->first.trigger_event == MQE_INVALID);

        out->push_back(mpv_counter_info());
        map<ClusterKey, vector<raw_puff>>::const_iterator it_o = it;
        ++it;
        fillCounterInfo(&out->back(), curr_decomp_offset, curr_comp_offset,
                    kilopuffs, it_o, it);
    }
}

static
const mpv_counter_info &findCounter(const vector<mpv_counter_info> &counters,
                                    u32 i) {
    for (const auto &counter : counters) {
        if (i >= counter.kilo_begin && i < counter.kilo_end) {
            return counter;
        }
    }
    assert(0);
    return counters.front();
}

bytecode_ptr<NFA> mpvCompile(const vector<raw_puff> &puffs_in,
                             const vector<raw_puff> &triggered_puffs,
                             const ReportManager &rm) {
    assert(!puffs_in.empty() || !triggered_puffs.empty());
    u32 puffette_count = puffs_in.size() + triggered_puffs.size();

    map<ClusterKey, vector<raw_puff>> puff_clusters;
    populateClusters(puffs_in, triggered_puffs, &puff_clusters);

    u32 curr_comp_offset = 0;

    u32 curr_decomp_offset = sizeof(mpv_decomp_state);
    curr_decomp_offset += 16 * puff_clusters.size();

    vector<mpv_counter_info> counters;
    fillCounterInfos(&counters, &curr_decomp_offset, &curr_comp_offset,
                    puff_clusters);

    u32 pq_offset = curr_decomp_offset;
    curr_decomp_offset += sizeof(mpv_pq_item) * puff_clusters.size();

    u32 rl_offset = curr_decomp_offset;
    curr_decomp_offset += sizeof(ReportID) * puffette_count;

    u32 reporter_offset = curr_decomp_offset;
    curr_decomp_offset += mmbit_size(puff_clusters.size());

    u32 active_offset = curr_comp_offset;
    curr_comp_offset += mmbit_size(puff_clusters.size());

    u32 len = calcSize(puff_clusters, counters);

    DEBUG_PRINTF("%u puffs, len = %u\n", puffette_count, len);

    auto nfa = make_zeroed_bytecode_ptr<NFA>(len);

    mpv_puffette *pa_base = (mpv_puffette *)
        ((char *)nfa.get() + sizeof(NFA) + sizeof(mpv)
         + sizeof(mpv_kilopuff) * puff_clusters.size()
         + sizeof(mpv_counter_info) * counters.size());
    mpv_puffette *pa = pa_base;

    writeSentinel(pa);

    ++pa; /* skip init sentinel */

    u32 min_repeat = ~0U;
    u32 max_counter = 0; /* max counter that we may need to know about is one
                            more than largest repeat */
    for (const vector<raw_puff> &puffs : puff_clusters | map_values) {
        max_counter = max(max_counter, puffs.back().repeats + 1);
        min_repeat = min(min_repeat, puffs.front().repeats);
    }

    mpv *m = (mpv *)getMutableImplNfa(nfa.get());
    m->kilo_count = verify_u32(puff_clusters.size());
    m->counter_count = verify_u32(counters.size());
    m->puffette_count = puffette_count;
    m->pq_offset = pq_offset;
    m->reporter_offset = reporter_offset;
    m->report_list_offset = rl_offset;
    m->active_offset = active_offset;
    m->top_kilo_begin = verify_u32(triggered_puffs.size());
    m->top_kilo_end = verify_u32(puff_clusters.size());

    mpv_kilopuff *kp_begin = (mpv_kilopuff *)(m + 1);
    mpv_kilopuff *kp = kp_begin;
    for (auto it = puff_clusters.begin(); it != puff_clusters.end(); ++it) {
        writeKiloPuff(it, rm,
                      findCounter(counters, kp - kp_begin).counter_offset, m,
                      kp, &pa);
        ++kp;
    }
    assert((char *)pa == (char *)nfa.get() + len);

    mpv_counter_info *out_ci = (mpv_counter_info *)kp;
    for (const auto &counter : counters) {
        *out_ci = counter;
        ++out_ci;
    }
    assert((char *)out_ci == (char *)pa_base);

    writeCoreNfa(nfa.get(), len, min_repeat, max_counter, curr_comp_offset,
                 curr_decomp_offset);

    return nfa;
}

} // namespace ue2
