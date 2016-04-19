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

#include "mcclellancompile_util.h"

#include "rdfa.h"
#include "util/container.h"
#include "util/ue2_containers.h"
#include "ue2common.h"

#include <deque>

#include <boost/functional/hash/hash.hpp>

using namespace std;

namespace ue2 {

#define INIT_STATE 1

static
u32 count_dots(const raw_dfa &raw) {
    assert(raw.start_anchored == INIT_STATE);

    u32 i = INIT_STATE;
    for (; i < raw.states.size() && i != raw.start_floating; i++) {
        DEBUG_PRINTF("checking %u\n", i);
        assert(raw.states[i].reports.empty());
        assert(raw.states[i].reports_eod.empty());

        for (symbol_t s = 0; s < raw.getImplAlphaSize(); s++) {
            DEBUG_PRINTF("%hu -> %hu\n", s, raw.states[i].next[s]);
            if (raw.states[i].next[s] != i + 1) {
                goto validate;
            }
        }

        if (!raw.states[raw.states[i].next[0]].reports.empty()
            || !raw.states[raw.states[i].next[0]].reports_eod.empty()) {
            goto validate;
        }

        DEBUG_PRINTF("got dot\n");
    }

 validate:
    u32 dot_count = i - INIT_STATE;

    /* we need to check that no later state has a transition into these leading
     * dots */
    for (; i < raw.states.size(); i++) {
        for (symbol_t s = 0; s < raw.getImplAlphaSize(); s++) {
            DEBUG_PRINTF("%hu -> %hu\n", s, raw.states[i].next[s]);
            dstate_id_t n = raw.states[i].next[s];
            if (n != DEAD_STATE && n <= dot_count) {
                return 0;
            }
        }
    }

    return dot_count;
}

static
void prune_leading_states(raw_dfa &raw, u32 count) {
    if (!count) {
        return;
    }

    for (u32 i = INIT_STATE + count; i < raw.states.size(); i++) {
        dstate &curr = raw.states[i - count];
        curr = raw.states[i];
        if (curr.daddy > count) {
            curr.daddy -= count;
        } else {
            curr.daddy = DEAD_STATE;
        }

        for (u32 j = 0; j < raw.alpha_size; j++) {
            assert(curr.next[j] == DEAD_STATE || curr.next[j] > count);
            if (curr.next[j]) {
                curr.next[j] -= count;
            }
        }
    }

    raw.states.erase(raw.states.end() - count, raw.states.end());
}

u32 remove_leading_dots(raw_dfa &raw) {
    u32 count = count_dots(raw);
    prune_leading_states(raw, count);
    DEBUG_PRINTF("removed %u leading dots\n", count);
    return count;
}

static never_inline
u32 calc_min_dist_from_bob(raw_dfa &raw, vector<u32> *dist_in) {
    vector<u32> &dist = *dist_in;
    dist.clear();
    dist.resize(raw.states.size(), ~0U);

    assert(raw.start_anchored != DEAD_STATE);

    deque<dstate_id_t> to_visit;
    to_visit.push_back(raw.start_anchored);
    dist[raw.start_anchored] = 0;

    u32 last_d = 0;

    while (!to_visit.empty()) {
        dstate_id_t s = to_visit.front();
        DEBUG_PRINTF("inspecting %u\n", s);
        to_visit.pop_front();
        assert(s != DEAD_STATE);

        u32 d = dist[s];
        assert(d >= last_d);
        assert(d != ~0U);

        for (u32 j = 0; j < raw.alpha_size; j++) {
            dstate_id_t t = raw.states[s].next[j];
            if (t == DEAD_STATE) {
                continue;
            }
            if (dist[t] == ~0U) {
                to_visit.push_back(t);
                dist[t] = d + 1;
            } else {
                assert(dist[t] <= d + 1);
            }
        }

        last_d = d;
    }

    return last_d;
}

static
void find_in_edges(const raw_dfa &raw, vector<vector<dstate_id_t> > *in_edges) {
    in_edges->clear();
    in_edges->resize(raw.states.size());
    ue2::unordered_set<dstate_id_t> seen;

    for (u32 s = 1; s < raw.states.size(); s++) {
        seen.clear();
        for (u32 j = 0; j < raw.alpha_size; j++) {
            dstate_id_t t = raw.states[s].next[j];
            if (contains(seen, t)) {
                continue;
            }
            seen.insert(t);
            (*in_edges)[t].push_back(s);
        }
    }
}

static
void calc_min_dist_to_accept(const raw_dfa &raw,
                             const vector<vector<dstate_id_t> > &in_edges,
                             vector<u32> *accept_dist) {
    vector<u32> &dist = *accept_dist;
    dist.clear();
    dist.resize(raw.states.size(), ~0U);

    /* for reporting states to start from */
    deque<dstate_id_t> to_visit;
    for (u32 s = 0; s < raw.states.size(); s++) {
        if (!raw.states[s].reports.empty()
            || !raw.states[s].reports_eod.empty()) {
            to_visit.push_back(s);
            dist[s] = 0;
        }
    }

    /* bfs */
    UNUSED u32 last_d = 0;
    while (!to_visit.empty()) {
        dstate_id_t s = to_visit.front();
        to_visit.pop_front();
        assert(s != DEAD_STATE);

        u32 d = dist[s];
        assert(d >= last_d);
        assert(d != ~0U);

        for (vector<dstate_id_t>::const_iterator it = in_edges[s].begin();
             it != in_edges[s].end(); ++it) {
            dstate_id_t t = *it;
            if (t == DEAD_STATE) {
                continue;
            }
            if (dist[t] == ~0U) {
                to_visit.push_back(t);
                dist[t] = d + 1;
            } else {
                assert(dist[t] <= d + 1);
            }
        }

        last_d = d;
    }
}

bool prune_overlong(raw_dfa &raw, u32 max_offset) {
    DEBUG_PRINTF("pruning to at most %u\n", max_offset);
    vector<u32> bob_dist;
    u32 max_min_dist_bob = calc_min_dist_from_bob(raw, &bob_dist);

    if (max_min_dist_bob <= max_offset) {
        return false;
    }

    vector<vector<dstate_id_t> > in_edges;
    find_in_edges(raw, &in_edges);

    vector<u32> accept_dist;
    calc_min_dist_to_accept(raw, in_edges, &accept_dist);

    in_edges.clear();

    /* look over the states and filter out any which cannot reach a report
     * states before max_offset */
    vector<dstate_id_t> new_ids(raw.states.size());
    vector<dstate> new_states;
    u32 count = 1;
    new_states.push_back(raw.states[DEAD_STATE]);

    for (u32 s = DEAD_STATE + 1; s < raw.states.size(); s++) {
        if (bob_dist[s] + accept_dist[s] > max_offset) {
            DEBUG_PRINTF("pruned %u: bob %u, report %u\n", s, bob_dist[s],
                          accept_dist[s]);
            new_ids[s] = DEAD_STATE;
        } else {
            new_ids[s] = count++;
            new_states.push_back(raw.states[s]);
            assert(new_states.size() == count);
            assert(new_ids[s] <= s);
        }
    }

    /* swap states */
    DEBUG_PRINTF("pruned %zu -> %u\n", raw.states.size(), count);
    raw.states.swap(new_states);
    new_states.clear();

    /* update edges and daddys to refer to the new ids */
    for (u32 s = DEAD_STATE + 1; s < raw.states.size(); s++) {
        for (u32 j = 0; j < raw.alpha_size; j++) {
            dstate_id_t old_t = raw.states[s].next[j];
            raw.states[s].next[j] = new_ids[old_t];
        }
        raw.states[s].daddy = new_ids[raw.states[s].daddy];
    }

    /* update specials */
    raw.start_floating = new_ids[raw.start_floating];
    raw.start_anchored = new_ids[raw.start_anchored];

    return true;
}

set<ReportID> all_reports(const raw_dfa &rdfa) {
    set<ReportID> all;
    for (const auto &ds : rdfa.states) {
        insert(&all, ds.reports);
        insert(&all, ds.reports_eod);
    }
    return all;
}

bool has_eod_accepts(const raw_dfa &rdfa) {
    for (const auto &ds : rdfa.states) {
        if (!ds.reports_eod.empty()) {
            return true;
        }
    }
    return false;
}

bool has_non_eod_accepts(const raw_dfa &rdfa) {
    for (const auto &ds : rdfa.states) {
        if (!ds.reports.empty()) {
            return true;
        }
    }
    return false;
}

size_t hash_dfa_no_reports(const raw_dfa &rdfa) {
    using boost::hash_combine;
    using boost::hash_range;

    size_t v = 0;
    hash_combine(v, rdfa.alpha_size);
    hash_combine(v, hash_range(begin(rdfa.alpha_remap), end(rdfa.alpha_remap)));

    for (const auto &ds : rdfa.states) {
        hash_combine(v, hash_range(begin(ds.next), end(ds.next)));
    }

    return v;
}

size_t hash_dfa(const raw_dfa &rdfa) {
    using boost::hash_combine;
    size_t v = 0;
    hash_combine(v, hash_dfa_no_reports(rdfa));
    hash_combine(v, all_reports(rdfa));
    return v;
}

static
bool has_self_loop(dstate_id_t s, const raw_dfa &raw) {
    u16 top_remap = raw.alpha_remap[TOP];
    for (u32 i = 0; i < raw.states[s].next.size(); i++) {
        if (i != top_remap && raw.states[s].next[i] == s) {
            return true;
        }
    }
    return false;
}

dstate_id_t get_sds_or_proxy(const raw_dfa &raw) {
    if (raw.start_floating != DEAD_STATE) {
        DEBUG_PRINTF("has floating start\n");
        return raw.start_floating;
    }

    DEBUG_PRINTF("looking for SDS proxy\n");

    dstate_id_t s = raw.start_anchored;

    if (has_self_loop(s, raw)) {
        return s;
    }

    u16 top_remap = raw.alpha_remap[TOP];

    ue2::unordered_set<dstate_id_t> seen;
    while (true) {
        seen.insert(s);
        DEBUG_PRINTF("basis %hu\n", s);

        /* check if we are connected to a state with a self loop */
        for (u32 i = 0; i < raw.states[s].next.size(); i++) {
            dstate_id_t t = raw.states[s].next[i];
            if (i != top_remap && t != DEAD_STATE && has_self_loop(t, raw)) {
                return t;
            }
        }

        /* find a neighbour to use as a basis for looking for the sds proxy */
        dstate_id_t t = DEAD_STATE;
        for (u32 i = 0; i < raw.states[s].next.size(); i++) {
            dstate_id_t tt = raw.states[s].next[i];
            if (i != top_remap && tt != DEAD_STATE && !contains(seen, tt)) {
                t = tt;
                break;
            }
        }

        if (t == DEAD_STATE) {
            /* we were unable to find a state to use as a SDS proxy */
            return DEAD_STATE;
        }

        s = t;
    }
}

} // namespace ue2
