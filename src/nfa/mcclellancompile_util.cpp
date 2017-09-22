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

#include "mcclellancompile_util.h"

#include "rdfa.h"
#include "util/container.h"
#include "util/hash.h"
#include "ue2common.h"

#include <deque>
#include <map>

using namespace std;

namespace ue2 {

#define INIT_STATE 1

static
bool state_has_reports(const raw_dfa &raw, dstate_id_t s) {
    const auto &ds = raw.states[s];
    return !ds.reports.empty() || !ds.reports_eod.empty();
}

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

        if (state_has_reports(raw, raw.states[i].next[0])) {
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
    dist.assign(raw.states.size(), ~0U);

    assert(raw.start_anchored != DEAD_STATE);

    deque<dstate_id_t> to_visit = { raw.start_anchored };
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

        for (dstate_id_t t : raw.states[s].next) {
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

bool clear_deeper_reports(raw_dfa &raw, u32 max_offset) {
    DEBUG_PRINTF("clearing reports on states deeper than %u\n", max_offset);
    vector<u32> bob_dist;
    u32 max_min_dist_bob = calc_min_dist_from_bob(raw, &bob_dist);

    if (max_min_dist_bob <= max_offset) {
        return false;
    }

    bool changed = false;
    for (u32 s = DEAD_STATE + 1; s < raw.states.size(); s++) {
        if (bob_dist[s] > max_offset && state_has_reports(raw, s)) {
            DEBUG_PRINTF("clearing reports on %u (depth %u)\n", s, bob_dist[s]);
            auto &ds = raw.states[s];
            ds.reports.clear();
            ds.reports_eod.clear();
            changed = true;
        }
    }

    if (!changed) {
        return false;
    }

    // We may have cleared all reports from the DFA, in which case it should
    // become empty.
    if (all_of_in(raw.states, [](const dstate &ds) {
            return ds.reports.empty() && ds.reports_eod.empty();
        })) {
        DEBUG_PRINTF("no reports left at all, dfa is dead\n");
        raw.start_anchored = DEAD_STATE;
        raw.start_floating = DEAD_STATE;
    }

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
    size_t v = 0;
    hash_combine(v, rdfa.alpha_size);
    hash_combine(v, rdfa.alpha_remap);

    for (const auto &ds : rdfa.states) {
        hash_combine(v, ds.next);
    }

    return v;
}

size_t hash_dfa(const raw_dfa &rdfa) {
    size_t v = 0;
    hash_combine(v, hash_dfa_no_reports(rdfa));
    hash_combine(v, all_reports(rdfa));
    return v;
}

static
bool can_die_early(const raw_dfa &raw, dstate_id_t s,
                   map<dstate_id_t, u32> &visited, u32 age_limit) {
    if (contains(visited, s) && visited[s] >= age_limit) {
        /* we have already visited (or are in the process of visiting) here with
         * a looser limit. */
        return false;
    }
    visited[s] = age_limit;

    if (s == DEAD_STATE) {
        return true;
    }

    if (age_limit == 0) {
        return false;
    }

    for (const auto &next : raw.states[s].next) {
        if (can_die_early(raw, next, visited, age_limit - 1)) {
            return true;
        }
    }

    return false;
}

bool can_die_early(const raw_dfa &raw, u32 age_limit) {
    map<dstate_id_t, u32> visited;
    return can_die_early(raw, raw.start_anchored, visited, age_limit);
}

bool is_dead(const raw_dfa &rdfa) {
    return rdfa.start_anchored == DEAD_STATE &&
           rdfa.start_floating == DEAD_STATE;
}

} // namespace ue2
