/*
 * Copyright (c) 2016, Intel Corporation
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

#include "mcclellancompile_accel.h"

#include "mcclellancompile_util.h"

#include "grey.h"
#include "nfagraph/ng_limex_accel.h"
#include "util/charreach.h"
#include "util/container.h"
#include "util/dump_charclass.h"

#include <vector>
#include <sstream>

#define PATHS_LIMIT 500

using namespace std;

namespace ue2 {

namespace {

struct path {
    vector<CharReach> reach;
    dstate_id_t dest = DEAD_STATE;
    explicit path(dstate_id_t base) : dest(base) {}
};

}

static UNUSED
string describeClasses(const vector<CharReach> &v) {
    std::ostringstream oss;
    for (const auto &cr : v) {
        describeClass(oss, cr);
    }
    return oss.str();
}

static
void dump_paths(const vector<path> &paths) {
    for (UNUSED const auto &p : paths) {
        DEBUG_PRINTF("[%s] -> %u\n", describeClasses(p.reach).c_str(), p.dest);
    }
    DEBUG_PRINTF("%zu paths\n", paths.size());
}

static
bool is_useful_path(const vector<path> &good, const path &p) {
    for (const auto &g : good) {
        assert(g.dest == p.dest);
        assert(g.reach.size() <= p.reach.size());
        auto git = g.reach.rbegin();
        auto pit = p.reach.rbegin();

        for (; git != g.reach.rend(); ++git, ++pit) {
            if (!pit->isSubsetOf(*git)) {
                goto next;
            }
        }
        DEBUG_PRINTF("better: [%s] -> %u\n",
                     describeClasses(g.reach).c_str(), g.dest);

        return false;
    next:;
    }

    return true;
}

static
path append(const path &orig, const CharReach &cr, u32 new_dest) {
    path p(new_dest);
    p.reach = orig.reach;
    p.reach.push_back(cr);

    return p;
}

static
void extend(const raw_dfa &rdfa, const path &p,
            map<u32, vector<path> > &all,
            vector<path> &out) {
    dstate s = rdfa.states[p.dest];

    if (!p.reach.empty() && p.reach.back().none()) {
        out.push_back(p);
        return;
    }

    if (!s.reports.empty()) {
        if (generates_callbacks(rdfa.kind)) {
            out.push_back(p);
            return;
        } else {
            path pp = append(p, CharReach(), p.dest);
            all[p.dest].push_back(pp);
            out.push_back(pp);
        }
    }

    if (!s.reports_eod.empty()) {
        path pp = append(p, CharReach(), p.dest);
        all[p.dest].push_back(pp);
        out.push_back(pp);
    }

    map<u32, CharReach> dest;
    for (unsigned i = 0; i < N_CHARS; i++) {
        u32 succ = s.next[rdfa.alpha_remap[i]];
        dest[succ].set(i);
    }

    for (const auto &e : dest) {
        path pp = append(p, e.second, e.first);
        if (!is_useful_path(all[e.first], pp)) {
            DEBUG_PRINTF("not useful: [%s] -> %u\n",
                         describeClasses(pp.reach).c_str(), pp.dest);
            continue;
        }

        DEBUG_PRINTF("----good: [%s] -> %u\n",
                         describeClasses(pp.reach).c_str(), pp.dest);
        all[e.first].push_back(pp);
        out.push_back(pp);
    }
}

static
vector<vector<CharReach> > generate_paths(const raw_dfa &rdfa, dstate_id_t base,
                                          u32 len) {
    vector<path> paths{ path(base) };
    map<u32, vector<path> > all;
    all[base].push_back(path(base));
    for (u32 i = 0; i < len && paths.size() < PATHS_LIMIT; i++) {
        vector<path> next_gen;
        for (const auto &p : paths) {
            extend(rdfa, p, all, next_gen);
        }

        paths = move(next_gen);
    }

    dump_paths(paths);

    vector<vector<CharReach> > rv;
    for (auto &p : paths) {
        rv.push_back(move(p.reach));
    }
    return rv;
}

static
AccelScheme look_for_offset_accel(const raw_dfa &rdfa, dstate_id_t base,
                                  u32 max_allowed_accel_offset) {
    DEBUG_PRINTF("looking for accel for %hu\n", base);
    vector<vector<CharReach> > paths = generate_paths(rdfa, base,
                                                   max_allowed_accel_offset + 1);
    AccelScheme as = findBestAccelScheme(paths, CharReach(), true);
    DEBUG_PRINTF("found %s + %u\n", describeClass(as.cr).c_str(), as.offset);
    return as;
}

static
vector<u16> find_nonexit_symbols(const raw_dfa &rdfa,
                                 const CharReach &escape) {
    set<u16> rv;
    CharReach nonexit = ~escape;
    for (auto i = nonexit.find_first(); i != CharReach::npos;
         i = nonexit.find_next(i)) {
        rv.insert(rdfa.alpha_remap[i]);
    }

    return vector<u16>(rv.begin(), rv.end());
}

static
set<dstate_id_t> find_region(const raw_dfa &rdfa, dstate_id_t base,
                             const AccelScheme &ei) {
    DEBUG_PRINTF("looking for region around %hu\n", base);

    set<dstate_id_t> region = {base};

    if (!ei.double_byte.empty()) {
        return region;
    }

    DEBUG_PRINTF("accel %s+%u\n", describeClass(ei.cr).c_str(), ei.offset);

    const CharReach &escape = ei.cr;
    auto nonexit_symbols = find_nonexit_symbols(rdfa, escape);

    vector<dstate_id_t> pending = {base};
    while (!pending.empty()) {
        dstate_id_t curr = pending.back();
        pending.pop_back();
        for (auto s : nonexit_symbols) {
            dstate_id_t t = rdfa.states[curr].next[s];
            if (contains(region, t)) {
                continue;
            }

            DEBUG_PRINTF("    %hu is in region\n", t);
            region.insert(t);
            pending.push_back(t);
        }
    }

    return region;
}

static
bool better(const AccelScheme &a, const AccelScheme &b) {
    if (!a.double_byte.empty() && b.double_byte.empty()) {
        return true;
    }

    if (!b.double_byte.empty()) {
        return false;
    }

    return a.cr.count() < b.cr.count();
}

static
vector<CharReach> reverse_alpha_remapping(const raw_dfa &rdfa) {
    vector<CharReach> rv(rdfa.alpha_size - 1); /* TOP not required */

    for (u32 i = 0; i < N_CHARS; i++) {
        rv.at(rdfa.alpha_remap[i]).set(i);
    }

    return rv;
}

map<dstate_id_t, AccelScheme> populateAccelerationInfo(const raw_dfa &rdfa,
                                                   const dfa_build_strat &strat,
                                                   const Grey &grey) {
    map<dstate_id_t, AccelScheme> rv;
    if (!grey.accelerateDFA) {
        return rv;
    }

    dstate_id_t sds_proxy = get_sds_or_proxy(rdfa);
    DEBUG_PRINTF("sds %hu\n", sds_proxy);

    for (size_t i = 0; i < rdfa.states.size(); i++) {
        if (i == DEAD_STATE) {
            continue;
        }

        /* Note on report acceleration states: While we can't accelerate while we
         * are spamming out callbacks, the QR code paths don't raise reports
         * during scanning so they can accelerate report states. */
        if (generates_callbacks(rdfa.kind) && !rdfa.states[i].reports.empty()) {
            continue;
        }

        size_t single_limit = i == sds_proxy ? ACCEL_DFA_MAX_FLOATING_STOP_CHAR
                                             : ACCEL_DFA_MAX_STOP_CHAR;
        DEBUG_PRINTF("inspecting %zu/%hu: %zu\n", i, sds_proxy, single_limit);

        AccelScheme ei = strat.find_escape_strings(i);
        if (ei.cr.count() > single_limit) {
            DEBUG_PRINTF("state %zu is not accelerable has %zu\n", i,
                         ei.cr.count());
            continue;
        }

        DEBUG_PRINTF("state %zu should be accelerable %zu\n",
                     i, ei.cr.count());

        rv[i] = ei;
    }

    /* provide accleration states to states in the region of sds */
    if (contains(rv, sds_proxy)) {
        AccelScheme sds_ei = rv[sds_proxy];
        sds_ei.double_byte.clear(); /* region based on single byte scheme
                                     * may differ from double byte */
        DEBUG_PRINTF("looking to expand offset accel to nearby states, %zu\n",
                     sds_ei.cr.count());
        auto sds_region = find_region(rdfa, sds_proxy, sds_ei);
        for (auto s : sds_region) {
            if (!contains(rv, s) || better(sds_ei, rv[s])) {
                rv[s] = sds_ei;
            }
        }
    }

    return rv;
}

static
bool double_byte_ok(const AccelScheme &info) {
    return !info.double_byte.empty()
        && info.double_cr.count() < info.double_byte.size()
        && info.double_cr.count() <= 2 && !info.double_byte.empty();
}

AccelScheme find_mcclellan_escape_info(const raw_dfa &rdfa, dstate_id_t this_idx,
                                       u32 max_allowed_accel_offset) {
    AccelScheme rv;
    rv.cr.clear();
    rv.offset = 0;
    const dstate &raw = rdfa.states[this_idx];
    const vector<CharReach> rev_map = reverse_alpha_remapping(rdfa);
    bool outs2_broken = false;
    map<dstate_id_t, CharReach> succs;

    for (u32 i = 0; i < rev_map.size(); i++) {
        if (raw.next[i] == this_idx) {
            continue;
        }

        const CharReach &cr_i = rev_map.at(i);

        rv.cr |= cr_i;
        dstate_id_t next_id = raw.next[i];

        DEBUG_PRINTF("next is %hu\n", next_id);
        const dstate &raw_next = rdfa.states[next_id];

        if (outs2_broken) {
            continue;
        }

        if (!raw_next.reports.empty() && generates_callbacks(rdfa.kind)) {
            DEBUG_PRINTF("leads to report\n");
            outs2_broken = true;  /* cannot accelerate over reports */
            continue;
        }
        succs[next_id] |= cr_i;
    }

    if (!outs2_broken) {
        for (const auto &e : succs) {
            const CharReach &cr_i = e.second;
            const dstate &raw_next = rdfa.states[e.first];

            CharReach cr_all_j;
            for (u32 j = 0; j < rev_map.size(); j++) {
                if (raw_next.next[j] == raw.next[j]) {
                    continue;
                }

                DEBUG_PRINTF("state %hu: adding sym %u -> %hu to 2 \n", e.first,
                             j, raw_next.next[j]);
                cr_all_j |= rev_map.at(j);
            }

            if (cr_i.count() * cr_all_j.count() > 8) {
                DEBUG_PRINTF("adding %zu to double_cr\n", cr_i.count());
                rv.double_cr |= cr_i;
            } else {
                for (auto ii = cr_i.find_first(); ii != CharReach::npos;
                     ii = cr_i.find_next(ii)) {
                    for (auto jj = cr_all_j.find_first(); jj != CharReach::npos;
                         jj = cr_all_j.find_next(jj)) {
                        rv.double_byte.emplace((u8)ii, (u8)jj);
                    }
                }
            }
        }

        if (rv.double_byte.size() > 8) {
            DEBUG_PRINTF("outs2 too big\n");
            outs2_broken = true;
        }

        if (outs2_broken) {
            rv.double_byte.clear();
        }
    }

    DEBUG_PRINTF("this %u, sds proxy %hu\n", this_idx, get_sds_or_proxy(rdfa));
    DEBUG_PRINTF("broken %d\n", outs2_broken);
    if (!double_byte_ok(rv) && !is_triggered(rdfa.kind)
        && this_idx == rdfa.start_floating
        && this_idx != DEAD_STATE) {
        DEBUG_PRINTF("looking for offset accel at %u\n", this_idx);
        auto offset = look_for_offset_accel(rdfa, this_idx,
                                            max_allowed_accel_offset);
        DEBUG_PRINTF("width %zu vs %zu\n", offset.cr.count(),
                      rv.cr.count());
        if (double_byte_ok(offset) || offset.cr.count() < rv.cr.count()) {
            DEBUG_PRINTF("using offset accel\n");
            rv = offset;
        }
    }

    return rv;
}

}
