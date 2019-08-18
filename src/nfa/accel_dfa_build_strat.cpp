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

#include "accel_dfa_build_strat.h"

#include "accel.h"
#include "grey.h"
#include "nfagraph/ng_limex_accel.h"
#include "shufticompile.h"
#include "trufflecompile.h"
#include "util/accel_scheme.h"
#include "util/charreach.h"
#include "util/container.h"
#include "util/dump_charclass.h"
#include "util/small_vector.h"
#include "util/verify_types.h"

#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#define PATHS_LIMIT 500

using namespace std;

namespace ue2 {

namespace {
struct path {
    small_vector<CharReach, MAX_ACCEL_DEPTH + 1> reach;
    dstate_id_t dest = DEAD_STATE;
    explicit path(dstate_id_t base) : dest(base) {}
};
};

template<typename Container>
void dump_paths(const Container &paths) {
    for (UNUSED const path &p : paths) {
        DEBUG_PRINTF("[%s] -> %u\n", describeClasses(p.reach).c_str(), p.dest);
    }
    DEBUG_PRINTF("%zu paths\n", paths.size());
}

static
vector<CharReach> reverse_alpha_remapping(const raw_dfa &rdfa) {
    vector<CharReach> rv(rdfa.alpha_size - 1); /* TOP not required */

    for (u32 i = 0; i < N_CHARS; i++) {
        rv.at(rdfa.alpha_remap[i]).set(i);
    }

    return rv;
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
        DEBUG_PRINTF("better: [%s] -> %u\n", describeClasses(g.reach).c_str(),
                     g.dest);

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
void extend(const raw_dfa &rdfa, const vector<CharReach> &rev_map,
            const path &p, unordered_map<u32, vector<path>> &all,
            vector<path> &out) {
    const dstate &s = rdfa.states[p.dest];

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
            out.push_back(move(pp));
        }
    }

    if (!s.reports_eod.empty()) {
        path pp = append(p, CharReach(), p.dest);
        all[p.dest].push_back(pp);
        out.push_back(move(pp));
    }

    flat_map<u32, CharReach> dest;
    for (u32 i = 0; i < rev_map.size(); i++) {
        u32 succ = s.next[i];
        dest[succ] |= rev_map[i];
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
        out.push_back(move(pp));
    }
}

static
vector<vector<CharReach>> generate_paths(const raw_dfa &rdfa,
                                         dstate_id_t base, u32 len) {
    const vector<CharReach> rev_map = reverse_alpha_remapping(rdfa);
    vector<path> paths{path(base)};
    unordered_map<u32, vector<path>> all;
    all[base].push_back(path(base));
    for (u32 i = 0; i < len && paths.size() < PATHS_LIMIT; i++) {
        vector<path> next_gen;
        for (const auto &p : paths) {
            extend(rdfa, rev_map, p, all, next_gen);
        }

        paths = move(next_gen);
    }

    dump_paths(paths);

    vector<vector<CharReach>> rv;
    rv.reserve(paths.size());
    for (auto &p : paths) {
        rv.push_back(vector<CharReach>(std::make_move_iterator(p.reach.begin()),
                                       std::make_move_iterator(p.reach.end())));
    }
    return rv;
}

static
AccelScheme look_for_offset_accel(const raw_dfa &rdfa, dstate_id_t base,
                                  u32 max_allowed_accel_offset) {
    DEBUG_PRINTF("looking for accel for %hu\n", base);
    vector<vector<CharReach>> paths =
        generate_paths(rdfa, base, max_allowed_accel_offset + 1);
    AccelScheme as = findBestAccelScheme(paths, CharReach(), true);
    DEBUG_PRINTF("found %s + %u\n", describeClass(as.cr).c_str(), as.offset);
    return as;
}

static UNUSED
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
bool double_byte_ok(const AccelScheme &info) {
    return !info.double_byte.empty() &&
           info.double_cr.count() < info.double_byte.size() &&
           info.double_cr.count() <= 2;
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

static
flat_set<u16> find_nonexit_symbols(const raw_dfa &rdfa,
                                   const CharReach &escape) {
    flat_set<u16> rv;
    CharReach nonexit = ~escape;
    for (auto i = nonexit.find_first(); i != nonexit.npos;
         i = nonexit.find_next(i)) {
        rv.insert(rdfa.alpha_remap[i]);
    }

    return rv;
}

static
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

    std::unordered_set<dstate_id_t> seen;
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

AccelScheme
accel_dfa_build_strat::find_escape_strings(dstate_id_t this_idx) const {
    AccelScheme rv;
    const raw_dfa &rdfa = get_raw();
    rv.cr.clear();
    rv.offset = 0;
    const dstate &raw = rdfa.states[this_idx];
    const vector<CharReach> rev_map = reverse_alpha_remapping(rdfa);
    bool outs2_broken = false;
    flat_map<dstate_id_t, CharReach> succs;

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
            outs2_broken = true; /* cannot accelerate over reports */
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
                        if (rv.double_byte.size() > 8) {
                            DEBUG_PRINTF("outs2 too big\n");
                            outs2_broken = true;
                            goto done;
                        }
                    }
                }
            }
        }

    done:
        assert(outs2_broken || rv.double_byte.size() <= 8);
        if (outs2_broken) {
            rv.double_byte.clear();
        }
    }

    DEBUG_PRINTF("this %u, sds proxy %hu\n", this_idx, get_sds_or_proxy(rdfa));
    DEBUG_PRINTF("broken %d\n", outs2_broken);
    if (!double_byte_ok(rv) && !is_triggered(rdfa.kind) &&
        this_idx == rdfa.start_floating && this_idx != DEAD_STATE) {
        DEBUG_PRINTF("looking for offset accel at %u\n", this_idx);
        auto offset =
            look_for_offset_accel(rdfa, this_idx, max_allowed_offset_accel());
        DEBUG_PRINTF("width %zu vs %zu\n", offset.cr.count(), rv.cr.count());
        if (double_byte_ok(offset) || offset.cr.count() < rv.cr.count()) {
            DEBUG_PRINTF("using offset accel\n");
            rv = offset;
        }
    }

    return rv;
}

void
accel_dfa_build_strat::buildAccel(UNUSED dstate_id_t this_idx,
                                  const AccelScheme &info,
                                  void *accel_out) {
    AccelAux *accel = (AccelAux *)accel_out;

    DEBUG_PRINTF("accelerations scheme has offset s%u/d%u\n", info.offset,
                 info.double_offset);
    accel->generic.offset = verify_u8(info.offset);

    if (double_byte_ok(info) && info.double_cr.none() &&
        info.double_byte.size() == 1) {
        accel->accel_type = ACCEL_DVERM;
        accel->dverm.c1 = info.double_byte.begin()->first;
        accel->dverm.c2 = info.double_byte.begin()->second;
        accel->dverm.offset = verify_u8(info.double_offset);
        DEBUG_PRINTF("state %hu is double vermicelli\n", this_idx);
        return;
    }

    if (double_byte_ok(info) && info.double_cr.none() &&
        (info.double_byte.size() == 2 || info.double_byte.size() == 4)) {
        bool ok = true;

        assert(!info.double_byte.empty());
        u8 firstC = info.double_byte.begin()->first & CASE_CLEAR;
        u8 secondC = info.double_byte.begin()->second & CASE_CLEAR;

        for (const pair<u8, u8> &p : info.double_byte) {
            if ((p.first & CASE_CLEAR) != firstC ||
                (p.second & CASE_CLEAR) != secondC) {
                ok = false;
                break;
            }
        }

        if (ok) {
            accel->accel_type = ACCEL_DVERM_NOCASE;
            accel->dverm.c1 = firstC;
            accel->dverm.c2 = secondC;
            accel->dverm.offset = verify_u8(info.double_offset);
            DEBUG_PRINTF("state %hu is nc double vermicelli\n", this_idx);
            return;
        }

        u8 m1;
        u8 m2;
        if (buildDvermMask(info.double_byte, &m1, &m2)) {
            accel->accel_type = ACCEL_DVERM_MASKED;
            accel->dverm.offset = verify_u8(info.double_offset);
            accel->dverm.c1 = info.double_byte.begin()->first & m1;
            accel->dverm.c2 = info.double_byte.begin()->second & m2;
            accel->dverm.m1 = m1;
            accel->dverm.m2 = m2;
            DEBUG_PRINTF(
                "building maskeddouble-vermicelli for 0x%02hhx%02hhx\n",
                accel->dverm.c1, accel->dverm.c2);
            return;
        }
    }

    if (double_byte_ok(info) &&
        shuftiBuildDoubleMasks(
            info.double_cr, info.double_byte, (u8 *)&accel->dshufti.lo1,
            (u8 *)&accel->dshufti.hi1, (u8 *)&accel->dshufti.lo2,
            (u8 *)&accel->dshufti.hi2)) {
        accel->accel_type = ACCEL_DSHUFTI;
        accel->dshufti.offset = verify_u8(info.double_offset);
        DEBUG_PRINTF("state %hu is double shufti\n", this_idx);
        return;
    }

    if (info.cr.none()) {
        accel->accel_type = ACCEL_RED_TAPE;
        DEBUG_PRINTF("state %hu is a dead end full of bureaucratic red tape"
                     " from which there is no escape\n",
                     this_idx);
        return;
    }

    if (info.cr.count() == 1) {
        accel->accel_type = ACCEL_VERM;
        accel->verm.c = info.cr.find_first();
        DEBUG_PRINTF("state %hu is vermicelli\n", this_idx);
        return;
    }

    if (info.cr.count() == 2 && info.cr.isCaselessChar()) {
        accel->accel_type = ACCEL_VERM_NOCASE;
        accel->verm.c = info.cr.find_first() & CASE_CLEAR;
        DEBUG_PRINTF("state %hu is caseless vermicelli\n", this_idx);
        return;
    }

    if (info.cr.count() > max_floating_stop_char()) {
        accel->accel_type = ACCEL_NONE;
        DEBUG_PRINTF("state %hu is too broad\n", this_idx);
        return;
    }

    accel->accel_type = ACCEL_SHUFTI;
    if (-1 != shuftiBuildMasks(info.cr, (u8 *)&accel->shufti.lo,
                               (u8 *)&accel->shufti.hi)) {
        DEBUG_PRINTF("state %hu is shufti\n", this_idx);
        return;
    }

    assert(!info.cr.none());
    accel->accel_type = ACCEL_TRUFFLE;
    truffleBuildMasks(info.cr, (u8 *)&accel->truffle.mask1,
                      (u8 *)&accel->truffle.mask2);
    DEBUG_PRINTF("state %hu is truffle\n", this_idx);
}

map<dstate_id_t, AccelScheme>
accel_dfa_build_strat::getAccelInfo(const Grey &grey) {
    map<dstate_id_t, AccelScheme> rv;
    raw_dfa &rdfa = get_raw();
    if (!grey.accelerateDFA) {
        return rv;
    }

    dstate_id_t sds_proxy = get_sds_or_proxy(rdfa);
    DEBUG_PRINTF("sds %hu\n", sds_proxy);

    /* Find accel info for a single state. */
    auto do_state = [&](size_t i) {
        if (i == DEAD_STATE) {
            return;
        }

        /* Note on report acceleration states: While we can't accelerate while
         * we are spamming out callbacks, the QR code paths don't raise reports
         * during scanning so they can accelerate report states. */
        if (generates_callbacks(rdfa.kind) && !rdfa.states[i].reports.empty()) {
            return;
        }

        size_t single_limit =
            i == sds_proxy ? max_floating_stop_char() : max_stop_char();
        DEBUG_PRINTF("inspecting %zu/%hu: %zu\n", i, sds_proxy, single_limit);

        AccelScheme ei = find_escape_strings(i);
        if (ei.cr.count() > single_limit) {
            DEBUG_PRINTF("state %zu is not accelerable has %zu\n", i,
                         ei.cr.count());
            return;
        }

        DEBUG_PRINTF("state %zu should be accelerable %zu\n", i, ei.cr.count());

        rv[i] = ei;
    };

    if (only_accel_init) {
        DEBUG_PRINTF("only computing accel for init states\n");
        do_state(rdfa.start_anchored);
        if (rdfa.start_floating != rdfa.start_anchored) {
            do_state(rdfa.start_floating);
        }
    } else {
        DEBUG_PRINTF("computing accel for all states\n");
        for (size_t i = 0; i < rdfa.states.size(); i++) {
            do_state(i);
        }
    }

    /* provide acceleration states to states in the region of sds */
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
};
