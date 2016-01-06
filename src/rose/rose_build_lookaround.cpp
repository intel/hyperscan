/*
 * Copyright (c) 2015, Intel Corporation
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

/** \file
 * \brief Rose compile-time analysis for lookaround masks.
 */
#include "rose_build_lookaround.h"

#include "rose_build_impl.h"
#include "nfa/castlecompile.h"
#include "nfa/goughcompile.h"
#include "nfa/rdfa.h"
#include "nfagraph/ng_repeat.h"
#include "nfagraph/ng_util.h"
#include "util/container.h"
#include "util/dump_charclass.h"
#include "util/graph_range.h"
#include "util/ue2_containers.h"
#include "util/verify_types.h"

#include <cstdlib>
#include <queue>

using namespace std;

namespace ue2 {

/** \brief Max search distance for reachability in front of a role. */
static const u32 MAX_FWD_LEN = 64;

/** \brief Max search distance for reachability behind a role. */
static const u32 MAX_BACK_LEN = 64;

/** \brief Max lookaround entries for a role. */
static const u32 MAX_LOOKAROUND_ENTRIES = 16;

/** \brief We would rather have lookarounds with smaller reach than this. */
static const u32 LOOKAROUND_WIDE_REACH = 200;

static
void getForwardReach(const NGHolder &g, u32 top, map<s32, CharReach> &look) {
    ue2::flat_set<NFAVertex> curr, next;

    // Consider only successors of start with the required top.
    for (const auto &e : out_edges_range(g.start, g)) {
        NFAVertex v = target(e, g);
        if (v == g.startDs) {
            continue;
        }
        if (g[e].top == top) {
            curr.insert(v);
        }
    }

    for (u32 i = 0; i < MAX_FWD_LEN; i++) {
        if (curr.empty() || contains(curr, g.accept) ||
            contains(curr, g.acceptEod)) {
            break;
        }

        next.clear();
        CharReach cr;

        for (auto v : curr) {
            assert(!is_special(v, g));
            cr |= g[v].char_reach;
            insert(&next, adjacent_vertices(v, g));
        }

        assert(cr.any());
        look[i] |= cr;
        curr.swap(next);
    }
}

static
void getBackwardReach(const NGHolder &g, ReportID report, u32 lag,
                      map<s32, CharReach> &look) {
    ue2::flat_set<NFAVertex> curr, next;

    for (auto v : inv_adjacent_vertices_range(g.accept, g)) {
        if (contains(g[v].reports, report)) {
            curr.insert(v);
        }
    }

    for (u32 i = lag + 1; i <= MAX_BACK_LEN; i++) {
        if (curr.empty() || contains(curr, g.start) ||
            contains(curr, g.startDs)) {
            break;
        }

        next.clear();
        CharReach cr;

        for (auto v : curr) {
            assert(!is_special(v, g));
            cr |= g[v].char_reach;
            insert(&next, inv_adjacent_vertices(v, g));
        }

        assert(cr.any());
        look[0 - i] |= cr;
        curr.swap(next);
    }
}

static
void getForwardReach(const CastleProto &castle, u32 top,
                     map<s32, CharReach> &look) {
    depth len = castle.repeats.at(top).bounds.min;
    len = min(len, depth(MAX_FWD_LEN));
    assert(len.is_finite());

    const CharReach &cr = castle.reach();
    for (u32 i = 0; i < len; i++) {
        look[i] |= cr;
    }
}

static
void getBackwardReach(const CastleProto &castle, ReportID report, u32 lag,
                      map<s32, CharReach> &look) {
    depth min_depth = depth::infinity();
    for (const auto &m : castle.repeats) {
        const PureRepeat &pr = m.second;
        if (contains(pr.reports, report)) {
            min_depth = min(min_depth, pr.bounds.min);
        }
    }

    if (!min_depth.is_finite()) {
        assert(0);
        return;
    }

    const CharReach &cr = castle.reach();
    for (u32 i = lag + 1; i <= min(lag + (u32)min_depth, MAX_BACK_LEN);
         i++) {
        look[0 - i] |= cr;
    }
}

static
void getForwardReach(const raw_dfa &rdfa, map<s32, CharReach> &look) {
    if (rdfa.states.size() < 2) {
        return;
    }

    ue2::flat_set<dstate_id_t> curr, next;
    curr.insert(rdfa.start_anchored);

    for (u32 i = 0; i < MAX_FWD_LEN && !curr.empty(); i++) {
        next.clear();
        CharReach cr;

        for (const auto state_id : curr) {
            const dstate &ds = rdfa.states[state_id];

            if (!ds.reports.empty() || !ds.reports_eod.empty()) {
                return;
            }

            for (unsigned c = 0; c < N_CHARS; c++) {
                dstate_id_t succ = ds.next[rdfa.alpha_remap[c]];
                if (succ != DEAD_STATE) {
                    cr.set(c);
                    next.insert(succ);
                }
            }
        }

        assert(cr.any());
        look[i] |= cr;
        curr.swap(next);
    }
}

static
void getSuffixForwardReach(const suffix_id &suff, u32 top,
                           map<s32, CharReach> &look) {
    if (suff.graph()) {
        getForwardReach(*suff.graph(), top, look);
    } else if (suff.castle()) {
        getForwardReach(*suff.castle(), top, look);
    } else if (suff.dfa()) {
        assert(top == 0); // DFA isn't multi-top capable.
        getForwardReach(*suff.dfa(), look);
    } else if (suff.haig()) {
        assert(top == 0); // DFA isn't multi-top capable.
        getForwardReach(*suff.haig(), look);
    }
}

static
void getRoseForwardReach(const left_id &left, u32 top,
                         map<s32, CharReach> &look) {
    if (left.graph()) {
        getForwardReach(*left.graph(), top, look);
    } else if (left.castle()) {
        getForwardReach(*left.castle(), top, look);
    } else if (left.dfa()) {
        assert(top == 0); // DFA isn't multi-top capable.
        getForwardReach(*left.dfa(), look);
    } else if (left.haig()) {
        assert(top == 0); // DFA isn't multi-top capable.
        getForwardReach(*left.haig(), look);
    }
}

static
void combineForwardMasks(const vector<map<s32, CharReach> > &rose_look,
                         map<s32, CharReach> &look) {
    for (u32 i = 0; i < MAX_FWD_LEN; i++) {
        for (const auto &rlook : rose_look) {
            if (contains(rlook, i)) {
                look[i] |= rlook.at(i);
            } else {
                look[i].setall();
            }
        }
    }
}

static
void findForwardReach(const RoseGraph &g, const RoseVertex v,
                      map<s32, CharReach> &look) {
    if (!g[v].reports.empty()) {
        DEBUG_PRINTF("acceptor\n");
        return;
    }

    // Non-leaf vertices can pick up a mask per successor prefix rose
    // engine.
    vector<map<s32, CharReach>> rose_look;
    for (const auto &e : out_edges_range(v, g)) {
        RoseVertex t = target(e, g);
        if (!g[t].left) {
            DEBUG_PRINTF("successor %zu has no leftfix\n", g[t].idx);
            return;
        }
        rose_look.push_back(map<s32, CharReach>());
        getRoseForwardReach(g[t].left, g[e].rose_top, rose_look.back());
    }

    if (g[v].suffix) {
        DEBUG_PRINTF("suffix engine\n");
        rose_look.push_back(map<s32, CharReach>());
        getSuffixForwardReach(g[v].suffix, g[v].suffix.top, rose_look.back());
    }

    combineForwardMasks(rose_look, look);
}

static
void findBackwardReach(const RoseGraph &g, const RoseVertex v,
                       map<s32, CharReach> &look) {
    if (!g[v].left) {
        return;
    }

    DEBUG_PRINTF("leftfix, report=%u, lag=%u\n", g[v].left.leftfix_report,
                 g[v].left.lag);

    if (g[v].left.graph) {
        getBackwardReach(*g[v].left.graph, g[v].left.leftfix_report,
                         g[v].left.lag, look);
    } else if (g[v].left.castle) {
        getBackwardReach(*g[v].left.castle, g[v].left.leftfix_report,
                         g[v].left.lag, look);
    }

    // TODO: implement DFA variants if necessary.
}

#if defined(DEBUG) || defined(DUMP_SUPPORT)
#include <sstream>
static UNUSED
string dump(const map<s32, CharReach> &look) {
    ostringstream oss;
    for (auto it = look.begin(), ite = look.end(); it != ite; ++it) {
        if (it != look.begin()) {
            oss << ", ";
        }
        oss << "{" << it->first << ": " << describeClass(it->second) << "}";
    }
    return oss.str();
}
#endif

static
void normalise(map<s32, CharReach> &look) {
    // We can erase entries where the reach is "all characters".
    vector<s32> dead;
    for (const auto &m : look) {
        if (m.second.all()) {
            dead.push_back(m.first);
        }
    }
    erase_all(&look, dead);
}

namespace {

struct LookPriority {
    explicit LookPriority(const map<s32, CharReach> &look_in) : look(look_in) {}

    bool operator()(s32 a, s32 b) const {
        const CharReach &a_reach = look.at(a);
        const CharReach &b_reach = look.at(b);
        if (a_reach.count() != b_reach.count()) {
            return a_reach.count() < b_reach.count();
        }
        return abs(a) < abs(b);
    }

private:
    const map<s32, CharReach> &look;
};

} // namespace

static
bool isFloodProne(const map<s32, CharReach> &look, const CharReach &flood_cr) {
    for (const auto &m : look) {
        const CharReach &look_cr = m.second;
        if (!overlaps(look_cr, flood_cr)) {
            return false;
        }
    }
    DEBUG_PRINTF("look can't escape flood on %s\n",
                  describeClass(flood_cr).c_str());
    return true;
}

static
bool isFloodProne(const map<s32, CharReach> &look,
                  const set<CharReach> &flood_reach) {
    if (flood_reach.empty()) {
        return false;
    }

    for (const CharReach &flood_cr : flood_reach) {
        if (isFloodProne(look, flood_cr)) {
            return true;
        }
    }

    return false;
}

static
void reduce(map<s32, CharReach> &look, set<CharReach> &flood_reach) {
    if (look.size() <= MAX_LOOKAROUND_ENTRIES) {
        return;
    }

    DEBUG_PRINTF("before reduce: %s\n", dump(look).c_str());

    // First, remove floods that we already can't escape; they shouldn't affect
    // the analysis below.
    for (auto it = flood_reach.begin(); it != flood_reach.end();) {
        if (isFloodProne(look, *it)) {
            DEBUG_PRINTF("removing inescapable flood on %s from analysis\n",
                         describeClass(*it).c_str());
            flood_reach.erase(it++);
        } else {
            ++it;
        }
    }

    LookPriority cmp(look);
    priority_queue<s32, vector<s32>, LookPriority> pq(cmp);
    for (const auto &m : look) {
        pq.push(m.first);
    }

    while (!pq.empty() && look.size() > MAX_LOOKAROUND_ENTRIES) {
        s32 d = pq.top();
        assert(contains(look, d));
        const CharReach cr(look[d]); // copy
        pq.pop();

        DEBUG_PRINTF("erasing {%d: %s}\n", d, describeClass(cr).c_str());
        look.erase(d);

        // If removing this entry would result in us becoming flood_prone on a
        // particular flood_reach case, reinstate it and move on.
        if (isFloodProne(look, flood_reach)) {
            DEBUG_PRINTF("reinstating {%d: %s} due to flood-prone check\n", d,
                         describeClass(cr).c_str());
            look.insert(make_pair(d, cr));
        }
    }

    while (!pq.empty()) {
        s32 d = pq.top();
        assert(contains(look, d));
        const CharReach cr(look[d]); // copy
        pq.pop();

        if (cr.count() < LOOKAROUND_WIDE_REACH) {
            continue;
        }

        DEBUG_PRINTF("erasing {%d: %s}\n", d, describeClass(cr).c_str());
        look.erase(d);

        // If removing this entry would result in us becoming flood_prone on a
        // particular flood_reach case, reinstate it and move on.
        if (isFloodProne(look, flood_reach)) {
            DEBUG_PRINTF("reinstating {%d: %s} due to flood-prone check\n", d,
                         describeClass(cr).c_str());
            look.insert(make_pair(d, cr));
        }
    }

    DEBUG_PRINTF("after reduce: %s\n", dump(look).c_str());
}

static
void findFloodReach(const RoseBuildImpl &tbi, const RoseVertex v,
                    set<CharReach> &flood_reach) {
    for (u32 lit_id : tbi.g[v].literals) {
        const ue2_literal &s = tbi.literals.right.at(lit_id).s;
        if (s.empty()) {
            continue;
        }
        if (is_flood(s)) {
            CharReach cr(*s.begin());
            DEBUG_PRINTF("flood-prone with reach: %s\n",
                          describeClass(cr).c_str());
            flood_reach.insert(cr);
        }
    }
}

static
map<s32, CharReach> findLiteralReach(const RoseBuildImpl &build,
                                     const RoseVertex v) {
    map<s32, CharReach> look;
    for (u32 lit_id : build.g[v].literals) {
        const rose_literal_id &lit = build.literals.right.at(lit_id);

        u32 i = lit.delay + 1;
        for (auto it = lit.s.rbegin(), ite = lit.s.rend(); it != ite; ++it) {
            look[0 - i] |= *it;
            i++;
        }
    }

    DEBUG_PRINTF("lit lookaround: %s\n", dump(look).c_str());
    return look;
}

/**
 * Trim lookaround checks from the prefix that overlap with the literals
 * themselves.
 */
static
void trimLiterals(const RoseBuildImpl &build, const RoseVertex v,
                  map<s32, CharReach> &look) {
    DEBUG_PRINTF("pre-trim lookaround: %s\n", dump(look).c_str());

    for (const auto &m : findLiteralReach(build, v)) {
        auto it = look.find(m.first);
        if (it == end(look)) {
            continue;
        }
        if (m.second.isSubsetOf(it->second)) {
            DEBUG_PRINTF("can trim entry at %d\n", it->first);
            look.erase(it);
        }
    }

    DEBUG_PRINTF("post-trim lookaround: %s\n", dump(look).c_str());
}

void findLookaroundMasks(const RoseBuildImpl &tbi, const RoseVertex v,
                         vector<LookEntry> &lookaround) {
    lookaround.clear();

    const RoseGraph &g = tbi.g;

    map<s32, CharReach> look;
    findBackwardReach(g, v, look);
    findForwardReach(g, v, look);
    trimLiterals(tbi, v, look);

    if (look.empty()) {
        return;
    }

    normalise(look);

    if (look.empty()) {
        return;
    }

    set<CharReach> flood_reach;
    findFloodReach(tbi, v, flood_reach);
    reduce(look, flood_reach);

    if (look.empty()) {
        return;
    }

    DEBUG_PRINTF("lookaround: %s\n", dump(look).c_str());
    lookaround.reserve(look.size());
    for (const auto &m : look) {
        s8 offset = verify_s8(m.first);
        lookaround.emplace_back(offset, m.second);
    }
}

static
bool getTransientPrefixReach(const NGHolder &g, u32 lag,
                             map<s32, CharReach> &look) {
    if (in_degree(g.accept, g) != 1) {
        DEBUG_PRINTF("more than one accept\n");
        return false;
    }

    // Currently we don't handle anchored prefixes, as we would need to be able
    // to represent the bounds from the anchor as well.
    if (out_degree(g.start, g) != 1) {
        DEBUG_PRINTF("anchored\n");
        return false;
    }

    if (out_degree(g.startDs, g) != 2) {
        DEBUG_PRINTF("more than one start\n");
        return false;
    }

    NFAVertex v = *(inv_adjacent_vertices(g.accept, g).first);
    u32 i = lag + 1;
    while (v != g.startDs) {
        DEBUG_PRINTF("i=%u, v=%u\n", i, g[v].index);
        if (is_special(v, g)) {
            DEBUG_PRINTF("special\n");
            return false;
        }

        look[0 - i] = g[v].char_reach;

        if (in_degree(v, g) != 1) {
            DEBUG_PRINTF("branch\n");
            return false;
        }

        v = *(inv_adjacent_vertices(v, g).first);
        i++;
    }

    DEBUG_PRINTF("done\n");
    return true;
}

static
void normaliseLeftfix(map<s32, CharReach> &look) {
    // We can erase entries where the reach is "all characters", except for the
    // very first one -- this might be required to establish a minimum bound on
    // the literal's match offset.

    // TODO: It would be cleaner to use a literal program instruction to check
    // the minimum bound explicitly.

    if (look.empty()) {
        return;
    }

    const auto earliest = begin(look)->first;

    vector<s32> dead;
    for (const auto &m : look) {
        if (m.second.all() && m.first != earliest) {
            dead.push_back(m.first);
        }
    }
    erase_all(&look, dead);
}

bool makeLeftfixLookaround(const RoseBuildImpl &build, const RoseVertex v,
                           vector<LookEntry> &lookaround) {
    lookaround.clear();

    const RoseGraph &g = build.g;
    const left_id leftfix(g[v].left);

    if (!contains(build.transient, leftfix)) {
        DEBUG_PRINTF("not transient\n");
        return false;
    }

    if (!leftfix.graph()) {
        DEBUG_PRINTF("only supported for graphs so far\n");
        return false;
    }

    map<s32, CharReach> look;
    if (!getTransientPrefixReach(*leftfix.graph(), g[v].left.lag, look)) {
        DEBUG_PRINTF("not a chain\n");
        return false;
    }

    trimLiterals(build, v, look);
    normaliseLeftfix(look);

    if (look.size() > MAX_LOOKAROUND_ENTRIES) {
        DEBUG_PRINTF("lookaround too big (%zu entries)\n", look.size());
        return false;
    }

    if (look.empty()) {
        DEBUG_PRINTF("lookaround empty; this is weird\n");
        return false;
    }

    lookaround.reserve(look.size());
    for (const auto &m : look) {
        s8 offset = verify_s8(m.first);
        lookaround.emplace_back(offset, m.second);
    }

    return true;
}

void mergeLookaround(vector<LookEntry> &lookaround,
                     const vector<LookEntry> &more_lookaround) {
    if (lookaround.size() >= MAX_LOOKAROUND_ENTRIES) {
        DEBUG_PRINTF("big enough!\n");
        return;
    }

    // Don't merge lookarounds at offsets we already have entries for.
    ue2::flat_set<s8> offsets;
    for (const auto &e : lookaround) {
        offsets.insert(e.offset);
    }

    map<s32, CharReach> more;
    LookPriority cmp(more);
    priority_queue<s32, vector<s32>, LookPriority> pq(cmp);
    for (const auto &e : more_lookaround) {
        if (!contains(offsets, e.offset)) {
            more.emplace(e.offset, e.reach);
            pq.push(e.offset);
        }
    }

    while (!pq.empty() && lookaround.size() < MAX_LOOKAROUND_ENTRIES) {
        const s32 offset = pq.top();
        pq.pop();
        const auto &cr = more.at(offset);
        DEBUG_PRINTF("added {%d,%s}\n", offset, describeClass(cr).c_str());
        lookaround.emplace_back(verify_s8(offset), cr);
    }

    // Order by offset.
    sort(begin(lookaround), end(lookaround),
         [](const LookEntry &a, const LookEntry &b) {
             return a.offset < b.offset;
         });
}

} // namespace ue2
