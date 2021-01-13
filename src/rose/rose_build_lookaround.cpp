/*
 * Copyright (c) 2015-2020, Intel Corporation
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
#include "util/flat_containers.h"
#include "util/verify_types.h"

#include <cstdlib>
#include <queue>
#include <sstream>

using namespace std;

namespace ue2 {

/** \brief Max search distance for reachability in front of a role. */
static const u32 MAX_FWD_LEN = 64;

/** \brief Max search distance for reachability behind a role. */
static const u32 MAX_BACK_LEN = 64;

/** \brief Max lookaround entries for a role. */
static const u32 MAX_LOOKAROUND_ENTRIES = 32;

/** \brief We would rather have lookarounds with smaller reach than this. */
static const u32 LOOKAROUND_WIDE_REACH = 200;

#if defined(DEBUG) || defined(DUMP_SUPPORT)
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
void getForwardReach(const NGHolder &g, u32 top, map<s32, CharReach> &look) {
    flat_set<NFAVertex> curr, next;

    // Consider only successors of start with the required top.
    for (const auto &e : out_edges_range(g.start, g)) {
        NFAVertex v = target(e, g);
        if (v == g.startDs) {
            continue;
        }
        if (contains(g[e].tops, top)) {
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
    flat_set<NFAVertex> curr, next;

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

    flat_set<dstate_id_t> curr, next;
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
            DEBUG_PRINTF("successor %zu has no leftfix\n", g[t].index);
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
        const ue2_literal &s = tbi.literals.at(lit_id).s;
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


namespace {
struct LookProto {
    LookProto(s32 offset_in, CharReach reach_in)
        : offset(offset_in), reach(move(reach_in)) {}
    s32 offset;
    CharReach reach;
};
}

static
vector<LookProto> findLiteralReach(const rose_literal_id &lit) {
    vector<LookProto> look;
    look.reserve(lit.s.length());

    s32 i = 0 - lit.s.length() - lit.delay;
    for (const auto &c : lit.s) {
        look.emplace_back(i, c);
        i++;
    }

    return look;
}

static
vector<LookProto> findLiteralReach(const RoseBuildImpl &build,
                                   const RoseVertex v) {
    bool first = true;
    vector<LookProto> look;

    for (u32 lit_id : build.g[v].literals) {
        const rose_literal_id &lit = build.literals.at(lit_id);
        auto lit_look = findLiteralReach(lit);

        if (first) {
            look = std::move(lit_look);
            first = false;
            continue;
        }

        // Erase elements from look with keys not in lit_look. Where a key is
        // in both maps, union its reach with the lookaround.
        auto jt = begin(lit_look);
        for (auto it = begin(look); it != end(look);) {
            if (jt == end(lit_look)) {
                // No further lit_look entries, erase remaining elements from
                // look.
                look.erase(it, end(look));
                break;
            }
            if (it->offset < jt->offset) {
                // Offset is present in look but not in lit_look, erase.
                it = look.erase(it);
            } else if (it->offset > jt->offset) {
                // Offset is preset in lit_look but not in look, ignore.
                ++jt;
            } else {
                // Offset is present in both, union its reach with look.
                it->reach |= jt->reach;
                ++it;
                ++jt;
            }
        }
    }

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
        auto it = look.find(m.offset);
        if (it == end(look)) {
            continue;
        }
        if (m.reach.isSubsetOf(it->second)) {
            DEBUG_PRINTF("can trim entry at %d\n", it->first);
            look.erase(it);
        }
    }

    DEBUG_PRINTF("post-trim lookaround: %s\n", dump(look).c_str());
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

static
bool trimMultipathLeftfix(const RoseBuildImpl &build, const RoseVertex v,
                          vector<map<s32, CharReach>> &looks) {
    size_t path_count = 0;
    for (auto &look : looks) {
        ++path_count;
        DEBUG_PRINTF("Path #%ld\n", path_count);

        assert(!look.empty());
        trimLiterals(build, v, look);

        if (look.empty()) {
            return false;
        }

        // Could be optimized here, just keep the empty byte of the longest path
        normaliseLeftfix(look);

        if (look.size() > MAX_LOOKAROUND_ENTRIES) {
            DEBUG_PRINTF("lookaround too big (%zu entries)\n", look.size());
            return false;
        }
    }
    return true;
}

static
void transToLookaround(const vector<map<s32, CharReach>> &looks,
                       vector<vector<LookEntry>> &lookarounds) {
    for (const auto &look : looks) {
        vector<LookEntry> lookaround;
        DEBUG_PRINTF("lookaround: %s\n", dump(look).c_str());
        lookaround.reserve(look.size());
        for (const auto &m : look) {
            if (m.first < -128 || m.first > 127) {
                DEBUG_PRINTF("range too big\n");
                lookarounds.clear();
                return;
            }
            s8 offset = verify_s8(m.first);
            lookaround.emplace_back(offset, m.second);
        }
        lookarounds.push_back(lookaround);
    }
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
bool checkShuftiBuckets(const vector<map<s32, CharReach>> &looks,
                        u32 bucket_size) {
    set<u32> bucket;
    for (const auto &look : looks) {
        for (const auto &l : look) {
            CharReach cr = l.second;
            if (cr.count() > 128) {
                cr.flip();
            }
            map <u16, u16> lo2hi;

            for (size_t i = cr.find_first(); i != CharReach::npos;) {
                u8 it_hi = i >> 4;
                u16 low_encode = 0;
                while (i != CharReach::npos && (i >> 4) == it_hi) {
                    low_encode |= 1 << (i &0xf);
                    i = cr.find_next(i);
                }
                lo2hi[low_encode] |= 1 << it_hi;
            }

            for (const auto &it : lo2hi) {
                u32 hi_lo = (it.second << 16) | it.first;
                bucket.insert(hi_lo);
            }
        }
    }
    DEBUG_PRINTF("shufti has %lu bucket(s)\n", bucket.size());
    return bucket.size() <= bucket_size;
}

static
bool getTransientPrefixReach(const NGHolder &g, ReportID report, u32 lag,
                             vector<map<s32, CharReach>> &looks) {
    if (!isAcyclic(g)) {
        DEBUG_PRINTF("contains back-edge\n");
        return false;
    }

    // Must be floating chains wired to startDs.
    if (!isFloating(g)) {
        DEBUG_PRINTF("not a floating start\n");
        return false;
    }

    vector<NFAVertex> curr;
    for (auto v : inv_adjacent_vertices_range(g.accept, g)) {
        if (v == g.start || v == g.startDs) {
            DEBUG_PRINTF("empty graph\n");
            return true;
        }
        if (contains(g[v].reports, report)) {
            curr.push_back(v);
        }
    }

    assert(!curr.empty());

    u32 total_len = curr.size();

    for (const auto &v : curr) {
        looks.emplace_back(map<s32, CharReach>());
        looks.back()[0 - (lag + 1)] = g[v].char_reach;
    }

    bool curr_active = false;

    /* For each offset -i, we backwardly trace the path by vertices in curr.
     * Once there are more than 8 paths and more than 64 bits total_len,
     * which means that neither MULTIPATH_LOOKAROUND nor MULTIPATH_SHUFTI
     * could be successfully built, we will give up the path finding.
     * Otherwise, the loop will halt when all vertices in curr are startDs.
     */
    for (u32 i = lag + 2; i < (lag + 2) + MAX_BACK_LEN; i++) {
        curr_active = false;
        size_t curr_size = curr.size();
        if (curr.size() > 1 && i > lag + MULTIPATH_MAX_LEN) {
            DEBUG_PRINTF("range is larger than 16 in multi-path\n");
            return false;
        }

        for (size_t idx = 0; idx < curr_size; idx++) {
            NFAVertex v = curr[idx];
            if (v == g.startDs) {
                continue;
            }
            assert(!is_special(v, g));

            for (auto u : inv_adjacent_vertices_range(v, g)) {
                if (u == g.start || u == g.startDs) {
                    curr[idx] = g.startDs;
                    break;
                }
            }

            if (is_special(curr[idx], g)) {
                continue;
            }

            for (auto u : inv_adjacent_vertices_range(v, g)) {
                curr_active = true;
                if (curr[idx] == v) {
                    curr[idx] = u;
                    looks[idx][0 - i] = g[u].char_reach;
                    total_len++;
                } else {
                    curr.push_back(u);
                    looks.push_back(looks[idx]);
                    (looks.back())[0 - i] = g[u].char_reach;
                    total_len += looks.back().size();
                }

                if (curr.size() > MAX_LOOKAROUND_PATHS && total_len > 64) {
                    DEBUG_PRINTF("too many branches\n");
                    return false;
                }
            }
        }
        if (!curr_active) {
            break;
        }
    }

    if (curr_active) {
        DEBUG_PRINTF("single path too long\n");
        return false;
    }

    // More than 8 paths, check multi-path shufti.
    if (curr.size() > MAX_LOOKAROUND_PATHS) {
        u32 bucket_size = total_len > 32 ? 8 : 16;
        if (!checkShuftiBuckets(looks, bucket_size)) {
            DEBUG_PRINTF("shufti has too many buckets\n");
            return false;
        }
    }

    assert(!looks.empty());
    if (looks.size() == 1) {
        DEBUG_PRINTF("single lookaround\n");
    } else {
        DEBUG_PRINTF("multi-path lookaround\n");
    }
    DEBUG_PRINTF("done\n");
    return true;
}

bool makeLeftfixLookaround(const RoseBuildImpl &build, const RoseVertex v,
                           vector<vector<LookEntry>> &lookaround) {
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

    vector<map<s32, CharReach>> looks;
    if (!getTransientPrefixReach(*leftfix.graph(), g[v].left.leftfix_report,
                                 g[v].left.lag, looks)) {
        DEBUG_PRINTF("graph has loop or too large\n");
        return false;
    }

    if (!trimMultipathLeftfix(build, v, looks)) {
        return false;
    }
    transToLookaround(looks, lookaround);

    return !lookaround.empty();
}

void mergeLookaround(vector<LookEntry> &lookaround,
                     const vector<LookEntry> &more_lookaround) {
    if (lookaround.size() >= MAX_LOOKAROUND_ENTRIES) {
        DEBUG_PRINTF("big enough!\n");
        return;
    }

    // Don't merge lookarounds at offsets we already have entries for.
    flat_set<s8> offsets;
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
