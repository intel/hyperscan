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

/** \file
 * \brief NFA acceleration analysis code.
 */
#include "ng_limex_accel.h"

#include "ng_holder.h"
#include "ng_misc_opt.h"
#include "ng_util.h"
#include "ue2common.h"

#include "nfa/accel.h"
#include "nfa/multiaccel_compilehelper.h"

#include "util/bitutils.h" // for CASE_CLEAR
#include "util/charreach.h"
#include "util/compile_context.h"
#include "util/container.h"
#include "util/dump_charclass.h"
#include "util/graph_range.h"
#include "util/target_info.h"

#include <algorithm>
#include <map>

using namespace std;

namespace ue2 {

#define WIDE_FRIEND_MIN 200

static
void findAccelFriendGeneration(const NGHolder &g, const CharReach &cr,
                               const flat_set<NFAVertex> &cands,
                               const flat_set<NFAVertex> &preds,
                               flat_set<NFAVertex> *next_cands,
                               flat_set<NFAVertex> *next_preds,
                               flat_set<NFAVertex> *friends) {
    for (auto v : cands) {
        if (contains(preds, v)) {
            continue;
        }

        const CharReach &acr = g[v].char_reach;
        DEBUG_PRINTF("checking %u\n", g[v].index);

        if (acr.count() < WIDE_FRIEND_MIN || !acr.isSubsetOf(cr)) {
            DEBUG_PRINTF("bad reach %zu\n", acr.count());
            continue;
        }

        for (auto u : inv_adjacent_vertices_range(v, g)) {
            if (!contains(preds, u)) {
                DEBUG_PRINTF("bad pred\n");
                goto next_cand;
            }
        }

        next_preds->insert(v);
        insert(next_cands, adjacent_vertices(v, g));

        DEBUG_PRINTF("%u is a friend indeed\n", g[v].index);
        friends->insert(v);
    next_cand:;
    }
}

void findAccelFriends(const NGHolder &g, NFAVertex v,
                      const map<NFAVertex, BoundedRepeatSummary> &br_cyclic,
                      u32 offset, flat_set<NFAVertex> *friends) {
    /* A friend of an accel state is a successor state which can only be on when
     * the accel is on. This requires that it has a subset of the accel state's
     * preds and a charreach which is a subset of the accel state.
     *
     * A friend can be safely ignored when accelerating provided there is
     * sufficient back-off. A friend is useful if it has a wide reach.
     */

    /* BR cyclic states which may go stale cannot have friends as they may
     * suddenly turn off leading their so-called friends stranded and alone.
     * TODO: restrict to only stale going BR cyclics
     */
    if (contains(br_cyclic, v) && !br_cyclic.at(v).unbounded()) {
        return;
    }

    u32 friend_depth = offset + 1;

    flat_set<NFAVertex> preds;
    insert(&preds, inv_adjacent_vertices(v, g));
    const CharReach &cr = g[v].char_reach;

    flat_set<NFAVertex> cands;
    insert(&cands, adjacent_vertices(v, g));

    flat_set<NFAVertex> next_preds;
    flat_set<NFAVertex> next_cands;
    for (u32 i = 0; i < friend_depth; i++) {
        findAccelFriendGeneration(g, cr, cands, preds, &next_cands, &next_preds,
                                  friends);
        preds.insert(next_preds.begin(), next_preds.end());
        next_preds.clear();
        cands.swap(next_cands);
        next_cands.clear();
    }
}

static
void buildTwoByteStops(flat_set<pair<u8, u8>> &twobyte, const CharReach &cr1,
                       const CharReach &cr2) {
    for (size_t c1 = cr1.find_first(); c1 != cr1.npos; c1 = cr1.find_next(c1)) {
        for (size_t c2 = cr2.find_first(); c2 != cr2.npos;
             c2 = cr2.find_next(c2)) {
            twobyte.emplace((u8)c1, (u8)c2);
        }
    }
}

static
void findStopLiteralsAtVertex(NFAVertex v, const NGHolder &g,
                              DoubleAccelInfo &build) {
    DEBUG_PRINTF("state %u\n", g[v].index);

    // double-byte accel is possible: calculate all single- and double-byte
    // accel literals.
    const CharReach &cr1 = g[v].char_reach;

    if (edge(v, g.accept, g).second) {
        // If this first byte is an accept state, it must contribute a
        // single-byte escape. We can still go on and calculate additional
        // double-byte ones, though.
        /* TODO: fix for rose */
        build.stop1 |= cr1;
    }

    flat_set<pair<u8, u8>> twobyte; // for just this starting state
    bool single = false;

    for (auto w : adjacent_vertices_range(v, g)) {
        if (w == g.accept || w == g.acceptEod) {
            continue;
        }
        const CharReach &cr2 = g[w].char_reach;
        size_t count = cr1.count() * cr2.count() + build.stop2.size();
        if (count > 0 && count <= 8) { // can't do more than 8 two-byte
            buildTwoByteStops(twobyte, cr1, cr2);
        } else {
            // two many two-byte literals, add the first byte as single
            single = true;
            break;
        }
    }

    if (single || twobyte.empty()) {
        assert(!cr1.none());
        build.stop1 |= cr1;
    } else {
        assert(!twobyte.empty());
        build.stop2.insert(twobyte.begin(), twobyte.end());
    }
}

static
bool is_bit5_insensitive(const flat_set<pair<u8, u8>> &stop) {
    if (stop.size() != 4) {
        return false;
    }

    const u8 a = stop.begin()->first & CASE_CLEAR;
    const u8 b = stop.begin()->second & CASE_CLEAR;

    for (flat_set<pair<u8, u8>>::const_iterator it = stop.begin();
         it != stop.end(); ++it) {
        if ((it->first & CASE_CLEAR) != a || (it->second & CASE_CLEAR) != b) {
            return false;
        }
    }

    return true;
}

static
bool is_dverm(const DoubleAccelInfo &a) {
    if (a.stop1.any()) {
        return false;
    }

    if (a.stop2.size() == 1) {
        return true;
    }

    return is_bit5_insensitive(a.stop2);
}

static
bool is_double_better(const DoubleAccelInfo &a, const DoubleAccelInfo &b) {
    /* Note: this is not an operator< */

    if (a.stop2.empty()) {
        return false;
    }

    if (b.stop2.empty()) {
        return true;
    }

    if (a.stop1.count() > b.stop1.count()) {
        return false;
    }

    if (a.stop1.count() < b.stop1.count()) {
        return true;
    }

    bool a_dvm = is_dverm(a);
    bool b_dvm = is_dverm(b);

    if (b_dvm && !a_dvm) {
        return false;
    }

    if (!b_dvm && a_dvm) {
        return true;
    }

    if (a.stop2.size() > b.stop2.size()) {
        return false;
    }

    if (a.stop2.size() < b.stop2.size()) {
        return true;
    }

    return a.offset < b.offset;
}

/** \brief Find the escape literals for a two byte accel at the given accel
 * offset */
static
void findDoubleAccel(const NGHolder &g, NFAVertex v, u32 accel_offset,
                     DoubleAccelInfo &build) {
    DEBUG_PRINTF("find double accel +%u for vertex %u\n", accel_offset,
                  g[v].index);
    build.offset = accel_offset;

    // Our accel state contributes single-byte escapes
    build.stop1 |= ~g[v].char_reach;

    flat_set<NFAVertex> searchStates; // states that contribute stop literals
    searchStates.insert(v); /* TODO: verify */

    /* Note: We cannot search past an accepting state */
    /* TODO: remove restriction for non-callback generating */
    flat_set<NFAVertex> nextStates;

    insert(&nextStates, adjacent_vertices(v, g));
    nextStates.erase(v);
    nextStates.erase(g.accept);
    nextStates.erase(g.acceptEod);

    searchStates.swap(nextStates);
    nextStates.clear();

    // subsequent iterations are simpler, just follow all edges
    for (u32 j = 1; j <= accel_offset; j++) {
        for (auto u : searchStates) {
            insert(&nextStates, adjacent_vertices(u, g));
            if (edge(u, g.accept, g).second) {
                nextStates.clear();
                break;
            }
            nextStates.erase(g.accept);
            nextStates.erase(g.acceptEod);
        }

        searchStates.swap(nextStates);
        nextStates.clear();
    }

    vector<NFAVertex> sorted;
    insert(&sorted, sorted.end(), searchStates);
    sort(sorted.begin(), sorted.end(), make_index_ordering(g));
    for (auto sv : sorted) {
        findStopLiteralsAtVertex(sv, g, build);
    }
}

DoubleAccelInfo findBestDoubleAccelInfo(const NGHolder &g, NFAVertex v) {
    DoubleAccelInfo rv;
    for (u32 offset = 0; offset <= MAX_ACCEL_DEPTH; offset++) {
        DoubleAccelInfo b_temp;
        findDoubleAccel(g, v, offset, b_temp);
        if (is_double_better(b_temp, rv)) {
            rv = b_temp;
        }
    }

    return rv;
}

static
void findPaths(const NGHolder &g, NFAVertex v,
               const vector<CharReach> &refined_cr,
               vector<vector<CharReach> > *paths,
               const flat_set<NFAVertex> &forbidden, u32 depth) {
    static const u32 MAGIC_TOO_WIDE_NUMBER = 16;
    if (!depth) {
        paths->push_back(vector<CharReach>());
        return;
    }
    if (v == g.accept || v == g.acceptEod) {
        paths->push_back(vector<CharReach>());
        if (!generates_callbacks(g) || v == g.acceptEod) {
            paths->back().push_back(CharReach()); /* red tape options */
        }
        return;
    }

    /* for the escape 'literals' we want to use the minimal cr so we
     * can be more selective */
    const CharReach &cr = refined_cr[g[v].index];

    if (out_degree(v, g) >= MAGIC_TOO_WIDE_NUMBER
        || hasSelfLoop(v, g)) {
        /* give up on pushing past this point */
        paths->push_back(vector<CharReach>());
        vector<CharReach> &p = paths->back();
        p.push_back(cr);
        return;
    }

    for (auto w : adjacent_vertices_range(v, g)) {
        if (contains(forbidden, w)) {
            /* path has looped back to one of the active+boring acceleration
             * states.  We can ignore this path if we have sufficient back-
             * off. */
            paths->push_back(vector<CharReach>());
            paths->back().push_back(CharReach());
            continue;
        }

        u32 new_depth = depth - 1;
        vector<vector<CharReach> > curr;
        do {
            curr.clear();
            findPaths(g, w, refined_cr, &curr, forbidden, new_depth);
        } while (new_depth-- && curr.size() >= MAGIC_TOO_WIDE_NUMBER);

        for (vector<vector<CharReach> >::iterator it = curr.begin();
             it != curr.end(); ++it) {
            paths->push_back(vector<CharReach>());
            vector<CharReach> &p = paths->back();
            p.swap(*it);
            p.push_back(cr);
        }
    }
}

static
AccelScheme merge(const AccelScheme &a, const AccelScheme &b) {
    return AccelScheme(a.cr | b.cr, MAX(a.offset, b.offset));
}

static
void findBest(vector<vector<CharReach> >::const_iterator pb,
              vector<vector<CharReach> >::const_iterator pe,
              const AccelScheme &curr, AccelScheme *best) {
    assert(curr.offset <= MAX_ACCEL_DEPTH);
    DEBUG_PRINTF("paths left %zu\n", pe - pb);
    if (pb == pe) {
        *best = curr;
        return;
    }

    DEBUG_PRINTF("p len %zu\n", pb->end() - pb->begin());

    vector<AccelScheme> priority_path;
    u32 i = 0;
    for (vector<CharReach>::const_iterator p = pb->begin(); p != pb->end();
         ++p, i++) {
        priority_path.push_back(AccelScheme(*p & ~curr.cr, i));
    }

    sort(priority_path.begin(), priority_path.end());
    for (vector<AccelScheme>::iterator it = priority_path.begin();
         it != priority_path.end(); ++it) {
        vector<AccelScheme>::iterator jt = it + 1;
        for (; jt != priority_path.end(); ++jt) {
            if (!it->cr.isSubsetOf(jt->cr)) {
                break;
            }
        }
        priority_path.erase(it + 1, jt);
        DEBUG_PRINTF("||%zu\n", it->cr.count());
    }
    DEBUG_PRINTF("---\n");

    for (vector<AccelScheme>::const_iterator it = priority_path.begin();
         it != priority_path.end(); ++it) {
        DEBUG_PRINTF("%u:|| = %zu; p remaining len %zu\n", i, it->cr.count(),
                     priority_path.end() - it);

        AccelScheme in = merge(curr, *it);

        if (in > *best) {
            DEBUG_PRINTF("worse\n");
            continue;
        }
        AccelScheme temp = *best;
        findBest(pb + 1, pe, in, &temp);
        if (temp < *best) {
            DEBUG_PRINTF("new best\n");
            *best = temp;
            if (curr.cr == best->cr) {
                return; /* could only get better by offset */
            }
        }
    }
}

#ifdef DEBUG

static
void dumpPaths(const vector<vector<CharReach> > &paths) {
    for (vector<vector<CharReach> >::const_iterator p = paths.begin();
         p != paths.end(); ++p) {
        DEBUG_PRINTF("path: [");
        for (vector<CharReach>::const_iterator it = p->begin(); it != p->end();
             ++it) {
            printf(" [");
            describeClass(stdout, *it, 20, CC_OUT_TEXT);
            printf("]");
        }
        printf(" ]\n");
    }
}
#endif

static
void blowoutPathsLessStrictSegment(vector<vector<CharReach> > &paths) {
    /* paths segments which are a superset of an earlier segment should never be
     * picked as an acceleration segment -> to improve processing just replace
     * with dot */
    for (auto &p : paths) {
        for (auto it = p.begin(); it != p.end();  ++it) {
            for (auto jt = next(it); jt != p.end(); ++jt) {
                if (it->isSubsetOf(*jt)) {
                    *jt = CharReach::dot();
                }
            }
        }
    }
}

static
void unifyPathsLastSegment(vector<vector<CharReach> > &paths) {
    /* try to unify paths which only differ in the last segment */
    for (vector<vector<CharReach> >::iterator p = paths.begin();
         p != paths.end() && p + 1 != paths.end();) {
        vector<CharReach> &a = *p;
        vector<CharReach> &b = *(p + 1);

        if (a.size() != b.size()) {
            ++p;
            continue;
        }

        u32 i = 0;
        for (; i < a.size() - 1; i++) {
            if (a[i] != b[i]) {
                break;
            }
        }
        if (i == a.size() - 1) {
            /* we can unify these paths */
            a[i] |= b[i];
            paths.erase(p + 1);
        } else {
            ++p;
        }
    }
}

static
void improvePaths(vector<vector<CharReach> > &paths) {
#ifdef DEBUG
    DEBUG_PRINTF("orig paths\n");
    dumpPaths(paths);
#endif
    blowoutPathsLessStrictSegment(paths);

    sort(paths.begin(), paths.end());

    unifyPathsLastSegment(paths);

#ifdef DEBUG
    DEBUG_PRINTF("opt paths\n");
    dumpPaths(paths);
#endif
}

AccelScheme findBestAccelScheme(vector<vector<CharReach> > paths,
                                const CharReach &terminating) {
    improvePaths(paths);

    DEBUG_PRINTF("we have %zu paths\n", paths.size());
    if (paths.size() > 40) {
        return AccelScheme(); /* too many paths to explore */
    }

    /* if we were smart we would do something netflowy on the paths to find the
     * best cut. But we aren't, so we will just brute force it.
     */
    AccelScheme curr(terminating, 0U);
    AccelScheme best;
    findBest(paths.begin(), paths.end(), curr, &best);

    /* find best is a bit lazy in terms of minimising the offset, see if we can
     * make it better. need to find the min max offset that we need.*/
    u32 offset = 0;
    for (vector<vector<CharReach> >::iterator p = paths.begin();
         p != paths.end(); ++p) {
        u32 i = 0;
        for (vector<CharReach>::iterator it = p->begin(); it != p->end();
             ++it, i++) {
            if (it->isSubsetOf(best.cr)) {
                break;
            }
        }
        offset = MAX(offset, i);
    }
    assert(offset <= best.offset);
    best.offset = offset;

    return best;
}

AccelScheme nfaFindAccel(const NGHolder &g, const vector<NFAVertex> &verts,
                         const vector<CharReach> &refined_cr,
                         const map<NFAVertex, BoundedRepeatSummary> &br_cyclic,
                         bool allow_wide) {
    CharReach terminating;
    for (auto v : verts) {
        if (!hasSelfLoop(v, g)) {
            DEBUG_PRINTF("no self loop\n");
            return AccelScheme(); /* invalid scheme */
        }

        // check that this state is reachable on most characters
        terminating |= ~g[v].char_reach;
    }

    DEBUG_PRINTF("set vertex has %zu stop chars\n", terminating.count());
    size_t limit = allow_wide ? ACCEL_MAX_FLOATING_STOP_CHAR
                              : ACCEL_MAX_STOP_CHAR;
    if (terminating.count() > limit) {
        return AccelScheme(); /* invalid scheme */
    }

    vector<vector<CharReach> > paths;
    flat_set<NFAVertex> ignore_vert_set(verts.begin(), verts.end());

    /* Note: we can not in general (TODO: ignore when possible) ignore entries
     * into the bounded repeat cyclic states as that is when the magic happens
     */
    for (map<NFAVertex, BoundedRepeatSummary>::const_iterator it
             = br_cyclic.begin();
         it != br_cyclic.end(); ++it) {
        /* TODO: can allow if repeatMin <= 1 ? */
        ignore_vert_set.erase(it->first);
    }

    for (auto v : verts) {
        for (auto w : adjacent_vertices_range(v, g)) {
            if (w != v) {
                findPaths(g, w, refined_cr, &paths, ignore_vert_set,
                          MAX_ACCEL_DEPTH);
            }
        }
    }

    /* paths built wrong: reverse them */
    for (vector<vector<CharReach> >::iterator it = paths.begin();
         it != paths.end(); ++it) {
        reverse(it->begin(), it->end());
    }

    return findBestAccelScheme(std::move(paths), terminating);
}

NFAVertex get_sds_or_proxy(const NGHolder &g) {
    DEBUG_PRINTF("looking for sds proxy\n");
    if (proper_out_degree(g.startDs, g)) {
        return g.startDs;
    }

    NFAVertex v = NFAGraph::null_vertex();
    for (auto w : adjacent_vertices_range(g.start, g)) {
        if (w != g.startDs) {
            if (!v) {
                v = w;
            } else {
                return g.startDs;
            }
        }
    }

    if (!v) {
        return g.startDs;
    }

    while (true) {
        if (hasSelfLoop(v, g)) {
            DEBUG_PRINTF("woot %u\n", g[v].index);
            return v;
        }
        if (out_degree(v, g) != 1) {
            break;
        }
        NFAVertex u = getSoleDestVertex(g, v);
        if (!g[u].char_reach.all()) {
            break;
        }
        v = u;
    }

    return g.startDs;
}

static
NFAVertex find_next(const NFAVertex v, const NGHolder &g) {
    NFAVertex res = NFAGraph::null_vertex();
    for (NFAVertex u :  adjacent_vertices_range(v, g)) {
        if (u != v) {
            res = u;
            break;
        }
    }
    return res;
}

/** \brief Check if vertex \a v is a multi accelerable state (for a limex NFA). */
MultibyteAccelInfo nfaCheckMultiAccel(const NGHolder &g,
                                      const vector<NFAVertex> &states,
                                      const CompileContext &cc) {
    // For a set of states to be accelerable, we basically have to have only
    // one state to accelerate.
    if (states.size() != 1) {
        DEBUG_PRINTF("can't accelerate multiple states\n");
        return MultibyteAccelInfo();
    }

    // Get our base vertex
    NFAVertex v = states[0];

    // We need the base vertex to be a self-looping dotall leading to exactly
    // one vertex.
    if (!hasSelfLoop(v, g)) {
        DEBUG_PRINTF("base vertex has self-loop\n");
        return MultibyteAccelInfo();
    }

    if (!g[v].char_reach.all()) {
        DEBUG_PRINTF("can't accelerate anything but dot\n");
        return MultibyteAccelInfo();
    }

    if (proper_out_degree(v, g) != 1) {
        DEBUG_PRINTF("can't accelerate states with multiple successors\n");
        return MultibyteAccelInfo();
    }

    // find our start vertex
    NFAVertex cur = find_next(v, g);
    if (cur == NFAGraph::null_vertex()) {
        DEBUG_PRINTF("invalid start vertex\n");
        return MultibyteAccelInfo();
    }

    bool has_offset = false;
    u32 offset = 0;
    CharReach cr = g[cur].char_reach;

    // if we start with a dot, we have an offset, so defer figuring out the
    // real CharReach for this accel scheme
    if (cr == CharReach::dot()) {
        has_offset = true;
        offset = 1;
    }

    // figure out our offset
    while (has_offset) {
        // vertices have to have no self loops
        if (hasSelfLoop(cur, g)) {
            DEBUG_PRINTF("can't have self-loops\n");
            return MultibyteAccelInfo();
        }

        // we have to have exactly 1 successor to have this acceleration scheme
        if (out_degree(cur, g) != 1) {
            DEBUG_PRINTF("can't have multiple successors\n");
            return MultibyteAccelInfo();
        }

        cur = *adjacent_vertices(cur, g).first;

        // if we met a special vertex, bail out
        if (is_special(cur, g)) {
            DEBUG_PRINTF("can't have special vertices\n");
            return MultibyteAccelInfo();
        }

        // now, get the real char reach
        if (g[cur].char_reach != CharReach::dot()) {
            cr = g[cur].char_reach;
            has_offset = false;
        } else {
            offset++;
        }
    }

    // now, fire up the compilation machinery
    target_t ti = cc.target_info;
    unsigned max_len = ti.has_avx2() ? MULTIACCEL_MAX_LEN_AVX2 : MULTIACCEL_MAX_LEN_SSE;
    MultiaccelCompileHelper mac(cr, offset, max_len);

    while (mac.canAdvance()) {
        // vertices have to have no self loops
        if (hasSelfLoop(cur, g)) {
            break;
        }

        // we have to have exactly 1 successor to have this acceleration scheme
        if (out_degree(cur, g) != 1) {
            break;
        }

        cur = *adjacent_vertices(cur, g).first;

        // if we met a special vertex, bail out
        if (is_special(cur, g)) {
            break;
        }

        mac.advance(g[cur].char_reach);
    }
    MultibyteAccelInfo mai = mac.getBestScheme();
#ifdef DEBUG
    DEBUG_PRINTF("Multibyte acceleration scheme: type: %u offset: %u lengths: %u,%u\n",
                 mai.type, mai.offset, mai.len1, mai.len2);
    for (size_t c = mai.cr.find_first(); c != CharReach::npos; c = mai.cr.find_next(c)) {
        DEBUG_PRINTF("multibyte accel char: %zu\n", c);
    }
#endif
    return mai;
}

/** \brief Check if vertex \a v is an accelerable state (for a limex NFA). */
bool nfaCheckAccel(const NGHolder &g, NFAVertex v,
                   const vector<CharReach> &refined_cr,
                   const map<NFAVertex, BoundedRepeatSummary> &br_cyclic,
                   AccelScheme *as, bool allow_wide) {
    // For a state to be accelerable, our current criterion is that it be a
    // large character class with a self-loop and narrow set of possible other
    // successors (i.e. no special successors, union of successor reachability
    // is small).
    if (!hasSelfLoop(v, g)) {
        return false;
    }

    // check that this state is reachable on most characters
    /* we want to use the maximal reach here (in the graph) */
    CharReach terminating = g[v].char_reach;
    terminating.flip();

    DEBUG_PRINTF("vertex %u is cyclic and has %zu stop chars%s\n",
                 g[v].index, terminating.count(),
                 allow_wide ? " (w)" : "");

    size_t limit = allow_wide ? ACCEL_MAX_FLOATING_STOP_CHAR
                              : ACCEL_MAX_STOP_CHAR;
    if (terminating.count() > limit) {
        DEBUG_PRINTF("too leaky\n");
        return false;
    }

    flat_set<NFAVertex> curr, next;

    insert(&curr, adjacent_vertices(v, g));
    curr.erase(v); // erase self-loop

    // We consider offsets of zero through three; this is fairly arbitrary at
    // present and could probably be increased (FIXME)
    /* WARNING: would/could do horrible things to compile time */
    bool stop = false;
    vector<CharReach> depthReach(MAX_ACCEL_DEPTH);
    unsigned int depth;
    for (depth = 0; !stop && depth < MAX_ACCEL_DEPTH; depth++) {
        CharReach &cr = depthReach[depth];
        for (auto t : curr) {
            if (is_special(t, g)) {
                // We've bumped into the edge of the graph, so we should stop
                // searching.
                // Exception: iff our cyclic state is not a dot, than we can
                // safely accelerate towards an EOD accept.

                /* Exception: nfas that don't generate callbacks so accepts are
                 * fine too */
                if (t == g.accept && !generates_callbacks(g)) {
                    stop = true; // don't search beyond this depth
                    continue;
                } else if (t == g.accept) {
                    goto depth_done;
                }

                assert(t == g.acceptEod);
                stop = true; // don't search beyond this depth
            } else {
                // Non-special vertex
                insert(&next, adjacent_vertices(t, g));
                /* for the escape 'literals' we want to use the minimal cr so we
                 * can be more selective */
                cr |= refined_cr[g[t].index];
            }
        }

        cr |= terminating;
        DEBUG_PRINTF("depth %u has unioned reach %zu\n", depth, cr.count());

        curr.swap(next);
        next.clear();
    }

depth_done:

    if (depth == 0) {
        return false;
    }

    DEBUG_PRINTF("selecting from depth 0..%u\n", depth);

    /* Look for the most awesome acceleration evar */
    for (unsigned int i = 0; i < depth; i++) {
        if (depthReach[i].none()) {
            DEBUG_PRINTF("red tape acceleration engine depth %u\n", i);
            *as = AccelScheme(CharReach(), i);
            return true;
        }
    }

    // First, loop over our depths and see if we have a suitable 2-byte
    // caseful vermicelli option: this is the (second) fastest accel we have
    if (depth > 1) {
        for (unsigned int i = 0; i < (depth - 1); i++) {
            const CharReach &cra = depthReach[i];
            const CharReach &crb = depthReach[i + 1];
            if ((cra.count() == 1 && crb.count() == 1)
                || (cra.count() == 2 && crb.count() == 2
                    && cra.isBit5Insensitive() && crb.isBit5Insensitive())) {
                DEBUG_PRINTF("two-byte vermicelli, depth %u\n", i);
                *as = AccelScheme(CharReach::dot(), i);
                return true;
            }
        }
    }

    // Second option: a two-byte shufti (i.e. less than eight 2-byte
    // literals)
    if (depth > 1) {
        for (unsigned int i = 0; i < (depth - 1); i++) {
            if (depthReach[i].count()*depthReach[i+1].count() <= 8) {
                DEBUG_PRINTF("two-byte shufti, depth %u\n", i);
                *as = AccelScheme(CharReach::dot(), i);
                return true;
            }
        }
    }

    // Look for one byte accel schemes verm/shufti;
    vector<NFAVertex> verts(1, v);
    *as = nfaFindAccel(g, verts, refined_cr, br_cyclic, allow_wide);
    DEBUG_PRINTF("as width %zu\n", as->cr.count());
    return as->cr.count() <= ACCEL_MAX_STOP_CHAR || allow_wide;
}

} // namespace ue2
