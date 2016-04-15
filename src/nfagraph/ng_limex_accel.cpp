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

struct SAccelScheme {
    SAccelScheme(const CharReach &cr_in, u32 offset_in)
        : cr(cr_in), offset(offset_in) {
        assert(offset <= MAX_ACCEL_DEPTH);
    }

    SAccelScheme() {}

    bool operator<(const SAccelScheme &b) const {
        const SAccelScheme &a = *this;

        const size_t a_count = cr.count(), b_count = b.cr.count();
        if (a_count != b_count) {
            return a_count < b_count;
        }

        /* TODO: give bonus if one is a 'caseless' character */
        ORDER_CHECK(offset);
        ORDER_CHECK(cr);
        return false;
    }

    CharReach cr = CharReach::dot();
    u32 offset = MAX_ACCEL_DEPTH + 1;
};

static
void findBest(vector<vector<CharReach> >::const_iterator pb,
              vector<vector<CharReach> >::const_iterator pe,
              const SAccelScheme &curr, SAccelScheme *best) {
    assert(curr.offset <= MAX_ACCEL_DEPTH);
    DEBUG_PRINTF("paths left %zu\n", pe - pb);
    if (pb == pe) {
        if (curr < *best) {
            DEBUG_PRINTF("new best\n");
            *best = curr;
        }
        *best = curr;
        return;
    }

    DEBUG_PRINTF("p len %zu\n", pb->end() - pb->begin());

    vector<SAccelScheme> priority_path;
    priority_path.reserve(pb->size());
    u32 i = 0;
    for (vector<CharReach>::const_iterator p = pb->begin(); p != pb->end();
         ++p, i++) {
        SAccelScheme as(*p | curr.cr, MAX(i, curr.offset));
        if (*best < as) {
            DEBUG_PRINTF("worse\n");
            continue;
        }
        priority_path.push_back(move(as));
    }

    sort(priority_path.begin(), priority_path.end());
    for (auto it = priority_path.begin(); it != priority_path.end(); ++it) {
        auto jt = next(it);
        for (; jt != priority_path.end(); ++jt) {
            if (!it->cr.isSubsetOf(jt->cr)) {
                break;
            }
        }
        priority_path.erase(next(it), jt);
        DEBUG_PRINTF("||%zu\n", it->cr.count());
    }
    DEBUG_PRINTF("---\n");

    for (vector<SAccelScheme>::const_iterator it = priority_path.begin();
         it != priority_path.end(); ++it) {
        DEBUG_PRINTF("%u:|| = %zu; p remaining len %zu\n", i, it->cr.count(),
                     priority_path.end() - it);

        SAccelScheme in = move(*it);

        if (*best < in) {
            DEBUG_PRINTF("worse\n");
            continue;
        }
        findBest(pb + 1, pe, in, best);

        if (curr.cr == best->cr) {
            return; /* could only get better by offset */
        }
    }
}

struct DAccelScheme {
    DAccelScheme(const CharReach &cr_in, u32 offset_in)
        : double_cr(cr_in), double_offset(offset_in) {
        assert(double_offset <= MAX_ACCEL_DEPTH);
    }

    bool operator<(const DAccelScheme &b) const {
        const DAccelScheme &a = *this;

        size_t a_dcount = a.double_cr.count();
        size_t b_dcount = b.double_cr.count();

        assert(!a.double_byte.empty() || a_dcount || a.double_offset);
        assert(!b.double_byte.empty() || b_dcount || b.double_offset);

        if (a_dcount != b_dcount) {
            return a_dcount < b_dcount;
        }

        if (!a_dcount) {
            bool cd_a = buildDvermMask(a.double_byte);
            bool cd_b = buildDvermMask(b.double_byte);
            if (cd_a != cd_b) {
                return cd_a > cd_b;
            }
        }

        ORDER_CHECK(double_byte.size());
        ORDER_CHECK(double_offset);

        /* TODO: give bonus if one is a 'caseless' character */
        ORDER_CHECK(double_byte);
        ORDER_CHECK(double_cr);

        return false;
    }

    ue2::flat_set<std::pair<u8, u8> > double_byte;
    CharReach double_cr;
    u32 double_offset = 0;
};

static
DAccelScheme make_double_accel(DAccelScheme as, CharReach cr_1,
                               const CharReach &cr_2_in, u32 offset_in) {
    cr_1 &= ~as.double_cr;
    CharReach cr_2 = cr_2_in & ~as.double_cr;
    u32 offset = offset_in;

    if (cr_1.none()) {
        DEBUG_PRINTF("empty first element\n");
        ENSURE_AT_LEAST(&as.double_offset, offset);
        return as;
    }

    if (cr_2_in != cr_2 || cr_2.none()) {
        offset = offset_in + 1;
    }

    size_t two_count = cr_1.count() * cr_2.count();

    DEBUG_PRINTF("will generate raw %zu pairs\n", two_count);

    if (!two_count) {
        DEBUG_PRINTF("empty element\n");
        ENSURE_AT_LEAST(&as.double_offset, offset);
        return as;
    }

    if (two_count > DOUBLE_SHUFTI_LIMIT) {
        if (cr_2.count() < cr_1.count()) {
            as.double_cr |= cr_2;
            offset = offset_in + 1;
        } else {
            as.double_cr |= cr_1;
        }
    } else {
        for (auto i = cr_1.find_first(); i != CharReach::npos;
             i = cr_1.find_next(i)) {
            for (auto j = cr_2.find_first(); j != CharReach::npos;
                 j = cr_2.find_next(j)) {
                as.double_byte.emplace(i, j);
            }
        }
    }

    ENSURE_AT_LEAST(&as.double_offset, offset);
    DEBUG_PRINTF("construct da %zu pairs, %zu singles, offset %u\n",
                 as.double_byte.size(), as.double_cr.count(), as.double_offset);
    return as;
}

static
void findDoubleBest(vector<vector<CharReach> >::const_iterator pb,
              vector<vector<CharReach> >::const_iterator pe,
              const DAccelScheme &curr, DAccelScheme *best) {
    assert(curr.double_offset <= MAX_ACCEL_DEPTH);
    DEBUG_PRINTF("paths left %zu\n", pe - pb);
    DEBUG_PRINTF("current base: %zu pairs, %zu singles, offset %u\n",
                 curr.double_byte.size(), curr.double_cr.count(),
                 curr.double_offset);
    if (pb == pe) {
        if (curr < *best) {
            *best = curr;
            DEBUG_PRINTF("new best: %zu pairs, %zu singles, offset %u\n",
                         best->double_byte.size(), best->double_cr.count(),
                         best->double_offset);
        }
        return;
    }

    DEBUG_PRINTF("p len %zu\n", pb->end() - pb->begin());

    vector<DAccelScheme> priority_path;
    priority_path.reserve(pb->size());
    u32 i = 0;
    for (vector<CharReach>::const_iterator p = pb->begin();
         p != pb->end() && next(p) != pb->end();
         ++p, i++) {
        DAccelScheme as = make_double_accel(curr, *p, *next(p), i);
        if (*best < as) {
            DEBUG_PRINTF("worse\n");
            continue;
        }
        priority_path.push_back(move(as));
    }

    sort(priority_path.begin(), priority_path.end());
    DEBUG_PRINTF("%zu candidates for this path\n", priority_path.size());
    DEBUG_PRINTF("input best: %zu pairs, %zu singles, offset %u\n",
                 best->double_byte.size(), best->double_cr.count(),
                 best->double_offset);

    for (vector<DAccelScheme>::const_iterator it = priority_path.begin();
         it != priority_path.end(); ++it) {
        DAccelScheme in = move(*it);
        DEBUG_PRINTF("in: %zu pairs, %zu singles, offset %u\n",
                     in.double_byte.size(), in.double_cr.count(),
                     in.double_offset);
        if (*best < in) {
            DEBUG_PRINTF("worse\n");
            continue;
        }
        findDoubleBest(pb + 1, pe, in, best);
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

#define MAX_DOUBLE_ACCEL_PATHS 10

static
DAccelScheme findBestDoubleAccelScheme(vector<vector<CharReach> > paths,
                                       const CharReach &terminating) {
    DEBUG_PRINTF("looking for double accel, %zu terminating symbols\n",
                 terminating.count());
    unifyPathsLastSegment(paths);

#ifdef DEBUG
    DEBUG_PRINTF("paths:\n");
    dumpPaths(paths);
#endif

    /* if there are too many paths, shorten the paths to reduce the number of
     * distinct paths we have to consider */
    while (paths.size() > MAX_DOUBLE_ACCEL_PATHS) {
        for (auto &p : paths) {
            if (p.empty()) {
                return DAccelScheme(terminating, 0U);
            }
            p.pop_back();
        }
        unifyPathsLastSegment(paths);
    }

    if (paths.empty()) {
        return DAccelScheme(terminating, 0U);
    }

    DAccelScheme curr(terminating, 0U);
    DAccelScheme best(CharReach::dot(), 0U);
    findDoubleBest(paths.begin(), paths.end(), curr, &best);
    DEBUG_PRINTF("da %zu pairs, %zu singles\n", best.double_byte.size(),
                 best.double_cr.count());
    return best;
}

#define MAX_EXPLORE_PATHS 40

AccelScheme findBestAccelScheme(vector<vector<CharReach> > paths,
                                const CharReach &terminating,
                                bool look_for_double_byte) {
    AccelScheme rv;
    if (look_for_double_byte) {
        DAccelScheme da = findBestDoubleAccelScheme(paths, terminating);
        if (da.double_byte.size() <= DOUBLE_SHUFTI_LIMIT) {
            rv.double_byte = move(da.double_byte);
            rv.double_cr = move(da.double_cr);
            rv.double_offset = da.double_offset;
        }
    }

    improvePaths(paths);

    DEBUG_PRINTF("we have %zu paths\n", paths.size());
    if (paths.size() > MAX_EXPLORE_PATHS) {
        return rv; /* too many paths to explore */
    }

    /* if we were smart we would do something netflowy on the paths to find the
     * best cut. But we aren't, so we will just brute force it.
     */
    SAccelScheme curr(terminating, 0U);
    SAccelScheme best;
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

    rv.offset = best.offset;
    rv.cr = best.cr;
    if (rv.cr.count() < rv.double_cr.count()) {
        rv.double_byte.clear();
    }

    return rv;
}

AccelScheme nfaFindAccel(const NGHolder &g, const vector<NFAVertex> &verts,
                         const vector<CharReach> &refined_cr,
                         const map<NFAVertex, BoundedRepeatSummary> &br_cyclic,
                         bool allow_wide, bool look_for_double_byte) {
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

    return findBestAccelScheme(std::move(paths), terminating,
                               look_for_double_byte);
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
            *as = AccelScheme();
            as->offset = i;
            as->cr = CharReach();
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
                *as = AccelScheme();
                as->offset = i;
                return true;
            }
        }
    }

    // Second option: a two-byte shufti (i.e. less than eight 2-byte
    // literals)
    if (depth > 1) {
        for (unsigned int i = 0; i < (depth - 1); i++) {
            if (depthReach[i].count() * depthReach[i+1].count()
                <= DOUBLE_SHUFTI_LIMIT) {
                DEBUG_PRINTF("two-byte shufti, depth %u\n", i);
                *as = AccelScheme();
                as->offset = i;
                return true;
            }
        }
    }

    // Look for offset accel schemes verm/shufti;
    vector<NFAVertex> verts(1, v);
    *as = nfaFindAccel(g, verts, refined_cr, br_cyclic, allow_wide, true);
    DEBUG_PRINTF("as width %zu\n", as->cr.count());
    return as->cr.count() <= ACCEL_MAX_STOP_CHAR || allow_wide;
}

} // namespace ue2
