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

/** \file
 * \brief NFA acceleration analysis code.
 */
#include "ng_limex_accel.h"

#include "ng_holder.h"
#include "ng_misc_opt.h"
#include "ng_util.h"
#include "ue2common.h"

#include "nfa/accel.h"

#include "util/bitutils.h" // for CASE_CLEAR
#include "util/charreach.h"
#include "util/compile_context.h"
#include "util/container.h"
#include "util/dump_charclass.h"
#include "util/graph_range.h"
#include "util/small_vector.h"
#include "util/target_info.h"

#include <algorithm>
#include <map>

#include <boost/range/adaptor/map.hpp>

using namespace std;
using boost::adaptors::map_keys;

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
        DEBUG_PRINTF("checking %zu\n", g[v].index);

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

        DEBUG_PRINTF("%zu is a friend indeed\n", g[v].index);
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
               vector<vector<CharReach>> *paths,
               const flat_set<NFAVertex> &forbidden, u32 depth) {
    static const u32 MAGIC_TOO_WIDE_NUMBER = 16;
    if (!depth) {
        paths->push_back({});
        return;
    }
    if (v == g.accept || v == g.acceptEod) {
        paths->push_back({});
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
        paths->push_back({cr});
        return;
    }

    vector<vector<CharReach>> curr;
    for (auto w : adjacent_vertices_range(v, g)) {
        if (contains(forbidden, w)) {
            /* path has looped back to one of the active+boring acceleration
             * states.  We can ignore this path if we have sufficient back-
             * off. */
            paths->push_back({cr});
            continue;
        }

        u32 new_depth = depth - 1;
        do {
            curr.clear();
            findPaths(g, w, refined_cr, &curr, forbidden, new_depth);
        } while (new_depth-- && curr.size() >= MAGIC_TOO_WIDE_NUMBER);

        for (auto &c : curr) {
            c.push_back(cr);
            paths->push_back(std::move(c));
        }
    }
}

namespace {
struct SAccelScheme {
    SAccelScheme(CharReach cr_in, u32 offset_in)
        : cr(std::move(cr_in)), offset(offset_in) {
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
}

/**
 * \brief Limit on the number of (recursive) calls to findBestInternal().
 */
static constexpr size_t MAX_FINDBEST_CALLS = 1000000;

static
void findBestInternal(vector<vector<CharReach>>::const_iterator pb,
                      vector<vector<CharReach>>::const_iterator pe,
                      size_t *num_calls, const SAccelScheme &curr,
                      SAccelScheme *best) {
    assert(curr.offset <= MAX_ACCEL_DEPTH);

    if (++(*num_calls) > MAX_FINDBEST_CALLS) {
        DEBUG_PRINTF("hit num_calls limit %zu\n", *num_calls);
        return;
    }

    DEBUG_PRINTF("paths left %zu\n", pe - pb);
    if (pb == pe) {
        if (curr < *best) {
            *best = curr;
            DEBUG_PRINTF("new best: count=%zu, class=%s, offset=%u\n",
                         best->cr.count(), describeClass(best->cr).c_str(),
                         best->offset);
        }
        return;
    }

    DEBUG_PRINTF("p len %zu\n", pb->end() - pb->begin());

    small_vector<SAccelScheme, 10> priority_path;
    priority_path.reserve(pb->size());
    u32 i = 0;
    for (auto p = pb->begin(); p != pb->end(); ++p, i++) {
        SAccelScheme as(*p | curr.cr, max(i, curr.offset));
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

    for (const SAccelScheme &in : priority_path) {
        DEBUG_PRINTF("in: count %zu\n", in.cr.count());
        if (*best < in) {
            DEBUG_PRINTF("worse\n");
            continue;
        }
        findBestInternal(pb + 1, pe, num_calls, in, best);

        if (curr.cr == best->cr) {
            return; /* could only get better by offset */
        }
    }
}

static
SAccelScheme findBest(const vector<vector<CharReach>> &paths,
                      const CharReach &terminating) {
    SAccelScheme curr(terminating, 0U);
    SAccelScheme best;
    size_t num_calls = 0;
    findBestInternal(paths.begin(), paths.end(), &num_calls, curr, &best);
    DEBUG_PRINTF("findBest completed, num_calls=%zu\n", num_calls);
    DEBUG_PRINTF("selected scheme: count=%zu, class=%s, offset=%u\n",
                 best.cr.count(), describeClass(best.cr).c_str(), best.offset);
    return best;
}

namespace {
struct DAccelScheme {
    DAccelScheme(CharReach cr_in, u32 offset_in)
        : double_cr(std::move(cr_in)), double_offset(offset_in) {
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

    flat_set<pair<u8, u8>> double_byte;
    CharReach double_cr;
    u32 double_offset = 0;
};
}

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

    small_vector<DAccelScheme, 10> priority_path;
    priority_path.reserve(pb->size());
    u32 i = 0;
    for (auto p = pb->begin(); p != pb->end() && next(p) != pb->end();
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

    for (const DAccelScheme &in : priority_path) {
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
void dumpPaths(const vector<vector<CharReach>> &paths) {
    for (const auto &path : paths) {
        DEBUG_PRINTF("path: [");
        for (const auto &cr : path) {
            printf(" [");
            describeClass(stdout, cr, 20, CC_OUT_TEXT);
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

AccelScheme findBestAccelScheme(vector<vector<CharReach>> paths,
                                const CharReach &terminating,
                                bool look_for_double_byte) {
    AccelScheme rv;
    if (look_for_double_byte) {
        DAccelScheme da = findBestDoubleAccelScheme(paths, terminating);
        if (da.double_byte.size() <= DOUBLE_SHUFTI_LIMIT) {
            rv.double_byte = std::move(da.double_byte);
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
    SAccelScheme best = findBest(paths, terminating);

    /* find best is a bit lazy in terms of minimising the offset, see if we can
     * make it better. need to find the min max offset that we need.*/
    u32 offset = 0;
    for (const auto &path : paths) {
        u32 i = 0;
        for (const auto &cr : path) {
            if (cr.isSubsetOf(best.cr)) {
                break;
            }
            i++;
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

    vector<vector<CharReach>> paths;
    flat_set<NFAVertex> ignore_vert_set(verts.begin(), verts.end());

    /* Note: we can not in general (TODO: ignore when possible) ignore entries
     * into the bounded repeat cyclic states as that is when the magic happens
     */
    for (auto v : br_cyclic | map_keys) {
        /* TODO: can allow if repeatMin <= 1 ? */
        ignore_vert_set.erase(v);
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
    for (auto &path : paths) {
        reverse(path.begin(), path.end());
    }

    return findBestAccelScheme(std::move(paths), terminating,
                               look_for_double_byte);
}

NFAVertex get_sds_or_proxy(const NGHolder &g) {
    DEBUG_PRINTF("looking for sds proxy\n");
    if (proper_out_degree(g.startDs, g)) {
        return g.startDs;
    }

    NFAVertex v = NGHolder::null_vertex();
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
            DEBUG_PRINTF("woot %zu\n", g[v].index);
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

    DEBUG_PRINTF("vertex %zu is cyclic and has %zu stop chars%s\n",
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
