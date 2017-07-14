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

#include "rose_build_castle.h"

#include "rose_build_impl.h"
#include "ue2common.h"
#include "nfa/castlecompile.h"
#include "nfagraph/ng_holder.h"
#include "nfagraph/ng_puff.h"
#include "util/charreach.h"
#include "util/compile_context.h"
#include "util/container.h"
#include "util/dump_charclass.h"
#include "util/graph_range.h"
#include "util/ue2string.h"

#include <map>
#include <set>
#include <string>
#include <vector>

#include <boost/range/adaptor/map.hpp>

using namespace std;
using boost::adaptors::map_values;

namespace ue2 {

static
void makeCastle(LeftEngInfo &left,
            unordered_map<const NGHolder *, shared_ptr<CastleProto>> &cache) {
    if (left.dfa || left.haig || left.castle) {
        return;
    }
    if (!left.graph) {
        return;
    }

    const NGHolder &h = *left.graph;
    DEBUG_PRINTF("prefix %p\n", &h);

    if (contains(cache, &h)) {
        DEBUG_PRINTF("using cached CastleProto\n");
        left.castle = cache[&h];
        left.graph.reset();
        return;
    }

    PureRepeat pr;
    if (isPureRepeat(h, pr) && pr.reports.size() == 1) {
        DEBUG_PRINTF("vertex preceded by infix repeat %s\n",
                     pr.bounds.str().c_str());
        left.castle = make_shared<CastleProto>(h.kind, pr);
        cache[&h] = left.castle;
        left.graph.reset();
    }
}

static
void makeCastleSuffix(RoseBuildImpl &tbi, RoseVertex v,
            unordered_map<const NGHolder *, shared_ptr<CastleProto>> &cache) {
    RoseSuffixInfo &suffix = tbi.g[v].suffix;
    if (!suffix.graph) {
        return;
    }
    const NGHolder &h = *suffix.graph;
    DEBUG_PRINTF("suffix %p\n", &h);

    if (contains(cache, &h)) {
        DEBUG_PRINTF("using cached CastleProto\n");
        suffix.castle = cache[&h];
        suffix.graph.reset();
        return;
    }

    // The MPV will probably do a better job on the cases it's designed
    // for.
    const bool fixed_depth = tbi.g[v].min_offset == tbi.g[v].max_offset;
    if (isPuffable(h, fixed_depth, tbi.rm, tbi.cc.grey)) {
        DEBUG_PRINTF("leaving suffix for puff\n");
        return;
    }

    PureRepeat pr;
    if (isPureRepeat(h, pr) && pr.reports.size() == 1) {
        DEBUG_PRINTF("suffix repeat %s\n", pr.bounds.str().c_str());

        // Right now, the Castle uses much more stream state to represent a
        // {m,1} repeat than just leaving it to an NFA.
        if (pr.bounds.max <= depth(1)) {
            DEBUG_PRINTF("leaving for other engines\n");
            return;
        }

        suffix.castle = make_shared<CastleProto>(h.kind, pr);
        cache[&h] = suffix.castle;
        suffix.graph.reset();
    }
}

static
vector<rose_literal_id> literals_for_vertex(const RoseBuildImpl &tbi,
                                            RoseVertex v) {
    vector<rose_literal_id> rv;

    for (const u32 id : tbi.g[v].literals) {
        rv.push_back(tbi.literals.at(id));
    }

    return rv;
}

static
void renovateCastle(RoseBuildImpl &tbi, CastleProto *castle,
              const vector<RoseVertex> &verts) {
    DEBUG_PRINTF("looking to renovate\n");

    if (castle->repeats.size() != 1) {
        assert(0);  /* should not have merged castles yet */
        return;
    }

    PureRepeat &pr = castle->repeats.begin()->second;
    if (pr.bounds.max.is_finite()) {
        /* repeat cannot be turned into pseudo .* */
        return;
    }

    RoseGraph &g = tbi.g;
    const CharReach &cr = castle->reach();

    DEBUG_PRINTF("cr || %zu\n", cr.count());

    u32 allowed_to_remove = ~0;
    size_t min_succ_lit_len = 0;

    for (RoseVertex v : verts) {
        assert(g[v].left.castle.get() == castle);
        DEBUG_PRINTF("%zu checks at lag %u\n", g[v].index, g[v].left.lag);
        vector<rose_literal_id> lits = literals_for_vertex(tbi, v);
        for (const auto &e : lits) {
            DEBUG_PRINTF("%s +%u\n", dumpString(e.s).c_str(), e.delay);
            if (e.delay) {
                return; /* bail - TODO: be less lazy */
            }

            vector<CharReach> rem_local_cr;
            u32 ok_count = 0;
            for (auto it = e.s.end() - g[v].left.lag; it != e.s.end(); ++it) {
                if (!isSubsetOf(*it, cr)) {
                    break;
                }

                ok_count++;
            }
            LIMIT_TO_AT_MOST(&allowed_to_remove, ok_count);
            ENSURE_AT_LEAST(&min_succ_lit_len, e.elength());
        }
    }

    DEBUG_PRINTF("possible to decrease lag by %u\n", allowed_to_remove);


    for (RoseVertex v : verts) {
        assert(g[v].left.lag >= allowed_to_remove);
        g[v].left.lag -= allowed_to_remove;
    }

    assert(castle->repeats.size() == 1); /* should not have merged castles yet */

    pr.bounds.max += allowed_to_remove;

    /* Although it is always safe to increase the min bound as well, we would
     * rather not as a >0 min bound means that we have to store state as well.
     *
     * As it was legal to run with the original lag, we know that it is not
     * possible to have an overlapping match which finishes within the trigger
     * literal past the original lag point. However, if there is already a min
     * bound constraint this would be broken if we did not also increase the
     * min bound. */

    if (pr.bounds.min > 0ULL || allowed_to_remove > min_succ_lit_len) {
        pr.bounds.min += allowed_to_remove;
    }
}

void makeCastles(RoseBuildImpl &tbi) {
    if (!tbi.cc.grey.allowCastle && !tbi.cc.grey.allowLbr) {
        return;
    }

    RoseGraph &g = tbi.g;

    // Caches so that we can reuse analysis on graphs we've seen already.
    unordered_map<const NGHolder *, shared_ptr<CastleProto> > left_cache;
    unordered_map<const NGHolder *, shared_ptr<CastleProto> > suffix_cache;

    unordered_map<CastleProto *, vector<RoseVertex>> rev;

    for (RoseVertex v : vertices_range(g)) {
        if (g[v].left && !tbi.isRootSuccessor(v)) {
            makeCastle(g[v].left, left_cache);
            if (g[v].left.castle) {
                rev[g[v].left.castle.get()].push_back(v);
            }
        }

        if (g[v].suffix) {
            makeCastleSuffix(tbi, v, suffix_cache);
        }
    }

    for (const auto &e : rev) {
        renovateCastle(tbi, e.first, e.second);
    }
}

bool unmakeCastles(RoseBuildImpl &tbi) {
    RoseGraph &g = tbi.g;

    const size_t MAX_UNMAKE_VERTICES = 64;

    map<left_id, vector<RoseVertex> > left_castles;
    map<suffix_id, vector<RoseVertex> > suffix_castles;
    bool changed = false;

    for (auto v : vertices_range(g)) {
        const LeftEngInfo &left = g[v].left;
        if (left.castle && left.castle->repeats.size() > 1) {
            left_castles[left].push_back(v);
        }
        const RoseSuffixInfo &suffix = g[v].suffix;
        if (suffix.castle && suffix.castle->repeats.size() > 1) {
            suffix_castles[suffix].push_back(v);
        }
    }

    for (const auto &e : left_castles) {
        assert(e.first.castle());
        shared_ptr<NGHolder> h = makeHolder(*e.first.castle(), tbi.cc);
        if (!h || num_vertices(*h) > MAX_UNMAKE_VERTICES) {
            continue;
        }
        DEBUG_PRINTF("replace rose with holder (%zu vertices)\n",
                     num_vertices(*h));
        for (auto v : e.second) {
            assert(g[v].left.castle.get() == e.first.castle());
            g[v].left.graph = h;
            g[v].left.castle.reset();
            changed = true;
        }
    }

    for (const auto &e : suffix_castles) {
        assert(e.first.castle());
        shared_ptr<NGHolder> h = makeHolder(*e.first.castle(), tbi.cc);
        if (!h || num_vertices(*h) > MAX_UNMAKE_VERTICES) {
            continue;
        }
        DEBUG_PRINTF("replace suffix with holder (%zu vertices)\n",
                     num_vertices(*h));
        for (auto v : e.second) {
            assert(g[v].suffix.castle.get() == e.first.castle());
            g[v].suffix.graph = h;
            g[v].suffix.castle.reset();
            changed = true;
        }
    }

    return changed;
}

void remapCastleTops(RoseBuildImpl &tbi) {
    unordered_map<CastleProto *, vector<RoseVertex>> rose_castles;
    unordered_map<CastleProto *, vector<RoseVertex>> suffix_castles;

    RoseGraph &g = tbi.g;
    for (auto v : vertices_range(g)) {
        if (g[v].left.castle) {
            rose_castles[g[v].left.castle.get()].push_back(v);
        }
        if (g[v].suffix.castle) {
            suffix_castles[g[v].suffix.castle.get()].push_back(v);
        }
    }

    DEBUG_PRINTF("%zu rose castles, %zu suffix castles\n", rose_castles.size(),
                  suffix_castles.size());

    map<u32, u32> top_map;

    // Remap Rose Castles.
    for (const auto &rc : rose_castles) {
        CastleProto *c = rc.first;
        const vector<RoseVertex> &verts = rc.second;

        DEBUG_PRINTF("rose castle %p (%zu repeats) has %zu verts\n", c,
                      c->repeats.size(), verts.size());

        top_map.clear();
        remapCastleTops(*c, top_map);

        // Update the tops on the edges leading into vertices in v.
        for (auto v : verts) {
            for (const auto &e : in_edges_range(v, g)) {
                g[e].rose_top = top_map.at(g[e].rose_top);
            }
        }
    }

    // Remap Suffix Castles.
    for (const auto &e : suffix_castles) {
        CastleProto *c = e.first;
        const vector<RoseVertex> &verts = e.second;

        DEBUG_PRINTF("suffix castle %p (%zu repeats) has %zu verts\n", c,
                      c->repeats.size(), verts.size());

        top_map.clear();
        remapCastleTops(*c, top_map);

        // Update the tops on the suffixes.
        for (auto v : verts) {
            assert(g[v].suffix);
            g[v].suffix.top = top_map.at(g[v].suffix.top);
        }
    }
}

bool triggerKillsRoseCastle(const RoseBuildImpl &tbi, const left_id &left,
                            const set<ue2_literal> &all_lits,
                            const RoseEdge &e) {
    assert(left.castle());
    const CastleProto &c = *left.castle();

    const depth max_width = findMaxWidth(c);
    DEBUG_PRINTF("castle max width is %s\n", max_width.str().c_str());

    /* check each pred literal to see if they all kill previous castle
     * state */
    for (u32 lit_id : tbi.g[source(e, tbi.g)].literals) {
        const rose_literal_id &pred_lit = tbi.literals.at(lit_id);
        const ue2_literal s = findNonOverlappingTail(all_lits, pred_lit.s);
        const CharReach &cr = c.reach();

        DEBUG_PRINTF("s=%s, castle reach=%s\n", dumpString(s).c_str(),
                      describeClass(cr).c_str());

        for (const auto &s_cr : s) {
            if (!overlaps(cr, s_cr)) {
                DEBUG_PRINTF("reach %s kills castle\n",
                             describeClass(s_cr).c_str());
                goto next_pred;
            }
        }

        if (max_width < depth(s.length())) {
            DEBUG_PRINTF("literal width >= castle max width\n");
            goto next_pred;
        }

        return false;

    next_pred:;
    }

    return true;
}

} // namespace ue2
