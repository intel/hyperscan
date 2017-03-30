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
 * \brief Analysis pass to reform leading dots.
 *
 * We have found that many regexes found in the wild use an anchored dot-repeat
 * to represent an unanchored pattern, particularly if they have been used with
 * a regex engine that assumes that a pattern is anchored. This pass reforms
 * patterns that begin with sequences of dots into a more standard form.
 *
 * In addition, both anchored and unanchored patterns with dot repeats as
 * prefixes will have these prefixes reformed into a canonical form, which some
 * later analyses depend upon.
 */
#include "ng_anchored_dots.h"

#include "grey.h"
#include "ng_holder.h"
#include "ng_util.h"
#include "ue2common.h"
#include "util/container.h"
#include "util/depth.h"
#include "util/graph_range.h"

#include <algorithm>
#include <queue>
#include <set>
#include <vector>

using namespace std;

namespace ue2 {

static
bool findStarts(const NGHolder &g, set<NFAVertex> &anchored,
                set<NFAVertex> &unanchored) {
    // Populate unanchored map
    for (auto v : adjacent_vertices_range(g.startDs, g)) {
        if (is_special(v, g)) {
            continue;
        }
        unanchored.insert(v);
    }

    // Populate anchored map
    for (auto v : adjacent_vertices_range(g.start, g)) {
        if (is_special(v, g)) {
            continue;
        }
        anchored.insert(v);
    }

    if (unanchored == anchored) {
        anchored.clear();
    } else if (!unanchored.empty() && !anchored.empty()) {
        return false;
    }

    return !anchored.empty() || !unanchored.empty();
}

namespace {
class DotInfo {
public:
    DotInfo(NFAVertex v, bool se, u32 idx)
        : vertex(v), hasSelfLoop(se), index(idx) {}

    bool operator<(const DotInfo &other) const {
        if (hasSelfLoop != other.hasSelfLoop)
            return hasSelfLoop < other.hasSelfLoop;
        // tie break with vertex id: lowest ID wins
        return index > other.index;
    }

    NFAVertex vertex;
    bool hasSelfLoop;
    u32 index;
};
}

// Returns nullptr if all vertices in the given set are not dots.
// We can only pick one dot vertex, so we go for a dot-star if it exists,
// otherwise the dot without a self-edge with the lowest ID.
static
NFAVertex findReformable(const NGHolder &g, const set<NFAVertex> &starts,
                         set<NFAVertex> &otherV) {
    priority_queue<DotInfo> dotq;
    for (auto v : starts) {
        if (is_dot(v, g)) {
            u32 idx = g[v].index;
            dotq.push(DotInfo(v, hasSelfLoop(v, g), idx));
        }
    }

    if (dotq.empty()) {
        return NGHolder::null_vertex();
    }

    const DotInfo &dot = dotq.top();
    otherV = starts;
    otherV.erase(dot.vertex);
    DEBUG_PRINTF("selected dot vertex %u (%s)\n", dot.index,
                 dot.hasSelfLoop ? "has self-edge" : "no self-edge");
    DEBUG_PRINTF("%zu other vertices\n", otherV.size());
    return dot.vertex;
}

// Returns true if the given vertex is only preceded by start. If start is
// graph.startDs (i.e. unanchored), the given vertex can also be connected to
// graph.start. If selfLoopIsAcceptable is set, self-loops are ignored.
static
bool isStartNode(NFAVertex v, NFAVertex start, const NGHolder &g,
                 bool selfLoopIsAcceptable) {
    for (auto u : inv_adjacent_vertices_range(v, g)) {
        if (selfLoopIsAcceptable && u == v) {
            continue;
        } else if (u == start) {
            continue;
        } else if (start == g.startDs && u == g.start) {
            continue;
        } else {
            return false;
        }
    }
    return true;
}

// Note: this will only remove the anchored first dot in the chain -- any other
// removable nodes will be handled by the unanchored case below.
static
void reformAnchoredRepeatsComponent(NGHolder &g,
                                    set<NFAVertex> &compAnchoredStarts,
                                    set<NFAVertex> &compUnanchoredStarts,
                                    set<NFAVertex> &dead, depth *startBegin,
                                    depth *startEnd) {
    // anchored cases can not have any unanchored starts
    if (!compUnanchoredStarts.empty()) {
        DEBUG_PRINTF("we have unanchored starts, skipping\n");
        return;
    }

    NFAVertex dotV = NGHolder::null_vertex();
    set<NFAVertex> otherV;
    dotV = findReformable(g, compAnchoredStarts, otherV);
    if (dotV == NGHolder::null_vertex()) {
        DEBUG_PRINTF("no candidate reformable dot found.\n");
        return;
    }

    NFAEdge loopEdge;
    bool selfLoop = false;
    bool bustOut = false;

    for (const auto &e : out_edges_range(dotV, g)) {
        NFAVertex t = target(e, g);
        if (t == dotV) {
            selfLoop = true;
            loopEdge = e;
            continue;
        }

        if (is_special(t, g)) {
            bustOut = true;
            break;
        }

        if (!otherV.empty() && otherV.find(t) == otherV.end()) {
            bustOut = true;
            break;
        }
    }

    if (bustOut) {
        DEBUG_PRINTF("busting out\n");
        return;
    }

    if (!isStartNode(dotV, g.start, g, true)) {
        DEBUG_PRINTF("fleeing: vertex %zu has other preds\n", g[dotV].index);
        return;
    }

    /* get bounds */
    depth min;
    depth max(1);

    if (selfLoop) {
        // A self-loop indicates that this is a '.+' or '.*'
        max = depth::infinity();
    }

    if (!otherV.empty()) {
        /* We require that the successors of the dot node are are the same
         * as the start vertex. TODO: remember why.
         */
        if (selfLoop) {
            if (otherV.size() != out_degree(dotV, g) - 1) {
                return;
            }
        } else {
            if (otherV.size() != out_degree(dotV, g)) {
                return;
            }
        }

        min = depth(0);
    } else {
        min = depth(1);
    }

    *startBegin = min;
    *startEnd = max;

    for (auto t : adjacent_vertices_range(dotV, g)) {
        if (t != dotV) {
            add_edge_if_not_present(g.startDs, t, g);
            add_edge_if_not_present(g.start, t, g);
            compUnanchoredStarts.insert(t);
        }
    }

    for (auto v : otherV) {
        remove_edge(g.start, v, g);
    }

    DEBUG_PRINTF("removing vertex %zu\n", g[dotV].index);
    clear_vertex(dotV, g);
    dead.insert(dotV);
    compAnchoredStarts.erase(dotV);
}

static
void reformUnanchoredRepeatsComponent(NGHolder &g,
                                      set<NFAVertex> &compAnchoredStarts,
                                      set<NFAVertex> &compUnanchoredStarts,
                                      set<NFAVertex> &dead,
                                      depth *startBegin, depth *startEnd) {
    // unanchored cases can not have any anchored starts
    if (!compAnchoredStarts.empty()) {
        DEBUG_PRINTF("we have anchored starts, skipping\n");
        return;
    }

    while (true) {
        NFAVertex dotV = NGHolder::null_vertex();
        set<NFAVertex> otherV;
        dotV = findReformable(g, compUnanchoredStarts, otherV);
        if (dotV == NGHolder::null_vertex()) {
            DEBUG_PRINTF("no candidate reformable dot found.\n");
            return;
        }

        NFAEdge loopEdge;
        bool selfLoop = false;
        bool bustOut = false;

        for (const auto &e : out_edges_range(dotV, g)) {
            NFAVertex t = target(e, g);

            if (t == dotV) {
                selfLoop = true;
                loopEdge = e;
                continue;
            }

            if (is_special(t, g)) {
                bustOut = true;
                break;
            }

            if (!otherV.empty() && otherV.find(t) == otherV.end()) {
                bustOut = true;
                break;
            }
        }

        if (bustOut) {
            DEBUG_PRINTF("busting out\n");
            if (!selfLoop) {
                return;
            }

            for (auto v : otherV) {
                if (!edge(dotV, v, g).second) {
                    return;
                }
            }

            // A self-loop indicates that this is a '.+' or '.*'
            DEBUG_PRINTF("self-loop detected on %zu\n", g[dotV].index);
            *startEnd = depth::infinity();
            remove_edge(dotV, dotV, g);
            return;
        }

        if (!isStartNode(dotV, g.startDs, g, true)) {
            DEBUG_PRINTF("fleeing: vertex %zu has other preds\n",
                         g[dotV].index);
            return;
        }

        /* get bounds */
        depth min(1);
        depth max(1);

        if (selfLoop) {
            // A self-loop indicates that this is a '.+' or '.*'
            DEBUG_PRINTF("self-loop detected\n");
            max = depth::infinity();
        }

        if (!otherV.empty()) {
            if (!selfLoop && otherV.size() != out_degree(dotV, g)) {
                return;
            }

            if (selfLoop && otherV.size() != out_degree(dotV, g) - 1) {
                return;
            }

            if (min > depth(1)) {
                /* this is not a case we can handle */
                DEBUG_PRINTF("min greater than one, skipping\n");
                return;
            }
            min = depth(0);
        }

        *startBegin += min;
        *startEnd += max;

        for (auto v : otherV) {
            remove_edge(g.start, v, g);
            remove_edge(g.startDs, v, g);
        }

        compUnanchoredStarts.clear();
        for (auto t : adjacent_vertices_range(dotV, g)) {
            if (t != dotV) {
                DEBUG_PRINTF("connecting sds -> %zu\n", g[t].index);
                add_edge(g.startDs, t, g);
                add_edge(g.start, t, g);
                compUnanchoredStarts.insert(t);
            }
        }

        DEBUG_PRINTF("removing vertex %zu\n", g[dotV].index);
        dead.insert(dotV);
        clear_vertex(dotV, g);
        compUnanchoredStarts.erase(dotV);
    }
}

// for t to be another optional dot, it must have only in-edges from v and from
// starts
static
bool isOptionalDot(NFAVertex t, NFAVertex v, const NGHolder &g) {
    if (!is_dot(t, g)) {
        return false;
    }

    bool found_v = false, found_start = false;

    for (auto u : inv_adjacent_vertices_range(t, g)) {
        if (u == v) {
            found_v = true;
        } else if (u == g.start || u == g.startDs) {
            found_start = true;
        } else {
            return false;
        }
    }

    return found_v && found_start;
}

static
bool gatherParticipants(const NGHolder &g,
                        NFAVertex start, NFAVertex initialDot,
                        set<NFAVertex> &dots, set<NFAVertex> &succ) {
    // Walk the graph downwards from the initial dot; each dot will have:
    //      1) a single optional dot successor, or
    //      2) N successors (our terminating case)
    dots.insert(initialDot);
    NFAVertex v = initialDot;

    while (out_degree(v, g) == 1) {
        NFAVertex t = *(adjacent_vertices(v, g).first);
        // for t to be another optional dot, it must have only in-edges from v
        // and from starts
        if (isOptionalDot(t, v, g)) {
            // another dot; bail if we've seen it once already
            if (dots.find(t) != dots.end()) {
                DEBUG_PRINTF("cycle detected at vertex %zu\n", g[t].index);
                return false;
            }
            dots.insert(t);
            v = t;
            continue;
        }
        // otherwise, we found a terminating dot state
        break;
    }

    // Our terminating states are the successors of v.
    // All of these MUST have an edge from start as well.
    for (auto w : adjacent_vertices_range(v, g)) {
        succ.insert(w);
        if (!edge(start, w, g).second) {
            DEBUG_PRINTF("failing, vertex %zu does not have edge from start\n",
                         g[w].index);
            return false;
        }
    }

    /* All the non chained v connected to start must be in succ as well
     * TODO: remember why (and document). */
    for (auto u : adjacent_vertices_range(start, g)) {
        if (is_special(u, g)) {
            continue;
        }
        if (!contains(dots, u) && !contains(succ, u)) {
            return false;
        }
    }

    return !succ.empty();
}

static
void collapseVariableDotRepeat(NGHolder &g, NFAVertex start,
                               set<NFAVertex> &dead, UNUSED depth *startBegin,
                               depth *startEnd) {
    // Handle optional dot repeat prefixes, e.g.
    //     /^.{0,30}foo/s, /^.{0,5}foo/s, unanchored equivs
    // Note that this code assumes that fixed repeats ('^.{5,20}') have been
    // pruned already, down (in this case) to '^.{0,15}'.

    // The first of our optional dots must be connected to start. The jump edge
    // past it will be verified in gatherParticipants(). If start is
    // graph.start, it should not be connected to startDs.
    NFAVertex initialDot = NGHolder::null_vertex();
    for (auto v : adjacent_vertices_range(start, g)) {
        if (is_special(v, g)) {
            continue;
        }
        if (is_dot(v, g) && isStartNode(v, start, g, false)) {
            if (initialDot) {
                return;
            }
            initialDot = v;
            DEBUG_PRINTF("initial dot vertex is %zu\n", g[v].index);
        }
    }

    if (!initialDot) {
        return;
    }

    // Collect all the other optional dot vertices and the successor vertices
    // by walking down the graph from initialDot
    set<NFAVertex> dots, succ;
    if (!gatherParticipants(g, start, initialDot, dots, succ)) {
        DEBUG_PRINTF("gatherParticipants failed\n");
        return;
    }

    DEBUG_PRINTF("optional dot repeat with %zu participants, "
                 "terminating in %zu non-dot nodes\n",
                 dots.size(), succ.size());

    // Remove all the participants and set the start offset
    dead.insert(dots.begin(), dots.end());

    DEBUG_PRINTF("current offsets: %s-%s\n", startBegin->str().c_str(),
                 startEnd->str().c_str());

    if (start == g.start && startEnd->is_infinite()) {
        *startEnd = depth(dots.size());
    } else if (startEnd->is_finite()) {
        *startEnd += dots.size();
    }
    assert(startEnd->is_reachable());

    // Connect our successor vertices to both start and startDs.
    for (auto v : succ) {
        add_edge_if_not_present(g.start, v, g);
        add_edge_if_not_present(g.startDs, v, g);
    }
}

static
void deleteVertices(set<NFAVertex> &dead, NGHolder &g) {
    if (!dead.empty()) {
        DEBUG_PRINTF("pruning %zu vertices\n", dead.size());
        remove_vertices(dead, g);
    }
    dead.clear();
}

static
void reformAnchoredRepeats(NGHolder &g, depth *startBegin, depth *startEnd) {
    DEBUG_PRINTF("component\n");
    set<NFAVertex> anchored, unanchored, dead;
    if (!findStarts(g, anchored, unanchored)) {
        DEBUG_PRINTF("no starts\n");
        return;
    }

    reformAnchoredRepeatsComponent(g, anchored, unanchored, dead, startBegin,
                                   startEnd);
    deleteVertices(dead, g);

    reformUnanchoredRepeatsComponent(g, anchored, unanchored, dead, startBegin,
                                     startEnd);
    deleteVertices(dead, g);
}

static
void collapseVariableRepeats(NGHolder &g, depth *startBegin, depth *startEnd) {
    DEBUG_PRINTF("collapseVariableRepeats\n");
    set<NFAVertex> dead;

    collapseVariableDotRepeat(g, g.start, dead, startBegin, startEnd);
    deleteVertices(dead, g);

    collapseVariableDotRepeat(g, g.startDs, dead, startBegin, startEnd);
    deleteVertices(dead, g);
}

static
void addDotsBetween(NGHolder &g, NFAVertex lhs, vector<NFAVertex> &rhs,
                    depth min_repeat, depth max_repeat) {
    const bool unbounded = max_repeat.is_infinite();
    if (unbounded) {
        max_repeat = min_repeat;
    }

    assert(max_repeat.is_finite());

    NFAVertex u = lhs;

    if (!min_repeat && unbounded) {
        NFAVertex v = add_vertex(g);
        add_edge(u, v, g);
        g[v].char_reach.setall();

        for (auto w : rhs) {
            add_edge(lhs, w, g);
        }
    }

    for (u32 i = 0; i < min_repeat; i++) {
        NFAVertex v = add_vertex(g);
        add_edge(u, v, g);
        g[v].char_reach.setall();
        u = v;
    }

    NFAVertex split = u;
    /* lhs now split point for optional */
    for (u32 i = min_repeat; i < max_repeat; i++) {
        NFAVertex v = add_vertex(g);
        add_edge(u, v, g);
        if (u != split) {
            add_edge(split, v, g);
        }
        g[v].char_reach.setall();
        u = v;
    }

    if (unbounded) {
        add_edge(u, u, g);
    }

    for (auto w : rhs) {
        add_edge(u, w, g);
        if (split != u) {
            add_edge(split, w, g);
        }
    }
}

static
void restoreLeadingDots(NGHolder &g, const depth &startBegin,
                        const depth &startEnd) {
    if (startBegin == depth(0) && startEnd.is_infinite()) {
        return;
    }
    DEBUG_PRINTF("ungobble (%s, %s)\n", startBegin.str().c_str(),
                 startEnd.str().c_str());

    for (UNUSED auto v : adjacent_vertices_range(g.start, g)) {
        assert(edge(g.startDs, v, g).second);
    }
    clear_out_edges(g.start, g);
    add_edge(g.start, g.startDs, g);

    const bool unbounded = startEnd.is_infinite();

    NFAVertex root = unbounded ? g.startDs : g.start;

    vector<NFAVertex> rhs;
    insert(&rhs, rhs.end(), adjacent_vertices(g.startDs, g));
    rhs.erase(remove(rhs.begin(), rhs.end(), g.startDs), rhs.end());
    for (auto v : rhs) {
        remove_edge(g.startDs, v, g);
    }

    addDotsBetween(g, root, rhs, startBegin, startEnd);
    renumber_vertices(g);
    renumber_edges(g);
}

// Entry point.
void reformLeadingDots(NGHolder &g) {
    depth startBegin(0);
    depth startEnd = depth::infinity();

    reformAnchoredRepeats(g, &startBegin, &startEnd);
    collapseVariableRepeats(g, &startBegin, &startEnd);
    restoreLeadingDots(g, startBegin, startEnd);
}

} // namespace ue2
