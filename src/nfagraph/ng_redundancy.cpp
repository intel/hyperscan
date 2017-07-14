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
 * \brief NFA graph reductions.
 *
 * This code attempts to make the NFA graph smaller by performing a number of
 * local transformations:
 *
 * ### (1) removal of redundant vertices:
 *
 *      v is redundant wrt to u if succ(v) is a subset of succ(u)
 *                            AND pred(v) is a subset of pred(u)
 *                            AND cr(v) is a subset of cr(u)
 *
 * ### (2) 'diamond' transformation:
 *
 *      given succ(v) == succ(u) and pred(v) == pred(u),
 *      v and u can be replaced by w with succ(w) = succ(v), pred(w) = pred(v),
 *                                   and cr(w) = union(cr(v), cr(u))
 *
 * ### (3) locally identifiable left equivalence:
 *
 *     given pred(v) == pred(u) (**) and cr(v) == cr(u),
 *     v and u can be replaced by w with pred(w) = pred(v), cr(w) = cr(v),
 *                                  and succ(w) = union(succ(v), succ(u))
 *
 * ### (4) locally identifiable right equivalence:
 *
 *     given succ(v) == succ(u) (**) and cr(v) == cr(u),
 *     v and u can be replaced by w with succ(w) = succ(v), cr(w) = cr(v),
 *                                  and pred(w) = union(pred(v), pred(u))
 *
 * NOTE (**): for left and right equivalence, we can also do the transform if
 * set(u) contains u, set(v) contains v and the sets are otherwise equal. This
 * enables equivalent vertices with self-loops to be merged.
 *
 * If v and u raise accepts, they can only be merged if they raise the same
 * report IDs.
 *
 * Transformations are applied repeatedly until the graph stops changing.
 *
 * Note that the final graph may depend on the order in which these
 * transformations are applied. In order to reduce the non-determinism the
 * following order is imposed: (1); (2); (3) + (4).
 */
#include "ng_redundancy.h"

#include "ng_holder.h"
#include "ng_calc_components.h"
#include "ng_dominators.h"
#include "ng_prune.h"
#include "ng_util.h"
#include "ue2common.h"
#include "util/container.h"
#include "util/flat_containers.h"
#include "util/graph_range.h"

#include <algorithm>
#include <cassert>
#include <map>
#include <set>
#include <vector>

#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/reverse_graph.hpp>

using namespace std;

namespace ue2 {

namespace {

/** Precalculated (and maintained) information about a vertex. */
class VertexInfo {
public:
    flat_set<NFAVertex> pred; //!< predecessors of this vertex
    flat_set<NFAVertex> succ; //!< successors of this vertex
    bool isAccept = false;    //!< does this vertex lead to accept?
    bool isRemoved = false;   //!< have we already removed this vertex?

    size_t inDegree() const { return pred.size(); }
    size_t outDegree() const { return succ.size(); }
};

class VertexInfoMap {
public:
    explicit VertexInfoMap(const NGHolder &gg)
        : g(gg), infos(num_vertices(gg)) {}
    VertexInfo &operator[](NFAVertex v) {
        u32 i = g[v].index;
        assert(i < infos.size());
        return infos[i];
    }

    const VertexInfo &operator[](NFAVertex v) const {
        u32 i = g[v].index;
        assert(i < infos.size());
        return infos[i];
    }

private:
    const NGHolder &g;
    vector<VertexInfo> infos;
};

} // namespace

/** Populates the info map with their predecessor and successor states, and
 * whether they are accept states. */
static
void populateContainers(const NGHolder &g, VertexInfoMap &infoMap) {
    for (auto v : vertices_range(g)) {
        VertexInfo &info = infoMap[v];
        assert(info.pred.empty() && info.succ.empty());

        // Build successor and predecessor sets
        insert(&info.pred, inv_adjacent_vertices(v, g));
        insert(&info.succ, adjacent_vertices(v, g));

        // Note whether the vertex is an accept state
        if (!is_special(v, g)) {
            if (contains(info.succ, g.accept)
                || contains(info.succ, g.acceptEod)) {
                info.isAccept = true;
            }
        }
    }
}

/** Helper function to take the intersection of two sorted vertex sets
 * in-place. */
static
void inplaceIntersection(vector<NFAVertex> &vset1,
                         const flat_set<NFAVertex> &vset2) {
    const NFAVertex GONE = NGHolder::null_vertex();

    vector<NFAVertex>::iterator it = vset1.begin(), ite = vset1.end();
    flat_set<NFAVertex>::const_iterator jt = vset2.begin(), jte = vset2.end();

    while ((it != ite) && (jt != jte)) {
        assert(*it != GONE);

        if (*it < *jt) {
            // present in vset1 but not in vset2. Set to null, remove in a
            // second pass.
            *it = GONE;
            ++it;
        } else if (*jt < *it) {
            // present in vset2 but not in vset1, skip.
            ++jt;
        } else {
            // present in both sets.
            ++it; ++jt;
        }
    }

    // Left overs are only in that set.
    vset1.erase(it, ite);

    // Remove nulls created above.
    vset1.erase(remove(vset1.begin(), vset1.end(), GONE), vset1.end());
}

/** Find the intersection of the successors of our predecessors. */
static
void succPredIntersection(const NFAVertex v, const flat_set<NFAVertex> &predSet,
                          const VertexInfoMap &infoMap,
                          vector<NFAVertex> &intersection,
                          bool considerSelf = true /* follow self loops */) {
    /* find a good seed for the intersection */
    const flat_set<NFAVertex> *best = nullptr;
    for (auto u : predSet) {
        if (!considerSelf && u == v) {
            continue;
        }

        const flat_set<NFAVertex> &succSet = infoMap[u].succ;
        if (!best || succSet.size() <= best->size()) {
            best = &succSet;

            // Break out if we've reduced our intersection to [v]
            if (best->size() == 1) {
                assert(*(best->begin()) == v);
                intersection.push_back(v);
                return;
            }
        }
    }

    if (best) {
        insert(&intersection, intersection.end(), *best);
    }

    for (auto u : predSet) {
        if (!considerSelf && u == v) {
            continue;
        }

        inplaceIntersection(intersection, infoMap[u].succ);

        // Check: intersection should always be at least size 1
        assert(!intersection.empty());

        // Break out if we've reduced our intersection to [v]
        if (intersection.size() == 1) {
            assert(*intersection.begin() == v);
            return;
        }
    }
}

/** Find the intersection of the predecessors of our successors. */
static
void predSuccIntersection(const NFAVertex v,
                          const flat_set<NFAVertex> &succSet,
                          const VertexInfoMap &infoMap,
                          vector<NFAVertex> &intersection,
                          bool considerSelf = true /* follow self loops */) {
    /* find a good seed for the intersection */
    const flat_set<NFAVertex> *best = nullptr;
    for (auto w : succSet) {
        if (!considerSelf && w == v) {
            continue;
        }

        const flat_set<NFAVertex> &predSet = infoMap[w].pred;
        if (!best || predSet.size() <= best->size()) {
            best = &predSet;

            // Break out if we've reduced our intersection to [v]
            if (best->size() == 1) {
                assert(*(best->begin()) == v);
                intersection.push_back(v);
                return;
            }
        }
    }

    if (best) {
        insert(&intersection, intersection.end(), *best);
    }

    for (auto w : succSet) {
        if (!considerSelf && w == v) {
            continue;
        }

        inplaceIntersection(intersection, infoMap[w].pred);

        // Check: intersection should always be at least size 1
        assert(!intersection.empty());

        // Break out if we've reduced our intersection to [v]
        if (intersection.size() == 1) {
            assert(*intersection.begin() == v);
            return;
        }
    }
}

/** Update containers to take into account the removal of vertex v. */
static
void markForRemoval(const NFAVertex v, VertexInfoMap &infoMap,
                    set<NFAVertex> &removable) {
    VertexInfo &info = infoMap[v];
    assert(!info.isRemoved);
    assert(!contains(removable, v));
    info.isRemoved = true;
    removable.insert(v);

    // remove v from its predecessors' successors
    for (auto u : info.pred) {
        infoMap[u].succ.erase(v);
    }

    // remove v from its successors' predecessors
    for (auto w : info.succ) {
        infoMap[w].pred.erase(v);
    }
}

static
bool hasInEdgeTops(const NGHolder &g, NFAVertex v) {
    NFAEdge e = edge(g.start, v, g);
    return e && !g[e].tops.empty();
}

/** Transform (1), removal of redundant vertices. */
static
bool doUselessMergePass(NGHolder &g, som_type som, VertexInfoMap &infoMap,
                        set<NFAVertex> &removable) {
    /* useless merges can be done in any order, no need to take any care with
     * ordering */

    // Temporary vectors used for intersections below
    vector<NFAVertex> succPredSet, predSuccSet, intersection;

    bool changed = false;
    for (auto v : vertices_range(g)) {
        VertexInfo &info = infoMap[v];

        if (info.isRemoved) {
            continue;
        }

        assert(!contains(removable, v));

        if (is_special(v, g)) {
            continue;
        }

        /* we do not need to check for out edge tops - as only specials (start)
         * can have tops and they are already disqualified. */
        if (hasInEdgeTops(g, v)) {
            continue; // Conservatively skip anything with nonzero tops.
        }

        if (info.pred.empty() || info.succ.empty()) {
            DEBUG_PRINTF("vertex %zu has empty pred/succ list\n", g[v].index);
            assert(0); // non-special states should always have succ/pred lists
            continue;
        }

        // The following cases are more complex and rely on the intersection of
        // Succ(Pred(v)) and Pred(Succ(v))

        // Compute intersections, operating on the smaller set first
        // Note that we use vectors here, as set_intersection underneath
        // guarantees sorted output, and vectors were quite a bit
        // faster than sets or lists.

        succPredSet.clear();
        predSuccSet.clear();

        if (info.pred.size() <= info.succ.size()) {
            succPredIntersection(v, info.pred, infoMap, succPredSet);
            if (succPredSet.size() == 1) {
                // nobody in here but us chickens
                assert(*succPredSet.begin() == v);
                continue;
            }
            predSuccIntersection(v, info.succ, infoMap, predSuccSet);
            if (predSuccSet.size() == 1) {
                assert(*predSuccSet.begin() == v);
                continue;
            }
        } else {
            predSuccIntersection(v, info.succ, infoMap, predSuccSet);
            if (predSuccSet.size() == 1) {
                assert(*predSuccSet.begin() == v);
                continue;
            }
            succPredIntersection(v, info.pred, infoMap, succPredSet);
            if (succPredSet.size() == 1) {
                assert(*succPredSet.begin() == v);
                continue;
            }
        }

        // Find the intersection of Succ(Pred(v)) and Pred(Succ(v))
        intersection.clear();
        set_intersection(succPredSet.begin(), succPredSet.end(),
                         predSuccSet.begin(), predSuccSet.end(),
                         back_inserter(intersection));

        /* Boring if it is just us in the intersection */
        if (intersection.size() < 2) {
            continue;
        }

        // Compare char_reach, mark v for removal if any members of
        // the intersection have an equal or greater reach
        const CharReach &currReach = g[v].char_reach;
        const auto &currReports = g[v].reports;
        for (auto t : intersection) {
            const VertexInfo &info2 = infoMap[t];

            /* start is never a succ of a state, so will never be in the
             * predsucc/succpred intersection */
            assert(t != g.start);

            if (t == v || info2.isRemoved) {
                continue;
            }

            // For each candidate C to make V redundant, check:
            // if V is an accept state, C must be an accept state for
            //      the same pattern
            // pred(C) is a superset of pred(V)
            // succ(C) is a superset of succ(V)
            // reach(C) is a superset of reach(V)
            //
            // Note: pred/sec tests are covered by the intersections
            // calculated above.

            /* note: links to accepts are also tracked in succs */
            if (info.isAccept && currReports != g[t].reports) {
                continue;
            }

            if (som) {
                if (t == g.startDs) {
                    continue;
                }
                if (is_virtual_start(t, g) != is_virtual_start(v, g)) {
                    continue;
                }
            }

            /* we do not need to check for out edge tops - as only start
             * can have tops and it has already been ruled out. */
            if (hasInEdgeTops(g, t)) {
                continue; // Conservatively skip anything with nonzero tops.
            }

            CharReach &otherReach = g[t].char_reach;
            if (currReach.isSubsetOf(otherReach)) {
                DEBUG_PRINTF("removing redundant vertex %zu (keeping %zu)\n",
                             g[v].index, g[t].index);
                markForRemoval(v, infoMap, removable);
                changed = true;
                break;
            }
        }
    }

    return changed;
}

/** Transform (2), diamond merge pass. */
static
bool doDiamondMergePass(NGHolder &g, som_type som, VertexInfoMap &infoMap,
                        set<NFAVertex> &removable) {
    // Temporary vectors used for intersections below
    vector<NFAVertex> succPredSet, predSuccSet, intersection;

    bool changed = false;
    for (auto v : vertices_range(g)) {
        VertexInfo &info = infoMap[v];

        if (info.isRemoved) {
            continue;
        }

        assert(!contains(removable, v));

        if (is_special(v, g)) {
            continue;
        }

        /* we do not need to check for out edge tops - as only specials (start)
         * can have tops and they are already disqualified. */
        if (hasInEdgeTops(g, v)) {
            continue; // Conservatively skip anything with nonzero tops.
        }

        if (info.pred.empty() || info.succ.empty()) {
            assert(0); // non-special states should always have succ/pred lists
            continue;
        }

        // The following cases are more complex and rely on the intersection of
        // Succ(Pred(v)) and Pred(Succ(v))

        // Compute intersections, operating on the smaller set first
        // Note that we use vectors here, as set_intersection underneath
        // guarantees sorted output, and vectors were quite a bit faster than
        // sets or lists.

        succPredSet.clear();
        predSuccSet.clear();

        if (info.pred.size() <= info.succ.size()) {
            succPredIntersection(v, info.pred, infoMap, succPredSet);
            if (succPredSet.size() == 1) {
                // nobody in here but us chickens
                assert(*succPredSet.begin() == v);
                continue;
            }
            predSuccIntersection(v, info.succ, infoMap, predSuccSet);
            if (predSuccSet.size() == 1) {
                assert(*predSuccSet.begin() == v);
                continue;
            }
        } else {
            predSuccIntersection(v, info.succ, infoMap, predSuccSet);
            if (predSuccSet.size() == 1) {
                assert(*predSuccSet.begin() == v);
                continue;
            }
            succPredIntersection(v, info.pred, infoMap, succPredSet);
            if (succPredSet.size() == 1) {
                assert(*succPredSet.begin() == v);
                continue;
            }
        }

        // Find the intersection of Succ(Pred(v)) and Pred(Succ(v))
        intersection.clear();
        set_intersection(succPredSet.begin(), succPredSet.end(),
                         predSuccSet.begin(), predSuccSet.end(),
                         back_inserter(intersection));

        /* Boring if it is just us in the intersection */
        if (intersection.size() < 2) {
            continue;
        }

        const CharReach &currReach = g[v].char_reach;
        const auto &currReports = g[v].reports;
        for (auto t : intersection) {
            const VertexInfo &info2 = infoMap[t];

            if (t == v || info2.isRemoved || is_special(t, g)) {
                continue;
            }

            /* note: links to accepts are also tracked in succs */
            if (info.isAccept && currReports != g[t].reports) {
                continue;
            }

            /* we do not need to check for out edge tops - as only specials
             * (start) can have tops and they are already disqualified. */
            if (hasInEdgeTops(g, t)) {
                continue; // Conservatively skip anything with nonzero tops.
            }

            if (som) {
                if (is_virtual_start(v, g) != is_virtual_start(t, g)) {
                    continue; // can only merge like with like.
                }
            }

            // If in-degree of v == in-degree of target
            // and out-degree of v == out-degree of target
            //    (because pred and succ are supersets)
            // then combine charreach of v into target and remove v
            if (info.inDegree() == info2.inDegree()
                && info.outDegree() == info2.outDegree()) {
                // add character reachability of v into target
                CharReach &otherReach = g[t].char_reach;
                otherReach |= currReach;
                // v can be removed
                DEBUG_PRINTF("removing redundant vertex %zu and merging "
                             "reachability with vertex %zu\n",
                             g[v].index, g[t].index);
                markForRemoval(v, infoMap, removable);
                changed = true;
                break;
            }
        }
    }

    return changed;
}

namespace {

struct ReachMismatch {};

class ReachSubsetVisitor : public boost::default_dfs_visitor {
public:
    explicit ReachSubsetVisitor(const CharReach &r) : cr(r) {}

    template <class Graph, class Vertex>
    void discover_vertex(const Vertex &v, const Graph &g) const {
        if (is_any_start(v, g)) {
            return; // start vertices are OK
        } else if (is_special(v, g)) {
            assert(0);
            throw ReachMismatch(); // other special nodes??
        }

        const CharReach &vcr = g[v].char_reach;
        DEBUG_PRINTF("checking if vcr (%zu) is subset of (%zu)\n", vcr.count(),
                     cr.count());
        if (vcr != (vcr & cr)) {
            throw ReachMismatch();
        }
    }

private:
    const CharReach &cr;
};

/** Terminator function for DFS used in pathReachSubset. */
template <class Graph, class Vertex> class VertexIs {
public:
    explicit VertexIs(const Vertex &v) : vertex(v) {}
    bool operator()(const Vertex &v, const Graph &) const {
        return v == vertex;
    }

private:
    Vertex vertex;
};

} // namespace

/** Returns true if every vertex on paths leading to edge \p e has reachability
 * which is a subset of the reachability of \p dom */
static
bool reversePathReachSubset(const NFAEdge &e, const NFAVertex &dom,
                            const NGHolder &g) {
    const CharReach &domReach = g[dom].char_reach;
    if (domReach.all()) {
        return true;
    }

    NFAVertex start = source(e, g);
    using RevGraph = boost::reverse_graph<NGHolder, const NGHolder &>;
    map<RevGraph::vertex_descriptor, boost::default_color_type> vertexColor;

    // Walk the graph backwards from v, examining each node. We fail (return
    // false) if we encounter a node with reach NOT a subset of domReach, and
    // we stop searching at dom.
    try {
        depth_first_visit(RevGraph(g), start,
                          ReachSubsetVisitor(domReach),
                          make_assoc_property_map(vertexColor),
                          VertexIs<RevGraph, RevGraph::vertex_descriptor>(dom));
    } catch(ReachMismatch&) {
        return false;
    }

    return true;
}

/** Returns true if every vertex on paths leading from edge \p e has
 * reachability which is a subset of the reachability of \p dom */
static
bool forwardPathReachSubset(const NFAEdge &e, const NFAVertex &dom,
                            const NGHolder &g) {
    const CharReach &domReach = g[dom].char_reach;
    if (domReach.all()) {
        return true;
    }

    NFAVertex start = target(e, g);
    map<NFAVertex, boost::default_color_type> vertexColor;

    // Walk the graph forward from v, examining each node. We fail (return
    // false) if we encounter a node with reach NOT a subset of domReach, and
    // we stop searching at dom.
    try {
        depth_first_visit(g, start, ReachSubsetVisitor(domReach),
                          make_assoc_property_map(vertexColor),
                          VertexIs<NGHolder, NFAVertex>(dom));
    } catch(ReachMismatch&) {
        return false;
    }

    return true;
}

static
bool allOutsSpecial(NFAVertex v, const NGHolder &g) {
    for (auto w : adjacent_vertices_range(v, g)) {
        if (!is_special(w, g)) {
            return false;
        }
    }
    return true;
}

static
bool allInsSpecial(NFAVertex v, const NGHolder &g) {
    for (auto u : inv_adjacent_vertices_range(v, g)) {
        if (!is_special(u, g)) {
            return false;
        }
    }
    return true;
}

/** Cheaply check whether this graph can't be reduced at all, because it is
 * just a chain of vertices with no other edges. */
static
bool isIrreducible(const NGHolder &g) {
    for (auto v : vertices_range(g)) {
        // skip specials
        if (is_special(v, g)) {
            continue;
        }

        if (in_degree(v, g) != 1 && !allInsSpecial(v, g)) {
            return false;
        }
        if (out_degree(v, g) != 1 && !allOutsSpecial(v, g)) {
            return false;
        }
    }

    /* if calcComponents got sleepy and went home, the above checks don't hold
     * as it assumes there is only one connected component. */
    if (isAlternationOfClasses(g)) {
        return false;
    }

    return true;
}

static
u32 findCyclic(const NGHolder &g, vector<bool> &cyclic) {
    u32 count = 0;

    cyclic.resize(num_vertices(g));

    for (auto v : vertices_range(g)) {
        assert(g[v].index < cyclic.size());
        if (hasSelfLoop(v, g)) {
            count++;
            cyclic[g[v].index] = true;
        }
    }

    return count;
}

static
void findCyclicDom(NGHolder &g, vector<bool> &cyclic,
                   set<NFAEdge> &dead, som_type som) {
    auto dominators = findDominators(g);

    for (auto v : vertices_range(g)) {
        if (is_special(v, g)) {
            continue;
        }

        // Path in through a dominator (e.g. '.+a?foobar')
        NFAVertex dom = dominators[v];
        if (dom && cyclic[g[dom].index]
            && edge(dom, v, g).second) {

            if (som && dom == g.startDs) {
                continue;
            }

            DEBUG_PRINTF("vertex %zu is dominated by directly-connected cyclic "
                         "vertex %zu\n", g[v].index, g[dom].index);

            // iff all paths through in-edge e of v involve vertices whose
            // reachability is a subset of reach(dom), we can delete edge e.
            for (const auto &e : in_edges_range(v, g)) {
                if (source(e, g) == dom) {
                    continue;
                }

                if (reversePathReachSubset(e, dom, g)) {
                    DEBUG_PRINTF("edge (%zu, %zu) can be removed: leading "
                                 "paths share dom reach\n",
                                 g[source(e, g)].index, g[target(e, g)].index);
                    dead.insert(e);
                    if (source(e, g) == v) {
                        cyclic[g[v].index] = false;
                    }
                    continue;
                }
            }
        }
    }
}

static
void findCyclicPostDom(NGHolder &g, vector<bool> &cyclic,
                       set<NFAEdge> &dead) {
    auto postdominators = findPostDominators(g);

    for (auto v : vertices_range(g)) {
        if (is_special(v, g)) {
            continue;
        }

        // Path out through a post-dominator (e.g. a?.+foobar')
        NFAVertex postdom = postdominators[v];
        if (postdom && cyclic[g[postdom].index] && edge(v, postdom, g).second) {
            DEBUG_PRINTF("vertex %zu is postdominated by directly-connected "
                         "cyclic vertex %zu\n", g[v].index, g[postdom].index);

            // iff all paths through in-edge e of v involve vertices whose
            // reachability is a subset of reach(dom), we can delete edge e.
            for (const auto &e : out_edges_range(v, g)) {
                if (target(e, g) == postdom) {
                    continue;
                }

                if (forwardPathReachSubset(e, postdom, g)) {
                    DEBUG_PRINTF("edge (%zu, %zu) can be removed: trailing "
                                 "paths share postdom reach\n",
                                 g[source(e, g)].index, g[target(e, g)].index);
                    if (target(e, g) == v) {
                        cyclic[g[v].index] = false;
                    }
                    dead.insert(e);
                    continue;
                }
            }
        }
    }
}

bool removeRedundancy(NGHolder &g, som_type som) {
    DEBUG_PRINTF("rr som = %d\n", (int)som);
    renumber_vertices(g);

    // Cheap check: if all the non-special vertices have in-degree one and
    // out-degree one, there's no redundancy in this here graph and we can
    // vamoose.
    if (isIrreducible(g)) {
        return false;
    }

    VertexInfoMap infoMap(g);

    // Populate maps of successors and predecessors, and accept status
    populateContainers(g, infoMap);

    /* Run multiple passes: terminate when a full pass doesn't remove
     * any vertices */
    bool doUseless = true;
    bool doDiamond = true;
    set<NFAVertex> removable;
    while (doUseless || doDiamond) {
        if (doUseless
            && doUselessMergePass(g, som, infoMap, removable)) {
            doDiamond = true;
        }
        doUseless = false;

        if (doDiamond
            && doDiamondMergePass(g, som, infoMap, removable)) {
            doUseless = true;
        }
        doDiamond = false;
    }
    DEBUG_PRINTF("found %zu removable vertices overall.\n", removable.size());
    remove_vertices(removable, g);

    return !removable.empty();
}

/** UE-524: remove edges into nodes that are dominated by cyclic nodes with
 * reachability that is a superset of all paths feeding into that edge. */
bool removeCyclicDominated(NGHolder &g, som_type som) {
    set<NFAEdge> dead;
    vector<bool> cyclic;
    bool changed = false;

    findCyclic(g, cyclic);

    findCyclicDom(g, cyclic, dead, som);
    if (!dead.empty()) {
        remove_edges(dead, g);
        pruneUseless(g);
        dead.clear();
        cyclic.clear();  // need to recalculate cyclic as ids have changed
        findCyclic(g, cyclic);
        changed = true;
    }

    findCyclicPostDom(g, cyclic, dead);
    if (!dead.empty()) {
        remove_edges(dead, g);
        pruneUseless(g);
        dead.clear();
        changed = true;
    }

    return changed;
}

} // namespace ue2
