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
 * \brief Literal analysis and scoring.
 */
#include "ng_literal_analysis.h"

#include "ng_holder.h"
#include "ng_split.h"
#include "ng_util.h"
#include "ue2common.h"
#include "rose/rose_common.h"
#include "util/compare.h"
#include "util/depth.h"
#include "util/graph.h"
#include "util/graph_range.h"
#include "util/ue2string.h"

#include <algorithm>
#include <fstream>
#include <queue>

#include <boost/graph/boykov_kolmogorov_max_flow.hpp>

using namespace std;
using boost::vertex_index;

namespace ue2 {

/** Maximum number of paths to generate. */
static const u32 MAX_WIDTH = 11;

/** Scoring adjustment for 'uniqueness' in literal. */
static const u64a WEIGHT_OF_UNIQUENESS = 250;

namespace {

/* Small literal graph type used for the suffix tree used in
 * compressAndScore. */

typedef boost::adjacency_list_traits<boost::vecS, boost::vecS,
                                     boost::bidirectionalS> LitGraphTraits;
typedef LitGraphTraits::vertex_descriptor LitVertex;
typedef LitGraphTraits::edge_descriptor LitEdge;

struct LitGraphVertexProps {
    LitGraphVertexProps() {}
    explicit LitGraphVertexProps(const ue2_literal::elem &c_in) : c(c_in) {}
    ue2_literal::elem c; // string element (char + bool)
};

struct LitGraphEdgeProps {
    LitGraphEdgeProps() {}
    explicit LitGraphEdgeProps(u64a score_in) : score(score_in) {}
    u64a score = NO_LITERAL_AT_EDGE_SCORE;
};

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS,
                              LitGraphVertexProps, LitGraphEdgeProps,
                              boost::no_property> LitGraph;

typedef pair<LitVertex, NFAVertex> VertexPair;
typedef std::queue<VertexPair> LitVertexQ;

} // namespace

#ifdef DUMP_SUPPORT

/** \brief Dump the literal graph in Graphviz format. */
static UNUSED
void dumpGraph(const char *filename, const LitGraph &lg, const LitVertex &root,
               const LitVertex &sink) {
    ofstream fout(filename);

    fout << "digraph G {" << endl;

    for (auto v : vertices_range(lg)) {
        fout << boost::get(vertex_index, lg, v);
        if (v == root) {
            fout << "[label=\"ROOT\"];";
        } else if (v == sink) {
            fout << "[label=\"SINK\"];";
        } else {
            ue2_literal s;
            s.push_back(lg[v].c);
            fout << "[label=\"" << dumpString(s) << "\"];";
        }
        fout << endl;
    }

    for (const auto &e : edges_range(lg)) {
        LitVertex u = source(e, lg), v = target(e, lg);
        fout << boost::get(vertex_index, lg, u) << " -> " <<
                boost::get(vertex_index, lg, v) <<
                "[label=\"" << lg[e].score << "\"]" <<
                ";" << endl;
    }

    fout << "}" << endl;
}

#endif // DUMP_SUPPORT

static
bool allowExpand(size_t numItems, size_t totalPathsSoFar) {
    if (numItems == 0) {
        return false;
    }

    if (numItems + totalPathsSoFar > MAX_WIDTH) {
        return false;
    }

    return true;
}

static
LitVertex addToLitGraph(LitGraph &lg, LitVertex sink,
                        LitVertex pred, const ue2_literal::elem &c) {
    // Check if we already have this in the graph.
    for (auto v : adjacent_vertices_range(pred, lg)) {
        if (v == sink) {
            continue;
        }
        if (lg[v].c == c) {
            return v;
        }
    }

    LitVertex lv = add_vertex(LitGraphVertexProps(c), lg);
    add_edge(pred, lv, lg);
    return lv;
}

static
void addToQueue(LitVertexQ &workQ, LitGraph &lg, LitVertex sink,
                LitVertex pred, const CharReach &cr, NFAVertex v) {
    for (size_t i = cr.find_first(); i != CharReach::npos; i = cr.find_next(i)) {
        if (myisupper(i) && cr.test(mytolower(i))) {
            // ignore upper half of a nocase pair
            continue;
        }

        bool nocase = myislower(i) && cr.test(mytoupper(i));
        ue2_literal::elem c((char)i, nocase);
        LitVertex lv = addToLitGraph(lg, sink, pred, c);
        workQ.push(VertexPair(lv, v));
    }
}

static
void initWorkQueue(LitVertexQ &workQ, LitGraph &lg, LitVertex root,
                   LitVertex sink, const NGHolder &g, const NFAEdge &e) {
    NFAVertex u = source(e, g);
    NFAVertex v = target(e, g);
    const CharReach &cr = g[v].char_reach;

    if (!allowExpand(cr.count(), 0)) {
        return;
    }

    addToQueue(workQ, lg, sink, root, cr, u);
}

static
u32 crCardinality(const CharReach &cr) {
    // Special-case for handling dots, much faster than running the find_next
    // loop below.
    if (cr.all()) {
        return 230; // [^A-Z]
    }

    u32 rv = 0;
    for (size_t i = cr.find_first(); i != CharReach::npos; i = cr.find_next(i)) {
        if (myisupper(i) && cr.test(mytolower(i))) {
            // ignore upper half of a nocase pair
            continue;
        }
        rv++;
    }

    return rv;
}

/** Filter out literals that include other literals as suffixes. We do this by
 * identifying vertices connected to the sink and removing their other
 * out-edges. */
static
void filterLitGraph(LitGraph &lg, const LitVertex sink) {
    for (auto v : inv_adjacent_vertices_range(sink, lg)) {
        remove_out_edge_if(v, [&lg, &sink](const LitEdge &e) {
            return target(e, lg) != sink;
        }, lg);
    }

    // We could do a DFS-and-prune here, if we wanted. Right now, we just
    // handle it in extractLiterals by throwing away paths that don't run all
    // the way from sink to root.
}

/** Extracts all the literals from the given literal graph. Walks the graph
 * from each predecessor of the sink (note: it's a suffix tree except for this
 * convenience) towards the source, storing each string as we go. */
static
void extractLiterals(const LitGraph &lg, const LitVertex root,
                     const LitVertex sink, set<ue2_literal> &s) {
    ue2_literal lit;

    for (auto u : inv_adjacent_vertices_range(sink, lg)) {
        lit.clear();
        while (u != root) {
            lit.push_back(lg[u].c);
            assert(in_degree(u, lg) <= 1);
            LitGraph::inv_adjacency_iterator ai2, ae2;
            tie(ai2, ae2) = inv_adjacent_vertices(u, lg);
            if (ai2 == ae2) {
                // Path has been cut, time for the next literal.
                goto next_literal;
            }
            u = *ai2;
        }
        s.insert(lit);
next_literal:
        ;
    }
}

#ifndef NDEBUG
static
bool hasSuffixLiterals(const set<ue2_literal> &s) {
    for (auto it = s.begin(), ite = s.end(); it != ite; ++it) {
        for (auto jt = std::next(it); jt != ite; ++jt) {
            if (isSuffix(*it, *jt) || isSuffix(*jt, *it)) {
                DEBUG_PRINTF("'%s' and '%s' have suffix issues\n",
                             dumpString(*it).c_str(),
                             dumpString(*jt).c_str());
                return true;
            }
        }
    }
    return false;
}
#endif

static
void processWorkQueue(const NGHolder &g, const NFAEdge &e,
                      set<ue2_literal> &s) {
    if (is_special(target(e, g), g)) {
        return;
    }

    LitGraph lg;
    LitVertex root = add_vertex(lg);
    LitVertex sink = add_vertex(lg);

    LitVertexQ workQ;
    initWorkQueue(workQ, lg, root, sink, g, e);

    while (!workQ.empty()) {
        const LitVertex lv = workQ.front().first;
        const NFAVertex &t = workQ.front().second;
        const CharReach &cr = g[t].char_reach;

        u32 cr_card = crCardinality(cr);
        size_t numItems = cr_card * in_degree(t, g);
        size_t committed_count = workQ.size() + in_degree(sink, lg) - 1;

        if (g[t].index == NODE_START) {
            // reached start, add to literal set
            add_edge_if_not_present(lv, sink, lg);
            goto next_work_elem;
        }

        // Expand next vertex
        if (allowExpand(numItems, committed_count)) {
            for (auto u : inv_adjacent_vertices_range(t, g)) {
                addToQueue(workQ, lg, sink, lv, cr, u);
            }
            goto next_work_elem;
        }

        // Expand this vertex
        if (allowExpand(cr_card, committed_count)) {
            for (size_t i = cr.find_first(); i != CharReach::npos;
                 i = cr.find_next(i)) {
                if (myisupper(i) && cr.test(mytolower(i))) {
                    // ignore upper half of a nocase pair
                    continue;
                }

                bool nocase = myislower(i) && cr.test(mytoupper(i));
                ue2_literal::elem c((char)i, nocase);
                LitVertex lt = addToLitGraph(lg, sink, lv, c);
                add_edge_if_not_present(lt, sink, lg);
            }
            goto next_work_elem;
        }

        // add to literal set
        add_edge_if_not_present(lv, sink, lg);
    next_work_elem:
        workQ.pop();
    }

    filterLitGraph(lg, sink);
    //dumpGraph("litgraph.dot", lg, root, sink);
    extractLiterals(lg, root, sink, s);

    // Our literal set should contain no literal that is a suffix of another.
    assert(!hasSuffixLiterals(s));

    DEBUG_PRINTF("edge %u (%u->%u) produced %zu literals\n", g[e].index,
                 g[source(e, g)].index, g[target(e, g)].index, s.size());
}

static
u64a litUniqueness(const string &s) {
    CharReach seen(s);
    return seen.count();
}

/** Count the significant bits of this literal (i.e. seven for nocase alpha,
 * eight for everything else). */
static
u64a litCountBits(const ue2_literal &lit) {
    u64a n = 0;
    for (const auto &c : lit) {
        n += c.nocase ? 7 : 8;
    }
    return n;
}

/** Returns a fairly arbitrary score for the given literal, used to compare the
 * suitability of different candidates. */
static
u64a scoreLiteral(const ue2_literal &s) {
    // old scoring scheme: SUM(s in S: 1/s.len()^2)
    // now weight (currently 75/25) with number of unique chars
    // in the string
    u64a len = litCountBits(s);
    u64a lenUnique = litUniqueness(s.get_string()) * 8;

    u64a weightedLen = (1000ULL - WEIGHT_OF_UNIQUENESS) * len +
                         WEIGHT_OF_UNIQUENESS * lenUnique;
    weightedLen /= 8;

    DEBUG_PRINTF("scored literal '%s' %llu\n",
                 escapeString(s.get_string()).c_str(), weightedLen);

    return weightedLen;
}


/**
 * calculateScore has the following properties:
 * - score of literal is the same as the score of the reversed literal;
 * - score of substring of literal is worse than the original literal's score;
 * - score of any literal should be non-zero.
 */
static
u64a calculateScore(const ue2_literal &s) {
    if (s.empty()) {
        return NO_LITERAL_AT_EDGE_SCORE;
    }

    u64a weightedLen = scoreLiteral(s);

    DEBUG_PRINTF("len %zu, wl %llu\n", s.length(), weightedLen);
    u64a rv = 1000000000000000ULL/(weightedLen * weightedLen * weightedLen);

    if (!rv) {
        rv = 1;
    }
    DEBUG_PRINTF("len %zu, score %llu\n", s.length(), rv);
    return rv;
}

/** Adds a literal in reverse order, building up a suffix tree. */
static
void addReversedLiteral(const ue2_literal &lit, LitGraph &lg,
                        const LitVertex &root, const LitVertex &sink) {
    DEBUG_PRINTF("literal: '%s'\n", escapeString(lit).c_str());
    ue2_literal suffix;
    LitVertex v = root;
    for (auto it = lit.rbegin(), ite = lit.rend(); it != ite; ++it) {
        suffix.push_back(*it);
        LitVertex w;
        for (auto v2 : adjacent_vertices_range(v, lg)) {
            if (v2 != sink && lg[v2].c == *it) {
                w = v2;
                goto next_char;
            }
        }
        w = add_vertex(LitGraphVertexProps(*it), lg);
        add_edge(v, w, LitGraphEdgeProps(calculateScore(suffix)), lg);
next_char:
        v = w;
    }

    // Wire the last vertex to the sink.
    add_edge(v, sink, lg);
}

static
void extractLiterals(const vector<LitEdge> &cutset, const LitGraph &lg,
                     const LitVertex &root, set<ue2_literal> &s) {
    for (const auto &e : cutset) {
        LitVertex u = source(e, lg), v = target(e, lg);
        ue2_literal lit;
        lit.push_back(lg[v].c);
        while (u != root) {
            lit.push_back(lg[u].c);
            assert(in_degree(u, lg) == 1);
            LitGraph::inv_adjacency_iterator ai, ae;
            tie(ai, ae) = inv_adjacent_vertices(u, lg);
            if (ai == ae) {
                // Path has been cut, time for the next literal.
                goto next_literal;
            }
            u = *ai;
        }
        DEBUG_PRINTF("extracted: '%s'\n", escapeString(lit).c_str());
        s.insert(lit);
next_literal:
        ;
    }
}

#ifdef DEBUG
static UNUSED
const char *describeColor(boost::default_color_type c) {
    switch (c) {
    case boost::white_color:
        return "white";
    case boost::gray_color:
        return "gray";
    case boost::green_color:
        return "green";
    case boost::red_color:
        return "red";
    case boost::black_color:
        return "black";
    default:
        return "unknown";
    }
}
#endif

/**
 * The BGL's boykov_kolmogorov_max_flow requires that all edges have their
 * reverse edge in the graph. This function adds them, returning the new edges
 * and constructing a map of (edge, rev edge).
 */
static
vector<LitEdge> addReverseEdges(LitGraph &lg,
                ue2::unordered_map<LitEdge, LitEdge> &reverse_edge_map) {
    vector<LitEdge> reverseMe;

    reverse_edge_map.clear();
    reverse_edge_map.reserve(num_edges(lg) * 2);

    for (const auto &e : edges_range(lg)) {
        LitVertex u = source(e, lg), v = target(e, lg);
        assert(u != v);

        bool exists;
        LitEdge rev;
        tie(rev, exists) = edge(v, u, lg);
        if (exists) {
            reverse_edge_map[e] = rev;
        } else {
            reverseMe.push_back(e);
        }
    }

    vector<LitEdge> reverseEdges;
    reverseEdges.reserve(reverseMe.size());

    for (const auto &e : reverseMe) {
        LitVertex u = source(e, lg), v = target(e, lg);
        LitEdge rev = add_edge(v, u, lg[e], lg).first;
        reverseEdges.push_back(rev);
        reverse_edge_map[e] = rev;
        reverse_edge_map[rev] = e;
    }

    return reverseEdges;
}

static
void findMinCut(LitGraph &lg, const LitVertex &root, const LitVertex &sink,
                vector<LitEdge> &cutset) {
    cutset.clear();

    //dumpGraph("litgraph.dot", lg, root, sink);

    assert(!in_degree(root, lg));
    assert(!out_degree(sink, lg));

    // Add reverse edges for the convenience of the BGL's max flow algorithm.
    ue2::unordered_map<LitEdge, LitEdge> reverse_edge_map;
    vector<LitEdge> tempEdges = addReverseEdges(lg, reverse_edge_map);

    const auto v_index_map = get(vertex_index, lg);
    const size_t num_verts = num_vertices(lg);
    vector<boost::default_color_type> colors(num_verts);
    vector<s32> distances(num_verts);
    vector<LitEdge> predecessors(num_verts);
    ue2::unordered_map<LitEdge, u64a> residuals;
    residuals.reserve(num_edges(lg));

    UNUSED u64a flow = boykov_kolmogorov_max_flow(lg,
            get(&LitGraphEdgeProps::score, lg),
            make_assoc_property_map(residuals),
            make_assoc_property_map(reverse_edge_map),
            make_iterator_property_map(predecessors.begin(), v_index_map),
            make_iterator_property_map(colors.begin(), v_index_map),
            make_iterator_property_map(distances.begin(), v_index_map),
            get(vertex_index, lg), root, sink);
    DEBUG_PRINTF("done, flow = %llu\n", flow);

    // Remove temporary reverse edges.
    for (const auto &e : tempEdges) {
        remove_edge(e, lg);
    }

    vector<LitEdge> white_cut, black_cut;
    u64a white_flow = 0, black_flow = 0;

    for (const auto &e : edges_range(lg)) {
        const LitVertex u = source(e, lg), v = target(e, lg);
        const auto ucolor = colors[boost::get(vertex_index, lg, u)];
        const auto vcolor = colors[boost::get(vertex_index, lg, v)];

        DEBUG_PRINTF("edge %zu:%s -> %zu:%s score %llu\n",
                     boost::get(vertex_index, lg, u), describeColor(ucolor),
                     boost::get(vertex_index, lg, v), describeColor(vcolor),
                     lg[e].score);

        if (ucolor != boost::white_color && vcolor == boost::white_color) {
            assert(target(e, lg) != sink);
            white_cut.push_back(e);
            white_flow += lg[e].score;
        }
        if (ucolor == boost::black_color && vcolor != boost::black_color) {
            assert(target(e, lg) != sink);
            black_cut.push_back(e);
            black_flow += lg[e].score;
        }
    }

    DEBUG_PRINTF("white flow = %llu, black flow = %llu\n",
                 white_flow, black_flow);
    assert(white_flow && black_flow);

    if (white_flow <= black_flow) {
        DEBUG_PRINTF("selected white cut\n");
        cutset.swap(white_cut);
    } else {
        DEBUG_PRINTF("selected black cut\n");
        cutset.swap(black_cut);
    }

    DEBUG_PRINTF("min cut has %zu edges\n", cutset.size());
    assert(!cutset.empty());
}

/** Takes a set of literals and derives a better one from them, returning its
 * score. Literals with a common suffix S will be replaced with S. (for
 * example, {foobar, fooobar} -> {oobar}).
 */
u64a compressAndScore(set<ue2_literal> &s) {
    if (s.empty()) {
        return NO_LITERAL_AT_EDGE_SCORE;
    }

    if (s.size() == 1) {
        return calculateScore(*s.begin());
    }

    UNUSED u64a initialScore = scoreSet(s);
    DEBUG_PRINTF("begin, initial literals have score %llu\n",
                  initialScore);

    LitGraph lg;
    const LitVertex root = add_vertex(lg);
    const LitVertex sink = add_vertex(lg);

    for (const auto &lit : s) {
        addReversedLiteral(lit, lg, root, sink);
    }

    DEBUG_PRINTF("suffix tree has %zu vertices and %zu edges\n",
                  num_vertices(lg), num_edges(lg));

    vector<LitEdge> cutset;
    findMinCut(lg, root, sink, cutset);

    s.clear();
    extractLiterals(cutset, lg, root, s);

    u64a score = scoreSet(s);
    DEBUG_PRINTF("compressed score is %llu\n", score);
    assert(score <= initialScore);
    return score;
}

u64a scoreSet(const set<ue2_literal> &s) {
    if (s.empty()) {
        return NO_LITERAL_AT_EDGE_SCORE;
    }

    u64a score = 1ULL;

    for (const auto &lit : s) {
        score += calculateScore(lit);
    }

    return score;
}

set<ue2_literal> getLiteralSet(const NGHolder &g, const NFAEdge &e) {
    set<ue2_literal> s;
    processWorkQueue(g, e, s);
    return s;
}

set<ue2_literal> getLiteralSet(const NGHolder &g, const NFAVertex &v,
                               bool only_first_encounter) {
    set<ue2_literal> s;

    if (is_special(v, g)) {
        return s;
    }

    set<ue2_literal> ls;

    for (const auto &e : in_edges_range(v, g)) {
        if (source(e, g) == v && only_first_encounter) {
            continue; /* ignore self loop on root vertex as we are interested in
                       * the first time we visit the vertex on the way to
                       * accept. In fact, we can ignore any back edges - but
                       * they would require a bit of effort to discover. */
        }

        ls = getLiteralSet(g, e);
        if (ls.empty()) {
            s.clear();
            return s;
        } else {
            s.insert(ls.begin(), ls.end());
        }
    }

    return s;
}

vector<u64a> scoreEdges(const NGHolder &g) {
    assert(hasCorrectlyNumberedEdges(g));

    vector<u64a> scores(num_edges(g));

    for (const auto &e : edges_range(g)) {
        u32 eidx = g[e].index;
        assert(eidx < scores.size());
        set<ue2_literal> ls = getLiteralSet(g, e);
        scores[eidx] = compressAndScore(ls);
    }

    return scores;
}

static
bool splitOffLeadingLiteral_i(const NGHolder &g, bool anch,
                              ue2_literal *lit_out,
                              NGHolder *rhs) {
    NFAVertex u;
    NFAVertex v;

    if (!anch) {
        DEBUG_PRINTF("looking for leading floating literal\n");
        set<NFAVertex> s_succ;
        insert(&s_succ, adjacent_vertices(g.start, g));

        set<NFAVertex> sds_succ;
        insert(&sds_succ, adjacent_vertices(g.startDs, g));

        bool floating = is_subset_of(s_succ, sds_succ);
        if (!floating) {
            DEBUG_PRINTF("not floating\n");
            return false;
        }

        sds_succ.erase(g.startDs);
        if (sds_succ.size() != 1) {
            DEBUG_PRINTF("branchy root\n");
            return false;
        }

        u = g.startDs;
        v = *sds_succ.begin();
    } else {
        DEBUG_PRINTF("looking for leading anchored literal\n");

        if (proper_out_degree(g.startDs, g)) {
            DEBUG_PRINTF("not anchored\n");
            return false;
        }

        set<NFAVertex> s_succ;
        insert(&s_succ, adjacent_vertices(g.start, g));
        s_succ.erase(g.startDs);
        if (s_succ.size() != 1) {
            DEBUG_PRINTF("branchy root\n");
            return false;
        }

        u = g.start;
        v = *s_succ.begin();
    }

    while (true) {
        DEBUG_PRINTF("validating vertex %u\n", g[v].index);

        assert(v != g.acceptEod && v != g.accept);

        const CharReach &cr = g[v].char_reach;
        if (cr.count() != 1 && !cr.isCaselessChar()) {
            break;
        }

        // Rose can only handle mixed-sensitivity literals up to the max mask
        // length.
        if (lit_out->length() >= MAX_MASK2_WIDTH) {
            if (mixed_sensitivity(*lit_out)) {
                DEBUG_PRINTF("long and mixed sensitivity\n");
                break;
            }
            if (ourisalpha((char)cr.find_first())) {
                if (cr.isCaselessChar() != lit_out->any_nocase()) {
                    DEBUG_PRINTF("stop at mixed sensitivity on '%c'\n",
                                 (char)cr.find_first());
                    break;
                }
            }
        }

        if (edge(v, g.accept, g).second || edge(v, g.acceptEod, g).second) {
            DEBUG_PRINTF("connection to accept\n");
            break;
        }

        lit_out->push_back(cr.find_first(), cr.isCaselessChar());
        u = v;

        if (out_degree(v, g) != 1) {
            DEBUG_PRINTF("out_degree != 1\n");
            break;
        }

        v = *adjacent_vertices(v, g).first;

        if (in_degree(v, g) != 1) {
            DEBUG_PRINTF("blargh\n"); /* picks up cases where there is no path
                                       * to case accept (large cycles),
                                       * ensures term */
            break;
        }
    }

    if (lit_out->empty()) {
        return false;
    }
    assert(u != g.startDs);

    ue2::unordered_map<NFAVertex, NFAVertex> rhs_map;
    vector<NFAVertex> pivots;
    insert(&pivots, pivots.end(), adjacent_vertices(u, g));
    splitRHS(g, pivots, rhs, &rhs_map);

    DEBUG_PRINTF("literal is '%s' (len %zu)\n", dumpString(*lit_out).c_str(),
                 lit_out->length());
    assert(is_triggered(*rhs));
    return true;
}

bool splitOffLeadingLiteral(const NGHolder &g, ue2_literal *lit_out,
                            NGHolder *rhs) {
    return splitOffLeadingLiteral_i(g, false, lit_out, rhs);
}

bool splitOffAnchoredLeadingLiteral(const NGHolder &g, ue2_literal *lit_out,
                                    NGHolder *rhs) {
    return splitOffLeadingLiteral_i(g, true, lit_out, rhs);
}


bool getTrailingLiteral(const NGHolder &g, ue2_literal *lit_out) {
    if (in_degree(g.acceptEod, g) != 1) {
        return false;
    }

    NFAVertex v = getSoleSourceVertex(g, g.accept);

    if (!v) {
        return false;
    }

    set<ue2_literal> s = getLiteralSet(g, v, false);

    if (s.size() != 1) {
        return false;
    }

    const ue2_literal &lit = *s.begin();

    if (lit.length() > MAX_MASK2_WIDTH && mixed_sensitivity(lit)) {
        DEBUG_PRINTF("long & mixed-sensitivity, Rose can't handle this.\n");
        return false;
    }

    *lit_out = lit;
    return true;
}

} // namespace ue2
