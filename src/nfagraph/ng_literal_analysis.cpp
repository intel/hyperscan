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
#include "util/graph_small_color_map.h"
#include "util/ue2_graph.h"
#include "util/ue2string.h"

#include <algorithm>
#include <fstream>
#include <queue>

#include <boost/graph/boykov_kolmogorov_max_flow.hpp>

using namespace std;

namespace ue2 {

/** Maximum number of paths to generate. */
static const u32 MAX_WIDTH = 11;

/** Scoring adjustment for 'uniqueness' in literal. */
static const u64a WEIGHT_OF_UNIQUENESS = 250;

namespace {

/* Small literal graph type used for the suffix tree used in
 * compressAndScore. */

struct LitGraphVertexProps {
    LitGraphVertexProps() = default;
    explicit LitGraphVertexProps(ue2_literal::elem c_in) : c(move(c_in)) {}
    ue2_literal::elem c; // string element (char + bool)
    size_t index = 0; // managed by ue2_graph
};

struct LitGraphEdgeProps {
    LitGraphEdgeProps() = default;
    explicit LitGraphEdgeProps(u64a score_in) : score(score_in) {}
    u64a score = NO_LITERAL_AT_EDGE_SCORE;
    size_t index = 0; // managed by ue2_graph
};

struct LitGraph
    : public ue2_graph<LitGraph, LitGraphVertexProps, LitGraphEdgeProps> {

    LitGraph() : root(add_vertex(*this)), sink(add_vertex(*this)) {}

    const vertex_descriptor root;
    const vertex_descriptor sink;
};

typedef LitGraph::vertex_descriptor LitVertex;
typedef LitGraph::edge_descriptor LitEdge;

typedef pair<LitVertex, NFAVertex> VertexPair;
typedef std::queue<VertexPair> LitVertexQ;

} // namespace

#ifdef DUMP_SUPPORT

/** \brief Dump the literal graph in Graphviz format. */
static UNUSED
void dumpGraph(const char *filename, const LitGraph &lg) {
    ofstream fout(filename);

    fout << "digraph G {" << endl;

    for (auto v : vertices_range(lg)) {
        fout << lg[v].index;
        if (v == lg.root) {
            fout << "[label=\"ROOT\"];";
        } else if (v == lg.sink) {
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
        fout << lg[u].index << " -> " << lg[v].index << "[label=\""
             << lg[e].score << "\"]"
             << ";" << endl;
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
LitVertex addToLitGraph(LitGraph &lg, LitVertex pred,
                        const ue2_literal::elem &c) {
    // Check if we already have this in the graph.
    for (auto v : adjacent_vertices_range(pred, lg)) {
        if (v == lg.sink) {
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
void addToQueue(LitVertexQ &workQ, LitGraph &lg, LitVertex pred,
                const CharReach &cr, NFAVertex v) {
    for (size_t i = cr.find_first(); i != CharReach::npos;
         i = cr.find_next(i)) {
        if (myisupper(i) && cr.test(mytolower(i))) {
            // ignore upper half of a nocase pair
            continue;
        }

        bool nocase = myislower(i) && cr.test(mytoupper(i));
        ue2_literal::elem c((char)i, nocase);
        LitVertex lv = addToLitGraph(lg, pred, c);
        workQ.push(VertexPair(lv, v));
    }
}

static
void initWorkQueue(LitVertexQ &workQ, LitGraph &lg, const NGHolder &g,
                   const NFAEdge &e) {
    NFAVertex u = source(e, g);
    NFAVertex v = target(e, g);
    const CharReach &cr = g[v].char_reach;

    if (!allowExpand(cr.count(), 0)) {
        return;
    }

    addToQueue(workQ, lg, lg.root, cr, u);
}

static
u32 crCardinality(const CharReach &cr) {
    // Special-case for handling dots, much faster than running the find_next
    // loop below.
    if (cr.all()) {
        return 230; // [^A-Z]
    }

    u32 rv = 0;
    for (size_t i = cr.find_first(); i != CharReach::npos;
         i = cr.find_next(i)) {
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
void filterLitGraph(LitGraph &lg) {
    for (auto v : inv_adjacent_vertices_range(lg.sink, lg)) {
        remove_out_edge_if(v, [&lg](const LitEdge &e) {
            return target(e, lg) != lg.sink;
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
void extractLiterals(const LitGraph &lg, set<ue2_literal> &s) {
    ue2_literal lit;

    for (auto u : inv_adjacent_vertices_range(lg.sink, lg)) {
        lit.clear();
        while (u != lg.root) {
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

    LitVertexQ workQ;
    initWorkQueue(workQ, lg, g, e);

    while (!workQ.empty()) {
        const LitVertex lv = workQ.front().first;
        const NFAVertex &t = workQ.front().second;
        const CharReach &cr = g[t].char_reach;

        u32 cr_card = crCardinality(cr);
        size_t numItems = cr_card * in_degree(t, g);
        size_t committed_count = workQ.size() + in_degree(lg.sink, lg) - 1;

        if (g[t].index == NODE_START) {
            // reached start, add to literal set
            add_edge_if_not_present(lv, lg.sink, lg);
            goto next_work_elem;
        }

        // Expand next vertex
        if (allowExpand(numItems, committed_count)) {
            for (auto u : inv_adjacent_vertices_range(t, g)) {
                addToQueue(workQ, lg, lv, cr, u);
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
                LitVertex lt = addToLitGraph(lg, lv, c);
                add_edge_if_not_present(lt, lg.sink, lg);
            }
            goto next_work_elem;
        }

        // add to literal set
        add_edge_if_not_present(lv, lg.sink, lg);
    next_work_elem:
        workQ.pop();
    }

    filterLitGraph(lg);
    //dumpGraph("litgraph.dot", lg);
    extractLiterals(lg, s);

    // Our literal set should contain no literal that is a suffix of another.
    assert(!hasSuffixLiterals(s));

    DEBUG_PRINTF("edge %zu (%zu->%zu) produced %zu literals\n", g[e].index,
                 g[source(e, g)].index, g[target(e, g)].index, s.size());
}

bool bad_mixed_sensitivity(const ue2_literal &s) {
    /* TODO: if the mixed cases is entirely within MAX_MASK2_WIDTH of the end,
     * we should be able to handle it */
    return mixed_sensitivity(s) && s.length() > MAX_MASK2_WIDTH;
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
void addReversedLiteral(const ue2_literal &lit, LitGraph &lg) {
    DEBUG_PRINTF("literal: '%s'\n", escapeString(lit).c_str());
    ue2_literal suffix;
    LitVertex v = lg.root;
    for (auto it = lit.rbegin(), ite = lit.rend(); it != ite; ++it) {
        suffix.push_back(*it);
        LitVertex w;
        for (auto v2 : adjacent_vertices_range(v, lg)) {
            if (v2 != lg.sink && lg[v2].c == *it) {
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
    add_edge(v, lg.sink, lg);
}

static
void extractLiterals(const vector<LitEdge> &cutset, const LitGraph &lg,
                     set<ue2_literal> &s) {
    for (const auto &e : cutset) {
        LitVertex u = source(e, lg);
        LitVertex v = target(e, lg);
        ue2_literal lit;
        lit.push_back(lg[v].c);
        while (u != lg.root) {
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
const char *describeColor(small_color c) {
    switch (c) {
    case small_color::white:
        return "white";
    case small_color::gray:
        return "gray";
    case small_color::black:
        return "black";
    default:
        return "unknown";
    }
}
#endif

/**
 * The BGL's boykov_kolmogorov_max_flow requires that all edges have their
 * reverse edge in the graph. This function adds them, returning a vector
 * mapping edge index to reverse edge. Note: LitGraph should be a DAG so there
 * should be no existing reverse_edges.
 */
static
vector<LitEdge> add_reverse_edges_and_index(LitGraph &lg) {
    const size_t edge_count = num_edges(lg);
    vector<LitEdge> fwd_edges;
    fwd_edges.reserve(edge_count);
    for (const auto &e : edges_range(lg)) {
        fwd_edges.push_back(e);
    }

    vector<LitEdge> rev_map(2 * edge_count);

    for (const auto &e : fwd_edges) {
        LitVertex u = source(e, lg);
        LitVertex v = target(e, lg);

        assert(!edge(v, u, lg).second);

        LitEdge rev = add_edge(v, u, LitGraphEdgeProps(0), lg).first;
        rev_map[lg[e].index] = rev;
        rev_map[lg[rev].index] = e;
    }

    return rev_map;
}

static
void findMinCut(LitGraph &lg, vector<LitEdge> &cutset) {
    cutset.clear();

    //dumpGraph("litgraph.dot", lg);

    assert(!in_degree(lg.root, lg));
    assert(!out_degree(lg.sink, lg));
    size_t num_real_edges = num_edges(lg);

    // Add reverse edges for the convenience of the BGL's max flow algorithm.
    vector<LitEdge> rev_edges = add_reverse_edges_and_index(lg);

    const auto v_index_map = get(&LitGraphVertexProps::index, lg);
    const auto e_index_map = get(&LitGraphEdgeProps::index, lg);
    const size_t num_verts = num_vertices(lg);
    auto colors = make_small_color_map(lg);
    vector<s32> distances(num_verts);
    vector<LitEdge> predecessors(num_verts);
    vector<u64a> residuals(num_edges(lg));

    UNUSED u64a flow = boykov_kolmogorov_max_flow(lg,
            get(&LitGraphEdgeProps::score, lg),
            make_iterator_property_map(residuals.begin(), e_index_map),
            make_iterator_property_map(rev_edges.begin(), e_index_map),
            make_iterator_property_map(predecessors.begin(), v_index_map),
            colors,
            make_iterator_property_map(distances.begin(), v_index_map),
            v_index_map, lg.root, lg.sink);
    DEBUG_PRINTF("done, flow = %llu\n", flow);

    /* remove reverse edges */
    remove_edge_if([&](const LitEdge &e) {
                       return lg[e].index >= num_real_edges;
                   }, lg);

    vector<LitEdge> white_cut, black_cut;
    u64a white_flow = 0, black_flow = 0;

    for (const auto &e : edges_range(lg)) {
        const LitVertex u = source(e, lg), v = target(e, lg);
        const auto ucolor = get(colors, u);
        const auto vcolor = get(colors, v);

        DEBUG_PRINTF("edge %zu:%s -> %zu:%s score %llu\n", lg[u].index,
                     describeColor(ucolor), lg[v].index, describeColor(vcolor),
                     lg[e].score);

        if (ucolor != small_color::white && vcolor == small_color::white) {
            assert(v != lg.sink);
            white_cut.push_back(e);
            white_flow += lg[e].score;
        }
        if (ucolor == small_color::black && vcolor != small_color::black) {
            assert(v != lg.sink);
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

    for (const auto &lit : s) {
        addReversedLiteral(lit, lg);
    }

    DEBUG_PRINTF("suffix tree has %zu vertices and %zu edges\n",
                  num_vertices(lg), num_edges(lg));

    vector<LitEdge> cutset;
    findMinCut(lg, cutset);

    s.clear();
    extractLiterals(cutset, lg, s);

    u64a score = scoreSet(s);
    DEBUG_PRINTF("compressed score is %llu\n", score);
    assert(score <= initialScore);
    return score;
}

/* like compressAndScore, but replaces long mixed sensitivity literals with
 * something weaker. */
u64a sanitizeAndCompressAndScore(set<ue2_literal> &lits) {
    const size_t maxExploded = 8; // only case-explode this far

    /* TODO: the whole compression thing could be made better by systematically
     * considering replacing literal sets not just by common suffixes but also
     * by nocase literals. */

    vector<ue2_literal> replacements;

    for (auto it = lits.begin(); it != lits.end();) {
        auto jt = it;
        ++it;

        if (!bad_mixed_sensitivity(*jt)) {
            continue;
        }

        /* we have to replace *jt with something... */
        ue2_literal s = *jt;
        lits.erase(jt);

        vector<ue2_literal> exploded;
        for (auto cit = caseIterateBegin(s); cit != caseIterateEnd(); ++cit) {
            exploded.emplace_back(*cit, false);
            if (exploded.size() > maxExploded) {
                goto dont_explode;
            }
        }
        insert(&replacements, replacements.end(), exploded);

        continue;
    dont_explode:
        make_nocase(&s);
        replacements.push_back(s);
    }

    insert(&lits, replacements);
    return compressAndScore(lits);
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

vector<u64a> scoreEdges(const NGHolder &g, const flat_set<NFAEdge> &known_bad) {
    assert(hasCorrectlyNumberedEdges(g));

    vector<u64a> scores(num_edges(g));

    for (const auto &e : edges_range(g)) {
        u32 eidx = g[e].index;
        assert(eidx < scores.size());
        if (contains(known_bad, e)) {
            scores[eidx] = NO_LITERAL_AT_EDGE_SCORE;
        } else {
            set<ue2_literal> ls = getLiteralSet(g, e);
            scores[eidx] = compressAndScore(ls);
        }
    }

    return scores;
}

bool splitOffLeadingLiteral(const NGHolder &g, ue2_literal *lit_out,
                            NGHolder *rhs) {
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

    NFAVertex u = g.startDs;
    NFAVertex v = *sds_succ.begin();

    while (true) {
        DEBUG_PRINTF("validating vertex %zu\n", g[v].index);

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

    unordered_map<NFAVertex, NFAVertex> rhs_map;
    vector<NFAVertex> pivots = make_vector_from(adjacent_vertices(u, g));
    splitRHS(g, pivots, rhs, &rhs_map);

    DEBUG_PRINTF("literal is '%s' (len %zu)\n", dumpString(*lit_out).c_str(),
                 lit_out->length());
    assert(is_triggered(*rhs));
    return true;
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

bool literalIsWholeGraph(const NGHolder &g, const ue2_literal &lit) {
    NFAVertex v = g.accept;

    for (auto it = lit.rbegin(), ite = lit.rend(); it != ite; ++it) {
        NGHolder::inv_adjacency_iterator ai, ae;
        tie(ai, ae) = inv_adjacent_vertices(v, g);
        if (ai == ae) {
            assert(0); // no predecessors?
            return false;
        }
        v = *ai++;
        if (ai != ae) {
            DEBUG_PRINTF("branch, fail\n");
            return false;
        }

        if (is_special(v, g)) {
            DEBUG_PRINTF("special found, fail\n");
            return false;
        }

        const CharReach &cr_g = g[v].char_reach;
        const CharReach &cr_l = *it;

        if (!cr_l.isSubsetOf(cr_g)) {
            /* running over the prefix is needed to prevent false postives */
            DEBUG_PRINTF("reach fail\n");
            return false;
        }
    }

    // Our last value for v should have only start states for predecessors.
    for (auto u : inv_adjacent_vertices_range(v, g)) {
        if (!is_any_start(u, g)) {
            DEBUG_PRINTF("pred is not start\n");
            return false;
        }
    }

    assert(num_vertices(g) == lit.length() + N_SPECIALS);

    DEBUG_PRINTF("ok\n");
    return true;
}

} // namespace ue2
