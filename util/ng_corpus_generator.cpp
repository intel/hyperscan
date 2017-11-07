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
 * \brief Corpus Generation tool.
 */

#include "config.h"

#include "ng_corpus_generator.h"

#include "ng_corpus_editor.h"
#include "compiler/compiler.h"
#include "nfagraph/ng.h"
#include "nfagraph/ng_util.h"
#include "ue2common.h"
#include "util/container.h"
#include "util/graph_range.h"
#include "util/make_unique.h"
#include "util/ue2string.h"
#include "util/unicode_def.h"
#include "util/unicode_set.h"

#include <algorithm>
#include <deque>
#include <memory>
#include <set>
#include <sstream>
#include <unordered_set>
#include <vector>

#include <boost/utility.hpp>

using namespace std;
using namespace ue2;

typedef vector<NFAVertex> VertexPath;

#if defined(DEBUG)
// For debugging output
static
string pathToString(const NGHolder &g, const VertexPath &p) {
    ostringstream oss;
    oss << '[';
    for (auto i = p.begin(); i != p.end(); ++i) {
        if (i != p.begin()) {
            oss << ',';
        }
        oss << g[*i].index;
    }
    oss << ']';
    return oss.str();
}
#endif

/** True if this graph has no non-special successors of start or startDs. */
static
bool graph_is_empty(const NGHolder &g) {
    for (const auto &v : adjacent_vertices_range(g.start, g)) {
        if (!is_special(v, g)) {
            return false;
        }
    }
    for (const auto &v : adjacent_vertices_range(g.start, g)) {
        if (!is_special(v, g)) {
            return false;
        }
    }

    return true;
}

static
string encodeUtf8(const vector<unichar> &v) {
    string rv;
    for (const unichar &cp : v) {
        if (cp < UTF_2CHAR_MIN) {
            rv.push_back(cp);
        } else if (cp < UTF_3CHAR_MIN) {
            rv.push_back(UTF_TWO_BYTE_HEADER | (cp >> UTF_CONT_SHIFT));
            rv.push_back(makeContByte(cp));
        } else if (cp < UTF_4CHAR_MIN) {
            rv.push_back(UTF_THREE_BYTE_HEADER | (cp >> (2 * UTF_CONT_SHIFT)));
            rv.push_back(makeContByte(cp >> UTF_CONT_SHIFT));
            rv.push_back(makeContByte(cp));
        } else {
            rv.push_back(UTF_FOUR_BYTE_HEADER | (cp >> (3 * UTF_CONT_SHIFT)));
            rv.push_back(makeContByte(cp >> (2 * UTF_CONT_SHIFT)));
            rv.push_back(makeContByte(cp >> UTF_CONT_SHIFT));
            rv.push_back(makeContByte(cp));
        }
    }
    return rv;
}

template<class Iter, class Val>
static
bool has_greater_than(Iter it, Iter end, const Val &v, size_t limit) {
    for (; it != end; ++it) {
        if (*it == v) {
            if (limit == 0) {
                return true;
            }
            --limit;
        }
    }
    return false;
}

static
void findPaths(const NGHolder &g, CorpusProperties &cProps,
               vector<VertexPath> &allPaths, size_t cycleLimit,
               size_t corpusLimit) {
    // The maximum number of open (in progress) paths. New paths beyond this
    // limit will evict a random existing one.
    const size_t MAX_OPEN = min((size_t)1000, corpusLimit * 10);

    vector<unique_ptr<VertexPath>> open;
    open.push_back(ue2::make_unique<VertexPath>(1, g.start));

    unordered_set<NFAVertex> one_way_in;
    for (const auto &v : vertices_range(g)) {
        if (in_degree(v, g) <= 1) {
            one_way_in.insert(v);
        }
    }

    while (!open.empty()) {
        u32 slot = cProps.rand(0, open.size() - 1);
        swap(open.at(slot), open.back());
        auto p = std::move(open.back());
        open.pop_back();
        NFAVertex u = p->back();

        DEBUG_PRINTF("dequeuing path %s, back %zu\n",
                     pathToString(g, *p).c_str(), g[u].index);

        NGHolder::adjacency_iterator ai, ae;
        for (tie(ai, ae) = adjacent_vertices(u, g); ai != ae; ++ai) {
            NFAVertex v = *ai;

            if (u == g.startDs && v == g.startDs) {
                // explicitly avoid following startDs self-loop, as we have
                // other mechanisms for adding prefixes to our corpora.
                continue;
            }

            // Accept vertices generate completed paths.
            if (v == g.accept || v == g.acceptEod) {
                DEBUG_PRINTF("path complete: %s\n",
                             pathToString(g, *p).c_str());
                allPaths.push_back(*p);
                if (allPaths.size() >= corpusLimit) {
                    DEBUG_PRINTF("full, going home\n");
                    return;
                }

                // No meaningful edges out of accept or acceptEod.
                continue;
            }

            if (!contains(one_way_in, v) &&
                has_greater_than(p->begin(), p->end(), v, cycleLimit)) {
                // Note that vertices that only have one predecessor don't need
                // their cycle limit checked, as their predecessors will have
                // the same count.
                DEBUG_PRINTF("exceeded cycle limit for v=%zu, pruning path\n",
                             g[v].index);
                continue;
            }

            // If we've got no further adjacent vertices, re-use p rather than
            // copying it for the next path.
            unique_ptr<VertexPath> new_path;
            if (boost::next(ai) == ae) {
                new_path = std::move(p);
            } else {
                new_path = make_unique<VertexPath>(*p);
            }

            new_path->push_back(v);
            if (open.size() < MAX_OPEN) {
                open.push_back(std::move(new_path));
            } else {
                u32 victim = cProps.rand(0, open.size() - 1);
                open[victim] = std::move(new_path);
            }
        }
    }
    DEBUG_PRINTF("bored, going home\n");
}

namespace {

/** \brief Concrete implementation */
class CorpusGeneratorImpl : public CorpusGenerator {
public:
    CorpusGeneratorImpl(const NGHolder &graph_in, const ExpressionInfo &expr_in,
                        CorpusProperties &props);
    ~CorpusGeneratorImpl() = default;

    void generateCorpus(vector<string> &data);

private:
    unsigned char getRandomChar();
    unsigned char getMatchChar(const CharReach &cr);
    unsigned char getUnmatchChar(const CharReach &cr);

    unsigned char getChar(NFAVertex v);
    void newGenerator(vector<string> &data);
    string pathToCorpus(const VertexPath &path);

    /** \brief Generate a string of random bytes between minLen and maxLen
     * bytes in length. */
    void addRandom(const min_max &mm, string *out);

    /** \brief Info about this expression. */
    const ExpressionInfo &expr;

    /** \brief The NFA graph we operate over. */
    const NGHolder &graph;

    /** \brief Reference to our corpus generator properties object (stores some
     * state) */
    CorpusProperties &cProps;
};

CorpusGeneratorImpl::CorpusGeneratorImpl(const NGHolder &graph_in,
                                         const ExpressionInfo &expr_in,
                                         CorpusProperties &props)
    : expr(expr_in), graph(graph_in), cProps(props) {
    // if this pattern is to be matched approximately
    if ((expr.edit_distance || expr.hamm_distance) && !props.editDistance) {
        props.editDistance =
            props.rand(0, expr.hamm_distance + expr.edit_distance + 1);
    }
}

void CorpusGeneratorImpl::generateCorpus(vector<string> &data) {
    newGenerator(data);

    if (cProps.editDistance && !data.empty() &&
        data.size() < cProps.corpusLimit) {
        // Create more entries by copying the corpora and applying edits
        size_t diff = cProps.corpusLimit - data.size();
        size_t repeats = diff / data.size();
        size_t remains = diff % data.size();
        vector<string> newdata;
        for (size_t i = 0; i < repeats; i++) {
            std::copy(data.begin(), data.end(), std::back_inserter(newdata));
        }
        if (remains) {
            std::copy_n(data.begin(), remains, std::back_inserter(newdata));
        }
        for (auto &s : newdata) {
            editCorpus(&s, cProps);
        }
        std::move(newdata.begin(), newdata.end(), back_inserter(data));
    } else if (cProps.editDistance) {
        // If the caller has asked us, apply edit distance to corpora
        for (auto &s : data) {
            editCorpus(&s, cProps);
        }
    }
}

/** \brief Generate a random character, taking care to stick to the alphabet
 * that we've been asked for. */
u8 CorpusGeneratorImpl::getRandomChar() {
    return 'a' + cProps.rand(0, min(cProps.alphabetSize, (u32)CharReach::npos));
}

/** \brief Select a random character from the given string of valid match
 * characters. */
unsigned char CorpusGeneratorImpl::getMatchChar(const CharReach &cr) {
    unsigned int num = cr.count();
    if (num == 0) {
        return 0;
    } else if (num == 1) {
        return (unsigned char)cr.find_first();
    } else if (num == 256) {
        // Dot class, any character is OK!
        return (unsigned char)cProps.rand(0, 255);
    }
    else {
        unsigned idx = cProps.rand(0, num - 1);
        return (unsigned char)cr.find_nth(idx);
    }
}

/** \brief Select a character that does not belong to the given bitset. This
 * makes no guarantees on unmatchability if the bitset is full. */
unsigned char CorpusGeneratorImpl::getUnmatchChar(const CharReach &cr) {
    return getMatchChar(~cr);
}

void CorpusGeneratorImpl::addRandom(const min_max &mm, string *out) {
    assert(mm.min <= mm.max);
    u32 range = mm.max - mm.min;
    u32 len = mm.min + (range ? cProps.rand(0, range - 1) : 0);
    for (u32 i = 0; i < len; ++i) {
        out->push_back(getRandomChar());
    }
}

unsigned char CorpusGeneratorImpl::getChar(NFAVertex v) {
    const CharReach &cr = graph[v].char_reach;

    switch (cProps.throwDice()) {
    case CorpusProperties::ROLLED_MATCH:
        return getMatchChar(cr);
    case CorpusProperties::ROLLED_UNMATCH:
        return getUnmatchChar(cr);
    case CorpusProperties::ROLLED_RANDOM: /* character pulled from hat */
        return getRandomChar();
    }
    assert(0);
    return 0;
}

/** \brief Convert a path through the graph to a corpus string. */
string CorpusGeneratorImpl::pathToCorpus(const VertexPath &path) {
    string s;

    // Add random prefix
    if (cProps.prefixRange.max) {
        addRandom(cProps.prefixRange, &s);
    }

    // Generate a corpus from our path
    for (const auto &e : path) {
        if (!is_special(e, graph)) {
            s += getChar(e);
        }
    }

    // Add random suffix
    if (cProps.suffixRange.max) {
        addRandom(cProps.suffixRange, &s);
    }

    return s;
}

void CorpusGeneratorImpl::newGenerator(vector<string> &outdata) {
    const unsigned int maxCycles = cProps.getCycleLimit().second;
    DEBUG_PRINTF("generating up to %u corpora, cycle limit of %u\n",
                 cProps.corpusLimit, maxCycles);

    vector<VertexPath> allPaths;

    // Special case: if the graph has ONLY special vertices, then this is
    // likely to be an odd vacuous pattern or a pattern that can never match.
    // In these cases, an empty corpus is useful.
    if (graph_is_empty(graph)) {
        VertexPath empty(1, graph.start);
        allPaths.push_back(empty);
    }

    // build a set of unique paths
    findPaths(graph, cProps, allPaths, maxCycles, cProps.corpusLimit);

    // transform paths into corpora: we do this repeatedly until we (a) hit our
    // limit, or (b) don't generate any new corpora for any of our paths.
    set<string> data;
    while (data.size() < cProps.corpusLimit) {
        size_t count = data.size();
        for (const auto &path : allPaths) {
            string s = pathToCorpus(path);
            if (data.insert(s).second) {
                DEBUG_PRINTF("corpus %zu (%zu bytes): '%s'\n", data.size(),
                             s.size(), escapeString(s).c_str());
                if (data.size() == cProps.corpusLimit) {
                    goto hit_limit;
                }
            }
        }
        if (data.size() == count) {
            break; // we're finding it hard to generate more corpora
        }
    }

hit_limit:
    DEBUG_PRINTF("%zu corpora built\n", data.size());

    // populate the output vector from the set we built.
    outdata.reserve(data.size());
    copy(data.begin(), data.end(), back_inserter(outdata));
}

/** \brief Concrete implementation for UTF-8 */
class CorpusGeneratorUtf8 : public CorpusGenerator {
public:
    CorpusGeneratorUtf8(const NGHolder &graph_in, const ExpressionInfo &expr_in,
                        CorpusProperties &props);
    ~CorpusGeneratorUtf8() = default;

    void generateCorpus(vector<string> &data);

private:
    unichar getRandomChar();
    unichar getMatchChar(CodePointSet cps);
    unichar getUnmatchChar(const CodePointSet &cps);

    unichar getChar(const CodePointSet &cps);
    void newGenerator(vector<vector<unichar> > &data);
    vector<unichar> pathToCorpus(const vector<CodePointSet> &path);

    /** \brief Generate a random string between min and max codepoints in
     * length. */
    void addRandom(const min_max &mm, vector<unichar> *out);

    /** \brief Info about this expression. */
    const ExpressionInfo &expr;

    /** \brief The NFA graph we operate over. */
    const NGHolder &graph;

    /** \brief Reference to our corpus generator properties object (stores some
     * state) */
    CorpusProperties &cProps;
};

CorpusGeneratorUtf8::CorpusGeneratorUtf8(const NGHolder &graph_in,
                                         const ExpressionInfo &expr_in,
                                         CorpusProperties &props)
    : expr(expr_in), graph(graph_in), cProps(props) {
    // we do not support Utf8 for approximate matching
    if (expr.edit_distance) {
        throw CorpusGenerationFailure("UTF-8 for edited patterns is not "
                                      "supported.");
    }
}

void CorpusGeneratorUtf8::generateCorpus(vector<string> &data) {
    vector<vector<unichar>> raw;
    newGenerator(raw);

    // If the caller has asked us, apply edit distance to corpora
    if (cProps.editDistance) {
        for (auto &e : raw) {
            editCorpus(&e, cProps);
        }
    }

    for (const auto &e : raw) {
        data.push_back(encodeUtf8(e));
    }
}

/** \brief Generate a random character, taking care to stick to the alphabet
 * that we've been asked for. */
unichar CorpusGeneratorUtf8::getRandomChar() {
    u32 range = MAX_UNICODE + 1
                - (UNICODE_SURROGATE_MAX + UNICODE_SURROGATE_MIN + 1);
    range = min(cProps.alphabetSize, range);
    assert(range);

    unichar c = 'a' + cProps.rand(0, range - 1);

    if (c >= UNICODE_SURROGATE_MIN) {
        c =+ UNICODE_SURROGATE_MAX + 1;
    }

    return c % (MAX_UNICODE + 1);
}

/** \brief Select a random character from the given string of valid match
 * characters. */
unichar CorpusGeneratorUtf8::getMatchChar(CodePointSet cps) {
    cps.unsetRange(UNICODE_SURROGATE_MIN, UNICODE_SURROGATE_MAX);
    u32 num = cps.count();
    if (num == 0) {
        return 0;
    } else if (num == 1) {
        return lower(*cps.begin());
    } else {
        unichar rv = cps.at(cProps.rand(0, num - 1));
        assert(rv != INVALID_UNICODE);
        return rv;
    }
}

/** \brief Select a character that does not belong to the given bitset. This
 * makes no guarantees on unmatchability if the bitset is full. */
unichar CorpusGeneratorUtf8::getUnmatchChar(const CodePointSet &cps) {
    return getMatchChar(~cps);
}

void CorpusGeneratorUtf8::addRandom(const min_max &mm, vector<unichar> *out) {
    assert(mm.min <= mm.max);
    u32 range = mm.max - mm.min;
    u32 len = mm.min + (range ? cProps.rand(0, range - 1) : 0);
    for (u32 i = 0; i < len; ++i) {
        out->push_back(getRandomChar());
    }
}

unichar CorpusGeneratorUtf8::getChar(const CodePointSet &cps) {
    switch (cProps.throwDice()) {
    case CorpusProperties::ROLLED_MATCH:
        return getMatchChar(cps);
    case CorpusProperties::ROLLED_UNMATCH:
        return getUnmatchChar(cps);
    case CorpusProperties::ROLLED_RANDOM: /* character pulled from hat */
        return getRandomChar();
    }
    assert(0);
    return 0;
}

/** \brief Convert a path through the graph to a corpus string. */
vector<unichar>
CorpusGeneratorUtf8::pathToCorpus(const vector<CodePointSet> &path) {
    vector<unichar> s;

    // Add random prefix
    if (cProps.prefixRange.max) {
        addRandom(cProps.prefixRange, &s);
    }

    // Generate a corpus from our path
    for (const auto &e : path) {
        s.push_back(getChar(e));
    }

    // Add random suffix
    if (cProps.suffixRange.max) {
        addRandom(cProps.suffixRange, &s);
    }

    return s;
}

static
u32 classify_vertex(const NGHolder &g, NFAVertex v) {
    const CharReach &cr = g[v].char_reach;
    if (cr.isSubsetOf(UTF_ASCII_CR)) {
        return 1;
    } else if (cr.isSubsetOf(UTF_TWO_START_CR)) {
        return 2;
    } else if (cr.isSubsetOf(UTF_THREE_START_CR)) {
        return 3;
    } else if (cr.isSubsetOf(UTF_FOUR_START_CR)) {
        return 4;
    }

    /* this can happen due to dummy vertices from zwa */
    return 1;
}

static
void fillCodePointSet(const CharReach &cr, CodePointSet *out, u8 mask = 0xff) {
    for (u32 i = cr.find_first(); i != CharReach::npos; i = cr.find_next(i)) {
        out->set(i & mask);
    }
}

static
void expandCodePointSet(const CharReach &cr, CodePointSet *out, u32 mask,
                        u32 n) {
    CodePointSet base;
    base.swap(*out);
    for (u32 i = cr.find_first(); i != CharReach::npos; i = cr.find_next(i)) {
        u32 val = (i & mask) << (n * UTF_CONT_SHIFT);
        for (const auto &cp : base) {
            unichar ll = lower(cp);
            unichar uu = upper(cp);
            out->setRange(val + ll, MIN(val + uu, MAX_UNICODE));
        }
    }
}

static
void decodePath(const NGHolder &g, const VertexPath &in,
                vector<CodePointSet> &out) {
    VertexPath::const_iterator it = in.begin();
    while (it != in.end()) {
        if (is_special(*it, g)) {
            ++it;
            continue;
        }

        out.push_back(CodePointSet());
        CodePointSet &cps = out.back();

        switch (classify_vertex(g, *it)) {
        case 1:
            fillCodePointSet(g[*it].char_reach, &cps);
            ++it;
            break;
        case 2:
            fillCodePointSet(g[*(it + 1)].char_reach, &cps,
                             UTF_CONT_BYTE_VALUE_MASK);
            expandCodePointSet(g[*it].char_reach, &cps,
                               ~UTF_TWO_BYTE_HEADER, 1);
            it += 2;
            break;
        case 3:
            fillCodePointSet(g[*(it + 2)].char_reach, &cps,
                             UTF_CONT_BYTE_VALUE_MASK);
            expandCodePointSet(g[*(it + 1)].char_reach, &cps,
                               UTF_CONT_BYTE_VALUE_MASK, 1);
            expandCodePointSet(g[*it].char_reach, &cps,
                               ~UTF_THREE_BYTE_HEADER, 2);
            it += 3;
            break;
        case 4:
            fillCodePointSet(g[*(it + 3)].char_reach, &cps,
                             UTF_CONT_BYTE_VALUE_MASK);
            expandCodePointSet(g[*(it + 2)].char_reach, &cps,
                               UTF_CONT_BYTE_VALUE_MASK, 1);
            expandCodePointSet(g[*(it + 1)].char_reach, &cps,
                               UTF_CONT_BYTE_VALUE_MASK, 2);
            expandCodePointSet(g[*it].char_reach, &cps,
                               ~UTF_FOUR_BYTE_HEADER, 3);
            it += 4;
            break;
        default:;
            assert(0);
            ++it;
        }
    }
}

static
void translatePaths(const NGHolder &graph,
                    const vector<VertexPath> &allPathsTemp,
                    vector<vector<CodePointSet>> *out) {
    assert(out);
    for (const auto &path : allPathsTemp) {
        out->push_back(vector<CodePointSet>());
        decodePath(graph, path, out->back());
    }
}

void CorpusGeneratorUtf8::newGenerator(vector<vector<unichar>> &outdata) {
    const u32 maxCycles = cProps.getCycleLimit().second;
    DEBUG_PRINTF("generating up to %u corpora, cycle limit of %u\n",
                 cProps.corpusLimit, maxCycles);

    vector<vector<CodePointSet>> allPaths;

    // Special case: if the graph has ONLY special vertices, then this is
    // likely to be an odd vacuous pattern or a pattern that can never match.
    // In these cases, an empty corpus is useful.
    if (graph_is_empty(graph)) {
        allPaths.push_back(vector<CodePointSet>());
    } else {
        // build a set of unique paths
        vector<VertexPath> allPathsTemp;
        findPaths(graph, cProps, allPathsTemp, maxCycles, cProps.corpusLimit);
        translatePaths(graph, allPathsTemp, &allPaths);
    }

    // transform paths into corpora: we do this repeatedly until we (a) hit our
    // limit, or (b) don't generate any new corpora for any of our paths.
    set<vector<unichar> > data;
    while (data.size() < cProps.corpusLimit) {
        size_t count = data.size();
        for (const auto &path : allPaths) {
            vector<unichar> vu = pathToCorpus(path);
            if (data.insert(vu).second) {
                if (data.size() == cProps.corpusLimit) {
                    goto hit_limit;
                }
            }
        }
        if (data.size() == count) {
            break; // we're finding it hard to generate more corpora
        }
    }

hit_limit:
    DEBUG_PRINTF("%zu corpora built\n", data.size());

    // populate the output vector from the set we built.
    outdata.reserve(data.size());
    copy(data.begin(), data.end(), back_inserter(outdata));
}

} // namespace

CorpusGenerator::~CorpusGenerator() { }

// External entry point

unique_ptr<CorpusGenerator> makeCorpusGenerator(const NGHolder &graph,
                                                const ExpressionInfo &expr,
                                                CorpusProperties &props) {
    if (expr.utf8) {
        return ue2::make_unique<CorpusGeneratorUtf8>(graph, expr, props);
    } else {
        return ue2::make_unique<CorpusGeneratorImpl>(graph, expr, props);
    }
}
