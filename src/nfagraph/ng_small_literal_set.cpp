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
 * \brief Rose construction from NGHolder for cases representing small literal
 * sets.
 */
#include "ng_small_literal_set.h"

#include "grey.h"
#include "ng_holder.h"
#include "ng_util.h"
#include "rose/rose_build.h"
#include "util/compare.h"
#include "util/compile_context.h"
#include "util/container.h"
#include "util/graph_range.h"
#include "util/order_check.h"
#include "util/ue2string.h"
#include "ue2common.h"

#include <map>
#include <set>
#include <vector>
#include <boost/range/adaptor/map.hpp>

using namespace std;
using boost::adaptors::map_keys;

namespace ue2 {

/** \brief The maximum number of literals to accept per pattern. */
static const size_t MAX_LITERAL_SET_SIZE = 30;

/**
 * \brief The maximum number of literals to accept per pattern where at least
 * one is weak (has period < MIN_STRONG_PERIOD).
 */
static const size_t MAX_WEAK_LITERAL_SET_SIZE = 20;

/**
 * \brief The minimum string period to consider a literal "strong" (and not
 * apply the weak size limit).
 */
static const size_t MIN_STRONG_PERIOD = 3;

namespace {

struct sls_literal {
    bool anchored;
    bool eod;
    ue2_literal s;

    explicit sls_literal(bool a) : anchored(a), eod(false) {}

    sls_literal append(char c, bool nocase) const {
        sls_literal rv(anchored);
        rv.s = s;
        rv.s.push_back(ue2_literal::elem(c, nocase));

        return rv;
    }
};

static
bool operator<(const sls_literal &a, const sls_literal &b) {
    ORDER_CHECK(anchored);
    ORDER_CHECK(eod);
    ORDER_CHECK(s);

    return false;
}

} // namespace

static
bool checkLongMixedSensitivityLiterals(
        const map<sls_literal, flat_set<ReportID>> &literals) {
    const size_t len = MAX_MASK2_WIDTH;

    for (const sls_literal &lit : literals | map_keys) {
        if (mixed_sensitivity(lit.s) && lit.s.length() > len) {
            return false;
        }
    }

    return true;
}

static
bool findLiterals(const NGHolder &g,
                  map<sls_literal, flat_set<ReportID>> *literals) {
    vector<NFAVertex> order = getTopoOrdering(g);

    vector<set<sls_literal>> built(num_vertices(g));
    vector<size_t> read_count(num_vertices(g));

    for (auto it = order.rbegin(); it != order.rend(); ++it) {
        NFAVertex v = *it;
        set<sls_literal> &out = built[g[v].index];
        read_count[g[v].index] = out_degree(v, g);

        DEBUG_PRINTF("setting read_count to %zu for %zu\n",
                      read_count[g[v].index], g[v].index);

        assert(out.empty());
        if (v == g.start) {
            out.insert(sls_literal(true));
            continue;
        } else if (v == g.startDs) {
            out.insert(sls_literal(false));
            continue;
        }

        bool eod = v == g.acceptEod;
        bool accept = v == g.accept || v == g.acceptEod;
        const CharReach &cr = g[v].char_reach;

        for (auto u : inv_adjacent_vertices_range(v, g)) {
            if (u == g.accept) {
                continue;
            }

            if (u == g.start && edge(g.startDs, v, g).second) {
                /* floating start states may have connections to start and
                 * startDs - don't create duplicate anchored literals */
                DEBUG_PRINTF("skipping as floating\n");
                continue;
            }

            set<sls_literal> &in = built[g[u].index];
            DEBUG_PRINTF("getting from %zu (%zu reads to go)\n",
                          g[u].index, read_count[g[u].index]);
            assert(!in.empty());
            assert(read_count[g[u].index]);

            for (const sls_literal &lit : in) {
                if (accept) {
                    sls_literal accept_lit = lit; // copy
                    accept_lit.eod = eod;
                    insert(&(*literals)[accept_lit], g[u].reports);
                    continue;
                }

                for (size_t c = cr.find_first(); c != cr.npos;
                     c = cr.find_next(c)) {
                    bool nocase = ourisalpha(c) && cr.test(mytoupper(c))
                        && cr.test(mytolower(c));

                    if (nocase && (char)c == mytolower(c)) {
                        continue; /* uppercase already handled us */
                    }

                    out.insert(lit.append((u8)c, nocase));

                    if (out.size() + literals->size() > MAX_LITERAL_SET_SIZE) {
                        DEBUG_PRINTF("too big %zu + %zu\n", out.size(),
                                      literals->size());
                        return false;
                    }
                }
            }

            read_count[g[u].index]--;
            if (!read_count[g[u].index]) {
                DEBUG_PRINTF("clearing %zu as finished reading\n", g[u].index);
                in.clear();
            }
        }
    }

    return true;
}

static
size_t min_period(const map<sls_literal, flat_set<ReportID>> &literals) {
    size_t rv = SIZE_MAX;

    for (const sls_literal &lit : literals | map_keys) {
        rv = min(rv, minStringPeriod(lit.s));
    }
    DEBUG_PRINTF("min period %zu\n", rv);
    return rv;
}

// If this component is just a small set of literals and can be handled by
// Rose, feed it directly into rose.
bool handleSmallLiteralSets(RoseBuild &rose, const NGHolder &g,
                            const CompileContext &cc) {
    if (!cc.grey.allowSmallLiteralSet) {
        return false;
    }

    if (!isAcyclic(g)) {
        /* literal sets would typically be acyclic... */
        DEBUG_PRINTF("not acyclic\n");
        return false;
    }

    if (!hasNarrowReachVertex(g, MAX_LITERAL_SET_SIZE * 2 + 1)) {
        DEBUG_PRINTF("vertex with wide reach found\n");
        return false;
    }

    DEBUG_PRINTF("looking for literals\n");

    map<sls_literal, flat_set<ReportID>> literals;
    if (!findLiterals(g, &literals)) {
        DEBUG_PRINTF(":(\n");
        return false;
    }

    assert(!literals.empty());

    if (literals.size() > MAX_LITERAL_SET_SIZE) {
        /* try a mask instead */
        DEBUG_PRINTF("too many literals\n");
        return false;
    }

    size_t period = min_period(literals);
    if (period < MIN_STRONG_PERIOD &&
        literals.size() > MAX_WEAK_LITERAL_SET_SIZE) {
        DEBUG_PRINTF("too many literals with weak period\n");
        return false;
    }

    if (!checkLongMixedSensitivityLiterals(literals)) {
        DEBUG_PRINTF("long mixed\n");
        return false;
    }

    DEBUG_PRINTF("adding %zu literals\n", literals.size());
    for (const auto &m : literals) {
        const sls_literal &lit = m.first;
        const auto &reports = m.second;
        rose.add(lit.anchored, lit.eod, lit.s, reports);
    }

    return true;
}

} // namespace ue2
