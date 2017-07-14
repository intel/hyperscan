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
 * \brief Loose equality testing for NGHolder graphs.
 *
 * Loose equality check for holders' graph structure and vertex_index,
 * vertex_char_reach and (optionally reports).
 */
#include "ng_is_equal.h"

#include "grey.h"
#include "ng_holder.h"
#include "ng_util.h"
#include "ue2common.h"
#include "util/container.h"
#include "util/flat_containers.h"
#include "util/graph_range.h"
#include "util/make_unique.h"

using namespace std;

namespace ue2 {

namespace {
struct check_report {
    virtual ~check_report() {}
    virtual bool operator()(const flat_set<ReportID> &reports_a,
                            const flat_set<ReportID> &reports_b) const = 0;
};

struct full_check_report : public check_report {
    bool operator()(const flat_set<ReportID> &reports_a,
                    const flat_set<ReportID> &reports_b) const override {
        return reports_a == reports_b;
    }
};

struct equiv_check_report : public check_report {
    equiv_check_report(ReportID a_in, ReportID b_in)
        : a_rep(a_in), b_rep(b_in) {}

    bool operator()(const flat_set<ReportID> &reports_a,
                    const flat_set<ReportID> &reports_b) const override {
        return contains(reports_a, a_rep) == contains(reports_b, b_rep);
    }
private:
    ReportID a_rep;
    ReportID b_rep;
};

/** Comparison functor used to sort by vertex_index. */
template<typename Graph>
struct VertexIndexOrdering {
    explicit VertexIndexOrdering(const Graph &g_in) : g(g_in) {}
    bool operator()(typename Graph::vertex_descriptor a,
                    typename Graph::vertex_descriptor b) const {
        assert(a == b || g[a].index != g[b].index);
        return g[a].index < g[b].index;
    }
private:
    const Graph &g;
};

template<typename Graph>
static
VertexIndexOrdering<Graph> make_index_ordering(const Graph &g) {
    return VertexIndexOrdering<Graph>(g);
}

}

static
bool is_equal_i(const NGHolder &a, const NGHolder &b,
                const check_report &check_rep) {
    assert(hasCorrectlyNumberedVertices(a));
    assert(hasCorrectlyNumberedVertices(b));

    size_t num_verts = num_vertices(a);
    if (num_verts != num_vertices(b)) {
        return false;
    }

    vector<NFAVertex> vert_a;
    vector<NFAVertex> vert_b;
    vector<NFAVertex> adj_a;
    vector<NFAVertex> adj_b;

    vert_a.reserve(num_verts);
    vert_b.reserve(num_verts);
    adj_a.reserve(num_verts);
    adj_b.reserve(num_verts);

    insert(&vert_a, vert_a.end(), vertices(a));
    insert(&vert_b, vert_b.end(), vertices(b));

    sort(vert_a.begin(), vert_a.end(), make_index_ordering(a));
    sort(vert_b.begin(), vert_b.end(), make_index_ordering(b));

    for (size_t i = 0; i < vert_a.size(); i++) {
        NFAVertex va = vert_a[i];
        NFAVertex vb = vert_b[i];
        DEBUG_PRINTF("vertex %zu\n", a[va].index);

        // Vertex index must be the same.
        if (a[va].index != b[vb].index) {
            DEBUG_PRINTF("bad index\n");
            return false;
        }

        // Reach must be the same.
        if (a[va].char_reach != b[vb].char_reach) {
            DEBUG_PRINTF("bad reach\n");
            return false;
        }

        if (!check_rep(a[va].reports, b[vb].reports)) {
            DEBUG_PRINTF("bad reports\n");
            return false;
        }

        // Other vertex properties may vary.

        /* Check successors */
        adj_a.clear();
        adj_b.clear();
        insert(&adj_a, adj_a.end(), adjacent_vertices(va, a));
        insert(&adj_b, adj_b.end(), adjacent_vertices(vb, b));

        if (adj_a.size() != adj_b.size()) {
            DEBUG_PRINTF("bad adj\n");
            return false;
        }

        sort(adj_a.begin(), adj_a.end(), make_index_ordering(a));
        sort(adj_b.begin(), adj_b.end(), make_index_ordering(b));

        for (size_t j = 0; j < adj_a.size(); j++) {
            if (a[adj_a[j]].index != b[adj_b[j]].index) {
                DEBUG_PRINTF("bad adj\n");
                return false;
            }
        }
    }

    /* check top for edges out of start */
    vector<pair<u32, flat_set<u32>>> top_a;
    vector<pair<u32, flat_set<u32>>> top_b;

    for (const auto &e : out_edges_range(a.start, a)) {
        top_a.emplace_back(a[target(e, a)].index, a[e].tops);
    }
    for (const auto &e : out_edges_range(b.start, b)) {
        top_b.emplace_back(b[target(e, b)].index, b[e].tops);
    }

    sort(top_a.begin(), top_a.end());
    sort(top_b.begin(), top_b.end());

    if (top_a != top_b) {
        DEBUG_PRINTF("bad top\n");
        return false;
    }

    DEBUG_PRINTF("good\n");
    return true;
}

/** \brief loose hash of an NGHolder; equal if is_equal would return true. */
u64a hash_holder(const NGHolder &g) {
    size_t rv = 0;

    for (auto v : vertices_range(g)) {
        hash_combine(rv, g[v].index);
        hash_combine(rv, g[v].char_reach);

        for (auto w : adjacent_vertices_range(v, g)) {
            hash_combine(rv, g[w].index);
        }
    }

    return rv;
}

bool is_equal(const NGHolder &a, const NGHolder &b) {
    DEBUG_PRINTF("testing %p %p\n", &a, &b);

    if (&a == &b) {
        return true;
    }

    return is_equal_i(a, b, full_check_report());
}

bool is_equal(const NGHolder &a, ReportID a_rep,
              const NGHolder &b, ReportID b_rep) {
    DEBUG_PRINTF("testing %p %p\n", &a, &b);

    if (&a == &b && a_rep == b_rep) {
        return true;
    }

    return is_equal_i(a, b, equiv_check_report(a_rep, b_rep));
}

} // namespace ue2
