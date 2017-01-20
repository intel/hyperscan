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
 * \brief Execute an NFA over a given input, returning the set of states that
 * are active afterwards.
 *
 * Note: although our external interfaces for execute_graph() use std::set, we
 * use a dynamic bitset containing the vertex indices internally for
 * performance.
 */
#include "ng_execute.h"

#include "ng_holder.h"
#include "ng_util.h"
#include "ue2common.h"
#include "util/container.h"
#include "util/dump_charclass.h"
#include "util/graph_range.h"
#include "util/ue2string.h"

#include <sstream>
#include <string>

#include <boost/dynamic_bitset.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/reverse_graph.hpp>

using namespace std;
using boost::dynamic_bitset;

namespace ue2 {

struct StateInfo {
    StateInfo(NFAVertex v, const CharReach &cr) : vertex(v), reach(cr) {}
    StateInfo() : vertex(NGHolder::null_vertex()) {}
    NFAVertex vertex;
    CharReach reach;
};

#ifdef DEBUG
static
std::string dumpStates(const dynamic_bitset<> &s) {
    std::ostringstream oss;
    for (size_t i = s.find_first(); i != s.npos; i = s.find_next(i)) {
        oss << i << " ";
    }
    return oss.str();
}
#endif

static
void step(const NGHolder &g, const vector<StateInfo> &info,
          const dynamic_bitset<> &in, dynamic_bitset<> *out) {
    out->reset();
    for (size_t i = in.find_first(); i != in.npos; i = in.find_next(i)) {
        NFAVertex u = info[i].vertex;
        for (auto v : adjacent_vertices_range(u, g)) {
            out->set(g[v].index);
        }
    }
}

static
void filter_by_reach(const vector<StateInfo> &info, dynamic_bitset<> *states,
                     const CharReach &cr) {
    for (size_t i = states->find_first(); i != states->npos;
         i = states->find_next(i)) {
        if ((info[i].reach & cr).none()) {
            states->reset(i);
        }
    }
}

template<typename inputT>
static
void execute_graph_i(const NGHolder &g, const vector<StateInfo> &info,
                     const inputT &input, dynamic_bitset<> *states,
                     bool kill_sds) {
    dynamic_bitset<> &curr = *states;
    dynamic_bitset<> next(curr.size());
    DEBUG_PRINTF("%zu states in\n", states->count());

    for (const auto &e : input) {
        DEBUG_PRINTF("processing %s\n", describeClass(e).c_str());
        step(g, info, curr, &next);
        if (kill_sds) {
            next.reset(NODE_START_DOTSTAR);
        }
        filter_by_reach(info, &next, e);
        next.swap(curr);

        if (curr.empty()) {
            DEBUG_PRINTF("went dead\n");
            break;
        }
    }

    DEBUG_PRINTF("%zu states out\n", states->size());
}

static
dynamic_bitset<> makeStateBitset(const NGHolder &g,
                                 const flat_set<NFAVertex> &in) {
    dynamic_bitset<> work_states(num_vertices(g));
    for (const auto &v : in) {
        u32 idx = g[v].index;
        work_states.set(idx);
    }
    return work_states;
}

static
flat_set<NFAVertex> getVertices(const dynamic_bitset<> &in,
                                const vector<StateInfo> &info) {
    flat_set<NFAVertex> out;
    for (size_t i = in.find_first(); i != in.npos; i = in.find_next(i)) {
        out.insert(info[i].vertex);
    }
    return out;
}

static
vector<StateInfo> makeInfoTable(const NGHolder &g) {
    vector<StateInfo> info(num_vertices(g));
    for (auto v : vertices_range(g)) {
        u32 idx = g[v].index;
        const CharReach &cr = g[v].char_reach;
        assert(idx < info.size());
        info[idx] = StateInfo(v, cr);
    }
    return info;
}

flat_set<NFAVertex> execute_graph(const NGHolder &g, const ue2_literal &input,
                                  const flat_set<NFAVertex> &initial_states,
                                  bool kill_sds) {
    assert(hasCorrectlyNumberedVertices(g));

    auto info = makeInfoTable(g);
    auto work_states = makeStateBitset(g, initial_states);

    execute_graph_i(g, info, input, &work_states, kill_sds);

    return getVertices(work_states, info);
}

flat_set<NFAVertex> execute_graph(const NGHolder &g,
                                  const vector<CharReach> &input,
                                  const flat_set<NFAVertex> &initial_states) {
    assert(hasCorrectlyNumberedVertices(g));

    auto info = makeInfoTable(g);
    auto work_states = makeStateBitset(g, initial_states);

    execute_graph_i(g, info, input, &work_states, false);

    return getVertices(work_states, info);
}

namespace {
class eg_visitor : public boost::default_dfs_visitor {
public:
    eg_visitor(const NGHolder &running_g_in, const vector<StateInfo> &info_in,
               const NGHolder &input_g_in,
               map<NFAVertex, dynamic_bitset<> > &states_in)
        : vertex_count(num_vertices(running_g_in)), running_g(running_g_in),
          info(info_in), input_g(input_g_in), states(states_in),
          succs(vertex_count) {}

    void finish_vertex(NFAVertex input_v,
                   const boost::reverse_graph<NGHolder, const NGHolder &> &) {
        if (input_v == input_g.accept) {
            return;
        }
        assert(input_v != input_g.acceptEod);

        DEBUG_PRINTF("finished p%zu\n", input_g[input_v].index);

        /* finish vertex is called on vertex --> implies that all its parents
         * (in the forward graph) are also finished. Our parents will have
         *  pushed all of their successors for us into our stateset. */
        states[input_v].resize(vertex_count);
        dynamic_bitset<> our_states = states[input_v];
        states[input_v].reset();

        filter_by_reach(info, &our_states,
                        input_g[input_v].char_reach);

        if (input_v != input_g.startDs &&
            edge(input_v, input_v, input_g).second) {
            bool changed;
            do {
                DEBUG_PRINTF("actually not finished -> have self loop\n");
                succs.reset();
                step(running_g, info, our_states, &succs);
                filter_by_reach(info, &succs,
                                input_g[input_v].char_reach);
                dynamic_bitset<> our_states2 = our_states | succs;
                changed = our_states2 != our_states;
                our_states.swap(our_states2);
            } while (changed);
        }

        DEBUG_PRINTF("  active rstates: %s\n", dumpStates(our_states).c_str());

        succs.reset();
        step(running_g, info, our_states, &succs);

        /* we need to push into all our (forward) children their successors
         * from us. */
        for (auto v : adjacent_vertices_range(input_v, input_g)) {
            DEBUG_PRINTF("pushing our states to pstate %zu\n",
                         input_g[v].index);
            if (v == input_g.startDs) {
                /* no need for intra start edges */
                continue;
            }

            states[v].resize(vertex_count); // May not yet exist

            if (v != input_g.accept) {
                states[v] |= succs;
            } else {
                /* accept is a magical pseudo state which does not consume
                 * characters and we are using to collect the output states. We
                 * must fill it with our states rather than our succs. */
                DEBUG_PRINTF("prev outputted rstates: %s\n",
                             dumpStates(states[v]).c_str());
                DEBUG_PRINTF("outputted rstates: %s\n",
                             dumpStates(our_states).c_str());

                states[v] |= our_states;

                DEBUG_PRINTF("new outputted rstates: %s\n",
                             dumpStates(states[v]).c_str());
            }
        }

        /* note: the states at this vertex are no longer required */
    }

private:
    const size_t vertex_count;
    const NGHolder &running_g;
    const vector<StateInfo> &info;
    const NGHolder &input_g;
    map<NFAVertex, dynamic_bitset<> > &states; /* vertex in input_g -> set of
                                                  states in running_g */
    dynamic_bitset<> succs; // temp use internally
};
} // namespace

flat_set<NFAVertex> execute_graph(const NGHolder &running_g,
                                  const NGHolder &input_dag,
                                  const flat_set<NFAVertex> &input_start_states,
                                  const flat_set<NFAVertex> &initial_states) {
    DEBUG_PRINTF("g has %zu vertices, input_dag has %zu vertices\n",
                 num_vertices(running_g), num_vertices(input_dag));
    assert(hasCorrectlyNumberedVertices(running_g));
    assert(in_degree(input_dag.acceptEod, input_dag) == 1);

    map<NFAVertex, boost::default_color_type> colours;
    /* could just a topo order, but really it is time to pull a slightly bigger
     * gun: DFS */
    boost::reverse_graph<NGHolder, const NGHolder &> revg(input_dag);
    map<NFAVertex, dynamic_bitset<> > dfs_states;

    auto info = makeInfoTable(running_g);
    auto input_fs = makeStateBitset(running_g, initial_states);

    for (auto v : input_start_states) {
        dfs_states[v] = input_fs;
    }

    depth_first_visit(revg, input_dag.accept,
                      eg_visitor(running_g, info, input_dag, dfs_states),
                      make_assoc_property_map(colours));

    auto states = getVertices(dfs_states[input_dag.accept], info);

#ifdef DEBUG
    DEBUG_PRINTF("  output rstates:");
    for (const auto &v : states) {
        printf(" %zu", running_g[v].index);
    }
    printf("\n");
#endif

    return states;
}

flat_set<NFAVertex> execute_graph(const NGHolder &running_g,
                                  const NGHolder &input_dag,
                                  const flat_set<NFAVertex> &initial_states) {
    auto input_start_states = {input_dag.start, input_dag.startDs};
    return execute_graph(running_g, input_dag, input_start_states,
                         initial_states);
}

static
bool can_die_early(const NGHolder &g, const vector<StateInfo> &info,
                   const dynamic_bitset<> &s,
                   map<dynamic_bitset<>, u32> &visited, u32 age_limit) {
    if (contains(visited, s) && visited[s] >= age_limit) {
        /* we have already (or are in the process) of visiting here with a
         * looser limit. */
        return false;
    }
    visited[s] = age_limit;

    if (s.none()) {
        DEBUG_PRINTF("dead\n");
        return true;
    }

    if (age_limit == 0) {
        return false;
    }

    dynamic_bitset<> all_succ(s.size());
    step(g, info, s, &all_succ);
    all_succ.reset(NODE_START_DOTSTAR);

    for (u32 i = 0; i < N_CHARS; i++) {
        dynamic_bitset<> next = all_succ;
        filter_by_reach(info, &next, CharReach(i));
        if (can_die_early(g, info, next, visited, age_limit - 1)) {
            return true;
        }
    }

    return false;
}

bool can_die_early(const NGHolder &g, u32 age_limit) {
    if (proper_out_degree(g.startDs, g)) {
        return false;
    }
    const vector<StateInfo> &info = makeInfoTable(g);
    map<dynamic_bitset<>, u32> visited;
    return can_die_early(g, info, makeStateBitset(g, {g.start}), visited,
                         age_limit);
}

} // namespace ue2
