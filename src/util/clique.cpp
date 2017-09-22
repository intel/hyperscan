/*
 * Copyright (c) 2016-2017, Intel Corporation
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
 * \brief An algorithm to find cliques.
 */

#include "clique.h"
#include "container.h"
#include "graph_range.h"
#include "make_unique.h"

#include <map>
#include <set>
#include <stack>

using namespace std;

namespace ue2 {

static
vector<u32> getNeighborInfo(const CliqueGraph &g,
                     const CliqueVertex &cv, const set<u32> &group) {
    u32 id = g[cv].stateId;
    vector<u32> neighbor;
    // find neighbors for cv
    for (const auto &v : adjacent_vertices_range(cv, g)) {
        if (g[v].stateId != id && contains(group, g[v].stateId)){
            neighbor.push_back(g[v].stateId);
            DEBUG_PRINTF("Neighbor:%u\n", g[v].stateId);
        }
    }

    return neighbor;
}

static
vector<u32> findCliqueGroup(CliqueGraph &cg) {
    stack<vector<u32>> gStack;

    // Create mapping between vertex and id
    map<u32, CliqueVertex> vertexMap;
    vector<u32> init;
    for (const auto &v : vertices_range(cg)) {
        vertexMap[cg[v].stateId] = v;
        init.push_back(cg[v].stateId);
    }
    gStack.push(init);

    // Get the vertex to start from
    vector<u32> clique;
    while (!gStack.empty()) {
        vector<u32> g = move(gStack.top());
        gStack.pop();

        // Choose a vertex from the graph
        u32 id = g[0];
        CliqueVertex &n = vertexMap.at(id);
        clique.push_back(id);
        // Corresponding vertex in the original graph
        set<u32> subgraphId(g.begin(), g.end());
        auto neighbor = getNeighborInfo(cg, n, subgraphId);
        // Get graph consisting of neighbors for left branch
        if (!neighbor.empty()) {
            gStack.push(neighbor);
        }
    }

    return clique;
}

template<typename Graph>
bool graph_empty(const Graph &g) {
    typename Graph::vertex_iterator vi, ve;
    tie(vi, ve) = vertices(g);
    return vi == ve;
}

vector<vector<u32>> removeClique(CliqueGraph &cg) {
    DEBUG_PRINTF("graph size:%zu\n", num_vertices(cg));
    vector<vector<u32>> cliquesVec = {findCliqueGroup(cg)};
    while (!graph_empty(cg)) {
        const vector<u32> &c = cliquesVec.back();
        vector<CliqueVertex> dead;
        for (const auto &v : vertices_range(cg)) {
            u32 id = cg[v].stateId;
            if (find(c.begin(), c.end(), id) != c.end()) {
                dead.push_back(v);
            }
        }
        for (const auto &v : dead) {
            clear_vertex(v, cg);
            remove_vertex(v, cg);
        }
        if (graph_empty(cg)) {
            break;
        }
        auto clique = findCliqueGroup(cg);
        cliquesVec.push_back(clique);
    }

    return cliquesVec;
}

} // namespace ue2
