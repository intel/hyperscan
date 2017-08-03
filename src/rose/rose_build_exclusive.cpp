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

#include "rose_build_exclusive.h"

#include "ue2common.h"
#include "rose_build_merge.h"
#include "nfa/castlecompile.h"
#include "nfagraph/ng_execute.h"
#include "nfagraph/ng_holder.h"
#include "nfagraph/ng_util.h"
#include "util/clique.h"
#include "util/compile_context.h"
#include "util/container.h"
#include "util/flat_containers.h"
#include "util/graph.h"
#include "util/make_unique.h"

using namespace std;

namespace ue2 {

template<typename role_id>
struct RoleChunk {
    vector<RoleInfo<role_id>> roles;
};

static
CharReach getReachability(const NGHolder &h) {
    CharReach cr;
    for (const auto &v : vertices_range(h)) {
        if (!is_special(v, h)) {
            cr |= h[v].char_reach;
        }
    }
    return cr;
}

template<typename role_id>
static
vector<RoleChunk<role_id>> divideIntoChunks(const RoseBuildImpl &build,
                                 set<RoleInfo<role_id>> &roleInfoSet) {
    u32 chunkSize = build.cc.grey.tamaChunkSize;
    u32 cnt = 1;
    vector<RoleChunk<role_id>> chunks;
    RoleChunk<role_id> roleChunk;
    for (const auto &roleInfo : roleInfoSet) {
        if (cnt == chunkSize) {
            cnt -= chunkSize;
            chunks.push_back(roleChunk);
            roleChunk.roles.clear();
        }
        roleChunk.roles.push_back(roleInfo);
        cnt++;
    }

    if (cnt > 1) {
        chunks.push_back(roleChunk);
    }

    return chunks;
}

/* add prefix literals to engine graph */
static
bool addPrefixLiterals(NGHolder &h, unordered_set<u32> &tailId,
                       const vector<vector<CharReach>> &triggers) {
    DEBUG_PRINTF("add literals to graph\n");

    NFAVertex start = h.start;
    vector<NFAVertex> heads;
    vector<NFAVertex> tails;
    for (const auto &lit : triggers) {
        NFAVertex last = start;
        if (lit.empty()) {
            return false;
        }
        u32 i = 0;
        for (const auto &c : lit) {
            DEBUG_PRINTF("lit:%s \n", c.to_string().c_str());
            NFAVertex u = add_vertex(h);
            h[u].char_reach = c;
            if (!i++) {
                heads.push_back(u);
                last = u;
                continue;
            }
            add_edge(last, u, h);
            last = u;
        }
        tails.push_back(last);
        tailId.insert(h[last].index);
    }

    for (auto v : adjacent_vertices_range(start, h)) {
        if (v != h.startDs) {
            for (auto &t : tails) {
                add_edge(t, v, h);
            }
        }
    }

    clear_out_edges(start, h);
    add_edge(h.start, h.start, h);
    for (auto &t : heads) {
        add_edge(start, t, h);
    }

    DEBUG_PRINTF("literals addition done\n");
    return true;
}

/* check if one literal is suffix of another */
static
bool isSuffix(const vector<vector<CharReach>> &triggers1,
              const vector<vector<CharReach>> &triggers2) {
    // literal suffix test
    for (const auto &lit1 : triggers1) {
        for (const auto &lit2 : triggers2) {
            const size_t len = min(lit1.size(), lit2.size());
            if (equal(lit1.rbegin(), lit1.rbegin() + len,
                      lit2.rbegin(), overlaps)) {
                return true;
            }
        }
    }
    return false;
}

/* prepare initial infix or suffix graph used for exclusive analysis */
template<typename role_id>
static
u32 prepareRoleGraph(NGHolder &h, const role_id &s1) {
    u32 num = 0;
    if (s1.castle()) {
        num = num_vertices(h);
        NFAVertex u = add_vertex(h);
        h[u].char_reach = s1.castle()->reach();
        add_edge(h.startDs, u, h);
        // add self loop to repeat characters
        add_edge(u, u, h);
    } else if (s1.graph()) {
        const NGHolder &g = *s1.graph();
        cloneHolder(h, g);
        num = num_vertices(h);
    } else {
        // only infixes and suffixes with graph properties are possible
        // candidates, already filtered out other cases before
        // exclusive analysis
        assert(0);
    }

    return num;
}

/* get a subset of literal if reset character is found */
static
vector<CharReach> findStartPos(const CharReach &cr1,
                               const vector<CharReach> &lit) {
    auto it = lit.rbegin(), ite = lit.rend();
    u32 pos = lit.size();
    for (; it != ite; it++) {
        if (!overlaps(cr1, *it)) {
            break;
        }
        pos--;
    }

    return vector<CharReach> (lit.begin() + pos, lit.end());
}

template<typename role_id>
static
bool isExclusive(const NGHolder &h,
                 const u32 num, unordered_set<u32> &tailId,
                 map<u32, unordered_set<u32>> &skipList,
                 const RoleInfo<role_id> &role1,
                 const RoleInfo<role_id> &role2) {
    const u32 id1 = role1.id;
    const u32 id2 = role2.id;

    if (contains(skipList, id1) && contains(skipList[id1], id2)) {
        return false;
    }

    const auto &triggers1 = role1.literals;
    const auto &triggers2 = role2.literals;
    if (isSuffix(triggers1, triggers2)) {
        skipList[id2].insert(id1);
        return false;
    }

    DEBUG_PRINTF("role id2:%u\n", id2);
    const auto &cr1 = role1.cr;
    if (overlaps(cr1, role2.last_cr)) {
        CharReach cr = cr1 | role1.prefix_cr;
        flat_set<NFAVertex> states;
        for (const auto &lit : triggers2) {
            auto lit1 = findStartPos(cr, lit);
            if (lit1.empty()) {
                continue;
            }

            states.clear();

            if (lit1.size() < lit.size()) {
                // Only starts.
                states.insert(h.start);
                states.insert(h.startDs);
            } else {
                // All vertices.
                insert(&states, vertices(h));
            }

            auto activeStates = execute_graph(h, lit1, states);
            // Check if only literal states are on
            for (const auto &s : activeStates) {
                if ((!is_any_start(s, h) && h[s].index <= num) ||
                    contains(tailId, h[s].index)) {
                    skipList[id2].insert(id1);
                    return false;
                }
            }
        }
    }

    return true;
}

template<typename role_id>
static
unordered_set<u32> checkExclusivity(const NGHolder &h,
                                    const u32 num, unordered_set<u32> &tailId,
                                    map<u32, unordered_set<u32>> &skipList,
                                    const RoleInfo<role_id> &role1,
                                    const RoleChunk<role_id> &roleChunk) {
    unordered_set<u32> info;
    const u32 id1 = role1.id;
    for (const auto &role2 : roleChunk.roles) {
        const u32 id2 = role2.id;
        if (id1 != id2 && isExclusive(h, num, tailId, skipList,
                                      role1, role2)) {
            info.insert(id2);
        }
    }

    return info;
}

static
void findCliques(const map<u32, set<u32>> &exclusiveGroups,
                 vector<vector<u32>> &exclusive_roles) {
    if (exclusiveGroups.empty()) {
        return;
    }
    // Construct the exclusivity graph
    map<u32, CliqueVertex> vertex_map;
    unique_ptr<CliqueGraph> cg = make_unique<CliqueGraph>();

    // Add vertices representing infixes/suffixes
    for (const auto &e : exclusiveGroups) {
        const u32 id = e.first;
        CliqueVertex v1 = add_vertex(CliqueVertexProps(id), *cg);
        vertex_map[id] = v1;
    }

    // Wire exclusive pairs
    for (const auto &e1 : exclusiveGroups) {
        const u32 literalId1 = e1.first;
        CliqueVertex lv = vertex_map[literalId1];
        const set<u32> &exclusiveSet = e1.second;
        for (const auto &e2 : exclusiveGroups) {
            const u32 literalId2 = e2.first;
            if (literalId1 < literalId2 &&
                contains(exclusiveSet, literalId2)) {
                add_edge(lv, vertex_map[literalId2], *cg);
                DEBUG_PRINTF("Wire %u:%u\n", literalId1, literalId2);
            }
        }
    }

    // Find clique groups
    const auto &clique = removeClique(*cg);
    for (const auto &i : clique) {
        DEBUG_PRINTF("cliq:%zu\n", i.size());
        if (i.size() > 1) {
            exclusive_roles.push_back(i);
        }
    }
    DEBUG_PRINTF("Clique graph size:%zu\n", exclusive_roles.size());
}

static
map<u32, set<u32>> findExclusiveGroups(const RoseBuildImpl &build,
            const map<u32, unordered_set<u32>> &exclusiveInfo,
            const map<u32, vector<RoseVertex>> &vertex_map,
            const bool is_infix) {
    map<u32, set<u32>> exclusiveGroups;
    for (const auto &e : exclusiveInfo) {
        u32 i = e.first;
        const auto &s = e.second;
        set<u32> group;
        set<RoseVertex> q1(vertex_map.at(i).begin(),
                           vertex_map.at(i).end());
        DEBUG_PRINTF("vertex set:%zu\n", q1.size());
        for (const auto &val : s) {
            set<RoseVertex> q2(vertex_map.at(val).begin(),
                               vertex_map.at(val).end());
            if (contains(exclusiveInfo.at(val), i) &&
                (!is_infix || mergeableRoseVertices(build, q1, q2))) {
                group.insert(val);
            }
        }
        if (!group.empty()) {
            exclusiveGroups[i] = group;
        }
    }

    return exclusiveGroups;
}

template<typename role_id>
static
bool setTriggerLiterals(RoleInfo<role_id> &roleInfo,
        const map<u32, vector<vector<CharReach>>> &triggers) {
    u32 minLiteralLen = ~0U;
    for (const auto &tr : triggers) {
        for (const auto &lit : tr.second) {
            if (lit.empty()) {
                return false;
            }
            minLiteralLen = min(minLiteralLen, (u32)lit.size());
            roleInfo.last_cr |= lit.back();
            for (const auto &c : lit) {
                roleInfo.prefix_cr |= c;
            }
            roleInfo.literals.push_back(lit);
        }
    }

    if (roleInfo.role.graph()) {
        const NGHolder &g = *roleInfo.role.graph();
        roleInfo.cr = getReachability(g);
    } else if (roleInfo.role.castle()) {
        roleInfo.cr = roleInfo.role.castle()->reach();
    }

    // test the score of this engine
    roleInfo.score = 256 - roleInfo.cr.count() + minLiteralLen;
    if (roleInfo.score < 20) {
        return false;
    }

    return true;
}

bool setTriggerLiteralsInfix(RoleInfo<left_id> &roleInfo,
        const map<u32, vector<vector<CharReach>>> &triggers) {
    return setTriggerLiterals(roleInfo, triggers);
}

bool setTriggerLiteralsSuffix(RoleInfo<suffix_id> &roleInfo,
        const map<u32, vector<vector<CharReach>>> &triggers) {
    return setTriggerLiterals(roleInfo, triggers);
}

template<typename role_id>
static
void exclusiveAnalysis(const RoseBuildImpl &build,
               const map<u32, vector<RoseVertex>> &vertex_map,
               set<RoleInfo<role_id>> &roleInfoSet,
               vector<vector<u32>> &exclusive_roles, const bool is_infix) {
    const auto &chunks = divideIntoChunks(build, roleInfoSet);
    DEBUG_PRINTF("Exclusivity analysis entry\n");
    map<u32, unordered_set<u32>> exclusiveInfo;

    for (const auto &roleChunk : chunks) {
        map<u32, unordered_set<u32>> skipList;
        for (const auto &role1 : roleChunk.roles) {
            const u32 id1 = role1.id;
            const role_id &s1 = role1.role;
            const auto &triggers1 = role1.literals;

            NGHolder h;
            u32 num = prepareRoleGraph(h, s1);
            DEBUG_PRINTF("role id1:%u\n", id1);
            unordered_set<u32> tailId;
            if (!addPrefixLiterals(h, tailId, triggers1)) {
                continue;
            }

            exclusiveInfo[id1] = checkExclusivity(h, num, tailId,
                                             skipList, role1, roleChunk);
        }
    }

    // Create final candidate exclusive groups
    const auto exclusiveGroups =
        findExclusiveGroups(build, exclusiveInfo, vertex_map, is_infix);
    exclusiveInfo.clear();

    // Find cliques for each exclusive groups
    findCliques(exclusiveGroups, exclusive_roles);
}

void exclusiveAnalysisInfix(const RoseBuildImpl &build,
               const map<u32, vector<RoseVertex>> &vertex_map,
               set<RoleInfo<left_id>> &roleInfoSet,
               vector<vector<u32>> &exclusive_roles) {
    exclusiveAnalysis(build, vertex_map, roleInfoSet, exclusive_roles,
                      true);
}

void exclusiveAnalysisSuffix(const RoseBuildImpl &build,
               const map<u32, vector<RoseVertex>> &vertex_map,
               set<RoleInfo<suffix_id>> &roleInfoSet,
               vector<vector<u32>> &exclusive_roles) {
    exclusiveAnalysis(build, vertex_map, roleInfoSet, exclusive_roles,
                      false);
}

} // namespace ue2
