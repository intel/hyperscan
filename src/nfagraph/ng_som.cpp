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

/**
 * \file
 * \brief SOM ("Start of Match") analysis.
 */

#include "ng_som.h"

#include "ng.h"
#include "ng_dump.h"
#include "ng_equivalence.h"
#include "ng_execute.h"
#include "ng_haig.h"
#include "ng_limex.h"
#include "ng_literal_analysis.h"
#include "ng_prune.h"
#include "ng_redundancy.h"
#include "ng_region.h"
#include "ng_reports.h"
#include "ng_som_add_redundancy.h"
#include "ng_som_util.h"
#include "ng_split.h"
#include "ng_util.h"
#include "ng_violet.h"
#include "ng_width.h"
#include "grey.h"
#include "ue2common.h"
#include "compiler/compiler.h"
#include "nfa/goughcompile.h"
#include "nfa/nfa_internal.h" // for MO_INVALID_IDX
#include "parser/position.h"
#include "som/som.h"
#include "rose/rose_build.h"
#include "rose/rose_in_util.h"
#include "util/alloc.h"
#include "util/compare.h"
#include "util/compile_error.h"
#include "util/container.h"
#include "util/dump_charclass.h"
#include "util/graph_range.h"
#include "util/make_unique.h"

#include <algorithm>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace std;

namespace ue2 {

static const size_t MAX_SOM_PLANS = 10;
static const size_t MAX_SOMBE_CHAIN_VERTICES = 4000;

#define MAX_REV_NFA_PREFIX 80

namespace {
struct som_plan {
    som_plan(const shared_ptr<NGHolder> &p, const CharReach &e, bool i,
             u32 parent_in) : prefix(p), escapes(e), is_reset(i),
                              no_implement(false), parent(parent_in) { }
    shared_ptr<NGHolder> prefix;
    CharReach escapes;
    bool is_reset;
    bool no_implement;
    u32 parent; // index of parent plan in the vector.

    // Reporters: a list of vertices in the graph that must be have their
    // reports updated at implementation time to report this plan's
    // som_loc_out.
    vector<NFAVertex> reporters;

    // Similar, but these report the som_loc_in.
    vector<NFAVertex> reporters_in;
};
}

static
bool regionCanEstablishSom(const NGHolder &g,
                           const unordered_map<NFAVertex, u32> &regions,
                           const u32 region, const vector<NFAVertex> &r_exits,
                           const vector<DepthMinMax> &depths) {
    if (region == regions.at(g.accept) ||
        region == regions.at(g.acceptEod)) {
        DEBUG_PRINTF("accept in region\n");
        return false;
    }

    DEBUG_PRINTF("region %u\n", region);
    for (UNUSED auto v : r_exits) {
        DEBUG_PRINTF("    exit %zu\n", g[v].index);
    }

    /* simple if each region exit is at fixed distance from SOM. Note SOM does
       not include virtual starts */
    for (auto v : r_exits) {
        assert(regions.at(v) == region);
        const DepthMinMax &d = depths.at(g[v].index);
        if (d.min != d.max) {
            DEBUG_PRINTF("failing %zu as %s != %s\n", g[v].index,
                         d.min.str().c_str(), d.max.str().c_str());
            return false;
        }
    }
    DEBUG_PRINTF("region %u/%zu is good\n", regions.at(r_exits[0]),
                 g[r_exits[0]].index);

    return true;
}

namespace {

struct region_info {
    region_info() : optional(false), dag(false) {}
    vector<NFAVertex> enters;
    vector<NFAVertex> exits;
    vector<NFAVertex> full;
    bool optional; /* skip edges around region */
    bool dag; /* completely acyclic */
};

}

static
void buildRegionMapping(const NGHolder &g,
                        const unordered_map<NFAVertex, u32> &regions,
                        map<u32, region_info> &info,
                        bool include_region_0 = false) {
    for (auto v : vertices_range(g)) {
        u32 region = regions.at(v);
        if (!include_region_0 && (is_any_start(v, g) || region == 0)) {
            continue;
        }
        assert(!region || !is_any_start(v, g));

        if (is_any_accept(v, g)) {
            continue;
        }

        if (isRegionEntry(g, v, regions)) {
            info[region].enters.push_back(v);
        }
        if (isRegionExit(g, v, regions)) {
            info[region].exits.push_back(v);
        }
        info[region].full.push_back(v);
    }

    for (auto &m : info) {
        if (!m.second.enters.empty()
            && isOptionalRegion(g, m.second.enters.front(), regions)) {
            m.second.optional = true;
        }
        m.second.dag = true; /* will be cleared for cyclic regions later */
    }

    set<NFAEdge> be;
    BackEdges<set<NFAEdge> > backEdgeVisitor(be);
    boost::depth_first_search(g, visitor(backEdgeVisitor).root_vertex(g.start));

    for (const auto &e : be) {
        NFAVertex u = source(e, g);
        NFAVertex v = target(e, g);
        if (is_special(u, g) || is_special(v, g)) {
            assert(is_special(u, g) && is_special(v, g));
            continue;
        }
        u32 r = regions.at(v);
        assert(regions.at(u) == r);
        info[r].dag = false;
    }

    if (include_region_0) {
        info[0].dag = false;
    }

    #ifdef DEBUG
    for (const auto &m : info) {
        u32 r = m.first;
        const region_info &r_i = m.second;
        DEBUG_PRINTF("region %u:%s%s\n", r,
                     r_i.dag ? " (dag)" : "",
                     r_i.optional ? " (optional)" : "");
        DEBUG_PRINTF("  enters:");
        for (u32 i = 0; i < r_i.enters.size(); i++) {
            printf(" %zu", g[r_i.enters[i]].index);
        }
        printf("\n");
        DEBUG_PRINTF("  exits:");
        for (u32 i = 0; i < r_i.exits.size(); i++) {
            printf(" %zu", g[r_i.exits[i]].index);
        }
        printf("\n");
        DEBUG_PRINTF("  all:");
        for (u32 i = 0; i < r_i.full.size(); i++) {
            printf(" %zu", g[r_i.full[i]].index);
        }
        printf("\n");
    }
    #endif
}

static
bool validateXSL(const NGHolder &g,
                 const unordered_map<NFAVertex, u32> &regions,
                 const u32 region, const CharReach &escapes, u32 *bad_region) {
    /* need to check that the escapes escape all of the graph past region */
    u32 first_bad_region = ~0U;
    for (auto v : vertices_range(g)) {
        u32 v_region = regions.at(v);
        if (!is_special(v, g) && v_region > region &&
            (escapes & g[v].char_reach).any()) {
            DEBUG_PRINTF("problem with escapes for %zu\n", g[v].index);
            first_bad_region = MIN(first_bad_region, v_region);
        }
    }

    if (first_bad_region != ~0U) {
        *bad_region = first_bad_region;
        return false;
    }

    return true;
}

static
bool validateEXSL(const NGHolder &g,
                  const unordered_map<NFAVertex, u32> &regions,
                  const u32 region, const CharReach &escapes,
                  const NGHolder &prefix, u32 *bad_region) {
    /* EXSL: To be a valid EXSL with escapes e, we require that all states
     * go dead after /[e][^e]*{subsequent prefix match}/.
     */

    /* TODO: this is overly conservative as it allow partial matches from the
     * prefix to be considered even when the tail has processed some [^e] */

    u32 first_bad_region = ~0U;
    const vector<CharReach> escapes_vec(1, escapes);
    const vector<CharReach> notescapes_vec(1, ~escapes);

    flat_set<NFAVertex> states;
    /* turn on all states past the prefix */
    DEBUG_PRINTF("region %u is cutover\n", region);
    for (auto v : vertices_range(g)) {
        if (!is_special(v, g) && regions.at(v) > region) {
            states.insert(v);
        }
    }

    /* process the escapes */
    states = execute_graph(g, escapes_vec, states);

    /* flood with any number of not escapes */
    flat_set<NFAVertex> prev_states;
    while (prev_states != states) {
        prev_states = states;
        states = execute_graph(g, notescapes_vec, states);
        insert(&states, prev_states);
    }

    /* find input starts to use for when we are running the prefix through as
     * when the escape character arrives we may be in matching the prefix
     * already */
    flat_set<NFAVertex> prefix_start_states;
    for (auto v : vertices_range(prefix)) {
        if (v != prefix.accept && v != prefix.acceptEod
            /* and as we have already made it past the prefix once */
            && v != prefix.start) {
            prefix_start_states.insert(v);
        }
    }

    prefix_start_states =
        execute_graph(prefix, escapes_vec, prefix_start_states);

    assert(contains(prefix_start_states, prefix.startDs));
    /* see what happens after we feed it the prefix */
    states = execute_graph(g, prefix, prefix_start_states, states);

    for (auto v : states) {
        assert(v != g.accept && v != g.acceptEod); /* no cr -> should never be
                                                    * on */
        DEBUG_PRINTF("state still active\n");
        first_bad_region = MIN(first_bad_region, regions.at(v));
    }

    if (first_bad_region != ~0U) {
        *bad_region = first_bad_region;
        return false;
    }

    return true;
}

static
bool isPossibleLock(const NGHolder &g,
                    map<u32, region_info>::const_iterator region,
                    const map<u32, region_info> &info,
                    CharReach *escapes_out) {
    /* TODO: we could also check for self-loops on curr region */

    /* TODO: some straw-walking logic. lowish priority has we know there can
     * only be optional regions between us and the cyclic */

    assert(region != info.end());
    map<u32, region_info>::const_iterator next_region = region;
    ++next_region;
    if (next_region == info.end()) {
        assert(0); /* odd */
        return false;
    }

    const region_info &next_info = next_region->second;
    if (next_info.enters.empty()) {
        assert(0); /* odd */
        return false;
    }

    if (next_info.full.size() == 1 && !next_info.dag) {
       *escapes_out = ~g[next_info.full.front()].char_reach;
       return true;
    }

    return false;
}

static
unique_ptr<NGHolder>
makePrefix(const NGHolder &g, const unordered_map<NFAVertex, u32> &regions,
           const region_info &curr, const region_info &next,
           bool renumber = true) {
    const vector<NFAVertex> &curr_exits = curr.exits;
    const vector<NFAVertex> &next_enters = next.enters;

    assert(!next_enters.empty());
    assert(!curr_exits.empty());

    unique_ptr<NGHolder> prefix_ptr = ue2::make_unique<NGHolder>();
    NGHolder &prefix = *prefix_ptr;

    deque<NFAVertex> lhs_verts;
    insert(&lhs_verts, lhs_verts.end(), vertices(g));

    unordered_map<NFAVertex, NFAVertex> lhs_map; // g -> prefix
    fillHolder(&prefix, g, lhs_verts, &lhs_map);
    prefix.kind = NFA_OUTFIX;

    // We need a reverse mapping to track regions.
    unordered_map<NFAVertex, NFAVertex> rev_map; // prefix -> g
    for (const auto &e : lhs_map) {
        rev_map.emplace(e.second, e.first);
    }

    clear_in_edges(prefix.accept, prefix);
    clear_in_edges(prefix.acceptEod, prefix);
    add_edge(prefix.accept, prefix.acceptEod, prefix);

    assert(!next_enters.empty());
    assert(next_enters.front() != NGHolder::null_vertex());
    u32 dead_region = regions.at(next_enters.front());
    DEBUG_PRINTF("curr_region %u, dead_region %u\n",
                 regions.at(curr_exits.front()), dead_region);
    for (auto v : inv_adjacent_vertices_range(next_enters.front(), g)) {
        if (regions.at(v) >= dead_region) {
            continue;
        }
        /* add edge to new accepts */
        NFAVertex p_v = lhs_map[v];
        add_edge(p_v, prefix.accept, prefix);
    }

    assert(in_degree(prefix.accept, prefix) != 0);

    /* prune everything past the picked region */
    vector<NFAVertex> to_clear;
    assert(contains(lhs_map, curr_exits.front()));
    NFAVertex p_u = lhs_map[curr_exits.front()];
    DEBUG_PRINTF("p_u: %zu\n", prefix[p_u].index);
    for (auto p_v : adjacent_vertices_range(p_u, prefix)) {
        auto v = rev_map.at(p_v);
        if (p_v == prefix.accept || regions.at(v) < dead_region) {
            continue;
        }
        to_clear.push_back(p_v);
    }

    for (auto v : to_clear) {
        DEBUG_PRINTF("clearing in_edges on %zu\n", prefix[v].index);
        clear_in_edges(v, prefix);
    }

    pruneUseless(prefix, renumber /* sometimes we want no renumber to keep
                                     depth map valid */);

    assert(num_vertices(prefix) > N_SPECIALS);
    return prefix_ptr;
}

static
void replaceTempSomSlot(ReportManager &rm, NGHolder &g, u32 real_slot) {
    const u32 temp_slot = UINT32_MAX;
    /* update the som slot on the prefix report */
    for (auto v : inv_adjacent_vertices_range(g.accept, g)) {
        auto &reports = g[v].reports;
        assert(reports.size() == 1);
        Report ir = rm.getReport(*reports.begin());
        if (ir.onmatch != temp_slot) {
            continue;
        }
        ir.onmatch = real_slot;
        ReportID rep = rm.getInternalId(ir);

        assert(reports.size() == 1);
        reports.clear();
        reports.insert(rep);
    }
}

static
void setPrefixReports(ReportManager &rm, NGHolder &g, ReportType ir_type,
                      u32 som_loc, const vector<DepthMinMax> &depths,
                      bool prefix_by_rev) {
    Report ir = makeCallback(0U, 0);
    ir.type = ir_type;
    ir.onmatch = som_loc;

    /* add report for storing in som location on new accepts */
    for (auto v : inv_adjacent_vertices_range(g.accept, g)) {
        if (prefix_by_rev) {
            ir.somDistance = MO_INVALID_IDX; /* will be populated properly
                                              * later */
        } else {
            const DepthMinMax &d = depths.at(g[v].index);
            assert(d.min == d.max);
            ir.somDistance = d.max;
        }
        ReportID rep = rm.getInternalId(ir);

        auto &reports = g[v].reports;
        reports.clear();
        reports.insert(rep);
    }
}

static
void updatePrefixReports(ReportManager &rm, NGHolder &g, ReportType ir_type) {
    /* update the som action on the prefix report */
    for (auto v : inv_adjacent_vertices_range(g.accept, g)) {
        auto &reports = g[v].reports;
        assert(reports.size() == 1);
        Report ir = rm.getReport(*reports.begin());
        ir.type = ir_type;
        ReportID rep = rm.getInternalId(ir);

        assert(reports.size() == 1);
        reports.clear();
        reports.insert(rep);
    }
}

static
void updatePrefixReportsRevNFA(ReportManager &rm, NGHolder &g,
                               u32 rev_comp_id) {
    /* update the action on the prefix report, to refer to a reverse nfa,
     * report type is also adjusted. */
    for (auto v : inv_adjacent_vertices_range(g.accept, g)) {
        auto &reports = g[v].reports;
        assert(reports.size() == 1);
        Report ir = rm.getReport(*reports.begin());
        switch (ir.type) {
        case INTERNAL_SOM_LOC_SET:
            ir.type = INTERNAL_SOM_LOC_SET_SOM_REV_NFA;
            break;
        case INTERNAL_SOM_LOC_SET_IF_UNSET:
            ir.type = INTERNAL_SOM_LOC_SET_SOM_REV_NFA_IF_UNSET;
            break;
        case INTERNAL_SOM_LOC_SET_IF_WRITABLE:
            ir.type = INTERNAL_SOM_LOC_SET_SOM_REV_NFA_IF_WRITABLE;
            break;
        default:
            assert(0);
            break;
        }

        ir.revNfaIndex = rev_comp_id;
        ReportID rep = rm.getInternalId(ir);

        assert(reports.size() == 1);
        reports.clear();
        reports.insert(rep);
    }
}

static
void setMidfixReports(ReportManager &rm, const som_plan &item,
                      const u32 som_slot_in, const u32 som_slot_out) {
    assert(item.prefix);
    NGHolder &g = *item.prefix;

    Report ir = makeCallback(0U, 0);
    ir.type = item.is_reset ? INTERNAL_SOM_LOC_COPY
                            : INTERNAL_SOM_LOC_COPY_IF_WRITABLE;
    ir.onmatch = som_slot_out;
    ir.somDistance = som_slot_in;
    ReportID rep = rm.getInternalId(ir);

    /* add report for storing in som location on new accepts */
    for (auto v : inv_adjacent_vertices_range(g.accept, g)) {
        auto &reports = g[v].reports;
        reports.clear();
        reports.insert(rep);
    }
}

static
bool finalRegion(const NGHolder &g,
                 const unordered_map<NFAVertex, u32> &regions,
                 NFAVertex v) {
    u32 region = regions.at(v);
    for (auto w : adjacent_vertices_range(v, g)) {
        if (w != g.accept && w != g.acceptEod && regions.at(w) != region) {
            return false;
        }
    }

    return true;
}

static
void replaceExternalReportsWithSomRep(ReportManager &rm, NGHolder &g,
                                      NFAVertex v, ReportType ir_type,
                                      u64a param) {
    assert(!g[v].reports.empty());

    flat_set<ReportID> r_new;

    for (const ReportID &report_id : g[v].reports) {
        Report ir = rm.getReport(report_id);

        if (ir.type != EXTERNAL_CALLBACK) {
            /* we must have already done whatever magic we needed to do to this
             * report */
            r_new.insert(report_id);
            continue;
        }

        ir.type = ir_type;
        ir.somDistance = param;
        ReportID rep = rm.getInternalId(ir);

        DEBUG_PRINTF("vertex %zu, replacing report %u with %u (type %u)\n",
                     g[v].index, report_id, rep, ir_type);
        r_new.insert(rep);
    }
    g[v].reports = r_new;
}

/* updates the reports on all vertices leading to the sink */
static
void makeSomRelReports(ReportManager &rm, NGHolder &g, NFAVertex sink,
                       const vector<DepthMinMax> &depths) {
    for (auto v : inv_adjacent_vertices_range(sink, g)) {
        if (v == g.accept) {
            continue;
        }

        const DepthMinMax &d = depths.at(g[v].index);
        assert(d.min == d.max);
        replaceExternalReportsWithSomRep(rm, g, v, EXTERNAL_CALLBACK_SOM_REL,
                                         d.min);
    }
}

/* updates the reports on all the provided vertices */
static
void makeSomRelReports(ReportManager &rm, NGHolder &g,
                       const vector<NFAVertex> &to_update,
                       const vector<DepthMinMax> &depths) {
    for (auto v : to_update) {
        const DepthMinMax &d = depths.at(g[v].index);
        assert(d.min == d.max);
        replaceExternalReportsWithSomRep(rm, g, v, EXTERNAL_CALLBACK_SOM_REL,
                                         d.min);
    }
}

static
void makeSomAbsReports(ReportManager &rm, NGHolder &g, NFAVertex sink) {
    for (auto v : inv_adjacent_vertices_range(sink, g)) {
        if (v == g.accept) {
            continue;
        }
        replaceExternalReportsWithSomRep(rm, g, v, EXTERNAL_CALLBACK_SOM_ABS,
                                         0);
    }
}

static
void updateReportToUseRecordedSom(ReportManager &rm, NGHolder &g, u32 som_loc) {
    for (auto v : inv_adjacent_vertices_range(g.accept, g)) {
        replaceExternalReportsWithSomRep(rm, g, v, EXTERNAL_CALLBACK_SOM_STORED,
                                         som_loc);
    }
    for (auto v : inv_adjacent_vertices_range(g.acceptEod, g)) {
        if (v == g.accept) {
            continue;
        }
        replaceExternalReportsWithSomRep(rm, g, v, EXTERNAL_CALLBACK_SOM_STORED,
                                         som_loc);
    }
}

static
void updateReportToUseRecordedSom(ReportManager &rm, NGHolder &g,
                                  const vector<NFAVertex> &to_update,
                                  u32 som_loc) {
    for (auto v : to_update) {
        replaceExternalReportsWithSomRep(rm, g, v, EXTERNAL_CALLBACK_SOM_STORED,
                                         som_loc);
    }
}

static
bool createEscaper(NG &ng, const NGHolder &prefix, const CharReach &escapes,
                   u32 som_loc) {
    ReportManager &rm = ng.rm;

    /* escaper = /prefix[^escapes]*[escapes]/ */
    DEBUG_PRINTF("creating escaper for %u\n", som_loc);
    NGHolder h;
    cloneHolder(h, prefix);
    assert(h.kind == NFA_OUTFIX);

    NFAVertex u = add_vertex(h);
    h[u].char_reach = ~escapes;

    NFAVertex v = add_vertex(h);
    h[v].char_reach = escapes;

    for (auto w : inv_adjacent_vertices_range(h.accept, h)) {
        add_edge(w, u, h);
        add_edge(w, v, h);
        h[w].reports.clear();
    }

    clear_in_edges(h.accept, h);

    add_edge(u, v, h);
    add_edge(u, u, h);
    add_edge(v, h.accept, h);

    Report ir = makeCallback(0U, 0);
    ir.type = INTERNAL_SOM_LOC_MAKE_WRITABLE;
    ir.onmatch = som_loc;
    h[v].reports.insert(rm.getInternalId(ir));
    return ng.addHolder(h);
}

static
void fillHolderForLockCheck(NGHolder *out, const NGHolder &g,
                            const map<u32, region_info> &info,
                            map<u32, region_info>::const_iterator picked) {
    /* NOTE: This is appropriate for firstMatchIsFirst */
    DEBUG_PRINTF("prepping for lock check\n");

    NGHolder &midfix = *out;

    map<NFAVertex, NFAVertex> v_map;
    v_map[g.start] = midfix.start;
    v_map[g.startDs] = midfix.startDs;

    /* include the lock region */
    assert(picked != info.end());
    auto graph_last = next(picked);

    assert(!graph_last->second.dag);
    assert(graph_last->second.full.size() == 1);

    for (auto jt = graph_last; ; --jt) {
        DEBUG_PRINTF("adding r %u to midfix\n", jt->first);

        /* add all vertices in region, create mapping */
        for (auto v : jt->second.full) {
            DEBUG_PRINTF("adding v %zu to midfix\n", g[v].index);
            if (contains(v_map, v)) {
                continue;
            }

            /* treat all virtual starts as happening anywhere, so that the
             * virtual start is not counted as part of the SoM */
            if (is_virtual_start(v, g)) {
                v_map[v] = midfix.startDs;
                continue;
            }

            NFAVertex vnew = add_vertex(g[v], midfix);
            v_map[v] = vnew;
        }

        /* add edges leaving region verts based on mapping */
        for (auto v : jt->second.full) {
            NFAVertex u = v_map[v];
            for (auto w : adjacent_vertices_range(v, g)) {
                if (w == g.accept || w == g.acceptEod) {
                    add_edge_if_not_present(u, midfix.accept, midfix);
                    continue;
                }
                if (!contains(v_map, w)) {
                    add_edge_if_not_present(u, midfix.accept, midfix);
                } else {
                    add_edge_if_not_present(u, v_map[w], midfix);
                }
            }
        }

        if (jt == info.begin()) {
            break;
        }
    }

    /* add edges from startds to the enters of all the initial optional
     * regions and the first mandatory region. */
    for (auto jt = info.begin(); ; ++jt) {
        for (auto enter : jt->second.enters) {
            assert(contains(v_map, enter));
            NFAVertex v = v_map[enter];
            add_edge_if_not_present(midfix.startDs, v, midfix);
        }

        if (!jt->second.optional) {
            break;
        }

        if (jt == graph_last) {
            /* all regions are optional - add a direct edge to accept */
            add_edge_if_not_present(midfix.startDs, midfix.accept, midfix);
            break;
        }
    }

    assert(in_degree(midfix.accept, midfix));
    renumber_vertices(midfix);
}

static
void fillRoughMidfix(NGHolder *out, const NGHolder &g,
                     const unordered_map<NFAVertex, u32> &regions,
                     const map<u32, region_info> &info,
                     map<u32, region_info>::const_iterator picked) {
    /* as we are not the first prefix, we are probably not acyclic. We need to
     * generate an acyclic holder to acts a fake prefix to sentClearsTail.
     * This will result in a more conservative estimate. */
    /* NOTE: This is not appropriate for firstMatchIsFirst */
    NGHolder &midfix = *out;
    add_edge(midfix.startDs, midfix.accept, midfix);

    map<NFAVertex, NFAVertex> v_map;

    map<u32, region_info>::const_iterator jt = picked;
    for (; jt->second.dag; --jt) {
        DEBUG_PRINTF("adding r %u to midfix\n", jt->first);
        if (!jt->second.optional) {
            clear_out_edges(midfix.startDs, midfix);
            add_edge(midfix.startDs, midfix.startDs, midfix);
        }

        /* add all vertices in region, create mapping */
        for (auto v : jt->second.full) {
            DEBUG_PRINTF("adding v %zu to midfix\n", g[v].index);
            NFAVertex vnew = add_vertex(g[v], midfix);
            v_map[v] = vnew;
        }

        /* add edges leaving region verts based on mapping */
        for (auto v : jt->second.full) {
            NFAVertex u = v_map[v];
            for (auto w : adjacent_vertices_range(v, g)) {
                if (w == g.accept || w == g.acceptEod) {
                    continue;
                }
                if (!contains(v_map, w)) {
                    add_edge_if_not_present(u, midfix.accept, midfix);
                } else {
                    add_edge_if_not_present(u, v_map[w], midfix);
                }
            }
        }

        /* add edges from startds to enters */
        for (auto enter : jt->second.enters) {
            assert(contains(v_map, enter));
            NFAVertex v = v_map[enter];
            add_edge(midfix.startDs, v, midfix);
        }

        if (jt == info.begin()) {
            break;
        }
    }

    /* we can include the exits of the regions leading in */
    if (!jt->second.dag) {
        u32 first_early_region = jt->first;
        clear_out_edges(midfix.startDs, midfix);
        add_edge(midfix.startDs, midfix.startDs, midfix);

        do {
            for (auto v : jt->second.exits) {
                DEBUG_PRINTF("adding v %zu to midfix\n", g[v].index);
                NFAVertex vnew = add_vertex(g[v], midfix);
                v_map[v] = vnew;

                /* add edges from startds to new vertices */
                add_edge(midfix.startDs, vnew, midfix);
            }

            /* add edges leaving region verts based on mapping */
            for (auto v : jt->second.exits) {
                NFAVertex u = v_map[v];
                for (auto w : adjacent_vertices_range(v, g)) {
                    if (w == g.accept || w == g.acceptEod
                        || regions.at(w) <= first_early_region) {
                        continue;
                    }
                    if (!contains(v_map, w)) {
                        add_edge_if_not_present(u, midfix.accept, midfix);
                    } else {
                        add_edge_if_not_present(u, v_map[w], midfix);
                    }
                }
            }
        } while (jt->second.optional && jt != info.begin() && (jt--)->first);

        if (jt->second.optional) {
            assert(!jt->second.exits.empty());
            NFAVertex v = v_map[jt->second.exits.front()];
            for (auto w : adjacent_vertices_range(v, midfix)) {
                add_edge(midfix.startDs, w, midfix);
            }
        }
    }
}

static
bool beginsWithDotStar(const NGHolder &g) {
    bool hasDot = false;

    // We can ignore the successors of start, as matches that begin there will
    // necessarily have a SOM of 0.

    set<NFAVertex> succ;
    insert(&succ, adjacent_vertices(g.startDs, g));
    succ.erase(g.startDs);

    for (auto v : succ) {
        // We want 'dot' states that aren't virtual starts.
        if (g[v].char_reach.all() &&
                !g[v].assert_flags) {
            hasDot = true;
            set<NFAVertex> dotsucc;
            insert(&dotsucc, adjacent_vertices(v, g));
            if (dotsucc != succ) {
                DEBUG_PRINTF("failed dot-star succ check\n");
                return false;
            }
        }
    }

    if (hasDot) {
        DEBUG_PRINTF("begins with dot-star\n");
    }
    return hasDot;
}

static
bool buildMidfix(NG &ng, const som_plan &item, const u32 som_slot_in,
                 const u32 som_slot_out) {
    assert(item.prefix);
    assert(hasCorrectlyNumberedVertices(*item.prefix));

    /* setup escaper for second som_location if required */
    if (item.escapes.any()) {
        if (!createEscaper(ng, *item.prefix, item.escapes, som_slot_out)) {
            return false;
        }
    }

    /* ensure we copy som from prev loc */
    setMidfixReports(ng.rm, item, som_slot_in, som_slot_out);

    /* add second prefix/1st midfix */
    if (!ng.addHolder(*item.prefix)) {
        DEBUG_PRINTF("---addHolder failed---\n");
        return false;
    }

    return true;
}

static
bool isMandRegionBetween(map<u32, region_info>::const_iterator a,
                         map<u32, region_info>::const_iterator b) {
    while (b != a) {
        if (!b->second.optional) {
            return true;
        }
        --b;
    }

    return false;
}

// Attempts to advance the current plan. Returns true if we advance to the end
// (woot!); updates picked, plan and bad_region.
static
bool advancePlan(const NGHolder &g,
                 const unordered_map<NFAVertex, u32> &regions,
                 const NGHolder &prefix, bool stuck,
                 map<u32, region_info>::const_iterator &picked,
                 const map<u32, region_info>::const_iterator furthest,
                 const map<u32, region_info>::const_iterator furthest_lock,
                 const CharReach &next_escapes, som_plan &plan,
                 u32 *bad_region) {
    u32 bad_region_r = 0;
    u32 bad_region_x = 0;
    u32 bad_region_e = 0;
    DEBUG_PRINTF("curr %u\n", picked->first);

    if (sentClearsTail(g, regions, prefix, furthest->first, &bad_region_r)) {
        plan.is_reset = true;
        picked = furthest;
        DEBUG_PRINTF("Prefix clears tail, woot!\n");
        return true;
    } else {
        DEBUG_PRINTF("Reset failed, first bad region %u\n", bad_region_r);
    }

    if (stuck) {
        u32 to_region = furthest_lock->first;
        if (validateXSL(g, regions, to_region, next_escapes, &bad_region_x)) {
            DEBUG_PRINTF("XSL\n");
            picked = furthest_lock;
            plan.escapes = next_escapes;
            return true;
        } else {
            DEBUG_PRINTF("XSL failed, first bad region %u\n", bad_region_x);
        }

        if (validateEXSL(g, regions, to_region, next_escapes, prefix,
                         &bad_region_e)) {
            DEBUG_PRINTF("EXSL\n");
            picked = furthest_lock;
            plan.escapes = next_escapes;
            return true;
        } else {
            DEBUG_PRINTF("EXSL failed, first bad region %u\n", bad_region_e);
        }
    } else {
        DEBUG_PRINTF("!stuck, skipped XSL and EXSL\n");
    }

    assert(!plan.is_reset);

    *bad_region = max(bad_region_x, bad_region_e);
    if (bad_region_r >= *bad_region) {
        *bad_region = bad_region_r;
        plan.is_reset = true;
        plan.escapes.clear();
        picked = furthest;
    } else {
        picked = furthest_lock;
        plan.escapes = next_escapes;
    }

    DEBUG_PRINTF("first bad region now %u\n", *bad_region);
    return false;
}

static
bool addPlan(vector<som_plan> &plan, u32 parent) {
    DEBUG_PRINTF("adding plan %zu with parent %u\n", plan.size(),
                 parent);

    if (plan.size() >= MAX_SOM_PLANS) {
        DEBUG_PRINTF("too many plans!\n");
        return false;
    }

    plan.emplace_back(nullptr, CharReach(), false, parent);
    return true;
}

// Fetches all preds of {accept, acceptEod} for this graph.
static
void addReporterVertices(const NGHolder &g, vector<NFAVertex> &reporters) {
    set<NFAVertex> tmp;
    insert(&tmp, inv_adjacent_vertices(g.accept, g));
    insert(&tmp, inv_adjacent_vertices(g.acceptEod, g));
    tmp.erase(g.accept);

#ifdef DEBUG
    DEBUG_PRINTF("add reporters:");
    for (UNUSED auto v : tmp) {
        printf(" %zu", g[v].index);
    }
    printf("\n");
#endif

    reporters.insert(reporters.end(), tmp.begin(), tmp.end());
}

// Fetches all preds of {accept, acceptEod} in this region.
static
void addReporterVertices(const region_info &r, const NGHolder &g,
                         vector<NFAVertex> &reporters) {
    for (auto v : r.exits) {
        if (edge(v, g.accept, g).second || edge(v, g.acceptEod, g).second) {
            DEBUG_PRINTF("add reporter %zu\n", g[v].index);
            reporters.push_back(v);
        }
    }
}

// Fetches the mappings of all preds of {accept, acceptEod} in this region.
static
void addMappedReporterVertices(const region_info &r, const NGHolder &g,
                        const unordered_map<NFAVertex, NFAVertex> &mapping,
                        vector<NFAVertex> &reporters) {
    for (auto v : r.exits) {
        if (edge(v, g.accept, g).second || edge(v, g.acceptEod, g).second) {
            DEBUG_PRINTF("adding v=%zu\n", g[v].index);
            auto it = mapping.find(v);
            assert(it != mapping.end());
            reporters.push_back(it->second);
        }
    }
}

// Clone a version of the graph, but only including the in-edges of `enter'
// from earlier regions.
static
void cloneGraphWithOneEntry(NGHolder &out, const NGHolder &g,
                       const unordered_map<NFAVertex, u32> &regions,
                       NFAVertex entry, const vector<NFAVertex> &enters,
                       unordered_map<NFAVertex, NFAVertex> &orig_to_copy) {
    orig_to_copy.clear();
    cloneHolder(out, g, &orig_to_copy);

    assert(contains(orig_to_copy, entry));
    const u32 region = regions.at(entry);

    for (auto v : enters) {
        if (v == entry) {
            continue;
        }
        assert(contains(orig_to_copy, v));

        for (auto u : inv_adjacent_vertices_range(v, g)) {
            if (regions.at(u) < region) {
                assert(edge(orig_to_copy[u], orig_to_copy[v], out).second);
                remove_edge(orig_to_copy[u], orig_to_copy[v], out);
            }
        }
    }

    pruneUseless(out);
}

static
void expandGraph(NGHolder &g, unordered_map<NFAVertex, u32> &regions,
                 vector<NFAVertex> &enters) {
    assert(!enters.empty());
    const u32 split_region = regions.at(enters.front());

    vector<NFAVertex> new_enters;

    // Gather the list of vertices in the split region and subsequent regions.
    vector<NFAVertex> tail_vertices;
    for (auto v : vertices_range(g)) {
        if (is_special(v, g) || regions.at(v) < split_region) {
            continue;
        }
        tail_vertices.push_back(v);
    }

    for (auto enter : enters) {
        DEBUG_PRINTF("processing enter %zu\n", g[enter].index);
        map<NFAVertex, NFAVertex> orig_to_copy;

        // Make a copy of all of the tail vertices, storing region info along
        // the way.
        for (auto v : tail_vertices) {
            auto v2 = clone_vertex(g, v);
            orig_to_copy[v] = v2;
            regions[v2] = regions.at(v);
        }

        // Wire up the edges: edges from previous regions come from the
        // original vertices, while edges internal to and beyond the split
        // region go to the copies.

        for (const auto &m : orig_to_copy) {
            NFAVertex v = m.first, v2 = m.second;

            for (const auto &e : out_edges_range(v, g)) {
                NFAVertex t = target(e, g);
                u32 t_region = regions.at(t);
                if (t_region >= split_region && !is_special(t, g)) {
                    assert(contains(orig_to_copy, t));
                    t = orig_to_copy[t];
                }
                add_edge_if_not_present(v2, t, g[e], g);
            }

            for (const auto &e : in_edges_range(v, g)) {
                NFAVertex u = source(e, g);
                if (regions.at(u) >= split_region && !is_special(u, g)) {
                    assert(contains(orig_to_copy, u));
                    u = orig_to_copy[u];
                }
                add_edge_if_not_present(u, v2, g[e], g);
            }

        }

        // Clear the in-edges from earlier regions of the OTHER enters for this
        // copy of the split region.
        for (auto v : enters) {
            if (v == enter) {
                continue;
            }

            remove_in_edge_if(orig_to_copy[v],
                              [&](const NFAEdge &e) {
                                    NFAVertex u = source(e, g);
                                    return regions.at(u) < split_region;
                              }, g);
        }

        new_enters.push_back(orig_to_copy[enter]);
    }

    // Remove the original set of tail vertices.
    remove_vertices(tail_vertices, g);
    pruneUseless(g);
    regions = assignRegions(g);

    enters.swap(new_enters);
}

static
bool doTreePlanningIntl(NGHolder &g,
            const unordered_map<NFAVertex, u32> &regions,
            const map<u32, region_info> &info,
            map<u32, region_info>::const_iterator picked, u32 bad_region,
            u32 parent_plan,
            const unordered_map<NFAVertex, NFAVertex> &copy_to_orig,
            vector<som_plan> &plan, const Grey &grey) {
    assert(picked != info.end());

    DEBUG_PRINTF("picked=%u\n", picked->first);
    DEBUG_PRINTF("parent is %u\n", parent_plan);

    map<u32, region_info>::const_iterator furthest;

    bool to_end = false;
    while (!to_end) {
        DEBUG_PRINTF("picked is %u\n", picked->first);
        DEBUG_PRINTF("first bad region now %u\n", bad_region);

        furthest = info.find(bad_region); /* first bad */
        if (furthest == info.end()) {
            DEBUG_PRINTF("no partition\n");
            return false;
        }
        --furthest; /* last region we can establish som for */

        if (furthest->first <= picked->first) {
            DEBUG_PRINTF("failed to make any progress\n");
            return false;
        }

        map<u32, region_info>::const_iterator furthest_lock = furthest;
        CharReach next_escapes;
        bool lock_found;
        /* The last possible lock in the range that we examine should be the
         * best. If the previous plan is a lock, this follow as any early lock
         * must have a reach that is a subset of the last plan's lock. If the
         * last plan is a resetting plan ..., ?is this true? */
        do {
            lock_found = isPossibleLock(g, furthest_lock, info,
                                          &next_escapes);
        } while (!lock_found && (--furthest_lock)->first > picked->first);
        DEBUG_PRINTF("lock possible? %d\n", (int)lock_found);

        if (lock_found && !isMandRegionBetween(picked, furthest_lock)) {
            lock_found = false;
        }

        if (!isMandRegionBetween(picked, furthest)) {
            return false;
        }

        /* There is no certainty that the som at a reset location will always
         * go forward */
        if (plan[parent_plan].is_reset && lock_found) {
            NGHolder midfix;
            DEBUG_PRINTF("checking if midfix is suitable for lock\n");
            fillHolderForLockCheck(&midfix, g, info, furthest_lock);

            if (!firstMatchIsFirst(midfix)) {
                DEBUG_PRINTF("not stuck\n");
                lock_found = false;
            }
        }

        if (!addPlan(plan, parent_plan)) {
            return false;
        }

        to_end = false;

        if (lock_found && next_escapes.none()) {
            picked = furthest_lock;
            to_end = true;
        }

        if (!to_end) {
            NGHolder conservative_midfix; /* for use in reset, exsl analysis */
            fillRoughMidfix(&conservative_midfix, g, regions, info, furthest);
            dumpHolder(conservative_midfix, 15, "som_pathmidfix", grey);

            u32 old_bad_region = bad_region;
            to_end = advancePlan(g, regions, conservative_midfix, lock_found,
                                 picked, furthest, furthest_lock, next_escapes,
                                 plan.back(), &bad_region);
            if (!to_end
                && bad_region <= old_bad_region) { /* we failed to progress */
                DEBUG_PRINTF("failed to make any progress\n");
                return false;
            }
        }

        /* handle direct edge to accepts from region */
        if (edge(furthest->second.exits.front(), g.accept, g).second
                || edge(furthest->second.exits.front(), g.acceptEod, g).second) {
            map<u32, region_info>::const_iterator it = furthest;
            do {
                addMappedReporterVertices(it->second, g, copy_to_orig,
                                          plan.back().reporters_in);
            } while (it != info.begin() && it->second.optional && (it--)->first);
        }

        /* create second prefix */
        plan.back().prefix = makePrefix(g, regions, furthest->second,
                                        next(furthest)->second);
        parent_plan = plan.size() - 1;
    }

    // The last region contributes reporters. If it's optional, the regions
    // before it do as well.
    map<u32, region_info>::const_reverse_iterator it = info.rbegin();
    do {
        DEBUG_PRINTF("add mapped reporters for region %u\n", it->first);
        addMappedReporterVertices(it->second, g, copy_to_orig,
                                  plan.back().reporters);
    } while (it->second.optional && it != info.rend() &&
             (++it)->first > furthest->first);

    return true;
}

static
bool doTreePlanning(NGHolder &g,
                    map<u32, region_info>::const_iterator presplit,
                    map<u32, region_info>::const_iterator picked,
                    vector<som_plan> &plan, const Grey &grey) {
    DEBUG_PRINTF("picked is %u\n", picked->first);
    DEBUG_PRINTF("presplit is %u\n", presplit->first);

    map<u32, region_info>::const_iterator splitter = next(presplit);
    vector<NFAVertex> enters = splitter->second.enters; // mutable copy
    DEBUG_PRINTF("problem region has %zu entry vertices\n", enters.size());

    if (enters.size() <= 1) {
        // TODO: Splitting a region with one entry won't get us anywhere, but
        // it shouldn't create buggy analyses either. See UE-1892.
        DEBUG_PRINTF("nothing to split\n");
        return false;
    }

    if (plan.size() + enters.size() > MAX_SOM_PLANS) {
        DEBUG_PRINTF("splitting this tree would hit the plan limit.\n");
        return false;
    }

    assert(!plan.empty());
    const u32 parent_plan = plan.size() - 1;

    // Make a copy of the graph, with the subgraph under each enter vertex
    // duplicated without the edges into the other enter vertices.
    // NOTE WELL: this will invalidate 'info' from the split point, but it's
    // OK... we don't use it after this.
    auto g_regions = assignRegions(g);
    expandGraph(g, g_regions, enters);
    dumpHolder(g, g_regions, 14, "som_expandedtree", grey);

    for (auto v : enters) {
        DEBUG_PRINTF("enter %zu\n", g[v].index);

        // For this entry vertex, construct a version of the graph without the
        // other entries in this region (g_path), and calculate its depths and
        // regions.

        NGHolder g_path;
        unordered_map<NFAVertex, NFAVertex> orig_to_copy;
        cloneGraphWithOneEntry(g_path, g, g_regions, v, enters, orig_to_copy);
        auto regions = assignRegions(g_path);
        dumpHolder(g_path, regions, 14, "som_treepath", grey);

        map<u32, region_info> path_info;
        buildRegionMapping(g_path, regions, path_info);

        // Translate 'picked' to the corresponding region iterator over the
        // g_path graph. we can't trust the numbering, so we use a vertex
        // instead.
        NFAVertex picked_v = picked->second.enters.front();
        assert(contains(orig_to_copy, picked_v));
        u32 picked_region = regions.at(orig_to_copy[picked_v]);
        map<u32, region_info>::const_iterator path_pick =
            path_info.find(picked_region);
        if (path_pick == path_info.end()) {
            assert(0); // odd
            return false;
        }

        // Similarly, find our bad_region.
        assert(contains(orig_to_copy, v));
        u32 bad_region = regions.at(orig_to_copy[v]);

        // It's possible that the region may have grown to include its
        // successors, in which case we (currently) run screaming. Just
        // checking the size should be sufficient here.
        if (picked->second.full.size() != path_pick->second.full.size()) {
            DEBUG_PRINTF("picked region has grown, bailing\n");
            return false;
        }

        // Construct reverse mapping from vertices in g_path to g.
        unordered_map<NFAVertex, NFAVertex> copy_to_orig;
        for (const auto &m : orig_to_copy) {
            copy_to_orig.insert(make_pair(m.second, m.first));
        }

        bool to_end = doTreePlanningIntl(g_path, regions, path_info, path_pick,
                                         bad_region, parent_plan,
                                         copy_to_orig, plan, grey);
        if (!to_end) {
            return false;
        }
    }

    return true;
}

enum dsp_behaviour {
    ALLOW_MODIFY_HOLDER,
    DISALLOW_MODIFY_HOLDER /* say no to tree planning */
};

static
bool doSomPlanning(NGHolder &g, bool stuck_in,
                   const unordered_map<NFAVertex, u32> &regions,
                   const map<u32, region_info> &info,
                   map<u32, region_info>::const_iterator picked,
                   vector<som_plan> &plan,
                   const Grey &grey,
                   dsp_behaviour behaviour = ALLOW_MODIFY_HOLDER) {
    DEBUG_PRINTF("in picked is %u\n", picked->first);

    /* Need to verify how far the lock covers */
    u32 bad_region;
    NGHolder *ap_pref = plan.back().prefix.get();
    NGHolder ap_temp;
    if (hasBigCycles(*ap_pref)) {
        fillRoughMidfix(&ap_temp, g, regions, info, picked);
        ap_pref = &ap_temp;
    }

    bool to_end = advancePlan(g, regions, *ap_pref, stuck_in, picked,
                              picked, picked, plan.back().escapes,
                              plan.back(), &bad_region);

    if (to_end) {
        DEBUG_PRINTF("advanced through the whole graph in one go!\n");
        addReporterVertices(g, plan.back().reporters);
        return true;
    }

    map<u32, region_info>::const_iterator prev_furthest = picked;
    map<u32, region_info>::const_iterator furthest;

    furthest = info.find(bad_region); /* first bad */
    if (furthest == info.begin() || furthest == info.end()) {
        DEBUG_PRINTF("no partition\n");
        return false;
    }
    --furthest; /* last region we can establish som for */

    if (furthest->first <= picked->first) {
    do_tree:
        /* unable to establish SoM past the last picked region */
        if (behaviour == DISALLOW_MODIFY_HOLDER) {
            /* tree planning mutates the graph */
            return false;
        }

        DEBUG_PRINTF("failed to make any progress\n");
        assert(!plan.empty());
        if (plan.size() == 1) {
            DEBUG_PRINTF("not handling initial alternations yet\n");
            return false;
        }
        plan.pop_back();
        return doTreePlanning(g, furthest, prev_furthest, plan, grey);
    }

    furthest = picked;
    while (!to_end) {
        prev_furthest = furthest;

        DEBUG_PRINTF("prev further is %u\n", prev_furthest->first);
        DEBUG_PRINTF("first bad region now %u\n", bad_region);

        furthest = info.find(bad_region); /* first bad */
        if (furthest == info.begin() || furthest == info.end()) {
            DEBUG_PRINTF("no partition\n");
            return false;
        }
        --furthest; /* last region we can establish som for */

        map<u32, region_info>::const_iterator furthest_lock = furthest;
        CharReach next_escapes;
        bool stuck;
        do {
            stuck = isPossibleLock(g, furthest_lock, info, &next_escapes);
        } while (!stuck && (--furthest_lock)->first > prev_furthest->first);
        DEBUG_PRINTF("lock possible? %d\n", (int)stuck);
        DEBUG_PRINTF("furthest_lock=%u\n", furthest_lock->first);

        if (stuck && !isMandRegionBetween(prev_furthest, furthest_lock)) {
            stuck = false;
        }

        if (!isMandRegionBetween(prev_furthest, furthest)) {
            DEBUG_PRINTF("no mand region between %u and %u\n",
                         prev_furthest->first, furthest->first);
            return false;
        }

        /* There is no certainty that the som at a reset location will always
         * go forward */
        if (plan.back().is_reset && stuck) {
            NGHolder midfix;
            fillHolderForLockCheck(&midfix, g, info, furthest_lock);

            DEBUG_PRINTF("checking if midfix is suitable for lock\n");
            if (!firstMatchIsFirst(midfix)) {
                DEBUG_PRINTF("not stuck\n");
                stuck = false;
            }
        }

        assert(!plan.empty());
        if (!addPlan(plan, plan.size() - 1)) {
            return false;
        }

        to_end = false;

        if (stuck && next_escapes.none()) {
            picked = furthest_lock;
            to_end = true;
        }

        if (!to_end) {
            NGHolder conservative_midfix; /* for use in reset, exsl analysis */
            fillRoughMidfix(&conservative_midfix, g, regions, info, furthest);

            u32 old_bad_region = bad_region;
            to_end = advancePlan(g, regions, conservative_midfix, stuck, picked,
                                 furthest, furthest_lock, next_escapes,
                                 plan.back(), &bad_region);

            if (!to_end
                && bad_region <= old_bad_region) { /* we failed to progress */
                goto do_tree;
            }
        }

        /* handle direct edge to accepts from region */
        if (edge(furthest->second.exits.front(), g.accept, g).second
            || edge(furthest->second.exits.front(), g.acceptEod, g).second) {
            map<u32, region_info>::const_iterator it = furthest;
            do {
                DEBUG_PRINTF("direct edge to accept from region %u\n",
                             it->first);
                addReporterVertices(it->second, g, plan.back().reporters_in);
            } while (it != info.begin() && it->second.optional
                     && (it--)->first);
        }

        /* create second prefix */
        plan.back().prefix = makePrefix(g, regions, furthest->second,
                                        next(furthest)->second);
    }
    DEBUG_PRINTF("(final) picked is %u\n", picked->first);

    // The last region contributes reporters. If it's optional, the regions
    // before it do as well.
    map<u32, region_info>::const_reverse_iterator it = info.rbegin();
    do {
        DEBUG_PRINTF("region %u contributes reporters to last plan\n",
                     it->first);
        addReporterVertices(it->second, g, plan.back().reporters);
    } while (it->second.optional && it != info.rend() &&
             (++it)->first > furthest->first);

    DEBUG_PRINTF("done!\n");
    return true;
}

static
void dumpSomPlan(UNUSED const NGHolder &g, UNUSED const som_plan &p,
                 UNUSED size_t num) {
#if defined(DEBUG) || defined(DUMP_PLANS)
    DEBUG_PRINTF("plan %zu: prefix=%p, escapes=%s, is_reset=%d, "
                 "parent=%u\n",
                 num, p.prefix.get(),
                 describeClass(p.escapes, 20, CC_OUT_TEXT).c_str(),
                 p.is_reset, p.parent);
    printf("  reporters:");
    for (auto v : p.reporters) {
        printf(" %zu", g[v].index);
    }
    printf("\n");
    printf("  reporters_in:");
    for (auto v : p.reporters_in) {
        printf(" %zu", g[v].index);
    }
    printf("\n");
#endif
}

/**
 * Note: if we fail to build a midfix/ng.addHolder, we throw a pattern too
 * large exception as (1) if previous ng modification have been applied (other
 * midfixes have been applied), ng will be an undefined state on return and (2)
 * if the head of a pattern cannot be implemented we are generally unable to
 * implement the full pattern.
 */
static
void implementSomPlan(NG &ng, const ExpressionInfo &expr, u32 comp_id,
                      NGHolder &g, vector<som_plan> &plan,
                      const u32 first_som_slot) {
    ReportManager &rm = ng.rm;
    SomSlotManager &ssm = ng.ssm;

    DEBUG_PRINTF("%zu plans\n", plan.size());
    assert(plan.size() <= MAX_SOM_PLANS);
    assert(!plan.empty());

    vector<u32> som_slots(plan.size());
    som_slots[0] = first_som_slot;

    // Root plan, which already has a SOM slot assigned (first_som_slot).
    dumpSomPlan(g, plan.front(), 0);
    dumpSomSubComponent(*plan.front().prefix, "04_som", expr.index, comp_id, 0,
                        ng.cc.grey);
    assert(plan.front().prefix);
    if (plan.front().escapes.any() && !plan.front().is_reset) {
        /* setup escaper for first som location */
        if (!createEscaper(ng, *plan.front().prefix, plan.front().escapes,
                           first_som_slot)) {
            throw CompileError(expr.index, "Pattern is too large.");
        }
    }

    assert(plan.front().reporters_in.empty());
    updateReportToUseRecordedSom(rm, g, plan.front().reporters, first_som_slot);

    // Tree of plans, encoded in a vector.
    vector<som_plan>::const_iterator it = plan.begin();
    for (++it; it != plan.end(); ++it) {
        const u32 plan_num = it - plan.begin();
        dumpSomPlan(g, *it, plan_num);
        dumpSomSubComponent(*it->prefix, "04_som", expr.index, comp_id,
                            plan_num, ng.cc.grey);

        assert(it->parent < plan_num);
        u32 som_slot_in = som_slots[it->parent];
        u32 som_slot_out = ssm.getSomSlot(*it->prefix, it->escapes,
                                          it->is_reset, som_slot_in);
        som_slots[plan_num] = som_slot_out;

        assert(!it->no_implement);
        if (!buildMidfix(ng, *it, som_slot_in, som_slot_out)) {
            throw CompileError(expr.index, "Pattern is too large.");
        }
        updateReportToUseRecordedSom(rm, g, it->reporters_in, som_slot_in);
        updateReportToUseRecordedSom(rm, g, it->reporters, som_slot_out);
    }

    /* create prefix to set the som_loc */
    if (!plan.front().no_implement) {
        renumber_vertices(*plan.front().prefix);
        assert(plan.front().prefix->kind == NFA_OUTFIX);
        if (!ng.addHolder(*plan.front().prefix)) {
            throw CompileError(expr.index, "Pattern is too large.");
        }
    }
}

static
void anchorStarts(NGHolder &g) {
    vector<NFAEdge> dead;
    for (const auto &e : out_edges_range(g.startDs, g)) {
        NFAVertex v = target(e, g);
        if (v == g.startDs) {
            continue;
        }
        add_edge_if_not_present(g.start, v, g[e], g);
        dead.push_back(e);
    }
    remove_edges(dead, g);
}

static
void setZeroReports(NGHolder &g) {
    set<NFAVertex> acceptors;
    insert(&acceptors, inv_adjacent_vertices(g.accept, g));
    insert(&acceptors, inv_adjacent_vertices(g.acceptEod, g));
    acceptors.erase(g.accept);

    for (auto v : vertices_range(g)) {
        auto &reports = g[v].reports;
        reports.clear();

        if (!contains(acceptors, v)) {
            continue;
        }

        // We use the report ID to store the offset adjustment used for virtual
        // starts.

        if (g[v].assert_flags & POS_FLAG_VIRTUAL_START) {
            reports.insert(1);
        } else {
            reports.insert(0);
        }
    }
}

/* updates the reports on all vertices leading to the sink */
static
void makeSomRevNfaReports(ReportManager &rm, NGHolder &g, NFAVertex sink,
                          const ReportID report, const u32 comp_id) {
    // Construct replacement report.
    Report ir = rm.getReport(report);
    ir.type = EXTERNAL_CALLBACK_SOM_REV_NFA;
    ir.revNfaIndex = comp_id;
    ReportID new_report = rm.getInternalId(ir);

    for (auto v : inv_adjacent_vertices_range(sink, g)) {
        if (v == g.accept) {
            continue;
        }

        auto &r = g[v].reports;
        if (contains(r, report)) {
            r.erase(report);
            r.insert(new_report);
        }
    }
}

static
void clearProperInEdges(NGHolder &g, const NFAVertex sink) {
    vector<NFAEdge> dead;
    for (const auto &e : in_edges_range(sink, g)) {
        if (source(e, g) == g.accept) {
            continue;
        }
        dead.push_back(e);
    }

    if (dead.empty()) {
        return;
    }

    remove_edges(dead, g);
    pruneUseless(g, false);
}

namespace {
struct SomRevNfa {
    SomRevNfa(NFAVertex s, ReportID r, bytecode_ptr<NFA> n)
        : sink(s), report(r), nfa(move(n)) {}
    NFAVertex sink;
    ReportID report;
    bytecode_ptr<NFA> nfa;
};
}

static
bytecode_ptr<NFA> makeBareSomRevNfa(const NGHolder &g,
                                    const CompileContext &cc) {
    // Create a reversed anchored version of this NFA which fires a zero report
    // ID on accept.
    NGHolder g_rev;
    reverseHolder(g, g_rev);
    anchorStarts(g_rev);
    setZeroReports(g_rev);

    // Prep for actual construction.
    renumber_vertices(g_rev);
    g_rev.kind = NFA_REV_PREFIX;
    reduceGraphEquivalences(g_rev, cc);
    removeRedundancy(g_rev, SOM_NONE);

    DEBUG_PRINTF("building a rev NFA with %zu vertices\n", num_vertices(g_rev));

    auto nfa = constructReversedNFA(g_rev, cc);
    if (!nfa) {
        return nfa;
    }

    // Set some useful properties.
    depth maxWidth = findMaxWidth(g);
    if (maxWidth.is_finite()) {
        nfa->maxWidth = (u32)maxWidth;
    } else {
        nfa->maxWidth = 0;
    }
    depth minWidth = findMinWidth(g);
    nfa->minWidth = (u32)minWidth;

    return nfa;
}

static
bool makeSomRevNfa(vector<SomRevNfa> &som_nfas, const NGHolder &g,
                   const ReportID report, const NFAVertex sink,
                   const CompileContext &cc) {
    // Clone the graph with ONLY the given report vertices on the given sink.
    NGHolder g2;
    cloneHolder(g2, g);
    clearProperInEdges(g2, sink == g.accept ? g2.acceptEod : g2.accept);
    pruneAllOtherReports(g2, report);

    if (in_degree(g2.accept, g2) == 0 && in_degree(g2.acceptEod, g2) == 1) {
        DEBUG_PRINTF("no work to do for this sink\n");
        return true;
    }

    renumber_vertices(g2); // for findMinWidth, findMaxWidth.

    auto nfa = makeBareSomRevNfa(g2, cc);
    if (!nfa) {
        DEBUG_PRINTF("couldn't build rev nfa\n");
        return false;
    }

    som_nfas.emplace_back(sink, report, move(nfa));
    return true;
}

static
bool doSomRevNfa(NG &ng, NGHolder &g, const CompileContext &cc) {
    ReportManager &rm = ng.rm;

    // FIXME might want to work on a graph without extra redundancy?
    depth maxWidth = findMaxWidth(g);
    DEBUG_PRINTF("maxWidth=%s\n", maxWidth.str().c_str());

    if (maxWidth > depth(ng.maxSomRevHistoryAvailable)) {
        DEBUG_PRINTF("too wide\n");
        return false;
    }

    set<ReportID> reports = all_reports(g);
    DEBUG_PRINTF("%zu reports\n", reports.size());

    // We distinguish between reports and accept/acceptEod sinks in order to
    // correctly handle cases which do different things on eod/normal accepts.
    // Later, it might be more elegant to do this with a single NFA and
    // multi-tops.

    vector<SomRevNfa> som_nfas;

    for (auto report : reports) {
        if (!makeSomRevNfa(som_nfas, g, report, g.accept, cc)) {
            return false;
        }
        if (!makeSomRevNfa(som_nfas, g, report, g.acceptEod, cc)) {
            return false;
        }
    }

    for (auto &som_nfa : som_nfas) {
        assert(som_nfa.nfa);

        // Transfer ownership of the NFA to the SOM slot manager.
        u32 comp_id = ng.ssm.addRevNfa(move(som_nfa.nfa), maxWidth);

        // Replace this report on 'g' with a SOM_REV_NFA report pointing at our
        // new component.
        makeSomRevNfaReports(rm, g, som_nfa.sink, som_nfa.report, comp_id);
    }

    if (ng.cc.streaming) {
        assert(ng.ssm.somHistoryRequired() <=
               max(cc.grey.maxHistoryAvailable, ng.maxSomRevHistoryAvailable));
    }

    return true;
}

static
u32 doSomRevNfaPrefix(NG &ng, const ExpressionInfo &expr, NGHolder &g,
                      const CompileContext &cc) {
    depth maxWidth = findMaxWidth(g);

    assert(maxWidth <= depth(ng.maxSomRevHistoryAvailable));
    assert(all_reports(g).size() == 1);

    auto nfa = makeBareSomRevNfa(g, cc);
    if (!nfa) {
        throw CompileError(expr.index, "Pattern is too large.");
    }

    if (ng.cc.streaming) {
        assert(ng.ssm.somHistoryRequired() <=
               max(cc.grey.maxHistoryAvailable, ng.maxSomRevHistoryAvailable));
    }

    return ng.ssm.addRevNfa(move(nfa), maxWidth);
}

static
bool is_literable(const NGHolder &g, NFAVertex v) {
    const CharReach &cr = g[v].char_reach;
    return cr.count() == 1 || cr.isCaselessChar();
}

static
void append(ue2_literal &s, const CharReach &cr) {
    assert(cr.count() == 1 || cr.isCaselessChar());
    s.push_back(cr.find_first(), cr.isCaselessChar());
}

static
map<u32, region_info>::const_iterator findLaterLiteral(const NGHolder &g,
                            const map<u32, region_info> &info,
                            map<u32, region_info>::const_iterator lower_bound,
                            ue2_literal &s_out, const Grey &grey) {
#define MIN_LITERAL_LENGTH 3
    s_out.clear();
    bool past_lower = false;
    ue2_literal s;
    map<u32, region_info>::const_iterator it;
    for (it = info.begin(); it != info.end(); ++it) {
        if (it == lower_bound) {
            past_lower = true;
        }
        if (!it->second.optional && it->second.dag
            && it->second.full.size() == 1
            && is_literable(g, it->second.full.front())) {
            append(s, g[it->second.full.front()].char_reach);

            if (s.length() >= grey.maxHistoryAvailable && past_lower) {
                goto exit;
            }
        } else {
            if (past_lower && it != lower_bound
                && s.length() >= MIN_LITERAL_LENGTH) {
                --it;
                goto exit;
            }
            s.clear();
        }
    }

    if (past_lower && it != lower_bound && s.length() >= MIN_LITERAL_LENGTH) {
        --it;
        s_out = s;
        return it;
    }
 exit:
    if (s.length() > grey.maxHistoryAvailable) {
        ue2_literal::const_iterator jt = s.end() - grey.maxHistoryAvailable;
        for (; jt != s.end(); ++jt) {
            s_out.push_back(*jt);
        }
    } else {
        s_out = s;
    }
    return it;
}

static
bool attemptToBuildChainAfterSombe(SomSlotManager &ssm, NGHolder &g,
                  const unordered_map<NFAVertex, u32> &regions,
                  const map<u32, region_info> &info,
                  map<u32, region_info>::const_iterator picked,
                  const Grey &grey,
                  vector<som_plan> *plan) {
    DEBUG_PRINTF("trying to chain from %u\n", picked->first);
    const u32 numSomLocsBefore = ssm.numSomSlots(); /* for rollback */

    shared_ptr<NGHolder> prefix = makePrefix(g, regions, picked->second,
                                             next(picked)->second);

    // Quick check to stop us from trying this on huge graphs, which causes us
    // to spend forever in ng_execute looking at cases that will most like
    // fail. See UE-2078.
    size_t prefix_size = num_vertices(*prefix);
    size_t total_size = num_vertices(g);
    assert(total_size >= prefix_size);
    if (total_size - prefix_size > MAX_SOMBE_CHAIN_VERTICES) {
        DEBUG_PRINTF("suffix has %zu vertices, fail\n",
                     total_size - prefix_size);
        return false;
    }

    clearReports(*prefix);
    for (auto u : inv_adjacent_vertices_range(prefix->accept, *prefix)) {
        (*prefix)[u].reports.insert(0);
    }

    dumpHolder(*prefix, 0, "full_haiglit_prefix", grey);

    CharReach escapes;
    bool stuck = isPossibleLock(g, picked, info, &escapes);
    if (stuck) {
        NGHolder gg;
        fillHolderForLockCheck(&gg, g, info, picked);

        stuck = firstMatchIsFirst(gg);
    }

    DEBUG_PRINTF("stuck = %d\n", (int)stuck);

    // Note: no-one should ever pay attention to the root plan's som_loc_in.
    plan->emplace_back(prefix, escapes, false, 0);
    plan->back().no_implement = true;

    dumpHolder(*plan->back().prefix, 22, "som_prefix", grey);

    /* don't allow tree planning to mutate the graph */
    if (!doSomPlanning(g, stuck, regions, info, picked, *plan, grey,
                       DISALLOW_MODIFY_HOLDER)) {
        // Rollback SOM locations.
        ssm.rollbackSomTo(numSomLocsBefore);

        DEBUG_PRINTF("fail to chain\n");
        return false;
    }

    return true;
}

static
void setReportOnHaigPrefix(RoseBuild &rose, NGHolder &h) {
    ReportID haig_report_id = rose.getNewNfaReport();
    DEBUG_PRINTF("setting report id of %u\n", haig_report_id);

    clearReports(h);
    for (auto u : inv_adjacent_vertices_range(h.accept, h)) {
        h[u].reports.clear();
        h[u].reports.insert(haig_report_id);
    }
}

static
bool tryHaig(RoseBuild &rose, NGHolder &g,
             const unordered_map<NFAVertex, u32> &regions,
             som_type som, u32 somPrecision,
             map<u32, region_info>::const_iterator picked,
             shared_ptr<raw_som_dfa> *haig, shared_ptr<NGHolder> *haig_prefix,
             const Grey &grey) {
    DEBUG_PRINTF("trying to build a haig\n");
    shared_ptr<NGHolder> prefix = makePrefix(g, regions, picked->second,
                                             next(picked)->second);
    prefix->kind = NFA_PREFIX;
    setReportOnHaigPrefix(rose, *prefix);
    dumpHolder(*prefix, 0, "haig_prefix", grey);
    vector<vector<CharReach> > triggers; /* empty for prefix */
    *haig = attemptToBuildHaig(*prefix, som, somPrecision, triggers, grey);
    if (!*haig) {
        DEBUG_PRINTF("failed to haig\n");
        return false;
    }
    *haig_prefix = prefix;
    return true;
}

static
void roseAddHaigLiteral(RoseBuild &tb, const shared_ptr<NGHolder> &prefix,
                        const shared_ptr<raw_som_dfa> &haig,
                        const ue2_literal &lit, const set<ReportID> &reports) {
    assert(prefix && haig);

    DEBUG_PRINTF("trying to build a sombe from %s\n", dumpString(lit).c_str());

    RoseInGraph ig;
    RoseInVertex s = add_vertex(RoseInVertexProps::makeStart(false), ig);
    RoseInVertex v = add_vertex(RoseInVertexProps::makeLiteral(lit), ig);

    add_edge(s, v, RoseInEdgeProps(prefix, haig, lit.length()), ig);

    assert(!reports.empty());
    RoseInVertex a = add_vertex(RoseInVertexProps::makeAccept(reports), ig);
    add_edge(v, a, RoseInEdgeProps(0U, 0U), ig);

    calcVertexOffsets(ig);

    UNUSED bool rv = tb.addSombeRose(ig);
    assert(rv); // TODO: recover from addRose failure
}

static
sombe_rv doHaigLitSom(NG &ng, NGHolder &g, const ExpressionInfo &expr,
                      u32 comp_id, som_type som,
                      const unordered_map<NFAVertex, u32> &regions,
                      const map<u32, region_info> &info,
                      map<u32, region_info>::const_iterator lower_bound) {
    DEBUG_PRINTF("entry\n");
    assert(g.kind == NFA_OUTFIX);
    const CompileContext &cc = ng.cc;
    ReportManager &rm = ng.rm;
    SomSlotManager &ssm = ng.ssm;

    if (!cc.grey.allowHaigLit) {
        return SOMBE_FAIL;
    }

    const u32 numSomLocsBefore = ssm.numSomSlots(); /* for rollback */
    u32 som_loc = ssm.getPrivateSomSlot();

    if (!checkViolet(rm, g, false, cc) && !isImplementableNFA(g, &rm, cc)) {
        // This is an optimisation: if we can't build a Haig from a portion of
        // the graph, then we won't be able to manage it as an outfix either
        // when we fall back.
        throw CompileError(expr.index, "Pattern is too large.");
    }

    while (1) {
        DEBUG_PRINTF("lower bound is %u\n", lower_bound->first);
        ue2_literal s;
        map<u32, region_info>::const_iterator lit
            = findLaterLiteral(g, info, lower_bound, s, cc.grey);
        if (lit == info.end()) {
            DEBUG_PRINTF("failed to find literal\n");
            ssm.rollbackSomTo(numSomLocsBefore);
            return SOMBE_FAIL;
        }
        DEBUG_PRINTF("test literal: %s [r=%u]\n", dumpString(s).c_str(),
                     lit->first);

        if (s.length() > MAX_MASK2_WIDTH && mixed_sensitivity(s)) {
            DEBUG_PRINTF("long & mixed-sensitivity, Rose can't handle this\n");
            lower_bound = lit;
            ++lower_bound;
            continue;
        }

        shared_ptr<raw_som_dfa> haig;
        shared_ptr<NGHolder> haig_prefix;
        map<u32, region_info>::const_iterator haig_reg = lit;

        if (edge(lit->second.exits.front(), g.acceptEod, g).second) {
            /* TODO: handle */
            ssm.rollbackSomTo(numSomLocsBefore);
            return SOMBE_FAIL;
        }

        advance(haig_reg, -(s32)s.length());

        if (!haig_reg->first && haig_reg->second.full.size() == 2) {
            /* just starts */

            /* TODO: make below assertion true, reset checks could be stronger
             * (12356)
             */
            /* assert(!attemptToBuildChainAfterSombe(ng, g, info, lit, cc.grey,
               &plan)); */

            lower_bound = lit;
            ++lower_bound;
            continue; /* somebody else should have been able to chain */
        }

        bool ok = true;
        set<ReportID> rep;
        if (next(lit) != info.end()) {
            /* non terminal literal */

            /* TODO: handle edges to accept ? */
            vector<som_plan> plan;
            if (edge(lit->second.exits.front(), g.accept, g).second) {
                insert(&rep, g[lit->second.exits.front()].reports);
                remove_edge(lit->second.exits.front(), g.accept, g);
                g[lit->second.exits.front()].reports.clear();

                /* Note: we can mess with the graph as this is the last literal
                 * we will find and on failure the graph will be thrown away */
            }

            ok = attemptToBuildChainAfterSombe(ssm, g, regions, info, lit,
                                               cc.grey, &plan);
            ok = ok && tryHaig(*ng.rose, g, regions, som, ssm.somPrecision(),
                               haig_reg, &haig, &haig_prefix, cc.grey);

            if (!ok) {
                DEBUG_PRINTF(":( going to next attempt\n");
                goto next_try;
            }

            implementSomPlan(ng, expr, comp_id, g, plan, som_loc);

            Report ir = makeCallback(0U, 0);
            assert(!plan.empty());
            if (plan.front().is_reset) {
                ir.type = INTERNAL_SOM_LOC_SET_FROM;
            } else {
                ir.type = INTERNAL_SOM_LOC_SET_FROM_IF_WRITABLE;
            }
            ir.onmatch = som_loc;
            rep.insert(rm.getInternalId(ir));
        } else {
            /* terminal literal */
            ok = tryHaig(*ng.rose, g, regions, som, ssm.somPrecision(), haig_reg,
                         &haig, &haig_prefix, cc.grey);

            /* find report */
            insert(&rep, g[lit->second.exits.front()].reports);

            /* TODO: som_loc is unused */
        }

        if (ok) {
            roseAddHaigLiteral(*ng.rose, haig_prefix, haig, s, rep);
            if (next(lit) != info.end()) {
                return SOMBE_HANDLED_INTERNAL;
            } else {
                ssm.rollbackSomTo(numSomLocsBefore);
                return SOMBE_HANDLED_ALL;
            }
        }
next_try:
        lower_bound = lit;
        ++lower_bound;
    }
    assert(0);
    return SOMBE_FAIL;
}

static
bool leadingLiterals(const NGHolder &g, set<ue2_literal> *lits,
                     set<NFAVertex> *terminals) {
    /* TODO: smarter (topo) */
#define MAX_LEADING_LITERALS 20
    set<NFAVertex> s_succ;
    insert(&s_succ, adjacent_vertices(g.start, g));

    set<NFAVertex> sds_succ;
    insert(&sds_succ, adjacent_vertices(g.startDs, g));

    if (!is_subset_of(s_succ, sds_succ)) {
        DEBUG_PRINTF("not floating\n");
        return false;
    }

    sds_succ.erase(g.startDs);

    map<NFAVertex, vector<ue2_literal> > curr;
    curr[g.startDs].push_back(ue2_literal());

    map<NFAVertex, set<NFAVertex> > seen;
    map<NFAVertex, vector<ue2_literal> > next;

    bool did_expansion = true;
    while (did_expansion) {
        did_expansion = false;
        u32 count = 0;
        assert(!curr.empty());
        for (const auto &m : curr) {
            const NFAVertex u = m.first;
            const vector<ue2_literal> &base = m.second;
            DEBUG_PRINTF("expanding from %zu\n", g[u].index);
            for (auto v : adjacent_vertices_range(u, g)) {
                if (v == g.startDs) {
                    continue;
                }
                if (contains(seen[u], v)) {
                    DEBUG_PRINTF("loop\n");
                    goto skip_to_next_terminal;
                }
                if (is_any_accept(v, g) || is_match_vertex(v, g)) {
                    DEBUG_PRINTF("match\n");
                    goto skip_to_next_terminal;
                }
                if (g[v].char_reach.count() > 2 * MAX_LEADING_LITERALS) {
                    DEBUG_PRINTF("wide\n");
                    goto skip_to_next_terminal;
                }
            }

            for (auto v : adjacent_vertices_range(u, g)) {
                assert(!contains(seen[u], v));
                if (v == g.startDs) {
                    continue;
                }
                insert(&seen[v], seen[u]);
                seen[v].insert(v);
                CharReach cr = g[v].char_reach;
                vector<ue2_literal> &out = next[v];

                DEBUG_PRINTF("expanding to %zu (|| = %zu)\n", g[v].index,
                             cr.count());
                for (size_t c = cr.find_first(); c != CharReach::npos;
                     c = cr.find_next(c)) {
                    bool nocase = ourisalpha(c) && cr.test(mytoupper(c))
                        && cr.test(mytolower(c));

                    if (nocase && (char)c == mytolower(c)) {
                        continue; /* uppercase already handled us */
                    }

                    for (const auto &lit : base) {
                        if (count >= MAX_LEADING_LITERALS) {
                            DEBUG_PRINTF("count %u\n", count);
                            goto exit;
                        }
                        did_expansion = true;
                        out.push_back(lit);
                        out.back().push_back(c, nocase);
                        count++;
                        if (out.back().length() > MAX_MASK2_WIDTH
                            && mixed_sensitivity(out.back())) {
                            goto exit;
                        }

                    }
                }
            }
            if (0) {
            skip_to_next_terminal:
                insert(&next[u], next[u].end(), base);
                count += base.size();
                if (count > MAX_LEADING_LITERALS) {
                    DEBUG_PRINTF("count %u\n", count);
                    goto exit;
                }
            }
        }

        curr.swap(next);
        next.clear();
    };
 exit:;
    for (const auto &m : curr) {
        NFAVertex t = m.first;
        if (t == g.startDs) {
            assert(curr.size() == 1);
            return false;
        }
        assert(!is_special(t, g));
        terminals->insert(t);
        insert(lits, m.second);
    }
    assert(lits->size() <= MAX_LEADING_LITERALS);
    return !lits->empty();
}

static
bool splitOffLeadingLiterals(const NGHolder &g, set<ue2_literal> *lit_out,
                             NGHolder *rhs) {
    DEBUG_PRINTF("looking for a leading literals\n");

    set<NFAVertex> terms;
    if (!leadingLiterals(g, lit_out, &terms)) {
        return false;
    }

    for (UNUSED const auto &lit : *lit_out) {
        DEBUG_PRINTF("literal is '%s' (len %zu)\n", dumpString(lit).c_str(),
                     lit.length());
    }

    /* need to validate that it is a clean split */
    assert(!terms.empty());
    set<NFAVertex> adj_term1;
    insert(&adj_term1, adjacent_vertices(*terms.begin(), g));
    for (auto v : terms) {
        DEBUG_PRINTF("term %zu\n", g[v].index);
        set<NFAVertex> temp;
        insert(&temp, adjacent_vertices(v, g));
        if (temp != adj_term1) {
            DEBUG_PRINTF("bad split\n");
            return false;
        }
    }

    unordered_map<NFAVertex, NFAVertex> rhs_map;
    vector<NFAVertex> pivots;
    insert(&pivots, pivots.end(), adj_term1);
    splitRHS(g, pivots, rhs, &rhs_map);

    assert(is_triggered(*rhs));
    return true;
}

static
void findBestLiteral(const NGHolder &g,
                     const unordered_map<NFAVertex, u32> &regions,
                     ue2_literal *lit_out, NFAVertex *v,
                     const CompileContext &cc) {
    map<u32, region_info> info;
    buildRegionMapping(g, regions, info, false);

    ue2_literal best;
    NFAVertex best_v = NGHolder::null_vertex();

    map<u32, region_info>::const_iterator lit = info.begin();
    while (1) {
        ue2_literal s;
        lit = findLaterLiteral(g, info, lit, s, cc.grey);
        if (lit == info.end()) {
            break;
        }
        DEBUG_PRINTF("test literal: %s [r=%u]\n", dumpString(s).c_str(),
                     lit->first);

        if (s.length() > MAX_MASK2_WIDTH && mixed_sensitivity(s)) {
            DEBUG_PRINTF("long & mixed-sensitivity, Rose can't handle this\n");
            ++lit;
            continue;
        }

        if (s.length() > best.length()) {
            best = s;
            assert(!lit->second.exits.empty());
            best_v = lit->second.exits[0];
        }

        ++lit;
    }

    lit_out->swap(best);
    *v = best_v;
}

static
bool splitOffBestLiteral(const NGHolder &g,
                         const unordered_map<NFAVertex, u32> &regions,
                         ue2_literal *lit_out, NGHolder *lhs, NGHolder *rhs,
                         const CompileContext &cc) {
    NFAVertex v = NGHolder::null_vertex();

    findBestLiteral(g, regions, lit_out, &v, cc);
    if (lit_out->empty()) {
        return false;
    }

    DEBUG_PRINTF("literal is '%s'\n", dumpString(*lit_out).c_str());

    unordered_map<NFAVertex, NFAVertex> lhs_map;
    unordered_map<NFAVertex, NFAVertex> rhs_map;

    splitGraph(g, v, lhs, &lhs_map, rhs, &rhs_map);

    DEBUG_PRINTF("v = %zu\n", g[v].index);

    return true;
}

/**
 * Replace the given graph's EXTERNAL_CALLBACK reports with
 * EXTERNAL_CALLBACK_SOM_PASS reports.
 */
void makeReportsSomPass(ReportManager &rm, NGHolder &g) {
    for (const auto &v : vertices_range(g)) {
        const auto &reports = g[v].reports;
        if (reports.empty()) {
            continue;
        }

        flat_set<ReportID> new_reports;
        for (const ReportID &id : reports) {
            const Report &report = rm.getReport(id);
            if (report.type != EXTERNAL_CALLBACK) {
                new_reports.insert(id);
                continue;
            }
            Report report2 = report;
            report2.type = EXTERNAL_CALLBACK_SOM_PASS;
            new_reports.insert(rm.getInternalId(report2));
        }

        g[v].reports = new_reports;
    }
}

static
bool doLitHaigSom(NG &ng, NGHolder &g, som_type som) {
    ue2_literal lit;
    shared_ptr<NGHolder> rhs = make_shared<NGHolder>();
    if (!rhs) {
        assert(0);
        throw std::bad_alloc();
    }
    if (!ng.cc.grey.allowLitHaig) {
        return false;
    }

    dumpHolder(g, 90, "lithaig_full", ng.cc.grey);

    if (!splitOffLeadingLiteral(g, &lit, &*rhs)) {
        DEBUG_PRINTF("no literal\n");
        return false;
    }

    if (lit.length() < ng.cc.grey.minRoseLiteralLength) {
        DEBUG_PRINTF("lit too short\n");
        return false;
    }

    assert(lit.length() <= MAX_MASK2_WIDTH || !mixed_sensitivity(lit));

    makeReportsSomPass(ng.rm, *rhs);

    dumpHolder(*rhs, 91, "lithaig_rhs", ng.cc.grey);

    vector<vector<CharReach> > triggers;
    triggers.push_back(as_cr_seq(lit));

    assert(rhs->kind == NFA_SUFFIX);
    shared_ptr<raw_som_dfa> haig
        = attemptToBuildHaig(*rhs, som, ng.ssm.somPrecision(), triggers,
                             ng.cc.grey, false /* lit implies adv som */);
    if (!haig) {
        DEBUG_PRINTF("failed to haig\n");
        return false;
    }
    DEBUG_PRINTF("haig %p\n", haig.get());

    RoseInGraph ig;
    RoseInVertex s = add_vertex(RoseInVertexProps::makeStart(false), ig);
    RoseInVertex v = add_vertex(RoseInVertexProps::makeLiteral(lit), ig);
    add_edge(s, v, RoseInEdgeProps(0, ROSE_BOUND_INF), ig);

    RoseInVertex a
        = add_vertex(RoseInVertexProps::makeAccept(set<ReportID>()), ig);
    add_edge(v, a, RoseInEdgeProps(haig), ig);

    calcVertexOffsets(ig);

    return ng.rose->addSombeRose(ig);
}

static
bool doHaigLitHaigSom(NG &ng, NGHolder &g,
                      const unordered_map<NFAVertex, u32> &regions,
                      som_type som) {
    if (!ng.cc.grey.allowLitHaig) {
        return false;
    }

    // In streaming mode, we can only delay up to our max available history.
    const u32 max_delay =
        ng.cc.streaming ? ng.cc.grey.maxHistoryAvailable : MO_INVALID_IDX;

    ue2_literal lit;
    shared_ptr<NGHolder> rhs = make_shared<NGHolder>();
    shared_ptr<NGHolder> lhs = make_shared<NGHolder>();
    if (!rhs || !lhs) {
        assert(0);
        throw std::bad_alloc();
    }

    if (!splitOffBestLiteral(g, regions, &lit, &*lhs, &*rhs, ng.cc)) {
        return false;
    }

    DEBUG_PRINTF("split off best lit '%s' (len=%zu)\n", dumpString(lit).c_str(),
                 lit.length());

    if (lit.length() < ng.cc.grey.minRoseLiteralLength) {
        DEBUG_PRINTF("lit too short\n");
        return false;
    }

    assert(lit.length() <= MAX_MASK2_WIDTH || !mixed_sensitivity(lit));

    if (edge(rhs->start, rhs->acceptEod, *rhs).second) {
        return false; /* TODO: handle */
    }

    makeReportsSomPass(ng.rm, *rhs);

    dumpHolder(*lhs, 92, "haiglithaig_lhs", ng.cc.grey);
    dumpHolder(*rhs, 93, "haiglithaig_rhs", ng.cc.grey);

    u32 delay = removeTrailingLiteralStates(*lhs, lit, max_delay);

    RoseInGraph ig;
    RoseInVertex s
        = add_vertex(RoseInVertexProps::makeStart(false), ig);
    RoseInVertex v = add_vertex(RoseInVertexProps::makeLiteral(lit), ig);

    bool lhs_all_vac = true;
    NGHolder::adjacency_iterator ai, ae;
    for (tie(ai, ae) = adjacent_vertices(lhs->startDs, *lhs);
         ai != ae && lhs_all_vac; ++ai) {
        if (!is_special(*ai, *lhs)) {
            lhs_all_vac = false;
        }
    }
    for (tie(ai, ae) = adjacent_vertices(lhs->start, *lhs);
         ai != ae && lhs_all_vac; ++ai) {
        if (!is_special(*ai, *lhs)) {
            lhs_all_vac = false;
        }
    }

    if (lhs_all_vac) {
        /* lhs is completely vacuous --> no prefix needed */
        add_edge(s, v, RoseInEdgeProps(0, ROSE_BOUND_INF), ig);
    } else {
        assert(delay == lit.length());
        setReportOnHaigPrefix(*ng.rose, *lhs);
        vector<vector<CharReach> > prefix_triggers; /* empty for prefix */
        assert(lhs->kind == NFA_PREFIX);
        shared_ptr<raw_som_dfa> l_haig
            = attemptToBuildHaig(*lhs, som, ng.ssm.somPrecision(),
                                 prefix_triggers, ng.cc.grey);
        if (!l_haig) {
            DEBUG_PRINTF("failed to haig\n");
            return false;
        }
        DEBUG_PRINTF("lhs haig %p\n", l_haig.get());

        add_edge(s, v, RoseInEdgeProps(lhs, l_haig, delay), ig);
    }

    if (!edge(rhs->start, rhs->accept, *rhs).second) {
        assert(rhs->kind == NFA_SUFFIX);

        vector<vector<CharReach> > triggers;
        triggers.push_back(as_cr_seq(lit));

        ue2_literal lit2;
        if (getTrailingLiteral(g, &lit2)
            && lit2.length() >= ng.cc.grey.minRoseLiteralLength
            && minStringPeriod(lit2) >= 2) {

            /* TODO: handle delay */
            size_t overlap = maxOverlap(lit, lit2, 0);
            u32 delay2 = min((size_t)max_delay, lit2.length() - overlap);
            delay2 = removeTrailingLiteralStates(*rhs, lit2, delay2);
            rhs->kind = NFA_INFIX;
            assert(delay2 <= lit2.length());
            setReportOnHaigPrefix(*ng.rose, *rhs);

            shared_ptr<raw_som_dfa> m_haig
                = attemptToBuildHaig(*rhs, som, ng.ssm.somPrecision(),
                                     triggers, ng.cc.grey, true);
            DEBUG_PRINTF("mhs haig %p\n", m_haig.get());
            if (!m_haig) {
                DEBUG_PRINTF("failed to haig\n");
                return false;
            }

            RoseInVertex w
                = add_vertex(RoseInVertexProps::makeLiteral(lit2), ig);
            add_edge(v, w, RoseInEdgeProps(rhs, m_haig, delay2), ig);

            NFAVertex reporter = getSoleSourceVertex(g, g.accept);
            assert(reporter);
            const auto &reports = g[reporter].reports;
            RoseInVertex a =
                add_vertex(RoseInVertexProps::makeAccept(reports), ig);
            add_edge(w, a, RoseInEdgeProps(0U, 0U), ig);
        } else {
            /* TODO: analysis to see if som is in fact always increasing */
            shared_ptr<raw_som_dfa> r_haig
                = attemptToBuildHaig(*rhs, som, ng.ssm.somPrecision(),
                                     triggers, ng.cc.grey, true);
            DEBUG_PRINTF("rhs haig %p\n", r_haig.get());
            if (!r_haig) {
                DEBUG_PRINTF("failed to haig\n");
                return false;
            }
            RoseInVertex a
                = add_vertex(RoseInVertexProps::makeAccept(set<ReportID>()),
                             ig);
            add_edge(v, a, RoseInEdgeProps(r_haig), ig);
        }
    } else {
        DEBUG_PRINTF("has start->accept edge\n");
        if (in_degree(g.acceptEod, g) > 1) {
            DEBUG_PRINTF("also has a path to EOD\n");
            return false;
        }
        NFAVertex reporter = getSoleSourceVertex(g, g.accept);
        if (!reporter) {
            return false; /* TODO: later */
        }
        const auto &reports = g[reporter].reports;
        assert(!reports.empty());
        RoseInVertex a =
            add_vertex(RoseInVertexProps::makeAccept(reports), ig);
        add_edge(v, a, RoseInEdgeProps(0U, 0U), ig);
    }

    calcVertexOffsets(ig);

    return ng.rose->addSombeRose(ig);
}

static
bool doMultiLitHaigSom(NG &ng, NGHolder &g, som_type som) {
    set<ue2_literal> lits;
    shared_ptr<NGHolder> rhs = make_shared<NGHolder>();
    if (!ng.cc.grey.allowLitHaig) {
        return false;
    }

    dumpHolder(g, 90, "lithaig_full", ng.cc.grey);

    if (!splitOffLeadingLiterals(g, &lits, &*rhs)) {
        DEBUG_PRINTF("no literal\n");
        return false;
    }

    makeReportsSomPass(ng.rm, *rhs);

    dumpHolder(*rhs, 91, "lithaig_rhs", ng.cc.grey);

    vector<vector<CharReach>> triggers;
    for (const auto &lit : lits) {
        if (lit.length() < ng.cc.grey.minRoseLiteralLength) {
            DEBUG_PRINTF("lit too short\n");
            return false;
        }

        assert(lit.length() <= MAX_MASK2_WIDTH || !mixed_sensitivity(lit));
        triggers.push_back(as_cr_seq(lit));
    }

    bool unordered_som_triggers = true; /* TODO: check overlaps to ensure that
                                         * we can promise ordering */

    assert(rhs->kind == NFA_SUFFIX);
    shared_ptr<raw_som_dfa> haig
        = attemptToBuildHaig(*rhs, som, ng.ssm.somPrecision(), triggers,
                             ng.cc.grey, unordered_som_triggers);
    if (!haig) {
        DEBUG_PRINTF("failed to haig\n");
        return false;
    }
    DEBUG_PRINTF("haig %p\n", haig.get());

    RoseInGraph ig;
    RoseInVertex s = add_vertex(RoseInVertexProps::makeStart(false), ig);

    RoseInVertex a
        = add_vertex(RoseInVertexProps::makeAccept(set<ReportID>()), ig);

    for (const auto &lit : lits) {
        RoseInVertex v = add_vertex(RoseInVertexProps::makeLiteral(lit), ig);
        add_edge(s, v, RoseInEdgeProps(0, ROSE_BOUND_INF), ig);
        add_edge(v, a, RoseInEdgeProps(haig), ig);
    }

    calcVertexOffsets(ig);

    return ng.rose->addSombeRose(ig);
}

static
bool trySombe(NG &ng, NGHolder &g, som_type som) {
    if (doLitHaigSom(ng, g, som)) {
        return true;
    }

    auto regions = assignRegions(g);

    if (doHaigLitHaigSom(ng, g, regions, som)) {
        return true;
    }

    if (doMultiLitHaigSom(ng, g, som)) {
        return true;
    }

    return false;
}

static
map<u32, region_info>::const_iterator pickInitialSomCut(const NGHolder &g,
                        const unordered_map<NFAVertex, u32> &regions,
                        const map<u32, region_info> &info,
                        const vector<DepthMinMax> &depths) {
    map<u32, region_info>::const_iterator picked = info.end();
    for (map<u32, region_info>::const_iterator it = info.begin();
         it != info.end(); ++it) {
        if (it->second.exits.empty()) {
            assert(it == info.begin());
            continue;
        }

        if (!regionCanEstablishSom(g, regions, it->first, it->second.exits,
                                   depths)) {
            /* last region is as far as we can go */
            DEBUG_PRINTF("region %u is beyond the fixed region\n", it->first);
            break;
        }
        picked = it;
    }

    return picked;
}

static
map<u32, region_info>::const_iterator tryForLaterRevNfaCut(const NGHolder &g,
                              const unordered_map<NFAVertex, u32> &regions,
                              const map<u32, region_info> &info,
                              const vector<DepthMinMax> &depths,
                              const map<u32, region_info>::const_iterator &orig,
                              const CompileContext &cc) {
    DEBUG_PRINTF("trying for later rev nfa cut\n");
    assert(orig != info.end());

    vector<map<u32, region_info>::const_iterator> cands;

    map<u32, region_info>::const_iterator it = orig;
    ++it;
    for (; it != info.end(); ++it) {
        /* for simplicity */
        if (it->second.exits.size() != 1 || it->second.optional) {
            continue;
        }
        NFAVertex v = *it->second.exits.begin();

        if (edge(v, g.accept, g).second || edge(v, g.acceptEod, g).second) {
            continue; /* for simplicity would require external som nfa reports
                       * as well. */
        }

        const depth &max_depth = depths[g[v].index].max;
        if (max_depth >
            depth(cc.grey.somMaxRevNfaLength - 1)) { /* virtual starts */
            continue;
        }

        if (max_depth > depth(MAX_REV_NFA_PREFIX)) {
            /* probably not a good idea, anyway */
            continue;
        }

        cands.push_back(it);
    }

    while (!cands.empty()) {
        map<u32, region_info>::const_iterator rv = cands.back();
        cands.pop_back();

        NFAVertex v = *rv->second.exits.begin();

        set<ue2_literal> lits = getLiteralSet(g, v);
        compressAndScore(lits);
        if (lits.empty()) {
        next_region:
            continue;
        }
        for (const auto &lit : lits) {
            if (lit.length() <= 3 || minStringPeriod(lit) < 2) {
                goto next_region;
            }
        }

        if (rv->second.enters.empty()
            || find(rv->second.full.begin(), rv->second.full.end(), g.startDs)
                    != rv->second.full.end()) {
            continue;
        }

        if (!isMandRegionBetween(info.begin(), rv)
            && info.begin()->second.optional) {
            continue;
        }

        /* check to see if it is a reasonable size */
        auto prefix =
            makePrefix(g, regions, rv->second, next(rv)->second, false);

        NGHolder g_rev;
        reverseHolder(*prefix, g_rev);
        anchorStarts(g_rev);

        renumber_vertices(g_rev);
        g_rev.kind = NFA_REV_PREFIX;
        reduceGraphEquivalences(g_rev, cc);
        removeRedundancy(g_rev, SOM_NONE);

        if (num_vertices(g_rev) > 128) { /* too big */
            continue;
        }

        return rv;
    }

    return info.end();
}

static
unique_ptr<NGHolder> makePrefixForChain(NGHolder &g,
                         const unordered_map<NFAVertex, u32> &regions,
                         const map<u32, region_info> &info,
                         const map<u32, region_info>::const_iterator &picked,
                         vector<DepthMinMax> *depths, bool prefix_by_rev,
                         ReportManager &rm) {
    DEBUG_PRINTF("making prefix for chain attempt\n");
    auto prefix =
        makePrefix(g, regions, picked->second, next(picked)->second, false);

    /* For the root SOM plan, we use a temporary SOM slot to start with so that
     * we don't have to do any complicated rollback operations if the call to
     * doSomPlanning() below fails. The temporary SOM slot is replaced with a
     * real one afterwards. */
    const u32 temp_som_loc = UINT32_MAX;
    setPrefixReports(rm, *prefix, INTERNAL_SOM_LOC_SET_IF_WRITABLE,
                     temp_som_loc, *depths, prefix_by_rev);

    /* handle direct edge to accepts from region */
    if (edge(picked->second.exits.front(), g.accept, g).second
            || edge(picked->second.exits.front(), g.acceptEod, g).second) {
        map<u32, region_info>::const_iterator it = picked;
        do {
            makeSomRelReports(rm, g, it->second.exits, *depths);
        } while (it != info.begin() && it->second.optional && (it--)->first);
    }

    depths->clear(); /* renumbering invalidates depths */
    renumber_vertices(*prefix);

    DEBUG_PRINTF("done\n");
    return prefix;
}

sombe_rv doSom(NG &ng, NGHolder &g, const ExpressionInfo &expr, u32 comp_id,
               som_type som) {
    assert(som);
    DEBUG_PRINTF("som hello\n");
    ReportManager &rm = ng.rm;
    SomSlotManager &ssm = ng.ssm;
    const CompileContext &cc = ng.cc;

    // Special case: if g is completely anchored or begins with a dot-star, we
    // know that we have an absolute SOM of zero all the time.
    if (!proper_out_degree(g.startDs, g) || beginsWithDotStar(g)) {
        makeSomAbsReports(rm, g, g.accept);
        makeSomAbsReports(rm, g, g.acceptEod);
        return SOMBE_HANDLED_INTERNAL;
    }

    if (!cc.grey.allowSomChain) {
        return SOMBE_FAIL;
    }

    // A pristine copy of the input graph, which must be restored to in paths
    // that return false. Also used as the forward graph for som rev nfa
    // construction.
    NGHolder g_pristine;
    cloneHolder(g_pristine, g);

    vector<DepthMinMax> depths = getDistancesFromSOM(g);

    // try a redundancy pass.
    if (addSomRedundancy(g, depths)) {
        depths = getDistancesFromSOM(g); // recalc
    }

    auto regions = assignRegions(g);

    dumpHolder(g, regions, 11, "som_explode", cc.grey);

    map<u32, region_info> info;
    buildRegionMapping(g, regions, info);

    map<u32, region_info>::const_iterator picked
        = pickInitialSomCut(g, regions, info, depths);
    DEBUG_PRINTF("picked %u\n", picked->first);
    if (picked == info.end() || picked->second.exits.empty()) {
        DEBUG_PRINTF("no regions/no progress possible\n");
        clear_graph(g);
        cloneHolder(g, g_pristine);
        if (doSomRevNfa(ng, g, cc)) {
            return SOMBE_HANDLED_INTERNAL;
        } else {
            return SOMBE_FAIL;
        }
    }

    if (finalRegion(g, regions, picked->second.exits[0])) {
        makeSomRelReports(rm, g, g.accept, depths);
        makeSomRelReports(rm, g, g.acceptEod, depths);
        return SOMBE_HANDLED_INTERNAL;
    }

    if (doSomRevNfa(ng, g_pristine, cc)) {
        clear_graph(g);
        cloneHolder(g, g_pristine);
        return SOMBE_HANDLED_INTERNAL;
    }

    bool prefix_by_rev = false;
    map<u32, region_info>::const_iterator picked_old = picked;
    map<u32, region_info>::const_iterator rev_pick
        = tryForLaterRevNfaCut(g, regions, info, depths, picked, cc);
    if (rev_pick != info.end()) {
        DEBUG_PRINTF("found later rev prefix cut point\n");
        assert(rev_pick != picked);
        picked = rev_pick;
        prefix_by_rev = true;
    } else {
        /* sanity checks for picked region, these checks have already been done
         * if we are using a prefix reverse nfa. */
        if (picked->second.enters.empty()
            || find(picked->second.full.begin(), picked->second.full.end(),
                    g.startDs) != picked->second.full.end()) {
            clear_graph(g);
            cloneHolder(g, g_pristine);
            return SOMBE_FAIL;
        }

        if (!isMandRegionBetween(info.begin(), picked)
            && info.begin()->second.optional) {
            clear_graph(g);
            cloneHolder(g, g_pristine);
            return SOMBE_FAIL;
        }
    }

    DEBUG_PRINTF("region %u is the final\n", picked->first);

    shared_ptr<NGHolder> prefix = makePrefixForChain(
        g, regions, info, picked, &depths, prefix_by_rev, rm);
    /* note depths cleared as we have renumbered */

    CharReach escapes;
    bool stuck = isPossibleLock(g, picked, info, &escapes);
    if (stuck) {
        DEBUG_PRINTF("investigating potential lock\n");

        NGHolder gg;
        fillHolderForLockCheck(&gg, g, info, picked);

        stuck = firstMatchIsFirst(gg);
    }

    if (stuck && escapes.none()) {
        /* leads directly to .* --> woot */
        DEBUG_PRINTF("initial slot is full lock\n");
        u32 som_loc = ssm.getSomSlot(*prefix, escapes, false,
                                     SomSlotManager::NO_PARENT);
        replaceTempSomSlot(rm, *prefix, som_loc);

        /* update all reports on g to report the som_loc's som */
        updateReportToUseRecordedSom(rm, g, som_loc);

        /* create prefix to set the som_loc */
        updatePrefixReports(rm, *prefix, INTERNAL_SOM_LOC_SET_IF_UNSET);
        if (prefix_by_rev) {
            u32 rev_comp_id = doSomRevNfaPrefix(ng, expr, *prefix, cc);
            updatePrefixReportsRevNFA(rm, *prefix, rev_comp_id);
        }
        renumber_vertices(*prefix);
        if (!ng.addHolder(*prefix)) {
            DEBUG_PRINTF("failed to add holder\n");
            clear_graph(g);
            cloneHolder(g, g_pristine);
            return SOMBE_FAIL;
        }

        DEBUG_PRINTF("ok found initial lock\n");
        return SOMBE_HANDLED_INTERNAL;
    }

    vector<som_plan> plan;
 retry:
    // Note: no-one should ever pay attention to the root plan's parent.
    plan.push_back(som_plan(prefix, escapes, false, 0));
    dumpHolder(*plan.back().prefix, 12, "som_prefix", cc.grey);
    if (!prefix_by_rev) {
        if (!doSomPlanning(g, stuck, regions, info, picked, plan, cc.grey)) {
            DEBUG_PRINTF("failed\n");
            clear_graph(g);
            cloneHolder(g, g_pristine);
            return SOMBE_FAIL;
        }
    } else {
        DEBUG_PRINTF("trying for som plan\n");
        if (!doSomPlanning(g, stuck, regions, info, picked, plan, cc.grey,
                           DISALLOW_MODIFY_HOLDER)) {
            /* Note: the larger prefixes generated by reverse nfas may not
             * advance as fair as the original prefix - so we should retry
             * with a smaller prefix. */

            prefix_by_rev = false;
            stuck = false; /* if we reached a lock, then prefix_by_rev would not
                            * have advanced. */
            picked = picked_old;
            plan.clear();
            depths = getDistancesFromSOM(g); /* due to renumbering, need to
                                              * regenerate */
            prefix = makePrefixForChain(g, regions, info, picked, &depths,
                                        prefix_by_rev, rm);
            escapes.clear();
            DEBUG_PRINTF("retrying\n");
            goto retry;
        }
    }
    DEBUG_PRINTF("som planning ok\n");

    /* if the initial prefix is weak is if sombe approaches are better */
    if (findMinWidth(*prefix) <= depth(2)) {
        DEBUG_PRINTF("weak prefix... seeing if sombe can help out\n");
        NGHolder g2;
        cloneHolder(g2, g_pristine);
        if (trySombe(ng, g2, som)) {
            return SOMBE_HANDLED_ALL;
        }
    }

    /* From this point we know that we are going to succeed or die horribly with
     * a pattern too large. Anything done past this point can be considered
     * committed to the compile. */

    regions = assignRegions(g); // Update as g may have changed.

    DEBUG_PRINTF("-- get slot for initial plan\n");
    u32 som_loc;
    if (plan[0].is_reset) {
        som_loc = ssm.getInitialResetSomSlot(*prefix, g, regions,
                                picked->first, &plan[0].no_implement);
    } else {
        som_loc = ssm.getSomSlot(*prefix, escapes, false,
                                 SomSlotManager::NO_PARENT);
    }

    replaceTempSomSlot(rm, *prefix, som_loc);

    if (plan.front().is_reset) {
        updatePrefixReports(rm, *prefix, INTERNAL_SOM_LOC_SET);
    }
    if (prefix_by_rev && !plan.front().no_implement) {
        u32 rev_comp_id = doSomRevNfaPrefix(ng, expr, *prefix, cc);
        updatePrefixReportsRevNFA(rm, *prefix, rev_comp_id);
    }

    implementSomPlan(ng, expr, comp_id, g, plan, som_loc);

    DEBUG_PRINTF("success\n");
    return SOMBE_HANDLED_INTERNAL;
}

sombe_rv doSomWithHaig(NG &ng, NGHolder &g, const ExpressionInfo &expr,
                       u32 comp_id, som_type som) {
    assert(som);

    DEBUG_PRINTF("som+haig hello\n");

    // A pristine copy of the input graph, which must be restored to in paths
    // that return false. Also used as the forward graph for som rev nfa
    // construction.
    NGHolder g_pristine;
    cloneHolder(g_pristine, g);

    if (trySombe(ng, g, som)) {
        return SOMBE_HANDLED_ALL;
    }

    if (!ng.cc.grey.allowHaigLit || !ng.cc.grey.allowSomChain) {
        return SOMBE_FAIL;
    }

    // know that we have an absolute SOM of zero all the time.
    assert(edge(g.startDs, g.startDs, g).second);

    vector<DepthMinMax> depths = getDistancesFromSOM(g);

    // try a redundancy pass.
    if (addSomRedundancy(g, depths)) {
        depths = getDistancesFromSOM(g);
    }

    auto regions = assignRegions(g);

    dumpHolder(g, regions, 21, "som_explode", ng.cc.grey);

    map<u32, region_info> info;
    buildRegionMapping(g, regions, info, true);

    sombe_rv rv =
        doHaigLitSom(ng, g, expr, comp_id, som, regions, info, info.begin());
    if (rv == SOMBE_FAIL) {
        clear_graph(g);
        cloneHolder(g, g_pristine);
    }
    return rv;
}

} // namespace ue2
