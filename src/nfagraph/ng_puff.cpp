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
 * \brief Puff construction from NGHolder.
 */
#include "ng_puff.h"

#include "grey.h"
#include "ng_depth.h"
#include "ng_holder.h"
#include "ng_prune.h"
#include "ng_repeat.h"
#include "ng_reports.h"
#include "ng_util.h"
#include "ue2common.h"
#include "nfa/nfa_api_queue.h"
#include "nfa/mpvcompile.h"
#include "rose/rose_build.h"
#include "util/compile_context.h"
#include "util/graph_range.h"
#include "util/report_manager.h"

#include <vector>

using namespace std;

namespace ue2 {

static const unsigned MIN_PUFF_LENGTH = 16;
static const unsigned HEAD_BACKOFF    = 16;

static
size_t countChain(const NGHolder &g, NFAVertex v) {
    size_t count = 0;
    while (v) {
        DEBUG_PRINTF("counting vertex %zu\n", g[v].index);
        if (is_special(v, g)) {
            break;
        }

        count++;
        v = getSoleDestVertex(g, v);
    }
    DEBUG_PRINTF("done %zu\n", count);
    return count;
}

static
void wireNewAccepts(NGHolder &g, NFAVertex head,
                    const flat_set<ReportID> &chain_reports) {
    for (auto u : inv_adjacent_vertices_range(head, g)) {
        if (is_special(u, g)) {
            continue;
        }

        DEBUG_PRINTF("adding edge: %zu -> accept\n", g[u].index);
        assert(!edge(u, g.accept, g).second);
        assert(!edge(u, g.acceptEod, g).second);
        add_edge(u, g.accept, g);

        // Replace reports with our chain reports.
        auto &u_reports = g[u].reports;
        u_reports.clear();
        u_reports.insert(chain_reports.begin(), chain_reports.end());
    }
}

static
bool isFixedDepth(const NGHolder &g, NFAVertex v) {
    // If the vertex is reachable from startDs, it can't be fixed depth.
    auto depthFromStartDs = calcDepthsFrom(g, g.startDs);

    u32 idx = g[v].index;
    const DepthMinMax &ds = depthFromStartDs.at(idx);
    if (ds.min.is_reachable()) {
        DEBUG_PRINTF("vertex reachable from startDs\n");
        return false;
    }

    auto depthFromStart = calcDepthsFrom(g, g.start);

    /* we can still consider the head of a puff chain as at fixed depth if
     * it has a self-loop: so we look at all the preds of v (other than v
     * itself) */

    assert(v && !is_special(v, g));

    u32 count = 0;
    for (auto u : inv_adjacent_vertices_range(v, g)) {
        if (u == v) {
            continue; // self-loop
        }
        count++;

        idx = g[u].index;
        const DepthMinMax &d = depthFromStart.at(idx);
        if (d.min != d.max) {
            return false;
        }
    }

    return count != 0; // at least one fixed-depth pred
}

static
bool singleStart(const NGHolder &g) {
    set<NFAVertex> seen;

    for (auto v : adjacent_vertices_range(g.start, g)) {
        if (!is_special(v, g)) {
            DEBUG_PRINTF("saw %zu\n", g[v].index);
            seen.insert(v);
        }
    }
    for (auto v : adjacent_vertices_range(g.startDs, g)) {
        if (!is_special(v, g)) {
            DEBUG_PRINTF("saw %zu\n", g[v].index);
            seen.insert(v);
        }
    }

    DEBUG_PRINTF("comp has %zu starts\n", seen.size());

    return seen.size() == 1;
}

static
bool triggerResetsPuff(const NGHolder &g, NFAVertex head) {
    const CharReach puff_escapes = ~g[head].char_reach;

    for (auto u : inv_adjacent_vertices_range(head, g)) {
        if (!g[u].char_reach.isSubsetOf(puff_escapes)) {
            DEBUG_PRINTF("no reset on trigger %zu %zu\n", g[u].index,
                         g[head].index);
            return false;
        }
    }

    DEBUG_PRINTF("reset on trigger\n");
    return true;
}

/** ".*[X]{N}" can be treated as ".*[X]{N,}" (misc_opt does reverse transform)
 * */
static
bool triggerFloodsPuff(const NGHolder &g, NFAVertex head) {
    DEBUG_PRINTF("head = %zu\n", g[head].index);

    const CharReach &puff_cr = g[head].char_reach;

    /* we can use the pred of the head as the base of our check if it the cr
     * matches as if
     *   head cr subsetof pred cr: if head is being pushed on then puff must
     *       still being pushed on
     *   pred cr subsetof head cr: if the puff matches then head must be also
     *       always be on if the is connected to a wide enough cyclic
     */
    if (proper_in_degree(head, g) == 1
        && puff_cr == g[getSoleSourceVertex(g, head)].char_reach) {
        head = getSoleSourceVertex(g, head);
        DEBUG_PRINTF("temp new head = %zu\n", g[head].index);
    }

    for (auto s : inv_adjacent_vertices_range(head, g)) {
        DEBUG_PRINTF("s = %zu\n", g[s].index);
        if (!puff_cr.isSubsetOf(g[s].char_reach)) {
            DEBUG_PRINTF("no flood on trigger %zu %zu\n", g[s].index,
                         g[head].index);
            return false;
        }

        if (!hasSelfLoop(s, g) && s != g.start) {
            DEBUG_PRINTF("no self loop\n");
            return false;
        }

        if (s == g.start && !edge(g.startDs, head, g).second) {
            DEBUG_PRINTF("not float\n");
            return false;
        }
    }

    DEBUG_PRINTF("reset on trigger\n");
    return true;
}

static
u32 allowedSquashDistance(const CharReach &cr, u32 min_width, const NGHolder &g,
                          NFAVertex pv, bool prefilter) {
    CharReach accept_cr;
    DEBUG_PRINTF("hello |cr|=%zu %d\n", cr.count(), (int)cr.find_first());

    if (prefilter) {
        /* a later prefilter stage make weaken the lead up so we can't be sure
         * that all the triggers will be squashing the puffette. */
        return 0;
    }

    /* TODO: inspect further back in the pattern */
    for (auto u : inv_adjacent_vertices_range(pv, g)) {
        accept_cr |= g[u].char_reach;
    }

    DEBUG_PRINTF("|accept_cr|=%zu\n", accept_cr.count());

    if ((accept_cr & cr).any()) {
        return 0; /* the accept byte doesn't always kill the puffette. TODO:
                   * maybe if we look further back we could find something that
                   * would kill the puffette... */
    }
    DEBUG_PRINTF("returning squash distance of %u\n", min_width);
    return min_width;
}

/** Gives a stronger puff trigger when the trigger is connected to a wide
 * cyclic state (aside from sds) */
static
void improveHead(NGHolder &g, NFAVertex *a, vector<NFAVertex> *nodes) {
    DEBUG_PRINTF("attempting to improve puff trigger\n");
    assert(!nodes->empty());
    const CharReach &puff_cr = g[nodes->back()].char_reach;
    if (puff_cr.all()) {
        return; /* we can't really do much with this one */
    }

    /* add the runway */
    DEBUG_PRINTF("backing off - allowing a decent header\n");
    assert(nodes->size() > HEAD_BACKOFF);
    for (u32 i = 0; i < HEAD_BACKOFF - 1; i++) {
        nodes->pop_back();
    }
    *a = nodes->back();
    nodes->pop_back();
}

static
void constructPuff(NGHolder &g, const NFAVertex a, const NFAVertex puffv,
                   const CharReach &cr, const ReportID report, u32 width,
                   bool fixed_depth, bool unbounded, bool auto_restart,
                   RoseBuild &rose, ReportManager &rm,
                   flat_set<ReportID> &chain_reports, bool prefilter) {
    DEBUG_PRINTF("constructing Puff for report %u\n", report);
    DEBUG_PRINTF("a = %zu\n", g[a].index);

    const Report &puff_report = rm.getReport(report);
    const bool simple_exhaust = isSimpleExhaustible(puff_report);

    const bool pureAnchored = a == g.start && singleStart(g);
    if (!pureAnchored) {
        if (a == g.startDs || a == g.start) {
            DEBUG_PRINTF("add outfix ar(false)\n");

            raw_puff rp(width, unbounded, report, cr, auto_restart,
                        simple_exhaust);
            rose.addOutfix(rp);
            return;
        }

        DEBUG_PRINTF("add chain tail\n");
        u32 qi = ~0U;
        u32 event = MQE_TOP;
        raw_puff rp(width, unbounded, report, cr);
        rose.addChainTail(rp, &qi, &event);
        assert(qi != ~0U);
        u32 squashDistance = allowedSquashDistance(cr, width, g, puffv,
                                                   prefilter);

        Report ir = makeMpvTrigger(event, squashDistance);
        /* only need to trigger once if floatingUnboundedDot */
        bool floatingUnboundedDot = unbounded && cr.all() && !fixed_depth;
        if (floatingUnboundedDot) {
            ir.ekey = rm.getUnassociatedExhaustibleKey();
        }
        ReportID id = rm.getInternalId(ir);
        chain_reports.insert(id);
    } else {
        DEBUG_PRINTF("add outfix ar(%d)\n", (int)auto_restart);
        assert(!auto_restart || unbounded);
        raw_puff rp(width, unbounded, report, cr, auto_restart, simple_exhaust);
        rose.addOutfix(rp);
    }
}

static
bool doComponent(RoseBuild &rose, ReportManager &rm, NGHolder &g, NFAVertex a,
                 set<NFAVertex> &dead, const CompileContext &cc,
                 bool prefilter) {
    DEBUG_PRINTF("hello\n");
    vector<NFAVertex> nodes;
    const CharReach &cr = g[a].char_reach;
    bool isDot = cr.all();
    bool unbounded = false;
    bool exhaustible = can_exhaust(g, rm);

    while (true) {
        if (is_special(a, g)) {
            DEBUG_PRINTF("stopped puffing due to special vertex\n");
            break;
        }

        if (g[a].char_reach != cr) {
            DEBUG_PRINTF("stopped puffing due to change in character "
                         "reachability\n");
            break;
        }

        if (proper_in_degree(a, g) != 1) {
            DEBUG_PRINTF("stopped puffing due to in degree != 1\n");
            break;
        }

        size_t outDegree = out_degree(a, g);
        if (outDegree != 1 && (!hasSelfLoop(a, g) || outDegree != 2)) {
            DEBUG_PRINTF("stopping puffing due to out degree\n");
            break;
        }

        if (hasSelfLoop(a, g)) {
            DEBUG_PRINTF("has self-loop, marking unbounded\n");
            unbounded = true;
        }

        nodes.push_back(a);
        DEBUG_PRINTF("vertex %zu has in_degree %zu\n", g[a].index,
                     in_degree(a, g));

        a = getSoleSourceVertex(g, a);

        assert(a); /* already checked that old a had a proper in degree of 1 */

        // Snark: we can't handle this case, because we can only handle a
        // single report ID on a vertex
        if (is_match_vertex(a, g)) {
            DEBUG_PRINTF("stop puffing due to vertex that leads to accept\n");
            if (!nodes.empty()) {
                nodes.pop_back();
            }
            break;
        }
    }

    if (!nodes.empty() && proper_in_degree(nodes.back(), g) != 1) {
        for (auto u : inv_adjacent_vertices_range(nodes.back(), g)) {
            if (is_special(u, g)) {
                DEBUG_PRINTF("pop\n");
                a = nodes.back();
                nodes.pop_back();
                break;
            }
        }
    }

    if (a != g.startDs && edge(g.startDs, a, g).second
        && proper_out_degree(a, g) == 1
        && g[a].char_reach == cr) {
        nodes.push_back(a);
        a = g.startDs;
    }

    bool auto_restart = false;

    DEBUG_PRINTF("a = %zu\n", g[a].index);

    if (nodes.size() < MIN_PUFF_LENGTH || a == g.startDs) {
        DEBUG_PRINTF("bad %zu %zu\n", nodes.size(), g[a].index);
        if (nodes.size() < MIN_PUFF_LENGTH) {
            return false;
        } else {
            DEBUG_PRINTF("mark unbounded\n");
            unbounded = true;
            a = g.start;
            auto_restart = !isDot;
        }
    }

    bool supported = false;
    bool fixed_depth = isFixedDepth(g, nodes.back());

    if (exhaustible) {
        supported = true;
    } else if (fixed_depth) {
        supported = true;
    } else if (unbounded) {
        /* any C{n, } can be supported as all ranges will be squashed together
         * only need to track the first */
        supported = true;
    } else if (triggerResetsPuff(g, nodes.back())) {
        supported = true;
    } else if (triggerFloodsPuff(g, nodes.back())) {
        DEBUG_PRINTF("trigger floods puff\n");
        supported = true;
        unbounded = true;
    }

    if (!supported) {
        DEBUG_PRINTF("not supported\n");
        return false;
    }

    if (cc.grey.puffImproveHead && a != g.start) {
        if (edge(g.startDs, a, g).second) {
            goto skip_improve; /* direct sds cases are better handled by auto
                                * restarting puffettes */
        }

        if (fixed_depth) {
            goto skip_improve; /* no danger of trigger floods */
        }

        /* if we come after something literalish don't bother */
        if (g[a].char_reach.count() <= 2
            && in_degree(a, g) == 1
            && g[getSoleSourceVertex(g, a)].char_reach.count() <= 2) {
            goto skip_improve;
        }

        if (nodes.size() < MIN_PUFF_LENGTH + HEAD_BACKOFF) {
            return false; /* not enough of the puff left to worth bothering
                             about */
        }

        improveHead(g, &a, &nodes);
    skip_improve:;
    }

    assert(!nodes.empty());
    const auto &reports = g[nodes[0]].reports;
    assert(!reports.empty());

    for (auto report : reports) {
        const Report &ir = rm.getReport(report);
        const bool highlander = ir.ekey != INVALID_EKEY;
        if (!unbounded && highlander && !isSimpleExhaustible(ir)) {
            DEBUG_PRINTF("report %u is bounded highlander but not simple "
                         "exhaustible\n",
                         report);
            return false;
        }

        if (ir.type == INTERNAL_ROSE_CHAIN) {
            DEBUG_PRINTF("puffettes cannot be chained together\n");
            return false;
        }
    }

    NFAVertex puffv = nodes.back();
    assert(puffv != NGHolder::null_vertex());
    u32 width = countChain(g, nodes.back());

    flat_set<ReportID> chain_reports;

    for (auto report : reports) {
        constructPuff(g, a, puffv, cr, report, width, fixed_depth, unbounded,
                      auto_restart, rose, rm, chain_reports, prefilter);
    }

    if (!chain_reports.empty()) {
        wireNewAccepts(g, puffv, chain_reports);
    }

    dead.insert(nodes.begin(), nodes.end());
    return true;
}

bool splitOffPuffs(RoseBuild &rose, ReportManager &rm, NGHolder &g,
                   bool prefilter, const CompileContext &cc) {
    if (!cc.grey.allowPuff) {
        return false;
    }

    size_t count = 0;
    set<NFAVertex> dead;

    for (auto v : inv_adjacent_vertices_range(g.accept, g)) {
        if (doComponent(rose, rm, g, v, dead, cc, prefilter)) {
            count++;
        }
    }

    if (!dead.empty()) {
        remove_vertices(dead, g);
        pruneUseless(g);
    }

    DEBUG_PRINTF("puffs: %zu\n", count);
    return num_vertices(g) <= N_SPECIALS;
}

bool isPuffable(const NGHolder &g, bool fixed_depth,
                const ReportManager &rm, const Grey &grey) {
    if (!grey.allowPuff) {
        return false;
    }

    if (!onlyOneTop(g)) {
        DEBUG_PRINTF("more than one top\n");
        return false;
    }

    const set<ReportID> reports = all_reports(g);
    if (reports.size() != 1) {
        DEBUG_PRINTF("too many reports\n");
        return false;
    }

    const Report &ir = rm.getReport(*reports.begin());

    if (ir.type == INTERNAL_ROSE_CHAIN) {
        DEBUG_PRINTF("puffettes cannot be chained together\n");
        return false;
    }

    PureRepeat repeat;
    if (!isPureRepeat(g, repeat)) {
        DEBUG_PRINTF("not pure bounded repeat\n");
        return false;
    }

    if (repeat.bounds.min == depth(0)) {
        DEBUG_PRINTF("repeat min bound is zero\n");
        return false;
    }

    // We can puff if:
    // (a) repeat is {N,}; or
    // (b) repeat is {N} and fixed-depth, or highlander (and will accept the
    // first match)

    DEBUG_PRINTF("repeat is %s\n", repeat.bounds.str().c_str());

    if (repeat.bounds.max.is_infinite()) {
        return true;
    }

    if (repeat.bounds.min == repeat.bounds.max) {
        if (fixed_depth) {
            DEBUG_PRINTF("fixed depth\n");
            return true;
        }

        const bool highlander = ir.ekey != INVALID_EKEY;

        // If we're highlander, we must be simple-exhaustible as well.
        if (highlander && isSimpleExhaustible(ir)) {
            return true;
        }
    }

    return false;
}

} // namespace ue2
