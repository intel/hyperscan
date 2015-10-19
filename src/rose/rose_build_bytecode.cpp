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

#include "rose_build_impl.h"

#include "ue2common.h"
#include "grey.h"
#include "hs_compile.h" // for HS_MODE_*
#include "rose_build_add_internal.h"
#include "rose_build_anchored.h"
#include "rose_build_infix.h"
#include "rose_build_lookaround.h"
#include "rose_build_scatter.h"
#include "rose_build_util.h"
#include "rose_build_width.h"
#include "hwlm/hwlm.h" /* engine types */
#include "hwlm/hwlm_build.h"
#include "nfa/castlecompile.h"
#include "nfa/goughcompile.h"
#include "nfa/mcclellancompile.h"
#include "nfa/nfa_api_queue.h"
#include "nfa/nfa_build_util.h"
#include "nfa/nfa_internal.h"
#include "nfa/shufticompile.h"
#include "nfagraph/ng_holder.h"
#include "nfagraph/ng_lbr.h"
#include "nfagraph/ng_limex.h"
#include "nfagraph/ng_mcclellan.h"
#include "nfagraph/ng_repeat.h"
#include "nfagraph/ng_reports.h"
#include "nfagraph/ng_revacc.h"
#include "nfagraph/ng_stop.h"
#include "nfagraph/ng_util.h"
#include "nfagraph/ng_width.h"
#include "sidecar/sidecar.h"
#include "sidecar/sidecar_compile.h"
#include "som/slot_manager.h"
#include "util/alloc.h"
#include "util/bitutils.h"
#include "util/boundary_reports.h"
#include "util/charreach.h"
#include "util/charreach_util.h"
#include "util/compile_context.h"
#include "util/compile_error.h"
#include "util/container.h"
#include "util/graph_range.h"
#include "util/dump_charclass.h"
#include "util/internal_report.h"
#include "util/multibit_build.h"
#include "util/order_check.h"
#include "util/queue_index_factory.h"
#include "util/report_manager.h"
#include "util/ue2string.h"
#include "util/verify_types.h"

#include <algorithm>
#include <iomanip>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <vector>
#include <utility>

#include <boost/range/adaptor/map.hpp>

using namespace std;
using boost::adaptors::map_values;
using boost::adaptors::map_keys;

namespace ue2 {

/* The rose bytecode construction is a giant cesspit.
 *
 * One issue is that bits and pieces are constructed piecemeal and these
 * sections are used by later in the construction process. Until the very end of
 * the construction there is no useful invariant holding for the bytecode. This
 * makes reordering / understanding the construction process awkward as there
 * are hidden dependencies everywhere. We should start by shifting towards
 * a model where the bytecode is only written to during the construction so that
 * the dependencies can be understood by us mere mortals.
 *
 * I am sure the construction process is also bad from a number of other
 * standpoints as well but the can come later.
 *
 * Actually, one other annoying issues the plague of member functions on the
 * impl which tightly couples the internals of this file to all the other rose
 * build files. Need more egregiously awesome free functions.
 */

namespace /* anon */ {

// Orders RoseEdge edges by the state index of the source node
struct EdgeSourceStateCompare {
    EdgeSourceStateCompare(const RoseGraph &g_,
                           const vector<RoseRole> &roleTable_) :
                                g(g_), roleTable(roleTable_) {}
    bool operator()(const RoseEdge &a, const RoseEdge &b) const {
        u32 arole = g[source(a, g)].role;
        u32 brole = g[source(b, g)].role;
        if (arole >= roleTable.size()) {
            DEBUG_PRINTF("bad arole %u (idx=%zu)\n", arole, g[source(a, g)].idx);
        }
        if (brole >= roleTable.size()) {
            DEBUG_PRINTF("bad brole %u (idx=%zu)\n", brole, g[source(b, g)].idx);
        }
        assert(arole < roleTable.size());
        assert(brole < roleTable.size());
        return roleTable.at(arole).stateIndex < roleTable.at(brole).stateIndex;
    }
    const RoseGraph &g;
    const vector<RoseRole> &roleTable;
};

struct RoseTriggerOrdering {
    RoseTriggerOrdering() {}
    bool operator()(const RoseTrigger &a, const RoseTrigger &b) const {
        ORDER_CHECK(queue);
        ORDER_CHECK(event);
        ORDER_CHECK(cancel_prev_top);
        return false;
    }
};
struct RoseTriggerEquality {
    RoseTriggerEquality() {}
    bool operator()(const RoseTrigger &a, const RoseTrigger &b) const {
        return a.queue == b.queue
            && a.event == b.event
            && a.cancel_prev_top == b.cancel_prev_top;
    }
};

struct left_build_info {
    // Constructor for an engine implementation.
    left_build_info(NFA *n, u32 q, u32 l, u32 t, rose_group sm,
                    const std::vector<u8> &stops, u32 max_ql, u8 cm_count,
                    const CharReach &cm_cr)
        : nfa(n), queue(q), lag(l), transient(t), squash_mask(sm),
          stopAlphabet(stops), max_queuelen(max_ql),
          countingMiracleCount(cm_count), countingMiracleReach(cm_cr) {
        assert(n);
    }

    // Constructor for a lookaround implementation.
    explicit left_build_info(const vector<LookEntry> &look)
        : has_lookaround(true), lookaround(look) {}

    NFA *nfa = nullptr; /* uniquely idents the left_build_info */
    u32 queue = 0; /* also uniquely idents the left_build_info */
    u32 lag = 0;
    u32 transient = 0;
    rose_group squash_mask = ~rose_group{0};
    vector<u8> stopAlphabet;
    u32 max_queuelen = 0;
    u8 countingMiracleCount = 0;
    CharReach countingMiracleReach;
    u32 countingMiracleOffset = 0; /* populated later when laying out bytecode */
    bool has_lookaround = false;
    vector<LookEntry> lookaround; // alternative implementation to the NFA
};

struct build_context : boost::noncopyable {
    /** \brief Rose Role information.
     * These entries are filled in by a number of functions as other tables are
     * created.
     */
    vector<RoseRole> roleTable;

    /** \brief minimum depth in number of hops from root/anchored root. */
    map<RoseVertex, u32> depths;

    /** \brief information about engines to the left of a vertex */
    map<RoseVertex, left_build_info> leftfix_info;

    /** \brief Number of roles with a state bit.
     * This set by buildInitialRoleTable() and should be constant throughout
     * the rest of the compile.
     */
    size_t numStates = 0;

    // Very simple cache from sparse iter to offset, used when building up
    // iterators in early misc.
    map<vector<mmbit_sparse_iter>, u32> iterCache;

    /** \brief maps RoseRole index to a list of RosePred indices */
    map<u32, vector<u32> > rolePredecessors;

    /** \brief Lookaround table for Rose roles. */
    vector<LookEntry> lookaround;

    /** \brief Map from literal final ID to a set of non-root role IDs. */
    ue2::unordered_map<u32, set<u32>> litNonRootRoles;

    /* contents of rose immediately following the RoseEngine. */
    vector<char> engine_blob;

    /* base offset of engine_blob in the bytecode */
    const u32 engine_blob_base = ROUNDUP_16(sizeof(RoseEngine));
};

}

/* vertex ordered by their role index */
static
vector<RoseVertex> get_ordered_verts(const RoseGraph &g) {
    vector<RoseVertex> verts;
    insert(&verts, verts.end(), vertices_range(g));
    sort(verts.begin(), verts.end(),
         [&g](const RoseVertex &a, const RoseVertex &b) {
             return g[a].role < g[b].role;
         });
    return verts;
}

static
u32 countRosePrefixes(const vector<LeftNfaInfo> &roses) {
    u32 num = 0;
    for (const auto &r : roses) {
        if (!r.infix) {
            num++;
        }
    }
    return num;
}

static
bool isPureFloating(const RoseBuildImpl &tbi) {
    if (!tbi.outfixes.empty()) {
        DEBUG_PRINTF("has outfixes\n");
        return false;
    }

    const RoseGraph &g = tbi.g;

    if (!isLeafNode(tbi.anchored_root, g)) {
        DEBUG_PRINTF("has anchored vertices\n");
        return false;
    }

    for (auto v : vertices_range(g)) {
        if (tbi.root == v) {
            continue;
        }

        if (tbi.anchored_root == v) {
            assert(isLeafNode(v, g));
            continue;
        }

        if (!tbi.hasDirectFinalId(v) || !tbi.isFloating(v)) {
            DEBUG_PRINTF("vertex %zu isn't floating and direct\n", g[v].idx);
            return false;
        }

        for (ReportID r : g[v].reports) {
            const Report &ri = tbi.rm.getReport(r);
            if (!isExternalReport(ri)) {
                DEBUG_PRINTF("vertex %zu has non-external report\n", g[v].idx);
                return false;
            }
        }
    }

    DEBUG_PRINTF("pure floating literals\n");
    return true;
}

static
bool isSingleOutfix(const RoseBuildImpl &tbi, u32 outfixEndQueue) {
    for (auto v : vertices_range(tbi.g)) {
        if (tbi.isAnyStart(v)) {
            continue;
        }
        if (tbi.hasLiteralInTable(v, ROSE_ANCHORED_SMALL_BLOCK)) {
            continue;
        }
        DEBUG_PRINTF("has role\n");
        return false;
    }

    if (tbi.ssm.numSomSlots()) {
        return false;
    }

    if (!tbi.boundary.report_at_eod.empty()) {
        return false; /* streaming runtime makes liberal use of broken flag */
    }

    return outfixEndQueue == 1;
}

static
u8 pickRuntimeImpl(const RoseBuildImpl &tbi, u32 outfixEndQueue) {
    if (isPureFloating(tbi)) {
        return ROSE_RUNTIME_PURE_LITERAL;
    }

    if (isSingleOutfix(tbi, outfixEndQueue)) {
        return ROSE_RUNTIME_SINGLE_OUTFIX;
    }

    return ROSE_RUNTIME_FULL_ROSE;
}

static
void fillStateOffsets(const RoseBuildImpl &tbi, const sidecar *side,
                      u32 rolesWithStateCount, u32 anchorStateSize,
                      u32 activeArrayCount, u32 activeLeftCount,
                      u32 laggedRoseCount, u32 floatingStreamStateRequired,
                      u32 historyRequired, RoseStateOffsets *so) {
    /* runtime state (including role state) first and needs to be u32-aligned */
    u32 curr_offset = sizeof(RoseRuntimeState)
                    + mmbit_size(rolesWithStateCount);

    so->sidecar = curr_offset;
    if (side) {
        so->sidecar_size = sidecarEnabledSize(side);
        curr_offset += so->sidecar_size;
    }

    so->activeLeafArray = curr_offset; /* TODO: limit size of array */
    curr_offset += mmbit_size(activeArrayCount);

    so->activeLeftArray = curr_offset; /* TODO: limit size of array */
    so->activeLeftArray_size = mmbit_size(activeLeftCount);
    curr_offset += so->activeLeftArray_size;

    so->floatingMatcherState = curr_offset;
    curr_offset += floatingStreamStateRequired;

    // ONE WHOLE BYTE for each active leftfix with lag.
    so->leftfixLagTable = curr_offset;
    curr_offset += laggedRoseCount;

    // Anchored state is McClellan full state, and needs to be 2-byte aligned.
    // We potentially waste a byte here.
    if (curr_offset % 2) {
        curr_offset++;
    }
    so->anchorState = curr_offset;
    curr_offset += anchorStateSize;

    so->groups = curr_offset;
    so->groups_size = (tbi.group_end + 7) / 8;
    assert(so->groups_size <= sizeof(u64a));
    curr_offset += so->groups_size;

    // The history consists of the bytes in the history only. YAY
    so->history = curr_offset;
    curr_offset += historyRequired;

    // Exhausted bit vector.
    so->exhausted = curr_offset;
    curr_offset += ROUNDUP_N(tbi.rm.numEkeys(), 8) / 8;

    // SOM locations and valid/writeable multibit structures.
    if (tbi.ssm.numSomSlots()) {
        const u32 somWidth = tbi.ssm.somPrecision();
        if (somWidth) { // somWidth is zero in block mode.
            curr_offset = ROUNDUP_N(curr_offset, somWidth);
            so->somLocation = curr_offset;
            curr_offset += tbi.ssm.numSomSlots() * somWidth;
        } else {
            so->somLocation = 0;
        }
        so->somValid = curr_offset;
        curr_offset += mmbit_size(tbi.ssm.numSomSlots());
        so->somWritable = curr_offset;
        curr_offset += mmbit_size(tbi.ssm.numSomSlots());
    } else {
        // No SOM handling, avoid growing the stream state any further.
        so->somLocation = 0;
        so->somValid = 0;
        so->somWritable = 0;
    }

    // note: state space for mask nfas is allocated later
    so->end = curr_offset;
}

// Get the mask of initial vertices due to root and anchored_root.
rose_group RoseBuildImpl::getInitialGroups() const {
    rose_group groups = getSuccGroups(root) | getSuccGroups(anchored_root);
    DEBUG_PRINTF("initial groups = %016llx\n", groups);
    return groups;
}

static
bool nfaStuckOn(const NGHolder &g) {
    assert(!proper_out_degree(g.startDs, g));
    set<NFAVertex> succ;
    insert(&succ, adjacent_vertices(g.start, g));
    succ.erase(g.startDs);

    set<NFAVertex> asucc;
    set<u32> tops;
    set<u32> done_tops;

    for (const auto &e : out_edges_range(g.start, g)) {
        tops.insert(g[e].top);
        if (!g[target(e, g)].char_reach.all()) {
            continue;
        }

        asucc.clear();
        insert(&asucc, adjacent_vertices(target(e, g), g));

        if (asucc == succ) {
            done_tops.insert(g[e].top);
        }
    }

    return tops == done_tops;
}

namespace {
struct PredTopPair {
    PredTopPair(RoseVertex v, u32 t) : pred(v), top(t) {}
    bool operator<(const PredTopPair &b) const {
        const PredTopPair &a = *this;
        ORDER_CHECK(pred);
        ORDER_CHECK(top);
        return false;
    }
    RoseVertex pred;
    u32 top;
};
}

static
void findFixedDepthTops(const RoseGraph &g, const set<PredTopPair> &triggers,
                        map<u32, u32> *fixed_depth_tops) {
    DEBUG_PRINTF("|trig| %zu\n", triggers.size());
    /* find all pred roles for this holder, group by top */
    /* if all pred roles for a given top have the same min and max offset, we
     * add the top to the fixed_depth_top map */
    map<u32, set<RoseVertex> > pred_by_top;
    for (const auto &ptp : triggers) {
        u32 top = ptp.top;
        RoseVertex u = ptp.pred;
        pred_by_top[top].insert(u);
    }

    for (const auto &e : pred_by_top) {
        u32 top = e.first;
        const set<RoseVertex> &preds = e.second;
        if (!g[*preds.begin()].fixedOffset()) {
            continue;
        }
        u32 depth = g[*preds.begin()].min_offset;
        for (RoseVertex u : preds) {
            if (g[u].min_offset != depth || g[u].max_offset != depth) {
                goto next_top;
            }
        }
        DEBUG_PRINTF("%u at depth %u\n", top, depth);
        (*fixed_depth_tops)[top] = depth;
    next_top:;
    }
}

/**
 * \brief Heuristic for picking between a DFA or NFA implementation of an
 * engine.
 */
static
aligned_unique_ptr<NFA> pickImpl(aligned_unique_ptr<NFA> dfa_impl,
                                 aligned_unique_ptr<NFA> nfa_impl) {
    assert(nfa_impl);
    assert(dfa_impl);
    assert(isMcClellanType(dfa_impl->type));

    // If our NFA is an LBR, it always wins.
    if (isLbrType(nfa_impl->type)) {
        return nfa_impl;
    }

    bool d_accel = has_accel(*dfa_impl);
    bool n_accel = has_accel(*nfa_impl);
    bool d_big = dfa_impl->type == MCCLELLAN_NFA_16;
    bool n_vsmall = nfa_impl->nPositions <= 32;
    bool n_br = has_bounded_repeats(*nfa_impl);
    DEBUG_PRINTF("da %d na %d db %d nvs %d nbr %d\n", (int)d_accel,
                 (int)n_accel, (int)d_big, (int)n_vsmall, (int)n_br);
    if (d_big) {
        if (!n_vsmall) {
            if (d_accel || !n_accel) {
                return dfa_impl;
            } else {
                return nfa_impl;
            }
        } else {
            if (n_accel) {
                return nfa_impl;
            } else {
                return dfa_impl;
            }
        }
    } else {
        /* favour a McClellan 8, unless the nfa looks really good and the dfa
         * looks like trouble */
        if (!d_accel && n_vsmall && n_accel && !n_br) {
            return nfa_impl;
        } else {
            return dfa_impl;
        }
    }
}

/**
 * \brief Builds an LBR if there's one repeat in the given CastleProto,
 * otherwise a Castle.
 */
static
aligned_unique_ptr<NFA>
buildRepeatEngine(const CastleProto &proto,
                  const map<u32, vector<vector<CharReach>>> &triggers,
                  const CompileContext &cc) {
    // If we only have one repeat, the LBR should always be the best possible
    // implementation.
    if (proto.repeats.size() == 1 && cc.grey.allowLbr) {
        return constructLBR(proto.repeats.begin()->second, triggers.at(0), cc);
    }

    aligned_unique_ptr<NFA> castle_nfa = buildCastle(proto, triggers, cc);
    assert(castle_nfa); // Should always be constructible.
    return castle_nfa;
}

/* builds suffix nfas */
static
aligned_unique_ptr<NFA>
buildSuffix(const ReportManager &rm, const SomSlotManager &ssm,
            const map<u32, u32> &fixed_depth_tops,
            const map<u32, vector<vector<CharReach>>> &triggers,
            suffix_id suff, const CompileContext &cc) {
    if (suff.castle()) {
        auto n = buildRepeatEngine(*suff.castle(), triggers, cc);
        assert(n);
        return n;
    }

    if (suff.haig()) {
        auto n = goughCompile(*suff.haig(), ssm.somPrecision(), cc);
        assert(n);
        return n;
    }

    if (suff.dfa()) {
        auto d = mcclellanCompile(*suff.dfa(), cc);
        assert(d);
        return d;
    }

    assert(suff.graph());
    NGHolder &holder = *suff.graph();
    assert(holder.kind == NFA_SUFFIX);
    const bool oneTop = onlyOneTop(holder);
    bool compress_state = cc.streaming;

    // Take a shot at the LBR engine.
    if (oneTop) {
        auto lbr = constructLBR(holder, triggers.at(0), cc);
        if (lbr) {
            return lbr;
        }
    }

    auto n = constructNFA(holder, &rm, fixed_depth_tops, triggers,
                          compress_state, cc);
    assert(n);

    if (oneTop && cc.grey.roseMcClellanSuffix) {
        if (cc.grey.roseMcClellanSuffix == 2 || n->nPositions > 128 ||
            !has_bounded_repeats_other_than_firsts(*n)) {
            auto rdfa = buildMcClellan(holder, &rm, false, triggers.at(0),
                                       cc.grey);
            if (rdfa) {
                auto d = mcclellanCompile(*rdfa, cc);
                assert(d);
                if (cc.grey.roseMcClellanSuffix != 2) {
                    n = pickImpl(move(d), move(n));
                } else {
                    n = move(d);
                }

                assert(n);
                if (isMcClellanType(n->type)) {
                    // DFA chosen. We may be able to set some more properties
                    // in the NFA structure here.
                    u64a maxOffset = findMaxOffset(holder, rm);
                    if (maxOffset != MAX_OFFSET && maxOffset < 0xffffffffull) {
                        n->maxOffset = (u32)maxOffset;
                        DEBUG_PRINTF("dfa max offset %llu\n", maxOffset);
                    } else {
                        n->maxOffset = 0; // inf
                    }
                }
            }
        }
    }
    return n;
}

static
void findInfixTriggers(const RoseBuildImpl &build,
                       map<left_id, set<PredTopPair> > *infixTriggers) {
    const RoseGraph &g = build.g;
    for (auto v : vertices_range(g)) {
        if (!g[v].left) {
            continue;
        }

        set<PredTopPair> &triggers = (*infixTriggers)[left_id(g[v].left)];

        for (const auto &e : in_edges_range(v, g)) {
            RoseVertex u = source(e, g);
            if (build.isAnyStart(u)) {
                continue;
            }
            triggers.insert(PredTopPair(u, g[e].rose_top));
        }
    }
}

static
vector<CharReach> as_cr_seq(const rose_literal_id &lit) {
    vector<CharReach> rv = as_cr_seq(lit.s);
    for (u32 i = 0; i < lit.delay; i++) {
        rv.push_back(CharReach::dot());
    }

    /* TODO: take into account cmp/msk */
    return rv;
}

/**
 * \brief Returns a map of trigger literals as sequences of CharReach, grouped
 * by top index.
 */
static
void findTriggerSequences(const RoseBuildImpl &tbi,
                          const set<PredTopPair> &triggers,
                          map<u32, vector<vector<CharReach> > > *trigger_lits) {
    map<u32, set<u32> > lit_ids_by_top;
    for (const PredTopPair &t : triggers) {
        insert(&lit_ids_by_top[t.top], tbi.g[t.pred].literals);
    }

    for (const auto &e : lit_ids_by_top) {
        const u32 top = e.first;
        const set<u32> &lit_ids = e.second;

        for (u32 id :  lit_ids) {
            const rose_literal_id &lit = tbi.literals.right.at(id);
            (*trigger_lits)[top].push_back(as_cr_seq(lit));
        }
    }
}

static aligned_unique_ptr<NFA>
makeLeftNfa(const RoseBuildImpl &tbi, left_id &left,
            const bool is_prefix, const bool is_transient,
            const map<left_id, set<PredTopPair> > &infixTriggers,
            const CompileContext &cc) {
    aligned_unique_ptr<NFA> n;

    // Should compress state if this rose is non-transient and we're in
    // streaming mode.
    const bool compress_state = !is_transient;

    assert(!left.graph()
           || left.graph()->kind == (is_prefix ? NFA_PREFIX : NFA_INFIX));

    // Holder should be implementable as an NFA at the very least.
    if (!left.dfa() && left.graph()) {
        assert(isImplementableNFA(*left.graph(), nullptr, cc));
    }

    map<u32, u32> fixed_depth_tops;
    if (!is_prefix /* infix */) {
        const set<PredTopPair> &triggers = infixTriggers.at(left);
        findFixedDepthTops(tbi.g, triggers, &fixed_depth_tops);
    }

    if (left.castle()) {
        assert(!is_prefix);
        map<u32, vector<vector<CharReach> > > triggers;
        findTriggerSequences(tbi, infixTriggers.at(left), &triggers);
        n = buildRepeatEngine(*left.castle(), triggers, cc);
        assert(n);
        return n; // Castles/LBRs are always best!
    }

    if (left.dfa()) {
        n = mcclellanCompile(*left.dfa(), cc);
    } else if (left.graph() && cc.grey.roseMcClellanPrefix == 2 && is_prefix &&
               !is_transient) {
        auto rdfa = buildMcClellan(*left.graph(), nullptr, cc.grey);
        if (rdfa) {
            n = mcclellanCompile(*rdfa, cc);
        }
    }

    // We can attempt to build LBRs for infixes.
    if (!n && !is_prefix && left.graph() && onlyOneTop(*left.graph())) {
        map<u32, vector<vector<CharReach> > > triggers;
        findTriggerSequences(tbi, infixTriggers.at(left), &triggers);
        assert(contains(triggers, 0)); // single top
        n = constructLBR(*left.graph(), triggers[0], cc);
    }

    if (!n && left.graph()) {
        map<u32, vector<vector<CharReach>>> triggers;
        findTriggerSequences(tbi, infixTriggers.at(left), &triggers);
        n = constructNFA(*left.graph(), nullptr, fixed_depth_tops, triggers,
                         compress_state, cc);
    }

    if (cc.grey.roseMcClellanPrefix == 1 && is_prefix && !left.dfa()
        && left.graph()
        && (!n || !has_bounded_repeats_other_than_firsts(*n) || !is_fast(*n))) {
        auto rdfa = buildMcClellan(*left.graph(), nullptr, cc.grey);
        if (rdfa) {
            auto d = mcclellanCompile(*rdfa, cc);
            assert(d);
            n = pickImpl(move(d), move(n));
        }
    }

    return n;
}

static
void setLeftNfaProperties(NFA &n, const left_id &left) {
    depth min_width = findMinWidth(left);
    DEBUG_PRINTF("min_width=%s\n", min_width.str().c_str());
    u32 min_width_value = min_width.is_finite() ? (u32)min_width : 0;
    n.minWidth = min_width_value;

    depth max_width = findMaxWidth(left);
    DEBUG_PRINTF("max_width=%s\n", max_width.str().c_str());
    u32 max_width_value = max_width.is_finite() ? (u32)max_width : 0;
    n.maxWidth = max_width_value;

    // FIXME: NFA::maxOffset in Rose can't be found from reports as they don't
    // map to internal_report structures; it would have to come from the Rose
    // graph.
}

static
bool buildLeftfixes(const RoseBuildImpl &tbi, QueueIndexFactory &qif,
                    vector<aligned_unique_ptr<NFA>> *built_out,
                    set<u32> *no_retrigger_queues,
                    map<RoseVertex, left_build_info> *leftfix_info,
                    bool do_prefix) {
    const RoseGraph &g = tbi.g;
    const CompileContext &cc = tbi.cc;

    ue2::unordered_map<left_id, NFA *> seen;

    map<left_id, set<PredTopPair> > infixTriggers;
    findInfixTriggers(tbi, &infixTriggers);

    for (auto v : vertices_range(g)) {
        if (!g[v].left) {
            continue;
        }

        bool is_prefix = tbi.isRootSuccessor(v);

        if (do_prefix != is_prefix) {
            /* we require prefixes and then infixes */
            continue;
        }

        left_id leftfix(g[v].left);

        // Sanity check: our NFA should contain each of the tops mentioned on
        // our in-edges.
        assert(roseHasTops(g, v));

        NFA *n;
        u32 qi; // queue index, set below.
        u32 lag = g[v].left.lag;
        bool is_transient = contains(tbi.transient, leftfix);

        if (is_transient && tbi.cc.grey.roseLookaroundMasks) {
            vector<LookEntry> lookaround;
            if (makeLeftfixLookaround(tbi, v, lookaround)) {
                DEBUG_PRINTF("implementing as lookaround!\n");
                leftfix_info->emplace(v, left_build_info(lookaround));
                continue;
            }
        }

        if (contains(seen, leftfix)) {
            // NFA already built.
            n = seen[leftfix];
            qi = n->queueIndex;
            assert(qi < built_out->size());
            DEBUG_PRINTF("sharing leftfix, qi=%u\n", qi);
        } else {
            DEBUG_PRINTF("making %sleftfix\n", is_transient ? "transient " : "");

            aligned_unique_ptr<NFA> nfa;

            // Need to build NFA, which is either predestined to be a Haig (in
            // SOM mode) or could be all manner of things.
            if (leftfix.haig()) {
                nfa = goughCompile(*leftfix.haig(), tbi.ssm.somPrecision(), cc);
            }  else {
                assert(tbi.isNonRootSuccessor(v) != tbi.isRootSuccessor(v));
                nfa = makeLeftNfa(tbi, leftfix, is_prefix, is_transient,
                                  infixTriggers, cc);
            }

            if (!nfa) {
                assert(!"failed to build leftfix");
                return false;
            }

            setLeftNfaProperties(*nfa, leftfix);

            qi = qif.get_queue();
            assert(qi == built_out->size());
            nfa->queueIndex = qi;

            if (!is_prefix && !leftfix.haig() && leftfix.graph() &&
                nfaStuckOn(*leftfix.graph())) {
                DEBUG_PRINTF("%u sticks on\n", qi);
                no_retrigger_queues->insert(qi);
            }

            n = nfa.get();
            seen.insert(make_pair(leftfix, n));
            DEBUG_PRINTF("built leftfix, qi=%u\n", qi);
            built_out->push_back(move(nfa));
        }

        rose_group squash_mask = tbi.rose_squash_masks.at(leftfix);

        // Leftfixes can have stop alphabets.
        vector<u8> stop(N_CHARS, 0);
        /* haigs track som information - need more care */
        som_type som = leftfix.haig() ? SOM_LEFT : SOM_NONE;
        if (leftfix.graph()) {
            stop = findLeftOffsetStopAlphabet(*leftfix.graph(), som);
        } else if (leftfix.castle()) {
            stop = findLeftOffsetStopAlphabet(*leftfix.castle(), som);
        }

        // Infix NFAs can have bounds on their queue lengths.
        u32 max_queuelen = UINT32_MAX;
        if (!is_prefix) {
            set<ue2_literal> lits;
            for (auto u : inv_adjacent_vertices_range(v, tbi.g)) {
                for (u32 lit_id : tbi.g[u].literals) {
                    lits.insert(tbi.literals.right.at(lit_id).s);
                }
            }
            DEBUG_PRINTF("%zu literals\n", lits.size());
            max_queuelen = findMaxInfixMatches(leftfix, lits);
            if (max_queuelen < UINT32_MAX) {
                max_queuelen++;
            }
        }

        u32 max_width;
        if (is_transient) {
            depth d = findMaxWidth(leftfix);
            assert(d.is_finite());
            max_width = d;
        } else {
            max_width = 0;
        }

        u8 cm_count = 0;
        CharReach cm_cr;
        if (cc.grey.allowCountingMiracles) {
            findCountingMiracleInfo(leftfix, stop, &cm_count, &cm_cr);
        }

        leftfix_info->insert(
                make_pair(v, left_build_info(n, qi, lag, max_width,
                                             squash_mask, stop, max_queuelen,
                                             cm_count, cm_cr)));
    }

    return true;
}

static
void findSuffixTriggers(const RoseBuildImpl &tbi,
                        map<suffix_id, set<PredTopPair> > *suffixTriggers) {
    const RoseGraph &g = tbi.g;
    for (auto v : vertices_range(g)) {
        if (!g[v].suffix) {
            continue;
        }
        PredTopPair ptp(v, g[v].suffix.top);
        (*suffixTriggers)[g[v].suffix].insert(ptp);
    }
}

static
bool hasNonSmallBlockOutfix(const vector<OutfixInfo> &outfixes) {
    for (const auto &out : outfixes) {
        if (!out.in_sbmatcher) {
            return true;
        }
    }
    return false;
}

static
aligned_unique_ptr<NFA> buildOutfix(RoseBuildImpl &tbi, OutfixInfo &outfix) {
    assert(!outfix.is_dead()); // should not be marked dead.
    assert(!outfix.nfa); // should not be already built.

    const CompileContext &cc = tbi.cc;
    const ReportManager &rm = tbi.rm;

    aligned_unique_ptr<NFA> n;
    if (outfix.rdfa) {
        // Unleash the McClellan!
        n = mcclellanCompile(*outfix.rdfa, cc);
    } else if (outfix.haig) {
        // Unleash the Goughfish!
        n = goughCompile(*outfix.haig, tbi.ssm.somPrecision(), cc);
    } else if (outfix.holder) {
        NGHolder &h = *outfix.holder;
        assert(h.kind == NFA_OUTFIX);

        // Build NFA.
        if (!n) {
            const map<u32, u32> fixed_depth_tops; /* no tops */
            const map<u32, vector<vector<CharReach>>> triggers; /* no tops */
            bool compress_state = cc.streaming;
            n = constructNFA(h, &rm, fixed_depth_tops, triggers, compress_state,
                             cc);
        }

        // Try for a DFA upgrade.
        if (n && cc.grey.roseMcClellanOutfix
            && !has_bounded_repeats_other_than_firsts(*n)) {
            auto rdfa = buildMcClellan(h, &rm, cc.grey);
            if (rdfa) {
                auto d = mcclellanCompile(*rdfa, cc);
                if (d) {
                    n = pickImpl(move(d), move(n));
                }
            }
        }
    } else if (!outfix.puffettes.empty()) {
        assert(0);
    }

    if (n && tbi.cc.grey.reverseAccelerate) {
        buildReverseAcceleration(n.get(), outfix.rev_info, outfix.minWidth);
    }

    outfix.nfa = n.get();
    return n;
}

static
void prepMpv(RoseBuildImpl &tbi, vector<aligned_unique_ptr<NFA>> *built_nfas,
             size_t *historyRequired, bool *mpv_as_outfix) {
    assert(built_nfas->empty());
    *mpv_as_outfix = false;
    OutfixInfo *mpv = nullptr;

    /* assume outfixes are just above chain tails in queue indices */
    for (auto &out : tbi.outfixes) {
        if (out.is_nonempty_mpv()) {
            assert(!mpv);
            mpv = &out;
        } else {
            assert(!out.chained);
        }
    }

    if (!mpv) {
        return;
    }

    assert(mpv->chained);
    assert(!mpv->nfa);
    auto nfa = mpvCompile(mpv->puffettes, mpv->triggered_puffettes);
    assert(nfa);
    if (!nfa) {
        throw CompileError("Unable to generate bytecode.");
    }

    if (tbi.cc.grey.reverseAccelerate) {
        buildReverseAcceleration(nfa.get(), mpv->rev_info, mpv->minWidth);
    }

    u32 qi = mpv->get_queue(tbi.qif);
    assert(qi == built_nfas->size());
    nfa->queueIndex = qi;

    DEBUG_PRINTF("built mpv\n");

    if (!*historyRequired && requires_decompress_key(*nfa)) {
        *historyRequired = 1;
    }

    mpv->nfa = nfa.get();
    built_nfas->push_back(move(nfa));
    *mpv_as_outfix = !mpv->puffettes.empty();
}

static
void setOutfixProperties(NFA &n, const OutfixInfo &outfix) {
    depth min_width = outfix.minWidth;
    DEBUG_PRINTF("min_width=%s\n", min_width.str().c_str());
    u32 min_width_value = min_width.is_finite() ? (u32)min_width : 0;
    n.minWidth = min_width_value;

    depth max_width = outfix.maxWidth;
    DEBUG_PRINTF("max_width=%s\n", max_width.str().c_str());
    u32 max_width_value = max_width.is_finite() ? (u32)max_width : 0;
    n.maxWidth = max_width_value;

    DEBUG_PRINTF("max_offset=%llu\n", outfix.maxOffset);
    u32 max_offset_value = outfix.maxOffset < ~0U ? (u32)outfix.maxOffset : 0;
    n.maxOffset = max_offset_value;

    DEBUG_PRINTF("maxBAWidth=%u\n", outfix.maxBAWidth);
    if (outfix.maxBAWidth != ROSE_BOUND_INF && outfix.maxBAWidth < 256) {
        n.maxBiAnchoredWidth = verify_u8(outfix.maxBAWidth);
    }
}

static
bool prepOutfixes(RoseBuildImpl &tbi,
                  vector<aligned_unique_ptr<NFA>> *built_nfas,
                  size_t *historyRequired) {
    if (tbi.cc.grey.onlyOneOutfix && tbi.outfixes.size() > 1) {
        DEBUG_PRINTF("we have %zu outfixes, but Grey::onlyOneOutfix is set\n",
                     tbi.outfixes.size());
        throw ResourceLimitError();
    }

    assert(tbi.qif.allocated_count() == built_nfas->size());
    /* assume outfixes are just above chain tails in queue indices */
    built_nfas->reserve(tbi.outfixes.size());

    for (auto &out : tbi.outfixes) {
        if (out.chained) {
            continue; /* already done */
        }
        DEBUG_PRINTF("building outfix %zd (holder %p rdfa %p)\n",
                     &out - &tbi.outfixes[0], out.holder.get(), out.rdfa.get());
        auto n = buildOutfix(tbi, out);
        if (!n) {
            assert(0);
            return false;
        }

        setOutfixProperties(*n, out);

        u32 qi = tbi.qif.get_queue();
        assert(qi == built_nfas->size());
        n->queueIndex = qi;

        if (!*historyRequired && requires_decompress_key(*n)) {
            *historyRequired = 1;
        }

        built_nfas->push_back(move(n));
    }

    return true;
}

static
void findSuffixes(const RoseBuildImpl &tbi, QueueIndexFactory &qif,
                  map<suffix_id, u32> *suffixes) {
    const RoseGraph &g = tbi.g;

    for (auto v : vertices_range(g)) {
        if (!g[v].suffix) {
            continue;
        }

        const suffix_id s(g[v].suffix);

        DEBUG_PRINTF("vertex %zu triggers suffix %p\n", g[v].idx, s.graph());

        // We may have already built this NFA.
        if (contains(*suffixes, s)) {
            continue;
        }

        u32 queue = qif.get_queue();
        DEBUG_PRINTF("assigning %p to queue %u\n", s.graph(), queue);
        suffixes->insert(make_pair(s, queue));
    }
}

static
void setSuffixProperties(NFA &n, const suffix_id &suff,
                         const ReportManager &rm) {
    depth min_width = findMinWidth(suff);
    DEBUG_PRINTF("min_width=%s\n", min_width.str().c_str());
    u32 min_width_value = min_width.is_finite() ? (u32)min_width : 0;
    n.minWidth = min_width_value;

    depth max_width = findMaxWidth(suff);
    DEBUG_PRINTF("max_width=%s\n", max_width.str().c_str());
    u32 max_width_value = max_width.is_finite() ? (u32)max_width : 0;
    n.maxWidth = max_width_value;

    u64a max_offset = findMaxOffset(all_reports(suff), rm);
    DEBUG_PRINTF("max_offset=%llu\n", max_offset);
    u32 max_offset_value = max_offset < ~0U ? (u32)max_offset : 0;
    n.maxOffset = max_offset_value;
}

static
bool buildSuffixes(const RoseBuildImpl &tbi,
                   vector<aligned_unique_ptr<NFA>> *built_nfas,
                   map<suffix_id, u32> *suffixes,
                   set<u32> *no_retrigger_queues) {
    map<suffix_id, set<PredTopPair> > suffixTriggers;
    findSuffixTriggers(tbi, &suffixTriggers);

    for (const auto &e : *suffixes) {
        const suffix_id &s = e.first;
        const u32 queue = e.second;
        const set<PredTopPair> &s_triggers = suffixTriggers.at(s);

        map<u32, u32> fixed_depth_tops;
        findFixedDepthTops(tbi.g, s_triggers, &fixed_depth_tops);

        map<u32, vector<vector<CharReach>>> triggers;
        findTriggerSequences(tbi, s_triggers, &triggers);

        auto n = buildSuffix(tbi.rm, tbi.ssm, fixed_depth_tops, triggers,
                             s, tbi.cc);
        if (!n) {
            return false;
        }

        setSuffixProperties(*n, s, tbi.rm);

        n->queueIndex = queue;
        if (s.graph() && nfaStuckOn(*s.graph())) { /* todo: have corresponding
                                                    * haig analysis */
            assert(!s.haig());
            DEBUG_PRINTF("%u sticks on\n", queue);
            no_retrigger_queues->insert(queue);
        }

        if (built_nfas->size() <= queue) {
            built_nfas->resize(queue + 1);
        }

        (*built_nfas)[queue] = move(n);
    }

    return true;
}

static
void pad_engine_blob(build_context &bc, size_t align) {
    assert(ISALIGNED_N(bc.engine_blob_base, align));
    size_t s = bc.engine_blob.size();

    if (ISALIGNED_N(s, align)) {
        return;
    }

    bc.engine_blob.resize(s + align - s % align);
}

template<typename T>
static
u32 add_to_engine_blob(build_context &bc, const T &a) {
    static_assert(is_pod<T>::value, "should be pod");
    pad_engine_blob(bc, alignof(T));

    size_t rv = bc.engine_blob_base + bc.engine_blob.size();
    assert(rv >= bc.engine_blob_base);

    assert(ISALIGNED_N(bc.engine_blob.size(), alignof(T)));

    bc.engine_blob.resize(bc.engine_blob.size() + sizeof(a));
    memcpy(&bc.engine_blob.back() - sizeof(a) + 1, &a, sizeof(a));

    return verify_u32(rv);
}

template<typename Iter>
static
u32 add_to_engine_blob(build_context &bc, Iter b, const Iter &e) {
    using value_type = typename Iter::value_type;
    static_assert(is_pod<value_type>::value, "should be pod");
    pad_engine_blob(bc, alignof(value_type));

    size_t rv = bc.engine_blob_base + bc.engine_blob.size();
    assert(rv >= bc.engine_blob_base);

    assert(ISALIGNED_N(bc.engine_blob.size(), alignof(value_type)));

    size_t total_added_length = sizeof(*b) * distance(b, e);
    bc.engine_blob.resize(bc.engine_blob.size() + total_added_length);
    char *p = bc.engine_blob.data() + bc.engine_blob.size()
            - total_added_length;
    for (; b != e; ++b, p += sizeof(*b)) {
        memcpy(p, &*b, sizeof(*b));
    }
    assert(p - 1 == &bc.engine_blob.back());

    return verify_u32(rv);
}

static
void buildCountingMiracles(RoseBuildImpl &build, build_context &bc) {
    map<pair<CharReach, u8>, u32> pre_built;

    // To ensure compile determinism, we need to iterate over our leftfixes in
    // a stronger order than directly over bc.leftfix_info.
    vector<RoseVertex> cm_vertices;
    for (const auto &m : bc.leftfix_info) {
        if (m.second.countingMiracleCount) {
            cm_vertices.push_back(m.first);
        }
    }
    sort(begin(cm_vertices), end(cm_vertices), VertexIndexComp(build.g));

    DEBUG_PRINTF("%zu vertices with counting miracles\n", cm_vertices.size());

    for (const auto &v : cm_vertices) {
        auto &lbi = bc.leftfix_info.at(v);
        assert(lbi.countingMiracleCount);

        const CharReach &cr = lbi.countingMiracleReach;
        assert(!cr.all() && !cr.none());

        auto key = make_pair(cr, lbi.countingMiracleCount);
        if (contains(pre_built, key)) {
            lbi.countingMiracleOffset = pre_built[key];
            continue;
        }

        RoseCountingMiracle rcm;
        memset(&rcm, 0, sizeof(rcm));

        if (cr.count() == 1) {
            rcm.c = cr.find_first();
        } else {
            rcm.shufti = 1;
            int rv = shuftiBuildMasks(cr, &rcm.lo, &rcm.hi);
            if (rv == -1) {
                DEBUG_PRINTF("failed to build shufti\n");
                lbi.countingMiracleCount = 0; /* remove counting miracle */
                continue;
            }

            rcm.poison = (~cr).find_first();
        }

        rcm.count = lbi.countingMiracleCount;

        lbi.countingMiracleOffset = add_to_engine_blob(bc, rcm);
        pre_built[key] = lbi.countingMiracleOffset;
        DEBUG_PRINTF("built cm for count of %u @ %u\n", rcm.count,
                     lbi.countingMiracleOffset);
    }
}

static
bool buildNfas(RoseBuildImpl &tbi, QueueIndexFactory &qif,
               vector<aligned_unique_ptr<NFA>> *built_nfas,
               map<suffix_id, u32> *suffixes,
               map<RoseVertex, left_build_info> *leftfix_info,
               set<u32> *no_retrigger_queues, u32 *leftfixBeginQueue) {
    findSuffixes(tbi, qif, suffixes);

    if (!buildSuffixes(tbi, built_nfas, suffixes, no_retrigger_queues)) {
        return false;
    }

    *leftfixBeginQueue = qif.allocated_count();

    if (!buildLeftfixes(tbi, qif, built_nfas, no_retrigger_queues, leftfix_info,
                        true)) {
        return false;
    }

    if (!buildLeftfixes(tbi, qif, built_nfas, no_retrigger_queues, leftfix_info,
                        false)) {
        return false;
    }

    return true;
}

static
void allocateStateSpace(const NFA *nfa, const set<u32> &transient_queues,
                        RoseStateOffsets *so, NfaInfo *nfa_infos,
                        u32 *currFullStateSize, u32 *maskStateSize,
                        u32 *tStateSize) {
    u32 qi = nfa->queueIndex;
    bool transient = transient_queues.find(qi) != transient_queues.end();
    u32 stateSize = verify_u32(nfa->streamStateSize);

    u32 state_offset;
    if (transient) {
        state_offset = *tStateSize;
        *tStateSize += stateSize;
    } else {
        // Pack NFA state on to the end of the Rose state.
        state_offset = so->end;
        so->end += stateSize;
        *maskStateSize += stateSize;
    }

    nfa_infos[qi].stateOffset = state_offset;

    // Uncompressed state must be aligned.
    u32 scratchStateSize = verify_u32(nfa->scratchStateSize);
    u32 alignReq = state_alignment(*nfa);
    assert(alignReq);
    while (*currFullStateSize % alignReq) {
        (*currFullStateSize)++;
    }
    nfa_infos[qi].fullStateOffset = *currFullStateSize;
    *currFullStateSize += scratchStateSize;
}

static
void findTransientQueues(const map<RoseVertex, left_build_info> &leftfix_info,
                         set<u32> *out) {
    DEBUG_PRINTF("curating transient queues\n");
    for (const auto &rbi : leftfix_info | map_values) {
        if (rbi.transient) {
            DEBUG_PRINTF("q %u is transient\n", rbi.queue);
            out->insert(rbi.queue);
        }
    }
}

static
void updateNfaState(const vector<aligned_unique_ptr<NFA>> &built_nfas,
                    const map<RoseVertex, left_build_info> &leftfix_info,
                    RoseStateOffsets *so, NfaInfo *nfa_infos,
                    u32 *fullStateSize, u32 *nfaStateSize, u32 *tStateSize) {
    *nfaStateSize = 0;
    *tStateSize = 0;
    *fullStateSize = 0;

    set<u32> transient_queues;
    findTransientQueues(leftfix_info, &transient_queues);

    for (const auto &n : built_nfas) {
        allocateStateSpace(n.get(), transient_queues, so, nfa_infos,
                           fullStateSize, nfaStateSize, tStateSize);
    }
}

static
void buildLitBenefits(const RoseBuildImpl &tbi, RoseEngine *engine,
                      u32 base_lits_benefits_offset) {
    lit_benefits *lba = (lit_benefits *)((char *)engine
                                         + base_lits_benefits_offset);
    DEBUG_PRINTF("base offset %u\n", base_lits_benefits_offset);
    for (u32 i = 0; i < tbi.nonbenefits_base_id; i++) {
        assert(contains(tbi.final_id_to_literal, i));
        assert(tbi.final_id_to_literal.at(i).size() == 1);
        u32 lit_id = *tbi.final_id_to_literal.at(i).begin();
        const ue2_literal &s = tbi.literals.right.at(lit_id).s;
        DEBUG_PRINTF("building mask for lit %u (fid %u) %s\n", lit_id, i,
                     dumpString(s).c_str());
        assert(s.length() <= MAX_MASK2_WIDTH);
        u32 j = 0;
        for (const auto &e : s) {
            lba[i].and_mask.a8[j] = e.nocase ? 0 : CASE_BIT;
            lba[i].expected.e8[j] = e.nocase ? 0 : (CASE_BIT & e.c);
            DEBUG_PRINTF("a%02hhx e%02hhx\n", lba[i].and_mask.a8[j],
                         lba[i].expected.e8[j]);
            j++;
        }
    }
}

/* does not include history requirements for outfixes or literal matchers */
u32 RoseBuildImpl::calcHistoryRequired() const {
    u32 m = cc.grey.minHistoryAvailable;

    for (auto v : vertices_range(g)) {
        if (g[v].suffix) {
            m = MAX(m, 2); // so that history req is at least 1, for state
                           // compression.
            /* TODO: check if suffix uses state compression */
        }

        if (g[v].left) {
            const u32 lag = g[v].left.lag;
            const left_id leftfix(g[v].left);
            if (contains(transient, leftfix)) {
                u32 mv = lag + findMaxWidth(leftfix);

                // If this vertex has an event literal, we need to add one to
                // cope with it.
                if (hasLiteralInTable(v, ROSE_EVENT)) {
                    mv++;
                }

                m = MAX(m, mv);
            } else {
                /* rose will be caught up from (lag - 1), also need an extra
                 * byte behind that to find the decompression key */
                m = MAX(m, lag + 1);
                m = MAX(m, 2); // so that history req is at least 1, for state
                               // compression.
            }
        }
    }

    // Delayed literals contribute to history requirement as well.
    for (const auto &e : literals.right) {
        const u32 id = e.first;
        const auto &lit = e.second;
        if (lit.delay) {
            // If the literal is delayed _and_ has a mask that is longer than
            // the literal, we need enough history to match the whole mask as
            // well when rebuilding delayed matches.
            size_t len = std::max(lit.elength(), lit.msk.size() + lit.delay);
            ENSURE_AT_LEAST(&m, verify_u32(len));
        }

        /* Benefit checks require data is available. */
        if (literal_info.at(id).requires_benefits) {
            ENSURE_AT_LEAST(&m,
                            MIN(verify_u32(lit.elength()), MAX_MASK2_WIDTH));
        }
    }

    m = MAX(m, max_rose_anchored_floating_overlap);

    DEBUG_PRINTF("m=%u, ematcher_region_size=%u\n", m, ematcher_region_size);

    if (ematcher_region_size >= m) {
        return ematcher_region_size;
    }

    return m ? m - 1 : 0;
}

static
u32 sizeSideSuccMasks(const sidecar *stable,
                      const map<set<u32>, set<RoseVertex> > &side_succ_map) {
    if (!stable) {
        return 0;
    }

    return verify_u32((side_succ_map.size() + 1 /* for init */)
                      * sidecarEnabledSize(stable));
}

static
void populateSideSuccLists(const RoseBuildImpl &tbi, build_context &bc,
        const sidecar *stable, RoseEngine *engine, u32 base_offset,
        const map<set<u32>, set<RoseVertex> > &sidecar_succ_map) {
    const RoseGraph &g = tbi.g;

    if (!stable) {
        return;
    }

    u32 enabled_size = sidecarEnabledSize(stable);
    char *curr = (char *)engine + base_offset;

    for (const auto &e : sidecar_succ_map) {
        u32 offset = verify_u32(curr - (char *)engine);

        memset(curr, 0, enabled_size);
        /* populate the list */
        for (u32 side_id : e.first) {
            sidecarEnabledAdd(stable, (sidecar_enabled *)curr, side_id);
        }

        curr += enabled_size;

        /* update the role entries */
        for (RoseVertex v : e.second) {
            if (v == tbi.root) {
                DEBUG_PRINTF("setting root emask\n");
                engine->initSideEnableOffset = offset;
            } else {
                DEBUG_PRINTF("setting boring emask\n");
                assert(g[v].role < bc.roleTable.size());
                bc.roleTable[g[v].role].sidecarEnableOffset = offset;
            }
        }
    }

    if (!engine->initSideEnableOffset) {
        DEBUG_PRINTF("add a blank enabled for root\n");
        engine->initSideEnableOffset = verify_u32(curr - (char *)engine);
        memset(curr, 0, enabled_size);
        curr += enabled_size;
    }
}

/* Also creates a map of sidecar id set to the roles which enables that set
 */
static
void markSideEnablers(RoseBuildImpl &build,
                      map<set<u32>, set<RoseVertex> > *scmap) {
    map<RoseVertex, set<u32> > enablers;
    u32 side_id = 0;
    for (const auto &e : build.side_squash_roles) {
        for (RoseVertex v : e.second) {
            enablers[v].insert(side_id);
        }

        side_id++;
    }

    for (const auto &e : enablers) {
        (*scmap)[e.second].insert(e.first);
    }
}

#ifdef DEBUG
static UNUSED
string dumpMask(const vector<u8> &v) {
    ostringstream oss;
    for (u8 e : v) {
        oss << setfill('0') << setw(2) << hex << (unsigned int)e;
    }
    return oss.str();
}
#endif

static
bool maskFromLeftGraph(const LeftEngInfo &left, vector<u8> &msk,
                       vector<u8> &cmp) {
    const u32 lag = left.lag;
    const ReportID report = left.leftfix_report;

    DEBUG_PRINTF("leftfix with lag %u, report %u\n", lag, report);

    assert(left.graph);
    const NGHolder &h = *left.graph;
    assert(in_degree(h.acceptEod, h) == 1); // no eod reports

    // Start with the set of reporter vertices for this leftfix.
    set<NFAVertex> curr;
    for (auto u : inv_adjacent_vertices_range(h.accept, h)) {
        if (contains(h[u].reports, report)) {
            curr.insert(u);
        }
    }
    assert(!curr.empty());

    size_t i = HWLM_MASKLEN - lag - 1;
    do {
        if (curr.empty() || contains(curr, h.start)
            || contains(curr, h.startDs)) {
            DEBUG_PRINTF("end of the road\n");
            break;
        }

        set<NFAVertex> next;
        CharReach cr;
        for (NFAVertex v : curr) {
            const auto &v_cr = h[v].char_reach;
            DEBUG_PRINTF("vertex %u, reach %s\n", h[v].index,
                         describeClass(v_cr).c_str());
            cr |= v_cr;
            insert(&next, inv_adjacent_vertices(v, h));
        }
        make_and_cmp_mask(cr, &msk.at(i), &cmp.at(i));
        DEBUG_PRINTF("%zu: reach=%s, msk=%u, cmp=%u\n", i,
                     describeClass(cr).c_str(), msk[i], cmp[i]);
        curr.swap(next);
    } while (i-- > 0);

    return true;
}

static
bool maskFromLeftCastle(const LeftEngInfo &left, vector<u8> &msk,
                        vector<u8> &cmp) {
    const u32 lag = left.lag;
    const ReportID report = left.leftfix_report;

    DEBUG_PRINTF("leftfix with lag %u, report %u\n", lag, report);

    assert(left.castle);
    const CastleProto &c = *left.castle;

    depth min_width(depth::infinity());
    for (const PureRepeat &repeat : c.repeats | map_values) {
        if (contains(repeat.reports, report)) {
            min_width = min(min_width, repeat.bounds.min);
        }
    }

    DEBUG_PRINTF("castle min width for this report is %s\n",
                 min_width.str().c_str());

    if (!min_width.is_finite() || min_width == depth(0)) {
        DEBUG_PRINTF("bad min width\n");
        return false;
    }

    u32 len = min_width;
    u32 end = HWLM_MASKLEN - lag;
    for (u32 i = end; i > end - min(end, len); i--) {
        make_and_cmp_mask(c.reach(), &msk.at(i - 1), &cmp.at(i - 1));
    }

    return true;
}

static
bool maskFromLeft(const LeftEngInfo &left, vector<u8> &msk, vector<u8> &cmp) {
    if (left.lag >= HWLM_MASKLEN) {
        DEBUG_PRINTF("too much lag\n");
        return false;
    }

    if (left.graph) {
        return maskFromLeftGraph(left, msk, cmp);
    } else if (left.castle) {
        return maskFromLeftCastle(left, msk, cmp);
    }

    return false;
}

static
bool maskFromPreds(const RoseBuildImpl &tbi, const rose_literal_id &id,
                   const RoseVertex v, vector<u8> &msk, vector<u8> &cmp) {
    const RoseGraph &g = tbi.g;

    // For right now, wuss out and only handle cases with one pred.
    if (in_degree(v, g) != 1) {
        return false;
    }

    // Root successors have no literal before them.
    if (tbi.isRootSuccessor(v)) {
        return false;
    }

    // If we have a single predecessor with a short bound, we may be able to
    // fill out a mask with the trailing bytes of the previous literal. This
    // allows us to improve literals like the 'bar' in 'fo.bar'.

    RoseEdge e = *(in_edges(v, g).first);
    u32 bound = g[e].maxBound;
    if (bound != g[e].minBound || bound >= HWLM_MASKLEN) {
        return false;
    }

    bound += id.s.length();
    if (bound >= HWLM_MASKLEN) {
        return false;
    }

    DEBUG_PRINTF("bound %u\n", bound);

    RoseVertex u = source(e, g);
    if (g[u].literals.size() != 1) {
        DEBUG_PRINTF("u has %zu literals\n", g[u].literals.size());
        return false;
    }

    u32 u_lit_id = *(g[u].literals.begin());
    const rose_literal_id &u_id = tbi.literals.right.at(u_lit_id);
    DEBUG_PRINTF("u has lit: %s\n", escapeString(u_id.s).c_str());

    // Number of characters to take from the back of u's literal.
    size_t u_len = u_id.s.length();
    size_t u_sublen = min(u_len, (size_t)HWLM_MASKLEN - bound);

    size_t i = HWLM_MASKLEN - (bound + u_sublen);

    ue2_literal::const_iterator it, ite;
    for (it = u_id.s.begin() + (u_len - u_sublen), ite = u_id.s.end();
            it != ite; ++it) {
        make_and_cmp_mask(*it, &msk.at(i), &cmp.at(i));
        ++i;
    }

    return true;
}

static
bool findHamsterMask(const RoseBuildImpl &tbi, const rose_literal_id &id,
                     const rose_literal_info &info, const RoseVertex v,
                     vector<u8> &msk, vector<u8> &cmp) {
    // Start with zero masks.
    msk.assign(HWLM_MASKLEN, 0);
    cmp.assign(HWLM_MASKLEN, 0);

    // Masks can come from literal benefits (for mixed-case literals).
    if (info.requires_benefits) {
        assert(mixed_sensitivity(id.s));

        size_t j = 0;
        for (ue2_literal::const_reverse_iterator it = id.s.rbegin(),
                                                 ite = id.s.rend();
             it != ite && j < HWLM_MASKLEN; ++it, ++j) {
            size_t offset = HWLM_MASKLEN - j - 1;
            const CharReach &cr = *it;
            make_and_cmp_mask(cr, &msk[offset], &cmp[offset]);
        }
        return true;
    }

    const LeftEngInfo &left = tbi.g[v].left;
    if (left && left.lag < HWLM_MASKLEN) {
        if (maskFromLeft(left, msk, cmp)) {
            DEBUG_PRINTF("mask from a leftfix!\n");
            return true;
        }
    }

    if (id.s.length() < HWLM_MASKLEN) {
        if (maskFromPreds(tbi, id, v, msk, cmp)) {
            DEBUG_PRINTF("mask from preds!\n");
            return true;
        }
    }

    return false;
}

static
bool hamsterMaskCombine(vector<u8> &msk, vector<u8> &cmp,
                        const vector<u8> &v_msk, const vector<u8> &v_cmp) {
    assert(msk.size() == HWLM_MASKLEN && cmp.size() == HWLM_MASKLEN);
    assert(v_msk.size() == HWLM_MASKLEN && v_cmp.size() == HWLM_MASKLEN);

    u8 all_masks = 0;

    for (size_t i = 0; i < HWLM_MASKLEN; i++) {
        u8 filter = ~(cmp[i] ^ v_cmp[i]);
        msk[i] &= v_msk[i];
        msk[i] &= filter;
        cmp[i] &= filter;

        all_masks |= msk[i];
    }

    // Return false if we have no bits on in any mask elements.
    return all_masks != 0;
}

static
bool findHamsterMask(const RoseBuildImpl &tbi, const rose_literal_id &id,
                     const rose_literal_info &info,
                     vector<u8> &msk, vector<u8> &cmp) {
    if (!tbi.cc.grey.roseHamsterMasks) {
        return false;
    }

    if (!info.delayed_ids.empty()) {
        // Not safe to add masks to delayed literals at this late stage.
        return false;
    }

    size_t num = 0;
    vector<u8> v_msk, v_cmp;

    for (RoseVertex v : info.vertices) {
        if (!findHamsterMask(tbi, id, info, v, v_msk, v_cmp)) {
            DEBUG_PRINTF("no mask\n");
            return false;
        }

        if (!num++) {
            // First (or only) vertex, this becomes the mask/cmp pair.
            msk = v_msk;
            cmp = v_cmp;
        } else {
            // Multiple vertices with potentially different masks. We combine
            // them into an 'advisory' mask.
            if (!hamsterMaskCombine(msk, cmp, v_msk, v_cmp)) {
                DEBUG_PRINTF("mask went to zero\n");
                return false;
            }
        }
    }

    normaliseLiteralMask(id.s, msk, cmp);

    if (msk.empty()) {
        DEBUG_PRINTF("no mask\n");
        return false;
    }

    DEBUG_PRINTF("msk=%s, cmp=%s\n", dumpMask(msk).c_str(),
                 dumpMask(cmp).c_str());
    return true;
}

static
bool isDirectHighlander(const RoseBuildImpl &tbi,
                        const rose_literal_info &info) {
    u32 final_id = info.final_id;
    assert(final_id != MO_INVALID_IDX);

    if ((final_id & LITERAL_MDR_FLAG) == LITERAL_MDR_FLAG) {
        u32 i = final_id & ~LITERAL_MDR_FLAG;
        assert(i < tbi.mdr_reports.size());
        for (ReportID report = tbi.mdr_reports[i]; report != MO_INVALID_IDX;
             report = tbi.mdr_reports[++i]) {
            const Report &ir = tbi.rm.getReport(report);
            if (!isSimpleExhaustible(ir)) {
                return false;
            }
        }
        return true;
    } else if (final_id & LITERAL_DR_FLAG) {
        ReportID report = final_id & ~LITERAL_DR_FLAG;
        const Report &ir = tbi.rm.getReport(report);
        if (isSimpleExhaustible(ir)) {
            return true;
        }
    }

    return false;
}

// Called by isNoRunsLiteral below.
static
bool isNoRunsVertex(const RoseBuildImpl &tbi, NFAVertex u) {
    const RoseGraph &g = tbi.g;
    if (!g[u].isBoring()) {
        DEBUG_PRINTF("u=%zu is not boring\n", g[u].idx);
        return false;
    }

    if (!g[u].reports.empty()) {
        DEBUG_PRINTF("u=%zu has accept\n", g[u].idx);
        return false;
    }

    if (g[u].escapes.any()) {
        DEBUG_PRINTF("u=%zu has escapes\n", g[u].idx);
        return false;
    }

    /* TODO: handle non-root roles as well. It can't be that difficult... */

    if (!in_degree_equal_to(u, g, 1)) {
        DEBUG_PRINTF("u=%zu is not a root role\n", g[u].idx);
        return false;
    }

    RoseEdge e;
    bool exists;
    tie(e, exists) = edge_by_target(tbi.root, u, g);

    if (!exists) {
        DEBUG_PRINTF("u=%zu is not a root role\n", g[u].idx);
        return false;
    }

    if (g[e].minBound != 0 || g[e].maxBound != ROSE_BOUND_INF) {
        DEBUG_PRINTF("u=%zu has bounds from root\n", g[u].idx);
        return false;
    }

    for (const auto &oe : out_edges_range(u, g)) {
        RoseVertex v = target(oe, g);
        if (g[oe].maxBound != ROSE_BOUND_INF) {
            DEBUG_PRINTF("edge (%zu,%zu) has max bound\n", g[u].idx,
                    g[target(oe, g)].idx);
            return false;
        }
        if (g[v].left) {
            DEBUG_PRINTF("v=%zu has rose prefix\n", g[v].idx);
            return false;
        }
    }
    return true;
}

static
bool isNoRunsLiteral(const RoseBuildImpl &tbi, UNUSED const u32 id,
                     const rose_literal_info &info) {
    DEBUG_PRINTF("lit id %u\n", id);

    if (info.requires_benefits) {
        DEBUG_PRINTF("requires benefits\n"); // which would need confirm
        return false;
    }

    if (isDirectHighlander(tbi, info)) {
        DEBUG_PRINTF("highlander direct report\n");
        return true;
    }

    // Undelayed vertices.
    for (RoseVertex v : info.vertices) {
        if (!isNoRunsVertex(tbi, v)) {
            return false;
        }
    }

    // Delayed vertices.
    for (u32 d : info.delayed_ids) {
        assert(d < tbi.literal_info.size());
        const rose_literal_info &delayed_info = tbi.literal_info.at(d);
        assert(delayed_info.undelayed_id == id);
        for (RoseVertex v : delayed_info.vertices) {
            if (!isNoRunsVertex(tbi, v)) {
                return false;
            }
        }
    }

    DEBUG_PRINTF("is no-runs literal\n");
    return true;
}

void fillHamsterLiteralList(const RoseBuildImpl &tbi, rose_literal_table table,
                            vector<hwlmLiteral> *hl) {
    for (const auto &e : tbi.literals.right) {
        const u32 id = e.first;
        if (!tbi.hasFinalId(id)) {
            continue;
        }

        if (e.second.delay) {
            continue; /* delay id's are virtual-ish */
        }

        if (e.second.table != table) {
            continue; /* wrong table */
        }

        assert(id < tbi.literal_info.size());
        const rose_literal_info &info = tbi.literal_info[id];
        u32 final_id = info.final_id;
        rose_group groups = info.group_mask;
        /* Note: requires_benefits are handled in the literal entries */
        const ue2_literal &lit = e.second.s;

        DEBUG_PRINTF("lit='%s'\n", escapeString(lit).c_str());

        vector<u8> msk = e.second.msk; // copy
        vector<u8> cmp = e.second.cmp; // copy

        if (msk.empty()) {
            // Try and pick up an advisory mask.
            if (!findHamsterMask(tbi, e.second, info, msk, cmp)) {
                msk.clear(); cmp.clear();
            } else {
                DEBUG_PRINTF("picked up late mask %zu\n", msk.size());
            }
        }

        bool noruns = isNoRunsLiteral(tbi, id, info);

        if (info.requires_explode) {
            DEBUG_PRINTF("exploding lit\n");
            const vector<u8> empty_msk; // msk/cmp will be empty
            case_iter cit = caseIterateBegin(lit);
            case_iter cite = caseIterateEnd();
            for (; cit != cite; ++cit) {
                DEBUG_PRINTF("id=%u, s='%s', nocase=%d, noruns=%d msk=%s, "
                             "cmp=%s (exploded)\n",
                             final_id, escapeString(lit.get_string()).c_str(),
                             0, noruns, dumpMask(msk).c_str(),
                             dumpMask(cmp).c_str());
                hl->push_back(hwlmLiteral(*cit, false, noruns, final_id, groups,
                                          empty_msk, empty_msk));
            }
        } else {
            const std::string &s = lit.get_string();
            const bool nocase = lit.any_nocase();

            DEBUG_PRINTF("id=%u, s='%s', nocase=%d, noruns=%d, msk=%s, "
                         "cmp=%s\n",
                         final_id, escapeString(s).c_str(), (int)nocase, noruns,
                         dumpMask(msk).c_str(), dumpMask(cmp).c_str());

            if (!maskIsConsistent(s, nocase, msk, cmp)) {
                DEBUG_PRINTF("msk/cmp for literal can't match, skipping\n");
                continue;
            }

            hl->push_back(hwlmLiteral(lit.get_string(), lit.any_nocase(),
                                      noruns, final_id, groups, msk, cmp));
        }
    }
}

static
aligned_unique_ptr<HWLM> buildFloatingMatcher(const RoseBuildImpl &tbi,
                                              size_t *fsize,
                                              size_t *historyRequired,
                                              size_t *streamStateRequired) {
    *fsize = 0;

    vector<hwlmLiteral> fl;
    fl.reserve(tbi.literals.size());
    fillHamsterLiteralList(tbi, ROSE_FLOATING, &fl);
    if (fl.empty()) {
        DEBUG_PRINTF("empty floating matcher\n");
        return nullptr;
    }

    hwlmStreamingControl ctl;
    hwlmStreamingControl *ctlp;
    if (tbi.cc.streaming) {
        ctl.history_max = tbi.cc.grey.maxHistoryAvailable;
        ctl.history_min = MAX(*historyRequired,
                              tbi.cc.grey.minHistoryAvailable);
        DEBUG_PRINTF("streaming control, history max=%zu, min=%zu\n",
                     ctl.history_max, ctl.history_min);
        ctlp = &ctl;
    } else {
        ctlp = nullptr; // Null for non-streaming.
    }

    aligned_unique_ptr<HWLM> ftable =
        hwlmBuild(fl, ctlp, false, tbi.cc, tbi.getInitialGroups());
    if (!ftable) {
        throw CompileError("Unable to generate bytecode.");
    }

    if (tbi.cc.streaming) {
        DEBUG_PRINTF("literal_history_required=%zu\n",
                ctl.literal_history_required);
        DEBUG_PRINTF("literal_stream_state_required=%zu\n",
                ctl.literal_stream_state_required);
        assert(ctl.literal_history_required <= tbi.cc.grey.maxHistoryAvailable);
        *historyRequired = max(*historyRequired,
                ctl.literal_history_required);
        *streamStateRequired = ctl.literal_stream_state_required;
    }

    *fsize = hwlmSize(ftable.get());
    assert(*fsize);
    DEBUG_PRINTF("built floating literal table size %zu bytes\n", *fsize);
    return ftable;
}

namespace {
struct LongerThanLimit {
    explicit LongerThanLimit(size_t len) : max_len(len) {}
    bool operator()(const hwlmLiteral &lit) const {
        return lit.s.length() > max_len;
    }
private:
    size_t max_len;
};
}

static
aligned_unique_ptr<HWLM> buildSmallBlockMatcher(const RoseBuildImpl &tbi,
                                                size_t *sbsize) {
    *sbsize = 0;

    if (tbi.cc.streaming) {
        DEBUG_PRINTF("streaming mode\n");
        return nullptr;
    }

    u32 float_min = findMinWidth(tbi, ROSE_FLOATING);
    if (float_min > ROSE_SMALL_BLOCK_LEN) {
        DEBUG_PRINTF("floating table has large min width %u, fail\n", float_min);
        return nullptr;
    }

    vector<hwlmLiteral> lits;
    fillHamsterLiteralList(tbi, ROSE_FLOATING, &lits);
    if (lits.empty()) {
        DEBUG_PRINTF("no floating table\n");
        return nullptr;
    } else if (lits.size() == 1) {
        DEBUG_PRINTF("single floating literal, noodle will be fast enough\n");
        return nullptr;
    }

    vector<hwlmLiteral> anchored_lits;
    fillHamsterLiteralList(tbi, ROSE_ANCHORED_SMALL_BLOCK, &anchored_lits);
    if (anchored_lits.empty()) {
        DEBUG_PRINTF("no small-block anchored literals\n");
        return nullptr;
    }

    lits.insert(lits.end(), anchored_lits.begin(), anchored_lits.end());

    // Remove literals that are longer than our small block length, as they can
    // never match. TODO: improve by removing literals that have a min match
    // offset greater than ROSE_SMALL_BLOCK_LEN, which will catch anchored cases
    // with preceding dots that put them over the limit.
    lits.erase(std::remove_if(lits.begin(), lits.end(),
                              LongerThanLimit(ROSE_SMALL_BLOCK_LEN)),
               lits.end());

    if (lits.empty()) {
        DEBUG_PRINTF("no literals shorter than small block len\n");
        return nullptr;
    }

    aligned_unique_ptr<HWLM> hwlm =
        hwlmBuild(lits, nullptr, true, tbi.cc, tbi.getInitialGroups());
    if (!hwlm) {
        throw CompileError("Unable to generate bytecode.");
    }

    *sbsize = hwlmSize(hwlm.get());
    assert(*sbsize);
    DEBUG_PRINTF("built small block literal table size %zu bytes\n", *sbsize);
    return hwlm;
}

static
aligned_unique_ptr<HWLM> buildEodAnchoredMatcher(const RoseBuildImpl &tbi,
                                                 size_t *esize) {
    *esize = 0;

    vector<hwlmLiteral> el;
    fillHamsterLiteralList(tbi, ROSE_EOD_ANCHORED, &el);

    if (el.empty()) {
        DEBUG_PRINTF("no eod anchored literals\n");
        assert(!tbi.ematcher_region_size);
        return nullptr;
    }

    assert(tbi.ematcher_region_size);

    hwlmStreamingControl *ctlp = nullptr; // not a streaming case
    aligned_unique_ptr<HWLM> etable =
        hwlmBuild(el, ctlp, true, tbi.cc, tbi.getInitialGroups());
    if (!etable) {
        throw CompileError("Unable to generate bytecode.");
    }

    *esize = hwlmSize(etable.get());
    assert(*esize);
    DEBUG_PRINTF("built eod-anchored literal table size %zu bytes\n", *esize);
    return etable;
}

static
aligned_unique_ptr<sidecar> buildSideMatcher(const RoseBuildImpl &tbi,
                                             size_t *ssize) {
    *ssize = 0;

    if (tbi.side_squash_roles.empty()) {
        DEBUG_PRINTF("no sidecar\n");
        return nullptr;
    }
    assert(tbi.cc.grey.allowSidecar);

    vector<CharReach> sl;

     /* TODO: ensure useful sidecar entries only */
    for (const CharReach &cr : tbi.side_squash_roles | map_keys) {
        sl.push_back(cr);
    }

    aligned_unique_ptr<sidecar> stable = sidecarCompile(sl);
    if (!stable) {
        throw CompileError("Unable to generate bytecode.");
    }

    *ssize = sidecarSize(stable.get());
    assert(*ssize);
    DEBUG_PRINTF("built sidecar literal table size %zu bytes\n", *ssize);
    return stable;
}

// Adds a sparse iterator to the end of the iterator table, returning its
// offset.
static
u32 addIteratorToTable(build_context &bc,
                       const vector<mmbit_sparse_iter> &iter) {
    if (contains(bc.iterCache, iter)) {
        DEBUG_PRINTF("cache hit\n");
        u32 offset = bc.iterCache.at(iter);
        return offset;
    }

    u32 offset = add_to_engine_blob(bc, iter.begin(), iter.end());

    bc.iterCache.insert(make_pair(iter, offset));

    return offset;
}

static
bool hasLastByteHistoryOutEdge(const RoseGraph &g, RoseVertex v) {
    for (const auto &e : out_edges_range(v, g)) {
        if (g[e].history == ROSE_ROLE_HISTORY_LAST_BYTE) {
            return true;
        }
    }
    return false;
}

static
u32 buildLastByteIter(const RoseGraph &g, build_context &bc) {
    vector<u32> lb_roles;

    for (auto v : vertices_range(g)) {
        if (hasLastByteHistoryOutEdge(g, v)) {
            u32 role = g[v].role;
            assert(role < bc.roleTable.size());
            lb_roles.push_back(bc.roleTable[role].stateIndex);
        }
    }

    if (lb_roles.empty()) {
        return 0; /* invalid offset */
    }

    vector<mmbit_sparse_iter> iter;
    mmbBuildSparseIterator(iter, lb_roles, bc.numStates);
    return addIteratorToTable(bc, iter);
}

#ifdef DEBUG
static
const char *describeHistory(RoseRoleHistory history) {
    switch (history) {
    case ROSE_ROLE_HISTORY_NONE:
        return "NONE";
    case ROSE_ROLE_HISTORY_ANCH:
        return "ANCH (previous role at fixed offset)";
    case ROSE_ROLE_HISTORY_LAST_BYTE:
        return "LAST_BYTE (previous role matches only at EOD)";
    case ROSE_ROLE_HISTORY_INVALID:
        return "INVALID";
    }
    assert(0);
    return "UNKNOWN";
}
#endif

static
u32 calcNfaSize(const vector<aligned_unique_ptr<NFA>> &nfas) {
    size_t nfas_size = 0;

    for (const auto &n : nfas) {
        nfas_size += ROUNDUP_CL(n->length);
    }
    return verify_u32(nfas_size);
}

static
void enforceEngineSizeLimit(const NFA *n, const size_t nfa_size, const Grey &grey) {
    // Global limit.
    if (nfa_size > grey.limitEngineSize) {
        throw ResourceLimitError();
    }

    // Type-specific limit checks follow.

    if (isDfaType(n->type)) {
        if (nfa_size > grey.limitDFASize) {
            throw ResourceLimitError();
        }
    } else if (isNfaType(n->type)) {
        if (nfa_size > grey.limitNFASize) {
            throw ResourceLimitError();
        }
    } else if (isLbrType(n->type)) {
        if (nfa_size > grey.limitLBRSize) {
            throw ResourceLimitError();
        }
    }
}

/* copies nfas into the final engine and updates role to reflect nfa offset */
static
u32 copyInNFAs(const RoseBuildImpl &tbi, vector<RoseRole> *roleTable,
               const vector<aligned_unique_ptr<NFA>> &built_nfas,
               const set<u32> &no_retrigger_queues, NfaInfo *infos,
               u32 base_nfa_offset,
               const map<suffix_id, u32> &suffixes, char *ptr) {
    const RoseGraph &g = tbi.g;
    const CompileContext &cc = tbi.cc;

    // Enforce engine count resource limit.
    if (built_nfas.size() > cc.grey.limitRoseEngineCount) {
        throw ResourceLimitError();
    }

    vector<u32> suffix_base(built_nfas.size());
    vector<bool> classic_top(built_nfas.size(), false);

    for (u32 i = 0; i < built_nfas.size(); i++) {
        const NFA *n = built_nfas[i].get();

        // Enforce individual engine size limit.
        enforceEngineSizeLimit(n, n->length, cc.grey);

        DEBUG_PRINTF("copying in nfa %u: len=%u, offset=%u\n", i, n->length,
                     base_nfa_offset);

        memcpy(ptr + base_nfa_offset, n, n->length);
        suffix_base[i] = base_nfa_offset;

        if (!isMultiTopType(n->type)) {
            classic_top[i] = true;
        }

        infos[i].nfaOffset = base_nfa_offset;
        if (contains(no_retrigger_queues, i)) {
            infos[i].no_retrigger = 1;
        }
        base_nfa_offset += ROUNDUP_CL(n->length);
    }

    /* Write NFA indices into RoseRole structures for suffix NFAs */
    for (auto v : vertices_range(g)) {
        if (!g[v].suffix) {
            continue;
        }

        u32 nfa_index = suffixes.at(g[v].suffix);
        assert(nfa_index < suffix_base.size());

        assert(g[v].role < roleTable->size());
        RoseRole &tr = (*roleTable)[g[v].role];
        tr.suffixOffset = suffix_base[nfa_index];

        // DFAs/Puffs have no MQE_TOP_N support, so they get a classic TOP
        // event.
        if (classic_top[nfa_index]) {
            assert(!g[v].suffix.graph || onlyOneTop(*g[v].suffix.graph));
            tr.suffixEvent = MQE_TOP;
        } else {
            assert(!g[v].suffix.haig);
            u32 top = (u32)MQE_TOP_FIRST + g[v].suffix.top;
            assert(top < MQE_INVALID);
            tr.suffixEvent = top;
        }

        /* mark suffixes triggered by etable literals */
        if (tbi.isInETable(v)) {
            infos[nfa_index].eod = 1;
        }
    }

    return base_nfa_offset;
}

static
u32 findMinFloatingLiteralMatch(const RoseBuildImpl &tbi) {
    const RoseGraph &g = tbi.g;
    u32 minWidth = ROSE_BOUND_INF;
    for (auto v : vertices_range(g)) {
        if (tbi.isAnchored(v) || tbi.isVirtualVertex(v)) {
            DEBUG_PRINTF("skipping %zu anchored or root\n", g[v].idx);
            continue;
        }

        u32 w = g[v].min_offset;
        DEBUG_PRINTF("%zu m_o = %u\n", g[v].idx, w);

        if (w < minWidth) {
            minWidth = w;
        }
    }

    return minWidth;
}

static
vector<RoseTrigger> buildRoseTriggerList(const RoseGraph &g, RoseVertex u,
                         const map<RoseVertex, left_build_info> &leftfix_info) {
    // Terminator struct that marks the end of each role's trigger list.
    RoseTrigger terminator;
    memset(&terminator, 0, sizeof(RoseTrigger));
    terminator.queue = MO_INVALID_IDX;
    terminator.event = MQE_INVALID;
    terminator.cancel_prev_top = false;

    vector<RoseTrigger> rv;

    for (const auto &e : out_edges_range(u, g)) {
        RoseVertex v = target(e, g);
        if (!g[v].left) {
            continue;
        }

        assert(contains(leftfix_info, v));
        const left_build_info &rbi = leftfix_info.at(v);
        if (rbi.has_lookaround) {
            continue;
        }
        assert(rbi.nfa);

        // DFAs have no TOP_N support, so they get a classic MQE_TOP event.
        u32 top;
        if (!isMultiTopType(rbi.nfa->type)) {
            assert(num_tops(g[v].left) == 1);
            top = MQE_TOP;
        } else {
            top = (enum mqe_event)((u32)MQE_TOP_FIRST + g[e].rose_top);
            assert(top < MQE_INVALID);
        }

        rv.push_back(terminator);
        RoseTrigger &trigger = rv.back();
        trigger.queue = rbi.nfa->queueIndex;
        trigger.event = top;
        trigger.cancel_prev_top = g[e].rose_cancel_prev_top;
    }

    if (rv.empty()) {
        return rv;
    }

    sort(rv.begin(), rv.end(), RoseTriggerOrdering());
    rv.erase(unique(rv.begin(), rv.end(), RoseTriggerEquality()), rv.end());

    rv.push_back(terminator);

    return rv;
}

static
void buildRoseTriggerLists(const RoseBuildImpl &tbi, build_context &bc) {
    const RoseGraph &g = tbi.g;
    for (auto u : vertices_range(g)) {
        if (tbi.isAnyStart(u) || g[u].literals.empty()
            || tbi.hasDirectFinalId(u)) {
            continue;
        }

        assert(g[u].role < bc.roleTable.size());
        RoseRole &tr = bc.roleTable.at(g[u].role);

        vector<RoseTrigger> trigs = buildRoseTriggerList(g, u, bc.leftfix_info);

        if (!trigs.empty()) {
            assert(trigs.size() != 1); /* at min should be trig + term */
            tr.infixTriggerOffset = add_to_engine_blob(bc, trigs.begin(),
                                                       trigs.end());
        }
    }
}

static
void buildSuffixEkeyLists(const RoseBuildImpl &tbi, build_context &bc,
                          const QueueIndexFactory &qif,
                          const map<suffix_id, u32> &suffixes,
                          vector<u32> *out) {
    out->resize(qif.allocated_count());

    map<u32, vector<u32> > qi_to_ekeys; /* for determinism */

    for (const auto &e : suffixes) {
        const suffix_id &s = e.first;
        u32 qi = e.second;
        set<u32> ekeys = reportsToEkeys(all_reports(s), tbi.rm);

        if (!ekeys.empty()) {
            qi_to_ekeys[qi] = {ekeys.begin(), ekeys.end()};
        }
    }

    /* for each outfix also build elists */
    for (const auto &outfix : tbi.outfixes) {
        assert(outfix.nfa);
        u32 qi = outfix.nfa->queueIndex;
        set<u32> ekeys = reportsToEkeys(all_reports(outfix), tbi.rm);

        if (!ekeys.empty()) {
            qi_to_ekeys[qi] = {ekeys.begin(), ekeys.end()};
        }
    }

    for (auto &e : qi_to_ekeys) {
        assert(!e.second.empty());
        e.second.push_back(INVALID_EKEY); /* terminator */
        (*out)[e.first] = add_to_engine_blob(bc, e.second.begin(),
                                             e.second.end());
    }
}

static
bool hasMpvTrigger(const set<u32> &reports, const ReportManager &rm) {
    for (u32 r : reports) {
        if (rm.getReport(r).type == INTERNAL_ROSE_CHAIN) {
            return true;
        }
    }

    return false;
}

static
bool anyEndfixMpvTriggers(const RoseBuildImpl &tbi) {
    const RoseGraph &g = tbi.g;
    ue2::unordered_set<suffix_id> done;

    /* suffixes */
    for (auto v : vertices_range(g)) {
        if (!g[v].suffix) {
            continue;
        }
        if (contains(done, g[v].suffix)) {
            continue; /* already done */
        }
        done.insert(g[v].suffix);

        if (hasMpvTrigger(all_reports(g[v].suffix), tbi.rm)) {
            return true;
        }
    }

    /* outfixes */
    for (const auto &out : tbi.outfixes) {
        assert(out.nfa);
        if (hasMpvTrigger(all_reports(out), tbi.rm)) {
            return true;
        }
    }

    return false;
}

static
bool hasInternalReport(const set<ReportID> &reports, const ReportManager &rm) {
    for (ReportID r : reports) {
        if (!isExternalReport(rm.getReport(r))) {
            return true;
        }
    }
    return false;
}

static
void populateNfaInfoBasics(NfaInfo *infos, const vector<OutfixInfo> &outfixes,
                           const ReportManager &rm,
                           const map<suffix_id, u32> &suffixes,
                           const vector<u32> &ekeyListOffsets) {
    for (const auto &out : outfixes) {
        assert(out.nfa);
        const u32 qi = out.nfa->queueIndex;

        infos[qi].in_sbmatcher = out.in_sbmatcher;
        if (!hasInternalReport(all_reports(out), rm)) {
            infos[qi].only_external = 1;
        }

        infos[qi].ekeyListOffset = ekeyListOffsets[qi];
    }

    for (const auto &e : suffixes) {
        const suffix_id &s = e.first;
        u32 qi = e.second;

        if (!hasInternalReport(all_reports(s), rm)) {
            infos[qi].only_external = 1;
        }

        infos[qi].ekeyListOffset = ekeyListOffsets[qi];
    }
}

struct DerivedBoundaryReports {
    explicit DerivedBoundaryReports(const BoundaryReports &boundary) {
        insert(&report_at_0_eod_full, boundary.report_at_0_eod);
        insert(&report_at_0_eod_full, boundary.report_at_eod);
        insert(&report_at_0_eod_full, boundary.report_at_0);
    }
    set<ReportID> report_at_0_eod_full;
};

static
void reserveBoundaryReports(const BoundaryReports &boundary,
                            const DerivedBoundaryReports &dboundary,
                            RoseBoundaryReports *out, u32 *currOffset) {
    u32 curr = *currOffset;
    curr = ROUNDUP_N(curr, alignof(ReportID));
    memset(out, 0, sizeof(*out));

    /* report lists are + 1 in size due to terminator */
    if (!boundary.report_at_eod.empty()) {
        out->reportEodOffset = curr;
        curr += sizeof(ReportID) * (boundary.report_at_eod.size() + 1);
    }
    if (!boundary.report_at_0.empty()) {
        out->reportZeroOffset = curr;
        curr += sizeof(ReportID) * (boundary.report_at_0.size() + 1);
    }
    if (!dboundary.report_at_0_eod_full.empty()) {
        out->reportZeroEodOffset = curr;
        curr += sizeof(ReportID) * (dboundary.report_at_0_eod_full.size() + 1);
    }

    DEBUG_PRINTF("report ^:  %zu\n", boundary.report_at_0.size());
    DEBUG_PRINTF("report $:  %zu\n", boundary.report_at_eod.size());
    DEBUG_PRINTF("report ^$: %zu\n", dboundary.report_at_0_eod_full.size());

    *currOffset = curr;
}

static
void fillInBoundaryReports(RoseEngine *engine, u32 offset,
                           const set<ReportID> &rl) {
    if (rl.empty()) {
        return;
    }

    u32 *out = (u32 *)((char *)engine + offset);
    assert(ISALIGNED(out));

    for (ReportID r : rl) {
        *out = r;
        ++out;
    }

    *out = MO_INVALID_IDX;
}

static
void populateBoundaryReports(RoseEngine *engine,
                             const BoundaryReports &boundary,
                             const DerivedBoundaryReports &dboundary,
                             const RoseBoundaryReports &offsets) {
    engine->boundary.reportEodOffset = offsets.reportEodOffset;
    engine->boundary.reportZeroOffset = offsets.reportZeroOffset;
    engine->boundary.reportZeroEodOffset = offsets.reportZeroEodOffset;

    fillInBoundaryReports(engine, offsets.reportEodOffset,
                          boundary.report_at_eod);
    fillInBoundaryReports(engine, offsets.reportZeroOffset,
                          boundary.report_at_0);
    fillInBoundaryReports(engine, offsets.reportZeroEodOffset,
                          dboundary.report_at_0_eod_full);
}

static
void fillInReportInfo(RoseEngine *engine, u32 reportOffset,
                      const ReportManager &rm, const vector<Report> &reports) {
    internal_report *dest = (internal_report *)((char *)engine + reportOffset);
    engine->intReportOffset = reportOffset;
    engine->intReportCount = (u32)reports.size();

    assert(ISALIGNED(dest));

    for (const auto &report : reports) {
        writeInternalReport(report, rm, dest++);
    }

    DEBUG_PRINTF("%zu reports of size %zu\n", reports.size(),
                 sizeof(internal_report));
}

static
void populateInvDkeyTable(char *ptr, const ReportManager &rm) {
    vector<ReportID> table = rm.getDkeyToReportTable();
    memcpy(ptr, table.data(), byte_length(table));
}

static
bool hasSimpleReports(const vector<Report> &reports) {
    auto it = find_if(reports.begin(), reports.end(), isComplexReport);

    DEBUG_PRINTF("runtime has %scomplex reports\n",
                 it == reports.end() ? "no " : "");
    return it == reports.end();
}

static
void prepSomRevNfas(const SomSlotManager &ssm, u32 *rev_nfa_table_offset,
                    vector<u32> *nfa_offsets, u32 *currOffset) {
    const deque<aligned_unique_ptr<NFA>> &nfas = ssm.getRevNfas();

    *currOffset = ROUNDUP_N(*currOffset, alignof(u32));
    *rev_nfa_table_offset = *currOffset;
    *currOffset += sizeof(u32) * nfas.size();

    *currOffset = ROUNDUP_CL(*currOffset);
    for (const auto &n : nfas) {
        u32 bs_offset;
        bs_offset = *currOffset;
        nfa_offsets->push_back(bs_offset);
        *currOffset += ROUNDUP_CL(n->length);
        /* note: som rev nfas don't need a queue assigned as only run in block
         * mode reverse */
    }

    assert(nfa_offsets->size() == nfas.size());
}

static
void fillInSomRevNfas(RoseEngine *engine, const SomSlotManager &ssm,
                      u32 rev_nfa_table_offset,
                      const vector<u32> &nfa_offsets) {
    const deque<aligned_unique_ptr<NFA>> &nfas = ssm.getRevNfas();
    assert(nfa_offsets.size() == nfas.size());

    engine->somRevCount = (u32)nfas.size();
    engine->somRevOffsetOffset = rev_nfa_table_offset;

    if (nfas.empty()) {
        return;
    }

    char *out = (char *)engine + rev_nfa_table_offset;
    size_t table_size = sizeof(u32) * nfa_offsets.size();
    memcpy(out, nfa_offsets.data(), table_size);
    out = (char *)engine + ROUNDUP_CL(rev_nfa_table_offset + table_size);

    // Write the SOM reverse NFAs into place.
    UNUSED size_t i = 0;
    for (const auto &n : nfas) {
        assert(n != nullptr);
        assert(out == (char *)engine + nfa_offsets[i]);

        memcpy(out, n.get(), n->length);
        out += ROUNDUP_CL(n->length);
        DEBUG_PRINTF("wrote som rev nfa with len %u\n", n->length);
        ++i;
    }
}

static
vector<const rose_literal_info *>
getLiteralInfoByFinalId(const RoseBuildImpl &build, u32 final_id) {
    vector<const rose_literal_info *> out;

    const auto &final_id_to_literal = build.final_id_to_literal;
    assert(contains(final_id_to_literal, final_id));

    const auto &lits = final_id_to_literal.find(final_id)->second;
    assert(!lits.empty());

    for (const auto &lit_id : lits) {
        const rose_literal_info &li = build.literal_info[lit_id];
        assert(li.final_id == final_id);
        out.push_back(&li);
    }

    return out;
}

static
void buildRootRoleTable(const RoseBuildImpl &tbi, u32 roleTableOffset,
                        vector<RoseLiteral> &literalTable,
                        vector<u32> *rootRoleTable) {
    for (u32 id = 0; id < literalTable.size(); id++) {
        RoseLiteral &tl = literalTable[id];
        const rose_literal_info &lit_info =
            **getLiteralInfoByFinalId(tbi, id).begin();
        const auto &vertices = lit_info.vertices;

        tl.rootRoleOffset = verify_u32(rootRoleTable->size());
        tl.rootRoleCount = 0;

        for (RoseVertex v : vertices) {
            if (tbi.isRootSuccessor(v)) {
                if (tbi.hasDirectFinalId(v)) {
                    DEBUG_PRINTF("[skip root role %u as direct]\n",
                                 tbi.g[v].role);
                    continue;
                }
                assert(tbi.isRootSuccessor(v));
                u32 role_offset
                    = roleTableOffset + tbi.g[v].role * sizeof(RoseRole);
                rootRoleTable->push_back(role_offset);
                tl.rootRoleCount++;
                DEBUG_PRINTF("root role %u\n", tbi.g[v].role);
            }
        }

        if (!tl.rootRoleCount) {
            tl.rootRoleOffset = 0;
        } else if (tl.rootRoleCount > 1) {
            // Sort the entries for this literal by role index
            vector<u32>::iterator begin = rootRoleTable->begin()
                                        + tl.rootRoleOffset;
            vector<u32>::iterator end = begin + tl.rootRoleCount;
            sort(begin, end);
        } else if (tl.rootRoleCount == 1) {
            /* if there is only one root role, the rose literal stores the
             * offset directly */
            tl.rootRoleOffset = (*rootRoleTable)[tl.rootRoleOffset];
        }

        DEBUG_PRINTF("literal %u: %u root roles, starting from idx=%u\n", id,
                     tl.rootRoleCount, tl.rootRoleOffset);
    }
}

static
void buildActiveLeftIter(const vector<LeftNfaInfo> &leftTable,
                         vector<mmbit_sparse_iter> &out) {
    vector<u32> keys;
    for (size_t i = 0; i < leftTable.size(); i++) {
        if (!leftTable[i].transient) {
            DEBUG_PRINTF("rose %zu is active\n", i);
            keys.push_back(verify_u32(i));
        }
    }

    DEBUG_PRINTF("%zu active roses\n", keys.size());

    if (keys.empty()) {
        out.clear();
        return;
    }

    mmbBuildSparseIterator(out, keys, leftTable.size());
}

static
bool hasEodAnchors(const RoseBuildImpl &tbi,
                   const vector<aligned_unique_ptr<NFA>> &built_nfas,
                   u32 outfixEndQueue) {
    assert(outfixEndQueue <= built_nfas.size());
    for (u32 i = 0; i < outfixEndQueue; i++) {
        if (nfaAcceptsEod(built_nfas[i].get())) {
            DEBUG_PRINTF("outfix has eod\n");
            return true;
        }
    }

    if (tbi.eod_event_literal_id != MO_INVALID_IDX) {
        DEBUG_PRINTF("eod is an event to be celebrated\n");
        return true;
    }
    for (auto v : vertices_range(tbi.g)) {
        if (tbi.g[v].eod_accept) {
            DEBUG_PRINTF("literally report eod\n");
            return true;
        }
        if (tbi.g[v].suffix && has_eod_accepts(tbi.g[v].suffix)) {
            DEBUG_PRINTF("eod suffix\n");
            return true;
        }
    }
    DEBUG_PRINTF("yawn\n");
    return false;
}

static
void fetchEodAnchors(map<ReportID, vector<RoseEdge> > &eods,
                     const RoseGraph &g) {
    for (auto v : vertices_range(g)) {
        if (!g[v].eod_accept) {
            continue;
        }

        DEBUG_PRINTF("vertex %zu (with %zu preds) fires on EOD\n", g[v].idx,
                     in_degree(v, g));

        assert(!g[v].reports.empty());
        for (const auto r : g[v].reports) {
            // In-edges go into eod list.
            for (const auto &e : in_edges_range(v, g)) {
                eods[r].push_back(e);
            }
        }
    }
}

/* creates (and adds to rose) a sparse iterator visiting pred states/roles,
 * returns a pair:
 * - the offset of the itermap
 * - the offset for the sparse iterator.
 */
static
pair<u32, u32> addPredSparseIter(build_context &bc,
                      const map<u32, vector<RoseIterRole> > &predStates) {
    vector<u32> keys;
    for (u32 k : predStates | map_keys) {
        keys.push_back(k);
    }

    vector<mmbit_sparse_iter> iter;
    mmbBuildSparseIterator(iter, keys, bc.numStates);
    assert(!iter.empty());
    DEBUG_PRINTF("iter size = %zu\n", iter.size());

    // Build mapping tables and add to iter table
    u32 iterOffset = addIteratorToTable(bc, iter);

    vector<RoseIterMapping> itermap;
    for (const auto &p : predStates) {
        u32 iterRoleOffset = add_to_engine_blob(bc, p.second.begin(),
                                                p.second.end());
        itermap.push_back(RoseIterMapping());
        itermap.back().offset = iterRoleOffset;
        itermap.back().count = verify_u32(p.second.size());
    }
    u32 iterMapOffset = add_to_engine_blob(bc, itermap.begin(), itermap.end());

    return make_pair(iterMapOffset, iterOffset);
}

static
void createPred(const RoseBuildImpl &tbi, build_context &bc,
                const RoseEdge &e, vector<RosePred> &predTable) {
    const RoseGraph &g = tbi.g;

    DEBUG_PRINTF("building pred %zu of type %s\n", predTable.size(),
                 describeHistory(g[e].history));
    RoseVertex u = source(e, g);
    RoseVertex v = target(e, g);

    u32 lit_length = 0;
    if (!g[v].eod_accept) {
        // Use the minimum literal length.
        lit_length = verify_u32(tbi.minLiteralLen(v));
    }

    bc.rolePredecessors[g[v].role].push_back(verify_u32(predTable.size()));

    predTable.push_back(RosePred());
    RosePred &tp = predTable.back();
    memset(&tp, 0, sizeof(tp));
    tp.role = g[u].role;
    tp.minBound = g[e].minBound + lit_length;
    tp.maxBound = g[e].maxBound == ROSE_BOUND_INF ? ROSE_BOUND_INF
                                                 : g[e].maxBound + lit_length;

    // Find the history scheme appropriate to this edge. Note that these may be
    // updated later, as the history collected by the predecessor role is
    // dependent on all its out edges.
    tp.historyCheck = g[e].history;
    if (tp.historyCheck == ROSE_ROLE_HISTORY_ANCH) {
        assert(g[u].max_offset != ROSE_BOUND_INF);
        /* pred role does not need to know about history scheme */
        DEBUG_PRINTF("absing (%u,%u + %u) u%u/%zu v%u/%zu\n", tp.minBound,
                     tp.maxBound, g[u].max_offset, g[u].role, g[u].idx,
                     g[v].role, g[v].idx);
        tp.minBound += g[u].max_offset; /* make absolute */
        if (tp.maxBound != ROSE_BOUND_INF) {
            tp.maxBound += g[u].max_offset; /* make absolute */
        }
    }

    if (tp.historyCheck == ROSE_ROLE_HISTORY_NONE) {
        tp.minBound = 0;
    }

    DEBUG_PRINTF("built pred %zu of %u %u %hhu:%s\n", predTable.size() - 1,
                 tp.minBound, tp.maxBound, tp.historyCheck,
                 describeHistory((RoseRoleHistory)tp.historyCheck));
}

/* returns a pair containing the iter map offset and iter offset */
static
pair<u32, u32> buildEodAnchorRoles(RoseBuildImpl &tbi, build_context &bc,
                                   vector<RosePred> &predTable) {
    const RoseGraph &g = tbi.g;
    map<ReportID, vector<RoseEdge> > eods;
    fetchEodAnchors(eods, g);

    if (eods.empty()) {
        DEBUG_PRINTF("no EOD anchors\n");
        return {0, 0};
    }

    // pred state id -> role/pred entries
    map<u32, vector<RoseIterRole> > predStates;

    for (const auto &er : eods) {
        // Create a role to fire this particular report.
        DEBUG_PRINTF("creating EOD accept role %zu for report %u\n",
                     bc.roleTable.size(), er.first);
        bc.roleTable.push_back(RoseRole());
        RoseRole &tr = bc.roleTable.back();
        memset(&tr, 0, sizeof(tr));
        tr.stateIndex = MMB_INVALID;
        tr.predOffset = ROSE_OFFSET_INVALID;
        tr.reportId = er.first;
        tr.flags = ROSE_ROLE_FLAG_ACCEPT_EOD;

        // Collect the state IDs of this report's vertices to add to the EOD
        // sparse iterator, creating pred entries appropriately.
        for (const auto &e : er.second) {
            RoseVertex v = source(e, g);
            DEBUG_PRINTF("vertex %zu has role %u\n", g[v].idx, g[v].role);
            assert(g[v].role < bc.roleTable.size());
            RoseRole &predRole = bc.roleTable[g[v].role];

            createPred(tbi, bc, e, predTable);
            const RosePred &tp = predTable.back();

            RoseIterRole ir = {
                (u32)(bc.roleTable.size() - 1),
                (u32)(predTable.size() - 1)
            };
            predStates[predRole.stateIndex].push_back(ir);

            if (out_degree(v, g) == 1 && tp.minBound == 0 && tp.maxBound == 0) {
                // Since it leads ONLY to an EOD accept with bounds (0, 0), we
                // can tag this role with the "must match at end of block"
                // flag.
                DEBUG_PRINTF("flagging role %u as ONLY_AT_END\n", g[v].role);

                /* There is no pointing enforcing this check at runtime if
                 * the predRole is only fired by eod event literal */
                if (g[v].literals.size() != 1
                    || *g[v].literals.begin() != tbi.eod_event_literal_id) {
                    predRole.flags |= ROSE_ROLE_FLAG_ONLY_AT_END;
                }
            }
            predRole.flags |= ROSE_ROLE_FLAG_PRED_OF_EOD;
        }
    }

    return addPredSparseIter(bc, predStates);
}

static
void buildSideEntriesAndIters(const RoseBuildImpl &tbi, build_context &bc,
                              const set<RoseVertex> &squash_roles,
                              vector<RoseSide> &sideTable) {
    const RoseGraph &g = tbi.g;

    sideTable.push_back(RoseSide()); /* index in array gives an implicit id */
    RoseSide &tsb = sideTable.back();
    memset(&tsb, 0, sizeof(tsb));

    if (squash_roles.empty()) {
        return;
    }

    set<RoseVertex> squashed_succ;

    // Build a vector of the roles' state IDs
    vector<u32> states;
    for (RoseVertex v : squash_roles) {
        assert(g[v].role < bc.roleTable.size());
        const RoseRole &tr = bc.roleTable[g[v].role];
        DEBUG_PRINTF("considering role %u, state index %u\n", g[v].role,
                     tr.stateIndex);
        assert(tr.stateIndex != MMB_INVALID);

        states.push_back(tr.stateIndex);
        DEBUG_PRINTF("side %zu squashes state index %u/role %u\n",
                     sideTable.size() - 1, tr.stateIndex, g[v].role);

        /* we cannot allow groups to be squashed if the source vertex is in an
         * anchored table due to ordering issue mean that a literals cannot
         * set groups */
        if (tbi.isAnchored(v) && g[v].max_offset != 1) {
            DEBUG_PRINTF("%u has anchored table pred no squashy\n", g[v].role);
            continue;
        }

        DEBUG_PRINTF("role %u is fine to g squash\n", g[v].role);

        for (auto w : adjacent_vertices_range(v, g)) {
            if (in_degree(w, g) == 1) { /* TODO: improve: check that each pred
                                         * is in id's squash role */
                squashed_succ.insert(w);
            }
        }
    }

    // Build sparse iterators and add to table.
    assert(!states.empty());

    vector<mmbit_sparse_iter> iter;
    mmbBuildSparseIterator(iter, states, bc.numStates);
    assert(!iter.empty());
    tsb.squashIterOffset = addIteratorToTable(bc, iter);

    // Build a mask of groups.
    rose_group squash_groups = 0;
    for (u32 i = 0; i < ROSE_GROUPS_MAX; i++) {
        if (!contains(tbi.group_to_literal, i)) {
            continue;
        }

        DEBUG_PRINTF("checking group %u for %zu's squash mask\n", i,
                     sideTable.size() - 1);

        const set<u32> &group_lits = tbi.group_to_literal.find(i)->second;

        /* check for each literal in this group if it is squashed by this
         * sidecar escape */
        for (u32 lit : group_lits) {
            DEBUG_PRINTF("inspecting lit %u\n", lit);
            const rose_literal_info &this_info = tbi.literal_info.at(lit);

            /* check that all roles belonging to this literal are squashed */
            for (RoseVertex v : this_info.vertices) {
                DEBUG_PRINTF("checking if role is squashed %u...\n", g[v].role);
                if (squashed_succ.find(v) != squashed_succ.end()) {
                    continue;
                }

                DEBUG_PRINTF("...role not taken %u\n", g[v].role);

                /* if the literal is length 1 and anchored (0,0) when can ignore
                 * it as any matching must have happened before the side lit
                 * arrived */
                if (g[v].max_offset == 1) {
                    DEBUG_PRINTF("we can ignore this role as 1st byte only\n");
                    continue;
                }

                goto fail_group;
            }
        }

        continue;

    fail_group:
        DEBUG_PRINTF("group %u is not squashed\n", i);
        /* we need to keep this group active */
        squash_groups |= 1ULL << i;
    }

    DEBUG_PRINTF("%zu group squash mask: %016llx\n", sideTable.size() - 1,
                 squash_groups);
    tsb.squashGroupMask = squash_groups;
}

// Construct sparse iterators for squashes
static
void buildSideTable(const RoseBuildImpl &build, build_context &bc,
                    vector<RoseSide> &sideTable) {
    for (const auto &e : build.side_squash_roles) {
        buildSideEntriesAndIters(build, bc, e.second, sideTable);
    }
}

static
void fillLookaroundTables(char *look_base, char *reach_base,
                          const vector<LookEntry> &look_vec) {
    DEBUG_PRINTF("%zu lookaround table entries\n", look_vec.size());

    s8 *look = (s8 *)look_base;
    u8 *reach = (u8 *)reach_base; // base for 256-bit bitvectors

    for (const auto &le : look_vec) {
        *look = verify_s8(le.offset);
        const CharReach &cr = le.reach;

        assert(cr.any()); // Should be at least one character!
        fill_bitvector(cr, reach);

        ++look;
        reach += REACH_BITVECTOR_LEN;
    }
}

static
bool hasBoundaryReports(const BoundaryReports &boundary) {
    if (!boundary.report_at_0.empty()) {
        DEBUG_PRINTF("has boundary reports at 0\n");
        return true;
    }
    if (!boundary.report_at_0_eod.empty()) {
        DEBUG_PRINTF("has boundary reports at 0 eod\n");
        return true;
    }
    if (!boundary.report_at_eod.empty()) {
        DEBUG_PRINTF("has boundary reports at eod\n");
        return true;
    }
    DEBUG_PRINTF("no boundary reports\n");
    return false;
}

static
bool needsSidecarCatchup(const RoseBuildImpl &build, u32 id) {
    const RoseGraph &g = build.g;

    for (RoseVertex v : build.literal_info.at(id).vertices) {
        if (g[v].escapes.any()) {
            return true;
        }

        for (RoseVertex u : inv_adjacent_vertices_range(v, g)) {
            if (g[u].escapes.any()) {
                return true;
            }
        }
    }

    return false;
}

static
void createLiteralEntry(const RoseBuildImpl &tbi, build_context &bc,
                        vector<RoseLiteral> &literalTable) {
    const u32 final_id = verify_u32(literalTable.size());
    assert(contains(tbi.final_id_to_literal, final_id));
    const u32 literalId = *tbi.final_id_to_literal.at(final_id).begin();
    /* all literal ids associated with this final id should result in identical
     * literal entry */
    const auto &lit_infos = getLiteralInfoByFinalId(tbi, final_id);
    const rose_literal_info &arb_lit_info = **lit_infos.begin();
    const auto &vertices = arb_lit_info.vertices;

    literalTable.push_back(RoseLiteral());
    RoseLiteral &tl = literalTable.back();
    memset(&tl, 0, sizeof(tl));

    // These two are set by buildRootRoleTable.
    tl.rootRoleOffset = 0;
    tl.rootRoleCount = 0;

    tl.groups = 0;
    for (const auto &li : lit_infos) {
        tl.groups |= li->group_mask;
    }

    assert(tl.groups || tbi.literals.right.at(literalId).table == ROSE_ANCHORED
           || tbi.literals.right.at(literalId).table == ROSE_EVENT);

    // Minimum depth based on this literal's roles.
    tl.minDepth = calcMinDepth(bc.depths, vertices);

    DEBUG_PRINTF("lit %u: role minDepth=%u\n", final_id, tl.minDepth);

    // If this literal squashes its group behind it, store that data too
    tl.squashesGroup = arb_lit_info.squash_group;

    // Setup the delay stuff
    const auto &children = arb_lit_info.delayed_ids;
    if (children.empty()) {
        tl.delay_mask = 0;
        tl.delayIdsOffset = ROSE_OFFSET_INVALID;
    } else {
        map<u32, u32> local_delay_map; // delay -> relative child id
        for (const auto &int_id : children) {
            const rose_literal_id &child_literal = tbi.literals.right.at(int_id);
            u32 child_id = tbi.literal_info[int_id].final_id;
            u32 delay_index = child_id - tbi.delay_base_id;
            tl.delay_mask |= 1U << child_literal.delay;
            local_delay_map[child_literal.delay] = delay_index;
        }

        vector<u32> delayIds;
        for (const auto &did : local_delay_map | map_values) {
            delayIds.push_back(did);
        }

        tl.delayIdsOffset = add_to_engine_blob(bc, delayIds.begin(),
                                               delayIds.end());

    }

    assert(!tbi.literals.right.at(literalId).delay || !tl.delay_mask);

    tl.requires_side = needsSidecarCatchup(tbi, literalId);
}

// Construct the literal table.
static
void buildLiteralTable(const RoseBuildImpl &tbi, build_context &bc,
                       vector<RoseLiteral> &literalTable) {
    size_t numLiterals = tbi.final_id_to_literal.size();
    literalTable.reserve(numLiterals);

    for (size_t i = 0; i < numLiterals; ++i) {
        createLiteralEntry(tbi, bc, literalTable);
    }
}

static
void createRoleEntry(RoseBuildImpl &tbi, build_context &bc,
                     RoseVertex v, vector<RoseRole> &roleTable,
            ue2::unordered_map<vector<LookEntry>, size_t> &lookaround_cache,
            u32 *nextStateIndex) {
    RoseGraph &g = tbi.g;

    // Vertices have been normalised by now to have <= 1 reports.
    assert(g[v].reports.size() <= 1);

    // set role ID in the graph where we can find it later
    u32 roleId = (u32)roleTable.size();
    g[v].role = roleId;
    // track id if it's a nonroot role for use in buildSparseIter
    if (!tbi.isRootSuccessor(v)) {
        for (const auto &lit_id : g[v].literals) {
            u32 final_id = tbi.literal_info.at(lit_id).final_id;
            bc.litNonRootRoles[final_id].insert(roleId);
        }
    }

    roleTable.push_back(RoseRole());
    RoseRole &tr = roleTable.back();
    memset(&tr, 0, sizeof(tr));

    DEBUG_PRINTF("creating role %u for i%zu, eod %u, s (%p,%p)\n", roleId,
                 g[v].idx, (u32)g[v].eod_accept, g[v].suffix.graph.get(),
                 g[v].suffix.haig.get());

    // accept roles get their report ID.
    if (!g[v].reports.empty()) {
        DEBUG_PRINTF("%zu reports\n", g[v].reports.size());
        assert(g[v].reports.size() == 1);
        tr.reportId = *g[v].reports.begin();
        assert(tr.reportId < tbi.rm.numReports());
        const Report &ir = tbi.rm.getReport(tr.reportId);
        if (isInternalSomReport(ir)) {
            tr.flags |= ROSE_ROLE_FLAG_SOM_REPORT;
        }
        if (ir.type == INTERNAL_ROSE_CHAIN) {
            tr.flags |= ROSE_ROLE_FLAG_CHAIN_REPORT;
        }
    } else {
        tr.reportId = MO_INVALID_IDX;
    }

    tr.leftfixReport = g[v].left.leftfix_report;
    assert(!tbi.cc.streaming || g[v].left.lag <= MAX_STORED_LEFTFIX_LAG);
    tr.leftfixLag = g[v].left.lag;
    tr.depth = (u8)min(254U, bc.depths.at(v));
    tr.groups = g[v].groups;
    tr.flags |= ROSE_ROLE_PRED_NONE;

    if (contains(bc.leftfix_info, v)) {
        const left_build_info &lni = bc.leftfix_info.at(v);
        if (!lni.has_lookaround) {
            tr.flags |= ROSE_ROLE_FLAG_ROSE;
            tr.leftfixQueue = lni.nfa->queueIndex;
        }
    }

    if (!g[v].literals.empty()) {
        /* all literals for a role come from the same table -> inspect any */
        switch (tbi.literals.right.at(*g[v].literals.begin()).table) {
        case ROSE_ANCHORED:
            tr.flags |= ROSE_ROLE_FLAG_ANCHOR_TABLE;
            break;
        case ROSE_EOD_ANCHORED:
            tr.flags |= ROSE_ROLE_FLAG_EOD_TABLE;
            break;
        default:
            ;
        }
    }

    // Leaf nodes don't need state indices, as they don't have successors.
    /* TODO: also don't need a state index if all edges are nfa based */
    if (isLeafNode(v, g)) {
        tr.stateIndex = MMB_INVALID;
    } else {
        tr.stateIndex = (*nextStateIndex)++;
    }

    /* we are a suffaig - need to update role to provide som to the
     * suffix. */
    bool has_som = false;
    if (g[v].left.tracksSom()) {
        tr.flags |= ROSE_ROLE_FLAG_SOM_ROSEFIX;
        has_som = true;
    } else if (g[v].som_adjust) {
        tr.somAdjust = g[v].som_adjust;
        tr.flags |= ROSE_ROLE_FLAG_SOM_ADJUST;
        has_som = true;
    }

    if (has_som && !g[v].reports.empty()) {
        tr.flags |= ROSE_ROLE_FLAG_REPORT_START;
    }

    vector<LookEntry> look;
    if (tbi.cc.grey.roseLookaroundMasks) {
        // Lookaround from leftfix (mandatory).
        if (contains(bc.leftfix_info, v) &&
            bc.leftfix_info.at(v).has_lookaround) {
            DEBUG_PRINTF("using leftfix lookaround\n");
            look = bc.leftfix_info.at(v).lookaround;
        }
        // We may be able to find more lookaround info (advisory) and merge it
        // in.
        vector<LookEntry> look_more;
        findLookaroundMasks(tbi, v, look_more);
        mergeLookaround(look, look_more);
    }
    if (look.empty()) {
        DEBUG_PRINTF("no lookaround\n");
        tr.lookaroundIndex = MO_INVALID_IDX;
        tr.lookaroundCount = 0;
    } else {
        auto it = lookaround_cache.find(look);
        if (it != lookaround_cache.end()) {
            DEBUG_PRINTF("reusing look at idx %zu\n", it->second);
            tr.lookaroundIndex = verify_u32(it->second);
        } else {
            size_t idx = bc.lookaround.size();
            lookaround_cache.insert(make_pair(look, idx));
            insert(&bc.lookaround, bc.lookaround.end(), look);
            DEBUG_PRINTF("adding look at idx %zu\n", idx);
            tr.lookaroundIndex = verify_u32(idx);
        }
        tr.lookaroundCount = verify_u32(look.size());
    }

    DEBUG_PRINTF("role id=%u, stateidx=%u, reportId=%u, "
                 "depth=%u, groups=0x%016llx\n", roleId, tr.stateIndex,
                 tr.reportId, tr.depth, tr.groups);
}

// Construct an initial role table containing the basic role information.
static
void buildInitialRoleTable(RoseBuildImpl &tbi, build_context &bc) {
    DEBUG_PRINTF("building role table\n");

    const RoseGraph &g = tbi.g;
    vector<RoseRole> &roleTable = bc.roleTable;

    // Create a list of vertices, ordered by depth.
    vector<RoseVertex> verts;
    insert(&verts, verts.end(), vertices(g));
    sort(begin(verts), end(verts), [&bc, &g](const RoseVertex &a,
                                             const RoseVertex &b) {
        return tie(bc.depths.at(a), g[a].idx) < tie(bc.depths.at(b), g[b].idx);
    });

    // LookEntry list cache, so that we don't have to go scanning through the
    // full list to find cases we've used already.
    ue2::unordered_map<vector<LookEntry>, size_t> lookaround_cache;

    // Write a role entry for every vertex that represents a real literal.
    // Direct reports are skipped.
    // We start the state indices from one after the last one used (on the
    // anchored root, if it exists).
    u32 stateIndex = verify_u32(roleTable.size());

    for (RoseVertex v : verts) {
        if (tbi.isVirtualVertex(v)) {
            DEBUG_PRINTF("vertex idx=%zu is virtual\n", g[v].idx);
            continue;
        }
        if (tbi.hasDirectFinalId(v)) {
            DEBUG_PRINTF("vertex idx=%zu is direct report\n", g[v].idx);
            continue;
        }

        assert(!g[v].literals.empty());
        createRoleEntry(tbi, bc, v, roleTable, lookaround_cache, &stateIndex);
    }

    bc.numStates = stateIndex;
    DEBUG_PRINTF("wrote %zu roles with %u states\n", roleTable.size(),
                 stateIndex);
}

// Construct pred table and sparse iterators over preds.
static
void buildPredTable(const RoseBuildImpl &tbi, build_context &bc,
                    vector<RosePred> &predTable) {
    const RoseGraph &g = tbi.g;

    // We write our preds out in role index order just to give things some
    // repeatability.
    vector<RoseVertex> verts = get_ordered_verts(g);

    for (RoseVertex v : verts) {
        if (tbi.isAnyStart(v) || g[v].role == MO_INVALID_IDX) {
            continue;
        }

        assert(g[v].role < bc.roleTable.size());
        RoseRole &tr = bc.roleTable.at(g[v].role);

        // Assumption: if a vertex is a root role, it must have only one
        // predecessor.
        assert(!tbi.isRootSuccessor(v) || in_degree(v, g) == 1);

        // Check if we can use a "simple" check, i.e. one pred, bounds [0,
        // inf], no overlap and not anchor->float transition.
        if (in_degree(v, g) == 1) {
            const RoseEdge &e = *in_edges(v, g).first;
            RoseVertex u = source(e, g);
            DEBUG_PRINTF("single edge: (role=%u)->(role=%u) with bounds "
                         "[%u, %u]\n", g[u].role, g[v].role, g[e].minBound,
                         g[e].maxBound);
            if (tbi.isAnyStart(u)) {
                /* we have ourselves a root role */
                assert(u != tbi.root || g[e].maxBound == ROSE_BOUND_INF);
                if (u == tbi.root && g[e].minBound == 0) {
                    DEBUG_PRINTF("root role with .* edge, no pred needed\n");
                    continue; /* no pred required */
                }
                tr.predOffset = verify_u32(predTable.size());
                tr.flags &= ROSE_ROLE_PRED_CLEAR_MASK;
                tr.flags |= ROSE_ROLE_PRED_ROOT;
                createPred(tbi, bc, e, predTable);
                continue;
            }

            assert(!g[u].literals.empty() && !g[v].literals.empty());
            bool pseudo_delay_history = true;
            for (u32 ul : g[u].literals) {
                pseudo_delay_history = !!tbi.literals.right.at(ul).delay;
            }
            if (!pseudo_delay_history) {
                DEBUG_PRINTF("max_overlap = %zu\n",
                             tbi.maxLiteralOverlap(u, v));
            }
            if (g[e].minBound == 0 && g[e].maxBound == ROSE_BOUND_INF
                && (pseudo_delay_history || !tbi.maxLiteralOverlap(u, v))) {
                tr.flags &= ROSE_ROLE_PRED_CLEAR_MASK;
                tr.flags |= ROSE_ROLE_PRED_SIMPLE;
                bc.rolePredecessors[g[v].role].push_back(g[u].role);
                continue;
            }
        }

        assert(in_degree(v, g) >= 1);
        tr.flags &= ROSE_ROLE_PRED_CLEAR_MASK;
        tr.flags |= ROSE_ROLE_PRED_ANY;

        // Collect in-edges, ordered by the state index of the predecessor.
        vector<RoseEdge> edges = make_vector_from(in_edges(v, g));
        sort(edges.begin(), edges.end(),
             EdgeSourceStateCompare(g, bc.roleTable));

        vector<u32> keys;

        // Create preds and collect state indices for our sparse iterator.
        for (const auto &e : edges) {
            createPred(tbi, bc, e, predTable);
            RoseVertex u = source(e, g);
            assert(g[u].role < bc.roleTable.size());
            u32 stateIdx = bc.roleTable.at(g[u].role).stateIndex;
            if (stateIdx != MMB_INVALID) {
                keys.push_back(stateIdx);
            }
        }

        vector<mmbit_sparse_iter> iter;
        mmbBuildSparseIterator(iter, keys, bc.numStates);
        assert(!iter.empty());

        tr.predOffset = addIteratorToTable(bc, iter);
    }
}

static
bool hasUsefulStops(const left_build_info &rbi) {
    for (u32 i = 0; i < N_CHARS; i++) {
        if (rbi.stopAlphabet[i]) {
            return true;
        }
    }
    return false;
}

static
void buildLeftInfoTable(const RoseBuildImpl &tbi, build_context &bc,
                        u32 leftfixBeginQueue, u32 leftfixCount,
                        vector<LeftNfaInfo> &leftTable, u32 *laggedRoseCount,
                        size_t *history) {
    const RoseGraph &g = tbi.g;
    const CompileContext &cc = tbi.cc;

    ue2::unordered_set<u32> done_core;

    leftTable.resize(leftfixCount);

    u32 lagIndex = 0;

    vector<RoseVertex> verts = get_ordered_verts(g);
    for (RoseVertex v : verts) {
        if (!g[v].left) {
            continue;
        }
        assert(contains(bc.leftfix_info, v));
        const left_build_info &lbi = bc.leftfix_info.at(v);
        if (lbi.has_lookaround) {
            continue;
        }

        assert(lbi.nfa);
        assert(lbi.nfa->queueIndex >= leftfixBeginQueue);
        u32 left_index = lbi.nfa->queueIndex - leftfixBeginQueue;
        assert(left_index < leftfixCount);

        /* seedy hack to make miracles more effective.
         *
         * TODO: make miracle seeking not depend on history length and have
         * runt scans */
        if (hasUsefulStops(lbi)) {
            ENSURE_AT_LEAST(history,
                           (size_t)MIN(cc.grey.maxHistoryAvailable,
                                       g[v].left.lag + 1
                                           + cc.grey.miracleHistoryBonus));
        }

        LeftNfaInfo &left = leftTable[left_index];
        if (!contains(done_core, left_index)) {
            done_core.insert(left_index);
            memset(&left, 0, sizeof(left));
            left.squash_mask = ~0ULL;

            DEBUG_PRINTF("populating info for %u\n", left_index);

            left.maxQueueLen = lbi.max_queuelen;

            if (hasUsefulStops(lbi)) {
                assert(lbi.stopAlphabet.size() == N_CHARS);
                left.stopTable = add_to_engine_blob(bc, lbi.stopAlphabet.begin(),
                                                    lbi.stopAlphabet.end());
            }

            assert(lbi.countingMiracleOffset || !lbi.countingMiracleCount);
            left.countingMiracleOffset = lbi.countingMiracleOffset;

            DEBUG_PRINTF("mw = %u\n", lbi.transient);
            left.transient = verify_u8(lbi.transient);
            left.infix = tbi.isNonRootSuccessor(v);

            // A rose has a lagIndex if it's non-transient and we are
            // streaming.
            if (!lbi.transient && cc.streaming) {
                assert(lagIndex < ROSE_OFFSET_INVALID);
                left.lagIndex = lagIndex++;
            } else {
                left.lagIndex = ROSE_OFFSET_INVALID;
            }

            DEBUG_PRINTF("rose %u is %s\n", left_index,
                         left.infix ? "infix" : "prefix");
        }

        // Update squash mask.
        left.squash_mask &= lbi.squash_mask;

        // Update the max delay.
        ENSURE_AT_LEAST(&left.maxLag, lbi.lag);

        if (contains(g[v].literals, tbi.eod_event_literal_id)) {
            left.eod_check = 1;
        }
    }

    DEBUG_PRINTF("built %u roses with lag indices\n", lagIndex);
    *laggedRoseCount = lagIndex;
}

// Build sparse iterators for literals.
static
void buildSparseIter(build_context &bc, vector<RoseLiteral> &literalTable,
                     const vector<RosePred> &predTable) {
    for (u32 finalId = 0; finalId != literalTable.size(); ++finalId) {
        RoseLiteral &tl = literalTable[finalId];

        if (!contains(bc.litNonRootRoles, finalId)) {
            // This literal has no nonroot roles => no sparse iter
            tl.iterOffset = ROSE_OFFSET_INVALID;
            tl.iterMapOffset = ROSE_OFFSET_INVALID;
            continue;
        }

        const auto &roles = bc.litNonRootRoles.at(finalId);
        assert(!roles.empty());

        // Collect the state IDs of the predecessors of the roles of this
        // literal.

        // pred state id -> role/pred entries
        map<u32, vector<RoseIterRole> > predStates;

        for (u32 r : roles) {
            const RoseRole &tr = bc.roleTable.at(r);
            if (tr.flags & ROSE_ROLE_PRED_SIMPLE) {
                u32 p = bc.rolePredecessors.at(r)[0];
                assert(p != ROSE_OFFSET_INVALID);
                RoseIterRole ir = { r, ROSE_OFFSET_INVALID };
                predStates[bc.roleTable[p].stateIndex].push_back(ir);
            } else {
                const vector<u32> &myPreds = bc.rolePredecessors.at(r);
                for (u32 pred_entry : myPreds) {
                    u32 p = predTable.at(pred_entry).role;
                    RoseIterRole ir = { r, pred_entry };
                    assert(p < bc.roleTable.size());
                    predStates[bc.roleTable[p].stateIndex].push_back(ir);
                }
            }
        }

        tie(tl.iterMapOffset, tl.iterOffset) = addPredSparseIter(bc, predStates);
    }
}

static
void calcAnchoredMatches(const RoseBuildImpl &build, vector<ReportID> &art,
                         vector<u32> &arit) {
    const RoseGraph &g = build.g;

    u32 max_report = 0;

    for (RoseVertex v : vertices_range(g)) {
        if (!build.isAnchored(v)) {
            continue;
        }

        for (ReportID r : g[v].reports) {
            art.push_back(r);
            max_report = max(max_report, r);
        }
    }

    assert(max_report < MO_INVALID_IDX);

    arit.resize(max_report + 1, MO_INVALID_IDX);
    for (u32 i = 0; i < art.size(); i++) {
        DEBUG_PRINTF("art[%u] = %u\n", i, art[i]);
        arit[art[i]] = i;
        DEBUG_PRINTF("arit[%u] = %u\n", art[i], arit[art[i]]);
    }
}

static
u32 history_required(const rose_literal_id &key) {
    if (key.msk.size() < key.s.length()) {
        return key.elength() - 1;
    } else {
        return key.msk.size() + key.delay - 1;
    }
}

static
void fillMatcherDistances(const RoseBuildImpl &build, RoseEngine *engine) {
    const RoseGraph &g = build.g;

    engine->floatingDistance = 0;
    engine->floatingMinDistance = ROSE_BOUND_INF;
    engine->anchoredDistance = 0;
    engine->maxFloatingDelayedMatch = 0;
    u32 delayRebuildLength = 0;
    engine->smallBlockDistance = 0;

    for (auto v : vertices_range(g)) {
        if (g[v].literals.empty()) {
            continue;
        }

        assert(g[v].min_offset < ROSE_BOUND_INF); // cannot == ROSE_BOUND_INF
        assert(g[v].min_offset <= g[v].max_offset);

        for (u32 lit_id : g[v].literals) {
            const rose_literal_id &key = build.literals.right.at(lit_id);
            u32 max_d = g[v].max_offset;
            u32 min_d = g[v].min_offset;

            if (build.literal_info[lit_id].undelayed_id != lit_id) {
                /* this is a delayed match; need to update delay properties */
                /* TODO: can delayed literals ever be in another table ? */
                if (key.table == ROSE_FLOATING) {
                    ENSURE_AT_LEAST(&engine->maxFloatingDelayedMatch, max_d);
                    ENSURE_AT_LEAST(&delayRebuildLength, history_required(key));
                }
            }

            /* for the FloatingDistances we need the true max depth of the
               string */
            if (max_d != ROSE_BOUND_INF && key.table != ROSE_ANCHORED) {
                assert(max_d >= key.delay);
                max_d -= key.delay;
            }

            switch (key.table) {
            case ROSE_FLOATING:
                ENSURE_AT_LEAST(&engine->floatingDistance, max_d);
                if (min_d >= key.elength()) {
                    LIMIT_TO_AT_MOST(&engine->floatingMinDistance,
                                     min_d - (u32)key.elength());
                } else {
                    /* overlapped literals from rose + anchored table can
                     * cause us to underflow due to sloppiness in
                     * estimates */
                    engine->floatingMinDistance = 0;
                }
                break;
            case ROSE_ANCHORED_SMALL_BLOCK:
                ENSURE_AT_LEAST(&engine->smallBlockDistance, max_d);
                break;
            case ROSE_ANCHORED:
                ENSURE_AT_LEAST(&engine->anchoredDistance, max_d);
                break;
            case ROSE_EOD_ANCHORED:
                // EOD anchored literals are in another table, so they
                // don't contribute to these calculations.
                break;
            case ROSE_EVENT:
                break; // Not a real literal.
            }
        }
    }

    // Floating literals go in the small block table too.
    ENSURE_AT_LEAST(&engine->smallBlockDistance, engine->floatingDistance);

    // Clipped by its very nature.
    LIMIT_TO_AT_MOST(&engine->smallBlockDistance, 32U);

    engine->delayRebuildLength = delayRebuildLength;

    DEBUG_PRINTF("anchoredDistance = %u\n", engine->anchoredDistance);
    DEBUG_PRINTF("floatingDistance = %u\n", engine->floatingDistance);
    DEBUG_PRINTF("smallBlockDistance = %u\n", engine->smallBlockDistance);
    assert(engine->anchoredDistance <= build.cc.grey.maxAnchoredRegion);

    /* anchored->floating squash literals may lower floating min distance */
    /* TODO: find actual value */
    if (!engine->anchoredDistance) {
        return;
    }

    /* could be improved, if we have any side squash stuff and an anchored table
     * set the min float distance to 0 */
    if (!build.side_squash_roles.empty()) {
        engine->floatingMinDistance = 0;
    }
}

aligned_unique_ptr<RoseEngine> RoseBuildImpl::buildFinalEngine(u32 minWidth) {
    DerivedBoundaryReports dboundary(boundary);

    // Build literal matchers
    size_t asize = 0, fsize = 0, ssize = 0, esize = 0, sbsize = 0;

    size_t floatingStreamStateRequired = 0;
    size_t historyRequired = calcHistoryRequired(); // Updated by HWLM.

    aligned_unique_ptr<void> atable =
        buildAnchoredAutomataMatcher(*this, &asize);
    aligned_unique_ptr<HWLM> ftable = buildFloatingMatcher(
        *this, &fsize, &historyRequired, &floatingStreamStateRequired);
    aligned_unique_ptr<sidecar> stable = buildSideMatcher(*this, &ssize);
    aligned_unique_ptr<HWLM> etable = buildEodAnchoredMatcher(*this, &esize);
    aligned_unique_ptr<HWLM> sbtable = buildSmallBlockMatcher(*this, &sbsize);

    build_context bc;
    bc.depths = findDepths(*this);

    // Build NFAs
    vector<aligned_unique_ptr<NFA>> built_nfas;
    map<suffix_id, u32> suffixes;
    set<u32> no_retrigger_queues;
    bool mpv_as_outfix;
    prepMpv(*this, &built_nfas, &historyRequired, &mpv_as_outfix);
    u32 outfixBeginQueue = qif.allocated_count();
    if (!prepOutfixes(*this, &built_nfas, &historyRequired)) {
        return nullptr;
    }
    u32 outfixEndQueue = qif.allocated_count();
    u32 leftfixBeginQueue = outfixEndQueue;

    if (!buildNfas(*this, qif, &built_nfas, &suffixes, &bc.leftfix_info,
                   &no_retrigger_queues, &leftfixBeginQueue)) {
        return nullptr;
    }
    buildCountingMiracles(*this, bc);

    u32 queue_count = qif.allocated_count(); /* excludes anchored matcher q;
                                              * som rev nfas */
    if (queue_count > cc.grey.limitRoseEngineCount) {
        throw ResourceLimitError();
    }

    u32 lit_benefits_size =
        verify_u32(sizeof(lit_benefits) * nonbenefits_base_id);
    assert(ISALIGNED_16(lit_benefits_size));
    u32 nfas_size = calcNfaSize(built_nfas);

    // Build our other tables
    DEBUG_PRINTF("nfas_size %u\n", nfas_size);

    vector<u32> suffixEkeyLists;
    buildSuffixEkeyLists(*this, bc, qif, suffixes, &suffixEkeyLists);

    buildInitialRoleTable(*this, bc);

    DEBUG_PRINTF("roletable %zu\n", bc.roleTable.size());

    vector<RosePred> predTable;
    buildPredTable(*this, bc, predTable);

    u32 laggedRoseCount = 0;
    vector<LeftNfaInfo> leftInfoTable;
    buildLeftInfoTable(*this, bc, leftfixBeginQueue,
                       queue_count - leftfixBeginQueue, leftInfoTable,
                       &laggedRoseCount, &historyRequired);

    buildRoseTriggerLists(*this, bc);

    vector<RoseLiteral> literalTable;
    buildLiteralTable(*this, bc, literalTable);
    buildSparseIter(bc, literalTable, predTable);

    u32 eodIterOffset;
    u32 eodIterMapOffset;

    tie(eodIterMapOffset, eodIterOffset) = buildEodAnchorRoles(*this, bc,
                                                               predTable);

    vector<RoseSide> sideTable;
    buildSideTable(*this, bc, sideTable);

    vector<mmbit_sparse_iter> activeLeftIter;
    buildActiveLeftIter(leftInfoTable, activeLeftIter);

    u32 lastByteOffset = buildLastByteIter(g, bc);

    // Enforce role table resource limit.
    if (bc.roleTable.size() > cc.grey.limitRoseRoleCount) {
        throw ResourceLimitError();
    }

    u32 amatcherOffset = 0;
    u32 fmatcherOffset = 0;
    u32 smatcherOffset = 0;
    u32 ematcherOffset = 0;
    u32 sbmatcherOffset = 0;

    u32 currOffset;  /* relative to base of RoseEngine */
    if (!bc.engine_blob.empty()) {
        currOffset = bc.engine_blob_base + byte_length(bc.engine_blob);
    } else {
        currOffset = sizeof(RoseEngine);
    }

    currOffset = ROUNDUP_CL(currOffset);
    DEBUG_PRINTF("currOffset %u\n", currOffset);

    /* leave space for the nfas */
    u32 base_nfa_offset = currOffset;
    currOffset += nfas_size;

    /* leave space for the benefits listing */
    u32 base_lits_benefits_offset = currOffset;
    currOffset += lit_benefits_size;

    if (atable) {
        currOffset = ROUNDUP_CL(currOffset);
        amatcherOffset = currOffset;
        currOffset += (u32)asize;
    }

    if (ftable) {
        currOffset = ROUNDUP_CL(currOffset);
        fmatcherOffset = currOffset;
        currOffset += (u32)fsize;
    }

    if (stable) {
        currOffset = ROUNDUP_CL(currOffset);
        smatcherOffset = currOffset;
        currOffset += (u32)ssize;
    }

    if (etable) {
        currOffset = ROUNDUP_CL(currOffset);
        ematcherOffset = currOffset;
        currOffset += (u32)esize;
    }

    if (sbtable) {
        currOffset = ROUNDUP_CL(currOffset);
        sbmatcherOffset = currOffset;
        currOffset += (u32)sbsize;
    }

    const vector<Report> &int_reports = rm.reports();

    currOffset = ROUNDUP_CL(currOffset);
    u32 intReportOffset = currOffset;
    currOffset += sizeof(internal_report) * int_reports.size();

    u32 literalOffset = ROUNDUP_N(currOffset, alignof(RoseLiteral));
    u32 literalLen = sizeof(RoseLiteral) * literalTable.size();
    currOffset = literalOffset + literalLen;

    u32 sideOffset = ROUNDUP_N(currOffset, alignof(RoseSide));
    currOffset = sideOffset + byte_length(sideTable);

    u32 roleOffset = ROUNDUP_N(currOffset, alignof(RoseRole));
    u32 roleLen = sizeof(RoseRole) * bc.roleTable.size();
    currOffset = roleOffset + roleLen;

    u32 leftOffset = ROUNDUP_N(currOffset, alignof(LeftNfaInfo));
    u32 roseLen = sizeof(LeftNfaInfo) * leftInfoTable.size();
    currOffset = leftOffset + roseLen;

    u32 lookaroundReachOffset = currOffset;
    u32 lookaroundReachLen = REACH_BITVECTOR_LEN * bc.lookaround.size();
    currOffset = lookaroundReachOffset + lookaroundReachLen;

    u32 lookaroundTableOffset = currOffset;
    u32 lookaroundTableLen = sizeof(s8) * bc.lookaround.size();
    currOffset = lookaroundTableOffset + lookaroundTableLen;

    u32 predOffset = ROUNDUP_N(currOffset, alignof(RosePred));
    u32 predLen = sizeof(RosePred) * predTable.size();
    currOffset = predOffset + predLen;

    u32 nfaInfoOffset = ROUNDUP_N(currOffset, sizeof(u32));
    u32 nfaInfoLen = sizeof(NfaInfo) * queue_count;
    currOffset = nfaInfoOffset + nfaInfoLen;

    vector<u32> rootRoleTable;
    buildRootRoleTable(*this, roleOffset, literalTable, &rootRoleTable);

    u32 rootRoleOffset = ROUNDUP_N(currOffset, sizeof(u32));
    u32 rootRoleLen = sizeof(u32) * rootRoleTable.size();
    currOffset = rootRoleOffset + rootRoleLen;

    vector<ReportID> art; // Reports raised by anchored roles
    vector<u32> arit; // inverse reportID -> position in art
    calcAnchoredMatches(*this, art, arit);

    currOffset = ROUNDUP_N(currOffset, sizeof(ReportID));
    u32 anchoredReportMapOffset = currOffset;
    currOffset += art.size() * sizeof(ReportID);

    currOffset = ROUNDUP_N(currOffset, sizeof(u32));
    u32 anchoredReportInverseMapOffset = currOffset;
    currOffset += arit.size() * sizeof(u32);

    /* sidecar may contain sse in silly cases */
    currOffset = ROUNDUP_N(currOffset, 16);
    u32 sideSuccListOffset = currOffset;
    map<set<u32>, set<RoseVertex> > sidecar_succ_map;
    markSideEnablers(*this, &sidecar_succ_map);
    currOffset += sizeSideSuccMasks(stable.get(), sidecar_succ_map);

    currOffset = ROUNDUP_N(currOffset, alignof(ReportID));
    u32 multidirectOffset = currOffset;
    currOffset += mdr_reports.size() * sizeof(ReportID);

    currOffset = ROUNDUP_N(currOffset, alignof(mmbit_sparse_iter));
    u32 activeLeftIterOffset = currOffset;
    currOffset += activeLeftIter.size() * sizeof(mmbit_sparse_iter);

    u32 activeArrayCount = leftfixBeginQueue;
    u32 activeLeftCount = leftInfoTable.size();
    u32 rosePrefixCount = countRosePrefixes(leftInfoTable);

    RoseBoundaryReports boundary_out;
    reserveBoundaryReports(boundary, dboundary, &boundary_out, &currOffset);

    u32 rev_nfa_table_offset;
    vector<u32> rev_nfa_offsets;
    prepSomRevNfas(ssm, &rev_nfa_table_offset, &rev_nfa_offsets, &currOffset);

    // Build engine header and copy tables into place.

    u32 anchorStateSize = anchoredStateSize(atable.get());

    DEBUG_PRINTF("rose history required %zu\n", historyRequired);
    assert(!cc.streaming || historyRequired <= cc.grey.maxHistoryAvailable);

    // Some SOM schemes (reverse NFAs, for example) may require more history.
    historyRequired = max(historyRequired, (size_t)ssm.somHistoryRequired());

    assert(!cc.streaming || historyRequired <=
           max(cc.grey.maxHistoryAvailable, cc.grey.somMaxRevNfaLength));

    RoseStateOffsets stateOffsets;
    memset(&stateOffsets, 0, sizeof(stateOffsets));
    fillStateOffsets(*this, stable.get(), bc.numStates, anchorStateSize,
                     activeArrayCount, activeLeftCount, laggedRoseCount,
                     floatingStreamStateRequired, historyRequired,
                     &stateOffsets);

    scatter_plan_raw state_scatter;
    buildStateScatterPlan(sizeof(RoseRuntimeState), bc.numStates,
                          activeLeftCount, rosePrefixCount, stateOffsets,
                          cc.streaming, activeArrayCount, outfixBeginQueue,
                          outfixEndQueue, &state_scatter);

    currOffset = ROUNDUP_N(currOffset, alignof(scatter_unit_u64a));

    u32 state_scatter_aux_offset = currOffset;
    currOffset += aux_size(state_scatter);

    currOffset = ROUNDUP_N(currOffset, alignof(ReportID));
    u32 dkeyOffset = currOffset;
    currOffset += rm.numDkeys() * sizeof(ReportID);

    aligned_unique_ptr<RoseEngine> engine
        = aligned_zmalloc_unique<RoseEngine>(currOffset);
    assert(engine); // will have thrown bad_alloc otherwise.
    char *ptr = (char *)engine.get();
    assert(ISALIGNED_CL(ptr));

    if (atable) {
        assert(amatcherOffset >= base_nfa_offset);
        assert(amatcherOffset);
        memcpy(ptr + amatcherOffset, atable.get(), asize);
    }
    if (ftable) {
        assert(fmatcherOffset);
        assert(fmatcherOffset >= base_nfa_offset);
        memcpy(ptr + fmatcherOffset, ftable.get(), fsize);
    }
    if (stable) {
        assert(smatcherOffset);
        assert(smatcherOffset >= base_nfa_offset);
        memcpy(ptr + smatcherOffset, stable.get(), ssize);
    }
    if (etable) {
        assert(ematcherOffset);
        assert(ematcherOffset >= base_nfa_offset);
        memcpy(ptr + ematcherOffset, etable.get(), esize);
    }
    if (sbtable) {
        assert(sbmatcherOffset);
        assert(sbmatcherOffset >= base_nfa_offset);
        memcpy(ptr + sbmatcherOffset, sbtable.get(), sbsize);
    }

    memcpy(&engine->stateOffsets, &stateOffsets, sizeof(stateOffsets));

    engine->historyRequired = verify_u32(historyRequired);

    engine->ekeyCount = rm.numEkeys();
    engine->dkeyCount = rm.numDkeys();
    engine->invDkeyOffset = dkeyOffset;
    populateInvDkeyTable(ptr + dkeyOffset, rm);

    engine->somHorizon = ssm.somPrecision();
    engine->somLocationCount = ssm.numSomSlots();

    engine->simpleCallback = !rm.numEkeys() && hasSimpleReports(rm.reports());

    fillInReportInfo(engine.get(), intReportOffset, rm, int_reports);

    engine->literalOffset = literalOffset;
    engine->literalCount = verify_u32(literalTable.size());
    engine->runtimeImpl = pickRuntimeImpl(*this, outfixEndQueue);
    engine->mpvTriggeredByLeaf = anyEndfixMpvTriggers(*this);

    engine->sideOffset = sideOffset;
    engine->sideCount = verify_u32(sideTable.size());

    engine->activeArrayCount = activeArrayCount;
    engine->activeLeftCount = activeLeftCount;
    engine->queueCount = queue_count;

    engine->group_weak_end = group_weak_end;

    engine->rolesWithStateCount = bc.numStates;

    engine->roleOffset = roleOffset;
    engine->roleCount = verify_u32(bc.roleTable.size());
    engine->leftOffset = leftOffset;
    engine->roseCount = verify_u32(leftInfoTable.size());
    engine->lookaroundTableOffset = lookaroundTableOffset;
    engine->lookaroundReachOffset = lookaroundReachOffset;
    engine->outfixBeginQueue = outfixBeginQueue;
    engine->outfixEndQueue = outfixEndQueue;
    engine->leftfixBeginQueue = leftfixBeginQueue;
    engine->initMpvNfa = mpv_as_outfix ? 0 : MO_INVALID_IDX;
    engine->predOffset = predOffset;
    engine->predCount = verify_u32(predTable.size());
    engine->stateSize = mmbit_size(bc.numStates);
    engine->anchorStateSize = anchorStateSize;
    engine->nfaInfoOffset = nfaInfoOffset;
    engine->anchoredReportMapOffset = anchoredReportMapOffset;
    engine->anchoredReportInverseMapOffset
        = anchoredReportInverseMapOffset;
    engine->multidirectOffset = multidirectOffset;
    engine->rootRoleCount = verify_u32(rootRoleTable.size());
    engine->rootRoleOffset = rootRoleOffset;

    engine->eodIterOffset = eodIterOffset;
    engine->eodIterMapOffset = eodIterMapOffset;

    engine->lastByteHistoryIterOffset = lastByteOffset;

    u32 delay_count = verify_u32(literalTable.size() - delay_base_id);
    engine->delay_count = delay_count;
    engine->delay_slot_size = mmbit_size(delay_count);
    engine->delay_base_id = delay_base_id;
    engine->anchored_base_id = anchored_base_id;
    engine->anchored_count = delay_base_id - anchored_base_id;
    engine->nonbenefits_base_id = nonbenefits_base_id;
    engine->literalBenefitsOffsets = base_lits_benefits_offset;

    populateSideSuccLists(*this, bc, stable.get(), engine.get(),
                          sideSuccListOffset, sidecar_succ_map);
    engine->rosePrefixCount = rosePrefixCount;

    engine->activeLeftIterOffset
        = activeLeftIter.empty() ? 0 : activeLeftIterOffset;

    // Set scanning mode.
    if (!cc.streaming) {
        engine->mode = HS_MODE_BLOCK;
    } else if (cc.vectored) {
        engine->mode = HS_MODE_VECTORED;
    } else {
        engine->mode = HS_MODE_STREAM;
    }

    // The Small Write matcher is (conditionally) added to the RoseEngine in
    // another pass by the caller. Set to zero (meaning no SMWR engine) for
    // now.
    engine->smallWriteOffset = 0;

    engine->amatcherOffset = amatcherOffset;
    engine->ematcherOffset = ematcherOffset;
    engine->sbmatcherOffset = sbmatcherOffset;
    engine->fmatcherOffset = fmatcherOffset;
    engine->smatcherOffset = smatcherOffset;
    engine->amatcherMinWidth = findMinWidth(*this, ROSE_ANCHORED);
    engine->fmatcherMinWidth = findMinWidth(*this, ROSE_FLOATING);
    engine->eodmatcherMinWidth = findMinWidth(*this, ROSE_EOD_ANCHORED);
    engine->amatcherMaxBiAnchoredWidth = findMaxBAWidth(*this, ROSE_ANCHORED);
    engine->fmatcherMaxBiAnchoredWidth = findMaxBAWidth(*this, ROSE_FLOATING);
    engine->size = currOffset;
    engine->minWidth = hasBoundaryReports(boundary) ? 0 : minWidth;
    engine->minWidthExcludingBoundaries = minWidth;
    engine->maxSafeAnchoredDROffset = findMinWidth(*this, ROSE_FLOATING);
    engine->floatingMinLiteralMatchOffset = findMinFloatingLiteralMatch(*this);

    engine->maxBiAnchoredWidth = findMaxBAWidth(*this);
    engine->noFloatingRoots = hasNoFloatingRoots();
    engine->hasFloatingDirectReports = floating_direct_report;
    engine->requiresEodCheck = hasEodAnchors(*this, built_nfas,
                                             outfixEndQueue);
    engine->requiresEodSideCatchup = hasEodSideLink();
    engine->hasOutfixesInSmallBlock = hasNonSmallBlockOutfix(outfixes);
    engine->canExhaust = rm.patternSetCanExhaust();
    engine->hasSom = hasSom;
    engine->anchoredMatches = verify_u32(art.size());

    /* populate anchoredDistance, floatingDistance, floatingMinDistance, etc */
    fillMatcherDistances(*this, engine.get());

    engine->initialGroups = getInitialGroups();
    engine->totalNumLiterals = verify_u32(literalTable.size());
    engine->asize = verify_u32(asize);
    engine->ematcherRegionSize = ematcher_region_size;
    engine->floatingStreamState = verify_u32(floatingStreamStateRequired);
    populateBoundaryReports(engine.get(), boundary, dboundary, boundary_out);

    write_out(&engine->state_init, (char *)engine.get(), state_scatter,
              state_scatter_aux_offset);

    if (eod_event_literal_id != MO_INVALID_IDX) {
        engine->hasEodEventLiteral = 1;
        DEBUG_PRINTF("eod literal id=%u, final_id=%u\n", eod_event_literal_id,
                     literal_info.at(eod_event_literal_id).final_id);
        engine->eodLiteralId = literal_info.at(eod_event_literal_id).final_id;
    }

    if (anchoredIsMulti(*engine)) {
        DEBUG_PRINTF("multiple anchored dfas\n");
        engine->maxSafeAnchoredDROffset = 1;
        engine->floatingMinLiteralMatchOffset = 1; /* regard matches from other
                                                      anchored tables as
                                                      floating as unordered. */
    } else {
        /* overly conservative, really need the min offset of non dr anchored
           matches */
        engine->maxSafeAnchoredDROffset = MIN(engine->maxSafeAnchoredDROffset,
                                        engine->floatingMinLiteralMatchOffset);
    }

    NfaInfo *nfa_infos = (NfaInfo *)(ptr + nfaInfoOffset);
    populateNfaInfoBasics(nfa_infos, outfixes, rm, suffixes, suffixEkeyLists);
    updateNfaState(built_nfas, bc.leftfix_info, &engine->stateOffsets, nfa_infos,
                   &engine->scratchStateSize, &engine->nfaStateSize,
                   &engine->tStateSize);

    // Copy in the NFAs and update roles
    engine->nfaRegionBegin = base_nfa_offset;
    engine->nfaRegionEnd = copyInNFAs(*this, &bc.roleTable, built_nfas,
                                      no_retrigger_queues, nfa_infos,
                                      base_nfa_offset, suffixes, ptr);
    // We're done with the NFAs.
    built_nfas.clear();

    /* do after update mask */
    buildLitBenefits(*this, engine.get(), base_lits_benefits_offset);

    // Copy in other tables
    memcpy(ptr + bc.engine_blob_base, bc.engine_blob.data(),
           byte_length(bc.engine_blob));

    memcpy(ptr + engine->literalOffset, literalTable.data(),
           byte_length(literalTable));
    memcpy(ptr + engine->roleOffset, bc.roleTable.data(),
           byte_length(bc.roleTable));
    copy(leftInfoTable.begin(), leftInfoTable.end(),
         (LeftNfaInfo *)(ptr + engine->leftOffset));

    fillLookaroundTables(ptr + lookaroundTableOffset,
                         ptr + lookaroundReachOffset, bc.lookaround);

    fillInSomRevNfas(engine.get(), ssm, rev_nfa_table_offset, rev_nfa_offsets);
    memcpy(ptr + engine->predOffset, predTable.data(), byte_length(predTable));
    memcpy(ptr + engine->rootRoleOffset, rootRoleTable.data(),
           byte_length(rootRoleTable));
    memcpy(ptr + engine->anchoredReportMapOffset, art.data(), byte_length(art));
    memcpy(ptr + engine->anchoredReportInverseMapOffset, arit.data(),
           byte_length(arit));
    memcpy(ptr + engine->multidirectOffset, mdr_reports.data(),
           byte_length(mdr_reports));

    copy(activeLeftIter.begin(), activeLeftIter.end(),
         (mmbit_sparse_iter *)(ptr + engine->activeLeftIterOffset));

    memcpy(ptr + engine->sideOffset, sideTable.data(), byte_length(sideTable));

    DEBUG_PRINTF("rose done %p\n", engine.get());
    return engine;
}

} // namespace ue2
