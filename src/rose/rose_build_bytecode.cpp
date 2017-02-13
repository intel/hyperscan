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

#include "rose_build_impl.h"

#include "ue2common.h"
#include "grey.h"
#include "hs_compile.h" // for HS_MODE_*
#include "rose_build_add_internal.h"
#include "rose_build_anchored.h"
#include "rose_build_engine_blob.h"
#include "rose_build_exclusive.h"
#include "rose_build_groups.h"
#include "rose_build_infix.h"
#include "rose_build_long_lit.h"
#include "rose_build_lookaround.h"
#include "rose_build_matchers.h"
#include "rose_build_program.h"
#include "rose_build_scatter.h"
#include "rose_build_util.h"
#include "rose_build_width.h"
#include "rose_internal.h"
#include "rose_program.h"
#include "hwlm/hwlm.h" /* engine types */
#include "hwlm/hwlm_literal.h"
#include "nfa/castlecompile.h"
#include "nfa/goughcompile.h"
#include "nfa/mcclellancompile.h"
#include "nfa/mcclellancompile_util.h"
#include "nfa/mcsheng_compile.h"
#include "nfa/nfa_api_queue.h"
#include "nfa/nfa_build_util.h"
#include "nfa/nfa_internal.h"
#include "nfa/shengcompile.h"
#include "nfa/shufticompile.h"
#include "nfa/tamaramacompile.h"
#include "nfa/tamarama_internal.h"
#include "nfagraph/ng_execute.h"
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
#include "smallwrite/smallwrite_build.h"
#include "som/slot_manager.h"
#include "util/alloc.h"
#include "util/bitutils.h"
#include "util/boundary_reports.h"
#include "util/charreach.h"
#include "util/charreach_util.h"
#include "util/compile_context.h"
#include "util/compile_error.h"
#include "util/container.h"
#include "util/fatbit_build.h"
#include "util/graph_range.h"
#include "util/make_unique.h"
#include "util/multibit_build.h"
#include "util/order_check.h"
#include "util/popcount.h"
#include "util/queue_index_factory.h"
#include "util/report_manager.h"
#include "util/ue2string.h"
#include "util/verify_types.h"

#include <algorithm>
#include <array>
#include <map>
#include <queue>
#include <set>
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

struct left_build_info {
    // Constructor for an engine implementation.
    left_build_info(u32 q, u32 l, u32 t, rose_group sm,
                    const std::vector<u8> &stops, u32 max_ql, u8 cm_count,
                    const CharReach &cm_cr)
        : queue(q), lag(l), transient(t), squash_mask(sm), stopAlphabet(stops),
          max_queuelen(max_ql), countingMiracleCount(cm_count),
          countingMiracleReach(cm_cr) {}

    // Constructor for a lookaround implementation.
    explicit left_build_info(const vector<LookEntry> &look)
        : has_lookaround(true), lookaround(look) {}

    u32 queue = 0; /* uniquely idents the left_build_info */
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

/**
 * \brief Structure tracking which resources are used by this Rose instance at
 * runtime.
 *
 * We use this to control how much initialisation we need to do at the
 * beginning of a stream/block at runtime.
 */
struct RoseResources {
    bool has_outfixes = false;
    bool has_suffixes = false;
    bool has_leftfixes = false;
    bool has_literals = false;
    bool has_states = false;
    bool checks_groups = false;
    bool has_lit_delay = false;
    bool has_lit_check = false; // long literal support
    bool has_anchored = false;
    bool has_floating = false;
    bool has_eod = false;
};

struct build_context : boost::noncopyable {
    /** \brief information about engines to the left of a vertex */
    map<RoseVertex, left_build_info> leftfix_info;

    /** \brief mapping from suffix to queue index. */
    map<suffix_id, u32> suffixes;

    /** \brief Mapping from vertex to key, for vertices with a
     * CHECK_NOT_HANDLED instruction. */
    ue2::unordered_map<RoseVertex, u32> handledKeys;

    /** \brief Number of roles with a state bit.
     *
     * This is set by assignStateIndices() and should be constant throughout
     * the rest of the compile.
     */
    size_t numStates = 0;

    /** \brief Simple cache of programs written to engine blob, used for
     * deduplication. */
    ue2::unordered_map<RoseProgram, u32, RoseProgramHash,
                       RoseProgramEquivalence> program_cache;

    /** \brief LookEntry list cache, so that we don't have to go scanning
     * through the full list to find cases we've used already. */
    ue2::unordered_map<vector<LookEntry>, size_t> lookaround_cache;

    /** \brief Lookaround table for Rose roles. */
    vector<LookEntry> lookaround;

    /** \brief State indices, for those roles that have them. */
    ue2::unordered_map<RoseVertex, u32> roleStateIndices;

    /** \brief Mapping from queue index to bytecode offset for built engines
     * that have already been pushed into the engine_blob. */
    ue2::unordered_map<u32, u32> engineOffsets;

    /** \brief List of long literals (ones with CHECK_LONG_LIT instructions)
     * that need hash table support. */
    vector<ue2_case_string> longLiterals;

    /** \brief Minimum offset of a match from the floating table. */
    u32 floatingMinLiteralMatchOffset = 0;

    /** \brief Long literal length threshold, used in streaming mode. */
    size_t longLitLengthThreshold = 0;

    /** \brief Contents of the Rose bytecode immediately following the
     * RoseEngine. */
    RoseEngineBlob engine_blob;

    /** \brief True if reports need CATCH_UP instructions, to catch up anchored
     * matches, suffixes, outfixes etc. */
    bool needs_catchup = false;

    /** \brief True if this Rose engine has an MPV engine. */
    bool needs_mpv_catchup = false;

    /** \brief Resources in use (tracked as programs are added). */
    RoseResources resources;

    /** \brief Mapping from every vertex to the groups that must be on for that
     * vertex to be reached. */
    ue2::unordered_map<RoseVertex, rose_group> vertex_group_map;

    /** \brief Global bitmap of groups that can be squashed. */
    rose_group squashable_groups = 0;

    /** \brief Mapping from final ID to the set of literals it is used for. */
    map<u32, flat_set<u32>> final_id_to_literal;
};

/** \brief subengine info including built engine and
* corresponding triggering rose vertices */
struct ExclusiveSubengine {
    aligned_unique_ptr<NFA> nfa;
    vector<RoseVertex> vertices;
};

/** \brief exclusive info to build tamarama */
struct ExclusiveInfo {
    // subengine info
    vector<ExclusiveSubengine> subengines;
    // all the report in tamarama
    set<ReportID> reports;
    // assigned queue id
    u32 queue;

    // workaround a deficiency in the standard (as explained by STL @ MS) we
    // need to tell the compiler that ExclusiveInfo is moveable-only by
    // deleting the copy cons so that vector doesn't get confused
    ExclusiveInfo() = default;
    ExclusiveInfo(const ExclusiveInfo &) = delete;
    ExclusiveInfo(ExclusiveInfo &&) = default;
};

}

static
const NFA *get_nfa_from_blob(const build_context &bc, u32 qi) {
    assert(contains(bc.engineOffsets, qi));
    u32 nfa_offset = bc.engineOffsets.at(qi);
    assert(nfa_offset >= bc.engine_blob.base_offset);
    const NFA *n = (const NFA *)(bc.engine_blob.data() + nfa_offset -
                                 bc.engine_blob.base_offset);
    assert(n->queueIndex == qi);
    return n;
}

static
const NFA *add_nfa_to_blob(build_context &bc, NFA &nfa) {
    u32 qi = nfa.queueIndex;
    u32 nfa_offset = bc.engine_blob.add(nfa, nfa.length);
    DEBUG_PRINTF("added nfa qi=%u, type=%u, length=%u at offset=%u\n", qi,
                  nfa.type, nfa.length, nfa_offset);

    assert(!contains(bc.engineOffsets, qi));
    bc.engineOffsets.emplace(qi, nfa_offset);

    const NFA *n = get_nfa_from_blob(bc, qi);
    assert(memcmp(&nfa, n, nfa.length) == 0);
    return n;
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

/**
 * \brief True if this Rose engine needs to run a catch up whenever a report is
 * generated.
 *
 * Catch up is necessary if there are output-exposed engines (suffixes,
 * outfixes) or an anchored table (anchored literals, acyclic DFAs).
 */
static
bool needsCatchup(const RoseBuildImpl &build,
                  const vector<raw_dfa> &anchored_dfas) {
    if (!build.outfixes.empty()) {
        DEBUG_PRINTF("has outfixes\n");
        return true;
    }
    if (!anchored_dfas.empty()) {
        DEBUG_PRINTF("has anchored dfas\n");
        return true;
    }

    const RoseGraph &g = build.g;

    for (auto v : vertices_range(g)) {
        if (build.root == v) {
            continue;
        }
        if (build.anchored_root == v) {
            continue;
        }
        if (g[v].suffix) {
            DEBUG_PRINTF("vertex %zu has suffix\n", g[v].index);
            return true;
        }

    }

    DEBUG_PRINTF("no need for catch-up on report\n");
    return false;
}

static
bool isPureFloating(const RoseResources &resources, const CompileContext &cc) {
    if (!resources.has_floating) {
        DEBUG_PRINTF("no floating table\n");
        return false;
    }

    if (resources.has_outfixes || resources.has_suffixes ||
        resources.has_leftfixes) {
        DEBUG_PRINTF("has engines\n");
        return false;
    }

    if (resources.has_anchored) {
        DEBUG_PRINTF("has anchored matcher\n");
        return false;
    }

    if (resources.has_eod) {
        DEBUG_PRINTF("has eod work to do\n");
        return false;
    }

    if (resources.has_states) {
        DEBUG_PRINTF("has states\n");
        return false;
    }

    if (resources.has_lit_delay) {
        DEBUG_PRINTF("has delayed literals\n");
        return false;
    }

    if (cc.streaming && resources.has_lit_check) {
        DEBUG_PRINTF("has long literals in streaming mode, which needs "
                     "long literal table support\n");
        return false;
    }

    if (resources.checks_groups) {
        DEBUG_PRINTF("has group checks\n");
        return false;
    }

    DEBUG_PRINTF("pure floating literals\n");
    return true;
}

static
bool isSingleOutfix(const RoseBuildImpl &tbi) {
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

    return tbi.outfixes.size() == 1;
}

static
u8 pickRuntimeImpl(const RoseBuildImpl &build, const build_context &bc,
                   UNUSED u32 outfixEndQueue) {
    DEBUG_PRINTF("has_outfixes=%d\n", bc.resources.has_outfixes);
    DEBUG_PRINTF("has_suffixes=%d\n", bc.resources.has_suffixes);
    DEBUG_PRINTF("has_leftfixes=%d\n", bc.resources.has_leftfixes);
    DEBUG_PRINTF("has_literals=%d\n", bc.resources.has_literals);
    DEBUG_PRINTF("has_states=%d\n", bc.resources.has_states);
    DEBUG_PRINTF("checks_groups=%d\n", bc.resources.checks_groups);
    DEBUG_PRINTF("has_lit_delay=%d\n", bc.resources.has_lit_delay);
    DEBUG_PRINTF("has_lit_check=%d\n", bc.resources.has_lit_check);
    DEBUG_PRINTF("has_anchored=%d\n", bc.resources.has_anchored);
    DEBUG_PRINTF("has_floating=%d\n", bc.resources.has_floating);
    DEBUG_PRINTF("has_eod=%d\n", bc.resources.has_eod);

    if (isPureFloating(bc.resources, build.cc)) {
        return ROSE_RUNTIME_PURE_LITERAL;
    }

    if (isSingleOutfix(build)) {
        return ROSE_RUNTIME_SINGLE_OUTFIX;
    }

    return ROSE_RUNTIME_FULL_ROSE;
}

/**
 * \brief True if this Rose engine needs to run MPV catch up in front of
 * non-MPV reports.
 */
static
bool needsMpvCatchup(const RoseBuildImpl &build) {
    const auto &outfixes = build.outfixes;
    bool has_mpv =
        any_of(begin(outfixes), end(outfixes), [](const OutfixInfo &outfix) {
            return outfix.is_nonempty_mpv();
        });

    if (!has_mpv) {
        DEBUG_PRINTF("no mpv\n");
        return false;
    }

    if (isSingleOutfix(build)) {
        DEBUG_PRINTF("single outfix\n");
        return false;
    }

    return true;
}

static
void fillStateOffsets(const RoseBuildImpl &tbi, u32 rolesWithStateCount,
                      u32 anchorStateSize, u32 activeArrayCount,
                      u32 activeLeftCount, u32 laggedRoseCount,
                      u32 longLitStreamStateRequired, u32 historyRequired,
                      RoseStateOffsets *so) {
    u32 curr_offset = 0;

    // First, runtime status (stores per-stream state, like whether we need a
    // delay rebuild or have been told to halt matching.)
    curr_offset += sizeof(u8);

    // Role state storage.
    curr_offset += mmbit_size(rolesWithStateCount);

    so->activeLeafArray = curr_offset; /* TODO: limit size of array */
    curr_offset += mmbit_size(activeArrayCount);

    so->activeLeftArray = curr_offset; /* TODO: limit size of array */
    so->activeLeftArray_size = mmbit_size(activeLeftCount);
    curr_offset += so->activeLeftArray_size;

    so->longLitState = curr_offset;
    curr_offset += longLitStreamStateRequired;

    // ONE WHOLE BYTE for each active leftfix with lag.
    so->leftfixLagTable = curr_offset;
    curr_offset += laggedRoseCount;

    so->anchorState = curr_offset;
    curr_offset += anchorStateSize;

    so->groups = curr_offset;
    so->groups_size = (tbi.group_end + 7) / 8;
    assert(so->groups_size <= sizeof(u64a));
    curr_offset += so->groups_size;

    // The history consists of the bytes in the history only. YAY
    so->history = curr_offset;
    curr_offset += historyRequired;

    // Exhaustion multibit.
    so->exhausted = curr_offset;
    curr_offset += mmbit_size(tbi.rm.numEkeys());

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
    rose_group groups = getSuccGroups(root)
                      | getSuccGroups(anchored_root)
                      | boundary_group_mask;

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
        insert(&tops, g[e].tops);
        if (!g[target(e, g)].char_reach.all()) {
            continue;
        }

        asucc.clear();
        insert(&asucc, adjacent_vertices(target(e, g), g));

        if (asucc == succ) {
            insert(&done_tops, g[e].tops);
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
    assert(isDfaType(dfa_impl->type));

    // If our NFA is an LBR, it always wins.
    if (isLbrType(nfa_impl->type)) {
        return nfa_impl;
    }

    // if our DFA is an accelerated Sheng, it always wins.
    if (isShengType(dfa_impl->type) && has_accel(*dfa_impl)) {
        return dfa_impl;
    }

    bool d_accel = has_accel(*dfa_impl);
    bool n_accel = has_accel(*nfa_impl);
    bool d_big = isBigDfaType(dfa_impl->type);
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
                  const CompileContext &cc, const ReportManager &rm) {
    // If we only have one repeat, the LBR should always be the best possible
    // implementation.
    if (proto.repeats.size() == 1 && cc.grey.allowLbr) {
        return constructLBR(proto, triggers.at(0), cc, rm);
    }

    auto castle_nfa = buildCastle(proto, triggers, cc, rm);
    assert(castle_nfa); // Should always be constructible.
    return castle_nfa;
}

static
aligned_unique_ptr<NFA> getDfa(raw_dfa &rdfa, bool is_transient,
                               const CompileContext &cc,
                               const ReportManager &rm) {
    // Unleash the Sheng!!
    auto dfa = shengCompile(rdfa, cc, rm);
    if (!dfa && !is_transient) {
        // Sheng wasn't successful, so unleash McClellan!
        /* We don't try the hybrid for transient prefixes due to the extra
         * bytecode and that they are usually run on small blocks */
        dfa = mcshengCompile(rdfa, cc, rm);
    }
    if (!dfa) {
        // Sheng wasn't successful, so unleash McClellan!
        dfa = mcclellanCompile(rdfa, cc, rm);
    }
    return dfa;
}

/* builds suffix nfas */
static
aligned_unique_ptr<NFA>
buildSuffix(const ReportManager &rm, const SomSlotManager &ssm,
            const map<u32, u32> &fixed_depth_tops,
            const map<u32, vector<vector<CharReach>>> &triggers,
            suffix_id suff, const CompileContext &cc) {
    if (suff.castle()) {
        auto n = buildRepeatEngine(*suff.castle(), triggers, cc, rm);
        assert(n);
        return n;
    }

    if (suff.haig()) {
        auto n = goughCompile(*suff.haig(), ssm.somPrecision(), cc, rm);
        assert(n);
        return n;
    }

    if (suff.dfa()) {
        auto d = getDfa(*suff.dfa(), false, cc, rm);
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
        auto lbr = constructLBR(holder, triggers.at(0), cc, rm);
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
                auto d = getDfa(*rdfa, false, cc, rm);
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
    const ReportManager &rm = tbi.rm;

    aligned_unique_ptr<NFA> n;

    // Should compress state if this rose is non-transient and we're in
    // streaming mode.
    const bool compress_state = !is_transient;

    assert(is_prefix || !left.graph() || left.graph()->kind == NFA_INFIX);
    assert(!is_prefix || !left.graph() || left.graph()->kind == NFA_PREFIX
           || left.graph()->kind == NFA_EAGER_PREFIX);

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
        n = buildRepeatEngine(*left.castle(), triggers, cc, rm);
        assert(n);
        return n; // Castles/LBRs are always best!
    }

    if (left.dfa()) {
        n = getDfa(*left.dfa(), is_transient, cc, rm);
    } else if (left.graph() && cc.grey.roseMcClellanPrefix == 2 && is_prefix &&
               !is_transient) {
        auto rdfa = buildMcClellan(*left.graph(), nullptr, cc.grey);
        if (rdfa) {
            n = getDfa(*rdfa, is_transient, cc, rm);
            assert(n);
        }
    }

    // We can attempt to build LBRs for infixes.
    if (!n && !is_prefix && left.graph() && onlyOneTop(*left.graph())) {
        map<u32, vector<vector<CharReach> > > triggers;
        findTriggerSequences(tbi, infixTriggers.at(left), &triggers);
        assert(triggers.size() == 1); // single top
        n = constructLBR(*left.graph(), triggers.begin()->second, cc, rm);
    }

    if (!n && left.graph()) {
        map<u32, vector<vector<CharReach>>> triggers;
        if (left.graph()->kind == NFA_INFIX) {
            findTriggerSequences(tbi, infixTriggers.at(left), &triggers);
        }
        n = constructNFA(*left.graph(), nullptr, fixed_depth_tops, triggers,
                         compress_state, cc);
    }

    if (cc.grey.roseMcClellanPrefix == 1 && is_prefix && !left.dfa()
        && left.graph()
        && (!n || !has_bounded_repeats_other_than_firsts(*n) || !is_fast(*n))) {
        auto rdfa = buildMcClellan(*left.graph(), nullptr, cc.grey);
        if (rdfa) {
            auto d = getDfa(*rdfa, is_transient, cc, rm);
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
void appendTailToHolder(NGHolder &h, const flat_set<ReportID> &reports,
                        const vector<NFAVertex> &starts,
                        const vector<CharReach> &tail) {
    assert(!tail.empty());
    NFAVertex curr = add_vertex(h);
    for (NFAVertex v : starts) {
        assert(!edge(v, h.acceptEod, h).second);
        assert(h[v].reports == reports);
        h[v].reports.clear();
        remove_edge(v, h.accept, h);
        add_edge(v, curr, h);
    }
    auto it = tail.begin();
    h[curr].char_reach = *it;
    ++it;
    while (it != tail.end()) {
        NFAVertex old = curr;
        curr = add_vertex(h);
        add_edge(old, curr, h);
        assert(!it->none());
        h[curr].char_reach = *it;
        ++it;
    }

    h[curr].reports = reports;
    add_edge(curr, h.accept, h);
}

static
void appendTailToHolder(NGHolder &h, const vector<CharReach> &tail) {
    assert(in_degree(h.acceptEod, h) == 1);
    assert(!tail.empty());

    map<flat_set<ReportID>, vector<NFAVertex> > reporters;
    for (auto v : inv_adjacent_vertices_range(h.accept, h)) {
        reporters[h[v].reports].push_back(v);
    }

    for (const auto &e : reporters) {
        appendTailToHolder(h, e.first, e.second, tail);
    }

    renumber_edges(h);
}

static
u32 decreaseLag(const RoseBuildImpl &build, NGHolder &h,
                const vector<RoseVertex> &succs) {
    const RoseGraph &rg = build.g;
    static const size_t MAX_RESTORE_LEN = 5;

    vector<CharReach> restored(MAX_RESTORE_LEN);
    for (RoseVertex v : succs) {
        u32 lag = rg[v].left.lag;
        for (u32 lit_id : rg[v].literals) {
            u32 delay = build.literals.right.at(lit_id).delay;
            const ue2_literal &literal = build.literals.right.at(lit_id).s;
            assert(lag <= literal.length() + delay);
            size_t base = literal.length() + delay - lag;
            if (base >= literal.length()) {
                return 0;
            }
            size_t len = literal.length() - base;
            len = MIN(len, restored.size());
            restored.resize(len);
            auto lit_it = literal.begin() + base;
            for (u32 i = 0; i < len; i++) {
                assert(lit_it != literal.end());
                restored[i] |= *lit_it;
                ++lit_it;
            }
        }
    }

    assert(!restored.empty());

    appendTailToHolder(h, restored);

    return restored.size();
}

#define EAGER_DIE_BEFORE_LIMIT 10

struct eager_info {
    shared_ptr<NGHolder> new_graph;
    u32 lag_adjust = 0;
};

static
bool checkSuitableForEager(bool is_prefix, const left_id &left,
                           const RoseBuildImpl &build,
                           const vector<RoseVertex> &succs,
                           rose_group squash_mask, rose_group initial_groups,
                           eager_info &ei, const CompileContext &cc) {
    DEBUG_PRINTF("checking prefix --> %016llx...\n", squash_mask);

    const RoseGraph &rg = build.g;

    if (!is_prefix) {
        DEBUG_PRINTF("not prefix\n");
        return false; /* only prefixes (for now...) */
    }

    if ((initial_groups & squash_mask) == initial_groups) {
        DEBUG_PRINTF("no squash -- useless\n");
        return false;
    }

    for (RoseVertex s : succs) {
        if (build.isInETable(s)
            || contains(rg[s].literals, build.eod_event_literal_id)) {
            return false; /* Ignore EOD related prefixes */
        }
    }

    if (left.dfa()) {
        const raw_dfa &dfa = *left.dfa();
        if (dfa.start_floating != DEAD_STATE) {
            return false; /* not purely anchored */
        }
        if (!dfa.states[dfa.start_anchored].reports.empty()) {
            return false; /* vacuous (todo: handle?) */
        }

        if (!can_die_early(dfa, EAGER_DIE_BEFORE_LIMIT)) {
            return false;
        }
        ei.new_graph = rg[succs[0]].left.graph;
    } else if (left.graph()) {
        const NGHolder &g = *left.graph();
        if (proper_out_degree(g.startDs, g)) {
            return false; /* not purely anchored */
        }

        ei.new_graph = cloneHolder(*left.graph());
        auto gg = ei.new_graph;
        gg->kind = NFA_EAGER_PREFIX;

        ei.lag_adjust = decreaseLag(build, *gg, succs);

        if (is_match_vertex(gg->start, *gg)) {
            return false; /* should not still be vacuous as lag decreased */
        }

        if (!can_die_early(*gg, EAGER_DIE_BEFORE_LIMIT)) {
            DEBUG_PRINTF("not eager as stuck alive\n");
            return false;
        }

        /* We need to ensure that adding in the literals does not cause us to no
         * longer be able to build an nfa. */
        bool ok = isImplementableNFA(*gg, nullptr, cc);
        if (!ok) {
            return false;
        }
    } else {
        DEBUG_PRINTF("unable to determine if good for eager running\n");
        return false;
    }

    DEBUG_PRINTF("eager prefix\n");
    return true;
}

static
left_id updateLeftfixWithEager(RoseGraph &g, const eager_info &ei,
                               const vector<RoseVertex> &succs) {
    u32 lag_adjust = ei.lag_adjust;
    auto gg = ei.new_graph;
    for (RoseVertex v : succs) {
        g[v].left.graph = gg;
        assert(g[v].left.lag >= lag_adjust);
        g[v].left.lag -= lag_adjust;
        DEBUG_PRINTF("added %u literal chars back, new lag %u\n", lag_adjust,
                     g[v].left.lag);
    }
    left_id leftfix = g[succs[0]].left;

    if (leftfix.graph()) {
        assert(leftfix.graph()->kind == NFA_PREFIX
               || leftfix.graph()->kind == NFA_EAGER_PREFIX);
        leftfix.graph()->kind = NFA_EAGER_PREFIX;
    }
    if (leftfix.dfa()) {
        assert(leftfix.dfa()->kind == NFA_PREFIX);
        leftfix.dfa()->kind = NFA_EAGER_PREFIX;
    }

    return leftfix;
}

static
bool buildLeftfix(RoseBuildImpl &build, build_context &bc, bool prefix, u32 qi,
                  const map<left_id, set<PredTopPair> > &infixTriggers,
                  set<u32> *no_retrigger_queues, set<u32> *eager_queues,
                  const map<left_id, eager_info> &eager,
                  const vector<RoseVertex> &succs, left_id leftfix) {
    RoseGraph &g = build.g;
    const CompileContext &cc = build.cc;
    const ReportManager &rm = build.rm;

    bool is_transient = contains(build.transient, leftfix);
    rose_group squash_mask = build.rose_squash_masks.at(leftfix);

    DEBUG_PRINTF("making %sleftfix\n", is_transient ? "transient " : "");

    if (contains(eager, leftfix)) {
        eager_queues->insert(qi);
        leftfix = updateLeftfixWithEager(g, eager.at(leftfix), succs);
    }

    aligned_unique_ptr<NFA> nfa;
    // Need to build NFA, which is either predestined to be a Haig (in SOM mode)
    // or could be all manner of things.
    if (leftfix.haig()) {
        nfa = goughCompile(*leftfix.haig(), build.ssm.somPrecision(), cc, rm);
    }  else {
        nfa = makeLeftNfa(build, leftfix, prefix, is_transient, infixTriggers,
                          cc);
    }

    if (!nfa) {
        assert(!"failed to build leftfix");
        return false;
    }

    setLeftNfaProperties(*nfa, leftfix);

    build.leftfix_queue_map.emplace(leftfix, qi);
    nfa->queueIndex = qi;

    if (!prefix && !leftfix.haig() && leftfix.graph()
        && nfaStuckOn(*leftfix.graph())) {
        DEBUG_PRINTF("%u sticks on\n", qi);
        no_retrigger_queues->insert(qi);
    }

    DEBUG_PRINTF("built leftfix, qi=%u\n", qi);
    add_nfa_to_blob(bc, *nfa);

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
    if (!prefix) {
        set<ue2_literal> lits;
        for (RoseVertex v : succs) {
            for (auto u : inv_adjacent_vertices_range(v, g)) {
                for (u32 lit_id : g[u].literals) {
                    lits.insert(build.literals.right.at(lit_id).s);
                }
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

    for (RoseVertex v : succs) {
        bc.leftfix_info.emplace(v, left_build_info(qi, g[v].left.lag, max_width,
                                                   squash_mask, stop,
                                                   max_queuelen, cm_count,
                                                   cm_cr));
    }

    return true;
}

static
unique_ptr<TamaInfo> constructTamaInfo(const RoseGraph &g,
                     const vector<ExclusiveSubengine> &subengines,
                     const bool is_suffix) {
    unique_ptr<TamaInfo> tamaInfo = ue2::make_unique<TamaInfo>();
    for (const auto &sub : subengines) {
        const auto &rose_vertices = sub.vertices;
        NFA *nfa = sub.nfa.get();
        set<u32> tops;
        for (const auto &v : rose_vertices) {
            if (is_suffix) {
                tops.insert(g[v].suffix.top);
            } else {
                for (const auto &e : in_edges_range(v, g)) {
                    tops.insert(g[e].rose_top);
                }
            }
        }
        tamaInfo->add(nfa, tops);
    }

    return tamaInfo;
}

static
void updateTops(const RoseGraph &g, const TamaInfo &tamaInfo,
                TamaProto &tamaProto,
                const vector<ExclusiveSubengine> &subengines,
                const map<pair<const NFA *, u32>, u32> &out_top_remap,
                const bool is_suffix) {
    u32 i = 0;
    for (const auto &n : tamaInfo.subengines) {
        for (const auto &v : subengines[i].vertices) {
            if (is_suffix) {
                tamaProto.add(n, g[v].index, g[v].suffix.top,
                              out_top_remap);
            } else {
                for (const auto &e : in_edges_range(v, g)) {
                    tamaProto.add(n, g[v].index, g[e].rose_top,
                                  out_top_remap);
                }
            }
        }
        i++;
    }
}

static
shared_ptr<TamaProto> constructContainerEngine(const RoseGraph &g,
                                               build_context &bc,
                                               const ExclusiveInfo &info,
                                               const u32 queue,
                                               const bool is_suffix) {
    const auto &subengines = info.subengines;
    auto tamaInfo =
        constructTamaInfo(g, subengines, is_suffix);

    map<pair<const NFA *, u32>, u32> out_top_remap;
    auto n = buildTamarama(*tamaInfo, queue, out_top_remap);
    add_nfa_to_blob(bc, *n);

    DEBUG_PRINTF("queue id:%u\n", queue);
    shared_ptr<TamaProto> tamaProto = make_shared<TamaProto>();
    tamaProto->reports = info.reports;
    updateTops(g, *tamaInfo, *tamaProto, subengines,
               out_top_remap, is_suffix);
    return tamaProto;
}

static
void buildInfixContainer(RoseGraph &g, build_context &bc,
                         const vector<ExclusiveInfo> &exclusive_info) {
    // Build tamarama engine
    for (const auto &info : exclusive_info) {
        const u32 queue = info.queue;
        const auto &subengines = info.subengines;
        auto tamaProto =
            constructContainerEngine(g, bc, info, queue, false);

        for (const auto &sub : subengines) {
            const auto &verts = sub.vertices;
            for (const auto &v : verts) {
                DEBUG_PRINTF("vert id:%zu\n", g[v].index);
                g[v].left.tamarama = tamaProto;
            }
        }
    }
}

static
void buildSuffixContainer(RoseGraph &g, build_context &bc,
                          const vector<ExclusiveInfo> &exclusive_info) {
    // Build tamarama engine
    for (const auto &info : exclusive_info) {
        const u32 queue = info.queue;
        const auto &subengines = info.subengines;
        auto tamaProto =
            constructContainerEngine(g, bc, info, queue, true);
        for (const auto &sub : subengines) {
            const auto &verts = sub.vertices;
            for (const auto &v : verts) {
                DEBUG_PRINTF("vert id:%zu\n", g[v].index);
                g[v].suffix.tamarama = tamaProto;
            }
            const auto &v = verts[0];
            suffix_id newSuffix(g[v].suffix);
            bc.suffixes.emplace(newSuffix, queue);
        }
    }
}

static
void updateExclusiveInfixProperties(const RoseBuildImpl &build,
                                    build_context &bc,
                                    const vector<ExclusiveInfo> &exclusive_info,
                                    set<u32> *no_retrigger_queues) {
    const RoseGraph &g = build.g;
    for (const auto &info : exclusive_info) {
        // Set leftfix optimisations, disabled for tamarama subengines
        rose_group squash_mask = ~rose_group{0};
        // Leftfixes can have stop alphabets.
        vector<u8> stop(N_CHARS, 0);
        // Infix NFAs can have bounds on their queue lengths.
        u32 max_queuelen = 0;
        u32 max_width = 0;
        u8 cm_count = 0;
        CharReach cm_cr;

        const auto &qi = info.queue;
        const auto &subengines = info.subengines;
        bool no_retrigger = true;
        for (const auto &sub : subengines) {
            const auto &verts = sub.vertices;
            const auto &v_first = verts[0];
            left_id leftfix(g[v_first].left);
            if (leftfix.haig() || !leftfix.graph() ||
                !nfaStuckOn(*leftfix.graph())) {
                no_retrigger = false;
            }

            for (const auto &v : verts) {
                set<ue2_literal> lits;
                for (auto u : inv_adjacent_vertices_range(v, build.g)) {
                    for (u32 lit_id : build.g[u].literals) {
                        lits.insert(build.literals.right.at(lit_id).s);
                    }
                }
                DEBUG_PRINTF("%zu literals\n", lits.size());

                u32 queuelen = findMaxInfixMatches(leftfix, lits);
                if (queuelen < UINT32_MAX) {
                    queuelen++;
                }
                max_queuelen = max(max_queuelen, queuelen);
            }
        }

        if (no_retrigger) {
            no_retrigger_queues->insert(qi);
        }

        for (const auto &sub : subengines) {
            const auto &verts = sub.vertices;
            for (const auto &v : verts) {
                u32 lag = g[v].left.lag;
                bc.leftfix_info.emplace(
                    v, left_build_info(qi, lag, max_width, squash_mask, stop,
                                       max_queuelen, cm_count, cm_cr));
            }
        }
    }
}

static
void updateExclusiveSuffixProperties(const RoseBuildImpl &build,
                                const vector<ExclusiveInfo> &exclusive_info,
                                set<u32> *no_retrigger_queues) {
    const RoseGraph &g = build.g;
    for (auto &info : exclusive_info) {
        const auto &qi = info.queue;
        const auto &subengines = info.subengines;
        bool no_retrigger = true;
        for (const auto &sub : subengines) {
            const auto &v_first = sub.vertices[0];
            suffix_id suffix(g[v_first].suffix);
            if (!suffix.graph() || !nfaStuckOn(*suffix.graph())) {
                no_retrigger = false;
                break;
            }
        }

        if (no_retrigger) {
            no_retrigger_queues->insert(qi);
        }
    }
}

static
void buildExclusiveInfixes(RoseBuildImpl &build, build_context &bc,
                           QueueIndexFactory &qif,
                           const map<left_id, set<PredTopPair>> &infixTriggers,
                           const map<u32, vector<RoseVertex>> &vertex_map,
                           const vector<vector<u32>> &groups,
                           set<u32> *no_retrigger_queues) {
    RoseGraph &g = build.g;
    const CompileContext &cc = build.cc;

    vector<ExclusiveInfo> exclusive_info;
    for (const auto &gp : groups) {
        ExclusiveInfo info;
        for (const auto &id : gp) {
            const auto &verts = vertex_map.at(id);
            left_id leftfix(g[verts[0]].left);

            bool is_transient = false;
            auto n = makeLeftNfa(build, leftfix, false, is_transient,
                                 infixTriggers, cc);
            assert(n);

            setLeftNfaProperties(*n, leftfix);

            ExclusiveSubengine engine;
            engine.nfa = move(n);
            engine.vertices = verts;
            info.subengines.push_back(move(engine));
        }
        info.queue = qif.get_queue();
        exclusive_info.push_back(move(info));
    }
    updateExclusiveInfixProperties(build, bc, exclusive_info,
                                   no_retrigger_queues);
    buildInfixContainer(g, bc, exclusive_info);
}

static
void findExclusiveInfixes(RoseBuildImpl &build, build_context &bc,
                          QueueIndexFactory &qif,
                          const map<left_id, set<PredTopPair>> &infixTriggers,
                          set<u32> *no_retrigger_queues) {
    const RoseGraph &g = build.g;

    set<RoleInfo<left_id>> roleInfoSet;
    map<u32, vector<RoseVertex>> vertex_map;

    u32 role_id = 0;
    map<left_id, u32> leftfixes;
    for (auto v : vertices_range(g)) {
        if (!g[v].left || build.isRootSuccessor(v)) {
            continue;
        }

        left_id leftfix(g[v].left);

        // Sanity check: our NFA should contain each of the tops mentioned on
        // our in-edges.
        assert(roseHasTops(build, v));

        if (contains(leftfixes, leftfix)) {
            // NFA already built.
            u32 id = leftfixes[leftfix];
            if (contains(vertex_map, id)) {
                vertex_map[id].push_back(v);
            }
            DEBUG_PRINTF("sharing leftfix, id=%u\n", id);
            continue;
        }

        if (leftfix.graph() || leftfix.castle()) {
            leftfixes.emplace(leftfix, role_id);
            vertex_map[role_id].push_back(v);

            map<u32, vector<vector<CharReach>>> triggers;
            findTriggerSequences(build, infixTriggers.at(leftfix), &triggers);
            RoleInfo<left_id> info(leftfix, role_id);
            if (setTriggerLiteralsInfix(info, triggers)) {
                roleInfoSet.insert(info);
            }
            role_id++;
        }
    }

    if (leftfixes.size() > 1) {
        DEBUG_PRINTF("leftfix size:%zu\n", leftfixes.size());
        vector<vector<u32>> groups;
        exclusiveAnalysisInfix(build, vertex_map, roleInfoSet, groups);
        buildExclusiveInfixes(build, bc, qif, infixTriggers, vertex_map,
                              groups, no_retrigger_queues);
    }
}

static
bool buildLeftfixes(RoseBuildImpl &tbi, build_context &bc,
                    QueueIndexFactory &qif, set<u32> *no_retrigger_queues,
                    set<u32> *eager_queues, bool do_prefix) {
    RoseGraph &g = tbi.g;
    const CompileContext &cc = tbi.cc;

    map<left_id, set<PredTopPair> > infixTriggers;
    vector<left_id> order;
    unordered_map<left_id, vector<RoseVertex> > succs;
    findInfixTriggers(tbi, &infixTriggers);

    if (cc.grey.allowTamarama && cc.streaming && !do_prefix) {
        findExclusiveInfixes(tbi, bc, qif, infixTriggers,
                             no_retrigger_queues);
    }

    for (auto v : vertices_range(g)) {
        if (!g[v].left || g[v].left.tamarama) {
            continue;
        }

        assert(tbi.isNonRootSuccessor(v) != tbi.isRootSuccessor(v));
        bool is_prefix = tbi.isRootSuccessor(v);

        if (do_prefix != is_prefix) {
            /* we require prefixes and then infixes */
            continue;
        }

        left_id leftfix(g[v].left);

        // Sanity check: our NFA should contain each of the tops mentioned on
        // our in-edges.
        assert(roseHasTops(tbi, v));

        bool is_transient = contains(tbi.transient, leftfix);

        // Transient leftfixes can sometimes be implemented solely with
        // lookarounds, in which case we don't need to build an engine.
        // TODO: Handle SOM-tracking cases as well.
        if (cc.grey.roseLookaroundMasks && is_transient &&
            !g[v].left.tracksSom()) {
            vector<LookEntry> lookaround;
            if (makeLeftfixLookaround(tbi, v, lookaround)) {
                DEBUG_PRINTF("implementing as lookaround!\n");
                bc.leftfix_info.emplace(v, left_build_info(lookaround));
                continue;
            }
        }

        if (!contains(succs, leftfix)) {
            order.push_back(leftfix);
        }

        succs[leftfix].push_back(v);
    }

    rose_group initial_groups = tbi.getInitialGroups();
    rose_group combined_eager_squashed_mask = ~0ULL;

    map<left_id, eager_info> eager;

    for (const left_id &leftfix : order) {
        const auto &left_succs = succs[leftfix];

        rose_group squash_mask = tbi.rose_squash_masks.at(leftfix);
        eager_info ei;

        if (checkSuitableForEager(do_prefix, leftfix, tbi, left_succs,
                                  squash_mask, initial_groups, ei, cc)) {
            eager[leftfix] = ei;
            combined_eager_squashed_mask &= squash_mask;
            DEBUG_PRINTF("combo %016llx...\n", combined_eager_squashed_mask);
        }
    }

    if (do_prefix && combined_eager_squashed_mask & initial_groups) {
        DEBUG_PRINTF("eager groups won't squash everyone - be lazy\n");
        eager_queues->clear();
        eager.clear();
    }

    for (const left_id &leftfix : order) {
        buildLeftfix(tbi, bc, do_prefix, qif.get_queue(), infixTriggers,
                     no_retrigger_queues, eager_queues, eager, succs[leftfix],
                     leftfix);
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

namespace {
class OutfixBuilder : public boost::static_visitor<aligned_unique_ptr<NFA>> {
public:
    explicit OutfixBuilder(const RoseBuildImpl &build_in) : build(build_in) {}

    aligned_unique_ptr<NFA> operator()(boost::blank&) const {
        return nullptr;
    };

    aligned_unique_ptr<NFA> operator()(unique_ptr<raw_dfa> &rdfa) const {
        // Unleash the mighty DFA!
        return getDfa(*rdfa, false, build.cc, build.rm);
    }

    aligned_unique_ptr<NFA> operator()(unique_ptr<raw_som_dfa> &haig) const {
        // Unleash the Goughfish!
        return goughCompile(*haig, build.ssm.somPrecision(), build.cc,
                            build.rm);
    }

    aligned_unique_ptr<NFA> operator()(unique_ptr<NGHolder> &holder) const {
        const CompileContext &cc = build.cc;
        const ReportManager &rm = build.rm;

        NGHolder &h = *holder;
        assert(h.kind == NFA_OUTFIX);

        // Build NFA.
        const map<u32, u32> fixed_depth_tops; /* no tops */
        const map<u32, vector<vector<CharReach>>> triggers; /* no tops */
        bool compress_state = cc.streaming;
        auto n = constructNFA(h, &rm, fixed_depth_tops, triggers,
                              compress_state, cc);

        // Try for a DFA upgrade.
        if (n && cc.grey.roseMcClellanOutfix &&
            !has_bounded_repeats_other_than_firsts(*n)) {
            auto rdfa = buildMcClellan(h, &rm, cc.grey);
            if (rdfa) {
                auto d = getDfa(*rdfa, false, cc, rm);
                if (d) {
                    n = pickImpl(move(d), move(n));
                }
            }
        }

        return n;
    }

    aligned_unique_ptr<NFA> operator()(UNUSED MpvProto &mpv) const {
        // MPV construction handled separately.
        assert(mpv.puffettes.empty());
        return nullptr;
    }

private:
    const RoseBuildImpl &build;
};
}

static
aligned_unique_ptr<NFA> buildOutfix(RoseBuildImpl &build, OutfixInfo &outfix) {
    assert(!outfix.is_dead()); // should not be marked dead.

    auto n = boost::apply_visitor(OutfixBuilder(build), outfix.proto);
    if (n && build.cc.grey.reverseAccelerate) {
        buildReverseAcceleration(n.get(), outfix.rev_info, outfix.minWidth);
    }

    return n;
}

static
void prepMpv(RoseBuildImpl &tbi, build_context &bc, size_t *historyRequired,
             bool *mpv_as_outfix) {
    assert(bc.engineOffsets.empty()); // MPV should be first
    *mpv_as_outfix = false;
    OutfixInfo *mpv_outfix = nullptr;

    /* assume outfixes are just above chain tails in queue indices */
    for (auto &out : tbi.outfixes) {
        if (out.is_nonempty_mpv()) {
            assert(!mpv_outfix);
            mpv_outfix = &out;
        } else {
            assert(!out.mpv());
        }
    }

    if (!mpv_outfix) {
        return;
    }

    auto *mpv = mpv_outfix->mpv();
    auto nfa = mpvCompile(mpv->puffettes, mpv->triggered_puffettes, tbi.rm);
    assert(nfa);
    if (!nfa) {
        throw CompileError("Unable to generate bytecode.");
    }

    if (tbi.cc.grey.reverseAccelerate) {
        buildReverseAcceleration(nfa.get(), mpv_outfix->rev_info,
                                 mpv_outfix->minWidth);
    }

    u32 qi = mpv_outfix->get_queue(tbi.qif);
    nfa->queueIndex = qi;

    DEBUG_PRINTF("built mpv\n");

    if (!*historyRequired && requires_decompress_key(*nfa)) {
        *historyRequired = 1;
    }

    add_nfa_to_blob(bc, *nfa);
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
bool prepOutfixes(RoseBuildImpl &tbi, build_context &bc,
                  size_t *historyRequired) {
    if (tbi.cc.grey.onlyOneOutfix && tbi.outfixes.size() > 1) {
        DEBUG_PRINTF("we have %zu outfixes, but Grey::onlyOneOutfix is set\n",
                     tbi.outfixes.size());
        throw ResourceLimitError();
    }

    assert(tbi.qif.allocated_count() == bc.engineOffsets.size());

    for (auto &out : tbi.outfixes) {
        if (out.mpv()) {
            continue; /* already done */
        }
        DEBUG_PRINTF("building outfix %zd\n", &out - &tbi.outfixes[0]);
        auto n = buildOutfix(tbi, out);
        if (!n) {
            assert(0);
            return false;
        }

        setOutfixProperties(*n, out);

        n->queueIndex = out.get_queue(tbi.qif);

        if (!*historyRequired && requires_decompress_key(*n)) {
            *historyRequired = 1;
        }

        add_nfa_to_blob(bc, *n);
    }

    return true;
}

static
void assignSuffixQueues(RoseBuildImpl &build, build_context &bc) {
    const RoseGraph &g = build.g;

    for (auto v : vertices_range(g)) {
        if (!g[v].suffix) {
            continue;
        }

        const suffix_id s(g[v].suffix);

        DEBUG_PRINTF("vertex %zu triggers suffix %p\n", g[v].index, s.graph());

        // We may have already built this NFA.
        if (contains(bc.suffixes, s)) {
            continue;
        }

        u32 queue = build.qif.get_queue();
        DEBUG_PRINTF("assigning %p to queue %u\n", s.graph(), queue);
        bc.suffixes.emplace(s, queue);
        build.suffix_queue_map.emplace(s, queue);
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
void buildExclusiveSuffixes(RoseBuildImpl &build, build_context &bc,
                            QueueIndexFactory &qif,
                            map<suffix_id, set<PredTopPair>> &suffixTriggers,
                            const map<u32, vector<RoseVertex>> &vertex_map,
                            const vector<vector<u32>> &groups,
                            set<u32> *no_retrigger_queues) {
    RoseGraph &g = build.g;

    vector<ExclusiveInfo> exclusive_info;
    for (const auto &gp : groups) {
        ExclusiveInfo info;
        for (const auto &id : gp) {
            const auto &verts = vertex_map.at(id);
            suffix_id s(g[verts[0]].suffix);

            const set<PredTopPair> &s_triggers = suffixTriggers.at(s);

            map<u32, u32> fixed_depth_tops;
            findFixedDepthTops(g, s_triggers, &fixed_depth_tops);

            map<u32, vector<vector<CharReach>>> triggers;
            findTriggerSequences(build, s_triggers, &triggers);

            auto n = buildSuffix(build.rm, build.ssm, fixed_depth_tops,
                                 triggers, s, build.cc);
            assert(n);

            setSuffixProperties(*n, s, build.rm);

            ExclusiveSubengine engine;
            engine.nfa = move(n);
            engine.vertices = verts;
            info.subengines.push_back(move(engine));

            const auto &reports = all_reports(s);
            info.reports.insert(reports.begin(), reports.end());
        }
        info.queue = qif.get_queue();
        exclusive_info.push_back(move(info));
    }
    updateExclusiveSuffixProperties(build, exclusive_info,
                                    no_retrigger_queues);
    buildSuffixContainer(g, bc, exclusive_info);
}

static
void findExclusiveSuffixes(RoseBuildImpl &tbi, build_context &bc,
                  QueueIndexFactory &qif,
                  map<suffix_id, set<PredTopPair>> &suffixTriggers,
                  set<u32> *no_retrigger_queues) {
    const RoseGraph &g = tbi.g;

    map<suffix_id, u32> suffixes;
    set<RoleInfo<suffix_id>> roleInfoSet;
    map<u32, vector<RoseVertex>> vertex_map;
    u32 role_id = 0;
    for (auto v : vertices_range(g)) {
        if (!g[v].suffix) {
            continue;
        }

        const suffix_id s(g[v].suffix);

        DEBUG_PRINTF("vertex %zu triggers suffix %p\n", g[v].index, s.graph());

        // We may have already built this NFA.
        if (contains(suffixes, s)) {
            u32 id = suffixes[s];
            if (!tbi.isInETable(v)) {
                vertex_map[id].push_back(v);
            }
            continue;
        }

        // Currently disable eod suffixes for exclusive analysis
        if (!tbi.isInETable(v) && (s.graph() || s.castle())) {
            DEBUG_PRINTF("assigning %p to id %u\n", s.graph(), role_id);
            suffixes.emplace(s, role_id);

            vertex_map[role_id].push_back(v);
            const set<PredTopPair> &s_triggers = suffixTriggers.at(s);
            map<u32, vector<vector<CharReach>>> triggers;
            findTriggerSequences(tbi, s_triggers, &triggers);

            RoleInfo<suffix_id> info(s, role_id);
            if (setTriggerLiteralsSuffix(info, triggers)) {
                roleInfoSet.insert(info);
            }
            role_id++;
        }
    }

    if (suffixes.size() > 1) {
        DEBUG_PRINTF("suffix size:%zu\n", suffixes.size());
        vector<vector<u32>> groups;
        exclusiveAnalysisSuffix(tbi, vertex_map, roleInfoSet, groups);
        buildExclusiveSuffixes(tbi, bc, qif, suffixTriggers, vertex_map,
                               groups, no_retrigger_queues);
    }
}

static
bool buildSuffixes(const RoseBuildImpl &tbi, build_context &bc,
                   set<u32> *no_retrigger_queues,
                   const map<suffix_id, set<PredTopPair>> &suffixTriggers) {
    // To ensure compile determinism, build suffix engines in order of their
    // (unique) queue indices, so that we call add_nfa_to_blob in the same
    // order.
    vector<pair<u32, suffix_id>> ordered;
    for (const auto &e : bc.suffixes) {
        ordered.emplace_back(e.second, e.first);
    }
    sort(begin(ordered), end(ordered));

    for (const auto &e : ordered) {
        const u32 queue = e.first;
        const suffix_id &s = e.second;

        if (s.tamarama()) {
            continue;
        }

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

        add_nfa_to_blob(bc, *n);
    }

    return true;
}

static
void buildCountingMiracles(build_context &bc) {
    map<pair<CharReach, u8>, u32> pre_built;

    for (left_build_info &lbi : bc.leftfix_info | map_values) {
        if (!lbi.countingMiracleCount) {
            continue;
        }

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
            int rv = shuftiBuildMasks(cr, (u8 *)&rcm.lo, (u8 *)&rcm.hi);
            if (rv == -1) {
                DEBUG_PRINTF("failed to build shufti\n");
                lbi.countingMiracleCount = 0; /* remove counting miracle */
                continue;
            }

            rcm.poison = (~cr).find_first();
        }

        rcm.count = lbi.countingMiracleCount;

        lbi.countingMiracleOffset = bc.engine_blob.add(rcm);
        pre_built[key] = lbi.countingMiracleOffset;
        DEBUG_PRINTF("built cm for count of %u @ %u\n", rcm.count,
                     lbi.countingMiracleOffset);
    }
}

/* Note: buildNfas may reduce the lag for vertices that have prefixes */
static
bool buildNfas(RoseBuildImpl &tbi, build_context &bc, QueueIndexFactory &qif,
               set<u32> *no_retrigger_queues, set<u32> *eager_queues,
               u32 *leftfixBeginQueue) {
    map<suffix_id, set<PredTopPair>> suffixTriggers;
    findSuffixTriggers(tbi, &suffixTriggers);

    if (tbi.cc.grey.allowTamarama && tbi.cc.streaming) {
        findExclusiveSuffixes(tbi, bc, qif, suffixTriggers,
                              no_retrigger_queues);
    }

    assignSuffixQueues(tbi, bc);

    if (!buildSuffixes(tbi, bc, no_retrigger_queues, suffixTriggers)) {
        return false;
    }
    suffixTriggers.clear();

    *leftfixBeginQueue = qif.allocated_count();

    if (!buildLeftfixes(tbi, bc, qif, no_retrigger_queues, eager_queues,
                        true)) {
        return false;
    }

    if (!buildLeftfixes(tbi, bc, qif, no_retrigger_queues, eager_queues,
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
    for (const auto &build : leftfix_info | map_values) {
        if (build.transient) {
            DEBUG_PRINTF("q %u is transient\n", build.queue);
            out->insert(build.queue);
        }
    }
}

static
void updateNfaState(const build_context &bc, RoseStateOffsets *so,
                    NfaInfo *nfa_infos, u32 *fullStateSize, u32 *nfaStateSize,
                    u32 *tStateSize) {
    *nfaStateSize = 0;
    *tStateSize = 0;
    *fullStateSize = 0;

    set<u32> transient_queues;
    findTransientQueues(bc.leftfix_info, &transient_queues);

    for (const auto &m : bc.engineOffsets) {
        const NFA *n = get_nfa_from_blob(bc, m.first);
        allocateStateSpace(n, transient_queues, so, nfa_infos, fullStateSize,
                           nfaStateSize, tStateSize);
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
u32 buildLastByteIter(const RoseGraph &g, build_context &bc) {
    vector<u32> lb_roles;

    for (auto v : vertices_range(g)) {
        if (!hasLastByteHistorySucc(g, v)) {
            continue;
        }
        // Eager EOD reporters won't have state indices.
        auto it = bc.roleStateIndices.find(v);
        if (it != end(bc.roleStateIndices)) {
            lb_roles.push_back(it->second);
            DEBUG_PRINTF("last byte %u\n", it->second);
        }
    }

    if (lb_roles.empty()) {
        return 0; /* invalid offset */
    }

    vector<mmbit_sparse_iter> iter;
    mmbBuildSparseIterator(iter, lb_roles, bc.numStates);
    return bc.engine_blob.add_iterator(iter);
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

static
u32 findMinFloatingLiteralMatch(const RoseBuildImpl &build,
                                const vector<raw_dfa> &anchored_dfas) {
    if (anchored_dfas.size() > 1) {
        DEBUG_PRINTF("multiple anchored dfas\n");
        /* We must regard matches from other anchored tables as unordered, as
         * we do for floating matches. */
        return 1;
    }

    const RoseGraph &g = build.g;
    u32 minWidth = ROSE_BOUND_INF;
    for (auto v : vertices_range(g)) {
        if (build.isAnchored(v) || build.isVirtualVertex(v)) {
            DEBUG_PRINTF("skipping %zu anchored or root\n", g[v].index);
            continue;
        }

        u32 w = g[v].min_offset;
        DEBUG_PRINTF("%zu m_o = %u\n", g[v].index, w);

        if (w < minWidth) {
            minWidth = w;
        }
    }

    return minWidth;
}

static
void buildSuffixEkeyLists(const RoseBuildImpl &tbi, build_context &bc,
                          const QueueIndexFactory &qif,
                          vector<u32> *out) {
    out->resize(qif.allocated_count());

    map<u32, vector<u32> > qi_to_ekeys; /* for determinism */

    for (const auto &e : bc.suffixes) {
        const suffix_id &s = e.first;
        u32 qi = e.second;
        set<u32> ekeys = reportsToEkeys(all_reports(s), tbi.rm);

        if (!ekeys.empty()) {
            qi_to_ekeys[qi] = {ekeys.begin(), ekeys.end()};
        }
    }

    /* for each outfix also build elists */
    for (const auto &outfix : tbi.outfixes) {
        u32 qi = outfix.get_queue();
        set<u32> ekeys = reportsToEkeys(all_reports(outfix), tbi.rm);

        if (!ekeys.empty()) {
            qi_to_ekeys[qi] = {ekeys.begin(), ekeys.end()};
        }
    }

    for (auto &e : qi_to_ekeys) {
        assert(!e.second.empty());
        e.second.push_back(INVALID_EKEY); /* terminator */
        (*out)[e.first] = bc.engine_blob.add(e.second.begin(),
                                             e.second.end());
    }
}

/** Returns sparse iter offset in engine blob. */
static
u32 buildEodNfaIterator(build_context &bc, const u32 activeQueueCount) {
    vector<u32> keys;
    for (u32 qi = 0; qi < activeQueueCount; ++qi) {
        const NFA *n = get_nfa_from_blob(bc, qi);
        if (nfaAcceptsEod(n)) {
            DEBUG_PRINTF("nfa qi=%u accepts eod\n", qi);
            keys.push_back(qi);
        }
    }

    if (keys.empty()) {
        return 0;
    }

    DEBUG_PRINTF("building iter for %zu nfas\n", keys.size());

    vector<mmbit_sparse_iter> iter;
    mmbBuildSparseIterator(iter, keys, activeQueueCount);
    return bc.engine_blob.add_iterator(iter);
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
        if (hasMpvTrigger(all_reports(out), tbi.rm)) {
            return true;
        }
    }

    return false;
}

static
void populateNfaInfoBasics(const RoseBuildImpl &build, const build_context &bc,
                           const vector<OutfixInfo> &outfixes,
                           const vector<u32> &ekeyListOffsets,
                           const set<u32> &no_retrigger_queues,
                           NfaInfo *infos) {
    const u32 num_queues = build.qif.allocated_count();
    for (u32 qi = 0; qi < num_queues; qi++) {
        const NFA *n = get_nfa_from_blob(bc, qi);
        enforceEngineSizeLimit(n, n->length, build.cc.grey);

        NfaInfo &info = infos[qi];
        info.nfaOffset = bc.engineOffsets.at(qi);
        info.ekeyListOffset = ekeyListOffsets[qi];
        info.no_retrigger = contains(no_retrigger_queues, qi) ? 1 : 0;
    }

    // Mark outfixes that are in the small block matcher.
    for (const auto &out : outfixes) {
        const u32 qi = out.get_queue();
        infos[qi].in_sbmatcher = out.in_sbmatcher;
    }

    // Mark suffixes triggered by EOD table literals.
    const RoseGraph &g = build.g;
    for (auto v : vertices_range(g)) {
        if (!g[v].suffix) {
            continue;
        }
        u32 qi = bc.suffixes.at(g[v].suffix);
        if (build.isInETable(v)) {
            infos[qi].eod = 1;
        }
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
void applyFinalSpecialisation(RoseProgram &program) {
    assert(!program.empty());
    assert(program.back().code() == ROSE_INSTR_END);
    if (program.size() < 2) {
        return;
    }

    /* Replace the second-to-last instruction (before END) with a one-shot
     * specialisation if available. */
    auto it = next(program.rbegin());
    if (auto *ri = dynamic_cast<const RoseInstrReport *>(it->get())) {
        DEBUG_PRINTF("replacing REPORT with FINAL_REPORT\n");
        program.replace(it, make_unique<RoseInstrFinalReport>(
                                ri->onmatch, ri->offset_adjust));
    }
}

static
void recordResources(RoseResources &resources, const RoseProgram &program) {
    for (const auto &ri : program) {
        switch (ri->code()) {
        case ROSE_INSTR_TRIGGER_SUFFIX:
            resources.has_suffixes = true;
            break;
        case ROSE_INSTR_TRIGGER_INFIX:
        case ROSE_INSTR_CHECK_INFIX:
        case ROSE_INSTR_CHECK_PREFIX:
        case ROSE_INSTR_SOM_LEFTFIX:
            resources.has_leftfixes = true;
            break;
        case ROSE_INSTR_SET_STATE:
        case ROSE_INSTR_CHECK_STATE:
        case ROSE_INSTR_SPARSE_ITER_BEGIN:
        case ROSE_INSTR_SPARSE_ITER_NEXT:
            resources.has_states = true;
            break;
        case ROSE_INSTR_CHECK_GROUPS:
            resources.checks_groups = true;
            break;
        case ROSE_INSTR_PUSH_DELAYED:
            resources.has_lit_delay = true;
            break;
        case ROSE_INSTR_CHECK_LONG_LIT:
        case ROSE_INSTR_CHECK_LONG_LIT_NOCASE:
            resources.has_lit_check = true;
            break;
        default:
            break;
        }
    }
}

static
void recordResources(RoseResources &resources,
                     const RoseBuildImpl &build) {
    if (!build.outfixes.empty()) {
        resources.has_outfixes = true;
    }
    for (u32 i = 0; i < build.literal_info.size(); i++) {
        if (build.hasFinalId(i)) {
            resources.has_literals = true;
            break;
        }
    }

    const auto &g = build.g;
    for (const auto &v : vertices_range(g)) {
        if (g[v].eod_accept) {
            resources.has_eod = true;
            break;
        }
        if (g[v].suffix && has_eod_accepts(g[v].suffix)) {
            resources.has_eod = true;
            break;
        }
    }
}

static
void recordLongLiterals(build_context &bc, const RoseProgram &program) {
    for (const auto &ri : program) {
        if (const auto *ri_check =
                dynamic_cast<const RoseInstrCheckLongLit *>(ri.get())) {
            DEBUG_PRINTF("found CHECK_LONG_LIT for string '%s'\n",
                         escapeString(ri_check->literal).c_str());
            bc.longLiterals.emplace_back(ri_check->literal, false);
            continue;
        }
        if (const auto *ri_check =
                dynamic_cast<const RoseInstrCheckLongLitNocase *>(ri.get())) {
            DEBUG_PRINTF("found CHECK_LONG_LIT_NOCASE for string '%s'\n",
                         escapeString(ri_check->literal).c_str());
            bc.longLiterals.emplace_back(ri_check->literal, true);
        }
    }
}

static
u32 writeProgram(build_context &bc, RoseProgram &&program) {
    if (program.empty()) {
        DEBUG_PRINTF("no program\n");
        return 0;
    }

    auto it = bc.program_cache.find(program);
    if (it != end(bc.program_cache)) {
        DEBUG_PRINTF("reusing cached program at %u\n", it->second);
        return it->second;
    }

    recordResources(bc.resources, program);
    recordLongLiterals(bc, program);

    u32 len = 0;
    auto prog_bytecode = writeProgram(bc.engine_blob, program, &len);
    u32 offset = bc.engine_blob.add(prog_bytecode.get(), len,
                                    ROSE_INSTR_MIN_ALIGN);
    DEBUG_PRINTF("prog len %u written at offset %u\n", len, offset);
    bc.program_cache.emplace(move(program), offset);
    return offset;
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
bool canEagerlyReportAtEod(const RoseBuildImpl &build, const RoseEdge &e) {
    const auto &g = build.g;
    const auto v = target(e, g);

    if (!build.g[v].eod_accept) {
        return false;
    }

    // If there's a graph between us and EOD, we shouldn't be eager.
    if (build.g[v].left) {
        return false;
    }

    // Must be exactly at EOD.
    if (g[e].minBound != 0 || g[e].maxBound != 0) {
        return false;
    }

    // In streaming mode, we can only eagerly report EOD for literals in the
    // EOD-anchored table, as that's the only time we actually know where EOD
    // is. In block mode, we always have this information.
    const auto u = source(e, g);
    if (build.cc.streaming && !build.isInETable(u)) {
        return false;
    }

    return true;
}

static
bool hasEodAnchors(const RoseBuildImpl &build, const build_context &bc,
                   u32 outfixEndQueue) {
    for (u32 i = 0; i < outfixEndQueue; i++) {
        if (nfaAcceptsEod(get_nfa_from_blob(bc, i))) {
            DEBUG_PRINTF("outfix has eod\n");
            return true;
        }
    }

    if (build.eod_event_literal_id != MO_INVALID_IDX) {
        DEBUG_PRINTF("eod is an event to be celebrated\n");
        return true;
    }

    const RoseGraph &g = build.g;
    for (auto v : vertices_range(g)) {
        if (g[v].eod_accept) {
            DEBUG_PRINTF("literally report eod\n");
            return true;
        }
        if (g[v].suffix && has_eod_accepts(g[v].suffix)) {
            DEBUG_PRINTF("eod suffix\n");
            return true;
        }
    }
    DEBUG_PRINTF("yawn\n");
    return false;
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

/**
 * \brief True if the given vertex is a role that can only be switched on at
 * EOD.
 */
static
bool onlyAtEod(const RoseBuildImpl &tbi, RoseVertex v) {
    const RoseGraph &g = tbi.g;

    // All such roles have only (0,0) edges to vertices with the eod_accept
    // property, and no other effects (suffixes, ordinary reports, etc, etc).

    if (isLeafNode(v, g) || !g[v].reports.empty() || g[v].suffix) {
        return false;
    }

    for (const auto &e : out_edges_range(v, g)) {
        RoseVertex w = target(e, g);
        if (!g[w].eod_accept) {
            return false;
        }
        assert(!g[w].reports.empty());
        assert(g[w].literals.empty());

        if (g[e].minBound || g[e].maxBound) {
            return false;
        }
    }

    /* There is no pointing enforcing this check at runtime if
     * this role is only fired by the eod event literal */
    if (tbi.eod_event_literal_id != MO_INVALID_IDX &&
        g[v].literals.size() == 1 &&
        *g[v].literals.begin() == tbi.eod_event_literal_id) {
        return false;
    }

    return true;
}

static
u32 addLookaround(build_context &bc, const vector<LookEntry> &look) {
    // Check the cache.
    auto it = bc.lookaround_cache.find(look);
    if (it != bc.lookaround_cache.end()) {
        DEBUG_PRINTF("reusing look at idx %zu\n", it->second);
        return verify_u32(it->second);
    }

    // Linear scan for sequence.
    auto seq_it = search(begin(bc.lookaround), end(bc.lookaround), begin(look),
                         end(look));
    if (seq_it != end(bc.lookaround)) {
        size_t idx = distance(begin(bc.lookaround), seq_it);
        DEBUG_PRINTF("linear scan found look at idx %zu\n", idx);
        bc.lookaround_cache.emplace(look, idx);
        return verify_u32(idx);
    }

    // New sequence.
    size_t idx = bc.lookaround.size();
    bc.lookaround_cache.emplace(look, idx);
    insert(&bc.lookaround, bc.lookaround.end(), look);
    DEBUG_PRINTF("adding look at idx %zu\n", idx);
    return verify_u32(idx);
}

static
bool checkReachMask(const CharReach &cr, u8 &andmask, u8 &cmpmask) {
    size_t reach_size = cr.count();
    assert(reach_size > 0);
    // check whether entry_size is some power of 2.
    if ((reach_size - 1) & reach_size) {
        return false;
    }
    make_and_cmp_mask(cr, &andmask, &cmpmask);
    if ((1 << popcount32((u8)(~andmask))) ^ reach_size) {
        return false;
    }
    return true;
}

static
bool checkReachWithFlip(const CharReach &cr, u8 &andmask,
                       u8 &cmpmask, u8 &flip) {
    if (checkReachMask(cr, andmask, cmpmask)) {
        flip = 0;
        return true;
    }
    if (checkReachMask(~cr, andmask, cmpmask)) {
        flip = 1;
        return true;
    }
    return false;
}

static
bool makeRoleByte(const vector<LookEntry> &look, RoseProgram &program) {
    if (look.size() == 1) {
        const auto &entry = look[0];
        u8 andmask_u8, cmpmask_u8;
        u8 flip;
        if (!checkReachWithFlip(entry.reach, andmask_u8, cmpmask_u8, flip)) {
            return false;
        }
        s32 checkbyte_offset = verify_s32(entry.offset);
        DEBUG_PRINTF("CHECK BYTE offset=%d\n", checkbyte_offset);
        const auto *end_inst = program.end_instruction();
        auto ri = make_unique<RoseInstrCheckByte>(andmask_u8, cmpmask_u8, flip,
                                                  checkbyte_offset, end_inst);
        program.add_before_end(move(ri));
        return true;
    }
    return false;
}

static
bool makeRoleMask(const vector<LookEntry> &look, RoseProgram &program) {
    if (look.back().offset < look.front().offset + 8) {
        s32 base_offset = verify_s32(look.front().offset);
        u64a and_mask = 0;
        u64a cmp_mask = 0;
        u64a neg_mask = 0;
        for (const auto &entry : look) {
            u8 andmask_u8, cmpmask_u8, flip;
            if (!checkReachWithFlip(entry.reach, andmask_u8,
                                    cmpmask_u8, flip)) {
                return false;
            }
            DEBUG_PRINTF("entry offset %d\n", entry.offset);
            u32 shift = (entry.offset - base_offset) << 3;
            and_mask |= (u64a)andmask_u8 << shift;
            cmp_mask |= (u64a)cmpmask_u8 << shift;
            if (flip) {
                neg_mask |= 0xffLLU << shift;
            }
        }
        DEBUG_PRINTF("CHECK MASK and_mask=%llx cmp_mask=%llx\n",
                     and_mask, cmp_mask);
        const auto *end_inst = program.end_instruction();
        auto ri = make_unique<RoseInstrCheckMask>(and_mask, cmp_mask, neg_mask,
                                                  base_offset, end_inst);
        program.add_before_end(move(ri));
        return true;
    }
    return false;
}

static UNUSED
string convertMaskstoString(u8 *p, int byte_len) {
    string s;
    for (int i = 0; i < byte_len; i++) {
        u8 hi = *p >> 4;
        u8 lo = *p & 0xf;
        s += (char)(hi + (hi < 10 ? 48 : 87));
        s += (char)(lo + (lo < 10 ? 48 : 87));
        p++;
    }
    return s;
}

static
bool makeRoleMask32(const vector<LookEntry> &look,
                    RoseProgram &program) {
    if (look.back().offset >= look.front().offset + 32) {
        return false;
    }
    s32 base_offset = verify_s32(look.front().offset);
    array<u8, 32> and_mask, cmp_mask;
    and_mask.fill(0);
    cmp_mask.fill(0);
    u32 neg_mask = 0;
    for (const auto &entry : look) {
        u8 andmask_u8, cmpmask_u8, flip;
        if (!checkReachWithFlip(entry.reach, andmask_u8,
                                cmpmask_u8, flip)) {
            return false;
        }
        u32 shift = entry.offset - base_offset;
        assert(shift < 32);
        and_mask[shift] = andmask_u8;
        cmp_mask[shift] = cmpmask_u8;
        if (flip) {
            neg_mask |= 1 << shift;
        }
    }

    DEBUG_PRINTF("and_mask %s\n",
                 convertMaskstoString(and_mask.data(), 32).c_str());
    DEBUG_PRINTF("cmp_mask %s\n",
                 convertMaskstoString(cmp_mask.data(), 32).c_str());
    DEBUG_PRINTF("neg_mask %08x\n", neg_mask);
    DEBUG_PRINTF("base_offset %d\n", base_offset);

    const auto *end_inst = program.end_instruction();
    auto ri = make_unique<RoseInstrCheckMask32>(and_mask, cmp_mask, neg_mask,
                                                base_offset, end_inst);
    program.add_before_end(move(ri));
    return true;
}

// Sorting by the size of every bucket.
// Used in map<u32, vector<s8>, cmpNibble>.
struct cmpNibble {
    bool operator()(const u32 data1, const u32 data2) const{
        u32 size1 = popcount32(data1 >> 16) * popcount32(data1 << 16);
        u32 size2 = popcount32(data2 >> 16) * popcount32(data2 << 16);
        return std::tie(size1, data1) < std::tie(size2, data2);
    }
};

// Insert all pairs of bucket and offset into buckets.
static really_inline
void getAllBuckets(const vector<LookEntry> &look,
                 map<u32, vector<s8>, cmpNibble> &buckets, u32 &neg_mask) {
    s32 base_offset = verify_s32(look.front().offset);
    for (const auto &entry : look) {
        CharReach cr = entry.reach;
        // Flip heavy character classes to save buckets.
        if (cr.count() > 128 ) {
            cr.flip();
        } else {
            neg_mask ^= 1 << (entry.offset - base_offset);
        }
        map <u16, u16> lo2hi;
        // We treat Ascii Table as a 16x16 grid.
        // Push every row in cr into lo2hi and mark the row number.
        for (size_t i = cr.find_first(); i != CharReach::npos;) {
            u8 it_hi = i >> 4;
            u16 low_encode = 0;
            while (i != CharReach::npos && (i >> 4) == it_hi) {
                low_encode |= 1 << (i & 0xf);
                i = cr.find_next(i);
            }
            lo2hi[low_encode] |= 1 << it_hi;
        }
        for (const auto &it : lo2hi) {
            u32 hi_lo = (it.second << 16) | it.first;
            buckets[hi_lo].push_back(entry.offset);
        }
    }
}

// Once we have a new bucket, we'll try to combine it with all old buckets.
static really_inline
void nibUpdate(map<u32, u16> &nib, u32 hi_lo) {
    u16 hi = hi_lo >> 16;
    u16 lo = hi_lo & 0xffff;
    for (const auto pairs : nib) {
        u32 old = pairs.first;
        if ((old >> 16) == hi || (old & 0xffff) == lo) {
            if (!nib[old | hi_lo]) {
                nib[old | hi_lo] = nib[old] | nib[hi_lo];
            }
        }
    }
}

static really_inline
void nibMaskUpdate(array<u8, 32> &mask, u32 data, u8 bit_index) {
    for (u8 index = 0; data > 0; data >>= 1, index++) {
        if (data & 1) {
            // 0 ~ 7 bucket in first 16 bytes,
            // 8 ~ 15 bucket in second 16 bytes.
            if (bit_index >= 8) {
                mask[index + 16] |= 1 << (bit_index - 8);
            } else {
                mask[index] |= 1 << bit_index;
            }
        }
    }
}

static
bool makeRoleShufti(const vector<LookEntry> &look,
                    RoseProgram &program) {

    s32 base_offset = verify_s32(look.front().offset);
    if (look.back().offset >= base_offset + 32) {
        return false;
    }
    array<u8, 32> hi_mask, lo_mask;
    hi_mask.fill(0);
    lo_mask.fill(0);
    array<u8, 32> bucket_select_hi, bucket_select_lo;
    bucket_select_hi.fill(0); // will not be used in 16x8 and 32x8.
    bucket_select_lo.fill(0);
    u8 bit_index = 0; // number of buckets
    map<u32, u16> nib; // map every bucket to its bucket number.
    map<u32, vector<s8>, cmpNibble> bucket2offsets;
    u32 neg_mask = ~0u;

    getAllBuckets(look, bucket2offsets, neg_mask);

    for (const auto &it : bucket2offsets) {
        u32 hi_lo = it.first;
        // New bucket.
        if (!nib[hi_lo]) {
            if (bit_index >= 16) {
                return false;
            }
            nib[hi_lo] = 1 << bit_index;

            nibUpdate(nib, hi_lo);
            nibMaskUpdate(hi_mask, hi_lo >> 16, bit_index);
            nibMaskUpdate(lo_mask, hi_lo & 0xffff, bit_index);
            bit_index++;
        }

        DEBUG_PRINTF("hi_lo %x bucket %x\n", hi_lo, nib[hi_lo]);

        // Update bucket_select_mask.
        u8 nib_hi = nib[hi_lo] >> 8;
        u8 nib_lo = nib[hi_lo] & 0xff;
        for (const auto offset : it.second) {
            bucket_select_hi[offset - base_offset] |= nib_hi;
            bucket_select_lo[offset - base_offset] |= nib_lo;
        }
    }

    DEBUG_PRINTF("hi_mask %s\n",
                 convertMaskstoString(hi_mask.data(), 32).c_str());
    DEBUG_PRINTF("lo_mask %s\n",
                 convertMaskstoString(lo_mask.data(), 32).c_str());
    DEBUG_PRINTF("bucket_select_hi %s\n",
                 convertMaskstoString(bucket_select_hi.data(), 32).c_str());
    DEBUG_PRINTF("bucket_select_lo %s\n",
                 convertMaskstoString(bucket_select_lo.data(), 32).c_str());

    const auto *end_inst = program.end_instruction();
    if (bit_index < 8) {
        if (look.back().offset < base_offset + 16) {
            neg_mask &= 0xffff;
            array<u8, 32> nib_mask;
            array<u8, 16> bucket_select_mask_16;
            copy(lo_mask.begin(), lo_mask.begin() + 16, nib_mask.begin());
            copy(hi_mask.begin(), hi_mask.begin() + 16, nib_mask.begin() + 16);
            copy(bucket_select_lo.begin(), bucket_select_lo.begin() + 16,
                 bucket_select_mask_16.begin());
            auto ri = make_unique<RoseInstrCheckShufti16x8>
                      (nib_mask, bucket_select_mask_16,
                       neg_mask, base_offset, end_inst);
            program.add_before_end(move(ri));
        } else {
            array<u8, 16> hi_mask_16;
            array<u8, 16> lo_mask_16;
            copy(hi_mask.begin(), hi_mask.begin() + 16, hi_mask_16.begin());
            copy(lo_mask.begin(), lo_mask.begin() + 16, lo_mask_16.begin());
            auto ri = make_unique<RoseInstrCheckShufti32x8>
                      (hi_mask_16, lo_mask_16, bucket_select_lo,
                       neg_mask, base_offset, end_inst);
            program.add_before_end(move(ri));
        }
    } else {
        if (look.back().offset < base_offset + 16) {
            neg_mask &= 0xffff;
            array<u8, 32> bucket_select_mask_32;
            copy(bucket_select_lo.begin(), bucket_select_lo.begin() + 16,
                 bucket_select_mask_32.begin());
            copy(bucket_select_hi.begin(), bucket_select_hi.begin() + 16,
                 bucket_select_mask_32.begin() + 16);
            auto ri = make_unique<RoseInstrCheckShufti16x16>
                      (hi_mask, lo_mask, bucket_select_mask_32,
                       neg_mask, base_offset, end_inst);
            program.add_before_end(move(ri));
        } else {
            auto ri = make_unique<RoseInstrCheckShufti32x16>
                      (hi_mask, lo_mask, bucket_select_hi, bucket_select_lo,
                       neg_mask, base_offset, end_inst);
            program.add_before_end(move(ri));
        }
    }
    return true;
}

/**
 * Builds a lookaround instruction, or an appropriate specialization if one is
 * available.
 */
static
void makeLookaroundInstruction(build_context &bc, const vector<LookEntry> &look,
                               RoseProgram &program) {
    assert(!look.empty());

    if (makeRoleByte(look, program)) {
        return;
    }

    if (look.size() == 1) {
        s8 offset = look.begin()->offset;
        u32 look_idx = addLookaround(bc, look);
        auto ri = make_unique<RoseInstrCheckSingleLookaround>(offset, look_idx,
                                                    program.end_instruction());
        program.add_before_end(move(ri));
        return;
    }

    if (makeRoleMask(look, program)) {
        return;
    }

    if (makeRoleMask32(look, program)) {
        return;
    }

    if (makeRoleShufti(look, program)) {
        return;
    }

    u32 look_idx = addLookaround(bc, look);
    u32 look_count = verify_u32(look.size());

    auto ri = make_unique<RoseInstrCheckLookaround>(look_idx, look_count,
                                                    program.end_instruction());
    program.add_before_end(move(ri));
}

static
void makeRoleLookaround(RoseBuildImpl &build, build_context &bc, RoseVertex v,
                        RoseProgram &program) {
    if (!build.cc.grey.roseLookaroundMasks) {
        return;
    }

    vector<LookEntry> look;

    // Lookaround from leftfix (mandatory).
    if (contains(bc.leftfix_info, v) && bc.leftfix_info.at(v).has_lookaround) {
        DEBUG_PRINTF("using leftfix lookaround\n");
        look = bc.leftfix_info.at(v).lookaround;
    }

    // We may be able to find more lookaround info (advisory) and merge it
    // in.
    vector<LookEntry> look_more;
    findLookaroundMasks(build, v, look_more);
    mergeLookaround(look, look_more);

    if (look.empty()) {
        return;
    }

    makeLookaroundInstruction(bc, look, program);
}

static
void makeRoleCheckLeftfix(RoseBuildImpl &build, build_context &bc, RoseVertex v,
                          RoseProgram &program) {
    auto it = bc.leftfix_info.find(v);
    if (it == end(bc.leftfix_info)) {
        return;
    }
    const left_build_info &lni = it->second;
    if (lni.has_lookaround) {
        return; // Leftfix completely implemented by lookaround.
    }

    assert(!build.cc.streaming ||
           build.g[v].left.lag <= MAX_STORED_LEFTFIX_LAG);

    bool is_prefix = build.isRootSuccessor(v);
    const auto *end_inst = program.end_instruction();

    unique_ptr<RoseInstruction> ri;
    if (is_prefix) {
        ri = make_unique<RoseInstrCheckPrefix>(lni.queue, build.g[v].left.lag,
                                               build.g[v].left.leftfix_report,
                                               end_inst);
    } else {
        ri = make_unique<RoseInstrCheckInfix>(lni.queue, build.g[v].left.lag,
                                              build.g[v].left.leftfix_report,
                                              end_inst);
    }
    program.add_before_end(move(ri));
}

static
void makeRoleAnchoredDelay(RoseBuildImpl &build, build_context &bc,
                           RoseVertex v, RoseProgram &program) {
    // Only relevant for roles that can be triggered by the anchored table.
    if (!build.isAnchored(v)) {
        return;
    }

    // If this match cannot occur after floatingMinLiteralMatchOffset, we do
    // not need this check.
    if (build.g[v].max_offset <= bc.floatingMinLiteralMatchOffset) {
        return;
    }

    const auto *end_inst = program.end_instruction();
    auto ri = make_unique<RoseInstrAnchoredDelay>(build.g[v].groups, end_inst);
    program.add_before_end(move(ri));
}

static
void makeDedupe(const RoseBuildImpl &build, const Report &report,
                RoseProgram &program) {
    const auto *end_inst = program.end_instruction();
    auto ri =
        make_unique<RoseInstrDedupe>(report.quashSom, build.rm.getDkey(report),
                                     report.offsetAdjust, end_inst);
    program.add_before_end(move(ri));
}

static
void makeDedupeSom(const RoseBuildImpl &build, const Report &report,
                   RoseProgram &program) {
    const auto *end_inst = program.end_instruction();
    auto ri = make_unique<RoseInstrDedupeSom>(report.quashSom,
                                              build.rm.getDkey(report),
                                              report.offsetAdjust, end_inst);
    program.add_before_end(move(ri));
}

static
void makeCatchup(RoseBuildImpl &build, build_context &bc,
                 const flat_set<ReportID> &reports, RoseProgram &program) {
    if (!bc.needs_catchup) {
        return;
    }

    // Everything except the INTERNAL_ROSE_CHAIN report needs catchup to run
    // before reports are triggered.

    auto report_needs_catchup = [&](const ReportID &id) {
        const Report &report = build.rm.getReport(id);
        return report.type != INTERNAL_ROSE_CHAIN;
    };

    if (!any_of(begin(reports), end(reports), report_needs_catchup)) {
        DEBUG_PRINTF("none of the given reports needs catchup\n");
        return;
    }

    program.add_before_end(make_unique<RoseInstrCatchUp>());
}

static
void makeCatchupMpv(RoseBuildImpl &build, build_context &bc, ReportID id,
                    RoseProgram &program) {
    if (!bc.needs_mpv_catchup) {
        return;
    }

    const Report &report = build.rm.getReport(id);
    if (report.type == INTERNAL_ROSE_CHAIN) {
        return;
    }

    program.add_before_end(make_unique<RoseInstrCatchUpMpv>());
}

static
void writeSomOperation(const Report &report, som_operation *op) {
    assert(op);

    memset(op, 0, sizeof(*op));

    switch (report.type) {
    case EXTERNAL_CALLBACK_SOM_REL:
        op->type = SOM_EXTERNAL_CALLBACK_REL;
        break;
    case INTERNAL_SOM_LOC_SET:
        op->type = SOM_INTERNAL_LOC_SET;
        break;
    case INTERNAL_SOM_LOC_SET_IF_UNSET:
        op->type = SOM_INTERNAL_LOC_SET_IF_UNSET;
        break;
    case INTERNAL_SOM_LOC_SET_IF_WRITABLE:
        op->type = SOM_INTERNAL_LOC_SET_IF_WRITABLE;
        break;
    case INTERNAL_SOM_LOC_SET_SOM_REV_NFA:
        op->type = SOM_INTERNAL_LOC_SET_REV_NFA;
        break;
    case INTERNAL_SOM_LOC_SET_SOM_REV_NFA_IF_UNSET:
        op->type = SOM_INTERNAL_LOC_SET_REV_NFA_IF_UNSET;
        break;
    case INTERNAL_SOM_LOC_SET_SOM_REV_NFA_IF_WRITABLE:
        op->type = SOM_INTERNAL_LOC_SET_REV_NFA_IF_WRITABLE;
        break;
    case INTERNAL_SOM_LOC_COPY:
        op->type = SOM_INTERNAL_LOC_COPY;
        break;
    case INTERNAL_SOM_LOC_COPY_IF_WRITABLE:
        op->type = SOM_INTERNAL_LOC_COPY_IF_WRITABLE;
        break;
    case INTERNAL_SOM_LOC_MAKE_WRITABLE:
        op->type = SOM_INTERNAL_LOC_MAKE_WRITABLE;
        break;
    case EXTERNAL_CALLBACK_SOM_STORED:
        op->type = SOM_EXTERNAL_CALLBACK_STORED;
        break;
    case EXTERNAL_CALLBACK_SOM_ABS:
        op->type = SOM_EXTERNAL_CALLBACK_ABS;
        break;
    case EXTERNAL_CALLBACK_SOM_REV_NFA:
        op->type = SOM_EXTERNAL_CALLBACK_REV_NFA;
        break;
    case INTERNAL_SOM_LOC_SET_FROM:
        op->type = SOM_INTERNAL_LOC_SET_FROM;
        break;
    case INTERNAL_SOM_LOC_SET_FROM_IF_WRITABLE:
        op->type = SOM_INTERNAL_LOC_SET_FROM_IF_WRITABLE;
        break;
    default:
        // This report doesn't correspond to a SOM operation.
        assert(0);
        throw CompileError("Unable to generate bytecode.");
    }

    op->onmatch = report.onmatch;

    switch (report.type) {
    case EXTERNAL_CALLBACK_SOM_REV_NFA:
    case INTERNAL_SOM_LOC_SET_SOM_REV_NFA:
    case INTERNAL_SOM_LOC_SET_SOM_REV_NFA_IF_UNSET:
    case INTERNAL_SOM_LOC_SET_SOM_REV_NFA_IF_WRITABLE:
        op->aux.revNfaIndex = report.revNfaIndex;
        break;
    default:
        op->aux.somDistance = report.somDistance;
        break;
    }
}

static
void makeReport(RoseBuildImpl &build, const ReportID id,
                const bool has_som, RoseProgram &program) {
    assert(id < build.rm.numReports());
    const Report &report = build.rm.getReport(id);

    RoseProgram report_block;
    const RoseInstruction *end_inst = report_block.end_instruction();

    // Handle min/max offset checks.
    if (report.minOffset > 0 || report.maxOffset < MAX_OFFSET) {
        auto ri = make_unique<RoseInstrCheckBounds>(report.minOffset,
                                                    report.maxOffset, end_inst);
        report_block.add_before_end(move(ri));
    }

    // If this report has an exhaustion key, we can check it in the program
    // rather than waiting until we're in the callback adaptor.
    if (report.ekey != INVALID_EKEY) {
        auto ri = make_unique<RoseInstrCheckExhausted>(report.ekey, end_inst);
        report_block.add_before_end(move(ri));
    }

    // External SOM reports that aren't passthrough need their SOM value
    // calculated.
    if (isExternalSomReport(report) &&
        report.type != EXTERNAL_CALLBACK_SOM_PASS) {
        auto ri = make_unique<RoseInstrSomFromReport>();
        writeSomOperation(report, &ri->som);
        report_block.add_before_end(move(ri));
    }

    // Min length constraint.
    if (report.minLength > 0) {
        assert(build.hasSom);
        auto ri = make_unique<RoseInstrCheckMinLength>(
            report.offsetAdjust, report.minLength, end_inst);
        report_block.add_before_end(move(ri));
    }

    if (report.quashSom) {
        report_block.add_before_end(make_unique<RoseInstrSomZero>());
    }

    switch (report.type) {
    case EXTERNAL_CALLBACK:
        if (!has_som) {
            // Dedupe is only necessary if this report has a dkey, or if there
            // are SOM reports to catch up.
            bool needs_dedupe = build.rm.getDkey(report) != ~0U || build.hasSom;
            if (report.ekey == INVALID_EKEY) {
                if (needs_dedupe) {
                    report_block.add_before_end(
                        make_unique<RoseInstrDedupeAndReport>(
                            report.quashSom, build.rm.getDkey(report),
                            report.onmatch, report.offsetAdjust, end_inst));
                } else {
                    report_block.add_before_end(make_unique<RoseInstrReport>(
                        report.onmatch, report.offsetAdjust));
                }
            } else {
                if (needs_dedupe) {
                    makeDedupe(build, report, report_block);
                }
                report_block.add_before_end(make_unique<RoseInstrReportExhaust>(
                    report.onmatch, report.offsetAdjust, report.ekey));
            }
        } else { // has_som
            makeDedupeSom(build, report, report_block);
            if (report.ekey == INVALID_EKEY) {
                report_block.add_before_end(make_unique<RoseInstrReportSom>(
                    report.onmatch, report.offsetAdjust));
            } else {
                report_block.add_before_end(
                    make_unique<RoseInstrReportSomExhaust>(
                        report.onmatch, report.offsetAdjust, report.ekey));
            }
        }
        break;
    case INTERNAL_SOM_LOC_SET:
    case INTERNAL_SOM_LOC_SET_IF_UNSET:
    case INTERNAL_SOM_LOC_SET_IF_WRITABLE:
    case INTERNAL_SOM_LOC_SET_SOM_REV_NFA:
    case INTERNAL_SOM_LOC_SET_SOM_REV_NFA_IF_UNSET:
    case INTERNAL_SOM_LOC_SET_SOM_REV_NFA_IF_WRITABLE:
    case INTERNAL_SOM_LOC_COPY:
    case INTERNAL_SOM_LOC_COPY_IF_WRITABLE:
    case INTERNAL_SOM_LOC_MAKE_WRITABLE:
    case INTERNAL_SOM_LOC_SET_FROM:
    case INTERNAL_SOM_LOC_SET_FROM_IF_WRITABLE:
        if (has_som) {
            auto ri = make_unique<RoseInstrReportSomAware>();
            writeSomOperation(report, &ri->som);
            report_block.add_before_end(move(ri));
        } else {
            auto ri = make_unique<RoseInstrReportSomInt>();
            writeSomOperation(report, &ri->som);
            report_block.add_before_end(move(ri));
        }
        break;
    case INTERNAL_ROSE_CHAIN: {
        report_block.add_before_end(make_unique<RoseInstrReportChain>(
            report.onmatch, report.topSquashDistance));
        break;
    }
    case EXTERNAL_CALLBACK_SOM_REL:
    case EXTERNAL_CALLBACK_SOM_STORED:
    case EXTERNAL_CALLBACK_SOM_ABS:
    case EXTERNAL_CALLBACK_SOM_REV_NFA:
        makeDedupeSom(build, report, report_block);
        if (report.ekey == INVALID_EKEY) {
            report_block.add_before_end(make_unique<RoseInstrReportSom>(
                report.onmatch, report.offsetAdjust));
        } else {
            report_block.add_before_end(make_unique<RoseInstrReportSomExhaust>(
                report.onmatch, report.offsetAdjust, report.ekey));
        }
        break;
    case EXTERNAL_CALLBACK_SOM_PASS:
        makeDedupeSom(build, report, report_block);
        if (report.ekey == INVALID_EKEY) {
            report_block.add_before_end(make_unique<RoseInstrReportSom>(
                report.onmatch, report.offsetAdjust));
        } else {
            report_block.add_before_end(make_unique<RoseInstrReportSomExhaust>(
                report.onmatch, report.offsetAdjust, report.ekey));
        }
        break;

    default:
        assert(0);
        throw CompileError("Unable to generate bytecode.");
    }

    assert(!report_block.empty());
    program.add_block(move(report_block));
}

static
void makeRoleReports(RoseBuildImpl &build, build_context &bc, RoseVertex v,
                     RoseProgram &program) {
    const auto &g = build.g;

    /* we are a suffaig - need to update role to provide som to the
     * suffix. */
    bool has_som = false;
    if (g[v].left.tracksSom()) {
        assert(contains(bc.leftfix_info, v));
        const left_build_info &lni = bc.leftfix_info.at(v);
        program.add_before_end(
            make_unique<RoseInstrSomLeftfix>(lni.queue, g[v].left.lag));
        has_som = true;
    } else if (g[v].som_adjust) {
        program.add_before_end(
            make_unique<RoseInstrSomAdjust>(g[v].som_adjust));
        has_som = true;
    }

    const auto &reports = g[v].reports;
    makeCatchup(build, bc, reports, program);

    RoseProgram report_block;
    for (ReportID id : reports) {
        makeReport(build, id, has_som, report_block);
    }
    program.add_before_end(move(report_block));
}

static
void makeRoleSuffix(RoseBuildImpl &build, build_context &bc, RoseVertex v,
                    RoseProgram &program) {
    const auto &g = build.g;
    if (!g[v].suffix) {
        return;
    }
    assert(contains(bc.suffixes, g[v].suffix));
    u32 qi = bc.suffixes.at(g[v].suffix);
    assert(contains(bc.engineOffsets, qi));
    const NFA *nfa = get_nfa_from_blob(bc, qi);
    u32 suffixEvent;
    if (isContainerType(nfa->type)) {
        auto tamaProto = g[v].suffix.tamarama.get();
        assert(tamaProto);
        u32 top = (u32)MQE_TOP_FIRST +
                  tamaProto->top_remap.at(make_pair(g[v].index,
                                                    g[v].suffix.top));
        assert(top < MQE_INVALID);
        suffixEvent = top;
    } else if (isMultiTopType(nfa->type)) {
        assert(!g[v].suffix.haig);
        u32 top = (u32)MQE_TOP_FIRST + g[v].suffix.top;
        assert(top < MQE_INVALID);
        suffixEvent = top;
    } else {
        // DFAs/Puffs have no MQE_TOP_N support, so they get a classic TOP
        // event.
        assert(!g[v].suffix.graph || onlyOneTop(*g[v].suffix.graph));
        suffixEvent = MQE_TOP;
    }
    program.add_before_end(
        make_unique<RoseInstrTriggerSuffix>(qi, suffixEvent));
}

static
void makeRoleGroups(RoseBuildImpl &build, build_context &bc, RoseVertex v,
                    RoseProgram &program) {
    const auto &g = build.g;
    rose_group groups = g[v].groups;
    if (!groups) {
        return;
    }

    // The set of "already on" groups as we process this vertex is the
    // intersection of the groups set by our predecessors.
    assert(in_degree(v, g) > 0);
    rose_group already_on = ~rose_group{0};
    for (const auto &u : inv_adjacent_vertices_range(v, g)) {
        already_on &= bc.vertex_group_map.at(u);
    }

    DEBUG_PRINTF("already_on=0x%llx\n", already_on);
    DEBUG_PRINTF("squashable=0x%llx\n", bc.squashable_groups);
    DEBUG_PRINTF("groups=0x%llx\n", groups);

    already_on &= ~bc.squashable_groups;
    DEBUG_PRINTF("squashed already_on=0x%llx\n", already_on);

    // We don't *have* to mask off the groups that we know are already on, but
    // this will make bugs more apparent.
    groups &= ~already_on;

    if (!groups) {
        DEBUG_PRINTF("no new groups to set, skipping\n");
        return;
    }

    program.add_before_end(make_unique<RoseInstrSetGroups>(groups));
}

static
void makeRoleInfixTriggers(RoseBuildImpl &build, build_context &bc,
                           RoseVertex u, RoseProgram &program) {
    const auto &g = build.g;

    vector<RoseInstrTriggerInfix> infix_program;

    for (const auto &e : out_edges_range(u, g)) {
        RoseVertex v = target(e, g);
        if (!g[v].left) {
            continue;
        }

        assert(contains(bc.leftfix_info, v));
        const left_build_info &lbi = bc.leftfix_info.at(v);
        if (lbi.has_lookaround) {
            continue;
        }

        const NFA *nfa = get_nfa_from_blob(bc, lbi.queue);

        // DFAs have no TOP_N support, so they get a classic MQE_TOP event.
        u32 top;
        if (isContainerType(nfa->type)) {
            auto tamaProto = g[v].left.tamarama.get();
            assert(tamaProto);
            top = MQE_TOP_FIRST + tamaProto->top_remap.at(
                                      make_pair(g[v].index, g[e].rose_top));
            assert(top < MQE_INVALID);
        } else if (!isMultiTopType(nfa->type)) {
            assert(num_tops(g[v].left) == 1);
            top = MQE_TOP;
        } else {
            top = MQE_TOP_FIRST + g[e].rose_top;
            assert(top < MQE_INVALID);
        }

        infix_program.emplace_back(g[e].rose_cancel_prev_top, lbi.queue, top);
    }

    if (infix_program.empty()) {
        return;
    }

    // Order, de-dupe and add instructions to the end of program.
    sort(begin(infix_program), end(infix_program),
         [](const RoseInstrTriggerInfix &a, const RoseInstrTriggerInfix &b) {
             return tie(a.cancel, a.queue, a.event) <
                    tie(b.cancel, b.queue, b.event);
         });
    infix_program.erase(unique(begin(infix_program), end(infix_program)),
                        end(infix_program));
    for (const auto &ri : infix_program) {
        program.add_before_end(make_unique<RoseInstrTriggerInfix>(ri));
    }
}

static
void makeRoleSetState(const build_context &bc, RoseVertex v,
                      RoseProgram &program) {
    // We only need this instruction if a state index has been assigned to this
    // vertex.
    auto it = bc.roleStateIndices.find(v);
    if (it == end(bc.roleStateIndices)) {
        return;
    }
    program.add_before_end(make_unique<RoseInstrSetState>(it->second));
}

static
void makeRoleCheckBounds(const RoseBuildImpl &build, RoseVertex v,
                         const RoseEdge &e, RoseProgram &program) {
    const RoseGraph &g = build.g;
    const RoseVertex u = source(e, g);

    // We know that we can trust the anchored table (DFA) to always deliver us
    // literals at the correct offset.
    if (build.isAnchored(v)) {
        DEBUG_PRINTF("literal in anchored table, skipping bounds check\n");
        return;
    }

    // Use the minimum literal length.
    u32 lit_length = g[v].eod_accept ? 0 : verify_u32(build.minLiteralLen(v));

    u64a min_bound = g[e].minBound + lit_length;
    u64a max_bound = g[e].maxBound == ROSE_BOUND_INF
                         ? ROSE_BOUND_INF
                         : g[e].maxBound + lit_length;

    if (g[e].history == ROSE_ROLE_HISTORY_ANCH) {
        assert(g[u].fixedOffset());
        // Make offsets absolute.
        min_bound += g[u].max_offset;
        if (max_bound != ROSE_BOUND_INF) {
            max_bound += g[u].max_offset;
        }
    }

    assert(max_bound <= ROSE_BOUND_INF);
    assert(min_bound <= max_bound);

    // CHECK_BOUNDS instruction uses 64-bit bounds, so we can use MAX_OFFSET
    // (max value of a u64a) to represent ROSE_BOUND_INF.
    if (max_bound == ROSE_BOUND_INF) {
        max_bound = MAX_OFFSET;
    }

    // This instruction should be doing _something_ -- bounds should be tighter
    // than just {length, inf}.
    assert(min_bound > lit_length || max_bound < MAX_OFFSET);

    const auto *end_inst = program.end_instruction();
    program.add_before_end(
        make_unique<RoseInstrCheckBounds>(min_bound, max_bound, end_inst));
}

static
void makeRoleCheckNotHandled(build_context &bc, RoseVertex v,
                             RoseProgram &program) {
    u32 handled_key;
    if (contains(bc.handledKeys, v)) {
        handled_key = bc.handledKeys.at(v);
    } else {
        handled_key = verify_u32(bc.handledKeys.size());
        bc.handledKeys.emplace(v, handled_key);
    }

    const auto *end_inst = program.end_instruction();
    auto ri = make_unique<RoseInstrCheckNotHandled>(handled_key, end_inst);
    program.add_before_end(move(ri));
}

static
void makeRoleEagerEodReports(RoseBuildImpl &build, build_context &bc,
                             RoseVertex v, RoseProgram &program) {
    RoseProgram eod_program;

    for (const auto &e : out_edges_range(v, build.g)) {
        if (canEagerlyReportAtEod(build, e)) {
            RoseProgram block;
            makeRoleReports(build, bc, target(e, build.g), block);
            eod_program.add_block(move(block));
        }
    }

    if (eod_program.empty()) {
        return;
    }

    if (!onlyAtEod(build, v)) {
        // The rest of our program wasn't EOD anchored, so we need to guard
        // these reports with a check.
        const auto *end_inst = eod_program.end_instruction();
        eod_program.insert(begin(eod_program),
                           make_unique<RoseInstrCheckOnlyEod>(end_inst));
    }

    program.add_before_end(move(eod_program));
}

static
RoseProgram makeProgram(RoseBuildImpl &build, build_context &bc,
                        const RoseEdge &e) {
    const RoseGraph &g = build.g;
    auto v = target(e, g);

    RoseProgram program;

    // First, add program instructions that enforce preconditions without
    // effects.

    makeRoleAnchoredDelay(build, bc, v, program);

    if (onlyAtEod(build, v)) {
        DEBUG_PRINTF("only at eod\n");
        const auto *end_inst = program.end_instruction();
        program.add_before_end(make_unique<RoseInstrCheckOnlyEod>(end_inst));
    }

    if (g[e].history == ROSE_ROLE_HISTORY_ANCH) {
        makeRoleCheckBounds(build, v, e, program);
    }

    // This program may be triggered by different predecessors, with different
    // offset bounds. We must ensure we put this check/set operation after the
    // bounds check to deal with this case.
    if (in_degree(v, g) > 1) {
        makeRoleCheckNotHandled(bc, v, program);
    }

    makeRoleLookaround(build, bc, v, program);
    makeRoleCheckLeftfix(build, bc, v, program);

    // Next, we can add program instructions that have effects. This must be
    // done as a series of blocks, as some of them (like reports) are
    // escapable.

    RoseProgram effects_block;

    RoseProgram reports_block;
    makeRoleReports(build, bc, v, reports_block);
    effects_block.add_block(move(reports_block));

    RoseProgram infix_block;
    makeRoleInfixTriggers(build, bc, v, infix_block);
    effects_block.add_block(move(infix_block));

    // Note: SET_GROUPS instruction must be after infix triggers, as an infix
    // going dead may switch off groups.
    RoseProgram groups_block;
    makeRoleGroups(build, bc, v, groups_block);
    effects_block.add_block(move(groups_block));

    RoseProgram suffix_block;
    makeRoleSuffix(build, bc, v, suffix_block);
    effects_block.add_block(move(suffix_block));

    RoseProgram state_block;
    makeRoleSetState(bc, v, state_block);
    effects_block.add_block(move(state_block));

    // Note: EOD eager reports may generate a CHECK_ONLY_EOD instruction (if
    // the program doesn't have one already).
    RoseProgram eod_block;
    makeRoleEagerEodReports(build, bc, v, eod_block);
    effects_block.add_block(move(eod_block));

    program.add_before_end(move(effects_block));
    return program;
}

static
u32 writeBoundaryProgram(RoseBuildImpl &build, build_context &bc,
                         const set<ReportID> &reports) {
    if (reports.empty()) {
        return 0;
    }

    // Note: no CATCHUP instruction is necessary in the boundary case, as we
    // should always be caught up (and may not even have the resources in
    // scratch to support it).

    const bool has_som = false;
    RoseProgram program;
    for (const auto &id : reports) {
        makeReport(build, id, has_som, program);
    }
    applyFinalSpecialisation(program);
    return writeProgram(bc, move(program));
}

static
RoseBoundaryReports
makeBoundaryPrograms(RoseBuildImpl &build, build_context &bc,
                     const BoundaryReports &boundary,
                     const DerivedBoundaryReports &dboundary) {
    RoseBoundaryReports out;
    memset(&out, 0, sizeof(out));

    DEBUG_PRINTF("report ^:  %zu\n", boundary.report_at_0.size());
    DEBUG_PRINTF("report $:  %zu\n", boundary.report_at_eod.size());
    DEBUG_PRINTF("report ^$: %zu\n", dboundary.report_at_0_eod_full.size());

    out.reportEodOffset =
        writeBoundaryProgram(build, bc, boundary.report_at_eod);
    out.reportZeroOffset =
        writeBoundaryProgram(build, bc, boundary.report_at_0);
    out.reportZeroEodOffset =
        writeBoundaryProgram(build, bc, dboundary.report_at_0_eod_full);

    return out;
}

static
void assignStateIndices(const RoseBuildImpl &build, build_context &bc) {
    const auto &g = build.g;

    u32 state = 0;

    for (auto v : vertices_range(g)) {
        // Virtual vertices (starts, EOD accept vertices) never need state
        // indices.
        if (build.isVirtualVertex(v)) {
            continue;
        }

        // We only need a state index if we have successors that are not
        // eagerly-reported EOD vertices.
        bool needs_state_index = false;
        for (const auto &e : out_edges_range(v, g)) {
            if (!canEagerlyReportAtEod(build, e)) {
                needs_state_index = true;
                break;
            }
        }

        if (!needs_state_index) {
            continue;
        }

        /* TODO: also don't need a state index if all edges are nfa based */
        bc.roleStateIndices.emplace(v, state++);
    }

    DEBUG_PRINTF("assigned %u states (from %zu vertices)\n", state,
                 num_vertices(g));
    bc.numStates = state;
}

static
bool hasUsefulStops(const left_build_info &build) {
    for (u32 i = 0; i < N_CHARS; i++) {
        if (build.stopAlphabet[i]) {
            return true;
        }
    }
    return false;
}

static
void buildLeftInfoTable(const RoseBuildImpl &tbi, build_context &bc,
                        const set<u32> &eager_queues,
                        u32 leftfixBeginQueue, u32 leftfixCount,
                        vector<LeftNfaInfo> &leftTable, u32 *laggedRoseCount,
                        size_t *history) {
    const RoseGraph &g = tbi.g;
    const CompileContext &cc = tbi.cc;

    ue2::unordered_set<u32> done_core;

    leftTable.resize(leftfixCount);

    u32 lagIndex = 0;

    for (RoseVertex v : vertices_range(g)) {
        if (!g[v].left) {
            continue;
        }
        assert(contains(bc.leftfix_info, v));
        const left_build_info &lbi = bc.leftfix_info.at(v);
        if (lbi.has_lookaround) {
            continue;
        }

        assert(lbi.queue >= leftfixBeginQueue);
        u32 left_index = lbi.queue - leftfixBeginQueue;
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
                left.stopTable = bc.engine_blob.add(lbi.stopAlphabet.begin(),
                                                    lbi.stopAlphabet.end());
            }

            assert(lbi.countingMiracleOffset || !lbi.countingMiracleCount);
            left.countingMiracleOffset = lbi.countingMiracleOffset;

            DEBUG_PRINTF("mw = %u\n", lbi.transient);
            left.transient = verify_u8(lbi.transient);
            left.infix = tbi.isNonRootSuccessor(v);
            left.eager = contains(eager_queues, lbi.queue);

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

static
void addPredBlockSingle(u32 pred_state, RoseProgram &pred_block,
                        RoseProgram &program) {
    // Prepend an instruction to check the pred state is on.
    const auto *end_inst = pred_block.end_instruction();
    pred_block.insert(begin(pred_block),
                      make_unique<RoseInstrCheckState>(pred_state, end_inst));
    program.add_block(move(pred_block));
}

static
void addPredBlocksAny(build_context &bc, map<u32, RoseProgram> &pred_blocks,
                      RoseProgram &program) {
    RoseProgram sparse_program;

    vector<u32> keys;
    for (const u32 &key : pred_blocks | map_keys) {
        keys.push_back(key);
    }

    const RoseInstruction *end_inst = sparse_program.end_instruction();
    auto ri = make_unique<RoseInstrSparseIterAny>(bc.numStates, keys, end_inst);
    sparse_program.add_before_end(move(ri));

    RoseProgram &block = pred_blocks.begin()->second;
    sparse_program.add_before_end(move(block));
    program.add_block(move(sparse_program));
}

static
void addPredBlocksMulti(build_context &bc, map<u32, RoseProgram> &pred_blocks,
                        RoseProgram &program) {
    assert(!pred_blocks.empty());

    RoseProgram sparse_program;
    const RoseInstruction *end_inst = sparse_program.end_instruction();
    vector<pair<u32, const RoseInstruction *>> jump_table;

    // BEGIN instruction.
    auto ri_begin =
        make_unique<RoseInstrSparseIterBegin>(bc.numStates, end_inst);
    RoseInstrSparseIterBegin *begin_inst = ri_begin.get();
    sparse_program.add_before_end(move(ri_begin));

    // NEXT instructions, one per pred program.
    u32 prev_key = pred_blocks.begin()->first;
    for (auto it = next(begin(pred_blocks)); it != end(pred_blocks); ++it) {
        auto ri = make_unique<RoseInstrSparseIterNext>(prev_key, begin_inst,
                                                       end_inst);
        sparse_program.add_before_end(move(ri));
        prev_key = it->first;
    }

    // Splice in each pred program after its BEGIN/NEXT.
    auto out_it = begin(sparse_program);
    for (auto &m : pred_blocks) {
        u32 key = m.first;
        RoseProgram &flat_prog = m.second;
        assert(!flat_prog.empty());
        const size_t block_len = flat_prog.size() - 1; // without INSTR_END.

        assert(dynamic_cast<const RoseInstrSparseIterBegin *>(out_it->get()) ||
               dynamic_cast<const RoseInstrSparseIterNext *>(out_it->get()));
        out_it = sparse_program.insert(++out_it, move(flat_prog));

        // Jump table target for this key is the beginning of the block we just
        // spliced in.
        jump_table.emplace_back(key, out_it->get());

        assert(distance(begin(sparse_program), out_it) + block_len <=
               sparse_program.size());
        advance(out_it, block_len);
    }

    // Write the jump table back into the SPARSE_ITER_BEGIN instruction.
    begin_inst->jump_table = move(jump_table);

    program.add_block(move(sparse_program));
}

static
void addPredBlocks(build_context &bc, map<u32, RoseProgram> &pred_blocks,
                   RoseProgram &program) {
    // Trim empty blocks, if any exist.
    for (auto it = pred_blocks.begin(); it != pred_blocks.end();) {
        if (it->second.empty()) {
            it = pred_blocks.erase(it);
        } else {
            ++it;
        }
    }

    const size_t num_preds = pred_blocks.size();
    if (num_preds == 0) {
        return;
    }

    if (num_preds == 1) {
        const auto head = pred_blocks.begin();
        addPredBlockSingle(head->first, head->second, program);
        return;
    }

    // First, see if all our blocks are equivalent, in which case we can
    // collapse them down into one.
    const auto &blocks = pred_blocks | map_values;
    if (all_of(begin(blocks), end(blocks), [&](const RoseProgram &block) {
            return RoseProgramEquivalence()(*begin(blocks), block);
        })) {
        DEBUG_PRINTF("all blocks equiv\n");
        addPredBlocksAny(bc, pred_blocks, program);
        return;
    }

    addPredBlocksMulti(bc, pred_blocks, program);
}

static
void makePushDelayedInstructions(const RoseBuildImpl &build,
                                 const flat_set<u32> &lit_ids,
                                 RoseProgram &program) {
    assert(!lit_ids.empty());
    const auto &arb_lit_info = build.literal_info.at(*lit_ids.begin());
    if (arb_lit_info.delayed_ids.empty()) {
        return;
    }

    for (const auto &int_id : arb_lit_info.delayed_ids) {
        const auto &child_literal = build.literals.right.at(int_id);
        u32 child_id = build.literal_info[int_id].final_id;
        u32 delay_index = child_id - build.delay_base_id;

        DEBUG_PRINTF("delay=%u child_id=%u\n", child_literal.delay, child_id);

        auto ri = make_unique<RoseInstrPushDelayed>(
            verify_u8(child_literal.delay), delay_index);
        program.add_before_end(move(ri));
    }
}

static
rose_group getLitGroupsUnion(const RoseBuildImpl &build,
                             const flat_set<u32> &lit_ids) {
    rose_group groups = 0;
    for (auto lit_id : lit_ids) {
        const auto &info = build.literal_info.at(lit_id);
        groups |= info.group_mask;
    }
    return groups;
}

static
void makeGroupCheckInstruction(const RoseBuildImpl &build,
                               const flat_set<u32> &lit_ids,
                               RoseProgram &program) {
    rose_group groups = getLitGroupsUnion(build, lit_ids);
    if (!groups) {
        return;
    }
    program.add_before_end(make_unique<RoseInstrCheckGroups>(groups));
}

static
void makeCheckLitMaskInstruction(const RoseBuildImpl &build, build_context &bc,
                                 const flat_set<u32> &lit_ids,
                                 RoseProgram &program) {
    const auto &lit_info = build.literal_info.at(*lit_ids.begin());
    if (!lit_info.requires_benefits) {
        return;
    }

    vector<LookEntry> look;

    assert(lit_ids.size() == 1);
    u32 lit_id = *lit_ids.begin();
    const ue2_literal &s = build.literals.right.at(lit_id).s;
    DEBUG_PRINTF("building mask for lit %u: %s\n", lit_id,
                 dumpString(s).c_str());
    assert(s.length() <= MAX_MASK2_WIDTH);
    s32 i = 0 - s.length();
    for (const auto &e : s) {
        if (!e.nocase) {
            look.emplace_back(verify_s8(i), e);
        }
        i++;
    }

    assert(!look.empty());
    makeLookaroundInstruction(bc, look, program);
}

static
void makeGroupSquashInstruction(const RoseBuildImpl &build,
                                const flat_set<u32> &lit_ids,
                                RoseProgram &program) {
    assert(!lit_ids.empty());
    const u32 lit_id = *lit_ids.begin();
    const auto &info = build.literal_info[lit_id];
    if (!info.squash_group) {
        return;
    }

    rose_group groups = getLitGroupsUnion(build, lit_ids);
    if (!groups) {
        return;
    }

    DEBUG_PRINTF("squashes 0x%llx\n", groups);
    program.add_before_end(
        make_unique<RoseInstrSquashGroups>(~groups)); // Note negated.
}

static
u32 findMaxOffset(const RoseBuildImpl &build, u32 lit_id) {
    const auto &lit_vertices = build.literal_info.at(lit_id).vertices;
    assert(!lit_vertices.empty());

    u32 max_offset = 0;
    for (const auto &v : lit_vertices) {
        max_offset = max(max_offset, build.g[v].max_offset);
    }

    return max_offset;
}

static
void makeRecordAnchoredInstruction(const RoseBuildImpl &build,
                                   build_context &bc, u32 final_id,
                                   RoseProgram &program) {
    assert(contains(bc.final_id_to_literal, final_id));
    const auto &lit_ids = bc.final_id_to_literal.at(final_id);

    // Must be anchored.
    assert(!lit_ids.empty());
    if (build.literals.right.at(*begin(lit_ids)).table != ROSE_ANCHORED) {
        return;
    }

    // If this anchored literal can never match past
    // floatingMinLiteralMatchOffset, we will never have to record it.
    u32 max_offset = 0;
    for (u32 lit_id : lit_ids) {
        assert(build.literals.right.at(lit_id).table == ROSE_ANCHORED);
        max_offset = max(max_offset, findMaxOffset(build, lit_id));
    }

    if (max_offset <= bc.floatingMinLiteralMatchOffset) {
        return;
    }

    program.add_before_end(make_unique<RoseInstrRecordAnchored>(final_id));
}

static
u32 findMinOffset(const RoseBuildImpl &build, u32 lit_id) {
    const auto &lit_vertices = build.literal_info.at(lit_id).vertices;
    assert(!lit_vertices.empty());

    u32 min_offset = UINT32_MAX;
    for (const auto &v : lit_vertices) {
        min_offset = min(min_offset, build.g[v].min_offset);
    }

    return min_offset;
}

static
void makeCheckLitEarlyInstruction(const RoseBuildImpl &build, build_context &bc,
                                  const flat_set<u32> &lit_ids,
                                  const vector<RoseEdge> &lit_edges,
                                  RoseProgram &program) {
    if (lit_edges.empty()) {
        return;
    }

    if (bc.floatingMinLiteralMatchOffset == 0) {
        return;
    }

    RoseVertex v = target(lit_edges.front(), build.g);
    if (!build.isFloating(v)) {
        return;
    }

    if (lit_ids.empty()) {
        return;
    }

    size_t min_len = SIZE_MAX;
    u32 min_offset = UINT32_MAX;
    for (u32 lit_id : lit_ids) {
        const auto &lit = build.literals.right.at(lit_id);
        size_t lit_min_len = lit.elength();
        u32 lit_min_offset = findMinOffset(build, lit_id);
        DEBUG_PRINTF("lit_id=%u has min_len=%zu, min_offset=%u\n", lit_id,
                     lit_min_len, lit_min_offset);
        min_len = min(min_len, lit_min_len);
        min_offset = min(min_offset, lit_min_offset);
    }

    DEBUG_PRINTF("has min_len=%zu, min_offset=%u, "
                 "global min is %u\n", min_len, min_offset,
                 bc.floatingMinLiteralMatchOffset);

    // If we can't match before the min offset, we don't need the check.
    if (min_len >= bc.floatingMinLiteralMatchOffset) {
        DEBUG_PRINTF("no need for check, min is %u\n",
                     bc.floatingMinLiteralMatchOffset);
        return;
    }

    assert(min_offset >= bc.floatingMinLiteralMatchOffset);
    assert(min_offset < UINT32_MAX);

    DEBUG_PRINTF("adding lit early check, min_offset=%u\n", min_offset);
    const auto *end_inst = program.end_instruction();
    program.add_before_end(
        make_unique<RoseInstrCheckLitEarly>(min_offset, end_inst));
}

static
void makeCheckLiteralInstruction(const RoseBuildImpl &build,
                                 const build_context &bc,
                                 const flat_set<u32> &lits,
                                 RoseProgram &program) {
    assert(bc.longLitLengthThreshold > 0);

    DEBUG_PRINTF("lits [%s], long lit threshold %zu\n",
                 as_string_list(lits).c_str(), bc.longLitLengthThreshold);

    if (lits.size() != 1) {
        // final_id sharing is only allowed for literals that are short enough
        // to not require any additional confirm work.
        assert(all_of(begin(lits), end(lits), [&](u32 lit_id) {
            const rose_literal_id &lit = build.literals.right.at(lit_id);
            return lit.s.length() <= ROSE_SHORT_LITERAL_LEN_MAX;
        }));
        return;
    }

    u32 lit_id = *lits.begin();
    if (build.isDelayed(lit_id)) {
        return;
    }

    const rose_literal_id &lit = build.literals.right.at(lit_id);

    if (lit.s.length() <= ROSE_SHORT_LITERAL_LEN_MAX) {
        DEBUG_PRINTF("lit short enough to not need confirm\n");
        return;
    }

    // Check resource limits as well.
    if (lit.s.length() > build.cc.grey.limitLiteralLength) {
        throw ResourceLimitError();
    }

    if (lit.s.length() <= bc.longLitLengthThreshold) {
        DEBUG_PRINTF("is a medium-length literal\n");
        const auto *end_inst = program.end_instruction();
        unique_ptr<RoseInstruction> ri;
        if (lit.s.any_nocase()) {
            ri = make_unique<RoseInstrCheckMedLitNocase>(lit.s.get_string(),
                                                         end_inst);
        } else {
            ri = make_unique<RoseInstrCheckMedLit>(lit.s.get_string(),
                                                   end_inst);
        }
        program.add_before_end(move(ri));
        return;
    }

    // Long literal support should only really be used for the floating table
    // in streaming mode.
    assert(lit.table == ROSE_FLOATING && build.cc.streaming);

    DEBUG_PRINTF("is a long literal\n");

    const auto *end_inst = program.end_instruction();
    unique_ptr<RoseInstruction> ri;
    if (lit.s.any_nocase()) {
        ri = make_unique<RoseInstrCheckLongLitNocase>(lit.s.get_string(),
                                                      end_inst);
    } else {
        ri = make_unique<RoseInstrCheckLongLit>(lit.s.get_string(), end_inst);
    }
    program.add_before_end(move(ri));
}

static
bool hasDelayedLiteral(RoseBuildImpl &build,
                       const vector<RoseEdge> &lit_edges) {
    auto is_delayed = bind(&RoseBuildImpl::isDelayed, &build, _1);
    for (const auto &e : lit_edges) {
        auto v = target(e, build.g);
        const auto &lits = build.g[v].literals;
        if (any_of(begin(lits), end(lits), is_delayed)) {
            return true;
        }
    }
    return false;
}

static
RoseProgram buildLitInitialProgram(RoseBuildImpl &build, build_context &bc,
                                   u32 final_id,
                                   const vector<RoseEdge> &lit_edges) {
    RoseProgram program;

    // No initial program for EOD.
    if (final_id == MO_INVALID_IDX) {
        return program;
    }

    DEBUG_PRINTF("final_id %u\n", final_id);

    const auto &lit_ids = bc.final_id_to_literal.at(final_id);

    // Check long literal info.
    makeCheckLiteralInstruction(build, bc, lit_ids, program);

    // Check lit mask.
    makeCheckLitMaskInstruction(build, bc, lit_ids, program);

    // Check literal groups. This is an optimisation that we only perform for
    // delayed literals, as their groups may be switched off; ordinarily, we
    // can trust the HWLM matcher.
    if (hasDelayedLiteral(build, lit_edges)) {
        makeGroupCheckInstruction(build, lit_ids, program);
    }

    // Add instructions for pushing delayed matches, if there are any.
    makePushDelayedInstructions(build, lit_ids, program);

    // Add pre-check for early literals in the floating table.
    makeCheckLitEarlyInstruction(build, bc, lit_ids, lit_edges, program);

    return program;
}

static
RoseProgram buildLiteralProgram(RoseBuildImpl &build, build_context &bc,
                                u32 final_id,
                                const vector<RoseEdge> &lit_edges) {
    const auto &g = build.g;

    DEBUG_PRINTF("final id %u, %zu lit edges\n", final_id, lit_edges.size());

    RoseProgram program;

    // Predecessor state id -> program block.
    map<u32, RoseProgram> pred_blocks;

    // Construct sparse iter sub-programs.
    for (const auto &e : lit_edges) {
        const auto &u = source(e, g);
        if (build.isAnyStart(u)) {
            continue; // Root roles are not handled with sparse iterator.
        }
        DEBUG_PRINTF("sparse iter edge (%zu,%zu)\n", g[u].index,
                     g[target(e, g)].index);
        assert(contains(bc.roleStateIndices, u));
        u32 pred_state = bc.roleStateIndices.at(u);
        pred_blocks[pred_state].add_block(makeProgram(build, bc, e));
    }

    // Add blocks to deal with non-root edges (triggered by sparse iterator or
    // mmbit_isset checks).
    addPredBlocks(bc, pred_blocks, program);

    // Add blocks to handle root roles.
    for (const auto &e : lit_edges) {
        const auto &u = source(e, g);
        if (!build.isAnyStart(u)) {
            continue;
        }
        DEBUG_PRINTF("root edge (%zu,%zu)\n", g[u].index,
                     g[target(e, g)].index);
        program.add_block(makeProgram(build, bc, e));
    }

    if (final_id != MO_INVALID_IDX) {
        const auto &lit_ids = bc.final_id_to_literal.at(final_id);
        RoseProgram root_block;

        // Literal may squash groups.
        makeGroupSquashInstruction(build, lit_ids, root_block);

        // Literal may be anchored and need to be recorded.
        makeRecordAnchoredInstruction(build, bc, final_id, root_block);

        program.add_block(move(root_block));
    }

    // Construct initial program up front, as its early checks must be able to
    // jump to end and terminate processing for this literal.
    auto lit_program = buildLitInitialProgram(build, bc, final_id, lit_edges);
    lit_program.add_before_end(move(program));
    return lit_program;
}

static
RoseProgram buildLiteralProgram(RoseBuildImpl &build, build_context &bc,
                                const flat_set<u32> &final_ids,
                                const map<u32, vector<RoseEdge>> &lit_edges) {
    assert(!final_ids.empty());

    DEBUG_PRINTF("entry, %zu final ids\n", final_ids.size());
    const vector<RoseEdge> no_edges;

    RoseProgram program;
    for (const auto &final_id : final_ids) {
        const auto *edges_ptr = &no_edges;
        if (contains(lit_edges, final_id)) {
            edges_ptr = &(lit_edges.at(final_id));
        }
        auto prog = buildLiteralProgram(build, bc, final_id, *edges_ptr);
        DEBUG_PRINTF("final_id=%u, prog has %zu entries\n", final_id,
                     prog.size());
        program.add_block(move(prog));
    }
    return program;
}

static
u32 writeLiteralProgram(RoseBuildImpl &build, build_context &bc,
                        const flat_set<u32> &final_ids,
                        const map<u32, vector<RoseEdge>> &lit_edges) {
    RoseProgram program = buildLiteralProgram(build, bc, final_ids, lit_edges);
    if (program.empty()) {
        return 0;
    }
    applyFinalSpecialisation(program);
    return writeProgram(bc, move(program));
}

static
u32 buildDelayRebuildProgram(RoseBuildImpl &build, build_context &bc,
                             const flat_set<u32> &final_ids) {
    if (!build.cc.streaming) {
        return 0; // We only do delayed rebuild in streaming mode.
    }

    RoseProgram program;

    for (const auto &final_id : final_ids) {
        const auto &lit_ids = bc.final_id_to_literal.at(final_id);
        assert(!lit_ids.empty());

        const auto &arb_lit_info = build.literal_info.at(*lit_ids.begin());
        if (arb_lit_info.delayed_ids.empty()) {
            continue; // No delayed IDs, no work to do.
        }

        RoseProgram prog;
        makeCheckLiteralInstruction(build, bc, lit_ids, prog);
        makeCheckLitMaskInstruction(build, bc, lit_ids, prog);
        makePushDelayedInstructions(build, lit_ids, prog);
        program.add_block(move(prog));
    }

    if (program.empty()) {
        return 0;
    }
    applyFinalSpecialisation(program);
    return writeProgram(bc, move(program));
}

static
map<u32, vector<RoseEdge>> findEdgesByLiteral(const RoseBuildImpl &build) {
    // Use a set of edges while building the map to cull duplicates.
    map<u32, flat_set<RoseEdge>> unique_lit_edge_map;

    const auto &g = build.g;
    for (const auto &e : edges_range(g)) {
        const auto &v = target(e, g);
        for (const auto &lit_id : g[v].literals) {
            assert(lit_id < build.literal_info.size());
            u32 final_id = build.literal_info.at(lit_id).final_id;
            if (final_id == MO_INVALID_IDX) {
                // Unused, special report IDs are handled elsewhere.
                continue;
            }
            unique_lit_edge_map[final_id].insert(e);
        }
    }

    // Build output map, sorting edges by (source, target) vertex index.
    map<u32, vector<RoseEdge>> lit_edge_map;
    for (const auto &m : unique_lit_edge_map) {
        auto edge_list = vector<RoseEdge>(begin(m.second), end(m.second));
        sort(begin(edge_list), end(edge_list),
             [&g](const RoseEdge &a, const RoseEdge &b) {
                 return tie(g[source(a, g)].index, g[target(a, g)].index) <
                        tie(g[source(b, g)].index, g[target(b, g)].index);
             });
        lit_edge_map.emplace(m.first, edge_list);
    }

    return lit_edge_map;
}

static
rose_literal_id getFragment(const rose_literal_id &lit) {
    if (lit.s.length() <= ROSE_SHORT_LITERAL_LEN_MAX) {
        DEBUG_PRINTF("whole lit is frag\n");
        return lit;
    }

    rose_literal_id frag = lit;
    frag.s = frag.s.substr(frag.s.length() - ROSE_SHORT_LITERAL_LEN_MAX);

    DEBUG_PRINTF("fragment: %s\n", dumpString(frag.s).c_str());
    return frag;
}

static
rose_group getGroups(const RoseBuildImpl &build, const flat_set<u32> &lit_ids) {
    rose_group groups = 0;
    for (auto lit_id : lit_ids) {
        auto &info = build.literal_info.at(lit_id);
        groups |= info.group_mask;
    }
    return groups;
}

static
void groupByFragment(RoseBuildImpl &build, const build_context &bc) {
    u32 frag_id = 0;

    struct FragmentInfo {
        vector<u32> final_ids;
        rose_group groups = 0;
    };

    map<rose_literal_id, FragmentInfo> frag_info;

    auto &final_to_frag = build.final_to_frag_map;
    auto &fragments = build.fragments;

    for (const auto &m : bc.final_id_to_literal) {
        u32 final_id = m.first;
        const auto &lit_ids = m.second;
        assert(!lit_ids.empty());

        auto groups = getGroups(build, lit_ids);

        if (lit_ids.size() > 1) {
            final_to_frag.emplace(final_id, frag_id);
            fragments.emplace_back(frag_id, groups);
            frag_id++;
            continue;
        }

        const auto lit_id = *lit_ids.begin();
        const auto &lit = build.literals.right.at(lit_id);
        if (lit.s.length() < ROSE_SHORT_LITERAL_LEN_MAX) {
            final_to_frag.emplace(final_id, frag_id);
            fragments.emplace_back(frag_id, groups);
            frag_id++;
            continue;
        }

        // Combining fragments that squash their groups is unsafe.
        const auto &info = build.literal_info[lit_id];
        if (info.squash_group) {
            final_to_frag.emplace(final_id, frag_id);
            fragments.emplace_back(frag_id, groups);
            frag_id++;
            continue;
        }

        DEBUG_PRINTF("fragment candidate: final_id=%u %s\n", final_id,
                     dumpString(lit.s).c_str());
        auto &fi = frag_info[getFragment(lit)];
        fi.final_ids.push_back(final_id);
        fi.groups |= groups;
    }

    for (const auto &m : frag_info) {
        const auto &fi = m.second;
        DEBUG_PRINTF("frag %s -> ids: %s\n", dumpString(m.first.s).c_str(),
                     as_string_list(fi.final_ids).c_str());
        fragments.emplace_back(frag_id, fi.groups);
        for (const auto final_id : fi.final_ids) {
            assert(!contains(final_to_frag, final_id));
            final_to_frag.emplace(final_id, frag_id);
        }
        frag_id++;
    }
}

/**
 * \brief Build the interpreter programs for each literal.
 */
static
void buildLiteralPrograms(RoseBuildImpl &build, build_context &bc) {
    // Build a reverse mapping from fragment -> final_id.
    map<u32, flat_set<u32>> frag_to_final_map;
    for (const auto &m : build.final_to_frag_map) {
        frag_to_final_map[m.second].insert(m.first);
    }

    DEBUG_PRINTF("%zu fragments\n", build.fragments.size());
    auto lit_edge_map = findEdgesByLiteral(build);

    for (auto &frag : build.fragments) {
        const auto &final_ids = frag_to_final_map[frag.fragment_id];
        DEBUG_PRINTF("frag_id=%u, final_ids=[%s]\n", frag.fragment_id,
                     as_string_list(final_ids).c_str());
        frag.lit_program_offset =
            writeLiteralProgram(build, bc, final_ids, lit_edge_map);
        frag.delay_program_offset =
            buildDelayRebuildProgram(build, bc, final_ids);
    }
}

static
u32 buildDelayPrograms(RoseBuildImpl &build, build_context &bc) {
    auto lit_edge_map = findEdgesByLiteral(build);

    vector<u32> programs;

    for (u32 final_id = build.delay_base_id;
         final_id < bc.final_id_to_literal.size(); final_id++) {
        u32 offset = writeLiteralProgram(build, bc, {final_id}, lit_edge_map);
        programs.push_back(offset);
    }

    DEBUG_PRINTF("%zu delay programs\n", programs.size());
    return bc.engine_blob.add(begin(programs), end(programs));
}

static
u32 buildAnchoredPrograms(RoseBuildImpl &build, build_context &bc) {
    auto lit_edge_map = findEdgesByLiteral(build);

    vector<u32> programs;

    for (u32 final_id = build.anchored_base_id;
         final_id < build.delay_base_id; final_id++) {
        u32 offset = writeLiteralProgram(build, bc, {final_id}, lit_edge_map);
        programs.push_back(offset);
    }

    DEBUG_PRINTF("%zu anchored programs\n", programs.size());
    return bc.engine_blob.add(begin(programs), end(programs));
}

/**
 * \brief Returns all reports used by output-exposed engines, for which we need
 * to generate programs.
 */
static
set<ReportID> findEngineReports(const RoseBuildImpl &build) {
    set<ReportID> reports;

    // The small write engine uses these engine report programs.
    insert(&reports, build.smwr.all_reports());

    for (const auto &outfix : build.outfixes) {
        insert(&reports, all_reports(outfix));
    }

    const auto &g = build.g;
    for (auto v : vertices_range(g)) {
        if (g[v].suffix) {
            insert(&reports, all_reports(g[v].suffix));
        }
    }

    DEBUG_PRINTF("%zu engine reports (of %zu)\n", reports.size(),
                 build.rm.numReports());
    return reports;
}

static
pair<u32, u32> buildReportPrograms(RoseBuildImpl &build, build_context &bc) {
    const auto reports = findEngineReports(build);
    vector<u32> programs;
    programs.reserve(reports.size());

    for (ReportID id : reports) {
        RoseProgram program;
        const bool has_som = false;
        makeCatchupMpv(build, bc, id, program);
        makeReport(build, id, has_som, program);
        applyFinalSpecialisation(program);
        u32 offset = writeProgram(bc, move(program));
        programs.push_back(offset);
        build.rm.setProgramOffset(id, offset);
        DEBUG_PRINTF("program for report %u @ %u (%zu instructions)\n", id,
                     programs.back(), program.size());
    }

    u32 offset = bc.engine_blob.add(begin(programs), end(programs));
    u32 count = verify_u32(programs.size());
    return {offset, count};
}

static
RoseProgram makeEodAnchorProgram(RoseBuildImpl &build, build_context &bc,
                                 const RoseEdge &e, const bool multiple_preds) {
    const RoseGraph &g = build.g;
    const RoseVertex v = target(e, g);

    RoseProgram program;

    if (g[e].history == ROSE_ROLE_HISTORY_ANCH) {
        makeRoleCheckBounds(build, v, e, program);
    }

    if (multiple_preds) {
        // Only necessary when there is more than one pred.
        makeRoleCheckNotHandled(bc, v, program);
    }

    const auto &reports = g[v].reports;
    makeCatchup(build, bc, reports, program);

    const bool has_som = false;
    RoseProgram report_block;
    for (const auto &id : reports) {
        makeReport(build, id, has_som, report_block);
    }
    program.add_before_end(move(report_block));

    return program;
}

static
bool hasEodAnchoredSuffix(const RoseBuildImpl &build) {
    const RoseGraph &g = build.g;
    for (auto v : vertices_range(g)) {
        if (g[v].suffix && build.isInETable(v)) {
            DEBUG_PRINTF("vertex %zu is in eod table and has a suffix\n",
                         g[v].index);
            return true;
        }
    }
    return false;
}

static
bool hasEodMatcher(const RoseBuildImpl &build) {
    const RoseGraph &g = build.g;
    for (auto v : vertices_range(g)) {
        if (build.isInETable(v)) {
            DEBUG_PRINTF("vertex %zu is in eod table\n", g[v].index);
            return true;
        }
    }
    return false;
}

static
void addEodAnchorProgram(RoseBuildImpl &build, build_context &bc,
                         bool in_etable, RoseProgram &program) {
    const RoseGraph &g = build.g;

    // Predecessor state id -> program block.
    map<u32, RoseProgram> pred_blocks;

    for (auto v : vertices_range(g)) {
        if (!g[v].eod_accept) {
            continue;
        }

        DEBUG_PRINTF("vertex %zu (with %zu preds) fires on EOD\n", g[v].index,
                     in_degree(v, g));

        vector<RoseEdge> edge_list;
        for (const auto &e : in_edges_range(v, g)) {
            RoseVertex u = source(e, g);
            if (build.isInETable(u) != in_etable) {
                DEBUG_PRINTF("pred %zu %s in etable\n", g[u].index,
                             in_etable ? "is not" : "is");
                continue;
            }
            if (canEagerlyReportAtEod(build, e)) {
                DEBUG_PRINTF("already done report for vertex %zu\n", g[u].index);
                continue;
            }
            edge_list.push_back(e);
        }

        const bool multiple_preds = edge_list.size() > 1;
        for (const auto &e : edge_list) {
            RoseVertex u = source(e, g);
            assert(contains(bc.roleStateIndices, u));
            u32 pred_state = bc.roleStateIndices.at(u);
            pred_blocks[pred_state].add_block(
                makeEodAnchorProgram(build, bc, e, multiple_preds));
        }
    }

    addPredBlocks(bc, pred_blocks, program);
}

static
void addEodEventProgram(RoseBuildImpl &build, build_context &bc,
                        RoseProgram &program) {
    if (build.eod_event_literal_id == MO_INVALID_IDX) {
        return;
    }

    const RoseGraph &g = build.g;
    const auto &lit_info = build.literal_info.at(build.eod_event_literal_id);
    assert(lit_info.delayed_ids.empty());
    assert(!lit_info.squash_group);
    assert(!lit_info.requires_benefits);

    // Collect all edges leading into EOD event literal vertices.
    vector<RoseEdge> edge_list;
    for (const auto &v : lit_info.vertices) {
        for (const auto &e : in_edges_range(v, g)) {
            edge_list.push_back(e);
        }
    }

    // Sort edge list for determinism, prettiness.
    sort(begin(edge_list), end(edge_list),
         [&g](const RoseEdge &a, const RoseEdge &b) {
             return tie(g[source(a, g)].index, g[target(a, g)].index) <
                    tie(g[source(b, g)].index, g[target(b, g)].index);
         });

    program.add_block(
        buildLiteralProgram(build, bc, MO_INVALID_IDX, edge_list));
}

static
void addEnginesEodProgram(u32 eodNfaIterOffset, RoseProgram &program) {
    if (!eodNfaIterOffset) {
        return;
    }

    RoseProgram block;
    block.add_before_end(make_unique<RoseInstrEnginesEod>(eodNfaIterOffset));
    program.add_block(move(block));
}

static
void addSuffixesEodProgram(const RoseBuildImpl &build, RoseProgram &program) {
    if (!hasEodAnchoredSuffix(build)) {
        return;
    }

    RoseProgram block;
    block.add_before_end(make_unique<RoseInstrSuffixesEod>());
    program.add_block(move(block));
}

static
void addMatcherEodProgram(const RoseBuildImpl &build, RoseProgram &program) {
    if (!hasEodMatcher(build)) {
        return;
    }

    RoseProgram block;
    block.add_before_end(make_unique<RoseInstrMatcherEod>());
    program.add_block(move(block));
}

static
u32 writeEodProgram(RoseBuildImpl &build, build_context &bc,
                    u32 eodNfaIterOffset) {
    RoseProgram program;

    addEodEventProgram(build, bc, program);
    addEnginesEodProgram(eodNfaIterOffset, program);
    addEodAnchorProgram(build, bc, false, program);
    addMatcherEodProgram(build, program);
    addEodAnchorProgram(build, bc, true, program);
    addSuffixesEodProgram(build, program);

    if (program.empty()) {
        return 0;
    }

    applyFinalSpecialisation(program);
    return writeProgram(bc, move(program));
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

            DEBUG_PRINTF("checking %u: elen %zu min/max %u/%u\n", lit_id,
                         key.elength_including_mask(), min_d, max_d);

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
                if (min_d >= key.elength_including_mask()) {
                    LIMIT_TO_AT_MOST(&engine->floatingMinDistance,
                                     min_d - (u32)key.elength_including_mask());
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
}

static
u32 buildEagerQueueIter(const set<u32> &eager, u32 leftfixBeginQueue,
                        u32 queue_count,
                        build_context &bc) {
    if (eager.empty()) {
        return 0;
    }

    vector<u32> vec;
    for (u32 q : eager) {
        assert(q >= leftfixBeginQueue);
        vec.push_back(q - leftfixBeginQueue);
    }

    vector<mmbit_sparse_iter> iter;
    mmbBuildSparseIterator(iter, vec, queue_count - leftfixBeginQueue);
    return bc.engine_blob.add_iterator(iter);
}

static
void allocateFinalIdToSet(RoseBuildImpl &build, build_context &bc,
                          const set<u32> &lits, u32 *next_final_id) {
    const auto &g = build.g;
    auto &literal_info = build.literal_info;
    auto &final_id_to_literal = bc.final_id_to_literal;

    /* We can allocate the same final id to multiple literals of the same type
     * if they share the same vertex set and trigger the same delayed literal
     * ids and squash the same roles and have the same group squashing
     * behaviour. Benefits literals cannot be merged. */

    for (u32 int_id : lits) {
        rose_literal_info &curr_info = literal_info[int_id];
        const rose_literal_id &lit = build.literals.right.at(int_id);
        const auto &verts = curr_info.vertices;

        // Literals with benefits cannot be merged.
        if (curr_info.requires_benefits) {
            DEBUG_PRINTF("id %u has benefits\n", int_id);
            goto assign_new_id;
        }

        // Literals that need confirmation with CHECK_LONG_LIT or CHECK_MED_LIT
        // cannot be merged.
        if (lit.s.length() > ROSE_SHORT_LITERAL_LEN_MAX) {
            DEBUG_PRINTF("id %u needs lit confirm\n", int_id);
            goto assign_new_id;
        }

        if (!verts.empty() && curr_info.delayed_ids.empty()) {
            vector<u32> cand;
            insert(&cand, cand.end(), g[*verts.begin()].literals);
            for (auto v : verts) {
                vector<u32> temp;
                set_intersection(cand.begin(), cand.end(),
                                 g[v].literals.begin(),
                                 g[v].literals.end(),
                                 inserter(temp, temp.end()));
                cand.swap(temp);
            }

            for (u32 cand_id : cand) {
                if (cand_id >= int_id) {
                    break;
                }

                const auto &cand_info = literal_info[cand_id];
                const auto &cand_lit = build.literals.right.at(cand_id);

                if (cand_lit.s.length() > ROSE_SHORT_LITERAL_LEN_MAX) {
                    continue;
                }

                if (cand_info.requires_benefits) {
                    continue;
                }

                if (!cand_info.delayed_ids.empty()) {
                    /* TODO: allow cases where delayed ids are equivalent.
                     * This is awkward currently as the have not had their
                     * final ids allocated yet */
                    continue;
                }

                if (lits.find(cand_id) == lits.end()
                    || cand_info.vertices.size() != verts.size()
                    || cand_info.squash_group != curr_info.squash_group) {
                    continue;
                }

                /* if we are squashing groups we need to check if they are the
                 * same group */
                if (cand_info.squash_group
                    && cand_info.group_mask != curr_info.group_mask) {
                    continue;
                }

                u32 final_id = cand_info.final_id;
                assert(final_id != MO_INVALID_IDX);
                assert(curr_info.final_id == MO_INVALID_IDX);
                curr_info.final_id = final_id;
                final_id_to_literal[final_id].insert(int_id);
                goto next_lit;
            }
        }

    assign_new_id:
        /* oh well, have to give it a fresh one, hang the expense */
        DEBUG_PRINTF("allocating final id %u to %u\n", *next_final_id, int_id);
                assert(curr_info.final_id == MO_INVALID_IDX);
        curr_info.final_id = *next_final_id;
        final_id_to_literal[*next_final_id].insert(int_id);
        (*next_final_id)++;
    next_lit:;
    }
}

static
bool isUsedLiteral(const RoseBuildImpl &build, u32 lit_id) {
    assert(lit_id < build.literal_info.size());
    const auto &info = build.literal_info[lit_id];
    if (!info.vertices.empty()) {
        return true;
    }

    for (const u32 &delayed_id : info.delayed_ids) {
        assert(delayed_id < build.literal_info.size());
        const rose_literal_info &delayed_info = build.literal_info[delayed_id];
        if (!delayed_info.vertices.empty()) {
            return true;
        }
    }

    DEBUG_PRINTF("literal %u has no refs\n", lit_id);
    return false;
}

/** \brief Allocate final literal IDs for all literals.  */
static
void allocateFinalLiteralId(RoseBuildImpl &build, build_context &bc) {
    set<u32> anch;
    set<u32> norm;
    set<u32> delay;

    /* undelayed ids come first */
    assert(bc.final_id_to_literal.empty());
    u32 next_final_id = 0;
    for (u32 i = 0; i < build.literal_info.size(); i++) {
        assert(!build.hasFinalId(i));

        if (!isUsedLiteral(build, i)) {
            /* what is this literal good for? absolutely nothing */
            continue;
        }

        // The special EOD event literal has its own program and does not need
        // a real literal ID.
        if (i == build.eod_event_literal_id) {
            assert(build.eod_event_literal_id != MO_INVALID_IDX);
            continue;
        }

        if (build.isDelayed(i)) {
            assert(!build.literal_info[i].requires_benefits);
            delay.insert(i);
        } else if (build.literals.right.at(i).table == ROSE_ANCHORED) {
            anch.insert(i);
        } else {
            norm.insert(i);
        }
    }

    /* normal lits */
    allocateFinalIdToSet(build, bc, norm, &next_final_id);

    /* next anchored stuff */
    build.anchored_base_id = next_final_id;
    allocateFinalIdToSet(build, bc, anch, &next_final_id);

    /* delayed ids come last */
    build.delay_base_id = next_final_id;
    allocateFinalIdToSet(build, bc, delay, &next_final_id);
}

static
aligned_unique_ptr<RoseEngine> addSmallWriteEngine(RoseBuildImpl &build,
                                        aligned_unique_ptr<RoseEngine> rose) {
    assert(rose);

    if (roseIsPureLiteral(rose.get())) {
        DEBUG_PRINTF("pure literal case, not adding smwr\n");
        return rose;
    }

    u32 qual = roseQuality(rose.get());
    auto smwr_engine = build.smwr.build(qual);
    if (!smwr_engine) {
        DEBUG_PRINTF("no smwr built\n");
        return rose;
    }

    const size_t mainSize = roseSize(rose.get());
    const size_t smallWriteSize = smwrSize(smwr_engine.get());
    DEBUG_PRINTF("adding smwr engine, size=%zu\n", smallWriteSize);

    const size_t smwrOffset = ROUNDUP_CL(mainSize);
    const size_t newSize = smwrOffset + smallWriteSize;

    auto rose2 = aligned_zmalloc_unique<RoseEngine>(newSize);
    char *ptr = (char *)rose2.get();
    memcpy(ptr, rose.get(), mainSize);
    memcpy(ptr + smwrOffset, smwr_engine.get(), smallWriteSize);

    rose2->smallWriteOffset = verify_u32(smwrOffset);
    rose2->size = verify_u32(newSize);

    return rose2;
}

/**
 * \brief Returns the pair (number of literals, max length) for all real
 * literals in the floating table that are in-use.
 */
static
pair<size_t, size_t> floatingCountAndMaxLen(const RoseBuildImpl &build) {
    size_t num = 0;
    size_t max_len = 0;

    for (const auto &e : build.literals.right) {
        const u32 id = e.first;
        const rose_literal_id &lit = e.second;

        if (lit.table != ROSE_FLOATING) {
            continue;
        }
        if (lit.delay) {
            // Skip delayed literals, so that we only count the undelayed
            // version that ends up in the HWLM table.
            continue;
        }
        if (!isUsedLiteral(build, id)) {
            continue;
        }

        num++;
        max_len = max(max_len, lit.s.length());
    }
    DEBUG_PRINTF("%zu floating literals with max_len=%zu\n", num, max_len);
    return {num, max_len};
}

size_t calcLongLitThreshold(const RoseBuildImpl &build,
                            const size_t historyRequired) {
    const auto &cc = build.cc;

    // In block mode, we don't have history, so we don't need long literal
    // support and can just use "medium-length" literal confirm. TODO: we could
    // specialize further and have a block mode literal confirm instruction.
    if (!cc.streaming) {
        return SIZE_MAX;
    }

    size_t longLitLengthThreshold = ROSE_LONG_LITERAL_THRESHOLD_MIN;

    // Expand to size of history we've already allocated. Note that we need N-1
    // bytes of history to match a literal of length N.
    longLitLengthThreshold = max(longLitLengthThreshold, historyRequired + 1);

    // If we only have one literal, allow for a larger value in order to avoid
    // building a long literal table for a trivial Noodle case that we could
    // fit in history.
    const auto num_len = floatingCountAndMaxLen(build);
    if (num_len.first == 1) {
        if (num_len.second > longLitLengthThreshold) {
            DEBUG_PRINTF("expanding for single literal of length %zu\n",
                         num_len.second);
            longLitLengthThreshold = num_len.second;
        }
    }

    // Clamp to max history available.
    longLitLengthThreshold =
        min(longLitLengthThreshold, size_t{cc.grey.maxHistoryAvailable} + 1);

    return longLitLengthThreshold;
}

aligned_unique_ptr<RoseEngine> RoseBuildImpl::buildFinalEngine(u32 minWidth) {
    DerivedBoundaryReports dboundary(boundary);

    size_t historyRequired = calcHistoryRequired(); // Updated by HWLM.
    size_t longLitLengthThreshold = calcLongLitThreshold(*this,
                                                         historyRequired);
    DEBUG_PRINTF("longLitLengthThreshold=%zu\n", longLitLengthThreshold);

    build_context bc;
    allocateFinalLiteralId(*this, bc);
    groupByFragment(*this, bc);

    // Write the fragment IDs into the literal_info structures.
    for (auto &info : literal_info) {
        if (info.final_id == MO_INVALID_IDX) {
            continue;
        }
        assert(contains(final_to_frag_map, info.final_id));
        info.fragment_id = final_to_frag_map.at(info.final_id);
    }

    auto anchored_dfas = buildAnchoredDfas(*this);

    bc.floatingMinLiteralMatchOffset =
        findMinFloatingLiteralMatch(*this, anchored_dfas);
    bc.longLitLengthThreshold = longLitLengthThreshold;
    bc.needs_catchup = needsCatchup(*this, anchored_dfas);
    recordResources(bc.resources, *this);
    if (!anchored_dfas.empty()) {
        bc.resources.has_anchored = true;
    }
    bc.needs_mpv_catchup = needsMpvCatchup(*this);
    bc.vertex_group_map = getVertexGroupMap(*this);
    bc.squashable_groups = getSquashableGroups(*this);

    auto boundary_out = makeBoundaryPrograms(*this, bc, boundary, dboundary);

    u32 reportProgramOffset;
    u32 reportProgramCount;
    tie(reportProgramOffset, reportProgramCount) =
        buildReportPrograms(*this, bc);

    // Build NFAs
    set<u32> no_retrigger_queues;
    bool mpv_as_outfix;
    prepMpv(*this, bc, &historyRequired, &mpv_as_outfix);
    u32 outfixBeginQueue = qif.allocated_count();
    if (!prepOutfixes(*this, bc, &historyRequired)) {
        return nullptr;
    }
    u32 outfixEndQueue = qif.allocated_count();
    u32 leftfixBeginQueue = outfixEndQueue;

    set<u32> eager_queues;

    /* Note: buildNfas may reduce the lag for vertices that have prefixes */
    if (!buildNfas(*this, bc, qif, &no_retrigger_queues, &eager_queues,
                   &leftfixBeginQueue)) {
        return nullptr;
    }
    u32 eodNfaIterOffset = buildEodNfaIterator(bc, leftfixBeginQueue);
    buildCountingMiracles(bc);

    u32 queue_count = qif.allocated_count(); /* excludes anchored matcher q;
                                              * som rev nfas */
    if (queue_count > cc.grey.limitRoseEngineCount) {
        throw ResourceLimitError();
    }

    vector<u32> suffixEkeyLists;
    buildSuffixEkeyLists(*this, bc, qif, &suffixEkeyLists);

    assignStateIndices(*this, bc);

    u32 laggedRoseCount = 0;
    vector<LeftNfaInfo> leftInfoTable;
    buildLeftInfoTable(*this, bc, eager_queues, leftfixBeginQueue,
                       queue_count - leftfixBeginQueue, leftInfoTable,
                       &laggedRoseCount, &historyRequired);

    buildLiteralPrograms(*this, bc);
    u32 delayProgramOffset = buildDelayPrograms(*this, bc);
    u32 anchoredProgramOffset = buildAnchoredPrograms(*this, bc);

    u32 eodProgramOffset = writeEodProgram(*this, bc, eodNfaIterOffset);

    size_t longLitStreamStateRequired = 0;
    u32 longLitTableOffset = buildLongLiteralTable(*this, bc.engine_blob,
                bc.longLiterals, longLitLengthThreshold, &historyRequired,
                &longLitStreamStateRequired);

    vector<mmbit_sparse_iter> activeLeftIter;
    buildActiveLeftIter(leftInfoTable, activeLeftIter);

    u32 lastByteOffset = buildLastByteIter(g, bc);
    u32 eagerIterOffset = buildEagerQueueIter(eager_queues, leftfixBeginQueue,
                                              queue_count, bc);

    // Enforce role table resource limit.
    if (num_vertices(g) > cc.grey.limitRoseRoleCount) {
        throw ResourceLimitError();
    }

    u32 currOffset;  /* relative to base of RoseEngine */
    if (!bc.engine_blob.empty()) {
        currOffset = bc.engine_blob.base_offset + bc.engine_blob.size();
    } else {
        currOffset = sizeof(RoseEngine);
    }

    UNUSED const size_t engineBlobSize = bc.engine_blob.size(); // test later

    currOffset = ROUNDUP_CL(currOffset);
    DEBUG_PRINTF("currOffset %u\n", currOffset);

    // Build anchored matcher.
    size_t asize = 0;
    u32 amatcherOffset = 0;
    auto atable = buildAnchoredMatcher(*this, anchored_dfas, &asize);
    if (atable) {
        currOffset = ROUNDUP_CL(currOffset);
        amatcherOffset = currOffset;
        currOffset += verify_u32(asize);
    }

    // Build floating HWLM matcher.
    rose_group fgroups = 0;
    size_t fsize = 0;
    auto ftable = buildFloatingMatcher(*this, bc.longLitLengthThreshold,
                                       &fgroups, &fsize, &historyRequired);
    u32 fmatcherOffset = 0;
    if (ftable) {
        currOffset = ROUNDUP_CL(currOffset);
        fmatcherOffset = currOffset;
        currOffset += verify_u32(fsize);
        bc.resources.has_floating = true;
    }

    // Build delay rebuild HWLM matcher.
    size_t drsize = 0;
    auto drtable =
        buildDelayRebuildMatcher(*this, bc.longLitLengthThreshold, &drsize);
    u32 drmatcherOffset = 0;
    if (drtable) {
        currOffset = ROUNDUP_CL(currOffset);
        drmatcherOffset = currOffset;
        currOffset += verify_u32(drsize);
    }

    // Build EOD-anchored HWLM matcher.
    size_t esize = 0;
    auto etable = buildEodAnchoredMatcher(*this, &esize);
    u32 ematcherOffset = 0;
    if (etable) {
        currOffset = ROUNDUP_CL(currOffset);
        ematcherOffset = currOffset;
        currOffset += verify_u32(esize);
    }

    // Build small-block HWLM matcher.
    size_t sbsize = 0;
    auto sbtable = buildSmallBlockMatcher(*this, &sbsize);
    u32 sbmatcherOffset = 0;
    if (sbtable) {
        currOffset = ROUNDUP_CL(currOffset);
        sbmatcherOffset = currOffset;
        currOffset += verify_u32(sbsize);
    }

    u32 leftOffset = ROUNDUP_N(currOffset, alignof(LeftNfaInfo));
    u32 roseLen = sizeof(LeftNfaInfo) * leftInfoTable.size();
    currOffset = leftOffset + roseLen;

    u32 lookaroundReachOffset = currOffset;
    u32 lookaroundReachLen = REACH_BITVECTOR_LEN * bc.lookaround.size();
    currOffset = lookaroundReachOffset + lookaroundReachLen;

    u32 lookaroundTableOffset = currOffset;
    u32 lookaroundTableLen = sizeof(s8) * bc.lookaround.size();
    currOffset = lookaroundTableOffset + lookaroundTableLen;

    u32 nfaInfoOffset = ROUNDUP_N(currOffset, sizeof(u32));
    u32 nfaInfoLen = sizeof(NfaInfo) * queue_count;
    currOffset = nfaInfoOffset + nfaInfoLen;

    currOffset = ROUNDUP_N(currOffset, alignof(mmbit_sparse_iter));
    u32 activeLeftIterOffset = currOffset;
    currOffset += activeLeftIter.size() * sizeof(mmbit_sparse_iter);

    u32 activeArrayCount = leftfixBeginQueue;
    u32 activeLeftCount = leftInfoTable.size();
    u32 rosePrefixCount = countRosePrefixes(leftInfoTable);

    u32 rev_nfa_table_offset;
    vector<u32> rev_nfa_offsets;
    prepSomRevNfas(ssm, &rev_nfa_table_offset, &rev_nfa_offsets, &currOffset);

    // Build engine header and copy tables into place.

    u32 anchorStateSize = atable ? anchoredStateSize(*atable) : 0;

    DEBUG_PRINTF("rose history required %zu\n", historyRequired);
    assert(!cc.streaming || historyRequired <= cc.grey.maxHistoryAvailable);

    // Some SOM schemes (reverse NFAs, for example) may require more history.
    historyRequired = max(historyRequired, (size_t)ssm.somHistoryRequired());

    assert(!cc.streaming || historyRequired <=
           max(cc.grey.maxHistoryAvailable, cc.grey.somMaxRevNfaLength));

    RoseStateOffsets stateOffsets;
    memset(&stateOffsets, 0, sizeof(stateOffsets));
    fillStateOffsets(*this, bc.numStates, anchorStateSize,
                     activeArrayCount, activeLeftCount, laggedRoseCount,
                     longLitStreamStateRequired, historyRequired,
                     &stateOffsets);

    scatter_plan_raw state_scatter;
    buildStateScatterPlan(sizeof(u8), bc.numStates,
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
        assert(amatcherOffset);
        memcpy(ptr + amatcherOffset, atable.get(), asize);
    }
    if (ftable) {
        assert(fmatcherOffset);
        memcpy(ptr + fmatcherOffset, ftable.get(), fsize);
    }
    if (drtable) {
        assert(drmatcherOffset);
        memcpy(ptr + drmatcherOffset, drtable.get(), drsize);
    }
    if (etable) {
        assert(ematcherOffset);
        memcpy(ptr + ematcherOffset, etable.get(), esize);
    }
    if (sbtable) {
        assert(sbmatcherOffset);
        memcpy(ptr + sbmatcherOffset, sbtable.get(), sbsize);
    }

    memcpy(&engine->stateOffsets, &stateOffsets, sizeof(stateOffsets));

    engine->historyRequired = verify_u32(historyRequired);

    engine->ekeyCount = rm.numEkeys();
    engine->dkeyCount = rm.numDkeys();
    engine->dkeyLogSize = fatbit_size(engine->dkeyCount);
    engine->invDkeyOffset = dkeyOffset;
    copy_bytes(ptr + dkeyOffset, rm.getDkeyToReportTable());

    engine->somHorizon = ssm.somPrecision();
    engine->somLocationCount = ssm.numSomSlots();
    engine->somLocationFatbitSize = fatbit_size(engine->somLocationCount);

    engine->needsCatchup = bc.needs_catchup ? 1 : 0;

    engine->reportProgramOffset = reportProgramOffset;
    engine->reportProgramCount = reportProgramCount;
    engine->delayProgramOffset = delayProgramOffset;
    engine->anchoredProgramOffset = anchoredProgramOffset;
    engine->runtimeImpl = pickRuntimeImpl(*this, bc, outfixEndQueue);
    engine->mpvTriggeredByLeaf = anyEndfixMpvTriggers(*this);

    engine->activeArrayCount = activeArrayCount;
    engine->activeLeftCount = activeLeftCount;
    engine->queueCount = queue_count;
    engine->activeQueueArraySize = fatbit_size(queue_count);
    engine->eagerIterOffset = eagerIterOffset;
    engine->handledKeyCount = bc.handledKeys.size();
    engine->handledKeyFatbitSize = fatbit_size(engine->handledKeyCount);

    engine->rolesWithStateCount = bc.numStates;

    engine->leftOffset = leftOffset;
    engine->roseCount = verify_u32(leftInfoTable.size());
    engine->lookaroundTableOffset = lookaroundTableOffset;
    engine->lookaroundReachOffset = lookaroundReachOffset;
    engine->outfixBeginQueue = outfixBeginQueue;
    engine->outfixEndQueue = outfixEndQueue;
    engine->leftfixBeginQueue = leftfixBeginQueue;
    engine->initMpvNfa = mpv_as_outfix ? 0 : MO_INVALID_IDX;
    engine->stateSize = mmbit_size(bc.numStates);
    engine->anchorStateSize = anchorStateSize;
    engine->nfaInfoOffset = nfaInfoOffset;

    engine->eodProgramOffset = eodProgramOffset;

    engine->lastByteHistoryIterOffset = lastByteOffset;

    engine->delay_count =
        verify_u32(bc.final_id_to_literal.size() - delay_base_id);
    engine->delay_fatbit_size = fatbit_size(engine->delay_count);
    engine->delay_base_id = delay_base_id;
    engine->anchored_base_id = anchored_base_id;
    engine->anchored_count = delay_base_id - anchored_base_id;
    engine->anchored_fatbit_size = fatbit_size(engine->anchored_count);

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
    engine->drmatcherOffset = drmatcherOffset;
    engine->longLitTableOffset = longLitTableOffset;
    engine->amatcherMinWidth = findMinWidth(*this, ROSE_ANCHORED);
    engine->fmatcherMinWidth = findMinWidth(*this, ROSE_FLOATING);
    engine->eodmatcherMinWidth = findMinWidth(*this, ROSE_EOD_ANCHORED);
    engine->amatcherMaxBiAnchoredWidth = findMaxBAWidth(*this, ROSE_ANCHORED);
    engine->fmatcherMaxBiAnchoredWidth = findMaxBAWidth(*this, ROSE_FLOATING);
    engine->size = currOffset;
    engine->minWidth = hasBoundaryReports(boundary) ? 0 : minWidth;
    engine->minWidthExcludingBoundaries = minWidth;
    engine->floatingMinLiteralMatchOffset = bc.floatingMinLiteralMatchOffset;

    engine->maxBiAnchoredWidth = findMaxBAWidth(*this);
    engine->noFloatingRoots = hasNoFloatingRoots();
    engine->requiresEodCheck = hasEodAnchors(*this, bc, outfixEndQueue);
    engine->hasOutfixesInSmallBlock = hasNonSmallBlockOutfix(outfixes);
    engine->canExhaust = rm.patternSetCanExhaust();
    engine->hasSom = hasSom;

    /* populate anchoredDistance, floatingDistance, floatingMinDistance, etc */
    fillMatcherDistances(*this, engine.get());

    engine->initialGroups = getInitialGroups();
    engine->floating_group_mask = fgroups;
    engine->totalNumLiterals = verify_u32(literal_info.size());
    engine->asize = verify_u32(asize);
    engine->ematcherRegionSize = ematcher_region_size;
    engine->longLitStreamState = verify_u32(longLitStreamStateRequired);

    engine->boundary.reportEodOffset = boundary_out.reportEodOffset;
    engine->boundary.reportZeroOffset = boundary_out.reportZeroOffset;
    engine->boundary.reportZeroEodOffset = boundary_out.reportZeroEodOffset;

    write_out(&engine->state_init, (char *)engine.get(), state_scatter,
              state_scatter_aux_offset);

    NfaInfo *nfa_infos = (NfaInfo *)(ptr + nfaInfoOffset);
    populateNfaInfoBasics(*this, bc, outfixes, suffixEkeyLists,
                          no_retrigger_queues, nfa_infos);
    updateNfaState(bc, &engine->stateOffsets, nfa_infos,
                   &engine->scratchStateSize, &engine->nfaStateSize,
                   &engine->tStateSize);

    // Copy in other tables
    bc.engine_blob.write_bytes(engine.get());
    copy_bytes(ptr + engine->leftOffset, leftInfoTable);

    fillLookaroundTables(ptr + lookaroundTableOffset,
                         ptr + lookaroundReachOffset, bc.lookaround);

    fillInSomRevNfas(engine.get(), ssm, rev_nfa_table_offset, rev_nfa_offsets);
    copy_bytes(ptr + engine->activeLeftIterOffset, activeLeftIter);

    // Safety check: we shouldn't have written anything to the engine blob
    // after we copied it into the engine bytecode.
    assert(bc.engine_blob.size() == engineBlobSize);

    // Add a small write engine if appropriate.
    engine = addSmallWriteEngine(*this, move(engine));

    DEBUG_PRINTF("rose done %p\n", engine.get());
    return engine;
}

} // namespace ue2
