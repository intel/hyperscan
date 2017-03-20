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
#include "rose_build_dump.h"
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
#include "util/dump_charclass.h"
#include "util/fatbit_build.h"
#include "util/graph_range.h"
#include "util/make_unique.h"
#include "util/multibit_build.h"
#include "util/noncopyable.h"
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

static constexpr u32 INVALID_QUEUE = ~0U;

struct left_build_info {
    // Constructor for an engine implementation.
    left_build_info(u32 q, u32 l, u32 t, rose_group sm,
                    const std::vector<u8> &stops, u32 max_ql, u8 cm_count,
                    const CharReach &cm_cr)
        : queue(q), lag(l), transient(t), squash_mask(sm), stopAlphabet(stops),
          max_queuelen(max_ql), countingMiracleCount(cm_count),
          countingMiracleReach(cm_cr) {}

    // Constructor for a lookaround implementation.
    explicit left_build_info(const vector<vector<LookEntry>> &looks)
        : has_lookaround(true), lookaround(looks) {}

    u32 queue = INVALID_QUEUE; /* uniquely idents the left_build_info */
    u32 lag = 0;
    u32 transient = 0;
    rose_group squash_mask = ~rose_group{0};
    vector<u8> stopAlphabet;
    u32 max_queuelen = 0;
    u8 countingMiracleCount = 0;
    CharReach countingMiracleReach;
    u32 countingMiracleOffset = 0; /* populated later when laying out bytecode */
    /* leftfix can be completely implemented with lookaround */
    bool has_lookaround = false;
    vector<vector<LookEntry>> lookaround; // alternative implementation to the NFA
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

struct build_context : noncopyable {
    /** \brief information about engines to the left of a vertex */
    map<RoseVertex, left_build_info> leftfix_info;

    /** \brief mapping from suffix to queue index. */
    map<suffix_id, u32> suffixes;

    /** \brief Simple cache of programs written to engine blob, used for
     * deduplication. */
    ue2::unordered_map<RoseProgram, u32, RoseProgramHash,
                       RoseProgramEquivalence> program_cache;

    /** \brief LookEntry list cache, so that we can reuse the look index and
     * reach index for the same lookaround. */
    ue2::unordered_map<vector<vector<LookEntry>>,
                       pair<size_t, size_t>> lookaround_cache;

    /** \brief Lookaround table for Rose roles. */
    vector<vector<vector<LookEntry>>> lookaround;

    /** \brief Lookaround look table size. */
    size_t lookTableSize = 0;

    /** \brief Lookaround reach table size.
     * since single path lookaround and multi-path lookaround have different
     * bitvectors range (32 and 256), we need to maintain both look table size
     * and reach table size. */
    size_t reachTableSize = 0;

    /** \brief State indices, for those roles that have them.
     * Each vertex present has a unique state index in the range
     * [0, roleStateIndices.size()). */
    unordered_map<RoseVertex, u32> roleStateIndices;

    /** \brief Mapping from queue index to bytecode offset for built engines
     * that have already been pushed into the engine_blob. */
    ue2::unordered_map<u32, u32> engineOffsets;

    /** \brief List of long literals (ones with CHECK_LONG_LIT instructions)
     * that need hash table support. */
    vector<ue2_case_string> longLiterals;

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
};

/** \brief Data only used during construction of various programs (literal,
 * anchored, delay, etc). */
struct ProgramBuild : noncopyable {
    explicit ProgramBuild(u32 fMinLitOffset)
        : floatingMinLiteralMatchOffset(fMinLitOffset) {
    }

    /** \brief Minimum offset of a match from the floating table. */
    const u32 floatingMinLiteralMatchOffset;

    /** \brief Mapping from vertex to key, for vertices with a
     * CHECK_NOT_HANDLED instruction. */
    ue2::unordered_map<RoseVertex, u32> handledKeys;

    /** \brief Mapping from Rose literal ID to anchored program index. */
    map<u32, u32> anchored_programs;

    /** \brief Mapping from Rose literal ID to delayed program index. */
    map<u32, u32> delay_programs;

    /** \brief Mapping from every vertex to the groups that must be on for that
     * vertex to be reached. */
    ue2::unordered_map<RoseVertex, rose_group> vertex_group_map;

    /** \brief Global bitmap of groups that can be squashed. */
    rose_group squashable_groups = 0;
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
u8 pickRuntimeImpl(const RoseBuildImpl &build, const RoseResources &resources,
                   UNUSED u32 outfixEndQueue) {
    DEBUG_PRINTF("has_outfixes=%d\n", resources.has_outfixes);
    DEBUG_PRINTF("has_suffixes=%d\n", resources.has_suffixes);
    DEBUG_PRINTF("has_leftfixes=%d\n", resources.has_leftfixes);
    DEBUG_PRINTF("has_literals=%d\n", resources.has_literals);
    DEBUG_PRINTF("has_states=%d\n", resources.has_states);
    DEBUG_PRINTF("checks_groups=%d\n", resources.checks_groups);
    DEBUG_PRINTF("has_lit_delay=%d\n", resources.has_lit_delay);
    DEBUG_PRINTF("has_lit_check=%d\n", resources.has_lit_check);
    DEBUG_PRINTF("has_anchored=%d\n", resources.has_anchored);
    DEBUG_PRINTF("has_floating=%d\n", resources.has_floating);
    DEBUG_PRINTF("has_eod=%d\n", resources.has_eod);

    if (isPureFloating(resources, build.cc)) {
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
void fillStateOffsets(const RoseBuildImpl &build, u32 rolesWithStateCount,
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
    so->groups_size = (build.group_end + 7) / 8;
    assert(so->groups_size <= sizeof(u64a));
    curr_offset += so->groups_size;

    // The history consists of the bytes in the history only. YAY
    so->history = curr_offset;
    curr_offset += historyRequired;

    // Exhaustion multibit.
    so->exhausted = curr_offset;
    curr_offset += mmbit_size(build.rm.numEkeys());

    // SOM locations and valid/writeable multibit structures.
    if (build.ssm.numSomSlots()) {
        const u32 somWidth = build.ssm.somPrecision();
        if (somWidth) { // somWidth is zero in block mode.
            curr_offset = ROUNDUP_N(curr_offset, somWidth);
            so->somLocation = curr_offset;
            curr_offset += build.ssm.numSomSlots() * somWidth;
        } else {
            so->somLocation = 0;
        }
        so->somValid = curr_offset;
        curr_offset += mmbit_size(build.ssm.numSomSlots());
        so->somWritable = curr_offset;
        curr_offset += mmbit_size(build.ssm.numSomSlots());
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
                                const vector<ExclusiveInfo> &exclusive_info,
                                map<RoseVertex, left_build_info> &leftfix_info,
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
                leftfix_info.emplace(v, left_build_info(qi, lag, max_width,
                                                        squash_mask, stop,
                                                        max_queuelen, cm_count,
                                                        cm_cr));
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
    updateExclusiveInfixProperties(build, exclusive_info, bc.leftfix_info, 
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
            vector<vector<LookEntry>> lookaround;
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
aligned_unique_ptr<NFA> buildOutfix(const RoseBuildImpl &build, OutfixInfo &outfix) {
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
void assignSuffixQueues(RoseBuildImpl &build, map<suffix_id, u32> &suffixes) {
    const RoseGraph &g = build.g;

    for (auto v : vertices_range(g)) {
        if (!g[v].suffix) {
            continue;
        }

        const suffix_id s(g[v].suffix);

        DEBUG_PRINTF("vertex %zu triggers suffix %p\n", g[v].index, s.graph());

        // We may have already built this NFA.
        if (contains(suffixes, s)) {
            continue;
        }

        u32 queue = build.qif.get_queue();
        DEBUG_PRINTF("assigning %p to queue %u\n", s.graph(), queue);
        suffixes.emplace(s, queue);
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

    assignSuffixQueues(tbi, bc.suffixes);

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
void allocateStateSpace(const NFA *nfa, NfaInfo &nfa_info, bool is_transient,
                        RoseStateOffsets *so, u32 *scratchStateSize,
                        u32 *streamStateSize, u32 *transientStateSize) {
    u32 state_offset;
    if (is_transient) {
        // Transient engines do not use stream state, but must have room in
        // transient state (stored in scratch).
        state_offset = *transientStateSize;
        *transientStateSize += nfa->streamStateSize;
    } else {
        // Pack NFA stream state on to the end of the Rose stream state.
        state_offset = so->end;
        so->end += nfa->streamStateSize;
        *streamStateSize += nfa->streamStateSize;
    }

    nfa_info.stateOffset = state_offset;

    // Uncompressed state in scratch must be aligned.
    u32 alignReq = state_alignment(*nfa);
    assert(alignReq);
    *scratchStateSize = ROUNDUP_N(*scratchStateSize, alignReq);
    nfa_info.fullStateOffset = *scratchStateSize;
    *scratchStateSize += nfa->scratchStateSize;
}

static
set<u32>
findTransientQueues(const map<RoseVertex, left_build_info> &leftfix_info) {
    DEBUG_PRINTF("curating transient queues\n");
    set<u32> out;
    for (const auto &left : leftfix_info | map_values) {
        if (left.transient) {
            DEBUG_PRINTF("q %u is transient\n", left.queue);
            out.insert(left.queue);
        }
    }
    return out;
}

static
void updateNfaState(const build_context &bc, vector<NfaInfo> &nfa_infos,
                    RoseStateOffsets *so, u32 *scratchStateSize,
                    u32 *streamStateSize, u32 *transientStateSize) {
    if (nfa_infos.empty()) {
        assert(bc.engineOffsets.empty());
        return;
    }

    *streamStateSize = 0;
    *transientStateSize = 0;
    *scratchStateSize = 0;

    auto transient_queues = findTransientQueues(bc.leftfix_info);

    for (const auto &m : bc.engineOffsets) {
        const NFA *nfa = get_nfa_from_blob(bc, m.first);
        u32 qi = nfa->queueIndex;
        bool is_transient = contains(transient_queues, qi);
        NfaInfo &nfa_info = nfa_infos[qi];
        allocateStateSpace(nfa, nfa_info, is_transient, so, scratchStateSize,
                           streamStateSize, transientStateSize);
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

    auto iter = mmbBuildSparseIterator(lb_roles, bc.roleStateIndices.size());
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
vector<u32> buildSuffixEkeyLists(const RoseBuildImpl &build, build_context &bc,
                                 const QueueIndexFactory &qif) {
    vector<u32> out(qif.allocated_count());

    map<u32, vector<u32>> qi_to_ekeys; /* for determinism */

    for (const auto &e : bc.suffixes) {
        const suffix_id &s = e.first;
        u32 qi = e.second;
        set<u32> ekeys = reportsToEkeys(all_reports(s), build.rm);

        if (!ekeys.empty()) {
            qi_to_ekeys[qi] = {ekeys.begin(), ekeys.end()};
        }
    }

    /* for each outfix also build elists */
    for (const auto &outfix : build.outfixes) {
        u32 qi = outfix.get_queue();
        set<u32> ekeys = reportsToEkeys(all_reports(outfix), build.rm);

        if (!ekeys.empty()) {
            qi_to_ekeys[qi] = {ekeys.begin(), ekeys.end()};
        }
    }

    for (auto &e : qi_to_ekeys) {
        u32 qi = e.first;
        auto &ekeys = e.second;
        assert(!ekeys.empty());
        ekeys.push_back(INVALID_EKEY); /* terminator */
        out[qi] = bc.engine_blob.add_range(ekeys);
    }

    return out;
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

    auto iter = mmbBuildSparseIterator(keys, activeQueueCount);
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

struct DerivedBoundaryReports {
    explicit DerivedBoundaryReports(const BoundaryReports &boundary) {
        insert(&report_at_0_eod_full, boundary.report_at_0_eod);
        insert(&report_at_0_eod_full, boundary.report_at_eod);
        insert(&report_at_0_eod_full, boundary.report_at_0);
    }
    set<ReportID> report_at_0_eod_full;
};

static
void addSomRevNfas(build_context &bc, RoseEngine &proto,
                   const SomSlotManager &ssm) {
    const auto &nfas = ssm.getRevNfas();
    vector<u32> nfa_offsets;
    nfa_offsets.reserve(nfas.size());
    for (const auto &nfa : nfas) {
        assert(nfa);
        u32 offset = bc.engine_blob.add(*nfa, nfa->length);
        DEBUG_PRINTF("wrote SOM rev NFA %zu (len %u) to offset %u\n",
                     nfa_offsets.size(), nfa->length, offset);
        nfa_offsets.push_back(offset);
        /* note: som rev nfas don't need a queue assigned as only run in block
         * mode reverse */
    }

    proto.somRevCount = verify_u32(nfas.size());
    proto.somRevOffsetOffset = bc.engine_blob.add_range(nfa_offsets);
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
                     const RoseBuildImpl &build,
                     const vector<LitFragment> &fragments) {
    if (!build.outfixes.empty()) {
        resources.has_outfixes = true;
    }

    resources.has_literals = !fragments.empty();

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
void recordLongLiterals(vector<ue2_case_string> &longLiterals,
                        const RoseProgram &program) {
    for (const auto &ri : program) {
        if (const auto *ri_check =
                dynamic_cast<const RoseInstrCheckLongLit *>(ri.get())) {
            DEBUG_PRINTF("found CHECK_LONG_LIT for string '%s'\n",
                         escapeString(ri_check->literal).c_str());
            longLiterals.emplace_back(ri_check->literal, false);
            continue;
        }
        if (const auto *ri_check =
                dynamic_cast<const RoseInstrCheckLongLitNocase *>(ri.get())) {
            DEBUG_PRINTF("found CHECK_LONG_LIT_NOCASE for string '%s'\n",
                         escapeString(ri_check->literal).c_str());
            longLiterals.emplace_back(ri_check->literal, true);
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
    recordLongLiterals(bc.longLiterals, program);

    u32 len = 0;
    auto prog_bytecode = writeProgram(bc.engine_blob, program, &len);
    u32 offset = bc.engine_blob.add(prog_bytecode.get(), len,
                                    ROSE_INSTR_MIN_ALIGN);
    DEBUG_PRINTF("prog len %u written at offset %u\n", len, offset);
    bc.program_cache.emplace(move(program), offset);
    return offset;
}

static
u32 writeActiveLeftIter(RoseEngineBlob &engine_blob,
                        const vector<LeftNfaInfo> &leftInfoTable) {
    vector<u32> keys;
    for (size_t i = 0; i < leftInfoTable.size(); i++) {
        if (!leftInfoTable[i].transient) {
            DEBUG_PRINTF("leftfix %zu is active\n", i);
            keys.push_back(verify_u32(i));
        }
    }

    DEBUG_PRINTF("%zu active leftfixes\n", keys.size());

    if (keys.empty()) {
        return 0;
    }

    auto iter = mmbBuildSparseIterator(keys, verify_u32(leftInfoTable.size()));
    return engine_blob.add_iterator(iter);
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
void writeLookaround(const vector<LookEntry> &look_vec, s8 *&look, u8 *&reach) {
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
void writeMultipathLookaround(const vector<vector<LookEntry>> &multi_look,
                              s8 *&look, u8 *&reach) {
    for (const auto &m : multi_look) {
        u8 u = 0;
        assert(m.size() == MAX_LOOKAROUND_PATHS);
        for (size_t i = 0; i < m.size(); i++) {
            if (m[i].reach.none()) {
                u |= (u8)1U << i;
            }
        }
        std::fill_n(reach, MULTI_REACH_BITVECTOR_LEN, u);

        for (size_t i = 0; i < m.size(); i++) {
            const CharReach &cr = m[i].reach;
            if (cr.none()) {
                continue;
            }
            *look = m[i].offset;

            for (size_t c = cr.find_first(); c != cr.npos;
                 c = cr.find_next(c)) {
                reach[c] |= (u8)1U << i;
            }
        }

        ++look;
        reach += MULTI_REACH_BITVECTOR_LEN;
    }
}

static
void writeLookaroundTables(build_context &bc, RoseEngine &proto) {
    vector<s8> look_table(bc.lookTableSize, 0);
    vector<u8> reach_table(bc.reachTableSize, 0);
    s8 *look = look_table.data();
    u8 *reach = reach_table.data();
    for (const auto &l : bc.lookaround) {
        if (l.size() == 1) {
            writeLookaround(l.front(), look, reach);
        } else {
            writeMultipathLookaround(l, look, reach);
        }
    }

    proto.lookaroundTableOffset = bc.engine_blob.add_range(look_table);
    proto.lookaroundReachOffset = bc.engine_blob.add_range(reach_table);
}

static
void writeDkeyInfo(const ReportManager &rm, RoseEngineBlob &engine_blob,
                   RoseEngine &proto) {
    const auto inv_dkeys = rm.getDkeyToReportTable();
    proto.invDkeyOffset = engine_blob.add_range(inv_dkeys);
    proto.dkeyCount = rm.numDkeys();
    proto.dkeyLogSize = fatbit_size(proto.dkeyCount);
}

static
void writeLeftInfo(RoseEngineBlob &engine_blob, RoseEngine &proto,
                   const vector<LeftNfaInfo> &leftInfoTable) {
    proto.leftOffset = engine_blob.add_range(leftInfoTable);
    proto.activeLeftIterOffset
        = writeActiveLeftIter(engine_blob, leftInfoTable);
    proto.roseCount = verify_u32(leftInfoTable.size());
    proto.activeLeftCount = verify_u32(leftInfoTable.size());
    proto.rosePrefixCount = countRosePrefixes(leftInfoTable);
}

static
void writeNfaInfo(const RoseBuildImpl &build, build_context &bc,
                  RoseEngine &proto, const set<u32> &no_retrigger_queues) {
    const u32 queue_count = build.qif.allocated_count();
    if (!queue_count) {
        return;
    }

    auto ekey_lists = buildSuffixEkeyLists(build, bc, build.qif);

    vector<NfaInfo> infos(queue_count);
    memset(infos.data(), 0, sizeof(NfaInfo) * queue_count);

    for (u32 qi = 0; qi < queue_count; qi++) {
        const NFA *n = get_nfa_from_blob(bc, qi);
        enforceEngineSizeLimit(n, n->length, build.cc.grey);

        NfaInfo &info = infos[qi];
        info.nfaOffset = bc.engineOffsets.at(qi);
        assert(qi < ekey_lists.size());
        info.ekeyListOffset = ekey_lists.at(qi);
        info.no_retrigger = contains(no_retrigger_queues, qi) ? 1 : 0;
    }

    // Mark outfixes that are in the small block matcher.
    for (const auto &out : build.outfixes) {
        const u32 qi = out.get_queue();
        assert(qi < infos.size());
        infos.at(qi).in_sbmatcher = out.in_sbmatcher;
    }

    // Mark suffixes triggered by EOD table literals.
    const RoseGraph &g = build.g;
    for (auto v : vertices_range(g)) {
        if (!g[v].suffix) {
            continue;
        }
        u32 qi = bc.suffixes.at(g[v].suffix);
        assert(qi < infos.size());
        if (build.isInETable(v)) {
            infos.at(qi).eod = 1;
        }
    }

    // Update state offsets to do with NFAs in proto and in the NfaInfo
    // structures.
    updateNfaState(bc, infos, &proto.stateOffsets, &proto.scratchStateSize,
                   &proto.nfaStateSize, &proto.tStateSize);

    proto.nfaInfoOffset = bc.engine_blob.add_range(infos);
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
void addLookaround(build_context &bc,
                   const vector<vector<LookEntry>> &look,
                   u32 &look_index, u32 &reach_index) {
    // Check the cache.
    auto it = bc.lookaround_cache.find(look);
    if (it != bc.lookaround_cache.end()) {
        look_index = verify_u32(it->second.first);
        reach_index = verify_u32(it->second.second);
        DEBUG_PRINTF("reusing look at idx %u\n", look_index);
        DEBUG_PRINTF("reusing reach at idx %u\n", reach_index);
        return;
    }

    size_t look_idx = bc.lookTableSize;
    size_t reach_idx = bc.reachTableSize;

    if (look.size() == 1) {
        bc.lookTableSize += look.front().size();
        bc.reachTableSize += look.front().size() * REACH_BITVECTOR_LEN;
    } else {
        bc.lookTableSize += look.size();
        bc.reachTableSize += look.size() * MULTI_REACH_BITVECTOR_LEN;
    }

    bc.lookaround_cache.emplace(look, make_pair(look_idx, reach_idx));
    bc.lookaround.emplace_back(look);

    DEBUG_PRINTF("adding look at idx %zu\n", look_idx);
    DEBUG_PRINTF("adding reach at idx %zu\n", reach_idx);
    look_index =  verify_u32(look_idx);
    reach_index = verify_u32(reach_idx);
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
                   map<u32, vector<s8>, cmpNibble> &buckets, u64a &neg_mask) {
    s32 base_offset = verify_s32(look.front().offset);
    for (const auto &entry : look) {
        CharReach cr = entry.reach;
        // Flip heavy character classes to save buckets.
        if (cr.count() > 128 ) {
            cr.flip();
        } else {
            neg_mask ^= 1ULL << (entry.offset - base_offset);
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
bool getShuftiMasks(const vector<LookEntry> &look, array<u8, 32> &hi_mask,
                    array<u8, 32> &lo_mask, u8 *bucket_select_hi,
                    u8 *bucket_select_lo, u64a &neg_mask,
                    u8 &bit_idx, size_t len) {
    map<u32, u16> nib; // map every bucket to its bucket number.
    map<u32, vector<s8>, cmpNibble> bucket2offsets;
    s32 base_offset = look.front().offset;

    bit_idx = 0;
    neg_mask = ~0ULL;

    getAllBuckets(look, bucket2offsets, neg_mask);

    for (const auto &it : bucket2offsets) {
        u32 hi_lo = it.first;
        // New bucket.
        if (!nib[hi_lo]) {
            if ((bit_idx >= 8 && len == 64) || bit_idx >= 16) {
                return false;
            }
            nib[hi_lo] = 1 << bit_idx;

            nibUpdate(nib, hi_lo);
            nibMaskUpdate(hi_mask, hi_lo >> 16, bit_idx);
            nibMaskUpdate(lo_mask, hi_lo & 0xffff, bit_idx);
            bit_idx++;
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
    return true;
}

static
unique_ptr<RoseInstruction>
makeCheckShufti16x8(u32 offset_range, u8 bucket_idx,
                    const array<u8, 32> &hi_mask, const array<u8, 32> &lo_mask,
                    const array<u8, 32> &bucket_select_mask,
                    u32 neg_mask, s32 base_offset,
                    const RoseInstruction *end_inst) {
    if (offset_range > 16 || bucket_idx > 8) {
        return nullptr;
    }
    array<u8, 32> nib_mask;
    array<u8, 16> bucket_select_mask_16;
    copy(lo_mask.begin(), lo_mask.begin() + 16, nib_mask.begin());
    copy(hi_mask.begin(), hi_mask.begin() + 16, nib_mask.begin() + 16);
    copy(bucket_select_mask.begin(), bucket_select_mask.begin() + 16,
         bucket_select_mask_16.begin());
    return make_unique<RoseInstrCheckShufti16x8>
           (nib_mask, bucket_select_mask_16,
            neg_mask & 0xffff, base_offset, end_inst);
}

static
unique_ptr<RoseInstruction>
makeCheckShufti32x8(u32 offset_range, u8 bucket_idx,
                    const array<u8, 32> &hi_mask, const array<u8, 32> &lo_mask,
                    const array<u8, 32> &bucket_select_mask,
                    u32 neg_mask, s32 base_offset,
                    const RoseInstruction *end_inst) {
    if (offset_range > 32 || bucket_idx > 8) {
        return nullptr;
    }

    array<u8, 16> hi_mask_16;
    array<u8, 16> lo_mask_16;
    copy(hi_mask.begin(), hi_mask.begin() + 16, hi_mask_16.begin());
    copy(lo_mask.begin(), lo_mask.begin() + 16, lo_mask_16.begin());
    return make_unique<RoseInstrCheckShufti32x8>
           (hi_mask_16, lo_mask_16, bucket_select_mask,
            neg_mask, base_offset, end_inst);
}

static
unique_ptr<RoseInstruction>
makeCheckShufti16x16(u32 offset_range, u8 bucket_idx,
                     const array<u8, 32> &hi_mask, const array<u8, 32> &lo_mask,
                     const array<u8, 32> &bucket_select_mask_lo,
                     const array<u8, 32> &bucket_select_mask_hi,
                     u32 neg_mask, s32 base_offset,
                     const RoseInstruction *end_inst) {
    if (offset_range > 16 || bucket_idx > 16) {
        return nullptr;
    }

    array<u8, 32> bucket_select_mask_32;
    copy(bucket_select_mask_lo.begin(), bucket_select_mask_lo.begin() + 16,
         bucket_select_mask_32.begin());
    copy(bucket_select_mask_hi.begin(), bucket_select_mask_hi.begin() + 16,
         bucket_select_mask_32.begin() + 16);
    return make_unique<RoseInstrCheckShufti16x16>
           (hi_mask, lo_mask, bucket_select_mask_32,
            neg_mask & 0xffff, base_offset, end_inst);
}
static
unique_ptr<RoseInstruction>
makeCheckShufti32x16(u32 offset_range, u8 bucket_idx,
                     const array<u8, 32> &hi_mask, const array<u8, 32> &lo_mask,
                     const array<u8, 32> &bucket_select_mask_lo,
                     const array<u8, 32> &bucket_select_mask_hi,
                     u32 neg_mask, s32 base_offset,
                     const RoseInstruction *end_inst) {
    if (offset_range > 32 || bucket_idx > 16) {
        return nullptr;
    }

    return make_unique<RoseInstrCheckShufti32x16>
           (hi_mask, lo_mask, bucket_select_mask_hi,
            bucket_select_mask_lo, neg_mask, base_offset, end_inst);
}

static
bool makeRoleShufti(const vector<LookEntry> &look,
                    RoseProgram &program) {

    s32 base_offset = verify_s32(look.front().offset);
    if (look.back().offset >= base_offset + 32) {
        return false;
    }

    u8 bucket_idx = 0; // number of buckets
    u64a neg_mask_64;
    array<u8, 32> hi_mask;
    array<u8, 32> lo_mask;
    array<u8, 32> bucket_select_hi;
    array<u8, 32> bucket_select_lo;
    hi_mask.fill(0);
    lo_mask.fill(0);
    bucket_select_hi.fill(0); // will not be used in 16x8 and 32x8.
    bucket_select_lo.fill(0);

    if (!getShuftiMasks(look, hi_mask, lo_mask, bucket_select_hi.data(),
                        bucket_select_lo.data(), neg_mask_64, bucket_idx, 32)) {
        return false;
    }
    u32 neg_mask = (u32)neg_mask_64;

    DEBUG_PRINTF("hi_mask %s\n",
                 convertMaskstoString(hi_mask.data(), 32).c_str());
    DEBUG_PRINTF("lo_mask %s\n",
                 convertMaskstoString(lo_mask.data(), 32).c_str());
    DEBUG_PRINTF("bucket_select_hi %s\n",
                 convertMaskstoString(bucket_select_hi.data(), 32).c_str());
    DEBUG_PRINTF("bucket_select_lo %s\n",
                 convertMaskstoString(bucket_select_lo.data(), 32).c_str());

    const auto *end_inst = program.end_instruction();
    s32 offset_range = look.back().offset - base_offset + 1;

    auto ri = makeCheckShufti16x8(offset_range, bucket_idx, hi_mask, lo_mask,
                                  bucket_select_lo, neg_mask, base_offset,
                                  end_inst);
    if (!ri) {
        ri = makeCheckShufti32x8(offset_range, bucket_idx, hi_mask, lo_mask,
                                 bucket_select_lo, neg_mask, base_offset,
                                 end_inst);
    }
    if (!ri) {
        ri = makeCheckShufti16x16(offset_range, bucket_idx, hi_mask, lo_mask,
                                  bucket_select_lo, bucket_select_hi,
                                  neg_mask, base_offset, end_inst);
    }
    if (!ri) {
        ri = makeCheckShufti32x16(offset_range, bucket_idx, hi_mask, lo_mask,
                                  bucket_select_lo, bucket_select_hi,
                                  neg_mask, base_offset, end_inst);
    }
    assert(ri);
    program.add_before_end(move(ri));

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
        u32 look_idx, reach_idx;
        vector<vector<LookEntry>> lookaround;
        lookaround.emplace_back(look);
        addLookaround(bc, lookaround, look_idx, reach_idx);
        // We don't need look_idx here.
        auto ri = make_unique<RoseInstrCheckSingleLookaround>(offset, reach_idx,
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

    u32 look_idx, reach_idx;
    vector<vector<LookEntry>> lookaround;
    lookaround.emplace_back(look);
    addLookaround(bc, lookaround, look_idx, reach_idx);
    u32 look_count = verify_u32(look.size());

    auto ri = make_unique<RoseInstrCheckLookaround>(look_idx, reach_idx,
                                                    look_count,
                                                    program.end_instruction());
    program.add_before_end(move(ri));
}

#if defined(DEBUG) || defined(DUMP_SUPPORT)
static UNUSED
string dumpMultiLook(const vector<LookEntry> &looks) {
    ostringstream oss;
    for (auto it = looks.begin(); it != looks.end(); ++it) {
        if (it != looks.begin()) {
            oss << ", ";
        }
        oss << "{" << int(it->offset) << ": " << describeClass(it->reach) << "}";
    }
    return oss.str();
}
#endif

static
bool makeRoleMultipathShufti(const vector<vector<LookEntry>> &multi_look,
                             RoseProgram &program) {
    if (multi_look.empty()) {
        return false;
    }

    // find the base offset
    assert(!multi_look[0].empty());
    s32 base_offset = multi_look[0].front().offset;
    s32 last_start = base_offset;
    s32 end_offset = multi_look[0].back().offset;
    size_t multi_len = 0;

    for (const auto &look : multi_look) {
        assert(look.size() > 0);
        multi_len += look.size();

        LIMIT_TO_AT_MOST(&base_offset, look.front().offset);
        ENSURE_AT_LEAST(&last_start, look.front().offset);
        ENSURE_AT_LEAST(&end_offset, look.back().offset);
    }

    assert(last_start < 0);

    if (end_offset - base_offset >= MULTIPATH_MAX_LEN) {
        return false;
    }

    if (multi_len <= 16) {
        multi_len = 16;
    } else if (multi_len <= 32) {
        multi_len = 32;
    } else if (multi_len <= 64) {
        multi_len = 64;
    } else {
        DEBUG_PRINTF("too long for multi-path\n");
        return false;
    }

    vector<LookEntry> linear_look;
    array<u8, 64> data_select_mask;
    data_select_mask.fill(0);
    u64a hi_bits_mask = 0;
    u64a lo_bits_mask = 0;

    for (const auto &look : multi_look) {
        assert(linear_look.size() < 64);
        lo_bits_mask |= 1LLU << linear_look.size();
        for (const auto &entry : look) {
            assert(entry.offset - base_offset < MULTIPATH_MAX_LEN);
            data_select_mask[linear_look.size()] =
                                          verify_u8(entry.offset - base_offset);
            linear_look.emplace_back(verify_s8(linear_look.size()), entry.reach);
        }
        hi_bits_mask |= 1LLU << (linear_look.size() - 1);
    }

    u8 bit_index = 0; // number of buckets
    u64a neg_mask;
    array<u8, 32> hi_mask;
    array<u8, 32> lo_mask;
    array<u8, 64> bucket_select_hi;
    array<u8, 64> bucket_select_lo;
    hi_mask.fill(0);
    lo_mask.fill(0);
    bucket_select_hi.fill(0);
    bucket_select_lo.fill(0);

    if (!getShuftiMasks(linear_look, hi_mask, lo_mask, bucket_select_hi.data(),
                        bucket_select_lo.data(), neg_mask, bit_index,
                        multi_len)) {
        return false;
    }

    DEBUG_PRINTF("hi_mask %s\n",
                 convertMaskstoString(hi_mask.data(), 16).c_str());
    DEBUG_PRINTF("lo_mask %s\n",
                 convertMaskstoString(lo_mask.data(), 16).c_str());
    DEBUG_PRINTF("bucket_select_hi %s\n",
                 convertMaskstoString(bucket_select_hi.data(), 64).c_str());
    DEBUG_PRINTF("bucket_select_lo %s\n",
                 convertMaskstoString(bucket_select_lo.data(), 64).c_str());
    DEBUG_PRINTF("data_select_mask %s\n",
                 convertMaskstoString(data_select_mask.data(), 64).c_str());
    DEBUG_PRINTF("hi_bits_mask %llx\n", hi_bits_mask);
    DEBUG_PRINTF("lo_bits_mask %llx\n", lo_bits_mask);
    DEBUG_PRINTF("neg_mask %llx\n", neg_mask);
    DEBUG_PRINTF("base_offset %d\n", base_offset);
    DEBUG_PRINTF("last_start %d\n", last_start);

    // Since we don't have 16x16 now, just call 32x16 instead.
    if (bit_index > 8) {
        assert(multi_len <= 32);
        multi_len = 32;
    }

    const auto *end_inst = program.end_instruction();
    assert(multi_len == 16 || multi_len == 32 || multi_len == 64);
    if (multi_len == 16) {
        neg_mask &= 0xffff;
        assert(!(hi_bits_mask & ~0xffffULL));
        assert(!(lo_bits_mask & ~0xffffULL));
        assert(bit_index <=8);
        array<u8, 32> nib_mask;
        copy(begin(lo_mask), begin(lo_mask) + 16, nib_mask.begin());
        copy(begin(hi_mask), begin(hi_mask) + 16, nib_mask.begin() + 16);

        auto ri = make_unique<RoseInstrCheckMultipathShufti16x8>
                  (nib_mask, bucket_select_lo, data_select_mask, hi_bits_mask,
                   lo_bits_mask, neg_mask, base_offset, last_start, end_inst);
        program.add_before_end(move(ri));
    } else if (multi_len == 32) {
        neg_mask &= 0xffffffff;
        assert(!(hi_bits_mask & ~0xffffffffULL));
        assert(!(lo_bits_mask & ~0xffffffffULL));
        if (bit_index <= 8) {
            auto ri = make_unique<RoseInstrCheckMultipathShufti32x8>
                      (hi_mask, lo_mask, bucket_select_lo, data_select_mask,
                       hi_bits_mask, lo_bits_mask, neg_mask, base_offset,
                       last_start, end_inst);
            program.add_before_end(move(ri));
        } else {
            auto ri = make_unique<RoseInstrCheckMultipathShufti32x16>
                      (hi_mask, lo_mask, bucket_select_hi, bucket_select_lo,
                       data_select_mask, hi_bits_mask, lo_bits_mask, neg_mask,
                       base_offset, last_start, end_inst);
            program.add_before_end(move(ri));
        }
    } else {
        auto ri = make_unique<RoseInstrCheckMultipathShufti64>
                  (hi_mask, lo_mask, bucket_select_lo, data_select_mask,
                   hi_bits_mask, lo_bits_mask, neg_mask, base_offset,
                   last_start, end_inst);
        program.add_before_end(move(ri));
    }
    return true;
}

static
void makeRoleMultipathLookaround(build_context &bc,
                                 const vector<vector<LookEntry>> &multi_look,
                                 RoseProgram &program) {
    assert(!multi_look.empty());
    assert(multi_look.size() <= MAX_LOOKAROUND_PATHS);
    vector<vector<LookEntry>> ordered_look;
    set<s32> look_offset;

    assert(!multi_look[0].empty());
    s32 last_start = multi_look[0][0].offset;

    // build offset table.
    for (const auto &look : multi_look) {
        assert(look.size() > 0);
        last_start = max(last_start, (s32)look.begin()->offset);

        for (const auto &t : look) {
            look_offset.insert(t.offset);
        }
    }

    array<u8, MULTIPATH_MAX_LEN> start_mask;
    if (multi_look.size() < MAX_LOOKAROUND_PATHS) {
        start_mask.fill((1 << multi_look.size()) - 1);
    } else {
        start_mask.fill(0xff);
    }

    u32 path_idx = 0;
    for (const auto &look : multi_look) {
        for (const auto &t : look) {
            assert(t.offset >= (int)*look_offset.begin());
            size_t update_offset = t.offset - *look_offset.begin() + 1;
            if (update_offset < start_mask.size()) {
                start_mask[update_offset] &= ~(1 << path_idx);
            }
        }
        path_idx++;
    }

    for (u32 i = 1; i < MULTIPATH_MAX_LEN; i++) {
        start_mask[i] &= start_mask[i - 1];
        DEBUG_PRINTF("start_mask[%u] = %x\n", i, start_mask[i]);
    }

    assert(look_offset.size() <= MULTIPATH_MAX_LEN);

    assert(last_start < 0);

    for (const auto &offset : look_offset) {
        vector<LookEntry> multi_entry;
        multi_entry.resize(MAX_LOOKAROUND_PATHS);

        for (size_t i = 0; i < multi_look.size(); i++) {
            for (const auto &t : multi_look[i]) {
                if (t.offset == offset) {
                    multi_entry[i] = t;
                }
            }
        }
        ordered_look.emplace_back(multi_entry);
    }

    u32 look_idx, reach_idx;
    addLookaround(bc, ordered_look, look_idx, reach_idx);
    u32 look_count = verify_u32(ordered_look.size());

    auto ri = make_unique<RoseInstrMultipathLookaround>(look_idx, reach_idx,
                                                        look_count, last_start,
                                                        start_mask,
                                                    program.end_instruction());
    program.add_before_end(move(ri));
}

static
void makeRoleLookaround(const RoseBuildImpl &build, build_context &bc,
                        RoseVertex v, RoseProgram &program) {
    if (!build.cc.grey.roseLookaroundMasks) {
        return;
    }

    vector<vector<LookEntry>> looks;

    // Lookaround from leftfix (mandatory).
    if (contains(bc.leftfix_info, v) && bc.leftfix_info.at(v).has_lookaround) {
        DEBUG_PRINTF("using leftfix lookaround\n");
        looks = bc.leftfix_info.at(v).lookaround;
    }

    // We may be able to find more lookaround info (advisory) and merge it
    // in.
    if (looks.size() <= 1) {
        vector<LookEntry> look;
        vector<LookEntry> look_more;
        if (!looks.empty()) {
            look = move(looks.front());
        }
        findLookaroundMasks(build, v, look_more);
        mergeLookaround(look, look_more);
        if (!look.empty()) {
            makeLookaroundInstruction(bc, look, program);
        }
        return;
    }

    if (!makeRoleMultipathShufti(looks, program)) {
        assert(looks.size() <= 8);
        makeRoleMultipathLookaround(bc, looks, program);
    }
}

static
void makeRoleCheckLeftfix(const RoseBuildImpl &build,
                          const map<RoseVertex, left_build_info> &leftfix_info,
                          RoseVertex v, RoseProgram &program) {
    auto it = leftfix_info.find(v);
    if (it == end(leftfix_info)) {
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
void makeRoleAnchoredDelay(const RoseBuildImpl &build,
                           u32 floatingMinLiteralMatchOffset,
                           RoseVertex v, RoseProgram &program) {
    // Only relevant for roles that can be triggered by the anchored table.
    if (!build.isAnchored(v)) {
        return;
    }

    // If this match cannot occur after floatingMinLiteralMatchOffset, we do
    // not need this check.
    if (build.g[v].max_offset <= floatingMinLiteralMatchOffset) {
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
void makeCatchup(const RoseBuildImpl &build, bool needs_catchup,
                 const flat_set<ReportID> &reports, RoseProgram &program) {
    if (!needs_catchup) {
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
void makeCatchupMpv(const RoseBuildImpl &build, bool needs_mpv_catchup,
                    ReportID id, RoseProgram &program) {
    if (!needs_mpv_catchup) {
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
void makeReport(const RoseBuildImpl &build, const ReportID id,
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
void makeRoleReports(const RoseBuildImpl &build, const build_context &bc,
                     RoseVertex v, RoseProgram &program) {
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
    makeCatchup(build, bc.needs_catchup, reports, program);

    RoseProgram report_block;
    for (ReportID id : reports) {
        makeReport(build, id, has_som, report_block);
    }
    program.add_before_end(move(report_block));
}

static
void makeRoleSuffix(const RoseBuildImpl &build, const build_context &bc,
                    RoseVertex v, RoseProgram &program) {
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
void makeRoleGroups(const RoseBuildImpl &build, ProgramBuild &prog_build,
                    RoseVertex v, RoseProgram &program) {
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
        already_on &= prog_build.vertex_group_map.at(u);
    }

    DEBUG_PRINTF("already_on=0x%llx\n", already_on);
    DEBUG_PRINTF("squashable=0x%llx\n", prog_build.squashable_groups);
    DEBUG_PRINTF("groups=0x%llx\n", groups);

    already_on &= ~prog_build.squashable_groups;
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
void makeRoleInfixTriggers(const RoseBuildImpl &build, const build_context &bc,
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
    sort_and_unique(infix_program, [](const RoseInstrTriggerInfix &a,
                                      const RoseInstrTriggerInfix &b) {
        return tie(a.cancel, a.queue, a.event) <
               tie(b.cancel, b.queue, b.event);
    });
    for (const auto &ri : infix_program) {
        program.add_before_end(make_unique<RoseInstrTriggerInfix>(ri));
    }
}

static
void makeRoleSetState(const unordered_map<RoseVertex, u32> &roleStateIndices,
                      RoseVertex v, RoseProgram &program) {
    // We only need this instruction if a state index has been assigned to this
    // vertex.
    auto it = roleStateIndices.find(v);
    if (it == end(roleStateIndices)) {
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
void makeRoleCheckNotHandled(ProgramBuild &prog_build, RoseVertex v,
                             RoseProgram &program) {
    u32 handled_key;
    if (contains(prog_build.handledKeys, v)) {
        handled_key = prog_build.handledKeys.at(v);
    } else {
        handled_key = verify_u32(prog_build.handledKeys.size());
        prog_build.handledKeys.emplace(v, handled_key);
    }

    const auto *end_inst = program.end_instruction();
    auto ri = make_unique<RoseInstrCheckNotHandled>(handled_key, end_inst);
    program.add_before_end(move(ri));
}

static
void makeRoleEagerEodReports(const RoseBuildImpl &build, build_context &bc,
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
RoseProgram makeProgram(const RoseBuildImpl &build, build_context &bc,
                        ProgramBuild &prog_build, const RoseEdge &e) {
    const RoseGraph &g = build.g;
    auto v = target(e, g);

    RoseProgram program;

    // First, add program instructions that enforce preconditions without
    // effects.

    makeRoleAnchoredDelay(build, prog_build.floatingMinLiteralMatchOffset, v,
                          program);

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
        makeRoleCheckNotHandled(prog_build, v, program);
    }

    makeRoleLookaround(build, bc, v, program);
    makeRoleCheckLeftfix(build, bc.leftfix_info, v, program);

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
    makeRoleGroups(build, prog_build, v, groups_block);
    effects_block.add_block(move(groups_block));

    RoseProgram suffix_block;
    makeRoleSuffix(build, bc, v, suffix_block);
    effects_block.add_block(move(suffix_block));

    RoseProgram state_block;
    makeRoleSetState(bc.roleStateIndices, v, state_block);
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
u32 writeBoundaryProgram(const RoseBuildImpl &build, build_context &bc,
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
void makeBoundaryPrograms(const RoseBuildImpl &build, build_context &bc,
                          const BoundaryReports &boundary,
                          const DerivedBoundaryReports &dboundary,
                          RoseBoundaryReports &out) {
    DEBUG_PRINTF("report ^:  %zu\n", boundary.report_at_0.size());
    DEBUG_PRINTF("report $:  %zu\n", boundary.report_at_eod.size());
    DEBUG_PRINTF("report ^$: %zu\n", dboundary.report_at_0_eod_full.size());

    out.reportEodOffset =
        writeBoundaryProgram(build, bc, boundary.report_at_eod);
    out.reportZeroOffset =
        writeBoundaryProgram(build, bc, boundary.report_at_0);
    out.reportZeroEodOffset =
        writeBoundaryProgram(build, bc, dboundary.report_at_0_eod_full);
}

static
unordered_map<RoseVertex, u32> assignStateIndices(const RoseBuildImpl &build) {
    const auto &g = build.g;

    u32 state = 0;
    unordered_map<RoseVertex, u32> roleStateIndices;
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
        roleStateIndices.emplace(v, state++);
    }

    DEBUG_PRINTF("assigned %u states (from %zu vertices)\n", state,
                 num_vertices(g));

    return roleStateIndices;
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
                left.stopTable = bc.engine_blob.add_range(lbi.stopAlphabet);
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
void addPredBlocksAny(map<u32, RoseProgram> &pred_blocks, u32 num_states,
                      RoseProgram &program) {
    RoseProgram sparse_program;

    vector<u32> keys;
    for (const u32 &key : pred_blocks | map_keys) {
        keys.push_back(key);
    }

    const RoseInstruction *end_inst = sparse_program.end_instruction();
    auto ri = make_unique<RoseInstrSparseIterAny>(num_states, keys, end_inst);
    sparse_program.add_before_end(move(ri));

    RoseProgram &block = pred_blocks.begin()->second;
    sparse_program.add_before_end(move(block));
    program.add_block(move(sparse_program));
}

static
void addPredBlocksMulti(map<u32, RoseProgram> &pred_blocks,
                        u32 num_states, RoseProgram &program) {
    assert(!pred_blocks.empty());

    RoseProgram sparse_program;
    const RoseInstruction *end_inst = sparse_program.end_instruction();
    vector<pair<u32, const RoseInstruction *>> jump_table;

    // BEGIN instruction.
    auto ri_begin = make_unique<RoseInstrSparseIterBegin>(num_states, end_inst);
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
void addPredBlocks(map<u32, RoseProgram> &pred_blocks, u32 num_states,
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
        addPredBlocksAny(pred_blocks, num_states, program);
        return;
    }

    addPredBlocksMulti(pred_blocks, num_states, program);
}

static
void makePushDelayedInstructions(const RoseBuildImpl &build,
                                 ProgramBuild &prog_build, u32 lit_id,
                                 RoseProgram &program) {
    const auto &info = build.literal_info.at(lit_id);

    vector<RoseInstrPushDelayed> delay_instructions;

    for (const auto &delayed_lit_id : info.delayed_ids) {
        DEBUG_PRINTF("delayed lit id %u\n", delayed_lit_id);
        assert(contains(prog_build.delay_programs, delayed_lit_id));
        u32 delay_id = prog_build.delay_programs.at(delayed_lit_id);
        const auto &delay_lit = build.literals.right.at(delayed_lit_id);
        delay_instructions.emplace_back(verify_u8(delay_lit.delay), delay_id);
    }

    sort_and_unique(delay_instructions, [](const RoseInstrPushDelayed &a,
                                           const RoseInstrPushDelayed &b) {
        return tie(a.delay, a.index) < tie(b.delay, b.index);
    });

    for (const auto &ri : delay_instructions) {
        program.add_before_end(make_unique<RoseInstrPushDelayed>(ri));
    }
}

static
void makeGroupCheckInstruction(const RoseBuildImpl &build, u32 lit_id,
                               RoseProgram &program) {
    const auto &info = build.literal_info.at(lit_id);
    rose_group groups = info.group_mask;
    if (!groups) {
        return;
    }
    program.add_before_end(make_unique<RoseInstrCheckGroups>(groups));
}

static
void makeCheckLitMaskInstruction(const RoseBuildImpl &build, build_context &bc,
                                 u32 lit_id, RoseProgram &program) {
    const auto &info = build.literal_info.at(lit_id);
    if (!info.requires_benefits) {
        return;
    }

    vector<LookEntry> look;

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
                                u32 lit_id,
                                RoseProgram &program) {
    const auto &info = build.literal_info.at(lit_id);
    if (!info.squash_group) {
        return;
    }

    rose_group groups = info.group_mask;
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
                                   ProgramBuild &prog_build, u32 lit_id,
                                   RoseProgram &program) {
    if (build.literals.right.at(lit_id).table != ROSE_ANCHORED) {
        return;
    }
    if (!contains(prog_build.anchored_programs, lit_id)) {
        return;
    }
    auto anch_id = prog_build.anchored_programs.at(lit_id);
    DEBUG_PRINTF("adding RECORD_ANCHORED for anch_id=%u\n", anch_id);
    program.add_before_end(make_unique<RoseInstrRecordAnchored>(anch_id));
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
void makeCheckLitEarlyInstruction(const RoseBuildImpl &build, u32 lit_id,
                                  const vector<RoseEdge> &lit_edges,
                                  u32 floatingMinLiteralMatchOffset,
                                  RoseProgram &program) {
    if (lit_edges.empty()) {
        return;
    }

    if (floatingMinLiteralMatchOffset == 0) {
        return;
    }

    RoseVertex v = target(lit_edges.front(), build.g);
    if (!build.isFloating(v)) {
        return;
    }

    const auto &lit = build.literals.right.at(lit_id);
    size_t min_len = lit.elength();
    u32 min_offset = findMinOffset(build, lit_id);
    DEBUG_PRINTF("has min_len=%zu, min_offset=%u, global min is %u\n", min_len,
                 min_offset, floatingMinLiteralMatchOffset);

    // If we can't match before the min offset, we don't need the check.
    if (min_len >= floatingMinLiteralMatchOffset) {
        DEBUG_PRINTF("no need for check, min is %u\n",
                     floatingMinLiteralMatchOffset);
        return;
    }

    assert(min_offset >= floatingMinLiteralMatchOffset);
    assert(min_offset < UINT32_MAX);

    DEBUG_PRINTF("adding lit early check, min_offset=%u\n", min_offset);
    const auto *end_inst = program.end_instruction();
    program.add_before_end(
        make_unique<RoseInstrCheckLitEarly>(min_offset, end_inst));
}

static
void makeCheckLiteralInstruction(const RoseBuildImpl &build,  u32 lit_id,
                                 size_t longLitLengthThreshold,
                                 RoseProgram &program) {
    assert(longLitLengthThreshold > 0);

    DEBUG_PRINTF("lit_id=%u, long lit threshold %zu\n", lit_id,
                 longLitLengthThreshold);

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

    if (lit.s.length() <= longLitLengthThreshold) {
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
bool hasDelayedLiteral(const RoseBuildImpl &build,
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
RoseProgram buildLitInitialProgram(const RoseBuildImpl &build,
                                   build_context &bc, ProgramBuild &prog_build,
                                   u32 lit_id,
                                   const vector<RoseEdge> &lit_edges) {
    RoseProgram program;

    // Check long literal info.
    makeCheckLiteralInstruction(build, lit_id, bc.longLitLengthThreshold,
                                program);

    // Check lit mask.
    makeCheckLitMaskInstruction(build, bc, lit_id, program);

    // Check literal groups. This is an optimisation that we only perform for
    // delayed literals, as their groups may be switched off; ordinarily, we
    // can trust the HWLM matcher.
    if (hasDelayedLiteral(build, lit_edges)) {
        makeGroupCheckInstruction(build, lit_id, program);
    }

    // Add instructions for pushing delayed matches, if there are any.
    makePushDelayedInstructions(build, prog_build, lit_id, program);

    // Add pre-check for early literals in the floating table.
    makeCheckLitEarlyInstruction(build, lit_id, lit_edges,
                                 prog_build.floatingMinLiteralMatchOffset,
                                 program);

    return program;
}

static
RoseProgram buildLiteralProgram(const RoseBuildImpl &build, build_context &bc,
                                ProgramBuild &prog_build, u32 lit_id,
                                const vector<RoseEdge> &lit_edges,
                                bool is_anchored_program) {
    const auto &g = build.g;

    DEBUG_PRINTF("lit id=%u, %zu lit edges\n", lit_id, lit_edges.size());

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
        pred_blocks[pred_state].add_block(
            makeProgram(build, bc, prog_build, e));
    }

    // Add blocks to deal with non-root edges (triggered by sparse iterator or
    // mmbit_isset checks).
    addPredBlocks(pred_blocks, bc.roleStateIndices.size(), program);

    // Add blocks to handle root roles.
    for (const auto &e : lit_edges) {
        const auto &u = source(e, g);
        if (!build.isAnyStart(u)) {
            continue;
        }
        DEBUG_PRINTF("root edge (%zu,%zu)\n", g[u].index,
                     g[target(e, g)].index);
        program.add_block(makeProgram(build, bc, prog_build, e));
    }

    if (lit_id == build.eod_event_literal_id) {
        assert(build.eod_event_literal_id != MO_INVALID_IDX);
        return program;
    }

    RoseProgram root_block;

    // Literal may squash groups.
    makeGroupSquashInstruction(build, lit_id, root_block);

    // Literal may be anchored and need to be recorded.
    if (!is_anchored_program) {
        makeRecordAnchoredInstruction(build, prog_build, lit_id, root_block);
    }

    program.add_block(move(root_block));

    // Construct initial program up front, as its early checks must be able
    // to jump to end and terminate processing for this literal.
    auto lit_program =
        buildLitInitialProgram(build, bc, prog_build, lit_id, lit_edges);
    lit_program.add_before_end(move(program));

    return lit_program;
}

/**
 * \brief Consumes list of program blocks, checks them for duplicates and then
 * concatenates them into one program.
 */
static
RoseProgram assembleProgramBlocks(vector<RoseProgram> &&blocks) {
    RoseProgram program;

    DEBUG_PRINTF("%zu blocks before dedupe\n", blocks.size());

    sort(blocks.begin(), blocks.end(),
         [](const RoseProgram &a, const RoseProgram &b) {
             RoseProgramHash hasher;
             return hasher(a) < hasher(b);
         });

    blocks.erase(unique(blocks.begin(), blocks.end(), RoseProgramEquivalence()),
                 blocks.end());

    DEBUG_PRINTF("%zu blocks after dedupe\n", blocks.size());

    for (auto &prog : blocks) {
        program.add_block(move(prog));
    }

    return program;
}

static
u32 writeLiteralProgram(const RoseBuildImpl &build, build_context &bc,
                        ProgramBuild &prog_build, const vector<u32> &lit_ids,
                        const map<u32, vector<RoseEdge>> &lit_edge_map,
                        bool is_anchored_program) {
    assert(!lit_ids.empty());

    // If we have multiple literals and any of them squash groups, we will have
    // to add a CLEAR_WORK_DONE instruction to each literal program block to
    // clear the work_done flags so that it's only set if a state has been
    // switched on for that literal.

    // Note that we add it to every lit program, as they may be
    // reordered/uniquified by assembleProgramBlocks() above.
    const bool needs_clear_work = lit_ids.size() > 1 &&
        any_of_in(lit_ids, [&](u32 lit_id) {
            return build.literal_info.at(lit_id).squash_group;
        });

    vector<RoseProgram> blocks;

    const vector<RoseEdge> no_edges;

    for (const auto &lit_id : lit_ids) {
        DEBUG_PRINTF("lit_id=%u\n", lit_id);
        const vector<RoseEdge> *edges_ptr;
        if (contains(lit_edge_map, lit_id)) {
            edges_ptr = &lit_edge_map.at(lit_id);
        } else {
            edges_ptr = &no_edges;
        }
        auto prog = buildLiteralProgram(build, bc, prog_build, lit_id,
                                        *edges_ptr, is_anchored_program);
        if (needs_clear_work) {
            RoseProgram clear_block;
            clear_block.add_before_end(make_unique<RoseInstrClearWorkDone>());
            prog.add_block(move(clear_block));
        }
        blocks.push_back(move(prog));
    }

    auto program = assembleProgramBlocks(move(blocks));

    if (program.empty()) {
        return 0;
    }
    applyFinalSpecialisation(program);
    return writeProgram(bc, move(program));
}

static
u32 writeDelayRebuildProgram(const RoseBuildImpl &build, build_context &bc,
                             ProgramBuild &prog_build,
                             const vector<u32> &lit_ids) {
    assert(!lit_ids.empty());

    if (!build.cc.streaming) {
        return 0; // We only do delayed rebuild in streaming mode.
    }

    vector<RoseProgram> blocks;

    for (const auto &lit_id : lit_ids) {
        DEBUG_PRINTF("lit_id=%u\n", lit_id);
        const auto &info = build.literal_info.at(lit_id);
        if (info.delayed_ids.empty()) {
            continue; // No delayed IDs, no work to do.
        }

        RoseProgram prog;
        makeCheckLiteralInstruction(build, lit_id, bc.longLitLengthThreshold,
                                    prog);
        makeCheckLitMaskInstruction(build, bc, lit_id, prog);
        makePushDelayedInstructions(build, prog_build, lit_id, prog);
        blocks.push_back(move(prog));
    }

    auto program = assembleProgramBlocks(move(blocks));

    if (program.empty()) {
        return 0;
    }
    applyFinalSpecialisation(program);
    return writeProgram(bc, move(program));
}

/**
 * \brief Returns a map from literal ID to a list of edges leading into
 * vertices with that literal ID.
 */
static
map<u32, vector<RoseEdge>> findEdgesByLiteral(const RoseBuildImpl &build) {
    // Use a set of edges while building the map to cull duplicates.
    map<u32, flat_set<RoseEdge>> unique_lit_edge_map;

    const auto &g = build.g;
    for (const auto &e : edges_range(g)) {
        const auto &v = target(e, g);
        for (const auto &lit_id : g[v].literals) {
            unique_lit_edge_map[lit_id].insert(e);
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
        lit_edge_map.emplace(m.first, std::move(edge_list));
    }

    return lit_edge_map;
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
vector<LitFragment> groupByFragment(const RoseBuildImpl &build) {
    vector<LitFragment> fragments;
    u32 frag_id = 0;

    struct FragmentInfo {
        vector<u32> lit_ids;
        rose_group groups = 0;
    };

    map<rose_literal_id, FragmentInfo> frag_info;

    for (const auto &m : build.literals.right) {
        const u32 lit_id = m.first;
        const auto &lit = m.second;
        const auto &info = build.literal_info.at(lit_id);

        if (!isUsedLiteral(build, lit_id)) {
            DEBUG_PRINTF("lit %u is unused\n", lit_id);
            continue;
        }

        if (lit.table == ROSE_EVENT) {
            DEBUG_PRINTF("lit %u is an event\n", lit_id);
            continue;
        }

        auto groups = info.group_mask;

        if (lit.s.length() < ROSE_SHORT_LITERAL_LEN_MAX) {
            fragments.emplace_back(frag_id, groups, lit_id);
            frag_id++;
            continue;
        }

        DEBUG_PRINTF("fragment candidate: lit_id=%u %s\n", lit_id,
                     dumpString(lit.s).c_str());
        auto &fi = frag_info[getFragment(lit)];
        fi.lit_ids.push_back(lit_id);
        fi.groups |= groups;
    }

    for (auto &m : frag_info) {
        auto &fi = m.second;
        DEBUG_PRINTF("frag %s -> ids: %s\n", dumpString(m.first.s).c_str(),
                     as_string_list(fi.lit_ids).c_str());
        sort(fi.lit_ids.begin(), fi.lit_ids.end()); /* to match old behaviour */
        fragments.emplace_back(frag_id, fi.groups, move(fi.lit_ids));
        frag_id++;
        assert(frag_id == fragments.size());
    }

    return fragments;
}

/**
 * \brief Build the interpreter programs for each literal.
 */
static
void buildLiteralPrograms(const RoseBuildImpl &build,
                          vector<LitFragment> &fragments, build_context &bc,
                          ProgramBuild &prog_build) {
    DEBUG_PRINTF("%zu fragments\n", fragments.size());
    auto lit_edge_map = findEdgesByLiteral(build);

    for (auto &frag : fragments) {
        DEBUG_PRINTF("frag_id=%u, lit_ids=[%s]\n", frag.fragment_id,
                     as_string_list(frag.lit_ids).c_str());

        frag.lit_program_offset
            = writeLiteralProgram(build, bc, prog_build, frag.lit_ids,
                                  lit_edge_map, false);
        frag.delay_program_offset
            = writeDelayRebuildProgram(build, bc, prog_build, frag.lit_ids);
    }
}

/**
 * \brief Write delay replay programs to the bytecode.
 *
 * Returns the offset of the beginning of the program array, and the number of
 * programs.
 */
static
pair<u32, u32> writeDelayPrograms(const RoseBuildImpl &build,
                                  const vector<LitFragment> &fragments,
                                  build_context &bc,
                                  ProgramBuild &prog_build) {
    auto lit_edge_map = findEdgesByLiteral(build);

    vector<u32> programs; // program offsets indexed by (delayed) lit id
    unordered_map<u32, u32> cache; // program offsets we have already seen

    for (const auto &frag : fragments) {
        for (const u32 lit_id : frag.lit_ids) {
            const auto &info = build.literal_info.at(lit_id);

            for (const auto &delayed_lit_id : info.delayed_ids) {
                DEBUG_PRINTF("lit id %u delay id %u\n", lit_id, delayed_lit_id);
                u32 offset = writeLiteralProgram(build, bc, prog_build,
                                                 {delayed_lit_id}, lit_edge_map,
                                                 false);

                u32 delay_id;
                auto it = cache.find(offset);
                if (it != end(cache)) {
                    delay_id = it->second;
                    DEBUG_PRINTF("reusing delay_id %u for offset %u\n",
                                 delay_id, offset);
                } else {
                    delay_id = verify_u32(programs.size());
                    programs.push_back(offset);
                    cache.emplace(offset, delay_id);
                    DEBUG_PRINTF("assigned new delay_id %u for offset %u\n",
                                 delay_id, offset);
                }
                prog_build.delay_programs.emplace(delayed_lit_id, delay_id);
            }
        }
    }

    DEBUG_PRINTF("%zu delay programs\n", programs.size());
    return {bc.engine_blob.add_range(programs), verify_u32(programs.size())};
}

/**
 * \brief Write anchored replay programs to the bytecode.
 *
 * Returns the offset of the beginning of the program array, and the number of
 * programs.
 */
static
pair<u32, u32> writeAnchoredPrograms(const RoseBuildImpl &build,
                                     const vector<LitFragment> &fragments,
                                     build_context &bc,
                                     ProgramBuild &prog_build) {
    auto lit_edge_map = findEdgesByLiteral(build);

    vector<u32> programs; // program offsets indexed by anchored id
    unordered_map<u32, u32> cache; // program offsets we have already seen

    for (const auto &frag : fragments) {
        for (const u32 lit_id : frag.lit_ids) {
            const auto &lit = build.literals.right.at(lit_id);

            if (lit.table != ROSE_ANCHORED) {
                continue;
            }

            // If this anchored literal can never match past
            // floatingMinLiteralMatchOffset, we will never have to record it.
            if (findMaxOffset(build, lit_id)
                <= prog_build.floatingMinLiteralMatchOffset) {
                DEBUG_PRINTF("can never match after "
                             "floatingMinLiteralMatchOffset=%u\n",
                             prog_build.floatingMinLiteralMatchOffset);
                continue;
            }

            u32 offset = writeLiteralProgram(build, bc, prog_build, {lit_id},
                                             lit_edge_map, true);
            DEBUG_PRINTF("lit_id=%u -> anch prog at %u\n", lit_id, offset);

            u32 anch_id;
            auto it = cache.find(offset);
            if (it != end(cache)) {
                anch_id = it->second;
                DEBUG_PRINTF("reusing anch_id %u for offset %u\n", anch_id, offset);
            } else {
                anch_id = verify_u32(programs.size());
                programs.push_back(offset);
                cache.emplace(offset, anch_id);
                DEBUG_PRINTF("assigned new anch_id %u for offset %u\n", anch_id,
                             offset);
            }
            prog_build.anchored_programs.emplace(lit_id, anch_id);
        }
    }

    DEBUG_PRINTF("%zu anchored programs\n", programs.size());
    return {bc.engine_blob.add_range(programs), verify_u32(programs.size())};
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
pair<u32, u32> buildReportPrograms(const RoseBuildImpl &build,
                                   build_context &bc) {
    const auto reports = findEngineReports(build);
    vector<u32> programs;
    programs.reserve(reports.size());

    for (ReportID id : reports) {
        RoseProgram program;
        const bool has_som = false;
        makeCatchupMpv(build, bc.needs_mpv_catchup, id, program);
        makeReport(build, id, has_som, program);
        applyFinalSpecialisation(program);
        u32 offset = writeProgram(bc, move(program));
        programs.push_back(offset);
        build.rm.setProgramOffset(id, offset);
        DEBUG_PRINTF("program for report %u @ %u (%zu instructions)\n", id,
                     programs.back(), program.size());
    }

    u32 offset = bc.engine_blob.add_range(programs);
    u32 count = verify_u32(programs.size());
    return {offset, count};
}

static
RoseProgram makeEodAnchorProgram(const RoseBuildImpl &build,
                                 bool needs_catchup,
                                 ProgramBuild &prog_build, const RoseEdge &e,
                                 const bool multiple_preds) {
    const RoseGraph &g = build.g;
    const RoseVertex v = target(e, g);

    RoseProgram program;

    if (g[e].history == ROSE_ROLE_HISTORY_ANCH) {
        makeRoleCheckBounds(build, v, e, program);
    }

    if (multiple_preds) {
        // Only necessary when there is more than one pred.
        makeRoleCheckNotHandled(prog_build, v, program);
    }

    const auto &reports = g[v].reports;
    makeCatchup(build, needs_catchup, reports, program);

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
void addEodAnchorProgram(const RoseBuildImpl &build, const build_context &bc,
                         ProgramBuild &prog_build, bool in_etable,
                         RoseProgram &program) {
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
                makeEodAnchorProgram(build, bc.needs_catchup, prog_build, e,
                                     multiple_preds));
        }
    }

    addPredBlocks(pred_blocks, bc.roleStateIndices.size(), program);
}

static
void addEodEventProgram(const RoseBuildImpl &build, build_context &bc,
                        ProgramBuild &prog_build, RoseProgram &program) {
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

    program.add_block(buildLiteralProgram(
        build, bc, prog_build, build.eod_event_literal_id, edge_list, false));
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
u32 writeEodProgram(const RoseBuildImpl &build, build_context &bc,
                    ProgramBuild &prog_build, u32 eodNfaIterOffset) {
    RoseProgram program;

    addEodEventProgram(build, bc, prog_build, program);
    addEnginesEodProgram(eodNfaIterOffset, program);
    addEodAnchorProgram(build, bc, prog_build, false, program);
    addMatcherEodProgram(build, program);
    addEodAnchorProgram(build, bc, prog_build, true, program);
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
u32 writeEagerQueueIter(const set<u32> &eager, u32 leftfixBeginQueue,
                        u32 queue_count, RoseEngineBlob &engine_blob) {
    if (eager.empty()) {
        return 0;
    }

    vector<u32> vec;
    for (u32 q : eager) {
        assert(q >= leftfixBeginQueue);
        vec.push_back(q - leftfixBeginQueue);
    }

    auto iter = mmbBuildSparseIterator(vec, queue_count - leftfixBeginQueue);
    return engine_blob.add_iterator(iter);
}

static
aligned_unique_ptr<RoseEngine> addSmallWriteEngine(const RoseBuildImpl &build,
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
    const size_t smallWriteSize = smwr_engine.size();
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

static
map<left_id, u32> makeLeftQueueMap(const RoseGraph &g,
                         const map<RoseVertex, left_build_info> &leftfix_info) {
    map<left_id, u32> lqm;
    for (const auto &e : leftfix_info) {
        if (e.second.has_lookaround) {
            continue;
        }
        DEBUG_PRINTF("%zu: using queue %u\n", g[e.first].index, e.second.queue);
        assert(e.second.queue != INVALID_QUEUE);
        left_id left(g[e.first].left);
        assert(!contains(lqm, left) || lqm[left] == e.second.queue);
        lqm[left] = e.second.queue;
    }

    return lqm;
}

aligned_unique_ptr<RoseEngine> RoseBuildImpl::buildFinalEngine(u32 minWidth) {
    // We keep all our offsets, counts etc. in a prototype RoseEngine which we
    // will copy into the real one once it is allocated: we can't do this
    // until we know how big it will be.
    RoseEngine proto;
    memset(&proto, 0, sizeof(proto));

    // Set scanning mode.
    if (!cc.streaming) {
        proto.mode = HS_MODE_BLOCK;
    } else if (cc.vectored) {
        proto.mode = HS_MODE_VECTORED;
    } else {
        proto.mode = HS_MODE_STREAM;
    }

    DerivedBoundaryReports dboundary(boundary);

    size_t historyRequired = calcHistoryRequired(); // Updated by HWLM.
    size_t longLitLengthThreshold = calcLongLitThreshold(*this,
                                                         historyRequired);
    DEBUG_PRINTF("longLitLengthThreshold=%zu\n", longLitLengthThreshold);

    vector<LitFragment> fragments = groupByFragment(*this);

    auto anchored_dfas = buildAnchoredDfas(*this, fragments);

    build_context bc;
    u32 floatingMinLiteralMatchOffset
        = findMinFloatingLiteralMatch(*this, anchored_dfas);
    bc.longLitLengthThreshold = longLitLengthThreshold;
    bc.needs_catchup = needsCatchup(*this, anchored_dfas);
    recordResources(bc.resources, *this, fragments);
    if (!anchored_dfas.empty()) {
        bc.resources.has_anchored = true;
    }
    bc.needs_mpv_catchup = needsMpvCatchup(*this);

    makeBoundaryPrograms(*this, bc, boundary, dboundary, proto.boundary);

    tie(proto.reportProgramOffset, proto.reportProgramCount) =
        buildReportPrograms(*this, bc);

    // Build NFAs
    bool mpv_as_outfix;
    prepMpv(*this, bc, &historyRequired, &mpv_as_outfix);
    proto.outfixBeginQueue = qif.allocated_count();
    if (!prepOutfixes(*this, bc, &historyRequired)) {
        return nullptr;
    }
    proto.outfixEndQueue = qif.allocated_count();
    proto.leftfixBeginQueue = proto.outfixEndQueue;

    set<u32> no_retrigger_queues;
    set<u32> eager_queues;

    /* Note: buildNfas may reduce the lag for vertices that have prefixes */
    if (!buildNfas(*this, bc, qif, &no_retrigger_queues, &eager_queues,
                   &proto.leftfixBeginQueue)) {
        return nullptr;
    }
    u32 eodNfaIterOffset = buildEodNfaIterator(bc, proto.leftfixBeginQueue);
    buildCountingMiracles(bc);

    u32 queue_count = qif.allocated_count(); /* excludes anchored matcher q;
                                              * som rev nfas */
    if (queue_count > cc.grey.limitRoseEngineCount) {
        throw ResourceLimitError();
    }

    // Enforce role table resource limit.
    if (num_vertices(g) > cc.grey.limitRoseRoleCount) {
        throw ResourceLimitError();
    }

    bc.roleStateIndices = assignStateIndices(*this);

    u32 laggedRoseCount = 0;
    vector<LeftNfaInfo> leftInfoTable;
    buildLeftInfoTable(*this, bc, eager_queues, proto.leftfixBeginQueue,
                       queue_count - proto.leftfixBeginQueue, leftInfoTable,
                       &laggedRoseCount, &historyRequired);

    // Information only needed for program construction.
    ProgramBuild prog_build(floatingMinLiteralMatchOffset);
    prog_build.vertex_group_map = getVertexGroupMap(*this);
    prog_build.squashable_groups = getSquashableGroups(*this);

    tie(proto.anchoredProgramOffset, proto.anchored_count) =
        writeAnchoredPrograms(*this, fragments, bc, prog_build);

    tie(proto.delayProgramOffset, proto.delay_count) =
        writeDelayPrograms(*this, fragments, bc, prog_build);

    buildLiteralPrograms(*this, fragments, bc, prog_build);

    proto.eodProgramOffset =
        writeEodProgram(*this, bc, prog_build, eodNfaIterOffset);

    size_t longLitStreamStateRequired = 0;
    proto.longLitTableOffset = buildLongLiteralTable(*this, bc.engine_blob,
                bc.longLiterals, longLitLengthThreshold, &historyRequired,
                &longLitStreamStateRequired);

    proto.lastByteHistoryIterOffset = buildLastByteIter(g, bc);
    proto.eagerIterOffset = writeEagerQueueIter(
        eager_queues, proto.leftfixBeginQueue, queue_count, bc.engine_blob);

    addSomRevNfas(bc, proto, ssm);

    writeLookaroundTables(bc, proto);
    writeDkeyInfo(rm, bc.engine_blob, proto);
    writeLeftInfo(bc.engine_blob, proto, leftInfoTable);

    // Build anchored matcher.
    size_t asize = 0;
    auto atable = buildAnchoredMatcher(*this, fragments, anchored_dfas, &asize);
    if (atable) {
        proto.amatcherOffset = bc.engine_blob.add(atable.get(), asize, 64);
    }

    // Build floating HWLM matcher.
    rose_group fgroups = 0;
    size_t fsize = 0;
    auto ftable = buildFloatingMatcher(*this, fragments,
                                       bc.longLitLengthThreshold,
                                       &fgroups, &fsize, &historyRequired);
    if (ftable) {
        proto.fmatcherOffset = bc.engine_blob.add(ftable.get(), fsize, 64);
        bc.resources.has_floating = true;
    }

    // Build delay rebuild HWLM matcher.
    size_t drsize = 0;
    auto drtable = buildDelayRebuildMatcher(*this, fragments,
                                            bc.longLitLengthThreshold, &drsize);
    if (drtable) {
        proto.drmatcherOffset = bc.engine_blob.add(drtable.get(), drsize, 64);
    }

    // Build EOD-anchored HWLM matcher.
    size_t esize = 0;
    auto etable = buildEodAnchoredMatcher(*this, fragments, &esize);
    if (etable) {
        proto.ematcherOffset = bc.engine_blob.add(etable.get(), esize, 64);
    }

    // Build small-block HWLM matcher.
    size_t sbsize = 0;
    auto sbtable = buildSmallBlockMatcher(*this, fragments, &sbsize);
    if (sbtable) {
        proto.sbmatcherOffset = bc.engine_blob.add(sbtable.get(), sbsize, 64);
    }

    proto.activeArrayCount = proto.leftfixBeginQueue;

    proto.anchorStateSize = atable ? anchoredStateSize(*atable) : 0;

    DEBUG_PRINTF("rose history required %zu\n", historyRequired);
    assert(!cc.streaming || historyRequired <= cc.grey.maxHistoryAvailable);

    // Some SOM schemes (reverse NFAs, for example) may require more history.
    historyRequired = max(historyRequired, (size_t)ssm.somHistoryRequired());

    assert(!cc.streaming || historyRequired <=
           max(cc.grey.maxHistoryAvailable, cc.grey.somMaxRevNfaLength));

    fillStateOffsets(*this, bc.roleStateIndices.size(), proto.anchorStateSize,
                     proto.activeArrayCount, proto.activeLeftCount,
                     laggedRoseCount, longLitStreamStateRequired,
                     historyRequired, &proto.stateOffsets);

    // Write in NfaInfo structures. This will also update state size
    // information in proto.
    writeNfaInfo(*this, bc, proto, no_retrigger_queues);

    scatter_plan_raw state_scatter = buildStateScatterPlan(
        sizeof(u8), bc.roleStateIndices.size(), proto.activeLeftCount,
        proto.rosePrefixCount, proto.stateOffsets, cc.streaming,
        proto.activeArrayCount, proto.outfixBeginQueue, proto.outfixEndQueue);

    u32 currOffset;  /* relative to base of RoseEngine */
    if (!bc.engine_blob.empty()) {
        currOffset = bc.engine_blob.base_offset + bc.engine_blob.size();
    } else {
        currOffset = sizeof(RoseEngine);
    }

    currOffset = ROUNDUP_CL(currOffset);
    DEBUG_PRINTF("currOffset %u\n", currOffset);

    currOffset = ROUNDUP_N(currOffset, alignof(scatter_unit_u64a));
    u32 state_scatter_aux_offset = currOffset;
    currOffset += aux_size(state_scatter);

    proto.historyRequired = verify_u32(historyRequired);
    proto.ekeyCount = rm.numEkeys();

    proto.somHorizon = ssm.somPrecision();
    proto.somLocationCount = ssm.numSomSlots();
    proto.somLocationFatbitSize = fatbit_size(proto.somLocationCount);

    proto.needsCatchup = bc.needs_catchup ? 1 : 0;

    proto.runtimeImpl = pickRuntimeImpl(*this, bc.resources,
                                        proto.outfixEndQueue);
    proto.mpvTriggeredByLeaf = anyEndfixMpvTriggers(*this);

    proto.queueCount = queue_count;
    proto.activeQueueArraySize = fatbit_size(queue_count);
    proto.handledKeyCount = prog_build.handledKeys.size();
    proto.handledKeyFatbitSize = fatbit_size(proto.handledKeyCount);

    proto.rolesWithStateCount = bc.roleStateIndices.size();

    proto.initMpvNfa = mpv_as_outfix ? 0 : MO_INVALID_IDX;
    proto.stateSize = mmbit_size(bc.roleStateIndices.size());

    proto.delay_fatbit_size = fatbit_size(proto.delay_count);
    proto.anchored_fatbit_size = fatbit_size(proto.anchored_count);

    // The Small Write matcher is (conditionally) added to the RoseEngine in
    // another pass by the caller. Set to zero (meaning no SMWR engine) for
    // now.
    proto.smallWriteOffset = 0;

    proto.amatcherMinWidth = findMinWidth(*this, ROSE_ANCHORED);
    proto.fmatcherMinWidth = findMinWidth(*this, ROSE_FLOATING);
    proto.eodmatcherMinWidth = findMinWidth(*this, ROSE_EOD_ANCHORED);
    proto.amatcherMaxBiAnchoredWidth = findMaxBAWidth(*this, ROSE_ANCHORED);
    proto.fmatcherMaxBiAnchoredWidth = findMaxBAWidth(*this, ROSE_FLOATING);
    proto.minWidth = hasBoundaryReports(boundary) ? 0 : minWidth;
    proto.minWidthExcludingBoundaries = minWidth;
    proto.floatingMinLiteralMatchOffset = floatingMinLiteralMatchOffset;

    proto.maxBiAnchoredWidth = findMaxBAWidth(*this);
    proto.noFloatingRoots = hasNoFloatingRoots();
    proto.requiresEodCheck = hasEodAnchors(*this, bc, proto.outfixEndQueue);
    proto.hasOutfixesInSmallBlock = hasNonSmallBlockOutfix(outfixes);
    proto.canExhaust = rm.patternSetCanExhaust();
    proto.hasSom = hasSom;

    /* populate anchoredDistance, floatingDistance, floatingMinDistance, etc */
    fillMatcherDistances(*this, &proto);

    proto.initialGroups = getInitialGroups();
    proto.floating_group_mask = fgroups;
    proto.totalNumLiterals = verify_u32(literal_info.size());
    proto.asize = verify_u32(asize);
    proto.ematcherRegionSize = ematcher_region_size;
    proto.longLitStreamState = verify_u32(longLitStreamStateRequired);

    proto.size = currOffset;

    // Time to allocate the real RoseEngine structure.
    auto engine = aligned_zmalloc_unique<RoseEngine>(currOffset);
    assert(engine); // will have thrown bad_alloc otherwise.

    // Copy in our prototype engine data.
    memcpy(engine.get(), &proto, sizeof(proto));

    write_out(&engine->state_init, (char *)engine.get(), state_scatter,
              state_scatter_aux_offset);

    // Copy in the engine blob.
    bc.engine_blob.write_bytes(engine.get());

    // Add a small write engine if appropriate.
    engine = addSmallWriteEngine(*this, move(engine));

    DEBUG_PRINTF("rose done %p\n", engine.get());

    dumpRose(*this, fragments, makeLeftQueueMap(g, bc.leftfix_info),
             bc.suffixes, engine.get());

    return engine;
}

} // namespace ue2
