/*
 * Copyright (c) 2015-2020, Intel Corporation
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
#include "rose_build_misc.h"
#include "rose_build_program.h"
#include "rose_build_resources.h"
#include "rose_build_scatter.h"
#include "rose_build_util.h"
#include "rose_build_width.h"
#include "rose_internal.h"
#include "rose_program.h"
#include "hwlm/hwlm.h" /* engine types */
#include "hwlm/hwlm_build.h"
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
#include "util/bitutils.h"
#include "util/boundary_reports.h"
#include "util/charreach.h"
#include "util/charreach_util.h"
#include "util/compile_context.h"
#include "util/compile_error.h"
#include "util/container.h"
#include "util/fatbit_build.h"
#include "util/graph_range.h"
#include "util/insertion_ordered.h"
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

struct build_context : noncopyable {
    /** \brief information about engines to the left of a vertex */
    map<RoseVertex, left_build_info> leftfix_info;

    /** \brief mapping from suffix to queue index. */
    map<suffix_id, u32> suffixes;

    /** \brief engine info by queue. */
    map<u32, engine_info> engine_info_by_queue;

    /** \brief Simple cache of programs written to engine blob, used for
     * deduplication. */
    unordered_map<RoseProgram, u32, RoseProgramHash,
                  RoseProgramEquivalence> program_cache;

    /** \brief State indices, for those roles that have them.
     * Each vertex present has a unique state index in the range
     * [0, roleStateIndices.size()). */
    unordered_map<RoseVertex, u32> roleStateIndices;

    /** \brief Mapping from queue index to bytecode offset for built engines
     * that have already been pushed into the engine_blob. */
    unordered_map<u32, u32> engineOffsets;

    /** \brief List of long literals (ones with CHECK_LONG_LIT instructions)
     * that need hash table support. */
    vector<ue2_case_string> longLiterals;

    /** \brief Contents of the Rose bytecode immediately following the
     * RoseEngine. */
    RoseEngineBlob engine_blob;

    /** \brief True if this Rose engine has an MPV engine. */
    bool needs_mpv_catchup = false;

    /** \brief Resources in use (tracked as programs are added). */
    RoseResources resources;
};

/** \brief subengine info including built engine and
* corresponding triggering rose vertices */
struct ExclusiveSubengine {
    bytecode_ptr<NFA> nfa;
    vector<RoseVertex> vertices;
};

/** \brief exclusive info to build tamarama */
struct ExclusiveInfo : noncopyable {
    // subengine info
    vector<ExclusiveSubengine> subengines;
    // all the report in tamarama
    set<ReportID> reports;
    // assigned queue id
    u32 queue;
};

}

static
void add_nfa_to_blob(build_context &bc, NFA &nfa) {
    u32 qi = nfa.queueIndex;
    u32 nfa_offset = bc.engine_blob.add(nfa, nfa.length);
    DEBUG_PRINTF("added nfa qi=%u, type=%u, length=%u at offset=%u\n", qi,
                  nfa.type, nfa.length, nfa_offset);

    assert(!contains(bc.engineOffsets, qi));
    bc.engineOffsets.emplace(qi, nfa_offset);
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
 * \brief True if this Rose engine needs to run a catch up whenever a literal
 * report is generated.
 *
 * Catch up is necessary if there are output-exposed engines (suffixes,
 * outfixes).
 */
static
bool needsCatchup(const RoseBuildImpl &build) {
    /* Note: we could be more selective about when we need to generate catch up
     * instructions rather than just a boolean yes/no - for instance, if we know
     * that a role can only match before the point that an outfix/suffix could
     * match, we do not strictly need a catchup instruction.
     *
     * However, this would add a certain amount of complexity to the
     * catchup logic and would likely have limited applicability - how many
     * reporting roles have a fixed max offset and how much time is spent on
     * catchup for these cases?
     */

    if (!build.outfixes.empty()) {
        /* TODO: check that they have non-eod reports */
        DEBUG_PRINTF("has outfixes\n");
        return true;
    }

    const RoseGraph &g = build.g;

    for (auto v : vertices_range(g)) {
        if (g[v].suffix) {
            /* TODO: check that they have non-eod reports */
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
        DEBUG_PRINTF("has long literals in streaming mode, which needs long "
                     "literal table support\n");
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
    so->activeLeafArray_size = mmbit_size(activeArrayCount);

    so->activeLeftArray = curr_offset; /* TODO: limit size of array */
    curr_offset += mmbit_size(activeLeftCount);
    so->activeLeftArray_size = mmbit_size(activeLeftCount);

    so->longLitState = curr_offset;
    curr_offset += longLitStreamStateRequired;
    so->longLitState_size = longLitStreamStateRequired;

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
    so->exhausted_size = mmbit_size(build.rm.numEkeys());

    // Logical multibit.
    so->logicalVec = curr_offset;
    so->logicalVec_size = mmbit_size(build.rm.numLogicalKeys() +
                                     build.rm.numLogicalOps());
    curr_offset += so->logicalVec_size;

    // Combination multibit.
    so->combVec = curr_offset;
    so->combVec_size = mmbit_size(build.rm.numCkeys());
    curr_offset += so->combVec_size;

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
        so->somMultibit_size = mmbit_size(build.ssm.numSomSlots());
    } else {
        // No SOM handling, avoid growing the stream state any further.
        so->somLocation = 0;
        so->somValid = 0;
        so->somWritable = 0;
    }

    // note: state space for mask nfas is allocated later
    so->nfaStateBegin = curr_offset;
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
bytecode_ptr<NFA> pickImpl(bytecode_ptr<NFA> dfa_impl,
                           bytecode_ptr<NFA> nfa_impl,
                           bool fast_nfa) {
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
            if (n_accel && fast_nfa) {
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
bytecode_ptr<NFA>
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
bytecode_ptr<NFA> getDfa(raw_dfa &rdfa, bool is_transient,
                         const CompileContext &cc, const ReportManager &rm) {
    // Unleash the Sheng!!
    auto dfa = shengCompile(rdfa, cc, rm, false);
    if (!dfa && !is_transient) {
        // Sheng wasn't successful, so unleash McClellan!
        /* We don't try the hybrid for transient prefixes due to the extra
         * bytecode and that they are usually run on small blocks */
        dfa = mcshengCompile(rdfa, cc, rm);
    }
    if (!dfa) {
        dfa = sheng32Compile(rdfa, cc, rm, false);
    }
    if (!dfa) {
        dfa = sheng64Compile(rdfa, cc, rm, false);
    }
    if (!dfa && !is_transient) {
        dfa = mcshengCompile64(rdfa, cc, rm);
    }
    if (!dfa) {
        // Sheng wasn't successful, so unleash McClellan!
        dfa = mcclellanCompile(rdfa, cc, rm, false);
    }
    return dfa;
}

/* builds suffix nfas */
static
bytecode_ptr<NFA>
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

    bool fast_nfa = false;
    auto n = constructNFA(holder, &rm, fixed_depth_tops, triggers,
                          compress_state, fast_nfa, cc);
    assert(n);

    if (oneTop && cc.grey.roseMcClellanSuffix) {
        if (cc.grey.roseMcClellanSuffix == 2 || n->nPositions > 128 ||
            !has_bounded_repeats_other_than_firsts(*n) || !fast_nfa) {
            auto rdfa = buildMcClellan(holder, &rm, false, triggers.at(0),
                                       cc.grey);
            if (rdfa) {
                auto d = getDfa(*rdfa, false, cc, rm);
                assert(d);
                if (cc.grey.roseMcClellanSuffix != 2) {
                    n = pickImpl(move(d), move(n), fast_nfa);
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

        for (u32 id : lit_ids) {
            const rose_literal_id &lit = tbi.literals.at(id);
            (*trigger_lits)[top].push_back(as_cr_seq(lit));
        }
    }
}

static
bytecode_ptr<NFA> makeLeftNfa(const RoseBuildImpl &tbi, left_id &left,
                        const bool is_prefix, const bool is_transient,
                        const map<left_id, set<PredTopPair>> &infixTriggers,
                        const CompileContext &cc) {
    const ReportManager &rm = tbi.rm;

    bytecode_ptr<NFA> n;

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

    bool fast_nfa = false;
    if (!n && left.graph()) {
        map<u32, vector<vector<CharReach>>> triggers;
        if (left.graph()->kind == NFA_INFIX) {
            findTriggerSequences(tbi, infixTriggers.at(left), &triggers);
        }
        n = constructNFA(*left.graph(), nullptr, fixed_depth_tops, triggers,
                         compress_state, fast_nfa, cc);
    }

    if (cc.grey.roseMcClellanPrefix == 1 && is_prefix && !left.dfa()
        && left.graph()
        && (!n || !has_bounded_repeats_other_than_firsts(*n) || !fast_nfa)) {
        auto rdfa = buildMcClellan(*left.graph(), nullptr, cc.grey);
        if (rdfa) {
            auto d = getDfa(*rdfa, is_transient, cc, rm);
            assert(d);
            n = pickImpl(move(d), move(n), fast_nfa);
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
            u32 delay = build.literals.at(lit_id).delay;
            const ue2_literal &literal = build.literals.at(lit_id).s;
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
void enforceEngineSizeLimit(const NFA *n, const Grey &grey) {
    const size_t nfa_size = n->length;
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

    bytecode_ptr<NFA> nfa;
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
    enforceEngineSizeLimit(nfa.get(), cc.grey);
    bc.engine_info_by_queue.emplace(nfa->queueIndex,
                                    engine_info(nfa.get(), is_transient));

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
                    lits.insert(build.literals.at(lit_id).s);
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
                tamaProto.add(n, g[v].index, g[v].suffix.top, out_top_remap);
            } else {
                for (const auto &e : in_edges_range(v, g)) {
                    tamaProto.add(n, g[v].index, g[e].rose_top, out_top_remap);
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
                                               const bool is_suffix,
                                               const Grey &grey) {
    const auto &subengines = info.subengines;
    auto tamaInfo = constructTamaInfo(g, subengines, is_suffix);

    map<pair<const NFA *, u32>, u32> out_top_remap;
    auto n = buildTamarama(*tamaInfo, queue, out_top_remap);
    enforceEngineSizeLimit(n.get(), grey);
    bc.engine_info_by_queue.emplace(n->queueIndex, engine_info(n.get(), false));
    add_nfa_to_blob(bc, *n);

    DEBUG_PRINTF("queue id:%u\n", queue);
    shared_ptr<TamaProto> tamaProto = make_shared<TamaProto>();
    tamaProto->reports = info.reports;
    updateTops(g, *tamaInfo, *tamaProto, subengines, out_top_remap, is_suffix);
    return tamaProto;
}

static
void buildInfixContainer(RoseGraph &g, build_context &bc,
                         const vector<ExclusiveInfo> &exclusive_info,
                         const Grey &grey) {
    // Build tamarama engine
    for (const auto &info : exclusive_info) {
        const u32 queue = info.queue;
        const auto &subengines = info.subengines;
        auto tamaProto =
            constructContainerEngine(g, bc, info, queue, false, grey);

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
                          const vector<ExclusiveInfo> &exclusive_info,
                          const Grey &grey) {
    // Build tamarama engine
    for (const auto &info : exclusive_info) {
        const u32 queue = info.queue;
        const auto &subengines = info.subengines;
        auto tamaProto = constructContainerEngine(g, bc, info, queue, true,
                                                  grey);
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
                        lits.insert(build.literals.at(lit_id).s);
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
    buildInfixContainer(g, bc, exclusive_info, build.cc.grey);
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

        if (leftfix.haig()) {
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

    map<left_id, set<PredTopPair>> infixTriggers;
    findInfixTriggers(tbi, &infixTriggers);

    insertion_ordered_map<left_id, vector<RoseVertex>> succs;

    if (cc.grey.allowTamarama && cc.streaming && !do_prefix) {
        findExclusiveInfixes(tbi, bc, qif, infixTriggers, no_retrigger_queues);
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

        succs[leftfix].push_back(v);
    }

    rose_group initial_groups = tbi.getInitialGroups();
    rose_group combined_eager_squashed_mask = ~0ULL;

    map<left_id, eager_info> eager;

    for (const auto &m : succs) {
        const left_id &leftfix = m.first;
        const auto &left_succs = m.second;

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

    for (const auto &m : succs) {
        const left_id &leftfix = m.first;
        const auto &left_succs = m.second;
        buildLeftfix(tbi, bc, do_prefix, qif.get_queue(), infixTriggers,
                     no_retrigger_queues, eager_queues, eager, left_succs,
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
class OutfixBuilder : public boost::static_visitor<bytecode_ptr<NFA>> {
public:
    explicit OutfixBuilder(const RoseBuildImpl &build_in) : build(build_in) {}

    bytecode_ptr<NFA> operator()(boost::blank&) const {
        return nullptr;
    };

    bytecode_ptr<NFA> operator()(unique_ptr<raw_dfa> &rdfa) const {
        // Unleash the mighty DFA!
        return getDfa(*rdfa, false, build.cc, build.rm);
    }

    bytecode_ptr<NFA> operator()(unique_ptr<raw_som_dfa> &haig) const {
        // Unleash the Goughfish!
        return goughCompile(*haig, build.ssm.somPrecision(), build.cc,
                            build.rm);
    }

    bytecode_ptr<NFA> operator()(unique_ptr<NGHolder> &holder) const {
        const CompileContext &cc = build.cc;
        const ReportManager &rm = build.rm;

        NGHolder &h = *holder;
        assert(h.kind == NFA_OUTFIX);

        // Build NFA.
        const map<u32, u32> fixed_depth_tops; /* no tops */
        const map<u32, vector<vector<CharReach>>> triggers; /* no tops */
        bool compress_state = cc.streaming;
        bool fast_nfa = false;
        auto n = constructNFA(h, &rm, fixed_depth_tops, triggers,
                              compress_state, fast_nfa, cc);

        // Try for a DFA upgrade.
        if (n && cc.grey.roseMcClellanOutfix &&
            (!has_bounded_repeats_other_than_firsts(*n) || !fast_nfa)) {
            auto rdfa = buildMcClellan(h, &rm, cc.grey);
            if (rdfa) {
                auto d = getDfa(*rdfa, false, cc, rm);
                if (d) {
                    n = pickImpl(move(d), move(n), fast_nfa);
                }
            }
        }

        return n;
    }

    bytecode_ptr<NFA> operator()(UNUSED MpvProto &mpv) const {
        // MPV construction handled separately.
        assert(mpv.puffettes.empty());
        return nullptr;
    }

private:
    const RoseBuildImpl &build;
};
}

static
bytecode_ptr<NFA> buildOutfix(const RoseBuildImpl &build, OutfixInfo &outfix) {
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
    enforceEngineSizeLimit(nfa.get(), tbi.cc.grey);
    bc.engine_info_by_queue.emplace(nfa->queueIndex,
                                    engine_info(nfa.get(), false));

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
        enforceEngineSizeLimit(n.get(), tbi.cc.grey);
        bc.engine_info_by_queue.emplace(n->queueIndex,
                                        engine_info(n.get(), false));

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
    buildSuffixContainer(g, bc, exclusive_info, build.cc.grey);
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

        if (s.haig()) {
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
        enforceEngineSizeLimit(n.get(), tbi.cc.grey);
        bc.engine_info_by_queue.emplace(n->queueIndex,
                                        engine_info(n.get(), false));

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
void allocateStateSpace(const engine_info &eng_info, NfaInfo &nfa_info,
                        RoseStateOffsets *so, u32 *scratchStateSize,
                        u32 *transientStateSize) {
    u32 state_offset;
    if (eng_info.transient) {
        // Transient engines do not use stream state, but must have room in
        // transient state (stored in scratch).
        state_offset = *transientStateSize;
        *transientStateSize += eng_info.stream_size;
    } else {
        // Pack NFA stream state on to the end of the Rose stream state.
        state_offset = so->end;
        so->end += eng_info.stream_size;
    }

    nfa_info.stateOffset = state_offset;

    // Uncompressed state in scratch must be aligned.
    *scratchStateSize = ROUNDUP_N(*scratchStateSize, eng_info.scratch_align);
    nfa_info.fullStateOffset = *scratchStateSize;
    *scratchStateSize += eng_info.scratch_size;
}

static
void updateNfaState(const build_context &bc, vector<NfaInfo> &nfa_infos,
                    RoseStateOffsets *so, u32 *scratchStateSize,
                    u32 *transientStateSize) {
    if (nfa_infos.empty()) {
        return;
    }

    *transientStateSize = 0;
    *scratchStateSize = 0;

    for (u32 qi = 0; qi < nfa_infos.size(); qi++) {
        NfaInfo &nfa_info = nfa_infos[qi];
        const auto &eng_info = bc.engine_info_by_queue.at(qi);
        allocateStateSpace(eng_info, nfa_info, so, scratchStateSize,
                           transientStateSize);
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
    for (u32 id = 0; id < literals.size(); id++) {
        const auto &lit = literals.at(id);
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
        const auto &eng_info = bc.engine_info_by_queue.at(qi);
        if (eng_info.accepts_eod) {
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
bool anyEndfixMpvTriggers(const RoseBuildImpl &build) {
    const RoseGraph &g = build.g;
    unordered_set<suffix_id> done;

    /* suffixes */
    for (auto v : vertices_range(g)) {
        if (!g[v].suffix) {
            continue;
        }
        if (contains(done, g[v].suffix)) {
            continue; /* already done */
        }
        done.insert(g[v].suffix);

        if (hasMpvTrigger(all_reports(g[v].suffix), build.rm)) {
            return true;
        }
    }

    /* outfixes */
    for (const auto &out : build.outfixes) {
        if (hasMpvTrigger(all_reports(out), build.rm)) {
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
void recordResources(RoseResources &resources, const RoseBuildImpl &build,
                     const vector<raw_dfa> &anchored_dfas,
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

    resources.has_anchored = !anchored_dfas.empty();
    resources.has_anchored_multiple = anchored_dfas.size() > 1;
    for (const auto &rdfa : anchored_dfas) {
        if (rdfa.states.size() > 256) {
            resources.has_anchored_large = true;
        }
    }

}

static
u32 writeProgram(build_context &bc, RoseProgram &&program) {
    if (program.empty()) {
        DEBUG_PRINTF("no program\n");
        return 0;
    }

    applyFinalSpecialisation(program);

    auto it = bc.program_cache.find(program);
    if (it != end(bc.program_cache)) {
        DEBUG_PRINTF("reusing cached program at %u\n", it->second);
        return it->second;
    }

    recordResources(bc.resources, program);
    recordLongLiterals(bc.longLiterals, program);

    auto prog_bytecode = writeProgram(bc.engine_blob, program);
    u32 offset = bc.engine_blob.add(prog_bytecode);
    DEBUG_PRINTF("prog len %zu written at offset %u\n", prog_bytecode.size(),
                 offset);
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
bool hasEodAnchors(const RoseBuildImpl &build, const build_context &bc,
                   u32 outfixEndQueue) {
    for (u32 i = 0; i < outfixEndQueue; i++) {
        const auto &eng_info = bc.engine_info_by_queue.at(i);
        if (eng_info.accepts_eod) {
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
void writeLogicalInfo(const ReportManager &rm, RoseEngineBlob &engine_blob,
                      RoseEngine &proto) {
    const auto &tree = rm.getLogicalTree();
    proto.logicalTreeOffset = engine_blob.add_range(tree);
    const auto &combMap = rm.getCombInfoMap();
    proto.combInfoMapOffset = engine_blob.add_range(combMap);
    proto.lkeyCount = rm.numLogicalKeys();
    proto.lopCount = rm.numLogicalOps();
    proto.ckeyCount = rm.numCkeys();
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
                   &proto.tStateSize);

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

static
void makeBoundaryPrograms(const RoseBuildImpl &build, build_context &bc,
                          const BoundaryReports &boundary,
                          const DerivedBoundaryReports &dboundary,
                          RoseBoundaryReports &out) {
    DEBUG_PRINTF("report ^:  %zu\n", boundary.report_at_0.size());
    DEBUG_PRINTF("report $:  %zu\n", boundary.report_at_eod.size());
    DEBUG_PRINTF("report ^$: %zu\n", dboundary.report_at_0_eod_full.size());

    auto eod_prog = makeBoundaryProgram(build, boundary.report_at_eod);
    out.reportEodOffset = writeProgram(bc, move(eod_prog));

    auto zero_prog = makeBoundaryProgram(build, boundary.report_at_0);
    out.reportZeroOffset = writeProgram(bc, move(zero_prog));

    auto zeod_prog = makeBoundaryProgram(build, dboundary.report_at_0_eod_full);
    out.reportZeroEodOffset = writeProgram(bc, move(zeod_prog));
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
                        const set<u32> &eager_queues, u32 leftfixBeginQueue,
                        u32 leftfixCount, vector<LeftNfaInfo> &leftTable,
                        u32 *laggedRoseCount, size_t *history) {
    const RoseGraph &g = tbi.g;
    const CompileContext &cc = tbi.cc;

    unordered_set<u32> done_core;

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
        }

        DEBUG_PRINTF("rose %u is %s\n", left_index,
                     left.infix ? "infix" : "prefix");

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
RoseProgram makeLiteralProgram(const RoseBuildImpl &build, build_context &bc,
                               ProgramBuild &prog_build, u32 lit_id,
                               const vector<vector<RoseEdge>> &lit_edge_map,
                               bool is_anchored_replay_program) {
    DEBUG_PRINTF("lit_id=%u\n", lit_id);
    assert(lit_id < lit_edge_map.size());

    return makeLiteralProgram(build, bc.leftfix_info, bc.suffixes,
                              bc.engine_info_by_queue, bc.roleStateIndices,
                              prog_build, lit_id, lit_edge_map.at(lit_id),
                              is_anchored_replay_program);
}

static
RoseProgram makeFragmentProgram(const RoseBuildImpl &build, build_context &bc,
                               ProgramBuild &prog_build,
                               const vector<u32> &lit_ids,
                               const vector<vector<RoseEdge>> &lit_edge_map) {
    assert(!lit_ids.empty());

    vector<RoseProgram> blocks;
    for (const auto &lit_id : lit_ids) {
        auto prog = makeLiteralProgram(build, bc, prog_build, lit_id,
                                       lit_edge_map, false);
        blocks.push_back(move(prog));
    }

    return assembleProgramBlocks(move(blocks));
}

/**
 * \brief Returns a map from literal ID to a list of edges leading into
 * vertices with that literal ID.
 */
static
vector<vector<RoseEdge>> findEdgesByLiteral(const RoseBuildImpl &build) {
    vector<vector<RoseEdge>> lit_edge_map(build.literals.size());

    const auto &g = build.g;
    for (const auto &v : vertices_range(g)) {
        for (const auto &lit_id : g[v].literals) {
            assert(lit_id < lit_edge_map.size());
            auto &edge_list = lit_edge_map.at(lit_id);
            insert(&edge_list, edge_list.end(), in_edges(v, g));
        }
    }

    // Sort edges in each edge list by (source, target) indices. This gives us
    // less surprising ordering in program generation for a literal with many
    // edges.
    for (auto &edge_list : lit_edge_map) {
        sort(begin(edge_list), end(edge_list), [&g](const RoseEdge &a,
                                                    const RoseEdge &b) {
            return tie(g[source(a, g)].index, g[target(a, g)].index) <
                   tie(g[source(b, g)].index, g[target(b, g)].index);
        });
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
rose_literal_id getFragment(rose_literal_id lit) {
    if (lit.s.length() > ROSE_SHORT_LITERAL_LEN_MAX) {
        // Trim to last ROSE_SHORT_LITERAL_LEN_MAX bytes.
        lit.s.erase(0, lit.s.length() - ROSE_SHORT_LITERAL_LEN_MAX);
    }
    DEBUG_PRINTF("fragment: %s\n", dumpString(lit.s).c_str());
    return lit;
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

    for (u32 lit_id = 0; lit_id < build.literals.size(); lit_id++) {
        const auto &lit = build.literals.at(lit_id);
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
            fragments.emplace_back(frag_id, lit.s, groups, lit_id);
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
        auto &lit = m.first;
        auto &fi = m.second;
        DEBUG_PRINTF("frag %s -> ids: %s\n", dumpString(m.first.s).c_str(),
                     as_string_list(fi.lit_ids).c_str());
        fragments.emplace_back(frag_id, lit.s, fi.groups, move(fi.lit_ids));
        frag_id++;
        assert(frag_id == fragments.size());
    }

    return fragments;
}

static
void buildIncludedIdMap(unordered_map<u32, pair<u32, u8>> &includedIdMap,
                        const LitProto *litProto) {
    if (!litProto) {
        return;
    }
    const auto &proto = *litProto->hwlmProto;
    for (const auto &lit : proto.lits) {
        if (contains(includedIdMap, lit.id)) {
            const auto &included_id = includedIdMap[lit.id].first;
            const auto &squash = includedIdMap[lit.id].second;
            // The squash behavior should be the same for the same literal
            // in different literal matchers.
            if (lit.included_id != included_id ||
                lit.squash != squash) {
                includedIdMap[lit.id] = make_pair(INVALID_LIT_ID, 0);
                DEBUG_PRINTF("find different included info for the"
                             " same literal\n");
            }
        } else if (lit.included_id != INVALID_LIT_ID) {
            includedIdMap[lit.id] = make_pair(lit.included_id, lit.squash);
        } else {
            includedIdMap[lit.id] = make_pair(INVALID_LIT_ID, 0);
        }
    }
}

static
void findInclusionGroups(vector<LitFragment> &fragments,
                         LitProto *fproto, LitProto *drproto,
                         LitProto *eproto, LitProto *sbproto) {
    unordered_map<u32, pair<u32, u8>> includedIdMap;
    unordered_map<u32, pair<u32, u8>> includedDelayIdMap;
    buildIncludedIdMap(includedIdMap, fproto);
    buildIncludedIdMap(includedDelayIdMap, drproto);
    buildIncludedIdMap(includedIdMap, eproto);
    buildIncludedIdMap(includedIdMap, sbproto);

    size_t fragNum = fragments.size();
    vector<u32> candidates;
    for (size_t j = 0; j < fragNum; j++) {
        DEBUG_PRINTF("frag id %lu\n", j);
        u32 id = j;
        if (contains(includedIdMap, id) ||
            contains(includedDelayIdMap, id)) {
            candidates.push_back(j);
            DEBUG_PRINTF("find candidate\n");
        }
    }

    for (const auto &c : candidates) {
        auto &frag = fragments[c];
        u32 id = c;
        if (contains(includedIdMap, id) &&
            includedIdMap[id].first != INVALID_LIT_ID) {
            const auto &childId = includedIdMap[id];
            frag.included_frag_id = childId.first;
            frag.squash = childId.second;
            DEBUG_PRINTF("frag id %u child frag id %u\n", c,
                         frag.included_frag_id);
        }

        if (contains(includedDelayIdMap, id) &&
            includedDelayIdMap[id].first != INVALID_LIT_ID) {
            const auto &childId = includedDelayIdMap[id];
            frag.included_delay_frag_id = childId.first;
            frag.delay_squash = childId.second;

            DEBUG_PRINTF("delay frag id %u child frag id %u\n", c,
                             frag.included_delay_frag_id);
        }
    }
}

static
void buildFragmentPrograms(const RoseBuildImpl &build,
                           vector<LitFragment> &fragments,
                           build_context &bc, ProgramBuild &prog_build,
                           const vector<vector<RoseEdge>> &lit_edge_map) {
    // Sort fragments based on literal length and case info to build
    // included literal programs before their parent programs.
    vector<LitFragment> ordered_fragments(fragments);
    stable_sort(begin(ordered_fragments), end(ordered_fragments),
         [](const LitFragment &a, const LitFragment &b) {
             auto len1 = a.s.length();
             auto caseful1 = !a.s.any_nocase();
             auto len2 = b.s.length();
             auto caseful2 = !b.s.any_nocase();
             return tie(len1, caseful1) < tie(len2, caseful2);
         });

    for (auto &frag : ordered_fragments) {
        auto &pfrag = fragments[frag.fragment_id];
        DEBUG_PRINTF("frag_id=%u, lit_ids=[%s]\n", pfrag.fragment_id,
                     as_string_list(pfrag.lit_ids).c_str());

        auto lit_prog = makeFragmentProgram(build, bc, prog_build,
                                            pfrag.lit_ids, lit_edge_map);
        if (pfrag.included_frag_id != INVALID_FRAG_ID &&
            !lit_prog.empty()) {
            auto &cfrag = fragments[pfrag.included_frag_id];
            assert(pfrag.s.length() >= cfrag.s.length() &&
                   !pfrag.s.any_nocase() >= !cfrag.s.any_nocase());
            u32 child_offset = cfrag.lit_program_offset;
            DEBUG_PRINTF("child %u offset %u\n", cfrag.fragment_id,
                         child_offset);
            addIncludedJumpProgram(lit_prog, child_offset, pfrag.squash);
        }
        pfrag.lit_program_offset = writeProgram(bc, move(lit_prog));

        // We only do delayed rebuild in streaming mode.
        if (!build.cc.streaming) {
            continue;
        }

        auto rebuild_prog = makeDelayRebuildProgram(build, prog_build,
                                                    pfrag.lit_ids);
        if (pfrag.included_delay_frag_id != INVALID_FRAG_ID &&
            !rebuild_prog.empty()) {
            auto &cfrag = fragments[pfrag.included_delay_frag_id];
            assert(pfrag.s.length() >= cfrag.s.length() &&
                   !pfrag.s.any_nocase() >= !cfrag.s.any_nocase());
            u32 child_offset = cfrag.delay_program_offset;
            DEBUG_PRINTF("child %u offset %u\n", cfrag.fragment_id,
                         child_offset);
            addIncludedJumpProgram(rebuild_prog, child_offset,
                                   pfrag.delay_squash);
        }
        pfrag.delay_program_offset = writeProgram(bc, move(rebuild_prog));
    }
}

static
void updateLitProtoProgramOffset(vector<LitFragment> &fragments,
                                 LitProto &litProto, bool delay) {
    auto &proto = *litProto.hwlmProto;
    for (auto &lit : proto.lits) {
        auto fragId = lit.id;
        auto &frag = fragments[fragId];
        if (delay) {
            DEBUG_PRINTF("delay_program_offset:%u\n",
                         frag.delay_program_offset);
            lit.id = frag.delay_program_offset;
        } else {
            DEBUG_PRINTF("lit_program_offset:%u\n",
                         frag.lit_program_offset);
            lit.id = frag.lit_program_offset;
        }
    }
}

static
void updateLitProgramOffset(vector<LitFragment> &fragments,
                            LitProto *fproto, LitProto *drproto,
                            LitProto *eproto, LitProto *sbproto) {
    if (fproto) {
        updateLitProtoProgramOffset(fragments, *fproto, false);
    }

    if (drproto) {
        updateLitProtoProgramOffset(fragments, *drproto, true);
    }

    if (eproto) {
        updateLitProtoProgramOffset(fragments, *eproto, false);
    }

    if (sbproto) {
        updateLitProtoProgramOffset(fragments, *sbproto, false);
    }
}

/**
 * \brief Build the interpreter programs for each literal.
 */
static
void buildLiteralPrograms(const RoseBuildImpl &build,
                          vector<LitFragment> &fragments, build_context &bc,
                          ProgramBuild &prog_build, LitProto *fproto,
                          LitProto *drproto, LitProto *eproto,
                          LitProto *sbproto) {
    DEBUG_PRINTF("%zu fragments\n", fragments.size());
    auto lit_edge_map = findEdgesByLiteral(build);

    findInclusionGroups(fragments, fproto, drproto, eproto, sbproto);

    buildFragmentPrograms(build, fragments, bc, prog_build, lit_edge_map);

    // update literal program offsets for literal matcher prototypes
    updateLitProgramOffset(fragments, fproto, drproto, eproto, sbproto);
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
                auto prog = makeLiteralProgram(build, bc, prog_build,
                                               delayed_lit_id, lit_edge_map,
                                               false);
                u32 offset = writeProgram(bc, move(prog));

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
            const auto &lit = build.literals.at(lit_id);

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

            auto prog = makeLiteralProgram(build, bc, prog_build, lit_id,
                                           lit_edge_map, true);
            u32 offset = writeProgram(bc, move(prog));
            DEBUG_PRINTF("lit_id=%u -> anch prog at %u\n", lit_id, offset);

            u32 anch_id;
            auto it = cache.find(offset);
            if (it != end(cache)) {
                anch_id = it->second;
                DEBUG_PRINTF("reusing anch_id %u for offset %u\n", anch_id,
                             offset);
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
        auto program = makeReportProgram(build, bc.needs_mpv_catchup, id);
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
                DEBUG_PRINTF("already done report for vertex %zu\n",
                             g[u].index);
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
                makeEodAnchorProgram(build, prog_build, e, multiple_preds));
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

    auto block = makeLiteralProgram(build, bc.leftfix_info, bc.suffixes,
                                    bc.engine_info_by_queue,
                                    bc.roleStateIndices, prog_build,
                                    build.eod_event_literal_id, edge_list,
                                    false);
    program.add_block(move(block));
}

static
RoseProgram makeEodProgram(const RoseBuildImpl &build, build_context &bc,
                           ProgramBuild &prog_build, u32 eodNfaIterOffset) {
    RoseProgram program;

    addEodEventProgram(build, bc, prog_build, program);
    addEnginesEodProgram(eodNfaIterOffset, program);
    addEodAnchorProgram(build, bc, prog_build, false, program);
    if (hasEodMatcher(build)) {
        addMatcherEodProgram(program);
    }
    addEodAnchorProgram(build, bc, prog_build, true, program);
    if (hasEodAnchoredSuffix(build)) {
        addSuffixesEodProgram(program);
    }

    return program;
}

static
RoseProgram makeFlushCombProgram(const RoseEngine &t) {
    RoseProgram program;
    if (t.ckeyCount) {
        addFlushCombinationProgram(program);
    }
    return program;
}

static
RoseProgram makeLastFlushCombProgram(const RoseEngine &t) {
    RoseProgram program;
    if (t.ckeyCount) {
        addLastFlushCombinationProgram(program);
    }
    return program;
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
            const rose_literal_id &key = build.literals.at(lit_id);
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
bytecode_ptr<RoseEngine> addSmallWriteEngine(const RoseBuildImpl &build,
                                             const RoseResources &res,
                                             bytecode_ptr<RoseEngine> rose) {
    assert(rose);

    if (roseIsPureLiteral(rose.get())) {
        DEBUG_PRINTF("pure literal case, not adding smwr\n");
        return rose;
    }

    u32 qual = roseQuality(res, rose.get());
    auto smwr_engine = build.smwr.build(qual);
    if (!smwr_engine) {
        DEBUG_PRINTF("no smwr built\n");
        return rose;
    }

    const size_t mainSize = rose.size();
    const size_t smallWriteSize = smwr_engine.size();
    DEBUG_PRINTF("adding smwr engine, size=%zu\n", smallWriteSize);

    const size_t smwrOffset = ROUNDUP_CL(mainSize);
    const size_t newSize = smwrOffset + smallWriteSize;

    auto rose2 = make_zeroed_bytecode_ptr<RoseEngine>(newSize, 64);
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

    for (u32 id = 0; id < build.literals.size(); id++) {
        const rose_literal_id &lit = build.literals.at(id);

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

bytecode_ptr<RoseEngine> RoseBuildImpl::buildFinalEngine(u32 minWidth) {
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
    recordResources(bc.resources, *this, anchored_dfas, fragments);
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
    ProgramBuild prog_build(floatingMinLiteralMatchOffset,
                            longLitLengthThreshold, needsCatchup(*this));
    prog_build.vertex_group_map = getVertexGroupMap(*this);
    prog_build.squashable_groups = getSquashableGroups(*this);

    tie(proto.anchoredProgramOffset, proto.anchored_count) =
        writeAnchoredPrograms(*this, fragments, bc, prog_build);

    tie(proto.delayProgramOffset, proto.delay_count) =
        writeDelayPrograms(*this, fragments, bc, prog_build);

    // Build floating HWLM matcher prototype.
    rose_group fgroups = 0;
    auto fproto = buildFloatingMatcherProto(*this, fragments,
                                            longLitLengthThreshold,
                                            &fgroups, &historyRequired);

    // Build delay rebuild HWLM matcher prototype.
    auto drproto = buildDelayRebuildMatcherProto(*this, fragments,
                                                 longLitLengthThreshold);

    // Build EOD-anchored HWLM matcher prototype.
    auto eproto = buildEodAnchoredMatcherProto(*this, fragments);

    // Build small-block HWLM matcher prototype.
    auto sbproto = buildSmallBlockMatcherProto(*this, fragments);

    buildLiteralPrograms(*this, fragments, bc, prog_build, fproto.get(),
                         drproto.get(), eproto.get(), sbproto.get());

    auto eod_prog = makeEodProgram(*this, bc, prog_build, eodNfaIterOffset);
    proto.eodProgramOffset = writeProgram(bc, move(eod_prog));

    size_t longLitStreamStateRequired = 0;
    proto.longLitTableOffset
        = buildLongLiteralTable(*this, bc.engine_blob, bc.longLiterals,
                                longLitLengthThreshold, &historyRequired,
                                &longLitStreamStateRequired);

    proto.lastByteHistoryIterOffset = buildLastByteIter(g, bc);
    proto.eagerIterOffset = writeEagerQueueIter(
        eager_queues, proto.leftfixBeginQueue, queue_count, bc.engine_blob);

    addSomRevNfas(bc, proto, ssm);

    writeDkeyInfo(rm, bc.engine_blob, proto);
    writeLeftInfo(bc.engine_blob, proto, leftInfoTable);
    writeLogicalInfo(rm, bc.engine_blob, proto);

    auto flushComb_prog = makeFlushCombProgram(proto);
    proto.flushCombProgramOffset = writeProgram(bc, move(flushComb_prog));

    auto lastFlushComb_prog = makeLastFlushCombProgram(proto);
    proto.lastFlushCombProgramOffset =
        writeProgram(bc, move(lastFlushComb_prog));

    // Build anchored matcher.
    auto atable = buildAnchoredMatcher(*this, fragments, anchored_dfas);
    if (atable) {
        proto.amatcherOffset = bc.engine_blob.add(atable);
    }

    // Build floating HWLM matcher.
    auto ftable = buildHWLMMatcher(*this, fproto.get());
    if (ftable) {
        proto.fmatcherOffset = bc.engine_blob.add(ftable);
        bc.resources.has_floating = true;
    }

    // Build delay rebuild HWLM matcher.
    auto drtable = buildHWLMMatcher(*this, drproto.get());
    if (drtable) {
        proto.drmatcherOffset = bc.engine_blob.add(drtable);
    }

    // Build EOD-anchored HWLM matcher.
    auto etable = buildHWLMMatcher(*this, eproto.get());
    if (etable) {
        proto.ematcherOffset = bc.engine_blob.add(etable);
    }

    // Build small-block HWLM matcher.
    auto sbtable = buildHWLMMatcher(*this, sbproto.get());
    if (sbtable) {
        proto.sbmatcherOffset = bc.engine_blob.add(sbtable);
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
    proto.asize = verify_u32(atable.size());
    proto.ematcherRegionSize = ematcher_region_size;

    proto.size = currOffset;

    // Time to allocate the real RoseEngine structure, at cacheline alignment.
    auto engine = make_zeroed_bytecode_ptr<RoseEngine>(currOffset, 64);
    assert(engine); // will have thrown bad_alloc otherwise.

    // Copy in our prototype engine data.
    memcpy(engine.get(), &proto, sizeof(proto));

    write_out(&engine->state_init, (char *)engine.get(), state_scatter,
              state_scatter_aux_offset);

    // Copy in the engine blob.
    bc.engine_blob.write_bytes(engine.get());

    // Add a small write engine if appropriate.
    engine = addSmallWriteEngine(*this, bc.resources, move(engine));

    DEBUG_PRINTF("rose done %p\n", engine.get());

    dumpRose(*this, fragments, makeLeftQueueMap(g, bc.leftfix_info),
             bc.suffixes, engine.get());

    return engine;
}

} // namespace ue2
