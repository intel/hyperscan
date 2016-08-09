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

#include "rose_build_impl.h"

#include "ue2common.h"
#include "grey.h"
#include "hs_compile.h" // for HS_MODE_*
#include "rose_build_add_internal.h"
#include "rose_build_anchored.h"
#include "rose_build_exclusive.h"
#include "rose_build_groups.h"
#include "rose_build_infix.h"
#include "rose_build_lookaround.h"
#include "rose_build_matchers.h"
#include "rose_build_scatter.h"
#include "rose_build_util.h"
#include "rose_build_width.h"
#include "rose_program.h"
#include "hwlm/hwlm.h" /* engine types */
#include "nfa/castlecompile.h"
#include "nfa/goughcompile.h"
#include "nfa/mcclellancompile.h"
#include "nfa/mcclellancompile_util.h"
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
 * \brief Possible jump targets for roles that perform checks.
 *
 * Fixed up into offsets before the program is written to bytecode.
 */
enum class JumpTarget {
    NO_JUMP,        //!< Instruction does not jump.
    PROGRAM_END,    //!< Jump to end of program.
    NEXT_BLOCK,     //!< Jump to start of next block (sparse iter check, etc).
    FIXUP_DONE,     //!< Target fixup already applied.
};

/** \brief Role instruction model used at compile time. */
class RoseInstruction {
public:
    RoseInstruction(enum RoseInstructionCode c, JumpTarget j) : target(j) {
        memset(&u, 0, sizeof(u));
        u.end.code = c;
    }

    explicit RoseInstruction(enum RoseInstructionCode c)
        : RoseInstruction(c, JumpTarget::NO_JUMP) {}

    bool operator<(const RoseInstruction &a) const {
        if (code() != a.code()) {
            return code() < a.code();
        }
        if (target != a.target) {
            return target < a.target;
        }
        return memcmp(&u, &a.u, sizeof(u)) < 0;
    }

    bool operator==(const RoseInstruction &a) const {
        return code() == a.code() && target == a.target &&
               memcmp(&u, &a.u, sizeof(u)) == 0;
    }

    enum RoseInstructionCode code() const {
        // Note that this sort of type-punning (relying on identical initial
        // layout) is explicitly allowed by the C++11 standard.
        return (enum RoseInstructionCode)u.end.code;
    }

    const void *get() const {
        switch (code()) {
        case ROSE_INSTR_CHECK_LIT_MASK: return &u.checkLitMask;
        case ROSE_INSTR_CHECK_LIT_EARLY: return &u.checkLitEarly;
        case ROSE_INSTR_CHECK_GROUPS: return &u.checkGroups;
        case ROSE_INSTR_CHECK_ONLY_EOD: return &u.checkOnlyEod;
        case ROSE_INSTR_CHECK_BOUNDS: return &u.checkBounds;
        case ROSE_INSTR_CHECK_NOT_HANDLED: return &u.checkNotHandled;
        case ROSE_INSTR_CHECK_LOOKAROUND: return &u.checkLookaround;
        case ROSE_INSTR_CHECK_MASK: return &u.checkMask;
        case ROSE_INSTR_CHECK_BYTE: return &u.checkByte;
        case ROSE_INSTR_CHECK_INFIX: return &u.checkInfix;
        case ROSE_INSTR_CHECK_PREFIX: return &u.checkPrefix;
        case ROSE_INSTR_ANCHORED_DELAY: return &u.anchoredDelay;
        case ROSE_INSTR_PUSH_DELAYED: return &u.pushDelayed;
        case ROSE_INSTR_RECORD_ANCHORED: return &u.recordAnchored;
        case ROSE_INSTR_CATCH_UP: return &u.catchUp;
        case ROSE_INSTR_CATCH_UP_MPV: return &u.catchUpMpv;
        case ROSE_INSTR_SOM_ADJUST: return &u.somAdjust;
        case ROSE_INSTR_SOM_LEFTFIX: return &u.somLeftfix;
        case ROSE_INSTR_SOM_FROM_REPORT: return &u.somFromReport;
        case ROSE_INSTR_SOM_ZERO: return &u.somZero;
        case ROSE_INSTR_TRIGGER_INFIX: return &u.triggerInfix;
        case ROSE_INSTR_TRIGGER_SUFFIX: return &u.triggerSuffix;
        case ROSE_INSTR_DEDUPE: return &u.dedupe;
        case ROSE_INSTR_DEDUPE_SOM: return &u.dedupeSom;
        case ROSE_INSTR_REPORT_CHAIN: return &u.reportChain;
        case ROSE_INSTR_REPORT_SOM_INT: return &u.reportSomInt;
        case ROSE_INSTR_REPORT_SOM_AWARE: return &u.reportSomAware;
        case ROSE_INSTR_REPORT: return &u.report;
        case ROSE_INSTR_REPORT_EXHAUST: return &u.reportExhaust;
        case ROSE_INSTR_REPORT_SOM: return &u.reportSom;
        case ROSE_INSTR_REPORT_SOM_EXHAUST: return &u.reportSomExhaust;
        case ROSE_INSTR_DEDUPE_AND_REPORT: return &u.dedupeAndReport;
        case ROSE_INSTR_FINAL_REPORT: return &u.finalReport;
        case ROSE_INSTR_CHECK_EXHAUSTED: return &u.checkExhausted;
        case ROSE_INSTR_CHECK_MIN_LENGTH: return &u.checkMinLength;
        case ROSE_INSTR_SET_STATE: return &u.setState;
        case ROSE_INSTR_SET_GROUPS: return &u.setGroups;
        case ROSE_INSTR_SQUASH_GROUPS: return &u.squashGroups;
        case ROSE_INSTR_CHECK_STATE: return &u.checkState;
        case ROSE_INSTR_SPARSE_ITER_BEGIN: return &u.sparseIterBegin;
        case ROSE_INSTR_SPARSE_ITER_NEXT: return &u.sparseIterNext;
        case ROSE_INSTR_ENGINES_EOD: return &u.enginesEod;
        case ROSE_INSTR_SUFFIXES_EOD: return &u.suffixesEod;
        case ROSE_INSTR_MATCHER_EOD: return &u.matcherEod;
        case ROSE_INSTR_END: return &u.end;
        }
        assert(0);
        return &u.end;
    }

    size_t length() const {
        switch (code()) {
        case ROSE_INSTR_CHECK_LIT_MASK: return sizeof(u.checkLitMask);
        case ROSE_INSTR_CHECK_LIT_EARLY: return sizeof(u.checkLitEarly);
        case ROSE_INSTR_CHECK_GROUPS: return sizeof(u.checkGroups);
        case ROSE_INSTR_CHECK_ONLY_EOD: return sizeof(u.checkOnlyEod);
        case ROSE_INSTR_CHECK_BOUNDS: return sizeof(u.checkBounds);
        case ROSE_INSTR_CHECK_NOT_HANDLED: return sizeof(u.checkNotHandled);
        case ROSE_INSTR_CHECK_LOOKAROUND: return sizeof(u.checkLookaround);
        case ROSE_INSTR_CHECK_MASK: return sizeof(u.checkMask);
        case ROSE_INSTR_CHECK_BYTE: return sizeof(u.checkByte);
        case ROSE_INSTR_CHECK_INFIX: return sizeof(u.checkInfix);
        case ROSE_INSTR_CHECK_PREFIX: return sizeof(u.checkPrefix);
        case ROSE_INSTR_ANCHORED_DELAY: return sizeof(u.anchoredDelay);
        case ROSE_INSTR_PUSH_DELAYED: return sizeof(u.pushDelayed);
        case ROSE_INSTR_RECORD_ANCHORED: return sizeof(u.recordAnchored);
        case ROSE_INSTR_CATCH_UP: return sizeof(u.catchUp);
        case ROSE_INSTR_CATCH_UP_MPV: return sizeof(u.catchUpMpv);
        case ROSE_INSTR_SOM_ADJUST: return sizeof(u.somAdjust);
        case ROSE_INSTR_SOM_LEFTFIX: return sizeof(u.somLeftfix);
        case ROSE_INSTR_SOM_FROM_REPORT: return sizeof(u.somFromReport);
        case ROSE_INSTR_SOM_ZERO: return sizeof(u.somZero);
        case ROSE_INSTR_TRIGGER_INFIX: return sizeof(u.triggerInfix);
        case ROSE_INSTR_TRIGGER_SUFFIX: return sizeof(u.triggerSuffix);
        case ROSE_INSTR_DEDUPE: return sizeof(u.dedupe);
        case ROSE_INSTR_DEDUPE_SOM: return sizeof(u.dedupeSom);
        case ROSE_INSTR_REPORT_CHAIN: return sizeof(u.reportChain);
        case ROSE_INSTR_REPORT_SOM_INT: return sizeof(u.reportSomInt);
        case ROSE_INSTR_REPORT_SOM_AWARE: return sizeof(u.reportSomAware);
        case ROSE_INSTR_REPORT: return sizeof(u.report);
        case ROSE_INSTR_REPORT_EXHAUST: return sizeof(u.reportExhaust);
        case ROSE_INSTR_REPORT_SOM: return sizeof(u.reportSom);
        case ROSE_INSTR_REPORT_SOM_EXHAUST: return sizeof(u.reportSomExhaust);
        case ROSE_INSTR_DEDUPE_AND_REPORT: return sizeof(u.dedupeAndReport);
        case ROSE_INSTR_FINAL_REPORT: return sizeof(u.finalReport);
        case ROSE_INSTR_CHECK_EXHAUSTED: return sizeof(u.checkExhausted);
        case ROSE_INSTR_CHECK_MIN_LENGTH: return sizeof(u.checkMinLength);
        case ROSE_INSTR_SET_STATE: return sizeof(u.setState);
        case ROSE_INSTR_SET_GROUPS: return sizeof(u.setGroups);
        case ROSE_INSTR_SQUASH_GROUPS: return sizeof(u.squashGroups);
        case ROSE_INSTR_CHECK_STATE: return sizeof(u.checkState);
        case ROSE_INSTR_SPARSE_ITER_BEGIN: return sizeof(u.sparseIterBegin);
        case ROSE_INSTR_SPARSE_ITER_NEXT: return sizeof(u.sparseIterNext);
        case ROSE_INSTR_ENGINES_EOD: return sizeof(u.enginesEod);
        case ROSE_INSTR_SUFFIXES_EOD: return sizeof(u.suffixesEod);
        case ROSE_INSTR_MATCHER_EOD: return sizeof(u.matcherEod);
        case ROSE_INSTR_END: return sizeof(u.end);
        }
        assert(0);
        return 0;
    }

    union {
        ROSE_STRUCT_CHECK_LIT_MASK checkLitMask;
        ROSE_STRUCT_CHECK_LIT_EARLY checkLitEarly;
        ROSE_STRUCT_CHECK_GROUPS checkGroups;
        ROSE_STRUCT_CHECK_ONLY_EOD checkOnlyEod;
        ROSE_STRUCT_CHECK_BOUNDS checkBounds;
        ROSE_STRUCT_CHECK_NOT_HANDLED checkNotHandled;
        ROSE_STRUCT_CHECK_LOOKAROUND checkLookaround;
        ROSE_STRUCT_CHECK_MASK checkMask;
        ROSE_STRUCT_CHECK_BYTE checkByte;
        ROSE_STRUCT_CHECK_INFIX checkInfix;
        ROSE_STRUCT_CHECK_PREFIX checkPrefix;
        ROSE_STRUCT_ANCHORED_DELAY anchoredDelay;
        ROSE_STRUCT_PUSH_DELAYED pushDelayed;
        ROSE_STRUCT_RECORD_ANCHORED recordAnchored;
        ROSE_STRUCT_CATCH_UP catchUp;
        ROSE_STRUCT_CATCH_UP_MPV catchUpMpv;
        ROSE_STRUCT_SOM_ADJUST somAdjust;
        ROSE_STRUCT_SOM_LEFTFIX somLeftfix;
        ROSE_STRUCT_SOM_FROM_REPORT somFromReport;
        ROSE_STRUCT_SOM_ZERO somZero;
        ROSE_STRUCT_TRIGGER_INFIX triggerInfix;
        ROSE_STRUCT_TRIGGER_SUFFIX triggerSuffix;
        ROSE_STRUCT_DEDUPE dedupe;
        ROSE_STRUCT_DEDUPE_SOM dedupeSom;
        ROSE_STRUCT_REPORT_CHAIN reportChain;
        ROSE_STRUCT_REPORT_SOM_INT reportSomInt;
        ROSE_STRUCT_REPORT_SOM_AWARE reportSomAware;
        ROSE_STRUCT_REPORT report;
        ROSE_STRUCT_REPORT_EXHAUST reportExhaust;
        ROSE_STRUCT_REPORT_SOM reportSom;
        ROSE_STRUCT_REPORT_SOM_EXHAUST reportSomExhaust;
        ROSE_STRUCT_DEDUPE_AND_REPORT dedupeAndReport;
        ROSE_STRUCT_FINAL_REPORT finalReport;
        ROSE_STRUCT_CHECK_EXHAUSTED checkExhausted;
        ROSE_STRUCT_CHECK_MIN_LENGTH checkMinLength;
        ROSE_STRUCT_SET_STATE setState;
        ROSE_STRUCT_SET_GROUPS setGroups;
        ROSE_STRUCT_SQUASH_GROUPS squashGroups;
        ROSE_STRUCT_CHECK_STATE checkState;
        ROSE_STRUCT_SPARSE_ITER_BEGIN sparseIterBegin;
        ROSE_STRUCT_SPARSE_ITER_NEXT sparseIterNext;
        ROSE_STRUCT_ENGINES_EOD enginesEod;
        ROSE_STRUCT_SUFFIXES_EOD suffixesEod;
        ROSE_STRUCT_MATCHER_EOD matcherEod;
        ROSE_STRUCT_END end;
    } u;

    JumpTarget target;
};

static
size_t hash_value(const RoseInstruction &ri) {
    size_t val = 0;
    boost::hash_combine(val, ri.code());
    boost::hash_combine(val, ri.target);
    const char *bytes = (const char *)ri.get();
    const size_t len = ri.length();
    for (size_t i = 0; i < len; i++) {
        boost::hash_combine(val, bytes[i]);
    }
    return val;
}

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
    bool has_lit_mask = false;
    bool has_anchored = false;
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

    /** \brief Very simple cache from sparse iter to offset, used when building
     * up iterators in early misc. */
    map<vector<mmbit_sparse_iter>, u32> iterCache;

    /** \brief Simple cache of programs written to engine blob, used for
     * deduplication. */
    ue2::unordered_map<vector<RoseInstruction>, u32> program_cache;

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

    /** \brief Literal programs, indexed by final_id, after they have been
     * written to the engine_blob. */
    vector<u32> litPrograms;

    /** \brief Minimum offset of a match from the floating table. */
    u32 floatingMinLiteralMatchOffset = 0;

    /** \brief Contents of the Rose bytecode immediately following the
     * RoseEngine. */
    vector<char, AlignedAllocator<char, 64>> engine_blob;

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

    /** \brief Base offset of engine_blob in the Rose engine bytecode. */
    static constexpr u32 engine_blob_base = ROUNDUP_CL(sizeof(RoseEngine));
};

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

static
u32 add_to_engine_blob(build_context &bc, const void *a, const size_t len,
                       const size_t align) {
    pad_engine_blob(bc, align);

    size_t rv = bc.engine_blob_base + bc.engine_blob.size();
    assert(rv >= bc.engine_blob_base);
    DEBUG_PRINTF("write %zu bytes at offset %zu\n", len, rv);

    assert(ISALIGNED_N(bc.engine_blob.size(), align));

    bc.engine_blob.resize(bc.engine_blob.size() + len);
    memcpy(&bc.engine_blob.back() - len + 1, a, len);

    return verify_u32(rv);
}

template<typename T>
static
u32 add_to_engine_blob(build_context &bc, const T &a) {
    static_assert(is_pod<T>::value, "should be pod");
    return add_to_engine_blob(bc, &a, sizeof(a), alignof(T));
}

template<typename T>
static
u32 add_to_engine_blob(build_context &bc, const T &a, const size_t len) {
    static_assert(is_pod<T>::value, "should be pod");
    return add_to_engine_blob(bc, &a, len, alignof(T));
}

template<typename Iter>
static
u32 add_to_engine_blob(build_context &bc, Iter b, const Iter &e) {
    using value_type = typename std::iterator_traits<Iter>::value_type;
    static_assert(is_pod<value_type>::value, "should be pod");

    if (b == e) {
        return 0;
    }

    u32 offset = add_to_engine_blob(bc, *b);
    for (++b; b != e; ++b) {
        add_to_engine_blob(bc, *b);
    }

    return offset;
}

static
const NFA *get_nfa_from_blob(const build_context &bc, u32 qi) {
    assert(contains(bc.engineOffsets, qi));
    u32 nfa_offset = bc.engineOffsets.at(qi);
    assert(nfa_offset >= bc.engine_blob_base);
    const NFA *n = (const NFA *)(bc.engine_blob.data() + nfa_offset -
                                 bc.engine_blob_base);
    assert(n->queueIndex == qi);
    return n;
}

static
const NFA *add_nfa_to_blob(build_context &bc, NFA &nfa) {
    u32 qi = nfa.queueIndex;
    u32 nfa_offset = add_to_engine_blob(bc, nfa, nfa.length);
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
 * This is only the case if there are no anchored literals, suffixes, outfixes
 * etc.
 */
static
bool needsCatchup(const RoseBuildImpl &build) {
    if (!build.outfixes.empty()) {
        DEBUG_PRINTF("has outfixes\n");
        return true;
    }

    const RoseGraph &g = build.g;

    if (!isLeafNode(build.anchored_root, g)) {
        DEBUG_PRINTF("has anchored vertices\n");
        return true;
    }

    for (auto v : vertices_range(g)) {
        if (build.root == v) {
            continue;
        }

        if (build.anchored_root == v) {
            assert(isLeafNode(v, g));
            continue;
        }

        if (g[v].suffix) {
            DEBUG_PRINTF("vertex %zu has suffix\n", g[v].idx);
            return true;
        }

    }

    DEBUG_PRINTF("no need for catch-up on report\n");
    return false;
}

static
bool isPureFloating(const RoseResources &resources) {
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
    DEBUG_PRINTF("has_lit_mask=%d\n", bc.resources.has_lit_mask);
    DEBUG_PRINTF("has_anchored=%d\n", bc.resources.has_anchored);
    DEBUG_PRINTF("has_eod=%d\n", bc.resources.has_eod);

    if (isPureFloating(bc.resources)) {
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
                      u32 floatingStreamStateRequired, u32 historyRequired,
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

    so->floatingMatcherState = curr_offset;
    curr_offset += floatingStreamStateRequired;

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
aligned_unique_ptr<NFA> getDfa(raw_dfa &rdfa, const CompileContext &cc,
                               const ReportManager &rm) {
    // Unleash the Sheng!!
    auto dfa = shengCompile(rdfa, cc, rm);
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
        auto d = getDfa(*suff.dfa(), cc, rm);
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
                auto d = getDfa(*rdfa, cc, rm);
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
        n = getDfa(*left.dfa(), cc, rm);
    } else if (left.graph() && cc.grey.roseMcClellanPrefix == 2 && is_prefix &&
               !is_transient) {
        auto rdfa = buildMcClellan(*left.graph(), nullptr, cc.grey);
        if (rdfa) {
            n = getDfa(*rdfa, cc, rm);
            assert(n);
        }
    }

    // We can attempt to build LBRs for infixes.
    if (!n && !is_prefix && left.graph() && onlyOneTop(*left.graph())) {
        map<u32, vector<vector<CharReach> > > triggers;
        findTriggerSequences(tbi, infixTriggers.at(left), &triggers);
        assert(contains(triggers, 0)); // single top
        n = constructLBR(*left.graph(), triggers[0], cc, rm);
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
            auto d = getDfa(*rdfa, cc, rm);
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

    h.renumberEdges();
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
                tamaProto.add(n, g[v].idx, g[v].suffix.top,
                              out_top_remap);
            } else {
                for (const auto &e : in_edges_range(v, g)) {
                    tamaProto.add(n, g[v].idx, g[e].rose_top,
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
                DEBUG_PRINTF("vert id:%lu\n", g[v].idx);
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
                DEBUG_PRINTF("vert id:%lu\n", g[v].idx);
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
        assert(roseHasTops(g, v));

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
        DEBUG_PRINTF("leftfix size:%lu\n", leftfixes.size());
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
        assert(roseHasTops(g, v));

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
        return getDfa(*rdfa, build.cc, build.rm);
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
                auto d = getDfa(*rdfa, cc, rm);
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

        DEBUG_PRINTF("vertex %zu triggers suffix %p\n", g[v].idx, s.graph());

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

        DEBUG_PRINTF("vertex %zu triggers suffix %p\n", g[v].idx, s.graph());

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
        DEBUG_PRINTF("suffix size:%lu\n", suffixes.size());
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
        }
    }

    if (lb_roles.empty()) {
        return 0; /* invalid offset */
    }

    vector<mmbit_sparse_iter> iter;
    mmbBuildSparseIterator(iter, lb_roles, bc.numStates);
    return addIteratorToTable(bc, iter);
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
        (*out)[e.first] = add_to_engine_blob(bc, e.second.begin(),
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
    return addIteratorToTable(bc, iter);
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

/**
 * \brief Flattens a list of role programs into one finalised program with its
 * fail_jump/done_jump targets set correctly.
 */
static
vector<RoseInstruction>
flattenProgram(const vector<vector<RoseInstruction>> &programs) {
    vector<RoseInstruction> out;

    vector<u32> offsets; // offset of each instruction (bytes)
    vector<u32> blocks; // track which block we're in
    vector<u32> block_offsets; // start offsets for each block

    DEBUG_PRINTF("%zu program blocks\n", programs.size());

    size_t curr_offset = 0;
    for (const auto &program : programs) {
        DEBUG_PRINTF("block with %zu instructions\n", program.size());
        block_offsets.push_back(curr_offset);
        for (const auto &ri : program) {
            assert(ri.code() != ROSE_INSTR_END);
            out.push_back(ri);
            offsets.push_back(curr_offset);
            blocks.push_back(block_offsets.size() - 1);
            curr_offset += ROUNDUP_N(ri.length(), ROSE_INSTR_MIN_ALIGN);
        }
    }

    // Add a final END instruction, which is its own block.
    out.emplace_back(ROSE_INSTR_END);
    block_offsets.push_back(curr_offset);
    offsets.push_back(curr_offset);

    assert(offsets.size() == out.size());

    for (size_t i = 0; i < out.size(); i++) {
        auto &ri = out[i];

        u32 jump_target = 0;
        switch (ri.target) {
        case JumpTarget::NO_JUMP:
        case JumpTarget::FIXUP_DONE:
            continue; // Next instruction.
        case JumpTarget::PROGRAM_END:
            assert(i != out.size() - 1);
            jump_target = offsets.back();
            break;
        case JumpTarget::NEXT_BLOCK:
            assert(blocks[i] + 1 < block_offsets.size());
            jump_target = block_offsets[blocks[i] + 1];
            break;
        }

        // We currently always make progress and never jump backwards.
        assert(jump_target > offsets[i]);
        assert(jump_target <= offsets.back());
        u32 jump_val = jump_target - offsets[i];

        switch (ri.code()) {
        case ROSE_INSTR_ANCHORED_DELAY:
            ri.u.anchoredDelay.done_jump = jump_val;
            break;
        case ROSE_INSTR_CHECK_ONLY_EOD:
            ri.u.checkOnlyEod.fail_jump = jump_val;
            break;
        case ROSE_INSTR_CHECK_BOUNDS:
            ri.u.checkBounds.fail_jump = jump_val;
            break;
        case ROSE_INSTR_CHECK_NOT_HANDLED:
            ri.u.checkNotHandled.fail_jump = jump_val;
            break;
        case ROSE_INSTR_CHECK_LOOKAROUND:
            ri.u.checkLookaround.fail_jump = jump_val;
            break;
        case ROSE_INSTR_CHECK_MASK:
            ri.u.checkMask.fail_jump = jump_val;
            break;
        case ROSE_INSTR_CHECK_BYTE:
            ri.u.checkByte.fail_jump = jump_val;
            break;
        case ROSE_INSTR_CHECK_INFIX:
            ri.u.checkInfix.fail_jump = jump_val;
            break;
        case ROSE_INSTR_CHECK_PREFIX:
            ri.u.checkPrefix.fail_jump = jump_val;
            break;
        case ROSE_INSTR_DEDUPE:
            ri.u.dedupe.fail_jump = jump_val;
            break;
        case ROSE_INSTR_DEDUPE_SOM:
            ri.u.dedupeSom.fail_jump = jump_val;
            break;
        case ROSE_INSTR_DEDUPE_AND_REPORT:
            ri.u.dedupeAndReport.fail_jump = jump_val;
            break;
        case ROSE_INSTR_CHECK_EXHAUSTED:
            ri.u.checkExhausted.fail_jump = jump_val;
            break;
        case ROSE_INSTR_CHECK_MIN_LENGTH:
            ri.u.checkMinLength.fail_jump = jump_val;
            break;
        case ROSE_INSTR_CHECK_STATE:
            ri.u.checkState.fail_jump = jump_val;
            break;
        case ROSE_INSTR_SPARSE_ITER_BEGIN:
            ri.u.sparseIterBegin.fail_jump = jump_val;
            break;
        case ROSE_INSTR_SPARSE_ITER_NEXT:
            ri.u.sparseIterNext.fail_jump = jump_val;
            break;
        default:
            assert(0); // Unhandled opcode?
            break;
        }

        ri.target = JumpTarget::FIXUP_DONE;
    }

    return out;
}

static
void applyFinalSpecialisation(vector<RoseInstruction> &program) {
    assert(!program.empty());
    assert(program.back().code() == ROSE_INSTR_END);
    if (program.size() < 2) {
        return;
    }

    /* Replace the second-to-last instruction (before END) with a one-shot
     * specialisation if available. */
    auto &ri = *(next(program.rbegin()));
    switch (ri.code()) {
    case ROSE_INSTR_REPORT: {
        DEBUG_PRINTF("replacing REPORT with FINAL_REPORT\n");
        auto ri2 = RoseInstruction(ROSE_INSTR_FINAL_REPORT);
        ri2.u.finalReport.onmatch = ri.u.report.onmatch;
        ri2.u.finalReport.offset_adjust = ri.u.report.offset_adjust;
        ri = ri2;
        break;
    }
    default:
        break;
    }
}

static
void recordResources(RoseResources &resources,
                     const vector<RoseInstruction> &program) {
    for (const auto &ri : program) {
        switch (ri.code()) {
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
        case ROSE_INSTR_CHECK_LIT_MASK:
            resources.has_lit_mask = true;
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
u32 writeProgram(build_context &bc, const vector<RoseInstruction> &program) {
    if (program.empty()) {
        DEBUG_PRINTF("no program\n");
        return 0;
    }

    assert(program.back().code() == ROSE_INSTR_END);
    assert(program.size() >= 1);

    // This program must have been flattened; i.e. all check instructions must
    // have their jump offsets set.
    assert(all_of(begin(program), end(program), [](const RoseInstruction &ri) {
        return ri.target == JumpTarget::NO_JUMP ||
               ri.target == JumpTarget::FIXUP_DONE;
    }));

    auto it = bc.program_cache.find(program);
    if (it != end(bc.program_cache)) {
        DEBUG_PRINTF("reusing cached program at %u\n", it->second);
        return it->second;
    }

    recordResources(bc.resources, program);

    DEBUG_PRINTF("writing %zu instructions\n", program.size());
    u32 programOffset = 0;
    for (const auto &ri : program) {
        u32 offset =
            add_to_engine_blob(bc, ri.get(), ri.length(), ROSE_INSTR_MIN_ALIGN);
        DEBUG_PRINTF("code %u len %zu written at offset %u\n", ri.code(),
                     ri.length(), offset);
        if (!programOffset) {
            programOffset = offset;
        }
    }
    DEBUG_PRINTF("program begins at offset %u\n", programOffset);
    bc.program_cache.emplace(program, programOffset);
    return programOffset;
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
bool makeRoleByte(const vector<LookEntry> &look,
                  vector<RoseInstruction> &program) {
    if (look.size() == 1) {
        const auto &entry = look[0];
        u8 andmask_u8, cmpmask_u8;
        u8 flip;
        if (!checkReachWithFlip(entry.reach, andmask_u8, cmpmask_u8, flip)) {
            return false;
        }
        s32 checkbyte_offset = verify_s32(entry.offset);
        DEBUG_PRINTF("CHECK BYTE offset=%d\n", checkbyte_offset);
        auto ri = RoseInstruction(ROSE_INSTR_CHECK_BYTE,
                                  JumpTarget::NEXT_BLOCK);
        ri.u.checkByte.and_mask = andmask_u8;
        ri.u.checkByte.cmp_mask = cmpmask_u8;
        ri.u.checkByte.negation = flip;
        ri.u.checkByte.offset = checkbyte_offset;
        program.push_back(ri);
        return true;
    }
    return false;
}

static
bool makeRoleMask(const vector<LookEntry> &look,
                  vector<RoseInstruction> &program) {
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
        auto ri = RoseInstruction(ROSE_INSTR_CHECK_MASK,
                                  JumpTarget::NEXT_BLOCK);
        ri.u.checkMask.and_mask = and_mask;
        ri.u.checkMask.cmp_mask = cmp_mask;
        ri.u.checkMask.neg_mask = neg_mask;
        ri.u.checkMask.offset = base_offset;
        program.push_back(ri);
        return true;
    }
    return false;
}

static
void makeRoleLookaround(RoseBuildImpl &build, build_context &bc, RoseVertex v,
                        vector<RoseInstruction> &program) {
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

    if (makeRoleByte(look, program)) {
        return;
    }

    if (makeRoleMask(look, program)) {
        return;
    }

    DEBUG_PRINTF("role has lookaround\n");
    u32 look_idx = addLookaround(bc, look);
    u32 look_count = verify_u32(look.size());

    auto ri = RoseInstruction(ROSE_INSTR_CHECK_LOOKAROUND,
                              JumpTarget::NEXT_BLOCK);
    ri.u.checkLookaround.index = look_idx;
    ri.u.checkLookaround.count = look_count;
    program.push_back(ri);
}

static
void makeRoleCheckLeftfix(RoseBuildImpl &build, build_context &bc, RoseVertex v,
                          vector<RoseInstruction> &program) {
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
    if (is_prefix) {
        auto ri =
            RoseInstruction(ROSE_INSTR_CHECK_PREFIX, JumpTarget::NEXT_BLOCK);
        ri.u.checkPrefix.queue = lni.queue;
        ri.u.checkPrefix.lag = build.g[v].left.lag;
        ri.u.checkPrefix.report = build.g[v].left.leftfix_report;
        program.push_back(move(ri));
    } else {
        auto ri =
            RoseInstruction(ROSE_INSTR_CHECK_INFIX, JumpTarget::NEXT_BLOCK);
        ri.u.checkInfix.queue = lni.queue;
        ri.u.checkInfix.lag = build.g[v].left.lag;
        ri.u.checkInfix.report = build.g[v].left.leftfix_report;
        program.push_back(move(ri));
    }
}

static
void makeRoleAnchoredDelay(RoseBuildImpl &build, build_context &bc,
                           RoseVertex v, vector<RoseInstruction> &program) {
    // Only relevant for roles that can be triggered by the anchored table.
    if (!build.isAnchored(v)) {
        return;
    }

    // If this match cannot occur after floatingMinLiteralMatchOffset, we do
    // not need this check.
    if (build.g[v].max_offset <= bc.floatingMinLiteralMatchOffset) {
        return;
    }

    auto ri = RoseInstruction(ROSE_INSTR_ANCHORED_DELAY,
                              JumpTarget::NEXT_BLOCK);
    ri.u.anchoredDelay.groups = build.g[v].groups;
    program.push_back(ri);
}

static
void makeDedupe(const RoseBuildImpl &build, const Report &report,
                vector<RoseInstruction> &report_block) {
    auto ri = RoseInstruction(ROSE_INSTR_DEDUPE, JumpTarget::NEXT_BLOCK);
    ri.u.dedupe.quash_som = report.quashSom;
    ri.u.dedupe.dkey = build.rm.getDkey(report);
    ri.u.dedupe.offset_adjust = report.offsetAdjust;
    report_block.push_back(move(ri));
}

static
void makeDedupeSom(const RoseBuildImpl &build, const Report &report,
                   vector<RoseInstruction> &report_block) {
    auto ri = RoseInstruction(ROSE_INSTR_DEDUPE_SOM, JumpTarget::NEXT_BLOCK);
    ri.u.dedupeSom.quash_som = report.quashSom;
    ri.u.dedupeSom.dkey = build.rm.getDkey(report);
    ri.u.dedupeSom.offset_adjust = report.offsetAdjust;
    report_block.push_back(move(ri));
}

static
void makeCatchup(RoseBuildImpl &build, build_context &bc,
                 const flat_set<ReportID> &reports,
                 vector<RoseInstruction> &program) {
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

    program.emplace_back(ROSE_INSTR_CATCH_UP);
}

static
void makeCatchupMpv(RoseBuildImpl &build, build_context &bc, ReportID id,
                    vector<RoseInstruction> &program) {
    if (!bc.needs_mpv_catchup) {
        return;
    }

    const Report &report = build.rm.getReport(id);
    if (report.type == INTERNAL_ROSE_CHAIN) {
        return;
    }

    program.emplace_back(ROSE_INSTR_CATCH_UP_MPV);
}

static
void writeSomOperation(const Report &report, som_operation *op) {
    assert(op);

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
                const bool has_som, vector<RoseInstruction> &program) {
    assert(id < build.rm.numReports());
    const Report &report = build.rm.getReport(id);

    vector<RoseInstruction> report_block;

    // Handle min/max offset checks.
    if (report.minOffset > 0 || report.maxOffset < MAX_OFFSET) {
        auto ri = RoseInstruction(ROSE_INSTR_CHECK_BOUNDS,
                                  JumpTarget::NEXT_BLOCK);
        ri.u.checkBounds.min_bound = report.minOffset;
        ri.u.checkBounds.max_bound = report.maxOffset;
        report_block.push_back(move(ri));
    }

    // If this report has an exhaustion key, we can check it in the program
    // rather than waiting until we're in the callback adaptor.
    if (report.ekey != INVALID_EKEY) {
        auto ri = RoseInstruction(ROSE_INSTR_CHECK_EXHAUSTED,
                                  JumpTarget::NEXT_BLOCK);
        ri.u.checkExhausted.ekey = report.ekey;
        report_block.push_back(move(ri));
    }

    // External SOM reports that aren't passthrough need their SOM value
    // calculated.
    if (isExternalSomReport(report) &&
        report.type != EXTERNAL_CALLBACK_SOM_PASS) {
        auto ri = RoseInstruction(ROSE_INSTR_SOM_FROM_REPORT);
        writeSomOperation(report, &ri.u.somFromReport.som);
        report_block.push_back(move(ri));
    }

    // Min length constraint.
    if (report.minLength > 0) {
        assert(build.hasSom);
        auto ri = RoseInstruction(ROSE_INSTR_CHECK_MIN_LENGTH,
                                  JumpTarget::NEXT_BLOCK);
        ri.u.checkMinLength.end_adj = report.offsetAdjust;
        ri.u.checkMinLength.min_length = report.minLength;
        report_block.push_back(move(ri));
    }

    if (report.quashSom) {
        report_block.emplace_back(ROSE_INSTR_SOM_ZERO);
    }

    switch (report.type) {
    case EXTERNAL_CALLBACK:
        if (!has_som) {
            // Dedupe is only necessary if this report has a dkey, or if there
            // are SOM reports to catch up.
            bool needs_dedupe = build.rm.getDkey(report) != ~0U || build.hasSom;
            if (report.ekey == INVALID_EKEY) {
                if (needs_dedupe) {
                    report_block.emplace_back(ROSE_INSTR_DEDUPE_AND_REPORT,
                                              JumpTarget::NEXT_BLOCK);
                    auto &ri = report_block.back();
                    ri.u.dedupeAndReport.quash_som = report.quashSom;
                    ri.u.dedupeAndReport.dkey = build.rm.getDkey(report);
                    ri.u.dedupeAndReport.onmatch = report.onmatch;
                    ri.u.dedupeAndReport.offset_adjust = report.offsetAdjust;
                } else {
                    report_block.emplace_back(ROSE_INSTR_REPORT);
                    auto &ri = report_block.back();
                    ri.u.report.onmatch = report.onmatch;
                    ri.u.report.offset_adjust = report.offsetAdjust;
                }
            } else {
                if (needs_dedupe) {
                    makeDedupe(build, report, report_block);
                }
                report_block.emplace_back(ROSE_INSTR_REPORT_EXHAUST);
                auto &ri = report_block.back();
                ri.u.reportExhaust.onmatch = report.onmatch;
                ri.u.reportExhaust.offset_adjust = report.offsetAdjust;
                ri.u.reportExhaust.ekey = report.ekey;
            }
        } else { // has_som
            makeDedupeSom(build, report, report_block);
            if (report.ekey == INVALID_EKEY) {
                report_block.emplace_back(ROSE_INSTR_REPORT_SOM);
                auto &ri = report_block.back();
                ri.u.reportSom.onmatch = report.onmatch;
                ri.u.reportSom.offset_adjust = report.offsetAdjust;
            } else {
                report_block.emplace_back(ROSE_INSTR_REPORT_SOM_EXHAUST);
                auto &ri = report_block.back();
                ri.u.reportSomExhaust.onmatch = report.onmatch;
                ri.u.reportSomExhaust.offset_adjust = report.offsetAdjust;
                ri.u.reportSomExhaust.ekey = report.ekey;
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
            report_block.emplace_back(ROSE_INSTR_REPORT_SOM_AWARE);
            auto &ri = report_block.back();
            writeSomOperation(report, &ri.u.reportSomAware.som);
        } else {
            report_block.emplace_back(ROSE_INSTR_REPORT_SOM_INT);
            auto &ri = report_block.back();
            writeSomOperation(report, &ri.u.reportSomInt.som);
        }
        break;
    case INTERNAL_ROSE_CHAIN: {
        report_block.emplace_back(ROSE_INSTR_REPORT_CHAIN);
        auto &ri = report_block.back();
        ri.u.reportChain.event = report.onmatch;
        ri.u.reportChain.top_squash_distance = report.topSquashDistance;
        break;
    }
    case EXTERNAL_CALLBACK_SOM_REL:
    case EXTERNAL_CALLBACK_SOM_STORED:
    case EXTERNAL_CALLBACK_SOM_ABS:
    case EXTERNAL_CALLBACK_SOM_REV_NFA:
        makeDedupeSom(build, report, report_block);
        if (report.ekey == INVALID_EKEY) {
            report_block.emplace_back(ROSE_INSTR_REPORT_SOM);
            auto &ri = report_block.back();
            ri.u.reportSom.onmatch = report.onmatch;
            ri.u.reportSom.offset_adjust = report.offsetAdjust;
        } else {
            report_block.emplace_back(ROSE_INSTR_REPORT_SOM_EXHAUST);
            auto &ri = report_block.back();
            ri.u.reportSomExhaust.onmatch = report.onmatch;
            ri.u.reportSomExhaust.offset_adjust = report.offsetAdjust;
            ri.u.reportSomExhaust.ekey = report.ekey;
        }
        break;
    case EXTERNAL_CALLBACK_SOM_PASS:
        makeDedupeSom(build, report, report_block);
        if (report.ekey == INVALID_EKEY) {
            report_block.emplace_back(ROSE_INSTR_REPORT_SOM);
            auto &ri = report_block.back();
            ri.u.reportSom.onmatch = report.onmatch;
            ri.u.reportSom.offset_adjust = report.offsetAdjust;
        } else {
            report_block.emplace_back(ROSE_INSTR_REPORT_SOM_EXHAUST);
            auto &ri = report_block.back();
            ri.u.reportSomExhaust.onmatch = report.onmatch;
            ri.u.reportSomExhaust.offset_adjust = report.offsetAdjust;
            ri.u.reportSomExhaust.ekey = report.ekey;
        }
        break;

    default:
        assert(0);
        throw CompileError("Unable to generate bytecode.");
    }

    assert(!report_block.empty());
    report_block = flattenProgram({report_block});
    assert(report_block.back().code() == ROSE_INSTR_END);
    report_block.pop_back();
    insert(&program, program.end(), report_block);
}

static
void makeRoleReports(RoseBuildImpl &build, build_context &bc, RoseVertex v,
                     vector<RoseInstruction> &program) {
    const auto &g = build.g;

    /* we are a suffaig - need to update role to provide som to the
     * suffix. */
    bool has_som = false;
    if (g[v].left.tracksSom()) {
        assert(contains(bc.leftfix_info, v));
        const left_build_info &lni = bc.leftfix_info.at(v);
        auto ri = RoseInstruction(ROSE_INSTR_SOM_LEFTFIX);
        ri.u.somLeftfix.queue = lni.queue;
        ri.u.somLeftfix.lag = g[v].left.lag;
        program.push_back(ri);
        has_som = true;
    } else if (g[v].som_adjust) {
        auto ri = RoseInstruction(ROSE_INSTR_SOM_ADJUST);
        ri.u.somAdjust.distance = g[v].som_adjust;
        program.push_back(ri);
        has_som = true;
    }

    const auto &reports = g[v].reports;
    makeCatchup(build, bc, reports, program);

    for (ReportID id : reports) {
        makeReport(build, id, has_som, program);
    }
}

static
void makeRoleSuffix(RoseBuildImpl &build, build_context &bc, RoseVertex v,
                    vector<RoseInstruction> &program) {
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
                  tamaProto->top_remap.at(make_pair(g[v].idx,
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
    auto ri = RoseInstruction(ROSE_INSTR_TRIGGER_SUFFIX);
    ri.u.triggerSuffix.queue = qi;
    ri.u.triggerSuffix.event = suffixEvent;
    program.push_back(ri);
}

static
void makeRoleGroups(RoseBuildImpl &build, build_context &bc, RoseVertex v,
                    vector<RoseInstruction> &program) {
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

    auto ri = RoseInstruction(ROSE_INSTR_SET_GROUPS);
    ri.u.setGroups.groups = groups;
    program.push_back(ri);
}

static
void makeRoleInfixTriggers(RoseBuildImpl &build, build_context &bc,
                           RoseVertex u, vector<RoseInstruction> &program) {
    const auto &g = build.g;

    vector<RoseInstruction> infix_program;

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
                                      make_pair(g[v].idx, g[e].rose_top));
            assert(top < MQE_INVALID);
        } else if (!isMultiTopType(nfa->type)) {
            assert(num_tops(g[v].left) == 1);
            top = MQE_TOP;
        } else {
            top = MQE_TOP_FIRST + g[e].rose_top;
            assert(top < MQE_INVALID);
        }

        auto ri = RoseInstruction(ROSE_INSTR_TRIGGER_INFIX);
        ri.u.triggerInfix.queue = lbi.queue;
        ri.u.triggerInfix.event = top;
        ri.u.triggerInfix.cancel = g[e].rose_cancel_prev_top;
        infix_program.push_back(ri);
    }

    if (infix_program.empty()) {
        return;
    }

    // Order, de-dupe and add instructions to the end of program.
    sort(begin(infix_program), end(infix_program));
    unique_copy(begin(infix_program), end(infix_program),
                back_inserter(program));
}

static
void makeRoleSetState(const build_context &bc, RoseVertex v,
                      vector<RoseInstruction> &program) {
    // We only need this instruction if a state index has been assigned to this
    // vertex.
    auto it = bc.roleStateIndices.find(v);
    if (it == end(bc.roleStateIndices)) {
        return;
    }

    u32 idx = it->second;
    auto ri = RoseInstruction(ROSE_INSTR_SET_STATE);
    ri.u.setState.index = idx;
    program.push_back(ri);
}

static
void makeRoleCheckBounds(const RoseBuildImpl &build, RoseVertex v,
                         const RoseEdge &e, vector<RoseInstruction> &program) {
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

    auto ri = RoseInstruction(ROSE_INSTR_CHECK_BOUNDS, JumpTarget::NEXT_BLOCK);
    ri.u.checkBounds.min_bound = min_bound;
    ri.u.checkBounds.max_bound = max_bound;

    program.push_back(move(ri));
}

static
void makeRoleCheckNotHandled(build_context &bc, RoseVertex v,
                             vector<RoseInstruction> &program) {
    auto ri = RoseInstruction(ROSE_INSTR_CHECK_NOT_HANDLED,
                              JumpTarget::NEXT_BLOCK);

    u32 handled_key;
    if (contains(bc.handledKeys, v)) {
        handled_key = bc.handledKeys.at(v);
    } else {
        handled_key = verify_u32(bc.handledKeys.size());
        bc.handledKeys.emplace(v, handled_key);
    }

    ri.u.checkNotHandled.key = handled_key;

    program.push_back(move(ri));
}

static
void makeRoleEagerEodReports(RoseBuildImpl &build, build_context &bc,
                             RoseVertex v, vector<RoseInstruction> &program) {
    vector<RoseInstruction> eod_program;

    for (const auto &e : out_edges_range(v, build.g)) {
        if (canEagerlyReportAtEod(build, e)) {
            makeRoleReports(build, bc, target(e, build.g), eod_program);
        }
    }

    if (eod_program.empty()) {
        return;
    }

    if (!onlyAtEod(build, v)) {
        // The rest of our program wasn't EOD anchored, so we need to guard
        // these reports with a check.
        program.emplace_back(ROSE_INSTR_CHECK_ONLY_EOD, JumpTarget::NEXT_BLOCK);
    }

    program.insert(end(program), begin(eod_program), end(eod_program));
}

static
vector<RoseInstruction> makeProgram(RoseBuildImpl &build, build_context &bc,
                                    const RoseEdge &e) {
    const RoseGraph &g = build.g;
    auto v = target(e, g);

    vector<RoseInstruction> program;

    // First, add program instructions that enforce preconditions without
    // effects.

    makeRoleAnchoredDelay(build, bc, v, program);

    if (onlyAtEod(build, v)) {
        DEBUG_PRINTF("only at eod\n");
        program.push_back(RoseInstruction(ROSE_INSTR_CHECK_ONLY_EOD,
                                          JumpTarget::NEXT_BLOCK));
    }

    if (g[e].history == ROSE_ROLE_HISTORY_ANCH) {
        makeRoleCheckBounds(build, v, e, program);
    }

    // This program may be triggered by different predecessors, with different
    // offset bounds. We must ensure we put this check/set operation after the
    // bounds check to deal with this case.
    if (hasGreaterInDegree(1, v, g)) {
        makeRoleCheckNotHandled(bc, v, program);
    }

    makeRoleLookaround(build, bc, v, program);
    makeRoleCheckLeftfix(build, bc, v, program);

    // Next, we can add program instructions that have effects.

    makeRoleReports(build, bc, v, program);

    makeRoleInfixTriggers(build, bc, v, program);

    // Note: SET_GROUPS instruction must be after infix triggers, as an infix
    // going dead may switch off groups.
    makeRoleGroups(build, bc, v, program);

    makeRoleSuffix(build, bc, v, program);

    makeRoleSetState(bc, v, program);

    // Note: EOD eager reports may generate a CHECK_ONLY_EOD instruction (if
    // the program doesn't have one already).
    makeRoleEagerEodReports(build, bc, v, program);

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
    vector<RoseInstruction> program;
    for (const auto &id : reports) {
        makeReport(build, id, has_som, program);
    }
    program = flattenProgram({program});
    applyFinalSpecialisation(program);
    return writeProgram(bc, program);
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
                left.stopTable = add_to_engine_blob(bc, lbi.stopAlphabet.begin(),
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
void addPredBlocksSingle(
    map<u32, vector<vector<RoseInstruction>>> &predProgramLists,
    vector<RoseInstruction> &program) {

    vector<vector<RoseInstruction>> prog_blocks;

    for (const auto &m : predProgramLists) {
        const u32 &pred_state = m.first;
        assert(!m.second.empty());
        auto subprog = flattenProgram(m.second);

        // Check our pred state.
        auto ri = RoseInstruction(ROSE_INSTR_CHECK_STATE,
                                  JumpTarget::NEXT_BLOCK);
        ri.u.checkState.index = pred_state;
        subprog.insert(begin(subprog), ri);
        assert(subprog.back().code() == ROSE_INSTR_END);
        subprog.pop_back();
        prog_blocks.push_back(move(subprog));
    }

    auto prog = flattenProgram(prog_blocks);
    program.insert(end(program), begin(prog), end(prog));
}

static
u32 programLength(const vector<RoseInstruction> &program) {
    u32 len = 0;
    for (const auto &ri : program) {
        len += ROUNDUP_N(ri.length(), ROSE_INSTR_MIN_ALIGN);
    }
    return len;
}

static
void addPredBlocksMulti(build_context &bc,
                    map<u32, vector<vector<RoseInstruction>>> &predProgramLists,
                    vector<RoseInstruction> &program) {
    assert(!predProgramLists.empty());

    // First, add the iterator itself.
    vector<u32> keys;
    for (const auto &elem : predProgramLists) {
        keys.push_back(elem.first);
    }
    DEBUG_PRINTF("%zu keys: %s\n", keys.size(), as_string_list(keys).c_str());

    vector<mmbit_sparse_iter> iter;
    mmbBuildSparseIterator(iter, keys, bc.numStates);
    assert(!iter.empty());
    u32 iter_offset = addIteratorToTable(bc, iter);

    // Construct our program, starting with the SPARSE_ITER_BEGIN
    // instruction, keeping track of the jump offset for each sub-program.
    vector<RoseInstruction> sparse_program;
    vector<u32> jump_table;

    sparse_program.push_back(RoseInstruction(ROSE_INSTR_SPARSE_ITER_BEGIN,
                                             JumpTarget::PROGRAM_END));
    u32 curr_offset = programLength(program) + programLength(sparse_program);

    for (const auto &e : predProgramLists) {
        DEBUG_PRINTF("subprogram %zu has offset %u\n", jump_table.size(),
                     curr_offset);
        jump_table.push_back(curr_offset);
        assert(!e.second.empty());
        auto subprog = flattenProgram(e.second);

        if (e.first != keys.back()) {
            // For all but the last subprogram, replace the END instruction
            // with a SPARSE_ITER_NEXT.
            assert(!subprog.empty());
            assert(subprog.back().code() == ROSE_INSTR_END);
            subprog.back() = RoseInstruction(ROSE_INSTR_SPARSE_ITER_NEXT,
                                             JumpTarget::PROGRAM_END);
        }

        curr_offset += programLength(subprog);
        insert(&sparse_program, end(sparse_program), subprog);
    }

    // Strip the END instruction from the last block.
    assert(sparse_program.back().code() == ROSE_INSTR_END);
    sparse_program.pop_back();

    sparse_program = flattenProgram({sparse_program});

    // Write the jump table into the bytecode.
    const u32 jump_table_offset =
        add_to_engine_blob(bc, begin(jump_table), end(jump_table));

    // Write jump table and iterator offset into sparse iter instructions.
    auto keys_it = begin(keys);
    for (auto &ri : sparse_program) {
        switch (ri.code()) {
        case ROSE_INSTR_SPARSE_ITER_BEGIN:
            ri.u.sparseIterBegin.iter_offset = iter_offset;
            ri.u.sparseIterBegin.jump_table = jump_table_offset;
            break;
        case ROSE_INSTR_SPARSE_ITER_NEXT:
            ri.u.sparseIterNext.iter_offset = iter_offset;
            ri.u.sparseIterNext.jump_table = jump_table_offset;
            assert(keys_it != end(keys));
            ri.u.sparseIterNext.state = *keys_it++;
            break;
        default:
            break;
        }
    }

    program.insert(end(program), begin(sparse_program), end(sparse_program));
}

static
void addPredBlocks(build_context &bc,
                   map<u32, vector<vector<RoseInstruction>>> &predProgramLists,
                   vector<RoseInstruction> &program) {
    const size_t num_preds = predProgramLists.size();
    if (num_preds == 0) {
        program = flattenProgram({program});
        return;
    }

    if (num_preds == 1) {
        addPredBlocksSingle(predProgramLists, program);
        return;
    }

    addPredBlocksMulti(bc, predProgramLists, program);
}

/**
 * Returns the pair (program offset, sparse iter offset).
 */
static
vector<RoseInstruction> makeSparseIterProgram(build_context &bc,
                    map<u32, vector<vector<RoseInstruction>>> &predProgramLists,
                    const vector<RoseInstruction> &root_program,
                    const vector<RoseInstruction> &pre_program) {
    vector<RoseInstruction> program;
    u32 curr_offset = 0;

    // Add pre-program first.
    for (const auto &ri : pre_program) {
        program.push_back(ri);
        curr_offset += ROUNDUP_N(ri.length(), ROSE_INSTR_MIN_ALIGN);
    }

    // Add blocks to deal with non-root edges (triggered by sparse iterator or
    // mmbit_isset checks). This operation will flatten the program up to this
    // point.
    addPredBlocks(bc, predProgramLists, program);

    // If we have a root program, replace the END instruction with it. Note
    // that the root program has already been flattened.
    assert(!program.empty());
    assert(program.back().code() == ROSE_INSTR_END);
    if (!root_program.empty()) {
        program.pop_back();
        program.insert(end(program), begin(root_program), end(root_program));
    }

    return program;
}

static
void makePushDelayedInstructions(const RoseBuildImpl &build, u32 final_id,
                                 vector<RoseInstruction> &program) {
    const auto &lit_infos = getLiteralInfoByFinalId(build, final_id);
    const auto &arb_lit_info = **lit_infos.begin();
    if (arb_lit_info.delayed_ids.empty()) {
        return;
    }

    for (const auto &int_id : arb_lit_info.delayed_ids) {
        const auto &child_literal = build.literals.right.at(int_id);
        u32 child_id = build.literal_info[int_id].final_id;
        u32 delay_index = child_id - build.delay_base_id;

        DEBUG_PRINTF("final_id=%u delay=%u child_id=%u\n", final_id,
                     child_literal.delay, child_id);

        auto ri = RoseInstruction(ROSE_INSTR_PUSH_DELAYED);
        ri.u.pushDelayed.delay = verify_u8(child_literal.delay);
        ri.u.pushDelayed.index = delay_index;
        program.push_back(move(ri));
    }
}

static
rose_group getFinalIdGroupsUnion(const RoseBuildImpl &build, u32 final_id) {
    assert(contains(build.final_id_to_literal, final_id));
    const auto &lit_infos = getLiteralInfoByFinalId(build, final_id);

    rose_group groups = 0;
    for (const auto &li : lit_infos) {
        groups |= li->group_mask;
    }
    return groups;
}

static
void makeGroupCheckInstruction(const RoseBuildImpl &build, u32 final_id,
                               vector<RoseInstruction> &program) {
    rose_group groups = getFinalIdGroupsUnion(build, final_id);
    if (!groups) {
        return;
    }

    auto ri = RoseInstruction(ROSE_INSTR_CHECK_GROUPS);
    ri.u.checkGroups.groups = groups;
    program.push_back(move(ri));
}

static
void makeCheckLitMaskInstruction(const RoseBuildImpl &build, u32 final_id,
                                 vector<RoseInstruction> &program) {
    assert(contains(build.final_id_to_literal, final_id));
    const auto &lit_infos = getLiteralInfoByFinalId(build, final_id);
    assert(!lit_infos.empty());

    if (!lit_infos.front()->requires_benefits) {
        return;
    }

    auto ri = RoseInstruction(ROSE_INSTR_CHECK_LIT_MASK);

    assert(build.final_id_to_literal.at(final_id).size() == 1);
    u32 lit_id = *build.final_id_to_literal.at(final_id).begin();
    const ue2_literal &s = build.literals.right.at(lit_id).s;
    DEBUG_PRINTF("building mask for lit %u (final id %u) %s\n", lit_id,
                 final_id, dumpString(s).c_str());
    assert(s.length() <= MAX_MASK2_WIDTH);
    u32 i = 0;
    for (const auto &e : s) {
        ri.u.checkLitMask.and_mask.a8[i] = e.nocase ? 0 : CASE_BIT;
        ri.u.checkLitMask.cmp_mask.a8[i] = e.nocase ? 0 : (CASE_BIT & e.c);
        i++;
    }

    program.push_back(move(ri));
}

static
void makeGroupSquashInstruction(const RoseBuildImpl &build, u32 final_id,
                                vector<RoseInstruction> &program) {
    assert(contains(build.final_id_to_literal, final_id));
    const auto &lit_infos = getLiteralInfoByFinalId(build, final_id);

    if (!lit_infos.front()->squash_group) {
        return;
    }

    rose_group groups = getFinalIdGroupsUnion(build, final_id);
    if (!groups) {
        return;
    }

    DEBUG_PRINTF("final_id %u squashes 0x%llx\n", final_id, groups);

    auto ri = RoseInstruction(ROSE_INSTR_SQUASH_GROUPS);
    ri.u.squashGroups.groups = ~groups; // Negated, so we can just AND it in.
    program.push_back(move(ri));
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
                                   vector<RoseInstruction> &program) {
    assert(contains(build.final_id_to_literal, final_id));
    const auto &lit_ids = build.final_id_to_literal.at(final_id);

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

    auto ri = RoseInstruction(ROSE_INSTR_RECORD_ANCHORED);
    ri.u.recordAnchored.id = final_id;
    program.push_back(move(ri));
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
                                  u32 final_id,
                                  const vector<RoseEdge> &lit_edges,
                                  vector<RoseInstruction> &program) {
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

    const auto &lit_ids = build.final_id_to_literal.at(final_id);
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

    DEBUG_PRINTF("final_id=%u has min_len=%zu, min_offset=%u, "
                 "global min is %u\n", final_id, min_len, min_offset,
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
    auto ri = RoseInstruction(ROSE_INSTR_CHECK_LIT_EARLY);
    ri.u.checkLitEarly.min_offset = min_offset;
    program.push_back(move(ri));
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
vector<RoseInstruction> buildLitInitialProgram(RoseBuildImpl &build,
                                    build_context &bc, u32 final_id,
                                    const vector<RoseEdge> &lit_edges) {
    vector<RoseInstruction> pre_program;

    // No initial program for EOD.
    if (final_id == MO_INVALID_IDX) {
        return pre_program;
    }

    DEBUG_PRINTF("final_id %u\n", final_id);

    // Check lit mask.
    makeCheckLitMaskInstruction(build, final_id, pre_program);

    // Check literal groups. This is an optimisation that we only perform for
    // delayed literals, as their groups may be switched off; ordinarily, we
    // can trust the HWLM matcher.
    if (hasDelayedLiteral(build, lit_edges)) {
        makeGroupCheckInstruction(build, final_id, pre_program);
    }

    // Add instructions for pushing delayed matches, if there are any.
    makePushDelayedInstructions(build, final_id, pre_program);

    // Add pre-check for early literals in the floating table.
    makeCheckLitEarlyInstruction(build, bc, final_id, lit_edges, pre_program);

    return pre_program;
}

static
vector<RoseInstruction> buildLiteralProgram(RoseBuildImpl &build,
                                            build_context &bc, u32 final_id,
                                            const vector<RoseEdge> &lit_edges) {
    const auto &g = build.g;

    DEBUG_PRINTF("final id %u, %zu lit edges\n", final_id, lit_edges.size());

    // pred state id -> list of programs
    map<u32, vector<vector<RoseInstruction>>> predProgramLists;

    // Construct sparse iter sub-programs.
    for (const auto &e : lit_edges) {
        const auto &u = source(e, g);
        if (build.isAnyStart(u)) {
            continue; // Root roles are not handled with sparse iterator.
        }
        DEBUG_PRINTF("sparse iter edge (%zu,%zu)\n", g[u].idx,
                     g[target(e, g)].idx);
        assert(contains(bc.roleStateIndices, u));
        u32 pred_state = bc.roleStateIndices.at(u);
        auto program = makeProgram(build, bc, e);
        if (program.empty()) {
            continue;
        }
        predProgramLists[pred_state].push_back(program);
    }

    // Construct sub-program for handling root roles.
    vector<vector<RoseInstruction>> root_programs;
    for (const auto &e : lit_edges) {
        const auto &u = source(e, g);
        if (!build.isAnyStart(u)) {
            continue;
        }
        DEBUG_PRINTF("root edge (%zu,%zu)\n", g[u].idx, g[target(e, g)].idx);
        auto role_prog = makeProgram(build, bc, e);
        if (role_prog.empty()) {
            continue;
        }
        root_programs.push_back(role_prog);
    }

    if (final_id != MO_INVALID_IDX) {
        vector<RoseInstruction> prog;

        // Literal may squash groups.
        makeGroupSquashInstruction(build, final_id, prog);

        // Literal may be anchored and need to be recorded.
        makeRecordAnchoredInstruction(build, bc, final_id, prog);

        if (!prog.empty()) {
            root_programs.push_back(move(prog));
        }
    }

    vector<RoseInstruction> root_program;
    if (!root_programs.empty()) {
        root_program = flattenProgram(root_programs);
    }

    auto pre_program = buildLitInitialProgram(build, bc, final_id, lit_edges);

    // Put it all together.
    return makeSparseIterProgram(bc, predProgramLists, root_program,
                                 pre_program);
}

static
u32 writeLiteralProgram(RoseBuildImpl &build, build_context &bc, u32 final_id,
                        const vector<RoseEdge> &lit_edges) {
    auto program = buildLiteralProgram(build, bc, final_id, lit_edges);
    if (program.empty()) {
        return 0;
    }
    // Note: already flattened.
    applyFinalSpecialisation(program);
    return writeProgram(bc, program);
}

static
u32 buildDelayRebuildProgram(RoseBuildImpl &build, build_context &bc,
                             u32 final_id) {
    const auto &lit_infos = getLiteralInfoByFinalId(build, final_id);
    const auto &arb_lit_info = **lit_infos.begin();
    if (arb_lit_info.delayed_ids.empty()) {
        return 0; // No delayed IDs, no work to do.
    }

    vector<RoseInstruction> program;
    makeCheckLitMaskInstruction(build, final_id, program);
    makePushDelayedInstructions(build, final_id, program);
    assert(!program.empty());
    program = flattenProgram({program});
    applyFinalSpecialisation(program);
    return writeProgram(bc, program);
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
                 return tie(g[source(a, g)].idx, g[target(a, g)].idx) <
                        tie(g[source(b, g)].idx, g[target(b, g)].idx);
             });
        lit_edge_map.emplace(m.first, edge_list);
    }

    return lit_edge_map;
}

/**
 * \brief Build the interpreter programs for each literal.
 *
 * Returns the base of the literal program list and the base of the delay
 * rebuild program list.
 */
static
pair<u32, u32> buildLiteralPrograms(RoseBuildImpl &build, build_context &bc) {
    const u32 num_literals = build.final_id_to_literal.size();
    auto lit_edge_map = findEdgesByLiteral(build);

    bc.litPrograms.resize(num_literals);
    vector<u32> delayRebuildPrograms(num_literals);

    for (u32 finalId = 0; finalId != num_literals; ++finalId) {
        const auto &lit_edges = lit_edge_map[finalId];

        bc.litPrograms[finalId] =
            writeLiteralProgram(build, bc, finalId, lit_edges);
        delayRebuildPrograms[finalId] =
            buildDelayRebuildProgram(build, bc, finalId);
    }

    u32 litProgramsOffset =
        add_to_engine_blob(bc, begin(bc.litPrograms), end(bc.litPrograms));
    u32 delayRebuildProgramsOffset = add_to_engine_blob(
        bc, begin(delayRebuildPrograms), end(delayRebuildPrograms));

    return {litProgramsOffset, delayRebuildProgramsOffset};
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

    vector<RoseInstruction> program;
    for (ReportID id : reports) {
        program.clear();
        const bool has_som = false;
        makeCatchupMpv(build, bc, id, program);
        makeReport(build, id, has_som, program);
        program = flattenProgram({program});
        applyFinalSpecialisation(program);
        u32 offset = writeProgram(bc, program);
        programs.push_back(offset);
        build.rm.setProgramOffset(id, offset);
        DEBUG_PRINTF("program for report %u @ %u (%zu instructions)\n", id,
                     programs.back(), program.size());
    }

    u32 offset = add_to_engine_blob(bc, begin(programs), end(programs));
    u32 count = verify_u32(programs.size());
    return {offset, count};
}

static
vector<RoseInstruction> makeEodAnchorProgram(RoseBuildImpl &build,
                                             build_context &bc,
                                             const RoseEdge &e,
                                             const bool multiple_preds) {
    const RoseGraph &g = build.g;
    const RoseVertex v = target(e, g);

    vector<RoseInstruction> program;

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
    for (const auto &id : reports) {
        makeReport(build, id, has_som, program);
    }

    return program;
}

static
bool hasEodAnchoredSuffix(const RoseBuildImpl &build) {
    const RoseGraph &g = build.g;
    for (auto v : vertices_range(g)) {
        if (g[v].suffix && build.isInETable(v)) {
            DEBUG_PRINTF("vertex %zu is in eod table and has a suffix\n",
                         g[v].idx);
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
            DEBUG_PRINTF("vertex %zu is in eod table\n", g[v].idx);
            return true;
        }
    }
    return false;
}

static
void addEodAnchorProgram(RoseBuildImpl &build, build_context &bc,
                         bool in_etable, vector<RoseInstruction> &program) {
    const RoseGraph &g = build.g;

    // pred state id -> list of programs
    map<u32, vector<vector<RoseInstruction>>> predProgramLists;

    for (auto v : vertices_range(g)) {
        if (!g[v].eod_accept) {
            continue;
        }

        DEBUG_PRINTF("vertex %zu (with %zu preds) fires on EOD\n", g[v].idx,
                     in_degree(v, g));

        vector<RoseEdge> edge_list;
        for (const auto &e : in_edges_range(v, g)) {
            RoseVertex u = source(e, g);
            if (build.isInETable(u) != in_etable) {
                DEBUG_PRINTF("pred %zu %s in etable\n", g[u].idx,
                             in_etable ? "is not" : "is");
                continue;
            }
            if (canEagerlyReportAtEod(build, e)) {
                DEBUG_PRINTF("already done report for vertex %zu\n", g[u].idx);
                continue;
            }
            edge_list.push_back(e);
        }

        const bool multiple_preds = edge_list.size() > 1;
        for (const auto &e : edge_list) {
            RoseVertex u = source(e, g);
            assert(contains(bc.roleStateIndices, u));
            u32 predStateIdx = bc.roleStateIndices.at(u);

            auto prog = makeEodAnchorProgram(build, bc, e, multiple_preds);
            if (prog.empty()) {
                continue;
            }
            predProgramLists[predStateIdx].push_back(prog);
        }
    }

    if (predProgramLists.empty()) {
        return;
    }
    if (!program.empty()) {
        assert(program.back().code() == ROSE_INSTR_END);
        program.pop_back();
    }
    addPredBlocks(bc, predProgramLists, program);
}

static
void addEodEventProgram(RoseBuildImpl &build, build_context &bc,
                        vector<RoseInstruction> &program) {
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
             return tie(g[source(a, g)].idx, g[target(a, g)].idx) <
                    tie(g[source(b, g)].idx, g[target(b, g)].idx);
         });

    auto prog = buildLiteralProgram(build, bc, MO_INVALID_IDX, edge_list);
    program.insert(end(program), begin(prog), end(prog));
}

static
void addEnginesEodProgram(u32 eodNfaIterOffset,
                          vector<RoseInstruction> &program) {
    if (!eodNfaIterOffset) {
        return;
    }

    auto ri = RoseInstruction(ROSE_INSTR_ENGINES_EOD);
    ri.u.enginesEod.iter_offset = eodNfaIterOffset;
    if (!program.empty()) {
        assert(program.back().code() == ROSE_INSTR_END);
        program.pop_back();
    }
    program.push_back(move(ri));
    program.emplace_back(ROSE_INSTR_END);
}

static
void addSuffixesEodProgram(const RoseBuildImpl &build,
                           vector<RoseInstruction> &program) {
    if (!hasEodAnchoredSuffix(build)) {
        return;
    }

    if (!program.empty()) {
        assert(program.back().code() == ROSE_INSTR_END);
        program.pop_back();
    }
    program.emplace_back(ROSE_INSTR_SUFFIXES_EOD);
    program.emplace_back(ROSE_INSTR_END);
}

static
void addMatcherEodProgram(const RoseBuildImpl &build,
                          vector<RoseInstruction> &program) {
    if (!hasEodMatcher(build)) {
        return;
    }

    if (!program.empty()) {
        assert(program.back().code() == ROSE_INSTR_END);
        program.pop_back();
    }
    program.emplace_back(ROSE_INSTR_MATCHER_EOD);
    program.emplace_back(ROSE_INSTR_END);
}

static
u32 writeEodProgram(RoseBuildImpl &build, build_context &bc,
                    u32 eodNfaIterOffset) {
    vector<RoseInstruction> program;

    addEodEventProgram(build, bc, program);
    addEnginesEodProgram(eodNfaIterOffset, program);
    addEodAnchorProgram(build, bc, false, program);
    addMatcherEodProgram(build, program);
    addEodAnchorProgram(build, bc, true, program);
    addSuffixesEodProgram(build, program);

    if (program.size() == 1) {
        assert(program.back().code() == ROSE_INSTR_END);
        return 0;
    }

    if (program.empty()) {
        return 0;
    }

    applyFinalSpecialisation(program);
    return writeProgram(bc, program);
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
    return addIteratorToTable(bc, iter);
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

aligned_unique_ptr<RoseEngine> RoseBuildImpl::buildFinalEngine(u32 minWidth) {
    DerivedBoundaryReports dboundary(boundary);

    size_t historyRequired = calcHistoryRequired(); // Updated by HWLM.

    auto anchored_dfas = buildAnchoredDfas(*this);

    build_context bc;
    bc.floatingMinLiteralMatchOffset =
        findMinFloatingLiteralMatch(*this, anchored_dfas);
    bc.needs_catchup = needsCatchup(*this);
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
    buildCountingMiracles(*this, bc);

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

    u32 litProgramOffset;
    u32 litDelayRebuildProgramOffset;
    tie(litProgramOffset, litDelayRebuildProgramOffset) =
        buildLiteralPrograms(*this, bc);

    u32 eodProgramOffset = writeEodProgram(*this, bc, eodNfaIterOffset);

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
        currOffset = bc.engine_blob_base + byte_length(bc.engine_blob);
    } else {
        currOffset = sizeof(RoseEngine);
    }

    UNUSED const size_t engineBlobSize =
        byte_length(bc.engine_blob); // test later

    currOffset = ROUNDUP_CL(currOffset);
    DEBUG_PRINTF("currOffset %u\n", currOffset);

    // Build anchored matcher.
    size_t asize = 0;
    u32 amatcherOffset = 0;
    auto atable = buildAnchoredMatcher(*this, anchored_dfas, bc.litPrograms,
                                       &asize);
    if (atable) {
        currOffset = ROUNDUP_CL(currOffset);
        amatcherOffset = currOffset;
        currOffset += verify_u32(asize);
    }

    // Build floating HWLM matcher.
    rose_group fgroups = 0;
    size_t fsize = 0;
    size_t floatingStreamStateRequired = 0;
    auto ftable = buildFloatingMatcher(*this, &fgroups, &fsize, &historyRequired,
                                       &floatingStreamStateRequired);
    u32 fmatcherOffset = 0;
    if (ftable) {
        currOffset = ROUNDUP_CL(currOffset);
        fmatcherOffset = currOffset;
        currOffset += verify_u32(fsize);
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
                     floatingStreamStateRequired, historyRequired,
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
    engine->invDkeyOffset = dkeyOffset;
    copy_bytes(ptr + dkeyOffset, rm.getDkeyToReportTable());

    engine->somHorizon = ssm.somPrecision();
    engine->somLocationCount = ssm.numSomSlots();

    engine->needsCatchup = bc.needs_catchup ? 1 : 0;

    engine->literalCount = verify_u32(final_id_to_literal.size());
    engine->litProgramOffset = litProgramOffset;
    engine->litDelayRebuildProgramOffset = litDelayRebuildProgramOffset;
    engine->reportProgramOffset = reportProgramOffset;
    engine->reportProgramCount = reportProgramCount;
    engine->runtimeImpl = pickRuntimeImpl(*this, bc, outfixEndQueue);
    engine->mpvTriggeredByLeaf = anyEndfixMpvTriggers(*this);

    engine->activeArrayCount = activeArrayCount;
    engine->activeLeftCount = activeLeftCount;
    engine->queueCount = queue_count;
    engine->eagerIterOffset = eagerIterOffset;
    engine->handledKeyCount = bc.handledKeys.size();

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

    u32 delay_count = verify_u32(final_id_to_literal.size() - delay_base_id);
    engine->delay_count = delay_count;
    engine->delay_base_id = delay_base_id;
    engine->anchored_base_id = anchored_base_id;
    engine->anchored_count = delay_base_id - anchored_base_id;

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
    engine->floatingStreamState = verify_u32(floatingStreamStateRequired);

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
    copy_bytes(ptr + bc.engine_blob_base, bc.engine_blob);
    copy_bytes(ptr + engine->leftOffset, leftInfoTable);

    fillLookaroundTables(ptr + lookaroundTableOffset,
                         ptr + lookaroundReachOffset, bc.lookaround);

    fillInSomRevNfas(engine.get(), ssm, rev_nfa_table_offset, rev_nfa_offsets);
    copy_bytes(ptr + engine->activeLeftIterOffset, activeLeftIter);

    // Safety check: we shouldn't have written anything to the engine blob
    // after we copied it into the engine bytecode.
    assert(byte_length(bc.engine_blob) == engineBlobSize);

    // Add a small write engine if appropriate.
    engine = addSmallWriteEngine(*this, move(engine));

    DEBUG_PRINTF("rose done %p\n", engine.get());
    return engine;
}

} // namespace ue2
