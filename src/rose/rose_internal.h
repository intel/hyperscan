/*
 * Copyright (c) 2015-2019, Intel Corporation
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
 * \brief Rose data structures.
 */

#ifndef ROSE_INTERNAL_H
#define ROSE_INTERNAL_H

#include "ue2common.h"
#include "rose_common.h"
#include "util/scatter.h"

#define ROSE_OFFSET_INVALID          0xffffffff

// Group constants
typedef u64a rose_group;

// Delayed literal stuff
#define DELAY_BITS                  5
#define DELAY_SLOT_COUNT            (1U << DELAY_BITS)
#define MAX_DELAY                   (DELAY_SLOT_COUNT - 1)
#define DELAY_MASK                  (DELAY_SLOT_COUNT - 1)

/* Allocation of Rose literal ids
 *
 * The rose literal id space is segmented:
 *
 * ---- 0
 * |  | 'Normal' undelayed literals in either e or f tables
 * |  |
 * |  |
 * |  |
 * ---- anchored_base_id
 * |  | literals from the a table
 * |  |
 * ---- delay_base_id
 * |  | Delayed version of normal literals
 * |  |
 * ---- literalCount
 */

/* Rose Literal Sources
 *
 * Rose currently gets events (mainly roseProcessMatch calls) from a number of
 * sources:
 * 1) The floating table
 * 2) The anchored table
 * 3) Delayed literals
 * 4) Suffix NFAs
 * 5) Literal masks
 * 5) End anchored table
 * 6) Prefix / Infix nfas
 *
 * Care is required to ensure that events appear to come into Rose in order
 * (or sufficiently ordered for Rose to cope). Generally the progress of the
 * floating table is considered the canonical position in the buffer.
 *
 * Anchored table:
 * The anchored table is run before the floating table as nothing in it can
 * depend on a floating literal. Order is achieved by two measures:
 * a) user matches^1 are logged and held until the floating matcher passes that
 *    point;
 * b) any floating role with an anchored predecessor has a history relationship
 *    to enforce the ordering.
 *
 * Delayed literals:
 * Delayed literal ordering is handled by delivering any pending delayed
 * literals before processing any floating match.
 *
 * Suffix:
 * Suffixes are always pure terminal roles. Prior to raising a match^2, pending
 * NFA queues are run to the current point (floating or delayed literal) as
 * appropriate.
 *
 * Literal Masks:
 * These are triggered from either floating literals or delayed literals and
 * inspect the data behind them. Matches are raised at the same location as the
 * trigger literal so there are no ordering issues. Masks are always pure
 * terminal roles.
 *
 * Lookaround:
 * These are tests run on receipt of a role that "look around" the match,
 * checking characters at nearby offsets against reachability masks. Each role
 * can have a list of these lookaround offset/reach pairs, ordered in offset
 * order, and any failure will prevent the role from being switched on. Offsets
 * are relative to the byte after a literal match, and can be negative.
 *
 * Prefix / Infix:
 * TODO: remember / discuss
 *
 * End anchored table:
 * All user matches occur at the last byte. We do this last, so no problems
 * (yippee)
 *
 * ^1 User matches which occur before any possible match from the other tables
 *    are not delayed.
 * ^2 Queues may also be run to the current location if a queue is full and
 *    needs to be emptied.
 * ^3 There is no need to catch up at the end of a block scan as it contains no
 *    terminals.
 */

struct RoseCountingMiracle {
    char shufti; /** 1: count shufti class; 0: count a single character */
    u8 count; /** minimum number of occurrences for the counting
               * miracle char to kill the leftfix. */
    u8 c; /** character to look for if not shufti */
    u8 poison; /** character not in the shufti mask */
    m128 lo; /** shufti lo mask */
    m128 hi; /** shufti hi mask */
};

struct LeftNfaInfo {
    u32 maxQueueLen;
    u32 maxLag; // maximum of successor roles' lag
    u32 lagIndex; // iff lag != 0, index into leftfixLagTable
    u32 stopTable; // stop table index, or ROSE_OFFSET_INVALID
    u8 transient; /**< 0 if not transient, else max width of transient prefix */
    char infix; /* TODO: make flags */
    char eager; /**< nfa should be run eagerly to first match or death */
    char eod_check; /**< nfa is used by the event eod literal */
    u32 countingMiracleOffset; /** if not 0, offset to RoseCountingMiracle. */
    rose_group squash_mask; /* & mask applied when rose nfa dies */
};

struct NfaInfo {
    u32 nfaOffset;
    u32 stateOffset;
    u32 fullStateOffset; /* offset in scratch, relative to ??? */
    u32 ekeyListOffset; /* suffix, relative to base of rose, 0 if no ekeys */
    u8 no_retrigger; /* TODO */
    u8 in_sbmatcher;  /**< this outfix should not be run in small-block
                       * execution, as it will be handled by the sbmatcher
                       * HWLM table. */
    u8 eod; /* suffix is triggered by the etable --> can only produce eod
             * matches */
};

#define MAX_STORED_LEFTFIX_LAG 127 /* max leftfix lag that we can store in one
                                    * whole byte (OWB) (streaming only). Other
                                    * values in OWB are reserved for zombie
                                    * status */
#define OWB_ZOMBIE_ALWAYS_YES 128 /* nfa will always answer yes to any rose
                                   * prefix checks */

/* offset of the status flags in the stream state. */
#define ROSE_STATE_OFFSET_STATUS_FLAGS 0

/* offset of role mmbit in stream state (just after the status flag byte). */
#define ROSE_STATE_OFFSET_ROLE_MMBIT   sizeof(u8)

/**
 * \brief Rose state offsets.
 *
 * Stores pre-calculated offsets (in bytes) to MOST of the state structures
 * used by Rose, relative to the start of stream state.
 *
 * State not covered by this structure includes:
 *
 * -# the first byte, containing the status bitmask
 * -# the role state multibit
 */
struct RoseStateOffsets {
    /** History buffer.
     *
     * Max size of history is RoseEngine::historyRequired. */
    u32 history;

    /** Exhausted multibit.
     *
     * entry per exhaustible key (used by Highlander mode). If a bit is set,
     * reports with that ekey should not be delivered to the user. */
    u32 exhausted;

    /** size in bytes of exhausted multibit */
    u32 exhausted_size;

    /** Logical multibit.
     *
     * entry per logical key(operand/operator) (used by Logical Combination). */
    u32 logicalVec;

    /** size in bytes of logical multibit */
    u32 logicalVec_size;

    /** Combination multibit.
     *
     * entry per combination key (used by Logical Combination). */
    u32 combVec;

    /** size in bytes of combination multibit */
    u32 combVec_size;

    /** Multibit for active suffix/outfix engines. */
    u32 activeLeafArray;

    /** Size of multibit for active suffix/outfix engines in bytes. */
    u32 activeLeafArray_size;

    /** Multibit for active leftfix (prefix/infix) engines. */
    u32 activeLeftArray;

    /** Size of multibit for active leftfix (prefix/infix) engines in bytes. */
    u32 activeLeftArray_size;

    /** Table of lag information (stored as one byte per engine) for active
     * Rose leftfix engines. */
    u32 leftfixLagTable;

    /** State for anchored matchers (McClellan DFAs). */
    u32 anchorState;

    /** Packed Rose groups value. */
    u32 groups;

    /** Size of packed Rose groups value, in bytes. */
    u32 groups_size;

    /** State for long literal support. */
    u32 longLitState;

    /** Size of the long literal state. */
    u32 longLitState_size;

    /** Packed SOM location slots. */
    u32 somLocation;

    /** Multibit guarding SOM location slots. */
    u32 somValid;

    /** Multibit guarding SOM location slots. */
    u32 somWritable;

    /** Size of each of the somValid and somWritable multibits, in bytes. */
    u32 somMultibit_size;

    /** Begin of the region where NFA engine state is stored.
     * The NFA state region extends to end. */
    u32 nfaStateBegin;

    /** Total size of Rose state, in bytes. */
    u32 end;
};

struct RoseBoundaryReports {
    /** \brief 0 if no reports list, otherwise offset of program to run to
     * deliver reports at EOD. */
    u32 reportEodOffset;

    /** \brief 0 if no reports list, otherwise offset of program to run to
     * deliver reports at offset 0. */
    u32 reportZeroOffset;

    /** \brief 0 if no reports list, otherwise offset of program to run to
     * deliver reports if EOD is at offset 0. Superset of other programs. */
    u32 reportZeroEodOffset;
};

/* NFA Queue Assignment
 *
 * --- 0
 * (|) chained mpv (if present)
 *  #
 * --- outfixBeginQueue -
 *  | outfixes. enabled at offset 0.
 *  |
 *  #
 * --- outfixEndQueue -
 *  | suffixes. enabled by rose roles.
 *  |
 *  #
 * --- leftfixBeginQueue -
 *  | prefixes
 *  |
 *  #
 * --- ?
 *  | infixes
 *  |
 *  #
 */

#define ROSE_RUNTIME_FULL_ROSE     0
#define ROSE_RUNTIME_PURE_LITERAL  1
#define ROSE_RUNTIME_SINGLE_OUTFIX 2

/**
 * \brief Runtime structure header for Rose.
 *
 * Runtime structure header for Rose.
 * In memory, we follow this with:
 *   -# the "engine blob"
 *   -# anchored 'literal' matcher table
 *   -# floating literal matcher table
 *   -# eod-anchored literal matcher table
 *   -# small block table
 *   -# array of NFA offsets, one per queue
 *   -# array of state offsets, one per queue (+)
 *
 *  (+) stateOffset array note: Offsets in the array are either into the stream
 *  state (normal case) or into the tstate region of scratch (for transient rose
 *  nfas). Rose nfa info table can distinguish the cases.
 */
struct RoseEngine {
    u8  pureLiteral; /* Indicator of pure literal API */
    u8  noFloatingRoots; /* only need to run the anchored table if something
                          * matched in the anchored table */
    u8  requiresEodCheck; /* stuff happens at eod time */
    u8  hasOutfixesInSmallBlock; /**< has at least one outfix that must run even
                                    in small block scans. */
    u8  runtimeImpl; /**< can we just run the floating table or a single outfix?
                      * or do we need a full rose? */
    u8  mpvTriggeredByLeaf; /**< need to check (suf|out)fixes for mpv trigger */
    u8  canExhaust; /**< every pattern has an exhaustion key */
    u8  hasSom; /**< has at least one pattern which tracks SOM. */
    u8  somHorizon; /**< width in bytes of SOM offset storage (governed by
                        SOM precision) */
    u32 mode; /**< scanning mode, one of HS_MODE_{BLOCK,STREAM,VECTORED} */
    u32 historyRequired; /**< max amount of history required for streaming */
    u32 ekeyCount; /**< number of exhaustion keys */
    u32 lkeyCount; /**< number of logical keys */
    u32 lopCount; /**< number of logical ops */
    u32 ckeyCount; /**< number of combination keys */
    u32 logicalTreeOffset; /**< offset to mapping from lkey to LogicalOp */
    u32 combInfoMapOffset; /**< offset to mapping from ckey to combInfo */
    u32 dkeyCount; /**< number of dedupe keys */
    u32 dkeyLogSize; /**< size of fatbit for storing dkey log (bytes) */
    u32 invDkeyOffset; /**< offset to table mapping from dkeys to the external
                         *  report ids */
    u32 somLocationCount; /**< number of som locations required */
    u32 somLocationFatbitSize; /**< size of SOM location fatbit (bytes) */
    u32 rolesWithStateCount; // number of roles with entries in state bitset
    u32 stateSize; /* size of the state bitset
                    * WARNING: not the size of the rose state */
    u32 anchorStateSize; /* size of the state for the anchor dfas */
    u32 tStateSize; /* total size of the state for transient rose nfas */
    u32 scratchStateSize; /**< uncompressed state req'd for NFAs in scratch;
                           * used for sizing scratch only. */
    u32 smallWriteOffset; /**< offset of small-write matcher */
    u32 amatcherOffset; // offset of the anchored literal matcher (bytes)
    u32 ematcherOffset; // offset of the eod-anchored literal matcher (bytes)
    u32 fmatcherOffset; // offset of the floating literal matcher (bytes)
    u32 drmatcherOffset; // offset of the delayed rebuild table (bytes)
    u32 sbmatcherOffset; // offset of the small-block literal matcher (bytes)
    u32 longLitTableOffset; // offset of the long literal table
    u32 amatcherMinWidth; /**< minimum number of bytes required for a pattern
                           * involved with the anchored table to produce a full
                           * match. */
    u32 fmatcherMinWidth; /**< minimum number of bytes required for a pattern
                           * involved with the floating table to produce a full
                           * match. */
    u32 eodmatcherMinWidth; /**< minimum number of bytes required for a pattern
                               * involved with the eod table to produce a full
                               * match. */
    u32 amatcherMaxBiAnchoredWidth; /**< maximum number of bytes that can still
                                     * produce a match for a pattern involved
                                     * with the anchored table. */
    u32 fmatcherMaxBiAnchoredWidth; /**< maximum number of bytes that can still
                                     * produce a match for a pattern involved
                                     * with the anchored table. */

    /**
     * \brief Offset of u32 array of program offsets for reports used by
     * output-exposed engines.
     */
    u32 reportProgramOffset;

    /**
     * \brief Number of programs for reports used by output-exposed engines.
     */
    u32 reportProgramCount;

    /**
     * \brief Offset of u32 array of program offsets for delayed replay of
     * literals.
     */
    u32 delayProgramOffset;

    /**
     * \brief Offset of u32 array of program offsets for anchored literals.
     */
    u32 anchoredProgramOffset;

    u32 activeArrayCount; //number of nfas tracked in the active array
    u32 activeLeftCount; //number of nfas tracked in the active rose array
    u32 queueCount;      /**< number of nfa queues */
    u32 activeQueueArraySize; //!< size of fatbit for active queues (bytes)

    u32 eagerIterOffset; /**< offset to sparse iter for eager prefixes or 0 if
                          * none */

    /** \brief Number of keys used by CHECK_SET_HANDLED instructions in role
     * programs. */
    u32 handledKeyCount;

    /** \brief Size of the handled keys fatbit in scratch (bytes). */
    u32 handledKeyFatbitSize;

    u32 leftOffset;
    u32 roseCount;

    u32 eodProgramOffset; //!< EOD program, otherwise 0.
    u32 flushCombProgramOffset; /**< FlushCombination program, otherwise 0 */
    u32 lastFlushCombProgramOffset; /**< LastFlushCombination program,
                                     * otherwise 0 */

    u32 lastByteHistoryIterOffset; // if non-zero

    /** \brief Minimum number of bytes required to match. */
    u32 minWidth;

    /** \brief Minimum number of bytes required to match, excluding boundary
     * reports. */
    u32 minWidthExcludingBoundaries;

    u32 maxBiAnchoredWidth; /* ROSE_BOUND_INF if any non bianchored patterns
                             * present */
    u32 anchoredDistance; // region to run the anchored table over
    u32 anchoredMinDistance; /* start of region to run anchored table over */
    u32 floatingDistance; /* end of region to run the floating table over
                             ROSE_BOUND_INF if not bounded */
    u32 floatingMinDistance; /* start of region to run floating table over */
    u32 smallBlockDistance; /* end of region to run the floating table over
                               ROSE_BOUND_INF if not bounded */
    u32 floatingMinLiteralMatchOffset; /* the minimum offset that we can get a
                                        * 'valid' match from the floating
                                        * table */
    u32 nfaInfoOffset; /* offset to the nfa info offset array */
    rose_group initialGroups;
    rose_group floating_group_mask; /* groups that are used by the ftable */
    u32 size; // (bytes)
    u32 delay_count; /* number of delayed literal ids. */
    u32 delay_fatbit_size; //!< size of each delay fatbit in scratch (bytes)
    u32 anchored_count; /* number of anchored literal ids */
    u32 anchored_fatbit_size; //!< size of each anch fatbit in scratch (bytes)
    u32 maxFloatingDelayedMatch; /* max offset that a delayed literal can
                                  * usefully be reported */
    u32 delayRebuildLength; /* length of the history region which needs to be
                             * rescanned when we are doing a delayed literal
                             * rebuild scan. */
    struct RoseStateOffsets stateOffsets;
    struct RoseBoundaryReports boundary;
    u32 totalNumLiterals; /* total number of literals including dr */
    u32 asize; /* size of the atable */
    u32 outfixBeginQueue; /* first outfix queue */
    u32 outfixEndQueue; /* one past the last outfix queue */
    u32 leftfixBeginQueue; /* first prefix/infix queue */
    u32 initMpvNfa; /* (allegedly chained) mpv to force on at init */
    u32 rosePrefixCount; /* number of rose prefixes */
    u32 activeLeftIterOffset; /* mmbit_sparse_iter over non-transient roses */
    u32 ematcherRegionSize; /* max region size to pass to ematcher */
    u32 somRevCount; /**< number of som reverse nfas */
    u32 somRevOffsetOffset; /**< offset to array of offsets to som rev nfas */
    u32 longLitStreamState; // size in bytes

    struct scatter_full_plan state_init;
};

struct ALIGN_CL_DIRECTIVE anchored_matcher_info {
    u32 next_offset; /* relative to this, 0 for end */
    u32 state_offset; /* relative to anchorState */
    u32 anchoredMinDistance; /* start of region to run anchored table over */
};

/**
 * \brief Long literal subtable for a particular mode (caseful or nocase).
 */
struct RoseLongLitSubtable {
    /**
     * \brief Offset of the hash table (relative to RoseLongLitTable base).
     *
     * Offset is zero if no such table exists.
     */
    u32 hashOffset;

    /**
     * \brief Offset of the bloom filter (relative to RoseLongLitTable base).
     *
     * Offset is zero if no such table exists.
     */
    u32 bloomOffset;

    /** \brief lg2 of the size of the hash table. */
    u8 hashBits;

    /** \brief Size of the bloom filter in bits. */
    u8 bloomBits;

    /** \brief Number of bits of packed stream state used.  */
    u8 streamStateBits;
};

/**
 * \brief Long literal table header.
 */
struct RoseLongLitTable {
    /**
     * \brief Total size of the whole table (including strings, bloom filters,
     * hash tables).
     */
    u32 size;

    /** \brief Caseful sub-table (hash table and bloom filter). */
    struct RoseLongLitSubtable caseful;

    /** \brief Caseless sub-table (hash table and bloom filter). */
    struct RoseLongLitSubtable nocase;

    /** \brief Total size of packed stream state in bytes. */
    u8 streamStateBytes;

    /** \brief Max length of literal prefixes. */
    u8 maxLen;
};

/**
 * \brief One of these structures per hash table entry in our long literal
 * table.
 */
struct RoseLongLitHashEntry {
    /**
     * \brief Offset of the literal string itself, relative to
     * RoseLongLitTable base. Zero if this bucket is empty.
     */
    u32 str_offset;

    /** \brief Length of the literal string. */
    u32 str_len;
};

static really_inline
const struct anchored_matcher_info *getALiteralMatcher(
        const struct RoseEngine *t) {
    if (!t->amatcherOffset) {
        return NULL;
    }

    const char *lt = (const char *)t + t->amatcherOffset;
    assert(ISALIGNED_CL(lt));
    return (const struct anchored_matcher_info *)lt;
}

struct HWLM;

static really_inline
const struct HWLM *getFLiteralMatcher(const struct RoseEngine *t) {
    if (!t->fmatcherOffset) {
        return NULL;
    }

    const char *lt = (const char *)t + t->fmatcherOffset;
    assert(ISALIGNED_CL(lt));
    return (const struct HWLM *)lt;
}

static really_inline
const void *getSBLiteralMatcher(const struct RoseEngine *t) {
    if (!t->sbmatcherOffset) {
        return NULL;
    }

    const char *matcher = (const char *)t + t->sbmatcherOffset;
    assert(ISALIGNED_N(matcher, 8));
    return matcher;
}

static really_inline
const struct LeftNfaInfo *getLeftTable(const struct RoseEngine *t) {
    const struct LeftNfaInfo *r
        = (const struct LeftNfaInfo *)((const char *)t + t->leftOffset);
    assert(ISALIGNED_N(r, 4));
    return r;
}

struct mmbit_sparse_iter; // forward decl

static really_inline
const struct mmbit_sparse_iter *getActiveLeftIter(const struct RoseEngine *t) {
    assert(t->activeLeftIterOffset);
    const struct mmbit_sparse_iter *it = (const struct mmbit_sparse_iter *)
            ((const char *)t + t->activeLeftIterOffset);
    assert(ISALIGNED_N(it, 4));
    return it;
}

static really_inline
const struct NfaInfo *getNfaInfoByQueue(const struct RoseEngine *t, u32 qi) {
    const struct NfaInfo *infos
        = (const struct NfaInfo *)((const char *)t + t->nfaInfoOffset);
    assert(ISALIGNED_N(infos, sizeof(u32)));

    return &infos[qi];
}

static really_inline
const struct NFA *getNfaByInfo(const struct RoseEngine *t,
                               const struct NfaInfo *info) {
    return (const struct NFA *)((const char *)t + info->nfaOffset);
}

static really_inline
const struct NFA *getNfaByQueue(const struct RoseEngine *t, u32 qi) {
    const struct NfaInfo *info = getNfaInfoByQueue(t, qi);
    return getNfaByInfo(t, info);
}

static really_inline
u32 queueToLeftIndex(const struct RoseEngine *t, u32 qi) {
    assert(qi >= t->leftfixBeginQueue);
    return qi - t->leftfixBeginQueue;
}

static really_inline
const struct LeftNfaInfo *getLeftInfoByQueue(const struct RoseEngine *t,
                                             u32 qi) {
    const struct LeftNfaInfo *infos = getLeftTable(t);
    return &infos[queueToLeftIndex(t, qi)];
}

struct SmallWriteEngine;

static really_inline
const struct SmallWriteEngine *getSmallWrite(const struct RoseEngine *t) {
    if (!t->smallWriteOffset) {
        return NULL;
    }

    const struct SmallWriteEngine *smwr =
        (const struct SmallWriteEngine *)((const char *)t + t->smallWriteOffset);
    return smwr;
}

#endif // ROSE_INTERNAL_H
