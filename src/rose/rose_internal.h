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

#define DELAY_FLOAT_DIRTY      (1U << 7) /* delay literal matched in history */

// Direct report stuff
#define LITERAL_DR_FLAG   (1U << 31)
#define LITERAL_MDR_FLAG  ((1U << 30) | (1U << 31))

/** \brief True if literal is either a direct report or a multi-direct report.
 * */
static really_inline
u32 isLiteralDR(u32 id) {
    return id & LITERAL_DR_FLAG;
}

static really_inline
u32 isLiteralMDR(u32 id) {
    return (id & LITERAL_MDR_FLAG) == LITERAL_MDR_FLAG;
}

static really_inline
ReportID literalToReport(u32 id) {
    assert(id & LITERAL_DR_FLAG);
    assert(!(id & (LITERAL_MDR_FLAG ^ LITERAL_DR_FLAG)));
    return id & ~LITERAL_DR_FLAG;
}

// Structure representing a literal. Each literal may have many roles.
struct RoseLiteral {
    u32 rootRoleOffset; /**< If rootRoleCount == 1, this is an offset relative
                         * to the rose engine to the root role associated with
                         * the literal.
                         * If rootRoleCount > 1, this is the first index into
                         * the rootRoleTable indicating the root roles.
                         */
    u32 rootRoleCount; // number of root roles
    u32 iterOffset; // offset of sparse iterator, relative to rose
    u32 iterMapOffset; // offset of the iter mapping table, relative to rose
    rose_group groups; // bitset of groups that cause this literal to fire.
    u8 minDepth; // the minimum of this literal's roles' depths (for depths > 1)
    u8 squashesGroup; /**< literal switches off its group behind it if it sets a
                       * role */
    u8 requires_side; // need to catch up sidecar for this literal
    u32 delay_mask; /**< bit set indicates that the literal inserts a delayed
                     * match at the given offset */
    u32 delayIdsOffset; // offset to array of ids to poke in the delay structure
};

/* properties for sidecar entries, yay */
struct RoseSide {
    u32 squashIterOffset; // offset of the squash sparse iterator, rose relative
    rose_group squashGroupMask; // squash literal squash masks
};

/* Allocation of Rose literal ids
 *
 * The rose literal id space is segmented:
 *
 * ---- 0
 * |  | Normal undelayed literals in the e, or f tables which require a
 * |  | manual benefits confirm on match [a table never requires benefits]
 * |  |
 * ---- nonbenefits_base_id
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
 * ...
 * ...
 * ...
 * ---- LITERAL_DR_FLAG
 * |  | Direct Report literals: immediately raise an internal report with id
 * |  | given by (lit_id & ~LITERAL_DR_FLAG). Raised by a or f tables (or e??).
 * |  | No RoseLiteral structure
 * |  |
 * |  |
 * ----
 *
 * Note: sidecar 'literals' are in a complete separate space
 */

/* Rose Literal Sources
 *
 * Rose currently gets events (mainly roseProcessMatch calls) from 8 sources:
 * 1) The floating table
 * 2) The anchored table
 * 3) Delayed literals
 * 4) Sidecar literal matcher
 * 5) suffixes NFAs
 * 6) masksv2 (literals with benefits)
 * 7) End anchored table
 * 8) prefix / infix nfas
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
 * Sidecar:
 * The sidecar matcher is unique in that it does not return match
 * location information. Sidecar literals are escapes between two normal
 * roles. The sidecar matcher is caught up to the floating matcher
 * before any possible predecessor role, any possible successor role, and
 * at stream boundaries^3.
 *
 * Suffix:
 * Suffixes are always pure terminal roles. Prior to raising a match^2, pending
 * NFA queues are run to the current point (floating or delayed literal) as
 * appropriate.
 *
 * Maskv2:
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

// We have different types of role history storage.
enum RoseRoleHistory {
    ROSE_ROLE_HISTORY_NONE, // I'm sorry, I don't recall.
    ROSE_ROLE_HISTORY_ANCH, // used when previous role is at a fixed offset
    ROSE_ROLE_HISTORY_LAST_BYTE, /* used when previous role can only match at the
                                  * last byte of a stream */
    ROSE_ROLE_HISTORY_INVALID // history not yet assigned
};

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
    char eod_check; /**< nfa is used by the event eod literal */
    u32 countingMiracleOffset; /** if not 0, offset to RoseCountingMiracle. */
    rose_group squash_mask; /* & mask applied when rose nfa dies */
};

// A list of these is used to trigger prefix/infix roses.
struct RoseTrigger {
    u32 queue; // queue index of leftfix
    u32 event; // queue event, from MQE_*
    u8 cancel_prev_top;
};

struct NfaInfo {
    u32 nfaOffset;
    u32 stateOffset;
    u32 fullStateOffset; /* offset in scratch, relative to ??? */
    u32 ekeyListOffset; /* suffix, relative to base of rose, 0 if no ekeys */
    u8 no_retrigger; /* TODO */
    u8 only_external; /**< does not raise any som internal events or chained
                       * rose events */
    u8 in_sbmatcher;  /**< this outfix should not be run in small-block
                       * execution, as it will be handled by the sbmatcher
                       * HWLM table. */
    u8 eod; /* suffix is triggered by the etable --> can only produce eod
             * matches */
};

#define ROSE_ROLE_FLAG_ANCHOR_TABLE  (1U << 0)  /**< role is triggered from
                                                 * anchored table */
#define ROSE_ROLE_FLAG_ACCEPT_EOD    (1U << 2)  /**< "fake" role, fires callback
                                                 * at EOD */
#define ROSE_ROLE_FLAG_ONLY_AT_END   (1U << 3)  /**< role can only be switched on
                                                 * at end of block */
#define ROSE_ROLE_FLAG_PRED_OF_EOD   (1U << 4)  /**< eod is a successor literal
                                                 * of the role */
#define ROSE_ROLE_FLAG_EOD_TABLE     (1U << 5)  /**< role is triggered from eod
                                                 * table */
#define ROSE_ROLE_FLAG_ROSE          (1U << 6)  /**< rose style prefix nfa for
                                                 * role */
#define ROSE_ROLE_FLAG_SOM_REPORT    (1U << 7)  /**< report id is only used to
                                                 * manipulate som */
#define ROSE_ROLE_FLAG_REPORT_START  (1U << 8)  /**< som som som som */
#define ROSE_ROLE_FLAG_CHAIN_REPORT  (1U << 9)  /**< report id is only used to
                                                 * start an outfix engine */
#define ROSE_ROLE_FLAG_SOM_ADJUST    (1U << 10) /**< som value to use is offset
                                                 * from match end location */
#define ROSE_ROLE_FLAG_SOM_ROSEFIX   (1U << 11) /**< som value to use is provided
                                                 * by prefix/infix */

/* We allow different types of role-predecessor relationships. These are stored
 * in with the flags */
#define ROSE_ROLE_PRED_NONE         (1U << 20) /**< the only pred is the root,
                                                * [0, inf] bounds */
#define ROSE_ROLE_PRED_SIMPLE       (1U << 21) /**< single [0,inf] pred, no
                                                * offset tracking */
#define ROSE_ROLE_PRED_ROOT         (1U << 22) /**< pred is root or anchored
                                                * root, and we have bounds */
#define ROSE_ROLE_PRED_ANY          (1U << 23) /**< any of our preds can match */

#define ROSE_ROLE_PRED_CLEAR_MASK (~(ROSE_ROLE_PRED_NONE       \
                                    | ROSE_ROLE_PRED_SIMPLE    \
                                    | ROSE_ROLE_PRED_ROOT      \
                                    | ROSE_ROLE_PRED_ANY))

#define MAX_STORED_LEFTFIX_LAG 127 /* max leftfix lag that we can store in one
                                    * whole byte (OWB) (streaming only). Other
                                    * values in OWB are reserved for zombie
                                    * status */
#define OWB_ZOMBIE_ALWAYS_YES 128 /* nfa will always answer yes to any rose
                                   * prefix checks */

// Structure representing a literal role.
struct RoseRole {
    u32 flags;
    u32 predOffset; // either offset of pred sparse iterator, or
                    // (for ROSE_ROLE_PRED_ROOT) index of single RosePred.
    rose_group groups; /**< groups to enable when role is set (groups of succ
                        *  literals) */
    ReportID reportId; // report ID, or MO_INVALID_IDX
    u32 stateIndex; /**< index into state multibit, or MMB_INVALID. Roles do not
                     * require a state bit if they are terminal */
    u32 suffixEvent; // queue event, from MQE_
    u8 depth; /**< depth of this vertex from root in the tree, or 255 if greater.
               */
    u32 suffixOffset; /**< suffix nfa: 0 if no suffix associated with the role,
                       *  relative to base of the rose. */
    ReportID leftfixReport; // (pre|in)fix report to check, or MO_INVALID_IDX.
    u32 leftfixLag; /**< distance behind match where we need to check the
                     * leftfix engine status */
    u32 leftfixQueue; /**< queue index of the prefix/infix before role */
    u32 infixTriggerOffset; /* offset to list of infix roses to trigger */
    u32 sidecarEnableOffset; /**< offset to list of sidecar literals to enable
                              */
    u32 somAdjust; /**< som for the role is offset from end match offset */

    u32 lookaroundIndex; /**< index of lookaround offset/reach in table, or
                          * MO_INVALID_IDX. */
    u32 lookaroundCount; /**< number of lookaround entries. */
};

// Structure representing a predecessor relationship
struct RosePred {
    u32 role; // index of predecessor role
    u32 minBound; // min bound on distance from pred (_ANCH ->absolute offset)
    u32 maxBound; /* max bound on distance from pred, or ROSE_BOUND_INF
                   * (_ANCH -> absolute offset ) */
    u8 historyCheck; // from enum RoseRoleHistory
};

// Structure mapping between the dense index produced by the literal sparse
// iterator and a list of roles.
struct RoseIterMapping {
    u32 offset; // offset into iter role table
    u32 count; // number of roles
};

struct RoseIterRole {
    u32 role;
    u32 pred;
};

/**
 * \brief Rose state offsets.
 *
 * Stores pre-calculated offsets (in bytes) to MOST of the state structures
 * used by Rose, relative to the start of stream state.
 *
 * State not covered by this structure includes:
 *
 * -# the RoseRuntimeState structure
 * -# the role state multibit
 */
struct RoseStateOffsets {
    /** History buffer.
     *
     * First byte is an 8-bit count of the number of valid history bytes
     * available, followed by the history itself. Max size of history is
     * RoseEngine::historyRequired. */
    u32 history;

    /** Exhausted bitvector.
     *
     * 1 bit per exhaustible key (used by Highlander mode). If a bit is set,
     * reports with that ekey should not be delivered to the user. */
    u32 exhausted;

    /** Sidecar state. */
    u32 sidecar;

    /** Size of sidecar state, in bytes. */
    u32 sidecar_size;

    /** Multibit for active suffix/outfix engines. */
    u32 activeLeafArray;

    /** Multibit for active Rose (prefix/infix) engines. */
    u32 activeLeftArray;

    /** Size of the active Rose array multibit, in bytes. */
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

    /** State for floating literal matcher (managed by HWLM). */
    u32 floatingMatcherState;

    /** Packed SOM location slots. */
    u32 somLocation;

    /** Multibit guarding SOM location slots. */
    u32 somValid;

    /** Multibit guarding SOM location slots. */
    u32 somWritable;

    /** Total size of Rose state, in bytes. */
    u32 end;
};

struct RoseBoundaryReports {
    u32 reportEodOffset; /**< 0 if no reports lits, otherwise offset of
                          * MO_INVALID_IDX terminated list to report at EOD */
    u32 reportZeroOffset; /**< 0 if no reports lits, otherwise offset of
                           * MO_INVALID_IDX terminated list to report at offset
                           * 0 */
    u32 reportZeroEodOffset; /**< 0 if no reports lits, otherwise offset of
                              * MO_INVALID_IDX terminated list to report if eod
                              * is at offset 0. Superset of other lists. */
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

// Runtime structure header for Rose.
// In memory, we follow this with:
//   1a. anchored 'literal' matcher table
//   1b. floating literal matcher table
//   1c. sidecar 'literal' matcher table
//   1d. eod-anchored literal matcher table
//   1e. small block table
//   2. array of RoseLiteral (literalCount entries)
//   3. array of RoseRole (roleCount entries)
//   4. array of RosePred (predCount entries)
//   8. array of NFA offsets, one per queue
//   9. array of state offsets, one per queue (+)
//  10. array of role ids for the set of all root roles
//  12. multi-direct report array
/*
 *  (+) stateOffset array note: Offsets in the array are either into the stream
 *  state (normal case) or into the tstate region of scratch (for transient rose
 *  nfas). Rose nfa info table can distinguish the cases.
 */
struct RoseEngine {
    u8  hasFloatingDirectReports; // has at least one floating direct report literal
    u8  noFloatingRoots; /* only need to run the anchored table if something
                          * matched in the anchored table */
    u8  requiresEodCheck; /* stuff happens at eod time */
    u8  requiresEodSideCatchup; /* we need to do a sidecar catchup before eod
                                 * checks */
    u8  hasEodEventLiteral; // fires a ROSE_EVENT literal at eod time.
    u8  hasOutfixesInSmallBlock; /**< has at least one outfix that must run even
                                    in small block scans. */
    u8  runtimeImpl; /**< can we just run the floating table or a single outfix?
                      * or do we need a full rose? */
    u8  mpvTriggeredByLeaf; /**< need to check (suf|out)fixes for mpv trigger */
    u8  canExhaust; /**< every pattern has an exhaustion key */
    u8  hasSom; /**< has at least one pattern which tracks SOM. */
    u8  somHorizon; /**< width in bytes of SOM offset storage (governed by
                        SOM precision) */
    u8  simpleCallback; /**< has only external reports with no bounds checks,
                             plus no exhaustion keys */
    u32 mode; /**< scanning mode, one of HS_MODE_{BLOCK,STREAM,VECTORED} */
    u32 historyRequired; /**< max amount of history required for streaming */
    u32 ekeyCount; /**< number of exhaustion keys */
    u32 dkeyCount; /**< number of dedupe keys */
    u32 invDkeyOffset; /**< offset to table mapping from dkeys to the external
                         *  report ids */
    u32 somLocationCount; /**< number of som locations required */
    u32 rolesWithStateCount; // number of roles with entries in state bitset
    u32 stateSize; /* size of the state bitset
                    * WARNING: not the size of the rose state */
    u32 anchorStateSize; /* size of the state for the anchor dfas */
    u32 nfaStateSize; /* total size of the state for the mask/rose nfas */
    u32 tStateSize; /* total size of the state for transient rose nfas */
    u32 scratchStateSize; /**< uncompressed state req'd for NFAs in scratch;
                           * used for sizing scratch only. */
    u32 smallWriteOffset; /**< offset of small-write matcher */
    u32 amatcherOffset; // offset of the anchored literal matcher (bytes)
    u32 ematcherOffset; // offset of the eod-anchored literal matcher (bytes)
    u32 fmatcherOffset; // offset of the floating literal matcher (bytes)
    u32 smatcherOffset; // offset of the sidecar literal matcher (bytes)
    u32 sbmatcherOffset; // offset of the small-block literal matcher (bytes)
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
    u32 intReportOffset; /**< offset of array of internal_report structures */
    u32 intReportCount; /**< number of internal_report structures */
    u32 literalOffset; // offset of RoseLiteral array (bytes)
    u32 literalCount; // number of RoseLiteral entries [NOT number of literals]
    u32 sideOffset; /**< offset of RoseSide array (bytes), indexed by
                     *sidecar ids */
    u32 sideCount; /**< number of RoseSide entries */
    u32 multidirectOffset; /**< offset of multi-direct report list. */
    u32 activeArrayCount; //number of nfas tracked in the active array
    u32 activeLeftCount; //number of nfas tracked in the active rose array
    u32 queueCount;      /**< number of nfa queues */
    u32 roleOffset; // offset of RoseRole array (bytes)
    u32 roleCount; // number of RoseRole entries
    u32 predOffset; // offset of RosePred array (bytes)
    u32 predCount; // number of RosePred entries
    u32 rootRoleOffset;
    u32 rootRoleCount;

    u32 leftOffset;
    u32 roseCount;
    u32 lookaroundTableOffset; //!< base of lookaround offset list (of s8 values)
    u32 lookaroundReachOffset; /**< base of lookaround reach bitvectors (32
                                * bytes each) */

    u32 eodIterOffset; // or 0 if no eod iterator
    u32 eodIterMapOffset;

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
    u32 maxSafeAnchoredDROffset; /* the maximum offset that we can safely raise
                                  * a direct report from the anchored table
                                  * without delaying it */
    u32 floatingMinLiteralMatchOffset; /* the minimum offset that we can get a
                                        * 'valid' match from the floating
                                        * table */
    u32 nfaInfoOffset; /* offset to the nfa info offset array */
    u32 anchoredReportMapOffset; /* am_log index --> reportid */
    u32 anchoredReportInverseMapOffset; /*  reportid --> am_log index */
    rose_group initialGroups;
    u32 size; // (bytes)
    u32 anchoredMatches; /* number of anchored roles generating matches */
    u32 delay_count; /* number of delayed literal ids. */
    u32 delay_slot_size; /* size of delay slot mmbit. */
    u32 delay_base_id; /* literal id of the first delayed literal.
                        * delayed literal ids are contiguous */
    u32 anchored_count; /* number of anchored literal ids */
    u32 anchored_base_id; /* literal id of the first literal in the A table.
                           * anchored literal ids are contiguous */
    u32 nonbenefits_base_id; /* first literal id without benefit conf.
                              * contiguous, blah, blah */
    u32 maxFloatingDelayedMatch; /* max offset that a delayed literal can
                                  * usefully be reported */
    u32 delayRebuildLength; /* length of the history region which needs to be
                             * rescanned when we are doing a delayed literal
                             * rebuild scan. */
    struct RoseStateOffsets stateOffsets;
    struct RoseBoundaryReports boundary;
    u32 totalNumLiterals; /* total number of literals including dr */
    u32 asize; /* size of the atable */
    u32 initSideEnableOffset; /* sidecar literals enabled initially */
    u32 outfixBeginQueue; /* first outfix queue */
    u32 outfixEndQueue; /* one past the last outfix queue */
    u32 leftfixBeginQueue; /* first prefix/infix queue */
    u32 initMpvNfa; /* (allegedly chained) mpv to force on at init */
    u32 rosePrefixCount; /* number of rose prefixes */
    u32 activeLeftIterOffset; /* mmbit_sparse_iter over non-transient roses */
    u32 ematcherRegionSize; /* max region size to pass to ematcher */
    u32 literalBenefitsOffsets; /* offset to array of benefits indexed by lit
                                   id */
    u32 somRevCount; /**< number of som reverse nfas */
    u32 somRevOffsetOffset; /**< offset to array of offsets to som rev nfas */
    u32 nfaRegionBegin; /* start of the nfa region, debugging only */
    u32 nfaRegionEnd; /* end of the nfa region, debugging only */
    u32 group_weak_end; /* end of weak groups, debugging only */
    u32 floatingStreamState; // size in bytes
    u32 eodLiteralId; // literal ID for eod ROSE_EVENT if used, otherwise 0.

    struct scatter_full_plan state_init;
};

struct lit_benefits {
    union {
        u64a a64[MAX_MASK2_WIDTH/sizeof(u64a)];
        u8 a8[MAX_MASK2_WIDTH];
    } and_mask;
    union {
        u64a e64[MAX_MASK2_WIDTH/sizeof(u64a)];
        u8 e8[MAX_MASK2_WIDTH];
    } expected;
};

#if defined(_WIN32)
#pragma pack(push, 1)
#endif
// Rose runtime state
struct RoseRuntimeState {
    u8 stored_depth; /* depth at stream boundary */
    u8 flags; /* high bit true if delay rebuild needed */
    u8 broken; /* user has requested that we stop matching */
#if defined(_WIN32)
};
#pragma pack(pop)
#else
} __attribute__((packed));
#endif

struct ALIGN_CL_DIRECTIVE anchored_matcher_info {
    u32 next_offset; /* relative to this, 0 for end */
    u32 state_offset; /* relative to anchorState */
    u32 anchoredMinDistance; /* start of region to run anchored table over */
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
const void *getSLiteralMatcher(const struct RoseEngine *t) {
    if (!t->smatcherOffset) {
        return NULL;
    }

    const char *st = (const char *)t + t->smatcherOffset;
    assert(ISALIGNED_N(st, 8));
    return st;
}

static really_inline
const void *getELiteralMatcher(const struct RoseEngine *t) {
    if (!t->ematcherOffset) {
        return NULL;
    }

    const char *et = (const char *)t + t->ematcherOffset;
    assert(ISALIGNED_N(et, 8));
    return et;
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
const struct RoseLiteral *getLiteralTable(const struct RoseEngine *t) {
    const struct RoseLiteral *tl
        = (const struct RoseLiteral *)((const char *)t + t->literalOffset);
    assert(ISALIGNED_N(tl, 4));
    return tl;
}

static really_inline
const struct RoseSide *getSideEntryTable(const struct RoseEngine *t) {
    const struct RoseSide *rs
        = (const struct RoseSide *)((const char *)t + t->sideOffset);
    assert(ISALIGNED(rs));
    return rs;
}

static really_inline
const struct RoseRole *getRoleTable(const struct RoseEngine *t) {
    const struct RoseRole *r
        = (const struct RoseRole *)((const char *)t + t->roleOffset);
    assert(ISALIGNED_N(r, 4));
    return r;
}

static really_inline
const struct RosePred *getPredTable(const struct RoseEngine *t) {
    const struct RosePred *p
        = (const struct RosePred *)((const char *)t + t->predOffset);
    assert(ISALIGNED_N(p, 4));
    return p;
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
const u32 *getRootRoleTable(const struct RoseEngine *t) {
    const u32 *r = (const u32 *)((const char *)t + t->rootRoleOffset);
    assert(ISALIGNED_N(r, 4));
    return r;
}

static really_inline
const struct lit_benefits *getLiteralBenefitsTable(
                                              const struct RoseEngine *t) {
    return (const struct lit_benefits *)
        ((const char *)t + t->literalBenefitsOffsets);
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
