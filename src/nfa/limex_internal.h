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

/** \file
    This file provides the internal structures and definitions required for the
    real NFAs (aka limex NFAs );

    Limex NFAs now have variable length in memory. They look like this:

        LimExNFA structure
            Fixed length, e.g. LimExNFA256.
        Reachability table
            Variable length array of state bitvectors, mapped into by
            NFACommonXXX.reachMap.
        Tops
            Variable length array of state bitvectors, used for TOP_N events.
        Acceleration structures
            Variable length array of AccelAux structs.
        Accepts
            Variable length array of NFAAccept structs.
        EOD Accepts
            Variable length array of NFAAccept structs.
        Exceptions
            Variable length array of NFAExceptionXXX structs.
        Repeat Structure Offsets
            Array of u32 offsets that point at each "Repeat Structure" (below)
        Repeat Structures
            Variable length repeat structures, addressed via
            NFAException32::repeatOffset etc.

    The state associated with the NFA is split into:

    -# The "traditional" NFA state as a bitvector. This is stored in the
       first N bytes of the state space (length given in
       NFACommonXXX.stateSize), and may be stored shrunk to CEIL(stateSize/8)
       or compressed. If it is stored compressed, than the
       LIMEX_FLAG_COMPRESS_STATE flag is set in NFACommonXXX.flags.
    -# Extended NFA state, only used in some LimEx NFAs. This consists of a
       variable length array of LimExNFAExtendedState structures, each with
       pointers to a packed list of mmbit structures that follows them. Only
       present when used.

    The value of NFA.stateSize gives the total state size in bytes (the sum of
    all the above).

    Number of shifts should be always greater or equal to 1
    Number of shifts 0 means that no appropriate NFA engine was found.

*/

#ifndef LIMEX_INTERNAL_H
#define LIMEX_INTERNAL_H

#include "nfa_internal.h"
#include "repeat_internal.h"

// Constants
#define MAX_SHIFT_COUNT 8   /**< largest number of shifts used by a LimEx NFA */
#define MAX_SHIFT_AMOUNT 16 /**< largest shift amount used by a LimEx NFA */

#define LIMEX_FLAG_COMPRESS_STATE  1 /**< pack state into stream state */
#define LIMEX_FLAG_COMPRESS_MASKED 2 /**< use reach mask-based compression */
#define LIMEX_FLAG_CANNOT_DIE      4 /**< limex cannot have no states on */
#define LIMEX_FLAG_EXTRACT_EXP     8 /**< use limex exception bit extraction */

enum LimExTrigger {
    LIMEX_TRIGGER_NONE = 0,
    LIMEX_TRIGGER_POS = 1,
    LIMEX_TRIGGER_TUG = 2
};

enum LimExSquash {
    LIMEX_SQUASH_NONE = 0,   //!< no squash for you!
    LIMEX_SQUASH_CYCLIC = 1, //!< squash due to cyclic state
    LIMEX_SQUASH_TUG = 2,    //!< squash due to tug trigger with stale estate
    LIMEX_SQUASH_REPORT = 3  //!< squash when report is raised
};

/* uniform looking types for the macros */
typedef u8   u_8;
typedef u16  u_16;
typedef u32  u_32;
typedef u64a u_64;
typedef m128 u_128;
typedef m256 u_256;
typedef m384 u_384;
typedef m512 u_512;

#define CREATE_NFA_LIMEX(size)                                              \
struct NFAException##size {                                                 \
    u_##size squash; /**< mask of states to leave on */                     \
    u_##size successors; /**< mask of states to switch on */                \
    u32 reports; /**< offset to start of reports list, or MO_INVALID_IDX */ \
    u32 repeatOffset; /**< offset to NFARepeatInfo, or MO_INVALID_IDX */    \
    u8 hasSquash; /**< from enum LimExSquash */                             \
    u8 trigger; /**< from enum LimExTrigger */                              \
};                                                                          \
                                                                            \
struct LimExNFA##size {                                                     \
    u8 reachMap[N_CHARS]; /**< map of char -> entry in reach[] */           \
    u32 reachSize; /**< number of reach masks */                            \
    u32 accelCount; /**< number of entries in accel table */                \
    u32 accelTableOffset; /* rel. to start of LimExNFA */                   \
    u32 accelAuxCount; /**< number of entries in aux table */               \
    u32 accelAuxOffset; /* rel. to start of LimExNFA */                     \
    u32 acceptCount;                                                        \
    u32 acceptOffset; /* rel. to start of LimExNFA */                       \
    u32 acceptEodCount;                                                     \
    u32 acceptEodOffset; /* rel. to start of LimExNFA */                    \
    u32 exceptionCount;                                                     \
    u32 exceptionOffset; /* rel. to start of LimExNFA */                    \
    u32 repeatCount;                                                        \
    u32 repeatOffset;                                                       \
    u32 squashOffset; /* rel. to start of LimExNFA; for accept squashing */ \
    u32 squashCount;                                                        \
    u32 topCount;                                                           \
    u32 topOffset; /* rel. to start of LimExNFA */                          \
    u32 stateSize; /**< not including extended history */                   \
    u32 flags;                                                              \
    u_##size init;                                                          \
    u_##size initDS;                                                        \
    u_##size accept; /**< mask of accept states */                          \
    u_##size acceptAtEOD; /**< mask of states that accept at EOD */         \
    u_##size accel; /**< mask of accelerable states */                      \
    u_##size accelPermute; /**< pshufb permute mask (not GPR) */            \
    u_##size accelCompare; /**< pshufb compare mask (not GPR) */            \
    u_##size accel_and_friends; /**< mask of accelerable states + likely
                                    *  followers */                         \
    u_##size compressMask; /**< switch off before compress */               \
    u_##size exceptionMask;                                                 \
    u_##size repeatCyclicMask; /**< also includes tug states */             \
    u_##size zombieMask; /**< zombie if in any of the set states */         \
    u_##size shift[MAX_SHIFT_COUNT];                                        \
    u32 shiftCount; /**< number of shift masks used */                      \
    u8 shiftAmount[MAX_SHIFT_COUNT]; /**< shift amount for each mask */     \
    m512 exceptionShufMask; /**< exception byte shuffle mask  */            \
    m512 exceptionBitMask; /**< exception bit mask */                       \
    m512 exceptionAndMask; /**< exception and mask */                       \
};

CREATE_NFA_LIMEX(32)
CREATE_NFA_LIMEX(64)
CREATE_NFA_LIMEX(128)
CREATE_NFA_LIMEX(256)
CREATE_NFA_LIMEX(384)
CREATE_NFA_LIMEX(512)

/** \brief Structure describing a bounded repeat within the LimEx NFA.
 *
 * This struct is followed in memory by:
 *
 * -# a RepeatInfo structure
 * -# a variable-sized lookup table for REPEAT_SPARSE_OPTIMAL_P repeats
 * -# a TUG mask
 */
struct NFARepeatInfo {
    u32 cyclicState;      //!< index of this repeat's cyclic state
    u32 ctrlIndex;        //!< index of this repeat's control block
    u32 packedCtrlOffset; //!< offset to packed control block in stream state
    u32 stateOffset;      //!< offset to repeat state in stream state
    u32 stateSize;        //!< total size of packed stream state for this repeat
    u32 tugMaskOffset;    //!< offset to tug mask (rel. to NFARepeatInfo)
};

struct NFAAccept {
    u8 single_report; //!< If true, 'reports' is report id.

    /**
     * \brief If single report is true, this is the report id to fire.
     * Otherwise, it is the offset (relative to the start of the LimExNFA
     * structure) of a list of reports, terminated with MO_INVALID_IDX.
     */
    u32 reports;

    u32 squash;  //!< Offset (from LimEx) into squash masks, or MO_INVALID_IDX.
};

#endif
