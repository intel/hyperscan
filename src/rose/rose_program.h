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

/** \file
 * \brief Rose data structures to do with role programs.
 */

#ifndef ROSE_ROSE_PROGRAM_H
#define ROSE_ROSE_PROGRAM_H

#include "som/som_operation.h"
#include "rose_internal.h"
#include "ue2common.h"

/** \brief Minimum alignment for each instruction in memory. */
#define ROSE_INSTR_MIN_ALIGN 8U

/** \brief Role program instruction opcodes. */
enum RoseInstructionCode {
    ROSE_INSTR_ANCHORED_DELAY,    //!< Delay until after anchored matcher.
    ROSE_INSTR_CHECK_LIT_MASK,    //!< Check and/cmp mask.
    ROSE_INSTR_CHECK_LIT_EARLY,   //!< Skip matches before floating min offset.
    ROSE_INSTR_CHECK_GROUPS,      //!< Check that literal groups are on.
    ROSE_INSTR_CHECK_ONLY_EOD,    //!< Role matches only at EOD.
    ROSE_INSTR_CHECK_BOUNDS,      //!< Bounds on distance from offset 0.
    ROSE_INSTR_CHECK_NOT_HANDLED, //!< Test & set role in "handled".
    ROSE_INSTR_CHECK_LOOKAROUND,  //!< Lookaround check.
    ROSE_INSTR_CHECK_INFIX,       //!< Infix engine must be in accept state.
    ROSE_INSTR_CHECK_PREFIX,      //!< Prefix engine must be in accept state.
    ROSE_INSTR_PUSH_DELAYED,      //!< Push delayed literal matches.
    ROSE_INSTR_CATCH_UP,          //!< Catch up engines, anchored matches.
    ROSE_INSTR_CATCH_UP_MPV,      //!< Catch up the MPV.
    ROSE_INSTR_SOM_ADJUST,        //!< Set SOM from a distance to EOM.
    ROSE_INSTR_SOM_LEFTFIX,       //!< Acquire SOM from a leftfix engine.
    ROSE_INSTR_SOM_FROM_REPORT,   //!< Acquire SOM from a som_operation.
    ROSE_INSTR_SOM_ZERO,          //!< Set SOM to zero.
    ROSE_INSTR_TRIGGER_INFIX,     //!< Trigger an infix engine.
    ROSE_INSTR_TRIGGER_SUFFIX,    //!< Trigger a suffix engine.
    ROSE_INSTR_DEDUPE,            //!< Run deduplication for report.
    ROSE_INSTR_DEDUPE_SOM,        //!< Run deduplication for SOM report.
    ROSE_INSTR_REPORT_CHAIN,      //!< Fire a chained report (MPV).
    ROSE_INSTR_REPORT_SOM_INT,    //!< Manipulate SOM only.
    ROSE_INSTR_REPORT_SOM_AWARE,  //!< Manipulate SOM from SOM-aware source.

    /** \brief Fire a report. */
    ROSE_INSTR_REPORT,

    /** \brief Fire an exhaustible report. */
    ROSE_INSTR_REPORT_EXHAUST,

    /** \brief Fire a SOM report. */
    ROSE_INSTR_REPORT_SOM,

    /** \brief Fire an exhaustible SOM report. */
    ROSE_INSTR_REPORT_SOM_EXHAUST,

    /** \brief Super-instruction combining DEDUPE and REPORT. */
    ROSE_INSTR_DEDUPE_AND_REPORT,

    /**
     * \brief Fire a report and stop program execution. This is a
     * specialisation intended for short, frequently-executed programs.
     */
    ROSE_INSTR_FINAL_REPORT,

    ROSE_INSTR_CHECK_EXHAUSTED,   //!< Check if an ekey has already been set.
    ROSE_INSTR_CHECK_MIN_LENGTH,  //!< Check (EOM - SOM) against min length.
    ROSE_INSTR_SET_STATE,         //!< Switch a state index on.
    ROSE_INSTR_SET_GROUPS,        //!< Set some literal group bits.
    ROSE_INSTR_SQUASH_GROUPS,     //!< Conditionally turn off some groups.
    ROSE_INSTR_CHECK_STATE,       //!< Test a single bit in the state multibit.
    ROSE_INSTR_SPARSE_ITER_BEGIN, //!< Begin running a sparse iter over states.
    ROSE_INSTR_SPARSE_ITER_NEXT,  //!< Continue running sparse iter over states.
    ROSE_INSTR_END                //!< End of program.
};

struct ROSE_STRUCT_ANCHORED_DELAY {
    u8 code; //!< From enum RoseInstructionCode.
    rose_group groups; //!< Bitmask.
    u32 done_jump; //!< Jump forward this many bytes if successful.
};

union RoseLiteralMask {
    u64a a64[MAX_MASK2_WIDTH / sizeof(u64a)];
    u8 a8[MAX_MASK2_WIDTH];
};

/** Note: check failure will halt program. */
struct ROSE_STRUCT_CHECK_LIT_MASK {
    u8 code; //!< From enum RoseInstructionCode.
    union RoseLiteralMask and_mask;
    union RoseLiteralMask cmp_mask;
};

/** Note: check failure will halt program. */
struct ROSE_STRUCT_CHECK_LIT_EARLY {
    u8 code; //!< From enum RoseInstructionCode.
};

/** Note: check failure will halt program. */
struct ROSE_STRUCT_CHECK_GROUPS {
    u8 code; //!< From enum RoseInstructionCode.
    rose_group groups; //!< Bitmask.
};

struct ROSE_STRUCT_CHECK_ONLY_EOD {
    u8 code; //!< From enum RoseInstructionCode.
    u32 fail_jump; //!< Jump forward this many bytes on failure.
};

struct ROSE_STRUCT_CHECK_BOUNDS {
    u8 code; //!< From enum RoseInstructionCode.
    u64a min_bound; //!< Min distance from zero.
    u64a max_bound; //!< Max distance from zero.
    u32 fail_jump; //!< Jump forward this many bytes on failure.
};

struct ROSE_STRUCT_CHECK_NOT_HANDLED {
    u8 code; //!< From enum RoseInstructionCode.
    u32 key; //!< Key in the "handled_roles" fatbit in scratch.
    u32 fail_jump; //!< Jump forward this many bytes if we have seen key before.
};

struct ROSE_STRUCT_CHECK_LOOKAROUND {
    u8 code; //!< From enum RoseInstructionCode.
    u32 index;
    u32 count;
    u32 fail_jump; //!< Jump forward this many bytes on failure.
};

struct ROSE_STRUCT_CHECK_INFIX {
    u8 code; //!< From enum RoseInstructionCode.
    u32 queue; //!< Queue of leftfix to check.
    u32 lag; //!< Lag of leftfix for this case.
    ReportID report; //!< ReportID of leftfix to check.
    u32 fail_jump; //!< Jump forward this many bytes on failure.
};

struct ROSE_STRUCT_CHECK_PREFIX {
    u8 code; //!< From enum RoseInstructionCode.
    u32 queue; //!< Queue of leftfix to check.
    u32 lag; //!< Lag of leftfix for this case.
    ReportID report; //!< ReportID of leftfix to check.
    u32 fail_jump; //!< Jump forward this many bytes on failure.
};

struct ROSE_STRUCT_PUSH_DELAYED {
    u8 code; //!< From enum RoseInstructionCode.
    u8 delay; // Number of bytes to delay.
    u32 index; // Delay literal index (relative to first delay lit).
};

struct ROSE_STRUCT_CATCH_UP {
    u8 code; //!< From enum RoseInstructionCode.
};

struct ROSE_STRUCT_CATCH_UP_MPV {
    u8 code; //!< From enum RoseInstructionCode.
};

struct ROSE_STRUCT_SOM_ADJUST {
    u8 code; //!< From enum RoseInstructionCode.
    u32 distance; //!< Distance to EOM.
};

struct ROSE_STRUCT_SOM_LEFTFIX {
    u8 code; //!< From enum RoseInstructionCode.
    u32 queue; //!< Queue index of leftfix providing SOM.
    u32 lag; //!< Lag of leftfix for this case.
};

struct ROSE_STRUCT_SOM_FROM_REPORT {
    u8 code; //!< From enum RoseInstructionCode.
    struct som_operation som;
};

struct ROSE_STRUCT_SOM_ZERO {
    u8 code; //!< From enum RoseInstructionCode.
};

struct ROSE_STRUCT_TRIGGER_INFIX {
    u8 code; //!< From enum RoseInstructionCode.
    u8 cancel; //!< Cancels previous top event.
    u32 queue; //!< Queue index of infix.
    u32 event; //!< Queue event, from MQE_*.
};

struct ROSE_STRUCT_TRIGGER_SUFFIX {
    u8 code; //!< From enum RoseInstructionCode.
    u32 queue; //!< Queue index of suffix.
    u32 event; //!< Queue event, from MQE_*.
};

struct ROSE_STRUCT_DEDUPE {
    u8 code; //!< From enum RoseInstructionCode.
    u8 quash_som; //!< Force SOM to zero for this report.
    u32 dkey; //!< Dedupe key.
    s32 offset_adjust; //!< Offset adjustment to apply to end offset.
    u32 fail_jump; //!< Jump forward this many bytes on failure.
};

struct ROSE_STRUCT_DEDUPE_SOM {
    u8 code; //!< From enum RoseInstructionCode.
    u8 quash_som; //!< Force SOM to zero for this report.
    u32 dkey; //!< Dedupe key.
    s32 offset_adjust; //!< Offset adjustment to apply to end offset.
    u32 fail_jump; //!< Jump forward this many bytes on failure.
};

struct ROSE_STRUCT_REPORT_CHAIN {
    u8 code; //!< From enum RoseInstructionCode.
    u32 event; //!< Queue event, from MQE_*. Must be a top.

    /**
     * \brief Number of bytes behind us that we are allowed to squash
     * identical top events on the queue.
     */
    u64a top_squash_distance;
};

struct ROSE_STRUCT_REPORT_SOM_INT {
    u8 code; //!< From enum RoseInstructionCode.
    struct som_operation som;
};

struct ROSE_STRUCT_REPORT_SOM_AWARE {
    u8 code; //!< From enum RoseInstructionCode.
    struct som_operation som;
};

struct ROSE_STRUCT_REPORT {
    u8 code; //!< From enum RoseInstructionCode.
    ReportID onmatch; //!< Report ID to deliver to user.
    s32 offset_adjust; //!< Offset adjustment to apply to end offset.
};

struct ROSE_STRUCT_REPORT_EXHAUST {
    u8 code; //!< From enum RoseInstructionCode.
    ReportID onmatch; //!< Report ID to deliver to user.
    s32 offset_adjust; //!< Offset adjustment to apply to end offset.
    u32 ekey; //!< Exhaustion key.
};

struct ROSE_STRUCT_REPORT_SOM {
    u8 code; //!< From enum RoseInstructionCode.
    ReportID onmatch; //!< Report ID to deliver to user.
    s32 offset_adjust; //!< Offset adjustment to apply to end offset.
};

struct ROSE_STRUCT_REPORT_SOM_EXHAUST {
    u8 code; //!< From enum RoseInstructionCode.
    ReportID onmatch; //!< Report ID to deliver to user.
    s32 offset_adjust; //!< Offset adjustment to apply to end offset.
    u32 ekey; //!< Exhaustion key.
};

struct ROSE_STRUCT_DEDUPE_AND_REPORT {
    u8 code; //!< From enum RoseInstructionCode.
    u8 quash_som; //!< Force SOM to zero for this report.
    u32 dkey; //!< Dedupe key.
    ReportID onmatch; //!< Report ID to deliver to user.
    s32 offset_adjust; //!< Offset adjustment to apply to end offset.
    u32 fail_jump; //!< Jump forward this many bytes on failure.
};

struct ROSE_STRUCT_FINAL_REPORT {
    u8 code; //!< From enum RoseInstructionCode.
    ReportID onmatch; //!< Report ID to deliver to user.
    s32 offset_adjust; //!< Offset adjustment to apply to end offset.
};

struct ROSE_STRUCT_CHECK_EXHAUSTED {
    u8 code; //!< From enum RoseInstructionCode.
    u32 ekey; //!< Exhaustion key to check.
    u32 fail_jump; //!< Jump forward this many bytes on failure.
};

struct ROSE_STRUCT_CHECK_MIN_LENGTH {
    u8 code; //!< From enum RoseInstructionCode.
    s32 end_adj; //!< Offset adjustment to add to EOM first.
    u64a min_length; //!< Minimum distance from SOM to EOM.
    u32 fail_jump; //!< Jump forward this many bytes on failure.
};

struct ROSE_STRUCT_SET_STATE {
    u8 code; //!< From enum RoseInstructionCode.
    u32 index; //!< State index in multibit.
};

struct ROSE_STRUCT_SET_GROUPS {
    u8 code; //!< From enum RoseInstructionCode.
    rose_group groups; //!< Bitmask to OR into groups.
};

struct ROSE_STRUCT_SQUASH_GROUPS {
    u8 code; //!< From enum RoseInstructionCode.
    rose_group groups; //!< Bitmask to AND into groups.
};

struct ROSE_STRUCT_CHECK_STATE {
    u8 code; //!< From enum RoseInstructionCode.
    u32 index; //!< State index in the role multibit.
    u32 fail_jump; //!< Jump forward this many bytes on failure.
};

/**
 * Note that the offsets in the jump table are always relative to the start of
 * the program, not the current instruction.
 */
struct ROSE_STRUCT_SPARSE_ITER_BEGIN {
    u8 code; //!< From enum RoseInstructionCode.
    u32 iter_offset; //!< Offset of mmbit_sparse_iter structure.
    u32 jump_table; //!< Offset of jump table indexed by sparse iterator.
    u32 fail_jump; //!< Jump forward this many bytes on failure.
};

/**
 * Note that the offsets in the jump table are always relative to the start of
 * the program, not the current instruction.
 */
struct ROSE_STRUCT_SPARSE_ITER_NEXT {
    u8 code; //!< From enum RoseInstructionCode.
    u32 iter_offset; //!< Offset of mmbit_sparse_iter structure.
    u32 jump_table; //!< Offset of jump table indexed by sparse iterator.
    u32 state; // Current state index.
    u32 fail_jump; //!< Jump forward this many bytes on failure.
};

struct ROSE_STRUCT_END {
    u8 code; //!< From enum RoseInstructionCode.
};

#endif // ROSE_ROSE_PROGRAM_H
