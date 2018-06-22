/*
 * Copyright (c) 2015-2018, Intel Corporation
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
#include "util/simd_types.h"

/** \brief Minimum alignment for each instruction in memory. */
#define ROSE_INSTR_MIN_ALIGN 8U

/** \brief Role program instruction opcodes. */
enum RoseInstructionCode {
    ROSE_INSTR_END,               //!< End of program.
    ROSE_INSTR_ANCHORED_DELAY,    //!< Delay until after anchored matcher.
    ROSE_INSTR_CHECK_LIT_EARLY,   //!< Skip matches before floating min offset.
    ROSE_INSTR_CHECK_GROUPS,      //!< Check that literal groups are on.
    ROSE_INSTR_CHECK_ONLY_EOD,    //!< Role matches only at EOD.
    ROSE_INSTR_CHECK_BOUNDS,      //!< Bounds on distance from offset 0.
    ROSE_INSTR_CHECK_NOT_HANDLED, //!< Test & set role in "handled".
    ROSE_INSTR_CHECK_SINGLE_LOOKAROUND, //!< Single lookaround check.
    ROSE_INSTR_CHECK_LOOKAROUND,  //!< Lookaround check.
    ROSE_INSTR_CHECK_MASK,        //!< 8-bytes mask check.
    ROSE_INSTR_CHECK_MASK_32,     //!< 32-bytes and/cmp/neg mask check.
    ROSE_INSTR_CHECK_BYTE,        //!< Single Byte check.
    ROSE_INSTR_CHECK_SHUFTI_16x8, //!< Check 16-byte data by 8-bucket shufti.
    ROSE_INSTR_CHECK_SHUFTI_32x8, //!< Check 32-byte data by 8-bucket shufti.
    ROSE_INSTR_CHECK_SHUFTI_16x16, //!< Check 16-byte data by 16-bucket shufti.
    ROSE_INSTR_CHECK_SHUFTI_32x16, //!< Check 32-byte data by 16-bucket shufti.
    ROSE_INSTR_CHECK_INFIX,       //!< Infix engine must be in accept state.
    ROSE_INSTR_CHECK_PREFIX,      //!< Prefix engine must be in accept state.
    ROSE_INSTR_PUSH_DELAYED,      //!< Push delayed literal matches.
    ROSE_INSTR_DUMMY_NOP,         //!< NOP. Should not exist in build programs.
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
    ROSE_INSTR_SPARSE_ITER_ANY,   //!< Test for any bit in the sparse iterator.

    /** \brief Check outfixes and suffixes for EOD and fire reports if so. */
    ROSE_INSTR_ENGINES_EOD,

    /** \brief Catch up and check active suffixes for EOD and fire reports if
     * so. */
    ROSE_INSTR_SUFFIXES_EOD,

    /** \brief Run the EOD-anchored HWLM literal matcher. */
    ROSE_INSTR_MATCHER_EOD,

    /**
     * \brief Confirm a case-sensitive literal at the current offset. In
     * streaming mode, this makes use of the long literal table.
     */
    ROSE_INSTR_CHECK_LONG_LIT,

    /**
     * \brief Confirm a case-insensitive literal at the current offset. In
     * streaming mode, this makes use of the long literal table.
     */
    ROSE_INSTR_CHECK_LONG_LIT_NOCASE,

    /**
     * \brief Confirm a case-sensitive "medium length" literal at the current
     * offset. In streaming mode, this will check history if needed.
     */
    ROSE_INSTR_CHECK_MED_LIT,

    /**
     * \brief Confirm a case-insensitive "medium length" literal at the current
     * offset. In streaming mode, this will check history if needed.
     */
    ROSE_INSTR_CHECK_MED_LIT_NOCASE,

    /**
     * \brief Clear the "work done" flag used by the SQUASH_GROUPS instruction.
     */
    ROSE_INSTR_CLEAR_WORK_DONE,

    /** \brief Check lookaround if it has multiple paths. */
    ROSE_INSTR_MULTIPATH_LOOKAROUND,

    /**
     * \brief Use shufti to check lookaround with multiple paths. The total
     * length of the paths is 16 bytes at most and shufti has 8 buckets.
     * All paths can be at most 16 bytes long.
     */
    ROSE_INSTR_CHECK_MULTIPATH_SHUFTI_16x8,

    /**
     * \brief Use shufti to check lookaround with multiple paths. The total
     * length of the paths is 32 bytes at most and shufti has 8 buckets.
     * All paths can be at most 16 bytes long.
     */
    ROSE_INSTR_CHECK_MULTIPATH_SHUFTI_32x8,

    /**
     * \brief Use shufti to check lookaround with multiple paths. The total
     * length of the paths is 32 bytes at most and shufti has 16 buckets.
     * All paths can be at most 16 bytes long.
     */
    ROSE_INSTR_CHECK_MULTIPATH_SHUFTI_32x16,

    /**
     * \brief Use shufti to check multiple paths lookaround. The total
     * length of the paths is 64 bytes at most and shufti has 8 buckets.
     * All paths can be at most 16 bytes long.
     */
    ROSE_INSTR_CHECK_MULTIPATH_SHUFTI_64,

    /**
     * \brief Jump to the program of included literal.
     */
    ROSE_INSTR_INCLUDED_JUMP,

    /**
     * \brief Set matching status of a sub-expression.
     */
    ROSE_INSTR_SET_LOGICAL,

    /**
     * \brief Set combination status pending checking.
     */
    ROSE_INSTR_SET_COMBINATION,

    /**
     * \brief Check if compliant with any logical constraints.
     */
    ROSE_INSTR_FLUSH_COMBINATION,

    /** \brief Mark as exhausted instead of report while quiet. */
    ROSE_INSTR_SET_EXHAUST,

    LAST_ROSE_INSTRUCTION = ROSE_INSTR_SET_EXHAUST //!< Sentinel.
};

struct ROSE_STRUCT_END {
    u8 code; //!< From enum RoseInstructionCode.
};

struct ROSE_STRUCT_ANCHORED_DELAY {
    u8 code; //!< From enum RoseInstructionCode.
    rose_group groups; //!< Bitmask.
    u32 anch_id; //!< Program to restart after the delay.
    u32 done_jump; //!< Jump forward this many bytes if we have to delay.
};

struct ROSE_STRUCT_CHECK_LIT_EARLY {
    u8 code; //!< From enum RoseInstructionCode.
    u32 min_offset; //!< Minimum offset for this literal.
    u32 fail_jump; //!< Jump forward this many bytes on failure.
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

struct ROSE_STRUCT_CHECK_SINGLE_LOOKAROUND {
    u8 code; //!< From enum RoseInstructionCode.
    s8 offset; //!< The offset of the byte to examine.
    u32 reach_index; //!< Index for lookaround reach bitvectors.
    u32 fail_jump; //!< Jump forward this many bytes on failure.
};

struct ROSE_STRUCT_CHECK_LOOKAROUND {
    u8 code; //!< From enum RoseInstructionCode.
    u32 look_index; //!< Offset in bytecode of lookaround offset list.
    u32 reach_index; //!< Offset in bytecode of lookaround reach bitvectors.
    u32 count; //!< The count of lookaround entries in one instruction.
    u32 fail_jump; //!< Jump forward this many bytes on failure.
};

struct ROSE_STRUCT_CHECK_MASK {
    u8 code; //!< From enum roseInstructionCode.
    u64a and_mask; //!< 8-byte and mask.
    u64a cmp_mask; //!< 8-byte cmp mask.
    u64a neg_mask; //!< 8-byte negation mask.
    s32 offset; //!< Relative offset of the first byte.
    u32 fail_jump; //!< Jump forward this many bytes on failure.
};

struct ROSE_STRUCT_CHECK_MASK_32 {
    u8 code; //!< From enum RoseInstructionCode.
    u8 and_mask[32]; //!< 32-byte and mask.
    u8 cmp_mask[32]; //!< 32-byte cmp mask.
    u32 neg_mask; //!< negation mask with 32 bits.
    s32 offset; //!< Relative offset of the first byte.
    u32 fail_jump; //!< Jump forward this many bytes on failure.
};

struct ROSE_STRUCT_CHECK_BYTE {
    u8 code; //!< From enum RoseInstructionCode.
    u8 and_mask; //!< 8-bits and mask.
    u8 cmp_mask; //!< 8-bits cmp mask.
    u8 negation; //!< Flag about negation.
    s32 offset; //!< The relative offset.
    u32 fail_jump; //!< Jump forward this many bytes on failure.
};

// Since m128 and m256 could be missaligned in the bytecode,
// we'll use u8[16] and u8[32] instead in all rose_check_shufti structures.
struct ROSE_STRUCT_CHECK_SHUFTI_16x8 {
    u8 code; //!< From enum RoseInstructionCode.
    u8 nib_mask[32]; //!< High 16 and low 16 bits nibble mask in shufti.
    u8 bucket_select_mask[16]; //!< Mask for bucket assigning.
    u32 neg_mask; //!< Negation mask in low 16 bits.
    s32 offset; //!< Relative offset of the first byte.
    u32 fail_jump; //!< Jump forward this many bytes on failure.
};

struct ROSE_STRUCT_CHECK_SHUFTI_32x8 {
    u8 code; //!< From enum RoseInstructionCode.
    u8 hi_mask[16]; //!< High nibble mask in shufti.
    u8 lo_mask[16]; //!< Low nibble mask in shufti.
    u8 bucket_select_mask[32]; //!< Mask for bucket assigning.
    u32 neg_mask; //!< 32 bits negation mask.
    s32 offset; //!< Relative offset of the first byte.
    u32 fail_jump; //!< Jump forward this many bytes on failure.
};

struct ROSE_STRUCT_CHECK_SHUFTI_16x16 {
    u8 code; //!< From enum RoseInstructionCode.
    u8 hi_mask[32]; //!< High nibble mask in shufti.
    u8 lo_mask[32]; //!< Low nibble mask in shufti.
    u8 bucket_select_mask[32]; //!< Mask for bucket assigning.
    u32 neg_mask; //!< Negation mask in low 16 bits.
    s32 offset; //!< Relative offset of the first byte.
    u32 fail_jump; //!< Jump forward this many bytes on failure.
};

struct ROSE_STRUCT_CHECK_SHUFTI_32x16 {
    u8 code; //!< From enum RoseInstructionCode.
    u8 hi_mask[32]; //!< High nibble mask in shufti.
    u8 lo_mask[32]; //!< Low nibble mask in shufti.
    u8 bucket_select_mask_hi[32]; //!< Bucket mask for high 8 buckets.
    u8 bucket_select_mask_lo[32]; //!< Bucket mask for low 8 buckets.
    u32 neg_mask; //!< 32 bits negation mask.
    s32 offset; //!< Relative offset of the first byte.
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

struct ROSE_STRUCT_DUMMY_NOP {
    u8 code; //!< From enum RoseInstructionCode.
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

struct ROSE_STRUCT_SPARSE_ITER_ANY {
    u8 code; //!< From enum RoseInstructionCode.
    u32 iter_offset; //!< Offset of mmbit_sparse_iter structure.
    u32 fail_jump; //!< Jump forward this many bytes on failure.
};

struct ROSE_STRUCT_ENGINES_EOD {
    u8 code; //!< From enum RoseInstructionCode.
    u32 iter_offset; //!< Offset of mmbit_sparse_iter structure.
};

struct ROSE_STRUCT_SUFFIXES_EOD {
    u8 code; //!< From enum RoseInstructionCode.
};

struct ROSE_STRUCT_MATCHER_EOD {
    u8 code; //!< From enum RoseInstructionCode.
};

struct ROSE_STRUCT_CHECK_LONG_LIT {
    u8 code; //!< From enum RoseInstructionCode.
    u32 lit_offset; //!< Offset of literal string.
    u32 lit_length; //!< Length of literal string.
    u32 fail_jump; //!< Jump forward this many bytes on failure.
};

struct ROSE_STRUCT_CHECK_LONG_LIT_NOCASE {
    u8 code; //!< From enum RoseInstructionCode.
    u32 lit_offset; //!< Offset of literal string.
    u32 lit_length; //!< Length of literal string.
    u32 fail_jump; //!< Jump forward this many bytes on failure.
};

struct ROSE_STRUCT_CHECK_MED_LIT {
    u8 code; //!< From enum RoseInstructionCode.
    u32 lit_offset; //!< Offset of literal string.
    u32 lit_length; //!< Length of literal string.
    u32 fail_jump; //!< Jump forward this many bytes on failure.
};

struct ROSE_STRUCT_CHECK_MED_LIT_NOCASE {
    u8 code; //!< From enum RoseInstructionCode.
    u32 lit_offset; //!< Offset of literal string.
    u32 lit_length; //!< Length of literal string.
    u32 fail_jump; //!< Jump forward this many bytes on failure.
};

struct ROSE_STRUCT_CLEAR_WORK_DONE {
    u8 code; //!< From enum RoseInstructionCode.
};

struct ROSE_STRUCT_MULTIPATH_LOOKAROUND {
    u8 code; //!< From enum RoseInstructionCode.
    u32 look_index; //!< Offset in bytecode of lookaround offset list.
    u32 reach_index; //!< Offset in bytecode of lookaround reach bitvectors.
    u32 count; //!< The lookaround byte numbers for each path.
    s32 last_start; //!< The latest start offset among 8 paths.
    u8 start_mask[MULTIPATH_MAX_LEN]; /*!< Used to initialize path if left-most
                                       * data is missed. */
    u32 fail_jump; //!< Jump forward this many bytes on failure.
};

struct ROSE_STRUCT_CHECK_MULTIPATH_SHUFTI_16x8 {
    u8 code; //!< From enum RoseInstructionCode.
    u8 nib_mask[2 * sizeof(m128)]; //!< High and low nibble mask in shufti.
    u8 bucket_select_mask[sizeof(m128)]; //!< Mask for bucket assigning.
    u8 data_select_mask[sizeof(m128)]; //!< Shuffle mask for data ordering.
    u32 hi_bits_mask; //!< High-bits used in multi-path validation.
    u32 lo_bits_mask; //!< Low-bits used in multi-path validation.
    u32 neg_mask; //!< 64 bits negation mask.
    s32 base_offset; //!< Relative offset of the first byte.
    s32 last_start; //!< The latest start offset among 8 paths.
    u32 fail_jump; //!< Jump forward this many bytes on failure.
};

struct ROSE_STRUCT_CHECK_MULTIPATH_SHUFTI_32x8 {
    u8 code; //!< From enum RoseInstructionCode.
    u8 hi_mask[sizeof(m128)]; //!< High nibble mask in shufti.
    u8 lo_mask[sizeof(m128)]; //!< Low nibble mask in shufti.
    u8 bucket_select_mask[sizeof(m256)]; //!< Mask for bucket assigning.
    u8 data_select_mask[sizeof(m256)]; //!< Shuffle mask for data ordering.
    u32 hi_bits_mask; //!< High-bits used in multi-path validation.
    u32 lo_bits_mask; //!< Low-bits used in multi-path validation.
    u32 neg_mask; //!< 64 bits negation mask.
    s32 base_offset; //!< Relative offset of the first byte.
    s32 last_start; //!< The latest start offset among 8 paths.
    u32 fail_jump; //!< Jump forward this many bytes on failure.
};

struct ROSE_STRUCT_CHECK_MULTIPATH_SHUFTI_32x16 {
    u8 code; //!< From enum RoseInstructionCode.
    u8 hi_mask[sizeof(m256)]; //!< High nibble mask in shufti.
    u8 lo_mask[sizeof(m256)]; //!< Low nibble mask in shufti.
    u8 bucket_select_mask_hi[sizeof(m256)]; //!< Mask for bucket assigning.
    u8 bucket_select_mask_lo[sizeof(m256)]; //!< Mask for bucket assigning.
    u8 data_select_mask[sizeof(m256)]; //!< Shuffle mask for data ordering.
    u32 hi_bits_mask; //!< High-bits used in multi-path validation.
    u32 lo_bits_mask; //!< Low-bits used in multi-path validation.
    u32 neg_mask; //!< 64 bits negation mask.
    s32 base_offset; //!< Relative offset of the first byte.
    s32 last_start; //!< The latest start offset among 8 paths.
    u32 fail_jump; //!< Jump forward this many bytes on failure.
};

struct ROSE_STRUCT_CHECK_MULTIPATH_SHUFTI_64 {
    u8 code; //!< From enum RoseInstructionCode.
    u8 hi_mask[sizeof(m128)]; //!< High nibble mask in shufti.
    u8 lo_mask[sizeof(m128)]; //!< Low nibble mask in shufti.
    u8 bucket_select_mask[2 * sizeof(m256)]; //!< Mask for bucket assigning.
    u8 data_select_mask[2 * sizeof(m256)]; //!< Shuffle mask for data ordering.
    u64a hi_bits_mask; //!< High-bits used in multi-path validation.
    u64a lo_bits_mask; //!< Low-bits used in multi-path validation.
    u64a neg_mask; //!< 64 bits negation mask.
    s32 base_offset; //!< Relative offset of the first byte.
    s32 last_start; //!< The latest start offset among 8 paths.
    u32 fail_jump; //!< Jump forward this many bytes on failure.
};

struct ROSE_STRUCT_INCLUDED_JUMP {
    u8 code; //!< From enum RoseInstructionCode.
    u8 squash; //!< FDR confirm squash mask for included literal.
    u32 child_offset; //!< Program offset of included literal.
};

struct ROSE_STRUCT_SET_LOGICAL {
    u8 code; //!< From enum RoseInstructionCode.
    u32 lkey; //!< Logical key to set.
    s32 offset_adjust; //!< offsetAdjust from struct Report triggers the flush.
};

struct ROSE_STRUCT_SET_COMBINATION {
    u8 code; //!< From enum RoseInstructionCode.
    u32 ckey; //!< Combination key to set.
};

struct ROSE_STRUCT_FLUSH_COMBINATION {
    u8 code; //!< From enum RoseInstructionCode.
};

struct ROSE_STRUCT_SET_EXHAUST {
    u8 code; //!< From enum RoseInstructionCode.
    u32 ekey; //!< Exhaustion key.
};
#endif // ROSE_ROSE_PROGRAM_H
