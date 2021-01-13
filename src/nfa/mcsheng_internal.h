/*
 * Copyright (c) 2016-2020, Intel Corporation
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

#ifndef MCSHENG_INTERNAL_H
#define MCSHENG_INTERNAL_H

#include "nfa_internal.h"
#include "ue2common.h"
#include "util/simd_types.h"

#define ACCEPT_FLAG 0x8000
#define ACCEL_FLAG  0x4000
#define STATE_MASK  0x3fff

#define SHERMAN_STATE 1

#define SHERMAN_TYPE_OFFSET            0
#define SHERMAN_FIXED_SIZE            32

#define SHERMAN_LEN_OFFSET             1
#define SHERMAN_DADDY_OFFSET           2
#define SHERMAN_CHARS_OFFSET           4
#define SHERMAN_STATES_OFFSET(sso_len) (4 + (sso_len))

struct report_list {
    u32 count;
    ReportID report[];
};

struct mstate_aux {
    u32 accept;
    u32 accept_eod;
    u16 top;
    u32 accel_offset; /* relative to start of struct mcsheng; 0 if no accel */
};

#define MCSHENG_FLAG_SINGLE 1  /**< we raise only single accept id */

struct mcsheng {
    u16 state_count; /**< total number of states */
    u32 length; /**< length of dfa in bytes */
    u16 start_anchored; /**< anchored start state */
    u16 start_floating; /**< floating start state */
    u32 aux_offset; /**< offset of the aux structures relative to the start of
                     *  the nfa structure */
    u32 sherman_offset; /**< offset of array of sherman state offsets the
                         * state_info structures relative to the start of the
                         * nfa structure */
    u32 sherman_end; /**< offset of the end of the state_info structures
                      * relative to the start of the nfa structure */
    u16 sheng_end; /**< first non-sheng state */
    u16 sheng_accel_limit; /**< first sheng accel state. state given in terms of
                            * internal sheng ids */
    u16 accel_limit_8; /**< 8 bit, lowest accelerable state */
    u16 accept_limit_8; /**< 8 bit, lowest accept state */
    u16 sherman_limit; /**< lowest sherman state */
    u8  alphaShift;
    u8  flags;
    u8  has_accel; /**< 1 iff there are any accel plans */
    u8  remap[256]; /**< remaps characters to a smaller alphabet */
    ReportID arb_report; /**< one of the accepts that this dfa may raise */
    u32 accel_offset; /**< offset of accel structures from start of McClellan */
    m128 sheng_masks[N_CHARS];
};

/* pext masks for the runtime to access appropriately copies of bytes 1..7
 * representing the data from a u64a. */
extern const u64a mcsheng_pext_mask[8];

struct mcsheng64 {
    u16 state_count; /**< total number of states */
    u32 length; /**< length of dfa in bytes */
    u16 start_anchored; /**< anchored start state */
    u16 start_floating; /**< floating start state */
    u32 aux_offset; /**< offset of the aux structures relative to the start of
                     *  the nfa structure */
    u32 sherman_offset; /**< offset of array of sherman state offsets the
                         * state_info structures relative to the start of the
                         * nfa structure */
    u32 sherman_end; /**< offset of the end of the state_info structures
                      * relative to the start of the nfa structure */
    u16 sheng_end; /**< first non-sheng state */
    u16 sheng_accel_limit; /**< first sheng accel state. state given in terms of
                            * internal sheng ids */
    u16 accel_limit_8; /**< 8 bit, lowest accelerable state */
    u16 accept_limit_8; /**< 8 bit, lowest accept state */
    u16 sherman_limit; /**< lowest sherman state */
    u8  alphaShift;
    u8  flags;
    u8  has_accel; /**< 1 iff there are any accel plans */
    u8  remap[256]; /**< remaps characters to a smaller alphabet */
    ReportID arb_report; /**< one of the accepts that this dfa may raise */
    u32 accel_offset; /**< offset of accel structures from start of McClellan */
    m512 sheng_succ_masks[N_CHARS];
};

extern const u64a mcsheng64_pext_mask[8];

#endif
