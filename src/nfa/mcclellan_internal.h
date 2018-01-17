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

#ifndef MCCLELLAN_INTERNAL_H
#define MCCLELLAN_INTERNAL_H

#include "nfa_internal.h"

#ifdef __cplusplus
extern "C"
{
#endif

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
    u32 accel_offset; /* relative to start of struct mcclellan; 0 if no accel */
};

#define MCCLELLAN_FLAG_SINGLE 1  /**< we raise only single accept id */

struct mcclellan {
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
                      *  relative to the start of the nfa structure */
    u16 accel_limit_8; /**< 8 bit, lowest accelerable state */
    u16 accept_limit_8; /**< 8 bit, lowest accept state */
    u16 sherman_limit; /**< lowest sherman state */
    u8  alphaShift;
    u8  flags;
    u8  has_accel; /**< 1 iff there are any accel plans */
    u8  remap[256]; /**< remaps characters to a smaller alphabet */
    ReportID arb_report; /**< one of the accepts that this dfa may raise */
    u32 accel_offset; /**< offset of accel structures from start of McClellan */
    u32 haig_offset; /**< reserved for use by Haig, relative to start of NFA */
};

static really_inline
const char *findShermanState(UNUSED const struct mcclellan *m,
                             const char *sherman_base_offset, u32 sherman_base,
                             u32 s) {
    const char *rv
        = sherman_base_offset + SHERMAN_FIXED_SIZE * (s - sherman_base);
    assert(rv < (const char *)m + m->length - sizeof(struct NFA));
    UNUSED u8 type = *(const u8 *)(rv + SHERMAN_TYPE_OFFSET);
    assert(type == SHERMAN_STATE);
    return rv;
}

static really_inline
char *findMutableShermanState(char *sherman_base_offset, u16 sherman_base,
                              u32 s) {
    return sherman_base_offset + SHERMAN_FIXED_SIZE * (s - sherman_base);
}

#ifdef __cplusplus
}
#endif

#endif
