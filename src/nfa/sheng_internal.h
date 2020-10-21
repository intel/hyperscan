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

#ifndef SHENG_INTERNAL_H_
#define SHENG_INTERNAL_H_

#include "ue2common.h"
#include "util/simd_types.h"

#define SHENG_STATE_ACCEPT 0x10
#define SHENG_STATE_DEAD 0x20
#define SHENG_STATE_ACCEL 0x40
#define SHENG_STATE_MASK 0xF
#define SHENG_STATE_FLAG_MASK 0x70

#define SHENG32_STATE_ACCEPT 0x20
#define SHENG32_STATE_DEAD 0x40
#define SHENG32_STATE_ACCEL 0x80
#define SHENG32_STATE_MASK 0x1F
#define SHENG32_STATE_FLAG_MASK 0xE0

#define SHENG64_STATE_ACCEPT 0x40
#define SHENG64_STATE_DEAD 0x80
#define SHENG64_STATE_MASK 0x3F
#define SHENG64_STATE_FLAG_MASK 0xC0

#define SHENG_FLAG_SINGLE_REPORT 0x1
#define SHENG_FLAG_CAN_DIE 0x2
#define SHENG_FLAG_HAS_ACCEL 0x4

struct report_list {
    u32 count;
    ReportID report[];
};

struct sstate_aux {
    u32 accept;
    u32 accept_eod;
    u32 accel;
    u32 top;
};

struct sheng {
    m128 shuffle_masks[256];
    u32 length;
    u32 aux_offset;
    u32 report_offset;
    u32 accel_offset;
    u8 n_states;
    u8 anchored;
    u8 floating;
    u8 flags;
    ReportID report;
};

struct sheng32 {
    m512 succ_masks[256];
    u32 length;
    u32 aux_offset;
    u32 report_offset;
    u32 accel_offset;
    u8 n_states;
    u8 anchored;
    u8 floating;
    u8 flags;
    ReportID report;
};

struct sheng64 {
    m512 succ_masks[256];
    u32 length;
    u32 aux_offset;
    u32 report_offset;
    u32 accel_offset;
    u8 n_states;
    u8 anchored;
    u8 floating;
    u8 flags;
    ReportID report;
};

#endif /* SHENG_INTERNAL_H_ */
