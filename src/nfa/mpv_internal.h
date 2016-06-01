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

#ifndef MPV_INTERNAL_H
#define MPV_INTERNAL_H

#include "ue2common.h"

#define MPV_DOT    0
#define MPV_VERM   1
#define MPV_SHUFTI 2
#define MPV_TRUFFLE 3
#define MPV_NVERM  4

struct mpv_puffette {
    u32 repeats;
    char unbounded;

    /**
     * \brief Report is simple-exhaustible.
     *
     * If this is true, we do best-effort suppression of runs of reports, only
     * delivering the first one.
     */
    char simple_exhaust;

    ReportID report;
};

struct mpv_kilopuff {
    u32 counter_offset; /**< offset (in full stream state) to the counter that
                         * this kilopuff refers to */
    u32 count; /**< number of real (non sentinel mpv puffettes) */
    u32 puffette_offset; /**< relative to base of mpv, points past the 1st
                          * sent */
    u64a dead_point;
    u8 auto_restart;
    u8 type; /* MPV_DOT, MPV_VERM, etc */
    union {
        struct {
            char c;
        } verm;
        struct {
            m128 mask_lo;
            m128 mask_hi;
        } shuf;
        struct {
            m128 mask1;
            m128 mask2;
        } truffle;
    } u;
};

struct mpv_counter_info {
    u64a max_counter; /**< maximum value this counter needs to track */
    u32 counter_size; /**< number of bytes to represent the counter in stream
                       * state */
    u32 counter_offset; /**< offset that this counter is stored at in the
                         * full stream state */
    u32 kilo_begin; /**< first kilo to turn on when the counter is started */
    u32 kilo_end; /**< 1 + last kilo to turn on when the counter is started */
};

struct ALIGN_AVX_DIRECTIVE mpv {
    u32 kilo_count; /**< number of kilopuffs following */
    u32 counter_count; /**< number of counters managed by the mpv */
    u32 puffette_count; /**< total number of puffettes under all the kilos */
    u32 pq_offset; /**< offset to the priority queue in the decompressed
                    * state */
    u32 reporter_offset; /**< offset to the reporter mmbit in the decompressed
                          * state */
    u32 report_list_offset; /**< offset to the report list scratch space in the
                             * decompressed state */
    u32 active_offset; /**< offset to the active kp mmbit in the compressed
                        * state */
    u32 top_kilo_begin; /**< first kilo to switch on when top arrives */
    u32 top_kilo_end; /**< one past the last kilo to switch on when top
                       * arrives */
};

struct mpv_decomp_kilo {
    u64a limit;
    const struct mpv_puffette *curr;
};

/* note: size varies on different platforms */
struct mpv_decomp_state {
    u32 pq_size;
    char filled;
    u64a counter_adj; /**< progress not yet written to the real counters */
    struct mpv_decomp_kilo active[];
};

/* ---
 * | | mpv
 * ---
 * | |
 * | | kilo_count * mpv_kilopuffs
 * | |
 * ...
 * | |
 * ---
 * | |
 * | | counter_count * mpv_counter_infos
 * | |
 * ...
 * | |
 * ---
 * | | sentinel mpv_puffette
 * ---
 * | | mpv_puffettes for 1st kilopuff
 * | | (mpv_puffettes are ordered by minimum number of repeats)
 * | |
 * ---
 * | | sentinel mpv_puffette
 * ---
 * | | mpv_puffettes for 2nd kilopuff
 * ...
 * | |
 * ---
 * | | sentinel mpv_puffette
 * ---
 */

/*
 * Stream State
 * [Compressed Counter 0]
 * [Compressed Counter 1]
 * ...
 * [Compressed Counter N]
 * [mmbit of active kilopuffs]
 *
 * Decompressed State
 * [header (limit pq_size)]
 * [
 *     [kilo 1 current reports]
 *      ...
 *     [kilo N current reports]
 * ]
 * [
 *     [Full Counter 0]
 *     [Full Counter 1]
 *     ...
 *     [Full Counter N]
 * ]
 * [pq of kilo changes]
 * [scratch space for current report lists (total number of puffettes)]
 * [mmbit of kilopuffs with active reports]
 */

struct mpv_pq_item {
    u64a trigger_loc;
    u32 kilo;
};

/* returns pointer to first non sentinel mpv_puff */
static really_inline
const struct mpv_puffette *get_puff_array(const struct mpv *m,
                                          const struct mpv_kilopuff *kp) {
    return (const struct mpv_puffette *)((const char *)m + kp->puffette_offset);
}

static really_inline
const struct mpv_counter_info *get_counter_info(const struct mpv *m) {
    return (const struct mpv_counter_info *)((const char *)(m + 1)
                                 + m->kilo_count * sizeof(struct mpv_kilopuff));
}

#define MPV_DEAD_VALUE (~0ULL)
#define INVALID_REPORT (~0U)

#endif
