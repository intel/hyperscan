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

#ifndef GOUGH_INTERNAL_H
#define GOUGH_INTERNAL_H

#include "accel.h"
#include "mcclellan_internal.h"
#include "ue2common.h"

#define INVALID_SLOT (~0U)

#define GOUGH_INS_END 0
#define GOUGH_INS_MOV 1
#define GOUGH_INS_NEW 2
#define GOUGH_INS_MIN 3
/* todo: add instructions targeting acc reg? */

struct gough_ins {
    u32 op; /* u32 to avoid padding */
    u32 dest;
    u32 src; /* for GOUGH_INS_NEW, this specifies the adjustment to apply to the
              * current offset */
};

/*
 * HAPPY FUN ASCII ART TIME
 *
 * ----
 * |  | struct NFA
 * ----
 * ~~~~ normal(ish) mcclellan engine
 * ~~~~
 * ~~~~
 * ~~~~
 * ~~~~
 * ~~~~
 * ~~~~
 * ~~~~
 * ---- = m->haig_offset
 * |  | } struct gough_info
 * ----
 * |  | }
 * |  | } edge prog table -> provides the offset of the start of the program
 * |  | }                    to run when the edge is taken. 0 indicates no
 * |  | }                    work to do
 * ---- = h->top_prog_offset
 * |  | }
 * |  | } top prog table  -> provides the offset of the start of the program
 * |  | }                    to run when a top is taken from this state. 0
 * |  | }                    indicates nothing to do
 * ---- = h->prog_base_offset
 * |  | }
 * |  | } programs to run
 * |  | }
 * |  | }
 * ----
 */

struct gough_info {
    u32 top_prog_offset; /**< offset to the base of the top prog table */
    u32 prog_base_offset; /**< not used at runtime */
    u32 stream_som_loc_count; /**< number of som locs in the stream state */
    u8 stream_som_loc_width;  /**< number of bytes per som loc */
};

static really_inline
const struct gough_info *get_gough(const struct mcclellan *m) {
    assert(m->haig_offset);
    const char *n = (const char *)m - sizeof(struct NFA);
    return (const struct gough_info *)(n + m->haig_offset);
}

static really_inline
const u32 *get_gough_top_offsets(const struct mcclellan *m) {
    const struct gough_info *g = get_gough(m);
    if (!g->top_prog_offset) {
        return NULL;
    }
    const char *n = (const char *)m - sizeof(struct NFA);
    return (const u32 *)(n + g->top_prog_offset);
}

/* Gough state representation in scratch.
 *
 * During execution, gough tracks a number of variables containing potential
 * starts of match. These are all stored in a large array of u64a slots.
 */
struct gough_som_info {
    u64a slots[1]; /* 'flexible' member array */
};

struct gough_report {
    ReportID r;
    u32 som; /* som slot to report */
};

struct gough_report_list {
    u32 count;
    struct gough_report report[];
};

struct gough_accel {
    union AccelAux accel;
    u8 margin_dist;
    u32 prog_offset;
};

#endif
