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
 * \brief Large Bounded Repeat (LBR): data structures.
 */

#ifndef LBR_INTERNAL_H
#define LBR_INTERNAL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "repeat_internal.h"

/** \brief Common LBR header. */
struct lbr_common {
    u32 repeatInfoOffset;   //!< offset of RepeatInfo structure relative
                            //   to the start of lbr_common
    ReportID report;        //!< report to raise on match
};

struct lbr_dot {
    struct lbr_common common;
};

struct lbr_verm {
    struct lbr_common common;
    char c; //!< escape char
};

struct lbr_shuf {
    struct lbr_common common;
    m128 mask_lo; //!< shufti lo mask for escape chars
    m128 mask_hi; //!< shufti hi mask for escape chars
};

struct lbr_truf {
    struct lbr_common common;
    m128 mask1;
    m128 mask2;
};

/** \brief Uncompressed ("full") state structure used by the LBR. This is
 * stored in scratch, not in stream state. */
struct lbr_state {
    u64a lastEscape; //!< \brief offset of last escape seen.
    union RepeatControl ctrl; //!< \brief repeat control block. */
};

#ifdef __cplusplus
}
#endif

#endif // LBR_INTERNAL_H
