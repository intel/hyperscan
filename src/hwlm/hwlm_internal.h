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
 * \brief Hamster Wheel Literal Matcher: data structures.
 */

#ifndef HWLM_INTERNAL_H
#define HWLM_INTERNAL_H

#include "hwlm.h"
#include "ue2common.h"
#include "nfa/accel.h"

/** \brief Underlying engine is FDR. */
#define HWLM_ENGINE_FDR     12

/** \brief Underlying engine is Noodle. */
#define HWLM_ENGINE_NOOD    16

/** \brief Main Hamster Wheel Literal Matcher header. Followed by
 * engine-specific structure. */
struct HWLM {
    u8 type; /**< HWLM_ENGINE_NOOD or HWLM_ENGINE_FDR */
    hwlm_group_t accel1_groups; /**< accelerable groups. */
    union AccelAux accel1; /**< used if group mask is subset of accel1_groups */
    union AccelAux accel0; /**< fallback accel scheme */
};

/** \brief Fetch a const pointer to the underlying engine. */
#define HWLM_C_DATA(p) ((const void *)((const char *)(p)                  \
                                       + ROUNDUP_CL(sizeof(struct HWLM))))

/** \brief Fetch a pointer to the underlying engine. */
#define HWLM_DATA(p) ((void *)((char *)(p) + ROUNDUP_CL(sizeof(struct HWLM))))

#endif
