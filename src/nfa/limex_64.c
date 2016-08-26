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
 * \brief LimEx NFA: 128-bit SIMD runtime implementations.
 */

/* Limex64 is unusual on as on 32 bit platforms, at runtime it uses an m128 for
 * state calculations.
 */

//#define DEBUG_INPUT
//#define DEBUG_EXCEPTIONS

#include "limex.h"

#include "accel.h"
#include "limex_internal.h"
#include "nfa_internal.h"
#include "ue2common.h"
#include "util/bitutils.h"
#include "util/simd_utils.h"

// Common code
#define STATE_ON_STACK
#define ESTATE_ON_STACK

#include "limex_runtime.h"

#define SIZE          64
#define ENG_STATE_T   u64a

#ifdef ARCH_64_BIT
#define STATE_T       u64a
#define LOAD_FROM_ENG load_u64a
#else
#define STATE_T       m128
#define LOAD_FROM_ENG load_m128_from_u64a
#endif

#include "limex_exceptional.h"

#include "limex_state_impl.h"

#define INLINE_ATTR really_inline
#include "limex_common_impl.h"

#include "limex_runtime_impl.h"
