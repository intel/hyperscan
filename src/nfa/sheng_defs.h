/*
 * Copyright (c) 2016, Intel Corporation
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

#ifndef SHENG_DEFS_H
#define SHENG_DEFS_H

/*
 * Utility functions used by various versions of Sheng engine
 */
static really_inline
u8 isDeadState(const u8 a) {
    return a & SHENG_STATE_DEAD;
}

static really_inline
u8 isAcceptState(const u8 a) {
    return a & SHENG_STATE_ACCEPT;
}

static really_inline
u8 isAccelState(const u8 a) {
    return a & SHENG_STATE_ACCEL;
}

static really_inline
u8 hasInterestingStates(const u8 a, const u8 b, const u8 c, const u8 d) {
    return (a | b | c | d) & (SHENG_STATE_FLAG_MASK);
}

/* these functions should be optimized out, used by NO_MATCHES mode */
static really_inline
u8 dummyFunc4(UNUSED const u8 a, UNUSED const u8 b, UNUSED const u8 c,
              UNUSED const u8 d) {
    return 0;
}

static really_inline
u8 dummyFunc(UNUSED const u8 a) {
    return 0;
}

/*
 * Sheng function definitions for single byte loops
 */
/* callback output, can die */
#define SHENG_IMPL sheng_cod
#define DEAD_FUNC isDeadState
#define ACCEPT_FUNC isAcceptState
#define STOP_AT_MATCH 0
#include "sheng_impl.h"
#undef SHENG_IMPL
#undef DEAD_FUNC
#undef ACCEPT_FUNC
#undef STOP_AT_MATCH

/* callback output, can't die */
#define SHENG_IMPL sheng_co
#define DEAD_FUNC dummyFunc
#define ACCEPT_FUNC isAcceptState
#define STOP_AT_MATCH 0
#include "sheng_impl.h"
#undef SHENG_IMPL
#undef DEAD_FUNC
#undef ACCEPT_FUNC
#undef STOP_AT_MATCH

/* stop at match, can die */
#define SHENG_IMPL sheng_samd
#define DEAD_FUNC isDeadState
#define ACCEPT_FUNC isAcceptState
#define STOP_AT_MATCH 1
#include "sheng_impl.h"
#undef SHENG_IMPL
#undef DEAD_FUNC
#undef ACCEPT_FUNC
#undef STOP_AT_MATCH

/* stop at match, can't die */
#define SHENG_IMPL sheng_sam
#define DEAD_FUNC dummyFunc
#define ACCEPT_FUNC isAcceptState
#define STOP_AT_MATCH 1
#include "sheng_impl.h"
#undef SHENG_IMPL
#undef DEAD_FUNC
#undef ACCEPT_FUNC
#undef STOP_AT_MATCH

/* no match, can die */
#define SHENG_IMPL sheng_nmd
#define DEAD_FUNC isDeadState
#define ACCEPT_FUNC dummyFunc
#define STOP_AT_MATCH 0
#include "sheng_impl.h"
#undef SHENG_IMPL
#undef DEAD_FUNC
#undef ACCEPT_FUNC
#undef STOP_AT_MATCH

/* no match, can't die */
#define SHENG_IMPL sheng_nm
#define DEAD_FUNC dummyFunc
#define ACCEPT_FUNC dummyFunc
#define STOP_AT_MATCH 0
#include "sheng_impl.h"
#undef SHENG_IMPL
#undef DEAD_FUNC
#undef ACCEPT_FUNC
#undef STOP_AT_MATCH

/*
 * Sheng function definitions for 4-byte loops
 */
/* callback output, can die, accelerated */
#define SHENG_IMPL sheng4_coda
#define INTERESTING_FUNC hasInterestingStates
#define INNER_DEAD_FUNC isDeadState
#define OUTER_DEAD_FUNC dummyFunc
#define INNER_ACCEL_FUNC isAccelState
#define OUTER_ACCEL_FUNC dummyFunc
#define ACCEPT_FUNC isAcceptState
#define STOP_AT_MATCH 0
#include "sheng_impl4.h"
#undef SHENG_IMPL
#undef INTERESTING_FUNC
#undef INNER_DEAD_FUNC
#undef OUTER_DEAD_FUNC
#undef INNER_ACCEL_FUNC
#undef OUTER_ACCEL_FUNC
#undef ACCEPT_FUNC
#undef STOP_AT_MATCH

/* callback output, can die, not accelerated */
#define SHENG_IMPL sheng4_cod
#define INTERESTING_FUNC hasInterestingStates
#define INNER_DEAD_FUNC isDeadState
#define OUTER_DEAD_FUNC dummyFunc
#define INNER_ACCEL_FUNC dummyFunc
#define OUTER_ACCEL_FUNC dummyFunc
#define ACCEPT_FUNC isAcceptState
#define STOP_AT_MATCH 0
#include "sheng_impl4.h"
#undef SHENG_IMPL
#undef INTERESTING_FUNC
#undef INNER_DEAD_FUNC
#undef OUTER_DEAD_FUNC
#undef INNER_ACCEL_FUNC
#undef OUTER_ACCEL_FUNC
#undef ACCEPT_FUNC
#undef STOP_AT_MATCH

/* callback output, can't die, accelerated */
#define SHENG_IMPL sheng4_coa
#define INTERESTING_FUNC hasInterestingStates
#define INNER_DEAD_FUNC dummyFunc
#define OUTER_DEAD_FUNC dummyFunc
#define INNER_ACCEL_FUNC isAccelState
#define OUTER_ACCEL_FUNC dummyFunc
#define ACCEPT_FUNC isAcceptState
#define STOP_AT_MATCH 0
#include "sheng_impl4.h"
#undef SHENG_IMPL
#undef INTERESTING_FUNC
#undef INNER_DEAD_FUNC
#undef OUTER_DEAD_FUNC
#undef INNER_ACCEL_FUNC
#undef OUTER_ACCEL_FUNC
#undef ACCEPT_FUNC
#undef STOP_AT_MATCH

/* callback output, can't die, not accelerated */
#define SHENG_IMPL sheng4_co
#define INTERESTING_FUNC hasInterestingStates
#define INNER_DEAD_FUNC dummyFunc
#define OUTER_DEAD_FUNC dummyFunc
#define INNER_ACCEL_FUNC dummyFunc
#define OUTER_ACCEL_FUNC dummyFunc
#define ACCEPT_FUNC isAcceptState
#define STOP_AT_MATCH 0
#include "sheng_impl4.h"
#undef SHENG_IMPL
#undef INTERESTING_FUNC
#undef INNER_DEAD_FUNC
#undef OUTER_DEAD_FUNC
#undef INNER_ACCEL_FUNC
#undef OUTER_ACCEL_FUNC
#undef ACCEPT_FUNC
#undef STOP_AT_MATCH

/* stop at match, can die, accelerated */
#define SHENG_IMPL sheng4_samda
#define INTERESTING_FUNC hasInterestingStates
#define INNER_DEAD_FUNC isDeadState
#define OUTER_DEAD_FUNC dummyFunc
#define INNER_ACCEL_FUNC isAccelState
#define OUTER_ACCEL_FUNC dummyFunc
#define ACCEPT_FUNC isAcceptState
#define STOP_AT_MATCH 1
#include "sheng_impl4.h"
#undef SHENG_IMPL
#undef INTERESTING_FUNC
#undef INNER_DEAD_FUNC
#undef OUTER_DEAD_FUNC
#undef INNER_ACCEL_FUNC
#undef OUTER_ACCEL_FUNC
#undef ACCEPT_FUNC
#undef STOP_AT_MATCH

/* stop at match, can die, not accelerated */
#define SHENG_IMPL sheng4_samd
#define INTERESTING_FUNC hasInterestingStates
#define INNER_DEAD_FUNC isDeadState
#define OUTER_DEAD_FUNC dummyFunc
#define INNER_ACCEL_FUNC dummyFunc
#define OUTER_ACCEL_FUNC dummyFunc
#define ACCEPT_FUNC isAcceptState
#define STOP_AT_MATCH 1
#include "sheng_impl4.h"
#undef SHENG_IMPL
#undef INTERESTING_FUNC
#undef INNER_DEAD_FUNC
#undef OUTER_DEAD_FUNC
#undef INNER_ACCEL_FUNC
#undef OUTER_ACCEL_FUNC
#undef ACCEPT_FUNC
#undef STOP_AT_MATCH

/* stop at match, can't die, accelerated */
#define SHENG_IMPL sheng4_sama
#define INTERESTING_FUNC hasInterestingStates
#define INNER_DEAD_FUNC dummyFunc
#define OUTER_DEAD_FUNC dummyFunc
#define INNER_ACCEL_FUNC isAccelState
#define OUTER_ACCEL_FUNC dummyFunc
#define ACCEPT_FUNC isAcceptState
#define STOP_AT_MATCH 1
#include "sheng_impl4.h"
#undef SHENG_IMPL
#undef INTERESTING_FUNC
#undef INNER_DEAD_FUNC
#undef OUTER_DEAD_FUNC
#undef INNER_ACCEL_FUNC
#undef OUTER_ACCEL_FUNC
#undef ACCEPT_FUNC
#undef STOP_AT_MATCH

/* stop at match, can't die, not accelerated */
#define SHENG_IMPL sheng4_sam
#define INTERESTING_FUNC hasInterestingStates
#define INNER_DEAD_FUNC dummyFunc
#define OUTER_DEAD_FUNC dummyFunc
#define INNER_ACCEL_FUNC dummyFunc
#define OUTER_ACCEL_FUNC dummyFunc
#define ACCEPT_FUNC isAcceptState
#define STOP_AT_MATCH 1
#include "sheng_impl4.h"
#undef SHENG_IMPL
#undef INTERESTING_FUNC
#undef INNER_DEAD_FUNC
#undef OUTER_DEAD_FUNC
#undef INNER_ACCEL_FUNC
#undef OUTER_ACCEL_FUNC
#undef ACCEPT_FUNC
#undef STOP_AT_MATCH

/* no-match have interesting func as dummy, and die/accel checks are outer */

/* no match, can die, accelerated */
#define SHENG_IMPL sheng4_nmda
#define INTERESTING_FUNC dummyFunc4
#define INNER_DEAD_FUNC dummyFunc
#define OUTER_DEAD_FUNC isDeadState
#define INNER_ACCEL_FUNC dummyFunc
#define OUTER_ACCEL_FUNC isAccelState
#define ACCEPT_FUNC dummyFunc
#define STOP_AT_MATCH 0
#include "sheng_impl4.h"
#undef SHENG_IMPL
#undef INTERESTING_FUNC
#undef INNER_DEAD_FUNC
#undef OUTER_DEAD_FUNC
#undef INNER_ACCEL_FUNC
#undef OUTER_ACCEL_FUNC
#undef ACCEPT_FUNC
#undef STOP_AT_MATCH

/* no match, can die, not accelerated */
#define SHENG_IMPL sheng4_nmd
#define INTERESTING_FUNC dummyFunc4
#define INNER_DEAD_FUNC dummyFunc
#define OUTER_DEAD_FUNC isDeadState
#define INNER_ACCEL_FUNC dummyFunc
#define OUTER_ACCEL_FUNC dummyFunc
#define ACCEPT_FUNC dummyFunc
#define STOP_AT_MATCH 0
#include "sheng_impl4.h"
#undef SHENG_IMPL
#undef INTERESTING_FUNC
#undef INNER_DEAD_FUNC
#undef OUTER_DEAD_FUNC
#undef INNER_ACCEL_FUNC
#undef OUTER_ACCEL_FUNC
#undef ACCEPT_FUNC
#undef STOP_AT_MATCH

/* there is no performance benefit in accelerating a no-match case that can't
 * die */

/* no match, can't die */
#define SHENG_IMPL sheng4_nm
#define INTERESTING_FUNC dummyFunc4
#define INNER_DEAD_FUNC dummyFunc
#define OUTER_DEAD_FUNC dummyFunc
#define INNER_ACCEL_FUNC dummyFunc
#define OUTER_ACCEL_FUNC dummyFunc
#define ACCEPT_FUNC dummyFunc
#define STOP_AT_MATCH 0
#include "sheng_impl4.h"
#undef SHENG_IMPL
#undef INTERESTING_FUNC
#undef INNER_DEAD_FUNC
#undef OUTER_DEAD_FUNC
#undef INNER_ACCEL_FUNC
#undef OUTER_ACCEL_FUNC
#undef ACCEPT_FUNC
#undef STOP_AT_MATCH

#endif // SHENG_DEFS_H
