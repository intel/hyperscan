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

#ifndef MCCLELLANCOMPILE_ACCEL_H
#define MCCLELLANCOMPILE_ACCEL_H

#include "mcclellancompile.h"

#include <map>

namespace ue2 {

struct Grey;

#define ACCEL_DFA_MAX_OFFSET_DEPTH 4

/** Maximum tolerated number of escape character from an accel state.
 * This is larger than nfa, as we don't have a budget and the nfa cheats on stop
 * characters for sets of states */
#define ACCEL_DFA_MAX_STOP_CHAR 160

/** Maximum tolerated number of escape character from a sds accel state. Larger
 * than normal states as accelerating sds is important. Matches NFA value */
#define ACCEL_DFA_MAX_FLOATING_STOP_CHAR 192

std::map<dstate_id_t, AccelScheme> populateAccelerationInfo(const raw_dfa &rdfa,
                                                   const dfa_build_strat &strat,
                                                   const Grey &grey);

AccelScheme find_mcclellan_escape_info(const raw_dfa &rdfa,
                                       dstate_id_t this_idx,
                                       u32 max_allowed_accel_offset);

}

#endif
