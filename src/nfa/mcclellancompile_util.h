/*
 * Copyright (c) 2015-2017, Intel Corporation
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

#ifndef MCCLELLAN_COMPILE_UTIL_H
#define MCCLELLAN_COMPILE_UTIL_H

#include "rdfa.h"
#include "ue2common.h"

#include <set>

namespace ue2 {

u32 remove_leading_dots(raw_dfa &raw);

/**
 * \brief Clear reports on any states that are deeper than \a max_offset from
 * start of stream.
 *
 * Returns false if no changes are made to the DFA.
 */
bool clear_deeper_reports(raw_dfa &raw, u32 max_offset);

std::set<ReportID> all_reports(const raw_dfa &rdfa);
bool has_eod_accepts(const raw_dfa &rdfa);
bool has_non_eod_accepts(const raw_dfa &rdfa);

/** \brief Compute a simple hash of this raw_dfa. Does not include report
 * information. */
size_t hash_dfa_no_reports(const raw_dfa &rdfa);

/** \brief Compute a simple hash of this raw_dfa, including its reports. */
size_t hash_dfa(const raw_dfa &rdfa);

bool can_die_early(const raw_dfa &raw, u32 age_limit);

/**
 * \brief Returns true if this DFA cannot match, i.e. its start state is
 * DEAD_STATE.
 */
bool is_dead(const raw_dfa &rdfa);


} // namespace ue2

#endif
