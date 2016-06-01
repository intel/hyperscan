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
 * \brief Build code for Haig SOM DFA.
 */

#ifndef NG_HAIG_H
#define NG_HAIG_H

#include "ue2common.h"
#include "som/som.h"

#include <memory>
#include <vector>

namespace ue2 {

class CharReach;
class NGHolder;
struct Grey;
struct raw_som_dfa;

#define HAIG_FINAL_DFA_STATE_LIMIT 16383
#define HAIG_HARD_DFA_STATE_LIMIT 8192

/* unordered_som_triggers being true indicates that a live haig may be subjected
 * to later tops arriving with earlier soms (without the haig going dead in
 * between)
 */

std::unique_ptr<raw_som_dfa>
attemptToBuildHaig(const NGHolder &g, som_type som, u32 somPrecision,
                   const std::vector<std::vector<CharReach>> &triggers,
                   const Grey &grey, bool unordered_som_triggers = false);

std::unique_ptr<raw_som_dfa>
attemptToMergeHaig(const std::vector<const raw_som_dfa *> &dfas,
                   u32 limit = HAIG_HARD_DFA_STATE_LIMIT);

} // namespace ue2

#endif
