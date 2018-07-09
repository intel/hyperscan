/*
 * Copyright (c) 2015-2018, Intel Corporation
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
 * \brief Stop Alphabet calculation.
 */

#ifndef NG_STOP_H
#define NG_STOP_H

#include "ue2common.h"
#include "som/som.h"

#include <vector>

namespace ue2 {

struct CastleProto;
class CharReach;
class NGHolder;

/** Find the set of characters that are not present in the reachability of
 * graph \p g after a certain depth (currently 8). If a character in this set
 * is encountered, it means that the NFA is either dead or has not progressed
 * more than 8 characters from its start states.
 *
 * This is only used to guide merging heuristics, use
 * findLeftOffsetStopAlphabet for real uses.
 */
CharReach findStopAlphabet(const NGHolder &g, som_type som);

/** Calculate the stop alphabet for each depth from 0 to MAX_STOP_DEPTH. Then
 * build an eight-bit mask per character C, with each bit representing the
 * depth before the location of character C (if encountered) that the NFA would
 * be in a predictable start state. */
std::vector<u8> findLeftOffsetStopAlphabet(const NGHolder &g, som_type som);
std::vector<u8> findLeftOffsetStopAlphabet(const CastleProto &c, som_type som);

} // namespace ue2

#endif
