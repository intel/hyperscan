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

/** \file
 * \brief NFA graph state squashing analysis.
 */
#ifndef NG_SQUASH_H
#define NG_SQUASH_H

#include "ng_holder.h"
#include "som/som.h"
#include "ue2common.h"

#include <unordered_map>
#include <boost/dynamic_bitset.hpp>

namespace ue2 {

class NGHolder;
class ReportManager;

/**
 * Dynamically-sized bitset, as an NFA can have an arbitrary number of states.
 */
using NFAStateSet = boost::dynamic_bitset<>;

/**
 * Populates the squash mask for each vertex (i.e. the set of states to be left
 * on during squashing).
 *
 * The NFAStateSet in the output map is indexed by vertex_index.
 */
std::unordered_map<NFAVertex, NFAStateSet>
findSquashers(const NGHolder &g, som_type som = SOM_NONE);

/** Filters out squash states intended only for use in DFA construction. */
void filterSquashers(const NGHolder &g,
                     std::unordered_map<NFAVertex, NFAStateSet> &squash);

/** Populates squash masks for states that can be switched off by highlander
 * (single match) reporters. */
std::unordered_map<NFAVertex, NFAStateSet>
findHighlanderSquashers(const NGHolder &g, const ReportManager &rm);

} // namespace ue2

#endif // NG_SQUASH_H
