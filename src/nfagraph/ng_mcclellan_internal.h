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
 * \brief Shared build code for DFAs (McClellan, Haig).
 */

#ifndef NG_MCCLELLAN_INTERNAL_H
#define NG_MCCLELLAN_INTERNAL_H

#include "ue2common.h"
#include "nfa/mcclellancompile.h"
#include "nfagraph/ng_holder.h"
#include "util/charreach.h"
#include "util/graph_range.h"
#include "util/flat_containers.h"

#include <boost/dynamic_bitset.hpp>

#include <map>
#include <vector>

namespace ue2 {

struct raw_dfa;

/** Fills alpha, unalpha and returns alphabet size. */
u16 buildAlphabetFromEquivSets(const std::vector<CharReach> &esets,
                               std::array<u16, ALPHABET_SIZE> &alpha,
                               std::array<u16, ALPHABET_SIZE> &unalpha);

/** \brief Calculates an alphabet remapping based on the symbols which the
 * graph discriminates on. Throws in some special DFA symbols as well. */
void calculateAlphabet(const NGHolder &g, std::array<u16, ALPHABET_SIZE> &alpha,
                       std::array<u16, ALPHABET_SIZE> &unalpha, u16 *alphasize);

void getFullTransitionFromState(const raw_dfa &n, u16 state,
                                u16 *out_table);

/** produce a map of states on which it is valid to receive tops */
void markToppableStarts(const NGHolder &g, const flat_set<NFAVertex> &unused,
                        bool single_trigger,
                        const std::vector<std::vector<CharReach>> &triggers,
                        boost::dynamic_bitset<> *out);

/**
 * \brief Returns a set of start vertices that will not participate in an
 * implementation of this graph. These are either starts with no successors or
 * starts which are redundant with startDs.
 */
flat_set<NFAVertex> getRedundantStarts(const NGHolder &g);

template<typename autom>
void transition_graph(autom &nfa, const std::vector<NFAVertex> &vByStateId,
                      const typename autom::StateSet &in,
                      typename autom::StateSet *next) {
    typedef typename autom::StateSet StateSet;
    const NGHolder &graph = nfa.graph;
    const auto &unused = nfa.unused;
    const auto &alpha = nfa.alpha;
    const StateSet &squash = nfa.squash;
    const std::map<u32, StateSet> &squash_mask = nfa.squash_mask;
    const std::vector<CharReach> &cr_by_index = nfa.cr_by_index;

    for (symbol_t s = 0; s < nfa.alphasize; s++) {
        next[s].reset();
    }

    /* generate top transitions, false -> top = selfloop */
    bool top_allowed = is_triggered(graph);

    StateSet succ = nfa.dead;
    for (size_t i = in.find_first(); i != in.npos; i = in.find_next(i)) {
        NFAVertex u = vByStateId[i];

        for (const auto &v : adjacent_vertices_range(u, graph)) {
            if (contains(unused, v)) {
                continue;
            }
            succ.set(graph[v].index);
        }

        if (top_allowed && !nfa.toppable.test(i)) {
            /* we don't need to generate a top at this location as we are in
             * an nfa state which cannot be on when a trigger arrives. */
            top_allowed = false;
        }
    }

    StateSet active_squash = succ & squash;
    if (active_squash.any()) {
        for (size_t j = active_squash.find_first(); j != active_squash.npos;
             j = active_squash.find_next(j)) {
            succ &= squash_mask.find(j)->second;
        }
    }

    for (size_t j = succ.find_first(); j != succ.npos; j = succ.find_next(j)) {
        const CharReach &cr = cr_by_index[j];
        for (size_t s = cr.find_first(); s != cr.npos; s = cr.find_next(s)) {
            next[s].set(j); /* already alpha'ed */
        }
    }

    next[alpha[TOP]] = in;

    if (top_allowed) {
        /* we don't add in the anchored starts as the only case as the only
         * time it is appropriate is if no characters have been consumed.*/
        next[alpha[TOP]] |= nfa.initDS;

        active_squash = next[alpha[TOP]] & squash;
        if (active_squash.any()) {
            for (size_t j = active_squash.find_first(); j != active_squash.npos;
                 j = active_squash.find_next(j)) {
                next[alpha[TOP]] &= squash_mask.find(j)->second;
            }
        }
    }
}

} // namespace ue2

#endif
